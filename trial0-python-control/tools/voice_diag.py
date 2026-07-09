# -*- coding: utf-8 -*-
"""Voice KWS 诊断工具。

用法：
  1) 注入测试 wav 到 KWS 线程，验证整条链路（不含真实麦克风）：
     python tools/voice_diag.py --inject-wav /path/to/test.wav --keywords-file /path/to/keywords.txt

  2) 监听真实麦克风 5 秒，打印音频电平和 VAD 状态：
     python tools/voice_diag.py --device 6 --duration 5
"""
import argparse
import queue
import sys
import time
from pathlib import Path

import numpy as np
import sounddevice as sd
import soundfile as sf

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from voice import config
from voice.audio import _apply_gain
from voice.kws import SherpaKwsSpotter
from voice.vad import SileroVadGate


def rms_db(samples: np.ndarray) -> float:
    """计算音频块的 RMS 分贝值。"""
    samples = np.asarray(samples, dtype=np.float64)
    if len(samples) == 0:
        return -np.inf
    rms = np.sqrt(np.mean(samples ** 2))
    if rms <= 0:
        return -np.inf
    return 20.0 * np.log10(rms)


def diag_inject_wav(wav_path: str, keywords_file: str, provider: str) -> None:
    """把指定 wav 注入 SherpaKwsSpotter，看是否能识别关键词。"""
    print(f"[Diag] 加载 KWS: provider={provider}, keywords={keywords_file}")
    spotter = SherpaKwsSpotter(
        model_dir=config.SHERPA_MODEL_DIR,
        keywords_file=keywords_file,
        provider=provider,
        device_id=0,
        num_threads=1,
    )

    data, sr = sf.read(wav_path, dtype="float32")
    if data.ndim > 1:
        data = data.mean(axis=1)
    if sr != config.VOICE_SAMPLE_RATE:
        raise RuntimeError(f"{wav_path} 采样率为 {sr} Hz，需要 {config.VOICE_SAMPLE_RATE} Hz")

    vad = SileroVadGate(
        model_path=config.SILERO_VAD_MODEL,
        threshold=config.VOICE_VAD_THRESHOLD,
        hangover_ms=config.VOICE_VAD_HANGOVER_MS,
        sample_rate=config.VOICE_SAMPLE_RATE,
    )

    chunk_samples = 1600  # 0.1 s
    vad_state = False
    detections = []
    t0 = time.time()
    for i in range(0, len(data), chunk_samples):
        chunk = data[i:i + chunk_samples]
        spotter.accept_waveform(chunk)
        new_vad_state = vad.accept_waveform(chunk)
        db = rms_db(chunk)
        print(f"  t={(i / config.VOICE_SAMPLE_RATE):.2f}s  rms={db:6.1f}dB  vad={new_vad_state}")
        if new_vad_state:
            kw = spotter.decode()
            if kw:
                print(f"  >>> 识别到关键词: {kw}")
                detections.append(kw)
        elif vad_state:
            spotter.reset()
        vad_state = new_vad_state

    print(f"[Diag] 完成，耗时 {(time.time() - t0):.2f}s，共识别 {len(detections)} 次: {detections}")


def diag_mic(
    device_id: int,
    duration: float,
    use_vad: bool,
    gain: float,
    auto_gain: bool,
    auto_gain_target_db: float,
    auto_gain_max_db: float,
    auto_gain_min_db: float,
) -> None:
    """监听真实麦克风，打印电平和 VAD 状态。"""
    info = sd.query_devices(device_id, "input")
    print(f"[Diag] 麦克风设备: {info['name']}")
    print(f"[Diag] 监听 {duration} 秒，请对着麦克风说话...")
    if auto_gain:
        print(f"[Diag] 软件自动增益开启: target={auto_gain_target_db:.0f}dB max={auto_gain_max_db:.0f}dB")
    elif gain != 1.0:
        print(f"[Diag] 固定增益: {gain:.1f}x")

    vad = None
    if use_vad:
        vad = SileroVadGate(
            model_path=config.SILERO_VAD_MODEL,
            threshold=config.VOICE_VAD_THRESHOLD,
            hangover_ms=config.VOICE_VAD_HANGOVER_MS,
            sample_rate=config.VOICE_SAMPLE_RATE,
        )

    q: queue.Queue = queue.Queue()

    def callback(indata, frames, _time_info, status):
        if status:
            print(f"[sounddevice status] {status}", file=sys.stderr)
        # sounddevice 提供的是 (frames, channels)，取均值转为单声道 float32
        mono = indata.mean(axis=1).astype(np.float32)
        q.put(mono)

    device_info = sd.query_devices(device_id, "input")
    samplerate = int(device_info["default_samplerate"])
    channels = int(device_info["max_input_channels"])
    blocksize = 1600

    with sd.InputStream(
        device=device_id,
        samplerate=samplerate,
        channels=channels,
        blocksize=blocksize,
        dtype="float32",
        callback=callback,
    ):
        t0 = time.time()
        while time.time() - t0 < duration:
            try:
                block = q.get(timeout=0.1)
            except queue.Empty:
                continue
            # 重采样到 16 kHz（简单线性近似，仅用于诊断）
            if samplerate != config.VOICE_SAMPLE_RATE:
                ratio = config.VOICE_SAMPLE_RATE / samplerate
                n = int(len(block) * ratio)
                block = np.interp(
                    np.linspace(0, len(block), n, endpoint=False),
                    np.arange(len(block)),
                    block,
                ).astype(np.float32)
            raw_db = rms_db(block)
            boosted = _apply_gain(
                block,
                fixed_gain=gain,
                auto_gain=auto_gain,
                target_db=auto_gain_target_db,
                max_db=auto_gain_max_db,
                min_db=auto_gain_min_db,
            )
            post_db = rms_db(boosted)
            vad_state = vad.accept_waveform(boosted) if vad else False
            bar_len = max(0, int((min(post_db, 0.0) + 60) / 2)) if np.isfinite(post_db) else 0
            bar = "#" * bar_len
            print(
                f"  raw={raw_db:6.1f}dB post={post_db:6.1f}dB "
                f"[{bar:<30}] vad={vad_state}"
            )

    print("[Diag] 监听结束")


def main() -> None:
    parser = argparse.ArgumentParser(description="Voice KWS 诊断")
    parser.add_argument("--inject-wav", type=str, default=None, help="注入测试 wav 文件路径")
    parser.add_argument("--keywords-file", type=str, default=config.KEYWORDS_FILE, help="关键词文件路径")
    parser.add_argument("--provider", type=str, default="openvino", choices=["cpu", "openvino"])
    parser.add_argument("--device", type=int, default=config.VOICE_DEVICE_ID, help="麦克风设备号")
    parser.add_argument("--duration", type=float, default=5.0, help="麦克风监听时长（秒）")
    parser.add_argument("--no-vad", action="store_true", help="麦克风诊断时禁用 VAD")
    parser.add_argument("--gain", type=float, default=config.VOICE_GAIN, help="固定线性增益")
    parser.add_argument("--auto-gain", action="store_true", default=config.VOICE_AUTO_GAIN,
                        help="启用软件自动增益（默认开启）")
    parser.add_argument("--no-auto-gain", action="store_true", help="禁用软件自动增益")
    parser.add_argument("--auto-gain-target-db", type=float, default=config.VOICE_AUTO_GAIN_TARGET_DB,
                        help="自动增益目标 RMS（dB）")
    parser.add_argument("--auto-gain-max-db", type=float, default=config.VOICE_AUTO_GAIN_MAX_DB,
                        help="自动增益最大提升（dB）")
    parser.add_argument("--auto-gain-min-db", type=float, default=config.VOICE_AUTO_GAIN_MIN_DB,
                        help="自动增益噪声门限（dB）")
    args = parser.parse_args()

    auto_gain = args.auto_gain and not args.no_auto_gain

    if args.inject_wav:
        diag_inject_wav(args.inject_wav, args.keywords_file, args.provider)
    else:
        diag_mic(
            args.device,
            args.duration,
            use_vad=not args.no_vad,
            gain=args.gain,
            auto_gain=auto_gain,
            auto_gain_target_db=args.auto_gain_target_db,
            auto_gain_max_db=args.auto_gain_max_db,
            auto_gain_min_db=args.auto_gain_min_db,
        )


if __name__ == "__main__":
    main()
