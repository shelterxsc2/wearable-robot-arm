# -*- coding: utf-8 -*-
"""Live microphone → VAD → KWS diagnostic.

This script exposes exactly what happens inside VoiceKwsThread, one line per
audio chunk, so you can see whether the VAD is gating too aggressively and
causing the "only a small slice is decoded" symptom.

Usage:
    python tools/voice_live_kws_diag.py --duration 10 --record /tmp/live.wav
    python tools/voice_live_kws_diag.py --duration 10 --no-vad
"""
import argparse
import queue
import sys
import time
import wave
from pathlib import Path

import numpy as np
import sounddevice as sd

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from voice import config
from voice.audio import _apply_gain
from voice.kws import SherpaKwsSpotter
from voice.vad import SileroVadGate

SHERPA_MODEL_DIR = "/home/time/work/sherpa/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"
SHERPA_KEYWORDS_FILE = str(Path(__file__).resolve().parent.parent / "voice" / "keywords.txt")
SHERPA_VAD_MODEL = "/home/time/work/sherpa/models/silero_vad.onnx"


def rms_db(samples: np.ndarray) -> float:
    samples = np.asarray(samples, dtype=np.float64)
    if len(samples) == 0:
        return -np.inf
    rms = np.sqrt(np.mean(samples ** 2))
    if rms <= 0:
        return -np.inf
    return 20.0 * np.log10(rms)


def main() -> None:
    parser = argparse.ArgumentParser(description="Live mic KWS diagnostic")
    parser.add_argument("--provider", type=str, default="cpu", choices=["cpu", "openvino"])
    parser.add_argument("--device", type=int, default=config.VOICE_DEVICE_ID)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--keywords-file", type=str, default=SHERPA_KEYWORDS_FILE)
    parser.add_argument("--score", type=float, default=None)
    parser.add_argument("--threshold", type=float, default=None)
    parser.add_argument("--trailing-blanks", type=int, default=None)
    parser.add_argument("--vad-threshold", type=float, default=None)
    parser.add_argument("--vad-hangover-ms", type=float, default=None)
    parser.add_argument("--silence-reset-blocks", type=int, default=config.VOICE_SILENCE_RESET_BLOCKS,
                        help="Consecutive silent blocks before resetting KWS")
    parser.add_argument("--no-vad", action="store_true", help="Feed every block to KWS (no VAD gating)")
    parser.add_argument("--gain", type=float, default=config.VOICE_GAIN)
    parser.add_argument("--auto-gain", action="store_true", default=config.VOICE_AUTO_GAIN)
    parser.add_argument("--no-auto-gain", action="store_true")
    parser.add_argument("--auto-gain-target-db", type=float, default=config.VOICE_AUTO_GAIN_TARGET_DB)
    parser.add_argument("--auto-gain-max-db", type=float, default=config.VOICE_AUTO_GAIN_MAX_DB)
    parser.add_argument("--auto-gain-min-db", type=float, default=config.VOICE_AUTO_GAIN_MIN_DB)
    parser.add_argument("--record", type=str, default=None, help="Save KWS input audio to WAV for offline replay")
    parser.add_argument("--chunk-sec", type=float, default=0.1,
                        help="Print one status line every N seconds")
    args = parser.parse_args()

    auto_gain = args.auto_gain and not args.no_auto_gain

    print("=" * 70)
    print("Live mic → VAD → KWS diagnostic")
    print("=" * 70)
    print(f"  provider={args.provider}")
    print(f"  device={args.device}")
    print(f"  kws_score={args.score}")
    print(f"  kws_threshold={args.threshold}")
    print(f"  trailing_blanks={args.trailing_blanks}")
    print(f"  use_vad={not args.no_vad}")
    if not args.no_vad:
        print(f"  vad_threshold={args.vad_threshold}")
        print(f"  vad_hangover_ms={args.vad_hangover_ms}")
    print(f"  auto_gain={auto_gain}")
    print(f"  record={args.record}")
    print("=" * 70)
    print(f"Speak the keywords from {args.keywords_file}")
    print("Columns: t | raw_rms | post_rms | vad | decoded | #blocks | #vad_blocks")
    print("-" * 70)

    kws = SherpaKwsSpotter(
        model_dir=SHERPA_MODEL_DIR,
        keywords_file=args.keywords_file,
        provider=args.provider,
        device_id=0,
        keywords_score=args.score,
        keywords_threshold=args.threshold,
        num_trailing_blanks=args.trailing_blanks,
        num_threads=1,
    )

    vad = None
    if not args.no_vad:
        vad = SileroVadGate(
            model_path=SHERPA_VAD_MODEL,
            threshold=args.vad_threshold if args.vad_threshold is not None else config.VOICE_VAD_THRESHOLD,
            hangover_ms=args.vad_hangover_ms if args.vad_hangover_ms is not None else config.VOICE_VAD_HANGOVER_MS,
            sample_rate=config.VOICE_SAMPLE_RATE,
        )

    dev = sd.query_devices(args.device, "input")
    samplerate = int(dev["default_samplerate"])
    channels = min(int(dev["max_input_channels"]), config.VOICE_CHANNELS)
    ratio = config.VOICE_SAMPLE_RATE / samplerate
    blocksize = config.VOICE_BLOCKSIZE

    audio_q: queue.Queue = queue.Queue()

    def callback(indata, frames, _time_info, status):
        if status:
            print(f"[sounddevice] {status}", file=sys.stderr)
        f32 = indata.astype(np.float32) / 32768.0
        mono = f32.mean(axis=1) if (channels > 1 and f32.ndim > 1) else (f32[:, 0] if f32.ndim > 1 else f32)
        audio_q.put(mono.copy())

    wav_out = None
    if args.record:
        wav_out = wave.open(args.record, "wb")
        wav_out.setnchannels(1)
        wav_out.setsampwidth(2)
        wav_out.setframerate(config.VOICE_SAMPLE_RATE)

    chunk_samples = int(config.VOICE_SAMPLE_RATE * args.chunk_sec)
    buffer = np.zeros(0, dtype=np.float32)
    vad_state = False
    silent_blocks = 0
    total_blocks = 0
    vad_blocks = 0
    decode_blocks = 0
    detections = []

    t_start = time.time()
    report_t0 = t_start

    with sd.InputStream(
        device=args.device,
        channels=channels,
        samplerate=samplerate,
        dtype="int16",
        blocksize=blocksize,
        callback=callback,
    ):
        while time.time() - t_start < args.duration:
            try:
                block = audio_q.get(timeout=0.05)
            except queue.Empty:
                continue

            # Resample to 16 kHz.
            n = int(round(len(block) * ratio))
            if n != len(block):
                x_old = np.linspace(0.0, 1.0, len(block))
                x_new = np.linspace(0.0, 1.0, n)
                block_16k = np.interp(x_new, x_old, block).astype(np.float32)
            else:
                block_16k = block.astype(np.float32)

            raw_db = rms_db(block_16k)
            block_16k = _apply_gain(
                block_16k,
                fixed_gain=args.gain,
                auto_gain=auto_gain,
                target_db=args.auto_gain_target_db,
                max_db=args.auto_gain_max_db,
                min_db=args.auto_gain_min_db,
            )
            post_db = rms_db(block_16k)

            if wav_out is not None:
                pcm = np.clip(block_16k * 32767.0, -32768.0, 32767.0).astype(np.int16)
                wav_out.writeframes(pcm.tobytes())

            kws.accept_waveform(block_16k)
            total_blocks += 1

            # Match VoiceKwsThread: decode every block, only reset after
            # sustained silence.
            kw = kws.decode()
            if kw:
                detections.append(kw)
                print(f"\n*** DETECTED: {kw} at t={time.time() - t_start:.2f}s ***\n")

            if vad is not None:
                prev_state = vad_state
                vad_state = vad.accept_waveform(block_16k)
                if vad_state:
                    vad_blocks += 1
                    decode_blocks += 1
                    silent_blocks = 0
                else:
                    silent_blocks += 1
                    if prev_state and silent_blocks >= args.silence_reset_blocks:
                        kws.reset()
                        silent_blocks = 0
            else:
                decode_blocks += 1

            buffer = np.concatenate([buffer, block_16k])
            if len(buffer) >= chunk_samples:
                segment = buffer[:chunk_samples]
                buffer = buffer[chunk_samples:]
                bar_len = max(0, int((min(post_db, 0.0) + 60) / 2)) if np.isfinite(post_db) else 0
                bar = "#" * bar_len
                now = time.time() - t_start
                elapsed = time.time() - report_t0
                print(
                    f"t={now:5.2f}s "
                    f"raw={raw_db:6.1f}dB post={post_db:6.1f}dB "
                    f"[{bar:<30}] "
                    f"vad={vad_state if vad is not None else '-'} "
                    f"decoded={'Y' if kw else 'N'} "
                    f"blocks={total_blocks} vad_blocks={vad_blocks} "
                    f"({elapsed*1000:.0f}ms)"
                )
                report_t0 = time.time()
                total_blocks = 0
                vad_blocks = 0

    if wav_out is not None:
        wav_out.close()

    print("-" * 70)
    print(f"Finished. Total detections: {len(detections)}: {detections}")
    if args.record:
        print(f"Saved KWS input audio to: {args.record}")
        print(f"  Replay offline with: python tools/voice_diag.py --inject-wav {args.record} --keywords-file {args.keywords_file}")


if __name__ == "__main__":
    main()
