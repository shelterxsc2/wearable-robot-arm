#!/usr/bin/env python3
"""Standalone KWS demo ported from trial0 (no vision or camera)."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import wave

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "third_party" / "python"))
sys.path.insert(0, str(ROOT / "src"))
import numpy as np
from scipy import signal as scipy_signal
import sherpa_onnx
from voice_kws_server import create_spotter, create_vad

MODEL_DIR = ROOT / "models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"
KEYWORDS_FILE = Path(__file__).resolve().parent / "voice/keywords.txt"
VAD_MODEL = ROOT / "models/silero_vad.onnx"


def apply_gain(samples, fixed, automatic, target_db, max_db, min_db):
    if not automatic:
        return np.clip(samples * fixed, -1, 1).astype(np.float32)
    rms = float(np.sqrt(np.mean(samples.astype(np.float64) ** 2)))
    db = 20 * np.log10(max(rms, 1e-12))
    extra = 1.0 if db < min_db else min(10 ** (max_db / 20), 10 ** (target_db / 20) / max(rms, 1e-12))
    return np.clip(samples * fixed * extra, -1, 1).astype(np.float32)


def source_blocks(args):
    if args.wav:
        w = wave.open(args.wav, "rb")
        rate, channels = w.getframerate(), w.getnchannels()
        source = w
        proc = None
    else:
        dev = str(args.device)
        if dev.isdigit(): dev = f"hw:{dev},0"
        cmd = ["arecord", "-q", "-D", dev, "-t", "raw", "-f", "S16_LE",
               "-r", str(args.capture_rate), "-c", str(args.channels)]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        source, rate, channels = proc.stdout, args.capture_rate, args.channels
    try:
        while True:
            raw = (source.readframes(args.blocksize) if proc is None
                   else source.read(args.blocksize * channels * 2))
            if not raw: break
            x = np.frombuffer(raw, dtype="<i2").astype(np.float32).reshape(-1, channels).mean(axis=1) / 32768
            if rate != 16000:
                x = scipy_signal.resample_poly(x, 16000, rate).astype(np.float32)
            yield apply_gain(x, args.gain, args.auto_gain, args.auto_gain_target_db,
                             args.auto_gain_max_db, args.auto_gain_min_db)
    finally:
        if proc:
            proc.terminate()
            try: proc.wait(timeout=2)
            except subprocess.TimeoutExpired: proc.kill()
        else: source.close()


def main():
    p = argparse.ArgumentParser(description="Voice-only keyword spotting demo")
    p.add_argument("--provider", default="cpu", choices=["cpu"], help="KWS execution provider")
    p.add_argument("--device", default="default", help="ALSA device, or card index (e.g. 6 -> hw:6,0)")
    p.add_argument("--capture-rate", type=int, default=48000)
    p.add_argument("--channels", type=int, default=2)
    p.add_argument("--blocksize", type=int, default=1024)
    p.add_argument("--no-vad", action="store_true")
    p.add_argument("--vad-threshold", type=float, default=.3)
    p.add_argument("--vad-hangover-ms", type=float, default=800)
    p.add_argument("--silence-reset-blocks", type=int, default=10)
    p.add_argument("--score", type=float, default=1.0)
    p.add_argument("--threshold", type=float, default=.25)
    p.add_argument("--trailing-blanks", type=int, default=1)
    p.add_argument("--gain", type=float, default=1.0)
    p.add_argument("--auto-gain", dest="auto_gain", action="store_true", default=True)
    p.add_argument("--no-auto-gain", dest="auto_gain", action="store_false")
    p.add_argument("--auto-gain-target-db", type=float, default=-20)
    p.add_argument("--auto-gain-max-db", type=float, default=30)
    p.add_argument("--auto-gain-min-db", type=float, default=-55)
    p.add_argument("--duration", type=float, default=0)
    p.add_argument("--wav", help="optional WAV input instead of microphone")
    args = p.parse_args()

    cfg = {"model_dir":str(MODEL_DIR), "keywords_file":str(KEYWORDS_FILE),
           "vad_model":str(VAD_MODEL), "sample_rate":16000, "num_threads":1,
           "keywords_score":args.score, "keywords_threshold":args.threshold,
           "num_trailing_blanks":args.trailing_blanks, "use_vad":not args.no_vad,
           "vad_threshold":args.vad_threshold, "vad_hangover_ms":args.vad_hangover_ms}
    print("=" * 60, "\nVoice-only KWS demo\n" + "=" * 60)
    print(f"  provider={args.provider}\n  device={args.device}\n  score={args.score}")
    print(f"  threshold={args.threshold}\n  trailing_blanks={args.trailing_blanks}")
    print(f"  gain={args.gain}\n  auto_gain={args.auto_gain}\n  vad={not args.no_vad}")
    print("  keywords=" + " / ".join(x.split(" @")[1].strip() for x in KEYWORDS_FILE.read_text().splitlines()))
    print("=" * 60 + "\nSpeak a keyword. Press Ctrl-C to stop.\n", flush=True)

    kws, _ = create_spotter(cfg); stream = kws.create_stream()
    vad = create_vad(cfg, sherpa_onnx)
    running = True
    def stop(_sig, _frame):
        nonlocal running; running = False
    signal.signal(signal.SIGINT, stop); signal.signal(signal.SIGTERM, stop)
    start = time.monotonic(); detections = 0; vad_state = False
    silent_blocks = 0; pending_reset = False
    for block in source_blocks(args):
        if not running or (args.duration and time.monotonic() - start >= args.duration): break
        stream.accept_waveform(16000, block)
        while kws.is_ready(stream): kws.decode_stream(stream)
        kw = kws.get_result(stream)
        if kw:
            detections += 1
            print(f"[{time.strftime('%H:%M:%S')}] Detected: {kw}  (total={detections})", flush=True)
            kws.reset_stream(stream)
        if vad:
            previous = vad_state; vad.accept_waveform(block); vad_state = vad.is_speech_detected()
            while not vad.empty(): vad.pop()
            if vad_state: silent_blocks = 0; pending_reset = True
            else:
                silent_blocks = 1 if previous else silent_blocks + (1 if pending_reset else 0)
                if pending_reset and silent_blocks >= args.silence_reset_blocks:
                    kws.reset_stream(stream); pending_reset = False; silent_blocks = 0
    print(f"Total detections: {detections}")


if __name__ == "__main__": main()
