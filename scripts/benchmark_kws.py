#!/usr/bin/env python3
"""Reproducible real-time KWS/VAD CPU, RSS, RTF and call-latency benchmark."""
import argparse
import json
import os
from pathlib import Path
import statistics
import sys
import time
import wave

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "third_party" / "python"))
sys.path.insert(0, str(ROOT / "src"))
import numpy as np
import sherpa_onnx
from voice_kws_server import create_spotter, create_vad


def percentile(values, q):
    if not values:
        return 0.0
    a = sorted(values)
    return a[min(len(a) - 1, int((len(a) - 1) * q))]


def summary(values):
    return {
        "count": len(values), "mean_ms": statistics.fmean(values) if values else 0.0,
        "p50_ms": percentile(values, .50), "p95_ms": percentile(values, .95),
        "p99_ms": percentile(values, .99), "max_ms": max(values, default=0.0),
    }


def rss_kib():
    with open("/proc/self/statm") as f:
        pages = int(f.read().split()[1])
    return pages * os.sysconf("SC_PAGE_SIZE") / 1024


def load_wav(path):
    with wave.open(str(path), "rb") as w:
        assert w.getnchannels() == 1 and w.getsampwidth() == 2
        rate = w.getframerate()
        data = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2").astype(np.float32) / 32768
    assert rate == 16000
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60)
    ap.add_argument("--no-vad", action="store_true")
    ap.add_argument("--no-pace", action="store_true", help="offline full-speed RTF test")
    ap.add_argument("--config", default="config/voice_kws.json")
    args = ap.parse_args()
    cfg_path = (ROOT / args.config).resolve()
    cfg = json.loads(cfg_path.read_text())
    for key in ("model_dir", "keywords_file", "vad_model"):
        p = Path(cfg[key]); cfg[key] = str(p if p.is_absolute() else ROOT / p)
    cfg["use_vad"] = not args.no_vad

    kws, _ = create_spotter(cfg)
    stream = kws.create_stream()
    vad = create_vad(cfg, sherpa_onnx)
    wavs = sorted((Path(cfg["model_dir"]) / "test_wavs").glob("zh_*.wav"))
    audio = np.concatenate([load_wav(p) for p in wavs])
    block_samples = 320
    blocks = int(args.duration * cfg["sample_rate"] / block_samples)
    decode_ms, vad_ms, block_ms, rss = [], [], [], []
    detections = 0
    audio_pos = 0
    cpu0, wall0 = time.process_time(), time.perf_counter()
    deadline = wall0
    for i in range(blocks):
        if audio_pos + block_samples > len(audio): audio_pos = 0
        block = audio[audio_pos:audio_pos + block_samples]; audio_pos += block_samples
        t0 = time.perf_counter()
        stream.accept_waveform(cfg["sample_rate"], block)
        while kws.is_ready(stream):
            t = time.perf_counter(); kws.decode_stream(stream)
            decode_ms.append((time.perf_counter() - t) * 1000)
        result = kws.get_result(stream)
        if result:
            detections += 1; kws.reset_stream(stream)
        if vad is not None:
            t = time.perf_counter(); vad.accept_waveform(block)
            vad_ms.append((time.perf_counter() - t) * 1000)
            while not vad.empty(): vad.pop()
        block_ms.append((time.perf_counter() - t0) * 1000)
        if i % 10 == 0: rss.append(rss_kib())
        if not args.no_pace:
            deadline += block_samples / cfg["sample_rate"]
            delay = deadline - time.perf_counter()
            if delay > 0: time.sleep(delay)
    wall = time.perf_counter() - wall0
    cpu = time.process_time() - cpu0
    report = {
        "mode": "kws+vad-soft" if vad else "kws-only", "paced": not args.no_pace,
        "audio_seconds": blocks * block_samples / cfg["sample_rate"],
        "wall_seconds": wall, "process_cpu_seconds": cpu,
        "cpu_percent_one_core": cpu / wall * 100, "rtf": cpu / (blocks * block_samples / cfg["sample_rate"]),
        "rss_steady_mib": statistics.fmean(rss) / 1024,
        "rss_peak_mib": max(rss) / 1024, "detections": detections,
        "decode_stream": summary(decode_ms), "vad_accept": summary(vad_ms),
        "whole_block": summary(block_ms),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__": main()
