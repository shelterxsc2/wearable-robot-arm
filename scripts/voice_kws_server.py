#!/usr/bin/env python3
"""ALSA -> sherpa-onnx streaming KWS -> Unix JSON-lines sidecar."""
import argparse
import json
import os
from pathlib import Path
import selectors
import signal
import socket
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
bundled_python = ROOT / "third_party" / "python"
if bundled_python.is_dir():
    sys.path.insert(0, str(bundled_python))


def now_ms():
    return time.monotonic_ns() // 1_000_000


class EventServer:
    def __init__(self, path):
        self.path = path
        self.clients = set()
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(path)
        os.chmod(path, 0o660)
        self.sock.listen(4)
        self.sock.setblocking(False)

    def accept(self):
        while True:
            try:
                client, _ = self.sock.accept()
            except BlockingIOError:
                return
            client.setblocking(False)
            self.clients.add(client)

    def publish(self, event):
        data = (json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n").encode()
        dead = []
        for client in self.clients:
            try:
                client.sendall(data)
            except (BrokenPipeError, ConnectionResetError, BlockingIOError):
                dead.append(client)
        for client in dead:
            client.close()
            self.clients.discard(client)

    def close(self):
        for client in self.clients:
            client.close()
        self.sock.close()
        try:
            os.unlink(self.path)
        except FileNotFoundError:
            pass


def find_models(model_dir):
    model_dir = Path(model_dir)
    def pick(patterns):
        for pattern in patterns:
            found = sorted(model_dir.glob(pattern))
            if found:
                return str(found[0])
        raise FileNotFoundError(f"missing {'/'.join(patterns)} in {model_dir}")
    return {
        "tokens": str(model_dir / "tokens.txt"),
        "encoder": pick(["*encoder*.int8.onnx", "*encoder*.onnx"]),
        "decoder": pick(["*decoder*.onnx"]),
        "joiner": pick(["*joiner*.int8.onnx", "*joiner*.onnx"]),
    }


def create_spotter(cfg):
    try:
        import sherpa_onnx
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("sherpa_onnx Python module is unavailable; use the bundled C++ binary or install the matching wheel") from exc
    m = find_models(cfg["model_dir"])
    spotter = sherpa_onnx.KeywordSpotter(
        tokens=m["tokens"], encoder=m["encoder"], decoder=m["decoder"],
        joiner=m["joiner"], keywords_file=cfg["keywords_file"],
        num_threads=cfg["num_threads"], sample_rate=cfg["sample_rate"],
        feature_dim=80, keywords_score=cfg["keywords_score"],
        keywords_threshold=cfg["keywords_threshold"],
        num_trailing_blanks=cfg["num_trailing_blanks"], provider="cpu")
    return spotter, np


def create_vad(cfg, sherpa_onnx):
    if not cfg.get("use_vad", True):
        return None
    silero = sherpa_onnx.SileroVadModelConfig(
        model=cfg["vad_model"],
        threshold=cfg.get("vad_threshold", 0.3),
        min_silence_duration=cfg.get("vad_hangover_ms", 800) / 1000.0,
        min_speech_duration=0.1,
        window_size=512,
        max_speech_duration=20.0)
    vad_cfg = sherpa_onnx.VadModelConfig(
        silero_vad=silero, sample_rate=cfg["sample_rate"],
        num_threads=1, provider="cpu", debug=False)
    return sherpa_onnx.VoiceActivityDetector(vad_cfg, buffer_size_in_seconds=30)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/voice_kws.json")
    parser.add_argument("--wav", help="read a 16-bit mono 16-kHz raw PCM file instead of ALSA")
    args = parser.parse_args()
    cfg_path = Path(args.config).resolve()
    cfg = json.loads(cfg_path.read_text())
    root = cfg_path.parent.parent
    for key in ("model_dir", "keywords_file", "vad_model"):
        p = Path(cfg[key])
        cfg[key] = str(p if p.is_absolute() else root / p)

    spotter, np = create_spotter(cfg)
    import sherpa_onnx
    vad = create_vad(cfg, sherpa_onnx)
    stream = spotter.create_stream()
    server = EventServer(cfg["socket_path"])
    running = True
    def stop(_sig, _frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    if args.wav:
        source = open(args.wav, "rb")
        proc = None
    else:
        cmd = ["arecord", "-q", "-D", cfg["alsa_device"], "-t", "raw", "-f", "S16_LE",
               "-r", str(cfg["sample_rate"]), "-c", "1"]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        source = proc.stdout
    sequence = 0
    last_keyword = None
    last_emit = 0
    vad_state = False
    silent_blocks = 0
    awaiting_silence_reset = False
    try:
        print(f"[VoiceKWS] ready socket={cfg['socket_path']} provider=cpu vad={'soft' if vad else 'off'}", flush=True)
        while running:
            server.accept()
            raw = source.read(640)  # 20 ms at 16 kHz/S16_LE
            if not raw:
                if args.wav:
                    break
                raise RuntimeError("ALSA capture ended")
            samples = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
            stream.accept_waveform(cfg["sample_rate"], samples)
            while spotter.is_ready(stream):
                spotter.decode_stream(stream)
            keyword = spotter.get_result(stream)
            detected = now_ms()
            if keyword and (keyword != last_keyword or detected - last_emit >= cfg["cooldown_ms"]):
                sequence += 1
                server.publish({"type":"keyword", "keyword":keyword, "score":1.0,
                                "audio_end_ms":detected, "detected_ms":detected,
                                "sequence":sequence})
                print(f"[VoiceKWS] detected keyword={keyword} sequence={sequence}", flush=True)
                last_keyword, last_emit = keyword, detected
                spotter.reset_stream(stream)
            elif not keyword and detected - last_emit >= cfg["cooldown_ms"]:
                last_keyword = None

            # Soft gate, matching trial0: KWS always receives and decodes every
            # block. VAD only resets stale decoder state after speech has ended
            # and remained silent; a brief false negative never drops audio.
            if vad is not None:
                previous = vad_state
                vad.accept_waveform(samples)
                vad_state = vad.is_speech_detected()
                while not vad.empty():
                    vad.pop()
                if vad_state:
                    silent_blocks = 0
                    awaiting_silence_reset = True
                else:
                    if previous:
                        silent_blocks = 1
                    elif awaiting_silence_reset:
                        silent_blocks += 1
                    if awaiting_silence_reset and silent_blocks >= cfg.get("silence_reset_blocks", 10):
                        spotter.reset_stream(stream)
                        silent_blocks = 0
                        awaiting_silence_reset = False
    finally:
        server.close()
        if proc is not None:
            proc.terminate()
            proc.wait(timeout=2)
        else:
            source.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"[VoiceKWS] fatal: {exc}", file=sys.stderr)
        sys.exit(1)
