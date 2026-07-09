#!/usr/bin/env python3
"""One-command launcher: local RTSP server + main demo.

Usage:
    ./run_elf_all.py
    ./run_elf_all.py --show-window
    ./run_elf_all.py --no-stream

Ctrl+C (SIGINT) stops both the RTSP server and the main demo cleanly.
"""
from __future__ import annotations

import os
import shlex
import signal
import socket
import subprocess
import sys
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
os.chdir(BASE_DIR)

LOG_DIR = os.path.join(BASE_DIR, "logs")
os.makedirs(LOG_DIR, exist_ok=True)

CONDA_ACTIVATE = "source /home/time/miniconda3/etc/profile.d/conda.sh && conda activate trial0"
RTSP_HOST = "127.0.0.1"
RTSP_PORT = 8554
RTSP_READY_TIMEOUT = 10.0
_SHUTDOWN_TIMEOUT = 15.0

_stopped = False


def _wait_for_port(host: str, port: int, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.2):
                return True
        except Exception:
            time.sleep(0.1)
    return False


def _get_lan_ips() -> list[str]:
    """Return non-loopback IPv4 addresses for display."""
    ips: list[str] = []
    try:
        result = subprocess.run(
            ["hostname", "-I"], capture_output=True, text=True, check=False
        )
        for ip in result.stdout.strip().split():
            if ip.startswith("127.") or ":" in ip:
                continue
            ips.append(ip)
    except Exception:
        pass
    return ips


def _start_rtsp_server() -> subprocess.Popen:
    log_path = os.path.join(LOG_DIR, "mediamtx.log")
    log = open(log_path, "w")
    proc = subprocess.Popen(
        ["./start_rtsp_server.sh"],
        stdout=log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )
    if not _wait_for_port(RTSP_HOST, RTSP_PORT, RTSP_READY_TIMEOUT):
        print(f"[launcher] RTSP server did not become ready on {RTSP_HOST}:{RTSP_PORT}")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except Exception:
            proc.terminate()
        proc.wait(timeout=_SHUTDOWN_TIMEOUT)
        sys.exit(1)
    print(f"[launcher] RTSP server ready -> rtsp://{RTSP_HOST}:{RTSP_PORT}/stream")
    return proc


def _start_main_demo(args: list[str]) -> subprocess.Popen:
    quoted = " ".join(map(shlex.quote, args))
    cmd = ["bash", "-c", f"{CONDA_ACTIVATE} && exec ./run_elf_main.sh {quoted}"]
    proc = subprocess.Popen(cmd, preexec_fn=os.setsid)
    print(f"[launcher] main demo started (pid={proc.pid})")
    return proc


def _stop_proc(proc: subprocess.Popen | None, sig: int) -> None:
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except Exception:
        try:
            proc.send_signal(sig)
        except Exception:
            pass


def _cleanup(main_proc: subprocess.Popen | None, rtsp_proc: subprocess.Popen | None) -> None:
    global _stopped
    if _stopped:
        return
    _stopped = True
    print("\n[launcher] shutting down...")

    # 1. Ask the main demo to stop gracefully with SIGINT so it can send
    #    the H7 retract frame and join its worker threads.
    if main_proc is not None:
        print("[launcher] stopping main demo...")
        _stop_proc(main_proc, signal.SIGINT)
        try:
            main_proc.wait(timeout=_SHUTDOWN_TIMEOUT)
            print("[launcher] main demo stopped")
        except Exception:
            print("[launcher] main demo did not stop gracefully, killing...")
            _stop_proc(main_proc, signal.SIGKILL)
            try:
                main_proc.wait(timeout=3.0)
            except Exception:
                pass

    # 2. Stop the RTSP server after the main demo has released ffmpeg.
    if rtsp_proc is not None:
        print("[launcher] stopping RTSP server...")
        _stop_proc(rtsp_proc, signal.SIGTERM)
        try:
            rtsp_proc.wait(timeout=5.0)
            print("[launcher] RTSP server stopped")
        except Exception:
            _stop_proc(rtsp_proc, signal.SIGKILL)

    print("[launcher] stopped")


def main() -> None:
    extra_args = list(sys.argv[1:])

    has_stream_override = (
        "--rtsp" in extra_args
        or "--no-stream" in extra_args
        or any(a == "--stream-url" or a.startswith("--stream-url=") for a in extra_args)
    )

    # This launcher owns a local MediaMTX server, so default to local RTSP
    # instead of the cloud RTMP URL used by run_elf_main.sh directly.
    if not has_stream_override:
        extra_args.append("--rtsp")

    # Allow pushing to a LAN-visible address via RTSP_HOST.
    # If the user already passed --stream-url, do not override.
    rtsp_host = os.environ.get("RTSP_HOST")
    if rtsp_host and not any(a == "--stream-url" or a.startswith("--stream-url=") for a in extra_args):
        extra_args.extend(["--stream-url", f"rtsp://{rtsp_host}:8554/stream"])

    rtsp_proc = _start_rtsp_server()
    main_proc = _start_main_demo(extra_args)

    print("[launcher] RTSP server listening on all interfaces (0.0.0.0:8554)")
    for ip in _get_lan_ips():
        print(f"[launcher] LAN clients can watch at rtsp://{ip}:8554/stream")

    def _on_signal(signum, frame):
        _cleanup(main_proc, rtsp_proc)
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    try:
        main_proc.wait()
    finally:
        _cleanup(main_proc, rtsp_proc)


if __name__ == "__main__":
    main()
