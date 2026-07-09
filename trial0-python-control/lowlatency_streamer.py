# -*- coding: utf-8 -*-
"""Low-latency streaming + WebSocket reporter extracted from
/home/time/work/trial0/demos/camera_demo_lowlatency.py.

Usage from the main demo:
    streamer = LowLatencyStreamer(stream_url, ws_url, device_id="device-002")
    if streamer.probe_and_start(width, height, fps):
        # in output loop:
        streamer.write_frame(frame)
    # on shutdown:
    streamer.stop()

The behaviour mirrors the original camera_demo_lowlatency.py entry point,
with an added RTSP fallback:
1. Probe RTMP server.
2. If UP: start WebSocket worker (registration + 100ms heartbeat), wait for
   WS ready, then start ffmpeg RTMP stream.
3. If DOWN: probe the default RTSP server and start ffmpeg RTSP stream if UP.
4. If both are DOWN: do nothing (local-only mode).
"""
from __future__ import annotations

import json
import os
import queue
import socket
import subprocess
import threading
import time
from typing import Optional
from urllib.parse import urlparse

import numpy as np

FFMPEG_PATH = os.environ.get("FFMPEG_PATH", "ffmpeg")
VAAPI_DEVICE = os.environ.get("VAAPI_DEVICE", "/dev/dri/renderD128")
DEFAULT_DEVICE_ID = "device-002"

if "LIBVA_DRIVER_NAME" not in os.environ:
    os.environ["LIBVA_DRIVER_NAME"] = "iHD"


def _read_cloud_ip() -> Optional[str]:
    try:
        with open("/tmp/cloud_ip.txt", "r") as f:
            return f.readline().strip()
    except Exception:
        return None


def get_default_rtmp_url(device_id: str = DEFAULT_DEVICE_ID) -> str:
    """Return the same RTMP URL used by demos/camera_demo_lowlatency.py."""
    cloud_ip = _read_cloud_ip() or os.environ.get("CLOUD_IP", "47.93.162.124")
    return f"rtmp://{cloud_ip}:1935/live/{device_id}"


def get_default_ws_url(device_id: str = DEFAULT_DEVICE_ID) -> str:
    """Return the same WS URL used by demos/camera_demo_lowlatency.py."""
    return f"ws://47.93.162.124/ws?deviceId={device_id}"


def get_default_rtsp_url(device_id: str = DEFAULT_DEVICE_ID) -> str:
    """Return the default RTSP URL used when --rtsp is selected."""
    return "rtsp://127.0.0.1:8554/stream"


def _get_best_h264_encoder() -> str:
    try:
        result = subprocess.run(
            [FFMPEG_PATH, "-encoders"], capture_output=True, text=True, timeout=10
        )
        encoders = result.stdout
        if "h264_vaapi" in encoders:
            return "h264_vaapi"
        if "h264_qsv" in encoders:
            return "h264_qsv"
    except Exception:
        pass
    return "libx264"


def _probe_url(url: str, default_port: int, timeout_ms: int = 3000) -> bool:
    try:
        parsed = urlparse(url)
        host = parsed.hostname
        port = parsed.port or default_port
        with socket.create_connection((host, port), timeout=timeout_ms / 1000.0):
            return True
    except Exception as e:
        return e


def _probe_rtmp_server(rtmp_url: str, timeout_ms: int = 3000) -> bool:
    result = _probe_url(rtmp_url, 1935, timeout_ms)
    if result is True:
        return True
    print(f"[RTMP] Probe failed: {result}")
    return False


def _probe_rtsp_server(rtsp_url: str, timeout_ms: int = 3000) -> bool:
    result = _probe_url(rtsp_url, 554, timeout_ms)
    if result is True:
        return True
    print(f"[RTSP] Probe failed: {result}")
    return False


def _build_ffmpeg_cmd(rtmp_url: str, width: int, height: int, fps: int) -> list[str]:
    encoder = _get_best_h264_encoder()
    print(f"[FFmpeg] encoder={encoder}")

    if encoder == "h264_vaapi":
        rtsp_transport = os.environ.get("RTSP_TRANSPORT", "tcp")
        if rtmp_url.startswith("rtsp://"):
            out_fmt = ["-f", "rtsp", "-rtsp_transport", rtsp_transport, rtmp_url]
        else:
            out_fmt = ["-f", "flv", rtmp_url]
        return [
            FFMPEG_PATH, "-y",
            "-vaapi_device", VAAPI_DEVICE,
            "-f", "rawvideo", "-vcodec", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}",
            "-r", str(fps),
            "-thread_queue_size", "512",
            "-i", "-",
            "-vf", "format=nv12,hwupload",
            "-c:v", encoder,
            "-b:v", "4M", "-maxrate", "4M",
            "-g", str(max(1, fps // 2)),
            "-fps_mode", "passthrough",
        ] + out_fmt

    cmd = [
        FFMPEG_PATH, "-y",
        "-f", "rawvideo", "-vcodec", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{width}x{height}",
        "-r", str(fps),
        "-thread_queue_size", "512",
        "-i", "-",
        "-c:v", encoder,
        "-b:v", "4M", "-maxrate", "4M",
        "-g", str(max(1, fps // 2)),
        "-fps_mode", "passthrough",
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-probesize", "32", "-analyzeduration", "0",
    ]
    if encoder == "h264_qsv":
        cmd.extend(["-preset", "veryfast", "-pix_fmt", "nv12"])
    else:
        cmd.extend(["-preset", "ultrafast", "-tune", "zerolatency", "-pix_fmt", "yuv420p"])

    if rtmp_url.startswith("rtmp://"):
        cmd.extend(["-f", "flv", rtmp_url])
    elif rtmp_url.startswith("rtsp://"):
        # Default to UDP for compatibility with VLC's default RTSP transport.
        # Set RTSP_TRANSPORT=tcp to force TCP.
        rtsp_transport = os.environ.get("RTSP_TRANSPORT", "tcp")
        cmd.extend(["-f", "rtsp", "-rtsp_transport", rtsp_transport, rtmp_url])
    else:
        cmd.extend(["-f", "flv", rtmp_url])
    return cmd


def _default_json_default(o):
    if isinstance(o, np.generic):
        return float(o)
    return str(o)


class LowLatencyStreamer:
    """Mirrors camera_demo_lowlatency.py streaming + WS behaviour."""

    def __init__(
        self,
        stream_url: Optional[str] = None,
        ws_url: Optional[str] = None,
        device_id: str = "device-002",
        probe_timeout_ms: int = 3000,
        ws_ready_timeout_ms: int = 10000,
        rtmp_url: Optional[str] = None,  # backward compatibility
    ):
        self.stream_url = stream_url or rtmp_url
        self.ws_url = ws_url
        self.device_id = device_id
        self.probe_timeout_ms = probe_timeout_ms
        self.ws_ready_timeout_ms = ws_ready_timeout_ms

        self._stop_ev = threading.Event()
        self._ws_ready = threading.Event()
        self._ws_thread: Optional[threading.Thread] = None
        self._ffmpeg_proc: Optional[subprocess.Popen] = None
        self._ffmpeg_log = None
        self._started = False
        self._frame_count = 0
        self._stream_start_time: Optional[float] = None
        self._data_queue: queue.Queue = queue.Queue(maxsize=60)

    def _send_ws_heartbeat(self, ws, start_time: float):
        now = time.time()
        elapsed = (now - start_time) if start_time else 0.0
        msg = {
            "type": "frame_ts",
            "data": {
                "timestamp": int(now),
                "elapsed": round(elapsed, 3),
                "frame_count": self._frame_count,
                "device": self.device_id,
            },
        }
        ws.send(json.dumps(msg, default=_default_json_default))

    def _ws_worker(self):
        ws = None
        reconnect_delay = 2.0
        registered = False
        start_time = None
        last_beat = 0.0

        while not self._stop_ev.is_set():
            try:
                if ws is None:
                    self._ws_ready.clear()
                    registered = False
                    import websocket
                    ws = websocket.create_connection(self.ws_url, timeout=5)
                    print(f"[WS] Connected to cloud: {self.ws_url}")
                    reconnect_delay = 2.0
                    start_time = time.time()
                    ws.send(json.dumps({"type": "frame_ts"}))
                    print("[WS] Registration frame_ts sent")
                    time.sleep(0.2)
                    registered = True
                    self._ws_ready.set()
                    last_beat = time.time()

                now = time.time()
                if registered and now - last_beat >= 0.1:
                    self._send_ws_heartbeat(ws, start_time)
                    last_beat = now

                ws.settimeout(0.05)
                try:
                    msg = ws.recv()
                    packet = json.loads(msg)
                    ptype = packet.get("type")
                    if ptype in ("set_target", "target_pose"):
                        t = packet.get("target", {})
                        print(f"[Target] Received from cloud: {t}")
                    elif ptype == "ctrl_mode":
                        print(f"[CtrlMode] Received from cloud: {packet.get('ctrlMode', 0)}")
                    elif ptype == "track_obj":
                        print(f"[TrackObj] Received from cloud: {packet.get('trackObj', 0)}")
                except Exception:
                    pass

                try:
                    packet = self._data_queue.get(timeout=0.01)
                    ws.send(json.dumps(packet, default=_default_json_default))
                except queue.Empty:
                    pass

                time.sleep(0.01)
            except Exception as e:
                self._ws_ready.clear()
                registered = False
                if ws:
                    try:
                        ws.close()
                    except Exception:
                        pass
                    ws = None
                print(f"[WS] Connection error: {e}, reconnect in {reconnect_delay:.1f}s")
                for _ in range(int(reconnect_delay / 0.1)):
                    if self._stop_ev.is_set():
                        break
                    time.sleep(0.1)
                reconnect_delay = min(reconnect_delay * 1.5, 30.0)

        if ws:
            try:
                ws.close()
            except Exception:
                pass

    def _start_ws_worker(self):
        self._ws_thread = threading.Thread(target=self._ws_worker, daemon=True)
        self._ws_thread.start()
        print(f"[WS] WebSocket reporter started -> {self.ws_url}")
        print("[WS] Waiting for registration before starting RTMP stream...")

        wait_ms = 0
        while not self._ws_ready.is_set() and wait_ms < self.ws_ready_timeout_ms:
            time.sleep(0.1)
            wait_ms += 100
        if self._ws_ready.is_set():
            print("[WS] Registration confirmed. Starting RTMP stream now.")
        else:
            print("[WS] Registration timeout. Starting RTMP stream anyway.")

    def _start_ffmpeg(self, url: str, width: int, height: int, fps: int) -> bool:
        cmd = _build_ffmpeg_cmd(url, width, height, fps)
        try:
            self._ffmpeg_log = open("ffmpeg_lowlatency.log", "w")
            self._ffmpeg_proc = subprocess.Popen(
                cmd, stdin=subprocess.PIPE, stderr=self._ffmpeg_log
            )
            print(f"[Stream] FFmpeg started -> {url}")
            self._started = True
            return True
        except Exception as e:
            print(f"[Stream] Failed to start: {e}")
            self._ffmpeg_proc = None
            return False

    def probe_and_start(self, width: int, height: int, fps: int) -> bool:
        """Probe stream server and start ffmpeg.

        If stream_url is an RTSP URL, push RTSP directly (no WebSocket).
        If stream_url is an RTMP URL (or None for the default RTMP server),
        first probe RTMP. On failure, fall back to the default RTSP server.
        """
        url = self.stream_url

        # Explicit RTSP path
        if url and url.startswith("rtsp://"):
            print(f"[RTSP] Probing {url} ...")
            if not _probe_rtsp_server(url, self.probe_timeout_ms):
                print("[RTSP] Server is DOWN. Running locally without streaming.")
                return False
            print("[RTSP] Server is UP. Will push RTSP.")
            return self._start_ffmpeg(url, width, height, fps)

        # RTMP path (url is rtmp:// or None)
        rtmp_url = url or get_default_rtmp_url(self.device_id)
        print(f"[RTMP] Probing {rtmp_url} ...")
        if _probe_rtmp_server(rtmp_url, self.probe_timeout_ms):
            print("[RTMP] Server is UP. Will push RTMP + WebSocket.")
            self._start_ws_worker()
            return self._start_ffmpeg(rtmp_url, width, height, fps)

        # RTMP failed -> fall back to default RTSP
        print("[RTMP] Server is DOWN. Falling back to RTSP ...")
        rtsp_url = get_default_rtsp_url()
        print(f"[RTSP] Probing {rtsp_url} ...")
        if _probe_rtsp_server(rtsp_url, self.probe_timeout_ms):
            print("[RTSP] Server is UP. Will push RTSP.")
            return self._start_ffmpeg(rtsp_url, width, height, fps)

        print("[RTSP] Server is DOWN. Running locally without streaming.")
        return False

    def write_frame(self, frame) -> bool:
        if not self._started:
            return False
        self._frame_count += 1
        if self._stream_start_time is None and self._ffmpeg_proc is not None:
            self._stream_start_time = time.time()
        if self._ffmpeg_proc is not None and self._ffmpeg_proc.poll() is None:
            try:
                self._ffmpeg_proc.stdin.write(frame.tobytes())
                return True
            except Exception as e:
                print(f"[Stream] Write error: {e}")
                return False
        return False

    def stop(self) -> None:
        self._stop_ev.set()
        if self._ws_thread is not None:
            self._ws_thread.join(timeout=2.0)
        if self._ffmpeg_proc:
            try:
                self._ffmpeg_proc.stdin.close()
                self._ffmpeg_proc.wait(timeout=5)
            except Exception:
                self._ffmpeg_proc.kill()
            print("[Stream] stopped")
        if self._ffmpeg_log:
            try:
                self._ffmpeg_log.close()
            except Exception:
                pass
