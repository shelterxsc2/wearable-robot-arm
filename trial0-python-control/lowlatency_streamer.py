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
2. If RTMP handshake succeeds: start WebSocket worker (registration + 100ms heartbeat),
   wait for WS ready, then start ffmpeg RTMP stream.
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
import urllib.parse
import urllib.request
from enum import Enum
from typing import Optional
from urllib.parse import urlparse

import numpy as np

FFMPEG_PATH = os.environ.get("FFMPEG_PATH", "ffmpeg")
VAAPI_DEVICE = os.environ.get("VAAPI_DEVICE", "/dev/dri/renderD128")
DEFAULT_DEVICE_ID = "device-002"


class StreamState(str, Enum):
    IDLE = "idle"
    PROBING_CLOUD = "probing_cloud"
    STREAMING_CLOUD = "streaming_cloud"
    PROBING_LOCAL = "probing_local"
    STREAMING_LOCAL = "streaming_local"
    UNAVAILABLE = "unavailable"
    STOPPED = "stopped"

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


def build_ws_url(base_url: str, device_id: str, device_key: Optional[str] = None) -> str:
    """Add/replace deviceId and optional deviceKey query params."""
    parsed = urlparse(base_url)
    query = dict(urllib.parse.parse_qsl(parsed.query, keep_blank_values=True))
    query["deviceId"] = device_id
    if device_key:
        query["deviceKey"] = device_key
    return urllib.parse.urlunparse(parsed._replace(query=urllib.parse.urlencode(query)))


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
    tcp_result = _probe_url(rtmp_url, 1935, timeout_ms)
    if tcp_result is not True:
        print(f"[RTMP] TCP probe failed: {tcp_result}")
        return False

    timeout_s = max(2.0, timeout_ms / 1000.0 + 2.0)
    cmd = [
        FFMPEG_PATH, "-hide_banner", "-loglevel", "error", "-nostdin", "-y",
        "-f", "lavfi", "-i", "color=size=16x16:rate=1:color=black",
        "-t", "0.2", "-an", "-c:v", "libx264", "-preset", "ultrafast",
        "-tune", "zerolatency", "-pix_fmt", "yuv420p", "-f", "flv", rtmp_url,
    ]
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout_s, check=False
        )
    except subprocess.TimeoutExpired:
        print(f"[RTMP] Handshake probe timed out after {timeout_s:.1f}s")
        return False
    except Exception as e:
        print(f"[RTMP] Handshake probe failed: {e}")
        return False

    if result.returncode == 0:
        print("[RTMP] Handshake probe succeeded")
        return True
    err = (result.stderr or result.stdout or "").strip().splitlines()
    detail = err[-1] if err else f"exit={result.returncode}"
    print(f"[RTMP] Handshake probe failed: {detail}")
    return False


def _probe_rtsp_server(rtsp_url: str, timeout_ms: int = 3000) -> bool:
    result = _probe_url(rtsp_url, 554, timeout_ms)
    if result is True:
        return True
    print(f"[RTSP] Probe failed: {result}")
    return False


def _build_ffmpeg_cmd(rtmp_url: str, width: int, height: int, fps: int,
                      audio_fd: Optional[int] = None) -> list[str]:
    encoder = _get_best_h264_encoder()
    print(f"[FFmpeg] encoder={encoder}")

    cmd = [FFMPEG_PATH, "-y"]
    if encoder == "h264_vaapi":
        cmd.extend(["-vaapi_device", VAAPI_DEVICE])
    cmd.extend([
        "-f", "rawvideo", "-vcodec", "rawvideo", "-pix_fmt", "bgr24",
        "-s", f"{width}x{height}", "-r", str(fps),
        "-thread_queue_size", "512", "-i", "-",
    ])
    if audio_fd is not None:
        cmd.extend([
            "-thread_queue_size", "512", "-f", "f32le", "-ar", "16000",
            "-ac", "1", "-i", f"pipe:{audio_fd}",
        ])

    if encoder == "h264_vaapi":
        cmd.extend([
            "-vf", "format=nv12,hwupload",
            "-profile:v", "high",
            "-level:v", "4.0",
        ])
    cmd.extend(["-c:v", encoder])
    if encoder == "h264_vaapi":
        cmd.extend(["-rc_mode", "CQP", "-qp", "30"])
    else:
        cmd.extend(["-b:v", "4M", "-maxrate", "4M"])
    cmd.extend([
        "-g", str(max(1, fps // 2)), "-bf", "0", "-fps_mode", "passthrough",
    ])
    if encoder == "h264_qsv":
        cmd.extend(["-preset", "veryfast", "-pix_fmt", "nv12"])
    elif encoder != "h264_vaapi":
        cmd.extend(["-preset", "ultrafast", "-tune", "zerolatency", "-pix_fmt", "yuv420p"])

    if audio_fd is not None:
        cmd.extend([
            "-map", "0:v:0", "-map", "1:a:0",
            "-c:a", "aac", "-b:a", "64k", "-ar", "16000", "-ac", "1",
        ])
    if rtmp_url.startswith("rtmp://"):
        cmd.extend(["-flags:v", "+global_header"])
        cmd.extend(["-f", "flv", rtmp_url])
    elif rtmp_url.startswith("rtsp://"):
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
        device_key: Optional[str] = None,
        probe_timeout_ms: int = 3000,
        ws_ready_timeout_ms: int = 10000,
        rtmp_url: Optional[str] = None,  # backward compatibility
        on_target=None,
        on_zoom=None,
        on_view_mode=None,
        on_track_obj=None,
        on_annotation_mode=None,
        stream_mode: str = "auto",
        audio_queue: Optional[queue.Queue] = None,
    ):
        self.stream_url = stream_url or rtmp_url
        self.ws_url = build_ws_url(ws_url, device_id, device_key) if ws_url else None
        self.device_id = device_id
        self.device_key = device_key
        self.probe_timeout_ms = probe_timeout_ms
        self.ws_ready_timeout_ms = ws_ready_timeout_ms
        self.on_target = on_target
        self.on_zoom = on_zoom
        self.on_view_mode = on_view_mode
        self.on_track_obj = on_track_obj
        self.on_annotation_mode = on_annotation_mode
        self.stream_mode = stream_mode if stream_mode in ("auto", "cloud", "local") else "auto"
        self.stream_state = StreamState.IDLE
        self.active_stream_url: Optional[str] = None
        self.audio_queue = audio_queue

        self._stop_ev = threading.Event()
        self._ws_ready = threading.Event()
        self._ws_lock = threading.Lock()
        self._ws = None
        self._ws_thread: Optional[threading.Thread] = None
        self._ffmpeg_proc: Optional[subprocess.Popen] = None
        self._ffmpeg_log = None
        self._ffmpeg_lock = threading.RLock()
        self._switch_lock = threading.Lock()
        self._stream_width = 0
        self._stream_height = 0
        self._stream_fps = 0
        self._audio_thread: Optional[threading.Thread] = None
        self._audio_write_fd: Optional[int] = None
        self._started = False
        self._frame_count = 0
        self._stream_start_time: Optional[float] = None
        self._data_queue: queue.Queue = queue.Queue(maxsize=60)
        self._request_counter = 0
        self.record_state = {
            "isRecording": False,
            "currentVideoId": None,
            "currentStartTs": None,
            "currentDeviceId": None,
        }

    @staticmethod
    def _clamp_int(v, lo: int, hi: int, default: int = 0) -> int:
        try:
            iv = int(v)
        except Exception:
            iv = default
        return max(lo, min(hi, iv))

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
                    with self._ws_lock:
                        self._ws = ws
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
                        self._handle_target_packet(packet)
                    elif ptype == "set_zoom":
                        zoom = self._clamp_int(packet.get("zoom", 1), 0, 2, default=1)
                        if self.on_zoom is not None:
                            self.on_zoom(zoom, source="cloud")
                        print(f"[Zoom] Received from cloud: {zoom}")
                    elif ptype == "set_view_mode":
                        view_mode = self._clamp_int(packet.get("viewMode", 1), 0, 2, default=1)
                        if self.on_view_mode is not None:
                            self.on_view_mode(view_mode, source="cloud")
                        print(f"[ViewMode] Received from cloud: {view_mode}")
                    elif ptype == "set_annotation_mode":
                        annotation_mode = packet.get("annotationMode")
                        if isinstance(annotation_mode, bool):
                            if self.on_annotation_mode is not None:
                                self.on_annotation_mode(annotation_mode, source="cloud")
                            label = "annotation" if annotation_mode else "raw"
                            print(f"[AnnotationMode] Received from cloud: {annotation_mode} ({label})")
                        else:
                            print(f"[AnnotationMode] Ignored invalid value: {annotation_mode!r}")
                    elif ptype == "record_control_ack":
                        self._handle_record_ack(packet)
                    elif ptype == "record_state":
                        data = packet.get("data") or {}
                        if isinstance(data, dict):
                            self.record_state.update(data)
                        print(f"[Record] state: {self.record_state}")
                    elif ptype == "track_obj":
                        track_obj = self._clamp_int(packet.get("trackObj", 1), 0, 1, default=1)
                        if self.on_track_obj is not None:
                            self.on_track_obj(track_obj, source="cloud")
                        label = "first_person" if track_obj == 0 else "third_person"
                        print(f"[TrackObj] Received from cloud: {track_obj} ({label})")
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
                with self._ws_lock:
                    self._ws = None
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
        with self._ws_lock:
            self._ws = None

    def _handle_target_packet(self, packet: dict) -> None:
        target = packet.get("target") or {}
        xyz = {
            "x": self._clamp_int(target.get("x", 0), -5, 5, default=0),
            "y": self._clamp_int(target.get("y", 0), -5, 5, default=0),
            "z": self._clamp_int(target.get("z", 0), -5, 5, default=0),
        }
        if self.on_target is not None:
            self.on_target(xyz, source="cloud")
        print(f"[Target] Received from cloud: {xyz}")

    def _handle_record_ack(self, packet: dict) -> None:
        data = packet.get("data") or {}
        if isinstance(data, dict):
            self.record_state.update(data)
        success = bool(packet.get("success"))
        action = packet.get("action")
        if success:
            print(f"[Record] {action} ack ok: {self.record_state}")
        else:
            msg = f"{packet.get('error') or ''} {packet.get('message') or ''}".strip()
            print(f"[Record] {action} ack failed: {msg}")

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
        audio_read_fd = None
        audio_write_fd = None
        if self.audio_queue is not None:
            audio_read_fd, audio_write_fd = os.pipe()
        cmd = _build_ffmpeg_cmd(url, width, height, fps, audio_read_fd)
        try:
            with self._ffmpeg_lock:
                self._ffmpeg_log = open("ffmpeg_lowlatency.log", "w")
                popen_kwargs = {"stdin": subprocess.PIPE, "stderr": self._ffmpeg_log}
                if audio_read_fd is not None:
                    popen_kwargs["pass_fds"] = (audio_read_fd,)
                self._ffmpeg_proc = subprocess.Popen(cmd, **popen_kwargs)
                if audio_read_fd is not None:
                    os.close(audio_read_fd)
                    audio_read_fd = None
                    self._audio_write_fd = audio_write_fd
                    audio_write_fd = None
                    self._audio_thread = threading.Thread(target=self._audio_writer, daemon=True)
                    self._audio_thread.start()
                    print("[Stream] Audio enabled: mono 16 kHz AAC 64k")
                print(f"[Stream] FFmpeg started -> {url}")
                self._started = True
                self.active_stream_url = url
            return True
        except Exception as e:
            for fd in (audio_read_fd, audio_write_fd):
                if fd is not None:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
            print(f"[Stream] Failed to start: {e}")
            self._ffmpeg_proc = None
            return False

    def _stop_ffmpeg_only(self) -> None:
        with self._ffmpeg_lock:
            self._started = False
            if self._audio_write_fd is not None:
                try:
                    os.close(self._audio_write_fd)
                except OSError:
                    pass
                self._audio_write_fd = None
            audio_thread = self._audio_thread
            self._audio_thread = None
            proc = self._ffmpeg_proc
            self._ffmpeg_proc = None
            log = self._ffmpeg_log
            self._ffmpeg_log = None
            if proc is not None:
                try:
                    if proc.stdin is not None:
                        proc.stdin.close()
                    proc.wait(timeout=3.0)
                except Exception:
                    proc.kill()
                    try:
                        proc.wait(timeout=1.0)
                    except Exception:
                        pass
            if log is not None:
                try:
                    log.close()
                except Exception:
                    pass
            self.active_stream_url = None
        if audio_thread is not None and audio_thread is not threading.current_thread():
            audio_thread.join(timeout=1.0)

    def _audio_writer(self) -> None:
        while not self._stop_ev.is_set() and self._audio_write_fd is not None:
            try:
                block = self.audio_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                view = memoryview(np.asarray(block, dtype=np.float32).tobytes())
                while view:
                    written = os.write(self._audio_write_fd, view)
                    view = view[written:]
            except (BrokenPipeError, OSError):
                break

    def _set_stream_state(self, state: StreamState) -> None:
        if self.stream_state != state:
            print(f"[StreamState] {self.stream_state.value} -> {state.value}")
            self.stream_state = state

    def _start_local_stream(self, url: str, width: int, height: int, fps: int) -> bool:
        self._set_stream_state(StreamState.PROBING_LOCAL)
        print(f"[RTSP] Probing {url} ...")
        if not _probe_rtsp_server(url, self.probe_timeout_ms):
            self._set_stream_state(StreamState.UNAVAILABLE)
            print("[RTSP] Server is DOWN. Running locally without streaming.")
            return False
        print("[RTSP] Server is UP. Will push RTSP.")
        if self._start_ffmpeg(url, width, height, fps):
            self._set_stream_state(StreamState.STREAMING_LOCAL)
            return True
        self._set_stream_state(StreamState.UNAVAILABLE)
        return False

    def probe_and_start(self, width: int, height: int, fps: int) -> bool:
        """Select cloud or local output using the configured stream state machine."""
        self._stream_width = int(width)
        self._stream_height = int(height)
        self._stream_fps = int(fps)
        url = self.stream_url
        local_url = url if url and url.startswith("rtsp://") else get_default_rtsp_url()

        if self.stream_mode == "local" or (url and url.startswith("rtsp://")):
            return self._start_local_stream(local_url, width, height, fps)

        self._set_stream_state(StreamState.PROBING_CLOUD)
        rtmp_url = url or get_default_rtmp_url(self.device_id)
        print(f"[RTMP] Handshake probing {rtmp_url} ...")
        if _probe_rtmp_server(rtmp_url, self.probe_timeout_ms):
            print("[RTMP] Server is UP. Will push RTMP + WebSocket.")
            self._start_ws_worker()
            if self._start_ffmpeg(rtmp_url, width, height, fps):
                self._set_stream_state(StreamState.STREAMING_CLOUD)
                return True
            self._set_stream_state(StreamState.UNAVAILABLE)
            return False

        if self.stream_mode == "cloud":
            self._set_stream_state(StreamState.UNAVAILABLE)
            print("[RTMP] Server is DOWN. Cloud-only mode does not fall back.")
            return False

        print("[RTMP] Server is DOWN. Falling back to RTSP ...")
        return self._start_local_stream(local_url, width, height, fps)

    def switch_stream_mode(self, mode: str) -> dict:
        mode = (mode or "").strip().lower()
        if mode not in ("auto", "cloud", "local"):
            return {"ok": False, "error": "mode must be auto, cloud or local"}
        if not self._stream_width or not self._stream_height or not self._stream_fps:
            return {"ok": False, "error": "stream dimensions are not initialized"}

        with self._switch_lock:
            running = self._ffmpeg_proc is not None and self._ffmpeg_proc.poll() is None
            if running:
                already_cloud = self.stream_state == StreamState.STREAMING_CLOUD
                already_local = self.stream_state == StreamState.STREAMING_LOCAL
                if ((mode == "cloud" and already_cloud) or
                        (mode == "local" and already_local) or
                        (mode == "auto" and (already_cloud or already_local))):
                    self.stream_mode = mode
                    return self.get_stream_status(ok=True, changed=False)

            cloud_url = get_default_rtmp_url(self.device_id)
            local_url = get_default_rtsp_url(self.device_id)
            target_mode = mode
            target_url = None

            if mode in ("auto", "cloud"):
                print(f"[StreamSwitch] Probing cloud {cloud_url} ...")
                if _probe_rtmp_server(cloud_url, self.probe_timeout_ms):
                    target_mode = "cloud"
                    target_url = cloud_url
                elif mode == "cloud":
                    return {"ok": False, "mode": mode, "error": "cloud probe failed"}

            if target_url is None:
                print(f"[StreamSwitch] Probing local {local_url} ...")
                if not _probe_rtsp_server(local_url, self.probe_timeout_ms):
                    return {"ok": False, "mode": mode, "error": "local probe failed"}
                target_mode = "local"
                target_url = local_url

            if self.active_stream_url == target_url and self._ffmpeg_proc is not None and self._ffmpeg_proc.poll() is None:
                self.stream_mode = mode
                return self.get_stream_status(ok=True, changed=False)

            old_url = self.active_stream_url
            self._stop_ffmpeg_only()
            if not self._start_ffmpeg(
                target_url, self._stream_width, self._stream_height, self._stream_fps
            ):
                restored = False
                if old_url:
                    restored = self._start_ffmpeg(
                        old_url, self._stream_width, self._stream_height, self._stream_fps
                    )
                if restored:
                    old_state = (
                        StreamState.STREAMING_CLOUD
                        if old_url.startswith("rtmp://")
                        else StreamState.STREAMING_LOCAL
                    )
                    self._set_stream_state(old_state)
                else:
                    self._set_stream_state(StreamState.UNAVAILABLE)
                return {
                    "ok": False,
                    "mode": mode,
                    "error": "FFmpeg start failed",
                    "oldUrl": old_url,
                    "restored": restored,
                }

            self.stream_mode = mode
            state = StreamState.STREAMING_CLOUD if target_mode == "cloud" else StreamState.STREAMING_LOCAL
            self._set_stream_state(state)
            print(f"[StreamSwitch] {old_url} -> {target_url}")
            return self.get_stream_status(ok=True, changed=True)

    def get_stream_status(self, ok: bool = True, changed: Optional[bool] = None) -> dict:
        running = self._ffmpeg_proc is not None and self._ffmpeg_proc.poll() is None
        result = {
            "ok": ok,
            "mode": self.stream_mode,
            "state": self.stream_state.value,
            "running": running,
            "url": self.active_stream_url,
        }
        if changed is not None:
            result["changed"] = changed
        return result

    def send_ws_packet(self, packet: dict) -> bool:
        """Queue a packet for the cloud WebSocket, preserving stream thread ownership."""
        if not self.ws_url or not self._ws_ready.is_set():
            return False
        try:
            self._data_queue.put_nowait(packet)
            return True
        except queue.Full:
            return False

    def report_zoom(self, zoom: int) -> bool:
        return self.send_ws_packet({
            "type": "zoom",
            "zoom": self._clamp_int(zoom, 0, 2, default=1),
        })

    def report_view_mode(self, view_mode: int) -> bool:
        return self.send_ws_packet({
            "type": "view_mode",
            "viewMode": self._clamp_int(view_mode, 0, 2, default=1),
        })

    def report_track_obj(self, track_obj: int) -> bool:
        return self.send_ws_packet({
            "type": "track_obj",
            "trackObj": self._clamp_int(track_obj, 0, 1, default=1),
        })

    def report_annotation_mode(self, enabled: bool) -> bool:
        return self.send_ws_packet({
            "type": "annotation_mode",
            "annotationMode": bool(enabled),
        })

    def report_target_pose(self, x: int, y: int, z: int) -> bool:
        return self.send_ws_packet({
            "type": "target_pose",
            "target": {
                "x": self._clamp_int(x, -5, 5, default=0),
                "y": self._clamp_int(y, -5, 5, default=0),
                "z": self._clamp_int(z, -5, 5, default=0),
            },
        })

    def request_record(self, action: str, metadata: Optional[dict] = None) -> bool:
        action = (action or "").strip().lower()
        if action not in ("start", "stop"):
            return False
        self._request_counter += 1
        packet = {
            "type": "record_control",
            "action": action,
            "requestId": f"req-{int(time.time())}-{self._request_counter:03d}",
            "timestamp": time.time(),
            "metadata": metadata or {"source": "device"},
        }
        if self.send_ws_packet(packet):
            print(f"[Record] queued WS {action}: {packet['requestId']}")
            return True
        return self.request_record_http(action, packet["requestId"], packet["metadata"])

    def request_record_http(self, action: str, request_id: Optional[str] = None,
                            metadata: Optional[dict] = None) -> bool:
        """Fallback REST record-control call used when WS is unavailable."""
        action = (action or "").strip().lower()
        if action not in ("start", "stop"):
            return False
        host = os.environ.get("CLOUD_IP")
        if not host and self.stream_url and self.stream_url.startswith("rtmp://"):
            parsed = urlparse(self.stream_url)
            host = parsed.hostname
        host = host or "47.93.162.124"
        port = os.environ.get("CLOUD_HTTP_PORT", "3000")
        body = json.dumps({
            "deviceId": self.device_id,
            "requestId": request_id or f"req-{int(time.time())}",
            "metadata": metadata or {"source": "device"},
        }).encode("utf-8")
        url = (
            f"http://{host}:{port}/api/device/record/{action}"
            f"?deviceId={urllib.parse.quote(self.device_id)}"
        )
        req = urllib.request.Request(
            url,
            data=body,
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        if self.device_key:
            req.add_header("X-Device-ID", self.device_id)
            req.add_header("X-Device-Key", self.device_key)
        try:
            with urllib.request.urlopen(req, timeout=3.0) as resp:
                payload = json.loads(resp.read().decode("utf-8") or "{}")
            data = payload.get("data") or {}
            if isinstance(data, dict):
                self.record_state.update(data)
            ok = bool(payload.get("success", True))
            print(f"[Record] HTTP {action} ok={ok}: {self.record_state}")
            return ok
        except Exception as e:
            print(f"[Record] HTTP {action} failed: {e}")
            return False

    def write_frame(self, frame) -> bool:
        with self._ffmpeg_lock:
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

    def stop(self) -> None:
        self._stop_ev.set()
        self._stop_ffmpeg_only()
        if self._ws_thread is not None:
            self._ws_thread.join(timeout=2.0)
        self._set_stream_state(StreamState.STOPPED)
