# -*- coding: utf-8 -*-
"""Simple ffmpeg-based RTMP/RTSP streamer for the main demo.

Feeds raw BGR frames to an ffmpeg subprocess.  Encoder auto-selection
prefers hardware encoders (VAAPI, QSV) and falls back to libx264.
"""
import os
import shutil
import subprocess
import time
from typing import Optional

FFMPEG_PATH = os.environ.get("FFMPEG_PATH", "ffmpeg")


def _best_encoder() -> str:
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


def _build_cmd(url: str, width: int, height: int, fps: int) -> list[str]:
    encoder = _best_encoder()
    print(f"[Streamer] encoder={encoder}, size={width}x{height}, fps={fps}")

    if encoder == "h264_vaapi":
        vaapi_device = os.environ.get("VAAPI_DEVICE", "/dev/dri/renderD128")
        rtsp_transport = os.environ.get("RTSP_TRANSPORT", "udp")
        if url.startswith("rtsp://"):
            out_fmt = ["-f", "rtsp", "-rtsp_transport", rtsp_transport, url]
        else:
            out_fmt = ["-f", "flv", url]
        return [
            FFMPEG_PATH, "-y",
            "-vaapi_device", vaapi_device,
            "-f", "rawvideo", "-vcodec", "rawvideo", "-pix_fmt", "bgr24",
            "-s", f"{width}x{height}",
            "-r", str(fps),
            "-thread_queue_size", "512",
            "-i", "-",
            "-vf", "format=nv12,hwupload",
            "-c:v", encoder,
            "-b:v", "4M", "-maxrate", "4M",
            "-g", str(fps),
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
        "-g", str(fps),
        "-fps_mode", "passthrough",
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-probesize", "32", "-analyzeduration", "0",
    ]
    if encoder == "h264_qsv":
        cmd.extend(["-preset", "veryfast", "-pix_fmt", "nv12"])
    else:
        cmd.extend(["-preset", "ultrafast", "-tune", "zerolatency", "-pix_fmt", "yuv420p"])

    # RTMP -> flv; RTSP -> rtsp; otherwise let ffmpeg infer.
    if url.startswith("rtmp://"):
        cmd.extend(["-f", "flv", url])
    elif url.startswith("rtsp://"):
        rtsp_transport = os.environ.get("RTSP_TRANSPORT", "udp")
        cmd.extend(["-f", "rtsp", "-rtsp_transport", rtsp_transport, url])
    else:
        cmd.append(url)
    return cmd


class Streamer:
    """Push raw BGR frames to an RTMP/RTSP server via ffmpeg."""

    def __init__(
        self,
        url: str,
        width: int,
        height: int,
        fps: int = 20,
    ):
        if shutil.which(FFMPEG_PATH) is None:
            raise RuntimeError(f"ffmpeg not found: {FFMPEG_PATH}")
        self.url = url
        self.width = width
        self.height = height
        self.fps = fps
        self._proc: Optional[subprocess.Popen] = None
        self._log = open("ffmpeg_main.log", "w")
        self._started = False

    def start(self) -> bool:
        cmd = _build_cmd(self.url, self.width, self.height, self.fps)
        try:
            self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=self._log)
            self._started = True
            print(f"[Streamer] started -> {self.url}")
            return True
        except Exception as e:
            print(f"[Streamer] failed to start: {e}")
            return False

    def write_frame(self, frame) -> bool:
        if self._proc is None or self._proc.poll() is not None:
            return False
        try:
            self._proc.stdin.write(frame.tobytes())
            self._proc.stdin.flush()
            return True
        except Exception as e:
            print(f"[Streamer] write error: {e}")
            return False

    def stop(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            try:
                self._proc.stdin.close()
                self._proc.wait(timeout=5)
            except Exception:
                self._proc.kill()
            print("[Streamer] stopped")
        try:
            self._log.close()
        except Exception:
            pass
