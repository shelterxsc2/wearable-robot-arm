# -*- coding: utf-8 -*-
"""验证 camera_demo_lowlatency.py 中的 FPS 修复逻辑"""
import cv2
import time
import subprocess
import os

TARGET_CAMERA_FPS = int(os.environ.get('TARGET_CAMERA_FPS', '20'))
TARGET_CAMERA_FPS = max(10, min(30, TARGET_CAMERA_FPS))


def set_v4l2_exposure(target_fps):
    exposure_abs = int(round(10000.0 / target_fps))
    exposure_abs = max(50, min(10000, exposure_abs))
    try:
        subprocess.run(
            ['v4l2-ctl', '-d', '0',
             '--set-ctrl', f'auto_exposure=1,exposure_time_absolute={exposure_abs}'],
            capture_output=True, check=False, timeout=5
        )
        print(f"[Camera] Set exposure_time_absolute={exposure_abs} for target {target_fps}fps")
    except Exception as e:
        print(f"[Camera] Failed to set V4L2 exposure: {e}")


def measure_camera_fps(cap, duration=2.0):
    for _ in range(10):
        cap.read()
    t0 = time.time()
    frames = 0
    while time.time() - t0 < duration:
        ret, _ = cap.read()
        if ret:
            frames += 1
    elapsed = time.time() - t0
    return frames / elapsed if elapsed > 0 else 0.0


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

set_v4l2_exposure(TARGET_CAMERA_FPS)
fps = measure_camera_fps(cap, duration=3.0)
actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
fourcc_str = "".join([chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4)])
print(f"[Result] Target={TARGET_CAMERA_FPS}fps, Measured={fps:.1f}fps "
      f"({actual_w}x{actual_h} {fourcc_str})")
cap.release()
