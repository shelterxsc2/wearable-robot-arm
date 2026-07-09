# -*- coding: utf-8 -*-
"""摄像头与 pipeline 诊断工具：定位 15fps 瓶颈"""
import cv2
import time
import os
import sys
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 检查是否有摄像头
for i in range(4):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"\n[Camera] 发现摄像头 /dev/video{i}")
        cap.release()
        break
else:
    print("[Camera] 未找到可用摄像头")
    sys.exit(1)

def test_camera_fps(width, height, fourcc_name, duration=3.0):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if fourcc_name:
        fourcc = cv2.VideoWriter_fourcc(*fourcc_name)
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    # 预热
    for _ in range(10):
        cap.read()
    
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps_prop = cap.get(cv2.CAP_PROP_FPS)
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join([chr((actual_fourcc >> 8*i) & 0xFF) for i in range(4)])
    
    t0 = time.time()
    frames = 0
    while time.time() - t0 < duration:
        ret, _ = cap.read()
        if ret:
            frames += 1
    elapsed = time.time() - t0
    measured_fps = frames / elapsed
    cap.release()
    
    return {
        'req_w': width, 'req_h': height,
        'actual_w': actual_w, 'actual_h': actual_h,
        'fourcc': fourcc_str,
        'fps_prop': actual_fps_prop,
        'measured_fps': measured_fps,
        'frames': frames,
        'elapsed': elapsed
    }

print("\n" + "="*70)
print("1. 摄像头原生帧率测试（不推理、不推流）")
print("="*70)
configs = [
    (1920, 1080, 'MJPG'),
    (1920, 1080, None),
    (1280, 720, 'MJPG'),
    (1280, 720, None),
    (640, 480, 'MJPG'),
    (640, 480, None),
]
for w, h, fcc in configs:
    try:
        r = test_camera_fps(w, h, fcc, duration=3.0)
        fcc_req = fcc or 'auto'
        print(f"  {w}x{h} {fcc_req:5s} -> 实际 {r['actual_w']}x{r['actual_h']} {r['fourcc']:4s} "
              f"CAP_PROP_FPS={r['fps_prop']:.1f} 实测={r['measured_fps']:.1f}fps "
              f"({r['frames']}帧/{r['elapsed']:.2f}s)")
    except Exception as e:
        print(f"  {w}x{h} {fcc or 'auto'} -> 错误: {e}")

print("\n" + "="*70)
print("2. 当前 camera_demo_lowlatency.py 配置实测")
print("="*70)

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
for _ in range(10):
    cap.read()

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps_prop = cap.get(cv2.CAP_PROP_FPS)
fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
fcc_str = "".join([chr((fcc >> 8*i) & 0xFF) for i in range(4)])
print(f"  请求 1920x1080 MJPG BUFSIZE=1")
print(f"  实际 {w}x{h} {fcc_str:4s} CAP_PROP_FPS={fps_prop:.1f}")

t0 = time.time()
frames = 0
while time.time() - t0 < 3.0:
    ret, _ = cap.read()
    if ret:
        frames += 1
elapsed = time.time() - t0
print(f"  纯采集帧率: {frames/elapsed:.1f}fps ({frames}帧/{elapsed:.2f}s)")
cap.release()

print("\n" + "="*70)
print("3. 推理耗时测量（仅加载面部模型）")
print("="*70)
try:
    from ultralytics import YOLO
    face_model = YOLO(os.path.join(SCRIPT_DIR, "models", "best_wflw_v8_pose20_openvino_model"), task="pose")
    
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(5):
        cap.read()
    
    infer_times = []
    loop_times = []
    total_frames = 0
    t_start = time.time()
    while time.time() - t_start < 5.0:
        t_loop = time.time()
        ret, frame = cap.read()
        if not ret:
            continue
        t0 = time.time()
        _ = face_model(frame, device="intel:gpu", verbose=False)
        t1 = time.time()
        infer_times.append(t1 - t0)
        loop_times.append(time.time() - t_loop)
        total_frames += 1
    cap.release()
    
    if infer_times:
        print(f"  推理帧数: {len(infer_times)}")
        print(f"  平均单帧推理: {np.mean(infer_times)*1000:.1f}ms "
              f"(min={np.min(infer_times)*1000:.1f}ms max={np.max(infer_times)*1000:.1f}ms)")
        print(f"  平均端到端循环: {np.mean(loop_times)*1000:.1f}ms -> 理论 FPS={1.0/np.mean(loop_times):.1f}")
        print(f"  实测总帧数/时间: {total_frames}帧/{time.time()-t_start:.2f}s")
except Exception as e:
    print(f"  推理测试失败: {e}")

print("\n" + "="*70)
print("4. 关键结论")
print("="*70)
