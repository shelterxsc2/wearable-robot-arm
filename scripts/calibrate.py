#!/usr/bin/env python3
"""
USB Camera3 相机标定脚本（RK3588 全自动模式）

用法:
  python3 calibrate.py 9 6

采集要求：
  - 只保存检测到棋盘格角点 + 角度变化足够大的帧
  - 目标 20 张，务必让棋盘格在画面中心区域，多角度变化
"""

import cv2
import numpy as np
import os
import sys
import time

# ================== 配置 ==================
CHECKERBOARD = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) >= 3 else (9, 6)
SQUARE_SIZE = 25.0
SAVE_DIR = "./calib/images"
DEVICE = "/dev/video21"
TARGET_FRAMES = 20      # 目标保存帧数（只保存检测到角点 + 变化足够的）
MIN_INTERVAL = 0.8      # 相邻保存帧的最小时间间隔（秒）
MOVE_THRESH = 30.0      # 角点重心移动阈值（像素），超过才保存
MAX_RMS = 2.0           # RMS 阈值，超过则提示重新采集

# 准备 3D 坐标
objp = np.zeros((CHECKERBOARD[1] * CHECKERBOARD[0], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

obj_points = []
img_points = []
saved_count = 0
last_save_time = 0
last_centroid = None
frame_counter = 0

os.makedirs(SAVE_DIR, exist_ok=True)

# ================== 打开摄像头 ==================
print("[Calib] Opening camera...")
cap = cv2.VideoCapture(DEVICE)
if not cap.isOpened():
    print("[ERROR] Failed to open camera.")
    sys.exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[Calib] Camera opened: {w}x{h}")
print(f"[Calib] Target: {TARGET_FRAMES} valid frames (corners detected + angle changed)")
print("        Keep checkerboard in CENTER of frame.")
print("        Change distance and angle for each shot.")
print("        Press Ctrl+C to interrupt.\n")

# 预热几帧
for _ in range(5):
    cap.read()

# ================== 自动采集循环 ==================
try:
    while saved_count < TARGET_FRAMES:
        ret, frame = cap.read()
        if not ret:
            continue
        frame_counter += 1
        now = time.time()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)

        status = ""
        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            centroid = np.mean(corners2.reshape(-1, 2), axis=0)
            should_save = False
            move_dist = 0.0

            if saved_count == 0:
                should_save = True
                status = "FIRST frame saved"
            elif (now - last_save_time) >= MIN_INTERVAL:
                if last_centroid is not None:
                    move_dist = float(np.linalg.norm(centroid - last_centroid))
                    if move_dist >= MOVE_THRESH:
                        should_save = True
                        status = f"MOVED {move_dist:.1f}px | saved"
                    else:
                        status = f"moved {move_dist:.1f}px (< {MOVE_THRESH}) | skip"
                else:
                    should_save = True
                    status = "saved"

            if should_save:
                # 计算 move_dist 用于打印（在更新 last_centroid 之前）
                print_move = 0.0 if saved_count == 0 else move_dist
                fname = os.path.join(SAVE_DIR, f"frame_{saved_count:03d}.jpg")
                cv2.imwrite(fname, frame)
                obj_points.append(objp)
                img_points.append(corners2)
                last_save_time = now
                last_centroid = centroid.copy()
                saved_count += 1
                print(f"[Auto] {saved_count:2d}/{TARGET_FRAMES} | {status}")
            else:
                print(f"\r[Auto] {saved_count:2d}/{TARGET_FRAMES} | corners OK | {status}          ", end='', flush=True)
        else:
            print(f"\r[Auto] {saved_count:2d}/{TARGET_FRAMES} | NO corners detected | move checkerboard to CENTER |          ", end='', flush=True)

        # 每 ~3 秒保存一张预览图
        if saved_count > 0 and frame_counter % 90 == 0:
            preview = cv2.resize(frame, (480, 270))
            cv2.putText(preview, f"Saved:{saved_count}/{TARGET_FRAMES}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if found:
                cv2.drawChessboardCorners(preview, CHECKERBOARD, corners2 * 0.25, found)
            cv2.imwrite("/tmp/calib_preview.jpg", preview)

        time.sleep(0.03)

    print(f"\n[Calib] Collected {saved_count} valid frames. Starting calibration...")

except KeyboardInterrupt:
    print("\n[Calib] Interrupted by user.")
    cap.release()
    sys.exit(0)

cap.release()

# ================== 标定 ==================
print("[Calib] Running cv::calibrateCamera...")
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, (w, h), None, None
)

dist_flat = dist.ravel()

print("\n" + "="*50)
print("CALIBRATION RESULT")
print("="*50)
print(f"RMS reprojection error: {ret:.4f} px")
print(f"\nCamera Matrix (fx, fy, cx, cy):")
print(f"  fx = {K[0,0]:.2f}")
print(f"  fy = {K[1,1]:.2f}")
print(f"  cx = {K[0,2]:.2f}")
print(f"  cy = {K[1,2]:.2f}")
print(f"\nDistortion Coeffs (k1, k2, p1, p2, k3):")
print(f"  {dist_flat}")

# ================== 质量检查 ==================
quality_ok = True
issues = []

if ret > MAX_RMS:
    quality_ok = False
    issues.append(f"RMS = {ret:.2f} px > {MAX_RMS} px (images blurry or angles too similar)")

if abs(K[1,2] - h/2) > 150:
    quality_ok = False
    issues.append(f"cy = {K[1,2]:.1f} too far from center ({h/2:.0f}) (checkerboard mostly in upper/lower area)")

if abs(K[0,2] - w/2) > 150:
    quality_ok = False
    issues.append(f"cx = {K[0,2]:.1f} too far from center ({w/2:.0f})")

if abs(K[0,0] - K[1,1]) / max(K[0,0], K[1,1]) > 0.1:
    issues.append(f"fx/fy ratio = {K[0,0]/K[1,1]:.3f} (should be ~1.0)")

if abs(dist_flat[4]) > 1.0:
    issues.append(f"k3 = {dist_flat[4]:.3f} (|k3| > 1.0, possible overfit)")

print()
if quality_ok:
    print("[OK] Calibration quality looks good.")
else:
    print("[WARN] Calibration quality issues detected:")
    for issue in issues:
        print(f"       - {issue}")
    print("\n[SUGGESTION] Delete calib/images/*.jpg and re-run with better coverage.")

np.save("./calib/camera_matrix.npy", K)
np.save("./calib/dist_coeffs.npy", dist)
with open("./calib/camera_matrix.txt", "w") as f:
    f.write(f"fx = {K[0,0]:.4f}\n")
    f.write(f"fy = {K[1,1]:.4f}\n")
    f.write(f"cx = {K[0,2]:.4f}\n")
    f.write(f"cy = {K[1,2]:.4f}\n")
with open("./calib/dist_coeffs.txt", "w") as f:
    f.write(f"k1 = {dist_flat[0]:.6f}\n")
    f.write(f"k2 = {dist_flat[1]:.6f}\n")
    f.write(f"p1 = {dist_flat[2]:.6f}\n")
    f.write(f"p2 = {dist_flat[3]:.6f}\n")
    f.write(f"k3 = {dist_flat[4]:.6f}\n")

print("\n[Calib] Saved to calib/camera_matrix.* and calib/dist_coeffs.*")

# 可视化去畸变效果
sample = cv2.imread(os.path.join(SAVE_DIR, "frame_000.jpg"))
if sample is not None:
    undist = cv2.undistort(sample, K, dist, None, K)
    compare = np.hstack((cv2.resize(sample, (640,360)), cv2.resize(undist, (640,360))))
    cv2.putText(compare, "Original", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    cv2.putText(compare, "Undistorted", (660, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    cv2.imwrite("calib_result.png", compare)
    print("[Calib] Saved comparison to calib_result.png")

print("\n[Calib] Done.")
if quality_ok:
    print("       You can now copy fx/fy/cx/cy into src/rga_npu.cpp CAMERA_MATRIX.")
