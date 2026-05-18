#!/usr/bin/env python3
"""
USB Camera3 相机标定脚本（RK3588 全自动无头模式）

用法:
  python3 calibrate.py 9 6

自动流程：
  1. 持续检测棋盘格角点
  2. 角点稳定且视角变化足够大时，自动保存帧
  3. 收集够 TARGET_FRAMES 张后自动标定并输出结果
  4. 可随时按 Ctrl+C 中断
"""

import cv2
import numpy as np
import os
import sys
import time

# ================== 配置 ==================
CHECKERBOARD = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) >= 3 else (9, 6)
SQUARE_SIZE = 25.0
SAVE_DIR = "./calib_images"
DEVICE = "/dev/video21"
TARGET_FRAMES = 15      # 自动保存的目标帧数
MIN_INTERVAL = 0.8      # 相邻保存帧的最小时间间隔（秒）
MOVE_THRESH = 15.0      # 角点重心移动阈值（像素），超过才保存

# 准备 3D 坐标
objp = np.zeros((CHECKERBOARD[1] * CHECKERBOARD[0], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

obj_points = []
img_points = []
saved_count = 0
last_save_time = 0
last_centroid = None

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
print(f"[Calib] AUTO mode: will save {TARGET_FRAMES} frames automatically.")
print("        Show checkerboard from different angles.")
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

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK)

        now = time.time()
        should_save = False

        if found:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

            # 计算角点重心
            centroid = np.mean(corners2.reshape(-1, 2), axis=0)

            if saved_count == 0:
                # 第一帧：只要有角点就保存
                should_save = True
            elif (now - last_save_time) >= MIN_INTERVAL:
                if last_centroid is not None:
                    move_dist = np.linalg.norm(centroid - last_centroid)
                    if move_dist >= MOVE_THRESH:
                        should_save = True
                else:
                    should_save = True

            if should_save:
                fname = os.path.join(SAVE_DIR, f"frame_{saved_count:03d}.jpg")
                cv2.imwrite(fname, frame)
                obj_points.append(objp)
                img_points.append(corners2)
                last_save_time = now
                last_centroid = centroid
                saved_count += 1
                print(f"[Auto] Saved {saved_count}/{TARGET_FRAMES} | move={np.linalg.norm(centroid - (last_centroid if last_centroid is not None else centroid)):.1f}px")
            else:
                # 打印状态但不保存
                move_str = ""
                if last_centroid is not None:
                    move_dist = np.linalg.norm(centroid - last_centroid)
                    move_str = f" move={move_dist:.1f}px"
                print(f"\r[Auto] {saved_count}/{TARGET_FRAMES} | corners OK{move_str} | waiting...", end='', flush=True)
        else:
            print(f"\r[Auto] {saved_count}/{TARGET_FRAMES} | no corners      | waiting...", end='', flush=True)

        # 每 30 帧保存一张预览图供外部查看
        if saved_count > 0 and (int(now * 10) % 30 == 0):
            preview = cv2.resize(frame, (480, 270))
            cv2.putText(preview, f"Saved:{saved_count}/{TARGET_FRAMES}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if found:
                cv2.drawChessboardCorners(preview, CHECKERBOARD, corners2 * 0.25, found)
            cv2.imwrite("/tmp/calib_preview.jpg", preview)

        time.sleep(0.03)  # ~30fps

    print(f"\n[Calib] Collected {saved_count} frames. Starting calibration...")

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
print(f"  {dist.ravel()}")

np.save("camera_matrix.npy", K)
np.save("dist_coeffs.npy", dist)
with open("camera_matrix.txt", "w") as f:
    f.write(f"fx = {K[0,0]:.4f}\n")
    f.write(f"fy = {K[1,1]:.4f}\n")
    f.write(f"cx = {K[0,2]:.4f}\n")
    f.write(f"cy = {K[1,2]:.4f}\n")
with open("dist_coeffs.txt", "w") as f:
    f.write(f"k1 = {dist[0,0]:.6f}\n")
    f.write(f"k2 = {dist[0,1]:.6f}\n")
    f.write(f"p1 = {dist[0,2]:.6f}\n")
    f.write(f"p2 = {dist[0,3]:.6f}\n")
    f.write(f"k3 = {dist[0,4]:.6f}\n")

print("\n[Calib] Saved to camera_matrix.npy/txt and dist_coeffs.npy/txt")

# 可视化去畸变效果
sample = cv2.imread(os.path.join(SAVE_DIR, "frame_000.jpg"))
if sample is not None:
    undist = cv2.undistort(sample, K, dist, None, K)
    compare = np.hstack((cv2.resize(sample, (640,360)), cv2.resize(undist, (640,360))))
    cv2.putText(compare, "Original", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
    cv2.putText(compare, "Undistorted", (660, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    cv2.imwrite("calib_result.png", compare)
    print("[Calib] Saved comparison to calib_result.png")

print("\n[Calib] Done. Copy the fx/fy values into rga_npu.cpp CAMERA_MATRIX.")
