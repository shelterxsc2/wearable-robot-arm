# -*- coding: utf-8 -*-
"""
ELF 视觉链路移植 Demo（OpenVINO 版）
====================================
完全照抄 /home/time/work/elf_info/wearable-robot-arm/src/rga_npu.cpp 中
imu-victor-hat 分支的视觉 pipeline 逻辑：

    摄像头 → Body (yolo26s-pose) → Face ROI 估算 →
    Face Landmark 468 → 12 点 PnP → 3D 立方体/坐标轴 overlay →
    本地显示 (+ RTMP/RTSP 推流选择)

模型来源：/home/time/work/mymodel/
    - yolo26s-pose.onnx          (人体 17 COCO 关键点)
    - face_landmark_468.onnx     (MediaPipe Face Mesh 468 点)
"""
import os
import sys
import time
import math
import statistics
import socket
import struct
import subprocess
import threading
import argparse
from collections import deque

import cv2
import numpy as np
import openvino as ov

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ========== 模型路径 ==========
# YOLOv8n-pose OpenVINO IR（Ultralytics 官方模型，输出 [1,56,8400]）
BODY_MODEL_PATH = "/home/time/work/trial0/models/yolov8s-pose_openvino_model/yolov8s-pose.xml"
FACE_LM_MODEL_PATH = "/home/time/work/mymodel/face_landmark_468.onnx"
HAND_LM_MODEL_PATH = "/home/time/work/mymodel/openvino_pipeline/models/onnx/hand_landmarks_detector.onnx"
HAND_KPT_CLS_MODEL_PATH = "/home/time/work/mymodel/openvino_pipeline/model/keypoint_classifier/keypoint_classifier.onnx"
HAND_KPT_CLS_LABEL_PATH = "/home/time/work/mymodel/openvino_pipeline/model/keypoint_classifier/keypoint_classifier_label.csv"
RULE_MODEL_PATH = "/home/time/work/mymodel/rule_engine_v2.onnx"

# ========== 语音链路配置（sherpa 项目） ==========
SHERPA_MODEL_DIR = "/home/time/work/sherpa/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"
SHERPA_KEYWORDS_FILE = "/home/time/work/trial0/voice/keywords.txt"
SHERPA_VAD_MODEL = "/home/time/work/sherpa/models/silero_vad.onnx"

# ========== 摄像头 / 流配置 ==========
DEVICE_ID = "device-002"
RTMP_URL = f"rtmp://47.93.162.124:1935/live/{DEVICE_ID}"
# NOTE: this demo only pushes RTMP; WebSocket cloud heartbeat is implemented
# in demos/camera_demo_lowlatency.py. WS_URL is kept here for documentation
# but is not used by this script.
# WS_URL = f"ws://47.93.162.124/ws?deviceId={DEVICE_ID}"

CAM_WIDTH = 1920
CAM_HEIGHT = 1080
CAM_FPS_TARGET = 30

BODY_INPUT_SIZE = 640
FACE_LM_INPUT_SIZE = 192
HAND_LM_INPUT_SIZE = 224
OBJ_THRESHOLD = 0.25
NMS_THRESHOLD = 0.45
KPT_CONF_THRESHOLD = 0.30
FACE_LM_CONF_THRESHOLD = 0.30
HAND_LM_CONF_THRESHOLD = 0.50
HAND_CROP_MIN_PRESENCE = 0.70  # 手部 presence 低于此值时不进行裁减/绘制

# 模型在 CPU 还是 GPU 上跑：
#   yolov8n-pose 在 GPU 上推理更快
#   face_landmark_468.onnx 在 GPU 上正常且更快
#   rule_engine_v2.onnx 按用户要求放在 GPU 上
#   hand_landmarks_detector.onnx 按用户要求放在 GPU 上
BODY_DEVICE = "GPU"
FACE_DEVICE = "GPU"
HAND_DEVICE = "GPU"
RULE_DEVICE = "GPU"
DEBUG = False
SELFIE_MODE = False

# 摄像头是否上下颠倒安装（某些 Realtek USB 摄像头出厂即倒置）
# 设为 True 则在预处理前垂直翻转
FLIP_VERTICAL = False

# Arrow Lake 需要显式指定 iHD 驱动
if 'LIBVA_DRIVER_NAME' not in os.environ:
    os.environ['LIBVA_DRIVER_NAME'] = 'iHD'

# ========== PnP 3D 模板（从 elf rga_npu.cpp 照抄）==========
# face_landmark_468 -> MediaPipe canonical face geometry 的 12 点 PnP
FACE_LM_12_IDS = [
    133,  # 0: right eye inner
    263,  # 1: left eye outer
    1,    # 2: nose tip
    61,   # 3: right mouth corner
    291,  # 4: left mouth corner
    152,  # 5: chin
    33,   # 6: right eye outer
    362,  # 7: left eye inner
    48,   # 8: right nose
    278,  # 9: left nose
    105,  # 10: right brow
    334,  # 11: left brow
]

"""
Source: MediaPipe canonical_face_model.obj, vertex indices above.
Canonical coordinates are centimeters with +Y up and +Z toward the face front.
This project uses +Y down and face-front toward -Z, so vertices are transformed
as (x, -y, -z), scaled to millimeters, then translated to keep landmark 1 at
the previous nose anchor (0, -5, -90).
"""
FACE_LM_12_3D = np.array([
    [-18.564320, -42.121100, -52.823000],   # 133
    [ 44.458590, -42.908560, -46.978180],   # 263
    [  0.000000,  -5.000000, -90.000000],   # 1
    [-24.562060,  27.157560, -58.082800],   # 61
    [ 24.562060,  27.157560, -58.082800],   # 291
    [  0.000000,  77.765130, -57.888880],   # 152
    [-44.458590, -42.908560, -46.978180],   # 33
    [ 18.564320, -42.121100, -52.823000],   # 362
    [-16.086350,  -6.843490, -73.385890],   # 48
    [ 16.086350,  -6.843490, -73.385890],   # 278
    [-39.865620, -67.363520, -59.907110],   # 105
    [ 39.865620, -67.363520, -59.907110],   # 334
], dtype=np.float32)

# RuleEngine 关键点映射（来自 elf rga_npu.cpp）
# COCO 17 点按观察者视角交换左右，再追加 FaceMesh 嘴角/下巴 3 点 = 20 点
RULE_KPT_LR_SWAP = [
    (1, 2),   # left_eye <-> right_eye
    (3, 4),   # left_ear <-> right_ear
    (5, 6),   # left_shoulder <-> right_shoulder
    (7, 8),   # left_elbow <-> right_elbow
    (9, 10),  # left_wrist <-> right_wrist
    (11, 12), # left_hip <-> right_hip
    (13, 14), # left_knee <-> right_knee
    (15, 16), # left_ankle <-> right_ankle
]
RULE_FACE_LEFT_MOUTH = 291
RULE_FACE_RIGHT_MOUTH = 61
RULE_FACE_CHIN = 152

# USB Camera3 (Realtek) calibrated 2026-04-26
# RMS error: 0.9759 px | Checkerboard 9x6 corners @ 25mm
CAMERA_MATRIX = np.array([
    [689.58,   0.0,    982.87],
    [  0.0,  686.99,  394.59],
    [  0.0,    0.0,     1.0],
], dtype=np.float32)

DIST_COEFFS = np.array([
    -0.140459, 0.270074, 0.000120, 0.003055, -0.395587,
], dtype=np.float32)

# COCO keypoint 名称（用于调试）
COCO_KPT_NAMES = [
    "nose", "left_eye", "right_eye", "left_ear", "right_ear",
    "left_shoulder", "right_shoulder", "left_elbow", "right_elbow",
    "left_wrist", "right_wrist", "left_hip", "right_hip",
    "left_knee", "right_knee", "left_ankle", "right_ankle",
]


# ========== 1-Euro Filter（与 elf 一致）==========
class OneEuroFilter:
    def __init__(self, freq=30.0, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = 0.0

    def smoothing_factor(self, cutoff):
        r = 2.0 * math.pi * cutoff / self.freq
        return r / (r + 1.0)

    def exponential_smoothing(self, alpha, x, x_prev):
        return alpha * x + (1.0 - alpha) * x_prev

    def reset(self, x0):
        self.x_prev = x0
        self.dx_prev = 0.0

    def filter(self, x):
        if self.x_prev is None:
            self.x_prev = x
            return x
        dx = (x - self.x_prev) * self.freq
        a_d = self.smoothing_factor(self.d_cutoff)
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self.smoothing_factor(cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat


# ========== 目标人物 Tracker（简化版，保留 elf 核心逻辑）==========
def nms_pose(dets, iou_thresh=0.45):
    """对 body detections 做 NMS，保留最高置信度且 IoU 较低的框。"""
    if not dets:
        return []
    boxes = np.array([[d['x1'], d['y1'], d['x2'], d['y2']] for d in dets], dtype=np.float32)
    scores = np.array([d['score'] for d in dets], dtype=np.float32)
    indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), score_threshold=0.0, nms_threshold=iou_thresh)
    if len(indices) == 0:
        return []
    indices = indices.flatten() if hasattr(indices, 'flatten') else list(indices)
    return [dets[i] for i in indices]


# ========== 性能统计（从 camera_demo_lowlatency.py 移植）==========
class PerformanceProfiler:
    def __init__(self, report_interval=30):
        self.report_interval = report_interval
        self.records = []
        self.frame_idx = 0

    def record(self, data):
        self.records.append(data)
        self.frame_idx += 1
        if self.frame_idx % self.report_interval == 0:
            self.print_report()

    def print_report(self):
        recent = self.records[-self.report_interval:]

        def avg(key, multiply=1000.0):
            vals = [r.get(key, 0) * multiply for r in recent if key in r]
            return statistics.mean(vals) if vals else 0

        def max_val(key, multiply=1000.0):
            vals = [r.get(key, 0) * multiply for r in recent if key in r]
            return max(vals) if vals else 0

        print("\n" + "=" * 60)
        print(f"[ELF Pipeline] last {len(recent)} frames stats")
        print("-" * 60)
        print(f"  total       : avg={avg('total'):7.2f}ms  max={max_val('total'):7.2f}ms")
        print(f"  camera_io   : avg={avg('camera_io'):7.2f}ms")
        print(f"  body_total  : avg={avg('body_total'):7.2f}ms")
        print(f"    preproc   : avg={avg('body_pre'):7.2f}ms")
        print(f"    inference : avg={avg('body_infer'):7.2f}ms")
        print(f"    postproc  : avg={avg('body_post'):7.2f}ms")
        print(f"  face_infer  : avg={avg('face_infer'):7.2f}ms")
        print(f"  pnp_compute : avg={avg('pnp'):7.2f}ms")
        print(f"  rule_engine : avg={avg('rule'):7.2f}ms")
        print(f"  hand_infer  : avg={avg('hand_infer'):7.2f}ms")
        print(f"  stream_write: avg={avg('write'):7.2f}ms")
        print(f"  loop_fps    : avg={avg('loop_fps', 1.0):7.1f}")
        print("=" * 60)


class FaceTracker:
    WIN_SIZE = 10
    FACE_SIZE_MAX_LIFE = 5

    def __init__(self):
        self.history = deque(maxlen=self.WIN_SIZE)
        self.tracked = False
        self.last_cx = 0.0
        self.last_cy = 0.0
        self.last_face_w = 0.0
        self.last_face_h = 0.0
        self.face_size_life = 0
        self.prev_roi_size = 0.0

    def update(self, dets, img_w, img_h, center_large=False):
        """返回最佳 detection 的下标，未锁定返回 -1
        优先选择有有效人脸关键点（鼻子+眼）的检测框，避免选中背景/天花板。
        """
        short_edge = min(img_w, img_h)
        max_drift = short_edge * 0.25

        def has_face(det):
            kps = det['kps']
            return (kps[0]['visibility'] > KPT_CONF_THRESHOLD and
                    (kps[1]['visibility'] > KPT_CONF_THRESHOLD or
                     kps[2]['visibility'] > KPT_CONF_THRESHOLD))

        def det_area(det):
            return max(0, det['x2'] - det['x1']) * max(0, det['y2'] - det['y1'])

        best = None
        best_score = 1e9
        best_idx = -1

        if DEBUG and dets:
            print(f"[Tracker] tracked={self.tracked} candidates={len(dets)}")
            for i, det in enumerate(dets):
                cx = (det['x1'] + det['x2']) * 0.5
                cy = (det['y1'] + det['y2']) * 0.5
                has = has_face(det)
                print(f"  [{i}] conf={det['score']:.3f} cx={cx:.0f} cy={cy:.0f} has_face={has}")

        for i, det in enumerate(dets):
            cx = (det['x1'] + det['x2']) * 0.5
            cy = (det['y1'] + det['y2']) * 0.5

            face_ok = has_face(det)
            if center_large:
                if not face_ok:
                    continue
                center_dist = math.hypot(cx - img_w * 0.5, cy - img_h * 0.5)
                half_diagonal = max(1.0, math.hypot(img_w, img_h) * 0.5)
                center_norm = center_dist / half_diagonal
                area_norm = math.sqrt(det_area(det) / max(1.0, img_w * img_h))
                score = 0.60 * center_norm - 0.40 * area_norm
            elif SELFIE_MODE:
                # 自拍/操作者模式：选面积最大且带人脸关键点的框
                face_bonus = 0.0 if face_ok else short_edge * 0.5
                score = -math.sqrt(det_area(det)) * 2.0 + face_bonus
            elif self.tracked:
                d = math.hypot(cx - self.last_cx, cy - self.last_cy)
                if d > max_drift:
                    continue
                # 跟踪中仍优先保留带人脸关键点的框，防止漂到背景
                face_bonus = 0.0 if face_ok else short_edge * 0.3
                score = d + face_bonus
            else:
                # 未跟踪时：优先选有鼻子+眼的，再按到中心距离排序
                center_dist = math.hypot(cx - img_w * 0.5, cy - img_h * 0.5)
                face_bonus = 0.0 if face_ok else short_edge * 0.5
                # 适度偏好面积更大的框（真人通常比背景海报大）
                area_penalty = -math.sqrt(det_area(det)) * 0.05
                score = center_dist + face_bonus + area_penalty

            if score < best_score:
                best_score = score
                best = det
                best_idx = i

        self.history.append({
            'cx': (best['x1'] + best['x2']) * 0.5 if best else 0.0,
            'cy': (best['y1'] + best['y2']) * 0.5 if best else 0.0,
            'area': (best['x2'] - best['x1']) * (best['y2'] - best['y1']) if best else 0.0,
            'valid': best is not None,
        })

        valid_records = [r for r in self.history if r['valid']]
        if len(valid_records) >= 5 and best is not None:
            self.last_cx = sum(r['cx'] for r in valid_records) / len(valid_records)
            self.last_cy = sum(r['cy'] for r in valid_records) / len(valid_records)
            self.tracked = True
            if DEBUG:
                print(f"[Tracker] LOCK idx={best_idx} conf={best['score']:.3f} cx={self.last_cx:.0f} cy={self.last_cy:.0f}")
            return best_idx
        else:
            if DEBUG and best is not None:
                print(f"[Tracker] warming up idx={best_idx} valid={len(valid_records)}")
            self.tracked = False
            return -1

    def update_face_size(self, w, h):
        alpha = 0.5
        if self.last_face_w > 0 and self.last_face_h > 0:
            self.last_face_w = alpha * w + (1.0 - alpha) * self.last_face_w
            self.last_face_h = alpha * h + (1.0 - alpha) * self.last_face_h
        else:
            self.last_face_w = w
            self.last_face_h = h
        self.face_size_life = self.FACE_SIZE_MAX_LIFE

    def has_face_size(self):
        return self.face_size_life > 0 and self.last_face_w > 32.0 and self.last_face_h > 32.0

    def get_roi_size(self):
        if not self.has_face_size():
            return 0.0
        return max(self.last_face_w, self.last_face_h) * 1.3 + 50.0


# ========== RTMP / RTSP 探测与初始化 ==========
def probe_rtmp_server(rtmp_url, timeout_ms=3000):
    """从 elf stream_manager.cpp 的 probe_rtmp_server 翻译而来"""
    if rtmp_url.startswith("rtmp://"):
        p = rtmp_url[7:]
    elif rtmp_url.startswith("rtmps://"):
        p = rtmp_url[8:]
    else:
        print(f"[Probe] Invalid RTMP URL: {rtmp_url}")
        return False

    slash = p.find('/')
    colon = p.find(':')
    if colon != -1 and (slash == -1 or colon < slash):
        host = p[:colon]
        port = int(p[colon + 1:slash if slash != -1 else len(p)])
    else:
        host = p[:slash] if slash != -1 else p
        port = 1935

    print(f"[Probe] RTMP server {host}:{port} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout_ms / 1000.0)
    try:
        sock.connect((host, port))
        print(f"[Probe] RTMP server reachable.")
        return True
    except Exception as e:
        print(f"[Probe] RTMP server NOT reachable: {e}")
        return False
    finally:
        sock.close()


def build_ffmpeg_cmd(rtmp_url, width, height, fps=15):
    """构建 RTMP 推流 ffmpeg 命令（参考 camera_demo_lowlatency.py 的 VAAPI 分支）"""
    encoder = 'h264_vaapi'
    cmd = [
        'ffmpeg', '-y', '-f', 'rawvideo', '-vcodec', 'rawvideo',
        '-pix_fmt', 'bgr24', '-s', f'{width}x{height}', '-r', str(fps),
        '-thread_queue_size', '512', '-i', '-',
        '-vaapi_device', '/dev/dri/renderD128',
        '-vf', 'format=nv12,hwupload',
        '-c:v', encoder, '-b:v', '4M', '-maxrate', '4M', '-g', str(fps),
        '-pix_fmt', 'nv12', '-fps_mode', 'passthrough',
        '-f', 'flv', rtmp_url,
    ]
    return cmd


class Streamer:
    """极简推流管理器：优先 RTMP，不可达则本地 RTSP 占位"""
    def __init__(self, rtmp_url, width, height, fps=15):
        self.rtmp_url = rtmp_url
        self.width = width
        self.height = height
        self.fps = fps
        self.proc = None
        self.stream_type = None  # 'rtmp' or None

    def init_stream(self):
        if probe_rtmp_server(self.rtmp_url, 3000):
            cmd = build_ffmpeg_cmd(self.rtmp_url, self.width, self.height, self.fps)
            print(f"[Streamer] Starting RTMP: {' '.join(cmd)}")
            try:
                self.proc = subprocess.Popen(
                    cmd, stdin=subprocess.PIPE,
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
                )
                self.stream_type = 'rtmp'
                return True
            except Exception as e:
                print(f"[Streamer] ffmpeg start failed: {e}")
                self.proc = None
        else:
            print("[Streamer] RTMP unreachable. RTSP fallback not available in Python env.")
            print("[Streamer] Will show local preview only.")
        return False

    def write(self, frame):
        if self.proc is None or self.proc.stdin is None:
            return
        try:
            self.proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            print("[Streamer] ffmpeg broken pipe, stopping stream.")
            self.stop()

    def stop(self):
        if self.proc:
            try:
                self.proc.stdin.close()
                self.proc.wait(timeout=2)
            except Exception:
                self.proc.kill()
            self.proc = None


# ========== 模型加载 ==========
class ElfPipeline:
    def __init__(self):
        print("=" * 60)
        print("ELF Pipeline (OpenVINO)")
        print("=" * 60)
        self.core = ov.Core()
        print(f"OpenVINO version: {ov.__version__}")
        print(f"Available devices: {self.core.available_devices}")

        print(f"\n[1/2] Loading Body model: {BODY_MODEL_PATH}")
        self.body_model = self.core.read_model(BODY_MODEL_PATH)
        try:
            in_name = self.body_model.inputs[0].get_any_name()
        except Exception:
            in_name = '<index:0>'
        try:
            out_name = self.body_model.outputs[0].get_any_name()
        except Exception:
            out_name = '<index:0>'
        print(f"      Input:  {in_name} {self.body_model.inputs[0].get_shape()}")
        print(f"      Output: {out_name} {self.body_model.outputs[0].get_shape()}")
        self.body_compiled = self.core.compile_model(self.body_model, BODY_DEVICE)
        # 兼容没有 tensor name 的 OpenVINO IR（如 yolo26n-pose）
        try:
            self.body_input_name = self.body_compiled.inputs[0].get_any_name()
        except Exception:
            self.body_input_name = 0
        try:
            self.body_output_name = self.body_compiled.outputs[0].get_any_name()
        except Exception:
            self.body_output_name = 0

        print(f"\n[2/3] Loading Face Landmark model: {FACE_LM_MODEL_PATH}")
        self.face_lm_model = self.core.read_model(FACE_LM_MODEL_PATH)
        print(f"      Input:  {self.face_lm_model.inputs[0].get_any_name()} {self.face_lm_model.inputs[0].get_shape()}")
        for o in self.face_lm_model.outputs:
            print(f"      Output: {o.get_any_name()} {o.get_shape()}")
        self.face_lm_compiled = self.core.compile_model(self.face_lm_model, FACE_DEVICE)
        self.face_lm_input_name = self.face_lm_compiled.inputs[0].get_any_name()
        self.face_lm_output_name = self.face_lm_compiled.outputs[0].get_any_name()

        print(f"\n[3/4] Loading Hand Landmark model: {HAND_LM_MODEL_PATH}")
        self.hand_lm_model = self.core.read_model(HAND_LM_MODEL_PATH)
        print(f"      Input:  {self.hand_lm_model.inputs[0].get_any_name()} {self.hand_lm_model.inputs[0].get_shape()}")
        for o in self.hand_lm_model.outputs:
            print(f"      Output: {o.get_any_name()} {o.get_shape()}")
        self.hand_lm_compiled = self.core.compile_model(self.hand_lm_model, HAND_DEVICE)
        self.hand_lm_input_name = self.hand_lm_compiled.inputs[0].get_any_name()
        self.hand_lm_output_names = [o.get_any_name() for o in self.hand_lm_compiled.outputs]

        print(f"\n[4/4] Loading Hand Keypoint Classifier model: {HAND_KPT_CLS_MODEL_PATH}")
        self.hand_kpt_cls_model = self.core.read_model(HAND_KPT_CLS_MODEL_PATH)
        print(f"      Input:  {self.hand_kpt_cls_model.inputs[0].get_any_name()} {self.hand_kpt_cls_model.inputs[0].get_shape()}")
        print(f"      Output: {self.hand_kpt_cls_model.outputs[0].get_any_name()} {self.hand_kpt_cls_model.outputs[0].get_shape()}")
        self.hand_kpt_cls_compiled = self.core.compile_model(self.hand_kpt_cls_model, HAND_DEVICE)
        self.hand_kpt_cls_input_name = self.hand_kpt_cls_compiled.inputs[0].get_any_name()
        self.hand_kpt_cls_output_name = self.hand_kpt_cls_compiled.outputs[0].get_any_name()
        with open(HAND_KPT_CLS_LABEL_PATH, 'r', encoding='utf-8-sig') as f:
            self.hand_kpt_cls_labels = [line.strip() for line in f if line.strip()]
        print(f"      Labels: {self.hand_kpt_cls_labels}")

        print(f"\n[Rule] Loading Rule Engine model: {RULE_MODEL_PATH}")
        self.rule_model = self.core.read_model(RULE_MODEL_PATH)
        for i in self.rule_model.inputs:
            print(f"      Input:  {i.get_any_name()} {i.get_partial_shape()}")
        for o in self.rule_model.outputs:
            print(f"      Output: {o.get_any_name()} {o.get_partial_shape()}")
        self.rule_compiled = self.core.compile_model(self.rule_model, RULE_DEVICE)
        self.rule_input_names = []
        for i in self.rule_compiled.inputs:
            try:
                self.rule_input_names.append(i.get_any_name())
            except Exception:
                self.rule_input_names.append(len(self.rule_input_names))
        try:
            self.rule_output_name = self.rule_compiled.outputs[0].get_any_name()
        except Exception:
            self.rule_output_name = 0

        print("\n[OK] Models loaded.")
        print("=" * 60)

    # ------------------------------------------------------------------
    # Body Preprocess
    # ------------------------------------------------------------------
    def preprocess_body(self, frame):
        """
        YOLOv8n-pose 需要：
          - RGB 顺序
          - 数值范围 [0, 1]
          - NCHW [1,3,640,640]
          - Letterbox 保持长宽比
        """
        h, w = frame.shape[:2]
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        scale = BODY_INPUT_SIZE / max(h, w)
        new_w = int(w * scale)
        new_h = int(h * scale)
        resized = cv2.resize(img, (new_w, new_h))

        letterboxed = np.zeros((BODY_INPUT_SIZE, BODY_INPUT_SIZE, 3), dtype=np.float32)
        x_off = (BODY_INPUT_SIZE - new_w) // 2
        y_off = (BODY_INPUT_SIZE - new_h) // 2
        letterboxed[y_off:y_off + new_h, x_off:x_off + new_w] = resized

        inp = np.transpose(letterboxed, (2, 0, 1))[np.newaxis, ...]
        return inp, scale, x_off, y_off

    # ------------------------------------------------------------------
    # Body Postprocess
    # ------------------------------------------------------------------
    def postprocess_body(self, output, img_w, img_h, scale, x_off, y_off):
        """
        YOLOv8n-pose OpenVINO 输出 shape: [1, 56, 8400]
        每列 = [cx, cy, w, h, conf, kpt0_x, kpt0_y, kpt0_conf, ..., kpt16_x, kpt16_y, kpt16_conf]
        坐标在 640x640 letterbox 空间，需映射回原图。
        """
        dets = []
        arr = output[0].T  # [8400, 56]
        for i in range(arr.shape[0]):
            conf = arr[i, 4]
            if conf < OBJ_THRESHOLD:
                continue

            cx = arr[i, 0]
            cy = arr[i, 1]
            bw = arr[i, 2]
            bh = arr[i, 3]
            x1_640 = cx - bw * 0.5
            y1_640 = cy - bh * 0.5
            x2_640 = cx + bw * 0.5
            y2_640 = cy + bh * 0.5

            # 映射回原图
            x1 = (x1_640 - x_off) / scale
            y1 = (y1_640 - y_off) / scale
            x2 = (x2_640 - x_off) / scale
            y2 = (y2_640 - y_off) / scale

            kps = []
            for k in range(17):
                kx = arr[i, 5 + k * 3]
                ky = arr[i, 6 + k * 3]
                kv = arr[i, 7 + k * 3]
                kps.append({
                    'x': (kx - x_off) / scale,
                    'y': (ky - y_off) / scale,
                    'visibility': kv,
                })

            box_w = max(0.0, x2 - x1)
            box_h = max(0.0, y2 - y1)
            if box_w < 10 or box_h < 10:
                continue
            # 过滤明显不合理的宽高比
            aspect = box_h / box_w
            if aspect < 0.25 or aspect > 4.0:
                continue
            # 过滤有效关键点过少的检测框
            visible_kpts = sum(1 for kp in kps if kp['visibility'] > KPT_CONF_THRESHOLD)
            if visible_kpts < 3:
                continue

            dets.append({
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                'score': conf,
                'kps': kps,
            })

        # NMS + 按置信度排序
        dets = nms_pose(dets, iou_thresh=NMS_THRESHOLD)
        dets.sort(key=lambda d: -d['score'])
        return dets

    # ------------------------------------------------------------------
    # Face Landmark Preprocess / Inference
    # ------------------------------------------------------------------
    def infer_face_landmarks(self, frame, roi_x, roi_y, roi_w, roi_h):
        """
        从原图裁剪 ROI，resize 到 192x192，跑 face_landmark_468.onnx。
        返回：在 ROI 内 192x192 坐标的 468 个 (x,y,z) 或 None。
        """
        h, w = frame.shape[:2]
        roi_x = max(0, roi_x)
        roi_y = max(0, roi_y)
        roi_w = min(roi_w, w - roi_x)
        roi_h = min(roi_h, h - roi_y)
        if roi_w < 32 or roi_h < 32:
            return None

        crop = frame[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
        crop_rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        resized = cv2.resize(crop_rgb, (FACE_LM_INPUT_SIZE, FACE_LM_INPUT_SIZE))
        inp = np.expand_dims(resized, axis=0)  # [1,192,192,3]

        out = self.face_lm_compiled({self.face_lm_input_name: inp})
        lm = out[self.face_lm_output_name][0].reshape(-1, 3)  # [468, 3]
        return lm


# ========== PnP 与绘制 ==========
class PoseEstimator:
    def __init__(self):
        self.have_prev_pose = False
        self.prev_rvec = None
        self.prev_tvec = None
        self.pose_filter_init = False

        # 与 elf 一致：fl_rx..fl_tz 用于 OneEuro 滤波（当前禁用 PNP_USE_FILTER=0）
        self.flt_rx = OneEuroFilter(15.0, 2.5, 0.08, 1.0)
        self.flt_ry = OneEuroFilter(15.0, 2.5, 0.08, 1.0)
        self.flt_rz = OneEuroFilter(15.0, 2.5, 0.08, 1.0)
        self.flt_tx = OneEuroFilter(15.0, 2.5, 0.08, 1.0)
        self.flt_ty = OneEuroFilter(15.0, 2.5, 0.08, 1.0)
        self.flt_tz = OneEuroFilter(15.0, 2.5, 0.08, 1.0)

        # 固定安装校正矩阵：R_mount = Ry(-14°) * Rx(-0.10 rad)
        mp = -0.10
        my = -14.0 * math.pi / 180.0
        cp, sp = math.cos(mp), math.sin(mp)
        cy, sy = math.cos(my), math.sin(my)
        R_pitch = np.array([
            [1.0, 0.0, 0.0],
            [0.0,  cp, -sp],
            [0.0,  sp,  cp],
        ], dtype=np.float64)
        R_yaw = np.array([
            [ cy, 0.0,  sy],
            [0.0, 1.0, 0.0],
            [-sy, 0.0,  cy],
        ], dtype=np.float64)
        self.R_mount = R_yaw @ R_pitch

    def estimate_and_draw(self, frame, face_lm_2d):
        """
        12 点 PnP，绘制坐标轴与立方体，返回 (success, info_dict)。
        完全照抄 elf rga_npu.cpp 的 estimate_and_draw_pose 核心逻辑。
        """
        h, w = frame.shape[:2]

        # 根据当前分辨率动态缩放内参（基准标定分辨率 1920x1080）
        calib_w, calib_h = 1920.0, 1080.0
        scale_x = w / calib_w
        scale_y = h / calib_h
        K = CAMERA_MATRIX.copy()
        K[0, 0] *= scale_x
        K[1, 1] *= scale_y
        K[0, 2] *= scale_x
        K[1, 2] *= scale_y
        K = K.astype(np.float64)
        D = DIST_COEFFS.astype(np.float64)

        image_points = np.array(face_lm_2d, dtype=np.float32)
        object_points = FACE_LM_12_3D.astype(np.float32)

        if len(image_points) < 4 or len(image_points) != len(object_points):
            self.have_prev_pose = False
            return False, None

        rvec = self.prev_rvec.copy() if self.have_prev_pose else None
        tvec = self.prev_tvec.copy() if self.have_prev_pose else None

        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, K, D,
            rvec, tvec, self.have_prev_pose,
            cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            self.have_prev_pose = False
            return False, None

        # 重投影误差检查
        reproj_pts, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
        reproj_error = 0.0
        for i in range(len(image_points)):
            dx = image_points[i, 0] - reproj_pts[i, 0, 0]
            dy = image_points[i, 1] - reproj_pts[i, 0, 1]
            reproj_error += math.hypot(dx, dy)
        reproj_error /= len(image_points)

        if reproj_error > 50.0:
            print(f"[Pose] Bad frame skipped, reprojection error={reproj_error:.1f}px")
            self.have_prev_pose = False
            return False, None

        # 硬丢弃：tz < 0 表示人脸在相机后方，是镜像解
        if tvec[2, 0] < 0:
            print(f"[Pose] Mirror solution detected (tz={tvec[2,0]:.1f}), dropped")
            self.have_prev_pose = False
            return False, None

        # rvec 连续性检查：与上一帧旋转角差 > 60° 则丢弃
        if self.have_prev_pose:
            R_curr, _ = cv2.Rodrigues(rvec)
            R_prev, _ = cv2.Rodrigues(self.prev_rvec)
            R_rel = R_curr @ R_prev.T
            trace = np.trace(R_rel)
            cos_half = min(1.0, max(-1.0, (trace - 1.0) / 2.0))
            angle_diff = math.acos(cos_half)
            if angle_diff > math.pi / 3.0:
                print(f"[Pose] Jump detected ({math.degrees(angle_diff):.0f} deg), fallback to prev pose")
                rvec = self.prev_rvec.copy()
                tvec = self.prev_tvec.copy()
                self.pose_filter_init = False

        self.prev_rvec = rvec.copy()
        self.prev_tvec = tvec.copy()
        self.have_prev_pose = True

        # OneEuro 滤波（elf 中 PNP_USE_FILTER=0，这里同步禁用）
        rvec_f = rvec.copy()
        tvec_f = tvec.copy()

        # 固定安装校正
        R_face2cam_raw, _ = cv2.Rodrigues(rvec_f)
        R_calib = self.R_mount @ R_face2cam_raw
        rvec_calib, _ = cv2.Rodrigues(R_calib)

        # 提取欧拉角（ZYX: Yaw-Pitch-Roll）
        sy = math.sqrt(R_calib[0, 0] ** 2 + R_calib[1, 0] ** 2)
        if sy > 1e-6:
            head_pitch = math.atan2(R_calib[2, 1], R_calib[2, 2])
            head_yaw = math.atan2(-R_calib[2, 0], sy)
            head_roll = math.atan2(R_calib[1, 0], R_calib[0, 0])
        else:
            head_pitch = math.atan2(-R_calib[1, 2], R_calib[1, 1])
            head_yaw = math.atan2(-R_calib[2, 0], sy)
            head_roll = 0.0

        R_face2cam = R_calib
        R_cam2face = R_face2cam.T
        cam_pos = -R_cam2face @ tvec_f

        # 四元数
        qw, qx, qy, qz = self.rotation_matrix_to_quaternion(R_face2cam)

        # α 坐标系变换
        R_af = np.array([
            [1.0,  0.0,  0.0],
            [0.0,  0.0, -1.0],
            [0.0, -1.0,  0.0],
        ], dtype=np.float64)
        d_O_in_head = np.array([[0.0], [150.0], [180.0]], dtype=np.float64)
        R_F2A = R_af @ R_face2cam_raw
        t_cam_in_obj = -R_face2cam_raw.T @ tvec_f
        t_cam_in_alpha = R_F2A @ (t_cam_in_obj - d_O_in_head)

        # ---------- 绘制 ----------
        # 文字信息
        hp_deg = math.degrees(head_pitch)
        hy_deg = math.degrees(head_yaw)
        hr_deg = math.degrees(head_roll)
        rx, ry, rz = rvec_calib.flatten()

        info = {
            'yaw': hy_deg,
            'pitch': hp_deg,
            'roll': hr_deg,
            'rvec': (rx, ry, rz),
            'quat': (qw, qx, qy, qz),
            'cam_alpha': (t_cam_in_alpha[0, 0] / 10.0,
                          t_cam_in_alpha[1, 0] / 10.0,
                          t_cam_in_alpha[2, 0] / 10.0),
            'reproj_error': reproj_error,
        }

        # 原点和坐标轴
        origin, _ = cv2.projectPoints(np.array([[0, 0, 0]], dtype=np.float32),
                                      rvec_calib, tvec_f, K, D)
        origin = origin[0, 0].astype(int)
        axis_len = 40  # mm
        axis_3d = np.array([[axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]], dtype=np.float32)
        axis_2d, _ = cv2.projectPoints(axis_3d, rvec_calib, tvec_f, K, D)
        axis_2d = axis_2d[:, 0].astype(int)

        cv2.line(frame, tuple(origin), tuple(axis_2d[0]), (0, 0, 255), 2)   # X red
        cv2.line(frame, tuple(origin), tuple(axis_2d[1]), (0, 255, 0), 2)   # Y green
        cv2.line(frame, tuple(origin), tuple(axis_2d[2]), (255, 0, 0), 2)   # Z blue

        # 12 个 PnP 点：白方块 + 红叉（重投影） + 编号
        for i, (ipt, rpt) in enumerate(zip(image_points, reproj_pts[:, 0])):
            ix, iy = int(ipt[0]), int(ipt[1])
            rx_, ry_ = int(rpt[0]), int(rpt[1])
            cv2.rectangle(frame, (ix - 2, iy - 2), (ix + 2, iy + 2), (255, 255, 255), -1)
            cv2.drawMarker(frame, (rx_, ry_), (0, 0, 255), cv2.MARKER_CROSS, 7, 1)
            cv2.putText(frame, str(i), (ix + 6, iy - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 立方体
        cube_3d = np.array([
            [-40, -45, -20], [40, -45, -20], [40, 60, -20], [-40, 60, -20],
            [-40, -45, -100], [40, -45, -100], [40, 60, -100], [-40, 60, -100],
        ], dtype=np.float32)
        cube_2d, _ = cv2.projectPoints(cube_3d, rvec_calib, tvec_f, K, D)
        cube_2d = cube_2d[:, 0].astype(int)

        back_edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
        conn_edges = [(0, 4), (1, 5), (2, 6), (3, 7)]
        front_edges = [(4, 5), (5, 6), (6, 7), (7, 4)]
        for s, e in back_edges:
            cv2.line(frame, tuple(cube_2d[s]), tuple(cube_2d[e]), (0, 128, 0), 1)
        for s, e in conn_edges:
            cv2.line(frame, tuple(cube_2d[s]), tuple(cube_2d[e]), (255, 255, 0), 1)
        for s, e in front_edges:
            cv2.line(frame, tuple(cube_2d[s]), tuple(cube_2d[e]), (0, 255, 0), 2)

        # 文字 overlay（黑底白字）
        line1 = f"Yaw={hy_deg:.1f} Pitch={hp_deg:.1f} Roll={hr_deg:.1f}"
        line2 = f"rvec=({rx:.2f}, {ry:.2f}, {rz:.2f})"
        line3 = f"quat=({qw:.2f}, {qx:.2f}, {qy:.2f}, {qz:.2f})"
        line4 = f"cam_alpha=({info['cam_alpha'][0]:.1f}, {info['cam_alpha'][1]:.1f}, {info['cam_alpha'][2]:.1f}) cm"
        line5 = f"reproj_err={reproj_error:.1f}px"

        texts = [line1, line2, line3, line4, line5]
        max_w = 0
        total_h = 0
        for t in texts:
            (tw, th), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            max_w = max(max_w, tw)
            total_h += th + 8

        cv2.rectangle(frame, (8, 8), (8 + max_w + 24, 8 + total_h + 12), (0, 0, 0), -1)
        y_pos = 8 + 20
        for t in texts:
            cv2.putText(frame, t, (20, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            (tw, th), _ = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            y_pos += th + 8

        return True, info

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return qw, qx, qy, qz


# ========== RuleEngine 状态推理（NumPy 原型实现）==========
# 替换掉黑盒 rule_engine.onnx，逻辑完全复刻用户提供的 NumPy 原型。
class RuleEngineState:
    """
    NumPy 版规则引擎（BBox-delta + 手腕外展）。
    输出 7 维状态：
      [state, r_hold, l_hold, c_hold, n_hold, r_miss, l_miss]
    """
    STATE_NAMES = ["state", "r_hold", "l_hold", "c_hold", "n_hold", "r_miss", "l_miss"]

    # 阈值（与 NumPy 原型一致）
    HOLD_F = 5
    RESET_F = 2    # 收回更快
    CENTER_FORCE_F = 2
    NEUTRAL_FORCE_F = 1

    P_EXIT = 0.18
    P_WRIST_EXIT_R = 0.24    # keep 稍严
    P_WRIST_EXIT_PX = 110.0  # keep 稍严

    P_BBOX_DELTA_RATIO = 0.17    # trigger 稍严
    P_WRIST_EXTEND_RATIO = 0.25  # trigger 稍严
    P_DOMINANCE = 0.80

    EPS = 1e-6

    def __init__(self):
        self.state = np.zeros(7, dtype=np.float32)

    def build_kpts_20(self, body_det, face_lm_img, img_w):
        """构造 20 关键点 + valid_mask，并按用户要求对规则引擎做水平镜像。"""
        kps_20 = np.zeros((20, 2), dtype=np.float32)
        valid_mask = np.zeros(20, dtype=np.float32)

        coco_kps = body_det['kps']
        for i in range(17):
            if coco_kps[i]['visibility'] > KPT_CONF_THRESHOLD:
                kps_20[i, 0] = img_w - coco_kps[i]['x']
                kps_20[i, 1] = coco_kps[i]['y']
                valid_mask[i] = 1.0

        if face_lm_img and len(face_lm_img) > max(RULE_FACE_LEFT_MOUTH, RULE_FACE_RIGHT_MOUTH, RULE_FACE_CHIN):
            kps_20[17] = [img_w - face_lm_img[RULE_FACE_LEFT_MOUTH][0], face_lm_img[RULE_FACE_LEFT_MOUTH][1]]
            kps_20[18] = [img_w - face_lm_img[RULE_FACE_RIGHT_MOUTH][0], face_lm_img[RULE_FACE_RIGHT_MOUTH][1]]
            kps_20[19] = [img_w - face_lm_img[RULE_FACE_CHIN][0], face_lm_img[RULE_FACE_CHIN][1]]
            valid_mask[17:20] = 1.0

        return kps_20, valid_mask

    def infer(self, pipeline, body_det, face_lm_img, img_w):
        """使用 rule_engine_v2.onnx 在 GPU 上推理 7 维状态（输入已水平镜像）。"""
        kps_20, valid_mask = self.build_kpts_20(body_det, face_lm_img, img_w)
        # bbox 同步镜像，保持与关键点在同一坐标系
        bbox = np.array([img_w - body_det['x2'], body_det['y1'],
                         img_w - body_det['x1'], body_det['y2']], dtype=np.float32)

        inputs = {
            'kpts_flat': kps_20.reshape(1, -1),
            'valid_mask': valid_mask[np.newaxis, :],
            'bbox': bbox[np.newaxis, :],
            'fb': self.state[np.newaxis, :],
        }

        inp_list = []
        for name in pipeline.rule_input_names:
            inp_list.append(inputs[name])
        out = pipeline.rule_compiled(inp_list)
        out_tensor = out[pipeline.rule_output_name]
        self.state = out_tensor[0].astype(np.float32)
        return self.state.copy()

    def _rule_engine_numpy(self, kpts_20, valid_mask, bbox,
                           prev_state, prev_r_hold, prev_l_hold,
                           prev_c_hold, prev_n_hold, prev_r_miss, prev_l_miss):
        kpts = np.array(kpts_20, dtype=np.float32).reshape(20, 2)
        vm = np.array(valid_mask, dtype=np.float32)
        bbox_x1, bbox_y1, bbox_x2, bbox_y2 = bbox
        bbox_w = max(1.0, bbox_x2 - bbox_x1)

        def getx(idx):
            return float(kpts[idx, 0]) if 0 <= idx < 20 else 0.0

        def getv(idx):
            return float(vm[idx]) if 0 <= idx < 20 else 0.0

        # 稳定锚点
        ls_x, rs_x = getx(5), getx(6)
        mx_shoulder = (ls_x + rs_x) / 2.0
        ls_v, rs_v = getv(5), getv(6)

        # 伸手核心指标：bbox 中心 vs 肩膀中心
        mx_bbox = (bbox_x1 + bbox_x2) / 2.0
        bbox_delta = mx_bbox - mx_shoulder

        # scale
        shoulder_w = abs(rs_x - ls_x) if (ls_v > 0 and rs_v > 0) else 0.0
        scale = max(1.0, shoulder_w) if shoulder_w > 0 else bbox_w

        # 手腕坐标
        lwx, rwx = getx(9), getx(10)
        lv_w, rv_w = getv(9), getv(10)

        # 归一化距离（用于 keep 条件），参考肩膀中心
        mx = mx_shoulder
        right_wrist_t = max(0.0, (rwx - mx) / scale) * rv_w
        left_wrist_t = max(0.0, (mx - lwx) / scale) * lv_w
        rw_px = right_wrist_t * scale
        lw_px = left_wrist_t * scale

        # 优势度：取手腕相对于同侧肩膀的外展差
        right_extend = (rwx - rs_x) if rv_w > 0 else 0.0
        left_extend = (ls_x - lwx) if lv_w > 0 else 0.0
        diff_rl = right_extend - left_extend
        diff_lr = left_extend - right_extend

        # trigger（比例阈值，自适应远近）
        right_trigger = (bbox_delta > self.P_BBOX_DELTA_RATIO * bbox_w and
                         right_extend > self.P_WRIST_EXTEND_RATIO * bbox_w and
                         diff_rl > self.P_DOMINANCE and rv_w > 0)

        left_trigger = (bbox_delta < -self.P_BBOX_DELTA_RATIO * bbox_w and
                        left_extend > self.P_WRIST_EXTEND_RATIO * bbox_w and
                        diff_lr > self.P_DOMINANCE and lv_w > 0)

        # keep
        right_keep = (right_wrist_t > self.P_WRIST_EXIT_R and rw_px > self.P_WRIST_EXIT_PX)
        left_keep = (left_wrist_t > self.P_WRIST_EXIT_R and lw_px > self.P_WRIST_EXIT_PX)

        # 更新 hold / keep_miss
        right_hold = prev_r_hold + 1 if right_trigger else 0
        left_hold = prev_l_hold + 1 if left_trigger else 0

        right_ready = right_hold >= self.HOLD_F
        left_ready = left_hold >= self.HOLD_F

        if right_ready and not left_ready:
            base_state = 1
        elif not right_ready and left_ready:
            base_state = 2
        elif right_ready and left_ready:
            base_state = 1 if right_extend >= left_extend else 2
        else:
            base_state = prev_state

        # 更新 hold / keep_miss（与原版原型一致：非 active state 也累加）
        right_keep_miss = 0 if right_keep else prev_r_miss + 1
        left_keep_miss = 0 if left_keep else prev_l_miss + 1

        reset1 = (prev_state == 1 and right_keep_miss >= self.RESET_F)
        reset2 = (prev_state == 2 and left_keep_miss >= self.RESET_F)
        state_after_reset = 0 if (reset1 or reset2) else base_state

        # center / neutral 强制回 0
        center_th = 0.26 * scale
        wrists_center = (abs(lwx - mx) < center_th) and (abs(rwx - mx) < center_th)

        disp = abs(ls_x - mx)
        for idx in (6, 7, 8, 9, 10):
            disp += abs(getx(idx) - mx)
        disp_mean = disp / 6.0
        center_ok = (disp_mean / scale < 0.20) and wrists_center

        neutral_ok = abs(right_extend - left_extend) < 0.10 * scale

        center_hold = prev_c_hold + 1 if center_ok else 0
        neutral_hold = prev_n_hold + 1 if neutral_ok else 0

        force0 = (center_hold >= self.CENTER_FORCE_F) or (neutral_hold >= self.NEUTRAL_FORCE_F)
        state = 0 if force0 else state_after_reset

        # 调试打印（可通过全局 DEBUG 控制）
        if DEBUG:
            rt_flag = "R" if right_trigger else ""
            lt_flag = "L" if left_trigger else ""
            print(f"[RE] st={state} bd={bbox_delta:+.1f} rxt={right_extend:.1f} "
                  f"lxt={left_extend:.1f} drl={diff_rl:+.1f} tr=({rt_flag},{lt_flag}) "
                  f"rh={right_hold} lh={left_hold} rk={right_keep_miss} lk={left_keep_miss}")

        return [int(state), int(right_hold), int(left_hold),
                int(center_hold), int(neutral_hold),
                int(right_keep_miss), int(left_keep_miss)]

    def reset(self):
        self.state[:] = 0.0


# ========== ROI 估算（从 elf 照抄）==========
def estimate_face_roi(det, tracker, img_w, img_h):
    """
    输入：最佳人体 detection（已映射回原图坐标）
    输出：roi_x, roi_y, roi_w, roi_h 或 None（人脸不可用）
    """
    v_nose = det['kps'][0]['visibility']
    v_l_eye = det['kps'][1]['visibility']
    v_r_eye = det['kps'][2]['visibility']
    v_l_shoulder = det['kps'][5]['visibility']
    v_r_shoulder = det['kps'][6]['visibility']

    face_valid = True
    skip_reason = None
    if v_nose <= KPT_CONF_THRESHOLD:
        face_valid = False
        skip_reason = "nose low conf"
    elif v_l_eye <= KPT_CONF_THRESHOLD and v_r_eye <= KPT_CONF_THRESHOLD:
        face_valid = False
        skip_reason = "both eyes low conf"
    elif det['kps'][0]['y'] > img_h * 0.70:
        face_valid = False
        skip_reason = "nose too low"
    else:
        shoulder_y = (det['kps'][5]['y'] + det['kps'][6]['y']) * 0.5
        if shoulder_y - det['kps'][0]['y'] < img_h * 0.06:
            face_valid = False
            skip_reason = "face too short"

    def finalize_roi(rx, ry, rw, rh):
        if rx < 0:
            rx = 0
        if ry < 0:
            ry = 0
        if rx + rw > img_w:
            rw = img_w - rx
            rh = rw
        if ry + rh > img_h:
            rh = img_h - ry
            rw = rh
        rw = (rw // 16) * 16
        rh = rw
        if rx % 2 != 0:
            rx += 1
        if ry % 2 != 0:
            ry += 1
        if rw < 32 or rh < 32:
            return None
        return (rx, ry, rw, rh)

    # Fallback: 如果人体关键点不足，但检测框本身够大，尝试用框上半部估计人脸 ROI
    if not face_valid:
        box_w = det['x2'] - det['x1']
        box_h = det['y2'] - det['y1']
        # 框需覆盖一个合理的人脸区域：高度 > 15% 图像高，宽度 > 10% 图像宽
        if box_h > img_h * 0.15 and box_w > img_w * 0.10:
            roi_size = tracker.get_roi_size()
            if roi_size <= 0:
                # 用检测框上半部作为初始估计
                roi_size = max(box_w, box_h * 0.6)
            roi_size = max(150.0, min(640.0, roi_size))
            roi_cx = (det['x1'] + det['x2']) * 0.5
            roi_cy = det['y1'] + box_h * 0.25
            roi_x = int(roi_cx - roi_size * 0.5)
            roi_y = int(roi_cy - roi_size * 0.5)
            roi_w = int(roi_size)
            roi_h = roi_w
            roi = finalize_roi(roi_x, roi_y, roi_w, roi_h)
            if roi is not None:
                return roi, "fallback bbox"
        if tracker.face_size_life > 0:
            tracker.face_size_life -= 1
        return None, skip_reason

    roi_cx = det['kps'][0]['x']
    roi_cy = 0.0
    roi_size_f = 0.0

    if v_nose > 0.5:
        shoulder_w = abs(det['kps'][5]['x'] - det['kps'][6]['x'])
        shoulder_roi = shoulder_w * 1.6
        face_roi = tracker.get_roi_size()

        if face_roi > 0:
            target_roi = face_roi
            if target_roi > shoulder_roi:
                target_roi = shoulder_roi
            roi_cy = det['kps'][0]['y'] + tracker.last_face_h * 0.05
            roi_size_f = target_roi
        else:
            target_roi = shoulder_roi
            roi_cy = det['kps'][0]['y'] + 30.0
            if tracker.prev_roi_size > 0:
                diff = target_roi - tracker.prev_roi_size
                if diff > 16.0:
                    diff = 16.0
                roi_size_f = tracker.prev_roi_size + diff
            else:
                roi_size_f = target_roi

        tracker.prev_roi_size = roi_size_f
    else:
        roi_cx = (det['x1'] + det['x2']) * 0.5
        roi_cy = det['y1'] + (det['y2'] - det['y1']) * 0.35
        roi_size_f = (det['y2'] - det['y1']) * 0.55
        tracker.prev_roi_size = roi_size_f

    roi_size_f = max(150.0, min(640.0, roi_size_f))

    roi_x = int(roi_cx - roi_size_f * 0.5)
    roi_y = int(roi_cy - roi_size_f * 0.5)
    roi_w = int(roi_size_f)
    roi_h = int(roi_size_f)

    roi = finalize_roi(roi_x, roi_y, roi_w, roi_h)
    if roi is None:
        return None, "roi too small"
    return roi, None


# ========== 手部 ROI 与关键点 ==========
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),          # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8),          # Index
    (0, 9), (9, 10), (10, 11), (11, 12),     # Middle
    (0, 13), (13, 14), (14, 15), (15, 16),   # Ring
    (0, 17), (17, 18), (18, 19), (19, 20),   # Pinky
    (5, 9), (9, 13), (13, 17),               # Palm
]


def estimate_hand_roi(wrist_kp, elbow_kp, img_w, img_h):
    """
    根据前臂朝向（手腕-手肘方向）和手肘关键点预测手部中心位置，
    并以该预测中心到手肘的距离确定 ROI 大小。
    返回 (roi_x, roi_y, roi_w, roi_h) 或 None。
    """
    if wrist_kp['visibility'] <= KPT_CONF_THRESHOLD:
        return None

    # 有手肘关键点：利用前臂朝向预测手部中心
    if elbow_kp['visibility'] > KPT_CONF_THRESHOLD:
        dx = wrist_kp['x'] - elbow_kp['x']
        dy = wrist_kp['y'] - elbow_kp['y']
        forearm_len = math.hypot(dx, dy)
        if forearm_len < 1e-6:
            return None

        # 沿前臂方
        # /home/time/work/trial0/camera_demo_elf_pipeline.py向（手肘 -> 手腕）继续延伸，预测手掌/手指区域中心
        # 手掌中心约在手腕外侧 0.5 倍前臂长度处
        HAND_CENTER_EXTEND_RATIO = 0.50
        roi_cx = wrist_kp['x'] + dx * HAND_CENTER_EXTEND_RATIO
        roi_cy = wrist_kp['y'] + dy * HAND_CENTER_EXTEND_RATIO

        # ROI 大小：预测手部中心到手肘距离的 1.05 倍
        center_to_elbow = forearm_len * (1.0 + HAND_CENTER_EXTEND_RATIO)
        roi_size = center_to_elbow * 1.05
    else:
        # fallback：无手肘关键点时，以手腕为中心，使用固定大小
        roi_cx = wrist_kp['x']
        roi_cy = wrist_kp['y']
        roi_size = 250.0

    roi_size = max(150.0, min(400.0, roi_size))

    roi_x = int(roi_cx - roi_size * 0.5)
    roi_y = int(roi_cy - roi_size * 0.5)
    roi_w = int(roi_size)
    roi_h = int(roi_size)

    # 限制在图像内
    if roi_x < 0:
        roi_x = 0
    if roi_y < 0:
        roi_y = 0
    if roi_x + roi_w > img_w:
        roi_w = img_w - roi_x
    if roi_y + roi_h > img_h:
        roi_h = img_h - roi_y

    # 保证是偶数尺寸且不小于 32
    roi_w = (roi_w // 2) * 2
    roi_h = (roi_h // 2) * 2
    if roi_w < 32 or roi_h < 32:
        return None

    return roi_x, roi_y, roi_w, roi_h


def preprocess_hand_roi(frame, roi):
    """
    裁剪手部 ROI 并预处理为 hand_landmarks_detector 输入。
    输入 ROI: (x, y, w, h)
    返回: input_tensor [1, 224, 224, 3]
    """
    rx, ry, rw, rh = roi
    crop = frame[ry:ry + rh, rx:rx + rw]
    if crop.size == 0:
        return None
    rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (HAND_LM_INPUT_SIZE, HAND_LM_INPUT_SIZE))
    normalized = resized.astype(np.float32) / 127.5 - 1.0
    return np.expand_dims(normalized, axis=0)


def preprocess_hand_landmarks_for_classifier(landmarks):
    """
    将 21 个手部关键点归一化为 keypoint_classifier 的输入 [42]。
    与 mymodel/openvino_pipeline 中的 pre_process_landmark 保持一致。
    """
    landmark_list = [[int(landmarks[i, 0]), int(landmarks[i, 1])]
                     for i in range(21)]

    base_x, base_y = 0, 0
    for index, point in enumerate(landmark_list):
        if index == 0:
            base_x, base_y = point[0], point[1]
        landmark_list[index][0] -= base_x
        landmark_list[index][1] -= base_y

    flat = [v for point in landmark_list for v in point]
    max_value = max(map(abs, flat))
    if max_value == 0:
        max_value = 1

    normalized = [v / max_value for v in flat]
    return np.array(normalized, dtype=np.float32)


def compute_point_direction(landmarks):
    """
    计算食指指向方向（UP / DOWN / LEFT / RIGHT）。
    使用食指从 MCP (landmark 5) 到指尖 (landmark 8) 的向量。
    """
    try:
        index_mcp = landmarks[5, :2]
        index_tip = landmarks[8, :2]
        dx = float(index_tip[0] - index_mcp[0])
        dy = float(index_tip[1] - index_mcp[1])

        length = math.hypot(dx, dy)
        palm_size = math.hypot(
            float(landmarks[0, 0] - landmarks[9, 0]),
            float(landmarks[0, 1] - landmarks[9, 1]))
        if palm_size == 0 or length < palm_size * 0.2:
            return None

        if abs(dx) > abs(dy):
            return "RIGHT" if dx > 0 else "LEFT"
        else:
            return "DOWN" if dy > 0 else "UP"
    except Exception:
        return None


def correct_open_ok(gesture_id, landmarks, ok_threshold=0.35):
    """
    用拇指指尖 (4) 和食指指尖 (8) 的距离纠正 Open / OK 的误判。
    """
    try:
        thumb_tip = landmarks[4, :2]
        index_tip = landmarks[8, :2]
        wrist = landmarks[0, :2]
        middle_mcp = landmarks[9, :2]

        thumb_index_dist = math.hypot(
            float(thumb_tip[0] - index_tip[0]),
            float(thumb_tip[1] - index_tip[1]))
        palm_size = math.hypot(
            float(wrist[0] - middle_mcp[0]),
            float(wrist[1] - middle_mcp[1]))
        if palm_size == 0:
            return gesture_id

        ratio = thumb_index_dist / palm_size
        OPEN_ID, OK_ID = 0, 3

        if gesture_id == OPEN_ID and ratio < ok_threshold:
            return OK_ID
        if gesture_id == OK_ID and ratio >= ok_threshold:
            return OPEN_ID
    except Exception:
        pass
    return gesture_id


def correct_pointer_close(gesture_id, landmarks, pointer_threshold=0.55):
    """
    用食指伸出长度纠正 Pointer / Close 的误判。
    """
    try:
        index_mcp = landmarks[5, :2]
        index_tip = landmarks[8, :2]
        wrist = landmarks[0, :2]
        middle_mcp = landmarks[9, :2]

        index_length = math.hypot(
            float(index_tip[0] - index_mcp[0]),
            float(index_tip[1] - index_mcp[1]))
        palm_size = math.hypot(
            float(wrist[0] - middle_mcp[0]),
            float(wrist[1] - middle_mcp[1]))
        if palm_size == 0:
            return gesture_id

        ratio = index_length / palm_size
        CLOSE_ID, POINTER_ID = 1, 2

        if gesture_id == POINTER_ID and ratio < pointer_threshold:
            return CLOSE_ID
        if gesture_id == CLOSE_ID and ratio >= pointer_threshold:
            return POINTER_ID
    except Exception:
        pass
    return gesture_id


def classify_hand_gesture(pipeline, landmarks):
    """
    使用 OpenVINO ONNX keypoint_classifier 对手部关键点进行分类，
    并做 Open/OK、Pointer/Close 的几何修正，Pointer 时计算指向方向。
    返回 (gesture_label, gesture_id, point_direction)。
    """
    input_tensor = np.expand_dims(
        preprocess_hand_landmarks_for_classifier(landmarks), axis=0)
    out = pipeline.hand_kpt_cls_compiled(
        {pipeline.hand_kpt_cls_input_name: input_tensor})
    scores = out[pipeline.hand_kpt_cls_output_name][0]
    gesture_id = int(np.argmax(scores))

    # 几何修正
    gesture_id = correct_open_ok(gesture_id, landmarks)
    gesture_id = correct_pointer_close(gesture_id, landmarks)

    # Pointer 指向方向
    point_direction = None
    if gesture_id == 2:
        point_direction = compute_point_direction(landmarks)

    if 0 <= gesture_id < len(pipeline.hand_kpt_cls_labels):
        label = pipeline.hand_kpt_cls_labels[gesture_id]
    else:
        label = "?"

    if point_direction:
        label = f"{label}-{point_direction}"

    return label, gesture_id, point_direction


def detect_hand_landmarks(pipeline, frame, roi):
    """
    对 ROI 运行手部关键点检测与手势分类。
    返回 dict 或 None。
    """
    input_tensor = preprocess_hand_roi(frame, roi)
    if input_tensor is None:
        return None

    out = pipeline.hand_lm_compiled({pipeline.hand_lm_input_name: input_tensor})
    landmarks_px = out[pipeline.hand_lm_output_names[0]][0]      # [63]
    presence_raw = out[pipeline.hand_lm_output_names[1]][0, 0]   # [1]
    handedness_raw = out[pipeline.hand_lm_output_names[2]][0, 0] # [1]
    world_landmarks = out[pipeline.hand_lm_output_names[3]][0]   # [63]

    presence = 1.0 / (1.0 + math.exp(-float(presence_raw)))
    handedness = 1.0 / (1.0 + math.exp(-float(handedness_raw)))

    if presence < HAND_LM_CONF_THRESHOLD:
        return None

    landmarks_px = landmarks_px.reshape(21, 3)
    world_landmarks = world_landmarks.reshape(21, 3)

    # 映射回原始图像坐标
    rx, ry, rw, rh = roi
    landmarks_img = landmarks_px.copy()
    landmarks_img[:, 0] = landmarks_px[:, 0] / HAND_LM_INPUT_SIZE * rw + rx
    landmarks_img[:, 1] = landmarks_px[:, 1] / HAND_LM_INPUT_SIZE * rh + ry
    landmarks_img[:, 2] = landmarks_px[:, 2] / HAND_LM_INPUT_SIZE * max(rw, rh)

    # 手势分类（含 Open/OK、Pointer/Close 几何修正与 Pointer 指向方向）
    gesture_label, gesture_id, point_direction = classify_hand_gesture(
        pipeline, landmarks_img)

    return {
        "landmarks": landmarks_img,
        "world_landmarks": world_landmarks,
        "presence": presence,
        "handedness": "Right" if handedness > 0.5 else "Left",
        "gesture_label": gesture_label,
        "gesture_id": gesture_id,
        "point_direction": point_direction,
        "roi": roi,
    }


def draw_hand_landmarks(frame, hand_result, color=(0, 255, 0)):
    """在图像上绘制手部关键点和骨架。"""
    if hand_result is None:
        return
    landmarks = hand_result["landmarks"]
    h, w = frame.shape[:2]

    # 画连接线
    for a, b in HAND_CONNECTIONS:
        pa = landmarks[a]
        pb = landmarks[b]
        if 0 <= pa[0] < w and 0 <= pa[1] < h and 0 <= pb[0] < w and 0 <= pb[1] < h:
            cv2.line(frame, (int(pa[0]), int(pa[1])), (int(pb[0]), int(pb[1])), color, 2)

    # 画关键点
    for i, p in enumerate(landmarks):
        x, y = int(p[0]), int(p[1])
        if 0 <= x < w and 0 <= y < h:
            cv2.circle(frame, (x, y), 3, color, -1)

    # ROI 框
    rx, ry, rw, rh = hand_result["roi"]
    cv2.rectangle(frame, (rx, ry), (rx + rw, ry + rh), color, 2)


# ========== 主程序 ==========
def find_camera_index(preferred=None, max_index=9):
    """Auto-detect the first available V4L2 capture device.

    Realtek USB cameras sometimes enumerate as /dev/video1 or video2 instead
    of video0. Try ``preferred`` first, then scan 0..max_index.
    Returns the index or raises RuntimeError if none is available.
    """
    candidates = []
    if preferred is not None:
        candidates.append(preferred)
    candidates.extend(i for i in range(max_index + 1) if i != preferred)

    for idx in candidates:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            cap.release()
            if w > 0 and h > 0:
                print(f"[Camera] Auto-selected device index {idx} ({w}x{h})")
                return idx
        try:
            cap.release()
        except Exception:
            pass
    raise RuntimeError("No usable camera found. Check /dev/video* and USB connection.")


class FrameGrabber(threading.Thread):
    """Dedicated capture thread: always read the camera and keep only the latest frame.

    This decouples the USB camera's frame cadence from the heavy inference loop,
    so ``cap.read()`` blocking does not inflate the per-frame latency metrics.
    """

    def __init__(self, cap: cv2.VideoCapture, timeout: float = 0.05) -> None:
        super().__init__(daemon=True)
        self.cap = cap
        self._timeout = timeout
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._ready = threading.Event()
        self._stop_ev = threading.Event()

    def run(self) -> None:
        while not self._stop_ev.is_set():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self._lock:
                    self._frame = frame
                self._ready.set()

    def get(self) -> np.ndarray | None:
        """Return the latest frame, or None if no new frame arrived recently."""
        if self._ready.wait(timeout=self._timeout):
            with self._lock:
                frame = self._frame
                self._frame = None
            self._ready.clear()
            return frame
        return None

    def stop(self) -> None:
        self._stop_ev.set()


def main():
    global OBJ_THRESHOLD, DEBUG, SELFIE_MODE
    DEBUG = False
    SELFIE_MODE = False

    parser = argparse.ArgumentParser(description="ELF visual pipeline demo")
    parser.add_argument('--headless', action='store_true', help='Run without GUI window, save output frames')
    parser.add_argument('--output', type=str, default='/tmp/elf_frame.jpg', help='Output frame path in headless mode')
    parser.add_argument('--frames', type=int, default=30, help='Number of frames to process in headless mode')
    parser.add_argument('--flip', action='store_true', help='Flip image vertically (camera upside down)')
    parser.add_argument('--rotate', type=int, default=0, choices=[0, 90, 180, 270],
                        help='Rotate image clockwise by degrees before processing')
    parser.add_argument('--conf', type=float, default=OBJ_THRESHOLD, help='Body detection confidence threshold')
    parser.add_argument('--no-stream', action='store_true', help='Skip RTMP/RTSP streaming')
    parser.add_argument('--debug', action='store_true', help='Print detection/tracker debug info and draw all detections')
    parser.add_argument('--selfie', action='store_true', help='Operator selfie mode: prefer the largest/largest-face detection (closest person)')
    parser.add_argument('--camera-index', type=int, default=None,
                        help='OpenCV camera index (default: auto-detect first available camera)')

    # Voice KWS options
    parser.add_argument('--voice', action='store_true', help='Enable Sherpa keyword spotting background thread')
    parser.add_argument('--voice-device', type=int, default=6, help='sounddevice input device index for voice (default: USB mic)')
    parser.add_argument('--voice-provider', type=str, default='cpu', choices=['cpu', 'openvino'],
                        help='KWS execution provider: cpu (safe) or openvino (needs custom build)')
    parser.add_argument('--voice-capture-rate', type=int, default=48000, help='Microphone native sample rate')
    parser.add_argument('--voice-channels', type=int, default=2, help='Microphone native channels')
    parser.add_argument('--voice-blocksize', type=int, default=1024, help='Audio capture block size')
    parser.add_argument('--voice-no-vad', action='store_true', help='Disable Silero VAD front-end')
    parser.add_argument('--voice-vad-threshold', type=float, default=None,
                        help='VAD speech threshold, lower = more sensitive (default from voice/config.py)')
    parser.add_argument('--voice-vad-hangover-ms', type=float, default=None,
                        help='VAD hangover after speech ends (default from voice/config.py)')
    parser.add_argument('--voice-silence-reset-blocks', type=int, default=None,
                        help='Consecutive silent blocks before resetting KWS (default from voice/config.py)')
    parser.add_argument('--voice-score', type=float, default=None,
                        help='KWS keywords score (lower = easier to trigger, default from voice/config.py)')
    parser.add_argument('--voice-threshold', type=float, default=None,
                        help='KWS keywords threshold (lower = easier to trigger, default from voice/config.py)')
    parser.add_argument('--voice-trailing-blanks', type=int, default=None,
                        help='KWS num trailing blanks (default from voice/config.py)')
    parser.add_argument('--voice-gain', type=float, default=None,
                        help='Fixed linear audio gain (default from voice/config.py)')
    parser.add_argument('--voice-auto-gain', action='store_true', default=None,
                        help='Enable software auto-gain for quiet microphones')
    parser.add_argument('--voice-no-auto-gain', action='store_true',
                        help='Disable software auto-gain (default is enabled in config)')
    parser.add_argument('--voice-auto-gain-target-db', type=float, default=None,
                        help='Target RMS level for auto-gain (default -20 dB)')
    parser.add_argument('--voice-auto-gain-max-db', type=float, default=None,
                        help='Maximum auto-gain boost (default 30 dB)')
    parser.add_argument('--voice-auto-gain-min-db', type=float, default=None,
                        help='Minimum RMS to be boosted (noise floor, default -50 dB)')
    parser.add_argument('--screenshot-interval', type=float, default=0.0,
                        help='Save a screenshot every N seconds (0 = disable)')
    parser.add_argument('--screenshot-dir', type=str, default='/tmp/elf_screenshots',
                        help='Directory to save periodic screenshots')
    args = parser.parse_args()

    # 允许命令行覆盖置信度阈值
    OBJ_THRESHOLD = args.conf

    print("\n" + "=" * 60)
    print("ELF Pipeline Demo starting...")
    print("=" * 60)
    print(f"[Config] conf_threshold={OBJ_THRESHOLD}, flip={args.flip or FLIP_VERTICAL}, rotate={args.rotate}, debug={args.debug}, selfie={args.selfie}")
    DEBUG = args.debug
    SELFIE_MODE = args.selfie

    # 初始化模型
    pipeline = ElfPipeline()
    pose_est = PoseEstimator()
    tracker = FaceTracker()
    rule_engine_state = RuleEngineState()

    # 初始化摄像头（支持自动检测可用设备）
    cam_idx = find_camera_index(args.camera_index)
    cap = cv2.VideoCapture(cam_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    grabber = FrameGrabber(cap)
    grabber.start()
    print("[Camera] Frame grabber thread started")

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"\n[Camera] Actual resolution: {actual_w}x{actual_h}")

    # 初始化推流
    streamer = Streamer(RTMP_URL, actual_w, actual_h, fps=15)
    if args.no_stream:
        print("[Streamer] Streaming disabled by --no-stream")
    else:
        streamer.init_stream()

    # 初始化语音 KWS 后台线程（与视觉 pipeline 并行）
    voice_thread = None
    if args.voice:
        try:
            from voice import VoiceKwsThread
            auto_gain = None
            if args.voice_auto_gain:
                auto_gain = True
            elif args.voice_no_auto_gain:
                auto_gain = False
            voice_thread = VoiceKwsThread(
                model_dir=SHERPA_MODEL_DIR,
                keywords_file=SHERPA_KEYWORDS_FILE,
                provider=args.voice_provider,
                device_id=args.voice_device,
                capture_rate=args.voice_capture_rate,
                channels=args.voice_channels,
                blocksize=args.voice_blocksize,
                gain=args.voice_gain,
                auto_gain=auto_gain,
                auto_gain_target_db=args.voice_auto_gain_target_db,
                auto_gain_max_db=args.voice_auto_gain_max_db,
                auto_gain_min_db=args.voice_auto_gain_min_db,
                use_vad=not args.voice_no_vad,
                vad_model=SHERPA_VAD_MODEL,
                vad_threshold=args.voice_vad_threshold,
                vad_hangover_ms=args.voice_vad_hangover_ms,
                silence_reset_blocks=args.voice_silence_reset_blocks,
                keywords_score=args.voice_score,
                keywords_threshold=args.voice_threshold,
                num_trailing_blanks=args.voice_trailing_blanks,
            )
            voice_thread.start()
        except Exception as e:
            print(f"[Voice] Failed to start KWS thread: {e}")
            voice_thread = None

    # 性能统计
    frame_count = 0
    processed_frames = 0
    face_fail_count = 0
    start_time = time.time()
    profiler = PerformanceProfiler(report_interval=30)
    prev_frame_time = time.time()
    last_screenshot_time = start_time
    if args.screenshot_interval > 0:
        os.makedirs(args.screenshot_dir, exist_ok=True)
        print(f"[Screenshot] Saving every {args.screenshot_interval}s to {args.screenshot_dir}")

    print("\n[Main] Press 'q' to quit, 's' to toggle stream, 'r' to reset pose\n")

    try:
        while True:
            loop_start = time.time()
            frame = grabber.get()
            if frame is None:
                time.sleep(0.001)
                continue

            if args.flip or FLIP_VERTICAL:
                frame = cv2.flip(frame, 0)
            if args.rotate == 90:
                frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            elif args.rotate == 180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            elif args.rotate == 270:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            display = frame.copy()
            h, w = display.shape[:2]

            # 各模块耗时初始化
            t_camera = t_body = t_body_pre = t_body_inf = t_body_post = t_face = t_pnp = t_rule = t_hand = t_write = 0.0

            # ---------- Stage 1: Body Detection ----------
            t_camera = time.time() - loop_start

            t_body_pre_start = time.time()
            body_inp, scale, x_off, y_off = pipeline.preprocess_body(frame)
            t_body_pre = time.time() - t_body_pre_start

            t_body_inf_start = time.time()
            body_out = pipeline.body_compiled({pipeline.body_input_name: body_inp})
            t_body_inf = time.time() - t_body_inf_start

            t_body_post_start = time.time()
            body_dets = pipeline.postprocess_body(
                body_out[pipeline.body_output_name], w, h, scale, x_off, y_off
            )
            body_dets = nms_pose(body_dets, iou_thresh=NMS_THRESHOLD)
            t_body_post = time.time() - t_body_post_start
            t_body = t_body_pre + t_body_inf + t_body_post

            # 画所有人体框和关键点
            draw_limit = len(body_dets) if args.debug else min(3, len(body_dets))
            for det in body_dets[:draw_limit]:
                cv2.rectangle(display, (int(det['x1']), int(det['y1'])),
                              (int(det['x2']), int(det['y2'])), (0, 255, 255), 1)
                cv2.putText(display, f"{det['score']:.2f}",
                            (int(det['x1']), int(det['y1']) - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                for k, kp in enumerate(det['kps']):
                    if kp['visibility'] > KPT_CONF_THRESHOLD:
                        cv2.circle(display, (int(kp['x']), int(kp['y'])), 2, (0, 255, 0), -1)

            # 目标人物跟踪
            matched_idx = tracker.update(body_dets, w, h)

            if matched_idx >= 0 and matched_idx < len(body_dets):
                best_det = body_dets[matched_idx]

                # ---------- Stage 2: Face ROI ----------
                roi, skip_reason = estimate_face_roi(best_det, tracker, w, h)

                face_success_this_frame = False
                face_lm_img = None  # 468 点原图坐标，供 RuleEngine 使用
                if roi is not None:
                    roi_x, roi_y, roi_w, roi_h = roi
                    # 画 ROI 框
                    cv2.rectangle(display, (roi_x, roi_y),
                                  (roi_x + roi_w, roi_y + roi_h), (255, 0, 255), 2)

                    # ---------- Stage 3: Face Landmark 468 ----------
                    t_face_start = time.time()
                    lm = pipeline.infer_face_landmarks(frame, roi_x, roi_y, roi_w, roi_h)
                    t_face = time.time() - t_face_start

                    if lm is not None:
                        # Sanity check: 468 点分布
                        xs = lm[:, 0]
                        ys = lm[:, 1]
                        lm_min_x, lm_max_x = xs.min(), xs.max()
                        lm_min_y, lm_max_y = ys.min(), ys.max()
                        face_w_ratio = (lm_max_x - lm_min_x) / FACE_LM_INPUT_SIZE
                        face_h_ratio = (lm_max_y - lm_min_y) / FACE_LM_INPUT_SIZE

                        if 0.22 <= face_w_ratio <= 0.92 and 0.22 <= face_h_ratio <= 0.92:
                            # 更新 face size
                            face_w_img = (lm_max_x - lm_min_x) * (roi_w / FACE_LM_INPUT_SIZE)
                            face_h_img = (lm_max_y - lm_min_y) * (roi_h / FACE_LM_INPUT_SIZE)
                            tracker.update_face_size(face_w_img, face_h_img)

                            # 映射全部 468 点到原图（RuleEngine 需要嘴角/下巴）
                            lm_scale_x = roi_w / FACE_LM_INPUT_SIZE
                            lm_scale_y = roi_h / FACE_LM_INPUT_SIZE
                            face_lm_img = [(lm[i, 0] * lm_scale_x + roi_x,
                                            lm[i, 1] * lm_scale_y + roi_y)
                                           for i in range(len(lm))]

                            # 映射 12 个 PnP 点到原图
                            face_lm_2d = [face_lm_img[idx] for idx in FACE_LM_12_IDS]

                            # ---------- Stage 4: PnP ----------
                            t_pnp_start = time.time()
                            ok, info = pose_est.estimate_and_draw(display, face_lm_2d)
                            t_pnp = time.time() - t_pnp_start
                            if ok:
                                face_success_this_frame = True
                                face_fail_count = 0
                                cv2.putText(display, "PnP OK", (w - 120, 30),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                            else:
                                cv2.putText(display, "PnP FAIL", (w - 120, 30),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                        else:
                            if tracker.face_size_life > 0:
                                tracker.face_size_life -= 1
                            cv2.putText(display, f"FaceLM bad ratio", (roi_x, roi_y - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    cv2.putText(display, f"Skip face: {skip_reason}", (10, h - 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # ---------- Stage 5: Rule Engine ----------
                t_rule = 0.0
                rule_text = None
                try:
                    t_rule_start = time.time()
                    rule_state = rule_engine_state.infer(pipeline, best_det, face_lm_img, w)
                    t_rule = time.time() - t_rule_start
                    rule_text = "RULE: " + ", ".join(
                        f"{name}={int(rule_state[i])}"
                        for i, name in enumerate(RuleEngineState.STATE_NAMES)
                    )
                except Exception as e:
                    rule_text = f"RULE: err {type(e).__name__}"
                    print(f"[RuleEngine] inference failed: {e}")

                if rule_text:
                    # 放在画面左侧中上部（避免底部被截断）
                    y_pos = 140
                    (tw, th), _ = cv2.getTextSize(rule_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    cv2.rectangle(display, (8, y_pos - th - 6), (12 + tw, y_pos + 4), (0, 0, 0), -1)
                    cv2.putText(display, rule_text, (10, y_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

                # ---------- Stage 6: Hand Landmarks ----------
                t_hand_start = time.time()
                kps = best_det['kps']

                # 根据 RuleEngine 状态决定跳过哪只手的裁减：
                #   state=1 表示右手状态，不裁右手；
                #   state=2 表示左手状态，不裁左手。
                current_state = int(rule_state[0]) if rule_state is not None else 0
                skip_hand = set()
                if current_state == 1:
                    skip_hand.add(1)  # 1 = right
                elif current_state == 2:
                    skip_hand.add(0)  # 0 = left

                # COCO: 9=left wrist, 10=right wrist
                hand_rois = [
                    estimate_hand_roi(kps[9], kps[7], w, h),   # left hand
                    estimate_hand_roi(kps[10], kps[8], w, h),  # right hand
                ]
                hand_colors = [(255, 0, 0), (0, 255, 255)]  # left=blue, right=cyan
                hand_labels = ["Left", "Right"]
                hand_info_lines = []
                for idx, roi in enumerate(hand_rois):
                    if idx in skip_hand:
                        continue
                    if roi is None:
                        continue
                    hand_result = detect_hand_landmarks(pipeline, frame, roi)
                    if hand_result is None:
                        continue
                    # 置信度过低时不进行裁减/绘制，继续下一只手
                    if hand_result['presence'] < HAND_CROP_MIN_PRESENCE:
                        continue
                    draw_hand_landmarks(display, hand_result, color=hand_colors[idx])
                    gesture = hand_result.get('gesture_label', '-')
                    line = f"{hand_labels[idx]}: p={hand_result['presence']:.2f} {gesture}"
                    hand_info_lines.append(line)

                # 在右上角显示手部识别结果（放在 PnP 状态下方，避免重叠）
                if hand_info_lines:
                    x_pos = w - 10
                    y_start = 60
                    for i, line in enumerate(hand_info_lines):
                        y_pos = y_start + i * 28
                        (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)
                        cv2.rectangle(display, (x_pos - tw - 6, y_pos - th - 4),
                                      (x_pos + 4, y_pos + 6), (0, 0, 0), -1)
                        cv2.putText(display, line, (x_pos - tw, y_pos),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                t_hand = time.time() - t_hand_start

                # 连续多帧人脸/PnP 失败则重置跟踪，允许重新选择更优目标
                if not face_success_this_frame:
                    face_fail_count += 1
                    if face_fail_count >= 15:
                        print("[Tracker] Too many face failures, reset tracking")
                        tracker.tracked = False
                        tracker.history.clear()
                        face_fail_count = 0
                else:
                    face_fail_count = 0

            # 语音指令显示与动作派发
            if voice_thread is not None:
                latest = voice_thread.get_latest(consume=True)
                if latest:
                    keyword, _ts = latest
                    voice_text = f"VOICE: {keyword}"
                    print(f"[Main] Voice command: {keyword}")
                    # 简单动作映射示例
                    if keyword in ("PHOTO", "@拍照"):
                        photo_path = f"/tmp/elf_voice_photo_{int(time.time())}.jpg"
                        cv2.imwrite(photo_path, display)
                        print(f"[Main] Voice photo saved: {photo_path}")
                else:
                    voice_text = None
                    if voice_thread.last_keyword:
                        voice_text = f"VOICE(last): {voice_thread.last_keyword}"
                if voice_text:
                    (tw, th), _ = cv2.getTextSize(voice_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    cv2.rectangle(display, (8, 180 - th - 6), (12 + tw, 184), (0, 0, 0), -1)
                    cv2.putText(display, voice_text, (10, 180),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # FPS 显示（每帧瞬时帧率）
            frame_count += 1
            processed_frames += 1
            now = time.time()
            fps = 1.0 / (now - prev_frame_time + 1e-9)
            prev_frame_time = now

            cv2.putText(display, f"FPS: {fps:.1f} Dets: {len(body_dets)}",
                        (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 定期截图
            if args.screenshot_interval > 0:
                now = time.time()
                if now - last_screenshot_time >= args.screenshot_interval:
                    screenshot_path = os.path.join(
                        args.screenshot_dir,
                        f"elf_{time.strftime('%Y%m%d_%H%M%S')}_{frame_count:06d}.jpg"
                    )
                    cv2.imwrite(screenshot_path, display)
                    print(f"[Screenshot] Saved {screenshot_path}")
                    last_screenshot_time = now

            # 性能统计（loop_fps 用 total 时间计算，更准确）
            total_dt = time.time() - loop_start
            profiler.record({
                'total': total_dt,
                'camera_io': t_camera,
                'body_total': t_body,
                'body_pre': t_body_pre,
                'body_infer': t_body_inf,
                'body_post': t_body_post,
                'face_infer': t_face,
                'pnp': t_pnp,
                'rule': t_rule,
                'hand_infer': t_hand,
                'write': t_write,
                'loop_fps': 1.0 / (total_dt + 1e-9),
            })

            # 推流
            t_write_start = time.time()
            streamer.write(display)
            t_write = time.time() - t_write_start

            # 显示或保存
            if args.headless:
                if processed_frames % 10 == 0:
                    cv2.imwrite(args.output, display)
                    print(f"[Headless] Saved frame to {args.output}")
                if processed_frames >= args.frames:
                    print(f"[Headless] Processed {args.frames} frames, exiting.")
                    break
            else:
                cv2.imshow("ELF Pipeline Demo", display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    pose_est.have_prev_pose = False
                    pose_est.pose_filter_init = False
                    print("[Main] Pose reset")

    finally:
        if voice_thread is not None:
            voice_thread.stop()
        grabber.stop()
        grabber.join(timeout=1.0)
        cap.release()
        streamer.stop()
        cv2.destroyAllWindows()
        print("\n[Main] Demo stopped.")


if __name__ == "__main__":
    main()
