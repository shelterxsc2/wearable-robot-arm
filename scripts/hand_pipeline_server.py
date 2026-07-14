#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hand Pipeline Python Socket 服务端 —— RKNNLite v2

基于 rknn_models/inference_pipeline.py 封装为 Unix Socket 服务。
接收一帧 RGB 图像，运行手掌检测 + 21 关键点 + 手势分类，返回结果。

协议（小端）：
  Request:
    uint32 width
    uint32 height
    uint32 mode       # 0=detect, 1=landmarks, 2=gesture, 3=keypoint, 4=motion
    uint8  pixels[width*height*3]   # RGB, row-major

  Response (status == 0 时只有这 4 字节):
    uint32 status     # 0=no hand, 1=hand detected

  Response (status == 1 时追加):
    float  bbox[4]        # cx, cy, bw, bh (normalized)
    float  palm_score
    uint32 gesture_len
    char   gesture_label[gesture_len]
    float  gesture_score
    uint32 num_landmarks  # = 21
    float  landmarks[21*3]  # x, y, z in original image coordinates
"""

import argparse
import os
import socket
import struct
import sys
import traceback
from pathlib import Path

import cv2
import numpy as np

try:
    from rknnlite.api import RKNNLite
except ImportError:
    print("[ERROR] 未安装 RKNN-Toolkit-Lite2。请执行：pip install rknn-toolkit-lite2==2.3.2")
    sys.exit(1)


CANNED_GESTURE_LABELS = [
    "None",
    "Closed_Fist",
    "Open_Palm",
    "Pointing_Up",
    "Thumb_Down",
    "Thumb_Up",
    "Victory",
    "ILoveYou",
]
KEYPOINT_LABELS = ["Open", "Close", "Pointer", "OK"]
MOTION_LABELS = ["Static", "Clockwise", "Counter", "Move"]
GESTURE_LOG_PATH = "/tmp/hand_pipeline_gesture.log"
QUIET = False


def gesture_log(message: str):
    if QUIET:
        return
    print(message, flush=True)
    try:
        with open(GESTURE_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(message + "\n")
    except OSError:
        pass


class RKNNModel:
    def __init__(self, model_path: str, core_mask=RKNNLite.NPU_CORE_0_1_2):
        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            raise RuntimeError(f"加载 RKNN 模型失败: {model_path}")
        ret = self.rknn.init_runtime(core_mask=core_mask)
        if ret != 0:
            raise RuntimeError(f"初始化 RKNN runtime 失败: {model_path}")
        print(f"[OK] RKNN 模型已加载: {model_path}")

    def infer(self, input_data: np.ndarray) -> list:
        return self.rknn.inference(inputs=[input_data])

    def release(self):
        self.rknn.release()


def generate_palm_anchors(input_size=192):
    anchors = []
    layers = [
        {"stride": 8, "grid": 24, "num_anchors": 2},
        {"stride": 16, "grid": 12, "num_anchors": 6},
    ]
    for layer in layers:
        grid = layer["grid"]
        for y in range(grid):
            for x in range(grid):
                for _ in range(layer["num_anchors"]):
                    anchors.append({
                        "x_center": (x + 0.5) / grid,
                        "y_center": (y + 0.5) / grid,
                        "w": 1.0,
                        "h": 1.0,
                    })
    return anchors


class HandPipeline:
    def __init__(self, model_dir: Path, mode: str = "gesture"):
        self.model_dir = model_dir
        self.mode = mode
        self.anchors = generate_palm_anchors(192)

        if mode != "roi_gesture":
            self.palm_rknn = RKNNModel(str(model_dir / "hand_detector_fp16.rknn"))

        if mode in ("landmarks", "gesture", "keypoint", "motion", "roi_gesture"):
            self.landmark_rknn = RKNNModel(str(model_dir / "hand_landmarks_detector_fp16.rknn"))

        if mode in ("gesture", "roi_gesture"):
            self.gesture_embedder_rknn = RKNNModel(str(model_dir / "gesture_embedder_fp16.rknn"))
            self.canned_gesture_rknn = RKNNModel(str(model_dir / "canned_gesture_classifier_fp16.rknn"))

        if mode == "keypoint":
            self.keypoint_rknn = RKNNModel(str(model_dir / "keypoint_classifier_fp16.rknn"))
            self.keypoint_score_ema = None
            self.keypoint_label = ""
            self.keypoint_candidate = ""
            self.keypoint_candidate_count = 0
            self.keypoint_debounce_frames = 3
            self.keypoint_conf_threshold = 0.45

        if mode == "motion":
            self.motion_rknn = RKNNModel(str(model_dir / "point_history_classifier_fp16.rknn"))
            self.point_history = []

    def preprocess_palm(self, image: np.ndarray):
        h, w = image.shape[:2]
        # 输入约定为 RGB uint8
        rgb = image if (image.dtype == np.uint8 and image.shape[2] == 3) else image
        scale = 192 / max(h, w)
        new_w = int(w * scale)
        new_h = int(h * scale)
        resized = cv2.resize(rgb, (new_w, new_h))
        letterboxed = np.zeros((192, 192, 3), dtype=np.uint8)
        x_off = (192 - new_w) // 2
        y_off = (192 - new_h) // 2
        letterboxed[y_off:y_off + new_h, x_off:x_off + new_w] = resized
        return np.expand_dims(letterboxed, axis=0), {
            "scale": scale, "x_off": x_off, "y_off": y_off,
            "new_w": new_w, "new_h": new_h, "orig_w": w, "orig_h": h,
        }

    def detect_palm(self, image: np.ndarray):
        input_tensor, lb = self.preprocess_palm(image)
        outputs = self.palm_rknn.infer(input_tensor)
        detections, scores = outputs[0][0], outputs[1][0].flatten()
        scores_clipped = np.clip(scores, -50.0, 50.0)
        scores_sigmoid = 1.0 / (1.0 + np.exp(-scores_clipped))

        valid = np.where(scores_sigmoid >= 0.5)[0]
        if len(valid) == 0:
            return None

        best_idx = valid[np.argmax(scores_sigmoid[valid])]
        best_det = detections[best_idx]
        anchor = self.anchors[best_idx]

        x_scale = y_scale = w_scale = h_scale = 192.0
        cx_192 = best_det[0] / x_scale * anchor["w"] + anchor["x_center"]
        cy_192 = best_det[1] / y_scale * anchor["h"] + anchor["y_center"]
        bw_192 = best_det[2] / w_scale * anchor["w"]
        bh_192 = best_det[3] / h_scale * anchor["h"]

        w, h = lb["orig_w"], lb["orig_h"]
        x_off, y_off = lb["x_off"], lb["y_off"]
        new_w, new_h = lb["new_w"], lb["new_h"]

        x1_192 = (cx_192 - bw_192 / 2) * 192.0
        y1_192 = (cy_192 - bh_192 / 2) * 192.0
        x2_192 = (cx_192 + bw_192 / 2) * 192.0
        y2_192 = (cy_192 + bh_192 / 2) * 192.0

        x1 = (x1_192 - x_off) / new_w * w
        y1 = (y1_192 - y_off) / new_h * h
        x2 = (x2_192 - x_off) / new_w * w
        y2 = (y2_192 - y_off) / new_h * h

        cx = (x1 + x2) / 2 / w
        cy = (y1 + y2) / 2 / h
        bw = (x2 - x1) / w
        bh = (y2 - y1) / h

        # 限制在合理范围内
        cx = np.clip(cx, 0.0, 1.0)
        cy = np.clip(cy, 0.0, 1.0)
        bw = np.clip(bw, 0.0, 1.0)
        bh = np.clip(bh, 0.0, 1.0)

        palm_keypoints = []
        for k in range(7):
            kpx = best_det[4 + k * 2] / x_scale * anchor["w"] + anchor["x_center"]
            kpy = best_det[5 + k * 2] / y_scale * anchor["h"] + anchor["y_center"]
            palm_keypoints.append([
                (kpx * 192.0 - x_off) / new_w,
                (kpy * 192.0 - y_off) / new_h,
            ])

        return {
            "bbox": np.array([cx, cy, bw, bh]),
            "score": float(scores_sigmoid[best_idx]),
            "palm_keypoints": np.array(palm_keypoints),
        }

    def extract_roi(self, image: np.ndarray, detection: dict):
        h, w = image.shape[:2]
        cx, cy, bw, bh = detection["bbox"]
        palm_keypoints = detection["palm_keypoints"]

        wrist = palm_keypoints[0]
        middle_mcp = palm_keypoints[2]

        dx = middle_mcp[0] - wrist[0]
        dy = middle_mcp[1] - wrist[1]
        angle = np.arctan2(dy, dx)
        rotation = -np.pi / 2 - angle

        rect_cx = (wrist[0] + middle_mcp[0]) / 2
        rect_cy = (wrist[1] + middle_mcp[1]) / 2

        scale_x, scale_y = 2.6, 2.6
        shift_y = -0.5
        new_w = bw * scale_x
        new_h = bh * scale_y
        long_side = max(new_w, new_h)

        center_x = rect_cx * w
        center_y = (rect_cy + shift_y * bh) * h
        half_size = (long_side * max(w, h)) / 2

        angle_deg = rotation * 180 / np.pi
        M = cv2.getRotationMatrix2D((center_x, center_y), angle_deg, 1.0)
        rotated = cv2.warpAffine(image, M, (w, h), borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0))

        x1 = int(max(0, center_x - half_size))
        y1 = int(max(0, center_y - half_size))
        x2 = int(min(w, center_x + half_size))
        y2 = int(min(h, center_y + half_size))
        crop = rotated[y1:y2, x1:x2]

        if crop.size == 0:
            x1 = int(max(0, (cx - new_w / 2) * w))
            y1 = int(max(0, (cy - new_h / 2) * h))
            x2 = int(min(w, (cx + new_w / 2) * w))
            y2 = int(min(h, (cy + new_h / 2) * h))
            if x2 <= x1:
                x2 = min(w, x1 + 32)
                x1 = max(0, x2 - 32)
            if y2 <= y1:
                y2 = min(h, y1 + 32)
                y1 = max(0, y2 - 32)
            crop = image[y1:y2, x1:x2]

        if crop.size == 0:
            return None, None

        roi = cv2.resize(crop, (224, 224))
        return roi, {
            "crop_x1": x1, "crop_y1": y1, "crop_w": x2 - x1, "crop_h": y2 - y1,
            "rotation": rotation, "center_x": center_x, "center_y": center_y,
            "angle_deg": angle_deg,
        }

    def detect_landmarks(self, roi: np.ndarray):
        # 输入约定为 RGB uint8
        rgb = roi if (roi.dtype == np.uint8 and roi.shape[2] == 3) else roi
        input_tensor = np.expand_dims(rgb, axis=0).astype(np.uint8)
        outputs = self.landmark_rknn.infer(input_tensor)

        landmarks_px = outputs[0][0].reshape(21, 3)
        presence = float(1.0 / (1.0 + np.exp(-outputs[1][0, 0])))
        handedness = float(1.0 / (1.0 + np.exp(-outputs[2][0, 0])))
        world = outputs[3][0].reshape(21, 3)

        landmarks_norm = landmarks_px.copy()
        landmarks_norm[:, :2] /= 224.0

        return {
            "landmarks_norm": landmarks_norm,
            "landmarks_px": landmarks_px,
            "presence": presence,
            "handedness": "Right" if handedness > 0.5 else "Left",
            "world": world,
        }

    def map_to_original(self, landmarks_norm: np.ndarray, tf_info: dict):
        landmarks = landmarks_norm.copy()
        landmarks[:, 0] *= tf_info["crop_w"]
        landmarks[:, 1] *= tf_info["crop_h"]
        landmarks[:, 0] += tf_info["crop_x1"]
        landmarks[:, 1] += tf_info["crop_y1"]

        M_inv = cv2.getRotationMatrix2D(
            (tf_info["center_x"], tf_info["center_y"]), -tf_info["angle_deg"], 1.0
        )
        points = landmarks[:, :2].reshape(-1, 1, 2)
        rotated_back = cv2.transform(points, M_inv)
        landmarks[:, :2] = rotated_back.reshape(-1, 2)
        return landmarks

    def preprocess_gesture_landmarks(self, landmarks: np.ndarray):
        hand = landmarks.astype(np.float32).copy()
        hand -= hand[0]
        scale = max(
            float(np.max(hand[:, 0]) - np.min(hand[:, 0])),
            float(np.max(hand[:, 1]) - np.min(hand[:, 1])),
        )
        if scale < 1e-5:
            scale = 1e-5
        hand /= scale
        return hand

    def run_canned_gesture(self, hand: np.ndarray):
        input_tensor = np.expand_dims(hand, axis=0)
        embedding = self.gesture_embedder_rknn.infer(input_tensor)[0]
        scores = self.canned_gesture_rknn.infer(embedding.astype(np.float32))[0][0]
        idx = int(np.argmax(scores))
        label = CANNED_GESTURE_LABELS[idx] if idx < len(CANNED_GESTURE_LABELS) else f"class_{idx}"
        order = np.argsort(scores)[::-1]
        second = int(order[1]) if len(order) > 1 else idx
        margin = float(scores[idx] - scores[second])
        return label, float(scores[idx]), margin, scores

    def classify_gesture(self, landmarks_px: np.ndarray, world: np.ndarray):
        screen01 = landmarks_px.astype(np.float32).copy()
        screen01[:, :2] /= 224.0
        screen01[:, 2] /= 224.0
        candidates = [
            ("screen01", screen01),
            ("screen_center", self.preprocess_gesture_landmarks(landmarks_px)),
        ]
        results = []
        for name, hand in candidates:
            label, score, margin, scores = self.run_canned_gesture(hand)
            results.append((name, label, score, margin, scores))
        gesture_log("[CannedGestureCandidates] " + " | ".join(
            f"{name}:{label} {score:.3f}/{margin:.3f}"
            for name, label, score, margin, _ in results
        ))
        valid = [r for r in results if r[1] != "None" and r[2] >= 0.45 and r[3] >= 0.08]
        best = max(valid if valid else results, key=lambda r: (r[2], r[3]))
        name, label, score, margin, scores = best
        gesture_log(f"[CannedGestureSelected] {name}:{label} score={score:.3f} margin={margin:.3f} raw={scores}")
        return label, score, scores

    def classify_keypoint(self, landmarks_norm: np.ndarray):
        # MediaPipe 标准归一化：以手腕(landmark 0)为原点，缩放到 [-1, 1]
        kpts = landmarks_norm[:, :2].copy()
        kpts *= 224.0                 # 先转回 224x224 像素坐标
        kpts -= kpts[0]               # 平移到手腕为原点
        flat = kpts.flatten().astype(np.float32)
        max_val = np.max(np.abs(flat))
        if max_val > 1e-6:
            flat /= max_val           # 缩放到 [-1, 1]
        input_tensor = np.expand_dims(flat, axis=0)
        scores = self.keypoint_rknn.infer(input_tensor)[0][0]
        idx = int(np.argmax(scores))
        return KEYPOINT_LABELS[idx], float(scores[idx]), scores

    def smooth_keypoint(self, label: str, score: float, scores: np.ndarray):
        """EMA 分数平滑 + 候选标签消抖 + 置信度阈值"""
        if self.keypoint_score_ema is None:
            self.keypoint_score_ema = np.array(scores, dtype=np.float32)
        else:
            alpha = 0.3  # EMA 系数，越小越平滑
            self.keypoint_score_ema = alpha * np.array(scores, dtype=np.float32) + (1 - alpha) * self.keypoint_score_ema

        smooth_idx = int(np.argmax(self.keypoint_score_ema))
        smooth_label = KEYPOINT_LABELS[smooth_idx]
        smooth_score = float(self.keypoint_score_ema[smooth_idx])

        if smooth_score < self.keypoint_conf_threshold:
            # 置信度不足，保持上一帧标签
            return self.keypoint_label, 0.0, self.keypoint_score_ema

        if smooth_label == self.keypoint_candidate:
            self.keypoint_candidate_count += 1
        else:
            self.keypoint_candidate = smooth_label
            self.keypoint_candidate_count = 1

        if self.keypoint_candidate_count >= self.keypoint_debounce_frames:
            self.keypoint_label = self.keypoint_candidate

        out_score = smooth_score if self.keypoint_label == smooth_label else 0.0
        if self.keypoint_label != getattr(self, '_last_printed_label', None):
            print(f"[Keypoint] raw={scores} ema={self.keypoint_score_ema} -> {self.keypoint_label}:{out_score:.3f}", flush=True)
            self._last_printed_label = self.keypoint_label
        return self.keypoint_label, out_score, self.keypoint_score_ema

    def classify_motion(self, landmarks_norm: np.ndarray):
        index_tip = landmarks_norm[8, :2]
        self.point_history.append(index_tip.tolist())
        if len(self.point_history) < 16:
            return None, 0.0, None

        history = np.array(self.point_history[-16:]).flatten().astype(np.float32)
        input_tensor = np.expand_dims(history, axis=0)
        scores = self.motion_rknn.infer(input_tensor)[0][0]
        idx = int(np.argmax(scores))
        return MOTION_LABELS[idx], float(scores[idx]), scores

    def process(self, image: np.ndarray):
        if self.mode == "roi_gesture":
            roi = cv2.resize(image, (224, 224))
            lm_result = self.detect_landmarks(roi)
            if lm_result["presence"] < 0.5:
                return None
            return {
                "palm": {"bbox": np.array([0.5, 0.5, 1.0, 1.0]),
                         "score": lm_result["presence"]},
                "landmarks": lm_result["landmarks_px"],
                "world": lm_result["world"],
                "handedness": lm_result["handedness"],
                "presence": lm_result["presence"],
                "gesture": self.classify_gesture(lm_result["landmarks_px"], lm_result["world"]),
            }
        detection = self.detect_palm(image)
        if detection is None:
            return None

        if self.mode == "detect":
            return {"palm": detection}

        roi, tf_info = self.extract_roi(image, detection)
        if roi is None or tf_info is None:
            return None
        lm_result = self.detect_landmarks(roi)
        if lm_result["presence"] < 0.5:
            return None

        landmarks_orig = self.map_to_original(lm_result["landmarks_norm"], tf_info)

        result = {
            "palm": detection,
            "landmarks": landmarks_orig,
            "world": lm_result["world"],
            "handedness": lm_result["handedness"],
            "presence": lm_result["presence"],
        }

        if self.mode == "gesture":
            result["gesture"] = self.classify_gesture(lm_result["landmarks_px"], lm_result["world"])

        if self.mode == "keypoint":
            label, score, scores = self.classify_keypoint(lm_result["landmarks_norm"])
            result["gesture"] = self.smooth_keypoint(label, score, scores)
            result["keypoint_gesture"] = result["gesture"]

        if self.mode == "motion":
            result["gesture"] = self.classify_motion(lm_result["landmarks_norm"])
            result["motion"] = result["gesture"]

        return result

    def release(self):
        if hasattr(self, "palm_rknn"):
            self.palm_rknn.release()
        if hasattr(self, "landmark_rknn"):
            self.landmark_rknn.release()
        if hasattr(self, "gesture_embedder_rknn"):
            self.gesture_embedder_rknn.release()
        if hasattr(self, "canned_gesture_rknn"):
            self.canned_gesture_rknn.release()
        if hasattr(self, "keypoint_rknn"):
            self.keypoint_rknn.release()
        if hasattr(self, "motion_rknn"):
            self.motion_rknn.release()


def recvall(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def handle_client(conn, pipeline: HandPipeline):
    try:
        while True:
            header = recvall(conn, 12)
            if header is None:
                break
            width, height, mode = struct.unpack("<III", header)
            expected = width * height * 3
            if expected == 0 or expected > 1920 * 1080 * 3:
                print(f"[WARN] Invalid frame size: {width}x{height}")
                break

            pixel_data = recvall(conn, expected)
            if pixel_data is None:
                break

            image = np.frombuffer(pixel_data, dtype=np.uint8).reshape((height, width, 3))

            mode_map = {0: "detect", 1: "landmarks", 2: "gesture", 3: "keypoint",
                        4: "motion", 5: "roi_gesture"}
            req_mode = mode_map.get(mode, pipeline.mode)
            if req_mode != pipeline.mode:
                # 不同模式需要重新初始化，这里简单忽略或提示
                print(f"[WARN] Requested mode {req_mode} but pipeline is {pipeline.mode}")

            result = pipeline.process(image)

            if result is None:
                conn.sendall(struct.pack("<I", 0))
                continue

            palm = result["palm"]
            bbox = palm["bbox"]
            palm_score = palm["score"]

            gesture_label = ""
            gesture_score = 0.0
            if "gesture" in result and result["gesture"]:
                gesture_label, gesture_score, _ = result["gesture"]
            elif "keypoint_gesture" in result and result["keypoint_gesture"]:
                gesture_label, gesture_score, _ = result["keypoint_gesture"]
            elif "motion" in result and result["motion"]:
                gesture_label, gesture_score, _ = result["motion"]

            landmarks = result["landmarks"]
            num_lm = landmarks.shape[0]

            payload = struct.pack("<I", 1)
            payload += struct.pack("<ffff", float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3]))
            payload += struct.pack("<f", float(palm_score))
            payload += struct.pack("<I", len(gesture_label))
            payload += gesture_label.encode("utf-8")
            payload += struct.pack("<f", float(gesture_score))
            payload += struct.pack("<I", num_lm)
            lm_flat = landmarks.flatten().astype(np.float32)
            payload += lm_flat.tobytes()

            conn.sendall(payload)
    except Exception as e:
        print(f"[HandPipeline] Client handler error: {e}")
        traceback.print_exc()
    finally:
        conn.close()


def main():
    global QUIET
    parser = argparse.ArgumentParser(description="RK3588 Hand Pipeline Socket Server")
    default_models = Path(__file__).resolve().parent.parent / "models" / "hand"
    parser.add_argument("--model-dir", default=str(default_models),
                        help="RKNN 模型所在目录")
    parser.add_argument("--mode", default="gesture",
                        choices=["detect", "landmarks", "gesture", "keypoint", "motion", "roi_gesture"],
                        help="运行模式")
    parser.add_argument("--sock", default="/tmp/hand_pipeline.sock",
                        help="Unix socket 路径")
    parser.add_argument("--quiet", action="store_true",
                        help="关闭逐帧分类日志，供生产旁路使用")
    args = parser.parse_args()
    QUIET = args.quiet

    model_dir = Path(args.model_dir).resolve()
    if not model_dir.exists():
        print(f"[ERROR] Model dir not found: {model_dir}")
        sys.exit(1)

    print("=" * 60)
    print(f"[HandPipeline] mode={args.mode} | model_dir={model_dir}")
    print(f"[HandPipeline] socket={args.sock}")
    print("=" * 60)

    pipeline = HandPipeline(model_dir, mode=args.mode)

    if os.path.exists(args.sock):
        os.remove(args.sock)

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(args.sock)
    s.listen(5)
    os.chmod(args.sock, 0o666)
    print(f"[HandPipeline] Listening on {args.sock}")

    try:
        while True:
            conn, _ = s.accept()
            print("[HandPipeline] Client connected")
            handle_client(conn, pipeline)
            print("[HandPipeline] Client disconnected")
    except KeyboardInterrupt:
        print("\n[HandPipeline] Shutting down...")
    finally:
        s.close()
        if os.path.exists(args.sock):
            os.remove(args.sock)
        pipeline.release()
        print("[HandPipeline] Released")


if __name__ == "__main__":
    main()
