# -*- coding: utf-8 -*-
"""Dual-thread ping-pong pipeline demo.

Goal: overlap CPU preprocessing with GPU body inference, and run face/hand
models on NPU. Two worker threads alternate frames; a GPU lock protects only
the body inference call, so one thread can preprocess while the other uses GPU.

Voice KWS runs on CPU (optional).
"""
import argparse
import math
import os
import queue
import statistics
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import openvino as ov

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

# Reuse helpers from the main pipeline
from camera_demo_elf_pipeline import (
    BODY_INPUT_SIZE,
    CAM_HEIGHT,
    CAM_WIDTH,
    DIST_COEFFS,
    FACE_LM_INPUT_SIZE,
    HAND_LM_CONF_THRESHOLD,
    HAND_LM_INPUT_SIZE,
    KPT_CONF_THRESHOLD,
    NMS_THRESHOLD,
    OBJ_THRESHOLD,
    RULE_MODEL_PATH,
    classify_hand_gesture,
    draw_hand_landmarks,
    estimate_face_roi,
    estimate_hand_roi,
    find_camera_index,
    nms_pose,
    preprocess_hand_roi,
)

# Model paths
BODY_MODEL_PATH = "/home/time/work/trial0/models/yolov8s-pose_openvino_model/yolov8s-pose.xml"
FACE_MODEL_PATH = "/home/time/work/mymodel/face_landmark_468.onnx"
HAND_MODEL_PATH = "/home/time/work/mymodel/openvino_pipeline/models/onnx/hand_landmarks_detector.onnx"
HAND_CLS_MODEL_PATH = "/home/time/work/mymodel/openvino_pipeline/model/keypoint_classifier/keypoint_classifier.onnx"
HAND_CLS_LABEL_PATH = "/home/time/work/mymodel/openvino_pipeline/model/keypoint_classifier/keypoint_classifier_label.csv"

SHERPA_MODEL_DIR = "/home/time/work/sherpa/models/sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20"
SHERPA_KEYWORDS_FILE = os.path.join(SCRIPT_DIR, "voice", "keywords.txt")
SHERPA_VAD_MODEL = "/home/time/work/sherpa/models/silero_vad.onnx"


def preprocess_body(frame):
    """CPU preprocess for yolov8s-pose (copied from ElfPipeline)."""
    h, w = frame.shape[:2]
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    scale = BODY_INPUT_SIZE / max(h, w)
    new_w = int(w * scale)
    new_h = int(h * scale)
    resized = cv2.resize(img, (new_w, new_h))
    letterboxed = np.zeros((BODY_INPUT_SIZE, BODY_INPUT_SIZE, 3), dtype=np.float32)
    x_off = (BODY_INPUT_SIZE - new_w) // 2
    y_off = (BODY_INPUT_SIZE - new_h) // 2
    letterboxed[y_off : y_off + new_h, x_off : x_off + new_w] = resized
    inp = np.transpose(letterboxed, (2, 0, 1))[np.newaxis, ...]
    return inp, scale, x_off, y_off


def postprocess_body(output, img_w, img_h, scale, x_off, y_off):
    """CPU postprocess for yolov8s-pose (copied from ElfPipeline)."""
    dets = []
    arr = output[0].T  # [8400, 56]
    for i in range(arr.shape[0]):
        conf = arr[i, 4]
        if conf < OBJ_THRESHOLD:
            continue
        cx, cy, bw, bh = arr[i, 0], arr[i, 1], arr[i, 2], arr[i, 3]
        x1_640 = cx - bw * 0.5
        y1_640 = cy - bh * 0.5
        x2_640 = cx + bw * 0.5
        y2_640 = cy + bh * 0.5
        x1 = (x1_640 - x_off) / scale
        y1 = (y1_640 - y_off) / scale
        x2 = (x2_640 - x_off) / scale
        y2 = (y2_640 - y_off) / scale
        kps = []
        for k in range(17):
            kx = arr[i, 5 + k * 3]
            ky = arr[i, 6 + k * 3]
            kv = arr[i, 7 + k * 3]
            kps.append({"x": (kx - x_off) / scale, "y": (ky - y_off) / scale, "visibility": kv})
        box_w = max(0.0, x2 - x1)
        box_h = max(0.0, y2 - y1)
        if box_w < 10 or box_h < 10:
            continue
        aspect = box_h / box_w
        if aspect < 0.25 or aspect > 4.0:
            continue
        visible_kpts = sum(1 for kp in kps if kp["visibility"] > KPT_CONF_THRESHOLD)
        if visible_kpts < 3:
            continue
        dets.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2, "score": conf, "kps": kps})
    return dets


class DummyTracker:
    """Minimal tracker placeholder so estimate_face_roi can use its fallback path."""

    def __init__(self):
        self.prev_roi_size = 0.0
        self.last_face_w = 0.0
        self.last_face_h = 0.0
        self.face_size_life = 0

    def get_roi_size(self):
        return self.prev_roi_size


class SimplePipeline:
    """Load only the models needed for the ping-pong demo."""

    def __init__(self):
        print("=" * 60)
        print("PingPong Pipeline loading models...")
        print("=" * 60)
        core = ov.Core()
        print(f"Available devices: {core.available_devices}")

        print("[1/4] Body yolov8s-pose on GPU")
        body_model = core.read_model(BODY_MODEL_PATH)
        self.body_compiled = core.compile_model(body_model, "GPU")
        self.body_input_name = self._input_name(self.body_compiled)
        self.body_output_name = self._output_name(self.body_compiled)

        print("[2/4] Face landmark 468 on NPU")
        face_model = core.read_model(FACE_MODEL_PATH)
        self.face_lm_compiled = core.compile_model(face_model, "NPU")
        self.face_lm_input_name = self._input_name(self.face_lm_compiled)
        self.face_lm_output_name = self._output_name(self.face_lm_compiled)

        print("[3/4] Hand landmark on NPU")
        hand_model = core.read_model(HAND_MODEL_PATH)
        self.hand_lm_compiled = core.compile_model(hand_model, "NPU")
        self.hand_lm_input_names = [o.get_any_name() for o in self.hand_lm_compiled.outputs]
        self.hand_lm_input_name = self._input_name(self.hand_lm_compiled)

        print("[4/4] Hand classifier on CPU")
        hand_cls_model = core.read_model(HAND_CLS_MODEL_PATH)
        self.hand_kpt_cls_compiled = core.compile_model(hand_cls_model, "CPU")
        self.hand_kpt_cls_input_name = self._input_name(self.hand_kpt_cls_compiled)
        self.hand_kpt_cls_output_name = self._output_name(self.hand_kpt_cls_compiled)
        with open(HAND_CLS_LABEL_PATH, "r", encoding="utf-8-sig") as f:
            self.hand_kpt_cls_labels = [line.strip() for line in f if line.strip()]
        print(f"      Labels: {self.hand_kpt_cls_labels}")

        print("[5/5] Rule engine on GPU")
        rule_model = core.read_model(RULE_MODEL_PATH)
        self.rule_compiled = core.compile_model(rule_model, "GPU")
        self.rule_input_names = []
        for i in self.rule_compiled.inputs:
            try:
                self.rule_input_names.append(i.get_any_name())
            except Exception:
                self.rule_input_names.append(len(self.rule_input_names))
        self.rule_output_name = self._output_name(self.rule_compiled)
        for name in self.rule_input_names:
            print(f"      Input:  {name}")
        print(f"      Output: {self.rule_output_name}")

        # NPU models share a single infer request; serialize concurrent calls
        # from the two worker threads to avoid "Infer Request is busy".
        self.npu_lock = threading.Lock()
        print("[OK] Models loaded.")

    @staticmethod
    def _input_name(compiled):
        try:
            return compiled.inputs[0].get_any_name()
        except Exception:
            return 0

    @staticmethod
    def _output_name(compiled):
        try:
            return compiled.outputs[0].get_any_name()
        except Exception:
            return 0


def sigmoid(x):
    return 1.0 / (1.0 + math.exp(-float(x)))


def infer_face_landmarks(pipeline, frame, roi):
    """Run face_landmark_468 on NPU for the given ROI."""
    rx, ry, rw, rh = roi
    h, w = frame.shape[:2]
    rx = max(0, rx)
    ry = max(0, ry)
    rw = min(rw, w - rx)
    rh = min(rh, h - ry)
    if rw < 32 or rh < 32:
        return None
    crop = frame[ry : ry + rh, rx : rx + rw]
    rgb = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    resized = cv2.resize(rgb, (FACE_LM_INPUT_SIZE, FACE_LM_INPUT_SIZE))
    inp = np.expand_dims(resized, axis=0)
    with pipeline.npu_lock:
        out = pipeline.face_lm_compiled({pipeline.face_lm_input_name: inp})
    lm = out[pipeline.face_lm_output_name][0].reshape(-1, 3)
    return lm


def detect_hand_landmarks(pipeline, frame, roi):
    """Run hand landmark detector + classifier."""
    input_tensor = preprocess_hand_roi(frame, roi)
    if input_tensor is None:
        return None
    with pipeline.npu_lock:
        out = pipeline.hand_lm_compiled({pipeline.hand_lm_input_name: input_tensor})
    landmarks_px = out[pipeline.hand_lm_input_names[0]][0]
    presence_raw = out[pipeline.hand_lm_input_names[1]][0, 0]
    handedness_raw = out[pipeline.hand_lm_input_names[2]][0, 0]

    presence = sigmoid(presence_raw)
    handedness = sigmoid(handedness_raw)
    if presence < HAND_LM_CONF_THRESHOLD:
        return None

    landmarks_px = landmarks_px.reshape(21, 3)
    rx, ry, rw, rh = roi
    landmarks_img = landmarks_px.copy()
    landmarks_img[:, 0] = landmarks_px[:, 0] / HAND_LM_INPUT_SIZE * rw + rx
    landmarks_img[:, 1] = landmarks_px[:, 1] / HAND_LM_INPUT_SIZE * rh + ry
    landmarks_img[:, 2] = landmarks_px[:, 2] / HAND_LM_INPUT_SIZE * max(rw, rh)

    gesture_label, gesture_id, point_direction = classify_hand_gesture(
        pipeline, landmarks_img
    )
    return {
        "landmarks": landmarks_img,
        "presence": presence,
        "handedness": "Right" if handedness > 0.5 else "Left",
        "gesture_label": gesture_label,
        "gesture_id": gesture_id,
        "point_direction": point_direction,
        "roi": roi,
    }


def pick_best_detection(dets, img_w, img_h):
    """Pick the detection most likely to contain a usable face."""
    if not dets:
        return None
    best = None
    best_score = 1e9
    for det in dets:
        kps = det["kps"]
        has_face = (
            kps[0]["visibility"] > KPT_CONF_THRESHOLD
            and (kps[1]["visibility"] > KPT_CONF_THRESHOLD or kps[2]["visibility"] > KPT_CONF_THRESHOLD)
        )
        cx = (det["x1"] + det["x2"]) * 0.5
        cy = (det["y1"] + det["y2"]) * 0.5
        center_dist = math.hypot(cx - img_w * 0.5, cy - img_h * 0.5)
        score = center_dist + (0 if has_face else img_h * 0.5)
        if score < best_score:
            best_score = score
            best = det
    return best


class FrameStats:
    def __init__(self):
        self.lock = threading.Lock()
        self.records = []
        self.frame_count = 0

    def add(self, total_ms, body_ms, face_ms, hand_ms):
        with self.lock:
            self.records.append(
                {"total": total_ms, "body": body_ms, "face": face_ms, "hand": hand_ms}
            )
            self.frame_count += 1

    def print_report(self):
        with self.lock:
            recent = self.records[-30:]
            if not recent:
                return

            def avg(key):
                return statistics.mean(r[key] for r in recent)

            print("\n" + "=" * 60)
            print(f"[PingPong] last {len(recent)} frames")
            print("-" * 60)
            print(f"  total       : avg={avg('total'):7.2f}ms")
            print(f"  body_total  : avg={avg('body'):7.2f}ms")
            print(f"  face_infer  : avg={avg('face'):7.2f}ms")
            print(f"  hand_infer  : avg={avg('hand'):7.2f}ms")
            print(f"  loop_fps    : avg={1000.0 / avg('total'):7.1f}")
            print("=" * 60)
            self.records = []


def worker_loop(
    worker_id,
    pipeline,
    gpu_lock,
    in_q,
    out_q,
    stats,
    stop_ev,
    img_w,
    img_h,
):
    tracker_dummy = DummyTracker()
    while not stop_ev.is_set():
        try:
            frame_idx, frame = in_q.get(timeout=0.05)
        except queue.Empty:
            continue

        t0 = time.perf_counter()
        # 1) CPU preprocess
        body_inp, scale, x_off, y_off = preprocess_body(frame)
        t_pre = time.perf_counter()

        # 2) GPU inference with minimal lock
        with gpu_lock:
            body_out = pipeline.body_compiled({pipeline.body_input_name: body_inp})
        t_infer = time.perf_counter()

        # 3) CPU postprocess / NMS / ROI estimation
        body_dets = postprocess_body(
            body_out[pipeline.body_output_name], img_w, img_h, scale, x_off, y_off
        )
        body_dets = nms_pose(body_dets, iou_thresh=NMS_THRESHOLD)
        best_det = pick_best_detection(body_dets, img_w, img_h)

        display = frame.copy()
        face_ms = 0.0
        hand_ms = 0.0

        if best_det is not None:
            cv2.rectangle(
                display,
                (int(best_det["x1"]), int(best_det["y1"])),
                (int(best_det["x2"]), int(best_det["y2"])),
                (0, 255, 255),
                2,
            )

            # Face ROI + NPU inference
            t_face0 = time.perf_counter()
            face_roi, _skip_reason = estimate_face_roi(best_det, tracker_dummy, img_w, img_h)
            face_lm = None
            if face_roi is not None:
                rx, ry, rw, rh = face_roi
                cv2.rectangle(display, (rx, ry), (rx + rw, ry + rh), (255, 0, 255), 2)
                face_lm = infer_face_landmarks(pipeline, frame, face_roi)
            face_ms = (time.perf_counter() - t_face0) * 1000.0

            # Hand ROIs + NPU inference
            t_hand0 = time.perf_counter()
            kps = best_det["kps"]
            hand_rois = [
                estimate_hand_roi(kps[9], kps[7], img_w, img_h),
                estimate_hand_roi(kps[10], kps[8], img_w, img_h),
            ]
            hand_colors = [(255, 0, 0), (0, 255, 255)]
            for idx, roi in enumerate(hand_rois):
                if roi is not None:
                    hand_result = detect_hand_landmarks(pipeline, frame, roi)
                    if hand_result is not None:
                        draw_hand_landmarks(display, hand_result, color=hand_colors[idx])
            hand_ms = (time.perf_counter() - t_hand0) * 1000.0

        total_ms = (time.perf_counter() - t0) * 1000.0
        body_ms = (t_infer - t_pre) * 1000.0 + (t_pre - t0) * 1000.0
        stats.add(total_ms, body_ms, face_ms, hand_ms)

        # 4) Push to ordered output
        out_q.put((frame_idx, display))


def output_loop(out_q, stop_ev, stats, headless, frames_limit, output_dir):
    expected_idx = 0
    buffer = {}
    processed = 0
    os.makedirs(output_dir, exist_ok=True)

    while not stop_ev.is_set() or buffer or not out_q.empty():
        try:
            frame_idx, display = out_q.get(timeout=0.05)
        except queue.Empty:
            if not headless:
                # Keep the highgui event loop alive even when no new frames.
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    stop_ev.set()
            continue

        if frame_idx == expected_idx:
            _show(display, frame_idx, headless, output_dir, stop_ev)
            processed += 1
            expected_idx += 1
            while expected_idx in buffer:
                _show(buffer.pop(expected_idx), expected_idx, headless, output_dir, stop_ev)
                processed += 1
                expected_idx += 1
        else:
            buffer[frame_idx] = display
            if not headless:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    stop_ev.set()

        if processed % 30 == 0:
            stats.print_report()

        if headless and frames_limit and processed >= frames_limit:
            stop_ev.set()
            break

    print(f"[Output] Total displayed frames: {processed}")


def _show(display, frame_idx, headless, output_dir, stop_ev):
    if headless:
        if frame_idx % 10 == 0:
            path = os.path.join(output_dir, f"pingpong_{frame_idx:06d}.jpg")
            cv2.imwrite(path, display)
            print(f"[Headless] Saved {path}")
    else:
        cv2.imshow("PingPong Pipeline", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            stop_ev.set()


def capture_loop(cap, in_q, stop_ev):
    frame_idx = 0
    while not stop_ev.is_set():
        ret, frame = cap.read()
        if ret and frame is not None:
            try:
                in_q.put((frame_idx, frame), block=False)
                frame_idx += 1
            except queue.Full:
                pass


def main():
    parser = argparse.ArgumentParser(description="Dual-thread ping-pong pipeline demo")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--output-dir", type=str, default="/tmp/pingpong_frames")
    parser.add_argument("--voice", action="store_true", help="Enable voice KWS")
    parser.add_argument("--voice-provider", type=str, default="cpu", choices=["cpu", "openvino"],
                        help="KWS execution provider (default: cpu)")
    parser.add_argument("--voice-no-vad", action="store_true", help="Disable Silero VAD front-end")
    parser.add_argument("--voice-vad-threshold", type=float, default=None,
                        help="VAD speech threshold (lower = more sensitive)")
    parser.add_argument("--voice-vad-hangover-ms", type=float, default=None,
                        help="VAD hangover after speech ends")
    parser.add_argument("--queue-size", type=int, default=2, help="Input queue size")
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("PingPong dual-thread pipeline demo starting...")
    print("=" * 60)

    pipeline = SimplePipeline()

    cam_idx = find_camera_index()
    cap = cv2.VideoCapture(cam_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] Resolution {actual_w}x{actual_h}")

    gpu_lock = threading.Lock()
    in_q = queue.Queue(maxsize=args.queue_size)
    out_q = queue.Queue()
    stop_ev = threading.Event()
    stats = FrameStats()

    voice_thread = None
    if args.voice:
        from voice import VoiceKwsThread
        voice_thread = VoiceKwsThread(
            model_dir=SHERPA_MODEL_DIR,
            keywords_file=SHERPA_KEYWORDS_FILE,
            provider=args.voice_provider,
            device_id=6,
            use_vad=not args.voice_no_vad,
            vad_model=SHERPA_VAD_MODEL,
            vad_threshold=args.voice_vad_threshold,
            vad_hangover_ms=args.voice_vad_hangover_ms,
        )
        print(f"[Voice] provider={args.voice_provider}, vad={not args.voice_no_vad}")
        voice_thread.start()

    # Start threads
    cap_thread = threading.Thread(target=capture_loop, args=(cap, in_q, stop_ev), daemon=True)
    workers = []
    for i in range(2):
        t = threading.Thread(
            target=worker_loop,
            args=(i, pipeline, gpu_lock, in_q, out_q, stats, stop_ev, actual_w, actual_h),
            daemon=True,
        )
        workers.append(t)
    out_thread = threading.Thread(
        target=output_loop,
        args=(out_q, stop_ev, stats, args.headless, args.frames, args.output_dir),
        daemon=True,
    )

    cap_thread.start()
    for t in workers:
        t.start()
    out_thread.start()

    print("[Main] Press Ctrl-C to stop, or 'q' in window.")
    try:
        if args.headless and args.frames == 0:
            # headless without limit: run for a while then stop
            time.sleep(30)
            stop_ev.set()
        elif not args.headless:
            # window mode: wait until user closes or Ctrl-C
            while not stop_ev.is_set():
                time.sleep(0.05)
        else:
            # headless with frames limit: output_loop will stop when done
            out_thread.join()
    except KeyboardInterrupt:
        print("\n[Main] Stopping...")
    finally:
        stop_ev.set()
        out_thread.join(timeout=2.0)
        if voice_thread is not None:
            voice_thread.stop()
        cap.release()
        cv2.destroyAllWindows()
        stats.print_report()
        print("[Main] Demo stopped.")


if __name__ == "__main__":
    main()
