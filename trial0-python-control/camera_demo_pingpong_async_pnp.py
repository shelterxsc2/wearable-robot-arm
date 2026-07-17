# -*- coding: utf-8 -*-
"""Ping-pong pipeline with async body inference + PnP head pose.

Architecture:
    capture -> raw_q
    2x preprocessor -> body_in_q
    body_gpu_thread (async request pool) -> body_done_q
    1x postprocessor (body post + face/hand NPU + PnP + draw) -> out_q
    output_loop (reorder by frame index) -> display/save

Frame order is preserved for PnP state by emitting body results in submission
order and using a single postprocessor thread.
"""
import argparse
import math
import os
import queue
import signal
import statistics
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np
import openvino as ov

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

_g_stop_ev = None


def _signal_handler(signum, _frame):
    print(f"\n[Main] Received signal {signum}, stopping...")
    if _g_stop_ev is not None:
        _g_stop_ev.set()

# Reuse visual helpers from the original pingpong demo
from camera_demo_pingpong import (
    BODY_MODEL_PATH,
    CAM_HEIGHT,
    CAM_WIDTH,
    FACE_MODEL_PATH,
    HAND_CLS_LABEL_PATH,
    HAND_CLS_MODEL_PATH,
    HAND_MODEL_PATH,
    KPT_CONF_THRESHOLD,
    NMS_THRESHOLD,
    SHERPA_KEYWORDS_FILE,
    SHERPA_MODEL_DIR,
    SHERPA_VAD_MODEL,
    SimplePipeline,
    capture_loop,
    find_camera_index,
    nms_pose,
    postprocess_body,
    preprocess_body,
)
from camera_demo_elf_pipeline import (
    DIST_COEFFS,
    FACE_LM_12_IDS,
    FACE_LM_INPUT_SIZE,
    HAND_CROP_MIN_PRESENCE,
    FaceTracker,
    PoseEstimator,
    RuleEngineState,
    detect_hand_landmarks,
    draw_hand_landmarks,
    estimate_face_roi,
    estimate_hand_roi,
)
from elf_control_chain import (
    make_elf_control_thread,
)
from system_init import init_bluetooth_hci, start_ble_remote, start_wifi_thread
from lowlatency_streamer import (
    LowLatencyStreamer,
    get_default_rtmp_url,
    get_default_rtsp_url,
    get_default_ws_url,
)


# Feed rule/mode inference transitions into arm control.
ENABLE_RULE_MODE_CONTROL = True

# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------


@dataclass
class BodyJob:
    frame_idx: int
    frame: np.ndarray
    body_inp: np.ndarray
    scale: float
    x_off: int
    y_off: int


@dataclass
class BodyDone:
    frame_idx: int
    frame: np.ndarray
    body_out: np.ndarray
    scale: float
    x_off: int
    y_off: int
    body_ms: float  # time spent in GPU body inference for this frame


# ---------------------------------------------------------------------------
# Frame stats
# ---------------------------------------------------------------------------


class FrameStats:
    def __init__(self):
        self.lock = threading.Lock()
        self.records = []
        self.frame_count = 0

    def add(self, total_ms, body_ms, face_ms, hand_ms, pnp_ms, rule_ms):
        with self.lock:
            self.frame_count += 1

    def print_report(self):
        return


# ---------------------------------------------------------------------------
# Async body inference thread
# ---------------------------------------------------------------------------


class BodyGpuThread(threading.Thread):
    """Keep N async body infer requests in flight and emit results in order."""

    def __init__(self, body_compiled, body_input_name, body_in_q, body_done_q, stop_ev, pool_size=3):
        super().__init__(daemon=True)
        self.body_compiled = body_compiled
        self.body_input_name = body_input_name
        self.body_in_q = body_in_q
        self.body_done_q = body_done_q
        self.stop_ev = stop_ev
        self.pool_size = pool_size
        self._requests = [body_compiled.create_infer_request() for _ in range(pool_size)]
        self._available = queue.Queue()
        for r in self._requests:
            self._available.put(r)
        self._pending: list[tuple[BodyJob, ov.InferRequest, float]] = []

    def _submit_one(self, job: BodyJob):
        req = self._available.get()
        req.set_input_tensor(ov.Tensor(job.body_inp))
        req.start_async()
        self._pending.append((job, req, time.perf_counter()))

    def _emit_first_if_done(self) -> bool:
        if not self._pending:
            return False
        job, req, t0 = self._pending[0]
        # Wait for the earliest submitted request to finish so results stay ordered.
        req.wait()
        self._pending.pop(0)
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        out = np.array(req.get_output_tensor().data)
        self._available.put(req)
        self.body_done_q.put(
            BodyDone(job.frame_idx, job.frame, out, job.scale, job.x_off, job.y_off, elapsed_ms)
        )
        return True

    def run(self):
        while not self.stop_ev.is_set():
            # Fill the pool with new jobs
            while len(self._pending) < self.pool_size:
                try:
                    job = self.body_in_q.get(timeout=0.001)
                except queue.Empty:
                    break
                self._submit_one(job)

            # Emit the earliest result (blocks until it's ready)
            if self._pending:
                self._emit_first_if_done()
            else:
                time.sleep(0.001)

        # Drain remaining pending requests in order
        while self._pending:
            self._emit_first_if_done()


# ---------------------------------------------------------------------------
# Preprocessor / postprocessor threads
# ---------------------------------------------------------------------------


class PreprocessorThread(threading.Thread):
    def __init__(self, raw_q, body_in_q, stop_ev):
        super().__init__(daemon=True)
        self.raw_q = raw_q
        self.body_in_q = body_in_q
        self.stop_ev = stop_ev

    def run(self):
        while not self.stop_ev.is_set():
            try:
                frame_idx, frame = self.raw_q.get(timeout=0.05)
            except queue.Empty:
                continue
            body_inp, scale, x_off, y_off = preprocess_body(frame)
            self.body_in_q.put(BodyJob(frame_idx, frame, body_inp, scale, x_off, y_off))


class AnnotationMode:
    """Thread-safe switch between annotated output and the untouched camera frame."""

    def __init__(self):
        self._enabled = True
        self._lock = threading.Lock()

    def set_enabled(self, enabled: bool) -> bool:
        with self._lock:
            changed = self._enabled != enabled
            self._enabled = enabled
            return changed

    def is_enabled(self) -> bool:
        with self._lock:
            return self._enabled


class PostprocessorThread(threading.Thread):
    def __init__(self, pipeline, body_done_q, out_q, stop_ev, stats, img_w, img_h,
                 elf_thread=None, annotation_mode=None):
        super().__init__(daemon=True)
        self.pipeline = pipeline
        self.body_done_q = body_done_q
        self.out_q = out_q
        self.stop_ev = stop_ev
        self.stats = stats
        self.img_w = img_w
        self.img_h = img_h
        self.elf_thread = elf_thread
        self.annotation_mode = annotation_mode
        self.tracker = FaceTracker()
        self.pose_est = PoseEstimator()
        self.rule_engine_state = RuleEngineState()
        self.face_fail_count = 0
        self._last_queue_warning_s = 0.0
        self._gesture_candidate = None
        self._gesture_count = 0
        self._last_gesture_trigger_s = 0.0
        self._prev_rule_mode = None

    def run(self):
        while not self.stop_ev.is_set() or not self.body_done_q.empty():
            try:
                done = self.body_done_q.get(timeout=0.05)
            except queue.Empty:
                continue
            self._process(done)

    def _process(self, done: BodyDone):
        t0 = time.perf_counter()
        backlog = max(self.body_done_q.qsize(), self.out_q.qsize())
        if backlog >= 5 and t0 - self._last_queue_warning_s >= 1.0:
            print(
                f"[QUEUE-BACKLOG] body_done={self.body_done_q.qsize()} "
                f"out={self.out_q.qsize()}"
            )
            self._last_queue_warning_s = t0
        body_dets = postprocess_body(
            done.body_out, self.img_w, self.img_h, done.scale, done.x_off, done.y_off
        )
        body_dets = nms_pose(body_dets, iou_thresh=NMS_THRESHOLD)
        body_dets.sort(key=lambda d: -d["score"])

        # Default to face-like pipeline if no control thread or no scenario mode.
        mode = "face"

        # Scenario modes run on body detections and bypass face/hand PnP pipeline.
        # The vision thread only computes the target; the actual UART TX is
        # scheduled by ElfControlThread._tick() at the fixed 50 Hz control rate.
        if self.elf_thread is not None:
            try:
                mode = self.elf_thread.get_mode()
                now_us = int(time.perf_counter() * 1_000_000)
                cmd = None
                if mode == "intro":
                    cmd = self.elf_thread.controller.update_intro_control(
                        body_dets, self.img_w, self.img_h, now_us=now_us
                    )
                elif mode == "interview":
                    cmd = self.elf_thread.controller.update_interview_control(
                        body_dets, self.img_w, self.img_h, now_us=now_us
                    )
                if cmd is not None:
                    # Keep only the latest scenario target to avoid stale backlog.
                    sc_q = self.elf_thread.scenario_cmd_q
                    while not sc_q.empty():
                        try:
                            sc_q.get_nowait()
                        except queue.Empty:
                            break
                    sc_q.put_nowait({"type": "arm_cmd", "cmd": cmd})
            except queue.Full:
                pass
            except Exception as e:
                print(f"[ElfControl] Scenario mode error: {e}")

        display = done.frame.copy()
        face_ms = 0.0
        hand_ms = 0.0
        pnp_ms = 0.0
        rule_ms = 0.0

        # Draw all detections (same as ELF: top 3)
        draw_limit = min(3, len(body_dets))
        for det in body_dets[:draw_limit]:
            cv2.rectangle(
                display,
                (int(det["x1"]), int(det["y1"])),
                (int(det["x2"]), int(det["y2"])),
                (0, 255, 255),
                1,
            )
            cv2.putText(
                display,
                f"{det['score']:.2f}",
                (int(det["x1"]), int(det["y1"]) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                1,
            )
            for k, kp in enumerate(det["kps"]):
                if kp["visibility"] > KPT_CONF_THRESHOLD:
                    cv2.circle(display, (int(kp["x"]), int(kp["y"])), 2, (0, 255, 0), -1)

        # Overlay current scenario mode for debugging.
        if mode != "face":
            cv2.putText(
                display, f"Mode: {mode.upper()}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2
            )

        matched_idx = self.tracker.update(
            body_dets, self.img_w, self.img_h, center_large=(mode == "face")
        )

        # Base runs Face/PnP only in FACE. BODY and both scenario modes use
        # body detections only; scenario commands were already computed above.
        # In scenario modes, keep a narrow hand path alive so OK can return to FACE.
        if mode in ("body", "intro", "interview", "first_person"):
            if mode in ("intro", "interview"):
                t_rule0 = time.perf_counter()
                rule_state = self._infer_rule_for_body_only(body_dets)
                if rule_state is not None:
                    self._handle_rule_mode_transition(int(rule_state[0]))
                    cv2.putText(
                        display, f"RULE mode={int(rule_state[0])}", (10, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
                    )
                rule_ms = (time.perf_counter() - t_rule0) * 1000.0

                t_hand0 = time.perf_counter()
                self._handle_scenario_ok_gesture(body_dets, done.frame, display)
                hand_ms = (time.perf_counter() - t_hand0) * 1000.0
            total_ms = (time.perf_counter() - t0) * 1000.0
            self.stats.add(total_ms, done.body_ms, face_ms, hand_ms, pnp_ms, rule_ms)
            output_frame = (
                display
                if self.annotation_mode is None or self.annotation_mode.is_enabled()
                else done.frame
            )
            self.out_q.put((done.frame_idx, output_frame))
            return

        if 0 <= matched_idx < len(body_dets):
            best_det = body_dets[matched_idx]

            # ---------- Face ROI + Landmark 468 ----------
            t_face0 = time.perf_counter()
            face_roi, skip_reason = estimate_face_roi(
                best_det, self.tracker, self.img_w, self.img_h
            )
            face_lm = None
            face_lm_img = None
            face_success_this_frame = False
            if face_roi is not None:
                rx, ry, rw, rh = face_roi
                cv2.rectangle(display, (rx, ry), (rx + rw, ry + rh), (255, 0, 255), 2)
                face_lm = self._infer_face_lm(done.frame, face_roi)
                if face_lm is not None:
                    xs = face_lm[:, 0]
                    ys = face_lm[:, 1]
                    lm_min_x, lm_max_x = xs.min(), xs.max()
                    lm_min_y, lm_max_y = ys.min(), ys.max()
                    face_w_ratio = (lm_max_x - lm_min_x) / FACE_LM_INPUT_SIZE
                    face_h_ratio = (lm_max_y - lm_min_y) / FACE_LM_INPUT_SIZE

                    if 0.22 <= face_w_ratio <= 0.92 and 0.22 <= face_h_ratio <= 0.92:
                        face_w_img = (lm_max_x - lm_min_x) * (rw / FACE_LM_INPUT_SIZE)
                        face_h_img = (lm_max_y - lm_min_y) * (rh / FACE_LM_INPUT_SIZE)
                        self.tracker.update_face_size(face_w_img, face_h_img)

                        lm_scale_x = rw / FACE_LM_INPUT_SIZE
                        lm_scale_y = rh / FACE_LM_INPUT_SIZE
                        face_lm_img = [
                            (face_lm[i, 0] * lm_scale_x + rx,
                             face_lm[i, 1] * lm_scale_y + ry)
                            for i in range(len(face_lm))
                        ]

                        t_pnp0 = time.perf_counter()
                        face_lm_2d = [face_lm_img[i] for i in FACE_LM_12_IDS]
                        ok, info = self.pose_est.estimate_and_draw(display, face_lm_2d)
                        pnp_ms = (time.perf_counter() - t_pnp0) * 1000.0
                        if ok:
                            face_success_this_frame = True
                            cv2.putText(
                                display, "PnP OK", (self.img_w - 120, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
                            )
                            # Feed visual PnP correction into the ELF control chain
                            if self.elf_thread is not None:
                                try:
                                    controller = self.elf_thread.controller
                                    calib_mode = controller.calib_mode
                                    if calib_mode in (3, 4):
                                        self.elf_thread.put_pnp_calib_sample(
                                            yaw_deg=info["yaw"], pitch_deg=info["pitch"]
                                        )
                                    else:
                                        arm_yaw = controller.arm_target_yaw
                                        arm_pitch = controller.arm_target_pitch
                                        yaw_comp = controller.interp_yaw_table(arm_yaw)
                                        pitch_comp = controller.interp_pitch_table(arm_pitch)
                                        self.elf_thread.put_pnp_correction(
                                            yaw_correction=info["yaw"] + yaw_comp,
                                            pitch_correction=info["pitch"] + pitch_comp,
                                            valid=True,
                                        )
                                except Exception as e:
                                    print(f"[ElfControl] PnP feed error: {e}")
                        else:
                            cv2.putText(
                                display, "PnP FAIL", (self.img_w - 120, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2
                            )
                    else:
                        if self.tracker.face_size_life > 0:
                            self.tracker.face_size_life -= 1
                        cv2.putText(
                            display, "FaceLM bad ratio", (rx, ry - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                        )
            else:
                cv2.putText(
                    display, f"Skip face: {skip_reason}", (10, self.img_h - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                )
            face_ms = (time.perf_counter() - t_face0) * 1000.0

            # ---------- Rule Engine ----------
            t_rule0 = time.perf_counter()
            rule_text = None
            rule_state = None
            try:
                rule_state = self.rule_engine_state.infer(
                    self.pipeline, best_det, face_lm_img, self.img_w
                )
                rule_text = "RULE: " + ", ".join(
                    f"{name}={int(rule_state[i])}"
                    for i, name in enumerate(RuleEngineState.STATE_NAMES)
                )
                self._handle_rule_mode_transition(int(rule_state[0]))
            except Exception as e:
                rule_text = f"RULE: err {type(e).__name__}"
                print(f"[RuleEngine] inference failed: {e}")
            rule_ms = (time.perf_counter() - t_rule0) * 1000.0

            if rule_text:
                y_pos = 140
                (tw, th), _ = cv2.getTextSize(
                    rule_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
                )
                cv2.rectangle(
                    display, (8, y_pos - th - 6), (12 + tw, y_pos + 4), (0, 0, 0), -1
                )
                cv2.putText(
                    display, rule_text, (10, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
                )

            # ---------- Hand Landmarks ----------
            t_hand0 = time.perf_counter()
            kps = best_det["kps"]
            hand_rois = [
                estimate_hand_roi(kps[9], kps[7], self.img_w, self.img_h),
                estimate_hand_roi(kps[10], kps[8], self.img_w, self.img_h),
            ]
            hand_colors = [(255, 0, 0), (0, 255, 255)]
            hand_labels = ["Left", "Right"]
            hand_info_lines = []
            for idx, roi in enumerate(hand_rois):
                if not self._is_hand_above_shoulder(kps, idx):
                    continue
                if roi is None:
                    continue
                hand_result = detect_hand_landmarks(self.pipeline, done.frame, roi)
                if hand_result is None:
                    continue
                if hand_result["presence"] < HAND_CROP_MIN_PRESENCE:
                    continue
                draw_hand_landmarks(display, hand_result, color=hand_colors[idx])
                gesture = hand_result.get("gesture_label", "-")
                self._handle_gesture_control(idx, hand_result)
                line = f"{hand_labels[idx]}: p={hand_result['presence']:.2f} {gesture}"
                hand_info_lines.append(line)

            if hand_info_lines:
                x_pos = self.img_w - 10
                y_start = 60
                for i, line in enumerate(hand_info_lines):
                    y_pos = y_start + i * 28
                    (tw, th), _ = cv2.getTextSize(
                        line, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2
                    )
                    cv2.rectangle(
                        display, (x_pos - tw - 6, y_pos - th - 4),
                        (x_pos + 4, y_pos + 6), (0, 0, 0), -1
                    )
                    cv2.putText(
                        display, line, (x_pos - tw, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2
                    )
            hand_ms = (time.perf_counter() - t_hand0) * 1000.0

            # Reset tracker after too many face failures
            if not face_success_this_frame:
                self.face_fail_count += 1
                if self.face_fail_count >= 15:
                    print("[Tracker] Too many face failures, reset tracking")
                    self.tracker.tracked = False
                    self.tracker.history.clear()
                    self.face_fail_count = 0
            else:
                self.face_fail_count = 0

        total_ms = (time.perf_counter() - t0) * 1000.0
        self.stats.add(total_ms, done.body_ms, face_ms, hand_ms, pnp_ms, rule_ms)
        output_frame = (
            display
            if self.annotation_mode is None or self.annotation_mode.is_enabled()
            else done.frame
        )
        self.out_q.put((done.frame_idx, output_frame))

    def _infer_rule_for_body_only(self, body_dets):
        if not body_dets:
            return None

        try:
            return self.rule_engine_state.infer(
                self.pipeline, body_dets[0], None, self.img_w
            )
        except Exception as e:
            print(f"[RuleEngine] scenario inference failed: {e}")
            return None
    def _handle_rule_mode_transition(self, curr_mode: int) -> None:
        prev_mode = self._prev_rule_mode
        self._prev_rule_mode = curr_mode
        if not ENABLE_RULE_MODE_CONTROL or prev_mode is None or self.elf_thread is None:
            return
        if getattr(self.elf_thread, "is_remote_locked", lambda: False)():
            return

        try:
            target_mode = None
            if prev_mode == 0 and curr_mode == 1:
                target_mode = "intro"      # right hand raised
            elif prev_mode == 0 and curr_mode == 2:
                target_mode = "interview"  # left hand raised
            if target_mode is None:
                return
            if self.elf_thread.set_mode(target_mode, source="vision"):
                print(f"[RuleCmd] mode0 -> mode{curr_mode}: {target_mode}")
        except Exception as e:
            print(f"[RuleCmd] transition {prev_mode}->{curr_mode} failed: {e}")

    def _handle_scenario_ok_gesture(self, body_dets, frame, display) -> None:
        if self.elf_thread is None or not body_dets:
            return
        det = body_dets[0]
        kps = det["kps"]
        hand_rois = [
            estimate_hand_roi(kps[9], kps[7], self.img_w, self.img_h),
            estimate_hand_roi(kps[10], kps[8], self.img_w, self.img_h),
        ]
        hand_colors = [(255, 0, 0), (0, 255, 255)]
        for idx, roi in enumerate(hand_rois):
            if not self._is_hand_above_shoulder(kps, idx) or roi is None:
                continue
            hand_result = detect_hand_landmarks(self.pipeline, frame, roi)
            if hand_result is None or hand_result["presence"] < HAND_CROP_MIN_PRESENCE:
                continue
            draw_hand_landmarks(display, hand_result, color=hand_colors[idx])
            gesture = hand_result.get("gesture_label", "-")
            if gesture == "OK":
                self._handle_gesture_control(idx, hand_result, ok_only=True)
                cv2.putText(
                    display, f"Scenario hand: {gesture}", (10, 62),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
                )

    @staticmethod
    def _is_hand_above_shoulder(kps, hand_idx: int) -> bool:
        wrist_idx, shoulder_idx = ((9, 5) if hand_idx == 0 else (10, 6))
        wrist = kps[wrist_idx]
        shoulder = kps[shoulder_idx]
        if (wrist.get("visibility", 0.0) <= KPT_CONF_THRESHOLD or
                shoulder.get("visibility", 0.0) <= KPT_CONF_THRESHOLD):
            return False
        return float(wrist["y"]) < float(shoulder["y"])

    def _handle_gesture_control(self, hand_idx: int, hand_result: dict, ok_only: bool = False) -> None:
        if self.elf_thread is None:
            return

        label = hand_result.get("gesture_label", "") or ""
        action = None
        if ok_only:
            if label == "OK":
                action = ("mode", "face", "OK -> 正面")
        elif label == "Pointer-UP":
            if hand_idx == 0:  # COCO left wrist / anatomical left hand
                action = ("profile", 0, "左手上指 -> 拉近")
            elif hand_idx == 1:  # COCO right wrist / anatomical right hand
                action = ("profile", 1, "右手上指 -> 拉远")
        elif label == "OK":
            action = ("mode", "face", "OK -> 正面")
        elif label == "Open":
            action = ("mode", "intro", "Open -> 介绍")
        elif label == "Close":
            action = ("mode", "interview", "Close -> 采访")

        if action is None:
            self._gesture_candidate = None
            self._gesture_count = 0
            return

        key = (hand_idx, action[0], action[1])
        if key == self._gesture_candidate:
            self._gesture_count += 1
        else:
            self._gesture_candidate = key
            self._gesture_count = 1

        now_s = time.perf_counter()
        if self._gesture_count < 5:
            return
        if now_s - self._last_gesture_trigger_s < 1.5:
            return

        kind, value, desc = action
        try:
            if label == "OK" and self.elf_thread.confirm_voice_power_off():
                self._last_gesture_trigger_s = now_s
                self._gesture_count = 0
                print("[GestureCmd] OK -> power-off confirmed")
                return
            if kind == "profile":
                self.elf_thread.set_arm_profile(value)
                reporter = getattr(self.elf_thread, "cloud_reporter", None)
                if reporter is not None:
                    reporter.report_zoom(_profile_to_zoom(value))
            elif kind == "mode":
                reporter = getattr(self.elf_thread, "cloud_reporter", None)
                _set_mode_and_report_view(self.elf_thread, reporter, value, _mode_to_view_mode(value))
            self._last_gesture_trigger_s = now_s
            self._gesture_count = 0
            print(f"[GestureCmd] {desc}")
        except Exception as e:
            print(f"[GestureCmd] failed {desc}: {e}")

    def _infer_face_lm(self, frame, roi):
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
        with self.pipeline.npu_lock:
            out = self.pipeline.face_lm_compiled({self.pipeline.face_lm_input_name: inp})
        lm = out[self.pipeline.face_lm_output_name][0].reshape(-1, 3)
        return lm


# ---------------------------------------------------------------------------
# Output loop (same logic as original)
# ---------------------------------------------------------------------------


def output_loop(out_q, stop_ev, stats, headless, frames_limit, output_dir,
                run_result=None, display_q=None, ll_streamer=None, save_frames=False,
                annotation_mode=None):
    expected_idx = 0
    buffer = {}
    processed = 0
    prev_show_time = time.perf_counter()
    os.makedirs(output_dir, exist_ok=True)

    while not stop_ev.is_set() or buffer or not out_q.empty():
        try:
            frame_idx, display = out_q.get(timeout=0.05)
        except queue.Empty:
            continue

        if frame_idx == expected_idx:
            prev_show_time, processed, expected_idx = _emit_frame(
                display, frame_idx, prev_show_time, processed, expected_idx,
                headless, output_dir, display_q, ll_streamer, save_frames,
                annotation_mode
            )
            while expected_idx in buffer:
                display = buffer.pop(expected_idx)
                prev_show_time, processed, expected_idx = _emit_frame(
                    display, expected_idx, prev_show_time, processed, expected_idx,
                    headless, output_dir, display_q, ll_streamer, save_frames,
                    annotation_mode
                )
        else:
            buffer[frame_idx] = display

        if headless and frames_limit and processed >= frames_limit:
            stop_ev.set()
            break

    print(f"[Output] Total displayed frames: {processed}")
    if run_result is not None:
        run_result["processed"] = processed


def _emit_frame(display, frame_idx, prev_show_time, processed, expected_idx,
                headless, output_dir, display_q, ll_streamer, save_frames,
                annotation_mode):
    if annotation_mode is None or annotation_mode.is_enabled():
        prev_show_time = _overlay_fps(display, prev_show_time)
    else:
        prev_show_time = time.perf_counter()
    if ll_streamer is not None:
        ll_streamer.write_frame(display)
    if headless and save_frames:
        if frame_idx % 10 == 0:
            path = os.path.join(output_dir, f"pingpong_{frame_idx:06d}.jpg")
            cv2.imwrite(path, display)
            print(f"[Headless] Saved {path}")
    elif not headless and display_q is not None:
        display_q.put((frame_idx, display))
    return prev_show_time, processed + 1, expected_idx + 1


def _overlay_fps(display, prev_time):
    h, w = display.shape[:2]
    now = time.perf_counter()
    fps = 1.0 / (now - prev_time + 1e-9)
    cv2.putText(
        display, f"FPS: {fps:.1f}", (10, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
    )
    return now


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------




def _normalize_voice_keyword(keyword: str) -> str:
    keyword = (keyword or "").strip()
    if keyword.startswith("@"):
        keyword = keyword[1:]
    if "@" in keyword:
        keyword = keyword.rsplit("@", 1)[-1].strip()
    return keyword




def _mode_to_view_mode(mode: str) -> int:
    mode = (mode or "").lower()
    if mode == "interview":
        return 0
    if mode == "intro":
        return 2
    return 1


def _view_mode_to_mode(view_mode: int) -> str:
    return {0: "interview", 1: "face", 2: "intro"}.get(int(view_mode), "face")


def _report_third_person_before_view_mode(elf_thread, ll_streamer) -> None:
    if elf_thread is None or ll_streamer is None:
        return
    if elf_thread.get_mode() == "first_person":
        ll_streamer.report_track_obj(1)
        print("[CloudReport] first_person -> trackObj=1 before viewMode")


def _set_mode_and_report_view(elf_thread, ll_streamer, mode: str, view_mode: int) -> None:
    _report_third_person_before_view_mode(elf_thread, ll_streamer)
    ok = elf_thread.set_mode(mode)
    if ok and ll_streamer is not None:
        ll_streamer.report_view_mode(int(view_mode))
    return bool(ok)


def _profile_to_zoom(profile_idx: int) -> int:
    return 2 if int(profile_idx) == 0 else 0


def _zoom_to_profile(zoom: int) -> int:
    # l3=65 is retained for later tuning but excluded from normal controls.
    return 0 if int(zoom) == 2 else 1


def _handle_cloud_zoom(elf_thread, ll_streamer, zoom: int, source: str = "cloud") -> None:
    if elf_thread is None:
        return
    profile_idx = _zoom_to_profile(zoom)
    elf_thread.set_arm_profile(profile_idx)
    if ll_streamer is not None:
        ll_streamer.report_zoom(int(zoom))
    print(f"[CloudCmd] set_zoom={zoom} -> profile={profile_idx}")


def _handle_cloud_view_mode(elf_thread, ll_streamer, view_mode: int, source: str = "cloud") -> None:
    if elf_thread is None:
        return
    mode = _view_mode_to_mode(view_mode)
    _set_mode_and_report_view(elf_thread, ll_streamer, mode, int(view_mode))
    print(f"[CloudCmd] set_view_mode={view_mode} -> mode={mode}")


def _handle_cloud_track_obj(elf_thread, ll_streamer, track_obj: int, source: str = "cloud") -> None:
    if elf_thread is None:
        return
    if int(track_obj) == 0:
        elf_thread.set_mode("first_person")
        mode = "first_person"
    else:
        if elf_thread.get_mode() == "first_person":
            elf_thread.set_mode("face")
        mode = elf_thread.get_mode()
    if ll_streamer is not None:
        ll_streamer.report_track_obj(int(track_obj))
    print(f"[CloudCmd] track_obj={track_obj} -> {mode}")


def _handle_cloud_annotation_mode(annotation_mode, enabled: bool,
                                  source: str = "cloud") -> None:
    if annotation_mode is None:
        return
    changed = annotation_mode.set_enabled(bool(enabled))
    label = "annotation" if enabled else "raw"
    print(f"[CloudCmd] set_annotation_mode={bool(enabled)} -> {label}, changed={changed}")


def _handle_cloud_target(elf_thread, ll_streamer, target: dict, source: str = "cloud") -> None:
    if elf_thread is None:
        return
    x = int(target.get("x", 0))
    y = int(target.get("y", 0))
    z = int(target.get("z", 0))
    if ll_streamer is not None:
        ll_streamer.report_target_pose(x, y, z)
    ok = elf_thread.set_first_person_discrete_target(x, y, z)
    state = "sent" if ok else "stored"
    print(f"[CloudCmd] set_target first_person x={x} y={y} z={z} {state}={ok}")

def _handle_voice_command(voice_thread, elf_thread, ll_streamer=None,
                          annotation_mode=None) -> None:
    if voice_thread is None:
        return
    latest = voice_thread.get_latest(consume=True)
    if latest is None:
        return
    keyword, _ts = latest
    keyword = _normalize_voice_keyword(keyword)
    try:
        if keyword == "原画":
            changed = annotation_mode.set_enabled(False) if annotation_mode is not None else False
            if changed and ll_streamer is not None:
                ll_streamer.report_annotation_mode(False)
            print(f"[VoiceCmd] 原画 -> raw mode changed={changed}")
        elif keyword == "标注":
            changed = annotation_mode.set_enabled(True) if annotation_mode is not None else False
            if changed and ll_streamer is not None:
                ll_streamer.report_annotation_mode(True)
            print(f"[VoiceCmd] 标注 -> annotation mode changed={changed}")
        elif keyword == "拉远":
            elf_thread.set_arm_profile(1)  # far_l3_55, l3=55
            if ll_streamer is not None:
                ll_streamer.report_zoom(0)
            print("[VoiceCmd] 拉远 -> profile far_l3_55, zoom=0")
        elif keyword == "拉近":
            elf_thread.set_arm_profile(0)  # mid_l3_40, l3=40
            if ll_streamer is not None:
                ll_streamer.report_zoom(2)
            print("[VoiceCmd] 拉近 -> profile mid_l3_40, zoom=2")
        elif keyword == "介绍":
            _set_mode_and_report_view(elf_thread, ll_streamer, "intro", 2)
            print("[VoiceCmd] 介绍 -> mode intro, viewMode=2")
        elif keyword == "采访":
            _set_mode_and_report_view(elf_thread, ll_streamer, "interview", 0)
            print("[VoiceCmd] 采访 -> mode interview, viewMode=0")
        elif keyword in ("正面", "正脸"):
            _set_mode_and_report_view(elf_thread, ll_streamer, "face", 1)
            print(f"[VoiceCmd] {keyword} -> mode face, viewMode=1")
        elif keyword == "并肩":
            if elf_thread.get_mode() != "first_person":
                elf_thread.set_mode("first_person")
                if ll_streamer is not None:
                    ll_streamer.report_track_obj(0)
                print("[VoiceCmd] 并肩 -> mode first_person, trackObj=0")
            else:
                print("[VoiceCmd] 并肩 ignored; already first_person")
        elif keyword == "开机":
            ok = elf_thread.power_on_arm()
            print(f"[VoiceCmd] 开机 -> power_on ok={ok}")
        elif keyword == "关机":
            if elf_thread.get_mode() != "face":
                print(f"[VoiceCmd] 关机 ignored outside face: {elf_thread.get_mode()}")
            else:
                ok = elf_thread.request_voice_power_off(timeout_s=3.0)
                print(f"[VoiceCmd] 关机 -> waiting for OK gesture, started={ok}")
        elif keyword == "录制":
            ok = ll_streamer.request_record("start", {"source": "voice", "keyword": keyword}) if ll_streamer is not None else False
            print(f"[VoiceCmd] 录制 -> record_start ok={ok}")
        elif keyword == "停止":
            ok = ll_streamer.request_record("stop", {"source": "voice", "keyword": keyword}) if ll_streamer is not None else False
            print(f"[VoiceCmd] 停止 -> record_stop ok={ok}")
        else:
            print(f"[VoiceCmd] ignored keyword: {keyword}")
    except Exception as e:
        print(f"[VoiceCmd] command failed for {keyword}: {e}")

def main():
    parser = argparse.ArgumentParser(description="Ping-pong pipeline with async body + PnP")
    parser.add_argument("--show-window", action="store_true",
                        help="Show local OpenCV window (default: headless streaming mode)")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--output-dir", type=str, default="/tmp/pingpong_async_frames")
    parser.add_argument("--save-frames", action="store_true",
                        help="Save JPEG frames in headless mode (default: disabled)")
    parser.add_argument("--device-id", type=str, default="device-002",
                        help="Cloud device ID for WebSocket registration")
    parser.add_argument("--device-key", type=str, default=os.environ.get("DEVICE_KEY"),
                        help="Cloud device key for WebSocket/record-control authentication")
    parser.add_argument("--ws-url", type=str, default=None,
                        help="WebSocket URL (default: derived from stream URL)")
    parser.add_argument("--voice", action="store_true", default=True,
                        help="Enable voice KWS (default: True)")
    parser.add_argument("--no-voice", action="store_true",
                        help="Disable voice KWS")
    parser.add_argument("--voice-provider", type=str, default="cpu", choices=["cpu", "openvino"])
    parser.add_argument("--voice-no-vad", action="store_true")
    parser.add_argument("--voice-vad-threshold", type=float, default=None)
    parser.add_argument("--voice-vad-hangover-ms", type=float, default=None)
    parser.add_argument("--raw-queue-size", type=int, default=8)
    parser.add_argument("--body-pool-size", type=int, default=3, help="Async body infer request pool size")
    parser.add_argument("--preproc-threads", type=int, default=2)
    parser.add_argument("--elf-control", action="store_true", default=True,
                        help="Enable ELF hardware control chain (default: True)")
    parser.add_argument("--no-elf-control", action="store_true",
                        help="Disable ELF hardware control chain")
    parser.add_argument("--elf-stub", action="store_true", help="Use stub IMU/UART sources (no hardware)")
    parser.add_argument("--nrf-backend", type=str, default="stm32_uart",
                        choices=["dk2500", "stm32_uart"],
                        help="NRF24/head-IMU backend (default: stm32_uart)")
    parser.add_argument("--stm32-uart-port", type=str, default=None,
                        help="UART port for STM32 DataHub bridge (used with --nrf-backend stm32_uart)")
    parser.add_argument("--stm32-uart-baud", type=int, default=460_800,
                        help="Baudrate for STM32 DataHub bridge (default: 460800)")
    parser.add_argument("--wifi", action="store_true", help="Initialize WiFi on startup")
    parser.add_argument("--wifi-ssid", type=str, default="iQOO 12",
                        help="WiFi SSID (default: iQOO 12)")
    parser.add_argument("--wifi-passwd", type=str, default="070103xsc",
                        help="WiFi password")
    parser.add_argument("--wifi-iface", type=str, default="wlan0",
                        help="WiFi interface (default: wlan0)")
    parser.add_argument("--ble", action="store_true", help="Enable HC-08 BLE remote listener")
    parser.add_argument("--ble-mac", type=str, default="F8:2E:0C:E3:99:C8",
                        help="HC-08 BLE MAC address")
    parser.add_argument("--no-stream", action="store_true",
                        help="Disable RTMP/WebSocket streaming (default: streaming enabled)")
    parser.add_argument("--rtsp", action="store_true",
                        help="Compatibility alias for --stream-mode local")
    parser.add_argument("--stream-mode", choices=("auto", "cloud", "local"), default="auto",
                        help="Stream state machine mode: auto tries cloud then local; cloud/local force one target")
    parser.add_argument("--stream-url", type=str, default=None,
                        help="RTMP/RTSP URL to stream to. Defaults to cloud RTMP; failed RTMP handshake falls back to local RTSP.")
    parser.add_argument("--stream-fps", type=int, default=15,
                        help="Stream frame rate (default: 15)")
    parser.add_argument("--stream-width", type=int, default=None,
                        help="Stream width (default: camera width)")
    parser.add_argument("--stream-height", type=int, default=None,
                        help="Stream height (default: camera height)")
    parser.add_argument("--uart-port", type=str, default="/dev/ttyS9",
                        help="Legacy direct-H7 UART port (unused with stm32_uart bridge)")
    parser.add_argument("--nrf-spidev", type=str, default="/dev/spidev4.0",
                        help="SPI device for NRF24 (default: /dev/spidev4.0)")
    parser.add_argument("--imu2-bus", type=str, default="/dev/i2c-4",
                        help="I2C bus for IMU2 (default: /dev/i2c-4)")
    parser.add_argument("--ctrl-port", type=int, default=8080, help="HTTP control port (0 to disable)")
    args = parser.parse_args()
    if args.no_voice:
        args.voice = False

    print("\n" + "=" * 60)
    print("PingPong async body + PnP pipeline starting...")
    print("=" * 60)

    pipeline = SimplePipeline()
    control_state = "enabled" if ENABLE_RULE_MODE_CONTROL else "disabled"
    print(f"[Rule] Rule Engine model loaded; control integration={control_state}")
    # Legacy naming compatibility: detect_hand_landmarks reads hand outputs
    # from hand_lm_input_names in this pipeline variant.
    pipeline.hand_lm_output_names = pipeline.hand_lm_input_names

    cam_idx = find_camera_index()
    cap = cv2.VideoCapture(cam_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[Camera] Resolution {actual_w}x{actual_h}")

    elf_enabled = args.elf_control and not args.no_elf_control
    elf_thread = None
    if elf_enabled:
        elf_thread = make_elf_control_thread(
            stub=args.elf_stub,
            ctrl_port=args.ctrl_port,
            nrf_backend=args.nrf_backend,
            stm32_uart_port=args.stm32_uart_port,
            stm32_uart_baud=args.stm32_uart_baud,
        )
        mode = "stub" if args.elf_stub else args.nrf_backend
        print(f"[ElfControl] Enabled ({mode} mode), HTTP port={args.ctrl_port}")

    raw_q = queue.Queue(maxsize=args.raw_queue_size)
    body_in_q = queue.Queue(maxsize=args.body_pool_size * 2)
    body_done_q = queue.Queue()
    out_q = queue.Queue()
    stop_ev = threading.Event()
    stats = FrameStats()
    annotation_mode = AnnotationMode()
    print("[DisplayMode] annotation (default)")

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

    # Pipeline threads
    cap_thread = threading.Thread(target=capture_loop, args=(cap, raw_q, stop_ev), daemon=True)
    preproc_threads = [
        PreprocessorThread(raw_q, body_in_q, stop_ev) for _ in range(args.preproc_threads)
    ]
    body_gpu_thread = BodyGpuThread(
        pipeline.body_compiled,
        pipeline.body_input_name,
        body_in_q,
        body_done_q,
        stop_ev,
        pool_size=args.body_pool_size,
    )
    post_thread = PostprocessorThread(
        pipeline, body_done_q, out_q, stop_ev, stats, actual_w, actual_h,
        elf_thread, annotation_mode
    )
    run_result = {}
    display_q = queue.Queue()

    headless = not args.show_window

    # Default streaming tries cloud RTMP first. LowLatencyStreamer performs a
    # short RTMP publish handshake and falls back to local RTSP on failure.
    stream_mode = "local" if args.rtsp else args.stream_mode
    stream_url = args.stream_url
    if not args.no_stream and stream_url is None:
        if stream_mode == "local":
            stream_url = get_default_rtsp_url()
        else:
            stream_url = get_default_rtmp_url(args.device_id)

    # WebSocket URL: only used for RTMP cloud streaming.
    # For RTSP (local server) no WebSocket reporter is started.
    ws_url = args.ws_url
    if ws_url is None and stream_url and stream_url.startswith("rtmp://"):
        if stream_url == get_default_rtmp_url(args.device_id):
            ws_url = get_default_ws_url(args.device_id)
        else:
            parsed = urlparse(stream_url)
            host = parsed.hostname or "127.0.0.1"
            ws_url = f"ws://{host}/ws?deviceId={args.device_id}"

    ll_streamer = None
    if stream_url:
        ll_streamer = LowLatencyStreamer(
            stream_url=stream_url,
            ws_url=ws_url,
            device_id=args.device_id,
            device_key=args.device_key,
            on_target=lambda target, source="cloud": _handle_cloud_target(elf_thread, ll_streamer, target, source),
            on_zoom=lambda zoom, source="cloud": _handle_cloud_zoom(elf_thread, ll_streamer, zoom, source),
            on_view_mode=lambda view_mode, source="cloud": _handle_cloud_view_mode(elf_thread, ll_streamer, view_mode, source),
            on_track_obj=lambda track_obj, source="cloud": _handle_cloud_track_obj(elf_thread, ll_streamer, track_obj, source),
            on_annotation_mode=lambda enabled, source="cloud": _handle_cloud_annotation_mode(annotation_mode, enabled, source),
            stream_mode=stream_mode,
            audio_queue=(
                voice_thread.stream_audio_queue if voice_thread is not None else None
            ),
        )
        if elf_thread is not None:
            elf_thread.cloud_reporter = ll_streamer
        ll_streamer.probe_and_start(actual_w, actual_h, args.stream_fps)

    out_thread = threading.Thread(
        target=output_loop,
        args=(out_q, stop_ev, stats, headless, args.frames, args.output_dir,
              run_result, display_q, ll_streamer, args.save_frames, annotation_mode),
        daemon=True,
    )

    # Optional system-level initialization (matches Base main.cpp boot steps).
    wifi_thread = None
    ble_remote = None
    if args.wifi:
        wifi_thread = start_wifi_thread(
            ssid=args.wifi_ssid,
            passwd=args.wifi_passwd,
            iface=args.wifi_iface,
        )
    if args.ble:
        init_bluetooth_hci()

    cap_thread.start()
    for t in preproc_threads:
        t.start()
    body_gpu_thread.start()
    post_thread.start()
    out_thread.start()
    if elf_thread is not None:
        elf_thread.start()
        if args.ble:
            ble_remote = start_ble_remote(
                mac=args.ble_mac,
                ctrl_url=f"http://127.0.0.1:{args.ctrl_port}",
            )

    global _g_stop_ev
    _g_stop_ev = stop_ev
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    run_start = time.perf_counter()
    print("[Main] Press Ctrl-C to stop, or 'q' in window.")
    try:
        if headless:
            # Keep running until Ctrl+C or the configured --frames limit is reached.
            while not stop_ev.is_set():
                _handle_voice_command(voice_thread, elf_thread, ll_streamer, annotation_mode)
                time.sleep(0.1)
        else:
            # Run OpenCV highgui on the main thread to avoid Qt cross-thread warnings.
            while not stop_ev.is_set() or out_thread.is_alive() or not display_q.empty():
                if not stop_ev.is_set():
                    _handle_voice_command(voice_thread, elf_thread, ll_streamer, annotation_mode)
                try:
                    _frame_idx, display = display_q.get(timeout=0.05)
                except queue.Empty:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        stop_ev.set()
                    continue
                cv2.imshow("PingPong Async PnP", display)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    stop_ev.set()
    except KeyboardInterrupt:
        print("\n[Main] Stopping...")
    finally:
        stop_ev.set()
        # Drain queues before joining so worker threads unblock quickly.
        for q in (raw_q, body_in_q, body_done_q, out_q, display_q):
            while not q.empty():
                try:
                    q.get_nowait()
                except queue.Empty:
                    break
        if elf_thread is not None:
            elf_thread.flush_queues()
            elf_thread.stop()
        out_thread.join(timeout=2.0)
        body_gpu_thread.join(timeout=3.0)
        post_thread.join(timeout=2.0)
        if voice_thread is not None:
            voice_thread.stop()
        if ble_remote is not None:
            ble_remote.stop()
        if ll_streamer is not None:
            ll_streamer.stop()
        cap.release()
        cv2.destroyAllWindows()
        lingering = [t.name for t in threading.enumerate() if t is not threading.current_thread() and not t.daemon]
        if lingering:
            print(f"[SHUTDOWN-WAIT] non-daemon threads still alive: {lingering}")
        print("[Main] Demo stopped.")


if __name__ == "__main__":
    main()
