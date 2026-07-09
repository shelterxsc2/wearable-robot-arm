# -*- coding: utf-8 -*-
"""Benchmark wrapper for camera_demo_pingpong_async_pnp.py.

Measures per-module latency, fps, memory (RSS/VMS) and system memory
bandwidth (via Intel pcm) over a configurable number of frames.
"""
import argparse
import csv
import json
import math
import os
import queue
import statistics
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import psutil

import cv2
import numpy as np
import openvino as ov

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

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
    RULE_MODEL_PATH,
    FaceTracker,
    PoseEstimator,
    RuleEngineState,
    detect_hand_landmarks,
    draw_hand_landmarks,
    estimate_face_roi,
    estimate_hand_roi,
)
from elf_control_chain import make_elf_control_thread
from voice import VoiceKwsThread


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


class BenchStats:
    """Collect latency, fps, memory and voice timing samples."""

    def __init__(self):
        self.lock = threading.Lock()
        self.records = []
        self.mem_samples = []      # (rss_mb, vms_mb)
        self.fps_samples = []      # instantaneous fps
        self.voice_decode = []     # ms per decode() call
        self.voice_vad = []        # ms per VAD accept_waveform() call
        self.frame_count = 0

    def add_latency(self, total_ms, body_ms, face_ms, hand_ms, pnp_ms, rule_ms):
        with self.lock:
            self.records.append(
                {
                    "total": total_ms,
                    "body": body_ms,
                    "face": face_ms,
                    "hand": hand_ms,
                    "pnp": pnp_ms,
                    "rule": rule_ms,
                }
            )
            self.frame_count += 1

    def add_memory(self, rss_mb, vms_mb):
        with self.lock:
            self.mem_samples.append((rss_mb, vms_mb))

    def add_fps(self, fps):
        with self.lock:
            self.fps_samples.append(fps)

    def add_voice_decode(self, ms):
        with self.lock:
            self.voice_decode.append(ms)

    def add_voice_vad(self, ms):
        with self.lock:
            self.voice_vad.append(ms)

    @staticmethod
    def _percentile(arr, p):
        if not arr:
            return 0.0
        s = sorted(arr)
        n = len(s)
        k = (n - 1) * p / 100.0
        f = int(math.floor(k))
        c = int(math.ceil(k))
        if f == c:
            return float(s[f])
        return s[f] * (c - k) + s[c] * (k - f)

    def _module_report(self, key):
        with self.lock:
            vals = [r[key] for r in self.records]
        if not vals:
            return (0.0, 0.0, 0.0, 0.0)
        return (
            statistics.mean(vals),
            self._percentile(vals, 95),
            self._percentile(vals, 99),
            max(vals),
        )

    def _voice_report(self, arr):
        if not arr:
            return (0.0, 0.0, 0.0, 0.0)
        return (
            statistics.mean(arr),
            self._percentile(arr, 95),
            self._percentile(arr, 99),
            max(arr),
        )

    def _mem_report(self):
        with self.lock:
            rss = [m[0] for m in self.mem_samples]
            vms = [m[1] for m in self.mem_samples]
        if not rss:
            return (0.0, 0.0, 0.0, 0.0)
        return (
            statistics.mean(rss),
            max(rss),
            statistics.mean(vms),
            max(vms),
        )

    def print_report(self, wall_elapsed, processed, pcm_read=None, pcm_write=None,
                     pcm_read_max=None, pcm_write_max=None):
        rows = [
            ("body_infer", self._module_report("body")),
            ("face_infer", self._module_report("face")),
            ("hand_infer", self._module_report("hand")),
            ("pnp", self._module_report("pnp")),
            ("rule_engine", self._module_report("rule")),
            ("total_frame", self._module_report("total")),
        ]
        print("\n" + "=" * 72)
        print(f"[Bench] {processed} frames in {wall_elapsed:.2f}s "
              f"({processed / wall_elapsed:.1f} fps wall-clock)")
        print("-" * 72)
        print(f"{'Module':<16} {'avg(ms)':>10} {'p95(ms)':>10} "
              f"{'p99(ms)':>10} {'max(ms)':>10}")
        print("-" * 72)
        for name, (avg, p95, p99, mx) in rows:
            print(f"{name:<16} {avg:>10.2f} {p95:>10.2f} {p99:>10.2f} {mx:>10.2f}")

        vdec = self._voice_report(self.voice_decode)
        vvad = self._voice_report(self.voice_vad)
        if vdec[3] > 0 or vvad[3] > 0:
            print("-" * 72)
            print(f"{'voice_decode':<16} {vdec[0]:>10.2f} {vdec[1]:>10.2f} "
                  f"{vdec[2]:>10.2f} {vdec[3]:>10.2f}")
            print(f"{'voice_vad':<16} {vvad[0]:>10.2f} {vvad[1]:>10.2f} "
                  f"{vvad[2]:>10.2f} {vvad[3]:>10.2f}")

        rss_avg, rss_max, vms_avg, vms_max = self._mem_report()
        print("-" * 72)
        print(f"Memory (MB): rss_avg={rss_avg:.0f} rss_max={rss_max:.0f} "
              f"vms_avg={vms_avg:.0f} vms_max={vms_max:.0f}")

        if pcm_read:
            print(f"Bandwidth (GB/s): read_avg={statistics.mean(pcm_read):.2f} "
                  f"read_max={pcm_read_max:.2f} "
                  f"write_avg={statistics.mean(pcm_write):.2f} "
                  f"write_max={pcm_write_max:.2f}")
        else:
            print("Bandwidth (GB/s): N/A (run without --no-pcm to enable pcm)")
        print("=" * 72)

    def save_json(self, path, wall_elapsed, processed, pcm_read=None, pcm_write=None):
        with self.lock:
            records = self.records.copy()
        report = {
            "timestamp": datetime.now().isoformat(),
            "frames": processed,
            "wall_elapsed_s": wall_elapsed,
            "wall_fps": processed / wall_elapsed if wall_elapsed > 0 else 0.0,
            "modules": {},
        }
        for key in ("body", "face", "hand", "pnp", "rule", "total"):
            vals = [r[key] for r in records]
            report["modules"][key] = {
                "avg_ms": statistics.mean(vals) if vals else 0.0,
                "p95_ms": self._percentile(vals, 95),
                "p99_ms": self._percentile(vals, 99),
                "max_ms": max(vals) if vals else 0.0,
            }
        report["voice_decode"] = {
            "avg_ms": statistics.mean(self.voice_decode) if self.voice_decode else 0.0,
            "max_ms": max(self.voice_decode) if self.voice_decode else 0.0,
        }
        report["voice_vad"] = {
            "avg_ms": statistics.mean(self.voice_vad) if self.voice_vad else 0.0,
            "max_ms": max(self.voice_vad) if self.voice_vad else 0.0,
        }
        rss = [m[0] for m in self.mem_samples]
        vms = [m[1] for m in self.mem_samples]
        report["memory_mb"] = {
            "rss_avg": statistics.mean(rss) if rss else 0.0,
            "rss_max": max(rss) if rss else 0.0,
            "vms_avg": statistics.mean(vms) if vms else 0.0,
            "vms_max": max(vms) if vms else 0.0,
        }
        if pcm_read:
            report["bandwidth_gbps"] = {
                "read_avg": statistics.mean(pcm_read),
                "read_max": max(pcm_read),
                "write_avg": statistics.mean(pcm_write),
                "write_max": max(pcm_write),
            }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"[Bench] JSON report saved to {path}")


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


class PostprocessorThread(threading.Thread):
    def __init__(self, pipeline, body_done_q, out_q, stop_ev, stats, img_w, img_h, elf_thread=None):
        super().__init__(daemon=True)
        self.pipeline = pipeline
        self.body_done_q = body_done_q
        self.out_q = out_q
        self.stop_ev = stop_ev
        self.stats = stats
        self.img_w = img_w
        self.img_h = img_h
        self.elf_thread = elf_thread
        self.tracker = FaceTracker()
        self.pose_est = PoseEstimator()
        self.rule_engine_state = RuleEngineState()
        self.face_fail_count = 0

    def run(self):
        while not self.stop_ev.is_set() or not self.body_done_q.empty():
            try:
                done = self.body_done_q.get(timeout=0.05)
            except queue.Empty:
                continue
            self._process(done)

    def _process(self, done: BodyDone):
        t0 = time.perf_counter()
        body_dets = postprocess_body(
            done.body_out, self.img_w, self.img_h, done.scale, done.x_off, done.y_off
        )
        body_dets = nms_pose(body_dets, iou_thresh=NMS_THRESHOLD)
        body_dets.sort(key=lambda d: -d["score"])

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

        matched_idx = self.tracker.update(body_dets, self.img_w, self.img_h)

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
                            if self.elf_thread is not None:
                                try:
                                    controller = self.elf_thread.controller
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
            current_state = int(rule_state[0]) if rule_state is not None else 0
            skip_hand = set()
            if current_state == 1:
                skip_hand.add(1)  # right
            elif current_state == 2:
                skip_hand.add(0)  # left

            hand_rois = [
                estimate_hand_roi(kps[9], kps[7], self.img_w, self.img_h),
                estimate_hand_roi(kps[10], kps[8], self.img_w, self.img_h),
            ]
            hand_colors = [(255, 0, 0), (0, 255, 255)]
            hand_labels = ["Left", "Right"]
            hand_info_lines = []
            for idx, roi in enumerate(hand_rois):
                if idx in skip_hand:
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
        self.stats.add_latency(total_ms, done.body_ms, face_ms, hand_ms, pnp_ms, rule_ms)
        self.out_q.put((done.frame_idx, display))

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


def output_loop(out_q, stop_ev, stats, headless, frames_limit, output_dir, run_result=None):
    expected_idx = 0
    buffer = {}
    processed = 0
    prev_show_time = time.perf_counter()
    process = psutil.Process()
    os.makedirs(output_dir, exist_ok=True)

    while not stop_ev.is_set() or buffer or not out_q.empty():
        try:
            frame_idx, display = out_q.get(timeout=0.05)
        except queue.Empty:
            if not headless:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    stop_ev.set()
            continue

        if frame_idx == expected_idx:
            now = time.perf_counter()
            fps = 1.0 / (now - prev_show_time + 1e-9)
            stats.add_fps(fps)
            prev_show_time = now

            # sample memory once per displayed frame
            try:
                mem = process.memory_info()
                stats.add_memory(mem.rss / 1024 / 1024, mem.vms / 1024 / 1024)
            except Exception:
                pass

            _show(display, frame_idx, headless, output_dir, stop_ev)
            processed += 1
            expected_idx += 1
            while expected_idx in buffer:
                display = buffer.pop(expected_idx)
                now = time.perf_counter()
                fps = 1.0 / (now - prev_show_time + 1e-9)
                stats.add_fps(fps)
                prev_show_time = now
                try:
                    mem = process.memory_info()
                    stats.add_memory(mem.rss / 1024 / 1024, mem.vms / 1024 / 1024)
                except Exception:
                    pass
                _show(display, expected_idx, headless, output_dir, stop_ev)
                processed += 1
                expected_idx += 1
        else:
            buffer[frame_idx] = display
            if not headless:
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    stop_ev.set()

        if headless and frames_limit and processed >= frames_limit:
            stop_ev.set()
            break

    print(f"[Output] Total displayed frames: {processed}")
    if run_result is not None:
        run_result["processed"] = processed


def _overlay_fps(display, prev_time):
    h, w = display.shape[:2]
    now = time.perf_counter()
    fps = 1.0 / (now - prev_time + 1e-9)
    cv2.putText(
        display, f"FPS: {fps:.1f}", (10, h - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
    )
    return now


def _show(display, frame_idx, headless, output_dir, stop_ev):
    if headless:
        if frame_idx % 10 == 0:
            path = os.path.join(output_dir, f"pingpong_{frame_idx:06d}.jpg")
            cv2.imwrite(path, display)
            print(f"[Headless] Saved {path}")
    else:
        cv2.imshow("PingPong Async PnP", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            stop_ev.set()


# ---------------------------------------------------------------------------
# Profiling voice KWS
# ---------------------------------------------------------------------------


class ProfilingVoiceKwsThread(VoiceKwsThread):
    """VoiceKwsThread that times decode() and VAD accept_waveform()."""

    def __init__(self, *args, stats=None, **kwargs):
        super().__init__(*args, **kwargs)
        self.stats = stats

    def run(self):
        vad_state = False
        silent_blocks = 0
        while not self._stop_event.is_set():
            try:
                block = self.audio_queue.get(timeout=0.05)
            except queue.Empty:
                continue

            self.kws.accept_waveform(block)

            t0 = time.perf_counter()
            kw = self.kws.decode()
            if self.stats is not None:
                self.stats.add_voice_decode((time.perf_counter() - t0) * 1000.0)
            if kw:
                self._set_latest(kw)

            if self.vad is not None:
                prev_state = vad_state
                t1 = time.perf_counter()
                vad_state = self.vad.accept_waveform(block)
                if self.stats is not None:
                    self.stats.add_voice_vad((time.perf_counter() - t1) * 1000.0)
                if not vad_state:
                    silent_blocks += 1
                    if prev_state and silent_blocks >= self.silence_reset_blocks:
                        self.kws.reset()
                        silent_blocks = 0
                else:
                    silent_blocks = 0


# ---------------------------------------------------------------------------
# PCM memory-bandwidth parser
# ---------------------------------------------------------------------------


def parse_pcm_csv(path):
    """Parse Intel pcm -csv output and return system-level READ/WRITE lists (GB/s)."""
    reads = []
    writes = []
    try:
        with open(path, newline="") as f:
            r = csv.reader(f)
            next(r)          # skip group/header line
            header = next(r)
            read_idx = header.index("READ")
            write_idx = header.index("WRITE")
            for row in r:
                if not row or row[0].startswith("Date"):
                    continue
                try:
                    reads.append(float(row[read_idx]))
                    writes.append(float(row[write_idx]))
                except (ValueError, IndexError):
                    continue
    except Exception as e:
        print(f"[PCM] failed to parse {path}: {e}")
    return reads, writes


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Benchmark camera_demo_pingpong_async_pnp.py "
                    "with latency, fps, memory and bandwidth metrics."
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--frames", type=int, default=1000,
                        help="Number of frames to benchmark (default: 1000)")
    parser.add_argument("--output-dir", type=str, default="/tmp/pingpong_async_bench")
    parser.add_argument("--voice", action="store_true", help="Enable voice KWS")
    parser.add_argument("--voice-provider", type=str, default="cpu", choices=["cpu", "openvino"])
    parser.add_argument("--voice-no-vad", action="store_true")
    parser.add_argument("--voice-vad-threshold", type=float, default=None)
    parser.add_argument("--voice-vad-hangover-ms", type=float, default=None)
    parser.add_argument("--raw-queue-size", type=int, default=8)
    parser.add_argument("--body-pool-size", type=int, default=3, help="Async body infer request pool size")
    parser.add_argument("--preproc-threads", type=int, default=2)
    parser.add_argument("--elf-control", action="store_true", help="Enable ELF hardware control chain")
    parser.add_argument("--elf-stub", action="store_true", help="Use stub IMU/UART sources (no hardware)")
    parser.add_argument("--uart-port", type=str, default="/dev/ttyUSB0")
    parser.add_argument("--nrf-spidev", type=str, default="/dev/spidev0.0")
    parser.add_argument("--imu2-bus", type=str, default="/dev/i2c-1")
    parser.add_argument("--ctrl-port", type=int, default=8080, help="HTTP control port (0 to disable)")
    parser.add_argument("--no-pcm", action="store_true",
                        help="Disable Intel pcm memory-bandwidth monitoring")
    parser.add_argument("--pcm-csv", type=str, default="/tmp/pcm_pingpong_bench.csv")
    parser.add_argument("--pcm-interval", type=float, default=0.5)
    parser.add_argument("--sudo-password", type=str, default="kkk",
                        help="Password passed to sudo -S for pcm")
    args = parser.parse_args()

    print("\n" + "=" * 60)
    print("PingPong async body + PnP pipeline starting...")
    print("=" * 60)

    pipeline = SimplePipeline()

    # Load rule engine on GPU to match ELF pipeline
    print("[Rule] Loading Rule Engine model on GPU")
    core = ov.Core()
    pipeline.rule_model = core.read_model(RULE_MODEL_PATH)
    for i in pipeline.rule_model.inputs:
        print(f"      Input:  {i.get_any_name()} {i.get_partial_shape()}")
    for o in pipeline.rule_model.outputs:
        print(f"      Output: {o.get_any_name()} {o.get_partial_shape()}")
    pipeline.rule_compiled = core.compile_model(pipeline.rule_model, "GPU")
    pipeline.rule_input_names = []
    for i in pipeline.rule_compiled.inputs:
        try:
            pipeline.rule_input_names.append(i.get_any_name())
        except Exception:
            pipeline.rule_input_names.append(len(pipeline.rule_input_names))
    try:
        pipeline.rule_output_name = pipeline.rule_compiled.outputs[0].get_any_name()
    except Exception:
        pipeline.rule_output_name = 0
    # ELF hand helper expects output names under this attribute
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

    elf_thread = None
    if args.elf_control:
        elf_thread = make_elf_control_thread(
            stub=args.elf_stub,
            uart_port=args.uart_port,
            nrf_spidev=args.nrf_spidev,
            imu2_bus=args.imu2_bus,
            ctrl_port=args.ctrl_port,
        )
        mode = "stub" if args.elf_stub else "hardware"
        print(f"[ElfControl] Enabled ({mode} mode), HTTP port={args.ctrl_port}")

    raw_q = queue.Queue(maxsize=args.raw_queue_size)
    body_in_q = queue.Queue(maxsize=args.body_pool_size * 2)
    body_done_q = queue.Queue()
    out_q = queue.Queue()
    stop_ev = threading.Event()
    stats = BenchStats()

    # Launch Intel pcm for system memory bandwidth (needs sudo)
    pcm_proc = None
    if not args.no_pcm:
        print(f"[PCM] Starting pcm with sudo (interval={args.pcm_interval}s, csv={args.pcm_csv})")
        try:
            pcm_proc = subprocess.Popen(
                ["sudo", "-S", "pcm", "-r", str(args.pcm_interval),
                 "-csv=" + args.pcm_csv],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                text=True,
            )
            pcm_proc.stdin.write(args.sudo_password + "\n")
            pcm_proc.stdin.flush()
            time.sleep(0.5)
        except Exception as e:
            print(f"[PCM] failed to start pcm: {e}; continuing without bandwidth data")
            pcm_proc = None

    voice_thread = None
    if args.voice:
        voice_thread = ProfilingVoiceKwsThread(
            model_dir=SHERPA_MODEL_DIR,
            keywords_file=SHERPA_KEYWORDS_FILE,
            provider=args.voice_provider,
            device_id=6,
            use_vad=not args.voice_no_vad,
            vad_model=SHERPA_VAD_MODEL,
            vad_threshold=args.voice_vad_threshold,
            vad_hangover_ms=args.voice_vad_hangover_ms,
            stats=stats,
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
        pipeline, body_done_q, out_q, stop_ev, stats, actual_w, actual_h, elf_thread
    )
    run_result = {}
    out_thread = threading.Thread(
        target=output_loop,
        args=(out_q, stop_ev, stats, args.headless, args.frames, args.output_dir, run_result),
        daemon=True,
    )

    cap_thread.start()
    for t in preproc_threads:
        t.start()
    body_gpu_thread.start()
    post_thread.start()
    out_thread.start()
    if elf_thread is not None:
        elf_thread.start()

    run_start = time.perf_counter()
    print("[Main] Press Ctrl-C to stop, or 'q' in window.")
    try:
        if args.headless and args.frames == 0:
            time.sleep(30)
            stop_ev.set()
        elif not args.headless:
            while not stop_ev.is_set():
                time.sleep(0.05)
        else:
            out_thread.join()
    except KeyboardInterrupt:
        print("\n[Main] Stopping...")
    finally:
        stop_ev.set()
        out_thread.join(timeout=2.0)
        body_gpu_thread.join(timeout=3.0)
        post_thread.join(timeout=2.0)
        if voice_thread is not None:
            voice_thread.stop()
        if elf_thread is not None:
            elf_thread.stop()
        cap.release()
        cv2.destroyAllWindows()

        run_elapsed = time.perf_counter() - run_start
        processed = run_result.get("processed", 0)

        # Stop pcm and parse bandwidth data
        pcm_read = None
        pcm_write = None
        if pcm_proc is not None:
            try:
                pcm_proc.terminate()
                pcm_proc.wait(timeout=2.0)
            except Exception:
                try:
                    pcm_proc.kill()
                except Exception:
                    pass
            time.sleep(0.2)
            pcm_read, pcm_write = parse_pcm_csv(args.pcm_csv)

        pcm_read_max = max(pcm_read) if pcm_read else 0.0
        pcm_write_max = max(pcm_write) if pcm_write else 0.0

        stats.print_report(run_elapsed, processed, pcm_read, pcm_write,
                           pcm_read_max, pcm_write_max)

        os.makedirs(args.output_dir, exist_ok=True)
        json_path = os.path.join(args.output_dir,
                                 f"bench_{datetime.now():%Y%m%d_%H%M%S}.json")
        stats.save_json(json_path, run_elapsed, processed, pcm_read, pcm_write)

        if run_elapsed > 0:
            print(f"[Run] Processed {processed} frames in {run_elapsed:.2f}s "
                  f"({processed/run_elapsed:.1f} fps)")
        print("[Main] Demo stopped.")


if __name__ == "__main__":
    main()
