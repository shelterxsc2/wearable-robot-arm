# -*- coding: utf-8 -*-
"""Benchmark: body on GPU vs face+hand on NPU, alone and concurrent.

This answers the question: "Does running face+hand on NPU take longer than
running the large body model on GPU, and do they interfere when run together?"
"""
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import openvino as ov

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

BODY_MODEL_PATH = "/home/time/work/trial0/models/yolov8s-pose_openvino_model/yolov8s-pose.xml"
FACE_MODEL_PATH = "/home/time/work/mymodel/face_landmark_468.onnx"
HAND_MODEL_PATH = "/home/time/work/mymodel/openvino_pipeline/models/onnx/hand_landmarks_detector.onnx"

WARMUP = 10
ITERS = 100
DURATION = 10.0  # seconds for throughput test


def make_input(compiled, fill_value=0.0):
    shape = [list(compiled.inputs[0].get_shape())]
    if -1 in shape[0] or any(isinstance(d, str) for d in shape[0]):
        # Fallback for dynamic shapes (none expected here)
        shape = [[1, 3, 224, 224]]
    else:
        shape = [shape[0]]
    return np.full(shape[0], fill_value, dtype=np.float32)


def infer_many(compiled, inp, n):
    for _ in range(n):
        compiled(inp)


def measure_latency(compiled, inp, n, label):
    infer_many(compiled, inp, WARMUP)
    t0 = time.perf_counter()
    infer_many(compiled, inp, n)
    t1 = time.perf_counter()
    avg_ms = (t1 - t0) / n * 1000.0
    print(f"  {label}: avg={avg_ms:.2f}ms over {n} iters")
    return avg_ms


def worker(compiled, inp, duration, results, key):
    infer_many(compiled, inp, WARMUP)
    count = 0
    latencies = []
    t_end = time.perf_counter() + duration
    while time.perf_counter() < t_end:
        t0 = time.perf_counter()
        compiled(inp)
        t1 = time.perf_counter()
        latencies.append((t1 - t0) * 1000.0)
        count += 1
    results[key] = {"count": count, "latencies": latencies}


def concurrent_pair(body_compiled, body_inp, face_compiled, face_inp, hand_compiled, hand_inp, n):
    """Run one body (GPU) and one face+hand (NPU) pair concurrently, n times."""
    pair_times = []
    body_times = []
    face_times = []
    hand_times = []

    def body_job():
        t0 = time.perf_counter()
        body_compiled(body_inp)
        body_times.append((time.perf_counter() - t0) * 1000.0)

    def face_hand_job():
        t0 = time.perf_counter()
        face_compiled(face_inp)
        face_t = (time.perf_counter() - t0) * 1000.0
        face_times.append(face_t)
        t0 = time.perf_counter()
        hand_compiled(hand_inp)
        hand_t = (time.perf_counter() - t0) * 1000.0
        hand_times.append(hand_t)

    for _ in range(n):
        t_pair0 = time.perf_counter()
        t_body = threading.Thread(target=body_job)
        t_face_hand = threading.Thread(target=face_hand_job)
        t_body.start()
        t_face_hand.start()
        t_body.join()
        t_face_hand.join()
        pair_times.append((time.perf_counter() - t_pair0) * 1000.0)

    print(f"  Concurrent pair: avg={np.mean(pair_times):.2f}ms  max={np.max(pair_times):.2f}ms")
    print(f"    body alone in pair: avg={np.mean(body_times):.2f}ms")
    print(f"    face alone in pair: avg={np.mean(face_times):.2f}ms")
    print(f"    hand alone in pair: avg={np.mean(hand_times):.2f}ms")
    print(f"    face+hand in pair:  avg={np.mean(face_times)+np.mean(hand_times):.2f}ms")


def main():
    print("=" * 70)
    print("GPU/NPU concurrent benchmark")
    print("=" * 70)
    print(f"Body : {BODY_MODEL_PATH}")
    print(f"Face : {FACE_MODEL_PATH}")
    print(f"Hand : {HAND_MODEL_PATH}")
    print("=" * 70)

    core = ov.Core()
    print(f"Available devices: {core.available_devices}")

    print("\n[1/4] Loading models...")
    body_model = core.read_model(BODY_MODEL_PATH)
    face_model = core.read_model(FACE_MODEL_PATH)
    hand_model = core.read_model(HAND_MODEL_PATH)

    body_compiled = core.compile_model(body_model, "GPU")
    face_compiled = core.compile_model(face_model, "NPU")
    hand_compiled = core.compile_model(hand_model, "NPU")
    print("Models loaded.")

    body_inp = make_input(body_compiled)
    face_inp = make_input(face_compiled)
    hand_inp = make_input(hand_compiled)

    print("\n[2/4] Isolated latency")
    body_lat = measure_latency(body_compiled, body_inp, ITERS, "body (GPU)")
    face_lat = measure_latency(face_compiled, face_inp, ITERS, "face (NPU)")
    hand_lat = measure_latency(hand_compiled, hand_inp, ITERS, "hand (NPU)")
    print(f"  face+hand sequential (NPU): avg={face_lat + hand_lat:.2f}ms")

    print("\n[3/4] Concurrent pair latency (1 body GPU + 1 face/hand NPU per pair)")
    concurrent_pair(body_compiled, body_inp, face_compiled, face_inp, hand_compiled, hand_inp, ITERS)

    print("\n[4/4] Sustained concurrent throughput (10s)")
    results = {}
    t_body = threading.Thread(target=worker, args=(body_compiled, body_inp, DURATION, results, "body_gpu"))
    def face_hand_worker(compiled_f, inp_f, compiled_h, inp_h, duration, results, key):
        infer_many(compiled_f, inp_f, WARMUP)
        infer_many(compiled_h, inp_h, WARMUP)
        count = 0
        latencies = []
        t_end = time.perf_counter() + duration
        while time.perf_counter() < t_end:
            t0 = time.perf_counter()
            compiled_f(inp_f)
            compiled_h(inp_h)
            t1 = time.perf_counter()
            latencies.append((t1 - t0) * 1000.0)
            count += 1
        results[key] = {"count": count, "latencies": latencies}

    t_face_hand = threading.Thread(
        target=face_hand_worker,
        args=(face_compiled, face_inp, hand_compiled, hand_inp, DURATION, results, "face_hand_npu"),
    )

    t0 = time.perf_counter()
    t_body.start()
    t_face_hand.start()
    t_body.join()
    t_face_hand.join()
    elapsed = time.perf_counter() - t0

    body_res = results["body_gpu"]
    fh_res = results["face_hand_npu"]
    print(f"  Duration: {elapsed:.2f}s")
    print(f"  body (GPU)        : {body_res['count']} iters  "
          f"avg={np.mean(body_res['latencies']):.2f}ms  "
          f"throughput={body_res['count'] / elapsed:.1f} ips")
    print(f"  face+hand (NPU)   : {fh_res['count']} pairs  "
          f"avg={np.mean(fh_res['latencies']):.2f}ms/pair  "
          f"throughput={fh_res['count'] / elapsed:.1f} pairs/s")

    print("\n" + "=" * 70)
    print("Interpretation:")
    print(f"  body GPU alone      = {body_lat:.2f}ms")
    print(f"  face+hand NPU alone = {face_lat + hand_lat:.2f}ms")
    if face_lat + hand_lat > body_lat:
        print("  -> face+hand on NPU is SLOWER than body on GPU.")
    else:
        print("  -> face+hand on NPU is FASTER than body on GPU.")
    print("=" * 70)


if __name__ == "__main__":
    main()
