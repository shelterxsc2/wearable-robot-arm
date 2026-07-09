# -*- coding: utf-8 -*-
"""
单独测试 yolo26s-pose.onnx
=======================
尝试不同预处理/后处理方式，找出最佳效果。
"""
import os
import sys
import argparse
import math

import cv2
import numpy as np
import openvino as ov

MODEL_PATH = "/home/time/work/mymodel/yolo26s-pose.onnx"
INPUT_SIZE = 640

# COCO keypoint 连接（画图用）
COCO_SKELETON = [
    (0, 1), (0, 2), (1, 3), (2, 4),        # 脸
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10), # 手臂
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16), # 腿
    (5, 11), (6, 12), (5, 0), (6, 0), (11, 0), (12, 0), # 躯干
]


def load_model(device="CPU"):
    core = ov.Core()
    print(f"Available devices: {core.available_devices}")
    model = core.read_model(MODEL_PATH)
    print(f"Input: {model.inputs[0].get_any_name()} {model.inputs[0].get_shape()}")
    print(f"Output: {model.outputs[0].get_any_name()} {model.outputs[0].get_shape()}")
    compiled = core.compile_model(model, device)
    return compiled


def preprocess_rgb_255(frame):
    """RGB 0-255, NCHW, letterbox"""
    h, w = frame.shape[:2]
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.float32)
    scale = INPUT_SIZE / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img, (new_w, new_h))
    letterboxed = np.zeros((INPUT_SIZE, INPUT_SIZE, 3), dtype=np.float32)
    x_off = (INPUT_SIZE - new_w) // 2
    y_off = (INPUT_SIZE - new_h) // 2
    letterboxed[y_off:y_off + new_h, x_off:x_off + new_w] = resized
    inp = np.transpose(letterboxed, (2, 0, 1))[np.newaxis, ...]
    return inp, scale, x_off, y_off


def preprocess_rgb_1(frame):
    """RGB 0-1, NCHW, letterbox"""
    h, w = frame.shape[:2]
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    scale = INPUT_SIZE / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img, (new_w, new_h))
    letterboxed = np.zeros((INPUT_SIZE, INPUT_SIZE, 3), dtype=np.float32)
    x_off = (INPUT_SIZE - new_w) // 2
    y_off = (INPUT_SIZE - new_h) // 2
    letterboxed[y_off:y_off + new_h, x_off:x_off + new_w] = resized
    inp = np.transpose(letterboxed, (2, 0, 1))[np.newaxis, ...]
    return inp, scale, x_off, y_off


def preprocess_bgr_255(frame):
    """BGR 0-255, NCHW, letterbox"""
    h, w = frame.shape[:2]
    img = frame.astype(np.float32)
    scale = INPUT_SIZE / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img, (new_w, new_h))
    letterboxed = np.zeros((INPUT_SIZE, INPUT_SIZE, 3), dtype=np.float32)
    x_off = (INPUT_SIZE - new_w) // 2
    y_off = (INPUT_SIZE - new_h) // 2
    letterboxed[y_off:y_off + new_h, x_off:x_off + new_w] = resized
    inp = np.transpose(letterboxed, (2, 0, 1))[np.newaxis, ...]
    return inp, scale, x_off, y_off


def preprocess_stretch_rgb_255(frame):
    """RGB 0-255, NCHW, 直接拉伸到 640x640"""
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.float32)
    resized = cv2.resize(img, (INPUT_SIZE, INPUT_SIZE))
    inp = np.transpose(resized, (2, 0, 1))[np.newaxis, ...]
    return inp, None, 0, 0


def decode_output(output, img_w, img_h, scale, x_off, y_off, conf_thr=0.25):
    """
    output shape: [1, 300, 57]
    假设格式: [x1, y1, x2, y2, conf, class, kpt0_x, kpt0_y, kpt0_conf, ...]
    """
    dets = []
    arr = output[0]
    for i in range(arr.shape[0]):
        conf = arr[i, 4]
        if conf < conf_thr:
            continue

        x1 = (max(0.0, min(arr[i, 0], INPUT_SIZE)) - x_off) / scale
        y1 = (max(0.0, min(arr[i, 1], INPUT_SIZE)) - y_off) / scale
        x2 = (max(0.0, min(arr[i, 2], INPUT_SIZE)) - x_off) / scale
        y2 = (max(0.0, min(arr[i, 3], INPUT_SIZE)) - y_off) / scale

        kps = []
        for k in range(17):
            kx = (max(0.0, min(arr[i, 6 + k * 3], INPUT_SIZE)) - x_off) / scale
            ky = (max(0.0, min(arr[i, 7 + k * 3], INPUT_SIZE)) - y_off) / scale
            kv = arr[i, 8 + k * 3]
            kps.append((kx, ky, kv))

        dets.append({
            'bbox': (x1, y1, x2, y2),
            'conf': conf,
            'kps': kps,
        })
    return dets


def decode_output_stretch(output, img_w, img_h, conf_thr=0.25):
    """拉伸模式：坐标直接按比例映射"""
    dets = []
    arr = output[0]
    scale_x = img_w / INPUT_SIZE
    scale_y = img_h / INPUT_SIZE
    for i in range(arr.shape[0]):
        conf = arr[i, 4]
        if conf < conf_thr:
            continue
        x1 = arr[i, 0] * scale_x
        y1 = arr[i, 1] * scale_y
        x2 = arr[i, 2] * scale_x
        y2 = arr[i, 3] * scale_y
        kps = []
        for k in range(17):
            kx = arr[i, 6 + k * 3] * scale_x
            ky = arr[i, 7 + k * 3] * scale_y
            kv = arr[i, 8 + k * 3]
            kps.append((kx, ky, kv))
        dets.append({'bbox': (x1, y1, x2, y2), 'conf': conf, 'kps': kps})
    return dets


def nms(dets, thresh=0.45):
    if not dets:
        return []
    dets = sorted(dets, key=lambda x: x['conf'], reverse=True)
    keep = []
    while dets:
        best = dets.pop(0)
        keep.append(best)
        dets = [d for d in dets if iou(best['bbox'], d['bbox']) < thresh]
    return keep


def iou(a, b):
    x1 = max(a[0], b[0])
    y1 = max(a[1], b[1])
    x2 = min(a[2], b[2])
    y2 = min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def draw_detections(frame, dets, title=""):
    out = frame.copy()
    h, w = out.shape[:2]
    cv2.putText(out, f"{title} | dets={len(dets)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    for det in dets:
        x1, y1, x2, y2 = map(int, det['bbox'])
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(out, f"{det['conf']:.2f}", (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        for k, (kx, ky, kv) in enumerate(det['kps']):
            if kv > 0.3:
                color = (0, 255, 0) if k < 5 else (0, 0, 255) if k < 11 else (255, 0, 0)
                cv2.circle(out, (int(kx), int(ky)), 3, color, -1)
        for s, e in COCO_SKELETON:
            if det['kps'][s][2] > 0.3 and det['kps'][e][2] > 0.3:
                p1 = (int(det['kps'][s][0]), int(det['kps'][s][1]))
                p2 = (int(det['kps'][e][0]), int(det['kps'][e][1]))
                cv2.line(out, p1, p2, (255, 255, 0), 1)
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=str, default=None, help='Test image path')
    parser.add_argument('--camera', action='store_true', help='Use camera')
    parser.add_argument('--flip', action='store_true', help='Flip image vertically')
    parser.add_argument('--output-dir', type=str, default='/tmp/yolo_test', help='Output dir')
    parser.add_argument('--conf', type=float, default=0.25)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)

    compiled = load_model("CPU")
    input_name = compiled.inputs[0].get_any_name()
    output_name = compiled.outputs[0].get_any_name()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"Failed to load {args.image}")
            return
    elif args.camera:
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        ret, frame = cap.read()
        cap.release()
        if not ret:
            print("Camera failed")
            return
        cv2.imwrite(os.path.join(args.output_dir, "capture_raw.jpg"), frame)
    else:
        print("Use --image or --camera")
        return

    h, w = frame.shape[:2]
    print(f"Input image: {w}x{h}")

    if args.flip:
        frame = cv2.flip(frame, 0)
        cv2.imwrite(os.path.join(args.output_dir, "capture_flipped.jpg"), frame)

    variants = [
        ("rgb_255_letterbox", preprocess_rgb_255, decode_output),
        ("rgb_1_letterbox", preprocess_rgb_1, decode_output),
        ("bgr_255_letterbox", preprocess_bgr_255, decode_output),
        ("rgb_255_stretch", preprocess_stretch_rgb_255, decode_output_stretch),
    ]

    for name, pre_fn, dec_fn in variants:
        inp, scale, x_off, y_off = pre_fn(frame)
        out = compiled([inp])
        if scale is None:
            dets = dec_fn(out[output_name], w, h, conf_thr=args.conf)
        else:
            dets = dec_fn(out[output_name], w, h, scale, x_off, y_off, conf_thr=args.conf)
        dets = nms(dets)
        print(f"\n[{name}] detections: {len(dets)}")
        if dets:
            for i, d in enumerate(dets[:3]):
                print(f"  det {i}: conf={d['conf']:.3f}, bbox={d['bbox']}")
        vis = draw_detections(frame, dets, name)
        out_path = os.path.join(args.output_dir, f"result_{name}.jpg")
        cv2.imwrite(out_path, vis)
        print(f"  saved: {out_path}")


if __name__ == "__main__":
    main()
