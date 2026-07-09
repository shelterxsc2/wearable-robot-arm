# -*- coding: utf-8 -*-
"""对比 yolo26s-pose.onnx 和 trial0 已有的 yolo26n-pose OpenVINO IR"""
import os
import cv2
import numpy as np
import openvino as ov

COCO_SKELETON = [
    (0, 1), (0, 2), (1, 3), (2, 4),
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10),
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
    (5, 11), (6, 12), (5, 0), (6, 0),
]


def load_onnx(path):
    core = ov.Core()
    model = core.read_model(path)
    compiled = core.compile_model(model, "CPU")
    return compiled, compiled.inputs[0].get_any_name(), compiled.outputs[0].get_any_name()


def load_ir(xml_path):
    core = ov.Core()
    model = core.read_model(xml_path)
    compiled = core.compile_model(model, "GPU")
    return compiled, compiled.inputs[0], compiled.outputs[0]


def preprocess(frame):
    h, w = frame.shape[:2]
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    scale = 640 / max(h, w)
    new_w, new_h = int(w * scale), int(h * scale)
    resized = cv2.resize(img, (new_w, new_h))
    letterboxed = np.zeros((640, 640, 3), dtype=np.float32)
    x_off = (640 - new_w) // 2
    y_off = (640 - new_h) // 2
    letterboxed[y_off:y_off + new_h, x_off:x_off + new_w] = resized
    inp = np.transpose(letterboxed, (2, 0, 1))[np.newaxis, ...]
    return inp, scale, x_off, y_off


def infer(model, input_name, output_name, frame):
    inp, scale, x_off, y_off = preprocess(frame)
    out = model([inp])[output_name]
    arr = out[0]
    dets = []
    for i in range(arr.shape[0]):
        conf = arr[i, 4]
        if conf < 0.25:
            continue
        x1 = (arr[i, 0] - x_off) / scale
        y1 = (arr[i, 1] - y_off) / scale
        x2 = (arr[i, 2] - x_off) / scale
        y2 = (arr[i, 3] - y_off) / scale
        kps = []
        for k in range(17):
            kx = (arr[i, 6 + k * 3] - x_off) / scale
            ky = (arr[i, 7 + k * 3] - y_off) / scale
            kv = arr[i, 8 + k * 3]
            kps.append((kx, ky, kv))
        dets.append({'bbox': (x1, y1, x2, y2), 'conf': conf, 'kps': kps})
    return dets


def draw(frame, dets, title):
    out = frame.copy()
    cv2.putText(out, f"{title} dets={len(dets)}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    for d in dets[:5]:
        x1, y1, x2, y2 = map(int, d['bbox'])
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(out, f"{d['conf']:.2f}", (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        for s, e in COCO_SKELETON:
            if d['kps'][s][2] > 0.3 and d['kps'][e][2] > 0.3:
                p1 = (int(d['kps'][s][0]), int(d['kps'][s][1]))
                p2 = (int(d['kps'][e][0]), int(d['kps'][e][1]))
                cv2.line(out, p1, p2, (255, 0, 0), 1)
        for k, (kx, ky, kv) in enumerate(d['kps']):
            if kv > 0.3:
                cv2.circle(out, (int(kx), int(ky)), 2, (0, 0, 255), -1)
    return out


def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print("Camera failed")
        return
    frame = cv2.flip(frame, 0)
    cv2.imwrite('/tmp/compare_raw.jpg', frame)

    models = [
        ("yolo26s-pose.onnx (CPU)", *load_onnx('/home/time/work/mymodel/yolo26s-pose.onnx')),
        ("yolo26n-pose IR (GPU)", *load_ir('/home/time/work/trial0/models/yolo26n-pose (1)_openvino_model/yolo26n-pose (1).xml')),
    ]

    for title, model, in_name, out_name in models:
        dets = infer(model, in_name, out_name, frame)
        print(f"\n{title}: {len(dets)} detections")
        for i, d in enumerate(dets[:3]):
            print(f"  det {i}: conf={d['conf']:.3f} bbox={d['bbox']}")
        vis = draw(frame, dets, title)
        out_path = f"/tmp/compare_{title.split()[0]}.jpg"
        cv2.imwrite(out_path, vis)
        print(f"  saved: {out_path}")


if __name__ == "__main__":
    main()
