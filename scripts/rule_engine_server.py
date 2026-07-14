#!/usr/bin/env python3
"""
RuleEngine Python Socket 服务端 —— RKNNLite v2

模型: rule_engine_v2.rknn
输入 (10个):
  [0] kpts        [1,20,2] float32  -> 40 floats
  [1] bbox        [1,4]   float32  -> 4 floats  (x1,y1,x2,y2)
  [2] valid_mask  [1,20]  float32  -> 20 floats
  [3..9] state_fb [1,1]   int64    -> 7 int64s
协议: 44f + 20f + 7q = 312 bytes
输出 (7个): 7q = 56 bytes
"""

import struct
import socket
import os
import sys
import traceback
from pathlib import Path
import numpy as np
from rknnlite.api import RKNNLite

SOCK_PATH = "/tmp/rule_engine.sock"
MODEL_PATH = str(Path(__file__).resolve().parent.parent / "models" / "rule_engine_v2.rknn")

INPUT_FMT = "44f20f7q"
INPUT_SIZE = struct.calcsize(INPUT_FMT)   # 312
OUTPUT_FMT = "7q"
OUTPUT_SIZE = struct.calcsize(OUTPUT_FMT)  # 56


def main():
    if os.path.exists(SOCK_PATH):
        os.remove(SOCK_PATH)

    print("[RuleEngine-v2] Loading model...", flush=True)
    rknn = RKNNLite(verbose=False)
    if rknn.load_rknn(MODEL_PATH) != 0:
        print("[RuleEngine-v2] Load failed", flush=True)
        sys.exit(1)
    # 固定单核，避免与 C++ Body/Face 模型抢 NPU
    if rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0) != 0:
        print("[RuleEngine-v2] Init runtime failed", flush=True)
        sys.exit(1)

    # Warmup: 消除首次 inference 的 NPU 初始化延迟
    print("[RuleEngine-v2] Warmup...", flush=True)
    dummy = [
        np.zeros((1, 20, 2), dtype=np.float32),
        np.zeros((1, 4), dtype=np.float32),
        np.zeros((1, 20), dtype=np.float32),
    ] + [np.zeros((1, 1), dtype=np.int64) for _ in range(7)]
    rknn.inference(inputs=dummy)
    print("[RuleEngine-v2] Ready", flush=True)

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(SOCK_PATH)
    s.listen(5)
    os.chmod(SOCK_PATH, 0o666)
    print(f"[RuleEngine-v2] Listening on {SOCK_PATH}", flush=True)

    while True:
        conn, _ = s.accept()
        try:
            while True:
                data = b""
                while len(data) < INPUT_SIZE:
                    chunk = conn.recv(INPUT_SIZE - len(data))
                    if not chunk:
                        break
                    data += chunk
                if len(data) != INPUT_SIZE:
                    break

                vals = struct.unpack(INPUT_FMT, data)
                kpts = np.array(vals[:40], dtype=np.float32).reshape(1, 20, 2)
                bbox = np.array(vals[40:44], dtype=np.float32).reshape(1, 4)
                valid_mask = np.array(vals[44:64], dtype=np.float32).reshape(1, 20)
                fb = [np.array([[v]], dtype=np.int64) for v in vals[64:]]

                outputs = rknn.inference(inputs=[kpts, bbox, valid_mask] + fb)
                result = [int(o.item()) for o in outputs]

                conn.sendall(struct.pack(OUTPUT_FMT, *result))
        except Exception as e:
            print(f"[RuleEngine-v2] Error: {e}", flush=True)
            traceback.print_exc()
        finally:
            conn.close()


if __name__ == "__main__":
    main()
