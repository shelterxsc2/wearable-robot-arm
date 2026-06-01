#!/usr/bin/env python3
import struct
import socket
import os
import sys
import traceback
import numpy as np
from rknnlite.api import RKNNLite

SOCK_PATH = "/tmp/rule_engine.sock"
MODEL_PATH = "/home/elf/work/twice/models/rule_engine_handcraft.rknn"

# kpts[40 floats] + valid_mask[20 floats] + state_fb[7 int64s]
INPUT_FMT = "40f20f7q"
INPUT_SIZE = struct.calcsize(INPUT_FMT)
OUTPUT_FMT = "7q"
OUTPUT_SIZE = struct.calcsize(OUTPUT_FMT)

def main():
    if os.path.exists(SOCK_PATH):
        os.remove(SOCK_PATH)

    print("[RuleEngine] Loading model...", flush=True)
    rknn = RKNNLite(verbose=False)
    ret = rknn.load_rknn(MODEL_PATH)
    if ret != 0:
        print(f"[RuleEngine] Load failed: {ret}", flush=True)
        sys.exit(1)
    ret = rknn.init_runtime()
    if ret != 0:
        print(f"[RuleEngine] Init runtime failed: {ret}", flush=True)
        sys.exit(1)
    print("[RuleEngine] Ready", flush=True)

    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(SOCK_PATH)
    s.listen(5)
    os.chmod(SOCK_PATH, 0o666)
    print(f"[RuleEngine] Listening on {SOCK_PATH}", flush=True)

    while True:
        conn, addr = s.accept()
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
                valid_mask = np.array(vals[40:60], dtype=np.float32).reshape(1, 20)
                fb = list(vals[60:])
                fb_arrays = [np.array([[v]], dtype=np.int64) for v in fb]

                outputs = rknn.inference(inputs=[kpts, valid_mask] + fb_arrays)
                result = [int(o.item()) for o in outputs]

                out_data = struct.pack(OUTPUT_FMT, *result)
                conn.sendall(out_data)
        except Exception as e:
            print(f"[RuleEngine] Error: {e}", flush=True)
            traceback.print_exc()
        finally:
            conn.close()

if __name__ == "__main__":
    main()
