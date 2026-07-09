#!/usr/bin/env python3
"""
把当前 NumPy 规则引擎逻辑导出为 ONNX 模型，供 GPU 推理。

输入（与旧 rule_engine.onnx 保持一致）：
  - kpts_flat: [B, 40]  20 个关键点 (x,y)
  - valid_mask: [B, 20]
  - bbox: [B, 4]        (x1, y1, x2, y2)
  - fb: [B, 7]          上一帧状态反馈 [state, r_hold, l_hold, c_hold, n_hold, r_miss, l_miss]

输出：
  - state: [B, 7]

注意：所有逻辑用 torch.where 实现，避免 Python 控制流，确保 ONNX 导出成功。
"""

import os
import sys
import torch
import torch.nn as nn
import numpy as np

# 与 camera_demo_elf_pipeline.py 中 RuleEngineState 保持一致
HOLD_F = 5
RESET_F = 2
CENTER_FORCE_F = 2
NEUTRAL_FORCE_F = 1

P_EXIT = 0.18
P_WRIST_EXIT_R = 0.24
P_WRIST_EXIT_PX = 110.0

P_BBOX_DELTA_RATIO = 0.17
P_WRIST_EXTEND_RATIO = 0.25
P_DOMINANCE = 0.80

EPS = 1e-6


class RuleEngineONNX(nn.Module):
    def __init__(self):
        super().__init__()

    def forward(self, kpts_flat, valid_mask, bbox, fb):
        """
        kpts_flat: [B, 40]
        valid_mask: [B, 20]
        bbox: [B, 4]
        fb: [B, 7]
        """
        B = kpts_flat.shape[0]
        device = kpts_flat.device
        dtype = kpts_flat.dtype

        # 还原 20x2
        kpts = kpts_flat.reshape(B, 20, 2)

        # bbox
        bbox_x1 = bbox[:, 0]
        bbox_y1 = bbox[:, 1]
        bbox_x2 = bbox[:, 2]
        bbox_y2 = bbox[:, 3]
        bbox_w = torch.clamp(bbox_x2 - bbox_x1, min=1.0)

        # 取出关键点坐标
        def getx(idx):
            return kpts[:, idx, 0]

        def getv(idx):
            return valid_mask[:, idx]

        ls_x = getx(5)
        rs_x = getx(6)
        mx_shoulder = (ls_x + rs_x) * 0.5
        ls_v = getv(5)
        rs_v = getv(6)

        mx_bbox = (bbox_x1 + bbox_x2) * 0.5
        bbox_delta = mx_bbox - mx_shoulder

        # scale: 优先用肩膀宽度，否则用 bbox 宽度
        shoulder_w = torch.abs(rs_x - ls_x)
        both_valid = (ls_v > 0.0) & (rs_v > 0.0)
        scale = torch.where(both_valid, torch.clamp(shoulder_w, min=1.0), bbox_w)

        # 手腕
        lwx = getx(9)
        rwx = getx(10)
        lv_w = getv(9)
        rv_w = getv(10)

        # keep 条件，参考肩膀中心
        mx = mx_shoulder
        right_wrist_t = torch.relu((rwx - mx) / scale) * rv_w
        left_wrist_t = torch.relu((mx - lwx) / scale) * lv_w
        rw_px = right_wrist_t * scale
        lw_px = left_wrist_t * scale

        # 优势度
        right_extend = torch.where(rv_w > 0.0, rwx - rs_x, torch.zeros_like(rwx))
        left_extend = torch.where(lv_w > 0.0, ls_x - lwx, torch.zeros_like(lwx))
        diff_rl = right_extend - left_extend
        diff_lr = left_extend - right_extend

        # trigger
        right_trigger = (
            (bbox_delta > P_BBOX_DELTA_RATIO * bbox_w) &
            (right_extend > P_WRIST_EXTEND_RATIO * bbox_w) &
            (diff_rl > P_DOMINANCE) &
            (rv_w > 0.0)
        )
        left_trigger = (
            (bbox_delta < -P_BBOX_DELTA_RATIO * bbox_w) &
            (left_extend > P_WRIST_EXTEND_RATIO * bbox_w) &
            (diff_lr > P_DOMINANCE) &
            (lv_w > 0.0)
        )

        # keep
        right_keep = (right_wrist_t > P_WRIST_EXIT_R) & (rw_px > P_WRIST_EXIT_PX)
        left_keep = (left_wrist_t > P_WRIST_EXIT_R) & (lw_px > P_WRIST_EXIT_PX)

        # 反馈状态
        prev_state = fb[:, 0]
        prev_r_hold = fb[:, 1]
        prev_l_hold = fb[:, 2]
        prev_c_hold = fb[:, 3]
        prev_n_hold = fb[:, 4]
        prev_r_miss = fb[:, 5]
        prev_l_miss = fb[:, 6]

        # hold 计数器
        right_hold = torch.where(right_trigger, prev_r_hold + 1.0, torch.zeros_like(prev_r_hold))
        left_hold = torch.where(left_trigger, prev_l_hold + 1.0, torch.zeros_like(prev_l_hold))

        right_ready = right_hold >= HOLD_F
        left_ready = left_hold >= HOLD_F

        # base_state
        base_state = torch.zeros_like(prev_state)
        # right_ready and not left_ready -> 1
        base_state = torch.where(right_ready & ~left_ready, torch.ones_like(base_state), base_state)
        # not right_ready and left_ready -> 2
        base_state = torch.where(~right_ready & left_ready, torch.full_like(base_state, 2.0), base_state)
        # both ready -> 1 if right_extend >= left_extend else 2
        both_ready = right_ready & left_ready
        base_state = torch.where(
            both_ready,
            torch.where(right_extend >= left_extend, torch.ones_like(base_state), torch.full_like(base_state, 2.0)),
            base_state
        )
        # else -> prev_state
        neither_ready = ~(right_ready | left_ready)
        base_state = torch.where(neither_ready, prev_state, base_state)

        # miss 计数器
        right_keep_miss = torch.where(right_keep, torch.zeros_like(prev_r_miss), prev_r_miss + 1.0)
        left_keep_miss = torch.where(left_keep, torch.zeros_like(prev_l_miss), prev_l_miss + 1.0)

        # reset
        reset1 = (prev_state == 1.0) & (right_keep_miss >= RESET_F)
        reset2 = (prev_state == 2.0) & (left_keep_miss >= RESET_F)
        should_reset = reset1 | reset2
        state_after_reset = torch.where(should_reset, torch.zeros_like(prev_state), base_state)

        # center / neutral 强制回 0
        center_th = 0.26 * scale
        wrists_center = (torch.abs(lwx - mx) < center_th) & (torch.abs(rwx - mx) < center_th)

        disp = torch.abs(ls_x - mx)
        for idx in (6, 7, 8, 9, 10):
            disp = disp + torch.abs(getx(idx) - mx)
        disp_mean = disp / 6.0
        center_ok = (disp_mean / scale < 0.20) & wrists_center

        neutral_ok = torch.abs(right_extend - left_extend) < 0.10 * scale

        center_hold = torch.where(center_ok, prev_c_hold + 1.0, torch.zeros_like(prev_c_hold))
        neutral_hold = torch.where(neutral_ok, prev_n_hold + 1.0, torch.zeros_like(prev_n_hold))

        force0 = (center_hold >= CENTER_FORCE_F) | (neutral_hold >= NEUTRAL_FORCE_F)
        state = torch.where(force0, torch.zeros_like(prev_state), state_after_reset)

        # 输出 7 维状态
        out = torch.stack([
            state,
            right_hold,
            left_hold,
            center_hold,
            neutral_hold,
            right_keep_miss,
            left_keep_miss
        ], dim=1)

        return out


def verify_against_numpy(model, num_samples=100):
    """用随机输入对比 PyTorch 输出与 NumPy 实现输出。"""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from camera_demo_elf_pipeline import RuleEngineState

    re = RuleEngineState()
    model.eval()

    max_diff = 0.0
    for _ in range(num_samples):
        kpts = np.random.randn(20, 2).astype(np.float32) * 500 + 500
        vm = (np.random.rand(20) > 0.2).astype(np.float32)
        bbox = np.array([300.0, 100.0, 900.0, 700.0], dtype=np.float32)
        fb = np.random.randint(0, 10, size=7).astype(np.float32)

        np_out = re._rule_engine_numpy(kpts, vm, bbox, *fb)

        with torch.no_grad():
            pt_out = model(
                torch.tensor(kpts.reshape(1, 40), dtype=torch.float32),
                torch.tensor(vm.reshape(1, 20), dtype=torch.float32),
                torch.tensor(bbox.reshape(1, 4), dtype=torch.float32),
                torch.tensor(fb.reshape(1, 7), dtype=torch.float32)
            ).numpy()[0]

        diff = np.max(np.abs(np.array(np_out) - pt_out))
        max_diff = max(max_diff, diff)
        if diff > 1e-4:
            print(f"Diff={diff:.4f}, np={np_out}, pt={pt_out}")
            return False

    print(f"[Verify] max diff over {num_samples} samples: {max_diff:.6f}")
    return True


def main():
    model = RuleEngineONNX()
    model.eval()

    print("[Build] Verifying PyTorch model against NumPy implementation...")
    if not verify_against_numpy(model, num_samples=500):
        print("[Build] Verification FAILED")
        return
    print("[Build] Verification PASSED")

    # 构造 dummy 输入
    dummy_kpts = torch.randn(1, 40, dtype=torch.float32) * 500 + 500
    dummy_vm = (torch.rand(1, 20) > 0.2).float()
    dummy_bbox = torch.tensor([[300.0, 100.0, 900.0, 700.0]], dtype=torch.float32)
    dummy_fb = torch.zeros(1, 7, dtype=torch.float32)

    output_path = "/home/time/work/mymodel/rule_engine_v2.onnx"
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    print(f"[Build] Exporting ONNX to {output_path}...")
    # 固定 batch=1，避免 NPU 编译器因动态 shape 缺少 upper bound 而失败
    torch.onnx.export(
        model,
        (dummy_kpts, dummy_vm, dummy_bbox, dummy_fb),
        output_path,
        input_names=["kpts_flat", "valid_mask", "bbox", "fb"],
        output_names=["state"],
        opset_version=11,
        do_constant_folding=True,
    )
    print(f"[Build] ONNX model saved: {output_path}")


if __name__ == "__main__":
    main()
