"""
Pitch 静止态掐死修改的仿真验证
"""
import sys
sys.path.insert(0, '/home/elf/work/twice/simulation')

import numpy as np
from stm32_scurve_sim import simulate_events

# 真实 VOFA 中残余抖动段参数：位移 0.0001~0.0003，速度 < 0.02
# 将 deadband 设小，模拟云台对微小指令的响应

params = {'deadband': 0.0001, 'is_gimbal': True}

# 场景A：修改前 — 到位后 100ms 发极小微调指令
events_a = [
    (0.0,  1.40,    0x01),
    (2.0,  1.4003, 0x01),   # 微调 +0.0003（匹配 VOFA 残余段位移）
    (5.0,  1.4003, 0x01),
]

# 场景B：修改后 — 不发微调
events_b = [
    (0.0,  1.40,    0x01),
    (5.0,  1.40,    0x01),
]

r_a = simulate_events(events_a, version='current', dt=0.001, handle_params=params)
r_b = simulate_events(events_b, version='current', dt=0.001, handle_params=params)

print("=" * 60)
print("Pitch 静止态掐死修改 — 仿真对比")
print("=" * 60)

def analyze_post_stop(t, v, a, s, label, t_start=2.0):
    mask = t >= t_start
    if not np.any(mask):
        print(f"\n【{label}】无数据")
        return
    
    print(f"\n【{label}】:")
    print(f"  {t_start}s 后最大速度: {np.max(np.abs(v[mask])):.6f}")
    print(f"  {t_start}s 后最大加速度: {np.max(np.abs(a[mask])):.6f}")
    print(f"  {t_start}s 后位移变化: {s[mask][-1] - s[mask][0]:.6f}")
    
    active = np.where(np.abs(v[mask]) > 0.001)[0]
    if len(active) > 0:
        duration_ms = (active[-1] - active[0] + 1)
        print(f"  ⚠️  残余运动段: {duration_ms}ms")
        print(f"      峰值速度: {np.max(np.abs(v[mask][active[0]:active[-1]+1])):.6f}")
        print(f"      峰值加速度: {np.max(np.abs(a[mask][active[0]:active[-1]+1])):.4f}")
    else:
        print(f"  ✅ Clean stop")

analyze_post_stop(r_a['t'], r_a['v'], r_a['a'], r_a['s'], "场景A：修改前（发微调）")
analyze_post_stop(r_b['t'], r_b['v'], r_b['a'], r_b['s'], "场景B：修改后（掐死）")

# 找场景A中 2.0s 附近的运动段细节
print("\n" + "=" * 60)
print("场景A 2.0s ~ 2.3s 逐帧（每 10ms）:")
print("=" * 60)
print(f"{'time':>8s} | {'v':>10s} {'a':>10s} {'s':>10s} {'state':>6s}")
print("-" * 55)

for t_sample in np.arange(2.0, 2.31, 0.01):
    idx = int(t_sample / 0.001)
    if idx < len(r_a['v']):
        print(f"{t_sample:8.3f} | {r_a['v'][idx]:10.6f} {r_a['a'][idx]:10.4f} {r_a['s'][idx]:10.6f} {int(r_a['state'][idx]):6d}")

print("\n结论:")
print("- 场景A（修改前）2.0s 后发微调 → 下位机启动残余运动段 → '抖一下'")
print("- 场景B（修改后）2.0s 后不发指令 → v/a 保持为 0 → 平稳停止")
