"""
旧版 vs 最新版 对比验证 v2
重点复现用户描述的两大典型问题的根因差异
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from stm32_scurve_sim import SpeedPlanHandle, speed_plan_update, calc_decel_dist


def simulate_from_preset(preset_v, preset_a, preset_state, preset_s,
                         events, version, dt=0.001):
    """
    从预设状态开始仿真（模拟运动中被打断的瞬间）
    preset_v/a/state/s: 打断前一瞬间的状态
    events: [(inject_time, target, cmd_type), ...]
    """
    handle = SpeedPlanHandle(version=version)
    if version == 'old':
        handle.scale_short = [0.15, 0.20, 0.30]
        handle.j_limits_short = [6.0, 8.0, 10.0]
        handle.deadband = 0.025
    
    # 预置状态
    handle.v = preset_v
    handle.a = preset_a
    handle.state = preset_state
    handle.s = preset_s
    handle.direction_flag = 1.0 if preset_v >= 0 else -1.0
    
    position_actual = 0.0  # 简化：position_actual=0，target即error_s
    
    times, targets, plan_s, plan_v, plan_a, plan_state = [], [], [], [], [], []
    
    t = 0.0
    current_target = 0.0
    current_cmd_type = 0x01
    event_idx = 0
    events = sorted(events, key=lambda x: x[0])
    
    while t < (events[-1][0] + 1.5 if events else 1.5):
        while event_idx < len(events) and events[event_idx][0] <= t:
            current_target = events[event_idx][1]
            current_cmd_type = events[event_idx][2]
            handle.cmd_type = current_cmd_type
            handle.state = 1  # 强制 init
            event_idx += 1
        
        if handle.state != 0:
            speed_plan_update(handle, position_actual, current_target, t, dt)
        
        plan_pos = handle.position_initial + handle.direction_flag * handle.s
        position_actual += (plan_pos - position_actual) * 0.05
        
        times.append(t)
        targets.append(current_target)
        plan_s.append(handle.s)
        plan_v.append(handle.v)
        plan_a.append(handle.a)
        plan_state.append(handle.state)
        t += dt
    
    return {
        't': np.array(times),
        'target': np.array(targets),
        's': np.array(plan_s),
        'v': np.array(plan_v),
        'a': np.array(plan_a),
        'state': np.array(plan_state),
    }


print("=" * 80)
print("场景对比：旧版(old) vs 最新版(current)")
print("=" * 80)

# ==================== 场景1：末期急停晃动 ====================
# 模拟：电机正以 v=0.8, a=0（匀速phase4）运动，突然收到很近的目标
print("\n【场景1】末期打断：高速匀速中突然 target=0.05（很近）")
print("  预设: v=0.8, a=0, state=phase4, s=0.5 (已走一半)")

s1_old = simulate_from_preset(
    preset_v=0.8, preset_a=0.0, preset_state=6, preset_s=0.5,
    events=[(0.0, 0.05, 0x01)], version='old'
)
s1_cur = simulate_from_preset(
    preset_v=0.8, preset_a=0.0, preset_state=6, preset_s=0.5,
    events=[(0.0, 0.05, 0x01)], version='current'
)

# 计算关键指标
# 1. 是否能在 error_s 内停下
for label, res in [("旧版", s1_old), ("最新版", s1_cur)]:
    # 找 init 后的峰值速度
    init_idx = np.where(np.diff(res['state']) > 0)[0]
    if len(init_idx) > 0:
        start = init_idx[0]
        peak_v = np.max(np.abs(res['v'][start:]))
        final_v = np.abs(res['v'][-1])
        # 统计 a 的跳变
        a_jumps = np.sum(np.abs(np.diff(res['a'])) > 1.0)
        print(f"  {label}: peak_v_after_init={peak_v:.3f}, final_v={final_v:.4f}, a_jumps={a_jumps}")


# ==================== 场景2：idle后微调抖动 ====================
print("\n【场景2】idle后微调：先idle，然后 target=0.06，再 target=0.05")
print("  模拟：长距离运动后进入idle，上位机连续发两个微调指令")

s2_old = simulate_from_preset(
    preset_v=0.0, preset_a=0.0, preset_state=0, preset_s=0.0,
    events=[(0.0, 0.06, 0x01), (0.08, 0.05, 0x01)], version='old'
)
s2_cur = simulate_from_preset(
    preset_v=0.0, preset_a=0.0, preset_state=0, preset_s=0.0,
    events=[(0.0, 0.06, 0x01), (0.08, 0.05, 0x01)], version='current'
)

for label, res in [("旧版", s2_old), ("最新版", s2_cur)]:
    # 找两段运动之间的状态切换
    switches = np.sum(np.diff(res['state']) != 0)
    resets = 0
    s = res['s']
    for i in range(1, len(s)):
        if s[i-1] > 0.01 and s[i] < 0.001:
            resets += 1
    max_a = np.max(np.abs(res['a']))
    print(f"  {label}: state_switches={switches}, s_resets={resets}, max_a={max_a:.2f}")


# ==================== 场景3：运动中高频打断 ====================
print("\n【场景3】运动中高频打断：v=1.0, phase4，连续3个指令")
print("  0.0s: target=0.30 | 0.05s: target=0.25 | 0.10s: target=0.20")

events_hf = [
    (0.0, 0.30, 0x01),
    (0.05, 0.25, 0x01),
    (0.10, 0.20, 0x01),
]

s3_old = simulate_from_preset(
    preset_v=1.0, preset_a=0.0, preset_state=6, preset_s=0.8,
    events=events_hf, version='old'
)
s3_cur = simulate_from_preset(
    preset_v=1.0, preset_a=0.0, preset_state=6, preset_s=0.8,
    events=events_hf, version='current'
)

for label, res in [("旧版", s3_old), ("最新版", s3_cur)]:
    switches = np.sum(np.diff(res['state']) != 0)
    resets = 0
    s = res['s']
    for i in range(1, len(s)):
        if s[i-1] > 0.01 and s[i] < 0.001:
            resets += 1
    max_a = np.max(np.abs(res['a']))
    a_jumps = np.sum(np.abs(np.diff(res['a'])) > 1.0)
    # 最终是否到位（v≈0, state=idle）
    final_idle = res['state'][-1] == 0
    print(f"  {label}: switches={switches}, resets={resets}, max_a={max_a:.2f}, a_jumps={a_jumps}, final_idle={final_idle}")


# ==================== 可视化 ====================
fig, axes = plt.subplots(3, 3, figsize=(16, 11))

def plot_row(ax_row, res_old, res_cur, title):
    ax_row[0].plot(res_old['t'], res_old['s'], 'b-', lw=1.2, label='old')
    ax_row[0].plot(res_cur['t'], res_cur['s'], 'r-', lw=1.2, label='current')
    ax_row[0].set_ylabel('s')
    ax_row[0].set_title(title)
    ax_row[0].legend()
    
    ax_row[1].plot(res_old['t'], res_old['v'], 'b-', lw=1.2, label='old')
    ax_row[1].plot(res_cur['t'], res_cur['v'], 'r-', lw=1.2, label='current')
    ax_row[1].set_ylabel('v')
    ax_row[1].legend()
    
    ax_row[2].plot(res_old['t'], res_old['a'], 'b-', lw=1.2, label='old')
    ax_row[2].plot(res_cur['t'], res_cur['a'], 'r-', lw=1.2, label='current')
    ax_row[2].set_ylabel('a')
    ax_row[2].legend()

plot_row(axes[0], s1_old, s1_cur, 'Scene 1: Late interruption (v=0.8->target=0.05)')
plot_row(axes[1], s2_old, s2_cur, 'Scene 2: Idle + micro-adjust x2')
plot_row(axes[2], s3_old, s3_cur, 'Scene 3: High-freq interrupts (v=1.0)')

for ax in axes[-1]:
    ax.set_xlabel('Time (s)')

plt.tight_layout()
plt.savefig('compare_v2_direct.png', dpi=150)
print('\nSaved compare_v2_direct.png')
