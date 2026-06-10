"""
旧版 vs 最新版 对比验证 v3
修正状态检测逻辑，确保能正确捕获 init 注入后的行为差异
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from stm32_scurve_sim import SpeedPlanHandle, speed_plan_update, calc_decel_dist


def simulate_from_preset(preset_v, preset_a, preset_state, preset_s,
                         events, version, dt=0.001):
    handle = SpeedPlanHandle(version=version)
    if version == 'old':
        handle.scale_short = [0.15, 0.20, 0.30]
        handle.j_limits_short = [6.0, 8.0, 10.0]
        handle.deadband = 0.025
    
    handle.v = preset_v
    handle.a = preset_a
    handle.state = preset_state
    handle.s = preset_s
    handle.direction_flag = 1.0 if preset_v >= 0 else -1.0
    
    position_actual = 0.0
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
            handle.state = 1
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


def analyze_after_init(res):
    """找到 init 注入点（state 从非1变为1），分析之后的行为"""
    # state == 1 是 init，找第一个 init 出现的位置
    init_mask = res['state'] == 1
    if not np.any(init_mask):
        return {}
    init_idx = np.where(init_mask)[0][0]
    post = slice(init_idx, None)
    
    peak_v = np.max(np.abs(res['v'][post]))
    peak_a = np.max(np.abs(res['a'][post]))
    final_v = np.abs(res['v'][-1])
    final_idle = res['state'][-1] == 0
    
    # a 跳变
    a_jumps = np.sum(np.abs(np.diff(res['a'][post])) > 1.0)
    
    # s 重置（init 之后又有 s 清零）
    resets = 0
    s = res['s'][post]
    for i in range(1, len(s)):
        if s[i-1] > 0.01 and s[i] < 0.001:
            resets += 1
    
    # 状态切换
    switches = np.sum(np.diff(res['state'][post]) != 0)
    
    # 找第一个 a 的峰值时刻（从 init 到峰值的时间）
    a_post = np.abs(res['a'][post])
    if len(a_post) > 0:
        peak_a_idx = np.argmax(a_post)
        time_to_peak_a = peak_a_idx * 0.001
    else:
        time_to_peak_a = 0
    
    return {
        'peak_v': peak_v,
        'peak_a': peak_a,
        'final_v': final_v,
        'final_idle': final_idle,
        'a_jumps': a_jumps,
        'resets': resets,
        'switches': switches,
        'time_to_peak_a': time_to_peak_a,
    }


print("=" * 80)
print("旧版(old) vs 最新版(current) 核心场景对比")
print("=" * 80)

# ==================== 场景1：末期急停晃动 ====================
print("\n【场景1】长距离运动中被打断：v=0.8, a=0, 突然 target=0.05")
print("  旧版预期：init→phase1，a=0，v保留0.8，按S=0.05规划→v_limit极低→冲过头")
print("  新版预期：init→制动安全检查→v降到安全值→phase3_end→平滑减速")

s1_old = simulate_from_preset(0.8, 0.0, 6, 0.5, [(0.0, 0.05, 0x01)], 'old')
s1_cur = simulate_from_preset(0.8, 0.0, 6, 0.5, [(0.0, 0.05, 0x01)], 'current')

m1_old = analyze_after_init(s1_old)
m1_cur = analyze_after_init(s1_cur)

print(f"\n  指标                    | 旧版      | 最新版    | 意义")
print(f"  {'-'*65}")
for k in ['peak_v', 'peak_a', 'final_v', 'time_to_peak_a', 'a_jumps', 'resets', 'switches', 'final_idle']:
    v_old = m1_old.get(k, '-')
    v_cur = m1_cur.get(k, '-')
    if isinstance(v_old, float):
        v_old = f"{v_old:.4f}"
    if isinstance(v_cur, float):
        v_cur = f"{v_cur:.4f}"
    print(f"  {k:22s} | {v_old:9s} | {v_cur:9s} |")


# ==================== 场景2：idle后微调 ====================
print("\n【场景2】idle后连续微调：target=0.06 → 0.05 (间隔50ms)")
print("  旧版预期：j_limit=6, scale=0.15, deadband=0.025 → 小位移极慢，但频繁响应")
print("  新版预期：j_limit=12, scale=0.30, deadband=0.05 → 响应更快，但死区更大可能idle")

s2_old = simulate_from_preset(0, 0, 0, 0, [(0.0, 0.06, 0x01), (0.05, 0.05, 0x01)], 'old')
s2_cur = simulate_from_preset(0, 0, 0, 0, [(0.0, 0.06, 0x01), (0.05, 0.05, 0x01)], 'current')

m2_old = analyze_after_init(s2_old)
m2_cur = analyze_after_init(s2_cur)

print(f"\n  指标                    | 旧版      | 最新版    | 意义")
print(f"  {'-'*65}")
for k in ['peak_v', 'peak_a', 'final_v', 'time_to_peak_a', 'a_jumps', 'resets', 'switches', 'final_idle']:
    v_old = m2_old.get(k, '-')
    v_cur = m2_cur.get(k, '-')
    if isinstance(v_old, float):
        v_old = f"{v_old:.4f}"
    if isinstance(v_cur, float):
        v_cur = f"{v_cur:.4f}"
    print(f"  {k:22s} | {v_old:9s} | {v_cur:9s} |")


# ==================== 场景3：运动中多次打断 ====================
print("\n【场景3】运动中连续3次打断：v=1.0, 指令间隔50ms")
print("  0.00s: target=0.30 | 0.05s: target=0.25 | 0.10s: target=0.20")

events3 = [(0.0, 0.30, 0x01), (0.05, 0.25, 0x01), (0.10, 0.20, 0x01)]
s3_old = simulate_from_preset(1.0, 0.0, 6, 0.8, events3, 'old')
s3_cur = simulate_from_preset(1.0, 0.0, 6, 0.8, events3, 'current')

m3_old = analyze_after_init(s3_old)
m3_cur = analyze_after_init(s3_cur)

print(f"\n  指标                    | 旧版      | 最新版    | 意义")
print(f"  {'-'*65}")
for k in ['peak_v', 'peak_a', 'final_v', 'time_to_peak_a', 'a_jumps', 'resets', 'switches', 'final_idle']:
    v_old = m3_old.get(k, '-')
    v_cur = m3_cur.get(k, '-')
    if isinstance(v_old, float):
        v_old = f"{v_old:.4f}"
    if isinstance(v_cur, float):
        v_cur = f"{v_cur:.4f}"
    print(f"  {k:22s} | {v_old:9s} | {v_cur:9s} |")


# ==================== 可视化 ====================
fig, axes = plt.subplots(3, 3, figsize=(16, 11))

def plot_row(ax_row, res_old, res_cur, title):
    ax_row[0].plot(res_old['t'], res_old['s'], 'b-', lw=1.5, label='old')
    ax_row[0].plot(res_cur['t'], res_cur['s'], 'r-', lw=1.5, label='current')
    ax_row[0].set_ylabel('s (rad)')
    ax_row[0].set_title(title)
    ax_row[0].legend(loc='upper right')
    ax_row[0].set_ylim(bottom=-0.02)
    
    ax_row[1].plot(res_old['t'], res_old['v'], 'b-', lw=1.5, label='old')
    ax_row[1].plot(res_cur['t'], res_cur['v'], 'r-', lw=1.5, label='current')
    ax_row[1].set_ylabel('v (rad/s)')
    ax_row[1].legend()
    ax_row[1].axhline(0, color='k', ls='--', alpha=0.3)
    
    ax_row[2].plot(res_old['t'], res_old['a'], 'b-', lw=1.5, label='old')
    ax_row[2].plot(res_cur['t'], res_cur['a'], 'r-', lw=1.5, label='current')
    ax_row[2].set_ylabel('a (rad/s²)')
    ax_row[2].legend()
    ax_row[2].axhline(0, color='k', ls='--', alpha=0.3)

plot_row(axes[0], s1_old, s1_cur, 'Scene 1: Late interruption (v=0.8 → target=0.05)')
plot_row(axes[1], s2_old, s2_cur, 'Scene 2: Idle + micro-adjust x2')
plot_row(axes[2], s3_old, s3_cur, 'Scene 3: 3 interrupts in 100ms (v=1.0)')

for ax in axes[-1]:
    ax.set_xlabel('Time (s)')

plt.tight_layout()
plt.savefig('compare_v3_final.png', dpi=150)
print('\nSaved compare_v3_final.png')

# 额外：打印场景1的关键帧，验证制动安全逻辑
print("\n【场景1 详细帧对比】init 后前 30ms")
print(f"{'t':>6s} | {'old_s':>7s} | {'old_v':>7s} | {'old_a':>7s} | {'old_st':>6s} | {'cur_s':>7s} | {'cur_v':>7s} | {'cur_a':>7s} | {'cur_st':>6s}")
print("-" * 90)
for i in range(min(35, len(s1_old['t']))):
    print(f"{s1_old['t'][i]:6.3f} | {s1_old['s'][i]:7.4f} | {s1_old['v'][i]:7.4f} | {s1_old['a'][i]:7.4f} | {s1_old['state'][i]:6d} | "
          f"{s1_cur['s'][i]:7.4f} | {s1_cur['v'][i]:7.4f} | {s1_cur['a'][i]:7.4f} | {s1_cur['state'][i]:6d}")
