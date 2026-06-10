"""
旧版 vs 最新版 对比验证 v4
修正：让 target 突破死区，确保新版也会运动；修正 init 检测逻辑
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from stm32_scurve_sim import SpeedPlanHandle, speed_plan_update


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


def analyze_post_event(res):
    """找 target 第一次非零的位置（事件注入点），分析之后的行为"""
    # 找 target 第一次 > 0.001 的索引
    nonzero_targets = np.where(np.abs(res['target']) > 0.001)[0]
    if len(nonzero_targets) == 0:
        return {}
    start = nonzero_targets[0]
    post = slice(start, None)
    
    peak_v = np.max(np.abs(res['v'][post]))
    peak_a = np.max(np.abs(res['a'][post]))
    final_v = np.abs(res['v'][-1])
    final_idle = res['state'][-1] == 0
    
    a_jumps = np.sum(np.abs(np.diff(res['a'][post])) > 1.0)
    
    resets = 0
    s = res['s'][post]
    for i in range(1, len(s)):
        if s[i-1] > 0.01 and s[i] < 0.001:
            resets += 1
    
    switches = np.sum(np.diff(res['state'][post]) != 0)
    
    a_post = np.abs(res['a'][post])
    if len(a_post) > 0 and np.max(a_post) > 0.001:
        peak_a_idx = np.argmax(a_post)
        time_to_peak_a = peak_a_idx * 0.001
    else:
        time_to_peak_a = 0
    
    # 关键：到位时是否 "急停"（v 从 >0.1 直接掉到 0）
    v_post = res['v'][post]
    hard_stops = 0
    for i in range(1, len(v_post)):
        if v_post[i-1] > 0.1 and v_post[i] == 0.0:
            hard_stops += 1
    
    return {
        'peak_v': peak_v,
        'peak_a': peak_a,
        'final_v': final_v,
        'final_idle': final_idle,
        'a_jumps': a_jumps,
        'resets': resets,
        'switches': switches,
        'time_to_peak_a': time_to_peak_a,
        'hard_stops': hard_stops,
    }


print("=" * 80)
print("旧版(old) vs 最新版(current) 核心场景对比")
print("=" * 80)

# ==================== 场景1：末期急停晃动 ====================
# 关键修正：target=0.06 确保突破新版死区 0.05
print("\n【场景1】长距离运动中被打断：v=0.8, a=0, 突然 target=0.06")
print("  旧版预期：init→phase1，v保留0.8，按S=0.06规划→无制动检查→冲过头")
print("  新版预期：init→制动安全检查→v降到安全值→phase3_end→平滑减速")

s1_old = simulate_from_preset(0.8, 0.0, 6, 0.5, [(0.0, 0.06, 0x01)], 'old')
s1_cur = simulate_from_preset(0.8, 0.0, 6, 0.5, [(0.0, 0.06, 0x01)], 'current')

m1_old = analyze_post_event(s1_old)
m1_cur = analyze_post_event(s1_cur)

print(f"\n  {'指标':20s} | {'旧版':>10s} | {'最新版':>10s} | {'是否改善':>8s}")
print(f"  {'-'*58}")
for k, label in [('peak_v', '峰值速度'), ('peak_a', '峰值加速度'), ('final_v', '到位残余速度'),
                 ('time_to_peak_a', '到峰值a时间'), ('a_jumps', '加速度跳变'), ('resets', 's重置次数'),
                 ('switches', '状态切换'), ('hard_stops', '急停次数'), ('final_idle', '最终idle')]:
    v_old = m1_old.get(k, '-')
    v_cur = m1_cur.get(k, '-')
    if isinstance(v_old, float):
        v_old = f"{v_old:.4f}"
    if isinstance(v_cur, float):
        v_cur = f"{v_cur:.4f}"
    improved = ''
    if k == 'final_v' and isinstance(m1_old.get(k), float) and isinstance(m1_cur.get(k), float):
        improved = '✓' if m1_cur[k] < m1_old[k] else '✗'
    elif k == 'hard_stops' and isinstance(m1_old.get(k), (int, float)) and isinstance(m1_cur.get(k), (int, float)):
        improved = '✓' if m1_cur[k] < m1_old[k] else '✗'
    elif k == 'final_idle':
        improved = '✓' if m1_cur.get(k) == True else '✗'
    print(f"  {label:20s} | {str(v_old):>10s} | {str(v_cur):>10s} | {improved:>8s}")


# ==================== 场景2：idle后微调抖动 ====================
print("\n【场景2】idle后连续微调：target=0.07 → 0.06 (间隔50ms)")
print("  旧版：j=6, scale=0.15, deadband=0.025")
print("  新版：j=12, scale=0.30, deadband=0.05")

s2_old = simulate_from_preset(0, 0, 0, 0, [(0.0, 0.07, 0x01), (0.05, 0.06, 0x01)], 'old')
s2_cur = simulate_from_preset(0, 0, 0, 0, [(0.0, 0.07, 0x01), (0.05, 0.06, 0x01)], 'current')

m2_old = analyze_post_event(s2_old)
m2_cur = analyze_post_event(s2_cur)

print(f"\n  {'指标':20s} | {'旧版':>10s} | {'最新版':>10s} | {'是否改善':>8s}")
print(f"  {'-'*58}")
for k, label in [('peak_v', '峰值速度'), ('peak_a', '峰值加速度'), ('final_v', '到位残余速度'),
                 ('time_to_peak_a', '到峰值a时间'), ('a_jumps', '加速度跳变'), ('resets', 's重置次数'),
                 ('switches', '状态切换'), ('hard_stops', '急停次数')]:
    v_old = m2_old.get(k, '-')
    v_cur = m2_cur.get(k, '-')
    if isinstance(v_old, float):
        v_old = f"{v_old:.4f}"
    if isinstance(v_cur, float):
        v_cur = f"{v_cur:.4f}"
    improved = ''
    if k in ('final_v', 'hard_stops') and isinstance(m2_old.get(k), (int, float)) and isinstance(m2_cur.get(k), (int, float)):
        improved = '✓' if m2_cur[k] < m2_old[k] else '✗'
    print(f"  {label:20s} | {str(v_old):>10s} | {str(v_cur):>10s} | {improved:>8s}")


# ==================== 场景3：运动中高频打断 ====================
print("\n【场景3】运动中连续3次打断：v=1.0, 指令间隔50ms")
print("  0.00s: target=0.35 | 0.05s: target=0.30 | 0.10s: target=0.25")

events3 = [(0.0, 0.35, 0x01), (0.05, 0.30, 0x01), (0.10, 0.25, 0x01)]
s3_old = simulate_from_preset(1.0, 0.0, 6, 0.8, events3, 'old')
s3_cur = simulate_from_preset(1.0, 0.0, 6, 0.8, events3, 'current')

m3_old = analyze_post_event(s3_old)
m3_cur = analyze_post_event(s3_cur)

print(f"\n  {'指标':20s} | {'旧版':>10s} | {'最新版':>10s} | {'是否改善':>8s}")
print(f"  {'-'*58}")
for k, label in [('peak_v', '峰值速度'), ('peak_a', '峰值加速度'), ('final_v', '到位残余速度'),
                 ('time_to_peak_a', '到峰值a时间'), ('a_jumps', '加速度跳变'), ('resets', 's重置次数'),
                 ('switches', '状态切换'), ('hard_stops', '急停次数')]:
    v_old = m3_old.get(k, '-')
    v_cur = m3_cur.get(k, '-')
    if isinstance(v_old, float):
        v_old = f"{v_old:.4f}"
    if isinstance(v_cur, float):
        v_cur = f"{v_cur:.4f}"
    improved = ''
    if k in ('final_v', 'hard_stops') and isinstance(m3_old.get(k), (int, float)) and isinstance(m3_cur.get(k), (int, float)):
        improved = '✓' if m3_cur[k] < m3_old[k] else '✗'
    print(f"  {label:20s} | {str(v_old):>10s} | {str(v_cur):>10s} | {improved:>8s}")


# ==================== 可视化 ====================
fig, axes = plt.subplots(3, 3, figsize=(16, 11))

def plot_row(ax_row, res_old, res_cur, title):
    ax_row[0].plot(res_old['t'], res_old['s'], 'b-', lw=1.5, label='old')
    ax_row[0].plot(res_cur['t'], res_cur['s'], 'r-', lw=1.5, label='current')
    ax_row[0].set_ylabel('s')
    ax_row[0].set_title(title)
    ax_row[0].legend(loc='upper right')
    ax_row[0].set_ylim(bottom=-0.02)
    
    ax_row[1].plot(res_old['t'], res_old['v'], 'b-', lw=1.5, label='old')
    ax_row[1].plot(res_cur['t'], res_cur['v'], 'r-', lw=1.5, label='current')
    ax_row[1].set_ylabel('v')
    ax_row[1].legend()
    ax_row[1].axhline(0, color='k', ls='--', alpha=0.3)
    
    ax_row[2].plot(res_old['t'], res_old['a'], 'b-', lw=1.5, label='old')
    ax_row[2].plot(res_cur['t'], res_cur['a'], 'r-', lw=1.5, label='current')
    ax_row[2].set_ylabel('a')
    ax_row[2].legend()
    ax_row[2].axhline(0, color='k', ls='--', alpha=0.3)

plot_row(axes[0], s1_old, s1_cur, 'Scene 1: Late interruption (v=0.8 → target=0.06)')
plot_row(axes[1], s2_old, s2_cur, 'Scene 2: Idle + micro-adjust x2')
plot_row(axes[2], s3_old, s3_cur, 'Scene 3: 3 interrupts in 100ms (v=1.0)')

for ax in axes[-1]:
    ax.set_xlabel('Time (s)')

plt.tight_layout()
plt.savefig('compare_v4_final.png', dpi=150)
print('\nSaved compare_v4_final.png')

# 场景1 关键帧打印：验证制动安全
print("\n【场景1 关键帧】init 后前 50ms")
print(f"{'t':>6s} | {'old_s':>7s} | {'old_v':>7s} | {'old_a':>7s} | {'old_st':>6s} | {'cur_s':>7s} | {'cur_v':>7s} | {'cur_a':>7s} | {'cur_st':>6s} | {'note':>20s}")
print("-" * 110)
for i in range(min(55, len(s1_old['t']))):
    note = ''
    if s1_cur['state'][i] == 5 and (i == 0 or s1_cur['state'][i-1] != 5):
        note = 'cur: jump to p3_end'
    elif s1_cur['state'][i] == 7 and (i == 0 or s1_cur['state'][i-1] != 7):
        note = 'cur: enter p5'
    print(f"{s1_old['t'][i]:6.3f} | {s1_old['s'][i]:7.4f} | {s1_old['v'][i]:7.4f} | {s1_old['a'][i]:7.4f} | {s1_old['state'][i]:6d} | "
          f"{s1_cur['s'][i]:7.4f} | {s1_cur['v'][i]:7.4f} | {s1_cur['a'][i]:7.4f} | {s1_cur['state'][i]:6d} | {note:>20s}")
