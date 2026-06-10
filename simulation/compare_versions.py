"""
旧版 vs 最新版 下位机 S-curve 对比验证
======================================
复现用户描述的三大典型场景，验证当前版(8efeb36)是否显著优于旧版(22668cc/96169fd)。

注意：此仿真器复现的是下位机 Speed_Plan_Update 的规划值输出，
      与之前六个分析脚本的后验分析互为补充。
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from stm32_scurve_sim import SpeedPlanHandle, speed_plan_update, calc_decel_dist


def run_single_scenario(events, version, dt=0.001, pre_run_target=None, pre_run_time=0.0):
    """
    运行一个完整场景，支持预运行（让电机先跑起来，再注入打断事件）
    events: [(time_sec, target, cmd_type), ...]
    pre_run_target: 预运行阶段的目标位置（让电机进入高速状态）
    pre_run_time: 预运行时长
    """
    handle = SpeedPlanHandle(version=version)
    
    # 旧版参数差异
    if version == 'old':
        handle.scale_short = [0.15, 0.20, 0.30]
        handle.j_limits_short = [6.0, 8.0, 10.0]
        handle.deadband = 0.025
    
    handle.state = 0
    position_actual = 0.0
    
    times, targets, plan_s, plan_v, plan_a, plan_state = [], [], [], [], [], []
    
    t = 0.0
    current_target = 0.0
    current_cmd_type = 0x01
    event_idx = 0
    events = sorted(events, key=lambda x: x[0])
    
    # 预运行阶段
    if pre_run_target is not None and pre_run_time > 0:
        handle.state = 1
        current_target = pre_run_target
        while t < pre_run_time:
            if handle.state != 0:
                speed_plan_update(handle, position_actual, current_target, t, dt)
            plan_pos = handle.position_initial + handle.direction_flag * handle.s
            position_actual += (plan_pos - position_actual) * 0.05
            t += dt
    
    # 主运行阶段
    while t < (events[-1][0] + 2.0 if events else 2.0):
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
        'pos_actual': position_actual,
    }


def compute_metrics(res):
    """计算仿真指标，与 analyze_cmd.py / analyze_overrun.py 逻辑对应"""
    a = res['a']
    v = res['v']
    s = res['s']
    state = res['state']
    
    # 1. 加速度跳变 (>2.0)
    a_jumps = np.sum(np.abs(np.diff(a)) > 2.0)
    
    # 2. 状态切换次数
    state_switches = np.sum(np.diff(state) != 0)
    
    # 3. s 重置次数（同 analyze_cmd.py）
    resets = 0
    for i in range(1, len(s)):
        if s[i-1] > 0.01 and s[i] < 0.001:
            resets += 1
    
    # 4. 到位残余速度（idle 前的 v）
    idle_entries = np.where(np.diff(state) < 0)[0]
    residual_v = np.mean(np.abs(v[idle_entries])) if len(idle_entries) > 0 else 0
    
    # 5. 最大加速度
    max_a = np.max(np.abs(a))
    
    # 6. 模拟 analyze_overrun: insufficient braking distance
    # 找 state != 0 且 remaining 很小的帧
    overrun_count = 0
    for i in range(len(s)):
        if state[i] == 0:
            continue
        # 估算 error_s（简化：用 target - position_initial）
        # 这里直接用 s 和 v 估算
        if v[i] > 0.3 and s[i] > 0.001:
            # 假设当前 max_s ≈ s[i] + 剩余距离，简化用 s[i] 代替
            remaining = 0.1  # 假设剩余 0.1
            decel_needed = (v[i] ** 2) / (2 * 2.2)
            if remaining < decel_needed:
                overrun_count += 1
    
    return {
        'a_jumps': int(a_jumps),
        'state_switches': int(state_switches),
        'resets': int(resets),
        'residual_v': float(residual_v),
        'max_a': float(max_a),
        'overrun_count': int(overrun_count),
    }


# ============ 场景设计 ============

# 场景1：长距离运动末期被打断（用户描述的"急停晃动"根因）
# 先预跑 2.5s 让电机接近高速，然后突然把目标改到很近的位置
scenario1_events = [(2.5, 0.08, 0x01)]  # 新目标很近

# 场景2：idle 后微调 + 运动中再次被打断（用户描述的"微调抖动"）
scenario2_events = [
    (0.0, 1.50, 0x01),
    (6.0, 1.52, 0x01),   # idle 后微调
    (6.1, 1.51, 0x01),   # 运动中又改
]

# 场景3：高频追踪打断（测试鲁棒性）
scenario3_events = [
    (0.0, 1.20, 0x01),
    (0.5, 1.15, 0x00),
    (0.7, 1.10, 0x00),
    (0.9, 1.05, 0x00),
    (1.1, 1.00, 0x01),
    (1.3, 0.95, 0x00),
    (1.5, 0.90, 0x00),
]


print("=" * 80)
print("旧版(old) vs 最新版(current) S-curve 对比验证")
print("=" * 80)

# 场景1：末期打断
print("\n【场景1】长距离运动末期被打断：预跑 target=1.5，2.5s后突然 target=0.08")
res_old_s1 = run_single_scenario(scenario1_events, 'old', pre_run_target=1.5, pre_run_time=2.5)
res_cur_s1 = run_single_scenario(scenario1_events, 'current', pre_run_target=1.5, pre_run_time=2.5)

m_old = compute_metrics(res_old_s1)
m_cur = compute_metrics(res_cur_s1)

print(f"  指标              | 旧版(old) | 最新版(current) | 改善")
print(f"  {'-'*60}")
print(f"  加速度跳变(>2.0)  | {m_old['a_jumps']:8d}  | {m_cur['a_jumps']:10d}    | {'✓' if m_cur['a_jumps'] < m_old['a_jumps'] else '✗'}")
print(f"  状态切换次数      | {m_old['state_switches']:8d}  | {m_cur['state_switches']:10d}    | {'✓' if m_cur['state_switches'] < m_old['state_switches'] else '✗'}")
print(f"  s 重置次数        | {m_old['resets']:8d}  | {m_cur['resets']:10d}    | {'✓' if m_cur['resets'] < m_old['resets'] else '✗'}")
print(f"  到位残余速度      | {m_old['residual_v']:8.4f}  | {m_cur['residual_v']:10.4f}    | {'✓' if m_cur['residual_v'] < m_old['residual_v'] else '✗'}")
print(f"  最大加速度        | {m_old['max_a']:8.2f}  | {m_cur['max_a']:10.2f}    | {'✓' if m_cur['max_a'] <= m_old['max_a'] else '✗'}")

# 场景2：idle 后微调
print("\n【场景2】idle 后微调 + 运动中再次打断")
res_old_s2 = run_single_scenario(scenario2_events, 'old')
res_cur_s2 = run_single_scenario(scenario2_events, 'current')

m_old2 = compute_metrics(res_old_s2)
m_cur2 = compute_metrics(res_cur_s2)

print(f"  指标              | 旧版(old) | 最新版(current) | 改善")
print(f"  {'-'*60}")
print(f"  加速度跳变(>2.0)  | {m_old2['a_jumps']:8d}  | {m_cur2['a_jumps']:10d}    | {'✓' if m_cur2['a_jumps'] < m_old2['a_jumps'] else '✗'}")
print(f"  状态切换次数      | {m_old2['state_switches']:8d}  | {m_cur2['state_switches']:10d}    | {'✓' if m_cur2['state_switches'] < m_old2['state_switches'] else '✗'}")
print(f"  s 重置次数        | {m_old2['resets']:8d}  | {m_cur2['resets']:10d}    | {'✓' if m_cur2['resets'] < m_old2['resets'] else '✗'}")
print(f"  到位残余速度      | {m_old2['residual_v']:8.4f}  | {m_cur2['residual_v']:10.4f}    | {'✓' if m_cur2['residual_v'] < m_old2['residual_v'] else '✗'}")

# 场景3：高频追踪
print("\n【场景3】高频追踪打断（7个指令/1.5s）")
res_old_s3 = run_single_scenario(scenario3_events, 'old')
res_cur_s3 = run_single_scenario(scenario3_events, 'current')

m_old3 = compute_metrics(res_old_s3)
m_cur3 = compute_metrics(res_cur_s3)

print(f"  指标              | 旧版(old) | 最新版(current) | 改善")
print(f"  {'-'*60}")
print(f"  加速度跳变(>2.0)  | {m_old3['a_jumps']:8d}  | {m_cur3['a_jumps']:10d}    | {'✓' if m_cur3['a_jumps'] < m_old3['a_jumps'] else '✗'}")
print(f"  状态切换次数      | {m_old3['state_switches']:8d}  | {m_cur3['state_switches']:10d}    | {'✓' if m_cur3['state_switches'] < m_old3['state_switches'] else '✗'}")
print(f"  s 重置次数        | {m_old3['resets']:8d}  | {m_cur3['resets']:10d}    | {'✓' if m_cur3['resets'] < m_old3['resets'] else '✗'}")
print(f"  到位残余速度      | {m_old3['residual_v']:8.4f}  | {m_cur3['residual_v']:10.4f}    | {'✓' if m_cur3['residual_v'] < m_old3['residual_v'] else '✗'}")


# ============ 可视化 ============
fig, axes = plt.subplots(3, 3, figsize=(16, 12))

def plot_scenario(ax_row, res_old, res_cur, title):
    t_mask_old = res_old['t'] >= 0
    t_mask_cur = res_cur['t'] >= 0
    
    # s
    ax_row[0].plot(res_old['t'][t_mask_old], res_old['s'][t_mask_old], 'b-', lw=1, label='old')
    ax_row[0].plot(res_cur['t'][t_mask_cur], res_cur['s'][t_mask_cur], 'r-', lw=1, label='current')
    ax_row[0].set_ylabel('s (rad)')
    ax_row[0].set_title(title)
    ax_row[0].legend()
    
    # v
    ax_row[1].plot(res_old['t'][t_mask_old], res_old['v'][t_mask_old], 'b-', lw=1, label='old')
    ax_row[1].plot(res_cur['t'][t_mask_cur], res_cur['v'][t_mask_cur], 'r-', lw=1, label='current')
    ax_row[1].set_ylabel('v (rad/s)')
    ax_row[1].legend()
    
    # a
    ax_row[2].plot(res_old['t'][t_mask_old], res_old['a'][t_mask_old], 'b-', lw=1, label='old')
    ax_row[2].plot(res_cur['t'][t_mask_cur], res_cur['a'][t_mask_cur], 'r-', lw=1, label='current')
    ax_row[2].set_ylabel('a (rad/s²)')
    ax_row[2].legend()

plot_scenario(axes[0], res_old_s1, res_cur_s1, 'Scene 1: Late interruption')
plot_scenario(axes[1], res_old_s2, res_cur_s2, 'Scene 2: Idle + micro-adjust')
plot_scenario(axes[2], res_old_s3, res_cur_s3, 'Scene 3: High-frequency tracking')

for ax in axes[-1]:
    ax.set_xlabel('Time (s)')

plt.tight_layout()
plt.savefig('compare_old_vs_current.png', dpi=150)
print('\nSaved compare_old_vs_current.png')
