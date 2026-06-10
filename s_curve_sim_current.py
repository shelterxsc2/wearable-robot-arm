"""
S-curve 仿真：复现 8efeb36 版本下位机逻辑
支持外部输入 target 序列，模拟打断、idle、PID 跟踪
"""
import math
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Tuple

@dataclass
class SpeedPlanHandle:
    # 参数（固定）
    j: float = 80.0           # 基础 jerk
    a_max: float = 2.2        # 基础 max accel
    v_max: float = 3.0        # 基础 max velocity
    
    # 状态
    j_limit: float = 80.0
    a_limit: float = 2.2
    v_limit: float = 3.0
    a: float = 0.0
    v: float = 0.0
    s: float = 0.0            # 位移积分
    error_s: float = 0.0
    position_initial: float = 0.0
    direction_flag: float = 1.0
    cmd_type: int = 0x01
    state: int = 0            # 0=idle, 1=init, 2=phase1, 3=phase2, 4=phase3, 5=phase3_end, 6=phase4, 7=phase5, 8=phase6, 9=phase7
    time_stamp: float = 0.0
    
    # 配置
    deadband: float = 0.05    # idle 死区
    scale_short: List[float] = field(default_factory=lambda: [0.30, 0.40, 0.60])
    dist_thresholds: List[float] = field(default_factory=lambda: [0.08, 0.12, 0.30])
    j_limits_short: List[float] = field(default_factory=lambda: [12.0, 16.0, 20.0])
    brake_floor: float = 0.10
    clamp_margin: float = 1.2
    smart_jump_threshold: float = 0.6
    offset_non_gimbal: float = 0.03
    is_gimbal: bool = True    # 云台电机

def calc_v1(a_limit, j_limit):
    return a_limit * a_limit / j_limit

def calc_decel_dist(v, a_max, j):
    return (v * v) / (2.0 * a_max) + (v * a_max) / (2.0 * j)

def speed_plan_update(handle: SpeedPlanHandle, position_actual: float, position_target: float, t_now: float, dt: float):
    """复现 8efeb36 版本 Speed_Plan_Update 核心逻辑"""
    v1 = calc_v1(handle.a_limit, handle.j_limit)
    
    if handle.state == 0:  # idle
        handle.a = 0
        handle.v = 0
        handle.s = 0
        return
    
    elif handle.state == 1:  # init
        old_direction = handle.direction_flag
        handle.error_s = position_target - position_actual
        
        if not handle.is_gimbal:
            if position_actual > 0:
                handle.position_initial = position_actual + handle.offset_non_gimbal
            elif position_actual < 0:
                handle.position_initial = position_actual - handle.offset_non_gimbal
            else:
                handle.position_initial = position_actual
        else:
            handle.position_initial = position_actual
        
        # 死区
        if abs(handle.error_s) <= handle.deadband:
            handle.a = 0
            handle.v = 0
            handle.s = 0
            handle.state = 0
            return
        
        handle.direction_flag = 1.0 if handle.error_s >= 0 else -1.0
        
        # 方向反转衰减
        if old_direction * handle.direction_flag < 0:
            handle.v *= 0.3
        
        # 自适应 v_limit
        S = abs(handle.error_s)
        v1_base = calc_v1(handle.a_max, handle.j)
        b = (handle.a_max * handle.a_max) / handle.j
        discriminant = b * b + 4.0 * S * handle.a_max
        v_peak = (-b + math.sqrt(discriminant)) / 2.0
        
        if v_peak < 2.0 * v1_base:
            v_peak = (0.5 * handle.j * S * S) ** (1.0 / 3.0)
        
        scale = 1.0
        if S < handle.dist_thresholds[0]:
            scale = handle.scale_short[0]
        elif S < handle.dist_thresholds[1]:
            scale = handle.scale_short[1]
        elif S < handle.dist_thresholds[2]:
            scale = handle.scale_short[2]
        v_peak *= scale
        
        handle.v_limit = min(v_peak, handle.v_max)
        if handle.v_limit < 0:
            handle.v_limit = 0
        
        # 动态 j_limit
        if S < handle.dist_thresholds[0]:
            handle.j_limit = handle.j_limits_short[0]
        elif S < handle.dist_thresholds[1]:
            handle.j_limit = handle.j_limits_short[1]
        elif S < handle.dist_thresholds[2]:
            handle.j_limit = handle.j_limits_short[2]
        else:
            handle.j_limit = handle.j
        
        # 有效 a_limit
        v1_limit = calc_v1(handle.a_max, handle.j_limit)
        if handle.v_limit < 2.0 * v1_limit:
            handle.a_limit = math.sqrt(handle.j_limit * handle.v_limit)
        else:
            handle.a_limit = handle.a_max
        
        # ========== v9 安全逻辑 ==========
        v_abs = abs(handle.v)
        
        # 预测指令
        if handle.cmd_type == 0x00:
            handle.a = 0.0
            handle.s = 0.0
            handle.state = 2  # phase1
            return
        
        # 1. 速度钳制（120% margin）
        if v_abs > handle.v_limit * handle.clamp_margin:
            v_clamp = handle.v_limit * handle.clamp_margin
            if v_clamp > handle.v_max:
                v_clamp = handle.v_max
            handle.v = (1.0 if handle.v >= 0 else -1.0) * v_clamp
            v_abs = v_clamp
        
        # 2. 制动距离安全
        decel_needed = calc_decel_dist(v_abs, handle.a_limit, handle.j_limit)
        if decel_needed >= abs(handle.error_s):
            v_safe = v_abs
            while decel_needed >= abs(handle.error_s) and v_safe > handle.brake_floor:
                v_safe *= 0.92
                decel_needed = calc_decel_dist(v_safe, handle.a_limit, handle.j_limit)
            handle.v = (1.0 if handle.v >= 0 else -1.0) * v_safe
            handle.a = 0.0
            handle.s = 0.0
            handle.state = 5  # phase3_end
            return
        
        # 3. 智能相位跳转
        if v_abs > handle.v_limit * handle.smart_jump_threshold:
            handle.a = 0.0
            handle.s = 0.0
            handle.state = 5  # phase3_end
            return
        
        # 默认：正常 phase1
        handle.a = 0
        handle.s = 0
        handle.state = 2
        return
    
    elif handle.state == 2:  # phase1
        handle.a += handle.j_limit * dt
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        if handle.a >= handle.a_limit:
            handle.a = handle.a_limit
            handle.state = 3  # phase2
        if handle.v >= handle.v_limit:
            handle.state = 4  # phase3
    
    elif handle.state == 3:  # phase2
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        if handle.v >= handle.v_limit - v1:
            handle.state = 4  # phase3
        if handle.v >= handle.v_limit:
            handle.state = 4
    
    elif handle.state == 4:  # phase3
        handle.a -= handle.j_limit * dt
        handle.v += handle.a * dt
        if handle.v > handle.v_limit:
            handle.v = handle.v_limit
        handle.s += handle.v * dt
        if handle.a <= 0:
            handle.a = 0
            if handle.v > handle.v_limit:
                handle.v = handle.v_limit
            handle.state = 5  # phase3_end
    
    elif handle.state == 5:  # phase3_end
        decel_dist = calc_decel_dist(handle.v, handle.a_limit, handle.j_limit)
        if handle.s >= abs(handle.error_s) - decel_dist:
            handle.state = 7  # phase5
        else:
            handle.state = 6  # phase4
    
    elif handle.state == 6:  # phase4
        handle.s += handle.v * dt
        decel_dist = calc_decel_dist(handle.v, handle.a_limit, handle.j_limit)
        if handle.s >= abs(handle.error_s) - decel_dist:
            handle.state = 7  # phase5
    
    elif handle.state == 7:  # phase5
        handle.a -= handle.j * dt  # 注意：phase5 用 j 而不是 j_limit！
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        
        # RUNTIME SAFE
        remaining = abs(handle.error_s) - handle.s
        if remaining > 0.001 and handle.v > 0.1:
            min_a_needed = -(handle.v * handle.v) / (2.0 * remaining)
            if handle.a > min_a_needed:
                handle.a = min_a_needed
        
        if handle.a <= -handle.a_limit:
            handle.a = -handle.a_limit
            handle.state = 8  # phase6
    
    elif handle.state == 8:  # phase6
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        if handle.v <= v1:
            handle.state = 9  # phase7
    
    elif handle.state == 9:  # phase7
        handle.a += handle.j_limit * dt
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        if handle.a >= 0 or handle.v <= 0:
            handle.a = 0
            handle.v = 0
            handle.state = 0  # idle
    
    # 全局 idle 检测
    if abs(handle.s - abs(handle.error_s)) <= 0.003 or handle.v < 0 or handle.s - abs(handle.error_s) >= 0:
        handle.v = 0
        handle.a = 0
        handle.s = 0
        handle.state = 0

# ========== PID 跟踪模型 ==========
class PIDTracker:
    def __init__(self, Kp=80.0, Kd=2.0, Ki=0.0):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.integral = 0.0
        self.pos = 0.0
        self.vel = 0.0
        self.prev_err = 0.0
    
    def update(self, target_pos: float, dt: float):
        err = target_pos - self.pos
        self.integral += err * dt
        derr = (err - self.prev_err) / dt
        cmd = self.Kp * err + self.Ki * self.integral + self.Kd * derr
        # 简单电机模型：cmd → accel
        self.vel += cmd * dt * 0.01  # 增益缩放
        self.pos += self.vel * dt
        self.prev_err = err
        return self.pos, self.vel

# ========== 仿真主循环 ==========
def simulate(target_events: List[Tuple[float, float, int]], 
             dt: float = 0.001,
             pid_Kp: float = 80.0,
             pid_Kd: float = 2.0,
             handle_params: dict = None) -> dict:
    """
    target_events: List of (time_sec, position_target, cmd_type)
    返回仿真轨迹
    """
    handle = SpeedPlanHandle()
    if handle_params:
        for k, v in handle_params.items():
            setattr(handle, k, v)
    
    pid = PIDTracker(Kp=pid_Kp, Kd=pid_Kd)
    
    # 初始化
    handle.state = 0
    handle.time_stamp = 0.0
    
    # 排序事件
    events = sorted(target_events, key=lambda x: x[0])
    event_idx = 0
    
    # 记录
    times = []
    targets = []
    plan_s = []
    plan_v = []
    plan_a = []
    plan_state = []
    actual_pos = []
    actual_vel = []
    
    t = 0.0
    current_target = 0.0
    current_cmd_type = 0x01
    position_actual = 0.0
    
    while t < events[-1][0] + 3.0:
        # 检查新指令
        while event_idx < len(events) and events[event_idx][0] <= t:
            current_target = events[event_idx][1]
            current_cmd_type = events[event_idx][2]
            handle.cmd_type = current_cmd_type
            handle.state = 1  # init
            event_idx += 1
        
        # 更新 S-curve
        if handle.state != 0:
            speed_plan_update(handle, position_actual, current_target, t, dt)
        
        # PID 跟踪规划位置
        plan_position = handle.position_initial + handle.direction_flag * handle.s
        actual_pos_now, actual_vel_now = pid.update(plan_position, dt)
        position_actual = actual_pos_now
        
        # 记录
        times.append(t)
        targets.append(current_target)
        plan_s.append(handle.s)
        plan_v.append(handle.v)
        plan_a.append(handle.a)
        plan_state.append(handle.state)
        actual_pos.append(actual_pos_now)
        actual_vel.append(actual_vel_now)
        
        t += dt
    
    return {
        't': np.array(times),
        'target': np.array(targets),
        'plan_s': np.array(plan_s),
        'plan_v': np.array(plan_v),
        'plan_a': np.array(plan_a),
        'plan_state': np.array(plan_state),
        'actual_pos': np.array(actual_pos),
        'actual_vel': np.array(actual_vel),
    }

if __name__ == '__main__':
    # 测试：长距离运动 + idle + 微调
    events = [
        (0.0, 1.5, 0x01),    # 长距离运动
        (6.0, 1.52, 0x01),   # 长距离后的微调
    ]
    result = simulate(events, dt=0.001)
    
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    
    ax = axes[0]
    ax.plot(result['t'], result['target'], 'r--', label='target')
    ax.plot(result['t'], result['actual_pos'], 'b-', label='actual')
    ax.set_ylabel('Position (rad)')
    ax.legend()
    ax.set_title('Long move + micro-adjust')
    
    ax = axes[1]
    ax.plot(result['t'], result['plan_v'], 'g-', label='plan_v')
    ax.plot(result['t'], result['actual_vel'], 'm-', label='actual_v')
    ax.set_ylabel('Velocity')
    ax.legend()
    
    ax = axes[2]
    ax.plot(result['t'], result['plan_a'], 'k-', label='plan_a')
    ax.set_ylabel('Acceleration')
    ax.legend()
    
    ax = axes[3]
    ax.plot(result['t'], result['plan_state'], 'c-', label='state')
    ax.set_ylabel('State')
    ax.set_xlabel('Time (s)')
    ax.legend()
    
    plt.tight_layout()
    plt.savefig('sim_long_micro.png', dpi=150)
    print('Saved sim_long_micro.png')
