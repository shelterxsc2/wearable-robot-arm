"""
STM32 下位机 S-curve 仿真器
================================
复现 wearable-robot-arm 下位机 Control_Algorithm.c 中的 Speed_Plan_Update 逻辑。
支持三个版本的参数/行为切换：
  - version='old'    : 96169fd / 22668cc（旧版，无条件 phase1，无 runtime safe）
  - version='mid'    : 次新版（本地未上传，推测：强制制动检查，无分层决策）
  - version='current': 8efeb36（当前版，cmd_type 分支 + v9 安全逻辑 + runtime safe）

重要：cmd.txt 中的 VOFA 数据来自旧版/次新版下位机代码，非当前 8efeb36。
"""
import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np


@dataclass
class SpeedPlanHandle:
    """完全对应下位机 Speed_Plan_Handle_t 结构"""
    # 基础参数
    j: float = 80.0           # 基础 jerk limit
    a_max: float = 2.2        # 基础 accel max
    v_max: float = 3.0        # 基础 velocity max
    
    # 运行时限制
    j_limit: float = 80.0
    a_limit: float = 2.2
    v_limit: float = 3.0
    
    # 状态变量
    a: float = 0.0
    v: float = 0.0
    s: float = 0.0            # 位移积分（相对 position_initial）
    error_s: float = 0.0      # 目标 - 实际
    position_initial: float = 0.0
    direction_flag: float = 1.0
    cmd_type: int = 0x01      # 0x00=预测, 0x01=确认
    state: int = 0            # 0=idle, 1=init, 2=p1, 3=p2, 4=p3, 5=p3e, 6=p4, 7=p5, 8=p6, 9=p7
    time_stamp: float = 0.0
    
    # 版本相关配置（可在初始化时覆盖）
    deadband: float = 0.05
    scale_short: List[float] = field(default_factory=lambda: [0.30, 0.40, 0.60])
    dist_thresholds: List[float] = field(default_factory=lambda: [0.08, 0.12, 0.30])
    j_limits_short: List[float] = field(default_factory=lambda: [12.0, 16.0, 20.0])
    brake_floor: float = 0.10
    clamp_margin: float = 1.2
    smart_jump_threshold: float = 0.6
    offset_non_gimbal: float = 0.03
    is_gimbal: bool = True
    
    # 版本开关
    version: str = 'current'  # 'old' | 'mid' | 'current'
    
    # 次新版推测参数（仅在 version='mid' 时生效）
    mid_hard_brake: bool = True      # 次新版：强制制动检查
    mid_brake_rate: float = 0.85     # 次新版：每次降速比例
    mid_floor: float = 0.05          # 次新版：最低保留速度


def calc_v1(a_limit: float, j_limit: float) -> float:
    """对应下位机 Calc_V1：匀加速段峰值速度"""
    return a_limit * a_limit / j_limit


def calc_decel_dist(v: float, a_max: float, j: float) -> float:
    """对应下位机 Calc_Decel_Dist：S-curve 减速距离"""
    return (v * v) / (2.0 * a_max) + (v * a_max) / (2.0 * j)


def speed_plan_update(handle: SpeedPlanHandle,
                      position_actual: float,
                      position_target: float,
                      t_now: float,
                      dt: float) -> None:
    """
    复现下位机 Speed_Plan_Update 主状态机。
    根据 handle.version 自动切换旧版/次新版/当前版行为。
    """
    v1 = calc_v1(handle.a_limit, handle.j_limit)
    
    # ---------- idle ----------
    if handle.state == 0:
        handle.a = 0.0
        handle.v = 0.0
        handle.s = 0.0
        return
    
    # ---------- init ----------
    if handle.state == 1:
        old_direction = handle.direction_flag
        handle.error_s = position_target - position_actual
        
        # position_initial 偏移（仅非云台）
        if not handle.is_gimbal:
            if position_actual > 0.0:
                handle.position_initial = position_actual + handle.offset_non_gimbal
            elif position_actual < 0.0:
                handle.position_initial = position_actual - handle.offset_non_gimbal
            else:
                handle.position_initial = position_actual
        else:
            handle.position_initial = position_actual
        
        # 死区
        if abs(handle.error_s) <= handle.deadband:
            handle.a = 0.0
            handle.v = 0.0
            handle.s = 0.0
            handle.state = 0
            return
        
        handle.direction_flag = 1.0 if handle.error_s >= 0 else -1.0
        
        # 方向反转衰减
        if old_direction * handle.direction_flag < 0.0:
            handle.v *= 0.3
        
        # 自适应 v_limit + 动态 j_limit
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
        if handle.v_limit < 0.0:
            handle.v_limit = 0.0
        
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
        
        # ==================== 版本分支 ====================
        if handle.version == 'old':
            # 旧版：无条件 phase1，无任何安全保护
            handle.a = 0.0
            handle.s = 0.0
            handle.state = 2
            return
        
        elif handle.version == 'mid':
            # 次新版（推测）：强制制动检查，无条件降速，无分层决策
            v_abs = abs(handle.v)
            decel_needed = calc_decel_dist(v_abs, handle.a_limit, handle.j_limit)
            
            if handle.mid_hard_brake and decel_needed >= abs(handle.error_s):
                # 强制降到能停下（或直接降到 floor）
                v_safe = v_abs
                while decel_needed >= abs(handle.error_s) and v_safe > handle.mid_floor:
                    v_safe *= handle.mid_brake_rate
                    decel_needed = calc_decel_dist(v_safe, handle.a_limit, handle.j_limit)
                handle.v = (1.0 if handle.v >= 0 else -1.0) * v_safe
                handle.a = 0.0
                handle.s = 0.0
                handle.state = 5  # phase3_end
                return
            
            # 否则正常 phase1
            handle.a = 0.0
            handle.s = 0.0
            handle.state = 2
            return
        
        else:  # 'current'
            # 当前版 8efeb36：三层分层决策
            v_abs = abs(handle.v)
            
            # 0x00 预测指令：直接 phase1
            if handle.cmd_type == 0x00:
                handle.a = 0.0
                handle.s = 0.0
                handle.state = 2
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
                handle.state = 5
                return
            
            # 3. 智能相位跳转
            if v_abs > handle.v_limit * handle.smart_jump_threshold:
                handle.a = 0.0
                handle.s = 0.0
                handle.state = 5
                return
            
            # 默认 phase1
            handle.a = 0.0
            handle.s = 0.0
            handle.state = 2
            return
    
    # ---------- phase1 ----------
    if handle.state == 2:
        handle.a += handle.j_limit * dt
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        if handle.a >= handle.a_limit:
            handle.a = handle.a_limit
            handle.state = 3
        if handle.version != 'old' and handle.v >= handle.v_limit:
            handle.state = 4
    
    # ---------- phase2 ----------
    elif handle.state == 3:
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        if handle.v >= handle.v_limit - v1:
            handle.state = 4
        if handle.version != 'old' and handle.v >= handle.v_limit:
            handle.state = 4
    
    # ---------- phase3 ----------
    elif handle.state == 4:
        handle.a -= handle.j_limit * dt
        handle.v += handle.a * dt
        if handle.version != 'old' and handle.v > handle.v_limit:
            handle.v = handle.v_limit
        handle.s += handle.v * dt
        if handle.a <= 0:
            handle.a = 0.0
            if handle.v > handle.v_limit:
                handle.v = handle.v_limit
            handle.state = 5
    
    # ---------- phase3_end ----------
    elif handle.state == 5:
        decel_dist = calc_decel_dist(handle.v, handle.a_limit, handle.j_limit)
        if handle.s >= abs(handle.error_s) - decel_dist:
            handle.state = 7
        else:
            handle.state = 6
    
    # ---------- phase4 ----------
    elif handle.state == 6:
        handle.s += handle.v * dt
        decel_dist = calc_decel_dist(handle.v, handle.a_limit, handle.j_limit)
        if handle.s >= abs(handle.error_s) - decel_dist:
            handle.state = 7
    
    # ---------- phase5 ----------
    elif handle.state == 7:
        handle.a -= handle.j * dt
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        
        # RUNTIME SAFE（仅 current 版）
        if handle.version == 'current':
            remaining = abs(handle.error_s) - handle.s
            if remaining > 0.001 and handle.v > 0.1:
                min_a_needed = -(handle.v * handle.v) / (2.0 * remaining)
                if handle.a > min_a_needed:
                    handle.a = min_a_needed
        
        if handle.a <= -handle.a_limit:
            handle.a = -handle.a_limit
            handle.state = 8
    
    # ---------- phase6 ----------
    elif handle.state == 8:
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        if handle.v <= v1:
            handle.state = 9
    
    # ---------- phase7 ----------
    elif handle.state == 9:
        handle.a += handle.j_limit * dt
        handle.v += handle.a * dt
        handle.s += handle.v * dt
        if handle.a >= 0 or handle.v <= 0:
            handle.a = 0.0
            handle.v = 0.0
            handle.state = 0
    
    # ---------- 全局 idle 检测 ----------
    if abs(handle.s - abs(handle.error_s)) <= 0.003 or handle.v < 0 or handle.s - abs(handle.error_s) >= 0:
        handle.v = 0.0
        handle.a = 0.0
        handle.s = 0.0
        handle.state = 0


def simulate_events(events: List[Tuple[float, float, int]],
                    version: str = 'current',
                    dt: float = 0.001,
                    handle_params: Optional[dict] = None) -> dict:
    """
    用事件驱动方式运行仿真。
    events: List[(time_sec, position_target, cmd_type)]
    返回包含完整轨迹的字典。
    """
    handle = SpeedPlanHandle(version=version)
    if handle_params:
        for k, v in handle_params.items():
            setattr(handle, k, v)
    
    handle.state = 0
    handle.time_stamp = 0.0
    
    events = sorted(events, key=lambda x: x[0])
    event_idx = 0
    
    times, targets, states = [], [], []
    plan_s, plan_v, plan_a = [], [], []
    position_actual = 0.0
    
    t = 0.0
    current_target = 0.0
    current_cmd_type = 0x01
    
    while t < (events[-1][0] + 2.0 if events else 2.0):
        while event_idx < len(events) and events[event_idx][0] <= t:
            current_target = events[event_idx][1]
            current_cmd_type = events[event_idx][2]
            handle.cmd_type = current_cmd_type
            handle.state = 1
            event_idx += 1
        
        if handle.state != 0:
            speed_plan_update(handle, position_actual, current_target, t, dt)
        
        # 简化的位置跟踪（电机实际位置缓慢跟随规划）
        plan_pos = handle.position_initial + handle.direction_flag * handle.s
        position_actual += (plan_pos - position_actual) * 0.05
        
        times.append(t)
        targets.append(current_target)
        plan_s.append(handle.s)
        plan_v.append(handle.v)
        plan_a.append(handle.a)
        states.append(handle.state)
        
        t += dt
    
    return {
        't': np.array(times),
        'target': np.array(targets),
        's': np.array(plan_s),
        'v': np.array(plan_v),
        'a': np.array(plan_a),
        'state': np.array(states),
    }


if __name__ == '__main__':
    # 快速测试：对比三个版本处理同一打断事件
    events = [
        (0.0, 1.50, 0x01),   # 长距离
        (3.0, 0.05, 0x01),   # 运动中突然目标变近
    ]
    
    for ver in ['old', 'mid', 'current']:
        r = simulate_events(events, version=ver, dt=0.001)
        # 找打断时刻后的最大加速度
        mask = r['t'] >= 3.0
        max_a = np.abs(r['a'][mask]).max() if np.any(mask) else 0
        print(f'{ver:8s}: max_a_after_break={max_a:.2f}')
