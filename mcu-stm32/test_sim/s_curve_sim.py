#!/usr/bin/env python3
"""
S-Curve Speed Profile Simulator
Compare: fixed v_max vs adaptive v_limit
ASCII visualization
"""

import math

# Current MCU parameters
J = 15.5          # jerk rad/s^3
A_MAX = 1.5       # max accel rad/s^2
V_MAX = 0.45      # max speed rad/s
DT = 0.001        # simulation step 1ms

def calc_v1(a_max, j):
    return (a_max * a_max) / (2.0 * j)

def calc_decel_dist(v, a_max, j):
    return (v * v) / (2.0 * a_max) + (v * a_max) / (2.0 * j)

class SpeedPlanSimulator:
    """Replica of current STM32 state machine"""
    def __init__(self, j, a_max, v_max, use_v_limit=False):
        self.j = j
        self.a_max = a_max
        self.v_max = v_max
        self.use_v_limit = use_v_limit
        
        self.v_limit = v_max
        self.a = 0.0
        self.v = 0.0
        self.s = 0.0
        self.error_s = 0.0
        self.direction_flag = 1.0
        self.state = 'idle'
        
        self.history = {'t': [], 's': [], 'v': [], 'a': [], 'state': []}
        self.t = 0.0
    
    def reset(self):
        self.a = 0.0
        self.v = 0.0
        self.s = 0.0
        self.state = 'idle'
        self.t = 0.0
        self.history = {'t': [], 's': [], 'v': [], 'a': [], 'state': []}
    
    def init(self, target_disp):
        self.reset()
        self.error_s = target_disp
        self.direction_flag = 1.0 if target_disp >= 0 else -1.0
        
        if self.use_v_limit:
            S = abs(target_disp)
            v1 = calc_v1(self.a_max, self.j)
            
            a = 1.0
            b = (self.a_max ** 2) / self.j
            c = -S * self.a_max
            disc = b * b - 4.0 * a * c
            v_peak = (-b + math.sqrt(disc)) / (2.0 * a)
            
            if v_peak < 2.0 * v1:
                v_peak = (0.5 * self.j * S * S) ** (1.0 / 3.0)
            
            self.v_limit = min(v_peak, self.v_max)
            if self.v_limit < 0:
                self.v_limit = 0
        else:
            self.v_limit = self.v_max
        
        self.state = 'phase1'
    
    def step(self):
        v1 = calc_v1(self.a_max, self.j)
        
        if self.state == 'idle':
            pass
        elif self.state == 'phase1':
            self.a += self.j * DT
            self.v += self.a * DT
            self.s += self.v * DT
            if self.a >= self.a_max:
                self.a = self.a_max
                self.state = 'phase2'
        elif self.state == 'phase2':
            self.v += self.a * DT
            self.s += self.v * DT
            if self.v >= self.v_limit - v1:
                self.state = 'phase3'
        elif self.state == 'phase3':
            self.a -= self.j * DT
            self.v += self.a * DT
            self.s += self.v * DT
            if self.a <= 0:
                self.a = 0
                if self.v > self.v_limit:
                    self.v = self.v_limit
                self.state = 'phase3_end'
        elif self.state == 'phase3_end':
            decel_dist = calc_decel_dist(self.v, self.a_max, self.j)
            if self.s >= abs(self.error_s) - decel_dist:
                self.state = 'phase5'
            else:
                self.state = 'phase4'
        elif self.state == 'phase4':
            self.s += self.v * DT
            decel_dist = calc_decel_dist(self.v, self.a_max, self.j)
            if self.s >= abs(self.error_s) - decel_dist:
                self.state = 'phase5'
        elif self.state == 'phase5':
            self.a -= self.j * DT
            self.v += self.a * DT
            self.s += self.v * DT
            if self.a <= -self.a_max:
                self.a = -self.a_max
                self.state = 'phase6'
        elif self.state == 'phase6':
            self.v += self.a * DT
            self.s += self.v * DT
            if self.v <= v1:
                self.state = 'phase7'
        elif self.state == 'phase7':
            self.a += self.j * DT
            self.v += self.a * DT
            self.s += self.v * DT
            if self.a >= 0 or self.v <= 0:
                self.a = 0
                self.v = 0
                self.state = 'idle'
        
        if (abs(self.s - abs(self.error_s)) <= 0.003 or 
            self.v < 0 or 
            self.s - abs(self.error_s) >= 0):
            self.v = 0
            self.a = 0
            self.state = 'idle'
        
        self.t += DT
        self.history['t'].append(self.t)
        self.history['s'].append(self.s * self.direction_flag)
        self.history['v'].append(self.v * self.direction_flag)
        self.history['a'].append(self.a * self.direction_flag)
        self.history['state'].append(self.state)
    
    def run(self, target_disp, max_steps=50000):
        self.init(target_disp)
        for _ in range(max_steps):
            self.step()
            if self.state == 'idle':
                break
        return self.history


def ascii_plot(data_list, labels, title, width=70, height=12):
    """Simple ASCII line plot"""
    # data_list: list of (x_arr, y_arr)
    all_y = []
    for _, y_arr in data_list:
        all_y.extend(y_arr)
    if not all_y:
        return
    
    ymin, ymax = min(all_y), max(all_y)
    if ymin == ymax:
        ymax = ymin + 1e-9
    
    # Normalize to [0, height-1]
    def norm(y):
        return int((y - ymin) / (ymax - ymin) * (height - 1))
    
    # Build grid
    grid = [[' ' for _ in range(width)] for _ in range(height)]
    colors = ['*', '+', 'o', 'x']
    
    for idx, (x_arr, y_arr) in enumerate(data_list):
        if len(x_arr) == 0:
            continue
        ch = colors[idx % len(colors)]
        n = len(x_arr)
        for i in range(n):
            col = int(i / max(n - 1, 1) * (width - 1))
            row = height - 1 - norm(y_arr[i])
            grid[row][col] = ch
    
    print(f"  {title}")
    print(f"  Y range: [{ymin:.4f}, {ymax:.4f}]")
    legend = "  Legend: " + ", ".join(f"{labels[i]}={colors[i%len(colors)]}" for i in range(len(labels)))
    print(legend)
    for row in grid:
        print("  |" + "".join(row) + "|")
    print("  " + "-" * (width + 2))


def analyze_curve(hist, target):
    """Analyze a single trajectory"""
    peak_v = max(abs(v) for v in hist['v']) if hist['v'] else 0
    peak_a = max(abs(a) for a in hist['a']) if hist['a'] else 0
    final_s = hist['s'][-1] if hist['s'] else 0
    duration = hist['t'][-1] if hist['t'] else 0
    
    # Count state transitions to see curve shape
    states = hist['state']
    unique_states = []
    for s in states:
        if not unique_states or unique_states[-1] != s:
            unique_states.append(s)
    
    # Find where it hit peak accel
    hit_amax = any(abs(a) >= A_MAX * 0.99 for a in hist['a'])
    
    return {
        'time_ms': duration * 1000,
        'peak_v': peak_v,
        'peak_a': peak_a,
        'final_s': final_s,
        'states': unique_states,
        'hit_amax': hit_amax,
        'overshoot': final_s - target,
    }


def print_analysis(name, disp, fixed_hist, adaptive_hist):
    print(f"\n{'='*60}")
    print(f"CASE: {name} | target = {disp:.4f} rad")
    print(f"{'='*60}")
    
    f = analyze_curve(fixed_hist, disp)
    a = analyze_curve(adaptive_hist, disp)
    
    print(f"\n  FIXED v_max={V_MAX}:")
    print(f"    Time: {f['time_ms']:.1f} ms | PeakV: {f['peak_v']:.4f} | PeakA: {f['peak_a']:.4f}")
    print(f"    Final: {f['final_s']:.5f} | Overshoot: {f['overshoot']:.5f}")
    print(f"    Hit a_max: {f['hit_amax']} | State seq: {' -> '.join(f['states'])}")
    
    print(f"\n  ADAPTIVE v_limit:")
    print(f"    Time: {a['time_ms']:.1f} ms | PeakV: {a['peak_v']:.4f} | PeakA: {a['peak_a']:.4f}")
    print(f"    Final: {a['final_s']:.5f} | Overshoot: {a['overshoot']:.5f}")
    print(f"    Hit a_max: {a['hit_amax']} | State seq: {' -> '.join(a['states'])}")
    
    # ASCII plots
    # Velocity
    ascii_plot(
        [(fixed_hist['t'], fixed_hist['v']), (adaptive_hist['t'], adaptive_hist['v'])],
        ['Fixed', 'Adaptive'],
        'Velocity (rad/s)',
        width=70, height=10
    )
    # Acceleration
    ascii_plot(
        [(fixed_hist['t'], fixed_hist['a']), (adaptive_hist['t'], adaptive_hist['a'])],
        ['Fixed', 'Adaptive'],
        'Acceleration (rad/s^2)',
        width=70, height=10
    )


if __name__ == '__main__':
    test_cases = [
        ("TINY  0.005 rad (~5mm)", 0.005),
        ("SMALL 0.020 rad (~2cm)", 0.020),
        ("MED   0.050 rad (~4.5cm)", 0.050),
        ("LARGE 0.200 rad (~18cm)", 0.200),
    ]
    
    print("=" * 60)
    print(f"PARAMS: j={J}, a_max={A_MAX}, v_max={V_MAX}")
    v1 = calc_v1(A_MAX, J)
    s_sym = 2 * calc_decel_dist(V_MAX, A_MAX, J)
    print(f"KEY: v1={v1:.4f} rad/s, S_sym_min={s_sym:.4f} rad")
    print("=" * 60)
    
    for name, disp in test_cases:
        sim_fixed = SpeedPlanSimulator(J, A_MAX, V_MAX, use_v_limit=False)
        hist_fixed = sim_fixed.run(disp)
        
        sim_adaptive = SpeedPlanSimulator(J, A_MAX, V_MAX, use_v_limit=True)
        hist_adaptive = sim_adaptive.run(disp)
        
        print_analysis(name, disp, hist_fixed, hist_adaptive)
