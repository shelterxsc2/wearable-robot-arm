#!/usr/bin/env python3
"""
S-Curve Speed Profile Simulator v3
Compare old params vs new slower params
"""

import math

DT = 0.001

def calc_v1(a_max, j):
    return (a_max * a_max) / (2.0 * j)

def calc_decel_dist(v, a_max, j):
    return (v * v) / (2.0 * a_max) + (v * a_max) / (2.0 * j)

class SpeedPlanSimulator:
    def __init__(self, j, a_max, v_max):
        self.j = j
        self.a_max = a_max
        self.v_max = v_max
        
        self.v_limit = v_max
        self.a_limit = a_max
        self.a = 0.0
        self.v = 0.0
        self.s = 0.0
        self.error_s = 0.0
        self.direction_flag = 1.0
        self.state = 'idle'
        self.history = {'t': [], 's': [], 'v': [], 'a': [], 'state': [], 'a_lim': [], 'v_lim': []}
        self.t = 0.0
    
    def reset(self):
        self.a = 0.0
        self.v = 0.0
        self.s = 0.0
        self.state = 'idle'
        self.t = 0.0
        self.history = {'t': [], 's': [], 'v': [], 'a': [], 'state': [], 'a_lim': [], 'v_lim': []}
    
    def init(self, target_disp):
        self.reset()
        self.error_s = target_disp
        self.direction_flag = 1.0 if target_disp >= 0 else -1.0
        
        S = abs(target_disp)
        v1 = calc_v1(self.a_max, self.j)
        
        a = 1.0
        b = (self.a_max ** 2) / self.j
        c = -S * self.a_max
        disc = b * b - 4.0 * a * c
        v_peak = (-b + math.sqrt(disc)) / (2.0 * a)
        
        if v_peak < 2.0 * v1:
            v_peak = (0.5 * self.j * S * S) ** (1.0 / 3.0)
        
        scale = 1.0
        if S < 0.05:
            scale = 0.6
        elif S < 0.15:
            scale = 0.8
        v_peak *= scale
        
        self.v_limit = min(v_peak, self.v_max)
        if self.v_limit < 0:
            self.v_limit = 0
        
        v1_limit = calc_v1(self.a_max, self.j)
        if self.v_limit < 2.0 * v1_limit:
            self.a_limit = math.sqrt(self.j * self.v_limit)
        else:
            self.a_limit = self.a_max
        
        self.state = 'phase1'
    
    def step(self):
        v1 = calc_v1(self.a_limit, self.j)
        
        if self.state == 'idle':
            pass
        elif self.state == 'phase1':
            self.a += self.j * DT
            self.v += self.a * DT
            self.s += self.v * DT
            if self.a >= self.a_limit:
                self.a = self.a_limit
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
            decel_dist = calc_decel_dist(self.v, self.a_limit, self.j)
            if self.s >= abs(self.error_s) - decel_dist:
                self.state = 'phase5'
            else:
                self.state = 'phase4'
        elif self.state == 'phase4':
            self.s += self.v * DT
            decel_dist = calc_decel_dist(self.v, self.a_limit, self.j)
            if self.s >= abs(self.error_s) - decel_dist:
                self.state = 'phase5'
        elif self.state == 'phase5':
            self.a -= self.j * DT
            self.v += self.a * DT
            self.s += self.v * DT
            if self.a <= -self.a_limit:
                self.a = -self.a_limit
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
        self.history['a_lim'].append(self.a_limit)
        self.history['v_lim'].append(self.v_limit)
    
    def run(self, target_disp, max_steps=50000):
        self.init(target_disp)
        for _ in range(max_steps):
            self.step()
            if self.state == 'idle':
                break
        return self.history


def analyze(hist, target):
    peak_v = max(abs(v) for v in hist['v']) if hist['v'] else 0
    peak_a = max(abs(a) for a in hist['a']) if hist['a'] else 0
    final_s = hist['s'][-1] if hist['s'] else 0
    duration = hist['t'][-1] if hist['t'] else 0
    a_limit = hist['a_lim'][0] if hist['a_lim'] else 0
    v_limit = hist['v_lim'][0] if hist['v_lim'] else 0
    states = []
    for s in hist['state']:
        if not states or states[-1] != s:
            states.append(s)
    return {
        'time_ms': duration * 1000,
        'peak_v': peak_v,
        'peak_a': peak_a,
        'final_s': final_s,
        'a_limit': a_limit,
        'v_limit': v_limit,
        'states': states,
        'overshoot': final_s - target,
    }


if __name__ == '__main__':
    test_cases = [
        ("TINY  0.005 rad", 0.005),
        ("SMALL 0.020 rad", 0.020),
        ("MED   0.050 rad", 0.050),
        ("LARGE 0.200 rad", 0.200),
    ]
    
    print("=" * 80)
    print("OLD PARAMS: j=15.5, a_max=1.5, v_max=0.45")
    print("NEW PARAMS: j=10.0, a_max=1.0, v_max=0.30")
    print("=" * 80)
    
    for name, disp in test_cases:
        print(f"\n[{name}] target={disp:.3f}")
        print("-" * 80)
        
        sim_old = SpeedPlanSimulator(15.5, 1.5, 0.45)
        o = analyze(sim_old.run(disp), disp)
        print(f"OLD  v_lim={o['v_limit']:.3f} a_lim={o['a_limit']:.3f} | "
              f"Time={o['time_ms']:.0f}ms PeakV={o['peak_v']:.4f} PeakA={o['peak_a']:.4f} | "
              f"Final={o['final_s']:.5f}")
        print(f"     States: {'->'.join(o['states'])}")
        
        sim_new = SpeedPlanSimulator(10.0, 1.0, 0.30)
        n = analyze(sim_new.run(disp), disp)
        print(f"NEW  v_lim={n['v_limit']:.3f} a_lim={n['a_limit']:.3f} | "
              f"Time={n['time_ms']:.0f}ms PeakV={n['peak_v']:.4f} PeakA={n['peak_a']:.4f} | "
              f"Final={n['final_s']:.5f}")
        print(f"     States: {'->'.join(n['states'])}")
