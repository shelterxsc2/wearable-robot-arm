import re
import struct

def parse_c_string(s):
    result = bytearray()
    i = 0
    while i < len(s):
        if s[i] == '\\' and i + 1 < len(s):
            if s[i+1] == 'x' and i + 3 < len(s):
                try:
                    result.append(int(s[i+2:i+4], 16))
                    i += 4
                    continue
                except ValueError:
                    pass
            elif s[i+1] == 'r':
                result.append(0x0D)
                i += 2
                continue
            elif s[i+1] == 'n':
                result.append(0x0A)
                i += 2
                continue
            elif s[i+1] == 't':
                result.append(0x09)
                i += 2
                continue
        result.append(ord(s[i]))
        i += 1
    return bytes(result)

with open('cmd.txt', 'r', encoding='utf-8', errors='replace') as f:
    lines = f.readlines()

packets = []
for i, line in enumerate(lines):
    m = re.search(r'n=(\d+) \| (.+)$', line.strip())
    if not m:
        continue
    n = int(m.group(1))
    payload_str = m.group(2)
    if n != 16:
        continue
    bytes_data = parse_c_string(payload_str)
    if len(bytes_data) != 16:
        continue
    s, v, a, tail = struct.unpack('<ffff', bytes_data)
    tail_int = struct.unpack('<I', struct.pack('<f', tail))[0]
    if tail_int == 0x7F800000:
        packets.append((i, s, v, a))

move_success = []
for i, line in enumerate(lines):
    if 'move_success' in line:
        move_success.append((i, line.strip()))

segments = []
current_seg = []
packet_idx = 0

for i, line in enumerate(lines):
    while packet_idx < len(packets) and packets[packet_idx][0] == i:
        current_seg.append(packets[packet_idx])
        packet_idx += 1
    if 'move_success' in line:
        if current_seg:
            segments.append(current_seg)
            current_seg = []

if current_seg:
    segments.append(current_seg)

print("=" * 70)
print("LONG-DISTANCE MOTION INTERRUPTION ANALYSIS")
print("=" * 70)

for idx, seg in enumerate(segments):
    if len(seg) < 3:
        continue
    
    s_vals = [p[1] for p in seg]
    v_vals = [p[2] for p in seg]
    a_vals = [p[3] for p in seg]
    max_s = max(s_vals)
    
    # Only analyze long-distance motions: max_s > 1.0 rad (~57 deg)
    if max_s < 1.0:
        continue
    
    print(f"\n{'='*70}")
    print(f"SEGMENT {idx}: LONG-DISTANCE MOTION")
    print(f"{'='*70}")
    print(f"Total packets: {len(seg)}")
    print(f"Max planned displacement: {max_s:.4f} rad ({max_s*180/3.14159:.1f} deg)")
    print(f"Peak velocity: {max(v_vals):.4f} rad/s")
    print(f"Peak acceleration: {max(a_vals):.4f} rad/s^2")
    print(f"Min acceleration: {min(a_vals):.4f} rad/s^2")
    
    # Find all reset points: s drops from >0.01 to ~0
    resets = []
    for i in range(1, len(s_vals)):
        if s_vals[i-1] > 0.01 and s_vals[i] < 0.001:
            resets.append({
                'packet_idx': i,
                's_before': s_vals[i-1],
                'v_before': v_vals[i-1],
                'a_before': a_vals[i-1],
                'progress': s_vals[i-1] / max_s * 100 if max_s > 0 else 0
            })
    
    print(f"\nNumber of interruptions: {len(resets)}")
    
    if resets:
        print(f"\n{'-'*70}")
        print(f"INTERRUPTION DETAILS:")
        print(f"{'-'*70}")
        for ridx, r in enumerate(resets):
            # Determine phase based on acceleration sign and velocity trend
            a = r['a_before']
            v = r['v_before']
            
            phase_guess = "unknown"
            if a > 0.5:
                if v < max(v_vals) * 0.7:
                    phase_guess = "accelerating (early phase)"
                else:
                    phase_guess = "accelerating (late phase / near v_limit)"
            elif a < -0.5:
                phase_guess = "decelerating"
            elif abs(a) < 0.5:
                if v > 0.1:
                    phase_guess = "coasting (constant velocity)"
                else:
                    phase_guess = "near stop / idle"
            
            print(f"  [{ridx+1}] Packet {r['packet_idx']:3d}/{len(seg):3d} | "
                  f"s={r['s_before']:7.4f} ({r['progress']:5.1f}%) | "
                  f"v={r['v_before']:7.4f} | a={r['a_before']:7.4f} | {phase_guess}")
    
    # Show full s/v/a trajectory with reset markers
    print(f"\n{'-'*70}")
    print(f"FULL TRAJECTORY (sampling every ~10 packets):")
    print(f"{'-'*70}")
    print(f"{'Pkt':>5} | {'s':>8} | {'v':>8} | {'a':>8} | {'Note'}")
    print(f"{'-'*50}")
    
    reset_set = set(r['packet_idx'] for r in resets)
    for i in range(0, len(seg), max(1, len(seg)//20)):
        s, v, a = s_vals[i], v_vals[i], a_vals[i]
        note = "<<< RESET" if i in reset_set else ""
        if not note and i+1 < len(seg) and (s_vals[i+1] < 0.001 and s > 0.01):
            note = "<<< RESET"
        print(f"{i:5d} | {s:8.4f} | {v:8.4f} | {a:8.4f} | {note}")
    
    # Last packet
    if len(seg) - 1 not in [i for i in range(0, len(seg), max(1, len(seg)//20))]:
        i = len(seg) - 1
        print(f"{i:5d} | {s_vals[i]:8.4f} | {v_vals[i]:8.4f} | {a_vals[i]:8.4f} | END")

print(f"\n{'='*70}")
print("SUMMARY")
print(f"{'='*70}")

long_segments = [seg for seg in segments if len(seg) >= 3 and max(p[1] for p in seg) >= 1.0]
print(f"Total long-distance motions analyzed: {len(long_segments)}")

all_resets = []
for seg in long_segments:
    s_vals = [p[1] for p in seg]
    v_vals = [p[2] for p in seg]
    a_vals = [p[3] for p in seg]
    max_s = max(s_vals)
    for i in range(1, len(s_vals)):
        if s_vals[i-1] > 0.01 and s_vals[i] < 0.001:
            all_resets.append({
                'progress': s_vals[i-1] / max_s * 100,
                'v': v_vals[i-1],
                'a': a_vals[i-1]
            })

if all_resets:
    avg_progress = sum(r['progress'] for r in all_resets) / len(all_resets)
    print(f"Total interruptions in long motions: {len(all_resets)}")
    print(f"Average progress at interruption: {avg_progress:.1f}%")
    
    accel_count = sum(1 for r in all_resets if r['a'] > 0.5)
    coast_count = sum(1 for r in all_resets if abs(r['a']) <= 0.5 and r['v'] > 0.1)
    decel_count = sum(1 for r in all_resets if r['a'] < -0.5)
    
    print(f"\nBreakdown by phase:")
    print(f"  During acceleration (a > 0.5):  {accel_count} ({accel_count/len(all_resets)*100:.1f}%)")
    print(f"  During coasting (|a| <= 0.5):   {coast_count} ({coast_count/len(all_resets)*100:.1f}%)")
    print(f"  During deceleration (a < -0.5): {decel_count} ({decel_count/len(all_resets)*100:.1f}%)")
