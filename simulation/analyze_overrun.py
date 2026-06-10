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

print("=" * 80)
print("OVERRUN ANALYSIS: Focus on END-GAME interruptions & high-speed near-target")
print("=" * 80)

for idx, seg in enumerate(segments):
    if len(seg) < 3:
        continue
    
    s_vals = [p[1] for p in seg]
    v_vals = [p[2] for p in seg]
    a_vals = [p[3] for p in seg]
    max_s = max(s_vals)
    
    print(f"\n{'='*80}")
    print(f"SEGMENT {idx}: max_s={max_s:.4f} rad ({max_s*180/3.14159:.1f} deg), {len(seg)} packets")
    print(f"{'='*80}")
    
    # Find all reset points with details
    resets = []
    for i in range(1, len(s_vals)):
        if s_vals[i-1] > 0.01 and s_vals[i] < 0.001:
            resets.append({
                'pkt': i,
                's_before': s_vals[i-1],
                'v_before': v_vals[i-1],
                'a_before': a_vals[i-1],
                'progress': s_vals[i-1] / max_s * 100 if max_s > 0 else 0
            })
    
    if resets:
        print(f"\n  Resets found: {len(resets)}")
        for r in resets:
            # Classify: early (<30%), mid (30-70%), late (>70%)
            phase = "early" if r['progress'] < 30 else ("mid" if r['progress'] < 70 else "LATE")
            flag = "<<< HIGH-SPEED LATE RESET!" if (phase == "LATE" and r['v_before'] > 0.3) else ""
            print(f"    Pkt {r['pkt']:3d}: s={r['s_before']:7.4f} ({r['progress']:5.1f}%) | "
                  f"v={r['v_before']:7.4f} | a={r['a_before']:7.4f} | [{phase}] {flag}")
    
    # Focus: last 20 packets before move_success
    tail_len = min(25, len(seg))
    tail_start = len(seg) - tail_len
    
    print(f"\n  LAST {tail_len} PACKETS (end-game):")
    print(f"  {'Pkt':>4} | {'s':>8} | {'v':>8} | {'a':>8} | {'Note'}")
    print(f"  {'-'*55}")
    
    for i in range(tail_start, len(seg)):
        note = ""
        if i > 0 and s_vals[i-1] > 0.01 and s_vals[i] < 0.001:
            note = "<<< RESET (v={:.3f})".format(v_vals[i-1])
        elif s_vals[i] > 0.001 and v_vals[i] > 0.5 and (max_s - s_vals[i]) < 0.3:
            note = "<<< HIGH v near target!"
        elif s_vals[i] > 0.001 and a_vals[i] < -0.5 and v_vals[i] > 0.5:
            note = "<<< decel but v still high"
        print(f"  {i:4d} | {s_vals[i]:8.4f} | {v_vals[i]:8.4f} | {a_vals[i]:8.4f} | {note}")
    
    # Critical check: find packets where s is near max but v is still significant
    # This indicates the motor is approaching target at high speed with insufficient decel distance
    print(f"\n  CRITICAL FINDINGS:")
    critical_count = 0
    for i in range(len(seg)):
        if s_vals[i] > 0.001 and v_vals[i] > 0.5:
            remaining = max_s - s_vals[i]
            # Approx decel distance needed: v^2 / (2*a_max) with a_max~2.2
            decel_needed = (v_vals[i] ** 2) / (2 * 2.2)
            if remaining < decel_needed and remaining > 0:
                print(f"    Pkt {i}: s={s_vals[i]:.4f}, v={v_vals[i]:.4f}, "
                      f"remaining={remaining:.4f}, decel_needed={decel_needed:.4f} "
                      f"-> INSUFFICIENT BRAKING DISTANCE!")
                critical_count += 1
    if critical_count == 0:
        print(f"    (no critical high-speed near-target events in this segment)")

print(f"\n{'='*80}")
print("KEY EVIDENCE: Check if v > 0.5 when s is within last 0.3 rad of target")
print("If so, the motor cannot decelerate to 0 before reaching target -> OVERRUN")
print(f"{'='*80}")
