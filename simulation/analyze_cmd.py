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

print(f'Total VOFA packets: {len(packets)}')

print('\nSample packets (first 15 non-zero):')
count = 0
for p in packets:
    if abs(p[1]) > 0.0001 or abs(p[2]) > 0.0001 or abs(p[3]) > 0.0001:
        print(f'  Line {p[0]}: s={p[1]:.6f}, v={p[2]:.6f}, a={p[3]:.6f}')
        count += 1
        if count >= 15:
            break

move_success = []
for i, line in enumerate(lines):
    if 'move_success' in line:
        move_success.append((i, line.strip()))

print(f'\nTotal move_success: {len(move_success)}')
for idx, (line_num, text) in enumerate(move_success):
    print(f'  [{idx}] Line {line_num}: {text[:100]}')

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

print(f'\nTotal segments: {len(segments)}')

for idx, seg in enumerate(segments):
    if len(seg) < 3:
        continue
    s_vals = [p[1] for p in seg]
    v_vals = [p[2] for p in seg]
    a_vals = [p[3] for p in seg]
    
    max_s = max(s_vals)
    max_v = max(v_vals)
    max_a = max(a_vals)
    min_a = min(a_vals)
    
    v_zero_idx = None
    for i, v in enumerate(v_vals):
        if abs(v) < 0.001:
            v_zero_idx = i
            break
    
    resets = 0
    for i in range(1, len(s_vals)):
        if s_vals[i-1] > 0.01 and s_vals[i] < 0.001:
            resets += 1
    
    a_jumps = 0
    for i in range(1, len(a_vals)):
        if abs(a_vals[i] - a_vals[i-1]) > 2.0:
            a_jumps += 1
    
    print(f'\nSegment {idx}: {len(seg)} packets')
    print(f'  max_s={max_s:.4f}, max_v={max_v:.4f}, max_a={max_a:.4f}, min_a={min_a:.4f}')
    if v_zero_idx is not None:
        print(f'  v->0 at packet {v_zero_idx}/{len(seg)} (s={s_vals[v_zero_idx]:.4f})')
    else:
        print(f'  v never reached 0!')
    if resets:
        print(f'  *** s RESET {resets} times (plan interrupted!) ***')
    if a_jumps:
        print(f'  *** a discontinuities: {a_jumps} ***')
    
    print(f'  s first 5: {[round(x,4) for x in s_vals[:5]]}')
    print(f'  s last  5: {[round(x,4) for x in s_vals[-5:]]}')
    print(f'  v first 5: {[round(x,4) for x in v_vals[:5]]}')
    print(f'  v last  5: {[round(x,4) for x in v_vals[-5:]]}')
    print(f'  a first 5: {[round(x,4) for x in a_vals[:5]]}')
    print(f'  a last  5: {[round(x,4) for x in a_vals[-5:]]}')
