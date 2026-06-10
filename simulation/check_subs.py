import math, struct, re

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
                result.append(0x0D); i += 2; continue
            elif s[i+1] == 'n':
                result.append(0x0A); i += 2; continue
        result.append(ord(s[i])); i += 1
    return bytes(result)

with open('cmd.txt', 'r') as f:
    lines = f.readlines()

packets = []
for i, line in enumerate(lines):
    m = re.search(r'n=(\d+) \| (.+)$', line.strip())
    if not m:
        continue
    n = int(m.group(1))
    if n != 16:
        continue
    bytes_data = parse_c_string(m.group(2))
    if len(bytes_data) != 16:
        continue
    s, v, a, tail = struct.unpack('<ffff', bytes_data)
    if struct.unpack('<I', struct.pack('<f', tail))[0] == 0x7F800000:
        packets.append((i, s, v, a))

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

for seg_idx, seg in enumerate(segments):
    s_vals = [p[1] for p in seg]
    resets = [i for i in range(1, len(s_vals)) if s_vals[i-1] > 0.01 and s_vals[i] < 0.001]
    subs = []
    start = 0
    for r in resets:
        subs.append((start, r, max(s_vals[start:r])))
        start = r
    if start < len(s_vals):
        subs.append((start, len(s_vals), max(s_vals[start:])))
    
    print(f'SEG {seg_idx}: {len(seg)} pkts, {len(subs)} subs')
    for sub_idx, (st, en, mx) in enumerate(subs):
        pkts = en - st
        v_before = seg[st-1][2] if st > 0 else 0.0
        print(f'  sub{sub_idx}: pkts={pkts:3d}, max_s={mx:.4f}, v_inherit={v_before:.4f}')
