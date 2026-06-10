import re, struct

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
    if not m: continue
    n = int(m.group(1))
    if n != 16: continue
    bytes_data = parse_c_string(m.group(2))
    if len(bytes_data) != 16: continue
    s, v, a, tail = struct.unpack('<ffff', bytes_data)
    if struct.unpack('<I', struct.pack('<f', tail))[0] == 0x7F800000:
        packets.append((i, s, v, a))

# Split into segments by move_success
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
    v_vals = [p[2] for p in seg]
    
    resets = []
    for i in range(1, len(s_vals)):
        if s_vals[i-1] > 0.01 and s_vals[i] < 0.001:
            resets.append(i)
    
    print(f'\nSegment {seg_idx}: {len(seg)} packets, resets={len(resets)}')
    start = 0
    for r in resets:
        sub_s = s_vals[start:r]
        print(f'  Sub-seg {start}-{r-1}: max_s={max(sub_s):.4f}, v_before_reset={v_vals[r-1]:.4f}')
        start = r
    sub_s = s_vals[start:]
    if sub_s:
        print(f'  Sub-seg {start}-end: max_s={max(sub_s):.4f}')
