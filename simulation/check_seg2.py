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

print(f'Total segments: {len(segments)}')
for i, seg in enumerate(segments):
    print(f'  Seg {i}: {len(seg)} packets')
