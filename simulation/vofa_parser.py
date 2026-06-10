"""
VOFA 数据解析器：从 cmd.txt 提取 s,v,a 时间序列
cmd.txt 中的数据是文本转义格式（Python repr 风格），非原始二进制
"""
import struct
import re
from typing import List, Tuple
import numpy as np

def parse_cmd_txt(path: str = '../cmd.txt') -> List[Tuple[float, float, float, float]]:
    """
    解析 cmd.txt，返回 (timestamp_sec, s, v, a) 列表
    timestamp 以当天 0:00 为基准的秒数
    """
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        lines = f.readlines()
    
    pattern = re.compile(r'\[(\d{2}:\d{2}:\d{2}\.\d{3})\].*\|\s*(.*)')
    frames = []
    
    for line in lines:
        m = pattern.search(line)
        if not m:
            continue
        ts, payload_str = m.groups()
        try:
            payload = eval('b"' + payload_str + '"')
        except Exception:
            continue
        if len(payload) != 16:
            continue
        
        s, v, a, tail = struct.unpack('<ffff', payload)
        if tail != float('inf'):
            continue
        
        h, m, sec = ts.split(':')
        t_total = int(h) * 3600 + int(m) * 60 + float(sec)
        frames.append((t_total, s, v, a))
    
    return frames

def segment_frames(frames: List[Tuple[float, float, float, float]], 
                   reset_threshold: float = 0.001) -> List[List[Tuple[float, float, float, float]]]:
    """
    将连续帧分割为运动段。
    当 s 突然重置为接近 0（且上一帧 s 较大）时，视为新段开始。
    """
    segments = []
    current = []
    
    for i, (t, s, v, a) in enumerate(frames):
        if i > 0 and s < reset_threshold and frames[i - 1][1] > 0.01:
            if current:
                segments.append(current)
            current = [(t, s, v, a)]
        else:
            current.append((t, s, v, a))
    
    if current:
        segments.append(current)
    
    return segments

def frames_to_arrays(frames: List[Tuple[float, float, float, float]]) -> dict:
    """将帧列表转为 numpy 数组字典"""
    if not frames:
        return {}
    arr = np.array(frames)
    return {
        't': arr[:, 0],
        's': arr[:, 1],
        'v': arr[:, 2],
        'a': arr[:, 3],
    }

if __name__ == '__main__':
    frames = parse_cmd_txt()
    print(f'Total frames: {len(frames)}')
    segs = segment_frames(frames)
    print(f'Segments: {len(segs)}')
    for i, seg in enumerate(segs[:5]):
        d = frames_to_arrays(seg)
        print(f'  Seg {i}: {len(seg)} frames, max_s={d["s"].max():.4f}, max_v={np.abs(d["v"]).max():.4f}')
