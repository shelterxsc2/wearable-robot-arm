# -*- coding: utf-8 -*-
"""
低延迟优化版：修复 ffmpeg 时间戳问题
关键修改：
  1. 去掉 -use_wallclock_as_timestamps
  2. 显式指定 -r 15（隔帧推流的真实帧率）
  3. 加 -fps_mode passthrough 禁止帧重排
  4. 减小 GOP，去掉 bufsize
  5. 用 time.sleep 控制稳定推流间隔，替代 frame_count%2
"""
import cv2
from ultralytics import YOLO
import numpy as np
import time
import subprocess
import os
import statistics
import threading
import queue
import json
import websocket
import socket
import signal

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# 用系统 ffmpeg（Ubuntu 24.04 自带 h264_vaapi/qsv 支持）
FFMPEG_PATH = "ffmpeg"
STREAM_WIDTH = 1920
STREAM_HEIGHT = 1080
ENABLE_STREAMING = os.system("which ffmpeg >/dev/null 2>&1") == 0

# Arrow Lake 需要显式指定 iHD 驱动
if 'LIBVA_DRIVER_NAME' not in os.environ:
    os.environ['LIBVA_DRIVER_NAME'] = 'iHD'

DEVICE_ID = os.environ.get("DEVICE_ID", "device-002")
DEVICE_KEY = os.environ.get("DEVICE_KEY", "")

# ================== 摄像头帧率优化配置 ==================
# 目标摄像头帧率。该 Realtek USB 2.0 摄像头在 1080p 下实测最高约 20fps
#（曝光 62.5ms 时上限 16fps，曝光 50ms 时约 20fps），再往下硬件无法稳定。
# 默认 20；可通过环境变量覆盖，例如 TARGET_CAMERA_FPS=25。
TARGET_CAMERA_FPS = int(os.environ.get('TARGET_CAMERA_FPS', '20'))
TARGET_CAMERA_FPS = max(10, min(30, TARGET_CAMERA_FPS))
MEASURED_CAMERA_FPS = float(TARGET_CAMERA_FPS)  # 将在摄像头初始化后被实测值覆盖


def set_v4l2_exposure(target_fps):
    """通过 v4l2-ctl 设置手动曝光时间，使帧率接近目标值。
    exposure_time_absolute 单位是 100us：
      16fps -> 625, 20fps -> 500, 25fps -> 400, 30fps -> 333
    """
    exposure_abs = int(round(10000.0 / target_fps))  # 100us 单位
    exposure_abs = max(50, min(10000, exposure_abs))
    try:
        subprocess.run(
            ['v4l2-ctl', '-d', '0',
             '--set-ctrl', f'auto_exposure=1,exposure_time_absolute={exposure_abs}'],
            capture_output=True, check=False, timeout=5
        )
        print(f"[Camera] Set exposure_time_absolute={exposure_abs} for target {target_fps}fps")
    except Exception as e:
        print(f"[Camera] Failed to set V4L2 exposure: {e}")


def measure_camera_fps(cap, duration=2.0):
    """实测摄像头采集帧率，丢弃缓存旧帧。"""
    # 先丢弃几帧，让曝光/时序稳定
    for _ in range(10):
        cap.read()
    t0 = time.time()
    frames = 0
    while time.time() - t0 < duration:
        ret, _ = cap.read()
        if ret:
            frames += 1
    elapsed = time.time() - t0
    return frames / elapsed if elapsed > 0 else 0.0


# 从相对路径的 MediaMTX 配置中读取 RTMP 端口（改端口只需改 mediamtx.yml）
def get_rtmp_url():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mtx_yml = os.path.join(script_dir, "MediaMTX", "mediamtx.yml")
    port = 1935  # 默认端口
    if os.path.exists(mtx_yml):
        try:
            with open(mtx_yml, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line.startswith('rtmpAddress:'):
                        addr = line.split(':', 1)[1].strip()
                        if addr.startswith(':'):
                            port = int(addr[1:])
                        elif ':' in addr:
                            port = int(addr.split(':')[-1])
                        break
        except Exception:
            pass
    # 云服务器部署：使用公网 IP（可从环境变量覆盖）
    cloud_ip = os.environ.get('CLOUD_IP', '47.93.162.124')
    return f"rtmp://{cloud_ip}:1935/live/{DEVICE_ID}"

RTMP_URL = get_rtmp_url()
_ws_query = f"deviceId={DEVICE_ID}" + (f"&deviceKey={DEVICE_KEY}" if DEVICE_KEY else "")
WS_URL = f"ws://47.93.162.124/ws?{_ws_query}"

FACE_MODEL_PATH = os.path.join(SCRIPT_DIR, "models", "best_wflw_v8_pose20_openvino_model")
BODY_MODEL_PATH = os.path.join(SCRIPT_DIR, "models", "yolo26n-pose (1)_openvino_model")

# 推流每帧都推
INFER_SKIP = 1

# ================== 1-Euro Filter ==================
class OneEuroFilter:
    def __init__(self, t0=0.0, x0=0.0, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = x0
        self.dx_prev = 0.0
        self.t_prev = t0

    def smoothing_factor(self, t_e, cutoff):
        r = 2 * np.pi * cutoff * t_e
        return r / (r + 1)

    def exponential_smoothing(self, alpha, x, x_prev):
        return alpha * x + (1 - alpha) * x_prev

    def filter(self, t, x):
        if self.t_prev is None:
            self.t_prev = t
            self.x_prev = x
            return x
        t_e = t - self.t_prev
        if t_e <= 0:
            t_e = 1.0 / 30.0
        dx = (x - self.x_prev) / t_e
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat

# ================== PnP 配置 ==================
FACE_3D_TEMPLATE = np.array([
    [ 30.0, -25.0, -60.0],
    [-30.0, -25.0, -60.0],
    [  0.0,  -5.0, -90.0],
    [ 25.0,  20.0, -65.0],
    [-25.0,  20.0, -65.0],
    [  0.0,  50.0, -40.0],
], dtype=np.float32)

CAMERA_MATRIX = np.array([
    [1371,    0, 960],
    [   0, 1371, 540],
    [   0,    0,   1]
], dtype=np.float32)
DIST_COEFFS = np.zeros((4, 1), dtype=np.float32)

KPT_NAMES = {0: "nose", 1: "left_eye", 2: "right_eye", 17: "mouth_left", 18: "mouth_right", 19: "chin"}
REQUIRED_IDS = [2, 1, 0, 18, 17, 19]

# ================== 四元数工具 ==================
def rotvec_to_quat(rvec):
    theta = np.linalg.norm(rvec)
    if theta < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = rvec / theta
    half = theta * 0.5
    s = np.sin(half)
    return np.array([axis[0]*s, axis[1]*s, axis[2]*s, np.cos(half)])

def quat_to_euler(q):
    q = q / (np.linalg.norm(q) + 1e-8)
    x, y, z, w = q
    pitch = np.arctan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    sinp = 2.0*(w*y - z*x)
    yaw = np.copysign(np.pi/2, sinp) if abs(sinp) >= 1 else np.arcsin(sinp)
    roll = np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    return np.degrees([pitch, yaw, roll])

class PerformanceProfiler:
    def __init__(self, report_interval=30):
        self.report_interval = report_interval
        self.records = []
        self.frame_idx = 0

    def record(self, data):
        self.records.append(data)
        self.frame_idx += 1
        if self.frame_idx % self.report_interval == 0:
            self.print_report()

    def print_report(self):
        recent = self.records[-self.report_interval:]
        def avg(key):
            vals = [r.get(key, 0) * 1000 for r in recent if key in r]
            return statistics.mean(vals) if vals else 0
        def max_val(key):
            vals = [r.get(key, 0) * 1000 for r in recent if key in r]
            return max(vals) if vals else 0
        print("\n" + "="*60)
        print(f"[LowLatency] last {len(recent)} frames stats")
        print("-"*60)
        print(f"  total       : avg={avg('total'):7.2f}ms  max={max_val('total'):7.2f}ms")
        print(f"  infer_wall  : avg={avg('infer_wall'):7.2f}ms")
        print(f"  Face-GPU    : avg={avg('face_infer'):7.2f}ms")
        print(f"  pnp_compute : avg={avg('pnp'):7.2f}ms")
        print(f"  write()     : avg={avg('write'):7.2f}ms")
        print(f"  loop_fps    : avg={avg('loop_fps'):7.1f}")
        print("="*60)

# ================== 模型加载 ==================
print("Loading model...")
face_model = YOLO(FACE_MODEL_PATH, task="pose")
print("Model loaded")

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
# 关键：只保留1帧缓冲，减少V4L2队列延迟
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

# 设置曝光以接近目标帧率，并实测真实帧率
set_v4l2_exposure(TARGET_CAMERA_FPS)
MEASURED_CAMERA_FPS = measure_camera_fps(cap, duration=2.0)
actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
fourcc_str = "".join([chr((fourcc_int >> 8 * i) & 0xFF) for i in range(4)])
print(f"[Camera] Target={TARGET_CAMERA_FPS}fps, Measured={MEASURED_CAMERA_FPS:.1f}fps "
      f"({actual_w}x{actual_h} {fourcc_str})")
if MEASURED_CAMERA_FPS < TARGET_CAMERA_FPS * 0.85:
    print(f"[Camera] Warning: measured FPS is lower than target. "
          f"Consider increasing ambient light or lowering TARGET_CAMERA_FPS.")

# ================== 独立采集线程（最小延迟架构）====================
latest_frame = None
frame_lock = threading.Lock()
frame_ready = threading.Event()

def capture_thread():
    """独立采集线程：持续读摄像头，始终只保留最新帧"""
    global latest_frame
    while True:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                latest_frame = frame
            frame_ready.set()

capture_worker = threading.Thread(target=capture_thread, daemon=True)
capture_worker.start()
print("[Capture] Thread started, always keeping latest frame")

# ================== 滤波器初始化 ==================
flt_rx = OneEuroFilter(min_cutoff=2.5, beta=0.08)
flt_ry = OneEuroFilter(min_cutoff=2.5, beta=0.08)
flt_rz = OneEuroFilter(min_cutoff=2.5, beta=0.08)
flt_tx = OneEuroFilter(min_cutoff=2.5, beta=0.08)
flt_ty = OneEuroFilter(min_cutoff=2.5, beta=0.08)
flt_tz = OneEuroFilter(min_cutoff=2.5, beta=0.08)
prev_rvec = None
prev_tvec = None
REPROJ_THRESH = 10.0

# ================== FFmpeg 低延迟配置 ==================
VAAPI_DEVICE = os.environ.get('VAAPI_DEVICE', '/dev/dri/renderD128')

def get_best_h264_encoder(ffmpeg_path):
    """检测 ffmpeg 支持的 H.264 编码器，优先 VAAPI，其次 QSV，最后 libx264(CPU)"""
    try:
        result = subprocess.run([ffmpeg_path, '-encoders'], capture_output=True, text=True, timeout=10)
        encoders = result.stdout
        # VAAPI 在 Arrow Lake 上实测可用，QSV 不行
        if 'h264_vaapi' in encoders:
            return 'h264_vaapi'
        if 'h264_qsv' in encoders:
            return 'h264_qsv'
        if 'libx264' in encoders:
            return 'libx264'
    except Exception:
        pass
    return 'libx264'

def probe_rtmp_server(rtmp_url: str, timeout_ms: int = 3000) -> bool:
    """Probe whether the RTMP server is reachable by parsing host:port from URL."""
    try:
        # rtmp://host:port/path
        from urllib.parse import urlparse
        parsed = urlparse(rtmp_url)
        host = parsed.hostname
        port = parsed.port or 1935
        with socket.create_connection((host, port), timeout=timeout_ms / 1000.0):
            return True
    except Exception as e:
        print(f"[RTMP] Probe failed: {e}")
        return False


def build_ffmpeg_cmd(ffmpeg_path, rtmp_url, width, height, fps=None):
    """根据可用编码器构建 ffmpeg 命令"""
    if fps is None:
        fps = max(10, int(round(MEASURED_CAMERA_FPS)))
    encoder = get_best_h264_encoder(ffmpeg_path)
    print(f"[FFmpeg] 使用编码器: {encoder}")
    
    if encoder == 'h264_vaapi':
        # VAAPI 编码需要 hwupload + vaapi_device
        # 注意：VAAPI + RTMP 不能加 -fflags nobuffer 等低延迟参数，会导致服务器断连
        cmd = [
            ffmpeg_path, '-y',
            '-vaapi_device', VAAPI_DEVICE,
            '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}',
            '-r', str(fps),
            '-thread_queue_size', '512',
            '-i', '-',
            '-vf', 'format=nv12,hwupload',
            '-c:v', encoder,
            '-b:v', '4M', '-maxrate', '4M',
            '-g', '15',
            '-fps_mode', 'passthrough',
            '-f', 'flv', rtmp_url
        ]
    else:
        cmd = [
            ffmpeg_path, '-y',
            '-f', 'rawvideo', '-vcodec', 'rawvideo', '-pix_fmt', 'bgr24',
            '-s', f'{width}x{height}',
            '-r', str(fps),
            '-thread_queue_size', '512',
            '-i', '-',
            '-c:v', encoder,
            '-b:v', '4M', '-maxrate', '4M',
            '-g', '15',
            '-fps_mode', 'passthrough',
            '-fflags', 'nobuffer', '-flags', 'low_delay',
            '-probesize', '32', '-analyzeduration', '0',
            '-f', 'flv', rtmp_url
        ]
        if encoder == 'h264_qsv':
            cmd.extend(['-preset', 'veryfast', '-pix_fmt', 'nv12'])
        else:
            cmd.extend(['-preset', 'ultrafast', '-tune', 'zerolatency', '-pix_fmt', 'yuv420p'])
    return cmd

ffmpeg_proc = None
if ENABLE_STREAMING:
    cmd = build_ffmpeg_cmd(FFMPEG_PATH, RTMP_URL, STREAM_WIDTH, STREAM_HEIGHT)
    try:
        ffmpeg_log = open('ffmpeg_lowlatency.log', 'w')
        ffmpeg_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=ffmpeg_log)
        print(f"[Stream] FFmpeg low-latency started -> {RTMP_URL}")
    except Exception as e:
        print(f"[Stream] Failed to start: {e}")
        ffmpeg_proc = None

# ================== 推理线程 ==================
def infer_face(frame, model, container):
    t0 = time.time()
    results = model(frame, device="intel:gpu", verbose=False)
    t1 = time.time()
    container['results'] = results
    container['time'] = t1 - t0
    has_face = results[0].boxes is not None and len(results[0].boxes) > 0
    container['has_face'] = has_face

# ================== WebSocket 数据队列 ==================
data_queue = queue.Queue(maxsize=60)  # 缓存约 4 秒数据
stream_start_time = None
g_ws_ready = threading.Event()
g_stop_ws = threading.Event()


def _send_ws_heartbeat(ws, start_time, frame_count):
    """Send Base-compatible heartbeat frame_ts packet."""
    now = time.time()
    elapsed = (now - start_time) if start_time else 0.0
    msg = {
        'type': 'frame_ts',
        'data': {
            'timestamp': int(now),
            'elapsed': round(elapsed, 3),
            'frame_count': frame_count,
            'device': DEVICE_ID,
        }
    }
    ws.send(json.dumps(msg, default=lambda o: float(o) if isinstance(o, np.generic) else str(o)))

# ================== 前端遥控目标参数 ==================
target_pose = {'x': 0, 'y': 0, 'z': 0}  # 新协议: -5..5 离散三元组
target_lock = threading.Lock()

# 跟踪对象：0=第一人称，1=第三人称
track_obj = 1
track_lock = threading.Lock()
zoom = 1
view_mode = 1

def ws_worker():
    """WebSocket 客户端线程：Base-compatible registration + 100ms heartbeat."""
    ws = None
    reconnect_delay = 2.0
    registered = False
    start_time = None
    local_frame_count = 0
    last_beat = 0.0

    while not g_stop_ws.is_set():
        try:
            if ws is None:
                g_ws_ready.clear()
                registered = False
                ws = websocket.create_connection(WS_URL, timeout=5)
                print(f"[WS] Connected to cloud: {WS_URL}")
                reconnect_delay = 2.0
                start_time = time.time()

                # Base-compatible registration: pure frame_ts without data
                ws.send(json.dumps({'type': 'frame_ts'}))
                print("[WS] Registration frame_ts sent")
                # Give server a short moment to process registration
                time.sleep(0.2)
                registered = True
                g_ws_ready.set()
                last_beat = time.time()

            # 100 ms heartbeat loop, matching Base
            now = time.time()
            if registered and now - last_beat >= 0.1:
                _send_ws_heartbeat(ws, start_time, local_frame_count)
                last_beat = now

            # Non-blocking receive for server control messages / liveness detection
            ws.settimeout(0.05)
            try:
                msg = ws.recv()
                packet = json.loads(msg)
                ptype = packet.get('type')
                if ptype in ('set_target', 'target_pose'):
                    t = packet.get('target', {})
                    with target_lock:
                        target_pose['x'] = max(-5, min(5, int(t.get('x', 0))))
                        target_pose['y'] = max(-5, min(5, int(t.get('y', 0))))
                        target_pose['z'] = max(-5, min(5, int(t.get('z', 0))))
                    print(f"[Target] Received from cloud: {target_pose}")
                elif ptype == 'set_zoom':
                    global zoom
                    zoom = max(0, min(2, int(packet.get('zoom', 1))))
                    print(f"[Zoom] Received from cloud: {zoom}")
                elif ptype == 'set_view_mode':
                    global view_mode
                    view_mode = max(0, min(2, int(packet.get('viewMode', 1))))
                    print(f"[ViewMode] Received from cloud: {view_mode}")
                elif ptype == 'record_control_ack':
                    print(f"[Record] ack: {packet}")
                elif ptype == 'record_state':
                    print(f"[Record] state: {packet.get('data', {})}")
                elif ptype == 'track_obj':
                    obj = packet.get('trackObj', 1)
                    with track_lock:
                        global track_obj
                        track_obj = max(0, min(1, int(obj)))
                    obj_str = '第一人称' if track_obj == 0 else '第三人称'
                    print(f"[TrackObj] Received from cloud: {track_obj} ({obj_str})")
            except websocket.WebSocketTimeoutException:
                pass
            except Exception:
                pass

            # Forward data packets from main loop (if any)
            try:
                packet = data_queue.get(timeout=0.01)
                ws.send(json.dumps(packet, default=lambda o: float(o) if isinstance(o, np.generic) else str(o)))
            except queue.Empty:
                pass

            local_frame_count = frame_count

            time.sleep(0.01)
        except Exception as e:
            g_ws_ready.clear()
            registered = False
            if ws:
                try:
                    ws.close()
                except Exception:
                    pass
                ws = None
            print(f"[WS] Connection error: {e}, reconnect in {reconnect_delay:.1f}s")
            for _ in range(int(reconnect_delay / 0.1)):
                if g_stop_ws.is_set():
                    break
                time.sleep(0.1)
            reconnect_delay = min(reconnect_delay * 1.5, 30.0)

    if ws:
        try:
            ws.close()
        except Exception:
            pass

# ================== PnP 计算（无绘制）====================
def compute_pnp(face_results, t_now):
    """计算头部姿态（人脸相对于相机），返回姿态数据字典"""
    global prev_rvec, prev_tvec
    if face_results[0].boxes is None or len(face_results[0].boxes) == 0:
        return None
    
    keypoints_xy = face_results[0].keypoints.xy.cpu().numpy()
    kpts = keypoints_xy[0]
    
    kpt_map = {}
    for idx, (kx, ky) in enumerate(kpts):
        kpt_map[idx] = (float(kx), float(ky))
    
    if not all(kid in kpt_map for kid in REQUIRED_IDS):
        return None
    
    image_points = np.array([kpt_map[kid] for kid in REQUIRED_IDS], dtype=np.float32)
    
    if prev_rvec is not None and prev_tvec is not None:
        success, rvec_raw, tvec_raw = cv2.solvePnP(
            FACE_3D_TEMPLATE, image_points, CAMERA_MATRIX, DIST_COEFFS,
            rvec=prev_rvec.copy(), tvec=prev_tvec.copy(),
            useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE
        )
    else:
        success, rvec_raw, tvec_raw = cv2.solvePnP(
            FACE_3D_TEMPLATE, image_points, CAMERA_MATRIX, DIST_COEFFS,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
    
    if not success:
        return None
    
    rvec_raw = rvec_raw.flatten()
    tvec_raw = tvec_raw.flatten()

    # Mount correction: R_mount = Ry(-14°) * Rx(-0.10 rad), matching upstream.
    mp = -0.10
    my = -14.0 * math.pi / 180.0
    cp, sp = math.cos(mp), math.sin(mp)
    cy, sy = math.cos(my), math.sin(my)
    R_pitch = np.array([
        [1.0, 0.0, 0.0],
        [0.0,  cp, -sp],
        [0.0,  sp,  cp],
    ], dtype=np.float64)
    R_yaw = np.array([
        [ cy, 0.0,  sy],
        [0.0, 1.0, 0.0],
        [-sy, 0.0,  cy],
    ], dtype=np.float64)
    R_mount = R_yaw @ R_pitch
    R_face_to_cam_raw, _ = cv2.Rodrigues(rvec_raw)
    R_calib = R_mount @ R_face_to_cam_raw
    rvec_raw, _ = cv2.Rodrigues(R_calib)
    rvec_raw = rvec_raw.flatten()

    prev_rvec = rvec_raw.copy()
    prev_tvec = tvec_raw.copy()
    
    # 1-Euro Filter
    rvx = flt_rx.filter(t_now, rvec_raw[0])
    rvy = flt_ry.filter(t_now, rvec_raw[1])
    rvz = flt_rz.filter(t_now, rvec_raw[2])
    tx = flt_tx.filter(t_now, tvec_raw[0])
    ty = flt_ty.filter(t_now, tvec_raw[1])
    tz = flt_tz.filter(t_now, tvec_raw[2])
    
    display_rvec = np.array([rvx, rvy, rvz])
    tvec_f = np.array([tx, ty, tz])
    
    # 人脸相对于相机的欧拉角（直接用 rvec，不是相机逆姿态）
    q_disp = rotvec_to_quat(display_rvec)
    pitch, yaw, roll = quat_to_euler(q_disp)
    distance = float(np.linalg.norm(tvec_f))
    
    # 计算相机在人脸坐标系中的当前姿态和位置（与目标参数同坐标系）
    R_face_to_cam, _ = cv2.Rodrigues(rvec_raw)
    R_cam_to_face = R_face_to_cam.T
    cam_pos_face = -R_cam_to_face @ tvec_raw
    rvec_cam_to_face, _ = cv2.Rodrigues(R_cam_to_face)
    q_cam = rotvec_to_quat(rvec_cam_to_face.flatten())
    cam_pitch, cam_yaw, cam_roll = quat_to_euler(q_cam)
    
    # 面部关键点 2D 坐标
    kpts_2d = {name: kpt_map[kid] for kid, name in KPT_NAMES.items() if kid in kpt_map}
    
    return {
        'pitch': pitch, 'yaw': yaw, 'roll': roll,
        'distance_cm': distance / 10.0,
        'face_pos_mm': tvec_f.tolist(),          # 人脸中心在相机坐标系下的 (X,Y,Z) mm
        'keypoints_2d': kpts_2d,                 # 面部关键点像素坐标
        'cam_in_face': {                         # 相机在人脸坐标系中的当前姿态/位置
            'pos_mm': cam_pos_face.tolist(),
            'pitch': cam_pitch,
            'yaw': cam_yaw,
            'roll': cam_roll
        }
    }

profiler = PerformanceProfiler(report_interval=30)
prev_frame_time = time.time()
frame_count = 0
last_fps_time = time.time()
last_stream_time = time.time()

def _signal_handler(signum, _frame):
    print(f"\n[Main] Received signal {signum}, stopping...")
    g_stop_ws.set()


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)

print("\n" + "="*60)
print("Low-latency streaming + PnP compute (no draw)")
print("Control: s=toggle stream, q=quit")
print("="*60 + "\n")

# Probe RTMP server and decide whether to push RTMP + WebSocket (matches Base)
ws_thread = None
print(f"[Main] Probing RTMP server ({RTMP_URL})...")
if probe_rtmp_server(RTMP_URL, 3000):
    print("[Main] RTMP server is UP. Will push RTMP + WebSocket.")
    ws_thread = threading.Thread(target=ws_worker, daemon=True)
    ws_thread.start()
    print(f"[Main] WebSocket reporter started -> {WS_URL}")
    print("[Main] Waiting for WS registration before starting RTMP stream...")
    wait_ms = 0
    while not g_ws_ready.is_set() and wait_ms < 10000:
        time.sleep(0.1)
        wait_ms += 100
    if g_ws_ready.is_set():
        print("[Main] WS registration confirmed. Starting RTMP stream now.")
    else:
        print("[Main] WS registration timeout (10s). Starting RTMP stream anyway.")
else:
    print("[Main] RTMP server is DOWN. Will run locally without streaming/WebSocket.")

try:
    while not g_stop_ws.is_set():
        loop_start = time.time()

        # 键盘检测（先处理，避免被continue跳过）
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            g_stop_ws.set()
            break
        elif key == ord('s'):
            if ffmpeg_proc is not None:
                try:
                    ffmpeg_proc.stdin.close()
                    ffmpeg_proc.wait(timeout=3)
                except:
                    ffmpeg_proc.kill()
                ffmpeg_proc = None
                print("[Stream] Stopped")
            else:
                cmd = build_ffmpeg_cmd(FFMPEG_PATH, RTMP_URL, STREAM_WIDTH, STREAM_HEIGHT)
                ffmpeg_log = open('ffmpeg_lowlatency.log', 'w')
                ffmpeg_proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=ffmpeg_log)
                print("[Stream] Started")
        
        # 取最新帧（如果有），丢弃积压的旧帧
        has_frame = False
        with frame_lock:
            if latest_frame is not None:
                frame = latest_frame.copy()
                latest_frame = None  # 标记已消费
                has_frame = True
        
        if not has_frame:
            time.sleep(0.001)
            continue
        
        # ========== 单模型推理 ==========
        face_container = {'results': None, 'time': 0, 'has_face': False}
        
        t_infer_start = time.time()
        infer_face(frame, face_model, face_container)
        t_infer_wall = time.time() - t_infer_start
        
        # ========== PnP 姿态计算（无绘制）==========
        t_pnp_start = time.time()
        pose = compute_pnp(face_container['results'], time.time())
        t_pnp = time.time() - t_pnp_start
        
        # 帧率
        curr_time = time.time()
        
        # Update stream timing; the ws_worker sends Base-compatible heartbeat.
        if stream_start_time is None and ffmpeg_proc is not None:
            stream_start_time = curr_time

        fps = 1.0 / (curr_time - prev_frame_time + 1e-9)
        prev_frame_time = curr_time
        
        # 推流原始帧（每帧都推）
        t_write = 0.0
        if ffmpeg_proc is not None:
            try:
                if ffmpeg_proc.poll() is None:
                    t0 = time.time()
                    ffmpeg_proc.stdin.write(memoryview(frame))
                    t_write = time.time() - t0
            except Exception as e:
                print(f"[Stream] Write error: {e}")
        
        profiler.record({
            'total': time.time() - loop_start,
            'infer_wall': t_infer_wall,
            'face_infer': face_container['time'],
            'pnp': t_pnp,
            'write': t_write,
            'loop_fps': fps,
        })
        
        frame_count += 1
        if frame_count % 30 == 0:
            now = time.time()
            fps_30 = 30.0 / (now - last_fps_time + 1e-6)
            if pose:
                # 人体（人脸）相对于相机的姿态 + 位置 + 关键点坐标
                kpts = pose['keypoints_2d']
                kpt_str = " ".join([f"{n}:({x:.0f},{y:.0f})" for n, (x, y) in kpts.items()])
                pos = pose['face_pos_mm']
                print(f"[Info] FPS:{fps_30:.1f} Frames:{frame_count}")
                print(f"  Head->Cam  P:{pose['pitch']:.1f} Y:{pose['yaw']:.1f} R:{pose['roll']:.1f} "
                      f"D:{pose['distance_cm']:.1f}cm Pos:({pos[0]:.1f},{pos[1]:.1f},{pos[2]:.1f})mm")
                print(f"  Keypoints  {kpt_str}")
                
                # 目标差值（目标 - 当前）
                cam = pose['cam_in_face']
                with target_lock:
                    tgt = dict(target_pose)
                print(f"  TargetState x={tgt['x']} y={tgt['y']} z={tgt['z']} zoom={zoom} viewMode={view_mode}")
            else:
                print(f"[Info] FPS:{fps_30:.1f} Frames:{frame_count} (no face)")
            last_fps_time = now

except KeyboardInterrupt:
    print("\n[Info] Interrupted")
finally:
    g_stop_ws.set()
    if ws_thread is not None:
        ws_thread.join(timeout=2.0)
    if ffmpeg_proc:
        try:
            ffmpeg_proc.stdin.close()
            ffmpeg_proc.wait(timeout=5)
        except:
            ffmpeg_proc.kill()
    cap.release()
    print("[Done] Low-latency + PnP finished")
