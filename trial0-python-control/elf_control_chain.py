# -*- coding: utf-8 -*-
"""
ELF wearable-robot-arm hardware control chain (Python port).

This module ports the NRF24 head-IMU + IMU2 waist + visual PnP correction +
HTTP/BLE external control -> UART lower-level control chain from
/home/time/work/elf_info/wearable-robot-arm/src/rga_npu.cpp to Python.

It is designed to be backend-pluggable:
  - real hardware backends: STM32 DataHub bridge (default) or DK-2500 GPIO SPI
  - stub backends are used by default so the chain can be exercised on x86
    without any robot hardware connected.
"""
from __future__ import annotations

import abc
import http.server
import json
import math
import os
import queue
import socketserver
import struct
import sys
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Constants / configuration (mirrors rga_npu.cpp and nrf24_linux.h)
# ---------------------------------------------------------------------------

CONTROL_PERIOD_US = 50_000          # 50 ms control loop
CONTROL_PERIOD_S = 0.05

KI_PNP = 0.04
KI_PNP_PITCH_BIAS = 0.02
PNP_FRAME_ERROR_LIMIT_DEG = 15.0
K_PNP_PITCH_Z_CM = 0.0
PNP_INTEGRAL_DEADBAND_DEG = 3.0

CMD_YAW_THRESHOLD_DEG = 5.0
CMD_PITCH_THRESHOLD_DEG = 5.0
POS_DEADZONE_CM = 1.0
INTERVAL_US = 200_000
FAST_INTERVAL_US = 150_000

STATIONARY_ENTER_W = 2.0
STATIONARY_EXIT_W = 5.0
STABLE_TIMEOUT_US = 800_000

PITCH_BIAS_FULL_YAW_DEG = 35.0

# Calibration constants (from rga_npu.cpp)
PNP_CALIB_TARGETS_YAW = [0, -15, -30, -45, -60, -75, 15, 30, 45, 60, 75]
PNP_CALIB_TARGETS_PITCH = [0, -15, -30, 15, 30]
PNP_CALIB_PREPARE_US = 5_000_000
PNP_CALIB_SAMPLE_US = 8_000_000

HEAD_IMU_GRID_TARGETS = [
    ("left_up", -30.0, 20.0), ("up", 0.0, 20.0), ("right_up", 30.0, 20.0),
    ("left", -30.0, 0.0), ("center", 0.0, 0.0), ("right", 30.0, 0.0),
    ("left_down", -30.0, -20.0), ("down", 0.0, -20.0), ("right_down", 30.0, -20.0),
]

# Scenario mode constants (from rga_npu.cpp)
SCENARIO_KPT_CONF_THRESHOLD = 0.30
INTRO_BASE_X_CM = -20.0
INTRO_BASE_Y_CM = 85.0
INTRO_BASE_Z_CM = 15.0
INTRO_SERVO1_BASE_DEG = 65.0
INTRO_SERVO2_BASE_DEG = 30.0
INTRO_SERVO2_CENTER_LOCK_DEG = 50.0
INTRO_IMAGE_TO_YAW_SIGN = 1.0
INTRO_CENTER_HOLD_ENTER_NORM = 0.06
INTRO_CENTER_HOLD_EXIT_NORM = 0.12
INTRO_TARGET_WRIST_WEIGHT = 0.90
INTRO_SERVO2_GAIN_DEG = 60.0
INTRO_SERVO2_SPACE_MOVING_GAIN_SCALE = 0.20
INTRO_SERVO2_DELTA_LIMIT_DEG = 60.0
SCENARIO_SERVO_MAX_STEP_DEG = 5.0
INTRO_SERVO2_SEND_DEADBAND_DEG = 4.0
INTRO_HAND_CENTER_NORM = 0.18
INTRO_HAND_PRESENT_NORM = 0.30
INTRO_SPACE_CENTER_STEP_CM = 4.0
INTRO_SERVO_PERIOD_US = 700_000
INTRO_HAND_CENTER_HOLD_US = 800_000
INTRO_SPACE_CENTER_PERIOD_US = 900_000
INTRO_SPACE_PRESENT_PERIOD_US = 900_000
INTRO_SPACE_MOVING_GAIN_US = 900_000

INTERVIEW_BASE_X_CM = 10.0
INTERVIEW_BASE_Y_CM = 85.0
INTERVIEW_BASE_Z_CM = 15.0
INTERVIEW_SERVO1_BASE_DEG = 65.0
INTERVIEW_SERVO2_BASE_DEG = 50.0
INTERVIEW_IMAGE_TO_YAW_SIGN = 1.0
INTERVIEW_SERVO2_I_GAIN_DEG_PER_SEC = 24.0
INTERVIEW_SERVO2_DELTA_LIMIT_DEG = 45.0
INTERVIEW_SERVO2_SEND_DEADBAND_DEG = 3.0
INTERVIEW_HOLD_ENTER_NORM = 0.05
INTERVIEW_HOLD_EXIT_NORM = 0.10
INTERVIEW_WRIST_CONF_THRESHOLD = 0.15
INTERVIEW_SERVO_PERIOD_US = 700_000

FIRST_PERSON_BASE_X_CM = -20.0
FIRST_PERSON_BASE_Y_CM = 30.0
FIRST_PERSON_BASE_Z_CM = 20.0
# Safe first-person camera-space coordinate window, in cm.  The upstream arm
# protocol accepts int16 coordinates, but practical workspace is much smaller;
# keep this around the existing first-person base pose.
FIRST_PERSON_MIN_X_CM = -35.0
FIRST_PERSON_MAX_X_CM = -5.0
FIRST_PERSON_MIN_Y_CM = 15.0
FIRST_PERSON_MAX_Y_CM = 55.0
FIRST_PERSON_MIN_Z_CM = 5.0
FIRST_PERSON_MAX_Z_CM = 35.0
FIRST_PERSON_FORWARD_STEP_CM = 3.0
FIRST_PERSON_LEFT_STEP_CM = 3.0
FIRST_PERSON_UP_STEP_CM = 3.0
FIRST_PERSON_BASE_J4_DEG = 10.0
# Linear wrist-pitch compensation for the L3 endpoint height vector.
FIRST_PERSON_J4_DEG_PER_Z_CM = 1.0
FIRST_PERSON_BASE_J5_DEG = 180.0
FIRST_PERSON_YAW_GAIN = 1.0
FIRST_PERSON_PITCH_GAIN = 1.0
FIRST_PERSON_SERVO_DEADBAND_DEG = 1.0
FIRST_PERSON_SERVO_PERIOD_US = 100_000
AUTO_FIRST_PERSON_DELAY_S = 10.0
SAFE_POWER_OFF_WAIT_S = 3.0

NRF24_WY_HIST_SIZE = 10
NRF24_WZ_HIST_SIZE = 10
NRF24_ANGLE_HIST_SIZE = 16

# 8-state motion FSM
(
    STATE_STOP,
    STATE_STOP_TO_ACCEL,
    STATE_ACCEL,
    STATE_ACCEL_TO_CONST,
    STATE_CONST_SPEED,
    STATE_CONST_TO_DECEL,
    STATE_DECEL_TO_STOP,
    STATE_DECEL_STOP_TO_ACCEL,
) = range(8)

STATE_NAMES = {
    STATE_STOP: "STOP",
    STATE_STOP_TO_ACCEL: "STOP_TO_ACCEL",
    STATE_ACCEL: "ACCEL",
    STATE_ACCEL_TO_CONST: "ACCEL_TO_CONST",
    STATE_CONST_SPEED: "CONST_SPEED",
    STATE_CONST_TO_DECEL: "CONST_TO_DECEL",
    STATE_DECEL_TO_STOP: "DECEL_TO_STOP",
    STATE_DECEL_STOP_TO_ACCEL: "DECEL_STOP_TO_ACCEL",
}

# Default arm profile: far_l3_55
ARM_PROFILE_FAR_L3_55 = 1
ARM_PROFILE_MID_L3_40 = 0
ARM_PROFILE_EXTRA_FAR_L3_65 = 2

# PnP calibration tables (from rga_npu.cpp)
PNP_YAW_CALIB_L3_40 = [
    (-75.0, 8.1134), (-60.0, 10.0633), (-45.0, 8.1285), (-30.0, 7.3703),
    (-15.0, 5.3609), (0.0, 6.8933), (15.0, 5.8331), (30.0, 6.7012),
    (45.0, 5.7065), (60.0, 6.8654), (75.0, 9.7055),
]
PNP_YAW_CALIB_L3_55 = [
    (-75.0, 5.2863), (-60.0, 4.1826), (-45.0, 6.4065), (-30.0, 8.4788),
    (-15.0, 5.8080), (0.0, 7.2326), (15.0, 7.1712), (30.0, 8.1047),
    (45.0, 5.6603), (60.0, 7.9074),
]
PNP_PITCH_CALIB_L3_40 = [
    (-30.0, -2.3324), (-15.0, 8.4560), (0.0, -0.4869),
    (15.0, -7.4892), (30.0, -5.7054),
]
PNP_PITCH_CALIB_L3_55 = [
    (-30.0, -4.9463), (-15.0, 1.0454), (0.0, -2.3915),
    # Previous values: (15.0, -2.1471), (30.0, -5.5954)
    (15.0, 13.4265), (30.0, 17.1709),
]


@dataclass
class ArmKinematicsProfile:
    name: str
    l1: float
    l2: float
    l3: float
    l4: float
    pitch_z_gain: float
    servo1_baseline: float
    servo1_gain_up: float
    servo1_gain_down: float
    servo2_baseline: float
    servo2_yaw_gain: float
    position_pitch_sign: int
    pitch_pos_fade_start_yaw_deg: float
    pitch_pos_fade_end_yaw_deg: float
    yaw_calib: List[Tuple[float, float]]
    pitch_calib: List[Tuple[float, float]]


ARM_PROFILES = [
    ArmKinematicsProfile(
        name="mid_l3_40",
        l1=8.0, l2=5.0, l3=40.0, l4=28.0,
        pitch_z_gain=1.6,
        servo1_baseline=55.0,
        servo1_gain_up=-0.8,
        servo1_gain_down=-1.65,
        servo2_baseline=50.0,
        servo2_yaw_gain=0.2,
        position_pitch_sign=1,
        pitch_pos_fade_start_yaw_deg=60.0,
        pitch_pos_fade_end_yaw_deg=80.0,
        yaw_calib=PNP_YAW_CALIB_L3_40,
        pitch_calib=PNP_PITCH_CALIB_L3_40,
    ),
    ArmKinematicsProfile(
        name="far_l3_55",
        l1=9.2333333333, l2=5.7666666667, l3=55.0, l4=28.0,
        pitch_z_gain=1.6,
        servo1_baseline=60.0,
        servo1_gain_up=-0.5,
        servo1_gain_down=-1.65,
        servo2_baseline=50.0,
        servo2_yaw_gain=0.2,
        position_pitch_sign=1,
        pitch_pos_fade_start_yaw_deg=45.0,
        pitch_pos_fade_end_yaw_deg=60.0,
        yaw_calib=PNP_YAW_CALIB_L3_55,
        pitch_calib=PNP_PITCH_CALIB_L3_55,
    ),
    ArmKinematicsProfile(
        name="extra_far_l3_65",
        l1=9.2333333333, l2=5.7666666667, l3=65.0, l4=28.0,
        pitch_z_gain=1.6,
        servo1_baseline=65.0,
        servo1_gain_up=-0.5,
        servo1_gain_down=-1.65,
        servo2_baseline=50.0,
        servo2_yaw_gain=0.2,
        position_pitch_sign=1,
        pitch_pos_fade_start_yaw_deg=45.0,
        pitch_pos_fade_end_yaw_deg=60.0,
        yaw_calib=PNP_YAW_CALIB_L3_55,
        pitch_calib=PNP_PITCH_CALIB_L3_55,
    ),
]


YAW_AXIS = np.array([-0.033060, +0.019434, -0.999264], dtype=np.float32)
YAW_AXIS /= np.linalg.norm(YAW_AXIS)
PITCH_AXIS = np.array([+0.686274, +0.727293, -0.008561], dtype=np.float32)
PITCH_AXIS /= np.linalg.norm(PITCH_AXIS)

# False pitch measured while turning the head horizontally with the current
# projection axes. Values outside the measured yaw range are clamped.
PITCH_BIAS_BY_YAW = [
    (-53.646, +6.517),
    (-35.332, +4.707),
    (-18.719, +1.818),
    (0.000, 0.000),
    (+28.276, -1.214),
    (+46.974, -1.611),
]


# ---------------------------------------------------------------------------
# Small math helpers
# ---------------------------------------------------------------------------

def normalize_angle_deg(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg < -180.0:
        deg += 360.0
    return deg


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def clamp01(v: float) -> float:
    return clamp(v, 0.0, 1.0)


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


def eulerZYXToMat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """ZYX Euler (intrinsic roll->pitch->yaw) -> 3x3 rotation matrix."""
    r, p, y = map(deg2rad, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [     -sp,                  cp * sr,                  cp * cr],
    ], dtype=np.float32)


def quatToMat(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if n < 1e-6:
        return np.eye(3, dtype=np.float32)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    return np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
    ], dtype=np.float32)


def matToEulerZYX(R: np.ndarray) -> Tuple[float, float, float]:
    """Return (roll_deg, pitch_deg, yaw_deg) from a ZYX rotation matrix."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        yaw = math.atan2(R[1, 0], R[0, 0])
        pitch = math.atan2(-R[2, 0], sy)
        roll = math.atan2(R[2, 1], R[2, 2])
    else:
        yaw = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        roll = 0.0
    return rad2deg(roll), rad2deg(pitch), rad2deg(yaw)


def _parse_nrf24_11b_frame(frame: bytes) -> Tuple[bool, bool, Dict[str, Any]]:
    """Parse one 11-byte NRF24 IMU frame.

    Returns:
        (valid, is_orientation, data).
        * valid: True if header and checksum are OK and the type is known.
        * is_orientation: True for type 0x59 (orientation), False for 0x52 (gyro).
        * data: parsed fields for that frame type.
    """
    if len(frame) != 11 or frame[0] != 0x55:
        return False, False, {}
    checksum = sum(frame[:10]) & 0xFF
    if checksum != frame[10]:
        return False, False, {}

    data: Dict[str, Any] = {}
    if frame[1] == 0x59:
        q0, q1, q2, q3 = struct.unpack("<hhhh", frame[2:10])
        qw = q0 / 32768.0
        qx = q1 / 32768.0
        qy = q2 / 32768.0
        qz = q3 / 32768.0
        roll, pitch, yaw = matToEulerZYX(quatToMat(qw, qx, qy, qz))
        data.update(
            roll=roll, pitch=pitch, yaw=yaw,
            qw=qw, qx=qx, qy=qy, qz=qz,
        )
        return True, True, data
    if frame[1] == 0x52:
        wx, wy, wz = struct.unpack("<hhh", frame[2:8])
        data.update(
            wx=wx / 32768.0 * 2000.0,
            wy=wy / 32768.0 * 2000.0,
            wz=wz / 32768.0 * 2000.0,
        )
        return True, False, data
    return False, False, {}


def parse_nrf24_imu_payload(payload: bytes) -> Dict[str, Any]:
    """Parse a 22-byte NRF24 head-IMU payload.

    The payload is two 11-byte frames.  Each frame has:
      byte 0   : header 0x55
      byte 1   : type 0x59 (orientation/quaternion) or 0x52 (gyro)
      bytes 2-9: four int16 little-endian values
      byte 10  : checksum (low byte of sum of bytes 0..9)

    Orientation frame (0x59): q0..q3 scaled by 1/32768, normalized, and
    converted to ZYX Euler angles (roll, pitch, yaw) in degrees.

    Gyro frame (0x52): x/y/z scaled by 2000/32768 deg/s.

    This mirrors nrf24_linux.c::nrf24_parse_22b().
    """
    result: Dict[str, Any] = {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
        "wx": 0.0, "wy": 0.0, "wz": 0.0,
        "imu_valid": False,
        "quat_valid": False,
        "raw": payload,
    }
    if len(payload) != 22:
        return result

    got_orientation = False
    got_gyro = False
    for offset in (0, 11):
        valid, is_orientation, data = _parse_nrf24_11b_frame(
            payload[offset:offset + 11]
        )
        if not valid:
            continue
        if is_orientation:
            got_orientation = True
        else:
            got_gyro = True
        result.update(data)

    result["imu_valid"] = got_orientation
    result["quat_valid"] = got_orientation
    return result


def matToRotVecDeg(R: np.ndarray) -> np.ndarray:
    """Rotation vector representation (axis * angle_deg) of a matrix."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    c = clamp((trace - 1.0) * 0.5, -1.0, 1.0)
    angle = math.acos(c)
    if angle < 1e-5:
        return np.array([
            (R[2, 1] - R[1, 2]) * 0.5 * rad2deg(1.0),
            (R[0, 2] - R[2, 0]) * 0.5 * rad2deg(1.0),
            (R[1, 0] - R[0, 1]) * 0.5 * rad2deg(1.0),
        ], dtype=np.float32)
    s = 2.0 * math.sin(angle)
    if abs(s) < 1e-6:
        return np.zeros(3, dtype=np.float32)
    axis = np.array([
        (R[2, 1] - R[1, 2]) / s,
        (R[0, 2] - R[2, 0]) / s,
        (R[1, 0] - R[0, 1]) / s,
    ], dtype=np.float32)
    return axis * rad2deg(angle)


def axisProjectionYawPitch(R: np.ndarray) -> Tuple[float, float]:
    rv = matToRotVecDeg(R)
    return float(rv.dot(YAW_AXIS)), float(rv.dot(PITCH_AXIS))


def interp_table(table: List[Tuple[float, float]], x: float) -> float:
    xs = [p[0] for p in table]
    ys = [p[1] for p in table]
    if not xs:
        return 0.0
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(len(xs) - 1):
        if x <= xs[i + 1]:
            ratio = (x - xs[i]) / (xs[i + 1] - xs[i])
            return ys[i] + ratio * (ys[i + 1] - ys[i])
    return ys[-1]


def axisProjectionYawPitchCompensated(R: np.ndarray) -> Tuple[float, float]:
    yaw_deg, pitch_deg = axisProjectionYawPitch(R)
    pitch_bias_deg = interp_table(PITCH_BIAS_BY_YAW, yaw_deg)
    return yaw_deg, pitch_deg - pitch_bias_deg


# ---------------------------------------------------------------------------
# Motion FSM
# ---------------------------------------------------------------------------

class MotionContext:
    def __init__(self):
        self.reset()

    def reset(self):
        self.wz_first = 0.0
        self.wz_peak = 0.0
        self.wz_valley = 0.0
        self.wz_latest = 0.0
        self.wz_avg = 0.0
        self.has_crossed_zero = False
        self.initialized = False

    def update_from_hist(self, hist: List[float], count: int = 0, max_samples: int = 5):
        if not hist:
            self.initialized = False
            return
        if count <= 0 or count > len(hist):
            count = len(hist)
        start = max(0, count - max_samples)
        window = hist[start:count]
        n = len(window)
        if n == 0:
            self.initialized = False
            return
        self.wz_first = window[0]
        self.wz_peak = self.wz_valley = window[0]
        self.wz_latest = window[-1]
        self.has_crossed_zero = False
        s = 0.0
        for i, v in enumerate(window):
            self.wz_peak = max(self.wz_peak, v)
            self.wz_valley = min(self.wz_valley, v)
            s += v
            if i > 0 and window[i - 1] * v < 0.0:
                self.has_crossed_zero = True
        self.wz_avg = s / n
        self.initialized = True

    def is_stop(self) -> bool:
        return abs(self.wz_avg) < 5.0

    def accel_trend(self) -> bool:
        return (not self.has_crossed_zero) and abs(self.wz_latest) > abs(self.wz_first) + 10.0

    def decel_trend(self) -> bool:
        return (not self.has_crossed_zero) and abs(self.wz_latest) < abs(self.wz_first) - 10.0

    def steady_trend(self) -> bool:
        return (not self.has_crossed_zero) and abs(abs(self.wz_latest) - abs(self.wz_first)) <= 10.0


def next_motion_state(prev: int, ctx: MotionContext) -> int:
    is_stop = ctx.is_stop()
    acc = ctx.accel_trend()
    dec = ctx.decel_trend()
    steady = ctx.steady_trend()
    crossed = ctx.has_crossed_zero

    if prev == STATE_STOP:
        return STATE_STOP if is_stop else STATE_STOP_TO_ACCEL
    if prev == STATE_STOP_TO_ACCEL:
        if is_stop:
            return STATE_STOP
        if acc:
            return STATE_ACCEL
        if steady:
            return STATE_ACCEL_TO_CONST
        return STATE_ACCEL
    if prev == STATE_ACCEL:
        if is_stop:
            return STATE_STOP
        if crossed:
            return STATE_DECEL_STOP_TO_ACCEL
        if acc:
            return STATE_ACCEL
        if dec:
            return STATE_CONST_TO_DECEL
        if steady:
            return STATE_ACCEL_TO_CONST
        return STATE_ACCEL
    if prev == STATE_ACCEL_TO_CONST:
        if is_stop:
            return STATE_STOP
        if steady:
            return STATE_CONST_SPEED
        if dec:
            return STATE_CONST_TO_DECEL
        return STATE_CONST_SPEED
    if prev == STATE_CONST_SPEED:
        if is_stop:
            return STATE_STOP
        if dec:
            return STATE_CONST_TO_DECEL
        if acc:
            return STATE_ACCEL_TO_CONST
        return STATE_CONST_SPEED
    if prev == STATE_CONST_TO_DECEL:
        if is_stop:
            return STATE_STOP
        if dec:
            return STATE_DECEL_TO_STOP
        return STATE_DECEL_TO_STOP
    if prev == STATE_DECEL_TO_STOP:
        if is_stop:
            return STATE_STOP
        if (not is_stop) and acc:
            return STATE_DECEL_STOP_TO_ACCEL
        return STATE_DECEL_TO_STOP
    if prev == STATE_DECEL_STOP_TO_ACCEL:
        if is_stop:
            return STATE_STOP
        if acc:
            return STATE_ACCEL
        if dec:
            return STATE_DECEL_TO_STOP
        if steady:
            return STATE_ACCEL_TO_CONST
        return STATE_ACCEL
    return STATE_STOP


# ---------------------------------------------------------------------------
# Endpoint predictor
# ---------------------------------------------------------------------------

class EndpointPredictor:
    def __init__(self):
        self.pred_delta_yaw = 0.0
        self.pred_state = STATE_STOP
        self.valid = False

    def reset(self):
        self.pred_delta_yaw = 0.0
        self.valid = False

    def update(self, wz: float, state: int):
        abs_wz = abs(wz)
        if abs_wz < 20.0:
            dt = 0.50
        elif abs_wz < 60.0:
            dt = 0.35
        else:
            dt = 0.25
        k_table = {
            STATE_ACCEL: 0.35,
            STATE_ACCEL_TO_CONST: 0.70,
            STATE_CONST_SPEED: 0.30,
            STATE_CONST_TO_DECEL: 0.60,
            STATE_DECEL_TO_STOP: 0.25,
            STATE_STOP: 0.00,
            STATE_STOP_TO_ACCEL: 0.35,
            STATE_DECEL_STOP_TO_ACCEL: 0.45,
        }
        k = k_table.get(state, 0.40)
        self.pred_delta_yaw = wz * dt * k
        self.pred_state = state
        self.valid = (state != STATE_STOP)

    def get_target_yaw(self, current_yaw: float) -> float:
        return current_yaw + self.pred_delta_yaw


# ---------------------------------------------------------------------------
# Abstract sources / sinks
# ---------------------------------------------------------------------------

class Nrf24ImuSource(abc.ABC):
    """Source of head-worn IMU data (NRF24 link)."""

    A_INIT_SAMPLES = 5

    def __init__(self):
        self.r_init_set = False
        self.wait_a_init = False
        self._valid_count = 0
        self._init_lock = threading.Lock()
        self._init_accum: List[Dict[str, float]] = []

    @abc.abstractmethod
    def poll(self) -> Optional[Dict[str, Any]]:
        """Return latest head IMU dict or None if no new data."""
        raise NotImplementedError

    def start(self):
        pass

    def stop(self):
        pass

    def start_a_init(self):
        """Signal that the handshake wants to capture the A-init baseline.

        Matches Base main.cpp/handshake_thread setting g_wait_a_init = 1.
        """
        with self._init_lock:
            self.wait_a_init = True
            self._init_accum = []

    def _on_valid_frame(self, head_imu: Dict[str, Any]):
        """Default A-init: average 5 valid frames after start_a_init()."""
        self._valid_count += 1
        with self._init_lock:
            if self.r_init_set or not self.wait_a_init:
                return
            self._init_accum.append({
                "roll": float(head_imu.get("roll", 0.0)),
                "pitch": float(head_imu.get("pitch", 0.0)),
                "yaw": float(head_imu.get("yaw", 0.0)),
            })
            if len(self._init_accum) < self.A_INIT_SAMPLES:
                return
            avg_roll = sum(v["roll"] for v in self._init_accum) / len(self._init_accum)
            avg_pitch = sum(v["pitch"] for v in self._init_accum) / len(self._init_accum)
            avg_yaw = sum(v["yaw"] for v in self._init_accum) / len(self._init_accum)
            self.r_init_set = True
            self.wait_a_init = False
            self._init_accum = []
        print(f"[A-INIT] 5-frame avg captured: "
              f"roll={avg_roll:.2f} pitch={avg_pitch:.2f} yaw={avg_yaw:.2f}")


class Imu2Source(abc.ABC):
    """Source of on-board waist IMU2 data."""

    @abc.abstractmethod
    def poll(self) -> Dict[str, Any]:
        raise NotImplementedError

    def start(self):
        pass

    def stop(self):
        pass


class UartArmSink(abc.ABC):
    """UART sink for 11-byte arm target frames."""

    # FF verification frame sent after "init success" (matches main.cpp handshake).
    FF_VERIFY_FRAME = bytes([0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA])
    # Exit frame sent at shutdown (matches main.cpp cleanup).
    EXIT_FRAME = bytes([0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF])
    HOMING_BLOCK_S = 7.0

    def __init__(self):
        self.init_success = False
        self.arm_powered = False
        self.move_complete = True
        self.homing_done = False
        self.block_tx = False
        self._handshake_done = False
        self._handshake_lock = threading.Lock()

    @abc.abstractmethod
    def send_arm_target(self, x: float, y: float, z: float,
                        k1: float, k2: float, flag: int) -> bool:
        """Send 11-byte raw frame. k1=J5 yaw, k2=J4 pitch."""
        raise NotImplementedError

    def start(self):
        pass

    def stop(self):
        pass

    def send_power_on(self) -> bool:
        return False

    def send_power_off(self) -> bool:
        return False

    def _trigger_homing_block(self):
        """Set block_tx for the homing period. ElfControlThread will clear it."""
        with self._handshake_lock:
            if self._handshake_done:
                return
            self._handshake_done = True
        print(f"[UART-Handshake] Blocking TX for {self.HOMING_BLOCK_S}s homing + A-init")
        self.block_tx = True


# ---------------------------------------------------------------------------
# Stub sources / sink (default, runs on x86 without hardware)
# ---------------------------------------------------------------------------

class StubNrf24ImuSource(Nrf24ImuSource):
    """
    Generates a synthetic head motion so the control chain produces commands.
    The motion is a slow sinusoid in yaw and pitch with some noise.
    """

    def __init__(self, freq: float = 0.25, yaw_amp: float = 25.0,
                 pitch_amp: float = 15.0, noise: float = 0.2):
        super().__init__()
        self._start = time.perf_counter()
        self._freq = freq
        self._yaw_amp = yaw_amp
        self._pitch_amp = pitch_amp
        self._noise = noise
        self._last_wy = 0.0
        self._last_wz = 0.0
        self._wy_hist: List[float] = []
        self._wz_hist: List[float] = []
        self._roll_hist: List[float] = []
        self._pitch_hist: List[float] = []
        self._yaw_hist: List[float] = []

    def poll(self) -> Optional[Dict[str, Any]]:
        t = time.perf_counter() - self._start
        # Smooth sinusoidal motion
        yaw = self._yaw_amp * math.sin(2.0 * math.pi * self._freq * t)
        pitch = self._pitch_amp * math.sin(2.0 * math.pi * self._freq * t + 1.0)
        roll = 2.0 * math.sin(2.0 * math.pi * self._freq * t * 0.7)

        # Derivatives as gyro
        wz = self._yaw_amp * (2.0 * math.pi * self._freq) * math.cos(2.0 * math.pi * self._freq * t)
        wy = self._pitch_amp * (2.0 * math.pi * self._freq) * math.cos(2.0 * math.pi * self._freq * t + 1.0)
        wx = 0.5

        # Add noise
        yaw += (np.random.rand() - 0.5) * self._noise
        pitch += (np.random.rand() - 0.5) * self._noise
        roll += (np.random.rand() - 0.5) * self._noise

        # Update histories (simulate 100 Hz samples)
        self._wy_hist.append(wy)
        self._wz_hist.append(wz)
        self._roll_hist.append(roll)
        self._pitch_hist.append(pitch)
        self._yaw_hist.append(yaw)
        if len(self._wy_hist) > NRF24_WY_HIST_SIZE:
            self._wy_hist.pop(0)
        if len(self._wz_hist) > NRF24_WZ_HIST_SIZE:
            self._wz_hist.pop(0)
        if len(self._roll_hist) > NRF24_ANGLE_HIST_SIZE:
            self._roll_hist.pop(0)
        if len(self._pitch_hist) > NRF24_ANGLE_HIST_SIZE:
            self._pitch_hist.pop(0)
        if len(self._yaw_hist) > NRF24_ANGLE_HIST_SIZE:
            self._yaw_hist.pop(0)

        # Convert Euler to a simple quaternion for completeness
        R = eulerZYXToMat(roll, pitch, yaw)
        qw, qx, qy, qz = rotation_matrix_to_quaternion(R)

        data = {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "qw": qw, "qx": qx, "qy": qy, "qz": qz,
            "wx": wx, "wy": wy, "wz": wz,
            "quat_valid": True,
            "imu_valid": True,
            "wy_hist": list(self._wy_hist),
            "wz_hist": list(self._wz_hist),
            "roll_hist": list(self._roll_hist),
            "pitch_hist": list(self._pitch_hist),
            "yaw_hist": list(self._yaw_hist),
        }
        self._on_valid_frame(data)
        return data


class StubImu2Source(Imu2Source):
    """Returns a static valid waist IMU2 (used for dual-IMU path testing)."""

    def poll(self) -> Dict[str, Any]:
        return {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "valid": True}


class StubUartArmSink(UartArmSink):
    """Prints the frame to stdout and simulates a successful handshake."""

    def __init__(self):
        super().__init__()
        self.init_success = True
        self.arm_powered = True
        self.move_complete = True
        self.homing_done = True
        self.block_tx = False
        self._sent_count = 0

    def send_arm_target(self, x: float, y: float, z: float,
                        k1: float, k2: float, flag: int) -> bool:
        if self.block_tx:
            return False
        if not self.arm_powered:
            return False
        self._sent_count += 1
        flag_s = "PRED" if flag == 0x00 else "CONF"
        print(
            f"[UART-TX] x={x:+.1f} y={y:+.1f} z={z:+.1f} "
            f"k1={k1:+.1f} k2={k2:+.1f} flag=0x{flag:02X}({flag_s})"
        )
        # Simulate asynchronous move_complete toggle
        self.move_complete = False
        threading.Timer(0.08, lambda: setattr(self, "move_complete", True)).start()
        return True

    def send_power_on(self) -> bool:
        if self.arm_powered:
            return False
        self.arm_powered = True
        self.init_success = True
        print("[UART-TX] power on: FF AA ...")
        return True

    def send_power_off(self) -> bool:
        if not self.arm_powered:
            return False
        self.arm_powered = False
        self.block_tx = True
        print("[UART-TX] power off: AA FF ...")
        return True


def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to (qw, qx, qy, qz)."""
    trace = np.trace(R)
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qw, qx, qy, qz


# ---------------------------------------------------------------------------
# Real hardware backends (placeholders / optional dependencies)
# ---------------------------------------------------------------------------

class SerialUartArmSink(UartArmSink):
    """
    Real UART sink using pyserial.
    Receives 'init success' / 'move_success' strings from the STM32.
    """

    def __init__(self, port: str = "/dev/ttyS9", baudrate: int = 115200,
                 init_timeout: float = 10.0, block_tx_ms: int = 0,
                 send_ff_frame: bool = True):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.init_timeout = init_timeout
        self.block_tx_ms = block_tx_ms
        self.send_ff_frame = send_ff_frame
        self._serial = None
        self._rx_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()

    def start(self):
        try:
            import serial
        except ImportError as e:
            raise RuntimeError("SerialUartArmSink requires 'pyserial' package") from e
        self._serial = serial.Serial(
            self.port, self.baudrate,
            bytesize=8, parity="N", stopbits=1,
            timeout=0.1, write_timeout=1.0,
        )
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        print(f"[UART] Opened {self.port} @ {self.baudrate}, waiting for 'init success'...")
        deadline = time.perf_counter() + self.init_timeout
        while not self.init_success and time.perf_counter() < deadline:
            time.sleep(0.05)
        if self.init_success:
            print("[UART] 'init success' received")
        else:
            print("[UART] WARNING: init success not received")

    def stop(self):
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self._serial is not None and self._serial.is_open:
            self.send_power_off()
            time.sleep(0.05)
            self._serial.close()

    def _rx_loop(self):
        buf = b""
        while self._running:
            try:
                chunk = self._serial.read(64)
            except Exception:
                continue
            if not chunk:
                continue
            buf += chunk
            if b"init success" in buf:
                self.init_success = True
                buf = b""
            if b"move_success" in buf:
                if not self.homing_done:
                    self.homing_done = True
                self.move_complete = True
                buf = b""
            if len(buf) > 512:
                buf = buf[-256:]

    def _send_raw(self, data: bytes) -> bool:
        if self._serial is None or not self._serial.is_open:
            return False
        try:
            with self._lock:
                self._serial.write(data)
                self._serial.flush()
            return True
        except Exception as e:
            print(f"[UART] raw write failed: {e}")
            return False

    def send_arm_target(self, x: float, y: float, z: float,
                        k1: float, k2: float, flag: int) -> bool:
        if self._serial is None or not self._serial.is_open:
            return False
        if self.block_tx:
            return False
        if not self.arm_powered:
            return False
        data = struct.pack("<hhhhh", int(x), int(y), int(z), int(k1), int(k2))
        frame = data + bytes([flag])
        try:
            with self._lock:
                self._serial.write(frame)
                self._serial.flush()
            self.move_complete = False
            return True
        except Exception as e:
            print(f"[UART] write failed: {e}")
            return False

    def send_power_on(self) -> bool:
        if self.arm_powered:
            return False
        if not self.init_success:
            return False
        ok = self._send_raw(self.FF_VERIFY_FRAME)
        if ok:
            self.arm_powered = True
            self.init_success = True
        return ok

    def send_power_off(self) -> bool:
        if not self.arm_powered:
            return False
        ok = self._send_raw(self.EXIT_FRAME)
        if ok:
            self.arm_powered = False
            self.block_tx = True
        return ok


# ---------------------------------------------------------------------------
# HTTP control server
# ---------------------------------------------------------------------------

class ControlContext(abc.ABC):
    """Interface exposed by the control thread to the HTTP server."""

    @abc.abstractmethod
    def get_status(self) -> Dict[str, Any]:
        raise NotImplementedError

    @abc.abstractmethod
    def request_head_center(self):
        raise NotImplementedError

    @abc.abstractmethod
    def request_rebaseline(self):
        raise NotImplementedError

    @abc.abstractmethod
    def toggle_pitch_sign(self):
        raise NotImplementedError

    @abc.abstractmethod
    def set_arm_profile(self, idx: int, source: str = "local"):
        raise NotImplementedError

    @abc.abstractmethod
    def set_calib_mode(self, mode: int):
        raise NotImplementedError

    @abc.abstractmethod
    def power_on_arm(self) -> bool:
        raise NotImplementedError

    @abc.abstractmethod
    def request_voice_power_off(self, timeout_s: float = 3.0) -> bool:
        raise NotImplementedError

    @abc.abstractmethod
    def confirm_voice_power_off(self) -> bool:
        raise NotImplementedError

    @abc.abstractmethod
    def power_off_arm(self) -> bool:
        raise NotImplementedError

    def send_servo_test(self, k1: float, k2: float):
        raise NotImplementedError

    @abc.abstractmethod
    def set_mode(self, mode: str, source: str = "local"):
        raise NotImplementedError

    def unlock_remote_control(self):
        raise NotImplementedError

    def lock_remote_control(self):
        raise NotImplementedError

    @abc.abstractmethod
    def get_mode(self) -> str:
        raise NotImplementedError

    @abc.abstractmethod
    def switch_stream_mode(self, mode: str) -> Dict[str, Any]:
        raise NotImplementedError


def _parse_query(path: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    if "?" not in path:
        return out
    qs = path.split("?", 1)[1]
    for part in qs.split("&"):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k] = v
    return out


def _make_handler(context: ControlContext):
    class _Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # suppress default logging

        def _send_json(self, status: int, obj: Dict[str, Any]):
            body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Connection", "close")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _handle_stream_switch(self):
            mode = _parse_query(self.path).get("mode", "")
            if mode not in ("auto", "cloud", "local"):
                self._send_json(400, {"ok": False, "error": "mode must be auto, cloud or local"})
                return
            result = context.switch_stream_mode(mode)
            self._send_json(200 if result.get("ok") else 503, result)

        def do_GET(self):
            if self.path == "/status" or self.path.startswith("/status"):
                self._send_json(200, context.get_status())
            elif self.path.startswith("/stream"):
                self._handle_stream_switch()
            else:
                self._send_json(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            path = self.path
            q = _parse_query(path)
            if path.startswith("/stream"):
                self._handle_stream_switch()
            elif path.startswith("/mode"):
                t = q.get("type", "")
                if t in ("face", "body", "intro", "interview", "first_person"):
                    source = q.get("source", "local")
                    ok = context.set_mode(t, source=source)
                    self._send_json(200, {"ok": bool(ok), "mode": context.get_mode()})
                else:
                    self._send_json(400, {"ok": False, "error": "invalid type"})
            elif path.startswith("/profile"):
                try:
                    idx = int(q.get("idx", "-1"))
                except ValueError:
                    self._send_json(400, {"ok": False, "error": "idx must be int"})
                    return
                if idx < 0 or idx >= len(ARM_PROFILES):
                    self._send_json(400, {"ok": False, "error": "invalid profile idx"})
                    return
                source = q.get("source", "local")
                ok = context.set_arm_profile(idx, source=source)
                self._send_json(200, {"ok": bool(ok), "profile_idx": idx})
            elif path.startswith("/power"):
                action = q.get("action", "")
                if action == "on":
                    ok = context.power_on_arm()
                    self._send_json(200, {"ok": ok, "arm_powered": context.get_status().get("arm_powered", False)})
                elif action == "off":
                    ok = context.power_off_arm()
                    self._send_json(200, {"ok": ok, "arm_powered": context.get_status().get("arm_powered", False)})
                else:
                    self._send_json(400, {"ok": False, "error": "invalid action"})
            elif path.startswith("/calib"):
                if "mode" in q:
                    try:
                        mode = int(q["mode"])
                    except ValueError:
                        self._send_json(400, {"ok": False, "error": "mode must be int"})
                        return
                    if not 0 <= mode <= 5:
                        self._send_json(400, {"ok": False, "error": "mode must be 0..5"})
                        return
                    context.set_calib_mode(mode)
                    self._send_json(200, {"ok": True, "calib_mode": mode})
                else:
                    self._send_json(200, {"ok": True, "calib_mode": context.get_status().get("calib_mode", 0)})
            elif path.startswith("/servo"):
                try:
                    k1 = float(q.get("k1", "-1"))
                    k2 = float(q.get("k2", "-1"))
                except ValueError:
                    self._send_json(400, {"ok": False, "error": "invalid k1/k2"})
                    return
                if k1 >= 0 and k2 >= 0:
                    context.send_servo_test(k1, k2)
                    self._send_json(200, {"ok": True, "k1": k1, "k2": k2})
                else:
                    self._send_json(400, {"ok": False, "error": "missing k1 or k2"})
            elif path.startswith("/cmd"):
                action = q.get("action", "")
                source = q.get("source", "local")
                if action == "remote_unlock":
                    context.unlock_remote_control()
                    self._send_json(200, {"ok": True, "remote_locked": False})
                elif action == "remote_lock":
                    context.lock_remote_control()
                    self._send_json(200, {"ok": True, "remote_locked": True})
                elif action == "rebaseline":
                    context.request_rebaseline()
                    self._send_json(200, {"ok": True, "action": "rebaseline"})
                elif action == "head_center":
                    context.request_head_center()
                    self._send_json(200, {"ok": True, "action": "head_center"})
                elif action == "toggle_pitch_sign":
                    if source == "remote":
                        context.lock_remote_control()
                    context.toggle_pitch_sign()
                    self._send_json(200, {"ok": True, "action": "toggle_pitch_sign"})
                elif action == "nrf24_reset":
                    self._send_json(200, {"ok": True, "action": "nrf24_reset"})
                else:
                    self._send_json(400, {"ok": False, "error": "unknown action"})
            else:
                self._send_json(404, {"ok": False, "error": "not found"})

    return _Handler


class HttpControlServer:
    def __init__(self, context: ControlContext, port: int = 8080):
        self.context = context
        self.port = port
        self._server = None
        self._thread: Optional[threading.Thread] = None

    def start(self):
        handler = _make_handler(self.context)
        # Allow address reuse
        class _Server(socketserver.ThreadingTCPServer):
            allow_reuse_address = True
            daemon_threads = True

        self._server = _Server(("0.0.0.0", self.port), handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()
        print(f"[Ctrl] HTTP control server listening on port {self.port}")

    def stop(self):
        if self._server:
            self._server.shutdown()
            self._server.server_close()
        if self._thread:
            self._thread.join(timeout=2.0)


# ---------------------------------------------------------------------------
# Main controller
# ---------------------------------------------------------------------------

class Nrf24Controller:
    """
    Python port of rga_npu.cpp::nrf24_control_update().
    Runs the 50 Hz A-inverse / dual-IMU fusion, motion FSM, endpoint predictor,
    spherical IK, servo mapping and PnP drift correction.
    """

    def __init__(self, profile_idx: int = ARM_PROFILE_FAR_L3_55):
        self.profile_idx = profile_idx

        self.R_init: Optional[np.ndarray] = None
        self.R_imu2_init: Optional[np.ndarray] = None
        self.R_bias_total: Optional[np.ndarray] = None
        self.R_head_center_rel: Optional[np.ndarray] = None
        self.head_center_set = False
        self.head_center_request = True

        self.pitch_baseline_set = False
        self.yaw_baseline_set = False
        self.last_cmd_target_pitch = 0.0
        self.last_cmd_target_yaw = 0.0

        self.prev_state = STATE_STOP
        self.prev_state_yaw = STATE_STOP
        self.ctx = MotionContext()
        self.ctx_yaw = MotionContext()
        self.predictor = EndpointPredictor()
        self.predictor_yaw = EndpointPredictor()

        self.last_update_us = 0
        self.last_cmd_us = 0
        self.last_cmd_us_yaw = 0

        self.pitch_visual_bias_deg = 0.0
        self.effective_pitch_visual_bias_deg = 0.0
        self.last_is_stop_yaw_for_pitch_bias = True
        self.last_pitch_visual_bias_step = 0.0
        self.visual_pitch_pending_sign = 0
        self.visual_yaw_pending_sign = 0
        self.visual_pitch_last_cmd_sign = 0
        self.visual_yaw_last_cmd_sign = 0
        self.visual_pitch_blocked_sign = 0
        self.visual_yaw_blocked_sign = 0
        self.visual_pitch_reversal_pending = False
        self.visual_yaw_reversal_pending = False

        self.rel_rate_init = False
        self.prev_vec_pitch = 0.0
        self.prev_vec_yaw = 0.0
        self.prev_rel_rate_us = 0
        self.rel_wy_hist = [0.0] * NRF24_WY_HIST_SIZE
        self.rel_wz_hist = [0.0] * NRF24_WZ_HIST_SIZE
        self.rel_wy_idx = 0
        self.rel_wz_idx = 0
        self.rel_wy_count = 0
        self.rel_wz_count = 0

        self.prev_sent_tx = 0.0
        self.prev_sent_ty = 0.0
        self.prev_sent_tz = 0.0
        self.prev_sent_initialized = False

        self.was_stationary = False
        self.stationary_since_us = 0
        self.last_any_cmd_us = 0
        self.is_stationary_now = False

        self.head_stationary = 0
        self.arm_stable = 0

        self.head_pitch_control_sign = 1
        self.calib_mode = 0
        self.pose_mode = "face"  # face | body | intro | interview | first_person

        self.arm_target_yaw = 0.0
        self.arm_target_pitch = 0.0
        self._scenario_reset = {"intro": True, "interview": True}

        # Scenario mode state
        self._intro_state: Dict[str, Any] = {
            "first": True,
            "target_x": 0.0, "target_y": 0.0,
            "intro_space_x_cm": INTRO_BASE_X_CM,
            "intro_servo_yaw_deg": 0.0,
            "intro_yaw_hold": False,
            "hand_center_since_us": 0,
            "space_center_last_step_us": 0,
            "space_present_last_step_us": 0,
            "moving_gain_active_us": 0,
            "last_send_us": 0,
            "last_sent_j5": 0.0,
            "last_err_norm": 0.0,
        }
        self._interview_state: Dict[str, Any] = {
            "first": True,
            "target_x": 0.0, "target_y": 0.0,
            "interview_servo_yaw_deg": 0.0,
            "interview_yaw_hold": False,
            "last_send_us": 0,
            "last_sent_j5": 0.0,
            "last_source": 0,
        }

        self._first_person_state: Dict[str, Any] = {
            "first": True,
            "last_send_us": 0,
            "last_j4": FIRST_PERSON_BASE_J4_DEG,
            "last_j5": FIRST_PERSON_BASE_J5_DEG,
            "target_x": FIRST_PERSON_BASE_X_CM,
            "target_y": FIRST_PERSON_BASE_Y_CM,
            "target_z": FIRST_PERSON_BASE_Z_CM,
            "head_pitch": 0.0,
        }

        # Calibration state
        self._calib_state: Dict[str, Any] = {
            "last_cmd_us": 0,
            "locked_roll": 0.0,
            "locked_yaw": 0.0,
            "locked_tx": 0.0,
            "locked_ty": 0.0,
            "locked_tz": 0.0,
            "servo1": 50.0,
            "scan_angle": 100.0,
            "scan_dir": 1,
            "pnp_target_idx": -1,
            "pnp_phase": 0,  # 0=inactive, 1=prepare, 2=sample, 3=complete
            "pnp_phase_start_us": 0,
            "pnp_last_sent_idx": -1,
            "grid_target_idx": -1,
            "grid_phase": 0,
            "grid_phase_start_us": 0,
            "grid_accum": None,
        }

        self._pnp_lock = threading.Lock()
        self._pnp = {
            "yaw_correction": 0.0,
            "pitch_correction": 0.0,
            "valid": False,
            "ready": False,
        }

    @property
    def profile(self) -> ArmKinematicsProfile:
        idx = clamp(self.profile_idx, 0, len(ARM_PROFILES) - 1)
        return ARM_PROFILES[idx]

    def set_arm_profile(self, idx: int):
        self.profile_idx = clamp(idx, 0, len(ARM_PROFILES) - 1)
        profile = self.profile
        print(f"[ArmProfile] Switched to {profile.name} "
              f"(pitch_pos_fade={profile.pitch_pos_fade_start_yaw_deg:.0f}.."
              f"{profile.pitch_pos_fade_end_yaw_deg:.0f} deg)")

    def set_calib_mode(self, mode: int):
        self.calib_mode = clamp(mode, 0, 5)
        print(f"[Ctrl] Calibration mode set to {self.calib_mode}")

    # ------------------------------------------------------------------
    # Calibration helpers
    # ------------------------------------------------------------------
    def _read_servo_calib_file(self) -> float:
        try:
            with open("/tmp/servo_calib.txt", "r") as f:
                return float(f.read().strip())
        except Exception:
            return 50.0

    def _lock_calib_position(self, roll: float, yaw: float):
        st = self._calib_state
        st["locked_roll"] = roll
        st["locked_yaw"] = yaw
        profile = self.profile
        cum_pitch = deg2rad(roll)
        st["locked_tz"] = 30.0 + 15.0 * math.sin(cum_pitch)
        st["locked_tx"] = 0.0
        st["locked_ty"] = 10.0 + 57.0

    def _run_servo_calib(self, now_us: int, head_imu: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        st = self._calib_state
        if not st.get("calib_active"):
            st["calib_active"] = True
            self._lock_calib_position(
                head_imu.get("roll", 0.0), head_imu.get("yaw", 0.0)
            )
            print(f"[Servo-Calib] Servo/scan mode active: locked tx={st['locked_tx']:.1f} "
                  f"ty={st['locked_ty']:.1f} tz={st['locked_tz']:.1f}")

        # NOTE: Upstream Base (rga_npu.cpp) computes calib_servo1 from the file
        # or a scan angle, but then sends fixed k1=50/k2=145 regardless.  Target
        # intentionally uses the adjustable/scan value so that /servo?k1=...
        # and scan-test mode actually move the arm.  The difference is documented
        # in docs/PORTING_AUDIT_REPORT.md.
        if self.calib_mode == 1:
            st["servo1"] = self._read_servo_calib_file()
        else:  # mode 2: scan test
            st["scan_angle"] += st["scan_dir"] * 10.0
            if st["scan_angle"] >= 180.0:
                st["scan_angle"] = 180.0
                st["scan_dir"] = -1
            if st["scan_angle"] <= 100.0:
                st["scan_angle"] = 100.0
                st["scan_dir"] = 1
            st["servo1"] = st["scan_angle"]

        if now_us - st["last_cmd_us"] > 200_000:
            st["last_cmd_us"] = now_us
            self.arm_target_yaw = 0.0
            self.arm_target_pitch = 0.0
            return {
                "x": st["locked_tx"], "y": st["locked_ty"], "z": st["locked_tz"],
                "servo1": st["servo1"], "servo2": 50.0, "flag": 0x01,
            }
        return None

    def _run_pnp_yaw_calib(self, now_us: int) -> Optional[Dict[str, Any]]:
        return self._run_pnp_calib(
            now_us, PNP_CALIB_TARGETS_YAW,
            lambda t, p: self._pnp_yaw_target_kinematics(t, p),
            "/tmp/pnp_yaw_calib.csv",
        )

    def _run_pnp_pitch_calib(self, now_us: int) -> Optional[Dict[str, Any]]:
        return self._run_pnp_calib(
            now_us, PNP_CALIB_TARGETS_PITCH,
            lambda t, p: self._pnp_pitch_target_kinematics(t, p),
            "/tmp/pnp_pitch_calib.csv",
        )

    def _pnp_yaw_target_kinematics(self, target_deg: float, profile: ArmKinematicsProfile):
        yaw_rad = -deg2rad(target_deg)
        tx = profile.l3 * math.sin(yaw_rad)
        ty = profile.l4 + profile.l3 * math.cos(yaw_rad)
        tz = profile.l2 + profile.l1
        servo2 = profile.servo2_baseline + profile.servo2_yaw_gain * target_deg
        servo1 = profile.servo1_baseline
        return tx, ty, tz, servo2, servo1

    def _pnp_pitch_target_kinematics(self, target_deg: float, profile: ArmKinematicsProfile):
        pitch_rad = deg2rad(target_deg)
        tx = 0.0
        ty = profile.l4 - profile.l1 * math.sin(pitch_rad) + profile.l3 * math.cos(pitch_rad)
        tz = (profile.l2 +
              profile.l1 * math.cos(pitch_rad * profile.pitch_z_gain) +
              profile.l3 * math.sin(pitch_rad * profile.pitch_z_gain))
        gain = profile.servo1_gain_up if target_deg >= 0 else profile.servo1_gain_down
        servo1 = profile.servo1_baseline + gain * target_deg
        servo2 = profile.servo2_baseline
        return tx, ty, tz, servo2, servo1

    def _run_pnp_calib(self, now_us: int, targets: List[int],
                       kinematics, csv_path: str) -> Optional[Dict[str, Any]]:
        st = self._calib_state
        profile = self.profile
        idx = st["pnp_target_idx"]
        phase = st["pnp_phase"]
        is_yaw = csv_path.endswith("yaw_calib.csv")
        key = "pnp_yaw_accum" if is_yaw else "pnp_pitch_accum"

        # A completed calibration remains stopped until mode 0 resets it.
        if phase == 3:
            return None

        if idx < 0:
            idx = 0
            phase = 1
            st["pnp_target_idx"] = idx
            st["pnp_phase"] = phase
            st["pnp_phase_start_us"] = now_us
            st["pnp_last_sent_idx"] = -1
            st[key] = []
            print(f"[PnP-Calib] Moving arm through {'yaw' if is_yaw else 'pitch'} targets, "
                  f"{len(targets)} points")

        target_deg = targets[idx]
        elapsed = now_us - st["pnp_phase_start_us"]

        if phase == 1 and elapsed >= PNP_CALIB_PREPARE_US:
            phase = 2
            st["pnp_phase"] = phase
            st["pnp_phase_start_us"] = now_us
            st[key] = []
            print(f"[PnP-Calib] Target {target_deg:+d}: sampling 8s, face the camera")
        elif phase == 2 and elapsed >= PNP_CALIB_SAMPLE_US:
            accum = st.get(key, [])
            avg = self._avg(accum) if accum else 0.0
            self._write_pnp_csv(csv_path, target_deg, avg, len(accum), is_yaw)
            idx += 1
            if idx >= len(targets):
                st["pnp_phase"] = 3
                st["pnp_target_idx"] = len(targets)
                print("[PnP-Calib] All targets complete")
                return None
            phase = 1
            st["pnp_phase"] = phase
            st["pnp_target_idx"] = idx
            st["pnp_phase_start_us"] = now_us
            print(f"[PnP-Calib] next target {targets[idx]:+d} PREPARE")

        # Send exactly once on entry to each target; repeated sends restart the
        # lower controller speed plan and can make the physical arm oscillate.
        if st.get("pnp_last_sent_idx", -1) == idx:
            return None
        st["pnp_last_sent_idx"] = idx
        target_deg = targets[idx]
        tx, ty, tz, servo2, servo1 = kinematics(target_deg, profile)
        self.arm_target_yaw = target_deg if is_yaw else 0.0
        self.arm_target_pitch = target_deg if not is_yaw else 0.0
        return {
            "x": tx, "y": ty, "z": tz,
            "servo1": servo1, "servo2": servo2, "flag": 0x01,
        }

    def put_pnp_calib_sample(self, yaw_deg: float, pitch_deg: float):
        """Called by the visual thread during PnP calibration SAMPLE phase."""
        st = self._calib_state
        if st.get("pnp_phase") != 2:
            return
        if "pnp_yaw_accum" in st:
            st["pnp_yaw_accum"].append(yaw_deg)
        if "pnp_pitch_accum" in st:
            st["pnp_pitch_accum"].append(pitch_deg)

    @staticmethod
    def _write_pnp_csv(csv_path: str, target_deg: float, avg_deg: float,
                       sample_count: int, is_yaw: bool):
        header = "target_yaw_deg,pnp_yaw_avg_deg,sample_count\n" if is_yaw else "target_pitch_deg,pnp_pitch_avg_deg,sample_count\n"
        write_header = not os.path.exists(csv_path)
        try:
            with open(csv_path, "a") as f:
                if write_header:
                    f.write(header)
                f.write(f"{target_deg:.1f},{avg_deg:.4f},{sample_count}\n")
        except Exception as e:
            print(f"[PnP-Calib] Failed to write {csv_path}: {e}")

    def _run_head_imu_grid_calib(self, now_us: int, head_imu: Dict[str, Any],
                                  imu2: Dict[str, Any],
                                  r_init_set: bool) -> Optional[Dict[str, Any]]:
        st = self._calib_state
        targets = HEAD_IMU_GRID_TARGETS
        idx = st["grid_target_idx"]
        phase = st["grid_phase"]

        if idx < 0:
            idx = 0
            phase = 1
            st["grid_target_idx"] = idx
            st["grid_phase"] = phase
            st["grid_phase_start_us"] = now_us
            st["grid_accum"] = {
                "head_roll": [], "head_pitch": [], "head_yaw": [],
                "head_wx": [], "head_wy": [], "head_wz": [],
                "waist_valid": [], "waist_roll": [], "waist_pitch": [], "waist_yaw": [],
                "rel_roll": [], "rel_pitch": [], "rel_yaw": [],
            }
            print(f"[Head-IMU-Calib] 3x3 grid started, output=/tmp/head_imu_grid_calib.csv")
            return None

        if phase == 3:
            return None

        name, target_yaw, target_pitch = targets[idx]
        elapsed = now_us - st["grid_phase_start_us"]

        # Accumulate samples during sample phase.
        if phase == 2 and head_imu.get("imu_valid") and r_init_set and self.R_init is not None:
            acc = st["grid_accum"]
            acc["head_roll"].append(head_imu.get("roll", 0.0))
            acc["head_pitch"].append(head_imu.get("pitch", 0.0))
            acc["head_yaw"].append(head_imu.get("yaw", 0.0))
            acc["head_wx"].append(head_imu.get("wx", 0.0))
            acc["head_wy"].append(head_imu.get("wy", 0.0))
            acc["head_wz"].append(head_imu.get("wz", 0.0))
            imu2_valid = imu2.get("valid", False)
            acc["waist_valid"].append(1 if imu2_valid else 0)
            acc["waist_roll"].append(imu2.get("roll", 0.0))
            acc["waist_pitch"].append(imu2.get("pitch", 0.0))
            acc["waist_yaw"].append(imu2.get("yaw", 0.0))

            cr, cp, cy = head_imu.get("roll", 0.0), head_imu.get("pitch", 0.0), head_imu.get("yaw", 0.0)
            qw, qx, qy, qz = head_imu.get("qw", 1.0), head_imu.get("qx", 0.0), head_imu.get("qy", 0.0), head_imu.get("qz", 0.0)
            quat_valid = head_imu.get("quat_valid", False)
            R_head = quatToMat(qw, qx, qy, qz) if quat_valid else eulerZYXToMat(cr, cp, cy)
            R_head_delta = R_head @ self.R_init.T
            R_rel = R_head_delta
            if imu2_valid and self.R_imu2_init is not None:
                R_imu2 = eulerZYXToMat(imu2["roll"], imu2["pitch"], imu2["yaw"])
                R_imu2_delta = R_imu2 @ self.R_imu2_init.T
                R_rel = R_imu2_delta.T @ R_head_delta
            rel_roll, rel_pitch, rel_yaw = matToEulerZYX(R_rel)
            acc["rel_roll"].append(rel_roll)
            acc["rel_pitch"].append(rel_pitch)
            acc["rel_yaw"].append(rel_yaw)

        if phase == 1 and elapsed >= PNP_CALIB_PREPARE_US:
            phase = 2
            st["grid_phase"] = phase
            st["grid_phase_start_us"] = now_us
            print(f"[Head-IMU-Calib] Target {idx+1}/{len(targets)} {name} "
                  f"yaw={target_yaw:+.1f} pitch={target_pitch:+.1f} -> entering SAMPLE")
        elif phase == 2 and elapsed >= PNP_CALIB_SAMPLE_US:
            self._write_head_imu_grid_csv(st["grid_accum"], targets[idx])
            idx += 1
            if idx >= len(targets):
                phase = 3
                st["grid_phase"] = phase
                st["grid_target_idx"] = -1
                print("[Head-IMU-Calib] Complete: /tmp/head_imu_grid_calib.csv")
            else:
                phase = 1
                st["grid_phase"] = phase
                st["grid_target_idx"] = idx
                st["grid_phase_start_us"] = now_us
                st["grid_accum"] = {
                    "head_roll": [], "head_pitch": [], "head_yaw": [],
                    "head_wx": [], "head_wy": [], "head_wz": [],
                    "waist_valid": [], "waist_roll": [], "waist_pitch": [], "waist_yaw": [],
                    "rel_roll": [], "rel_pitch": [], "rel_yaw": [],
                }
                print(f"[Head-IMU-Calib] next grid target {targets[idx][0]}")
        return None

    @staticmethod
    def _avg(seq: List[float]) -> float:
        return sum(seq) / len(seq) if seq else 0.0

    def _write_head_imu_grid_csv(self, acc: Dict[str, Any], target: Tuple[str, float, float]):
        name, target_yaw, target_pitch = target
        path = "/tmp/head_imu_grid_calib.csv"
        header = (
            "target_name,target_yaw_deg,target_pitch_deg,sample_count,"
            "head_roll_avg,head_pitch_avg,head_yaw_avg,"
            "head_wx_avg,head_wy_avg,head_wz_avg,"
            "waist_valid,waist_roll_avg,waist_pitch_avg,waist_yaw_avg,"
            "rel_roll_avg,rel_pitch_avg,rel_yaw_avg\n"
        )
        write_header = not os.path.exists(path)
        try:
            with open(path, "a") as f:
                if write_header:
                    f.write(header)
                sample_count = len(acc["head_roll"])
                waist_valid_sum = sum(acc["waist_valid"])
                waist_valid = 1 if waist_valid_sum > sample_count // 2 else 0
                f.write(
                    f"{name},{target_yaw:.1f},{target_pitch:.1f},{sample_count},"
                    f"{self._avg(acc['head_roll']):.4f},"
                    f"{self._avg(acc['head_pitch']):.4f},"
                    f"{self._avg(acc['head_yaw']):.4f},"
                    f"{self._avg(acc['head_wx']):.4f},"
                    f"{self._avg(acc['head_wy']):.4f},"
                    f"{self._avg(acc['head_wz']):.4f},"
                    f"{waist_valid},"
                    f"{self._avg(acc['waist_roll']):.4f},"
                    f"{self._avg(acc['waist_pitch']):.4f},"
                    f"{self._avg(acc['waist_yaw']):.4f},"
                    f"{self._avg(acc['rel_roll']):.4f},"
                    f"{self._avg(acc['rel_pitch']):.4f},"
                    f"{self._avg(acc['rel_yaw']):.4f}\n"
                )
        except Exception as e:
            print(f"[Head-IMU-Calib] Failed to write {path}: {e}")

    def _run_calibration(self, now_us: int, head_imu: Dict[str, Any],
                         imu2: Dict[str, Any], r_init_set: bool) -> Optional[Dict[str, Any]]:
        mode = self.calib_mode
        if mode == 0:
            # Reset calibration state when leaving calibration mode.
            if self._calib_state.get("calib_active"):
                self._calib_state["calib_active"] = False
            if self._calib_state.get("pnp_target_idx", -1) >= 0:
                self._calib_state["pnp_target_idx"] = -1
                self._calib_state["pnp_phase"] = 0
                self._calib_state["pnp_last_sent_idx"] = -1
            if self._calib_state.get("grid_target_idx", -1) >= 0:
                self._calib_state["grid_target_idx"] = -1
                self._calib_state["grid_phase"] = 0
            return None
        if mode in (1, 2):
            return self._run_servo_calib(now_us, head_imu)
        if mode == 3:
            return self._run_pnp_yaw_calib(now_us)
        if mode == 4:
            return self._run_pnp_pitch_calib(now_us)
        if mode == 5:
            return self._run_head_imu_grid_calib(now_us, head_imu, imu2, r_init_set)
        return None

    # ------------------------------------------------------------------

    def set_first_person_target(self, x: float, y: float, z: float) -> Dict[str, Any]:
        """Set persistent first-person camera-space coordinates in cm."""
        st = self._first_person_state
        st["target_x"] = clamp(float(x), FIRST_PERSON_MIN_X_CM, FIRST_PERSON_MAX_X_CM)
        st["target_y"] = clamp(float(y), FIRST_PERSON_MIN_Y_CM, FIRST_PERSON_MAX_Y_CM)
        st["target_z"] = clamp(float(z), FIRST_PERSON_MIN_Z_CM, FIRST_PERSON_MAX_Z_CM)
        j4_base = FIRST_PERSON_BASE_J4_DEG + FIRST_PERSON_J4_DEG_PER_Z_CM * (st["target_z"] - FIRST_PERSON_BASE_Z_CM)
        pitch_control_deg = self.head_pitch_control_sign * st.get("head_pitch", 0.0)
        st["last_j4"] = clamp(j4_base - FIRST_PERSON_PITCH_GAIN * pitch_control_deg, -90.0, 90.0)
        print(
            f"[FirstPerson] target xyz set: x={st['target_x']:.1f} "
            f"y={st['target_y']:.1f} z={st['target_z']:.1f}"
        )
        return self._make_first_person_current_command()

    def set_first_person_discrete_target(self, x: int, y: int, z: int) -> Dict[str, Any]:
        """Map cloud input: +x forward, +y left, +z up (each -5..5)."""
        xi = int(clamp(int(x), -5, 5))
        yi = int(clamp(int(y), -5, 5))
        zi = int(clamp(int(z), -5, 5))
        return self.set_first_person_target(
            FIRST_PERSON_BASE_X_CM - yi * FIRST_PERSON_LEFT_STEP_CM,
            FIRST_PERSON_BASE_Y_CM + xi * FIRST_PERSON_FORWARD_STEP_CM,
            FIRST_PERSON_BASE_Z_CM + zi * FIRST_PERSON_UP_STEP_CM,
        )

    def _make_first_person_current_command(self) -> Dict[str, Any]:
        st = self._first_person_state
        return {
            "x": st.get("target_x", FIRST_PERSON_BASE_X_CM),
            "y": st.get("target_y", FIRST_PERSON_BASE_Y_CM),
            "z": st.get("target_z", FIRST_PERSON_BASE_Z_CM),
            "servo1": st.get("last_j4", FIRST_PERSON_BASE_J4_DEG),
            "servo2": st.get("last_j5", FIRST_PERSON_BASE_J5_DEG),
            "flag": 0x01,
        }

    def set_pose_mode(self, mode: str) -> Optional[Dict[str, Any]]:
        """Switch pose mode. Returns a face-home command if exiting scenario modes."""
        mode = mode.lower()
        if mode not in ("face", "body", "intro", "interview", "first_person"):
            return None
        old = self.pose_mode
        if mode != old:
            with self._pnp_lock:
                self._pnp["valid"] = False
                self._pnp["ready"] = False
                self._pnp["yaw_correction"] = 0.0
                self._pnp["pitch_correction"] = 0.0
        exiting_scenario_to_face = (mode == "face" and old in ("intro", "interview", "first_person"))
        home_cmd = None
        if exiting_scenario_to_face:
            home_cmd = self._make_face_home_command()
        elif mode == "first_person" and old != "first_person":
            home_cmd = self._make_first_person_initial_command()
        self.pose_mode = mode
        if mode == "intro" and old != "intro":
            self._scenario_reset["intro"] = True
            self._intro_state["first"] = True
        elif mode == "interview" and old != "interview":
            self._scenario_reset["interview"] = True
            self._interview_state["first"] = True
        elif mode == "first_person" and old != "first_person":
            self._first_person_state["first"] = True
            self.head_center_request = True
            print("[FirstPerson] requested head IMU center recapture")
        elif exiting_scenario_to_face:
            self.head_center_request = True
            print("[ModeHome] requested head IMU center recapture for FACE")
        print(f"[Mode] Switched to {mode.upper()}")
        return home_cmd

    def _make_face_home_command(self) -> Dict[str, Any]:
        """Return arm to a safe face-mode home pose when leaving scenario modes."""
        profile = self.profile
        tx, ty, tz, servo2, servo1 = self._compute_arm_pose(profile, 0.0, 0.0, 0.0)
        self.arm_target_yaw = 0.0
        self.arm_target_pitch = 0.0
        print(f"[ModeHome] face home pose: x={tx:.1f} y={ty:.1f} z={tz:.1f} "
              f"k1={servo2:.1f} k2={servo1:.1f}")
        return {"x": tx, "y": ty, "z": tz, "servo1": servo1, "servo2": servo2, "flag": 0x01}

    def _make_first_person_initial_command(self) -> Dict[str, Any]:
        st = self._first_person_state
        st.setdefault("target_z", FIRST_PERSON_BASE_Z_CM)
        st["head_pitch"] = 0.0
        st["last_j4"] = clamp(FIRST_PERSON_BASE_J4_DEG + FIRST_PERSON_J4_DEG_PER_Z_CM * (st["target_z"] - FIRST_PERSON_BASE_Z_CM), -90.0, 90.0)
        st["last_j5"] = FIRST_PERSON_BASE_J5_DEG
        st["last_send_us"] = 0
        st.setdefault("target_x", FIRST_PERSON_BASE_X_CM)
        st.setdefault("target_y", FIRST_PERSON_BASE_Y_CM)
        self.arm_target_yaw = 0.0
        self.arm_target_pitch = 0.0
        print(f"[FirstPerson] initial pose: x={st['target_x']:.1f} "
              f"y={st['target_y']:.1f} z={st['target_z']:.1f} "
              f"J5={FIRST_PERSON_BASE_J5_DEG:.1f} J4={st['last_j4']:.1f}")
        return self._make_first_person_current_command()

    def _update_first_person_control(self, vec_yaw: float, vec_pitch: float,
                                     now_us: int) -> Optional[Dict[str, Any]]:
        st = self._first_person_state
        pitch_control_deg = self.head_pitch_control_sign * vec_pitch
        j5 = clamp(FIRST_PERSON_BASE_J5_DEG + FIRST_PERSON_YAW_GAIN * vec_yaw, 0.0, 270.0)
        j4_base = FIRST_PERSON_BASE_J4_DEG + FIRST_PERSON_J4_DEG_PER_Z_CM * (st.get("target_z", FIRST_PERSON_BASE_Z_CM) - FIRST_PERSON_BASE_Z_CM)
        j4 = clamp(j4_base - FIRST_PERSON_PITCH_GAIN * pitch_control_deg, -90.0, 90.0)
        first = st.get("first", True)
        changed = (
            abs(j5 - st.get("last_j5", FIRST_PERSON_BASE_J5_DEG)) >= FIRST_PERSON_SERVO_DEADBAND_DEG
            or abs(j4 - st.get("last_j4", FIRST_PERSON_BASE_J4_DEG)) >= FIRST_PERSON_SERVO_DEADBAND_DEG
        )
        if not first and not changed:
            return None
        if not first and now_us - st.get("last_send_us", 0) < FIRST_PERSON_SERVO_PERIOD_US:
            return None
        st["first"] = False
        st["last_send_us"] = now_us
        st["last_j4"] = j4
        st["last_j5"] = j5
        st["head_pitch"] = vec_pitch
        self.arm_target_yaw = vec_yaw
        self.arm_target_pitch = vec_pitch
        return {
            "x": st.get("target_x", FIRST_PERSON_BASE_X_CM),
            "y": st.get("target_y", FIRST_PERSON_BASE_Y_CM),
            "z": st.get("target_z", FIRST_PERSON_BASE_Z_CM),
            "servo1": j4,
            "servo2": j5,
            "flag": 0x01,
            "info": {"mode": "first_person", "vec_yaw": vec_yaw, "vec_pitch": vec_pitch},
        }

    # ------------------------------------------------------------------
    # Scenario modes (intro / interview)
    # ------------------------------------------------------------------
    @staticmethod
    def _det_area(det: Dict[str, Any]) -> float:
        return max(0.0, (det["x2"] - det["x1"]) * (det["y2"] - det["y1"]))

    @staticmethod
    def _torso_center(det: Dict[str, Any]) -> Tuple[Optional[float], Optional[float]]:
        kps = det.get("kps", [])
        xs, ys = [], []
        for idx in (5, 6, 11, 12):
            if idx < len(kps) and kps[idx].get("visibility", 0) > 0:
                xs.append(kps[idx]["x"])
                ys.append(kps[idx]["y"])
        if not xs:
            return None, None
        return sum(xs) / len(xs), sum(ys) / len(ys)

    @staticmethod
    def _estimate_head_center(det: Dict[str, Any]) -> Tuple[Optional[float], Optional[float]]:
        kps = det.get("kps", [])
        xs, ys, w = [], [], []
        # COCO: nose=0, left eye=1, right eye=2, left ear=3, right ear=4
        for idx, weight in ((0, 2.0), (1, 1.0), (2, 1.0), (3, 0.5), (4, 0.5)):
            if idx < len(kps) and kps[idx].get("visibility", 0) > 0:
                xs.append(kps[idx]["x"] * weight)
                ys.append(kps[idx]["y"] * weight)
                w.append(weight)
        if not w:
            return None, None
        return sum(xs) / sum(w), sum(ys) / sum(w)

    @staticmethod
    def _estimate_frontal_score(det: Dict[str, Any]) -> float:
        kps = det.get("kps", [])
        if len(kps) < 17:
            return 0.0
        # Simple frontal score based on shoulder/hip symmetry.
        def v(idx):
            return kps[idx].get("visibility", 0)
        if v(5) == 0 or v(6) == 0 or v(11) == 0 or v(12) == 0:
            return 0.0
        sx = kps[5]["x"] + kps[11]["x"]
        dx = kps[6]["x"] + kps[12]["x"]
        dist = max(1.0, abs(sx) + abs(dx))
        return 1.0 - abs(sx - dx) / dist

    @staticmethod
    def _kp_visible(det: Dict[str, Any], idx: int, threshold: float = 0.0) -> bool:
        kps = det.get("kps", [])
        return idx < len(kps) and kps[idx].get("visibility", 0) > threshold

    @staticmethod
    def _kp_xy(det: Dict[str, Any], idx: int) -> Tuple[float, float]:
        kps = det.get("kps", [])
        return kps[idx]["x"], kps[idx]["y"]

    def update_intro_control(self, detections: List[Dict[str, Any]],
                             img_w: int, img_h: int,
                             now_us: Optional[int] = None) -> Optional[Dict[str, Any]]:
        """Intro scenario, aligned with Base commit 0fea25e."""
        st = self._intro_state
        current_us = now_us or int(time.perf_counter() * 1_000_000)

        def make_command() -> Dict[str, Any]:
            desired_j5 = clamp(
                INTRO_SERVO2_BASE_DEG + st["intro_servo_yaw_deg"],
                0.0,
                270.0,
            )
            j5 = clamp(
                desired_j5,
                st["last_sent_j5"] - SCENARIO_SERVO_MAX_STEP_DEG,
                st["last_sent_j5"] + SCENARIO_SERVO_MAX_STEP_DEG,
            )
            command = {
                "x": st["intro_space_x_cm"],
                "y": INTRO_BASE_Y_CM,
                "z": INTRO_BASE_Z_CM,
                "servo1": INTRO_SERVO1_BASE_DEG,
                "servo2": j5,
                "flag": 0x01,
            }
            st["last_sent_x"] = command["x"]
            st["last_sent_y"] = command["y"]
            st["last_sent_z"] = command["z"]
            st["last_sent_j5"] = j5
            st["last_send_us"] = current_us
            self.arm_target_yaw = 0.0
            self.arm_target_pitch = 0.0
            return command

        # Base sends the initial scenario pose immediately, even before the
        # first valid body detection arrives.
        if st["first"]:
            st.update({
                "first": False,
                "intro_space_x_cm": INTRO_BASE_X_CM,
                "intro_servo_yaw_deg": 0.0,
                "intro_yaw_hold": False,
                "hand_center_since_us": 0,
                "space_center_last_step_us": 0,
                "space_present_last_step_us": 0,
                "moving_gain_active_us": 0,
                "last_send_us": 0,
                "last_sent_x": 0.0,
                "last_sent_y": 0.0,
                "last_sent_z": 0.0,
                "last_sent_j5": INTRO_SERVO2_BASE_DEG,
            })
            return make_command()

        if not detections:
            return None
        det = detections[0]

        # Base falls back to the bounding-box center when torso keypoints are
        # unavailable, rather than suppressing the whole scenario update.
        tcx, tcy = self._torso_center(det)
        if tcx is None:
            tcx = (det["x1"] + det["x2"]) * 0.5
            tcy = (det["y1"] + det["y2"]) * 0.5

        # The presentation hand is COCO right wrist (index 10). Hand presence
        # is measured relative to the torso; image-center error is a separate
        # signal used only for J5 aiming.
        wrist_visible = self._kp_visible(det, 10, SCENARIO_KPT_CONF_THRESHOLD)
        hand_offset_norm = 0.0
        target_x, target_y = tcx, tcy
        if wrist_visible:
            wx, wy = self._kp_xy(det, 10)
            hand_offset_norm = (wx - tcx) / max(1.0, img_w * 0.5)
            target_x = INTRO_TARGET_WRIST_WEIGHT * wx + (1.0 - INTRO_TARGET_WRIST_WEIGHT) * tcx
            target_y = 0.70 * wy + 0.30 * tcy

        target_x = clamp(target_x, 0.0, float(max(0, img_w - 1)))
        target_y = clamp(target_y, 0.0, float(max(0, img_h - 1)))
        err_norm = clamp(
            (target_x - img_w * 0.5) / max(1.0, img_w * 0.5) * INTRO_IMAGE_TO_YAW_SIGN,
            -1.0,
            1.0,
        )
        err_abs = abs(err_norm)

        if st["intro_yaw_hold"]:
            if err_abs >= INTRO_CENTER_HOLD_EXIT_NORM:
                st["intro_yaw_hold"] = False
        elif err_abs <= INTRO_CENTER_HOLD_ENTER_NORM:
            st["intro_yaw_hold"] = True

        if wrist_visible and abs(hand_offset_norm) <= INTRO_HAND_CENTER_NORM:
            if st["hand_center_since_us"] == 0:
                st["hand_center_since_us"] = current_us
        else:
            st["hand_center_since_us"] = 0

        hand_center_stable = (
            st["hand_center_since_us"] != 0
            and current_us - st["hand_center_since_us"] >= INTRO_HAND_CENTER_HOLD_US
        )
        hand_extended = wrist_visible and abs(hand_offset_norm) >= INTRO_HAND_PRESENT_NORM
        space_moved = False

        # A centered presentation hand retracts the workspace toward x=0.
        if (
            hand_center_stable
            and current_us - st["space_center_last_step_us"] >= INTRO_SPACE_CENTER_PERIOD_US
        ):
            old_x = st["intro_space_x_cm"]
            if old_x < 0.0:
                st["intro_space_x_cm"] = min(0.0, old_x + INTRO_SPACE_CENTER_STEP_CM)
            elif old_x > 0.0:
                st["intro_space_x_cm"] = max(0.0, old_x - INTRO_SPACE_CENTER_STEP_CM)
            st["space_center_last_step_us"] = current_us
            space_moved = st["intro_space_x_cm"] != old_x

        # Extending the hand restores the presentation workspace toward x=-20.
        if (
            hand_extended
            and current_us - st["space_present_last_step_us"] >= INTRO_SPACE_PRESENT_PERIOD_US
        ):
            old_x = st["intro_space_x_cm"]
            if old_x > INTRO_BASE_X_CM:
                st["intro_space_x_cm"] = max(
                    INTRO_BASE_X_CM, old_x - INTRO_SPACE_CENTER_STEP_CM
                )
            elif old_x < INTRO_BASE_X_CM:
                st["intro_space_x_cm"] = min(
                    INTRO_BASE_X_CM, old_x + INTRO_SPACE_CENTER_STEP_CM
                )
            st["space_present_last_step_us"] = current_us
            space_moved = space_moved or st["intro_space_x_cm"] != old_x

        if space_moved:
            st["moving_gain_active_us"] = current_us

        effective_gain = INTRO_SERVO2_GAIN_DEG
        if (
            st["moving_gain_active_us"] != 0
            and current_us - st["moving_gain_active_us"] <= INTRO_SPACE_MOVING_GAIN_US
        ):
            effective_gain *= INTRO_SERVO2_SPACE_MOVING_GAIN_SCALE

        # Base only steers J5 while the presentation hand is clearly extended.
        # A hand held near the torso for 0.8 s locks J5 at the center angle.
        if hand_center_stable:
            st["intro_servo_yaw_deg"] = clamp(
                INTRO_SERVO2_CENTER_LOCK_DEG - INTRO_SERVO2_BASE_DEG,
                -INTRO_SERVO2_DELTA_LIMIT_DEG,
                INTRO_SERVO2_DELTA_LIMIT_DEG,
            )
            st["intro_yaw_hold"] = True
        elif hand_extended and not st["intro_yaw_hold"]:
            desired = clamp(
                err_norm * effective_gain,
                -INTRO_SERVO2_DELTA_LIMIT_DEG,
                INTRO_SERVO2_DELTA_LIMIT_DEG,
            )
            st["intro_servo_yaw_deg"] = (
                0.60 * st["intro_servo_yaw_deg"] + 0.40 * desired
            )

        j5 = clamp(
            INTRO_SERVO2_BASE_DEG + st["intro_servo_yaw_deg"], 0.0, 270.0
        )
        changed_enough = (
            abs(st["intro_space_x_cm"] - st["last_sent_x"]) >= INTRO_SPACE_CENTER_STEP_CM
            or abs(INTRO_BASE_Y_CM - st["last_sent_y"]) >= 0.8
            or abs(INTRO_BASE_Z_CM - st["last_sent_z"]) >= 0.8
            or abs(j5 - st["last_sent_j5"]) >= INTRO_SERVO2_SEND_DEADBAND_DEG
        )
        if changed_enough and current_us - st["last_send_us"] >= INTRO_SERVO_PERIOD_US:
            return make_command()
        return None

    def update_interview_control(self, detections: List[Dict[str, Any]],
                                 img_w: int, img_h: int,
                                 now_us: Optional[int] = None) -> Optional[Dict[str, Any]]:
        """Interview scenario: center on speaker(s) via integrated yaw servo."""
        st = self._interview_state
        first_update = st["first"]
        if first_update:
            st["first"] = False
            st["interview_servo_yaw_deg"] = 0.0
            st["last_send_us"] = 0
            st["last_sent_j5"] = INTERVIEW_SERVO2_BASE_DEG

        if not detections:
            return None

        current_us = now_us or int(time.perf_counter() * 1_000_000)
        dt = max(0.001, (current_us - st.get("_prev_us", current_us)) / 1e6)
        st["_prev_us"] = current_us

        # Sort by area descending.
        dets = sorted(detections, key=self._det_area, reverse=True)

        target_x, target_y = None, None
        source = 0
        if len(dets) >= 2:
            primary = dets[0]
            secondary = None
            primary_area = self._det_area(primary)
            for d in dets[1:]:
                if self._det_area(d) >= 0.25 * primary_area and self._estimate_frontal_score(d) >= 0.5:
                    secondary = d
                    break
            cx1, cy1 = self._estimate_head_center(primary)
            if secondary is not None:
                cx2, cy2 = self._estimate_head_center(secondary)
                if cx2 is not None:
                    target_x, target_y = (cx1 + cx2) * 0.5, (cy1 + cy2) * 0.5
                    source = 2
            if target_x is None and cx1 is not None:
                target_x, target_y = cx1, cy1
                source = 3
        else:
            det = dets[0]
            if self._kp_visible(det, 9, INTERVIEW_WRIST_CONF_THRESHOLD):
                target_x, target_y = self._kp_xy(det, 9)
                source = 1
            else:
                target_x, target_y = self._estimate_head_center(det)
                source = 3

        if target_x is None:
            return None

        # Low-pass filter target.
        st["target_x"] = 0.75 * st["target_x"] + 0.25 * target_x
        st["target_y"] = 0.75 * st["target_y"] + 0.25 * target_y

        err_norm = (st["target_x"] - img_w * 0.5) / (img_w * 0.5) * INTERVIEW_IMAGE_TO_YAW_SIGN
        err_abs = abs(err_norm)

        # Hysteresis.
        if not st["interview_yaw_hold"] and err_abs <= INTERVIEW_HOLD_ENTER_NORM:
            st["interview_yaw_hold"] = True
        elif st["interview_yaw_hold"] and err_abs >= INTERVIEW_HOLD_EXIT_NORM:
            st["interview_yaw_hold"] = False

        if not st["interview_yaw_hold"]:
            step = INTERVIEW_IMAGE_TO_YAW_SIGN * err_norm * INTERVIEW_SERVO2_I_GAIN_DEG_PER_SEC * dt
            st["interview_servo_yaw_deg"] = clamp(
                st["interview_servo_yaw_deg"] + step,
                -INTERVIEW_SERVO2_DELTA_LIMIT_DEG,
                INTERVIEW_SERVO2_DELTA_LIMIT_DEG,
            )

        desired_j5 = clamp(
            INTERVIEW_SERVO2_BASE_DEG + st["interview_servo_yaw_deg"], 0.0, 270.0
        )
        j5 = clamp(
            desired_j5,
            st["last_sent_j5"] - SCENARIO_SERVO_MAX_STEP_DEG,
            st["last_sent_j5"] + SCENARIO_SERVO_MAX_STEP_DEG,
        )
        j4 = INTERVIEW_SERVO1_BASE_DEG

        if current_us - st["last_send_us"] >= INTERVIEW_SERVO_PERIOD_US:
            if (first_update or
                    abs(j5 - st["last_sent_j5"]) >= INTERVIEW_SERVO2_SEND_DEADBAND_DEG):
                st["last_send_us"] = current_us
                st["last_sent_j5"] = j5
                st["last_source"] = source
                self.arm_target_yaw = st["interview_servo_yaw_deg"]
                self.arm_target_pitch = 0.0
                return {
                    "x": INTERVIEW_BASE_X_CM, "y": INTERVIEW_BASE_Y_CM, "z": INTERVIEW_BASE_Z_CM,
                    "servo1": j4, "servo2": j5, "flag": 0x01,
                }
        return None

    def toggle_pitch_sign(self):
        self.head_pitch_control_sign *= -1
        print(f"[Ctrl] Head pitch control sign = {self.head_pitch_control_sign}")

    def request_head_center(self):
        self.head_center_request = True
        print("[Ctrl] Head center requested")

    def request_rebaseline(self):
        """Reset center and biases, keep R_init."""
        self.head_center_request = True
        self.head_center_set = False
        self.pitch_visual_bias_deg = 0.0
        self.effective_pitch_visual_bias_deg = 0.0
        self.R_bias_total = None
        self._reset_visual_direction_state()
        self.last_cmd_target_pitch = 0.0
        self.last_cmd_target_yaw = 0.0
        self.predictor.reset()
        self.predictor_yaw.reset()
        print("[Ctrl] Rebaseline requested")

    def _reset_visual_direction_state(self):
        self.visual_pitch_pending_sign = 0
        self.visual_yaw_pending_sign = 0
        self.visual_pitch_last_cmd_sign = 0
        self.visual_yaw_last_cmd_sign = 0
        self.visual_pitch_blocked_sign = 0
        self.visual_yaw_blocked_sign = 0
        self.visual_pitch_reversal_pending = False
        self.visual_yaw_reversal_pending = False

    @staticmethod
    def _step_sign(value: float) -> int:
        return 1 if value > 0.0 else (-1 if value < 0.0 else 0)

    @staticmethod
    def _remove_calib_files():
        """Remove temporary calibration files, matching upstream main.cpp startup."""
        for p in ("/tmp/calib_mode.txt", "/tmp/servo_calib.txt"):
            try:
                if os.path.exists(p):
                    os.remove(p)
                    print(f"[Ctrl] Removed {p}")
            except Exception as e:
                print(f"[Ctrl] Failed to remove {p}: {e}")

    def interp_yaw_table(self, arm_yaw_deg: float) -> float:
        return interp_table(self.profile.yaw_calib, arm_yaw_deg)

    def interp_pitch_table(self, arm_pitch_deg: float) -> float:
        return interp_table(self.profile.pitch_calib, arm_pitch_deg)

    def put_pnp_correction(self, yaw_correction: float, pitch_correction: float,
                           valid: bool = True):
        with self._pnp_lock:
            self._pnp["yaw_correction"] = yaw_correction
            self._pnp["pitch_correction"] = pitch_correction
            self._pnp["valid"] = valid
            self._pnp["ready"] = valid

    def _read_pnp_correction(self) -> Tuple[bool, float, float]:
        with self._pnp_lock:
            return self._pnp["ready"], self._pnp["pitch_correction"], self._pnp["yaw_correction"]

    def _consume_pnp_correction(self) -> Tuple[bool, float, float]:
        with self._pnp_lock:
            ready = self._pnp["ready"]
            pc = self._pnp["pitch_correction"]
            yc = self._pnp["yaw_correction"]
            self._pnp["ready"] = False
            return ready, pc, yc

    def _recompute_after_bias(self, head_imu: Dict[str, Any], imu2: Dict[str, Any]):
        """Recompute vec_yaw/vec_pitch after applying R_bias_total."""
        cr, cp, cy = head_imu["roll"], head_imu["pitch"], head_imu["yaw"]
        qw, qx, qy, qz = head_imu["qw"], head_imu["qx"], head_imu["qy"], head_imu["qz"]
        quat_valid = head_imu.get("quat_valid", False)
        imu2_valid = imu2.get("valid", False)

        R_head = quatToMat(qw, qx, qy, qz) if quat_valid else eulerZYXToMat(cr, cp, cy)
        if self.R_bias_total is not None:
            R_head = self.R_bias_total @ R_head
        R_head_delta = R_head @ self.R_init.T
        R_rel = R_head_delta
        if imu2_valid and self.R_imu2_init is not None:
            R_imu2 = eulerZYXToMat(imu2["roll"], imu2["pitch"], imu2["yaw"])
            R_imu2_delta = R_imu2 @ self.R_imu2_init.T
            R_rel = R_imu2_delta.T @ R_head_delta

        if self.head_center_set and self.R_head_center_rel is not None:
            R_centered = self.R_head_center_rel.T @ R_rel
            vec_yaw, vec_pitch = axisProjectionYawPitchCompensated(R_centered)
        else:
            vec_yaw, vec_pitch = axisProjectionYawPitchCompensated(R_rel)
        return vec_yaw, vec_pitch

    def update(self, head_imu: Optional[Dict[str, Any]], imu2: Dict[str, Any],
               r_init_set: bool, now_us: int) -> Optional[Dict[str, Any]]:
        if now_us - self.last_update_us < CONTROL_PERIOD_US:
            return None
        self.last_update_us = now_us

        if head_imu is None:
            head_imu = {"imu_valid": False}

        # Scenario modes are driven by the vision pipeline, not the IMU loop.
        if self.pose_mode in ("intro", "interview"):
            return None

        cr = head_imu.get("roll", 0.0)
        cp = head_imu.get("pitch", 0.0)
        cy = head_imu.get("yaw", 0.0)
        qw = head_imu.get("qw", 1.0)
        qx = head_imu.get("qx", 0.0)
        qy = head_imu.get("qy", 0.0)
        qz = head_imu.get("qz", 0.0)
        quat_valid = head_imu.get("quat_valid", False)
        wx = head_imu.get("wx", 0.0)
        wy = head_imu.get("wy", 0.0)
        wz = head_imu.get("wz", 0.0)
        imu_valid = head_imu.get("imu_valid", False)
        wy_hist = head_imu.get("wy_hist", [])
        wz_hist = head_imu.get("wz_hist", [])

        imu2_valid = imu2.get("valid", False)

        # ----- A-inverse capture -----
        if imu_valid and r_init_set and self.R_init is None:
            self.R_init = quatToMat(qw, qx, qy, qz) if quat_valid else eulerZYXToMat(cr, cp, cy)
            if imu2_valid:
                self.R_imu2_init = eulerZYXToMat(imu2["roll"], imu2["pitch"], imu2["yaw"])
            print(f"[A-INIT] R_init built: head(roll={cr:.2f} pitch={cp:.2f} yaw={cy:.2f}) "
                  f"imu2_valid={int(imu2_valid)}")

        if imu_valid and r_init_set:
            if imu2_valid and self.R_imu2_init is None:
                self.R_imu2_init = eulerZYXToMat(imu2["roll"], imu2["pitch"], imu2["yaw"])
                print(f"[IMU2] Late R_init captured: roll={imu2['roll']:.2f} "
                      f"pitch={imu2['pitch']:.2f} yaw={imu2['yaw']:.2f}")

        # ----- Per-frame relative pose -----
        rel_roll = rel_pitch = rel_yaw = 0.0
        vec_pitch = vec_yaw = 0.0
        if imu_valid and r_init_set and self.R_init is not None:
            R_head = quatToMat(qw, qx, qy, qz) if quat_valid else eulerZYXToMat(cr, cp, cy)
            if self.R_bias_total is not None:
                R_head = self.R_bias_total @ R_head
            R_head_delta = R_head @ self.R_init.T
            R_rel = R_head_delta
            if imu2_valid and self.R_imu2_init is not None:
                R_imu2 = eulerZYXToMat(imu2["roll"], imu2["pitch"], imu2["yaw"])
                R_imu2_delta = R_imu2 @ self.R_imu2_init.T
                R_rel = R_imu2_delta.T @ R_head_delta
            rel_roll, rel_pitch, rel_yaw = matToEulerZYX(R_rel)
            if self.head_center_set and self.R_head_center_rel is not None:
                R_centered = self.R_head_center_rel.T @ R_rel
                vec_yaw, vec_pitch = axisProjectionYawPitchCompensated(R_centered)
            else:
                vec_yaw, vec_pitch = axisProjectionYawPitchCompensated(R_rel)

        # ----- Relative angular rates from differentiated vec angles -----
        rel_pitch_rate = wy
        rel_yaw_rate = wz
        if imu_valid and r_init_set and self.R_init is not None:
            if self.rel_rate_init and self.prev_rel_rate_us > 0 and now_us > self.prev_rel_rate_us:
                dt = (now_us - self.prev_rel_rate_us) / 1e6
                if 0.001 < dt < 0.5:
                    rel_pitch_rate = normalize_angle_deg(vec_pitch - self.prev_vec_pitch) / dt
                    rel_yaw_rate = normalize_angle_deg(vec_yaw - self.prev_vec_yaw) / dt
            else:
                rel_pitch_rate = 0.0
                rel_yaw_rate = 0.0
                self.rel_rate_init = True
            self.prev_vec_pitch = vec_pitch
            self.prev_vec_yaw = vec_yaw
            self.prev_rel_rate_us = now_us

            self.rel_wy_hist[self.rel_wy_idx] = rel_pitch_rate
            self.rel_wy_idx = (self.rel_wy_idx + 1) % NRF24_WY_HIST_SIZE
            if self.rel_wy_count < NRF24_WY_HIST_SIZE:
                self.rel_wy_count += 1

            self.rel_wz_hist[self.rel_wz_idx] = rel_yaw_rate
            self.rel_wz_idx = (self.rel_wz_idx + 1) % NRF24_WZ_HIST_SIZE
            if self.rel_wz_count < NRF24_WZ_HIST_SIZE:
                self.rel_wz_count += 1

            # Replace source histories with relative-rate histories for the FSM
            wy_hist = self.rel_wy_hist[:self.rel_wy_count]
            wz_hist = self.rel_wz_hist[:self.rel_wz_count]
            wy = rel_pitch_rate
            wz = rel_yaw_rate

        # Calibration modes override normal control.
        calib_cmd = self._run_calibration(now_us, head_imu, imu2, r_init_set)
        if calib_cmd is not None:
            return calib_cmd
        if self.calib_mode != 0:
            return None

        if not imu_valid or not r_init_set or self.R_init is None:
            return None

        # ----- Head center capture -----
        if not self.head_center_set or self.head_center_request:
            vec_pitch = 0.0
            vec_yaw = 0.0
            self.head_center_set = True
            self.head_center_request = False
            self.pitch_visual_bias_deg = 0.0
            self.effective_pitch_visual_bias_deg = 0.0
            self.last_is_stop_yaw_for_pitch_bias = True
            self.last_pitch_visual_bias_step = 0.0
            self._reset_visual_direction_state()
            self.last_cmd_target_pitch = 0.0
            self.last_cmd_target_yaw = 0.0
            self.predictor.reset()
            self.predictor_yaw.reset()
            print(f"[HeadCenter] vector center captured "
                  f"(rel_roll={rel_roll:+.2f} rel_pitch={rel_pitch:+.2f} rel_yaw={rel_yaw:+.2f})")
            # Recompute R_head_center_rel for future centered projection
            R_head = quatToMat(qw, qx, qy, qz) if quat_valid else eulerZYXToMat(cr, cp, cy)
            if self.R_bias_total is not None:
                R_head = self.R_bias_total @ R_head
            R_head_delta = R_head @ self.R_init.T
            self.R_head_center_rel = R_head_delta
            if imu2_valid and self.R_imu2_init is not None:
                R_imu2 = eulerZYXToMat(imu2["roll"], imu2["pitch"], imu2["yaw"])
                R_imu2_delta = R_imu2 @ self.R_imu2_init.T
                self.R_head_center_rel = R_imu2_delta.T @ R_head_delta

        if self.pose_mode == "first_person":
            return self._update_first_person_control(vec_yaw, vec_pitch, now_us)

        pitch_control_deg = self.head_pitch_control_sign * vec_pitch
        yaw_control_deg = vec_yaw

        # ----- Pitch FSM -----
        if not self.pitch_baseline_set:
            self.pitch_baseline_set = True
            self.last_cmd_target_pitch = 0.0
            self.predictor.reset()
        self.ctx.update_from_hist(wy_hist, len(wy_hist), 5)
        curr_state = next_motion_state(self.prev_state, self.ctx)
        self.prev_state = curr_state
        self.predictor.update(wy, curr_state)
        is_stop = self.ctx.is_stop()

        # ----- Yaw FSM -----
        if not self.yaw_baseline_set:
            self.yaw_baseline_set = True
            self.last_cmd_target_yaw = 0.0
            self.predictor_yaw.reset()
        self.ctx_yaw.update_from_hist(wz_hist, len(wz_hist), 5)
        curr_state_yaw = next_motion_state(self.prev_state_yaw, self.ctx_yaw)
        self.prev_state_yaw = curr_state_yaw
        self.predictor_yaw.update(wz, curr_state_yaw)
        is_stop_yaw = self.ctx_yaw.is_stop()

        # Pitch bias decay when yaw starts moving
        if self.last_is_stop_yaw_for_pitch_bias and not is_stop_yaw:
            self.pitch_visual_bias_deg *= 0.5
            self.last_pitch_visual_bias_step = 0.0
        self.last_is_stop_yaw_for_pitch_bias = is_stop_yaw

        # A corresponding IMU movement releases the visual direction lock.
        if not is_stop:
            self.visual_pitch_blocked_sign = 0
            self.visual_pitch_last_cmd_sign = 0
            self.visual_pitch_reversal_pending = False
            self.visual_pitch_pending_sign = 0
        if not is_stop_yaw:
            self.visual_yaw_blocked_sign = 0
            self.visual_yaw_last_cmd_sign = 0
            self.visual_yaw_reversal_pending = False
            self.visual_yaw_pending_sign = 0

        # ----- PnP drift correction -----
        # Consume each fresh visual frame without waiting for the IMUs or arm to stop.
        do_pnp_correct = False
        yaw_delta = 0.0
        pitch_delta = 0.0
        ready, pnp_pitch_corr, pnp_yaw_corr = self._consume_pnp_correction()
        if ready:
            pitch_corr_limited = clamp(
                pnp_pitch_corr, -PNP_FRAME_ERROR_LIMIT_DEG, PNP_FRAME_ERROR_LIMIT_DEG
            )
            yaw_corr_limited = clamp(
                pnp_yaw_corr, -PNP_FRAME_ERROR_LIMIT_DEG, PNP_FRAME_ERROR_LIMIT_DEG
            )
            pitch_step = KI_PNP_PITCH_BIAS * pitch_corr_limited
            yaw_step = -KI_PNP * yaw_corr_limited
            pitch_sign = self._step_sign(pitch_step)
            yaw_sign = self._step_sign(yaw_step)

            if (pitch_sign and not self.visual_pitch_reversal_pending and
                    pitch_sign != self.visual_pitch_blocked_sign):
                if (self.visual_pitch_last_cmd_sign and
                        pitch_sign != self.visual_pitch_last_cmd_sign):
                    self.visual_pitch_reversal_pending = True
                self.pitch_visual_bias_deg += pitch_step
                self.last_pitch_visual_bias_step = pitch_step
                self.visual_pitch_pending_sign = pitch_sign
                do_pnp_correct = True

            if (yaw_sign and not self.visual_yaw_reversal_pending and
                    yaw_sign != self.visual_yaw_blocked_sign):
                if (self.visual_yaw_last_cmd_sign and
                        yaw_sign != self.visual_yaw_last_cmd_sign):
                    self.visual_yaw_reversal_pending = True
                yaw_delta = yaw_step
                self.visual_yaw_pending_sign = yaw_sign
                do_pnp_correct = True

        if do_pnp_correct:
            if self.R_bias_total is None:
                self.R_bias_total = np.eye(3, dtype=np.float32)
            R_delta = eulerZYXToMat(0.0, pitch_delta, yaw_delta)
            self.R_bias_total = R_delta @ self.R_bias_total
            vec_yaw, vec_pitch = self._recompute_after_bias(head_imu, imu2)
            pitch_control_deg = self.head_pitch_control_sign * vec_pitch
            yaw_control_deg = vec_yaw

        # ----- Pitch bias weighting and position fade -----
        yaw_abs = abs(yaw_control_deg)
        weight = clamp01(yaw_abs / PITCH_BIAS_FULL_YAW_DEG)
        self.effective_pitch_visual_bias_deg = self.pitch_visual_bias_deg * (0.5 + 0.5 * weight)

        profile = self.profile
        fade_start = profile.pitch_pos_fade_start_yaw_deg
        fade_end = profile.pitch_pos_fade_end_yaw_deg
        pitch_position_weight = 1.0
        if yaw_abs > fade_start:
            pitch_position_weight = clamp01((fade_end - yaw_abs) / (fade_end - fade_start))
        position_pitch_visual_bias_deg = self.effective_pitch_visual_bias_deg * pitch_position_weight

        pitch_input_deg = normalize_angle_deg(pitch_control_deg - self.effective_pitch_visual_bias_deg)
        position_pitch_input_deg = normalize_angle_deg(pitch_control_deg - position_pitch_visual_bias_deg)

        target_pitch_deg = pitch_input_deg if is_stop else self.predictor.get_target_yaw(pitch_input_deg)
        target_position_pitch_deg = (position_pitch_input_deg if is_stop
                                     else self.predictor.get_target_yaw(position_pitch_input_deg))
        if self.visual_pitch_reversal_pending:
            midpoint_delta = normalize_angle_deg(target_pitch_deg - self.last_cmd_target_pitch) * 0.5
            target_pitch_deg = normalize_angle_deg(self.last_cmd_target_pitch + midpoint_delta)
            target_position_pitch_deg = target_pitch_deg
        delta_pitch_deg = normalize_angle_deg(target_pitch_deg - self.last_cmd_target_pitch)
        pitch_moved_enough = abs(delta_pitch_deg) > CMD_PITCH_THRESHOLD_DEG

        pitch_interval = FAST_INTERVAL_US if curr_state in (STATE_ACCEL_TO_CONST, STATE_CONST_TO_DECEL) else INTERVAL_US
        pitch_interval_ok = (now_us - self.last_cmd_us) >= pitch_interval
        pitch_should_cmd = pitch_interval_ok and (
            pitch_moved_enough or self.visual_pitch_reversal_pending
        )

        target_yaw_deg = yaw_control_deg if is_stop_yaw else self.predictor_yaw.get_target_yaw(yaw_control_deg)
        if self.visual_yaw_reversal_pending:
            midpoint_delta = normalize_angle_deg(target_yaw_deg - self.last_cmd_target_yaw) * 0.5
            target_yaw_deg = normalize_angle_deg(self.last_cmd_target_yaw + midpoint_delta)
        delta_yaw_deg = normalize_angle_deg(target_yaw_deg - self.last_cmd_target_yaw)
        yaw_moved_enough = abs(delta_yaw_deg) > CMD_YAW_THRESHOLD_DEG

        yaw_interval = FAST_INTERVAL_US if curr_state_yaw in (STATE_ACCEL_TO_CONST, STATE_CONST_TO_DECEL) else INTERVAL_US
        yaw_interval_ok = (now_us - self.last_cmd_us_yaw) >= yaw_interval
        yaw_should_cmd = yaw_interval_ok and (
            yaw_moved_enough or self.visual_yaw_reversal_pending
        )

        should_cmd = pitch_should_cmd or yaw_should_cmd
        result = None
        if should_cmd:
            tx, ty, tz, servo2, servo1 = self._compute_arm_pose(
                profile,
                normalize_angle_deg(target_pitch_deg - 0.0),
                normalize_angle_deg(target_position_pitch_deg - 0.0),
                normalize_angle_deg(target_yaw_deg - 0.0),
            )

            pos_changed_enough = False
            if self.visual_pitch_reversal_pending or self.visual_yaw_reversal_pending:
                pos_changed_enough = True
            elif not self.prev_sent_initialized:
                pos_changed_enough = True
            else:
                pos_changed_enough = (
                    abs(tx - self.prev_sent_tx) >= POS_DEADZONE_CM or
                    abs(ty - self.prev_sent_ty) >= POS_DEADZONE_CM or
                    abs(tz - self.prev_sent_tz) >= POS_DEADZONE_CM
                )

            if pos_changed_enough:
                self.prev_sent_tx = tx
                self.prev_sent_ty = ty
                self.prev_sent_tz = tz
                self.prev_sent_initialized = True

                is_prediction = (pitch_should_cmd and not is_stop) or (yaw_should_cmd and not is_stop_yaw)
                flag = 0x00 if is_prediction else 0x01

                result = {
                    "x": tx, "y": ty, "z": tz,
                    "servo1": servo1, "servo2": servo2, "flag": flag,
                    "info": {
                        "vec_pitch": vec_pitch,
                        "vec_yaw": vec_yaw,
                        "target_pitch": target_pitch_deg,
                        "target_yaw": target_yaw_deg,
                        "delta_pitch": delta_pitch_deg,
                        "delta_yaw": delta_yaw_deg,
                        "pitch_state": STATE_NAMES[curr_state],
                        "yaw_state": STATE_NAMES[curr_state_yaw],
                        "pitch_bias": self.pitch_visual_bias_deg,
                        "eff_bias": self.effective_pitch_visual_bias_deg,
                        "pos_weight": pitch_position_weight,
                        "pnp_ready": do_pnp_correct,
                    }
                }

                if pitch_should_cmd:
                    self.last_cmd_target_pitch = target_pitch_deg
                    self.last_cmd_us = now_us
                    if self.visual_pitch_pending_sign:
                        if self.visual_pitch_reversal_pending:
                            self.visual_pitch_blocked_sign = self.visual_pitch_pending_sign
                            self.visual_pitch_reversal_pending = False
                        else:
                            self.visual_pitch_last_cmd_sign = self.visual_pitch_pending_sign
                        self.visual_pitch_pending_sign = 0
                if yaw_should_cmd:
                    self.last_cmd_target_yaw = target_yaw_deg
                    self.last_cmd_us_yaw = now_us
                    if self.visual_yaw_pending_sign:
                        if self.visual_yaw_reversal_pending:
                            self.visual_yaw_blocked_sign = self.visual_yaw_pending_sign
                            self.visual_yaw_reversal_pending = False
                        else:
                            self.visual_yaw_last_cmd_sign = self.visual_yaw_pending_sign
                        self.visual_yaw_pending_sign = 0
                self.last_any_cmd_us = now_us

                self.arm_target_yaw = target_yaw_deg
                self.arm_target_pitch = target_pitch_deg

        # ----- Stationary / stable flags -----
        if should_cmd:
            self.last_any_cmd_us = now_us

        if not self.is_stationary_now:
            self.is_stationary_now = (abs(wx) < STATIONARY_ENTER_W and abs(wz) < STATIONARY_ENTER_W)
        else:
            if abs(wx) > STATIONARY_EXIT_W or abs(wz) > STATIONARY_EXIT_W:
                self.is_stationary_now = False

        if self.is_stationary_now:
            if not self.was_stationary:
                self.stationary_since_us = now_us
                self.was_stationary = True
            self.head_stationary = 1
        else:
            self.was_stationary = False
            self.head_stationary = 0

        self.arm_stable = 1 if (self.was_stationary and
                                (now_us - self.stationary_since_us) > STABLE_TIMEOUT_US) else 0

        return result

    def _compute_arm_pose(self, profile: ArmKinematicsProfile,
                          delta_pitch_servo_deg: float,
                          delta_pitch_position_deg: float,
                          delta_yaw_deg: float) -> Tuple[float, float, float, float, float]:
        position_pitch_deg = profile.position_pitch_sign * delta_pitch_position_deg
        cum_pitch = clamp(deg2rad(position_pitch_deg), -math.pi / 2, math.pi / 2)
        cum_yaw = clamp(-deg2rad(delta_yaw_deg), -math.pi / 2, math.pi / 2)

        tx = profile.l3 * math.sin(cum_yaw) * math.cos(cum_pitch)
        ty = (profile.l4 - profile.l1 * math.sin(cum_pitch) +
              profile.l3 * math.cos(cum_pitch) * math.cos(cum_yaw))
        tz = (profile.l2 +
              profile.l1 * math.cos(cum_pitch * profile.pitch_z_gain) +
              profile.l3 * math.sin(cum_pitch * profile.pitch_z_gain) * math.cos(cum_yaw))

        pitch_servo_k = profile.servo1_gain_up if delta_pitch_servo_deg >= 0 else profile.servo1_gain_down
        servo1 = clamp(profile.servo1_baseline + pitch_servo_k * delta_pitch_servo_deg, -90.0, 90.0)
        servo2 = clamp(profile.servo2_baseline + profile.servo2_yaw_gain * delta_yaw_deg, 0.0, 270.0)
        return tx, ty, tz, servo2, servo1


# ---------------------------------------------------------------------------
# Control thread
# ---------------------------------------------------------------------------

class ElfControlThread(threading.Thread, ControlContext):
    """
    50 Hz scheduler that polls IMU sources, runs the controller, and sends
    UART frames. Also exposes HTTP control endpoints.
    """

    def __init__(self,
                 nrf_source: Nrf24ImuSource,
                 imu2_source: Imu2Source,
                 uart_sink: UartArmSink,
                 ctrl_port: int = 8080):
        super().__init__(daemon=True)
        self.nrf_source = nrf_source
        self.imu2_source = imu2_source
        self.uart_sink = uart_sink
        self.ctrl_port = ctrl_port
        self.controller = Nrf24Controller(profile_idx=ARM_PROFILE_FAR_L3_55)
        self.http_server: Optional[HttpControlServer] = None
        self._stop_ev = threading.Event()
        self._lock = threading.RLock()
        self._status: Dict[str, Any] = {}
        self._loop_count = 0
        # Queue for scenario-mode commands computed by the vision thread;
        # the control thread consumes them at the fixed 50 Hz rate.
        self.scenario_cmd_q: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=2)
        self._power_lock = threading.Lock()
        self._power_reinit_thread: Optional[threading.Thread] = None
        self._auto_first_person_at: Optional[float] = None
        self._auto_first_person_done = False
        self._shutdown_in_progress = False
        self._voice_power_off_deadline = 0.0
        self._remote_control_locked = False

    def put_pnp_correction(self, yaw_correction: float, pitch_correction: float,
                           valid: bool = True):
        self.controller.put_pnp_correction(yaw_correction, pitch_correction, valid)

    def put_pnp_calib_sample(self, yaw_deg: float, pitch_deg: float):
        self.controller.put_pnp_calib_sample(yaw_deg, pitch_deg)

    def request_head_center(self):
        self.controller.request_head_center()

    def request_rebaseline(self):
        self.controller._remove_calib_files()
        self.controller.set_calib_mode(0)
        self.controller.request_rebaseline()

    def toggle_pitch_sign(self):
        self.controller.toggle_pitch_sign()

    def set_arm_profile(self, idx: int, source: str = "local") -> bool:
        if self._remote_control_locked and source != "remote":
            print(f"[RemoteLock] blocked profile change from {source}")
            return False
        self.controller.set_arm_profile(idx)
        if source == "remote":
            self._remote_control_locked = True
        return True

    def lock_remote_control(self):
        self._remote_control_locked = True
        print("[RemoteLock] locked by remote")

    def unlock_remote_control(self):
        self._remote_control_locked = False
        print("[RemoteLock] unlocked by remote")

    def is_remote_locked(self) -> bool:
        return self._remote_control_locked

    def set_calib_mode(self, mode: int):
        self.controller.set_calib_mode(mode)

    def set_first_person_discrete_target(self, x: int, y: int, z: int) -> bool:
        if self._remote_control_locked:
            print("[RemoteLock] blocked first-person target")
            return False
        cmd = self.controller.set_first_person_discrete_target(x, y, z)
        if self.controller.pose_mode != "first_person":
            self.set_mode("first_person")
        if not self.uart_sink.arm_powered:
            return False
        return self.uart_sink.send_arm_target(
            cmd["x"], cmd["y"], cmd["z"],
            cmd["servo2"], cmd["servo1"], cmd["flag"]
        )

    def power_on_arm(self) -> bool:
        with self._power_lock:
            if self.uart_sink.arm_powered:
                print("[Power] arm already on; ignoring power-on request")
                return False
            if not self.uart_sink.init_success:
                print("[Power] init success not received; ignoring power-on request")
                return False
            ok = self.uart_sink.send_power_on()
            if not ok:
                print("[Power] power-on frame failed")
                return False
            self._shutdown_in_progress = False
            self._voice_power_off_deadline = 0.0
            self.uart_sink.block_tx = True
            self._reset_control_after_power_on()
            self._power_reinit_thread = threading.Thread(
                target=self._finish_power_on_sequence, daemon=True
            )
            self._power_reinit_thread.start()
            print("[Power] arm power-on frame sent")
            self._update_status()
            return True

    def request_voice_power_off(self, timeout_s: float = 3.0) -> bool:
        with self._power_lock:
            if self._voice_power_off_deadline > time.perf_counter():
                print("[PowerConfirm] confirmation already pending")
                return False
            if self.controller.pose_mode != "face":
                print(f"[PowerConfirm] ignored outside FACE: {self.controller.pose_mode}")
                return False
            if not self.uart_sink.arm_powered:
                print("[PowerConfirm] arm already off")
                return False
            self._shutdown_in_progress = True
            self._auto_first_person_at = None
            if not self._send_safe_shutdown_pose_locked(wait_before_power_off=False):
                self._shutdown_in_progress = False
                self.uart_sink.block_tx = False
                print("[PowerConfirm] safe pose failed; confirmation not started")
                return False
            self._voice_power_off_deadline = time.perf_counter() + max(0.1, timeout_s)
            print(f"[PowerConfirm] show OK gesture within {timeout_s:.1f}s")
            return True

    def confirm_voice_power_off(self) -> bool:
        with self._power_lock:
            deadline = self._voice_power_off_deadline
            if deadline <= 0.0 or time.perf_counter() > deadline:
                return False
            self._voice_power_off_deadline = 0.0
            ok = self.uart_sink.send_power_off()
            if ok:
                self.flush_queues()
                print("[PowerConfirm] OK confirmed; arm power-off frame sent")
                self._update_status()
                return True
            self._shutdown_in_progress = False
            self.uart_sink.block_tx = False
            print("[PowerConfirm] power-off frame failed")
            return False

    def _check_voice_power_off_timeout(self) -> None:
        deadline = self._voice_power_off_deadline
        if deadline <= 0.0 or time.perf_counter() <= deadline:
            return
        with self._power_lock:
            if self._voice_power_off_deadline <= 0.0 or time.perf_counter() <= self._voice_power_off_deadline:
                return
            self._voice_power_off_deadline = 0.0
            self._shutdown_in_progress = False
            self.uart_sink.block_tx = False
            print("[PowerConfirm] timeout; power-off cancelled")
            self._update_status()

    def power_off_arm(self) -> bool:
        with self._power_lock:
            return self._safe_power_off_locked()

    def _safe_power_off_locked(self) -> bool:
        if not self.uart_sink.arm_powered:
            print("[Power] arm already off; ignoring power-off request")
            return False
        self._shutdown_in_progress = True
        self._voice_power_off_deadline = 0.0
        self._auto_first_person_at = None
        if not self._send_safe_shutdown_pose_locked():
            print("[Power] safe shutdown pose failed; power-off frame not sent")
            self._update_status()
            return False
        ok = self.uart_sink.send_power_off()
        if ok:
            self.flush_queues()
            print("[Power] arm power-off frame sent")
            self._update_status()
        return ok

    def _send_safe_shutdown_pose_locked(self, wait_before_power_off: bool = True) -> bool:
        self.flush_queues()
        self.controller.profile_idx = ARM_PROFILE_FAR_L3_55
        cmd = self.controller.set_pose_mode("face")
        if cmd is None:
            cmd = self.controller._make_face_home_command()
        self.uart_sink.block_tx = False
        ok = self.uart_sink.send_arm_target(
            cmd["x"], cmd["y"], cmd["z"],
            cmd["servo2"], cmd["servo1"], cmd["flag"]
        )
        if not ok:
            return False
        self.uart_sink.block_tx = True
        if wait_before_power_off:
            print(f"[Power] safe face pose sent; waiting {SAFE_POWER_OFF_WAIT_S:.1f}s before power-off")
            time.sleep(SAFE_POWER_OFF_WAIT_S)
        else:
            print("[Power] safe face pose sent; waiting for gesture confirmation")
        return True

    def _reset_control_after_power_on(self) -> None:
        self.flush_queues()
        self.controller.R_init = None
        self.controller.R_imu2_init = None
        self.controller.request_rebaseline()
        self.nrf_source.r_init_set = False
        self.nrf_source.wait_a_init = False
        self.nrf_source._valid_count = 0
        self._auto_first_person_at = None
        self._auto_first_person_done = False

    def _finish_power_on_sequence(self) -> None:
        if not isinstance(self.uart_sink, StubUartArmSink):
            print(f"[UART-Handshake] Waiting {self.uart_sink.HOMING_BLOCK_S}s for power-on homing...")
            time.sleep(self.uart_sink.HOMING_BLOCK_S)
        self._capture_a_init(timeout=10.0)
        self.uart_sink.block_tx = False
        self._auto_first_person_at = None
        self._auto_first_person_done = True
        self._update_status()
        print("[UART-Handshake] power-on TX unblocked")

    def _capture_a_init(self, timeout: float = 10.0) -> bool:
        print("[UART-Handshake] Starting A-init capture...")
        self.nrf_source.start_a_init()
        deadline = time.perf_counter() + timeout
        while not self.nrf_source.r_init_set and time.perf_counter() < deadline:
            time.sleep(0.05)
        if self.nrf_source.r_init_set:
            print("[UART-Handshake] A-init (R_init) captured")
            return True
        print("[UART-Handshake] WARNING: A-init timeout, continuing anyway")
        return False

    def _schedule_auto_first_person(self) -> None:
        self._auto_first_person_at = None
        self._auto_first_person_done = True

    def _check_auto_first_person(self) -> None:
        return

    def send_servo_test(self, k1: float, k2: float):
        # Persist k1 (J5 yaw) to /tmp/servo_calib.txt, matching upstream /servo handler.
        try:
            with open("/tmp/servo_calib.txt", "w") as f:
                f.write(f"{k1:.1f}\n")
        except Exception as e:
            print(f"[Ctrl] Failed to write /tmp/servo_calib.txt: {e}")
        profile = self.controller.profile
        tx = 0.0
        ty = profile.l4 + profile.l3
        tz = profile.l2 + profile.l1
        # flag=0x01 confirmed/stationary for servo calibration
        self.uart_sink.send_arm_target(tx, ty, tz, k1, k2, 0x01)
        print(f"[Ctrl] Servo test frame sent k1={k1} k2={k2}")

    def set_mode(self, mode: str, source: str = "local") -> bool:
        if self._remote_control_locked and source != "remote":
            print(f"[RemoteLock] blocked mode={mode} from {source}")
            return False
        home_cmd = self.controller.set_pose_mode(mode)
        if source == "remote":
            self._remote_control_locked = True
        if home_cmd is not None and self.uart_sink.arm_powered:
            self.uart_sink.send_arm_target(
                home_cmd["x"], home_cmd["y"], home_cmd["z"],
                home_cmd["servo2"], home_cmd["servo1"], home_cmd["flag"]
            )
        return True

    def switch_stream_mode(self, mode: str) -> Dict[str, Any]:
        reporter = getattr(self, "cloud_reporter", None)
        if reporter is None:
            return {"ok": False, "error": "streamer is not initialized"}
        return reporter.switch_stream_mode(mode)

    def get_mode(self) -> str:
        return self.controller.pose_mode

    def get_status(self) -> Dict[str, Any]:
        if not self._status:
            self._update_status()
        with self._lock:
            return dict(self._status)

    def flush_queues(self) -> None:
        """Drain scenario command queue used by the vision thread."""
        while True:
            try:
                self.scenario_cmd_q.get_nowait()
            except queue.Empty:
                break

    def _update_status(self):
        c = self.controller
        with self._lock:
            reporter = getattr(self, "cloud_reporter", None)
            stream_status = reporter.get_stream_status() if reporter is not None else None
            self._status = {
                "running": True,
                "stream": stream_status,
                "loop_count": self._loop_count,
                "arm_profile": c.profile.name,
                "arm_target_yaw": round(c.arm_target_yaw, 2),
                "arm_target_pitch": round(c.arm_target_pitch, 2),
                "pose_mode": c.pose_mode,
                "remote_control_locked": self._remote_control_locked,
                "head_stationary": c.head_stationary,
                "arm_stable": c.arm_stable,
                "head_center_set": c.head_center_set,
                "r_init_set": self.nrf_source.r_init_set,
                "head_pitch_sign": c.head_pitch_control_sign,
                "calib_mode": c.calib_mode,
                "pnp_calib_target_idx": c._calib_state.get("pnp_target_idx", -1),
                "pnp_calib_phase": c._calib_state.get("pnp_phase", 0),
                "grid_target_idx": c._calib_state.get("grid_target_idx", -1),
                "grid_phase": c._calib_state.get("grid_phase", 0),
                "uart_init_success": self.uart_sink.init_success,
                "arm_powered": self.uart_sink.arm_powered,
                "uart_tx_blocked": self.uart_sink.block_tx,
                "uart_move_complete": self.uart_sink.move_complete,
                "uart_homing_done": self.uart_sink.homing_done,
            }

    def start(self):
        self.nrf_source.start()
        self.imu2_source.start()
        self.uart_sink.start()

        # Startup remains powered off.  Voice or HTTP power-on runs the safe
        # homing and A-init sequence before normal arm commands are unblocked.
        if self.uart_sink.init_success:
            print("[UART-Handshake] init_success received; arm remains powered off")
        else:
            print("[UART-Handshake] init_success missing; arm remains powered off")
        self.uart_sink.block_tx = True

        if self.ctrl_port > 0:
            self.http_server = HttpControlServer(self, self.ctrl_port)
            self.http_server.start()
        super().start()

    def stop(self):
        self._shutdown_in_progress = True
        self._auto_first_person_at = None
        self._stop_ev.set()
        self.join(timeout=2.0)
        if self.uart_sink.arm_powered:
            ok = self.power_off_arm()
            if not ok and self.uart_sink.arm_powered:
                print("[Power] WARNING: suppressing direct power-off during stop after safe pose failure")
                self.uart_sink.arm_powered = False
                self.uart_sink.block_tx = True
        if self.http_server:
            self.http_server.stop()
        if self._power_reinit_thread and self._power_reinit_thread.is_alive():
            self._power_reinit_thread.join(timeout=1.0)
        self.uart_sink.stop()
        self.imu2_source.stop()
        self.nrf_source.stop()

    def run(self):
        print("[ElfControl] Thread started")
        previous_tick_start = None
        while not self._stop_ev.is_set():
            t0 = time.perf_counter()
            if previous_tick_start is not None:
                gap_ms = (t0 - previous_tick_start) * 1000.0
                if gap_ms >= 100.0:
                    print(f"[CTRL-LATE] tick_gap={gap_ms:.1f}ms")
            previous_tick_start = t0
            try:
                self._check_auto_first_person()
                self._tick()
            except Exception as e:
                print(f"[ElfControl] Tick error: {e}")
                traceback.print_exc()

            # Sleep until next 50 ms boundary
            elapsed = time.perf_counter() - t0
            if elapsed >= 0.100:
                print(f"[CTRL-LATE] tick_exec={elapsed * 1000.0:.1f}ms")
            sleep_s = max(0.0, CONTROL_PERIOD_S - elapsed)
            self._stop_ev.wait(sleep_s)
        print("[ElfControl] Thread stopped")

    def _tick(self):
        self._check_voice_power_off_timeout()
        if self._shutdown_in_progress:
            return
        # Consume vision-generated scenario commands at the fixed control rate.
        try:
            while True:
                sc = self.scenario_cmd_q.get_nowait()
                typ = sc.get("type")
                if typ == "intro":
                    self.set_mode("INTRO")
                    print("[Mode] scenario intro queued -> applied at control tick")
                elif typ == "interview":
                    self.set_mode("INTERVIEW")
                    print("[Mode] scenario interview queued -> applied at control tick")
                elif typ == "calib_mode":
                    self.set_calib_mode(sc.get("mode", 0))
                    print(f"[Mode] scenario calib_mode={sc.get('mode', 0)} applied")
                elif typ == "home":
                    self.request_rebaseline()
                    print("[Mode] scenario home queued")
                elif typ == "arm_cmd":
                    if not self.uart_sink.arm_powered:
                        continue
                    c = sc["cmd"]
                    ok = self.uart_sink.send_arm_target(
                        c["x"], c["y"], c["z"],
                        c["servo2"], c["servo1"], c["flag"]
                    )
        except queue.Empty:
            pass

        head_imu = self.nrf_source.poll()
        imu2 = self.imu2_source.poll()
        now_us = int(time.perf_counter() * 1_000_000)

        cmd = self.controller.update(
            head_imu, imu2,
            r_init_set=self.nrf_source.r_init_set,
            now_us=now_us,
        )

        if cmd is not None and self.uart_sink.arm_powered:
            # Calibration/scenario commands intentionally have no diagnostic
            # "info" payload; only the wire fields below are mandatory.
            flag = cmd["flag"]
            ok = self.uart_sink.send_arm_target(
                cmd["x"], cmd["y"], cmd["z"],
                cmd["servo2"], cmd["servo1"], flag
            )

        self._loop_count += 1
        if self._loop_count % 20 == 0:
            self._update_status()


# ---------------------------------------------------------------------------
# Convenience constructor for the async PnP demo
# ---------------------------------------------------------------------------

def make_elf_control_thread(stub: bool = True,
                            uart_port: str = "/dev/ttyS9",
                            ctrl_port: int = 8080,
                            nrf_backend: str = "stm32_uart",
                            stm32_uart_port: Optional[str] = None,
                            stm32_uart_baud: int = 460_800,
                            ) -> ElfControlThread:
    """Convenience constructor for ElfControlThread.

    Args:
        stub: If True, use synthetic stub backends.
        uart_port: Serial port for the arm UART sink (only used by the
            legacy direct-H7 backend, which is no longer exposed).
        ctrl_port: HTTP control port.
        nrf_backend: "stm32_uart" for the STM32F103 DataHub bridge (default),
            "dk2500" for Dk2500Nrf24ImuSource (DK-2500 IT8786 GPIO bit-banged
            SPI), or use stub=True for offline testing.
        stm32_uart_port: UART port for the STM32 bridge (used when
            nrf_backend="stm32_uart").  If None, auto-detect by probing USB
            serial ports for A5 bridge frames at stm32_uart_baud.
        stm32_uart_baud: UART baudrate for the STM32 bridge.
    """
    bridge_verbose = os.environ.get("BRIDGE_VERBOSE", "0") == "1"
    if stub:
        nrf = StubNrf24ImuSource()
        imu2 = StubImu2Source()
        uart = StubUartArmSink()
    elif nrf_backend == "stm32_uart":
        # STM32 bridge: NRF24 + IMU2 + arm control over one UART.
        from elf_control_chain_stm32 import (
            Stm32DataHubBridge,
            BridgeNrf24ImuSource,
            BridgeImu2Source,
            BridgeUartArmSink,
        )
        bridge = Stm32DataHubBridge(
            port=stm32_uart_port,
            baudrate=stm32_uart_baud,
            verbose=bridge_verbose,
        )
        nrf = BridgeNrf24ImuSource(bridge)
        imu2 = BridgeImu2Source(bridge)
        uart = BridgeUartArmSink(bridge)
    elif nrf_backend == "dk2500":
        # Local import avoids a circular dependency with
        # elf_control_chain_dk2500.py, which imports from this module.
        from elf_control_chain_dk2500 import Dk2500Nrf24ImuSource
        nrf = Dk2500Nrf24ImuSource()
        imu2 = StubImu2Source()
        uart = SerialUartArmSink(port=uart_port)
    else:
        raise ValueError(f"Unsupported nrf_backend: {nrf_backend!r}")
    return ElfControlThread(nrf, imu2, uart, ctrl_port=ctrl_port)
