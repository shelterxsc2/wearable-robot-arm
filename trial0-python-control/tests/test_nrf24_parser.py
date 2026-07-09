# -*- coding: utf-8 -*-
"""Unit tests for the NRF24 IMU payload parser.

The parser is shared by Dk2500Nrf24ImuSource, BridgeNrf24ImuSource and
StubNrf24ImuSource and mirrors nrf24_linux.c::nrf24_parse_22b().
"""
from __future__ import annotations

import math
import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from elf_control_chain import parse_nrf24_imu_payload


def _checksum(frame: bytearray) -> int:
    return sum(frame[:10]) & 0xFF


def _make_orientation_frame(q0: int, q1: int, q2: int, q3: int) -> bytes:
    frame = bytearray(11)
    frame[0] = 0x55
    frame[1] = 0x59
    struct.pack_into("<hhhh", frame, 2, q0, q1, q2, q3)
    frame[10] = _checksum(frame)
    return bytes(frame)


def _make_gyro_frame(wx: int, wy: int, wz: int) -> bytes:
    frame = bytearray(11)
    frame[0] = 0x55
    frame[1] = 0x52
    struct.pack_into("<hhh", frame, 2, wx, wy, wz)
    frame[10] = _checksum(frame)
    return bytes(frame)


class TestNrf24Parser(unittest.TestCase):

    def test_invalid_length(self):
        result = parse_nrf24_imu_payload(b"\x55" * 10)
        self.assertFalse(result["imu_valid"])
        self.assertFalse(result["quat_valid"])

    def test_bad_header(self):
        gyro = _make_gyro_frame(0, 0, 16384)  # 1000 deg/s on Z
        payload = b"\x00" + gyro[1:] + gyro
        result = parse_nrf24_imu_payload(payload)
        # First frame bad header -> ignored; second frame gyro only -> no orientation
        self.assertFalse(result["imu_valid"])
        self.assertIn("wz", result)
        self.assertAlmostEqual(result["wz"], 1000.0, places=1)

    def test_bad_checksum(self):
        orient = bytearray(_make_orientation_frame(23170, 0, 0, 23170))
        orient[10] ^= 0xFF
        gyro = _make_gyro_frame(0, 0, 0)
        payload = bytes(orient) + gyro
        result = parse_nrf24_imu_payload(payload)
        # Orientation checksum bad; only gyro parsed -> not imu_valid
        self.assertFalse(result["imu_valid"])

    def test_90_deg_yaw(self):
        # 90-degree rotation around Z: w=z=cos(45 deg)=sqrt(2)/2.
        q = int(round(32768 * math.sqrt(2) / 2))
        orient = _make_orientation_frame(q, 0, 0, q)
        gyro = _make_gyro_frame(0, 0, 0)
        result = parse_nrf24_imu_payload(orient + gyro)
        self.assertTrue(result["imu_valid"])
        self.assertTrue(result["quat_valid"])
        self.assertAlmostEqual(result["roll"], 0.0, places=1)
        self.assertAlmostEqual(result["pitch"], 0.0, places=1)
        self.assertAlmostEqual(result["yaw"], 90.0, places=1)

    def test_gyro_scaling(self):
        # 1000 deg/s on each axis.
        gyro_raw = int(round(1000 * 32768 / 2000))
        orient = _make_orientation_frame(32767, 0, 0, 0)
        gyro = _make_gyro_frame(gyro_raw, gyro_raw, gyro_raw)
        result = parse_nrf24_imu_payload(orient + gyro)
        self.assertTrue(result["imu_valid"])
        self.assertAlmostEqual(result["wx"], 1000.0, places=1)
        self.assertAlmostEqual(result["wy"], 1000.0, places=1)
        self.assertAlmostEqual(result["wz"], 1000.0, places=1)

    def test_both_orders(self):
        orient = _make_orientation_frame(32767, 0, 0, 0)
        gyro = _make_gyro_frame(0, 0, 1000)
        result1 = parse_nrf24_imu_payload(orient + gyro)
        result2 = parse_nrf24_imu_payload(gyro + orient)
        self.assertTrue(result1["imu_valid"])
        self.assertTrue(result2["imu_valid"])
        self.assertAlmostEqual(result1["roll"], result2["roll"], places=1)
        self.assertAlmostEqual(result1["wz"], result2["wz"], places=1)


if __name__ == "__main__":
    unittest.main()
