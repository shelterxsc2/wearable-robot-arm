# -*- coding: utf-8 -*-
"""Unit tests for the STM32F103 DataHub bridge classified frame protocol."""
from __future__ import annotations

import os
import struct
import sys
import unittest
from io import BytesIO
from typing import List

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from elf_control_chain_stm32 import (
    Stm32DataHubBridge,
    BridgeNrf24ImuSource,
    BridgeImu2Source,
    BridgeUartArmSink,
    build_downlink_frame,
    build_arm_target_frame,
    DOWNLINK_CMD_HEARTBEAT,
    DOWNLINK_CMD_ARM_TARGET,
    H7_INIT_FRAME,
    H7_RETRACT_FRAME,
    TYPE_WAIST_IMU,
    TYPE_NRF_IMU,
    TYPE_ARM_STREAM,
    WAIST_FRAME_LEN,
    NRF_PAYLOAD_LEN,
    ARM_VOFA_FRAME_LEN,
)


def _wit_checksum(frame: bytes) -> int:
    return sum(frame[:10]) & 0xFF


def _build_wit_angle_frame(roll: float, pitch: float, yaw: float) -> bytes:
    payload = struct.pack(
        "<hhh",
        int(round(roll * 32768.0 / 180.0)),
        int(round(pitch * 32768.0 / 180.0)),
        int(round(yaw * 32768.0 / 180.0)),
    )
    frame = bytes([0x55, 0x53]) + payload + bytes(2)
    frame = frame[:10] + bytes([_wit_checksum(frame)])
    return frame


def _build_wit_gyro_frame(wx: float, wy: float, wz: float) -> bytes:
    payload = struct.pack(
        "<hhh",
        int(round(wx * 32768.0 / 2000.0)),
        int(round(wy * 32768.0 / 2000.0)),
        int(round(wz * 32768.0 / 2000.0)),
    )
    frame = bytes([0x55, 0x52]) + payload + bytes(2)
    frame = frame[:10] + bytes([_wit_checksum(frame)])
    return frame


def _build_nrf_payload(roll: float, pitch: float, yaw: float,
                       wx: float, wy: float, wz: float) -> bytes:
    def q(v: float) -> int:
        # Scale to int16; 1.0 maps to 32767 to avoid overflow.
        return max(-32768, min(32767, int(round(v * 32767.0))))

    def w(v: float) -> int:
        return max(-32768, min(32767, int(round(v * 32768.0 / 2000.0))))

    orient = bytes([0x55, 0x59]) + struct.pack("<hhhh", q(1.0), q(0.0), q(0.0), q(0.0))
    orient = orient[:10] + bytes([sum(orient[:10]) & 0xFF])
    # Note: parse_nrf24_imu_payload converts quaternion to Euler, so we use a
    # simple identity quaternion here and rely on the gyro frame for motion.
    gyro = bytes([0x55, 0x52]) + struct.pack("<hhh", w(wx), w(wy), w(wz)) + bytes(2)
    gyro = gyro[:10] + bytes([sum(gyro[:10]) & 0xFF])
    return orient + gyro


def _build_vofa_frame(floats: List[float]) -> bytes:
    return struct.pack(f"<{len(floats)}f", *floats)


class _FakeSerial:
    """In-memory serial port for testing."""

    def __init__(self):
        self._rx = BytesIO()
        self._tx = BytesIO()
        self.is_open = True
        self.timeout = 0.0

    def read(self, n: int) -> bytes:
        data = self._rx.read(n)
        if not data:
            return b""
        return data

    def write(self, data: bytes):
        self._tx.write(data)

    def flush(self):
        pass

    def close(self):
        self.is_open = False

    def feed(self, data: bytes):
        pos = self._rx.tell()
        self._rx.seek(0, 2)
        self._rx.write(data)
        self._rx.seek(pos)

    def get_written(self) -> bytes:
        return self._tx.getvalue()


class TestDataHubBridgeProtocol(unittest.TestCase):

    def test_build_and_parse_waist_angle(self):
        bridge = Stm32DataHubBridge()
        payload = _build_wit_angle_frame(1.23, 2.34, 3.45)
        frame = Stm32DataHubBridge.build_bridge_frame(
            TYPE_WAIST_IMU, seq=5, payload=payload
        )
        self.assertEqual(frame[0], 0xA5)
        self.assertEqual(frame[1], TYPE_WAIST_IMU)
        self.assertEqual(frame[2], WAIST_FRAME_LEN)
        self.assertEqual(frame[3], 5)

        leftover = bridge._parse_buffer(frame + b"\x00\x11")
        self.assertEqual(leftover, b"\x11")

        waist = bridge.get_waist_imu()
        self.assertAlmostEqual(waist["roll"], 1.23, places=2)
        self.assertAlmostEqual(waist["pitch"], 2.34, places=2)
        self.assertAlmostEqual(waist["yaw"], 3.45, places=2)
        self.assertTrue(waist["valid"])

    def test_build_and_parse_waist_gyro(self):
        bridge = Stm32DataHubBridge()
        angle = _build_wit_angle_frame(0.0, 0.0, 0.0)
        # Use exact multiples of 2000/32768 to avoid quantization error.
        gyro = _build_wit_gyro_frame(2000.0 / 32, 4000.0 / 32, 6000.0 / 32)
        frame = (
            Stm32DataHubBridge.build_bridge_frame(TYPE_WAIST_IMU, 1, angle) +
            Stm32DataHubBridge.build_bridge_frame(TYPE_WAIST_IMU, 2, gyro)
        )
        bridge._parse_buffer(frame)

        waist = bridge.get_waist_imu()
        self.assertAlmostEqual(waist["wx"], 2000.0 / 32, places=2)
        self.assertAlmostEqual(waist["wy"], 4000.0 / 32, places=2)
        self.assertAlmostEqual(waist["wz"], 6000.0 / 32, places=2)

    def test_build_and_parse_nrf_imu(self):
        bridge = Stm32DataHubBridge()
        # Use exact multiples of 2000/32768 for gyro to avoid quantization.
        payload = _build_nrf_payload(10.0, 20.0, 30.0,
                                     2000.0 / 32, 4000.0 / 32, 6000.0 / 32)
        self.assertEqual(len(payload), NRF_PAYLOAD_LEN)
        frame = Stm32DataHubBridge.build_bridge_frame(
            TYPE_NRF_IMU, seq=7, payload=payload
        )
        bridge._parse_buffer(frame)

        head = bridge.get_head_imu()
        self.assertTrue(head["imu_valid"])
        # parse_nrf24_imu_payload normalizes quaternion and converts to Euler.
        # With identity quaternion, roll/pitch/yaw should all be near zero.
        self.assertAlmostEqual(head["roll"], 0.0, places=2)
        self.assertAlmostEqual(head["pitch"], 0.0, places=2)
        self.assertAlmostEqual(head["yaw"], 0.0, places=2)
        self.assertAlmostEqual(head["wx"], 2000.0 / 32, places=2)
        self.assertAlmostEqual(head["wy"], 4000.0 / 32, places=2)
        self.assertAlmostEqual(head["wz"], 6000.0 / 32, places=2)
        self.assertTrue(head["quat_valid"])
        qnorm = (head["qw"] ** 2 + head["qx"] ** 2 +
                 head["qy"] ** 2 + head["qz"] ** 2) ** 0.5
        self.assertAlmostEqual(qnorm, 1.0, places=3)

    def test_build_and_parse_arm_stream(self):
        bridge = Stm32DataHubBridge()
        floats = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, float("inf")]
        payload = _build_vofa_frame(floats)
        self.assertEqual(len(payload), ARM_VOFA_FRAME_LEN)
        frame = Stm32DataHubBridge.build_bridge_frame(
            TYPE_ARM_STREAM, seq=9, payload=payload
        )
        bridge._parse_buffer(frame)

        arm = bridge.get_arm_debug_vofa()
        self.assertTrue(arm["valid"])
        self.assertAlmostEqual(arm["s"], 1.0, places=5)
        self.assertAlmostEqual(arm["v"], 2.0, places=5)
        self.assertAlmostEqual(arm["a"], 3.0, places=5)
        self.assertAlmostEqual(arm["motor0_target"], 4.0, places=5)
        self.assertAlmostEqual(arm["motor0_actual"], 5.0, places=5)
        self.assertAlmostEqual(arm["error_s"], 6.0, places=5)
        self.assertAlmostEqual(arm["tail"], float("inf"))

    def test_parse_variable_arm_stream_formats(self):
        bridge = Stm32DataHubBridge()

        short_payload = _build_vofa_frame([1.0, 2.0, 3.0, float("inf")])
        bridge._parse_buffer(Stm32DataHubBridge.build_bridge_frame(
            TYPE_ARM_STREAM, seq=10, payload=short_payload
        ))
        stream = bridge.get_arm_stream()
        self.assertEqual(stream["format"], "vofa4")
        self.assertEqual(stream["floats"][:3], (1.0, 2.0, 3.0))

        sink = BridgeUartArmSink(bridge)
        sink.move_complete = False
        text_payload = b"move complete\r\n"
        bridge._parse_buffer(Stm32DataHubBridge.build_bridge_frame(
            TYPE_ARM_STREAM, seq=11, payload=text_payload
        ))
        stream = bridge.get_arm_stream()
        self.assertEqual(stream["format"], "ascii")
        self.assertEqual(stream["text"], "move complete")
        self.assertTrue(stream["move_complete"])
        self.assertTrue(sink.move_complete)

    def test_invalid_arm_length_resynchronizes(self):
        bridge = Stm32DataHubBridge()
        invalid_header = bytes([0xA5, TYPE_ARM_STREAM, 33, 1])
        valid_payload = b"move complete\r\n"
        valid_frame = Stm32DataHubBridge.build_bridge_frame(
            TYPE_ARM_STREAM, seq=12, payload=valid_payload
        )
        bridge._parse_buffer(invalid_header + valid_frame)
        self.assertEqual(bridge.get_arm_stream()["text"], "move complete")

    def test_crc_rejection(self):
        bridge = Stm32DataHubBridge()
        payload = _build_wit_angle_frame(10.0, 0.0, 0.0)
        frame = bytearray(Stm32DataHubBridge.build_bridge_frame(
            TYPE_WAIST_IMU, seq=1, payload=payload
        ))
        frame[-1] ^= 0xFF
        bridge._parse_buffer(bytes(frame))
        self.assertFalse(bridge.get_waist_imu()["valid"])

    def test_unknown_type_rejection(self):
        bridge = Stm32DataHubBridge()
        frame = Stm32DataHubBridge.build_bridge_frame(0xFF, 1, b"\x01\x02\x03")
        bridge._parse_buffer(frame)
        # Should not crash; state remains default.
        self.assertFalse(bridge.get_waist_imu()["valid"])

    def test_default_baudrate_is_460800(self):
        bridge = Stm32DataHubBridge()
        self.assertEqual(bridge.baudrate, 460800)

    def test_build_downlink_frame(self):
        frame = build_downlink_frame(DOWNLINK_CMD_HEARTBEAT, b"")
        self.assertEqual(frame, b"\xAA\x55\x00\x01\x01")

        payload = b"\x01\x02\x03"
        frame = build_downlink_frame(DOWNLINK_CMD_ARM_TARGET, payload)
        self.assertEqual(frame[:2], b"\xAA\x55")
        self.assertEqual(frame[2], len(payload))
        self.assertEqual(frame[3], DOWNLINK_CMD_ARM_TARGET)
        self.assertEqual(frame[4:4 + len(payload)], payload)
        self.assertEqual(frame[-1], sum(frame[2:-1]) & 0xFF)

    def test_build_arm_target_frame(self):
        frame = build_arm_target_frame(1.0, 2.0, 3.0, 4.0, 5.0, 0xAB)
        self.assertEqual(frame[:2], b"\xAA\x55")
        self.assertEqual(frame[2], 11)   # LEN
        self.assertEqual(frame[3], DOWNLINK_CMD_ARM_TARGET)
        x, y, z, k1, k2, flag = struct.unpack("<hhhhhB", frame[4:15])
        self.assertEqual((x, y, z, k1, k2, flag), (1, 2, 3, 4, 5, 0xAB))
        self.assertEqual(frame[15], sum(frame[2:15]) & 0xFF)

    def test_send_target_frame(self):
        bridge = Stm32DataHubBridge()
        fake = _FakeSerial()
        bridge._serial = fake

        ok = bridge.send_arm_target(1.0, 2.0, 3.0, 4.0, 5.0, 0x01)
        self.assertTrue(ok)

        written = fake.get_written()
        self.assertEqual(written[:2], b"\xAA\x55")
        self.assertEqual(written[2], 11)   # LEN
        self.assertEqual(written[3], DOWNLINK_CMD_ARM_TARGET)
        x, y, z, k1, k2, flag = struct.unpack("<hhhhhB", written[4:15])
        self.assertEqual((x, y, z, k1, k2, flag), (1, 2, 3, 4, 5, 0x01))
        self.assertEqual(written[15], sum(written[2:15]) & 0xFF)

    def test_send_h7_special_commands(self):
        bridge = Stm32DataHubBridge()
        fake = _FakeSerial()
        bridge._serial = fake

        self.assertTrue(bridge.send_h7_init())
        self.assertEqual(fake.get_written(), H7_INIT_FRAME)

        fake = _FakeSerial()
        bridge._serial = fake
        self.assertTrue(bridge.send_h7_retract())
        self.assertEqual(fake.get_written(), H7_RETRACT_FRAME)

    def test_bridge_source_counts_valid_frames(self):
        bridge = Stm32DataHubBridge()
        payload = _build_nrf_payload(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        src = BridgeNrf24ImuSource(bridge)
        self.assertFalse(src.r_init_set)
        src.start_a_init()
        for seq in range(5):
            frame = Stm32DataHubBridge.build_bridge_frame(
                TYPE_NRF_IMU, seq=seq, payload=payload
            )
            bridge._parse_buffer(frame)
            src.poll()
        self.assertTrue(src.r_init_set)

    def test_bridge_imu2_source(self):
        bridge = Stm32DataHubBridge()
        angle = _build_wit_angle_frame(5.5, 6.6, 7.7)
        frame = Stm32DataHubBridge.build_bridge_frame(
            TYPE_WAIST_IMU, seq=0, payload=angle
        )
        bridge._parse_buffer(frame)
        src = BridgeImu2Source(bridge)
        data = src.poll()
        self.assertAlmostEqual(data["roll"], 5.5, places=2)
        self.assertAlmostEqual(data["pitch"], 6.6, places=2)
        self.assertAlmostEqual(data["yaw"], 7.7, places=2)
        self.assertTrue(data["valid"])

    def test_bridge_arm_sink_power_on_after_init_success(self):
        bridge = Stm32DataHubBridge()
        fake = _FakeSerial()
        bridge._serial = fake

        sink = BridgeUartArmSink(bridge)
        self.assertFalse(sink.send_power_on())

        bridge._parse_buffer(Stm32DataHubBridge.build_bridge_frame(
            TYPE_ARM_STREAM, seq=20, payload=b"init success\r\n"
        ))
        self.assertTrue(sink.init_success)
        self.assertTrue(sink.send_power_on())

        ok = sink.send_arm_target(10, 20, 30, 40, 50, 0x01)
        self.assertTrue(ok)

        written = fake.get_written()
        self.assertEqual(written[:len(H7_INIT_FRAME)], H7_INIT_FRAME)
        target = written[len(H7_INIT_FRAME):]
        self.assertEqual(target[:2], b"\xAA\x55")
        self.assertEqual(target[2], 11)
        self.assertEqual(target[3], DOWNLINK_CMD_ARM_TARGET)
        x, y, z, k1, k2, flag = struct.unpack("<hhhhhB", target[4:15])
        self.assertEqual((x, y, z, k1, k2, flag), (10, 20, 30, 40, 50, 0x01))
        self.assertEqual(target[15], sum(target[2:15]) & 0xFF)

    def test_bridge_arm_sink_start_does_not_send_init_frame(self):
        bridge = Stm32DataHubBridge()
        fake = _FakeSerial()
        bridge._serial = fake
        sink = BridgeUartArmSink(bridge)
        sink.init_success = True
        sink.start()

        self.assertTrue(sink.init_success)
        self.assertEqual(fake.get_written(), b"")

    def test_bridge_arm_sink_stop_sends_retract_only_when_powered(self):
        bridge = Stm32DataHubBridge()
        fake = _FakeSerial()
        bridge._serial = fake
        sink = BridgeUartArmSink(bridge)

        sink.stop()
        self.assertEqual(fake.get_written(), b"")

        bridge._parse_buffer(Stm32DataHubBridge.build_bridge_frame(
            TYPE_ARM_STREAM, seq=21, payload=b"init success\r\n"
        ))
        self.assertTrue(sink.send_power_on())
        sink.stop()
        self.assertEqual(fake.get_written(), H7_INIT_FRAME + H7_RETRACT_FRAME)


if __name__ == "__main__":
    unittest.main()
