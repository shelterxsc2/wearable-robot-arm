# -*- coding: utf-8 -*-
"""Unit tests for upstream porting changes (non-STM32)."""
from __future__ import annotations

import os
import struct
import sys
import tempfile
import threading
import time
import unittest
from io import BytesIO
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import elf_control_chain as ecc
from elf_control_chain import (
    ARM_PROFILE_FAR_L3_55,
    ARM_PROFILE_MID_L3_40,
    ARM_PROFILES,
    ElfControlThread,
    FIRST_PERSON_BASE_J4_DEG,
    FIRST_PERSON_BASE_J5_DEG,
    FIRST_PERSON_BASE_X_CM,
    FIRST_PERSON_BASE_Y_CM,
    FIRST_PERSON_BASE_Z_CM,
    KI_PNP,
    KI_PNP_PITCH_BIAS,
    Nrf24Controller,
    PNP_INTEGRAL_DEADBAND_DEG,
    SerialUartArmSink,
    StubImu2Source,
    StubNrf24ImuSource,
    StubUartArmSink,
    UartArmSink,
)


class _FakeSerial:
    """In-memory serial port for testing."""

    def __init__(self):
        self._rx = BytesIO()
        self._tx = BytesIO()
        self.is_open = True
        self.timeout = 0.0

    def read(self, n: int) -> bytes:
        data = self._rx.read(n)
        return data if data else b""

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


class TestPnPParameters(unittest.TestCase):

    def test_constants_aligned_with_upstream(self):
        self.assertEqual(KI_PNP, 0.04)
        self.assertEqual(KI_PNP_PITCH_BIAS, 0.02)
        self.assertEqual(PNP_INTEGRAL_DEADBAND_DEG, 3.0)

    def test_profiles_have_compensation_tables(self):
        for p in ARM_PROFILES:
            self.assertTrue(len(p.yaw_calib) > 0)
            self.assertTrue(len(p.pitch_calib) > 0)


class TestHandshakeAndHoming(unittest.TestCase):

    def test_ff_frame_constant(self):
        self.assertEqual(
            UartArmSink.FF_VERIFY_FRAME,
            bytes([0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA]),
        )

    def test_serial_sink_sends_ff_and_blocks(self):
        sink = SerialUartArmSink(port="/dev/fake", send_ff_frame=True)
        fake = _FakeSerial()
        sink._serial = fake
        sink.init_success = True

        # Manually trigger the post-init handshake.
        sink._send_raw(UartArmSink.FF_VERIFY_FRAME)
        sink._trigger_homing_block()

        written = fake.get_written()
        self.assertTrue(written.startswith(UartArmSink.FF_VERIFY_FRAME))
        self.assertTrue(sink.block_tx)
        # TX should be rejected during homing block.
        self.assertFalse(sink.send_arm_target(1, 2, 3, 4, 5, 0x01))

    def test_stub_sink_honors_block_tx(self):
        sink = StubUartArmSink()
        sink.block_tx = True
        self.assertFalse(sink.send_arm_target(1, 2, 3, 4, 5, 0x01))

    def test_power_state_suppresses_duplicate_commands(self):
        sink = StubUartArmSink()
        self.assertTrue(sink.arm_powered)
        self.assertFalse(sink.send_power_on())
        self.assertTrue(sink.send_power_off())
        self.assertFalse(sink.arm_powered)
        self.assertFalse(sink.send_power_off())
        self.assertFalse(sink.send_arm_target(1, 2, 3, 4, 5, 0x01))
        self.assertTrue(sink.send_power_on())
        sink.block_tx = False
        self.assertTrue(sink.send_arm_target(1, 2, 3, 4, 5, 0x01))

    def test_power_off_from_first_person_sends_safe_face_pose_first(self):
        class RecordingSink(StubUartArmSink):
            def __init__(self):
                super().__init__()
                self.targets = []
                self.powered_off = False

            def send_arm_target(self, x, y, z, k1, k2, flag):
                ok = super().send_arm_target(x, y, z, k1, k2, flag)
                if ok:
                    self.targets.append((x, y, z, k1, k2, flag))
                return ok

            def send_power_off(self):
                ok = super().send_power_off()
                if ok:
                    self.powered_off = True
                return ok

        old_wait = ecc.SAFE_POWER_OFF_WAIT_S
        ecc.SAFE_POWER_OFF_WAIT_S = 0.0
        try:
            sink = RecordingSink()
            thread = ElfControlThread(StubNrf24ImuSource(), StubImu2Source(), sink, ctrl_port=0)
            thread.set_mode("first_person")
            sink.targets.clear()

            self.assertTrue(thread.power_off_arm())

            self.assertTrue(sink.powered_off)
            self.assertFalse(sink.arm_powered)
            self.assertEqual(thread.get_mode(), "face")
            self.assertEqual(thread.controller.profile_idx, ARM_PROFILE_FAR_L3_55)
            self.assertGreaterEqual(len(sink.targets), 1)
            _x, y, _z, _j5, _j4, flag = sink.targets[0]
            self.assertEqual(flag, 0x01)
            self.assertGreater(y, 80.0)
        finally:
            ecc.SAFE_POWER_OFF_WAIT_S = old_wait


class TestScenarioModes(unittest.TestCase):

    def test_mode_switching(self):
        ctrl = Nrf24Controller()
        self.assertEqual(ctrl.pose_mode, "face")
        ctrl.set_pose_mode("intro")
        self.assertEqual(ctrl.pose_mode, "intro")
        ctrl.set_pose_mode("interview")
        self.assertEqual(ctrl.pose_mode, "interview")
        home = ctrl.set_pose_mode("face")
        self.assertIsNotNone(home)
        self.assertIn("servo1", home)
        self.assertIn("servo2", home)

    def test_invalid_mode_rejected(self):
        ctrl = Nrf24Controller()
        self.assertIsNone(ctrl.set_pose_mode("unknown"))

    def test_first_person_initial_pose_and_head_mapping(self):
        ctrl = Nrf24Controller()
        cmd = ctrl.set_pose_mode("first_person")
        self.assertIsNotNone(cmd)
        self.assertEqual(ctrl.pose_mode, "first_person")
        self.assertEqual(cmd["x"], FIRST_PERSON_BASE_X_CM)
        self.assertEqual(cmd["y"], FIRST_PERSON_BASE_Y_CM)
        self.assertEqual(cmd["z"], FIRST_PERSON_BASE_Z_CM)
        self.assertEqual(cmd["servo1"], FIRST_PERSON_BASE_J4_DEG)
        self.assertEqual(cmd["servo2"], FIRST_PERSON_BASE_J5_DEG)

        cmd = ctrl._update_first_person_control(vec_yaw=5.0, vec_pitch=3.0, now_us=1_000_000)
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd["servo2"], FIRST_PERSON_BASE_J5_DEG + 5.0)
        self.assertEqual(cmd["servo1"], FIRST_PERSON_BASE_J4_DEG - 3.0)

        ctrl.toggle_pitch_sign()
        cmd = ctrl._update_first_person_control(vec_yaw=5.0, vec_pitch=6.0, now_us=1_200_000)
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd["servo1"], FIRST_PERSON_BASE_J4_DEG + 6.0)

    def test_intro_control_basic(self):
        ctrl = Nrf24Controller()
        ctrl.set_pose_mode("intro")
        # Detection with right wrist at center -> should eventually command.
        dets = [{
            "x1": 100, "y1": 100, "x2": 200, "y2": 300,
            "score": 0.9,
            "kps": [{"x": 150, "y": 150, "visibility": 1.0} for _ in range(17)],
        }]
        # Set torso center manually.
        dets[0]["kps"][5] = {"x": 320, "y": 240, "visibility": 1.0}
        dets[0]["kps"][6] = {"x": 320, "y": 240, "visibility": 1.0}
        dets[0]["kps"][11] = {"x": 320, "y": 240, "visibility": 1.0}
        dets[0]["kps"][12] = {"x": 320, "y": 240, "visibility": 1.0}
        now_us = int(time.perf_counter() * 1_000_000) + 1_000_000
        cmd = ctrl.update_intro_control(dets, 640, 480, now_us=now_us)
        self.assertIsNotNone(cmd)
        self.assertIn("servo1", cmd)
        self.assertIn("servo2", cmd)

    def test_interview_control_basic(self):
        ctrl = Nrf24Controller()
        ctrl.set_pose_mode("interview")
        dets = [{
            "x1": 100, "y1": 100, "x2": 200, "y2": 300,
            "score": 0.9,
            "kps": [{"x": 150, "y": 150, "visibility": 1.0} for _ in range(17)],
        }]
        now_us = int(time.perf_counter() * 1_000_000) + 1_000_000
        cmd = ctrl.update_interview_control(dets, 640, 480, now_us=now_us)
        self.assertIsNotNone(cmd)


class TestCalibrationModes(unittest.TestCase):

    def test_servo_calib_mode_returns_command(self):
        ctrl = Nrf24Controller()
        ctrl.set_calib_mode(1)
        head_imu = {"imu_valid": True, "roll": 5.0, "pitch": 0.0, "yaw": 10.0}
        now_us = 1_000_000
        cmd = ctrl.update(head_imu, {"valid": False}, r_init_set=False, now_us=now_us)
        # First tick locks position.
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd["flag"], 0x01)

    def test_pnp_yaw_calib_advances_targets(self):
        ctrl = Nrf24Controller()
        ctrl.set_calib_mode(3)
        head_imu = {"imu_valid": True, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        now_us = 100_000
        cmd = ctrl.update(head_imu, {"valid": False}, r_init_set=False, now_us=now_us)
        self.assertIsNotNone(cmd)
        self.assertEqual(ctrl._calib_state["pnp_target_idx"], 0)
        self.assertEqual(ctrl._calib_state["pnp_phase"], 1)

    def test_calibration_csv_written(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = os.path.join(tmpdir, "pnp_yaw_calib.csv")
            ctrl = Nrf24Controller()
            ctrl._write_pnp_csv(path, 15.0, 5.5, 10, is_yaw=True)
            with open(path, "r") as f:
                lines = f.readlines()
            self.assertEqual(len(lines), 2)
            self.assertIn("target_yaw_deg", lines[0])


class TestHttpEndpoints(unittest.TestCase):

    def test_status_includes_pose_mode(self):
        nrf = StubNrf24ImuSource()
        imu2 = StubImu2Source()
        uart = StubUartArmSink()
        thread = ElfControlThread(nrf, imu2, uart, ctrl_port=0)
        status = thread.get_status()
        self.assertEqual(status["pose_mode"], "face")

    def test_set_mode_via_context(self):
        nrf = StubNrf24ImuSource()
        imu2 = StubImu2Source()
        uart = StubUartArmSink()
        thread = ElfControlThread(nrf, imu2, uart, ctrl_port=0)
        thread.set_mode("intro")
        self.assertEqual(thread.get_mode(), "intro")
        # Exiting scenario to face sends a home command.
        thread.set_mode("face")
        self.assertEqual(thread.get_mode(), "face")


class TestBleRemote(unittest.TestCase):

    def test_frame_dispatch_logic(self):
        from tools.ble_remote import BleRemoteListener
        listener = BleRemoteListener(ctrl_url="http://example.com")
        calls = []
        listener._http_post = lambda path: calls.append(path) or True
        listener.feed_test_frame(0x01, 0x01)  # profile far_l3_55
        listener.feed_test_frame(0x01, 0x01)  # suppressed
        listener.feed_test_frame(0x02, 0x01)  # mode intro
        listener.feed_test_frame(0x03, 0x00)  # toggle pitch sign
        self.assertEqual(calls, ["/profile?idx=1", "/mode?type=intro", "/cmd?action=toggle_pitch_sign"])


if __name__ == "__main__":
    unittest.main()
