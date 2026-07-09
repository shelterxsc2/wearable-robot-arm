#!/usr/bin/env python3
"""STM32F103 DataHub bridge loop-back test (synchronous ping-pong).

This script runs on the PC and acts as both:
  - host on the bridge port (F103 USART3, 460800) sending arm target commands
  - robotic-arm simulator on the arm port (F103 USART2, 115200) receiving
    commands and replying with VOFA JustFloat 28-byte frames.

Expected F103 routing:
  host -> USART3 -> F103 -> USART2 -> arm
  arm  -> USART2 -> F103 -> USART3 -> host

The host uses the framed downlink protocol:
    AA 55 0B 30 <11 payload bytes> CRC8
where CRC8 = 0x0B + 0x30 + sum(payload).

The arm replies with a 28-byte VOFA JustFloat frame whose last float is
+inf (little-endian bytes 00 00 80 7F), which the F103 bridges as TYPE=0x53.

To match the real control flow (command -> response -> next command) and avoid
artefacts from simultaneous full-duplex traffic on cheap USB-TTL adapters, this
version uses a synchronous ping-pong loop instead of independent TX/RX threads.
"""
from __future__ import annotations

import argparse
import glob
import os
import struct
import sys
import time
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except Exception as exc:  # pragma: no cover
    raise RuntimeError("This script requires 'pyserial'") from exc

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from stm32_bridge_utils import (
    DEFAULT_ARM_BAUD,
    DEFAULT_BRIDGE_BAUD,
    drain_serial_port,
    detect_serial_ports as stm32_detect_serial_ports,
)

BRIDGE_SOF0 = 0xA5
BRIDGE_TYPE_WAIST_IMU = 0x51
BRIDGE_TYPE_NRF_IMU = 0x52
BRIDGE_TYPE_ARM = 0x53
BRIDGE_TYPES = (BRIDGE_TYPE_WAIST_IMU, BRIDGE_TYPE_NRF_IMU, BRIDGE_TYPE_ARM)

DOWNLINK_SOF0 = 0xAA
DOWNLINK_SOF1 = 0x55
DOWNLINK_CMD_ARM_TARGET = 0x30
DOWNLINK_LEN_11 = 0x0B

ARM_VOFA_LEN = 28

DEFAULT_STARTUP_DELAY_S = 2.0
DEFAULT_ARM_RX_TIMEOUT_S = 0.5
DEFAULT_HOST_RX_TIMEOUT_S = 0.5
DEFAULT_ARM_ECHO_MUTE_S = 0.020


def _crc8(data: bytes) -> int:
    return sum(data) & 0xFF


def build_framed_arm_target(payload11: bytes) -> bytes:
    assert len(payload11) == DOWNLINK_LEN_11
    body = bytes([DOWNLINK_LEN_11, DOWNLINK_CMD_ARM_TARGET]) + payload11
    return bytes([DOWNLINK_SOF0, DOWNLINK_SOF1]) + body + bytes([_crc8(body)])


def build_vofa_frame(seq: int) -> bytes:
    """28-byte VOFA JustFloat frame: 7 floats, tail is float('+inf')."""
    f0 = 1.0 + (seq % 100) * 0.1
    f1 = 2.0 + (seq % 50) * 0.05
    f2 = 3.0
    f3 = 4.0
    f4 = 5.0
    f5 = 6.0
    f6 = float("inf")  # bytes 00 00 80 7F (little-endian)
    return struct.pack("<fffffff", f0, f1, f2, f3, f4, f5, f6)


def vofa_tail_ok(payload: bytes) -> bool:
    return len(payload) >= 4 and payload[-4:] == bytes([0x00, 0x00, 0x80, 0x7F])


def open_port(port: str, baud: int) -> serial.Serial:
    # Use a short timeout so the synchronous loop is responsive.
    # startup delay is handled once both ports are open.
    s = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
        write_timeout=1.0,
    )
    s.reset_input_buffer()
    s.reset_output_buffer()
    return s


def detect_serial_ports(bridge_baud: int = DEFAULT_BRIDGE_BAUD) -> tuple:
    return stm32_detect_serial_ports(bridge_baud=bridge_baud)


class LoopbackTest:
    def __init__(
        self,
        bridge_port: str,
        bridge_baud: int,
        arm_port: str,
        arm_baud: int,
    ):
        self.host = open_port(bridge_port, bridge_baud)
        self.arm = open_port(arm_port, arm_baud)
        self.bridge_port = bridge_port
        self.bridge_baud = bridge_baud
        self.arm_port = arm_port
        self.arm_baud = arm_baud

        self.start_time = 0.0

        self.host_tx_count = 0
        self.arm_rx_count = 0
        self.arm_tx_count = 0
        self.bridge_arm_count = 0
        self.bridge_other_count = 0
        self.bridge_crc_bad = 0
        self.host_rx_bytes = 0
        self.echo_mute_until = 0.0

    def close(self):
        self.host.close()
        self.arm.close()

    def log(self, msg: str):
        t = time.time() - self.start_time
        print(f"[{t:7.3f}] {msg}", flush=True)

    def drain(self, duration_s: float = 0.05):
        drain_serial_port(self.host, duration_s)
        drain_serial_port(self.arm, duration_s)

    def wait_arm_command(self, expected: bytes, timeout_s: float) -> bool:
        """Read from the arm port until the expected 11-byte command appears."""
        deadline = time.time() + timeout_s
        buf = bytearray()

        while time.time() < deadline:
            data = self.arm.read(64)
            if data:
                if time.time() < self.echo_mute_until:
                    # USB-TTL echo of our own VOFA transmission: discard.
                    continue
                buf.extend(data)

            idx = buf.find(expected)
            if idx >= 0:
                # Keep any trailing bytes for the next cycle.
                buf = buf[idx + len(expected):]
                return True

            if len(buf) > 256:
                buf = buf[-128:]

            # Avoid busy-spinning when no data arrives.
            if not data:
                time.sleep(0.001)

        return False

    def wait_host_bridge(self, timeout_s: float) -> bool:
        """Read from the host port until one valid ARM VOFA bridge frame arrives."""
        deadline = time.time() + timeout_s
        buf = bytearray()

        while time.time() < deadline:
            data = self.host.read(256)
            if data:
                self.host_rx_bytes += len(data)
                buf.extend(data)

            while True:
                if len(buf) < 4:
                    break

                if buf[0] != BRIDGE_SOF0:
                    idx = buf.find(BRIDGE_SOF0)
                    if idx < 0:
                        buf.clear()
                        break
                    buf = buf[idx:]
                    continue

                frame_type = buf[1]
                payload_len = buf[2]
                total = 1 + 1 + 1 + 1 + payload_len + 1  # SOF TYPE LEN SEQ PAYLOAD CRC
                if len(buf) < total:
                    break

                seq = buf[3]
                payload = bytes(buf[4:4 + payload_len])
                crc_recv = buf[4 + payload_len]
                crc_calc = _crc8(buf[1:4 + payload_len])

                if crc_recv != crc_calc:
                    # Corrupted frames with an unknown type are treated as
                    # electrical echo/noise, not as valid bridge frames.
                    if frame_type in BRIDGE_TYPES:
                        self.bridge_crc_bad += 1
                        self.log(f"HOST RX bad CRC type=0x{frame_type:02X} seq={seq}")
                    buf.pop(0)
                    continue

                if frame_type == BRIDGE_TYPE_ARM:
                    self.bridge_arm_count += 1
                else:
                    self.bridge_other_count += 1

                buf = buf[total:]

                if frame_type == BRIDGE_TYPE_ARM and vofa_tail_ok(payload):
                    return True

            if len(buf) > 1024:
                buf = buf[-512:]

            # Avoid busy-spinning when no data arrives.
            if not data:
                time.sleep(0.001)

        return False

    def run(
        self,
        period_s: float = 0.1,
        duration_s: float = 10.0,
        cycles: Optional[int] = None,
    ) -> bool:
        print("=" * 60)
        print("STM32F103 DataHub bridge loop-back test (ping-pong)")
        print(f"  bridge: {self.bridge_port} @ {self.bridge_baud}")
        print(f"  arm:    {self.arm_port} @ {self.arm_baud}")
        print("=" * 60)

        self.start_time = time.time()

        # Give the F103 time to finish any boot/reset caused by DTR assertion
        # when the serial ports are opened.
        if DEFAULT_STARTUP_DELAY_S > 0:
            self.log(f"Waiting {DEFAULT_STARTUP_DELAY_S:.1f}s for F103 boot...")
            time.sleep(DEFAULT_STARTUP_DELAY_S)

        # Drain any stale bytes from the adapters before starting the test.
        self.drain(0.05)

        self.log(
            f"Loop test started: host={self.bridge_port}@{self.bridge_baud}, "
            f"arm={self.arm_port}@{self.arm_baud}"
        )

        max_cycles = cycles if cycles is not None else int(duration_s / max(period_s, 0.001))
        deadline = self.start_time + duration_s
        seq = 0

        try:
            while (cycles is None and time.time() < deadline) or (
                cycles is not None and seq < max_cycles
            ):
                payload = bytes([0xA0 + (seq & 0x0F)] + [i for i in range(10)])
                frame = build_framed_arm_target(payload)

                # 1. Host sends one command.
                try:
                    self.host.write(frame)
                    self.host.flush()
                    self.host_tx_count += 1
                    if seq < 3 or seq % 10 == 0:
                        self.log(f"HOST TX #{seq}: {frame.hex(' ')}")
                except serial.SerialException as e:
                    self.log(f"HOST TX error: {e}")
                    break

                # 2. Wait for the command to arrive at the simulated arm.
                if not self.wait_arm_command(payload, DEFAULT_ARM_RX_TIMEOUT_S):
                    self.log(f"ARM  RX timeout waiting for command #{seq}")
                    time.sleep(period_s)
                    seq += 1
                    continue

                self.arm_rx_count += 1
                if seq < 3 or seq % 10 == 0:
                    self.log(f"ARM  RX cmd #{self.arm_rx_count}: {payload.hex(' ')}")

                # 3. Simulated arm replies with one VOFA frame.
                vofa = build_vofa_frame(self.arm_rx_count)
                try:
                    self.arm.write(vofa)
                    self.echo_mute_until = time.time() + DEFAULT_ARM_ECHO_MUTE_S
                    self.arm_tx_count += 1
                    if seq < 3 or seq % 10 == 0:
                        self.log(
                            f"ARM  TX vofa #{self.arm_tx_count}: len={len(vofa)} "
                            f"tail={vofa[-4:].hex(' ')}"
                        )
                except serial.SerialException as e:
                    self.log(f"ARM TX error: {e}")
                    break

                # 4. Wait for the bridge frame to arrive at the host.
                if not self.wait_host_bridge(DEFAULT_HOST_RX_TIMEOUT_S):
                    self.log(f"HOST RX timeout waiting for bridge frame #{self.arm_tx_count}")
                elif seq < 3 or seq % 10 == 0:
                    self.log(f"HOST RX bridge ARM #{self.bridge_arm_count}")

                # 5. Quiet time before the next command cycle.
                seq += 1
                time.sleep(period_s)
        except KeyboardInterrupt:
            self.log("Interrupted by user")

        return self.print_summary()

    def print_summary(self) -> bool:
        print("\n========== Loop test summary ==========")
        print(f"Duration:            {time.time() - self.start_time:.1f} s")
        print(f"Host TX commands:    {self.host_tx_count}")
        print(f"Arm RX commands:     {self.arm_rx_count}")
        print(f"Arm TX VOFA frames:  {self.arm_tx_count}")
        print(f"Host RX bytes:       {self.host_rx_bytes}")
        print(f"Bridge ARM frames:   {self.bridge_arm_count}")
        print(f"Bridge other frames: {self.bridge_other_count}")
        print(f"Bridge CRC bad:      {self.bridge_crc_bad}")

        clean = (
            self.host_tx_count > 0
            and self.arm_rx_count == self.host_tx_count
            and self.arm_tx_count == self.host_tx_count
            and self.bridge_arm_count == self.host_tx_count
            and self.bridge_crc_bad == 0
        )
        print(f"Result:              {'PASS' if clean else 'FAIL'}")
        print("=======================================\n")
        return clean


def measure_effective_baud(port: str, requested_baud: int) -> Optional[float]:
    """Measure the real Tx throughput of a serial port."""
    try:
        with serial.Serial(port, requested_baud, timeout=0.05, write_timeout=2.0) as s:
            s.reset_output_buffer()
            payload = b"\x55" * 4096
            t0 = time.perf_counter()
            s.write(payload)
            s.flush()
            t1 = time.perf_counter()
            dt = t1 - t0
            if dt <= 0:
                return None
            return len(payload) * 10.0 / dt
    except Exception:
        return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    guessed_bridge, guessed_arm = detect_serial_ports()
    parser.add_argument(
        "--bridge-port",
        default=guessed_bridge or "/dev/ttyUSB0",
        help="Serial port for the F103 USART3 bridge link (default: auto)",
    )
    parser.add_argument(
        "--bridge-baud",
        type=int,
        default=DEFAULT_BRIDGE_BAUD,
        help=f"Baud rate for the bridge port (default: {DEFAULT_BRIDGE_BAUD})",
    )
    parser.add_argument(
        "--arm-port",
        default=guessed_arm or "/dev/ttyUSB1",
        help="Serial port for the F103 USART2 arm/H7 link (default: auto)",
    )
    parser.add_argument(
        "--arm-baud",
        type=int,
        default=DEFAULT_ARM_BAUD,
        help=f"Baud rate for the arm port (default: {DEFAULT_ARM_BAUD})",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=0.1,
        help="Quiet time between ping-pong cycles in seconds (default: 0.1)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Test duration in seconds, used when --cycles is not set (default: 10.0)",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=None,
        help="Fixed number of ping-pong cycles (overrides --duration)",
    )
    args = parser.parse_args()

    if args.arm_port is None:
        print("Error: could not auto-detect a second serial port for the arm link.")
        print("Available ports:")
        for p in sorted(
            glob.glob("/dev/ttyUSB*")
            + glob.glob("/dev/ttyACM*")
            + glob.glob("/dev/ttyS*")
        ):
            print(f"  {p}")
        return 1

    effective = measure_effective_baud(args.bridge_port, args.bridge_baud)
    if effective is not None and effective < args.bridge_baud * 0.8:
        print(
            f"\n[WARNING] Bridge port {args.bridge_port} requested {args.bridge_baud} baud "
            f"but actually achieves ~{effective:.0f} baud.\n"
            "The F103 firmware expects 460800 on USART3.  Either:\n"
            "  1. Rebuild the F103 firmware with APP_DK_UART_BAUDRATE=115200, or\n"
            "  2. Use a USB-UART adapter for the bridge link, or\n"
            "  3. Reconfigure the DK-2500 SIO UART clock in BIOS.\n"
        )

    test = None
    try:
        test = LoopbackTest(
            bridge_port=args.bridge_port,
            bridge_baud=args.bridge_baud,
            arm_port=args.arm_port,
            arm_baud=args.arm_baud,
        )
        ok = test.run(
            period_s=args.period,
            duration_s=args.duration,
            cycles=args.cycles,
        )
        return 0 if ok else 1
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        if test is not None:
            test.close()


if __name__ == "__main__":
    sys.exit(main())
