# -*- coding: utf-8 -*-
"""STM32F103 DataHub bridge backend for the ELF control chain.

The real STM32 firmware lives in:

    /home/time/work/stm32f103_datahub/

It is **not** maintained inside this repository.  This Python module speaks the
classified bridge protocol that the STM32F103 DataHub emits on USART3, and sends
host commands back to the bridge using the framed downlink protocol.

STM32F103 DataHub inputs
------------------------
- USART1 @ 230400 : waist JY61P IMU raw WIT frames
- nRF24L01+       : remote head IMU payload (22 bytes)
- USART2 @ 115200 : STM32H7 robotic-arm stream (VOFA JustFloat float[7])

STM32F103 DataHub uplink output (USART3 @ 460800)
-------------------------------------------------
Classified bridge frames:

    A5 TYPE LEN SEQ PAYLOAD... CRC

- `A5`   : frame header
- `TYPE` : source class
    - `0x51` waist JY61P IMU frame (11 bytes, WIT protocol)
    - `0x52` nRF24 remote IMU payload (22 bytes)
    - `0x53` STM32H7 arm stream (28 bytes, VOFA JustFloat float[7])
- `LEN`  : payload byte count
- `SEQ`  : 8-bit sequence number, wraps at 255
- `PAYLOAD`: original upstream bytes, unchanged
- `CRC`  : unsigned 8-bit sum over `TYPE + LEN + SEQ + PAYLOAD`

The bridge does not decode quaternion, gyro, WIT, or VOFA contents; it only
groups records and marks their source.  Python does the decoding here.

Host downlink input to STM32F103 DataHub (USART3 @ 460800)
----------------------------------------------------------
Framed command protocol (preferred for coordinate targets):

    AA 55 LEN CMD PAYLOAD... CRC

- `AA 55`: downlink frame header
- `LEN`  : payload byte count
- `CMD`  : command class
    - `0x01` heartbeat (LEN = 0), consumed by the bridge
    - `0x30` arm target (LEN = 11: int16 x, y, z, k1, k2 + 1 byte flag)
    - `0x10` target pose (LEN = 28), consumed by the bridge by default
- `CRC`  : unsigned 8-bit sum over `LEN + CMD + PAYLOAD`

Legacy raw arm commands (10 bytes) are also accepted by the bridge:

    x(2) y(2) z(2) k1(2) k2(2)

The bridge appends one `0x00` pad byte and forwards 11 bytes to the H7.
Special 10-byte H7 commands are forwarded unchanged:

    FF AA FF AA FF AA FF AA FF AA  -> H7 init command
    AA FF AA FF AA FF AA FF AA FF  -> H7 retract/exit command

For robustness the host should use the framed `CMD 0x30` format for normal
coordinate targets; otherwise a target whose first two bytes happen to be
`AA 55` would be mistaken for a framed command.
"""
from __future__ import annotations

import os
import struct
import threading
import time
import traceback
from typing import Any, Dict, List, Optional, Tuple

from elf_control_chain import (
    Imu2Source,
    Nrf24ImuSource,
    UartArmSink,
    eulerZYXToMat,
    parse_nrf24_imu_payload,
    rotation_matrix_to_quaternion,
)
from stm32_bridge_utils import (
    detect_bridge_port,
    DEFAULT_STARTUP_DELAY_S,
)

# ---------------------------------------------------------------------------
# Uplink bridge frame constants
# ---------------------------------------------------------------------------

BRIDGE_SOF = 0xA5

TYPE_WAIST_IMU = 0x51
TYPE_NRF_IMU = 0x52
TYPE_ARM_STREAM = 0x53

WAIST_FRAME_LEN = 11
NRF_PAYLOAD_LEN = 22
ARM_VOFA_FRAME_LEN = 28
ARM_VOFA_SHORT_FRAME_LEN = 16
ARM_STREAM_MAX_LEN = 32

# ---------------------------------------------------------------------------
# Downlink command frame constants
# ---------------------------------------------------------------------------

DOWNLINK_SOF = b"\xAA\x55"

DOWNLINK_CMD_HEARTBEAT = 0x01
DOWNLINK_CMD_TARGET_POSE = 0x10
DOWNLINK_CMD_ARM_TARGET = 0x30

DOWNLINK_ARM_TARGET_LEN = 11  # 5*int16 + 1 byte flag/pad

# Special H7 commands forwarded unchanged by the F103 bridge.
H7_INIT_FRAME = bytes([0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA])
H7_RETRACT_FRAME = bytes([0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF, 0xAA, 0xFF])


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _sum8(data: bytes) -> int:
    """Unsigned 8-bit sum (matches STM32 sum8)."""
    return sum(data) & 0xFF


def _parse_wit_frame(frame: bytes) -> Optional[Tuple[str, Dict[str, float]]]:
    """Parse an 11-byte JY61P/WIT frame.

    Returns ("angle" | "gyro" | "accel", data) or None if invalid.
    """
    if len(frame) != 11 or frame[0] != 0x55:
        return None
    if (sum(frame[:10]) & 0xFF) != frame[10]:
        return None

    def f(i: int, scale: float) -> float:
        return float(struct.unpack_from("<h", frame, i)[0]) * scale

    frame_type = frame[1]
    if frame_type == 0x51:
        return "accel", {
            "ax": f(2, 16.0 / 32768.0),
            "ay": f(4, 16.0 / 32768.0),
            "az": f(6, 16.0 / 32768.0),
        }
    if frame_type == 0x52:
        return "gyro", {
            "wx": f(2, 2000.0 / 32768.0),
            "wy": f(4, 2000.0 / 32768.0),
            "wz": f(6, 2000.0 / 32768.0),
        }
    if frame_type == 0x53:
        return "angle", {
            "roll": f(2, 180.0 / 32768.0),
            "pitch": f(4, 180.0 / 32768.0),
            "yaw": f(6, 180.0 / 32768.0),
        }
    return None


def _euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, ...]:
    """Convert ZYX Euler angles to a normalized quaternion."""
    R = eulerZYXToMat(roll_deg, pitch_deg, yaw_deg)
    return rotation_matrix_to_quaternion(R)


def build_downlink_frame(cmd: int, payload: bytes) -> bytes:
    """Build a host-to-bridge downlink frame: AA 55 LEN CMD PAYLOAD CRC."""
    body = bytes([len(payload) & 0xFF, cmd & 0xFF]) + payload
    crc = _sum8(body)
    return DOWNLINK_SOF + body + bytes([crc])


def build_arm_target_frame(x: float, y: float, z: float,
                           k1: float, k2: float, flag: int = 0) -> bytes:
    """Build a framed arm target command (CMD 0x30, LEN 11).

    The 11-byte payload is:
        int16 x, int16 y, int16 z, int16 k1, int16 k2, uint8 flag
    The F103 bridge forwards these 11 bytes to the H7 unchanged.
    """
    payload = struct.pack(
        "<hhhhhB",
        int(x), int(y), int(z), int(k1), int(k2),
        int(flag) & 0xFF,
    )
    return build_downlink_frame(DOWNLINK_CMD_ARM_TARGET, payload)


# ---------------------------------------------------------------------------
# Bridge class
# ---------------------------------------------------------------------------

class Stm32DataHubBridge:
    """Parse the STM32F103 DataHub classified bridge protocol on a serial port."""

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 460800,
        timeout: float = 0.05,
        startup_delay_s: float = DEFAULT_STARTUP_DELAY_S,
        verbose: bool = False,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.startup_delay_s = startup_delay_s
        self.verbose = verbose
        self._serial: Optional[Any] = None
        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        self._head_imu: Dict[str, Any] = {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "qw": 1.0, "qx": 0.0, "qy": 0.0, "qz": 0.0,
            "wx": 0.0, "wy": 0.0, "wz": 0.0,
            "imu_valid": False,
            "quat_valid": False,
            "sample_seq": 0,
            "updated_monotonic": 0.0,
        }
        self._waist_imu: Dict[str, Any] = {
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "wx": 0.0, "wy": 0.0, "wz": 0.0,
            "valid": False,
        }
        self._arm_debug_vofa: Dict[str, Any] = {
            "s": 0.0,              # planned displacement
            "v": 0.0,              # planned velocity
            "a": 0.0,              # planned acceleration
            "motor0_target": 0.0,  # LK4005_Motor_Handle[0].Motor_Position_Target
            "motor0_actual": 0.0,  # LK4005_Motor_Handle[0].Motor_Position_Actual
            "error_s": 0.0,        # error_s
            "tail": float("inf"),  # VOFA INFINITY tail marker
            "valid": False,
        }
        self._arm_stream: Dict[str, Any] = {
            "format": None,
            "payload": b"",
            "floats": (),
            "text": None,
            "move_complete": False,
            "valid": False,
        }
        self._arm_event_callback: Optional[Any] = None
        self._bridge_meta: Dict[str, Any] = {
            "seq": {TYPE_WAIST_IMU: 0, TYPE_NRF_IMU: 0, TYPE_ARM_STREAM: 0},
            "drop_count": 0,
        }

    @staticmethod
    def build_bridge_frame(frame_type: int, seq: int, payload: bytes) -> bytes:
        """Build a classified bridge frame (used by tests / simulators)."""
        body = bytes([frame_type, len(payload) & 0xFF, seq & 0xFF]) + payload
        crc = _sum8(body)
        return bytes([BRIDGE_SOF]) + body + bytes([crc])

    @staticmethod
    def build_target_frame(x: float, y: float, z: float,
                           k1: float, k2: float, flag: int = 0) -> bytes:
        """Build a downlink arm target frame (AA 55 0B 30 X Y Z K1 K2 FLAG CRC).

        Deprecated alias for build_arm_target_frame().  Kept for callers that
        expect a `build_target_frame` method on the bridge class.
        """
        return build_arm_target_frame(x, y, z, k1, k2, flag)

    def start(self):
        try:
            import serial
        except ImportError as e:
            raise RuntimeError("Stm32DataHubBridge requires 'pyserial'") from e

        port = self.port
        if not port:
            detected = detect_bridge_port(self.baudrate)
            if detected is None:
                raise RuntimeError(
                    "Could not auto-detect STM32 DataHub bridge port"
                )
            port = detected
            self.port = port

        self._serial = serial.Serial(
            port,
            self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=self.timeout,
            write_timeout=1.0,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

        # Wait for the F103 to recover from the DTR reset that pyserial asserts
        # when opening a CH341 adapter.
        if self.startup_delay_s > 0:
            time.sleep(self.startup_delay_s)

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        print(
            f"[STM32-DataHub] Bridge started on {port} @ {self.baudrate} baud"
        )

    def stop(self):
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self._serial and self._serial.is_open:
            self._serial.close()

    def _rx_loop(self):
        buf = b""
        while self._running:
            try:
                chunk = self._serial.read(256)
            except Exception:
                traceback.print_exc()
                time.sleep(0.1)
                continue
            if chunk:
                buf += chunk
                buf = self._parse_buffer(buf)
            else:
                time.sleep(0.001)

    def _parse_buffer(self, buf: bytes) -> bytes:
        while True:
            sof_idx = buf.find(bytes([BRIDGE_SOF]))
            if sof_idx < 0:
                # Keep the last byte in case it is the SOF byte.
                return buf[-1:] if buf else b""

            buf = buf[sof_idx:]
            if len(buf) < 4:
                return buf

            frame_type = buf[1]
            payload_len = buf[2]
            seq = buf[3]

            valid_len = (
                (frame_type == TYPE_WAIST_IMU and payload_len == WAIST_FRAME_LEN)
                or (frame_type == TYPE_NRF_IMU and payload_len == NRF_PAYLOAD_LEN)
                or (frame_type == TYPE_ARM_STREAM and 0 < payload_len <= ARM_STREAM_MAX_LEN)
            )
            if not valid_len:
                if self.verbose:
                    print(
                        f"[STM32-DataHub] Invalid frame length "
                        f"type=0x{frame_type:02X} len={payload_len}"
                    )
                buf = buf[1:]
                continue

            total = 1 + 3 + payload_len + 1  # SOF + TYPE/LEN/SEQ + payload + CRC
            if len(buf) < total:
                return buf

            frame = buf[:total]
            buf = buf[total:]

            # Validate type and CRC.
            if frame_type not in (TYPE_WAIST_IMU, TYPE_NRF_IMU, TYPE_ARM_STREAM):
                print(f"[STM32-DataHub] Unknown frame type=0x{frame_type:02X}")
                continue

            calc_crc = _sum8(frame[1:1 + 3 + payload_len])
            rx_crc = frame[1 + 3 + payload_len]
            if rx_crc != calc_crc:
                print(
                    f"[STM32-DataHub] CRC mismatch type=0x{frame_type:02X} "
                    f"calc=0x{calc_crc:02X} rx=0x{rx_crc:02X}"
                )
                continue

            payload = frame[4:4 + payload_len]
            self._handle_frame(frame_type, seq, payload)

    def _handle_frame(self, frame_type: int, seq: int, payload: bytes):
        if self.verbose and frame_type == TYPE_ARM_STREAM:
            preview = payload[:28].hex()
            if len(payload) > 28:
                preview += "..."
            print(
                f"[STM32-DataHub] RX ARM_STREAM seq={seq} len={len(payload)} "
                f"payload={preview}"
            )
        try:
            with self._lock:
                self._bridge_meta["seq"][frame_type] = seq

            if frame_type == TYPE_WAIST_IMU:
                self._handle_waist_frame(payload)
            elif frame_type == TYPE_NRF_IMU:
                self._handle_nrf_frame(payload)
            elif frame_type == TYPE_ARM_STREAM:
                self._handle_arm_frame(payload)
        except Exception as e:
            print(f"[STM32-DataHub] Frame handler error: {e}")
            traceback.print_exc()

    def _handle_waist_frame(self, payload: bytes):
        parsed = _parse_wit_frame(payload)
        if parsed is None:
            return
        kind, data = parsed
        with self._lock:
            if kind == "angle":
                self._waist_imu.update(
                    roll=data["roll"],
                    pitch=data["pitch"],
                    yaw=data["yaw"],
                    valid=True,
                )
            elif kind == "gyro":
                self._waist_imu.update(
                    wx=data["wx"],
                    wy=data["wy"],
                    wz=data["wz"],
                )

    def _handle_nrf_frame(self, payload: bytes):
        if len(payload) != NRF_PAYLOAD_LEN:
            return
        data = parse_nrf24_imu_payload(payload)
        if not data.get("imu_valid"):
            return
        roll = data.get("roll", 0.0)
        pitch = data.get("pitch", 0.0)
        yaw = data.get("yaw", 0.0)
        qw, qx, qy, qz = _euler_to_quat(roll, pitch, yaw)
        with self._lock:
            sample_seq = int(self._head_imu.get("sample_seq", 0)) + 1
            self._head_imu.update(
                roll=roll,
                pitch=pitch,
                yaw=yaw,
                qw=qw, qx=qx, qy=qy, qz=qz,
                wx=data.get("wx", 0.0),
                wy=data.get("wy", 0.0),
                wz=data.get("wz", 0.0),
                imu_valid=True,
                quat_valid=True,
                sample_seq=sample_seq,
                updated_monotonic=time.monotonic(),
            )

    def _handle_arm_frame(self, payload: bytes):
        stream_format = "binary"
        floats: Tuple[float, ...] = ()
        text = None
        move_complete = False
        init_success = False

        if payload.endswith(b"\r\n"):
            text = payload.decode("utf-8", errors="replace").rstrip("\r\n")
            stream_format = "ascii"
            text_l = text.strip().lower()
            move_complete = text_l == "move complete"
            init_success = text_l == "init success"
        elif len(payload) in (ARM_VOFA_SHORT_FRAME_LEN, ARM_VOFA_FRAME_LEN):
            floats = struct.unpack(f"<{len(payload) // 4}f", payload)
            stream_format = f"vofa{len(floats)}"

        with self._lock:
            self._arm_stream.update(
                format=stream_format,
                payload=bytes(payload),
                floats=floats,
                text=text,
                move_complete=move_complete,
                valid=True,
            )
            if len(floats) == 7:
                self._arm_debug_vofa.update(
                    s=floats[0],
                    v=floats[1],
                    a=floats[2],
                    motor0_target=floats[3],
                    motor0_actual=floats[4],
                    error_s=floats[5],
                    tail=floats[6],
                    valid=True,
                )
            callback = self._arm_event_callback

        if callback is not None:
            if init_success:
                callback("init_success", text)
            if move_complete:
                callback("move_complete", text)

    def send_raw(self, data: bytes) -> bool:
        """Send raw bytes to the bridge UART.

        This is intended for legacy 10-byte raw arm commands (including the
        special H7 init/retract patterns) that the bridge recognises without
        an AA 55 header.
        """
        if self._serial is None or not self._serial.is_open:
            return False
        try:
            t0 = time.perf_counter()
            with self._lock:
                t_lock = time.perf_counter()
                self._serial.write(data)
                t_write = time.perf_counter()
                self._serial.flush()
                t_flush = time.perf_counter()
            lock_ms = (t_lock - t0) * 1000.0
            write_ms = (t_write - t_lock) * 1000.0
            flush_ms = (t_flush - t_write) * 1000.0
            total_ms = (t_flush - t0) * 1000.0
            if total_ms >= 100.0:
                print(
                    f"[UART-SLOW] total={total_ms:.1f}ms lock={lock_ms:.1f}ms "
                    f"write={write_ms:.1f}ms flush={flush_ms:.1f}ms bytes={len(data)}"
                )
            if self.verbose:
                print(f"[STM32-DataHub] TX raw {data.hex()}")
            return True
        except Exception as e:
            print(f"[STM32-DataHub] send_raw failed: {e}")
            return False

    def send_downlink_frame(self, cmd: int, payload: bytes) -> bool:
        """Send a framed downlink command (AA 55 LEN CMD PAYLOAD CRC)."""
        return self.send_raw(build_downlink_frame(cmd, payload))

    def send_heartbeat(self) -> bool:
        """Send a heartbeat command consumed by the F103 bridge."""
        return self.send_downlink_frame(DOWNLINK_CMD_HEARTBEAT, b"")

    def send_h7_init(self) -> bool:
        """Send the special H7 init command through the bridge."""
        return self.send_raw(H7_INIT_FRAME)

    def send_h7_retract(self) -> bool:
        """Send the special H7 retract/exit command through the bridge."""
        return self.send_raw(H7_RETRACT_FRAME)

    def send_arm_target(self, x: float, y: float, z: float,
                        k1: float, k2: float, flag: int) -> bool:
        """Send a framed arm target command on the bridge UART.

        Frame format: AA 55 0B 30 X Y Z K1 K2 FLAG CRC
        The F103 bridge forwards the 11-byte payload to the H7 unchanged.
        """
        frame = build_arm_target_frame(x, y, z, k1, k2, flag)
        return self.send_raw(frame)

    def get_head_imu(self) -> Dict[str, Any]:
        with self._lock:
            data = dict(self._head_imu)
        age_s = time.monotonic() - float(data.get("updated_monotonic", 0.0))
        if age_s > 0.3:
            data["imu_valid"] = False
        data["age_s"] = age_s
        return data

    def get_waist_imu(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._waist_imu)

    def get_arm_debug_vofa(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._arm_debug_vofa)

    def get_arm_stream(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._arm_stream)

    def set_arm_event_callback(self, callback: Optional[Any]):
        with self._lock:
            self._arm_event_callback = callback

    def get_meta(self) -> Dict[str, Any]:
        with self._lock:
            return dict(self._bridge_meta)


# ---------------------------------------------------------------------------
# Source / sink adapters
# ---------------------------------------------------------------------------

class BridgeNrf24ImuSource(Nrf24ImuSource):
    """Head-worn IMU source that reads from the STM32 DataHub bridge."""

    def __init__(self, bridge: Stm32DataHubBridge):
        super().__init__()
        self._bridge = bridge
        self._last_sample_seq = -1

    def start(self):
        self._bridge.start()

    def stop(self):
        self._bridge.stop()

    def poll(self) -> Optional[Dict[str, Any]]:
        data = self._bridge.get_head_imu()
        sample_seq = int(data.get("sample_seq", 0))
        if data.get("imu_valid") and sample_seq != self._last_sample_seq:
            self._on_valid_frame(data)
            self._last_sample_seq = sample_seq
        return data


class BridgeImu2Source(Imu2Source):
    """Waist IMU2 source that reads from the STM32 DataHub bridge."""

    def __init__(self, bridge: Stm32DataHubBridge):
        self._bridge = bridge

    def poll(self) -> Dict[str, Any]:
        return self._bridge.get_waist_imu()


class BridgeUartArmSink(UartArmSink):
    """UART arm sink that sends target frames through the STM32 DataHub bridge.

    DK-2500 sends framed downlink commands on USART3 to the F103 bridge; the
    bridge routes them to the H7 arm controller on USART2.  Serial parameters:
    460800 8N1.

    Normal coordinate targets use the robust framed format:
        AA 55 0B 30 X(2) Y(2) Z(2) K1(2) K2(2) FLAG(1) CRC

    The special H7 init/retract commands are sent as raw 10-byte patterns and
    are forwarded unchanged by the bridge.

    If DK-2500 is connected directly to the H7 controller, use
    elf_control_chain.SerialUartArmSink instead.
    """

    def __init__(self, bridge: Stm32DataHubBridge):
        super().__init__()
        self._bridge = bridge
        self._bridge.set_arm_event_callback(self._handle_arm_event)

    def _handle_arm_event(self, event: str, _text: Optional[str]):
        if event == "init_success":
            self.init_success = True
        elif event == "move_complete":
            self.move_complete = True

    def start(self):
        print("[STM32-DataHub] Arm sink waiting for H7 init success...")
        deadline = time.perf_counter() + 10.0
        while not self.init_success and time.perf_counter() < deadline:
            time.sleep(0.05)
        if self.init_success:
            print("[STM32-DataHub] H7 init success received")
        else:
            print("[STM32-DataHub] WARNING: H7 init success not received")

    def stop(self):
        if self.init_success:
            self.send_power_off()
            time.sleep(0.05)

    def send_power_on(self) -> bool:
        if self.arm_powered:
            return False
        if not self.init_success:
            return False
        ok = self._bridge.send_h7_init()
        if ok:
            self.arm_powered = True
            self.init_success = True
        return ok

    def send_power_off(self) -> bool:
        if not self.arm_powered:
            return False
        ok = self._bridge.send_h7_retract()
        if ok:
            self.arm_powered = False
            self.block_tx = True
        return ok

    def send_arm_target(self, x: float, y: float, z: float,
                        k1: float, k2: float, flag: int) -> bool:
        if self.block_tx:
            return False
        if not self.arm_powered:
            return False
        ok = self._bridge.send_arm_target(x, y, z, k1, k2, flag)
        if ok:
            self.move_complete = False
            print(
                f"[UART-TX] x={x:+.1f} y={y:+.1f} z={z:+.1f} "
                f"k1={k1:+.1f} k2={k2:+.1f} flag=0x{flag:02X}"
            )
        return ok


def make_stm32_bridge_thread(
    port: Optional[str] = None,
    baudrate: int = 460800,
    ctrl_port: int = 8080,
) -> "ElfControlThread":
    """Convenience constructor for a fully bridged STM32 DataHub control thread."""
    from elf_control_chain import ElfControlThread

    bridge = Stm32DataHubBridge(port=port, baudrate=baudrate)
    nrf = BridgeNrf24ImuSource(bridge)
    imu2 = BridgeImu2Source(bridge)
    uart = BridgeUartArmSink(bridge)
    return ElfControlThread(nrf, imu2, uart, ctrl_port=ctrl_port)


def _demo():
    bridge = Stm32DataHubBridge()
    bridge.start()
    try:
        print("Waiting for STM32 DataHub bridge frames, press Ctrl-C to stop...")
        while True:
            print("HEAD:", bridge.get_head_imu())
            print("WAIST:", bridge.get_waist_imu())
            print("ARM:", bridge.get_arm_debug_vofa())
            print("META:", bridge.get_meta())
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    _demo()
