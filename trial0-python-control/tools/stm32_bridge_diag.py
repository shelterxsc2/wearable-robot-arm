#!/usr/bin/env python3
"""
STM32F103 DataHub USART2/3 链路诊断脚本
不依赖协议解析，直接双向灌包并统计收发，定位硬件/物理层问题。
"""
import argparse
import os
import struct
import sys
import threading
import time

try:
    import serial
except Exception as exc:  # pragma: no cover
    raise RuntimeError("This script requires 'pyserial'") from exc

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from stm32_bridge_utils import (
    open_serial_port,
    detect_serial_ports,
    drain_serial_port,
)


def build_downlink_frame(cmd: int, payload: bytes) -> bytes:
    frame = bytes([0xAA, 0x55, len(payload), cmd]) + payload
    crc = sum(frame[2:]) & 0xFF
    return frame + bytes([crc])


def build_arm_target_payload() -> bytes:
    return struct.pack("<hhhhhB", 0x1234, 0x1299, -1348, 0x0A11, -171, 0x05)


def build_vofa_frame(seq: int = 0) -> bytes:
    """H7 VOFA float[7] 帧，末尾必须是 00 00 80 7F 才能被 F103 桥接识别。"""
    floats = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    payload = b"".join(struct.pack("<f", f) for f in floats)
    payload += struct.pack("<f", float("+inf"))  # 0x7F800000 -> 00 00 80 7F LE
    assert len(payload) == 28
    crc = sum([0x53, 0x1C, seq] + list(payload)) & 0xFF
    return bytes([0xA5, 0x53, 0x1C, seq]) + payload + bytes([crc])


def hexdump(data: bytes, max_len: int = 128) -> str:
    if len(data) > max_len:
        return data[:max_len].hex().upper() + f"...({len(data)} bytes total)"
    return data.hex().upper()


def open_port(port: str, baud: int) -> serial.Serial:
    try:
        return open_serial_port(port, baud, startup_delay_s=0.0)
    except Exception as e:
        print(f"[ERROR] 无法打开 {port} @ {baud}: {e}")
        sys.exit(1)


def test_arm_loopback(arm: serial.Serial, arm_port: str = "arm") -> None:
    """把 USB-TTL 的 TX-RX 短接后才能通过。"""
    print(f"\n[TEST] {arm_port} 自环测试（需要把该 USB-TTL 的 TX 与 RX 短接）")
    test_data = b"LOOPBACK_TEST_115200\r\n"
    arm.reset_input_buffer()
    arm.write(test_data)
    arm.flush()
    time.sleep(0.1)
    rx = arm.read(arm.in_waiting)
    if rx == test_data:
        print("  PASS: 自环收到完整回显")
    else:
        print(f"  FAIL: 发送 {len(test_data)} bytes，收到 {len(rx)} bytes: {hexdump(rx)}")


def test_bridge_tx_to_arm(bridge: serial.Serial, arm: serial.Serial) -> None:
    print("\n[TEST] bridge -> F103 -> arm: 发送 target 命令，监听 arm 口")
    payload = build_arm_target_payload()
    frame = build_downlink_frame(0x30, payload)
    drain_serial_port(arm, 0.2)
    bridge.write(frame)
    bridge.flush()
    time.sleep(0.3)
    rx = arm.read(arm.in_waiting)
    print(f"  发送: {hexdump(frame)}")
    print(f"  arm 口收到 {len(rx)} bytes: {hexdump(rx)}")
    if payload in rx:
        print("  PASS: arm 口收到完整 11 字节 payload")
    elif len(rx) >= 11:
        print("  WARN: arm 口收到数据但 payload 未对齐，可能存在串扰/接触不良")
    else:
        print("  FAIL: arm 口未收到足够数据")


def test_crosstalk(bridge: serial.Serial, arm: serial.Serial) -> None:
    """Send a deliberately bad-CRC frame; any arm RX bytes must be crosstalk."""
    print("\n[TEST] 串扰探测：发送 CRC 错误的帧，F103 不应转发任何字节")
    payload = build_arm_target_payload()
    frame = build_downlink_frame(0x30, payload)
    bad_frame = frame[:-1] + bytes([(frame[-1] + 0xAB) & 0xFF])

    drain_serial_port(arm, 0.2)

    total_rx = 0
    samples = []
    for _ in range(5):
        drain_serial_port(arm, 0.05)
        bridge.write(bad_frame)
        bridge.flush()
        time.sleep(0.08)
        rx = arm.read(arm.in_waiting)
        total_rx += len(rx)
        if rx:
            samples.append(rx)
        time.sleep(0.05)

    print(f"  发送: {hexdump(bad_frame)}")
    print(f"  5 次探测 arm 口共收到 {total_rx} bytes")
    for i, s in enumerate(samples[:3]):
        print(f"    样本 {i}: {len(s)} bytes: {hexdump(s, max_len=64)}")

    if total_rx == 0:
        print("  PASS: arm 口未收到任何字节，串扰可忽略")
    else:
        print("  FAIL/串扰: arm 口收到 bridge TX 信号，说明存在物理串扰")
        print("  建议：缩短/分开杜邦线、共地、降低 bridge 波特率或加屏蔽")


def test_arm_rx_to_bridge(arm: serial.Serial, bridge: serial.Serial) -> None:
    print("\n[TEST] arm -> F103 -> bridge: 发送 VOFA 尾帧，监听 bridge 口")
    vofa = b"".join(struct.pack("<f", f) for f in [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    vofa += struct.pack("<f", float("+inf"))  # tail for raw bridge detection
    bridge.reset_input_buffer()
    arm.reset_input_buffer()
    arm.write(vofa)
    arm.flush()
    time.sleep(0.3)
    rx = bridge.read(bridge.in_waiting)
    print(f"  发送 {len(vofa)} bytes VOFA 尾帧到 arm 口")
    print(f"  bridge 口收到 {len(rx)} bytes: {hexdump(rx)}")
    if bytes([0xA5, 0x53]) in rx:
        print("  PASS: 发现 A5 53 桥接帧")
    else:
        print("  FAIL: 未发现 A5 53 桥接帧")


def test_bridge_frame_to_bridge(arm: serial.Serial, bridge: serial.Serial) -> None:
    """把完整的 A5 53 桥接帧直接发到 arm 口，F103 会原样桥接到 bridge。"""
    print("\n[TEST] arm -> F103 -> bridge: 发送完整 A5 53 桥接帧，监听 bridge 口")
    frame = build_vofa_frame(seq=42)
    bridge.reset_input_buffer()
    arm.reset_input_buffer()
    arm.write(frame)
    arm.flush()
    time.sleep(0.3)
    rx = bridge.read(bridge.in_waiting)
    print(f"  发送: {hexdump(frame)}")
    print(f"  bridge 口收到 {len(rx)} bytes: {hexdump(rx)}")
    if frame[1:3] in rx:
        print("  发现桥接帧类型/长度字节，可能有响应")


def drain_bridge_loop(bridge: serial.Serial, stop_event: threading.Event):
    """Continuously drain the bridge RX buffer so the F103 TX never blocks."""
    while not stop_event.is_set():
        try:
            bridge.read(bridge.in_waiting or 1)
        except Exception:
            pass


def test_noise_floor(bridge: serial.Serial, arm: serial.Serial) -> None:
    print("\n[TEST] 噪声底：bridge 静默 0.5s，统计 arm 口误收字节")
    drain_serial_port(bridge, 0.05)
    drain_serial_port(arm, 0.2)
    time.sleep(0.5)
    rx = arm.read(arm.in_waiting)
    print(f"  arm 口静默期间收到 {len(rx)} bytes: {hexdump(rx)}")


def main():
    guessed_bridge, guessed_arm = detect_serial_ports()

    parser = argparse.ArgumentParser()
    parser.add_argument("--bridge", default=guessed_bridge or "/dev/ttyUSB0")
    parser.add_argument("--bridge-baud", type=int, default=460800)
    parser.add_argument("--arm", default=guessed_arm or "/dev/ttyUSB1")
    parser.add_argument("--arm-baud", type=int, default=115200)
    parser.add_argument("--skip-loopback", action="store_true", help="跳过需要短接的自环测试")
    args = parser.parse_args()

    print(f"端口: bridge={args.bridge}@{args.bridge_baud}, arm={args.arm}@{args.arm_baud}")
    bridge = open_port(args.bridge, args.bridge_baud)
    arm = open_port(args.arm, args.arm_baud)
    # Give the F103 time to recover from the DTR reset caused by opening ports.
    time.sleep(2.0)
    # CH341 adapters can retain stale bytes across baud-rate changes or previous
    # runs.  Drain thoroughly before the real tests so residual FIFO bytes are not
    # mistaken for crosstalk.
    print("两个串口已打开（已等待 F103 启动）")
    print("正在排空 CH341 硬件 FIFO，请稍候...")
    drain_serial_port(bridge, 1.0)
    drain_serial_port(arm, 2.0)
    print()

    if not args.skip_loopback:
        test_arm_loopback(arm, args.arm)

    # These tests only look at the arm RX line.  Drain it thoroughly first because
    # CH341 adapters can leak residual bytes from earlier traffic.
    test_noise_floor(bridge, arm)
    test_crosstalk(bridge, arm)

    # Keep draining the bridge RX buffer in the background so F103 sensor uplink
    # does not block while we run the downlink checks.
    stop_drain = threading.Event()
    drain_thread = threading.Thread(target=drain_bridge_loop, args=(bridge, stop_drain), daemon=True)
    drain_thread.start()

    try:
        test_bridge_tx_to_arm(bridge, arm)
    finally:
        stop_drain.set()
        drain_thread.join(timeout=0.5)

    # For uplink tests we must capture the bridge response, so no background drain.
    test_arm_rx_to_bridge(arm, bridge)
    test_bridge_frame_to_bridge(arm, bridge)

    print("\n[提示]")
    print("  - 若自环 FAIL：/dev/ttyUSB1 适配器或驱动有问题，先换适配器")
    print("  - 若串扰探测 FAIL：bridge TX 信号耦合到 arm RX，需改善布线/接地/屏蔽")
    print("  - 若串扰 PASS 但下行 FAIL：F103 downlink 解析器或固件版本问题")
    print("  - 建议重刷最新编译的 hex 后重测")


if __name__ == "__main__":
    main()
