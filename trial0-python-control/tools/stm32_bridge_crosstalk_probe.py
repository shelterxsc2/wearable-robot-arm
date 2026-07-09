#!/usr/bin/env python3
"""
STM32F103 DataHub 串扰/物理层探测脚本

原理：
  向 bridge 口发送 CRC 故意错误的 AA 55 帧。F103 解析器会在 CRC 状态发现
  校验失败并丢弃该帧，不会向 USART2/arm 口转发任何字节。因此，如果 arm 口
  仍然收到数据，唯一来源就是 bridge TX 信号通过电容/电感/地线耦合到了
  arm RX 线上——即物理串扰。

用法：
  python tools/stm32_bridge_crosstalk_probe.py
  python tools/stm32_bridge_crosstalk_probe.py --bridge /dev/ttyUSB1 --arm /dev/ttyUSB0

输出：
  - 串扰可忽略：PASS
  - 串扰存在：FAIL，并给出建议
"""
import argparse
import os
import struct
import sys
import time

try:
    import serial
except Exception as exc:  # pragma: no cover
    raise RuntimeError("This script requires 'pyserial'") from exc

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from stm32_bridge_utils import (
    detect_serial_ports,
    drain_serial_port,
    open_serial_port,
)


def build_downlink_frame(cmd: int, payload: bytes) -> bytes:
    frame = bytes([0xAA, 0x55, len(payload), cmd]) + payload
    crc = sum(frame[2:]) & 0xFF
    return frame + bytes([crc])


def build_arm_target_payload() -> bytes:
    return struct.pack("<hhhhhB", 0x1234, 0x1299, -1348, 0x0A11, -171, 0x05)


def hexdump(data: bytes, max_len: int = 64) -> str:
    if len(data) > max_len:
        return data[:max_len].hex().upper() + f"...({len(data)} bytes total)"
    return data.hex().upper()


def probe_crosstalk(bridge: serial.Serial, arm: serial.Serial, iterations: int = 8) -> tuple:
    """Return (total_arm_bytes, list_of_sample_bytes)."""
    payload = build_arm_target_payload()
    frame = build_downlink_frame(0x30, payload)
    bad_frame = frame[:-1] + bytes([(frame[-1] + 0xAB) & 0xFF])

    drain_serial_port(arm, 0.3)

    total = 0
    samples = []
    for _ in range(iterations):
        drain_serial_port(arm, 0.05)
        bridge.write(bad_frame)
        bridge.flush()
        time.sleep(0.08)
        rx = arm.read(arm.in_waiting)
        total += len(rx)
        if rx:
            samples.append(rx)
        time.sleep(0.04)

    return total, samples, bad_frame


def main() -> int:
    guessed_bridge, guessed_arm = detect_serial_ports()

    parser = argparse.ArgumentParser(description="Probe bridge-to-arm crosstalk")
    parser.add_argument("--bridge", default=guessed_bridge or "/dev/ttyUSB0")
    parser.add_argument("--bridge-baud", type=int, default=460800)
    parser.add_argument("--arm", default=guessed_arm or "/dev/ttyUSB1")
    parser.add_argument("--arm-baud", type=int, default=115200)
    parser.add_argument("--iterations", type=int, default=8)
    args = parser.parse_args()

    print(f"端口: bridge={args.bridge}@{args.bridge_baud}, arm={args.arm}@{args.arm_baud}")
    bridge = open_serial_port(args.bridge, args.bridge_baud, startup_delay_s=0.0)
    arm = open_serial_port(args.arm, args.arm_baud, startup_delay_s=0.0)
    time.sleep(2.0)
    # CH341 adapters can retain stale bytes across baud-rate changes or previous
    # runs.  Drain thoroughly before the probe so residual FIFO bytes are not
    # mistaken for crosstalk.
    print("两个串口已打开（已等待 F103 启动）")
    print("正在排空 CH341 硬件 FIFO，请稍候...")
    drain_serial_port(bridge, 1.0)
    drain_serial_port(arm, 2.0)
    print()

    total, samples, bad_frame = probe_crosstalk(bridge, arm, args.iterations)

    print(f"发送错误 CRC 帧: {hexdump(bad_frame)}")
    print(f"{args.iterations} 次探测 arm 口共收到 {total} bytes")
    for i, s in enumerate(samples[:5]):
        print(f"  样本 {i}: {len(s)} bytes: {hexdump(s)}")

    print()
    if total == 0:
        print("[PASS] arm 口未收到任何字节，bridge -> arm 串扰可忽略")
        print("       F103 downlink 解析器工作正常（至少 CRC 拒绝路径正常）")
        return 0

    print("[FAIL] 检测到物理串扰：bridge TX 信号出现在 arm RX 线上")
    print("       F103 已丢弃错误 CRC 帧，但 arm 口仍收到 bridge 发送的字节")
    print()
    print("建议排查：")
    print("  1. 缩短 bridge TX / arm RX 杜邦线，避免长导线天线效应")
    print("  2. 将 bridge 与 arm 的 TX/RX 线分开，不要并行走线")
    print("  3. 确保两个 USB-TTL 模块共地（GND 短接）")
    print("  4. 在信号线两端靠近 MCU/适配器处加 100~330 Ω 串联电阻")
    print("  5. 使用屏蔽双绞线或降低 bridge 波特率（需同步修改 F103 固件）")
    print("  6. 若 crosstalk 来自 USB 侧共模噪声，把两个适配器插到不同 USB 控制器/带屏蔽 Hub")
    return 1


if __name__ == "__main__":
    sys.exit(main())
