#!/usr/bin/env python3
"""Standalone HC-08 BLE connection and notification diagnostic."""
from __future__ import annotations

import argparse
import asyncio
import signal

from bleak import BleakClient, BleakScanner

DEFAULT_MAC = "F8:2E:0C:E3:99:C8"
DEFAULT_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"


class FrameParser:
    def __init__(self) -> None:
        self._frame = bytearray()

    def feed(self, data: bytearray) -> None:
        print(f"[BLE-Test] RX raw: {bytes(data).hex(' ')}")
        for byte in data:
            if not self._frame:
                if byte == 0x55:
                    self._frame.append(byte)
                continue
            self._frame.append(byte)
            if len(self._frame) == 3:
                _, cmd, value = self._frame
                print(f"[BLE-Test] Frame: cmd=0x{cmd:02X} value=0x{value:02X}")
                self._frame.clear()


async def scan_devices(timeout: float) -> None:
    print(f"[BLE-Test] Scanning for {timeout:.1f}s...")
    devices = await BleakScanner.discover(timeout=timeout, return_adv=True)
    if not devices:
        print("[BLE-Test] No BLE devices found")
        return
    for address, (device, advertisement) in devices.items():
        name = device.name or advertisement.local_name or "<unknown>"
        print(f"[BLE-Test] Found {address} name={name!r} rssi={advertisement.rssi}")


async def run(args: argparse.Namespace) -> None:
    if args.scan:
        await scan_devices(args.scan_timeout)

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    frame_parser = FrameParser()
    while not stop.is_set():
        try:
            print(f"[BLE-Test] Connecting to {args.mac}...")
            async with BleakClient(args.mac, timeout=args.connect_timeout) as client:
                print(f"[BLE-Test] Connected: {client.is_connected}")
                print(f"[BLE-Test] Services: {len(client.services.services)}")
                await client.start_notify(
                    args.char_uuid,
                    lambda _sender, data: frame_parser.feed(data),
                )
                print(f"[BLE-Test] Notifications active on {args.char_uuid}")
                while client.is_connected and not stop.is_set():
                    try:
                        await asyncio.wait_for(stop.wait(), timeout=0.5)
                    except asyncio.TimeoutError:
                        pass
                if client.is_connected:
                    await client.stop_notify(args.char_uuid)
                elif not stop.is_set():
                    print("[BLE-Test] Device disconnected")
        except Exception as exc:
            print(f"[BLE-Test] Connection error: {type(exc).__name__}: {exc}")

        if stop.is_set() or not args.reconnect:
            break
        try:
            await asyncio.wait_for(stop.wait(), timeout=args.retry_delay)
        except asyncio.TimeoutError:
            pass

    print("[BLE-Test] Stopped")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mac", default=DEFAULT_MAC)
    parser.add_argument("--char-uuid", default=DEFAULT_CHAR_UUID)
    parser.add_argument("--connect-timeout", type=float, default=15.0)
    parser.add_argument("--retry-delay", type=float, default=5.0)
    parser.add_argument("--reconnect", action="store_true")
    parser.add_argument("--scan", action="store_true")
    parser.add_argument("--scan-timeout", type=float, default=8.0)
    args = parser.parse_args()
    asyncio.run(run(args))


if __name__ == "__main__":
    main()
