# -*- coding: utf-8 -*-
"""HC-08 BLE remote control listener.

This module implements the 3-byte BLE remote protocol from
/home/time/work/elf_info/wearable-robot-arm/src/bluetooth_spp.c in Python.
Instead of calling control functions directly, it sends HTTP POST requests to
the local ELF control server. This keeps it decoupled from the control thread
and lets it run as a standalone helper.

Protocol
--------
    [0x55] [CMD] [VAL]

    CMD 0x00 VAL 0x00     idle (resets debounce state)
    CMD 0x01 VAL 0x00     profile = mid_l3_40
    CMD 0x01 VAL 0x01     profile = far_l3_55
    CMD 0x02 VAL 0x00     scene = FACE
    CMD 0x02 VAL 0x01     scene = INTRO
    CMD 0x02 VAL 0x02     scene = INTERVIEW
    CMD 0x02 VAL 0x03     scene = BODY
    CMD 0x03 VAL any      toggle head pitch sign

Dependencies
------------
 bleak is required for real BLE operation:

    pip install bleak

If bleak is not installed the module can still be imported for testing, but
start() will raise RuntimeError.
"""
from __future__ import annotations

import asyncio
import sys
import threading
import time
import urllib.request
from typing import Optional

# HC-08 configuration (matches upstream bluetooth_spp.c)
HC08_NAME = "HC-08"
HC08_MAC = "F8:2E:0C:E3:99:C8"
HC08_SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
HC08_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

DEFAULT_CTRL_URL = "http://127.0.0.1:8080"


def _ensure_bleak():
    try:
        import bleak  # noqa: F401
        return True
    except ImportError:
        return False


class BleRemoteListener:
    """Listen to an HC-08 BLE remote and dispatch commands via HTTP."""

    def __init__(
        self,
        mac: str = HC08_MAC,
        ctrl_url: str = DEFAULT_CTRL_URL,
    ):
        self.mac = mac
        self.ctrl_url = ctrl_url.rstrip("/")
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_cmd = 0
        self._last_val = 0
        self._active_non_idle = False

    def _http_post(self, path: str) -> bool:
        url = f"{self.ctrl_url}{path}"
        try:
            req = urllib.request.Request(url, method="POST", data=b"")
            with urllib.request.urlopen(req, timeout=2.0) as resp:
                return resp.status == 200
        except Exception as e:
            print(f"[BLE-Remote] HTTP POST {url} failed: {e}")
            return False

    def _dispatch(self, cmd: int, val: int):
        # 0x00 always resets the debounce state.
        if cmd == 0x00:
            self._active_non_idle = False
            self._last_cmd = 0
            self._last_val = 0
            return

        # Repeat suppression: ignore identical (cmd, val) unless it is the
        # toggle command (0x03), which is suppressed only when cmd repeats.
        if self._active_non_idle and cmd == self._last_cmd:
            if cmd == 0x03 or val == self._last_val:
                return

        self._active_non_idle = True
        self._last_cmd = cmd
        self._last_val = val

        if cmd == 0x01:
            if val == 0x00:
                print("[BLE-Remote] profile -> mid_l3_40")
                self._http_post("/profile?idx=0")
            elif val == 0x01:
                print("[BLE-Remote] profile -> far_l3_55")
                self._http_post("/profile?idx=1")
            else:
                print(f"[BLE-Remote] profile value 0x{val:02X} ignored")

        elif cmd == 0x02:
            modes = {
                0x00: "face",
                0x01: "intro",
                0x02: "interview",
                0x03: "body",
            }
            mode = modes.get(val)
            if mode:
                print(f"[BLE-Remote] scene -> {mode}")
                self._http_post(f"/mode?type={mode}")
            else:
                print(f"[BLE-Remote] scene value 0x{val:02X} ignored")

        elif cmd == 0x03:
            print("[BLE-Remote] toggle pitch sign")
            self._http_post("/cmd?action=toggle_pitch_sign")

        else:
            print(f"[BLE-Remote] unknown command cmd=0x{cmd:02X} val=0x{val:02X}")

    def _feed_byte(self, b: int):
        if not hasattr(self, "_frame"):
            self._frame = bytearray()
        if len(self._frame) == 0:
            if b == 0x55:
                self._frame.append(b)
        else:
            self._frame.append(b)
            if len(self._frame) == 3:
                self._dispatch(self._frame[1], self._frame[2])
                self._frame.clear()

    async def _run_async(self):
        if not _ensure_bleak():
            raise RuntimeError("bleak is required for BLE operation: pip install bleak")
        from bleak import BleakClient

        print(f"[BLE-Remote] Connecting to {self.mac} ...")
        while self._running:
            try:
                async with BleakClient(self.mac) as client:
                    print(f"[BLE-Remote] Connected to {self.mac}")
                    await client.start_notify(
                        HC08_CHAR_UUID,
                        lambda _sender, data: [self._feed_byte(b) for b in data],
                    )
                    while self._running and client.is_connected:
                        await asyncio.sleep(0.1)
                    await client.stop_notify(HC08_CHAR_UUID)
            except Exception as e:
                print(f"[BLE-Remote] Connection error: {e}")
                if self._running:
                    await asyncio.sleep(5.0)
        print("[BLE-Remote] Stopped")

    def _run_loop(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._run_async())
        finally:
            loop.run_until_complete(loop.shutdown_asyncgens())
            loop.run_until_complete(loop.shutdown_default_executor())
            loop.close()

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()
        print("[BLE-Remote] Listener started")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def feed_test_frame(self, cmd: int, val: int):
        """Inject a 3-byte frame for unit testing without BLE hardware."""
        self._feed_byte(0x55)
        self._feed_byte(cmd)
        self._feed_byte(val)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="HC-08 BLE remote listener")
    parser.add_argument("--mac", default=HC08_MAC, help="HC-08 MAC address")
    parser.add_argument("--ctrl-url", default=DEFAULT_CTRL_URL,
                        help="ELF control HTTP server URL")
    args = parser.parse_args()

    listener = BleRemoteListener(mac=args.mac, ctrl_url=args.ctrl_url)
    listener.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()


if __name__ == "__main__":
    main()
