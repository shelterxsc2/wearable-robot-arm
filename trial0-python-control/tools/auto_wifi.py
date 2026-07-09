#!/usr/bin/env python3
"""Auto-connect to the last known WiFi on boot.

Usage:
    tools/auto_wifi.py [iface]

This script is meant to be run by a systemd service at boot. It reuses the
last successfully connected WiFi credentials stored in
~/.config/elf_wifi/last.json. If that network is unavailable, it falls back
to:
    1. "iQOO 12" / "070103xsc"
    2. "tkh1288" / "supercell5000"
"""
import glob
import os
import sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, BASE_DIR)

from system_init import init_wifi


def _guess_wifi_iface() -> str:
    """Return the first wireless interface found on the system."""
    for path in glob.glob("/sys/class/net/*/wireless"):
        return os.path.basename(os.path.dirname(path))
    for iface in ("wlo1", "wlan0", "wlp2s0", "wlp1s0"):
        if os.path.exists(f"/sys/class/net/{iface}"):
            return iface
    return "wlan0"


def main() -> int:
    iface = sys.argv[1] if len(sys.argv) > 1 else _guess_wifi_iface()
    print(f"[AutoWiFi] Using interface: {iface}")

    fallback_list = [
        ("iQOO 12", "070103xsc"),
        ("tkh1288", "supercell5000"),
    ]

    ok = init_wifi(iface=iface, fallback_list=fallback_list)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
