#!/usr/bin/env python3
"""Find which serial port the F103 DataHub bridge USART3 is on.

For each candidate bridge port we listen for A5 classified bridge frames at the
bridge baud rate.  The first port that emits A5 frames is the bridge link; the
remaining USB serial port is assumed to be the arm/H7 link.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from stm32_bridge_utils import (
    detect_serial_ports,
    list_usb_serial_ports,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bridge-baud",
        type=int,
        default=460800,
        help="Baud rate for the bridge link (default: 460800)",
    )
    args = parser.parse_args()

    print("USB serial ports:", ", ".join(list_usb_serial_ports()) or "none")
    bridge, arm = detect_serial_ports(bridge_baud=args.bridge_baud)
    if bridge is None:
        print("Could not detect the STM32 DataHub bridge port.")
        print("Make sure the F103 is powered, flashed, and connected.")
        return 1

    print(f"Detected bridge port: {bridge} @ {args.bridge_baud}")
    print(f"Detected arm port:    {arm} @ 115200")
    return 0


if __name__ == "__main__":
    sys.exit(main())
