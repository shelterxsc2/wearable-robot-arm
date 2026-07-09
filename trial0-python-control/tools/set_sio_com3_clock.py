#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Set IT8786 COM3 UART clock source to support high baud rates.

The DK-2500 CON3 UART is provided by the ITE IT8786E-I SIO chip as COM3
(LDN=0x08).  By default the COM3 UART runs from a 1.8432 MHz-equivalent
clock, which limits /dev/ttyS2 to about 115200 baud even when a higher
value is requested in software.

This script enters the SIO configuration mode and changes COM3 Special
Configuration Register 1 (index 0xF0) bits 2:1 to:

  11b -> 24 MHz / 1.625  (~14.7456 MHz UART clock)

With this clock:
  - divisor 2 -> ~460800 baud
  - divisor 1 -> ~921600 baud

The change is effective immediately but is normally reset at next boot
unless the BIOS preserves it.  Run this script from a systemd oneshot
service or cron @reboot to make it persistent.

Usage:
    sudo python3 tools/set_sio_com3_clock.py [--check] [--baud 460800|921600]
"""
from __future__ import annotations

import argparse
import os
import struct
import sys

# ITE IT8786 MB-PnP configuration port pair (default).
CFG_INDEX_PORT = 0x2E
CFG_DATA_PORT = 0x2F

# Logical Device Number for COM3.
LDN_COM3 = 0x08

# Special Configuration Register 1 for COM3.
REG_F0 = 0xF0

# Bits 2:1 of F0 select the UART clock source.
F0_CLOCK_MASK = 0x06
F0_CLOCK_14_7456MHZ = 0x06  # 24 MHz / 1.625


def _open_port():
    """Open /dev/port for byte-wide I/O access."""
    if not os.access("/dev/port", os.R_OK | os.W_OK):
        raise PermissionError(
            "Need read/write access to /dev/port. Run with sudo."
        )
    return open("/dev/port", "r+b", buffering=0)


def _outb(port: int, value: int, fd):
    fd.seek(port)
    fd.write(struct.pack("<B", value & 0xFF))


def _inb(port: int, fd) -> int:
    fd.seek(port)
    return struct.unpack("<B", fd.read(1))[0]


def _enter_config_mode(fd):
    for key in (0x87, 0x01, 0x55, 0x55):
        _outb(CFG_INDEX_PORT, key, fd)


def _exit_config_mode(fd):
    _outb(CFG_INDEX_PORT, 0x02, fd)
    _outb(CFG_DATA_PORT, 0x02, fd)


def _select_ldn(ldn: int, fd):
    _outb(CFG_INDEX_PORT, 0x07, fd)
    _outb(CFG_DATA_PORT, ldn, fd)


def _read_reg(reg: int, fd) -> int:
    _outb(CFG_INDEX_PORT, reg, fd)
    return _inb(CFG_DATA_PORT, fd)


def _write_reg(reg: int, value: int, fd):
    _outb(CFG_INDEX_PORT, reg, fd)
    _outb(CFG_DATA_PORT, value, fd)


def read_com3_f0() -> int:
    with _open_port() as fd:
        _enter_config_mode(fd)
        try:
            _select_ldn(LDN_COM3, fd)
            return _read_reg(REG_F0, fd)
        finally:
            _exit_config_mode(fd)


def set_com3_high_speed_clock(dry_run: bool = False) -> tuple[int, int]:
    with _open_port() as fd:
        _enter_config_mode(fd)
        try:
            _select_ldn(LDN_COM3, fd)
            old = _read_reg(REG_F0, fd)
            new = (old & ~F0_CLOCK_MASK) | F0_CLOCK_14_7456MHZ
            if not dry_run:
                _write_reg(REG_F0, new, fd)
                # Verify.
                verify = _read_reg(REG_F0, fd)
                if verify != new:
                    raise RuntimeError(
                        f"Register write failed: wrote 0x{new:02X}, read back 0x{verify:02X}"
                    )
            return old, new
        finally:
            _exit_config_mode(fd)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="Only read and print the current COM3 F0 register value.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be written but do not modify the SIO.",
    )
    args = parser.parse_args()

    if args.check:
        val = read_com3_f0()
        clk_bits = (val & F0_CLOCK_MASK) >> 1
        print(f"COM3 F0h = 0x{val:02X}, clock bits 2:1 = {clk_bits:02b}")
        if clk_bits == 0:
            print("  -> standard 1.8432 MHz clock, max ~115200 baud")
        elif clk_bits == 3:
            print("  -> 14.7456 MHz clock, supports 460800/921600 baud")
        else:
            print(f"  -> unknown clock setting {clk_bits}")
        return

    try:
        old, new = set_com3_high_speed_clock(dry_run=args.dry_run)
        action = "would change" if args.dry_run else "changed"
        print(f"{action} COM3 F0h from 0x{old:02X} to 0x{new:02X}")
        if not args.dry_run:
            print("COM3 (/dev/ttyS2) now supports 460800/921600 baud until next reboot.")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
