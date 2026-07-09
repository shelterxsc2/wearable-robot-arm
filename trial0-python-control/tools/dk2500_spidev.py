# -*- coding: utf-8 -*-
"""Bit-banged SPI master using DK-2500 GPIOs.

The DK-2500 CPU GSPI pins (pins 19/21/23/24) are 1.8 V signals and are not
exposed as /dev/spidev by the current Ubuntu kernel.  This module implements
a software SPI master on top of the IT8786 GPIOs provided by `dk2500_gpio`.

It is fast enough for NRF24L01+ (the chip supports SPI clocks well below
1 MHz and typically runs at 4-8 MHz; software SPI on x86 can reach several
hundred kHz to ~1 MHz depending on CPU scheduling).

Example:
    >>> from dk2500_gpio import GPIO6, GPIO20, GPIO21, GPIO5
    >>> from dk2500_spidev import BitBangSpi
    >>> spi = BitBangSpi(sck=GPIO6, mosi=GPIO20, miso=GPIO21, csn=GPIO5)
    >>> spi.transfer([0x00])          # read NRF24 CONFIG register

Note:
    Running this module requires root privileges (needs /dev/port access).
"""
from __future__ import annotations

import os
import time
from typing import List, Optional

from dk2500_gpio import Dk2500Gpio, GpioPin


class BitBangSpi:
    """Software SPI master (mode 0: CPOL=0, CPHA=0)."""

    def __init__(self, sck: GpioPin, mosi: GpioPin, miso: GpioPin, csn: GpioPin,
                 gpio: Optional[Dk2500Gpio] = None,
                 max_speed_hz: Optional[float] = None):
        self._gpio = gpio or Dk2500Gpio()
        self._own_gpio = gpio is None
        self.sck = sck
        self.mosi = mosi
        self.miso = miso
        self.csn = csn
        self.max_speed_hz = max_speed_hz or 500_000
        self._half_period = 0.5 / self.max_speed_hz

        # Initialize pins
        self._gpio.setup(sck, "out")
        self._gpio.setup(mosi, "out")
        self._gpio.setup(miso, "in")
        self._gpio.setup(csn, "out")

        self._gpio.write(csn, 1)
        self._gpio.write(sck, 0)

    def close(self):
        if self._own_gpio:
            self._gpio.close()

    def __enter__(self) -> "BitBangSpi":
        return self

    def __exit__(self, *args):
        self.close()

    def _delay(self):
        if self._half_period > 1e-6:
            time.sleep(self._half_period)

    def _transfer_byte(self, tx: int) -> int:
        rx = 0
        for i in range(7, -1, -1):
            # MOSI
            self._gpio.write(self.mosi, (tx >> i) & 1)
            self._delay()
            # SCK rising edge
            self._gpio.write(self.sck, 1)
            self._delay()
            # MISO sample
            rx_bit = self._gpio.read(self.miso)
            rx = (rx << 1) | rx_bit
            # SCK falling edge
            self._gpio.write(self.sck, 0)
            self._delay()
        return rx

    def transfer(self, tx_data: List[int], keep_cs: bool = False) -> List[int]:
        """Perform one SPI transfer.  CS is asserted for the whole transfer."""
        self._gpio.write(self.csn, 0)
        self._delay()
        rx = [self._transfer_byte(b) for b in tx_data]
        if not keep_cs:
            self._gpio.write(self.csn, 1)
            self._delay()
        return rx

    def read_reg(self, reg: int, length: int = 1) -> List[int]:
        """Read `length` bytes from a NRF24-style register."""
        cmd = [0x00 | reg] + [0xFF] * length
        return self.transfer(cmd)[1:]

    def write_reg(self, reg: int, data: List[int] | int):
        """Write to a NRF24-style register."""
        if isinstance(data, int):
            data = [data]
        self.transfer([0x20 | reg] + list(data))


def _loopback_test():
    import sys
    if os.geteuid() != 0:
        print("This test must run as root.", file=sys.stderr)
        sys.exit(1)

    # Connect MOSI and MISO with a jumper for the loopback test.
    from dk2500_gpio import GPIO6, GPIO20, GPIO21, GPIO5
    print("DK-2500 bit-banged SPI loopback test")
    print("Connect GPIO6(SCK), GPIO20(MOSI), GPIO21(MISO), GPIO5(CSN)")
    print("MOSI and MISO must be shorted for loopback.")
    print("=" * 50)

    with BitBangSpi(sck=GPIO6, mosi=GPIO20, miso=GPIO21, csn=GPIO5,
                    max_speed_hz=100_000) as spi:
        patterns = [
            [0x00],
            [0xFF],
            [0x55, 0xAA],
            [0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80],
            [0xDE, 0xAD, 0xBE, 0xEF],
        ]
        all_ok = True
        for tx in patterns:
            rx = spi.transfer(tx)
            ok = rx == tx
            all_ok = all_ok and ok
            status = "OK" if ok else "FAIL"
            print(f"TX {bytes(tx).hex().upper()} -> RX {bytes(rx).hex().upper()} : {status}")
        print("=" * 50)
        print("LOOPBACK " + ("PASSED" if all_ok else "FAILED"))


if __name__ == "__main__":
    _loopback_test()
