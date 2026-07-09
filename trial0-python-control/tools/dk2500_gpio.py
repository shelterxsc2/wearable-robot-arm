# -*- coding: utf-8 -*-
"""DK-2500 GPIO userspace driver via ITE IT8786 Super I/O.

The DK-2500 40-Pin debug interface exposes several GPIOs that are controlled
by the on-board ITE IT8786 Super I/O.  This module implements the same access
pattern used by the vendor Windows demo (WakIo.dll / wakio.sys):

* Enter IT8786 configuration mode through I/O ports 0x2E/0x2F
* Configure the Simple I/O base address (default 0x0A00)
* Read/write GPIO data through the Simple I/O data registers
* Direction is controlled by the Output/Input Selection registers in LDN 0x07

References:
* /home/time/work/DK/调试接口CON3 资料/IT8786_H_V0.7.2_industrial_20190328.pdf
* /home/time/work/DK/调试接口CON3 资料/客户订制40PIN接口定义-2026-03-16.xlsx
* /home/time/work/DK/调试接口CON3 资料/DK-2500_GPIO_Demo_V1.4/lib_x86/GlobalCfg.ini

Example:
    >>> from dk2500_gpio import Dk2500Gpio, GPIO3, GPIO5
    >>> gpio = Dk2500Gpio()
    >>> gpio.setup(GPIO3, "out")
    >>> gpio.write(GPIO3, 1)
    >>> print(gpio.read(GPIO3))

Note:
    Running this module requires root privileges (or CAP_SYS_RAWIO) because it
    accesses /dev/port.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Dict, List, Literal, Optional

# IT8786 configuration ports used by DK-2500 (see GlobalCfg.ini IO_PORT)
CFG_PORT = 0x2E
DAT_PORT = 0x2F

# Logical device number for GPIO configuration space
LDN_GPIO = 0x07

# Default Simple I/O base address programmed by DK-2500 firmware
DEFAULT_SIO_BASE = 0x0A00


@dataclass(frozen=True)
class GpioPin:
    name: str
    pin_num: int  # 40-pin header number
    sio_gp: str   # e.g. "SIO_GP70"
    group: int    # IT8786 GPIO group (1-9, A)
    bit: int      # bit position inside the group data register
    data_offset: int   # offset from SIO_BASE for the data register
    dir_index: int     # index in LDN=0x07 for the direction register
    level: str    # "3.3V" or "5V"
    notes: str = ""


# GPIOs exported on the 40-pin header, ordered by their logical IDs used in the
# vendor Windows GPIOTest demo.  Only pins that are usable as GPIO are listed.
GPIO3  = GpioPin("GPIO3",  7,  "SIO_GP70", 7, 0, 6, 0xCE, "5V")
GPIO5  = GpioPin("GPIO5",  13, "SIO_GP71", 7, 1, 6, 0xCE, "5V")
GPIO6  = GpioPin("GPIO6",  15, "SIO_GP72", 7, 2, 6, 0xCE, "5V")
GPIO11 = GpioPin("GPIO11", 29, "SIO_GP73", 7, 3, 6, 0xCE, "5V")
GPIO12 = GpioPin("GPIO12", 31, "SIO_GP74", 7, 4, 6, 0xCE, "5V")
GPIO13 = GpioPin("GPIO13", 33, "SIO_GP75", 7, 5, 6, 0xCE, "5V")
GPIO15 = GpioPin("GPIO15", 37, "SIO_GP76", 7, 6, 6, 0xCE, "3.3V")
GPIO19 = GpioPin("GPIO19", 16, "SIO_GP77", 7, 7, 6, 0xCE, "5V")
GPIO20 = GpioPin("GPIO20", 18, "SIO_GP82", 8, 2, 7, 0xCF, "3.3V")
GPIO21 = GpioPin("GPIO21", 22, "SIO_GP86", 8, 6, 7, 0xCF, "3.3V")
GPIO23 = GpioPin("GPIO23", 26, "SIO_GP56", 5, 6, 4, 0xCC, "3.3V",
                 notes="needs index 29h bit6=1")

USER_LED0 = GpioPin("USER_LED0", -1, "SIO_GP80", 8, 3, 7, 0xCF, "3.3V")
USER_LED1 = GpioPin("USER_LED1", -1, "SIO_GP81", 8, 4, 7, 0xCF, "3.3V")

ALL_GPIOS: List[GpioPin] = [
    GPIO3, GPIO5, GPIO6, GPIO11, GPIO12, GPIO13, GPIO15, GPIO19,
    GPIO20, GPIO21, GPIO23, USER_LED0, USER_LED1,
]

_BY_NAME: Dict[str, GpioPin] = {p.name: p for p in ALL_GPIOS}


class Dk2500Gpio:
    """Low-level GPIO access for DK-2500 via IT8786 Super I/O."""

    def __init__(self, sio_base: int = DEFAULT_SIO_BASE,
                 cfg_port: int = CFG_PORT, dat_port: int = DAT_PORT):
        self.sio_base = sio_base
        self.cfg_port = cfg_port
        self.dat_port = dat_port
        self._dev_port = open("/dev/port", "r+b", buffering=0)
        self._check_chip_id()
        self._ensure_sio_base()

    def __enter__(self) -> "Dk2500Gpio":
        return self

    def __exit__(self, *args):
        self.close()

    def close(self):
        if self._dev_port is not None:
            self._dev_port.close()
            self._dev_port = None

    # ------------------------------------------------------------------
    # Raw Super I/O access
    # ------------------------------------------------------------------

    def _cfg_out(self, val: int):
        self._dev_port.seek(self.cfg_port)
        self._dev_port.write(bytes([val & 0xFF]))

    def _dat_out(self, val: int):
        self._dev_port.seek(self.dat_port)
        self._dev_port.write(bytes([val & 0xFF]))

    def _dat_in(self) -> int:
        self._dev_port.seek(self.dat_port)
        return self._dev_port.read(1)[0]

    def _enter_config(self):
        """ITE enter-configuration sequence (GlobalCfg ENTER_KEY=0x55550187)."""
        for b in (0x87, 0x01, 0x55, 0x55):
            self._cfg_out(b)

    def _exit_config(self):
        self._cfg_out(0x02)
        self._dat_out(0x02)

    def _read_reg(self, index: int) -> int:
        self._cfg_out(index)
        return self._dat_in()

    def _write_reg(self, index: int, value: int):
        self._cfg_out(index)
        self._dat_out(value)

    def _select_ldn(self, ldn: int):
        self._write_reg(0x07, ldn)

    def _check_chip_id(self):
        self._enter_config()
        try:
            id1 = self._read_reg(0x20)
            id2 = self._read_reg(0x21)
            chip_id = (id1 << 8) | id2
            if chip_id != 0x8786:
                raise RuntimeError(
                    f"IT8786 not found at 0x{self.cfg_port:02X}/0x{self.dat_port:02X}: "
                    f"chip_id=0x{chip_id:04X}"
                )
        finally:
            self._exit_config()

    def _ensure_sio_base(self):
        """Make sure the Simple I/O base address is programmed to sio_base."""
        self._enter_config()
        try:
            self._select_ldn(LDN_GPIO)
            msb = self._read_reg(0x62)
            lsb = self._read_reg(0x63)
            current = (msb << 8) | lsb
            if current != self.sio_base:
                self._write_reg(0x62, (self.sio_base >> 8) & 0xFF)
                self._write_reg(0x63, self.sio_base & 0xFF)
        finally:
            self._exit_config()

    # ------------------------------------------------------------------
    # GPIO direction / data
    # ------------------------------------------------------------------

    def _read_direction(self, pin: GpioPin) -> int:
        self._enter_config()
        try:
            self._select_ldn(LDN_GPIO)
            return (self._read_reg(pin.dir_index) >> pin.bit) & 1
        finally:
            self._exit_config()

    def _write_direction(self, pin: GpioPin, output: bool):
        self._enter_config()
        try:
            self._select_ldn(LDN_GPIO)
            val = self._read_reg(pin.dir_index)
            mask = 1 << pin.bit
            if output:
                val |= mask
            else:
                val &= ~mask
            self._write_reg(pin.dir_index, val)
            # For GP56, enable the multifunction pin selection bit.
            if pin.sio_gp == "SIO_GP56":
                reg29 = self._read_reg(0x29)
                self._write_reg(0x29, reg29 | 0x40)
        finally:
            self._exit_config()

    def _read_data(self, pin: GpioPin) -> int:
        self._dev_port.seek(self.sio_base + pin.data_offset)
        return (self._dev_port.read(1)[0] >> pin.bit) & 1

    def _write_data(self, pin: GpioPin, val: int):
        self._dev_port.seek(self.sio_base + pin.data_offset)
        cur = self._dev_port.read(1)[0]
        mask = 1 << pin.bit
        if val:
            cur |= mask
        else:
            cur &= ~mask
        self._dev_port.seek(self.sio_base + pin.data_offset)
        self._dev_port.write(bytes([cur]))

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @staticmethod
    def by_name(name: str) -> GpioPin:
        if name not in _BY_NAME:
            raise KeyError(f"Unknown DK-2500 GPIO: {name!r}")
        return _BY_NAME[name]

    def setup(self, pin: GpioPin | str, direction: Literal["in", "out"],
              pull_up: Optional[bool] = None):
        """Set a GPIO direction."""
        if isinstance(pin, str):
            pin = self.by_name(pin)
        self._write_direction(pin, direction == "out")

    def read(self, pin: GpioPin | str) -> int:
        """Read a GPIO value (0 or 1)."""
        if isinstance(pin, str):
            pin = self.by_name(pin)
        return self._read_data(pin)

    def write(self, pin: GpioPin | str, val: int):
        """Write a GPIO value (0 or 1)."""
        if isinstance(pin, str):
            pin = self.by_name(pin)
        self._write_data(pin, val)

    def toggle(self, pin: GpioPin | str) -> int:
        """Toggle a GPIO and return the new value."""
        if isinstance(pin, str):
            pin = self.by_name(pin)
        new = 1 - self._read_data(pin)
        self._write_data(pin, new)
        return new

    def blink(self, pin: GpioPin | str, times: int = 5, period: float = 0.5):
        """Convenience: blink a GPIO."""
        if isinstance(pin, str):
            pin = self.by_name(pin)
        self.setup(pin, "out")
        for _ in range(times):
            self.write(pin, 1)
            time.sleep(period / 2)
            self.write(pin, 0)
            time.sleep(period / 2)


def _self_test():
    import sys
    if os.geteuid() != 0:
        print("This test must run as root (needs /dev/port access).", file=sys.stderr)
        sys.exit(1)

    print("DK-2500 GPIO self test")
    print("=" * 40)
    with Dk2500Gpio() as gpio:
        print(f"Simple I/O base: 0x{gpio.sio_base:04X}")
        for name in ("USER_LED0", "USER_LED1"):
            print(f"Blinking {name} ...")
            gpio.blink(name, times=4, period=0.5)
        print("Done.")


if __name__ == "__main__":
    _self_test()
