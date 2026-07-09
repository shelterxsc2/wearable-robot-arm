# -*- coding: utf-8 -*-
"""Minimal NRF24L01+ driver over a bit-banged SPI master.

This driver targets the head-IMU use case in the ELF control chain:
* Configure NRF24 as PRX on a fixed address/channel
* Poll IRQ or status and read 22-byte payloads from RX FIFO
* Expose the latest payload as a dict compatible with elf_control_chain

The SPI master is injected via the `spi` object, which must provide::

    transfer(tx: List[int]) -> List[int]

as implemented by dk2500_spidev.BitBangSpi.
"""
from __future__ import annotations

import time
from typing import Callable, List, Optional

# NRF24 register map
REG_CONFIG = 0x00
REG_EN_AA = 0x01
REG_EN_RXADDR = 0x02
REG_SETUP_AW = 0x03
REG_SETUP_RETR = 0x04
REG_RF_CH = 0x05
REG_RF_SETUP = 0x06
REG_STATUS = 0x07
REG_OBSERVE_TX = 0x08
REG_CD = 0x09
REG_RX_ADDR_P0 = 0x0A
REG_RX_ADDR_P1 = 0x0B
REG_RX_ADDR_P2 = 0x0C
REG_RX_ADDR_P3 = 0x0D
REG_RX_ADDR_P4 = 0x0E
REG_RX_ADDR_P5 = 0x0F
REG_TX_ADDR = 0x10
REG_RX_PW_P0 = 0x11
REG_RX_PW_P1 = 0x12
REG_RX_PW_P2 = 0x13
REG_RX_PW_P3 = 0x14
REG_RX_PW_P4 = 0x15
REG_RX_PW_P5 = 0x16
REG_FIFO_STATUS = 0x17
REG_DYNPD = 0x1C
REG_FEATURE = 0x1D

# Commands
CMD_R_REGISTER = 0x00
CMD_W_REGISTER = 0x20
CMD_R_RX_PAYLOAD = 0x61
CMD_W_TX_PAYLOAD = 0xA0
CMD_FLUSH_TX = 0xE1
CMD_FLUSH_RX = 0xE2
CMD_REUSE_TX_PL = 0xE3
CMD_NOP = 0xFF

# STATUS bits
STATUS_RX_DR = 0x40
STATUS_TX_DS = 0x20
STATUS_MAX_RT = 0x10
STATUS_TX_FULL = 0x01

# CONFIG bits
CONFIG_PRIM_RX = 0x01
CONFIG_PWR_UP = 0x02
CONFIG_CRC0 = 0x04
CONFIG_EN_CRC = 0x08
CONFIG_MASK_MAX_RT = 0x10
CONFIG_MASK_TX_DS = 0x20
CONFIG_MASK_RX_DR = 0x40


class Nrf24L01:
    def __init__(self, spi,
                 ce_set: Callable[[int], None],
                 irq_read: Optional[Callable[[], int]] = None,
                 channel: int = 0x53,
                 rx_address: bytes = b"\xB3\x47\xA1\x82\x69",
                 payload_width: int = 22,
                 rf_setup: int = 0x47):
        self.spi = spi
        self.ce_set = ce_set
        self.irq_read = irq_read
        self.channel = channel
        self.rx_address = rx_address
        self.payload_width = payload_width
        self.rf_setup = rf_setup

    def _cmd(self, cmd: int, data: List[int]) -> List[int]:
        return self.spi.transfer([cmd] + data)

    def read_reg(self, reg: int, length: int = 1) -> List[int]:
        return self._cmd(CMD_R_REGISTER | reg, [0xFF] * length)[1:]

    def write_reg(self, reg: int, data: List[int] | int):
        if isinstance(data, int):
            data = [data]
        self._cmd(CMD_W_REGISTER | reg, list(data))

    def flush_rx(self):
        self.spi.transfer([CMD_FLUSH_RX])

    def flush_tx(self):
        self.spi.transfer([CMD_FLUSH_TX])

    def get_status(self) -> int:
        return self._cmd(CMD_NOP, [0xFF])[0]

    def init_prx(self):
        """Initialize NRF24 as PRX.  Register values must match TX side exactly.

        Defaults are taken from elf_info/wearable-robot-arm/src/nrf24_linux.h:
        CONFIG=0x0F, EN_AA=0x01, EN_RXADDR=0x01, SETUP_AW=0x03,
        SETUP_RETR=0x55, RF_CH=0x53, RF_SETUP=0x47, RX_PW_P0=0x16,
        FEATURE=0x00, DYNPD=0x00.
        """
        self.ce_set(0)
        time.sleep(0.01)

        # Bring the chip up with CRC enabled, then program the registers.
        self.write_reg(REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRC0)
        time.sleep(0.01)

        self.write_reg(REG_EN_AA, 0x01)          # auto-ack on pipe 0
        self.write_reg(REG_EN_RXADDR, 0x01)      # enable pipe 0
        self.write_reg(REG_SETUP_AW, 0x03)       # 5-byte address
        self.write_reg(REG_SETUP_RETR, 0x55)     # 1500 us delay, 5 retries
        self.write_reg(REG_RF_CH, self.channel)
        self.write_reg(REG_RF_SETUP, self.rf_setup)
        self.write_reg(REG_RX_ADDR_P0, list(self.rx_address))
        self.write_reg(REG_TX_ADDR, list(self.rx_address))
        self.write_reg(REG_RX_PW_P0, self.payload_width)
        self.write_reg(REG_DYNPD, 0x00)
        self.write_reg(REG_FEATURE, 0x00)

        self.flush_rx()
        self.flush_tx()
        self.write_reg(REG_STATUS, STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT)

        self.write_reg(REG_CONFIG,
                       CONFIG_EN_CRC | CONFIG_CRC0 | CONFIG_PWR_UP | CONFIG_PRIM_RX)
        time.sleep(0.0015)

        self.ce_set(1)
        time.sleep(0.00013)  # > 130 us to enter RX mode

    def read_payload(self) -> Optional[bytes]:
        """Read one payload from RX FIFO if available."""
        status = self.get_status()
        if status & STATUS_RX_DR:
            rx = self._cmd(CMD_R_RX_PAYLOAD, [0xFF] * self.payload_width)
            self.write_reg(REG_STATUS, STATUS_RX_DR)
            return bytes(rx[1:])
        fifo = self.read_reg(REG_FIFO_STATUS)[0]
        if not (fifo & 0x01):  # RX_EMPTY bit = 0 -> data available
            rx = self._cmd(CMD_R_RX_PAYLOAD, [0xFF] * self.payload_width)
            return bytes(rx[1:])
        return None

    def wait_payload(self, timeout: float = 0.010) -> Optional[bytes]:
        """Block up to `timeout` seconds waiting for a payload."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.irq_read is not None and self.irq_read() == 0:
                return self.read_payload()
            payload = self.read_payload()
            if payload is not None:
                return payload
            time.sleep(0.0005)
        return None
