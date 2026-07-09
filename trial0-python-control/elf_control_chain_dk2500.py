# -*- coding: utf-8 -*-
"""DK-2500 specific hardware backends for the ELF control chain.

The DK-2500 exposes a 40-pin debug header.  Its GPIOs are driven by the
on-board ITE IT8786 Super I/O.  The CPU GSPI pins are not usable under the
stock Ubuntu kernel, so this backend implements SPI in software on top of the
IT8786 GPIOs and talks to an NRF24L01+ module.

Typical wiring for NRF24L01+ (3.3 V signals):

    DK-2500          NRF24L01+
    -------------------------
    Pin 1  (3.3V) -> VCC
    Pin 6  (GND)  -> GND
    Pin 15 GPIO6  -> SCK   (5V, use series resistor / level shifter)
    Pin 18 GPIO20 -> MISO  (3.3V)
    Pin 22 GPIO21 -> MOSI  (3.3V)
    Pin 13 GPIO5  -> CSN   (5V, use series resistor / level shifter)
    Pin 7  GPIO3  -> CE    (5V, use series resistor / level shifter)
    Pin 29 GPIO11 -> IRQ   (5V, use series resistor / level shifter)

Because most of the usable GPIOs on the 40-pin header are 5 V and the
NRF24L01+ is a 3.3 V part, it is strongly recommended to add a level shifter
or at least 1 kΩ series resistors on the 5 V driven lines.  For a safer and
faster link, connect the NRF24 to a 3.3 V USB-SPI adapter or MCU bridge
instead.
"""
from __future__ import annotations

import os
import sys
import threading
import time
import traceback
from typing import Any, Callable, Dict, Optional

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from elf_control_chain import (
    Nrf24ImuSource,
    NRF24_WY_HIST_SIZE,
    NRF24_WZ_HIST_SIZE,
    NRF24_ANGLE_HIST_SIZE,
    eulerZYXToMat,
    parse_nrf24_imu_payload,
    rotation_matrix_to_quaternion,
)

# Tools are in ./tools relative to this file
TOOLS_DIR = os.path.join(SCRIPT_DIR, "tools")
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

from dk2500_gpio import (
    Dk2500Gpio,
    GPIO3,
    GPIO5,
    GPIO6,
    GPIO11,
    GPIO20,
    GPIO21,
)
from dk2500_spidev import BitBangSpi
from nrf24 import Nrf24L01


class Dk2500Nrf24ImuSource(Nrf24ImuSource):
    """NRF24 head-IMU source using DK-2500 IT8786 GPIO bit-banged SPI.

    The constructor accepts pin assignments so you can choose 3.3 V or 5 V
    GPIOs according to your level-shifting scheme.  Defaults use the pins
    listed in the module docstring.

    Payload parsing follows the 22-byte format defined in
    elf_info/wearable-robot-arm/src/nrf24_linux.c: two 11-byte frames,
    orientation (0x59) and gyro (0x52), each with 0x55 header and a
    checksum byte.
    """

    def __init__(
        self,
        sck=GPIO6,
        mosi=GPIO21,
        miso=GPIO20,
        csn=GPIO5,
        ce=GPIO3,
        irq=GPIO11,
        spi_speed_hz: float = 250_000,
        channel: int = 0x53,
        rx_address: bytes = b"\xB3\x47\xA1\x82\x69",
        payload_width: int = 22,
        rf_setup: int = 0x47,
        on_payload: Optional[Callable[[bytes], Dict[str, Any]]] = None,
    ):
        super().__init__()
        self.sck = sck
        self.mosi = mosi
        self.miso = miso
        self.csn = csn
        self.ce = ce
        self.irq = irq
        self.spi_speed_hz = spi_speed_hz
        self.channel = channel
        self.rx_address = rx_address
        self.payload_width = payload_width
        self.rf_setup = rf_setup
        self.on_payload = on_payload or self._parse_payload

        self._gpio: Optional[Dk2500Gpio] = None
        self._spi: Optional[BitBangSpi] = None
        self._nrf: Optional[Nrf24L01] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        self._latest: Optional[Dict[str, Any]] = None

        # A-init averaging state (mirrors nrf24_linux.c RX thread)
        self._acc_roll = 0.0
        self._acc_pitch = 0.0
        self._acc_yaw = 0.0
        self._acc_count = 0

        # Circular history buffers for the motion FSM
        self._wy_hist = [0.0] * NRF24_WY_HIST_SIZE
        self._wz_hist = [0.0] * NRF24_WZ_HIST_SIZE
        self._wy_idx = 0
        self._wz_idx = 0
        self._wy_count = 0
        self._wz_count = 0

        self._roll_hist = [0.0] * NRF24_ANGLE_HIST_SIZE
        self._pitch_hist = [0.0] * NRF24_ANGLE_HIST_SIZE
        self._yaw_hist = [0.0] * NRF24_ANGLE_HIST_SIZE
        self._angle_idx = 0
        self._angle_count = 0

    def _parse_payload(self, payload: bytes) -> Dict[str, Any]:
        """Parse a 22-byte payload and maintain histories / A-init averages."""
        parsed = parse_nrf24_imu_payload(payload)
        if not parsed["imu_valid"]:
            return parsed

        # A-init: average the first 5 valid orientation frames.
        if not self.r_init_set:
            self._acc_roll += parsed["roll"]
            self._acc_pitch += parsed["pitch"]
            self._acc_yaw += parsed["yaw"]
            self._acc_count += 1
            if self._acc_count >= 5:
                avg_roll = self._acc_roll / 5.0
                avg_pitch = self._acc_pitch / 5.0
                avg_yaw = self._acc_yaw / 5.0
                # Keep quaternion consistent with the averaged Euler angles.
                R = eulerZYXToMat(avg_roll, avg_pitch, avg_yaw)
                qw, qx, qy, qz = rotation_matrix_to_quaternion(R)
                parsed.update(
                    roll=avg_roll,
                    pitch=avg_pitch,
                    yaw=avg_yaw,
                    qw=qw,
                    qx=qx,
                    qy=qy,
                    qz=qz,
                )
                self.r_init_set = True
                print(
                    f"[A-INIT] 5-frame avg captured: "
                    f"roll={avg_roll:.2f} pitch={avg_pitch:.2f} yaw={avg_yaw:.2f}"
                )

        # Update circular history buffers.
        self._wy_hist[self._wy_idx] = parsed["wy"]
        self._wy_idx = (self._wy_idx + 1) % NRF24_WY_HIST_SIZE
        if self._wy_count < NRF24_WY_HIST_SIZE:
            self._wy_count += 1

        self._wz_hist[self._wz_idx] = parsed["wz"]
        self._wz_idx = (self._wz_idx + 1) % NRF24_WZ_HIST_SIZE
        if self._wz_count < NRF24_WZ_HIST_SIZE:
            self._wz_count += 1

        self._roll_hist[self._angle_idx] = parsed["roll"]
        self._pitch_hist[self._angle_idx] = parsed["pitch"]
        self._yaw_hist[self._angle_idx] = parsed["yaw"]
        self._angle_idx = (self._angle_idx + 1) % NRF24_ANGLE_HIST_SIZE
        if self._angle_count < NRF24_ANGLE_HIST_SIZE:
            self._angle_count += 1

        # Expose histories to the controller.
        parsed["wy_hist"] = list(self._wy_hist[: self._wy_count])
        parsed["wz_hist"] = list(self._wz_hist[: self._wz_count])
        parsed["roll_hist"] = list(self._roll_hist[: self._angle_count])
        parsed["pitch_hist"] = list(self._pitch_hist[: self._angle_count])
        parsed["yaw_hist"] = list(self._yaw_hist[: self._angle_count])

        return parsed

    def start(self):
        if os.geteuid() != 0:
            raise RuntimeError(
                "Dk2500Nrf24ImuSource requires root (or CAP_SYS_RAWIO) for /dev/port access"
            )

        self._gpio = Dk2500Gpio()
        self._spi = BitBangSpi(
            sck=self.sck,
            mosi=self.mosi,
            miso=self.miso,
            csn=self.csn,
            gpio=self._gpio,
            max_speed_hz=self.spi_speed_hz,
        )

        self._nrf = Nrf24L01(
            spi=self._spi,
            ce_set=lambda v: self._gpio.write(self.ce, v),
            irq_read=lambda: self._gpio.read(self.irq),
            channel=self.channel,
            rx_address=self.rx_address,
            payload_width=self.payload_width,
            rf_setup=self.rf_setup,
        )

        self._nrf.init_prx()
        status = self._nrf.get_status()
        print(
            f"[DK2500 NRF24] Started on IT8786 GPIO bit-banged SPI, "
            f"status=0x{status:02X}, channel=0x{self.channel:02X}, "
            f"addr={self.rx_address.hex().upper()}"
        )

        self._running = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._nrf is not None:
            try:
                self._gpio.write(self.ce, 0)
            except Exception:
                pass
        if self._spi is not None:
            self._spi.close()
        if self._gpio is not None:
            self._gpio.close()

    def _rx_loop(self):
        """Poll NRF24 RX FIFO and parse every received payload."""
        errs = 0
        while self._running:
            try:
                drained = False
                while self._running:
                    payload = self._nrf.read_payload()
                    if payload is None:
                        break
                    drained = True
                    parsed = self.on_payload(payload)
                    with self._lock:
                        self._latest = parsed
                if not drained:
                    # No data: sleep briefly before polling again.
                    time.sleep(0.001)
                errs = 0
            except Exception:
                errs += 1
                traceback.print_exc()
                time.sleep(min(0.1, 0.01 * errs))

    def poll(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return self._latest


def _demo():
    import sys
    if os.geteuid() != 0:
        print("Run as root.", file=sys.stderr)
        sys.exit(1)

    src = Dk2500Nrf24ImuSource()
    src.start()
    try:
        print("Waiting for NRF24 payloads, press Ctrl-C to stop...")
        while True:
            data = src.poll()
            if data:
                print(data)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        src.stop()


if __name__ == "__main__":
    _demo()
