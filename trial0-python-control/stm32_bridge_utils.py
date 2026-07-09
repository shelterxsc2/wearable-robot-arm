# -*- coding: utf-8 -*-
"""Shared helpers for STM32F103 DataHub bridge serial ports.

The F103 DataHub bridge uses two serial links:
  - bridge link: USART3 @ 460800 (host commands in, sensor/arm frames out)
  - arm link:    USART2 @ 115200 (H7 robotic arm data in, commands out)

On Linux both links are usually USB-to-TTL adapters enumerated as /dev/ttyUSB*.
They are identical CH341 devices, so the kernel may assign either index to
either physical connector.  The helpers here detect which /dev/ttyUSB* is the
bridge by looking for A5 bridge frames, and add a startup delay after opening
because pyserial asserts DTR by default and can reset the F103.
"""
from __future__ import annotations

import glob
import time
from typing import Optional

try:
    import serial
except Exception as exc:  # pragma: no cover
    raise RuntimeError("stm32_bridge_utils requires 'pyserial'") from exc

BRIDGE_SOF0 = 0xA5
DEFAULT_BRIDGE_BAUD = 460800
DEFAULT_ARM_BAUD = 115200
DEFAULT_STARTUP_DELAY_S = 2.0


def list_usb_serial_ports() -> list:
    """Return sorted list of /dev/ttyUSB* and /dev/ttyACM* ports."""
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))


def detect_bridge_port(
    baud: int = DEFAULT_BRIDGE_BAUD,
    listen_s: float = 0.5,
) -> Optional[str]:
    """Probe USB serial ports and return the one emitting A5 bridge frames.

    Returns None if no bridge frame is seen on any port.
    """
    for port in list_usb_serial_ports():
        try:
            with serial.Serial(port, baud, timeout=0.1) as s:
                s.reset_input_buffer()
                time.sleep(listen_s)
                data = s.read(s.in_waiting)
                if bytes([BRIDGE_SOF0]) in data:
                    return port
        except Exception:
            continue
    return None


def detect_serial_ports(
    bridge_baud: int = DEFAULT_BRIDGE_BAUD,
    listen_s: float = 0.5,
) -> tuple:
    """Return (bridge_port, arm_port) guesses.

    If a port emitting A5 bridge frames is found, that becomes the bridge port
    and the other available USB serial port becomes the arm port.  Falls back
    to sorted order when no bridge frames are seen.
    """
    usb = list_usb_serial_ports()
    if len(usb) < 2:
        return (usb[0], None) if usb else (None, None)

    bridge = detect_bridge_port(bridge_baud, listen_s)
    if bridge is not None:
        arm = [p for p in usb if p != bridge][0]
        return bridge, arm

    # Fallback: assume first enumerated port is the bridge.
    return usb[0], usb[1]


def open_serial_port(
    port: str,
    baud: int,
    startup_delay_s: float = DEFAULT_STARTUP_DELAY_S,
) -> serial.Serial:
    """Open a serial port and wait for the target to recover from DTR reset.

    pyserial asserts DTR by default when opening a CH341 adapter.  If the
    adapter's DTR pin is wired to the target MCU reset, opening the port resets
    the F103.  A short delay gives the F103 time to boot before the caller
    starts sending commands.
    """
    s = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.05,
        write_timeout=1.0,
    )
    s.reset_input_buffer()
    s.reset_output_buffer()
    if startup_delay_s > 0:
        time.sleep(startup_delay_s)
    return s


def drain_serial_port(s: serial.Serial, duration_s: float = 0.2) -> int:
    """Thoroughly drain a CH341 USB-to-TTL receive buffer.

    ``serial.reset_input_buffer()`` only clears the kernel-side buffer.  Some
    CH341 adapters keep one or more bytes in their internal FIFO, which then
    leak into the next read and shift the byte stream.  This helper keeps
    reading for ``duration_s`` seconds so the hardware FIFO is emptied as well.

    Returns the number of bytes discarded.
    """
    deadline = time.time() + duration_s
    discarded = 0
    while time.time() < deadline:
        try:
            chunk = s.read(s.in_waiting or 1)
            discarded += len(chunk)
        except Exception:
            time.sleep(0.005)
    return discarded
