#!/usr/bin/env python3
"""Raw serial sniffer for STM32F103 DataHub bridge debugging.

Prints every byte received on the selected port in hex, optionally with
ASCII and timestamps.  Useful to verify which DK-2500 COM port the F103 is
actually wired to and whether the F103 is emitting anything.
"""

import argparse
import sys
import time

try:
    import serial
except Exception as exc:  # pragma: no cover
    raise RuntimeError("pyserial is required") from exc


def main() -> int:
    parser = argparse.ArgumentParser(description="Raw serial sniffer")
    parser.add_argument("--port", default="/dev/ttyS2", help="serial port")
    parser.add_argument("--baud", type=int, default=460800, help="baud rate")
    parser.add_argument("--ascii", action="store_true", help="also show ASCII")
    parser.add_argument("--ts", action="store_true", help="prefix timestamps")
    parser.add_argument("--duration", type=float, default=5.0, help="listen duration in seconds")
    parser.add_argument("--hex-only", action="store_true", help="one hex stream per line")
    args = parser.parse_args()

    print(f"Listening on {args.port} @ {args.baud} for {args.duration}s...")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.05) as s:
            s.reset_input_buffer()
            deadline = time.time() + args.duration
            buf = bytearray()
            total = 0
            while time.time() < deadline:
                chunk = s.read(max(1, s.in_waiting))
                if chunk:
                    total += len(chunk)
                    if args.hex_only:
                        print(chunk.hex().upper())
                    else:
                        buf.extend(chunk)
                        while len(buf) >= 16:
                            line = buf[:16]
                            del buf[:16]
                            ts = f"{time.time():.3f} " if args.ts else ""
                            hex_part = " ".join(f"{b:02X}" for b in line)
                            if args.ascii:
                                ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in line)
                                print(f"{ts}{hex_part:<48} {ascii_part}")
                            else:
                                print(f"{ts}{hex_part}")
            if buf and not args.hex_only:
                ts = f"{time.time():.3f} " if args.ts else ""
                hex_part = " ".join(f"{b:02X}" for b in buf)
                if args.ascii:
                    ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in buf)
                    print(f"{ts}{hex_part:<48} {ascii_part}")
                else:
                    print(f"{ts}{hex_part}")
            print(f"Total bytes received: {total}")
    except serial.SerialException as exc:
        print(f"ERROR: cannot open {args.port}: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
