#!/bin/bash
# Persistently configure DK-2500 CON3 (IT8786 COM3 / /dev/ttyS2) for high baud.
#
# Default IT8786 COM3 clock is 1.8432 MHz, limiting /dev/ttyS2 to ~115200.
# This script selects the 24 MHz / 1.625 (~14.7456 MHz) clock source so that
# /dev/ttyS2 can run at 460800 or 921600.
#
# Run at boot:
#   sudo cp tools/sio_com3_highspeed.sh /usr/local/bin/
#   sudo cp tools/sio-com3-highspeed.service /etc/systemd/system/
#   sudo systemctl enable --now sio-com3-highspeed
#
# The change is lost on cold boot unless this script runs again.

set -e

CFG_INDEX=0x2E
CFG_DATA=0x2F
LDN_COM3=0x08
REG_F0=0xF0

# ITE MB-PnP entry key for port 0x2E.
enter_config() {
    python3 - <<PY
import struct
idx=${CFG_INDEX}
for k in (0x87, 0x01, 0x55, 0x55):
    with open('/dev/port', 'r+b', buffering=0) as f:
        f.seek(idx)
        f.write(struct.pack('<B', k))
PY
}

exit_config() {
    python3 - <<PY
import struct
idx=${CFG_INDEX}
data=${CFG_DATA}
with open('/dev/port', 'r+b', buffering=0) as f:
    f.seek(idx); f.write(struct.pack('<B', 0x02))
    f.seek(data); f.write(struct.pack('<B', 0x02))
PY
}

read_f0() {
    python3 - <<PY
import struct
idx=${CFG_INDEX}
data=${CFG_DATA}
ldn=${LDN_COM3}
reg=${REG_F0}
with open('/dev/port', 'r+b', buffering=0) as f:
    f.seek(idx); f.write(struct.pack('<B', 0x07))
    f.seek(data); f.write(struct.pack('<B', ldn))
    f.seek(idx); f.write(struct.pack('<B', reg))
    f.seek(data)
    print(f'{f.read(1)[0]:02X}')
PY
}

write_f0() {
    local val="$1"
    python3 - <<PY
import struct
idx=${CFG_INDEX}
data=${CFG_DATA}
ldn=${LDN_COM3}
reg=${REG_F0}
val=int('${val}', 16)
with open('/dev/port', 'r+b', buffering=0) as f:
    f.seek(idx); f.write(struct.pack('<B', 0x07))
    f.seek(data); f.write(struct.pack('<B', ldn))
    f.seek(idx); f.write(struct.pack('<B', reg))
    f.seek(data); f.write(struct.pack('<B', val))
PY
}

if [ ! -r /dev/port ] || [ ! -w /dev/port ]; then
    echo "Error: need read/write access to /dev/port. Run with sudo." >&2
    exit 1
fi

enter_config
OLD="$(read_f0)"
# Preserve all bits except clock-source bits 2:1, set them to 11b.
NEW=$(printf '%02X' $(( (0x${OLD} & ~0x06) | 0x06 )))
write_f0 "${NEW}"
exit_config

echo "COM3 F0h changed from 0x${OLD} to 0x${NEW}"

# Tell the Linux 16550 driver the new base clock (14.7456 MHz / 16 = 921600).
# This makes termios requests for 460800 and 921600 use the correct divisor.
if [ -c /dev/ttyS2 ]; then
    setserial /dev/ttyS2 baud_base 921600
    echo "/dev/ttyS2 baud_base set to 921600"
fi
