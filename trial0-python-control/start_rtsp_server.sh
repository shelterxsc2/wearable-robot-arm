#!/bin/bash
# Start a local RTSP server for the default fallback stream URL.
# Default push URL: rtsp://127.0.0.1:8554/stream
#
# Usage:
#   ./start_rtsp_server.sh
#
# Then in another terminal:
#   ./run_elf_main.sh

set -e

cd "$(dirname "$0")"

MEDIAMTX="third_party/mediamtx/mediamtx"
CONFIG="third_party/mediamtx/mediamtx.yml"

if [ ! -x "$MEDIAMTX" ]; then
    echo "mediamtx not found: $MEDIAMTX"
    echo "Run the setup command or place mediamtx binary there."
    exit 1
fi

exec "$MEDIAMTX" "$CONFIG"
