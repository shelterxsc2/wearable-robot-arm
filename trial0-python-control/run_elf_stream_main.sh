#!/bin/bash
# Run the main visual+control demo with RTMP/RTSP streaming enabled.
# No local display window is opened (--headless); video is pushed to the
# given stream URL via ffmpeg.
#
# Usage:
#   ./run_elf_stream_main.sh rtmp://192.168.1.10:1935/live/device-002
#   ./run_elf_stream_main.sh rtsp://192.168.1.10:8554/live/device-002 --voice
#
# Additional args are forwarded to camera_demo_pingpong_async_pnp.py.

set -e

cd "$(dirname "$0")"

STREAM_URL="${1:-}"
if [ -z "$STREAM_URL" ]; then
    echo "Usage: $0 <rtmp://... or rtsp://...> [extra args]"
    exit 1
fi
shift

./run_elf_main.sh \
    --headless \
    --stream-url "${STREAM_URL}" \
    "$@"
