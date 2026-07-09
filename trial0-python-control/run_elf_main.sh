#!/bin/bash
# Run the main ELF demo in streaming mode by default:
#   bridge + voice + RTMP/WebSocket streaming
# No local window is opened unless --show-window is given.
#
# Usage:
#   ./run_elf_main.sh
#   ./run_elf_main.sh --show-window
#   CLOUD_IP=192.168.1.10 ./run_elf_main.sh
#   RTSP=1 ./run_elf_main.sh
#   ./run_elf_main.sh --no-stream --show-window

set -e

cd "$(dirname "$0")"

STREAM_ARGS=()
if [ -n "$STREAM_URL" ]; then
    STREAM_ARGS+=("--stream-url" "${STREAM_URL}")
fi
if [ -n "$RTSP" ]; then
    STREAM_ARGS+=("--rtsp")
fi

python3 camera_demo_pingpong_async_pnp.py \
    --elf-control \
    --ctrl-port 8080 \
    --ble \
    --ble-mac F8:2E:0C:E3:99:C8 \
    --voice \
    "${STREAM_ARGS[@]}" \
    "$@"
