#!/bin/bash
# Run the low-latency RTMP + WebSocket streaming demo.
#
# This is a separate entry point from the main visual+control pipeline.
# It opens its own camera, probes the RTMP server, connects WebSocket,
# and pushes video to the cloud.
#
# Configuration (hard-coded in demos/camera_demo_lowlatency.py):
#   DEVICE_ID = "device-002"
#   cloud_ip  = read from /tmp/cloud_ip.txt or default 47.93.162.124
#   WS_URL    = ws://<cloud_ip>/ws?deviceId=<DEVICE_ID>
#   RTMP_URL  = rtmp://<cloud_ip>:1935/live/<DEVICE_ID>
#
# Usage:
#   ./run_elf_streaming.sh
#   CLOUD_IP=192.168.1.10 ./run_elf_streaming.sh

set -e

cd "$(dirname "$0")"

# Optional: write cloud IP so the demo reads it from /tmp/cloud_ip.txt
if [ -n "$CLOUD_IP" ]; then
    echo "$CLOUD_IP" > /tmp/cloud_ip.txt
fi

python3 demos/camera_demo_lowlatency.py
