#!/bin/bash
# Run the main ELF demo with local display + voice KWS enabled.
# This is a convenience wrapper around run_elf_main.sh that adds --voice.
# Hardware defaults and control chain are unchanged from the Base setup.
#
# Usage:
#   ./run_elf_voice.sh
#   ./run_elf_voice.sh --voice-provider openvino
#   ./run_elf_voice.sh --headless --frames 1000

set -e

cd "$(dirname "$0")"

./run_elf_main.sh \
    --voice \
    "$@"
