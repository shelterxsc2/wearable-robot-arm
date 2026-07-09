#!/bin/bash
# Run the main ELF demo with a specific STM32 DataHub bridge UART port.
#
# Usage:
#   ./run_elf_bridge.sh /dev/ttyUSB0
#   ./run_elf_bridge.sh /dev/ttyUSB0 --voice

set -e

cd "$(dirname "$0")"

STM32_PORT="${1:-/dev/ttyUSB0}"
shift || true

./run_elf_main.sh \
    --stm32-uart-port "${STM32_PORT}" \
    "$@"
