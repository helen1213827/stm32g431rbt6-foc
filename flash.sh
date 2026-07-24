#!/bin/bash
# Flash script for STM32G431 via DAPLink (OpenOCD)
# Usage: ./flash.sh [build_type]
#   build_type: Debug (default) or Release

BUILD_TYPE="${1:-Debug}"
ELF_FILE="build/${BUILD_TYPE}/foc.elf"
CFG_FILE="$(dirname "$0")/openocd.cfg"

if [ ! -f "$ELF_FILE" ]; then
    echo "Error: $ELF_FILE not found. Build first: cmake --preset $BUILD_TYPE -B build/$BUILD_TYPE && cmake --build build/$BUILD_TYPE"
    exit 1
fi

echo "Flashing $ELF_FILE ..."
openocd -f "$CFG_FILE" \
    -c "init" \
    -c "reset init" \
    -c "halt" \
    -c "program $ELF_FILE verify reset exit"

if [ $? -eq 0 ]; then
    echo "=== Flash complete! ==="
else
    echo "=== Flash FAILED ==="
    exit 1
fi
