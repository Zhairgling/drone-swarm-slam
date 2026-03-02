#!/usr/bin/env bash
# flash_firmware.sh — Safe idf.py flash wrapper for XIAO ESP32S3 Sense
# Usage: ./tools/flash_firmware.sh [device]
#
# Preflight checks:
#   1. XIAO ESP32S3 detected on USB
#   2. Firmware binary built
#   3. Display info + require interactive confirmation
#
# Exit codes:
#   0 — success or user aborted
#   1 — no USB device found
#   2 — specified device path does not exist
#   3 — firmware binary not built
#   4 — idf.py flash failed

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
FIRMWARE_BIN="${REPO_ROOT}/firmware/build/drone_swarm_firmware.bin"

err() {
    echo "ERROR: $*" >&2
}

# --- 1. Detect or validate device ---
if [[ $# -ge 1 ]]; then
    DEVICE="$1"
    if [[ ! -e "${DEVICE}" ]]; then
        err "Device not found: ${DEVICE}"
        exit 2
    fi
else
    # DESIGN: /dev/cu.usbmodem* is the CDC-ACM path on macOS for XIAO ESP32S3.
    # cu.* is preferred over tty.* because cu.* does not echo characters back.
    shopt -s nullglob
    devices=( /dev/cu.usbmodem* )
    shopt -u nullglob

    if [[ ${#devices[@]} -eq 0 ]]; then
        err "No XIAO ESP32S3 device detected on USB (/dev/cu.usbmodem*)."
        err "Connect the device via USB or specify path: $0 /dev/cu.usbmodem<N>"
        exit 1
    fi
    DEVICE="${devices[0]}"
fi

# --- 2. Verify firmware binary exists ---
if [[ ! -f "${FIRMWARE_BIN}" ]]; then
    err "Firmware binary not found: ${FIRMWARE_BIN}"
    err "Run: cd firmware && idf.py build"
    exit 3
fi

# --- 3. Gather build info ---
# DESIGN: stat -f is macOS BSD; stat -c is GNU/Linux (used in CI containers).
if ! FIRMWARE_SIZE_BYTES="$(stat -f%z "${FIRMWARE_BIN}" 2>/dev/null)"; then
    FIRMWARE_SIZE_BYTES="$(stat -c%s "${FIRMWARE_BIN}")"
fi
FIRMWARE_SIZE_KB=$(( FIRMWARE_SIZE_BYTES / 1024 ))

if ! BUILD_TIMESTAMP="$(stat -f "%Sm" -t "%Y-%m-%d %H:%M:%S" "${FIRMWARE_BIN}" 2>/dev/null)"; then
    if ! BUILD_TIMESTAMP="$(date -r "${FIRMWARE_BIN}" "+%Y-%m-%d %H:%M:%S" 2>/dev/null)"; then
        BUILD_TIMESTAMP="unknown"
    fi
fi

# --- 4. Display preflight summary ---
echo "========================================"
echo " Drone Swarm SLAM — Firmware Flash"
echo "========================================"
printf "  Device    : %s\n" "${DEVICE}"
printf "  Firmware  : %s\n" "${FIRMWARE_BIN}"
printf "  Size      : %d KB (%d bytes)\n" "${FIRMWARE_SIZE_KB}" "${FIRMWARE_SIZE_BYTES}"
printf "  Built at  : %s\n" "${BUILD_TIMESTAMP}"
echo "========================================"
echo ""

# --- 5. Require interactive confirmation ---
# DESIGN: Hard 'read -p' prevents accidental flashing from scripts or CI.
read -r -p "Flash firmware to ${DEVICE}? [y/N] " REPLY
echo ""

case "${REPLY}" in
    y|Y) ;;
    *)
        echo "Aborted."
        exit 0
        ;;
esac

# --- 6. Flash ---
echo "Flashing..."
if idf.py -p "${DEVICE}" flash; then
    echo ""
    echo "SUCCESS: Firmware flashed to ${DEVICE}."
    exit 0
else
    err "Flash failed. Check USB connection and retry."
    exit 4
fi
