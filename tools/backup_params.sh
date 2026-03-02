#!/usr/bin/env bash
# backup_params.sh — Back up current ArduCopter parameters via MAVProxy
# Usage: ./tools/backup_params.sh [device] [baud]
#
# Saves params to: ardupilot/params/backup_YYYYMMDD_HHMMSS.param
#
# Exit codes:
#   0 — success
#   1 — no device found
#   2 — specified device not found
#   3 — mavproxy not installed
#   4 — backup failed

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PARAMS_DIR="${REPO_ROOT}/ardupilot/params"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BACKUP_FILE="${PARAMS_DIR}/backup_${TIMESTAMP}.param"
DEFAULT_BAUD=921600

err() {
    echo "ERROR: $*" >&2
}

# --- 1. Check mavproxy ---
if ! command -v mavproxy.py >/dev/null 2>&1; then
    err "mavproxy.py not found. Install with: pip3 install mavproxy"
    exit 3
fi

# --- 2. Detect or validate device ---
if [[ $# -ge 1 ]]; then
    DEVICE="$1"
    if [[ ! -e "${DEVICE}" ]]; then
        err "Device not found: ${DEVICE}"
        exit 2
    fi
else
    # DESIGN: /dev/cu.usbmodem* is the CDC-ACM path on macOS for H743-mini v3 USB.
    # cu.* is preferred over tty.* because it does not echo characters back.
    shopt -s nullglob
    devices=( /dev/cu.usbmodem* )
    shopt -u nullglob

    if [[ ${#devices[@]} -eq 0 ]]; then
        err "No FC device detected on USB (/dev/cu.usbmodem*)."
        err "Connect the H743-mini v3 via USB or specify path: $0 /dev/cu.usbmodem<N>"
        exit 1
    fi
    DEVICE="${devices[0]}"
fi

BAUD="${2:-${DEFAULT_BAUD}}"

# --- 3. Ensure output directory exists ---
mkdir -p "${PARAMS_DIR}"

# --- 4. Display info ---
echo "========================================"
echo " Drone Swarm SLAM — Parameter Backup"
echo "========================================"
printf "  Device    : %s\n" "${DEVICE}"
printf "  Baud      : %s\n" "${BAUD}"
printf "  Output    : %s\n" "${BACKUP_FILE}"
echo "========================================"
echo ""
echo "Connecting to FC and fetching parameters..."

# --- 5. Connect and backup ---
# DESIGN: --cmd runs commands sequentially; param fetch blocks until all params
# are received from the FC before param save executes.
# Logs go to /tmp to avoid cluttering the repository.
if ! mavproxy.py \
        --master "${DEVICE}" \
        --baud "${BAUD}" \
        --logfile /tmp/mavproxy_backup_"${TIMESTAMP}".tlog \
        --cmd "param fetch; param save ${BACKUP_FILE}; exit"; then
    err "MAVProxy failed. Check USB connection and FC power."
    exit 4
fi

if [[ ! -f "${BACKUP_FILE}" ]]; then
    err "Backup file was not created. Check MAVProxy output above."
    exit 4
fi

# Count param lines (NAME,VALUE format; skip comments and blank lines)
PARAM_COUNT="$(grep -c '^[A-Z]' "${BACKUP_FILE}" || true)"

echo ""
echo "SUCCESS: Backup saved."
printf "  File       : %s\n" "${BACKUP_FILE}"
printf "  Parameters : %d\n" "${PARAM_COUNT}"
