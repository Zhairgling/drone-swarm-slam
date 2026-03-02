#!/usr/bin/env bash
# upload_params.sh — Upload ArduCopter parameter file via MAVProxy
# Usage: ./tools/upload_params.sh <param_file> [device] [baud]
#
# Safety steps:
#   1. Auto-backup current params before uploading
#   2. Show diff vs new params (if diff_params.sh exists)
#   3. Require interactive confirmation before uploading
#   4. Spot-check a few key params after upload
#
# Exit codes:
#   0 — success or user aborted
#   1 — no device found
#   2 — specified device not found
#   3 — mavproxy not installed
#   4 — param file argument missing or not found
#   5 — backup failed
#   6 — upload failed

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_BAUD=921600

err() {
    echo "ERROR: $*" >&2
}

# --- 1. Validate param file argument ---
if [[ $# -lt 1 ]]; then
    err "Usage: $0 <param_file> [device] [baud]"
    err "Example: $0 ardupilot/params/companion_serial.param"
    exit 4
fi

PARAM_FILE="$1"
# Resolve relative paths against repo root so the script can be called from anywhere.
if [[ "${PARAM_FILE}" != /* ]]; then
    PARAM_FILE="${REPO_ROOT}/${PARAM_FILE}"
fi

if [[ ! -f "${PARAM_FILE}" ]]; then
    err "Parameter file not found: ${PARAM_FILE}"
    exit 4
fi

# --- 2. Check mavproxy ---
if ! command -v mavproxy.py >/dev/null 2>&1; then
    err "mavproxy.py not found. Install with: pip3 install mavproxy"
    exit 3
fi

# --- 3. Detect or validate device ---
if [[ $# -ge 2 ]]; then
    DEVICE="$2"
    if [[ ! -e "${DEVICE}" ]]; then
        err "Device not found: ${DEVICE}"
        exit 2
    fi
else
    # DESIGN: /dev/cu.usbmodem* is the CDC-ACM path on macOS for H743-mini v3 USB.
    shopt -s nullglob
    devices=( /dev/cu.usbmodem* )
    shopt -u nullglob

    if [[ ${#devices[@]} -eq 0 ]]; then
        err "No FC device detected on USB (/dev/cu.usbmodem*)."
        err "Connect the H743-mini v3 via USB or specify path: $0 <param_file> /dev/cu.usbmodem<N>"
        exit 1
    fi
    DEVICE="${devices[0]}"
fi

BAUD="${3:-${DEFAULT_BAUD}}"

# --- 4. Display upload plan ---
echo "========================================"
echo " Drone Swarm SLAM — Parameter Upload"
echo "========================================"
printf "  Param file : %s\n" "${PARAM_FILE}"
printf "  Device     : %s\n" "${DEVICE}"
printf "  Baud       : %s\n" "${BAUD}"
echo "========================================"
echo ""

# --- 5. Auto-backup before uploading (safety net) ---
echo "Step 1/3: Backing up current parameters (safety net)..."
echo ""

BACKUP_SCRIPT="${SCRIPT_DIR}/backup_params.sh"
if [[ ! -f "${BACKUP_SCRIPT}" ]]; then
    err "backup_params.sh not found: ${BACKUP_SCRIPT}"
    exit 5
fi
if [[ ! -x "${BACKUP_SCRIPT}" ]]; then
    err "backup_params.sh is not executable: ${BACKUP_SCRIPT}"
    exit 5
fi

if ! "${BACKUP_SCRIPT}" "${DEVICE}" "${BAUD}"; then
    err "Backup failed. Upload aborted for safety — no parameters were changed."
    exit 5
fi

# --- 6. Show diff (if diff_params.sh exists) ---
DIFF_SCRIPT="${SCRIPT_DIR}/diff_params.sh"
if [[ -x "${DIFF_SCRIPT}" ]]; then
    echo ""
    echo "Step 2/3: Showing parameter diff..."
    echo ""
    # DESIGN: SC2012 — ls -t needed for mtime sort; filenames are timestamp-based
    # and guaranteed to contain no spaces or special characters.
    # shellcheck disable=SC2012
    LATEST_BACKUP="$(ls -1t "${REPO_ROOT}/ardupilot/params/backup_"*.param 2>/dev/null | head -1 || true)"
    if [[ -n "${LATEST_BACKUP}" ]]; then
        "${DIFF_SCRIPT}" "${LATEST_BACKUP}" "${PARAM_FILE}" || true
    else
        echo "(No backup file found to diff against)"
    fi
else
    echo ""
    echo "Step 2/3: (diff_params.sh not found — skipping diff)"
fi

# --- 7. Require interactive confirmation ---
echo ""
echo "Step 3/3: Confirmation required"
echo ""
printf "  About to upload : %s\n" "${PARAM_FILE}"
printf "  To FC device    : %s @ %s baud\n" "${DEVICE}" "${BAUD}"
echo ""

# DESIGN: Hard 'read -r -p' blocks scripted/piped execution — intentional.
# The operator must confirm by typing 'y' at the terminal before any params change.
read -r -p "Upload parameters to FC? [y/N] " REPLY
echo ""

case "${REPLY}" in
    y|Y) ;;
    *)
        echo "Aborted. No parameters were changed."
        exit 0
        ;;
esac

# --- 8. Upload via MAVProxy ---
echo "Uploading parameters..."
UPLOAD_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

# DESIGN: 'param load' sends PARAM_SET for each line; ArduPilot auto-saves to
# EEPROM on receipt. 'param fetch' re-downloads from FC for subsequent show.
if ! mavproxy.py \
        --master "${DEVICE}" \
        --baud "${BAUD}" \
        --logfile /tmp/mavproxy_upload_"${UPLOAD_TIMESTAMP}".tlog \
        --cmd "param load ${PARAM_FILE}; param fetch; exit"; then
    err "Upload failed. Check USB connection and FC power."
    exit 6
fi

# --- 9. Spot-check a few key params ---
echo ""
echo "Verifying upload (re-reading params from FC)..."

# Build show commands for first 3 params in the uploaded file.
VERIFY_CMD=""
while IFS=',' read -r pname _; do
    VERIFY_CMD="${VERIFY_CMD}param show ${pname}; "
done < <(grep '^[A-Z]' "${PARAM_FILE}" | head -3)

if [[ -n "${VERIFY_CMD}" ]]; then
    VERIFY_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
    mavproxy.py \
        --master "${DEVICE}" \
        --baud "${BAUD}" \
        --logfile /tmp/mavproxy_verify_"${VERIFY_TIMESTAMP}".tlog \
        --cmd "${VERIFY_CMD}exit" || true
fi

echo ""
echo "SUCCESS: Parameters uploaded to FC."
printf "  Source : %s\n" "${PARAM_FILE}"
printf "  Device : %s\n" "${DEVICE}"
printf "  Note   : Some params require a FC reboot to take effect.\n"
