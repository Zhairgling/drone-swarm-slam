#!/usr/bin/env bash
# setup_fc.sh — First-time FC setup: load params, reboot, run calibrations
# Usage: ./tools/setup_fc.sh [device] [baud]
#
# Guides the operator through the full first-time setup sequence:
#   1. Pre-checks (mavproxy installed, FC on USB, props-off confirmation)
#   2. Load common.param
#   3. Load companion_serial.param
#   4. Reboot FC (params that require reboot take effect)
#   5. Accelerometer calibration (6 orientations, interactive)
#   6. Compass calibration (rotate drone, interactive)
#   7. RC calibration (move sticks to limits, interactive)
#   8. ESC calibration note (DSHOT600 — no calibration required)
#   9. Final param verification and summary
#
# Reference:
#   tools/upload_params.sh  — single-file param upload
#   tools/backup_params.sh  — param backup
#
# Exit codes:
#   0 — success or user aborted
#   1 — no USB device found
#   2 — specified device not found
#   3 — mavproxy.py not installed
#   4 — common.param load failed
#   5 — companion_serial.param load failed

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PARAMS_DIR="${REPO_ROOT}/ardupilot/params"
COMMON_PARAM="${PARAMS_DIR}/common.param"
COMPANION_PARAM="${PARAMS_DIR}/companion_serial.param"
DEFAULT_BAUD=921600
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

err()  { echo "ERROR: $*" >&2; }
info() { echo "  $*"; }

# --- 1. Check mavproxy ---
if ! command -v mavproxy.py >/dev/null 2>&1; then
    err "mavproxy.py not found. Install with: pip3 install mavproxy"
    exit 3
fi

# --- 2. Validate param files exist ---
if [[ ! -f "${COMMON_PARAM}" ]]; then
    err "common.param not found: ${COMMON_PARAM}"
    exit 4
fi
if [[ ! -f "${COMPANION_PARAM}" ]]; then
    err "companion_serial.param not found: ${COMPANION_PARAM}"
    exit 5
fi

# --- 3. Detect or validate device ---
if [[ $# -ge 1 ]]; then
    DEVICE="$1"
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
        err "Connect the H743-mini v3 via USB or specify: $0 /dev/cu.usbmodem<N>"
        exit 1
    fi
    DEVICE="${devices[0]}"
fi

BAUD="${2:-${DEFAULT_BAUD}}"

# --- Banner ---
echo ""
echo "========================================================"
echo "   Drone Swarm SLAM — First-Time FC Setup"
echo "========================================================"
echo ""
info "Device     : ${DEVICE}"
info "Baud       : ${BAUD}"
info "Param dir  : ${PARAMS_DIR}"
echo ""

# --- SAFETY: Props-off warning ---
echo "--------------------------------------------------------"
echo "  !!  SAFETY CHECK — READ BEFORE CONTINUING  !!"
echo "--------------------------------------------------------"
echo ""
echo "  This script will:"
echo "    • Load ArduCopter parameters"
echo "    • Reboot the FC"
echo "    • Run motor-related calibrations"
echo ""
echo "  Before proceeding, confirm ALL of the following:"
echo "    [ ] Propellers are PHYSICALLY REMOVED"
echo "    [ ] Battery is DISCONNECTED (USB power only)"
echo "    [ ] FC is powered and LED is active"
echo "    [ ] You are in a safe, open area"
echo ""
read -r -p "Confirm props OFF and FC ready? [y/N] " REPLY
echo ""
case "${REPLY}" in
    y|Y) ;;
    *)
        echo "Aborted. No changes made."
        exit 0
        ;;
esac

# ==============================================================
# Step 1/7 — Load common.param
# ==============================================================
echo ""
echo "========================================================"
echo "  Step 1/7 — Load common.param"
echo "========================================================"
echo ""
echo "  Loads: EKF3, GPS, compass, failsafes, flight modes,"
echo "         motor protocol (DSHOT600), logging."
echo ""
read -r -p "Load common.param? [y/N/skip] " REPLY
echo ""
case "${REPLY}" in
    y|Y) ;;
    s|S|skip)
        echo "Skipping common.param load."
        ;;
    *)
        echo "Aborted."
        exit 0
        ;;
esac

if [[ "${REPLY}" =~ ^[yY]$ ]]; then
    echo "Loading common.param..."
    if ! mavproxy.py \
            --master "${DEVICE}" \
            --baud "${BAUD}" \
            --logfile "/tmp/mavproxy_setup_common_${TIMESTAMP}.tlog" \
            --cmd "param load ${COMMON_PARAM}; param fetch; exit"; then
        err "Failed to load common.param. Check USB connection and FC power."
        exit 4
    fi
    echo ""
    echo "  OK: common.param loaded."
fi

# ==============================================================
# Step 2/7 — Load companion_serial.param
# ==============================================================
echo ""
echo "========================================================"
echo "  Step 2/7 — Load companion_serial.param"
echo "========================================================"
echo ""
echo "  Loads: SERIAL6 (UART4) config for XIAO ESP32S3 companion,"
echo "         MAVLink2 at 921600 baud, stream rates."
echo ""
read -r -p "Load companion_serial.param? [y/N/skip] " REPLY
echo ""
case "${REPLY}" in
    y|Y) ;;
    s|S|skip)
        echo "Skipping companion_serial.param load."
        ;;
    *)
        echo "Aborted."
        exit 0
        ;;
esac

if [[ "${REPLY}" =~ ^[yY]$ ]]; then
    echo "Loading companion_serial.param..."
    if ! mavproxy.py \
            --master "${DEVICE}" \
            --baud "${BAUD}" \
            --logfile "/tmp/mavproxy_setup_companion_${TIMESTAMP}.tlog" \
            --cmd "param load ${COMPANION_PARAM}; param fetch; exit"; then
        err "Failed to load companion_serial.param."
        exit 5
    fi
    echo ""
    echo "  OK: companion_serial.param loaded."
fi

# ==============================================================
# Step 3/7 — Reboot FC
# ==============================================================
echo ""
echo "========================================================"
echo "  Step 3/7 — Reboot FC"
echo "========================================================"
echo ""
echo "  A reboot is required for SERIAL6_PROTOCOL and other"
echo "  hardware-config params to take effect."
echo ""
read -r -p "Reboot FC now? [y/N/skip] " REPLY
echo ""
case "${REPLY}" in
    y|Y) ;;
    s|S|skip)
        echo "Skipping reboot. Some params may not be active until next power cycle."
        ;;
    *)
        echo "Aborted."
        exit 0
        ;;
esac

if [[ "${REPLY}" =~ ^[yY]$ ]]; then
    echo "Sending reboot command..."
    # DESIGN: || true — MAVProxy may exit non-zero when the FC disconnects mid-reboot.
    mavproxy.py \
        --master "${DEVICE}" \
        --baud "${BAUD}" \
        --logfile "/tmp/mavproxy_setup_reboot_${TIMESTAMP}.tlog" \
        --cmd "reboot; exit" || true
    echo ""
    echo "  FC is rebooting (~10 seconds). Wait for LED to settle."
    read -r -p "  Press [Enter] once FC is ready (LED active / solid)..."
    echo ""
    echo "  OK: FC rebooted."
fi

# ==============================================================
# Step 4/7 — Accelerometer Calibration
# ==============================================================
echo ""
echo "========================================================"
echo "  Step 4/7 — Accelerometer Calibration"
echo "========================================================"
echo ""
echo "  MAVProxy will guide you through 6 orientations:"
echo "    1. Level (top side up)       4. Nose down"
echo "    2. Left side down            5. Nose up"
echo "    3. Right side down           6. Bottom up (upside down)"
echo ""
echo "  Hold each orientation steady until prompted, then press"
echo "  ENTER in the MAVProxy terminal. Type 'exit' when done."
echo ""
read -r -p "Start accelerometer calibration? [y/N/skip] " REPLY
echo ""
case "${REPLY}" in
    y|Y) ;;
    s|S|skip)
        echo "Skipping accelerometer calibration."
        ;;
    *)
        echo "Aborted."
        exit 0
        ;;
esac

if [[ "${REPLY}" =~ ^[yY]$ ]]; then
    echo "Launching MAVProxy — follow the prompts, then type 'exit'..."
    echo ""
    # DESIGN: No 'exit' in --cmd so MAVProxy stays interactive after starting accelcal.
    # The user presses Enter for each orientation inside MAVProxy, then types 'exit'.
    mavproxy.py \
        --master "${DEVICE}" \
        --baud "${BAUD}" \
        --logfile "/tmp/mavproxy_setup_accel_${TIMESTAMP}.tlog" \
        --cmd "accelcal" || true
    echo ""
    echo "  OK: Accelerometer calibration session complete."
fi

# ==============================================================
# Step 5/7 — Compass Calibration
# ==============================================================
echo ""
echo "========================================================"
echo "  Step 5/7 — Compass Calibration"
echo "========================================================"
echo ""
echo "  Rotate the drone slowly through all orientations:"
echo "    • Full 360° rotations in each axis (yaw, pitch, roll)"
echo "    • Figure-8 pattern covers all angles efficiently"
echo "    • Stay clear of metal objects, power cables, computers"
echo ""
echo "  In MAVProxy, type 'magcal start' then rotate the drone."
echo "  Calibration completes automatically. Type 'exit' when done."
echo ""
read -r -p "Start compass calibration? [y/N/skip] " REPLY
echo ""
case "${REPLY}" in
    y|Y) ;;
    s|S|skip)
        echo "Skipping compass calibration."
        ;;
    *)
        echo "Aborted."
        exit 0
        ;;
esac

if [[ "${REPLY}" =~ ^[yY]$ ]]; then
    echo "Launching MAVProxy — type 'magcal start', rotate drone, then 'exit'..."
    echo ""
    # DESIGN: No 'exit' in --cmd — user interacts with magcal interactively.
    mavproxy.py \
        --master "${DEVICE}" \
        --baud "${BAUD}" \
        --logfile "/tmp/mavproxy_setup_compass_${TIMESTAMP}.tlog" \
        --cmd "magcal" || true
    echo ""
    echo "  OK: Compass calibration session complete."
fi

# ==============================================================
# Step 6/7 — RC Calibration
# ==============================================================
echo ""
echo "========================================================"
echo "  Step 6/7 — RC Calibration"
echo "========================================================"
echo ""
echo "  Ensure your ELRS transmitter is ON and bound."
echo ""
echo "  In MAVProxy, type 'rc calibrate', then:"
echo "    • Move all sticks to FULL EXTENTS (4 corners + center)"
echo "    • Move throttle to FULL UP and FULL DOWN"
echo "    • Toggle all switches to every position"
echo "  Type 'rc calibrate done' to save, then 'exit'."
echo ""
read -r -p "Start RC calibration? [y/N/skip] " REPLY
echo ""
case "${REPLY}" in
    y|Y) ;;
    s|S|skip)
        echo "Skipping RC calibration."
        ;;
    *)
        echo "Aborted."
        exit 0
        ;;
esac

if [[ "${REPLY}" =~ ^[yY]$ ]]; then
    echo "Launching MAVProxy — follow the RC calibration steps, then 'exit'..."
    echo ""
    # DESIGN: No 'exit' in --cmd — user moves sticks interactively.
    mavproxy.py \
        --master "${DEVICE}" \
        --baud "${BAUD}" \
        --logfile "/tmp/mavproxy_setup_rc_${TIMESTAMP}.tlog" \
        --cmd "rc calibrate" || true
    echo ""
    echo "  OK: RC calibration session complete."
fi

# ==============================================================
# Step 7a/7 — ESC Calibration (DSHOT note)
# ==============================================================
echo ""
echo "========================================================"
echo "  Step 7/7 — ESC Calibration"
echo "========================================================"
echo ""
echo "  !! PROPS-OFF WARNING — motors will spin during ESC cal !!"
echo ""
echo "  Hardware config: MOT_PWM_TYPE = 6 (DSHOT600)"
echo ""
echo "  DSHOT is a digital protocol — no min/max PWM calibration"
echo "  is required. Your DSHOT600 ESCs are ready to use without"
echo "  any calibration step."
echo ""
echo "  If you replaced ESCs with standard PWM types, you would"
echo "  need to run manual ESC calibration (throttle-high power-on"
echo "  procedure). See ArduCopter ESC Calibration docs."
echo ""
read -r -p "Skip ESC calibration (DSHOT — not required)? [y/N] " REPLY
echo ""
case "${REPLY}" in
    y|Y)
        echo "  OK: ESC calibration skipped (DSHOT600 — not required)."
        ;;
    *)
        echo "  Manual ESC calibration selected."
        echo ""
        echo "  !! CONFIRM: PROPS ARE PHYSICALLY REMOVED !!"
        read -r -p "  Props removed and it is safe to proceed? [y/N] " ESC_CONFIRM
        echo ""
        case "${ESC_CONFIRM}" in
            y|Y)
                echo "  Launching MAVProxy for ESC calibration..."
                mavproxy.py \
                    --master "${DEVICE}" \
                    --baud "${BAUD}" \
                    --logfile "/tmp/mavproxy_setup_esc_${TIMESTAMP}.tlog" \
                    --cmd "esc calibrate" || true
                echo ""
                echo "  OK: ESC calibration session complete."
                ;;
            *)
                echo "  ESC calibration aborted for safety."
                ;;
        esac
        ;;
esac

# ==============================================================
# Step 9 — Final Verification
# ==============================================================
echo ""
echo "========================================================"
echo "  Final Verification"
echo "========================================================"
echo ""
echo "Re-reading key parameters from FC..."
echo ""

VERIFY_TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
# Show params covering each loaded file: common.param (EKF, motor, frame) and
# companion_serial.param (SERIAL6).
VERIFY_CMD="param show AHRS_EKF_TYPE; param show EK3_ENABLE; param show MOT_PWM_TYPE; param show FRAME_CLASS; param show FRAME_TYPE; param show SERIAL6_PROTOCOL; param show SERIAL6_BAUD; exit"

mavproxy.py \
    --master "${DEVICE}" \
    --baud "${BAUD}" \
    --logfile "/tmp/mavproxy_setup_verify_${VERIFY_TIMESTAMP}.tlog" \
    --cmd "${VERIFY_CMD}" || true

# ==============================================================
# Summary
# ==============================================================
echo ""
echo "========================================================"
echo "  Setup Complete — Summary"
echo "========================================================"
echo ""
info "Parameters:"
info "  common.param          — EKF3, GPS, failsafes, flight modes, DSHOT600"
info "  companion_serial.param — SERIAL6 MAVLink2 @ 921600 for XIAO"
echo ""
info "Calibrations run interactively above (check MAVProxy output)."
echo ""
info "Next steps:"
info "  1. Reattach propellers (ensure tightened in correct rotation direction)"
info "  2. Verify arming in Stabilize mode (no errors in GCS)"
info "  3. Perform a brief hover test in a safe open area"
info "  4. Calibrate BATT_VOLT_MULT / BATT_AMP_PERVLT after first flight"
info "  5. Back up final params: ./tools/backup_params.sh"
echo ""
info "Log files: /tmp/mavproxy_setup_*.tlog"
echo ""
