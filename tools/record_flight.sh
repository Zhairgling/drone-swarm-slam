#!/usr/bin/env bash
# record_flight.sh — ROS 2 bag recorder for drone flight sessions
# Usage: ./tools/record_flight.sh [--drone-id N] [--duration SECONDS]
#
# Records all drone and SLAM topics to data/recordings/flight_YYYYMMDD_HHMMSS/
# using MCAP format with zstd compression. Requires the ground-station Docker
# container to be running:
#   docker compose -f docker/docker-compose.runtime.yml up -d ground-station
#
# Flags:
#   --drone-id N     Drone ID to record (default: 1)
#   --duration N     Stop recording after N seconds (default: until Ctrl+C)
#   --help           Show this message and exit
#
# Exit codes:
#   0 — success
#   1 — bad arguments
#   2 — Docker or ground-station container not running
#   3 — recording copy failed

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${REPO_ROOT}/docker/docker-compose.runtime.yml"
SERVICE="ground-station"

# Defaults
DRONE_ID=1
DURATION=""

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
err() { echo "ERROR: $*" >&2; }

usage() {
    sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        --drone-id)
            [[ $# -ge 2 ]] || { err "--drone-id requires a value"; exit 1; }
            DRONE_ID="$2"; shift 2 ;;
        --duration)
            [[ $# -ge 2 ]] || { err "--duration requires a value"; exit 1; }
            DURATION="$2"; shift 2 ;;
        --help|-h) usage 0 ;;
        *) err "Unknown argument: $1"; usage 1 ;;
    esac
done

# ---------------------------------------------------------------------------
# Validate environment
# ---------------------------------------------------------------------------
if ! command -v docker >/dev/null 2>&1; then
    err "docker not found. Install Docker Desktop."
    exit 2
fi

# DESIGN: 'ps -q' returns the container ID only when the service is running.
CONTAINER_ID="$(docker compose -f "${COMPOSE_FILE}" ps -q "${SERVICE}" 2>/dev/null || true)"
if [[ -z "${CONTAINER_ID}" ]]; then
    err "${SERVICE} container is not running."
    err "Start it: docker compose -f docker/docker-compose.runtime.yml up -d ${SERVICE}"
    exit 2
fi

# ---------------------------------------------------------------------------
# Paths and topics
# ---------------------------------------------------------------------------
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="flight_${TIMESTAMP}"
HOST_RECORDINGS="${REPO_ROOT}/data/recordings"
HOST_BAG="${HOST_RECORDINGS}/${BAG_NAME}"
# DESIGN: Write into /tmp inside the container so no volume reconfiguration is
# needed. The bag is copied to the host with 'docker cp' after recording ends.
CONTAINER_BAG="/tmp/ros2_bags/${BAG_NAME}"

TOPICS=(
    "/drone_${DRONE_ID}/tof/pointcloud"
    "/drone_${DRONE_ID}/camera/compressed"
    "/drone_${DRONE_ID}/mavlink/from_fc"
    "/drone_${DRONE_ID}/pose/estimated"
    "/slam/map"
    "/slam/map_3d"
    "/tf"
    "/tf_static"
)

mkdir -p "${HOST_RECORDINGS}"

# ---------------------------------------------------------------------------
# Header
# ---------------------------------------------------------------------------
echo "========================================"
echo " Drone Swarm SLAM — Flight Recorder"
echo "========================================"
printf "  Drone ID  : %s\n" "${DRONE_ID}"
printf "  Topics    : %d topics\n" "${#TOPICS[@]}"
printf "  Format    : MCAP + zstd\n"
printf "  Duration  : %s\n" "${DURATION:-until Ctrl+C}"
printf "  Output    : %s\n" "${HOST_BAG}"
echo "========================================"
echo ""

# ---------------------------------------------------------------------------
# Record
# ---------------------------------------------------------------------------
docker compose -f "${COMPOSE_FILE}" exec "${SERVICE}" \
    mkdir -p "$(dirname "${CONTAINER_BAG}")"

RECORD_ARGS=(
    ros2 bag record
    --storage mcap
    --compression-mode file
    --compression-format zstd
    --output "${CONTAINER_BAG}"
)

if [[ -n "${DURATION}" ]]; then
    RECORD_ARGS+=(--max-duration "${DURATION}")
fi

RECORD_ARGS+=("${TOPICS[@]}")

echo "Recording... (Ctrl+C to stop)"
echo ""

# DESIGN: '|| true' prevents set -e from aborting on Ctrl+C (SIGINT).
# docker compose exec forwards signals to the container process, so
# ros2 bag record flushes and closes the bag cleanly before exit.
docker compose -f "${COMPOSE_FILE}" exec "${SERVICE}" \
    "${RECORD_ARGS[@]}" || true

echo ""
echo "Recording stopped."

# ---------------------------------------------------------------------------
# Summary (while bag is still in the container)
# ---------------------------------------------------------------------------
echo ""
echo "========================================"
echo " Recording Summary"
echo "========================================"
docker compose -f "${COMPOSE_FILE}" exec "${SERVICE}" \
    ros2 bag info "${CONTAINER_BAG}" || true

# ---------------------------------------------------------------------------
# Copy bag to host
# ---------------------------------------------------------------------------
echo ""
echo "Copying bag to host..."
if ! docker cp "${CONTAINER_ID}:${CONTAINER_BAG}" "${HOST_BAG}"; then
    err "Failed to copy bag from container."
    err "Bag is still available inside the container at: ${CONTAINER_BAG}"
    exit 3
fi

# Clean up temp files inside the container
docker compose -f "${COMPOSE_FILE}" exec "${SERVICE}" \
    rm -rf "${CONTAINER_BAG}" 2>/dev/null || true

echo ""
echo "========================================"
printf "  Saved to: %s\n" "${HOST_BAG}"
echo "========================================"
