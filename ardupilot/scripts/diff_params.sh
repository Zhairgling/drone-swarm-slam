#!/usr/bin/env bash
# diff_params.sh — Compare two ArduPilot parameter files
#
# Usage:
#   ./scripts/diff_params.sh <file_a> <file_b>
#   ./scripts/diff_params.sh params/common.param drone_1.param
#
# Output: Lines where the same parameter has different values, or exists in
#         only one file. Comment lines (#) and blank lines are ignored.
#
# Exit codes:
#   0 — files are identical (ignoring comments and whitespace)
#   1 — differences found
#   2 — usage error

set -euo pipefail

if [[ $# -ne 2 ]]; then
    echo "Usage: $0 <file_a> <file_b>" >&2
    exit 2
fi

FILE_A="$1"
FILE_B="$2"

if [[ ! -f "$FILE_A" ]]; then
    echo "Error: '$FILE_A' not found" >&2
    exit 2
fi

if [[ ! -f "$FILE_B" ]]; then
    echo "Error: '$FILE_B' not found" >&2
    exit 2
fi

# Strip comments and blank lines, normalise whitespace, sort by param name.
# ArduPilot format: NAME VALUE  (space or tab separated)
strip_and_sort() {
    grep -v '^\s*#' "$1" \
    | grep -v '^\s*$' \
    | awk '{print $1, $2}' \
    | sort -k1,1
}

TMP_A=$(mktemp)
TMP_B=$(mktemp)
trap 'rm -f "$TMP_A" "$TMP_B"' EXIT

strip_and_sort "$FILE_A" > "$TMP_A"
strip_and_sort "$FILE_B" > "$TMP_B"

# Parameters only in file A (removed or renamed in B)
ONLY_A=$(comm -23 <(awk '{print $1}' "$TMP_A" | sort) \
                   <(awk '{print $1}' "$TMP_B" | sort))

# Parameters only in file B (added in B)
ONLY_B=$(comm -13 <(awk '{print $1}' "$TMP_A" | sort) \
                   <(awk '{print $1}' "$TMP_B" | sort))

# Parameters in both but with different values
CHANGED=$(join "$TMP_A" "$TMP_B" \
    | awk '$2 != $3 {printf "%-35s  %s  →  %s\n", $1, $2, $3}')

DIFF_FOUND=0

if [[ -n "$ONLY_A" ]]; then
    echo "=== Only in $FILE_A (not in $FILE_B) ==="
    while IFS= read -r param; do
        grep "^$param " "$TMP_A"
    done <<< "$ONLY_A"
    echo ""
    DIFF_FOUND=1
fi

if [[ -n "$ONLY_B" ]]; then
    echo "=== Only in $FILE_B (not in $FILE_A) ==="
    while IFS= read -r param; do
        grep "^$param " "$TMP_B"
    done <<< "$ONLY_B"
    echo ""
    DIFF_FOUND=1
fi

if [[ -n "$CHANGED" ]]; then
    echo "=== Changed values (param  A-value  →  B-value) ==="
    echo "$CHANGED"
    echo ""
    DIFF_FOUND=1
fi

if [[ $DIFF_FOUND -eq 0 ]]; then
    echo "No differences found between '$FILE_A' and '$FILE_B'."
fi

exit $DIFF_FOUND
