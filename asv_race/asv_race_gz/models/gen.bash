#!/usr/bin/env bash
set -euo pipefail

# Generate buoy_{color}.sdf for every supported color.
# Usage: ./gen_buoys.sh [buoy.sdf.xacro] [output_dir]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XACRO_FILE="${1:-${SCRIPT_DIR}/buoy.sdf.xacro}"
OUT_DIR="${2:-${SCRIPT_DIR}}"

COLORS=(red green blue yellow orange white black)

if [[ ! -f "$XACRO_FILE" ]]; then
    echo "Error: xacro file not found: $XACRO_FILE" >&2
    exit 1
fi

mkdir -p "$OUT_DIR"

for color in "${COLORS[@]}"; do
    out="${OUT_DIR}/buoy_${color}.sdf"
    xacro "$XACRO_FILE" name:="buoy_${color}" color:="${color}" > "$out"
    echo "Generated $out"
done
