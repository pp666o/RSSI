#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_decode_voxel_capture.sh <capture_dir>

Decode a raw capture produced by scripts/go2_capture_voxel_cloud_raw.sh.

It validates:
  - VoxelMapCompressed src_size
  - decoded output length
  - expected bit-packed voxel size = ceil(128*128*38/8) = 77824

It exports:
  - decoded/*.decoded.bin
  - decoded/*.lsb.xyz.csv and decoded/*.msb.xyz.csv
  - decoded/*.lsb.ply and decoded/*.msb.ply
  - decoded/*.projection.png when Pillow is available
  - decoded nearest rt/utlidar/cloud frame as .cloud.ply/.csv

Environment:
  MAX_FRAMES   Defaults to 5.
  OUTPUT_DIR   Defaults to <capture_dir>/decoded.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi

capture_dir="$1"
if [[ ! -d "$capture_dir" ]]; then
  echo "capture_dir does not exist: $capture_dir" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
max_frames="${MAX_FRAMES:-5}"
output_dir="${OUTPUT_DIR:-$capture_dir/decoded}"

exec "$project_root/scripts/decode_go2_voxel_map.py" \
  "$capture_dir" \
  --output-dir "$output_dir" \
  --max-frames "$max_frames"
