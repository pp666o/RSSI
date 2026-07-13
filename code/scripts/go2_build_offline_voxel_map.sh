#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_build_offline_voxel_map.sh <capture_dir>

Build an offline 3D/2D map from a raw SDK capture produced by:
  scripts/go2_capture_voxel_cloud_raw.sh

Outputs under <capture_dir>/offline_map by default:
  global_voxel_map.ply        3D occupied voxel map
  map_2d.pgm / map_2d.yaml    2D occupancy projection
  map_2d_path.svg             quick 2D preview with route overlay
  path_points.csv             connected route points
  path_3d.ply                 route polyline for 3D viewers
  waypoints_relative_text.txt relative x,y,yaw triples for go2_waypoint_follower
  offline_map_summary.json    processing summary

Environment:
  OUTPUT_DIR           Defaults to <capture_dir>/offline_map.
  BIT_ORDER            lsb or msb. Defaults to lsb.
  MAX_FRAMES           0 means all frames. Defaults to 0.
  FRAME_STEP           Decode every Nth frame. Defaults to 1.
  MIN_PATH_DISTANCE_M  Path downsample distance. Defaults to 0.15.
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
output_dir="${OUTPUT_DIR:-$capture_dir/offline_map}"
bit_order="${BIT_ORDER:-lsb}"
max_frames="${MAX_FRAMES:-0}"
frame_step="${FRAME_STEP:-1}"
min_path_distance_m="${MIN_PATH_DISTANCE_M:-0.15}"

exec "$script_dir/build_go2_offline_voxel_map.py" \
  "$capture_dir" \
  --output-dir "$output_dir" \
  --bit-order "$bit_order" \
  --max-frames "$max_frames" \
  --frame-step "$frame_step" \
  --min-path-distance "$min_path_distance_m"
