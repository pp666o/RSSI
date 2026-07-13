#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_capture_voxel_cloud_raw.sh [output_dir]

Capture raw Unitree VoxelMapCompressed frames and synchronized official
PointCloud2 frames using unitree_sdk2/example/go2/go2_voxel_cloud_logger.cpp.

Default output:
  ~/go2_voxel_cloud_captures/capture_<timestamp>

Environment:
  IFACE             Network interface. Defaults to enp5s0.
  DURATION_SEC      Capture duration. Defaults to 10.
  VOXEL_TOPIC       Defaults to rt/utlidar/voxel_map_compressed.
  CLOUD_TOPIC       Defaults to rt/utlidar/cloud.
  POSE_TOPIC        Defaults to rt/utlidar/robot_pose. Use none to disable.
  ODOM_TOPIC        Defaults to rt/utlidar/robot_odom. Use none to disable.
  MAX_VOXEL_FRAMES  Defaults to 0, unlimited during duration.
  MAX_CLOUD_FRAMES  Defaults to 0, unlimited during duration.
  BUILD_IF_NEEDED   true or false. Defaults to true.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
sdk_dir="$project_root/unitree_sdk2"
build_dir="$sdk_dir/build_go2"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${1:-$HOME/go2_voxel_cloud_captures/capture_${timestamp}}"

iface="${IFACE:-enp5s0}"
duration_sec="${DURATION_SEC:-10}"
voxel_topic="${VOXEL_TOPIC:-rt/utlidar/voxel_map_compressed}"
cloud_topic="${CLOUD_TOPIC:-rt/utlidar/cloud}"
pose_topic="${POSE_TOPIC:-rt/utlidar/robot_pose}"
odom_topic="${ODOM_TOPIC:-rt/utlidar/robot_odom}"
max_voxel_frames="${MAX_VOXEL_FRAMES:-0}"
max_cloud_frames="${MAX_CLOUD_FRAMES:-0}"
build_if_needed="${BUILD_IF_NEEDED:-true}"

exe="$build_dir/bin/go2_voxel_cloud_logger"
if [[ ! -x "$exe" ]]; then
  exe="$build_dir/example/go2/go2_voxel_cloud_logger"
fi

if [[ ! -x "$exe" ]]; then
  if [[ "$build_if_needed" != "true" ]]; then
    echo "go2_voxel_cloud_logger executable not found under $build_dir" >&2
    exit 2
  fi
  cmake -S "$sdk_dir" -B "$build_dir"
  cmake --build "$build_dir" --target go2_voxel_cloud_logger -j"$(nproc)"
fi

if [[ ! -x "$exe" ]]; then
  if [[ -x "$build_dir/bin/go2_voxel_cloud_logger" ]]; then
    exe="$build_dir/bin/go2_voxel_cloud_logger"
  elif [[ -x "$build_dir/example/go2/go2_voxel_cloud_logger" ]]; then
    exe="$build_dir/example/go2/go2_voxel_cloud_logger"
  else
    echo "go2_voxel_cloud_logger executable still not found after build." >&2
    exit 2
  fi
fi

mkdir -p "$output_dir"

echo "Capturing raw voxel/cloud frames:"
echo "  exe:              $exe"
echo "  output_dir:       $output_dir"
echo "  iface:            $iface"
echo "  duration_sec:     $duration_sec"
echo "  voxel_topic:      $voxel_topic"
echo "  cloud_topic:      $cloud_topic"
echo "  pose_topic:       $pose_topic"
echo "  odom_topic:       $odom_topic"
echo "  max_voxel_frames: $max_voxel_frames"
echo "  max_cloud_frames: $max_cloud_frames"
echo

exec "$exe" \
  "$iface" \
  "$output_dir" \
  "$duration_sec" \
  "$voxel_topic" \
  "$cloud_topic" \
  "$max_voxel_frames" \
  "$max_cloud_frames" \
  "$pose_topic" \
  "$odom_topic"
