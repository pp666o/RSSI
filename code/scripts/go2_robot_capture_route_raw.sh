#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_robot_capture_route_raw.sh [output_dir]

Run this on the Go2 computer. It records raw Unitree voxel/cloud/pose/odom
frames until Ctrl+C. The output can later be copied to the control computer
and processed with scripts/go2_process_route_capture.sh.

Environment:
  IFACE        Unitree network interface. Defaults to eth0.
  PROJECT_ROOT Defaults to the parent of this scripts directory.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="${PROJECT_ROOT:-$(cd "$script_dir/.." && pwd)}"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="${1:-$HOME/go2_route_captures/route_${timestamp}}"

mkdir -p "$output_dir" "$HOME/go2_route_captures"
echo "$output_dir" > "$HOME/go2_route_captures/latest_route_capture.txt" 2>/dev/null || true

echo "Go2 route capture:"
echo "  output_dir: $output_dir"
echo "  iface:      ${IFACE:-eth0}"
echo
echo "Press Ctrl+C to stop recording."

exec env \
  IFACE="${IFACE:-eth0}" \
  DURATION_SEC=0 \
  VOXEL_TOPIC="${VOXEL_TOPIC:-rt/utlidar/voxel_map_compressed}" \
  CLOUD_TOPIC="${CLOUD_TOPIC:-rt/utlidar/cloud}" \
  POSE_TOPIC="${POSE_TOPIC:-rt/utlidar/robot_pose}" \
  ODOM_TOPIC="${ODOM_TOPIC:-rt/utlidar/robot_odom}" \
  BUILD_IF_NEEDED="${BUILD_IF_NEEDED:-false}" \
  "$project_root/scripts/go2_capture_voxel_cloud_raw.sh" "$output_dir"
