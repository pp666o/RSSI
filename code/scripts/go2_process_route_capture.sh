#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_process_route_capture.sh <capture_dir> [route_name]

Run this on the control computer after copying a Go2 route capture directory.
It builds the offline voxel map and exports a route file for Go2 replay.

Outputs:
  <capture_dir>/offline_map/waypoints_relative_text.txt
  <capture_dir>/offline_map/replay_route_<route_name>.txt
  <capture_dir>/offline_map/replay_route_latest.txt
EOF
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 1
fi

capture_dir="$(realpath "$1")"
route_name="${2:-$(basename "$capture_dir")}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
offline_dir="${OUTPUT_DIR:-$capture_dir/offline_map}"

"$project_root/scripts/go2_build_offline_voxel_map.sh" "$capture_dir"

src="$offline_dir/waypoints_relative_text.txt"
if [[ ! -s "$src" ]]; then
  echo "Route file not generated or empty: $src" >&2
  exit 2
fi

route_file="$offline_dir/replay_route_${route_name}.txt"
cp "$src" "$route_file"
ln -sfn "$(basename "$route_file")" "$offline_dir/replay_route_latest.txt"

echo "Route processing complete."
echo "  capture_dir: $capture_dir"
echo "  route_file:  $route_file"
echo "  latest_link: $offline_dir/replay_route_latest.txt"
echo
echo "Copy route_file to the Go2, then run:"
echo "  scripts/go2_robot_replay_route.sh /path/on/go2/$(basename "$route_file")"
