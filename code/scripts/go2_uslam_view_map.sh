#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_uslam_view_map.sh

Open RViz for Unitree official USLAM map visualization:
  - /uslam/cloud_map
  - /uslam/localization/cloud_world
  - /uslam/localization/odom
  - /utlidar/voxel_map

This script only starts RViz. It does not send movement commands.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
workspace="$project_root/ros2_ws"

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  source_setup "/opt/ros/$ROS_DISTRO/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source_setup /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  source_setup /opt/ros/foxy/setup.bash
else
  echo "No ROS 2 setup.bash found under /opt/ros." >&2
  exit 2
fi
source_setup "$workspace/install/setup.bash"

exec ros2 launch go2_nav2_bridge go2_uslam_mapping_rviz.launch.py start_rviz:=true
