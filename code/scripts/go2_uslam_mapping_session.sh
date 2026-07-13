#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_uslam_mapping_session.sh [output_bag_dir]

Start a Go2 3D mapping session:
  1. Open RViz for official USLAM / utlidar 3D map visualization.
  2. Convert /utlidar/robot_pose into /go2/mapping_path for live trajectory display.
  3. Record USLAM, utlidar map, robot pose, IMU, and TF topics into a rosbag.

This script does not send walking commands. Move the robot manually during mapping.
Press Ctrl+C once to stop recording and close the launched visualization.

Environment:
  START_RVIZ             true/false. Defaults to true.
  START_PATH_VISUALIZER  true/false. Defaults to true.
  POSE_TOPIC             Defaults to /utlidar/robot_pose.
  PATH_TOPIC             Defaults to /go2/mapping_path.
  MIN_PATH_DISTANCE_M    Defaults to 0.05.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
workspace="$project_root/ros2_ws"
timestamp="$(date +%Y%m%d_%H%M%S)"
bag_dir="${1:-$HOME/go2_uslam_bags/uslam_mapping_${timestamp}}"
start_rviz="${START_RVIZ:-true}"
start_path_visualizer="${START_PATH_VISUALIZER:-true}"
pose_topic="${POSE_TOPIC:-/utlidar/robot_pose}"
path_topic="${PATH_TOPIC:-/go2/mapping_path}"
min_path_distance_m="${MIN_PATH_DISTANCE_M:-0.05}"

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

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting Go2 USLAM mapping visualization..."
ros2 launch go2_nav2_bridge go2_uslam_mapping_rviz.launch.py \
  start_rviz:="$start_rviz" \
  start_path_visualizer:="$start_path_visualizer" \
  pose_topic:="$pose_topic" \
  path_topic:="$path_topic" \
  min_path_distance_m:="$min_path_distance_m" &
launch_pid=$!

sleep 2

echo "Recording mapping bag to: $bag_dir"
echo "Move the robot manually. Press Ctrl+C once to stop."
exec "$project_root/scripts/go2_uslam_record_map.sh" "$bag_dir"
