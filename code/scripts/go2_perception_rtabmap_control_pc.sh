#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_perception_rtabmap_control_pc.sh [network_interface]

Starts the control-PC Go2 perception stack:
  - read-only Unitree DDS -> ROS 2 bridge
  - RViz live point cloud view
  - RTAB-Map when ros-humble-rtabmap-ros is installed

This script does not start Go2 walking, does not call BalanceStand, and does not
subscribe to /cmd_vel. The bridge is launched with motion_client=none.

Environment:
  RUN_DIR        Output directory. Defaults to ~/go2_rssi_runs/rtabmap_<timestamp>
  START_RTABMAP  true, false, or auto. Defaults to auto.
  START_RVIZ     true or false. Defaults to true.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

network_interface="${1:-enp5s0}"
project_root="/home/luping/桌面/RSSI/RSSI/code"
workspace="$project_root/ros2_ws"
timestamp="$(date +%Y%m%d_%H%M%S)"
run_dir="${RUN_DIR:-/home/luping/go2_rssi_runs/rtabmap_${timestamp}}"
start_rtabmap="${START_RTABMAP:-auto}"
start_rviz="${START_RVIZ:-true}"

mkdir -p "$run_dir"

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

if [[ "$start_rtabmap" == "auto" ]]; then
  if ros2 pkg prefix rtabmap_slam >/dev/null 2>&1; then
    start_rtabmap=true
  else
    start_rtabmap=false
    echo "RTAB-Map is not installed; starting bridge + RViz only." >&2
    echo "Install it with: sudo apt install ros-humble-rtabmap-ros ros-humble-rtabmap-viz" >&2
  fi
fi

echo "Run directory: $run_dir"
echo "Network interface: $network_interface"
echo "START_RTABMAP=$start_rtabmap START_RVIZ=$start_rviz"

exec ros2 launch go2_nav2_bridge go2_perception_rtabmap.launch.py \
  network_interface:="$network_interface" \
  start_bridge:=true \
  start_rtabmap:="$start_rtabmap" \
  start_rviz:="$start_rviz" \
  rtabmap_database_path:="$run_dir/go2_rtabmap.db"
