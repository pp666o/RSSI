#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_official_rtabmap_mapping.sh

Starts RTAB-Map mapping on the control PC using official Go2 ROS topics:
  - scan_cloud: /utlidar/cloud_deskewed
  - odom:       /utlidar/robot_odom
  - imu:        /utlidar/imu
  - RViz view:  /rtabmap/cloud_map and /utlidar/voxel_map

This script does not start any walking controller and does not send /cmd_vel.

Environment:
  RUN_DIR       Output directory. Defaults to ~/go2_rssi_runs/rtabmap_<timestamp>
  START_RVIZ    true or false. Defaults to true.
  CLOUD_TOPIC   Defaults to /utlidar/cloud_deskewed.
  ODOM_TOPIC    Defaults to /utlidar/robot_odom.
  IMU_TOPIC     Defaults to /utlidar/imu.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

project_root="/home/luping/桌面/RSSI/RSSI/code"
workspace="$project_root/ros2_ws"
timestamp="$(date +%Y%m%d_%H%M%S)"
run_dir="${RUN_DIR:-/home/luping/go2_rssi_runs/rtabmap_${timestamp}}"
start_rviz="${START_RVIZ:-true}"
cloud_topic="${CLOUD_TOPIC:-/utlidar/cloud_deskewed}"
odom_topic="${ODOM_TOPIC:-/utlidar/robot_odom}"
imu_topic="${IMU_TOPIC:-/utlidar/imu}"

mkdir -p "$run_dir"

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

echo "Run directory: $run_dir"
echo "cloud_topic=$cloud_topic"
echo "odom_topic=$odom_topic"
echo "imu_topic=$imu_topic"
echo "START_RVIZ=$start_rviz"

exec ros2 launch go2_nav2_bridge go2_official_rtabmap_mapping.launch.py \
  start_rtabmap:=true \
  start_rviz:="$start_rviz" \
  cloud_topic:="$cloud_topic" \
  odom_topic:="$odom_topic" \
  imu_topic:="$imu_topic" \
  rtabmap_database_path:="$run_dir/go2_official_mapping.db"
