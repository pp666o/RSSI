#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_offline_rtabmap_from_bag.sh <bag_dir>

Run offline RTAB-Map mapping on the control PC from a recorded Go2 rosbag.
The script starts RTAB-Map + RViz, then plays the bag with /clock.

Inputs expected in the bag:
  /utlidar/cloud_deskewed
  /utlidar/robot_odom
  /utlidar/imu
  /tf
  /tf_static

Environment:
  RUN_DIR          Output directory. Defaults to ~/go2_rssi_runs/offline_map_<timestamp>
  CLOUD_TOPIC      Defaults to /utlidar/cloud_deskewed.
  ODOM_TOPIC       Defaults to /utlidar/robot_odom.
  IMU_TOPIC        Defaults to /utlidar/imu.
  START_RVIZ       true or false. Defaults to true.
  PLAY_RATE        rosbag play rate. Defaults to 0.5.
  LOOP             true or false. Defaults to false.
  RTABMAP_ARGS     Defaults to --delete_db_on_start.

Output:
  $RUN_DIR/go2_offline_mapping.db
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

bag_dir="$1"
if [[ ! -d "$bag_dir" ]]; then
  echo "Bag directory does not exist: $bag_dir" >&2
  exit 2
fi

project_root="/home/luping/桌面/RSSI/RSSI/code"
workspace="$project_root/ros2_ws"
timestamp="$(date +%Y%m%d_%H%M%S)"
run_dir="${RUN_DIR:-/home/luping/go2_rssi_runs/offline_map_${timestamp}}"
cloud_topic="${CLOUD_TOPIC:-/utlidar/cloud_deskewed}"
odom_topic="${ODOM_TOPIC:-/utlidar/robot_odom}"
imu_topic="${IMU_TOPIC:-/utlidar/imu}"
start_rviz="${START_RVIZ:-true}"
play_rate="${PLAY_RATE:-0.5}"
loop="${LOOP:-false}"
rtabmap_args="${RTABMAP_ARGS:---delete_db_on_start}"

mkdir -p "$run_dir"

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

db_path="$run_dir/go2_offline_mapping.db"
launch_log="$run_dir/rtabmap_launch.log"
play_log="$run_dir/bag_play.log"

echo "Offline RTAB-Map mapping:"
echo "  bag:         $bag_dir"
echo "  output dir:  $run_dir"
echo "  database:    $db_path"
echo "  cloud_topic: $cloud_topic"
echo "  odom_topic:  $odom_topic"
echo "  imu_topic:   $imu_topic"
echo "  START_RVIZ:  $start_rviz"
echo "  PLAY_RATE:   $play_rate"
echo

cleanup() {
  set +e
  if [[ -n "${play_pid:-}" ]] && kill -0 "$play_pid" >/dev/null 2>&1; then
    kill -INT "$play_pid" >/dev/null 2>&1
    wait "$play_pid" >/dev/null 2>&1
  fi
  if [[ -n "${launch_pid:-}" ]] && kill -0 "$launch_pid" >/dev/null 2>&1; then
    kill -INT "$launch_pid" >/dev/null 2>&1
    wait "$launch_pid" >/dev/null 2>&1
  fi
}
trap cleanup EXIT INT TERM

ros2 launch go2_nav2_bridge go2_official_rtabmap_mapping.launch.py \
  start_rtabmap:=true \
  start_rviz:="$start_rviz" \
  cloud_topic:="$cloud_topic" \
  odom_topic:="$odom_topic" \
  imu_topic:="$imu_topic" \
  use_sim_time:=true \
  rtabmap_database_path:="$db_path" \
  rtabmap_args:="$rtabmap_args" \
  >"$launch_log" 2>&1 &
launch_pid=$!

echo "Started RTAB-Map launch pid=$launch_pid"
sleep 5

play_cmd=(ros2 bag play "$bag_dir" --clock --rate "$play_rate")
if [[ "$loop" == "true" ]]; then
  play_cmd+=(--loop)
fi

echo "Playing bag..."
"${play_cmd[@]}" >"$play_log" 2>&1 &
play_pid=$!

wait "$play_pid"
play_pid=""

echo
echo "Bag playback finished. Stopping RTAB-Map..."
sleep 3
cleanup
trap - EXIT INT TERM

echo "Done."
echo "Database: $db_path"
echo "Logs:"
echo "  $launch_log"
echo "  $play_log"
