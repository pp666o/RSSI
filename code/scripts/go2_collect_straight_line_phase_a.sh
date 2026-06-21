#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/go2_collect_straight_line_phase_a.sh <networkInterface> [distance_m] [speed_mps] [window_sec] [step_sec]

Example:
  RUN_DIR="$HOME/go2_rssi_runs/straight_001" scripts/go2_collect_straight_line_phase_a.sh eth0 70 0.20

Environment:
  RUN_DIR                 Output directory. Defaults to ~/go2_rssi_runs/straight_<timestamp>.
  BT_ATTACH               Attach /dev/ttyACM* with btattach before scanning. Defaults to auto.
                          Set to 0/off/no if HCI is already attached.
  BT_TTY                  UART Bluetooth device. Defaults to the largest /dev/ttyACM*.
  BT_BAUD                 UART baudrate for btattach. Defaults to 1000000.
  HCI_DEV                 Bluetooth controller for RSSI. If unset, auto-detected after btattach.
  BTATTACH_CLEANUP        Kill btattach started by this script on exit. Defaults to 0.
  WARMUP_SEC              Balance-stand time before moving. Defaults to 2.0.
  CONTROL_HZ              Go2 velocity command rate. Defaults to 20.
  MAX_RUNTIME_SEC         Safety timeout for odom-controlled movement.
                          Defaults to max(30, 4 * distance_m / speed_mps).
  RSSI_EXTRA_SEC          Extra RSSI/IMU collection budget if movement reaches
                          MAX_RUNTIME_SEC. Background collection is stopped as
                          soon as the Go2 runner exits. Defaults to 5.
  IMU_ACC_TO_G_SCALE      Optional override for Go2 accelerometer scale.
  IMU_GYRO_TO_DPS_SCALE   Optional override for Go2 gyroscope scale.
  IMU_ANGLE_TO_DEG_SCALE  Optional override for Go2 rpy angle scale.
  LOG_SPORT_STATE         Pass 1 to log rt/sportmodestate in sport_state_straight.csv.
                          Defaults to 1 and is required for odom-controlled distance stopping.
  USE_OBSTACLE_AVOID      Use Unitree official obstacles_avoid client for motion.
                          Defaults to 0 because this interface can be too
                          conservative in corridors. Set to 1 only for
                          controlled comparison tests.
  MOVE_MODE               Official motion command mode. Defaults to velocity.
                          increment: ObstaclesAvoid MoveToIncrementPosition.
                          velocity: ObstaclesAvoid/Sport Move velocity command.
  INCREMENT_STEP_M        Forward step for MOVE_MODE=increment only. Defaults
                          to 0, meaning full remaining target distance without
                          fixed 0.5 m segmentation.
  ENABLE_POINT_CLOUD      Deprecated in this runner. Defaults to 0.
                          Use scripts/go2_collect_straight_line_nav2.sh for
                          point-cloud costmap and collision-monitor avoidance.
  POINT_CLOUD_TOPIC       PointCloud2 DDS topic. Defaults to rt/utlidar/cloud.
  OBSTACLE_STOP_M         Stop/hold if local obstacle distance is this close.
                          Defaults to 0.35.
  OBSTACLE_SLOW_M         Start reducing speed below this distance. Defaults
                          to 0.90.
  POINT_CLOUD_FORWARD_M   Forward depth of the point-cloud safety zone.
                          Defaults to 1.50.
  POINT_CLOUD_HALF_WIDTH_M
                          Half width of the point-cloud safety zone. Defaults
                          to 0.60.
  POINT_CLOUD_MIN_X_M     Ignore near-field points before this forward distance.
                          Defaults to 0.25.
  ENABLE_LOCAL_AVOIDANCE  Deprecated in this runner. Defaults to 0.
                          Local obstacle handling is now delegated to Nav2.
  AVOID_YAW_RATE          Yaw rate while avoiding. Defaults to 0.35 rad/s.
  AVOID_LATERAL_SPEED     Lateral speed while avoiding. Defaults to 0.08 m/s.
  POINT_CLOUD_CENTER_HALF_WIDTH_M
                          Center lane half width for front blockage.
                          Defaults to 0.25.
  POINT_CLOUD_MIN_CENTER_POINTS
                          Minimum center points before treating a cluster as
                          an obstacle. Defaults to 4.
  AVOID_DIRECTION_HOLD_SEC
                          Hold chosen left/right avoidance direction to prevent
                          command flip-flop. Defaults to 0.8.
  WALL_AVOID_M            Start steering away when a side wall is this close.
                          Defaults to 0.10.
  WALL_TARGET_M           Preferred side-wall clearance. Defaults to 0.20.
  WALL_YAW_GAIN           Yaw correction gain for side-wall repulsion.
                          Defaults to 1.2.
  WALL_LATERAL_GAIN       Lateral correction gain for side-wall repulsion.
                          Defaults to 0.35.
  POINT_CLOUD_Y_SIGN      Set to -1 if left/right avoidance is reversed on
                          the robot. Defaults to 1.
  AVOID_SIDE_OFFSET_M     Lateral center of left/right bypass corridors.
                          Defaults to 0.25.
  AVOID_CORRIDOR_HALF_WIDTH_M
                          Half width of each bypass corridor. Defaults to 0.18.
  AVOID_OPEN_M            Bypass corridor must be clear this far ahead.
                          Defaults to 0.85.
  BLOCKED_HOLD_TIMEOUT_SEC
                          Hold this long if no local route is open. Defaults
                          to 2.0.
  ENABLE_VIDEO            Save front camera samples for diagnostics. Defaults
                          to 1.
  VIDEO_INTERVAL_SEC      Seconds between front camera samples. Defaults to 3.0.
  VIDEO_DIR               Front image output directory. Defaults to
                          "$RUN_DIR/front_images".
USAGE
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 1
fi

network_interface="$1"
distance_m="${2:-70}"
speed_mps="${3:-0.20}"
window_sec="${4:-1.0}"
step_sec="${5:-0.5}"
warmup_sec="${WARMUP_SEC:-2.0}"
control_hz="${CONTROL_HZ:-20}"
extra_sec="${RSSI_EXTRA_SEC:-5}"
log_sport_state="${LOG_SPORT_STATE:-1}"
use_obstacle_avoid="${USE_OBSTACLE_AVOID:-0}"
move_mode="${MOVE_MODE:-velocity}"
increment_step_m="${INCREMENT_STEP_M:-0.0}"
enable_point_cloud="${ENABLE_POINT_CLOUD:-0}"
point_cloud_topic="${POINT_CLOUD_TOPIC:-rt/utlidar/cloud}"
obstacle_stop_m="${OBSTACLE_STOP_M:-0.35}"
obstacle_slow_m="${OBSTACLE_SLOW_M:-0.90}"
point_cloud_forward_m="${POINT_CLOUD_FORWARD_M:-1.50}"
point_cloud_half_width_m="${POINT_CLOUD_HALF_WIDTH_M:-0.60}"
point_cloud_min_x_m="${POINT_CLOUD_MIN_X_M:-0.25}"
enable_local_avoidance="${ENABLE_LOCAL_AVOIDANCE:-0}"
avoid_yaw_rate="${AVOID_YAW_RATE:-0.35}"
avoid_lateral_speed="${AVOID_LATERAL_SPEED:-0.08}"
point_cloud_center_half_width_m="${POINT_CLOUD_CENTER_HALF_WIDTH_M:-0.25}"
point_cloud_min_center_points="${POINT_CLOUD_MIN_CENTER_POINTS:-4}"
avoid_direction_hold_sec="${AVOID_DIRECTION_HOLD_SEC:-0.8}"
wall_avoid_m="${WALL_AVOID_M:-0.10}"
wall_target_m="${WALL_TARGET_M:-0.20}"
wall_yaw_gain="${WALL_YAW_GAIN:-1.2}"
wall_lateral_gain="${WALL_LATERAL_GAIN:-0.35}"
point_cloud_y_sign="${POINT_CLOUD_Y_SIGN:-1}"
avoid_side_offset_m="${AVOID_SIDE_OFFSET_M:-0.25}"
avoid_corridor_half_width_m="${AVOID_CORRIDOR_HALF_WIDTH_M:-0.18}"
avoid_open_m="${AVOID_OPEN_M:-0.85}"
blocked_hold_timeout_sec="${BLOCKED_HOLD_TIMEOUT_SEC:-2.0}"
enable_video="${ENABLE_VIDEO:-1}"
video_interval_sec="${VIDEO_INTERVAL_SEC:-3.0}"

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${RUN_DIR:-$HOME/go2_rssi_runs/straight_$(date +%Y%m%d_%H%M%S)}"
video_dir="${VIDEO_DIR:-$run_dir/front_images}"

sdk_arch="$(uname -m)"
sdk_lib_dir="$project_root/unitree_sdk2/thirdparty/lib/$sdk_arch"
case ":${LD_LIBRARY_PATH:-}:" in
  *":$sdk_lib_dir:"*) ;;
  *) export LD_LIBRARY_PATH="$sdk_lib_dir:${LD_LIBRARY_PATH:-}" ;;
esac

imu_logger="$project_root/unitree_sdk2/build_go2/bin/go2_imu_logger"
line_runner="$project_root/unitree_sdk2/build_go2/bin/go2_straight_line_runner"
scan_script="$project_root/bt_migrate_pack/start_coded_scan.sh"
attach_script="$project_root/bt_migrate_pack/go2_attach_ttyacm.sh"

if [[ -n "${RSSI_BINARY:-}" ]]; then
  rssi_binary="$RSSI_BINARY"
else
  rssi_binary=""
  for candidate in \
    "$project_root/build_rssi/test_rssi" \
    "$project_root/test_rssi" \
    "$project_root/bt_migrate_pack/test_rssi"; do
    if [[ -x "$candidate" ]]; then
      rssi_binary="$candidate"
      break
    fi
  done
fi

if [[ ! -x "$imu_logger" || ! -x "$line_runner" ]]; then
  echo "Missing Go2 binaries." >&2
  echo "Build them with:" >&2
  echo "  cmake -S unitree_sdk2 -B unitree_sdk2/build_go2" >&2
  echo "  cmake --build unitree_sdk2/build_go2 --target go2_imu_logger go2_straight_line_runner -j\"\$(nproc)\"" >&2
  exit 1
fi

if [[ ! -x "$rssi_binary" ]]; then
  echo "Missing executable RSSI binary." >&2
  echo "Build it with:" >&2
  echo "  scripts/build_rssi_logger.sh" >&2
  exit 1
fi

if ! awk "BEGIN {exit !($distance_m > 0 && $speed_mps > 0 && $window_sec > 0 && $step_sec > 0)}"; then
  echo "distance_m, speed_mps, window_sec, and step_sec must be positive." >&2
  exit 1
fi

distance_m="$(awk -v x="$distance_m" 'BEGIN {if (x < 0.1) x = 0.1; if (x > 100.0) x = 100.0; printf "%.6f", x}')"
speed_mps="$(awk -v x="$speed_mps" 'BEGIN {if (x < 0.05) x = 0.05; if (x > 0.35) x = 0.35; printf "%.6f", x}')"
warmup_sec="$(awk -v x="$warmup_sec" 'BEGIN {if (x < 0.0) x = 0.0; if (x > 10.0) x = 10.0; printf "%.6f", x}')"
control_hz="$(awk -v x="$control_hz" 'BEGIN {if (x < 5.0) x = 5.0; if (x > 50.0) x = 50.0; printf "%.6f", x}')"
max_runtime_sec="${MAX_RUNTIME_SEC:-$(awk -v d="$distance_m" -v v="$speed_mps" 'BEGIN {x = 4 * d / v; if (x < 30) x = 30; printf "%.0f", x}')}"
max_runtime_sec="$(awk -v x="$max_runtime_sec" 'BEGIN {if (x < 1.0) x = 1.0; if (x > 3600.0) x = 3600.0; printf "%.6f", x}')"
duration_sec="$(awk -v m="$max_runtime_sec" -v w="$warmup_sec" -v e="$extra_sec" 'BEGIN {printf "%.0f", m + w + e}')"

mkdir -p "$run_dir"
echo "Output directory: $run_dir"
echo "Odom-controlled straight-line run: distance=${distance_m}m speed=${speed_mps}m/s max_runtime=${max_runtime_sec}s collection_duration=${duration_sec}s"
echo "LOG_SPORT_STATE=$log_sport_state"
echo "USE_OBSTACLE_AVOID=$use_obstacle_avoid"
echo "MOVE_MODE=$move_mode"
echo "INCREMENT_STEP_M=$increment_step_m"
echo "ENABLE_POINT_CLOUD=$enable_point_cloud"
echo "POINT_CLOUD_TOPIC=$point_cloud_topic"
echo "OBSTACLE_STOP_M=$obstacle_stop_m"
echo "OBSTACLE_SLOW_M=$obstacle_slow_m"
echo "POINT_CLOUD_FORWARD_M=$point_cloud_forward_m"
echo "POINT_CLOUD_HALF_WIDTH_M=$point_cloud_half_width_m"
echo "POINT_CLOUD_MIN_X_M=$point_cloud_min_x_m"
echo "ENABLE_LOCAL_AVOIDANCE=$enable_local_avoidance"
echo "AVOID_YAW_RATE=$avoid_yaw_rate"
echo "AVOID_LATERAL_SPEED=$avoid_lateral_speed"
echo "POINT_CLOUD_CENTER_HALF_WIDTH_M=$point_cloud_center_half_width_m"
echo "POINT_CLOUD_MIN_CENTER_POINTS=$point_cloud_min_center_points"
echo "AVOID_DIRECTION_HOLD_SEC=$avoid_direction_hold_sec"
echo "WALL_AVOID_M=$wall_avoid_m"
echo "WALL_TARGET_M=$wall_target_m"
echo "WALL_YAW_GAIN=$wall_yaw_gain"
echo "WALL_LATERAL_GAIN=$wall_lateral_gain"
echo "POINT_CLOUD_Y_SIGN=$point_cloud_y_sign"
echo "AVOID_SIDE_OFFSET_M=$avoid_side_offset_m"
echo "AVOID_CORRIDOR_HALF_WIDTH_M=$avoid_corridor_half_width_m"
echo "AVOID_OPEN_M=$avoid_open_m"
echo "BLOCKED_HOLD_TIMEOUT_SEC=$blocked_hold_timeout_sec"
echo "ENABLE_VIDEO=$enable_video"
echo "VIDEO_INTERVAL_SEC=$video_interval_sec"
echo "VIDEO_DIR=$video_dir"
echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}"
if [[ "$log_sport_state" != "1" ]]; then
  echo "LOG_SPORT_STATE must be 1 for odom-controlled distance stopping." >&2
  exit 1
fi

case "$move_mode" in
  increment|Increment|INCREMENT|1)
    motion_mode_arg="1"
    if [[ "$use_obstacle_avoid" != "1" ]]; then
      echo "MOVE_MODE=increment requires USE_OBSTACLE_AVOID=1." >&2
      exit 1
    fi
    ;;
  velocity|Velocity|VELOCITY|0)
    motion_mode_arg="0"
    ;;
  *)
    echo "MOVE_MODE must be increment or velocity." >&2
    exit 1
    ;;
esac

imu_args=("$network_interface" "$run_dir/imu_stream.csv" "$duration_sec")
if [[ -n "${IMU_ACC_TO_G_SCALE:-}" || -n "${IMU_GYRO_TO_DPS_SCALE:-}" || -n "${IMU_ANGLE_TO_DEG_SCALE:-}" ]]; then
  imu_args+=(
    "${IMU_ACC_TO_G_SCALE:-0.1019716213}"
    "${IMU_GYRO_TO_DPS_SCALE:-57.2957795131}"
    "${IMU_ANGLE_TO_DEG_SCALE:-57.2957795131}"
  )
fi

imu_pid=""
rssi_pid=""
runner_status=0
btattach_pid=""
btattach_started="0"

cleanup() {
  stop_background_collectors
  if [[ "${BTATTACH_CLEANUP:-0}" == "1" && "$btattach_started" == "1" && -n "$btattach_pid" ]]; then
    sudo kill "$btattach_pid" 2>/dev/null || true
    wait "$btattach_pid" 2>/dev/null || true
  fi
  sudo chown -R "$(id -u):$(id -g)" "$run_dir" 2>/dev/null || true
}

stop_background_collectors() {
  if [[ -n "$rssi_pid" ]] && kill -0 "$rssi_pid" 2>/dev/null; then
    sudo kill "$rssi_pid" 2>/dev/null || true
    wait "$rssi_pid" 2>/dev/null || true
  fi
  rssi_pid=""
  if [[ -n "$imu_pid" ]] && kill -0 "$imu_pid" 2>/dev/null; then
    kill "$imu_pid" 2>/dev/null || true
    wait "$imu_pid" 2>/dev/null || true
  fi
  imu_pid=""
}
trap cleanup EXIT INT TERM

bt_attach_mode="${BT_ATTACH:-auto}"
case "$bt_attach_mode" in
  0|false|False|FALSE|no|No|NO|off|Off|OFF)
    echo "Skipping ttyACM btattach because BT_ATTACH=$bt_attach_mode"
    ;;
  *)
    if [[ -f "$attach_script" ]]; then
      echo "Auto-attaching Bluetooth controller with $attach_script"
      attach_output="$(bash "$attach_script")"
      echo "$attach_output"

      parsed_hci="$(printf '%s\n' "$attach_output" | awk -F= '$1 == "HCI_DEV" {print $2}' | tail -n 1)"
      parsed_pid="$(printf '%s\n' "$attach_output" | awk -F= '$1 == "BT_ATTACH_PID" {print $2}' | tail -n 1)"
      parsed_started="$(printf '%s\n' "$attach_output" | awk -F= '$1 == "BT_ATTACH_STARTED" {print $2}' | tail -n 1)"

      if [[ -n "$parsed_hci" ]]; then
        export HCI_DEV="$parsed_hci"
      fi
      btattach_pid="$parsed_pid"
      btattach_started="${parsed_started:-0}"
    elif [[ "$bt_attach_mode" == "required" ]]; then
      echo "Missing required Bluetooth attach script: $attach_script" >&2
      exit 1
    else
      echo "No ttyACM attach script found; continuing with HCI_DEV=${HCI_DEV:-hci0}" >&2
    fi
    ;;
esac

sudo bash "$scan_script" "${HCI_DEV:-hci0}"

"$imu_logger" "${imu_args[@]}" &
imu_pid=$!

(
  cd "$run_dir"
  exec sudo "$rssi_binary" "$duration_sec" rssi_realtime_windows.csv \
    "$window_sec" "$step_sec" 1.0 0.0 none
) &
rssi_pid=$!

sleep 2
"$line_runner" "$network_interface" "$run_dir/sport_state_straight.csv" \
  "$distance_m" "$speed_mps" "$warmup_sec" "$control_hz" "$log_sport_state" "$max_runtime_sec" \
  "$use_obstacle_avoid" "$motion_mode_arg" "$increment_step_m" \
  "$enable_point_cloud" "$point_cloud_topic" "$obstacle_stop_m" "$obstacle_slow_m" \
  "$point_cloud_forward_m" "$point_cloud_half_width_m" \
  "$enable_video" "$video_interval_sec" "$video_dir" "$point_cloud_min_x_m" \
  "$enable_local_avoidance" "$avoid_yaw_rate" "$avoid_lateral_speed" \
  "$point_cloud_center_half_width_m" "$point_cloud_min_center_points" \
  "$avoid_direction_hold_sec" "$wall_avoid_m" "$wall_target_m" \
  "$wall_yaw_gain" "$wall_lateral_gain" \
  "$point_cloud_y_sign" "$avoid_side_offset_m" "$avoid_corridor_half_width_m" \
  "$avoid_open_m" "$blocked_hold_timeout_sec" || runner_status=$?

if [[ "$runner_status" -ne 0 ]]; then
  echo "Go2 straight-line runner exited with status $runner_status" >&2
fi

echo "Go2 runner finished; stopping RSSI/IMU background collectors."
stop_background_collectors

cat > "$run_dir/run_metadata.txt" <<EOF
network_interface=$network_interface
distance_m=$distance_m
speed_mps=$speed_mps
window_sec=$window_sec
step_sec=$step_sec
warmup_sec=$warmup_sec
control_hz=$control_hz
log_sport_state=$log_sport_state
use_obstacle_avoid=$use_obstacle_avoid
move_mode=$move_mode
motion_mode_arg=$motion_mode_arg
increment_step_m=$increment_step_m
enable_point_cloud=$enable_point_cloud
point_cloud_topic=$point_cloud_topic
obstacle_stop_m=$obstacle_stop_m
obstacle_slow_m=$obstacle_slow_m
point_cloud_forward_m=$point_cloud_forward_m
point_cloud_half_width_m=$point_cloud_half_width_m
point_cloud_min_x_m=$point_cloud_min_x_m
enable_local_avoidance=$enable_local_avoidance
avoid_yaw_rate=$avoid_yaw_rate
avoid_lateral_speed=$avoid_lateral_speed
point_cloud_center_half_width_m=$point_cloud_center_half_width_m
point_cloud_min_center_points=$point_cloud_min_center_points
avoid_direction_hold_sec=$avoid_direction_hold_sec
wall_avoid_m=$wall_avoid_m
wall_target_m=$wall_target_m
wall_yaw_gain=$wall_yaw_gain
wall_lateral_gain=$wall_lateral_gain
point_cloud_y_sign=$point_cloud_y_sign
avoid_side_offset_m=$avoid_side_offset_m
avoid_corridor_half_width_m=$avoid_corridor_half_width_m
avoid_open_m=$avoid_open_m
blocked_hold_timeout_sec=$blocked_hold_timeout_sec
enable_video=$enable_video
video_interval_sec=$video_interval_sec
video_dir=$video_dir
max_runtime_sec=$max_runtime_sec
rssi_duration_sec=$duration_sec
hci_dev=${HCI_DEV:-hci0}
rssi_binary=$rssi_binary
sdk_lib_dir=$sdk_lib_dir
ld_library_path=${LD_LIBRARY_PATH:-}
EOF

echo "Output files:"
ls -lh "$run_dir" || true
echo "Done. Output directory: $run_dir"
exit "$runner_status"
