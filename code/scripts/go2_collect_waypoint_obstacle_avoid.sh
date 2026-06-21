#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/go2_collect_waypoint_obstacle_avoid.sh <networkInterface> [relative_waypoints] [collection_duration_sec] [window_sec] [step_sec]

Examples:
  scripts/go2_collect_waypoint_obstacle_avoid.sh eth0 "[3.0, 0.0, 0.0]"
  RUN_DIR="$HOME/go2_rssi_runs/waypoint_10m_001" scripts/go2_collect_waypoint_obstacle_avoid.sh eth0 "[10.0, 0.0, 0.0]"
  scripts/go2_collect_waypoint_obstacle_avoid.sh eth0 "[10.0, 0.0, 0.0, 10.0, 5.0, 1.5708]"

Environment:
  RUN_DIR                    Output directory. Defaults to ~/go2_rssi_runs/waypoint_<timestamp>.
  ROS_SETUP                  ROS 2 setup file. Defaults to /opt/ros/foxy, then humble/jazzy/kilted.
  GO2_WS_SETUP               Overlay setup file. Defaults to ros2_ws/install/setup.bash.
  WAYPOINT_TIMEOUT_SEC       Timeout for the waypoint launch. Defaults to max(30, 4 * path_length / EXPECTED_SPEED_MPS).
  EXPECTED_SPEED_MPS         Used only for timeout estimation. Defaults to 0.30.
  RSSI_EXTRA_SEC             Added to waypoint timeout when collection_duration_sec is omitted. Defaults to 5.
  RSSI_WARMUP_SEC            Delay between starting RSSI/IMU and starting motion. Defaults to 3.
  MARKER_STEP_M              Marker spacing passed to RSSI collector. Defaults to 0.5.
  START_S_M                  Start route coordinate passed to RSSI collector. Defaults to 0.0.
  BT_ATTACH                  Passed to go2_collect_rssi_imu.sh. Defaults to auto.
  SUDO_NONINTERACTIVE        Passed to go2_collect_rssi_imu.sh. Defaults to 1.
  ALLOW_EXISTING_GO2_CONTROL Set to 1 to skip existing control-process preflight. Defaults to 0.
USAGE
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 1
fi

network_interface="$1"
relative_waypoints="${2:-[3.0, 0.0, 0.0]}"
collection_duration_sec="${3:-}"
window_sec="${4:-1.0}"
step_sec="${5:-0.5}"
marker_step_m="${MARKER_STEP_M:-0.5}"
start_s_m="${START_S_M:-0.0}"
expected_speed_mps="${EXPECTED_SPEED_MPS:-0.30}"
extra_sec="${RSSI_EXTRA_SEC:-5}"
warmup_sec="${RSSI_WARMUP_SEC:-3}"

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${RUN_DIR:-$HOME/go2_rssi_runs/waypoint_$(date +%Y%m%d_%H%M%S)}"
rssi_log="$run_dir/rssi_imu.log"
waypoint_log="$run_dir/waypoint_launch.log"
summary_file="$run_dir/run_summary.txt"

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

path_length_m="$(python3 - "$relative_waypoints" <<'PY'
import ast
import math
import sys

raw = sys.argv[1]
try:
    values = ast.literal_eval(raw)
except Exception as exc:
    raise SystemExit(f"Invalid relative_waypoints: {raw!r}: {exc}")

if not isinstance(values, (list, tuple)) or len(values) == 0 or len(values) % 3 != 0:
    raise SystemExit("relative_waypoints must be [x1, y1, yaw1, x2, y2, yaw2, ...]")

points = [(0.0, 0.0)]
for i in range(0, len(values), 3):
    points.append((float(values[i]), float(values[i + 1])))

length = 0.0
for a, b in zip(points, points[1:]):
    length += math.hypot(b[0] - a[0], b[1] - a[1])

print(f"{length:.6f}")
PY
)"

if ! awk "BEGIN {exit !($path_length_m > 0 && $expected_speed_mps > 0 && $window_sec > 0 && $step_sec > 0)}"; then
  echo "Invalid path length, expected speed, window_sec, or step_sec." >&2
  exit 1
fi

waypoint_timeout_sec="${WAYPOINT_TIMEOUT_SEC:-$(awk -v d="$path_length_m" -v v="$expected_speed_mps" 'BEGIN {x = 4 * d / v; if (x < 30) x = 30; printf "%.0f", x}')}"
waypoint_timeout_sec="$(awk -v x="$waypoint_timeout_sec" 'BEGIN {if (x < 5) x = 5; if (x > 7200) x = 7200; printf "%.0f", x}')"

if [[ -z "$collection_duration_sec" ]]; then
  collection_duration_sec="$(awk -v t="$waypoint_timeout_sec" -v e="$extra_sec" 'BEGIN {printf "%.0f", t + e}')"
fi
collection_duration_sec="$(awk -v x="$collection_duration_sec" 'BEGIN {if (x < 1) x = 1; if (x > 7200) x = 7200; printf "%.0f", x}')"

if [[ "${ALLOW_EXISTING_GO2_CONTROL:-0}" != "1" ]]; then
  existing="$(
    pgrep -af \
      'ros2 launch go2_nav2_bridge go2_waypoint_obstacle_avoid.launch.py|go2_nav2_bridge/lib/go2_nav2_bridge/go2_nav2_bridge|go2_nav2_bridge/lib/go2_nav2_bridge/go2_waypoint_follower' \
      || true
  )"
  if [[ -n "$existing" ]]; then
    echo "Existing Go2 waypoint control processes are still running:" >&2
    printf '%s\n' "$existing" >&2
    echo "Stop them first, or set ALLOW_EXISTING_GO2_CONTROL=1 if this is intentional." >&2
    exit 1
  fi
fi

mkdir -p "$run_dir"

sdk_arch="$(uname -m)"
for lib_dir in \
  "$project_root/unitree_sdk2/lib/$sdk_arch" \
  "$project_root/unitree_sdk2/thirdparty/lib/$sdk_arch"; do
  if [[ -d "$lib_dir" ]]; then
    case ":${LD_LIBRARY_PATH:-}:" in
      *":$lib_dir:"*) ;;
      *) export LD_LIBRARY_PATH="$lib_dir:${LD_LIBRARY_PATH:-}" ;;
    esac
  fi
done

if [[ -n "${ROS_SETUP:-}" ]]; then
  source_setup "$ROS_SETUP"
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  source_setup /opt/ros/foxy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source_setup /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_setup /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/kilted/setup.bash ]]; then
  source_setup /opt/ros/kilted/setup.bash
fi

default_overlay="$project_root/ros2_ws/install/setup.bash"
if [[ -n "${GO2_WS_SETUP:-}" ]]; then
  source_setup "$GO2_WS_SETUP"
elif [[ -f "$default_overlay" ]]; then
  source_setup "$default_overlay"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. Source ROS 2 first or set ROS_SETUP." >&2
  exit 1
fi

if ! ros2 pkg prefix go2_nav2_bridge >/dev/null 2>&1; then
  echo "go2_nav2_bridge is not built or not sourced." >&2
  exit 1
fi

cat > "$summary_file" <<EOF
run_dir=$run_dir
network_interface=$network_interface
relative_waypoints=$relative_waypoints
path_length_m=$path_length_m
waypoint_timeout_sec=$waypoint_timeout_sec
collection_duration_sec=$collection_duration_sec
window_sec=$window_sec
step_sec=$step_sec
marker_step_m=$marker_step_m
start_s_m=$start_s_m
EOF

echo "Output directory: $run_dir"
echo "relative_waypoints=$relative_waypoints"
echo "path_length_m=$path_length_m"
echo "RSSI/IMU collection duration=${collection_duration_sec}s"
echo "waypoint timeout=${waypoint_timeout_sec}s"

rssi_pid=""
launch_pid=""
cleanup_started=0

stop_rssi_collection() {
  if [[ -z "$rssi_pid" ]]; then
    return
  fi
  if ! kill -0 "$rssi_pid" 2>/dev/null; then
    wait "$rssi_pid" 2>/dev/null || true
    rssi_pid=""
    return
  fi

  echo "Stopping RSSI/IMU collection..."
  kill -INT "-$rssi_pid" 2>/dev/null || kill -INT "$rssi_pid" 2>/dev/null || true
  for _ in $(seq 1 20); do
    if ! kill -0 "$rssi_pid" 2>/dev/null; then
      wait "$rssi_pid" 2>/dev/null || true
      rssi_pid=""
      return
    fi
    sleep 0.2
  done
  kill -TERM "-$rssi_pid" 2>/dev/null || kill -TERM "$rssi_pid" 2>/dev/null || true
  wait "$rssi_pid" 2>/dev/null || true
  rssi_pid=""
}

cleanup() {
  if [[ "$cleanup_started" == "1" ]]; then
    return
  fi
  cleanup_started=1
  stop_waypoint_launch
  stop_rssi_collection
}
on_exit() {
  cleanup
}

on_signal() {
  cleanup
  exit 130
}

trap on_exit EXIT
trap on_signal INT TERM

publish_zero_cmd_vel() {
  if ! command -v ros2 >/dev/null 2>&1; then
    return
  fi
  for _ in $(seq 1 20); do
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
      >/dev/null 2>&1 || true
    sleep 0.1
  done
}

stop_waypoint_launch() {
  echo "Sending zero cmd_vel before stopping waypoint launch..."
  publish_zero_cmd_vel
  if [[ -z "$launch_pid" ]]; then
    return
  fi
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid" 2>/dev/null || true
    launch_pid=""
    return
  fi
  kill -INT "-$launch_pid" 2>/dev/null || kill -INT "$launch_pid" 2>/dev/null || true
  for _ in $(seq 1 30); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      wait "$launch_pid" 2>/dev/null || true
      launch_pid=""
      return
    fi
    sleep 0.2
  done
  publish_zero_cmd_vel
  kill -TERM "-$launch_pid" 2>/dev/null || kill -TERM "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  launch_pid=""
}

setsid env \
  RUN_DIR="$run_dir" \
  BT_ATTACH="${BT_ATTACH:-auto}" \
  SUDO_NONINTERACTIVE="${SUDO_NONINTERACTIVE:-1}" \
  "$project_root/scripts/go2_collect_rssi_imu.sh" \
  "$network_interface" "$collection_duration_sec" "$window_sec" "$step_sec" "$marker_step_m" "$start_s_m" \
  > "$rssi_log" 2>&1 < /dev/null &
rssi_pid=$!

sleep "$warmup_sec"
if ! kill -0 "$rssi_pid" 2>/dev/null; then
  echo "RSSI/IMU collection exited before waypoint launch. Log:" >&2
  tail -80 "$rssi_log" >&2 || true
  wait "$rssi_pid" 2>/dev/null || true
  exit 1
fi

echo "RSSI/IMU collection started: pid=$rssi_pid log=$rssi_log"
echo "Starting waypoint launch..."

set +e
setsid ros2 launch go2_nav2_bridge go2_waypoint_obstacle_avoid.launch.py \
    network_interface:="$network_interface" \
    relative_waypoints:="$relative_waypoints" \
  > >(tee "$waypoint_log") 2>&1 &
launch_pid=$!

deadline=$((SECONDS + waypoint_timeout_sec))
launch_status=0
while kill -0 "$launch_pid" 2>/dev/null; do
  if (( SECONDS >= deadline )); then
    launch_status=124
    break
  fi
  sleep 0.2
done

if [[ "$launch_status" == "0" ]]; then
  wait "$launch_pid"
  launch_status=$?
  launch_pid=""
else
  stop_waypoint_launch
fi
set -e

if [[ "$launch_status" == "124" ]]; then
  echo "Waypoint launch reached timeout (${waypoint_timeout_sec}s); stopping launch and RSSI collection."
elif [[ "$launch_status" != "0" ]]; then
  echo "Waypoint launch exited with status $launch_status."
else
  echo "Waypoint launch exited normally."
fi

stop_rssi_collection

echo "Run finished."
echo "Summary: $summary_file"
echo "RSSI/IMU log: $rssi_log"
echo "Waypoint log: $waypoint_log"
