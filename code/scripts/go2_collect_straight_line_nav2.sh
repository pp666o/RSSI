#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/go2_collect_straight_line_nav2.sh <networkInterface> [distance_m] [speed_mps] [window_sec] [step_sec]

Example:
  RUN_DIR="$HOME/go2_rssi_runs/nav2_1m" scripts/go2_collect_straight_line_nav2.sh eth0 1.0 0.30 1.0 0.5

Environment:
  RUN_DIR                 Output directory. Defaults to ~/go2_rssi_runs/nav2_straight_<timestamp>.
  ROS_SETUP               ROS 2 setup file. Defaults to /opt/ros/humble/setup.bash if present,
                          then jazzy/kilted, then current shell environment.
  NAV2_WS_SETUP           Optional overlay setup file after building code/ros2_ws.
                          Defaults to code/ros2_ws/install/setup.bash if present.
  NAV2_PARAMS_FILE        Motion parameter file. Defaults to package minimal controller params
                          for MOVE_BACKEND=direct, otherwise package straight params.
  MAX_RUNTIME_SEC         Timeout for the Nav2 straight-path client. Defaults to max(30, 4 * distance / speed).
  MOVE_BACKEND            direct, nav2, or cmd_vel. Defaults to nav2.
                          direct starts go2_nav2_bridge and a path_s/odom-based straight controller.
                          cmd_vel starts only go2_nav2_bridge and publishes /cmd_vel by time.
                          nav2 starts the Nav2 Controller action path.
  NAV2_STARTUP_SEC        Initial time to wait after launch before sending motion. Defaults to 3.
  DIRECT_DISTANCE_SOURCE  path_s, odom, or time for MOVE_BACKEND=direct. Defaults to path_s.
  DIRECT_CONTROL_HZ       Command publish rate for MOVE_BACKEND=direct. Defaults to 20.
  DIRECT_START_DELAY_SEC  Warmup delay before MOVE_BACKEND=direct starts moving. Defaults to 0.
  DIRECT_POINTCLOUD_AVOIDANCE
                          Set to 1 to use /go2/pointcloud for direct-mode front
                          stop/turn and side clearance correction. Defaults to 1.
  NAV2_READY_TIMEOUT_SEC  Time to wait for /controller_server to become active when
                          MOVE_BACKEND=nav2. Defaults to 25.
  NAV2_USE_NAVIGATE_TO_POSE
                          Set true to use Nav2 bt_navigator/NavigateToPose with replanning
                          instead of a one-shot FollowPath goal. Defaults to false.
  ALLOW_EXISTING_NAV2     Set to 1 to skip the preflight check for existing Nav2/Go2
                          bridge processes. Defaults to 0.
  RSSI_EXTRA_SEC          Extra RSSI/IMU collection time after motion budget. Defaults to 5.
  RSSI_MODE               local, remote, or off. Defaults to local.
                          local: RSSI receiver is on this computer.
                          remote: RSSI receiver is on the Go2 computer and is started over SSH.
                          off: do not start RSSI collection.
  ROBOT_SSH               SSH target for RSSI_MODE=remote, for example unitree@192.168.123.18.
  ROBOT_PROJECT_DIR       Project directory on the Go2 computer. Defaults to /home/unitree/rssi_go2.
  ROBOT_RUN_DIR           Remote RSSI output directory. Defaults to /home/unitree/go2_rssi_runs/<run_name>_rssi.
  ROBOT_NETWORK_INTERFACE Network interface passed to Go2-side collectors. Defaults to eth0.
  ROBOT_SSH_OPTS          Extra options passed to ssh/scp for RSSI_MODE=remote.
  REMOTE_RSSI_PREFLIGHT   Set to 0 to skip remote RSSI preflight. Defaults to 1.
  BT_ATTACH               Attach /dev/ttyACM* with btattach before scanning. Defaults to auto.
  BT_TTY                  UART Bluetooth device. Defaults to the largest /dev/ttyACM*.
  BT_BAUD                 UART baudrate for btattach. Defaults to 1000000.
  HCI_DEV                 Bluetooth controller for RSSI. If unset, auto-detected after btattach.
  BTATTACH_CLEANUP        Kill btattach started by this script on exit. Defaults to 0.
USAGE
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 1
fi

network_interface="$1"
distance_m="${2:-1.0}"
speed_mps="${3:-0.30}"
window_sec="${4:-1.0}"
step_sec="${5:-0.5}"
extra_sec="${RSSI_EXTRA_SEC:-5}"
rssi_mode="${RSSI_MODE:-local}"
move_backend="${MOVE_BACKEND:-nav2}"
nav2_use_navigate_to_pose="${NAV2_USE_NAVIGATE_TO_POSE:-false}"
nav2_use_planner="${NAV2_USE_PLANNER:-false}"

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${RUN_DIR:-$HOME/go2_rssi_runs/nav2_straight_$(date +%Y%m%d_%H%M%S)}"

imu_logger="$project_root/unitree_sdk2/build_go2/bin/go2_imu_logger"
scan_script="$project_root/bt_migrate_pack/start_coded_scan.sh"
attach_script="$project_root/bt_migrate_pack/go2_attach_ttyacm.sh"

source_setup() {
  local setup_file="$1"
  # ROS 2 setup files may read unset AMENT_* variables, which conflicts with set -u.
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

case "$rssi_mode" in
  local|remote|off) ;;
  *)
    echo "RSSI_MODE must be local, remote, or off. Got: $rssi_mode" >&2
    exit 1
    ;;
esac
case "$move_backend" in
  direct|nav2|cmd_vel) ;;
  *)
    echo "MOVE_BACKEND must be direct, nav2, or cmd_vel. Got: $move_backend" >&2
    exit 1
    ;;
esac
check_no_existing_nav2_processes() {
  if [[ "${ALLOW_EXISTING_NAV2:-0}" == "1" ]]; then
    return
  fi

  local existing
  existing="$(pgrep -af \
    'ros2 launch go2_nav2_bridge|go2_nav2_bridge/lib/go2_nav2_bridge/go2_nav2_bridge|nav2_straight_path_client|nav2_planner/planner_server|nav2_controller/controller_server|nav2_velocity_smoother/velocity_smoother|nav2_collision_monitor/collision_monitor|nav2_bt_navigator/bt_navigator|nav2_behaviors/behavior_server|nav2_lifecycle_manager/lifecycle_manager' \
    || true)"
  if [[ -z "$existing" ]]; then
    return
  fi

  echo "Existing Go2/Nav2 control processes are still running:" >&2
  printf '%s\n' "$existing" >&2
  echo >&2
  echo "Stop them before starting a new run, for example:" >&2
  echo "  pkill -INT -f 'ros2 launch go2_nav2_bridge|go2_nav2_bridge/lib/go2_nav2_bridge/go2_nav2_bridge|nav2_controller/controller_server|nav2_lifecycle_manager/lifecycle_manager'" >&2
  echo "If you intentionally want to reuse existing nodes, set ALLOW_EXISTING_NAV2=1." >&2
  exit 1
}

is_truthy() {
  case "$1" in
    1|true|True|TRUE|yes|Yes|YES|on|On|ON) return 0 ;;
    *) return 1 ;;
  esac
}

rssi_binary=""
if [[ "$rssi_mode" == "local" ]]; then
  if [[ -n "${RSSI_BINARY:-}" ]]; then
    rssi_binary="$RSSI_BINARY"
  else
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
fi

if [[ ! -x "$imu_logger" ]]; then
  echo "Missing Go2 IMU logger." >&2
  echo "Build it with:" >&2
  echo "  cmake -S unitree_sdk2 -B unitree_sdk2/build_go2" >&2
  echo "  cmake --build unitree_sdk2/build_go2 --target go2_imu_logger -j\"\$(nproc)\"" >&2
  exit 1
fi

if [[ "$rssi_mode" == "local" && ! -x "$rssi_binary" ]]; then
  echo "Missing executable RSSI binary." >&2
  echo "Build it with:" >&2
  echo "  scripts/build_rssi_logger.sh" >&2
  exit 1
fi

if ! awk "BEGIN {exit !($distance_m > 0 && $speed_mps > 0 && $window_sec > 0 && $step_sec > 0)}"; then
  echo "distance_m, speed_mps, window_sec, and step_sec must be positive." >&2
  exit 1
fi
check_no_existing_nav2_processes

distance_m="$(awk -v x="$distance_m" 'BEGIN {if (x < 0.1) x = 0.1; if (x > 100.0) x = 100.0; printf "%.6f", x}')"
speed_mps="$(awk -v x="$speed_mps" 'BEGIN {if (x < 0.05) x = 0.05; if (x > 0.35) x = 0.35; printf "%.6f", x}')"
max_runtime_sec="${MAX_RUNTIME_SEC:-$(awk -v d="$distance_m" -v v="$speed_mps" 'BEGIN {x = 4 * d / v; if (x < 30) x = 30; printf "%.0f", x}')}"
max_runtime_sec="$(awk -v x="$max_runtime_sec" 'BEGIN {if (x < 1.0) x = 1.0; if (x > 3600.0) x = 3600.0; printf "%.6f", x}')"
duration_sec="$(awk -v m="$max_runtime_sec" -v e="$extra_sec" 'BEGIN {printf "%.0f", m + e}')"

sdk_arch="$(uname -m)"
sdk_lib_dir="$project_root/unitree_sdk2/thirdparty/lib/$sdk_arch"
case ":${LD_LIBRARY_PATH:-}:" in
  *":$sdk_lib_dir:"*) ;;
  *) export LD_LIBRARY_PATH="$sdk_lib_dir:${LD_LIBRARY_PATH:-}" ;;
esac

if [[ -n "${ROS_SETUP:-}" ]]; then
  source_setup "$ROS_SETUP"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source_setup /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_setup /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/kilted/setup.bash ]]; then
  source_setup /opt/ros/kilted/setup.bash
fi

default_overlay="$project_root/ros2_ws/install/setup.bash"
if [[ -n "${NAV2_WS_SETUP:-}" ]]; then
  source_setup "$NAV2_WS_SETUP"
elif [[ -f "$default_overlay" ]]; then
  source_setup "$default_overlay"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 command not found. On Ubuntu 22.04, install/source ROS 2 Humble and build code/ros2_ws first." >&2
  exit 1
fi

if ! ros2 pkg prefix go2_nav2_bridge >/dev/null 2>&1; then
  echo "go2_nav2_bridge is not built or not sourced." >&2
  echo "Build it with:" >&2
  echo "  cd $project_root/ros2_ws" >&2
  echo "  colcon build --packages-select go2_nav2_bridge" >&2
  echo "  source install/setup.bash" >&2
  exit 1
fi

package_params_dir="$(ros2 pkg prefix go2_nav2_bridge)/share/go2_nav2_bridge/params"
if [[ -n "${NAV2_PARAMS_FILE:-}" ]]; then
  params_file="$NAV2_PARAMS_FILE"
elif [[ "$move_backend" == "direct" && -f "$package_params_dir/go2_nav2_minimal_controller_params.yaml" ]]; then
  params_file="$package_params_dir/go2_nav2_minimal_controller_params.yaml"
else
  params_file="$package_params_dir/go2_nav2_straight_params.yaml"
fi

mkdir -p "$run_dir"
if [[ "$move_backend" == "nav2" ]]; then
  runtime_params_file="$run_dir/nav2_runtime_params.yaml"
  python3 - "$params_file" "$runtime_params_file" "$speed_mps" "${NAV2_MIN_FORWARD_VX:-}" <<'PY'
import sys
import yaml

src, dst, speed_arg, min_forward_arg = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
with open(src, "r", encoding="utf-8") as f:
    data = yaml.safe_load(f)

bridge = data["go2_nav2_bridge"]["ros__parameters"]
follow = data["controller_server"]["ros__parameters"]["FollowPath"]
smoother = data["velocity_smoother"]["ros__parameters"]

configured_vx = float(bridge.get("max_vx_mps", follow.get("max_vel_x", 0.12)))
configured_vy = float(bridge.get("max_vy_mps", follow.get("max_vel_y", 0.06)))
configured_wz = float(bridge.get("max_wz_radps", follow.get("max_vel_theta", 0.35)))

vx = max(0.03, min(speed_arg, configured_vx))
vy = min(configured_vy, max(0.0, 0.5 * vx))
wz = configured_wz
min_forward_vx = min(vx, max(0.03, 0.60 * vx))
if min_forward_arg:
    min_forward_vx = max(0.0, min(vx, float(min_forward_arg)))

bridge["max_vx_mps"] = vx
bridge["max_vy_mps"] = vy
bridge["max_wz_radps"] = wz

follow["min_vel_x"] = min_forward_vx
follow["max_vel_x"] = vx
follow["max_vel_y"] = vy
follow["min_vel_y"] = -vy
follow["max_vel_theta"] = wz
follow["min_speed_xy"] = min_forward_vx
follow["max_speed_xy"] = max(vx, vy)
if "RegulatedPurePursuitController" in str(follow.get("plugin", "")):
    follow["max_linear_vel"] = vx
    follow["min_linear_vel"] = 0.0
    follow["max_angular_vel"] = wz
    follow["min_angular_vel"] = -wz
    follow["rotate_to_heading_angular_vel"] = min(wz, float(follow.get("rotate_to_heading_angular_vel", wz)))

smoother["max_velocity"] = [vx, vy, wz]
smoother["min_velocity"] = [0.0, -vy, -wz]

behavior = data.get("behavior_server", {}).get("ros__parameters")
if behavior is not None:
    behavior["max_rotational_vel"] = wz

with open(dst, "w", encoding="utf-8") as f:
    yaml.safe_dump(data, f, sort_keys=False)
PY
  params_file="$runtime_params_file"
fi
echo "Output directory: $run_dir"
echo "Go2 straight-line collection: backend=${move_backend} distance=${distance_m}m speed_limit=${speed_mps}m/s max_runtime=${max_runtime_sec}s collection_duration=${duration_sec}s"
echo "NAV2_PARAMS_FILE=$params_file"
echo "RSSI_MODE=$rssi_mode"
echo "MOVE_BACKEND=$move_backend"
echo "NAV2_USE_NAVIGATE_TO_POSE=$nav2_use_navigate_to_pose"
echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}"

imu_pid=""
rssi_pid=""
nav2_pid=""
btattach_pid=""
btattach_started="0"
client_status=0
remote_rssi_status=0
robot_ssh="${ROBOT_SSH:-}"
robot_project_dir="${ROBOT_PROJECT_DIR:-/home/unitree/rssi_go2}"
robot_network_interface="${ROBOT_NETWORK_INTERFACE:-eth0}"
robot_run_dir="${ROBOT_RUN_DIR:-/home/unitree/go2_rssi_runs/$(basename "$run_dir")_rssi}"
robot_rssi_pidfile="$robot_run_dir/remote_rssi.pid"
robot_ssh_opts="${ROBOT_SSH_OPTS:-}"

remote_shell() {
  # shellcheck disable=SC2086
  ssh $robot_ssh_opts "$robot_ssh" "$@"
}

remote_copy_from() {
  # shellcheck disable=SC2086
  scp $robot_ssh_opts "$robot_ssh:$1" "$2" >/dev/null 2>&1 || true
}

preflight_remote_rssi() {
  if [[ "${REMOTE_RSSI_PREFLIGHT:-1}" == "0" ]]; then
    echo "Skipping remote RSSI preflight because REMOTE_RSSI_PREFLIGHT=0"
    return
  fi

  local remote_cmd
  remote_cmd="cd '$robot_project_dir' && export BT_ATTACH='${BT_ATTACH:-auto}' BT_TTY='${BT_TTY:-}' BT_BAUD='${BT_BAUD:-1000000}' HCI_DEV='${HCI_DEV:-}' SUDO_NONINTERACTIVE=1 && exec scripts/go2_collect_rssi_imu.sh --preflight '$robot_network_interface' '$duration_sec' '$window_sec' '$step_sec' 1.0 0.0"

  echo "Checking remote Go2 RSSI readiness on $robot_ssh..."
  if remote_shell "bash -lc $(printf '%q' "$remote_cmd")" >"$run_dir/remote_rssi_preflight.log" 2>&1; then
    echo "Remote Go2 RSSI preflight passed."
    return
  fi

  echo "Remote Go2 RSSI preflight failed; motion will not start." >&2
  echo "Preflight log: $run_dir/remote_rssi_preflight.log" >&2
  tail -n 80 "$run_dir/remote_rssi_preflight.log" >&2 || true
  cat >&2 <<EOF

Fix one of these before collecting:
  1. On the Go2 computer, run once:
     cd $robot_project_dir
     scripts/configure_go2_rssi_sudoers.sh $robot_project_dir
  2. Or open a Go2 SSH terminal and cache sudo immediately before this run:
     ssh -t $robot_ssh 'sudo -v'

The first option is the stable one for one-window automated RSSI collection.
EOF
  exit 1
}

start_remote_rssi() {
  if [[ -z "$robot_ssh" ]]; then
    echo "RSSI_MODE=remote requires ROBOT_SSH, for example ROBOT_SSH=unitree@192.168.123.18" >&2
    exit 1
  fi

  preflight_remote_rssi

  local remote_cmd
  remote_cmd="cd '$robot_project_dir' && mkdir -p '$robot_run_dir' && echo \\\$\\\$ > '$robot_rssi_pidfile' && export RUN_DIR='$robot_run_dir' BT_ATTACH='${BT_ATTACH:-auto}' BT_TTY='${BT_TTY:-}' BT_BAUD='${BT_BAUD:-1000000}' HCI_DEV='${HCI_DEV:-}' BTATTACH_CLEANUP='${BTATTACH_CLEANUP:-0}' SUDO_NONINTERACTIVE=1 && exec scripts/go2_collect_rssi_imu.sh '$robot_network_interface' '$duration_sec' '$window_sec' '$step_sec' 1.0 0.0"

  echo "Starting remote Go2 RSSI collection on $robot_ssh:$robot_run_dir"
  remote_shell "bash -lc $(printf '%q' "$remote_cmd")" >"$run_dir/remote_rssi.log" 2>&1 &
  rssi_pid=$!
}

stop_remote_rssi() {
  if [[ "$rssi_mode" != "remote" || -z "$robot_ssh" ]]; then
    return
  fi
  remote_shell "bash -lc 'if [[ -f \"$robot_rssi_pidfile\" ]]; then kill -INT \$(cat \"$robot_rssi_pidfile\") 2>/dev/null || true; fi'" >/dev/null 2>&1 || true
}

copy_remote_rssi_outputs() {
  if [[ "$rssi_mode" != "remote" || -z "$robot_ssh" ]]; then
    return
  fi
  echo "Copying remote RSSI CSVs back from $robot_ssh:$robot_run_dir"
  remote_copy_from "$robot_run_dir/rssi_realtime_windows.csv" "$run_dir/rssi_realtime_windows.csv"
  remote_copy_from "$robot_run_dir/rssi_raw_stream.csv" "$run_dir/rssi_raw_stream.csv"
  remote_copy_from "$robot_run_dir/rssi_markers.csv" "$run_dir/rssi_markers.csv"
  remote_copy_from "$robot_run_dir/imu_stream.csv" "$run_dir/remote_imu_stream.csv"
}

print_nav2_diagnostics() {
  local log_file="$run_dir/nav2_launch.log"
  if [[ ! -f "$log_file" ]]; then
    return
  fi
  echo "Recent Nav2 diagnostics from $log_file:" >&2
  grep -E "ERROR|WARN|cmd_vel -> Move|No cmd_vel|Move returned|Received a goal|Failed|Controller Server|lifecycle|goal|NavigateToPose|Robot to|pointcloud_roi|bt_navigator|behavior_server" \
    "$log_file" | tail -n 80 >&2 || true
}

wait_for_lifecycle_node_active() {
  local node_name="$1"
  local timeout_sec="${NAV2_READY_TIMEOUT_SEC:-25}"
  local deadline=$((SECONDS + timeout_sec))
  local state=""

  echo "Waiting for ${node_name} to become active..."
  while (( SECONDS < deadline )); do
    state="$(timeout 3s ros2 lifecycle get "$node_name" 2>/dev/null || true)"
    if printf '%s\n' "$state" | grep -q "active"; then
      echo "${node_name} is active."
      return 0
    fi
    sleep 1
  done

  echo "Timed out waiting for ${node_name} to become active. Last state: ${state:-unknown}" >&2
  print_nav2_diagnostics
  return 1
}

wait_for_nav2_ready() {
  local log_file="$run_dir/nav2_launch.log"
  local timeout_sec="${NAV2_READY_TIMEOUT_SEC:-25}"
  local deadline=$((SECONDS + timeout_sec))
  local required_active_lines=1
  local lifecycle_nodes=(/controller_server)
  if is_truthy "$nav2_use_planner" || is_truthy "$nav2_use_navigate_to_pose"; then
    lifecycle_nodes+=(/planner_server)
  fi
  if is_truthy "$nav2_use_navigate_to_pose"; then
    lifecycle_nodes+=(/behavior_server /bt_navigator)
  fi
  if is_truthy "$nav2_use_navigate_to_pose"; then
    required_active_lines=2
  fi

  echo "Waiting for Nav2 managed nodes to become active..."
  while (( SECONDS < deadline )); do
    local all_active=1
    local node_name
    for node_name in "${lifecycle_nodes[@]}"; do
      local state
      state="$(timeout 2s ros2 lifecycle get "$node_name" 2>/dev/null || true)"
      if ! printf '%s\n' "$state" | grep -q "active"; then
        all_active=0
        break
      fi
    done
    if (( all_active == 1 )); then
      echo "Nav2 lifecycle nodes are active."
      return 0
    fi

    local active_lines=0
    if [[ -f "$log_file" ]]; then
      active_lines="$(grep -c "Managed nodes are active" "$log_file" 2>/dev/null || true)"
    fi
    if (( active_lines >= required_active_lines )); then
      echo "Nav2 managed nodes are active."
      return 0
    fi
    sleep 1
  done

  echo "Timed out waiting for Nav2 managed nodes to become active." >&2
  print_nav2_diagnostics
  return 1
}

stop_background_collectors() {
  if [[ -n "$rssi_pid" ]] && kill -0 "$rssi_pid" 2>/dev/null; then
    if [[ "$rssi_mode" == "remote" ]]; then
      stop_remote_rssi
      wait "$rssi_pid" 2>/dev/null || remote_rssi_status=$?
    else
      sudo kill "$rssi_pid" 2>/dev/null || true
      wait "$rssi_pid" 2>/dev/null || true
    fi
  fi
  rssi_pid=""
  if [[ -n "$imu_pid" ]] && kill -0 "$imu_pid" 2>/dev/null; then
    kill "$imu_pid" 2>/dev/null || true
    wait "$imu_pid" 2>/dev/null || true
  fi
  imu_pid=""
}

cleanup() {
  stop_background_collectors
  if [[ -n "$nav2_pid" ]] && kill -0 "$nav2_pid" 2>/dev/null; then
    kill "$nav2_pid" 2>/dev/null || true
    wait "$nav2_pid" 2>/dev/null || true
  fi
  nav2_pid=""
  if [[ "${BTATTACH_CLEANUP:-0}" == "1" && "$btattach_started" == "1" && -n "$btattach_pid" ]]; then
    sudo kill "$btattach_pid" 2>/dev/null || true
    wait "$btattach_pid" 2>/dev/null || true
  fi
  if [[ -w "$run_dir" ]]; then
    chown -R "$(id -u):$(id -g)" "$run_dir" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$rssi_mode" == "local" ]]; then
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

  bash "$scan_script" "${HCI_DEV:-hci0}"
elif [[ "$rssi_mode" == "remote" ]]; then
  start_remote_rssi
else
  echo "RSSI collection disabled because RSSI_MODE=off"
fi

"$imu_logger" "$network_interface" "$run_dir/imu_stream.csv" "$duration_sec" &
imu_pid=$!

if [[ "$rssi_mode" == "local" ]]; then
  (
    cd "$run_dir"
    exec sudo "$rssi_binary" "$duration_sec" rssi_realtime_windows.csv \
      "$window_sec" "$step_sec" 1.0 0.0 none
  ) &
  rssi_pid=$!
fi

ros2 launch go2_nav2_bridge go2_nav2_straight.launch.py \
  params_file:="$params_file" \
  move_backend:="$move_backend" \
  use_bt_navigator:="$nav2_use_navigate_to_pose" \
  use_planner:="$nav2_use_planner" \
  network_interface:="$network_interface" \
  >"$run_dir/nav2_launch.log" 2>&1 &
nav2_pid=$!

startup_sleep_sec="${NAV2_STARTUP_SEC:-}"
if [[ -z "$startup_sleep_sec" ]]; then
  if [[ "$move_backend" == "nav2" ]]; then
    startup_sleep_sec="3"
  else
    startup_sleep_sec="3"
  fi
fi
sleep "$startup_sleep_sec"

if [[ "$move_backend" == "cmd_vel" ]]; then
  move_duration_sec="$(awk -v d="$distance_m" -v v="$speed_mps" 'BEGIN {x = d / v; if (x < 0.5) x = 0.5; printf "%.3f", x}')"
  echo "Publishing direct /cmd_vel vx=$speed_mps for ${move_duration_sec}s"
  timeout "$max_runtime_sec" ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $speed_mps, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
    --times "$(awk -v t="$move_duration_sec" 'BEGIN {printf "%d", int(t * 10 + 0.5)}')" || client_status=$?
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
elif [[ "$move_backend" == "direct" ]]; then
  direct_outer_timeout_sec="$(awk -v x="$max_runtime_sec" 'BEGIN {printf "%.0f", x + 8}')"
  direct_control_hz="$(awk -v x="${DIRECT_CONTROL_HZ:-20.0}" 'BEGIN {if (x < 2.0) x = 2.0; if (x > 50.0) x = 50.0; printf "%.6f", x}')"
  direct_start_delay_sec="$(awk -v x="${DIRECT_START_DELAY_SEC:-0.0}" 'BEGIN {if (x < 0.0) x = 0.0; if (x > 10.0) x = 10.0; printf "%.6f", x}')"
  timeout "$direct_outer_timeout_sec" ros2 run go2_nav2_bridge go2_direct_straight_controller \
    --ros-args \
    --params-file "$params_file" \
    -p distance_m:="$distance_m" \
    -p speed_mps:="$speed_mps" \
    -p max_runtime_sec:="$max_runtime_sec" \
    -p control_hz:="$direct_control_hz" \
    -p start_delay_sec:="$direct_start_delay_sec" \
    -p distance_source:="${DIRECT_DISTANCE_SOURCE:-path_s}" \
    -p use_pointcloud_avoidance:="${DIRECT_POINTCLOUD_AVOIDANCE:-true}" \
    -p require_fresh_pointcloud:="${DIRECT_REQUIRE_FRESH_POINTCLOUD:-true}" \
    -p pointcloud_front_stop_m:="${DIRECT_FRONT_STOP_M:-0.42}" \
    -p pointcloud_front_turn_m:="${DIRECT_FRONT_TURN_M:-0.75}" \
    -p pointcloud_side_clearance_m:="${DIRECT_SIDE_CLEARANCE_M:-0.24}" \
    -p max_turn_radps:="${DIRECT_MAX_TURN_RADPS:-0.40}" || client_status=$?
else
  if wait_for_nav2_ready; then
    client_frame_args=()
    if [[ -n "${NAV2_CLIENT_FRAME:-}" ]]; then
      client_frame_args=(-p odom_frame:="$NAV2_CLIENT_FRAME")
    fi
    timeout "$max_runtime_sec" ros2 run go2_nav2_bridge nav2_straight_path_client \
      --ros-args \
      --params-file "$params_file" \
      -p distance_m:="$distance_m" \
      -p use_planner:="$nav2_use_planner" \
      -p use_navigate_to_pose:="$nav2_use_navigate_to_pose" \
      -p goal_lateral_offset_m:="${NAV2_GOAL_LATERAL_OFFSET_M:-0.0}" \
      "${client_frame_args[@]}" \
      -p plan_only:="${NAV2_PLAN_ONLY:-false}" || client_status=$?
  else
    client_status=1
  fi
fi

if [[ "$client_status" -ne 0 ]]; then
  echo "Motion client exited with status $client_status" >&2
  print_nav2_diagnostics
fi

echo "Go2 motion finished; stopping RSSI/IMU background collectors."
stop_background_collectors
copy_remote_rssi_outputs

cat > "$run_dir/run_metadata.txt" <<EOF
mode=go2_straight
move_backend=$move_backend
nav2_use_navigate_to_pose=$nav2_use_navigate_to_pose
network_interface=$network_interface
distance_m=$distance_m
speed_mps=$speed_mps
window_sec=$window_sec
step_sec=$step_sec
max_runtime_sec=$max_runtime_sec
rssi_duration_sec=$duration_sec
rssi_mode=$rssi_mode
remote_rssi_status=$remote_rssi_status
robot_ssh=$robot_ssh
robot_project_dir=$robot_project_dir
robot_network_interface=$robot_network_interface
robot_run_dir=$robot_run_dir
nav2_params_file=$params_file
hci_dev=${HCI_DEV:-hci0}
rssi_binary=$rssi_binary
sdk_lib_dir=$sdk_lib_dir
ld_library_path=${LD_LIBRARY_PATH:-}
EOF

echo "Output files:"
ls -lh "$run_dir" || true
echo "Done. Output directory: $run_dir"
exit "$client_status"
