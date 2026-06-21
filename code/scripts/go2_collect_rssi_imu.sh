#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/go2_collect_rssi_imu.sh <networkInterface> [duration_sec] [window_sec] [step_sec] [marker_step_m] [start_s_m]
  scripts/go2_collect_rssi_imu.sh --preflight <networkInterface> [duration_sec] [window_sec] [step_sec] [marker_step_m] [start_s_m]

Examples:
  scripts/go2_collect_rssi_imu.sh eth0 30
  scripts/go2_collect_rssi_imu.sh eth0 0 1.0 0.5 0.5 0.0

Environment:
  RUN_DIR                 Output directory. Defaults to ~/go2_rssi_runs/<timestamp>.
  BT_ATTACH               Attach /dev/ttyACM* with btattach before scanning. Defaults to auto.
                          Set to 0/off/no to skip, or required to fail if the attach script is missing.
  BT_TTY                  UART Bluetooth device. Defaults to the largest /dev/ttyACM*.
  BT_BAUD                 UART baudrate for btattach. Defaults to 1000000.
  HCI_DEV                 Bluetooth controller for RSSI. If unset, auto-detected after btattach.
  BTATTACH_CLEANUP        Kill btattach started by this script on exit. Defaults to 0.
  IMU_ACC_TO_G_SCALE      Optional override for Go2 accelerometer scale.
  IMU_GYRO_TO_DPS_SCALE   Optional override for Go2 gyroscope scale.
  IMU_ANGLE_TO_DEG_SCALE  Optional override for Go2 rpy angle scale.
  SUDO_NONINTERACTIVE     Set to 1 for remote/background runs. sudo uses -n
                          and fails immediately if passwordless sudo is not configured.
USAGE
}

preflight_only=0
if [[ "${1:-}" == "--preflight" ]]; then
  preflight_only=1
  shift
fi

if [[ $# -lt 1 ]]; then
  usage
  exit 1
fi

network_interface="$1"
duration_sec="${2:-0}"
window_sec="${3:-1.0}"
step_sec="${4:-0.5}"
marker_step_m="${5:-0.5}"
start_s_m="${6:-0.0}"

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${RUN_DIR:-$HOME/go2_rssi_runs/$(date +%Y%m%d_%H%M%S)}"

imu_logger="$project_root/unitree_sdk2/build_go2/bin/go2_imu_logger"
scan_script="$project_root/bt_migrate_pack/start_coded_scan.sh"
attach_script="$project_root/bt_migrate_pack/go2_attach_ttyacm.sh"

sudo_cmd() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  elif [[ "${SUDO_NONINTERACTIVE:-0}" == "1" ]]; then
    sudo -n "$@"
  else
    sudo "$@"
  fi
}

sudo_check() {
  if [[ "${EUID}" -eq 0 ]]; then
    return 0
  fi
  if [[ "${SUDO_NONINTERACTIVE:-0}" == "1" ]]; then
    sudo -n true
  else
    sudo -v
  fi
}

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

if [[ ! -x "$imu_logger" ]]; then
  echo "Missing Go2 IMU logger: $imu_logger" >&2
  echo "Prepare and build it with:" >&2
  echo "  git clone https://github.com/unitreerobotics/unitree_sdk2.git" >&2
  echo "  git -C unitree_sdk2 apply ../unitree_sdk2_go2_imu_logger.patch" >&2
  echo "  cmake -S unitree_sdk2 -B unitree_sdk2/build_go2" >&2
  echo "  cmake --build unitree_sdk2/build_go2 --target go2_imu_logger -j\"\$(nproc)\"" >&2
  exit 1
fi

if [[ ! -x "$rssi_binary" ]]; then
  echo "Missing executable RSSI binary." >&2
  echo "Build it with:" >&2
  echo "  scripts/build_rssi_logger.sh" >&2
  exit 1
fi

if [[ "$preflight_only" == "1" ]]; then
  echo "Remote RSSI preflight"
  echo "project_root=$project_root"
  echo "network_interface=$network_interface"
  echo "rssi_binary=$rssi_binary"
  echo "imu_logger=$imu_logger"

  if ! ip link show "$network_interface" >/dev/null 2>&1; then
    echo "Network interface not found: $network_interface" >&2
    exit 1
  fi

  if ! command -v btattach >/dev/null 2>&1; then
    echo "Missing command: btattach" >&2
    exit 1
  fi
  if ! command -v hciconfig >/dev/null 2>&1; then
    echo "Missing command: hciconfig" >&2
    exit 1
  fi
  if ! command -v hcitool >/dev/null 2>&1; then
    echo "Missing command: hcitool" >&2
    exit 1
  fi
fi

mkdir -p "$run_dir"
echo "Output directory: $run_dir"

imu_args=("$network_interface" "$run_dir/imu_stream.csv" 0)
if [[ -n "${IMU_ACC_TO_G_SCALE:-}" || -n "${IMU_GYRO_TO_DPS_SCALE:-}" || -n "${IMU_ANGLE_TO_DEG_SCALE:-}" ]]; then
  imu_args+=(
    "${IMU_ACC_TO_G_SCALE:-0.1019716213}"
    "${IMU_GYRO_TO_DPS_SCALE:-57.2957795131}"
    "${IMU_ANGLE_TO_DEG_SCALE:-57.2957795131}"
  )
fi

imu_pid=""
btattach_pid=""
btattach_started="0"

cleanup() {
  if [[ -n "$imu_pid" ]] && kill -0 "$imu_pid" 2>/dev/null; then
    kill "$imu_pid" 2>/dev/null || true
    wait "$imu_pid" 2>/dev/null || true
  fi
  if [[ "${BTATTACH_CLEANUP:-0}" == "1" && "$btattach_started" == "1" && -n "$btattach_pid" ]]; then
    sudo_cmd kill "$btattach_pid" 2>/dev/null || true
    wait "$btattach_pid" 2>/dev/null || true
  fi
  sudo_cmd chown -R "$(id -u):$(id -g)" "$run_dir" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

bt_attach_mode="${BT_ATTACH:-auto}"
case "$bt_attach_mode" in
  0|false|False|FALSE|no|No|NO|off|Off|OFF)
    echo "Skipping ttyACM btattach because BT_ATTACH=$bt_attach_mode"
    ;;
  *)
    if [[ -f "$attach_script" ]]; then
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

SUDO_NONINTERACTIVE="${SUDO_NONINTERACTIVE:-0}" bash "$scan_script" "${HCI_DEV:-hci0}"

if [[ "$preflight_only" == "1" ]]; then
  echo "preflight_ok=1"
  exit 0
fi

"$imu_logger" "${imu_args[@]}" &
imu_pid=$!

cd "$run_dir"
sudo_cmd "$rssi_binary" "$duration_sec" rssi_realtime_windows.csv \
  "$window_sec" "$step_sec" "$marker_step_m" "$start_s_m" none

echo "Done. Output directory: $run_dir"
