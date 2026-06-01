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

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${RUN_DIR:-$HOME/go2_rssi_runs/straight_$(date +%Y%m%d_%H%M%S)}"

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
echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}"
if [[ "$log_sport_state" != "1" ]]; then
  echo "LOG_SPORT_STATE must be 1 for odom-controlled distance stopping." >&2
  exit 1
fi

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
  sudo "$rssi_binary" "$duration_sec" rssi_realtime_windows.csv \
    "$window_sec" "$step_sec" 1.0 0.0 none
) &
rssi_pid=$!

sleep 2
"$line_runner" "$network_interface" "$run_dir/sport_state_straight.csv" \
  "$distance_m" "$speed_mps" "$warmup_sec" "$control_hz" "$log_sport_state" "$max_runtime_sec" || runner_status=$?

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
