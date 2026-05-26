#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  bt_migrate_pack/go2_run_rssi.sh auto [duration_sec] [distance_step_m]
  bt_migrate_pack/go2_run_rssi.sh manual [duration_sec] [distance_m]

Examples:
  BT_TTY=/dev/ttyACM1 bt_migrate_pack/go2_run_rssi.sh auto 5 0.5
  BT_TTY=/dev/ttyACM1 bt_migrate_pack/go2_run_rssi.sh manual 5 0

Environment:
  RUN_DIR            Output directory. Defaults to ./go2_rssi_runs/<timestamp>.
  BT_ATTACH          Attach /dev/ttyACM* with btattach before scanning. Defaults to auto.
                     Set to 0/off/no if you already attached HCI manually.
  BT_TTY             UART Bluetooth device. Defaults to the largest /dev/ttyACM*.
  BT_BAUD            UART baudrate for btattach. Defaults to 1000000.
  HCI_DEV            Bluetooth controller for RSSI. If unset, auto-detected after btattach.
  BTATTACH_CLEANUP   Kill btattach started by this script on exit. Defaults to 0.
USAGE
}

mode="${1:-auto}"
if [[ "$mode" == "-h" || "$mode" == "--help" ]]; then
  usage
  exit 0
fi

duration_sec="${2:-5}"
measurement_arg="${3:-}"

pack_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
attach_script="$pack_dir/go2_attach_ttyacm.sh"
scan_script="$pack_dir/start_coded_scan.sh"

case "$mode" in
  auto)
    rssi_binary="$pack_dir/test_rssi_auto"
    measurement_arg="${measurement_arg:-0.5}"
    ;;
  manual)
    rssi_binary="$pack_dir/test_rssi_manual"
    measurement_arg="${measurement_arg:-0}"
    ;;
  *)
    usage >&2
    exit 1
    ;;
esac

if [[ ! -x "$rssi_binary" ]]; then
  echo "Missing executable RSSI binary: $rssi_binary" >&2
  exit 1
fi

if [[ ! -f "$scan_script" ]]; then
  echo "Missing scan script: $scan_script" >&2
  exit 1
fi

run_dir="${RUN_DIR:-$PWD/go2_rssi_runs/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$run_dir"

btattach_pid=""
btattach_started="0"

cleanup() {
  if [[ "${BTATTACH_CLEANUP:-0}" == "1" && "$btattach_started" == "1" && -n "$btattach_pid" ]]; then
    sudo kill "$btattach_pid" 2>/dev/null || true
    wait "$btattach_pid" 2>/dev/null || true
  fi
  sudo chown -R "$(id -u):$(id -g)" "$run_dir" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

bt_attach_mode="${BT_ATTACH:-auto}"
case "$bt_attach_mode" in
  0|false|False|FALSE|no|No|NO|off|Off|OFF)
    hci_dev="${HCI_DEV:-hci0}"
    echo "Skipping ttyACM btattach because BT_ATTACH=$bt_attach_mode"
    ;;
  *)
    if [[ ! -f "$attach_script" ]]; then
      echo "Missing Bluetooth attach script: $attach_script" >&2
      exit 1
    fi
    attach_output="$(bash "$attach_script")"
    echo "$attach_output"
    hci_dev="$(printf '%s\n' "$attach_output" | awk -F= '$1 == "HCI_DEV" {print $2}' | tail -n 1)"
    btattach_pid="$(printf '%s\n' "$attach_output" | awk -F= '$1 == "BT_ATTACH_PID" {print $2}' | tail -n 1)"
    btattach_started="$(printf '%s\n' "$attach_output" | awk -F= '$1 == "BT_ATTACH_STARTED" {print $2}' | tail -n 1)"
    hci_dev="${hci_dev:-${HCI_DEV:-hci0}}"
    ;;
esac

sudo bash "$scan_script" "$hci_dev"

echo "Output directory: $run_dir"
cd "$run_dir"
sudo "$rssi_binary" "$duration_sec" "$measurement_arg"

echo "Done. Output directory: $run_dir"
