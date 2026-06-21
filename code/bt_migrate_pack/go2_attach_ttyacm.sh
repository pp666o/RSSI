#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  bt_migrate_pack/go2_attach_ttyacm.sh [tty_device]

Environment:
  BT_TTY              UART Bluetooth device. Defaults to the largest /dev/ttyACM*.
  BT_BAUD             UART baudrate for btattach. Defaults to 1000000.
  HCI_DEV             Preferred HCI device name. If unset, the script picks the new hciX.
  BT_ATTACH_LOG       btattach log path. Defaults to /tmp/go2_btattach_<tty>.log.
  BT_ATTACH_PIDFILE   btattach pid file. Defaults to /tmp/go2_btattach_<tty>.pid.
  BT_ATTACH_WAIT_SEC  Seconds to wait for hciX. Defaults to 8.

Output:
  Prints BT_TTY, BT_BAUD, BT_ATTACH_PID, BT_ATTACH_STARTED, BT_ATTACH_LOG, HCI_DEV.
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing command: $1" >&2
    exit 1
  fi
}

run_root() {
  if [[ "${EUID}" -eq 0 ]]; then
    "$@"
  elif [[ "${SUDO_NONINTERACTIVE:-0}" == "1" ]]; then
    sudo -n "$@"
  else
    sudo "$@"
  fi
}

start_btattach() {
  local tty="$1"
  local speed="$2"
  local output_log="$3"

  if [[ "${EUID}" -eq 0 ]]; then
    nohup btattach -B "$tty" -S "$speed" >"$output_log" 2>&1 &
  else
    nohup sudo -n btattach -B "$tty" -S "$speed" >"$output_log" 2>&1 &
  fi
  printf '%s\n' "$!"
}

list_hci_devices() {
  hciconfig 2>/dev/null | awk -F: '/^hci[0-9]+:/ {print $1}'
}

select_tty() {
  local requested="${BT_TTY:-${1:-}}"
  if [[ -n "$requested" ]]; then
    printf '%s\n' "$requested"
    return 0
  fi

  shopt -s nullglob
  local ports=(/dev/ttyACM*)
  shopt -u nullglob

  if [[ "${#ports[@]}" -eq 0 ]]; then
    echo "No /dev/ttyACM* device found. Plug in the Bluetooth board first." >&2
    exit 1
  fi

  printf '%s\n' "${ports[@]}" | sort -V | tail -n 1
}

find_existing_btattach() {
  local tty="$1"
  if [[ -f "$pidfile" ]]; then
    local pid
    pid="$(cat "$pidfile" 2>/dev/null || true)"
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      printf '%s\n' "$pid"
      return 0
    fi
  fi

  if command -v pgrep >/dev/null 2>&1; then
    pgrep -af btattach | awk -v tty="$tty" 'index($0, tty) {print $1; exit}'
  fi
}

choose_hci_device() {
  local before="$1"
  local after="$2"
  local requested="${HCI_DEV:-}"

  if [[ -n "$requested" ]] && printf '%s\n' "$after" | grep -qx "$requested"; then
    printf '%s\n' "$requested"
    return 0
  fi

  local dev
  while IFS= read -r dev; do
    [[ -z "$dev" ]] && continue
    if ! printf '%s\n' "$before" | grep -qx "$dev"; then
      printf '%s\n' "$dev"
      return 0
    fi
  done <<< "$after"

  printf '%s\n' "$after" | sort -V | tail -n 1
}

require_command btattach
require_command hciconfig
require_command awk
require_command sort

tty_dev="$(select_tty "${1:-}")"
if [[ ! -e "$tty_dev" ]]; then
  echo "Bluetooth UART device does not exist: $tty_dev" >&2
  exit 1
fi

baud="${BT_BAUD:-1000000}"
tty_name="$(basename "$tty_dev")"
log_path="${BT_ATTACH_LOG:-/tmp/go2_btattach_${tty_name}.log}"
pidfile="${BT_ATTACH_PIDFILE:-/tmp/go2_btattach_${tty_name}.pid}"
wait_sec="${BT_ATTACH_WAIT_SEC:-8}"

if ! [[ "$wait_sec" =~ ^[0-9]+$ ]]; then
  echo "BT_ATTACH_WAIT_SEC must be an integer." >&2
  exit 1
fi

before_hci="$(list_hci_devices || true)"
attach_started=0

if [[ "${EUID}" -ne 0 && "${SUDO_NONINTERACTIVE:-0}" != "1" ]]; then
  sudo -v
fi

attach_pid="$(find_existing_btattach "$tty_dev" || true)"
if [[ -n "$attach_pid" ]]; then
  echo "Reusing existing btattach pid $attach_pid for $tty_dev." >&2
else
  echo "Attaching Bluetooth controller from $tty_dev at $baud baud..." >&2
  attach_pid="$(start_btattach "$tty_dev" "$baud" "$log_path")"
  attach_started=1
  printf '%s\n' "$attach_pid" >"$pidfile"
fi

hci_dev=""
loops=$((wait_sec * 5))
for ((i = 0; i <= loops; ++i)); do
  after_hci="$(list_hci_devices || true)"
  hci_dev="$(choose_hci_device "$before_hci" "$after_hci" || true)"
  if [[ -n "$hci_dev" ]]; then
    break
  fi
  sleep 0.2
done

if [[ -z "$hci_dev" ]]; then
  echo "btattach did not expose an hci device within ${wait_sec}s." >&2
  echo "Check log: $log_path" >&2
  exit 1
fi

run_root hciconfig "$hci_dev" up

echo "BT_TTY=$tty_dev"
echo "BT_BAUD=$baud"
echo "BT_ATTACH_PID=$attach_pid"
echo "BT_ATTACH_STARTED=$attach_started"
echo "BT_ATTACH_LOG=$log_path"
echo "HCI_DEV=$hci_dev"
