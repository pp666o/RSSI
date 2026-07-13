#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_pull_latest_route_capture.sh [local_parent_dir]

Run this on the control computer. It pulls the latest Go2 route capture
recorded by scripts/go2_robot_capture_route_raw.sh.

Environment:
  GO2_HOST   Defaults to unitree@192.168.123.18.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

go2_host="${GO2_HOST:-unitree@192.168.123.18}"
local_parent="${1:-$HOME/go2_route_captures}"
remote_dir="$(ssh "$go2_host" 'cat "$HOME/go2_route_captures/latest_route_capture.txt"')"
if [[ -z "$remote_dir" ]]; then
  echo "No latest route capture recorded on $go2_host." >&2
  exit 2
fi

mkdir -p "$local_parent"
local_dir="$local_parent/$(basename "$remote_dir")"
rsync -avP "$go2_host:$remote_dir/" "$local_dir/"

echo "Pulled latest route capture."
echo "  remote_dir: $go2_host:$remote_dir"
echo "  local_dir:  $local_dir"
echo
echo "Process it with:"
echo "  scripts/go2_process_route_capture.sh \"$local_dir\""
