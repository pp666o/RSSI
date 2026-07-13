#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_push_replay_route.sh <route_file> [remote_dir]

Run this on the control computer. It copies a processed replay route to the
Go2 computer and prints the one-command replay line.

Environment:
  GO2_HOST   Defaults to unitree@192.168.123.18.
EOF
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 1
fi

route_file="$(realpath "$1")"
go2_host="${GO2_HOST:-unitree@192.168.123.18}"
remote_dir="${2:-/home/unitree/go2_routes}"
ssh "$go2_host" "mkdir -p '$remote_dir'"
rsync -avP "$route_file" "$go2_host:$remote_dir/"
remote_file="$remote_dir/$(basename "$route_file")"

echo "Pushed replay route."
echo "  local_file:  $route_file"
echo "  remote_file: $go2_host:$remote_file"
echo
echo "Run on the Go2:"
echo "  cd /home/unitree/rssi_go2"
echo "  XY_TOLERANCE_M=0.25 scripts/go2_robot_replay_route.sh \"$remote_file\""
