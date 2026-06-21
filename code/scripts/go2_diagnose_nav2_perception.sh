#!/usr/bin/env bash
set -euo pipefail

network_interface="${1:-enp5s0}"
duration_sec="${2:-12}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${RUN_DIR:-$HOME/go2_rssi_runs/nav2_perception_diag_$(date +%Y%m%d_%H%M%S)}"

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

if [[ -f "${ROS_SETUP:-/opt/ros/humble/setup.bash}" ]]; then
  source_setup "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
fi
if [[ -f "${NAV2_WS_SETUP:-$project_root/ros2_ws/install/setup.bash}" ]]; then
  source_setup "${NAV2_WS_SETUP:-$project_root/ros2_ws/install/setup.bash}"
fi

sdk_lib_dir="$project_root/unitree_sdk2/thirdparty/lib/x86_64"
if [[ -d "$sdk_lib_dir" ]]; then
  export LD_LIBRARY_PATH="$sdk_lib_dir:${LD_LIBRARY_PATH:-}"
fi

params_file="${NAV2_PARAMS_FILE:-$(ros2 pkg prefix go2_nav2_bridge)/share/go2_nav2_bridge/params/go2_nav2_straight_params.yaml}"
mkdir -p "$run_dir"
echo "Output directory: $run_dir"
echo "Starting no-motion Nav2 perception diagnosis for ${duration_sec}s"
echo "params_file=$params_file"
echo "network_interface=$network_interface"

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

ros2 launch go2_nav2_bridge go2_nav2_straight.launch.py \
  params_file:="$params_file" \
  move_backend:=nav2 \
  use_bt_navigator:=false \
  use_planner:=false \
  network_interface:="$network_interface" \
  >"$run_dir/nav2_launch.log" 2>&1 &
launch_pid=$!

wait_for_nav2_active() {
  local deadline=$((SECONDS + 60))
  echo "Waiting for Nav2 managed nodes active..."
  while (( SECONDS < deadline )); do
    if grep -q "Managed nodes are active" "$run_dir/nav2_launch.log" 2>/dev/null; then
      echo "Nav2 managed nodes active"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for Nav2 managed nodes to become active." >&2
  return 1
}

wait_for_nav2_active

python3 "$project_root/scripts/go2_nav2_perception_diag.py" --duration "$duration_sec" \
  | tee "$run_dir/perception_diag.txt"

echo
echo "Recent Nav2 perception logs:"
grep -E "pointcloud_roi|pointcloud_self_filter|Robot to|ERROR|WARN" "$run_dir/nav2_launch.log" \
  | tail -n 80 || true
echo "Done. Output directory: $run_dir"
