#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  scripts/sync_go2_phase_a.sh <user@robot_ip> [remote_dir]

Example:
  scripts/sync_go2_phase_a.sh unitree@192.168.123.18 /home/unitree/rssi_go2
  scripts/sync_go2_phase_a.sh unitree@192.168.123.18 '~/rssi_go2'

This syncs only the files needed for Go2 Phase A collection. It intentionally
does not upload paper/, .git/, local build directories, Python caches, or data.
It does not replace the robot's full unitree_sdk2 tree; it copies the custom
Go2 source files from go2_sdk/ into the robot SDK and patches
example/go2/CMakeLists.txt in place.
USAGE
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 1
fi

remote="$1"
remote_dir="${2:-~/rssi_go2}"
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "$remote_dir" == "$HOME/"* ]]; then
  cat >&2 <<EOF
remote_dir looks like a local path: $remote_dir

Your shell probably expanded ~/rssi_go2 before the script ran.
Use one of these instead:
  scripts/sync_go2_phase_a.sh $remote /home/unitree/rssi_go2
  scripts/sync_go2_phase_a.sh $remote '~/rssi_go2'
EOF
  exit 1
fi

required_paths=(
  "GO2_MIGRATION.md"
  "GO2_PHASE_A_STRAIGHT_LINE.md"
  "test_rssi.cpp"
  "scripts/build_rssi_logger.sh"
  "scripts/go2_collect_rssi_imu.sh"
  "scripts/go2_collect_straight_line_phase_a.sh"
  "scripts/sync_go2_phase_a.sh"
  "scripts/diagnose_go2_sdk_runtime.sh"
  "bt_migrate_pack/go2_attach_ttyacm.sh"
  "bt_migrate_pack/go2_run_rssi.sh"
  "bt_migrate_pack/start_coded_scan.sh"
  "bt_migrate_pack/test_rssi"
  "go2_sdk/go2_imu_logger.cpp"
  "go2_sdk/go2_straight_line_runner.cpp"
)

cd "$project_root"

for path in "${required_paths[@]}"; do
  if [[ ! -e "$path" ]]; then
    echo "Missing required path: $path" >&2
    exit 1
  fi
done

ssh "$remote" "mkdir -p $remote_dir"
rsync -avR "${required_paths[@]}" "$remote:$remote_dir/"
ssh "$remote" "REMOTE_DIR=$remote_dir bash -s" <<'REMOTE_PATCH'
set -euo pipefail
cmake_file="$REMOTE_DIR/unitree_sdk2/example/go2/CMakeLists.txt"
if [[ ! -f "$cmake_file" ]]; then
  echo "Missing robot SDK CMakeLists: $cmake_file" >&2
  echo "Prepare the full unitree_sdk2 tree on the robot before running this sync." >&2
  exit 1
fi
cp "$REMOTE_DIR/go2_sdk/go2_imu_logger.cpp" "$REMOTE_DIR/unitree_sdk2/example/go2/go2_imu_logger.cpp"
cp "$REMOTE_DIR/go2_sdk/go2_straight_line_runner.cpp" "$REMOTE_DIR/unitree_sdk2/example/go2/go2_straight_line_runner.cpp"
if ! grep -q '^add_executable(go2_imu_logger ' "$cmake_file"; then
  awk '
    { print }
    /^target_link_libraries\(go2_low_level unitree_sdk2\)/ {
      print ""
      print "add_executable(go2_imu_logger go2_imu_logger.cpp)"
      print "target_link_libraries(go2_imu_logger unitree_sdk2)"
      print ""
      print "add_executable(go2_straight_line_runner go2_straight_line_runner.cpp)"
      print "target_link_libraries(go2_straight_line_runner unitree_sdk2)"
    }
  ' "$cmake_file" > "$cmake_file.tmp"
  mv "$cmake_file.tmp" "$cmake_file"
elif ! grep -q '^add_executable(go2_straight_line_runner ' "$cmake_file"; then
  awk '
    { print }
    /^target_link_libraries\(go2_imu_logger unitree_sdk2\)/ {
      print ""
      print "add_executable(go2_straight_line_runner go2_straight_line_runner.cpp)"
      print "target_link_libraries(go2_straight_line_runner unitree_sdk2)"
    }
  ' "$cmake_file" > "$cmake_file.tmp"
  mv "$cmake_file.tmp" "$cmake_file"
fi
chmod +x "$REMOTE_DIR"/scripts/*.sh "$REMOTE_DIR"/bt_migrate_pack/*.sh "$REMOTE_DIR"/bt_migrate_pack/test_rssi 2>/dev/null || true
REMOTE_PATCH

cat <<EOF

Synced Phase A files to $remote:$remote_dir

Next commands on the robot:
  cd $remote_dir
  cmake -S unitree_sdk2 -B unitree_sdk2/build_go2
  cmake --build unitree_sdk2/build_go2 --target go2_imu_logger go2_straight_line_runner -j"\$(nproc)"
  scripts/build_rssi_logger.sh
  RUN_DIR="\$HOME/go2_rssi_runs/straight_test" scripts/go2_collect_straight_line_phase_a.sh eth0 0.5 0.05 1.0 0.5
EOF
