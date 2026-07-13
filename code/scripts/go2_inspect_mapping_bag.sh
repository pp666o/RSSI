#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_inspect_mapping_bag.sh <bag_dir>

Print rosbag metadata, disk size, and whether the core Go2 mapping topics are present.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 2
fi

bag_dir="$1"
if [[ ! -d "$bag_dir" ]]; then
  echo "Bag directory does not exist: $bag_dir" >&2
  exit 2
fi

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "$setup_file"
  set -u
}

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  source_setup "/opt/ros/$ROS_DISTRO/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source_setup /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  source_setup /opt/ros/foxy/setup.bash
else
  echo "No ROS 2 setup.bash found under /opt/ros." >&2
  exit 2
fi

echo "Bag directory: $bag_dir"
echo "Disk usage:"
du -sh "$bag_dir"
echo

info="$(ros2 bag info "$bag_dir")"
printf '%s\n' "$info"
echo

required_topics=(
  /utlidar/cloud_deskewed
  /utlidar/robot_odom
  /utlidar/imu
  /tf
  /tf_static
)

echo "Required-topic check:"
status=0
for topic in "${required_topics[@]}"; do
  if grep -qE "^[[:space:]]+Topic:[[:space:]]+$topic[[:space:]]" <<<"$info"; then
    echo "  OK      $topic"
  else
    echo "  MISSING $topic"
    status=1
  fi
done

exit "$status"
