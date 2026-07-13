#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/go2_record_mapping_bag.sh [output_bag_dir]

Record Go2 official topics for offline 3D mapping. Run this on the robot or on
the control PC if the official /utlidar and /tf topics are visible there.

Default output:
  /home/unitree/go2_mapping_bags/map_<timestamp>   when /home/unitree exists
  ~/go2_mapping_bags/map_<timestamp>              otherwise

Environment:
  STORAGE_ID   rosbag2 storage backend. Defaults to sqlite3.
  COMPRESSION  none, file, or message. Defaults to file.
  QOS_FILE     QoS override yaml. Defaults to project config when available.
  EXTRA_TOPICS Extra topics appended to the record command.

Recorded topics:
  /utlidar/cloud_deskewed
  /utlidar/cloud_base
  /utlidar/robot_odom
  /utlidar/robot_pose
  /utlidar/imu
  /tf
  /tf_static
  /utlidar/voxel_map
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
if [[ -d /home/unitree ]]; then
  default_root="/home/unitree/go2_mapping_bags"
else
  default_root="$HOME/go2_mapping_bags"
fi

bag_dir="${1:-$default_root/map_${timestamp}}"
storage_id="${STORAGE_ID:-sqlite3}"
compression="${COMPRESSION:-file}"
extra_topics="${EXTRA_TOPICS:-}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/.." && pwd)"
qos_file="${QOS_FILE:-$project_root/ros2_ws/src/go2_nav2_bridge/config/go2_mapping_bag_qos.yaml}"

topics=(
  /utlidar/cloud_deskewed
  /utlidar/cloud_base
  /utlidar/robot_odom
  /utlidar/robot_pose
  /utlidar/imu
  /tf
  /tf_static
  /utlidar/voxel_map
)

mkdir -p "$(dirname "$bag_dir")"

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

echo "Recording Go2 mapping bag:"
echo "  output:      $bag_dir"
echo "  storage:     $storage_id"
echo "  compression: $compression"
if [[ -f "$qos_file" ]]; then
  echo "  qos file:    $qos_file"
else
  echo "  qos file:    none"
fi
echo "  topics:"
printf '    %s\n' "${topics[@]}"
if [[ -n "$extra_topics" ]]; then
  echo "  extra topics: $extra_topics"
fi
echo
echo "Press Ctrl+C once to stop recording cleanly."

cmd=(ros2 bag record -o "$bag_dir" --storage "$storage_id")
if [[ -f "$qos_file" ]]; then
  cmd+=(--qos-profile-overrides-path "$qos_file")
fi
case "$compression" in
  none)
    ;;
  file|message)
    cmd+=(--compression-mode "$compression" --compression-format zstd)
    ;;
  *)
    echo "Unsupported COMPRESSION=$compression. Use none, file, or message." >&2
    exit 2
    ;;
esac

cmd+=("${topics[@]}")
if [[ -n "$extra_topics" ]]; then
  # shellcheck disable=SC2206
  extra_array=($extra_topics)
  cmd+=("${extra_array[@]}")
fi

exec "${cmd[@]}"
