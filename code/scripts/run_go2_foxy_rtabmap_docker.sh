#!/usr/bin/env bash
set -euo pipefail

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is not installed on this control PC." >&2
  echo "Install Docker first, then rerun this script." >&2
  exit 1
fi

image="${GO2_FOXY_RTABMAP_IMAGE:-go2-foxy-rtabmap:local}"
project_root="/home/luping/桌面/RSSI/RSSI/code"

if ! docker image inspect "$image" >/dev/null 2>&1; then
  docker build -t "$image" -f "$project_root/docker/go2_foxy_rtabmap.Dockerfile" "$project_root"
fi

xhost +local:docker >/dev/null

docker run --rm -it \
  --net=host \
  --ipc=host \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$project_root":/workspace/code:rw \
  -v /home/luping/go2_rssi_runs:/root/go2_rssi_runs:rw \
  "$image"
