#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/luping/桌面/RSSI/RSSI/code/ros2_ws/install/setup.bash

echo "Topics:"
ros2 topic list | grep -E '^/(go2|odom|tf|utlidar|uslam)' | sort || true

echo
echo "/go2/pointcloud rate:"
timeout 6 ros2 topic hz /go2/pointcloud || true

echo
echo "/go2/imu rate:"
timeout 5 ros2 topic hz /go2/imu || true

echo
echo "/odom rate:"
timeout 5 ros2 topic hz /odom || true

echo
echo "One /go2/pointcloud message header:"
ros2 topic echo --once /go2/pointcloud | sed -n '1,45p'
