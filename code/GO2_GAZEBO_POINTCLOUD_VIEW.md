# Go2 点云在 Gazebo 中实时显示

本功能把 `go2_nav2_bridge` 发布的 ROS 2 点云 `/go2/pointcloud` 转成 Gazebo GUI 的彩色 Marker，在 Gazebo Harmonic 界面中实时显示。颜色按高度变化：低处偏青/绿，高处偏黄/橙/紫，接近 Unitree App 中的点云层高效果。

## 构建

```bash
cd /home/luping/桌面/RSSI/RSSI/code/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_nav2_bridge
source install/setup.bash
```

## 推荐启动方式

如果 Go2 bridge 已经在运行，并且已经发布 `/go2/pointcloud`：

```bash
ros2 launch go2_nav2_bridge go2_gazebo_pointcloud_view.launch.py
```

如果希望同时启动 Go2 bridge 和 Gazebo 可视化：

```bash
ros2 launch go2_nav2_bridge go2_gazebo_pointcloud_view.launch.py \
  start_go2_bridge:=true \
  network_interface:=eth0
```

如果实际连接 Go2 的网卡不是 `eth0`，把 `network_interface` 改成实际网卡名。

## 常用参数

```bash
ros2 launch go2_nav2_bridge go2_gazebo_pointcloud_view.launch.py \
  voxel_size_m:=0.06 \
  point_size_m:=0.07 \
  max_points:=18000 \
  max_range_m:=5.0
```

- `cloud_topic`: ROS 2 点云话题，默认 `/go2/pointcloud`。
- `voxel_size_m`: 显示前的体素降采样尺寸，越小越细，但更吃性能。
- `point_size_m`: Gazebo 中每个点的显示尺寸。
- `max_points`: 每帧最多显示点数。
- `max_range_m`: 只显示机器人周围多少米内的点。
- `marker_topic`: Gazebo Marker 话题，默认 `/marker`。

## 排查

确认 Go2 bridge 正在发布点云：

```bash
ros2 topic hz /go2/pointcloud
ros2 topic echo --once /go2/pointcloud
```

如果 Gazebo 打开但看不到点云，先确认启动 bridge 使用的是点云开启的参数文件，或者参数中有：

```yaml
publish_point_cloud: true
ros_point_cloud_topic: /go2/pointcloud
```

本功能只负责可视化真实 Go2 点云，不把这些点变成 Gazebo 物理碰撞体。Nav2 的避障仍然使用 ROS 2 侧的 `/go2/pointcloud`、costmap、collision monitor 等链路。
