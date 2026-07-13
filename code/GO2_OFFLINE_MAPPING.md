# Go2 Offline Mapping Workflow

This workflow keeps the robot light:

1. Record official Go2 topics to a rosbag.
2. Copy the bag to the control PC.
3. Run RTAB-Map offline on the control PC.

The robot does not run RTAB-Map, RViz, or any walking controller in this workflow.

## 1. Record A Mapping Bag

Run on the robot, or on the control PC if the official `/utlidar/*` topics are visible there:

```bash
cd /home/unitree/rssi_go2
scripts/go2_record_mapping_bag.sh /home/unitree/go2_mapping_bags/corridor_001
```

Stop recording with one `Ctrl+C`.

Recorded topics:

- `/utlidar/cloud_deskewed`
- `/utlidar/cloud_base`
- `/utlidar/robot_odom`
- `/utlidar/robot_pose`
- `/utlidar/imu`
- `/tf`
- `/tf_static`
- `/utlidar/voxel_map`

The main RTAB-Map input is `/utlidar/cloud_deskewed`. `/utlidar/voxel_map` is kept for RViz replay and diagnostics.

## 2. Inspect The Bag

```bash
cd /home/unitree/rssi_go2
scripts/go2_inspect_mapping_bag.sh /home/unitree/go2_mapping_bags/corridor_001
```

The required topics for offline mapping are:

- `/utlidar/cloud_deskewed`
- `/utlidar/robot_odom`
- `/utlidar/imu`
- `/tf`
- `/tf_static`

## 3. Copy To Control PC

From the control PC:

```bash
mkdir -p /home/luping/go2_mapping_bags
rsync -avP unitree@192.168.123.18:/home/unitree/go2_mapping_bags/corridor_001 /home/luping/go2_mapping_bags/
```

## 4. Run Offline RTAB-Map

On the control PC:

```bash
cd /home/luping/桌面/RSSI/RSSI/code
RUN_DIR=/home/luping/go2_rssi_runs/corridor_001_map \
scripts/go2_offline_rtabmap_from_bag.sh /home/luping/go2_mapping_bags/corridor_001
```

Outputs:

- `/home/luping/go2_rssi_runs/corridor_001_map/go2_offline_mapping.db`
- `/home/luping/go2_rssi_runs/corridor_001_map/rtabmap_launch.log`
- `/home/luping/go2_rssi_runs/corridor_001_map/bag_play.log`

If `/utlidar/cloud_deskewed` is not good enough, test another cloud topic:

```bash
CLOUD_TOPIC=/utlidar/cloud_base \
RUN_DIR=/home/luping/go2_rssi_runs/corridor_001_map_cloud_base \
scripts/go2_offline_rtabmap_from_bag.sh /home/luping/go2_mapping_bags/corridor_001
```

## Memory And Disk Estimate

For a 20-100 m corridor mapping bag:

- Minimum usable RAM: 8 GB
- Recommended RAM: 16 GB
- Comfortable RAM for repeated tests: 32 GB
- Expected compressed bag size: about 0.5-3 GB for a few minutes, depending on point-cloud rate and voxel topic size
- Expected RTAB-Map database: hundreds of MB to a few GB

For longer runs, prefer:

```bash
PLAY_RATE=0.3 scripts/go2_offline_rtabmap_from_bag.sh <bag_dir>
```

Lower playback rate reduces CPU/RAM spikes and improves synchronization stability.

## Notes

- Use `/utlidar/voxel_map` for visualization and diagnostics, not as the primary SLAM input.
- Use `/utlidar/cloud_deskewed` or `/utlidar/cloud_base` for RTAB-Map scan-cloud input.
- If RTAB-Map reports TF errors, inspect whether the bag contains `/tf`, `/tf_static`, and `odom -> base_link`.
