# Go2 Phase A: Cumulative-Odom Straight-Line RSSI Collection

目标：先不上 RSSI 闭环控制，让 Go2 沿直线连续行走，同时采集 RSSI、Go2 IMU 和 Go2 sport state。路径距离以 Go2 `rt/sportmodestate` 中的位置估计为准，代码里记录为 `odom_s_m`；速度积分 `cmd_s_m` 只用于复盘，不作为真实标签。
## 代码同步
```bash
cd /home/luping/桌面/RSSI/RSSI/code

scp unitree_sdk2/example/go2/go2_straight_line_runner.cpp \
  unitree@192.168.123.18:/home/unitree/rssi_go2/unitree_sdk2/example/go2/go2_straight_line_runner.cpp

scp scripts/go2_collect_straight_line_phase_a.sh \
  unitree@192.168.123.18:/home/unitree/rssi_go2/scripts/go2_collect_straight_line_phase_a.sh

scp GO2_PHASE_A_STRAIGHT_LINE.md \
  unitree@192.168.123.18:/home/unitree/rssi_go2/GO2_PHASE_A_STRAIGHT_LINE.md
```
- Go2上重新编译
```bash
cd ~/rssi_go2
touch unitree_sdk2/example/go2/go2_straight_line_runner.cpp
cmake -S unitree_sdk2 -B unitree_sdk2/build_go2
cmake --build unitree_sdk2/build_go2 --target go2_straight_line_runner -j"$(nproc)"
```
- 验证
```bash
RUN_DIR="$HOME/go2_rssi_runs/test_1m_v030_hold" MAX_RUNTIME_SEC=60 \
scripts/go2_collect_straight_line_phase_a.sh eth0 1.0 0.30 1.0 0.5
```
- 删除文件
```bash
rm -rf ~/go2_rssi_runs/*
```
## 1. 当前策略

- 不停下来采集 RSSI，不人工按 Enter 标记距离。
- 输入的 `distance_m` 是目标真实路径距离，runner 会在 `odom_s_m >= distance_m` 时停止；到达目标后脚本会立即停止后台 RSSI/IMU 采集并退出，不再等到 `MAX_RUNTIME_SEC`。
- `MAX_RUNTIME_SEC` 只是安全超时，防止异常情况下无限走，不应该用“输入更大的距离”凑真实距离。
- 已验证 `0.30 m/s` 比 `0.05-0.20 m/s` 更适合作为当前 baseline；低速命令可能让 Go2 实际只移动几厘米。
- 运动命令默认走 `SportClient::Move(vx, 0, 0)` 连续速度模式。实测在走廊里 `ObstaclesAvoidClient` 会因为两侧墙或近场障碍过度保守，导致 1 m 测试走不到目标；而关闭官方避障、使用 sport 速度命令可以正常到达 1 m。
- 官方 `obstacles_avoid` client 仍保留为对照测试选项：显式设置 `USE_OBSTACLE_AVOID=1` 时，runner 会调用 `UseRemoteCommandFromApi(true)`、`SwitchSet(true)`，并通过 `ObstaclesAvoidClient::Move(vx, 0, 0)` 发送速度命令。
- 默认运动模式是 `MOVE_MODE=velocity`，即持续发送速度命令，并用 `odom_s_m` 到目标距离后停止。这样不会反复重置 `MoveToIncrementPosition()` 目标，能减少走走停停。
- 如果需要单独调试官方增量位姿命令，可以显式设置 `MOVE_MODE=increment`，即调用 `ObstaclesAvoidClient::MoveToIncrementPosition(remaining_distance, 0, 0)`。
- 本版不追求走廊中线控制，只做局部防撞和局部绕障：`sport` 接口负责稳定前进，runner 额外订阅点云并检查机器人正前方安全区。如果正前方近距离有障碍，runner 会检查左前、右前两条“候选通道”是否足够宽、足够远，再发送小幅 `vy` 横移和 `vyaw` 转向命令。只有左右候选通道都不安全时才原地 hold，默认 hold 2 秒后安全结束本次运行。
- 点云默认订阅 `POINT_CLOUD_TOPIC=rt/utlidar/cloud`，按 Go2/ROS 常见坐标约定理解为 `x` 向前、`y` 左右、`z` 高度。安全区默认是前方 `0.25-1.50 m`、左右 `±0.60 m`、高度 `-0.35-0.90 m`；正前方中心通道默认看 `±0.25 m`。`0.25 m` 以内的点会被忽略，用来过滤机身、腿部、地面近场噪声。正前方中心通道默认至少需要 `POINT_CLOUD_MIN_CENTER_POINTS=4` 个点才算障碍，避免单个噪声点导致抖动。
- 左右绕行不是再简单比较“哪边最近点更远”，而是检查以 `AVOID_SIDE_OFFSET_M=0.25 m` 为中心、半宽 `AVOID_CORRIDOR_HALF_WIDTH_M=0.18 m` 的左/右候选通道；候选通道必须在 `AVOID_OPEN_M=0.85 m` 以内没有障碍，才允许绕行。这样可以拒绝 5 cm 门缝、墙檐边缘这种“看起来有缝但机身过不去”的局部空隙。
- 如果实测“左边更空但机器狗往右走”，优先设置 `POINT_CLOUD_Y_SIGN=-1` 重新测试；这表示点云左右坐标或控制方向和代码假设相反，不应该继续靠墙距参数硬调。
- 绕障方向默认保持 `AVOID_DIRECTION_HOLD_SEC=0.8` 秒，避免点云轻微抖动时左/右方向来回切换。`vx/vy/vyaw` 也做了斜率限制，防止速度命令突变造成机身抖动。
- 走廊侧墙现在单独处理：如果左侧或右侧点云墙面距离低于 `WALL_AVOID_M=0.10 m`，runner 会给一个反方向的 `vy/vyaw` 修正，目标是保持约 `WALL_TARGET_M=0.20 m` 的侧向安全距离。这样能减少侧墙排斥过强导致的左右扭动，但也会更晚修正侧墙风险。
- 前置摄像头默认启用诊断采样，图片保存到 `RUN_DIR/front_images`。普通 RGB 图像没有直接距离信息，所以本版视觉不直接做控制，只用于复盘：看当时前方是否有墙、行人、玻璃门、强反光等。
- `odom_s_m` 是运动开始后 Go2 `position()[x,y]` 增量累计得到的二维路径长度，不再是投影到初始 yaw 的距离。这样避障绕行、横向偏移或 position/yaw 坐标系不完全一致时，也能按实际累计行走距离停止。
- RSSI 建库时先连续采集，后处理时按时间把每条 RSSI window 对齐到同一时刻的 `odom_s_m`，再按 0.5 m 或 1.0 m 分桶。

## 1.1 runner 算法逻辑

可以把现在的 runner 理解成四层：

1. 数据输入层：同时收 Go2 `rt/sportmodestate`、点云 `PointCloud2`、前置摄像头图像。`sportmodestate` 提供位置、速度、航向角和官方 `range_obstacle`；点云提供前方局部空间里的障碍点；摄像头只保存画面用于复盘。
2. 距离计数层：运动开始前记录初始位置，运动中用 Go2 估计位置的 `x/y` 增量累计出 `odom_s_m`。这个值表示机器狗实际走过的路径长度，不要求它完全沿直线或走在走廊中间。
3. 局部避障层：每个控制周期都算一次 `SafetyDecision`。点云会被分成正前方阻挡区、左候选通道、右候选通道。正前方清楚就正常走；正前方变近就降速；正前方有障碍且某一侧候选通道足够空，就给 `vy` 和 `vyaw`，让机器狗边前进边绕；两侧候选通道都不安全时，原地 hold 等待通道打开；如果点云不可用，再参考 Go2 sport state 里的 `range_obstacle` 兜底停车。
4. 运动输出层：默认走 `SportClient::Move(vx,vy,vyaw)`，按 `CONTROL_HZ` 连续发送速度命令。官方避障接口只作为 `USE_OBSTACLE_AVOID=1` 的对照选项。到达 `distance_m`、超时、收到 Ctrl+C、或障碍持续挡住超过 `BLOCKED_HOLD_TIMEOUT_SEC` 都会发送停止命令。默认 `BLOCKED_HOLD_TIMEOUT_SEC=2.0`，因为当前场地障碍基本是静态的，长时间等待没有收益。

## 2. 场地和信标

- 选择平整、空旷、可安全直行的路线。先从 1 m、3 m、10 m 逐级验证，最后再跑 70 m。
- 在地面标出起点、终点，70 m 路线建议每 10 m 做肉眼可见标记，方便录像复盘。
- 先放 3-4 个信标：
  - 4 个信标推荐放在 `s=0, 23, 47, 70 m` 附近。
  - 3 个信标推荐放在 `s=0, 35, 70 m` 附近。
  - 信标尽量离地 0.8-1.5 m，避开人体遮挡、金属大物体和墙角强反射。
  - 每次采集不要移动信标；移动后要重新建库。

## 3. 同步和构建

从控制电脑同步到 Go2，不要同步整个项目目录。因为 Go2 系统时间经常不准，`rsync` 可能根据时间戳误判“不需要覆盖”。当前建议直接强制 `scp` 关键文件：

```bash
cd /home/luping/桌面/RSSI/RSSI/code

scp go2_sdk/go2_straight_line_runner.cpp \
  unitree@192.168.123.18:/home/unitree/rssi_go2/unitree_sdk2/example/go2/go2_straight_line_runner.cpp

scp go2_sdk/go2_imu_logger.cpp \
  unitree@192.168.123.18:/home/unitree/rssi_go2/unitree_sdk2/example/go2/go2_imu_logger.cpp

scp scripts/go2_collect_straight_line_phase_a.sh \
  unitree@192.168.123.18:/home/unitree/rssi_go2/scripts/go2_collect_straight_line_phase_a.sh

scp GO2_PHASE_A_STRAIGHT_LINE.md \
  unitree@192.168.123.18:/home/unitree/rssi_go2/GO2_PHASE_A_STRAIGHT_LINE.md
```

在 Go2 上构建：

```bash
cd ~/rssi_go2

cmake -S unitree_sdk2 -B unitree_sdk2/build_go2
touch unitree_sdk2/example/go2/go2_straight_line_runner.cpp
cmake --build unitree_sdk2/build_go2 --target go2_imu_logger go2_straight_line_runner -j"$(nproc)"

scripts/build_rssi_logger.sh
```

采集脚本会自动处理两件事：

- 自动把 `unitree_sdk2/thirdparty/lib/aarch64` 放到 `LD_LIBRARY_PATH` 前面，避免错误加载 `/usr/local/lib` 中的 CycloneDDS。
- 自动运行 `bt_migrate_pack/go2_attach_ttyacm.sh` attach 蓝牙控制器。只有你已经手动 attach 过 HCI 时，才需要加 `BT_ATTACH=0 HCI_DEV=hci0`。

确认 Go2 上源码是新版：

```bash
grep -n "ObstaclesAvoid" unitree_sdk2/example/go2/go2_straight_line_runner.cpp
grep -n "ResetPathAccumulator" unitree_sdk2/example/go2/go2_straight_line_runner.cpp
grep -n "USE_OBSTACLE_AVOID" scripts/go2_collect_straight_line_phase_a.sh
grep -n "MOVE_MODE" scripts/go2_collect_straight_line_phase_a.sh
grep -n "POINT_CLOUD_TOPIC" scripts/go2_collect_straight_line_phase_a.sh
grep -n "OBSTACLE_STOP_M" scripts/go2_collect_straight_line_phase_a.sh
grep -n "ENABLE_VIDEO" scripts/go2_collect_straight_line_phase_a.sh
grep -n "exec sudo" scripts/go2_collect_straight_line_phase_a.sh
```

确认 runner 二进制是新版：

```bash
strings unitree_sdk2/build_go2/bin/go2_straight_line_runner | grep -E "Stop source|Max move runtime|obstacles_avoid|velocity|Local safety|Point cloud|Front video"
```

直接运行一次不带参数也可以检查 Usage。新版 Usage 必须包含 `[enable_point_cloud] [point_cloud_topic] ... [enable_video] [video_interval_sec] [video_dir]`：

```bash
./unitree_sdk2/build_go2/bin/go2_straight_line_runner
```

## 4. 逐级实机测试

先跑 1 m，确认 `odom_s_m` 控制停止正常：

```bash
RUN_DIR="$HOME/go2_rssi_runs/test_1m_v030" MAX_RUNTIME_SEC=60 \
scripts/go2_collect_straight_line_phase_a.sh eth0 1.0 0.30 1.0 0.5
```

默认参数使用 `sport` 连续速度命令，同时启用点云局部绕障和前置摄像头诊断。如果实机点云 topic 不是默认值，可以显式指定：

```bash
POINT_CLOUD_TOPIC="rt/utlidar/cloud" \
RUN_DIR="$HOME/go2_rssi_runs/test_1m_pc" MAX_RUNTIME_SEC=60 \
scripts/go2_collect_straight_line_phase_a.sh eth0 1.0 0.30 1.0 0.5
```

如果点云 topic 暂时没有确认，可以先关闭点云 gate，只验证官方避障接口和摄像头采样：

```bash
ENABLE_POINT_CLOUD=0 RUN_DIR="$HOME/go2_rssi_runs/test_1m_no_pc" MAX_RUNTIME_SEC=60 \
scripts/go2_collect_straight_line_phase_a.sh eth0 1.0 0.30 1.0 0.5
```

默认使用 `sport` 速度命令配合本地点云防撞。如果要对照测试官方避障接口，可以显式打开 `USE_OBSTACLE_AVOID=1`：

```bash
USE_OBSTACLE_AVOID=1 RUN_DIR="$HOME/go2_rssi_runs/debug_official_avoid" MAX_RUNTIME_SEC=60 \
scripts/go2_collect_straight_line_phase_a.sh eth0 1.0 0.30 1.0 0.5
```

如果要单独验证官方增量位姿命令，可以显式打开 increment 模式。这个模式现在只作为调试选项，不作为默认采集路径：

```bash
MOVE_MODE=increment INCREMENT_STEP_M=0 RUN_DIR="$HOME/go2_rssi_runs/test_1m_increment" MAX_RUNTIME_SEC=60 \
scripts/go2_collect_straight_line_phase_a.sh eth0 1.0 0.30 1.0 0.5
```

默认已经是连续速度模式，也可以显式写出参数：

```bash
MOVE_MODE=velocity RUN_DIR="$HOME/go2_rssi_runs/test_1m_velocity" MAX_RUNTIME_SEC=60 \
scripts/go2_collect_straight_line_phase_a.sh eth0 1.0 0.30 1.0 0.5
```

检查结果：

```bash
RUN_DIR="$HOME/go2_rssi_runs/test_1m_v030"

grep target_reached "$RUN_DIR/sport_state_straight.csv" || true

awk -F, 'NR>1 && $22==1 && $9!="nan"{
  if($9>max) max=$9
  last=$9
}
END{
  print "max odom_s_m =", max
  print "last odom_s_m =", last
}' "$RUN_DIR/sport_state_straight.csv"

cat "$RUN_DIR/run_metadata.txt"
```

通过标准：

- 终端日志出现 `Target distance reached.`，或 CSV 中有 `target_reached`。
- `has_state=1`，`odom_s_m` 是累计路径长度，终点应接近目标距离。
- 默认终端日志出现 `Motion client: sport`。如果显式设置 `USE_OBSTACLE_AVOID=1`，才会出现 `Motion client: obstacles_avoid`，并打印 `ObstaclesAvoid SwitchSet(true)`。
- 默认终端日志应出现 `Motion mode: velocity`。CSV 中 `motion_mode` 应为 `velocity`。
- 终端日志应出现 `Local safety: stop=... slow=... point_cloud=...`。如果点云收到并解析成功，CSV 里 `point_cloud_has_cloud=1`、`point_cloud_parse_ok=1`、`point_cloud_fresh=1`。
- 如果前方安全区有近距离障碍，CSV 的 `safety_state` 会从 `clear` 变为 `slow` 或 `stop`；`cmd_vx_mps` 会降低。如果左/右候选通道有一侧可走，`phase` 会出现 `move_local_avoidance`，并记录 `safety_steer_side=left/right`；如果两侧都不可走，`phase` 会出现 `blocked_hold`，默认最多等待 2 秒。
- `RUN_DIR/front_images` 应有前置摄像头抓拍图片；如果没有，需要看 `video_last_ret` 是否为 0。
- 终端应每秒打印一次 `Progress: odom_s_m=... / target=...`，用于直接观察当前累计里程。
- 默认速度模式不应反复打印 `Sent official increment command`。如果看到这类日志，说明 Go2 上仍是旧脚本/旧二进制，或显式设置了 `MOVE_MODE=increment`。
- 到达目标后脚本应立即打印 `Go2 runner finished; stopping RSSI/IMU background collectors.` 并退出。如果仍等到 `collection_duration`，说明 Go2 上脚本还是旧版。
- RSSI 文件不是长期全 `-999`；至少能看到已放置信标的 `beacon_*_count > 0`。

1 m 通过后再跑 3 m：

```bash
RUN_DIR="$HOME/go2_rssi_runs/test_3m_v030" MAX_RUNTIME_SEC=120 \
scripts/go2_collect_straight_line_phase_a.sh eth0 3.0 0.30 1.0 0.5
```

3 m 通过后再跑 10 m：

```bash
RUN_DIR="$HOME/go2_rssi_runs/test_10m_v030" MAX_RUNTIME_SEC=300 \
scripts/go2_collect_straight_line_phase_a.sh eth0 10.0 0.30 1.0 0.5
```

最后再跑 70 m：

```bash
RUN_DIR="$HOME/go2_rssi_runs/straight_70m_001" MAX_RUNTIME_SEC=1800 \
scripts/go2_collect_straight_line_phase_a.sh eth0 70.0 0.30 1.0 0.5
```

运行中可以 `Ctrl+C`，runner 会发送 `StopMove()`。正常到达目标后，终端应看到 `Target distance reached.` 和 `Straight-line runner finished.`，随后脚本会停止 RSSI/IMU 后台采集并写出输出文件。实机旁边仍必须有人能物理接管或急停。

## 5. 输出文件

每次运行输出到 `RUN_DIR`：

```text
rssi_realtime_windows.csv   RSSI 窗口均值/方差/count
rssi_raw_stream.csv         原始 RSSI 包
rssi_markers.csv            兼容旧流程的 marker 文件，本阶段不用人工 marker
imu_stream.csv              Go2 lowstate IMU
sport_state_straight.csv    Go2 sport state + cmd_s_m + odom_s_m
front_images/               前置摄像头诊断图片
run_metadata.txt            本次采集参数
```

`sport_state_straight.csv` 关键列：

```text
cmd_s_m      第 8 列：速度命令积分，只用于复盘
odom_s_m     第 9 列：Go2 sport state 的 xy 增量累计路径长度，优先作为 s_gt
odom_x/y     第 10/11 列：Go2 位置估计
yaw_rad      第 16 列：朝向
has_state    第 22 列：是否收到 sport state
command_ret  第 23 列：最近一次运动命令返回码
motion_client 第 24 列：运动通道，默认 sport
range_obstacle_min_m 后续列：Go2 sport state 中的最近障碍距离，仅用于诊断
motion_mode 后续列：官方运动模式，默认 velocity
safety_state 后续列：局部防撞状态，可能是 clear/slow/stop/no_local_sensor
safety_source 后续列：触发局部防撞的来源，可能是 point_cloud 或 range_obstacle
safety_obstacle_m 后续列：当前局部防撞看到的最近障碍距离
safety_speed_scale 后续列：速度缩放，1 表示正常速度，0 表示停止
safety_steer_side 后续列：局部绕障方向，可能是 left/right/none
safety_vy_mps / safety_vyaw_radps 后续列：局部绕障实际发出的横向速度和转向角速度
point_cloud_* 后续列：点云是否收到、是否解析成功、是否新鲜、左/中/右安全区最近距离和点数
point_cloud_min_left_path_m / point_cloud_min_right_path_m 后续列：左/右候选绕行通道里的最近障碍距离，用来判断是否真的能绕过去
video_* 后续列：摄像头是否启用、已保存图片数、最近一次返回码、最近图片路径
```

快速检查：

```bash
RUN_DIR="$HOME/go2_rssi_runs/test_3m_v030"

head -n 3 "$RUN_DIR/rssi_realtime_windows.csv"
head -n 3 "$RUN_DIR/sport_state_straight.csv"
tail -n 3 "$RUN_DIR/sport_state_straight.csv"
head -n 3 "$RUN_DIR/imu_stream.csv"
cut -d, -f4,5 "$RUN_DIR/rssi_raw_stream.csv" | sort | uniq -c
```

如果要清空旧采集数据：

```bash
rm -rf ~/go2_rssi_runs/*
```

## 6. RSSI 如何得到位置标签

RSSI 和 Go2 位置由不同程序记录，所以不会天然在同一个 CSV 里。离线处理时，对每一行 `rssi_realtime_windows.csv`，用时间戳去 `sport_state_straight.csv` 找最近的一行，把那一行的 `odom_s_m` 复制过来。

示意：

```text
rssi_realtime_windows.csv:
elapsed_sec=12.50, beacon_6_mean=-63

sport_state_straight.csv:
elapsed_sec=12.40, odom_s_m=0.281
elapsed_sec=12.50, odom_s_m=0.284
elapsed_sec=12.60, odom_s_m=0.287

合并后:
elapsed_sec=12.50, s=0.284, beacon_6_mean=-63
```

这一步得到训练用的连续标签：

```text
rssi_with_s.csv:
timestamp_unix,elapsed_sec,s,beacon_1_mean,...,beacon_10_mean
...
```

建指纹库时再按 `s` 分桶，例如每 0.5 m 一个 bin：

```text
s=0.0~0.5 m
s=0.5~1.0 m
s=1.0~1.5 m
```

分桶不是必须的控制逻辑，而是最简单 baseline 的统计方法。连续采集时每条 RSSI 都有一个连续 `s` 标签，但 RSSI 本身会有尖峰、丢包和多径噪声；把同一小段距离内多次经过或多个时间窗口的 RSSI 聚合起来，可以得到更稳定的均值、方差和 count。在线定位时仍然可以每个 RSSI window 都输出一次 `s_pred`，不会被 0.5 m bin 的采集方式限制。

在线定位时不要每 0.5 m 才输出一次。在线流程应该是每个实时 RSSI window 都输出一个 `s_pred`，再做时间平滑和最大速度限制：

```text
RSSI window -> s_pred -> s_filtered -> 当前路径位置和下一个目标点
```

## 7. 采集建议

- 同一条路线至少采 3 次，最好正向 3-5 次。
- 第一版先不要混入反向数据；反向会引入朝向、机体遮挡和人体遮挡差异。
- 每次起点、朝向、速度、信标位置尽量一致。
- 采集时人不要贴着机器人走在信标和机器人之间。
- 如果 `odom_s_m` 不单调或终点偏差很大，先排查 Go2 sport state 和行走模式，不要训练 RSSI 闭环。
- 当前 `odom_s_m` 是累计路径长度，理论上应单调增长；如果目标距离明显不符，先确认 Go2 上源码和二进制都是新版。
- 官方避障只能降低风险，不能保证避开所有障碍。70 m 测试仍要保持开阔场地、低速、旁边有人能接管或急停。
- 实机闭环前必须先做 shadow mode：实时输出 `s_pred`，但不接入运动控制。确认没有尖峰、长时间丢包和异常跳变后，再进入低速受限闭环。
