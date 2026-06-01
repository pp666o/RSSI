# Go2 Phase A: Odom-Controlled Straight-Line RSSI Collection

目标：先不上 RSSI 闭环控制，让 Go2 沿直线连续行走，同时采集 RSSI、Go2 IMU 和 Go2 sport state。路径距离以 Go2 `rt/sportmodestate` 中的位置估计为准，代码里记录为 `odom_s_m`；速度积分 `cmd_s_m` 只用于复盘，不作为真实标签。

## 1. 当前策略

- 不停下来采集 RSSI，不人工按 Enter 标记距离。
- 输入的 `distance_m` 是目标真实路径距离，runner 会在 `odom_s_m >= distance_m` 时停止；到达目标后脚本会立即停止后台 RSSI/IMU 采集并退出，不再等到 `MAX_RUNTIME_SEC`。
- `MAX_RUNTIME_SEC` 只是安全超时，防止异常情况下无限走，不应该用“输入更大的距离”凑真实距离。
- 已验证 `0.30 m/s` 比 `0.05-0.20 m/s` 更适合作为当前 baseline；低速命令可能让 Go2 实际只移动几厘米。
- RSSI 建库时先连续采集，后处理时按时间把每条 RSSI window 对齐到同一时刻的 `odom_s_m`，再按 0.5 m 或 1.0 m 分桶。

## 2. 场地和信标

- 选择平整、空旷、可安全直行的路线。先从 1 m、3 m、10 m 逐级验证，最后再跑 70 m。
- 在地面标出起点、终点，70 m 路线建议每 10 m 做肉眼可见标记，方便录像复盘。
- 先放 3-4 个信标：
  - 4 个信标推荐放在 `s=0, 23, 47, 70 m` 附近。
  - 3 个信标推荐放在 `s=0, 35, 70 m` 附近。
  - 信标尽量离地 0.8-1.5 m，避开人体遮挡、金属大物体和墙角强反射。
  - 每次采集不要移动信标；移动后要重新建库。

## 3. 同步和构建

从控制电脑同步到 Go2，不要同步整个项目目录：

```bash
cd /home/luping/桌面/RSSI/RSSI/code
scripts/sync_go2_phase_a.sh unitree@192.168.123.18 /home/unitree/rssi_go2
```

在 Go2 上构建：

```bash
cd ~/rssi_go2

# 如果 ~/rssi_go2/unitree_sdk2 还不存在，先准备一次完整 SDK：
# git clone https://github.com/unitreerobotics/unitree_sdk2.git unitree_sdk2

cmake -S unitree_sdk2 -B unitree_sdk2/build_go2
cmake --build unitree_sdk2/build_go2 --target go2_imu_logger go2_straight_line_runner -j"$(nproc)"

scripts/build_rssi_logger.sh
```

采集脚本会自动处理两件事：

- 自动把 `unitree_sdk2/thirdparty/lib/aarch64` 放到 `LD_LIBRARY_PATH` 前面，避免错误加载 `/usr/local/lib` 中的 CycloneDDS。
- 自动运行 `bt_migrate_pack/go2_attach_ttyacm.sh` attach 蓝牙控制器。只有你已经手动 attach 过 HCI 时，才需要加 `BT_ATTACH=0 HCI_DEV=hci0`。

确认 runner 是新版：

```bash
strings unitree_sdk2/build_go2/bin/go2_straight_line_runner | grep -E "Stop source|Max move runtime|odom-controlled"
```

应该能看到 `odom-controlled`、`Stop source` 或 `Max move runtime`。

## 4. 逐级实机测试

先跑 1 m，确认 `odom_s_m` 控制停止正常：

```bash
RUN_DIR="$HOME/go2_rssi_runs/test_1m_v030" MAX_RUNTIME_SEC=60 \
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
- `has_state=1`，`odom_s_m` 接近目标距离。
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
run_metadata.txt            本次采集参数
```

`sport_state_straight.csv` 关键列：

```text
cmd_s_m      第 8 列：速度命令积分，只用于复盘
odom_s_m     第 9 列：Go2 sport state 推算的一维路径距离，优先作为 s_gt
odom_x/y     第 10/11 列：Go2 位置估计
yaw_rad      第 16 列：朝向
has_state    第 22 列：是否收到 sport state
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
- 实机闭环前必须先做 shadow mode：实时输出 `s_pred`，但不接入运动控制。确认没有尖峰、长时间丢包和异常跳变后，再进入低速受限闭环。
