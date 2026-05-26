# Go2 RSSI + IMU Migration

目标是把原蓝牙板子的两个功能迁到机器狗电脑：

- RSSI：先复用 `bt_migrate_pack/test_rssi`，它会启动 `btmon` 解析 BLE 广播 RSSI。
- IMU：不用旧板子的串口 IMU，改成读取 Go2 SDK 的 `rt/lowstate`，输出兼容 `robot_s_baseline.py` 的 `imu_stream.csv`。

## 0. 原板子的启动链路和迁移结论

原板子的 RSSI 链路分成两层：

1. 旧板子专用硬件初始化：

```bash
ready_testmode.sh
```

它做的是：

```bash
cp /home/w20x/wte20x_scpi_call_set.json /wte/
cp /home/w20x/cal_data_v2/* /wte/
insmod /lib/modules/4.14.98-wte_bt+g5d6cbea/kernel/drivers/pci/host/xbmd.ko
fpga_io init
fpga_io clk_en 0
/home/w20x/scpi_service_96_100_tmode_debug
```

这部分依赖原板子的 WTE20x/FPGA/RF 硬件、`4.14.98-wte_bt` 定制内核、`xbmd.ko`、`fpga_io`、`/wte` 校准文件和 `scpi_service`。它不能直接迁到 Go2 辅助电脑上运行。

2. 标准 Linux 蓝牙 HCI 扫描和 RSSI 解析：

```bash
start_coded_scan.sh
test_rssi
```

`start_coded_scan.sh` 用 `hcitool -i hciX cmd ...` 配置 BLE Coded PHY 扫描；`test_rssi` 启动 `btmon`，从 HCI 事件里解析目标 beacon 的 MAC 和 RSSI。

所以迁移结论是：

- 能迁移：`start_coded_scan.sh` 的 HCI 配置逻辑、`test_rssi.cpp` 的 `btmon` 解析和窗口统计逻辑。
- 不能原样迁移：`ready_testmode.sh`、`scpi_service*`、`rfio_v6_6*`、`xbmd_50.ko` 这套 WTE/FPGA 初始化。
- Go2 上要跑 RSSI，必须让 Go2 辅助电脑看到一个 Linux 蓝牙控制器，也就是 `hciconfig -a` 里出现 `hci0` 或 `hci1`。
- 如果插到 Go2 USB 后出现 `/dev/ttyACM0`、`/dev/ttyACM1` 但没有 `hciX`，先用 `btattach` 把 UART 蓝牙控制器挂到 Linux HCI。仓库里的 `bt_migrate_pack/go2_attach_ttyacm.sh` 会自动选择编号最大的 `/dev/ttyACM*`。
- 如果只看到 `SEGGER J-Link`，没有 `/dev/ttyACM*` 或 `hciX`，那这块板子没有作为可用蓝牙 HCI/UART 设备暴露给 Go2；当前 `test_rssi` 无法直接使用它。

## 1. 先在 Go2 上确认能不能直接运行旧 RSSI 程序

下面开始都建议在控制电脑的 Ubuntu Terminal 里通过 SSH 登进机器狗电脑执行。也就是说代码最终跑在机器狗电脑上，不是只在控制电脑上跑。

先分清三个名字：

- `enp5s0`：你控制电脑上的网卡名，用来传给 Unitree SDK 程序，例如 `./go2_stand_example enp5s0`。
- `192.168.123.222`：你控制电脑在这张网卡上的本机 IP，通常不是机器狗 IP，不能拿它来 SSH 机器狗。
- `192.168.123.161`：通常是 Go2 的 MCU/运动控制通信地址，可以被 SDK 通信和 ping 到，但一般不是给用户 SSH 部署程序的 Ubuntu 主机。
- 可登录的机器狗辅助电脑 IP：Go2 Edu/R&D 带 Docking Station/辅助电脑时，常见是 `192.168.123.18`，用户 `unitree`，默认密码常见是 `123`。具体以你们机器/师兄文档为准。

SDK 例程能跑不代表你已经登录了机器狗电脑。例程是在控制电脑本地运行，通过 `enp5s0` 和机器狗通信。

如果 RSSI 蓝牙采集板插在控制电脑上，可以在控制电脑本地运行 RSSI；如果采集板插在机器狗 USB 上，RSSI 程序必须运行在机器狗上那个能识别这块 USB 蓝牙硬件的 Linux 系统里。

可以在控制电脑上先找机器狗同网段里哪些主机在线：

```bash
ip addr show enp5s0
ip neigh show dev enp5s0
ping 192.168.123.18
```

如果装了扫描工具，也可以看哪个 IP 开了 SSH：

```bash
sudo apt install -y nmap
nmap -p 22 192.168.123.0/24
```

找到开了 `22/tcp ssh` 的机器狗电脑 IP 后，再用厂家/师兄给你的用户名登录，例如：

```bash
ssh USERNAME@ROBOT_COMPUTER_IP
```

如果只有 `192.168.123.161` 在线，而 `192.168.123.18` 不在线或 22 端口不开，通常表示你当前只能从外部控制电脑用 SDK/DDS 开发，不能直接 SSH 到机器狗内部电脑部署程序。此时 USB 蓝牙采集板如果插在机器狗身上，外部控制电脑上的 `test_rssi` 看不到它；需要把采集板插到运行 `test_rssi` 的电脑上，或者使用 Go2 Edu/R&D 的辅助电脑/另接伴随计算机。

先把项目传到 Go2，`ROBOT_IP` 换成你已经 ping 通的机器狗 IP，用户名换成你的实际用户名：

```bash
rsync -av \
  --exclude 'unitree_sdk2/' \
  ./ USERNAME@ROBOT_IP:~/rssi_go2/
```

如果机器上没有 `rsync`，可以先用 `scp`：

```bash
scp -r GO2_MIGRATION.md bt_migrate_pack scripts unitree_sdk2_go2_imu_logger.patch USERNAME@ROBOT_IP:~/rssi_go2/
```

完整 `unitree_sdk2` 不建议直接放进本仓库普通 Git 历史；在 Go2 上准备 SDK 时，先 clone 官方 SDK，再应用本仓库的 IMU logger 补丁：

```bash
cd ~/rssi_go2
git clone https://github.com/unitreerobotics/unitree_sdk2.git
git -C unitree_sdk2 apply ../unitree_sdk2_go2_imu_logger.patch
```

然后 SSH 登进去：

```bash
ssh USERNAME@ROBOT_IP
cd ~/rssi_go2
chmod +x scripts/go2_collect_rssi_imu.sh \
  bt_migrate_pack/test_rssi \
  bt_migrate_pack/start_coded_scan.sh \
  bt_migrate_pack/go2_attach_ttyacm.sh \
  bt_migrate_pack/go2_run_rssi.sh
```

最小实机系统建议先在 Go2 上编译。原因是 RSSI 采集要用机器狗身上的蓝牙 HCI，IMU 要订阅机器狗本机/同网段 DDS，而且本机编译能避开 CPU 架构和动态库路径不一致的问题。控制电脑也可以编译，但只有在目标架构一致，或者你配置了交叉编译和运行时库路径时才省事。

```bash
uname -m
file bt_migrate_pack/test_rssi
```

`test_rssi` 是 `aarch64` 静态二进制。如果 Go2 输出也是 `aarch64`，可以直接试运行；如果 Go2 是 `x86_64`，这个二进制不能直接跑，需要拿到 `test_rssi.cpp` 源码重新编译，或者重写 RSSI 采集。

## 2. 检查蓝牙扫描能力

RSSI 程序必须运行在插着蓝牙采集硬件的那台 Linux 上。如果你的蓝牙采集板子插在机器狗 USB 上，下面命令就要 SSH 到机器狗电脑上执行。

```bash
which btmon
which hcitool
which btattach
ls /dev/ttyACM*
hciconfig -a
```

如果没有 `btmon`、`hcitool` 或 `btattach`：

```bash
sudo apt update
sudo apt install -y bluez
```

如果 `ls /dev/ttyACM*` 能看到蓝牙板子，但 `hciconfig -a` 还没有对应的 `hciX`，先挂载 UART 蓝牙控制器：

```bash
bash bt_migrate_pack/go2_attach_ttyacm.sh
```

脚本默认选择编号最大的 `/dev/ttyACM*`，也就是你之前文档里通常选择的 `/dev/ttyACM1`。如果要显式指定：

```bash
BT_TTY=/dev/ttyACM1 BT_BAUD=1000000 bash bt_migrate_pack/go2_attach_ttyacm.sh
```

它会输出类似：

```text
BT_TTY=/dev/ttyACM1
HCI_DEV=hci0
```

看到 `hciconfig -a` 里对应设备有 `UP RUNNING`，就说明这一步成功。

然后配置 BLE Coded PHY 扫描：

```bash
sudo bash bt_migrate_pack/start_coded_scan.sh
```

如果你的采集板不是 `hci0`，例如 `hciconfig -a` 里显示它是 `hci1`：

```bash
sudo hciconfig hci1 up
sudo bash bt_migrate_pack/start_coded_scan.sh hci1
```

检查扫描时可以临时开一个窗口运行：

```bash
sudo btmon
```

确认广播包在跳动后按 `Ctrl+C` 退出。后面的 RSSI 程序会自己启动 `btmon`，测试时不要把手动 `btmon` 窗口继续开着。

这个脚本只用了标准 HCI 命令，迁移时需要它；`ready_testmode.sh`、`scpi_service*`、`rfio_v6_6*`、`xbmd_50.ko` 是原 WTE/FPGA 板子的专用初始化，迁到 Go2 时先不要用。

## 3. 单独测试 RSSI

如果只想复用旧文档里的 `test_rssi_auto/manual` 流程，可以直接用迁移包装脚本。它会自动执行 `ttyACM -> btattach -> start_coded_scan -> test_rssi_auto/manual`，并把结果放进新的时间戳目录，避免继续追加到上一次的 `rssi_result.txt`：

```bash
BT_TTY=/dev/ttyACM1 bt_migrate_pack/go2_run_rssi.sh auto 5 0.5
BT_TTY=/dev/ttyACM1 bt_migrate_pack/go2_run_rssi.sh manual 5 0
```

如果已经手动执行过 `btattach`，可以跳过自动挂载：

```bash
BT_ATTACH=0 HCI_DEV=hci0 bt_migrate_pack/go2_run_rssi.sh auto 5 0.5
```

如果你是在控制电脑上跑宇树 SDK 例程，并且控制电脑是 `x86_64`，先用源码编译 RSSI 程序：

```bash
scripts/build_rssi_logger.sh
```

关闭旧串口 IMU，只采 RSSI：

```bash
sudo ./build_rssi/test_rssi 30 rssi_realtime_windows.csv 1.0 0.5 0.5 0.0 none
```

参数含义：

- `30`：采集 30 秒；用 `0` 表示一直采，直到 `Ctrl+C` 或输入 `q`。
- `rssi_realtime_windows.csv`：窗口均值/标准差输出。
- `1.0`：窗口长度 1 秒。
- `0.5`：窗口步长 0.5 秒。
- `0.5`：每次按 Enter 递增的路径距离 `s`，单位米。
- `0.0`：起点 `s`。
- `none`：禁用旧串口 IMU。

运行后应生成：

```text
rssi_realtime_windows.csv
rssi_raw_stream.csv
rssi_markers.csv
imu_stream.csv
```

其中这个 `imu_stream.csv` 会为空或无效，因为我们禁用了旧 IMU，真正的 IMU 用下一步的 Go2 logger。

## 4. 编译 Go2 IMU logger

下面命令都在本项目根目录运行。不要复用 SDK 自带的 `build/`，它可能带着别的机器路径缓存；新建 `build_go2` 更稳。

如果 `unitree_sdk2/example/go2/go2_imu_logger.cpp` 还不存在，先应用补丁：

```bash
git -C unitree_sdk2 apply ../unitree_sdk2_go2_imu_logger.patch
```

```bash
cmake -S unitree_sdk2 -B unitree_sdk2/build_go2
cmake --build unitree_sdk2/build_go2 --target go2_imu_logger -j"$(nproc)"
```

运行时需要填机器狗连接 SDK 使用的网卡。先看网卡名：

```bash
ip link
```

常见是 `eth0`，也可能是 `enp...` 或 `wlan0`。

测试 10 秒：

```bash
./unitree_sdk2/build_go2/bin/go2_imu_logger eth0 go2_imu_stream.csv 10
```

默认假设 Go2 SDK 的 accelerometer 是 `m/s^2`，gyroscope/rpy 是弧度制，程序会转换成：

- `acc_x_g,acc_y_g,acc_z_g`
- `gyro_x_dps,gyro_y_dps,gyro_z_dps`
- `roll_deg,pitch_deg,yaw_deg`

如果静止时 `sqrt(acc_x_g^2 + acc_y_g^2 + acc_z_g^2)` 明显不是接近 1，而是接近 0.1 或 9.8，说明单位假设不对。若 Go2 固件已经直接输出 `g/dps/deg`，运行时在最后加 `1 1 1`：

```bash
./unitree_sdk2/build_go2/bin/go2_imu_logger eth0 go2_imu_stream.csv 10 1 1 1
```

## 5. 正式采集时并行跑 RSSI 和 Go2 IMU

最省事的方法是用仓库里的脚本，它会自动执行：

1. 选择 `/dev/ttyACM*` 中编号最大的蓝牙串口。
2. 用 `btattach -B <tty> -S 1000000` 挂载成 `hciX`。
3. 调用 `start_coded_scan.sh` 配置 S2/S8 Coded PHY 扫描。
4. 并行启动 Go2 IMU logger 和 RSSI logger。

它会把所有输出放到同一个目录：

```bash
scripts/go2_collect_rssi_imu.sh eth0 0 1.0 0.5 0.5 0.0
```

参数依次是：

- `eth0`：SDK 网卡名。
- `0`：采集时长，`0` 表示一直采，直到 `Ctrl+C` 或输入 `q`。
- `1.0`：RSSI 窗口长度。
- `0.5`：RSSI 窗口步长。
- `0.5`：每次按 Enter 增加的路径距离。
- `0.0`：起始路径距离。

如果要指定输出目录：

```bash
RUN_DIR="$HOME/go2_rssi_runs/test_001" scripts/go2_collect_rssi_imu.sh eth0 0
```

如果自动选择的串口不对，指定蓝牙板：

```bash
BT_TTY=/dev/ttyACM1 scripts/go2_collect_rssi_imu.sh eth0 0
```

如果你已经手动 `btattach` 好了，只想复用已有 `hci0`：

```bash
BT_ATTACH=0 HCI_DEV=hci0 scripts/go2_collect_rssi_imu.sh eth0 0
```

默认采集脚本不会在退出时杀掉 `btattach`，方便你连续测试。若希望每次采集结束自动关闭这次脚本启动的 `btattach`：

```bash
BTATTACH_CLEANUP=1 scripts/go2_collect_rssi_imu.sh eth0 30
```

走到每个地面标记点时，在运行脚本的终端按 Enter，`s` 会按 `marker_step_m` 增加；如果走错，可以直接输入数字修正当前 `s`。

## 6. 用现有 baseline 处理

`robot_s_baseline.py` 需要 RSSI CSV 里有 `elapsed_sec` 才能和 IMU 对齐。新版 `test_rssi` 的 `rssi_realtime_windows.csv` 会有这个列。

示例：

```bash
python3 robot_s_baseline.py \
  --train path/to/train_rssi.csv \
  --test "$RUN_DIR/rssi_realtime_windows.csv" \
  --imu "$RUN_DIR/imu_stream.csv" \
  --out "$RUN_DIR/predictions.csv"
```

如果你采出来的 RSSI CSV 没有 `elapsed_sec`，先发我文件头部，我再帮你加一个对齐转换脚本。
