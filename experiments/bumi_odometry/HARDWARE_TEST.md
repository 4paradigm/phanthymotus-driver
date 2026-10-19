# 自行真机验证：固定相机、约一米前进

本说明针对 PR #408 的 `feat/bumi-loco-destination` 分支。里程计是独立实验，不会因部署旋转卡片就自动运行；距离控制尚未接入。ROS 链路尚未真机验证，先做环境检查，不直接进行二十次行走。

`loco_destination` 使用自己的闭环线程直接发送 SDK 运动指令，不通过 `loco_servo` 执行，也不依赖它启用。两张卡片仅共享底盘互斥保护：旋转时暂停已有流式控制，避免同时写速度；尚未接入的距离控制也按独立卡片实现。

## 1. 取得代码与实验环境

在独立目录取得 PR 分支，不覆盖正在运行卡片的部署目录：

```sh
git clone --branch feat/bumi-loco-destination --single-branch \
  https://github.com/Nidhogg2002/phanthymotus-driver.git bumi-odom-test
cd bumi-odom-test
export BUMI_ODOM_REPO="$PWD"
mkdir -p "$BUMI_ODOM_REPO/measurements"
```

板端需要单独准备的 ROS 2 Humble 实验环境；不要在正在测试其他卡片的生产容器中安装/替换 SDK。各实验终端均执行：

```sh
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=185
ros2 pkg prefix realsense2_camera
ros2 pkg prefix rtabmap_odom
ros2 pkg prefix robot_localization
ros2 pkg prefix tf2_ros
ros2 pkg prefix rtabmap_msgs
python3 -c 'import rclpy, pyrealsense2'
```

185 域也需确认没有其他任务占用。任何包缺失先解决实验环境，再继续。当前没有经 ARM64 集成验证的依赖锁文件，因此这还不是可直接替换的生产镜像。若使用实验容器，需能看到相机 USB/HID、udev 数据和对应 sysfs；仅开启 privileged 不保证 IMU 可见。

释放原相机采集者后，在实验环境运行以下只读能力检查；不要同时启动两个相机 pipeline：

```sh
python3 experiments/bumi_odometry/preflight.py > measurements/preflight.json
cat measurements/preflight.json
```

应有 color、depth、gyro、accel，且 missing_streams 为空、ready 为 true。ready 只代表枚举成功，还需下面的实际读流检查。此前生产容器只有 RGB/depth、没有 Motion Module，不能跳过这一项。

## 2. 安装外参怎么测

准备卷尺/直尺、铅垂线、地面胶带及水平/倾角测量工具。

1. 在**固定机身**选一个能反复定位的点 P 并贴标记，例如胸部刚性支架上的小点，作为本实验 `bumi_odom_base_link` 原点。以后测移动距离也用 P，不用脚尖、头顶或每次变化的点。
2. 定义机身坐标：X 朝前、Y 朝机器人左侧、Z 朝机身上方。用 CAD/安装尺寸或直尺和垂线，测相机 **camera_link 原点**相对 P 的前后、左右、上下三个分量，换成米。前方/左侧/上方为正。它不是光学坐标，也不是相机外壳中心；RealSense ROS 文档把 camera_link 原点放在 infra1（左 IR）位置，应结合 D435i 机械图定位。
3. 测 camera_link 相对于上述机身轴的安装旋转，优先用 CAD/安装图及倾角仪校验。需要的是将 camera_link 向量旋转到机身的四元数，顺序 xyzw。只有确认各轴平行且同向才可填 `[0,0,0,1]`；相机俯视时不能这样填，也不能把光学坐标的轴旋转再加一遍。
4. 另测起始机身相对于重力水平面的 roll/pitch，写入 initial_base_rpy_rad，单位弧度；局部起始 yaw 可以定义为 0。安装角和起始机身倾角是两件事。

保存 `measurements/mount.json`，字段如下；所有数值都要用实测值替换，不提供可直接误用的零外参模板：

| 字段 | 内容 |
| --- | --- |
| fixed_mount | true |
| calibration_id | 本次标定编号，如 bumi-chest-20261019 |
| translation_m | 相机原点在机身中的 `[x,y,z]`，米 |
| quaternion_xyzw | 相机到机身旋转 `[qx,qy,qz,qw]` |
| initial_base_rpy_rad | 起始机身重力姿态 `[roll,pitch,yaw]`，弧度 |

若只有量出的安装角而不会转四元数，保留带正负方向的 roll/pitch/yaw 和坐标轴说明再换算；不要猜四元数。首次检查还应使用 `ros2 run tf2_ros tf2_echo bumi_odom_base_link bumi_odom_camera_link` 核对平移、旋转及方向。

## 3. 先确认数据链路，再行走

先按 [README 的录包命令](README.md#录包与回放ros-集成尚待验证) 启动录包，再在另一个同域实验终端启动：

```sh
ros2 launch "$BUMI_ODOM_REPO/experiments/bumi_odometry/pipeline.launch.py" \
  calibration_file:="$BUMI_ODOM_REPO/measurements/mount.json" \
  output_csv:="$BUMI_ODOM_REPO/measurements/trial-001.csv" \
  start_camera:=true
```

另一个终端观察（同样先设置 ROS 环境及 ROS_DOMAIN_ID）：

```sh
ros2 topic echo /bumi_odom/status
ros2 topic hz /bumi_odom/bumi_odom_camera/imu
ros2 topic hz /bumi_odom/visual
```

应从 initializing 进入 tracking；gyro、RGB-D、视觉及滤波消息均存在，TF 连通。保持静止录 60 秒查看漂移，再做转动对照：测转动前后 P 的实际位移，不能假设人形原地转身绝对没有平移。出现 lost 就保留该失败记录，排查后重新启动进程和新 CSV。未到 tracking 时不要开始一米精度验收。

## 4. 前进一米怎么独立测量

1. 地面贴一条与起始机身水平前向一致的直线，用卷尺标出 0 和 1 米。机器人静稳后，将 P 用铅垂线投到地面，标记起点；不要用步数或发送的速度/时长代替距离。
2. 使用你已有的手动方式让机器人沿线前进，在约 1 米处停止。本实验进程不会控制机器人。静稳后再次投影**同一个 P**，标记终点。
3. 量起终点在起始前向上的投影差，作为 forward_m；量垂直于前向的差，机器人左侧为正，作为 lateral_m。真实距离在 0.9–1.1 米即可做本轮一米评估，不必强行停在 1.000 米，但必须写实际值。
4. 记录测量不确定度 uncertainty_m，建议不超过 0.01 米。例如测得 1.012 米、误差范围约 ±0.005 米，就填写 1.012 和 0.005。若机身晃动或投影模糊超过此范围，应扩大不确定度，不要仍写 5 毫米。
5. 起终点都留一段静稳数据；终点后继续采集至少 1 秒，并确认视觉诊断已覆盖终点后再停止。每次试验单独启动估计/质量进程、新 CSV 和录包，保留所有失败尝试。

## 5. 生成误差报告

CSV 第一列是 ROS 纳秒时间。按预先约定的静稳窗口选取规则，分别填写起点、终点窗口的 `[开始纳秒,结束纳秒]`；不是从程序启动起算的秒数。不要挑选误差最小的窗口。可用文本/表格查看 CSV 的 tracking 段来确定窗口；每个窗口至少 5 个样本、实际采样跨度至少 0.5 秒；每轴位置跨度超过 2 厘米或首末样本平均漂移速度超过 0.01 米/秒会被拒绝。不能选单点/零宽窗口。

在 measurements 中为每次试验保存 `trial-001.json`，格式见 [README 真值示例](README.md#本地距离评估)。其中 trial_id 与 manifest 一致，forward_m/lateral_m 填独立测量，scene 填场景，speed_m_s 填所用速度并注明来源，调参阶段 split 为 development，冻结参数后的留出试次为 acceptance。

创建 `measurements/trials.json`，路径均相对于该文件：

```json
{"data_source":"measured","trials":[{"trial_id":"trial-001","samples":"trial-001.csv","truth":"trial-001.json"}]}
```

每做一次都把该次加入 trials，包括失败试次；不能只录入成功数据，也不能用不同编号/场景重复引用同一个 CSV（包括链接或相对路径别名）。随后运行：

```sh
python3 experiments/bumi_odometry/evaluate.py \
  --manifest measurements/trials.json --output measurements/report.json
cat measurements/report.json
```

先用少量 development 试次验证流程和调参，再固定标定、配置及版本，做至少 20 次 acceptance，覆盖至少两个室内场景、不同朝向及拟使用速度。验收要求全部有效，且每次 `abs(forward_error_m)+uncertainty_m <= 0.05`。inconclusive 是无法确定，invalid 是数据无效，都不能算通过；只做一次即使误差小也不会得到整批验收通过。

结束后停止实验相机进程，恢复此前的相机采集者并检查原视觉卡片。保留每次 CSV、原始录包、mount.json、preflight.json、真实测量表、配置/版本和 report.json，便于回放定位。完成前不宣称机器人能自动前进一米并停在 ±5 厘米内。

参考：[RealSense ROS 坐标说明](https://github.com/realsenseai/realsense-ros#ros2robot-vs-opticalcamera-coordination-systems)、[D400 官方机械图与数据手册](https://cdrdv2-public.intel.com/841984/Intel-RealSense-D400-Series-Datasheet.pdf)。
