# Bumi RGB-D / IMU 里程计实验

本目录保留独立采集/回放/评估工具，本身不发送运动命令。正常部署审核镜像时，生产驱动已默认管理共享相机与融合，无需运行本目录的实验 launcher；它输出卡片使用的 `/bumi_odom/filtered` 和 `/bumi_odom/status`，继承生产 domain 42。请勿同时启动独立相机或在同一域重复发布融合输出。生产入口见 [Bumi README](../../noetix/bumi/README.md#loco_destination)。本实验默认独立域185，仅在协调相机占用后用于离线验证。

2026-10-19 按用户要求随原有 loco_destination PR #408 提交，实验运行仍保持独立。自行操作请先读 [真机测量与验证步骤](HARDWARE_TEST.md)。

## 本地距离评估

```sh
python3 -m pytest -q tests/test_bumi_odometry_evaluate.py
python3 experiments/bumi_odometry/evaluate.py --manifest /path/to/trials.json --output /path/to/report.json
```

manifest 示例（相对路径以 manifest 所在目录为基准）：同一 CSV 只能出现一次；解析后的路径重复（包括相对路径及符号链接别名）会保留为 invalid 尝试，不能靠改 trial_id 或 scene 重复计数。

```json
{"data_source":"measured","trials":[{"trial_id":"room-01","samples":"room-01.csv","truth":"room-01.json"}]}
```

合成数据必须标记 `data_source: synthetic`；未知来源或合成数据不会输出真机精度验证通过。

CSV 位姿列：`stamp_ns,x,y,z,qx,qy,qz,qw,status,segment_id`，导出器还写入故障源时间列 `fault_stamp_ns`。位姿为同一水平里程计坐标系下的**机身参考点**，米、纳秒、xyzw 四元数。不要直接将相机位姿冒充机身位姿。

真值 JSON 示例：

```json
{"trial_id":"room-01","start_ns":[1000000000,1500000000],"end_ns":[11000000000,11500000000],"forward_m":1.003,"lateral_m":0.012,"uncertainty_m":0.005,"scene":"room","speed_m_s":0.1,"split":"acceptance"}
```

起终点窗口必须各有至少 5 个样本、实际采样跨度至少 0.5 秒；位置各轴跨度不超过 2 厘米，首末样本各轴平均漂移速度不超过 0.01 米/秒，且处于同一 tracking 段。单点或零宽窗口不能证明静稳。取位置均值，以起始窗口第一帧机身 X 轴的水平投影定义前向；报告前向、横向及水平向量误差。时间倒退、段重置、失跟、采样间隙超过 0.5 秒、无有效窗口均判为无效，不从报告中剔除。**一个 CSV 对应一次试验**，任何时刻记录的 lost 都使整份记录无效，包括终点之后才到达的诊断；不能截掉故障尾部或将同一 CSV 拆成多次验收。

验收需至少 20 次独立验收试次、至少两个场景，每次真实前向位移在 0.9–1.1 米范围，全部有效且 `abs(前向误差)+测量不确定度 ≤0.05m`。其他距离仍可评估，但不作为一米精度通过的依据。误差区间跨边界则为 inconclusive。还须人工核查真实测量、独立留出数据、不同朝向及拟使用速度覆盖；JSON 中的 measured 是来源声明，不是对实测真实性的自动认证。该报告不验证自动行走的制动精度。

## 传感器预检与固定安装标定

```sh
python3 experiments/bumi_odometry/preflight.py > /path/to/preflight.json
```

预检只枚举，不打开流；退出码 0 仅表示存在 RGB、depth、gyro、accel 能力，**不代表实际采集或标定通过**。当前生产容器的枚举结果缺少 IMU，需在实验环境核实 udev 可见性及 SDK 版本/构建差异，不能直接改线上容器。

用户于 2026-10-16 确认相机固定，头部不能动。启动仍须提供真实安装标定 JSON：

- `fixed_mount: true` 与非空 `calibration_id`，便于数据溯源。
- `translation_m: [x,y,z]`：相机 `camera_link` 原点在机身参考系中的位置，米。
- `quaternion_xyzw: [qx,qy,qz,qw]`：将 camera_link 向量旋转到机身坐标的旋转；不是光学坐标系外参。
- `initial_base_rpy_rad: [roll,pitch,yaw]`：起始机身相对于重力对齐里程计坐标的姿态。用静止 IMU/外部水平基准核实 roll/pitch，局部起始 heading 可定义为 yaw=0。不得凭空认定机身完全水平。

未提供示例零外参文件，避免直接用于真机。

## 融合进程

需要 Humble 的 realsense2_camera、rtabmap_odom、robot_localization、tf2_ros、rclpy、rtabmap_msgs。配置集中在 `configuration.py`，避免 YAML 与启动参数重复维护。当前尚无已实际运行验证的版本锁文件。

所有实验终端使用同一独立 DDS 域，先确认该域没有其他任务占用：

```sh
export ROS_DOMAIN_ID=185
source /opt/ros/humble/setup.bash
ros2 launch /absolute/path/to/experiments/bumi_odometry/pipeline.launch.py \
  calibration_file:=/path/to/measured-mount.json \
  output_csv:=/path/to/new-trial.csv
```

默认 `start_camera:=false`，仅启动旁路估计及质量进程；没有输入时保持 initializing。不要与线上不同域的数据混用。只有约定采集窗口内、原相机采集已释放后，才增加 `start_camera:=true`。本工具不负责停止或重启线上服务。

RTAB-Map 不使用 IMU，输出 6D 位姿；EKF 融合视觉位姿和三轴角速度，禁用线加速度积分。RGB/深度按源时间近似配对，上限 20 毫秒，须实测配对延迟；图像订阅采用 best effort，兼容传感器 QoS。TF frame 使用 `bumi_odom_` 前缀，实验又以 DDS 域隔离。

`quality.py` 对原始视觉位姿、视觉诊断、IMU 及滤波位姿检查时间/有效性。视觉、IMU、滤波位姿或诊断消息超时 0.5 秒、时钟倒退、视觉失跟、距离累计量回退等使 lost 状态锁存，并立即刷盘一条带故障源时间的 lost 行，不依赖 EKF 是否继续输出。每次试验必须重新启动质量进程并使用新 CSV，不允许在同次试验中恢复为 tracking。输出文件使用独占创建，拒绝覆盖既有数据。终点静稳后继续记录至少 1 秒，并确认视觉位姿和诊断均已覆盖所选终点时刻后再结束采集；异常退出或未收齐诊断的录包不能作为有效验收数据。

## 录包与回放（ROS 集成尚待验证）

各命令在实验 DDS 域执行，录包前检查磁盘余量和资源基线；原始图像会占用明显带宽和存储。先启动记录，再启动唯一的相机采集。另存安装标定、预检 JSON、SDK/ROS 包版本及配置哈希。

```sh
ros2 bag record --storage sqlite3 -o /data/new-bumi-trial \
  /bumi_odom/bumi_odom_camera/color/image_raw \
  /bumi_odom/bumi_odom_camera/aligned_depth_to_color/image_raw \
  /bumi_odom/bumi_odom_camera/depth/image_rect_raw \
  /bumi_odom/bumi_odom_camera/color/camera_info \
  /bumi_odom/bumi_odom_camera/depth/camera_info \
  /bumi_odom/bumi_odom_camera/imu \
  /bumi_odom/bumi_odom_camera/gyro/sample \
  /bumi_odom/bumi_odom_camera/accel/sample \
  /bumi_odom/bumi_odom_camera/color/metadata \
  /bumi_odom/bumi_odom_camera/depth/metadata \
  /bumi_odom/bumi_odom_camera/gyro/metadata \
  /bumi_odom/bumi_odom_camera/accel/metadata \
  /bumi_odom/visual /bumi_odom/visual_info /bumi_odom/filtered /bumi_odom/status /tf /tf_static
```

回放重算时，启动融合配置（`use_sim_time:=true start_camera:=false`）及新输出 CSV。TF 静态安装从录包恢复，启动参数加 `publish_mount:=false`，避免重复发布；录包含相机内部静态外参。只回放以下输入，不回放记录的视觉/融合输出、status、动态 `/tf`，防止旧结果及旧 EKF TF 混入重算：

```sh
ros2 bag play /data/new-bumi-trial --clock --topics \
  /bumi_odom/bumi_odom_camera/color/image_raw \
  /bumi_odom/bumi_odom_camera/aligned_depth_to_color/image_raw \
  /bumi_odom/bumi_odom_camera/color/camera_info \
  /bumi_odom/bumi_odom_camera/imu /tf_static
```

每次回放重启所有估计进程和质量进程，重置滤波状态与时钟，使用相同标定和配置；至少重复两次，比较有效段及位移。若只检查当时记录的输出，应只回放旧估计/诊断，**不要同时启动估计链路**。不要对原始数据重新盖当前时间戳。

现场先录制 60 秒静止与转动对照，再收集少量开发试次调参，冻结参数后收集至少 20 次独立验收试次。由现场人员用现有手动方式移动机器人、测量同一机身参考点起终点；建议参考测量不确定度不超过 1 厘米。先用质量 CSV 选择静稳窗口，再填写独立实测真值。采集结束恢复原相机采集，并核查其他视觉卡片功能和资源占用。
