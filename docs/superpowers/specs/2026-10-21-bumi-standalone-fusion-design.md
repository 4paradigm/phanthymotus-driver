# Bumi 独立 RGB-D＋机身 IMU 融合里程计

状态：实施设计。尚未标定、构建、部署或完成硬件精度验收。

## 基线与范围

2026-10-21 读取 GitHub PR API 并 fetch 核实：#408 open，head
`40ae7fe80382f02085cb9e7bf55133cdc2c62151`；上游 main
`43f7e37c2a938fb1a1c511f19bd55622ae569014`。开发分支
`feat/bumi-rgbd-imu-odometry-sensor` 从 PR head 建立。原分支和其他工作区不改动。
平台上游 main 为 `6df601f612818716bb0dc7b180a9467e2c43172c`，通过 git show
读取，不切换、不修改平台工作区。

目标为完整的 RTAB-Map RGB-D 视觉里程计与 robot_localization EKF 松耦合融合，
不以自研视觉前端替代必要功能。不含回环和全局定位，不承诺漂移有界。
机身 IMU 角速度参与 EKF；加速度用于重力初始化和质量检查。加速度积分的启用以
重力、零偏、延迟及杠杆臂验证为条件，不因称作“融合”就盲目勾选所有变量。

## 历史实现审计

| 项目 | 源码证据 | 必须补齐 |
| --- | --- | --- |
| 独立实验 | experiments/bumi_odometry/configuration.py | 原本依赖相机 IMU，替换为机身 IMU 输入 |
| 托管融合 | managed/fusion_runtime.py | 零外参是假设；依赖检查仍要求 realsense2_camera |
| EKF | fusion_parameters | 原实现融合视觉六维位姿与三轴 gyro，未做加速度融合 |
| 时间 | device.py _body_imu_msg / camera_runtime.py capture | 原消息盖读取/发布时间，没有源时间与不确定性标识 |
| 质量 | FusionHealth | 四流独立新鲜不等于同一观测已被融合，缺少数据关联与复位隔离 |
| 消费 | managed/distance_control.py | 固定话题存在不代表画布连接；pose/status 分开发送存在段混合风险 |
| 仿真 | tests/bumi_fusion_smoke.py | 发布相机 IMU、订阅机身 IMU不匹配，静态随机纹理不能验证运动误差 |
| 安装 | bumi/Dockerfile、build.sh、deploy/service.yml | 独立预构建融合依赖层、完整发布/安装路径尚不存在 |

历史 2026-10-15 记录为 ARM64、D435i、固件 5.15.1.55；host librealsense 2.57
可枚举 motion，旧容器 pyrealsense2 2.59.1.11053 无 motion。不是本轮硬件复测。
2026-10-21 用户确认尚无实测安装外参与 IMU 约定记录。没有 ROS、Docker/Podman
的本地环境不能运行真实融合进程或 ARM64 镜像构建。

## 进程与安装

保留 Bumi bundle 的唯一 pyrealsense2 采集进程。额外标准 ROS Image/CameraInfo 和
带时间来源的元数据供只读 sidecar 订阅，现有 JPEG、Z16/zlib、内参、照片录像缓存
不变。sidecar 不挂载 USB、不安装/启动 ROS RealSense 相机节点、不依赖 Docker socket。
里程计 stop/reset/crash 只停止自己的 RTAB-Map、EKF 和桥接，不带停相机。

独立驱动目录与 driver.yaml 注册 sensor `rgbd_imu_odometry`，沿用 snake_case 命名。
通过已有多服务安装/服务声明部署独立镜像；不能只交一个需要手动启动的 Dockerfile。
基础依赖镜像安装并固定 Humble rtabmap-odom、robot-localization、TF、消息与运行库；
应用镜像只 COPY 自己的服务/配置与 common。无 desktop、rviz、realsense2-camera。
必要传递依赖允许保留。冷构建成本单独记录，依赖镜像按 digest 引用，CI registry cache
复用；不承诺首次依赖构建短于 30 分钟。不向普通 Bumi 镜像恢复融合 apt 安装。

## 时间与原始数据

相机输出原始 BGR、对齐深度、实时 CameraInfo（完整 K/D/R/P、实际光学 frame）、
depth_scale、RGB/depth 各自源时间与 timestamp domain、frame number、读取 wall/monotonic
时间、相机 epoch。Z16 转米使用 SDK depth_scale，不假设单位恒为毫米。
硬件时钟到 ROS 时钟必须显式映射并监控偏移/跳变；未经确认的时间域不能直接当 Unix。
RGB-D 按源采集时刻配对，初始允许差 20ms，属于待录包验证的门限。
彩色有畸变时在 sidecar 成对去畸变，深度 nearest 重采样；对齐不等于去畸变。

机身桥接保留 SDK ori/gyro/accel/原协方差、读取前后时间、poll 序号；源采样时间为 null，
stamp_source 明确为 host_read。不得以 poll_seq 或值变化证明源设备正在更新。
实测用受控转动的视觉角速度与 gyro 做延迟相关，报告固定偏移、抖动和适用区间；
恒速或静止不可观的记录拒绝生成延迟标定。未知源龄无法凭读取耗时上界化。
传输新鲜度可以测试，源采样新鲜度仍是公开限制。自动距离反馈需显式通过读时近似
的现场资格验证；未通过则拒绝启动，不伪造“精确同步”。
处理视觉晚到数据时评估 EKF smooth_lagged_data/history_length；缓存有界，延迟超界失效，
不使用 tf 时间偏移伪装观测同步。参数以锁定版本真实 ROS 回放验证。

## 坐标、参考点与估计

机身 B：x 前、y 左、z 上；RGB 光学 C：x 右、y 下、z 前。标定保存
T_B_C（将光学坐标变到机身参考点）、T_B_I、calibration_id、来源与测量不确定度。
显式验证 imu quaternion xyzw、I→world 或反向、ENU/NED、gyro/acc 单位与重力正负。
原 SDK 代码的约定只作为待核实假设。固定相机/头部不动是使用条件，不做动态头部补偿。
现有 bumi_model.urdf 存在 waist_yaw_joint，却未给出相机/IMU 安装链。头部固定不自动
证明相机相对 pelvis/base_link 刚性固定；标定时须确认 B 是哪个实物参考点，以及
相机和 IMU 是否跨腰关节。若跨关节则需要已核实的带时间关节 TF，或者在本功能运行
时锁定该关节；不能直接拿骨架渲染 URDF 代替安装标定。

采用 TF 让 RTAB-Map frame_id 指向 B，视觉输出即 T_O_B；若输出 C，则严格计算
T_O_B = T_O_C inverse(T_B_C)，不能仅旋转平移或直接减常量安装位移。
速度参考点变换须包含角速度叉乘杠杆臂，协方差按同一变换传播；避免二次补偿。
若以后融合离 B 有偏移的加速度，须核对角加速度与向心项补偿后再启用。

RTAB-Map 只吃已校验 RGB-D 与标定，不反馈 EKF 位姿、不重复吃同一路 IMU。
EKF 吃机身视觉位姿与已校准 gyro；保留完整 3D，禁止用 two_d_mode 掩盖步态俯仰。
初始 roll/pitch 由安静窗口和经验证重力约定获得，局部 yaw 可定义为零。
gyro 零偏/噪声与视觉协方差通过录包测量；拒绝非有限/非法协方差，不用固定小方差
冒充置信度。IMU 姿态可用于初始化及一致性检查；启用姿态观测要记录与 gyro 的相关性。

EKF 为唯一局部动态 TF 发布者。外部协议复用 common/odom.py 的 motus.odom/1：
速度 frame=body，pose 中明确 O 帧及 B 参考点，不变更旧 motion_state 的部分测量接口。
state/odom 的单个 String 消息同时携带 pose、twist、时间和 vendor 中的 quality、
segment_id、calibration_id、covariance、visual diagnostics、input ages、time provenance、
accuracy_validated=false。健康状态不能与另一段位姿拼接；未知置信信息用 null。

## 质量、重置与连续性

initializing：缺标定、输入、同步资格或静止初始化；不产出可供距离运动使用的有效位姿。
tracking：视觉、诊断、IMU、滤波均有效，RGB-D 配对与时间来源符合条件；有足够连续
观测并检查协方差、inliers/匹配、跳变、速度与方向一致性。
lost：视觉失跟、原始输入停止/超时、时钟异常、诊断缺失、滤波失效、进程退出、外参变化。
EKF 的 predict-only 输出不能维持 tracking。视觉 lost 立即使当前段永久失效，晚到正常
消息不能将本段修复成 tracking。质量门关联 visual/OdomInfo 源 stamp 与滤波处理范围，
不只检查四个“最新消息”是否分别新鲜。

reset 顺序：先发布 lost 与原因 → 结束旧段并隔离旧输入/输出 → 停止/重建 RTAB/EKF →
新 session UUID/segment、重新初始化 → 连续健康后 tracking。所有跨段数据拒绝。
段内提供连续位姿/速度；段间明确坐标重置。不会为了视觉上的连续拼接任意偏移，也
不会使旧动作在新段续走。晚到故障需保留且使相应试次失效。

## 实际连线与距离动作

平台 config.py 已按保存 connections 顺序解析 topic 并通过 start(input_topic/inputs)
传给目标，info() 检查真实订阅；canvas.js 在连接变化时有 start/stop，但 canvas.py
save_layout 只停止删卡，不保证所有保存入口的输入变更同步。仅驱动补订阅不足以
保证整个需求，需一份独立平台配套补丁，不能悄悄改另一个已有工作分支。

最小平台扩展：从保存图解析每个声明需要绑定的消费者，生成只针对其输入边的 binding
revision（含连接 id、源卡 id/port、目标卡 id/port、topic；无边与未解析边必须区分），
通过现有生命周期派发给驱动并得到确认。保存/启动/方案切换统一走该路径；绑定失败
不允许新动作。活动动作期间换线立即撤销旧 revision，并同步制动；控制器同时使用
短期绑定有效期/watchdog，平台/连接失联后停车。无连接也需要平台明确确认，不因缺参
自动选择 IMU。兼容旧平台的直接调用可保留现有纯 IMU实验，但不能宣称已满足图绑定。

驱动拒绝多路 odom，info 返回实际 input_topic 与 revision。动作受理时锁定 source、
revision、segment、calibration 与起点；每个非零命令前再次核对取消/故障/绑定失效。
无连接用原 imu_integrated_experimental；已连接未 ready 同步拒绝；运动中故障不降级。
直行位移为 (p(t)-p0) 在起始机身水平前向的投影，另查横向及航向偏差。
所有退出路径走原 SDK 零速度、共享底盘互斥、恰好一次 ACP；旋转算法保持不变，
不通过 loco_servo 执行。结果带 source/质量/segment/revision/估计误差与实际精度未验证。

## 验证与交付门槛

1. ROS-free 行为测试：真实保存连接解析与运行时消息、无/单/多连接、未解析来源、
   连接变化/租约失效、pose/status 原子性、旧段队列、静止/纯转/非零杠杆臂/直行、
   gyro 零偏和噪声、时间重复/倒退/漂移/长间隙、失跟、复位、取消和每个错误出口停车。
2. ARM64 构建：锁定依赖、启动 import/可执行检查、实际 RTAB+EKF ROS 集成。
   摄像头仿真生成投影纹理/深度及独立 IMU，模型真值与估计、控制目标分别保存；
   比较误差/失跟/制动，不能用目标驱动虚假估计。记录冷/热构建耗时、镜像大小和运行负载。
3. 原始真实录包两次重算：只回放输入与静态标定，不混入旧 filtered/TF/status；
   覆盖停止输入、丢帧、晚到诊断、重启与光照/低纹理，检查有效段和误差可复现。
4. 真机静止60秒、可测量纯转、前进对照：测同一机身点，不把转身时真实位移视为零。
   记录帧率、延迟分布、同步残差、CPU/RAM/温度、相机/媒体丢帧；资源限制防止相机饿死。
5. 冻结标定/参数后20次独立约1米留出试次、至少两场景；每次均计失败，报告
   signed forward/lateral/vector error 与测量不确定度。估计精度和自动制动终点精度分别验收。
   条件为 abs(估计前向-实测前向)+测量不确定度≤0.05m；目标距离不是真值。
6. 回归相机 JPEG/Z16/内参、照片录像、麦克风/声源方向/扬声器、旋转/急停/底盘互斥/ACP。
   单独停止/杀死/重启 sidecar，证明媒体仍运行。未实际构建、部署、硬件测试均标未执行。

## 当前已运行与未完成

本轮基线：原 loco_destination、direct camera capture、motus.odom 格式测试
130 passed in 57.62s。不是新融合实现的验证。本地未发现 ros2/docker/podman 命令。
未修改产品代码、未构建或推送镜像、未访问/控制机器人、未执行精度测试。

## 上游依据

- [RTAB-Map RGB-D 实现与参数](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_odom/src/nodelets/rgbd_odometry.cpp)
- [OdomInfo 原始诊断定义](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_msgs/msg/OdomInfo.msg)
- [robot_localization Humble 坐标和数据要求](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/preparing_sensor_data.rst)
- [EKF 超时预测与延迟观测处理](https://github.com/cra-ros-pkg/robot_localization/blob/humble-devel/doc/state_estimation_nodes.rst)
- [RealSense 深度尺度与投影坐标](https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0)

这些是2026-10-21查询的上游分支资料，不是已验证的部署锁文件；发布前必须锁定实测
兼容的包版本/源码提交和镜像 digest，且使用该版本再做同一套集成检查。
