# 云深处山猫 S10 Driver

本 Driver 对照山猫 S10《软件开发指南》V1.0.2，实现原生 `PatrolDevice` JSON over TLS/TCP。卡片命名兼顾 Lynx M20 的 `state`、`motion_info`、`hard_estop`、`charge_status`、`motion`、`charging`、`device`，以及 Go1/Go2/As2W 常见的 `imu`、`joints`、`battery`、`errors`、`navigation`。

## 协议

- MCP 服务端使用端口 `15724`（避免与 AimDK X2 的 `15717` 冲突）；机器人端默认 TLS/TCP 服务端 `10.21.33.103:30003`。证书校验保持开启；私有 CA 可配置在 `basic_server.ca_cert`。文档也列出 DTLS/UDP `30004`，但本驱动目前只实现标准库支持的 TLS/TCP，不降级明文。
- 16 字节头按格式字段 `JSON=1`、分包序号、协议版本 `1`、5 字节保留区 `0` 编码，单帧 ASDU 最大 1000 字节，分包总长最大 65535 字节；请求响应以报文 ID 和 Type/Command 关联。
- TLS 会话保持接收线程处理主动上报和请求响应，心跳 1Hz，驱动将状态报告按 `Type:Command` 缓存。主要报告为基础状态 2Hz、运控/关节 10Hz、设备状态 2Hz、故障 2Hz、导航状态 1Hz。

## 卡片

- `state` 汇总连接与所有已收到的设备主动报告。
- `motion_info` 提供运控报告中的速度、姿态、运动状态和剩余里程；`imu` 从同一报告提取加速度、角速度和姿态（原生协议频率 10Hz）；`joints` 提供 16 个轮足关节角度。
- `battery` 提供电池列表；`native_device_status` 提供设备温度、雷达/视频使能和 CPU 状态；`native_basic_status` 提供运动、步态、HES、电源模式、休眠与充电状态；`hard_estop` 和 `charge_status` 是便于编排的窄卡片。
- `errors` 提供原生协议聚合故障列表；`navigation_status` 提供导航任务、定位与置信度报告。
- `motion` 提供站立、趴下、阻尼、RL 控制、步态、使用模式、归一化轴和真实速度控制。轴指令仅常规/辅助模式有效；真实速度仅导航模式有效；两种速度报文不等待响应。速度以 10Hz 持续刷新，时长默认 1 秒并自动归零（`seconds=0` 持续到显式 `stopmotion`）；停止会同时向轴与真实速度接口发送完整零向量。
- `device` 提供头尾灯语、休眠/唤醒、心跳断联后自动休眠时间和 SDK 模式。自定义灯语按官方 `CustomMode` 与 `Led` 列表结构编码；SDK 频率限制为 1–200 Hz 且必须整除 1000。
- `charging` 提供 `start_charging`、`stop_charging` 两个自主充电动作；卡片生命周期的 `start`/`stop` 只报告 `ready`/`idle`，不会触发充电。风险较高的 `Charge=2` 强制清除状态不开放为普通动作。
- 本设备为 S10 Pro，默认 `model_variant` 设为 `pro` 并注册 `navigation` 动作卡，包含单点任务、取消、状态、定位初始化、地图位姿和感知状态。文档指出非 Pro 版本会以通用错误 `0xE00A` 拒绝导航功能；若将驱动用于标准版，需将配置改为 `standard` 并从 `driver.yaml` 卡片清单移除导航卡。

## 与参考驱动的差异

传感器和执行卡片沿用仓库常用命名与职责划分，但 S10 文档没有可供外部主机订阅的相机 URL、视频协议或流话题，故不提供 `camera` 卡。`/LIDAR/POINTS` 明确限制为仅机器人主机传输，故不提供外接 `lidar` 卡。文档的 `/IMU` 为 200Hz ROS 2 `sensor_msgs/Imu`，但本驱动当前的 `imu` 卡来自 10Hz 原生运控状态，而非 ROS 2 话题；如需 200Hz 原始话题，应在 ROS 2 Jazzy 环境单独接入。关节数据同理来自原生 10Hz 状态，不冒称 SDK 模式关节流或 ROS 话题。

Go1/Go2/As2W 中的语音、遥控器、SLAM、深度相机、特技、专用避障以及 `loco` 命令集都没有 S10 指南依据，因此不移植。M20 Pro 的导航卡片名称与动作边界可复用，但导航实现按 S10 自己的 ASDU 和 Pro 型号边界实现。

## ROS 2

指南还定义 Jazzy 话题 `/IMU`、`/LIDAR/POINTS`、`/MOTION_STATE`、`/GAIT`、`/MOTION_INFO`、`/NAV_CMD`、`/CHARGE_STATUS`、`/fault_aggregator`。当前 bundle 仅实现原生 TLS/TCP 传输，没有启动 ROS 节点；这是为了不把 Jazzy 自定义接口误装进仓库当前 ROS 基础镜像。点云话题即使启动 ROS 也仅限机器人主机传输。构建镜像不需要 ROS，按仓库通用 Docker driver 机制运行 MCP 服务。

## 部署与镜像

该驱动只使用 Python 标准库 TLS/TCP，不依赖 ROS，因此基于 `python:3.11-slim`。镜像保留 `ca-certificates` 用于系统 CA 链的 TLS 校验，PyYAML 用于通用 `load_config` 读取 YAML；两项依赖分别支持安全连接和配置加载。具体镜像体积以 CI 构建结果为准。Compose 片段使用 host 网络连接机器人网段，并配置本地日志轮转，不挂载 DDS 配置或设备目录。

## 安全与验证

- 切换步态前应确保机器人完全停止；步态会隐式切换至对应运动模式。
- 首次联调应在隔离环境进行，先验证 TLS 证书、型号/版本和状态上报，再低速验证运动方向与状态机。轴速度控制文档要求同一客户端连续控制，勿并发多个控制方。
- 已执行协议帧、编解码、流解码、卡片契约和 Python 语法验证。尚未在 S10 真机验证 TLS 证书链、请求响应字段兼容、ROS 话题、运动、导航、灯光和充电。
