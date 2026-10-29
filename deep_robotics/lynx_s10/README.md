# 云深处山猫 S10 Driver

本 Driver 对照山猫 S10《软件开发指南》V1.0.2，并在 S10 Pro 真机 `10.21.33.103:30003` 上验证，实现原生 `PatrolDevice` JSON over TCP。状态卡尽量采用 Go1/Go2/As2W 常见的 `imu`、`loco_state`、`joints`、`joint_state`、`battery` 命名；协议中无法归并的故障和 Pro 导航状态保留为独立卡。

## 协议

- MCP 服务端使用端口 `15724`（避免与 AimDK X2 的 `15717` 冲突）；机器人端 basic_server 为普通 TCP 服务端 `10.21.33.103:30003`，部署时可用 `S10_BASIC_SERVER_HOST` 覆盖主机地址。真机实测表明该端口接受明文帧，TLS 握手超时；向其发送正确的心跳 Type/Command 后返回 `ErrorCode: 0`。文档另列 DTLS/UDP `30004`，当前驱动尚未实现该通道。
- 16 字节头按格式字段 `JSON=1`、分包序号、协议版本 `1`、5 字节保留区 `0` 编码，单帧 ASDU 最大 1000 字节，分包总长最大 65535 字节；请求响应以报文 ID 和 Type/Command 关联。
- TCP 会话保持接收线程处理主动上报和请求响应，心跳 1Hz，驱动将状态报告按 `Type:Command` 缓存。主要报告为基础状态 2Hz、运控/关节 10Hz、设备状态 2Hz、故障 2Hz、导航状态 1Hz。

## 卡片

- `state` 汇总连接、最近错误与所有已收到的设备主动报告，适合诊断传输是否收到数据。
- `model` 返回官方 `S10.urdf`；`joints` 按 URDF 的固定 16 关节顺序返回 `sensor/skeleton` 结构（q/dq/tau/temperature），并在运控姿态可用时附带 `imu_quat`。`imu` 从运控报告提取加速度、角速度和姿态；`joint_state` 保留原生电机状态；`loco_state` 合并运控和基础状态报告；`battery` 提供电池列表。
- 急停、充电及设备健康状态分别保留在 `loco_state`、`battery` 或汇总 `state`，不再拆成多张厂商专用状态卡。
- `errors` 提供原生协议聚合故障列表；`navigation_status` 提供导航任务、定位与置信度报告。
- `loco` 沿用 As2W/Go2 的执行器命名，基础动作使用 `move`、`stop_move`、`stand_up`、`stand_down`、`damp`。S10 专属的 `gait`、`mode`、归一化 `axis` 和 `rl_control` 也在同一卡片中。真实速度仅导航模式有效；速度报文不等待响应，以 10Hz 刷新，`duration` 默认 1 秒后归零，`duration=-1` 持续至 `stop_move`。
- `device` 提供头尾灯语、休眠/唤醒、心跳断联后自动休眠时间和 SDK 模式。自定义灯语按官方 `CustomMode` 与 `Led` 列表结构编码；SDK 频率限制为 1–200 Hz 且必须整除 1000。
- `charging` 提供 `start_charging`、`stop_charging` 两个自主充电动作；卡片生命周期的 `start`/`stop` 只报告 `ready`/`idle`，不会触发充电。风险较高的 `Charge=2` 强制清除状态不开放为普通动作。
- 本设备为 S10 Pro，默认 `model_variant` 设为 `pro` 并注册 `navigation` 动作卡，包含单点任务、取消、状态、定位初始化、地图位姿和感知状态。文档指出非 Pro 版本会以通用错误 `0xE00A` 拒绝导航功能；若将驱动用于标准版，需将配置改为 `standard` 并从 `driver.yaml` 卡片清单移除导航卡。

## 与参考驱动的差异

传感器和执行卡片沿用仓库常用命名与职责划分，但 S10 文档没有可供外部主机订阅的相机 URL、视频协议或流话题，故不提供 `camera` 卡。`/LIDAR/POINTS` 明确限制为仅机器人主机传输，故不提供外接 `lidar` 卡。文档的 `/IMU` 为 200Hz ROS 2 `sensor_msgs/Imu`，但本驱动当前的 `imu` 卡来自 10Hz 原生运控状态，而非 ROS 2 话题；如需 200Hz 原始话题，应在 ROS 2 Jazzy 环境单独接入。关节数据同理来自原生 10Hz 状态，不冒称 SDK 模式关节流或 ROS 话题。

S10 URDF 含有指向 `meshes/*.STL` 的视觉网格引用；本次仅随镜像提供 URDF 文本，骨架关节名与运动学链可用，但不承诺展示厂家 STL 外观。`joints` 通过 MCP 读取最近一份原生关节上报，不创建仓库其他 ROS/DDS 驱动使用的 `topic_out` 实时流。

Go1/Go2/As2W 中的语音、遥控器、SLAM、深度相机、特技、专用避障以及 `loco` 命令集都没有 S10 指南依据，因此不移植。M20 Pro 的导航卡片名称与动作边界可复用，但导航实现按 S10 自己的 ASDU 和 Pro 型号边界实现。

## ROS 2

指南还定义 Jazzy 话题 `/IMU`、`/LIDAR/POINTS`、`/MOTION_STATE`、`/GAIT`、`/MOTION_INFO`、`/NAV_CMD`、`/CHARGE_STATUS`、`/fault_aggregator`。当前 bundle 仅实现原生 TCP 传输，没有启动 ROS 节点；这是为了不把 Jazzy 自定义接口误装进仓库当前 ROS 基础镜像。点云话题即使启动 ROS 也仅限机器人主机传输。构建镜像不需要 ROS，按仓库通用 Docker driver 机制运行 MCP 服务。

## 部署与镜像

该驱动使用 Python 标准库 TCP，不依赖 ROS，因此基于 `python:3.11-slim`。PyYAML 用于通用 `load_config` 读取 YAML。具体镜像体积以 CI 构建结果为准。Compose 片段使用 host 网络连接机器人网段，并配置本地日志轮转，不挂载 DDS 配置或设备目录。

## 安全与验证

- 切换步态前应确保机器人完全停止；步态会隐式切换至对应运动模式。
- 首次联调应在隔离环境进行，先验证 basic_server 响应、型号/版本和状态上报，再低速验证运动方向与状态机。该 TCP 通道不加密，须限制在可信机器人网络内。轴速度控制文档要求同一客户端连续控制，勿并发多个控制方。
- 已在 S10 Pro 真机验证 TCP 心跳成功（`ErrorCode: 0`）；状态报告、运动、导航、灯光和充电仍需进一步验证。
