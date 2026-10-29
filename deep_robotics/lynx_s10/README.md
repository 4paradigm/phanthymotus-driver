# 云深处山猫 S10 Driver

本 Driver 对照山猫 S10《软件开发指南》V1.0.2，并在 S10 Pro 真机上验证，实现原生 `PatrolDevice` JSON over TCP。部署主机是 S10 的 RK3588 机载 Linux 主机（SSH/WLAN 地址 `10.100.129.209`，机器人侧 `eth0` 地址 `10.21.33.103/24`）；机载 `robot_server` 进程在该主机监听 TCP `30003`，因此当前 host-network 容器连接 `10.21.33.103:30003` 是连接同机服务，不是把两个独立主机配置成同一 IP。状态卡尽量采用 Go1/Go2/As2W 常见的 `imu`、`loco_state`、`joints`、`joint_state`、`battery` 命名；协议中无法归并的故障和 Pro 导航状态保留为独立卡。

## 协议

- MCP 服务端使用端口 `15724`（避免与 AimDK X2 的 `15717` 冲突）；机载 `robot_server` 提供普通 TCP basic_server，监听 `0.0.0.0:30003`，当前主机通过 `10.21.33.103:30003` 访问；分离部署时 `S10_BASIC_SERVER_HOST` 必须改为机载 S10 主机的实际可达地址。真机实测表明该端口接受明文帧，TLS 握手超时；向其发送正确的心跳 Type/Command 后返回 `ErrorCode: 0`。文档另列 DTLS/UDP `30004`，当前驱动尚未实现该通道。
- 16 字节头按格式字段 `JSON=1`、分包序号、协议版本 `1`、5 字节保留区 `0` 编码，单帧 ASDU 最大 1000 字节，分包总长最大 65535 字节；请求响应以报文 ID 和 Type/Command 关联。
- TCP 会话保持接收线程处理主动上报和请求响应，心跳 1Hz，驱动将状态报告按 `Type:Command` 缓存。主要报告为基础状态 2Hz、运控/关节 10Hz、设备状态 2Hz、故障 2Hz、导航状态 1Hz。

## 卡片

- `state` 汇总连接、最近错误与所有已收到的设备主动报告，适合诊断传输是否收到数据。
- `model` 返回官方 `S10.urdf`；`joints` 按 URDF 的固定 16 关节顺序返回 `sensor/skeleton` 结构（q/dq/tau/temperature），并在运控姿态可用时附带 `imu_quat`。`imu` 从运控报告提取加速度、角速度和姿态；`joint_state` 保留原生电机状态；`loco_state` 合并运控和基础状态报告；`battery` 提供电池列表。
- 急停、充电及设备健康状态分别保留在 `loco_state`、`battery` 或汇总 `state`，不再拆成多张厂商专用状态卡。
- `errors` 提供原生协议聚合故障列表；`navigation_status` 提供导航任务、定位与置信度报告。
- `loco` 沿用 As2W/Go2 的执行器命名，基础动作使用 `move`、`stop_move`、`stand_up`、`stand_down`、`damp`。S10 专属的 `gait`、`mode`、归一化 `axis` 和 `rl_control` 也在同一卡片中；按该卡片的直接控制语义不要求 `confirm`。`stand_up/down`、步态和模式命令按 fire-and-forget 发送，不等待固件可能不返回的应答。`move` 会先切换至导航模式和平地步态，再发送速度报文；速度以 10Hz 刷新，`duration` 默认 1 秒后归零，最长 60 秒；兼容输入 `duration=-1` 也会在 60 秒后自动归零。速度控制方向及模式仍须在真机安全验证。
- `device` 提供头尾灯语、休眠/唤醒、心跳断联后自动休眠时间和 SDK 模式。休眠、唤醒、自动休眠配置和 SDK 模式更改必须显式传入 `confirm=true`。自定义灯语按官方 `CustomMode` 与 `Led` 列表结构编码；SDK 频率限制为 1–200 Hz 且必须整除 1000。
- `charging` 提供 `start_charging`、`stop_charging` 两个自主充电动作，两者都必须显式传入 `confirm=true`；卡片生命周期的 `start`/`stop` 只报告 `ready`/`idle`，不会触发充电。风险较高的 `Charge=2` 强制清除状态不开放为普通动作。
- 本设备为 S10 Pro，默认 `model_variant` 设为 `pro` 并注册 `navigation` 动作卡，包含单点任务、取消、状态、定位初始化、地图位姿和感知状态。单点导航和定位初始化必须显式传入 `confirm=true`；导航任务异步返回 action ID 并通过 ACP 报告完成、取消或错误。文档指出非 Pro 版本会以通用错误 `0xE00A` 拒绝导航功能；若将驱动用于标准版，需将配置改为 `standard` 并从 `driver.yaml` 卡片清单移除导航卡。

## 与参考驱动的差异

传感器和执行卡片沿用仓库常用命名与职责划分。真机 ROS graph 中 `/IMU`、`/JOINTS_DATA_10HZ`、`/BATTERY_DATA` 有发布样本，`/MOTION_INFO` 有发布者；驱动从机器人 ROS 域订阅，并通过 Agent Core ROS 域转发 `imu`、`joints`、`joint_state`、`loco_state`、`battery` 和 `lidar` 实时流。传感器卡的 `start/info` 响应会返回其 `topic_out`，供前端建立流订阅。虽然遥控器双激光雷达开关已开启，现场 `/LIDAR/POINTS` 当前仍无发布者；检查到的 `/DEPTH_POINTS`、`/LIO_ALIGNED_POINTS`、`/SEG_CLOUD` 和 `/cloud_now_g` 在 ROS graph 中有过 publisher discovery，但抽样期间没有发出消息，不能视作可用雷达流或直接替换话题。需进一步查设备雷达驱动为何未向其文档话题发布。MCP 查询仍使用原生 TCP 缓存。手册没有可供外部主机稳定订阅的相机 URL/视频协议，故暂不提供 `camera` 卡。

S10 URDF 含有指向 `meshes/*.STL` 的视觉网格引用；本次仅随镜像提供 URDF 文本，骨架关节名与运动学链可用，但不承诺展示厂家 STL 外观。`joints` 实时流使用真机 `JOINTS_DATA_10HZ` 的 16 个电机字段，并按 URDF 关节顺序映射到仓库通用 `sensor/skeleton`；`joint_state` 同时保留完整供应商消息字段。

Go1/Go2/As2W 中的语音、遥控器、SLAM、深度相机、特技、专用避障以及 `loco` 命令集都没有 S10 指南依据，因此不移植。M20 Pro 的导航卡片名称与动作边界可复用，但导航实现按 S10 自己的 ASDU 和 Pro 型号边界实现。

## ROS 2

指南还定义 Jazzy 话题 `/IMU`、`/LIDAR/POINTS`、`/MOTION_STATE`、`/GAIT`、`/MOTION_INFO`、`/NAV_CMD`、`/CHARGE_STATUS`、`/fault_aggregator`。bundle 使用仓库统一的 `run_driver` 双域 ROS 运行时：机器人域订阅标准消息，Agent Core 域发布 `topic_out`；原生 TCP 仍负责动作和设备状态。Fast DDS profile 在创建 ROS context 前生成，两个 ROS context 共用这个进程级 profile。当前 `service.yml` 按已查询的 S10 机载主机配置 `NETWORK_INTERFACE=eth0`、`ROBOT_NETWORK_CIDR=10.21.33.0/24`（主机接口 `10.21.33.103/24`；同一机载主机上的 `robot_server` 监听 `30003`，`basic_server` 地址配置为 `10.21.33.103`）。启动时会校验接口 IPv4、其直连子网和 basic_server 地址均位于该私有网段，且 CIDR 不宽于 `/24`；不匹配时拒绝启动。若将驱动迁移到独立主机，须改用 S10 机载主机的可达 TCP 地址，并确认 DDS 机器人网络隔离边界，不能使用办公网、共享 LAN 或与其他机器人共用的网络。由于该进程级 profile 对两个 context 生效，指定网段上的 DDS 流量包含 Agent Core domain-42；若网络不能隔离，不能部署此双域进程。`ROS_LOCALHOST_ONLY=0` 使选定网卡上的 ROS 发现保持可用。

## 部署与镜像

该驱动使用仓库 ROS 基础镜像以获得 `rclpy`/Fast DDS。Docker 多阶段构建在 builder 阶段按需安装 colcon 并编译 `drdds`，运行阶段只复制 `/lynx_s10_ws/install`，不携带源码、build 或 log 目录；运行阶段仅在基础镜像缺少时安装 `python3-pip`、Fast DDS RMW 和 `ca-certificates`。PyYAML 由 `requirements.txt` 管理并以 `--no-cache-dir` 安装。固定上游 revision 的 `drdds` 源码作为普通资源目录保存在 `resource/deep-robotics-msg/` 并随驱动构建；压缩 ZIP 仅本地保留且不提交。这样镜像构建无需访问 GitHub。Compose 片段使用 host 网络连接机器人网段，并包含该 S10 主机已确认的网卡和 CIDR；迁移到其他主机时必须先重新核实并修改这两个值，同时配置本地日志轮转。

## 安全与验证

- 切换步态前应确保机器人完全停止；步态会隐式切换至对应运动模式。
- 首次联调应在隔离环境进行，先验证 basic_server 响应、型号/版本和状态上报，再低速验证运动方向与状态机。该 TCP 通道不加密，须限制在可信机器人网络内。轴速度控制文档要求同一客户端连续控制，勿并发多个控制方。
- 已在 S10 Pro 真机验证 TCP 心跳成功（`ErrorCode: 0`）；状态报告、运动、导航、灯光和充电仍需进一步验证。双域 DDS 使用 driver-owned 受限 profile；尚未在目标部署网络验证 `eth0` 是否为机器人专用网段，因此须在部署前确认网段隔离。
