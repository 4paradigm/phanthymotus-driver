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
- `navigation_status` 提供导航任务、定位与置信度报告。
- `loco` 提供 `move`、`stop_move`、`stand_up`、`stand_down`、`damp`、`gait`、`mode` 和 `manual_control`。`move` 的前后/侧向速度单位为 m/s，偏航角速度参数 `yaw_rate_deg_s` 使用度/秒；发给固件前会转换为 rad/s。`manual_control` 是常规/辅助模式的六轴归一化控制强度（-1 到 1），不是角度；普通步态只响应前后、侧移和偏航分量。通过 ASDU 执行 `stand_up` 后会自动进入 RL 运控状态，无需单独切换。持续运动指令 10Hz 刷新，到时自动发零指令；各状态切换会等待状态上报并通过 ACP 发送 action completion。
- `light` 独立设置头尾灯颜色和效果。颜色使用固件支持的黑、白、绿、蓝具名选项；协议不支持任意 RGB 值。效果使用常亮、流水、渐变、呼吸、闪烁等具名选项；仅呼吸和闪烁要求 `head_cycle_seconds` 与 `tail_cycle_seconds`，分别控制头灯和尾灯周期，单位为秒。
- 不注册 `device` 控制卡：休眠/唤醒属于电源与安全管理，不交由 Agent 自主控制；SDK 关节流通过机载配置与标准状态话题提供。
- `charging` 提供 `start_charging`、`stop_charging` 两个自主充电动作，两者都必须显式传入 `confirm=true`；卡片生命周期的 `start`/`stop` 只报告 `ready`/`idle`，不会触发充电。风险较高的 `Charge=2` 强制清除状态不开放为普通动作。
- 本设备为 S10 Pro，默认 `model_variant` 设为 `pro` 并注册 `navigation` 动作卡，包含单点任务、取消、状态、定位初始化、地图位姿和感知状态。它只是在机器人当前已加载地图中执行目标点导航，不负责创建地图；使用前必须已有可用地图且定位正常。单点导航和定位初始化必须显式传入 `confirm=true`；导航任务异步返回 action ID 并通过 ACP 报告完成、取消或错误。文档指出非 Pro 版本会以通用错误 `0xE00A` 拒绝导航功能；若将驱动用于标准版，需将配置改为 `standard` 并从 `driver.yaml` 卡片清单移除导航卡。

## 与参考驱动的差异

传感器与执行卡片沿用仓库常用命名和职责。已提供的容器日志确认 `/IMU`、`/JOINTS_DATA_10HZ`、`/BATTERY_DATA`、`/MOTION_INFO` 都收到并发布到 `/host/lynx_s10/...`；旧日志没有记录 Agent Core 的活动订阅，且旧版同进程双 context 共用 FastDDS profile。因而此前可确认源到 driver、driver 到本地发布成功，故障范围落在 domain-42 发现/订阅边界；新实现把 Agent Core 发布者放进 loopback-only 独立进程，并用有界 latest-value IPC 队列跨进程转发。需部署后再查看 `state.ros_streams` 的 bridge 队列与 Agent Core 实际订阅，才能最终确认前端恢复。

旧版雷达点云最高约 10Hz、单帧约 8.5 万点；点云被均匀限采样至 2,000 点并以紧凑字节缓冲传递，避免队列积压。S10 Pro 提供前、后、左、右四路相机，对应 `camera_rear`、`camera_left`、`camera_front`、`camera_right` 四张 sensor 卡，默认 ROS topic 为 `/usb_camera_1/image_raw`、`image_raw_0`、`image_raw_1`、`image_raw_2`；现场部署仍应以 `ros2 topic list/info` 核对实际方向和 topic。相机和雷达订阅使用 ROS sensor-data（BEST_EFFORT）QoS。每路相机和雷达都有独立的 `spawn` 媒体 worker，worker 只负责 JPEG 编码或点云限采样；任务队列最多保留 2 帧，满载时丢弃旧帧。转换结果再进入每路独立的 maxsize=1 domain-42 bridge 进程，单路高负载不会阻塞其他流或控制/状态回调。容器日志应分别出现 `first ... sample received`、`first ... sample published` 和 `[dds-bridge] first sample published`；若没有，`state.ros_streams` 可用于区分源 topic、worker/队列和跨进程发布阶段。

S10 URDF 含有指向 `meshes/*.STL` 的视觉网格引用；本次仅随镜像提供 URDF 文本，骨架关节名与运动学链可用，但不承诺展示厂家 STL 外观。`joints` 实时流使用真机 `JOINTS_DATA_10HZ` 的 16 个电机字段，并按 URDF 关节顺序映射到仓库通用 `sensor/skeleton`；`joint_state` 同时保留完整供应商消息字段。

相机卡片按实际方位命名为 `camera_rear`、`camera_left`、`camera_front`、`camera_right`；厂商 USB 输入顺序对应 tail、left、head、right。

Go1/Go2/As2W 中的语音、遥控器、SLAM、深度相机、特技、专用避障以及 `loco` 命令集都没有 S10 指南依据，因此不移植。M20 Pro 的导航卡片名称与动作边界可复用，但导航实现按 S10 自己的 ASDU 和 Pro 型号边界实现。

## ROS 2

指南还定义 Jazzy 话题 `/IMU`、`/LIDAR/POINTS`、`/MOTION_STATE`、`/GAIT`、`/MOTION_INFO`、`/NAV_CMD`、`/CHARGE_STATUS`、`/fault_aggregator`。主进程只创建机器人域 context、订阅机载话题；domain 42 由 `domain_bridge.py` 里的独立 spawned publisher 进程持有，各自只绑定 loopback FastDDS profile。消息通过按流隔离的 maxsize=1 队列传递，新状态替换旧状态，不会积压。主进程的 FastDDS profile 只供机载 ROS 发现使用，不创建 Agent Core domain 42 participant。`service.yml` 按本台 S10 Pro 机载主机配置 `eth0`、`10.21.33.0/24`（主机接口 `10.21.33.103/24`；同机 `robot_server` 在 TCP `30003` 监听）。若迁移到其他主机，需改为其真实机器人网卡与 S10 TCP 地址；机器人 DDS 网络不可与办公网或其他机器人共享。

## 部署与镜像

该驱动使用仓库 ROS 基础镜像以获得 `rclpy`/Fast DDS。Docker 多阶段构建在 builder 阶段按需安装 colcon 并编译 `drdds`，运行阶段只复制 `/lynx_s10_ws/install`，不携带源码、build 或 log 目录；运行阶段仅在基础镜像缺少时安装 `python3-pip`、Fast DDS RMW 和 `ca-certificates`。`requirements.txt` 固定 PyYAML 与 Pillow：前者读取 YAML 配置，后者把 ROS `sensor_msgs/Image` 转成前端消费的 JPEG；安装使用 `--no-cache-dir`。固定上游 revision 的 `drdds` 源码作为普通资源目录保存在 `resource/deep-robotics-msg/` 并随驱动构建；压缩 ZIP 仅本地保留且不提交。这样镜像构建无需访问 GitHub。Compose 片段使用 host 网络连接机器人网段，并包含该 S10 主机已确认的网卡和 CIDR；迁移到其他主机时必须先重新核实并修改这两个值，同时配置本地日志轮转。

## 安全与验证

- 切换步态前应确保机器人完全停止；步态会隐式切换至对应运动模式。
- 首次联调应在隔离环境进行，先验证 basic_server 响应、型号/版本和状态上报，再低速验证运动方向与状态机。该 TCP 通道不加密，须限制在可信机器人网络内。轴速度控制文档要求同一客户端连续控制，勿并发多个控制方。
- 此前真机验证 TCP 心跳成功（`ErrorCode: 0`），用户提供的旧容器日志也确认 IMU、关节、电池、运动状态跨 ROS 输入和 driver 发布。当前 turn 无法 SSH 登录真机；本次改造的跨进程 bridge、相机帧和 domain-42 前端订阅恢复均仍需部署镜像后现场验收。
