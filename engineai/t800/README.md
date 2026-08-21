# EngineAI T800 Development Edition Driver

T800 的完整 Phanthy Motus MCP driver。机器人侧使用 ROS2 Humble、CycloneDDS、
Domain 69；Agent Core 数据流使用 Domain 42。驱动兼容两种部署方式：

1. 众擎自带 ROS2/Native SDK runtime 已运行，driver 只连接公开 ROS2 接口。
2. driver 通过 `native_sdk` 工具管理 Native SDK 子进程或 `robotics.service`。

协议基线：

- `engineai_ros2_workspace` community commit `ebd638e31709a038d3208517693d33174dbacb46`
- `engineai_robotics_native_sdk` commit `83204a459e0e786f855235a8507197496a79acc7`

## 工具

| 工具 | 类型 | 能力 |
|---|---|---|
| `joints` | sensor | 25 个关节的位置、速度、力矩及骨架数据流 |
| `imu` | sensor | 四元数、RPY、角速度、线加速度 |
| `battery` | sensor | 电源使能、电量、电压、电流、错误码 |
| `motor_health` | sensor | 电机/MOS 温度、电压、电流、掉线、使能及错误码 |
| `motor_state` | sensor | Native SDK 原始电机位置、速度和力矩 |
| `motor_command` | sensor | Native SDK 原始电机控制命令 |
| `joint_command_feedback` | sensor | Native SDK 最近关节控制命令反馈 |
| `gamepad` | sensor | 遥控器连接、按键和摇杆状态 |
| `motion_state` | sensor | 当前 Native SDK motion state 和允许转换 |
| `driver_health` | actuator | 每次执行返回一次机器人、麦克风与 Odin2 点云/双目/深度数据流的最新健康 JSON，不持续发布 |
| `robot_snapshot` | sensor | 运动、关节、IMU、电源和电机健康聚合快照 |
| `fault_summary` | sensor | 电机掉线/禁用/错误/过温及电源错误摘要 |
| `stability` | sensor | 基于 IMU 的倾斜和跌倒风险估计 |
| `joint_groups` | sensor | 腿、躯干、双臂、头部和全身关节名称/索引映射 |
| `capabilities` | sensor | Driver 能力发现、原生状态和已知限制 |
| `ros_graph` | sensor | 实时发现固件节点、topic、service 和尚未映射的新接口 |
| `model` | resource | 官方 `serial_t800.urdf` |
| `loco` | actuator | 100 Hz 速度控制；定时/持续、相对位移、转角和圆弧开环动作 |
| `motion_mode` | actuator | 任意状态切换及 idle/passive/站立/行走/舞蹈/起身/躺下快捷动作 |
| `gait` | actuator | 基于 Native SDK motion state 的步态选择；自动适配 `rl_basic`/`walk` 版本差异 |
| `dance` | actuator | 舞蹈列表、播放、停止和状态；官方基线为 `dance.mnn` + `dance.npz` |
| `joint_plan` | actuator | 索引/名称关节轨迹、头部/单臂姿态、当前位置保持、取消、复位和预置动作 |
| `motion_recorder` | actuator | 按指定采样率录制关节轨迹，手动/定时停止均自动落盘，并支持管理与回放 |
| `joint_plan_state` | sensor | 规划 request id、状态和进度 |
| `gesture` | actuator | 官方完整挥手/握手多步序列及任意自定义关节动作队列 |
| `joint_override` | actuator | 指定关节 100 Hz 覆盖控制 |
| `joint_bridge` | actuator | 全 25 关节最高 500 Hz 底层控制 |
| `led` | actuator | 众擎协议定义的 11 种灯效 |
| `tts` | actuator | 众擎 TTS 消息；topic 可配置 |
| `mic` | sensor | 内置麦克风 PCM-16 16kHz 采集，缓冲 1024 字节发布（满足 perception ASR 协议） |
| `speaker` | actuator | 订阅画布连接的 `audio/pcm-16k` 流，经官方 ALSA `aplay` 接口播放到机器人喇叭；音量经官方 `pactl` 接口控制 |
| `pointcloud` | sensor | Odin2 原始/SLAM 点云转发（`sensor/pointcloud` 二进制渲染流） |
| `camera` | sensor | Odin2 双目 JPEG 图像转发（左/右目 `image/jpeg` 流） |
| `depth` | sensor | Odin2 官方标定深度图转 640×480 毫米 16UC1（`image/depth-z16`） |
| `motor_power` | actuator | 电机 enable/disable 服务 |
| `native_node_control` | actuator | Native SDK 已注册 LogicNode 的动态 start/stop |
| `virtual_gamepad` | actuator | Native SDK LCM 虚拟手柄：12 按键、6 模拟量和 7 种官方组合键 |
| `safety` | actuator | 零速度、覆盖释放、关节阻尼及 passive/idle/stand 组合动作 |
| `native_sdk` | actuator | Native SDK status/start/stop/restart |

所有动作差异通过 `x-action-params` 声明。`loco` 始终要求机器人处于
`rl_basic` 或 `lower_body_balance`，运行中一旦离开这两个状态会立即归零停流。
`force=true` 仅保留给 joint override 和 joint bridge 的专家级接口。

`loco.move_displacement`、`turn_angle` 和 `arc` 由速度乘时间换算。T800
基础运动协议没有供控制闭环使用的定位反馈，因此它们仍是开环动作并返回
`open_loop: true`。若 Odin2 固件提供配置中的 odometry topic，
`motion_command_trace` 会把它用于状态显示，但不会据此闭环控制动作。
有限时长动作最多运行 3 秒且不建立 ACP 屏障；更长运动请拆分命令，或使用
`duration=-1` 持续发送并通过 `stop_move` 手动停止。这样停止动作不会被屏障拦截。

`gesture.play` 与旧的 `joint_plan.preset` 不同：前者执行官方示例里的完整多步
动作（挥手包含准备、举手、5 次摆动和复位；握手包含伸手、收手和复位），
后者保留为兼容接口，只发送单个目标姿势。`gesture.sequence` 可提交任意多步
关节动作队列。

`virtual_gamepad` 使用 Native SDK 官方通道
`virtual_gamepad/gamepad_keys`，默认连接 `udpm://239.255.76.67:7667?ttl=1`。
除了原始按键/摇杆外，提供 idle、passive、stand、walk、dance、get_up、
lie_down 组合键。LCM 输入会覆盖实体手柄输入，发送完成后 Driver 自动发布
全零包释放控制权。

`gait` 不会写入自定义 `gait.json`。官方 T800 Native SDK 的行走
策略配置位于 `assets/config/t800/.../*.yaml`，且不提供 `step_height`、
`stride_length` 等通用运行时调参契约。因此卡片只通过官方
`/motion/set_motion_state` 接口切换 `basic` / `balanced`，并以
`/motion/motion_state` 返回的可转换状态为准。`rl_terrain` 不属于当前 T800
状态机，因此不会作为可选项暴露；terrain 接口示例不能视为 T800 固件能力。

`motion_recorder.record_start` 是幂等的：录制中重复调用只返回当前会话，
不会意外停止。`record_stop` 同样可重复调用；设置 `duration > 0`
时超时会走与手动停止相同的落盘路径。状态中的 `last_recording`
可用于确认最近一次保存文件、停止原因和帧数。

录制与回放仅在 `lower_body_balance` 状态开放。回放不再把每个 20Hz 录制帧
提交为独立 joint plan，而是先用 0.5 秒五次曲线从当前位置平滑接入，再以
三次 Hermite 插值重采样为 100Hz `JointOverrideCommand` 连续轨迹；停止、
异常和完成路径都会发布 `weight=0` 释放覆盖。录制或回放完成后，状态返回
`needs_reset=true`，必须执行 `reset`：安全进入 `lower_body_balance` 并发送
官方 `REQUEST_RESET` 默认姿态请求。规划器确认回到 `IDLE` 后才允许下一次录制。

`pointcloud`、`camera`、`depth` 桥接 T800-Odin2 激光雷达相机（飞书文档
7.2 节）在 Orin 主板上发布的 `odin_ros_driver` topic。点云按
`[uint32 point_step][uint32 total_points][PointCloud2 bytes]` 二进制格式
重发布到 `sensor/pointcloud` 流；双目压缩图原样转发。深度订阅官方
`pcd2depth_ros2_node` 发布的 `/manifold/ODIN2/device0/depth`（`32FC1`、米），
转换为固定 640×480 的毫米 `16UC1`；点云到相机坐标系的标定投影、膨胀、
Sobel 边缘抑制和最近邻上采样均由众擎节点完成。使用 `depth` 前需按众擎
文档 7.2 节启动该深度图节点。
点云源可用 `pointcloud` 工具的 `select_source` action 在 `raw`/`slam`
之间切换。Odin2 topic 带逐设备前缀 `/{topic_prefix}/{model}/device{N}/`，
默认按 `config.yaml:topics.vision_*` 的 `/manifold/ODIN2/device0` 订阅，
上机前请用 `ros_graph` 工具核对实际前缀。

`speaker` 按众擎飞书《ROS2 接口开发文档》第8章实现：播放走官方 ALSA
接口 `aplay`（`-t raw -f S16_LE -r 16000 -c 1`，从 stdin 流式播放），
系统音量走官方 `pactl` 接口（`get-sink-volume`/`set-sink-volume
@DEFAULT_SINK@`，0-100）。画布把音频文件解码与用户 mic 采集统一转为
`audio/pcm-16k` 块流发布到 `topic_in`（与 G1 speaker 契约一致，含
utterance 结束的 8 字节 EOF magic），driver 只负责流式播放。镜像已含
`alsa-utils`；容器经 `-v /dev:/dev` 挂载声卡节点。

实时性：镜像内置 `/etc/asound.conf` 把 ALSA 默认设备路由到宿主
PulseAudio——这是官方「aplay 播放 + pactl 音量」模型成立的前提
（dmix 直通硬件时 pactl 音量不作用于播放输出，且 dmix 默认 ~341ms
缓冲会造成明显延迟）。`aplay` 带 `--buffer-time=100000 --period-time=20000`
压低读前缓冲；部署侧设置 `PULSE_LATENCY_MSEC=40` 控制 PulseAudio
tsched 延迟上限（见 `deploy/service.yml`）。

## 运行

机器人必须通过主机内置以太网口访问；官方默认 ROS Domain 为 69。

```bash
cd engineai/t800
docker build -t engineai-t800-driver .
docker run --rm --network host --privileged \
  -v /dev:/dev \
  -v ${T800_NATIVE_SDK_DIR:-/opt/engineai/native_sdk}:/opt/engineai/native_sdk \
  -v ${T800_PULSE_RUNTIME_DIR:-/run/user/1000/pulse}:/run/user/1000/pulse \
  -v ${T800_PULSE_CONFIG_DIR:-/home/ubuntu/.config/pulse}:/root/.config/pulse:ro \
  -e NETWORK_INTERFACE=${T800_NETWORK_INTERFACE:-eth1} \
  -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
  engineai-t800-driver
```

或在仓库根目录执行：

```bash
./build.sh engineai/t800
```

健康检查：`GET http://localhost:15708/health`；MCP 入口：
`POST http://localhost:15708/mcp`。

## Native SDK 模式

`config.yaml` 中的 `plugins.native_sdk.mode` 支持：

- `external`：默认，只报告外部 runtime 状态，不管理进程。
- `process`：在 `workdir` 中运行配置的 `command`。
- `systemd`：通过 host PID namespace 管理 `robotics.service`。

设置 `autostart: true` 可在 driver 启动时启动 Native SDK；设置
`stop_on_exit: true` 可在 driver 退出时停止由该配置管理的 runtime。

## 实机校准项

飞书私有文档和实际固件可能调整 topic 或状态名。首次上机前需要核对：

- `ros2 topic list -t` 与 `config.yaml:topics`；
- Odin2 实际 topic 前缀（`/{topic_prefix}/{model}/device{N}/`，默认按
  `/manifold/ODIN2/device0` 配置）；
- `/hardware/joint_state` 数组顺序是否仍为 J00..J24；
- TTS 的实际 topic；
- `motion_state.available_transition_motions` 返回的固件状态名；
- `/motion/node_control` 是否由当前 Native SDK 配置启用；
- 开发版的速度、刚度、阻尼和力矩允许范围。

低层控制要求机器人处于对应 Native SDK 状态。测试 joint bridge、覆盖控制、
起身或躺下时，应先悬挂机器人并由现场人员持有急停遥控器。
