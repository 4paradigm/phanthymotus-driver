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
| `driver_health` | sensor | 各 ROS2 数据源连接与新鲜度 |
| `robot_snapshot` | sensor | 运动、关节、IMU、电源和电机健康聚合快照 |
| `fault_summary` | sensor | 电机掉线/禁用/错误/过温及电源错误摘要 |
| `stability` | sensor | 基于 IMU 的倾斜和跌倒风险估计 |
| `joint_groups` | sensor | 腿、躯干、双臂、头部和全身关节名称/索引映射 |
| `capabilities` | sensor | Driver 能力发现、原生状态和已知限制 |
| `ros_graph` | sensor | 实时发现固件节点、topic、service 和尚未映射的新接口 |
| `model` | resource | 官方 `serial_t800.urdf` |
| `loco` | actuator | 100 Hz 速度控制；定时/持续、相对位移、转角和圆弧开环动作 |
| `motion_mode` | actuator | 任意状态切换及 idle/passive/站立/行走/舞蹈/起身/躺下快捷动作 |
| `dance` | actuator | 舞蹈列表、播放、停止和状态；官方基线为 `dance.mnn` + `dance.npz` |
| `joint_plan` | actuator | 索引/名称关节轨迹、头部/单臂姿态、当前位置保持、取消、复位和预置动作 |
| `joint_plan_state` | sensor | 规划 request id、状态和进度 |
| `gesture` | actuator | 官方完整挥手/握手多步序列及任意自定义关节动作队列 |
| `joint_override` | actuator | 指定关节 100 Hz 覆盖控制 |
| `joint_bridge` | actuator | 全 25 关节最高 500 Hz 底层控制 |
| `led` | actuator | 众擎协议定义的 11 种灯效 |
| `tts` | actuator | 众擎 TTS 消息；topic 可配置 |
| `motor_power` | actuator | 电机 enable/disable 服务 |
| `native_node_control` | actuator | Native SDK 已注册 LogicNode 的动态 start/stop |
| `virtual_gamepad` | actuator | Native SDK LCM 虚拟手柄：12 按键、6 模拟量和 7 种官方组合键 |
| `safety` | actuator | 零速度、覆盖释放、关节阻尼及 passive/idle/stand 组合动作 |
| `native_sdk` | actuator | Native SDK status/start/stop/restart |

所有动作差异通过 `x-action-params` 声明。`force=true` 可绕过 locomotion、
joint override 和 joint bridge 的 motion-state 门禁；完整高风险能力没有从
MCP schema 中隐藏。

`loco.move_displacement`、`turn_angle` 和 `arc` 由速度乘时间换算。当前官方
T800 协议没有里程计/定位反馈，因此它们是开环动作，返回结果会明确携带
`open_loop: true`，不能当作闭环导航精度承诺。

`gesture.play` 与旧的 `joint_plan.preset` 不同：前者执行官方示例里的完整多步
动作（挥手包含准备、举手、5 次摆动和复位；握手包含伸手、收手和复位），
后者保留为兼容接口，只发送单个目标姿势。`gesture.sequence` 可提交任意多步
关节动作队列。

`virtual_gamepad` 使用 Native SDK 官方通道
`virtual_gamepad/gamepad_keys`，默认连接 `udpm://239.255.76.67:7667?ttl=1`。
除了原始按键/摇杆外，提供 idle、passive、stand、walk、dance、get_up、
lie_down 组合键。LCM 输入会覆盖实体手柄输入，发送完成后 Driver 自动发布
全零包释放控制权。

## 运行

机器人必须通过主机内置以太网口访问；官方默认 ROS Domain 为 69。

```bash
cd engineai/t800
docker build -t engineai-t800-driver .
docker run --rm --network host --privileged \
  -e NETWORK_INTERFACE=eth0 \
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
- `/hardware/joint_state` 数组顺序是否仍为 J00..J24；
- TTS 的实际 topic；
- `motion_state.available_transition_motions` 返回的固件状态名；
- `/motion/node_control` 是否由当前 Native SDK 配置启用；
- 开发版的速度、刚度、阻尼和力矩允许范围。

低层控制要求机器人处于对应 Native SDK 状态。测试 joint bridge、覆盖控制、
起身或躺下时，应先悬挂机器人并由现场人员持有急停遥控器。
