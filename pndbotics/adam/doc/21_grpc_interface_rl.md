# 强化学习 gRPC 运动控制

本驱动支持 Adam 的 `pnd.robot.RobotControl` 强化学习接口。服务端固定监听
`50051`，默认配置见 `config.yaml`；与旧版传统控制接口（`6666`）不兼容。协议代码使用
厂家接口生成的 `proto/robot_control_pb2*.py`。

## 配置

```yaml
grpc_host: "10.10.20.127"
grpc_port: 50051
plugins:
  loco:
    enabled: true
    grpc_api: "rl"
```

## 真机执行前置条件

根据厂商的真机开发和遥控器说明，凡是通过 DDS/ROS2 向机器人发送执行器
指令（包括 `hand`、`arm` 以及未来的 `rt/lowcmd` 关节控制），应先人工让机器人
进入开发者模式：机器人悬挂并处于阻尼模式时，短按遥控器 `LO + RO`，确认 RCU
指示灯变为蓝色慢速呼吸。退出时短按 `LT + B`，回到阻尼模式。

这一步不是 MCP 或 gRPC API 能完成的操作；驱动不会模拟按键，也不会在启动卡片时
自动切换开发者模式。执行前仍需满足具体控制链路的条件：

- RL gRPC：动作卡会确认 `GetRobotState` 成功，按 `switchable_states` 自动切换到所需
  FSM 状态，并固定选择 `SetControlMode(domain_id=1)`。
- ROS2 上肢控制：机器人还必须处于站立状态，并通过遥控器开启实时遥操接收，直到
  控制台显示 `real time retarget start`；退出上肢外部控制后再停止卡片。
- DDS 手指/底层控制：开发者模式只代表允许外部 SDK，仍必须确认对应 `rt/*` 通道已
  发现且发送周期正常。

只读状态卡（如 `joints`、`imu`、`battery`、`estop`）不需要开发者模式，但仍依赖
机器人 Demo、DDS/ROS2 或 PAC 服务已经启动。实体急停始终使用遥控器 `LB + RB`，
不能由软件动作卡冒充。

也可以通过 `GRPC_HOST`、`GRPC_PORT` 和 `GRPC_API=rl` 覆盖配置。兼容的 `loco`
聚合卡提供 `GetRobotState`、`SetMode`、`SetVelocity`、`SetHeight`、`SetMotion`、
`SetTrackingMotion`、`SetControlMode`、`GetControlState` 和 `Shutdown`；驱动还提供
职责拆分的执行卡：`motion`（上半身动作文件）和 `tracking_motion`（全身轨迹文件）。
动作卡会在内部自动选择 RL 控制域，并切换到所需 FSM 状态；`posture`、`control_mode`
和 `safety` 不作为外部卡片暴露。

下发动作前先调用 `get_state`，只使用返回的 `switchable_states` 和
`available_actions`。动作卡会先校验目标状态，再轮询 `fsm_state` 确认异步切换完成；
`motion.play` 和 `tracking_motion.play` 会分别校验
`SetMotion`/`SetTrackingMotion` 出现在 `available_actions` 中。动作和轨迹文件必须是
机器人侧的 `.txt` 路径。速度和站高按接口约定限制在 `[-1, 1]`。当前机器人
服务端把 `SetVelocity` 与 `SetHeight` 标为预留接口，驱动会如实返回不支持状态，不会伪造
成功响应。

`SetControlMode` 的 `domain_id=0` 为传统控制，`1` 为 RL 控制。动作卡固定选择
`domain_id=1`，不要求上层显式管理控制域。普通动作停止使用 `motion.stop`，不宣称
能够触发实体急停；实体急停仍由 `estop` 只读卡和遥控器 `LB + RB` 提供。控制器关闭
仍只保留在兼容 `loco.shutdown` 动作中，不作为独立卡暴露。
