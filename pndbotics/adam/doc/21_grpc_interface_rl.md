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

也可以通过 `GRPC_HOST`、`GRPC_PORT` 和 `GRPC_API=rl` 覆盖配置。驱动的 `loco`
卡提供 `GetRobotState`、`SetMode`、`SetVelocity`、`SetHeight`、`SetMotion`、
`SetTrackingMotion`、`SetControlMode`、`GetControlState` 和 `Shutdown`。

下发动作前先调用 `get_state`，只使用返回的 `switchable_states` 和
`available_actions`。`set_mode` 是异步切换，需轮询 `fsm_state` 确认完成；动作和轨迹
文件必须是机器人侧的 `.txt` 路径。速度和站高按接口约定限制在 `[-1, 1]`。当前机器人
服务端把 `SetVelocity` 与 `SetHeight` 标为预留接口，驱动会如实返回不支持状态，不会伪造
成功响应。

`set_control_mode` 的 `domain_id=0` 为传统控制，`1` 为 RL 控制。驱动停止时不会自动
关闭机器人控制器，必须显式调用 `shutdown`。
