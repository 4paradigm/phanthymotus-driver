# 潜蛟 2.0 Pro Driver

本驱动按供应商《如何集成 Mini 系列、P 系列 ROV 运动控制功能？》实现 MAVLink v1 UDP 控制：

- ROV 监听 UDP `14550`；`target_ip`/`target_port` 在 `config.yaml` 配置。
- 每秒发送 `HEARTBEAT`，超过 3 秒没有收到 ROV 心跳则拒绝运动。
- `rov_control.arm`/`disarm` 使用 `COMMAND_LONG`，命令 `400`，`param1=1/0`。
- `rov_control.move` 使用 `RC_CHANNELS_OVERRIDE`。输入轴为 `[-1, 1]`，映射到 PWM `1100..1900`，中位 `1500`：
  `heave=chan1`、`pitch=chan2`、`forward=chan3`、`yaw=chan4`、`lateral=chan5`、`roll=chan7`（chan6 保留）。
- `rov_status` 提供连接、心跳年龄、解锁状态和最近错误。

将 `mock: true` 用于无硬件开发测试。接入真实设备前，先通过 QGroundControl 验证 UDP 链路并确认解锁/失联保护行为。
