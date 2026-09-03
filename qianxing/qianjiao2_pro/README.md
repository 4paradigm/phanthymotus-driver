# 潜蛟 2.0 Pro Driver

本驱动按供应商《如何集成 Mini 系列、P 系列 ROV 运动控制功能？》实现 MAVLink v1 UDP 控制：

- ROV 监听 UDP `14550`；`target_ip`/`target_port` 在 `config.yaml` 配置。
- 每秒发送 `HEARTBEAT`，超过 3 秒没有收到 ROV 心跳则拒绝运动。
- `control.unlock`/`lock` 使用 `COMMAND_LONG`，命令 `400`，`param1=0/1`。
- `control.move` 使用 `RC_CHANNELS_OVERRIDE`。输入轴为 `[-1, 1]`，映射到 PWM `1100..1900`，中位 `1500`：
  `heave=chan1`、`pitch=chan2`、`forward=chan3`、`yaw=chan4`、`lateral=chan5`、`roll=chan7`（chan6 保留）。
- `status` 提供连接、心跳年龄、温度、视频代理状态和最近错误。
- `status` 同时监听 UDP `8500` 的状态广播；`loco_state`、`battery`、`imu` 分别提供姿态/深度/定位、电池和 IMU 数据。

相机卡片：

- `camera` 提供 RTSP 转 JPEG 的实时视频卡；当前固件已确认的 HTTP 能力不足以稳定支持拍照和补光灯控制。

部署时请在 `/opt/phanthy-motus/.env` 或 compose 环境中设置 `MCP_ADVERTISE_HOST`（板卡可被 Agent Core 访问的地址）以及相机账号 `QIANJIAO_CAMERA_USER`、`QIANJIAO_CAMERA_PASSWORD`。密码不写入镜像或 MCP 返回值。

将 `mock: true` 用于无硬件开发测试。接入真实设备前，先通过 QGroundControl 验证 UDP 链路并确认解锁/失联保护行为。
