# Phanthy Motus 硬件驱动

[English](README.md) | [官网](https://motus.phanthy.com)

**[Phanthy Motus](https://github.com/4paradigm/phanthymotus)** 具身智能平台的硬件驱动集合。

每个驱动是一个独立的 [MCP](https://modelcontextprotocol.io) HTTP 服务器，将硬件能力暴露为工具。驱动启动后自动注册到 [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus)。

## 可用驱动

| 驱动 | 硬件 | 端口 | 说明 |
|------|------|------|------|
| `unitree/g1` | Unitree G1 人形机器人 | 15701 | 运动控制、机械臂、麦克风、扬声器、LED、状态监控 |
| `engineai/t800` | 众擎 T800 开发版 | 15708 | ROS2/Native SDK、全身状态、舞蹈/手势序列、虚拟手柄、运动与高低层控制 |
| `deep_robotics/lynx_m20` | 云深处山猫 M20 | 15716 | 官方 ROS 2/Fast DDS 接口与 basic_server TCP/UDP 原生控制，隔离标准版和 Pro 能力 |
| `brainco/revo2` | BrainCo Revo 2 灵巧手 | 15706 | 手指位置/预设手势/LED 控制、状态遥测，触觉版附带指尖触觉遥测 |
| `phanthy/remote_control` | 远程控制桥接 | 15710 | 远程控制中继 |

## 快速开始

### 环境要求

- Docker（ARM64）
- 一个运行中的 [Phanthy Motus Agent Core](https://github.com/4paradigm/phanthymotus) 实例

### 通过 Web Dashboard 部署（推荐）

最简单的方式是通过 Agent Core 的 Web Dashboard 部署驱动。在顶部菜单进入 **部署** —— 可以浏览已审核发布的驱动版本，选择版本后一键部署，无需手动构建。

### 构建和部署自定义驱动

如果需要从源码构建或开发自定义驱动：

```bash
cp .env.example .env  # 填写镜像仓库凭据

# 构建指定驱动
./build.sh unitree/g1
./build.sh engineai/t800
```

不传参数时，`build.sh` 会显示交互式多选菜单，选择要构建的驱动。也可以直接传路径用于 CI：

```bash
# 构建多个驱动
./build.sh unitree/g1 phanthy/remote_control
```

驱动容器启动后会自动向 Agent Core（`http://<agent-core>:15678/api/mcp`）发送注册请求。注册成功后即可在 Web Dashboard 中看到设备及其工具。

### 本地运行（无需 Docker）

```bash
cd unitree/g1
pip install -r requirements.txt
python main.py
```

## 工作原理

1. 驱动作为 MCP HTTP 服务器在指定端口启动
2. 驱动向 Agent Core 发送注册请求
3. Agent Core 通过 MCP `initialize` 和 `tools/list` 发现驱动的工具
4. 工具对 LLM Agent 可用，并显示在 Web Dashboard 中
5. LLM Agent 通过 MCP `tools/call` 调用工具

### Go2 Nav2 Driver 输入

Go2 Driver 复用同一套 lease 约束的
`phanthy.navigation.velocity_proposal.v1` 合同，固定订阅
`/ubuntu/navigation/nav2/velocity_proposal`。ROS 订阅和执行队列均为
latest-only；合法速度以 m/s 和 rad/s 原样交给 `SportClient.Move`。
终态或 TTL 超时调用 `StopMove`，并用调用后新的零速 `loco/state` 样本确认
停车后才释放任务或保留可恢复 lease。直接调用 loco、步态、动作或特技时，
会先撤销 Nav2 控制权，并在确认停车后才执行对应 RPC。

只读 `navigation_lidar` 和 `navigation_imu` 卡片把 Go2 原生
`rt/utlidar/cloud` 与 `rt/utlidar/imu` DDS 流转换为同样的
`/ubuntu/navigation/lidar` `PointCloud2` 和 `/ubuntu/navigation/imu` `Imu`
合同。两路输出使用同一个非空 REP-103 `sensor_frame`；Driver 将配置的
设备到传感器旋转同时应用于点云 xyz，以及 IMU 的姿态、角速度、线加速度和
全部协方差。MID360 点云与 IMU 三轴平行，Go2 安装轴已对齐 REP-103，因此
默认使用单位旋转。按 REP-145，静止且 Z 轴向上的 IMU 应输出 `+g`，这不是坐标翻转。
原始逐点 `time` 的单位为秒，Driver 将其转为帧内严格递增的 FLOAT64
绝对纳秒时间戳。Go2 的单条 raw DDS 消息只是近场点占比很高的部分包，因此 Driver
默认连续聚合两个包，拒绝间隔超过 120 ms 的跨缺口拼接，并过滤 0.5 m 内本体回波后
再发布一帧导航点云；所有保留点仍携带原始 `ring` 和 `time`。隔离 worker、源时钟归一化、fail-closed 就绪检查和两张卡共享生命周期
与 G1 导航传感器路径一致。同一 worker 还通过 ROS 2 静态变换广播器发布配置的
`base_link -> utlidar_lidar` 安装外参。Go2 默认采用[宇树出厂 `radar_joint` 变换](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go2_description/urdf/go2_description.urdf)
（`xyz=[0.28945, 0, -0.046825]`，`rpy=[0, 2.8782, 0]`）；外参缺失或非法时
worker 启动失败，不使用单位变换兜底。

## 开发新驱动

想要为新硬件添加驱动？请参阅 **[驱动开发指南](README_dev.md)** 获取完整规范，包括：

- MCP 协议实现（JSON-RPC 2.0 方法）
- 工具定义规范（`inputSchema`、`configSchema`、`multiInstance`、`x-action-params`）
- 实例管理（`multiInstance` 标志、configSchema `scope` 字段）
- Plugin 生命周期（`__init__`、`get_tool`、`start`、`stop`、`dispatch`）
- `driver.yaml` 和 `config.yaml` 元数据格式
- 注册与心跳机制
- 端口分配（15700–15799 范围）

简要概述：
- 每个驱动实现 MCP JSON-RPC 2.0 over HTTP（`initialize`、`tools/list`、`tools/call`）
- 工具命名规范：`{设备}_{动作}`（如 `loco_move`、`mic_start`）
- 驱动端口范围：**15700–15799**

参见 [CONTRIBUTING.md](CONTRIBUTING.md) 了解开发环境搭建和 PR 指南。

---

## 音频驱动与 ASR 兼容性要求

任何向感知层 ASR 插件发布音频的驱动，必须满足以下要求。不满足要求的驱动会导致 ASR 收到音频但始终无输出（VAD 静默丢弃不合规的帧）。

### ROS2 消息类型

```
audio_msgs/AudioChunk
  std_msgs/Header header
  string format          # 必须为 "audio/pcm-16k"
  uint8[] data           # 原始 PCM 字节
```

### PCM 格式

| 参数 | 要求 |
|------|------|
| 编码 | 16-bit 有符号整数，小端序（PCM_S16_LE） |
| 采样率 | **16 000 Hz** |
| 声道数 | **单声道（1 channel）** |
| `format` 字段 | `"audio/pcm-16k"` |

### 帧大小

| 参数 | 约束 |
|------|------|
| 最小值 | **1 024 字节**（512 个采样点 ≈ 32 ms） |
| 推荐范围 | 1 024 – 4 096 字节（32 – 128 ms） |

**小于 1 024 字节的帧会被 VAD 静默丢弃**，这是"ASR 有音频输入但没有文字输出"最常见的原因。

### 48 kHz USB 麦克风的常见陷阱

大多数 USB 音频设备的原生采样率为 48 000 Hz。降采样到 16 000 Hz 后，一个 512 帧的 ALSA period 只有 **170 个采样点（340 字节）**——低于最小值。必须将重采样输出积累到缓冲区，凑够 512 个采样点后再发布：

```python
TARGET = 1024  # 字节 — 512 个 int16 采样点 @ 16 kHz
_buf = bytearray()

# 在采集循环内，重采样到 16 kHz 之后：
_buf += resampled_bytes
while len(_buf) >= TARGET:
    chunk, _buf = bytes(_buf[:TARGET]), _buf[TARGET:]
    msg = AudioChunk()
    msg.format = "audio/pcm-16k"
    msg.data = list(chunk)
    publisher.publish(msg)
```

`unitree/g1/ext_devices.py` 的 `ext_mic` 插件已应用此模式。

完整的 VAD 调参选项参见主仓库 [perception/README.md](https://github.com/4paradigm/phanthymotus/blob/main/perception/README.md)。

## 许可证

[Apache License 2.0](LICENSE)
