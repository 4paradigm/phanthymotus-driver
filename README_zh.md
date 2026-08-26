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

开发版 G1 使用仓库内的 Git 部署入口。请先把当前分支推送到远程，并确保
本地工作树干净：

```bash
./unitree/g1/deploy/deploy-from-git.sh g1-bj-wifi
```

脚本不会把本地工作树复制到机器人。它会记录本地分支和提交，在目标机器的
`~/hanzebei/phanthymotus-driver` 中拉取该分支，要求远程分支 tip 与固定提交
完全一致，再使用仓库自带的 G1 Dockerfile 构建镜像。当 GitHub Git 端点不可用时，
GitHub 仓库会自动回退到官方 `codeload.github.com` 上的同一固定提交归档，
不使用第三方源码镜像。只有明确需要覆盖部署输入时才设置 `REPO_URL`、
`SOURCE_ARCHIVE_URL`、`SOURCE_REF`、`EXPECTED_COMMIT`、`REMOTE_REPO` 或
`IMAGE`。`DRY_RUN=1` 只校验并打印最终来源，不连接机器人。

G1 Dockerfile 默认同时使用阿里云 Ubuntu Ports 和 PyPI 国内镜像，并在安装
构建依赖前替换基础镜像遗留且已不可解析的腾讯云 Ubuntu 源。直接执行
Docker 构建时可通过 `UBUNTU_PORTS_MIRROR` 和 `PYPI_MIRROR` build arg 覆盖
默认值；运行时基础镜像来源不变。

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

### G1 受控导航速度执行

G1 `loco` actuator 接收由导航 lease 约束的
`phanthy.navigation.velocity_proposal.v1` 输入。订阅端使用可靠的
`KEEP_LAST(depth=1)`，执行端使用容量为 1 的 latest-only 队列：未读取
或已等待的旧速度都会被新速度替换，不会积压。proposal TTL 失效会立即
触发 `StopMove`；只有在返回后用新的 odometry 样本确认零速，同一导航
lease 才能保留并接受下一条新鲜 proposal。安全、身份、序列、RPC 和停车确认类
硬故障仍会解除武装。

`loco.start` 只订阅唯一 proposal topic，并保持物理停止。Driver 空闲时会先
完整校验首条新鲜、合法、非零 proposal，再原子绑定其 `nav_id` 并执行该帧。
当前任务期间，其他 ID 既不能替换 lease，也不能中断当前任务。终态零速
proposal 会先进入 `terminal_pending_stop`；只有零速停车确认成功后才退役
当前 ID，并保留订阅以等待下一任务。首次停车确认失败时保持 fail-closed，
后续重试确认成功会恢复 `awaiting_first_valid_proposal`。格式错误、过期、
零速首包、终态首包、已退役 ID 和任务中异 ID proposal 都不能建立或替换
lease，因此连续导航不需要 Agent Core 执行逐任务授权 action。

`nav_id` 只用于任务隔离和防重放，不是发布者身份认证。首包绑定模式以机器人
ROS 2/DDS 网络受信且已隔离为前提；能向 proposal topic 发布消息的非受信
参与者可能在空闲时抢占 lease，因此不得把该 topic 暴露给非受信发布者。

`loco info` 会返回 proposal 计数、合并数、实测 RPC/队列时延、滚动
RPC p50/p95/p99/max、逐原因拒绝统计及最近一次已确认停车。
`last_set_velocity_duration_ms` 表示实测 RPC 耗时，不是 proposal TTL 余量。

velocity proposal 合同与 `loco.move` 输入边界保持一致：前后和横向速度
均限制为 `[-1.0, 1.0] m/s`，偏航角速度限制为
`[-2.0, 2.0] rad/s`。Driver 仍会在运动 RPC 之前拒绝非有限数或超界值。

### G1 导航传感器

只读 `navigation_sensors` 插件使用一个隔离 worker 直接订阅 MID360 原始 DDS，
并发布算法无关的 `/ubuntu/navigation/lidar`（`PointCloud2`）和
`/ubuntu/navigation/imu`（`Imu`）。两路数据保留 MID360 共享源时钟并统一
归一化到 ROS system time；时钟未就绪、重置或样本无效时直接丢弃，不伪造
源时间戳。

`navigation_lidar` 与 `navigation_imu` 两张卡共享同一个 worker。停止任意一张
卡都会停止两路数据并释放 MID360 worker；再次启动任意一张卡会创建新的共享
worker。

### G1 相机自描述帧

RealSense 插件保留现有 `/ubuntu/camera/rgb` 压缩图、
`/ubuntu/camera/depth` 深度图和距离 topic 的消息语义，同时新增两个
BEST_EFFORT + KEEP_LAST(1) 自描述帧数据流。它们是通用传感器/数采输出，
不与导航算法绑定：

- `camera_rgb_frame` → `/ubuntu/camera/rgb_frame`：
  `phanthy.sensor.camera_rgb_frame.v1`；
- `camera_depth_frame` → `/ubuntu/camera/depth_frame`：
  `phanthy.sensor.camera_depth_frame.v1`。

两者以 `std_msgs/msg/UInt8MultiArray` 承载 `PSE1` 二进制 envelope：固定
小端头（magic、JSON 元数据长度、二进制载荷长度）之后依次是规范 JSON
元数据和 JPEG 或 zlib level 1 无损压缩的 Z16 小端载荷。Depth 元数据通过
`image.compression` 描述压缩方式，并同时记录压缩前后的字节数；消费者先用
zlib 解压即可恢复原始 Z16。每帧完整携带当前 RealSense profile 内参、
稳定 `calibration_id`、Depth→RGB 外参、源时间/Driver 接收时间，以及配置的
LiDAR→RGB 外参。源时间无效、尚在预热、发生时钟重置或倒序时仍发布该帧，
但明确标记为 `unavailable`，不会用发布时刻伪造采集时间。

仓库内置的 LiDAR→Camera 变换来自固定版本的宇树官方 G1 URDF，状态只能是
`factory_nominal`：它不是北京 G1 的实测外参。在该机器人上完成点云投影叠加
与像素残差验收并保存证据前，不得改成 `validated_on_device`。配置缺失或非法
时输出 `unavailable`，不会用单位矩阵冒充有效标定。相机重连后会重新读取
活动 profile；序列号、分辨率、内参、depth scale 或外参变化都会生成新的
`calibration_id`。

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
