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
| `chasing/qianjiao_p200_pro` | 潜行科技潜鲛 P200 Pro ROV | 15739 | MAVLink v1/UDP 6DOF 运动控制、锁定/解锁、心跳和连接状态 |
| [`limx/tron2`](limx/tron2/README.md) | 逐际动力 TRON2 双臂机器人 | 15791 | 头部/左右腕RGB摄像头、关节骨架与夹爪示意、双臂 MoveJ及自动验证复位 |
 | `brainco/revo2` | BrainCo Revo 2 灵巧手 | 15706 | 手指位置/预设手势/LED 控制、状态遥测，触觉版附带指尖触觉遥测 |
| `linkerhand/l20_lite` | 灵心巧手 L20 Lite | 15733 | USB-RS485/Modbus 115200、10 关节控制与状态读取 |
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
6. 需要文件的工具声明 `format: file` + `uploadTo: "mcp"`，字节经 `POST /file/upload`
   进到本容器（`common/file_intake.py`），回包给出的是**容器内部**的绝对路径 ——
   不需要任何共享挂载。详见 [README_dev.md 的 File Arguments 一节](README_dev.md#file-arguments-format-file-and-post-fileupload)。

### G1 导航传感器

只读 `navigation_sensors` 插件使用一个隔离 worker 直接订阅 MID360 原始 DDS，
并发布算法无关的 `/ubuntu/navigation/lidar`（`PointCloud2`）和
`/ubuntu/navigation/imu`（`Imu`）。两路数据保留 MID360 共享源时钟并统一
归一化到 ROS system time；时钟未就绪、重置或样本无效时直接丢弃，不伪造
源时间戳。
各导航流归一化后的时间戳重复或倒序时丢弃，不用上一帧加一伪造新样本。
诊断分别记录 `cloud_non_increasing_timestamps` 和
`imu_non_increasing_timestamps`；后续严格递增的样本可正常恢复。
点云转换只接受小端且满足 `row_step == width * point_step` 的紧凑行布局；
大端、带行 padding 或行长不足的有组织点云会被丢弃，避免错误解码。

`lidar_cloud` 保留旧点云输出作为第一项，并增加 `navigation_lidar` 端口输出标准
PointCloud2；新增 `lidar_imu` 输出 MID360 IMU，原机身 `imu` 不变。
两路标准输出共享 worker，但分别启停：停止一张卡不影响另一张，全部停止才释放
worker。重新启动不会发送上一启停周期的排队帧。安全检查改用内部
`/ubuntu/lidar/cloud_internal`，复用同一份点云转换，不依赖公开点云卡是否启用。

`lidar_imu` 声明 `format=sensor/imu`，原生 ROS 消息为
`sensor_msgs/msg/Imu`：姿态使用四元数，角速度单位为 rad/s，线加速度
单位为 m/s²，并保留三组标准 3×3 协方差。PhanthyMotus PR #141
会按原生类型订阅，再转成版本化的 `phanthy.sensor.imu.v1` 面板载荷，
不修改 ROS topic 本身。

### G1 相机自描述帧

RealSense 插件保留现有 `/ubuntu/camera/rgb` 压缩图、
`/ubuntu/camera/depth` 深度图和距离 topic 的消息语义，同时新增一条
BEST_EFFORT + KEEP_LAST(1) 自描述 RGB 帧，不与导航算法绑定：
`camera_rgb` 的 `rgb_frame` 端口 → `/ubuntu/camera/rgb_frame`
（`phanthy.sensor.camera_rgb_frame.v1`）。

停止相机卡只停止该卡的输出，不影响另一相机卡、距离输出或
拍照/录像。拍照使用按需启用的内部 JPEG 通道。全部输出及拍照需求停止后，共享
进程暂停相机采集。停止门控超时返回错误，不伪报 idle。

旧 Canvas 迁移（消费端同事负责）：

| 原卡片 | 新卡片 / 端口 |
|---|---|
| navigation_lidar | lidar_cloud / navigation_lidar |
| navigation_imu | lidar_imu / 原输出 |
| camera_rgb_frame | camera_rgb / rgb_frame |

移除的名字不保留隐藏别名。原输出维持列表首位和原描述，topic、ROS 类型、载荷、
QoS、schema、标定及时间语义不变；直接订阅 topic 无需修改协议。此轮整合尚未
部署或真机验收，本地测试不能代替 ROS/设备验收。

RGB 自描述帧以 `std_msgs/msg/UInt8MultiArray` 承载 `PSE1` 二进制 envelope：固定
小端头（magic、JSON 元数据长度、二进制载荷长度）之后依次是规范 JSON
元数据和 JPEG 载荷。每帧完整携带当前 RealSense profile 内参、
稳定 `calibration_id`、Depth→RGB 外参、源时间/Driver 接收时间，以及配置的
LiDAR→RGB 外参。`base_to_camera` 字段使用 `target_from_source` 约定和 row-major 4×4
齐次矩阵，语义固定为 `T_base_camera`，将
`camera_color_optical_frame` 中的点变换到 `base_link`。它也参与生成
`calibration_id`，因此消费端无需知道机器人型号。源时间无效、尚在预热、发生时钟重置或倒序时仍发布该帧，
但明确标记为 `unavailable`，不会用发布时刻伪造采集时间。

仓库内置的 LiDAR→Camera 和 Camera→Base 变换来自固定版本的宇树官方 G1 URDF，状态只能是
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
- 配置字段格式 —— 敏感字段（`format: "password"`、`x-sensitive`），以及「扫码装 App」
  「用手机连上这台机器人」的二维码（`format: "qr"`）
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
