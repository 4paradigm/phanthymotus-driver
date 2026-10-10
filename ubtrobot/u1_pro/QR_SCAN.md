# U1 Pro 单眼二维码卡片

第一版只使用 `camera_left`。卡片不会打开相机、控制机器人、访问网址或执行二维码内容。

## 数据链路

U1 左眼 → 现有相机插件 → `/<namespace>/camera/left`
（`sensor_msgs/CompressedImage`，JPEG）→ `qr_scan`
→ `/<namespace>/qr_scan/result`（`std_msgs/String`，JSON）。

扫码订阅和结果发布复用 U1 的平台侧 ROS context（默认 domain 42）。
设备相机 domain 2 的转发继续由原驱动负责。回调仅缓存最新一帧，后台限频解码。
不支持连接右眼或其他话题；错误连接会明确拒绝。

## 配置与结果

`config.yaml` 的 `plugins.qr_scan`：

- `scan_hz: 2`：每秒最多处理两帧。
- `stale_after_s: 3`：图像超过三秒未更新时，清除读取结果中的码内容。
- `rearm_after_s: 3`：同内容持续可见时只产生一次新事件；消失超过三秒后可重新触发。

结果包括 `status`、`session_id`、`codes`、`new_events`、`frame_sequence`、
`frame_age_s`、`frame_received_at` 和 `processed_at`。
`codes` 含 `text`、原始输入 JPEG 像素坐标 `corners_px`、宽高。
坐标对应相机插件输出的 JPEG，不一定对应厂商原始全分辨率画面。
状态包括 `waiting_for_frame`、`no_qr`、`detected`、`stale`、`decode_error`、`error`、`idle`。
`new_events` 是扫描更新时的事件列表；反复调用 `read` 可能读到同一结果，
消费者应用 `session_id + sequence` 去重，而不是每次 read 都触发操作。

## 最小开发验证

在仓库根目录使用虚拟环境：

```bash
python3 -m pip install -r ubtrobot/u1_pro/requirements.txt pillow pytest
python3 -m pytest tests/test_u1_qr_scan.py -q
```

测试使用真实 OpenCV JPEG 解码和真实 U1 RGB→JPEG 转换代码。
ROS 节点、消息及发布订阅端点使用替身：验证输入输出契约、生命周期和异常处理，
不证明 DDS 通信、相机硬件、ARM64 镜像或画布可用。

在已安装 ROS2、OpenCV 和 NumPy 的 Linux 环境中，还可运行真实 DDS 检查：

```bash
source /opt/ros/humble/setup.bash
python3 tests/u1_qr_dds_check.py
```

该脚本使用两个独立 ROS context、隔离的 domain 142、真实 JPEG 消息和 OpenCV 解码，
验证识别 JSON 和断流后的 `stale` 输出；无需连接机器人。
它不启动厂商 SDK 或实际摄像头，也不等同于完整 U1 driver 镜像构建。

## 按 landing 文档实机验收

1. 提交并推送 `feat/u1-driver` 到自己的 fork。
2. 向 `4paradigm/phanthymotus-driver` 创建 PR，在 PR 评论 `/request_bot_review`。
3. 镜像构建成功后，使用构建回复提供的部署指令在 U1 Pro 上部署。
   不直接猜测镜像标签或容器启动参数。
4. 在画布加入 U1 `camera_left` 和 `qr_scan`，连接左眼输出到扫码输入。
   先确认左眼画面正常，再启动扫码。无需运动或头部控制卡片。
5. 展示纸质二维码和手机屏幕二维码，分别包含 `U1-001`、中文和网址。
   检查识别内容、角点、事件以及响应时间。
6. 持续展示同一个码，确认不重复产生新事件；移开超过三秒再展示，应有新事件。
7. 展示空白画面，应显示 `no_qr`；停止相机，超过三秒应显示 `stale`，码内容为空。
8. 停止再启动扫码，确认 session 更新、没有旧结果、相机仍可使用。

实机结果应记录光照、二维码尺寸、距离、纸质/屏幕、成功次数和识别延迟。
离线验证通过后仍须完成上述硬件验收，才能认为功能可在 U1 上使用。

## 本次开发验证记录（2026-10-10）

- 14 项离线/模拟接口测试通过，包括真实 JPEG 二维码识别、中文/网址、双码、
  U1 RGB 转 JPEG 后输入扫码、JSON 输出、去重、断流、停止重启及异常输入。
- 73 项公共生命周期和 Docker COPY 检查通过。
- 真实 DDS 检查通过：在本机 Docker 的 ARM64 平台镜像（模拟执行）内，
  ROS2 Humble 的两个独立 context 使用 domain 142，JPEG 图像识别为 `U1-DDS-001`，
  成功收到 JSON 输出，断流后成功收到 `stale` 且码内容清空。
- 验证依赖为 OpenCV 4.11.0（wheel 4.11.0.86）和 NumPy 1.26.4；依赖检查无冲突。
- 未完成：完整 U1 driver ARM64 镜像构建、厂商 SDK/相机硬件、画布、实机距离和光照测试。
