# PR #525 腕部相机卡片通用化改造方案

我检查了 [phanthymotus-driver PR #525](https://github.com/4paradigm/phanthymotus-driver/pull/525?utm_source=chatgpt.com) 的最新提交 `75ee8f2e`，并对照了 Tianyi 的 `ext_mic`、`ext_camera` 和 Agent Core 的多实例机制。

建议将现有四张固定卡片统一成一张 `wrist_camera`，支持多实例、左右手配置、自动发现相机，以及 RGB/Depth 两个独立输出端口。

## 一、当前实现与目标

PR #525 目前针对 Tianyi 的两台 RealSense D405 开发，采用四张独立卡片：

- `camera_wrist_left_rgb`
- `camera_wrist_left_depth`
- `camera_wrist_right_rgb`
- `camera_wrist_right_depth`

代码存在以下限制：

| 项目     | PR #525 当前实现 | 建议修改                        |
| ------ | ------------ | --------------------------- |
| 卡片数量   | 固定四张         | 一种 `wrist_camera` 卡片        |
| 画布实例   | 每个名称对应一张固定卡片 | `multiInstance: true`，可多次添加 |
| 左右手    | 写死在卡片名称中     | 每张实例单独选择                    |
| 相机绑定   | 配置文件中写死左右序列号 | 自动发现 + 实例选择                 |
| 相机型号   | 强制检查 D405    | 根据设备能力选择采集后端                |
| 输出端口   | 每张卡片一个       | 同一张卡片同时输出 RGB、Depth         |
| 相机生命周期 | 每侧一套采集进程     | 按物理设备共享采集进程                 |

这里最重要的区别是：不限制相机型号，不代表任何 USB 摄像头都具有深度能力。

普通 UVC 摄像头一般只能输出 RGB；RGB-D 相机才能同时提供真实的 RGB 和深度数据。新设计应自动识别能力，而不是对没有深度功能的相机生成假的深度图。

## 二、卡片接口设计

建议沿用 `ext_mic` 的实例化机制，每添加一次卡片，Agent Core 自动生成独立 `instance_id`，用户通过卡片上的配置按钮选择设备和左、右手腕归属。

wrist_camera

Sensor · 多实例

实例 A · 左手腕 · USB Camera 1

RGB

image/jpeg

Depth

image/depth-zlib

卡片结构示意：两个独立输出可分别连接 VLA、图像处理或深度处理卡片。

建议配置项如下。

| 配置参数              | 可选值                    | 说明           |
| ----------------- | ---------------------- | ------------ |
| `side`            | `left` / `right`       | 对应机器人左、右手    |
| `device_id`       | `auto` / 设备列表          | 选择物理相机       |
| `fps`             | 15 / 30 / auto         | 自动协商支持帧率     |
| `resolution`      | auto / 设备支持值           | 自动匹配分辨率      |

这些配置应当都是 `scope: "instance"`，例如卡片 A 选择左手，卡片 B 选择右手，互不影响。

建议的工具声明核心结构：

```
{    "name": "wrist_camera",    "type": "sensor",    "multiInstance": True,    "topic_in": [],    "topic_out": [        {"format": "image/jpeg", "desc": "RGB"},        {"format": "image/depth-zlib", "desc": "Depth"}    ],    # configSchema: 每个实例独立选择 side、device_id、fps、resolution 等}
```

输出话题按实例 ID 生成，不能继续用固定的 `/left/rgb` 形式，否则多实例可能发生话题冲突。例如：

```
/nvidia_desktop/wrist_camera/card_a/rgb
/nvidia_desktop/wrist_camera/card_a/depth

/nvidia_desktop/wrist_camera/card_b/rgb
/nvidia_desktop/wrist_camera/card_b/depth
```

我检查了 Agent Core 的 [canvas.js](https://github.com/4paradigm/phanthymotus/blob/main/agent-core/web/js/canvas.js?utm_source=chatgpt.com) 和 [config.py](https://github.com/4paradigm/phanthymotus/blob/main/agent-core/src/api/config.py?utm_source=chatgpt.com)，目前已经按 `topic_out` 数组生成多个输出端口，并按端口下标解析连线。

因此，双输出端口不需要重新实现画布的连线系统。只需保证 `info()` 始终按 RGB、Depth 的固定顺序返回对应话题。

## 三、自动识别不同型号的 USB 相机

目前 PR #525 的采集进程存在两处硬限制：

```
if "D405" not in device_name:    raise RuntimeError(...)config.enable_stream(    rs.stream.color, 640, 480,    rs.format.rgb8, 15)
```

第一处强制检查型号，第二处固定采集格式。

建议改为按照相机实际能力选择采集方式。

USB 相机接入

枚举 / 设备标识 / 能力查询

RealSense SDK

支持 SDK 的 RGB-D 相机

V4L2 / UVC

标准 USB 摄像头

设备能力检测

RGB / Depth / 格式 / 分辨率 / FPS

RGB 编码

Depth 转换

### 推荐的设备发现策略

| 后端        | 识别方式                | 能力                     |
| --------- | ------------------- | ---------------------- |
| RealSense | `pyrealsense2` 枚举   | 根据型号和实际流配置提供 RGB/Depth |
| 标准 UVC    | `/dev/video*`、V4L2  | 通常提供 RGB               |
| 其他 RGB-D  | 厂商 SDK 或已有 ROS 2 话题 | 按实际设备能力接入              |

Tianyi 现有 `ext_camera` 已经实现 V4L2 枚举和格式探测，可以复用相关逻辑。但它明确排除了 RealSense，不能直接复制过来作为完整的腕部相机发现器。

建议新增独立的 `CameraDiscovery`，统一返回：

```
{    "id": "stable-device-id",    "name": "Detected USB Camera",    "backend": "realsense",    "capabilities": {        "rgb": True,        "depth": True    },    "resolutions": ["640x480"],    "fps_options": [15, 30]}
```

这里的设备 ID 应尽量使用相机序列号、`/dev/v4l/by-id` 或稳定的物理 USB 路径，而不是 `/dev/video0` 这种重启后可能变化的编号。

### 即插即用需要处理的特殊情况

- 只有一台相机时，可以尝试自动绑定。
- 有多台相机时，自动列出设备，但不能仅靠 USB 枚举顺序推断物理左右。
- 用户通过画面确认左右手后，保存设备绑定；如果装反，可以直接在配置里重新交换。
- 插拔相机后重新扫描，设备恢复时自动尝试重连。
- 如果相机缺少 Depth 能力，Depth 输出报告 `unavailable`，不生成虚假的深度数据。

为了让设备列表在热插拔后实时更新，可能需要对 Agent Core 的配置下拉列表增加刷新支持。现有 `ext_mic` 的动态选项主要在获取工具定义时生成，不能仅依赖初次扫描。

此外，USB 相机必须对运行驱动的容器可见。如果相机接在 Tianyi 的另一块计算板上，仍需在那块板上采集并通过网络桥接传输；本地 USB 枚举无法直接发现远程板子的设备。

## 四、左右手腕归属与设备重新绑定

每张卡片分别保存物理设备 `device_id` 和左、右手腕归属 `side`，两者不依赖 USB 枚举顺序。

用户通过设备标识或预览确认相机实际位于哪侧，再选择 `left` 或 `right`。如果左右相机装反，可以重新选择对应设备或修改归属，无需修改代码或重新构建镜像。

输出话题仍按 `instance_id` 生成，因此交换设备绑定或左右归属不改变话题地址，已有下游连线可以保留。

本次范围仅包含设备绑定和左右归属配置，保持相机原始画面方向以及当前 RGB/Depth 未对齐的约定。

当前 `image/depth-zlib` 预览按 640×480 的 uint16 毫米深度图解码。自动协商采集参数时仍需遵守这一尺寸约定；不兼容的配置应明确提示，避免发布无法正确解析的数据。

## 五、保留 PR #525 中已经实现的采集安全机制

不建议把现有采集模块全部推倒重写。

PR #525 已实现的独立进程采集、发布与停止同步、采集失败超时、RGB/Depth 独立健康检查，应该保留。

新架构建议改为：

`WristCameraPlugin → CameraManager → CameraSession → RealSense/UVC Backend`

其中 `CameraManager` 以物理设备 ID 管理采集资源。

例如画布存在两个相机实例时，每张卡片有自己的参数和两个话题，但每个物理相机只需要开启一套采集管线。如果多个卡片绑定同一相机，也应共享采集、独立发布，停止其中一张不能中断其他实例。

还要注意一个原有行为：PR #525 的四路视频在驱动启动时自动采集，因此不启动智能控制也能预览。

改成 `multiInstance` 后，驱动启动时还不知道未来会有哪些卡片实例。建议改为 保存实例配置后自动启动该实例的预览，并且在移除卡片时停止对应实例。这部分可能需要 Agent Core 的小范围生命周期适配，不能只修改驱动然后假设原有预览行为仍然成立。

## 六、具体文件修改范围

| 文件                                      | 修改内容                             |
| --------------------------------------- | -------------------------------- |
| `x-humanoid/tianyi2.0/wrist_cameras.py` | 重构为多实例、双输出、可插拔采集后端               |
| `x-humanoid/tianyi2.0/main.py`          | 注册单个 `WristCameraPlugin`，改为按实例启动 |
| `x-humanoid/tianyi2.0/config.yaml`      | 删除固定左右序列号的强依赖，保留全局默认参数           |
| `x-humanoid/tianyi2.0/driver.yaml`      | 四张卡片合并为一个 `wrist_camera`         |
| `x-humanoid/tianyi2.0/README.md`        | 更新设备发现、实例配置、左右重绑说明               |
| `tests/test_tianyi_wrist_*.py`          | 修改并新增多实例、热插拔、左右重绑及设备能力测试           |

参考代码：[PR #525 的 wrist_cameras.py](https://github.com/Nidhogg2002/phanthymotus-driver/blob/75ee8f2e15bed66d7da38b8b5f511be82d0ff746/x-humanoid/tianyi2.0/wrist_cameras.py?utm_source=chatgpt.com)

Agent Core 优先只调整必要的设备列表刷新、不可用端口状态和自动预览逻辑。现有多实例及多输出连线机制直接复用。

## 七、建议实施顺序

1. 统一卡片和多实例机制

   将四张固定卡片改为 `wrist_camera`，完成实例独立配置、两个输出口和实例话题管理。
2. 实现自动发现和设备绑定

   先支持现有 RealSense 系列，再加入通用 V4L2/UVC 后端。实现设备重新扫描、稳定 ID 和左右手重新绑定。
3. 完善生命周期和画布预览

   验证添加卡片、保存配置、开始/停止项目、移除卡片、断开 USB、重新插入后的状态恢复。
4. 硬件验证与 PR 更新

   在 Tianyi 两台现有腕部相机上验证 RGB/Depth 同时输出，再使用至少一台普通 UVC 相机确认非 RealSense 的自动发现路径。

## 八、需要重点验收的结果

最终至少满足以下行为：

验收清单

0/9

画布可以添加两张以上 wrist_camera 卡片

每张实例可独立设置 left/right 和物理设备

左右相机交换配置后，不需要修改代码或重新构建镜像

同一张卡片可以独立连接 RGB 和 Depth 输出

普通 UVC 相机能够输出 RGB，不伪造 Depth

RealSense 相机能够同时输出 RGB 和有效毫米深度

停止一张实例不会中断其他实例

拔插相机后设备列表及运行状态能正确恢复

保存配置后不启动智能控制也能预览视频

结论： 方案可行，并且 `ext_mic` 已经提供了多实例管理的参考实现。建议将 PR #525 的现有 RealSense 采集逻辑保留为一个后端，而不是将 D405 改名后继续硬编码；在其上增加设备发现、能力协商和独立实例层。

对于以后接入 VLA 训练的用途，还应保留真实相机内参、设备时间戳和左右手安装外参。当前 PR #525 的 RGB/Depth 尚未对齐，且使用主机时间戳，不能直接假定其满足高精度 RGB-D 融合或 VLA 数据采集要求。

以上为审查和改造方案，尚未修改或推送 PR #525 的代码。