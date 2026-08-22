# 云深处山猫 M20 Driver

本 Driver 依据供应商《山猫 M20 开发指南》（适用系统版本 V1.1.8）实现。

## 固定版本构建依赖

镜像构建不会在容器内访问 GitHub，也不会执行 `git clone`。仓库已包含固定版本的官方 `deep-robotics-msg` ZIP，无需手动下载：

- 提交版本：`a0d1a29eec5c4db5a9107595bb51e3be8122b86c`
- 文件名：`deep-robotics-msg-a0d1a29eec5c4db5a9107595bb51e3be8122b86c.zip`
- 放置位置：`deep_robotics/lynx_m20/deep-robotics-msg-a0d1a29eec5c4db5a9107595bb51e3be8122b86c.zip`
- 上游来源：`https://github.com/DeepRoboticsLab/deep-robotics-msg/archive/a0d1a29eec5c4db5a9107595bb51e3be8122b86c.zip`
- SHA256：`1d268a76e80af8ea5aa3dc28de0c236de87bce55a5d397fdad1e23515f02a537`

示例：

```bash
shasum -a 256 deep_robotics/lynx_m20/deep-robotics-msg-a0d1a29eec5c4db5a9107595bb51e3be8122b86c.zip
bash build.sh --mirror tuna deep_robotics/lynx_m20
```

Dockerfile 会在解压和编译前再次校验 SHA256。归档内保留上游 `LICENSE`，更新依赖时必须同时更新提交版本、文件名和 SHA256。

## 已实现接口

- `basic_server` 原生协议：TCP `10.21.31.103:30001` 可靠指令与 1 Hz 心跳，UDP `10.21.31.103:30000` 高频运动指令；包含官方 16 字节帧头、JSON ASDU、响应关联与状态上报缓存。
- ROS 2 / Fast DDS：`/MOTION_STATE`、`/GAIT`、`/NAV_CMD`、`/MOTION_INFO`、`/IMU`、前后雷达、硬急停、选配充电和 GNSS。
- 运动：起立、趴下、软急停、4 种官方步态、归一化轴控制、导航速度与停止。`axis`/`velocity` 支持 `duration`：留空默认 1 秒，大于 0 时持续刷新并到期自动归零，明确设为 0 时持续到独立 `stop`；底层 0.5 秒失联看门狗保持生效。ROS 2 起立会按文档等待反馈后执行 `state=1 → state=17`。
- 运动事件：新增只读 `motion_events` Sensor，发布动作请求/接受、真实运动状态与步态变化、运动开始/更新/停止、定时结束和命令失败；请求接受与真实反馈确认分开记录。
- 选配自主充电：开始、退出和异常强制复位。
- 设备与状态：前后灯、常规/导航/辅助模式、休眠与自动休眠、16 关节反馈、双电池、温度和错误列表。
- 双路相机：返回官方 H.265 RTSP 地址 `video1`/`video2`；文档明确相机不发布 ROS 2/DDS 话题。
- M20 Pro：默认开放里程计、定位初始化、单点导航、取消、状态查询，以及 `mapping_view` 建图视图。建图期间将 `/grid_map_3d` 的 `base_link` 点云通过 `/SLAM_ODOM` 转换并累积到 `map` 坐标，停止后切换到最终 `/GRID_MAP`。

## 型号边界

默认 `model_variant: pro`，用于当前 M20 Pro 的 Web Console 无外部配置部署。供应商文档明确建图、定位和内置导航仅 M20 Pro 支持；部署到标准版时必须将该值改为 `standard`，标准版不会注册这些工具。

`mapping_view` 是不依赖 SSH 的只读 Sensor，仅在 M20 Pro 上注册。启动卡片后，它将 `/grid_map_3d` 的 `base_link` 点云通过 `/SLAM_ODOM` 转换并累积到 `map` 坐标；停止卡片后，如果收到过 `/GRID_MAP`，则切换到最终占据栅格视图。实时点云按 `voxel_size` 去重、受 `max_buffer_points` 约束，并按 `publish_hz` 和 `max_points` 限制 Canvas 负载。数据包元数据和 `mapping_view.info` 会公布状态、数据源、坐标系、更新时间、更新次数及点数。最终二维地图只发布占据值大于等于 `occupied_threshold` 的栅格中心点。本 Driver 不负责启动、停止或保存 NOS 建图任务。

供应商文档未提供舞蹈、自定义特技或关节位置控制接口，本 Driver 不虚构这些能力。

## 验证状态

已在 M20 Pro 真机确认 `/grid_map_3d` 与 `/SLAM_ODOM` 均约 10 Hz，并确认 `/grid_map_3d` 为 `base_link` 坐标、XYZ float32、16 字节点步长，`/SLAM_ODOM` 为 `map` 坐标。通过 103 手动 SSH 启动 106 的 `mapping.service` 后，已确认 Agent Core/Canvas 中 `mapping_view` 可显示实时建图；最终 `/GRID_MAP` 切换仍需单独验真。实时坐标转换、点云累积、最终栅格编码和 Canvas 生命周期契约已通过开发机测试。速度方向、选配件存在性、充电及 Pro 导航仍未完成真机验证。首次联调前请确认系统版本为 V1.1.8、外接主机接入 `10.21.31.x` 或 `10.21.33.x` 网段，并确保没有与 `planner` 或 `charge_manager` 并发发布 `/NAV_CMD`。
