# PICO 设备实现（teleop_device）

本模块由独立 [PICO Driver](../README.md) 加载，负责 OpenXR 输入、WSS/WebRTC、下载、配对与最新输入缓存。不加载机器人模型、不求解 IK、不申请运动控制权、不调用 ActuCore。`ext_vr` 是内部目录名，公开卡片名为 `teleop_device`。

## 接口与传输

MCP 服务位于 15742，提供 `info/config/start/stop`；HTTPS/WSS 位于 15741。一个服务支持一个头显实例。设备唯一输出为 `/teleop/command`，format `data/teleop-cmd`，schema `motus.teleop.command/1`，只发送输入，不订阅机器人反馈。字段与生命周期见[输入契约](../README.md#输入契约)。

DDS 使用同机 domain 42、RELIABLE / KEEP_LAST(16) / VOLATILE。独立 writer 隔离阻塞发布，应用层仅保留最新待发姿态。OpenXR 位置转换为 `[-z,-x,y]`，四元数转换为 `[-qz,-qx,qy,qw]` 并归一化，得到 X 前、Y 左、Z 上的设备跟踪系，单位为米；这不是机器人基座坐标系。

输入携带设备身份、连接与跟踪空间代次、序号、接收主机时钟身份和采样时间。断流、心跳和重发不刷新旧位姿时效。松握仍采集；再次握住是新的输入帧，设备不负责重标定。

## 安装与连接

齿轮页显示管理 PIN、安装说明和下载网址，网址由 Driver 生成，不作为用户配置。普通 Core 仅显示文本，用户复制网址到 PICO 浏览器。下载 App 后返回网页，按连接入口打开已填机器人地址的 App；已配对 App 会自动重连。

首次在齿轮设置四位管理 PIN，网页输入 PIN 后才能生成邀请、打开配对窗口及撤销设备。下载 APK 和已配对 App 重连无需 PIN；留空保存保留已有 PIN。管理会话与限次规则见 [配置与生命周期](../README.md#配置与生命周期)。一次性邀请有有效期；手动发现配对需要打开 120 秒窗口并核对指纹。下载页与管理接口由同一服务托管。配对只建立输入连接，不授权机器人动作。发现使用 `_motus-teleop._tcp` 的 mDNS/DNS-SD 广播；跨网段或组播隔离环境用网址连接。

PICO 只显示配对、连接、透视与握把提示；机器人执行状态由控制卡在 Canvas 监控中显示。生命周期修改串行处理，输入采集不等待配置锁。

## 依赖与来源

`aiohttp` 提供 HTTPS/WSS，`aiortc` 及其锁定依赖提供 WebRTC 数据通道，`zeroconf` 提供局域网发现，`cryptography` 用于证书。`PyYAML` 用于 Driver 配置。运行依赖由现有镜像构建安装，APK 从带 SHA256 的制品清单下载，不将 APK 或 Android 构建工具纳入 Git。

来源与许可证见 [NOTICE](NOTICE.md)、[ADOPTED_SOURCE](ADOPTED_SOURCE.json)；客户端构建见[原生 App](openxr_capture_native/README.md)。测试覆盖本地 MCP、HTTPS、WSS/RTC、生命周期及配对页面；这些测试不等于实体头显、真实 DDS 或机器人验收。
