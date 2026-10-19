# Bumi managed fusion Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans inline.

**Goal:** 审核镜像打包并自动运行共享采集和融合，不再需要手工安装/启动。

**Architecture:** CameraPlugin管理一个RealSense ROS采集进程及旧接口适配器；独立融合管理节点初始化重力、启动visual/EKF、发布健康状态。各子进程有界回收和重启，不重启无关卡片。

**Tech Stack:** ROS2 Humble、Python、APT C++ nodes、pytest。

**Spec:** docs/superpowers/specs/2026-10-19-bumi-managed-fusion.md

## Global Constraints

原PR408，基线13f61cb7；用户授权直接实施/推送；不改ASR/声源方向、不复用loco_servo执行、不控制机器人、不替换运行中的生产容器。保留namespace彩色JPEG/深度zlib640x480@30。ROSdomain继承驱动42；融合10Hz；唯一相机所有者；默认参考相机原点；不猜机身外参；必需依赖缺失构建失败。

## Review Focus

- 相机/适配器/融合故障边界，不得连带停止旧相机输出。
- 彩深stride、endianness、颜色通道和源时间戳必须正确。
- ROS节点名、帧ID、QoS与实际Humble依赖一致。
- 初始化或恢复的数据不能误报tracking，变化segment使旧动作停止。
- SIGTERM及快速重启不得遗留相机进程或重复采集。

### Task 1: 共享相机及进程生命周期

**Files:** camera_runtime.py、device.py、tests/test_bumi_managed_fusion.py。

- [ ] 写单硬件采集者/旧话题编解码/内参/重启隔离/有界关闭失败测试并验红。
- [ ] 实现SharedCameraRuntime与兼容bridge，替换旧采集入口；保留缓存/卡片接口。
- [ ] 运行新测试和相机/拍照录像测试验绿并提交。

### Task 2: 默认自动融合

**Files:** fusion_runtime.py、distance_control.py、config.yaml和相关测试。

- [ ] 写重力静稳窗口、视觉EKF配置/帧/命令、四路健康门、恢复segment、错误状态测试并验红。
- [ ] 实现只读HealthNode/重力初始化、visual/EKF子进程、故障状态与独立重启，接入共享runtime默认启用。
- [ ] 跑融合/距离/Bumi回归验绿并提交。

### Task 3: 镜像、部署、自检和文档

**Files:** Dockerfile、deploy/service.yml、README及构建自检测试。

- [ ] 增加真实缺包/入口检查；依赖和运行文件打包、只读udev挂载、实际版本清单。
- [ ] 执行依赖/ROS模拟验证及相关完整回归，记录已执行/未执行项目。
- [ ] 独立只读复核并修严重问题；推送PR408，核对远端SHA。
