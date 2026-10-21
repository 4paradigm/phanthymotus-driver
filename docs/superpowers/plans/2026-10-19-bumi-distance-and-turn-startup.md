# Bumi distance and turn startup Implementation Plan

> **历史方案（2026-10-21）：** 本文记录早期 RGB-D/IMU 融合设计，不是当前生产镜像的安装或部署指南。生产驱动使用直接 `pyrealsense2` 采集与实验性机身 IMU 测距，不打包或启动 RTAB-Map/EKF。当前行为以 [`noetix/bumi/README.md`](../../../noetix/bumi/README.md) 为准；独立视觉融合实验见 [`experiments/bumi_odometry/README.md`](../../../experiments/bumi_odometry/README.md)。

> **For agentic workers:** Use superpowers:executing-plans inline. Steps use checkbox syntax.

**Goal:** 接入前进距离反馈控制并修复末段点动不能启动步态。

**Architecture:** 一个独立 distance_control.py 接收/校验融合反馈并闭环；device.py 复用目标动作 reservation/ACP worker，直接发布 SDK 命令。旋转仅调整脉冲启动阶段。

**Tech Stack:** Python、ROS 2 nav_msgs/std_msgs、pytest；无新依赖。

**Spec:** docs/superpowers/specs/2026-10-19-bumi-distance-and-turn-startup.md

## Global Constraints

基线93164457；原分支/PR408；不改声源方向/ASR；不复用loco_servo执行；不部署。前进0.1–3m；完成误差±0.05m；旋转±2°；源/接收时间超过0.5s失效；旋转启动等待上限0.5s。用户明确取消先真机验收的接入门槛，并要求直接实施，跳过重复的设计/计划确认。

## Review Focus

- 取消与非零发送的竞态：停下返回后不能重启。
- 旧tracking状态与新pose混用：源/接收双重时效和segment变化报错。
- 工作模式/共享动作竞争：不得同时驱动底盘。
- 估计位置跳变或过冲：不得误报完成。
- 旋转启动后的延迟角度反馈：不可无限拉长非零脉冲。

### Task 1: 旋转启动

**Files:** device.py；noetix/bumi/tests/test_loco_destination.py。

- [ ] 增加0.2s/0.35s启动延迟、正负小角度和取消测试，确认旧实现失败。
- [ ] 有界等待真实同向进展；启动后计算有效脉冲时间；带内立即刹车。无启动时错误退出。
- [ ] 全部旋转测试通过，提交。

### Task 2: 距离反馈和卡片集成

**Files:** noetix/bumi/distance_control.py；device.py；tests/test_bumi_distance_control.py；noetix/bumi/tests/test_loco_destination.py。

**Interfaces:** DistanceFeedback(executor) 的 start/stop/snapshot 返回新鲜且有效的 body pose；run_distance(owner,distance_m) 经 owner 的取消事件、模式检查和原子发布发送速度；owner._do_destination(...action...) 共用 reservation/ACP。

- [ ] 编写 schema、位置有效性与新鲜度、直线投影、静稳终点、无进展、过冲、失跟、停止/互斥/ACP 回归；确认失败。
- [ ] 实现接收节点、距离闭环与卡片/生命周期集成；直接 SDK 输出。
- [ ] 同步 README/HARDWARE_TEST 的运行方法、同 domain 要求、菜单刷新和实际精度限制。
- [ ] 相关 Bumi/control 测试通过；提交。

### Task 3: 独立复核并推送

- [ ] 运行合并测试、编译与diff检查；记录结果。
- [ ] 独立只读复核本轮提交，处理严重问题并重跑覆盖测试。
- [ ] 推送原PR408分支，核对远端head。保留真机精度未验证说明。
