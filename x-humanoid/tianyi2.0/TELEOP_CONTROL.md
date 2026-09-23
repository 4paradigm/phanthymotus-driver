# 天轶双 Driver 遥操

本轮使用两张卡：PICO Driver 的 `teleop_device` 和本 Driver 的 `teleop_control`。
只在 Canvas 连接前者的 Teleop Cmd 到后者；不要求 Agent Core / ActuCore 定制代码，
不连接旧 teleop、motion_control 或 arm 卡片。原 arm 仍可独立使用，本卡内部复用其
厂商位置发布和实测反馈。首版仅双臂，不发送手、腿或底盘目标。

本文件描述候选实现；部署、实体 PICO 和真机验收须另有证据。
正式范围与 TY-01～TY-06 见[实现契约](../../docs/plans/tianyi-teleop-control.md)。

## 配置与使用

普通 bundle 注册由 `config.yaml` 的 `teleop_control.enabled: true` 控制。
卡片齿轮配置 `mode`（默认 Shadow）、`calibration_path`、`position_scale`（默认 0.5）、
`joint_velocity_rad_s`（默认 1 rad/s）。`trajectory_smoothing` 默认为 false；显式打开后
另受 `joint_acceleration_rad_s2`（默认 2 rad/s²）约束。速度仍受 URDF 与原 1.5 rad/s 边界约束。

标定路径指向 Driver 已挂载的 `motus.tianyi-calibration.v1` JSON，必须
`hands_enabled: false`，包含实物匹配的 URDF、TCP、碰撞范围与相应执行验收资料。
仅空闲时允许修改；模型验证与本地持久化均成功后应用。保存于现有 data 挂载内的
`teleop-control.json`；`info.effective_config` 表示已应用值，`config_error` 单独报告失败，
不能把 Core 已保存的表单值当成 Driver 已生效。

1. 在 PICO 设备卡齿轮完成安装、连接和配对，连接两卡的 Cmd 端口。
2. 开启 Canvas 项目：接口就绪，不 claim、不输出运动。未绑定时返回 `waiting_binding`。
3. 在 PICO 点“开始遥操”：校验模型、准备会话，然后重新读取最新有效输入与实测 FK，
   建立一次相对映射。双握把已按住时要求先松开，防止开始瞬间使能。
4. 双握把使能；松开任意握把保持。再次双握直接处理下一有效帧，沿用原 anchor 和
   mapping_epoch。关节执行可从实测起步，但不修改手柄到机器人目标的映射。
5. PICO“结束并收臂”执行已有受控自然下垂流程，实测到位并释放才报告完成。
   PICO“立即停止”与 Canvas 停止只保持，不追加收臂。停止可取消正在收臂或冷准备的操作。

头显或输入超时只保持；输入恢复且反馈/碰撞条件满足后继续。不可达目标不执行失败解，
保持最后有效位置附近并继续尝试新输入，不增加越界搜索。真实空间重置进入
`needs_calibration`，握把不能解除；需要显式重新标定/开始新操作会话。

## 外部协议与内部执行

| 方向 | 约定 |
|---|---|
| PICO → 控制 | `/<namespace>/teleop/<设备实例>/command`；`data/teleop-cmd`；`motus.teleop.command/1` |
| 控制 → PICO | 同前缀 `/feedback`；`data/teleop-state`；`motus.teleop.feedback/1` |

格式定义在共享 `common/teleop_contract.py`。ROS 2 domain 42、`std_msgs/String` JSON，
RELIABLE / VOLATILE / KEEP_LAST 16。卡片仍只画一条前向边；反向反馈从已验证输入 topic
派生。控制实例与上游设备实例分开记录，命令和回执检查 binding 与 host clock。
MCP 管理仅接受无浏览器 Origin 的同机 loopback 请求；普通 Core 原有调用方式即可，
不增加 PICO 专用 Header 或 token。普通 arm 等其他卡的边界不因本卡改变。

输入为 X 前/Y 左/Z 上的设备跟踪系、米和 xyzw 四元数，并非机器人基座系。
`controller_to_palm` 沿用冻结版 OpenXR 局部标定，内部换基后应用；映射只在开始/显式标定建立。
手柄输入有效期 300 ms，不用头显时钟替代机器人时钟，不因心跳或转发刷新旧时间。

姿态只留最新一帧；`begin/finish/stop/calibrate` 操作另行处理，不被高序号姿态覆盖。
操作携带 request_id 与最长 5 秒入站期限；完全相同重试读回已有结果，不重复执行。
已经受理的收臂不会因请求期限或 PICO 掉线被取消。保留最近 32 条 accepted/completed/failed
回执；收到 accepted 不能显示为已完成。stop 校验绑定与有效请求，但不要求新鲜位姿、握把
或仍在线的 RTC；并发 stop 合并到一个有界停止任务，分别回最终结果。
每条回执保留原操作的 device_id / connection_epoch / space_epoch，不借用最后姿态的代次，
因此 RTC 断开后仍可正确匹配 WSS stop 的完成结果。

执行主进程拥有 MCP、会话、限速、反馈与保持/停止。IK 工作进程负责模型、FK/IK 和
运动段碰撞证明；最新值 IPC 携带会话、源序号、mapping_epoch、版本与原始期限。
保留原厂商 DDS/domain 0 与本机 DDS/domain 42 隔离辅助进程。辅助进程失联时进入可恢复
保持；重建后只接新帧。新控制路径解算结果直接交原 arm 接收函数，不增加内部 DDS 跳转。

输入、求解和执行频率独立。执行按真实有界周期推进完整关节参考；不把每个 IK 输出再次
缩成 20 ms 步长。暂停时间不累计成突然大步，松握/停止/反馈故障不经过可选平滑滤波。
运动证明先沿用冻结版可缩短的 20 ms 安全短段，再在额外有界预算内尝试扩大证明范围；
扩大失败保留已经证明的短段，不把完整参考整体拒绝。执行追到证明边沿后等新证明继续，
不因此反复 pause/resume；实测或当前命令已经越出证明时仍保持。任何反馈余量也先做碰撞
证明，执行线程不自行放宽模型、包络或限位。
worker 卡住、退出或返回旧代次结果时丢弃结果，执行主进程仍能读取反馈和停止。
`execution.armed` 与 `started` 分开表示 Canvas 接口和操作会话，均不能推断为硬件正在动；
真实输出由 `output_active`、实测反馈及具体错误说明。

## 冻结对照与离线验证

冻结 r4/r16 保留数学方向和自然下垂流程。本轮有意变化为：重握不重标定、完整关节参考
交执行插补、Canvas 停止仅保持、标准设备 DDS 替代 ActuCore 遥操。平滑默认为关闭。
直接对冻结 r4 `kinematics.py` 类执行非 identity 手掌变换对照，不只复制当前实现当参考。

2026-09-23 同一份 719 帧原始录制、同模型/标定/实测初值/1 rad/s 的离线数值对照：
冻结与候选均 508 帧求解成功、211 帧不可达；映射矩阵最大差 `1.33e-15`，508 组完整
IK 最大差 `2.07e-11 rad`。候选求解 P95 `20.11 ms`、最大 `60.76 ms`，冻结 P95
`8.57 ms`。这些是实际数值记录，不是新增验收误差门槛，也不是跟随误差或真机证明。

在仓根使用已安装锁定数值依赖的 Python 执行：

```sh
python3 -m pytest -q x-humanoid/tianyi2.0/tests/test_teleop_control*.py \
  x-humanoid/tianyi2.0/tests/test_motion_control*.py \
  x-humanoid/tianyi2.0/tests/test_motion_stream*.py
```

可设置 `TIANYI_FROZEN_TELEOP_DIR` 指向已提取冻结镜像的 `plugins/teleop` 目录，启用
64 帧实际冻结映射对照；缺少源码时明确 skip。数值测试使用真实 Pinocchio/IK 与有限速度、
加速度的非瞬时跟随 plant；进程测试实际创建 IK worker 并注入暂停/退出。总线单元测试用
匿名 socket 与 ROS allocation stub，不能代替真实 DDS、厂商反馈或硬件验收。

`tests/two_driver_dds_smoke.py` 为跨两份源码的真实 DDS 隔离集成入口。挂载 PICO 源码到
`/work/pico-src`、本仓到 `/work/control-src`，在 `--network none --read-only`、无设备且
`/tmp` 可写的候选容器中加载 ROS 环境，设置 `TIANYI_ISOLATED_DDS=1` 后执行该脚本。
它使用真正的 PICO `DeviceRuntime / OperatorCommands / RosTransport / BoundedWriter`、
控制端 DDS 隔离进程与 IK 工作进程，只将厂商硬件替换为有限速度 plant。输出两个进程的
JSONL 和最终结果，验证开始、重握、断流恢复、无输入收臂及 RTC 断开后的停止。该脚本的
存在或语法检查不代表真实 DDS 已通过；运行结果单独记录。

本轮完整测试还发现 `test_servo.py` 的 6 个既存失败：interrupt hook 期望、反馈消息 stub
与旧假时钟，与未修改 HEAD 的隔离副本一致。它们单独记录，不能把排除这些文件后的通过
报告成全仓通过，也不为通过而修改旧 servo 行为。

构建复用普通 `build.sh --mirror tuna x-humanoid/tianyi2.0`（可能发布，使用前核对授权）
或只构建的 `x-humanoid/tianyi2.0/deploy/build_teleop.sh <image-tag>`。必须带仓根 common
构建上下文；不需要额外遥操开关，不构建或替换 Core / ActuCore。当前离线通过不代表
真机通过；本轮 ARM64 镜像与真实 DDS 隔离运行已另有
[集成记录](../../docs/validation/tianyi-two-driver-integration-20260923.md)，按正式契约顺序逐项验收。
