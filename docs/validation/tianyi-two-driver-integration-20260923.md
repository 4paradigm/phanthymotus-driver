# 双 Driver 本轮集成记录

对应 #329 `teleop_device` 与 #321 `teleop_control`，2026-09-23。只修改 Driver 仓；Agent Core / ActuCore 无代码变更。未切换天轶业务容器、未发送真实动作、未申请 BOT review。构建和隔离测试在已核实的天轶 Orin 个人目录执行，测试容器无网络、无硬件挂载。

## 代码与普通宿主验证

- PICO：85 项 Python 测试通过，含 47 项共享契约；原生宿主、签名 APK、localhost HTTPS/WSS/RTC 另有记录，不能把重复契约测试叠加计数。
- 天轶：477 项相关回归通过，1 条上游 hppfcl 弃用警告。覆盖真实数值工作进程、停止取消冷准备、重握不改映射、过期结果、运动证明、收臂和停止回执。日志 `/private/tmp/tianyi-two-driver-final-scope-20260923.log`。
- 未修改的 `test_servo.py` 有 6 项失败，已在原 PR HEAD `9073b283f7f76f385ffd2d43c92a4cf2bd075eeb` 的独立导出目录复现：中断 hook、消息 stub 及时间假值与实际时钟不一致。其 19 项通过及 6 项失败单独记录，未计入上述 477；不声称全仓全绿。证据 `/private/tmp/tianyi-321-base-servo-20260923.log`。
- 普通 Core `9802eae43f158af75a9848909d7366d15c985e63` 原样导出，在真实 Chrome 的最小 DOM 场景运行实际齿轮、Canvas、活动监控 JS，以及 Core API/SQLite 和 PICO MCP。注册发现、配置保存与读回、分享密码脱敏、启动、输入文字监控、停止通过。不是完整生产页面或真实 ROS 监控桥接。8 个执行源文件的哈希记录于 `/private/tmp/pico-core-upstream-integration.json`。
- 普通齿轮只提供文本网址，用户已接受复制到浏览器；没有修改 Core 或注入 HTML。安装说明放在普通齿轮实际展示的 tool description。

本轮额外修复：保存配置与启动重叠导致密码临时文件竞态；RTC 断流后停止回执被旧姿态代次误丢；迟到的收臂操作在停止后重新受理。最后一项由另一子 Agent 独立复现并复核修复，取消后不再派发收臂，迟到完成不覆盖新状态。测试 fixture 曾把 stdlib `time.sleep` 全局替换，造成后续反馈测试失真；已改为仅替换 fixture 的模块引用，没有放宽生产时效。

477 项最终回归命令（在本仓根目录，使用本轮锁定数值依赖环境）：

```bash
TIANYI_FROZEN_TELEOP_DIR=/private/tmp/two-driver-baseline-20260923/frozen-actucore-r4/plugins/teleop \
/private/tmp/four-card-motion-tests-20260923/bin/python -m pytest -q \
  x-humanoid/tianyi2.0/tests \
  --ignore=x-humanoid/tianyi2.0/tests/test_servo.py \
  tests/test_teleop_contract.py \
  tests/test_tianyi_build_context.py \
  tests/test_tianyi_build_diagnostics.py \
  > /private/tmp/tianyi-two-driver-final-scope-20260923.log 2>&1
```

## 冻结数值对照

从实际保留镜像提取源码，没有启动冻结服务：

| 组件 | 冻结镜像 |
|---|---|
| ActuCore unified r4 | `sha256:584e7d3402c52c93a976f39740057dd73b9b90a06d0d4f935f356e035104c091` |
| Driver operator r16 | `sha256:f41c7bb5352a29d3a41dc3d5d6357000a2634ad27cfb580d4546a9b1476288c1` |

使用同一份 719 帧录制、同模型与标定、同帧实测关节初值、1 rad/s，分别运行实际冻结映射/IK 和候选映射/IK。每帧清除数值缓存以隔离求解差异；没有把目标直接当实测反馈。记录不含真实执行回放，因此不能作为跟随误差或硬件通过证据。

| 指标 | 冻结版 | 修复后候选 |
|---|---:|---:|
| 成功求解 | 508 | 508 |
| 不可达 | 211 | 211 |
| 其他拒绝 | 0 | 0 |
| 求解 P95 | 8.57 ms | 20.11 ms |
| 最大求解耗时 | 39.41 ms | 60.76 ms |

映射矩阵最大绝对差 `1.33e-15`，508 组完整 IK 解最大关节差 `2.07e-11 rad`。这些数值如实报告，不新增误差阈值。

初次对照曾暴露严重回归：候选仅 19 帧成功、475 帧躯干碰撞、14 帧超时。原因是一次证明过大的运动范围，拒绝了冻结版可执行的小步。修复保留完整 IK 参考，先沿用冻结版安全短段，再在有界预算内扩大已验证范围；扩大失败保留短段。执行只在已证明范围内推进，抵达边沿等待新证明，不反复重建会话。没有放宽躯干模型、碰撞边界或关节限位。

可复现入口 `tests/compare_frozen_teleop_recording.py`，通过 `--frozen/--record/--profile/--urdf/--output` 指定私有资料。原数据不进 Git。证据目录 `/private/tmp/two-driver-recording-ab-fixed-20260923`。

| 数据 | SHA256 |
|---|---|
| 录制 | `20868ccc0c8e3c89e2f26f3007cbb6e7a73e24c299c23245cd33ad1dc86f9096` |
| 原标定 | `95a8c68b11b74ef71f71fb09f36311b1b296d27e490997f02dc153d828cdbc64` |
| 官方模型 | `a7e742ad600c7f1e9eeecdd04046dea4cd5f81cccb6dde73f290968b345e4b20` |
| 冻结数值源码 | `86dcbd72faa4b6b2d1bcdac49023d29a2ef2f32a89733ff14a5b1b698443bf3c` |
| 候选数值源码 | `be0113a2a5f86e543af840f7cac39ad62054f14cb553a63939be62df1b53684f` |

另用直接提取的冻结类完成 64 帧非 identity 手柄至手掌变换对照，防止默认单位变换掩盖坐标错误。

## 普通 ARM64 构建与真实 DDS

在天轶 Orin 使用原样 `build.sh --mirror tuna pico/pico`、`build.sh --mirror tuna x-humanoid/tianyi2.0`，均成功；无 BOT 修改、无遥操专属开关、无镜像传输。天轶首次构建因继承腾讯内网 apt 地址而失败，已把该 Dockerfile 默认 apt 地址改为公开 HTTPS 源并复构通过；不修改宿主网络或 Docker daemon。

| 镜像 | 最终本地镜像 ID |
|---|---|
| PICO | `sha256:94768a952f6dd0f5c041e0ca67e7d72c40405635fa78b76ca7824604a1c5a806` |
| 天轶 | `sha256:941c3bb99ad6e6641ca7e9715ff941883145e7c3679c0de1369dd83c06c50b4d` |

工作目录 `/data/hanzebei/pico-tianyi/updates/two-driver-20260923`，日志 `pico-build-final.log`、`control-build-final.log`。标签使用本任务独立 namespace，不覆盖业务镜像。

`x-humanoid/tianyi2.0/tests/two_driver_dds_smoke.py` 在 `--network none --read-only`、无设备挂载、2 CPU / 4 GB 的容器中执行。真实 DeviceRuntime、OperatorCommands、RosTransport/BoundedWriter、ROS 2 DDS domain42、控制卡、DDS辅助进程、独立数值进程和 arm 接收逻辑参与；仅厂商输出与反馈替换为有限速度/加速度 plant。

最终结果：162 帧输入、244 次模拟执行写入；10 轮松握重握均保持同一操作者会话与 `mapping_epoch=1`，同时有实际 plant 位移；20 Hz 输入、断流保持与恢复通过。开始、无新头显帧的收臂、RTC 断开后的停止均收到 completed。结束后 plant 回到其合成模型零姿态并释放，不代表真实机器人自然下垂已经验收。证据 `evidence/dds-final/{producer.result.json,control.result.json,producer.jsonl,control.jsonl}`。

## 未完成阶段

- 完整生产 Canvas 两卡页面、真实 PICO 浏览器下载安装/信任/配对与透视按钮。
- 同一录制的完整冻结执行链 A/B 与实际厂商 `/arm/cmd_pos`、`/arm/status` 跟随证据。
- 开发真机测试、随后针对精确提交的 BOT review、最终真机验收。

本轮末次只读检查，Canvas 无编辑锁但项目正在运行，因此没有切换业务服务。进入部署前仍须重新核对使用情况、实际配置与控制状态；不能把历史空闲、构建通过或隔离 plant 当成部署/动作授权。
