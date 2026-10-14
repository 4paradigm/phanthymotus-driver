# User-intent control checkpoint — 2026-10-13

用户此前要求暂停，检查点为 `e649701`；独立保存分支为 `codex/piper-user-intent`。
2026-10-14 用户要求部署供自行测试，已按下方续接记录恢复工作。

## 已完成

- 新增独立 `intent_control.py`、`intent_motion.py`、`intent_monitor.py`，默认入口接入英文用户动作。
- Move 使用固定基座方向和官方 MOVE_L；相对目标只计算一次，保持朝向，检查末端组成帧、状态、关节限位与完成反馈。
- 正常状态下自动准备模式/电机，夹爪先保持测得开度使能再执行；支持整个任务取消和异步回报。
- Arm control 声明骨架和英文参数话题，后台只公开 model，不需要独立 piper_status 节点。
- 保留旧控制类及历史回归，也保留此前新增实时相机提交 `4254211`；生产相机文案按新 runtime 切换英文。
- 更新 Docker COPY 和 driver.yaml；旧入口测试改为明确验证历史接线，新的生产接线由 test_piper_intent.py 覆盖。

## 已有验证

- 指定集合：243 passed、153 subtests passed。
- 此后调整了相机英文错误包装和完成前取消检查；相关集合再次验证 27 passed、35 subtests passed。
- 实际未修改 Core dashboard：只有一个 arm_control 画布节点，能订阅骨架及参数，并通过真实 MCP ID 请求 PiPER model；浏览器无脚本错误。
- 证据：工作区 `acceptance/20261013_piper_intent/intent-one-card-dashboard.png` 和 `browser-result.json`。
- 所有动作测试均使用模拟 SDK，未执行真机动作。

## 暂停时记录的剩余工作（续接完成情况见末节）

1. 将新 runtime 的 task 状态同步与生命周期锁分开，保证停止等待 SDK 发送时，后台监控读取任务状态仍不阻塞。
2. 将最后一次取消检查和 completed 状态发布与 Stop 串行，补完成边界取消的并发回归，避免停止信号与完成发布之间的小窗口。
3. 补缓存完成标志、朝向偏差以及准备失败等针对性测试，再运行完整指定集合；整理最终接口/验收文档和实际控制表单预览。
4. 检查新接口下可选相机错误/结果文案，保持已有相机行为和现场时间戳修复。
5. 真实末端参考点、方向、自动准备、夹爪和停止尚未验收；当前代码是本地候选版本，不能宣称已完成现场交付。

当前 Core 的 actuator 数值输入没有预填 schema default 的实现；本版把默认值与范围放在占位提示开头，并在驱动内处理缺省/空输入。后续验证实际表单，不为了填默认值擅自修改 Core。

## 现场与授权状态

- 最近只读检查：机械臂 1 正常 CAN、六轴和夹爪使能、无故障、无当前动作；这些是历史检查结果，后续操作前重新读取。
- 镜像：`agilex-piper:local-arm1-live-camera-ts-a3a557ea`；智能控制当时开启。
- 用户已选择“先保留本地版本，暂不部署”。停止了等待 sudo 的只读源码复制命令，没有执行部署。
- 下次继续应先完成本地工作；现场部署/动作验收需遵循最新用户安排，不沿用历史状态直接操作。
- 保持 Core 不改、先仅机械臂 1、保留原 smoke-test.py 与部署备份；不将凭据写入 Git。
- 已发布 PR #382 的历史基线：`7d9e997`，Version `release.261013.74ab514`；此候选版本没有 Bot Version。

## 2026-10-14 续接与候选部署

- 已分离任务锁与生命周期锁，停止等待 SDK 时监控读取任务不等待生命周期锁。
- 完成发布与 Stop 串行；在异步预留执行权时先发布 preparing，防止早期停止遗漏，
  或 Stop 将已完成任务永久覆盖为 stopping。
- 补完成边界、异步线程启动前停止、缓存完成标志、朝向误差及准备失败回归。
- 指定集合：248 passed、153 subtests passed；候选镜像内以禁用网络、无设备映射方式运行
  20 项新接口模拟 SDK 测试，全部通过。
- 用户关闭智能控制后，仅重建机械臂 1。镜像 `agilex-piper:local-arm1-intent-9b5ae0e8`，
  备份 `/opt/phanthy-motus/backups/piper-arm1-intent-20261014-101007/`。
- 核对 Core 与机械臂 2 镜像及启动时间一致，camera_worker.py 哈希一致；保留现场配置，
  未用本地旧 runtime 覆盖现场，已核实 runtime.py 与 arm.py 内容一致。
- 画布移除机械臂 1 的独立 piper_status 节点，Arm control 声明骨架/参数话题；保留实时相机节点。
- 只调用 info 验证六轴与末端反馈新鲜且时间推进；未发送运动、模式、使能/失能命令。
  实际 Edge 确认英文动作菜单及参数范围提示，手动测试参数预置 1 mm、1%。
- 后续真实方向、自动准备、夹爪与停止验收由用户进行；尚未更新 PR 或请求新 Bot 构建。
  本候选版本 Version：尚未生成。旧 release.261013.74ab514 不代表此候选版本。
- 证据目录：工作区 `acceptance/20261014_piper_intent/`；部署脚本和只读验证记录已保留。

## 2026-10-14 退出示教并保持位置候选

- Arm control 新增 `exit_teaching_and_hold`（Exit teaching and hold），参见 `TEACHING_EXIT.md`。
- 先查询当前连接固件、确认静止与正常全使能反馈，再通过固定 SDK 的 CAN/MOVE_J
  以实测关节位置保持。无 Reset、回零或失能，普通 Move 仍拒绝示教模式。
- PiPER 测试集合：190 passed、186 subtests passed；候选容器内禁用网络且不映射设备，
  32 项相关模拟测试通过。现场只验元数据、反馈和相机，没有执行新动作。
- 已部署机械臂 1 镜像 `agilex-piper:local-arm1-teaching-hold-ef3d4022`；
  备份 `/opt/phanthy-motus/backups/piper-arm1-teaching-hold-20261014-115424/`。
- 保留新从臂 CAN1 和相机 /dev/piper-camera-2 绑定，Core、机械臂 2、相机 worker 未改；
  部署前后示教模式、关节角度及使能反馈一致。浏览器刷新后卡片说明已显示新动作。
- 现场退出示教与保持效果待用户验收；尚未推送 PR #382，新功能 Version 尚未生成。
- 证据目录：工作区 `acceptance/20261014_piper_teaching/`。

## 2026-10-14 方向微调候选

- 用户确认退出示教成功后点击 Move，现场只读检查为 MOVE_L + TARGET_POS_EXCEEDS_LIMIT；随后夹爪动作在预检查被拦截。此轮实现与部署未发送任何真机控制命令。
- 用户认可方向微调方案并要求执行。工具改成六个独立方向动作，每次 1 mm、固定 1% 速度，保留英文和基座坐标。旧 move 请求拒绝；大步入口留待真机验证。
- 内部较长请求逐个确认 <=1 mm 子目标，始终以原始起点计算；异常/停止不再发送后续目标。返回请求距离和实际位移，记录首个拒绝反馈。
- 增加 per-device 有界 JSONL 动作日志，目标写入成功后才可发送；Info/后台 Task 提供诊断。固件未恢复正常时仍拦截，未自动清错或绕过保护。
- 修正已使能待机状态的自动准备；兼容 Core 在只读 Info 中附带的空 input_topic，动作参数保持严格校验。
- 模拟和未修改 Core 的卡片/骨架浏览器测试已执行；最终数量与证据见工作区 acceptance/20261014_piper_direction/。接口和验收边界见 DIRECTION_JOG.md。
- 本地 203 passed、195 subtests passed；未修改 Core 的真实卡片/后台骨架浏览器验证通过。候选镜像无网络/设备映射环境运行 45 项相关模拟 unittest 通过。
- 智能控制重新读取为关闭且无当前任务后，仅部署机械臂 1：`agilex-piper:local-arm1-direction-b38149e6`；备份 `/opt/phanthy-motus/backups/piper-arm1-direction-20261014-124447/`。
- 部署前后 CAN 模式、使能和关节/末端反馈一致，超限报警仍在；未自动清错。元数据、PiPER 模型、反馈时间推进及相机约 10 fps 已只读验证。
- 未推送 PR #382，Bot Version 尚未生成，等待用户真机测试。机械臂 2、Core、现场配置、相机 worker 及原 smoke-test.py 未改。

## 2026-10-14 重复手动执行与 Stop 入口修复

- 用户点 Move forward 后重复执行报 stopping。现场日志和代码确认：Core 每次执行前调用 Start，历史 runtime 对所有占用中的任务都报 still stopping，连手动 Stop 也可能被挡住。
- Intent Arm control 单独兼容 Start：已有任务时只返回 busy/stopping，不修改取消、就绪或互斥状态；新动作继续拒绝，Stop 可到达；其它历史卡不变。
- 206 passed、195 subtests passed；候选镜像无设备/网络运行 48 项相关 unittest 通过；未修改 Core 卡片与骨架浏览器验证通过。
- 新鲜确认智能控制关闭、当前无任务后仅部署 arm 1：`agilex-piper:local-arm1-lifecycle-ea016db5`；备份 `/opt/phanthy-motus/backups/piper-arm1-lifecycle-20261014-125747/`。其它容器、现场配置、相机 worker 不变；仅 Info 验证反馈及相机，未执行控制。
- 原向前动作证据保存：起点 [53.907,2.765,220.173] mm，目标 X+1 mm，实际位移 0 mm，30 秒超时。现在又有目标超限报警。此修复只处理生命周期误报和 Stop 可达性，真实 MOVE_L 无位移原因仍待排查/验收。
- 未推送 PR #382，Bot Version 尚未生成，原 smoke-test.py 保留。证据目录 acceptance/20261014_piper_lifecycle/。
