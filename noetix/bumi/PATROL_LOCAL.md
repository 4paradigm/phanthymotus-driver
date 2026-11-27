# Bumi 固定点巡检本地实验

本实验沿用已有巡检基线，工具名为 `patrol`。巡检前的电量、运动模式、关节原始错误和数据断流检查集成在本卡内部。首版由操作员把 Bumi 放到配置的固定点位和视角，卡片采集图像、提供正常标准，Agent 或操作员输入观测结论，最后保存本地 JSON 报告。

## 在本机运行示例

以下示例使用真实巡检卡代码和明确标为合成的遥测、图像引用，无需 ROS 或机器人。

```powershell
python noetix/bumi/examples/patrol-local-demo.py --scenario normal
python noetix/bumi/examples/patrol-local-demo.py --scenario deviation
python noetix/bumi/examples/patrol-local-demo.py --scenario low-battery
python noetix/bumi/examples/patrol-local-demo.py --scenario raw-error
```

正常和异常观测场景会完成点位检查并将报告保存到 `output/patrol-local-demo/`。低电量在开始阶段返回 `blocked`，未知关节原始错误返回 `needs_review`。输出均标明 `environment=synthetic_local_demo`，报告的 `sent` 为 `false`。

## 本地驱动接入

将此工作副本的 `plugins.patrol.enabled` 改为 `true`，启用现有 `state`、`motion_state`、`camera`、`vision_capture`。本卡订阅 `/<namespace>/state/battery` 和 `/<namespace>/motion/state`，复用已有采集卡保存照片。部署配置把报告存到宿主机数据挂载中的 `/opt/phanthy-motus/data/patrol`。报告目录、最低电量和遥测超时可在配置文件调整。

驱动按配置加载一张巡检卡。现有示例 Skill 依赖 `patrol_baseline`，可关闭 `patrol` 后启用 `patrol_baseline` 继续运行。接入新卡的 Skill 使用下列 `patrol` 动作。

1. `start` 创建本卡的遥测订阅，等待输入更新。
2. `preflight` 查看电量、模式、关节和相机检查结果。
3. `begin` 传入本轮 `run_id`，检查通过后创建巡检会话。
4. `plan`、`read` 查看点位及正常标准。默认基线为 `fixed-passage` / `clear-walkway`。
5. `inspect` 传入 `point_id`，复查状态并拍照，返回 `inspection_id`、保存位置和检查标准。
6. `record` 传入 `inspection_id`、`check_id`、`decision` 和 `assessment`。判断值为 `normal`、`deviation` 或 `needs_review`。Agent 或操作员负责依据刚采集的图像填写真实结论。
7. `finish` 生成本地报告。检查项不齐时返回 `incomplete`，任意偏离或待复核项都会使 `all_normal=false`。再次调用返回同一份完成报告。
8. `stop` 结束本卡会话并释放自己的订阅。

## 判定与证据范围

默认最低电量为 20%，遥测接收超时为 3 秒。保护模式、非行走模式、关节速度达到 0.15 rad/s 或电池告警会阻止固定点采集。缺失输入、未知原始关节错误和不完整字段会进入复核。旧记录中的 `error=1` 仍按原始码展示并要求复核。

采集引用、点位和视角由卡片保存并绑定到当前会话，`record` 从这些记录生成证据。`frame_captured_at` 读取 JPEG 消息头里的 ROS 时间戳，由相机工作进程在发布时填入。该时间反映软件采集链路，镜头曝光时刻待硬件验证。缺少有效消息头时间时标为 `unknown`。缺时、超过点位基线时效或未来时间的观测进入复核。遥测新鲜度按本机 ROS 接收时刻计算，SDK 底层缓存的年龄需在真机另行核对。

本地实现已包含驱动注册、采集与报告流程。测试使用 ROS/SDK 边界桩和合成观测，真实 Bumi 的点位布置、相机时钟、状态恢复和画面判断均待实测。
