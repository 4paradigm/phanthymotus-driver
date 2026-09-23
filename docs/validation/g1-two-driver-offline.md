# G1 两卡离线验证记录

2026-09-23。初始审计后已修改并部署两卡候选，未发硬件动作。**完整实体 PICO 与真机门禁尚未通过**。

## 北京 G1 部署与无动作联调

PICO `two-driver-20260923-r2` 与 G1 `two-driver-site-20260923-r3` 均从源码在 ARM64 机上构建。Docker Hub 超时后，核验机上已有 micromamba 2.3.2 压缩包与二进制后复用；现场构建文件将不可达腾讯内网 apt 源及缓慢 PyPI 改为清华源，宿主全局配置未改。完整普通 Dockerfile 的网络可移植性尚未验证，不冒称原命令一次通过。

实际旧 G1 容器属于 `pr-image` 项目，其原 `/tmp/pr-image/docker-compose.yml` 已不存在。第一次用主项目切换发生同名冲突，旧容器未中断；随后按 Docker inspect 重建等价的个人目录 Compose，仅替换镜像，保留挂载、网络、PID/IPC、环境和启动命令。主 Compose 恢复原始字节。Core、ActuCore 容器 ID 均未变化。部署产物和回退镜像记录在机上个人目录 `updates/two-driver-20260923-r1/evidence/`。

原预览标定腿腰全零、FSM 500，与实测站姿及 FSM 801 不同。用户确认 500/801 都允许，新增模式允许列表，相关回归 31 passed；读取两秒稳定反馈均值生成仅 Shadow 使用的姿态记录，未补填任何验收标志。双臂电机错误码均零；更新后反馈 fault=false、model_verified=true，SDK 样本年龄约 0.73 ms，实际关节送入机上 IK 子进程自检通过。启动/订阅退出阶段出现一次 SDK reader 空引用日志，未观察到持续反馈中断；不以此断言 SDK 回调问题已修复。

真实 ROS 2 在禁网临时容器内验证 PICO 格式输入→实际 TeleopControl→反馈，未创建 SDK 发布者。机上 domain 42 可发现 G1 遥操节点和普通节点，从 PICO 容器只读订阅关节监控收到数据。该测试不包含实体头显。普通 Core 目录已列出两卡，原画布 7 卡及执行连线完整保留，新增两卡及命令连线；项目停止、Shadow、无租约、applied_sequence=-1。实体 PICO 尚未配置/配对，未完成真机动作或 BOT review。

## 最新 servo 位置下发回归

按用户确认，两卡入口启用 `servo_position`，不构造重力模型，不做限速插值；通过 `ArmSdkChannel.publish_arms` 发布完整目标并检查 SDK 写入结果。断流仅忘掉目标、保留权重，不持续推进旧目标。碰撞包络改为完整目标范围；旧四卡模式独立保留。

实际执行 `pytest unitree/g1/tests/test_teleop_servo_execution.py unitree/g1/tests/test_arm_stream.py tests/test_g1_arm_sdk.py -q -p no:cacheprovider`：51 passed。新增测试检查完整目标单次下发、无重力计算、超时保持后同会话恢复和写入失败；SDK 与反馈是替身。另在固定数值环境执行 `pytest unitree/g1/tests/test_motion_control_numeric.py -q -p no:cacheprovider`：3 passed，使用真实 G1 模型与 IK 子进程。未声称 DDS 完整集成或真机通过。

## 已运行的回归

| 对象 | 结果 | 边界 |
|---|---|---|
| G1 arm、卡片绑定、旧 motion 协议、原 SDK/servo | 100 passed、1 skipped，2.42 s | 跳过 `test_g1_servo_eef.py:362`：宿主缺少 ROS String；不算通过 |
| G1 真实模型、IK、数值子进程与配置 | 3 passed，4.88 s | macOS Pinocchio 3.1/CasADi 环境，不是目标 ARM64 镜像 |
| PICO 设备、生命周期、配对、传输和共享契约 | 85 passed，2.29 s | 本轮重跑，不代表实体头显验证 |

G1 前两组只读加载 `g1-motion-contract-20260923` 的未提交候选，没有搬运或修改该工作树。PICO 为 `pico-two-driver-20260923`。命令分别为：

```bash
# G1 候选根目录
PYTHONDONTWRITEBYTECODE=1 /private/tmp/four-card-motion-tests-20260923/bin/python -m pytest \
  unitree/g1/tests/test_arm_stream.py unitree/g1/tests/test_arm_card_binding.py \
  unitree/g1/tests/test_motion_control_contract.py tests/test_g1_arm_sdk.py \
  tests/test_g1_servo.py tests/test_g1_servo_eef.py -q -rs -p no:cacheprovider
PYTHONDONTWRITEBYTECODE=1 /private/tmp/g1-motion-numeric-20260923/bin/python -m pytest \
  unitree/g1/tests/test_motion_control_numeric.py -q -rs -p no:cacheprovider
# PICO 候选根目录
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=.:tests /private/tmp/ext-vr-tests-20260923/bin/python -m pytest \
  tests/test_pico_device.py tests/test_pico_lifecycle.py tests/test_pico_onboarding.py \
  tests/test_pico_transport.py tests/test_teleop_contract.py -q -p no:cacheprovider
```

## 组合对照

脚本 [validate_g1_offline.py](../../scripts/validate_g1_offline.py) 使用真实 MotionControl、独立 NumericalWorker、模型、碰撞检查、ArmStreamExecutor 与最终关节 RNEA。SDK、厂商 action 99、反馈和内部 DDS 是明确替身：JSON 序列化后直接调用消费端，反馈以最多 0.6 rad/s 推进，执行器配置 1 rad/s；不直接将目标当反馈。测试专用 `_prepared=True` 只对这个隔离执行器生效，没有填写或修改任何真实验收标志。

| 场景 | 实际观察 |
|---|---|
| 模型 FK 生成的 71 个可达目标，20 Hz 输入 | 71 次关节目标被接受、776 次模拟 SDK 写入；最大模拟关节运动 0.156996 rad |
| 同轮显式暂停/恢复 | 两个操作无错误；执行租约更新，发送目标的 mapping_epoch 始终为 1。这不是实体双握把入口验收 |
| 同轮插入 400 ms 输入空窗 | 出现 command_timeout 保持；之后 36 次目标接受，空窗前后执行会话相同 |
| 单帧不可达输入，再恢复可达轨迹 | 第 25 帧注入；第 29 帧重新接受，最终 66 次接受、798 次模拟写入；之后又经历断流并恢复，没有永久卡死 |
| 结束/release | 各轮 action 99 替身只调用一次，五次零权重写入后等待新的模拟反馈，最终释放占用。状态中的 physical_confirmed 来自模拟反馈，绝不代表真机确认 |

真实录制是已有 `poses.json` 的 71 帧，以 20 Hz **重采样**；不是按原采集时序回放。采用原录制第一帧的十关节位置为模拟起点、当前 G1 相对映射、测试几何和相同模型，仅 2 个关节目标实际被接受、30 次模拟写入，最大模拟关节运动 0.076233 rad。71 次状态采样中观察到碰撞 52 次、command_expired 11 次、motion_envelope_stale_start 5 次、target_published 3 次。状态采样可能重复或漏过一次决策，**不能将这些采样计数当作各输入帧的精确结果**；真实接受计数来自执行入口回调。

零位起点对照也仅前两次更新成功，多数采样为 torso_collision。起点、几何及记录不是冻结现场的完整快照，因此目前不能将回放失败归因于北京实物，也不能擅自关闭碰撞检查。应先恢复确切映射/模型/标定组合，再单独定位包络起点与过期问题。

重现可达轨迹（从本工作树根目录）：

```bash
PYTHONDONTWRITEBYTECODE=1 /private/tmp/g1-motion-numeric-20260923/bin/python \
  scripts/validate_g1_offline.py \
  --candidate ../g1-motion-contract-20260923 \
  --pico ../pico-two-driver-20260923 \
  --mapping ../pico-g1-actucore/actucore/plugins/teleop/g1_mapping.py \
  --recording /private/tmp/pico-g1-acceptance-0919/poses.json \
  --trajectory reachable --output /private/tmp/g1-two-driver-offline-20260923/reachable.json
```

增加 `--inject-ik-failure` 为故障注入；替换 `--trajectory reachable` 为 `--seed recorded` 为录制起点回放。脚本退出 1 是有意保留的整体不通过结果，详情写入 JSON；不能只看模拟 release 成功。

## 产品接线缺口

PICO 生成的合法 `motus.teleop.command/1` 消息通过其真实契约校验，但送入 G1 `receive_eef` 返回 `invalid_command_fields`。G1 当前暴露 `motion_control`，输入仍为 `/g1/motion/control/command`、`control/eef`；输出仍为关节 topic 和 `motus.motion.feedback/1`，不是两卡的 command/feedback 配对。

需要在 G1 Driver 内完成 `teleop_control` 接入：设备身份/输入代次、begin/finish/stop 回执、一次性相对映射、握把使能、输入过期、双向显示反馈，内部复用现有 IK/连续执行/arm.release。此逻辑尚未由本次验证脚本实现；脚本不充当产品适配层。Core/ActuCore 无需为离线验证修改。

冻结周六版本的镜像 ID、最终源码 manifest 和现场标定仍未齐全；当前 mapper 的哈希随报告记录，但不宣称它就是冻结现场字节。不能以天轶数值对照或 G1 现有源码单测替代该 A/B。

本地原始报告在 `/private/tmp/g1-two-driver-offline-20260923/`：`replay.json`、`reachable.json`、`recorded-seed.json`、`ik-recovery.json`；包含源码/录制 SHA256、逐次状态和接受序号，不纳入公共 PR。

## 北京 G1 开机后只读检查

用户通知开机后，通过本机现有 `bj-g1-wifi` 与已登记 HostKeyAlias 连通：aarch64、hostname ubuntu、Wi-Fi 地址符合设备档案。Canvas 无编辑锁、项目未运行；普通 Driver 为 `release.260922.1155296`，无 OOM/退出错误。未启停服务。

普通 Driver `tools/list` 没有 `teleop_control`，机上标定版本为 `BEIJING-SHADOW-PREVIEW-NOT-LIVE`，不是完成实物验证的 Live 标定。尚未切换候选，不把“开机可用”当作两卡已部署。ROS 图检查单独记录，不根据容器 Up 推断数据链正常。

实际 domain 42 图发现 22 个 topic、12 个节点（包含临时只读探针），包含 g1_low_state、g1_loco_state、Core subscriber 等；关节/运动状态 topic 有发布者和订阅者，没有新遥操 topic。读取 `/ubuntu/state/joints` 时先因探针默认 RELIABLE 与发布者 QoS 不匹配未收到数据；改用 BEST_EFFORT / depth=1 / VOLATILE 后，3 秒窗口收到 17 条，末条到达年龄 0.2 ms、最大到达间隔 104.06 ms。窗口包含发现时间，不能拿 17/3 作为稳定发布频率；该公共监控 topic 也不能替代 SDK 高频实测反馈。第一次未 source Humble 的探针缺少 rclpy，加载既有环境后正常，未安装依赖或修改容器。

## 2026-09-23 单向输入部署及发布者门槛排查（最新）

以上开机检查和旧两卡缺口为历史记录。当前已部署 device r6、control r7、APK v26，固定 `/teleop/command` 输入与 `/teleop/state` 监控；无 PICO 操作请求或反馈订阅。Canvas topic 迁移读回通过，其余卡片保留。

用户反馈按住不动时，只读确认：Canvas running、设备 collecting、输入序号 10298 已到 control；control HOLD 原因为 `external_arm_publisher`，arm idle、无执行权、applied_sequence=-1，无本轮动作输出。另有 `ik_worker_unavailable`，旧实现吞掉了 IPC 异常类型和子进程 stderr，不能倒推具体历史根因。

按用户指令本地删除发布者数量/观察等待门槛，保留其他反馈检查；新增 IPC timeout/exited 分类和异常日志。44 项执行/配置/IPC 诊断回归通过，真实模型数值 3 项通过。G1 上以 r7 镜像禁网、只读、无硬件挂载启动独立数值进程，20 次 render 全部成功，1.52–5.15 ms；这不是实时求解或真机跟随通过。现场容器未 OOM，短时总 CPU 374%，不足以证明历史超时原因。

本次新源码未部署；未停止运行画布、未触发运动、未改 Core/ActuCore。需下一次更新后依据具体 IPC 日志判断原故障，不能声称已解决 IK 现场异常。

## 2026-09-24 r8 部署

用户授权部署实测后，检查无编辑锁、arm idle/无控制权/无输出；停止上一轮画布，机上构建 `local/phanthy-motus/g1:ipc-20260924-r8`，只更新个人 Compose 中 G1 镜像。实际 Driver 容器 ID 为 88c585b752b4，Core 4723e02be334 与 ActuCore f9b09d66534d 保持不变。启动后反馈 fault=false，采样年龄约3.8ms，无标定错误。新轮观察记录在 `/hanzebei/pico-g1/updates/ipc-20260924-r8/observation.jsonl`。首次观察脚本未兼容尚未启动时 feedback=null，已改为空对象后继续；该脚本错误不属于 Driver 故障。

## 2026-09-24 r8 实测后状态门禁调整（尚未部署）

r8 已接收新输入，但记录反复为 torso_collision，applied_sequence=-1；后续 robot_safety_not_ready 锁存。只读订阅2秒共1864条，末帧只有 right_ankle_pitch 相对标定 +0.02009 rad、waist_yaw -0.02193 rad 超过旧0.02门槛，无电机/电压/温度异常触发。此为采样时刻证据，不倒推所有历史帧。

按用户最新明确要求，本地取消自加硬件状态及腰腿偏差 fault 门禁，保留原始电机遥测和协议/反馈时效检查。45项回归通过，包含带电机错误/异常电压温度的有效模拟帧不再被软件阈值阻塞，格式损坏仍拒绝。未更改运行服务、未清除现场锁存、未真机验证本次修改。torso_collision 与保持恢复仍未解决。

## 2026-09-24 碰撞与恢复、工作树归属（尚未部署）

现场实测 q 的机上禁网隔离检查：当前姿态及九个三轴偏移目标通过，向内0.1m可复现运动包络拒绝，向内0.2/0.3m出现姿态或运动段拒绝。旧实现连续4次失败后返回原始目标，首个返回帧可因滤波历史再次被拒绝。缺少当时完整手柄四元数和目标录制，不冒称用户原帧回放。

按最新要求关闭 G1 附加碰撞/工作区扫描，保留关节限位。修复失败滤波历史与 servo 恢复等待：同会话新有效帧不再等待速度归零，物理停止确认不伪填。50项接口/执行测试、5项真实数值测试通过，尚未部署或解除机上旧租约。

开发树被清理后从用户提供的 Trash 归档恢复，核对旧 PR 树全部 adopted-source-manifest 条目无分歧，26个两卡增量文件迁入开放 PR #330 工作树；目标原有四卡计划修改保留。迁移前备份及逐文件哈希在 /private/tmp/g1-pr330-adoption-20260924。未提交或推送，不把工作树迁入说成远端 PR 已更新。

## 2026-09-24 r9 部署完成

用户手动重启解除旧版锁存后，实时检查 Canvas 停止、无编辑锁，arm idle/无控制权/无输出。仅将 G1 镜像替换为 `local/phanthy-motus/g1:recovery-20260924-r9`，实际容器 49970ef77afc；Core 4723e02be334 与 ActuCore f9b09d66534d 未变。切换前后 Compose 差异只有目标镜像。启动反馈年龄约3.16ms，无标定错误，无动作输出。PICO connected=true、设备卡 idle，项目尚未启动。

机上 r9 禁网只读数值验证通过：独立 worker 报 collision_checks_enabled=false，向内目标求解返回10个有限关节值，无硬件输出。当前运行卡片尚未加载求解器，info 的 collision_checks_enabled=null，不能将 null 解释成已完成运行态标定。真实跟随及失败恢复等待用户启动 Canvas/握把验收。

### 2026-09-24 平滑与 1 rad/s 限速（r10 已部署，待实测）

参考 PR #322 从上一条实际下发位置接续的原则，G1 先做120ms时间常数的一阶平滑，再将各关节增量裁剪到 ±1 rad/s × dt。dt 使用成功下发间隔，最大50ms，避免断帧或重握积累大步长；基准不是每帧实测位置。只处理最新输入，不排队补发旧目标。此限制约束下发位置参考的变化速率，不是实测电机速度的硬保证。

遥操专用肩肘 kp=80/kd=3，腕部40/1.5；不修改普通 servo 的全局增益。松握重握在清空旧待发目标后可 resume，无需速度或位置静止确认；显式停止确认仍独立保留。已部署 r10，当前空闲、无控制权；异响原因及新一轮实体跟随仍待验收。

2026-09-24 r10：用户停止 Canvas 后旧会话 release_confirmed，ownership_held=false。机上构建 smoothing-20260924-r10，禁网只读 SDK 替身核验肩肘80/3、腕40/1.5通过；manifest核对通过。仅切 G1 Driver，实际容器 df3c0a2f036a，Core/ActuCore ID 保持不变。启动后 idle、无执行权、无输出，反馈年龄约3.05ms。没有主动发动作；此前磨齿/咔嚓声仍未确定原因，不宣称降低增益已解决机械异响。


## 2026-09-24 基线提交与 ROS 接收修复（未部署）

r9 运动基线已提交/推送 PR #330，commit `8a0fe61f2c1695da1906fdf786dff19710b5858d`。用户确认能动且有咔嚓声。基线镜像 image ID `sha256:a7b1105c770ebeea0d36133122ee54fe6c39e69485be25c5ab069c44ff351b38`；arm_sdk、arm_stream、teleop_control、teleop_bus、main 与保存镜像逐文件哈希一致。97项接口/执行、5项数值测试通过；不是整体验收或异响消除。r10及新ROS修复留在工作树，未混入基线。

当前修复116项单元/接口回归通过。真实 ROS 隔离脚本为 `scripts/validate_g1_ros_lifecycle.py`：在 G1 的既有 r10 镜像中覆盖内存模块源码执行，Docker --network none --read-only、临时 /tmp、无挂载/硬件/服务端口，不启动 main 或厂商SDK。540帧接收、20次topic切换、3次注入 InvalidHandle 后持续接收；创建/销毁均为 g1-teleop-ros 唯一线程。错误保持和下一有效帧清除通过，正常退出join通过。脚本要求 G1_TELEOP_ISOLATED_TEST=1；普通执行会拒绝。

首次隔离运行发现 `rclpy.handle` 在机上Humble不存在；改用 `rclpy.impl.implementation_singleton.rclpy_implementation.InvalidHandle` 后重测通过。未重启或切换运行服务，未发动作。普通Core完整重启/重新启动画布联调、完整新镜像构建与实机跟随未验证。


## r11b 部署验证

r11 镜像真实ROS隔离收发540帧、20次换topic和3次异常注入通过。切换前发现旧版空闲stop制造释放占用；未强行切换。用户重启旧Driver后，加入两项stop幂等修复，119项回归通过。r11b机上禁网构建，镜像内三次空闲stop均no_op，无SDK通道、无释放任务、无控制权；不表示物理停止验收。

仅更新G1 Compose image，容器cd6d0395038a、镜像local/phanthy-motus/g1:ros-receive-20260924-r11b。Core4723e02be334、ActuCoref9b09d66534d、PICO容器ID逐项核对未变。启动后arm idle、ownership/output=false、序号-1、无标定错误；两个ROS执行器心跳正常，invalid_handle_count=0。项目保持停止、binding=null，监控正确为waiting_binding。保留旧配置before.compose.json及deployment-check.json，位于机上个人更新目录。实体PICO跟随等待用户启动，未自动执行动作。


## r11b 实体 PICO 跟随反馈

用户在部署后启动项目并测试。机上 `input-observation.jsonl` 的18秒窗口：有效输入3219→4526、subscription_generation=1、ROS健康；执行目标序号非负，松开后operator_pause且hold_confirmed/resume_ready为true。执行会话恢复会重置序号，不将不同会话序号作连续比较。后续arm静态采样的last-command与实测差约1.312rad，处于松握保持且已forget_target，不能把该旧指令差当作运动期间跟随误差，也不能据此断言跟随误差合格。

用户反馈“这次的效果很满意”，对跟随、松开重握和声响的追问回复“正常的”。记录为本轮用户现场体验正常；没有独立噪声测量、长时测试或全轨迹误差统计。用户授权提交PR，Core/ActuCore仍未改变。
