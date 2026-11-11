# U1 Pro 头部单次诊断（尚未实现 head_orientation 卡片）

本目录不注册卡片，不由驱动导入，不改变 sound_direction、镜像或启动配置。
选择 **SoundSource** 作为唯一可选发送路径：它就是失败验收的路径，保留厂商运动学和姿态检查，
先定位原路径问题，再考虑产品实现。不会同时尝试 ViewTarget 或直接轨迹控制。

**停止能力：未确认。** SoundSource 是服务，没有 action goal handle。
退出工具、Ctrl-C、超时和销毁 client 都只是结束观察，不是停止机器人。
发现的 `/mc/uworld_motion_stop` 类型为 `uworld_motion_msgs/srv/Trigger`，但其作用域、
是否终止 SoundSource 内部任务、是否影响全身尚未确认，因此工具不调用它，也没有 stop 命令。
操作者必须事先知道现场已批准的停机方式。

## 失败链路与目前判断（2026-10-29，只读核查）

原始记录来自 10 月 27 日验收会话的终端输出、操作者回答和仍保留的厂商日志，
不是仅引用旧 README。可公开的选摘保存在 `failed_acceptance_20261027.json`。
没有保存 SSH、授权文件或完整会话。

| 环节 | 已找到的证据 | 尚不能证明的事项 |
| --- | --- | --- |
| 请求 | 部署提交 d2f1b804；domain 2、FastDDS 回环；`/mc/uworld_sound_source`，服务节点 `/mc/uworld_sound_follower_node` | 厂商角度的正负、中心、参考系、绝对/相对 |
| 参数 | 原始日志明确 `+10/-10`；代码设置 `with_waist=false`、请求 revolute max_vel/max_acc；worker 留下 prismatic 和 zone 为数值默认 0 | 当时速度/加速度精确值没有从原始请求恢复；0 是否触发默认或阻止杆运动未知，不能从示例值推断 |
| 回执 | 11:50:57 +10，11:50:58 `Motion was interrupted`；11:51:06、11:52:00 +10，11:52:16、11:52:22 -10，之后 success | 第一次中断者是谁；success 是否只代表规划/软件执行结束 |
| 反馈 | 卡片订阅 `/mc/neck_trj_controller/state` 的 `neck_base_joint` actual/desired；最后 -10 请求 initial_desired=-0.24519448 rad，之后 desired=0.30734477、actual=0.30843542 rad | actual 是否为编码器；这些值不能当成 -10°已到位 |
| 物理观察 | 操作者报告完全没转头，已有 head 点头可动 | 水平机构、路由或反馈转接哪里出了问题 |
| 硬件栈 | `walker-motion.motion-1` 的 `rosa_control_node`、`uworld_motion_daemon`；历史启动日志加载 `uworld_driver::UworldDriver` 和 standard_hardware | 插件存在不证明本次字段读自 CAN/编码器；仍可能有缓存/镜像/桥接 |
| 运行配置 | 历史与 10/28 启动日志指定 `/opt/walker/csh2_controller_config/share/csh2_controller_config/config/controllers/uworld_joint_trajectory_controller.yaml`；颈部 joint 顺序 base,a,b,c,d，编号 25,22,24,23,21；与硬件配置 CAN1/CAN0 映射一致 | 通用模板的 25,26,27,28,29 **不是该启动日志指定配置**，不能据此认定映射错误 |
| 控制器状态 | 10/26 与 10/28 日志有 `Starting neck_trj_controller`；配置包含这五个 joints | 当前是否 active/持有关节未获可靠实时查询；ROSA lifecycle 查询 getState 不可达，输出 UNINITIALIZED 是查询失败，不能判定实际 inactive |
| 跨层图 | ROS2 domain 2 目前仅看到 state/joint_states 的订阅者；实际运动栈是 ROSA | ROS2 没发布者不能证明控制器没运行；需检查 ROSA→ROS2 桥，及其反馈字段来源 |
| 覆盖 | 旧驱动发送前检查 wakeup、followup、visual disabled；卡片 TTS info active=null；第一次确有 interrupted | 不排除其他进程、待机行为或轨迹发布者覆盖；历史没有所有 writer 的时间对齐抓包 |
| 异常 | 10/27 11:50:48 四个 neck rod 出现约 0.05–0.06s device timeout，早于第一请求约 9 秒 | 短时警告不是根因证明；该时间窗未找到明确姿态限制错误，不等于不存在 |

当前优先排查方向：**SoundSource → uworld motion → ROSA 控制器 → 硬件/反馈桥**。
不是已确认“模拟器在动”，也不是已确认“电机故障”。之前只检查 use_sim_time=false
不能排除模拟硬件、缓存或目标镜像，这个判断必须纠正。

## 先运行只读采样

以下由操作者执行。本次开发没有复制工具到机器人、部署或运行在线采样。
在已有厂商控制容器中使用现有 ROS 环境，不安装新服务，不启动驱动。
将本目录三个运行文件（head_probe.py、head-dds-local.xml、可选 contract）自行放入可写临时目录，
例如容器内 `/tmp/head-diagnostic/`。本机离线测试文件无需复制。

```bash
source /root/uworld_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/head-diagnostic/head-dds-local.xml
export ROS_DOMAIN_ID=2
python3 /tmp/head-diagnostic/head_probe.py --seconds 12 | tee /tmp/head-observe.jsonl
```

默认只订阅反馈、轨迹并查询图，不创建运动 client。观察记录包括时间、完整五关节目标/actual/error、
joint_states、最多 16 个轨迹点和发布者 GID；最多每 topic 5Hz。其余杆关节不能按弧度解释。
`encoder_provenance=unverified` 和 `arrival_confirmed=false` 始终保留。
`end.feedback_fresh=false` 或发布者为空时，先定位 DDS/ROSA 桥；不要进入发送阶段。
如果 ROS 导入失败，说明当前环境不含这些接口，先报告错误；不要通过启动其他后端来绕过。

## 发送前只需确认这些具体问题

向厂商或掌握当前固件实现的同事询问：

1. SoundSource.degree 是相对当前头、相对躯干还是世界绝对角？正方向和零位是什么？是否只对某姿态/模式有效？
2. with_waist=false 时如何分配 neck base 与四杆运动？四杆 setup 的 0 表示默认、禁止还是无速度？zone=0 的含义？
3. max_vel/max_acc 两组的单位和本机有效小角度输入范围？本工具数值上限不是已验证安全范围。
4. `/mc/uworld_motion_stop` 是否同步终止 SoundSource 内部动作？是否全身停止、停止后保持还是回位？
5. `/mc/neck_trj_controller/state.actual`、`/mc/joint_states` 经哪个 ROSA→ROS2 节点发布？底层读取哪个 CAN/编码器字段，异常时是否回填目标/保留旧值？怎样只读查询 active 及 joint ownership？

将答复来源和确认语义填入 `contract.example.json` 的副本。模板故意不能通过校验，
**没有语义依据时，本阶段只执行上面的只读命令，不填写猜测角度，不发送试探值**。
这是列明的字段缺失，不要求完整 SDK 文档。

## 获得语义依据后的单次验证

现场由操作者确认：无 head/expression/tts、待机或其他自主颈部 writer 占用，
头部静止且有足够活动空间，具备既定停机手段。工具不自动关闭任何行为、不持有平台动作锁；
`--exclusive-neck-confirmed` 是现场确认，不能替代实际资源仲裁。

下面参数是变量，不提供尚未验证的可复制运动数值。依据确认的语义选 **一个非零小角度**，
绝对值最多 5°，两组速度/加速度都需明确填入，且不超过确认值与工具上限中的较小者。
命令只提交一次，不自动回中、反向、循环或重试：

```bash
python3 /tmp/head-diagnostic/head_probe.py --send-once \
  --contract /tmp/head-diagnostic/confirmed-contract.json \
  --exclusive-neck-confirmed --confirm-no-cancel \
  --degree "$VENDOR_DEGREE" --max-vel "$REV_VEL" --max-acc "$REV_ACC" \
  --prismatic-max-vel "$ROD_VEL" --prismatic-max-acc "$ROD_ACC" \
  --seconds 20 | tee /tmp/head-single.jsonl
```

工具先观察 5 秒并展示发布者，只有颈部 state 和 joint_states 都新鲜且有限才发请求。
此条件不是已验证编码器/控制权的证明。若只有通信桥问题，不应放宽此门槛。
`request_attempt` 记录所有请求字段；`receipt` 保存原始 success/status/message，
超时退出码 2，异常/中断也不会重发；正常退出码 0 只表示诊断观察结束，不代表动作成功。
`--seconds` 是整个采样窗口，从启动计时；超时/异常后运动结果未知，不要立即重复调用。

请同步录像或记录：请求时刻、实际头部是否动、方向/幅度、腰部是否保持、请求后是否保持姿态、
是否被覆盖。将 JSONL、物理观察和同一时间窗厂商日志一起反馈。
如果目标变、actual 变但物理仍不动，下一步核对 raw actuator/CAN 与桥接来源；
如果目标也不变，优先查服务拒绝、姿态状态、动作路由/覆盖。不要据反馈收敛判定验收通过。

## 离线测试

```bash
python3 -m unittest discover -s ubtrobot/u1_pro/diagnostics -p 'test_*.py'
python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py'
```

测试的 synthetic contract 和 ROS stubs 只验证门槛、单次提交、超时不重试和清理，
不能证明硬件运动、角度或停止。此阶段不提供新 Try it 镜像，不修改 Dockerfile。
