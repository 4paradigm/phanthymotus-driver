# G1 遥操与数值进程

本说明包含 G1 双臂遥操的使用、运行依赖与实现。驱动全部能力和启动入口见 [G1 Driver 说明](../README.md)。

网页生成可调用 Skill 使用 `g1_teaching` 卡，复用本文的原录制、回放与停止接口；不新建运动控制器。网页宿主需要 Core PR #318，G1 适配本身不修改 Core。流程、设备检查和持久化约束见[通用示教说明](../../../common/teaching/README.md#宇树-g1-的使用流程)，实际网页截图见[G1 界面验收](../../../docs/teaching/g1.md)。

## 使用

`teleop_control` 接收设备 Driver 的输入，在 G1 Driver 内完成相对映射、独立进程 IK 和双臂位置下发。默认注册该卡，市场清单包含 `teleop_control`。随镜像提供原厂 G1_23（每臂 5 关节、固定假手）的完整固定几何 profile；不依赖现场临时标定脚本或用户填写路径。Core 和 ActuCore 无需遥操专用修改。

1. 安装 PICO Driver，在 Canvas 添加 `teleop_device` 和 `teleop_control`，连接命令输出与控制卡输入。
2. 在设备卡齿轮页查看安装和配对说明，连接 PICO。
3. 启动项目。确认机器人和操作者的双臂自然下垂，松开双握把，长按左 X + 右 A 一秒。保持按钮直到基准建立；模型首次准备可能需要数秒。Driver 只在这次明确确认后采集实际姿态建立映射，不在首帧自动标定。
4. 松开按钮，再按下双握把接通。握把按下时的新目标必须通过 IK、输入时效和执行门禁。末端对齐误差仅供显示，不再作为接通条件。
5. 松开任一握把暂停输出并继续预览。再次接通需新一次双握；沿用原映射基准。输入中断或 IK 失败后不自动恢复；设备连接或跟踪空间改变后需再次确认双方下垂姿态并长按 X+A。
6. 需要复位时，松开双握把，长按左 Y + 右 B 一秒。每次按住只触发一次。Driver 取消旧映射和待发目标，复用执行器的权重交还路径：完成五次零权重写入后，通过 SDK 的 no-reply RPC 发送动作99。厂商控制复位轨迹和速度；Driver 不另设关节目标或保存任意姿态作为复位点。
7. 请求写入 DDS 成功后，Driver 立即显示 `waiting_baseline_confirmation`。操作员现场判断复位结果，确认机器人与操作者双臂下垂后，长按 X+A 建立新映射，松键并重新双握。Driver 不用自动静止或姿态判据确认 Reset，也不自动接通。
8. 从 Canvas 停止项目。独立调用 `arm.release` 会取消遥操并关闭输入，之后需从 Canvas 重新启动项目。Driver 重启后也需停止再启动项目恢复绑定，无需删线重连。

Reset 使用 [宇树手臂动作接口](https://support.unitree.com/home/en/G1_developer/arm_action_interface) 的 **Recover Initial Arm Pose（99）**。`RequestReset()` 复用 SDK `_CallNoReply`，返回0只表示本地 DDS 写入成功，不表示服务端接受或物理复位成功。无匹配订阅者、DDS 写入异常等发送失败保留错误，不自动重试。服务端执行失败不会经此 no-reply 调用返回，由操作员现场判断；不自动启停厂商服务。

遥操 Reset 的回执含 `operator_reset=true`、`request_sent=true`；发送阶段完成后 `state=completed`、`authority_released=true`，但 `physical_confirmed=false`、`return_completed=false`。Driver 清理旧租约、待发目标、proof 和 release_requested，关闭旧发送通道。`X+A` 只建立新基准，仍需新一次双握才申请新的运动租约。基准确认不检查静止阈值；反馈新鲜度、跟踪、IK 和接通执行检查继续生效。独立 `arm.release` 与轨迹停止仍保留原完成判据。

原厂99发出后没有可靠取消接口；发送期间发生停止或输入中断时，不得声称动作已经取消。

骨架反馈发布目标频率为30 Hz。ROS 接收等待以最早发布时间为上限，发布延迟后只发送最新快照，不补发积压帧；机器人控制频率保持不变。

PICO 继续使用既有 `left/right.controls.buttons` 的 `available`、`pressed`，不新增 topic。确认键是 `left.x + right.a`，Reset 键是 `left.y + right.b`；已有遥操、暂停、Reset 处理中或错误状态均接收这两组按键；X+A 重建基准，新的 Y+B 按键可重试发送。X+A 会先完成旧通道交权；已经进行的 DDS 写入需结束，不声称撤销已发送请求。同时按两组键不执行。输入超过 300 ms、按键松开或握把按下会中断一秒计时。

| 顶层 state | 顶层 reason | PICO 提示含义 |
|---|---|---|
| `ready` | `waiting_baseline_confirmation` | 双方双臂下垂，松握，长按 X+A 确认 |
| `ready` | `release_buttons_then_regrip` | 松开按钮，再次双握接通 |
| `ready` | `regrip_required` | 松握后再次双握；对齐数值仅供参考 |
| `hold` | `resetting` | 正在交还控制权并发送复位请求；不接通遥操 |
| `active` | `null` | 遥操已接通 |
| `hold` / `fault` | 具体错误码 | 显示失败原因；不报告复位成功 |

`alignment.state/ready` 继续表达真实的几何诊断；PICO 不得用它替代顶层执行状态或修改成恒为 true。
用户不需要填写实例 ID、模型路径、位移比例或 Shadow/Live。这些由框架和 Driver 预设管理；启动项目本身不发送运动目标。支持 FSM 500、801，不自动切换本体模式。

适用范围仅为原厂 G1_23 固定假手，不自动适配 G1_29、灵巧手或改装 TCP。附加碰撞与工作区扫描默认关闭；Driver 不自动停止其他控制程序。`servo`、`servo_eef` 可同时注册，但本实现不提供统一控制器仲裁。

## 实现

`MotionControl` 在 Driver 主进程只处理校验、最新目标和生命周期。`NumericalWorker` 在独立 CPU Python 进程加载 G1_23 模型、FK/IK、碰撞和显示；ROS、控制权、持续执行、保持和停止留在 arm。两个进程之间只有一个在途请求，协调器只保留最新待处理输入。没有GPU依赖或额外遥操构建开关。

### 模块与依赖

| 模块 | 职责 |
|---|---|
| `../teleop_control.py`、`../teleop_bus.py` | 设备输入、相对映射、会话与 ROS 订阅 |
| `../motion_control.py`、`../motion_bus.py` | 数值协调、最新目标与运动卡传输 |
| `worker.py`、`../../../common/motion/worker.py` | G1 求解器入口与独立进程 IPC |
| `kinematics.py`、`ik.py` | 固定关节、FK 与 CasADi IK |
| `profile.py`、`body23_fixed_hand.json` | 机型几何及每轮实测基准 |
| `collision.py`、`workspace.py`、`visualization.py` | 可选空间检查与诊断数据 |
| `../arm_stream.py` | 控制权、位置下发、保持与停止 |

主进程和数值子进程共用系统 `/usr/bin/python3` 及依赖，各自创建模型和 Data。数值依赖由 [镜像配方](../Dockerfile) 固定：NumPy 2.2.6、Pinocchio 4.1.0、CasADi 3.8.1、SciPy 1.15.3。构建步骤见 [G1 构建说明](../README.md#构建)。

七个碰撞网格在构建时下载，构建和运行时均核验 SHA256。网格不进入 Git；许可证与哈希清单位于 `models/collision/`。测试专用的未标定数据位于 `../tests/fixtures/calibration.json`，不随业务镜像安装。

### 输入与状态

| 方向 | Topic | 格式 |
|---|---|---|
| 设备 → 控制卡 | `/teleop/command` | `data/teleop-cmd`，JSON schema `motus.teleop.command/1` |
| 控制卡 → Canvas 监控与 PICO Driver | `/teleop/state` | `data/teleop-state` |

输入来源由 Canvas 连线决定。控制卡直接订阅传入的 `input_topic`，不限定 topic 路径、不按首帧绑定来源实例，也不额外认证消息中的实例或设备 ID。格式、时效、序号和代次检查仍保留；设备或跟踪空间变化后，需要松握并长按 X+A 明确确认后重建坐标基准。

设备输入为左右控制器位姿、握把、跟踪状态、身份、代次、序号与时效。PICO Driver 订阅同机 `/teleop/state`，把 `motus.teleop.feedback/1` 转发到头显。G1 提供 `visualization`（实测、IK、躯干线框）和 `alignment`（左右末端位置/朝向误差、阈值、状态、`ready`）。两端使用新版协议，不新增 topic 或协商。URDF、关节顺序、数值求解、对齐判定与执行均在 G1 Driver 内。

对齐比较实测末端和可执行 IK 末端，`error_reference=ik`。求解失败时状态为 `unreachable`，误差参考请求目标（`error_reference=requested_target`），不能使能；求解残差另列 `target_residual`。实测反馈超过 100 ms、输入/目标超过 300 ms 或代次改变时，撤销就绪与旧目标。3 cm / 15° 仅为诊断显示阈值，不用于使能。

### 机型 profile 与每轮基准

`motion/body23_fixed_hand.json` 固定关节映射、URDF SHA、双掌 TCP（腕轴前方 0.2 m）和手柄局部变换。该文件不包含某次现场腰腿角度，也不填写 acceptance 通过标记。每次初始化映射时，从同一份新鲜 LowState 采集双臂和 13 个腰腿关节，为数值工作进程生成本轮临时 profile；工作进程重启沿用同一份基准，新的遥操会话重新采样。缺失、过期或非有限实测值会明确报告，不用零值猜测机器人姿态。松握重握不重新标定。

适用范围是原厂 G1_23 固定假手；不宣称自动适配 G1_29、灵巧手或改装 TCP。附加碰撞检查仍关闭，旧现场预览空间盒未纳入默认 profile，不能将其当作碰撞验收结果。

卡片注册、info、默认启用及空闲 stop 不申请运动权、不创建 SDK 命令通道。`servo`、`servo_eef` 可以同时注册；本实现不新增统一控制器仲裁，也不在装载时因其他卡片 enabled 而拒绝遥操。旧会话/过期指令隔离与本遥操的停止取消仍保留。

### 执行与平滑

每个有效目标仅下发一次。以后端最后成功下发的位置为起点，做 120 ms 一阶指数平滑，再将各关节增量裁剪到 ±1 rad/s × dt；dt 最大 50 ms，断帧或重握不积累追赶额度。仅处理最新目标，不排队补发历史动作。这限制的是位置参考的变化率，不是实测电机速度的硬保证。

遥操肩肘增益为 kp=80/kd=3，腕部为 40/1.5；普通 servo 增益不变，不增加重力补偿。IK 失败丢弃失败结果及其滤波历史，并要求松握和重新双握后续接。松握后的执行器恢复不新增静止阈值；`continuation_ready` 表示可以继续接收目标，与物理保持确认 `hold_confirmed` 区分。

输入有效性、反馈时效与 URDF 关节限位仍校验；附加软件碰撞/工作区扫描默认关闭，不能据此认为任意姿态均无碰撞。原始电机遥测保留，厂商保护不修改。Driver 不自动停止其他控制程序。

### 状态与恢复

Canvas 监控的 `input_status` 区分 `waiting_binding`、`stopped`、`waiting_input`、`input_stale`、`transport_error` 和 `fresh`。`transport` 提供接收/接受序号、拒绝原因、订阅代次和执行器心跳；MCP 在线不等于 DDS 接收正常。

尚未建立基准时，启动无帧、输入过期或跟踪丢失会暂时进入 hold；新鲜且跟踪有效的输入恢复后，显示 `waiting_baseline_confirmation`，仍需 X+A 明确确认。SDK/复位失败及 `reset_interrupted` 保持原错误；后续输入中断和恢复不能清除这些错误。

订阅创建、读取和销毁由同一专用 ROS 线程执行；设备身份补全不重建同 topic 订阅。IK 子进程故障分别报告 `ik_worker_timeout`、`ik_worker_exited` 或 `ik_worker_unavailable`，原始异常保留在日志中。

空闲且从未接管 SDK 时，重复停止返回 no-op；不创建释放任务，也不声称已验证物理停止。未知的厂商动作结果仍报告未知，不通过重复 stop 清除。Driver 不持久化或自动恢复运动租约。

“以后端最后成功下发的位置续接”的设计参考 **@jsmy-CTH** 的 [PR #322](https://github.com/4paradigm/phanthymotus-driver/pull/322)。120 ms 平滑、1 rad/s 限幅及 50 ms 时间上限为本次 G1 适配参数。其他来源见 [来源与迁移边界](#来源与迁移边界)。

## 轨迹接口（供示教适配器调用）

录制、文件校验和回放属于 `teleop_control`；下游 #402 负责页面、试运行流程和 Skill。没有新增示教卡或 Core 改动。接口由现有 loopback MCP 提供，参数均放在 `arguments` 中。

| action | 参数 | 结果 |
|---|---|---|
| `info` | 无 | `input_fresh`、`mapped`、`can_record`、`recording`、`replay_active`、`last_action` |
| `record_start` | 无 | 新鲜、已映射的 live 遥操中开始录制；返回文件和服务端开始时间 |
| `record_stop` | 无 | 停止录制并等待最多 2 秒落盘；`complete`、`stopping`、`error` 分别说明结果 |
| `recordings` | 无 | 文件名、大小、完整文件/临时文件；列表不代替 `inspect` 校验 |
| `inspect` | `filename` | 校验后的 `filename/sha256/binding/joint_names/duration_seconds/frames/preview` |
| `start_difference` | `filename` | `ready/target_q/current_q/joint_delta_rad` |
| `play` | `filename` | 自动检查后开始回放，返回异步 `action_id` |
| `action_status` | `action_id` | 最近一次回放的 `running/completed/cancelled/error` 和结果 |
| `stop` | 无 | 立即请求取消遥操/回放，再等待原执行器交权；失败不报告停止成功 |

`record_stop` **不停止遥操**。适配器须先确认 `complete=true`，再调用 `stop`，分别检查文件结果与交权结果。仍在落盘时用 `info.recording` 轮询。指令超时、输入断流、IK 或 SDK 执行故障仅暂停轨迹采样，不终止录制。暂停帧保留最后一次成功下发的目标及当时的反馈时间；重复故障不重复写暂停帧。同一映射下恢复成功下发后，继续写入同一文件，保留暂停时长。运动控制的停止和恢复判据保持不变。显式停止遥操、Reset 或交还控制权会正常结束并保存已有录制；之后调用 record_stop 返回同一文件结果。Reset 前先结束采样，原厂复位动作不计入轨迹。映射改变仍结束当前录制。队列溢出、写盘失败、格式或限位校验失败仍使文件无效，不把临时文件列为可回放资产。

录制仅保存 SDK 成功接受的最终关节参考，已包含遥操平滑与限速。录制与载入不按文件帧间隔重复判定速度，避免时间戳抖动或暂停帧使有效输出被误拒绝。同时保存实测位置/速度、反馈单调时间、源输入序号、SDK 序号、映射代次及执行会话 ID。格式为 `motus.g1.trajectory/1` JSONL，10 维关节顺序在头部列出，角度 rad、速度 rad/s、时间 ns，无夹爪字段。后台队列最多 256 帧；单文件最多 64 MiB、120000 帧、10 分钟。溢出或写盘失败不会阻塞运动线程，但该录制无效。

文件位于 `teleop_control.data_dir/recordings`，默认 `/opt/phanthy-motus/data/g1/recordings`。部署模板已挂载持久化数据目录及只读 `/etc/machine-id:/run/g1-machine-id`。非模板部署可设置稳定且设备唯一的内部 `trajectory_device_id`；未提供设备身份时，录制明确失败。不要给多台设备配置相同身份。

文件绑定设备身份摘要、模型哈希、固定几何、关节顺序及录制时的腰腿基准。载入时核对设备、模型和关节顺序；腰腿基准仅保留为录制元数据，不作为回放门槛。更换设备或模型后不能直接复用文件。未完成文件、符号链接、越界文件名、摘要错误、非法时间或超限关节均拒绝。`sha256` 是完整文件摘要。

### 适配调用示例

在 Canvas 已启动遥操且 `info.can_record=true` 时：

```json
{"name":"teleop_control","arguments":{"action":"record_start"}}
{"name":"teleop_control","arguments":{"action":"record_stop"}}
{"name":"teleop_control","arguments":{"action":"stop"}}
{"name":"teleop_control","arguments":{"action":"inspect","filename":"g1-<id>.jsonl"}}
{"name":"teleop_control","arguments":{"action":"start_difference","filename":"g1-<id>.jsonl"}}
```

用户先恢复现场物体并确认运动空间无障碍。起点位置差值仅供显示，不设置位置容差门禁；`start_difference.ready` 只表示当前实测速度不超过0.02 rad/s，该显示值不作为位置模式接管门槛。确认执行后，Driver 先以每关节不超过0.1 rad/s的参考变化速度移动到录制首帧，再开始原录制时间轴；任意非零差值均通过限速过渡发送首帧，不直接跳到目标。过渡使用原执行器的位置发送与停止逻辑，不重新运行数值计算。首帧目标下发完成后直接进入原时间轴，不另等实测位置进入容差或自动确认物理到位。`inspect.preview` 最多 181 点，包含 `t/kind/target_q/feedback_q`，单位保持 rad。下游负责转换显示单位，不套用 PiPER 的 12 关节或夹爪字段。

```json
{"name":"teleop_control","arguments":{"action":"play","filename":"g1-<id>.jsonl"}}
{"name":"teleop_control","arguments":{"action":"action_status","action_id":"<执行返回值>"}}
```

`play` 只需文件名，调用即表示执行回放，不再填写布尔确认项或复制限时预检 ID。Driver 每次执行均重新读取并校验文件、设备、模型、关节顺序和控制权；不重复检查录制时的腿部和腰部基准。未开始遥操或仅等待基准确认时，Driver 自动关闭输入等待后回放；已有操作员会话或控制租约时仍拒绝抢占。查看文件用 `inspect`，查看起点差异用 `start_difference`，两者不触发运动。旧调用若明确传入 `execute=false`，返回 `use_inspect_before_replay`，不会意外执行动作。成功启动返回 `status=running` 和 `action_id`；`action_status` 的 `phase` 先为 `moving_to_start`，首帧目标下发完成后为 `replaying`。仅最后一项结果保留在 `info.last_action`，Driver 重启后不恢复动作。失败返回 `state=error/code/error`。

回放与遥操互斥。回放按原时间发送已经录制的关节参考，不重新运行 IK、数值包围区间或几何碰撞计算，也不再施加 120 ms 低通滤波。回放前停止数值协调进程，直接使用执行器的关节位置入口；控制权、反馈、关节限位、写入失败处置、保持和停止逻辑继续共用。已录制目标不再重复滤波或限速。发生调度抖动时，只取最新到期目标，不补发积压目标；暂停事件仍保留。发送值直接使用文件中的关节参考，时间轴不延长。暂停事件停止输出，恢复事件沿原时间继续。执行心跳超过 100 ms 或反馈失效时停止。回放不需要 PICO 持续跟踪，也不会消费手柄指令。

原时间轴结束后，完成原执行器停止交权才报告 `completed`；不增加末帧位置误差或速度阈值，不延长时间轴补发收敛目标。此状态表示指令发送与交权完成，不表示现场动作形态已获人工验收。取消为 `cancelled`；停止未确认仍为 `error`。ACP 事件 `tool=teleop_control` 与查询接口使用相同 `action_id`，只发一次终态通知。原有 action 99 未确认等控制故障不会由回放清除。

## 验证

### 离线验证

在仓库根目录、具备 G1 运行依赖和 pytest 的环境中执行：

```sh
python3 -m pytest -q -rs tests/test_teleop_contract.py tests/test_g1_servo_eef.py unitree/g1/tests
```

`test_motion_control_numeric.py` 使用真实模型、网格和 IK 子进程；缺少数值依赖时会跳过，不能把跳过记作通过。协议、执行器与线程用例使用硬件替身；线程用例还使用合成求解器测试超时、重启和旧代次隔离。这些测试不验证机器人动作或物理停止。

### ROS 生命周期验证

[手动验证脚本](../tests/validate_ros_lifecycle.py) 使用真实 ROS 节点和接收线程，检查订阅切换、线程归属、InvalidHandle 恢复及错误状态。控制对象使用替身，不导入硬件 SDK，不验证机器人动作。脚本不由默认 pytest 收集。

在 Linux ARM64 测试机的仓库根目录执行。将 `G1_TEST_IMAGE` 设置为已有的 G1 Driver 镜像；容器使用隔离网络、只读文件系统和只读源码，不挂载设备：

```sh
docker run --rm --network none --read-only --tmpfs /tmp \
  -v "$PWD:/src:ro" -w /src \
  -e G1_TELEOP_ISOLATED_TEST=1 -e PYTHONDONTWRITEBYTECODE=1 \
  -e PYTHONPATH=/src -e ROS_LOG_DIR=/tmp/ros-log \
  --entrypoint /bin/bash "$G1_TEST_IMAGE" -lc \
  'source /opt/ros/humble/setup.bash && python3 unitree/g1/tests/validate_ros_lifecycle.py'
```

成功时输出 `ROS ISOLATED PASS`。环境变量仅确认操作者已选择隔离环境，不会自动检查容器网络或设备挂载。真实模型验证仍使用 `test_motion_control_numeric.py`。

## 来源与迁移边界

数值实现从主仓 `30280160` 后的四卡契约工作树所含既有 G1 实现按模块选择性迁移，来源文件及 SHA256 见 `source-manifest.json`。原始算法和 G1 模型来自 Driver PR #152（`0adb64cb63e358244f4e02c0a7823523824f17a8`），Unitree `xr_teleoperate` revision `845b25a32f7febedf220e830952a7134897adb9d`，Copyright 2025 HangZhou YuShu TECHNOLOGY CO.,LTD，Apache-2.0。完整许可证沿用仓库 LICENSE。

保留 G1_23 模型、锁定关节、手掌变换、PR152 五自由度加权目标和保守碰撞检查；删除 IK 每输入帧的执行限幅和数值进程内无用途的力矩生成，执行 arm 对最终 q 单独 RNEA。未迁移 PICO SDK、采集、相对映射、硬件发布或控制权逻辑。源码迁移不等于冻结基线复现或北京真机通过。

碰撞网格来自 Unitree `xr_teleoperate` 提交 `817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b` 的 `assets/g1/meshes`。`models/collision/sha256.json` 与 LICENSE 保留；构建需固定源下载并核验全部哈希，运行时不下载。网格缺失时明确拒绝初始化，不跳过几何验证。

依赖要求带 CasADi 支持的 Pinocchio 4.1.0、CasADi 3.8.1、NumPy 2.2.6 和 SciPy 1.15.3。Dockerfile 使用系统 Python 安装带 CasADi 绑定的预编译 Pinocchio wheel；普通 PyPI pin wheel 不提供该绑定，不能据其安装成功声称数值通过。目标 Linux ARM64 构建与真实求解需分别验证。

`fetch_assets.py` 选择性迁移主仓 `deploy/fetch_g1_collision.py`，只调整默认目录和宿主说明；构建通过此入口固定下载，未在数值控制循环引入网络。
