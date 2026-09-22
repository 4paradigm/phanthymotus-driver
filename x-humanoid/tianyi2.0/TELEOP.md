# 天轶连续遥操执行契约

`teleop_executor` 接收普通 ActuCore 内 `teleop` 卡片产生的机器人目标。PICO 配对、控制器映射、IK、可达性和碰撞检查、显示、开始/结束操作及收臂轨迹均由 ActuCore 负责。Driver 负责执行权、最新目标、限位限速、反馈和停止确认，不解析 PICO 协议或自动回零。

## 启用与用户入口

功能默认关闭。顶层 `teleop.enabled: true` 才加载执行器；`live_enabled` 默认 false，不迁移现有设备配置。完整配置准备见[部署说明](deploy/TELEOP_RUNBOOK.md)。

用户通过 Canvas 的 ActuCore `teleop` 卡片配置连接、模式和映射，并通过卡片或 PICO 的操作控件开始、结束或立即停止。Driver 的 `teleop_executor` 是卡片使用的内部接口，不应要求操作者逐帧调用它。普通 ActuCore 提供同一 MCP 服务；不另部署独立遥操 ActuCore。

在 Canvas 中，将 ActuCore `teleop` 的 `control/teleop` 输出连接到对应机器人的 `teleop_executor` 输入。Driver 工具元数据顶层及 `info` 返回相同的 `x-teleop-target` 描述符：

```json
{
  "protocol_version": 1,
  "robot_profile": "tianyi2",
  "namespace": "<Driver 实际 namespace>",
  "command_topic": "/<Driver 实际 namespace>/motion/teleop/command",
  "feedback_topic": "/<Driver 实际 namespace>/motion/teleop/feedback"
}
```

这些路径来自执行器实际使用的命名空间，和本机 DDS 子进程创建的订阅、发布一致；命令输入格式为 `control/teleop`，反馈输出仍为 `data/json`。Agent Core 从所连卡片的 MCP 注册项与描述符解析目标，不通过固定端口或手工拼接路径猜测机器人。描述符版本是 Canvas 绑定契约版本，不改变下述 `motus.motion-target.v1` 连续目标协议。

开启智能控制后的遥操准备由 ActuCore 负责；PICO 开始、结束以及关闭智能控制时的收臂编排也由 ActuCore 处理。这个描述符本身不获取执行权、不启动运动，也没有为 Driver 新增项目级开始、结束或自动回零动作；原有租约、反馈与停止检查保持不变。

双臂专用配置必须在标定中显式设置 `hands_enabled: false`。此时既不要求手部开合端点，也不创建手部输出发布器；即使报文携带非零 hands，厂商写入路径仍屏蔽手部命令。省略此字段表示启用手部，非布尔值拒绝。手部启用时，使用已标定端点和现有 HandPlugin 转换，暂停不自动张手。

## 管理面与数据面

管理面为同机回环 MCP，不接受浏览器 Origin。租约密钥只从管理响应返回，不写入状态、追踪记录或 DDS。

| 操作 | 行为 |
|---|---|
| `info` | 返回状态、反馈、能力与诊断；不创建运动发布器 |
| `start` | 启动只读反馈、看门狗和本机消息总线；不接管、不运动 |
| `prepare_first_acceptance` | 检查前提并建立 60 秒进程内首次试验窗口；不接管 |
| `prepare_operator_session` | 显式建立持续操作准备状态；默认关闭，不接管 |
| `claim` | 检查反馈、标定和占用后取得租约，返回 boot_id/session_id/secret |
| `pause` | 清除目标并请求保持，保留执行权，关闭自动续接 |
| `recoverable_hold` | 租约所有者请求短暂 IK 保持；确认保持后可在同会话继续 |
| `resume` | 仅在可恢复、保持已确认且反馈有效时换会话重新使能 |
| `release` / `stop` | 请求保持；只有后续反馈确认停止才释放执行权 |
| `end_operator_session` | 无租约时撤销持续操作准备状态 |
| `trace_start` / `trace_stop` | 空闲时开启/关闭有界内存诊断记录 |

连续目标在 `/{namespace}/motion/teleop/command`，反馈在同前缀 `/feedback`；类型均为 `std_msgs/String`，ROS 2 domain 42、BEST_EFFORT、KEEP_LAST depth 1。专用子进程校验本机 DDS 隔离文件；厂商 domain 0 的发布只在 Driver 内进行。DDS 后的本地 datagram 队列也仅处理最新包；单周期最多取 128 包，仍无法排空则保持，不应用不确定的中间目标。

报文协议为 `motus.motion-target.v1`：

| 字段 | 约束 |
|---|---|
| `boot_id`, `session_id` | 当前 Driver 启动代次和租约 |
| `seq` | 当前会话内严格递增整数 |
| `generated_ns` | 与 Driver 同机、同启动周期的单调时钟 |
| `valid_for_ms` | 原始有效期 1–100 ms，不能通过重发延长 |
| `q` | 14 个有限数值，rad；左电机 11–17，右电机 21–27 |
| `hands` | 两个有限开合量，范围 [0, 1] |
| `mac` | 对协议字段规范 JSON 的 HMAC-SHA256 签名 |

每臂关节顺序为 shoulder_pitch、shoulder_roll、shoulder_yaw、elbow_pitch、wrist_yaw、wrist_pitch、wrist_roll。非有限数值、越限、过期、重复或乱序目标拒绝执行。旧 boot/session 报文先忽略并记录，不能扰动当前会话；当前会话签名错误仍进入保持。

MCP claim/resume 支持可选的 32 位小写十六进制 `request_id` 和 `request_valid_until_ns`（最长 300 ms）。同一身份与期限的重复请求只返回原结果，不重复接管或旋转租约。取消只作用于该请求创建的租约；取消先到时，迟到请求不能重新接管。管理超时本身不代表已取消或已停止。

## 位置控制与恢复

执行循环默认 50 Hz，只使用位置接口。每周期从上一下发目标推进，步长最多为配置速度乘 20 ms；目标领先实测的上界为配置速度乘 200 ms。速度默认 0.2 rad/s，由标定设置且不超过 URDF 限制与厂商接口上限 1.5 rad/s。该领先界不是本体急停承诺：Driver 崩溃后本体剩余目标行为必须单独验收。

ActuCore 在发送前检查实测/上一目标到新目标的运动段和碰撞；Driver 不重复求解 IK。不可达时保留最后有效输出附近的保持位置，恢复只使用新的有效目标，不排队补发旧输入。

状态明确区分 `idle / ready / active / hold / fault`。`applied_sequence` 只表示厂商发布函数已返回，不代表实际到位；实际运动必须读取反馈 q/dq。

- 暂停或断流清除旧目标，并在反馈新鲜时发送实测保持位置；冻结手部目标。
- 保持确认使用后续新反馈的位置与速度，不把停止发布当成停止确认。短暂反馈缺失期间不能使用旧位置伪造保持命令。
- HOLD 中只允许一次有界沉降重保持：偏差不超过速度乘 100 ms、速度不超过 0.02 rad/s，且不同新鲜反馈跨越 100 ms 的位移不超过 0.002 rad。重发后仍需新反馈满足原确认条件；首次保持后的总确认期限最多 2 秒，不能不断追逐漂移。
- 普通 pause/release、非法命令及真实故障关闭续接。握把松开后由 ActuCore 重新使能并用实测姿态建立相对基准。
- `recoverable_hold` 保留租约，确认保持后允许首个新的有效 IK 目标在同会话继续；不经过重复 pause/resume，不回放失败目标。ActuCore 通过 `timing_policy.recoverable_hold` 识别能力。
- `command_timeout`、合法但途中到期的 `command_expired` 和短暂反馈过期可在 `continuation_timeout_ms` 内同会话续接。默认 300 ms，允许 100–1000 ms；必须保持已确认、反馈和新目标有效。过期包不刷新窗口。窗口外必须显式恢复。
- 必需反馈在 100 ms 内才允许运动；持续过期超过 `feedback_fault_timeout_ms`（默认 300 ms，可配 100–1000 ms）锁存故障。缺失/未来时间戳、非有限反馈、急停、断电和关节故障不等待宽限。

停止失败时控制权锁存，不通过重启掩盖未知结果。停止检查不要求头腰继续精确匹配旧标定基准，但新运动仍检查本次会话固定体位。

## 标定、会话准备与共享执行权

标定文件校验模型哈希、14 关节映射、位置/速度限制及可选手部端点。普通 Live 所需 acceptance 包含模型、工作空间、PICO、外部控制排查、停止与进程崩溃验收记录；不允许伪填。电源订阅到达新鲜度与硬件实际采样新鲜度不同，后者不能由接收时间代替。

首次验收使用独立 first_acceptance 记录，包括 model_verified、workspace_verified、pico_verified、external_control_excluded 及 operator/date/evidence_sha256；只支持双臂。显式准备建立 60 秒窗口，resume 不延长，进程重启即丢弃，到期保持而不自动释放。

持续现场操作需另显式启用 `operator_session_enabled`，并调用 `prepare_operator_session`。它沿用首次验收前提，授予内存准备状态而不伪造普通 acceptance；不采用 60 秒试验窗口。release/stop、生命周期停止或进程重启撤销准备状态。准备失败不能沿用旧许可。此开关只用于已获现场授权的操作方式，不是完整设备验收证明。

每次显式准备、或普通 Live 新 claim 时，从新鲜静止反馈采集头/腰/腿固定关节基准；PICO 位姿不参与。接管中相对该基准的变化上限仍为 0.02 rad，暂停/resume 不重标定。长期 URDF 与几何记录不修改。

原动作、轨迹、手势、手部、头腰、导航、底盘与 servo 写入路径共享执行检查。已有异步动作未结束时拒绝 claim，不静默抢占；遥操持权期间拒绝竞争动作。取消超时的旧动作继续表示忙，不能错误清为空闲。

## 反馈、追踪与证据边界

遥操安全反馈使用独立 SingleThreadedExecutor，分批读取后让出调度；主 Driver domain-0 执行器也有有界让出。QoS 保持最新状态，不通过放宽新鲜度阈值掩盖调度延迟。线程失败记录 `feedback_executor_error`；销毁节点前确认线程退出。

反馈包括实测 q/dq、各来源年龄、执行权、保持/停止确认、首个故障、最近拒绝原因、续接次数和看门狗分段耗时。`last_vendor_command` 记录发布时刻、目标、速度与电机 ID，只证明发布返回。显式 trace 使用有界内存环，包含序号和目标但不包含 secret/MAC；异步消费者必须保留事件缺号或队列丢失证据。

2026-09-22 现场用户已确认双臂遥操体验可用；同轮反馈记录两次结束收臂并释放执行权。该结果不等于手部、所有故障注入、长期运行或 Driver 崩溃后的本体行为已全部验收。本次提交整理仅离线验证，未访问或操作展厅机器人。
