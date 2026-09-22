# 天轶 motion_control 与 arm 连续入口

Canvas 接线为 `teleop → motion_control → arm`。普通 ActuCore 的 teleop 只处理
设备连接、双握把和相对末端映射；本 Driver 持有天轶模型、IK、碰撞检查、反馈显示
和显式收臂。模型与原厂 URDF 的来源见 [NOTICE](tianyi_motion/NOTICE.md)。

开启智能控制只启动和校验链路，不获取执行权。PICO 开始进入准备状态，双握把
使能才 claim；松开任一握把保持，再握住使用实测末端重新建立相对基准。结束并
收臂或 Canvas 关闭项目调用 finish，Driver 独立完成收臂和释放；头显离线不影响
已经受理的收臂。立即停止取消收臂并保持，不将保持当作自然下垂。

## 配置和图连接

`motion_control.enabled` 默认 false。启用时要求 arm 插件存在，配置的
`calibration_path` 同时供 motion_control 和共享执行器读取；文件必须是
`motus.tianyi-calibration.v1`，包含可核验 URDF、14 关节顺序、手掌变换和碰撞边界。
本阶段要求 `hands_enabled: false`。零位不是通用厂商假设，只有在该标定关节限位
和碰撞空间内才允许收臂；所有验收依据继续由执行器检查。

卡片配置表单提供 `calibration_path` 和 `joint_velocity_rad_s`（默认 1 rad/s，
仍受 URDF 和 1.5 rad/s 已有上限约束）。路径指向已挂载 Driver 文件，URDF、TCP
和碰撞包络随该文件导入，UI 不编辑 JSON 内容。Core 按现有 configSchema 机制
持久化参数并调用 `config`；Driver 仅空闲时接受，完整验证后原子替换，失败保留
旧配置。成功后清空旧实测基准，需下一次 `calibrate`，`info.config` 可读回结果。

| 卡片 | 输入端口 | 输出端口 |
|---|---|---|
| ActuCore teleop | PICO 会话 | control/eef 双末端 |
| Driver motion_control | targets: control/eef | joints: control/joint；feedback: data/json |
| Driver arm | targets: control/joint | 不增加反馈 topic，内部共享执行状态 |

MCP `info` 和工具定义给出真实 namespace/topic：motion_control 使用
`x-teleop-target`（protocol_version 2）与 `x-motion-control`；arm 使用
`x-control-target`。后者包含 protocol_version、robot_profile、namespace、
command_topic、feedback_topic 和 resources（arm_l、arm_r）。`control_interface`
描述本卡消费的动作空间，`motion_control.info.control_interfaces.joints` 描述
输出关节空间；它们不与执行端地址描述符混用。

`motion_control.start` 可接收 input_topic、instance_id、control_interface、
control_interfaces 和 execution_binding。接口必须匹配本机 arm 的模式、14 维
关节序、单位、分组及目标 namespace/topic；不允许另指一台机器。此步骤无 claim。

## 连续命令

同机 ROS 2 domain 42，std_msgs/String JSON，BEST_EFFORT / VOLATILE / depth 1：

- `/<namespace>/motion/control/command`：mode=eef_pose。
- `/<namespace>/motion/arm/command`：mode=joint_position。
- `/<namespace>/motion/teleop/feedback`：复用执行反馈。

厂商 domain 0 与 loopback DDS 仍用独立上下文/进程。数值线程只发布 arm topic，
没有第二条厂商消息路径；arm 收到目标后进入原 MotionGate，50 Hz 看门狗负责
真正下发。IK、FK 和显示不会在看门狗内运行。

所有 v2 命令只允许以下字段，缺失、多余、重复 JSON 键都拒绝：

| 字段 | 规则 |
|---|---|
| schema | `motus.control/2` |
| boot_id、session_id | 当前启动与会话；Preview 使用不同会话和密钥 |
| seq、source_seq、mapping_epoch | 非负整数、小于 2^53；seq 严格递增，epoch 不倒退 |
| generated_ns、valid_until_ns | 同机单调时钟；不能用头显时钟。EEF 有效期≤300 ms；关节≤100 ms |
| mode、dof | eef_pose 或 joint_position，dof=14 |
| values | 有限数字。EEF 为左 xyz+xyzw，再右 xyz+xyzw，米及单位四元数；joint 为左7+右7 rad |
| model_version | 配置 URDF 的 SHA256 |
| calibration_version | 标定文件原始字节 SHA256 |
| frame | 标定 torso_frame；不能隐式接受 world 或另一胸部坐标系 |
| mac | 除 mac 外全部字段的排序、紧凑、禁 NaN JSON，以会话 hex secret 计算 HMAC-SHA256 |

IK 每次最多 100 ms；关节报文生成时间为求解完成时，期限取“当前+100 ms”与
源 EEF 期限的较小值，保留 source_seq 和 mapping_epoch。诊断同时记录源时间，
不能刷新过期输入或延长其寿命。DDS 和进程内队列均只保留每段最新目标。

现有 v1 gate 在最后一段保持原协议、签名和实测停止语义；v2 校验后作内部转换，
舍去不足 1 ms 的期限余数，不向上取整延长期限。v1 入口、旧动作和 servo 继续
共用这一个 gate，Preview 密钥不能调用 arm 流。

## 管理和反馈

管理仅通过 loopback MCP，不允许浏览器 Origin。动作如下：

| 动作 | 返回及副作用 |
|---|---|
| info | 实际执行状态和接口描述符 |
| start | 只准备总线与数值线程，返回 state=ready；execution_state 单独保留执行器状态，不 claim |
| config | 空闲时验证并导入标定路径和速度；不驱动硬件，不改写标定文件 |
| calibrate | 使用 Driver 本地文件；返回 calibrated、model_version、calibration_version、frame、effector_ids、eef_snapshot；持有执行权时拒绝 |
| prepare_preview | 返回 boot_id/session_id/secret/preview:true 与版本；不 claim、不发布厂商命令 |
| prepare_operator_session | 复用既有实测体位与验收准备，不 claim |
| claim / resume | 获取/更新共享执行租约，沿用 request_id/request_valid_until_ns 幂等管理 |
| pause / recoverable_hold | 保持；求解失败后仅新有效帧可在实测保持确认后续接 |
| release | 取消未完成输入/收臂，等待实际保持后释放；保留卡片工作线程 |
| stop | 按相同执行权规则请求保持及释放，并取消、等待本卡求解/收臂线程退出；共享反馈与看门狗继续运行 |
| end_operator_session | 无执行权后清理准备状态 |
| finish | 异步受理收臂，返回 operation_id、state、return_completed、authority_released |
| finish_status | 可按 operation_id 查询；失败有 error/code，成功必须真实归零稳定并释放 |

框架在 Preview 下调用无密钥 stop/end_operator_session 只清理零输出会话；Live
持有执行权时仍须正确密钥，不能借框架入口抢占。持续 finish 请求返回同一个
任务；失败后显式重试才重新进入收臂，失败状态不假装完成。

`stop` 后拒绝新连续输入，必须再次 `start` 或显式准备会话才能恢复卡片。
线程未能退出时返回 `motion_control_thread_stop_unconfirmed`，不允许静默重启。
软件线程退出不代表硬件已停止：实际控制权仍持有或停止未确认时继续返回 hold/fault，
不伪报 idle；普通 pause/recoverable_hold 不关闭卡片。非法 JSON、非对象报文或
错误的嵌套路由类型被拒绝，接收循环继续处理后续新帧。

反馈保留既有实测位置、速度、时效、所有权、停止确认与诊断，追加：

- control_interface/control_interfaces：本次动作空间和模型版本。
- eef_snapshot：poses=[左 pose7,右 pose7]、frame、state_seq、model_version、calibration_version、monotonic_ns。时间来自实际 arm 样本，不因 FK 重算而刷新。
- visualization：现有 `motus.tianyi-visualization.v1`，实测、当前 IK、历史保持模型、目标、边框和躯干参考线；无新反馈 topic。
- control_decision：输入序号、源序号、epoch、求解耗时和拒绝原因；finish 为独立进度。

Preview 的 ownership_held/output_active 恒为 false；hold_confirmed 仅指零输出
预览已暂停。若旧执行接口取得真实控制权，Preview 立即失效且不覆盖真实所有权。

## 验证边界

`tests/test_motion_control.py` 与 `tests/test_motion_control_lifecycle.py` 使用真实求解和门禁、独立有限速度 plant；实测 q
不会被目标发布直接覆盖。新增测试不连接机器人。实际 ARM64 构建、DDS
跨进程延迟、Canvas+PICO 使用与新架构的真机跟随/收臂需单独验收，不沿用旧
架构物理证据冒充已通过。跟随误差如实记录，不增加通过门槛或累计行程上限。
