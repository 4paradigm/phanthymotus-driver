# 北京 G1 两 Driver 遥操实施计划

2026-09-23 最新范围，替代此前四卡及设备反向反馈方案。采用已获用户批准的旧 G1 执行与模型草稿，原工作树不动；来源清单和历史结果保留在 docs/validation。当前只部署北京 G1，不操作天轶，Core/ActuCore 零改动。

## 卡片契约

- teleop_device 仅输出 `/teleop/command`，format `data/teleop-cmd`、schema `motus.teleop.command/1`。JSON 保留设备/实例身份、连接与空间代次、序号和时效；当前只支持同域单设备源。topic 不含品牌、日期和 Canvas ID。
- teleop_control 订阅连线指定输入，固定 topic 从首个合法新鲜输入锁定 source instance，运行中拒绝其他实例。旧命名输入兼容读取，但新生产端只发布固定 topic。
- `/teleop/state` 是控制卡的监控状态（data/teleop-state），不是设备反馈协议；设备不订阅它，不存在 control → device 自动反馈线。监控包含输入、IK、执行和停止原因。
- PICO 无开始/结束/停止按钮，无机器人状态或模型显示，仅显示配对连接及按住双握把提示。Canvas 启动控制卡后，首个跟踪有效且双握把松开的输入建立初始基准；此准备不 claim、不发动作。双握把使能，松任一保持，重握沿用原基准。真实空间改变需松开双握把重新建立基准，不能在持握时自动接管。
- Canvas 停止停止遥操；收臂复用现有 arm release/action99，不另造硬件回零功能。

## 配置简化

两卡齿轮保留单值 `usage_guide=无需配置`，仅使普通 Core 显示齿轮；卡片正面不暴露 usage_guide 或 instance_id。实例由框架传入，模型/比例/模式是 Driver 预设；旧 Canvas 模式不能覆盖部署预设。生产 Live，内部 Shadow 保留用于测试；项目启动不自动运动。

PICO 配对无需密码或管理登录。下载地址由 Driver 生成，显示在齿轮说明中而非配置字段；取消已安装、设备名和滤波配置。窗口及指纹确认仍保留，旧配对凭据保持。同签名新版 APK 随候选容器交付。

删除 Live 的 first_acceptance/acceptance 人工标记及 operator/date/evidence_sha256，不伪填验收。实时反馈新鲜度、消息有效性、关节限位、模型解算、过期及停止逻辑保留；额外硬件状态门禁按下方最新决策移除。FSM 500/801 均可，不自动切模式。两卡复用 servo 下发；最新 r10 增加 1 rad/s 指令限速与平滑，不增加重力补偿或误差通过门槛。

## 验证和部署

1. 离线：无密码配对、旧配置兼容、固定 topic 身份校验、无设备操作请求、松握准备/重握不重标定、停止恢复；真实 IK 独立进程和原执行回归。
2. 普通 Core 隔离 UI 验证齿轮、说明、字段及监控；原生宿主测试、APK 构建和稳定签名核验分别记录。
3. G1 重启后核验身份、空闲/锁/租约；机上源码构建，只更换两个 Driver。保留现有画布和 Core/ActuCore，仅更新两卡 topic 元数据。
4. 验证镜像、无动作状态、ROS 节点/反馈、APK 下载和 PICO 安装重连；真实跟随由用户现场发起，不用构建或软件测试冒称真机通过。

当前 G1 为 r11b，设备卡与 APK 保持既有版本；本轮用户实体跟随/恢复反馈正常，详见末尾结果及验证记录。周六冻结完整身份未齐，不宣称 A/B 通过；用户已授权提交现有 PR，本轮不申请 BOT 或修改其他事项状态。

## 发布者检查与 IK 诊断修正

按用户要求取消 `_fresh` 中 DDS 手臂发布者数量及观察等待门槛，不自动停止其他发布者。反馈新鲜度、实际故障、FSM、内部执行权等检查保留。IK IPC 区分 timeout/exited/unavailable，记录操作、请求号、异常链和进程退出码，子进程 stderr 进入容器日志。超时仍丢弃旧通道，不重放旧请求。离线回归后按用户部署指令完成 r8 机上构建和 Driver 切换；Core/ActuCore 不变，实测由用户启动 Canvas 与握把输入。

## 2026-09-24 硬件状态门禁简化

用户要求取消自加的电机 mode/motorstate、电压、温度、遥控器按键、mode_machine 和腰腿偏差合并 fault 判定。SDK 原始电机读数仅监控，不再据此产生 robot_safety_not_ready。保留反馈格式/有限数值/时效、FSM 500/801、指令时效、关节限位及停止流程，不修改厂商控制器保护。源码回归后再部署；不得以重启掩盖或自动解除现场旧租约/故障。碰撞与恢复行为按下方最新决策执行。

## 2026-09-24 碰撞策略及连续恢复

用户明确要求暂不使用附加软件碰撞检查：G1IK 默认 collision_checks=False，不构建或执行躯干/双臂碰撞包络及其工作区扫描；保留有限关节值和 URDF 关节上下限。显式 opt-in 用于离线诊断，运行监控输出 collision_checks_enabled，不能把关闭检查说成碰撞验收通过。

可恢复 IK 保持在 servo 清空待发目标后允许同会话新帧续接，以 continuation_after_ns 拒绝保持前旧帧，不以关节速度归零为续接条件。hold_confirmed 仍仅记录物理保持确认，不能伪填。松握/显式停止/真实执行失败仍走独立流程。求解后校验失败清除 IK 滤波历史，下一有效帧使用实测关节求解，不重标定手柄基准。

本事项工作树归属开放 PR #330 的 codex/g1-motion-contract-20260923。2026-09-24 从用户提供的归档恢复后，核对目标原有文件与已批准采用的来源 manifest 完全一致，再迁入两卡增量；原四卡计划的既有修改保留。之后只在该 PR 工作树开发，不再使用 g1-two-driver 游离工作树。部署前仍需确认旧控制权已释放，不用容器重启强行清除旧锁存。

### 2026-09-24 平滑与 1 rad/s 限速（r10 已部署，待实测）

参考 PR #322 从上一条实际下发位置接续的原则，G1 先做120ms时间常数的一阶平滑，再将各关节增量裁剪到 ±1 rad/s × dt。dt 使用成功下发间隔，最大50ms，避免断帧或重握积累大步长；基准不是每帧实测位置。只处理最新输入，不排队补发旧目标。此限制约束下发位置参考的变化速率，不是实测电机速度的硬保证。

遥操专用肩肘 kp=80/kd=3，腕部40/1.5；不修改普通 servo 的全局增益。松握重握在清空旧待发目标后可 resume，无需速度或位置静止确认；显式停止确认仍独立保留。已部署 r10，当前空闲、无控制权；异响原因及新一轮实体跟随仍待验收。


## 2026-09-24 ROS 接收退出与重启绑定修复（r11b 已部署，用户本轮实测反馈正常）

### 已核验证据与未确认项

- 现场 00:52:01，公共 bundle_spin 在 rclpy 的 _take_subscription 访问已请求销毁的 handle 时抛出 InvalidHandle；main.py 的 spin_once 无异常边界，线程退出而 MCP 继续在线。
- 明确代码缺陷：TeleopControl.receive 首帧把 binding.instance_id 从 None 填为设备身份；TeleopBus._bind 比较整个 binding，因身份变化销毁并重建相同 topic 的订阅。该后台线程与公共 executor 并行，制造无必要的销毁窗口。日志未记录销毁实体，尚不能证明它就是此次异常的唯一触发源。
- 重启后 PICO input_fresh=true、约2ms；控制卡 idle、binding=null、feedback=null。现场 Core 重注册路径恢复配置/工具发现，未重放卡片 start(input_topic)。重启没有删除画布连线，但没有恢复运行订阅。此前“正常会自动恢复”的表述不适用于这个宿主实现。
- 重启退出日志另有 release_result_unknown，需检查无租约 stop 幂等性；不能据此伪报物理停止或释放成功。

### 实施范围

1. 将传输绑定键限定为 command_topic/feedback_topic，设备身份锁定独立管理。同 topic 重复 start 和首帧身份补全不得重建订阅。同进程内为遥操配置独立 SingleThreadedExecutor，由唯一 g1-teleop-ros 线程完成绑定、spin 和销毁；真正换 topic 用订阅代次及当前 topic 再校验丢弃旧回调。外部 DDS 契约不变。
2. 为公共 spin 循环添加针对 InvalidHandle 的恢复边界、计数与限频日志；普通未知异常保留堆栈并标记接收不可用，不无条件吞错或忙循环。健康监测使用执行器心跳而非仅线程 is_alive。
3. 保留设备身份和时效验证；明确暴露 waiting_binding、waiting_input、input_stale、transport_error。监控记录最后接收/接受序号、输入年龄、拒绝原因、订阅代次、执行器心跳和异常。发送成功不能清除接收错误（当前共享 last_error 每200ms被覆盖）。
4. 保持 Core/ActuCore 零改动：Driver 重启后必须由 Canvas 停止再启动项目重新下发绑定，不删线、不自动恢复运动租约。Driver 监控明确提示等待重新启动卡片。若要求运行项目在 Driver 重启后自动恢复，需要宿主运行状态协调能力，单独报告用户决策，不私改 Core，也不通过 Driver 自行接管绕过项目状态。
5. 检查无租约、从未下发动作时 stop 的幂等返回；存在未知输出/释放结果时如实报告，不把未知当已停止。

### 验证顺序

- 无硬件单元回归：首帧身份补全、重复 start、真实 topic 改变、stop/start；断言订阅创建销毁次数、旧代次回调丢弃、健康与错误保持。
- 目标架构真实 ROS 隔离集成（独立禁网容器中的 domain 42、无硬件设备/厂商 SDK 输出）：持续发最新帧，反复绑定/解绑及注入 InvalidHandle，确认接收可恢复、CPU不空转、无旧帧重放；不以 mock 替代此项。
- 用普通 Core 隔离环境验证 Driver 重启前后已保存连线仍在、重新启动项目正确传入 input_topic；监控区分未绑定与输入过期。
- 用户授权切换后，只部署 G1 Driver；验证公共 ROS 回调、/teleop/command 与控制卡序号持续增长。重新启动项目和真实动作由用户发起。保持 r10 kp80、120ms平滑、1rad/s不变。
- 异响独立未决；ROS 接收修复不能当作机械异响已解决。本轮已实现并隔离验证接收修复，后续 r11b 已部署；未主动发动作。普通 Core 完整重启联调仍未进行，不冒称宿主自动恢复已实现。


### 基线与本轮结果

- 用户确认能运动但有咔嚓声的 r9 已由独立提交 `8a0fe61f2c1695da1906fdf786dff19710b5858d` 推送现有 PR #330；基线独立暂存快照97项接口/执行及5项数值通过。r10与本轮ROS修改未混入该提交。异响未排除，不是完整验收基线。
- 修复工作树116项接口/执行/生命周期测试通过。目标 G1 ARM64 上禁网、只读、无硬件挂载的真实 Humble 集成通过：接收540帧，20次换topic（订阅代次21），3次 InvalidHandle 注入后继续接收，生命周期操作全部来自唯一线程。首帧身份补全不重建订阅；错误不会被监控发布清除。
- 真实环境首轮发现 Humble 无 rclpy.handle 模块，已改为 implementation_singleton 的实际 InvalidHandle 类型，重新运行通过。
- 无控制权 stop 幂等测试通过；release_result_unknown 对应仍在途/未知的厂商释放操作，本次未更改它的语义，不吞掉未知结果。


### r11b 部署及重复停止修复

切换预检发现用户停止Canvas时，旧版 arm.stop 在无SDK流、无租约、无目标的情况下仍创建释放任务；重复stop又将 action_id=None 的 awaiting_feedback 错判为厂商动作已发送，锁存未知。用户亲手重启旧Driver后恢复空闲。新增无接管stop直接no_op完成（不伪报物理停止），以及无action99的awaiting_feedback重复停止幂等；原有未知厂商结果仍保留。相关119项回归通过，候选镜像内禁网3次空闲stop不创建释放任务。

r11b 只切 G1 image，保留配置、画布、Core/ActuCore/PICO；机上构建完成，实际运行image local/phanthy-motus/g1:ros-receive-20260924-r11b，容器cd6d0395038a。只读启动验收：arm idle、无控制权/输出、applied_sequence=-1；独立遥操与公共executor均healthy，异常计数0，waiting_binding明确提示Canvas重新启动。实体跟随需用户启动，未宣称完成。证据在个人目录 updates/ros-receive-20260924-r11b/deployment-check.json。


### r11b 实体 PICO 试验反馈

用户启动Canvas与实体PICO后，约18秒只读记录中输入序号3219→4526，订阅代次始终1、ROS健康；目标有实际下发，松握进入operator_pause，arm hold_confirmed/resume_ready=true。恢复会新建执行会话，执行序号重置不能解释成倒序重放。用户随后明确反馈“这次的效果很满意”“正常的”，授权提交现有PR。这是本轮现场跟随/恢复体验反馈，不扩展为长期运行、机械诊断或所有故障注入验收。异响没有独立声学测量，不声称排除了机械问题。
