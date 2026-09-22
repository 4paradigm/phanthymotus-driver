# 天轶连续遥操执行入口（Draft PR）

本文件后续带日期段落为历史验证记录，不代表当前实时部署状态。当前 PR 测试结果和未完成项见仓根 `docs/plans/tianyi-teleop-pr.md`。Agent Core 不在本 PR 范围。

本实现接收 ActuCore 输出的双臂 14 关节与双手开合目标，不解析 PICO 输入或求解 IK。默认未启用。已有独立回放与收臂的物理记录，连续遥操长期验收仍未完成；历史版本故障不等于当前运行状态。

## 入口与配置

顶层 `teleop.enabled: true` 才注册 `teleop_executor`，`live_enabled` 默认 false。Live 还要求已验证的标定文件与真实现场验收记录；不能填写虚假的 acceptance 标志。现有设备配置不迁移、不自动启用。

MCP `info/start/claim/pause/resume/release/stop` 只允许回环地址且无浏览器 Origin。info 不启动线程或发布器。claim 返回 boot_id/session_id/secret；pause/resume/release/stop 使用同一租约身份。secret 仅经回环 MCP 返回，不出现在状态和 DDS。

连续目标：`/{namespace}/motion/teleop/command`；执行反馈：同前缀 `/feedback`，均为 std_msgs/String，domain 42、BEST_EFFORT、depth 1。独立 DDS 子进程验证本机隔离配置；厂商 domain 0 发布只在 Driver。

安全反馈使用独立 SingleThreadedExecutor，不与旧传感器节点共享多线程执行器，避免高频手臂回调导致电源/手部/躯干回调饥饿。执行线程异常报告 `feedback_executor_error` 并进入 HOLD；销毁节点前必须确认线程退出。运动仍要求反馈在 100 ms 内；本地续接候选将临时过期先保持、持续过期超过 300 ms 才锁存故障，详见下节。独立进程只订阅探针通过不等于部署后同进程负载验证通过。

命令协议 `motus.motion-target.v1` 包含 boot_id、session_id、递增 seq、同机单调 generated_ns、1–100 ms valid_for_ms、q[14] rad、hands[2] 闭合量 [0,1]、HMAC-SHA256 mac。关节顺序为左 11–17、右 21–27。无效数值、越限、旧会话、过期、重放均不执行。

仅双臂验收：共享标定文件显式设置 `hands_enabled: false`，Driver 不要求开合端点，并在 `_emit` 实际厂商写入路径屏蔽手部命令（即使输入 hands=[1,1]）。协议形状不变，info 报告 hands_enabled；省略字段仍为 true，非布尔值拒绝。当前实现把手部反馈新鲜度和手部故障从双臂-only 执行条件中移出；仍订阅并通过 hand_ns、hand_fault_reasons 如实报告，不伪造新鲜时间。启用手部时仍检查；双臂、固定体位、上电及急停检查不变。共享执行锁和其他手势入口不变。此增量已于 2026-09-21 在 latest-input-20260921-r1 部署；物理跟随仍待验收。

## 跟随、暂停和恢复

- 50 Hz 最新目标；候选从上一下发目标按配置速度推进，每周期最多20ms步长，目标领先实测不超过200ms对应行程（r7候选，当前1rad/s时0.2rad）。避免每帧重置到实测导致目标永远停在执行器的小误差区；静止执行器也不会无限累积目标。ActuCore检查实测与上一目标到新目标的完整运动段，只发送已检查目标。最大速度不超过位置接口的1.5rad/s，默认仍为0.2rad/s；本轮现场配置1rad/s。该候选尚未完成真机验收，不能据此认定厂商死区已证实。
- 当前实现在 DDS 后的 Unix datagram 队列也只取最新目标，不逐条处理积压包。最新包仍校验签名、会话、序号、限位和原始有效期。单周期 128 包仍无法排空则 HOLD local_dds_command_backlog，不应用不确定的中间包。此增量已于 2026-09-21 在 latest-input-20260921-r1 部署；物理跟随仍待验收。
- pause/断流进入 HOLD，向双臂发布实测保持位置；冻结手部，不自动张手。后续新反馈位置和速度确认保持后才返回 hold_confirmed。
- operator_pause、command_timeout、command_expired 或未升级为故障的反馈过期 HOLD，可在新鲜反馈确认保持后显式 resume，换 session/secret、清空旧目标并从实测重建。硬件故障、非法命令或未确认保持不可 resume。仅下述短暂续接窗口允许同一会话的新目标恢复；单纯反馈恢复不启动运动。
- 保持后的有界沉降补偿候选：只在 HOLD 中、位置偏差不超过配置速度100ms行程、速度≤0.02rad/s且不同新鲜反馈跨越100ms的位置变化≤0.002rad时，最多重新下发一次实测保持位置。重发后仍须新的反馈满足原0.02rad位置及0.02rad/s速度条件；停止命令立即下发，确认窗口最多为首次保持后的2秒，不延长、不追逐持续漂移、不解除已有FAULT。新增停止诊断包含首次保持、重保持及稳定采样时间；短暂反馈过期不重置已有保持目标或期限。83项本地回归通过；r6已完成机上隔离并部署（真机重测中），此前r5已部署，多轮实际回放退出后的停止及释放已确认。完整三轮双臂跟随与组合验收仍未通过，不将停止结果等同于整体运动验收。
- release/stop 只在真实新反馈确认停止后释放；反馈过期或停止失败锁存控制权。applied_sequence 仅证明目标已发布，不证明本体已经到达。
- 共享入口覆盖现有动作、轨迹、手势及 servo。servo 或异步动作仍运行时拒绝 claim，不静默抢占。

## 验证与已知剩余工作

首次现场验收入口（r2 已部署，首测 Live 已配置，首次试验后故障锁存，未实物验收）：配置 `teleop.first_acceptance_enabled: true` 且 `live_enabled: true`，共享标定 `hands_enabled: false`，并提供独立 `first_acceptance` 记录，包含 model_verified、workspace_verified、pico_verified、external_control_excluded=true，以及 operator/date/evidence_sha256。这组记录不代替普通 acceptance，不要求提前伪造 stop_verified/driver_crash_verified。

按 2026-09-21 用户决定，不将厂商电源／急停通信及停止契约资料缺失作为接管阻塞；power_feedback_verified 仅保留资料来源状态，不要求设为 true。运行时急停、上电、关节故障和消息到达新鲜度检查不变。普通 Live 的 stop_verified/driver_crash_verified 指实际停止验收，不要求厂商书面保证；首次验收负责采集这些证据。

现场显式调用 `prepare_first_acceptance`，通过静止、反馈、无竞争及无其他动作检查后，仅建立进程内 60 秒准备窗口，不 claim、不初始化执行发布器。随后由现有 ActuCore Live 入口显式启动。窗口到期由 Driver HOLD，原因 first_acceptance_expired，不自动释放锁或重启；确认停止并释放后方可重新准备。resume 不延长窗口，进程重启清除准备状态。配置速度、短步位置限制、命令 TTL 和碰撞路径保持原契约。双臂-only claim 不初始化手部发布器。

定向测试：`python3 -m pytest x-humanoid/tianyi2.0/tests/test_motion_stream.py x-humanoid/tianyi2.0/tests/test_teleop_recovery.py`，从 Driver 仓根运行。测试中的注入反馈仅验证状态机，不代表真机。

胸部局部参考已移除 /robot_pose 依赖。session-baseline 候选将头、腰、腿静止位置改为本次机器人反馈基准：显式 prepare_first_acceptance 时采集；普通 Live 在无租约的新 claim 时采集。要求完整、新鲜、有限且无故障反馈及速度 <=0.02 rad/s。接管后位置相对本次基准仍限制 0.02 rad，暂停/resume 不重设；故障或持有租约时禁止重标定。未准备时 fixed_body 仅表示当前关节完整且静止，fixed_reference_source=not_prepared；准备后报告 robot_session_feedback 及实际/基准位置。profile 内旧 fixed_motor_positions_rad 保留为历史来源，不再作为会话位置基准。VR 位姿不参与采集；长期模型、几何及验收记录不改。该版本已由用户切换，09:07:48Z 试验确认使用新会话基准；完整真机遥操仍未通过。

ActuCore 候选已完成保持恢复接入和内存契约测试。Driver 定向及相关回归 60 passed；全部天轶目录测试另有 3 项上游已有失败，详见仓根实施计划。本机 ARM64/Humble 真实 ROS/MCP/独立 DDS 子进程的隔离跟随、暂停、恢复、断流、释放已通过，使用模拟厂商反馈/写入，不代表真机。

生命周期与失败路径检查、ARM64 候选镜像构建及实际主入口 Shadow 重启测试已通过。构建、隔离测试和现场前置项见 [部署准备说明](deploy/TELEOP_RUNBOOK.md)。电源订阅到达时间仍不是硬件采样新鲜度。现场模型、停止和进程崩溃后的保持行为必须另行验收。


## 短暂超时续接（2026-09-21，已部署至 Shadow）

旧目标有效期仍为原始输入剩余的 1–100 ms，过期即清空并保持，不通过续发相同目标延寿。新增 `teleop.continuation_timeout_ms`，默认 300、允许整数 100–1000：从最近接收的有效命令计时，仅内部 command_timeout、签名/会话/序号/目标均合法但途中到期的 command_expired，或短暂反馈过期可开启续接。过期包不更新最近有效命令时间，不延长窗口。`continuation_allowed` 表示窗口仍开放；`continuation_ready` 还要求保持已经确认。收到仍有效、同会话且序号递增的新目标，重新校验反馈、固定体位和验收窗口后恢复；等待保持确认期间收到的目标直接丢弃，不排队。

显式 pause/release、握把松开、非法命令、竞争控制及硬故障关闭续接；即使原 HOLD 原因为 command_timeout 也不能被迟到包重启。超过窗口需显式重使能。保持本身仍不得自动张手。

反馈超过 100 ms 时停止生成运动输出、清空旧目标；缺少新鲜位置时不伪造保持命令或停止回执。`teleop.feedback_fault_timeout_ms` 默认 300、允许整数 100–1000，任一必需反馈年龄超过该值、时间戳缺失/未来、非有限数值、急停/断电/故障均锁存，真实安全故障不等待宽限。全部反馈恢复后先发保持、用后续反馈确认，再允许窗口内的新目标续接。

反馈 `diagnostics.first_hold/first_fault/last_fault` 保留发生时间、接收/执行序号、命令期限及各反馈年龄；`reason` 锁存首个故障，不再被后续过期覆盖。`last_command` 含 generated/received/accepted/applied/deadline_ns，`last_rejected_command` 不含密钥。`watchdog_timing` 拆分接收、控制权检查、执行及反馈发送耗时，`max_tick_gap_ms` 记录调度间隔。ActuCore 另外记录 `diagnostics.last_send`；旧客户端可忽略新增字段，旧 Driver 没有续接字段时不放行自动恢复。

用户已同意放宽短暂超时；上述软件经过隔离测试并已配套部署 ActuCore 与 Driver，零输出标定通过，未改变 Agent Core。startup补丁部署后的22:06试验已执行3个目标，随后碰撞恢复超时和租约回复丢失，最终停止释放；未完成可用跟随验收。USB不是PICO遥操连接的前提。

## 下发状态诊断

反馈新增 `timing_policy.command_state_version=1` 和 `command_state`，记录成功调用输出函数后的关节位置、实际采样时间、输入目标序号、位置是否被执行限幅以及有限差分速度/加速度。`stationary_seed` 仅表示已通过静止接管检查的规划参考，`published=false`；`motion` 才是已下发位置。保持时导数为空，长于 100 ms 的采样间隔也不估计导数。

这些字段不代表实测电机状态、连续轨迹导数或已停止；物理反馈仍以 `feedback.q/dq` 和后续停止确认作为依据。发布失败不会推进证据，字段不含租约密钥。旧 ActuCore 可忽略新增字段；不改变 v1 目标签名、TTL、执行限幅或保持恢复协议。

## 管理调用恢复候选

claim/resume可携带32位小写十六进制request_id及同机单调request_valid_until_ns，最长300ms。Driver保存最近一次成功结果；相同请求、原身份和截止时间只返回该次结果，不再次执行接管、旋转租约或延长目标期限。请求取消、到期或对应租约已释放时返回明确错误；错误回执不暴露secret，DDS/info也不包含管理请求或凭据。

若resume已执行但客户端未收到回复，release可携带原request_id、截止时间及原租约身份，取消该请求创建的确切租约。其他所有者不受影响。取消先到、原请求后到时，原请求被拒绝；管理调用超时不能作为已取消或已停止的证据，客户端仍等待新鲜停止反馈。普通pause/release/stop使已缓存接管回执失效。

新租约尚无任何目标时，ready_timeout_ms采用配置的continuation_timeout_ms（当前300ms）；到期后执行保持并确认释放。收到首个目标后立即按原始<=100ms TTL处理，request_id重试不延长此期限或首次验收60秒窗口。ActuCore配套把碰撞恢复拆为检查、接管、重新求解发送，未知接管结果时不发运动目标。Driver102项本地检查及机上禁网ROS2/MCP回复丢失恢复通过（硬件输出0），恢复r2镜像已构建并核对源码，2026-09-21 22:53已配套切换至Shadow，零输出标定与无执行权状态已回查；尚未物理验收。

## 独立执行回放证据

`info`及DDS反馈新增`last_vendor_command`：成功调用厂商位置发布后的单调时间、q_rad、speed_rad_s、motor_ids、publish_returned。它只证明发布函数返回，不证明电机移动；真实验收仍比较feedback.q/dq。录制、轨迹编译和回放编排在ActuCore，Driver不解析PICO数据，也不播放历史租约或旧时间戳。

本次自动验收授权只覆盖双臂：每轮仍独立prepare、控制权互斥及限时；停止未确认时保留租约，不允许发布工具重启清锁。临时执行器私有凭据journal用于恢复原租约的释放，不写入普通录制或报告。新增发布诊断的本地39项定向测试通过；新回放版本尚未物理验收。

2026-09-22 r7候选仅调整运动目标领先界至200ms×速度，状态显式报告position_lead_rad。每周期20ms增量、100msTTL/反馈、1rad/s现场速度、原保持/释放判据不变；停止后一次沉降重保持仍最多100ms速度行程，不随运动领先界扩大。Driver消失时本体剩余目标最多0.2rad，须作为崩溃边界单独核验，不能宣称TTL已实现本体急停。实测单次位置命令也响应偏慢，未据此调整厂商固件或速度单位。

2026-09-22 r8候选先拒绝不属于当前boot/session的报文，不验其旧MAC、不执行也不改变当前状态/目标/序号/有效期/续接窗口；当前会话签名错误仍保持。该修复避免合法旧会话延迟包用新密钥校验失败而打断新会话。last_rejected_command增加foreign_session和整数sequence诊断，绝不输出secret/MAC。旧会话报文不授予恢复权，正常新鲜签名目标与Driver原保持确认仍是续接前提。

r8镜像a5cb0825已与ActuCore r51配套部署；105项专项检查、ARM64隔离链路通过。最新真机execution-1790035073464889322首轮PASS，第二轮round_deadline且停止释放确认，没有签名错误；整体验收和扩大至0.2rad领先界后的进程崩溃物理验证尚未完成。保留失败报告，不视作完整遥操可交付。

r9候选仅对遥操专用反馈线程分批取最新：每批最多10次非阻塞spin_once，批间可中断等待10ms；KEEP_LAST depth=1保留，不排队回放旧状态，100ms有效性判定不变。读取异常仍保持，关闭不继续排空。真实五路只读对照CPU下降约59%，回调年龄未超过100ms；107项专项通过。尚未机上构建/部署，不能据此声明运动时反馈性能已通过。可复现只读工具为tests/benchmark_feedback_scheduler.py。

r9已于网络恢复后部署，但execution-1790040138175162010首轮仍超时，停止释放确认，未通过整体验收。r10候选修正看门狗周期：50Hz等待包含收包、检查、发布的工作时间；超时不补发积压周期，仍只处理最新目标。108项专项及ARM64隔离通过，镜像d7584545构建完成，切换进行中。1rad/s、100msTTL/反馈和停止判据不变，未修改Python全局切换间隔。

r10已于09:33部署。execution-1790040871650421251前两轮PASS，第三轮末段超时；三轮停止释放确认，当前idle无输出，整体验收未通过。Core/Perception原实例未变，ActuCore仍r51。后续须继续解决实际反馈等待和执行滞后，不能把两轮通过表述为全部闭环完成。

Driver r11候选仅调整主Driver本体domain0 executor：每批10次非阻塞spin_once后让出5ms，避免持续高频反馈占满共享Python调度。domain42循环、遥操独立反馈循环、QoS与callback group保持原值；不是反馈时间放宽。原108项回归及新增3项调度测试通过，镜像已构建，尚未部署/真机验证。

r11 e0589eaf于09:54部署，ActuCore仍r51，Core/Perception镜像及启动时间不变。ARM64禁网链路151次合成写入、硬件0、停止确认PASS，但退出有ROS context警告，未称日志无异常。execution-1790042101319356759三轮独立真实执行全部PASS，暂停恢复与停止释放确认；组合于约4.17秒以combined_schedule_late退出，停止释放确认、反馈新鲜，完整IK误差仍明显超过容差。组合失败保留，整体尚未通过。最终check确认idle、无执行权和输出。下一步分别排查组合输入调度阻塞及完整IK参考速度可行性，不以三轮独立通过替代组合或崩溃验收。

Canvas编辑锁解除、项目未运行后，r52 a0f7e31c于10:16部署；Driver r11、Core和Perception原镜像及启动时间未变。原录制重新编译187点后execution-1790043470954909974三轮独立真机全部PASS；组合完整719帧/10.003秒，无调度迟到或运行异常，最终停止释放确认。输入迟到P95 20.75ms、最大67.26ms，异步日志在本轮消除了此前中断。组合完整IK误差仍未通过：最大关节误差约1.585rad；134行submitted、312行releasing，按failure sequence去重有18次不可达、7次IK超时。不能只归因1rad/s物理速度或把完整输入跑完当成正确跟随。下一步区分原记录近零起姿下不可达、求解预算与恢复占用，原误差判据未改。最终check确认idle、无执行权和输出。


2026-09-22候选接口：`recoverable_hold(session_id, secret)`仅由租约所有者请求短暂IK保持，清除旧目标，保持确认后接受同会话的新鲜递增目标。能力字段`timing_policy.recoverable_hold=true`；旧Driver缺少该字段时ActuCore仍走原pause/resume。普通pause/release、故障和无效目标关闭此续接；不能用延迟recoverable_hold重新打开已暂停会话。新目标仍受100ms TTL、反馈、碰撞、限速及首测窗口约束。短暂IK成功后的首帧不再丢弃，握把松开后的新相对基准机制不变。跟随误差仅如实报告，不设精度通过门槛。候选尚未部署或真机验证。


2026-09-22 10:42：候选ActuCore r53（291a3d16）与Driver r12（94987915）机上构建完成。194项配套检查通过，另91项Driver生命周期/时效/恢复检查通过（有重叠，不累加为唯一测试数）。真实719帧输入注入11次失败、5组连续失败，全部首个有效解同会话续接；此项使用合成求解/反馈，只证明恢复流程。ARM64禁网实际候选Driver验证租约认证、保持确认、同会话续接、普通暂停关闭续接和释放通过，硬件输出0。



用户确认并行部署结束后，使用绑定当前Compose SHA的恢复入口恢复r53/r12。10:43:52读回AC291a3d16、Driver94987915；Core/Perception原镜像与启动时间不变，Shadow idle无执行权，recoverable_hold能力真实反馈可见。原719帧在新版重新编译187点。之后Canvas编辑锁短暂出现，按用户退出编辑确认重新预检PASS，10:46开始三轮独立+完整组合真机回放；结果待实际进程完成，未提前宣称通过。


首轮r53/r12真机execution-1790045214467039759：三轮独立执行均完成并停止释放；组合仅33观测帧后以stop_unconfirmed锁存，最终关闭流程停止释放确认，双臂最大变化不足0.024rad，不能称跟随成功。源码与新增真实MCP序列化回归共同定位：DriverLink.call新动作recoverable_hold漏附租约，导致invalid_lease；直接调用Driver的旧测试未覆盖此缺口。已修复凭据动作列表，128项Core回归通过；机上r54 c9c44da7禁网真实ROS/DDS+HTTP MCP故障注入PASS，合成厂商写入152次、硬件0，含350ms IK保持后首个有效解原会话执行与最终释放。Driver仍r12不改；待r54部署与重新编译原录制后继续三轮及组合回放。


### 四段链路记录候选（2026-09-22）

新增显式 `paired` 模式：原录制编译轨迹独立执行一次后，回放同源原始输入实时IK。Driver默认关闭的 `trace_start/trace_stop` 仅空闲时切换记录开关；运行中有界内存事件环记录所选命令、拒绝原因、socket覆盖数及厂商发布，不包含租约secret/MAC。独立只读订阅器分别采集domain0本体命令/反馈与domain42 Driver事件，以机器人单调时钟关联；丢事件ID、写入溢出、缺少观察器结束记录均保留为证据缺口。

`deploy/run_tianyi_chain.py` 默认仅预检，显式 `--execute` 才进行已授权的真机双轮回放；它不构建、不改Compose、不操作Agent Core。两个记录器先读到反馈才启动；证据导出排除私有租约journal。`deploy/summarize_tianyi_chain.py` 仅离线分析，不以跟随误差设通过门槛。厂商无序号，100ms接收窗与1e-6rad编码容差只用于候选关联，重复保持目标标为歧义；没有观察到不等于证明没发布。

本地Core 132通过/1跳过、Driver 77通过；ARM64禁网真实ROS+MCP隔离PASS（合成厂商输出154次、真实硬件0）。构建ActuCore r55 b72574a5、Driver r14 7dc326f1，恢复Shadow预检及切换通过，Core/Perception保持原实例。下一步机上原录制重新编译、执行四段记录的同源双轮；上述测试与部署不代表新双轮真机通过。


四段回放实查补充：r55的paired入口仍残留三轮限制，未取得控制权；r56修正入口后，execution-1790047597362033229在claim前因power_ns_stale拒绝，Driver事件数0、last_vendor_command为空，无真实目标输出。旧客户端误把“从未授予租约的请求取消”当成“已运动后停止”，等待不存在的stop_confirmed而报未确认。r57/r15增加同一300ms事务内等待新鲜安全反馈，以及绑定请求nonce/boot、Driver无租约且从未遥操输出的取消回执；仍不伪造物理stop_confirmed，正常已执行停止流程不变。真实MCP序列化回归覆盖恢复、取消、错误nonce/boot；ARM64禁网测试含初始200ms电源反馈过期拒绝和零输出取消，PASS，合成输出153、硬件0。已仅切换两个遥操服务，Core保持f4ff6a48及03:08:03Z原启动时间。

ROS只读检查：/arm/status 2002帧、500.12Hz、最大间隔39.44ms、关节错误0；domain42遥操反馈73帧、35.01Hz、最大间隔75.59ms。命令/反馈类型及QoS匹配，本体命令的arm_pub与arm_gesture_pub均为本Driver已登记节点，idle没有命令输入发布者。Driver日志有HTTP BrokenPipeError，不能将它等同硬件执行失败；继续结合RPC耗时与链路证据追踪。观察器新增ROS节点/端点/QoS快照、Driver安全反馈时间戳，默认只读且有界退出。r57/r15新双轮真实结果待回放，不预先宣称完成。


r57/r15同源四段结果（execution-1790048187083962915）：独立执行通过且停止释放；厂商发布1063次、独立ROS收到1063条，Driver事件ID无缺口，两个观察器正常结束、队列丢帧0。组合回放在4.79s以combined_schedule_late终止，已发74条厂商命令且ROS收到74条，实际双臂有移动，最终停止确认。组合已观测68次adapter调用、5次IK超时、1次不可达；首要回放器瓶颈是同步runtime.status耗时95.5ms，加submit耗时46ms使下一输入错过时限。不能把该失败误称为ROS未下发。完整IK最大误差1.74192rad仅如实报告，不据此另设门槛；重复保持命令的时间/值匹配存在歧义。

r58仅修改ActuCore：新增不复制显示历史的control_status及可省略状态副本的heartbeat；回放器从已完成的adapter事件取得IK目标，输入线程不调用重型public status。按原始时间轴只递交当前最新到期输入，明确计数被覆盖输入；真正超过100ms且没有较新输入的尾帧仍拒绝，不延长10秒回放或命令TTL。本地167项相关回归通过，最终定向73项通过；ARM64禁网ROS/MCP通过。已仅重建ActuCore为5e78a400，Driver r15、Agent Core原实例不变，原719帧重新编译并原样双轮复测中。ROS记录额外独立订阅电源topic的到达与状态，用于区分本体发布中断和Driver回调延迟。


2026-09-22 r58/r15 四段真机回放已完成（execution-1790048907122090051）：独立执行与10秒完整输入回放均结束且停止释放确认。独立本体命令1056条、组合104条，与各自Driver发布计数一致；实测关节无非零错误码，状态最大间隔分别17.78/23.81ms。组合618条输入观测加101条明确覆盖，共719源帧；140次adapter调用、63次IK成功、9次超时及26次不可达，仍有最长1205.68ms本体目标发布空窗，末尾不可达未在窗口内恢复。报告passed仅表示程序完成、双臂实测移动及停止确认，不能表示连续跟随问题已解决。完整IK误差mean/P95/max为0.322/1.210/1.758rad，仅报告不设门槛。

电源topic独立接收3557条、最长间隔64.52ms；Driver记录的电源状态年龄最大244.69ms，指向订阅处理/调度延迟，尚不足以断言具体回调根因。ROS节点与端点QoS快照已保留；两观察器stderr为空、正常退出且队列丢帧0。Driver仍有HTTP BrokenPipeError，最终只读info为idle、ownership_held=false、output_active=false、stop_confirmed=true、feedback_executor_error=null。离线分析按trace-start ID划定本轮事件，旧轮缓存单列；本轮事件无缺号，9项证据分析回归通过。

本任务仍只操作ActuCore/Driver。11:51:47 Agent Core被其他操作换为16674c24并重建，期间注册短暂拒绝连接、11:52:17恢复；本任务未修改或恢复它。当前保留该Core版本。完整证据位于机器人/data/hanzebei/pico-tianyi/evidence/paired-chain-r58-r15；下一步继续分离IK可达性、恢复空窗和反馈回调延迟，不重复要求PICO人工录制。


## 显式持续操作会话（r16，已部署）

新增 `operator_session_enabled`，默认 false。启用后 `prepare_operator_session` 在空闲、原厂模型/活动空间/PICO/外部控制前提验证通过时，刷新本次固定姿态基准并授予内存中的操作准备状态；不发布目标、不获取租约，也不修改 acceptance 记录。该入口面向已授权持续现场操作，不沿用 `prepare_first_acceptance` 的60秒试验期限。

prepare 后 claim/resume 继续检查本体反馈、限位、时效、竞争与停止确认。显式 release/stop、Driver 生命周期 stop 或进程重启撤销准备状态；`end_operator_session` 仅允许无租约时清除准备状态。准备失败不能沿用之前的准备许可。普通首次验收入口仍保持原限时检查。

收臂轨迹由 ActuCore 用现有最新目标接口提交，Driver 不解析 PICO 菜单，也不在 pause/release/断线时自行回零。Agent Core 无需变更。

2026-09-22 r16（f41c7bb5）已配套 ActuCore r61（6bf21a5b）部署为 Shadow，未改 Agent Core 或普通 ActuCore。连续准备许可已配置，但不是硬件验收记录；新收臂和持续操作真机验收仍待完成。
