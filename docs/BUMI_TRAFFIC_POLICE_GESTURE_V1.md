# Bumi 交警手势 V1：薄封装、生命周期与主机验收

本轮在 Driver 的既有智能交警工作区上增加 `traffic_police_gesture`，业务动作只有 `stop`、`go`。前者是停车手势，后者是放行手势。两者都播放现场配置的示教编号；没有新增关节轨迹播放器、底层电机 publisher 或底盘控制器。

工作分支仍为 `feat/bumi-driver`，HEAD 为 `814b3ca953d33eba0aca9a61dcefa773f5da483b`。本轮开始时工作区已包含上一轮视觉、观察安全与 Skill 实现，继续保留。未连接机器人，未运行服务、Docker 构建或部署，未执行 Git commit/push/pull。主仓库只读参考。

## 先审查、再复用的路径

| 入口 | 审查结论及复用方式 |
| --- | --- |
| `noetix/bumi/device.py::LocoPlugin.dispatch` | 姿态、semantic_action、action_recording 原有身体动作分支抽成 `_execute_body_action`；原调用继续使用默认的临时预约，交警手势保留同一个 `_body_action` 直到确认退出 |
| `LocoPlugin._do_teaching_action` | 原 `play_recording(recording_id)` 负责准备模式并调用 `_trigger_user_action`；新 Card 直接复用，不复制播放逻辑 |
| `LocoPlugin._trigger_user_action` | 原 PLAYTEACH 提交、模式23反馈、running/accepted 语义保留；交警取消已锁存时不再提交播放 |
| `LocoPlugin._do_stop_playback/_send_walk_exit` | 复用原停止播放：仅从模式23发送 WALK，模式2幂等；保护/其他模式不自动起身或启用 |
| `LocoPlugin._prepare_workmode` | 原始接口保持原准备行为；交警入口额外要求已经处于正常 walking 模式2，状态变化后也不自动启用或起身 |
| `LocoPlugin.is_moving/_do_move/_do_destination` | 同一身体预约阻止平移、原始/观察旋转和其他身体动作；非零 `publish_velocity` 也受保护 |
| `noetix/bumi/loco_servo.py::LocoServoPlugin._start/_halt/_chassis_conflict` | 复用现有控制权仲裁；手势前 active servo 拒绝，手势期间 start/resume 受原 chassis conflict 限制 |
| `noetix/bumi/observation_safety.py::ObservationSafety._snapshot/_settled` | 共用模式、21关节故障/速度、IMU有限数与停稳检查；不建立另一套反馈模型 |
| `noetix/bumi/main.py::BumiDeviceBundle` | 仅把新配置传给既有 LocoPlugin；Card 由其原 `get_tools` 与 `dispatch` 注册和调用 |

SDK只有模式反馈，没有示教动作物理完成事件、示教目录/内容查询接口、足底接触确认或可独立验证的新鲜反馈时间。没有把这些能力补成虚构接口。原 action_recording 与 semantic_action 的业务协议未替换。

## 配置与接口

配置位置为 `noetix/bumi/config.yaml` 的 `plugins.traffic_police_gesture`：

```yaml
traffic_police_gesture:
  enabled: true
  stop_recording_id: null
  go_recording_id: null
  playback_timeout_s: 60
```

两个编号默认均未配置。现场用已有 action_recording 示教并确认内容后填写；Skill 不接受编号，也不修改配置。编号只接受整数0..65535，拒绝 bool、字符串、浮点数及越界值；这个范围来自原播放接口，不表示编号对应的录制一定存在。SDK不能查询实际编号或校验轨迹是否包含腿部/底盘运动，必须现场确认只采用允许的手势录制，不能静默替换成预设动作。配置在进程初始化时读取，没有新增运行中 config 接口。

| Action | 参数 | 返回及效果 |
| --- | --- | --- |
| `stop` | `execute: true` | 播放 `stop_recording_id`，保留身体预约；与 `stop_move` 含义不同 |
| `go` | `execute: true` | 播放 `go_recording_id`，保留身体预约 |
| `info` | 无 | 只读缓存，无SDK调用或运动；Card生命周期 `state/started` 与业务 `gesture_state/busy` 分开，保留配置、action_id、最近退出结果、last_error和物理完成限制 |
| `cancel` | 无 | 复用 stop_playback；确认模式2和连续停稳后释放；失败保持 locked、busy及身体预约，可重试安全退出 |

Card 类型为 actuator，单实例，无 Topic 输出。两项业务动作的 `execute` 都是 schema 中的必填真值。

P1-2：Core生命周期 `start` 只将Card标为ready，不要求配置recording ID，不读取SDK或发送动作；随后 `info` 在空闲时返回 `state=ready, gesture_state=idle, busy=false`。播放时仍为 `state=ready`，但 `gesture_state=running/accepted/awaiting_verification`、`busy=true`，不能据ready跳过身体占用检查。`cancel`保留Card ready；不带execute的生命周期 `stop` 先撤销ready，再走原cancel，确认退出后info为idle/started=false。未确认退出的locked通过 `state=error, gesture_state=locked, busy=true` 暴露，start不会掩盖它。完整Core契约、并发停止保护和测试见[P1-2说明](BUMI_TRAFFIC_POLICE_GESTURE_P1_2.md)。

**为什么业务动作多一个 execute 标记：**只读审查主仓库 `agent-core/src/api/config.py::_stop_cards`（约1415行）发现项目停止固定发送 `action=stop`，并没有可声明替代 stop 动词的接口。因此没有修改主仓库；`stop(execute=true)` 才是业务停车手势，不带 execute 的 stop 只取消。生命周期 start 同样只返回状态，绝不播放。此规则同时写入 Card 描述、参数描述、Skill 和 MCP 路由测试。

未配置返回示例：

```json
{"state":"error","reason":"gesture_not_configured","gesture":"stop","missing_config":["stop_recording_id"],"command_sent":false,"error":"Missing stop_recording_id"}
```

播放返回为原 teaching receipt，加 `gesture/recording_id/action_id/physical_completion_confirmed=false`。例如：

```json
{"state":"running","gesture":"stop","recording_id":101,"action_id":"bumi_traffic_police_gesture_<uuid>","physical_completion_confirmed":false,"playback_timeout_s":60}
```

101仅为格式示意，仓库配置及 Skill 均不使用这个编号。未收到模式23确认时为 accepted，不能据此声称编号有效或手势已执行。

## 安全和 ACP 生命周期

1. 业务动作不排队：并发、重复 stop、stop→go、go→stop 均拒绝第二次播放，必须先取消并核对空闲。
2. 同一 chassis RLock 下检查观察会话、已有身体预约、旋转/平移、servo及SDK停稳；观察会话必须先结束并核验 released/stop_confirmed。旋转运行时拒绝，不暗中抢占旋转。
3. 通过共享 helper 预约 `_body_action=traffic_police_gesture.<gesture>`，调用原 `_do_teaching_action("play_recording", ...)`。只有明确未发送命令的同步失败才直接释放；提交后异常进入原停止路径。
4. 整个播放阶段保留预约。SDK模式回到2只显示 awaiting_verification，不发布 completed，不自动释放，不推断手势效果。现场观察结束后仍调用 cancel，检查退出结果和 info。
5. watchdog 默认60秒，配置仅允许有限1..300秒；模式异常/查询失败或超时走同一 cancel。不能确认停止则 locked，后续动作仍被阻止。
6. 取消在操作锁外先锁存，再串行退出；启动准备阶段收到取消后不再提交 PLAYTEACH，旧监视线程不能取消新动作。并发取消至多发送一次正常退出，故障重试可再尝试原停止路径。
7. 使用仓库标准 action_id 与 ACP callback，终态只有 cancelled/error，不伪造物理 completed。P2-2将本地停止/验证/状态提交与网络投递分开：释放operation_lock后，每个异步action最多安排一次快照通知，无自动重试。单个按需daemon串行发送，队列最多16份、另有1份在途，记录最多32份；过载/发送失败独立记录，不改本地安全结果。同步启动失败仍不返回action_id或发送提前ACP。x-completion保持background及“播放预算+15秒”的timeout声明，但该声明不是可靠自动回收计时器：当前Core丢失early callback后仍可能留下pending，属于独立P2-1平台问题，不能依赖有限timeout兜底。详见[P2-2说明](BUMI_GESTURE_NOTIFICATION_P2_2.md)。
8. 两个中断 hook 都指向 cancel；项目生命周期 stop 也只取消。原 action_recording.stop_playback 在本 Card 活动时转到同一个取消入口；不绕过身体预约。LocoPlugin.stop 在 shutdown 中先执行退出和监视线程回收；无法停稳则保留 locked并报告，不能假称已释放。

`playback_exit_confirmed` 是SDK模式与连续停稳读数核验，不是手势物理完成或硬件急停认证。固定等待时长、ACP cancelled/error、模式2都不能代替现场确认。Driver不会自动恢复先前servo或播放下一动作。

`info.acp_notifications`单独给出queued、in_flight、closed及有界records（action_id、status、state、error）；state可为pending/sending/sent/failed。sent仅表示HTTP调用返回并读完响应，不证明Core pending已结算。cancel仍先完成原本地验证，再返回本地结果，不等待网络。shutdown不join网络线程；进程退出时在途/排队投递为best-effort，不能声明可靠送达。原3秒模式退出、2秒settled及Core整体5秒预算未改，不能据本轮修复宣称所有stop均小于5秒。

## Skill 完整调用关系

可导入文件为 [`skills/bumi-smart-traffic-police.json`](skills/bumi-smart-traffic-police.json)，沿用 Agent Core Skill Workbench、JSON校验和 `/api/skills/import`；本轮没有导入账户、启动项目或修改Core。已有相同slug但内容不同会冲突，需要按原导入机制处理，不能静默覆盖。

```text
Camera → VOP → traffic_light.observe/latest
                  ↓ 逐目标 color/symbol/置信度/独立多帧稳定性
       质量不足且允许：finish_observation
                  ↓
loco_relative_move.observation_rotate → 原旋转worker/ACP
                  ↓ 同编号ready、stop_confirmed
       traffic_light.observe → 停稳后新帧重新识别
                  ↓
绑定明确车道/目标方向及signal_id；unknown/冲突 → 报告，不做手势
                  ↓ 明确停车或允许放行
observation_end → released且stop_confirmed → finish_observation
                  ↓
traffic_police_gesture.info → state=ready、gesture_state=idle、busy=false、已配置、现场手势/站姿已确认
                  ↓
stop(execute=true) / go(execute=true)
                  ↓
共享 _execute_body_action → 原 _do_teaching_action(play_recording)
                  ↓ 保留 _body_action、禁止旋转/平移/第二手势
人工观察效果；异常/超时/中断或确认结束 → cancel
                  ↓
原 stop_playback → WALK/模式2 → 连续停稳 → 释放预约
```

Skill保留每个 signal_id、color、symbol、各类confidence与bulbs。任何红灯不等于全局停车，任何绿灯不等于所有方向放行。目标方向/对应关系不明确、箭头unknown或有适用信号冲突时不执行动作。所有退出路径清理手势、未结束观察会话及视觉订阅，不能把请求成功当作物理完成。

## 确定性测试

`tests/test_bumi_traffic_police_gesture.py` 使用真实 LocoPlugin、原 teaching/stop helper、假的Controller和事件回执；SDK不导入，非零底盘命令在测试替身中直接失败。实际MCP handler通过内存JSON-RPC调用，无端口监听。`tests/test_bumi_traffic_skill.py` 使用真实Core校验纯函数与LLM工具schema转换，不启动Core。

| 用户要求 | 对应测试函数（均在新手势测试中，另有Skill校验） |
| --- | --- |
| 1、2 stop/go映射 | `test_business_gesture_maps_config_to_the_real_teaching_path` |
| 3、4 缺少两个ID | `test_missing_recording_is_named_and_never_falls_back` |
| 5 错误ID | `test_invalid_recording_ids_are_rejected_before_sdk_calls`、`test_unknown_recording_existence_is_not_claimed_as_a_success` |
| 6 已有语义动作 | `test_body_action_reservation_is_never_bypassed`及模式参数测试 |
| 7 旋转运行 | `test_inflight_rotation_is_refused_without_cancelling_it` |
| 8 servo冲突 | `test_servo_control_conflict_does_not_preempt_the_policy` |
| 9—12 连续/交替/重复 | `test_repeated_and_alternating_gestures_do_not_queue_or_overlap`、`test_cancel_then_next_gesture_is_a_new_operation` |
| 13 播放异常 | `test_playback_exception_returns_sync_error_without_orphaning_acp` |
| 14 cancel | `test_plain_cancel_and_legacy_stop_playback_share_the_same_exit_path` |
| 15 取消后再执行 | `test_cancel_then_next_gesture_is_a_new_operation` |
| 16 info无副作用 | `test_info_has_no_motion_or_feedback_side_effects` |
| 17 禁止平移 | `test_gesture_excludes_raw_base_and_observation_motion` |
| 18 原身体互斥 | `test_business_gesture_maps_config_to_the_real_teaching_path`、`test_recording_body_action_reservation_is_never_bypassed` |
| 19 终态至多一次 | `test_repeated_cancel_produces_only_one_acp_terminal`、`test_simultaneous_cancel_requests_send_one_exit_and_one_terminal`及失败重试测试 |
| 20 shutdown释放 | `test_shutdown_cancels_and_joins_before_releasing_body`、`test_shutdown_failure_is_locked_and_not_reported_as_idle` |

另覆盖取消启动竞争、并发手势、未知编号存在性、反馈异常/停稳失败、模式变化不自动起身、超时、自然返回walking不冒充完成、原始录制/semantic/reset回归，以及项目stop不会播放手势。保留原 action_recording/semantic_action 测试，未删除或弱化。

手势扩展阶段视觉/观察/手势/Skill与Bumi回归共422项通过（56.42秒，无跳过）；P1-1打包修复结果见[P1-1接入说明](BUMI_TRAFFIC_LIGHT_P1_1.md)，后续P1-2生命周期修复结果见[P1-2说明](BUMI_TRAFFIC_POLICE_GESTURE_P1_2.md)。完整命令记录在[总交付说明](BUMI_SMART_TRAFFIC_POLICE_V1.md#主机测试与未覆盖项)。所有运动均为测试替身，不能据此声称实机手势精度、录制内容、机器人站稳或SDK反馈新鲜度已验收。

## 本轮文件与现场待验证项

新增 `noetix/bumi/traffic_police_gesture.py`、`tests/test_bumi_traffic_police_gesture.py`、本文。修改 `device.py`、`main.py`、`config.yaml`、`driver.yaml`、Dockerfile源码COPY清单、Bumi README、完整 Skill JSON、Skill测试与总说明；Camera/VOP和观察旋转继续复用；后续P1-1将traffic_light迁入Driver共享包并注册到Bumi，手势代码未因该迁移改变。

后续现场再完成：录制并核验两个ID及手势内容，确认没有平移；核对站姿、足底接触、安全摆动范围；检查模式23/2反馈与更新频率、退出延迟、故障锁定和硬件急停；测量真实交通灯/箭头精度及目标车道对应关系；最后在受控区域验证完整Skill。本轮不进行这些实机操作。

结束时复核：只读主仓库623个tracked/untracked文件的内容聚合SHA256、Git status及索引SHA256与本轮开始完全一致。Driver分支和HEAD未变化，所有修改均未暂存或提交。

## 源码定位

以下行号以当前未提交工作区为准，均相对于Driver仓库：

- `noetix/bumi/traffic_police_gesture.py:19`，`_NotificationDelivery`：单worker、有界通知与投递状态。
- `noetix/bumi/traffic_police_gesture.py:125`，`TrafficPoliceGesture.get_tool`：动作schema、execute、中断hooks及ACP声明。
- `noetix/bumi/traffic_police_gesture.py:176`，`info`：纯缓存的生命周期/业务/投递状态；`:204`，`start`；`:218`，`stop`；`:229`，`dispatch`：业务动作与生命周期stop区分。
- `noetix/bumi/traffic_police_gesture.py:244`，`_preflight`；`:269`，`play`：准入、配置映射、保留身体预约和原示教调用。
- `noetix/bumi/traffic_police_gesture.py:333`，`_notify_once`；`:346`，`_cancel_locked`；`:381`，`cancel`；`:416`，`shutdown`：首次终态快照、去重、原停止路径和资源释放。
- `noetix/bumi/device.py:720`，`LocoPlugin.dispatch`；`:786`，`_execute_body_action`：所有身体动作共用路径。
- `noetix/bumi/device.py:1490`，`_do_teaching_action`；`:1581`，`_do_stop_playback`；`:1629`，`_prepare_workmode`；`:1709`，`_trigger_user_action`：现有播放/退出语义及最小取消保护。
- `noetix/bumi/main.py:146`，`BumiDeviceBundle.__init__`：配置转交；`noetix/bumi/config.yaml:12`：两个null编号与超时配置。
- `docs/skills/bumi-smart-traffic-police.json:8`：requiredTools；`:21`：完整instruction。
- `tests/test_bumi_traffic_police_gesture.py:86`：真实共享示教路径测试；`tests/test_bumi_traffic_skill.py:62`：实际Core展开后的stop/go参数与安全语义。
