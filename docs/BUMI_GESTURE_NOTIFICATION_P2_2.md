# Bumi P2-2 Driver：本地安全停止与ACP投递分离

分支`feat/bumi-driver`；HEAD `814b3ca953d33eba0aca9a61dcefa773f5da483b`；主机基线537项。本轮只处理gesture同步网络阻塞，不修改Core、traffic_light、SDK、servo、observation或Skill。

## 修改前后调用链

原`TrafficPoliceGesture.cancel`在operation_lock内调用_cancel_locked，后者完成原stop_playback、mode exit、settled、身体释放/locked提交后，又同步_notify_once→device._acp_notify→urlopen(timeout=5)+response.read，最后才能释放operation_lock并返回。网络错误虽被捕获，网络等待仍会阻塞后续安全操作与cancel返回。

现在：

```text
原stop_playback / WALK / DEFAULT
  → 原模式退出确认 / settled
  → 提交本地结果，确认退出才释放身体；否则保持locked
  → 保存本action首次终态快照
  → 释放operation_lock
  → _notify_once安排一次有界后台投递
  → cancel返回本地结果（不等待投递完成）

独立daemon：快照 → ACP HTTP → 仅更新投递记录
```

3秒模式退出、2秒settled、原SDK/安全规则及watchdog不变。没有先返回success再后台验证；确认失败仍locked，物理完成字段仍false。已有watchdog join只等待本地安全worker，不等待通知线程。

## 通知生命周期与界限

`traffic_police_gesture.py::_NotificationDelivery`是gesture私有的小型投递器，不管理Core pending：

- 每张Card按需创建一个daemon worker，之后复用；每次cancel/info不会创建新通知线程。
- 队列最多16份，加1份在途；records最多32份，不淘汰在途/排队记录。过载显式failed并记录日志，不阻塞本地停止，不额外创建线程或重试。
- _cancel_locked在原本地提交处保存首次terminal_status/result快照；locked之后的本地退出重试不能替换已经确定的ACP终态。
- _notify_once使用原state RLock保护notified标记，仅异步action有一次安排机会。实际调度在operation_lock释放后执行。同步启动失败仍无callback/返回action_id。
- 调度时deepcopy结果；发送者只获得old action_id/status/result/tool，不引用当前Card的active、last_result或身体预约。
- worker仅在出队/更新记录时持有自己的Condition；HTTP期间没有投递锁、operation_lock或身体安全锁。
- 网络超时/拒绝/HTTP错误只记failed/error，不重写本地安全结果；HTTP成功也不能清除locked或宣称物理completed。

`device._acp_notify`保持同一URL、payload、5秒socket timeout和异常日志，仅新增返回sent/failed传输回执，供gesture投递器观察。其他原有ACP调用者可继续忽略返回值；没有改变loco_relative_move的回调/idle顺序。

## 可观察结果

原info字段及cancel结果保持；新增可选`info.acp_notifications`：

```json
{"queued":0,"in_flight":"<old_action_id>","closed":false,"records":[{"action_id":"<old_action_id>","status":"cancelled","state":"sending","error":null}]}
```

state为pending/sending/sent/failed。sent只说明发送函数返回；真实HTTP helper对应请求返回并读完响应，不能证明Core已注册/结算pending。网络失败理由保留在对应record，不进入last_error或改变gesture_state。记录为有界诊断尾，不提供持久可靠投递承诺。

旧action本地已安全退出时，网络仍在发送不再独占动作锁，新gesture按原准入检查决定能否执行。locked、awaiting_verification、观察/servo/身体冲突仍拒绝；网络状态不能授权动作。

## shutdown

沿原`stop_all→LocoPlugin.stop→gesture.shutdown`清理。本地cancel/验证及watchdog回收优先，随后关闭投递器的接收入口，不join网络线程。已排队工作在进程仍存活时继续尝试；在途/排队时明确记录best-effort日志，进程退出可丢失这些通知。重复shutdown不重播、不重发或重新激活动作。

## 平台边界

本轮未修P2-1：Core仍在业务响应后注册pending（主仓库`agent-core/src/mcp_client.py:1062`），未知action的mark_action_complete返回False（143行），HTTP endpoint却仍返回ok=true（`start.py:713/725`）。cleanup_stale_actions只移除Event已set项（1708行）；background跳过普通等待。声明timeout不是可靠自动回收兜底。

Core整体stop预算仍为5秒（`api/config.py:1378/1445`）。本地验证本身仍可能超过该期限；本轮只去掉网络造成的额外同步等待，不缩短验证或延长Core timeout，不改ACP协议，也不通过delay/retry掩盖平台race。

## 修改文件

- `noetix/bumi/traffic_police_gesture.py`：本地终态快照、锁释放后调度、单worker有界投递、独立info记录及非阻塞关闭。
- `noetix/bumi/device.py`：仅_acp_notify增加传输回执；其他源码保持。
- `tests/test_bumi_gesture_notification.py`：新增19项Event控制、无真实网络的测试。
- 原gesture、bundle shutdown、Core lifecycle测试：只等待异步测试回执并回收测试通知线程，原安全断言保持。
- 本文、手势说明、总说明：更正Core timeout兜底承诺，记录准确边界及测试命令。

## 测试

所有网络发送均为fake sender或被替换的urlopen；没有请求真实Core、监听HTTP端口或连接机器人。Event/barrier控制慢发送，不sleep 5秒。

新增测试覆盖：cancel/lifecycle stop在慢HTTP期间返回；HTTP期间操作锁/状态锁可用、info与合法新手势准入；timeout/connection/HTTP错误与本地安全分离；真实watchdog与cancel竞争仅安排一次；旧在途及排队快照不污染新动作；首次locked终态不被后续恢复改写；验证完成前不返回或通知；locked/awaiting_verification；重复cancel；shutdown与P1-4异常隔离；单worker/队列及记录上限；worker启动失败独立可观察。

按顺序运行新增通知、原gesture、bundle shutdown、Core lifecycle模块，再完整回归，完整命令见[总说明](BUMI_SMART_TRAFFIC_POLICE_V1.md#主机测试与未覆盖项)。使用既有临时Python环境，禁用pytest插件自动加载、缓存与字节码，未安装依赖。

最终分阶段组合为158 passed（6.26s），包括新增19项及原有139项；最终完整组合为 **556 collected、556 passed、0 failed、0 skipped，60.28秒**。原gesture/bundle shutdown/Core lifecycle三个测试文件的216/41/41条assert AST摘要与开始一致，仅补异步回执等待和测试线程回收。

范围核对：允许清单之外的Driver文件内容聚合哈希及索引未变；device.py除_acp_notify外的文本保持；gesture的schema/start/stop/dispatch/preflight/watchdog等方法AST保持。主仓库623个文件内容、Git status及索引与开始完全一致，聚合SHA256为`c11944f4e940604a701e5c652e5cdf9c67f1e140df6e81594f56b1f5e982d645`。本轮不实施Core修复，不声称可靠ACP送达或全部停止均满足5秒。
