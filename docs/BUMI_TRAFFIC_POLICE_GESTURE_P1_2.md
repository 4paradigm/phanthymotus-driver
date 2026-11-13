# P1-2：交警手势 Card 的 Core 启动握手

本轮仅修复 `traffic_police_gesture` 的生命周期状态。分支 `feat/bumi-driver`，HEAD `814b3ca953d33eba0aca9a61dcefa773f5da483b`。保留开始前的未提交工作；不修改主仓库、traffic_light、SDK、loco、servo、observation safety、ACP或全局stop_all。

## 真实 Core 契约与根因

以下路径相对于只读主仓库 `/home/wu/motus-lab/phanthymotus`：

| 位置 | 实际行为 |
| --- | --- |
| `agent-core/src/api/config.py:375`，`payload_of` | 解开code/data与MCP文本JSON |
| `:398`，`tool_state_of`，`:425` | 读取 `state`，不读取 `status`；只有无state且有error时推断error |
| `:440`，`start_outcome` | running通过；ready仅在phase=start通过；idle/cancelled为cancelled；loading/starting/stopping为waiting；stopped、error、unknown、缺失状态等为error |
| `:890`，`_start_and_resolve` | 发送start+instance_id，随后真实info；`:1001`在start曾声明ready时用phase=start核验info，因此ready→ready可通过，ready→idle仍失败并在`:1017`加入errors |
| `:1346`，`_start_project_run`尾部 | errors非空时调用_do_stop_project回滚；只有成功分支才设置project_running=true |
| `:1381`，`_stop_cards`；`:1415` | 项目停止发送stop+instance_id，不带execute；按照工具x-stop核验busy/failed等，不把未释放当成功 |
| `:1472`，`stop_removed_cards` | 删除卡片复用_stop_cards |

原先start直接返回ready，随后info把“没有业务动作”的idle放到顶层state，Core由此判定启动取消并回滚。不能通过修改Core接受idle解决。

## 最小修复

`noetix/bumi/traffic_police_gesture.py::TrafficPoliceGesture`增加由现有chassis RLock保护的 `_started` 与 `_stop_requests`；没有创建第二套播放器或控制器。

- `start`只标记ready，重复调用幂等；不读取SDK，不检查recording ID，不发送身体动作。关闭、禁用、生命周期正在停止或locked时不会声称ready。
- `info.state`表达Card生命周期，`gesture_state`保留原业务状态。配置、active_gesture/action_id/recording_id、busy、last_result及物理完成限制仍可观察，增加last_error。
- `cancel`仍走原退出/停稳/身体释放路径，保留Card ready。生命周期stop先撤销ready，计数保护并发stop，完成后允许重新start。stop过程中拒绝重启或新播放。
- 兼容原本未先start的显式业务调用：只有既有身体预约和preflight成功后，才可顺带标记ready。ready不替代任何运动准入条件。
- shutdown只增加撤销ready；原cancel/join流程保持。全局stop_all问题P1-4未处理。

## info 状态示例

以下为省略不相关字段的示例；`<action_id>`是格式占位，未假定现场recording ID。

```json
{"state":"ready","started":true,"gesture_state":"idle","busy":false,"stop_configured":false,"go_configured":false,"active_gesture":null,"action_id":null,"last_error":null,"physical_completion_confirmed":false}
```

```json
{"state":"ready","started":true,"gesture_state":"running","busy":true,"active_gesture":"stop","action_id":"<action_id>","physical_completion_confirmed":false}
```

cancel确认退出后，ACP原操作终态仍是cancelled；info保留退出回执，Card仍ready：

```json
{"state":"ready","started":true,"gesture_state":"idle","busy":false,"last_result":{"state":"idle","reason":"playback_exit_requested","playback_exit_confirmed":true,"physical_completion_confirmed":false}}
```

播放异常但已确认退出时，Card仍可工作，错误在last_result/last_error保留：

```json
{"state":"ready","started":true,"gesture_state":"idle","busy":false,"last_result":{"state":"error"},"last_error":"<原错误>","physical_completion_confirmed":false}
```

退出未确认必须继续保持身体占用，start不能隐藏锁定：

```json
{"state":"error","started":true,"gesture_state":"locked","busy":true,"active_gesture":"stop","error":"<退出未确认原因>","last_error":"<退出未确认原因>","physical_completion_confirmed":false}
```

若此前调用的是生命周期stop，则started=false；无论started取值，locked都仍为error/busy，不能继续动作。生命周期stop确认退出后info为state=idle、started=false、gesture_state=idle、busy=false；停止并发期间为state=stopping。

SDK没有物理完成事件，因此没有新增“completed手势”示例或终态。自然返回walking仍是gesture_state=awaiting_verification、busy=true；不能据此释放预约或声明手势物理完成。底层stop_playback回执的completed仅表示原退出协议，不等于手势完成。

## 安全与范围

业务stop/go仍要求 `execute is True`，false/null/0/1/字符串/对象/缺失均不能播放。Core不带execute的stop只走生命周期cleanup。未配置ID不阻塞tools/list/start/info；实际业务才返回原reason=gesture_not_configured、missing_config=[缺失字段]。没有改成另一个错误协议，也没有默认编号。

`_execute_body_action`、身体owner、servo冲突、观察预约、旋转互斥、_cancel_locked、停稳验证、watchdog、_notify_once和ACP声明均沿用；本轮没有修P1-3、P1-4或P2。Skill原文两处依赖旧info=idle的检查同步为state=ready+gesture_state=idle+busy=false；requiredTools、业务决策、旋转预算和cleanup规则保持。

## 主机验证

按要求依次执行：手势模块90 passed（3.84s），bundle/Skill 36 passed（0.60s），Core集成22 passed（1.51s），再完整回归。命令统一禁用pytest自动插件、字节码和缓存，使用已有 `/tmp/bumi-traffic-v1-tests/bin/python`；未安装依赖。

最终统一结果：**492 collected、492 passed、0 failed、0 skipped，58.66秒**。以440项基线为基础，新增30项手势状态/安全用例和22项Core集成/契约用例；原有测试全部保留，仅将原awaiting_verification断言迁至gesture_state并补上生命周期ready断言。`git diff --check`通过。

`tests/test_bumi_gesture_core_lifecycle.py`只读提取并编译真实Core函数及项目尾部回滚分支；使用真实Bumi bundle构造器、tools/list、tools/call、HTTP handler和LocoPlugin，Controller及MCP传输为内存替身。没有导入Core服务、打开数据库、监听端口或调用真实SDK。

关键断言：两个ID为空/已配置均start→info→Core成功且无errors/rollback；旧idle响应对照必定进入同一真实回滚分支；Core项目停止及removed-card清理不播放停车手势；locked被Core正确拒绝。完整组合命令见[总说明](BUMI_SMART_TRAFFIC_POLICE_V1.md#主机测试与未覆盖项)。

主机测试不能替代真机验收；本轮没有commit/push/build/deploy/SSH或机器人操作。
