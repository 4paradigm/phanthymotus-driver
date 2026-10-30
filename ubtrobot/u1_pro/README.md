# 优必选 U1 Pro 驱动

本驱动为 Agent Core 提供 U1 Pro 的音频、相机、动作和系统行为开关卡片。机器人 SDK 控制服务使用 ROS domain `20`，设备音频/相机 topic 使用 domain `2`，Agent Core 使用 domain `42`。

## 卡片

- `mic`：调用 U1 SDK 的音频 `open_stream`/`stream_state`，优先从 SDK 音频共享内存读取；启动时给 SDK ring 1 秒首帧优先窗口，未收到 SDK 帧才切换到设备 domain `2` 的 `/sys/device/audio_in/raw` topic 备用源。驱动按消息采样率、声道和格式转换为统一的 `audio/pcm-16k`，启动时等待首个 PCM 帧；失败会关闭流并报告错误。
- `speaker`：订阅连接到卡片的 `audio/pcm-16k` 输入，优先通过设备 PulseAudio TCP sink 实时播放；`PULSE_SERVER` 环境变量可覆盖默认地址 `tcp:127.0.0.1:34567`。运行镜像中的 `pulseaudio-utils` 提供 `paplay`，设备侧必须开放对应 PulseAudio sink。队列短时满载时施加背压以保留 PCM；播放进程退出或 writer 持续无进展时停止接收并报告错误，避免无限阻塞 ROS 回调。未安装或无法启动 `paplay` 时才尝试已验证可用的 `EnableAudioOut`/`/sys/device/audio_out/raw` 兼容路径；两条路径都不可用时拒绝启动。音量和静音服务不属于本驱动能力。

部署服务使用 `restart: always`，用于主机或 Docker daemon 重启后自动恢复驱动；运维人员手动停止容器后，Docker 会在 daemon 重启时再次启动它。若需临时停用，应通过部署编排管理该服务，而不是依赖 `docker stop` 保持停机。
- `tts`：通过 U1 SDK `play_text` 进行文本播报，支持打断和 ACP 异步完成回报。真机音量服务不纳入本驱动能力。
- `expression`、`head`：通过 SDK `play_action` 播放声明的动作名称。动作提交后以 ACP 关联播放结果；有本地活动任务时，`interrupt`/`stop` 调用 SDK 中断服务。两张卡与 TTS 共享一个活动任务槽。`expression` 另有 `idle_start/idle_stop`，用于周期性播放现有表情预设。
- `system_controls`：统一控制 `wakeup`、`wakeup_followup` 和 `visual_behavior` 三个固件行为开关。
- `idle_actions`：独立控制自动待机动作及自动眨眼，支持 `enable`、`disable`、`status`，设置由厂商配置服务持久保存。
- `camera_left`、`camera_right`：打开 SDK video stream，并分别订阅 domain `2` 的 `/sensor/camera/left_eye/color/raw`、`/sensor/camera/right_eye/color/raw`，转换后输出 JPEG。SDK 共享内存视频流没有选择左右眼的参数；只有元数据明确标记了对应眼睛时，才会将环形缓冲帧作为该眼的备用输入，避免把一幅画面伪装成左右两路。

## 启动与行为控制

启动时先检查/完成 SDK 授权，然后依次关闭并读取确认 `wakeup`、`wakeup_followup` 和 `visual_behavior`。任一关闭请求失败或读回状态不符，驱动启动失败，不会注册卡片。需要使用时，可通过 `system_controls` 单独或批量重新开启。

这些开关不能停止固件内部 Agent 进程、ROS 服务、安全控制环或已经提交的动作。关闭 `visual_behavior` 用于停止固件自主视觉行为，不会屏蔽驱动显式提交的 expression/head 动作；显式动作可通过对应卡片的 `interrupt` 中断。

授权使用只读挂载的 `U1_PRO_AUTH_FILE` JSON 和同目录 license 文件，或受保护的环境变量。认证文件不得提交到仓库或写入镜像。

### expression 周期待机模式

在原 `expression` 卡片中新增两个 action，使用已有 SDK 预设模拟待机，不加载自定义舵机轨迹：

| 用途 | expression 输入 |
| --- | --- |
| 画布开启智能控制时自动接管并启动周期 | `{"action":"start"}`（平台自动发送） |
| 关闭原厂待机并启动智能控制周期 | `{"action":"idle_start"}` |
| 结束智能控制并恢复原厂待机 | `{"action":"idle_stop"}` |
| 查询循环和当前任务 | `{"action":"info"}` |

将expression卡片保留在画布中，点击“开启智能控制”即可自动启动：平台会向该卡片发送MCP `start`，驱动内部按 `idle_start` 执行，无需再手动发送一次。`start` 快速返回 `state:"loading"` 和 `action_id`，平台通过 `info` 观察原厂关闭确认结果；成功后变为 `ready`，失败为 `error` 并附带原因。重复 `start` 在周期正在启动或运行时复用原周期和关联ID，不创建第二个线程；手动 `idle_start` 仍拒绝重复启动。驱动进程启动时的Python生命周期 `start()` 只准备卡片，不自动接管；没有在画布中开启智能控制时不会因此关闭原厂待机或播放周期。

`play` 的名称、参数及完成通知保持原用法；调用时先关闭周期调度，当前任务确属该循环时，自动调用SDK中断，再播放指定表情。`interrupt` 关闭周期并执行原有共享任务中断，保留智能控制期间原厂待机关闭的状态；`stop` 则在后台中断当前任务、恢复原厂待机并结束智能控制。`head` 的接口不变。

无需再单独操作 `idle_actions`：`expression.idle_start` 在后台调用同一个原生控制器发送 `disable`，服务成功且读回持久配置为关闭后，才启用周期。关闭失败或结果不确定时不发送周期表情。原厂已提交的动作仍允许完成；读回开关关闭不代表物理动作已经停止，当前接管流程没有原厂待机任务的完成屏障，首次周期动作与残余原厂动作仍可能重叠。运行中不要用独立开关另行开启原厂待机。独立 `idle_actions` 卡片继续保留，其自身接口不变。

`idle_start` 快速返回 `state:"starting"` 和唯一 `action_id`；关闭、读回及启动结果通过 ACP 完成通知返回。**启动完成表示原厂开关已关闭且循环已启用，不表示整个循环结束，也不表示原厂残余动作已停止。** 恢复原厂待机期间的启动会被拒绝，避免创建多个循环。`x-completion` 包含 `start`、`play`、`idle_start`、`idle_stop` 和 `stop`，兜底240秒，以覆盖启动请求收尾、当前任务等待和原生开关的有界调用；内部单次表情等待仍为120秒。

后台从当前固件实际开放的 `confused`、`gaze`、`distracted`、`frown`、`pout`、`thinking` 中等概率选择，避免连续重复；如果固件只提供其中一个，则允许重复该动作。启动后以及每次动作成功完成后随机等待0.5–1.5秒，已有TTS、head或expression任务时让出。预设通过同一个SDK任务槽串行提交，等待厂商对应UUID的结果事件后再安排下一轮，不按估算时长重复下发。周期中不再主动选择 `blink`。

循环内的每次预设仅走内部完成回调，不重复向ACP上报，避免每次表情都触发Agent对话。请求失败、取消或120秒没有结果时，循环进入 `error` 并停止安排新动作，详情通过 `expression.info.idle_loop.last_error` 和驱动日志查看；不会自动重发或全局中断。缺少完成事件时，活动任务槽会保留，不能把超时当成物理停止。

`expression.info` 保留 `active`，顶层 `state` 在启动确认期间为 `loading`，恢复期间为 `restoring`，失败为 `error`，其余为 `ready/idle`，并提供对应 `message`。新增 `idle_loop`：`enabled`、`state`、`current_action_id`、`last_expression`、`completed_count`、`last_error`、`available_expressions`，以及 `smart_control_active`（本次接管尚未完成原厂恢复）、`restoring_vendor_idle`（正在恢复）。后台活动任务在 `active` 中标记 `background:true`。`x-stop.pending_states` 声明 `restoring`，避免平台把恢复中当成已结束；平台自身每卡停止确认死线若先到，可能显示“停止未确认”，后台恢复结果仍通过ACP及info读取。

`idle_stop` 快速返回 `state:"restoring"` 和 `action_id`：立即冻结后续调度，在后台等待尚未返回的启动请求收尾及当前驱动任务完成，再发送 `enable`，成功读回开启后才通过ACP报告智能控制结束。恢复期间拒绝新expression、head和TTS任务；不会为了退出而中断其他前台任务。当前任务120秒无完成结果、启动未收尾、enable失败或读回不确定时报告错误，不虚报恢复成功，允许再次调用 `idle_stop` 重试。若原厂偏好原本就是关闭，显式结束仍按用户要求发送enable，而不是恢复旧值。普通 `expression.play` 已支持自动接管周期动作，不必先手动 `interrupt`：

1. 校验表情名称后保留已开启的周期，检查当前任务ID、`background`标记及来源，只抢占本卡片循环自己的动作。交接及前台播放期间不安排周期动作。
2. 如果旧待机请求仍在提交，等待其返回；随后调用SDK `interrupt`。SDK调用期间保留任务占用和交接预约，其他expression、head、TTS不能插入。
3. SDK明确确认中断请求后，再提交指定表情；如果旧动作已经自然结束，则跳过不必要的中断。
4. 中断失败或超时，不发送新表情；通过新表情的ACP关联ID报告错误，旧活动任务不会被假装取消。尚未确认的旧请求也不会被盲目重发。

接管路径在后台运行，MCP快速返回 `state:"queued"`、新表情的 `action_id` 和 `preempting_action_id`；最终播放结果仍走原 `play` 的ACP完成通知。新表情失败也只报告一次。调用 `play` 默认保留已开启的周期，手动表情完成、共享任务槽空闲后自动继续周期动作；原本未开启周期则不会因手动播放而开启。显式 `interrupt/idle_stop/stop` 会停止周期，不会在手动表情结束后自动重启。这仍属于智能控制，原厂待机保持关闭，明确调用 `idle_stop/stop` 才恢复。如果当前占用者是手动表情、TTS或head，则不会自动中断它，保留忙碌返回。周期恰好处于等待间隔时正常播放手动表情，无需发送中断。

共享中断也会等待已经接收的SDK提交结束，并阻止新任务在中断过程中进入。只有SDK确认中断请求后才清除对应任务、报告本地 `cancelled`；失败时保留任务槽并返回错误。SDK确认和本地 `cancelled` 都不是物理静止的证明；此改动没有增加传感器停止确认或复位动作。正常驱动关闭时，若本卡曾接管，会中断驱动任务并等待原厂恢复；未接管的卡片不因正常关闭而改写原厂偏好。进程被强杀或机器断电时无法保证发送enable，可用保留的独立idle_actions卡片恢复。

### 自动待机动作卡片 `idle_actions`

此开关使用厂商原生 ROSA 配置服务，而 `system_controls` 的三个开关走公开 SDK，且在驱动启动时自动关闭。因此保留独立 `idle_actions`，避免把持久的待机偏好混入 SDK 开关和原有 `all` 操作。这张独立卡片的启动和停止不改写待机偏好，显式 `enable`/`disable` 才会修改；`expression` 智能控制现在复用同一控制器，在接管和退出时自动操作该偏好。

在 `idle_actions` 卡片上调用：

| 用途 | 输入 |
| --- | --- |
| 关闭自动待机动作 | `{"action":"disable"}` |
| 开启自动待机动作 | `{"action":"enable"}` |
| 查询当前设置 | `{"action":"status"}` |

`status` 恢复为同步查询，MCP 直接返回实际开关状态，例如：

```json
{"ok":true,"code":"OK","data":{"enabled":false},"source":"vendor_user_config"}
```

`data.enabled` 为 `true` 表示待机开关已开启，`false` 表示已关闭。查询沿用最初的原生配置读取逻辑，读取命令超时为 5 秒；查询失败直接通过 MCP 报错。查询不创建后台任务，不返回 `action_id`，也不发送 ACP 完成通知。

`enable`、`disable` 异步执行。MCP 首次返回会显示具体命令、中文说明、目标配置及任务编号。例如关闭操作返回：

```json
{
  "action": "disable",
  "message": "正在关闭自动待机动作及自动眨眼，允许当前动作执行完成。",
  "request": {"config_key": "idle_action_enabled", "enabled": false},
  "state": "running",
  "action_id": "<UUID>"
}
```

`request.enabled` 表示本次修改的目标值。`running` 表示后台正在处理，最终修改结果以 ACP 完成通知为准。

驱动在后台完成修改，再向 Agent Core 的 `/api/acp/complete` POST 一次完成通知。修改操作可在输入中提供唯一的 `action_id`（1–128 位 ASCII 字母、数字或 `_ . : -`）；省略时生成 UUID。卡片声明 `x-completion`（仅 `enable`、`disable`，兜底超时 45 秒）及独立的 `u1_idle_config` 资源。

完成通知包含相同的 `action_id`、`tool:"idle_actions"` 和 `status:"completed"` 或 `status:"error"`。成功时，业务结果位于通知的 `result.result`，包含 `ok`、`requested`、`enabled` 和读回的 `state`。其中 `enabled` 是持久配置中的开关值；任务完成表示配置写入与读回确认完成，不表示电机已经静止。

卡片一次只接受一个后台修改操作，忙碌时立即拒绝新的修改请求，不建立等待队列。查询与修改由底层锁串行化。`start`/`stop` 同步返回；停止卡片后不再接收新的配置修改，已接受的任务继续完成并回报真实结果，同步查询仍可调用。恢复接收修改可调用 `start`。ACP 通知发送失败时会记录转义后的任务 ID，平台通过声明的超时兜底。

修改时的服务拒绝、超时、读回失败或状态不一致通过 ACP 的 `status:"error"` 与 `result.message` 回报。厂商错误详情会转义换行、ESC、NUL 等控制字符并限制长度（每段 240 字符，最终 ACP 错误消息最多 1024 字符），避免原始输出进入日志或完成通知。

`enable`/`disable` 的修改命令遇到外层进程等待超时或内层 `timeout` 退出（124/137）时，后台任务会自动读回一次当前设置，读回另设 5 秒等待上限，且不重发修改请求。操作仍通过 ACP 作为超时错误回报；`result.message` 保留超时原因、目标 `requested`、读回结果和 `request completion unconfirmed; no automatic retry`：

| 错误消息中的读回结果 | 含义 |
| --- | --- |
| `readback=match; observed_enabled=false` | 示例：目标为关闭，读回的持久设置也为关闭 |
| `readback=mismatch; observed_enabled=true` | 示例：目标为关闭，读回的持久设置仍为开启 |
| `readback=unknown; observed_enabled=unknown; readback_error=...` | 查询也失败，当前设置未知，并附查询错误原因 |

读回只是查询时刻的设置快照。即使符合目标，也不代表收到了本次请求的成功响应；不符合目标时，厂商服务也可能稍后才完成处理。调用方应据此决定后续查询或重试。`status` 查询超时直接通过 MCP 报告失败，不递归查询。

关闭会让当前固件的循环待机分支和自动眨眼分支退出，禁止继续安排 `S017/S018/S019/S007` 待机动作及 `S020/S032/S033` 交互眨眼。但已经提交给运控的轨迹可能继续执行到结束：当前固件的 `PlayTrajectoryMotionFileNode::onHalted()` 清理本地等待状态，没有向运控发送轨迹取消请求。一次真机日志中，关闭配置被行为树读到后，已提交的 `S018` 轨迹又过了约 16.7 秒才返回。因此，`disable` 成功表示待机偏好已关闭，不表示机器人已经静止。

固件仍可能执行待处理的一次性 `Z001` 归位，`expression`/`head` 显式请求的动作也仍可执行。这张卡片不提供急停。厂商另有 `/mc/uworld_motion_stop`（`uworld_motion_msgs/srv/Trigger`），但请求没有动作名称或 ID，无法用它指定只取消某条待机轨迹；默认关闭操作不调用该接口。

调用链为：驱动 → `nsenter` 进入宿主命名空间 → 宿主 Docker → 已运行的 `walker-behavior.behavior-1` → `/sys/config/user_manager/set_config`（`config_msgs/srv/SetConfig`），仅提交 `idle_action_enabled` 布尔值。ROSA 使用 `--no-daemon` 发现服务；收到业务成功响应后，从厂商 `/data/user_data/user_config_center.json` 读回核验。驱动不直接改写该文件，也不重启厂商程序。

该接口依赖当前固件的容器、服务和配置结构，公开 SDK domain `20` 没有对应接口。容器名可在 `config.yaml` 的 `idle_actions.native_container` 中调整。部署沿用现有的 `privileged: true`、`pid: host`、host network/IPC，并由 `util-linux` 提供 `nsenter`；宿主需有 `/usr/bin/docker`，厂商容器内需有 `python3`、`timeout`、`/opt/walker/setup.bash` 和 ROSA。缺少这些依赖时，该卡片操作会报错。

## 运行环境

部署需要 host network、domain `20`/`2`/`42` 所需的 ROS 访问，以及对 `/tmp/robo/ipc` 和 `/dev/shm` 的运行时挂载。`/tmp/robo/ipc` 是临时运行态，不应作为持久配置目录；服务端每次打开流都会重新创建共享内存文件。容器需要 ROS Humble 的 CycloneDDS RMW。Docker 构建阶段的 colcon/CMake/C++ 工具仅用于编译本地 ROS 接口；最终镜像还包含运行期 Pillow、NumPy、PyYAML 和 CycloneDDS 依赖。

## 本地检查

```bash
PYTHONPATH=ubtrobot/u1_pro python3 -m unittest discover -s ubtrobot/u1_pro -p 'test_*.py' -v
python3 scripts/check_service_yml.py ubtrobot/u1_pro
```
