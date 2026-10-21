# Bumi 动作编排：单卡面板与真机验证

## 使用面板

画布仅添加 `exercise_routine` 一张卡，ACTION 选择“打开动作编排面板”并执行。
打开页面、切换设置、预检和查询均不发送运动指令。

1. 下拉选择动作；用滑块或数字输入设置每轮播放次数、片段间休息、整套轮数和轮间休息。
2. 可添加动作、向前/向后移动、移除，调整整套顺序。普通使用不用写 PLAN JSON。
3. 默认“模拟，不控制机器人”和“人工确认完整结束”。先预检、模拟，再选择真机。
4. 真机启动前勾选现场确认。人工模式下，看到本段完整结束后点“本段已完整结束，继续”。
   面板自动携带当前任务与片段令牌，程序自行退出回放后继续，无需另调 action_recording 的 stop。
5. 需要按时间连续重播时，选择“按设定秒数退出并重播”，为每段填写现场核准的播放窗口。
   没有默认播放秒数，不会把“约10秒”的反馈自动变成截止时间。
6. 同一面板显示轮次、当前动作、已执行段数、播放窗口/休息倒计时和错误，随时可点“停止流程”。

定时模式的含义是：启动 PLAYTEACH → 等待所填窗口 → 退出回放并确认 walking → 休息 → 下一段。
设置两轮时，这条链路会执行两次，每次退出都由程序完成。
**窗口太短可能截断动作。定时流程完成仅说明计划和退出指令执行成功，不证明录制动作完整播放。**
SDK 没有可靠的录制完成事件或模式数据时间戳，不能凭计时、静止或 workmode=2 宣称自动物理完成。
walking 是工作模式名称，不代表机器人向前走。

## 平台与单卡控制

使用 Core 现有的 `inputSchema.x-workbench` / `motus.workbench/1`，返回自包含 HTML，
不需要修改或部署平台前端。面板由 Core 放入仅允许脚本的隔离 iframe，使用受限 MessagePort
调用当前卡片的白名单动作；页面没有 fetch、外部脚本或 token 访问。
部署的 Core 需要支持该工作台协议；旧 Core 可使用普通卡的动作下拉和数字输入，滑块仅在工作台内。

运行仍声明真实身体资源 base/leg/arm_l/arm_r，保留 actuator 类型及危险动作标记。
面板控制经平台现有的网页直调路径抵达 Driver，因此不会等待自身 run 的 ACP barrier。
不靠伪造资源标签或把写操作改成 sensor 绕过互斥。
面板关闭前，Core 查询无运动副作用的 info.busy；还在播放、停止、回调或退出未确认时拒绝关闭。
浏览器关闭不是硬件急停。停止、关闭设备、断开 DDS 等实机行为仍需现场验证。

新卡片清单不再暴露 `exercise_routine_control`。旧画布/API 的该名称保留受限别名，
仅能查询、确认、停止已有任务，不能启动计划；新项目不需要添加它。
普通 LLM 身体动作仍受 ACP barrier 约束，逐段人工确认应由现场工作台按钮发出。
平台中断使用 `on_interrupt_motion/on_interrupt_all → stop` hooks，直接取消而不等待普通动作排队。

## 不打开面板时的普通参数

`run` 和 `validate` 显示独立参数：

| 参数 | 含义 | 默认/范围 |
| --- | --- | --- |
| motion | 动作库下拉选择 | 必填 |
| rounds | 整套轮数 | 1；1–20 |
| play_count | 每轮该片段的播放次数 | 1；1–20 |
| rest_s | 片段之间休息秒数 | 0；0–600 |
| round_rest_s | 轮间休息秒数 | 0；0–600 |
| completion_strategy | operator 人工确认 / timed_exit 按窗口退出 | operator |
| playback_s | 定时模式的本段窗口秒数 | 定时模式必填，大于0且不超过600 |
| dry_run | true 模拟 / false 真机 | true |
| confirm | 真机启动现场确认；定时模式包含窗口适用性 | false |

普通卡上的数字框支持数字或数字文本；true/false 支持布尔或明确文本，空值默认模拟且不确认。
嵌套计划仍严格检查类型，不把 bool 当次数，不接受 NaN/Infinity、负数、未知字段或未知素材。

高级多动作通过 `validate_plan` / `run_plan` 使用 JSON；旧 `validate` / `run` + plan 调用仍可用。
下面的 key 是默认候选，未验证时仅能模拟，不是可直接用于真机的素材：

```json
{
  "action": "run_plan",
  "completion_strategy": "operator",
  "dry_run": true,
  "plan": {
    "rounds": 2,
    "round_rest_s": 10,
    "steps": [
      {"motion": "chest_expand", "play_count": 2, "rest_s": 5},
      {"motion": "arms_up", "play_count": 1, "rest_s": 0}
    ]
  }
}
```

选择 timed_exit 时，各 step 额外提供正数 playback_s；operator 模式不接受此字段。
人工确认仅接受当前 job_id、confirmation_token 和 confirm=true；面板从最新状态取得这三项。
旧令牌、重复、提前确认均拒绝。LLM 不得按预计时长自行发确认。

## 素材与真实完成边界

三个默认候选：扩胸、双臂上举、双臂前推。`resource/exercise_motions.json`
仍全部 verified=false、recording_id=null；没有预置101或任何用户专属录制。
真机两种模式都要求素材 verified=true、有效编号、当前 robot_id 一致、有验证记录、
素材 completion_strategy=operator（表示人工验证来源）及有效 timeout_s。定时模式不会绕过这些条件。

现场录制与验证：用 action_recording 的 start_recording 开始，finish_and_save_recording
保存到经核实的未占用编号；实际 play_recording 回放，现场看到完整结束后 stop_playback，
核对 confirmed=true、workmode=2。编号、起止姿态和素材适用性由现场确认；API不能枚举素材或证明保存成功。
随后在该机器的库里绑定 recording_id、robot_id、verified、verification_notes、timeout_s；
仅实测时长填 duration_s，估测时长只作 reported_duration_s，不能自动作为结束信号。
库可以放在 `/opt/phanthy-motus/data/bumi/`，config.yaml 的 library_path 指向该库，
robot_id 与素材一致。持久挂载沿用部署配置，重启加载，不在运行中热改库。

## 计数、休息、时限

同一步各次播放之间用 rest_s；还有下一步时，也使用刚结束这一步的 rest_s。
轮尾只用 round_rest_s，最后一次播放后不再休息。最多20步，总播放不超过200，预算不超过一小时。
人工模式预算按素材 timeout_s，定时模式按显式 playback_s，加各次启动/退出余量和休息。
模拟使用 dry_run_clip_s 的虚拟时间，不把它应用到真实设备。

status 的 exited_plays 是已成功执行并退出的段数；completed_plays 只累计人工确认完整结束的片段
（或模拟虚拟片段）。定时模式仅增加 exited_plays，completed_plays 保持0，full_playback_confirmed=false。
人工真机流程完整完成后 full_playback_confirmed=true，其证据来源仍是现场人员。
这些数字不测量健身者动作，也不是人体计数。

estimated_duration_s 仅在素材全部有已验证实测时长时给出，不含指令延迟或人工等待。
scheduled_duration_s 是显式窗口加休息的安排，不是物理完成估计。没有实测时长时 estimated_duration_s=null。

## 互斥、停止和 ACP

复用同一 LocoPlugin / HighController 和动作锁。run 返回前保留控制权，暂停 loco_servo，
播放期间拒绝流式速度；结束不自动恢复旧策略，不自动使能或起身。
每次退出必须 confirmed=true 且模式为 walking，才允许下一段。退出失败保留控制权，停止可重试；
保护模式不强行 WALK。停止先取消后续调度，再退出本任务回放，不把被截断片段计为完整播放。
其他显式身体指令先撤销旧流程、确认退出后才可接管，旧 worker 不得停止新回放。
外部 App 或其他进程绕过此 Driver 的同模式替换无法可靠检测；SDK 模式缓存新鲜度标为 unverifiable。

run/run_plan 返回唯一 action_id，声明 x-completion，向 `/api/acp/complete` 单次回报终态。
HTTP 响应 write+flush 后才开放回报，快速模拟也不抢先；响应失败取消且不发未登记回报。
回调超时或失败记日志，不伪造完成；仅默认 https://localhost:15678 使用本机自签证书例外。
驱动关闭会停止工作线程，不恢复旧计划。单机 demo 关闭网络回调。

## 本地验证与待验收

```bash
python3 noetix/bumi/exercise_routine.py --demo
python3 -m pytest tests/test_bumi_exercise_routine.py tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py tests/test_bumi_camera_specs.py tests/test_bumi_odom.py noetix/bumi/tests tests/test_file_upload_endpoint.py -q -p no:cacheprovider
```

回归覆盖表单文本/数字校验、两轮逐次退出重播、不同动作窗口、停止、保护、退出失败保留控制权、
人工令牌确认、同卡面板与旧控制别名、HTTP发布/ACP竞态、TLS范围及既有Bumi与上传接口。
本次实际执行：**372 passed，1条既有cgi弃用提示（15.04秒）**；demo完成两轮8段模拟。
无头Chrome在Core原版工作台的隔离iframe中实际操作：打开不发运动、滑块同步、两轮定时逐次退出、
同卡两次人工确认且令牌不同、添加/排序、窗口内停止阻止下一轮，全部通过且无JS异常。
浏览器连接的是真实调度器与模拟SDK；不是机器人实测。

本轮以本地模拟和页面交互验证为依据，未操纵真实机器人；提交、镜像与审查状态以PR实际回报为准。
仍需在目标Core核对工作台支持、素材绑定，以及真机两轮重播、人工确认、停止与异常，再由Master验收。
