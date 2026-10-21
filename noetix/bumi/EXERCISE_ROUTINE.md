# Bumi 动作编排：单卡面板与真机验证

## 当前旧 Core：先使用英文卡片表单

现场已确认 `core:release.261015.6453ff3` 没有工作台接入；`ACTION=ui` 只返回 HTML JSON。
本轮新增功能直接使用普通 `exercise_routine` 表单，不依赖工作台，不需要为联排升级 Core。
以下步骤需要本轮新代码；已部署的 Driver `release.261021.51178a8` 尚不包含这些入口。

### 加入已有录制101–107

先配置当前机器人的 `robot_id` 和持久 `library_path`，不要覆盖原来的101库。
空闲时选择 `ACTION=register_recordings` 并执行：只补齐101–107的缺项，保留已有101名称、绑定、验证和测时；不播放、不重录。
新增素材按编号命名为“动作102”等，`verified=false`，不会虚构各编号的动作内容。
已有相同编号的素材继续使用原来的 key（例如 `raise_hand_101`），新项使用 `recording_102` 等。
重复注册不会覆盖；若磁盘库被外部修改则拒绝写入，要求重启加载。每次写库前保留 `.bak`，随后原子替换。
注册完成后重新检测 Bumi 服务能力列表，刷新卡片下拉选项。

### 每个动作只测时一次，无需估秒数

1. `ACTION=calibrate`，选择一个 `MOTION`，设置 `DRY_RUN=false`、`CONFIRM=true`，执行会播放一次真实录制。
   这是单段测试入口，可以测试刚注册、尚未验证的素材；仍要求有效编号、当前机器人匹配、walking起始模式及现场确认，不自动起身。
2. 复制返回的 `job_id`，预先切换到 `ACTION=finish_calibration`，填入 `JOB_ID`。
3. 现场看到整个录制完整结束后，设置 `CONFIRM=true` 并执行。程序退出回放，确认成功后保存窗口和人工验证记录。
4. `ACTION=status` 应最终返回 `state=completed`、`calibration_saved=true` 及 `calibrated_window`。超时、停止、保护、退出失败不会保存测时。
5. 对其余录制重复一次。后续编排的 `PLAYBACK_S` 可留空，每段自动读取各自保存的窗口，重启后仍保留。

保存的 `playback_window.seconds` 从确认回放启动后计时，至人点击完整结束为止，包含人工反应时间；它不是录制文件的精确时长。
不会用它填充 `duration_s`，也不会把定时执行标记成物理完整播放。若覆盖同一编号的录制内容，必须重新测时，SDK不能检测编号背后的文件已改变。
测时超过素材的 `timeout_s` 会中止；默认新项为120秒。请在动作结束后及时确认，避免把长时间无操作保存成窗口。

### 顺序联排：101 → 103 → 102，共两轮

完成这些素材的测时后，在同一张卡设置：

| 英文字段 | 值 |
| --- | --- |
| ACTION | 先 `validate`，通过后改 `run` |
| MOTION | `raise_hand_101`（已有101 key） |
| MOTION_2 | `recording_103` |
| MOTION_3 | `recording_102` |
| MOTION_4 … MOTION_7 | 留空 |
| ROUNDS | `2` |
| PLAY_COUNT | `1` |
| REST_S | `2` |
| ROUND_REST_S | `3` |
| COMPLETION_STRATEGY | `timed_exit` |
| PLAYBACK_S | 留空，自动读取三个不同的测时窗口 |
| DRY_RUN | 先 `true` 模拟，再 `false` 真机预检/运行 |
| CONFIRM | 真机 `run` 时 `true` |

每轮按槽位顺序播放，空槽跳过；`PLAY_COUNT` 应用于每个所选动作。支持重复选择同一动作，按用户顺序执行，不按编号排序。
预检应有 `total_steps=3`、`total_plays=6`，各段 `playback_window_source=operator_calibration`。
真机仍须现场核对顺序、间歇及每次完整播放；定时终态 `exited_plays=6`，不宣称自动识别物理完成。
再运行一组，在第一段中点击 `ACTION=cancel`，验证不启动下一段或下一轮。
模拟测试另覆盖101–107七个动作的非排序序列，两轮共14段；这是模拟控制器结果，不是真机联排验收。

### 为什么不能直接自动识别结束

已核对 SDK 的 `include/highcontroller.h`、`examples_py/highcontrol_py.pyi` 和 `src/highcontrol_pybind.cpp`：
只有模式、关节、IMU等读数，没有录制列表/时长查询、播放进度或完成事件，也没有用于确认模式缓存新鲜度的接口。
动作中间可能停顿，关节静止不能作为完整结束条件；退出命令造成的 walking 也不能反推录制自然完成。
本轮消除的是反复人工估时；完整的自动完成检测仍需固件/SDK提供可靠事件，或取得实际录制文件并核对格式与时间轴。
目前用户未连接机器人，文件目录与格式未知，未实现凭猜测读取时长，也未在真机上播放101–107。

## 使用面板

画布仅添加 `exercise_routine` 一张卡，ACTION 选择“打开动作编排面板”并执行。
打开页面、切换设置、预检和查询均不发送运动指令。

1. 下拉选择动作；用滑块或数字输入设置每轮播放次数、片段间休息、整套轮数和轮间休息。
2. 可添加动作、向前/向后移动、移除，调整整套顺序。普通使用不用写 PLAN JSON。
3. 默认“模拟，不控制机器人”和“人工确认完整结束”。先预检、模拟，再选择真机。
4. 真机启动前勾选现场确认。人工模式下，看到本段完整结束后点“本段已完整结束，继续”。
   面板自动携带当前任务与片段令牌，程序自行退出回放后继续，无需另调 action_recording 的 stop。
5. 需要按时间连续重播时，选择“按设定秒数退出并重播”；已测时的动作可留空窗口，逐段读取保存值，也可显式覆盖。
   没有统一默认秒数，不会把“约10秒”的反馈自动变成截止时间。
6. 同一面板显示轮次、当前动作、已执行段数、播放窗口/休息倒计时和错误，随时可点“停止流程”。

定时模式的含义是：启动 PLAYTEACH → 等待所填窗口 → 退出回放并确认 walking → 休息 → 下一段。
设置两轮时，这条链路会执行两次，每次退出都由程序完成。
**窗口太短可能截断动作。定时流程完成仅说明计划和退出指令执行成功，不证明录制动作完整播放。**
SDK 没有可靠的录制完成事件或模式数据时间戳，不能凭计时、静止或 workmode=2 宣称自动物理完成。
walking 是工作模式名称，不代表机器人向前走。

## 平台与单卡控制

兼容面板依赖参考 Core 的 `inputSchema.x-workbench` / `motus.workbench/1`，返回自包含 HTML。
只有宿主支持该协议时，面板才由 Core 放入仅允许脚本的隔离 iframe，使用受限 MessagePort
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
| motion | 第一个动作 | 必填 |
| motion_2 … motion_7 | 按槽位顺序加入后续动作 | 留空跳过 |
| rounds | 整套轮数 | 1；1–20 |
| play_count | 每轮每个所选片段的播放次数 | 1；1–20 |
| rest_s | 片段之间休息秒数 | 0；0–600 |
| round_rest_s | 轮间休息秒数 | 0；0–600 |
| completion_strategy | operator 人工确认 / timed_exit 按窗口退出 | operator |
| playback_s | 覆盖所有所选动作的窗口；留空按各自测时 | 大于0且不超过600；无测时则拒绝省略 |
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

选择 timed_exit 时，各 step 可提供正数 playback_s，省略则读取各自动作的测时记录；operator 模式不接受此字段。
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
robot_id 与素材一致。持久挂载沿用部署配置。注册和测时入口在空闲或单段退出成功后原子保存并更新运行库；不支持外部文件热改，外部修改需重启加载。

## 计数、休息、时限

同一步各次播放之间用 rest_s；还有下一步时，也使用刚结束这一步的 rest_s。
轮尾只用 round_rest_s，最后一次播放后不再休息。最多20步，总播放不超过200，预算不超过一小时。
人工模式预算按素材 timeout_s，定时模式按显式或已保存的播放窗口，加各次启动/退出余量和休息。
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
控制权写入/回滚/释放与速度检查及发布共用SDK写锁，已在途的流式发布完成后才能建立编排控制权。
锁序为 action → SDK写锁；暂停servo前释放SDK写锁，servo线程只用自身锁 → SDK写锁，不反向获取action锁。
每次退出必须 confirmed=true 且模式为 walking，才允许下一段。退出失败保留控制权，停止可重试；
保护模式不强行 WALK。停止先取消后续调度，再退出本任务回放，不把被截断片段计为完整播放。
其他显式身体指令先撤销旧流程、确认退出后才可接管，旧 worker 不得停止新回放。
外部 App 或其他进程绕过此 Driver 的同模式替换无法可靠检测；SDK 模式缓存新鲜度标为 unverifiable。

run/run_plan/calibrate 返回唯一 action_id，声明 x-completion，向 `/api/acp/complete` 单次回报终态。
HTTP 响应 write+flush 后才开放回报，快速模拟也不抢先；响应失败取消且不发未登记回报。
回调超时或失败记日志，不伪造完成；仅默认 https://localhost:15678 使用本机自签证书例外。
驱动关闭会停止工作线程，不恢复旧计划。单机 demo 关闭网络回调。

## 本地验证与待验收

```bash
python3 noetix/bumi/exercise_routine.py --demo
python3 -m pytest tests/test_bumi_exercise_routine.py tests/test_bumi_routine_recordings.py tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py tests/test_bumi_camera_specs.py tests/test_bumi_odom.py noetix/bumi/tests tests/test_file_upload_endpoint.py -q -p no:cacheprovider
```

回归覆盖表单文本/数字校验、两轮逐次退出重播、不同动作窗口、停止、保护、退出失败保留控制权、
人工令牌确认、同卡面板与旧控制别名、HTTP发布/ACP竞态、TLS范围及既有Bumi与上传接口。
上一版实际执行：372 passed，1条既有cgi弃用提示（15.04秒）；demo完成两轮8段模拟。本轮测试记录见下方。
本轮最终执行：**402 passed，1条既有cgi弃用提示（16.80秒）**。
审查指出的控制权竞争已用确定性并发测试复现：旧实现先返回run、速度仍在途；修复后控制权等待同一写锁，且不与servo暂停形成锁循环。
新增验证覆盖：101–107幂等注册并保留101、旧Core所需七个顺序槽位、测时保存/重启读取、缺失或错绑窗口拒绝、
三动作及七动作各两轮的非排序联排、停止/超时/保护/退出失败/磁盘失败不保存，以及测时任务的HTTP响应发布与ACP回调竞态。
七动作模拟顺序为 `101 → 103 → 105 → 102 → 107 → 104 → 106`，两轮共14段，每段退出后才重播。
AST、工具JSON Schema及面板JavaScript语法检查通过。当前机器人离线，未做本轮真机验证。
上一版无头Chrome在隔离宿主中直接调用参考Core工作台，未覆盖实际画布入口：打开不发运动、滑块同步、两轮定时逐次退出、
同卡两次人工确认且令牌不同、添加/排序、窗口内停止阻止下一轮，全部通过且无JS异常。
浏览器连接的是真实调度器与模拟SDK；不是机器人实测。

本轮以本地模拟和页面交互验证为依据，未操纵真实机器人；提交、镜像与审查状态以PR实际回报为准。
仍需在目标Core核对工作台支持、素材绑定，以及真机两轮重播、人工确认、停止与异常，再由Master验收。
