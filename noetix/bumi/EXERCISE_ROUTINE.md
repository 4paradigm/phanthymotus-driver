# Bumi 动作编排：开发与真机验证

## 交付范围与真实状态

新增 `exercise_routine`（Bumi 动作编排）和 `exercise_routine_control`（编排控制），注册于现有 Bumi Bundle，
复用同一 `LocoPlugin` / `HighController`。支持顺序、片段播放次数、整套轮数、
片段间歇、轮间休息、异步任务、预览、状态、停止及异常处理，无需新前端。

三个候选素材是扩胸、双臂上举、双臂前推。随代码交付的
`resource/exercise_motions.json` 中全部 `verified: false`、`recording_id: null`。
没有录制过、没有证明编号存在、没有测出时长或动作完成条件，不能真机运行。

检查时的实际基线是 `43f7e37c2a938fb1a1c511f19bd55622ae569014`，不同于任务书
历史基线 `024b66e49eb2ef4d95602c21381c1c40ff9ba53b`。当前代码中已有独立声源方向、
相机内参、导航速度接口等演进。本实现沿用当前动作示教接口：PLAYTEACH 启动后只
报告 running/confirmed_started；walking=2、play_teach=23、protection=26。
SDK `include/highcontroller.h` 仍没有回放完成、素材列表或 get_mode 数据时间戳接口。

**真实完成策略仅为逐片段人工确认**。收到确认后退出回放，观察到 walking 后才继续。
不依据固定计时、静止或单独的模式改变自动宣称物理动作完成；不额外起身或使能。
未来自动策略必须增加有证据的完成信号与测试；配置为 timer/auto 等会拒绝真机运行。

## 本机先跑模拟

在仓库根目录执行：

```bash
python3 noetix/bumi/exercise_routine.py --demo
python3 -m pytest tests/test_bumi_exercise_routine.py tests/test_bumi_bundle.py -q
```

模拟入口只使用标准库，不导入 ROS/SDK、不连接机器人。内置演示两轮，共八次片段播放；
每次模拟片段只用 `dry_run_clip_s` 秒，这是虚拟时间，不是真实素材时长。
也可以把下面的计划保存为 JSON，用 `--plan /你的路径/plan.json` 模拟。
模拟会按实际设置等待 rest_s，长休息可 Ctrl+C 取消。

## 卡片接口

卡片参数由现有 inputSchema / x-action-params 提供；`start` 只返回 ready。
默认 `dry_run: true`。运行返回用于 ACP 的 action_id 和用于查询/确认的 job_id（当前值相同）。计划例子：

```json
{
  "rounds": 2,
  "round_rest_s": 10,
  "steps": [
    {"motion": "chest_expand", "play_count": 1, "rest_s": 5},
    {"motion": "arms_up", "play_count": 1, "rest_s": 5},
    {"motion": "arms_forward_push", "play_count": 1, "rest_s": 0}
  ]
}
```

开始与预检通过 MCP tool `exercise_routine`；执行期间的查询、确认和停止使用
`exercise_routine_control`，避免被运行中的身体 ACP 等待阻塞。arguments 分别为：

- 预览：`{"action":"validate","plan":上述对象,"dry_run":true}`。
- 模拟：`{"action":"run","plan":上述对象,"dry_run":true}`，返回 `action_id` 和 `job_id`。
- 真机预检：相同 validate 参数但 `dry_run:false`；任何素材未验证、编号未绑定、
  机器人不匹配或缺少验证记录时拒绝。validate 不读取或控制机器人。
- 真机启动：`{"action":"run","plan":上述对象,"dry_run":false,"confirm":true}`。
  只允许已处于 walking 且没有 loco.move 或其他编排占用。confirm 表示现场人员已
  确认机器人稳定站立、空间及素材适用；walking 仅是模式名称，不代表正在向前走。
- 查询（exercise_routine_control）：`{"action":"status"}` 或加 `job_id`。仅保留最近一项任务，旧 id 明确报错。
- 片段结束确认（exercise_routine_control）：当状态为 `awaiting_confirmation`，现场人员观察完整播放结束后，
  将 status 的 `job_id`、`confirmation_token` 原样填入
  `{"action":"continue","job_id":"当前id","confirmation_token":"当前令牌","confirm":true}`。
  每段令牌不同，提前、重复或旧令牌会拒绝。禁止让 LLM 按预计时长自行发送确认。
- 停止（exercise_routine_control）：`{"action":"stop"}`。先阻断后续调度，再由后台退出本卡回放；继续查 status
  直到 cancelled/error。停止不是硬件急停，不承诺毫秒级物理制动。
- 动作库及限制：`{"action":"info"}`。

状态包括 idle、running、awaiting_confirmation、resting、stopping、completed、
cancelled、error。进度包含轮、步骤、动作、片段播放序号、completed_plays、
休息剩余秒数、退出确认、控制权保留情况及错误；它是流程进度，不是轨迹百分比。
`completed_plays` 为已人工确认且成功退出的片段数（模拟时为虚拟片段数），
不是健身者动作次数，也不把中断片段计为完整示范。`internal_repetitions` 仅为素材说明。

## 休息、预估与上限

同一步各次播放之间用该步 rest_s；该步结束且还有下一步时也用它。
轮尾只用 round_rest_s；整套最终结束不休息，不叠加最后一步的 rest_s。

rounds、每步 play_count 各 1–20，步骤 1–20，总播放最多 200；休息 0–600 秒。
拒绝布尔值代替整数、NaN/Infinity、负值、错类型、未知字段和未知素材。
每段人工确认超时须大于 0 且不超过 600 秒；总超时预算（片段超时 + 启动/退出余量 +
间歇）不超过一小时，运行还有一小时硬期限。等待都可被取消。

仅当所有素材已验证且有实测 duration_s 时给出有条件的预计时长；否则为 null。
估计不含启动、退出及操作员确认等待。未配置素材的 120 秒仅作模拟计划的超时预算，
绝不用它判断真实动作结束；真实运行必须单独配置 timeout_s。

## 控制权、停止与生命周期

LocoPlugin 的动作入口使用同一可重入动作锁。编排在返回 run 前取得可撤销的身体
控制权；外部 loco、姿态、预设动作、录制回放或画布 stop 会先取消旧流程，
退出旧回放并确认后才执行外部动作。即使外部新回放仍是 mode=23，也不能让旧任务继续。
如果退出未确认，外部动作被拒绝，控制权保留；先排查状态，再显式 stop 重试。

编排开始前暂停 loco_servo；保留控制权期间，SDK 写锁内拒绝所有流式速度（包括零），
避免回放与 50 Hz 保持线程交错。流式卡也拒绝重新启动；编排结束不自动恢复旧策略。
外部独立 App/遥控器/其他进程绕过本 Driver 的同模式替换无法可靠检测；真机测试期间
应只保留本 Driver 一个指令来源。这个限制不能靠 get_mode 消除。

状态读取异常、保护或意外模式会中止；保护模式不强行 WALK。SDK getter 没有暴露
底层数据时间戳，因此读取成功也无法证明 DDS 缓存新鲜，info/status 如实标注
`sdk_data_freshness: unverifiable`。需要真机核查掉线行为，当前没有底层新鲜度保证。

驱动 stop 会取消并等待工作线程，关闭后不能再 run；重部署不恢复旧计划。
单任务重复 run 返回 busy；每个工作线程只发布一次终态。
`run` 声明 `x-completion`，返回唯一 action_id，以 `/api/acp/complete` 向 Agent Core
回报 completed/cancelled/error，ACP 等待期限为整套一小时上限加退出和回调余量。
HTTP run 响应写入并 flush 后才开放终态回报，并留出 Agent Core 登记 pending 的短窗口；
快速模拟提前结束也不能抢先回调。响应发送失败会取消任务、不上报未登记的 ACP；
驱动关闭会释放等待响应发布的线程。只有默认 https://localhost:15678 允许本机自签证书，
其他 HTTPS Agent Core 地址保留标准证书验证。
使用默认 AGENT_CORE_URL；回调失败记日志，不伪造动作结果或重复上报。Agent Core 不可用时
平台依赖 ACP 超时释放等待，现场仍可直接停止。单机 --demo 不发送 ACP 网络回调。

身体动作入口持有 base/leg/arm_l/arm_r 资源；新增的 `exercise_routine_control` 是
独立 operator 控制通道，只能查询、确认或取消已经持有身体控制权的任务。它没有 run、
plan 或 recording_id 入口，不能启动独立动作；确认必须匹配当前任务和片段令牌。
它仍是 actuator，未将可写操作伪装成 sensor/resource。逻辑控制通道与身体资源不相交，
因此运行期间可以通过正常工具调用处理确认/停止，而其他身体动作仍受 ACP 顺序约束。
真实 SDK 指令和下一片段始终在原任务的身体控制权下执行。两张卡均声明平台运动标记。

主卡声明 on_interrupt_motion/on_interrupt_all → stop hooks，支持平台按文档直接触发
取消，不等待身体 ACP barrier；此停止仍不等价于物理急停。工作线程启动失败不返回
可注册的 action_id；正常任务即使很快结束，也只回报一次对应 action_id 的终态。

## 制作第一个真实素材：现场逐步操作

1. 与 Master 一起确认示教条件、稳定站立、动作空间及硬件停止手段。
   先做双脚着地、双臂低幅动作；起止采用相同准备姿态，不能保证轨迹平滑拼接。
2. 由现场记录或厂商工具核对未占用 recording_id；当前 API 无法枚举/证明编号空闲，
   不要猜 101/102/103，也不要覆盖已有素材。
3. 在 action_recording 卡调用 `{"action":"start_recording"}`，按设备要求人工示教。
4. 选择经核实的编号，用 `{"action":"finish_and_save_recording","recording_id":你的编号}`
   保存。模式返回不证明持久化；必须实际回放验证。
5. 在清空运动范围后，用 `{"action":"play_recording","recording_id":你的编号}` 回放。
   记录启动延迟、完整播放时长、内部重复、起止姿态、中间停顿和结束后的模式；
   重复试验。running 只证明已观察到启动模式。
6. 现场确认播放完整结束，再调用 `{"action":"stop_playback"}`，核对 confirmed=true、
   workmode=2，并确认没有截断。测不准结束时继续保留人工确认策略。
7. 给三个素材分别填入动作库的 recording_id、robot_id、verified=true、
   duration_s（真实测量）、timeout_s（包含人工确认时间的上限）、verification_notes
   （日期、机器人、现场验证情况）、内部重复与起止姿态。不可复制测试夹具中的假编号和时长。
8. 在 config.yaml 的 plugins.exercise_routine.robot_id 填同一个真实机器人标识。
   动作库可复制到宿主机已有持久化挂载目录，再用 library_path 指向它；确认容器内可见。
   修改配置/动作库后重启对应 Driver 以载入，运行中计划使用不可变素材快照。
9. 先 validate dry_run=false，再运行单动作单次。现场完成一次 continue，确认退出和终态。
   逐步扩到三动作、一轮，再增加片段次数/轮数与间歇。全程保持人工确认。
10. 分别测试播放中 stop、休息中 stop、重复 run、旧确认令牌、外部卡介入。
    检查取消后不再启动后续片段。保护、掉线、退出失败等情形优先在 mock/台架验证，
    不要在无保护条件下刻意制造真机跌倒或断连。

## 部署与待验证事项

Dockerfile 已复制新模块，动作库随现有 resource/ 进入镜像；config.yaml 默认启用新卡，
依赖 loco 插件；缺少它或 HighController 时跳过新卡并记录原因，其他可用卡继续注册。
不改变原有动作卡参数。
代码及本地模拟验证已完成；镜像构建进度与部署指令以 GitHub PR 中 bot 的实际回复为准。
按入职流程推送功能分支并提 PR，评论 /request_bot_review 请求构建；
部署后检查画布卡片、参数表单、人工确认可达性、真实关节表现、SDK 模式与掉线行为，
再由对应 Master 验收。自动完成策略、三动作素材和物理安全/稳定性均未获真机证据。

## 可审阅的 PR 文案

标题：`feat(bumi): add operator-confirmed exercise routine card`

说明：Bumi 目前只能逐次播放示教录制。本变更新增 exercise_routine 卡，用受控动作库
管理顺序、片段播放次数、整套轮数和间歇，提供预览、后台任务、状态、逐片段确认和停止。
复用 LocoPlugin 并增加跨卡控制权，外部动作取消旧编排，退出未确认时阻止后续运动。
run 接入 ACP 一次性终态回报，独立编排控制卡用于运行期间的查询、人工确认和停止。
三个候选素材默认未绑定、未验证，真实运行拒绝使用；本地 dry-run 不连接硬件。
SDK 没有物理完成信号，本版明确采用现场人工确认，不宣称全自动编排或真机验收通过。

验证：运行本指南列出的 pytest 和无硬件 demo；具体结果以此次开发的最终验证记录为准。
仍需真机检查素材录制与回放、画布参数与确认交互、模式反馈、停止及故障行为。

## 本次本地验证记录（2026-10-21）

实际执行：

```bash
python3 -m pytest tests/test_bumi_exercise_routine.py tests/test_bumi_bundle.py tests/test_bumi_loco_servo.py tests/test_bumi_camera_specs.py tests/test_bumi_odom.py noetix/bumi/tests tests/test_file_upload_endpoint.py -q -p no:cacheprovider
```

结果：**313 passed, 1 warning in 13.57s**。覆盖新状态机、输入和素材校验、真实 LocoPlugin 的
mock SDK 调用路径、顺序/重复/轮数/休息、人工确认令牌、启动/退出未确认、
停止竞态、同模式外部回放替换、各显式动作入口、流式控制冲突、保护/状态读取异常、
线程启动失败、驱动关闭与缺少控制器的兼容性，以及既有 Bumi 相关回归。
新增 ACP action_id/终态单次回报、默认 HTTP 回调、回调失败、独立控制入口和中断 hook 覆盖。
另覆盖真实 HTTP 快速完成与响应发布竞态、响应失败/关闭清理、TLS 例外范围及原有上传端点。
一条 warning 为既有 common/file_intake.py 的 cgi 弃用提示。

`python3 noetix/bumi/exercise_routine.py --demo` 实际完成两轮、8 次模拟片段播放，
终态 completed，completed_plays=8，控制权未保留，物理退出确认为 null。
代码 AST、YAML、JSON Schema、卡片清单、Docker COPY 声明检查通过；
`git diff --check` 无空白错误。没有运行 ROS 节点、构建 ARM64 镜像或访问真实机器人。
