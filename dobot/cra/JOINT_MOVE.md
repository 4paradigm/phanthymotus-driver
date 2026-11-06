# joint_move

关节运动卡片使用官方 SDK，本次仅通过本地模拟连接验证，未进行真机运动测试。
安全皮肤卡片仍不纳入本版。

## 卡片形式

`joint_move` 为 actuator，提供五个可见 action，不提供 `get_status`：

| action | 内容 |
|---|---|
| `move_j` | 先 ClearError，检查并完成使能后执行 EnableSafeSkin(0) 和 ClearError，成功后复查状态，再运动至六个绝对关节目标角度 |
| `rel_joint_mov_j` | 按六轴相对偏移运动，使用同一套准备与完成流程 |
| `stop_motion` | 停止控制器运动队列/工程，也可停止点动 |
| `pause` | 暂停当前运动队列/工程，不暂停点动 |
| `resume` | 继续当前已暂停的队列/工程 |

`stop_motion/pause/resume` 作用于控制器的队列/工程，不限于本卡下发的运动。
卡片不申请控制权、不上电；move_j/resume 未使能时发送一次 EnableRobot 并等待反馈确认。

## 参数

| 参数 | 单位或范围 | 默认值 |
|---|---|---|
| `joint1_deg`、`joint2_deg` | 绝对角度，[-360, 360] 度 | 无，必填 |
| `joint3_deg` | 绝对角度，[-155, 155] 度 | 无，必填 |
| `joint4_deg`、`joint5_deg`、`joint6_deg` | 绝对角度，[-360, 360] 度 | 无，必填 |
| `joint1_offset_deg` 至 `joint6_offset_deg` | 相对偏移角度（度），0 表示该轴不变 | 无，必填，仅 rel_joint_mov_j |
| `velocity` | 本条运动速度比例，整数 1–100% | 10 |
| `acceleration` | 本条运动加速度比例，整数 1–100% | 10 |
| `detect_collision` | true 开启安全皮肤并设为 low；false 关闭 | false |
| `confirm_move` | 手动输入布尔值 true 确认本次操作 | 无 |

角度无隐式零值，不进行角度绕回或超限截断。初版只支持反馈识别为 CR3A（型号码 113）的设备，
角度范围依据《Dobot CRA 系列硬件使用手册》V3.2 附录 A 的 CR3A 参数。
这些是型号活动范围的输入校验，不能替代控制器中更严格的关节限制或现场避障检查。

参照 RM75 的确认参数形式，不提供下拉枚举或自动确认默认值。所有五个运动 action 都需要手动输入布尔值 `confirm_move=true`；六轴角度和速率仅对 `move_j` 生效。
速度、加速度直接映射到 SDK 的 `v/a`，不是固定的度/秒或度/秒²。实际速度还受全局速率影响。
固定发送 `cp=0`，不开放平滑过渡参数。

例如通过 MCP 调用：

```json
{
  "name": "joint_move",
  "arguments": {
    "action": "move_j",
    "joint1_deg": 10,
    "joint2_deg": -30,
    "joint3_deg": 60,
    "joint4_deg": 0,
    "joint5_deg": 90,
    "joint6_deg": -10,
    "velocity": 10,
    "acceleration": 10,
    "detect_collision": false,
    "confirm_move": true
  }
}
```

官方 SDK 生成的命令为：

```text
MovJ(joint={10.000000,-30.000000,60.000000,0.000000,90.000000,-10.000000},a=10,v=10,cp=0)
```

## 连接与状态

- 复用 `joints` 中配置的 IP 和反馈端口；本卡也提供相同共享配置，不依赖 joints 卡片启动。
- 准备阶段要求反馈有效、型号匹配、机器人静止且非拖拽/松抱闸。模式 9/11 和报警、碰撞、
  皮肤接近暂停标志不再提前阻止准备命令。
- `move_j`：先 ClearError，再必要时使能并等待确认 → 按 detect_collision 配置安全皮肤 → 清错 → 等待反馈确认已使能、
  模式 5 且无保护触发 → MovJ。已有暂停队列不会被当作新的 move_j 执行。
- `resume`：先 ClearError，再必要时使能并等待确认 → 按 detect_collision 配置安全皮肤 → 确认已使能、模式 10 且无保护触发
  → Continue；不额外清错。
- `stop_motion` 在反馈失效时仍可尝试发送，但必须已经配置控制器 IP。
- `info` 只介绍功能，`start/config/stop` 为内部生命周期，不发送物理运动指令。
  其中内部 `stop` 只关闭连接，真正停止运动须使用 `stop_motion`。
- action 返回异步任务 ID，终态通过 ACP 回调上报；实时角度继续读取 `joints`。
  不新增运动 sensor，不提供 `get_status` action。

五个 action 立即返回 `state=running` 和唯一 `action_id`，后台完成准备、提交和反馈监测。
通过 ACP POST 上报 `completed/error/cancelled`，对应同一个 action_id；SDK 应答和队列号不是到位通知。

| action | 完成条件 |
|---|---|
| move_j | 新反馈确认队列关联或已观察到运行，随后空闲且六轴实际角度到达目标（默认误差 ≤0.5°） |
| resume | 已观察到关联队列/运行结束；若有本卡暂停的目标，还要确认到达该目标 |
| pause | 连续两包新反馈确认模式 10、暂停标志，并且无运行/点动、六轴实际速度均 ≤0.1°/s |
| stop_motion | 连续两包新反馈确认无运行/点动、队列不再运行、六轴实际速度均 ≤0.1°/s |

运动超时或反馈异常返回 error，不把 ACK 当成完成；运动监测失败后尝试一次 Stop。
暂停/停止可中断当前动作，原动作以 cancelled 结束，新动作有独立 ID 和完成通知。
停止在反馈失效时仍可下发，但上报无法确认停稳，不上报 completed。
不自动重放运动命令；命令/反馈诊断保留在驱动内部，不新增状态输出口。

## 本地验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /tmp/dobot-build-fix-venv/bin/python -m pytest \
  tests/test_dobot_joint_move.py tests/test_dobot_relative_joint_move.py tests/test_dobot_completion.py \
  tests/test_dobot_cra_sensors.py tests/test_dobot_skeleton_structure.py -q
```

验证使用官方 SDK 和模拟反馈/命令连接，不发送真机运动请求。

## 运动前清错

每次 `move_j` 都在同一命令锁内执行 `ClearError()` → 检查成功应答 → 检查使能 → 必要时 EnableRobot 并等待反馈确认 → 配置安全皮肤 → 检查成功应答 → `ClearError()` → 检查成功应答 → 复查反馈状态 → `MovJ()`。
清错拒绝、应答异常或超时均中止本次运动，不自动重试。内部请求记录中的 `clear_error_response`
记录成功清错的应答。此应答不代表物理故障已消除。
最终运动检查保留：准备完成后若仍存在碰撞、保护触发或报警，则拒绝运动。
`resume` 只执行 enable 检查前的清错，不在关闭皮肤后再次清错；`pause` 和 `stop_motion` 不清错。

## 运动前安全皮肤设置

move_j 和 resume 提供布尔参数 detect_collision，默认 false：

- false：发送 EnableSafeSkin(0)，关闭安全皮肤。
- true：发送 EnableSafeSkin(1)，然后对小臂、J4、J5、J6 分别发送 SetSafeSkin(3,1)、
  SetSafeSkin(4,1)、SetSafeSkin(5,1)、SetSafeSkin(6,1)，灵敏度设为 low。

每条设置指令都检查应答并等待对应队列完成，失败或超时即中止，不自动重试。
本参数只控制安全皮肤，不修改机械臂本体的碰撞检测等级。
move_j 完成皮肤设置后再清错、最终检查并运动；resume 设置后最终检查并继续。
已有碰撞、皮肤暂停或报警允许进入准备步骤，但未消除时会阻止最终运动。
停止和暂停不修改安全皮肤设置；info 仅介绍功能，不提供设置状态读回。

## 自动使能

move_j 和 resume 在发送最终运动命令前读取实时 enable_status。已使能则不重复发送；
未使能时只发送一次 EnableRobot()，沿用控制器当前负载配置，并等待反馈确认。
默认等待上限 5 秒，可通过 joint_move.enable_timeout_s 配置。
拒绝、无效应答、超时、反馈失效或保护触发均中止，不重试使能或运动。
resume 在使能后还必须保持模式 10；若暂停队列已丢失、回到空闲模式，则拒绝 Continue。

最终状态确认最多等待 joint_move.enable_timeout_s 秒；应答成功不代表保护状态已经解除。
使能、皮肤设置或清错被控制器拒绝时直接中止，不改变上述顺序，也不自动重试。

## 完成通知部署

部署通过只读证书目录挂载提供 AGENT_CORE_CA_CERT，并使用带正确主机名的 AGENT_CORE_URL。
默认地址 https://phanthy-motus:15678，地址映射至 127.0.0.1；可通过 DOBOT_AGENT_CORE_URL、
DOBOT_AGENT_CORE_HOSTNAME、DOBOT_AGENT_CORE_ADDRESS、DOBOT_CA_DIR 和 DOBOT_AGENT_CORE_CA_CERT 覆盖。
TLS 不关闭校验，完成通知最多尝试三次，每次使用同一个 action_id；通知重试不重发机器人命令。缺少 CA 时拒绝开始运动/继续/暂停；显式 Stop 仍允许下发，回调失败会记入日志。
运动确认最长 joint_move.motion_timeout_s 秒（默认 300）；x-completion 的 timeout 另计准备与通信余量。

enable 检查前的清错应答保存在内部 pre_enable_clear_error_response；该步骤失败时不发送 EnableRobot、皮肤设置或运动命令。

安全皮肤开关及灵敏度设置都是队列指令：收到成功应答后等待新反馈确认对应队列 ID 已执行且队列不再运行。
此等待阶段允许本次准备指令导致的模式 7，但不允许实际运行、点动、拖拽或松抱闸。
清错后再次等待队列稳定，最后仍要求模式 5/10；等待超时记录队列 ID、模式及运行标志。

## 相对关节运动

rel_joint_mov_j 使用官方 SDK RelJointMovJ，六个偏移量单位为度，全部必填，不变的轴显式填写 0。
复用 move_j 的速度/加速度默认值、cp=0、detect_collision 默认 false、手动 confirm_move=true，
以及清错、使能、皮肤设置、队列等待、最终状态检查和 ACP 完成流程。
下发前用最新实际角度重新计算最终目标，校验实际角度加偏移不超出六轴标称行程；不进行角度绕回。
完成监测使用下发前计算的绝对目标，而不是将偏移量当作目标角度。

例如只让 J1 增加 10°、J3 减少 5°：

```json
{
  "action": "rel_joint_mov_j",
  "joint1_offset_deg": 10,
  "joint2_offset_deg": 0,
  "joint3_offset_deg": -5,
  "joint4_offset_deg": 0,
  "joint5_offset_deg": 0,
  "joint6_offset_deg": 0,
  "velocity": 10,
  "acceleration": 10,
  "detect_collision": false,
  "confirm_move": true
}
```
