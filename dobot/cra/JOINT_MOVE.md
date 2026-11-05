# joint_move

关节运动卡片使用官方 SDK，本次仅通过本地模拟连接验证，未进行真机运动测试。
安全皮肤卡片仍不纳入本版。

## 卡片形式

`joint_move` 为 actuator，提供四个可见 action，不提供 `get_status`：

| action | 内容 |
|---|---|
| `move_j` | 检查状态后执行 EnableSafeSkin(0) 和 ClearError，成功后复查状态，再运动至六个绝对关节目标角度 |
| `stop_motion` | 停止控制器运动队列/工程，也可停止点动 |
| `pause` | 暂停当前运动队列/工程，不暂停点动 |
| `resume` | 继续当前已暂停的队列/工程 |

`stop_motion/pause/resume` 作用于控制器的队列/工程，不限于本卡下发的运动。
卡片不申请控制权、不上电、不自动使能。

## 参数

| 参数 | 单位或范围 | 默认值 |
|---|---|---|
| `joint1_deg`、`joint2_deg` | 绝对角度，[-360, 360] 度 | 无，必填 |
| `joint3_deg` | 绝对角度，[-155, 155] 度 | 无，必填 |
| `joint4_deg`、`joint5_deg`、`joint6_deg` | 绝对角度，[-360, 360] 度 | 无，必填 |
| `velocity` | 本条运动速度比例，整数 1–100% | 10 |
| `acceleration` | 本条运动加速度比例，整数 1–100% | 10 |
| `confirm_move` | 下拉选择是否确认本次操作 | false |

角度无隐式零值，不进行角度绕回或超限截断。初版只支持反馈识别为 CR3A（型号码 113）的设备，
角度范围依据《Dobot CRA 系列硬件使用手册》V3.2 附录 A 的 CR3A 参数。
这些是型号活动范围的输入校验，不能替代控制器中更严格的关节限制或现场避障检查。

所有四个运动 action 都需要布尔值 `confirm_move=true`；六轴角度和速率仅对 `move_j` 生效。
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
- `move_j` 要求新鲜反馈、已使能且空闲（模式 5），无报警、碰撞、皮肤接近暂停、拖拽、
  松抱闸、点动或正在运行/暂停的队列；连接建立后再次检查状态。
- `pause` 要求队列/工程运行中（模式 7）；`resume` 要求已使能且暂停（模式 10），
  并检查保护状态，不执行自动清错或使能。
- `stop_motion` 在反馈失效时仍可尝试发送，但必须已经配置控制器 IP。
- `info` 只介绍功能，`start/config/stop` 为内部生命周期，不发送物理运动指令。
  其中内部 `stop` 只关闭连接，真正停止运动须使用 `stop_motion`。
- 执行结果由 action 返回；实时角度继续读取 `joints`。
  不新增运动 sensor，不提供 `get_status` action，不自动判断“已到达”。

返回 ACK 时记录 `submitted` 或 `accepted_unverified`，并保留 `effect_verified=false`。
超时或断线记录为结果不确定，不自动重发。本卡保存最近请求记录，
切换控制器地址/反馈端口时同时清空。

## 本地验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /tmp/dobot-build-fix-venv/bin/python -m pytest \
  tests/test_dobot_joint_move.py \
  tests/test_dobot_cra_sensors.py tests/test_dobot_skeleton_structure.py -q
```

验证使用官方 SDK 和模拟反馈/命令连接，不发送真机运动请求。

## 运动前清错

每次 `move_j` 都在同一命令锁内执行 `EnableSafeSkin(0)` → 检查成功应答 → `ClearError()` → 检查成功应答 → 复查反馈状态 → `MovJ()`。
清错拒绝、应答异常或超时均中止本次运动，不自动重试。返回结果中的 `clear_error_response`
记录成功清错的应答。此应答不代表物理故障已消除。
原有运动前置检查保留：存在碰撞、保护触发、报警或未使能时仍拒绝运动，不能用清错绕过保护。
`pause`、`stop_motion` 和 `resume` 不追加清错，以免影响暂停队列。

## 运动前关闭安全皮肤

每次有效的 `move_j` 和 `resume` 请求均先发送 `EnableSafeSkin(0)`，包括复用连接时。
收到成功应答后重新检查运动条件；拒绝、无效应答或超时均中止，不自动重试。
`move_j` 随后执行清错与 MovJ；`resume` 随后执行 Continue，不额外清错。
应答记录在 `safe_skin_disable_response`，不宣称已通过状态读回验证关闭效果。
保留原有运动前置检查，已有碰撞、皮肤暂停或其他保护触发仍会阻止请求。
停止和暂停不关闭安全皮肤，卡片不自动重新开启安全皮肤。
