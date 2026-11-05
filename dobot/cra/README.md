# Dobot CRA joints, safe skin and system control

基于官方 V4 Python SDK 的 Jetson Docker 驱动。提供关节骨架/转角、安全皮肤状态与设置，
以及系统控制；不修改平台前端，不加载 STL 外壳。SDK 版本见 [SDK.md](SDK.md)。

## 卡片

| 名称 | 类型 | 内容 |
|---|---|---|
| `joints` | sensor | 六轴实际角度驱动的 CR3A 结构骨架及转角 JSON 输出；共享连接配置入口 |
| `safe_skin_control` | actuator | 实时触发状态 JSON 输出、总开关和分区灵敏度 |
| `systemcontrol` | actuator | 系统状态/报警、本体上电、使能、停止/暂停/继续、速度和拖拽示教 |
| `model` | resource | 平台渲染器使用的固定直角骨架 URDF，不发布 ROS topic |

已移除 `connection`、`joint_states`、`robot_info`、`software_info`、`arm_all_state`、
`controller_state` 及独立 `safe_skin` sensor 卡片。升级后请删除画布中的旧卡片，在 `joints` 齿轮处重新配置连接。
不会自动改动已有画布或把旧 sensor 重命名为新控制功能。

关节卡和安全皮肤执行卡共用一个反馈连接，默认 10Hz 发布 `std_msgs/String` JSON：

- `/dobot_cra/state/joints`，`sensor/skeleton`；只包含 `timestamp_ms/available/joints`，
  每个关节只包含 `name/q`，`q` 为弧度，关节名为 `joint1..joint6`。
- `/dobot_cra/state/joint_angles`，`data/json`；`joints` 的第二个输出口，
  只包含 `timestamp_ms/available` 和 `joint1_deg` 至 `joint6_deg` 六个独立字段；
  每个关节转角单独一行，单位为度，不重复输出名称数组或弧度数组。
- `/dobot_cra/state/safe_skin`，`data/json`，属于 `safe_skin_control` 的输出口。
  保留原 topic 名称，画布中原有 `safe_skin` 连线需改接到 `safe_skin_control` 输出口。

两个 `joints` 输出共用同一份反馈快照，不新增卡片或控制器连接。JSON 角度不依赖 URDF
是否可用；实时数据可用时 `available=true`，断线或过期时为 `false`，六个角度均为 `null`。
JSON 输出示例（数值为示例）：

```json
{
  "timestamp_ms": 1793836800000,
  "available": true,
  "joint1_deg": 90,
  "joint2_deg": -30,
  "joint3_deg": 0,
  "joint4_deg": 0,
  "joint5_deg": 0,
  "joint6_deg": 0
}
```

更新后在平台重新添加 `joints` 或刷新该卡片输出定义，以显示新增 JSON 输出口。

断线或过期时，关节输出为 `available=false`，骨架不再输出旧角度，`joints=[]`；
皮肤输出为 `available=false`，触发状态为 `null`，不会把未知当作“没有触发”。
关节数据流不再输出 IP/端口、连接/超时诊断、包计数、SDK 来源、型号来源及错误记录。
骨架移除了 `idx/dq/degree` 等冗余字段，型号识别及错误说明保留在后台 `model` 资源中；
无法匹配模型时骨架为 `available=false, joints=[]`。`config` 响应仍确认实际配置的 IP/端口，
这些配置确认字段不混入实时输出。旧下游如依赖 `joint_names/angles_deg` 数组，请改读对应的
`joint1_deg` 至 `joint6_deg` 字段；如需弧度，请由各角度字段转换，
骨架渲染器仍直接使用弧度 `q`。

## 卡片介绍与状态查询

三张可见卡片均保留平台内部 `info`；只有两张执行卡提供可执行的 `get_status`：

- `info` 只返回名称、功能介绍、参数 schema、配置 schema 和输入输出口定义。
  平台用于详情按钮及端口解析；不读取反馈、连接状态或请求记录。
- 执行卡的 `get_status` 在 action 下拉框中可选，查询最新有效反馈和请求记录；不发送控制命令，
  不要求 `confirm_change`，不会打开 Dashboard 连接。
- ROS JSON 输出口持续发布实时数据，其结构不因这次分离而改变。

`joints` 不提供业务 action，通过骨架和 JSON 输出口持续发布数据，保留平台内部启停和配置。
`safe_skin_control.get_status` 返回触发反馈、
命令连接状态和最近设置请求；`systemcontrol.get_status` 返回系统实际状态和本卡请求记录。
旧执行卡调用方若通过 `info` 读取运行数据，请改用 `get_status`。

## 固定直角结构

反馈型号 113 自动识别 CR3A。保留上游原始 `resource/cr3_robot.urdf`，`model` 返回
从该文件及同版本 STL 派生显示骨架：两根长杆分别沿杆身中心面走线，再接回关节原点，
每根杆有两个固定直角拐点，共四处。J2 杆身中心面为局部 Z=0.1288m，J3 为 Z=0.0123m，
依据官方 CAD 中段截面确定，来源记录见 `resource/README.md`。
转角通过固定坐标系表示，末端补偿旋转使原有坐标变换不变。附加节点全部为 `fixed`，不接受控制角度。
原有六个可动轴的名称、轴向、限位及所有物理 link 的变换保持不变。

同时将原始复合 RPY 分解为单轴固定旋转，兼容平台现有 XYZ Euler 渲染顺序，
并保持标准 URDF 的 Rz*Ry*Rx 变换。测试对零位、当前样例姿态和 30 组随机角度逐一验证
全部物理 link 的位置与朝向，避免只画出直角却改变机械臂姿态。

模型只用于结构显示，不是运动规划模型或完整 CAD 外壳。未知/不匹配型号不会套用 CR3A。
先确认反馈有效再打开 `joints`，以便平台首次加载模型时取得正确型号。

## 安全皮肤状态

`safe_skin_control` 的 JSON 输出口只发布以下四项（示例值）：

```json
{
  "timestamp_ms": 1793836800000,
  "available": true,
  "any_approach_paused": false,
  "regions": {"arm": false, "j4": false, "j5": false, "j6": false}
}
```

`regions` 的值直接表示各区域是否触发接近暂停，`any_approach_paused` 为其汇总。
`timestamp_ms` 是本次输出的 Unix 毫秒时间；`available` 表示反馈是否新鲜有效。
断线或过期时 `available=false`，汇总和四区值均为 `null`。普通碰撞不等同于皮肤接近暂停。

此精简格式移除了 `regions` 内的 `part/raw_flag/approach_paused` 嵌套对象，以及系统状态、
连接诊断、控制请求记录和无读回依据的配置字段。使用旧路径的下游条件需改为直接读取
`regions.arm/j4/j5/j6`。最近控制请求、执行结果和 Dashboard 状态通过 `safe_skin_control`
的 `get_status` 查看；`last_request` 只是请求记录，不是实际配置读回。

反馈不提供皮肤安装检测、总开关或灵敏度读回；四区均为 false 不证明皮肤已安装或开启。
本版不虚构距离读数或 TCP 复位指令。

## 安全皮肤控制

启动驱动、添加卡片、点击 start/info/get_status 均不打开 Dashboard、不接管控制、不写入设置。
只有显式控制请求才懒连接 `29999`。所有请求通过一条序列化连接调用官方 SDK；
接收按分号完整组帧，断线/超时不自动重放写入。停止控制卡或容器会关闭该连接，
但不取消控制器已经接受的队列指令，也不自动退出设备的 TCP 模式。

| action | 参数 | 官方 SDK |
|---|---|---|
| `info` | 无 | 平台内部卡片介绍、参数和输出口定义 |
| `get_status` | 无 | 当前触发反馈、命令连接状态和最近设置请求 |
| `start/config/stop` | 无 | 返回 JSON 输出口定义；start 读取反馈，stop 关闭命令连接，均不修改皮肤设置 |
| `enable` | `confirm_change=true` | `EnableSafeSkin(1)`，开启安全皮肤总开关，不改变各部位灵敏度 |
| `disable` | `confirm_change=true` | `EnableSafeSkin(0)`，关闭安全皮肤总开关，不是机械臂下使能或碰撞复位 |
| `set_sensitivity` | `part=arm/j4/j5/j6`, `level=0/1/2/3`, `confirm_change=true` | 内部映射后调用 `SetSafeSkin(part,level)` |

部位下拉框显示“小臂、J4（第4关节）、J5（第5关节）、J6（第6关节）”，
对应参数值 `arm/j4/j5/j6`，SDK 使用的 3/4/5/6 编号只在内部映射。
请求记录返回 `part` 名称及中文 `part_name`，不再只输出数字部位。
总开关不再要求输入 `enabled`：选择 `enable` 或 `disable`，将 `confirm_change` 下拉框选择为
“确认执行本次设置”后提交即可。默认“未确认（不执行）”，不会发送设置命令。
旧的 `set_enabled` 动作已移除，调用方需改用 `enable/disable`。

灵敏度下拉框直接标出接近距离，依据《DOBOT 安全皮肤用户手册 (CRA)》V1.4（2026-01-22）
第 4.1 节（正文第 5 页）：

| level | 下拉框显示 |
|---|---|
| 0 | 关闭该部位 |
| 1 | 低灵敏度（接近距离 ≤5 cm） |
| 2 | 中灵敏度（接近距离 ≤10 cm） |
| 3 | 高灵敏度（接近距离 ≤15 cm） |

这是手册给出的档位范围，不是实时距离读数。手册参数表另注明不同部位感知距离有区别，
感知目标为人体、金属、液体等导体，对非导体感应能力较弱。
原有直接 API 调用的整数部位编号仍可解析，但新卡片不再展示数字部位选项。
每次写操作都要明确确认，平台按 actuator 危险操作处理。驱动要求新鲜反馈、机器人停止，
没有报警、碰撞、皮肤触发、拖拽、暂停或抱闸松开。
TCP 控制权统一通过 `systemcontrol.request_control` 请求，安全皮肤卡不提供此动作，
后台也拒绝从皮肤卡调用；皮肤查询不再混入 `tcp_control_requested`。
条件不满足时返回原因，不代为使能、清错、关闭其他保护或恢复运动。
设置过程中再次检查反馈，避免连接耗时后仍按旧状态发送。

开关和灵敏度是**队列指令**：返回 0 只说明接受请求。返回 ResultID 时状态为
`submitted`；旧固件不返回 ID 时为 `accepted_unverified`。无配置读回接口，所以即使
接受也始终标记 `configuration_verified=false`。可通过控制卡 `get_status` 查看请求返回的队列编号，
并在厂家软件中核对实际设置；下使能时接受的队列可能等待后续使能，不会由驱动自动执行使能。
明确错误码返回 `rejected`，通信不确定性返回 `failed_or_outcome_unknown`，均不自动重试。

文档没有单独的 TCP 皮肤复位函数；关闭/重开皮肤不作为复位动作。TCP 设置只在对应
控制模式中生效，退出后按厂家规则恢复原配置。SDK 错误 -1 还可能表示没有安装安全皮肤。

## 系统控制

`systemcontrol` 是独立的 actuator，与 `safe_skin_control` 共用同一个 `RobotController`、
29999 连接和命令锁，避免两张卡互相抢占 TCP 连接。两张卡分别保存 `last_request/last_error`，
互不覆盖；切换控制器地址/反馈端口时关闭命令连接并清空两张卡的请求记录。

| action | 参数 | SDK / 行为 |
|---|---|---|
| `info` | 无 | 平台内部卡片介绍及参数说明 |
| `get_status` | 无 | 查询实际使能/上电/运行/暂停/拖拽/碰撞/报警状态，另含原始状态、安全字节、队列编号及负载 |
| `get_errors` | 无 | `GetErrorID()`，返回 `error_ids` 数组；不清错 |
| `request_control` | `confirm_change=true` | `RequestControl()`；仅系统控制卡提供 |
| `power_on` | `confirm_change=true` | `PowerOn()`，仅给机械臂本体上电，不是控制柜开机 |
| `enable` | `confirm_change=true` | `EnableRobot()`，不覆盖已配置负载/偏心参数 |
| `disable` | `confirm_change=true` | `DisableRobot()` |
| `clear_error` | `confirm_change=true` | `ClearError()`，不自动使能或恢复运动 |
| `stop_motion` | `confirm_change=true` | `Stop()`，停止运动指令队列/工程 |
| `pause` | `confirm_change=true` | `Pause()`，仅运行队列/工程（模式 7），点动请用 `stop_motion` |
| `resume` | `confirm_change=true` | `Continue()`，显式恢复已暂停的运动队列 |
| `set_speed_factor` | `ratio=1..100`, `confirm_change=true` | `SpeedFactor(ratio)` |
| `start_drag` | `confirm_change=true` | `StartDrag()`，进入关节拖拽示教 |
| `stop_drag` | `confirm_change=true` | `StopDrag()`，退出拖拽示教 |

卡片生命周期 `start/config/info/stop` 不发送物理控制命令。`stop` 只关闭共享 Dashboard
连接，真正停止机械臂需要显式 `stop_motion`；关闭任意执行卡的连接不会取消已接受的队列。
下一次显式命令按需建立连接，不自动接管 TCP。反馈超时不阻止 `stop_motion/disable/stop_drag`
及 `get_errors` 尝试发送，但仍需要已配置 IP；这些 TCP 操作不能替代硬件急停。

其他操作要求新鲜反馈并在连接后再次校验。请求 TCP 控制要求未使能/下电，
且关闭手自动模式切换功能。上电要求模式 3，使能要求模式 4，
继续要求模式 10 且已使能；上电初始化期间不能越过模式检查立即使能。
上电/使能/继续/进入拖拽被报警、碰撞、皮肤触发、松抱闸等状态阻止。
清错只允许静止状态，不会代为消除故障原因，也不保证复位皮肤。
速度调整允许模式 4/5/7；每次写操作均要求布尔值 `confirm_change=true`。

写命令返回 `effect_verified=false`：ACK 不代表物理状态已改变，请在后续 `get_status` 中核对反馈。
`tcp_control_requested` 只记录本次连接期间请求过控制权，不是控制器当前控制模式的读回。
`get_status` 顶层提供 `enabled/powered_on/running/paused/dragging/collision/has_error/speed_scaling`，
并保留 `robot_status` 中的协议字段。`powered_on` 由模式推断：模式 3 为 false，
模式 2/4/5/6/7/8/10/11 为 true，报警、初始化或未知模式为 null。
反馈过期时 `available=false`，上述实时字段及 `robot_status` 全部为 null，
不会将旧状态或最近一次命令的目标值当作当前状态。
超时/连接失败不重放写命令，也不串联清错、使能、继续等动作。

## 网络与配置

Jetson 有线口接控制柜 LAN2：Jetson `192.168.200.10/24`，控制柜默认 `192.168.200.1`，
无需网关，Wi-Fi 可继续用于 SSH/互联网。

`DOBOT_CONTROLLER_IP`、`DOBOT_FEEDBACK_PORT`、`DOBOT_MODEL_NAME` 覆盖初始配置。
`config.yaml` 默认 IP 空，不配置时不联系控制器；部署片段默认 IP 为 LAN2。
`CONFIG_PATH` 可覆盖配置文件。`joints` 的共享配置提供 IP 和端口，变更时先关闭
Dashboard 和旧反馈连接、清空缓存，再连接新地址。IP 声明为敏感字段，不打包到共享 Solution。
支持平台保存配置及启动前自动下发的 `config` 动作；返回 `state="configured"`。
随后 `start` 无需重复传入 IP/端口；重复应用相同配置不会断开现有连接，非法配置保留原连接。

反馈端口 `30004/30005/30006` 默认周期为 8/200/1000ms；接收和过期超时至少覆盖三个周期。
默认 `30004/30005` 为 1 秒接收、2 秒过期，`30006` 均为 3 秒；支持配置更大值。
SDK 清理异常不会中断重连，诊断字段 `cleanup_error` 单独记录。

## 构建与检查

```bash
bash build.sh --mirror none dobot/cra
```

SDK 及构建工具固定版本，源码校验 SHA-256；构建机需访问 GitHub codeload。
部署使用 `/deploy/service.yml`、host 网络、平台 DDS loopback 配置，domain 42，
MCP 端口 `15746`，容器名 `embodied-dobot-cra`；无需主机 SDK 或 privileged。

```bash
python3 -m venv .venv
.venv/bin/pip install -r dobot/cra/build-requirements.txt
.venv/bin/pip install --no-build-isolation -r dobot/cra/requirements.txt pytest
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_dobot_cra_sensors.py tests/test_dobot_safe_skin.py tests/test_dobot_skeleton_structure.py tests/test_dobot_system_control.py -q
.venv/bin/python scripts/check_service_yml.py
```

## PR 验收

1. 升级镜像后确认只有 `joints` 一张 sensor、`safe_skin_control/systemcontrol` 两张执行卡和后台 model 资源。
   删除旧 `safe_skin` 卡，将其下游连线改接到 `safe_skin_control` 的 JSON 输出口。
2. 配置并启动 `joints`，确认六轴与实时角度一致，J4 前连杆可见直角而非斜杆；固定拐点不可控。
3. 核对 `safe_skin_control` JSON 输出的各分区标志；断网时应变为未知，重连后恢复。
4. 在受控现场停止机器人后，显式执行所需皮肤设置，对照厂家软件确认队列执行及实际配置。
   本地模拟测试不替代这一项硬件验证。
5. 本次开发验证不得自动在真机上关闭安全皮肤、使能或恢复运动。
6. 从 action 下拉框执行 `systemcontrol.get_status`，核对实际状态；`get_errors` 查询报警编号。
   `info` 只用于卡片介绍和端口发现。硬件动作须在受控现场单独验收。
