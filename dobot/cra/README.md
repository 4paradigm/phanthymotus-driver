# Dobot CRA joints and system control

基于官方 V4 Python SDK 的 Jetson Docker 驱动。本版只提供 `joints` 与 `systemcontrol`
两张可见卡片，URDF 由后台 `model` 资源提供。SDK 版本见 [SDK.md](SDK.md)。

## 卡片

| 名称 | 类型 | 内容 |
|---|---|---|
| `joints` | sensor | 六轴实际角度驱动的 CR3A 结构骨架、逐关节转角 JSON，以及共享连接配置 |
| `systemcontrol` | actuator | 系统状态查询、上/下使能、清除报警、进入/退出拖拽示教 |

`model` 为渲染器内部读取的 resource，不是可见卡片。
本版不注册 `safe_skin` 或 `safe_skin_control`，不发布安全皮肤 topic，也不提供皮肤设置入口。
升级后请移除画布中的旧安全皮肤卡片及连线；不会自动修改已有画布。

## 关节输出

默认以 30Hz（约每 33ms）发布 `std_msgs/String` JSON，两个输出共用同一份反馈快照：

- `/dobot_cra/state/joints`，`sensor/skeleton`；仅包含 `timestamp_ms/available/joints`。
  每个关节包含 `name/q`，`q` 为弧度，关节名为 `joint1..joint6`。
- `/dobot_cra/state/joint_angles`，`data/json`；包含 `timestamp_ms/available` 和
  `joint1_deg` 至 `joint6_deg`，每个关节转角单独一行，单位为度。

JSON 输出示例（数值为示例）：

```json
{
  "timestamp_ms": 1793856449751,
  "available": true,
  "joint1_deg": 90,
  "joint2_deg": -30,
  "joint3_deg": 0,
  "joint4_deg": 0,
  "joint5_deg": 0,
  "joint6_deg": 0
}
```

断线或过期时 `available=false`，骨架 `joints=[]`，六个角度字段均为 `null`，不输出旧角度。
角度 JSON 不依赖 URDF 是否可用；型号不匹配时仅骨架停止提供关节位置。
输出不含 IP/端口、连接诊断、包计数、SDK 来源或冗余角度数组。
旧下游若依赖 `joint_names/angles_deg`，请改读 `joint1_deg` 至 `joint6_deg`。

## 卡片介绍与系统状态

两张卡片的内部 `info` 只返回介绍、参数 schema、配置 schema 和输入输出口定义。
`joints` 通过输出口持续发布数据，不提供业务 action。
`systemcontrol.get_status` 为可选 action，读取当前反馈和最近请求记录，不打开命令连接。

| 状态字段 | 含义 |
|---|---|
| `available`、`timestamp_ms` | 反馈是否有效、本次查询的 Unix 毫秒时间 |
| `powered_on` | 从机器人模式推断的本体上电状态；报警、初始化或未知模式为 `null` |
| `enabled`、`running`、`paused`、`dragging` | 当前使能、运行、暂停、拖拽状态 |
| `collision`、`has_error` | 当前碰撞及是否存在报警 |
| `speed_scaling`、`robot_status` | 速度比例及协议状态字段 |
| `dashboard_connected`、`last_request`、`last_error` | 驱动命令连接状态、最近请求及错误记录 |

反馈失效时，实时状态和 `robot_status` 均为 `null`。请求记录不等于实际状态；
上使能请求的 ACK 不会把 `enabled` 强行置为 true。
本版不提供独立的 `get_errors` 动作，`has_error` 表示是否存在报警，不是报警编号列表。

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

## 系统控制

| action | 参数 | SDK / 行为 |
|---|---|---|
| `get_status` | 无 | 读取系统状态和最近请求记录 |
| `enable` | `confirm_change=true` | `EnableRobot()`，沿用控制器已有负载设置 |
| `disable` | `confirm_change=true` | `DisableRobot()` |
| `clear_error` | `confirm_change=true` | `ClearError()`，不自动使能或恢复运动 |
| `start_drag` | `confirm_change=true` | `StartDrag()`，进入关节拖拽示教 |
| `stop_drag` | `confirm_change=true` | `StopDrag()`，退出拖拽示教 |

已移除 `get_errors`、`request_control`、`power_on`、`stop_motion`、`pause`、`resume` 和
`set_speed_factor`。后台也拒绝这些动作，不保留隐式调用入口。运动控制留待后续运动卡片。

`start/config/info/stop` 是平台内部生命周期动作。`start` 启动反馈，`config` 确认配置，
`stop` 仅关闭命令连接，不发送机器人停止运动指令。
本版不会申请 TCP 控制权或给本体上电，操作前须在厂家软件中完成相应准备。

写操作按需建立一条序列化的 SDK Dashboard 连接，每次都要求布尔值 `confirm_change=true`。
上使能仅允许模式 4，进入拖拽允许模式 4/5，均要求新鲜反馈、机器人停止，无报警、碰撞、
皮肤接近暂停或松抱闸；连接后再次检查状态。清错仅允许静止状态，不代为排除故障原因。
下使能和退出拖拽允许在反馈失效时尝试发送，但仍须已配置控制器 IP。
切换 IP/端口会关闭旧连接并清空请求记录。

返回 `effect_verified=false`：ACK 仅代表控制器的应答，请通过后续 `get_status` 核对效果。
超时或连接失败不会重放写命令，不自动串联清错、使能或恢复运动。

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
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest tests/test_dobot_cra_sensors.py tests/test_dobot_skeleton_structure.py tests/test_dobot_system_control.py -q
.venv/bin/python scripts/check_service_yml.py
```

## PR 验收

1. 卡片列表仅有 `joints` 和 `systemcontrol`；`model` 保留为后台资源。
2. `joints` 的骨架使用六轴实际角度，JSON 的六个角度字段分别占一行。
3. 系统卡 action 仅有状态查询、上/下使能、清错和进入/退出拖拽；移除动作不能通过直接 API 调用。
4. `get_status` 在反馈失效时返回未知状态，`info` 只返回卡片介绍。
5. 本地模拟验证不操作真机，硬件操作由现场单独验收。
