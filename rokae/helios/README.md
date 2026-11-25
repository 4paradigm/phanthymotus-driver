# ROKAE Helios-2.1 驱动卡片

通过现有 iNex 和底盘 WebSocket 读取 20 轴关节及双电池状态（两张传感器卡可独立开始、停止）；`base_drive` 执行器卡经厂家底盘 SDK 做限时、低速的底盘移动，默认关闭，需部署时显式开启（见下文“底盘移动”）。不提供关节运动、使能、模式切换或充放电控制。

| 工具 | 类型 | 输出 |
|---|---|---|
| `joints` | sensor | 关节骨架和逐轴状态，默认 10 Hz |
| `battery` | sensor | 两块电池状态，默认 1 Hz |
| `model` | resource | 运行时读取本机 iNex 的 URDF，骨架渲染器自动调用 |
| `base_drive` | actuator | 底盘限时低速移动（前进/后退/旋转）与立即停车，见 [CHASSIS_MOVE_INTERFACE.md](CHASSIS_MOVE_INTERFACE.md) |

## 数据与单位

`joints` 只发布一条流 `/rokae_helios/state/joints`（`sensor/skeleton`）：每轴 `q`（rad）、`dq`（rad/s）、`motor_torque_nm`、`gravity_torque_nm`（Nm），按名称匹配 URDF；另含连接状态 `state` / `available`、`message`、数据年龄、接收时间和 `torque_status`。

| iNex `joint/state` 分组 | URDF 关节 | 轴数 |
|---|---|---|
| `pos.left_arm` | `left_joint_1..7` | 7 |
| `pos.right_arm` | `right_joint_1..7` | 7 |
| `pos.waist` | `body_joint_1..4` | 4 |
| `pos.head` | `head_joint_1..2` | 2 |

位置保持厂家原始弧度值，不改零位、符号或限位。独立 `joint/scope` 流提供电机反馈力矩和模型重力矩，单位 Nm；头部未报告重力矩时为 null。两路流分别判断过期，不宣称同步采样。电机反馈不是关节力矩传感器读数，尚未通过实际加载校验，详见 [TORQUE_INTERFACE.md](TORQUE_INTERFACE.md)。

`battery` 发布 `/rokae_helios/state/battery`（`data/json`），从底盘 `UiDatagram.robot_base.data_pool.multi_battery_info` 解码。输出采用与 G1 电池卡相同的扁平写法：所有值都是顶层字段，每块电池以 `battery_1_` / `battery_2_` 为前缀。官方 Core 的日志视图每个字段一行，监控页"最新"模式每个字段一格。示例（数值仅作格式说明）：

```json
{"battery_1_soc": 98, "battery_1_voltage": 53.26, "battery_1_current": 0,
 "battery_1_cell_vol_min": 3.515, "battery_1_cell_vol_max": 3.589,
 "battery_1_temperature_min": 25, "battery_1_temperature_max": 25, "battery_1_cycle": 10,
 "battery_1_remaining_capacity": 17346, "battery_1_full_charge_capacity": 17700, "battery_1_state": "online",
 "battery_2_soc": 98, "...": "battery_2_* 同上",
 "state": "online", "message": "两块电池状态正常", "age_ms": 250, "timestamp_ms": 1795165967411}
```

| 字段 | 单位 / 含义 |
|---|---|
| `battery_1_*` / `battery_2_*` | 前缀为厂家编号 1 / 2，尚未核对物理左右位置 |
| `battery_N_soc` | %；0 是有效数值 |
| `battery_N_voltage` | 总电压 V，由原始 mV 换算 |
| `battery_N_current` | 原始电流 mA，保留符号；不据此推断充放电 |
| `battery_N_cell_vol_min` / `_max` | 单体电压范围，V |
| `battery_N_temperature_min` / `_max` | 当前帧 BMS 最低 / 最高传感器温度，°C |
| `battery_N_cycle` | 循环次数 |
| `battery_N_remaining_capacity` / `battery_N_full_charge_capacity` | 剩余 / 当前满充容量，mAh，不是设计容量 |
| `battery_N_state` | `online`、`not_reported`、`invalid_soc`，或整体停止/过期状态 |
| `state` / `message` | 整体状态：`online`（两块有效）、`partial`（仅一块）、`idle`、`stale`、`reconnecting` 等 |
| `age_ms` | 距离最近一次有效电池报文的毫秒数 |

缺失值保留 null，不补零。不推断 SOH、设计容量或充电状态。单体电压尺度依据实机原始值和总压一致性，仍需厂家明确单位确认。BMS 版本字符串仍会解析校验，但不再发布。协议、来源及限制见 [BATTERY_INTERFACE.md](BATTERY_INTERFACE.md)。

关节默认 1 秒、电池默认 3 秒无有效报文即过期。断线、无效、过期或停止时清空相应数据；无关报文不刷新有效期。关节流的 `received_at_ms` 是 Driver 接收时间；`timestamp_ms` 是发布时刻。没有厂家采样时间，无法仅凭接收计数排除控制器内部数据冻结。

## 底盘移动 `base_drive`

与 AgileX Bunker 的 `base_drive` 同一契约：`move(vx, vy, vyaw, duration_s)` 必须给时长，到时自动停车；`stop_move` 立即停车（画布可见；Core 会隐藏系统动作 `stop`，`stop` 仍保留供项目停止时调用）；`info` 返回限速、SDK 连接与底盘异常等级。`x-completion` 经 Agent Core ACP 回报 completed / cancelled / error，`x-resource: base`。默认上限 0.2 m/s、15°/s、10 s（代码硬上限 0.5 m/s、30°/s，配置不能超过）；横移 `vy` 默认关闭。

速度经厂家 VMR AMR SDK 的第三方控速下发（`set_third_speed_ctrl` → 50 Hz `send_third_expected_speed` → 零速 → `close_third_speed_ctrl`），在独立 worker 进程内运行：每次移动自带截止时间，Driver 退出或 worker 被杀时底盘因收不到速度指令自动停车（厂家示例说明）。`info` 与每次移动结果附带里程计实测位移（前进/左移/转角）。

厂家 SDK 只有 CPython 3.12 版，而 ROS Humble 是 3.10：提供 SDK（`VMR_SDK_URL`）时，镜像同时安装固定版本、校验 SHA256 的独立 Python 3.12；未提供 SDK 时两者都不安装，镜像不增大（`/opt/sdk-python`，构建时先从 npmmirror 国内镜像、失败再从 GitHub 下载，均校验同一 SHA256），只给 SDK worker 使用，已在机器人只读实测可导入、连接并收到里程计。厂家 SDK 二进制不进 Git：参照 G1/Agibot A3/DJI 的做法，构建时由 Dockerfile 从项目 COS 下载并校验 SHA256（`VMR_SDK_URL` / `VMR_SDK_SHA256`），解压到镜像 `/opt/vmr-sdk`。未提供 URL 时镜像照常构建，卡片照常注册，`move` 明确报错。

**默认关闭**：移动功能需部署时显式开启（`HELIOS_CHASSIS_MOTION_ENABLED=1`；bot 测试脚本用 `curl -fsSL https://motus.phanthy.com/run-pr-image.sh | sudo HELIOS_CHASSIS_MOTION_ENABLED=1 bash -s -- <镜像>`，网页控制台部署则写入 `/opt/phanthy-motus/.env`），未开启时 `move` 被拒绝。底盘异常状态未知（worker 启动后尚未收到首条异常上报，或 SDK 无异常设备）时同样拒绝移动。

**舵轮转向**：底盘为舵轮，换方向时先转轮再行驶。`duration_s` 从里程计显示底盘沿指令方向实际走出 1 cm（或转过 1.5°）时计时（最多等 5 s，用于底盘休息后唤醒电机和舵轮转向；转轮时的速度尖峰不算），结果中 `start_delay_s` 为转向等待时间。

**休息唤醒**：底盘空闲后休息（`2/7`），唤醒时电机依次报 `2/1`（非使能）、`2/2`（刹车，短暂 1 级）再全部清除；仅在开始运动前、且只有这些电机状态时，最多容忍 2 s，其余情况照常停车。每次移动结果含 `diagnostics`，`info.last_move` 可查最近一次。

**前提**：底盘须在厂家界面建图、应用地图并重定位成功，且处于自动模式（非手动模式）。未定位时底盘刹车并报异常（311/4/2），`move` 会被拒绝并返回异常列表；手动模式下 SDK 速度被忽略。2026-11-24 已在现场看护下用本地预览镜像实测前进、后退、左右转、横移与 `stop_move`，详见接口文档。

## 画布与 Core 前端

在卡片共享配置中保存状态地址，再将 `joints` 和 `battery` 加入画布并启动。`model` 无需单独运行；首次请求时读取同一 iNex 地址的 `/api/robot_3d_model` 及其同源 `urdf_url`，核对 20 轴名称后在进程内缓存。没有把厂家模型打入镜像。模型服务暂不可用时明确返回错误，不影响关节和电池订阅启动。监控页从 `joints` 自动生成骨架。

官方 Core 无需修改即可使用：监控页按真实 Driver ID 读取 URDF 显示骨架，电池在日志视图每字段一行、"最新"模式每字段一格。已知差异：

- 官方骨架把 URDF 中全部连杆画成线段，包括头部 4 个相机、胸部深度相机和 4 个轮子的固定安装坐标；关节姿态本身取自实时数据。
- 厂家模型中 `head_link_2` 的坐标原点偏离头部中线 33 mm，使对称安装的头部相机连线看起来不对称。`model` 下发前把该原点沿 `head_joint_2` 的转轴（y）平移回中线，并对其子连杆和自身 visual/collision/inertial 做同量补偿：转轴、各相机的实际位置和运动学均不变。头部显示为正面上下两个相机（眼部深度相机、下巴 RGB）在中线、背面两个相机左右对称。模型不满足"绕 y 轴、无旋转偏置"条件时原样下发。
- 配置页"查看数据流"面板在官方 Core 中不读取 Driver 模型，显示的是通用默认人形，应以监控页为准。

`4paradigm/phanthymotus` 中另有可选的 Helios 显示适配分支（MCP ID 传递、URDF RPY 顺序修复、取景和人形外观），需单独评审，不是本 Driver 运行的前提。通用骨架可能保留失联前姿态，应结合骨架消息中的 `available` / `state` 判断在线状态。

## 配置

默认读取上身 `ws://192.168.0.161:8888/ws` 及底盘 `ws://192.168.0.100/ws/AGVInfo`。可通过共享配置 `state_url` / `battery_state_url`，或环境变量 `HELIOS_STATE_URL` / `HELIOS_BATTERY_STATE_URL` 覆盖。地址字段声明为敏感配置，避免打包 Solution 时公开私有地址。

`config.yaml` 控制发布、过期、连接和重连时间。MCP 仅监听 `127.0.0.1:15747`，ROS domain 42，复用 `common/vendor_runtime.py` 注册到 Core。WebSocket 只读订阅，不安装或运行厂家运动 SDK。

## 本地验证

仓库根目录安装本 Driver 的 requirements 及 pytest 后执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. python -m pytest -q tests/test_helios_joints.py tests/test_helios_battery.py tests/test_helios_torque.py tests/test_helios_model.py tests/test_helios_chassis_move.py
python scripts/check_service_yml.py rokae/helios
```

测试覆盖映射和单位、缺失和非有限值、独立过期、断线重连、停止再启动、握手取消、MCP 模型响应，以及电池连接不发送应用层控制消息。固定样本仅用于回归，不作为生产数据源。

## 构建与部署

使用仓库标准构建入口 `./build.sh rokae/helios`。Dockerfile 使用仓库通用的 `ros-base` 驱动基础镜像，使用系统 Python3 安装 aiohttp、PyYAML 和 Protobuf；不再携带 Core 应用及其虚拟环境。依赖安装使用 `--no-cache-dir`，源文件放在依赖层之后以便复用缓存。镜像只运行 Driver。

`deploy/service.yml` 为 bot 提取的标准 host 网络服务片段。Core 和 Driver 必须共享 loopback，并使用 domain 42 及同一 DDS loopback 配置；只改变 HTTP 地址不能解决 DDS 隔离。

当前调试机器人上的 Core 使用 bridge 网络。为支持该布局，提供 [deploy/compose.core-network.yml](deploy/compose.core-network.yml)，在 bot 生成的 Compose 文件之后合并，设置 `HELIOS_CORE_CONTAINER` 为实际 Core 容器名；该覆盖只调整 Driver 的网络命名空间。重建 Core 后必须重建 Driver 以跟随新容器。正式部署前应评审完整合并配置，并停止旧 Driver，避免双实例注册。

正式流程为个人 Fork 功能分支 → PR → `/request_bot_review` → bot 镜像和部署指令 → 真机画布验证 → Master / 老师验收 → 合并。已有手工调试部署不能替代 bot 镜像验证。此分支尚未完成正式构建、部署或验收，检查范围见 [VALIDATION.md](VALIDATION.md)。

模型来源见 [resource/SOURCE.md](resource/SOURCE.md)。本次不包含 STL、原厂 SDK 二进制、机器人访问令牌或机器专用的开机服务。
