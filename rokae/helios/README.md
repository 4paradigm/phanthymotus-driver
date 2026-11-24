# ROKAE Helios-2.1 状态卡片

通过现有 iNex 和底盘 WebSocket 读取 20 轴关节、双电池状态及相对平面位姿。三张传感器卡可独立开始、停止，不提供运动、使能、模式切换或充放电控制。

| 工具 | 类型 | 输出 |
|---|---|---|
| `joints` | sensor | 关节骨架和逐轴状态，默认 10 Hz |
| `battery` | sensor | 两块电池状态，默认 1 Hz |
| `odometry` | sensor | 相对位置 x/y、相对朝向 yaw，默认 10 Hz；停止再开始重新置零 |
| `model` | resource | 运行时读取本机 iNex 的 URDF，骨架渲染器自动调用 |

## 数据与单位

### 相对位姿

`odometry` 被动读取底盘 `UiDatagram.current_state.pose`，没有 online、heartbeat、
导航、速度或使能请求。第一条有效观测作为本轮参考原点：其位置为 `(0, 0)`、
朝向为 `0`；之后将地图坐标差旋转到起始朝向，x 向前、y 向左，yaw 为相对
起始朝向的角度（rad，归一化至 `[-pi, pi]`）。重复 start 不改原点；stop 后
再 start 才重新置零。短暂断线重连保留原点。

只提供位置和朝向，不计算速度、累计里程、轨迹或 TF。不把定位重置/跳变当作
真实行驶距离；此来源是底盘地图定位状态，不声称它是独立、连续的编码器里程计。
底盘切图、重定位后需重新开始本卡，不能保证跨定位重置的连续性。

- `/{namespace}/state/relative_pose`（`data/json`）：x/y/yaw 和基本连接、时效信息。
- `/{namespace}/state/odom`（`state/odom`）：`common/odom.py` 的 `motus.odom/1`。
  `info().odom_interface.provides=[]`，六个 twist 轴始终为 null；没有速度测量，
  不能据此判断速度为零。参考系为本卡定义的固定 `relative_start`，不是机器人
  当前机身系。标准 pose 的 z=0、roll/pitch=0 是明确的二维模型约定，不是高度测量。

**位置单位不得猜测。** 默认 `odometry_position_unit: unverified`，显示 x/y
保留原始尺度并标注 `source_unverified`，标准输出的 pose 为 null。厂家确认
本机位置单位后，在共享配置中选 `m` 或 `mm`，才发布米制标准位姿；单位改变会
重新开始订阅并置零。航向角的弧度约定依据已安装底盘页面的 `180*theta/pi` 显示
换算。没有把软件静止接收称作距离/角度精度验收。

停止、断线、无效或默认 1 秒无有效位姿时，显示坐标清空为 null。无关电池、
雷达报文和重复/回退的 source timeStamp 不刷新有效期。接收时间用本机单调时钟
判断过期；输出 `stamp_ms` 保留该次数据的接收时间，不因定时发布而刷新。原厂
timeStamp 单独保留为 source_stamp_ms，不宣称它等于硬件采集时刻或已完成跨设备
同步。新连接允许原厂时钟从新起点恢复。

协议投影、坐标定义、PR519 格式对应及验收边界见 [ODOMETRY_INTERFACE.md](ODOMETRY_INTERFACE.md)。

### 关节与电池

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

## 画布与 Core 前端

在卡片共享配置中保存状态地址，再将需要的 `joints`、`battery` 或 `odometry` 加入画布并启动。`model` 无需单独运行；首次请求时读取同一 iNex 地址的 `/api/robot_3d_model` 及其同源 `urdf_url`，核对 20 轴名称后在进程内缓存。没有把厂家模型打入镜像。模型服务暂不可用时明确返回错误，不影响传感器订阅启动。监控页从 `joints` 自动生成骨架。

官方 Core 无需修改即可使用：监控页按真实 Driver ID 读取 URDF 显示骨架，电池在日志视图每字段一行、"最新"模式每字段一格。已知差异：

- 官方骨架把 URDF 中全部连杆画成线段，包括头部 4 个相机、胸部深度相机和 4 个轮子的固定安装坐标；关节姿态本身取自实时数据。
- 厂家模型中 `head_link_2` 的坐标原点偏离头部中线 33 mm，使对称安装的头部相机连线看起来不对称。`model` 下发前把该原点沿 `head_joint_2` 的转轴（y）平移回中线，并对其子连杆和自身 visual/collision/inertial 做同量补偿：转轴、各相机的实际位置和运动学均不变。头部显示为正面上下两个相机（眼部深度相机、下巴 RGB）在中线、背面两个相机左右对称。模型不满足"绕 y 轴、无旋转偏置"条件时原样下发。
- 配置页"查看数据流"面板在官方 Core 中不读取 Driver 模型，显示的是通用默认人形，应以监控页为准。

`4paradigm/phanthymotus` 中另有可选的 Helios 显示适配分支（MCP ID 传递、URDF RPY 顺序修复、取景和人形外观），需单独评审，不是本 Driver 运行的前提。通用骨架可能保留失联前姿态，应结合骨架消息中的 `available` / `state` 判断在线状态。

## 配置

默认读取上身 `ws://192.168.0.161:8888/ws` 及底盘 `ws://192.168.0.100/ws/AGVInfo`。可通过共享配置 `state_url` / `battery_state_url` / `odometry_state_url`，或对应环境变量 `HELIOS_STATE_URL` / `HELIOS_BATTERY_STATE_URL` / `HELIOS_ODOMETRY_STATE_URL` 覆盖。地址字段声明为敏感配置，避免打包 Solution 时公开私有地址。

`config.yaml` 控制发布、过期、连接和重连时间。MCP 仅监听 `127.0.0.1:15747`，ROS domain 42，复用 `common/vendor_runtime.py` 注册到 Core。WebSocket 只读订阅，不安装或运行厂家运动 SDK。

## 本地验证

仓库根目录安装本 Driver 的 requirements 及 pytest 后执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. python -m pytest -q tests/test_helios_joints.py tests/test_helios_battery.py tests/test_helios_torque.py tests/test_helios_model.py tests/test_helios_odometry.py tests/test_odom_format.py
python scripts/check_service_yml.py rokae/helios
```

测试覆盖映射和单位、缺失和非有限值、独立过期、断线重连、停止再启动、握手取消、MCP 模型响应，以及电池连接不发送应用层控制消息。固定样本仅用于回归，不作为生产数据源。

## 构建与部署

使用仓库标准构建入口 `./build.sh rokae/helios`。Dockerfile 使用仓库通用的 `ros-base` 驱动基础镜像，使用系统 Python3 安装 aiohttp、PyYAML 和 Protobuf；不再携带 Core 应用及其虚拟环境。依赖安装使用 `--no-cache-dir`，源文件放在依赖层之后以便复用缓存。镜像只运行 Driver。

`deploy/service.yml` 为 bot 提取的标准 host 网络服务片段。Core 和 Driver 必须共享 loopback，并使用 domain 42 及同一 DDS loopback 配置；只改变 HTTP 地址不能解决 DDS 隔离。

当前调试机器人上的 Core 使用 bridge 网络。为支持该布局，提供 [deploy/compose.core-network.yml](deploy/compose.core-network.yml)，在 bot 生成的 Compose 文件之后合并，设置 `HELIOS_CORE_CONTAINER` 为实际 Core 容器名；该覆盖只调整 Driver 的网络命名空间。重建 Core 后必须重建 Driver 以跟随新容器。正式部署前应评审完整合并配置，并停止旧 Driver，避免双实例注册。

正式流程为个人 Fork 功能分支 → PR → `/request_bot_review` → bot 镜像和部署指令 → 真机画布验证 → Master / 老师验收 → 合并。已有手工调试部署不能替代 bot 镜像验证。此分支尚未完成正式构建、部署或验收，检查范围见 [VALIDATION.md](VALIDATION.md)。

模型来源见 [resource/SOURCE.md](resource/SOURCE.md)。本次不包含 STL、原厂 SDK 二进制、机器人访问令牌或机器专用的开机服务。
