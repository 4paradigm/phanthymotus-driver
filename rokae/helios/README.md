# ROKAE Helios-2.1 状态卡片

通过现有 iNex 和底盘 WebSocket 读取 20 轴关节及双电池状态。两张传感器卡可独立开始、停止，不提供运动、使能、模式切换或充放电控制。

| 工具 | 类型 | 输出 |
|---|---|---|
| `joints` | sensor | 关节骨架和逐轴状态，默认 10 Hz |
| `battery` | sensor | 两块电池状态，默认 1 Hz |
| `model` | resource | 运行时读取本机 iNex 的 URDF，骨架渲染器自动调用 |

## 数据与单位

`joints` 发布两条流：

- `/rokae_helios/state/joints`（`sensor/skeleton`）：`q` 为 rad，`dq` 为 rad/s，按名称匹配 URDF。
- `/rokae_helios/state/joint_status`（`data/json`）：`angles_rad`、`velocities_rad_s`、`motor_torques_nm`、`gravity_torques_nm`，以及连接状态、数据年龄、单位和接收时间。

| iNex `joint/state` 分组 | URDF 关节 | 轴数 |
|---|---|---|
| `pos.left_arm` | `left_joint_1..7` | 7 |
| `pos.right_arm` | `right_joint_1..7` | 7 |
| `pos.waist` | `body_joint_1..4` | 4 |
| `pos.head` | `head_joint_1..2` | 2 |

位置保持厂家原始弧度值，不改零位、符号或限位。独立 `joint/scope` 流提供电机反馈力矩和模型重力矩，单位 Nm；头部未报告重力矩时为 null。两路流分别判断过期，不宣称同步采样。电机反馈不是关节力矩传感器读数，尚未通过实际加载校验，详见 [TORQUE_INTERFACE.md](TORQUE_INTERFACE.md)。

`battery` 发布 `/rokae_helios/state/battery`（`data/json`），从底盘 `UiDatagram.robot_base.data_pool.multi_battery_info` 解码：

| 字段 | 单位 / 含义 |
|---|---|
| `id` | 厂家编号 1 / 2，尚未核对物理左右位置 |
| `soc_pct` | %；0 是有效数值 |
| `voltage_v` | V，由原始 mV 换算 |
| `current_ma` | 原始电流 mA，保留符号；当前展示不显示此字段 |
| `temperature_min_c` / `temperature_max_c` | 当前帧 BMS 最低 / 最高温度，°C |
| `cell_voltage_min_v` / `cell_voltage_max_v` | 单体电压范围，V |
| `remaining_capacity_mah` / `full_charge_capacity_mah` | 剩余 / 满充容量，mAh，不是设计容量 |
| `cycle_count` | 循环次数 |
| `bms_hardware_version` / `bms_software_version` | 原始 BMS 版本字符串 |

两块 SOC 均有效时 `complete=true`；仅一块有效为 `partial`。缺失值保留 null，不补零。不推断 SOH、设计容量或充电状态。单体电压尺度依据实机原始值和总压一致性，仍需厂家明确单位确认。协议、来源及限制见 [BATTERY_INTERFACE.md](BATTERY_INTERFACE.md)。

关节默认 1 秒、电池默认 3 秒无有效报文即过期。断线、无效、过期或停止时清空相应数据；无关报文不刷新有效期。`received_at_ms` 是 Driver 接收时间，`timestamp_ms` 是发布时刻，`source_timestamp=null`。没有厂家采样时间，无法仅凭接收计数排除控制器内部数据冻结。

## 画布与 Core 前端

在卡片共享配置中保存状态地址，再将 `joints` 和 `battery` 加入画布并启动。`model` 无需单独运行；首次请求时读取同一 iNex 地址的 `/api/robot_3d_model` 及其同源 `urdf_url`，核对 20 轴名称后在进程内缓存。没有把厂家模型打入镜像。模型服务暂不可用时明确返回错误，不影响关节和电池订阅启动。监控页从 `joints` 自动生成骨架及 `joint_status` 模块。

配套 Core 功能分支包含真实 MCP ID 传递、URDF RPY 修复、首帧回放和自动取景，以及 Helios 的显示适配：

- 上半身采用类似 G1 的肩线、短颈和圆点头部，保留全部原始运动学坐标。示意外观不能用于量取头肩位置，不加载 STL。
- 双电池沿用原生英文日志及字体，同帧共用一个接收时间；展示 SOC、电压、单体电压范围、当前最高传感器温度及循环次数。
- 容量、BMS 版本和诊断放入 Details。展开历史详情时保留记录及阅读位置，收起后恢复跟随。
- 当前界面隐藏 current、无来源的 SOH 和未知充电状态。原始 Driver 数据合约保留电流。

Core 变更需要在 `4paradigm/phanthymotus` 单独提交评审；仅更新 Driver 镜像不会安装这些前端变更。通用骨架可能保留失联前姿态，应结合 `available` / `state` 判断在线状态。

## 配置

默认读取上身 `ws://192.168.0.161:8888/ws` 及底盘 `ws://192.168.0.100/ws/AGVInfo`。可通过共享配置 `state_url` / `battery_state_url`，或环境变量 `HELIOS_STATE_URL` / `HELIOS_BATTERY_STATE_URL` 覆盖。地址字段声明为敏感配置，避免打包 Solution 时公开私有地址。

`config.yaml` 控制发布、过期、连接和重连时间。MCP 仅监听 `127.0.0.1:15747`，ROS domain 42，复用 `common/vendor_runtime.py` 注册到 Core。WebSocket 只读订阅，不安装或运行厂家运动 SDK。

## 本地验证

仓库根目录安装本 Driver 的 requirements 及 pytest 后执行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. python -m pytest -q tests/test_helios_joints.py tests/test_helios_battery.py tests/test_helios_torque.py tests/test_helios_model.py
python scripts/check_service_yml.py rokae/helios
```

测试覆盖映射和单位、缺失和非有限值、独立过期、断线重连、停止再启动、握手取消、MCP 模型响应，以及电池连接不发送应用层控制消息。固定样本仅用于回归，不作为生产数据源。

## 构建与部署

使用仓库标准构建入口 `./build.sh rokae/helios`。Dockerfile 使用仓库通用的 `ros-base` 驱动基础镜像，使用系统 Python3 安装 aiohttp、PyYAML 和 Protobuf；不再携带 Core 应用及其虚拟环境。依赖安装使用 `--no-cache-dir`，源文件放在依赖层之后以便复用缓存。镜像只运行 Driver。

`deploy/service.yml` 为 bot 提取的标准 host 网络服务片段。Core 和 Driver 必须共享 loopback，并使用 domain 42 及同一 DDS loopback 配置；只改变 HTTP 地址不能解决 DDS 隔离。

当前调试机器人上的 Core 使用 bridge 网络。为支持该布局，提供 [deploy/compose.core-network.yml](deploy/compose.core-network.yml)，在 bot 生成的 Compose 文件之后合并，设置 `HELIOS_CORE_CONTAINER` 为实际 Core 容器名；该覆盖只调整 Driver 的网络命名空间。重建 Core 后必须重建 Driver 以跟随新容器。正式部署前应评审完整合并配置，并停止旧 Driver，避免双实例注册。

正式流程为个人 Fork 功能分支 → PR → `/request_bot_review` → bot 镜像和部署指令 → 真机画布验证 → Master / 老师验收 → 合并。已有手工调试部署不能替代 bot 镜像验证。此分支尚未完成正式构建、部署或验收，检查范围见 [VALIDATION.md](VALIDATION.md)。

模型来源见 [resource/SOURCE.md](resource/SOURCE.md)。本次不包含 STL、原厂 SDK 二进制、机器人访问令牌或机器专用的开机服务。
