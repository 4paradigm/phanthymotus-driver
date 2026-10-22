# 云深处 S10 Pro 只读 Driver

在现有 PhanthyMotus 卡片框架中提供五张 sensor 卡：`state`、`joints`、`imu`、`odometry`、`lidar`。沿用画布自动生成的表单、启动/停止按钮及 JSON/点云视图，不增加独立网页。当前没有运动卡或低层控制入口。

## 卡片与数据

| 卡片 | 已验证的厂家数据源 | 平台输出 |
| --- | --- | --- |
| state 状态 | `/BATTERY_DATA`、`/MOTION_INFO` | `data/json` |
| joints 关节反馈 | `/JOINTS_DATA_10HZ`，实机 16 条记录 | `data/json` |
| imu | 标准 `/IMU` | `data/json` |
| odometry 里程计 | `/ODOM` | `data/json`，未声明 `motus.odom/1` |
| lidar 雷达点云 | `/LIDAR/POINTS` | `sensor/pointcloud` 加独立 `lidar_info` JSON 元数据 |

平台话题固定为 `/{namespace}/s10_pro/{card}`。默认 namespace 为 `s10_pro`，可在应用配置中指定独立设备名。`info` 即使未启动也返回固定端口；所有卡片默认空闲，只有画布启动后才订阅、发布。`stop` 只停止本卡输出和所属读取源；最后一张卡停止后，读取进程退出。

卡片支持 `info/start/stop/status/config`。设置可调整 `enabled`、`refresh_hz`（0.1–30 Hz）、`stale_after_s`（0.1–300 秒）；雷达还支持 `max_points`（100–40000，默认 6000）。配置由现有画布持久化并在重连时重放。刷新频率是应用限频，不是控制频率或已测得的传感器采样率。

原始数据和厂家 stamp 保留。新鲜度按本机接收单调时钟计算，不混用不同来源的时间尺度。未知的电池/关节单位、状态码含义保持未知，空 frame 不补成已知坐标系。没有新数据时显示 `no_data`，过期显示 `stale`，不将旧值标成在线。

## 两个进程分离 DDS

```text
厂家 domain 0 → reader.py（只有订阅）
                    │ 有界 latest-only NDJSON / base64 点云
                    ↓
             bridge.py / 五张卡片
                    │ platform domain 42 + loopback DDS profile
                    ↓
             Agent Core / 原有画布
```

平台进程只创建一套平台 DDS context，并在初始化前校验 loopback profile。读取子进程移除继承的平台 XML、domain 和相关发现环境，再使用配置明确指定的厂家 domain/profile；没有在一个 FastDDS 进程中切 XML。最多为每个源保留一条待发送记录，丢弃旧管道积压及上次启动的在途帧。丢失读取进程会自动重试，读不到数据时不会伪报连接。

点云读取实际字段偏移、数据类型、大小端、`point_step`、`row_step` 和行 padding，过滤无效坐标，均匀限点。输出为现有 renderer 所需的 `UInt8MultiArray`：小端 `[uint32 stride=16][uint32 N][N × float32 XYZI]`。不套用其他机器人的坐标翻转。画布现有显示轴映射不等于已核实的本机安装坐标；点云停止后可能留有最后一帧，是否实时以元数据的年龄和 fresh 为准。

## 本机原生运行

已检查的 NOS 为 Ubuntu 24.04.4 / ARM64 / Python 3.12 / ROS 2 Jazzy，安装 `drdds-ros2-msgs 1.0.23` 和 `drddslib 1.2.12`。建议首次部署使用这套原生消息包；AOS 的 SSH 环境为容器且当前搜索路径未发现 drdds。

在已准备好 Agent Core 与其 loopback profile 的 NOS 应用目录中：

```bash
source /opt/ros/jazzy/setup.bash
python3 deep_robotics/s10_pro/reader.py --config deep_robotics/s10_pro/config.yaml --self-check
python3 deep_robotics/s10_pro/preflight.py --config deep_robotics/s10_pro/config.yaml --public-contract

export PYTHONPATH="$PWD"
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml
export AGENT_CORE_URL=https://localhost:15678
python3 deep_robotics/s10_pro/main.py
```

`--self-check` / preflight 只检查导入和类型，不创建 ROS participant。运行主进程后通过 MCP 15745 由画布启动卡片；不要使用厂家的 `setup_ros2.sh` 或 `rl_deploy` 作为环境/连通性检查。

## 可选容器

Dockerfile 使用 Jazzy 基础镜像，并在镜像内编译固定版本的官方消息定义。构建上下文按仓库 `build.sh` 规则，将本目录内容与额外 `common/` 合并。`vendor/SOURCE.md` 记录来源与校验值，保留上游许可证。

该公开消息包与 NOS 安装包的版本号不同。部署容器前必须比较递归消息字段/类型签名，不以版本名推断兼容。不要把宿主 Jazzy 二进制挂到 Humble 容器。`deploy/service.yml` 是在完成签名检查和现场预检后使用的模板，不会自动启用。

## 验证状态

2026-10-22 已有独立订阅脚本在真机读到 7 类状态/传感器数据，证明原生读取链路可用；这不等于新 Driver 已通过真机验收。开发期间用户主动断开机械狗连接，后续新驱动持续运行与真实画布验收待恢复连接。

离线测试覆盖真实 MCP HTTP 传输、五张卡片配置与独立启停、并发取消与配置更新、未知/缺失/过期值、消息递归签名、16 条关节记录以及点云行布局与二进制契约。测试数据/假 ROS 不冒充现场数据。

当前本地与原生 ARM64/Jazzy 容器均通过 188 项针对性测试。ARM64 镜像已构建，reader 导入检查通过；`tests/s10_arm64_smoke.py` 在 `--network none` 隔离容器中验证了真实 ROS/DDS 的“合成状态发布 → 独立读取进程 → 平台 JSON/点云”、断流过期/恢复、单卡停止和末卡停止后进程退出。该脚本检查网卡列表，发现非 loopback 网卡会拒绝运行；它不是机械狗测试。

执行相关测试：

```bash
python3 -m pytest -q tests/test_s10_*.py
python3 scripts/check_service_yml.py deep_robotics/s10_pro
```

现场部署、十分钟数据观察、启停三轮、读取中断恢复和刷新保留的逐项要求及回退步骤见 [deploy/ACCEPTANCE.md](deploy/ACCEPTANCE.md)。本轮不包含运动控制、导航执行、SDK 授权、低层关节接管或厂家固件变更。
