# 云深处 S10 Pro Driver

在现有 PhanthyMotus 卡片框架中提供五张 sensor 卡：`state`、`joints`、`imu`、`odometry`、`lidar`，以及一张执行锁定的 `motion` actuator 卡。沿用现有画布的表单、按钮和 JSON/点云视图，没有低层关节控制入口。

## 本次提交与现场版本

本次提交接入了高层 DDS 动作后端、独立命令进程和运动反馈确认。生产入口的 `DDSProcessTransport.execution_enabled`、`DDSMotionBackend.execution_authorized`、`MotionPlugin.execution_authorized` 均固定为 `False`；画布配置不能修改这些授权。启动动作卡只订阅运动反馈，不启动命令 worker、不创建运动命令 publisher，也不自动切换机器人模式。动作调用返回 `unavailable`。

暂停开发时，现场仍运行六卡锁定镜像 `s10-pro-motion:53e517db`，镜像 ID 为 `sha256:0f3959dd8e3bdaae184591473589dff7e9456e2551d1d4d544fe0c7c9f61db14`，运行代码提交为 `53e517db`。本次新增的 DDS 后端候选尚未替换现场驱动。新链路已在 ARM64 Docker 的隔离网络内完成真实 ROS 2 合成冒烟：起立按反馈依次发送 `1 → 17`，限时速度到期后归零并确认停止，趴下获得反馈确认。该结果不是机器人动作验收；本次 PR 的检查结果另见 PR 描述。

## 卡片与数据

| 卡片 | 已验证的厂家数据源 | 平台输出 |
| --- | --- | --- |
| state 状态 | `/BATTERY_DATA`、`/MOTION_INFO` | `data/json` |
| joints 关节反馈 | `/JOINTS_DATA_10HZ`，实机 16 条记录 | `data/json` |
| imu | 标准 `/IMU` | `data/json` |
| odometry 里程计 | `/ODOM` | `data/json`，未声明 `motus.odom/1` |
| lidar 雷达点云 | `/LIDAR/POINTS` | `sensor/pointcloud` 加独立 `lidar_info` JSON 元数据 |

平台话题固定为 `/{namespace}/s10_pro/{card}`。默认 namespace 为 `s10_pro`，可在应用配置中指定独立设备名。`info` 即使未启动也返回固定端口；所有卡片默认空闲，只有画布启动后才订阅、发布。`stop` 只停止本卡输出及所属读取源；所有传感器卡和动作状态观察均停止后，读取进程退出。

传感器卡支持 `info/start/stop/status/config`。设置可调整 `enabled`、`refresh_hz`（0.1–30 Hz）、`stale_after_s`（0.1–300 秒）；雷达还支持 `max_points`（100–40000，默认 6000）。配置由现有画布持久化并在重连时重放。刷新频率是应用限频，不是控制频率或已测得的传感器采样率。

原始数据和厂家 stamp 保留。新鲜度按本机接收单调时钟计算，不混用不同来源的时间尺度。未知的电池/关节单位、状态码含义保持未知，空 frame 不补成已知坐标系。没有新数据时显示 `no_data`，过期显示 `stale`，不将旧值标成在线。

## 基础动作卡

`motion` 提供起立 `stand`、趴下 `lie`、限时速度 `velocity` 和本方停止 `stopmotion`。`start` 只启用状态观察；生命周期 `stop` 取消本方请求并停止本卡输出。固定输出是 `motion_status`、`motion_events`，没有连续控制输入。

实现包含互斥、取消优先、过期请求拒绝、有限时速度和限幅、控制权与反馈新鲜度检查。请求接受、实际反馈确认和完成分开记录；DDS 发布没有业务 ACK，传输层保留 `accepted=null`。停止为本应用持续零速与实际停止反馈确认，不是硬急停。

高层 DDS 路线依据[官方 S10 软件开发指南 V1.0.2](https://support.deeprobotics.cn/docs/#/259676015)和用户提供的[钉钉软件开发文档](https://alidocs.dingtalk.com/i/p/OlnXRnDldg3J7XLp/docs/dQPGYqjpJYgQaZdQtBNLa6a6Wakx1Z5N?dontjump=true)，使用 `/MOTION_STATE`、`/NAV_CMD` 和 `/MOTION_INFO`。起立必须先确认状态 `1`，再请求状态 `17`，不能将 ASDU 的自动转换规则套到 DDS。速度使用物理单位并要求明确的导航模式授权。命令进程独立于平台 DDS，只有显式进程授权且逐次控制权、取消状态和期限检查通过后才创建 publisher。通信超时会关闭命令进程，不自动重试动作。

当前默认 DDS 路线不需要 TLS 证书。保留的 `motion_transport.py` 是可选 ASDU/TLS 实验实现：严格校验 16 字节版本 1 帧头、JSON、有界帧和请求关联，连接要求 CA、主机名和独立证书指纹；生产入口未选择这条路线。此前仅诊断过 AOS 30003 的 TLS 1.3 证书链，未发送应用帧。

用户提供的手柄显示版本为 `v2.5.5`，尚未确定它对应系统、APP 还是控制服务版本。公开指南没有建立这台机器的完整控制权/手柄接管优先级，也没有确认速度指令失联后的机器人停止时限；2 秒心跳超时只表示停止状态上报，不能当作速度看门狗。`/MOTION_INFO` 的可见状态不能代替控制权、导航模式或失联停止保证。真机执行仍需匹配本机版本和身份、可验证的控制契约、现场准备及接管条件，当前未开放。

[GitHub SDK](https://github.com/DeepRoboticsLab/sdk_deploy)和[GitCode SDK](https://gitcode.com/DEEPRoboticsLab/sdk_deploy)主要用于低层运动算法/关节控制。其初始化路径会产生关节复位等命令，不能作为连通性或环境检查直接试跑。本驱动不调用该低层入口，也不复用 M20 的控制编码。

## DDS 进程分离

```text
厂家 domain 0 → reader.py（只有订阅）
                    │ 有界 latest-only NDJSON / base64 点云
                    ↓
             bridge.py / 传感器卡与动作反馈观察
                    │ platform domain 42 + loopback DDS profile
                    ↓
             Agent Core / 原有画布

动作请求 → 独立 motion_worker.py → 厂家 domain 0
           当前生产入口禁止启动，仅在隔离测试中显式启用
```

平台进程只创建一套平台 DDS context，并在初始化前校验 loopback profile。读取子进程移除继承的平台 XML、domain 和相关发现环境，再使用配置明确指定的厂家 domain/profile；没有在一个 FastDDS 进程中切 XML。最多为每个源保留一条待发送记录，丢弃旧管道积压及上次启动的在途帧。丢失读取进程会自动重试，读不到数据时不会伪报连接。命令 worker 使用独立的有界请求/响应管道，读取进程的自动恢复不会重放命令。

点云读取实际字段偏移、数据类型、大小端、`point_step`、`row_step` 和行 padding，过滤无效坐标，均匀限点。输出为现有 renderer 所需的 `UInt8MultiArray`：小端 `[uint32 stride=16][uint32 N][N × float32 XYZI]`。不套用其他机器人的坐标翻转。画布显示轴映射不等于已核实的本机安装坐标；点云停止后可能留有最后一帧，是否实时以元数据的年龄和 fresh 为准。

## Docker 部署

2026-10-22 现场采用同一 NOS 上的两个独立 Docker 容器：Agent Core 使用已有 Humble 应用镜像，driver 使用 Jazzy 镜像。应用目录为 `/userdata/phanthy-motus-s10`，画布入口为 [S10 Pro Agent Core](https://10.21.33.106:15678)。访问凭据保存在独立应用配置中，不进入仓库。

Dockerfile 在镜像内编译固定版本的官方消息定义。构建上下文按仓库 `build.sh` 规则，将本目录内容与额外 `common/` 合并。`vendor/SOURCE.md` 记录来源与校验值，保留上游许可证。公开消息包与 NOS 安装包 `drdds-ros2-msgs 1.0.23` 的版本号不同；现场递归字段/类型比较已通过，本次 DDS 候选涉及的九种消息类型也已与宿主匹配。更换镜像或厂家消息包后仍须重新比较，不能只依据包版本名推断兼容，也不能将宿主 Jazzy 二进制挂入 Humble 容器。

两个容器使用 host network，平台 domain 42 共同加载 loopback-only `dds-local.xml`。driver 的独立 reader 子进程通过 [deploy/s10-reader-udp.xml](deploy/s10-reader-udp.xml) 只启用 UDPv4，读取厂家 domain 0。默认运输配置下容器可读取 drdds 状态，但未收到本机标准 IMU、里程计与点云；单独启用 host IPC 没有解决，子进程使用自有 UDP-only XML 后三类标准消息均可读取。现场保留 private IPC，不要求 `ipc: host`。

NOS 当前没有 Docker Compose 插件，现场容器由 Docker CLI 管理；应用目录中的 `docker-compose.yml` 用于持久化记录服务配置。[deploy/service.yml](deploy/service.yml) 是平台安装模板。按团队 PR 流程生成镜像后，应使用机器人实际具备的工具部署并重新验收该版本。

外部挂载的应用 `config.yaml` 应显式设置：

```yaml
reader:
  domain_id: 0
  vendor_profile: "/opt/phanthy-motus/s10-reader-udp.xml"
```

`deploy/service.yml` 使用平台惯例的 `/opt/phanthy-motus` 宿主路径。现场配置的只读挂载如下：

```yaml
volumes:
  - /userdata/phanthy-motus-s10/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro
  - /userdata/phanthy-motus-s10/s10-reader-udp.xml:/opt/phanthy-motus/s10-reader-udp.xml:ro
  - /userdata/phanthy-motus-s10/config.yaml:/work/deep_robotics/s10_pro/config.yaml:ro
```

启动前确认两份 XML 均为真实文件。应用自有 UDP-only XML 仅供厂家域子进程加载；平台父进程继续使用 loopback profile。不修改 `/opt/robot/fastdds.xml`，不运行会修改它的厂家 `setup_ros2.sh`。默认仓库配置保留 `motion.enabled: false`，部署时使用独立的外部应用配置。

## 只读诊断与验证

已检查的 NOS 为 Ubuntu 24.04.4 / ARM64 / Python 3.12 / ROS 2 Jazzy，安装 `drdds-ros2-msgs 1.0.23` 和 `drddslib 1.2.12`。原生环境可用于独立诊断，现场应用仍采用 Docker。AOS 的 SSH 环境为容器，已检查的搜索路径未发现 drdds。

```bash
source /opt/ros/jazzy/setup.bash
python3 deep_robotics/s10_pro/reader.py --config deep_robotics/s10_pro/config.yaml --self-check
python3 deep_robotics/s10_pro/preflight.py --config deep_robotics/s10_pro/config.yaml --public-contract
python3 deep_robotics/s10_pro/motion_preflight.py --public-contract
```

这些导入/类型检查不创建 ROS participant；`motion_preflight.py --graph` 可另做只读图探测，不创建命令 publisher。不要使用厂家的 `setup_ros2.sh` 或 `rl_deploy` 作为环境检查。

历史五卡镜像 `s10-pro-readonly:3a1cb138` 已完成：五卡真实 fresh 数据、30 秒 Core WebSocket 数据通路、三轮独立启停和 reader 中断恢复。连续 600.09 秒的 Core/Humble DDS 观察收到 599 状态、2946 关节、2970 IMU、2966 里程计、576 点云及 1194 雷达元数据帧；JSON 均 fresh，每帧点云 6000 点，无解析错误。这些是对应旧版本的验收记录，不表示本次候选已完成同样的现场验收。

随后部署的 `53e517db` 六卡锁定版本通过了五卡真实数据及动作 locked 状态通道检查。浏览器控制仍在 Work settings 中关闭，尚未验收真实画布点云渲染、截图和刷新保留。没有发送过真机起立、趴下、速度、模式切换、复位或急停命令。

离线测试覆盖真实 MCP HTTP、卡片配置与并发启停、旧值/缺失值/时钟异常、点云二进制、递归消息类型，以及动作许可、期限、取消和反馈。`tests/s10_arm64_smoke.py` 与 `tests/s10_motion_dds_smoke.py` 只能在 `--network none` 容器中运行；脚本发现非 loopback 网卡会拒绝执行。合成发布者和 fake backend 不代表机器人侧控制权或失联停止保证。

```bash
python3 -m pytest -q tests/test_s10_*.py
python3 scripts/check_service_yml.py deep_robotics/s10_pro
```

部署、分版本验收和应用回退要求见 [deploy/ACCEPTANCE.md](deploy/ACCEPTANCE.md)。
