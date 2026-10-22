# 云深处 S10 Pro Driver

在现有 PhanthyMotus 卡片框架中提供五张 sensor 卡：`state`、`joints`、`imu`、`odometry`、`lidar`。另有 `motion` 基础动作卡，真机执行仍锁定，界面启用无法解除锁。沿用画布现有表单、按钮及 JSON/点云视图，没有低层关节控制入口。

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

## 基础动作卡

`motion` 是 actuator 卡，提供起立 `stand`、趴下 `lie`、限时速度 `velocity` 和本方停止 `stopmotion`。`start` 只启用卡片状态，生命周期 `stop` 取消本方请求并停止本卡输出。固定输出是 `motion_status`、`motion_events`，没有连续控制输入。

实现包含互斥、取消优先、过期请求拒绝、有限时速度和限幅、控制权与反馈新鲜度检查。请求接受、实际反馈确认和完成分开记录；速度协议没有 ACK，传输层保留 `accepted=null`。停止为本应用持续零速与实际停止反馈确认，不是硬急停。

依据为[官方 S10 软件开发指南 V1.0.2](https://support.deeprobotics.cn/docs/#/259676015)，明确涵盖 STD/PRO，不能套用 M20 编码。传输层实现严格 16 字节版本 1 帧头、JSON、有界帧和请求关联。TLS 默认不连接，启用时要求 CA、主机名校验和独立证书指纹，没有明文或跳过证书验证的业务回退。

本机 AOS 30003 提供 TLS 1.3，但实际证书链不同于 NOS 设备链。已收集公开链做诊断，尚未发送任何应用帧。用户提供手柄显示 `v2.5.5`，仍需区分系统/APP 版本并对照控制服务。厂家指南未明确完整接管优先级和失联停止保证，2 秒心跳超时只表示停止状态上报，不能当作速度看门狗。

生产构造器固定 `execution_authorized=False`，未绑定真实控制后端；动作调用返回 `unavailable`，不发送运动。解除锁需要本机版本对应的 TLS 信任材料、控制权/接管与失联停止确认，以及现场准备和人工接管确认。离线 fake backend 测试不代表动作真机验收。

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

## 已部署的 Docker 运行方式

2026-10-22 现场采用同一 NOS 上的两个独立 Docker 容器：Agent Core 使用已有 Humble 应用镜像，driver 使用 Jazzy 镜像。应用目录为 `/userdata/phanthy-motus-s10`，画布入口为 [S10 Pro Agent Core](https://10.21.33.106:15678)。访问认证沿用该独立应用配置，仓库不保存访问凭据。

已部署 driver 镜像为 `s10-pro-readonly:3a1cb138`，镜像 ID 为 `sha256:ffdaa0eb9187299028d63f2fde87de842cb2fe8e129e6c56bc10617de271dc28`。当前运行代码对应提交 `3a1cb138`；下述运输层修复通过应用配置和只读挂载生效，没有替换镜像内 Python 代码。

Dockerfile 使用 Jazzy 基础镜像，并在镜像内编译固定版本的官方消息定义。构建上下文按仓库 `build.sh` 规则，将本目录内容与额外 `common/` 合并。`vendor/SOURCE.md` 记录来源与校验值，保留上游许可证。该公开消息包与 NOS 安装包 `drdds-ros2-msgs 1.0.23` 的版本号不同；现场递归消息字段/类型签名比较结果为 `compatible: true`。后续换镜像或厂家消息包时仍需重新比较，不能依据包版本名推断兼容，也不能将宿主 Jazzy 二进制挂入 Humble 容器。

两个容器使用 host network，平台 domain 42 共同加载 loopback-only `dds-local.xml`。driver 的独立 reader 子进程通过 [deploy/s10-reader-udp.xml](deploy/s10-reader-udp.xml) 只启用 UDPv4，读取厂家 domain 0。现场发现默认运输配置下容器能够读取 drdds 状态，但不能读取本机的标准 IMU、里程计与点云；单独启用 host IPC 没有解决，reader 使用自有 UDP-only XML 后三类标准消息均可读取。因此现场保留 private IPC，不要求 `ipc: host`。

NOS 当前没有 Docker Compose 插件，两个容器实际由 Docker CLI 创建和管理；应用目录中的 `docker-compose.yml` 用于持久化记录服务配置。`deploy/service.yml` 是平台安装模板，不能将现场记录直接当作已可执行的 `docker compose` 工作流。

在外部挂载的应用 `config.yaml` 中显式设置：

```yaml
reader:
  domain_id: 0
  vendor_profile: "/opt/phanthy-motus/s10-reader-udp.xml"
```

`deploy/service.yml` 使用平台惯例的 `/opt/phanthy-motus` 宿主路径。现场独立应用配置中记录以下只读挂载；目标路径相同，宿主来源位于持久化目录：

```yaml
volumes:
  - /userdata/phanthy-motus-s10/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro
  - /userdata/phanthy-motus-s10/s10-reader-udp.xml:/opt/phanthy-motus/s10-reader-udp.xml:ro
  - /userdata/phanthy-motus-s10/config.yaml:/work/deep_robotics/s10_pro/config.yaml:ro
```

启动前确认两份 XML 均为真实文件。UDP-only XML 仅由新应用的 reader 子进程加载；平台父进程继续使用 loopback profile，不修改 `/opt/robot/fastdds.xml`，也不执行会修改它的厂家 `setup_ros2.sh`。默认仓库 `config.yaml` 保持不变，部署时使用外部应用配置覆盖。

## 可选的本机原生运行

已检查的 NOS 为 Ubuntu 24.04.4 / ARM64 / Python 3.12 / ROS 2 Jazzy，安装 `drdds-ros2-msgs 1.0.23` 和 `drddslib 1.2.12`。原生运行仍可用于开发或独立诊断；本次现场运行采用上面的两容器方式。AOS 的 SSH 环境为容器且已检查的搜索路径未发现 drdds。

在已准备好 Agent Core、平台 profile 和应用配置的 NOS 应用目录中：

```bash
source /opt/ros/jazzy/setup.bash
python3 deep_robotics/s10_pro/reader.py --config deep_robotics/s10_pro/config.yaml --self-check
python3 deep_robotics/s10_pro/preflight.py --config deep_robotics/s10_pro/config.yaml --public-contract

export PYTHONPATH="$PWD"
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=/userdata/phanthy-motus-s10/dds-local.xml
export AGENT_CORE_URL=https://localhost:15678
python3 deep_robotics/s10_pro/main.py
```

`--self-check` / preflight 只检查导入和类型，不创建 ROS participant。运行主进程后通过 MCP 15745 由画布启动卡片；不要使用厂家的 `setup_ros2.sh` 或 `rl_deploy` 作为环境/连通性检查。

## 验证状态

2026-10-22 已在 NOS 完成独立 Core 与 driver 两容器部署，五卡布局已保存。已完成的现场检查如下：

- 五卡 MCP/API 持续检查返回真实 fresh 数据；HTTPS API 的 21 项审计通过，未认证请求返回 401。独立应用已配置认证，自动项目启动关闭，审计时 `project_running=false`。
- Core 的 30 秒 WebSocket 检查六条输出均通过：状态约 1 Hz，关节/IMU/里程计约 4.95 Hz，雷达元数据约 2 Hz；雷达二进制收到 27 帧，约 0.90 Hz，每帧 6000 点，最大帧间隔约 3.01 秒。该结果验证 Jazzy driver → Humble Core DDS → WebSocket 的数据通路，不等于浏览器已经渲染。输出频率不是厂家原始传感器采样率。
- 三轮独立启停通过：停止一张卡后其 DDS 输出停止，其余仍启动的卡继续；末卡停止后 reader 进程与六源订阅均为 0，重新启动后 reader 为 1 个、每源订阅为 1 个。
- 仅暂停新应用 reader 子进程时五卡均转 stale，雷达二进制停止而元数据继续；恢复该进程后数据恢复。终止 reader 后自动重启并恢复 fresh，厂家进程未被停止。

最终连续 600.09 秒的 Core/Humble DDS 观察通过：599 状态帧、2946 关节帧、2970 IMU 帧、2966 里程计帧、576 点云帧及 1194 雷达元数据帧；所有 JSON 帧均 fresh，点云每帧 6000 点，无解析错误。浏览器操作在 Work settings 中关闭，尚未进行真实浏览器点云渲染、截图和刷新保留验收。

离线测试覆盖真实 MCP HTTP 传输、五张卡片配置与独立启停、并发取消与配置更新、未知/缺失/过期值、消息递归签名、16 条关节记录以及点云行布局与二进制契约。测试数据/假 ROS 不冒充现场数据。

五卡版本在本地与原生 ARM64/Jazzy 容器均通过 188 项针对性测试。加入 motion 后本地 305 项通过，ARM64 新版结果另行记录。ARM64 镜像已构建，reader 导入检查通过；`tests/s10_arm64_smoke.py` 在 `--network none` 隔离容器中验证了真实 ROS/DDS 的“合成状态发布 → 独立读取进程 → 平台 JSON/点云”、断流过期/恢复、单卡停止和末卡停止后进程退出。该脚本检查网卡列表，发现非 loopback 网卡会拒绝运行；它不是机械狗测试。

执行相关测试：

```bash
python3 -m pytest -q tests/test_s10_*.py
python3 scripts/check_service_yml.py deep_robotics/s10_pro
```

现场部署、十分钟数据观察、启停三轮、读取中断恢复和刷新保留的逐项要求及回退步骤见 [deploy/ACCEPTANCE.md](deploy/ACCEPTANCE.md)。当前完成了动作卡代码与离线验证，生产执行仍锁定；没有导航执行、低层关节接管或厂家固件变更。
