# 云深处 S10 Pro Driver

在现有 PhanthyMotus 卡片框架中提供五张 sensor 卡：`state`、`joints`、`imu`、`odometry`、`lidar`，以及一张执行锁定的 `motion` actuator 卡。沿用现有画布的表单、按钮和 JSON/点云视图，没有低层关节控制入口。

## 2026-10-23 整改与版本边界

本轮在现有六卡范围内增加 driver 异步 ACP、英文工具定义与共用参数校验。`stand/lie/velocity` 通过校验并受理后立即返回本次 S10 任务的 `action_id`，后台执行并通过 ACP 报告唯一终态；不再把动作完成等待占在 MCP 调用线程上。卡片、工具/动作标识、平台话题和原画布布局保持兼容。

生产入口的 `DDSProcessTransport.execution_enabled`、`DDSMotionBackend.execution_authorized`、`MotionPlugin.execution_authorized` 仍固定为 `False`，画布配置不能修改授权。启动动作卡只观察运动反馈，不启动命令 worker、不创建运动命令 publisher，也不自动切换机器人模式。新运动请求仍同步返回 `unavailable`，不创建 ACP pending 任务。

当前生产入口尚未绑定真实 `authority_provider` 和本机控制契约，也没有已授权的命令 worker 启动路径；这部分生产接入尚未完成。仅启用卡片、连接手柄或解除急停无法替代该实现。`readiness` 和 `next_steps` 分别列出卡片生命周期、三层进程授权、机器证据、应用自身要求、控制权适配器、实际反馈年龄和 worker 状态；`execution_available` 必须同时通过这些检查。既有 `blocked_by` 字段保留兼容。

运动只读观察使用独立 `ReaderBridge` 子进程，与传感器/点云读取分开，避免点云转换或该管道积压拖延运动反馈；state 卡继续使用自己的原始数据源。启动和关闭只回收各自观察者，运动反馈的 0.5 秒门槛保持。该隔离不赋予控制权，也不改变厂家协议。

2026-10-23 已在 NOS 通过 Docker 部署代码提交 `058a369a`，driver 镜像为 `s10-pro-readiness:058a369a`（image ID `sha256:907e3a89c6a47c48f49239f1cd92974ff0457e44fcf1e35d883ade26bb3e7a22`）。全部产品文件哈希与测试源码一致；本轮 driver 更新未替换 Core，旧容器和私有备份保留。ARM64/Jazzy Docker 全量测试 **750 passed**；独立运动反馈与点云的隔离合成检查通过。真机 30 秒内 112 个运动反馈样本全部新鲜，最大年龄约 0.158 秒，五张传感器卡均 fresh；0.5 秒门槛保持。该观察窗口不代表真机动作验收，未发送起立、趴下或速度命令。`55933d8e`、`s10-pro-motion:53e517db` 和 `s10-pro-acp:f85a38f7` 均为历史版本。

## 卡片与数据

| 卡片 | 已验证的厂家数据源 | 平台输出 |
| --- | --- | --- |
| Robot State (`state`) | `/BATTERY_DATA`、`/MOTION_INFO` | `data/json` |
| Joint Feedback (`joints`) | `/JOINTS_DATA_10HZ`，历史实机 16 条记录 | `data/json` |
| IMU (`imu`) | 标准 `/IMU` | `data/json` |
| Odometry (`odometry`) | `/ODOM` | `data/json`，未声明 `motus.odom/1` |
| LiDAR Point Cloud (`lidar`) | `/LIDAR/POINTS` | `sensor/pointcloud` 加独立 `lidar_info` JSON 元数据 |

平台话题固定为 `/{namespace}/s10_pro/{card}`。默认 namespace 为 `s10_pro`，可在应用配置中指定独立设备名。`info` 即使未启动也返回固定端口；所有卡片默认空闲，只有画布启动后才订阅、发布。`stop` 只停止本卡输出及所属读取源；所有传感器卡和动作状态观察均停止后，读取进程退出。

传感器卡支持 `info/start/stop/status/config`。设置可调整 `enabled`、`refresh_hz`（0.1–30 Hz）、`stale_after_s`（0.1–300 秒）；雷达还支持整数 `max_points`（100–40000，默认 6000）。刷新频率默认依次为 1/5/5/5/2 Hz，传感器过期阈值默认 5 秒。刷新频率是应用发布限频，不是原始采样率保证。Core 的配置保存及输入限制见下文，不能将 UI 保存成功当作 driver 已接受配置。

原始数据和厂家 stamp 保留。新鲜度按本机接收单调时钟计算，不混用不同来源的时间尺度。未知的电池/关节单位、状态码含义保持未知，空 frame 不补成已知坐标系。没有新数据时显示 `no_data`，过期显示 `stale`，不将旧值标成在线。

## 基础动作卡

Basic Motion (`motion`) 提供起立 `stand`、趴下 `lie`、限时速度 `velocity` 和本方停止 `stopmotion`。`start` 只启用状态观察；生命周期 `stop` 取消本方请求并停止本卡输出。固定输出是 `motion_status`、`motion_events`，没有连续控制输入。driver 提供的卡片名称、动作说明、参数、结果和错误文案使用英文；这不改变 Core 自有界面按钮的语言。

实现包含互斥、取消优先、过期请求拒绝、有限时速度和限幅、控制权与反馈新鲜度检查。请求接受、实际反馈确认和完成分开记录；DDS 发布没有业务 ACK，传输层保留 `accepted=null`。停止为本应用持续零速与实际停止反馈确认，不是硬急停。

高层 DDS 路线依据[官方 S10 软件开发指南 V1.0.2](https://support.deeprobotics.cn/docs/#/259676015)和用户提供的[钉钉软件开发文档](https://alidocs.dingtalk.com/i/p/OlnXRnDldg3J7XLp/docs/dQPGYqjpJYgQaZdQtBNLa6a6Wakx1Z5N?dontjump=true)，使用 `/MOTION_STATE`、`/NAV_CMD` 和 `/MOTION_INFO`。起立必须先确认状态 `1`，再请求状态 `17`，不能将 ASDU 的自动转换规则套到 DDS。速度使用物理单位并要求明确的导航模式授权。命令进程独立于平台 DDS，只有显式进程授权且逐次控制权、取消状态和期限检查通过后才创建 publisher。通信超时会关闭命令进程，不自动重试动作。

机器人高层 DDS 路线不使用 TLS；本轮新增的 driver → Core ACP 回调使用独立的 HTTPS 信任配置，见 Docker 部署说明。保留的 `motion_transport.py` 是可选 ASDU/TLS 实验实现：严格校验 16 字节版本 1 帧头、JSON、有界帧和请求关联，连接要求 CA、主机名和独立证书指纹；生产入口未选择这条路线。此前仅诊断过 AOS 30003 的 TLS 1.3 证书链，未发送应用帧。

用户提供的手柄显示版本为 `v2.5.5`，尚未确定它对应系统、APP 还是控制服务版本。公开指南没有建立这台机器的完整控制权/手柄接管优先级，也没有确认速度指令失联后的机器人停止时限；2 秒心跳超时只表示停止状态上报，不能当作速度看门狗。`/MOTION_INFO` 的可见状态不能代替控制权、导航模式或失联停止保证。真机执行仍需匹配本机版本和身份、可验证的控制契约、现场准备及接管条件，当前未开放。

[GitHub SDK](https://github.com/DeepRoboticsLab/sdk_deploy)和[GitCode SDK](https://gitcode.com/DEEPRoboticsLab/sdk_deploy)主要用于低层运动算法/关节控制。其初始化路径会产生关节复位等命令，不能作为连通性或环境检查直接试跑。本驱动不调用该低层入口，也不复用 M20 的控制编码。

## 异步 ACP 与重试

`inputSchema.x-completion` 仅声明 `stand/lie/velocity`，`silent_completion:false`，Core 等待上限为 90 秒；该上限不是机器人运动时长。锁定、忙碌或非法参数同步拒绝，不返回顶层 `action_id`。受理返回 `state:accepted`、`status:running`、真实 `action_id` 和 `request_id`，不代表物理成功。

后台任务只有一个终态提交出口，状态为 `completed/error/cancelled`。起立、趴下必须得到新鲜真实反馈确认；速度必须到期、获得运动反馈且确认停止，不能仅以发布成功或计时结束判为 completed。取消优先请求原任务停止，`stopmotion` 立即返回受理状态与 `original_action_id`，终态由原任务上报。无活动任务不虚构任务；停止未确认保留 `error` 和 `stop_confirmed:false`。运动反馈 0.5 秒门槛不放宽。

同一 `request_id` 对同一动作目标重试复用原任务，不再次运动；若目标改变则拒绝。活跃重试返回原 `action_id`，已终态重试同步返回原结果与 `original_action_id`，不再返回顶层 `action_id`，防止 Core 重新登记已回收任务。查询/刷新不新建任务、不补发手工 trigger。任务记录持久化到 `motion-tasks.sqlite3`；进程重启不恢复物理动作，未结束记录变为停止未确认的 error。任务历史最多 4096 项，满后拒绝新任务并保留已有身份，不自动删除历史以腾出空间。

终态通过 `POST /api/acp/complete` 发送，详情保留在任务/卡片记录中。`acp-outbox.sqlite3` 保存不可变终态与待发送记录；只有 HTTPS 成功且 ACK 的 `ok:true`、同一 `action_id`、`state:matched/cached/duplicate` 都满足才标为 acknowledged。默认发送窗口 60 秒、重试间隔 1 秒、最多 12 次尝试；达到期限或次数仍保留 `unacknowledged`，从未发送则为 `not_delivered`，不伪报已送达。查询不延长期限，重启后仍使用原 ID 和终态。Core 的早到缓存和去重为有界内存机制，不能声称跨 Core 重启无限期 exactly-once。

## 共用参数与 Core 界面边界

`parameters.py` 同时生成 driver UI/LLM schema 并执行服务端校验。当前默认速度为 0，`vx/vy/yaw_rate` 的有效范围分别为 ±0.2 m/s、±0.1 m/s、±0.2 rad/s；duration 范围 0.1–2 秒，默认 `min(1 秒, 当前最大时长)`。schema 随当前 driver 配置更新。这些是应用限制，不是厂家最大能力；真机约束交集仍未完成核验，不能据可配置上限解锁运动。

动作参数缺省使用声明默认值；配置更新缺省保留当前值。显式 null、空字符串、bool 冒充数值、NaN/Inf、越界、未知字段或重复的直接/嵌套配置字段均明确拒绝，不转零、不截断、不部分应用。`request_id` 缺省生成 UUID，显式值须非空白且不超过 128 字符；`issued_at_ms` 缺省生成 Unix 整数毫秒，`ttl_ms` 为 100–2000 的整数，默认 1000，超过 TTL 或未来偏差大于 100 毫秒的首次请求拒绝。TTL 是受理有效期，不是运动时长。动作/目标不默认选中可执行运动。

单独部署 driver 不能修复旧 Core 的提前保存、整数截断或表单不刷新的行为。经用户明确授权，配套 Core 工作树仅修改 `api/canvas.py`、`web/js/sidebar.js`、`web/js/canvas.js` 三个产品文件：原始配置严格校验，等待 driver 接受后保存，再标准 ping 更新 LLM；前端不截断整数、显式清空会拒绝，并按新 schema 更新已打开表单，保留布局、连线、结果和 ACP 观察者。若已应用/保存后目录刷新失败，会明确报告这些状态，不假装回滚或完全成功。

配套 Core 改动保留既有 dirty snapshot，独立交付于工作区 `acceptance/s10pro-20261023`，不混入 driver PR；本轮未替换现场 Core。只读基线审查在 `core-contract-review.md`，其“未授权”描述是授权前的历史记录。新旧组件的隔离回归不能证明现场 Core 已支持；若仅更新 driver，应执行标准 MCP ping 并重新加载旧画布，不能声称自动同步已部署。

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
  - /userdata/phanthy-motus-s10/state:/work/state
```

本轮新增 driver 专属持久目录，`S10_STATE_DIR` 默认 `/work/state`，内含任务记录与 ACP outbox。宿主目录须仅本应用可读写，不能与 Core 数据库或其他机器人 state 共用。容器替换保留此挂载；不要通过删库、清空状态或换目录绕过请求去重。模板 `deploy/service.yml` 已声明 state 挂载，但未包含任何证书或访问凭据。

ACP 默认目的地为 `AGENT_CORE_URL=https://localhost:15678`。正常 CA 模式可设置 `S10_ACP_CA_FILE`，同时校验信任链和服务名；使用系统信任库时可省略该变量。自签名现场可改用操作员预先核对并交付的单张 PEM 叶证书，通过 `S10_ACP_PINNED_LEAF_FILE` 精确固定该证书；此模式保留 `CERT_REQUIRED` 并在发送凭据/正文前比对完整叶证书摘要，不从网络自动采信。两种变量不能同时设置，不允许关闭证书校验、明文 HTTP 或跟随重定向。例如为独立应用追加：

```yaml
volumes:
  - /userdata/phanthy-motus-s10/core-cert.pem:/run/s10/acp-leaf.pem:ro
  - /userdata/phanthy-motus-s10/.env:/run/s10/core-access.env:ro
environment:
  - S10_STATE_DIR=/work/state
  - AGENT_CORE_URL=https://localhost:15678
  - S10_ACP_PINNED_LEAF_FILE=/run/s10/acp-leaf.pem
  - S10_ACP_ENV_FILE=/run/s10/core-access.env
```

该片段需要合并进服务现有的 volumes/environment。凭据从 `ACCESS_TOKEN` 环境变量读取；未设置时读取 `S10_ACP_ENV_FILE` 指向的文件，否则依次检查 `/opt/phanthy-motus/.env` 与 `/work/.env`。凭据文件仅在本机只读挂载，不把值写入仓库、镜像或日志。Core 的 ACP 认证例外仅允许 loopback 来源，host network 下使用本机地址；更换容器网络或共享 Core 版本后须另行核对认证合同。

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

历史 `53e517db` 六卡锁定版本通过了五卡真实数据及动作 locked 状态通道检查。10 月 22 日后续候选 `55933d8e` 的部署、六卡截图及检测包另有交付记录；当时的点云视图空白和动作反馈过期问题未纳入本轮修复。本轮没有真机，也不复用历史证据作为新版本验收。没有发送过真机起立、趴下、速度、模式切换、复位或急停命令。

本轮只进行本地开发及隔离验证，不连接真机。ARM64/Jazzy 镜像 `s10-pro-acp:offline-20261023`（ID `sha256:bcb0e18456ca14dca1cc4e84992fd581c09f93599a6dd2bb82bfbc0e27440ff9`）已通过 710 项测试，镜像产品文件哈希与本轮源码一致。真实 ROS 2 合成验证通过起立 `1 → 17`、限时速度归零/停止确认、趴下、三个本地 ACP ACK，以及传感器输出、过期恢复、独立停止和 reader 退出。真实 driver/Core 函数组件的 7 个 ACP 场景通过；独立浏览器验证覆盖 S10 schema 刷新、22 个严格输入用例和刷新期间 ACP，另回归 Franka/PiPER。完整测试证据在工作区 `acceptance/s10pro-20261023`。

`tests/s10_arm64_smoke.py` 与 `tests/s10_motion_dds_smoke.py` 只用于 `--network none` 容器；隔离检查拒绝启用的非 loopback 网卡、非 loopback 路由或 IPv6 地址，允许 Docker 暴露的关闭且无路由的空闲隧道设备。隔离环境中的合成发布者、假控制权、fake backend 和本地 ACK 不能证明机器人动作成功、控制权、实际 ACP 部署或失联停止保证。

```bash
python3 -m pytest -q tests/test_s10_*.py
python3 scripts/check_service_yml.py deep_robotics/s10_pro
```

部署、分版本验收和应用回退要求见 [deploy/ACCEPTANCE.md](deploy/ACCEPTANCE.md)。
