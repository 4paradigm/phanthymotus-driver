# S10 Pro Docker 部署与验收

## 2026-10-23 本轮检查点

本轮 driver 范围为异步 ACP、英文工具定义、共用参数与严格校验。六卡范围、原标识与画布布局保持：Robot State (`state`)、Joint Feedback (`joints`)、IMU (`imu`)、Odometry (`odometry`)、LiDAR Point Cloud (`lidar`)、Basic Motion (`motion`)。动作标识仍为 `stand/lie/velocity/stopmotion`，厂家话题、网络、DDS、固件边界保持。用户另行明确授权了配套 Core 三个产品文件的最小修改与离线回归；该补丁独立交付，不进入 driver PR。点云视图问题和实际运动反馈延迟不属于本轮修复。

最后已知的 2026-10-22 现场候选为 `55933d8e`。更早的 `s10-pro-motion:53e517db`，镜像 ID `sha256:0f3959dd8e3bdaae184591473589dff7e9456e2551d1d4d544fe0c7c9f61db14`，仅是历史记录。本轮没有真机环境，没有连接或读取现场版本，尚未替换现场容器，也未发送任何真机动作。

本轮隔离验证如下，证据保存在工作区 `acceptance/s10pro-20261023`，不以历史测试代替本轮结果：

| 项目 | 本轮状态 |
| --- | --- |
| driver 镜像 | `s10-pro-acp:offline-20261023`，ARM64，`sha256:bcb0e18456ca14dca1cc4e84992fd581c09f93599a6dd2bb82bfbc0e27440ff9`；产品文件哈希与源码一致 |
| Docker 内参数、动作、ACP 等测试 | 710 passed，加载 Jazzy 与编译消息；含真实本地 HTTPS 验证 |
| `--network none` ROS 合成测试 | 传感器过期/恢复/独立停止/reader 退出；动作 `1 → 17`、速度归零/停止确认、趴下及三个本地 ACK 均通过 |
| 真实 driver/Core 函数组件 ACP | 7 场景通过；早到、丢 ACK 重试、完成/取消/错误保留真实状态 |
| 配套 Core 配置链路 | 65 项新增后端测试；8 场景真实 loopback MCP→Core 保存/ping/LLM schema 整合通过 |
| 浏览器组件回归 | S10 10 组（含 22 输入校验），Franka 11、PiPER 11、标题 9、完成日志去重 16 组通过；Web 单测 210 项通过 |
| 真机版本/传感器/动作验收 | 未进行，无真机环境 |
| 现场 Core 修改或替换 | 未实施；仅在本地修改获准三文件并离线测试 |

## 执行锁与异步完成合同

生产构造器继续将 `DDSProcessTransport.execution_enabled`、`DDSMotionBackend.execution_authorized`、`MotionPlugin.execution_authorized` 固定为 `False`。动作卡的 start 仅取得只读反馈观察 lease，不启动命令 worker、不创建运动命令 publisher、不切换模式或授予控制权。画布设置不能改变这些进程授权。

异步合同仅适用于通过全部校验且真正受理的 `stand/lie/velocity`：

- `inputSchema.x-completion.actions` 仅含上述三项，`timeout:90`、`silent_completion:false`，资源为 `base`。90 秒是 Core 等待上限，不是实际运动时长。
- 新受理返回真实 `action_id`、关联 `request_id` 和 accepted/running 状态；锁定、忙碌、非法参数同步拒绝，不创建 pending。
- 后台执行由唯一终态提交出口通知 `completed/error/cancelled`。stand/lie 需新鲜真实姿态反馈；velocity 需时长到期、运动反馈与停止反馈确认。发布成功、DDS 无业务 ACK、受理成功或计时结束均不能单独当成完成。
- stopmotion 优先取消本方活动任务并立即返回受理状态与 `original_action_id`，不另建动作。原任务负责停止尝试、停止确认与唯一终态。没有活动任务时不伪造任务；停止不能确认必须保留 error 与 `stop_confirmed:false`。
- 同一请求 ID 与相同目标重试不再运动；目标改变则拒绝。活跃重放返回原 ID；终态重放同步返回原结果与 `original_action_id`，不再返回顶层 `action_id`，防止 Core 重登记和重复通知。
- info/status、传感器启停/配置仍同步。查询和刷新不创建完成通知；driver 不手拼 collector 日志，不额外发送详细手工 trigger。

任务 ID/终态保存在 `motion-tasks.sqlite3`。重启不恢复物理运动，未结束任务转为 `driver_restarted_stop_unconfirmed` 的 error。终态在 `acp-outbox.sqlite3` 持久化后发送至 `/api/acp/complete`。只有 HTTPS 成功且收到 `ok:true`、同一 action_id 和 `state:matched/cached/duplicate` 的明确 ACK 才标为 acknowledged；HTTP 200、空响应、旧 Core 无 state 回执、错 ID 或无法验证的 TLS 都不算。

默认 outbox 发送窗口为 60 秒、间隔 1 秒、最多 12 次尝试；达到期限/次数仍保留 `unacknowledged`，未尝试则保留 `not_delivered`。重试使用同 ID 和不可变终态；查询不延长期限、不覆盖已有终态。Core 早到缓存及回执去重是有界内存机制，不能承诺 Core 重启后仍无限期 exactly-once。

## 共用参数与英文定义

参数 schema 与服务端校验统一来自 `parameters.py`。driver 提供的卡片名称、动作展示/说明、字段说明、结果和错误为英文；文档及沟通为中文。Core 自有按钮语言不在 driver 定义范围内。

当前默认速度为 0；vx/vy/yaw_rate 范围分别为 ±0.2 m/s、±0.1 m/s、±0.2 rad/s，duration 为 0.1–2 秒、默认 min(1 秒, 当前最大时长)。范围随当前 driver 配置生成，不扩张默认约束；这些是应用限制，不是厂家最大能力，真机约束交集仍需核实。motion 反馈固定 0.5 秒新鲜度门槛不修改。

传感器 refresh_hz 默认 1/5/5/5/2，范围 0.1–30 Hz，表示应用发布频率上限；stale_after_s 默认 5 秒，范围 0.1–300 秒；LiDAR max_points 默认 6000，范围为整数 100–40000。

动作参数缺省取声明默认，配置缺省保留当前值。显式 null、空字符串、bool 冒充数值、非有限值、越界、未知字段、直接与嵌套重复字段均拒绝，不转零、不截断、不部分应用。请求 ID 缺省 UUID、显式不超过 128 字符且非空白；issued_at_ms 缺省 Unix 整数毫秒，ttl_ms 是整数 100–2000，默认 1000。首次请求超过 TTL 或未来偏差大于 100 毫秒拒绝；TTL 不等于运动时长。动作/目标不默认选择可执行运动。

## 配套 Core 与部署边界

配套 Core 基线为 `0897a8d28d3139380b72ed83712397eb281a8cd3` 加原有未提交补丁。早到缓存、手动画布 ACP 登记、只读终态 API 与显示去重来自该 dirty snapshot，不能归给干净 HEAD 或认定现场已支持。本轮保护已有修改，只在用户明确授权后修改 `src/api/canvas.py`、`web/js/sidebar.js`、`web/js/canvas.js`，并单独导出本轮差异。授权前的历史审查在 `core-contract-review.md`。

配套逻辑严格校验原始配置，等待 driver 接受后保存，再标准 MCP ping 更新 LLM schema；配置省略保留旧值。driver 拒绝时不写入；已接受/保存但刷新失败时明确返回 applied/saved/schema_refreshed 状态，不伪装回滚。保存发生在 ping 前，避免首次上线的自动恢复任务重放旧值。

前端不截断整数，不把显式清空当成省略；配置和动作的非法可见输入不发送。工具定义变化时刷新表单范围、默认值和标题，保留卡片根、布局、连线、结果和 ACP 观察。独立浏览器夹具使用真实 Core 函数与模拟服务，不是现场画布验收。

配套补丁依赖上述既有 ACP 基线，不可仅凭 HEAD 在未知 Core 上覆盖三个文件。本轮本地修改与测试授权不代表共享 Core 部署或真机动作授权。仅更新 driver 时，旧 Core 仍需标准 ping 后重新加载画布。

## 本地 Docker 与测试画布验收

本轮没有真机，仅使用本地隔离环境。模拟 DDS 控制输出限制在 `--network none` Docker 内；函数/浏览器夹具不使用 ROS，HTTP 整合只用临时 loopback 端口。两个 ROS smoke 脚本拒绝启用的非 loopback 网卡、非 loopback 路由或 IPv6 地址，允许 Docker 暴露的关闭且无路由的空闲隧道设备；不得用 robot Wi-Fi、真实控制进程或现场 Core 验收。

针对性验收应记录源码版本、镜像 ID、测试配置与场景结果：

1. schema、服务端边界、缺省与非法值逐项对应；所有非法请求无控制输出。配置更新后 tools/list、Core registry 和打开的表单反映有效范围；记录使用的确切配套 Core 快照。
2. 真实 S10 受理返回 ID；模拟完成/失败/取消/反馈过期/快速完成/停止未确认/完成取消竞态均正确。每个 accepted 任务只有一个不可变终态；accepted/running 不解除成功后续等待。
3. 实际 Core ACP 函数组件覆盖早到回调、ACK 丢失同 ID 重试、并发重试、通知失败重试；每任务一 collector 通知，结果失败/取消不当作成功。
4. 持久状态重开、未结束任务恢复为 error、请求去重、终态重放、发送期限与未送达状态可检查；不得删除 state 规避测试失败。
5. ACP HTTPS 覆盖 CA/服务名验证、精确叶证书模式、错误证书、无效 ACK、认证失败和重定向拒绝；测试证书/凭据仅本地生成使用，不进交付仓库。
6. 测试画布使用真实 S10 schema 与真实 Core 组件，标明 Simulated only — no hardware；检查六卡英文、原标识/布局、完成/失败/取消通知、查询/刷新不重复通知。组件夹具不等同完整 Core HTTP/TLS 部署；分别记录未测层。

Core 的 sync 外层 completed 仅表示等待已结束，业务成功以对应 action 的终态 status 为准。isolated fake feedback、虚构控制权和合成 ROS publisher 不能证明机械狗运动成功，不能放宽反馈 0.5 秒门槛或解除真机执行锁。

## 现场部署前置条件（本轮尚未执行）

1. 按团队 PR/image 流程取得准确提交和镜像，保存本轮测试证据，不能把本地构建镜像当成机器人已部署版本。
2. 真机恢复后先读取当前镜像/版本、端口、应用配置和用户修改；备份仅本应用容器、配置及 driver state，保留现有现场服务。不要按旧记录覆盖新状态。
3. 原生 NOS Jazzy 的 `preflight.py --public-contract` 与 `motion_preflight.py --public-contract` 和容器递归消息契约重新比对。原生消息包 1.0.23 或其他版本名不能代替字段/类型兼容验证。
4. 核对实际 Core 端点/回执、手动画布登记与终态显示能力。现有只读记录不含现场 ACP 源文件身份，不能自动认定 dirty Core 能力已部署。
5. 默认授权均为 false；idle、传感器使用、motion 状态观察、停止或图检查时没有命令 worker/publisher。部署验收不包含任何物理动作。

## Docker 拓扑、持久状态和 ACP TLS

历史现场应用目录为 `/userdata/phanthy-motus-s10`，Core URL 为 `https://10.21.33.106:15678`；这是历史入口，当前未连通核验。Core 与 driver 为同一 NOS 上独立容器：Core 使用既有 Humble 应用镜像，driver 自带 Jazzy 与已编译公开消息定义，不能把宿主 Jazzy 二进制挂入 Humble。两者采用 host network；平台 DDS domain 42 加 loopback-only dds-local.xml；MCP 为 15745，namespace 为 s10_pro。历史 NOS 无 Compose 插件，容器通过 Docker CLI 管理；service.yml 为平台服务模板，现场工具须重新核对。

reader 为独立进程，使用 domain 0 和只读自有 s10-reader-udp.xml，不继承平台 XML。模板路径为 `/opt/phanthy-motus/s10-reader-udp.xml`，外部配置 reader.vendor_profile 必须显式指向真实文件。该 UDP-only 配置曾解决容器本机标准 IMU/ODOM/点云未收到的问题；保留 private IPC，不修改厂家 XML、服务或运行 setup_ros2.sh。命令 worker 仍 dormant。

本轮新增的 `S10_STATE_DIR` 默认 `/work/state`，模板新增宿主 `/opt/phanthy-motus/s10-pro/state:/work/state`。独立现场应用可将宿主 source 调整为 `/userdata/phanthy-motus-s10/state`。该目录只含 driver 任务与 outbox，不挂 Core 数据库；仅本应用可读写，替换容器和回滚时都须保留。不要与其他应用共用、清空或复制别人的 state。

机器人 DDS 本身不需要 TLS；driver → Core 的 ACP 回调需要 HTTPS 信任。默认 AGENT_CORE_URL 为 `https://localhost:15678`：

- 系统信任库或 `S10_ACP_CA_FILE`：正常验证 CA 链和服务名。
- `S10_ACP_PINNED_LEAF_FILE`：操作员预先核对的单张 PEM 叶证书；保持 CERT_REQUIRED，握手后在发送正文/凭据前比对精确叶证书摘要。不能从在线端点自动采信，不同时配置 CA_FILE 与 PINNED_LEAF_FILE。
- 凭据优先从 ACCESS_TOKEN 环境变量读取，否则从 S10_ACP_ENV_FILE 指定文件读取；未指定文件时检查 `/opt/phanthy-motus/.env`、`/work/.env`。值不得进入 Git、镜像、说明或日志。凭据/证书文件单独只读挂载，模板未内置它们。
- 不接受明文 HTTP、URL 内用户名密码、跳过验证或重定向。Core 的 ACP 认证例外仅 loopback 来源；更改 Docker 网络后不能假定仍满足。

证书与凭据挂载示例见上级 README；需要把片段合并至原有 volumes/environment，而不是覆盖 DDS/state 挂载。

## 历史现场证据与当前未测项

历史五卡 `s10-pro-readonly:3a1cb138` 镜像 ID `sha256:ffdaa0eb9187299028d63f2fde87de842cb2fe8e129e6c56bc10617de271dc28` 曾通过五类传感器新鲜数据、Core WebSocket 数据通路、三轮独立启停、reader 暂停/终止恢复与连续 600.09 秒观察。该观察收到 599 state、2946 joints、2970 IMU、2966 odometry、576 cloud、1194 lidar-info；JSON 均 fresh、每帧点云 6000 点，无解析错误。最后 reader 退出检查须在所有传感器和 motion observer 均停止后进行。

更早 `53e517db` 六卡锁定版通过传感器与 locked 状态检查；10 月 22 日后续 `55933d8e` 候选的六卡截图与检测包另有历史交付记录，点云视图空白、动作反馈过期仍为已知未修项。以上不是本轮新版本、ACP、浏览器或真机动作验收。本轮结果以检查点及检测包中的实际证据为准。

手柄显示 v2.5.5 尚未匹配系统/控制服务版本。/MOTION_INFO 不含厂家任务 ID；本地 request 标记关联不等于厂家任务回执。控制权、导航模式、手柄接管规则和机器人失联停止边界仍未核实；指南 2 秒心跳只涉及状态上报，不能当作运动 deadman。既有实际反馈新鲜度终检失败仍是阻塞项，本轮不扩大修复或放宽阈值。

若未来单独申请真机执行，需先建立身份/版本、控制权、模式与失联停止证据及现场接管条件。当前零速也要求本方有效控制权；不能用配置字段声称有特殊停止权来绕过 publisher 规则。公开 sdk_deploy 是低层关节/RL 路径，初始化可能发送关节复位，不能当作连通性探针运行。

## 应用回退

仅停止本应用容器 `embodied-deep-robotics-s10-pro`，确认父进程/reader/本方后台任务退出，保留 driver state、原镜像和独立应用配置。恢复备份前核对后续用户修改；不要清空持久任务库或凭空把旧 running 标记成功。Core 数据库/证书/现场厂家服务不在本轮替换范围。

停止或回退应用不是硬急停，也不证明机器人已停止；不重启厂家 AOS/NOS 服务，不修改其他应用。
