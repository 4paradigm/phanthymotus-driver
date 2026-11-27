# 启元 Q1 探索版

`battery` 是只读传感器卡。在 Motus 画布添加「启元 Q1 探索版 → battery」后，打开「查看数据流」可查看实时电池读数。驱动启动后，卡片每 0.5 秒向 Core 发布一次 JSON，默认话题为 `/primebot_q1/battery`，格式为 `data/json`。

数据来自 `/aima/hal/bms/state`（`aimdk_msgs/msg/Bms`）。画布 JSON 的 `battery` 对象包含以下字段。

| 指标 | 字段 | 单位 |
| --- | --- | --- |
| 荷电状态（SOC） | `percentage` | %，直接取自 BMS 剩余容量百分比 |
| 电压、电流、输出功率 | `voltage_v`、`current_a`、`power_w` | V、A、W，电流为负表示放电 |
| 电池温度 | `temperature_c` | °C |
| 剩余容量 | `remaining_capacity_mah` | mAh |
| 完整循环次数 | `cycle_count` | 次 |
| 累计充放电容量 | `cycle_total_capacity_mah` | mAh |
| 厂商电池状态位 | `status_bits` | BMS 原始位掩码 |

`info` 返回卡片状态、输出话题与最近读数。driver 默认持续发布电池数据，Motus 的系统动作 `stop` 和 `start` 可以暂停和恢复发布。`battery.state` 为 `waiting` 表示等待首个样本，`live` 表示样本在 5 秒以内，`stale` 表示最后读数已过期。`age_s` 给出样本年龄。 已保存的旧画布需要将 `status` 卡换成 `battery`，并将下游连接指向 `/primebot_q1/battery`。

## preset_postures 出厂动作卡片

在 Motus 添加「启元 Q1 探索版 → preset_postures」。布局沿用 Bumi 的
`semantic_action`：一个 `action` 下拉框、每个动作独立的中文说明以及执行按钮，
无需填写 SDK 动作编号或文件路径。卡片类型是 `actuator`，开启画布只准备卡片，
不会自动使能电机、起身或播放动作。

| 动作类别 | action（中文含义） |
| --- | --- |
| 基础姿态 | `lie_face_up` 仰面躺下（1010）、`lie_face_down` 趴下／俯卧（1009）、`stand_up` 自主站起（1007）、`store_default` 折叠收储（1001） |
| 上肢互动 | `wave` 挥手、`beckon` 招手、`handshake` 握手、`bump_fists` 碰拳、`show_happiness` 开心、`make_heart` 比心、`cheer` 加油、`stretch` 伸懒腰、`speak_a/b/c` 三种讲话、`hug` 拥抱、`refuse` 拒绝、`dance_a/b/c` 三种上肢舞蹈 |
| 全身动作 | `intro_pose7` 亮相、`boxing` 拳击、`dance_locking` 锁舞、`dance_mj` 星际滑步、`dance_go` 跃动时刻、`happy_pick` 元气蹦蹦、`dance_q` 奇妙节拍、`show_enter` 闪亮登场、`show_return` 潇洒退场、`run_jump` 欢乐巡游、`siuzhuan` 进球狂欢、`dream_fly` 梦羽轻扬、`dance_babyq` 星际萌芽、`taiji` 太极、`explosive_gunfire` 爆裂节拍、`xinbaodao` 新宝岛 |
| 操作与查询 | `reset` 动作复位；`status` 只读查询当前运控状态、服务可用性和上次请求结果 |

上肢互动和全身舞蹈执行前，由操作者确认机器人稳定站立、双脚着地、空间平坦防滑且无人和障碍物，
并准备官方急停。驱动检查新鲜有效的机身直立朝向与运控的
`RUNNING/switch_ready`，只从走跑、全身控制或越障模式进行直接切换。
高层运控从 `/torso_imu_orientation` 读取朝向；若原始 IMU 有有效数据则优先使用。
姿态数据缺失或超过 0.5 秒时，`standing=null` 表示未知，禁止执行动作。
当前机身固件的 `GetMcAction` 没有填充切换就绪字段：驱动使用
`/aima/mc/common/state` 中不超过 0.5 秒、且与 RPC 模式及状态一致的数据，
并保留 `rpc_switch_ready` 与 `switch_ready_source` 便于诊断。
没有新鲜状态时不会把 false 改为 true；上述站立动作遇到摔倒、被提起、移动中或状态不一致均拒绝。
构建补充的消息定义及固件兼容性见 [motion_msgs](motion_msgs/README.md)。
上肢动作通过 `SetMcPresetMotion` 执行（16 个 Q1 枚举）；全身动作通过
`SetMcMotion` 执行（16 个官方公开名称）。下发前分别确认进入
`BIPED_WHOLE_BODY_CTRL` 或 `BIPED_WALK_RUN`，不会从被动、阻尼、躺姿或收储模式自动起身。
枚举来自[官方接口说明](https://github.com/QyroTech/qyro-q1-sdk/blob/main/docs/zh/接口说明.md)，
与既有 v1.1.8.0 SDK 匹配。机身配置中其他未在公开文档列出的动作没有加入卡片。

四个基础姿态通过一次 `SetMcAction` 直接发送对应枚举和名称，不调用上肢动画或全身舞蹈接口。
`lie_face_up/lie_face_down` 要求稳定直立且处于走跑、全身控制或越障模式。
`stand_up` 从地面姿态请求起身，直立时拒绝重复起身；`store_default` 请求官方折叠收储。
后两者不套用“必须站立、不得报告倒地”的条件，具体起始姿态与模式切换是否合法交给官方运控判断。
四个动作都要求新鲜且与查询一致的运控状态、有效朝向、切换就绪、未被提起且未移动，
阻尼保护模式下拒绝执行。驱动不会为满足条件而先切被动、阻尼或自行串联其他动作。
固件拒绝时原样保留 `code/state/reason`，操作者使用官方方式调整后再显式提交。
`status.basic_action_readiness` 分别说明四个动作的本地前置条件；`can_request=true`
仅表示可提交请求，不保证固件接受。顶层 `ready/not_ready` 仍描述站立互动／舞蹈的条件，
因此躺姿下 `not_ready` 不代表不能请求 `stand_up`。
基础姿态回执包括 `target_action`、起始状态和任务编号；随后用 `status` 查询实时模式、朝向和
`last_request`，物理完成由现场观察确认，不把到达某个模式或等待固定时长当作完成。

`state=accepted` 和 `task_id` 表示厂商服务已接受请求，**不代表物理动作完成**。
没有使用定时等待伪造完成通知。服务错误同时检查 `header.code` 和 `state`，
运动命令超时不重试；`unknown` 表示可能已发送命令但结果未确认。
ROS 客户端由隔离进程复用，首次服务发现最多等待 8 秒并等待连接匹配。
只有只读 `GetMcAction` 响应超时可重试一次，运动命令不重发。
首次显式操作会启动连接进程。若 ROS 尚在初始化，立即返回 `initializing`，
本次动作不会发送或排队；连接建立后需重新显式执行。这样避免机身启动期间
的 DDS 初始化阻塞 MCP 请求，也不会在初始化结束后突然补发旧动作。
初始化会等待四个服务被发现并预留 1 秒完成客户端匹配；繁忙机身上可能需要数十秒。
重复的并发调用返回 `busy`，不会排队后补发。`start/stop` 是卡片生命周期，
`stop` 取消尚未发送的后续调度，不能保证停止已经送达机器人的动作。

`reset` 是显式操作：全身控制模式发送 `DEFAULT_POS=0` 并允许打断上肢动画；
全身模仿模式请求返回走跑模式。它需要固件允许切换，可能被拒绝，**不是急停**。
官方运控服务不可用时，`status` 显示 `unavailable`；检查 App 开发模式及官方运控服务，
卡片不会擅自修改开发模式、启动官方服务或解除保护。
Core 自动轮询的系统动作 `info` 立即返回缓存及其年龄，不反复创建 DDS 查询进程；
需要更新机身状态时，在动作下拉框选择 `status` 执行。缓存的 `ready/idle` 是卡片状态，
机身是否满足前置条件见 `status_snapshot`，过期缓存不会用于授权动作。

## 测试

纯逻辑测试可在仓库根目录运行（无需 ROS）：

```bash
PYTHONPATH=. python3 -m unittest discover -s primebot/q1/tests -p test_preset_postures.py -v
```

在已加载 `/opt/ros/humble/setup.bash` 和 `/q1_ws/install/setup.bash` 的 ARM64
Q1 镜像内，挂载仓库到 `/repo` 后执行：

```bash
cd /repo
Q1_RUN_DDS_TESTS=1 PYTHONPATH="/repo:$PYTHONPATH" python3 -m unittest discover -s primebot/q1/tests -p 'test_*.py' -v
```

动作集成测试固定使用隔离 DDS 域 **93** 的模拟服务，电池回归测试使用 **87/88**，
不会向真机域 0 下发动作。测试覆盖模式前置条件、取消、错误码、超时不重试、
全部动作映射和实际 ROS 消息编解码。真机物理动作需由现场操作者另行验收。

## ARM64 Docker 部署

Apple 芯片 Mac 和 Linux ARM64（Jetson／板载机）使用同一套 Ubuntu 22.04 / ROS 2 Humble 驱动镜像。先启动同一台部署主机上的 Motus Core，并接通 Q1 的机器人网段。主机需要 Python 3、Docker 和 Docker Compose v2。在驱动仓库根目录运行：

```bash
python3 primebot/q1/deploy/q1.py build
python3 primebot/q1/deploy/q1.py start
python3 primebot/q1/deploy/q1.py status
python3 primebot/q1/deploy/q1.py stop
```

`build` 在首次部署或更新驱动源码时运行，生成 `primebot-q1:local`。已有 ARM64 驱动镜像时，可用 `start --image <镜像名>`。启动后，`status` 显示 Core 注册情况和最近的电池样本。看到 `registered` 和 `live` 后，在 Motus 画布添加 `battery` 卡片并打开「查看数据流」。`stop` 停止 Q1 服务，Core 继续运行。

- **Apple 芯片 Mac**使用 Docker Desktop。启动命令同时管理宿主机的 DDS 转发、容器内的 UDP 转发和 driver。见 [Mac 有线联调说明](tests/mac/README.md)。
- **Linux ARM64** 上，Core 和 Q1 driver 都使用主机网络，driver 直接通过 Q1 网段读取厂商 DDS。脚本默认读取主机的 `/opt/phanthy-motus/dds-local.xml`，并连接 `https://localhost:15678`。若本机 Core 使用 HTTP 或 XML 在其他位置，可运行 `start --core-url http://127.0.0.1:15678 --core-dds /实际路径/dds-local.xml`。团队的 Linux 部署流程也可继续使用原有 [部署配置](deploy/service.yml)。

镜像编译启元 SDK 提交 `618d995d4362f981ab5945624f8be9343c2cb68b` 的 `aimdk_msgs` 包。构建从[启元开发者社区](https://dev.primebot.com/download)下载官方 v1.1.8.0 SDK，校验 ZIP 的 SHA-256，仅将 `aimdk_msgs` 放进编译工作空间。相同 ZIP 的镜像地址可用构建参数 `Q1_SDK_URL` 指定。colcon 和 rosidl 生成消息接口，`nav_msgs`、`sensor_msgs` 是厂商包声明的消息依赖，Fast DDS 和 PyYAML 分别用于通信与加载驱动配置。机器人读取进程在域 0 使用厂商 DDS 配置，Motus 侧在域 42 使用 Core 的 DDS 配置。Core 通过 `http://localhost:15748/mcp` 访问驱动。驱动沿用 X2 的 Motus 公共运行时。[启元消息源码](https://github.com/PrimebotTech/primebot-q1-sdk/tree/618d995d4362f981ab5945624f8be9343c2cb68b/aimdk_msgs) · [X2 驱动](../../agibot/AimDK_X2/main.py)

在已接入 Q1 的 Linux ROS 环境加载启元工作空间后，可用下列命令核对原始 BMS 消息。

```bash
timeout 5 ros2 topic echo /aima/hal/bms/state --once
```

启元官方快速入门给出开发者模式、Ubuntu 22.04 / Humble 环境、网线直连和 BMS 读取步骤。电池消息字段与状态位可查接口手册。[SDK 快速入门与集成指南](https://github.com/PrimebotTech/primebot-q1-sdk/blob/main/docs/zh/SDK快速入门与集成指南.md) · [接口说明](https://raw.githubusercontent.com/PrimebotTech/primebot-q1-sdk/main/docs/zh/接口说明.md)

本地集成测试及真机验证范围见[验证记录](验证记录.md)。
