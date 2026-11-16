# Leju Kuavo 5-W Driver

基于乐聚 [Kuavo 5-W 接口使用文档](https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/4%E5%BC%80%E5%8F%91%E6%8E%A5%E5%8F%A3/Kuavo%205-W%20%E6%8E%A5%E5%8F%A3%E4%BD%BF%E7%94%A8%E6%96%87%E6%A1%A3/)实现。
开发约定参考仓库 `README_dev.md`、`unitree/g1`、`x-humanoid/tianyi2.0`，以及 Agent Core 的 MCP / ACP 契约。
补充核查了官方 WebSocket SDK、轮臂真机示例和控制器源码；仓库位置、固定提交、字段映射及版本差异见 [SDK_ANALYSIS.md](SDK_ANALYSIS.md)。
2026-11-16 真机上位机的双机 ROS 拓扑、控制接口状态及 DDS 配置检查见 [ROBOT_INSPECTION.md](ROBOT_INSPECTION.md)。
SDK 底层同样是 `roslibpy → rosbridge → ROS1`。本驱动保留同协议的薄适配层，不要求在 ROS2 容器里安装完整机器人 SDK。

## 接入架构

机器人侧已确认只有 `/opt/ros/noetic`。本驱动采用两个独立运行环境：

```text
Kuavo ROS1 Noetic 控制器
  ↕ 官方 ROS1 topics / services
rosbridge_server（机器人侧，WebSocket :9090）
  ↕ rosbridge v2 JSON 协议
本驱动（ROS2 Humble 容器，MCP :15747/mcp）
  → ROS2 std_msgs/String，domain 42，FastDDS loopback profile
Agent Core（同一主机，https://localhost:15678）
```

ROS1 没有 DDS domain。本驱动负责 ROS1 → ROS2 消息转换，Agent Core 侧固定为 domain **42**。
`deploy/service.yml` 使用 host 网络，并挂载、选择 `/opt/phanthy-motus/dds-local.xml`，与本机 Agent Core 的隔离传输保持一致。
启动前检查该 profile 是文件；缺失、误挂成目录或 RMW 配置冲突时拒绝创建 ROS2 participant。

ROS2 / Agent Core 与驱动容器需要在同一 Linux 主机；机器人 ROS1 可以在本机或另一台可达主机。
容器内提供 ROS2 Humble，机器人宿主机无需额外安装 ROS2，也不需要构建 ROS2 版 `kuavo_msgs`。
ROS1 自定义消息的加载由机器人侧 rosbridge 完成。

## 机器人侧准备

`/opt/ros/noetic` 的存在不代表已安装 rosbridge。先在机器人侧准备包：

```sh
sudo apt-get update
sudo apt-get install ros-noetic-rosbridge-server
source /opt/ros/noetic/setup.bash
# 接着 source 机器人实际使用的 Kuavo catkin 工作空间 devel/setup.bash。
# 使用真实工作空间路径，确保 kuavo_msgs、ocs2_msgs 等厂商包可被 ROS 找到。
roslaunch rosbridge_server rosbridge_websocket.launch address:=127.0.0.1 port:=9090
```

上述地址适用于驱动与机器人 ROS1 在同一主机。远程连接时，将 `address` 设置为机器人可达的网卡地址，
并将驱动的 `ROSBRIDGE_URL` 设置为对应 WebSocket URL；该接口能够发布运动命令，应仅在受控机器人网络上开放。
ROS master、多机 ROS 的 `ROS_MASTER_URI` / `ROS_IP` 应在启动 rosbridge 的 ROS1 环境中正确配置。

启动前可检查：

```sh
rostopic type /mobile_manipulator/lb_mpc_control_mode  # std_msgs/Int8
rostopic type /enable_control_state                  # std_msgs/Bool
rosservice type /mobile_manipulator_mpc_control      # kuavo_msgs/changeTorsoCtrlMode
rosservice type /rosapi/get_param                    # rosapi/GetParam
rosparam get /robot_type                             # 1 = wheel-arm
rosparam get /use_shm_communication                   # 决定传感器话题；缺省 false
rostopic type /sensors_data_raw                      # kuavo_msgs/sensorsData；SHM 模式加 _shm
rostopic type /odom                                  # nav_msgs/Odometry
rosmsg show kuavo_msgs/robotHeadMotionData
rosmsg show kuavo_msgs/twoArmHandPoseCmd
```

本驱动不会自动启动机器人控制器、启用控制、切换 MPC 模式或开启 quick mode。
默认 `motion_enabled: false`，允许读取状态、传感器和本地模型，但拒绝运动、恢复控制及模式切换。
软件暂停和 `stop_move` 仍可调用。现场核实接口、单位及停止语义后，仅在有人监督的联调环境中，
将实际加载的配置文件设置为 `motion_enabled: true` 并重启驱动；配置必须是 YAML 布尔值，不能写字符串。
`state.info.feedback.motion_enabled` 与执行卡 `info.motion_enabled` 可确认当前配置。
机器人必须处于允许运动的状态，且收到本次 WebSocket 连接内的控制状态和新鲜模式反馈后才接受运动指令。
启动和重连后，独立线程经 rosapi 探测机型、robot_version、自由度、SHM 参数及 `/humanoid_description`。
必须确认 wheel-arm=1、下肢4、双臂14、头2后才能运动；探测失败时可通过 `state.info.feedback.robot_discovery` 查原因。
参数缺失不回退到 SDK 的双足默认配置。机器人控制器/模型配置更改后应重新连接驱动。

## 卡片与官方接口

| 卡片 | 动作 | 官方 ROS1 接口 |
| --- | --- | --- |
| `loco` | `move`、`move_relative`、`move_to`、`stop_move` | `/cmd_vel`、`/cmd_pose`、`/cmd_pose_world` |
| `arm` | `move_joints`、`set_pose`、`stop_move` | `/kuavo_arm_traj`、`/mm/two_arm_hand_pose_cmd` |
| `leg` | `move_joints`、`stop_move` | `/lb_leg_traj` |
| `waist` | `set_pose`、`stop_move` | `/cmd_lb_torso_pose` |
| `head` | `move`、`stop_move` | `/cmd_head_vel` |
| `switch_mode` | `pause`、`resume`、`set_mode`、`get_current_mode` | `/enable_control`、`/mobile_manipulator_mpc_control` 及状态话题 |
| `state` | 系统 `start`、`stop`、`info` | MPC 模式、软件控制状态、reach-time、`/humanoid_wheel/eePoses` |
| `joints` | 系统 `start`、`stop`、`info` | `/sensors_data_raw[_shm]` 实测 q/dq/ddq/tau；`sensor/skeleton` |
| `imu` | 系统 `start`、`stop`、`info` | 同一原始消息的 IMU；`data/json` |
| `loco_state` | 系统 `start`、`stop`、`info` | `/odom` 本体系 vx/vy/wz；`state/odom`、`motus.odom/1` |
| `model` | resource 读取 | 本地 `resource/kuavo5w_model.urdf` 参考骨架；离线可读 |

sensor/actuator 卡片实现系统生命周期动作；`model` 为读取型 resource。
业务参数通过 `x-action-params` 分组并声明动作各自的 required；`dispatch()` 返回普通 dict，HTTP 层只包装一次 MCP content。
`config.yaml` 的 `plugins.<card>.enabled` 决定卡片是否暴露。

参数单位与约束：

- 底盘速度：本体系 `vx/vy` 为 m/s，`vyaw` 为 rad/s。驱动限制分别为 ±0.3、±0.3、±0.5，`duration` 必填，范围 0.1–10s。
- 底盘位姿：`x/y` 为米，`yaw` 为弧度，三个值均必填。`move_relative` 相对当前位置，`move_to` 使用机器人启动世界系。
- 双臂关节：`positions` 必须是左臂7 + 右臂7共14个角度，单位**度**。
- 双手末端：`frame` 必填 `world` 或 `local`，分别对应官方 frame=1/2；左右手均需完整 xyz（米）和单位四元数 xyzw。
  不启用文档标为暂不支持的自定义 IK 参数或肘部约束。
- 下肢：`positions` 必须为4个角度，单位**度**。
- 躯干：`x/z` 米、`pitch/yaw` 弧度，四个值均必填。文档不支持的 y/roll 不暴露。
- 头部：`yaw/pitch` 为**度/秒**，驱动保守限制 ±30/±20，`duration` 必填，0.1–10s。
  机器人角度限位由官方控制器执行。暂未暴露缺乏到达反馈的头部一次性增量指令。

上述速度和时长上限属于**驱动策略**，不代表厂家最大能力或完整的工作空间、关节限位及碰撞保护。
末端、下肢和躯干目标需由调用方根据机器人实际安全工作空间设置。

模式：0=NoControl（被动）、1=ArmOnly、2=BaseOnly、3=BaseArm、4=ArmEeOnly。
底盘要求模式2/3；双臂关节要求1/3；末端允许1/3/4。下肢、躯干和头部要求非0模式。
驱动不会根据动作隐式切换模式，也不会调用 `/enable_lb_arm_quick_mode`：该服务会绕过 MPC。
部署时应确认其他客户端没有开启 quick mode；本驱动没有该模式的官方反馈来源，不能自动判断外部设置。

## ACP、并发和停止语义

底盘、双臂、下肢、躯干和头部的运动动作均声明 `x-completion`，返回唯一 `action_id`，
结束后 POST `${AGENT_CORE_URL}/api/acp/complete`。资源分别为 `base`、`arm_l/arm_r`、`lower_body`、`head`；
`leg` 与 `waist` 共享资源和驱动内部通道。同通道在途动作收到新命令会返回 busy，调用方应等待完成或显式停止。
底盘速度/位姿也共享 busy 通道，避免位置指令被尚未过期的速度指令覆盖，或无效新参数误取消旧 ACP。
`switch_mode` 不指定局部资源，采用 Agent Core 的全局互斥；其短状态转换按指南同步等待反馈。
暂停、恢复和模式切换共用最多2s的服务受理+状态确认预算，而不是分别等待5s和2s；
前台发令争用控制锁最多等待0.25s，未取得锁时明确返回 busy。反馈超时返回错误，要求查询状态后再决定，
不自动重试或改为后台继续等待，确认期间重连也不能以新会话反馈证明旧请求成功。
所有执行卡声明 `on_interrupt_motion` hook。

位姿/关节运动订阅相应的 `/lb_*_reach_time`（`std_msgs/Float32`），只使用指令发送后收到的反馈。
反馈须在3s内到达，估计值接受0–110s；ACP fallback timeout 为125s。
双臂/下肢 `move_joints` 发令前要求0.5s内实测关节位置；指令后收到新 reach-time，并等过预计时长后，
还要求实测目标关节误差≤0.035rad，连续新测量覆盖≥0.2s，再返回 `measured_convergence: true`。
预计结束后5s内未收敛、实测丢失/异常会返回 ACP error 并尝试全身软件暂停。
底盘和躯干/双手笛卡尔动作的完成时间仍依据**官方到达时间估计**，明确标记 `measured_convergence: false`。
这些反馈没有命令 ID，因此需要保证本驱动独占所用命令话题；外部客户端同时发令可能导致反馈归属不明。

速度动作以20Hz发送，时长到达后归零，底盘额外等待文档规定的1s停止窗口，头部等待0.3s。
ACP 同样标记为时长/控制器超时推断。WebSocket发送成功仅代表写入连接，不代表 ROS 控制器确认执行。
发布器在连接时预先建立，并等待1s连接准备窗口；一次性指令仍以 reach-time 反馈检查接收情况。

执行卡系统 `start`/`stop` 是立即返回 `ready`/`idle` 的生命周期标记，不访问机器人或等待控制锁。
`info` 和 `state` 另行报告实际连接状态；生命周期 `ready` 不表示已连接或允许运动。
已接受的动作仍由驱动运行时和 ACP 跟踪，不因生命周期标记而丢失；需要停止运动时必须调用
`stop_move`、`switch_mode.pause` 或触发执行卡的中断 hook。驱动退出时会清理在途动作。

`stop_move` 和中断操作会发底盘/头部零速度并请求 `/enable_control=false` **全身软件暂停**，
以避免位置目标在零速度命令退出后继续执行；它们会影响其他部位并取消所有在途 ACP。
之后需显式 `switch_mode.resume`，再发送新的运动目标。
服务 `success=true` 只表示受理，暂停/恢复以 `/enable_control_state` 确认；保护窗口或重复请求被拒会返回错误。
暂停确认失败时返回 `physical_stop_confirmed: false`，并继续阻止本驱动发运动命令。

发布、取消和暂停使用同一把锁，ACP HTTP 回调在独立队列中执行，不阻塞零速发送。
断线、模式反馈超过2s未更新、控制暂停或连接代次变化都会终止在途动作并尝试软件暂停。
**断网时软件指令不能保证停止真机**，物理安全仍需机器人原有控制器和现场急停保障。
回调失败有日志和有限重试，驱动退出有有限的回调排空时间。
重连仅恢复发布器/订阅，清除旧反馈，绝不重放运动命令或重试超时服务。

## 状态转发与能力边界

`state.start` 后以5Hz发布 `/<namespace>/state/control`，类型 `std_msgs/String`，格式 `data/json`。
`state.info` 随时返回相同快照。内容包含连接代次、模式反馈新鲜度、软件控制状态、各字段接收时间/年龄、
reach-time 以及实测双手世界系末端位姿（左6 + 右6，各为 `[x,y,z,yaw,pitch,roll]`，米/弧度）。
断线/重连会清除上一连接的数值；停止状态卡输出不会停止用于运动检查的内部反馈订阅。
即使断线，已启动的状态/传感卡仍发布明确 disconnected/invalid 消息，避免保留“已连接”的旧显示。

新增传感卡均按10Hz输出 `std_msgs/String` 到 Agent Core 的 domain42：

| 卡片 | 输出话题 | 内容 |
| --- | --- | --- |
| `joints` | `/<namespace>/state/joints` | 下肢4+左臂7+右臂7+头2，q rad、dq rad/s、ddq rad/s²；名称从实际 URDF 的已核实 child-link 顺序映射；原始 effort 保留在 vendor |
| `imu` | `/<namespace>/state/imu` | quaternion 明确 wxyz、gyroscope rad/s、accelerometer/free_acceleration m/s²；不猜安装 frame |
| `loco_state` | `/<namespace>/state/odom` | motus.odom/1，本体系 vx/vy/wz 窗口均值；其他轴 null，原始 pose/twist/header 位于 vendor |

0.5s无测量或数据不合法会输出 `valid:false`；缺失力矩/速度不补零。
`model` 与 G1、天轶 2.0 一样读取驱动 `resource/` 下的 URDF，前端无需 STL 即可显示静态骨架，
不依赖机器人在线或控制器启动。模型来源、固定版本和许可证见 [resource/README.md](resource/README.md)。
该文件是 400062 参考模型，尚未核实与现场硬件修订及标定完全一致，不能用作控制模型。
实时更新仍要求实测 q；真机 `/humanoid_description` 缺失，或其20个受测关节名称与本地模型不一致时，
`joints` 输出 invalid、空关节数组和原因，静态 `model` 仍可读取。`state.info` 中的
`robot_discovery.model_error` 表示运行时模型校验错误，`skeleton_model` 记录本地来源与名称校验状态。
参考姿态不代表测得的 q=0。原始20关节不含底盘轮角/手指；这些不属于本卡的实测能力。

SDK 对 effort 单位同时允许 N·m 或 A，因此默认 tau=null、单位未知，保留 `vendor.joint_torque`。
只有在真机核实所有输出为关节端 N·m 后才配置 `sensors.joint_torque_unit: N*m`，届时才输出标准 tau。

`loco_state.info` 返回 `odom_interface`；消息携带时间来源、原始 frame、窗口样本数，空窗口不重发旧速度。
底盘里程计的只读输出独立于全身控制栈的启动/机型探测；探测失败会阻止运动，不会阻止读取已存在的有效 `/odom`。
默认仅接受 child_frame_id 为 base_link/base_footprint 的 odom；真机名称不同时先核实 TF/发布器，再配置 `sensors.odom_body_frames`。
不将未经定位和漂移核实的 pose 提升为标准导航位姿。

未将 `currentMpcTarget`、`optimizedState_mrt` 等期望量当作传感器；尚未实现 `motus.control/1` 连续控制入口。
未因通用 SDK 列出相机、电池、手爪、语音或双足步态，就假定该台 5-W 安装并开放了这些功能。

## 构建和部署

仓库 `build.sh` 会将驱动目录内容作为构建上下文，并将 `driver.yaml` 的 `../../common` 复制为 `common/`。
Dockerfile 已按此布局配置，并将部署片段放入镜像 `/deploy/service.yml`。
基础镜像已提供 `python3-pip`；Dockerfile 仅在缺失时安装 ROS2 FastDDS RMW 和 `std_msgs`，
分别用于选定的 DDS participant 和 `String` 状态发布。不安装本地 ROS2 `geometry_msgs`：
厂商 ROS1 `Twist` 是 rosbridge JSON 消息，由机器人侧解析。
Python 依赖仅为配置解析用 PyYAML 和连接 rosbridge 用 websocket-client，不安装完整厂商 SDK。
这些是运行所需的依赖；实际基础镜像是否已提供及最终镜像增量仍需容器构建核实，不能用离线测试替代。
本地仅构建、不推送的示例（从仓库根目录运行）：

```sh
BUILD_CTX=$(mktemp -d)
cp -R lejurobot/kuavo5-w/. "$BUILD_CTX/"
cp -R common "$BUILD_CTX/common"
docker build --platform linux/arm64 -t lejurobot-kuavo5-w:dev "$BUILD_CTX"
```

通过 Agent Core 安装时使用 `deploy/service.yml`。默认连接 `ws://127.0.0.1:9090`；
远程 ROS1 可通过部署环境 `KUAVO_ROSBRIDGE_URL=ws://<robot-ip>:9090` 覆盖。
直接启动 Python 时使用 `ROSBRIDGE_URL`；`CONFIG_PATH` 可指定配置文件，`ros_namespace` 默认使用规范化后的 hostname。
MCP 端口15747，Agent Core回调默认 `https://localhost:15678`，可通过 `AGENT_CORE_URL` 覆盖。
注册请求携带与 `driver.yaml` 一致的 id、名称、URL、`transport: http` 和 `category: driver`。
当前 Core 的注册匹配仍以 URL/名称为准，不依赖请求 id；显式字段与共享注册工具保持一致。
本驱动使用局部 topic 命名空间，不修改其他驱动或 Agent Core 的 DDS domain。

## 检查与真机验收

从仓库根目录运行离线检查（只需 Python、PyYAML，无需 ROS）：

```sh
PYTHONPYCACHEPREFIX=/tmp/kuavo5w-pycache python3 -m unittest discover -s lejurobot/kuavo5-w/tests -v
PYTHONPYCACHEPREFIX=/tmp/kuavo5w-pycache python3 -m py_compile lejurobot/kuavo5-w/*.py
python3 scripts/check_service_yml.py lejurobot/kuavo5-w
```

MCP 探测示例（只查询状态）：

```sh
curl -s http://localhost:15747/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}'
curl -s http://localhost:15747/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"state","arguments":{"action":"info"}}}'
```

真机仍需验证：

1. ROS1 上述类型/服务正确，rosbridge/rosapi 能加载厂商消息和参数，模式持续50Hz更新，软件控制状态可接收。
2. 分别 start 传感卡，确认 domain42 收到 control/joints/imu/odom，字段、URDF名称、方向和 frame 正确，外部网络不接收该 DDS 数据。
3. 仅在现场有人监督且接口就绪时显式开启 `motion_enabled`，分别以小幅度验证底盘、双臂、下肢、躯干和头部的方向与单位；检查每个动作的 ACP 成功/取消/错误。
4. 验证软件暂停、过渡窗口拒绝、恢复后需重新发目标、rosbridge 断线后不重放指令，以及多个执行卡共享通道的仲裁。
5. 检查关节 ACP 在实测到位后才成功；卡住、数据中断或误差超限不能成功。测量位姿 reach-time 估计的偏差，评估进一步闭环。

离线测试不能替代容器构建、DDS 互通和真机运动验证。
