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
  → ROS2 String / UInt8MultiArray / CompressedImage，domain 42，FastDDS loopback profile
Agent Core（同一主机，https://localhost:15678）
```

ROS1 没有 DDS domain。本驱动负责 ROS1 → ROS2 消息转换，Agent Core 侧固定为 domain **42**。
`deploy/service.yml` 使用 host 网络，并挂载、选择 `/opt/phanthy-motus/dds-local.xml`，与本机 Agent Core 的隔离传输保持一致。
本驱动只通过 WebSocket/DDS 接口访问机器人，不直接访问设备，不申请 `privileged`，
也不挂载可写 data 目录；仅挂载只读 DDS profile 与操作人员配置文件。
启动前检查该 profile 是文件；缺失、误挂成目录或 RMW 配置冲突时拒绝创建 ROS2 participant。

ROS2 / Agent Core 与驱动容器需要在同一 Linux 主机；机器人 ROS1 可以在本机或另一台可达主机。
容器内提供 ROS2 Humble，机器人宿主机无需额外安装 ROS2，也不需要构建 ROS2 版 `kuavo_msgs`。
ROS1 自定义消息的加载由机器人侧 rosbridge 完成。
MCP 仅监听 `127.0.0.1:15747`，只接受本机无 `Origin` 的服务端请求，并校验 `Host` 为该端口的
`localhost`/`127.0.0.1`；浏览器预检、任意浏览器来源及外部客户端均被拒绝，不返回通配 CORS。
页面通过 Agent Core 调用驱动，不直接访问运动端口。这是同机进程信任边界，不是本机进程之间的身份鉴权。

### 容器部署配置

Dashboard 将镜像内的 `deploy/service.yml` 合并到宿主机 Compose，保留其中的配置挂载和环境变量。
部署片段把宿主机 `/opt/phanthy-motus/drivers/lejurobot-kuavo5-w/config.yaml` **只读**挂到
`/config/kuavo5-w.yaml`，并设置 `CONFIG_PATH` 指向它。操作人员在首次 Dashboard 部署前准备该文件：

```sh
# 在驱动仓库根目录执行，目标是运行 Agent Core / 驱动容器的 Linux 主机。
# 首次初始化使用；已有现场配置时不要覆盖。
sudo install -d -m 0755 /opt/phanthy-motus/drivers/lejurobot-kuavo5-w
sudo test ! -e /opt/phanthy-motus/drivers/lejurobot-kuavo5-w/config.yaml && \
  sudo install -m 0644 lejurobot/kuavo5-w/config.yaml /opt/phanthy-motus/drivers/lejurobot-kuavo5-w/config.yaml
```

没有源码的部署主机也可先从所选驱动镜像的停止容器中用 `docker cp` 提取 `/work/config.yaml`，
再安装到同一宿主机路径；不需要启动控制程序或重建镜像。挂载要求 Docker Compose 支持
`bind.create_host_path: false`；文件缺失时部署失败，不自动创建同名目录。
文件应是完整配置，保留默认的 `sensors.joint_limits: null`，现场核验后再填写下节记录。
修改宿主机文件后需重新创建驱动容器以重新加载并更新文件挂载，例如在宿主机 Compose 目录执行
`docker compose up -d --force-recreate lejurobot-kuavo5-w`；重新创建会中断当前驱动，应在机器人受控停止后进行。
Dashboard 重部署也复用此路径，不会用镜像默认文件覆盖现场配置。该路径不提供 Dashboard 在线编辑功能。

直接运行源码时，未设置 `CONFIG_PATH` 仍读取同目录的 `config.yaml`；显式设置的路径打不开或 YAML 无法解析时
启动失败，不回退到包内默认配置。配置和硬件核验记录由操作人员管理，不应提交到仓库。

镜像依赖会有体积成本：FastDDS RMW / std_msgs / sensor_msgs 仅在基础镜像未提供时安装；`requirements.txt`
声明 PyYAML（配置解析）及 websocket-client（rosbridge）。构建时先实际尝试 `import yaml`，已可用就不再
通过 pip 安装 PyYAML，否则按 requirements 补齐；websocket-client 仍由 requirements 管理，最后检查两者均可导入。
平台 `deploy/ros-base/Dockerfile` 未直接声明 PyYAML，其上游 ROS 镜像是否提供不能仅凭该文件确定。
尚未对生产 Dockerfile 与 ros-base 完成全量构建体积对比，不能承诺零增长。
现场增量候选镜像基于已有 release 构建，只验证当前依赖可复用，不代表生产基础镜像无需安装缺失包。

## 机器人侧准备

### 一次安装，后续驱动启动自动检查

在 ROS1 上位机安装 `ros-noetic-rosbridge-server` 后，从本目录执行一次（参数使用现场 ROS 网络）：

```sh
sudo python3 host/install.py --user leju_kuavo \
  --workspace /home/leju_kuavo/kuavo_ros_application \
  --master http://192.168.26.1:11311 --ros-ip 192.168.26.12
```

安装器准备现场配置（保留已有值，仅补充缺失的外设插件），安装固定用途 systemd 服务。
驱动启动后每5秒通过 `127.0.0.1:15748/ensure` 请求确保 rosbridge、只读电池转发器运行；
它们异常退出由 systemd 重启。`switch_mode.get_current_mode` 的 `feedback.host_runtime` 报告检查状态。
容器不需要宿主机 sudo、SSH 密码、Docker socket 或特权权限。
头部 RGB/深度卡在 `start` 缺帧时分别请求 `/ensure/head`、`/ensure/depth`，确保固定相机和相应 relay 服务运行。
助手仅能启动 rosbridge、电池 relay 及已安装的固定头部 RGB/深度服务，不接受命令或 unit 参数；仅监听 loopback，拒绝浏览器 Origin 和非本机 Host。
ROS消息解码以普通用户运行，固定启动助手独立运行。旧版 rosbridge 忽略 address 参数时，通过进程内 wrapper
明确指定 loopback 绑定，并在进程内为 ROS1 订阅启用 TCP_NODELAY、2MB 接收缓冲；不修改系统 ROS 包。
保留全部 odom 接收样本，不通过 queue_size=1 在输入端丢弃测量。

若上下位机 `kuavo_msgs/sensorsData` 的 MD5 不同，即使存在500Hz发布者，TCPROS也会拒绝订阅。
从实际控制器工作区复制其生成的 `kuavo_msgs` Python包（包含msg和srv）到上位机独立目录，
安装时添加 `--message-overlay /path/to/controller-python-packages`。该目录直接包含 `kuavo_msgs/`，
不是源码msg目录；应与当前控制器版本同步，不能用另一版本猜测字段。安装器复制到自身 `ros1_messages/`，
仅bridge和rosapi进程优先加载，不覆盖厂商工作区或系统ROS包。未传该参数时保留已有覆盖层。
现场版本63的sensorsData含FTsensor_data，已验证用该方法恢复真实20关节数据。

### 下位机开机自动初始化并暂停

全身控制栈属于下位机硬件进程，不由上位机驱动容器反复创建。现场版本63已获操作人员明确授权：
每次下位机开机自动启动并初始化，完成后进入软件暂停。初始化会使能电机并可能调整姿态；
版本、标定、遥控器与开机活动范围必须先核实，不能照搬到其他硬件。

将 `host/controller_boot.py`、`host/install_controller.py` 放到下位机同一目录，显式安装一次：

```sh
sudo python3 host/install_controller.py \
  --workspace /home/lab/kuavo-ros-opensource \
  --master http://192.168.26.1:11311 --ros-ip 192.168.26.1 \
  --confirm-auto-initialize-and-pause
```

安装器只enable `kuavo5w-controller.service`，不停止或替换当前控制器。
如需在本次开机启用监管，可显式 `sudo systemctl start kuavo5w-controller.service`：
发现已有控制栈时只观察，不重复启动、初始化或发暂停；仅当其版本、20关节实测、初始化标志及
新收到的暂停反馈均符合要求才接管状态，报告 `launch_ownership: external`。
已有控制器未就绪或未暂停时保持等待，不能抢占另一个控制者。

正常下次开机，服务等待厂商H12服务建立ROS master，确认没有已有控制器后启动
`ROBOT_VERSION=63 roslaunch humanoid_controllers load_kuavo_real_wheel.launch joystick_type:=h12 start_way:=auto`。
核实机型、关节、服务来源后仅调用一次 `/humanoid_controller/real_initial_start`；
等待请求终态及之后的新鲜MPC反馈，再请求 `/enable_control(false)`，确认新收到的false后才报告complete。
初始化超时保留请求，不重发；失败不假报就绪。控制进程故障不自动重启，也不调用enable=true。
服务自有进程退出后需要现场检查再手动处理，不能用自动重启掩盖急停或硬件故障。

确认下位机服务正常后，在上位机现场配置设 `host_runtime.controller_boot_required: true`，
驱动的运动、resume和模式切换会等待开机流程确认；pause始终允许尝试。
`switch_mode.get_current_mode.feedback.controller_boot` 展示结果；
`boot_pause_confirmed` 仅代表开机流程曾确认暂停，当前使能仍以独立 `control_enabled` 反馈为准。
开机状态超过3秒未更新会阻止运动；一旦发现开机服务，本次驱动会话即使重连也保留该要求。
兼容旧手动部署，随包配置默认false；不应在已使用开机服务的现场关闭检查绕过故障。

容器重启只重连这些服务，不重新初始化或自动恢复运动。下位机开机服务和上位机安装器均须部署，
仅有容器的restart策略不能恢复另一台机器上的电机进程。
本次已验证真实20关节反馈、MPC模式、软件暂停/恢复及当前已暂停程序的只读接管；
尚未进行实际关机重启，也未验证位移或关节目标运动。arm/leg仍需要下文的硬件核验限位配置。

默认自动启动 `joints/imu/loco_state/battery/laser_scan/camera_left/camera_right/camera_head/camera_depth` 的输出。
卡片 `start` 先检查是否有新鲜图像；缺帧时异步请求启动，左右相机调用对应 `set_camera_enabled(true)`，头部卡请求上述宿主机服务。
运行期间缺帧每5秒重试，已有新鲜帧不重复开启；未运行的头部卡不由全局检查器启动。不会启动 WBC、MPC 或使能机器人运动。
`sensors.auto_start` 可缩小列表，`plugins.<name>.enabled: false` 可禁用卡片。
远程 rosbridge 部署可设置 `host_runtime.enabled: false`，由远端自行管理其 ROS1 服务。
首次缺少宿主机配置导致 bind 挂载失败时，容器代码尚未运行，仍需一次安装/初始化；无法靠容器内自检绕过。

### 外设卡与现场型号

| 卡片 | 数据与启动行为 |
| --- | --- |
| `battery` | `/battery` 主电池的 SOC(%)、电流(A)、温度(°C)；保留另两路厂商来源及不同 status，不把主电池0V当成有效电压 |
| `laser_scan` | 首路 `sensor/pointcloud` 原生二进制点云，次路原始 `/scan` JSON；保留全部有效点，laser 坐标、z=0，不推测底盘 TF |
| `camera_left` | 左路 `/camera_1/color/image_raw/compressed`，原生 ROS2 `CompressedImage` / `image/jpeg` |
| `camera_right` | 右路 `/camera_2/color/image_raw/compressed`，同上；保留源时间戳，空 frame_id 保持为空 |
| `camera_head` | 头部 Orbbec Gemini335L，640×480 RGB JPEG；宿主机已有 Orbbec ROS1 驱动和 OpenCV/numpy 时安装转发 |
| `camera_depth` | 头部原生640×480 uint16毫米深度，`image/depth-zlib` 无损压缩；0为无效距离，未对齐RGB |
| `imu` | 优先独立 `/imu`，全身 raw 为备用；不再要求全身控制器存在才能读取 IMU |

电池转发器校验现场三种已核实 topic/type/MD5 组合，动态加载发布者提供的有界定义，输出标准 String。
四路图像分别使用独立 WebSocket 接收器和发布线程，目标上限10Hz；只保留最新帧，DDS BEST_EFFORT/depth1。
底盘 JPEG 不重新编码；头部通过宿主机已有 OpenCV/numpy 编码，未增加驱动镜像依赖。
现场旧版 ROS1 缺少 compressed_image_transport 插件，`host/head_jpeg.py` 保留原始 header、处理行填充，
以 latest-only 方式将头部原始 RGB 转 JPEG。按现场授权，头部共用RGBD进程开启深度及深度激光，RGB/深度均为640×480@15Hz源流。
`host/head_depth.py` 保留原始header，将已由厂商驱动换算为毫米的16UC1图像转换为小端无损zlib；不使用PNG/RVL compressedDepth格式，不增加镜像依赖。
RGB和深度卡共享物理相机进程，启动任一头部卡可能同时打开两路源流；只有处于运行状态的卡输出DDS数据。
WebSocket 使用 CPython 严格 UTF-8 解码和 JSON 校验，避免纯 Python 逐字节校验造成积压，不放宽非法编码检查。
安装器自动检测 `/home/<user>/kuavo_data_pilot` 或指定 `--head-workspace`，仅在找到已构建 Orbbec 驱动时安装固定头部服务。
底盘源时间戳为0，无法据此测得端到端延迟；卡片 info 中 publish_delay_ms 仅是驱动接收至 DDS 发布耗时。
传感卡 stop 停止该卡输出，相机 stop 不关闭其他卡或进程共享的物理图像流。
现场 USB 音频说明要求 EB03_LJ 声卡，上位机未枚举到该卡；Pulse source 仅为播放 monitor，不是麦克风。
ALSA PCM 可写不证明真实扬声器出声，因此不注册虚假可用的 `mic/speaker`。接线/供电尚未确认。
已发现 Livox MID360 配置和 SDK，但专用网口无物理 carrier，未见第二雷达有效发布者；当前点云只来自底盘二维雷达。

`KUA5W0710701302138` 是操作人员提供的机身标识，可在现场配置 `robot_serial` 中记录；
没有证据可把它等同于控制器 `robot_version`。当前资源URDF来自现场版本63控制器的biped_s63，
20关节名称及文件几何已对齐；未独立测量实物尺寸或校准。
`joints.info.skeleton_model` 明示 `reference_only` / `hardware_geometry_verified`；缺少全身 `/humanoid_description`
或实测关节流时不伪造姿态。现场 `/robot_description` 是底盘模型，不能替代全身 URDF。
必须取得匹配的全身模型、关节顺序及现场核验限位后才能验证关节控制。

### 手动安装与排查

`/opt/ros/noetic` 的存在不代表已安装 rosbridge。未使用上面的安装器时，可手动准备包并排查：

```sh
sudo apt-get update
sudo apt-get install ros-noetic-rosbridge-server
source /opt/ros/noetic/setup.bash
# 接着 source 机器人实际使用的 Kuavo catkin 工作空间 devel/setup.bash。
# 使用真实工作空间路径，确保 kuavo_msgs、ocs2_msgs 等厂商包可被 ROS 找到。
roslaunch rosbridge_server rosbridge_websocket.launch address:=127.0.0.1 port:=9090
```

上述地址适用于驱动与机器人 ROS1 在同一主机。远程连接时，将 `address` 设置为机器人可达的网卡地址，
并将宿主机操作配置的 `rosbridge_url` 设置为对应 WebSocket URL；需要覆盖配置时，在 Compose 执行环境或
其 `.env` 中显式设置 `KUAVO_ROSBRIDGE_URL`。未设置或为空时使用配置值，不强制 localhost。
直接运行源码可用非空的 `ROSBRIDGE_URL` 覆盖配置；两者都未配置时才默认 `ws://127.0.0.1:9090`。
该接口能够发布运动命令，应仅在受控机器人网络上开放。
ROS master、多机 ROS 的 `ROS_MASTER_URI` / `ROS_IP` 应在启动 rosbridge 的 ROS1 环境中正确配置。

启动前可检查：

```sh
rostopic type /mobile_manipulator/lb_mpc_control_mode  # std_msgs/Int8
rostopic type /enable_control_state                  # std_msgs/Bool
rosservice type /mobile_manipulator_mpc_control      # kuavo_msgs/changeTorsoCtrlMode
rosservice type /rosapi/get_param                    # rosapi/GetParam
rosservice type /rosapi/publishers                   # rosapi/Publishers，只读使能来源检查
rosparam get /robot_type                             # 1 = wheel-arm
rosparam get /use_shm_communication                   # 决定传感器话题；缺省 false
rostopic type /sensors_data_raw                      # kuavo_msgs/sensorsData；SHM 模式加 _shm
rostopic type /odom                                  # nav_msgs/Odometry
rosmsg show kuavo_msgs/robotHeadMotionData
rosmsg show kuavo_msgs/twoArmHandPoseCmd
```

下位机已显式安装的开机服务负责初始化后暂停；驱动容器本身不创建硬件控制进程。
普通执行动作会按需恢复控制并选择兼容MPC模式，无需先执行 `switch_mode.resume/set_mode`。
`motion_enabled` 默认 `true`，未配置该字段时也默认开启；双臂/下肢关节动作还必须配置现场核验的限位，见下节。
该开关仅允许处理运动请求，后台检查、读取传感器、卡片start/info不会触发恢复或发运动目标；原有运行时检查仍生效。
如需只读运行，可在实际加载的配置文件中设置 `motion_enabled: false` 并重启驱动；此时拒绝运动、
恢复控制及模式切换，仍允许状态/传感器/模型读取、软件暂停和 `stop_move`。该字段必须是 YAML 布尔值。
`switch_mode.get_current_mode.feedback.motion_enabled` 与执行卡 `info.motion_enabled` 可确认当前配置。
机器人必须处于允许运动的状态，且收到本次 WebSocket 连接内的控制状态和新鲜模式反馈后才接受运动指令。
启动和重连后，独立线程经 rosapi 探测机型、robot_version、自由度、SHM 参数及 `/humanoid_description`。
必须确认 wheel-arm=1、下肢4、双臂14、头2后才能运动；探测失败时可通过 `switch_mode.get_current_mode.feedback.robot_discovery` 查原因。
参数缺失不回退到 SDK 的双足默认配置。后台探测在每轮结束约1s后重新读取完整身份参数和 URDF；
成功读取的身份有效期为3s，从该轮读取开始计时，慢查询不会在完成时伪造新租约。
查询失败、身份过期或模型/限位绑定变化会拒绝/终止相关动作，不依赖 WebSocket 重连；
相同来源和连接代次不重复订阅原始传感器。状态在 `robot_discovery.identity_verification` 查看。
轮询不是参数变更通知或原子快照，仍存在检测窗口；必须在机器人停止且无在途动作时修改控制器参数，
不支持边运动边更换模型或标定。现场还需确认 rosapi 查询时延满足3s有效期。

## 卡片与官方接口

| 卡片 | 动作 | 官方 ROS1 接口 |
| --- | --- | --- |
| `loco` | `move`、`move_relative`、`move_to`、`stop_move` | `/cmd_vel`、`/cmd_pose`、`/cmd_pose_world` |
| `arm` | `move_joints`、`set_pose`、`stop_move` | `/kuavo_arm_traj`、`/mm/two_arm_hand_pose_cmd` |
| `leg` | `move_joints`、`stop_move` | `/lb_leg_traj` |
| `waist` | `set_pose`、`stop_move` | `/cmd_lb_torso_pose` |
| `head` | `move`、`stop_move` | `/cmd_head_vel` |
| `switch_mode` | `pause`、`resume`、`set_mode`、`get_current_mode` | `/enable_control`、`/mobile_manipulator_mpc_control` 及状态话题 |
| `joints` | 系统 `start`、`stop`、`info` | 两路：`sensor/skeleton` 骨架姿态 + `data/json` 原始 joint_state |
| `imu` | 系统 `start`、`stop`、`info` | 同一原始消息的 IMU；`data/json` |
| `loco_state` | 系统 `start`、`stop`、`info` | 两路：map/odom 位置、角度、速度 JSON + 独立 `state/odom` / `motus.odom/1` |
| `model` | resource 读取 | 本地 `resource/kuavo5w_model.urdf` 参考骨架；离线可读 |

sensor/actuator 卡片实现系统生命周期动作；`model` 为读取型 resource。
业务参数通过 `x-action-params` 分组并声明动作各自的 required；`dispatch()` 返回普通 dict，HTTP 层只包装一次 MCP content。
`config.yaml` 的 `plugins.<card>.enabled` 决定卡片是否暴露。

`switch_mode` 操作全身轮臂 MPC，不是独立底盘导航模式，也不会启动控制栈。
它保留为高级诊断及手动干预入口，不是正常操作的前置卡片：开启智能控制后直接执行loco/arm等动作即可。
`set_mode` 的0–4选项同时显示编号和名称，兼容画布下拉框传来的精确数字字符串，
发送给 ROS 服务时仍为整数。`resume`/`set_mode` 需要真实机型参数及控制反馈；
`controller_not_ready` 表示这些前提缺失，不能通过填入默认参数绕过。
`pause` 不依赖机型探测，仍会尝试零速、取消在途动作及请求暂停；若控制服务/反馈缺失，
返回 `physical_stop_confirmed: false`，不能假报暂停成功。查询返回的 `state: ready` 仅表示查询成功。

左右/头部 RGB、头部深度、电池、二维雷达点云已适配；尚未提供 `mic`、`speaker`、`controlled_spatial` 卡片。
真机底盘另有导航主机 `192.168.26.22`，其 HTTP API 已验证可读当前地图和地图列表；
任务下发、取消确认、建图保存/服务重启的完整协议仍待核实，上位机 rosbridge 也缺少对应自定义服务类型。
用户已确认开机音效，底盘状态包含音乐编号和音量控制，但尚未找到任意 PCM/TTS 的实际播放入口；
不能由上下位机 USB 声卡未枚举推断整机没有扬声器。
接口快照、可行性及导航取消/ACP 适配前提见 `ROBOT_INSPECTION.md` 的能力核查记录。

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
末端和躯干目标仍需由调用方根据机器人实际安全工作空间设置；关节限位检查不等同于碰撞保护。

### 关节限位配置

随包的 `sensors.joint_limits: null` 表示尚未提供硬件核验记录，因此 `arm.move_joints` 和
`leg.move_joints` 会在发布前拒绝，其他动作仍遵循各自的运行时检查。不会从显示用参考 URDF 推断可信控制限位，
也不会把越界目标截断后发送。必须由现场负责人核验实际硬件、标定和控制器配置，再填写以下记录：

| `sensors.joint_limits` 字段 | 要求 |
| --- | --- |
| `verification` | 非空、最多512字符的现场核验记录引用，记录应能追溯机器人硬件及标定 |
| `robot_version` | 与运行时参数的值及类型完全一致 |
| `runtime_urdf_sha256` | 运行时 `/humanoid_description` 原始字符串按 UTF-8 编码的 SHA-256 |
| `arm.joint_names` | 原始20关节顺序的 `[4:18]`，左臂7 + 右臂7，名称/顺序完全一致 |
| `arm.lower_deg` / `arm.upper_deg` | 各14个经现场核验的有限角度值，单位度，逐项 lower < upper |
| `leg.joint_names` | 原始20关节顺序的 `[0:4]`，名称/顺序完全一致 |
| `leg.lower_deg` / `leg.upper_deg` | 各4个经现场核验的有限角度值，单位度，逐项 lower < upper |

可只配置 `arm` 或 `leg`，未配置的卡片关节动作继续拒绝。上下界允许取等值；不接受布尔值、NaN 或 Inf。
`verification` 是操作人员对核验记录的声明，驱动只能校验格式和运行时绑定，不能自动证明限位经过硬件验证。
URDF 名称匹配或文件含有限位也不构成该证明，本说明不提供未经核验的限位数值。
在机器人已 source 正确 ROS 环境后，可只读获取原始参数指纹：

```sh
python3 -c "import hashlib,rospy; print(hashlib.sha256(rospy.get_param('/humanoid_description').encode('utf-8')).hexdigest())"
```

不要对 `rosparam get` 的 YAML 输出计算指纹。配置需重启加载；可用性及核验记录引用在
`switch_mode.get_current_mode.feedback.robot_discovery.joint_limits` 查看。运行时模型指纹或配置身份不匹配时不允许关节动作。

模式：0=NoControl（被动）、1=ArmOnly、2=BaseOnly、3=BaseArm、4=ArmEeOnly。
底盘要求模式2/3；双臂关节要求1/3；末端允许1/3/4。下肢、躯干和头部要求非0模式。
动作参数及关节目标/限位先校验，再按需恢复并等待恢复后的新鲜MPC反馈，最后准备模式及发布目标。
当前模式兼容且反馈新鲜时不重复切换；不兼容或NoControl时选择3=BaseArm，兼顾底盘和双臂。
现场版本63的厂商实现实际固定执行并发布BaseArm：模式服务虽会接受其他编号，
但不会改变实际运行模式。该机器正常动作直接沿用反馈中的BaseArm，不依赖切换到ArmOnly/BaseOnly；
手动请求其他模式时仍会因缺少对应反馈而报错，不把服务受理冒充生效。
恢复后也会重新核实模式；失败或被停止打断时不发送原运动目标，并尝试软件暂停。
执行结果 `control_preparation` 报告是否恢复、是否改变模式及最终模式。
这不是绕过检查的强制恢复：开机未确认、反馈过期、目标越界、发送不确定或旧控制请求未结清仍会拒绝。
驱动不会调用 `/enable_lb_arm_quick_mode`，该服务会绕过MPC。
部署时应确认其他客户端没有开启 quick mode；本驱动没有该模式的官方反馈来源，不能自动判断外部设置。

## ACP、并发和停止语义

底盘、双臂、下肢、躯干和头部的运动动作均声明 `x-completion`，返回唯一 `action_id`，
结束后 POST `${AGENT_CORE_URL}/api/acp/complete`。资源分别为 `base`、`arm_l/arm_r`、`lower_body`、`head`；
`leg` 与 `waist` 共享资源和驱动内部通道。同通道在途动作收到新命令会返回 busy，调用方应等待完成或显式停止。
底盘速度/位姿也共享 busy 通道，避免位置指令被尚未过期的速度指令覆盖，或无效新参数误取消旧 ACP。
`switch_mode` 不指定局部资源，采用 Agent Core 的全局互斥；其短状态转换按指南同步等待反馈。
同为BaseArm时不同资源可并发；若改变全局模式需要影响其他在途动作，则返回busy，不能静默重置它们。
每个暂停、恢复或模式服务分别使用最多2s的“受理+状态确认”总预算，而不是分别等待5s和2s；
一个动作可能依次需要恢复、等待恢复后的MPC流、切换模式，最多三段各2s准备预算，准备阶段等待不持有发布锁。
前台发令争用控制锁最多等待0.25s，未取得锁时明确返回 busy。反馈超时返回错误，要求查询状态后再决定，
不自动重试或改为后台继续等待，确认期间重连也不能以新会话反馈证明旧请求成功。
其中限时锁只适用于普通请求：控制服务采用“锁内验证并发送、锁外等待响应/状态”。
`stop_move`/`switch_mode.pause` 不走0.25s busy 拒绝路径，会优先阻止运动、发送零速、取消 ACP，
并使此前恢复/模式切换的令牌失效，再发全身暂停请求。旧请求尚未发送时不会继续发送，
已发送请求的迟到结果也不能解除本地运动阻塞。停止确认仍受网络和厂商服务是否受理的限制。
本地取消/超时不等于 ROS 端取消：每个已发送的恢复请求还单独保留未决记录，
只有同一连接代次内收到明确拒绝，或受理成功并观测到本次请求后的有效 `enable=true` 终态，才可结清。
存在未决恢复时，停止仍立即发送零速、取消 ACP 并尝试 disable，但返回
`physical_stop_confirmed: false`，不会因一次 False 反馈或缓存 False 宣称 `paused`。
即使旧恢复在该次 disable 等待期间才结清，该次停止仍不确认；调用方须在结清后显式再次暂停，
使最终 disable 发送在旧恢复终态之后。不会自动重试控制服务或自动重新启用。

`switch_mode.get_current_mode.feedback.remote_resume_unresolved` 暴露该状态；未结清时拒绝新恢复/模式切换。
只保留至多一个未决恢复的响应句柄，超时/本地取消后仍能收到迟到响应；传输断开不是控制器拒绝，
重连和新会话 False 不会消除旧请求的不确定性。无法取得旧请求终态或发生部分发送/断线时，
需现场核实并处置控制器状态；重启驱动本身不证明远端请求已取消。
模式写入同样保留最多一个未决响应，`remote_mode_unresolved` 为true时不允许新动作恢复并覆盖旧写入。
现场版本63的模式服务在修改模式后返回终态result；保留到该结果可避免旧写入在新准备之后才生效。
模式服务终态并不代替运动前的新鲜模式话题确认。
所有执行卡声明 `on_interrupt_motion` hook。

位姿/关节运动订阅相应的 `/lb_*_reach_time`（`std_msgs/Float32`），只使用指令发送后收到的反馈。
反馈须在3s内到达，估计值接受0–110s；ACP fallback timeout 为125s。
双臂/下肢 `move_joints` 发令前要求运行时 `/humanoid_description` 通过关节映射/名称校验，
并有0.5s内实测关节位置、匹配的现场限位记录且目标在限位内；模型/限位缺失、不匹配或在途验证失效均拒绝/终止关节动作。
关节实测还要求 `sensor_time` 是合法、非零的 ROS 采集时间，机器人与驱动系统时钟必须同步；
接收时采集年龄最多0.5s，仅容忍最多0.05s的未来时钟误差。缺失、零值、过旧或明显未来的时间戳不用于关节测量，
不以回调时间或递增 header.seq 替代采集证据。相同/倒序采集时间不更新数据或新鲜度；重复投递最终过期。
指令后收到新 reach-time，并等过预计时长后，
还要求实测目标关节误差≤0.035rad，不同的指令后采集帧按原始纳秒时间跨度覆盖≥0.2s，
再返回 `measured_convergence: true`。回调重放、接收间隔及容忍的时钟误差均不能推进采集跨度。
若现场 `sensor_time` 使用启动相对时间或模拟时钟，当前驱动会拒绝关节控制；需先核实并适配真实时钟来源，不能关闭新鲜度检查绕过。
预计结束后5s内未收敛、实测丢失/异常会返回 ACP error 并尝试全身软件暂停。
底盘和躯干/双手笛卡尔动作的完成时间仍依据**官方到达时间估计**，明确标记 `measured_convergence: false`。
这些反馈没有命令 ID，因此需要保证本驱动独占所用命令话题及 `/enable_control`；
外部客户端同时发令可能导致反馈归属不明，也不在本驱动的控制顺序保证之内。

速度动作以20Hz发送，时长到达后归零，底盘额外等待文档规定的1s停止窗口，头部等待0.3s。
ACP 同样标记为时长/控制器超时推断。WebSocket发送成功仅代表写入连接，不代表 ROS 控制器确认执行。
发布器在连接时预先建立，并等待1s连接准备窗口；一次性指令仍以 reach-time 反馈检查接收情况。

首次运动发送抛异常也可能已部分或完整送达控制器。驱动在释放发令锁之前设置
`motion_blocked` 和 `motion_send_uncertain`，随后在锁外等待全身暂停，取消在途 ACP 并以 error 结束一次。
请求返回原 `action_id`、`command_delivery: uncertain` 和 `physical_stop_confirmed: false`，
不会因错误返回就遗忘可能已执行的目标。暂停仍发送零速和 disable，但即使收到 False 也不宣称停止已确认。
该不确定性在进程内锁存，重连不会清除，拒绝新运动、恢复和模式切换；退出时仍尝试暂停。
必须现场核实并处置控制器遗留目标后再重启恢复，重启驱动本身不能证明旧目标已取消。

执行卡系统 `start`/`stop` 是立即返回 `ready`/`idle` 的生命周期标记，不访问机器人或等待控制锁。
`info` 和 `switch_mode.get_current_mode` 另行报告实际连接状态；生命周期 `ready` 不表示已连接或允许运动。
已接受的动作仍由驱动运行时和 ACP 跟踪，不因生命周期标记而丢失；需要停止运动时必须调用
`stop_move`、`switch_mode.pause` 或触发执行卡的中断 hook。驱动退出时会清理在途动作。

`stop_move` 和中断操作会发底盘/头部零速度并请求 `/enable_control=false` **全身软件暂停**，
以避免位置目标在零速度命令退出后继续执行；它们会影响其他部位并取消所有在途 ACP。
停止后不重放任何旧目标；下一次显式提交的新运动动作会自动准备恢复和模式，无需单独点resume。
停止会使尚在参数检查、等待发令锁或准备控制的旧请求失效，旧请求不能随后重新恢复或取消更新的动作。
服务 `success=true` 只表示受理，暂停/恢复以 `/enable_control_state` 确认；保护窗口或重复请求被拒会返回错误。
暂停确认失败时返回 `physical_stop_confirmed: false`，并继续阻止本驱动发运动命令。

发布、取消和控制请求发送使用同一把短时锁；服务/反馈等待以及 ACP HTTP 回调不持有该锁。
断线、模式反馈超过2s未更新、使能验证失效、控制暂停或连接代次变化都会终止在途动作并尝试软件暂停。
**断网时软件指令不能保证停止真机**，物理安全仍需机器人原有控制器和现场急停保障。
回调失败有日志和有限重试，驱动退出有有限的回调排空时间。
ROS 消息处理异常按话题记录首次失败，持续异常每5s采样并累计失败数，恢复时记录一次恢复；
日志字段继续限长并转义，不会为每个异常帧打印一行。ACP HTTP 回调保持有限重试策略。
MCP 请求日志在所有 HTTP 连接间共享采样器，重复成功/失败请求分别每5s汇总抑制数；
实际卡片状态变化单独记录，`info`/模式查询轮询不产生状态变化日志。HTTP 200 中的 MCP 工具错误仍归为失败。
重连仅恢复发布器/订阅，清除旧反馈，绝不重放运动命令或重试超时服务。

### 使能状态的只读确认

固定版本官方控制器将 `/enable_control_state` 作为 latched Bool，在初始化及状态变化时发布，
不是周期心跳。驱动不因“正常状态长期不变”直接判为失效，也不会重复发送 enable=true 来刷新状态。
独立线程在每次检查结束约1s后再次检查：通过 `/rosapi/publishers` 确认恰好一个来源，
再使用同一 rosbridge subscription id 重订阅，请求重新投递 latched 状态，不取消原有订阅。
发布者查询和重新投递分别最多等待0.5s；只有实际收到检查开始后的状态投递才续期3s验证租约。
查询失败、来源缺失/冲突或未重新投递时立即使租约失效；没有新消息也不会自动给旧缓存续期。

运动及恢复/模式切换要求有效租约和租约期内接收的使能状态；过期的 True 不能作为已启用依据，
过期的 False 不能跳过暂停请求。恢复时验证未知则拒绝请求，等待只读检查，不擅自调用启用服务。
`switch_mode.get_current_mode.feedback.enable_verification` 提供有效性、年龄、来源及错误。
ROS 图查询和 latch 重新投递是状态通路检查，不是带生产者时间戳的硬件心跳，也不证明物理静止；
ROS master 缓存、具体 rosbridge 版本和真机控制器行为仍需现场核实。

## 传感输出与能力边界

已移除公开 `state` 卡；控制安全检查的内部反馈订阅仍保留。
诊断通过 `switch_mode.get_current_mode.feedback` 获取，包括连接、控制反馈、robot_discovery 和 host_runtime，
不再发布原来无助于传感展示的 control 流。

`joints/imu/loco_state` 按10Hz输出，battery/laser_scan按5Hz输出：

| 卡片 | 输出话题 | 内容 |
| --- | --- | --- |
| `joints` | `/<namespace>/state/joints` | 经运行时模型校验的20关节 `sensor/skeleton` 姿态；缺失实测时 invalid |
| `joints` | `/<namespace>/state/joint_state` | 原始 joint_q/joint_v/joint_vd/joint_torque、来源和采集时间；独立于骨架模型，供诊断，不用于绕过控制校验 |
| `imu` | `/<namespace>/state/imu` | 实测 wxyz quaternion、rpy、角速度和加速度；不猜安装 frame |
| `loco_state` | `/<namespace>/state/loco_state` | 扁平 x/y/z、roll/pitch/yaw、yaw_deg 和速度；优先新鲜 `/move_base/amcl_pose` map 定位，缺失时明确回退到 odom frame |
| `loco_state` | `/<namespace>/state/odom` | 独立 `motus.odom/1` 本体系 vx/vy/wz 窗口均值；未知轴 null，不将 map 位姿混入标准 odom |
| `battery` | `/<namespace>/state/battery` | 扁平 main_/chassis_/small_ 标量字段，单位写在键名中；保留不同状态码，不将0V或缺失数据当作有效测量 |
| `laser_scan` | `/<namespace>/state/laser_scan/points` | 首路 `sensor/pointcloud`：UInt8MultiArray 中 `<u32 step=16><u32 N><float32 x,y,z,intensity × N>`；z/intensity=0 |
| `laser_scan` | `/<namespace>/state/laser_scan` | 原始 angle/range 元数据及测距 JSON；无效测距 null |
| `camera_depth` | `/<namespace>/state/camera_depth` | CompressedImage，format=`image/depth-zlib`；zlib解压后640×480小端uint16毫米深度，0无效，保留采集header |

前端默认使用 topic_out 首路，因此雷达首路改为可视化点云，loco_state 首路改为位置/姿态展示。
旧画布保存的卡名/输出可能需刷新驱动或重新添加卡片，驱动兼容旧配置 camera_rgb/camera_rgb_right 的开启设置。

0.5s无测量或数据不合法会输出 `valid:false`；缺失力矩/速度不补零。
`model` 与 G1、天轶 2.0 一样读取驱动 `resource/` 下的 URDF，前端无需 STL 即可显示静态骨架，
不依赖机器人在线或控制器启动。模型来源、固定版本和许可证见 [resource/README.md](resource/README.md)。
该文件与现场版本63控制器使用的biped_s63一致，尚未独立核实实物尺寸及标定，不能用作控制模型。
实时更新仍要求实测 q；真机 `/humanoid_description` 缺失，或其20个受测关节名称与本地模型不一致时，
`joints` 输出 invalid、空关节数组和原因，静态 `model` 仍可读取。`switch_mode.get_current_mode.feedback` 中的
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
基础镜像已提供 `python3-pip`；Dockerfile 仅在缺失时安装 ROS2 FastDDS RMW、`std_msgs` 和 `sensor_msgs`，
用于选定的 DDS participant、`String` 状态和原生压缩图像发布。不安装本地 ROS2 `geometry_msgs`：
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
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"switch_mode","arguments":{"action":"get_current_mode"}}}'
```

真机仍需验证：

1. ROS1 上述类型/服务正确，rosbridge/rosapi 能加载厂商消息和参数，模式持续50Hz更新；确认使能 latched 重投递和发布者查询可用，验证发布者消失/探测超时后阻止运动。
2. 分别 start 传感卡，确认 domain42 收到 joints/joint_state/imu/loco_state/odom 及外设数据，字段、URDF名称、方向和 frame 正确，外部网络不接收该 DDS 数据。
3. 在现场有人监督且接口就绪时，分别以小幅度验证底盘、双臂、下肢、躯干和头部的方向与单位；检查每个动作的 ACP 成功/取消/错误。
4. 验证软件暂停、过渡窗口拒绝、恢复后需重新发目标、rosbridge 断线后不重放指令，以及多个执行卡共享通道的仲裁；在恢复/切换模式等待时触发停止，确认停止不因 busy 被丢弃，旧请求不能重新放开运动。
5. 检查关节 ACP 在实测到位后才成功；卡住、数据中断或误差超限不能成功。测量位姿 reach-time 估计的偏差，评估进一步闭环。

离线测试不能替代容器构建、DDS 互通和真机运动验证。


2026-11-16 最新现场验收：Agent Core内连续30秒收到深度300帧、左右相机各300帧、头部RGB299帧，全部格式有效。
头部深度640×480 uint16毫米，保留原始采集时间，采集至DDS回调最大257.5ms；未测试浏览器最终渲染或端到端时延。
IMU301帧、位置展示和标准odom各300帧且全部有效，电池/雷达点云各150帧，点云末帧3201点。
停止深度卡与relay后，基础检查不会启动闲置深度；重新start卡能自动恢复服务与有效数据。
全身joints/joint_state仍因无真实发布者明确invalid。199项离线测试、Python编译和部署片段检查通过。
详细检查记录见 ROBOT_INSPECTION.md 文末。
