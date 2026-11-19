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
驱动数据面通过 WebSocket/DDS 访问机器人；安装面通过SSH分发镜像自带程序。容器不申请 `privileged`，
不挂载Docker socket或宿主机根目录；仅挂只读DDS profile和独立可写配置目录，用于配置、SSH信任及私有凭据。
启动前检查该 profile 是文件；缺失、误挂成目录或 RMW 配置冲突时拒绝创建 ROS2 participant。

ROS2 / Agent Core 与驱动容器需要在同一 Linux 主机；机器人 ROS1 可以在本机或另一台可达主机。
容器内提供 ROS2 Humble，机器人宿主机无需额外安装 ROS2，也不需要构建 ROS2 版 `kuavo_msgs`。
ROS1 自定义消息的加载由机器人侧 rosbridge 完成。
MCP 仅监听 `127.0.0.1:15747`，只接受本机无 `Origin` 的服务端请求，并校验 `Host` 为该端口的
`localhost`/`127.0.0.1`；浏览器预检、任意浏览器来源及外部客户端均被拒绝，不返回通配 CORS。
页面通过 Agent Core 调用驱动，不直接访问运动端口。这是同机进程信任边界，不是本机进程之间的身份鉴权。

### 首次部署与自动准备

镜像包含全部自有配套程序和安装器（`bootstrap.py`、`host/bootstrap_remote.py`、`host/`）。
Dashboard/PR镜像部署片段把宿主机 `/opt/phanthy-motus/drivers/lejurobot-kuavo5-w/` 挂到 `/config`；
首次允许Docker创建目录，驱动仅在指定路径缺失时独占创建 `/config/config.yaml`（0600）。
已有操作配置、关节限位、禁用设置和凭据不被镜像默认值覆盖；不再要求部署前手工创建配置文件。
`CONFIG_PATH` 显式指向其他路径但文件缺失，或已有YAML损坏时仍报错，不悄悄回退。

整机连接默认值统一位于 `connection_defaults.py`：`DEFAULTS` 保存三台主机的地址、账号、ROS IP，
`DEFAULT_PASSWORDS` 保存对应默认SSH密码。`base_drive`及其他卡片不再提供连接参数configSchema或config动作；
不需要往画布放置额外配置卡。日常动作走ROS1/rosbridge，SSH仅供统一后台准备/修复三台主机。

按操作人员要求，镜像内置当前5-W/Jaten现场确认的默认值。正常部署无需填写账号密码；
默认值变化时修改上述模块，重新构建镜像并重建容器生效，重启旧镜像不会获得源码修改。
包内YAML不再重复地址/账号/ROS IP，避免遮蔽模块默认值。
这些值不作为所有批次、所有官方文档版本的通用凭据声明。

兼容已有部署的显式覆盖仍保留：环境变量、操作配置中的角色字段及0600私有凭据文件优先于模块默认值。
因此修改模块后，若旧部署曾配置相同字段，需同时移除或更新对应覆盖；驱动不会擅自覆盖已有操作配置。
也可通过 `KUAVO_HOST/CONTROLLER/CHASSIS_SSH_PASSWORD`、相应 `SUDO_PASSWORD` 环境变量，
或宿主机配置目录内0600权限的 `ssh-secrets.json` 提供连接信息：

```json
{
  "host": {"password": "<上位机SSH密码>"},
  "controller": {"password": "<控制下位机SSH密码>"},
  "chassis": {"password": "<独立底盘SSH密码>", "sudo_password": "<若与SSH不同>"}
}
```

支持在各角色私有配置中设置 `key_file`（容器内私钥路径），或使用相应 `KUAVO_<ROLE>_SSH_KEY`。
显式配置私钥时优先使用非交互公钥认证，不因内置密码阻止私钥使用。
每台主机可设 `use_default_credentials: false` 禁用其内置密码回退，改用公钥/显式凭据；只尝试选定的一套凭据，不轮询猜密码。
空环境默认值不遮蔽私有文件，空格密码不被trim。OpenSSH使用accept-new首次信任并持久化known_hosts；
主机密钥变化报错，不用StrictHostKeyChecking=no绕过。无法登录时MCP仍可提供分主机诊断错误，
不会因此伪造传感数据或放开运动。

默认ROS网络：上位机容器通过127.0.0.1登录本机，ROS_IP=192.168.26.12；
控制下位机192.168.26.1/lab；独立底盘192.168.26.22/jt；ROS master为192.168.26.1:11311。
`provisioning` 可覆盖master、每台机器的address/user/ros_ip/workspace及上位机head_workspace；
地址/账号也可用 `KUAVO_<ROLE>_ADDRESS/USER/ROS_IP/WORKSPACE` 环境变量覆盖。
自动发现对应用户及/home下的厂商应用/控制工作区，匹配不唯一就报错，不猜测另一份工作区。

### 部署时提供凭据（无需卡片配置）

以下是现有的部署级覆盖方式，整机只配置一次。它们不会自动删除镜像内的默认密码；
若要完全采纳凭据审查建议，还需另行去掉内置密码回退并更换已公开的真实密码。

**方式一：宿主机私有文件（现有部署可直接使用）。** 部署片段已将
`/opt/phanthy-motus/drivers/lejurobot-kuavo5-w/` 映射到容器 `/config`，无需新增挂载。
在机器人上位机执行下列命令；touch不覆盖已有文件，编辑时保留其他角色的已有内容：

```sh
sudo install -d -m 0700 /opt/phanthy-motus/drivers/lejurobot-kuavo5-w
sudo touch /opt/phanthy-motus/drivers/lejurobot-kuavo5-w/ssh-secrets.json
sudo chmod 0600 /opt/phanthy-motus/drivers/lejurobot-kuavo5-w/ssh-secrets.json
sudoedit /opt/phanthy-motus/drivers/lejurobot-kuavo5-w/ssh-secrets.json
```

填写上文host/controller/chassis的JSON，将占位内容替换为真实SSH密码；sudo密码相同时不用另填。
文件不提交Git、不复制入镜像。后台每轮准备时重新读取，通常下一轮约30秒内使用，无需重建镜像。
只要显式覆盖了所有需要密码认证的角色，就不会为这些角色使用内置密码。

**方式二：环境变量。** 现有Compose片段已转发三台主机的SSH及sudo密码变量。
例如用交互输入给出上位机密码，避免在命令字面量中写密码；其他两角色同理：

```sh
read -rs KUAVO_HOST_SSH_PASSWORD; echo
export KUAVO_HOST_SSH_PASSWORD
# 使用实际部署的Compose文件和project参数：
docker compose -p your-project -f /path/to/docker-compose.yml up -d --force-recreate
unset KUAVO_HOST_SSH_PASSWORD
```

另两角色为`KUAVO_CONTROLLER_SSH_PASSWORD`和`KUAVO_CHASSIS_SSH_PASSWORD`；
sudo密码不同时分别设置相应`KUAVO_<ROLE>_SUDO_PASSWORD`。
必须在运行Compose的同一环境设置，宿主机export不会改变已运行容器，单纯docker restart也不注入新值。
非空环境密码优先于秘密文件；环境变量可被有容器管理权限的人查看，因此优先考虑私有文件或密钥。

**方式三：SSH密钥。** 为部署生成密钥，将公钥分别加入三台主机对应账号的authorized_keys；
私钥留在上位机专用目录（例如宿主机配置目录的ssh/id_ed25519，权限0600），不要提交或复制进镜像。
现有目录挂载后，秘密文件中的三个角色可分别写`{"key_file":"/config/ssh/id_ed25519"}`，
也可以使用各自独立密钥。注意这里填写的是容器内路径，并移除同角色旧password字段及非空SSH密码环境变量，
避免显式密码继续选择密码认证。密钥存在时不会回退内置密码；sudo需配置目标机非交互权限，
否则在该角色的秘密文件记录中单独提供sudo_password。

| 目标 | 容器自动完成 |
| --- | --- |
| 控制下位机 | 查找版本63轮臂工作区和Drake，取得该控制器生成的kuavo_msgs；分发开机监管、安装systemd单元，首次本次开机未尝试过时启动，完成初始化→BaseArm确认→软件暂停 |
| 上位机 | 分发并安装loopback rosbridge、rosapi消息覆盖层、电池转发、固定操作管理器、头部RGB/深度launch及转发器；确保基础服务启动，头部卡缺帧时按需启动相机服务 |
| 独立底盘 | 分发PCM播放服务、准备用户PulseAudio；确保厂商jaten.service运行，以提供左右相机、二维雷达、IMU、里程计等；扬声器首块PCM才创建播放器 |

所有自有源码随镜像构建复制，不依赖助手留在/tmp下的文件。分发使用固定文件白名单和固定安装目的地，
不接受画布传入shell命令；以文件/配置哈希、systemd单元及消息覆盖层检查进行幂等校验。
普通进程探测或端口存在不能替代部署校验。每轮约30秒检查，缺少/损坏的自有文件或单元会修复，
停止的非电机服务会启动，健康服务不反复restart。运行状态见 `base_drive.info.feedback.host_runtime.provisioning.targets`；
controller/host/chassis分别报告checking/ready/blocked及错误，不传输私有凭据到状态中。

容器默认不需要源码仓库、额外pip脚本或手工SSH安装。必要前提仍是机器人已有**厂商ROS Noetic、控制程序、
匹配的硬件/标定、Orbbec驱动工作区及底盘jaten程序**，并允许SSH/sudo管理。
缺少通用rosbridge、Python图像依赖或PulseAudio运行包时，自动从目标机配置的软件源安装固定白名单包；
需要可用软件源，也可设 `provisioning.install_missing_packages: false` 只报告缺失。
不会从网络下载并猜测安装专有控制固件、Drake、标定或整套厂商SDK；不支持的控制版本明确阻止运动。
缺少真实麦克风、未核验的关节限位或不兼容的导航协议，也不会靠自动部署变成“全部可用”。

### 控制器自动启动的边界

随包默认 `host_runtime.controller_boot_required: true`、`provisioning.auto_initialize_and_pause: true`。
首次自动部署在确认本次开机未启动过监管服务后请求一次启动；容器重启只观察已运行程序，
**不重启电机、不重复初始化**。自有控制进程故障/退出、被人工停止，或记录表明本次开机已尝试过时，
不自动再次拉起；需要现场排障。源码升级可安装到磁盘供下次开机使用，不重启当前控制器。
关闭auto_initialize_and_pause会禁用该服务未来自动开机启动，保留当前运行进程；不把配置切换当成急停。

监管服务等待厂商H12服务/ROS master，检查真实控制节点和原始关节发布者，忽略joy、bridge及自身订阅者。
启动前检查 `/opt/drake/lib/libdrake.so`，显式传入库路径。固定版本63/20关节及厂商服务来源通过后，
只请求一次初始化，等待其后新鲜BaseArm(mode=3)反馈，再请求enable=false并确认新false。
超时不重发、不假报complete。若已有外部控制器，仅在其已初始化且已暂停、型号/测量匹配时观察接管，
不抢占另一控制者。`boot_pause_confirmed`是开机历史结果，当前使能仍以独立新鲜反馈为准。
开机状态超过3秒未更新会阻止运动。新运动动作在参数/限位通过后自动恢复，不恢复旧目标；
相机、音频或基础遥测的start/info不会触发enable=true。

### 数据桥接、相机和人工排障

上下位机kuavo_msgs可能同名但MD5不同；自动部署会从实际控制器导出生成的Python msg/srv，
仅供bridge/rosapi进程优先加载，不覆盖厂商工作区。消息映射变化时重载bridge，不重启控制器。
本地固定助手每5秒检查基础bridge/电池服务；头部RGB/深度卡缺帧时分别请求固定的head/depth操作，
不接受任意unit或命令。wrapper将旧版rosbridge绑定到loopback，设置TCP_NODELAY/2MB缓冲；不改系统ROS包。

默认自动输出joints/imu/loco_state/battery/laser_scan及四路图像；运行中的相机卡缺帧约每5秒重试，
未运行的头部卡不启动相机。speaker.start同时唤醒底盘服务检查。首次安装尚未完成时卡可能返回starting/error，
通过分主机状态排除凭据/厂商运行时问题后可重新start；安装不是运动动作，不伪造ACP完成。
如显式禁用host_runtime或provisioning，就由外部自行管理，不再承诺自动准备。

`host/install.py`、`host/install_controller.py`、`host/install_speaker.py` 仍是容器实际调用的安装器，
可在排障时手工执行，但不再是正常部署前置步骤。升级会保留现场配置及禁用开关。
完整断电后的时序仍需在另一台同款出厂机器复验；单台既有环境的成功不能代替该验收。

### 构建依赖

FastDDS/std_msgs/sensor_msgs以及新增openssh-client/sshpass仅在基础镜像缺失时安装；后两者用于自动跨机部署，
无需SSH服务端或容器特权。密码通过环境/标准输入传递，不放进SSH argv。PyYAML/websocket-client保持requirements声明，
复用已有PyYAML及 `/ros_ws/install/audio_msgs`；没有消息生成工具链或新增厂商SDK。
新增SSH客户端和源码有镜像体积成本，不能称零增长。基础镜像若使用只能内网解析的腾讯apt源，
构建可显式传 `--build-arg UBUNTU_APT_MIRROR=https://ports.ubuntu.com/ubuntu-ports`；默认不改已有软件源。
2026-11-18已在真机用本Dockerfile构建候选，基于既有release由736907954增至743100785字节
（+6192831字节，约6.19MB，含新增SSH客户端和源码）。此为相对既有驱动release的增量，
不是生产ros-base全新构建的总增量；完整基线对比仍需发布构建核实。

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
因此暂不注册 `mic`。扬声器已在独立底盘电脑的ALC897模拟输出完成现场听音确认，见下文 `speaker`。
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
普通执行动作会按需恢复控制并确认兼容MPC模式，已移除对外的 `switch_mode` 卡。
`motion_enabled` 默认 `true`，未配置该字段时也默认开启；双臂/下肢关节动作还必须配置现场核验的限位，见下节。
该开关仅允许处理运动请求，后台检查、读取传感器、卡片start/info不会触发恢复或发运动目标；原有运行时检查仍生效。
如需只读运行，可在实际加载的配置文件中设置 `motion_enabled: false` 并重启驱动；此时拒绝运动、
恢复控制及模式切换，仍允许状态/传感器/模型读取、软件暂停和 `stop_move`。该字段必须是 YAML 布尔值。
`base_drive.info.feedback.motion_enabled` 与执行卡 `info.motion_enabled` 可确认当前配置。
机器人必须处于允许运动的状态，且收到本次 WebSocket 连接内的控制状态和新鲜模式反馈后才接受运动指令。
启动和重连后，独立线程经 rosapi 探测机型、robot_version、自由度、SHM 参数及 `/humanoid_description`。
必须确认 wheel-arm=1、下肢4、双臂14、头2后才能运动；探测失败时可通过 `base_drive.info.feedback.robot_discovery` 查原因。
参数缺失不回退到 SDK 的双足默认配置。后台探测在每轮结束约1s后重新读取完整身份参数和 URDF；
成功读取的身份有效期为3s，从该轮读取开始计时，慢查询不会在完成时伪造新租约。
查询失败、身份过期或模型/限位绑定变化会拒绝/终止相关动作，不依赖 WebSocket 重连；
相同来源和连接代次不重复订阅原始传感器。状态在 `robot_discovery.identity_verification` 查看。
轮询不是参数变更通知或原子快照，仍存在检测窗口；必须在机器人停止且无在途动作时修改控制器参数，
不支持边运动边更换模型或标定。现场还需确认 rosapi 查询时延满足3s有效期。

## 卡片与官方接口

| 卡片 | 动作 | 官方 ROS1 接口 |
| --- | --- | --- |
| `base_drive` | `move`、`move_relative`、`stop_move` | `/cmd_vel`；相对位姿动作使用新鲜 `/odom` 闭环 |
| `base_servo` | `pause`、`resume`；生命周期 `start/stop/info` | ActuCore navi 的 `control/velocity` → `/cmd_vel` |
| `arm` | `set`、`zero`、`stop_move` | `/kuavo_arm_traj` |
| `leg` | `set`、`zero`、`move_joints`、`stop_move` | `/lb_leg_traj` |
| `head` | `move_pos`、`zero`、`stop_move` | `/robot_head_motion_data`；停止残余速度用`/cmd_head_vel` |
| `controlled_spatial` | 建图、保存、打点、载图、按坐标/语义点导航、`stop_nav` | 原生 Leju 导航服务 + JaTen 地图 API |
| `speaker` | `start`、`stop`、`info`、`set_volume`、`get_volume` | ROS2 `AudioChunk` → 独立rosbridge → 自有底盘PCM播放服务 |
| `joints` | 系统 `start`、`stop`、`info` | 两路：`sensor/skeleton` 骨架姿态 + `data/json` 原始 joint_state |
| `imu` | 系统 `start`、`stop`、`info` | 同一原始消息的 IMU；`data/json` |
| `loco_state` | 系统 `start`、`stop`、`info` | 两路：map/odom 位置、角度、速度 JSON + 独立 `state/odom` / `motus.odom/1` |
| `model` | resource 读取 | 本地 `resource/kuavo5w_model.urdf` 参考骨架；离线可读 |

`base_servo` 是轮式底盘连续伺服卡。在画布将 ActuCore navi 的 `control/velocity` 输出连接到它；Agent Core 在 start 传入 `input_topic`，并从返回值读取 `control_interface` 描述符。相机/深度连接对应传感器，里程计连接 `loco_state` 的 odom 流。

速度协议与 R1 一致：ROS2 `std_msgs/String` JSON，`motus.control/1`、`mode=twist`、`dof=6`，机身系 `[vx,vy,vz,wx,wy,wz]`，单位 m/s、rad/s，vz/wx/wy 必须为零。复用主节点 domain 42 的隔离 DDS，工作线程经 rosbridge 转发 ROS1 `/cmd_vel`。期望10Hz、最大20Hz；校验序号、TTL和观测时间，300ms断流发零速、拒绝500ms以上旧观测；慢速使能完成后再次检查首帧是否过期，不执行积压旧目标。ROS2回调仅保留最新消息，不等待SSH或控制服务。

`pause`及全局中断锁存暂停，需`resume`才接收后续帧；普通`base_drive.move/move_relative`优先暂停伺服。与厂商导航/建图共用内部base通道互斥。速度流不创建ACP pending，视觉导航到达由ActuCore navi负责ACP。容器启动不会自动订阅速度流。`info`返回输入话题、描述符、底盘准备状态、最近校验结果和执行次数。厂商控制器的速度超时机制保留。

速度依据为2026-11-19真机只读核对：正在使用的 `humanoid_wheel_interface/config/kuavo_s63/task.info` 中 `wheel_move.max_vel=[0.8,0.8,1.6]`；底盘 `leju_node.launch` 及运行时 `/leju_node/max_linear_vel`、`max_angular_vel` 均为1.0。驱动取交集：平移合速度0.8m/s、旋转1rad/s。这是厂商软件命令包络，不宣称硬件极限；其他固件仍可能进一步限幅。官方《轮臂控制API》给出了单位和停止语义，没有统一的硬件最高速度数字。`controlled_spatial`仍遵守厂商路径规划器自己的速度限制。

新增模块复用既有 `common/control`、std_msgs及rclpy，无新增pip/apt依赖。旧的持久化配置需加入`plugins.base_servo: {enabled: true}`，显式禁用配置仍受尊重。404项离线测试覆盖速度流、断流、旧帧、全局暂停、动作抢占、重连及使能期间暂停的晚响应竞态。新镜像已验证domain 42下的真实DDS收发和断流零速，ROS1端使用内存替身，不连接电机；尚未以新档位执行真机运动或验证整条视觉导航到达。

sensor/actuator 卡片实现系统生命周期动作；`model` 为读取型 resource。
`model.start/stop/info`仅返回独立生命周期状态，不加入传感器发布循环；
显式`read`或不带action的模型请求返回普通`{"urdf": "..."}`字典，保持前端骨架读取兼容。
业务参数通过 `x-action-params` 分组并声明动作各自的 required；`dispatch()` 返回普通 dict，HTTP 层只包装一次 MCP content。
`config.yaml` 的 `plugins.<card>.enabled` 决定卡片是否暴露。

初始化流程等待实际BaseArm反馈（mode=3），再进入软件暂停；新的运动动作自动恢复控制。
不提供单独的模式切换/恢复卡，底盘、手臂等执行卡的 `stop_move` 和中断 hook 仍会全身软件暂停。
底盘执行卡已从 `loco` 更名为 `base_drive`，动作、单位、ACP及`base`资源锁保持一致。
旧部署配置的`plugins.loco`自动迁移为新名称；若同时指定`plugins.base_drive`，以新键为准。
旧画布上的loco卡需替换为base_drive。遥测卡`loco_state`及其已有双流主题不随执行卡改名。

诊断通过任一执行卡的系统 `info.feedback` 读取，例如 `base_drive.info`；旧画布中的switch_mode卡应删除。
`controller_not_ready` 表示控制器前提缺失，不能通过填入默认参数绕过。
`stop_move` 不依赖机型探测，仍会尝试零速、取消在途动作及请求暂停；若控制服务/反馈缺失，
返回 `physical_stop_confirmed: false`，不能假报暂停成功。查询返回的 `state: ready` 仅表示查询成功。

左右/头部 RGB、头部深度、电池、二维雷达点云、扬声器已适配；未发现可用麦克风，暂不提供 `mic`。

### controlled_spatial：建图、打点和导航

动作名称与天轶2.0保持一致，后端使用本机实际安装的 Leju/JaTen 协议，不能调用 Slamtec 接口：

| 动作 | 参数 | 行为 |
| --- | --- | --- |
| `start_mapping` | `map_name` | 按厂商流程启动 Cartographer，随后恢复上位遥控器所需的 AUTO 速度通路；不覆盖已有同名地图 |
| `stop_mapping` | 无 | 保存地图/路网、重启厂商地图服务，载入并确认地图文件身份 |
| `tag_place` | `name`，可选 `description` | 保存当前 map 系 x/y/yaw；建图期间先绑定当前建图会话 |
| `untag_place` / `list_tags` | `name` / 无 | 删除/列出当前地图语义点，列表含相对距离和方向 |
| `list_maps` / `load_map` | 无 / `map_name` | 列出真实 PGM 地图 / 载入并切回自动模式 |
| `navigate_to_pose` | `x`、`y`（米）、`yaw`（弧度），可选 `speed` | 按 map 系目标导航 |
| `navigate_to_tag` | `tag_name`，可选 `speed` | 只精确匹配当前地图中的语义点 |
| `stop_nav` | 无 | 全身暂停，同时取消原生导航并核实新鲜里程计停稳 |

`base_drive.start` 与 `controlled_spatial.start` 共用底盘启动准备：检查硬件急停/碰撞/故障，
调用厂商 CANCEL_CURRENT_TASK(2) 取消旧任务，确认新鲜里程计连续停稳，再在软急停有效时
调用 CANCEL_SOFT_STOP(1)，最后核实原始反馈、界面状态与停稳；首次立即返回 `state: loading` 和英文消息 `Checking chassis status in background, please wait. Control will be ready automatically.`；`info` 可查询 loading/running/error。两张卡共享同一后台检查，同一次连接且未停止时不重复取消任务。停止或重连后重新检查。
这一步不发送移动目标，不恢复旧任务，不使能全身控制；后续显式运动仍按既有流程恢复控制。
启动只要求新鲜的`agvState/state`故障与行驶状态，不依赖地图定位帧；实际导航仍必须收齐`pose2d/agvState/state/localizationStatus`并通过定位校验。每次快照使用新连接，所需帧的接收年龄须≤2s。启动快照超时或断连时最多读取3次，期间保持loading并显示缺失字段；仅重试读取，不重复取消任务或解除请求。持续失败返回`fresh_navigation_state_unavailable: missing=...; reason=...`，方便区分缺帧与连接故障。中断或关闭会阻止后续准备。
两张卡的 `info` 只读，显示地图、定位、急停、在途任务和建图阶段。
`motion_enabled: false`、硬件故障或已存在的在途/未确认动作会阻止准备，不会强行清除。
停止可抢占准备：较早解除请求返回后，再设置软急停(0)、取消任务并核实反馈；
若解除请求因网络/ROS传输失败仍可能在执行，则保留持久化未确认状态、禁止运动，也不假报已停止。
明确返回的拒绝或反馈异常在重新确认软急停后可重试 start。
把新卡片加入画布后使用。已有 `/config/config.yaml` 不会被镜像覆盖，旧部署需在
`plugins` 下增加 `controlled_spatial: {enabled: true}`；新部署的镜像默认已开启。
连接参数共用 `provisioning.chassis`，不需要每张卡重复填写 SSH 信息。
SSH RPC 辅助源码随镜像构建，每次在底盘执行固定操作，使用当地已安装的 ROS 消息类；
不需要手工安装新服务，不额外开端口或复制一个可能不兼容的消息工作区。

导航的提交、查询和取消均核验为同一服务提供者。ACP 使用厂商返回的 task_id 跟踪，
到达还需核对地图、位姿和零速反馈；不是“服务返回 success 就完成”。
速度默认0.15m/s、范围0.05–0.3m/s；启用避障，避障距离0.5m，不暴露禁用避障开关。
`base_drive` 和新卡共用内部 base 动作锁及 Agent Core 的 base 资源声明，中断走 `stop_nav`。地图流程每次ChangeMode均等待实际模式反馈再发下一条命令，不把接口应答当作模式已切换；载图若收到厂商8211模式前置条件拒绝，保留错误ACP但不遗留“远端仍在执行”的锁，允许重试。超时/断连仍保留未确认状态。
停止会先请求全身禁用，不因旧导航提交阻塞这一步，再按顺序原生取消并核实停稳。

**载图不等于定位成功**。载图/保存不会伪造 positionInitialized、解除底盘软件急停、硬件急停、
碰撞或故障。定位未就绪或软件急停仍有效时，导航明确拒绝；软件急停可由卡片 start 按上述顺序准备，定位仍需按厂商操作恢复。
`base_drive`和`controlled_spatial`可以同时start；start共用准备结果，不因卡片都开启就占用运动锁。只有正在执行的动作互斥。建图前停止自动位姿/速度动作并暂停`base_servo`输入，松开遥控器；停稳检查使用连续新鲜原始`/odom`，不以网页残留的非零velocity误判。`start_mapping`在厂商MANUAL模式启动建图后切回AUTO，并同时确认建图仍有效和AUTO模式反馈；全身板G12遥控器经MPC使用底盘的自动速度源，若一直停留在MANUAL，输入会被底盘仲裁屏蔽。AUTO在这里表示速度输入通路，不代表自动探索，驱动不发送探索目标。全身软件暂停或底盘软急停仍会阻止遥控运动；切换底盘模式不会解除这两层暂停。驱动不关闭厂商遥控节点，但自动命令与遥控命令会共享厂商底盘控制链路，不能保证同时操作的结果；不要在自动动作期间拨动摇杆。`stop_move`是全身软件暂停，也会影响经过全身控制器的遥控输入，并非只取消MCP请求。
建图、保存、载图和导航都返回 action_id 并通过 ACP 报终态。保存地图含厂商最终轨迹优化，可能耗时数分钟；保存响应最多等待25分钟，整个保存/载图流程最多30分钟，其余地图操作与导航最多150秒。HTTP调用立即返回executing，不等待优化结束；`info`在此期间显示saving及英文等待说明。保存期间缺少地图TF/定位帧不代表优化已失败，不要重启建图或重复提交保存。

语义点和未确认操作保存在已有持久卷 `/config/controlled_spatial.db`，
绑定地图名、**PGM+YAML 的 SHA256**和路网 MD5；同名地图内容/坐标配置变动后，旧点不会继续导航。
保存建图成功后才把会话内的点绑定到最终地图身份。
丢失提交/地图操作确认时不自动重试，也不因容器重启忘掉在途状态；`info` 可查看 pending。
已知任务可再次 `stop_nav` 请求取消；提交去向、解除请求或地图变更仍不明时保持阻断，
需要现场核对厂商状态后恢复，不提供“忽略错误继续运动”的复位入口。若`stop_mapping`已在SaveCartoMap阶段超时，可再次调用`stop_mapping`接续收尾：只读等待原保存生成完整文件（含在同步保存完成后写入的有效`.path`，且所有文件连续2秒不变），不会再次提交SaveCartoMap；随后载图并绑定原会话的语义点。保存未确认前`load_map`仍会拒绝，不能清除记录强行载图。地图转换与原生导航的阻断原因分别提示；被阻断的leg等运动请求不会发出目标或主动暂停机器人。`stop_nav`仍请求全身暂停，但不会把未确认的地图操作当作已完成。
运行期间不要用另一客户端切换地图/改变坐标原点：周期核对不是原子的跨系统锁。

离线测试覆盖任务ID、取消顺序、重启保留、地图指纹、语义点、地图转换和两卡互斥。
已在真机只读确认服务同源、地图列表/PGM/YAML指纹及新鲜状态；
**实际建图保存、载图、导航与途中取消尚未现场运动验收**，不能以单元测试替代。

厂商编号音效不是任意PCM接口；本驱动使用已实际听到声音的底盘标准音频输出。

### 扬声器安装与使用

正常部署由容器自动安装到底盘。以下仅为人工排障时的等价命令，在**独立底盘电脑**执行：

```sh
sudo python3 host/install_speaker.py --user jt --master http://192.168.26.1:11311 --ros-ip 192.168.26.22 --volume 30
```

仅使用该机已有ROS Noetic、`paplay/pactl`和PulseAudio；不安装SDK、声卡库或TTS引擎，不在播放器中保存SSH凭据；默认连接值集中于驱动，用户覆盖密码另存私有配置。
安装 `kuavo5w-speaker.service` 并启用音频用户linger，允许用户音频服务开机运行；
自动启用该用户的 `pulseaudio.socket`，无需先登录桌面。轻量ROS服务随系统启动，但无音频数据时不启动播放器。
服务只访问音频，无控制器/电机操作；`speaker` 不依赖 `motion_enabled` 或恢复电机。

画布将TTS/音频节点的 `audio/pcm-16k` 输出连接到 `speaker`，Core在 `start` 传入 `input_topic`。
输入为标准 `audio_msgs/msg/AudioChunk`，16kHz、单声道、小端signed int16；接受Core的
`pcm_16k_16bit_mono`格式标识和TTS的 `audio/pcm-16k`，EOF魔数只用于结束标识，不会当声音播放。
复用基础镜像 `/ros_ws/install` 已有消息定义，Docker启动时加载并在构建时检查导入，不增加消息构建依赖。
`start` 检查服务及已核实的模拟sink，分配新会话；服务缺失或声卡不可用时返回错误，不退回假输出。

每张扬声器卡有独立WebSocket和播放线程，ROS2回调只校验/入队；最多100个100ms块，
按实时速率转发，底盘队列上限1秒、非阻塞写入播放器。过载/进程异常停止，不无限堆积。
会话token和递增序号拒绝旧会话/重复包；独立1.5秒墙钟租约在输入链路丢失后停止本服务的播放器，
重连不会重放旧流。`stop` 销毁订阅、清空缓存并确认底盘停止；无法确认则明确报错。
不提供独立interrupt动作或interrupt hook；打断由前方TTS/PCM生成器处理，speaker只消费上游送达的流。
EOF只作结束标记跳过，后续PCM继续播放；如需清空已排队PCM，显式stop后重新start。保留mouth资源。
与其他驱动的流式speaker一致，这张卡以start/stop维持数据平面，不给每个PCM块伪造ACP动作；
运动卡原有ACP不变。本卡不提供逐句声学播放完成ACP。

`set_volume(volume=0..100)` 设置本PCM播放流的数字增益；0静音、100原始幅度。
百分比使用原PulseAudio的立方增益曲线，避免同为30%时升级后突然变响。
底盘确认request_id后才返回成功，`get_volume`实时读取底盘值，可在未start时使用，也可播放中更改。
音量变化不重启播放进程/会话；`--volume`只设服务启动默认值30%。`info.output_status.volume_percent`显示底盘值；
不修改系统主音量、不停止厂商导航提示音。`output_status.bytes_written`是写入播放器的字节数，
不是麦克风回录证明。2026-11-18现场已通过完整ROS2→rosbridge→底盘链路播放三遍“待命中”，
124416字节全部写入，操作人员确认听到，stop确认idle；完整断电后音频自启动仍待复验。
此前静音测试的interrupt属于旧接口历史结果；当前版本已移除该动作，stop和断连租约保护保持。
接口快照、可行性及导航取消/ACP 适配前提见 `ROBOT_INSPECTION.md` 的能力核查记录。

参数单位与约束：

- 底盘速度：本体系 `vx/vy` 单位m/s，平移合速度≤0.8；`vyaw` 单位**度/秒**，范围±57.2957795131。`duration` 必填，0.1–10s，或 `-1` 持续运动至 `stop_move`。连续动作返回running/continuous，不注册永远无法完成的ACP pending；保留本地通道占用、心跳与安全检查，停止/断连/反馈失效时终止。有限时长与位姿动作仍保持正常ACP等待及资源屏障。
- 底盘位姿：`x/y` 为米，`yaw` 为**度**，三个值均必填。`move_relative` 的x/y是动作实际发送时机身系相对位移，yaw是相对转角。已移除`move_to`：odom原点由厂商里程计初始化/重置决定，本驱动没有可复现的绝对零点管理；地图绝对位置使用`controlled_spatial.navigate_to_pose`。位置反馈来自新鲜带时间戳的原始`/odom`，按20Hz发送有界`/cmd_vel`。末段使用有限幅、抗饱和积分补偿，只有新测量才累积；2cm/1°内连续0.3s新测量属于精确完成。另允许近似完成：残差≤10cm/5°且连续1s新测量漂移≤2mm/0.2°时，先发零速，再用至少1.1s新测量确认稳定（最多等4s），才发送成功ACP。近似完成明确返回`approximate: true`、`measured_convergence: false`以及目标、实测、剩余距离/各轴误差和英文说明；精确完成返回`approximate: false`。不以缓存的重复帧、旧反馈、明显未停稳或范围外误差宣称成功。各轴组在进入容差后发零速并清除积分，不提高所选速度档位。`info.motion`显示目标、实测、误差及速度；8s无实质进展、缺少反馈、坐标系改变或110s总超时会暂停并报告误差。`expected_duration_s`是按距离/转角与速度档估计的时间，`timeout_s`才是总超时。
  `speed` 可选 `low`（默认：0.24m/s、17.1887°/s）、`medium`（0.48m/s、34.3775°/s）、`high`（0.8m/s、57.2958°/s），按厂商控制链命令上限的30%/60%/100%设置，接近目标时减速。位移上限5m；实测位置误差≤2cm、角度误差≤1°且不同样本持续0.3s为精确完成；上述稳定小残差分支为近似完成，附实测误差。没有避障规划，避障导航使用`controlled_spatial`。
- 双臂 `set` 可填写`left_joint_1_rad`…`left_joint_7_rad`、`right_joint_1_rad`…`right_joint_7_rad`中的任意关节，单位**rad**。左右臂1、4、7号关节相对于厂商原始数据反向；其余保持厂商方向。只转换明确填写的目标，未填关节保留实际位置，原始joint_state及URDF不反向。
  省略、null或空白表示保持当前实测位置；0是有效目标；全部为空拒绝。实际发送前重新取未指定关节的实测值。`zero`同时归零全部14个手臂关节。字段description包含实时限位（rad/度），方向反转后的上下界也同步转换；未发现限位时不声明虚构范围且拒绝执行。旧完整`move_joints`度数组保持原始厂商方向，未在UI展示。
- 下肢 `leg` 包含膝、腿、腰俯仰与偏航，删除重复的`waist`卡。`set`接受`knee_deg`、`leg_deg`、`waist_pitch_deg`、`waist_yaw_deg`的一个或多个绝对**度**值；省略时保持实测值。`zero`归零全部4关节。兼容`move_joints(positions)`四个度值。
  `set_height(height)`接受`waist_yaw_link`原点在`base_link`坐标系中的绝对Z坐标（m），不是头顶离地高度。厂商轮臂API的`/cmd_lb_torso_pose.linear.z`及控制器`task.info`中的`baseFrame/torsoFrame`定义了这一参考系。此前0.74–1.06只是驱动固定筛选值，没有足够依据作为该机器人的实际工作范围，现已移除。先调用`get_height`获取实测`height_m`与当前姿态下的`reachable_height_intervals_m`：由实时URDF和关节限位求解析边界，保持躯干x/俯仰及腰偏航；这些是运动学可达区间，并非碰撞/负载安全认证。不可达请求返回当前值和区间，不静默夹紧。通过厂商`/cmd_lb_torso_pose`联动膝、腿、腰，ACP仍用真实关节收敛确认；不支持的几何或缺失反馈在恢复控制前拒绝。
- 头部 `move_pos(yaw,pitch)` 为绝对**度**，yaw范围[-30,30]、pitch[-25,25]；**Pitch正值向上、负值向下**，只转换卡片输入，原始反馈不改。至少填一轴；省略轴在实际发令前取实测角度。`zero`两个轴归零。
  实机厂商位置环存在数度偏差，目前没有经过验证的反馈补偿参数。`move_pos/zero`返回`executing/action_id`，通过真实`joint_state`对应的关节反馈判断完成：各轴≤1°并连续稳定0.3s为精确完成；各轴残差≤5°且连续1s漂移≤0.2°可近似完成。ACP返回`approximate`、`target`、`measured`、`error`（yaw/pitch，度，均使用卡片方向）和说明，不再仅以指令发送成功回执。必须使用新鲜、不同且指令后采集的帧；旧帧、断流、失去控制或8s内无法稳定会报错并暂停。中断只发一次cancelled；没有修改增益、零点或伪造joint_state。

`controlled_spatial.motion_music_enabled: false`（默认）在容器启动后独立后台执行，无需启动底盘卡；连接暂不可用时每30秒重试，不触发取消任务、解除急停或电机使能。仅把已核实的厂商ID-1运动曲目`1_flower.mp3`替换为有效静音MP3；原件与SHA256记录保存在同目录`.motus-motion-music/`，设为true可恢复。保留ID-2告警、充电、障碍提示和TTS，不改变系统音量、不重启底盘管理节点。使用底盘已有ffmpeg，不自动安装新包；未知文件布局、缺少ffmpeg或操作员已改动曲目时返回unsupported/blocked，显示在`info.startup.motion_music`。已经开始的播放可能到本次结束后才静音。

`controlled_spatial.standby_voice_enabled: false`（默认）同样在容器启动时关闭厂商ID-7“待命中”曲目`7_standby.mp3`，独立备份至`.motus-standby-voice/`；设为true恢复。状态显示在`info.startup.standby_voice`。两项音频设置独立处理，任一失败不会阻止另一项；不关闭speaker/TTS或报警音。

上述速度和时长上限属于**驱动策略**，不代表厂家最大能力或完整的工作空间、关节限位及碰撞保护。
末端和躯干目标仍需由调用方根据机器人实际安全工作空间设置；关节限位检查不等同于碰撞保护。

### 关节限位配置

默认 arm/leg 的 `*_limit_source: vendor_runtime` 与厂商SDK的运行时模型取限位方式一致：
读取实际 `/humanoid_description` 中14个手臂及4个下肢关节的有限上下界，再与 `/kuavo_configuration`
的20关节度制上下界取交集。要求关节类型、名称、数量、顺序及版本身份校验通过；
按1s周期刷新，身份租约失效或参数缺失时拒绝新动作并终止在途关节动作。
不使用随包显示模型、通用±π或其他型号的限位兜底，也不截断越界目标。
这是厂商运行时配置校验，**不等同于独立硬件认证**；健康状态明确标记 `hardware_verified: false`。

需要现场核验记录的部署可改为 `arm_limit_source: commissioned`；省略该键的旧配置仍使用此严格策略。
下肢也可设置 `leg_limit_source: commissioned` 要求现场核验记录。显式填写 `sensors.joint_limits` 时优先使用该记录，
记录错误或缺少arm字段不会退回厂商配置。记录格式如下：

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
`base_drive.info.feedback.robot_discovery.joint_limits` 查看。运行时模型指纹或配置身份不匹配时不允许关节动作。

模式：0=NoControl（被动）、1=ArmOnly、2=BaseOnly、3=BaseArm、4=ArmEeOnly。
底盘要求模式2/3；双臂关节要求1/3；末端允许1/3/4。下肢、躯干和头部要求非0模式。
动作参数及关节目标/限位先校验，再按需恢复并等待恢复后的新鲜MPC反馈，最后准备模式及发布目标。
当前模式兼容且反馈新鲜时不重复切换；不兼容或NoControl时选择3=BaseArm，兼顾底盘和双臂。
现场版本63的厂商实现实际固定执行并发布BaseArm：模式服务虽会接受其他编号，
但不会改变实际运行模式。该机器正常动作直接沿用反馈中的BaseArm，不依赖切换到ArmOnly/BaseOnly；
驱动不暴露这些无效的手动模式操作，也不把服务受理冒充生效。
恢复后也会重新核实模式；失败或被停止打断时不发送原运动目标，并尝试软件暂停。
执行结果 `control_preparation` 报告是否恢复、是否改变模式及最终模式。
这不是绕过检查的强制恢复：开机未确认、反馈过期、目标越界、发送不确定或旧控制请求未结清仍会拒绝。
驱动不会调用 `/enable_lb_arm_quick_mode`，该服务会绕过MPC。
部署时应确认其他客户端没有开启 quick mode；本驱动没有该模式的官方反馈来源，不能自动判断外部设置。

## ACP、并发和停止语义

底盘、双臂和下肢的持续/轨迹动作声明 `x-completion`，返回唯一 `action_id`，
结束后 POST `${AGENT_CORE_URL}/api/acp/complete`。资源分别为 `base`、`arm_l/arm_r`、`lower_body`；头部仅同步确认发令，保留 `head` 资源与停止hook，不声明异步物理到位。
`leg` 独占下肢通道，包含腰部关节。同通道在途动作收到新命令会返回 busy，调用方应等待完成或显式停止。
底盘速度/位姿也共享 busy 通道，避免位置指令被尚未过期的速度指令覆盖，或无效新参数误取消旧 ACP。
全局恢复/模式准备由驱动内部仲裁，不另行暴露执行卡；其短状态转换同步等待反馈。
同为BaseArm时不同资源可并发；若改变全局模式需要影响其他在途动作，则返回busy，不能静默重置它们。
每个暂停、恢复或模式服务分别使用最多2s的“受理+状态确认”总预算，而不是分别等待5s和2s；
一个动作可能依次需要恢复、等待恢复后的MPC流、切换模式，最多三段各2s准备预算，准备阶段等待不持有发布锁。
前台发令争用控制锁最多等待0.25s，未取得锁时明确返回 busy。反馈超时返回错误，要求查询状态后再决定，
不自动重试或改为后台继续等待，确认期间重连也不能以新会话反馈证明旧请求成功。
其中限时锁只适用于普通请求：控制服务采用“锁内验证并发送、锁外等待响应/状态”。
`stop_move`/中断hook 不走0.25s busy 拒绝路径，会优先阻止运动、发送零速、取消 ACP，
并使此前恢复/模式切换的令牌失效，再发全身暂停请求。旧请求尚未发送时不会继续发送，
已发送请求的迟到结果也不能解除本地运动阻塞。停止确认仍受网络和厂商服务是否受理的限制。
本地取消/超时不等于 ROS 端取消：每个已发送的恢复请求还单独保留未决记录，
只有同一连接代次内收到明确拒绝，或受理成功并观测到本次请求后的有效 `enable=true` 终态，才可结清。
存在未决恢复时，停止仍立即发送零速、取消 ACP 并尝试 disable，但返回
`physical_stop_confirmed: false`，不会因一次 False 反馈或缓存 False 宣称 `paused`。
即使旧恢复在该次 disable 等待期间才结清，该次停止仍不确认；调用方须在结清后显式再次暂停，
使最终 disable 发送在旧恢复终态之后。不会自动重试控制服务或自动重新启用。

`base_drive.info.feedback.remote_resume_unresolved` 暴露该状态；未结清时拒绝新恢复/模式切换。
只保留至多一个未决恢复的响应句柄，超时/本地取消后仍能收到迟到响应；传输断开不是控制器拒绝，
重连和新会话 False 不会消除旧请求的不确定性。无法取得旧请求终态或发生部分发送/断线时，
需现场核实并处置控制器状态；重启驱动本身不证明远端请求已取消。
模式写入同样保留最多一个未决响应，`remote_mode_unresolved` 为true时不允许新动作恢复并覆盖旧写入。
现场版本63的模式服务在修改模式后返回终态result；保留到该结果可避免旧写入在新准备之后才生效。
模式服务终态并不代替运动前的新鲜模式话题确认。
所有运动执行卡声明 `on_interrupt_motion` hook；speaker不声明打断hook。

位姿运动订阅相应的 `/lb_*_reach_time`（`std_msgs/Float32`），只使用指令发送后收到的反馈。
反馈须在3s内到达，估计值接受0–110s；ACP fallback timeout 为125s。
双臂/下肢 `move_joints` 发令前要求运行时 `/humanoid_description` 通过关节映射/名称校验，
并有0.5s内实测关节位置、上述策略认可的限位且目标在限位内；模型/限位缺失、不匹配或在途验证失效均拒绝/终止关节动作。
关节时间基准由`sensors.joint_timebase`选择，默认`auto`。已现场核对的版本63硬件循环每轮成功readSensor后用当前ROS时间填`header.stamp`，
而`sensor_time`从启动时间累加固定周期，会相对墙钟漂移。因此auto仅在运行时robot_version为整数63时采用`kuavo_v63_cycle`：
以header检查0.5s发布新鲜度（未来容差0.05s），以合法非零sensor_time辨别独立采样；两种时间均须递增，
相邻周期/头时间增量相差超过0.05s时拒绝。重复sensor_time即使带新header也不刷新测量，倒序header亦不接受。
其他版本auto使用`wall`，仍对sensor_time本身检查同样的0.5s/0.05s墙钟年龄；可显式配置wall，
不因时间过旧而自动学习偏移或切换策略。显式kuavo_v63_cycle也要求运行时版本63；两台主机墙钟必须同步。
缺失、零值、陈旧或明显未来的所选时间戳不用于关节测量，不以回调时间或header.seq代替时间证据。
关节动作不依赖reach-time：现场v63的 `/kuavo_arm_traj` 路径未产生该估计反馈。
要求实测目标关节误差≤0.035rad，不同的指令后采集帧在原始周期时间与新鲜测量时间两个基准中都覆盖≥0.2s，
再返回 `measured_convergence: true`。回调重放、接收间隔及容忍的时钟误差均不能推进采集跨度。
未核对的启动相对时间、其他模拟时钟或固件时间语义变化需单独适配，不能关闭新鲜度检查绕过。
关节动作发令后115s内未收敛、实测丢失/异常会返回 ACP error 并尝试全身软件暂停。
底盘和躯干/双手笛卡尔动作的完成时间仍依据**官方到达时间估计**，明确标记 `measured_convergence: false`。
这些反馈没有命令 ID，因此需要保证本驱动独占所用命令话题及 `/enable_control`；
外部客户端同时发令可能导致反馈归属不明，也不在本驱动的控制顺序保证之内。

速度动作以20Hz发送，时长到达后归零，底盘额外等待文档规定的1s停止窗口，头部等待0.3s。
ACP 同样标记为时长/控制器超时推断。WebSocket发送成功仅代表写入连接，不代表 ROS 控制器确认执行。
发布器在连接时预先建立，并等待1s连接准备窗口；关节指令以实测收敛确认结果，笛卡尔指令以reach-time反馈估计结果。

首次运动发送抛异常也可能已部分或完整送达控制器。驱动在释放发令锁之前设置
`motion_blocked` 和 `motion_send_uncertain`，随后在锁外等待全身暂停，取消在途 ACP 并以 error 结束一次。
请求返回原 `action_id`、`command_delivery: uncertain` 和 `physical_stop_confirmed: false`，
不会因错误返回就遗忘可能已执行的目标。暂停仍发送零速和 disable，但即使收到 False 也不宣称停止已确认。
该不确定性在进程内锁存，重连不会清除，拒绝新运动、恢复和模式切换；退出时仍尝试暂停。
必须现场核实并处置控制器遗留目标后再重启恢复，重启驱动本身不能证明旧目标已取消。

执行卡系统 `start`/`stop` 是立即返回 `ready`/`idle` 的生命周期标记，不访问机器人或等待控制锁。
`info` 另行报告实际连接状态和控制反馈；生命周期 `ready` 不表示已连接或允许运动。
已接受的动作仍由驱动运行时和 ACP 跟踪，不因生命周期标记而丢失；需要停止运动时必须调用
`stop_move` 或触发执行卡的中断 hook。驱动退出时会清理在途动作。

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
`base_drive.info.feedback.enable_verification` 提供有效性、年龄、来源及错误。
ROS 图查询和 latch 重新投递是状态通路检查，不是带生产者时间戳的硬件心跳，也不证明物理静止；
ROS master 缓存、具体 rosbridge 版本和真机控制器行为仍需现场核实。

## 传感输出与能力边界

已移除公开 `state` 卡；控制安全检查的内部反馈订阅仍保留。
诊断通过 `base_drive.info.feedback` 获取，包括连接、控制反馈、robot_discovery 和 host_runtime，
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
`joints` 输出 invalid、空关节数组和原因，静态 `model` 仍可读取。`base_drive.info.feedback` 中的
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
并条件补充openssh-client/sshpass，分别用于DDS participant、遥测/图像及跨机自动部署。不安装本地 ROS2 `geometry_msgs`：
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
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/call","params":{"name":"base_drive","arguments":{"action":"info"}}}'
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

### 2026-11-19 卡片调整验证边界

新增方向、限位转换、同步头部回执、下肢实时限位/升降逆解、底盘三档闭环速度/持续运动和共享后台启动的离线回归测试。读取了三个板子的厂商源码与配置；新增的升降及底盘位姿反馈控制尚未做旁站运动验收。指令限速不等于实测速度保证，关节限位也不等于碰撞保护。

本轮388项离线测试、Python编译和service.yml检查通过。上位机候选镜像基于当前release构建；只读验证确认arm/leg实时限位有效、重复waist已移除、头部同步发送语义生效。单独关闭ID-1运动音乐的操作返回disabled，其他曲目未改。未向电机发送测试目标；升降、连续运动和三档位姿闭环仍待旁站验收。

2026-11-19 有人旁站验证：低速 `move_relative(x=0.2,y=0.2,yaw=90)`；原比例控制停在约0.072m/2.35°残差，末段积分修正后约13.1s完成ACP，完成时原始里程计残差约0.0182m/0.987°，测试后软件暂停。仅覆盖本次姿态/地面/速度；建图修复已在真机以只读原始里程计停稳探针验证，未创建或切换地图。

近似完成是驱动的任务结束策略，不代表厂商精度保证；仅应用于`base_drive.move_relative`与`head.move_pos/zero`，不改变`controlled_spatial`的原生任务/地图到达确认，也不改变arm/leg的测量收敛要求。

重启恢复：`controlled_spatial.info`在四类导航反馈尚未齐备时返回`loading`及`feedback_status.missing`，保留后台启动结果，不将一次状态读取超时当作启动失败；实际导航仍严格要求新鲜完整反馈。底盘speaker进程每2s核对自身start/stop/status的ROS注册，启动宽限10s、连续三次缺失/无法验证即退出，由systemd自动重启并重新注册；处理控制板ROS Master重启而底盘进程仍存活的情况，不重放音频start或PCM。

### Native navigation velocity ownership

Native navigation uses the chassis path tracker. Before submitting its goal,
`controlled_spatial` disables the upper controller's `/enable_vel_control` and
requires fresh `/enable_vel_control_state` feedback. Otherwise upper MPC output
can select `nav_mpc_move_base` and override the native navigation velocity.
`base_drive` and `base_servo` restore this switch before publishing; starting
mapping restores the upper MPC remote-control route without resuming motors.
Stops retain the software stop and require native cancellation plus fresh
stationary odometry. Factory historical task queries can retain status 1 after
a goal has been replaced; that history alone does not prove it is still active.

On the commissioned robot, a supervised `navigate_to_pose` test with a 0.2 m
forward target and 0.1 m/s requested speed kept `nav_mpc_path_track` selected.
Map feedback recorded approximately 0.150 m displacement and 0.050 m remaining
error; native status changed from 1 to 2 and the driver completed after its
stationary check. The factory request uses a 0.1 m position threshold. The test
ended with confirmed whole-body disable and chassis software stop. This checks
the short navigation/control-ownership path, not long-route or obstacle trials.
