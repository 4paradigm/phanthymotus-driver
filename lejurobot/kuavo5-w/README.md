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
本驱动（ROS2 Humble 容器，MCP :15749/mcp）
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
MCP 仅监听 `127.0.0.1:15749`，只接受本机无 `Origin` 的服务端请求，并校验 `Host` 为该端口的
`localhost`/`127.0.0.1`；浏览器预检、任意浏览器来源及外部客户端均被拒绝，不返回通配 CORS。
页面通过 Agent Core 调用驱动，不直接访问运动端口。这是同机进程信任边界，不是本机进程之间的身份鉴权。
原默认端口15747已由主分支Helios驱动使用；本驱动加载旧配置时将该值映射到15749，
不改写持久化文件，其他自定义端口保持原值。15748仍是独立宿主机runtime manager端口。

### 首次部署与自动准备

镜像包含全部自有配套程序和安装器（`bootstrap.py`、`host/bootstrap_remote.py`、`host/`）。
Dashboard/PR镜像部署片段把宿主机 `/opt/phanthy-motus/drivers/lejurobot-kuavo5-w/` 挂到 `/config`；
首次允许Docker创建目录，驱动仅在指定路径缺失时独占创建 `/config/config.yaml`（0600）。
已有操作配置、关节限位、禁用设置和凭据不被镜像默认值覆盖；不再要求部署前手工创建配置文件。
`CONFIG_PATH` 显式指向其他路径但文件缺失，或已有YAML损坏时仍报错，不悄悄回退。

整机连接默认值统一位于 `connection_defaults.py`：`DEFAULTS` 保存三台主机的地址、账号、ROS IP，
`DEFAULT_PASSWORDS` 保存对应默认SSH密码。`base_drive`及其他执行卡不再提供连接参数configSchema或config动作；
不需要往画布放置额外配置卡。日常动作走ROS1/rosbridge，SSH供统一后台准备/修复三台主机及麦克风音频采集。

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
控制下位机192.168.26.1/lab；独立底盘192.168.26.22/jt。控制器ROS master为192.168.26.1:11311；底盘原生导航、状态和键盘守护使用底盘自己的ROS master（SSH到底盘后为127.0.0.1:11311）。`provisioning.master`仍用于控制器/桥接；可用`provisioning.chassis.master`覆盖底盘原生主站，旧配置未提供该字段时也默认使用底盘本机。两套主站不混用：开机遥控释放前对上位机G12和`/move_base/base_cmd_vel`的检查从控制器发布者建立只读TCPROS连接，核对发布者及消息类型，不把这些话题转发或注册到底盘控制图中。
`provisioning` 可覆盖master、每台机器的address/user/ros_ip/workspace及上位机head_workspace；
地址/账号也可用 `KUAVO_<ROLE>_ADDRESS/USER/ROS_IP/WORKSPACE` 环境变量覆盖。
自动发现对应用户及/home下的厂商应用/控制工作区，匹配不唯一就报错，不猜测另一份工作区。

### 部署时提供凭据（无需卡片配置）

以下都是可选的部署级覆盖，整机只配置一次。按操作人员要求，本版本保留内置默认密码；
默认账号密码未变时，不要求额外提供文件。镜像升级不会修改机器人账号密码。
内置值会出现在源码和镜像中，不作为私密凭据；不希望使用时可按角色关闭回退并提供私有密码或密钥。

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

`host_runtime.remote_ready_on_first_boot: true` 在上述初始化/暂停之后，允许驱动为本次下位机开机做 **一次** G12遥控交接。监管服务本身依然不请求恢复控制。交接要求开机完成时间在60秒内、G12转换器来自已核对的本机SDK进程、8轴/11键消息新鲜且摇杆回中/无按键、机型及全身暂停反馈可信，并且没有建图、导航、关节任务、待确认操作或用户控制请求。任一前置条件不足即保留暂停并报告原因，不用“先前进一下”探测。

控制写入之前，底盘状态连接暂时不可用会在同一次交接内最多等待20秒，且仍受60秒开机窗口限制；等待期间每次重查停止、连接代次和控制权限。任意使能、软停或释放写入开始后均不自动重试整段交接。等待状态显示为 `preparing/waiting_for_chassis_feedback`。

开机完成报告早于独立使能来源校验时，显示 `waiting/waiting_for_enable_verification`，在原60秒窗口内继续只读等待，不提前消费控制器交接许可。停止、用户动作、连接变更和窗口过期仍会终止交接。

机器人型号参数发现尚未完成或校验已过期时同样先等待，显示 `waiting/waiting_for_robot_identity`。后台发现恢复后才允许继续；不绕过型号校验，不延长开机窗口。

导航Web连接不可用时，遥控交接改用底盘本机ROS只读检查：两次健康/AUTO/非建图/非运动状态，中间核对新鲜且持续静止的里程计。该备用操作不发模式、使能、软停或任务命令；实际控制交接仍保留原有保护检查。

遥控准备由容器后台执行，不依赖画布开启智能控制。交接先建立上下两块板之间的速度桥，成功后登记遥控路由；空闲时由G12控制，底盘控制卡执行时临时接管，正常完成后归还。显式停车、急停、故障或未确认结束的操作不会自动恢复遥控。已有 `/config/config.yaml` 不会随镜像升级覆盖，旧配置需在 `host_runtime` 下补上 `remote_ready_on_first_boot: true`；仅修改配置不会立即使能当前机器人。

驱动先确认底盘软停、清理旧任务并暂停原生跟踪器，在软停保护下恢复全身BaseArm和上层速度通路。最终由底盘本机再次核对AUTO、非建图、健康/使能、连续零速度输出、新鲜G12回中及里程计停稳，确认`nav_mpc_move_base`来源后才解除软停；全程只发零速度，不发位移或关节目标。上位机Linux boot ID许可在MCP接收请求前消费，下位机boot ID许可在任何控制写入前原子消费，两者均存入现有持久化数据库；因此即使容器在收到下位机反馈前重启，也不能重新登记自动恢复。容器重启、失败、重连、用户停止都不重试；已经在运行的外部控制器也不会自动接管。删除持久化数据库会丢失该历史，因此不应把清库作为恢复遥控的办法。

卡片生命周期start不取消/抢占这次启动准备；底盘卡只返回ready并说明准备已延后。遥控已就绪时，两张底盘卡start保留现有上层通路，`base_servo.start`只挂接输入并保持paused，需显式resume接管。任何运动、建图、导航或停止请求都会撤销尚未完成的自动交接；旧后台流程不能在任务开始后恢复遥控。建图仍走MANUAL网页键盘，上层速度关闭；原生导航走AUTO/`nav_mpc_path_track`，上层速度关闭；这两种任务不会因启动准备而被切回遥控。

诊断见`base_drive.info.control_ownership.boot_remote`（以及其他含control_ownership的info）：`waiting/preparing/ready/skipped/error`是交接结果，当前使能仍看独立实时反馈。升级后必须让新的下位机监管服务在下次正常开机生效；旧服务不提供boot ID/G12校验信息时自动交接会跳过。仅只读使用可设置`remote_ready_on_first_boot: false`或`motion_enabled: false`。

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
| `camera_depth` | 头部640×480 uint16毫米深度，经Orbbec SDK软件D2C对齐头部RGB；`image/depth-zlib` 无损压缩，0无效；仅在运行时验证像素几何一致后发布 |
| `imu` | 优先独立 `/imu`，全身 raw 为备用；不再要求全身控制器存在才能读取 IMU |

电池转发器校验现场三种已核实 topic/type/MD5 组合，动态加载发布者提供的有界定义，输出标准 String。
四路图像分别使用独立 WebSocket 接收器和发布线程，目标上限10Hz；只保留最新帧，DDS BEST_EFFORT/depth1。
底盘 JPEG 不重新编码；头部通过宿主机已有 OpenCV/numpy 编码，未增加驱动镜像依赖。
现场旧版 ROS1 缺少 compressed_image_transport 插件，`host/head_jpeg.py` 保留原始 header、处理行填充，
以 latest-only 方式将头部原始 RGB 转 JPEG。按现场授权，头部共用RGBD进程开启深度及深度激光，RGB/深度均为640×480@15Hz源流。
`host/head_depth.py` 保留原始header，将已由厂商驱动换算为毫米的16UC1图像转换为小端无损zlib；不使用PNG/RVL compressedDepth格式，不增加镜像依赖。
RGB和深度卡共享物理相机进程，启动任一头部卡可能同时打开两路源流；只有处于运行状态的卡输出DDS数据。

头部中继按源时间戳配对后唤醒编码线程，以最多10Hz输出最新帧；头部两路不再叠加rosbridge的100ms限速。
`info.feedback.source_age_at_receive_ms`显示源时间戳到驱动接收的观察帧龄，
`publish_delay_ms`只表示接收后到DDS发布的耗时；前者依赖SDK时间戳，不代表已验证的采集时钟映射。
两者均不包含Core转发、网络和浏览器显示延迟。厂商原始深度的滞后不会被重写时间戳掩盖。
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
该开关允许运动请求及上述显式配置的一次性开机遥控交接；传感器读取、卡片start/info不会触发恢复或发运动目标；原有运行时检查仍生效。
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

视觉导航接线：`camera_head → vop → navi`（检测结果）、`camera_depth → navi`（米制深度）、`loco_state` 的 `state/odom → navi`，最后 `navi → base_servo`。需要预览叠加时，也可将 `camera_head` 的图像直接连入 navi。**不要把底盘 camera_left/right 的检测框与头部 camera_depth 混用**；它们来自不同镜头，像素位置不对应。底盘RGB如需视觉深度，应从同一路RGB生成 visual_depth。头部相机可转动，当前 navi 不补偿头部姿态，因此导航前须让头部朝正前，导航期间保持不动。

四路相机在 `info/start` 返回 `motus.camera/1` 声明；分辨率取实际JPEG或深度图，内参K/D来自各自ROS CameraInfo，不复制其他型号的FOV。缺少标定时保持 unknown。头部深度只有在 `depth_registration=true`、RGB/深度都使用color optical frame、分辨率和K一致时，才与RGB共享镜头ID并输出有效帧；等待校验时为 loading。VOP沿输入传递镜头ID和分辨率，navi按声明的 `image/depth-zlib` 绑定深度，而非依赖话题名后缀。更新后须重新启动上游和navi，以刷新启动时冻结的声明。

对照[PR519](https://github.com/4paradigm/phanthymotus-driver/pull/519)的空间观测差异和真机只读核对结果见
[SPATIAL_INPUT_AUDIT.md](SPATIAL_INPUT_AUDIT.md)。已补实时URDF＋实测关节角的动态机身到头部相机外参（厂家模型估计，未做安装标定）；
相机IMU的计算端口存在也不代表原始gyro/accel一定在输出。融合/建图前应检查实时数据，不能只看服务active或参数enabled。

`base_servo.info.control_interface.footprint` 优先读取真机 `/humanoid_description` 中 `base_link` 的直立圆柱碰撞体，标注 `source: vendor-spec`、`scope: chassis`，附运行时URDF哈希；当前v63模型半径0.37m，因此前/后/半宽均为0.37m，不另加规划器padding。此为厂家底盘模型，非全身实测包络，机器人高度仍为null。模型缺失/不支持时退回厂商costmap多边形加padding并标记estimate；若costmap包络更大则保留更大的估计值。当前现场costmap为±0.34m加0.01m padding。伸臂、头部姿态和实际避障效果须另行验证。


速度协议与 R1 一致：ROS2 `std_msgs/String` JSON，`motus.control/1`、`mode=twist`、`dof=6`，机身系 `[vx,vy,vz,wx,wy,wz]`，单位 m/s、rad/s，vz/wx/wy 必须为零。复用主节点 domain 42 的隔离 DDS，工作线程经 rosbridge 转发 ROS1 `/cmd_vel`。期望10Hz、最大20Hz；校验序号、TTL和观测时间，300ms断流发零速、拒绝500ms以上旧观测；慢速使能完成后再次检查首帧是否过期，不执行积压旧目标。ROS2回调仅保留最新消息，不等待SSH或控制服务。

`pause`及全局中断锁存暂停，需`resume`才接收后续帧；普通`base_drive.move/move_relative`优先暂停伺服。与厂商导航/建图共用内部base通道互斥。速度流不创建ACP pending，视觉导航到达由ActuCore navi负责ACP。容器启动不会自动订阅速度流。`info`返回输入话题、描述符、底盘准备状态、最近校验结果和执行次数。厂商控制器的速度超时机制保留。

`base_servo.start` 成功接受输入订阅统一返回执行器生命周期 `state: ready`；
`stream_state` 与 `startup` 单独报告 paused/loading/running 和底盘准备结果。
ready 仅表示卡片生命周期已接受，不表示底盘已完成准备或已使能；未完成准备时不执行速度输入。
`info` 保留实际流状态，重复 start 同样返回 ready，且不重新接管其他底盘动作。

速度依据为2026-11-19真机只读核对：正在使用的 `humanoid_wheel_interface/config/kuavo_s63/task.info` 中 `wheel_move.max_vel=[0.8,0.8,1.6]`；底盘 `leju_node.launch` 及运行时 `/leju_node/max_linear_vel`、`max_angular_vel` 均为1.0。驱动取交集：平移合速度0.8m/s、旋转1rad/s。这是厂商软件命令包络，不宣称硬件极限；其他固件仍可能进一步限幅。官方《轮臂控制API》给出了单位和停止语义，没有统一的硬件最高速度数字。`controlled_spatial`仍遵守厂商路径规划器自己的速度限制。

新增模块复用既有 `common/control`、std_msgs及rclpy，无新增pip/apt依赖。旧的持久化配置需加入`plugins.base_servo: {enabled: true}`，显式禁用配置仍受尊重。404项离线测试覆盖速度流、断流、旧帧、全局暂停、动作抢占、重连及使能期间暂停的晚响应竞态。新镜像已验证domain 42下的真实DDS收发和断流零速，ROS1端使用内存替身，不连接电机；尚未以新档位执行真机运动或验证整条视觉导航到达。

sensor/actuator 卡片实现系统生命周期动作；`model` 为读取型 resource。
`model.start/stop/info`仅返回独立生命周期状态，不加入传感器发布循环；
显式`read`或不带action的模型请求返回普通`{"urdf": "..."}`字典，保持前端骨架读取兼容。
业务参数通过 `x-action-params` 分组并声明动作各自的 required；`dispatch()` 返回普通 dict，HTTP 层只包装一次 MCP content。
`config.yaml` 的 `plugins.<card>.enabled` 决定卡片是否暴露。

初始化流程等待实际BaseArm反馈（mode=3），再进入软件暂停；启用一次性开机遥控交接且全部保护通过时会准备G12，其他情况下保留暂停。新的运动动作仍可按需准备控制。
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
| `start_mapping` | `map_name` | 按厂商流程在MANUAL启动Cartographer，保持网页键盘通路、关闭上位速度；不覆盖已有同名地图 |
| `stop_mapping` | 无 | 保存地图/路网、重启厂商地图服务，载入并确认地图文件身份 |
| `reconcile_mapping` | `map_name`、`confirm_abandon=true` | 仅在底盘已重启且旧任务不再可能运行时，放弃对应的失效建图记录；不保存、不删除文件/点位，不发运动或恢复控制 |
| `tag_place` | `name`，可选 `description` | 保存当前 map 系 x/y/yaw；建图期间先绑定当前建图会话 |
| `untag_place` / `list_tags` | `name` / 无 | 删除/列出当前地图语义点，列表含相对距离和方向 |
| `list_maps` / `load_map` | 无 / `map_name` | 列出底盘和上位机归档地图 / 如有需要从归档恢复到底盘，再载入并切回自动模式 |
| `delete_map` | `map_name`、`confirm_delete: true` | 永久删除非活动地图在底盘和上位机的文件，以及关联点位。支持仅存于上位机的地图 |
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
`spatial_map` 是独立的地图预览卡，读取上位机的 `<data>/maps/kuavo5w` 快照，
默认展示最近归档的地图版本。`list_maps` 列出可预览地图，`select_map` 切换预览。
导航卡的列表还显示尚未归档的底盘地图。仅在上位机存有归档的地图标记为 `archive_only`，可直接用 `load_map` 载入。
要删除底盘正在使用的地图，请先用 `load_map` 切换到另一张地图。
画面显示占据栅格、点位标记及其方向，`info` 返回点位名称和坐标。
点位按栅格和路网指纹匹配，活动导航地图仍由 `controlled_spatial.load_map` 切换。

连接参数共用 `provisioning.chassis`，不需要每张卡重复填写 SSH 信息。
SSH RPC 辅助源码随镜像构建，每次在底盘执行固定操作，使用当地已安装的 ROS 消息类；
不需要手工安装新服务，不额外开端口或复制一个可能不兼容的消息工作区。

导航的提交、查询和取消均核验为同一服务提供者。ACP 使用厂商返回的 task_id 跟踪，
到达还需核对地图、位姿和零速反馈；不是“服务返回 success 就完成”。
速度上限默认0.15m/s、可设范围0.05–0.5m/s，实际速度由厂商规划器和避障决定。避障距离为0.5m。
`base_drive` 和新卡共用内部 base 动作锁及 Agent Core 的 base 资源声明，中断走 `stop_nav`。地图流程每次ChangeMode均等待实际模式反馈再发下一条命令，不把接口应答当作模式已切换；载图若收到厂商8211模式前置条件拒绝，保留错误ACP但不遗留“远端仍在执行”的锁，允许重试。超时/断连仍保留未确认状态。
停止会先请求全身禁用，不因旧导航提交阻塞这一步，再按顺序原生取消并核实停稳。

**载图不等于定位成功**。载图/保存不会伪造 positionInitialized、解除底盘软件急停、硬件急停、
碰撞或故障。定位未就绪或软件急停仍有效时，导航明确拒绝；软件急停可由卡片 start 按上述顺序准备，定位仍需按厂商操作恢复。
`load_map` 确认地图身份后会短暂等待厂商定位反馈，返回 `localization.state`。
`initialized` 表示厂商已报告在所选地图上完成初始化，并附带可用的原始匹配分数。
`unconfirmed` 表示载图成功，但还没有确认定位。打点记录的是导航目标，不能作为机器人当前位置的默认值。
新建地图会记录建图开始时的静止位姿，并与保存后的地图指纹绑定。
载图后，若全身已暂停且底盘软急停有效，驱动用实时激光在该起点附近匹配地图。
匹配明确时才向 Cartographer 发初始位姿，并用激光与回读位姿复核。
如果起点匹配不明确，载图结果会提示定位未确认。回到起点附近、停稳后重新调用 `load_map`。
旧地图没有建图起点记录时，继续使用厂商原有的定位流程。
`base_drive`和`controlled_spatial`可以同时start，共用底盘预检。已有建图会话时，两张卡的start只报告准备已延后，不取消任务、不切模式、不解除暂停；`base_servo.start`只连接输入并保持paused。厂商跟踪器队列为空且明确发布`FINISH_STOP`时可能不再发布速度；此时暂停确认使用底盘软停和新鲜静止里程计，不等待不存在的空闲零速流。已结清但遗留的`chassis_preparation.delivery_uncertain=false`记录，在无控制/地图/导航任务且两次新鲜网页反馈确认空闲时，可通过只读`held_stationary`再次确认软停、跟踪器结束和静止里程计后自动核销；不解除软停、不恢复运动。真正未确认的释放请求、导航提交和其他忙状态仍保留阻断。

建图采用厂家网页的手动通路：`start_mapping`先关闭上位机速度输出、确认底盘软停并取消旧原生任务，在底盘MANUAL模式开始建图，之后保持MANUAL。无需G12遥控器，不再为遥控器恢复AUTO。全身BaseArm与底盘MANUAL是不同状态。`executing`仅表示操作已开始，ACP完成才确认建图就绪；若此期间打开`mapping_console`，面板自动等待同一次建图完成后再准备键盘，不发送方向或解除软停。停止、保存、断线、失焦或该次建图失败会取消等待，旧面板不会接管新建图会话。

`start_mapping`是本次建图遥控授权，返回及完成ACP都会用英文提示调用`mapping_console`、不要使用G12遥控器。在15678画布执行`controlled_spatial.mapping_console`打开内嵌面板（`motus.workbench/1`）后，面板取得焦点即自动准备键盘控制，无需额外启用确认。键盘准备异步检查/停止旧任务，确认MANUAL建图和驱动使能，并收到首帧零速后才释放底盘软停；不调用全身使能，也不解除硬件急停/碰撞/故障。准备期间的方向输入被丢弃，就绪后必须重新按键。`keyboard_open/command/release/status`保留为工作台内部租约接口，不作为独立的画布/LLM动作；操作者只使用`mapping_console`。停止或ROS重连使旧页面授权失效，必须重新打开面板才能重新取得控制。

按住Alt+W/S或I/K前后移动，Alt+A/D或J/L左右转向（不是平移）；也可按住方向按钮。上限0.15m/s、0.3rad/s，松开按键或Alt发送零速；面板失焦、页面隐藏或关闭会撤销控制。浏览器和底盘侧各有300ms心跳租约，底盘临时守护进程通过固定本机WebSocket `RobotMotion{vx,vy=0,vw}`发令，SSH断连/EOF、租约失效、MANUAL/建图/健康状态丢失时发零并请求底盘软停；这不是硬实时制动时延承诺。进程通过现有SSH流临时运行，不安装服务、不新增远程包或端口。

整个建图会话禁止`base_drive`、`base_servo.resume`及原生导航接管，即使键盘暂时未按下。独立的arm/head/leg可在全身控制已就绪且无需共享使能/模式变更时并行；否则明确拒绝准备，不暗中改变控制状态。`stop_mapping`自动撤销面板授权、释放键盘并等待确认停稳，无需人工先点释放；停止未确认时不提交保存。键盘仍占用时载图、删除继续拒绝。确认停止失败会持久化阻塞标记，`stop_nav`经原生软停及里程计确认后才能清除。保存/载图保持底盘软停，服务重启后再次确认，地图流程不会恢复键盘或全身控制。结束建图后，上位机底盘动作重新预检，不能复用建图前的启动缓存。`stop_move`/`stop_nav`仍是全身暂停，并同时撤销键盘租约。外部厂家网页/遥控器不在本驱动内仲裁，不能与画布同时驾驶。
建图、保存、载图和导航都返回 action_id 并通过 ACP 报终态。保存地图含厂商最终轨迹优化，可能耗时数分钟；保存响应最多等待25分钟，保存及随后的自动载图最多共30分钟。单独载图和启动建图最多150秒。HTTP调用立即返回executing，不等待优化结束；`info`在此期间显示saving及英文等待说明。保存期间缺少地图TF/定位帧不代表优化已失败，不要重启建图或重复提交保存。

若进程/机器曾重启，数据库可能仍保留`saving / ChangeCartoMap`等未确认阶段。卡片启动、状态/地图查询及地图/导航操作遇到这类无本地工作线程的旧会话时，会自动核对并清理失效会话记录，无需人工执行恢复动作。清理要求底盘确实已在旧操作之后重启，连续两次新鲜状态均明确未建图、未行驶且三类任务列表为空，同时没有其他控制权或停止不确定性；任何信息缺失或变化都保留记录，后续请求间隔至少2秒再自动核查。新记录使用底盘boot ID区分开机；旧记录还要求底盘开机时间晚于旧命令、时钟一致。容器重启、地图列表出现文件、或者网页显示未建图，都不能单独证明旧保存请求已结束；不要为绕过检查而重启正在保存的底盘。自动清理只原子整理本地会话/转换记录，保留审计、地图和点位，不切模式、不恢复运动，也不清理导航/软停等其他保护。清理后继续原请求；若原请求是`stop_mapping`，则返回`completion_basis=obsolete_mapping_session_cleared`、`map_saved=false`，不声称保存成功。`info.mapping_cleanup`提供自动检查状态；`stop`/`stop_nav`不等待此检查。

恢复后，`delete_map(map_name, confirm_delete=true)`可删除未被使用的残缺地图，包括0字节PGM或缺少部分配套文件的失败产物。删除按实际存在文件的大小、修改时间及SHA-256清单再次核对，不允许删除活动地图；备份和载入仍保留完整地图校验，不会把残缺文件当作有效地图。

地图数据统一使用上位机持久目录 `/opt/phanthy-motus/data`（Compose 同路径挂载）：

- 语义点、地图身份、未确认操作和最近一次地图流程耗时：`/opt/phanthy-motus/data/kuavo5w/controlled_spatial.db`。
- 地图文件备份：`/opt/phanthy-motus/data/maps/kuavo5w/<地图名>/versions/<快照指纹>/`，包含 `.pgm`、`.yaml`、`.pbstream`、`.path` 和 `manifest.json`；`<地图名>/current.json` 指向最后完整核验的快照。
- 厂商运行原件仍在底盘 `/home/jt/jmap_config/jmaps/`。上位机副本用于持久保存。显式调用 `delete_map` 会删除两处文件。

升级首次启动会用 SQLite backup 将旧 `/config/controlled_spatial.db` 迁入新目录，保留旧文件；目标已存在时绝不覆盖，显式 `controlled_spatial.db_path` 也不会被改写。容器删除/重建保留宿主机数据。手工运行需设置 `KUAVO_DATA_DIR=/opt/phanthy-motus/data` 并挂载该目录；未设置时保持旧数据库路径，地图备份关闭。
后台备份在启动和地图操作完成后触发，空闲时每分钟重试；建图/地图转换期间让路。逐块传输并核对大小、SHA256和传输前后文件状态，完整成功后才原子更新快照指针；失败不覆盖旧快照，也不阻断导航或ACP。`info.storage` 显示实际路径和备份状态。新增代码与数据卷不增加运行依赖，地图数据不进入镜像。

语义点绑定地图名、**PGM+YAML 的 SHA256**和路网 MD5；同名地图内容/坐标配置变动后，旧点不会继续导航。
保存建图成功后才把会话内的点绑定到最终地图身份。
丢失提交/地图操作确认时不自动重试，也不因容器重启忘掉在途状态；`info` 可查看 pending。
已知任务可再次 `stop_nav` 请求取消；提交去向、解除请求或地图变更仍不明时保持阻断，
需要现场核对厂商状态后恢复，不提供“忽略错误继续运动”的复位入口。若`stop_mapping`已在SaveCartoMap阶段超时，可再次调用`stop_mapping`接续收尾：只读等待原保存生成完整文件（含在同步保存完成后写入的有效`.path`，且所有文件连续2秒不变），不会再次提交SaveCartoMap；随后绑定已保存文件的指纹和原会话语义点，不自动载图。保存未确认前`load_map`仍会拒绝，不能清除记录强行载图。地图转换与原生导航的阻断原因分别提示；被阻断的leg等运动请求不会发出目标或主动暂停机器人。`stop_nav`仍请求全身暂停，但不会把未确认的地图操作当作已完成。
运行期间不要用另一客户端切换地图/改变坐标原点：周期核对不是原子的跨系统锁。

离线测试覆盖任务ID、取消顺序、重启保留、地图指纹、语义点、地图转换和两卡互斥。
已在真机只读确认服务同源、地图列表/PGM/YAML指纹及新鲜状态；
已有现场建图、保存、载图和小范围导航/取消记录；这些不代表全场景运动验收，不能以单元测试替代现场验证。

同一次建图可以反复打开 mapping_console：离开面板会释放控制并确认停车，执行 tag_place 后重新打开面板即可继续采图，地图会话和已打点不变。base_drive 与 controlled_spatial 可以同时 start，共享预检；执行时按底盘资源互斥。base_drive/base_servo 接管不能仅依据上位机 enable_vel_control 标志：还需在停车并暂停旧跟踪器、确认上位机零输出后，将底盘 mux 切为 nav_mpc_move_base 并核实反馈，再发布新目标。原生导航接管、地图流程、停止或重连后，上位机通路需重新确认。定时 move 的 ACP 仍仅表示发令持续时间和厂商看门狗窗口结束，不表示实测发生位移。

键盘守护因租约等原因主动停车时，stopped 确认可能在驱动等待 ready 的读取中到达。当该帧确认厂商 RobotMotion 零速已发送且新鲜里程计已连续静止时记为键盘停车（兼容旧版软件急停停车确认），退出时保留该确认；不再等待第二帧后将连接关闭误报为停止未确认。真实未确认仍保留保护。面板在停止警告后每秒只读刷新 keyboard_status，收到 idle 且 busy=false 才清除旧警告，不自动重新接管。info.keyboard.last_failure 保留经过白名单过滤的失败原因，避免把所有错误归为同一条泛化信息。

键盘SSH回复按块读取并保留未完成JSON行，避免逐字节读取超时后丢掉半帧及随后的停车确认。方向/状态请求回复期限仍为1.5秒，释放租约为12秒、全身停止为60秒；这只影响等待回复，不延长300ms运动租约。回复超时明确表示停止未确认，不能声称键盘已释放；键盘空闲状态也不能替代全身停止的独立确认。MANUAL 键盘松键只发送 RobotMotion 零速并保持会话，再按方向可继续；退出、断连或心跳超时发送零速并确认新鲜里程计静止，不要求 soft_estop=true，不反复切换调度软停。孤立会话的停止恢复使用同一零速链路。此修改尚未完成真机运动验收。

#### 厂商保存与载图耗时

`SaveCartoMap` 是同步的地图收尾调用，包含 Cartographer 最终优化及地图输出，并非只写一张 PGM 图片。现场 2026-11-20 的一次保存从约 00:31:59 持续到 00:51:08；期间仍有优化进度和剩余工作项，约19分钟主要发生在厂商内部。该实例不能作为正常固定耗时；轨迹、约束积压和计算资源都会影响保存，驱动不通过中断优化或改厂商SLAM参数来假装加速。保存之后收尾建图服务并保持停车；载入栅格/路网和重定位由显式 load_map 执行。

驱动减少了可避免的开销：模式已由新鲜反馈确认时不重复切换；载图等待使用轻量地图名查询，完成时再核对完整地图指纹；同一张内容未变、此前已确认载入且仍定位初始化的地图，重复 `load_map` 不再重载 Cartographer/路网，保留当前定位。内容变化、定位未初始化或无法证明状态时仍走完整载图流程。模式切换始终等待实际反馈，不能仅凭命令应答跳过前置条件。
`info.map_transition.stage`、`map_stage_elapsed_s` 和 `last_map_operation`，以及地图ACP中的 `timing`，给出当前阶段/累计耗时和各命令阶段耗时（含模式确认），用于区分厂商优化、重启、加载与通信等待。它们是实测耗时，不是预计完成百分比。

厂商编号音效不是任意PCM接口；本驱动使用已实际听到声音的底盘标准音频输出。

### 外置麦克风

提供多实例传感器卡 `ext_mic`，输出标准 `AudioChunk` / `audio/pcm-16k`：
16 kHz、单声道、S16_LE，32 ms 一块，可连接 ASR/VAD。
目前没有确认可用的内置麦克风，因此不暴露 `mic` 卡；旧配置中的 mic 不会重新开启该卡。

1. 将标准 USB Audio Class 麦克风插到上位机、控制下位机或独立底盘之一。
2. 设备后台每15秒只读探测一次；插入后稍等并刷新驱动卡片列表，在全局变量控制中选择麦克风。
3. 开启传感器卡即开始采集，关闭即停止；没有业务执行动作或“执行”表单。每个 ext_mic 实例独立选设备。
   无设备时显示“No USB microphone detected”，不会将主板录音接口或扬声器回放作为可用麦克风。
   start仍会重新探测，以确认设备未被拔出。

设备选择只显示 USB ALSA 采集端点，隐藏 PulseAudio 重复别名、Jetson APE 路由端点和未验证的模拟接口。
已保存的模拟输入选择也会拒绝启动。配置仅为 `device_index`，不需要每张卡填写 SSH 账号密码。
复用三台机器共享 SSH 设置；私有覆盖优先于内置默认值。固定 helper 随镜像打包，开启时经加密 SSH 执行，
无需手工安装常驻录音服务、开放新端口或挂载容器 /dev/snd；不新增音频 pip/apt 依赖。
远端需要已有 arecord，USB 专用声卡/特殊驱动需另行验证。

不自动录音、不保存录音、不向第三方发送。info提供真实字节数、RMS、peak和错误；
它们证明采集数据通路，不等于验证了人声。无设备/断连时不发布合成静音，不自动恢复录音。
每路独立工作线程、小队列BEST_EFFORT，远端帧时间用于顺序与延迟检查，DDS时间为驱动接收时间。
关闭卡片/容器会终止采集，SSH关闭或10秒无心跳时远端helper退出并关闭录音进程。

2026-11-20 三板未枚举到 USB 音频设备。模拟输入虽有 capture 接口，但测试为全零、低噪声或无数据，
底盘启动瞬态衰减后也只有低噪声，因此没有证据确认内置麦克风可收人声。
之前模拟通路的MCP/DDS测试不作为USB实物或声学验收；外置麦克风需实际接入后验证。

### 扬声器安装与使用

正常部署由容器自动安装到底盘。以下仅为人工排障时的等价命令，在**独立底盘电脑**执行：

```sh
sudo python3 host/install_speaker.py --user jt --master http://192.168.26.1:11311 --ros-ip 192.168.26.22 --volume 50
```

仅使用该机已有ROS Noetic、`paplay/pactl`和PulseAudio；不安装SDK、声卡库或TTS引擎，不在播放器中保存SSH凭据；默认连接值集中于驱动，私有覆盖密码另存于部署配置。
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
音量变化不重启播放进程/会话；`--volume`只设服务启动默认值50%。`info.output_status.volume_percent`显示底盘值；
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
  `set_height(height)`接受`waist_yaw_link`原点在`base_link`坐标系中的绝对Z坐标（m），不是头顶离地高度。厂商轮臂API的`/cmd_lb_torso_pose.linear.z`及控制器`task.info`中的`baseFrame/torsoFrame`定义了这一参考系。此前0.74–1.06只是驱动固定筛选值，没有足够依据作为该机器人的实际工作范围，现已移除。先调用`get_height`获取实测`height_m`与当前姿态下的`reachable_height_intervals_m`：由实时URDF和关节限位求解析边界，保持躯干x/俯仰及腰偏航；这些是运动学可达区间，并非碰撞/负载安全认证。不可达请求返回当前值和区间，不静默夹紧。通过厂商`/cmd_lb_torso_pose`联动膝、腿、腰，ACP仍用真实关节收敛确认；不支持的几何或缺失反馈在恢复控制前拒绝。 `get_height`另返回`head_top_height_m`：基于实时URDF头部球形碰撞包络和当前姿态的最高点估计，参考系仍为`base_link`，不是实测离地身高；缺少可用头部几何时为null并提供原因。`set_height`立即返回动作前当前头顶高度、`requested_height_m`及`target_head_top_height_m`预测值，`height_sample_phase=before_motion`；动作完成ACP再返回收敛后的当前高度（`after_measured_convergence`），不把目标值当作已到达实测值。
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
所有执行卡统一要求3=BaseArm，包括兼容保留的双臂末端接口；不再因单独使用手臂或底盘保留ArmOnly/BaseOnly。
动作参数及关节目标/限位先校验，再按需恢复并等待恢复后的新鲜MPC反馈，最后准备模式及发布目标。
当前模式为BaseArm且反馈新鲜时不重复切换；其余模式在没有在途动作时统一切回BaseArm，等待实际反馈后发令。
现场版本63的厂商实现实际固定执行并发布BaseArm：模式服务虽会接受其他编号，
但不会改变实际运行模式。该机器正常动作直接沿用反馈中的BaseArm，不依赖切换到ArmOnly/BaseOnly；
驱动不暴露这些无效的手动模式操作，也不把服务受理冒充生效。
恢复后也会重新核实模式；失败或被停止打断时不发送原运动目标，并尝试软件暂停。
执行结果 `control_preparation` 报告是否恢复、是否改变模式及最终模式。
这不是绕过检查的强制恢复：开机未确认、反馈过期、目标越界、发送不确定或旧控制请求未结清仍会拒绝。
驱动不会调用 `/enable_lb_arm_quick_mode`，该服务会绕过MPC。
部署时应确认其他客户端没有开启 quick mode；本驱动没有该模式的官方反馈来源，不能自动判断外部设置。

## 卡片控制权与组合使用

**BaseArm是全身控制模式，AUTO/MANUAL是厂商底盘模式，速度通路是第三项独立状态。**
驱动不再在卡片之间切换ArmOnly/BaseOnly；原生导航只交接底盘速度输出，不停用手臂、头部或下肢。

| 卡片/操作 | 实际执行者与通路 | 控制模式 | 组合规则 |
| --- | --- | --- | --- |
| `base_drive.move/move_relative` | 驱动定时/里程计闭环 → `/cmd_vel` → 上位机MPC → 底盘 | BaseArm（3）、底盘AUTO；上位机速度输出开启 | 独占base；可并行arm/head/leg；显式命令会暂停base_servo |
| `base_servo` | ActuCore navi速度流 → ControlSink → `/cmd_vel` → 上位机MPC → 底盘 | BaseArm（3）、底盘AUTO；上位机速度输出开启 | 独占base；可并行arm/head/leg；显式底盘/导航命令优先，之后须resume并发送新帧 |
| `controlled_spatial.navigate_*` | 厂商MoveToTarget → 原生路径规划/跟踪 → `nav_mpc_path_track` | BaseArm、底盘AUTO；上位机速度输出关闭 | 独占base；目标已确认且任务正常时可并行arm/head/leg；不与遥控器/其它底盘动作同时控制 |
| `controlled_spatial.start_mapping/mapping_console` | 厂商Cartographer；画布键盘 → 底盘临时看门狗 → RobotMotion | 底盘MANUAL；上位机速度输出关闭；面板不使能全身 | 建图会话独占底盘通路；启动其它卡不抢权；独立关节仅在共享状态已就绪时并行 |
| `controlled_spatial.stop_mapping/load_map` | 厂商地图服务 | 先释放键盘并确认停稳；按当前模式选择MANUAL零速/AUTO调度停车，再确认MANUAL；保存后回AUTO并停车、不会载图，load_map独立载图和定位；不改全身BaseArm | 地图转换独占base；未确认的转换不能被新底盘运动覆盖 |
| `arm` | `/kuavo_arm_traj` → 厂商全身控制器 | BaseArm；不修改底盘速度开关 | 独占arms，与base/head/leg独立 |
| `head` | `/robot_head_motion_data` → 厂商头部控制 | BaseArm；不修改底盘速度开关 | 独占head，与base/arms/leg独立 |
| `leg` | 下肢关节轨迹/躯干位姿 → 厂商MPC/WBC | BaseArm；不修改底盘速度开关 | 独占lower_body，包含腰；与base/arms/head独立 |
| `speaker` | PCM流 → 底盘播放服务 → 扬声器 | 不参与运动模式 | 可与执行/传感器卡并行 |
| `joints/model/imu/loco_state/battery/laser_scan/camera_*` | ROS反馈、模型及独立图像/点云转发 | 不参与运动模式 | 不占运动通道，不改变控制权 |

卡片处于启动状态不等于占有底盘：启动另一张底盘卡不会取消正在执行的底盘任务。
`base_servo`在底盘已被占用/未确认时允许连接输入，但保持paused，必须等当前操作结束后resume。
底盘动作之间仍互斥，`base_drive`和原生导航之间须等完成或stop后再发令；伺服流不能反抢离散动作。
正常切换底盘速度通路以及底盘预检不会暂停正在运行的独立关节动作。
没有在途动作时才允许全身恢复/全局模式准备；外部模式改变、反馈过期或真正的未确认写入仍安全拒绝/暂停，不能以“兼容”为由忽略。

`base_drive/arm/head/leg/base_servo/controlled_spatial.info.control_ownership`只读显示：
BaseArm策略、实测模式、确认的使能状态、底盘速度通路、厂商实际速度源及其回执年龄、在途资源拥有者、当前控制转换。
`active_resources`表示驱动持有的动作通道；遥控器不在驱动内仲裁，不能由空通道推断摇杆中位。
自动底盘运动时不要同时操作遥控摇杆；硬件急停始终保留。

停止范围必须区分：`stop_move`、`controlled_spatial.stop_nav/stop`和运动异常走**全身软件暂停**，取消所有在途动作；
`base_servo.pause/stop`只暂停该伺服流并向其拥有的底盘通道发零速，不停止其它独立通道。
正常ACP完成不会软件暂停全身，也不会取消其它卡片。

## ACP、并发和停止语义

底盘、双臂和下肢的持续/轨迹动作声明 `x-completion`，返回唯一 `action_id`，
结束后 POST `${AGENT_CORE_URL}/api/acp/complete`。驱动内部通道为 `base`、`arms`、`lower_body`、`head`；头部同样使用ACP，按实测到位或稳定后的有限残差结束，并报告误差。
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
| `camera_depth` | `/<namespace>/state/camera_depth` | CompressedImage，format=`image/depth-zlib`；zlib解压后640×480小端uint16毫米深度，0无效，RGB对齐后使用color optical frame并保留采集时间 |

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

`odom_interface` 明确提供输出 `topic`、10Hz频率和 `timing.stamp_source: received`。速度摘要的顶层时间戳固定为驱动收到ROS消息的主机时间；厂商时间和选择原因保留于 `vendor.source_timing`，原始ROS header及 `pose` / `vendor.motus_slam` 的采集时间保持不变，避免把接收时间冒充测量时间。跨板时钟及测量到接收的延迟上限尚未验证，因此不虚报 `max_measurement_delay_ms`；navi仍可提示未声明测量延迟并保留通用有效期。当前Agent Core还必须将该声明按连线话题转发为navi的 `odom_interfaces`，仅在驱动info中存在声明不等于navi已经收到；此驱动不修改ActuCore或Agent Core。
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
MCP 端口15749，Agent Core回调默认 `https://localhost:15678`，可通过 `AGENT_CORE_URL` 覆盖。
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
curl -s http://localhost:15749/mcp -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}'
curl -s http://localhost:15749/mcp -H 'Content-Type: application/json' \
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
requires fresh `/enable_vel_control_state` feedback after changing it. This
state is latched, not a periodic heartbeat: a confirmed value is retained within
the same bridge connection and invalidated on reconnect. Otherwise upper MPC output
can select `nav_mpc_move_base` and override the native navigation velocity.
After confirming the upper velocity switch, the goal RPC holds the chassis
software stop, cancels the dispatch task, and pauses the actual LQR tracker via
`/path_track_request_service` (`OP_PAUSE`). It requires fresh paused/empty-task
feedback, at least 0.3 s of zero tracker output, and fresh stationary odometry.
The factory `MotionState` header holds the last event timestamp even while
publishing continuously; tracker feedback is checked using new nonlatched
messages from `/lqr_path_track`, rather than rejecting that event timestamp.
`SetDispatchMode.STATUS_IDLE` returns control to the dispatcher but does **not**
select a mux velocity source. The driver explicitly requests
`nav_mpc_path_track` on `/twist_mux_new/in/switch/nav_source` and waits for a new
acknowledgement from `/twist_mux_new/out/nav_source_used`; an old latched value
does not count. Only then does it submit `MoveToTarget`. After acceptance, it
requires new nonlatched `map`-frame tracking-path messages whose endpoint matches
the requested goal (within 0.1 m, stable across at least three samples over
0.2 s). This prevents resuming a different old path while new planning is pending.
It then resumes the tracker, rechecks source ownership, and releases the software stop.
Any unresolved step retains/reasserts the stop and reports its stage; an uncertain
goal delivery remains blocked until reconciled. If the chassis still reports
`nav_mpc_move_base` five seconds after accepting the goal, the driver stops and
cancels the task.
`navigate_to_pose` and `navigate_to_tag` accept a 0.05–0.5 m/s linear speed
limit (default 0.15 m/s). The factory planner and obstacle handling determine
the actual speed.
`base_drive` and `base_servo` restore this switch before publishing. Mapping
keeps the chassis in MANUAL with upper MPC velocity disabled; canvas keyboard
commands use the guarded factory RobotMotion route instead.
Stops retain the software stop and require native dispatch cancellation, a
confirmed LQR pause/zero output, and fresh stationary odometry. The factory's
dispatch cancellation can leave a direct LQR point task active; its path DELETE
also rejects the currently executing edge. The driver therefore holds that task
paused until a new goal is accepted, rather than claiming the path was deleted.
Factory historical task queries can retain status 1 after
a goal has been replaced; that history alone does not prove it is still active.

In an earlier commissioned-robot validation, a supervised `navigate_to_pose` test with a 0.2 m
forward target and 0.1 m/s requested speed kept `nav_mpc_path_track` selected.
Map feedback recorded approximately 0.150 m displacement and 0.050 m remaining
error; native status changed from 1 to 2 and the driver completed after its
stationary check. The factory request uses a 0.1 m position threshold. The test
ended with confirmed whole-body disable and chassis software stop. This checks
the short navigation/control-ownership path, not long-route or obstacle trials.

On 2026-11-25, a new 0.2 m trial did not move meaningfully: the installed
`mcrp_global_planner/MCRPlanner` crashed in
`PathOptimizer::objectiveFunction()` during global planning. The driver held the
software stop when it could not confirm a replacement path. Under operator
authorization, changing only the planner plugin and its startup-config reference
to a private copy using the factory-listed, installed `navfn/NavfnROS` produced
a seven-pose map-frame plan in approximately 85 ms, with the requested endpoint.
The original factory YAML, costmaps, footprint, obstacle settings and speed
parameters were unchanged. This was a planning-only check, not a successful
motion test. A subsequent power cycle restored the factory MCR selection; the
private config and rollback record remain under
`~/.motus/navfn-validation-20261125/` on the chassis. No persistent planner
override has been installed, and motion validation of the fallback is pending.

On 2026-11-27 the MCR optimizer failed again during the reported P1 request
(02:22:20); the LQR process respawned and the driver could not confirm the new
path. The driver now detects a changed LQR URI/PID during path confirmation and
reports the specific failure. `load_map` also reports an error completion when
localization remains unconfirmed, retaining its detailed result in
`info.last_map_localization`. Relocalization uses the chassis-local software
hold and fresh odometry; `/enable_control_state` belongs to the other ROS master.
Its TF listener waits for a scan covered by its actual timestamp history.

A blocked native submission must not delay the physical stop request. Stop now
requests an independent chassis hold before waiting for submission ordering.
A chassis-local stop generation prevents an older navigation/preparation helper
from subsequently releasing that hold. Cancellation and stationary confirmation
are still required before clearing the unresolved-task record; a zero command
on the upper-MPC route cannot cancel a native tracker on another mux input.

The current vendor binaries still need investigation: a held Navfn `GetPlan`
diagnostic blocked in `GlobalPlannerService::makePlanServiceCallback()` calling
`makePlan()` on the same mutex; tracker pause/cancel replies also did not always
produce the requested runtime state. Small navigation trials did not pass the
complete acceptance/arrival/stop checks. These changes must not be described as
a verified end-to-end navigation fix, and no automatic planner replacement or
unconditional stop release is installed by the driver.

`stop_nav` judges completion by a confirmed chassis output hold, fresh stationary
odometry and a stop generation that prevents older driver requests releasing the
hold. A missing factory `CAR_PAUSED` acknowledgement alone no longer leaves the
driver task pending. Cancellation/pause are attempted, and their outcomes are
retained in `info.last_navigation_stop`; successful physical stop clears the
pending task in persistent storage. The output hold remains asserted after an
explicit stop. A subsequent control action must prepare its own route; clearing
metadata does not claim the factory tracker queue is empty or resume motion.

### 2026-11-20 camera/navi geometry verification

Compared the shared camera/control contracts with Bumi, Tianyi 2.0, R1 and the
ActuCore navi consumer. Added camera-owned declarations and live factory costmap
footprint provenance; no new Python/APT dependencies. The head SDK launch now
requests software depth registration and frame sync. The installed Orbbec SDK
supports SW D2C on Gemini 335L; runtime verification confirmed both CameraInfo
streams use 640×480, `camera_head_color_optical_frame`, and identical K
(fx=367.468933, fy=367.310486, cx=320.404572, cy=245.939484). The depth stream uses
the RGB lens identity only after these checks; native/mismatched frames are
rejected. Depth is uint16 millimetres with 0 invalid, compressed losslessly.

A no-motion live integration test used head RGB → VOP, RGB-aligned hardware
depth, and body-frame odometry. ActuCore correctly bound
`/ubuntu/state/camera_depth` by its explicit `image/depth-zlib` declaration;
this also requires the companion ActuCore role-binding fix, not only a driver
update. The site ran an older ActuCore image without the newer declaration
reader, so its local validation image also backported that reader and start
argument forwarding. A future normal release should build from the updated
ActuCore source rather than depend on the local validation image.

That initial integration used a patched ActuCore validation image. For the
older unmodified release, the driver now declares the hardware depth card's
primary output as `/ubuntu/state/camera_head/visual_depth` (namespace-dependent),
which its existing suffix-based binder recognises even before DDS discovery.
The card remains named `camera_depth`; the wire format stays `image/depth-zlib`
and the source remains measured head-camera depth, not a visual-depth model.
`/ubuntu/state/camera_depth` remains a backwards-compatible alias publishing
the identical bytes and source header while the card is running. CameraInfo
describes both endpoints with the same calibrated lens identity. Existing
canvas links should resolve the current port-0 topic from `info()` on the next
project start; a stale direct topic override must be refreshed. This producer
compatibility path does not require changing ActuCore or adding a visual_depth
inference card. Older navi can still omit its optional RGB overlay if its
separate graph lookup runs before discovery; depth role binding is independent
of that graph lookup.

This compatibility path was verified against the unmodified site ActuCore
`release.261113.a332240-jetson-jp5.11`: navi started with the new depth endpoint,
the objects endpoint and `/ubuntu/state/odom`, and decoded measured depths for
visible detections. An eight-second independent DDS observation received 78
depth images, 80 odometry messages and 64 detection messages; the latest depth
had 94.2% valid pixels. Its optional RGB preview was not bound because that
version still relies on an immediate graph lookup for RGB. This does not block
the objects/depth navigation inputs, but the preview limitation remains.
No hardware control sink or navigation target was enabled during validation;
temporary navi/VOP instances were stopped afterwards. The change is in the
driver only; the ActuCore image and source were not altered for this test.

VOP forwarded 640×480, K and the head lens ID; navi reported
`objects_frame=[640,480]` and normalised all eight boxes in one sampled frame.
An independent eight-second DDS read received 80 depth, 81 odom and 33 detection
messages; the latest depth had 83.9% valid pixels. Navi adopted horizontal
half-FOV 0.716 rad from K and the factory ±0.35 m estimated envelope. Remaining
notes concerned that estimate, negotiated speed caps and adoption of real
camera geometry, rather than missing inputs. These observations establish data
compatibility, not navigation clearance or physical arrival performance.

The project was stopped before the update, whole-body control remained software
paused, `base_servo` had no input subscription and applied zero commands. No
navigation target was sent. Temporary navi/VOP instances were stopped after the
test; the project remains stopped for the operator to reconnect and start.
The head must face forward and stay stationary during this navigation mode;
head-to-base motion compensation and a measured full-body envelope remain
outside this verification.

### PR #519 camera, depth, IMU and odometry compatibility

The Kuavo producer follows the numeric layouts reviewed in driver PR #519
(head `090231d1477af7a60a4398d58e21d99d867694b4`). These are additional
computation interfaces; the body IMU display and navigation velocity window
keep their existing semantics.

| Output | Format and parameters |
| --- | --- |
| `camera_head` | `image/jpeg`, 640×480; camera-owned `motus.camera/1` containing effective K, zero D, optical frame, lens ID and source-clock provenance |
| `camera_depth` | `image/depth-zlib`, lossless little-endian uint16 millimetres, 640×480, zero invalid; the same rectified optical rays, K and lens ID as head RGB |
| `imu.camera_imu` | Named `sensor/imu` output on `/ubuntu/state/camera_imu`; `std_msgs/msg/String` JSON with `schema=motus.imu/1` |
| `loco_state` odom output | `state/odom`, `motus.odom/1`; body-frame velocity plus source `/odom` pose, xyzw quaternion, frames and covariance |

One shared host RGB-D relay owns both public head image streams. It applies
the measured RGB distortion and registered depth calibration to common
rectified rays, rather than merely changing the metadata. Images and their
CameraInfo retain each original source header. Pairing permits at most 50 ms
capture skew. Its bounded 16-frame source buffers tolerate software registration
transport delay without widening that capture-time bound; only the latest
completed pair waits for encoding. Independent transport/consumer drops can
still remove one member of a published pair. Left/right chassis cameras retain
their own calibration and lens IDs and are not paired with head depth.

The camera IMU port preserves integer `stamp_ns` and `accel_stamp_ns`, raw
`angular_velocity` xyz in rad/s and `linear_acceleration` xyz in m/s². It carries
SDK-internal `camera_from_gyro`/`camera_from_accel` rotations and the actual
sensor origins as `camera_from_gyro_translation`/`camera_from_accel_translation`
in metres. Raw vectors are not pre-rotated. The helper associates gyro/accel
within 20 ms, drains bounded batches, and never substitutes body IMU values or
invented zero lever arms. The existing first `data/json` output remains the
independent body IMU display. Connect the named `camera_imu` port when using
RGB-D IMU assistance, not that display output.

Odometry includes `pose.stamp_ms`, `frame_id`, `child_frame_id`, position,
quaternion and covariance. `vendor.motus_slam.stamp_ns` preserves exact source
nanoseconds independently of the averaged navigation twist and its receipt-time
fallback. A missing/invalid source pose is reported unavailable; it is not
replaced by map localization or a fabricated pose. Wheel odometry drift remains
unbounded.

The host helpers and launch configuration are shipped in the image/bootstrap
bundle and installed through the existing managed provisioning path. No new
Python/APT dependency is required; rectification uses host OpenCV/NumPy and
camera-internal transforms use the installed ROS1 SDK TF. The obsolete separate
depth relay is stopped during upgrade.

The new `state/extrinsic` output supplies capture-associated model estimates of
`body_from_camera`. The host relay also joins `base_link` to the SDK depth root
using runtime URDF kinematics and measured joints. These interfaces do **not**
supply an installation-calibrated mount or verified cross-board clock synchronization. The articulated head must remain
forward and stationary for the current navi path. SDK camera-internal IMU
geometry is distinct from a measured camera-to-robot mounting transform. The
installed Perception version does not include the PR371 RGB-D fusion backend;
numeric producer compatibility and navi input tests do not establish full
fusion, SLAM accuracy or navigation performance.

2026-11-20 follow-up verification: 504 driver tests passed (one local OpenCV
test skipped); all four RGB-D geometry/pairing tests passed on the actual host
with OpenCV. The companion ActuCore navi suite passed 290 tests with 21 skips.
A final eight-second DDS observation with the sensor cards explicitly started
received 70 RGB images, 70 depth images, 989 camera IMU packets and 71 odometry
packets. The latest depth had 89.3% nonzero pixels; the observed IMU gyro/accel
skew was at most 0.0771 ms with no duplicate IMU timestamps. These are observed
sample counts, not guaranteed output rates or calibrated timing accuracy.
Independent readers can lose image pairs: the 50 ms bound belongs to the
source relay's pairing, not nearest timestamps after separate DDS losses.

The installed SDK also exhibited native IMU silence after camera service
restarts. Restarting only the relay did not recover it; invoking the factory
camera-only `reboot_device` service restored the native streams. This remains
a restart-reliability limitation; no automatic camera reboot policy was added
and the short successful data check does not establish reboot endurance.
The project remains stopped, whole-body control software paused and base_servo
disconnected with zero applied commands. Temporary navi/VOP instances were
stopped; the final image is a local validation build, not a published release.

2026-11-23 stationary follow-up: the new model-mount candidate produced 160/160
available capture-associated matrices, with successful body-to-optical TF lookup.
The candidate was reverted after validation; this is not a full live container
upgrade. Native IMU silence was reproduced even in an independent SDK reader as
root, then recovered by a camera-only device soft reboot. See the spatial input
audit for timing, geometry limitations and deployment status.

键盘SSH确认等待与运动租约分开：运行中允许最多1秒接收回复，底盘本地300ms失联零速保护不变，不重发旧方向。帧按80ms起点间隔发送，通信耗时计入周期，不在回复后额外等待80ms；`info.keyboard.last_roundtrip_ms/max_roundtrip_ms` 显示当前会话通信耗时，供定位抖动。该修改需部署后再进行现场持续操作验证。

`stop_mapping` 在 MANUAL 中用厂商 RobotMotion 零速和新鲜里程计静止完成保存预检，不要求调度软急停反馈；退出建图后用 AUTO 调度停车，不自动载入地图或按建图起点重定位。地图命令尚未发送时的预检失败会保留原建图会话并清除本次过渡记录，允许重试，不要求重启底盘。

保存与载图分离：`stop_mapping` 返回 `map_loaded:false`，按保存文件的实际指纹绑定语义点并触发上位机同步；`spatial_map` 列表与底盘/本地地图并集一致，选择地图时校验同步预览副本，不改变底盘当前地图、模式或定位。文件同步失败明确报告 `map_preview_sync_failed`，不再将尚未同步的底盘地图误报为不存在。

上位机底盘控制的模式切换：`base_drive/base_servo` 需要 AUTO；从已退出建图的 MANUAL 接管时，先确认无建图/键盘占用和新鲜静止，发送厂商 MANUAL 零速、关闭上位机速度输出，确认原生跟踪器已结束及上位机持续零输出，再切 AUTO 并等待实际反馈，之后才取消旧任务、核对速度源及解除调度停车并发送目标。`controlled_spatial` 建图键盘用 MANUAL，原生导航用 AUTO；卡片生命周期 start 不等于申请建图控制权。模式转换与运动共用停止世代和控制锁，停止可抢占，转换未确认不发目标。

执行模式核对：所有全身运动卡（base_drive、base_servo、arm、leg、head、原生导航）共同使用上位机 BaseArm=3，不把底盘的 MANUAL/AUTO 当成同一个模式枚举。每次新的底盘执行都读取实际模式；缓存的上位机通路若遇到 MANUAL 或不同 mux 选择会重新交接，避免仅凭旧缓存发目标。原生导航在自动准备前允许“已定位但仍MANUAL/调度停车”，准备后再次核对AUTO再发目标；硬件急停、碰撞、驱动故障、未初始化定位和当前图不匹配仍拒绝。模式改变不会替代载图或重定位。

`info.control_ownership.execution_modes` 提供各卡/地图动作的只读模式要求。arm/leg/head 的执行只准备 BaseArm，不改变底盘模式或速度源；tag_place/get_pose/list_maps/delete_map、spatial_map预览、speaker及传感器不为了读取或显示而切运动模式。已有任务/建图占用时不能用另一张卡的执行隐式抢权，须先结束占用；停止请求不会为了“满足模式”而恢复使能。

键盘故障停车分两路：正常松键保留会话并发 RobotMotion 零速；释放/断连/看门狗退出先用同一厂商连接发零速并确认新鲜里程计静止。如果 WebSocket 已坏、零速写失败或静止不能确认，底盘本地守护改走独立 ROS SetDispatchMode(control_mode=0)，要求软停反馈为真、里程计静止、再核对软停仍为真，才返回已停车。故障兜底不解除软停、不切模式、不发非零目标。两条路径均未确认时返回 keyboard_fallback_stop_unconfirmed 并保留持久化阻断；不能以idle或进程已退出证明真机停住。MANUAL下厂商ROS软停曾有不确认记录，此兜底是否有效仍需真机故障场景验证，单元测试不替代该验证。

已结束的MANUAL准备失败恢复：显式start的共享预检可处理仅有`chassis_preparation.delivery_uncertain:false`的残留记录。要求全身及上位机速度输出均已关闭、无活动任务/地图操作/键盘或未确认写入，随后请求厂商零速（失败时走独立ROS停车兜底）、确认原生跟踪器FINISH_STOP及新鲜里程计静止，再清理该准备记录。此过程不切AUTO、不使能、不解除停车，AUTO交接仍留到实际运动执行；恢复证据记录在`info.last_preparation_recovery`。info本身保持只读，不会为恢复而发停车或运动请求。

上位机速度关闭时，实测 `/move_base/base_cmd_vel` 只保留一帧零速，不连续发布。仅对非G12的AUTO/上位机交接，检查同时订阅经过唯一发布者、类型和MD5核验的 `/enable_vel_control_state`：关闭反馈为false、最终速度为零、两条连接持续有效且观察窗口满300ms时，可确认关闭输出通路的静止；任何非零、使能回调或连接失败立即撤销。没有关闭反馈时仍要求至少三帧持续零速；G12仍要求持续新鲜零速和中立遥控输入。MANUAL→AUTO的只读预检前，已确认停车与关闭输出会记为`delivery_uncertain:false, stage:observe_manual_auto_guard`，只读检查失败不再伪装成模式命令投递未确认。

准备记录按实际动作阶段持久化：初始状态读取和MANUAL静止检查均为delivery_uncertain:false；只有即将发送AUTO切换或原生prepare/upper_handoff时才标为true，并记录operation/stage；已收回的失败响应保留厂商stage/error_code及确定的投递状态。info.last_preparation_failure保留最初失败及当时记录，避免后续“需要先stop”覆盖根因。只读预检失败不会制造新的运动命令投递不确定性；真实未确认的模式/解除命令仍保留阻断。

底盘键盘接收端按SSH连接持有JSON行缓冲，分片跨解析读取超时不会丢弃，合并到达的后续帧也保留；改为有界按块读取，单帧仍限制1024字节。解析缓冲不延长运动授权：守护的300ms租约一旦超时仍请求停车并退出，不重用迟到方向帧，不在旧会话中自动恢复运动。

### 上位机速度到原生底盘的跨板连接

`base_drive` 与main分支保持同一路径：`/cmd_vel`→厂商MPC/WBC→上位机`/move_base/base_cmd_vel`，不绕过MPC直接把卡片目标发给电机。现场发现控制器192.168.26.1与底盘192.168.26.22各自的ROS Master之间没有厂商wheel_bridge，导致接管和命令持续时间都能确认，但底盘收不到速度。驱动现在在首次上位机底盘动作前，经既有SSH通道创建底盘临时跨板连接；每个Master分别初始化真实ROS节点，转发MPC输出并反向转发厂商BaseCmdVelStatus，使用板子已有Noetic和生成消息，不安装依赖、不启动厂商控制进程、不修改原厂源码或launch。

非零转发需要当前base任务为base_drive/base_servo、无控制转换、相同stop世代和连接、已验证upper mux、全身/速度使能为真、底盘AUTO且无建图/急停/软停/故障、速度输入不超过200ms且底盘反馈不超过300ms。速度仍受0.8m/s与1rad/s上限约束。驱动每40ms发送带随机nonce与递增seq的许可，底盘本地300ms超时发零速并确认静止；零速无法确认时请求独立ROS软停。观察子进程随父进程退出，底盘文件锁阻止重复桥接。

切换为原生导航/建图或退出base任务时，撤销许可、发一次零速，然后停止向该底盘输入持续注入速度；原生导航仍选nav_mpc_path_track，键盘仍走MANUAL RobotMotion。交接前先关闭旧上位机速度输出再恢复全身，交接完成才为新任务开启速度。info.control_ownership.upper_velocity_bridge显示连接状态与非零转发帧数。定时move的completed仍是命令时长完成；实际位移须以底盘里程计确认。

2026-11-27已授权现场验证：0.1m/s前向目标持续1秒，底盘输入、mux输入/输出均记录非零且峰值约0.1m/s，里程计约0.13米（含MPC减速尾段）；随后全身暂停、底盘软停与新鲜里程计静止均确认。另已验证零许可连接和断连停车。此次未重复真机导航或建图运动，跨卡片隔离由离线回归覆盖；不将此短程测试视为全部导航验收。

### 执行卡与G12遥控的交接

底盘空闲且已使能时交给G12；base_drive/base_servo实际执行期间切到卡片来源，controlled_spatial导航、建图和地图转换期间阻断遥控底盘输入。卡片仅start不永久占用底盘。使用ROS subscriber的publisherUpdate接口临时选择MPC两处消费者的/cmd_vel和/cmd_vel_world来源：卡片只允许/rosbridge_websocket，遥控只允许原厂两个joy command节点，hold不接收这两条速度输入。核对旧TCPROS连接已退出后才确认交接。遥控节点和/joy、/stop_robot通路不被停止或重映射，原厂文件和launch不改。断开驱动时恢复Master中当前注册的原输入连接。

base_drive正常完成或base_servo释放数据租约后，先等经核验的G12输入连续回中300ms，再接回遥控来源并重新等待MPC零输出300ms，才允许底盘速度；杆未回中不会沿用旧方向。原生导航完成后，在现有全身使能仍为true、无活动任务/未确认操作时自动准备上位机速度通路，再执行同样的遥控交还。显式全身stop、故障、未知写入、建图未结束时不自动使能或解除阻断；之后若操作员明确重新使能控制器，则可在新停止世代内重新准备遥控，仍需回中。info.control_ownership.upper_velocity_bridge.remote_state显示blocked/waiting_for_neutral/ready，remote_return_error显示交还失败。

2026-11-27已在全身关闭状态下真机核对hold/card/remote三种MPC输入连接均生效且能恢复原连接；随后在旁站授权下恢复遥控，操作员确认能正常移动、回中停止。另执行1秒全零base_drive任务，期间remote_state=blocked、完成后无额外恢复调用即回到ready；测试期间非零转发计数不变。未执行额外原生导航或建图运动，相关隔离和导航完成交还由离线回归覆盖。
