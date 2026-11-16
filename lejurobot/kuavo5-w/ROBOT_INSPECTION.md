# 真机上位机只读检查记录

检查日期：2026-11-16；最初状态检查时间为机器人主机时区 `2026-11-16T14:16:05+08:00` 附近。
前文为分阶段历史快照，后续节点、参数和端口可能改变；当前结果以文末最近的追加记录为准。本文不记录登录密码。

## 主机及 ROS 拓扑

| 项目 | 实测 |
| --- | --- |
| SSH 上位机 | `192.168.123.214`，hostname `ubuntu` |
| 系统 | Ubuntu 20.04.6 LTS，aarch64，5.10.216-tegra |
| ROS 安装 | `/opt/ros/noetic`；未在 `/opt/ros` 发现 ROS2 |
| 上位机机器人内网接口 | `nuccon`：`192.168.26.12/24` |
| 上位机无线接口 | `wlan0`：`192.168.123.214/24` |
| 配置的 ROS master | `ROS_MASTER_URI=http://kuavo_master:11311`；hosts 映射到 `192.168.26.1` |
| 配置的 ROS_IP | `192.168.26.12` |
| 底盘/相机节点所在地址 | ROS lookupNode 返回 `192.168.26.22`，包括 `/leju_node`、`/ukf_odom`、`/camera_left`、`/camera_right` |

不要在上位机另起一个默认 localhost ROS master，否则会形成两个互不可见的 ROS 图。
远程 SSH 非交互 shell 不应依赖 `.bashrc` 自动提供 ROS 环境。

## 控制栈的实际状态

ROS master `192.168.26.1:11311` 可达，查询到288个 topic 类型记录。
topic 类型记录同时包含订阅关系，**不代表存在发布者或正在产生数据**。

只调用了读取状态的 `std_srvs/Trigger` 服务：

```text
/websocket_sdk_srv/get_robot_launch_status
success: True
message: "unlaunch"
```

这里 success 表示查询服务成功，不能解释为机器人控制器已启动。

| 接口 | 观察到的状态 |
| --- | --- |
| `/sensors_data_raw` | 类型为 kuavo_msgs/sensorsData；只有 `/joy_node` 订阅，没有发布者；等待单帧3秒超时 |
| `/odom` | `/ukf_odom` 发布，成功订阅单帧；header.frame_id=`odom`，child_frame_id=`base_footprint` |
| `/battery` | `/leju_node` 发布，类型为 leju_mobile_base_msgs/Battery；未确认数据结构/单位 |
| `/camera_1/color/image_raw`、`/camera_2/color/image_raw` | ROS 图中存在 sensor_msgs/Image 接口；未抓取图像内容或验证帧率 |
| `/cmd_vel` | 有 `/websocket_sdk_start_node` 发布者；所查图中没有该话题订阅者 |
| `/cmd_pose`、`/cmd_head_vel` | 有 `/joy_node` 发布者；不能据此认定 WBC/MPC 已接收 |
| `/enable_control_state`、`/mobile_manipulator/lb_mpc_control_mode` | 未见发布者 |
| `/enable_control`、`/mobile_manipulator_mpc_control` | 未在服务注册表中出现 |
| `/robot_type`、`/robot_version`、`/armRealDof`、`/headRealDof`、`/legRealDof`、`/waistRealDof` | ROS master 返回参数未设置 |
| `/use_shm_communication`、`/humanoid_description`、`/modelPath`、`/end_effector_type` | ROS master 返回参数未设置 |

这些结果与 SDK 的 `unlaunch` 一致：当前图中尚未出现驱动所需的完整轮臂控制接口。
尚不能确认真机的控制仓库提交、控制版本或关节/力矩实测值。

上位机 `.bashrc` 中存在 `ROBOT_VERSION=53`，但它是本地 shell 配置，不是运行中控制器发布的机型参数；
不能据此把这台机器判成双足，也不能据此自动选择5-W模型。

## 模型不能混用

`/humanoid_description` 不存在，但 `/robot_description` 存在。
读取后确认其 robot name=`jtrobots_nidec`，只有2个 link，没有活动关节。
它是当前底盘模型，不能作为 Kuavo 下肢/双臂/头部的完整骨架。
因此保留驱动对真实 `/humanoid_description` 和20关节 child-link 映射的要求。

## SDK、消息和桥接准备情况

- 上位机已经安装 `rosbridge_server`、`rosapi` ROS Noetic 包。
- 上位机 `127.0.0.1:9090` 与 ROS master 主机 `192.168.26.1:9090` 均连接拒绝；未发现可连接的 rosbridge。
- 当前上位机 python3 环境通过 pip show 未查询到 `kuavo-humanoid-sdk-ws` 或 `kuavo-humanoid-sdk` 分发包；未穷举其他虚拟环境。
- 应用仓库为 `/home/leju_kuavo/kuavo_ros_application`，HEAD=`7756742d`，提交日期2026-04-07，且已有现场修改；检查没有覆盖这些修改。
- source 应用工作空间后可以找到 `kuavo_msgs`、`dynamic_biped`，原始 jointData 的 q/v/vd/torque 四数组及 IMU 字段与适配器所用结构相符。
- 当前上位机环境未找到 `leju_mobile_base_msgs` 消息包，因此不能仅凭 `/battery` 的 topic 名称就新增电池适配。

在此拓扑下，rosbridge 可以部署在上位机，与 Agent Core 同机；它通过 ROS1 内网连接 master。
以下仅为准备命令示例，**此次未执行启动**：

```sh
source /opt/ros/noetic/setup.bash
source /home/leju_kuavo/kuavo_ros_application/devel/setup.bash
export ROS_MASTER_URI=http://192.168.26.1:11311
export ROS_IP=192.168.26.12
roslaunch rosbridge_server rosbridge_websocket.launch address:=127.0.0.1 port:=9090
```

这只能补齐远程 ROS 接入，不会补出尚未启动的轮臂 WBC/MPC 接口。
机器人控制栈须在现场确认安全状态、版本和启动方式后单独就绪；本次没有调用任何启动、停止、使能、站立或运动服务。

## Agent Core / DDS

- 正在运行 `phanthy-motus-agent-core-1`，镜像 `core:release.261111.2b00f1d`，使用 host 网络。
- 本机15678端口正在监听；未发现 Kuavo 驱动容器或15747端口监听。
- 容器环境为 ROS_DOMAIN_ID=42、RMW_IMPLEMENTATION=rmw_fastrtps_cpp，使用 `/opt/phanthy-motus/dds-local.xml`。
- DDS profile 中使用127.0.0.1接口白名单，禁用 builtin transports。
- 宿主机和 Agent Core 容器内该文件 SHA256 相同：`7f0c0ad4875dd67e897b8735ebca058ad733f1b5e385c8bb5c0ffcda913d1b7a`。

这些是配置一致性的检查，尚不是驱动→Agent Core 的实际 DDS 话题接收验证。
驱动仍应与 Agent Core 同机运行，通过 WebSocket 接 ROS1，而不是直接尝试跨 Wi-Fi 接收 loopback DDS。

## 对当前驱动的结论

1. `/odom` 的 base_footprint 与现有 frame 白名单相符，不必猜测或放宽坐标系。
2. rosbridge 默认 localhost URL 适合与 Agent Core 同机的部署，但目前9090尚未启动。
3. 当前控制参数和实测关节缺失时，驱动应继续拒绝运动，不能绕过保护或用默认零值替代。
4. 控制接口就绪后，才可核实实际自由度、URDF、关节数量、力矩单位和各动作的 ACP；本次没有运动验证。

后续本地实现补充：驱动现已将官方固定版本 400062 的参考骨架放入
`resource/kuavo5w_model.urdf`，通过 `model` resource 供前端离线显示。
这不改变本次现场观测：具体硬件模型及标定仍待核实；实时 `joints` 仍需真机模型名称校验和新鲜实测数据。
本地参考文件的存在不视为控制器就绪或运动验证通过。来源与许可证见 `resource/README.md`。

本次只进行了查询及短时只读订阅，没有部署驱动、重启服务或修改机器人配置。

## 追加：直接订阅 ROS1 实际数据

2026-11-16 14:19:36（+08:00）通过上位机原生 rospy 直接订阅，无需 rosbridge，也未启动全身控制栈。

| 来源 | 实测结果 |
| --- | --- |
| `/odom` | 约3秒收到179帧，接收频率约59.72Hz；frame=odom / child=base_footprint |
| `/battery` | percentage=96.5，temperature≈23.67，current≈-3.20，health=99，status=4；voltage 原始值为0，不能作为真实零电压判断 |
| `/jaten_behavior_manager/out/leju/battery` | percentage=96.5、temperature≈23.67、status=2；应保留来源，不能与 `/battery` 的 status=4 混为同一语义 |
| `/jaten_behavior_manager/out/leju/small_battery` | percentage=100、voltage≈44.83、temperature=24、health=100、status=4 |
| `/sensors_data_raw` | 3秒内0帧，仍无发布者 |

本次里程计末帧的 vx≈-0.000199m/s、vy≈-0.000241m/s、wz≈0.000887rad/s，接近静止；
这些数值仅是该时刻的测量，不是发出的运动指令。

自定义电池消息通过 `rospy.AnyMsg` 接收，并读取发布者 TCPROS connection header 中的
`type`、`md5sum`、`message_definition`，使用 `genpy.dynamic` 动态解码成功。
这补齐了先前“本机未安装消息包”时尚未取得的字段定义；缺少本机消息包不妨碍这种直接只读检查。

- `leju_mobile_base_msgs/Battery`：MD5=`194127313f3eab30becb94a79e4c5562`。
- `jaten_msgs/leju_battery`：MD5=`29a3d6075735cfbefd0218e8a3a12e0d`。
- 发布端定义说明 voltage 为 V、current 为 A、percentage 为0–100、temperature 为摄氏度；
  health 为0–100，time_remaining 声明为秒，但本次未核实其数值的实际意义。
- status 常量为 unknown=0、charging=1、discharging=2、not_charging=3、full=4；
  两个主电池来源的状态值不一致，后续适配必须保留来源和原始字段。

上述动态读取方式用于检查；不能据此假定普通 rosbridge 已具备这些本机缺失的消息类。
若新增通过 rosbridge 的电池卡，还需补齐相应 ROS1 消息类，或由 ROS1 只读适配器转换成标准消息。

2026-11-16 14:22:00（+08:00）进一步只读订阅相机原始/压缩话题及打开状态：

- 两路原始图像与压缩图像各自的3秒采样窗口均未收到帧。
- `/camera_left/open_state` 和 `/camera_right/open_state` 都收到 `false`。
- 因此当时相机报告关闭，不能把已注册的话题误报为已有图像流；检查未发布开关命令。

根据以上实测，驱动已将 `loco_state` 数据读取与全身机型探测解耦：
只要连接正常、里程计 frame/数据有效且新鲜即可输出；机型探测失败或切换 raw SHM 来源不再清空独立的底盘里程计。
运动指令仍要求正确机型、控制模式及使能状态；连接断开/代次变化仍清除全部旧测量。

## 追加：相机、音频与导航能力核查

2026-11-16 15:54–15:56（+08:00）再次只读查询 ROS 图，并短时订阅状态及图像帧计数。
未发送导航目标、取消、建图、定位初始化、相机开关、音频录制或播放命令。

| 能力 | 当时实测 | 当前驱动覆盖 |
| --- | --- | --- |
| 相机 | 两路 `/camera_{1,2}/color/image_raw/compressed` 为 `sensor_msgs/CompressedImage`，有对应发布节点；两路 `open_state=false`，3s 内均0帧 | 尚无相机卡；不能宣称可用图像流 |
| 麦克风 | ROS 图未发现 audio / micphone / speaker 相关话题 | 尚无 `mic` 卡；不能从声卡存在推断可采集 |
| 扬声器 | `/play_music`、`/stop_music`、`/speech_synthesis` 服务均不存在；ALSA 仅列出 Jetson APE/HDA | 尚无 `speaker` 卡；未验证实际音频设备及格式 |
| 地图 | `/map` 为 `nav_msgs/OccupancyGrid`，收到1092×1135、分辨率约0.05m、map frame 的地图 | 尚无地图卡 |
| 定位 | `/move_base/amcl_pose` 有 `geometry_msgs/PoseWithCovarianceStamped` 实测反馈，frame=map | 尚未适配空间位置流 |
| 导航 | `/move_base/status` 为 `GoalStatusArray`，收到1项 PENDING；`/move_base_simple/goal` 有 `/lqr_path_track` 订阅 | 尚无 `controlled_spatial`，未发送目标验证 |

真机导航服务类型：

- `/move_base/base_move` → `leju_mobile_base_msgs/BaseMove`
- `/move_base/move_to_target` → `leju_mobile_base_msgs/MoveToTarget`
- `/move_base/dispatch_move` → `leju_mobile_base_msgs/DispatchMove`
- `/move_base/check_arrived` → `leju_mobile_base_msgs/CheckArrived`
- `/get_jmap_lists` → `jaten_msgs/MapLists`
- `/jmap_operate` → `jaten_msgs/MapOpt`
- `/save_map` → `cartographer_ros_msgs/SaveMap`
- `/start_localization` → `leju_mobile_base_msgs/StartLocalization`

SDK 示例中的 `/load_map`、`/get_all_maps`、`/get_current_map`、`/navigate_to_task_point`
服务均不在该次服务注册表中；`/move_base/cancel` 也不在话题类型表中。
这说明底盘确有导航相关栈，但不能直接把 SDK 示例等同于该机接口。
`/navigation_control` 当时仅见 `/joy_node` 订阅，没有发布者；本次未据名称推测其暂停语义。

后续 `controlled_spatial` 应沿用其他驱动的 `navigate_to_pose`、`navigate_to_tag`、
`list_maps`、`load_map` 等实际可支持动作，并与 `loco` 共用 `base` 资源、接入 ACP。
在开放导航动作前，必须取得当前底盘自定义服务的请求/响应定义及取消协议，核实目标ID关联、
失败状态、停止确认和控制仲裁；不能直接使用 SDK 的“status_list 首项”判定本次导航完成，
也不能只发零速便假定导航规划器不会继续发送速度。建图和地图管理只声明已经适配的能力。

当前配置仍只注册 `loco/arm/leg/waist/head/switch_mode/state/joints/imu/loco_state/model`，
本次核查没有添加相机、`mic`、`speaker` 或 `controlled_spatial` 的占位卡。

## 追加：现场授权开启外设与启动集成（2026-11-16）

以下结果更新上文的相机关闭状态；未启动全身控制器或发送运动指令。

- 两个 `set_camera_enabled` 服务实际为 `std_srvs/SetBool`，调用 true 均返回成功。开启后左右路压缩图像
  在约3秒内各收到53帧，最新帧约108/97KB，JPEG，源 frame_id 为空，不臆造光学坐标系。
- `/scan` 在3秒内收到45帧，每帧3701测距，最新约3226个有限值，范围0.05–50m，frame=laser。
- 独立 `/imu` 在3秒内约310帧，frame=imu_link；驱动现优先使用该流，不依赖缺失的全身控制器参数。
- `/battery` 为 `leju_mobile_base_msgs/Battery`，MD5 `194127313f3eab30becb94a79e4c5562`；
  实测 SOC96.5%、电流−3.2A、温度约24.67°C、voltage=0、status=4。
  `/jaten_behavior_manager/out/leju/battery` 的 SOC相同但 status=2，保留两者原值。
  small_battery 约44.99V/100%，不能挪用作主电池电压。
  两个 jaten 流的类型为 `jaten_msgs/leju_battery`，MD5 `29a3d6075735cfbefd0218e8a3a12e0d`。
- 本地缺少上述厂商消息 Python 类，新增普通用户只读 relay，校验 type/MD5 后动态解码，转标准 String。
- 音频仅见 Jetson APE/HDA；PulseAudio 唯一 source 是播放 monitor，不是麦克风。
  蓝牙设备列表为空，脚本寻找的 EB03_LJ 未连接。hw:0,0 的1秒录制4秒后超时；100ms数字静音播放返回0，
  只能证明 PCM 可写，不能证明真实扬声器出声。因此暂不添加 mic/speaker 卡。
- 操作人员提供标识 `KUA5W0710701302138`。没有找到它与控制器版本的映射；
  `/humanoid_description`、全身关节流仍缺失，`/robot_description` 只有底盘模型。
  仓库400062参考模型不能宣称已验证适用于此台实机。

已实现 `battery/laser_scan/camera_rgb/camera_rgb_right`，开机自动启动传感输出并按需开启相机。
安装固定 loopback 助手后，驱动启动可以确保 rosbridge 和电池 relay 运行；不会启动/使能 WBC/MPC。
host helper 的首次安装仍需现场管理员执行一次，详见 README。

### 集成验收结果

- 上位机已安装助手，9090 和15748均只监听127.0.0.1。主动停止 bridge/relay 后，运行中的驱动约1秒内
  自动重新请求启动两者；随后 ROS1连接及传感输出恢复。未触碰全身运动控制器。
- 候选镜像 `kuavo5w:local-peripherals-261116` 已在现场运行，Compose 通过同目录
  `local-peripherals.yml` 显式覆盖镜像，不冒用已发布 release 标签。该本地标签只存在于现场 Docker。
- Agent Core 容器内以 domain42 订阅8秒，收到 IMU80、odom81、电池41、雷达40、雷达点集40帧，
  左/右相机34/36帧。有效雷达输出3701测距，俯视点集约1577点；电池当时约94.83%。
- 最终候选版本的两路 DDS JPEG 均可用 OpenCV 解码为1024×1280×3。源图像时间戳为0、frame_id为空，
  原样保留，不能据此宣称相机时间同步或 TF 已核验。
- `joints.info` 返回空实测数组及 `controller_not_ready`，并明确参考模型未核验、现场配置标识为
  `KUA5W0710701302138`；这不是硬件读取的型号证明。
- 候选覆盖镜像736972192字节，所用已发布 release 736836914字节，增量135278字节。
  这是复用已有依赖的代码覆盖层对比，不是生产 Dockerfile 对 ros-base 的完整体积增量；后者仍待构建验证。
- 本地173项离线测试、Python编译及 service.yml检查通过。实际运动、控制停止、完整关节模型、
  mic物理采集及speaker实际出声仍未验收。


## 追加：传感卡输出整理与头部设备（2026-11-16）

本节更新前文旧卡名和设备状态。仅操作传感服务、传感卡生命周期及驱动测试容器，未启动/使能 WBC、MPC 或发运动目标。

- `joints` 现在声明骨架姿态、原始 `joint_state` 两路。原始数组读取不依赖模型校验，但不能用于绕过动作校验。
  `/sensors_data_raw` 仍无全身反馈发布者，运行时全身参数/URDF仍缺失，因此两路明确 invalid；不使用底盘轮速冒充20关节。
  `KUA5W0710701302138` 仍只是操作人员提供的机身标识，400062参考模型未完成硬件几何核验。
- `loco_state` 两路分别为扁平位置/姿态/速度展示，以及 actucore navi 的 `motus.odom/1`。
  真实 `/move_base/amcl_pose` 提供 map 定位，采样约 x=4.008m、y=−2.205m、yaw=−88.41°；
  odom 保持 body-frame 测量规范，不将 map 位姿混入标准里程计。
- 已移除公开 `state` 卡，内部安全检查保留；诊断改从 `switch_mode.get_current_mode` 查询。
- 电池按 main_/chassis_/small_ 展开为标量字段；不同状态码保留，主电池0V仍输出 null。
- `laser_scan` 首路为与其他驱动一致的原生 UInt8MultiArray `sensor/pointcloud`，次路保留测距 JSON。
  全部有效测距投影到 laser 平面的 z=0 点，不再限制为约一半点数；这不是额外3D雷达数据。
- 左右卡改名 `camera_left` / `camera_right`，增加 `camera_head`。三路使用独立 WebSocket 接收和 latest-only 发布线程，
  DDS BEST_EFFORT/depth1，发布时不持全局外设锁；底盘JPEG不重编码。
- 头部 USB 设备实际为 Orbbec Gemini335L，存在 `/dev/video0..7` 和已经构建的 OrbbecSDK_ROS1。
  以 color-only 640×480@15 启动；关闭深度、深度激光、TF发布，不改全身控制器。
  因现场缺 compressed_image_transport，使用已有 OpenCV/numpy 的普通用户 relay 以10Hz编码 JPEG，保留真实 header。
- 三路 DDS JPEG 均在 Agent Core 容器接收，经宿主机 OpenCV 解码成功：左右1024×1280×3，头部480×640×3。
  底盘源时间戳仍是0、frame_id为空；头部为 camera_head_color_optical_frame，有系统采集时间。
- 修正相机性能瓶颈：现场 websocket-client 纯 Python UTF-8 校验450KB约194ms/帧，DDS发布330KB约0.084ms/帧。
  使用 CPython strict UTF-8 解码替代逐字节校验，非法编码及 JSON 仍拒绝；驱动 CPU 从约101%降至约11–16%，
  驱动接收至DDS发布 info 样本约0.4–0.6ms。该数字不等于浏览器端到端时延；剩余桥接批量延迟的最终修复见下方。
- 直接 ROS1 10秒对照：odom589帧、IMU1000帧、左/右JPEG179/180帧、头部原始150帧、头部JPEG100帧。
  头部原始/压缩源平均采集年龄约5/38ms。数据源在工作，不可将桥接后丢帧归咎于硬件无图像。
- 第二雷达：已安装 Livox SDK，MID360配置指向192.168.1.191、host192.168.1.102；lidarcon administratively UP，
  但物理 carrier=0、NO-CARRIER，ping不通，未发现活跃 Livox/scan_2 发布者。其他滤波/Cartographer点云为同一2D雷达派生，不能当作第二台雷达。
- 音频更正：本机 audio/README_audio_setup.md 指的是 **USB EB03_LJ / ShengZhi_EB03_LJ**，不是蓝牙音箱。
  当前上位机 ALSA 仅Jetson APE/HDA，Pulse唯一source是播放monitor，没有枚举到USB EB03采集设备。
  对下位机使用现有凭据的一次SSH检查被拒，不能据此判断整台机器人绝对没有音频硬件。
  操作人员暂不能确认音频和第二雷达接线/供电；因此不添加虚假可用的 mic/speaker 卡。

现场使用本地覆盖镜像 `kuavo5w:local-sensor-ui-261116`，没有发布或冒用原release标签。
头部启动与JPEG relay已纳入一次安装后由驱动启动助手确保运行的固定服务；后续按卡启动行为见下节。

### 最终低延迟桥接与 DDS 验收

- 独立客户端对照确认旧bridge的TCPROS接收会聚批：10秒126帧、最大接收间隔约0.568s、平均采集年龄436ms。
  进程内为rospy订阅设置tcp_nodelay=True、buff_size=2,000,000后，10秒585帧，最大间隔约0.066s，
  无超过100ms的间隔，平均采集年龄2.9ms；不设置queue_size丢弃输入测量，不修改系统ROS包。
  已纳入host/rosbridge_loopback.py，重启持续有效。
- 最终连续30秒Agent Core DDS实测：左右相机各300帧、头部299帧；IMU/odom/loco_state各300帧且全有效；
  电池和二进制点云各150帧。点云末帧3158个有效点，三路JPEG再次成功解码。
- 头部原始采集时间至Agent DDS回调平均90.3ms、最小16.4ms、最大159.1ms；未测浏览器绘制/网络传输延迟。
  底盘源stamp为0，不能伪造左右端到端时延。标准odom末帧窗口6个样本、stamp_skew_ms=5，未再见空窗口。
- 全身joints/joint_state各300个明确invalid输出，原因仍是机器人控制器参数/全身反馈源缺失；没有补造姿态。
- 190项离线测试通过，含双流、模型边界、独立图像线程/最新帧覆盖、点云二进制、扁平电池、
  严格UTF-8解码、TCPROS订阅设置及头部JPEG行填充/格式检查；Python编译与service.yml检查通过。

- 最终本地候选镜像为eb5f51302b7b，736988544字节；对比已发布release的736836914字节，增加151630字节。
  本次复用现场release依赖；仍不代表生产Dockerfile相对ros-base的完整增量。
  重建并重新创建容器后再次核对15张卡、三组双流声明及三路相机info均正确；头部camera/JPEG relay、runtime manager、rosbridge服务均active。


## 追加：头部深度与卡片按需恢复（2026-11-16）

本节更新上节color-only配置。经授权开启头部深度及深度激光，仅操作传感服务，未启动WBC/MPC、使能电机或发运动指令。

- Orbbec Gemini335L以RGB+深度640×480@15Hz运行，深度未对齐RGB，关闭TF及厂商点云发布。
  原生深度为16UC1、step1280、frame=`camera_head_depth_optical_frame`，单位毫米；单帧269528个非零像素，有效距离1390–5544mm。
- 增加 `camera_depth`，宿主机stdlib relay以latest-only 10Hz无损zlib压缩毫米uint16数据；DDS CompressedImage.format=`image/depth-zlib`，
  符合当前Agent Core深度渲染契约。每帧校验解压后614400字节，保留真实采集时间；未测试浏览器最终渲染与端到端时延。
- Agent Core容器domain42连续30秒：深度300帧、左右RGB各300帧、头部RGB299帧，全部格式有效。
  深度末帧267192个非零像素、最小1387mm、中位2894mm、最大11695mm；采集到DDS回调最大257.5ms，RGB头部最大200.52ms。
  测得最大距离随场景和深度噪声变化，不代表标称量程或精度。
  IMU301帧、odom/loco_state各300帧、电池/点云各150帧均有效，点云末帧3201点。
- 卡片start若有新鲜图像不再请求开启；缺帧时左右相机调用原有SetBool，头部RGB/深度分别请求固定 `/ensure/head`、`/ensure/depth`。
  请求异步执行，运行中缺帧至少间隔5秒重试；后台基础检查只确保rosbridge和电池relay，不启动闲置头部卡。
  共用RGBD进程，stop不关闭其他卡依赖的物理流。
- 真机恢复回归：停止深度卡和深度relay，等待6秒，relay仍inactive；重新start深度卡后relay恢复active且深度feedback.valid=true。
- 第二雷达复查：lidarcon carrier仍为0，ROS无活跃Livox或scan_2发布者。已有MID360配置目标192.168.1.191、主机192.168.1.102，
  仍需现场确认接线/供电和实际硬件；没有把已有二维派生云或深度图冒称三维雷达。
- 用户反馈点云不显示时，DDS本身有有效点；Agent日志出现同名topic旧String与新UInt8MultiArray订阅类型冲突。
  重启Agent一次后日志确认正确pointcloud订阅建立。尚未直接观察浏览器最终渲染；升级改变topic类型时须清理旧订阅状态。
- 本地候选镜像 `kuavo5w:local-depth-261116`（9f530d375e0e），736995734字节，相对既有release736836914字节增加158820字节，
  本轮未安装依赖；该代码覆盖层差值不代表生产Dockerfile对ros-base的完整增量。
- 最终199项离线测试、Python编译、部署checker、diff检查通过；重建后四路图像、雷达、loco_state卡均running且valid，宿主机helper/头部相机/深度relay均active。
