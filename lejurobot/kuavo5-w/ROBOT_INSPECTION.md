# 真机上位机只读检查记录

检查日期：2026-11-16；末次状态检查时间为机器人主机时区 `2026-11-16T14:16:05+08:00` 附近。
这是当时的运行快照，后续节点、参数和端口可能改变。本文不记录登录密码。

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
