# Kuavo 5-W SDK / 源码核查与适配决策

核查日期：2026-11-16。这里区分“已核实的公开源码”和“机器人现场实际版本”；现场只读检查另见 `ROBOT_INSPECTION.md`，尚未确认运行中全身控制器对应的源码版本。

## 1. 源码在哪里

官方 WebSocket SDK 自身的 `setup.py` 将源码仓库指向：

- [Gitee：leju-robot/kuavo-ros-opensource](https://gitee.com/leju-robot/kuavo-ros-opensource/)
- [LejuRobotics GitHub 仓库](https://github.com/LejuRobotics/kuavo-ros-opensource)

SDK 在机器人控制仓库内，不必另找一个名为 SDK 的独立仓库。本次通过 GitHub 公开源核查，固定提交
`d9c036e19fce5689fcfa8c596ee89b2045bebb32`；**不假定这个提交等于真机或 Gitee 当前 master**。
下载 SDK、示例及文档的两百余个文本文件供检索，重点交叉阅读连接层、control/state/param、轮臂示例，
并补读消息定义、轮臂控制器、数据管理器和 5-W URDF。

以下路径均相对该提交根目录：

| 路径 | 参考价值 | 本驱动如何使用 |
| --- | --- | --- |
| `src/kuavo_humanoid_websocket_sdk/` | 可通过 pip 安装的 WebSocket SDK 源码、文档、atomic_skills 示例 | 核对远程协议、参数、传感器和轮臂分支 |
| `src/kuavo_humanoid_sdk/` | 在 ROS1 环境直接运行的 SDK | 交叉检查消息和参数；不把 rospy 带进 ROS2 容器 |
| `src/demo/test_kuavo_wheel_real/` | 轮臂真机示例，`lb_ctrl_api.py` 包含服务类型/请求字段 | 核对 5-W 模式与版本差异，避免调用双足控制 |
| `src/kuavo_msgs/msg/{sensorsData,jointData,imuData}.msg` | 原始实测数据定义 | 映射 q/dq/ddq/tau 和 IMU |
| `src/humanoid-control/humanoid_controllers/src/ControlDataManager.cpp` | 控制器实际消费的传感器与 odom | 复核加速度/力矩字段和本体速度 |
| `src/humanoid-control/humanoid_controllers/src/humanoidController_wheel_wbc.cpp` | 轮臂自由度参数、观测与 FK | 复核下肢4、双臂14、头2及实测关节顺序 |
| `src/kuavo_assets/models/biped_s400062/urdf/biped_s400062.urdf` | 5-W 模型参考 | 精简后随驱动打包显示；真机模型用于实测关节名称校验 |
| `docs/4开发接口/Kuavo 5-W 接口使用文档.md` | 5-W 特定控制接口/单位/优先级 | 执行卡的主要依据 |

可重现入口：[固定提交源码树](https://github.com/LejuRobotics/kuavo-ros-opensource/tree/d9c036e19fce5689fcfa8c596ee89b2045bebb32)。
未将整套第三方代码复制进驱动，也没有将官方示例的机器型号/默认状态当成真机配置。

## 2. WebSocket SDK 实际协议

`kuavo_humanoid_sdk/common/websocket_kuavo_sdk.py` 创建的是 `roslibpy.Ros(host, port=9090)`。
`kuavo/core/ros/{control,state,param}.py` 使用 ROS topic/service/param，底层为 rosbridge。
因此这份 SDK 文档**没有推翻原来的 rosbridge 架构**，而是补充了适配依据：

```text
ROS1 Noetic + kuavo_msgs + rosbridge/rosapi
    ↕ rosbridge WebSocket JSON
driver 薄适配层 + ROS2 Humble
    → domain 42 / FastDDS loopback → Agent Core
```

“支持 ROS1、暂不支持 ROS2”指 SDK 对接的机器人接口。WebSocket 客户端不要求机器人安装 ROS2；
也不意味着 Agent Core 可以直接订阅 ROS1 消息。ROS1 没有 DDS domain，桥接后的 ROS2 participant 仍需 domain42。

保留小型适配层的原因：便于约束重连不重放运动、分离协议成功与服务执行成功、严格检查状态、接入 ACP，
同时避开整个 SDK 的单例连接、双足默认值和不相关策略依赖。现有传输是 SDK 所用协议的实现，**不是完整 SDK 的替代品**。

## 3. 版本和接口差异

官方[SDK 介绍](https://kuavo.lejurobot.com/manual/basic_usage/kuavo-ros-control/docs/4开发接口/kuavo-humanoid-websocket-sdk接口/kuavo-humanoid-websocket-sdk介绍/)要求 SDK 与下位机版本匹配：
正式版对应控制仓库 master，测试版对应 beta。文档的 `1.2.1` 是版本匹配示例，不是对任意真机的推荐固定版本。
通用说明中的“28自由度、12腿部”是双足配置，不能用于 5-W。

| 项目 | 核查结果 / 限制 |
| --- | --- |
| 机型参数 | `/robot_type=1` 为轮臂，2 为双足；不使用 SDK 的缺省双足值 |
| 自由度参数 | wheel WBC 写入 armRealDof=14、legRealDof=4、headRealDof=2、waistRealDof=0；驱动探测并校验 |
| 原始传感器 | `/use_shm_communication` 选择 `/sensors_data_raw_shm` 或 `/sensors_data_raw`；后者为 SDK 的默认选择 |
| 观测 | 轮臂是 `/mobile_manipulator_mpc_observation`，不是双足的 `/humanoid_mpc_observation`；本驱动直接使用 raw 实测，避免默认零状态 |
| 末端指令 | 轮臂使用 `/mm/two_arm_hand_pose_cmd`，不使用双足路径 |
| 下肢单位 | `/lb_leg_traj` topic 使用度；SDK `/lb_leg_control_srv` 服务使用弧度；不可混用 |
| 头部 | 通用 SDK 有位置接口；本驱动move_pos改用SDK `/robot_head_motion_data`绝对角度接口（度），并以真实反馈确认ACP；`/cmd_head_vel`仅用于清除残余速度 |
| quick mode | 手册部分版本为 SetBool；固定源码示例已为 `changeLbQuickModeSrv` + `quickMode` 枚举；本驱动不调用，不猜测类型 |
| 新增服务 | 源码还含定时轨迹、末端误差查询等新接口；未确认真机版本前不替换手册中的稳定控制接口 |

文档对下位机 1.3.0 提及 `load_kuavo_real.launch` 的 `with_mm_ik=true`。
缺少 `/websocket_sdk_srv/get_robot.launch_status` 时，官方建议启动
`roslaunch h12pro_controller_node kuavo_humanoid_sdk_ws_srv.launch`。
该 launch 是 SDK 启动管理服务；**它没有代替 rosbridge_websocket.launch**。
本驱动不调用 SDK 的机器人启动管理接口，也不会自动更改上述 launch 或启动控制器。

## 4. 实测关节、模型与 IMU

消息定义及 `ControlDataManager::sensorsDataCallback` 一致：

| ROS1 字段 | 输出 | 单位 |
| --- | --- | --- |
| joint_data.joint_q | q | rad |
| joint_data.joint_v | dq | rad/s |
| joint_data.joint_vd | ddq | rad/s² |
| joint_data.joint_torque | vendor.joint_torque；单位核实后才填 tau | SDK 数据类允许 N·m 或 A，真机未核实 |
| imu_data.quat | quaternion，明确 wxyz | 无量纲 |
| imu_data.gyro | gyroscope | rad/s |
| imu_data.acc / free_acc | accelerometer / free_acceleration | m/s² |

特别注意：固定源码的 WebSocket SDK `state.py` 的关节回调把 `joint_vd` 传给 `torque`，
并将 current/torque 传给 `acceleration`，与消息定义及控制器消费代码矛盾。驱动按后两者映射，测试专门防止这两个字段混淆。
另外，SDK 的 `interfaces/data_types.py` 将 effort 单位表述为 Newton-meters **or Amperes**，消息定义没有消除这项歧义。
因此默认标准 tau 和其单位为 null，原始 joint_torque 完整保留在 vendor；
必须现场确认该型号硬件输出为关节端 N·m 后，才设置 `sensors.joint_torque_unit: N*m`。
不使用未知电机力矩常数或减速比，把电流猜成力矩。

原始数组顺序经轮臂控制器与 SDK 切片交叉核查为：下肢4、左臂7、右臂7、头部2。
驱动从真机 `/humanoid_description` 获取 URDF，通过以下 child link 找出真实 joint name，**不按 XML 遍历顺序猜数组顺序**：

```text
knee_link, leg_link, waist_link, waist_yaw_link,
zarm_l1_link ... zarm_l7_link,
zarm_r1_link ... zarm_r7_link,
zhead_1_link, zhead_2_link
```

参考 400062 URDF 对应 `knee_joint/leg_joint/waist_pitch_joint/waist_yaw_joint`、`zarm_*_joint`、`zhead_*_joint`。
参考模型还包含轮子和手指；raw 这20个关节不包含它们，驱动不伪造其测量值。
SDK 通用 `joint_names()` 的轮臂分支仅列下肢，部分模型目录的 controller_joint_names 配置仍带双足名称，均不直接照搬。
静态骨架遵循 G1、天轶 2.0 的目录及接口约定：`model` 读取本地
`resource/kuavo5w_model.urdf`，断线和控制器未启动均可读取，不依赖真机参数。
模型由上述固定提交的 400062 URDF 精简而来，保留全部连杆名称和关节定义，去除 visual/collision/inertial；
前端只绘制骨架，不需要网格。来源、校验和及许可证见 `resource/README.md` 与 `resource/LICENSE.leju`。
它是未完成现场几何/标定确认的参考模型，不能用作运动控制依据。
真机 URDF 缺失或20个受测关节名称与本地模型不一致时，`joints` 输出 invalid + 空数组，
诊断记录 `model_error`；静态模型仍可用，参考姿态不当作实测零角度。
本地文件本身缺失或解析失败时，`model` 才返回本地资源错误。
没有将 IMU 四元数直接套到轮式底盘根节点：其安装 frame 未在现场核实。

现场补充：独立标准 `/imu` 已有100Hz左右数据，因此优先读取该流，保留原始 frame，不依赖全身 raw。
当前机身标识 `KUA5W0710701302138` 由操作人员提供，不能据此选择400062几何或确认控制器版本。
全身 `/humanoid_description` 仍缺失；底盘的 `/robot_description` 不能替代全身模型。

外设适配以现场注册类型和实际数据为准：左右相机开启服务为 `std_srvs/SetBool`，压缩图像透传为
ROS2 `sensor_msgs/CompressedImage`；`/scan` 为标准 LaserScan。三路电池为底盘自定义消息，
缺少本地类时由普通用户 ROS1 relay 按已核验 type/MD5 解码，转标准 String。上述做法不安装完整 SDK，
镜像可能需要补充缺失的 ROS2 sensor_msgs；不因 SDK 具有音频示例就注册尚未验证的mic/speaker卡。

## 5. 里程计

`ControlDataManager::odomCallback` 取 `/odom.twist.twist` 的 x/y/wz，
`computeObservationFromSensorData` 随后将这些量从 body 旋转到 world，说明这里应读取**旋转之前的本体速度**。
SDK 数据类的“world”文字描述不能覆盖控制器中的实际变换。

驱动只将 vx/vy/wz 作为测量轴；vz/wx/wy 为 null。
运行时要求 child_frame_id 在 `sensors.odom_body_frames` 白名单内，默认 base_link/base_footprint。
不同实际 frame 需先核实 TF/发布器再配置；未知 frame 不输出有效速度。
原始 pose、twist、header 保存在 vendor；不把未经定位漂移评估的 pose 宣称为可导航的绝对位置。
使用 common.odom 的接口/样本/时间来源工具；10Hz 按窗口均值输出，空窗口不重发旧速度。

## 6. 卡片与 ACP 修改

- 新增同其他驱动一致的 `joints`、`imu`、`loco_state`，以及 resource `model`；控制诊断现从 `switch_mode.get_current_mode` 获取，不再暴露 `state` 卡。
- rosapi 参数读取在独立线程，不阻塞负责接收 service_response 的 WebSocket 线程。
- 启动/重连重新探测；机型/自由度未知或不匹配时拒绝运动。断线清除测量与模型，活动传感卡持续发布 invalid。
- 关节动作必须先验证运行时模型的关节映射/名称，并有0.5s内实测 q；在途模型失效也会终止动作。发送后必须收到新 reach-time，等待预计时长，再检测目标误差≤0.035rad，连续新测量覆盖≥0.2s。
  预计结束后5s内未收敛、实测丢失、连接/模式异常都结束为 error 并尝试软件暂停。
- 双臂/下肢关节动作额外要求现场核验的限位记录，绑定 robot_version、原始运行时 URDF 的 SHA-256 和准确关节顺序；越界在发布前拒绝，在途继续校验。默认 `sensors.joint_limits: null`，仅这两类关节动作不可用，不从显示骨架取控制限位。记录是现场负责人声明，软件绑定检查不替代硬件核验；配置格式见 README。
- 首次运动发送异常视为可能已送达：锁内先阻塞运动并锁存 `motion_send_uncertain`，锁外尝试全身暂停并以 error 取消 ACP，返回原 action_id 和 delivery uncertain。False/disable 受理不能证明旧目标取消；重连不清除、退出仍暂停。需现场处置控制器后再重启，重启本身不构成恢复证明。
- MCP 请求日志跨连接共享5s采样和抑制计数，区分成功与工具错误（含 HTTP 200），另记录实际卡片状态变化，轮询不制造状态变化；使用标准库，不增加依赖。
- 关节测量按 `sensor_time` 原始纳秒时间去重及拒绝倒序，不用回调时间/header.seq 证明新采集；要求两端系统时钟同步、采集年龄≤0.5s且未来偏差≤0.05s。指令后不同采集帧的原始时间跨度≥0.2s才允许收敛；不支持把启动相对/模拟时钟猜成系统时钟，实际时间基准仍需真机验收。
- 身份探测不再成功后停止：每轮结束约1s后经 ROS2 正常 timer 路径在后台重新读取参数/URDF，身份租约从读开始起最多3s；失败或过期拒绝/终止动作，模型或版本变化重新核对限位绑定。保持同会话原始传感器订阅，不因刷新制造重放。轮询存在检测窗口且多参数读取非原子，配置变更必须停机进行。
- Compose 的 rosbridge 环境覆盖未显式提供时为空，驱动读取完整操作配置中的 `rosbridge_url`；只在显式非空覆盖时优先采用环境值，两者均缺省才使用 localhost。
- 底盘/笛卡尔目标保留 reach-time 估计；速度保留时长+控制器 watchdog 推断，均不冒充实测收敛。
- 底盘新命令与旧命令共享 busy 通道。避免原先“先取消旧 ACP，再验证新指令”，也避免 cmd_vel 优先级窗口覆盖新位置目标。
- 每个物理执行卡继续保留 ACP、资源声明和 interrupt hook，动作分组补齐 required 参数。
- 系统 `start`/`stop` 仅返回生命周期标记，不调用全身暂停；在途动作仍由 bundle/ACP 管理，停止由 `stop_move`/interrupt 显式执行。
- 暂停/恢复/模式切换使用共用2s的服务与反馈预算；普通前台控制锁等待上限0.25s，生命周期不等待该锁。停止不走 busy 拒绝路径，令牌使旧恢复/模式切换失效；网络响应在锁外等待，旧请求在停止之后不能继续发送或解除本地阻塞。
- `motion_enabled` 默认 true（未配置亦然），运动、恢复及切换模式仍接受原有运行时检查；显式设为 false 时可只读运行，保留暂停/停止能力。
- 注册补齐 id/transport；rosbridge 错误日志只输出有界转义字段；移除本地未使用的 ROS2 geometry_msgs 依赖。
- MCP 只绑定127.0.0.1，校验对端/Host，拒绝所有 Origin 和浏览器预检；动作、服务和暂停错误均在源头限长并转义日志。
- 本地取消不撤销 ROS 服务执行。恢复请求的独立未决记录在超时/抢占后继续存在；仅同会话明确拒绝，或受理成功加有效 True 终态，才能结清。停止发送时仍有未决恢复，就不确认该次暂停；结清后需显式再次暂停，确保最终 disable 有序。断线/部分发送不能作为结清依据，也不提供重启清除不确定性的保证。
- ROS 消息回调按话题输出首次异常、每5s采样和一次恢复日志；无直接硬件访问，移除 privileged 及未使用的可写 data 挂载，不增加依赖。
- 官方 `humanoidController_wheel_wbc.cpp` 通过 `advertise(..., true)` 发布 latched `/enable_control_state`，只在初始化、暂停受理和恢复到 IDLE 时发布。不能按普通周期话题直接加2s消息超时。
  驱动独立线程读取 `/rosapi/publishers` 并重复同 id 订阅，核实来源及 latch 重新投递，续期3s状态通路验证；失败立即失效，过期 True/False 不走运动或暂停/恢复快捷路径。这个检查不等价于硬件心跳或物理停止确认。
  另核对 rosbridge 官方 ROS1 实现：[rosapi_node](https://github.com/RobotWebTools/rosbridge_suite/blob/ros1/rosapi/scripts/rosapi_node)、
  [subscribe capability](https://github.com/RobotWebTools/rosbridge_suite/blob/ros1/rosbridge_library/src/rosbridge_library/capabilities/subscribe.py)、
  [MultiSubscriber](https://github.com/RobotWebTools/rosbridge_suite/blob/ros1/rosbridge_library/src/rosbridge_library/internal/subscribers.py)。
  重复订阅会调用 `add_callback` 重新投递 latch；现场版本不支持时驱动拒绝运动并报告验证错误，不伪造刷新时间。

## 7. 仍需真机确认

实际控制仓库版本、参数与 URDF、rosbridge/rosapi 自定义消息可见性、odom 的 child frame、消息时延、实测收敛阈值、
DDS42/loopback 的跨容器接收，以及软件暂停和断网时控制器行为都需要真机验收。
公开 SDK 中的相机、手爪、语音、双足步态等能力不等于该台5-W已安装并开放；本轮不虚构这些卡。


## 8. 现场传感适配更新（2026-11-16）

- joints 双流：模型绑定的 skeleton 与不依赖模型的原始 joint_state；诊断数组绝不绕过关节控制的模型、时间和限位校验。
- loco_state 双流：map/odom 位姿展示与独立 body-frame motus.odom/1；地图定位使用实际 `/move_base/amcl_pose`，未实现导航执行卡。
- 电池字段全部展开，点云使用现有前端 `sensor/pointcloud` 二进制布局，并保留原始 LaserScan JSON。
- 左右 JPEG 分别命名 camera_left/camera_right；头部为实测 Orbbec Gemini335L，使用现场已有ROS1驱动和宿主机JPEG relay，未引入厂商完整SDK到镜像。
- 每相机独立接收/发布线程和有界latest-only缓存；WebSocket取消纯Python编码扫描，改用严格本地UTF-8解码，避免图像处理占满GIL；仍保持连接代次检查。
- 未见可用USB EB03音频设备，Livox网口无物理链路；不从已安装SDK、配置或话题名推断硬件在线。

具体源流、模型缺失原因、设备检查和集成验证以 ROBOT_INSPECTION.md 文末记录为准。

## 9. 头部深度与按卡启动（2026-11-16）

- 依据现场已安装 OrbbecSDK_ROS1 的 `ob_camera_node.cpp` / `ros_setup.cpp`：深度发布前已乘 `getValueScale()`，输出为16UC1毫米。
  原生640×480@15Hz实机验证通过；启用深度激光，关闭RGB对齐、TF及厂商点云发布。
- Agent Core现有 `image/depth-zlib` 渲染契约固定640×480小端uint16毫米；`camera_depth` 使用标准CompressedImage承载，format同名，保留采集header。
  宿主机stdlib relay仅剥除行填充/处理字节序并zlib无损压缩；0仍表示无效。驱动有界解压验证，不把JPEG/PNG/RVL当深度。
- 头部RGB/深度卡start缺帧时通过固定loopback路径请求对应systemd服务；仅启用中的卡缺帧会重试。
  全局检查只确保桥接和电池服务，stop只停止卡输出，避免关闭共用RGBD源进程。
- 当前laser_scan是二维测距的z=0投影。MID360配置与SDK不能证明硬件在线；专用网口carrier=0、无Livox发布者，不能提供真实三维雷达流。

### 2026-11-18 绝对头部接口补查

同一固定提交的 `humanoid_controllers/src/ControlDataManager.cpp` 保留 `/robot_head_motion_data` 的
`headExternalControlCallback`，读取yaw/pitch度值、按HEAD_JOINT_LIMITS限制并转换为rad写入目标状态；
`headVelCallback/headDeltaCallback`是额外的速度/增量入口，不是唯一可用接口。WebSocket SDK的
`ControlRobotHeadWebsocket.pub_control_robot_head`同样使用该绝对话题。驱动现在直接发送绝对度值，
不把实测位置差当成sticky目标增量，也不以估计转动时间冒充到位。本轮只读确认真机该话题有
`/nodelet_manager`订阅者，越界参数在发令前拒绝；尚未发送有效绝对角度目标做物理运动验收。

### 2026-11-18 版本63关节时间基准核对

此前将sensor_time当作墙钟采集时间的假设不适用于现场版本63硬件库。下位机NTP正常，
但sensor_time相对header落后约0.8秒；厂商installed/tests/motor_test.cpp也显示sensor_time使用start_time。
进一步只读检查实际installed/lib/libros_hardware_plant_cxx.so的
`HighlyDynamic::HardwareNode::sensorThreadFunc(ros::Publisher&)`确认：

核对库SHA-256：`16d3c27fa9704b8c5e5c012c7f6adced792f2843531661f7c52f810b1c14269f`。

- 0x288dc2调用ros::Time::now，保存循环初始时间。
- 0x288ea1调用HardwarePlant::readSensor，失败分支sleep后回循环，不发布该轮数据。
- 0x289f83重新调用ros::Time::now并写header.stamp；0x289f96–0x289f9d将循环累计时间写sensor_time。
- 发布和Rate::sleep后，0x28a4a9–0x28a4b8将配置的周期dt转Duration并累加到该循环时间。

因此该字段可用于区分成功采样循环和顺序，不能用于UTC数据年龄。驱动auto策略仅对运行时整数版本63
使用原始header.stamp做0.5秒新鲜度检查，并继续要求sensor_time与header双递增、增量一致，
重复/倒序帧不续租，不以接收时间或header.seq替代。ACP要求命令后有效时间和周期跨度均至少0.2秒。
其他版本仍使用严格sensor_time墙钟策略；硬件库若升级并改变语义，应重新核对，不能据此推断所有版本相同。


### 2026-11-18 头部未到位：官方配置与实际适配器核查

本节更新此前“尚未发送有效头部目标”的记录；不将命令被接收视为物理运动验收。

- 官方源码入口仍为Gitee的`leju-robot/kuavo-ros-opensource`，也提供LejuRobotics GitHub镜像。
  固定提交`d9c036e19fce5689fcfa8c596ee89b2045bebb32`的
  [v63配置](https://github.com/LejuRobotics/kuavo-ros-opensource/blob/d9c036e19fce5689fcfa8c596ee89b2045bebb32/src/kuavo_assets/config/kuavo_v63/kuavo.json)
  与真机运行参数均为头部kp=[10,10]、kd=[4,3]，位置范围yaw±30°、pitch±25°。
  通用SDK的yaw±80°比此范围宽，因此驱动卡片、请求校验及省略轴保持均收紧至±30°。
- 真机`src/kuavo_humanoid_sdk`的robot_head→core→ROS control链路：公共接口rad转度，
  `/robot_head_motion_data`发送一次`joint_data=[yaw,pitch]`；驱动的绝对度接口与之相符。
  ControlDataManager接收度并转rad，wheel控制器直接将目标写入`/joint_cmd`尾部两关节。
- 当前进程361577的初始化日志确认DUAL_BUS实际创建`motorevo::MotorevoActuator`，
  不能因为进程同时加载`libruiwo_actuatorCXXLib.so`就把旧RuiWoActuator当作实际发送路径。
  现场工作树HEAD为`69274e610eca8640abfa42821ecbac0721d16602`，不等于上述公开参考提交；
  `installed/lib/libmotorevo_controller.so` SHA-256为
  `9d4f97b963f3bf94d85553332723670e62ad7f5a151ccabd9ac17d740a5b1ea7`。
- 固定公开提交中的`motorevo_actuator.cpp::set_positions`将硬件层度值转rad，优先采用每轮传入的kp/kd，
  缺省时才回退到缓存；`motor_ctrl.cpp`将位置/增益编码进PTM帧。当前日志明确将头部默认35/2覆盖为
  10/4、10/3，实际`/joint_cmd`也为后者。CAN YAML的35/2不是已验证的修复建议，
  `/hardware/get_motor_param`仅前四个关节有效，其余0不能解释为头部实时kp=0。
- 现场CAN配置Head_yaw/Head_pitch设备ID为15/16、ratio均36、ignore均false；方向分别false/true。
  公开v62 CAN示例方向相反，不能将示例方向/零点覆盖到这台机器。
- 已有旁站验证：yaw=10°时`/joint_cmd`为0.174532925rad，实测约6.673°；之后授权的yaw=5°测试中，
  绝对话题只观察到一次5°目标，`/joint_cmd`保持5°，6秒内实测约6.63°，用户观察几乎没动、无异常声音。
  这不能证明固定2/3缩放；亦未确认摩擦、增益、标定或CAN输出哪项是根因。
- 测试后请求全身软件暂停。厂商控制器仍保留绝对头部目标，软件暂停不等于清除该目标或硬件失能。
  本轮后续检查全部只读，没有改增益、减速比、方向、零点，未重新初始化或重复发送头部目标。
  仍需针对实际批次的厂家参数/电机反馈与CAN发送端联合验证，不能以增大指令或放宽ACP误差代替修复。
- 驱动保持实测±1°、不同采集周期跨0.2秒的ACP到位标准；超时现在带目标、实测及误差，
  继续报错并请求软件暂停。323项离线测试通过；本轮限位/诊断代码尚未部署到真机运行容器。


## 2026-11-19：三板复查、厂商限位与小角度旁站验证

本节更新前述arm不可用的结论；头部精确到位仍未通过。

- 再查上位机SDK/应用、下位机运行控制栈、独立底盘程序。手臂/头部控制归下位机，
  底盘导航/音频SDK没有另一套可替代的上肢控制接口。
- 厂商SDK `kuavo/robot_info.py::get_arm_joint_limits()`从实际机器人URDF读取手臂限位。
  驱动新增显式`arm_limit_source: vendor_runtime`，只接受运行时模型中14个revolute关节限位，
  与`/kuavo_configuration`的20关节位置限位（度）取交集；不以显示URDF作为控制依据。
  保留`commissioned`策略及显式核验记录优先级；没有把厂商配置标记成独立硬件认证。
- 实际参数robot_version=63，NUM_JOINT=20、NUM_ARM_JOINT=14、NUM_HEAD_JOINT=2。
  本机验证得到左臂度制上下界为[-180,90]/[-20,180]/[-85,85]/[-150,0]/[-90,90]/[-75,40]/[-40,40]，
  右臂为[-180,90]/[-180,20]/[-85,85]/[-150,0]/[-90,90]/[-40,75]/[-40,40]。
  此列表仅记录当次读取，代码不内置这些值，其他机器人重新读自己的配置。
- 首次旁站arm目标16.943°、实测16.889°，但ACP报缺少`arm_reach_time`。
  `/kuavo_arm_traj`已有厂商轨迹接收实现，当前控制器没有为本次调用提供aggregate预计时间。
  关节动作改以命令后实测独立采样持续到位完成（原0.035rad/0.2s阈值不变），
  设115s总超时以留在125s ACP窗口内；反馈/模型/限位失效仍立即报错并尝试暂停。
  笛卡尔动作继续用厂商预计时间，不混淆为实测完成。
- 修复后的正式arm卡旁站复验：左臂第1关节16.888542°→14.899590°，目标14.888542°，
  Agent Core收到`kuavo_arm_ce1119b97e6c4d26b0f5360cfa01559d`的`completed`事件。
  用户确认看到此前小角度动作、无抖动或异常声音。验证范围仅一个关节约2°，不是全工作空间验收。
- 头部正程目标8.416785°，实测从3.416785°到4.880998°后停滞，误差3.535787°；
  返程目标3.416785°，实测4.793403°，误差1.376618°。均正确报ACP到位失败。
  `/joint_cmd`保持正确目标、kp10/10；小角度响应与此前大角度日志不符合固定比例错误。
- 另查下位机`leju_motorevo_tools`提交532b0da5：工具支持驱动参数、错误码、方向辨识和磨线，
  配置包含v53头部及v62通用模板，没有找到该v63批次专用修复说明。
  模板中头部PA4310-25不能证明本机实际减速比或零位；公开Motorevo实现的正常set_positions
  将角度直接转rad，ratio字段用于换编码器零位圈，不支持用25/36缩放指令来修复。
  现有11月13日厂商维护报告已记载CAN回零未收敛，但没有确定根因。
- 没有运行会独占CAN的诊断/磨线/方向辨识程序，没有写电机参数或重启控制器。
  头部需进一步由厂商核对本机电机固件、PTM输出及机械/参数；当前证据不足以给出可验证的自动调参修复。
  模型仍与运行时biped_s63的全部35个joint定义一致，头部q[18]/q[19]按原始rad输出；不更换显示URDF掩盖物理偏差。

## 2026-11-19：controlled_spatial 的原生协议适配

进一步在独立底盘核对已安装生成类、厂商地图前端和后端命令实现后，找到与导航同源的取消接口：

- `/move_base/set_dispatch_mode`，`leju_mobile_base_msgs/SetDispatchMode`，
  control_mode=2 是 CANCEL_CURRENT_TASK，0/1是软件急停设置/解除，驱动不调用0/1。
  与 MoveToTarget / CheckArrived 一样由 `/leju_node` 提供；每次 RPC 都核对三者类型和唯一同源提供者。
- MoveToTarget 返回 task_id；CheckArrived 用同一个ID、blocking=false、timeout=0查询。
  底盘附带测试把状态0/1/2/3解释为PENDING/ACTIVE/ARRIVED/FAILED；未知状态不当成功。
  取消确认还需 cancel 返回成功及新采样 /odom 停稳，不使用杀节点的 `/cancel_task`。
- 地图后端是底盘 `:8888`，新鲜状态来自 `/ws/home` 的 pose2d、state、agvState、localizationStatus。
  `POST /map/list?type=pgm`（空body）读取栅格列表；默认列表是path路网，不能冒充栅格地图。
  `GET /map/current?returnMapData=false` 的 mapMd5 是路网标识，不能单独标识栅格坐标系。
  固定只读 SSH 操作对当前地图的PGM+YAML计算SHA256，再组合路网MD5绑定语义点。
- 厂商前端采用 ChangeMode(MANUAL)、StartCartoMapping({})、
  SaveCartoMap(mapName,savePath)、AgvSystemRestart({})、ChangeCartoMap/ChangeRoadMap(mapName)。
  WebSocket命令使用 method/id/params，匹配taskResult中method和id，错误或丢失确认不重发。
  保存必须等待PGM列表出现，再等服务恢复并核对加载后的文件身份；不能只凭success完成ACP。
  ChangeMode(AUTO)恢复模式不表示已定位，也不意味着解除底盘软件急停。

`host/navigation_rpc.py` 随镜像构建，用既有SSH通路在底盘运行固定操作、导入当地ROS消息类。
无需新守护服务、rosbridge重启或安装额外SDK。容器只增加Python源文件，不增加运行依赖。
本轮实际只读探针确认了服务同源、地图文件指纹和状态格式，并发现现场websocket-client对象
不支持context manager；已使用contextlib.closing兼容，回归也使用不实现__enter__的socket。
源码实现与现场运动验收区分：建图/保存/载图、MoveToTarget以及中途取消未在本轮实际调用。

## 2026-11-19：卡片 start 准备底盘及厂商绑定兼容

按操作者要求，base_drive.start与controlled_spatial.start共用准备流程：硬件故障检查、
CANCEL_CURRENT_TASK(2)、不同新鲜里程计采样停稳、必要时CANCEL_SOFT_STOP(1)、
原始RobotChassisState及WebSocket反馈核对。info不改变状态，start不发送位姿/速度目标，
不使能全身控制。准备与执行共享控制转换锁，停止使旧准备失效；较早解除返回后再设置软件急停、取消并确认停稳。
未确定投递的解除持久化保留，不能靠重启抹掉；确定拒绝/反馈失败经确认停止后允许重试。

现场进一步发现同名服务类型并不保证兼容：/move_base/set_dispatch_mode运行服务MD5为
09d13be4b7fff0abe4e61d99b81ca03b，但底盘Python绑定为f126da6618daf59c6668404bf03dea2d，
因此旧绑定调用在TCPROS握手阶段被拒绝，未执行请求。MoveToTarget/CheckArrived的Python绑定则与服务一致。
同机/opt/jaten/include/leju_mobile_base_msgs/SetDispatchMode{,Request,Response}.h与运行服务匹配，
其请求为uint8 control_mode，响应为bool success/string message；按字段计算服务MD5亦为09d13...。
适配先检查所有服务的实际类型/MD5；已安装Python绑定匹配时直接使用，否则只接受上述固定字段布局、
C++服务头MD5与服务器三者一致的版本，利用现有genpy生成内存消息类，不改厂家文件、不绕过校验、不安装包。
现场已完成请求字节[2]及响应的序列化往返检查，原生soft_stop操作也已返回成功、停稳且software_estop=true。
其他未知ABI保持拒绝，而非把名字相同当作可控。
