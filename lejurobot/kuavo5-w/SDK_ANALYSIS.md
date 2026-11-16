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
| 头部 | 通用 SDK 有位置接口；本驱动继续使用 5-W 手册明确规定的 `/cmd_head_vel`，单位度/秒 |
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

- 新增同其他驱动一致的 `joints`、`imu`、`loco_state`，以及 resource `model`；保留 `state` 诊断。
- rosapi 参数读取在独立线程，不阻塞负责接收 service_response 的 WebSocket 线程。
- 启动/重连重新探测；机型/自由度未知或不匹配时拒绝运动。断线清除测量与模型，活动传感卡持续发布 invalid。
- 关节动作必须先验证运行时模型的关节映射/名称，并有0.5s内实测 q；在途模型失效也会终止动作。发送后必须收到新 reach-time，等待预计时长，再检测目标误差≤0.035rad，连续新测量覆盖≥0.2s。
  预计结束后5s内未收敛、实测丢失、连接/模式异常都结束为 error 并尝试软件暂停。
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
