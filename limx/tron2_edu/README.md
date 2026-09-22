# LimX TRON 2 EDU

基于厂商上层 WebSocket SDK，端口 15740。机器人保留厂商运控；本 Driver
不接管底层力矩控制，也不启动 SDK 示例中的动作序列。

| 安装构型 | 本版本功能 |
| --- | --- |
| `fixed_arms` / `mobile_arms` | 身份与连接、运行模式/诊断/电量、关节状态、双臂末端位姿、14 关节目标限位检查 |
| `biped` | 上述基本信息、50 Hz 有租期的行走速度比值（x/y/z） |
| `wheeled_biped` | 基本信息、50 Hz 有租期的轮足 x/z 速度，拒绝横移 |

只在所选构型中发布对应 MCP 工具。模块更换后必须停止 Driver，按机器人信息页
更新 ACCID 和 profile 后重新启动。不能把双臂、掌足、轮足作为可同时调用的能力。
移动双臂的升降台/底盘及逐际二指夹爪提供可选状态采集；它们的运动控制、相机
和回充不在本版功能内。

## 配置与运行

将 config.yaml 复制到 `/opt/phanthy-motus/config/tron2.yaml`，填写实际 endpoint
（如厂商说明的 `ws://<机器人地址>:5000`）、ACCID、构型和 enabled。
这些私有连接信息只保存在宿主机配置中，不放入 Canvas/Solution。
连接必须收到 ACCID 一致的消息；所有请求严格匹配 guid 和响应类型。
不自动重连，不重试运动指令，不把发送成功或连接断开当作物理动作完成。

在仓库根目录运行 `./build.sh limx/tron2_edu`，或 ROS 2 Humble 环境中：

```bash
PYTHONPATH=. python3 limx/tron2_edu/main.py
```

默认禁止运动。启用足式速度前，在本体侧选择上层开发模式，通过厂商控制器
进入 WALK，再设 `motion_enabled: true`。Driver 不自动站起、切模式或恢复摔倒。
`tron2_velocity.set_velocity` 参数 x/y/z、lease（0.05–0.5 秒）。必须持续续租；
到期保持发送零速度。发送和 stop 使用同一锁，stop 返回后不会再发旧的非零目标。
缺失新鲜状态、IMU/电机故障、控制器拒绝、断线都会清除目标并锁定运动。
恢复需先断开再显式连接，随后发送新指令。网络中断后不能保证零速度送达，
状态不会报告已物理停止。stop 是速度归零请求，不是紧急断电。

掌足 x/y/z 是无量纲比例；轮足 x 是 m/s，y 必须为 0。
厂商文档把轮足旋转 z 标作 m/s，单位表述存在歧义，本版不将它宣称为 rad/s；
现场需与厂商确认后配置速度上限（默认绝对值 0.2）。

## 定频采集与录制

`tron2_telemetry` 提供 start、stop、get、info，声明 data/json 输出话题
`/<namespace>/tron2/state`，ROS 类型为 std_msgs/msg/String。连接成功后自动启动
采集，也可单独 start；后者不会连接机器人或使能本体功能。stop 只停止采集，不
停止本体运动，断开 connection 则同时结束采集和本 Driver 持有的速度流。

telemetry_hz 默认 5 Hz（可设 1–10），它是每频道轮询及发布的频率上限。单独工作
线程串行查询，不持有运动锁，慢响应不会累积请求队列；实际采样率以接收时间为准。

| 频道 | 构型/启用条件 |
| --- | --- |
| robot_info | 全构型，从厂商通知读取模式、诊断、电量/软件版本 |
| imu | 全构型，仅采集上游已开启的 IMU 通知；不自动使能或关闭全局 IMU |
| joint_states / eef_pose | 双臂构型，原生关节状态与左右末端位姿 |
| gripper | 双臂构型，gripper_state_enabled=true，需已安装逐际二指夹爪 |
| lifter / chassis | mobile_arms 且 mobile_state_enabled=true，需对应模块 |

每频道分别返回 fresh、age_ms、error、sample。sample 保留厂商数据，包含原始
WebSocket source_timestamp_ms、本机 received_at_ns / received_monotonic_ns、
本地接收 sequence、连接 session_id；查询还提供 round_trip_ms。
外层 timestamp_ms 是发布时刻，不能当作硬件采样时刻。厂商数据若自带 timestamp
也保留在 data 内；未提供的时间不会用本机时间冒充。

fresh 只按本机接收年龄计算（robot_info 2.5 秒，其余 1.5 秒），不证明源端实时性。
过期、异常、断线或来自上一连接的数据返回 sample=null。相同样本重复发布时
原始时间和序号不变；sequence 不是硬件丢帧计数。各频道独立采样，未做时钟同步。
关节数据验证等长、有限数值和关节名；末端保留厂商 WXYZ 顺序并检查单位四元数。
可选模块保留厂商原始字段和单位，不将未提供的反馈填成零。ACCID 不写入数据话题。

last_velocity_command 记录速度发送目标、GUID、提交/发送时间和发送状态。
它也覆盖 watchdog/stop 发出的零速度；sent 不代表物理动作已执行，
physical_execution_confirmed 始终为 false。此频道不能替代本体实测速度。

在装有 rosbag2 的同一主机使用以下命令录制。示例假设 `ROS_NAMESPACE=robot`，
实际话题名从 tron2_telemetry.info.topic_out 获取：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml
ros2 bag record /robot/tron2/state
```

本版提供标准 ROS 2 数据入口；相机同步、天轶专用遥操录制格式和 VLA 数据集导出
仍属平台适配范围，没有在 Driver 中重建一套录制系统。

## 双臂运动的明确缺口

[官方 SDK 开发指南 V1.2 §3.5.5](https://www.limxdynamics.com/zh/documents/847884267345285120)
说明 `request_emgy_stop` 只在空闲模式响应，运动中不响应。因此本版不提供
MoveJ/MoveP/MoveH、夹爪运动或遥操 ServoJ/ServoP 的执行工具。
`tron2_arm_target.validate` 仅检查左右各 7 关节（rad）的文档限位，不发送运动，
也不代表通过自碰撞/环境碰撞检查。

双臂执行需要厂商提供：运行中取消/保持的接口与确认反馈、适用固件版本、
ServoJ 的 16 维顺序、失联/断流后的控制器行为。当前主控软件版本应从
`tron2_robot_info.get` 的 sw_version 读取，再与该合同核对。不得用猜测接口补齐。

厂商参考：[用户手册](https://www.limxdynamics.com/zh/documents/844648486841487360)、
[安装视频](https://www.limxdynamics.com/zh/videos)。资料核对日期：2026-09-22。

## 验证

```bash
python3 -m pytest -q tests/test_tron2_edu_driver.py
python3 scripts/check_service_yml.py limx/tron2_edu
```

协议与生命周期测试使用本地假传输，不连接机器人。Docker、真实 ROS、实际
固件及本体动作必须在设备环境另行验收；离线通过不等于已完成真机验证。
