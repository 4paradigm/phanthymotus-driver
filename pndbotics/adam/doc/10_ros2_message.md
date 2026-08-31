# ROS2消息定义

> Source: https://wiki.pndbotics.com/robot/ros2_message/

## ROS2 接口

底层通信提供了用户端PC与机器人之间的数据交互功能。

- 订阅话题 `lowstate`（类型：`pnd_adam.msg.dds_.LowState_`） 获取 Adam 当前状态。
- 订阅话题 `handstate`（类型：`pnd_adam.msg.dds_.HandState_`）获取手指当前状态。
- 发布话题 `lowcmd`（类型：`pnd_adam.msg.dds_.LowCmd_`） 控制全身关节电机（不含灵巧手）、电池等设备。
- 发布话题 `handcmd`（类型：`pnd_adam.msg.dds_.Handcmd_`） 控制手指关节电机。

> **注意**: ROS2 接口的话题名称不带 `rt/` 前缀（DDS接口使用 `rt/lowstate`，ROS2接口使用 `lowstate`）。消息结构体定义与DDS版本完全相同，请参见 `09_dds_message.md`。

---

## 接口说明

采用ROS2订阅或发布话题。话题信息存储在由 IDL 定义的结构体中，常用结构体有：

| 结构体名称 | 说明 |
|-----------|------|
| `_BatteryData_.py` | 电池状态（电压、电流、电量等） |
| `_HandCmd_.py` | 灵巧手期望位置控制 |
| `_HandState_.py` | 灵巧手实际状态反馈 |
| `_IMUState_.py` | Adam IMU 姿态及加速度数据 |
| `_LowCmd_.py` | Adam 底层全身控制命令 |
| `_LowState_.py` | Adam 底层全身状态汇总 |
| `_MotorCmd_.py` | 单个电机控制参数（位置、速度、力矩及增益） |
| `_MotorState_.py` | 单个电机状态反馈 |

---

## 消息定义

消息结构体与DDS版本完全一致，请参考 DDS消息定义文档（09_dds_message.md），唯一区别是话题名称：

| DDS 话题 | ROS2 话题 | 说明 |
|----------|-----------|------|
| `rt/lowcmd` | `lowcmd` | 身体期望位置控制 |
| `rt/lowstate` | `lowstate` | 身体实际状态反馈 |
| `rt/handcmd` | `handcmd` | 手指期望位置控制 |
| `rt/handstate` | `handstate` | 手指实际状态反馈 |
