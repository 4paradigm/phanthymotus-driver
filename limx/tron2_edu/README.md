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
移动双臂的升降台/底盘、相机、夹爪、回充不在本版功能内。

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
