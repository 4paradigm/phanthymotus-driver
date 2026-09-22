# Franka 双臂 Driver

通过官方 franka_ros2 的 ROS 2 Action 接口控制**两台独立机械臂**，MCP 端口
15741。左右臂分别提供关节状态、点到点轨迹及取消；可选官方 Franka 夹爪。
本包不安装/启动本体驱动，不在 Python 中实现 FCI 实时力矩循环。

## 适用条件

- 每台机械臂已有与机器人型号、固件、libfranka 配套的官方 franka_ros2 控制栈。
- 每臂暴露一个 `control_msgs/action/FollowJointTrajectory` Action，以及独立命名
  空间的 JointState 话题；轨迹控制器使用 ros2_control JTC 的 spline 插值。
- 可选夹爪使用官方 `franka_gripper` 的 `control_msgs/action/GripperCommand`。
- 本版没有 FR3 Duo 一体机控制器适配、双臂同步启动、Cartesian IK、环境/自碰撞
  规划、遥操流接入或自动错误恢复。两套独立 FR3/Panda 的实际兼容性仍需现场确认。

参考接口核对自 [franka_ros2 humble](https://github.com/frankarobotics/franka_ros2/tree/humble)、
[官方 FR3 轨迹控制器配置](https://github.com/frankarobotics/franka_ros2/blob/humble/franka_fr3_moveit_config/config/fr3_ros_controllers.yaml)、
[JTC 接口](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
和 [Franka 夹爪实现](https://github.com/frankarobotics/franka_ros2/blob/humble/franka_gripper/src/gripper_action_server.cpp)。
核对日期：2026-09-22，humble commit `3b50164eabbe2d461baca007bbbbe03c024387bd`。
现场需记录使用的上游 commit/版本，不能将 Jazzy 控制栈
自动视为与本 Humble 部署等价。

## 安装和配置

1. 在 Linux 主机按厂商文档配置两台机械臂，分别使用 left/right 等独立 namespace。
   在对应域用 `ros2 action list -t`、`ros2 topic list -t` 核对真实接口。
2. 将 config.yaml 复制到 `/opt/phanthy-motus/config/franka.yaml`。示例是两台独立
   FR3 的名字，不会自动识别到货型号；按实际 ROS 图填写 Action、状态话题、7 个
   关节名。joint_state_topic 必须分开，即使两台使用相同的关节名称。
3. 开启运动前，为每臂填入现场确认的 7 维 lower/upper（rad）、max_velocity
   （rad/s）、max_acceleration（rad/s²），并设 motion_enabled=true。
   默认仅可读，空限位不能启用运动。
4. 若启用夹爪，填写 gripper_action、gripper_max_width（全开口宽度，m）、
   gripper_max_force（N）。不要将第三方夹爪绑定为官方 Franka 夹爪。
5. 在仓库根目录执行 `./build.sh franka/dual_arm`。部署模板要求官方 ROS 控制器
   与 Driver 在**同一主机**，robot_domain_id 默认为 0；Agent Core 域为 42。
   回环 DDS 配置不会发现另一台主机上的厂商 ROS 节点。

本体 IP 由官方控制栈持有，不复制到可分享的 Canvas。Agent Core CA 挂载到
`/work/certs/cert.pem`，ACP 回调校验证书；回调失败可从 info.last_completion 查看。

## 调用与停止

`franka_left_arm.move` / `franka_right_arm.move` 接收 7 维 joint（rad）、duration
（0.5–30 秒）。必须有 0.5 秒内收到的位置与速度反馈，且机械臂静止。Driver 向
JTC 提交起点和终点的位置、零速度与零加速度，使其生成五次插值，并按解析峰值
检查速度和加速度上限。该约束针对目标轨迹，不代表测得的电机速度/加速度上限。
返回 action_id；只有控制器成功终态才上报 ACP completed。

`stop` 取消该工具持有的 Action，待控制器终态才释放占用。停止早于 goal 接收时，
延迟接收的 goal 仍会被取消。超时也请求取消；结果未知则保持锁定，不能继续叠加
目标。Controller ready 只说明 Action 服务存在，不等于机器人已使能。
此停止依赖官方控制器与网络，不替代本体紧急停止，也不会取消其他客户端的目标。

夹爪 move 的 width 是两指间的**总开口宽度**。官方 GripperCommand 的 position
表示单指位移，所以发送时除以 2；force 使用 N。夹爪取消通过官方 Action，
其实现调用 gripper.stop()。达到终态前不会把提交成功当作夹爪到位。

## 验证

```bash
python3 -m pytest -q tests/test_franka_dual_arm_driver.py
python3 scripts/check_service_yml.py franka/dual_arm
```

离线测试覆盖请求/取消竞态、错误终态、限位、反馈与夹爪宽度；不连接本体。
ROS Action 联调、Docker ARM64 构建、实际型号与固件、安装坐标和碰撞检查仍需
设备环境验收。厂商控制器必须保持其本体安全限制，不能靠延长 duration 替代规划。
