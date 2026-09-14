# ROS2 项目概述 (https://www.brainco-hz.com/docs/revolimb-hand/revo2/ros2/overview.html)

Source: (see original URL)

# Revo 2 灵巧手 ROS 2 项目概述 ​

BrainCo Hand ROS 2 是面向 BrainCo Revo 2 灵巧手的 ROS 2 软件包集合，包含硬件驱动、仿真和运动规划相关功能。本项目基于 ROS 2 Humble 开发，支持单手和双手配置。

## 仓库地址和示例代码 ​

- Github
- ROS 2 Example

## 主要特性 ​

- 硬件驱动 ：支持 Modbus、CAN FD 和 EtherCAT 通信协议
- ros2_control 集成 ：提供 ros2_control 硬件接口
- Gazebo 仿真环境 ：基于 Ignition Gazebo 6 的物理仿真
- MoveIt 运动规划 ：提供 MoveIt 2 配置与规划接口
- 实时控制 ：提供手指关节控制循环
- 双手支持 ：支持左手、右手以及双手同时控制
- 机械臂集成 ：提供 RM65 机械臂与 Revo 2 灵巧手的集成演示

## 系统要求 ​

### 基础环境 ​

- 操作系统 ：Ubuntu 22.04
- ROS 版本 ：ROS 2 Humble
- Python 版本 ：Python 3.8+

### 硬件要求（硬件控制模式） ​

#### Modbus 模式（默认） ​

- Modbus 串口设备（如 /dev/ttyUSB0 ）
- 串口权限配置（用户需在 dialout 组中）

#### CAN FD 模式（可选） ​

- ZLG USB-CAN FD 设备（如 USBCANFD-200U）
- CAN FD 总线连接

### 仿真环境要求 ​

- Gazebo ：Ignition Gazebo 6（用于仿真功能包）
- MoveIt 2 ：用于运动规划功能包

## 功能包说明 ​

### 1. revo2_description ​

**功能**：Revo 2 灵巧手的 URDF 模型描述包，提供机器人模型的 3D 可视化。

**主要特性**：

- 左右手 URDF 模型
- RViz 可视化支持
- 关节和链接定义

### 2. brainco_hardware ​

**功能**：Revo 2 灵巧手的硬件驱动包，提供基于 ros2_control 的硬件接口。

**主要特性**：

- 支持 Modbus 、CAN FD 和 Ethercat 通信协议
- ros2_control 硬件接口实现
- 支持单手和双手配置
- MoveIt 集成支持
- 实时关节位置和速度反馈

### 3. brainco_gazebo ​

**功能**：Revo 2 灵巧手的 Gazebo 仿真包，提供物理仿真环境。

**主要特性**：

- 基于 Ignition Gazebo 6 的物理仿真
- 支持单手和双手仿真
- Gazebo-MoveIt 整合
- RViz 可视化集成

### 4. brainco_moveit_config ​

**功能**：Revo 2 灵巧手的 MoveIt 配置包，提供运动规划功能。

**主要特性**：

- 支持左手、右手和双手配置
- MoveIt 2 集成
- FakeSystem 支持

## 快速开始 ​

### 1. 环境准备 ​

bash

```
# 检查 ROS 2 环境
echo $ROS_DISTRO  # 应该输出: humble

# 创建工作空间（如果还没有）
mkdir -p <workspace>/src
cd <workspace>
```

**注意**：请将 `<workspace>` 替换为您的工作空间路径，例如 `~/brainco_ws`。

### 2. 克隆仓库 ​

bash

```
# 进入工作空间 src 目录
cd <workspace>/src

# 克隆主仓库
git clone https://github.com/BrainCoTech/brainco_hand_ros2.git
cd brainco_hand_ros2

# 使用 vcs 工具克隆依赖仓库
vcs import . < brainco_hand.repos --recursive --skip-existing
```

**说明**：如果您的系统没有安装 `vcs` 工具，请先安装：

bash

```
sudo apt-get install python3-vcstool
```

### 3. 安装依赖并构建 ​

bash

```
# 返回工作空间根目录
cd <workspace>

# 更新软件包列表
sudo apt-get update

# 更新 rosdep 数据库
rosdep update

# 安装所有依赖项
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 构建所有功能包
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --continue-on-error

# 激活工作空间
source install/setup.bash
```

## 使用场景 ​

### 场景 1：硬件控制（真实灵巧手） ​

如果您有真实的 Revo 2 灵巧手硬件，可以使用 `brainco_hand_driver` 进行控制：

bash

```
# 启动右手系统（Modbus 模式）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right

# 启动右手系统（带 MoveIt）
ros2 launch brainco_hand_driver revo2_real_moveit.launch.py hand_type:=right

# 启动双手系统
ros2 launch brainco_hand_driver dual_revo2_real_moveit.launch.py
```

### 场景 2：Gazebo 仿真 ​

如果您想在没有硬件的情况下进行开发和测试，可以使用 Gazebo 仿真：

bash

```
# 启动单手仿真
ros2 launch brainco_gazebo revo2_hand_gazebo.launch.py hand_type:=right

# 启动双手仿真
ros2 launch brainco_gazebo dual_revo2_hand_gazebo.launch.py

# 启动带 MoveIt 的仿真
ros2 launch brainco_gazebo revo2_hand_gazebo_moveit.launch.py hand_type:=right
```

### 场景 3：MoveIt 运动规划（无硬件） ​

如果您想使用 MoveIt 进行运动规划，但不需要硬件或仿真，可以使用 FakeSystem：

bash

```
# 启动右手 MoveIt（FakeSystem）
ros2 launch brainco_moveit_config revo2_right_moveit.launch.py

# 启动双手 MoveIt（FakeSystem）
ros2 launch brainco_moveit_config dual_revo2_moveit.launch.py
```

## 双手 MoveIt 仿真示意 ​

## 控制接口 ​

### Topic 接口 ​

所有功能包都提供标准的 ROS 2 Topic 接口：

- 关节状态 ： /joint_states (sensor_msgs/JointState)
- 轨迹命令 ： /xxx_revo2_hand_controller/joint_trajectory (trajectory_msgs/JointTrajectory)

### Action 接口 ​

- 轨迹执行 ： /xxx_revo2_hand_controller/follow_joint_trajectory (control_msgs/action/FollowJointTrajectory)

### 控制示例 ​

#### 右手张开手掌 ​

bash

```
ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "right_thumb_proximal_joint",
      "right_thumb_metacarpal_joint",
      "right_index_proximal_joint",
      "right_middle_proximal_joint",
      "right_ring_proximal_joint",
      "right_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 1}
    }]
  }'
```

#### 右手握拳 ​

bash

```
ros2 topic pub --once /right_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "right_thumb_proximal_joint",
      "right_thumb_metacarpal_joint",
      "right_index_proximal_joint",
      "right_middle_proximal_joint",
      "right_ring_proximal_joint",
      "right_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.1, 0.8, 1.4, 1.4, 1.4, 1.4],
      time_from_start: {sec: 2}
    }]
  }'
```

## 监控和调试 ​

### 系统状态检查 ​

bash

```
# 列出所有运行的节点
ros2 node list

# 列出所有控制器
ros2 control list_controllers

# 检查硬件组件
ros2 control list_hardware_components

# 检查硬件接口
ros2 control list_hardware_interfaces

# 列出所有话题
ros2 topic list

# 列出所有动作
ros2 action list

# 监控关节状态
ros2 topic echo /joint_states
```

## 联系方式 ​

如有问题或建议，请联系 BrainCo 开发团队或者通过页面帮助入口提交工单。
