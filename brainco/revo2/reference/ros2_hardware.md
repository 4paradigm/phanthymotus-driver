# ROS2 硬件驱动包 (https://www.brainco-hz.com/docs/revolimb-hand/revo2/ros2/hardware.html)

Source: (see original URL)

# Revo 2 灵巧手驱动包-- Modbus / CAN FD ​

## 概述 ​

BrainCo Hand Driver 驱动包为 BrainCo Revo 2 灵巧手提供基于 Modbus/CAN FD 通信协议的 ROS 2 硬件接口。该功能包实现了 ros2_control 硬件接口，支持通过串口或 CAN FD 对 6 个手指关节进行实时控制和状态监控。

**通信协议支持：**

- Modbus （默认）：通过串口通信，兼容性好，部署简单
- CAN FD ：支持同时控制多只手，需 ZLG CAN FD 硬件设备

## 特性 ​

- 通信协议 ：支持 Modbus 和 CAN FD 双协议
- ros2_control 集成 ：ros2_control 硬件接口实现
- 双手支持 ：支持左手和右手配置，以及双手同时控制
- MoveIt 集成 ：可选的 MoveIt 集成用于运动规划
- 实时控制 ：高频控制循环，实现精确的手指操控
- 状态反馈 ：所有关节的位置、速度反馈
- 轨迹控制 ：关节轨迹控制器支持，实现平滑运动执行
- 自动检测 ：支持 Modbus 自动检测串口和从站 ID（仅单手模式）

## 环境系统 ​

- Ubuntu 22.04
- ROS 2 (Humble)
- Python 3.8+

### Modbus 模式（默认） ​

- Modbus 串口设备（如 /dev/ttyUSB0）

### CAN FD 模式（可选） ​

- ZLG USB-CAN FD 设备（如 USBCANFD-200U）
- ZLG CAN FD 驱动库（已随包内置）
- CAN FD 总线连接

## 安装和设置 ​

### 1. 依赖项 ​

确保已安装以下软件包：

bash

```
# ROS 2 依赖项
sudo apt install ros-humble-controller-manager ros-humble-joint-trajectory-controller
sudo apt install ros-humble-joint-state-broadcaster ros-humble-robot-state-publisher
sudo apt install ros-humble-moveit ros-humble-moveit-ros-planning-interface

# Python 依赖项
pip3 install rclpy trajectory_msgs sensor_msgs control_msgs
```

### 2. BrainCo Stark SDK ​

BrainCo Stark SDK 在 `vendor/` 目录中，可直接使用。如需更新 SDK 版本，可运行：

bash

```
cd brainco_hardware/brainco_hand_driver
./scripts/download_sdk.sh
```

### 3. 构建工作空间 ​

bash

```
# 进入工作空间
cd ~/brainco_ws

# 安装依赖项
rosdep install --ignore-src --from-paths src -y -r

# 构建功能包
colcon build --packages-select brainco_hand_driver --symlink-install

# Source 工作空间
source install/setup.bash
```

### 4. 验证安装 ​

bash

```
# 检查功能包是否可用
ros2 pkg list | grep brainco_hand_driver

# 检查串口设备权限
ls -l /dev/ttyUSB*
# 如果权限不足，将用户添加到 dialout 组：
sudo usermod -a -G dialout $USER
# 然后重新登录
```

## 硬件连接 ​

### Modbus 串口设置 ​

在使用驱动之前，请确保您的 Modbus 串口设置正确：

bash

```
# 检查串口设备
ls -l /dev/ttyUSB*

# 检查串口权限
groups | grep dialout

# 如果没有权限，添加用户到 dialout 组
sudo usermod -a -G dialout $USER
# 重新登录后生效

# 测试串口连接（可选）
sudo apt install minicom
sudo minicom -D /dev/ttyUSB0
```

## 快速开始 ​

### 启动右手系统 ​

bash

```
# Modbus 模式（默认）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right

# CAN FD 模式（需要 ZLG USB-CAN FD 设备）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=right protocol:=canfd
```

### 启动左手系统 ​

bash

```
# Modbus 模式（默认）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left

# CAN FD 模式（需要 ZLG USB-CAN FD 设备）
ros2 launch brainco_hand_driver revo2_system.launch.py hand_type:=left protocol:=canfd
```

### 启动 MoveIt 集成 ​

bash

```
# 右手带 MoveIt（Modbus 模式）
ros2 launch brainco_hand_driver revo2_real_moveit.launch.py hand_type:=right

# 右手带 MoveIt（CAN FD 模式）
ros2 launch brainco_hand_driver revo2_real_moveit.launch.py hand_type:=right protocol:=canfd

# 左手带 MoveIt（Modbus 模式）
ros2 launch brainco_hand_driver revo2_real_moveit.launch.py hand_type:=left

# 左手带 MoveIt（CAN FD 模式）
ros2 launch brainco_hand_driver revo2_real_moveit.launch.py hand_type:=left protocol:=canfd
```

### 启动双手系统（MoveIt） ​

bash

```
# 双手系统 Modbus 模式（默认）
ros2 launch brainco_hand_driver dual_revo2_real_moveit.launch.py
```

### Launch 参数 ​

| 参数 | 默认值 | 描述 |
| --- | --- | --- |
| hand_type | right | 手型选择： left 或 right |
| protocol | modbus | 通信协议： modbus 或 canfd |
| protocol_config_file | "" | 协议配置文件（YAML），留空则使用默认配置 |

## 控制接口 ​

### 使用 Topic 接口 ​

右手轨迹控制示例：

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
      positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
      time_from_start: {sec: 2}
    }]
  }'
```

左手轨迹控制示例：

bash

```
ros2 topic pub --once /left_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "left_thumb_proximal_joint",
      "left_thumb_metacarpal_joint",
      "left_index_proximal_joint",
      "left_middle_proximal_joint",
      "left_ring_proximal_joint",
      "left_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
      time_from_start: {sec: 2}
    }]
  }'
```

### 使用 Action 接口 ​

右手 Action 控制示例：

bash

```
ros2 action send_goal /right_revo2_hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{
    trajectory: {
      joint_names: [
        "right_thumb_proximal_joint",
        "right_thumb_metacarpal_joint",
        "right_index_proximal_joint",
        "right_middle_proximal_joint",
        "right_ring_proximal_joint",
        "right_pinky_proximal_joint"
      ],
      points: [
        {
          positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 1}
        },
        {
          positions: [0.5, 0.0, 1.4, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 0.0, 0.0],
          time_from_start: {sec: 3}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 0.0],
          time_from_start: {sec: 4}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 1.4],
          time_from_start: {sec: 5}
        }
      ]
    }
  }'
```

左手 Action 控制示例：

bash

```
ros2 action send_goal /left_revo2_hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{
    trajectory: {
      joint_names: [
        "left_thumb_proximal_joint",
        "left_thumb_metacarpal_joint",
        "left_index_proximal_joint",
        "left_middle_proximal_joint",
        "left_ring_proximal_joint",
        "left_pinky_proximal_joint"
      ],
      points: [
        {
          positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 1}
        },
        {
          positions: [0.5, 0.0, 1.4, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 0.0, 0.0],
          time_from_start: {sec: 3}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 0.0],
          time_from_start: {sec: 4}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 1.4],
          time_from_start: {sec: 5}
        }
      ]
    }
  }'
```

### 归零位置（Home Position） ​

返回归零位置：

bash

```
# 右手
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

# 左手
ros2 topic pub --once /left_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "left_thumb_proximal_joint",
      "left_thumb_metacarpal_joint",
      "left_index_proximal_joint",
      "left_middle_proximal_joint",
      "left_ring_proximal_joint",
      "left_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 1}
    }]
  }'
```

## 使用 MoveIt 控制灵巧手 ​

MoveIt 提供了图形化界面，可以方便地规划和执行灵巧手的运动。

### 启动 MoveIt ​

首先启动 MoveIt 集成：

bash

```
# 启动右手 MoveIt
ros2 launch brainco_hand_driver revo2_real_moveit.launch.py hand_type:=right

# 或启动左手 MoveIt
ros2 launch brainco_hand_driver revo2_real_moveit.launch.py hand_type:=left

# 或启动双手 MoveIt
ros2 launch brainco_hand_driver dual_revo2_real_moveit.launch.py \
    left_port:=/dev/ttyUSB0 \
    right_port:=/dev/ttyUSB1
```

启动后，RViz 会自动打开并显示灵巧手的可视化界面。

### 1. 选择运动组 ​

在 RViz 的左侧面板中，找到 "Motion Planning" 插件。首先需要选择要控制的运动组：

- 对于单手模式：选择 revo2_left_hand 或 revo2_right_hand
- 对于双手模式：可以选择 revo2_left_hand 或 revo2_right_hand 分别控制左右手

### 2. 设置目标 ​

在 "Motion Planning" 面板中，切换到 "Planning" 标签页，然后点击 "Query" 下的 "Select Goal State" 或直接使用 "Joint Values" 标签：

MoveIt 提供了预定义的姿态，通过 `config/xxx.srdf` 定义和加载，可以在 "Planning" 标签页的 "Query" 部分选择：

- hand_open ：张开手
- hand_half_close ：半握
- hand_close ：握拳

### 3. 规划并执行运动 ​

设置好目标后，点击 "Plan & Execute" 按钮进行规划并执行：

- Plan ：规划从当前位置到目标位置的运动轨迹
- Execute ：执行规划好的轨迹，控制真实灵巧手运动
- Plan & Execute ：自动规划并执行

规划成功后，您可以在 RViz 中看到：

- 绿色轨迹：规划的运动路径
- 橙色轨迹：已执行的路径

### 4. 执行结果 ​

点击 "Execute" 后，MoveIt 会将轨迹发送给控制器，真实灵巧手会按照规划的轨迹运动：

执行过程中，您可以：

- 在 RViz 中实时观察灵巧手的运动
- 通过 /joint_states 话题监控关节状态
- 使用 ros2 topic echo /joint_states 查看详细的关节反馈

## 监控和调试 ​

### 检查系统状态 ​

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

### 预期的输出 ​

#### 核心节点列表（右手配置） ​

bash

```
/controller_manager
/joint_state_broadcaster
/right_revo2_hand_controller
/robot_state_publisher
```

#### 控制器状态 ​

bash

```
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
```

#### 硬件组件（右手配置） ​

bash

```
Hardware Component 1
  name: Revo2RightSystem
  type: system
  plugin name: brainco_hand_driver/BraincoHandHardware
  state: id=3 label=active
  command interfaces
    right_thumb_proximal_joint/position [available] [claimed]
    right_thumb_metacarpal_joint/position [available] [claimed]
    right_index_proximal_joint/position [available] [claimed]
    right_middle_proximal_joint/position [available] [claimed]
    right_ring_proximal_joint/position [available] [claimed]
    right_pinky_proximal_joint/position [available] [claimed]
```

#### 硬件接口（右手配置） ​

bash

```
command interfaces
  right_index_proximal_joint/position [available] [claimed]
  right_middle_proximal_joint/position [available] [claimed]
  right_pinky_proximal_joint/position [available] [claimed]
  right_ring_proximal_joint/position [available] [claimed]
  right_thumb_metacarpal_joint/position [available] [claimed]
  right_thumb_proximal_joint/position [available] [claimed]
state interfaces
  right_index_proximal_joint/position
  right_index_proximal_joint/velocity
  right_middle_proximal_joint/position
  right_middle_proximal_joint/velocity
  right_pinky_proximal_joint/position
  right_pinky_proximal_joint/velocity
  right_ring_proximal_joint/position
  right_ring_proximal_joint/velocity
  right_thumb_metacarpal_joint/position
  right_thumb_metacarpal_joint/velocity
  right_thumb_proximal_joint/position
  right_thumb_proximal_joint/velocity
```

#### 核心话题列表（右手配置） ​

bash

```
/joint_states
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

#### 核心动作列表（右手配置） ​

bash

```
/right_revo2_hand_controller/follow_joint_trajectory
```

## 关节映射 ​

### 右手关节 ​

| 关节名称 | 描述 | 角度范围（弧度） | 最大角度（度） |
| --- | --- | --- | --- |
| right_thumb_proximal_joint | 拇指近端关节 | 0 ~ 1.03 | 59 |
| right_thumb_metacarpal_joint | 拇指掌骨关节 | 0 ~ 1.57 | 90 |
| right_index_proximal_joint | 食指近端关节 | 0 ~ 1.41 | 81 |
| right_middle_proximal_joint | 中指近端关节 | 0 ~ 1.41 | 81 |
| right_ring_proximal_joint | 无名指近端关节 | 0 ~ 1.41 | 81 |
| right_pinky_proximal_joint | 小指近端关节 | 0 ~ 1.41 | 81 |

### 左手关节 ​

| 关节名称 | 描述 | 角度范围（弧度） | 最大角度（度） |
| --- | --- | --- | --- |
| left_thumb_proximal_joint | 拇指近端关节 | 0 ~ 1.03 | 59 |
| left_thumb_metacarpal_joint | 拇指掌骨关节 | 0 ~ 1.57 | 90 |
| left_index_proximal_joint | 食指近端关节 | 0 ~ 1.41 | 81 |
| left_middle_proximal_joint | 中指近端关节 | 0 ~ 1.41 | 81 |
| left_ring_proximal_joint | 无名指近端关节 | 0 ~ 1.41 | 81 |
| left_pinky_proximal_joint | 小指近端关节 | 0 ~ 1.41 | 81 |

## 功能包结构 ​

bash

```
brainco_hand_driver/
├── launch/                                      # Launch 文件
│   ├── revo2_system.launch.py                    # 主系统启动文件
│   ├── revo2_real_moveit.launch.py              # MoveIt 集成启动文件（单手）
│   └── dual_revo2_real_moveit.launch.py         # MoveIt 集成启动文件（双手）
├── config/                                      # 配置文件
│   ├── protocol_*.yaml                          # 协议配置文件（Modbus/CAN FD）
│   ├── xxx.srdf                                 # MoveIt 语义描述文件
│   ├── xxx.urdf.xacro                           # URDF XACRO 文件
│   ├── xxx.ros2_control.xacro                   # ros2_control 配置
│   ├── xxx_controllers.yaml                     # 控制器配置
│   ├── xxx_initial_positions.yaml               # 初始位置配置
│   └── ...                                      # 其他配置文件
├── include/                                     # 头文件
│   └── brainco_hand_driver/                     # 驱动头文件
│       ├── brainco_hand_hardware.hpp            # 硬件接口头文件
│       └── logger_macros.hpp                    # 日志宏定义
├── src/                                         # 源文件
│   └── brainco_hand_hardware.cpp                # 硬件接口实现
├── scripts/                                     # 工具脚本
│   └── download_sdk.sh                          # SDK 下载脚本
├── vendor/                                      # BrainCo Stark SDK
│   └── dist/                                    # SDK 分发文件
├── CMakeLists.txt                               # 构建配置
├── package.xml                                  # 功能包描述
├── brainco_hand_driver_plugins.xml              # 插件描述
└── README.md                                    # 英文说明
```

# Revo 2 灵巧手驱动包 -- EtherCAT ​

## 概述 ​

BrainCo Hand EtherCAT 驱动包为 BrainCo Revo 2 灵巧手提供基于 EtherCAT 通信协议的 ROS 2 硬件接口。该功能包实现了 ros2_control 硬件接口，支持通过 EtherCAT 主站对 6 个手指关节进行实时控制和状态监控。

## 特性 ​

- EtherCAT 通信 ：与 Revo 2 灵巧手硬件的实时 EtherCAT 通信
- ros2_control 集成 ：ros2_control 硬件接口实现
- 双手支持 ：支持左手和右手配置
- MoveIt 集成 ：可选的 MoveIt 集成用于运动规划
- 实时控制 ：高频控制循环，实现精确的手指操控
- 状态反馈 ：所有关节的位置、速度反馈
- 轨迹控制 ：关节轨迹控制器支持，实现平滑运动执行

## 环境系统 ​

- Ubuntu 22.04
- ROS 2 (Humble)
- EtherCAT 主站 (IgH EtherCAT Master)
- Python 3.8+

## 安装和设置 ​

### 1. 依赖项 ​

确保已安装以下软件包：

bash

```
# ROS 2 依赖项
sudo apt install ros-humble-controller-manager ros-humble-joint-trajectory-controller
sudo apt install ros-humble-joint-state-broadcaster ros-humble-robot-state-publisher

# Python 依赖项
pip3 install rclpy trajectory_msgs sensor_msgs control_msgs

# EtherCAT 主站（如果尚未安装）
# 请按照系统的 EtherCAT 主站安装指南进行安装
```

### 2. 构建工作空间 ​

bash

```
# 进入工作空间
cd ~/brainco_ws

# 安装依赖项
rosdep install --ignore-src --from-paths src -y -r

# 构建功能包
colcon build --packages-select brainco_hand_ethercat_driver stark_ethercat_interface stark_ethercat_driver --symlink-install

# Source 工作空间
source install/setup.bash
```

### 3. 验证安装 ​

bash

```
# 检查功能包是否可用
ros2 pkg list | grep brainco_hand_ethercat_driver

# 检查 EtherCAT 主站状态
sudo systemctl status ethercat
```

## 硬件连接 ​

### EtherCAT 设置 ​

在使用驱动之前，请确保您的 EtherCAT 设置正确：

bash

```
# 检查网络接口
ip link show

# 检查 EtherCAT 主站状态
sudo systemctl status ethercat

# 检查 EtherCAT 从站设备
sudo ethercat slaves

# 检查设备 SDO/PDO 信息
sudo ethercat sdos
sudo ethercat pdos
```

## 快速开始 ​

### 启动右手系统 ​

bash

```
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py hand_type:=right
```

### 启动左手系统 ​

bash

```
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py hand_type:=left
```

bash

```
# 过滤域信息输出
ros2 launch brainco_hand_ethercat_driver revo2_system.launch.py 2>&1 | grep -v "\[ros2_control_node-1\] Domain"
```

### 启动 MoveIt 集成 ​

bash

```
# 右手带 MoveIt
ros2 launch brainco_hand_ethercat_driver revo2_real_moveit.launch.py hand_type:=right

# 左手带 MoveIt
ros2 launch brainco_hand_ethercat_driver revo2_real_moveit.launch.py hand_type:=left
```

### Launch 参数 ​

#### 基础系统参数（revo2_system.launch.py） ​

| 参数 | 默认值 | 描述 |
| --- | --- | --- |
| hand_type | right | 手型选择： left 或 right |
| prefix | "" | 关节名称前缀，用于多机器人设置 |
| ctrl_param_duration_ms | 20 | 运动时间参数，单位毫秒（1ms = 最快速度） |
| robot_controller | 自动生成 | 控制器名称（根据 hand_type 自动生成） |

#### MoveIt 集成参数（revo2_real_moveit.launch.py） ​

| 参数 | 默认值 | 描述 |
| --- | --- | --- |
| hand_type | right | 手型选择： left 或 right |
| ctrl_param_duration_ms | 10 | 运动时间参数，单位毫秒（1ms = 最快速度） |
| use_rviz | true | 是否启动 RViz 可视化 |
| publish_monitored_planning_scene | true | 是否发布监控的规划场景 |

## 控制接口 ​

### 使用 Topic 接口 ​

右手轨迹控制示例：

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
      positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
      time_from_start: {sec: 2}
    }]
  }'
```

左手轨迹控制示例：

bash

```
ros2 topic pub --once /left_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "left_thumb_proximal_joint",
      "left_thumb_metacarpal_joint",
      "left_index_proximal_joint",
      "left_middle_proximal_joint",
      "left_ring_proximal_joint",
      "left_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.5, 0.5, 0.5, 0.5, 0.5, 0.5],
      time_from_start: {sec: 2}
    }]
  }'
```

### 使用 Action 接口 ​

右手 Action 控制示例：

bash

```
ros2 action send_goal /right_revo2_hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{
    trajectory: {
      joint_names: [
        "right_thumb_proximal_joint",
        "right_thumb_metacarpal_joint",
        "right_index_proximal_joint",
        "right_middle_proximal_joint",
        "right_ring_proximal_joint",
        "right_pinky_proximal_joint"
      ],
      points: [
        {
          positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 1}
        },
        {
          positions: [0.5, 0.0, 1.4, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 0.0, 0.0],
          time_from_start: {sec: 3}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 0.0],
          time_from_start: {sec: 4}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 1.4],
          time_from_start: {sec: 5}
        }
      ]
    }
  }'
```

左手 Action 控制示例：

bash

```
ros2 action send_goal /left_revo2_hand_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  '{
    trajectory: {
      joint_names: [
        "left_thumb_proximal_joint",
        "left_thumb_metacarpal_joint",
        "left_index_proximal_joint",
        "left_middle_proximal_joint",
        "left_ring_proximal_joint",
        "left_pinky_proximal_joint"
      ],
      points: [
        {
          positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
          time_from_start: {sec: 1}
        },
        {
          positions: [0.5, 0.0, 1.4, 0.0, 0.0, 0.0],
          time_from_start: {sec: 2}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 0.0, 0.0],
          time_from_start: {sec: 3}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 0.0],
          time_from_start: {sec: 4}
        },
        {
          positions: [0.5, 0.0, 1.4, 1.4, 1.4, 1.4],
          time_from_start: {sec: 5}
        }
      ]
    }
  }'
```

### 归零位置（Home Position） ​

返回归零位置：

bash

```
# 右手
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

# 左手
ros2 topic pub --once /left_revo2_hand_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  '{
    joint_names: [
      "left_thumb_proximal_joint",
      "left_thumb_metacarpal_joint",
      "left_index_proximal_joint",
      "left_middle_proximal_joint",
      "left_ring_proximal_joint",
      "left_pinky_proximal_joint"
    ],
    points: [{
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 1}
    }]
  }'
```

## 测试脚本 ​

功能包包含一个交互式测试脚本：

bash

```
# 进入脚本目录
cd <package_path>/scripts/

# 交互式菜单
python3 test_revo2_ethercat.py --menu

# 直接测试命令
python3 test_revo2_ethercat.py --test all_fingers --amplitude 0.8
python3 test_revo2_ethercat.py --test individual
python3 test_revo2_ethercat.py --test home
```

## 监控和调试 ​

### 检查系统状态 ​

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

### 预期的输出 ​

#### 核心节点列表（右手配置） ​

bash

```
/controller_manager
/joint_state_broadcaster
/right_revo2_hand_controller
/robot_state_publisher
```

#### 控制器状态 ​

bash

```
joint_state_broadcaster      joint_state_broadcaster/JointStateBroadcaster          active
right_revo2_hand_controller   joint_trajectory_controller/JointTrajectoryController  active
```

#### 硬件组件（右手配置） ​

bash

```
Hardware Component 1
  name: Revo2RightEthercatSystem
  type: system
  plugin name: stark_ethercat_driver/EthercatDriver
  state: id=3 label=active
  command interfaces
    right_thumb_proximal_joint/position [available] [claimed]
    right_thumb_metacarpal_joint/position [available] [claimed]
    right_index_proximal_joint/position [available] [claimed]
    right_middle_proximal_joint/position [available] [claimed]
    right_ring_proximal_joint/position [available] [claimed]
    right_pinky_proximal_joint/position [available] [claimed]
```

#### 硬件接口（右手配置） ​

bash

```
command interfaces
  right_index_proximal_joint/position [available] [claimed]
  right_middle_proximal_joint/position [available] [claimed]
  right_pinky_proximal_joint/position [available] [claimed]
  right_ring_proximal_joint/position [available] [claimed]
  right_thumb_metacarpal_joint/position [available] [claimed]
  right_thumb_proximal_joint/position [available] [claimed]
state interfaces
  right_index_proximal_joint/effort
  right_index_proximal_joint/position
  right_index_proximal_joint/velocity
  right_middle_proximal_joint/effort
  right_middle_proximal_joint/position
  right_middle_proximal_joint/velocity
  right_pinky_proximal_joint/effort
  right_pinky_proximal_joint/position
  right_pinky_proximal_joint/velocity
  right_ring_proximal_joint/effort
  right_ring_proximal_joint/position
  right_ring_proximal_joint/velocity
  right_thumb_metacarpal_joint/effort
  right_thumb_metacarpal_joint/position
  right_thumb_metacarpal_joint/velocity
  right_thumb_proximal_joint/effort
  right_thumb_proximal_joint/position
  right_thumb_proximal_joint/velocity
```

#### 核心话题列表（右手配置） ​

bash

```
/joint_states
/right_revo2_hand_controller/joint_trajectory
/robot_description
/tf
/tf_static
```

#### 核心动作列表（右手配置） ​

bash

```
/right_revo2_hand_controller/follow_joint_trajectory
```

## 关节映射 ​

### 右手关节 ​

| 关节名称 | 描述 | 角度范围（弧度） | 最大角度（度） |
| --- | --- | --- | --- |
| right_thumb_proximal_joint | 拇指近端关节 | 0 ~ 1.03 | 59 |
| right_thumb_metacarpal_joint | 拇指掌骨关节 | 0 ~ 1.57 | 90 |
| right_index_proximal_joint | 食指近端关节 | 0 ~ 1.41 | 81 |
| right_middle_proximal_joint | 中指近端关节 | 0 ~ 1.41 | 81 |
| right_ring_proximal_joint | 无名指近端关节 | 0 ~ 1.41 | 81 |
| right_pinky_proximal_joint | 小指近端关节 | 0 ~ 1.41 | 81 |

### 左手关节 ​

| 关节名称 | 描述 | 角度范围（弧度） | 最大角度（度） |
| --- | --- | --- | --- |
| left_thumb_proximal_joint | 拇指近端关节 | 0 ~ 1.03 | 59 |
| left_thumb_metacarpal_joint | 拇指掌骨关节 | 0 ~ 1.57 | 90 |
| left_index_proximal_joint | 食指近端关节 | 0 ~ 1.41 | 81 |
| left_middle_proximal_joint | 中指近端关节 | 0 ~ 1.41 | 81 |
| left_ring_proximal_joint | 无名指近端关节 | 0 ~ 1.41 | 81 |
| left_pinky_proximal_joint | 小指近端关节 | 0 ~ 1.41 | 81 |

## 功能包结构 ​

bash

```
brainco_hand_ethercat_driver/
├── launch/                                      # Launch 文件
│   ├── revo2_system.launch.py                    # 主系统启动文件
│   └── revo2_real_moveit.launch.py               # MoveIt 集成启动文件
├── config/                                      # 配置文件
├── include/                                     # 头文件
│   └── revo2_ethercat_plugins/                  # 插件头文件
├── src/                                         # 源文件
│   └── revo2_joints_system_slave.cpp            # 硬件接口实现
├── scripts/                                     # 工具脚本
│   └── test_revo2_ethercat.py                   # 测试脚本
├── CMakeLists.txt                               # 构建配置
├── package.xml                                  # 功能包描述
├── revo2_plugins.xml                            # 插件描述
└── README.md                                    # 英文说明
```

## 联系方式 ​

如有问题或建议，请联系 BrainCo 开发团队或者通过页面帮助入口提交工单。
