# ROS2接口说明

> Source: https://wiki.pndbotics.com/robot/ros2_interface/

上层运动控制开发基于PNDbotics机器人控制系统开发，通过调用底层运动控制接口可实现上层运动控制功能，也可基于开放接口及内置运动规划控制算法进行个性化开发。上层运动控制实现方式为ROS2，用户可根据例程开发控制上肢部分。

## 启动adam_demo

启动adam_demo脚本时需要在root用户下，在 `adam_demo/bin/` 目录下执行：

```bash
sudo su
sh run.sh
```

## 工程编译

启动adam_demo的脚本中使用了sudo权限，所以需要在编译前先切换到root用户下，编译包括 `robot_state_publisher` 和 `robot_state_subscriber` 两个包：

```bash
sudo su
source build.sh
```

## 数据接收节点

adam_demo启动后即可运行下面的指令接收上肢数据：

```bash
sh run_subscriber.sh
```

## 数据发布节点

- 当机器人处于站立状态时，xbox手柄中 `xx` 的正按键按下，终端打印 `real time retarget start` 表示机器人进入接收外部数据状态，此时执行：

```bash
sh run_publisher.sh
```

- 使用方法参考 `ros2_test/src/robot_state_publisher/robot_state_publisher/robot_state_publisher_node.py` 中描述，文件的 `63-68` 行为控制姿态和高度和上肢的示例，文件的 `43-60` 行为控制手指的示例

- 机器人停止接收外部数据时，xbox手柄中 `xx` 的负按键按下，终端打印 `real time retarget stop` 表示机器人停止接收外部数据状态

## 代码下载

- 代码仓库地址：[pnd_adam_ros2_publish](https://github.com/pndbotics/pnd_adam_ros2_publish/tree/master)

## 代码解析

### robot_state_publisher_node.py

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(
            JointState,
            'joint_states',
            10)
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        self.counter = 0

        # 定义关节名称，与retarget.cpp中的joint_name_publisher_保持一致
        self.joint_names = [
            "dof_pos/waistRoll",
            "dof_pos/waistPitch",
            "dof_pos/waistYaw",
            "dof_pos/shoulderPitch_Left",
            "dof_pos/shoulderRoll_Left",
            "dof_pos/shoulderYaw_Left",
            "dof_pos/elbow_Left",
            "dof_pos/wristYaw_Left",
            "dof_pos/wristPitch_Left",
            "dof_pos/wristRoll_Left",
            "dof_pos/shoulderPitch_Right",
            "dof_pos/shoulderRoll_Right",
            "dof_pos/shoulderYaw_Right",
            "dof_pos/elbow_Right",
            "dof_pos/wristYaw_Right",
            "dof_pos/wristPitch_Right",
            "dof_pos/wristRoll_Right",
            "root_pos/z",
            "dof_pos/hand_pinky_Left",
            "dof_pos/hand_ring_Left",
            "dof_pos/hand_middle_Left",
            "dof_pos/hand_index_Left",
            "dof_pos/hand_thumb_1_Left",
            "dof_pos/hand_thumb_2_Left",
            "dof_pos/hand_pinky_Right",
            "dof_pos/hand_ring_Right",
            "dof_pos/hand_middle_Right",
            "dof_pos/hand_index_Right",
            "dof_pos/hand_thumb_1_Right",
            "dof_pos/hand_thumb_2_Right"
        ]

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # 初始化position数组
        position_array = np.zeros(len(self.joint_names), dtype=np.float64)
        position_array[:3] = 0.0      # 腰部姿态 复合角范围建议从小到大尝试
        position_array[3:10] = 0.0    # 左臂关节 单位为弧度
        position_array[10:17] = 0.0   # 右臂关节 单位为弧度
        position_array[17] = 1.0      # base高度 范围[0.6m 1.0m]，站立时为1.0m

        # 手指范围[0 1000]，1000表示手指完全伸直，0表示手指完全弯曲
        hand_control_left = np.zeros(6, dtype=np.float64)
        hand_control_left[0] = 500.0   # pinky
        hand_control_left[1] = 500.0   # ring
        hand_control_left[2] = 500.0   # middle
        hand_control_left[3] = 500.0   # index
        hand_control_left[4] = 1000.0  # thumb_1
        hand_control_left[5] = 1000.0  # thumb_2

        hand_control_right = np.zeros(6, dtype=np.float64)
        hand_control_right[0] = 500.0
        hand_control_right[1] = 500.0
        hand_control_right[2] = 500.0
        hand_control_right[3] = 500.0
        hand_control_right[4] = 1000.0
        hand_control_right[5] = 1000.0

        position_array[18:24] = hand_control_left
        position_array[24:30] = hand_control_right

        msg.position = position_array.tolist()
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = JointStatePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### robot_state_subscriber_node.py

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class RobotStateSubscriber(Node):
    def __init__(self):
        super().__init__('robot_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'robot_states',
            self.listener_callback,
            10)
        self.counter = 0

    def listener_callback(self, msg):
        self.counter += 1
        if self.counter % 5 == 0:
            print("接收到消息：")
            max_name_length = max(len(name) for name in msg.name) if msg.name else 20
            for i in range(len(msg.name)):
                position = msg.position[i] if i < len(msg.position) else "N/A"
                velocity = msg.velocity[i] if i < len(msg.velocity) else "N/A"
                effort = msg.effort[i] if i < len(msg.effort) else "N/A"
                print(f"关节名称: {msg.name[i]:<{max_name_length}} 位置: {str(position):<8} 速度: {str(velocity):<8} 加速度: {str(effort):<8}")
            print("------------------------------")

def main(args=None):
    rclpy.init(args=args)
    subscriber = RobotStateSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 关节名称映射（ROS2 JointState topic）

发布话题：`joint_states`，订阅话题：`robot_states`

| 索引 | 关节名称 | 说明 |
|------|---------|------|
| 0 | dof_pos/waistRoll | 腰部横滚 |
| 1 | dof_pos/waistPitch | 腰部俯仰 |
| 2 | dof_pos/waistYaw | 腰部偏航 |
| 3 | dof_pos/shoulderPitch_Left | 左肩俯仰 |
| 4 | dof_pos/shoulderRoll_Left | 左肩横滚 |
| 5 | dof_pos/shoulderYaw_Left | 左肩偏航 |
| 6 | dof_pos/elbow_Left | 左肘 |
| 7 | dof_pos/wristYaw_Left | 左腕偏航 |
| 8 | dof_pos/wristPitch_Left | 左腕俯仰 |
| 9 | dof_pos/wristRoll_Left | 左腕横滚 |
| 10 | dof_pos/shoulderPitch_Right | 右肩俯仰 |
| 11 | dof_pos/shoulderRoll_Right | 右肩横滚 |
| 12 | dof_pos/shoulderYaw_Right | 右肩偏航 |
| 13 | dof_pos/elbow_Right | 右肘 |
| 14 | dof_pos/wristYaw_Right | 右腕偏航 |
| 15 | dof_pos/wristPitch_Right | 右腕俯仰 |
| 16 | dof_pos/wristRoll_Right | 右腕横滚 |
| 17 | root_pos/z | 机身高度 [0.6m~1.0m] |
| 18 | dof_pos/hand_pinky_Left | 左手小指 [0~1000] |
| 19 | dof_pos/hand_ring_Left | 左手无名指 |
| 20 | dof_pos/hand_middle_Left | 左手中指 |
| 21 | dof_pos/hand_index_Left | 左手食指 |
| 22 | dof_pos/hand_thumb_1_Left | 左手拇指1 |
| 23 | dof_pos/hand_thumb_2_Left | 左手拇指2 |
| 24 | dof_pos/hand_pinky_Right | 右手小指 |
| 25 | dof_pos/hand_ring_Right | 右手无名指 |
| 26 | dof_pos/hand_middle_Right | 右手中指 |
| 27 | dof_pos/hand_index_Right | 右手食指 |
| 28 | dof_pos/hand_thumb_1_Right | 右手拇指1 |
| 29 | dof_pos/hand_thumb_2_Right | 右手拇指2 |
