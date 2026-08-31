# ROS2底层运动参考例程

> Source: https://wiki.pndbotics.com/robot/low_level_ros2/

底层运动开发例程实现了将机器人从任意初始关节位置，复位至预备状态，然后用两种不同模式控制机器人踝关节摆动和手指位置。例程源代码位于 `pnd_ros2/example`，可从 [此处](https://github.com/pndbotics/pnd_ros2) 访问。

源码目录：

```
├── adam_lite
│   └── adam_lite_low_level_example.py
├── adam_sp
│   └── adam_sp_low_level_example.py
├── adam_pro
│   └── adam_pro_low_level_example.py
└── adam_u
    └── adam_u_low_level_example.py
```

## 运行方式

1. 确保ros2 msg接口编译完成：

```bash
cd $ROOT
git clone https://github.com/pndbotics/pnd_ros2.git
cd pnd_ros2
colcon build
```

2. 使用网线连接机器人

3. 运行参考例程（以Adam_Pro为例）：

```bash
cd $ROOT/pnd_ros2
# 注册ROS2环境
source /opt/ros/humble/setup.bash
# 注册pnd_adam ros2 消息接口
source install/setup.bash
# 运行例子
python3 example/adam_pro/adam_pro_low_level_example.py
```

## 代码解析

以下代码针对Adam Pro 版本机型，其他版本机型的例程结构均类似，只是关节index定义不同。

### 核心类型与功能介绍

| 类型名称 | 描述 |
|----------|------|
| ADAMJointIndex | 所有关节按名称排序索引。也是各个关节在 `LowCmd.MotorCmd` 命令数组中的下标 |
| LowCmd | 电机指令相关ROS2消息结构体 |
| LowState | 电机状态相关ROS2消息结构体 |
| HandCmd | 手指命令相关ROS2消息结构体 |
| DemonController | 核心控制逻辑类（ROS2 node） |

```python
class DemonController(Node):
    def __init__(self):
        super().__init__('demon_controller')
        self.time_ = 0.0
        self.control_dt_ = 0.0025  # [2.5ms]
        self.duration_ = 3.0       # [3 s]
        self.counter_ = 0

        self.lowcmd_pub_ = self.create_publisher(LowCmd, 'lowcmd', 10)
        self.handcmd_pub_ = self.create_publisher(HandCmd, 'handcmd', 10)

        self.getstate_flag = False
        self.timer = self.create_timer(self.control_dt_, self.LowCmdWrite)
        self.mutex = threading.Lock()

        self.low_state = LowState()
        self.low_state.motor_state = [MotorState() for _ in range(ADAM_PRO_NUM_MOTOR)]

        self.low_cmd = LowCmd()
        self.low_cmd.motor_cmd = [MotorCmd() for _ in range(ADAM_PRO_NUM_MOTOR)]

        self.hand_cmd = HandCmd()
        self.close_hand = np.array([500]*12, dtype=int)
        self.open_hand = np.array([1000]*12, dtype=int)

        self.lowstate_sub_ = self.create_subscription(
            LowState, "lowstate", self.getLowState, 10)
```

### 底层状态接收回调函数

话题 `lowstate` 回调实现了：
- 记录当前各个关节所在位置，为闭环控制提供数据
- 补齐MotorState对象个数，避免后续越界访问崩溃
- 隔一段时间打印IMU欧拉角

### 底层指令发送函数

`DemonController.LowCmdWrite` 用于周期性(2.5ms)发送底层指令：

- 阶段 1：将 Adam 机器人从任意初始关节位置，复位至预备状态 (前 3秒)
- 阶段 2：踝关节 PR控制模式控制踝关节持续摆动 3秒
- 阶段 3：把左右手指打开，一直到程序终止

发布话题：
- `lowcmd` — 身体关节控制
- `handcmd` — 手指控制
