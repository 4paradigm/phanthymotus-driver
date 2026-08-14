# DDS底层运动参考例程

> Source: https://wiki.pndbotics.com/robot/low_level/

底层运动开发例程实现了将机器人从任意初始关节位置，复位至预备状态，然后用两种不同模式控制机器人踝关节摆动和手指位置。例程源代码位于 `pnd_sdk_python/example/low_level`，可从 [此处](https://github.com/pndbotics/pnd_sdk_python/tree/main/example/low_level) 访问。

源码目录按照机器人型号分别对应sample代码例子：

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

## 代码解析

以下代码针对Adam Pro 版本机型，其他版本机型的例程结构均类似，只是关节index定义不同。

### 核心类型与功能介绍

| 类型名称 | 描述 |
|----------|------|
| ADAMJointIndex | 所有关节按名称排序索引。也是各个关节在LowCmd.MotorCmd命令数组中的下标 |
| LowCmd | 电机指令相关结构体 |
| LowState | 电机状态相关结构体 |
| HandCmd | 手指命令相关结构体 |
| Custom | 核心控制逻辑类 |
| ChannelPublisher | DDS发布器 |
| ChannelSubscriber | DDS订阅器 |

例程的主程序定义在 `adam_pro_low_level_example.py` 中，主程序实例化 `Custom` 类。在 `Custom` 的构造函数中创建了两个发布器和一个订阅器：

- 定义12维手指自由度在握拳时的位置参数 `Custom.close_hand`
- 创建 `rt/lowcmd` 主题电机位置发布器，`rt/handcmd` 主题手指位置发布器
- 底层状态接收 `rt/lowstate` 主题回调，调用函数 `Custom.LowStateHandler`

```python
class Custom:
    def __init__(self):
        self.time_ = 0.0
        self.control_dt_ = 0.0025  # [2.5ms]
        self.duration_ = 3.0       # [3 s]
        self.counter_ = 0
        self.low_cmd = pnd_adam_msg_dds__LowCmd_(ADAM_SP_NUM_MOTOR)
        self.low_state = pnd_adam_msg_dds__LowState_(ADAM_SP_NUM_MOTOR)
        self.hand_cmd = pnd_adam_msg_dds__HandCmd_()
        self.close_hand = np.array([500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500, 500], dtype=int)
        self.getstate_flag = False

    def Init(self):
        self.lowcmd_publisher_ = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher_.Init()

        self.hand_pub = ChannelPublisher("rt/handcmd", HandCmd_)
        self.hand_pub.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)
```

### 创建周期发布关节位置指令线程

程序启动后，立即启动线程，以 `Custom.control_dt_` (2.5ms) 定义的时间间隔调用 `Custom.LowCmdWrite`：

```python
def Start(self):
    self.lowCmdWriteThreadPtr = RecurrentThread(
        interval=self.control_dt_, target=self.LowCmdWrite, name="control"
    )
    self.lowCmdWriteThreadPtr.Start()
```

### 底层状态接收回调函数

当接收到话题 `rt/lowstate` 的底层状态消息时，程序会自动回调 `Custom.LowStateHandler` 函数：

- 记录当前各个关节所在位置，为闭环控制提供数据
- 隔一段时间打印IMU欧拉角

```python
def LowStateHandler(self, msg: LowState_):
    self.low_state = msg
    self.getstate_flag = True
    self.counter_ += 1
    if (self.counter_ % 500 == 0):
        self.counter_ = 0
        print(self.low_state.imu_state.ypr)
```

### 底层指令发送函数

`Custom.LowCmdWrite` 用于周期性发送底层指令，实现了：

- 阶段 1：将 Adam 机器人从任意初始关节位置，复位至预备状态 (程序启动的前 3秒)
- 阶段 2：采用踝关节 PR控制模式控制踝关节持续摆动 3秒
- 阶段 3：把左右手指打开，一直到程序运行终止

```python
def LowCmdWrite(self):
    if(self.getstate_flag):
        self.time_ += self.control_dt_

        if self.time_ < self.duration_:
            # 阶段1: 从当前位置平滑回零位
            for i in range(ADAM_PRO_NUM_MOTOR):
                ratio = np.clip(self.time_ / self.duration_, 0.0, 1.0)
                self.low_cmd.motor_cmd[i].mode = 1   # 1:Enable, 0:Disable
                self.low_cmd.motor_cmd[i].tau = 0.
                self.low_cmd.motor_cmd[i].q = (1.0 - ratio) * self.low_state.motor_state[i].q
                self.low_cmd.motor_cmd[i].dq = 0.
                self.low_cmd.motor_cmd[i].kp = Kp[i]
                self.low_cmd.motor_cmd[i].kd = Kd[i]
            for i in range(12):
                self.hand_cmd.position[i] = self.close_hand[i]

        elif self.time_ < self.duration_ * 2:
            # 阶段2: 踝关节摆动
            max_P = np.pi * 30.0 / 180.0
            max_R = np.pi * 10.0 / 180.0
            t = self.time_ - self.duration_
            L_P_des = max_P * np.sin(2.0 * np.pi * t)
            L_R_des = max_R * np.sin(2.0 * np.pi * t)
            R_P_des = max_P * np.sin(2.0 * np.pi * t)
            R_R_des = -max_R * np.sin(2.0 * np.pi * t)
            self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnklePitch].q = L_P_des
            self.low_cmd.motor_cmd[ADAMJointIndex.LeftAnkleRoll].q = L_R_des
            self.low_cmd.motor_cmd[ADAMJointIndex.RightAnklePitch].q = R_P_des
            self.low_cmd.motor_cmd[ADAMJointIndex.RightAnkleRoll].q = R_R_des

        elif self.time_ < self.duration_ * 3:
            # 阶段3: 手指打开
            for i in range(ADAM_PRO_NUM_MOTOR):
                self.low_cmd.motor_cmd[i].mode = 1
                self.low_cmd.motor_cmd[i].tau = 0.
                self.low_cmd.motor_cmd[i].q = 0
                self.low_cmd.motor_cmd[i].dq = 0.
                self.low_cmd.motor_cmd[i].kp = Kp[i]
                self.low_cmd.motor_cmd[i].kd = Kd[i]
            for i in range(12):
                self.hand_cmd.position[i] = self.open_hand[i]

        self.hand_pub.Write(self.hand_cmd)
        self.lowcmd_publisher_.Write(self.low_cmd)
```

### 关键参数

- **控制周期**: 2.5ms (400Hz)
- **复位时间**: 3秒
- **电机模式**: `mode = 1` (Enable), `mode = 0` (Disable)
- **控制公式**: `tau = tau_ff + kp * (q_des - q) + kd * (dq_des - dq)`
