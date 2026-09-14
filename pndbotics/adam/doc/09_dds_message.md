# DDS消息定义

> Source: https://wiki.pndbotics.com/robot/dds_message/

## DDS 接口

底层通信提供了用户端PC与机器人之间的数据交互功能。

- 订阅话题 `rt/lowstate`（类型：`pnd_adam.msg.dds_.LowState_`） 获取 Adam 当前状态。
- 订阅话题 `rt/handstate`（类型：`pnd_adam.msg.dds_.HandState_`）获取手指当前状态。
- 发布话题 `rt/lowcmd`（类型：`pnd_adam.msg.dds_.LowCmd_`） 控制全身关节电机（不含灵巧手）、电池等设备。
- 发布话题 `rt/handcmd`（类型：`pnd_adam.msg.dds_.Handcmd_`） 控制手指关节电机。

---

## 接口说明

采用 DDS API 里介绍的方法订阅或发布话题。话题信息存储在由 IDL 定义的结构体中，常用结构体有：

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

## DDS消息定义

### 下发消息：`rt/lowcmd`（身体期望位置）

| 字段 | 类型 | 维度 | 含义 |
|------|------|------|------|
| `mode_pr` | `uint8` | 1 | 模式控制 |
| `motor_cmd` | `MotorCmd_` | * | 变长 (见下) |
| `reserve` | `uint32` | 1 | 保留字段 |

**`motor_cmd` 说明**：其长度根据本机型号而定：
- **Adam Pro**：31 维度执行器数据
- **Adam SP**：29 维度执行器数据
- **Adam Lite**：23 维度执行器数据

### 订阅消息：`rt/lowstate`（身体实际状态）

| 字段 | 类型 | 维度 | 含义 |
|------|------|------|------|
| `mode_pr` | `uint8` | 1 | 当前模式状态 |
| `tick` | `uint32` | 1 | 系统时间戳/滴答数 |
| `imu_state` | `IMUState_` | 9 | IMU 姿态数据 |
| `motor_state` | `MotorState_` | * | 变长 (见下) |
| `wireless_remote` | `short` | 40 | 遥控器手柄数据 (见下) |
| `battery_data` | `BatteryData_` | 1 | 电池电量与状态信息 |
| `reserve` | `uint32` | 1 | 保留字段 |

**`motor_state` 说明**：其长度根据本机型号而定：
- **Adam Pro**：31 维度执行器数据
- **Adam SP**：29 维度执行器数据
- **Adam Lite**：23 维度执行器数据

**`wireless_remote` 说明**：索引20-40为空，0-19分别对应手柄遥控器按键：
`"lx"`, `"ly"`, `"rx"`, `"ry"`, `"lt"`, `"rt"`, `"xx"`, `"yy"`, `"a"`, `"b"`, `"x"`, `"y"`, `"lb"`, `"rb"`, `"back"`, `"start"`, `"home"`, `"lo"`, `"ro"`

---

### 辅助结构体定义

#### 电池状态类型：`BatteryData_`

| 字段 | 类型 | 含义 |
|------|------|------|
| `timestamp_ms` | `int64` | 毫秒级时间戳 |
| `voltage` | `float32` | 电压 (V) |
| `current` | `float32` | 电流 (A) |
| `power` | `float32` | 功率 (W) |
| `wh_accumulated` | `float32` | 累计功耗 (Wh) |
| `status` | `string` | 电池状态描述 |

#### IMU数据类型：`IMUState_`

| 字段 | 类型 | 维度 | 含义 |
|------|------|------|------|
| `quaternion` | `float32` | 4 | 四元数 [w,x,y,z] |
| `gyroscope` | `float32` | 3 | 角速度 [x,y,z] |
| `accelerometer` | `float32` | 3 | 加速度 [x,y,z] |
| `ypr` | `float32` | 3 | 姿态角 [yaw,pitch,roll] |
| `temperature` | `int16` | 1 | 传感器温度 |

#### 单个关节控制命令：`MotorCmd_`

| 字段 | 类型 | 含义 |
|------|------|------|
| `mode` | `uint8` | 模式（默认0） |
| `q` | `float32` | 期望位置 (rad) |
| `dq` | `float32` | 期望速度 (rad/s) |
| `tau` | `float32` | 期望力矩 (Nm) |
| `kp` | `float32` | 比例增益（刚度） |
| `kd` | `float32` | 微分增益（阻尼） |
| `ki` | `float32` | 积分增益 |
| `reserve` | `uint32` | 保留字段 |

#### 单个关节状态反馈：`MotorState_`

| 字段 | 类型 | 含义 |
|------|------|------|
| `mode` | `uint8` | 当前模式 |
| `q` | `float32` | 实际位置 (rad) |
| `dq` | `float32` | 实际速度 (rad/s) |
| `ddq` | `float32` | 实际加速度 (rad/s^2) |
| `tau_est` | `float32` | 估计力矩 (Nm) |
| `state` | `uint32` | 电机运行状态码 |
| `reserve` | `uint32` | 保留字段 |

---

### 灵巧手接口

#### 下发消息：`rt/handcmd`（手指期望位置）

| 字段 | 类型 | 维度 | 含义 |
|------|------|------|------|
| `position` | `uint32` | 12 | 手部期望位置 |
| `reserve` | `uint32` | 1 | 保留字段 |

#### 订阅消息：`rt/handstate`（手指实际状态）

| 字段 | 类型 | 维度 | 含义 |
|------|------|------|------|
| `position` | `uint32` | 12 | 手部实际位置 |
| `reserve` | `uint32` | 1 | 保留字段 |
