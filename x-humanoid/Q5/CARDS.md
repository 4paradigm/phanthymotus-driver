# Q5 功能卡片清单

> 星动纪元 Q5 轮式人形机器人 — 41 个 MCP 功能卡片
>
> ROS2 Domain: 211 | MCP Port: 15708 | 更新日期: 2026-08-06

---

## 一、状态卡（Sensor — 只读）

### 系统状态（7个）

| # | 卡片名 | 数据来源 | 作用 |
|---|--------|---------|------|
| 1 | `system_state` | `/xbot_state` + `/system/heartbeat` + `/system_monitor/status` + `/query_xbot_state`(service) | 机器人运行状态(INIT/READY/ACTIVE/ERROR)、CPU使用率、心跳；支持 `query_xbot_state` 主动查询 |
| 2 | `exceptions` | `/xbot_state` | 异常信息列表（模块/部位/等级/描述/解决方案），等级分为致命/严重/错误/警告/提示/信息 |
| 3 | `battery` | `/battery_state` + `/get_battery_param` `/Battery/GetVersion` `/Battery/ElectricState` | 电池电量百分比、电压、电流、温度、充放电状态、灯带颜色；支持 `get_param`/`get_version`/`get_electric_state`。续航4小时+，2.5小时快充 |
| 4 | `estop` | `/xbot_state` + `/emergency_service` | 急停状态 — 硬件按钮/遥控器急停，激活时切断电机电源 |
| 5 | `fault` | `/fault_array` + `/fault_aggregator/highest_level` + `/clear_errors` `/clear_hand_sensor`(service) | 关节模组故障错误码诊断，11种错误码。`clear` 操作调用 `/clear_errors` + `/clear_hand_sensor` |
| 6 | `temperature` | `/temperature` | 各关节/电机实时温度(℃) |
| 7 | `imu` | `/dynamic_joint_states` + `/xbot_state` | IMU姿态数据 — roll/pitch/yaw(°)、加速度(m/s²)、角速度(°/s) |

### 关节状态（1个）

| # | 卡片名 | 数据来源 | 作用 |
|---|--------|---------|------|
| 8 | `joints` | `/joint_states` | **全身46DOF关节状态汇总** — 所有关节位置(°)、速度(°/s)、扭矩(N·m)，按关节名索引 |

### 传感器（7个）

| # | 卡片名 | 数据来源 | 作用 |
|---|--------|---------|------|
| 9 | `camera_head` | RealSense D435i | RGB彩色图(base64 JPEG) + 深度图(uint16 raw) + 相机内参(K/D/R/P) |
| 10 | `lidar` | `/slam/map_cmap` | 360°混合固态激光雷达点云/地图数据 |
| 11 | `mic` | sounddevice 直访 | 麦克风阵列音频流，16kHz单声道PCM。录音请用 `audio` 卡片 |
| 12 | `asr` | `/speech/sentence_topic` | 语音识别结果，实时语音转文字 |
| 13 | `hand_sensor` | `/hand_sensor` | 灵巧手触觉+力+位置反馈 (XHAND1带触觉，XHand Lite可能为空) |
| 14 | `diagnostics` | `/diagnostics_agg` + `/diagnostics_nuc` + `/diagnostics_orin` + `/cpu_freq` | NUC/Orin整机诊断汇总、CPU频率 |
| 15 | `odometry` | `/wr1_base_drive_controller/odom` | 底盘里程计 — 位姿(x/y/yaw) + 速度(vx/vy/wz) |

### 输入设备（2个）

| # | 卡片名 | 数据来源 | 作用 |
|---|--------|---------|------|
| 16 | `joystick` | `/joy` | 手柄/遥控器 — 摇杆轴值 + 按键状态 |
| 17 | `teleop` | `/teleop_state` + `/teleoperation_health` + `/teleoperation_calib_state` + `/teleoperation/service` | 遥操作状态/健康/标定，支持 `start`/`stop` 启停遥操作服务 |

### 诊断/变换（2个）

| # | 卡片名 | 数据来源 | 作用 |
|---|--------|---------|------|
| 18 | `tf` | `/tf` | 实时坐标系变换树 |
| 19 | `motion` | `/motion_manager/motion_status` + `/motion_manager/transition_event` | 运动管理器状态；支持 `change_state`/`get_state`/`get_available_states`/`motion_request` |

---

## 二、动作卡（Actuator — 可写）

### 运动控制（8个）

| # | 卡片名 | 输出接口 | 作用 | 关键参数 |
|---|--------|---------|------|---------|
| 20 | `chassis` | `/wr1_base_drive_controller/cmd_vel` | **差速底盘控制** — 前后移动、原地旋转、停止 | `action`: move/rotate/stop/set_speed |
| 21 | `head` | `/wr1_controller/commands` | **头部2DOF控制** — 看向方向、指定角度、回正 | `action`: move_pos/look_at/reset；`yaw`: ±60°；`pitch`: ±30° |
| 22 | `arm` | `/wr1_controller/commands` + `/set_zero_pos` `/set_custom_home_position` | **双臂14DOF关节角度控制** — `home` 标定零位 | `action`: move_pos/reset/home |
| 23 | `arm_servo` | `/servo_poses` + `/get_pose` + `/get_servo_poses` | **双臂笛卡尔位姿控制(MPC)** — 末端XYZ+四元数 | `action`: move_pose/get_pose |
| 24 | `waist` | `/wr1_controller/commands` | **腰部旋转+腿部升降** | `action`: move_waist/move_height/set_zero |
| 25 | `hand` | `/hand_controller/commands` + `/Pause_EE_Retarget` `/Start_EE_Retarget` | **灵巧手预设手势+逐指控制** — 兼容XHand Lite/XHand1 | `action`: open_palm/fist/.../pause_ee_retarget/start_ee_retarget |
| 26 | `hand_low` | `/hand_controller/commands` | **灵巧手底层自由控制** | `action`: set_joint/reset |
| 27 | `brake` | `/control_brake` (Service) | **关节抱闸控制** | `action`: engage/release/status |

### 关节配置（1个）

| # | 卡片名 | 输出接口 | 作用 |
|---|--------|---------|------|
| 28 | `joint_config` | 29个 `/get_*` `/set_*` 关节参数服务 | **关节参数读写** — KP/KD/KI/摩擦力/力矩系数等 |

### 交互类（5个）

| # | 卡片名 | 输出接口 | 作用 |
|---|--------|---------|------|
| 29 | `tts` | `/speech/sentence_topic` | **语音合成** — 文字转语音播放 |
| 30 | `audio_player` | `/audio_player/play` (Action) | **音频文件播放** |
| 31 | `audio` | sounddevice 直访 | **声卡直接访问** — 录音/播放/设备列表 |
| 32 | `led` | `/led_control` | **机身指示灯带控制** |
| 33 | `chat` | 无（XOS界面控制） | 大模型语音对话开关 — 占位卡片 |

### 动作播放（2个）

| # | 卡片名 | 输出接口 | 作用 |
|---|--------|---------|------|
| 34 | `gesture_player` | `/gesture/upper_limb_play` | **上肢录制动作回放** |
| 35 | `action_player` | 已废弃 | 请使用 `gesture_player` |

### 导航和工具（4个）

| # | 卡片名 | 输出接口 | 作用 |
|---|--------|---------|------|
| 36 | `nav` | era_nav_msgs + `/navigate/*` services + `/initialpose` | **导航控制** |
| 37 | `mpc_controller` | `/mpc/*` + `/mobile_manipulator_mpc_reset` + `/sdk/*` | **MPC算法/SDK启停控制** |
| 38 | `bag_record` | ros2 bag record | **ROS2 Bag录制** |
| 39 | `bag_playback` | ros2 bag play | **ROS2 Bag回放** |

### 遥控/电源（2个）

| # | 卡片名 | 输出接口 | 作用 |
|---|--------|---------|------|
| 40 | `remote_control` | `/send_remote/command` + `/remote_control/trigger_play` | **遥控指令收发** |
| 41 | `power` | `/shutdown_service` + `/ethercat_emergency` | **电源管理** — 关机 / EtherCAT急停（请谨慎使用） |

---

## 三、资源卡（Resource）

| # | 卡片名 | 作用 |
|---|--------|------|
| — | `model` | 返回 Q5 URDF 骨架模型文件，用于 3D 可视化 |

---

## 四、默认禁用的卡片

| 卡片 | 原因 |
|------|------|
| `chat` | Q5 无 `/chat/enable` ROS2 接口，对话功能通过 XOS 后台控制 |
| `action_player` | Q5 无 `/action/cmd` topic，已废弃，迁移到 `gesture_player` |

---

## 五、卡片调用方式

所有卡片通过 MCP JSON-RPC 协议调用：

```json
{
  "jsonrpc": "2.0",
  "id": 1,
  "method": "tools/call",
  "params": {
    "name": "<卡片名>",
    "arguments": {
      "action": "<动作名>",
      "<参数名>": "<值>"
    }
  }
}
```

### 调用示例

```bash
# 查看所有卡片
curl -s http://localhost:15708/mcp \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}'

# 获取电池状态
curl -s http://localhost:15708/mcp \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"battery","arguments":{}}}'

# 获取里程计
curl -s http://localhost:15708/mcp \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"odometry","arguments":{}}}'

# 头部看向右边
curl -s http://localhost:15708/mcp \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"head","arguments":{"action":"look_at","target":"right"}}}'

# 张开右手
curl -s http://localhost:15708/mcp \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"hand","arguments":{"action":"open_palm","side":"right"}}}'

# 底盘前进
curl -s http://localhost:15708/mcp \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"chassis","arguments":{"action":"move","vx":0.5}}}'

# 语音播报
curl -s http://localhost:15708/mcp \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/call","params":{"name":"tts","arguments":{"action":"speak","text":"你好"}}}'
```

---

## 六、卡片速查表

| 类别 | 卡片名 | 类型 | 一句话 |
|------|--------|------|--------|
| 状态 | `system_state` | sensor | 机器人运行状态+CPU，支持query_xbot_state |
| 状态 | `exceptions` | sensor | 异常/错误信息 |
| 状态 | `battery` | sensor | 电量/电压/充放电+参数/版本 |
| 状态 | `estop` | sensor | 急停状态 |
| 状态 | `fault` | sensor | 关节故障错误码，clear调用/clear_errors |
| 状态 | `temperature` | sensor | 关节电机温度 |
| 状态 | `imu` | sensor | 姿态角+加速度 |
| 状态 | `joints` | sensor | 全身46DOF关节 |
| 状态 | `camera_head` | sensor | RGB-D相机 |
| 状态 | `lidar` | sensor | 激光雷达 |
| 状态 | `mic` | sensor | 麦克风阵列 |
| 状态 | `asr` | sensor | 语音识别 |
| 状态 | `hand_sensor` | sensor | 灵巧手触觉/力反馈 |
| 状态 | `diagnostics` | sensor | NUC/Orin诊断+CPU频率 |
| 状态 | `odometry` | sensor | 底盘里程计(pose+twist) |
| 状态 | `joystick` | sensor | 手柄摇杆+按键 |
| 状态 | `teleop` | sensor | 遥操作状态/健康，支持start/stop |
| 状态 | `tf` | sensor | 坐标系变换 |
| 状态 | `motion` | sensor | 运动管理器状态+切换+请求 |
| 动作 | `chassis` | actuator | 底盘前后+旋转 |
| 动作 | `head` | actuator | 头部yaw/pitch |
| 动作 | `arm` | actuator | 双臂14DOF角度+home标定 |
| 动作 | `arm_servo` | actuator | 双臂笛卡尔MPC+get_pose |
| 动作 | `waist` | actuator | 腰旋转+腿升降 |
| 动作 | `hand` | actuator | 灵巧手预设手势+EE重定向 |
| 动作 | `hand_low` | actuator | 灵巧手底层控制 |
| 动作 | `brake` | actuator | 关节抱闸控制 |
| 动作 | `joint_config` | actuator | 关节参数读写(29个服务) |
| 动作 | `tts` | actuator | 语音合成 |
| 动作 | `audio_player` | actuator | 音频文件播放 |
| 动作 | `audio` | actuator | 声卡录音/播放 |
| 动作 | `led` | actuator | 灯带控制 |
| 动作 | `chat` | actuator | 对话开关(占位) |
| 动作 | `gesture_player` | actuator | 预录动作回放 |
| 动作 | `nav` | actuator | 导航控制+服务 |
| 动作 | `mpc_controller` | actuator | MPC/SDK启停+reset_mmpc |
| 动作 | `bag_record` | actuator | Bag录制 |
| 动作 | `bag_playback` | actuator | Bag回放 |
| 动作 | `remote_control` | actuator | 遥控指令收发 |
| 动作 | `power` | actuator | 关机/EtherCAT急停 |
| 资源 | `model` | resource | URDF模型 |
