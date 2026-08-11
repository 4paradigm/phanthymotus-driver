# Bumi Edu Max Driver

Phanthy Motus driver bundle for the Bumi Edu Max humanoid robot (21DOF, 双足).
The driver bridges Bumi SDK (CycloneDDS) capabilities to Agent Core via MCP tools.

## 架构

```
算力板 (Jetson Orin)                          运控板 (RK3576)
┌──────────────────────────┐                 ┌──────────────────────┐
│  bumi-driver (this)      │   DDS 500Hz     │  运控程序             │
│  ├─ main.py (MCP server) │ ←──────────→    │  ├─ 原厂运控算法      │
│  ├─ device.py (插件)      │   CycloneDDS    │  └─ 电机控制接口      │
│  ├─ media_plugins.py      │                 │                        │
│  └─ BumiSDK (pybind11)    │                 │  硬件层 (21 电机+IMU)  │
└──────────┬───────────────┘                 └──────────────────────┘
           │ HTTP JSON-RPC (MCP)
           ▼
     Agent Core
```

## 通信

- **DDS (CycloneDDS 0.11)**: 算力板 ←→ 运控板，500Hz 状态推送 + 控制指令
- **MCP (HTTP JSON-RPC)**: 算力板 ←→ Agent Core，工具暴露 + 传感器 topic

## 卡片清单

### 传感器 (18 张)
| 卡片 | 插件 | 来源 |
|---|---|---|
| joints | StatePlugin | `get_joint_state()` |
| imu | StatePlugin | `get_imu_data()` |
| battery | StatePlugin | `get_robot_bms_data()` |
| estop | StatePlugin | `get_mode()` (workmode=26/30) |
| robot_faults | StatePlugin | 电机 error 聚合 |
| model | StatePlugin | URDF |
| camera_head | CameraPlugin | RealSense D435i RGB |
| camera_depth | DepthCameraPlugin | RealSense D435i Z16 |
| camera_pointcloud | PointCloudPlugin | RealSense D435i 点云 |
| remote_event | RemoteEventPlugin | `from_dds_get_joydata()` |
| motors | MotorsPlugin | `get_joint_state()` (2Hz) |
| joystick_direct | JoystickDirectPlugin | AoLionDriver (USB) |
| media_system_status | MediaSystemStatusPlugin | `get_system_status()` |
| media_system_error | MediaSystemErrorPlugin | `get_system_error()` |
| microphone | MicrophonePlugin | `get_audio_capture_data()` |
| speaker_audio | SpeakerAudioPlugin | `get_audio_playback_data()` |
| video_capture | VideoCapturePlugin | `get_video_capture_data()` |
| video_desensed | VideoDesensedPlugin | `get_video_capture_desensed_data()` |

### 执行器 (23 张)
| 卡片 | 插件 | 协议 |
|---|---|---|
| stand | StandPlugin | `publish_cmd(START/SWITCH)` |
| walk | WalkPlugin | `publish_cmd(x,y,z,WALK)` |
| arm_gesture | ArmGesturePlugin | `publish_cmd(SWING/SHAKE/CHEER/TEAR)` |
| dance | DancePlugin | `publish_cmd(DANCE/DANCE1/DANCE2)` |
| teach | TeachPlugin | `publish_cmd(STARTTEACH/SAVETEACH/PLAYTEACH)` |
| fall_recovery | FallRecoveryPlugin | `publish_cmd(FALLTOSTAND/STANDTOFALL)` |
| wakeword | WakewordPlugin | `MediaController.wakeup/sleep/restart` |
| volume | VolumePlugin | `get/set_volume()` |
| timeout_config | TimeoutConfigPlugin | `get/set_timeout()` |
| beep_switch | BeepSwitchPlugin | `get/set_audio_cue_enable()` |
| audio_routing | AudioRoutingPlugin | 7 个路由开关 |
| audio_capture_control | AudioCaptureControlPlugin | `pause/resume_audio_capture()` |
| audio_playback_control | AudioPlaybackControlPlugin | `pause/resume_audio_playback()` |
| video_capture_control | VideoCaptureControlPlugin | `pause/resume_video_capture()` |
| external_audio_input | ExternalAudioInputPlugin | `publish_external_audio_stream()` |
| external_audio_output | ExternalAudioOutputPlugin | `publish_external_audio_playback_stream()` |
| external_video_input | ExternalVideoInputPlugin | `publish_external_video_stream()` |
| rl_policy (可选) | RLPolicyPlugin | LowController + ONNX |

## 目录结构

```
bumi/
├── main.py              # MCP HTTP Server + 插件加载
├── device.py            # 传感器/执行器插件 (StatePlugin, CameraPlugin, ...)
├── media_plugins.py     # MediaController 插件 (语音/音视频)
├── config.yaml          # 插件启用配置 + DDS 配置
├── driver.yaml          # 驱动元信息
├── requirements.txt
├── Dockerfile
├── config/
│   └── dds.xml          # CycloneDDS 配置 (运控板 192.168.55.102)
├── resource/
│   └── bumi_model.urdf  # URDF 骨架模型
├── build/               # pybind11 编译产物 (.so)
└── deploy/
    └── service.yml      # Docker Compose 部署
```

## 开发

### 前置条件

1. Bumi SDK (`noetix_sdk_bumi`) 已编译，`build/` 下有 `highcontrol_py.so`、`lowcontrol_py.so`、`mediacontrol_py.so`
2. 算力板 SSH 可达: `ssh noetix@192.168.55.101`

### 本地运行

```bash
python3 main.py
```

### Docker 部署

```bash
docker build -t bumi-driver .
docker compose -f deploy/service.yml up -d
```

## 安全提示

调试过程中，请全程使用机器人吊架，以免错误程序失控造成损失。
