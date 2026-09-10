# AimDK X2 — topics & services catalog

Transcribed verbatim from the vendor SDK's own `topics_and_services` file (in
`aimdk-aarch64-a424add7-artifacts.zip`), so future maintainers don't need to re-download the
SDK just to see what's available. Vendor's literal `_5F_` in service names is the ROS 2
mangled-name encoding of `_` used by their own tooling — not a typo.

## Wired into this driver

| Catalog entry | Driver tool | Notes |
|---|---|---|
| `/aima/hal/imu/chest/state`, `/aima/hal/imu/torso/state` | `imu` | merged into one `data/json` stream |
| `/aima/mc/leg_odometry` | `leg_odometry` | `nav_msgs/msg/Odometry`, mirrored as `data/json`; locomotion/leg odometry, not SLAM localization |
| `/aima/hal/joint/hand/state` | `hand_state` (optional) | `HandStateArray`, includes touch sensors. Disabled by default: the verified X2 reports `HandType.NONE` and empty joint arrays on both sides. |
| `/aima/hal/sensor/touch_head` | `head_touch` | `TouchState`, confirmed publisher on the X2 unit |
| `/aima/hal/pmu/state` | `pmu_state` | `PmuState`, confirmed publisher on the X2 unit |
| `/aima/hal/joint/hand/command` | `hand_command` (optional) | Disabled by default with `hand_state`; enable only after dexterous-hand hardware is present. |
| `/aima/hal/joint/*/command` | `joint_command` | wildcard resolved to `leg`/`waist`/`arm`/`head` |
| `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` | `lidar` (optional) | `sensor/pointcloud`; disabled by default because the verified unit has no publisher. |
| `/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed` | `camera_rgb` | catalog documents `rgbd_head_front/rgb_image/compressed` instead, but on real hardware that topic has zero publishers — confirmed via `ros2 topic info` that `rgb_head_front_center` is what's actually live (30Hz); see below |
| `/aima/hal/sensor/rgb_head_front_center/camera_info` | internal calibration input for `camera_rgb_frame` | `CameraInfo`, confirmed publisher on the X2 unit; intentionally not exposed as a separate card |
| `/aima/hal/sensor/rgbd_head_front/depth_image` | `camera_depth` (optional) | zero publishers on real hardware, and no depth topic exists anywhere in the live `ros2 topic list` on this unit. Disabled by default pending vendor confirmation. |
| `/aima/mc/locomotion/velocity` | `locomotion` | |
| `/integrated_command` | `slam_control` (optional) | plain `std_msgs/String`, not a service; disabled by default because SLAM is an optional vendor module and this unit has no subscriber. |
| `/relocalization_pose` | `slam_control` (optional) | disabled with SLAM; this unit has no subscriber. |
| `/slam/lidar_odom` | `slam_pose` (optional) | disabled by default because this unit has no publisher. |
| `/aimdk_5Fmsgs/srv/GetAllJointState` | `joint_state` | |
| `/aimdk_5Fmsgs/srv/GetHandType` | `hand_state` (action `info`) | |
| `/aimdk_5Fmsgs/srv/GetMcAction`, `/aima/mc/common/state` | `mc_state`, `mc_mode` ACP confirmation | `GetMcAction` remains call-on-demand; the verified X2 also broadcasts `McCommonState`, whose `action_info.action_desc/status` confirms a requested mode has become active. |
| `/aimdk_5Fmsgs/srv/SetMcAction` | `mc_mode` | SDK enum is not a firmware capability list. This X2 rejected `STAND_UP_DEFAULT` and `ZERO_TORQUE_DEFAULT` with `can not find action`; `PASSIVE_DEFAULT`/`STAND_DEFAULT` only acknowledged the request, while `DAMPING_DEFAULT` is the sole observed end-to-end working mode. The driver exposes only `DAMPING_DEFAULT` by default and confirms activation from `/aima/mc/common/state`. |
| `/aimdk_5Fmsgs/srv/SetMcPresetMotion` | `preset_motion` | |
| `/aimdk_5Fmsgs/srv/SetMcInputSource`, `GetCurrentInputSource` | `locomotion` (optional managed-source `register`/`disable`) | Firmware accepts built-in source names (`rc`, `vr`, `app_proxy`, `interaction`, `pnc`). Historical successful velocity bags use `source=rc`; default driver operation reuses it and does not add/disable a source. |
| `/aimdk_5Fmsgs/srv/GetSystemState` | `system_state` | |
| `/aimdk_5Fmsgs/srv/GetRobotResources` | `linkcraft_catalog` | |
| `/aimdk_5Fmsgs/srv/ExecuteActionResource` | `linkcraft` | |
| `/aimdk_5Fmsgs/srv/SetPmuLed` | `pmu_led` | |
| `/aimdk_5Fmsgs/srv/PlayTts` | `tts` | |
| `/aimdk_5Fmsgs/srv/PlayEmoji` | `emoji` | |
| `/aimdk_5Fmsgs/srv/SetMicSourceRequest`, `GetMicSourceRequest` | `mic_source` | |
| `/aimdk_5Fmsgs/srv/GetStoredMapByName` | `map_get` | |

## Available in the SDK but not yet wired

Left out of this driver's initial tool set to stay within the approved plan's scope — add a
new plugin in `device.py` if a use case comes up:

- `/agent/process_audio_output`, `/face_ui_proxy/status` — top-level status topics, purpose not
  fully documented in the SDK's public catalog.
- `/aima/hal/audio/capture`, `/aima/hal/audio/playback`, `/aima/hal/audio/focus_response`,
  `/aima/hal/audio/play_state` — raw audio I/O topics; this driver relies on `tts`/`PlayTts`
  instead of raw audio playback.
- `/aima/hal/sensor/rgb_head_rear/*`, `/aima/hal/sensor/stereo_head_front_{left,right}/*` —
  additional cameras beyond the front RGBD pair this driver exposes.
- `/aimdk_5Fmsgs/srv/AbandonAudioFocus`, `RequestAudioFocus`, `GetMute`, `SetMute`, `GetVolume`,
  `SetVolume` — audio focus/volume management.
- `/aimdk_5Fmsgs/srv/PlayAudioFile`, `PlayVideo`, `PlayVideoGroup` — media playback beyond TTS.
- `/aimdk_5Fmsgs/srv/SetAgentPropertiesRequest`, `MigrateSystemState` — agent/system config
  management, not part of the sensor/actuator tool surface.

## Explicitly unreleased (per vendor docs, marked "待发布")

- 故障与系统管理模块 (fault & system management)
- 视觉(recognition) beyond raw camera streams
- 开发者模式 (developer mode)

No placeholder tools were added for these.
