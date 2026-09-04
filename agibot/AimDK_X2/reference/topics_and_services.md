# AimDK X2 — topics & services catalog

Transcribed verbatim from the vendor SDK's own `topics_and_services` file (in
`aimdk-aarch64-a424add7-artifacts.zip`), so future maintainers don't need to re-download the
SDK just to see what's available. Vendor's literal `_5F_` in service names is the ROS 2
mangled-name encoding of `_` used by their own tooling — not a typo.

## Wired into this driver

| Catalog entry | Driver tool | Notes |
|---|---|---|
| `/aima/hal/imu/chest/state`, `/aima/hal/imu/torso/state` | `imu` | merged into one `data/json` stream |
| `/aima/hal/joint/hand/state` | `hand_state` | `HandStateArray`, includes touch sensors |
| `/aima/hal/joint/hand/command` | `hand_command` | |
| `/aima/hal/joint/*/command` | `joint_command` | wildcard resolved to `leg`/`waist`/`arm`/`head` |
| `/aima/hal/pmu/state` | — | not currently exposed as a tool (no card in the approved plan) |
| `/aima/hal/sensor/lidar_chest_front/lidar_pointcloud` | `lidar` | `sensor/pointcloud` |
| `/aima/hal/sensor/rgb_head_front_center/rgb_image/compressed` | `camera_rgb` (`interaction`) | `CompressedImage`; publisher confirmed on X2 hardware on 2026-09-04 |
| `/aima/hal/sensor/rgbd_head_front/rgb_image/compressed` | `camera_rgb` (`rgbd_front`) | `CompressedImage`; publisher confirmed on X2 hardware on 2026-09-04 |
| `/aima/hal/sensor/stereo_head_front_left/rgb_image/compressed` | `camera_rgb` (`stereo_left`) | `CompressedImage`; publisher confirmed on X2 hardware on 2026-09-04 |
| `/aima/hal/sensor/stereo_head_front_right/rgb_image/compressed` | `camera_rgb` (`stereo_right`) | `CompressedImage`; publisher confirmed on X2 hardware on 2026-09-04 |
| `/aima/hal/sensor/rgb_head_rear/rgb_image/compressed` | `camera_rgb` (`rear`) | `CompressedImage`; publisher confirmed on X2 hardware on 2026-09-04 |
| `/aima/hal/sensor/rgbd_head_front/depth_image` | `camera_depth` | `Image`; publisher confirmed on X2 hardware on 2026-09-04 |
| `/aima/mc/locomotion/velocity` | `locomotion` | |
| `/integrated_command` | `slam_control` | plain `std_msgs/String`, not a service |
| `/relocalization_pose` | `slam_control` | |
| `/slam/lidar_odom` | `slam_pose` | |
| `/aimdk_5Fmsgs/srv/GetAllJointState` | `joint_state` | |
| `/aimdk_5Fmsgs/srv/GetHandType` | `hand_state` (action `info`) | |
| `/aimdk_5Fmsgs/srv/GetMcAction` | `mc_state` | no broadcast topic exists, so this is call-on-demand |
| `/aimdk_5Fmsgs/srv/SetMcAction` | `mc_mode` | |
| `/aimdk_5Fmsgs/srv/SetMcPresetMotion` | `preset_motion` | |
| `/aimdk_5Fmsgs/srv/SetMcInputSource`, `GetCurrentInputSource` | `locomotion` (action `register`/`disable`) | |
| `/aimdk_5Fmsgs/srv/GetSystemState` | `system_state` | |
| `/aimdk_5Fmsgs/srv/GetRobotResources` | `linkcraft_catalog` | |
| `/aimdk_5Fmsgs/srv/ExecuteActionResource` | `linkcraft` | |
| `/aimdk_5Fmsgs/srv/SetPmuLed` | `pmu_led` | |
| `/aimdk_5Fmsgs/srv/PlayTts` | `tts` | |
| `/aimdk_5Fmsgs/srv/PlayEmoji` | `emoji` | |
| `/aimdk_5Fmsgs/srv/SetMicSourceRequest`, `GetMicSourceRequest` | `mic_source` | |
| `/aimdk_5Fmsgs/srv/GetStoredMapByName` | `map_get` | |

## `camera_rgb` multi-instance routing

`camera_rgb` is one sensor-card type with five selectable sources.  Its
`camera_source` setting has `scope: instance`, so two copies of the card can select different
cameras and run concurrently.  `interaction` is the default to preserve the previous card's
source selection.

Each card instance gets an independent core-domain output topic.  A canvas card whose ID is
`card-example-1` reports:

```text
/{namespace}/agibot_x2/camera_rgb/card_example_1  (image/jpeg)
```

`info(instance_id)` returns this path before streaming starts.  `start(instance_id)` creates
the selected robot-domain subscription and core-domain publisher; `stop(instance_id)` destroys
only that instance's resources.  Changing `camera_source` while an instance is running replaces
its subscription but keeps its output topic stable.

`camera_depth` remains a separate, single-instance card because its ROS message and output format
(`sensor_msgs/msg/Image`, `image/depth-z16`) differ from the compressed RGB streams.  The presence
of left/right image publishers alone is not evidence that those streams are synchronized,
rectified, or ready for stereo ranging.

## Available in the SDK but not yet wired

Left out of this driver's initial tool set to stay within the approved plan's scope — add a
new plugin in `device.py` if a use case comes up:

- `/agent/process_audio_output`, `/face_ui_proxy/status` — top-level status topics, purpose not
  fully documented in the SDK's public catalog.
- `/aima/hal/audio/capture`, `/aima/hal/audio/playback`, `/aima/hal/audio/focus_response`,
  `/aima/hal/audio/play_state` — raw audio I/O topics; this driver relies on `tts`/`PlayTts`
  instead of raw audio playback.
- `/aima/hal/sensor/touch_head` — head touch sensor, no dedicated tool yet.
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
