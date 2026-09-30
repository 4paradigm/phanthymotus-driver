# AgiBot A3 (AimDK v3.2) — interfaces catalog

Transcribed from the AimDK A3-Ultra developer guide
(https://open.agibot.com/docs/aimdk/a3-ultra/v3_2/dev_guide). Unlike X2's
`aimdk_msgs` ROS service package, the A3 exposes its whole control surface as
**HTTP JSON RPC** (POST `http://{host}:{port}/rpc/aimdk.protocol.{Service}/{Method}`)
plus **ROS 2 Jazzy topics** (Fast DDS, `ros2_plugin_proto` for protobuf carriers) —
so this catalog is organized by compute unit / port, not by ROS package.

Card naming is aligned with x-humanoid/tianyi2.0, robotera/q5_bundle, unitree/g1
and noetix/bumi: `lidar_cloud` (g1), `battery`/`estop` (tianyi2.0/q5_bundle),
`loco` (noetix/bumi 腿式同款 — 轮式底盘卡才叫 base_drive/chassis_raw，A3 是
双足人形), `arm_control`/`hand_control`/`head_control`/
`waist_control` (tianyi2.0/q5_bundle style). The `resource_list` card is
dissolved into `list` actions on the play cards; `arm_compliance` is absorbed
by `arm_control`; `wakeup` is dropped (AimDK v3.2 exposes only wake-word
events, no raw mic stream — useless without audio access).

Compute units (fixed IPs, subnet 10.42.10.x; third-party host joins the same
subnet with `ROS_DOMAIN_ID=232` and the robot's `ros_dds_configuration.xml`
copied from HDU `/agibot/software/v0/entry/cfg/`):

| Unit | IP | Role |
|---|---|---|
| HDU | 10.42.10.10 | interaction (TTS/agent/audio/mic), resources |
| ADU | 10.42.10.11 | SLAM (mapping/localization/PnC/skills) |
| MDU | 10.42.10.12 | motion control, HDS alerts |

## Wired into this driver

### HTTP JSON RPC (all POST)

| Service@unit:port | Method | Driver tool | Notes |
|---|---|---|---|
| MotionControlActionService@MDU:56322 | SetAction | `mc_mode` | `command.action` = `MotionControlAction_{DAMPING,GET_UP,LIE_DOWN,PASSIVE}`; fixed FSM enforced first (non-standing states only accept get_up, MOTION accepts damping/lie_down/passive), illegal transitions rejected with a suggestion, then cross-checked against runtime GetAvailableActions |
| MotionControlActionService@MDU:56322 | GetAction | `mc_mode` (`get_state`) | |
| MotionControlActionService@MDU:56322 | GetAvailableActions | `mc_mode` (`available`) | |
| MotionControlMotionService@MDU:56322 | Enable/Disable/CheckArmCompliance | `arm_control` (`compliance_enable`/`compliance_disable`/`compliance_check`) | all take `{}` |
| MotionCommandService@MDU:56444 | SendMotionCommand | `motion_play` | `motion_id` is the motion file's **absolute path**; `cmd_end` auto-returns to initial pose |
| HDSService@MDU:50587 | GetAlertList | `alerts` | hard doc limit ≤0.2 Hz — driver enforces a monotonic 5 s cooldown |
| TTSService@HDU:59301 | PlayTTS | `tts` | `text` ≤1024 bytes, `priority_level` `INTERACTION_L6` |
| TTSService@HDU:59301 | PlayMediaFile | `tts` (`play_media`) | |
| TTSService@HDU:59301 | GetAudioStatus | `tts` (`status`) | by `trace_id` |
| TTSService@HDU:59301 | StopTTSTraceId | `tts` (`stop_trace_id`) | |
| AgentControlService@HDU:59301 | SetVoiceEnable / GetVoiceEnable | `interaction` | |
| AgentControlService@HDU:59301 | SetAgentPropertiesRequest | `interaction` (`mode_*`) | properties `{"2": "normal"\|"only_voice"}` — needs reboot |
| HalAudioService@HDU:59301 | SetMicSourceRequest / GetMicSourceRequest | `interaction` (`mic_*`) | 0=internal (v3.2 hardware BUG — avoid), 1=external |
| HalAudioService@HDU:56666 | GetAudioVolume / SetAudioVolume | `audio` (`get_volume`/`set_volume`/`mute`/`unmute`) | `type` `SPEAKER_BUILT_IN`; driver hard-caps at 70 (>70 risks damage) |
| HalAudioService@HDU:56666 | PlayFile / StopPlay | `audio` (`play`/`stop_play`) | raw file playback (TTS alternative) |
| ResourceService@HDU:51049 | GetResourceList | `list` actions on `motion_play` (motion) / `face_play` (emoticon) / `audio` (audio) / `skill_play` (skill, `list_offring_work` for offring_work) + `controlled_spatial` (`list_maps`, map class) | resource_list card dissolved into the play cards |
| MappingService@ADU:50807 | StartMapping / StopMapping | `controlled_spatial` (`start_mapping`/`stop_save`/`stop_discard`) | StopMapping with `map_name` = SAVING_MAP |
| MappingService@ADU:50807 | Get2DWholeMap | `controlled_spatial` (`get_map`) / `spatial_map` | resolution/origin/occupancy grid; `spatial_map` republishes it as a `sensor/mapping` point cloud |
| MappingService@ADU:50807 | GetStoredMapNames / GetCurrentWorkingMap / RenameMap | `controlled_spatial` (`list_maps`/`current_map`/`rename_map`) | |
| LocalizationService@ADU:50807 | GetTopoMsgs | — | not exposed as a card (the 2D grid covers it) |
| PncService@ADU:53176 | ActionNaviToPose / ActionNaviToPoint / … | `controlled_spatial` (`navi_to_goal`/`navi_to_pose`/`linear_to_goal`/`linear_to_pose`/`move_forward`/`spin_turn`) | `task_id=0` auto-assigns; driver remembers the assigned id |
| PncService@ADU:53176 | ActionGetState | `controlled_spatial` (`nav_state`) | |
| PncService@ADU:53176 | ActionCancelTask / ActionEmergencyStop / … | `controlled_spatial` (`cancel`/`pause`/`resume`) | |
| SLAMRelocalizationService@ADU:50583 | SLAMStartNormalRelocalization | `controlled_spatial` (`start_relocalization`) | required before navigation |
| SLAMRelocalizationService@ADU:50583 | SLAMStopNormalRelocalization | `controlled_spatial` (`stop_relocalization`) | optional `reloc_pose` |
| SkillPilotService@ADU:50583 | AutoCharging | `auto_charging` | command START/STOP/RESET; trigger AGENT=1 |
| SkillPilotService@ADU:50583 | SkillPackage | `skill_play` | Start/Pause/Continue/Stop by `session_id`; `state` reads the mirrored skill_status stream |

### ROS 2 topics (robot domain 232, BEST_EFFORT QoS)

| Topic | Message | Driver tool | Notes |
|---|---|---|---|
| `/motion/control/locomotion_velocity` | RosMsgWrapper (pb `LocomotionVelocity`) | `loco` | forward/lateral normalized −1..1, angular deg/s (±57.3 = 1 rad/s); MOTION mode only; duration-bounded with ACP |
| `/motion/control/move_waist` | RosMsgWrapper (pb `MoveWaist`) | `waist_control` | waist_pitch/waist_yaw rad, waist_height m |
| `/skill/pilot/face/play` | RosMsgWrapper (pb `FacePlayInfo`) | `face_play` | e_path/e_id/repeat/priority(440)/is_stop |
| `/motion/control/arm_joint_command` | sensor_msgs/JointState | `arm_control` (`send`) | 100 Hz ≤30 ms gap, velocity/effort = 0, ≤4 rad/s |
| `/motion/control/hand_joint_command` | sensor_msgs/JointState | `hand_control` | 0..2000 per finger; frame_id = AgiHand/O10Hand |
| `/motion/control/neck_joint_command` | sensor_msgs/JointState | `head_control` | head_yaw/head_pitch |
| `/motion/control/arm_joint_state` | sensor_msgs/JointState | `joints` (`arm`) | mirrored to core as JSON |
| `/motion/control/hand_joint_state` | sensor_msgs/JointState | `joints` (`hand`) | frame_id carries hand type |
| `/motion/control/neck_joint_state` | sensor_msgs/JointState | `joints` (`neck`) | |
| `/hal/neck_middle_livox_lidar/pointcloud` | sensor_msgs/PointCloud2 | `lidar_cloud` | neck Livox |
| `/ros2/body_drive/pelvis_imu/data` | sensor_msgs/Imu | `imu` | pelvis IMU |
| `/ros2/body_drive/torso_imu/data` | sensor_msgs/Imu | `imu` | torso IMU (merged into one card) |
| 9× camera topics (see `device.py` `CAMERA_TOPICS`) | sensor_msgs/Image | `camera` | head×3 fisheye, chest D457 rgb+depth, waist D415 rgb+depth, wrist×2 D405; config picks the streams (default includes chest depth) — RGB re-encoded to `image/jpeg`, depth to `image/depth-zlib` (README_dev renderer formats; raw mirror would be 614KB/frame) |

### Protobuf-carrier streams (need `a3_aimdk` wheel to decode)

| Topic | pb type | Driver tool |
|---|---|---|
| `/aima/bms/data/pb_3Aaimdk_2Eprotocol_2EBmsStateChannel` | BmsStateChannel | `battery` |
| `/hal_state/emergency/pb_3Aaimdk_2Eprotocol_2EEmergencyStateChannel` | EmergencyStateChannel | `estop` |
| `/skill/pilot/skill_status` | SkillStatus | `skill_play` (`state`) |

## Documented but not wired

- Body posture / head-pose "attention" HTTP APIs beyond the four SetAction modes
  (GetAvailableActions returns the authoritative live list at runtime).
- H265 CompressedVideo foxglove streams for every camera — the raw
  `sensor_msgs/Image` mirrors are preferred, re-encoded to JPEG (RGB) and
  zlib uint16 (depth) on the fly (H265 decode support is inconsistent on the
  consumer side, and `image/raw` has no dashboard renderer at all).
- MappingService real-time progress callbacks (`no_realtime_data: true` is sent
  when starting mapping; status is polled via the same RPCs).
- `/agent/wakeup/pb_3Aaimdk_2Eprotocol_2EWakeUpResult` (WakeUpResult pb): only
  carries wake-word events, no raw mic stream — a wakeup card would be useless
  without audio access, so it is intentionally dropped.
- The A3's 12 leg joints: no dev-interface topic exposes them individually
  (naming is internal to the MC firmware), so they appear in the URDF only.

## Wheel / vendor artifacts (NOT vendored in this repo)

- `prebuilt/a3_aimdk-3.1.0-py3-none-any.whl` — protobuf definitions
  (`aimdk.protocol_pb2`). Ships in the robot's AimDK 开发包 (dev kit); the image
  CMD pip-installs it from the `/agibot-devkit` mount at container start.
- `prebuilt/ros2_plugin_proto_aarch64/` — the ROS 2 interface package providing
  `ros2_plugin_proto/msg/RosMsgWrapper`; its `local_setup.bash` is sourced by the
  image CMD from the same mount.
- Without these the driver degrades gracefully: pb payloads fall back to JSON
  bytes in `RosMsgWrapper.data`, and the pb-decoded stream cards
  (battery/estop + skill_play's `state` query) are withheld.

## URDF

`../resource/a3_ultra.urdf` is hand-written from dev guide §1.3 (关节运动空间)
and §7.1.4/§7.1.6/§7.1.7 topic tables — AgiBot publishes no A3 URDF download
(unlike X2's `X2_URDF` zip). Joint tree + limits only; no visual/collision
meshes. Note the `/motion/control/arm_joint_command` limits enforced in
`device.py` `ARM_JOINT_LIMITS` are stricter than §1.3 for some joints.
