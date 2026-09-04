# AimDK X2 — `aimdk_msgs` field schemas used by `device.py`

Transcribed verbatim from the vendor SDK's `aimdk_msgs/interface/robot/**/*.msg`/`*.srv`
sources (in `aimdk-aarch64-a424add7-artifacts.zip`), so future maintainers don't need to
re-download and re-extract the SDK to see field names/types. Only the messages/services this
driver actually uses are listed; the full interface package (168 `.msg` + 54 `.srv` files) is
vendored in `../aimdk_msgs-a424add7.zip`.

## Common

```
common/msg/MessageHeader.msg:  builtin_interfaces/Time stamp; string frame_id; uint32 sequence; builtin_interfaces/Time meas_stamp
common/msg/RequestHeader.msg:  builtin_interfaces/Time stamp
common/msg/CommonRequest.msg:  RequestHeader header
common/msg/CommonResponse.msg: ResponseHeader header; CommonState status; string message
common/msg/CommonTaskResponse.msg: ResponseHeader header; uint64 task_id; CommonState state
common/msg/CommonState.msg:    int32 value; UNKNOWN=0 SUCCESS=1 FAILURE=2 ABORTED=3 TIMEOUT=4
                                INVALID=5 IN_MANUAL=6 NOT_READY=100 PENDING=200 CREATED=300 RUNNING=400
```

Request-header convention is inconsistent across services (confirmed by direct read, not a
transcription error): some services take a `CommonRequest` under a field literally named
`request`, some take a `CommonRequest` under a field named `header`, and `SetMcAction`/
`SetMcPresetMotion` take a bare `RequestHeader` (only `.stamp`, no `frame_id`) under `header`.
`device.py`'s `AimdkNodes.request_header()` returns a `CommonRequest`; the two `RequestHeader`
cases set `.header.stamp` directly instead of using that helper.

## HAL (joints / hands)

```
hal/msg/JointCommand.msg:      string name; float64 position; float64 velocity; float64 effort;
                                float64 stiffness; float64 damping
hal/msg/JointCommandArray.msg: MessageHeader header; JointCommand[] joints
hal/msg/JointState.msg:        string name; float64 position; float64 velocity; float64 effort; uint16 error_code
hal/msg/HandCommand.msg:       string name; float64 position; float64 velocity; float64 acceleration;
                                float64 deceleration; float64 effort
hal/msg/HandCommandArray.msg:  MessageHeader header; HandType left_hand_type; HandCommand[] left_hands;
                                HandType right_hand_type; HandCommand[] right_hands
hal/msg/HandState.msg:         string name; float64 position; float64 velocity; float64 effort;
                                int32 state; int32 faultcode
hal/msg/HandStateArray.msg:    MessageHeader header; HandType left_hand_type; HandState[] left_hands;
                                HandTouchSensorData left_touch_sensors; HandType right_hand_type;
                                HandState[] right_hands; HandTouchSensorData right_touch_sensors
hal/msg/HandTouchSensorData.msg: uint8[36] palm_touch_data; uint8[36] back_of_hand_touch_data;
                                uint8[16] thumb_touch_data; uint8[16] index_finger_touch_data;
                                uint8[16] middle_finger_touch_data; uint8[16] ring_finger_touch_data;
                                uint8[16] little_finger_touch_data
                                (actual populated length varies by vendor part: 雷赛=36/palm+back,
                                12/fingers; Omnihands=16/fingers — arrays are padded/truncated by the SDK)
hal/msg/HandType.msg:          uint8 value; NONE=0x00 NIMBLE_HANDS=0x01 CLAW=0x02
                                LEISAI_NIMBLE_HANDS=0x03 ERROR=0xFF
hal/srv/GetAllJointState.srv:  req: CommonRequest request
                                resp: CommonResponse reponse[sic]; JointState[] head_joints;
                                JointState[] arm_joints; JointState[] waist_joints; JointState[] leg_joints
hal/srv/GetHandType.srv:       req: CommonRequest request
                                resp: CommonResponse reponse[sic]; HandType left_hands_type; HandType right_hands_type
hal/srv/SetPmuLed.srv:         req: CommonRequest request; string trace_id;
                                uint8 led_strip_mode (CONSTANT=0 BREATH=1 FLASH=2 FLOW=3 MAX=4);
                                uint8 r; uint8 g; uint8 b; int32 priority; bool reset_priority
                                resp: ResponseHeader header; uint16 status_code
```

## Sensors

```
hal/msg/TouchState.msg: uint8 event_type; uint32[8] data; uint32[8] threshold; bool[8] is_touched
hal/msg/PmuState.msg: string pmu_software_version; string pmu_hardware_version;
                     uint32 pmu_bool_status; float64 battery_voltage; float64 output_48v_voltage;
                     float64 output_12v_voltage; float64 pmu_temperature; float64 fan_speed;
                     uint8 fan_pecentage; ...
sensor_msgs/msg/CameraInfo: standard ROS 2 camera calibration and projection parameters
```

## Motion control (`mc`)

```
mc/action/McAction.msg:        int32 value; PASSIVE_DEFAULT=1 SOFT_EMERGENCY_STOP=2 DAMPING_DEFAULT=3
                                ZERO_TORQUE_DEFAULT=4 JOINT_DEFAULT=100 JOINT_FREEZE=101
                                STAND_DEFAULT=200 STAND_BODY_CONTROL=201 LOCOMOTION_DEFAULT=300
                                RUN_DEFAULT=301 LOCOMOTION_STEP=302 VR_REMOTE_CONTROLLER=400
                                SIT_DOWN_DEFAULT=2000 CROUCH_DOWN_DEFAULT=2002 LIE_DOWN_DEFAULT=2004
                                STAND_UP_DEFAULT=2005 ASCEND_STAIRS=2006 DESCEND_STAIRS=2008
mc/action/McActionCommand.msg: McAction action; string action_desc
mc/action/McActionInfo.msg:    McAction current_action; string action_desc; McActionStatus status
mc/action/srv/SetMcAction.srv: req: RequestHeader header; string source; McActionCommand command
                                resp: CommonResponse response
mc/action/srv/GetMcAction.srv: req: CommonRequest request
                                resp: ResponseHeader header; McActionInfo info
mc/motion/msg/McControlArea.msg: int32 value; NONE=0 LEFT_HAND=1 RIGHT_HAND=2 HEAD=4 WAIST=8
                                (bitmask for body-part locking during preset motions — distinct
                                from the leg/waist/arm/head split GetAllJointState returns)
mc/motion/msg/McLocomotionVelocity.msg: MessageHeader header; string source;
                                float64 forward_velocity; float64 lateral_velocity; float64 angular_velocity
mc/motion/msg/McPresetMotion.msg: int32 value; large named gesture enum, see PRESET_MOTIONS in device.py
mc/motion/msg/McInputSource.msg: string name; int32 priority; int32 timeout
mc/motion/srv/SetMcPresetMotion.srv: req: RequestHeader header; McControlArea area; McPresetMotion motion;
                                bool interrupt; string ani_path; uint64 play_timestamp
                                resp: CommonTaskResponse response
mc/motion/srv/SetMcInputSource.srv: req: CommonRequest request; McInputAction action; McInputSource input_source
                                resp: CommonTaskResponse response
mc/motion/srv/GetCurrentInputSource.srv: req: CommonRequest request
                                resp: CommonTaskResponse response; McInputSource input_source
```

## System / resources / interaction

```
sm/srv/GetSystemState.srv:     req: CommonRequest header
                                resp: CommonResponse header; string cur_state; SystemStatus curr_status
app_proxy/srv/GetRobotResources.srv: req: CommonRequest header
                                resp: CommonResponse header; RobotResource[] robot_resources
app_proxy/msg/RobotResource.msg: string resource_key; CurrentVersion current_version
app_proxy/srv/ExecuteActionResource.srv: req: CommonRequest header; string resource_key;
                                string resource_version; SlaveDevice[] slaves; string meta (JSON —
                                vendor's own literal "BODY_MONTION"/"ARM_MONTION" typo preserved
                                in device.py's LinkcraftPlugin, not introduced here)
                                resp: CommonResponse header
interaction/srv/PlayTts.srv:   req: CommonRequest header; PlayTtsRequest tts_req
                                resp: CommonResponse header; PlayTtsResponse tts_resp
interaction/msg/PlayTtsRequest.msg: string text; TtsPriorityLevel priority_level; uint32 priority_weight;
                                string domain; string trace_id; bool is_interrupted
interaction/msg/PlayTtsResponse.msg: string text; TtsPriorityLevel priority_level; uint32 priority_weight;
                                string domain; string trace_id; bool is_success; string error_message;
                                uint32 estimated_duration
interaction/msg/TtsPriorityLevel.msg: uint8 value; UNKNOWN=0x00 BACKGROUND_L1=0x01 SERVICE_L2=0x02
                                MISSION_L4=0x04 INTERACTION_L6=0x06 SYSTEM_L7=0x07 WARNING_L8=0x08 SAFETY_L10=0x0a
face_ui/srv/PlayEmoji.srv:     req: CommonRequest header; uint8 emotion_id (large enum, see EMOJI_IDS
                                in device.py); uint8 mode (ONCE=1 LOOP=2); int32 priority
                                resp: CommonResponse header; bool success; string message
interaction/srv/GetMicSourceRequest.srv: req: CommonRequest header
                                resp: CommonResponse header; uint32 mic_source
interaction/srv/SetMicSourceRequest.srv: req: CommonRequest header; uint32 mic_source (0=internal, 1=external)
                                resp: CommonResponse header
                                (field layout confirmed via py_examples/set_mic_source.py, not the
                                .srv source directly — vendor's own client retries up to 8x/0.25s
                                because "remote peer is NOT handled well by ROS")
app_proxy/srv/GetStoredMapByName.srv: req: std_msgs/Header header; string map_name
                                resp: int32 code; string map_path; MapInfo map_info; string map_id
                                (confirmed via py_examples/get_map.py — uses std_msgs/Header, NOT
                                CommonRequest, unlike every other service above)
```
