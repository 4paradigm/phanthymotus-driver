# TRON2 pose presets

0.9.0 exposes one `pose` field: `salute`, `wave`, `welcome`. Chinese preset values, reset, side, speed, action and custom joints are not public options. Use arm_control.reset for a separate return to the original natural-arms-down home.

| Preset | Arms | Absolute native joint targets and order |
|---|---|---|
| salute | Right | Shoulder pitch -75°, roll -60°, yaw 0° leads elbow -120°; hold 1s; extend elbow then lower shoulders to original home |
| wave | Right | Shoulder -65°/-25°/0° leads elbow -100°; joint9 yaw -20°/+20° three cycles; verify stationary arm, right gripper 85→0mm once, verify closure; extend then return home |
| welcome | Both | Left shoulder -35°/+45°/+40°, right -35°/-45°/-40° lead elbows -65°; hold 1s; extend then return home |

Right-only presets preserve the actual left arm. Wrists become zero while bending; extension restores original home elbow/wrist values. Left gripper is unchanged. Only wave commands the right gripper, using speed/force grades10/10 (not Newtons); reported opening does not prove grasp. Millimetres retain the nominal, uncalibrated 0..85mm to 0..94raw mapping.

```json
{"pose":"wave"}
```

## Admission and automatic head centering

Validate the request, original home identity/firmware, all targets, generated path screening and complete ACP budget before the first command. Arms must be within1° of original home with complete fresh stationary feedback. Head feedback must be valid, stationary and within its mechanical range. Native VR head following must already be off, and known local teleop must be inactive; no robot mode or global setting is changed. Wave also checks the right gripper before sending head or arm motion.

If either native head axis is more than1° from zero, first send one MoveH to pitch0/yaw0 under the same parent pose owner. Require the exact positive ACK and advancing stationary target feedback before any arm or gripper command. Unknown ACK or missed head target aborts the preset and retains protection. Head centering is a real movement explicitly caused by executing pose, never by startup, discovery, info, planning or deployment. Invalid requests cannot initiate it. Already-centered heads receive no MoveH. ServoJ subsequently holds both head targets at zero.

The pose owns arm_l/arm_r/head throughout centering, trajectory, gripper and home. It shares ArmCompletion with arm_control and excludes other head/gripper calls. There is one parent action_id and one terminal tool=pose ACP callback. Gripper steps are private children of this exact owner, without independent ACP. This local ownership plus control-source checks does not establish a cross-process hardware arbiter; other motion sources must remain stopped.

## Continuous preset trajectory

The planner generates time-indexed C2 quintic joint ramps, with zero endpoint velocity/acceleration. Disjoint shoulder/elbow ramps overlap by25% while the shoulder starts first; same-joint reversals do not overlap or overshoot. There is no stationary-feedback dwell at each arm waypoint. Salute/welcome still include their requested1s hold. Wave retains stationary verification before its gripper barrier and at final home, then streams the return trajectory only after the gripper is verified.

Segment travel/duration planning remains12deg/s for raising/bending/return and24deg/s for wave yaw. With quintic easing, commanded peak speed is bounded by22.5deg/s and45deg/s respectively, and peak acceleration by60deg/s². Durations may extend to satisfy acceleration bounds. These are planned setpoints, not measured motor or Cartesian velocity guarantees. arm_control itself still accepts its joint/time pair; it has no global fixed joint speed.

The transport uses documented request_servoj with16native radians (14arm+2centered head), filter_ratio1.0, target cadence300Hz. Python/Jetson/WebSocket scheduling is not hard real time; delayed sends are never replayed in a catch-up burst. A gap over50ms cancels further output. Joint sampling uses the original single WebSocket reader at up to10Hz during pose and restores its idle cadence on every exit. Receipt age and advancing vendor timestamps must remain within350ms; regressions, frozen timestamps, non-ready robot status, vendor failure notification, head drift or arm tracking error above8° stop subsequent commands. Control-source/home checks repeat asynchronously during the stream. This supervision is not collision avoidance or a hardware stop guarantee.

ServoJ has **no positive ACK**. A `submitted` reply proves only socket submission and has arrival_verified=false, vendor_acknowledged=false. Terminal arrival requires at least3advancing samples over0.5s, error≤1°, positional stability≤0.2° and complete near-zero velocity feedback. Only verified final home and successful parent ACP delivery release ownership. Separate arm_control and move_head completion behavior is preserved.

## Failure, stop and verification scope

A lifecycle stop halts further ServoJ output; it does not prove physical standstill, so the parent retains protection until operator verification. It is not a hardware emergency stop. During a gripper step, stop finishes verification of the already-sent step and cancels later writes. Unknown sends, head centering failure, stale feedback, tracking/owner/source changes, gripper error and callback delivery failure do not trigger retries, forced home or replay.

The generated quintic/overlap path is sampled against the pinned TRON2 URDF and the existing joint-linear skeletal exclusion checks. The model lacks full collision geometry; collision_checked=false and physical_presets_verified=false remain explicit. Offline tests exercise paths, real local transport/ACP, head-first sequencing, invalid requests, source/feedback faults and owner retention. Deployment executes no gestures; actual tracking, scheduling and motion quality require separate field acceptance.

Manufacturer protocol: https://www.limxdynamics.com/en/documents/856848486581276672, sections3.6.2 and3.6.4.
