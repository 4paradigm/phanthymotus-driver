# Booster Robotics C++ SDK — reference notes

Full source was downloaded via `codeload.github.com` (not `git clone` — the task asked to avoid
the slow full-history clone; `raw.githubusercontent.com` was also unreachable from the dev
environment, so `codeload.github.com`/`api.github.com` were used instead) and is mirrored on COS
rather than committed to this repo (git isn't a great home for an 18MB binary blob):

```bash
curl -fsSL -o booster_robotics_sdk-main.zip \
  https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/booster/booster_robotics_sdk-main.zip
curl -fsSL -o booster_robotics_sdk_ros2-main.zip \
  https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/booster/booster_robotics_sdk_ros2-main.zip
```

`booster_robotics_sdk_ros2-main.zip` is the ROS2 message/service package
(`BoosterApiReqMsg`/`RespMsg`, `AgentService`, `RpcService`, plus IDL for
`ImuState`/`LowCmd`/`LowState`/`MotorCmd`/`MotorState`/`Odometer`/etc — build with `colcon`,
depends on `rclcpp`/`ament_cmake`/`rosidl_default_generators`).

This driver does **not** build against either repo directly — `boosteros`/
`booster_robotics_sdk_python` (both pip-installable, see `../python-sdk/README.md`) are the
actual runtime dependency. These zips exist purely as offline reference for the underlying
RPC/DDS surface (useful if a future card needs something `boosteros` doesn't expose).

## Repo layout (`booster_robotics_sdk`)

```
example/high_level/   b1_loco_example_client.cpp, b1_arm_sdk_example.cpp, b1_7dof_arm_sdk_example.cpp
example/low_level/    b1_low_sdk_example.cpp, low_level_publisher/subscriber.cpp,
                      battery_state_subscriber.cpp, odometer_example.cpp,
                      low_level_hand_data_subscriber.cpp, b1_7dof_arm_low_sdk_example.cpp
include/booster/common/dds/       DDS entity/topic-channel/callback plumbing
include/booster/idl/              IDL message headers: ai/, audio/, b1/ (LowCmd, LowState,
                                   MotorCmd/State, ImuState, Odometer, BatteryState,
                                   FallDownState, RemoteControllerState, Hand*, Kick,
                                   LightControlMsg, RobocupBehaviorStatus, ...),
                                   camera/, geometry_msgs/, builtin_interfaces/
include/booster/robot/            High-level client headers:
    ai/            api.hpp, client.hpp, const.hpp
    audio/         audio_manager.h, audio_player.h, audio_recorder.h, audio_localizer.h, ...
    b1/            b1_loco_client.hpp, b1_loco_api.hpp, move_controller.hpp,
                   arm_controller.hpp, b1_api_const.hpp
    camera/        camera_client.hpp, camera_api.hpp
    channel/       channel_factory.hpp, channel_publisher.hpp, channel_subscriber.hpp
    common/        device_info(_parser).hpp, camera_info_parser.hpp, entities.hpp, robot_shared.hpp
    device/light/  light_control_client.hpp, light_control_api.hpp
    rpc/           rpc_client.hpp, rpc_server.hpp, request(_header).hpp, response(_header).hpp, error.hpp
    vision/        vision_client.hpp, handeye_calib_client.hpp
    x5_camera/     x5_camera_client.hpp
include/booster_fastdds/          Vendored FastDDS/FastCDR C++ headers (their own DDS layer,
                                   not ROS2's rmw — this is why `boosteros`'s "ROS 2 域 ID"
                                   terminology in the Python docs doesn't necessarily mean a real
                                   rclpy dependency at the wire level)
```

## Communication model (from the top-level README + doc site nav)

Two interface families, mirrored in `boosteros`:

- **High-level RPC** (`include/booster/robot/*/*_client.hpp`) — request/response calls: loco
  (`b1_loco_client.hpp`), arm (`arm_controller.hpp`), camera, vision, audio, device/light, AI/LUI.
  `boosteros.robots.booster.BoosterRobot` wraps these as `set_velocity`/`set_mode`/`do_action`/
  `get_up`/etc.
- **Low-level Topic** (DDS publish/subscribe via `channel_publisher.hpp`/`channel_subscriber.hpp`)
  — direct sensor/motor IDL messages (`LowCmd`/`LowState`/`MotorCmd`/`MotorState`/`ImuState`/
  `Odometer`/`BatteryState`/`FallDownState`) for full joint-level control, matching what
  `example/low_level/*.cpp` demonstrates. `boosteros.get_joint_states()`/`set_joints()` are the
  Python-level equivalent of subscribing/publishing these directly.

## Python bindings

`booster_robotics_sdk_python` (pip, manylinux wheels for cp310–cp314, x86_64+aarch64) is the
pybind11 binding of this same C++ SDK — 1:1 API parity with the RPC/Topic split above. The repo
README states this explicitly: "This release package provides the C++ SDK. Python SDK delivery
is handled by the pip package." `boosteros` is declared as depending on
`booster-robotics-sdk-python>=1.6.0,<1.7.0` — i.e. it's a higher-level wrapper on top, not a
separate implementation.

## K1 URDF / assets

`resource/k1_model.urdf` in this driver was copied from
`BoosterRobotics/booster_assets` → `robots/K1/K1_22dof.urdf` (fetched via the GitHub contents
API, base64-decoded — `raw.githubusercontent.com` was unreachable). That repo also has
`K1_locomotion.urdf`, `K1_22dof.xml`/`K1_22dof_parallel.xml` (MuJoCo), and `motions/K1/*.csv`
(reference motion clips) not pulled in here.

**Known joint-name mismatch**: `K1_22dof.urdf`'s joint names are SolidWorks-exporter style —
lowercase, `_joint` suffix (e.g. `aahead_yaw_joint`, `left_shoulder_roll_joint`) — while
`boosteros.get_joint_states()` returns SDK-style names (e.g. `AAHead_Yaw`, `Left_Shoulder_Roll`,
and inconsistently `Head_Pitch` in the SDK's own quick-start example output, which doesn't even
match its neighboring `AAHead_Yaw` naming pattern). `device.py::_load_urdf_joint_name_map()`
matches by a case/underscore/suffix-normalized key rather than a hardcoded table; **this needs
re-verification against a real K1** once one is reachable — see the plan's verification section.
