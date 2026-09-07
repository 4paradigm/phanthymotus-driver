# RealMan RM75-6F-V Driver

This Linux ARM64 image exposes read-only RM75 controller data to Phanthy Motus
through MCP. It calls the official RealMan API2 Python SDK directly; it does not
build or run the official ROS2 `rm_driver`.

```text
Card / Agent Core -> MCP Driver -> RealMan API2 SDK -> arm controller
```

The SDK connection is disabled by default, so an image smoke test cannot move
or contact the robot. After checking the network, E-stop and work area, enable
the connection with:

```bash
RM_DRIVER_ENABLED=1
RM_ARM_IP=<controller-ip>
RM_TCP_PORT=8080
```

Available tools are:

- `connection`: SDK connection state.
- `joint_states`: seven joint angles in radians, plus raw SDK degrees.
- `robot_info`, `software_info`, `arm_all_state`, `controller_state`: read-only
  API2 queries.
- `model`: simplified RM75-6F-V URDF for skeleton display.
- `joint_control`: bounded joint-space motion and controlled stop.

Motion has two independent interlocks. The container must set
`RM_MOTION_ENABLED=1`, and every `set` call must include
`confirm_motion=true`. `joint1_deg` through `joint7_deg` are absolute targets
in degrees. An omitted joint defaults to its measured position at the start of
the request, while the supplied targets are sent together as one API2 `movej`
trajectory so the controller plans all joints concurrently. The
Driver rejects non-finite values, targets outside the
official RM75 limits, speed above 10 percent, disabled
joints, and any reported arm or joint error. It sends non-blocking API2
`rm_movej`, monitors the measured joints until they reach the target, and
supports `stopmotion` while movement is active.

The first supervised hardware test should change exactly one joint by no more
than 1 degree at 1 percent speed. A reachable physical E-stop and a clear work
area are required. Software interlocks do not replace the robot safety system.

The HTTP service listens on port `15718` and provides `/health` and `/mcp`.
The normal Agent Core runtime still initializes its ROS/DDS transport, but robot
communication itself goes directly through API2 TCP port `8080`.

Only the Python SDK wrapper is stored in Git. The Linux ARM64 `libapi_c.so` is
downloaded from COS during the image build and verified by SHA-256 before it is
installed. The component installs `python3-yaml` because the shared runtime
loads `config.yaml`, and `ros-humble-rmw-fastrtps-cpp` because the shared
runtime creates the Agent Core ROS 2 participant. No compiler, pip, or ROS
build tooling is installed. See `vendor/SOURCE.md` for provenance and
redistribution notes.
