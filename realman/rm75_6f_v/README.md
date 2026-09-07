# RealMan RM75-6F-V Driver

This Linux ARM64 image exposes RM75 controller data and bounded joint motion to
Phanthy Motus through MCP. It calls the official RealMan API2 Python SDK
directly; it does not build or run the official ROS2 `rm_driver`.

```text
Card / Agent Core -> MCP Driver -> RealMan API2 SDK -> arm controller
```

The deployment service connects to `192.168.1.18:8080` with motion enabled by
default. Before starting it, verify the network, physical E-stop, work area and
joint order. Every movement still requires explicit per-call confirmation:

```bash
confirm_motion=true
```

Available tools are:

- `connection`: SDK connection state.
- `joint_states`: seven joint angles in radians, plus raw SDK degrees.
- `robot_info`, `software_info`, `arm_all_state`, `controller_state`: read-only
  API2 queries.
- `model`: simplified RM75-6F-V URDF for skeleton display.
- `joint_control`: bounded joint-space motion and controlled stop.

The deployment enables its motion capability, and every `set` call must still
include `confirm_motion=true`. `joint1_deg` through `joint7_deg` are absolute
targets in degrees. An omitted joint defaults to its measured position at the
start of the request, while the supplied targets are sent together as one API2
`movej` trajectory so the controller plans all joints concurrently. The
Driver rejects non-finite values, targets outside the
official RM75 limits, speed above 10 percent, disabled
joints, and any reported arm or joint error. It sends non-blocking API2
`rm_movej`, monitors the measured joints until they reach the target, and
supports `stopmotion` while movement is active. The card does not expose a
fixed `timeout_seconds`: after a 2-second startup grace period, the driver asks
for a controlled slow stop and reports `motion_stalled` through ACP if the
maximum joint error has not improved by at least 0.05 degrees for 10 seconds.
It also derives an internal deadline from commanded distance and speed, capped
at 300 seconds, as a final safeguard.

The first supervised hardware test should change exactly one joint by no more
than 1 degree at 1 percent speed. A reachable physical E-stop and a clear work
area are required. Software interlocks do not replace the robot safety system.

The HTTP service listens on port `15718` and provides `/health` and `/mcp`.
The normal Agent Core runtime still initializes its ROS/DDS transport, but robot
communication itself goes directly through API2 TCP port `8080`.

Only the Python SDK wrapper is stored in Git. The operator's licensed Linux
ARM64 `libapi_c.so` must be installed on the robot host at
`/opt/realman/rm_api2/libs/linux_arm/libapi_c.so`; `service.yml` mounts that
directory read-only at the path expected by API2. The expected SHA-256 is
`5b9d236a5cf901cdf05418d9ef5815a77a8c717af0ff037e7aad9247beb76fb9`.
Verify the operator-provided file before starting the enabled Driver:

```bash
echo "5b9d236a5cf901cdf05418d9ef5815a77a8c717af0ff037e7aad9247beb76fb9  /opt/realman/rm_api2/libs/linux_arm/libapi_c.so" \
  | sha256sum -c -
```

The image can still be smoke-tested manually with `RM_DRIVER_ENABLED=0` without
the library, but the deployment service defaults to a live, motion-capable
connection. An enabled connection fails with an explicit mount error when the
library is absent. Set `RM_API2_LIB_DIR` to override the host directory. ACP
HTTPS callbacks verify the Agent Core hostname and certificate using
`AGENT_CORE_CA_CERT`; unencrypted callbacks are accepted only on loopback.

The component installs `python3-yaml` because the shared runtime loads
`config.yaml`, and `ros-humble-rmw-fastrtps-cpp` because the shared runtime
creates the Agent Core ROS 2 participant. No compiler, pip, or ROS build tooling
is installed. See `vendor/SOURCE.md` for provenance notes.
