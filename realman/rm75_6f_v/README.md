# RealMan RM75-6F-V Driver

This is an upper-level Phanthy Motus MCP adapter for a RealMan RM75-6F-V arm. It does not replace the official `rm_driver`; it expects the official ROS2 Humble driver to be running and connected to the arm.

Runtime chain:

```text
Card system -> this MCP driver -> official rm_driver ROS2 topics -> RM75 built-in controller
```

## WSL smoke test

In one WSL terminal, start the official driver:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rm_driver rm_75_driver.launch.py
```

In another WSL terminal, start this adapter:

```bash
cd /mnt/e/第四范式/phanthymotus-driver-main/phanthymotus-driver-main/realman/rm75_6f_v
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
PYTHONPATH=/mnt/e/第四范式/phanthymotus-driver-main/phanthymotus-driver-main python3 main.py
```

Health check:

```bash
curl http://localhost:15718/health
```

List MCP tools:

```bash
curl -s http://localhost:15718/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}'
```

## First safe operations

Use `state`, `joint_states`, `joint_error`, and `rm_error` first. The `joint_control.set` action publishes to `/rm_driver/movej_cmd` and should only be tested with a known-safe target pose, low speed, clear workspace, and an operator ready to stop the arm.

## Driver surface

- HTTP service: `/health` and `/mcp` on port `15718`.
- MCP resources: `model` returns a simplified URDF for skeleton rendering.
- MCP sensors: `state`, `joint_states`, `arm_state`, `arm_original_state`, `arm_current_status`, `joint_error`, `rm_error`, plus optional command result streams when matching `rm_ros_interfaces` message types are available. `state` is a request/response snapshot and intentionally has no `topic_out`; use the individual stream tools for topic-backed cards.
- MCP actuator: `joint_control` supports `set`, `stopmotion`, and `clear_joint_error`. The `set` action exposes seven numeric inputs, `joint1` through `joint7`, defaults omitted joints to `0.0`, and publishes them as one full 7-DOF `movej` command. `clear_joint_error` publishes the official joint-error-clear command for `joint_num` 1 through 7.

`joint_control` uses explicit `joint1`..`joint7` target values and defaults omitted joints to `0.0`, so it never guesses omitted joints from `/joint_states`. Out-of-limit or non-finite joint values are rejected before any command is published.

`movej` performs a conservative preflight before publishing: recent `/joint_states` required, configured joint limits checked, and active error topics rejected. MoveJ populates the RealMan `rm_ros_interfaces/msg/Movej` fields available in the sourced workspace: `joint`, `speed`/`v` for velocity percentage, optional `block`, optional `trajectory_connect`, optional `dof`, and optional `r` for blend radius. Seven-joint control uses the Agent Core completion contract: the MCP call returns an `action_id` immediately, a worker publishes the ROS command, then reports `/api/acp/complete` when `movej_result` arrives, the target is reached within `safety.joint_target_tolerance_rad`, an error appears, or the bounded timeout expires. Set `safety.require_*` fields in `config.yaml` only when deliberately testing around those guards.

The URDF in `resource/rm75_6f_v.urdf` is intentionally simplified for card-system skeleton rendering. Use the vendor description package when an accurate kinematic or visual model is required.

## Infrastructure notes

- `driver.yaml` is required registration/build metadata for Agent Core and does not increase image size.
- `deploy/service.yml` is required to mount the externally built official `rm_driver` workspace, use host networking for localhost MCP registration and ROS2/DDS discovery, access device/network resources, and cap container logs; it does not increase image size.
- `Dockerfile` installs `python3-pip`, `ros-humble-rmw-fastrtps-cpp`, and `requirements.txt` because the selected ROS base image is not guaranteed to include Python package installation support, the configured RMW implementation, or PyYAML for `config.yaml` loading. This intentionally increases the image by those runtime dependencies only.
- `common/vendor_runtime.py` is touched only to escape and cap host-network HTTP request-line logging for all drivers using the shared runtime. This shared safety fix does not increase image size.
