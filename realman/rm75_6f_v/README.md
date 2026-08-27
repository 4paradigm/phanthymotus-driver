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
curl http://localhost:15721/health
```

List MCP tools:

```bash
curl -s http://localhost:15721/mcp \
  -H 'Content-Type: application/json' \
  -d '{"jsonrpc":"2.0","id":1,"method":"tools/list","params":{}}'
```

## First safe operations

Use `state`, `joint_states`, `joint_error`, and `rm_error` first. The `arm_motion.movej` action publishes to `/rm_driver/movej_cmd` and should only be tested with a known-safe target pose, low speed, clear workspace, and an operator ready to stop the arm.

## Driver surface

- HTTP service: `/health` and `/mcp`.
- MCP resources: `model` returns a simplified URDF for skeleton rendering.
- MCP sensors: `state`, `joint_states`, `arm_state`, `arm_original_state`, `arm_current_status`, `joint_error`, `rm_error`, plus optional command result streams when matching `rm_ros_interfaces` message types are available.
- MCP actuator: `arm_motion` supports `movej`, `stopmotion`, and `clear_joint_error`.

`movej` performs a conservative preflight before publishing: recent `/joint_states` required, configured joint limits checked, and active error topics rejected. Set `safety.require_*` fields in `config.yaml` only when deliberately testing around those guards.
