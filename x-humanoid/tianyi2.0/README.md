# Tianyi 2.0 Pro Driver

Phanthy Motus driver bundle for the Tianyi 2.0 Pro humanoid robot. The driver
bridges robot-side ROS2 topics on domain 0 to Agent Core topics on domain 42 and
exposes the capabilities as MCP tools.

## Service state card

`service_state` aggregates the robot's internal `NodeState` messages and reports
the latest running/idle state for each service topic.

| Item | Value |
|---|---|
| Tool name | `service_state` |
| Tool type | `sensor` |
| Robot-side inputs | `/bodycontrol_state`, `/node/status` |
| Agent Core output | `/{namespace}/state/service_state` |
| Output format | `data/json` |

The target robot confirms both inputs as `bodyctrl_msgs/msg/NodeState` with
RELIABLE publishers. `/node/status` continuously reports the process manager;
`/bodycontrol_state` is an event-oriented stream published by components such as
the power board and Bluetooth server.

## Motor faults card

`motor_faults` reports only motors whose vendor error code is non-zero. It does
not duplicate the joint position, velocity, current, or temperature stream.

| Item | Value |
|---|---|
| Tool name | `motor_faults` |
| Tool type | `sensor` |
| Robot-side inputs | `/head/status`, `/arm/status`, `/waist/status`, `/leg/status` |
| Agent Core output | `/{namespace}/state/motor_faults` |
| Output format | `data/json` |

When a motor's error code returns to zero, it is removed from the active fault
list. Error codes are preserved as vendor-provided integers; this driver does
not invent descriptions without an authoritative error-code table.

Both cards are read-only. They do not command joints, stop the robot, trigger an
emergency stop, or modify any robot-side configuration.
