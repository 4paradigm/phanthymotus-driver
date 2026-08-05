# Interface Protocol

Description of the interface protocol.

## Interface Protocol


| Topic Name                        | Type      | Message                    | Description                                                                                                                 | Example                                        |
| --------------------------------- | --------- | -------------------------- | --------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| /motion/joint_override_command    | Publish   | JointOverrideCommand.msg   | Override control commands for specific joints; supports smooth trajectory generation with quintic polynomial interpolation. | upper_joint_override_example.py                |
| /motion/set_motion_state          | Publish   | MotionStateRequest.msg     | Request to switch the robot motion state (e.g. switch to terrain walking gait).                                             | switch_to_terrain_walking_gait_example.py      |
| /motion/motion_state              | Subscribe | MotionState.msg            | Receive current robot motion state, including current state and available transitions.                                      | switch_to_terrain_walking_gait_example.py      |
| /hardware/led_control             | Publish   | LedControl.msg             | Control robot LED light patterns; supports multiple colors and effects.                                                     | led_control_example.py                         |
| /motion/joint_motion_plan/request | Publish   | JointMotionPlanRequest.msg | Request a joint motion planning task; supports trajectory planning and reset to default pose.                               | joint_mutiple_motion_plan_example.py           |
| /motion/joint_motion_plan/state   | Subscribe | JointMotionPlanState.msg   | Receive joint motion planner state and execution progress.                                                                  | joint_mutiple_motion_plan_example.py           |
| /hardware/joint_state             | Subscribe | JointState.msg             | Receive current state of all joints (position, velocity, torque).                                                           | joint_bridge_example.py                        |
| /hardware/joint_command           | Publish   | JointCommand.msg           | Send joint control commands to control motion of all joints (e.g. via joint bridge).                                        | joint_bridge_example.py, joint_test_example.cc |
| /motion/body_vel_cmd              | Publish   | BodyVelCmd.msg             | Body velocity command (linear velocity and yaw angular velocity).                                                           | body_velocity_control_example.py               |
| /hardware/gamepad_keys            | Subscribe | GamepadKeys.msg            | Gamepad input data.                                                                                                         | rl_basic_example.cc                            |
| /hardware/imu_info                | Subscribe | ImuInfo.msg                | IMU sensor data.                                                                                                            | rl_basic_example.cc                            |


