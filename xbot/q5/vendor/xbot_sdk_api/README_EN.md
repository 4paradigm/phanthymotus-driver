# 1. System Introduction
## 1.1 Feature Overview
RobotEraSDKAPI is a Python library based on ROS2 that provides comprehensive robot control functionality, including:

- Basic Control: Robot initialization, joint state monitoring, lifecycle management
- Trajectory Control: Predefined trajectory execution, custom trajectory planning, synchronous/asynchronous control
- MPC Control: Model Predictive Control, advanced algorithm control, ServoPose publishing
- Hand Control: XHand/XHand Lite joint control, grasping actions, custom hand movements
- Base Control: Linear and angular velocity control, forward/backward movement, turning motion

## 1.2 Use Cases

- Robot research and development
- Prototype validation and testing

# 2. Environmental Requirements

- Must be used within the developer environment provided by RobotEra
- In environments with ros2-humble + cyclonedds, the following operations need to be performed:
```shell
# Download the corresponding message definitions and source
git clone https://github.com/roboterax/teleop_client.git
cd teleop_client
colcon build
source install/setup.bash

# Set environment variables
export ROS_DOMAIN_ID=211
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

# 3. Quick Start
## 3.1 Basic Usage Example
```python
#!/usr/bin/env python3
import rclpy
from robot_controller import RobotController, TrajectoryController

def main():
    try:
        # Create robot controller
        robot = RobotController("my_robot")
        
        # Create trajectory controller, default model is Q5.
        # For L3 or L7, change "Q5" to "L3" or "L7".
        trajectory = TrajectoryController(robot, "Q5")
        
        # Start joint service
        robot.start_joint_service()
        
        # Initialize joints (approximately 20 seconds)
        robot.initialize_joints()
        
        # Set zero position
        trajectory.set_zero_position()
        
        # Move to target position
        target_positions = {
            "left_wrist_pitch_joint": 0.5,
            "left_wrist_roll_joint": -0.3
        }
        trajectory.move_to_joint_positions(target_positions, duration=3.0)
        
        print("Robot control completed!")
        
    finally:
        # Clean up resources
        robot.shutdown()

if __name__ == "__main__":
    main()
```

## 3.2 MPC Control Example
```python
import rclpy
from robot_controller import RobotController, MPCController
from xbot_common_interfaces.msg import ServoPose
from geometry_msgs.msg import PoseStamped

def main():
    try:
        # Create controllers
        robot = RobotController("mpc_robot")
        mpc = MPCController(robot)
        
        # Activate algorithm control
        robot.activate_algorithm_control()
        
        # Start MPC
        mpc.start_mpc()
        
        # Create ServoPose message
        msg = ServoPose()
        # Set left arm position
        msg.left_pose = PoseStamped()
        msg.left_pose.pose.position.x = 0.425
        msg.left_pose.pose.position.y = 0.3
        msg.left_pose.pose.position.z = 0.292
        
        # Publish ServoPose
        mpc.publish_servo_pose(msg, rate_hz=50.0, duration=3.0)
        
        # Stop MPC
        mpc.stop_mpc()
        
    finally:
        robot.shutdown()

if __name__ == "__main__":
    main()
```

## 3.3 Hand and Base Control Example
```python
#!/usr/bin/env python3
import rclpy
import time
from robot_controller import RobotController, TrajectoryController

def main():
    try:
        # Create robot controller
        robot = RobotController("hand_and_base_demo")
        # Create trajectory controller, default model is Q5.
        # For L3 or L7, change "Q5" to "L3" or "L7".
        trajectory = TrajectoryController(robot, "Q5")
        
        # Initialize robot
        robot.start_joint_service()
        robot.initialize_joints()
        trajectory.set_zero_position()
        trajectory.set_lift_up_position()
        
        # Hand control example
        print("=== Hand Control Example ===")
        
        # XHand Lite grasp
        robot.control_xhand_lite_grasp()
        time.sleep(3)
        
        # XHand grasp
        robot.control_xhand_grasp()
        time.sleep(3)
        
        # Custom hand control
        custom_joints = ['left_hand_thumb_bend_joint', 'left_hand_thumb_rota_joint1']
        custom_positions = [0.8, 0.8]
        robot.control_hand_joints(custom_joints, custom_positions)
        time.sleep(3)
        
        # Base control example
        print("=== Base Control Example ===")
        
        # Move forward (10Hz frequency, 1 second duration)
        robot.control_base_drive(linear_x=0.2, angular_z=0.0, frequency=10.0, duration=1.0)
        
        # Turn left (10Hz frequency, 1 second duration)
        robot.control_base_drive(linear_x=0.0, angular_z=0.5, frequency=10.0, duration=1.0)
        
        # Move forward and turn right (10Hz frequency, 1 second duration)
        robot.control_base_drive(linear_x=0.1, angular_z=-0.3, frequency=10.0, duration=1.0)
        
        # Stop
        robot.stop_base_drive()
        
        print("Hand and base control completed!")
        
    finally:
        robot.shutdown()

if __name__ == "__main__":
    main()
```

## 3.4 Standalone Control Examples

### 3.4.1 Hand Control Example
```python
#!/usr/bin/env python3
# examples/hand_control_demo.py
import rclpy
import time
from robot_controller import RobotController, TrajectoryController

def main():
    try:
        robot = RobotController("hand_control_demo")
        # Create trajectory controller, default model is Q5.
        # For L3 or L7, change "Q5" to "L3" or "L7".
        trajectory = TrajectoryController(robot, "Q5")
        
        # Initialize robot
        robot.start_joint_service()
        robot.initialize_joints()
        trajectory.set_zero_position()
        trajectory.set_lift_up_position()
        
        # XHand Lite grasp
        robot.control_xhand_lite_grasp()
        time.sleep(3)
        
        # XHand grasp
        robot.control_xhand_grasp()
        time.sleep(3)
        
        # Custom hand control
        custom_joints = ['left_hand_thumb_bend_joint', 'left_hand_thumb_rota_joint1']
        custom_positions = [0.8, 0.8]
        robot.control_hand_joints(custom_joints, custom_positions)
        time.sleep(3)
        
        print("Hand control completed!")
        
    finally:
        robot.shutdown()

if __name__ == "__main__":
    main()
```

### 3.4.2 Base Control Example
```python
#!/usr/bin/env python3
# examples/base_control_demo.py
import rclpy
import time
from robot_controller import RobotController, TrajectoryController

def main():
    try:
        robot = RobotController("base_control_demo")
        # Create trajectory controller, default model is Q5.
        # For L3 or L7, change "Q5" to "L3" or "L7".
        trajectory = TrajectoryController(robot, "Q5")
        
        # Initialize robot
        robot.start_joint_service()
        robot.initialize_joints()
        trajectory.set_zero_position()
        trajectory.set_lift_up_position()
        
        # Move forward and turn (10Hz frequency, 1 second duration)
        robot.control_base_drive(linear_x=0.2, angular_z=0.5, frequency=10.0, duration=1.0)
        
        # Stop
        robot.stop_base_drive()
        
        print("Base control completed!")
        
    finally:
        robot.shutdown()

if __name__ == "__main__":
    main()
```

# 4. Detailed Usage Instructions
## 4.1 Robot Initialization Process
**Step 1: Create Controllers**
```python
# Create base controller
robot = RobotController("robot_name")

# Create trajectory controller, default model is Q5.
# For L3 or L7, change "Q5" to "L3" or "L7".
trajectory = TrajectoryController(robot, "Q5")

# Create MPC controller
mpc = MPCController(robot)
```
**Step 2: Start Joint Service**
```python
# Start joint service
success = robot.start_joint_service(
    app_name="my_application",  # Application name
    sync_control=False,         # Whether to use synchronous control
    launch_mode="pos"          # Launch mode (position control)
)

if not success:
    print("Failed to start joint service")
    return
```
**Step 3: Initialize Joints**
```python
# Initialize joints (this process takes approximately 20 seconds)
success = robot.initialize_joints(timeout=25.0)

if not success:
    print("Joint initialization failed")
    return
```
**Step 4: Verify System Status**
```python
# Check if robot is ready
if robot.is_ready():
    print("Robot is ready")
else:
    print("Robot is not ready")
    return

# Get joint information
joints_info = robot.get_all_joints_info()
print(f"Available joints: {[joint['name'] for joint in joints_info]}")
```
## 4.2 Trajectory Control Usage
**Predefined Trajectories**
```
# Set zero position (all joints return to zero position)
success = trajectory.set_zero_position(duration=4.0)

# Set lift-up position (robot lifts to safe position)
success = trajectory.set_lift_up_position(duration=3.0)
```
**Custom Trajectory Control**
```python
# Define target positions
target_positions = {
    "left_shoulder_pitch_joint": 0.5,    # Left shoulder pitch
    "left_shoulder_roll_joint": -0.3,    # Left shoulder roll
    "right_shoulder_pitch_joint": -0.5,  # Right shoulder pitch
    "right_shoulder_roll_joint": 0.3,    # Right shoulder roll
}

# Synchronously move to target position
success = trajectory.move_to_joint_positions(
    target_positions=target_positions,
)
```
**Asynchronous Trajectory Control**
```python
def progress_callback(progress):
    """Progress callback function"""
    print(f"Execution progress: {progress*100:.1f}%")

def completion_callback(success, result):
    """Completion callback function"""
    if success:
        print(f"Trajectory execution completed: {result}")
    else:
        print(f"Trajectory execution failed: {result}")

# Execute trajectory asynchronously
success = trajectory.move_to_joint_positions(
    target_positions=target_positions,
    duration=5.0,
    async_execution=True,
    progress_callback=progress_callback,
    completion_callback=completion_callback
)
```
## 4.3 MPC Control Usage
**MPC Control Process**
```python
# 1. Activate algorithm control permission
if not robot.activate_algorithm_control():
    print("Failed to activate algorithm control")
    return

# 2. Start MPC
if not mpc.start_mpc():
    print("Failed to start MPC")
    return

# 3. Query MPC status
status = mpc.query_mpc()
if status == 1:
    print("MPC is started")
elif status == 0:
    print("MPC is stopped")
else:
    print("MPC status unknown")

# 4. Publish ServoPose
def create_pose_stamped(x, y, z, qx, qy, qz, qw, frame_id='base_link'):
    """Create PoseStamped message"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position = Point(x=x, y=y, z=z)
    pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    return pose

# Create ServoPose message
msg = ServoPose()

# Set left arm position and orientation
msg.left_pose = create_pose_stamped(
    x=0.425, y=0.3, z=0.292,
    qx=0.612, qy=-0.016, qz=0.79, qw=-0.006
)

# Set right arm position and orientation
msg.right_pose = create_pose_stamped(
    x=0.417, y=-0.146, z=0.382,
    qx=0.521, qy=0.1, qz=0.842, qw=-0.096
)

# Set head position and orientation
msg.head_pose = create_pose_stamped(
    x=0.0, y=0.0, z=0.0,
    qx=0.0, qy=0.0, qz=0.0, qw=1.0
)

# Publish ServoPose (50Hz frequency, 3 seconds duration)
mpc.publish_servo_pose(msg, rate_hz=50.0, duration=3.0)

# 5. Stop MPC
mpc.stop_mpc()

# 6. Deactivate algorithm control permission
robot.deactivate_algorithm_control()
```

## 4.4 Status Monitoring
**Get Joint Status**
```python
# Get all joint information
joints_info = robot.get_all_joints_info()
for joint in joints_info:
    print(f"Joint {joint['name']}:")
    print(f"  Position: {joint['position']:.3f} rad")
    print(f"  Velocity: {joint['velocity']:.3f} rad/s")
    print(f"  Effort: {joint['effort']:.3f} Nm")

# Get specific joint information
joint_info = robot.get_joint_info("left_shoulder_pitch")
if joint_info:
    print(f"Left shoulder pitch joint: {joint_info}")
```

# 5. API Detailed Documentation

## 5.1 RobotController Class
|Function|Description|
|---|---|
|RobotController|Constructor|
|start_joint_service|Start joint service|
|initialize_joints|Initialize joint module|
|get_joint_states|Get current joint states|
|get_joint_info|Get specific joint information|
|get_all_joints_info|Get all joint information|
|is_ready|Check if robot is ready|
|activate_algorithm_control|Activate algorithm control permission|
|deactivate_algorithm_control|Deactivate algorithm control permission|
|control_hand_joints|Control hand joints|
|control_xhand_lite_grasp|Control XHand Lite grasping action|
|control_xhand_grasp|Control XHand grasping action|
|control_base_drive|Control base movement|
|stop_base_drive|Stop base movement|


**Constructor**
```python
RobotController(node_name: str = "robot_controller")
```
**Function**: Create a robot controller instance.
**Parameters**:
- node_name: ROS2 node name

**start_joint_service()**
```python
start_joint_service(app_name: str = "", sync_control: bool = False, launch_mode: str = "pos") -> bool
```
**Function**: Start joint service.
**Parameters**:
- app_name: Application name
- sync_control: Whether to use synchronous control
- launch_mode: Launch mode
**Returns**: 
- Returns True on success
- Returns False on failure

**initialize_joints()**
```python
initialize_joints(timeout: float = 25.0) -> bool
```
**Function**: Initialize joint module.
**Parameters**:
- timeout: Timeout in seconds
**Returns**: Returns True on success, False on failure

**get_joint_states()**
```python
get_joint_states() -> Optional[JointState]
```
**Function**: Get current joint states.
**Returns**: JointState message or None

**get_joint_info()**
```python
get_joint_info(joint_name: str) -> Optional[Dict[str, Any]]
```
**Function**: Get specific joint information.
**Parameters**:
- joint_name: Joint name
**Returns**: Dictionary containing joint information or None

**get_all_joints_info()**
```python
get_all_joints_info() -> List[Dict[str, Any]]
```
**Function**: Get all joint information.
**Returns**: List of joint information

**is_ready()**
```python
is_ready() -> bool
```
**Function**: Check if robot is ready.
**Returns**: Returns True if ready, False otherwise

**activate_algorithm_control()**
```python
activate_algorithm_control() -> bool
```
**Function**: Activate algorithm control permission.
**Returns**: Returns True on success, False on failure

**deactivate_algorithm_control()**
```python
deactivate_algorithm_control() -> bool
```
**Function**: Deactivate algorithm control permission.
**Returns**: Returns True on success, False on failure

**control_hand_joints()**
```python
control_hand_joints(joint_names: List[str], positions: List[float], 
                   velocities: Optional[List[float]] = None,
                   feedforward: Optional[List[float]] = None,
                   kp: Optional[List[float]] = None,
                   kd: Optional[List[float]] = None) -> bool
```
**Function**: Control hand joint movement.
**Parameters**:
- joint_names: List of joint names
- positions: List of target positions
- velocities: List of target velocities (optional, default is 0.0)
- feedforward: List of feedforward forces/torques (optional, default is 350.0)
- kp: List of proportional gains (optional, default is 100.0)
- kd: List of derivative gains (optional, default is 0.0)
**Returns**: Returns True on success, False on failure

**control_xhand_lite_grasp()**
```python
control_xhand_lite_grasp() -> bool
```
**Function**: Control XHand Lite to perform gentle grasping action.
**Returns**: Returns True on success, False on failure

**control_xhand_grasp()**
```python
control_xhand_grasp() -> bool
```
**Function**: Control XHand to perform gentle grasping action.
**Returns**: Returns True on success, False on failure

**control_base_drive()**
```python
control_base_drive(linear_x: float = 0.0, angular_z: float = 0.0, 
                   frequency: float = None, duration: float = None) -> bool
```
**Function**: Control base movement.
**Parameters**:
- linear_x: Linear velocity in X direction (m/s)
- angular_z: Angular velocity in Z direction (rad/s)
- frequency: Publishing frequency (Hz). If this parameter is provided, duration must also be provided
- duration: Publishing duration in seconds. If this parameter is provided, frequency must also be provided
**Returns**: Returns True on success, False on failure

**Note**:
- If frequency and duration parameters are not provided, the command will only be sent once
- If frequency and duration parameters are provided, commands will be continuously published at the specified frequency for the specified duration

**stop_base_drive()**
```python
stop_base_drive() -> bool
```
**Function**: Stop base movement.
**Returns**: Returns True on success, False on failure

## 5.2 TrajectoryController Class
|Function|Description|
|---|---|
|TrajectoryController|Constructor|
|set_zero_position|Set joints to zero position|
|set_lift_up_position|Set joints to lift-up position|
|move_to_joint_positions|Move joints to target positions|


**Constructor**
```python
TrajectoryController(robot_controller, model: str)
```
**Function**: Create a trajectory controller instance.
**Parameters**:
- robot_controller: RobotController instance
- model: Robot model name, currently supports "Q5", "L3", and "L7".

**set_zero_position()**
```python
set_zero_position(duration: float = 4.0) -> bool
```
**Function**: Set joints to zero position.
**Parameters**: 
- duration: Execution time in seconds
**Returns**: Returns True on success, False on failure

**set_lift_up_position()**
```python
set_lift_up_position(duration: float = 4.0) -> bool
```
**Function**: Set joints to lift-up position.
**Parameters**:
- duration: Execution time in seconds
**Returns**: Returns True on success, False on failure

**move_to_joint_positions**
```python
move_to_joint_positions(target_positions: Dict[str, float], duration: float = 3.0, 
                       kp: Optional[List[float]] = None, kd: Optional[List[float]] = None, 
                       rate_hz: float = 100.0, async_execution: bool = False, 
                       progress_callback: Optional[Callable] = None, 
                       completion_callback: Optional[Callable] = None) -> bool
```

**Function**: Move joints to target positions.

**Parameters**:
- target_positions: Dictionary of target positions
- duration: Execution time in seconds
- kp: List of proportional gains
- kd: List of derivative gains
- rate_hz: Publishing frequency
- async_execution: Whether to execute asynchronously
- progress_callback: Progress callback function
- completion_callback: Completion callback function
**Returns**: Returns True on success, False on failure

## 5.3 MPCController Class
|Function|Description|
|---|---|
|MPCController|Constructor|
|start_mpc|Start MPC control|
|stop_mpc|Stop MPC control|
|query_mpc|Query MPC status|
|publish_servo_pose|Move to a position|
|||


**Constructor**
```python
MPCController(robot_controller: Node)
```
**Function**: Create an MPC controller instance.
**Parameters**:
- robot_controller: RobotController node instance

**start_mpc()**
```python
start_mpc() -> bool
```
**Function**: Start MPC control.
**Returns**: Returns True on success, False on failure

**stop_mpc()**
```python
stop_mpc() -> bool
```
**Function**: Stop MPC control.
**Returns**: Returns True on success, False on failure

**query_mpc()**
```python
query_mpc() -> Optional[int]
```
**Function**: Query MPC status.
**Returns**: 1 indicates started, 0 indicates stopped, None indicates query failed

**publish_servo_pose()**
```python
publish_servo_pose(msg: ServoPose, rate_hz: float = 10.0, duration: float = 2.0)
```
**Function**: Move to a position
**Parameters**:
- msg: ServoPose message
- rate_hz: Publishing frequency
- duration: Publishing duration in seconds

