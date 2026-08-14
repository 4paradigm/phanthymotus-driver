"""
Robot Controller Base Class
Provides basic robot control functionality using ROS2 services.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from xbot_common_interfaces.srv import DynamicLaunch
from xbot_common_interfaces.action import SimpleActions
from xbot_common_interfaces.msg import HybridJointCommand
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import time
import threading
from typing import Optional, Dict, Any, List


class RobotController(Node):
    """
    Base robot controller class that provides common robot control functionality.
    
    This class encapsulates ROS2 service calls for robot initialization and control.
    """
    
    def __init__(self, node_name: str = "robot_controller"):
        """
        Initialize the robot controller.
        
        Args:
            node_name: Name for the ROS2 node
        """
        super().__init__(node_name)
        
        # Create callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Service clients
        self.dynamic_launch_client = self.create_client(
            DynamicLaunch, 
            '/dynamic_launch',
            callback_group=self.callback_group
        )
        self.ready_service_client = self.create_client(
            Trigger,
            '/ready_service', 
            callback_group=self.callback_group
        )
        self.activate_service_client = self.create_client(
            Trigger,
            '/activate_service',
            callback_group=self.callback_group
        )
        self.deactivate_service_client = self.create_client(
            Trigger,
            '/deactivate_service',
            callback_group=self.callback_group
        )
        
        # Action client for simple predefined actions (e.g., zero, lift_up)
        self.trajectory_action_client = ActionClient(
            self,
            SimpleActions,
            '/simple_actions',
            callback_group=self.callback_group
        )
        
        # Joint state subscriber
        self.joint_states_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10
        )
        
        # Hand control publisher
        self.hand_command_publisher = self.create_publisher(
            HybridJointCommand,
            '/hand_controller/commands',
            10
        )
        
        # Base drive control publisher
        self.base_drive_publisher = self.create_publisher(
            TwistStamped,
            '/wr1_base_drive_controller/cmd_vel',
            10
        )
        
        # Current joint states
        self.current_joint_states: Optional[JointState] = None
        self.joint_states_lock = threading.Lock()
    
    def _joint_states_callback(self, msg: JointState):
        """Callback for joint states topic."""
        with self.joint_states_lock:
            self.current_joint_states = msg
    
    def start_joint_service(self, app_name: str = "", sync_control: bool = False, 
                          launch_mode: str = "pos") -> bool:
        """
        Start the joint service.
        
        Args:
            app_name: Application name for the service
            sync_control: Whether to use synchronous control
            launch_mode: Launch mode ('pos' for position control)
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self.dynamic_launch_client.wait_for_service(timeout_sec=10):
                raise RuntimeError("Ready service not available")

            request = DynamicLaunch.Request()
            request.app_name = app_name
            request.sync_control = sync_control
            request.launch_mode = launch_mode
            
            self.get_logger().info("Starting joint service...")
            future = self.dynamic_launch_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                self.get_logger().info("Joint service started successfully")
                return True
            else:
                self.get_logger().error("Failed to start joint service")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error starting joint service: {str(e)}")
            return False
    
    def initialize_joints(self, timeout: float = 25.0) -> bool:
        """
        Initialize joint modules. This process takes about 20 seconds.
        
        Args:
            timeout: Maximum time to wait for initialization
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self.ready_service_client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError("Ready service not available")

            request = Trigger.Request()
            
            self.get_logger().info("Initializing joint modules (this may take ~20 seconds)...")
            future = self.ready_service_client.call_async(request)
            
            # Wait for completion with timeout
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > timeout:
                    self.get_logger().error("Joint initialization timed out")
                    return False
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.result() is not None:
                self.get_logger().info("Joint modules initialized successfully")
                return True
            else:
                self.get_logger().error("Failed to initialize joint modules")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error initializing joints: {str(e)}")
            return False
    
    def get_joint_states(self) -> Optional[JointState]:
        """
        Get current joint states.
        
        Returns:
            Current joint states message or None if not available
        """
        with self.joint_states_lock:
            return self.current_joint_states
    
    def get_joint_info(self, joint_name: str) -> Optional[Dict[str, Any]]:
        """
        Get information about a specific joint.
        
        Args:
            joint_name: Name of the joint
            
        Returns:
            Dictionary containing joint information or None if not found
        """
        joint_states = self.get_joint_states()
        if joint_states is None:
            return None
        
        try:
            joint_index = joint_states.name.index(joint_name)
            return {
                'name': joint_states.name[joint_index],
                'position': joint_states.position[joint_index],
                'velocity': joint_states.velocity[joint_index],
                'effort': joint_states.effort[joint_index] if joint_states.effort else 0.0
            }
        except ValueError:
            self.get_logger().warning(f"Joint '{joint_name}' not found")
            return None
    
    def get_all_joints_info(self) -> List[Dict[str, Any]]:
        """
        Get information about all joints.
        
        Returns:
            List of dictionaries containing joint information
        """
        joint_states = self.get_joint_states()
        if joint_states is None:
            return []
        
        joints_info = []
        for i, name in enumerate(joint_states.name):
            joints_info.append({
                'name': name,
                'position': joint_states.position[i],
                'velocity': joint_states.velocity[i],
                'effort': joint_states.effort[i] if joint_states.effort else 0.0
            })
        
        return joints_info
    
    def is_ready(self) -> bool:
        """
        Check if the robot is ready for operation.
        
        Returns:
            True if robot is ready, False otherwise
        """
        return self.current_joint_states is not None
    
    def control_hand_joints(self, joint_names: List[str], positions: List[float], 
                           velocities: Optional[List[float]] = None,
                           feedforward: Optional[List[float]] = None,
                           kp: Optional[List[float]] = None,
                           kd: Optional[List[float]] = None) -> bool:
        """
        Control hand joints using HybridJointCommand.
        
        Args:
            joint_names: List of joint names to control
            positions: Target positions for each joint
            velocities: Target velocities for each joint (optional, defaults to 0.0)
            feedforward: Feedforward force/torque for each joint (optional, defaults to 350.0)
            kp: Proportional gain for each joint (optional, defaults to 100.0)
            kd: Derivative gain for each joint (optional, defaults to 0.0)
            
        Returns:
            True if command was published successfully, False otherwise
        """
        try:
            if len(joint_names) != len(positions):
                self.get_logger().error("Number of joint names must match number of positions")
                return False
            
            # Set defaults
            if velocities is None:
                velocities = [0.0] * len(joint_names)
            if feedforward is None:
                feedforward = [350.0] * len(joint_names)
            if kp is None:
                kp = [100.0] * len(joint_names)
            if kd is None:
                kd = [0.0] * len(joint_names)
            
            # Validate lengths
            if (len(velocities) != len(joint_names) or 
                len(feedforward) != len(joint_names) or
                len(kp) != len(joint_names) or
                len(kd) != len(joint_names)):
                self.get_logger().error("All parameter lists must have the same length as joint_names")
                return False
            
            # Create command message
            cmd = HybridJointCommand()
            cmd.header = Header()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = ''
            
            cmd.joint_name = joint_names
            cmd.position = positions
            cmd.velocity = velocities
            cmd.feedforward = feedforward
            cmd.kp = kp
            cmd.kd = kd
            
            # Publish command
            self.hand_command_publisher.publish(cmd)
            self.get_logger().info(f"Published hand control command for {len(joint_names)} joints")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error controlling hand joints: {str(e)}")
            return False
    
    def control_xhand_lite_grasp(self) -> bool:
        """
        Control XHand Lite to perform a gentle grasp gesture.
        
        Returns:
            True if command was published successfully, False otherwise
        """
        joint_names = [
            'left_hand_thumb_bend_joint', 'left_hand_thumb_rota_joint1', 
            'left_hand_index_joint1', 'left_hand_mid_joint1', 
            'left_hand_ring_joint1', 'left_hand_pinky_joint1',
            'right_hand_thumb_bend_joint', 'right_hand_thumb_rota_joint1', 
            'right_hand_index_joint1', 'right_hand_mid_joint1', 
            'right_hand_ring_joint1', 'right_hand_pinky_joint1'
        ]
        
        positions = [0.5, 0.5, 1.0, 1.0, 1.0, 1.0,
                    0.5, 0.5, 1.0, 1.0, 1.0, 1.0]
        
        return self.control_hand_joints(joint_names, positions)
    
    def control_xhand_grasp(self) -> bool:
        """
        Control XHand to perform a gentle grasp gesture.
        
        Returns:
            True if command was published successfully, False otherwise
        """
        joint_names = [
            'left_hand_thumb_bend_joint', 'left_hand_thumb_rota_joint1', 
            'left_hand_thumb_rota_joint2', 'left_hand_index_bend_joint', 
            'left_hand_index_joint1', 'left_hand_index_joint2',
            'left_hand_mid_joint1', 'left_hand_mid_joint2', 
            'left_hand_ring_joint1', 'left_hand_ring_joint2', 
            'left_hand_pinky_joint1', 'left_hand_pinky_joint2',
            'right_hand_thumb_bend_joint', 'right_hand_thumb_rota_joint1', 
            'right_hand_thumb_rota_joint2', 'right_hand_index_bend_joint', 
            'right_hand_index_joint1', 'right_hand_index_joint2',
            'right_hand_mid_joint1', 'right_hand_mid_joint2', 
            'right_hand_ring_joint1', 'right_hand_ring_joint2', 
            'right_hand_pinky_joint1', 'right_hand_pinky_joint2'
        ]
        
        positions = [0.5, 0.5, 1.0, 0.3, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                    0.5, 0.5, 1.0, 0.3, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        
        return self.control_hand_joints(joint_names, positions)
    
    def control_base_drive(self, 
                           linear_x: float = 0.0, 
                           angular_z: float = 0.0,
                           frequency: float = None,
                           duration: float = None) -> bool:
        """
        Control base drive with linear and angular velocities.
        
        Args:
            linear_x: Linear velocity in X direction (m/s)
            angular_z: Angular velocity around Z axis (rad/s)
            frequency: Publishing frequency in Hz. If provided with duration, keeps publishing at this rate
            duration: Total time in seconds to keep publishing at the given frequency
            
        Returns:
            True if command was published successfully, False otherwise
        """
        try:
            # Validate frequency/duration if provided
            if (frequency is None) ^ (duration is None):
                self.get_logger().error("Both frequency and duration must be provided together")
                return False
            if frequency is not None and frequency <= 0.0:
                self.get_logger().error("frequency must be > 0")
                return False
            if duration is not None and duration <= 0.0:
                self.get_logger().error("duration must be > 0")
                return False

            def publish_once():
                cmd = TwistStamped()
                cmd.header = Header()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = ''
                cmd.twist.linear.x = linear_x
                cmd.twist.linear.y = 0.0
                cmd.twist.linear.z = 0.0
                cmd.twist.angular.x = 0.0
                cmd.twist.angular.y = 0.0
                cmd.twist.angular.z = angular_z
                self.base_drive_publisher.publish(cmd)

            # Publish once or repeatedly based on provided parameters
            if frequency is None and duration is None:
                publish_once()
                self.get_logger().info(
                    f"Published base drive command once: linear_x={linear_x}, angular_z={angular_z}")
                return True
            
            start_time = time.time()
            period = 1.0 / frequency
            next_pub = start_time
            self.get_logger().info(
                f"Publishing base drive for {duration:.3f}s at {frequency:.3f}Hz: linear_x={linear_x}, angular_z={angular_z}")
            while rclpy.ok() and (time.time() - start_time) < duration:
                now = time.time()
                if now >= next_pub:
                    publish_once()
                    next_pub += period
                # Sleep a small amount to avoid busy-wait; respect short periods
                remaining = max(0.0, next_pub - time.time())
                time.sleep(min(0.002, remaining))
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error controlling base drive: {str(e)}")
            return False
    
    def stop_base_drive(self) -> bool:
        """
        Stop the base drive by setting all velocities to zero.
        
        Returns:
            True if command was published successfully, False otherwise
        """
        return self.control_base_drive(linear_x=0.0, angular_z=0.0)
    
    def shutdown(self):
        """Shutdown the robot controller."""
        self.get_logger().info("Shutting down robot controller...")
        self.destroy_node()

    def activate_algorithm_control(self) -> bool:
        """Call /activate_service to enable algorithm control (e.g., MPC)."""
        try:
            if not self.activate_service_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("/activate_service not available")
                return False
            self.get_logger().info("Activating algorithm control...")
            future = self.activate_service_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            return future.result() is not None and future.result().success
        except Exception as e:
            self.get_logger().error(f"activate_algorithm_control error: {e}")
            return False

    def deactivate_algorithm_control(self) -> bool:
        """Call /deactivate_service to disable algorithm control."""
        try:
            if not self.deactivate_service_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("/deactivate_service not available")
                return False
            future = self.deactivate_service_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            return future.result() is not None and future.result().success
        except Exception as e:
            self.get_logger().error(f"deactivate_algorithm_control error: {e}")
            return False

