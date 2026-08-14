"""
Trajectory Controller Class
Provides trajectory control functionality using ROS2 actions.
"""

import rclpy
from rclpy.action import ActionClient
from xbot_common_interfaces.action import SimpleActions
from xbot_common_interfaces.msg import HybridJointCommand
from typing import Optional, List, Dict, Any, Callable
import time
import threading


class TrajectoryController:
    """
    Trajectory controller class for managing robot movement trajectories.
    
    This class provides methods for executing predefined trajectories,
    custom trajectory planning, and trajectory monitoring.
    """
    
    def __init__(self, robot_controller, model: str):
        """
        Initialize the trajectory controller.
        
        Args:
            robot_controller: Instance of RobotController class
            model: Robot model name. If 'Q5', use '/wr1_controller/commands',
                   otherwise use 'hybrid_body_controller/commands'.
        """
        self.robot_controller = robot_controller
        self.logger = robot_controller.get_logger()
        
        # Action client for simple actions control (zero, lift_up, etc.)
        self.trajectory_action_client = robot_controller.trajectory_action_client
        
        # Publisher for low-level joint commands (hybrid controller)
        if model == "Q5":
            controller_topic = "/wr1_controller/commands"
        else:
            controller_topic = "hybrid_body_controller/commands"
        self.hybrid_cmd_publisher = self.robot_controller.create_publisher(
            HybridJointCommand, controller_topic, 1
        )
        
    
    def set_zero_position(self, duration: float = 4.0) -> bool:
        """
        Set joints to zero position.
        This process takes about 2 seconds.
        
        Args:
            duration: Duration for the trajectory execution
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self.trajectory_action_client.wait_for_server(timeout_sec=10):
                raise RuntimeError("SimpleActions action server not available")

            goal_msg = SimpleActions.Goal()
            goal_msg.action_name = "zero"
            goal_msg.time_cost = float(duration)
            
            self.logger.info(f"Setting joints to zero position (time_cost: {duration}s)...")
            
            # Send goal and wait for result
            future = self.trajectory_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.robot_controller, future)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.logger.error("SimpleActions goal was rejected")
                return False
            
            self.logger.info("SimpleActions goal accepted, waiting for result...")
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.robot_controller, result_future)
            
            result_msg = result_future.result().result
            if result_msg is not None and result_msg.result == result_msg.SUCCESS:
                self.logger.info("Zero position set successfully")
                return True
            else:
                code = getattr(result_msg, "result", None)
                msg = getattr(result_msg, "message", "")
                self.logger.error(f"Failed to set zero position, code={code}, message='{msg}'")
                return False
                
        except Exception as e:
            self.logger.error(f"Error setting zero position: {str(e)}")
            return False
    
    def set_lift_up_position(self, duration: float = 4.0) -> bool:
        """
        Set joints to lift up position.
        This process takes about 2 seconds.
        
        Args:
            duration: Duration for the trajectory execution
            
        Returns:
            True if successful, False otherwise
        """
        try:
            if not self.trajectory_action_client.wait_for_server(timeout_sec=10):
                raise RuntimeError("SimpleActions action server not available")
                
            goal_msg = SimpleActions.Goal()
            goal_msg.action_name = "lift_up"
            goal_msg.time_cost = float(duration)
            
            self.logger.info(f"Setting joints to lift up position (time_cost: {duration}s)...")
            
            # Send goal and wait for result
            future = self.trajectory_action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.robot_controller, future)
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.logger.error("SimpleActions goal was rejected")
                return False
            
            self.logger.info("SimpleActions goal accepted, waiting for result...")
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.robot_controller, result_future)
            
            result_msg = result_future.result().result
            if result_msg is not None and result_msg.result == result_msg.SUCCESS:
                self.logger.info("Lift up position set successfully")
                return True
            else:
                code = getattr(result_msg, "result", None)
                msg = getattr(result_msg, "message", "")
                self.logger.error(f"Failed to set lift up position, code={code}, message='{msg}'")
                return False
                
        except Exception as e:
            self.logger.error(f"Error setting lift up position: {str(e)}")
            return False
    
    def move_to_joint_positions_sync(self,
                                     target_positions: Dict[str, float],
                                     duration: float = 3.0,
                                     kp: Optional[List[float]] = None,
                                     kd: Optional[List[float]] = None,
                                     rate_hz: float = 100.0) -> bool:
        """
        Move specified joints to target positions synchronously using linear interpolation and publish HybridJointCommand.

        Args:
            target_positions: Mapping of joint_name -> target position (rad)
            duration: Time to reach the target in seconds
            kp: Optional per-joint proportional gains (len == num joints). If None, leave empty to use controller defaults.
            kd: Optional per-joint derivative gains (len == num joints). If None, leave empty to use controller defaults.
            rate_hz: Publishing rate for the interpolation loop

        Returns:
            True if completed without timeout, False otherwise
        """
        joint_states = self.robot_controller.get_joint_states()
        if joint_states is None:
            self.logger.error("No joint states received; cannot move joints.")
            return False

        # Snapshot current joint state
        joint_names: List[str] = list(joint_states.name)
        start_positions_map: Dict[str, float] = {n: p for n, p in zip(joint_states.name, joint_states.position)}

        # Build end positions for all known joints; keep unchanged for unspecified joints
        end_positions_map: Dict[str, float] = {}
        for name in joint_names:
            end_positions_map[name] = target_positions.get(name, start_positions_map.get(name, 0.0))

        # Prepare gain arrays if provided; otherwise leave empty to use controller defaults
        use_custom_gains = kp is not None or kd is not None
        if use_custom_gains:
            num_joints = len(joint_names)
            if kp is None:
                kp = [0.0] * num_joints
            if kd is None:
                kd = [0.0] * num_joints
            if len(kp) != num_joints or len(kd) != num_joints:
                self.logger.error("kp/kd length must match number of joints")
                return False

        start_time = time.time()
        end_time = start_time + max(0.01, duration)
        dt = 1.0 / max(1.0, rate_hz)

        self.logger.info(f"Moving {len(target_positions)} joints over {duration:.2f}s...")

        while True:
            now = time.time()
            alpha = 1.0 if now >= end_time else max(0.0, min(1.0, (now - start_time) / (end_time - start_time)))

            cmd = HybridJointCommand()
            cmd.header.stamp = self.robot_controller.get_clock().now().to_msg()
            cmd.joint_name = joint_names

            # Interpolate and fill positions
            positions: List[float] = []
            for name in joint_names:
                p0 = start_positions_map.get(name, 0.0)
                p1 = end_positions_map.get(name, p0)
                positions.append(p0 + (p1 - p0) * alpha)
            cmd.position = positions

            # Zero velocities/feedforward for position hold; controller may ignore if not used
            cmd.velocity = [0.0] * len(joint_names)
            cmd.feedforward = [0.0] * len(joint_names)

            # Optional gains
            if use_custom_gains:
                cmd.kp = kp  # type: ignore[arg-type]
                cmd.kd = kd  # type: ignore[arg-type]

            # Publish
            self.hybrid_cmd_publisher.publish(cmd)

            if alpha >= 1.0:
                break

            rclpy.spin_once(self.robot_controller, timeout_sec=0.0)
            time.sleep(dt)

        self.logger.info("Move-to-position completed.")
        return True
    
    def move_to_joint_positions_async(self,
                                      target_positions: Dict[str, float],
                                      duration: float = 3.0,
                                      kp: Optional[List[float]] = None,
                                      kd: Optional[List[float]] = None,
                                      rate_hz: float = 100.0,
                                      progress_callback: Optional[Callable] = None,
                                      completion_callback: Optional[Callable] = None) -> bool:
        """
        Move specified joints to target positions asynchronously using linear interpolation.

        Args:
            target_positions: Mapping of joint_name -> target position (rad)
            duration: Time to reach the target in seconds
            kp: Optional per-joint proportional gains (len == num joints). If None, leave empty to use controller defaults.
            kd: Optional per-joint derivative gains (len == num joints). If None, leave empty to use controller defaults.
            rate_hz: Publishing rate for the interpolation loop
            progress_callback: Optional callback for progress updates (progress: float)
            completion_callback: Optional callback for completion (success: bool, result: str)

        Returns:
            True if started successfully, False otherwise
        """
        joint_states = self.robot_controller.get_joint_states()
        if joint_states is None:
            self.logger.error("No joint states received; cannot move joints.")
            if completion_callback:
                completion_callback(False, "No joint states available")
            return False

        # Snapshot current joint state
        joint_names: List[str] = list(joint_states.name)
        start_positions_map: Dict[str, float] = {n: p for n, p in zip(joint_states.name, joint_states.position)}

        # Build end positions for all known joints; keep unchanged for unspecified joints
        end_positions_map: Dict[str, float] = {}
        for name in joint_names:
            end_positions_map[name] = target_positions.get(name, start_positions_map.get(name, 0.0))

        # Prepare gain arrays if provided; otherwise leave empty to use controller defaults
        use_custom_gains = kp is not None or kd is not None
        if use_custom_gains:
            num_joints = len(joint_names)
            if kp is None:
                kp = [0.0] * num_joints
            if kd is None:
                kd = [0.0] * num_joints
            if len(kp) != num_joints or len(kd) != num_joints:
                self.logger.error("kp/kd length must match number of joints")
                if completion_callback:
                    completion_callback(False, "Invalid gain parameters")
                return False

        self.logger.info(f"Starting async move of {len(target_positions)} joints over {duration:.2f}s...")
        
        # Start async movement in a separate thread
        import threading
        def async_move():
            try:
                start_time = time.time()
                end_time = start_time + max(0.01, duration)
                dt = 1.0 / max(1.0, rate_hz)

                while True:
                    now = time.time()
                    alpha = 1.0 if now >= end_time else max(0.0, min(1.0, (now - start_time) / (end_time - start_time)))

                    cmd = HybridJointCommand()
                    cmd.header.stamp = self.robot_controller.get_clock().now().to_msg()
                    cmd.joint_name = joint_names

                    # Interpolate and fill positions
                    positions: List[float] = []
                    for name in joint_names:
                        p0 = start_positions_map.get(name, 0.0)
                        p1 = end_positions_map.get(name, p0)
                        positions.append(p0 + (p1 - p0) * alpha)
                    cmd.position = positions

                    # Zero velocities/feedforward for position hold; controller may ignore if not used
                    cmd.velocity = [0.0] * len(joint_names)
                    cmd.feedforward = [0.0] * len(joint_names)

                    # Optional gains
                    if use_custom_gains:
                        cmd.kp = kp  # type: ignore[arg-type]
                        cmd.kd = kd  # type: ignore[arg-type]

                    # Publish
                    self.hybrid_cmd_publisher.publish(cmd)

                    # Call progress callback
                    if progress_callback:
                        progress_callback(alpha)

                    if alpha >= 1.0:
                        break

                    rclpy.spin_once(self.robot_controller, timeout_sec=0.0)
                    time.sleep(dt)

                self.logger.info("Async move-to-position completed.")
                if completion_callback:
                    completion_callback(True, "Move completed successfully")
                    
            except Exception as e:
                self.logger.error(f"Error in async move: {str(e)}")
                if completion_callback:
                    completion_callback(False, f"Move failed: {str(e)}")

        # Start the async movement
        move_thread = threading.Thread(target=async_move)
        move_thread.daemon = True
        move_thread.start()
        
        return True
    
    def move_to_joint_positions(self,
                                target_positions: Dict[str, float],
                                duration: float = 3.0,
                                kp: Optional[List[float]] = None,
                                kd: Optional[List[float]] = None,
                                rate_hz: float = 100.0,
                                async_execution: bool = False,
                                progress_callback: Optional[Callable] = None,
                                completion_callback: Optional[Callable] = None) -> bool:
        """
        Move specified joints to target positions. Convenience method that calls sync or async version.

        Args:
            target_positions: Mapping of joint_name -> target position (rad)
            duration: Time to reach the target in seconds
            kp: Optional per-joint proportional gains (len == num joints). If None, leave empty to use controller defaults.
            kd: Optional per-joint derivative gains (len == num joints). If None, leave empty to use controller defaults.
            rate_hz: Publishing rate for the interpolation loop
            async_execution: If True, use async execution; if False, use sync execution
            progress_callback: Optional callback for progress updates (only used in async mode)
            completion_callback: Optional callback for completion (only used in async mode)

        Returns:
            True if started/completed successfully, False otherwise
        """
        if async_execution:
            return self.move_to_joint_positions_async(
                target_positions, duration, kp, kd, rate_hz, progress_callback, completion_callback
            )
        else:
            return self.move_to_joint_positions_sync(
                target_positions, duration, kp, kd, rate_hz
            )

