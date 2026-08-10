"""
MPC Controller
Encapsulates MPC start/stop/query via teleoperation service and publishes ServoPose.
"""

import json
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from xbot_common_interfaces.srv import StringMessage
from xbot_common_interfaces.msg import ServoPose


class MPCController:
    """
    Controller for MPC operations and ServoPose publishing.

    This class uses the provided RobotController node to create service clients and publishers.
    """

    def __init__(self, robot_controller: Node):
        self.node = robot_controller
        self.logger = robot_controller.get_logger()
        self.callback_group = ReentrantCallbackGroup()

        # Teleoperation service for MPC start/stop/query
        self.teleop_client = self.node.create_client(
            StringMessage, '/teleoperation/service', callback_group=self.callback_group
        )

        # Servo pose publisher
        self.servo_pose_pub = self.node.create_publisher(ServoPose, '/servo_poses', 10)

    # -------------------- Teleoperation helpers --------------------
    def _wait_for_service(self, client, name: str, timeout: float = 10.0) -> bool:
        if client.wait_for_service(timeout_sec=timeout):
            return True
        self.logger.error(f"Service {name} not available")
        return False

    def _call_teleop(self, payload: dict, timeout: float = 10.0) -> Optional[StringMessage.Response]:
        if not self._wait_for_service(self.teleop_client, '/teleoperation/service', timeout):
            return None
        request = StringMessage.Request()
        request.data = json.dumps(payload)
        future = self.teleop_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    # -------------------- MPC controls --------------------
    def start_mpc(self) -> bool:
        """Start MPC: sends {"type":"mpc", "message": "{\"command\":\"start\"}"}."""
        payload = {"type": "mpc", "message": json.dumps({"command": "start"})}
        resp = self._call_teleop(payload)
        ok = resp is not None and getattr(resp, 'result', False)
        self.logger.info("MPC start: %s" % ("OK" if ok else "FAILED"))
        time.sleep(3)
        return ok

    def stop_mpc(self) -> bool:
        payload = {"type": "mpc", "message": json.dumps({"command": "stop"})}
        resp = self._call_teleop(payload)
        ok = resp is not None and getattr(resp, 'result', False)
        self.logger.info("MPC stop: %s" % ("OK" if ok else "FAILED"))
        return ok

    def query_mpc(self) -> Optional[int]:
        """Query MPC state. Returns 1 if started, 0 if stopped, or None on error."""
        payload = {"type": "mpc", "message": json.dumps({"command": "query"})}
        resp = self._call_teleop(payload)
        if resp is None:
            return None
        # Some implementations return integer status in message; attempt to parse
        try:
            # resp.message could be JSON or plain; try parse first
            data = json.loads(resp.message) if resp.message else {}
            status = data.get('status')
            if isinstance(status, int):
                return status
        except Exception:
            pass
        # Fallback: interpret result boolean
        return 1 if getattr(resp, 'result', False) else 0

    def publish_servo_pose(self, msg: ServoPose, rate_hz: float = 80.0, duration: float = 2.0):
        """Publish ServoPose at rate for duration seconds."""
        dt = 1.0 / max(1.0, rate_hz)
        end_time = time.time() + max(0.0, duration)
        while time.time() < end_time and rclpy.ok():
            # ServoPose itself does not have a header field; stamp the inner PoseStamped headers instead.
            now_stamp = self.node.get_clock().now().to_msg()
            if msg.left_pose is not None:
                msg.left_pose.header.stamp = now_stamp
            if msg.right_pose is not None:
                msg.right_pose.header.stamp = now_stamp
            if msg.head_pose is not None:
                msg.head_pose.header.stamp = now_stamp

            self.servo_pose_pub.publish(msg)
            time.sleep(dt)
