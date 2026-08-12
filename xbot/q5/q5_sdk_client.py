"""
q5_sdk_client.py — Q5 shared ROS2 client (read-only).

Subscribes to Q5 ROS2 topics (/joint_states, /battery_state, IMU streams, etc.)
and provides a thread-safe snapshot() dict. All plugins read from this single
snapshot instead of each maintaining their own subscriptions.
"""

from __future__ import annotations

import math
import threading
import time

# Message freshness threshold (ms)
STALE_THRESHOLD_MS = 5000

_SENSOR_TOPICS = {
    "battery": "/battery_state",
    "imu_accel": "/camera/camera/accel/sample",
    "imu_gyro": "/camera/camera/gyro/sample",
    "fault_array": "/fault_array",
    "hand_sensor": "/hand_sensor",
    "odom": "/wr1_base_drive_controller/odom",
}


class Q5SdkClient:
    """Q5 read-only ROS2 client: subscribes to topics → thread-safe snapshot()."""

    def __init__(self, joint_state_position_unit: str = "radians"):
        if joint_state_position_unit not in ("degrees", "radians"):
            raise ValueError("joint_state_position_unit must be 'degrees' or 'radians'")
        self.available = False
        self._lock = threading.Lock()
        self._running = False
        self._node = None
        self._snapshot: dict = {"fresh": False}
        self._last_joint_stamp = 0.0
        self._sensor_snapshots = {}
        self._last_sensor_received = {}
        self._lifecycle_state = "unknown"
        self._lifecycle_source = "unavailable"
        self._lifecycle_client = None
        self._lifecycle_request_type = None
        self._lifecycle_request_pending = False
        self._executor = None
        self._joint_state_position_unit = joint_state_position_unit

    def _init_ros2(self, executor):
        if executor is None:
            return
        try:
            from rclpy.node import Node
            from sensor_msgs.msg import JointState
            from sensor_msgs.msg import BatteryState, Imu
            from nav_msgs.msg import Odometry
            from xbot_common_interfaces.msg import FaultArray, HandXd12
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy
            from lifecycle_msgs.srv import GetState

            self._node = Node("q5_sdk_client")
            # A BEST_EFFORT subscription is compatible with both Q5 publisher
            # modes. A RELIABLE subscription cannot receive a BEST_EFFORT
            # high-rate /joint_states publisher, which otherwise leaves every
            # motion card stuck at fresh=false.
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

            self._node.create_subscription(
                JointState, "/joint_states", self._on_joint_state, qos)
            self._node.create_subscription(
                BatteryState, _SENSOR_TOPICS["battery"], self._on_battery, qos)
            self._node.create_subscription(
                Imu, _SENSOR_TOPICS["imu_accel"], self._on_imu_accel, qos)
            self._node.create_subscription(
                Imu, _SENSOR_TOPICS["imu_gyro"], self._on_imu_gyro, qos)
            self._node.create_subscription(
                FaultArray, _SENSOR_TOPICS["fault_array"], self._on_fault_array, qos)
            self._node.create_subscription(
                HandXd12, _SENSOR_TOPICS["hand_sensor"], self._on_hand_sensor, qos)
            self._node.create_subscription(
                Odometry, _SENSOR_TOPICS["odom"], self._on_odom, qos)
            self._lifecycle_client = self._node.create_client(
                GetState, "/motion_manager/get_state")
            self._lifecycle_request_type = GetState.Request
            self._node.create_timer(1.0, self._refresh_lifecycle_state)

            # RobotStatus is vendor-defined and published with TRANSIENT_LOCAL
            # durability.
            try:
                from xbot_common_interfaces.msg import RobotStatus

                status_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self._node.create_subscription(
                    RobotStatus, "/xbot_state", self._on_robot_status, status_qos)
            except Exception as e:
                print(f"[Q5SdkClient] RobotStatus unavailable: {e}", flush=True)

            executor.add_node(self._node)
            self._executor = executor
            self.available = True
            print("[Q5SdkClient] ROS2 subscriptions ready (/joint_states)", flush=True)
        except Exception as e:
            print(f"[Q5SdkClient] STUB (ROS2 subscription unavailable: {e})", flush=True)

    def _refresh_lifecycle_state(self):
        """Poll Q5 motion_manager lifecycle without invoking an initialization action."""
        client = self._lifecycle_client
        if client is None or self._lifecycle_request_pending:
            return
        if not client.service_is_ready():
            with self._lock:
                self._lifecycle_source = "service_unavailable"
            return
        try:
            future = client.call_async(self._lifecycle_request_type())
            self._lifecycle_request_pending = True
            future.add_done_callback(self._on_lifecycle_state)
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(f"lifecycle query failed: {e}")

    def _on_lifecycle_state(self, future):
        try:
            response = future.result()
            state = getattr(response, "current_state", None)
            label = str(getattr(state, "label", "unknown") or "unknown")
            with self._lock:
                self._lifecycle_state = label
                self._lifecycle_source = "/motion_manager/get_state"
        except Exception as e:
            if self._node is not None:
                self._node.get_logger().warn(f"lifecycle response failed: {e}")
        finally:
            self._lifecycle_request_pending = False

    def _on_joint_state(self, msg):
        """JointState callback → update snapshot."""
        try:
            joint_map = {}
            velocity_map = {}
            effort_map = {}
            joint_count = len(msg.name)

            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    position = float(msg.position[i])
                    joint_map[name] = math.radians(position) if self._joint_state_position_unit == "degrees" else position
                if i < len(msg.velocity):
                    velocity = float(msg.velocity[i])
                    velocity_map[name] = math.radians(velocity) if self._joint_state_position_unit == "degrees" else velocity
                if i < len(msg.effort):
                    effort_map[name] = float(msg.effort[i])

            received_at_ms = int(time.time() * 1000)
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            message_timestamp_ms = None
            if stamp is not None and (stamp.sec or stamp.nanosec):
                message_timestamp_ms = int(stamp.sec * 1000 + stamp.nanosec / 1_000_000)

            with self._lock:
                self._last_joint_stamp = time.time()
                self._snapshot = {
                    "available": True,
                    "fresh": True,
                    "stale": False,
                    "timestamp_ms": received_at_ms,
                    "received_at_ms": received_at_ms,
                    "message_timestamp_ms": message_timestamp_ms,
                    "joints": joint_map,
                    "velocities": velocity_map,
                    "efforts": effort_map,
                    "joint_names": list(msg.name),
                    "joint_count": joint_count,
                    "position_unit": "rad",
                    "source_position_unit": "deg" if self._joint_state_position_unit == "degrees" else "rad",
                    "header_frame": msg.header.frame_id if hasattr(msg, 'header') else "",
                }

            # 首次收到数据或数据有重大变化时打印日志
            if not hasattr(self, '_first_joint_data_received'):
                self._first_joint_data_received = True
                print(f"[Q5SdkClient] JointState received: {joint_count} joints, "
                      f"names={list(msg.name)[:5]}{'...' if len(msg.name)>5 else ''}", flush=True)
        except Exception as e:
            print(f"[Q5SdkClient] _on_joint_state error: {e}", flush=True)

    @staticmethod
    def _message_timestamp_ms(msg):
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is not None and (stamp.sec or stamp.nanosec):
            return int(stamp.sec * 1000 + stamp.nanosec / 1_000_000)
        return None

    @staticmethod
    def _vector(msg, field):
        value = getattr(msg, field, None)
        if value is None:
            return None
        return {"x": float(value.x), "y": float(value.y), "z": float(value.z)}

    def _on_battery(self, msg):
        received_at_ms = int(time.time() * 1000)
        snapshot = {
            "available": True,
            "received_at_ms": received_at_ms,
            "message_timestamp_ms": self._message_timestamp_ms(msg),
            "voltage": float(msg.voltage),
            "temperature": float(msg.temperature),
            "current": float(msg.current),
            "charge": float(msg.charge),
            "capacity": float(msg.capacity),
            "design_capacity": float(msg.design_capacity),
            "percentage": float(msg.percentage),
            "power_supply_status": int(msg.power_supply_status),
            "power_supply_health": int(msg.power_supply_health),
            "power_supply_technology": int(msg.power_supply_technology),
            "present": bool(msg.present),
            "location": str(msg.location),
            "serial_number": str(msg.serial_number),
        }
        with self._lock:
            self._sensor_snapshots["battery"] = snapshot
            self._last_sensor_received["battery"] = time.time()

    def _on_robot_status(self, msg):
        received_at_ms = int(time.time() * 1000)
        snapshot = {
            "available": True,
            "received_at_ms": received_at_ms,
            "state": int(msg.state),
            "message": str(msg.msg),
        }
        with self._lock:
            self._sensor_snapshots["robot_status"] = snapshot
            self._last_sensor_received["robot_status"] = time.time()

    def _on_fault_array(self, msg):
        received_at_ms = int(time.time() * 1000)
        faults = []
        for f in msg.faults:
            faults.append({
                "fault_code": f.fault_code,
                "name": f.name,
                "level": f.level,
                "message": f.message,
                "joint_name": f.joint_name,
                "fault_type": f.fault_type,
                "is_active": f.is_active,
            })
        snapshot = {
            "available": True,
            "received_at_ms": received_at_ms,
            "fault_count": len(faults),
            "faults": faults,
        }
        with self._lock:
            self._sensor_snapshots["faults"] = snapshot
            self._last_sensor_received["faults"] = time.time()

    def _on_hand_sensor(self, msg):
        received_at_ms = int(time.time() * 1000)
        data = {
            "left": {
                "thumb": list(msg.lefthumb),
                "index": list(msg.leftindex),
                "mid": list(msg.leftmid),
                "pinky": list(msg.leftpinky),
                "ring": list(msg.leftring),
            },
            "right": {
                "thumb": list(msg.righthumb),
                "index": list(msg.rightindex),
                "mid": list(msg.rightmid),
                "pinky": list(msg.rightpinky),
                "ring": list(msg.rightring),
            },
        }
        snapshot = {
            "available": True,
            "received_at_ms": received_at_ms,
            "hand_data": data,
        }
        with self._lock:
            self._sensor_snapshots["hand"] = snapshot
            self._last_sensor_received["hand"] = time.time()

    def _on_odom(self, msg):
        received_at_ms = int(time.time() * 1000)
        pose = msg.pose.pose
        twist = msg.twist.twist
        snapshot = {
            "available": True,
            "received_at_ms": received_at_ms,
            "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
            "orientation": {"x": pose.orientation.x, "y": pose.orientation.y,
                            "z": pose.orientation.z, "w": pose.orientation.w},
            "linear_velocity": {"x": twist.linear.x, "y": twist.linear.y, "z": twist.linear.z},
            "angular_velocity": {"x": twist.angular.x, "y": twist.angular.y, "z": twist.angular.z},
        }
        with self._lock:
            self._sensor_snapshots["odom"] = snapshot
            self._last_sensor_received["odom"] = time.time()

    def _on_imu_accel(self, msg):
        self._update_imu("linear_acceleration", self._vector(msg, "linear_acceleration"), msg)

    def _on_imu_gyro(self, msg):
        self._update_imu("angular_velocity", self._vector(msg, "angular_velocity"), msg)

    def _update_imu(self, field, value, msg):
        received_at_ms = int(time.time() * 1000)
        with self._lock:
            snapshot = dict(self._sensor_snapshots.get("imu", {}))
            snapshot.update({
                "available": True,
                "received_at_ms": received_at_ms,
                "message_timestamp_ms": self._message_timestamp_ms(msg),
            })
            if value is not None:
                snapshot[field] = value
            self._sensor_snapshots["imu"] = snapshot
            self._last_sensor_received["imu"] = time.time()

    def start(self, executor=None):
        if not self._running:
            self._init_ros2(executor)
            self._running = self.available

    def stop(self):
        self._running = False
        if self._node is not None:
            try:
                if self._executor is not None:
                    self._executor.remove_node(self._node)
                self._node.destroy_node()
            except Exception:
                pass
            finally:
                self._node = None
                self._executor = None
                self._lifecycle_client = None
                self._lifecycle_request_type = None
                self._lifecycle_request_pending = False

    def snapshot(self) -> dict:
        with self._lock:
            snap = dict(self._snapshot) if self._snapshot else {
                "available": False, "fresh": False, "stale": False, "age_ms": None,
            }

        # Check freshness
        if snap.get("fresh"):
            elapsed_ms = int((time.time() - self._last_joint_stamp) * 1000)
            snap["age_ms"] = elapsed_ms
            if elapsed_ms > STALE_THRESHOLD_MS:
                snap["fresh"] = False
                snap["stale"] = True
            else:
                snap["stale"] = False

        return snap

    def sensor_snapshot(self, name: str) -> dict:
        """Return a sensor snapshot with a local receive-age/freshness verdict."""
        with self._lock:
            snap = dict(self._sensor_snapshots.get(name, {}))
            received = self._last_sensor_received.get(name)
        if not snap or received is None:
            return {"available": False, "fresh": False, "age_ms": None}
        age_ms = int((time.time() - received) * 1000)
        snap["age_ms"] = age_ms
        snap["fresh"] = age_ms <= STALE_THRESHOLD_MS
        if not snap["fresh"]:
            snap["stale"] = True
        else:
            snap["stale"] = False
        return snap

    @property
    def sensor_topics(self):
        return dict(_SENSOR_TOPICS)

    def get_lifecycle_state(self) -> str:
        with self._lock:
            state = self._lifecycle_state

        # 如果状态是 "unknown" 或服务不可用，尝试立即刷新一次
        if state in ("unknown", "service_unavailable") and self._lifecycle_client:
            try:
                if self._lifecycle_client.service_is_ready():
                    future = self._lifecycle_client.call_async(self._lifecycle_request_type())
                    self._lifecycle_request_pending = True
                    # 等待结果
                    rclpy.spin_until_future_complete(self._node, future, timeout_sec=1.0)
                    if future.result():
                        response = future.result()
                        state_label = getattr(response, "current_state", None)
                        label = str(getattr(state_label, "label", "unknown") or "unknown")
                        with self._lock:
                            self._lifecycle_state = label
                            state = label
            except Exception:
                pass

        return state

    def full_snapshot(self) -> dict:
        """Return complete snapshot for bridge worker (joints + all sensors).

        Used by q5_bridge_worker subprocess to publish to Domain 42/ROS2 topics.
        Returns a flat dict with nested sensor data under '_sensor_*' keys.
        """
        with self._lock:
            snap = dict(self._snapshot) if self._snapshot else {}
            # Merge sensor snapshots under '_sensor_' prefix
            for key, val in self._sensor_snapshots.items():
                snap[f"_sensor_{key}"] = dict(val)
            for key, val in self._last_sensor_received.items():
                snap[f"_sensor_received_{key}"] = val

        # Check freshness
        if snap.get("fresh"):
            elapsed_ms = int((time.time() - self._last_joint_stamp) * 1000)
            snap["age_ms"] = elapsed_ms
            snap["fresh"] = elapsed_ms <= STALE_THRESHOLD_MS
            snap["stale"] = not snap["fresh"]
        else:
            snap["fresh"] = False
            snap["stale"] = True

        return snap

    def get_diagnostic_info(self) -> dict:
        """获取诊断信息，帮助调试传感器和lifecycle问题"""
        with self._lock:
            lifecycle_state = self._lifecycle_state
            lifecycle_source = self._lifecycle_source

        joint_snap = self.snapshot()

        # 检查各个传感器的状态
        sensor_states = {}
        for sensor_name in self._sensor_snapshots.keys():
            sensor_states[sensor_name] = self.sensor_snapshot(sensor_name)

        # 计算最后一次收到数据的时间
        last_joint_data = self._last_joint_stamp if hasattr(self, '_last_joint_stamp') else 0
        last_joint_age_ms = int((time.time() - last_joint_data) * 1000) if last_joint_data > 0 else -1

        return {
            "available": self.available,
            "running": self._running,
            "lifecycle_state": lifecycle_state,
            "lifecycle_source": lifecycle_source,
            "joint_data": {
                "available": joint_snap.get("available", False),
                "fresh": joint_snap.get("fresh", False),
                "stale": joint_snap.get("stale", False),
                "age_ms": joint_snap.get("age_ms"),
                "joint_count": joint_snap.get("joint_count", 0),
                "last_received_ms_ago": last_joint_age_ms,
            },
            "sensor_states": sensor_states,
            "node_initialized": self._node is not None,
            "lifecycle_service_available": self._lifecycle_client is not None,
        }
