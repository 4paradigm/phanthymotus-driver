#!/usr/bin/env python3
"""
drivers/unitree/g1/safety_harness.py — SmartMotion 统一运动控制安全层。

单例类，集中管理所有运动控制（直接速度指令 + SLAM自主导航），提供：
  - LiDAR 障碍物感知：前方减速/停止，侧方紧急停止
  - 状态机：IDLE / MOVING / NAVIGATING / NAV_PAUSED
  - 运动事件发布：data/json 格式到 ROS2 topic

此模块不是 MCP plugin，由驱动生命周期管理，默认自动启动。
"""

import enum
import json
import math
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.slam.slam_client import SlamClient


# ── Constants ────────────────────────────────────────────────────────────────

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)


# ── Enums & Data Classes ─────────────────────────────────────────────────────

class MotionState(enum.Enum):
    IDLE = "idle"
    MOVING = "moving"
    NAVIGATING = "navigating"
    NAV_PAUSED = "nav_paused"


class StopReason(enum.Enum):
    COMMAND = "command"
    OBSTACLE = "obstacle"
    DURATION_EXPIRED = "duration_expired"
    NAV_COMPLETE = "nav_complete"


class SpeedZone(enum.Enum):
    NORMAL = "normal"
    DECELERATED = "decelerated"
    STOPPED = "stopped"


@dataclass
class SpeedLimits:
    vx_normal: float = 1.0
    vx_max: float = 1.2
    vx_decel: float = 0.5
    vy_normal: float = 0.2
    vy_max: float = 0.4
    vy_decel: float = 0.1
    vyaw_normal: float = 0.4
    vyaw_max: float = 1.5
    vyaw_decel: float = 0.2


@dataclass
class MotionCommand:
    vx: float = 0.0
    vy: float = 0.0
    vyaw: float = 0.0
    duration: float = -1.0
    start_time: float = 0.0
    estimated_end_time: Optional[float] = None


@dataclass
class NavCommand:
    target_name: str = ""
    target_pose: dict = field(default_factory=dict)
    start_time: float = 0.0


# ── Internal ROS2 Node ───────────────────────────────────────────────────────

class _SafetyHarnessNode(Node):
    """ROS2 node for publishing motion events."""

    def __init__(self, namespace: str):
        super().__init__("g1_safety_harness")
        self._event_pub = self.create_publisher(
            String, f"/{namespace}/safety/motion_events", _LOW_LAT_QOS
        )
        self.get_logger().info(f"SafetyHarnessNode ready — topic: /{namespace}/safety/motion_events")

    def publish_event(self, event: dict) -> None:
        msg = String()
        msg.data = json.dumps(event)
        self._event_pub.publish(msg)


# ── SmartMotion Singleton ────────────────────────────────────────────────────

class SmartMotion:
    """Singleton safety harness for G1 locomotion and navigation."""

    _instance: Optional["SmartMotion"] = None
    _singleton_lock = threading.Lock()

    def __new__(cls, *args, **kwargs):
        with cls._singleton_lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
            return cls._instance

    def __init__(self, loco_client: LocoClient, slam_client: SlamClient,
                 namespace: str, executor, config: dict):
        if hasattr(self, "_initialized"):
            return
        self._initialized = True

        self._loco_client = loco_client
        self._slam_client = slam_client
        self._namespace = namespace
        self._executor = executor

        # Config
        self._decel_threshold = config.get("decel_threshold", 2.0)
        self._stop_threshold = config.get("stop_threshold", 0.8)
        self._lateral_threshold = config.get("lateral_threshold", 1.5)
        self._cone_half_angle = math.radians(config.get("cone_half_angle", 30))
        self._z_min = config.get("z_min", 0.1)
        self._z_max = config.get("z_max", 1.8)

        # State
        self._state = MotionState.IDLE
        self._current_cmd: Optional[MotionCommand] = None
        self._nav_cmd: Optional[NavCommand] = None
        self._speed_zone = SpeedZone.NORMAL
        self._limits = SpeedLimits()
        self._state_lock = threading.Lock()

        # Obstacle detection results
        self._min_obstacle_dist: float = float("inf")
        self._min_obstacle_angle: float = 0.0
        self._lateral_obstacle: bool = False
        self._obstacle_lock = threading.Lock()

        # Duration timer
        self._move_timer: Optional[threading.Timer] = None

        # ROS2 node
        self._node = _SafetyHarnessNode(namespace)
        executor.add_node(self._node)

        # LiDAR subscription
        self._setup_lidar_subscription()

        # Background processing thread
        self._running = True
        self._process_thread = threading.Thread(
            target=self._obstacle_loop, daemon=True, name="safety_harness"
        )
        self._process_thread.start()

        print(f"[SmartMotion] initialized — decel={self._decel_threshold}m, stop={self._stop_threshold}m")

    # ── Public API: Direct Motion ────────────────────────────────────────

    def move(self, vx: float, vy: float, vyaw: float, duration: float = -1.0) -> dict:
        """Issue a move command through the safety harness."""
        with self._state_lock:
            # If navigating, stop navigation first
            if self._state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
                self._do_stop_nav_locked()

            # Cancel existing timer
            if self._move_timer:
                self._move_timer.cancel()
                self._move_timer = None

            # Publish new_command event if already moving
            previous = None
            if self._state == MotionState.MOVING and self._current_cmd:
                previous = {
                    "vx": self._current_cmd.vx,
                    "vy": self._current_cmd.vy,
                    "vyaw": self._current_cmd.vyaw,
                }

            # Clamp velocities
            clamped_vx, clamped_vy, clamped_vyaw = self._clamp(vx, vy, vyaw)

            # Store command (with original requested values for resume after decel)
            now = time.time()
            end_time = (now + duration) if duration > 0 else None
            self._current_cmd = MotionCommand(
                vx=vx, vy=vy, vyaw=vyaw,
                duration=duration, start_time=now, estimated_end_time=end_time,
            )
            self._state = MotionState.MOVING
            self._speed_zone = SpeedZone.NORMAL

            # Execute movement
            ret = self._loco_client.Move(clamped_vx, clamped_vy, clamped_vyaw, True)

            # Set duration timer
            if duration > 0:
                self._move_timer = threading.Timer(duration, self._duration_expired)
                self._move_timer.start()

            # Publish events
            if previous:
                self._publish_event("new_command", {
                    "previous": previous,
                    "new": {"vx": clamped_vx, "vy": clamped_vy, "vyaw": clamped_vyaw},
                })
            else:
                self._publish_event("motion_start", {
                    "params": {"vx": clamped_vx, "vy": clamped_vy, "vyaw": clamped_vyaw, "duration": duration},
                })

            return {
                "ret": ret, "vx": clamped_vx, "vy": clamped_vy, "vyaw": clamped_vyaw,
                "duration": duration, "state": self._state.value,
            }

    def stop(self, reason: StopReason = StopReason.COMMAND) -> dict:
        """Stop all movement."""
        with self._state_lock:
            return self._do_stop_locked(reason)

    # ── Public API: Navigation ───────────────────────────────────────────

    def navigate_to(self, x: float, y: float, yaw: float, target_name: str = "") -> dict:
        """Issue a navigation command through the safety harness."""
        with self._state_lock:
            # If currently moving, stop first
            if self._state == MotionState.MOVING:
                self._do_stop_locked(StopReason.COMMAND)
            # If already navigating, stop nav first
            elif self._state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
                self._do_stop_nav_locked()

            # Convert yaw to quaternion
            q_z = math.sin(yaw / 2)
            q_w = math.cos(yaw / 2)

            # Execute navigation
            code, resp = self._slam_client.NavigateTo(x, y, 0, 0, 0, q_z, q_w)

            if code != 0:
                return {"error": f"NavigateTo failed, code={code}", "response": resp}

            # Update state
            now = time.time()
            label = target_name or f"({x:.1f}, {y:.1f})"
            self._nav_cmd = NavCommand(
                target_name=label,
                target_pose={"x": x, "y": y, "yaw": yaw},
                start_time=now,
            )
            self._state = MotionState.NAVIGATING
            self._speed_zone = SpeedZone.NORMAL

            self._publish_event("nav_start", {
                "target_name": label,
                "target_pose": {"x": x, "y": y, "yaw": yaw},
            })

            return {"status": "navigating", "target": label, "pose": {"x": x, "y": y, "yaw": yaw}}

    def pause_nav(self, reason: StopReason = StopReason.COMMAND) -> dict:
        """Pause current navigation."""
        with self._state_lock:
            if self._state != MotionState.NAVIGATING:
                return {"error": f"Cannot pause nav: state is {self._state.value}"}

            code, resp = self._slam_client.PauseNav()
            self._state = MotionState.NAV_PAUSED

            obstacle_dist = None
            if reason == StopReason.OBSTACLE:
                with self._obstacle_lock:
                    obstacle_dist = self._min_obstacle_dist

            self._publish_event("nav_paused", {
                "reason": reason.value,
                **({"obstacle_distance": round(obstacle_dist, 2)} if obstacle_dist is not None else {}),
            })

            return {"status": "paused"} if code == 0 else {"error": f"PauseNav failed, code={code}"}

    def resume_nav(self) -> dict:
        """Resume paused navigation."""
        with self._state_lock:
            if self._state != MotionState.NAV_PAUSED:
                return {"error": f"Cannot resume nav: state is {self._state.value}"}

            code, resp = self._slam_client.ResumeNav()
            self._state = MotionState.NAVIGATING

            self._publish_event("nav_resumed", {})

            return {"status": "resumed"} if code == 0 else {"error": f"ResumeNav failed, code={code}"}

    def stop_nav(self) -> dict:
        """Stop and cancel navigation."""
        with self._state_lock:
            return self._do_stop_nav_locked()

    # ── Public API: State ────────────────────────────────────────────────

    def get_state(self) -> dict:
        """Return current motion state."""
        with self._state_lock:
            result = {"state": self._state.value, "speed_zone": self._speed_zone.value}
            if self._current_cmd and self._state == MotionState.MOVING:
                cmd = self._current_cmd
                result["motion"] = {
                    "vx": cmd.vx, "vy": cmd.vy, "vyaw": cmd.vyaw,
                    "duration": cmd.duration,
                    "start_time": cmd.start_time,
                    "estimated_end_time": cmd.estimated_end_time,
                }
            if self._nav_cmd and self._state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
                result["navigation"] = {
                    "target_name": self._nav_cmd.target_name,
                    "target_pose": self._nav_cmd.target_pose,
                    "start_time": self._nav_cmd.start_time,
                }
        with self._obstacle_lock:
            result["obstacle_distance"] = round(self._min_obstacle_dist, 2) if self._min_obstacle_dist != float("inf") else None
            result["lateral_obstacle"] = self._lateral_obstacle
        return result

    def shutdown(self) -> None:
        """Clean shutdown."""
        self._running = False
        with self._state_lock:
            if self._move_timer:
                self._move_timer.cancel()
                self._move_timer = None
            if self._state == MotionState.MOVING:
                self._loco_client.StopMove()
            elif self._state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
                try:
                    self._slam_client.PauseNav()
                except Exception:
                    pass
            self._state = MotionState.IDLE
        print("[SmartMotion] shutdown complete")

    # ── Internal: Stop helpers ───────────────────────────────────────────

    def _do_stop_locked(self, reason: StopReason) -> dict:
        """Stop movement (must hold _state_lock)."""
        if self._move_timer:
            self._move_timer.cancel()
            self._move_timer = None

        self._loco_client.StopMove()

        was_moving = self._state == MotionState.MOVING
        self._state = MotionState.IDLE
        self._current_cmd = None
        self._speed_zone = SpeedZone.NORMAL

        if was_moving:
            event_data = {"reason": reason.value}
            if reason == StopReason.OBSTACLE:
                with self._obstacle_lock:
                    event_data["obstacle_distance"] = round(self._min_obstacle_dist, 2)
                    event_data["obstacle_angle_deg"] = round(math.degrees(self._min_obstacle_angle), 1)
            self._publish_event("motion_stop", event_data)

        return {"ret": 0, "state": "idle", "reason": reason.value}

    def _do_stop_nav_locked(self) -> dict:
        """Stop navigation (must hold _state_lock)."""
        try:
            self._slam_client.PauseNav()
        except Exception:
            pass

        was_nav = self._state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED)
        self._state = MotionState.IDLE
        self._nav_cmd = None
        self._speed_zone = SpeedZone.NORMAL

        if was_nav:
            self._publish_event("nav_stopped", {"reason": "command"})

        return {"status": "stopped"}

    # ── Internal: LiDAR Subscription ─────────────────────────────────────

    def _setup_lidar_subscription(self) -> None:
        """Subscribe to DDS rt/utlidar/cloud_livox_mid360 directly."""
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
            self._cloud_sub = ChannelSubscriber("rt/utlidar/cloud_livox_mid360", PointCloud2_)
            self._cloud_sub.Init(self._on_cloud, 10)
            print("[SmartMotion] subscribed to rt/utlidar/cloud_livox_mid360")
        except Exception as e:
            print(f"[SmartMotion] WARNING: failed to subscribe LiDAR: {e}")
            print("[SmartMotion] obstacle detection disabled — running without LiDAR safety")

    def _on_cloud(self, msg) -> None:
        """DDS callback: parse point cloud and compute obstacle distances.

        Uses numpy for fast vectorized processing. All points are evaluated —
        no downsampling — to reliably detect small obstacles like human legs.
        """
        import numpy as np

        # Determine current motion heading
        with self._state_lock:
            state = self._state
            if state == MotionState.MOVING and self._current_cmd:
                heading = math.atan2(self._current_cmd.vy, self._current_cmd.vx) \
                    if (abs(self._current_cmd.vx) > 0.01 or abs(self._current_cmd.vy) > 0.01) else 0.0
            elif state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
                heading = 0.0
            else:
                heading = 0.0

        # Parse PointCloud2 field offsets
        field_map = {}
        for f in msg.fields:
            field_map[f.name] = f.offset

        x_off = field_map.get("x", 0)
        y_off = field_map.get("y", 4)
        z_off = field_map.get("z", 8)

        point_step = msg.point_step
        total_points = msg.width * msg.height
        data = bytes(msg.data)
        data_len = len(data)

        n_valid = min(total_points, data_len // point_step)
        if n_valid == 0:
            return

        # Reshape raw bytes into (n_valid, point_step) and extract float32 fields
        buf = np.frombuffer(data, dtype=np.uint8, count=n_valid * point_step).reshape(n_valid, point_step)
        px = buf[:, x_off:x_off+4].copy().view(dtype='<f4').flatten()
        py = buf[:, y_off:y_off+4].copy().view(dtype='<f4').flatten()
        pz = buf[:, z_off:z_off+4].copy().view(dtype='<f4').flatten()

        # Vectorized filtering: height, distance
        z_mask = (pz >= self._z_min) & (pz <= self._z_max)
        dist_sq = px * px + py * py
        range_mask = (dist_sq >= 0.04) & (dist_sq <= 6.25)  # 0.2m - 2.5m
        valid = z_mask & range_mask

        if not np.any(valid):
            with self._obstacle_lock:
                self._min_obstacle_dist = float("inf")
                self._min_obstacle_angle = 0.0
                self._lateral_obstacle = False
            return

        vx = px[valid]
        vy = py[valid]
        vdist = np.sqrt(dist_sq[valid])

        # Angle relative to heading — vectorized
        point_angles = np.arctan2(vy, vx)
        angle_diffs = np.abs(np.mod(point_angles - heading + math.pi, 2 * math.pi) - math.pi)

        # Forward cone detection
        cone_half = self._cone_half_angle
        forward_mask = angle_diffs <= cone_half
        min_forward_dist = float("inf")
        min_forward_angle = 0.0

        if np.any(forward_mask):
            forward_dists = vdist[forward_mask]
            min_idx = np.argmin(forward_dists)
            min_forward_dist = float(forward_dists[min_idx])
            min_forward_angle = float(point_angles[forward_mask][min_idx])

        # Lateral cross-traffic (45°-90° from heading, within stop threshold)
        lateral_mask = (angle_diffs >= math.radians(45)) & \
                       (angle_diffs <= math.radians(90)) & \
                       (vdist < self._stop_threshold)
        lateral_detected = bool(np.any(lateral_mask))

        with self._obstacle_lock:
            self._min_obstacle_dist = min_forward_dist
            self._min_obstacle_angle = min_forward_angle
            self._lateral_obstacle = lateral_detected

    # ── Internal: Obstacle Processing Loop ───────────────────────────────

    def _obstacle_loop(self) -> None:
        """Background loop (10Hz): check obstacle state, adjust speed."""
        while self._running:
            time.sleep(0.1)

            with self._obstacle_lock:
                dist = self._min_obstacle_dist
                lateral = self._lateral_obstacle

            with self._state_lock:
                if self._state == MotionState.MOVING:
                    self._process_moving_obstacles(dist, lateral)
                elif self._state == MotionState.NAVIGATING:
                    self._process_nav_obstacles(dist, lateral)
                elif self._state == MotionState.NAV_PAUSED:
                    self._process_nav_paused_obstacles(dist, lateral)

    def _process_moving_obstacles(self, dist: float, lateral: bool) -> None:
        """Handle obstacles while in MOVING state (must hold _state_lock)."""
        if dist <= self._stop_threshold:
            # Emergency stop — forward obstacle too close
            if self._speed_zone != SpeedZone.STOPPED:
                self._speed_zone = SpeedZone.STOPPED
                self._do_stop_locked(StopReason.OBSTACLE)
        elif dist <= self._decel_threshold or lateral:
            # Decelerate — forward obstacle in warning zone OR lateral cross-traffic
            if self._speed_zone != SpeedZone.DECELERATED:
                self._speed_zone = SpeedZone.DECELERATED
                self._apply_deceleration()
        else:
            # Normal speed
            if self._speed_zone == SpeedZone.DECELERATED:
                self._speed_zone = SpeedZone.NORMAL
                self._apply_resume()

    def _process_nav_obstacles(self, dist: float, lateral: bool) -> None:
        """Handle obstacles while NAVIGATING (must hold _state_lock)."""
        if dist <= self._stop_threshold or lateral:
            # Emergency pause navigation
            try:
                self._slam_client.PauseNav()
            except Exception:
                pass
            self._state = MotionState.NAV_PAUSED
            self._publish_event("nav_paused", {
                "reason": "obstacle",
                "obstacle_distance": round(dist, 2),
            })

    def _process_nav_paused_obstacles(self, dist: float, lateral: bool) -> None:
        """Handle obstacle clearing while NAV_PAUSED (must hold _state_lock)."""
        if dist > self._decel_threshold and not lateral:
            # Obstacle cleared, resume navigation
            try:
                self._slam_client.ResumeNav()
            except Exception:
                pass
            self._state = MotionState.NAVIGATING
            self._publish_event("nav_resumed", {})

    def _apply_deceleration(self) -> None:
        """Re-issue Move with decelerated speeds (must hold _state_lock)."""
        if not self._current_cmd:
            return
        cmd = self._current_cmd
        decel_vx = max(-self._limits.vx_decel, min(self._limits.vx_decel, cmd.vx))
        decel_vy = max(-self._limits.vy_decel, min(self._limits.vy_decel, cmd.vy))
        decel_vyaw = max(-self._limits.vyaw_decel, min(self._limits.vyaw_decel, cmd.vyaw))

        self._loco_client.Move(decel_vx, decel_vy, decel_vyaw, True)

        with self._obstacle_lock:
            obs_dist = self._min_obstacle_dist
            obs_angle = self._min_obstacle_angle

        self._publish_event("motion_decelerate", {
            "obstacle_distance": round(obs_dist, 2),
            "obstacle_angle_deg": round(math.degrees(obs_angle), 1),
            "original_speed": {"vx": cmd.vx, "vy": cmd.vy, "vyaw": cmd.vyaw},
            "new_speed": {"vx": decel_vx, "vy": decel_vy, "vyaw": decel_vyaw},
        })

    def _apply_resume(self) -> None:
        """Re-issue Move with original speeds (must hold _state_lock)."""
        if not self._current_cmd:
            return
        cmd = self._current_cmd
        clamped_vx, clamped_vy, clamped_vyaw = self._clamp(cmd.vx, cmd.vy, cmd.vyaw)

        self._loco_client.Move(clamped_vx, clamped_vy, clamped_vyaw, True)

        self._publish_event("motion_resume", {
            "speed": {"vx": clamped_vx, "vy": clamped_vy, "vyaw": clamped_vyaw},
        })

    # ── Internal: Velocity Clamping ──────────────────────────────────────

    def _clamp(self, vx: float, vy: float, vyaw: float) -> tuple:
        """Clamp velocities to current zone limits."""
        if self._speed_zone == SpeedZone.DECELERATED:
            vx = max(-self._limits.vx_decel, min(self._limits.vx_decel, vx))
            vy = max(-self._limits.vy_decel, min(self._limits.vy_decel, vy))
            vyaw = max(-self._limits.vyaw_decel, min(self._limits.vyaw_decel, vyaw))
        else:
            vx = max(-self._limits.vx_max, min(self._limits.vx_max, vx))
            vy = max(-self._limits.vy_max, min(self._limits.vy_max, vy))
            vyaw = max(-self._limits.vyaw_max, min(self._limits.vyaw_max, vyaw))
        return vx, vy, vyaw

    # ── Internal: Event Publishing ───────────────────────────────────────

    def _publish_event(self, event_type: str, data: dict) -> None:
        """Publish motion event to ROS2 topic."""
        event = {"type": event_type, "timestamp": time.time(), **data}
        self._node.publish_event(event)
        print(f"[SmartMotion] event: {event_type} | {json.dumps(data)}")

    # ── Internal: Duration Timer ─────────────────────────────────────────

    def _duration_expired(self) -> None:
        """Timer callback when move duration expires."""
        with self._state_lock:
            self._move_timer = None
            if self._state == MotionState.MOVING:
                self._do_stop_locked(StopReason.DURATION_EXPIRED)
