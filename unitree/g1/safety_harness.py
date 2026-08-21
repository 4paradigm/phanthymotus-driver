#!/usr/bin/env python3
"""
drivers/unitree/g1/safety_harness.py — SmartMotion 独立进程安全层。

架构：
  - SmartMotionProcess: 独立子进程，拥有自己的 DDS 通道和 ROS2 节点
    - 订阅 LiDAR 点云，10Hz 全量 numpy 处理
    - 执行 LocoClient / SlamClient RPC 调用
    - 发布运动事件到 ROS2 topic
  - SmartMotionProxy: 主进程中的轻量代理，通过 multiprocessing Queue 通信
    - 对外暴露与原 SmartMotion 相同的 API
    - 非阻塞命令发送，同步等待结果

此模块不是 MCP plugin，由驱动生命周期管理，默认自动启动。
"""

import enum
import json
import math
import multiprocessing as mp
import os
import queue
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional


# ── Enums & Data Classes (shared between processes) ──────────────────────────

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
    TILT = "tilt"
    FOOT_AIRBORNE = "foot_airborne"
    COMM_TIMEOUT = "comm_timeout"
    JOINT_OVERHEAT = "joint_overheat"


class SpeedZone(enum.Enum):
    NORMAL = "normal"
    DECELERATED = "decelerated"
    STOPPED = "stopped"


@dataclass
class SpeedLimits:
    vx_normal: float = 1.0
    vx_max: float = 5.0
    vx_decel: float = 0.5
    vy_normal: float = 0.2
    vy_max: float = 5.0
    vy_decel: float = 0.1
    vyaw_normal: float = 0.4
    vyaw_max: float = 5.0
    vyaw_decel: float = 0.2


# ── SmartMotionProxy (main process) ─────────────────────────────────────────

class SmartMotionProxy:
    """Main-process proxy that communicates with the SmartMotion subprocess."""

    def __init__(self, namespace: str, config: dict, network_iface: str):
        ctx = mp.get_context("spawn")
        self._cmd_queue = ctx.Queue()
        self._result_queue = ctx.Queue()
        self._proc = ctx.Process(
            target=_run_smart_motion_process,
            args=(namespace, config, network_iface, self._cmd_queue, self._result_queue),
            name="smart_motion", daemon=True,
        )
        self._proc.start()
        print(f"[SmartMotionProxy] subprocess started → pid={self._proc.pid}")
        # Per-request result routing: avoid race when multiple threads call _call
        self._pending: dict[str, queue.Queue] = {}  # req_id → per-request queue
        self._dispatch_lock = threading.Lock()
        self._dispatch_thread = threading.Thread(target=self._dispatch_results, daemon=True)
        self._dispatch_thread.start()
        self._req_counter = 0
        self._req_lock = threading.Lock()

    def _next_req_id(self) -> str:
        with self._req_lock:
            self._req_counter += 1
            return f"req_{self._req_counter}"

    def _dispatch_results(self):
        """Background thread that routes results from subprocess to the correct caller."""
        while True:
            try:
                item = self._result_queue.get(timeout=1.0)
                req_id = item.pop("_req_id", None) if isinstance(item, dict) else None
                if req_id and req_id in self._pending:
                    self._pending[req_id].put(item)
                else:
                    # Fallback: shouldn't happen, but don't lose the result
                    # Put it in any waiting queue (legacy behavior)
                    with self._dispatch_lock:
                        for q in self._pending.values():
                            q.put(item)
                            break
            except queue.Empty:
                continue
            except Exception:
                continue

    def _call(self, method: str, timeout: float = 15.0, **kwargs) -> dict:
        """Send command to subprocess and wait for result (thread-safe)."""
        req_id = self._next_req_id()
        result_q = queue.Queue(maxsize=1)
        with self._dispatch_lock:
            self._pending[req_id] = result_q
        try:
            self._cmd_queue.put({"method": method, "_req_id": req_id, **kwargs})
            result = result_q.get(timeout=timeout)
            return result
        except queue.Empty:
            return {"error": f"SmartMotion subprocess timeout ({method})"}
        finally:
            with self._dispatch_lock:
                self._pending.pop(req_id, None)

    def move(self, vx: float, vy: float, vyaw: float, duration: float = -1.0) -> dict:
        return self._call("move", vx=vx, vy=vy, vyaw=vyaw, duration=duration)

    def stop(self, reason: str = "command") -> dict:
        return self._call("stop", reason=reason)

    def navigate_to(self, x: float, y: float, yaw: float, target_name: str = "",
                    speed: float = 0.5, mode: int = 1) -> dict:
        return self._call("navigate_to", x=x, y=y, yaw=yaw, target_name=target_name,
                          speed=speed, mode=mode)

    def pause_nav(self, reason: str = "command") -> dict:
        return self._call("pause_nav", reason=reason)

    def resume_nav(self) -> dict:
        return self._call("resume_nav")

    def stop_nav(self) -> dict:
        return self._call("stop_nav")

    def wait_nav_done(self, stall_timeout: float = 60) -> dict:
        return self._call("wait_nav_done", stall_timeout=stall_timeout,
                          timeout=stall_timeout + 30)

    def get_state(self) -> dict:
        return self._call("get_state")

    # ── SLAM RPC passthrough (single SlamClient owns all SLAM operations) ──

    def start_mapping(self) -> dict:
        return self._call("start_mapping", timeout=15.0)

    def stop_mapping(self, pcd_path: str) -> dict:
        return self._call("stop_mapping", pcd_path=pcd_path, timeout=15.0)

    def init_pose(self, x=0.0, y=0.0, z=0.0, q_x=0.0, q_y=0.0, q_z=0.0, q_w=1.0,
                  address="") -> dict:
        return self._call("init_pose", x=x, y=y, z=z, q_x=q_x, q_y=q_y,
                          q_z=q_z, q_w=q_w, address=address, timeout=15.0)

    def shutdown(self) -> None:
        try:
            self._cmd_queue.put({"method": "shutdown"})
            self._proc.join(timeout=3.0)
        except Exception:
            pass
        if self._proc.is_alive():
            self._proc.terminate()
            self._proc.join(timeout=2.0)
        print("[SmartMotionProxy] subprocess stopped")


# ── SmartMotion subprocess entry ─────────────────────────────────────────────

def _run_smart_motion_process(namespace: str, config: dict, network_iface: str,
                              cmd_queue: mp.Queue, result_queue: mp.Queue):
    """Entry point for the SmartMotion subprocess.

    Initializes its own DDS channel, RPC clients, ROS2 node, and LiDAR subscription.
    Runs independently from the main driver process — no GIL contention.
    """
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String, UInt8MultiArray

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.g1.slam.slam_client import SlamClient

    _QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=200,
        durability=DurabilityPolicy.VOLATILE,
    )

    # ── Initialize DDS ──
    ChannelFactoryInitialize(0, network_iface)
    print(f"[SmartMotion:pid={os.getpid()}] DDS initialized on {network_iface}")

    # ── Initialize RPC clients ──
    loco_client = LocoClient()
    loco_client.Init()
    loco_client.SetTimeout(10.0)

    slam_client = SlamClient()
    slam_client.Init()
    slam_client.SetTimeout(10.0)  # must be AFTER Init() — Init() resets timeout to 5.0

    print(f"[SmartMotion:pid={os.getpid()}] LocoClient + SlamClient ready")

    # ── Initialize ROS2 ──
    rclpy.init()
    executor = SingleThreadedExecutor()

    class _EventNode(Node):
        def __init__(self):
            super().__init__("g1_safety_harness")
            self._pub = self.create_publisher(
                String, f"/{namespace}/safety/motion_events", _QOS
            )

        def publish(self, event: dict):
            msg = String()
            msg.data = json.dumps(event)
            self._pub.publish(msg)

    event_node = _EventNode()
    executor.add_node(event_node)
    print(f"[SmartMotion:pid={os.getpid()}] ROS2 event node ready → /{namespace}/safety/motion_events")

    # ── Config ──
    decel_threshold = config.get("decel_threshold", 2.0)
    stop_threshold = config.get("stop_threshold", 0.8)
    cone_half_angle = math.radians(config.get("cone_half_angle", 30))
    z_min = config.get("z_min", 0.1)
    z_max = config.get("z_max", 1.8)
    limits = SpeedLimits()

    # ── State ──
    state = MotionState.IDLE
    current_cmd = None  # dict: {vx, vy, vyaw, duration, start_time, end_time}
    nav_cmd = None      # dict: {target_name, target_pose, start_time}
    speed_zone = SpeedZone.NORMAL
    move_timer = None   # threading.Timer

    # Obstacle state (written by LiDAR callback, read by main loop)
    obstacle_lock = threading.Lock()
    obstacle_dist = float("inf")
    obstacle_angle = 0.0
    lateral_obstacle = False

    # Nav arrival state (updated by rt/slam_info DDS callback in this subprocess)
    slam_info_lock = threading.Lock()
    nav_arrived_flag = False
    nav_arrived_error = None
    nav_current_pose = None

    # ── Helpers ──
    def clamp(vx, vy, vyaw, zone):
        if zone == SpeedZone.DECELERATED:
            vx = max(-limits.vx_decel, min(limits.vx_decel, vx))
            vy = max(-limits.vy_decel, min(limits.vy_decel, vy))
            vyaw = max(-limits.vyaw_decel, min(limits.vyaw_decel, vyaw))
        else:
            vx = max(-limits.vx_max, min(limits.vx_max, vx))
            vy = max(-limits.vy_max, min(limits.vy_max, vy))
            vyaw = max(-limits.vyaw_max, min(limits.vyaw_max, vyaw))
        return vx, vy, vyaw

    def publish_event(event_type, data):
        event = {"type": event_type, "timestamp": time.time(), **data}
        event_node.publish(event)
        print(f"[SmartMotion] event: {event_type} | {json.dumps(data)}", flush=True)

    def do_stop(reason_str):
        nonlocal state, current_cmd, speed_zone, move_timer, stop_repeat_count
        if move_timer:
            move_timer.cancel()
            move_timer = None
        loco_client.StopMove()
        was_moving = state == MotionState.MOVING
        state = MotionState.IDLE
        current_cmd = None
        speed_zone = SpeedZone.NORMAL
        stop_repeat_count = 3  # repeat StopMove to ensure controller stops
        if was_moving:
            event_data = {"reason": reason_str}
            if reason_str == "obstacle":
                with obstacle_lock:
                    event_data["obstacle_distance"] = round(obstacle_dist, 2)
                    event_data["obstacle_angle_deg"] = round(math.degrees(obstacle_angle), 1)
            publish_event("motion_stop", event_data)
        return {"ret": 0, "state": "idle", "reason": reason_str}

    def do_stop_nav():
        nonlocal state, nav_cmd, speed_zone
        try:
            slam_client.PauseNav()
        except Exception:
            pass
        was_nav = state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED)
        state = MotionState.IDLE
        nav_cmd = None
        speed_zone = SpeedZone.NORMAL
        if was_nav:
            publish_event("nav_stopped", {"reason": "command"})
        return {"status": "stopped"}

    def duration_expired():
        nonlocal state, current_cmd, speed_zone, move_timer
        move_timer = None
        if state == MotionState.MOVING:
            do_stop("duration_expired")

    # ── LiDAR DDS Subscription ──
    def on_cloud(msg):
        nonlocal obstacle_dist, obstacle_angle, lateral_obstacle

        # Get heading
        if state == MotionState.MOVING and current_cmd:
            vx_cmd = current_cmd.get("vx", 0)
            vy_cmd = current_cmd.get("vy", 0)
            heading = math.atan2(vy_cmd, vx_cmd) if (abs(vx_cmd) > 0.01 or abs(vy_cmd) > 0.01) else 0.0
        else:
            # NAVIGATING/IDLE: LiDAR is in body frame, heading=0 = robot forward
            heading = 0.0

        # Parse UInt8MultiArray format: [uint32 point_step][uint32 total_points][raw bytes]
        raw = bytes(msg.data)
        if len(raw) < 8:
            return
        point_step = struct.unpack_from('<I', raw, 0)[0]
        total_points = struct.unpack_from('<I', raw, 4)[0]
        data = raw[8:]

        n_valid = min(total_points, len(data) // point_step)
        if n_valid == 0:
            return

        # Numpy batch extraction (xyz at offsets 0, 4, 8 — already gravity-aligned)
        buf = np.frombuffer(data, dtype=np.uint8, count=n_valid * point_step).reshape(n_valid, point_step)
        px = buf[:, 0:4].copy().view(dtype='<f4').flatten()
        py = buf[:, 4:8].copy().view(dtype='<f4').flatten()
        pz = buf[:, 8:12].copy().view(dtype='<f4').flatten()

        # Vectorized filter
        z_mask = (pz >= z_min) & (pz <= z_max)
        dist_sq = px * px + py * py
        range_mask = (dist_sq >= 0.04) & (dist_sq <= 6.25)
        valid = z_mask & range_mask

        if not np.any(valid):
            with obstacle_lock:
                obstacle_dist = float("inf")
                obstacle_angle = 0.0
                lateral_obstacle = False
            return

        vx_pts = px[valid]
        vy_pts = py[valid]
        vdist = np.sqrt(dist_sq[valid])

        point_angles = np.arctan2(vy_pts, vx_pts)

        # Normal mode: forward cone based on heading
        angle_diffs = np.abs(np.mod(point_angles - heading + math.pi, 2 * math.pi) - math.pi)
        # Forward cone
        forward_mask = angle_diffs <= cone_half_angle
        min_fwd_dist = float("inf")
        min_fwd_angle = 0.0
        if np.any(forward_mask):
            fwd_dists = vdist[forward_mask]
            idx = np.argmin(fwd_dists)
            min_fwd_dist = float(fwd_dists[idx])
            min_fwd_angle = float(point_angles[forward_mask][idx])

            if min_fwd_dist <= decel_threshold and state in (MotionState.MOVING, MotionState.NAVIGATING):
                fwd_x = vx_pts[forward_mask][idx]
                fwd_y = vy_pts[forward_mask][idx]
                fwd_z = pz[valid][forward_mask][idx]
                # This is a per-cloud callback (~10 Hz) and the condition holds for
                # as long as an obstacle stays in range, so log the *entry* into
                # that state unthrottled — that is the operationally interesting
                # event — and then sample the steady-state readout.
                self._fwd_log_n = getattr(self, '_fwd_log_n', 0) + 1
                if self._fwd_log_n == 1 or self._fwd_log_n % 100 == 0:
                    print(f"[SmartMotion:lidar] closest_fwd: dist={min_fwd_dist:.2f}m "
                          f"xyz=({fwd_x:.2f},{fwd_y:.2f},{fwd_z:.2f}) "
                          f"angle={math.degrees(min_fwd_angle):.1f}° "
                          f"total_fwd_pts={int(forward_mask.sum())} "
                          f"heading={math.degrees(heading):.1f}° "
                          f"(n={self._fwd_log_n})", flush=True)
            else:
                # Left the decel zone — reset so re-entry logs immediately.
                self._fwd_log_n = 0

        # Lateral (45°-90°, within stop_threshold)
        lat_mask = (angle_diffs >= math.radians(45)) & \
                   (angle_diffs <= math.radians(90)) & \
                   (vdist < stop_threshold)
        lat_detected = bool(np.any(lat_mask))

        with obstacle_lock:
            obstacle_dist = min_fwd_dist
            obstacle_angle = min_fwd_angle
            lateral_obstacle = lat_detected

    try:
        event_node.create_subscription(
            UInt8MultiArray, f"/{namespace}/lidar/cloud", on_cloud, _QOS)
        print(f"[SmartMotion:pid={os.getpid()}] LiDAR subscribed (ROS2 /{namespace}/lidar/cloud)")
    except Exception as e:
        print(f"[SmartMotion:pid={os.getpid()}] WARNING: LiDAR subscribe failed: {e}")

    # ── Safety monitors config ──
    tilt_threshold = math.radians(config.get("tilt_threshold_deg", 35))
    foot_force_min = config.get("foot_force_min", 10)
    foot_airborne_timeout = config.get("foot_airborne_timeout", 0.2)
    comm_timeout = config.get("comm_timeout", 0.5)
    motor_temp_decel = config.get("motor_temp_decel", 75)
    motor_temp_stop = config.get("motor_temp_stop", 85)

    # Safety state
    safety_lock = threading.Lock()
    last_lowstate_time = time.monotonic()
    tilt_triggered = False
    foot_airborne_start = 0.0
    foot_force_seen_nonzero = False  # only enable airborne detection after seeing real force
    max_motor_temp = 0.0
    last_temp_check = 0.0
    stop_repeat_count = 0  # counter for repeated StopMove after emergency

    def emergency_stop(reason_str, extra=None):
        """Emergency stop with optional damp mode for tilt."""
        nonlocal state, current_cmd, speed_zone, move_timer, stop_repeat_count
        if move_timer:
            move_timer.cancel()
            move_timer = None
        loco_client.StopMove()
        if reason_str == "tilt":
            try:
                loco_client.SetFsmId(1)  # damp mode
            except Exception:
                pass
        was_active = state in (MotionState.MOVING, MotionState.NAVIGATING, MotionState.NAV_PAUSED)
        if state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            try:
                slam_client.PauseNav()
            except Exception:
                pass
        state = MotionState.IDLE
        current_cmd = None
        speed_zone = SpeedZone.NORMAL
        stop_repeat_count = 3  # repeat StopMove in main loop to ensure it takes effect
        if was_active:
            event_data = {"reason": reason_str}
            if extra:
                event_data.update(extra)
            publish_event("safety_stop", event_data)
            print(f"[SmartMotion] emergency_stop({reason_str}): StopMove sent", flush=True)

    # ── LowState DDS Subscription (IMU tilt + joint temp) ──
    def on_lowstate(msg):
        nonlocal last_lowstate_time, tilt_triggered, max_motor_temp, last_temp_check

        now = time.monotonic()
        with safety_lock:
            last_lowstate_time = now

        # Tilt detection (20Hz, every callback)
        imu = msg.imu_state
        roll = abs(float(imu.rpy[0]))
        pitch = abs(float(imu.rpy[1]))

        if (roll > tilt_threshold or pitch > tilt_threshold):
            if not tilt_triggered and state in (MotionState.MOVING, MotionState.NAVIGATING, MotionState.NAV_PAUSED):
                tilt_triggered = True
                emergency_stop("tilt", {
                    "roll_deg": round(math.degrees(roll), 1),
                    "pitch_deg": round(math.degrees(pitch), 1),
                })
        else:
            tilt_triggered = False

        # Joint temperature (1Hz check)
        if now - last_temp_check >= 1.0:
            last_temp_check = now
            temp_max = 0.0
            for m in msg.motor_state:
                for t in m.temperature:
                    if float(t) > temp_max:
                        temp_max = float(t)
            with safety_lock:
                max_motor_temp = temp_max

    try:
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
        lowstate_sub.Init(on_lowstate, 10)
        print(f"[SmartMotion:pid={os.getpid()}] LowState subscribed (tilt + temp)")
    except Exception as e:
        print(f"[SmartMotion:pid={os.getpid()}] WARNING: LowState subscribe failed: {e}")

    # ── OdomState DDS Subscription (foot force) ──
    def on_odom(msg):
        nonlocal foot_airborne_start, foot_force_seen_nonzero

        if state != MotionState.MOVING:
            foot_airborne_start = 0.0
            return

        forces = list(msg.foot_force)
        if len(forces) < 4:
            return

        all_airborne = all(f < foot_force_min for f in forces[:4])

        # Only enable airborne detection after seeing at least one valid (non-zero) reading
        if not foot_force_seen_nonzero:
            if not all_airborne:
                foot_force_seen_nonzero = True
                print(f"[SmartMotion] foot_force sensor active: {[round(f,1) for f in forces[:4]]}", flush=True)
            return  # skip detection until sensor is confirmed working

        if all_airborne:
            now = time.monotonic()
            if foot_airborne_start == 0.0:
                foot_airborne_start = now
            elif now - foot_airborne_start > foot_airborne_timeout:
                airborne_ms = round((now - foot_airborne_start) * 1000)
                foot_airborne_start = 0.0
                emergency_stop("foot_airborne", {
                    "foot_forces": [round(f, 1) for f in forces[:4]],
                    "airborne_duration_ms": airborne_ms,
                })
        else:
            foot_airborne_start = 0.0

    try:
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
        odom_sub = ChannelSubscriber("rt/odommodestate", SportModeState_)
        odom_sub.Init(on_odom, 10)
        print(f"[SmartMotion:pid={os.getpid()}] OdomState subscribed (foot force)")
    except Exception as e:
        print(f"[SmartMotion:pid={os.getpid()}] WARNING: OdomState subscribe failed: {e}")

    # ── SLAM info DDS subscription (for nav arrival/obstacle callbacks) ──
    slam_info_msg_count = 0
    slam_info_pos_count = 0

    def on_slam_info(msg):
        nonlocal nav_arrived_flag, nav_arrived_error, nav_current_pose
        nonlocal slam_info_msg_count, slam_info_pos_count
        slam_info_msg_count += 1
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        msg_type = data.get("type", "")
        if msg_type in ("pos_info", "mapping_info"):
            slam_info_pos_count += 1
            pose_data = data.get("data", {}).get("currentPose")
            if pose_data:
                q_x = float(pose_data.get("q_x", 0.0))
                q_y = float(pose_data.get("q_y", 0.0))
                q_z = float(pose_data.get("q_z", 0.0))
                q_w = float(pose_data.get("q_w", 1.0))
                yaw = math.atan2(
                    2 * (q_w * q_z + q_x * q_y),
                    1 - 2 * (q_y * q_y + q_z * q_z),
                )
                with slam_info_lock:
                    nav_current_pose = {
                        "x": pose_data["x"], "y": pose_data["y"], "yaw": round(yaw, 3)
                    }
                # Debug: log first few pos_info to verify subprocess receives them
                if slam_info_pos_count <= 3:
                    print(f"[SmartMotion] slam_info pos_info #{slam_info_pos_count}: "
                          f"x={pose_data['x']:.3f} y={pose_data['y']:.3f}", flush=True)
        elif msg_type == "ctrl_info":
            # ctrl_info is never published by SLAM service (verified via test_slam_info_logger)
            # Keep handler for future compatibility but don't rely on it
            ctrl_data = data.get("data", {})
            if ctrl_data.get("is_arrived"):
                with slam_info_lock:
                    nav_arrived_flag = True
            obs = ctrl_data.get("obsInfo", {})
            if obs.get("state") and obs.get("time", 0) > 10:
                with slam_info_lock:
                    nav_arrived_error = "blocked by obstacle for >10s"
                    nav_arrived_flag = True

    try:
        from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_ as StringMsg_
        slam_info_sub = ChannelSubscriber("rt/slam_info", StringMsg_)
        slam_info_sub.Init(on_slam_info, 10)
        print(f"[SmartMotion:pid={os.getpid()}] rt/slam_info subscribed (nav arrival)")
    except Exception as e:
        print(f"[SmartMotion:pid={os.getpid()}] WARNING: rt/slam_info subscribe failed: {e}")

    # ── Command handlers ──
    def handle_move(vx, vy, vyaw, duration):
        nonlocal state, current_cmd, speed_zone, move_timer

        if state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            do_stop_nav()
        if move_timer:
            move_timer.cancel()
            move_timer = None

        previous = None
        if state == MotionState.MOVING and current_cmd:
            previous = {"vx": current_cmd["vx"], "vy": current_cmd["vy"], "vyaw": current_cmd["vyaw"]}

        clamped_vx, clamped_vy, clamped_vyaw = clamp(vx, vy, vyaw, SpeedZone.NORMAL)

        now = time.time()
        current_cmd = {
            "vx": vx, "vy": vy, "vyaw": vyaw,
            "duration": duration, "start_time": now,
            "end_time": (now + duration) if duration > 0 else None,
        }
        state = MotionState.MOVING
        speed_zone = SpeedZone.NORMAL

        ret = loco_client.Move(clamped_vx, clamped_vy, clamped_vyaw, True)

        if duration > 0:
            move_timer = threading.Timer(duration, duration_expired)
            move_timer.start()

        if previous:
            publish_event("new_command", {
                "previous": previous,
                "new": {"vx": clamped_vx, "vy": clamped_vy, "vyaw": clamped_vyaw},
            })
        else:
            publish_event("motion_start", {
                "params": {"vx": clamped_vx, "vy": clamped_vy, "vyaw": clamped_vyaw, "duration": duration},
            })

        return {"ret": ret, "vx": clamped_vx, "vy": clamped_vy, "vyaw": clamped_vyaw,
                "duration": duration, "state": state.value}

    def handle_navigate_to(x, y, yaw, target_name, speed=0.5, mode=1, stall_timeout=60):
        nonlocal state, nav_cmd, speed_zone, nav_arrived_flag, nav_arrived_error

        if state == MotionState.MOVING:
            do_stop("command")
        elif state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            do_stop_nav()

        # Reset arrival state for this navigation session
        with slam_info_lock:
            nav_arrived_flag = False
            nav_arrived_error = None

        q_z = math.sin(yaw / 2)
        q_w = math.cos(yaw / 2)
        # Clear any paused state before starting new navigation.
        # If previous nav was paused (by obstacle or user), SLAM service may still
        # be in paused state — NavigateTo would fail with code=3104.
        try:
            slam_client.ResumeNav()
        except Exception:
            pass
        code, resp = slam_client.NavigateTo(x, y, 0, 0, 0, q_z, q_w,
                                              speed=speed, mode=mode)

        if code != 0:
            return {"error": f"NavigateTo failed, code={code}", "response": resp}

        label = target_name or f"({x:.1f}, {y:.1f})"
        nav_cmd = {"target_name": label, "target_pose": {"x": x, "y": y, "yaw": yaw}, "start_time": time.time()}
        state = MotionState.NAVIGATING
        speed_zone = SpeedZone.NORMAL

        publish_event("nav_start", {"target_name": label, "target_pose": {"x": x, "y": y, "yaw": yaw}})
        # Return immediately — non-blocking. wait_navigation_done handles arrival detection.
        return {"status": "navigating", "target": label, "pose": {"x": x, "y": y, "yaw": yaw}}

    def handle_wait_nav_done(stall_timeout=60):
        """Block until navigation completes or robot is stuck.
        Arrival detection: pose-based (distance to target < 0.3m).
        Also checks ctrl_info.is_arrived if SLAM service ever publishes it.
        Runs safety checks and ROS2 spin during the wait."""
        nonlocal state, nav_arrived_flag, nav_arrived_error, nav_cmd, speed_zone, stop_repeat_count
        poll_interval = 0.5
        last_pose = None
        stall_start = time.time()
        wait_start = time.time()

        with slam_info_lock:
            last_pose = dict(nav_current_pose) if nav_current_pose else None

        if last_pose is None:
            print(f"[SmartMotion] wait_nav_done: WARNING no pose data yet "
                  f"(pos_info count={slam_info_pos_count})", flush=True)

        while state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            elapsed = time.time() - wait_start

            with slam_info_lock:
                arrived = nav_arrived_flag
                error = nav_arrived_error
                pose = dict(nav_current_pose) if nav_current_pose else None
                nav_arrived_flag = False
                nav_arrived_error = None

            # ctrl_info-based arrival (secondary — SLAM service may not publish ctrl_info)
            if arrived:
                state = MotionState.IDLE
                nav_cmd = None
                speed_zone = SpeedZone.NORMAL
                if error:
                    return {"status": "error", "error": error}
                print(f"[SmartMotion] wait_nav_done: arrived via ctrl_info "
                      f"after {elapsed:.1f}s, pose={pose}", flush=True)
                return {"status": "arrived", "pose": pose}

            # Pose-based arrival detection (primary mechanism)
            if pose and nav_cmd:
                tx = nav_cmd["target_pose"]["x"]
                ty = nav_cmd["target_pose"]["y"]
                dist = math.sqrt((pose["x"] - tx)**2 + (pose["y"] - ty)**2)
                if dist < 0.3:
                    state = MotionState.IDLE
                    nav_cmd = None
                    speed_zone = SpeedZone.NORMAL
                    print(f"[SmartMotion] wait_nav_done: arrived via pose "
                          f"after {elapsed:.1f}s, dist={dist:.3f}m, pose={pose}", flush=True)
                    return {"status": "arrived", "pose": pose}
                # Debug log every 5s
                if int(elapsed) % 5 == 0 and abs(elapsed - int(elapsed)) < poll_interval:
                    print(f"[SmartMotion] wait_nav_done: {elapsed:.0f}s elapsed, "
                          f"dist={dist:.3f}m, pose={pose}", flush=True)

            # Stall detection — no movement for stall_timeout seconds.
            # Note: NAV_PAUSED (obstacle stop) doesn't count as stall —
            # the robot is intentionally stopped waiting for obstacle to clear.
            if state == MotionState.NAVIGATING and pose and last_pose:
                dx = pose["x"] - last_pose["x"]
                dy = pose["y"] - last_pose["y"]
                moved = math.sqrt(dx * dx + dy * dy)
                if moved > 0.05:
                    stall_start = time.time()
                    last_pose = pose
                else:
                    last_pose = pose

            if state == MotionState.NAVIGATING and time.time() - stall_start > stall_timeout:
                do_stop_nav()
                print(f"[SmartMotion] wait_nav_done: TIMEOUT after {stall_timeout}s "
                      f"no movement, pose={pose}", flush=True)
                return {"status": "timeout",
                        "error": f"No movement for {stall_timeout}s, navigation cancelled",
                        "pose": pose}

            # Run safety checks and ROS2 spin during wait
            process_safety_checks()
            if stop_repeat_count > 0 and state == MotionState.IDLE:
                loco_client.StopMove()
                stop_repeat_count -= 1
            executor.spin_once(timeout_sec=0)

            time.sleep(poll_interval)

        # State changed externally (e.g., emergency stop)
        print(f"[SmartMotion] wait_nav_done: state changed to {state.value}", flush=True)
        return {"status": "stopped", "state": state.value}

    def handle_pause_nav(reason_str):
        nonlocal state
        if state != MotionState.NAVIGATING:
            return {"error": f"Cannot pause nav: state is {state.value}"}
        code, _ = slam_client.PauseNav()
        state = MotionState.NAV_PAUSED
        event_data = {"reason": reason_str}
        if reason_str == "obstacle":
            with obstacle_lock:
                event_data["obstacle_distance"] = round(obstacle_dist, 2)
        publish_event("nav_paused", event_data)
        return {"status": "paused"} if code == 0 else {"error": f"PauseNav failed, code={code}"}

    def handle_resume_nav():
        nonlocal state
        if state != MotionState.NAV_PAUSED:
            return {"error": f"Cannot resume nav: state is {state.value}"}
        code, _ = slam_client.ResumeNav()
        state = MotionState.NAVIGATING
        publish_event("nav_resumed", {})
        return {"status": "resumed"} if code == 0 else {"error": f"ResumeNav failed, code={code}"}

    def handle_get_state():
        result = {"state": state.value, "speed_zone": speed_zone.value}
        if current_cmd and state == MotionState.MOVING:
            result["motion"] = current_cmd.copy()
        if nav_cmd and state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            result["navigation"] = nav_cmd.copy()
        with obstacle_lock:
            result["obstacle_distance"] = round(obstacle_dist, 2) if obstacle_dist != float("inf") else None
            result["lateral_obstacle"] = lateral_obstacle
        return result

    # ── Safety checks (inline in main loop) ──
    def process_safety_checks():
        nonlocal state, speed_zone

        # 1. Communication timeout
        with safety_lock:
            lowstate_age = time.monotonic() - last_lowstate_time
            temp = max_motor_temp

        if lowstate_age > comm_timeout and state in (MotionState.MOVING, MotionState.NAVIGATING, MotionState.NAV_PAUSED):
            emergency_stop("comm_timeout", {"last_msg_age_ms": round(lowstate_age * 1000)})
            return

        # 2. Joint overheat
        if state == MotionState.MOVING:
            if temp > motor_temp_stop:
                emergency_stop("joint_overheat", {"max_temp": round(temp, 1)})
                return
            elif temp > motor_temp_decel:
                if speed_zone != SpeedZone.DECELERATED:
                    speed_zone = SpeedZone.DECELERATED
                    if current_cmd:
                        dvx, dvy, dvyaw = clamp(current_cmd["vx"], current_cmd["vy"], current_cmd["vyaw"], SpeedZone.DECELERATED)
                        loco_client.Move(dvx, dvy, dvyaw, True)
                        publish_event("joint_overheat", {
                            "max_temp": round(temp, 1),
                            "action": "decelerate",
                            "new_speed": {"vx": dvx, "vy": dvy, "vyaw": dvyaw},
                        })
                    return

        # 3. Obstacle detection (original logic)
        with obstacle_lock:
            dist = obstacle_dist
            lateral = lateral_obstacle

        if state == MotionState.MOVING:
            cmd = current_cmd  # snapshot to avoid race condition
            if dist <= stop_threshold:
                if speed_zone != SpeedZone.STOPPED:
                    speed_zone = SpeedZone.STOPPED
                    print(f"[SmartMotion:obstacle] STOP — dist={dist:.2f}m angle={math.degrees(obstacle_angle):.1f}° lateral={lateral}", flush=True)
                    do_stop("obstacle")
            elif dist <= decel_threshold or lateral:
                if speed_zone != SpeedZone.DECELERATED:
                    speed_zone = SpeedZone.DECELERATED
                    print(f"[SmartMotion:obstacle] DECEL — dist={dist:.2f}m angle={math.degrees(obstacle_angle):.1f}° lateral={lateral}", flush=True)
                    if cmd:
                        dvx, dvy, dvyaw = clamp(cmd["vx"], cmd["vy"], cmd["vyaw"], SpeedZone.DECELERATED)
                        loco_client.Move(dvx, dvy, dvyaw, True)
                        with obstacle_lock:
                            od = obstacle_dist
                            oa = obstacle_angle
                        publish_event("motion_decelerate", {
                            "obstacle_distance": round(od, 2),
                            "obstacle_angle_deg": round(math.degrees(oa), 1),
                            "original_speed": {"vx": cmd["vx"], "vy": cmd["vy"], "vyaw": cmd["vyaw"]},
                            "new_speed": {"vx": dvx, "vy": dvy, "vyaw": dvyaw},
                        })
            else:
                if speed_zone == SpeedZone.DECELERATED:
                    speed_zone = SpeedZone.NORMAL
                    if cmd:
                        cvx, cvy, cvyaw = clamp(cmd["vx"], cmd["vy"], cmd["vyaw"], SpeedZone.NORMAL)
                        loco_client.Move(cvx, cvy, cvyaw, True)
                        publish_event("motion_resume", {"speed": {"vx": cvx, "vy": cvy, "vyaw": cvyaw}})

        elif state == MotionState.NAVIGATING:
            # In mode=1 (stop-on-obstacle), SLAM service handles obstacle stopping.
            # No local PauseNav/ResumeNav needed — avoids conflicts with SLAM state.
            pass

        elif state == MotionState.NAV_PAUSED:
            # Obstacle cleared — resume navigation.
            # Only resume if we were paused by local obstacle detection.
            if dist > decel_threshold and not lateral:
                try:
                    slam_client.ResumeNav()
                except Exception:
                    pass
                state = MotionState.NAVIGATING
                speed_zone = SpeedZone.NORMAL
                publish_event("nav_resumed", {})
                print(f"[SmartMotion:nav_obstacle] RESUME NAV — "
                      f"dist={dist:.2f}m, obstacle cleared", flush=True)

    # ── Main loop ──
    print(f"[SmartMotion:pid={os.getpid()}] entering main loop")
    running = True
    last_obstacle_check = 0.0
    last_slam_info_log = 0.0

    while running:
        # Process commands (non-blocking)
        try:
            cmd = cmd_queue.get(timeout=0.05)
            method = cmd.get("method")
            result = None

            if method == "move":
                result = handle_move(cmd["vx"], cmd["vy"], cmd["vyaw"], cmd["duration"])
            elif method == "stop":
                result = do_stop(cmd.get("reason", "command"))
            elif method == "navigate_to":
                result = handle_navigate_to(cmd["x"], cmd["y"], cmd["yaw"],
                                            cmd.get("target_name", ""),
                                            speed=cmd.get("speed", 0.5),
                                            mode=cmd.get("mode", 1),
                                            stall_timeout=cmd.get("stall_timeout", 60))
            elif method == "pause_nav":
                result = handle_pause_nav(cmd.get("reason", "command"))
            elif method == "resume_nav":
                result = handle_resume_nav()
            elif method == "stop_nav":
                result = do_stop_nav()
            elif method == "wait_nav_done":
                result = handle_wait_nav_done(cmd.get("stall_timeout", 60))
            elif method == "get_state":
                result = handle_get_state()
            elif method == "start_mapping":
                code, resp = slam_client.StartMapping()
                result = {"code": code, "response": resp}
            elif method == "stop_mapping":
                pcd_path = cmd["pcd_path"]
                slam_client.SetTimeout(15.0)
                try:
                    code, resp = slam_client.StopMapping(pcd_path)
                finally:
                    slam_client.SetTimeout(10.0)
                result = {"code": code, "response": resp}
            elif method == "init_pose":
                code, resp = slam_client.InitPose(
                    cmd.get("x", 0.0), cmd.get("y", 0.0), cmd.get("z", 0.0),
                    cmd.get("q_x", 0.0), cmd.get("q_y", 0.0),
                    cmd.get("q_z", 0.0), cmd.get("q_w", 1.0),
                    cmd.get("address", "")
                )
                result = {"code": code, "response": resp}
            elif method == "shutdown":
                if state == MotionState.MOVING:
                    loco_client.StopMove()
                elif state in (MotionState.NAVIGATING, MotionState.NAV_PAUSED):
                    try:
                        slam_client.PauseNav()
                    except Exception:
                        pass
                running = False
                result = {"status": "shutdown"}

            if result is not None:
                result["_req_id"] = cmd.get("_req_id")
                result_queue.put(result)
        except queue.Empty:
            pass

        # Safety checks at 10Hz
        now = time.monotonic()
        if now - last_obstacle_check >= 0.1:
            last_obstacle_check = now
            process_safety_checks()

            # Repeat StopMove after emergency_stop to ensure controller receives it
            if stop_repeat_count > 0 and state == MotionState.IDLE:
                loco_client.StopMove()
                stop_repeat_count -= 1

        # Periodic health log: verify slam_info subscription is working
        if now - last_slam_info_log >= 10.0:
            last_slam_info_log = now
            print(f"[SmartMotion] health: slam_info msgs={slam_info_msg_count} "
                  f"pos_info={slam_info_pos_count} state={state.value}", flush=True)

        # Spin ROS2 (non-blocking)
        executor.spin_once(timeout_sec=0)

    # Cleanup
    if move_timer:
        move_timer.cancel()
    executor.shutdown()
    rclpy.shutdown()
    print(f"[SmartMotion:pid={os.getpid()}] shutdown complete")
