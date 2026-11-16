"""Kuavo 5-W cards: documented ROS 1 interfaces -> Agent Core domain 42.

Joint convergence uses real sensorsData positions; base/Cartesian reach-time
completion remains explicitly an estimate. Never use MPC targets as sensors.
"""
from __future__ import annotations

import json
import math
import os
import queue
import ssl
import threading
import time
import urllib.request
from uuid import uuid4

from std_msgs.msg import String
from kuavo_sensors import KuavoSensors, SENSOR_CARDS


REACH = {
    "base_reach_time": "/lb_cmd_pose_reach_time",
    "leg_reach_time": "/lb_leg_joint_reach_time",
    "torso_reach_time": "/lb_torso_pose_reach_time",
    "arm_reach_time": "/lb_arm_joint_reach_time",
    "arm_ee_reach_time": "/lb_arm_ee_reach_time",
}
MODES = {0: "NoControl", 1: "ArmOnly", 2: "BaseOnly", 3: "BaseArm", 4: "ArmEeOnly"}
CONTROL_TIMEOUT_S = 2.0  # One total budget for service acceptance AND state confirmation.
DISPATCH_LOCK_TIMEOUT_S = 0.25
ENABLE_LEASE_S = 3.0
ENABLE_PROBE_INTERVAL_S = 1.0
ENABLE_TOPIC = '/enable_control_state'


def _safe_log(value, limit=200):
    if not isinstance(value, str):
        return f'<{type(value).__name__}>'
    return value[:limit].encode('unicode_escape').decode('ascii')[:limit]


def _acp_complete(action_id: str, tool: str, status: str, result: dict) -> None:
    context = ssl.create_default_context()
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE  # Agent Core's local self-signed certificate (README_dev).
    body = json.dumps({"action_id": action_id, "status": status, "result": result,
                       "tool": tool, "ts": time.time()}, allow_nan=False).encode()
    url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
    for attempt in range(3):
        try:
            request = urllib.request.Request(f"{url}/api/acp/complete", data=body,
                headers={"Content-Type": "application/json"}, method="POST")
            with urllib.request.urlopen(request, timeout=3, context=context):
                return
        except Exception as exc:
            print(f"[ACP] {_safe_log(action_id)} attempt {attempt + 1} failed: {_safe_log(str(exc))}", flush=True)
            if attempt < 2:
                time.sleep(0.2)


class RobotState:
    """Internal feedback stays subscribed even when the state card is stopped."""
    PREFIX = "state"

    def __init__(self, ros1, node, namespace: str, sensors, motion_enabled=True):
        self.ros1 = ros1
        self.sensors = sensors
        self._topic = f"/{namespace}/state/control"
        self._pub = node.create_publisher(String, self._topic, 10)
        self._lock = threading.RLock()
        self._values = {}
        self._seen = {}
        self._wall = {}
        self._generation = -1
        self._running = False
        self.motion_blocked = False
        self.motion_enabled = motion_enabled
        self._enable_verified_at = None
        self._enable_check_error = 'waiting_for_enable_check'
        self._enable_publishers = []
        self._enable_probe_thread = None
        self._enable_probe_lock = threading.Lock()
        self._enable_probe_after = 0
        self._closed = threading.Event()
        ros1.subscribe("/mobile_manipulator/lb_mpc_control_mode", "std_msgs/Int8",
                       lambda msg: self._update("mpc_mode", msg.get("data")))
        ros1.subscribe("/enable_control_state", "std_msgs/Bool",
                       lambda msg: self._update("control_enabled", msg.get("data")))
        for key, topic in REACH.items():
            ros1.subscribe(topic, "std_msgs/Float32",
                           lambda msg, k=key: self._update(k, msg.get("data")))
        # Actual world-frame end-effector poses, NOT currentMpcTarget desired states.
        ros1.subscribe("/humanoid_wheel/eePoses", "std_msgs/Float64MultiArray",
                       lambda msg: self._update("ee_world_poses", msg.get("data")))
        self._timer = node.create_timer(0.2, self._publish)
        self._enable_timer = node.create_timer(0.2, self._enable_tick)

    def _sync_session(self) -> tuple[bool, int]:
        connected, generation = self.ros1.connection
        with self._lock:
            if not connected or generation != self._generation:
                self._values.clear()
                self._seen.clear()
                self._wall.clear()
                self._generation = generation
                self._enable_verified_at = None
                self._enable_check_error = 'connection_changed'
                self._enable_publishers = []
        return connected, generation

    def _update(self, key: str, value) -> None:
        connected, _ = self._sync_session()
        if not connected:
            return
        if key == "control_enabled":
            if not isinstance(value, bool):
                return
        elif key == "mpc_mode":
            if isinstance(value, bool) or not isinstance(value, int) or value not in MODES:
                return
        elif key == "ee_world_poses":
            if (not isinstance(value, list) or len(value) != 12 or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                        not math.isfinite(v) for v in value)):
                return
        elif (isinstance(value, bool) or not isinstance(value, (int, float))
              or not math.isfinite(value) or value < 0):
            return
        with self._lock:
            self._values[key] = value
            self._seen[key] = time.monotonic()
            self._wall[key] = time.time()

    def read(self, key: str):
        self._sync_session()
        with self._lock:
            return self._values.get(key), self._seen.get(key, 0)

    def snapshot(self) -> dict:
        connected, generation = self._sync_session()
        now = time.monotonic()
        with self._lock:
            fields = {key: {"value": value, "ts": self._wall[key],
                            "age_s": max(0, now - self._seen[key])}
                      for key, value in self._values.items()}
            mode_fresh = "mpc_mode" in self._seen and now - self._seen["mpc_mode"] <= 2
            return {"connected": connected, "connection_generation": generation,
                    "mode_feedback_fresh": mode_fresh,
                    "motion_blocked": self.motion_blocked, "fields": fields,
                    "motion_enabled": self.motion_enabled,
                    "enable_verification": {
                        'valid': self._enable_trusted(now), 'error': self._enable_check_error,
                        'age_s': None if self._enable_verified_at is None else max(0, now - self._enable_verified_at),
                        'publishers': list(self._enable_publishers), 'lease_s': ENABLE_LEASE_S,
                        'method': 'rosapi_publishers_and_latched_redelivery'},
                    "robot_discovery": self.sensors.health(),
                    "ee_pose_layout": "left[x,y,z,yaw,pitch,roll],right[x,y,z,yaw,pitch,roll]",
                    "ee_pose_units": "m,rad", "reach_time_units": "s",
                    "completion_basis": "per_action_see_acp_result"}

    def require_motion(self, allowed_modes=None) -> None:
        self.require_motion_enabled()
        self.sensors.require_profile()
        if not self._sync_session()[0]:
            raise ConnectionError("ROS 1 bridge disconnected; no commands were queued")
        if not self.ros1.motion_ready:
            raise RuntimeError("ROS 1 publisher connections are warming up; retry after 1s")
        enabled = self.confirmed_enable()
        mode, seen = self.read("mpc_mode")
        if self.motion_blocked or enabled is not True:
            raise RuntimeError("control is paused/unknown; confirm switch_mode.resume first")
        if time.monotonic() - seen > 2 or mode not in MODES or mode == 0:
            raise RuntimeError("MPC mode is missing, stale or NoControl")
        if allowed_modes and mode not in allowed_modes:
            raise RuntimeError(f"current MPC mode {mode} does not support this action; set_mode explicitly")

    def require_motion_enabled(self):
        if not self.motion_enabled:
            raise RuntimeError("motion is disabled by configuration; set motion_enabled: true and restart the driver")

    def _enable_trusted(self, now):
        seen = self._seen.get('control_enabled')
        return (self._enable_verified_at is not None and seen is not None
                and now - self._enable_verified_at <= ENABLE_LEASE_S
                and now - seen <= ENABLE_LEASE_S)

    def confirmed_enable(self):
        connected, _ = self._sync_session()
        with self._lock:
            if not connected or not self._enable_trusted(time.monotonic()):
                raise RuntimeError('control-enable verification is missing or stale; wait for read-only probe')
            return self._values.get('control_enabled')

    def _enable_tick(self):
        if self._closed.is_set() or time.monotonic() < self._enable_probe_after:
            return
        if not self._enable_probe_lock.acquire(blocking=False):
            return
        def probe():
            try:
                self.verify_enable()
            finally:
                self._enable_probe_after = time.monotonic() + ENABLE_PROBE_INTERVAL_S
                self._enable_probe_lock.release()
        self._enable_probe_thread = threading.Thread(target=probe, daemon=True, name='enable-check')
        self._enable_probe_thread.start()

    def verify_enable(self):
        """Read-only probe: a registered source AND a new latched delivery.

        Bool has no producer stamp and is event-driven, so age alone is not a
        producer heartbeat. Re-subscribe with the SAME id (no unsubscribe gap).
        Official rosbridge replays the publisher's latch on repeated subscribe.
        """
        connected, generation = self._sync_session()
        try:
            if not connected:
                raise ConnectionError('ROS bridge disconnected')
            response = self.ros1.call_service('/rosapi/publishers', 'rosapi/Publishers',
                                              {'topic': ENABLE_TOPIC}, 0.5)
            publishers = response.get('publishers')
            if (not isinstance(publishers, list) or len(publishers) != 1
                    or not isinstance(publishers[0], str) or not publishers[0]):
                raise RuntimeError('expected exactly one control-enable publisher')
            issued = time.monotonic()
            self.ros1.subscribe(ENABLE_TOPIC, 'std_msgs/Bool',
                               lambda msg: self._update('control_enabled', msg.get('data')))
            deadline = time.monotonic() + 0.5
            while not self._closed.is_set() and time.monotonic() < deadline:
                if self.ros1.connection != (True, generation):
                    raise ConnectionError('connection changed during enable verification')
                with self._lock:
                    seen = self._seen.get('control_enabled')
                    if (self._generation == generation and seen is not None and seen >= issued):
                        self._enable_verified_at = time.monotonic()
                        self._enable_publishers = publishers
                        self._enable_check_error = None
                        return
                self._closed.wait(0.01)
            raise TimeoutError('no control-enable latched delivery after read-only probe')
        except Exception as exc:
            with self._lock:
                if self._generation == generation:
                    self._enable_verified_at = None
                    self._enable_publishers = []
                    self._enable_check_error = _safe_log(str(exc))

    def close(self):
        self._closed.set()
        if self._enable_probe_thread is not None:
            self._enable_probe_thread.join(timeout=1.1)

    def _publish(self) -> None:
        if self._running:
            msg = String()
            msg.data = json.dumps(self.snapshot(), separators=(",", ":"), allow_nan=False)
            self._pub.publish(msg)

    def tool(self) -> dict:
        return {"name": self.PREFIX, "type": "sensor",
                "description": "Kuavo 5-W 连接、控制模式、软件暂停、动作到达时间估计及实测双臂世界系末端位姿；不将 MPC 目标当实测反馈。",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self._topic, "format": "data/json"}]}

    def get_tool(self):
        return self.tool()

    def start(self):
        self._running = True

    def stop(self):
        self._running = False

    def dispatch(self, action: str, _args=None) -> dict:
        if action == "start":
            self.start()
            state = "running"
        elif action == "stop":
            self.stop()
            state = "idle"
        elif action == "info":
            state = "running" if self._running else "idle"
        else:
            return {"state": "error", "error": f"unknown action: {action}"}
        result = {"state": state, "feedback": self.snapshot()}
        if action == "start" or (action == "info" and self._running):
            result["topic_out"] = self.tool()["topic_out"]
        return result


class ActionTracker:
    """The same lock serializes motion publication, cancellation and global pause."""
    def __init__(self):
        self.lock = threading.RLock()
        self._active = {}
        self._closed = False
        self._callbacks = queue.Queue()
        self._worker = threading.Thread(target=self._deliver, daemon=True, name="acp-callbacks")
        self._worker.start()

    def _deliver(self):
        while True:
            item = self._callbacks.get()
            try:
                if item is None:
                    return
                _acp_complete(*item)
            except Exception as exc:
                print(f"[ACP] callback worker failed: {_safe_log(str(exc))}", flush=True)
            finally:
                self._callbacks.task_done()

    def begin(self, channel: str, tool: str):
        with self.lock:
            if channel in self._active:
                raise RuntimeError(f"{channel} is busy; wait for ACP or stop it first")
            action_id, cancelled = f"kuavo_{tool}_{uuid4().hex}", threading.Event()
            self._active[channel] = (action_id, tool, cancelled)
            return action_id, cancelled

    def current(self, channel, action_id):
        active = self._active.get(channel)
        return active is not None and active[0] == action_id

    def finish(self, channel, action_id, tool, status, result):
        with self.lock:
            if self.current(channel, action_id):
                active = self._active.pop(channel)
                active[2].set()
                self._callbacks.put((action_id, tool, status, result))

    def cancel(self, channel, reason, status="cancelled"):
        with self.lock:
            active = self._active.pop(channel, None)
            if active:
                aid, tool, event = active
                event.set()
                self._callbacks.put((aid, tool, status, {"reason": reason, "channel": channel}))

    def cancel_all(self, reason, status="cancelled"):
        with self.lock:
            for channel, (aid, tool, event) in list(self._active.items()):
                event.set()
                self._callbacks.put((aid, tool, status, {"reason": reason, "channel": channel}))
            self._active.clear()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._callbacks.put(None)
        self._worker.join(timeout=3)
        if self._worker.is_alive():
            print("[ACP] shutdown callback drain timed out; remaining callbacks are best effort", flush=True)


def _number(value, name, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    if (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f"{name} is outside [{low}, {high}]")
    return value


def _vector(value, size, name):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} must contain exactly {size} numbers")
    return [_number(v, name) for v in value]


def _array_schema(size, description):
    return {"type": "array", "items": {"type": "number"}, "minItems": size,
            "maxItems": size, "description": description}


class _KuavoPlugin:
    ALLOWED_MODES = None

    def __init__(self, bundle):
        self.bundle = bundle
        self.ros1, self.state, self.tracker = bundle.ros1, bundle.state, bundle.tracker

    def schema(self, description, properties, actions, resource=None, async_actions=(), stop="stop_move", required=None):
        properties = {"action": {"type": "string", "enum": list(actions)}, **properties}
        schema = {"type": "object", "properties": properties, "required": ["action"],
                  "x-action-params": {k: {"params": v[0], "description": v[1],
                                               "required": (required or {}).get(k, [])}
                                      for k, v in actions.items()}}
        if resource:
            schema["x-resource"] = resource
        if async_actions:
            schema["x-completion"] = {"actions": list(async_actions), "timeout": 125}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": stop}}
        return {"name": self.PREFIX, "type": "actuator", "description": description,
                "inputSchema": schema}

    def get_tool(self):
        return self.tool()

    def start(self):
        pass  # ROS runtime belongs to the bundle, not an individual actuator card.

    def stop(self):
        pass  # Lifecycle marker; active actions remain owned by the bundle/ACP.

    def dispatch(self, action, args):
        try:
            if action == "start":
                self.start()
                return {"state": "ready"}
            if action == "stop":
                self.stop()
                return {"state": "idle"}
            if action == "info":
                return {"state": "ready" if self.ros1.connection[0] else "disconnected",
                        "motion_enabled": self.state.motion_enabled}
            # Control services wait outside the motion lock. Stops never take
            # the ordinary request's timed-lock/busy path.
            if action == 'stop_move':
                return self.bundle.pause('stopped_by_request')
            if self.PREFIX == 'switch_mode':
                return self._dispatch(action, args)
            if not self.tracker.lock.acquire(timeout=DISPATCH_LOCK_TIMEOUT_S):
                raise RuntimeError("control transition is busy; query state before retrying")
            try:
                return self._dispatch(action, args)
            finally:
                self.tracker.lock.release()
        except Exception as exc:
            print(f"[{self.PREFIX}] {_safe_log(action)}: {type(exc).__name__}: {_safe_log(str(exc))}", flush=True)
            return {"state": "error", "error": _safe_log(str(exc))}

    def _async_reach(self, key, publish, allowed_modes=None, joint_target=None):
        modes = allowed_modes or self.ALLOWED_MODES
        self.state.require_motion(modes)
        if joint_target is not None:
            self.bundle.sensors.joint_positions()  # Reject before publishing if feedback is unavailable.
        tracked = self.tracker.begin(self.CHANNEL, self.PREFIX)
        aid, cancelled = tracked
        generation = self.ros1.connection[1]
        issued = time.monotonic()
        try:
            publish()
        except Exception as exc:
            self.tracker.finish(self.CHANNEL, aid, self.PREFIX, "error", {"error": str(exc)})
            raise

        def worker():
            deadline, duration, seen_at = time.monotonic() + 3, None, 0
            converged_since = None
            while not cancelled.wait(0.02):
                try:
                    with self.tracker.lock:
                        if not self.tracker.current(self.CHANNEL, aid):
                            return
                        self.state.require_motion(modes)
                        if self.ros1.connection[1] != generation:
                            raise ConnectionError("connection changed during motion")
                        measured = self.bundle.sensors.joint_positions() if joint_target is not None else None
                        if duration is None:
                            value, seen = self.state.read(key)
                            if seen >= issued and value is not None:
                                if not 0 <= value <= 110:
                                    raise RuntimeError("vendor reach-time estimate exceeds the 110s driver limit")
                                duration, seen_at = float(value), seen
                            elif time.monotonic() >= deadline:
                                raise TimeoutError(f"no fresh {key} feedback within 3s")
                        if duration is not None and time.monotonic() >= seen_at + duration:
                            if joint_target is not None:
                                offset, target = joint_target
                                q, measured_at = measured
                                close = (measured_at >= issued and all(abs(a - b) <= 0.035
                                         for a, b in zip(q[offset:offset + len(target)], target)))
                                if close:
                                    converged_since = converged_since or measured_at
                                else:
                                    converged_since = None
                                if time.monotonic() >= seen_at + duration + 5:
                                    raise TimeoutError('measured joint target did not converge within 5s of reach-time estimate')
                                if converged_since is None or measured_at - converged_since < 0.2:
                                    continue
                            self.tracker.finish(self.CHANNEL, aid, self.PREFIX, "completed", {
                                "completion_basis": "measured_joint_convergence" if joint_target is not None else "vendor_reach_time_estimate",
                                "estimated_duration_s": duration,
                                "measured_convergence": joint_target is not None})
                            return
                except Exception as exc:
                    self.bundle.pause(_safe_log(str(exc)), status="error", expected=(self.CHANNEL, aid))
                    return
        threading.Thread(target=worker, daemon=True, name=f"acp-{self.PREFIX}").start()
        return {"state": "executing", "action_id": aid,
                "completion_basis": "measured_joint_convergence" if joint_target is not None else "vendor_reach_time_estimate",
                "measured_convergence": False}

    def _timed_velocity(self, duration, send, zero, stop_delay):
        self.state.require_motion(self.ALLOWED_MODES)
        aid, cancelled = self.tracker.begin(self.CHANNEL, self.PREFIX)
        generation = self.ros1.connection[1]
        try:
            send()  # Fail synchronously if the first command cannot be sent.
        except Exception as exc:
            self.tracker.finish(self.CHANNEL, aid, self.PREFIX, "error", {"error": str(exc)})
            raise
        deadline = time.monotonic() + duration

        def worker():
            try:
                while not cancelled.wait(0.05):
                    with self.tracker.lock:
                        if not self.tracker.current(self.CHANNEL, aid):
                            return
                        self.state.require_motion(self.ALLOWED_MODES)
                        if self.ros1.connection[1] != generation:
                            raise ConnectionError("connection changed during motion")
                        if time.monotonic() >= deadline:
                            zero()
                            break
                        send()
                else:
                    return
                # Vendor watchdog delay, not measured physical stop.
                until = time.monotonic() + stop_delay
                while not cancelled.wait(0.05):
                    with self.tracker.lock:
                        if not self.tracker.current(self.CHANNEL, aid):
                            return
                        self.state.require_motion(self.ALLOWED_MODES)
                        if self.ros1.connection[1] != generation:
                            raise ConnectionError("connection changed during stopping")
                        if time.monotonic() >= until:
                            self.tracker.finish(self.CHANNEL, aid, self.PREFIX, "completed", {
                                "duration_s": duration, "completion_basis": "command_duration_and_vendor_watchdog",
                                "stop_delay_s": stop_delay, "measured_convergence": False})
                            return
            except Exception as exc:
                self.bundle.pause(_safe_log(str(exc)), status="error", expected=(self.CHANNEL, aid))
        threading.Thread(target=worker, daemon=True, name=f"velocity-{self.PREFIX}").start()
        return {"state": "executing", "action_id": aid, "duration_s": duration,
                "completion_basis": "command_duration_and_vendor_watchdog"}


class LocoPlugin(_KuavoPlugin):
    PREFIX, CHANNEL, ALLOWED_MODES = "loco", "base", (2, 3)

    def tool(self):
        return self.schema("Kuavo 5-W 轮式底盘：有界速度、相对/世界位姿。stop_move 使用全身软件暂停防止位置目标续行。", {
            "vx": {"type": "number", "minimum": -0.3, "maximum": 0.3, "description": "本体前向速度 m/s，驱动保守上限"},
            "vy": {"type": "number", "minimum": -0.3, "maximum": 0.3, "description": "本体左向速度 m/s，驱动保守上限"},
            "vyaw": {"type": "number", "minimum": -0.5, "maximum": 0.5, "description": "逆时针角速度 rad/s，驱动保守上限"},
            "duration": {"type": "number", "minimum": 0.1, "maximum": 10},
            "x": {"type": "number", "description": "相对位移/世界 X 米"},
            "y": {"type": "number", "description": "相对位移/世界 Y 米"},
            "yaw": {"type": "number", "description": "相对/世界 yaw 弧度"},
        }, {"move": (["vx", "vy", "vyaw", "duration"], "本体速度运动，到时发零速度并等待文档的 1s 停止窗口"),
            "move_relative": (["x", "y", "yaw"], "相对当前位置的位姿目标"),
            "move_to": (["x", "y", "yaw"], "启动世界系的绝对位姿目标"),
            "stop_move": ([], "发零速并请求全身软件暂停；恢复须显式 resume")},
            "base", ("move", "move_relative", "move_to"),
            required={"move": ["duration"], "move_relative": ["x", "y", "yaw"], "move_to": ["x", "y", "yaw"]})

    def _dispatch(self, action, args):
        if action == "move":
            vx = _number(args.get("vx", 0), "vx", -0.3, 0.3)
            vy = _number(args.get("vy", 0), "vy", -0.3, 0.3)
            vyaw = _number(args.get("vyaw", 0), "vyaw", -0.5, 0.5)
            duration = _number(args.get("duration"), "duration", 0.1, 10)
            return self._timed_velocity(duration,
                lambda: self.bundle.twist("/cmd_vel", x=vx, y=vy, yaw=vyaw),
                lambda: self.bundle.twist("/cmd_vel"), 1.0)
        if action in ("move_relative", "move_to"):
            x, y, yaw = [_number(args.get(k), k) for k in ("x", "y", "yaw")]
            topic = "/cmd_pose" if action == "move_relative" else "/cmd_pose_world"
            return self._async_reach("base_reach_time",
                lambda: self.bundle.twist(topic, x=x, y=y, yaw=yaw))
        raise ValueError(f"unknown action: {action}")


class ArmPlugin(_KuavoPlugin):
    PREFIX, CHANNEL, ALLOWED_MODES = "arm", "arms", (1, 3)

    def tool(self):
        return self.schema("Kuavo 5-W 双臂关节/末端目标；关节单位度，位置米、四元数 xyzw。不自动开启 quick mode。", {
            "positions": _array_schema(14, "左臂7 + 右臂7关节，度"),
            "frame": {"type": "string", "enum": ["world", "local"], "description": "末端命令坐标系，必填"},
            "left_position": _array_schema(3, "左手 xyz 米"),
            "right_position": _array_schema(3, "右手 xyz 米"),
            "left_quaternion": _array_schema(4, "左手单位四元数 xyzw"),
            "right_quaternion": _array_schema(4, "右手单位四元数 xyzw"),
        }, {"move_joints": (["positions"], "设置双臂14关节角度"),
            "set_pose": (["frame", "left_position", "left_quaternion", "right_position", "right_quaternion"], "同时设置双手完整末端位姿"),
            "stop_move": ([], "请求全身软件暂停")}, ["arm_l", "arm_r"], ("move_joints", "set_pose"),
            required={"move_joints": ["positions"], "set_pose": ["frame", "left_position", "left_quaternion", "right_position", "right_quaternion"]})

    def _dispatch(self, action, args):
        if action == "move_joints":
            positions = _vector(args.get("positions"), 14, "positions")
            return self._async_reach("arm_reach_time", lambda: self.ros1.publish(
                "/kuavo_arm_traj", "sensor_msgs/JointState", {
                    "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
                    "name": [f"joint{i}" for i in range(1, 15)],
                    "position": positions, "velocity": [], "effort": []}),
                joint_target=(4, [math.radians(v) for v in positions]))
        if action == "set_pose":
            if args.get("frame") not in ("world", "local"):
                raise ValueError("frame must be world or local")
            poses = {}
            for side in ("left", "right"):
                pos = _vector(args.get(f"{side}_position"), 3, f"{side}_position")
                quat = _vector(args.get(f"{side}_quaternion"), 4, f"{side}_quaternion")
                norm = math.sqrt(sum(x * x for x in quat))
                if not math.isfinite(norm) or abs(norm - 1) > 0.01:
                    raise ValueError("quaternion must be normalized (norm within 0.01 of 1)")
                poses[f"{side}_pose"] = {"pos_xyz": pos, "quat_xyzw": [x / norm for x in quat],
                                        "elbow_pos_xyz": [0.0] * 3, "joint_angles": [0.0] * 7}
            return self._async_reach("arm_ee_reach_time", lambda: self.ros1.publish(
                "/mm/two_arm_hand_pose_cmd", "kuavo_msgs/twoArmHandPoseCmd", {
                    "hand_poses": {"header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""}, **poses},
                    "frame": 1 if args["frame"] == "world" else 2,
                    "use_custom_ik_param": False, "joint_angles_as_q0": False,
                    "ik_param": {}}), allowed_modes=(1, 3, 4))
        raise ValueError(f"unknown action: {action}")


class LowerBodyPlugin(_KuavoPlugin):
    CHANNEL = "lower_body"

    def __init__(self, bundle, card="leg"):
        super().__init__(bundle)
        self.PREFIX = card

    def tool(self):
        if self.PREFIX == "leg":
            return self.schema("Kuavo 5-W 下肢4关节目标，度；与 waist 共享控制通道。", {
                "positions": _array_schema(4, "四个下肢关节目标角度，度")}, {
                "move_joints": (["positions"], "设置四个下肢关节角度"),
                "stop_move": ([], "请求全身软件暂停")}, "lower_body", ("move_joints",),
                required={"move_joints": ["positions"]})
        return self.schema("Kuavo 5-W 躯干相对底盘位姿；仅支持 x/z/pitch/yaw，四个值均必填。", {
            "x": {"type": "number", "description": "相对底盘 X 米"},
            "z": {"type": "number", "description": "相对底盘 Z 米"},
            "pitch": {"type": "number", "description": "俯仰弧度"},
            "yaw": {"type": "number", "description": "偏航弧度"}}, {
            "set_pose": (["x", "z", "pitch", "yaw"], "设置躯干完整有效位姿"),
            "stop_move": ([], "请求全身软件暂停")}, "lower_body", ("set_pose",),
            required={"set_pose": ["x", "z", "pitch", "yaw"]})

    def _dispatch(self, action, args):
        if self.PREFIX == "leg" and action == "move_joints":
            positions = _vector(args.get("positions"), 4, "positions")
            return self._async_reach("leg_reach_time", lambda: self.ros1.publish(
                "/lb_leg_traj", "sensor_msgs/JointState", {
                    "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
                    "name": [f"joint{i}" for i in range(1, 5)],
                    "position": positions, "velocity": [], "effort": []}),
                joint_target=(0, [math.radians(v) for v in positions]))
        if self.PREFIX == "waist" and action == "set_pose":
            values = {k: _number(args.get(k), k) for k in ("x", "z", "pitch", "yaw")}
            return self._async_reach("torso_reach_time", lambda: self.bundle.twist(
                "/cmd_lb_torso_pose", **values))
        raise ValueError(f"unknown action: {action}")


class HeadPlugin(_KuavoPlugin):
    PREFIX, CHANNEL = "head", "head"

    def tool(self):
        return self.schema("Kuavo 5-W 头部有界速度运动；yaw/pitch 使用度/秒，到时归零。不提供缺乏完成反馈的一次性角度目标。", {
            "yaw": {"type": "number", "minimum": -30, "maximum": 30, "description": "yaw 度/秒，驱动保守上限"},
            "pitch": {"type": "number", "minimum": -20, "maximum": 20, "description": "pitch 度/秒，驱动保守上限"},
            "duration": {"type": "number", "minimum": 0.1, "maximum": 10}}, {
            "move": (["yaw", "pitch", "duration"], "有界头部角速度控制，到时归零并等待 0.3s 超时窗口"),
            "stop_move": ([], "请求全身软件暂停")}, "head", ("move",), required={"move": ["duration"]})

    def _dispatch(self, action, args):
        if action != "move":
            raise ValueError(f"unknown action: {action}")
        yaw = _number(args.get("yaw", 0), "yaw", -30, 30)
        pitch = _number(args.get("pitch", 0), "pitch", -20, 20)
        duration = _number(args.get("duration"), "duration", 0.1, 10)
        return self._timed_velocity(duration,
            lambda: self.ros1.publish("/cmd_head_vel", "kuavo_msgs/robotHeadMotionData", {"joint_data": [yaw, pitch]}),
            self.bundle.zero_head, 0.3)


class ControlPlugin(_KuavoPlugin):
    PREFIX = "switch_mode"

    def tool(self):
        # No x-resource: global exclusivity is intentional for whole-body transitions.
        return self.schema("Kuavo 5-W 软件暂停/恢复及 MPC 模式切换；以状态话题确认，不把服务受理当生效。", {
            "mode": {"type": "integer", "enum": list(MODES), "description": str(MODES)}}, {
            "pause": ([], "暂停全身，取消在途 ACP；不是硬件急停"),
            "resume": ([], "恢复控制，保持冻结姿态，须重新发运动目标"),
            "set_mode": (["mode"], "切换官方 MPC 模式；0 为被动 NoControl"),
            "get_current_mode": ([], "读取最近控制反馈")}, stop="pause",
            required={"set_mode": ["mode"]})

    def _dispatch(self, action, args):
        if action == "get_current_mode":
            return {"state": "ready", "feedback": self.state.snapshot()}
        if action == "pause":
            return self.bundle.pause("software_pause")
        if action == "resume":
            return self.bundle.resume()
        if action == "set_mode":
            mode = args.get("mode")
            if isinstance(mode, bool) or not isinstance(mode, int) or mode not in MODES:
                raise ValueError("mode must be an integer from 0 to 4")
            return self.bundle.set_mode(mode)
        raise ValueError(f"unknown action: {action}")


class KuavoBundle:
    def __init__(self, config, namespace, ros1, ros2_node):
        motion_enabled = config.get("motion_enabled", True)
        if type(motion_enabled) is not bool:
            raise ValueError("motion_enabled must be a YAML boolean")
        self.ros1, self.namespace = ros1, namespace
        for topic, message_type in (
            ("/cmd_vel", "geometry_msgs/Twist"),
            ("/cmd_pose", "geometry_msgs/Twist"),
            ("/cmd_pose_world", "geometry_msgs/Twist"),
            ("/cmd_lb_torso_pose", "geometry_msgs/Twist"),
            ("/kuavo_arm_traj", "sensor_msgs/JointState"),
            ("/lb_leg_traj", "sensor_msgs/JointState"),
            ("/cmd_head_vel", "kuavo_msgs/robotHeadMotionData"),
            ("/mm/two_arm_hand_pose_cmd", "kuavo_msgs/twoArmHandPoseCmd"),
        ):
            ros1.advertise(topic, message_type)
        self.sensors = KuavoSensors(config.get('sensors', {}), namespace, ros1, ros2_node)
        self.state = RobotState(ros1, ros2_node, namespace, self.sensors, motion_enabled)
        self.tracker = ActionTracker()
        self._closing = False
        self._transition = None
        self.plugins = [LocoPlugin(self), ArmPlugin(self), LowerBodyPlugin(self),
                        LowerBodyPlugin(self, "waist"), HeadPlugin(self), ControlPlugin(self)]
        self._enabled = {name for name, cfg in config.get("plugins", {}).items()
                         if cfg.get("enabled", True)} if "plugins" in config else {
                             "loco", "arm", "leg", "waist", "head", "switch_mode", "state", *SENSOR_CARDS}

    def twist(self, topic, x=0.0, y=0.0, z=0.0, pitch=0.0, yaw=0.0):
        self.ros1.publish(topic, "geometry_msgs/Twist", {
            "linear": {"x": x, "y": y, "z": z},
            "angular": {"x": 0.0, "y": pitch, "z": yaw}})

    def zero_head(self):
        self.ros1.publish("/cmd_head_vel", "kuavo_msgs/robotHeadMotionData", {"joint_data": [0.0, 0.0]})

    def _new_transition(self, action):
        # Caller holds tracker.lock. A stop may supersede any older transition.
        if self._transition is not None:
            self._transition['cancelled'].set()
        token = {'action': action, 'cancelled': threading.Event(),
                 'generation': self.ros1.connection[1]}
        self._transition = token
        return token

    def _check_transition(self, token):
        if self._transition is not token or token['cancelled'].is_set():
            raise InterruptedError('control request superseded by stop')
        if self.ros1.connection != (True, token['generation']):
            raise ConnectionError('connection changed while confirming control state')

    def _begin_control(self, action):
        if not self.tracker.lock.acquire(timeout=DISPATCH_LOCK_TIMEOUT_S):
            raise RuntimeError('control transition is busy; query state before retrying')
        try:
            self.state.require_motion_enabled()
            self.sensors.require_profile()
            if self._closing or self._transition is not None or self.tracker._active:
                raise RuntimeError('motion/control is busy; stop and await confirmation before retrying')
            enabled = self.state.confirmed_enable()
            if action == 'set_mode' and enabled is not True:
                raise RuntimeError('connect and resume control before changing mode')
            was_blocked = self.state.motion_blocked
            self.state.motion_blocked = True
            return self._new_transition(action), enabled, was_blocked
        finally:
            self.tracker.lock.release()

    def _end_control(self, token):
        with self.tracker.lock:
            if self._transition is token:
                self._transition = None

    def confirm_control(self, token, service, service_type, args, accepted_key, key, expected):
        # Serialize only validation + SEND, never the remote response wait.
        # Thus a stop packet cannot be followed by an older resume/mode SEND.
        with self.tracker.lock:
            self._check_transition(token)
            issued = time.monotonic()
            pending = self.ros1.start_service(service, service_type, args)
        remaining = max(0.0, issued + CONTROL_TIMEOUT_S - time.monotonic())
        response = self.ros1.wait_service(pending, remaining, token['cancelled'])
        if response.get(accepted_key) is not True:
            raise RuntimeError(_safe_log(response.get('message', 'control request rejected')))
        remaining = max(0.0, issued + CONTROL_TIMEOUT_S - time.monotonic())
        self.wait_state(key, expected, issued, timeout=remaining, generation=token['generation'], token=token)

    def wait_state(self, key, expected, issued, timeout=CONTROL_TIMEOUT_S, generation=None, token=None):
        if generation is None:
            generation = self.ros1.connection[1]
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if token is not None:
                with self.tracker.lock:
                    self._check_transition(token)
            connected, current = self.ros1.connection
            if not connected or generation != current:
                raise ConnectionError("connection changed while confirming control state")
            value, seen = self.state.read(key)
            if seen >= issued and value == expected:
                return
            time.sleep(min(0.02, max(0, deadline - time.monotonic())))
        raise TimeoutError(f"request accepted but fresh {key}={expected} not confirmed; read state before retry")

    def pause(self, reason, status="cancelled", expected=None):
        with self.tracker.lock:
            if expected is not None and not self.tracker.current(*expected):
                return {'state': 'idle'}  # An old worker cannot cancel a newer action.
            interrupted = self._transition is not None
            token = self._new_transition('pause')
            self.state.motion_blocked = True
            errors = []
            for stop in (lambda: self.twist("/cmd_vel"), self.zero_head):
                try:
                    stop()
                except Exception as exc:
                    errors.append(_safe_log(str(exc)))
            self.tracker.cancel_all(reason, status)
        try:
            try:
                enabled = self.state.confirmed_enable()
            except RuntimeError:
                enabled = None
            # Even a fresh pre-resume False cannot bypass a superseding pause.
            if enabled is False and not interrupted:
                with self.tracker.lock:
                    self._check_transition(token)
                    return {"state": "paused", "control_enabled": False,
                            "completion_basis": "enable_control_state", "scope": "whole_body"}
            self.confirm_control(token, "/enable_control", "std_srvs/SetBool", {"data": False},
                                 "success", "control_enabled", False)
            with self.tracker.lock:
                self._check_transition(token)
                return {"state": "paused", "control_enabled": False,
                        "completion_basis": "enable_control_state", "scope": "whole_body"}
        except Exception as exc:
            errors.append(_safe_log(str(exc)))
            error = _safe_log('; '.join(errors))
            print(f"[pause] physical stop unconfirmed: {error}", flush=True)
            return {"state": "error", "error": error,
                    "physical_stop_confirmed": False, "motion_blocked": True}
        finally:
            self._end_control(token)

    def resume(self):
        token, enabled, _ = self._begin_control('resume')
        try:
            if enabled is not True:
                self.confirm_control(token, "/enable_control", "std_srvs/SetBool", {"data": True},
                                     "success", "control_enabled", True)
            with self.tracker.lock:
                self._check_transition(token)
                self.state.motion_blocked = False
                return {"state": "ready", "control_enabled": True,
                        "completion_basis": "enable_control_state", "new_motion_command_required": True}
        finally:
            self._end_control(token)

    def set_mode(self, mode):
        token, _, was_blocked = self._begin_control('set_mode')
        try:
            self.confirm_control(token, '/mobile_manipulator_mpc_control',
                                 'kuavo_msgs/changeTorsoCtrlMode', {'control_mode': mode},
                                 'result', 'mpc_mode', mode)
            with self.tracker.lock:
                self._check_transition(token)
                self.state.motion_blocked = was_blocked
                return {'state': 'completed', 'mode': mode, 'mode_name': MODES[mode],
                        'completion_basis': 'mpc_mode_feedback'}
        finally:
            self._end_control(token)

    def get_all_tools(self):
        tools = [p.get_tool() for p in self.plugins if p.PREFIX in self._enabled]
        return (tools + [t for t in self.sensors.tools() if t['name'] in self._enabled]
                + ([self.state.get_tool()] if "state" in self._enabled else []))

    def dispatch(self, name, args):
        args = dict(args)
        action = args.pop("action", "start")
        if name not in self._enabled:
            return None
        if self._closing:
            return {"state": "error", "error": "driver is shutting down"}
        if name == "state":
            return self.state.dispatch(action, args)
        if name in SENSOR_CARDS:
            return self.sensors.dispatch(name, action)
        for plugin in self.plugins:
            if plugin.PREFIX == name:
                return plugin.dispatch(action, args)
        return None

    def close(self):
        with self.tracker.lock:
            if self._closing:
                return
            self._closing = True
            needs_pause = bool(self.tracker._active) or self._transition is not None
        if needs_pause:
            result = self.pause("driver_shutdown")
            if result.get("error"):
                print(f"[shutdown] {_safe_log(result.get('error'))}", flush=True)
        self.state.close()
        with self.tracker.lock:
            self.state._running = False
            self.sensors.close()
        self.tracker.close()
