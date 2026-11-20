"""Kuavo 5-W cards: documented ROS 1 interfaces -> Agent Core domain 42.

Joint/head/base completion uses real feedback; approximate completion reports
its measured residual. Cartesian reach-time remains an explicit estimate.
Never use MPC targets as sensors.
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
from collections import deque
from uuid import uuid4

from std_msgs.msg import String
from kuavo_sensors import KuavoSensors, SENSOR_CARDS
from peripherals import Peripherals, CARDS as PERIPHERAL_CARDS
from speaker import Speaker
from microphone import Microphones
from startup import StartupChecks
from controlled_spatial import ControlledSpatialPlugin
from base_servo import BaseServo
from base_limits import LINEAR_LIMIT, ANGULAR_LIMIT, SPEED_FRACTIONS


REACH = {
    "base_reach_time": "/lb_cmd_pose_reach_time",
    "leg_reach_time": "/lb_leg_joint_reach_time",
    "torso_reach_time": "/lb_torso_pose_reach_time",
    "arm_reach_time": "/lb_arm_joint_reach_time",
    "arm_ee_reach_time": "/lb_arm_ee_reach_time",
}
MODES = {0: "NoControl", 1: "ArmOnly", 2: "BaseOnly", 3: "BaseArm", 4: "ArmEeOnly"}
CARD_CONTROL_MODE = 3  # All cards share BaseArm; only base velocity ownership changes.
CONTROL_TIMEOUT_S = 2.0  # One total budget for service acceptance AND state confirmation.
DISPATCH_LOCK_TIMEOUT_S = 0.25
ENABLE_LEASE_S = 3.0
ENABLE_PROBE_INTERVAL_S = 1.0
ENABLE_TOPIC = '/enable_control_state'
# Kuavo v63 kuavo.json has narrower yaw limits than the generic SDK (80 deg).
# Keep the 5-W card inside this conservative envelope, including held axes.
HEAD_LIMITS_DEG = (('yaw', -30, 30), ('pitch', -25, 25))


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

    def __init__(self, ros1, node, namespace: str, sensors, motion_enabled=True, boot_required=False):
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
        self.remote_resume_unresolved = False
        self.remote_mode_unresolved = False
        self.motion_send_uncertain = False
        self.motion_enabled = motion_enabled
        self._enable_verified_at = None
        self._enable_check_error = 'waiting_for_enable_check'
        self._enable_publishers = []
        self._enable_probe_thread = None
        self._enable_probe_lock = threading.Lock()
        self._enable_probe_after = 0
        self._closed = threading.Event()
        self._controller_boot = None
        self._controller_boot_at = None
        self._boot_required = boot_required
        ros1.subscribe("/mobile_manipulator/lb_mpc_control_mode", "std_msgs/Int8",
                       lambda msg: self._update("mpc_mode", msg.get("data")))
        ros1.subscribe("/enable_control_state", "std_msgs/Bool",
                       lambda msg: self._update("control_enabled", msg.get("data")))
        ros1.subscribe('/enable_vel_control_state', 'std_msgs/Bool',
                       lambda msg: self._update('upper_velocity_enabled', msg.get('data')))
        ros1.subscribe('/twist_mux_new/out/nav_source_used', 'std_msgs/String',
                       lambda msg: self._update('base_velocity_source', msg.get('data')))
        for key, topic in REACH.items():
            ros1.subscribe(topic, "std_msgs/Float32",
                           lambda msg, k=key: self._update(k, msg.get("data")))
        # Actual world-frame end-effector poses, NOT currentMpcTarget desired states.
        ros1.subscribe("/humanoid_wheel/eePoses", "std_msgs/Float64MultiArray",
                       lambda msg: self._update("ee_world_poses", msg.get("data")))
        ros1.subscribe('/kuavo5w/controller_boot_state', 'std_msgs/String', self._boot_update)
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
                self._controller_boot = None
                self._controller_boot_at = None
        return connected, generation

    def _boot_update(self, message):
        self._sync_session()
        raw = message.get('data')
        if not isinstance(raw, str) or len(raw) > 2048:
            return
        try:
            value = json.loads(raw)
        except (ValueError, TypeError):
            return
        if (not isinstance(value, dict) or value.get('state') not in
                ('waiting_hardware', 'initializing', 'pausing', 'complete', 'failed')
                or type(value.get('boot_pause_confirmed')) is not bool):
            return
        with self._lock:
            self._boot_required = True
            self._controller_boot = {
                'state': value['state'], 'boot_pause_confirmed': value['boot_pause_confirmed'],
                'initialization_requested': value.get('initialization_requested') is True,
                'initialization_accepted': value.get('initialization_accepted') is True,
                'initial_control_mode': 3 if type(value.get('initial_control_mode')) is int
                    and value['initial_control_mode'] == 3 else None,
                'launch_ownership': value.get('launch_ownership') if value.get('launch_ownership')
                    in ('external', 'service') else None,
                'error': _safe_log(value['error'], 100) if isinstance(value.get('error'), str) else None,
                'meaning': 'boot_result_not_current_control_state'}
            self._controller_boot_at = time.monotonic()

    def require_boot_complete(self):
        # Optional for previously commissioned manual deployments. Once this
        # session observes a boot owner, its gate may not be bypassed by silence.
        with self._lock:
            boot = self._controller_boot
            if self._boot_required and (boot is None or boot['state'] != 'complete'
                    or not boot['boot_pause_confirmed'] or time.monotonic() - self._controller_boot_at > 3):
                raise RuntimeError('controller boot initialization/pause is not confirmed; inspect controller_boot')

    def _update(self, key: str, value) -> None:
        connected, _ = self._sync_session()
        if not connected:
            return
        if key in ('control_enabled', 'upper_velocity_enabled'):
            if not isinstance(value, bool):
                return
        elif key == 'base_velocity_source':
            if not isinstance(value, str) or len(value) > 80 or not value.isascii() or any(ord(c) < 32 for c in value):
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
                    "remote_resume_unresolved": self.remote_resume_unresolved,
                    "remote_mode_unresolved": self.remote_mode_unresolved,
                    "motion_send_uncertain": self.motion_send_uncertain,
                    'controller_boot': None if self._controller_boot is None else {
                        **self._controller_boot, 'age_s': max(0, now - self._controller_boot_at)},
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
            raise RuntimeError("control is paused/unknown; action preparation or a fresh control check is required")
        if time.monotonic() - seen > 2 or mode not in MODES or mode == 0:
            raise RuntimeError("MPC mode is missing, stale or NoControl")
        if allowed_modes and mode not in allowed_modes:
            raise RuntimeError(f"current MPC mode {mode} does not support this action; control preparation must confirm a compatible mode")

    def require_motion_enabled(self):
        self.require_boot_complete()
        if not self.motion_enabled:
            raise RuntimeError("motion is disabled by configuration; set motion_enabled: true and restart the driver")

    def _enable_trusted(self, now):
        seen = self._seen.get('control_enabled')
        return (self._enable_verified_at is not None and seen is not None
                and now - self._enable_verified_at <= ENABLE_LEASE_S
                and now - seen <= ENABLE_LEASE_S)

    def confirmed_enable(self):
        return self.confirmed_enable_sample()[0]

    def confirmed_enable_sample(self):
        connected, _ = self._sync_session()
        with self._lock:
            if not connected or not self._enable_trusted(time.monotonic()):
                raise RuntimeError('control-enable verification is missing or stale; wait for read-only probe')
            return self._values.get('control_enabled'), self._seen['control_enabled']

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
        self._no_completion = set()
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

    def begin(self, channel: str, tool: str, notify=True):
        with self.lock:
            if channel in self._active:
                raise RuntimeError(f"{channel} is busy; wait for ACP or stop it first")
            action_id, cancelled = f"kuavo_{tool}_{uuid4().hex}", threading.Event()
            self._active[channel] = (action_id, tool, cancelled)
            if not notify:
                self._no_completion.add(action_id)
            return action_id, cancelled

    def current(self, channel, action_id):
        active = self._active.get(channel)
        return active is not None and active[0] == action_id

    def finish(self, channel, action_id, tool, status, result):
        with self.lock:
            if self.current(channel, action_id):
                active = self._active.pop(channel)
                active[2].set()
                if action_id in self._no_completion:
                    self._no_completion.discard(action_id)
                else:
                    self._callbacks.put((action_id, tool, status, result))

    def cancel(self, channel, reason, status="cancelled"):
        with self.lock:
            active = self._active.pop(channel, None)
            if active:
                aid, tool, event = active
                event.set()
                if aid in self._no_completion:
                    self._no_completion.discard(aid)
                else:
                    self._callbacks.put((aid, tool, status, {"reason": reason, "channel": channel}))

    def cancel_all(self, reason, status="cancelled"):
        with self.lock:
            for channel, (aid, tool, event) in list(self._active.items()):
                event.set()
                if aid in self._no_completion:
                    self._no_completion.discard(aid)
                else:
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


def _angle_supplied(arguments, field):
    value = arguments.get(field)
    return value is not None and not (isinstance(value, str) and not value.strip())


def _vector(value, size, name):
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} must contain exactly {size} numbers")
    return [_number(v, name) for v in value]


def _array_schema(size, description):
    return {"type": "array", "items": {"type": "number"}, "minItems": size,
            "maxItems": size, "description": description}


class _UncertainMotionSend(RuntimeError):
    def __init__(self, channel, action_id, reason):
        super().__init__(_safe_log(reason))
        self.channel, self.action_id = channel, action_id


class _KuavoPlugin:
    ALLOWED_MODES = (CARD_CONTROL_MODE,)

    def __init__(self, bundle):
        self.bundle = bundle
        self.ros1, self.state, self.tracker = bundle.ros1, bundle.state, bundle.tracker
        self._request_context = threading.local()

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
        self._request_context.stop_epoch = self.bundle._stop_epoch
        try:
            if action == "start":
                result = self.start()
                return result if isinstance(result, dict) else {"state": "ready"}
            if action == "stop":
                self.stop()
                return {"state": "idle"}
            if action == "info":
                startup = self.bundle.spatial.startup_status() if self.PREFIX == "base_drive" else {}
                return {"state": startup.get("state") if startup.get("state") in ("loading", "error") else "ready" if self.ros1.connection[0] else "disconnected",
                        "motion_enabled": self.state.motion_enabled,
                        **({"motion": self._pose_progress} if self.PREFIX == "base_drive" else {}),
                        "startup": self.bundle.spatial.startup_status() if self.PREFIX == "base_drive" else None,
                        "control_ownership": self.bundle.control_ownership(),
                        "feedback": {**self.state.snapshot(),
                                     "host_runtime": dict(self.bundle.startup_checks.status)}}
            # Control services wait outside the motion lock. Stops never take
            # the ordinary request's timed-lock/busy path.
            if action == 'stop_move':
                return self.bundle.pause('stopped_by_request')
            # Argument validation is local. Motion helpers reserve preparation,
            # wait for services outside the lock, then atomically recheck/send.
            return self._dispatch(action, args)
        except _UncertainMotionSend as exc:
            # start_motion has released the publication lock. Never
            # hold it while waiting for a pause service response.
            stopped = self.bundle.pause(str(exc), status='error', expected=(exc.channel, exc.action_id))
            return {'state': 'error', 'error': str(exc), 'action_id': exc.action_id,
                    'command_delivery': 'uncertain', 'physical_stop_confirmed': False,
                    'motion_blocked': True, 'pause': stopped}
        except Exception as exc:
            print(f"[{self.PREFIX}] {_safe_log(action)}: {type(exc).__name__}: {_safe_log(str(exc))}", flush=True)
            return {"state": "error", "error": _safe_log(str(exc))}

    def _joint_set(self, updates, card=None):
        """Named absolute angles; preserve unspecified joints at send time."""
        card = card or self.PREFIX
        offset, size, topic = (4, 14, '/kuavo_arm_traj') if card == 'arm' else (0, 4, '/lb_leg_traj')
        target = [0.] * size
        def validate():
            candidate, measured = self.bundle.sensors.resolve_joint_target(card, updates)
            target[:] = candidate
            return measured
        return self._async_reach('arm_reach_time' if card == 'arm' else 'leg_reach_time',
            lambda: self.ros1.publish(topic, 'sensor_msgs/JointState', {
                'header': {'seq': 0, 'stamp': {'secs': 0, 'nsecs': 0}, 'frame_id': ''},
                'name': [f'joint{i}' for i in range(1, size + 1)],
                'position': [math.degrees(v) for v in target], 'velocity': [], 'effort': []}),
            joint_target=(offset, target), joint_card=card, preflight=validate)

    def _async_reach(self, key, publish, allowed_modes=None, joint_target=None, joint_card=None, preflight=None):
        modes = allowed_modes or self.ALLOWED_MODES
        joint_card = joint_card or self.PREFIX
        validate = preflight or (None if joint_target is None else
            lambda: self.bundle.sensors.validate_joint_target(joint_card, joint_target[1]))
        aid, cancelled, generation, issued, preparation = self.bundle.start_motion(
            self.CHANNEL, self.PREFIX, modes, publish, validate,
            request_epoch=self._request_context.stop_epoch)

        def worker():
            # Joint trajectories do not consistently publish reach-time on
            # vendor versions (notably /kuavo_arm_traj on v63). Their
            # completion evidence is measured convergence, not an estimate.
            deadline = issued + (115 if joint_target is not None else 3)
            duration, seen_at = None, 0
            converged_since_ns = converged_since_at = None
            last_capture_ns = None
            while not cancelled.wait(0.02):
                try:
                    with self.tracker.lock:
                        if not self.tracker.current(self.CHANNEL, aid):
                            return
                        self.state.require_motion(modes)
                        if self.ros1.connection[1] != generation:
                            raise ConnectionError("connection changed during motion")
                        measured = (self.bundle.sensors.validate_joint_target(joint_card, joint_target[1])
                                    if joint_target is not None else None)
                        if joint_target is not None and time.monotonic() >= deadline:
                            raise TimeoutError('measured joint target did not converge within 115s')
                        if joint_target is None and duration is None:
                            value, seen = self.state.read(key)
                            if seen >= issued and value is not None:
                                if not 0 <= value <= 110:
                                    raise RuntimeError("vendor reach-time estimate exceeds the 110s driver limit")
                                duration, seen_at = float(value), seen
                            elif time.monotonic() >= deadline:
                                raise TimeoutError(f"no fresh {key} feedback within 3s")
                        if joint_target is not None or (duration is not None and time.monotonic() >= seen_at + duration):
                            if joint_target is not None:
                                offset, target = joint_target
                                q, measured_at, capture_ns = measured
                                # Polling a cached sample is not a new observation.
                                # Keep freshness/deadline checks active while waiting.
                                if measured_at < issued or (last_capture_ns is not None
                                                            and capture_ns <= last_capture_ns):
                                    continue
                                last_capture_ns = capture_ns
                                close = all(abs(a - b) <= 0.035
                                            for a, b in zip(q[offset:offset + len(target)], target))
                                if close:
                                    if converged_since_ns is None:
                                        converged_since_ns = capture_ns
                                        converged_since_at = measured_at
                                else:
                                    converged_since_ns = converged_since_at = None
                                if (converged_since_ns is None or capture_ns - converged_since_ns < 200_000_000
                                        or measured_at - converged_since_at < .2):
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
                'control_preparation': preparation,
                "completion_basis": "measured_joint_convergence" if joint_target is not None else "vendor_reach_time_estimate",
                "measured_convergence": False}

    def _timed_velocity(self, duration, send, zero, stop_delay):
        aid, cancelled, generation, _, preparation = self.bundle.start_motion(
            self.CHANNEL, self.PREFIX, self.ALLOWED_MODES, send,
            request_epoch=self._request_context.stop_epoch, notify=duration != -1)
        deadline = float('inf') if duration == -1 else time.monotonic() + duration

        def worker():
            try:
                while not cancelled.wait(0.05):
                    with self.tracker.lock:
                        if not self.tracker.current(self.CHANNEL, aid):
                            return
                        self.bundle.require_base_control(self.PREFIX)
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
                        self.bundle.require_base_control(self.PREFIX)
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
        return {"state": "running" if duration == -1 else "executing",
                **({"continuous": True} if duration == -1 else {"action_id": aid}), "duration_s": duration,
                **({"expected_duration_s": duration + stop_delay} if duration != -1 else {}),
                'control_preparation': preparation,
                "completion_basis": "continuous_until_stop" if duration == -1 else "command_duration_and_vendor_watchdog"}


class _StableFeedback:
    """A bounded window of distinct measurements, never repeated cache reads."""
    def __init__(self, duration, linear_count=0, linear_epsilon=.002,
                 angular_epsilon=.2, angular_period=None):
        self.duration = duration
        self.linear_count, self.linear_epsilon = linear_count, linear_epsilon
        self.angular_epsilon, self.angular_period = angular_epsilon, angular_period
        self.samples = deque(maxlen=256)
        self.last_capture = self.last_received = None

    def reset(self):
        self.samples.clear()

    def update(self, values, captured, received):
        if self.last_capture is not None and captured <= self.last_capture:
            return False
        if self.last_received is not None and received-self.last_received > .35:
            self.reset()
        self.last_capture, self.last_received = captured, received
        self.samples.append((captured, received, tuple(values)))
        while (len(self.samples) > 2 and captured-self.samples[1][0] >= self.duration
               and received-self.samples[1][1] >= self.duration):
            self.samples.popleft()
        if (len(self.samples) < 3 or captured-self.samples[0][0] < self.duration
                or received-self.samples[0][1] < self.duration):
            return False
        spans = []
        for i, anchor in enumerate(self.samples[0][2]):
            values = [sample[2][i]-anchor for sample in self.samples]
            if i >= self.linear_count and self.angular_period is not None:
                period = self.angular_period
                values = [(v+period/2) % period-period/2 for v in values]
            spans.append(max(values)-min(values))
        return (math.hypot(*spans[:self.linear_count]) <= self.linear_epsilon
                and all(v <= self.angular_epsilon for v in spans[self.linear_count:]))


class _PoseServo:
    """Bounded integral correction for small-command steady-state error.

    Integrate only distinct observations near the target. Clear bias inside
    each axis group's tolerance; never raise the selected speed envelope.
    """
    POSITION_TOLERANCE = .02
    YAW_TOLERANCE = math.radians(1.)

    def __init__(self, linear, angular):
        self.linear, self.angular = linear, angular
        self.bias = [0., 0., 0.]
        self.capture = None

    def command(self, dx, dy, da, yaw, captured):
        dt = max(0., min(.1, captured-self.capture)) if self.capture is not None else 0.
        if self.capture is None or captured > self.capture:
            self.capture = captured
        distance = math.hypot(dx, dy)
        if distance <= self.POSITION_TOLERANCE or distance > .15:
            self.bias[:2] = [0., 0.]
        elif dt:
            if dx*self.bias[0]+dy*self.bias[1] < 0:
                self.bias[:2] = [0., 0.]
            candidate = [self.bias[0]+.3*dx*dt, self.bias[1]+.3*dy*dt]
            magnitude = math.hypot(*candidate)
            scale = min(1., min(.08, self.linear*.5)/magnitude) if magnitude else 1.
            candidate = [v*scale for v in candidate]
            if math.hypot(.8*dx+candidate[0], .8*dy+candidate[1]) <= self.linear:
                self.bias[:2] = candidate
        if abs(da) <= self.YAW_TOLERANCE or abs(da) > .35:
            self.bias[2] = 0.
        elif dt:
            if da*self.bias[2] < 0:
                self.bias[2] = 0.
            cap = min(.12, self.angular*.5)
            candidate = max(-cap, min(cap, self.bias[2]+.3*da*dt))
            if abs(.8*da+candidate) <= self.angular:
                self.bias[2] = candidate
        wx, wy = (.8*dx+self.bias[0], .8*dy+self.bias[1]) if distance > self.POSITION_TOLERANCE else (0., 0.)
        magnitude = math.hypot(wx, wy)
        scale = min(1., self.linear/magnitude) if magnitude else 1.
        wx, wy = wx*scale, wy*scale
        wz = max(-self.angular, min(self.angular, .8*da+self.bias[2])) if abs(da) > self.YAW_TOLERANCE else 0.
        return math.cos(yaw)*wx+math.sin(yaw)*wy, -math.sin(yaw)*wx+math.cos(yaw)*wy, wz


class BaseDrivePlugin(_KuavoPlugin):
    PREFIX, CHANNEL, ALLOWED_MODES = "base_drive", "base", (CARD_CONTROL_MODE,)

    def __init__(self, bundle):
        super().__init__(bundle)
        self._pose_progress = None

    def start(self):
        return self.bundle.spatial.start_chassis()

    def tool(self):
        tool = self.schema(f"平移≤{LINEAR_LIMIT} m/s，旋转≤{math.degrees(ANGULAR_LIMIT):.4f} deg/s；Kuavo 5-W轮式底盘，无避障，避障请用controlled_spatial。", {
            "vx": {"type": "number", "minimum": -LINEAR_LIMIT, "maximum": LINEAR_LIMIT, "description": "≤0.8 m/s（平移合速度）；本体前向速度"},
            "vy": {"type": "number", "minimum": -LINEAR_LIMIT, "maximum": LINEAR_LIMIT, "description": "≤0.8 m/s（平移合速度）；本体左向速度"},
            "vyaw": {"type": "number", "minimum": -math.degrees(ANGULAR_LIMIT), "maximum": math.degrees(ANGULAR_LIMIT), "description": "±57.2958 deg/s；逆时针角速度"},
            "duration": {"type": "number", "description": "0.1–10 s；-1持续至stop_move；运动时长", "minimum": -1, "maximum": 10},
            "speed": {"type": "string", "enum": ["low", "medium", "high"], "default": "low", "description": "low=0.24 m/s、17.19°/s；medium=0.48、34.38；high=0.8、57.30；厂商命令上限30%/60%/100%"},
            "x": {"type": "number", "description": "合位移≤5 m；相对机身X位移，米"},
            "y": {"type": "number", "description": "合位移≤5 m；相对机身Y位移，米"},
            "yaw": {"type": "number", "description": "相对偏航转角，度"},
        }, {"move": (["vx", "vy", "vyaw", "duration"], "0.1–10 s或-1持续至stop_move；本体速度运动，到时发零速度并等待1s停止窗口"),
            "move_relative": (["x", "y", "yaw", "speed"], "≤5 m；到位2cm/1°；稳定残差≤10cm/5°可近似完成并报告误差；相对机身位移/转角，米/度，无避障"),
            "stop_move": ([], "发零速并请求全身软件暂停；下一次新动作自动准备控制")},
            "base", ("move", "move_relative"),
            required={"move": ["duration"], "move_relative": ["x", "y", "yaw"]})
        return tool

    def _dispatch(self, action, args):
        if action == "move":
            vx = _number(args.get("vx", 0), "vx", -LINEAR_LIMIT, LINEAR_LIMIT)
            vy = _number(args.get("vy", 0), "vy", -LINEAR_LIMIT, LINEAR_LIMIT)
            vyaw = math.radians(_number(args.get("vyaw", 0), "vyaw", -math.degrees(ANGULAR_LIMIT), math.degrees(ANGULAR_LIMIT)))
            if math.hypot(vx, vy) > LINEAR_LIMIT + 1e-9:
                raise ValueError("planar velocity magnitude exceeds 0.8 m/s")
            duration = _number(args.get("duration"), "duration")
            if duration != -1 and not .1 <= duration <= 10:
                raise ValueError("duration must be -1 or between 0.1 and 10 seconds")
            return self._timed_velocity(duration,
                lambda: self.bundle.twist("/cmd_vel", x=vx, y=vy, yaw=vyaw),
                lambda: self.bundle.twist("/cmd_vel"), 1.0)
        if action == "move_relative":
            x, y, yaw = [_number(args.get(k), k) for k in ("x", "y", "yaw")]
            yaw = math.radians(yaw)
            speed = args.get('speed', 'low')
            if speed not in ('low', 'medium', 'high'):
                raise ValueError('speed must be low, medium or high')
            return self._pose_move(x, y, yaw, speed)
        raise ValueError(f"unknown action: {action}")

    def _pose_move(self, x, y, yaw, speed):
        fraction = SPEED_FRACTIONS[speed]
        linear, angular = LINEAR_LIMIT*fraction, ANGULAR_LIMIT*fraction
        target, frame = [], []
        def validate():
            pose, _, source = self.bundle.sensors.odom_pose()
            px, py, pa = pose
            target[:] = [px+math.cos(pa)*x-math.sin(pa)*y,
                         py+math.sin(pa)*x+math.cos(pa)*y, pa+yaw]
            frame[:] = [source]
            if math.hypot(target[0]-px, target[1]-py) > 5:
                raise ValueError('pose displacement exceeds 5m; use controlled_spatial navigation')
        aid, cancelled, generation, issued, preparation = self.bundle.start_motion(
            self.CHANNEL, self.PREFIX, self.ALLOWED_MODES,
            lambda: self.bundle.twist('/cmd_vel'), validate,
            request_epoch=self._request_context.stop_epoch)
        command_wall = time.time()
        self._pose_progress = {'state': 'executing', 'action_id': aid, 'frame_id': frame[0],
                               'target': {'x_m': target[0], 'y_m': target[1], 'yaw_deg': math.degrees(target[2])}}
        def worker():
            settled = last_capture = None
            servo = _PoseServo(linear, angular)
            best_score, last_progress = float('inf'), issued
            plateau = _StableFeedback(1., 2, angular_epsilon=math.radians(.2), angular_period=2*math.pi)
            stopped = _StableFeedback(1.1, 2, angular_epsilon=math.radians(.2), angular_period=2*math.pi)
            approximate_stop = None
            def complete(approximate):
                progress = self._pose_progress
                error = progress['error']
                result = {'completion_basis': 'measured_odometry_stable_residual' if approximate else 'measured_odometry_convergence',
                          'measured_convergence': not approximate, 'approximate': approximate,
                          'frame_id': progress['frame_id'], 'target': progress['target'], 'measured': progress['measured'], 'error': error,
                          'message': f"Completed{' approximately' if approximate else ''}; remaining distance {error['position_m']:.4f} m, yaw error {error['yaw_deg']:+.3f} deg."}
                self._pose_progress = {**progress, 'state': 'completed', 'completion': result}
                self.tracker.finish(self.CHANNEL, aid, self.PREFIX, 'completed', result)
            try:
                while not cancelled.wait(.05):
                    with self.tracker.lock:
                        if not self.tracker.current(self.CHANNEL, aid):
                            return
                        self.bundle.require_base_control(self.PREFIX)
                        if self.ros1.connection != (True, generation):
                            raise ConnectionError('connection changed during pose motion')
                        if time.monotonic()-issued > 110:
                            error = self._pose_progress.get('error', {})
                            raise TimeoutError(f"pose motion exceeded 110s; position_error_m={error.get('position_m')}; yaw_error_deg={error.get('yaw_deg')}")
                        pose, captured, source = self.bundle.sensors.odom_pose()
                        if source != frame[0]:
                            raise RuntimeError('odometry frame changed during pose motion')
                        dx, dy = target[0]-pose[0], target[1]-pose[1]
                        da = math.atan2(math.sin(target[2]-pose[2]), math.cos(target[2]-pose[2]))
                        distance = math.hypot(dx, dy)
                        self._pose_progress = {**self._pose_progress,
                            'measured': {'x_m': pose[0], 'y_m': pose[1], 'yaw_deg': math.degrees(pose[2])},
                            'error': {'position_m': round(distance, 6), 'x_m': round(dx, 6), 'y_m': round(dy, 6), 'yaw_deg': round(math.degrees(da), 6)},
                            'elapsed_s': round(time.monotonic()-issued, 3), 'capture_time_s': captured}
                        close = distance <= servo.POSITION_TOLERANCE and abs(da) <= servo.YAW_TOLERANCE
                        near = distance <= .10 and abs(da) <= math.radians(5.)
                        now = time.monotonic()
                        if approximate_stop is not None:
                            self.bundle.twist('/cmd_vel')
                            self._pose_progress = {**self._pose_progress, 'command': {'vx_m_s': 0., 'vy_m_s': 0., 'yaw_rad_s': 0.}}
                            if not near:
                                raise RuntimeError('base moved outside approximate arrival bounds while stopping')
                            if stopped.update(pose, captured, now):
                                complete(True)
                                return
                            if now-approximate_stop > 4.:
                                raise TimeoutError('approximate arrival stop not confirmed')
                            continue
                        if near and captured >= command_wall:
                            stable = plateau.update(pose, captured, now)
                        else:
                            plateau.reset()
                            stable = False
                        if stable and not close:
                            approximate_stop = now
                            self._pose_progress = {**self._pose_progress, 'state': 'stopping', 'approximate': True,
                                                   'command': {'vx_m_s': 0., 'vy_m_s': 0., 'yaw_rad_s': 0.}}
                            self.bundle.twist('/cmd_vel')
                            continue
                        score = max(distance/servo.POSITION_TOLERANCE, abs(da)/servo.YAW_TOLERANCE)
                        if score < best_score-.1:
                            best_score, last_progress = score, time.monotonic()
                        elif not close and time.monotonic()-last_progress > 8:
                            raise TimeoutError(f'pose motion stalled for 8s; position_error_m={distance:.4f}; yaw_error_deg={math.degrees(da):.3f}')
                        if close:
                            servo.command(dx, dy, da, pose[2], captured)
                            self._pose_progress = {**self._pose_progress, 'command': {'vx_m_s': 0., 'vy_m_s': 0., 'yaw_rad_s': 0.}}
                            self.bundle.twist('/cmd_vel')
                            if last_capture is None or captured > last_capture:
                                if settled is None:
                                    settled = (captured, time.monotonic())
                                elif captured-settled[0] >= .3 and time.monotonic()-settled[1] >= .3:
                                    complete(False)
                                    return
                        else:
                            settled = None
                            vx, vy, wz = servo.command(dx, dy, da, pose[2], captured)
                            self._pose_progress = {**self._pose_progress, 'command': {'vx_m_s': vx, 'vy_m_s': vy, 'yaw_rad_s': wz}}
                            self.bundle.twist('/cmd_vel', x=vx, y=vy, yaw=wz)
                        last_capture = captured
            except Exception as exc:
                self._pose_progress = {**self._pose_progress, 'state': 'error', 'reason': _safe_log(str(exc))}
                self.bundle.pause(_safe_log(str(exc)), status='error', expected=(self.CHANNEL, aid))
            finally:
                if (cancelled.is_set() and self._pose_progress.get('action_id') == aid
                        and self._pose_progress.get('state') in ('executing', 'stopping')):
                    self._pose_progress = {**self._pose_progress, 'state': 'cancelled'}
        threading.Thread(target=worker, daemon=True, name='base-pose-feedback').start()
        return {'state': 'executing', 'action_id': aid, 'control_preparation': preparation,
                'speed': speed, 'linear_limit_m_s': linear, 'angular_limit_deg_s': math.degrees(angular),
                'completion_basis': 'measured_odometry_or_stable_residual',
                'approximate_limits': {'position_m': .10, 'yaw_deg': 5., 'stable_s': 1.},
                'expected_duration_s': round(math.hypot(x,y)/linear+abs(yaw)/angular+2., 1),
                'timeout_s': 110, 'stall_timeout_s': 8}


class ArmPlugin(_KuavoPlugin):
    PREFIX, CHANNEL, ALLOWED_MODES = "arm", "arms", (CARD_CONTROL_MODE,)
    JOINTS = {f'{side}_joint_{i}': (offset + i - 1, f'{side}_joint_{i}_rad',
              f'{label}臂关节{i}（zarm_{letter}{i}_joint）')
              for side, offset, label, letter in [('left', 0, '左', 'l'), ('right', 7, '右', 'r')]
              for i in range(1, 8)}

    REVERSED = frozenset((0, 3, 6, 7, 10, 13))

    def tool(self):
        fields = {}
        try:
            bounds = self.bundle.sensors.joint_bounds('arm')
        except (RuntimeError, ValueError):
            bounds = None
        for i, field, label in self.JOINTS.values():
            description = label + '绝对角度 rad；不填保持实测位置'
            entry = {'type': 'number'}
            if bounds:
                low, high = bounds[i]
                if i in self.REVERSED:
                    low, high = -high, -low
                entry.update(minimum=low, maximum=high)
                description = f'范围[{low:.6g}, {high:.6g}] rad（[{math.degrees(low):.3f}, {math.degrees(high):.3f}]度）；' + description
            else:
                description = '范围待实时模型/控制器限位核验；未核验时拒绝执行；' + description
            fields[field] = {**entry, 'description': description}
        return self.schema('Kuavo 5-W 双臂绝对角度rad；关节1/4/7使用卡片方向，原始joint_state不变。',
            fields, {'set': (list(fields), '同时设置双臂关节；未填保持当前实测角度'),
                     'zero': ([], '全部14个手臂关节归零'),
                     'stop_move': ([], '请求全身软件暂停')}, ['arm_l', 'arm_r'], ('set', 'zero'))

    def _dispatch(self, action, args):
        if action in ('set', 'zero'):
            updates = {i: (0. if action == 'zero' else
                           (-1 if i in self.REVERSED else 1) * _number(args[field], field))
                       for i, field, _ in self.JOINTS.values()
                       if action == 'zero' or _angle_supplied(args, field)}
            if not updates:
                raise ValueError('set requires at least one absolute joint angle; omitted joints keep their measured positions')
            return self._joint_set(updates)
        # Preserve the old full-array/Cartesian wire calls for existing clients;
        # the card advertises only set and its software stop.
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
                    "ik_param": {}}), allowed_modes=(CARD_CONTROL_MODE,))
        raise ValueError(f"unknown action: {action}")


class LowerBodyPlugin(_KuavoPlugin):
    CHANNEL, ALLOWED_MODES = "lower_body", (3,)
    FIELDS = {'knee_deg': (0, '膝关节knee_joint'), 'leg_deg': (1, '腿关节leg_joint'),
              'waist_pitch_deg': (2, '腰俯仰waist_pitch_joint'), 'waist_yaw_deg': (3, '腰偏航waist_yaw_joint')}

    def __init__(self, bundle, card="leg"):
        super().__init__(bundle)
        self.PREFIX = card

    def tool(self):
        if self.PREFIX == "leg":
            fields = {}
            try:
                bounds = self.bundle.sensors.joint_bounds('leg')
            except (RuntimeError, ValueError):
                bounds = None
            for field, (index, label) in self.FIELDS.items():
                entry = {'type': 'number'}
                if bounds:
                    low, high = [math.degrees(v) for v in bounds[index]]
                    entry.update(minimum=low, maximum=high)
                    prefix = f'[{low:.3f}, {high:.3f}]°；'
                else:
                    prefix = '范围待实时关节限位核验；未核验时拒绝执行；'
                fields[field] = {**entry, 'description': prefix+label+'绝对角度；set省略时保持实测值'}
            range_summary = ('、'.join(f'{field.removesuffix("_deg")}[{math.degrees(bounds[i][0]):.3f},{math.degrees(bounds[i][1]):.3f}]°'
                                      for field,(i,_) in self.FIELDS.items())
                             if bounds else '角度范围待实时关节限位核验')
            return self.schema(range_summary+"；动态高度范围见get_height；Kuavo 5-W 下肢含膝、腿、腰俯仰/偏航；set_height保持躯干前后位置和俯仰，协调前三关节升降。", {
                **fields,
                "height": {"type": "number", "description": "动态可达范围见get_height，单位m；waist_yaw_link原点相对base_link的Z坐标，非头顶离地高度"},
                "positions": _array_schema(4, "四个下肢关节目标角度，度")}, {
                'get_height': ([], '动态可达范围与当前高度，单位m；保持当前躯干x/俯仰和腰偏航，基于实时URDF和关节限位'),
                'set_height': (['height'], '动态可达范围见get_height，单位m；协调膝、腿、腰垂直升降，保持当前躯干x/俯仰和腰偏航'),
                'set': (list(self.FIELDS), '按名称设置部分/全部下肢关节绝对角度，度；至少填写一个'),
                'zero': ([], '四个下肢关节归零，包含腰部；需现场核验限位'),
                "move_joints": (["positions"], "设置四个下肢关节角度，需配置现场核验的关节限位"),
                "stop_move": ([], "请求全身软件暂停")}, "lower_body", ('set', 'zero', 'set_height', "move_joints"),
                required={"move_joints": ["positions"], "set_height": ["height"]})
    def _dispatch(self, action, args):
        if self.PREFIX == 'leg' and action == 'get_height':
            return {'state': 'ready', **self.bundle.sensors.height_state()}
        if self.PREFIX == 'leg' and action == 'set_height':
            height = _number(args.get('height'), 'height')
            target, torso = [0.] * 4, {}
            def validate():
                candidate, pose = self.bundle.sensors.height_target(height, with_pose=True)
                target[:] = candidate
                torso.update(pose)
                return self.bundle.sensors.validate_joint_target('leg', target)
            return self._async_reach('torso_reach_time',
                lambda: self.bundle.twist('/cmd_lb_torso_pose', **torso),
                joint_target=(0, target), joint_card='leg', preflight=validate)
        if self.PREFIX == 'leg' and action in ('set', 'zero'):
            updates = {i: 0. if action == 'zero' else math.radians(_number(args[field], field))
                       for field, (i, _) in self.FIELDS.items() if action == 'zero' or field in args}
            if not updates:
                raise ValueError('set requires at least one named joint angle')
            return self._joint_set(updates, 'leg')
        if self.PREFIX == "leg" and action == "move_joints":
            positions = _vector(args.get("positions"), 4, "positions")
            return self._async_reach("leg_reach_time", lambda: self.ros1.publish(
                "/lb_leg_traj", "sensor_msgs/JointState", {
                    "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
                    "name": [f"joint{i}" for i in range(1, 5)],
                    "position": positions, "velocity": [], "effort": []}),
                joint_target=(0, [math.radians(v) for v in positions]))
        raise ValueError(f"unknown action: {action}")


class HeadPlugin(_KuavoPlugin):
    PREFIX, CHANNEL = "head", "head"

    def tool(self):
        return self.schema("yaw[-30,30]°、pitch[-25,25]°；Kuavo 5-W头部绝对角度。到位≤1°；残差≤5°且稳定1s可近似完成，ACP报告实测误差。", {
            'yaw': {'type': 'number', 'minimum': -30, 'maximum': 30, 'description': '[-30,30]°；偏航绝对角度，不填保持当前实测角度'},
            'pitch': {'type': 'number', 'minimum': -25, 'maximum': 25, 'description': '[-25,25]°；俯仰绝对角度，正值向上，不填保持当前实测角度'},
        }, {'move_pos': (['yaw', 'pitch'], 'yaw[-30,30]°、pitch[-25,25]°；设置绝对角度，至少填一个，未填保持当前角度'),
            'zero': ([], '头部yaw/pitch绝对角度归零'),
            'stop_move': ([], '请求全身软件暂停')}, 'head', ('move_pos', 'zero'))

    def _dispatch(self, action, args):
        if action not in ('move_pos', 'zero'):
            raise ValueError(f'unknown action: {action}; use move_pos with absolute yaw/pitch in degrees')
        updates = {i: 0. if action == 'zero' else (-1 if i == 1 else 1) * _number(args[field], field, low, high)
                   for i, (field, low, high) in enumerate(HEAD_LIMITS_DEG)
                   if action == 'zero' or _angle_supplied(args, field)}
        if not updates:
            raise ValueError('move_pos requires yaw or pitch; omitted axes keep their measured angles')
        target_deg = [0., 0.]
        def validate():
            measured = self.bundle.sensors.joint_positions()
            current = [math.degrees(v) for v in measured[0][18:20]]
            for i, value in updates.items():
                current[i] = value
            target_deg[:] = [_number(value, field, low, high)
                             for value, (field, low, high) in zip(current, HEAD_LIMITS_DEG)]
            return measured
        def publish():
            # Cancel residual velocity integration before the SDK's absolute
            # head command. Both topics are implemented by ControlDataManager.
            self.bundle.zero_head()
            self.ros1.publish('/robot_head_motion_data', 'kuavo_msgs/robotHeadMotionData',
                              {'joint_data': target_deg})
        aid, cancelled, generation, issued, preparation = self.bundle.start_motion(
            self.CHANNEL, self.PREFIX, self.ALLOWED_MODES, publish,
            validate, request_epoch=self._request_context.stop_epoch)

        result = {'completion_basis': 'measured_head_or_stable_residual', 'measured_convergence': False,
                  'yaw': target_deg[0], 'pitch': -target_deg[1], 'units': 'deg',
                  'approximate_limits': {'max_error_deg': 5., 'stable_s': 1.}}
        def complete():
            exact = _StableFeedback(.3, angular_epsilon=.2)
            plateau = _StableFeedback(1., angular_epsilon=.2)
            error = None
            try:
                while not cancelled.wait(.02):
                    with self.tracker.lock:
                        if not self.tracker.current(self.CHANNEL, aid):
                            return
                        self.state.require_motion(self.ALLOWED_MODES)
                        if self.ros1.connection != (True, generation):
                            raise ConnectionError('connection changed during head motion')
                        q, measured_at, capture_ns = self.bundle.sensors.joint_positions()
                        now = time.monotonic()
                        if now-issued > 8.:
                            raise TimeoutError(f'head feedback did not settle within 8s; error_deg={error}')
                        if measured_at < issued:
                            continue
                        measured = [math.degrees(v) for v in q[18:20]]
                        error = [a-b for a,b in zip(target_deg, measured)]
                        close = max(map(abs, error)) <= 1.
                        near = max(map(abs, error)) <= 5.
                        if not close:
                            exact.reset()
                        if not near:
                            plateau.reset()
                        captured = capture_ns/1e9
                        arrived = close and exact.update(measured, captured, now)
                        approximate = near and plateau.update(measured, captured, now)
                        if not arrived and not approximate:
                            continue
                        approximate = not arrived
                        residual = {'yaw_deg': round(error[0], 6), 'pitch_deg': round(-error[1], 6)}
                        self.tracker.finish(self.CHANNEL, aid, self.PREFIX, 'completed', {
                            'completion_basis': 'measured_head_stable_residual' if approximate else 'measured_head_convergence',
                            'measured_convergence': not approximate, 'approximate': approximate,
                            'target': {'yaw_deg': target_deg[0], 'pitch_deg': -target_deg[1]},
                            'measured': {'yaw_deg': measured[0], 'pitch_deg': -measured[1]}, 'error': residual,
                            'message': f"Completed{' approximately' if approximate else ''}; remaining yaw error {residual['yaw_deg']:+.3f} deg, pitch error {residual['pitch_deg']:+.3f} deg."})
                        return
            except Exception as exc:
                self.bundle.pause(_safe_log(str(exc)), status='error', expected=(self.CHANNEL, aid))
        threading.Thread(target=complete, daemon=True, name='acp-head-feedback').start()
        return {'state': 'executing', 'action_id': aid,
                'expected_duration_s': 2., 'timeout_s': 8., 'control_preparation': preparation, **result}


class KuavoBundle:
    def __init__(self, config, namespace, ros1, ros2_node, media_factory=None):
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
            ("/robot_head_motion_data", "kuavo_msgs/robotHeadMotionData"),
            ("/mm/two_arm_hand_pose_cmd", "kuavo_msgs/twoArmHandPoseCmd"),
        ):
            ros1.advertise(topic, message_type)
        self.sensors = KuavoSensors(config.get('sensors', {}), namespace, ros1, ros2_node)
        self.sensors.robot_serial = config.get('robot_serial')
        self.startup_checks = StartupChecks(config.get('host_runtime', {}).get('enabled', True),
                                           config.get('provisioning', {}))
        self.peripherals = Peripherals(ros1, ros2_node, namespace, media_factory=media_factory,
            camera_ensure=self.startup_checks.ensure_camera if self.startup_checks.enabled else None)
        self.speaker = Speaker(ros2_node, media_factory,
            ensure=self.startup_checks.ensure_speaker if self.startup_checks.enabled else None)
        self.microphones = Microphones(ros2_node, namespace, self.startup_checks.provisioner,
                                       config.get('microphones', {}))
        self._auto_start = config.get('sensors', {}).get('auto_start',
            ['joints', 'imu', 'loco_state', *PERIPHERAL_CARDS])
        boot_required = config.get('host_runtime', {}).get('controller_boot_required', False)
        if type(boot_required) is not bool:
            raise ValueError('host_runtime.controller_boot_required must be a YAML boolean')
        self.state = RobotState(ros1, ros2_node, namespace, self.sensors, motion_enabled, boot_required)
        self.tracker = ActionTracker()
        self._closing = False
        self._transition = None
        self._remote_resume = None
        self._remote_mode = None
        self._stop_epoch = 0
        self.plugins = [BaseDrivePlugin(self), ArmPlugin(self), LowerBodyPlugin(self),
                        HeadPlugin(self)]
        self.spatial = ControlledSpatialPlugin(self, config.get('controlled_spatial', {}))
        self.base_servo = BaseServo(self, ros2_node)
        if "plugins" in config:
            plugin_config = dict(config['plugins'])
            # Preserve old deployments, but an explicit new setting wins.
            if 'base_drive' not in plugin_config and 'loco' in plugin_config:
                plugin_config['base_drive'] = plugin_config['loco']
            plugin_config.pop('loco', None)
            self._enabled = {name for name, cfg in plugin_config.items() if cfg.get('enabled', True)}
        else:
            self._enabled = {"base_drive", "base_servo", "controlled_spatial", "arm", "leg", "head", "speaker", "mic", "ext_mic",
                             *SENSOR_CARDS, *PERIPHERAL_CARDS}
        aliases = {'loco': 'base_drive', 'camera_rgb': 'camera_left', 'camera_rgb_right': 'camera_right'}
        self._enabled = {aliases.get(name, name) for name in self._enabled if name not in ('state', 'switch_mode', 'waist')}
        self._auto_start = [aliases.get(name, name) for name in self._auto_start if name not in ('state', 'switch_mode', 'waist')]

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
            if action != 'prepare_chassis':
                self.sensors.require_profile()
            if self.state.motion_send_uncertain:
                raise RuntimeError('motion command delivery is uncertain; verify controller recovery before further control')
            if self._resume_unresolved():
                raise RuntimeError('remote resume is unresolved; motion remains blocked; request pause after resolution')
            if self._mode_unresolved():
                raise RuntimeError('remote mode change is unresolved; motion remains blocked')
            base_only = action in ('prepare_chassis', 'mapping_velocity')
            if base_only and self.spatial.unresolved():
                raise RuntimeError(self.spatial.motion_block_reason())
            occupied = 'base' in self.tracker._active if base_only else bool(self.tracker._active)
            if self._closing or self._transition is not None or occupied:
                raise RuntimeError('motion/control is busy; stop and await confirmation before retrying')
            enabled = self.state.confirmed_enable() if action != 'prepare_chassis' else None
            if action == 'set_mode' and enabled is not True:
                raise RuntimeError('connect and resume control before changing mode')
            was_blocked = self.state.motion_blocked
            if not base_only:
                self.state.motion_blocked = True
            return self._new_transition(action), enabled, was_blocked
        finally:
            self.tracker.lock.release()

    def _end_control(self, token):
        with self.tracker.lock:
            if self._transition is token:
                self._transition = None

    def _mode_unresolved(self):
        # The commissioned controller mutates currentMpcControlMode_ before
        # returning its service result. Retain this response after cancellation
        # so an older in-flight mode write cannot overtake a new preparation.
        request = self._remote_mode
        if request is None:
            return False
        if self.ros1.connection != (True, request['generation']) or request['pending'] is None:
            return True
        response = self.ros1.service_result(request['pending'])
        if response is not None and type(response.get(request.get('accepted_key', 'result'))) is bool:
            self._remote_mode = None
            self.state.remote_mode_unresolved = False
            return False
        return True

    def _publish_motion_locked(self, channel, tool, modes, publish, validate, preparation, notify=True):
        self.state.require_motion(modes)
        if channel == 'base':
            self.require_base_control(tool)
        if validate is not None:
            validate()
        aid, cancelled = self.tracker.begin(channel, tool, notify=notify)
        issued, generation = time.monotonic(), self.ros1.connection[1]
        try:
            publish()
        except Exception as exc:
            self.state.motion_blocked = True
            self.state.motion_send_uncertain = True
            raise _UncertainMotionSend(channel, aid, str(exc)) from exc
        return aid, cancelled, generation, issued, preparation

    def start_motion(self, channel, tool, modes, publish, validate=None, request_epoch=None, notify=True):
        """One user action owns resume -> mode -> target, interruptible at each step."""
        if not self.tracker.lock.acquire(timeout=DISPATCH_LOCK_TIMEOUT_S):
            raise RuntimeError('control transition is busy; query state before retrying')
        try:
            if request_epoch is not None and request_epoch != self._stop_epoch:
                raise InterruptedError('motion request superseded by stop')
            self.state.require_motion_enabled()
            self.sensors.require_profile()
            if self._closing or self._transition is not None:
                raise RuntimeError('motion/control is busy; wait for preparation or stop it first')
            if self.state.motion_send_uncertain:
                raise RuntimeError('motion command delivery is uncertain; controller reconciliation is required')
            if channel == 'base' and self.spatial.startup_status().get('state') == 'loading':
                raise RuntimeError('Checking chassis status in background, please wait.')
            if self.spatial.unresolved() and not self.spatial.allows_independent_motion(channel):
                raise RuntimeError(self.spatial.motion_block_reason())
            if self._resume_unresolved():
                raise RuntimeError('remote resume is unresolved; motion remains blocked')
            if self._mode_unresolved():
                raise RuntimeError('remote mode change is unresolved; motion remains blocked')
            if channel in self.tracker._active:
                raise RuntimeError(f'{channel} is busy; wait for ACP or stop it first')
            if not self.ros1.motion_ready:
                raise RuntimeError('ROS 1 publisher connections are warming up; retry after 1s')
            # Never enable a robot for an invalid/unverified joint target.
            if validate is not None:
                validate()
            enabled = self.state.confirmed_enable()
            mode, seen = self.state.read('mpc_mode')
            compatible = mode == CARD_CONTROL_MODE and (not modes or mode in modes)
            fresh = time.monotonic() - seen <= 2
            preparation = {'automatic': True, 'resumed': False, 'mode_changed': False, 'mode': mode}
            upper_velocity = tool != 'controlled_spatial'
            # This vendor Bool is latched and published on changes, not a
            # heartbeat. read() invalidates it on a connection-generation change.
            velocity, _ = self.state.read('upper_velocity_enabled')
            velocity_ready = channel != 'base' or velocity is upper_velocity
            if enabled is True and not self.state.motion_blocked and compatible and fresh and velocity_ready:
                return self._publish_motion_locked(channel, tool, modes, publish, validate, preparation, notify=notify)
            base_handoff_only = (channel == 'base' and enabled is True and not self.state.motion_blocked
                                 and compatible and fresh and not velocity_ready)
            if self.tracker._active and not base_handoff_only:
                raise RuntimeError('control preparation needs an idle robot; another action is still running')
            target_mode = CARD_CONTROL_MODE
            token = self._new_transition('prepare_motion')
            token['tool'] = tool
            if not base_handoff_only:
                self.state.motion_blocked = True
        finally:
            self.tracker.lock.release()
        try:
            if enabled is not True:
                resume_issued = time.monotonic()
                self.confirm_control(token, '/enable_control', 'std_srvs/SetBool', {'data': True},
                                     'success', 'control_enabled', True)
                preparation['resumed'] = True
                # /enable_control belongs to the hardware node; the MPC node
                # receives enable asynchronously and rejects mode writes while
                # disabled. Wait for its resumed stream before a mode write.
                mode = self.wait_mode_feedback(token, resume_issued)
                compatible = mode == CARD_CONTROL_MODE and (not modes or mode in modes)
                fresh = True
            if not compatible or not fresh:
                self.confirm_control(token, '/mobile_manipulator_mpc_control',
                    'kuavo_msgs/changeTorsoCtrlMode', {'control_mode': target_mode},
                    'result', 'mpc_mode', target_mode)
                preparation['mode_changed'] = mode != target_mode
            if channel == 'base' and not velocity_ready:
                self.confirm_control(token, '/enable_vel_control', 'std_srvs/SetBool',
                    {'data': upper_velocity}, 'success', 'upper_velocity_enabled', upper_velocity)
                preparation['velocity_owner'] = 'upper_mpc' if upper_velocity else 'native_navigation'
            with self.tracker.lock:
                self._check_transition(token)
                if self._resume_unresolved() or self._mode_unresolved():
                    raise RuntimeError('remote control preparation is unresolved')
                self.state.confirmed_enable()
                self.state.motion_blocked = False
                preparation['mode'] = target_mode
                result = self._publish_motion_locked(channel, tool, modes, publish, validate, preparation, notify=notify)
                self._transition = None
                return result
        except _UncertainMotionSend:
            raise  # The caller owns cancellation of this already-tracked action.
        except Exception:
            self.pause('automatic_control_preparation_failed', status='error', expected_transition=token)
            raise
        finally:
            self._end_control(token)

    def prepare_mapping_velocity(self, request_epoch):
        """Restore the factory remote's MPC route without enabling motors."""
        with self.tracker.lock:
            if request_epoch != self._stop_epoch:
                raise InterruptedError('mapping preparation superseded by stop')
            token, _, was_blocked = self._begin_control('mapping_velocity')
        try:
            self.confirm_control(token, '/enable_vel_control', 'std_srvs/SetBool',
                {'data': True}, 'success', 'upper_velocity_enabled', True)
            with self.tracker.lock:
                self._check_transition(token)
                self.state.motion_blocked = was_blocked
        except Exception:
            self.pause('mapping_velocity_preparation_failed', expected_transition=token)
            raise
        finally:
            self._end_control(token)

    def require_base_control(self, tool):
        self.state.require_motion((CARD_CONTROL_MODE,))
        velocity, _ = self.state.read('upper_velocity_enabled')
        if velocity is not (tool != 'controlled_spatial'):
            raise RuntimeError('base velocity ownership changed; stop and prepare the requested base controller')

    def control_ownership(self):
        """Read-only policy/feedback view; never prepare or switch a controller."""
        with self.tracker.lock:
            enabled = None
            try:
                enabled = self.state.confirmed_enable()
            except RuntimeError:
                pass
            mode, _ = self.state.read('mpc_mode')
            velocity, _ = self.state.read('upper_velocity_enabled')
            source, source_at = self.state.read('base_velocity_source')
            owners = {channel: active[1] for channel, active in self.tracker._active.items()}
            transition = self._transition
            return {'mode_policy': 'BaseArm', 'required_mode': CARD_CONTROL_MODE,
                    'measured_mode': mode, 'control_enabled': enabled,
                    'motion_blocked': self.state.motion_blocked,
                    'base_velocity_route': ('upper_mpc' if velocity is True else
                                            'native_navigation' if velocity is False else 'unknown'),
                    'observed_chassis_velocity_source': source,
                    'observed_source_age_s': time.monotonic()-source_at if source is not None else None,
                    'active_resources': owners,
                    'base_exclusive': ['base_drive', 'base_servo', 'controlled_spatial', 'remote_control'],
                    'transition': transition['action'] if transition is not None else None,
                    'stop_move_scope': 'whole_body', 'servo_pause_scope': 'base_servo_only'}

    def wait_mode_feedback(self, token, after):
        deadline = time.monotonic() + CONTROL_TIMEOUT_S
        while time.monotonic() < deadline:
            with self.tracker.lock:
                self._check_transition(token)
                self.state.require_motion_enabled()
                self.state.confirmed_enable()
                mode, seen = self.state.read('mpc_mode')
                if mode in MODES and seen >= after and time.monotonic() - seen <= 2:
                    return mode
            time.sleep(.02)
        raise TimeoutError('resumed MPC mode feedback not confirmed; motion target was not sent')

    def _resume_unresolved(self):
        """Caller holds tracker.lock. Local timeout cannot cancel ROS execution.

        Only an explicit rejection, or acceptance plus fresh terminal True,
        settles our resume. A disconnected/replaced session cannot settle it.
        Subsequent pause requests still send zero/disable immediately, but may
        not claim success while this hazard remains. No automatic enable/retry.
        """
        request = self._remote_resume
        if request is None:
            return False
        if self.ros1.connection != (True, request['generation']) or request['pending'] is None:
            return True
        response = self.ros1.service_result(request['pending'])
        if response is None:
            return True
        settled = response.get('success') is False
        if response.get('success') is True:
            try:
                enabled, seen = self.state.confirmed_enable_sample()
                settled = enabled is True and seen >= request['issued']
            except RuntimeError:
                pass
        if settled and self.ros1.connection == (True, request['generation']):
            self._remote_resume = None
            self.state.remote_resume_unresolved = False
            return False
        return True

    def confirm_control(self, token, service, service_type, args, accepted_key, key, expected):
        # Serialize only validation + SEND, never the remote response wait.
        # Thus a stop packet cannot be followed by an older resume/mode SEND.
        with self.tracker.lock:
            self._check_transition(token)
            issued = time.monotonic()
            enabling = service == '/enable_control' and args.get('data') is True
            changing_mode = service in ('/mobile_manipulator_mpc_control', '/enable_vel_control')
            if enabling:
                # Mark before SEND: even a send exception can mean partial delivery.
                self._remote_resume = {'pending': None, 'issued': issued,
                                       'generation': token['generation']}
                self.state.remote_resume_unresolved = True
            if changing_mode:
                self._remote_mode = {'pending': None, 'generation': token['generation'], 'accepted_key': accepted_key}
                self.state.remote_mode_unresolved = True
            pending = self.ros1.start_service(service, service_type, args)
            if enabling:
                self._remote_resume['pending'] = pending
                self.ros1.retain_service(pending)
            if changing_mode:
                self._remote_mode['pending'] = pending
                self.ros1.retain_service(pending)
        remaining = max(0.0, issued + CONTROL_TIMEOUT_S - time.monotonic())
        response = self.ros1.wait_service(pending, remaining, token['cancelled'])
        if changing_mode:
            with self.tracker.lock:
                self._mode_unresolved()
        if response.get(accepted_key) is not True:
            if enabling:
                with self.tracker.lock:
                    self._resume_unresolved()
            raise RuntimeError(_safe_log(response.get('message', 'control request rejected')))
        remaining = max(0.0, issued + CONTROL_TIMEOUT_S - time.monotonic())
        self.wait_state(key, expected, issued, timeout=remaining, generation=token['generation'], token=token)
        if enabling:
            with self.tracker.lock:
                self._resume_unresolved()

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

    def pause(self, reason, status="cancelled", expected=None, expected_transition=None):
        with self.tracker.lock:
            if expected_transition is not None and self._transition is not expected_transition:
                return {'state': 'idle'}  # An old preparation cannot pause a newer action.
            if expected is not None and not self.tracker.current(*expected):
                return {'state': 'idle'}  # An old worker cannot cancel a newer action.
            self.base_servo.mark_paused()
            self._stop_epoch += 1
            interrupted = self._transition is not None
            unresolved = self._resume_unresolved()
            uncertain_motion = self.state.motion_send_uncertain
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
            # Whole-body disable must not wait for a blocked chassis submit.
            # Even a fresh pre-resume False cannot bypass a superseding pause.
            if not (enabled is False and not interrupted and not unresolved and not uncertain_motion and not errors):
                try:
                    self.confirm_control(token, "/enable_control", "std_srvs/SetBool", {"data": False},
                                         "success", "control_enabled", False)
                except Exception as exc:
                    errors.append(_safe_log(str(exc)))
            # Native navigation is independent of /enable_control. Always try
            # cancellation even when the whole-body disable was unconfirmed.
            try:
                self.spatial.cancel_remote()
            except Exception:
                errors.append('native navigation cancellation/stop is unconfirmed')
            with self.tracker.lock:
                self._check_transition(token)
                # If an old resume was outstanding when disable was sent,
                # even its resolution now is too late to order THIS disable.
                # A later explicit pause must send disable after resolution.
                if unresolved or self._resume_unresolved():
                    raise RuntimeError('remote resume is unresolved; pause cannot be confirmed')
                if uncertain_motion:
                    raise RuntimeError('motion command delivery is uncertain; controller reconciliation is required')
                if errors or self.spatial.unresolved():
                    raise RuntimeError('native navigation stop is unresolved')
                return {"state": "paused", "control_enabled": False,
                        "completion_basis": "enable_control_state", "scope": "whole_body"}
        except Exception as exc:
            errors.append(_safe_log(str(exc)))
            error = _safe_log('; '.join(errors))
            print(f"[pause] physical stop unconfirmed: {error}", flush=True)
            return {"state": "error", "error": error,
                    "physical_stop_confirmed": False, "motion_blocked": True,
                    "remote_resume_unresolved": self.state.remote_resume_unresolved,
                    "motion_send_uncertain": self.state.motion_send_uncertain}
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
                if self._resume_unresolved():
                    raise RuntimeError('remote resume terminal state is not confirmed')
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
                + [t for t in self.peripherals.tools() if t['name'] in self._enabled]
                + ([self.spatial.get_tool()] if 'controlled_spatial' in self._enabled else [])
                + ([self.speaker.tool()] if 'speaker' in self._enabled else [])
                + [t for t in self.microphones.tools() if t['name'] in self._enabled]
                + ([self.base_servo.tool()] if 'base_servo' in self._enabled else []))

    def start_background(self):
        self.startup_checks.start()
        self.spatial.start_background()
        self.peripherals.start()
        if self._enabled.intersection({'mic', 'ext_mic'}):
            self.microphones.start_background()
        allowed = {'joints', 'imu', 'loco_state', *PERIPHERAL_CARDS}
        for name in self._auto_start:
            if name in allowed and name in self._enabled:
                self.dispatch(name, {'action': 'start'})

    def dispatch(self, name, args):
        args = dict(args)
        # The skeleton renderer fetches the resource with arguments={}.
        action = args.pop("action", "read" if name == "model" else "start")
        if name not in self._enabled:
            return None
        if self._closing:
            return {"state": "error", "error": "driver is shutting down"}
        if name == 'base_drive' and action in ('move', 'move_relative'):
            try:
                self.base_servo.preempt()
            except Exception as exc:
                return {'state': 'error', 'error': _safe_log(str(exc))}
        if name == 'base_servo':
            return self.base_servo.dispatch(action, args)
        if name in ('mic', 'ext_mic'):
            return self.microphones.dispatch(name, action, args)
        if name == 'speaker':
            return self.speaker.dispatch(action, args)
        if name == 'controlled_spatial':
            return self.spatial.dispatch(action, args)
        if name in PERIPHERAL_CARDS:
            return self.peripherals.dispatch(name, action)
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
            needs_pause = (bool(self.tracker._active) or self._transition is not None
                           or self._remote_resume is not None or self._remote_mode is not None
                           or self.state.motion_send_uncertain or self.spatial.unresolved())
        if needs_pause:
            result = self.pause("driver_shutdown")
            if result.get("error"):
                print(f"[shutdown] {_safe_log(result.get('error'))}", flush=True)
        self.base_servo.close()
        self.state.close()
        self.spatial.close()
        self.startup_checks.close()
        self.peripherals.close()
        self.microphones.close()
        audio_stop = self.speaker.close()
        if audio_stop.get('error'):
            print(f"[speaker] shutdown {_safe_log(audio_stop['error'])}", flush=True)
        with self.tracker.lock:
            self.state._running = False
            self.sensors.close()
        self.tracker.close()
