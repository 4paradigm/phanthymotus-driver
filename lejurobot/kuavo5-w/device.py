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
from peripherals import Peripherals, CARDS as PERIPHERAL_CARDS
from speaker import Speaker
from bootstrap import config_schema as provisioning_schema
from startup import StartupChecks


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


class _UncertainMotionSend(RuntimeError):
    def __init__(self, channel, action_id, reason):
        super().__init__(_safe_log(reason))
        self.channel, self.action_id = channel, action_id


class _KuavoPlugin:
    ALLOWED_MODES = None

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
                self.start()
                return {"state": "ready"}
            if action == "stop":
                self.stop()
                return {"state": "idle"}
            if action == "info":
                return {"state": "ready" if self.ros1.connection[0] else "disconnected",
                        "motion_enabled": self.state.motion_enabled,
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
            deadline, duration, seen_at = time.monotonic() + 3, None, 0
            converged_since_ns = None
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
                                q, measured_at, capture_ns = measured
                                if time.monotonic() >= seen_at + duration + 5:
                                    raise TimeoutError('measured joint target did not converge within 5s of reach-time estimate')
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
                                else:
                                    converged_since_ns = None
                                if converged_since_ns is None or capture_ns - converged_since_ns < 200_000_000:
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
            request_epoch=self._request_context.stop_epoch)
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
                'control_preparation': preparation,
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
            "stop_move": ([], "发零速并请求全身软件暂停；下一次新动作自动准备控制")},
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
    JOINTS = {f'{side}_joint_{i}': (offset + i - 1, f'{side}_joint_{i}_rad',
              f'{label}臂关节{i}（zarm_{letter}{i}_joint）')
              for side, offset, label, letter in [('left', 0, '左', 'l'), ('right', 7, '右', 'r')]
              for i in range(1, 8)}

    def tool(self):
        fields = {field: {'type': 'number', 'description': label + '绝对角度 rad；set省略时保持实测角度'}
                  for _, field, label in self.JOINTS.values()}
        named = {name: ([field], label + '绝对角度 rad') for name, (_, field, label) in self.JOINTS.items()}
        return self.schema("Kuavo 5-W 双臂绝对关节控制：按关节名输入rad或set同时设置多关节；未指定关节保持实测位置。需现场核验限位。", {
            **fields,
            "positions": _array_schema(14, "左臂7 + 右臂7关节，度"),
            "frame": {"type": "string", "enum": ["world", "local"], "description": "末端命令坐标系，必填"},
            "left_position": _array_schema(3, "左手 xyz 米"),
            "right_position": _array_schema(3, "右手 xyz 米"),
            "left_quaternion": _array_schema(4, "左手单位四元数 xyzw"),
            "right_quaternion": _array_schema(4, "右手单位四元数 xyzw"),
        }, {**named, 'set': (list(fields), '按名称同时设置部分/全部关节，rad；至少填写一个'),
            'zero': ([], '双臂14关节绝对角度归零，需核验限位'),
            "move_joints": (["positions"], "兼容整组14关节角度，度；需现场核验限位"),
            "set_pose": (["frame", "left_position", "left_quaternion", "right_position", "right_quaternion"], "同时设置双手完整末端位姿"),
            "stop_move": ([], "请求全身软件暂停")}, ["arm_l", "arm_r"], (*named, 'set', 'zero', "move_joints", "set_pose"),
            required={**{name: [field] for name, (_, field, _) in self.JOINTS.items()},
                      "move_joints": ["positions"], "set_pose": ["frame", "left_position", "left_quaternion", "right_position", "right_quaternion"]})

    def _dispatch(self, action, args):
        if action in self.JOINTS or action in ('set', 'zero'):
            selected = self.JOINTS if action in ('set', 'zero') else {action: self.JOINTS[action]}
            updates = {i: 0. if action == 'zero' else _number(args[field], field)
                       for i, field, _ in selected.values() if action == 'zero' or field in args}
            if not updates:
                raise ValueError('at least one named absolute joint angle is required')
            return self._joint_set(updates)
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
    FIELDS = {'knee_deg': (0, '膝关节knee_joint'), 'leg_deg': (1, '腿关节leg_joint'),
              'waist_pitch_deg': (2, '腰俯仰waist_pitch_joint'), 'waist_yaw_deg': (3, '腰偏航waist_yaw_joint')}

    def __init__(self, bundle, card="leg"):
        super().__init__(bundle)
        self.PREFIX = card

    def tool(self):
        if self.PREFIX == "leg":
            return self.schema("Kuavo 5-W 下肢4关节目标，度；与 waist 共享控制通道。", {
                **{field: {'type': 'number', 'description': label + '绝对角度，度；set省略时保持实测值'}
                   for field, (_, label) in self.FIELDS.items()},
                "positions": _array_schema(4, "四个下肢关节目标角度，度")}, {
                'set': (list(self.FIELDS), '按名称设置部分/全部下肢关节绝对角度，度；至少填写一个'),
                'zero': ([], '四个下肢关节归零，包含腰部；需现场核验限位'),
                "move_joints": (["positions"], "设置四个下肢关节角度，需配置现场核验的关节限位"),
                "stop_move": ([], "请求全身软件暂停")}, "lower_body", ('set', 'zero', "move_joints"),
                required={"move_joints": ["positions"]})
        return self.schema("Kuavo 5-W 腰部绝对关节角度rad，与leg共享通道；另保留躯干相对底盘位姿接口。", {
            'waist_pitch_rad': {'type': 'number', 'description': '腰俯仰关节绝对角度rad'},
            'waist_yaw_rad': {'type': 'number', 'description': '腰偏航关节绝对角度rad'},
            "x": {"type": "number", "description": "相对底盘 X 米"},
            "z": {"type": "number", "description": "相对底盘 Z 米"},
            "pitch": {"type": "number", "description": "俯仰弧度"},
            "yaw": {"type": "number", "description": "偏航弧度"}}, {
            'waist_pitch': (['waist_pitch_rad'], '设置腰俯仰绝对关节角度rad，需下肢核验限位'),
            'waist_yaw': (['waist_yaw_rad'], '设置腰偏航绝对关节角度rad，需下肢核验限位'),
            'zero': ([], '仅腰俯仰/偏航归零，保留腿部当前角度'),
            "set_pose": (["x", "z", "pitch", "yaw"], "设置躯干完整有效位姿"),
            "stop_move": ([], "请求全身软件暂停")}, "lower_body", ('waist_pitch', 'waist_yaw', 'zero', "set_pose"),
            required={'waist_pitch': ['waist_pitch_rad'], 'waist_yaw': ['waist_yaw_rad'],
                      "set_pose": ["x", "z", "pitch", "yaw"]})

    def _dispatch(self, action, args):
        if self.PREFIX == 'leg' and action in ('set', 'zero'):
            updates = {i: 0. if action == 'zero' else math.radians(_number(args[field], field))
                       for field, (i, _) in self.FIELDS.items() if action == 'zero' or field in args}
            if not updates:
                raise ValueError('set requires at least one named joint angle')
            return self._joint_set(updates, 'leg')
        if self.PREFIX == 'waist' and action in ('waist_pitch', 'waist_yaw', 'zero'):
            if action == 'zero':
                updates = {2: 0., 3: 0.}
            else:
                field = action + '_rad'
                updates = {2 if action == 'waist_pitch' else 3: _number(args.get(field), field)}
            return self._joint_set(updates, 'leg')
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
        return self.schema("Kuavo 5-W 头部绝对角度，单位度；yaw/pitch两自由度，使用实测到位确认ACP。", {
            'yaw': {'type': 'number', 'minimum': -80, 'maximum': 80, 'description': '偏航绝对角度，度'},
            'pitch': {'type': 'number', 'minimum': -25, 'maximum': 25, 'description': '俯仰绝对角度，度'},
        }, {'move_pos': (['yaw', 'pitch'], '设置头部两个绝对角度，单位度'),
            'zero': ([], '头部yaw/pitch绝对角度归零'),
            'stop_move': ([], '请求全身软件暂停')}, 'head', ('move_pos', 'zero'),
            required={'move_pos': ['yaw', 'pitch']})

    def _dispatch(self, action, args):
        if action not in ('move_pos', 'zero'):
            raise ValueError(f'unknown action: {action}; use move_pos with absolute yaw/pitch in degrees')
        target_deg = [0., 0.] if action == 'zero' else [
            _number(args.get('yaw'), 'yaw', -80, 80), _number(args.get('pitch'), 'pitch', -25, 25)]
        target = [math.radians(v) for v in target_deg]
        def publish():
            # Cancel residual velocity integration before the SDK's absolute
            # head command. Both topics are implemented by ControlDataManager.
            self.bundle.zero_head()
            self.ros1.publish('/robot_head_motion_data', 'kuavo_msgs/robotHeadMotionData',
                              {'joint_data': target_deg})
        aid, cancelled, generation, issued, preparation = self.bundle.start_motion(
            self.CHANNEL, self.PREFIX, self.ALLOWED_MODES, publish,
            self.bundle.sensors.joint_positions, request_epoch=self._request_context.stop_epoch)

        def worker():
            converged_since_ns = last_capture_ns = None
            deadline = issued + 20.
            while not cancelled.wait(.02):
                try:
                    with self.tracker.lock:
                        if not self.tracker.current(self.CHANNEL, aid):
                            return
                        self.state.require_motion(self.ALLOWED_MODES)
                        if self.ros1.connection[1] != generation:
                            raise ConnectionError('connection changed during head motion')
                        q, measured_at, capture_ns = self.bundle.sensors.joint_positions()
                        if time.monotonic() >= deadline:
                            raise TimeoutError('head absolute target did not converge within 20s')
                        if measured_at < issued or (last_capture_ns is not None and capture_ns <= last_capture_ns):
                            continue
                        last_capture_ns = capture_ns
                        close = all(abs(a - b) <= math.radians(1.) for a, b in zip(q[18:20], target))
                        if not close:
                            converged_since_ns = None
                        elif converged_since_ns is None:
                            converged_since_ns = capture_ns
                        elif capture_ns - converged_since_ns >= 200_000_000:
                            self.tracker.finish(self.CHANNEL, aid, self.PREFIX, 'completed', {
                                'completion_basis': 'measured_head_convergence', 'measured_convergence': True,
                                'yaw': target_deg[0], 'pitch': target_deg[1], 'units': 'deg'})
                            return
                except Exception as exc:
                    self.bundle.pause(_safe_log(str(exc)), status='error', expected=(self.CHANNEL, aid))
                    return
        threading.Thread(target=worker, daemon=True, name='acp-head-position').start()
        return {'state': 'executing', 'action_id': aid, 'control_preparation': preparation,
                'completion_basis': 'measured_head_convergence', 'yaw': target_deg[0],
                'pitch': target_deg[1], 'units': 'deg'}


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
        self.plugins = [LocoPlugin(self), ArmPlugin(self), LowerBodyPlugin(self),
                        LowerBodyPlugin(self, "waist"), HeadPlugin(self)]
        self._enabled = {name for name, cfg in config.get("plugins", {}).items()
                         if cfg.get("enabled", True)} if "plugins" in config else {
                             "loco", "arm", "leg", "waist", "head", "speaker", *SENSOR_CARDS, *PERIPHERAL_CARDS}
        aliases = {'camera_rgb': 'camera_left', 'camera_rgb_right': 'camera_right'}
        self._enabled = {aliases.get(name, name) for name in self._enabled if name not in ('state', 'switch_mode')}
        self._auto_start = [aliases.get(name, name) for name in self._auto_start if name not in ('state', 'switch_mode')]

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
            if self.state.motion_send_uncertain:
                raise RuntimeError('motion command delivery is uncertain; verify controller recovery before further control')
            if self._resume_unresolved():
                raise RuntimeError('remote resume is unresolved; motion remains blocked; request pause after resolution')
            if self._mode_unresolved():
                raise RuntimeError('remote mode change is unresolved; motion remains blocked')
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
        if response is not None and type(response.get('result')) is bool:
            self._remote_mode = None
            self.state.remote_mode_unresolved = False
            return False
        return True

    def _publish_motion_locked(self, channel, tool, modes, publish, validate, preparation):
        self.state.require_motion(modes)
        if validate is not None:
            validate()
        aid, cancelled = self.tracker.begin(channel, tool)
        issued, generation = time.monotonic(), self.ros1.connection[1]
        try:
            publish()
        except Exception as exc:
            self.state.motion_blocked = True
            self.state.motion_send_uncertain = True
            raise _UncertainMotionSend(channel, aid, str(exc)) from exc
        return aid, cancelled, generation, issued, preparation

    def start_motion(self, channel, tool, modes, publish, validate=None, request_epoch=None):
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
            compatible = mode in MODES and mode != 0 and (not modes or mode in modes)
            fresh = time.monotonic() - seen <= 2
            preparation = {'automatic': True, 'resumed': False, 'mode_changed': False, 'mode': mode}
            if enabled is True and not self.state.motion_blocked and compatible and fresh:
                return self._publish_motion_locked(channel, tool, modes, publish, validate, preparation)
            if self.tracker._active:
                raise RuntimeError('control preparation needs an idle robot; another action is still running')
            target_mode = mode if compatible else 3
            token = self._new_transition('prepare_motion')
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
                compatible = mode != 0 and (not modes or mode in modes)
                target_mode = mode if compatible else 3
                fresh = True
            if not compatible or not fresh:
                self.confirm_control(token, '/mobile_manipulator_mpc_control',
                    'kuavo_msgs/changeTorsoCtrlMode', {'control_mode': target_mode},
                    'result', 'mpc_mode', target_mode)
                preparation['mode_changed'] = mode != target_mode
            with self.tracker.lock:
                self._check_transition(token)
                if self._resume_unresolved() or self._mode_unresolved():
                    raise RuntimeError('remote control preparation is unresolved')
                self.state.confirmed_enable()
                self.state.motion_blocked = False
                preparation['mode'] = target_mode
                result = self._publish_motion_locked(channel, tool, modes, publish, validate, preparation)
                self._transition = None
                return result
        except _UncertainMotionSend:
            raise  # The caller owns cancellation of this already-tracked action.
        except Exception:
            self.pause('automatic_control_preparation_failed', status='error', expected_transition=token)
            raise
        finally:
            self._end_control(token)

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
            changing_mode = service == '/mobile_manipulator_mpc_control'
            if enabling:
                # Mark before SEND: even a send exception can mean partial delivery.
                self._remote_resume = {'pending': None, 'issued': issued,
                                       'generation': token['generation']}
                self.state.remote_resume_unresolved = True
            if changing_mode:
                self._remote_mode = {'pending': None, 'generation': token['generation']}
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
            # Even a fresh pre-resume False cannot bypass a superseding pause.
            if enabled is False and not interrupted and not unresolved and not uncertain_motion:
                with self.tracker.lock:
                    self._check_transition(token)
                    return {"state": "paused", "control_enabled": False,
                            "completion_basis": "enable_control_state", "scope": "whole_body"}
            self.confirm_control(token, "/enable_control", "std_srvs/SetBool", {"data": False},
                                 "success", "control_enabled", False)
            with self.tracker.lock:
                self._check_transition(token)
                # If an old resume was outstanding when disable was sent,
                # even its resolution now is too late to order THIS disable.
                # A later explicit pause must send disable after resolution.
                if unresolved or self._resume_unresolved():
                    raise RuntimeError('remote resume is unresolved; pause cannot be confirmed')
                if uncertain_motion:
                    raise RuntimeError('motion command delivery is uncertain; controller reconciliation is required')
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
        for tool in tools:
            if tool['name'] == 'loco':
                tool['configSchema'] = provisioning_schema()
        return (tools + [t for t in self.sensors.tools() if t['name'] in self._enabled]
                + [t for t in self.peripherals.tools() if t['name'] in self._enabled]
                + ([self.speaker.tool()] if 'speaker' in self._enabled else []))

    def start_background(self):
        self.startup_checks.start()
        self.peripherals.start()
        allowed = {'joints', 'imu', 'loco_state', *PERIPHERAL_CARDS}
        for name in self._auto_start:
            if name in allowed and name in self._enabled:
                self.dispatch(name, {'action': 'start'})

    def dispatch(self, name, args):
        args = dict(args)
        action = args.pop("action", "start")
        if name not in self._enabled:
            return None
        if self._closing:
            return {"state": "error", "error": "driver is shutting down"}
        if name == 'loco' and action == 'config':
            return self.startup_checks.provisioner.configure(args)
        if name == 'speaker':
            return self.speaker.dispatch(action, args)
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
                           or self.state.motion_send_uncertain)
        if needs_pause:
            result = self.pause("driver_shutdown")
            if result.get("error"):
                print(f"[shutdown] {_safe_log(result.get('error'))}", flush=True)
        self.state.close()
        self.startup_checks.close()
        self.peripherals.close()
        audio_stop = self.speaker.close()
        if audio_stop.get('error'):
            print(f"[speaker] shutdown {_safe_log(audio_stop['error'])}", flush=True)
        with self.tracker.lock:
            self.state._running = False
            self.sensors.close()
        self.tracker.close()
