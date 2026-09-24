"""Robot-side adapter for the ActuCore motion stream (no OpenXR dependency)."""
from __future__ import annotations

import hashlib
from collections import deque
import copy
import json
import math
import os
import socket
import subprocess
import sys
from types import SimpleNamespace
from pathlib import Path
import re
import threading
import time
import xml.etree.ElementTree as ET

from motion_stream import MotionGate, vector

ARM_NAMES = [f"{side}_{joint}_joint" for side in ("left", "right") for joint in (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow_pitch",
    "wrist_yaw", "wrist_pitch", "wrist_roll")]
MOTOR_IDS = list(range(11, 18)) + list(range(21, 28))
# All motion-capable tools, including autonomous base motion and calibration.
MOTION_TOOLS = {"arm", "arm_gesture", "hand", "head", "head_gesture", "waist",
                "nav", "home", "controlled_spatial", "spatial_map", "chassis_raw", "servo"}


def load_profile(path):
    raw=Path(path).read_bytes()
    profile = json.loads(raw)
    if profile.get("schema") != "motus.tianyi-calibration.v1":
        raise ValueError("calibration_schema")
    urdf = Path(profile["urdf_path"])
    if not urdf.is_absolute(): urdf = Path(path).parent / urdf
    model_bytes=urdf.read_bytes()
    if hashlib.sha256(model_bytes).hexdigest() != profile["urdf_sha256"]:
        raise ValueError("calibration_model_changed")
    joints = {j.attrib["name"]: j for j in ET.fromstring(model_bytes).findall("joint")}
    names=profile["arm_joint_names"]
    if len(names)!=14 or len(set(names))!=14:raise ValueError("arm_joint_mapping")
    limits = [(float(joints[n].find("limit").attrib["lower"]),
               float(joints[n].find("limit").attrib["upper"])) for n in names]
    velocity=profile.get('joint_velocity_rad_s',0.2)
    if type(velocity) not in (int,float) or not 0<velocity<=min(1.5,min(float(joints[n].find('limit').attrib['velocity']) for n in names)):
        raise ValueError('joint_velocity_limit')
    hands_enabled = profile.get('hands_enabled', True)
    if type(hands_enabled) is not bool:
        raise ValueError('hands_enabled_boolean_required')
    if hands_enabled:
        for side in ("left", "right"):
            for endpoint in ("open", "closed"):
                values = vector(profile["hands"][side][endpoint], 6, "hand_profile")
                if any(not 0 <= x <= 100 for x in values):
                    raise ValueError("hand_profile_limit")
    return profile, limits, hashlib.sha256(raw).hexdigest()


def accepted(profile, *, first_acceptance=False):
    # Only a site-owned evidence record can enable Live. No example passes this gate.
    evidence = profile.get("first_acceptance" if first_acceptance else "acceptance", {})
    # Vendor documentary power/stop guarantees are not an admission condition.
    # Runtime estop, power, fault and arrival-freshness checks remain mandatory;
    # power_feedback_verified is retained only as provenance, never fabricated.
    required = ["model_verified", "workspace_verified", "pico_verified",
                "external_control_excluded"]
    if not first_acceptance:
        required += ["stop_verified", "driver_crash_verified"]
    return (all(evidence.get(k) is True for k in required)
        and bool(re.fullmatch(r"[0-9a-f]{64}", str(evidence.get("evidence_sha256", ""))))
        and all(isinstance(evidence.get(k), str) and evidence[k].strip()
                for k in ("operator", "date", "evidence_sha256")))


class TeleopExecutor:
    def __init__(self, cfg, namespace, ros2, arm, hand, plugins):
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_-]{0,63}", namespace):
            raise ValueError("teleop_requires_robot_namespace")
        self.cfg, self.ros2, self.arm, self.hand = cfg, ros2, arm, hand
        self.plugins = plugins
        self.ns = namespace
        self.topic = f"/{namespace}/motion/teleop"
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.RLock()
        self._operator_prepared = False
        self._management_receipt = None
        self._cancelled_management = {}
        self._subscribed = False
        self._streams = {}
        self._hands = {}
        self._hand_errors = {}
        self._power = None
        self._fixed_baseline = None
        self._nodes = []
        self._thread = None
        self._feedback_executor = None
        self._feedback_thread = None
        self._feedback_closed = threading.Event()
        self._feedback_error = None
        self._closed = threading.Event()
        self._output_ready = False
        self._last_vendor_command = None
        self._foreign_publishers = []
        self._trace_lock = threading.Lock()
        self._trace_enabled = False
        self._trace_id = None
        self._trace_count = 0
        self._trace_events = deque(maxlen=32)
        self._watchdog_timing = {}
        self._tick_timing = {}
        self.profile = None
        self.profile_sha256 = None
        self.profile_error = "calibration_missing"
        limits = [(-math.pi, math.pi)] * 14
        if cfg.get("calibration_path"):
            try:
                self.profile, limits, self.profile_sha256 = load_profile(cfg["calibration_path"])
                self.profile_error = None
            except (ValueError, KeyError, OSError, ET.ParseError) as exc:
                self.profile_error = str(exc)
        self.gate = MotionGate(self.feedback, self._emit, limits,
            live_enabled=cfg.get("live_enabled") is True,
            velocity=(self.profile or {}).get("joint_velocity_rad_s",0.2),
            hands_enabled=(self.profile or {}).get("hands_enabled", True),
            continuation_timeout_ms=cfg.get("continuation_timeout_ms", 300),
            feedback_fault_timeout_ms=cfg.get("feedback_fault_timeout_ms", 300),
            acceptance_check=self._execution_accepted)
        self.node = None
        self._bus_socket=None;self._bus_process=None
        self.motion_control = None
        self.teleop_control = None
        self._bus_send_lock = threading.Lock()
        self._status_thread = None
        self._bus_recovery_thread = None
        self._bus_recovery_lock = threading.Lock()
        self._bus_retry_after_ns = 0
        self._bus_health = {'generation': 0, 'state': 'stopped', 'status_dropped': 0}

    def _execution_accepted(self):
        if not self.profile:
            return False
        if accepted(self.profile):
            return True
        if (self.cfg.get('operator_session_enabled') is True and self._operator_prepared
                and self.profile.get('hands_enabled') is False):
            return True
        deadline = self.gate.first_acceptance_deadline_ns
        return (self.cfg.get('first_acceptance_enabled') is True
                and self.profile.get('hands_enabled') is False
                and accepted(self.profile, first_acceptance=True)
                and deadline is not None and self.gate.clock() < deadline)

    def _ensure_node(self):
        if self.node is None:
            from rclpy.node import Node
            from rclpy.executors import SingleThreadedExecutor
            self.node = Node(self.ns+"_motion_feedback", context=self.ros2.ctx_tianyi)
            # High-rate arm callbacks in the shared multithreaded executor can
            # repeatedly win the node's mutually-exclusive group, starving
            # power/body/hand callbacks. A dedicated sequential executor keeps
            # this safety snapshot independent of the legacy sensor workload.
            self._feedback_executor = SingleThreadedExecutor(context=self.ros2.ctx_tianyi)
            self._feedback_executor.add_node(self.node)

    def get_tool(self):
        actions = ["info", "start", "stop", "claim", "release", "pause", "recoverable_hold", "resume", "prepare_first_acceptance", "prepare_operator_session", "end_operator_session", "trace_start", "trace_stop"]
        return {"name": "teleop_executor", "type": "actuator", "multiInstance": False,
            "description": "天轶运动执行入口：仅供同机 ActuCore 获取控制权与停止，不接收 PICO 输入。",
            "x-teleop-target": {"protocol_version": 1, "robot_profile": "tianyi2",
                "namespace": self.ns, "command_topic": self.topic + "/command",
                "feedback_topic": self.topic + "/feedback"},
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": actions},
                "session_id": {"type": "string"}, "secret": {"type": "string", "format": "password"},
                "request_id": {"type": "string"}, "request_valid_until_ns": {"type": "integer"}},
                "required": ["action"], "additionalProperties": False,
                "x-resource": ["arm_l", "arm_r", "hand_l", "hand_r"],
                "x-action-params": {a: {"params": (["session_id", "secret"] if a in ("release", "pause", "stop", "resume", "recoverable_hold") else [])
                    + (["request_id", "request_valid_until_ns"] if a in ("claim", "resume", "release") else [])}
                                    for a in actions}},
            "topic_in": [{"topic": self.topic + "/command", "format": "control/teleop"}],
            "topic_out": [{"topic": self.topic + "/feedback", "format": "data/json"}]}

    def subscribe_feedback(self):
        """Read-only subscriptions, also used by the deployment preflight."""
        if self._subscribed:
            return
        self._ensure_node()
        from bodyctrl_msgs.msg import MotorStatusMsg, PowerBoardKeyStatus
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String, UInt32MultiArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        for part in ("arm", "head", "waist", "leg"):
            self.node.create_subscription(MotorStatusMsg, f"/{part}/status",
                lambda msg, p=part: self._motors(p, msg), qos)
        self.node.create_subscription(PowerBoardKeyStatus, "/power/board/key_status", self._power_cb, qos)
        for side in ("left", "right"):
            self.node.create_subscription(JointState, f"/inspire_hand/state/{side}_hand",
                lambda msg, s=side: self._hand_cb(s, msg), qos)
            self.node.create_subscription(UInt32MultiArray, f"/inspire_hand/error/{side}_hand",
                lambda msg, s=side: self._hand_error_cb(s, msg), qos)
        self._subscribed = True
        self._feedback_error = None
        self._feedback_closed.clear()
        self._feedback_thread = threading.Thread(target=self._spin_feedback,
            name='tianyi-teleop-feedback', daemon=True)
        self._feedback_thread.start()

    def _spin_feedback(self):
        # Vendor joint streams arrive around 400–500 Hz each. Draining them
        # continuously competes for the GIL with the motion watchdog. KEEP_LAST
        # depth=1 already discards superseded samples; take bounded batches and
        # yield between them instead. Safety still uses actual callback times
        # and the unchanged 100 ms freshness check, never a fabricated refresh.
        try:
            while not self._feedback_closed.is_set():
                for _ in range(10):
                    if self._feedback_closed.is_set():
                        return
                    self._feedback_executor.spin_once(timeout_sec=0.)
                self._feedback_closed.wait(.01)
        except Exception as exc:
            if not self._feedback_closed.is_set():
                self._feedback_error = type(exc).__name__
                self.gate.hold('feedback_executor_failed')

    def start(self):
        with self._lifecycle_lock:
            if (self._thread and self._thread.is_alive() and self._bus_process.poll() is None
                    and (self._feedback_executor is None or
                         self._feedback_thread and self._feedback_thread.is_alive())):
                return
            if self.gate.session_id:
                raise RuntimeError('executor_restart_requires_confirmed_release')
            try:
                self._dispose_resources()
                self._start()
            except Exception:
                self._dispose_resources()
                raise

    def _start(self):
        self.subscribe_feedback()
        self._bus_socket, self._bus_process = self._spawn_bus()
        self._closed.clear()
        self._bus_health.update(state='ready', generation=self._bus_health['generation']+1)
        self._status_thread = threading.Thread(target=self._status_loop,
            name='tianyi-motion-status', daemon=True)
        self._status_thread.start()
        self._thread = threading.Thread(target=self._watchdog_loop, name="tianyi-motion-watchdog", daemon=True)
        self._thread.start()

    def _spawn_bus(self):
        # The vendor process uses its own DDS profile. The command receiver must
        # run in a separate process so domain 42 really uses loopback-only DDS.
        parent,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
        parent.setblocking(False)
        env={**os.environ,'ROS_DOMAIN_ID':'42','RMW_IMPLEMENTATION':'rmw_fastrtps_cpp',
             'FASTRTPS_DEFAULT_PROFILES_FILE':'/opt/phanthy-motus/dds-local.xml'}
        try:
            process=subprocess.Popen([sys.executable,__file__,'--bus',str(child.fileno()),self.ns]
                + (['--control-v2'] if self.motion_control is not None else []),
                pass_fds=(child.fileno(),),env=env)
        except Exception:
            parent.close()
            raise
        finally:
            child.close()
        return parent, process

    def _communication_hold(self, reason):
        # A communication outage must not convert a recoverable v2 stream into
        # a permanent pause, or reopen an explicit pause/release/hardware fault.
        if (getattr(self.gate, '_continuous_v2', False)
                and (self.gate.state in ('ready', 'active') or self.gate._can_continue())):
            self.gate.hold(reason, recoverable=True)
        elif self.gate.state not in ('hold', 'fault'):
            self.gate.hold(reason)

    def _request_bus_recovery(self, reason):
        self._communication_hold(reason)
        self._bus_health.update(state='recovering', reason=reason)
        with self._bus_recovery_lock:
            if (self._closed.is_set() or time.monotonic_ns() < self._bus_retry_after_ns
                    or self._bus_recovery_thread and self._bus_recovery_thread.is_alive()):
                return
            self._bus_retry_after_ns = time.monotonic_ns()+250_000_000
            self._bus_recovery_thread = threading.Thread(target=self._recover_bus,
                name='tianyi-motion-bus-recovery', daemon=True)
            self._bus_recovery_thread.start()

    @staticmethod
    def _close_bus(wire, process):
        if wire is not None:
            wire.close()
        if process is not None:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=.5)

    def _recover_bus(self):
        # Only transport resources change; gate, credentials, release request
        # and motion finish receipts remain owned by their original objects.
        try:
            with self._bus_send_lock:
                old = self._bus_socket, self._bus_process
                self._bus_socket = self._bus_process = None
            self._close_bus(*old)
            if self._closed.is_set():
                return
            wire, process = self._spawn_bus()
            with self._bus_send_lock:
                if self._closed.is_set():
                    self._close_bus(wire, process)
                    return
                self._bus_socket, self._bus_process = wire, process
                self._bus_health.update(state='ready', reason=None,
                    generation=self._bus_health['generation']+1)
        except Exception as exc:
            self._bus_health.update(state='unavailable', reason=type(exc).__name__)

    def _send_bus(self, raw):
        if not self._bus_send_lock.acquire(blocking=False):
            raise BlockingIOError('local_bus_writer_busy')
        try:
            wire = self._bus_socket
            if wire is None:
                raise OSError('local_dds_unavailable')
            return wire.send(raw)
        finally:
            self._bus_send_lock.release()

    def _status_once(self):
        try:
            if self.teleop_control is not None and self.teleop_control.binding:
                value = {'teleop_device_binding': copy.deepcopy(self.teleop_control.binding),
                         'teleop_feedback': self.teleop_control.feedback()}
            else:
                value = self.info()
            self._send_bus(json.dumps(value, allow_nan=False).encode())
        except BlockingIOError:
            # Latest-only status: congestion is not a new hardware fault or a
            # reason to revoke the owner's ability to continue with fresh data.
            self._bus_health['status_dropped'] += 1
        except OSError:
            self._request_bus_recovery('feedback_transport_failed')
        except Exception as exc:
            self._bus_health['status_error'] = type(exc).__name__

    def _status_loop(self):
        while not self._closed.is_set():
            self._status_once()
            self._closed.wait(.02)

    def _watchdog_loop(self):
        while not self._closed.is_set():
            started = time.monotonic_ns()
            if self._bus_process is None or self._bus_process.poll() is not None:
                self._request_bus_recovery('local_dds_process_exited')
            self._receive_latest_command()
            received = time.monotonic_ns()
            self.tick()
            executed = time.monotonic_ns()
            finished = time.monotonic_ns()
            self._watchdog_timing = {"started_ns": started,
                "command_receive_ms": (received-started)/1e6,
                **self._tick_timing,
                "feedback_send_ms": (finished-executed)/1e6,
                "cycle_ms": (finished-started)/1e6,
                "max_cycle_ms": max(self._watchdog_timing.get("max_cycle_ms",0.), (finished-started)/1e6)}
            # 50 Hz includes the work above, not an extra 20 ms after it.
            # Rebase on each actual start: overruns never accrue catch-up ticks.
            self._closed.wait(max(0., .02-(time.monotonic_ns()-started)/1e9))

    def _receive_latest_command(self):
        # DDS depth=1 does not bound the downstream datagram socket queue.
        # Never let an expired intermediate packet pre-empt a fresh latest one.
        latest = {}
        operations = []
        received = 0
        wire = self._bus_socket
        if wire is None:
            return
        for _ in range(128):
            try:
                raw = wire.recv(65_665)  # 64 KiB public contract plus bounded IPC route wrapper.
                try:
                    value = json.loads(raw, object_pairs_hook=self._unique)
                    route = value.get('_motion_route', 'legacy') if isinstance(value, dict) else 'legacy'
                    if route == 'teleop_operation':
                        operations.append(value.get('packet'))
                        received += 1
                        continue
                    if route not in ('eef', 'arm', 'teleop_input'):
                        route = 'legacy'
                except (ValueError, TypeError, RecursionError):
                    route = 'legacy'
                latest[route] = raw
                received += 1
            except BlockingIOError:
                # Input identity is current before admitting operations, but stop
                # outranks any begin/finish in this bounded receive batch.
                if 'teleop_input' in latest:
                    self._command(SimpleNamespace(data=latest.pop('teleop_input')))
                if self.teleop_control is not None:
                    operations.sort(key=lambda p: 0 if isinstance(p,dict) and p.get('action')=='stop' else 1)
                    for operation in operations:
                        try:self.teleop_control.receive(operation)
                        except (ValueError, KeyError, TypeError) as exc:
                            self._trace('teleop_operation_rejected', reason=str(exc))
                if latest:
                    self._trace('receive_batch', received=received, superseded=received-len(latest))
                    for raw in latest.values():
                        self._command(SimpleNamespace(data=raw))
                return
            except OSError:
                self._request_bus_recovery('local_dds_receive_failed')
                return
        if self.teleop_control is not None:
            for operation in operations:
                if isinstance(operation,dict) and operation.get('action')=='stop':
                    try:self.teleop_control.receive(operation)
                    except (ValueError, KeyError, TypeError) as exc:
                        self._trace('teleop_operation_rejected', reason=str(exc))
        # A sustained flood must not monopolize the watchdog or apply a packet
        # whose position in the queue is unknown. Holding never replays it.
        self._communication_hold('local_dds_command_backlog')
        self._trace('receive_backlog', received=received, discarded=received)

    def tick(self):
        started = time.monotonic_ns()
        # A competing publisher can appear after claim; keep checking while owned.
        if self.gate.session_id:
            deadline = self.gate.first_acceptance_deadline_ns
            if deadline is not None and self.gate.clock() >= deadline:
                if self.gate.state in ('ready', 'active'):
                    self.gate.hold('first_acceptance_expired')
            try:
                if self.foreign_publishers():
                    self.gate.hold("external_motion_publishers_present")
            except Exception:
                self.gate.hold("external_control_check_failed")
        checked = time.monotonic_ns()
        self.gate.tick()
        self._tick_timing = {"ownership_check_ms": (checked-started)/1e6,
                             "gate_tick_ms": (time.monotonic_ns()-checked)/1e6}

    def _motors(self, part, msg):
        with self._lock:
            self._streams[part] = (time.monotonic_ns(), {int(x.name): (float(x.pos), float(x.speed), int(x.error)) for x in msg.status})

    def _power_cb(self, msg):
        with self._lock:
            self._power = (time.monotonic_ns(), bool(msg.is_power_on.data),
                           bool(msg.is_estop.data or msg.is_remote_estop.data))

    def _hand_cb(self, side, msg):
        with self._lock:
            try:
                values = vector(list(msg.position), 6, "hand_feedback")
                self._hands[side] = (time.monotonic_ns(), values)
            except ValueError:
                self._hands.pop(side, None)

    def _hand_error_cb(self, side, msg):
        with self._lock:
            self._hand_errors[side] = (time.monotonic_ns(), list(msg.data))

    def feedback(self):
        with self._lock:
            arm_ns, motors = self._streams.get("arm", (0, {}))
            power = self._power or (0, False, True)
            complete = all(i in motors for i in MOTOR_IDS)
            errors = [self._hand_errors.get(s, (0, [])) for s in ("left", "right")]
            invalid_motor = any(not all(math.isfinite(v) for v in x[:2]) for _,m in self._streams.values() for x in m.values())
            fixed = [self._streams.get(p, (0, {})) for p in ("head", "waist", "leg")]
            from device import _HEAD_JOINTS, _WAIST_JOINTS, _LEG_JOINTS
            maps=(_HEAD_JOINTS,_WAIST_JOINTS,_LEG_JOINTS)
            positions={str(mid):v[0] for _,m in fixed for mid,v in m.items()}
            fixed_complete=all(set(m)==set(names) for (_,m),names in zip(fixed,maps))
            locked=getattr(self,'_fixed_baseline',None)
            calibrated_fixed=(fixed_complete and all(math.isfinite(q) for q in positions.values())
                and (locked is None or (set(positions)==set(locked)
                    and all(abs(q-locked[mid])<=0.02 for mid,q in positions.items()))))
            # IK is relative to the chest. World odometry is not an input to
            # this arm-only controller; retain the calibrated body joints.
            fixed_ns = min(s[0] for s in fixed)
            faults = []
            if self._feedback_error: faults.append('feedback_executor_failed')
            if not complete: faults.append('arm_feedback_incomplete')
            if invalid_motor: faults.append('motor_feedback_nonfinite')
            hand_faults = []
            for side, (_, values) in zip(('left','right'), errors):
                if len(values) != 6: hand_faults.append(side+'_hand_errors_missing')
                elif any(values): hand_faults.append(side+'_hand_fault')
            if (self.profile or {}).get('hands_enabled', True):
                faults.extend(hand_faults)
            for part, (_, values) in self._streams.items():
                faults.extend(f'{part}_motor_{mid}_error_{value[2]}' for mid,value in values.items() if value[2] != 0)
            return {"arm_ns": arm_ns, "power_ns": power[0],
                "hand_ns": min([self._hands.get(s, (0,))[0] for s in ("left", "right")] + [t for t,_ in errors]),
                "fixed_ns": fixed_ns, "power_on": power[1], "estop": power[2],
                "fault": bool(faults), "fault_reasons": faults,
                "hand_fault_reasons": hand_faults,
                "power_freshness_basis": "subscription_arrival_only",
                "fixed_body": bool(calibrated_fixed and all(m for _,m in fixed)
                                   and all(abs(v[1]) <= 0.02 for _,m in fixed for v in m.values())),
                "fixed_motor_positions_rad": positions,
                "fixed_reference_positions_rad": dict(locked) if locked is not None else None,
                "fixed_reference_source": "robot_session_feedback" if locked is not None else "not_prepared",
                "q": [motors[i][0] for i in MOTOR_IDS] if complete else [],
                "dq": [motors[i][1] for i in MOTOR_IDS] if complete else [],
                "hands_measured": {s: v[1] for s,v in self._hands.items()}}

    def _capture_fixed_baseline(self):
        """Caller holds gate.lock; capture robot feedback only, never XR poses."""
        if self.gate.session_id:
            raise ValueError('fixed_baseline_requires_idle')
        with self._lock:
            # Ignore only the previous session's position reference. Freshness,
            # power, estop, faults and stationary checks remain mandatory.
            s, _, dq = self.gate._feedback(stopping=True)
            from device import _HEAD_JOINTS, _WAIST_JOINTS, _LEG_JOINTS
            expected={str(mid) for names in (_HEAD_JOINTS,_WAIST_JOINTS,_LEG_JOINTS) for mid in names}
            positions=s.get('fixed_motor_positions_rad',{})
            if set(positions)!=expected or any(type(q) not in (int,float) or not math.isfinite(q) for q in positions.values()):
                raise ValueError('fixed_feedback_incomplete')
            if max(map(abs,dq))>.02 or any(abs(v[1])>.02 for part in ('head','waist','leg') for v in self._streams[part][1].values()):
                raise ValueError('robot_not_stopped')
            self._fixed_baseline=dict(positions)

    def _trace(self, event, **fields):
        # Immutable bounded in-memory events only; observers do asynchronous I/O.
        if not self._trace_enabled:return
        with self._trace_lock:
            self._trace_count += 1
            self._trace_events.append({'event_id':self._trace_count,
                'monotonic_ns':time.monotonic_ns(), 'event':event, **fields})

    def _trace_snapshot(self):
        with self._trace_lock:
            return {'trace_id':self._trace_id, 'enabled':self._trace_enabled,
                    'event_count':self._trace_count, 'capacity':32,
                    'events':list(self._trace_events)}

    def _command(self, msg):
        received_ns = time.monotonic_ns()
        packet = None
        try:
            if len(msg.data) > 65_664:raise ValueError('command_too_large')
            packet = json.loads(msg.data, object_pairs_hook=self._unique)
        except (ValueError, TypeError, RecursionError):
            self.gate.hold('invalid_command')
            self._trace('command_decision',received_ns=received_ns,accepted=False,reason='invalid_command')
            return
        if (isinstance(packet, dict) and packet.get('_motion_route') == 'teleop_input'
                and self.teleop_control is not None):
            try:return self.teleop_control.receive(packet['packet'])
            except (ValueError, KeyError, TypeError) as exc:
                self._trace('teleop_input_rejected', reason=str(exc))
                return False
        if len(msg.data) > 16384:
            self.gate.hold('invalid_command')
            self._trace('command_decision',received_ns=received_ns,accepted=False,reason='invalid_command')
            return False
        if isinstance(packet, dict) and '_motion_route' in packet:
            try:
                if set(packet) != {'_motion_route', 'packet'} or self.motion_control is None:
                    raise ValueError('control_interface_unavailable')
                if type(packet['_motion_route']) is not str or type(packet['packet']) is not dict:
                    raise ValueError('invalid_control_route_payload')
                if packet['_motion_route'] == 'eef':
                    accepted = self.motion_control.receive_eef(packet['packet'])
                elif packet['_motion_route'] == 'arm':
                    accepted = self.arm.accept_control(packet['packet'])
                else:
                    raise ValueError('invalid_control_route')
                reason = None if accepted else 'hold_confirmation_or_time_fence'
            except (ValueError, TypeError, KeyError, OverflowError, RecursionError) as exc:
                accepted, reason = False, str(exc)
                if self.motion_control is not None:
                    self.motion_control.rejected(packet.get('_motion_route'), packet.get('packet'), reason)
            self._trace('control_v2_decision', received_ns=received_ns,
                        route=packet.get('_motion_route'), accepted=accepted, reason=reason)
            return
        fields = {}
        if isinstance(packet,dict):
            for key in ('session_id','seq','generated_ns'):
                value=packet.get(key)
                if type(value) is int or isinstance(value,str) and len(value)<=64:fields[key]=value
            q=packet.get('q')
            try:
                if isinstance(q,list) and len(q)==14 and all(type(x) in (int,float) and math.isfinite(x) for x in q):fields['q_rad']=list(q)
            except OverflowError:pass  # Invalid numeric input is rejected by the gate, never by logging.
        try:
            accepted=self.gate.accept(packet)
            reason=None if accepted else 'hold_confirmation_or_time_fence'
        except ValueError as exc:
            accepted=False;reason=str(exc)
        except (TypeError, RecursionError):
            self.gate.hold('invalid_command');accepted=False;reason='invalid_command'
        self._trace('command_decision',received_ns=received_ns,accepted=accepted,reason=reason,**fields)

    @staticmethod
    def _unique(pairs):
        value = {}
        for key, item in pairs:
            if key in value:
                raise ValueError("duplicate_command_field")
            value[key] = item
        return value

    def _emit(self, q, hands):
        if not self._output_ready:
            raise ValueError("output_not_initialized")
        publish_started_ns=time.monotonic_ns()
        result = self.arm._send_pos({"left": [math.degrees(x) for x in q[:7]],
                                     "right": [math.degrees(x) for x in q[7:]]}, self.gate.velocity)
        if result.get("error"):
            raise ValueError("arm_publish_failed")
        self._last_vendor_command = {"monotonic_ns": time.monotonic_ns(),
            "q_rad": list(q), "speed_rad_s": self.gate.velocity,
            "motor_ids": list(MOTOR_IDS), "publish_returned": True}
        self._trace('vendor_publish', publish_started_ns=publish_started_ns,
            publish_returned_ns=self._last_vendor_command['monotonic_ns'],
            session_id=self.gate.session_id,
            seq=self.gate.latest['seq'] if self.gate.latest else None,
            kind='target' if self.gate.latest else 'hold', q_rad=list(q), speed_rad_s=self.gate.velocity)
        if hands is not None and self.profile.get('hands_enabled', True):
            for side, closure in zip(("left", "right"), hands):
                profile = self.profile["hands"][side]
                angles = [a + closure*(b-a) for a,b in zip(profile["open"],profile["closed"])]
                if self.hand._send_angles(side, angles).get("error"):
                    raise ValueError("hand_publish_failed")

    def publish_joint_command(self, packet):
        """Validated numerical result -> arm receiver; only its gate emits."""
        if self.teleop_control is not None:
            return self.arm.accept_control(packet)  # Internal arm reuse, no extra Canvas/DDS hop.
        raw = json.dumps({'_motion_route': 'joint_output', 'packet': packet}, allow_nan=False).encode()
        if self._bus_socket is None:
            raise ValueError('local_dds_unavailable')
        try:
            self._send_bus(raw)
        except OSError as exc:
            raise ValueError('joint_publish_failed') from exc

    def install_motion_envelope(self, record):
        return self.gate.install_motion_envelope(record)

    def foreign_publishers(self):
        owned={(n.get_namespace(),n.get_name()) for p in self.plugins
               for n in [getattr(p,'_pub_node',None)] if n is not None}
        topics=['/cmd_vel','/move_base_simple/goal']+[f'/{part}/cmd_{kind}'
            for part in ('arm','head','waist','leg') for kind in ('pos','ctrl','vel','current','dis','set_zero')]
        topics += [f'/inspire_hand/ctrl/{s}_hand' for s in ('left','right')]
        foreign=[]
        for topic in topics:
            for peer in self.node.get_publishers_info_by_topic(topic):
                if (peer.node_namespace,peer.node_name) not in owned:
                    foreign.append({'topic':topic,'node':peer.node_namespace.rstrip('/')+'/'+peer.node_name})
        self._foreign_publishers=foreign
        return foreign

    def legacy_busy(self):
        return any((getattr(getattr(p, "_sequence", None), "_thread", None) is not None
                    or getattr(p, "_active_poll", None) or getattr(p, "_nav_active", False)
                    or (getattr(p, "PREFIX", None) == "servo" and getattr(p, "_running", False)))
                   for p in self.plugins)

    def info(self):
        tool = self.get_tool()
        return {**self.gate.status(), "calibration_error": self.profile_error,
                "bus": dict(self._bus_health),
                "watchdog_timing": dict(getattr(self, '_watchdog_timing', {})),
                "last_vendor_command": copy.deepcopy(getattr(self, "_last_vendor_command", None)),
                "trace": self._trace_snapshot(),
                "feedback_executor_error": self._feedback_error,
                "hands_enabled": (self.profile or {}).get('hands_enabled', True),
                "calibration_sha256": self.profile_sha256, "foreign_publishers": self._foreign_publishers,
                "publisher_present": self._output_ready,
                "x-teleop-target": tool["x-teleop-target"],
                "topic_in": tool["topic_in"], "topic_out": tool["topic_out"],
                **(self.motion_control.feedback_fields() if self.motion_control is not None else {}),
                **({'teleop_device_binding': copy.deepcopy(self.teleop_control.binding),
                    'teleop_feedback': self.teleop_control.feedback()}
                   if self.teleop_control is not None and self.teleop_control.binding else {})}

    def dispatch(self, action, args):
        if action == 'info':
            return self.info()
        with self._lifecycle_lock:
            try:
                result=self._management_dispatch(action, args)
            except ValueError as exc:
                result={"state": "error", "code": str(exc), "error": str(exc)}
            if action in ('release','stop') and not result.get('error'):
                self._operator_prepared=False
            if action not in ('trace_start','trace_stop'):
                self._trace('management_result',action=action,accepted=not bool(result.get('error')),
                            reason=result.get('code'),session_id=self.gate.session_id)
            return result

    def _management_dispatch(self, action, args):
        """Replay an acknowledged management result, never repeat its mutation.

        The nonce and original credentials stay on loopback MCP. A cancellation
        may use the original credentials if the new lease reply was lost.
        Neither a replay nor a cancellation creates a motion target.
        """
        import hmac
        request_id = args.get('request_id')
        receipt = getattr(self, '_management_receipt', None)
        if request_id is not None:
            if action not in ('claim', 'resume', 'release') or not re.fullmatch('[0-9a-f]{32}', str(request_id)):
                raise ValueError('invalid_management_request')
            until = args.get('request_valid_until_ns')
            if type(until) is not int:
                raise ValueError('invalid_management_deadline')
            now = self.gate.clock()
            cancelled = {key:expiry for key,expiry in getattr(self,'_cancelled_management',{}).items() if expiry>now}
            self._cancelled_management = cancelled
            if request_id in cancelled and action != 'release':
                raise ValueError('management_cancelled')
            if receipt and receipt['id'] == request_id:
                original = receipt['args']
                if (until != original['request_valid_until_ns']
                        or args.get('session_id') != original.get('session_id')
                        or not hmac.compare_digest(str(args.get('secret', '')), str(original.get('secret', '')))):
                    raise ValueError('invalid_management_request')
                if action == 'release':
                    # Cancellation remains valid after the RPC retry deadline;
                    # it can only release the exact lease created by this nonce.
                    with self.gate.lock:
                        if self.gate.session_id == receipt['result']['session_id']:
                            result = self.gate.hold('operator_pause', release=True)
                        elif not self.gate.session_id:
                            result = self.gate.status()
                        else:
                            raise ValueError('management_owner_changed')
                    receipt['cancelled'] = True
                    return result
                if receipt['cancelled']:
                    raise ValueError('management_cancelled')
                if action != receipt['action']:
                    raise ValueError('invalid_management_request')
                if self.gate.clock() >= until:
                    raise ValueError('management_request_expired')
                if self.gate.session_id != receipt['result']['session_id']:
                    raise ValueError('management_lease_released')
                return dict(receipt['result'])
            if action == 'release':
                # Request may never have reached us. Normal lease validation
                # still prevents cancelling someone else's owner. Keep a
                # tombstone so a delayed original request cannot claim later.
                if len(cancelled)>=64 and request_id not in cancelled:
                    raise ValueError('management_cancel_queue_full')
                cancelled[request_id]=now+300_000_000
                result=self._dispatch(action, args)
                # An initial rejected claim never emitted a teleop target. Fence
                # this nonce explicitly; do not manufacture a physical stop flag.
                with self.gate.lock:
                    if (not result.get('error') and not self.gate.session_id
                            and self.gate.applied_seq==-1 and not self.gate.output_active
                            and self._last_vendor_command is None):
                        result={**result,'cancelled_request_id':request_id,'cancelled_without_output':True}
                return result
            remaining = until-self.gate.clock()
            if remaining <= 0:
                raise ValueError('management_request_expired')
            if remaining > 300_000_000:
                raise ValueError('invalid_management_deadline')
            if len(cancelled)>=64:
                raise ValueError('management_cancel_queue_full')
        result = self._dispatch(action, args)
        if request_id is not None and not result.get('error') and action in ('claim', 'resume'):
            if receipt:
                # A delayed retry of an earlier claim must not reacquire after
                # this new management operation is subsequently released.
                self._cancelled_management[receipt['id']]=receipt['args']['request_valid_until_ns']
            self._management_receipt = {'id': request_id, 'args': dict(args),
                'action': action, 'result': dict(result), 'cancelled': False}
        elif action in ('pause', 'release', 'stop') and not result.get('error') and receipt:
            receipt['cancelled'] = True
            self._cancelled_management[receipt['id']]=receipt['args']['request_valid_until_ns']
        return result

    def _dispatch(self, action, args):
        try:
            if action in ('trace_start','trace_stop'):
                with self.gate.lock, self._trace_lock:
                    if self.gate.session_id:raise ValueError('trace_control_requires_idle')
                    if action=='trace_start':
                        if self._trace_enabled:raise ValueError('trace_already_active')
                        self._trace_id=os.urandom(8).hex();self._trace_count=0;self._trace_events.clear()
                    self._trace_enabled=action=='trace_start'
                return self._trace_snapshot()
            if action == "info":
                return self.info()
            if action == "start":
                self.start()
                return {**self.info(), "execution_state": self.gate.state, "state": "ready"}
            if action == "end_operator_session":
                if self.gate.session_id:
                    raise ValueError('operator_session_still_owned')
                self._operator_prepared = False
                return {'operator_session_prepared': False}
            if action in ("prepare_first_acceptance", "prepare_operator_session"):
                operator = action == "prepare_operator_session"
                if operator and self.cfg.get('operator_session_enabled') is not True:
                    raise ValueError('operator_session_disabled')
                if ((not operator and self.cfg.get('first_acceptance_enabled') is not True)
                        or not self.gate.live_enabled):
                    raise ValueError('first_acceptance_disabled')
                if (not self.profile or self.profile.get('hands_enabled') is not False
                        or not (operator or accepted(self.profile, first_acceptance=True))):
                    raise ValueError('first_acceptance_prerequisites_missing')
                if self.gate.session_id or self.legacy_busy():
                    raise ValueError('first_acceptance_requires_idle')
                self.start()
                if self.foreign_publishers():
                    raise ValueError('external_motion_publishers_present')
                with self.gate.lock:
                    self.gate.first_acceptance_deadline_ns = None
                    self._operator_prepared = False
                    self._capture_fixed_baseline()
                    self._operator_prepared = operator
                    if not operator:
                        self.gate.first_acceptance_deadline_ns = self.gate.clock() + 60_000_000_000
                return {**self.info(), ('operator_session_prepared' if operator else 'first_acceptance_prepared'): True}
            if action == "claim":
                self.start()
                if self.foreign_publishers():raise ValueError('external_motion_publishers_present')
                with self.gate.lock:
                    if not self.gate.session_id and accepted(self.profile or {}):
                        if self.legacy_busy():raise ValueError('legacy_motion_active')
                        self._capture_fixed_baseline()
                    lease = self.gate.claim(self.legacy_busy)
                try:
                    if not self.arm._pos_publisher:self.arm.start()
                    hands_enabled = self.profile.get('hands_enabled', True)
                    if hands_enabled and (not self.hand._left_pub or not self.hand._right_pub):self.hand.start()
                    if not self.arm._pos_publisher or (hands_enabled and not all((self.hand._left_pub, self.hand._right_pub))):
                        raise RuntimeError("hardware_publishers_unavailable")
                    self._output_ready = True
                except Exception:
                    self.gate.hold("output_start_failed", release=True)
                    raise
                return {"state": "ready", **lease}
            if action in ("pause", "release", "stop", "resume", "recoverable_hold"):
                with self.gate.lock:
                    if self.gate.session_id:
                        import hmac
                        if (args.get("session_id") != self.gate.session_id
                                or not hmac.compare_digest(str(args.get("secret", "")), self.gate.secret)):
                            raise ValueError("invalid_lease")
                    if action == "recoverable_hold":
                        if not self.gate.session_id:raise ValueError("invalid_lease")
                        return self.gate.hold("ik_recoverable", recoverable=True)
                    if action == "resume":
                        return {"state": "ready", **self.gate.resume()}
                    if action in ('release', 'stop'):
                        self._operator_prepared = False
                    return self.gate.hold("operator_pause", release=action in ("release", "stop"))
            raise ValueError("unknown_action")
        except (ValueError, RuntimeError, OSError) as exc:
            return {"state": "error", "code": str(exc), "error": str(exc)}

    def stop(self):
        if self.motion_control is not None:
            self.motion_control.stop()
        with self._lifecycle_lock:
            self._operator_prepared=False
            self.gate.hold("driver_shutdown", release=True)
            deadline = time.monotonic() + 0.6
            while self.gate.session_id and time.monotonic() < deadline:
                self.gate.tick()
                time.sleep(0.02)
            self._dispose_resources()
            # Ownership stays latched if physical stop was not confirmed.
            return self.info()

    def _dispose_resources(self):
        self._closed.set()
        if self._thread:
            self._thread.join(timeout=1)
            if self._thread.is_alive():
                raise RuntimeError('executor_thread_stop_unconfirmed')
            self._thread = None
        for name in ('_status_thread', '_bus_recovery_thread'):
            thread = getattr(self, name, None)
            if thread is not None:
                thread.join(timeout=2)
                if thread.is_alive():
                    raise RuntimeError(name.strip('_')+'_stop_unconfirmed')
                setattr(self, name, None)
        self._feedback_closed.set()
        if self._feedback_thread:
            self._feedback_thread.join(timeout=1)
            if self._feedback_thread.is_alive():
                raise RuntimeError('feedback_thread_stop_unconfirmed')
            self._feedback_thread = None
        if self._bus_process:
            if self._bus_process.poll() is None:self._bus_process.terminate()
            try:self._bus_process.wait(timeout=1)
            except subprocess.TimeoutExpired:self._bus_process.kill();self._bus_process.wait()
            self._bus_process = None
        if self._bus_socket:
            self._bus_socket.close()
            self._bus_socket = None
        if self.node is not None:
            self._feedback_executor.remove_node(self.node)
            self.node.destroy_node()
            self.node = None
        if self._feedback_executor is not None:
            self._feedback_executor.shutdown(timeout_sec=1)
            self._feedback_executor = None
        self._subscribed = False


def run_local_bus(fd,namespace,control_v2=False):
    """Only process allowed to receive teleop DDS commands; no robot-side context."""
    try:
        from common import logsafe
    except ModuleNotFoundError as exc:
        if exc.name != 'common':
            raise
        # Same checkout layout as main.py; the image stages common beside us.
        # Resolve from this source file, never from cwd or an environment path.
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
        from common import logsafe
    logsafe.install(check_fd=False)
    expected=Path(__file__).with_name('dds-local.xml').read_bytes()
    configured=Path(os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']).read_bytes()
    if configured!=expected:raise RuntimeError('local_dds_profile_mismatch')
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
    from std_msgs.msg import String
    rclpy.init(domain_id=42)
    node=Node('tianyi_teleop_local_bus')
    wire=socket.socket(fileno=fd);wire.setblocking(False)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
    def command(msg):
        if len(msg.data)>8192:return
        try:wire.send(msg.data.encode())
        except BlockingIOError:pass  # A dropped latest target expires in the Driver.
    topic=f'/{namespace}/motion/teleop'
    node.create_subscription(String,topic+'/command',command,qos)
    pub=node.create_publisher(String,topic+'/feedback',qos)
    joint_pub = None
    external_sub = external_pub = None
    external_topic = None
    def configure_external(value):
        nonlocal external_sub, external_pub, external_topic
        binding = value.get('teleop_device_binding')
        if not isinstance(binding, dict):return
        from common.teleop_contract import binding_from_topic, topics
        command_topic, feedback_topic = topics(*binding_from_topic(binding['command_topic']))
        if feedback_topic != binding.get('feedback_topic'):raise ValueError('invalid_feedback_binding')
        if external_topic == command_topic:return
        if external_sub is not None:node.destroy_subscription(external_sub)
        if external_pub is not None:node.destroy_publisher(external_pub)
        def external_command(msg):
            if len(msg.data) > 65536:return
            try:
                packet = json.loads(msg.data, object_pairs_hook=TeleopExecutor._unique)
                if len(json.dumps(packet,allow_nan=False).encode()) > 65536:return
                if packet.get('kind') != 'input':return
                route = 'teleop_input'
                wire.send(json.dumps({'_motion_route': route, 'packet': packet}, allow_nan=False).encode())
            except (ValueError, TypeError, AttributeError, RecursionError, BlockingIOError):pass
        # Latest-only input; all lifecycle operations are local MCP calls.
        external_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
        external_sub=node.create_subscription(String,command_topic,external_command,external_qos)
        external_pub=node.create_publisher(String,feedback_topic,external_qos)
        external_topic=command_topic
    if control_v2:
        def routed(route, msg):
            if len(msg.data) > 8192:return
            try:
                packet = json.loads(msg.data, object_pairs_hook=TeleopExecutor._unique)
                if type(packet) is not dict:return
                wire.send(json.dumps({'_motion_route': route, 'packet': packet}, allow_nan=False).encode())
            except (ValueError, TypeError, RecursionError, BlockingIOError):pass
        node.create_subscription(String,f'/{namespace}/motion/control/command',lambda msg:routed('eef',msg),qos)
        node.create_subscription(String,f'/{namespace}/motion/arm/command',lambda msg:routed('arm',msg),qos)
        joint_pub = node.create_publisher(String,f'/{namespace}/motion/arm/command',qos)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.01)
            latest=None;latest_joint=None
            for _ in range(128):
                try:
                    raw=wire.recv(65536)
                except BlockingIOError:break
                try:
                    # This direction carries parent feedback / solved joints,
                    # not DDS commands. A malformed datagram must not kill the
                    # shared bus or replace the previous valid latest result.
                    value=json.loads(raw,object_pairs_hook=TeleopExecutor._unique)
                    if type(value) is not dict:continue
                    if '_motion_route' in value:
                        if (set(value)!={'_motion_route','packet'}
                                or value['_motion_route']!='joint_output'
                                or type(value['packet']) is not dict):continue
                        latest_joint=json.dumps(value['packet'],allow_nan=False)
                    else:
                        configure_external(value)
                        latest=json.dumps(value,allow_nan=False)
                except (ValueError,TypeError,RecursionError):continue
            if latest_joint is not None and joint_pub is not None:
                msg=String();msg.data=latest_joint;joint_pub.publish(msg)
            if latest:
                msg=String();msg.data=latest;pub.publish(msg)
                if external_pub is not None:
                    feedback=json.loads(latest).get('teleop_feedback')
                    # Canvas can inspect unbound diagnostics through info;
                    # device-correlated feedback needs an actual input source.
                    if isinstance(feedback,dict) and feedback.get('instance_id'):
                        msg=String();msg.data=json.dumps(feedback,allow_nan=False);external_pub.publish(msg)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass  # SIGTERM can already have shut down the ROS context.
    finally:
        wire.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    if len(sys.argv) not in (4,5) or sys.argv[1]!='--bus' or (len(sys.argv)==5 and sys.argv[4]!='--control-v2'):
        raise SystemExit('internal local DDS bus only')
    run_local_bus(int(sys.argv[2]),sys.argv[3],len(sys.argv)==5)
