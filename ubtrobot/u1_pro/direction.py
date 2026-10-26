"""U1 directional cards; ROS resources are created only at card start.

SoundSource has no verified cancellation contract. Unknown outcomes retain the
shared actuator reservation, rather than allowing a preset to fight the neck.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
import uuid
from pathlib import Path

from head_transport import HeadServiceTransport

from common.vendor_runtime import action_schema, tool

DOA_TOPIC = "/robo/audio/subscribe/doa_event"
NECK_TOPIC = "/mc/neck_trj_controller/state"
SOUND_SOURCE = "/mc/uworld_sound_source"


def number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} must be in [{low}, {high}]")
    return value


def parse_doa(event):
    """Decode the confirmed String JSON transport, including SDK data envelopes."""
    if hasattr(event, "data"):
        event = event.data
    for _ in range(6):
        if isinstance(event, str):
            if len(event) > 16384:
                raise ValueError("DOA event too large")
            event = json.loads(event)
        elif isinstance(event, dict) and "azimuth" not in event and "data" in event:
            event = event["data"]
        else:
            break
    if not isinstance(event, dict):
        raise ValueError("DOA event must contain an object")
    angle = number(event.get("azimuth"), "azimuth", -180, 180)
    if angle == 180:
        raise ValueError("azimuth must be less than 180")
    confidence = event.get("confidence")
    if confidence is not None:
        confidence = number(confidence, "confidence", 0, 1)
    if "valid" in event and not isinstance(event["valid"], bool):
        raise ValueError("valid must be boolean")
    return angle, confidence, event.get("valid", True)


class SoundDirectionPlugin:
    PREFIX = "sound_direction"

    def __init__(self, nodes):
        self.nodes = nodes
        self.running = False
        self._lock = threading.RLock()
        self._subscription = self._publisher = self._timer = None
        self._latest = None
        self._received = None
        self._error = ""
        self.stale_after = 2.0
        self.alpha = 0.35
        self.topic = f"/{nodes.namespace}/u1_pro/sound_direction"
        self._expired_published = False
        self._last_publish = 0.0

    def get_tool(self):
        schema = action_schema({
            "start": (["stale_after", "alpha"], "Subscribe only; never enables wakeup or turns the head."),
            "stop": ([], "Stop subscription and invalidate the last direction."),
            "info": ([], "Read direction, validity and SDK confidence."),
            "status": ([], "查看声源状态、最近方向及当前参数。"),
            "configure": (["stale_after", "alpha"], "设置过期时间与平滑系数；不启用厂商行为。"),
        }, {
            "stale_after": {"type": "number", "minimum": 0.1, "maximum": 60, "default": 2},
            "alpha": {"type": "number", "minimum": 0.01, "maximum": 1, "default": 0.35},
        })
        # Optional tuning parameters, also usable with the parameterless lifecycle start.
        return tool(self.PREFIX, "sensor", "声源方向（度，SDK 左负右正；坐标参考系待标定），只读输出。", schema,
                    topic_out=[{"topic": self.topic, "format": "data/json"}])

    def start(self, args=None):
        args = args or {}
        stale = number(args.get("stale_after", 2), "stale_after", .1, 60)
        alpha = number(args.get("alpha", .35), "alpha", .01, 1)
        with self._lock:
            if self.running:
                return self._state()
            self.stale_after, self.alpha = stale, alpha
            self._latest = self._received = None
            self._error = ""
            try:
                self._publisher = self.nodes.core.create_publisher(self.nodes.String, self.topic, 10)
                self._subscription = self.nodes.robot.create_subscription(
                    self.nodes.String, DOA_TOPIC, self._on_event, self.nodes._sensor_qos)
                self._timer = self.nodes.robot.create_timer(.2, self._expire)
                self.running = True
                self._publish()
            except Exception:
                self.stop()
                raise
            return self._state()

    def stop(self):
        with self._lock:
            self.running = False
            if self._timer is not None:
                self.nodes.robot.destroy_timer(self._timer)
                self._timer = None
            if self._subscription is not None:
                self.nodes.robot.destroy_subscription(self._subscription)
                self._subscription = None
            if self._publisher is not None:
                self._publish()
                self.nodes.core.destroy_publisher(self._publisher)
                self._publisher = None
            return self._state()

    def _state(self):
        age = None if self._received is None else max(0, time.monotonic() - self._received)
        latest = dict(self._latest or {})
        valid = bool(self.running and latest.get("event_valid") and age is not None
                     and age <= self.stale_after and not self._error)
        data_state = "stopped" if not self.running else "invalid" if self._error else "no_data" if age is None else "stale" if age > self.stale_after else "valid" if valid else "invalid"
        return {"state": "running" if self.running else "idle", "valid": valid,
                "data_state": data_state, "settings": {"stale_after": self.stale_after, "alpha": self.alpha},
                "direction_degrees": latest.get("smoothed") if valid else None,
                "raw_degrees": latest.get("raw"), "last_direction_degrees": latest.get("smoothed"),
                "confidence": latest.get("confidence"), "received_at": latest.get("received_at"),
                "age_seconds": age, "stale": age is not None and age > self.stale_after,
                "unit": "degree", "coordinate_convention": "SDK: left negative, right positive; zero usually front; reference frame unverified",
                "error": self._error, "topic_out": [{"topic": self.topic, "format": "data/json"}]}

    def _publish(self):
        message = self.nodes.String()
        message.data = json.dumps(self._state(), allow_nan=False)
        self._publisher.publish(message)
        self._last_publish = time.monotonic()

    def _on_event(self, event):
        with self._lock:
            if not self.running:
                return
            try:
                angle, confidence, valid = parse_doa(event)
                previous = self._latest
                fresh = self._received is not None and time.monotonic() - self._received <= self.stale_after
                smoothed = angle
                if valid and fresh and previous and previous["event_valid"]:
                    delta = (angle - previous["smoothed"] + 180) % 360 - 180
                    smoothed = (previous["smoothed"] + self.alpha * delta + 180) % 360 - 180
                self._received = time.monotonic()
                self._latest = {"raw": angle, "smoothed": smoothed, "confidence": confidence,
                                "event_valid": valid, "received_at": time.time()}
                self._error = ""
                self._expired_published = False
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self._error = str(exc)[:256]
                if self._latest is not None:
                    self._latest["event_valid"] = False
            self._publish()

    def _expire(self):
        with self._lock:
            if self.running:
                expired = self._state()["stale"]
                if (expired and not self._expired_published) or time.monotonic() - self._last_publish >= 1:
                    self._publish()
                    self._expired_published = expired

    def dispatch(self, action, args):
        if action == "start":
            return self.start(args)
        if action == "stop":
            return self.stop()
        if action == "configure":
            stale = number(args.get("stale_after", self.stale_after), "stale_after", .1, 60)
            alpha = number(args.get("alpha", self.alpha), "alpha", .01, 1)
            with self._lock:
                self.stale_after, self.alpha = stale, alpha
                if self.running:
                    self._publish()
                return self._state()
        if action in ("info", "status"):
            with self._lock:
                return self._state()
        return None


class HeadOrientationPlugin:
    PREFIX = "head_orientation"

    def __init__(self, nodes, audio):
        self.nodes, self.audio = nodes, audio
        # One lock owns both the audio reservation and all orientation state.
        self._lock = audio._lock
        self.running = False
        self._client = self._subscription = self._timer = None
        self._request_type = None
        self._task = None
        self._feedback = None
        self._feedback_at = None
        self._stable_since = None
        self._last_desired = None
        self._error = ""
        self._generation = 0
        self._stationary_since = None
        config = vars(nodes).get("config")
        self._state_path = Path(config.get("head_orientation_state_file", "/opt/phanthy-motus/data/u1-head-orientation.json")) if isinstance(config, dict) else None
        if self._state_path and self._state_path.exists():
            try:
                saved = json.loads(self._state_path.read_text())
                if saved.get("unresolved"):
                    self._task = saved["task"]
                    self._task["prior_state"] = self._task.get("state")
                    self._task["state"] = "prior_outcome_unknown"
                    self._task["accepted"] = False
                    self.audio._orientation_owner = self
                    self._error = "上次转向结果未知；未自动解除占用，请先人工确认机器人状态"
            except (OSError, ValueError, KeyError, TypeError, AttributeError):
                self._task = {"state": "prior_outcome_unknown", "accepted": False}
                self.audio._orientation_owner = self
                self._error = "无法读取头部任务恢复记录，保留占用"

    def get_tool(self):
        schema = action_schema({
            "start": ([], "Prepare service client and read-only neck feedback; no movement."),
            "turn": (["degree", "max_vel", "max_acc", "timeout", "confirm"], "Submit vendor degree, with_waist=false. Coordinate semantics need hardware validation."),
            "acknowledge": (["confirm"], "人工确认未知任务的剩余风险后解除本地占用；要求稳定反馈和只读链路检查，不会停止或移动机器人。"),
            "stop": ([], "Disable new turns; physical cancellation is unverified and no stop command is sent."),
            "info": ([], "Read task, controller feedback and errors; acceptance never means arrival."),
        }, {
            "action_id": {"type": "string", "description": "平台完成回调关联 ID；未指定时由驱动生成。"},
            "confirm": {"type": "boolean", "description": "Must be true for operator hardware acceptance of uncalibrated vendor motion; omitted/false sends no command."},
            "degree": {"type": "number", "minimum": -20, "maximum": 20, "description": "Vendor degree, provisional ±20 input cap; absolute/relative and sign unverified."},
            "max_vel": {"type": "number", "exclusiveMinimum": 0, "maximum": 0.4, "description": "MotionSetup vendor units, unverified. Explicit value required; no speed guarantee."},
            "max_acc": {"type": "number", "exclusiveMinimum": 0, "maximum": 0.4, "description": "MotionSetup vendor units, unverified. Explicit value required."},
            "timeout": {"type": "number", "minimum": 1, "maximum": 60, "default": 15},
        })
        schema["x-is-dangerous"] = True
        schema["x-completion"] = {"actions": ["turn"], "timeout": 75}
        return tool(self.PREFIX, "actuator", "头部水平转向：单一厂商 SoundSource 路径；回中、物理停止与角度语义待真机验收。", schema)

    def start(self):
        with self._lock:
            if self.running:
                return self._state()
            if self._client is None:
                from uworld_motion_msgs.srv import SoundSource
                from control_msgs.msg import JointTrajectoryControllerState
                try:
                    self._request_type = SoundSource.Request
                    self._client = HeadServiceTransport()
                    self._subscription = self.nodes.audio_device.create_subscription(
                        JointTrajectoryControllerState, NECK_TOPIC, self._on_feedback, self.nodes._sensor_qos)
                    self._timer = self.nodes.audio_device.create_timer(.1, self._tick)
                except Exception:
                    self.close()
                    raise
            self.running = True
            return self._state()

    def _state(self):
        return {"state": self._task["state"] if self._task else "ready" if self.running else "idle",
                "action_id": self._task.get("action_id") if self._task else None,
                "enabled": self.running, "task": dict(self._task) if self._task else None,
                "feedback": dict(self._feedback) if self._feedback else None,
                "feedback_age_seconds": None if self._feedback_at is None else time.monotonic() - self._feedback_at,
                "error": self._error, "arrival_confirmed": False,
                "center_supported": False, "physical_stop_supported": False,
                "angle_semantics": "vendor degree; sign, reference and absolute/relative unverified",
                "transport": "isolated FastDDS loopback domain 2; CycloneDDS feedback",
                "blocking": self._blocking(), "recovery_requires_operator_acknowledgement": getattr(self.audio, "_orientation_owner", None) is self}

    def turn(self, args):
        degree = number(args.get("degree"), "degree", -20, 20)
        velocity = number(args.get("max_vel"), "max_vel", 0, .4)
        acceleration = number(args.get("max_acc"), "max_acc", 0, .4)
        if velocity == 0 or acceleration == 0:
            raise ValueError("max_vel and max_acc must be positive")
        timeout = number(args.get("timeout", 15), "timeout", 1, 60)
        if args.get("confirm") is not True:
            return {"state": "error", "message": "uncalibrated vendor motion requires explicit confirm=true for operator hardware acceptance; no command sent"}
        with self._lock:
            if not self.running:
                return {"state": "error", "message": "start head_orientation first"}
            if self.audio._active or getattr(self.audio, "_orientation_owner", None):
                return {"state": "error", "message": "头部资源被占用或上次结果未知，请查看 blocking；不要重复发送转向", "blocking": self._blocking()}
            if self._feedback_at is None or time.monotonic() - self._feedback_at > 1:
                return {"state": "error", "message": "fresh neck feedback required"}
            self._generation += 1
            generation = self._generation
            self._task = {"id": str(uuid.uuid4()), "action_id": str(args.get("action_id") or uuid.uuid4())[:128], "degree": degree, "state": "checking",
                          "deadline": time.monotonic() + timeout, "accepted": False,
                          "desired_changed": False, "initial_desired": self._feedback["desired_radians"]}
            self.audio._orientation_owner = self
            self._stable_since = None
            self._error = ""
            threading.Thread(target=self._submit, args=(generation, degree, velocity, acceleration),
                             daemon=True, name="u1-head-orientation").start()
            return self._state()

    def _submit(self, generation, degree, velocity, acceleration):
        sent = False
        try:
            # Read-only checks, no implicit behavior changes. Reservation blocks local enables.
            for name in ("wakeup_enabled_state", "wakeup_followup_state", "vision_enabled_state"):
                from device import U1Nodes
                if U1Nodes._system_enabled_value(self.nodes.get_system_enabled(name)) is not False:
                    raise RuntimeError("autonomous behavior must be confirmed disabled before turning")
            if not self._client.wait_for_service(timeout_sec=1):
                raise RuntimeError("SoundSource service unavailable")
            with self._lock:
                if generation != self._generation or not self.running or self._task["state"] != "checking":
                    return
                if time.monotonic() > self._task["deadline"]:
                    raise TimeoutError("task expired before submission")
                if self._feedback_at is None or time.monotonic() - self._feedback_at > 1:
                    raise RuntimeError("neck feedback became stale before submission")
                request = self._request_type()
                request.degree, request.with_waist = degree, False
                request.revolute_setup.max_vel = velocity
                request.revolute_setup.max_acc = acceleration
                # Explicit zeros for dead/servo zones and inactive prismatic setup.
                request.revolute_setup.toy_zone = request.revolute_setup.servo_zone = 0.0
                request.prismatic_setup.max_vel = request.prismatic_setup.max_acc = 0.0
                request.prismatic_setup.toy_zone = request.prismatic_setup.servo_zone = 0.0
                self._task["state"] = "request_pending"
                self._persist(True)
                self._log_state()
                sent = True  # A call_async exception may still leave a request in flight.
                future = self._client.call_async(request)
                self._future = future  # Keep it alive; cancelling locally cannot cancel vendor motion.
                future.add_done_callback(lambda result: self._on_response(generation, result))
        except Exception as exc:
            with self._lock:
                if generation != self._generation:
                    return
                self._error = str(exc)[:256]
                self._task["state"] = "outcome_unknown" if sent else "error"
                self._log_state()
                if not sent:
                    self._release()
                self._finish("error")

    def _blocking(self):
        if self.audio._active:
            return {"kind": "audio", "task": dict(self.audio._active)}
        if getattr(self.audio, "_orientation_owner", None) is self:
            return {"kind": "head_orientation", "task": dict(self._task or {}), "error": self._error}
        return None

    def _persist(self, unresolved):
        if self._state_path:
            self._state_path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self._state_path.with_suffix(".tmp")
            with temporary.open("w") as handle:
                json.dump({"unresolved": unresolved, "task": self._task}, handle, allow_nan=False)
                handle.flush()
                os.fsync(handle.fileno())
            temporary.replace(self._state_path)
            descriptor = os.open(self._state_path.parent, os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)

    def _finish(self, status):
        task = self._task
        if not task or task.get("completion_notified") or not task.get("action_id"):
            return
        task["completion_notified"] = True
        result = {"state": task["state"], "task_id": task["id"], "action_id": task["action_id"],
                  "arrival_confirmed": False, "message": self._error}
        try:
            self._persist(getattr(self.audio, "_orientation_owner", None) is self)
        except OSError as exc:
            result["persistence_error"] = str(exc)[:128]
        from device import _acp_notify
        # Do not block feedback or stop on an HTTP completion callback.
        threading.Thread(target=_acp_notify, args=(task["action_id"], status, result, self.PREFIX),
                         daemon=True, name="u1-head-completion").start()

    def _log_state(self):
        print(f"[U1 head] state={self._task['state']} task={self._task.get('id', 'restored')} error={self._error[:256]!r}", flush=True)

    def _release(self):
        if getattr(self.audio, "_orientation_owner", None) is self:
            self._persist(False)
            self.audio._orientation_owner = None

    def _on_response(self, generation, future):
        with self._lock:
            if generation != self._generation:
                return
            try:
                response = future.result()
                self._task["vendor_status"] = int(response.status)
                self._task["vendor_message"] = str(response.message)[:256]
                if not response.success or response.status != 0:
                    self._task["state"] = "error"
                    self._error = "vendor rejected SoundSource request"
                    self._release()
                    self._finish("error")
                else:
                    self._task["accepted"] = True
                    if self._task["state"] == "request_pending":
                        self._task["state"] = "accepted_unverified"
                    # Only convergence observed AFTER the service response is considered.
                    self._stable_since = None
                    self._persist(True)
                self._log_state()
            except Exception as exc:
                self._task["state"] = "outcome_unknown"
                self._error = str(exc)[:256]
                self._log_state()
                self._finish("error")

    def _on_feedback(self, message):
        with self._lock:
            try:
                index = list(message.joint_names).index("neck_base_joint")
                actual = number(message.actual.positions[index], "actual", -math.inf, math.inf)
                desired = number(message.desired.positions[index], "desired", -math.inf, math.inf)
            except (ValueError, TypeError, IndexError, AttributeError):
                self._stable_since = None
                return
            now = time.monotonic()
            continuous = self._feedback_at is not None and now - self._feedback_at < .5
            stationary = (continuous and self._feedback is not None and abs(actual - self._feedback["actual_radians"]) < .002
                          and self._last_desired is not None and abs(desired - self._last_desired) < .001 and abs(actual - desired) < .02)
            if not stationary:
                self._stationary_since = None
            elif self._stationary_since is None:
                self._stationary_since = now
            self._feedback = {"joint": "neck_base_joint", "actual_radians": actual,
                              "desired_radians": desired, "received_at": time.time()}
            self._feedback_at = now
            if self._task and getattr(self.audio, "_orientation_owner", None) is self:
                if abs(desired - self._task.get("initial_desired", desired)) > .005:
                    self._task["desired_changed"] = True
                settled = (self._task["accepted"] and self._task["desired_changed"]
                           and abs(actual - desired) < .02 and self._last_desired is not None
                           and abs(desired - self._last_desired) < .001)
                if not settled or not continuous:
                    self._stable_since = None
                elif self._stable_since is None:
                    self._stable_since = now
                elif now - self._stable_since >= 1:
                    self._task["state"] = "settled_unverified"
                    self._error = "controller settled; requested orientation and physical stop remain unverified"
                    self._release()
                    self._finish("error")  # Controller settling is not verified requested-angle completion.
            self._last_desired = desired

    def _tick(self):
        with self._lock:
            if (self._task and getattr(self.audio, "_orientation_owner", None) is self
                    and time.monotonic() > self._task.get("deadline", float("inf"))
                    and self._task["state"] not in ("stop_unconfirmed", "timeout_unconfirmed", "outcome_unknown", "prior_outcome_unknown")):
                if self._task["state"] == "checking":
                    self._generation += 1  # Invalidate a check that has not submitted motion.
                    self._task["state"] = "error"
                    self._release()
                else:
                    self._task["state"] = "timeout_unconfirmed"
                self._error = "厂商服务未确认任务结果，保留占用；请查看任务和 blocking，不要重复转向"
                self._log_state()
                self._finish("error")

    def stop(self):
        with self._lock:
            self.running = False
            if getattr(self.audio, "_orientation_owner", None) is self:
                if self._task["state"] == "checking":
                    self._generation += 1
                    self._task["state"] = "cancelled_before_submission"
                    released = self._release()
                    self._finish("cancelled" if released else "error")
                else:
                    self._task["state"] = "stop_unconfirmed"
                    self._error = "no verified vendor cancellation; reservation retained until feedback settles"
                    self._finish("error")
            return self._state()

    def close(self):
        self.stop()
        with self._lock:
            self._generation += 1
            if self._timer is not None:
                self.nodes.audio_device.destroy_timer(self._timer)
                self._timer = None
            if self._subscription is not None:
                self.nodes.audio_device.destroy_subscription(self._subscription)
                self._subscription = None
            if self._client is not None:
                self._client.close()
                self._client = None

    def acknowledge(self, args):
        if args.get("confirm") is not True:
            return {"state": "error", "message": "必须人工确认机器人当前状态及未知请求的剩余风险，再填写 confirm=true；此操作不停止机器人"}
        with self._lock:
            if getattr(self.audio, "_orientation_owner", None) is not self:
                return self._state()
            if self._task["state"] not in ("prior_outcome_unknown", "timeout_unconfirmed", "outcome_unknown", "stop_unconfirmed"):
                return {"state": "error", "message": "任务仍在执行或检查，不允许解除占用"}
            if self._client is None or self._client.inflight:
                return {"state": "error", "message": "服务请求仍在等待结果，暂不能解除占用"}
            generation = self._generation
        try:
            if not self._client.wait_for_service(timeout_sec=1):
                raise RuntimeError("只读控制链路未就绪")
        except Exception as exc:
            return {"state": "error", "message": str(exc)[:256]}
        with self._lock:
            if generation != self._generation or self._feedback_at is None or time.monotonic() - self._feedback_at > .5 or self._stationary_since is None or time.monotonic() - self._stationary_since < 2:
                return {"state": "error", "message": "需要连续两秒新鲜且稳定的颈部反馈；没有解除占用"}
            self._task["state"] = "operator_acknowledged_unverified"
            if not self._release():
                return self._state()
            self._generation += 1
            self._error = "操作者确认后仅解除本地占用；没有确认旧请求完成或物理停止"
            self._log_state()
            return self._state()

    def dispatch(self, action, args):
        if action == "start":
            return self.start()
        if action == "acknowledge":
            return self.acknowledge(args)
        if action == "turn":
            return self.turn(args)
        if action == "center":
            return {"state": "error", "message": "neutral orientation unverified; no command sent"}
        if action == "stop":
            return self.stop()
        if action == "info":
            with self._lock:
                return self._state()
        return None
