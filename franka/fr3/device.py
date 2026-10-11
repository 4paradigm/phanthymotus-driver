"""Read-only Franka FR3 joint states with per-card IP configuration."""

from __future__ import annotations

import json
import logging
import math
import re
import threading
from pathlib import Path

from common.vendor_runtime import tool
from hardware import SessionManager, canonical_ip


JOINT_NAMES = tuple(f"fr3_joint{i}" for i in range(1, 8))
JOINT_FORMAT = "sensor/skeleton"
log = logging.getLogger(__name__)


class RosPublisher:
    """One independently stoppable DDS publisher per canvas card."""

    def __init__(self, ros2, topic, hz, callback):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        self._ros2 = ros2
        self._message = String
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE,
        )
        self._node = Node("franka_" + topic.strip("/").replace("/", "_"), context=ros2.ctx_core)
        try:
            self._publisher = self._node.create_publisher(String, topic, qos)
            self._node.create_timer(1.0 / hz, callback)
            ros2.executor_core.add_node(self._node)
        except Exception:
            self._node.destroy_node()
            self._node = None
            raise

    def publish(self, payload):
        message = self._message()
        message.data = json.dumps(payload, allow_nan=False, separators=(",", ":"))
        self._publisher.publish(message)

    def close(self):
        node, self._node = self._node, None
        if node is not None:
            try:
                self._ros2.executor_core.remove_node(node)
            finally:
                node.destroy_node()


class _Instance:
    def __init__(self):
        self.lock = threading.RLock()
        self.config = {}
        self.session = None
        self.publisher = None
        self.publish_error = None
        self.start_epoch = 0


class FrankaPlugin:
    """Canvas instances are consumers; sessions own physical connections."""

    def __init__(self, config, namespace="franka", ros2=None, *, manager=None,
                 publisher_factory=None):
        self._namespace = namespace.strip("/") or "franka"
        self._manager = manager if manager is not None else SessionManager()
        self._publisher_factory = publisher_factory or (
            lambda topic, hz, callback: RosPublisher(ros2, topic, hz, callback)
        )
        self._lock = threading.RLock()
        self._instances = {}
        self._defaults = {}
        self._closed = False
        resource = Path(__file__).with_name("resource") / "fr3.urdf"
        self._urdf = resource.read_text(encoding="utf-8")

    def get_tools(self):
        definition = tool(
            "joint_states", "sensor",
            "Franka FR3：读取七个关节的实际角度并显示姿态；只读，不触发运动。",
            topic_out=[{"format": JOINT_FORMAT}],
        )
        definition["multiInstance"] = True
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "robot_ip": {
                    "type": "string", "title": "机械臂 IP", "scope": "instance",
                    "x-sensitive": True,
                    "description": "Franka 控制柜的 IP；相同 IP 的卡片共享状态读取连接。",
                },
                "publish_hz": {
                    "type": "number", "title": "显示频率 (Hz)", "scope": "instance",
                    "minimum": 1, "maximum": 30, "default": 10,
                    "description": "画布状态发布频率，不是机械臂控制频率。",
                },
            },
            "required": ["robot_ip"],
        }
        return [definition, tool("model", "resource", "Franka FR3 七轴运动链 URDF，供姿态显示使用。")]

    def start(self):
        # Registering the driver never opens robot connections.
        pass

    def stop(self):
        with self._lock:
            self._closed = True
            instances = list(self._instances.items())
        errors = []
        for key, instance in instances:
            try:
                with instance.lock:
                    self._stop_instance(key, instance)
            except Exception as exc:
                errors.append(exc)
        try:
            self._manager.close()
        finally:
            if errors:
                raise errors[0]

    @staticmethod
    def _instance_id(args):
        value = args.get("instance_id", "")
        if not isinstance(value, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_-]{0,127}", value):
            raise ValueError("A valid instance_id is required")
        return value

    def _get_instance(self, instance_id):
        with self._lock:
            if self._closed:
                raise RuntimeError("Driver is shutting down")
            instance = self._instances.get(instance_id)
            if instance is not None:
                return instance
            for other_id in self._instances:
                if other_id != instance_id and other_id.replace("-", "_") == instance_id.replace("-", "_"):
                    raise ValueError("instance_id collides with another card's DDS topic")
            instance = self._instances[instance_id] = _Instance()
            return instance

    def _topic(self, instance_id):
        # Match the other robot drivers' state/joints suffix, with a namespace
        # per canvas instance so several FR3s can coexist without topic clashes.
        return [{"topic": f"/{self._namespace}/{instance_id.replace('-', '_')}/state/joints",
                 "format": JOINT_FORMAT}]

    @staticmethod
    def _validate(cfg):
        cfg = dict(cfg)
        if "robot_ip" in cfg:
            value = cfg["robot_ip"]
            if not isinstance(value, str):
                raise ValueError("robot_ip must be an IP address string")
            # Clearing a saved field is allowed while idle, but cannot connect.
            cfg["robot_ip"] = canonical_ip(value) if value else ""
        value = cfg.get("publish_hz", 10)
        if isinstance(value, bool):
            raise ValueError("publish_hz must be a number from 1 to 30")
        try:
            hz = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError("publish_hz must be a number from 1 to 30") from exc
        if not math.isfinite(hz) or not 1 <= hz <= 30:
            raise ValueError("publish_hz must be a number from 1 to 30")
        cfg["publish_hz"] = hz
        return cfg

    def _info(self, instance_id, instance):
        snapshot = instance.session.snapshot() if instance.session is not None else {}
        connected = snapshot.get("connected", False)
        active = instance.publisher is not None
        state = "idle"
        if active:
            state = "running" if connected else "loading"
            if instance.publish_error:
                state = "error"
        message = "Configure robot_ip and start this card"
        if active:
            message = "Receiving robot state" if connected else (
                snapshot.get("error")
                or "Waiting for robot state; check IP, FCI activation and SDK compatibility"
            )
            message = instance.publish_error or message
        return {
            "state": state,
            "readiness": snapshot.get("state", "idle"),
            "connected": connected,
            "robot_ip": instance.config.get("robot_ip", ""),
            "read_only": True,
            "data_age_ms": snapshot.get("data_age_ms"),
            "error": instance.publish_error or snapshot.get("error"),
            "publication_error": instance.publish_error,
            "message": message,
            "topic_in": [], "topic_out": self._topic(instance_id),
        }

    def _publish(self, instance, epoch):
        # Do not block DDS teardown on a callback waiting for this instance lock.
        if not instance.lock.acquire(blocking=False):
            return
        try:
            if self._start_cancelled(instance, epoch):
                return  # A queued timer from before stop/restart owns no stream.
            if instance.publisher is None or instance.session is None:
                return
            snapshot = instance.session.snapshot()
            if not snapshot.get("connected"):
                return  # Never republish an old pose with a fresh timestamp.
            positions = snapshot.get("q", [])
            if len(positions) != 7 or not all(math.isfinite(v) for v in positions):
                return
            payload = {
                "timestamp_ms": snapshot["timestamp_ms"],
                "format": JOINT_FORMAT, "position_unit": "rad", "angle_unit": "deg",
                "joint_count": 7, "robot_mode": snapshot.get("robot_mode"),
                "joints": [
                    {"idx": i, "name": JOINT_NAMES[i], "q": position, "degree": math.degrees(position)}
                    for i, position in enumerate(positions)
                ],
            }
            instance.publisher.publish(payload)
            instance.publish_error = None
        except Exception as exc:
            # Timer exceptions escape spin_once and stop the shared DDS
            # executor. Keep a failed publisher local to its card and retry on
            # the next tick; only log the outage transition, not every frame.
            error = f"State publication failed: {type(exc).__name__}: {str(exc)[:200]}"
            if instance.publish_error is None:
                log.warning("%s", error.encode("unicode_escape").decode("ascii"))
            instance.publish_error = error
        finally:
            instance.lock.release()

    def _stop_instance(self, key, instance):
        publisher, instance.publisher = instance.publisher, None
        session, instance.session = instance.session, None
        instance.publish_error = None
        try:
            if publisher is not None:
                publisher.close()
        finally:
            if session is not None:
                self._manager.release(instance.config["robot_ip"], key)

    def _start_cancelled(self, instance, epoch):
        with self._lock:
            return self._closed or instance.start_epoch != epoch

    def dispatch(self, action, args):
        name = args.get("_tool_name", action)
        if name == "model":
            return {"urdf": self._urdf}
        if name != "joint_states":
            return None
        if action not in ("config", "start", "stop", "info"):
            return None
        allowed = ("robot_ip", "publish_hz")
        supplied = {key: args[key] for key in allowed if key in args}
        if action == "config" and args.get("instance_id", "") == "":
            # Core's sidebar may send instance fields as shared defaults.
            with self._lock:
                if self._closed:
                    raise RuntimeError("Driver is shutting down")
                self._defaults = self._validate({**self._defaults, **supplied})
            return {"state": "configured", "read_only": True}
        instance_id = self._instance_id(args)
        instance = self._get_instance(instance_id)
        with self._lock:
            epoch = instance.start_epoch
            if action == "stop":
                # Signal before waiting for initialization's instance lock.
                # A pending acquire/setup must not leave a running card after
                # the user has already asked to stop it.
                instance.start_epoch += 1
        with instance.lock:
            if action in ("config", "start"):
                with self._lock:
                    if self._closed:
                        raise RuntimeError("Driver is shutting down")
                    defaults = dict(self._defaults)
                if action == "start" and self._start_cancelled(instance, epoch):
                    return self._info(instance_id, instance)
                cfg = self._validate({**defaults, **instance.config, **supplied})
                if instance.session is not None and cfg != instance.config:
                    raise ValueError("Stop this card before changing its IP or publish frequency")
                instance.config = cfg
                if action == "start" and instance.session is None:
                    if not cfg.get("robot_ip"):
                        raise ValueError("robot_ip is required; configure this card before starting")
                    instance.session = self._manager.acquire(cfg["robot_ip"], instance_id)
                    try:
                        if self._start_cancelled(instance, epoch):
                            self._stop_instance(instance_id, instance)
                            return self._info(instance_id, instance)
                        instance.publisher = self._publisher_factory(
                            self._topic(instance_id)[0]["topic"],
                            cfg["publish_hz"],
                            lambda: self._publish(instance, epoch),
                        )
                        if self._start_cancelled(instance, epoch):
                            self._stop_instance(instance_id, instance)
                    except Exception:
                        self._stop_instance(instance_id, instance)
                        raise
                return self._info(instance_id, instance)
            if action == "stop":
                self._stop_instance(instance_id, instance)
                return self._info(instance_id, instance)
            if action == "info":
                return self._info(instance_id, instance)
        return None


def build_plugins(config, namespace, ros2):
    return [FrankaPlugin(config, namespace, ros2)]
