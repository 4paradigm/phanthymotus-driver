"""Q5 IMU card: linear acceleration + angular velocity from D455 accel/gyro."""

from __future__ import annotations

import json
import math
import time

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "imu"
TYPE = "sensor"
TOPIC = "/{ns}/q5/imu"
FMT = "data/json"
HZ = 10.0
NODE = "q5_imu"
DESC = "Q5 IMU：D455 加速度计 + 陀螺仪，线加速度与角速度"
STALE_THRESHOLD_MS = 2000


def _vec(v):
    if v is None:
        return None
    x, y, z = getattr(v, "x", None), getattr(v, "y", None), getattr(v, "z", None)
    if x is None or y is None or z is None:
        return None
    if not (math.isfinite(float(x)) and math.isfinite(float(y)) and math.isfinite(float(z))):
        return None
    return {"x": float(x), "y": float(y), "z": float(z)}


def _magnitude(v):
    if not isinstance(v, dict):
        return None
    try:
        return round(math.sqrt(v["x"] ** 2 + v["y"] ** 2 + v["z"] ** 2), 6)
    except (KeyError, TypeError):
        return None


def build(snap: dict, now_ms: int | None = None) -> dict:
    now_ms = int(time.time() * 1000) if now_ms is None else now_ms
    received_at_ms = snap.get("received_at_ms")
    age_ms = None if received_at_ms is None else now_ms - received_at_ms
    fresh = age_ms is not None and age_ms <= STALE_THRESHOLD_MS
    accel = snap.get("linear_acceleration")
    gyro = snap.get("angular_velocity")
    accel_vec = _vec(accel) if accel else None
    gyro_vec = _vec(gyro) if gyro else None

    data = {
        "timestamp_ms": now_ms,
        "received_at_ms": received_at_ms,
        "age_ms": age_ms,
        "fresh": bool(fresh),
        "available": bool(snap.get("available", False)),
        "source_topics": ["/camera/camera/accel/sample", "/camera/camera/gyro/sample"],
    }
    if accel_vec:
        data["linear_acceleration"] = accel_vec
        data["linear_acceleration_magnitude"] = _magnitude(accel_vec)
    if gyro_vec:
        data["angular_velocity"] = gyro_vec
        data["angular_velocity_magnitude"] = _magnitude(gyro_vec)
    if not data["available"]:
        data["message"] = "未收到 D455 IMU 消息"
    elif not fresh:
        data["message"] = "IMU 消息已过期"
    return data


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = TOPIC.format(ns=namespace)
        self._node = None
        self._pub = None
        self._executor = executor
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0 / HZ, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[{CARD}] ROS2 publisher unavailable: {e}", flush=True)
                self._node = None
                self._pub = None

    def _data(self):
        return build(self._client.sensor_snapshot("imu"))

    def _tick(self):
        if self._pub is None:
            return
        msg = String()
        msg.data = json.dumps(self._data(), ensure_ascii=False)
        self._pub.publish(msg)

    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": ["info", "start", "stop"]},
            }, "required": ["action"], "additionalProperties": False},
            "topic_out": topic_out(self._topic, FMT),
        }

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "running" if self._pub else "unavailable"}
        if action == "stop":
            return {"state": "idle"}
        if action in ("info", "read", "get", CARD):
            return {"state": "running" if self._pub else "unavailable", "data": self._data(),
                    "topic_out": topic_out(self._topic, FMT)}
        return None

    def stop(self):
        """Remove the ROS2 node from the executor and destroy it on shutdown."""
        if self._node is not None and self._executor is not None:
            try:
                self._executor.remove_node(self._node)
            except Exception:
                pass
        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            finally:
                self._node = None
                self._pub = None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
