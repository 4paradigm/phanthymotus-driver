"""Unitree G1 joint health sensor card.

The analyzer is deliberately independent from ROS2/DDS so its safety-related
classification can be tested without robot hardware.  The plugin layer only
adapts ``rt/lowstate`` samples to the analyzer and publishes JSON snapshots.
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque


G1_MOTOR_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
    "reserved_motor_29", "reserved_motor_30", "reserved_motor_31",
    "reserved_motor_32", "reserved_motor_33", "reserved_motor_34",
)

_LEVEL_RANK = {"normal": 0, "warning": 1, "critical": 2}


def _finite(value, default=0.0) -> float:
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


class JointHealthAnalyzer:
    """Classify G1 motor telemetry using conservative configurable limits."""

    def __init__(self, config: dict | None = None):
        cfg = config or {}
        self.warning_temp_c = float(cfg.get("warning_temp_c", 70.0))
        self.critical_temp_c = float(cfg.get("critical_temp_c", 85.0))
        self.warning_temp_rise_c_per_min = float(
            cfg.get("warning_temp_rise_c_per_min", 12.0)
        )
        self.critical_temp_rise_c_per_min = float(
            cfg.get("critical_temp_rise_c_per_min", 20.0)
        )
        self.warning_torque_nm = float(cfg.get("warning_torque_nm", 45.0))
        self.critical_torque_nm = float(cfg.get("critical_torque_nm", 70.0))
        self.torque_consecutive_samples = max(
            1, int(cfg.get("torque_consecutive_samples", 10))
        )
        self.stale_timeout_sec = float(cfg.get("stale_timeout_sec", 1.0))
        self.publish_interval_sec = float(cfg.get("publish_interval_sec", 0.5))
        self.sample_interval_sec = float(cfg.get("sample_interval_sec", 0.1))
        self._history_window_sec = max(
            5.0, float(cfg.get("temperature_history_sec", 30.0))
        )
        self._lock = threading.Lock()
        self._latest: list[dict] = []
        self._last_update: float | None = None
        self._temp_history = [deque() for _ in G1_MOTOR_NAMES]
        self._torque_counts = [0 for _ in G1_MOTOR_NAMES]

        if self.warning_temp_c >= self.critical_temp_c:
            raise ValueError("warning_temp_c must be lower than critical_temp_c")
        if self.warning_temp_rise_c_per_min >= self.critical_temp_rise_c_per_min:
            raise ValueError(
                "warning_temp_rise_c_per_min must be lower than "
                "critical_temp_rise_c_per_min"
            )
        if self.warning_torque_nm >= self.critical_torque_nm:
            raise ValueError("warning_torque_nm must be lower than critical_torque_nm")

    @staticmethod
    def _read(motor, field, default=0.0):
        if isinstance(motor, dict):
            return motor.get(field, default)
        return getattr(motor, field, default)

    def update(self, motors, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else float(now)
        latest = []
        with self._lock:
            for idx, motor in enumerate(list(motors)[:len(G1_MOTOR_NAMES)]):
                temperatures = self._read(motor, "temperature", []) or []
                valid_temperatures = [
                    _finite(value) for value in temperatures
                    if -40.0 <= _finite(value, -999.0) <= 200.0
                ]
                temperature = max(valid_temperatures) if valid_temperatures else None
                torque = _finite(self._read(motor, "tau_est", 0.0))

                if temperature is not None:
                    history = self._temp_history[idx]
                    history.append((now, temperature))
                    cutoff = now - self._history_window_sec
                    while len(history) > 1 and history[0][0] < cutoff:
                        history.popleft()
                    elapsed = now - history[0][0]
                    rise_rate = (
                        (temperature - history[0][1]) * 60.0 / elapsed
                        if elapsed >= 5.0 else 0.0
                    )
                else:
                    rise_rate = 0.0

                abs_torque = abs(torque)
                if abs_torque >= self.warning_torque_nm:
                    self._torque_counts[idx] += 1
                else:
                    self._torque_counts[idx] = 0

                latest.append({
                    "idx": idx,
                    "joint": G1_MOTOR_NAMES[idx],
                    "temperature_c": temperature,
                    "temperature_rise_c_per_min": round(rise_rate, 2),
                    "torque_nm": round(torque, 3),
                    "velocity_rad_s": round(
                        _finite(self._read(motor, "dq", 0.0)), 4
                    ),
                    "position_rad": round(
                        _finite(self._read(motor, "q", 0.0)), 4
                    ),
                    "motorstate_raw": int(self._read(motor, "motorstate", 0)),
                    "over_torque_samples": self._torque_counts[idx],
                })
            self._latest = latest
            self._last_update = now
        return self.snapshot(now)

    def reset_baseline(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            for idx, item in enumerate(self._latest):
                temperature = item["temperature_c"]
                self._temp_history[idx].clear()
                if temperature is not None:
                    self._temp_history[idx].append((now, temperature))
                self._torque_counts[idx] = 0
                item["over_torque_samples"] = 0
                item["temperature_rise_c_per_min"] = 0.0
        return self.snapshot(now)

    def _alerts_for(self, motor: dict) -> list[dict]:
        alerts = []

        def add(level: str, reason: str, value, threshold):
            alerts.append({
                "level": level,
                "reason": reason,
                "joint": motor["joint"],
                "motor_idx": motor["idx"],
                "value": value,
                "threshold": threshold,
            })

        temperature = motor["temperature_c"]
        if temperature is None:
            add("warning", "temperature_unavailable", None, None)
        elif temperature >= self.critical_temp_c:
            add("critical", "temperature_high", temperature, self.critical_temp_c)
        elif temperature >= self.warning_temp_c:
            add("warning", "temperature_high", temperature, self.warning_temp_c)

        rise_rate = motor["temperature_rise_c_per_min"]
        if rise_rate >= self.critical_temp_rise_c_per_min:
            add(
                "critical", "temperature_rising_fast", rise_rate,
                self.critical_temp_rise_c_per_min,
            )
        elif rise_rate >= self.warning_temp_rise_c_per_min:
            add(
                "warning", "temperature_rising_fast", rise_rate,
                self.warning_temp_rise_c_per_min,
            )

        if motor["over_torque_samples"] >= self.torque_consecutive_samples:
            abs_torque = abs(motor["torque_nm"])
            if abs_torque >= self.critical_torque_nm:
                add("critical", "sustained_high_torque", abs_torque, self.critical_torque_nm)
            else:
                add("warning", "sustained_high_torque", abs_torque, self.warning_torque_nm)
        return alerts

    def snapshot(self, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            latest = [dict(item) for item in self._latest]
            last_update = self._last_update

        if last_update is None:
            return {
                "state": "waiting",
                "health_level": "warning",
                "reason": "no_lowstate_data",
                "summary": {"healthy": 0, "warning": 0, "critical": 0},
                "alerts": [],
                "motors": [],
                "last_update_ago_ms": None,
            }

        age_sec = max(0.0, now - last_update)
        alerts = []
        motor_levels = []
        for motor in latest:
            motor_alerts = self._alerts_for(motor)
            alerts.extend(motor_alerts)
            motor["health_level"] = max(
                (alert["level"] for alert in motor_alerts),
                key=lambda level: _LEVEL_RANK[level],
                default="normal",
            )
            motor_levels.append(motor["health_level"])

        state = "running"
        if age_sec > self.stale_timeout_sec:
            state = "stale"
            alerts.insert(0, {
                "level": "critical",
                "reason": "lowstate_timeout",
                "joint": None,
                "motor_idx": None,
                "value": round(age_sec, 3),
                "threshold": self.stale_timeout_sec,
            })

        counts = {
            level: sum(1 for item_level in motor_levels if item_level == level)
            for level in ("normal", "warning", "critical")
        }
        health_level = max(
            (alert["level"] for alert in alerts),
            key=lambda level: _LEVEL_RANK[level],
            default="normal",
        )
        return {
            "state": state,
            "health_level": health_level,
            "summary": {
                "healthy": counts["normal"],
                "warning": counts["warning"],
                "critical": counts["critical"],
            },
            "alerts": alerts,
            "motors": latest,
            "last_update_ago_ms": round(age_sec * 1000),
        }


class JointHealthPlugin:
    PREFIX = "joint_health"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._topic = f"/{namespace}/health/joints"
        self._analyzer = JointHealthAnalyzer(plugin_config)
        self._node = self._make_node()
        executor.add_node(self._node)

    def _make_node(self):
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
        )
        from std_msgs.msg import String
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        analyzer = self._analyzer
        topic = self._topic
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        class JointHealthNode(Node):
            def __init__(self):
                super().__init__("g1_joint_health")
                self._publisher = self.create_publisher(String, topic, qos)
                self._last_publish = 0.0
                self._last_sample = 0.0
                self._subscriber = None
                try:
                    # Keep a strong reference: otherwise the DDS reader may be
                    # garbage-collected and silently stop delivering samples.
                    self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
                    self._subscriber.Init(self._on_lowstate, 10)
                    self.get_logger().info(
                        f"JointHealthNode subscribed rt/lowstate -> {topic}"
                    )
                except Exception as exc:
                    self.get_logger().warning(
                        f"JointHealthNode failed to subscribe rt/lowstate: {exc}"
                    )

            def _on_lowstate(self, msg):
                now = time.monotonic()
                if now - self._last_sample < analyzer.sample_interval_sec:
                    return
                self._last_sample = now
                snapshot = analyzer.update(msg.motor_state, now)
                if now - self._last_publish < analyzer.publish_interval_sec:
                    return
                self._last_publish = now
                output = String()
                output.data = json.dumps(snapshot, ensure_ascii=False)
                self._publisher.publish(output)

        return JointHealthNode()

    def get_tool(self) -> dict:
        return {
            "name": "joint_health",
            "type": "sensor",
            "multiInstance": False,
            "description": (
                "G1 joint health diagnostics: per-joint temperature, temperature "
                "rise, sustained torque, and lowstate freshness. Read-only; does "
                "not stop or command the robot."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["snapshot", "list_alerts", "reset_baseline"],
                        "default": "snapshot",
                        "description": "Diagnostic query to perform",
                    },
                },
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict:
        if action in ("joint_health", "snapshot"):
            return self._analyzer.snapshot()
        if action == "list_alerts":
            snapshot = self._analyzer.snapshot()
            return {
                "state": snapshot["state"],
                "health_level": snapshot["health_level"],
                "alerts": snapshot["alerts"],
                "last_update_ago_ms": snapshot["last_update_ago_ms"],
            }
        if action == "reset_baseline":
            snapshot = self._analyzer.reset_baseline()
            return {
                "state": snapshot["state"],
                "health_level": snapshot["health_level"],
                "baseline_reset": True,
                "last_update_ago_ms": snapshot["last_update_ago_ms"],
            }
        if action == "start":
            return {"state": "running"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            snapshot = self._analyzer.snapshot()
            return {
                "state": snapshot["state"],
                "health_level": snapshot["health_level"],
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
                "last_update_ago_ms": snapshot["last_update_ago_ms"],
            }
        return {"error": f"Unknown action: {action}"}
