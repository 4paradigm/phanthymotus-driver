"""Read-only Go1 power estimates from the shared HighState snapshot.

Battery power uses BMS cell voltages and current. Joint power is estimated
mechanical shaft power (tauEst * dq), not motor electrical input power.
"""

from __future__ import annotations

import json
import math
import threading
import time

from go1_sdk_client import JOINT_NAMES

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from std_msgs.msg import String
    _HAS_ROS2 = True
    _QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST, depth=1,
                      durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False


_STALE_S = 2.5
_MAX_GAP_S = 2.5
_CELL_COUNT = 10  # Go1 SDK BmsState.cell_vol[10]
_MIN_RUNTIME_OBSERVATION_S = 60.0
_MIN_RUNTIME_SOC_DROP = 3.0


def _sample_time(snap: dict, now: float) -> float | None:
    """Return a recent source timestamp; a read-time timestamp is not enough."""
    if not snap.get("fresh"):
        return None
    try:
        stamp = float(snap["sample_monotonic_s"])
    except (KeyError, TypeError, ValueError):
        return None
    if not math.isfinite(stamp) or stamp <= 0 or not -0.1 <= now - stamp <= _STALE_S:
        return None
    return stamp


def _battery_measurement(snap: dict) -> tuple[dict | None, str | None]:
    bat = snap.get("battery")
    if not isinstance(bat, dict):
        return None, "bms_unavailable"
    try:
        cells = [float(v) for v in bat["cell_voltage_mv"]]
        current_ma = float(bat["current_ma"])
        status = int(bat["status_code"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return None, "invalid_bms_fields"
    if (len(cells) != _CELL_COUNT or
            any(not math.isfinite(v) or not 2000 <= v <= 5000 for v in cells) or
            not math.isfinite(current_ma) or abs(current_ma) > 200000):
        return None, "invalid_bms_values"
    direction = "discharge" if status == 1 else "charge" if status in (2, 3, 4) else "unknown"
    try:
        soc = float(bat["soc_percent"])
        soc = soc if math.isfinite(soc) and 0 <= soc <= 100 else None
    except (KeyError, TypeError, ValueError, OverflowError):
        soc = None
    voltage_v = sum(cells) / 1000.0
    power_w = voltage_v * abs(current_ma) / 1000.0
    return {"voltage_v": round(voltage_v, 3),
            "current_a": round(current_ma / 1000.0, 3),
            "power_w": round(power_w, 3),
            "soc_percent": soc,
            "direction": direction,
            "signed_power_w": round(power_w if direction == "discharge" else -power_w, 3)
            if direction != "unknown" else None}, None


class BatteryPowerPlugin:
    """Integrate only consecutive, fresh BMS samples while the bundle runs."""

    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = f"/{namespace}/state/battery_power"
        self._node = None
        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._thread = None
        self._last = None  # (source monotonic timestamp, unrounded W, direction)
        self._discharged_wh = 0.0
        self._charged_wh = 0.0
        self._covered_s = 0.0
        self._runtime_start = None  # (first continuous discharge timestamp, SOC%)
        self._runtime_last_soc = None
        self._latest = self._unavailable("no_sample_yet")
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_battery_power")
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(1.0, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[battery_power] ROS2 unavailable, MCP polling only: {e}", flush=True)
                self._node = None

    def _unavailable(self, reason: str) -> dict:
        return {"timestamp_ms": int(time.time() * 1000), "fresh": False,
                "available": False, "reason": reason,
                "discharged_since_start_wh": round(self._discharged_wh, 4),
                "charged_since_start_wh": round(self._charged_wh, 4),
                "covered_duration_s": round(self._covered_s, 2),
                "energy_scope": "driver_process_since_start",
                "remaining_runtime_minutes": None,
                "runtime_estimate_reason": reason}

    def _sample(self, snap: dict, now: float | None = None) -> dict:
        """One deterministic step, also used by the background sampler."""
        now = time.monotonic() if now is None else now
        with self._lock:
            stamp = _sample_time(snap, now)
            if stamp is None:
                self._last = None
                self._runtime_start = None
                self._runtime_last_soc = None
                self._latest = self._unavailable("stale_or_missing_sample")
                return dict(self._latest)
            measurement, reason = _battery_measurement(snap)
            if measurement is None:
                self._last = None
                self._runtime_start = None
                self._runtime_last_soc = None
                self._latest = self._unavailable(reason)
                return dict(self._latest)
            if self._last is not None and stamp == self._last[0]:
                return dict(self._latest)
            power = measurement["power_w"]
            direction = measurement["direction"]
            if self._last is not None and not 0 < stamp - self._last[0] <= _MAX_GAP_S:
                self._runtime_start = None
                self._runtime_last_soc = None
                if stamp < self._last[0]:
                    self._last = None
            if self._last is not None:
                previous_stamp, previous_power, previous_direction = self._last
                dt = stamp - previous_stamp
                if 0 < dt <= _MAX_GAP_S and direction == previous_direction:
                    energy_wh = (previous_power + power) * 0.5 * dt / 3600.0
                    if direction == "discharge":
                        self._discharged_wh += energy_wh
                        self._covered_s += dt
                    elif direction == "charge":
                        self._charged_wh += energy_wh
                        self._covered_s += dt
            if self._last is None or stamp > self._last[0]:
                self._last = (stamp, power, direction)
            soc = measurement["soc_percent"]
            runtime_minutes = None
            if direction != "discharge":
                self._runtime_start = None
                self._runtime_last_soc = None
                runtime_reason = "not_discharging"
            elif soc is None:
                self._runtime_start = None
                self._runtime_last_soc = None
                runtime_reason = "soc_unavailable"
            elif soc == 0:
                self._runtime_start = None
                self._runtime_last_soc = None
                runtime_minutes = 0.0
                runtime_reason = None
            else:
                if self._runtime_start is None or (self._runtime_last_soc is not None and soc > self._runtime_last_soc):
                    self._runtime_start = (stamp, soc)
                self._runtime_last_soc = soc
                start_stamp, start_soc = self._runtime_start
                observed_s = stamp - start_stamp
                drop = start_soc - soc
                if observed_s >= _MIN_RUNTIME_OBSERVATION_S and drop >= _MIN_RUNTIME_SOC_DROP:
                    runtime_minutes = round(soc * observed_s / drop / 60.0, 1)
                    runtime_reason = None
                else:
                    runtime_reason = "insufficient_soc_history"
            self._latest = {"timestamp_ms": int(time.time() * 1000),
                            "sample_monotonic_s": stamp,
                            "control_level": snap.get("control_level", "HIGHLEVEL"),
                            "fresh": True, "available": True,
                            **measurement,
                            "discharged_since_start_wh": round(self._discharged_wh, 4),
                            "charged_since_start_wh": round(self._charged_wh, 4),
                            "covered_duration_s": round(self._covered_s, 2),
                            "energy_scope": "driver_process_since_start",
                            "remaining_runtime_minutes": runtime_minutes,
                            "runtime_estimate_reason": runtime_reason,
                            "runtime_estimate_method": "observed_soc_decline",
                            "measurement_kind": "battery_electrical_estimate"}
            return dict(self._latest)

    def _run(self):
        while not self._shutdown.is_set():
            try:
                self._sample(self._client.snapshot())
            except Exception as e:
                with self._lock:
                    self._last = None
                    self._runtime_start = None
                    self._runtime_last_soc = None
                    self._latest = self._unavailable(f"snapshot_error: {e}")
            self._shutdown.wait(1.0)

    def _tick(self):
        try:
            with self._lock:
                data = dict(self._latest)
            msg = String()
            msg.data = json.dumps(data)
            self._pub.publish(msg)
        except Exception as e:
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        return {"name": "battery_power", "type": "sensor", "multiInstance": False,
                "description": "Go1 BMS power, discharge/charge Wh and rough remaining runtime from observed SOC decline; read-only",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._shutdown.clear()
            self._thread = threading.Thread(target=self._run, daemon=True, name="go1_battery_power")
            self._thread.start()

    def stop(self):
        self._shutdown.set()
        if self._thread is not None:
            self._thread.join(timeout=2.5)

    def dispatch(self, action, args):
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in ("info", "read", "get", "battery_power"):
            with self._lock:
                data = dict(self._latest)
            return {"state": "running", "data": data,
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_battery_power(plugin_config, namespace, executor, client):
    return BatteryPowerPlugin(plugin_config, namespace, executor, client)


def _build_joint_power(snap: dict, now: float | None = None) -> dict:
    now = time.monotonic() if now is None else now
    out = {"timestamp_ms": int(time.time() * 1000),
           "control_level": snap.get("control_level", "HIGHLEVEL"),
           "fresh": False, "available": False,
           "measurement_kind": "estimated_joint_mechanical_power",
           "electrical_power_available": False}
    if _sample_time(snap, now) is None:
        out["reason"] = "stale_or_missing_sample"
        return out
    joints = snap.get("joints")
    if not isinstance(joints, list) or len(joints) < 12:
        out["reason"] = "incomplete_joint_state"
        return out
    result = []
    try:
        for i, joint in enumerate(joints[:12]):
            torque = float(joint["tau"])
            speed = float(joint["dq"])
            if not math.isfinite(torque) or not math.isfinite(speed):
                raise ValueError("non-finite joint data")
            result.append({"idx": i, "name": JOINT_NAMES[i],
                           "estimated_torque_nm": torque, "angular_speed_rad_s": speed,
                           "mechanical_power_w": round(torque * speed, 3)})
    except (KeyError, TypeError, ValueError):
        out["reason"] = "invalid_joint_state"
        return out
    powers = [joint["mechanical_power_w"] for joint in result]
    out.update({"fresh": True, "available": True, "joints": result,
                "positive_mechanical_power_w": round(sum(max(p, 0.0) for p in powers), 3),
                "negative_mechanical_power_w": round(sum(min(p, 0.0) for p in powers), 3)})
    return out


class JointPowerPlugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        self._topic = f"/{namespace}/state/joint_power"
        self._node = None
        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node("go1_joint_power")
                self._pub = self._node.create_publisher(String, self._topic, _QOS)
                self._node.create_timer(0.1, self._tick)
                executor.add_node(self._node)
            except Exception as e:
                print(f"[joint_power] ROS2 unavailable, MCP polling only: {e}", flush=True)
                self._node = None

    def _tick(self):
        try:
            msg = String()
            msg.data = json.dumps(_build_joint_power(self._client.snapshot()))
            self._pub.publish(msg)
        except Exception as e:
            self._node.get_logger().error(f"publish {self._topic} error: {e}")

    def get_tool(self):
        return {"name": "joint_power", "type": "sensor", "multiInstance": False,
                "description": "Go1 12 joints' estimated mechanical power (tauEst*dq), not electrical power; read-only",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}

    def start(self): pass
    def stop(self): pass

    def dispatch(self, action, args):
        if action == "start": return {"state": "running"}
        if action == "stop": return {"state": "idle"}
        if action in ("info", "read", "get", "joint_power"):
            return {"state": "running", "data": _build_joint_power(self._client.snapshot()),
                    "topic_out": ([{"topic": self._topic, "format": "data/json"}] if self._node else [])}
        return None


def make_joint_power(plugin_config, namespace, executor, client):
    return JointPowerPlugin(plugin_config, namespace, executor, client)
