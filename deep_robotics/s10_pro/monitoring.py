"""Honest, read-only S10 monitor summaries; no robot control dependencies.

Raw observations remain in the card payload. These summaries deliberately keep
unknown units, event meanings and optional hardware availability explicit.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from collections import OrderedDict


MOTION_CAPABILITY = {
    "supported": False,
    "status": "requires_vendor_contract",
    "description": "Physical motion execution remains locked pending verification of this robot's control interface, authority and stop contract.",
}
DEVICE_ACTIONS = ("lights", "mode", "sleep", "wake", "auto_sleep", "sleep_status")
DEVICE_REASON = "This robot's S10 device protocol, version and completion feedback have not been verified. No device command is sent."
OPTIONAL_CAPABILITIES = {
    "lidar_rear": "An independent rear LiDAR source has not been verified; the main cloud is not reused.",
    "camera_front": "No independently identified, browser-compatible front video source has been verified.",
    "camera_rear": "No independently identified, browser-compatible rear video source has been verified.",
    "gps": "Passive samples on 2026-10-23 reported fixquality=0 and zero satellites. Valid positioning and hardware availability remain unverified.",
    "charge_status": "Passive samples on 2026-10-23 reported state=0 and error_code=0. The charging-state mapping remains unverified; no charging state is inferred.",
    "navigation": "The local navigation, localization, goal and cancellation contracts have not been verified.",
}


def _object(value):
    return value if isinstance(value, dict) else {}


def _number(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def motion_summary(raw, source):
    data = _object(_object(raw).get("data"))
    state = _object(data.get("motion_state")).get("state")
    gait = _object(data.get("gait_state")).get("gait")
    return {
        "motion_state": state, "gait": gait,
        "motion_state_meaning": "unknown", "gait_meaning": "unknown",
        "velocity": {"x": _number(data.get("vel_x")), "y": _number(data.get("vel_y")),
                     "yaw": _number(data.get("vel_yaw"))},
        "height": _number(data.get("height")),
        "units": {"velocity_x": "m/s", "velocity_y": "m/s", "velocity_yaw": "rad/s", "height": None},
        "fresh": source.get("fresh", False), "age_s": source.get("age_s"),
        "control_ownership": "unknown", "usage_mode": "unknown",
        "note": "Measured motion feedback does not establish control ownership or permission to move.",
    }


def estop_summary(raw, source):
    value = _object(raw).get("value")
    value = value if type(value) is int else None
    # These values were observed while the user triggered/released HES on this
    # robot. Keep that empirical indication distinct from a vendor contract.
    observed = {0: "observed_released", 1: "observed_triggered"}.get(value, "unknown")
    return {
        "value": value, "indication": observed if source.get("fresh") else "unknown",
        "last_observed_indication": observed,
        "mapping_basis": "Observed on this robot on 2026-10-23: triggered=1, released=0.",
        "manufacturer_mapping_verified": False,
        "fresh": source.get("fresh", False), "age_s": source.get("age_s"),
        "safe_to_move": None,
        "note": "A fresh released indication alone does not authorize physical execution.",
    }


def joint_label(value):
    if isinstance(value, (list, tuple)):
        try:
            return bytes(value).split(b"\0", 1)[0].decode("ascii")
        except (ValueError, TypeError, UnicodeDecodeError):
            return None
    return value if isinstance(value, str) else None


def device_summary(battery_raw, joints_raw, resource_values=None, stale_after_s=5.):
    batteries = _object(battery_raw).get("data")
    batteries = batteries if isinstance(batteries, list) else []
    joints = _object(_object(joints_raw).get("data")).get("joints_data")
    joints = joints if isinstance(joints, list) else []
    temperatures = [{"joint": joint_label(item.get("name")),
                     "motor_temperature": _number(item.get("motion_temp")),
                     "driver_temperature": _number(item.get("driver_temp"))}
                    for item in joints if isinstance(item, dict)]
    resources = {}
    for key, (raw, source, _binary) in (resource_values or {}).items():
        raw = _object(raw)
        items = raw.get("items")
        items = items if isinstance(items, list) else []
        observed = []
        for item in items:
            if not isinstance(item, dict) or not isinstance(item.get("data"), dict):
                continue
            receipt_age, envelope_age = _number(item.get("age_s")), _number(source.get("age_s"))
            age = receipt_age + envelope_age if receipt_age is not None and envelope_age is not None else None
            observed.append({**copy.deepcopy(item), "age_s": age,
                             "fresh": bool(source.get("fresh") and age is not None and 0 <= age <= stale_after_s)})
        resources[key] = {"status": "live" if source.get("fresh") else "stale" if source.get("received") else "unavailable",
                          "fresh": source.get("fresh", False), "age_s": source.get("age_s"),
                          "items": observed, "truncated": raw.get("truncated", False),
                          "latest": copy.deepcopy(raw.get("latest")),
                          "scope": "Bounded observations keyed by SoC, filesystem or interface; each item has its own age."}
    return {
        "battery_count": len(batteries) if battery_raw is not None else None,
        "batteries": [{"index": index, "battery_level": item.get("battery_level"),
                       "voltage": item.get("voltage"), "current": item.get("current"),
                       "remaining_capacity": item.get("remaining_capacity"),
                       "nominal_capacity": item.get("nominal_capacity"),
                       "temperatures": copy.deepcopy(item.get("battery_temperature"))}
                      for index, item in enumerate(batteries) if isinstance(item, dict)],
        "joint_count": len(joints) if joints_raw is not None else None,
        "joint_temperatures": temperatures,
        "units": {"battery_level": None, "voltage": None, "current": None,
                  "capacity": None, "battery_temperature": None, "joint_temperature": None},
        "compute_resources": resources or {"status": "unverified", "reason": "No verified resource-status source is connected."},
        "compute_units": {"cpu_temperature": "degC", "cpu_utilization": "%", "cpu_frequency": "kHz",
                          "memory": "KB", "filesystem_space": "bytes", "filesystem_inodes": "count",
                          "network_throughput": "bytes/s", "network_counters": "count"},
        "peripherals": {"status": "unverified", "reason": "No verified peripheral-status source is connected."},
        "charging": {"status": "unknown", "reason": "Battery current and MOS state are not a verified charging-state mapping."},
    }


def capabilities_snapshot():
    supported = {
        "state": "Battery, motion feedback and emergency-stop observations with explicit source age.",
        "motion_info": "Read-only /MOTION_INFO state, gait, velocity and height fields.",
        "device_status": "Observed battery count, raw battery/joint temperatures and per-source computing-resource observations.",
        "joints": "Sixteen joint feedback records have been observed; no unverified robot skeleton is used.",
        "imu": "Standard ROS IMU observations; original coordinate frames are preserved.",
        "odometry": "Read-only /ODOM observations; original coordinate frames are preserved.",
        "lidar": "Main LiDAR cloud conversion is connected; canvas rendering requires separate visual acceptance.",
        "hard_estop": "Read-only HES observations; the manufacturer mapping remains unverified.",
        "errors": "Fresh active-fault aggregate snapshots plus bounded fault and exception event observations; no-message does not mean fault-free.",
        "motion_events": "Read-only view of existing application motion events; no additional ACP notifier.",
        "ros_graph": "Read-only ROS discovery; publishers do not establish control ownership.",
    }
    return {
        "model": "DEEPRobotics S10 Pro", "read_only": True,
        "scope": "Driver integration status; this is not a claim that every card passed live visual acceptance.",
        "available": {key: {"status": "read_only", "description": description}
                      for key, description in supported.items()},
        "conditional": {key: {"status": "unverified", "registered": False, "reason": reason}
                        for key, reason in OPTIONAL_CAPABILITIES.items()},
        "motion": copy.deepcopy(MOTION_CAPABILITY),
        "device": {"status": "unavailable", "reason": DEVICE_REASON,
                   "actions": {name: {"available": False, "reason": DEVICE_REASON} for name in DEVICE_ACTIONS}},
        "charging_action": {"status": "excluded", "registered": False, "reason": "Autonomous charging commands are excluded from this driver."},
        "control_ownership": "unknown", "manufacturer_hes_mapping": "unverified",
        "hardware_validated": False,
    }


class ErrorObservations:
    """Bounded observations using the message definitions read from this robot.

    FaultEventArray is a current active snapshot. FaultEvent and Exception are
    event observations; their latest-value IPC cannot provide a complete log.
    Only a fresh well-formed aggregate can establish the reported current list.
    """
    def __init__(self, limit=64):
        self.limit = limit
        self._seen_sequences = {}
        self._history = OrderedDict()

    @staticmethod
    def _fault_key(fault):
        return json.dumps([fault.get("code"), fault.get("resources"), fault.get("source")],
                          sort_keys=True, allow_nan=False, separators=(",", ":"))

    @staticmethod
    def _classification(source, raw):
        if source == "fault_event":
            kind = raw.get("type")
            return {1: "fault_started", 2: "fault_recovered"}.get(kind, "unclassified_fault_event") if type(kind) is int else "unclassified_fault_event"
        if source == "exception" and type(raw.get("is_abnormal")) is bool:
            return "exception_abnormal" if raw["is_abnormal"] else "exception_recovered"
        if source == "fault_aggregator" and isinstance(raw.get("active_faults"), list):
            return "active_fault_snapshot"
        return "unclassified_observation"

    def update(self, values):
        for source, (raw, info, _binary) in values.items():
            if not isinstance(raw, dict) or not info.get("received"):
                continue
            sequence = info.get("sequence")
            if self._seen_sequences.get(source) == sequence:
                continue
            self._seen_sequences[source] = sequence
            ignored = {"header", "timestamp"} if source == "fault_aggregator" else {"header"}
            content = {key: value for key, value in raw.items() if key not in ignored}
            encoded = json.dumps(content, sort_keys=True, allow_nan=False, separators=(",", ":"))
            fingerprint = hashlib.sha256(encoded.encode()).hexdigest()
            if fingerprint in self._history:
                record = self._history[fingerprint]
                if source not in record["sources"]:
                    record["sources"].append(source)
                record["last_received_at_ms"] = info.get("received_at_ms")
                record["observations"] += 1
                self._history.move_to_end(fingerprint)
            else:
                self._history[fingerprint] = {
                    "id": fingerprint[:16], "sources": [source],
                    "first_received_at_ms": info.get("received_at_ms"),
                    "last_received_at_ms": info.get("received_at_ms"),
                    "observations": 1, "data": copy.deepcopy(raw),
                    "classification": self._classification(source, raw),
                }
            while len(self._history) > self.limit:
                self._history.popitem(last=False)
        history = copy.deepcopy(list(reversed(self._history.values())))
        snapshot, info, _binary = values.get("fault_aggregator", (None, {}, None))
        active = _object(snapshot).get("active_faults")
        valid = isinstance(active, list) and all(isinstance(item, dict) for item in active)
        unique = list({self._fault_key(item): item for item in active}.values()) if valid else None
        current = copy.deepcopy(unique) if valid and info.get("fresh") else None
        return {
            "current_fault_state": ("active_faults_reported" if current else "no_active_faults_reported") if current is not None else "unknown",
            "current_faults": current, "active_fault_count": len(current) if current is not None else None,
            "current_fault_source": "fault_aggregator" if current is not None else None,
            "last_snapshot_faults": copy.deepcopy(unique), "snapshot_fresh": info.get("fresh", False),
            "recovered_faults": [item for item in history if item["classification"] in ("fault_recovered", "exception_recovered")],
            "history": history,
            "history_limit": self.limit, "history_scope": "Observations retained in this driver process; not a complete robot fault log.",
            "semantics_verified": True,
            "semantics_basis": "FaultEvent, FaultEventArray and Exception message definitions read from this robot on 2026-10-23.",
            "note": "Only a fresh aggregate reports the current active list. Fault START/STOP and exception abnormal/recovery are observed events; latest-value IPC may miss intervening events. No messages do not mean fault-free.",
        }
