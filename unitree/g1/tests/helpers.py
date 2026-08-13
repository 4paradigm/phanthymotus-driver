from __future__ import annotations

import base64
import time
import uuid

import numpy as np
from teleop.descriptor import PREFLIGHT_SCHEMA


class FakeLowStateReader:
    def __init__(self, *, mode_machine: int = 4):
        self.mode_machine = mode_machine
        self.q = np.linspace(-0.2, 0.2, 10)
        self.dq = np.zeros(10)
        self.reads = 0

    def read_arm_state(self):
        self.reads += 1
        all_q = np.zeros(35)
        all_q[[15, 16, 17, 18, 19, 22, 23, 24, 25, 26]] = self.q
        return {
            "joint_positions": self.q.copy(),
            "joint_velocities": self.dq.copy(),
            "all_joint_positions": all_q,
            "mode_machine": self.mode_machine,
            "sample_monotonic": time.monotonic(),
        }


class FakeIkSolver:
    def __init__(self):
        self.calls = []
        self.resets = []

    def reset(self, current_q):
        self.resets.append(np.asarray(current_q, dtype=float).copy())

    def solve(self, left, right, current_q, current_dq):
        self.calls.append((left.copy(), right.copy(), current_q.copy(), current_dq.copy()))
        return current_q + 0.1, np.zeros(10)

    def ready(self):
        return True

    def snapshot(self):
        return {"ready": True, "warmup_ms": 1.0, "history_depth": 0}

    def current_targets(self, current_q):
        return np.eye(4), np.eye(4)


class FakeIkDiagnostic:
    def dispatch(self, action, args):
        return {
            "state": "ready",
            "action": action,
            "arguments": dict(args),
            "hardware_output": False,
        }

    def status(self):
        return {
            "state": "ready",
            "hardware_output": False,
            "publisher_created": False,
        }


def startup_preflight(runtime, *, warmup_ms: float = 1.0) -> dict:
    status = runtime.status()
    return {
        "schema": PREFLIGHT_SCHEMA,
        "ready": False,
        "stage": "service_startup",
        "code": None,
        "message": None,
        "mode": runtime.mode,
        "profile_id": runtime.profile_id,
        "hardware_output": runtime.actuation_enabled,
        "publisher_created": runtime.actuation_enabled,
        "low_state": {
            "ready": True,
            "mode_machine": 4,
            "required_mode_machine": 4,
            "sample_age_ms": 1.0,
            "arm_joint_count": 10,
            "motor_joint_count": 35,
        },
        "ik": {
            "ready": True,
            "warmup_ms": warmup_ms,
            "model": "g1_body23.urdf",
            "solver": "pinocchio-casadi-ipopt",
        },
        "identity": {
            "driver_id": runtime.driver_id,
            "robot_id": runtime.robot_id,
            "capability_digest": runtime.capability_digest,
        },
        "dispatch": {
            "ready": status["dispatch"]["ready"],
            "kind": status["dispatch"]["kind"],
            "state": status["dispatch"]["state"],
            "stop_acknowledged": status["dispatch"]["stop_acknowledged"],
            "fault_code": status["dispatch"]["fault_code"],
        },
    }


def capture_config(*, state_file=None) -> dict:
    """In-memory Capture config for service tests without opening a listener."""

    return {
        "public_wss_url": "wss://127.0.0.1:15702/ws/teleop-capture",
        "ca_certificate_base64": base64.b64encode(
            b"-----BEGIN CERTIFICATE-----\ntest\n-----END CERTIFICATE-----\n"
        ).decode("ascii"),
        "state_file": state_file,
        "presence_interval_ms": 1000,
        "presence_timeout_ms": 5000,
    }


def session():
    return {
        "session_id": str(uuid.uuid4()),
        "epoch": 1,
        "fence": "f" * 32,
    }


def frame(runtime, identity, *, sequence: int, clutch_sequence: int, deadman: bool):
    return {
        "schema_version": 1,
        "boot_id": runtime.boot_id,
        "session_id": identity["session_id"],
        "epoch": identity["epoch"],
        "fence": identity["fence"],
        "sequence": sequence,
        "client_monotonic_ns": 123,
        "mode": runtime.mode,
        "deadman": deadman,
        "clutch_sequence": clutch_sequence,
        "tracking": {
            "head": True,
            "left_controller": True,
            "right_controller": True,
        },
        "head": {
            "position": [0.0, 1.6, 0.0],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        },
        "left_controller": {
            "position": [-0.2, 1.2, -0.4],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        },
        "right_controller": {
            "position": [0.2, 1.2, -0.4],
            "orientation": [0.0, 0.0, 0.0, 1.0],
        },
        "controllers": {
            "left": {"axes": [], "buttons": []},
            "right": {"axes": [], "buttons": []},
        },
    }


def rtc_frame(runtime, identity, *, sequence: int, clutch_sequence: int, deadman: bool):
    """Build the public RTC frame; session authority is peer-bound, not sent by Quest."""

    value = frame(
        runtime,
        identity,
        sequence=sequence,
        clutch_sequence=clutch_sequence,
        deadman=deadman,
    )
    for private_key in ("boot_id", "session_id", "epoch", "fence"):
        value.pop(private_key)
    return value
