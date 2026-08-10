from __future__ import annotations

import uuid

TEST_SECRET = "test-only-teleop-ticket-secret-32-bytes-minimum"
TEST_DRIVER_TOKEN = "test-driver-token-private-0001"


class FakeClock:
    def __init__(self, value: float = 100.0):
        self.value = value

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> None:
        self.value += seconds


def new_session(*, epoch: int = 1, fence: str | None = None) -> dict:
    return {
        "session_id": str(uuid.uuid4()),
        "epoch": epoch,
        "fence": fence or ("f" * 32),
    }


def identity(runtime, session: dict) -> dict:
    return {
        "boot_id": runtime.boot_id,
        "session_id": session["session_id"],
        "epoch": session["epoch"],
        "fence": session["fence"],
    }


def valid_frame(runtime, session: dict, *, sequence: int = 1, clutch_sequence: int = 1,
                deadman: bool = True, tracking: bool = True) -> dict:
    pose = {"position": [0.0, 1.0, 0.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    return {
        "schema_version": 1,
        **identity(runtime, session),
        "sequence": sequence,
        "client_monotonic_ns": sequence * 1_000_000,
        "mode": "shadow",
        "deadman": deadman,
        "clutch_sequence": clutch_sequence,
        "tracking": {
            "head": tracking,
            "left_controller": tracking,
            "right_controller": tracking,
        },
        "head": dict(pose) if tracking else None,
        "left_controller": dict(pose) if tracking else None,
        "right_controller": dict(pose) if tracking else None,
        "controllers": {
            "left": {"axes": [0.0, 0.0], "buttons": [0.0, 1.0]},
            "right": {"axes": [0.0, 0.0], "buttons": [0.0, 1.0]},
        },
        "base_twist": {"linear": [0.0, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]},
    }


def rtc_wire_frame(runtime, session: dict, **kwargs) -> dict:
    frame = valid_frame(runtime, session, **kwargs)
    for field in ("boot_id", "session_id", "epoch", "fence"):
        frame.pop(field)
    return frame


def contains_value(value, needle: str) -> bool:
    if isinstance(value, dict):
        return any(key == needle or contains_value(item, needle) for key, item in value.items())
    if isinstance(value, list):
        return any(contains_value(item, needle) for item in value)
    return value == needle
