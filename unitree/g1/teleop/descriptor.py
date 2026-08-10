"""Pure, DDS-free descriptors for the G1 teleoperation tools."""

from __future__ import annotations

import copy
import hashlib
import json
from typing import Any

PROFILE_ID = "unitree_g1_23_dual_arm_controller_v1"
SHADOW_PROTOCOL = "motus.teleop.shadow.v1"
LIVE_PROTOCOL = "motus.teleop.live.v1"
RTC_FRAME_PROTOCOL = "motus.teleop.rtc-frame.v1"
RTC_CONTROL_PROTOCOL = "motus.teleop.rtc-control.v1"
SIGNALING_PROTOCOL = "motus.teleop.webrtc-offer-answer.v1"
SIGNALING_AUDIENCE = "motus-teleop-rtc"
PREFLIGHT_SCHEMA = "motus.teleop.g1-preflight.v1"
RECORDING_DISPATCH = "motus.teleop.dispatch.recording.v1"
HARDWARE_DISPATCH = "motus.teleop.dispatch.hardware.v1"
MODES = frozenset({"shadow", "live"})

CAPABILITIES = {
    "profile_id": PROFILE_ID,
    "input_bindings": {
        "head": {"required": True, "role": "reference"},
        "left_controller": {"required": True, "role": "left_end_effector"},
        "right_controller": {"required": True, "role": "right_end_effector"},
    },
    "outputs": {
        "dual_arm": {"enabled": True, "joint_count": 10},
        "base": {"enabled": False},
        "hands": {"enabled": False},
    },
    "effectors": ["dual_arm"],
}


def canonical_json(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def protocol_for_mode(mode: str) -> str:
    _require_mode(mode)
    return LIVE_PROTOCOL if mode == "live" else SHADOW_PROTOCOL


def dispatch_for_mode(mode: str) -> str:
    _require_mode(mode)
    return HARDWARE_DISPATCH if mode == "live" else RECORDING_DISPATCH


def capability_binding(mode: str) -> dict:
    _require_mode(mode)
    return {
        "protocol": protocol_for_mode(mode),
        "mode": mode,
        "profile_id": PROFILE_ID,
        "capabilities": copy.deepcopy(CAPABILITIES),
        "dispatch_contract": dispatch_for_mode(mode),
        "signaling": {
            "protocol": SIGNALING_PROTOCOL,
            "path": "/offer",
            "access": "authenticated-core-proxy-only",
            "audience": SIGNALING_AUDIENCE,
        },
    }


def capability_digest(mode: str) -> str:
    return hashlib.sha256(canonical_json(capability_binding(mode))).hexdigest()


def tool_definitions(
    *,
    mode: str,
    driver_id: str = "unitree-g1",
    driver_name: str = "Unitree G1 Bundle",
    robot_id: str | None = None,
    signaling_enabled: bool = True,
) -> list[dict]:
    """Build descriptors without importing ROS, DDS, aiortc, NumPy or an IK solver."""

    _require_mode(mode)
    if not signaling_enabled:
        raise ValueError("G1 teleoperation cannot be advertised without authenticated RTC signaling")
    effective_robot_id = robot_id or driver_id
    identity = {
        "boot_id": {"type": "string", "format": "uuid"},
        "session_id": {"type": "string", "format": "uuid"},
        "epoch": {"type": "integer", "minimum": 1},
        "fence": {"type": "string", "minLength": 24},
    }
    prepare_action = "prepare_live" if mode == "live" else "prepare_shadow"
    actions = {
        "stop": {"params": [], "description": "Release the teleoperation lifecycle safely"},
        prepare_action: {
            "params": ["session_id", "epoch", "fence"],
            "description": (
                "Prepare explicitly enabled G1 arm_sdk hardware control"
                if mode == "live"
                else "Prepare a zero-output G1 dual-arm Shadow session"
            ),
        },
        "heartbeat": {"params": list(identity), "description": "Renew the Core-owned lease"},
        "pause": {"params": list(identity), "description": "Pause and confirm safe output"},
        "release": {"params": list(identity), "description": "Release authority and confirm safe output"},
        "soft_stop": {"params": list(identity), "description": "Latch safe output until a newer prepare"},
        "status": {"params": [], "description": "Read bounded session and output diagnostics"},
    }
    x_teleop = {
        **capability_binding(mode),
        "driver_id": driver_id,
        "driver_name": driver_name,
        "robot_id": effective_robot_id,
        "actuation_enabled": mode == "live",
        "capability_digest": capability_digest(mode),
    }
    session = {
        "name": "teleop_session",
        "type": "actuator",
        "multiInstance": False,
        "description": (
            "Unitree G1_23 dual-arm controller teleoperation. Base and hands are disabled; "
            "only the root G1 Driver may own the arm_sdk publisher."
        ),
        "annotations": {"destructiveHint": mode == "live", "idempotentHint": False},
        "inputSchema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "action": {"type": "string", "enum": list(actions)},
                **identity,
            },
            "required": ["action"],
            "x-action-params": actions,
        },
        "x-teleop": copy.deepcopy(x_teleop),
    }
    state = {
        "name": "teleop_state",
        "type": "resource",
        "multiInstance": False,
        "readOnly": True,
        "description": "Read-only G1 teleoperation state, latency, target and measured joints",
        "annotations": {
            "readOnlyHint": True,
            "destructiveHint": False,
            "idempotentHint": True,
        },
        "inputSchema": {"type": "object", "properties": {}, "additionalProperties": False},
        "x-teleop": copy.deepcopy(x_teleop),
    }
    return [session, state]


def _require_mode(mode: str) -> None:
    if mode not in MODES:
        raise ValueError(f"mode must be one of {sorted(MODES)}")


__all__ = [
    "CAPABILITIES",
    "HARDWARE_DISPATCH",
    "LIVE_PROTOCOL",
    "MODES",
    "PREFLIGHT_SCHEMA",
    "PROFILE_ID",
    "RECORDING_DISPATCH",
    "SHADOW_PROTOCOL",
    "SIGNALING_AUDIENCE",
    "capability_binding",
    "capability_digest",
    "canonical_json",
    "dispatch_for_mode",
    "protocol_for_mode",
    "tool_definitions",
]
