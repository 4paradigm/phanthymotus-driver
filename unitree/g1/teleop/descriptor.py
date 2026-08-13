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
CAPTURE_PROTOCOL = "motus.teleop.capture.v1"
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
            "capture_protocol": CAPTURE_PROTOCOL,
            "path": "/ws/teleop-capture",
            "access": "paired-capture-credential-only",
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
    instance_id = {
        "instance_id": {
            "type": "string",
            "maxLength": 128,
            "description": "Optional stock Core card instance identifier",
        }
    }
    card_lifecycle_actions = {
        "start": {
            "params": [],
            "description": "Start the stock Core card lifecycle",
        },
        "info": {
            "params": [],
            "description": "Read stock Core card lifecycle information",
        },
        "stop": {
            "params": [],
            "description": "Stop the stock Core card lifecycle",
        },
    }
    actions = {
        **copy.deepcopy(card_lifecycle_actions),
        "pair_headset": {
            "params": [],
            "description": "创建一次性头显配对信息并返回 WSS 地址与公共 CA",
        },
        "revoke_headset": {
            "params": [],
            "description": "撤销已配对头显及本地会话并安全停止",
        },
        "pause": {"params": [], "description": "暂停会话并确认安全停止"},
        "release": {"params": [], "description": "释放本地会话并确认安全停止"},
        "emergency_stop": {
            "params": [],
            "description": "紧急撤销本地控制权并确认安全停止",
        },
        "status": {"params": [], "description": "读取会话、Capture、RTC 与输出状态"},
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
            "Unitree G1_23 双臂遥操作。会话、租约和一次性 RTC 授权由 Driver "
            "本地持有；底盘和手部关闭，只有根 G1 Driver 可持有 arm_sdk publisher。"
        ),
        "annotations": {"destructiveHint": mode == "live", "idempotentHint": False},
        "inputSchema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "action": {"type": "string", "enum": list(actions)},
                **instance_id,
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
        "inputSchema": {
            "type": "object",
            "properties": {
                "action": {
                    "type": "string",
                    "enum": [*card_lifecycle_actions, "status"],
                },
                **copy.deepcopy(instance_id),
            },
            "x-action-params": {
                **copy.deepcopy(card_lifecycle_actions),
                "status": {"params": [], "description": "Read teleoperation state"},
            },
            "additionalProperties": False,
        },
        "x-teleop": copy.deepcopy(x_teleop),
    }
    ik_actions = {
        **copy.deepcopy(card_lifecycle_actions),
        "solve": {
            "params": ["frame_json"],
            "description": (
                "Parse one strict OpenXR pose-frame JSON string and run the shared "
                "G1_23 IK solver without hardware output"
            ),
        },
        "self_test": {
            "params": [],
            "description": (
                "Solve the currently measured end-effector targets without hardware output"
            ),
        },
        "reset": {
            "params": [],
            "description": "Reset the shared IK seed to the current measured arm posture",
        },
        "status": {
            "params": [],
            "description": "Read bounded IK diagnostic state",
        },
    }
    ik = {
        "name": "teleop_ik",
        "type": "processor",
        "multiInstance": False,
        "description": (
            "G1_23 dual-arm IK diagnostic. It shares the real-time solver but never "
            "constructs or writes an arm_sdk publisher."
        ),
        "annotations": {
            "readOnlyHint": False,
            "destructiveHint": False,
            "idempotentHint": False,
        },
        "inputSchema": {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "action": {"type": "string", "enum": list(ik_actions)},
                **instance_id,
                "frame_json": {
                    "type": "string",
                    "maxLength": 16384,
                    "description": (
                        "Strict JSON object with head, left_controller and "
                        "right_controller poses; each pose has position [x,y,z] "
                        "and orientation [x,y,z,w]"
                    ),
                },
            },
            "required": ["action"],
            "x-action-params": ik_actions,
        },
        "x-teleop": copy.deepcopy(x_teleop),
        "x-teleop-diagnostic": {
            "profile_id": PROFILE_ID,
            "diagnostic_hardware_output": False,
            "diagnostic_publisher_present": False,
            "diagnostic_output_active": False,
            "shares_realtime_ik": True,
        },
    }
    return [session, state, ik]


def _require_mode(mode: str) -> None:
    if mode not in MODES:
        raise ValueError(f"mode must be one of {sorted(MODES)}")


__all__ = [
    "CAPABILITIES",
    "CAPTURE_PROTOCOL",
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
