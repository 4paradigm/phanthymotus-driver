"""Pure control helpers for the EngineAI T800 driver.

This module intentionally has no ROS dependency.  It owns validation, joint
layout, stream lifetimes, and the public MCP schemas so those contracts can be
tested on a development machine without ROS2 or a robot.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Sequence

import numpy as np


T800_JOINT_NAMES = (
    "J00_HIP_PITCH_L",
    "J01_HIP_ROLL_L",
    "J02_HIP_YAW_L",
    "J03_KNEE_PITCH_L",
    "J04_ANKLE_PITCH_L",
    "J05_ANKLE_ROLL_L",
    "J06_HIP_PITCH_R",
    "J07_HIP_ROLL_R",
    "J08_HIP_YAW_R",
    "J09_KNEE_PITCH_R",
    "J10_ANKLE_PITCH_R",
    "J11_ANKLE_ROLL_R",
    "J12_TORSO_YAW",
    "J13_SHOULDER_PITCH_L",
    "J14_SHOULDER_ROLL_L",
    "J15_SHOULDER_YAW_L",
    "J16_ELBOW_PITCH_L",
    "J17_ELBOW_YAW_L",
    "J18_SHOULDER_PITCH_R",
    "J19_SHOULDER_ROLL_R",
    "J20_SHOULDER_YAW_R",
    "J21_ELBOW_PITCH_R",
    "J22_ELBOW_YAW_R",
    "J23_HEAD_PITCH",
    "J24_HEAD_YAW",
)

T800_JOINT_GROUPS = {
    "left_leg": tuple(range(0, 6)),
    "right_leg": tuple(range(6, 12)),
    "legs": tuple(range(0, 12)),
    "torso": (12,),
    "left_arm": tuple(range(13, 18)),
    "right_arm": tuple(range(18, 23)),
    "arms": tuple(range(13, 23)),
    "head": (23, 24),
    "upper_body": tuple(range(12, 25)),
    "all": tuple(range(25)),
}

T800_JOINT_INDEX = {name: index for index, name in enumerate(T800_JOINT_NAMES)}

# Hard position limits copied from resource/serial_t800.urdf.  Gesture
# choreography validates every requested target against this table before it
# reaches the robot-facing planner; keeping the layout next to the canonical
# joint names makes index drift visible in the pure control tests.
T800_JOINT_POSITION_LIMITS = (
    (-3.316, 2.269),
    (-1.082, 2.059),
    (-1.42244667, 3.6022778),
    (0.0, 2.355),
    (-0.68068, 0.68068),
    (-0.3491, 0.1745),
    (-3.316, 2.269),
    (-2.059, 1.082),
    (-3.6022778, 1.42244667),
    (0.0, 2.355),
    (-0.68068, 0.68068),
    (-0.1745, 0.3491),
    (-4.381, 1.2392),
    (-2.967, 2.793),
    (-0.384, 2.443),
    (-2.618, 2.618),
    (-2.286, 0.262),
    (-2.618, 2.618),
    (-2.967, 2.793),
    (-2.443, 0.384),
    (-2.618, 2.618),
    (-2.286, 0.262),
    (-2.618, 2.618),
    (-0.523, 0.523),
    (-1.222, 1.222),
)

# Hard velocity limits copied from the same URDF, in rad/s.
T800_JOINT_VELOCITY_LIMITS = (
    25.96, 25.31, 23.19, 25.96, 33.51, 33.51,
    25.96, 25.31, 23.19, 25.96, 33.51, 33.51,
    23.19, 33.51, 33.51, 33.51, 33.51, 35.2,
    33.51, 33.51, 33.51, 33.51, 35.2, 35.2, 35.2,
)

MOTION_STATES = (
    "idle",
    "passive",
    "pd_stand",
    "rl_basic",
    "lower_body_balance",
    "joint_bridge",
    "pd_sitground",
    "walk_server",
    "rl_mimic_supine_to_stance",
    "rl_mimic_stance_to_supine",
    "rl_mimic_left_to_right",
    "rl_mimic_right_to_left",
    "rl_amp",
    "rl_terrain",
    "rl_recover_prone",
    "rl_floor_sitting",
    "dance",
)

WALK_MOTION_STATES = ("rl_basic", "lower_body_balance")

LED_MODES = {
    "blink_red": 0x1,
    "blink_green": 0x2,
    "blink_blue": 0x3,
    "blink_white": 0x4,
    "constant_white": 0x5,
    "constant_green": 0x6,
    "breathe_white": 0x7,
    "water_white": 0x8,
    "breathe_red": 0x9,
    "blink_orange": 0xA,
    "constant_orange": 0xB,
}


def clamp(value: float, lower: float, upper: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("value must be finite")
    return max(lower, min(upper, value))


class ControlValidationError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = str(code)


def validate_locomotion_request(
    action: str,
    arguments: dict,
    *,
    limits: Sequence[float],
    max_timed_duration_sec: float,
) -> dict:
    """Validate a loco action without coercing non-JSON-number inputs."""
    if len(limits) != 3 or any(
        not math.isfinite(float(limit)) or float(limit) <= 0 for limit in limits
    ):
        raise ValueError("locomotion limits must contain three positive finite values")
    max_vx, max_vy, max_vyaw = [float(limit) for limit in limits]
    defaults = {
        "move": {"vx": 0.0, "vy": 0.0, "vyaw": 0.0, "duration": 1.0},
        "move_displacement": {"x_m": 0.0, "y_m": 0.0, "speed_m_s": 0.3},
        "turn_angle": {"angle_rad": 0.0, "angular_speed_rad_s": 0.5},
        "arc": {"radius_m": 0.0, "angle_rad": 0.0, "linear_speed_m_s": 0.3},
    }
    if action not in defaults:
        raise ControlValidationError(
            "INVALID_ARGUMENT", f"unknown locomotion action: {action}"
        )
    values = {}
    for name, default in defaults[action].items():
        raw_value = arguments.get(name, default)
        if raw_value is None and action == "move":
            raw_value = default
        if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
            raise ControlValidationError(
                "INVALID_ARGUMENT", f"{name} must be a JSON number"
            )
        try:
            value = float(raw_value)
        except OverflowError:
            raise ControlValidationError(
                "INVALID_ARGUMENT", f"{name} must be finite"
            ) from None
        if not math.isfinite(value):
            raise ControlValidationError(
                "INVALID_ARGUMENT", f"{name} must be finite"
            )
        values[name] = value

    if action == "move":
        vx = values["vx"]
        vy = values["vy"]
        vyaw = values["vyaw"]
        duration = values["duration"]
        violations = [
            f"{name}={value:.6g} outside [{-limit:.6g}, {limit:.6g}]"
            for name, value, limit in (
                ("vx", vx, max_vx),
                ("vy", vy, max_vy),
                ("vyaw", vyaw, max_vyaw),
            )
            if value < -limit or value > limit
        ]
        if violations:
            raise ControlValidationError("SAFETY_LIMIT", "; ".join(violations))
    elif action == "move_displacement":
        x_m = values["x_m"]
        y_m = values["y_m"]
        speed = values["speed_m_s"]
        distance = math.hypot(x_m, y_m)
        if distance == 0:
            raise ControlValidationError(
                "INVALID_ARGUMENT",
                "x_m and y_m must define a non-zero displacement",
            )
        if speed < 0.01:
            raise ControlValidationError(
                "INVALID_ARGUMENT", "speed_m_s must be at least 0.01"
            )
        minimum_duration = max(
            abs(x_m) / max_vx if x_m else 0.0,
            abs(y_m) / max_vy if y_m else 0.0,
        )
        maximum_speed = distance / minimum_duration
        if speed > maximum_speed:
            raise ControlValidationError(
                "SAFETY_LIMIT",
                f"speed_m_s={speed:.6g} exceeds directional safety limit "
                f"{maximum_speed:.6g}",
            )
        duration = distance / speed
        vx = x_m / duration
        vy = y_m / duration
        vyaw = 0.0
    elif action == "turn_angle":
        angle = values["angle_rad"]
        speed = values["angular_speed_rad_s"]
        if angle == 0:
            raise ControlValidationError(
                "INVALID_ARGUMENT", "angle_rad must be non-zero"
            )
        if speed < 0.01:
            raise ControlValidationError(
                "INVALID_ARGUMENT", "angular_speed_rad_s must be at least 0.01"
            )
        if speed > max_vyaw:
            raise ControlValidationError(
                "SAFETY_LIMIT",
                f"angular_speed_rad_s={speed:.6g} exceeds safety limit "
                f"{max_vyaw:.6g}",
            )
        vx = vy = 0.0
        vyaw = math.copysign(speed, angle)
        duration = abs(angle) / speed
    else:
        radius = values["radius_m"]
        angle = values["angle_rad"]
        linear = values["linear_speed_m_s"]
        if radius <= 0 or angle == 0 or linear == 0:
            raise ControlValidationError(
                "INVALID_ARGUMENT",
                "radius_m must be positive; angle_rad and linear_speed_m_s "
                "must be non-zero",
            )
        if abs(linear) > max_vx:
            raise ControlValidationError(
                "SAFETY_LIMIT",
                f"linear_speed_m_s={linear:.6g} outside "
                f"[{-max_vx:.6g}, {max_vx:.6g}]",
            )
        angular_speed = abs(linear) / radius
        if angular_speed > max_vyaw:
            raise ControlValidationError(
                "SAFETY_LIMIT",
                f"arc angular speed {angular_speed:.6g} exceeds safety limit "
                f"{max_vyaw:.6g}",
            )
        vx = linear
        vy = 0.0
        vyaw = math.copysign(angular_speed, angle)
        duration = abs(angle) / angular_speed

    if duration < 0 and duration != -1:
        raise ControlValidationError(
            "INVALID_ARGUMENT",
            "duration must be -1 or a non-negative finite number",
        )
    if duration != -1 and duration > float(max_timed_duration_sec):
        raise ControlValidationError(
            "SAFETY_LIMIT",
            f"timed locomotion duration {duration:.3f}s exceeds "
            f"{float(max_timed_duration_sec):.0f}s safety limit; "
            "split the command or use move duration=-1 with stop_move",
        )
    return {
        "vx": vx,
        "vy": vy,
        "vyaw": vyaw,
        "duration": duration,
        "open_loop": action != "move",
    }


def float_list(
    value: object,
    name: str,
    *,
    size: int | None = None,
    allow_empty: bool = False,
) -> list[float]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{name} must be an array")
    result = [float(item) for item in value]
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite numbers")
    if not result and not allow_empty:
        raise ValueError(f"{name} cannot be empty")
    if size is not None and len(result) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    return result


def int_list(
    value: object,
    name: str,
    *,
    size: int | None = None,
    allow_empty: bool = False,
) -> list[int]:
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{name} must be an array")
    result = []
    for item in value:
        if isinstance(item, bool):
            raise ValueError(f"{name} must contain integers")
        converted = int(item)
        if float(item) != converted:
            raise ValueError(f"{name} must contain integers")
        result.append(converted)
    if not result and not allow_empty:
        raise ValueError(f"{name} cannot be empty")
    if size is not None and len(result) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    return result


def validate_joint_indices(indices: object, *, allow_empty: bool = False) -> list[int]:
    result = int_list(indices, "joint_indices", allow_empty=allow_empty)
    if len(set(result)) != len(result):
        raise ValueError("joint_indices must be unique")
    invalid = [idx for idx in result if idx < 0 or idx >= len(T800_JOINT_NAMES)]
    if invalid:
        raise ValueError(f"joint_indices out of range: {invalid}")
    return result


def validate_joint_positions(
    indices: object,
    positions: object,
    *,
    limit_margin_rad: float = 0.0,
) -> tuple[list[int], list[float]]:
    """Validate finite targets against the T800 URDF joint limits."""
    validated_indices = validate_joint_indices(indices)
    validated_positions = float_list(
        positions, "target_positions", size=len(validated_indices)
    )
    margin = float(limit_margin_rad)
    if not math.isfinite(margin) or margin < 0:
        raise ValueError("limit_margin_rad must be a non-negative finite number")
    violations = []
    for index, position in zip(validated_indices, validated_positions):
        hard_lower, hard_upper = T800_JOINT_POSITION_LIMITS[index]
        lower = hard_lower + margin
        upper = hard_upper - margin
        if lower > upper or position < lower or position > upper:
            violations.append(
                f"{T800_JOINT_NAMES[index]}={position:.6g} outside "
                f"[{lower:.6g}, {upper:.6g}]"
            )
    if violations:
        raise ValueError("joint target exceeds safe position limit: " + "; ".join(violations))
    return validated_indices, validated_positions


def validate_parallel_arrays(indices: Sequence[int], **arrays: Sequence[float]) -> None:
    for name, values in arrays.items():
        if len(values) not in (0, len(indices)):
            raise ValueError(f"{name} must be empty or match joint_indices length")


def joint_payload(
    position: Sequence[float],
    velocity: Sequence[float],
    torque: Sequence[float],
    *,
    timestamp_ms: int | None = None,
) -> dict:
    if not (len(position) == len(velocity) == len(torque)):
        raise ValueError("joint state arrays must have the same length")
    if len(position) > len(T800_JOINT_NAMES):
        raise ValueError("joint state contains more than 25 joints")
    joints = [
        {
            "idx": idx,
            "name": T800_JOINT_NAMES[idx],
            "q": float(position[idx]),
            "dq": float(velocity[idx]),
            "tau": float(torque[idx]),
        }
        for idx in range(len(position))
    ]
    return {
        "joints": joints,
        "timestamp_ms": timestamp_ms if timestamp_ms is not None else int(time.time() * 1000),
    }


def validate_recording_frames(
    frames: object,
    *,
    max_frames: int,
    joint_count: int = len(T800_JOINT_NAMES),
    minimum_frames: int = 1,
    max_duration_ms: float | None = None,
    minimum_interval_ms: float | None = None,
) -> list[dict]:
    """Validate and normalize persisted motion-recorder frames."""
    if not isinstance(frames, list):
        raise ValueError("frames must be a JSON array")
    if len(frames) < int(minimum_frames):
        if int(minimum_frames) == 2:
            raise ValueError("at least two timestamped frames are required")
        raise ValueError(f"recording must contain at least {minimum_frames} frame(s)")
    if len(frames) > int(max_frames):
        raise ValueError(f"recording exceeds maximum frame count {max_frames}")
    if joint_count != len(T800_JOINT_NAMES):
        raise ValueError(
            f"joint_count must match the T800 layout ({len(T800_JOINT_NAMES)})"
        )
    if minimum_interval_ms is not None and (
        not math.isfinite(float(minimum_interval_ms))
        or float(minimum_interval_ms) <= 0
    ):
        raise ValueError("minimum_interval_ms must be positive and finite")

    normalized = []
    previous_timestamp = None
    for frame_index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            raise ValueError(f"frame {frame_index} must be an object")
        if "timestamp" not in frame:
            raise ValueError(f"frame {frame_index} is missing timestamp")
        timestamp_value = frame["timestamp"]
        if isinstance(timestamp_value, bool) or not isinstance(
            timestamp_value, (int, float)
        ):
            raise ValueError(f"frame {frame_index} timestamp must be a number")
        try:
            timestamp = float(timestamp_value)
        except OverflowError:
            raise ValueError(
                f"frame {frame_index} timestamp must be finite"
            ) from None
        if not math.isfinite(timestamp) or timestamp < 0:
            raise ValueError(
                f"frame {frame_index} timestamp must be finite and non-negative"
            )
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            raise ValueError("frame timestamps must be strictly increasing")
        if (
            previous_timestamp is not None
            and minimum_interval_ms is not None
            and timestamp - previous_timestamp < float(minimum_interval_ms)
        ):
            raise ValueError(
                f"frame interval {timestamp - previous_timestamp:.6g}ms is below "
                f"minimum {float(minimum_interval_ms):.6g}ms"
            )
        previous_timestamp = timestamp

        positions_value = frame.get("positions")
        if not isinstance(positions_value, list):
            raise ValueError(f"frame {frame_index} positions must be an array")
        if len(positions_value) != joint_count:
            raise ValueError(
                f"frame {frame_index} positions must contain exactly "
                f"{joint_count} joints"
            )
        if any(
            isinstance(value, bool) or not isinstance(value, (int, float))
            for value in positions_value
        ):
            raise ValueError(
                f"frame {frame_index} positions must contain only numbers"
            )
        try:
            positions = [float(value) for value in positions_value]
        except OverflowError:
            raise ValueError(
                f"frame {frame_index} positions must be finite"
            ) from None
        if not all(math.isfinite(value) for value in positions):
            raise ValueError(f"frame {frame_index} positions must be finite")
        limit_violations = []
        for joint_index, position in enumerate(positions):
            lower, upper = T800_JOINT_POSITION_LIMITS[joint_index]
            if position < lower or position > upper:
                joint_name = T800_JOINT_NAMES[joint_index]
                limit_violations.append(
                    f"{joint_name}={position:.6g} outside "
                    f"[{lower:.6g}, {upper:.6g}]"
                )
        if limit_violations:
            raise ValueError(
                f"frame {frame_index} exceeds safe position limit: "
                + "; ".join(limit_violations)
            )

        normalized_frame = {
            "timestamp": timestamp,
            "positions": positions,
        }
        velocities_value = frame.get("velocities")
        if velocities_value is not None:
            if not isinstance(velocities_value, list) or len(velocities_value) != joint_count:
                raise ValueError(
                    f"frame {frame_index} velocities must contain exactly "
                    f"{joint_count} joints"
                )
            if any(
                isinstance(value, bool) or not isinstance(value, (int, float))
                for value in velocities_value
            ):
                raise ValueError(
                    f"frame {frame_index} velocities must contain only numbers"
                )
            try:
                velocities = [float(value) for value in velocities_value]
            except OverflowError:
                raise ValueError(
                    f"frame {frame_index} velocities must be finite"
                ) from None
            if not all(math.isfinite(value) for value in velocities):
                raise ValueError(f"frame {frame_index} velocities must be finite")
            normalized_frame["velocities"] = velocities
        normalized.append(normalized_frame)
    if normalized and max_duration_ms is not None:
        duration_ms = normalized[-1]["timestamp"] - normalized[0]["timestamp"]
        if duration_ms > float(max_duration_ms):
            raise ValueError(
                f"recording duration {duration_ms:.6g}ms exceeds maximum "
                f"{float(max_duration_ms):.6g}ms"
            )
    return normalized


def validate_recording_document(
    data: object,
    *,
    max_frames: int,
    minimum_frames: int,
    max_duration_ms: float,
    minimum_interval_ms: float | None = None,
) -> tuple[list[dict], dict]:
    if not isinstance(data, dict):
        raise ValueError("recording root must be an object")
    metadata = data.get("metadata", {})
    if not isinstance(metadata, dict):
        raise ValueError("recording metadata must be an object")
    frames = validate_recording_frames(
        data.get("frames"),
        max_frames=max_frames,
        minimum_frames=minimum_frames,
        max_duration_ms=max_duration_ms,
        minimum_interval_ms=minimum_interval_ms,
    )
    return frames, dict(metadata)


def resample_joint_trajectory(
    frames: list[dict],
    *,
    joint_indices: Sequence[int],
    current_positions: Sequence[float],
    playback_rate_hz: float,
    speed_scale: float,
    entry_blend_sec: float,
    max_samples: int | None = None,
) -> list[tuple[list[float], list[float]]]:
    """Build a bounded C1 trajectory with a quintic entry blend."""
    playback_rate_hz = float(playback_rate_hz)
    speed_scale = float(speed_scale)
    entry_blend_sec = float(entry_blend_sec)
    if not math.isfinite(playback_rate_hz) or playback_rate_hz <= 0:
        raise ValueError("playback_rate_hz must be positive and finite")
    if not math.isfinite(speed_scale) or speed_scale <= 0:
        raise ValueError("speed_scale must be positive and finite")
    if not math.isfinite(entry_blend_sec) or entry_blend_sec <= 0:
        raise ValueError("entry_blend_sec must be positive and finite")
    indices = validate_joint_indices(joint_indices)
    timestamps = np.asarray([float(frame["timestamp"]) for frame in frames], dtype=float)
    if timestamps.ndim != 1 or len(timestamps) < 2:
        raise ValueError("at least two timestamped frames are required")
    timestamps = (timestamps - timestamps[0]) / 1000.0 / float(speed_scale)
    if np.any(np.diff(timestamps) <= 0):
        raise ValueError("frame timestamps must be strictly increasing")

    positions = np.asarray(
        [[float(frame["positions"][index]) for index in indices] for frame in frames],
        dtype=float,
    )
    if positions.shape != (len(frames), len(indices)):
        raise ValueError("each frame must contain all requested joint positions")
    if not np.all(np.isfinite(positions)):
        raise ValueError("joint positions must be finite")
    if len(current_positions) <= max(indices):
        raise ValueError("current joint state is unavailable for entry blend")
    current = np.asarray([float(current_positions[index]) for index in indices], dtype=float)
    if current.shape != (len(indices),) or not np.all(np.isfinite(current)):
        raise ValueError("current joint state is unavailable for entry blend")

    duration = float(timestamps[-1])
    sample_count = max(2, int(math.ceil(duration * playback_rate_hz)) + 1)
    blend_count = max(2, int(math.ceil(entry_blend_sec * playback_rate_hz)) + 1)
    total_sample_count = sample_count + blend_count - 1
    if max_samples is not None and total_sample_count > int(max_samples):
        raise ValueError(
            f"playback sample count {total_sample_count} exceeds maximum "
            f"{int(max_samples)}"
        )
    sample_times = np.linspace(0.0, duration, sample_count)
    tangents = np.gradient(positions, timestamps, axis=0)
    segments = np.searchsorted(timestamps, sample_times, side="right") - 1
    segments = np.clip(segments, 0, len(timestamps) - 2)
    left_time = timestamps[segments]
    segment_duration = timestamps[segments + 1] - left_time
    phase = (sample_times - left_time) / segment_duration
    phase2 = phase * phase
    phase3 = phase2 * phase
    left_position = positions[segments]
    right_position = positions[segments + 1]
    sampled_positions = (
        (2.0 * phase3 - 3.0 * phase2 + 1.0)[:, None] * left_position
        + (phase3 - 2.0 * phase2 + phase)[:, None]
        * segment_duration[:, None] * tangents[segments]
        + (-2.0 * phase3 + 3.0 * phase2)[:, None] * right_position
        + (phase3 - phase2)[:, None]
        * segment_duration[:, None] * tangents[segments + 1]
    )
    sampled_positions = np.clip(
        sampled_positions,
        np.minimum(left_position, right_position),
        np.maximum(left_position, right_position),
    )
    sampled_velocities = np.gradient(sampled_positions, sample_times, axis=0)

    blend_times = np.linspace(0.0, entry_blend_sec, blend_count)
    delta = sampled_positions[0] - current
    initial_velocity = np.zeros_like(current)
    final_velocity = sampled_velocities[0]
    a0 = current
    a1 = initial_velocity
    a2 = np.zeros_like(current)
    a3 = (
        20.0 * delta - (8.0 * final_velocity + 12.0 * initial_velocity) * entry_blend_sec
    ) / (2.0 * entry_blend_sec**3)
    a4 = (
        -30.0 * delta + (14.0 * final_velocity + 16.0 * initial_velocity) * entry_blend_sec
    ) / (2.0 * entry_blend_sec**4)
    a5 = (
        12.0 * delta - 6.0 * (final_velocity + initial_velocity) * entry_blend_sec
    ) / (2.0 * entry_blend_sec**5)
    t = blend_times[:, None]
    blend_positions = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
    blend_positions = np.clip(
        blend_positions,
        np.minimum(current, sampled_positions[0]),
        np.maximum(current, sampled_positions[0]),
    )
    blend_velocities = np.gradient(blend_positions, blend_times, axis=0)
    sampled_positions = np.vstack((blend_positions, sampled_positions[1:]))
    sampled_velocities = np.vstack((blend_velocities, sampled_velocities[1:]))
    velocity_limits = np.asarray(
        [T800_JOINT_VELOCITY_LIMITS[index] for index in indices],
        dtype=float,
    )
    feedforward_peak = np.max(np.abs(sampled_velocities), axis=0)
    combined_times = np.concatenate(
        (blend_times, entry_blend_sec + sample_times[1:])
    )
    target_intervals = np.diff(combined_times)
    if np.any(target_intervals <= 0):
        raise ValueError("playback target intervals must be strictly positive")
    target_step_peak = np.max(
        np.abs(np.diff(sampled_positions, axis=0))
        / target_intervals[:, None],
        axis=0,
    )
    violations = []
    for index, feedforward, target_step, limit in zip(
        indices, feedforward_peak, target_step_peak, velocity_limits
    ):
        if target_step > limit + 1e-6:
            violations.append(
                f"target-step velocity {T800_JOINT_NAMES[index]}="
                f"{target_step:.6g}rad/s exceeds {limit:.6g}rad/s"
            )
        elif feedforward > limit + 1e-6:
            violations.append(
                f"feedforward velocity {T800_JOINT_NAMES[index]}="
                f"{feedforward:.6g}rad/s exceeds {limit:.6g}rad/s"
            )
    if violations:
        raise ValueError(
            "derived joint velocity exceeds URDF safety limit: "
            + "; ".join(violations)
        )
    return [
        (sampled_positions[index].tolist(), sampled_velocities[index].tolist())
        for index in range(len(sampled_positions))
    ]


@dataclass(frozen=True)
class StreamSnapshot:
    active: bool
    started_at: float | None
    deadline: float | None
    last_publish_at: float | None
    publish_count: int
    error: str | None


class RepeatingCommand:
    """Publish the latest command at a fixed rate until stopped or expired."""

    def __init__(
        self,
        publisher: Callable[[dict], None],
        stop_publisher: Callable[[], None],
        *,
        rate_hz: float,
        clock: Callable[[], float] = time.monotonic,
    ):
        if rate_hz <= 0:
            raise ValueError("rate_hz must be positive")
        self._publisher = publisher
        self._stop_publisher = stop_publisher
        self._period = 1.0 / rate_hz
        self._clock = clock
        self._lifecycle_lock = threading.RLock()
        self._publish_lock = threading.Lock()
        self._lock = threading.Lock()
        self._stop_event: threading.Event | None = None
        self._started_at: float | None = None
        self._deadline: float | None = None
        self._last_publish_at: float | None = None
        self._publish_count = 0
        self._last_error: str | None = None

    def start(self, command: dict, duration: float) -> StreamSnapshot:
        duration = float(duration)
        if not math.isfinite(duration) or (duration < 0 and duration != -1):
            raise ValueError("duration must be -1 or a non-negative finite number")
        with self._lifecycle_lock:
            self._stop_locked()
            if duration == 0:
                return self.snapshot()

            stop_event = threading.Event()
            started_at = self._clock()
            deadline = None if duration == -1 else started_at + duration
            with self._publish_lock:
                with self._lock:
                    self._stop_event = stop_event
                    self._started_at = started_at
                    self._deadline = deadline
                    self._last_publish_at = started_at
                    self._publish_count = 1
                    self._last_error = None
                try:
                    # Publish once before handing off to the worker. The lock
                    # orders this command before a concurrent stop.
                    self._publisher(command)
                except Exception as exc:
                    with self._lock:
                        if self._stop_event is stop_event:
                            self._stop_event = None
                            self._publish_count = 0
                            self._last_publish_at = None
                            self._last_error = str(exc)
                    raise

            thread = threading.Thread(
                target=self._run,
                args=(command, stop_event, deadline),
                daemon=True,
                name="t800-command-stream",
            )
            thread.start()
            return self.snapshot()

    def _run(
        self,
        command: dict,
        stop_event: threading.Event,
        deadline: float | None,
    ) -> None:
        try:
            while not stop_event.wait(self._period):
                now = self._clock()
                if deadline is not None and now >= deadline:
                    break
                with self._publish_lock:
                    with self._lock:
                        owns_stream = (
                            self._stop_event is stop_event
                            and not stop_event.is_set()
                        )
                    if not owns_stream:
                        break
                    self._publisher(command)
                    with self._lock:
                        if self._stop_event is stop_event:
                            self._last_publish_at = now
                            self._publish_count += 1
        except Exception as exc:
            with self._lock:
                if self._stop_event is stop_event:
                    self._last_error = str(exc)
        finally:
            with self._publish_lock:
                with self._lock:
                    owns_stream = self._stop_event is stop_event
                    if owns_stream:
                        self._stop_event = None
                if owns_stream:
                    try:
                        self._stop_publisher()
                    except Exception as exc:
                        with self._lock:
                            if self._last_error is None:
                                self._last_error = f"stop publish failed: {exc}"

    def stop(self) -> bool:
        with self._lifecycle_lock:
            return self._stop_locked()

    def _stop_locked(self) -> bool:
        with self._publish_lock:
            with self._lock:
                stop_event = self._stop_event
                if stop_event is None:
                    return False
                self._stop_event = None
                stop_event.set()
            self._stop_publisher()
            return True

    def snapshot(self) -> StreamSnapshot:
        with self._lock:
            return StreamSnapshot(
                active=self._stop_event is not None,
                started_at=self._started_at,
                deadline=self._deadline,
                last_publish_at=self._last_publish_at,
                publish_count=self._publish_count,
                error=self._last_error,
            )


def action_schema(
    actions: dict[str, tuple[list[str], str]],
    properties: dict,
    description: str,
    *,
    completion: tuple[list[str], int] | None = None,
) -> dict:
    schema = {
        "type": "object",
        "properties": {
            "action": {
                "type": "string",
                "enum": list(actions),
                "description": description,
            },
            **properties,
        },
        "required": ["action"],
        "x-action-params": {
            name: {"params": params, "description": action_description}
            for name, (params, action_description) in actions.items()
        },
    }
    if completion is not None:
        schema["x-completion"] = {
            "actions": completion[0],
            "timeout": completion[1],
        }
    return schema


def sensor_action_schema() -> dict:
    """Lifecycle schema used by Agent Core to start and resolve sensor topics."""
    actions = {
        "start": ([], "启动卡片数据流"),
        "info": ([], "返回卡片状态和实际输出 topic"),
        "stop": ([], "停止卡片数据流"),
        "status": ([], "返回最新传感器状态"),
    }
    schema = action_schema(actions, {}, "传感器生命周期动作")
    # Reading a sensor directly without an action remains supported.
    schema.pop("required", None)
    return schema


def array_property(description: str, *, item_type: str = "number") -> dict:
    return {
        "type": "array",
        "items": {"type": item_type},
        "description": description,
    }


def sensor_tool(name: str, description: str, topic: str, fmt: str) -> dict:
    return {
        "name": name,
        "type": "sensor",
        "multiInstance": False,
        "readOnly": True,
        "description": description,
        "inputSchema": sensor_action_schema(),
        "topic_out": [{"topic": topic, "format": fmt}],
    }


def optional_floats(args: dict, name: str, count: int) -> list[float]:
    value = args.get(name, [])
    return float_list(value, name, size=count, allow_empty=True) if value else []


def list_or_default(values: Iterable[float], size: int, default: float = 0.0) -> list[float]:
    result = list(values)
    return result if result else [default] * size
