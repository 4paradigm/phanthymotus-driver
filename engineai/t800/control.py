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

MOTION_STATES = (
    "idle",
    "passive",
    "pd_stand",
    "walk",
    "dance",
    "supine_to_stance",
    "stance_to_supine",
    "joint_bridge",
    "lower_body_balance",
    "rl_terrain",
)

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


@dataclass(frozen=True)
class StreamSnapshot:
    active: bool
    started_at: float | None
    deadline: float | None
    last_publish_at: float | None
    publish_count: int


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
        self._lock = threading.Lock()
        self._stop_event: threading.Event | None = None
        self._started_at: float | None = None
        self._deadline: float | None = None
        self._last_publish_at: float | None = None
        self._publish_count = 0

    def start(self, command: dict, duration: float) -> StreamSnapshot:
        duration = float(duration)
        if not math.isfinite(duration) or duration < -1:
            raise ValueError("duration must be -1 or a non-negative finite number")
        self.stop()
        if duration == 0:
            return self.snapshot()

        stop_event = threading.Event()
        started_at = self._clock()
        deadline = None if duration == -1 else started_at + duration
        with self._lock:
            self._stop_event = stop_event
            self._started_at = started_at
            self._deadline = deadline
            self._last_publish_at = started_at
            self._publish_count = 1
        try:
            # Publish once before handing off to the worker. This guarantees a
            # short command is not lost if the scheduler starts the thread late.
            self._publisher(command)
        except Exception:
            with self._lock:
                if self._stop_event is stop_event:
                    self._stop_event = None
                    self._publish_count = 0
                    self._last_publish_at = None
            raise

        def run() -> None:
            try:
                while not stop_event.wait(self._period):
                    now = self._clock()
                    if deadline is not None and now >= deadline:
                        break
                    self._publisher(command)
                    with self._lock:
                        self._last_publish_at = now
                        self._publish_count += 1
                    stop_event.wait(self._period)
            finally:
                with self._lock:
                    owns_stream = self._stop_event is stop_event
                    if owns_stream:
                        self._stop_event = None
                # A replaced stream was already stopped before its replacement
                # started.  It must not inject a late zero into the new stream.
                if owns_stream:
                    self._stop_publisher()

        threading.Thread(target=run, daemon=True, name="t800-command-stream").start()
        return self.snapshot()

    def stop(self) -> bool:
        with self._lock:
            stop_event = self._stop_event
            self._stop_event = None
        if stop_event is None:
            return False
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
            )


def action_schema(
    actions: dict[str, tuple[list[str], str]],
    properties: dict,
    description: str,
) -> dict:
    return {
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
