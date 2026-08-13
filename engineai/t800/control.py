"""Pure control helpers for the EngineAI T800 driver.

This module intentionally has no ROS dependency.  It owns validation, joint
layout, stream lifetimes, and the public MCP schemas so those contracts can be
tested on a development machine without ROS2 or a robot.
"""

from __future__ import annotations

import math
import struct
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
            self._last_publish_at = None
            self._publish_count = 0

        def run() -> None:
            try:
                while not stop_event.is_set():
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


def array_property(description: str, *, item_type: str = "number") -> dict:
    return {
        "type": "array",
        "items": {"type": item_type},
        "description": description,
    }


def quaternion_to_yaw_rad(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_odometry_payload(
    *,
    frame_id: str,
    child_frame_id: str,
    position: Sequence[float],
    orientation: Sequence[float],
    linear_velocity: Sequence[float],
    angular_velocity: Sequence[float],
    stamp_sec: int = 0,
    stamp_nanosec: int = 0,
    received_monotonic: float | None = None,
    stale_timeout_sec: float = 1.0,
    now_monotonic: float | None = None,
) -> dict:
    if len(position) < 3 or len(orientation) < 4:
        raise ValueError("odometry pose must include position xyz and orientation xyzw")
    if len(linear_velocity) < 3 or len(angular_velocity) < 3:
        raise ValueError("odometry twist must include linear and angular xyz")

    qx, qy, qz, qw = (float(orientation[0]), float(orientation[1]),
                       float(orientation[2]), float(orientation[3]))
    payload = {
        "x": float(position[0]),
        "y": float(position[1]),
        "z": float(position[2]),
        "qx": qx,
        "qy": qy,
        "qz": qz,
        "qw": qw,
        "yaw_rad": quaternion_to_yaw_rad(qx, qy, qz, qw),
        "vx": float(linear_velocity[0]),
        "vy": float(linear_velocity[1]),
        "vz": float(linear_velocity[2]),
        "wx": float(angular_velocity[0]),
        "wy": float(angular_velocity[1]),
        "wz": float(angular_velocity[2]),
        "frame_id": str(frame_id or ""),
        "child_frame_id": str(child_frame_id or ""),
        "stamp_sec": int(stamp_sec),
        "stamp_nanosec": int(stamp_nanosec),
        "timestamp_ms": int(stamp_sec) * 1000 + int(stamp_nanosec) // 1_000_000,
        "source": "nav_msgs/Odometry",
    }
    if received_monotonic is not None:
        now = time.monotonic() if now_monotonic is None else now_monotonic
        payload["age_sec"] = max(0.0, now - received_monotonic)
        payload["stale"] = payload["age_sec"] > float(stale_timeout_sec)
    else:
        payload["age_sec"] = None
        payload["stale"] = True
    return payload


def sensor_tool(name: str, description: str, topic: str, fmt: str) -> dict:
    return {
        "name": name,
        "type": "sensor",
        "multiInstance": False,
        "readOnly": True,
        "description": description,
        "inputSchema": {"type": "object", "properties": {}},
        "topic_out": [{"topic": topic, "format": fmt}],
    }


# Occupancy cell values (nav_msgs/OccupancyGrid compatible semantics).
CELL_UNKNOWN = -1
CELL_FREE = 0
CELL_OCCUPIED = 100


def extract_xyz_from_pointcloud2(
    fields: Sequence,
    point_step: int,
    width: int,
    height: int,
    data: bytes | bytearray | Sequence[int],
) -> list[tuple[float, float, float]]:
    """Parse XYZ float32 fields from a sensor_msgs/PointCloud2 payload."""
    offsets: dict[str, int] = {}
    for field in fields:
        name = getattr(field, "name", None)
        if name is None and isinstance(field, dict):
            name = field.get("name")
            offset = int(field.get("offset", 0))
            datatype = int(field.get("datatype", 0))
        else:
            offset = int(getattr(field, "offset", 0))
            datatype = int(getattr(field, "datatype", 0))
        if name in ("x", "y", "z") and datatype in (7, 8):  # FLOAT32 / FLOAT64
            offsets[str(name)] = offset
    if not {"x", "y", "z"}.issubset(offsets):
        return []

    raw = bytes(data) if not isinstance(data, (bytes, bytearray)) else bytes(data)
    count = int(width) * int(height)
    step = int(point_step)
    if step <= 0 or count <= 0 or len(raw) < step:
        return []

    points: list[tuple[float, float, float]] = []
    for index in range(count):
        base = index * step
        if base + step > len(raw):
            break
        x = struct.unpack_from("<f", raw, base + offsets["x"])[0]
        y = struct.unpack_from("<f", raw, base + offsets["y"])[0]
        z = struct.unpack_from("<f", raw, base + offsets["z"])[0]
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        points.append((x, y, z))
    return points


def pack_sensor_mapping_binary(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    points_xyz: Sequence[Sequence[float]],
    *,
    max_points: int = 50000,
) -> bytes:
    """Agent Core sensor/mapping wire format used by Unitree spatial cards.

    Layout: [float32 robot_x,y,yaw][uint8 flags][uint32 N][float32 x,y,z × N]
    flags bit0=full_map, bit1=has_z. Occupied cells are published as points with z=0.
    """
    points = list(points_xyz)
    if len(points) > max_points:
        # Deterministic downsample for tests/reproducibility.
        stride = max(1, len(points) // max_points)
        points = points[::stride][:max_points]
    flags = 0x03
    header = struct.pack(
        "<fffBI",
        float(robot_x),
        float(robot_y),
        float(robot_yaw),
        flags,
        len(points),
    )
    body = bytearray()
    for point in points:
        body.extend(struct.pack("<fff", float(point[0]), float(point[1]), float(point[2])))
    return bytes(header + body)


class OccupancyGrid2D:
    """Accumulate SLAM points into a 2D occupancy grid (pure Python, no numpy)."""

    def __init__(
        self,
        *,
        resolution_m: float = 0.1,
        z_min_m: float = 0.1,
        z_max_m: float = 1.8,
        min_hits: int = 1,
        max_extent_m: float = 40.0,
    ):
        if resolution_m <= 0:
            raise ValueError("resolution_m must be > 0")
        self.resolution_m = float(resolution_m)
        self.z_min_m = float(z_min_m)
        self.z_max_m = float(z_max_m)
        self.min_hits = max(1, int(min_hits))
        self.max_extent_m = float(max_extent_m)
        self.frame_id = ""
        self._hits: dict[tuple[int, int], int] = {}
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._width = 0
        self._height = 0

    def clear(self) -> None:
        self._hits.clear()
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._width = 0
        self._height = 0
        self.frame_id = ""

    def ingest_points(self, points: Sequence[Sequence[float]], *, frame_id: str = "") -> int:
        accepted = 0
        half = self.max_extent_m * 0.5
        for point in points:
            if len(point) < 3:
                continue
            x, y, z = float(point[0]), float(point[1]), float(point[2])
            if not (self.z_min_m <= z <= self.z_max_m):
                continue
            if abs(x) > half or abs(y) > half:
                continue
            gx = int(math.floor(x / self.resolution_m))
            gy = int(math.floor(y / self.resolution_m))
            self._hits[(gx, gy)] = self._hits.get((gx, gy), 0) + 1
            accepted += 1
        if frame_id:
            self.frame_id = str(frame_id)
        self._rebuild_bounds()
        return accepted

    def _rebuild_bounds(self) -> None:
        if not self._hits:
            self._origin_x = 0.0
            self._origin_y = 0.0
            self._width = 0
            self._height = 0
            return
        xs = [key[0] for key in self._hits]
        ys = [key[1] for key in self._hits]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        self._origin_x = min_x * self.resolution_m
        self._origin_y = min_y * self.resolution_m
        self._width = max_x - min_x + 1
        self._height = max_y - min_y + 1

    def snapshot(self) -> dict:
        occupied = 0
        for hits in self._hits.values():
            if hits >= self.min_hits:
                occupied += 1
        total = self._width * self._height
        # v1: free cells require raycasting; untreated cells in bbox stay unknown.
        unknown = max(0, total - occupied)
        return {
            "frame_id": self.frame_id,
            "resolution_m": self.resolution_m,
            "origin_x": self._origin_x,
            "origin_y": self._origin_y,
            "width": self._width,
            "height": self._height,
            "occupied": occupied,
            "free": 0,
            "unknown": unknown,
            "hit_cells": len(self._hits),
            "z_min_m": self.z_min_m,
            "z_max_m": self.z_max_m,
        }

    def occupied_cell_centers(self, *, max_points: int = 50000) -> list[tuple[float, float, float]]:
        centers: list[tuple[float, float, float]] = []
        half = self.resolution_m * 0.5
        for (gx, gy), hits in self._hits.items():
            if hits < self.min_hits:
                continue
            centers.append((gx * self.resolution_m + half, gy * self.resolution_m + half, 0.0))
            if len(centers) >= max_points:
                break
        return centers


def optional_floats(args: dict, name: str, count: int) -> list[float]:
    value = args.get(name, [])
    return float_list(value, name, size=count, allow_empty=True) if value else []


def list_or_default(values: Iterable[float], size: int, default: float = 0.0) -> list[float]:
    result = list(values)
    return result if result else [default] * size
