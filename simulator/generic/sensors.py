"""Derive sensor payloads from world state.

Pure functions, no I/O, no ROS. Every sensor comes off the *same* world state
rather than inventing its own numbers — a camera that disagrees with the lidar
that disagrees with the odometry cannot test anything, and a suite that passes
against noise proves nothing.

Wire framing deliberately does **not** live here. These functions return numbers;
the cards pack them (`<fffBI` for `sensor/mapping`, zlib for `image/depth-zlib`).
That split is what lets a physics backend change where the numbers come from
without touching how they are framed.
"""

from __future__ import annotations

import math

from simulator.generic.geometry import OCCUPIED, OccupancyGrid


def odom(snapshot: dict) -> dict:
    pose = snapshot["pose"]
    return {
        "pose": pose,
        "twist": {"linear": snapshot["lin"], "angular": snapshot["ang"]},
        "odometer_m": snapshot["odometer"],
        "stamp": snapshot["t"],
    }


def battery(elapsed_s: float, odometer_m: float, config: dict | None = None) -> dict:
    """Drains with time and with distance, so a long tour visibly costs charge.

    Scripted low-battery is then a matter of `drain_per_metre`, not of a special
    case somewhere — the scenario changes one number and the robot runs flat.
    """
    cfg = config or {}
    start = float(cfg.get("start_percent", 100.0))
    per_second = float(cfg.get("drain_per_second", 0.005))
    per_metre = float(cfg.get("drain_per_metre", 0.05))
    percent = start - per_second * max(0.0, elapsed_s) - per_metre * max(0.0, odometer_m)
    percent = max(0.0, min(100.0, percent))
    return {
        "percent": round(percent, 2),
        "voltage": round(42.0 + 12.0 * percent / 100.0, 2),
        "charging": False,
        "low": percent <= float(cfg.get("low_threshold", 20.0)),
    }


def imu(sense: dict, stamp: float) -> dict:
    payload = dict(sense.get("imu") or {})
    payload["stamp"] = stamp
    return payload


def laser_scan(sense: dict, stamp: float) -> dict:
    payload = dict(sense.get("scan") or {})
    ranges = payload.get("ranges") or []
    payload["ranges"] = [round(value, 3) for value in ranges]
    payload["count"] = len(ranges)
    payload["stamp"] = stamp
    return payload


def joints(sense: dict, joint_names: list[str], stamp: float) -> dict:
    values = list(sense.get("joints") or [])
    # Names must match the URDF the `model` card serves, character for character,
    # or the skeleton renderer draws nothing and says nothing about why.
    return {"names": list(joint_names[:len(values)]),
            "positions": [round(value, 5) for value in values],
            "stamp": stamp}


def occupancy_points(grid: OccupancyGrid, max_points: int = 60000, z: float = 0.0
                     ) -> list[tuple[float, float, float]]:
    """Occupied cells as a point set, decimated to fit ``max_points``.

    The mapping renderer is a 3D point cloud, so a 2D grid reaches it as points
    at z≈0. Decimation is a fixed stride rather than a random sample: a map that
    flickers between frames reads as sensor noise that is not there.
    """
    occupied = [(cx, cy) for cy in range(grid.height) for cx in range(grid.width)
                if grid.cells[cy * grid.width + cx] == OCCUPIED]
    if not occupied:
        return []
    stride = max(1, math.ceil(len(occupied) / max(1, max_points)))
    return [(*grid.cell_to_world(cx, cy), z) for cx, cy in occupied[::stride]]


def marker_points(positions: list[tuple[float, float]], z: float,
                  radius: float = 0.12, per_marker: int = 12
                  ) -> list[tuple[float, float, float]]:
    """Small rings at ``z`` so waypoints and the travelled path read as distinct.

    Height carries the meaning because the renderer colours by z — the map sits
    at 0, the path just above it, the waypoints above that.
    """
    points: list[tuple[float, float, float]] = []
    for x, y in positions:
        for i in range(per_marker):
            theta = 2.0 * math.pi * i / per_marker
            points.append((x + radius * math.cos(theta), y + radius * math.sin(theta), z))
    return points
