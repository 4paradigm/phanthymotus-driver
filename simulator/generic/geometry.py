"""Pose and occupancy-grid primitives.

Pure stdlib on purpose. Everything here is plain data plus arithmetic, so the
whole collision and navigation chain tests on a laptop with no numpy, no ROS and
no simulator process.
"""

from __future__ import annotations

import base64
import json
import math
import zlib
from dataclasses import dataclass
from pathlib import Path

FREE = 0
OCCUPIED = 100
UNKNOWN = 255


def normalize_angle(theta: float) -> float:
    """Wrap to (-pi, pi]."""
    return math.atan2(math.sin(theta), math.cos(theta))


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def copy(self) -> "Pose":
        return Pose(self.x, self.y, self.yaw)

    def as_dict(self) -> dict:
        return {"x": round(self.x, 4), "y": round(self.y, 4), "yaw": round(self.yaw, 4)}

    def distance_to(self, other: "Pose") -> float:
        return math.hypot(other.x - self.x, other.y - self.y)

    def bearing_to(self, other: "Pose") -> float:
        return math.atan2(other.y - self.y, other.x - self.x)


class OccupancyGrid:
    """Row-major uint8 grid. Cell (0, 0) covers world ``origin`` to ``origin + resolution``.

    Anything outside the grid counts as occupied — the map edge is a wall, so a
    runaway integration cannot silently drive off into empty coordinates and
    report success.
    """

    def __init__(self, resolution: float, origin: tuple[float, float], width: int, height: int,
                 cells: bytearray | bytes | None = None):
        if resolution <= 0:
            raise ValueError("resolution must be positive")
        if width <= 0 or height <= 0:
            raise ValueError("grid must be non-empty")
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.width = int(width)
        self.height = int(height)
        if cells is None:
            self.cells = bytearray([FREE]) * (self.width * self.height)
        else:
            if len(cells) != self.width * self.height:
                raise ValueError(f"cells length {len(cells)} != {self.width}x{self.height}")
            self.cells = bytearray(cells)

    # ---- construction -------------------------------------------------

    @classmethod
    def blank(cls, resolution: float = 0.05, origin: tuple[float, float] = (-5.0, -5.0),
              width: int = 400, height: int = 400) -> "OccupancyGrid":
        return cls(resolution, origin, width, height)

    @classmethod
    def from_dict(cls, data: dict) -> "OccupancyGrid":
        raw = data.get("data")
        cells = None
        if raw is not None:
            cells = zlib.decompress(base64.b64decode(raw))
        return cls(data["resolution"], tuple(data["origin"]), data["width"], data["height"], cells)

    @classmethod
    def load(cls, path: str | Path) -> "OccupancyGrid":
        with Path(path).open(encoding="utf-8") as handle:
            return cls.from_dict(json.load(handle))

    def to_dict(self) -> dict:
        return {
            "resolution": self.resolution,
            "origin": list(self.origin),
            "width": self.width,
            "height": self.height,
            "data": base64.b64encode(zlib.compress(bytes(self.cells), 1)).decode(),
        }

    # ---- indexing -----------------------------------------------------

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def cell_to_world(self, cx: int, cy: int) -> tuple[float, float]:
        """Centre of the cell, not its corner."""
        return (self.origin[0] + (cx + 0.5) * self.resolution,
                self.origin[1] + (cy + 0.5) * self.resolution)

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height

    def at(self, cx: int, cy: int) -> int:
        if not self.in_bounds(cx, cy):
            return OCCUPIED
        return self.cells[cy * self.width + cx]

    def set_cell(self, cx: int, cy: int, value: int) -> None:
        if self.in_bounds(cx, cy):
            self.cells[cy * self.width + cx] = value

    def is_occupied(self, x: float, y: float) -> bool:
        """UNKNOWN counts as free — an unmapped cell is not a wall."""
        return self.at(*self.world_to_cell(x, y)) == OCCUPIED

    # ---- authoring helpers (tests and generated maps) -------------------

    def fill_rect(self, x0: float, y0: float, x1: float, y1: float, value: int = OCCUPIED) -> None:
        cx0, cy0 = self.world_to_cell(min(x0, x1), min(y0, y1))
        cx1, cy1 = self.world_to_cell(max(x0, x1), max(y0, y1))
        for cy in range(cy0, cy1 + 1):
            for cx in range(cx0, cx1 + 1):
                self.set_cell(cx, cy, value)

    def border(self, value: int = OCCUPIED) -> None:
        for cx in range(self.width):
            self.set_cell(cx, 0, value)
            self.set_cell(cx, self.height - 1, value)
        for cy in range(self.height):
            self.set_cell(0, cy, value)
            self.set_cell(self.width - 1, cy, value)

    # ---- queries ------------------------------------------------------

    def segment_blocked(self, x0: float, y0: float, x1: float, y1: float, radius: float = 0.0) -> bool:
        """Sample the swept segment at half-cell steps, widened by ``radius``.

        Half-cell rather than full-cell because a full-cell step can straddle a
        one-cell-thick wall and miss it entirely — which reads as the robot
        walking through a wall while every log stays clean.
        """
        length = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, int(math.ceil(length / (self.resolution * 0.5))))
        nx, ny = (0.0, 0.0)
        if length > 1e-9:
            nx, ny = (-(y1 - y0) / length, (x1 - x0) / length)  # unit normal
        offsets = [0.0] if radius <= 0 else [-radius, 0.0, radius]
        for i in range(steps + 1):
            t = i / steps
            px = x0 + (x1 - x0) * t
            py = y0 + (y1 - y0) * t
            for off in offsets:
                if self.is_occupied(px + nx * off, py + ny * off):
                    return True
        return False

    def raycast(self, x: float, y: float, theta: float, max_range: float) -> float:
        """Distance to the first occupied cell, or ``max_range`` if none."""
        step = self.resolution * 0.5
        dx, dy = math.cos(theta) * step, math.sin(theta) * step
        steps = int(max_range / step)
        px, py = x, y
        for i in range(1, steps + 1):
            px += dx
            py += dy
            if self.is_occupied(px, py):
                return i * step
        return max_range
