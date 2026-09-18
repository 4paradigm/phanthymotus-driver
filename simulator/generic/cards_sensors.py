"""Sensor cards.

Every one of these derives from the same world state, so the map, the odometry
and the laser scan cannot contradict each other. Wire framing lives here rather
than in `sensors.py` — that split is what lets a physics backend change where the
numbers come from without touching how they are packed.

See `card_base.py` for why `laser_scan` and `imu` publish `data/json` rather than
`sensor/lidar` and `sensor/imu`.
"""

from __future__ import annotations

import json
import math
import struct

from simulator.generic import sensors
from simulator.generic.card_base import Card


class OdomCard(Card):
    NAME = "odom"
    KIND = "sensor"
    DESCRIPTION = "虚拟底盘里程计 — 位姿、速度、累计里程"
    TOPIC = "odom"
    FORMAT = "data/json"
    HZ = 5.0
    ACTIONS = {"read": ([], "读取当前位姿与速度")}

    def payload(self):
        return sensors.odom(self.world.snapshot())

    def do_read(self, **_):
        return {"state": "running" if self._running else "idle", **self.payload()}


class ImuCard(Card):
    NAME = "imu"
    KIND = "sensor"
    DESCRIPTION = "虚拟 IMU — 由积分器导出的加速度与角速度，非随机数"
    TOPIC = "imu"
    FORMAT = "data/json"
    HZ = 10.0
    ACTIONS = {"read": ([], "读取当前 IMU 值")}

    def payload(self):
        snapshot = self.world.snapshot()
        return sensors.imu(self.world._backend.sense(["imu"]), snapshot["t"])  # noqa: SLF001

    def do_read(self, **_):
        return {"state": "running" if self._running else "idle", **self.payload()}


class LaserScanCard(Card):
    NAME = "laser_scan"
    KIND = "sensor"
    DESCRIPTION = "虚拟激光雷达 — 从占用栅格射线投射，与地图严格一致"
    TOPIC = "scan"
    FORMAT = "data/json"
    HZ = 5.0
    ACTIONS = {"read": ([], "读取一帧扫描")}

    def payload(self):
        snapshot = self.world.snapshot()
        return sensors.laser_scan(self.world._backend.sense(["scan"]), snapshot["t"])  # noqa: SLF001

    def do_read(self, **_):
        payload = self.payload()
        # The full beam list is for the panel, not for an LLM prompt; a reply
        # carrying 60 floats crowds out the thing the model is reasoning about.
        ranges = payload.pop("ranges", [])
        payload["min_range"] = round(min(ranges), 3) if ranges else None
        payload["nearest_bearing"] = (
            round(payload["angle_min"] + payload["angle_increment"] * ranges.index(min(ranges)), 3)
            if ranges else None)
        return {"state": "running" if self._running else "idle", **payload}


class BatteryCard(Card):
    NAME = "battery"
    KIND = "sensor"
    DESCRIPTION = "虚拟电池 — 随时间与里程下降，可脚本化触发低电"
    TOPIC = "battery"
    FORMAT = "data/json"
    HZ = 0.5
    ACTIONS = {"read": ([], "读取电量")}

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._battery_cfg = (config or {}).get("battery", {})
        self._epoch = None

    def start(self) -> None:
        if self._epoch is None:
            self._epoch = self.world.snapshot()["t"]
        super().start()

    def payload(self):
        snapshot = self.world.snapshot()
        epoch = self._epoch if self._epoch is not None else snapshot["t"]
        return sensors.battery(snapshot["t"] - epoch, snapshot["odometer"], self._battery_cfg)

    def do_read(self, **_):
        return {"state": "running" if self._running else "idle", **self.payload()}


class MapCard(Card):
    """`sensor/mapping` — occupancy grid, travelled path and waypoints as one point set.

    The wire format is not negotiable and is not self-describing, so it is
    written out explicitly here:

        <fffBI   robot_x, robot_y, display_yaw, flags, n_points
        float32[n][3]                                  the points
        <I + bytes                                     JSON metadata trailer

    Two traps, both silent:

    * ``display_yaw`` is the **negated** heading. The renderer's world is
      y-flipped relative to the robot frame, so an un-negated yaw points the
      cone the wrong way and nothing complains.
    * ``flags`` must have **bit 1 (`has_z`) set**. `mapping.js` sniffs the
      protocol version by testing `flags & 0x02`; with it clear it silently
      takes the legacy branch and the panel stays blank with nothing in any log.
      7 sets bits 0, 1 and 2, which is what the real card sends.
    """

    NAME = "map"
    KIND = "sensor"
    DESCRIPTION = "虚拟建图 — 占用栅格 + 已走轨迹 + 航点标记"
    TOPIC = "map"
    FORMAT = "sensor/mapping"
    HZ = 2.0
    BINARY = True
    ACTIONS = {"read": ([], "读取地图摘要（尺寸、分辨率、点数），不含点云本体")}

    MAX_POINTS = 80000
    FLAGS = 7                      # bit0 full_map | bit1 has_z | bit2 reserved
    Z_GRID = 0.0
    Z_TRAIL = 0.16
    Z_WAYPOINT = 0.28

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._waypoints_provider = None
        self._grid_cache: tuple[int, int, list] | None = None

    def set_waypoints_provider(self, fn) -> None:
        """Wired by the scenario card; without one the map simply has no markers."""
        self._waypoints_provider = fn

    def _grid_points(self, grid, budget: int) -> list[tuple[float, float, float]]:
        """Cached: the occupancy scan is O(cells) in Python and the grid only
        changes on a scenario reset. A 40x40 m map at 5 cm is 640k cells, which
        at the 2 Hz publish rate would burn a fifth of a core for a picture that
        never changes."""
        key = (id(grid), budget)
        if self._grid_cache is not None and self._grid_cache[:2] == key:
            return self._grid_cache[2]
        points = sensors.occupancy_points(grid, budget, self.Z_GRID)
        self._grid_cache = (*key, points)
        return points

    # ---- point assembly ------------------------------------------------

    def _waypoints(self) -> list[dict]:
        if self._waypoints_provider is None:
            return []
        try:
            return list(self._waypoints_provider() or [])
        except Exception:
            return []

    def points(self) -> list[tuple[float, float, float]]:
        grid = self.world._backend.state()["grid"]  # noqa: SLF001
        waypoints = self._waypoints()
        markers = sensors.marker_points([(w["x"], w["y"]) for w in waypoints], self.Z_WAYPOINT)
        trail = [(x, y, self.Z_TRAIL) for x, y in self.world.trail()]
        # Budget from the top: markers and trail are small and carry the most
        # meaning, so the grid is what gets decimated when the map is large.
        budget = max(0, self.MAX_POINTS - len(markers) - len(trail))
        # Quantise the budget so a trail growing one point at a time does not
        # invalidate the grid cache on every single frame.
        budget = (budget // 1000) * 1000
        return self._grid_points(grid, budget) + trail + markers

    def encode(self, points: list[tuple[float, float, float]], pose: dict, meta: dict) -> bytes:
        flat: list[float] = []
        for x, y, z in points:
            flat.extend((x, y, z))
        raw_meta = json.dumps(meta, ensure_ascii=False).encode()
        return (struct.pack("<fffBI", pose["x"], pose["y"], -pose["yaw"], self.FLAGS, len(points))
                + struct.pack(f"<{len(flat)}f", *flat)
                + struct.pack("<I", len(raw_meta)) + raw_meta)

    def payload(self):
        snapshot = self.world.snapshot()
        grid = self.world._backend.state()["grid"]  # noqa: SLF001
        points = self.points()
        waypoints = self._waypoints()
        meta = {
            "version": 3,
            "robot": {**snapshot["pose"], "pose_available": True},
            "resolution": grid.resolution,
            "origin": list(grid.origin),
            "size": [grid.width, grid.height],
            "grid_points": len(points),
            "trajectory_points": len(self.world.trail()),
            "tags": [{"name": w.get("name", ""), "x": w["x"], "y": w["y"]} for w in waypoints],
            "simulated": True,
        }
        return self.encode(points, snapshot["pose"], meta)

    def do_read(self, **_):
        grid = self.world._backend.state()["grid"]  # noqa: SLF001
        return {
            "state": "running" if self._running else "idle",
            "resolution": grid.resolution,
            "origin": list(grid.origin),
            "size": [grid.width, grid.height],
            "extent_m": [round(grid.width * grid.resolution, 2), round(grid.height * grid.resolution, 2)],
            "trail_points": len(self.world.trail()),
            "waypoints": [w.get("name", "") for w in self._waypoints()],
        }


class ModelCard(Card):
    """The URDF the skeleton renderer needs.

    `resource` rather than `sensor` so it is barrier-exempt — the joint renderer
    asks for it while a 90 second navigation is pending, and an actuator-typed
    model card would queue the whole canvas behind the tour.
    """

    NAME = "model"
    KIND = "resource"
    DESCRIPTION = "虚拟具身的 URDF — sensor/skeleton 渲染的硬依赖"
    TOPIC = ""

    def __init__(self, world, config, namespace, ros2=None):
        super().__init__(world, config, namespace, ros2)
        self._embodiment = (config or {}).get("embodiment", {})

    def dispatch(self, action: str, args: dict) -> dict:
        # A resource tool is called by its own name; vendor_runtime does not pop
        # an `action` for this kind.
        urdf = self._embodiment.get("urdf_inline") or _fallback_urdf(self.joint_names())
        return {"urdf": urdf, "joint_names": self.joint_names()}

    def joint_names(self) -> list[str]:
        names = list(self._embodiment.get("joint_names") or [])
        if names:
            return names
        return [f"joint_{index}" for index in range(int(self._embodiment.get("dof", 0)))]


def _fallback_urdf(joint_names: list[str]) -> str:
    """A serial chain whose joint names match `joint_names` character for character.

    The skeleton renderer matches by name; one mismatched name draws nothing and
    reports nothing, so the URDF is generated from the same list the `joints`
    card publishes rather than kept in a file that can drift from it.
    """
    links = ['<link name="base_link"/>']
    joints = []
    for index, name in enumerate(joint_names):
        child = f"link_{index + 1}"
        parent = "base_link" if index == 0 else f"link_{index}"
        links.append(f'<link name="{child}"/>')
        joints.append(
            f'<joint name="{name}" type="revolute">'
            f'<parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="0 0 {0.15:.2f}" rpy="0 0 0"/><axis xyz="0 0 1"/>'
            f'<limit lower="{-math.pi}" upper="{math.pi}" effort="10" velocity="2"/>'
            f"</joint>")
    return ('<?xml version="1.0"?>\n<robot name="sim_generic">\n  '
            + "\n  ".join(links + joints) + "\n</robot>\n")
