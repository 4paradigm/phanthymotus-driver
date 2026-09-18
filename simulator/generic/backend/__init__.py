"""World backends.

One interface, two implementations, and the cards never learn which one they are
talking to — the same shape `VLAProvider` uses to hide whether a policy is an
in-process SmolVLA or a machine-room server. Repo split is not abstraction split.

* `LocalBackend` — unicycle integration plus grid raycasting, in-process, on the
  robot. This is Stage 0/1. Stage 1 swaps only the collision query for a real
  FK/collision runtime; the interface does not move.
* `RemoteBackend` — Stage 2, served from `phanthymotus-cloud`, streaming state
  down rather than answering per-tick requests. Not implemented yet.

Three preconditions hold from day one so that swapping is a swap and not a
rewrite:

1. **No card integrates time.** Only a backend advances the world.
2. **`step()` must run faster than real time** — hence the injected clock.
3. **Sensor payloads leave `sense()` as arrays, not wire format.** The `<fffBI`
   packing and the zlib depth encoding live in the cards, so a physics backend
   changes where the numbers come from and not how they are framed.
"""

from __future__ import annotations

import math

from simulator.generic.geometry import OccupancyGrid, Pose, normalize_angle


class WorldBackend:
    """Interface. See module docstring for the contract that makes it swappable."""

    def reset(self, scene: dict) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def apply(self, cmd: dict) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def step(self, dt: float) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def state(self) -> dict:  # pragma: no cover - interface
        raise NotImplementedError

    def sense(self, kinds: list[str]) -> dict:  # pragma: no cover - interface
        raise NotImplementedError

    def health(self) -> bool:  # pragma: no cover - interface
        raise NotImplementedError

    def close(self) -> None:  # pragma: no cover - interface
        raise NotImplementedError


class LocalBackend(WorldBackend):
    """Unicycle base + interpolated joints + grid raycast.

    Deterministic on purpose. A failed tour then means the orchestration was
    wrong, not that the base slipped — which is the whole reason Stage 0 does not
    use a physics engine.
    """

    GRAVITY = 9.80665

    def __init__(self, scene: dict | None = None):
        self._grid = OccupancyGrid.blank()
        self._pose = Pose()
        self._joints: list[float] = []
        self._joint_targets: list[float] = []
        self._joint_speed = 1.0        # rad/s
        self._radius = 0.25            # m, footprint for collision
        self._lin_cmd = 0.0
        self._ang_cmd = 0.0
        self._lin = 0.0                # actual, after accel limiting
        self._ang = 0.0
        self._max_lin = 0.5
        self._max_ang = 0.6
        self._accel = 0.4
        self._ang_accel = 1.2
        self._contact: str | None = None
        self._odometer = 0.0
        self._scan = {"n_beams": 60, "fov": math.radians(270.0), "max_range": 8.0}
        self._closed = False
        if scene:
            self.reset(scene)

    # ---- lifecycle ----------------------------------------------------

    def reset(self, scene: dict) -> None:
        grid = scene.get("grid")
        if isinstance(grid, OccupancyGrid):
            self._grid = grid
        elif isinstance(grid, dict):
            self._grid = OccupancyGrid.from_dict(grid)

        spawn = scene.get("spawn") or {}
        self._pose = Pose(float(spawn.get("x", 0.0)), float(spawn.get("y", 0.0)),
                          float(spawn.get("yaw", 0.0)))

        motion = scene.get("motion") or {}
        self._max_lin = float(motion.get("max_lin", self._max_lin))
        self._max_ang = float(motion.get("max_ang", self._max_ang))
        self._accel = float(motion.get("accel", self._accel))
        self._ang_accel = float(motion.get("ang_accel", self._ang_accel))
        self._radius = float(motion.get("radius", self._radius))

        dof = int(scene.get("dof", len(self._joints)))
        initial = list(scene.get("joints") or [0.0] * dof)
        self._joints = [float(v) for v in initial]
        self._joint_targets = list(self._joints)
        self._joint_speed = float(scene.get("joint_speed", self._joint_speed))

        self._scan.update(scene.get("scan") or {})
        self._lin = self._ang = self._lin_cmd = self._ang_cmd = 0.0
        self._contact = None
        self._odometer = 0.0

    def health(self) -> bool:
        return not self._closed

    def close(self) -> None:
        self._closed = True

    # ---- command ------------------------------------------------------

    def apply(self, cmd: dict) -> None:
        if "lin" in cmd or "ang" in cmd:
            self._lin_cmd = _clamp(float(cmd.get("lin", 0.0)), -self._max_lin, self._max_lin)
            self._ang_cmd = _clamp(float(cmd.get("ang", 0.0)), -self._max_ang, self._max_ang)
        if cmd.get("joints") is not None:
            target = [float(v) for v in cmd["joints"]]
            if len(target) != len(self._joints):
                raise ValueError(f"joint command has {len(target)} values, embodiment has {len(self._joints)}")
            self._joint_targets = target

    # ---- integrate ----------------------------------------------------

    def step(self, dt: float) -> None:
        if dt <= 0:
            return
        self._lin = _approach(self._lin, self._lin_cmd, self._accel * dt)
        self._ang = _approach(self._ang, self._ang_cmd, self._ang_accel * dt)

        self._contact = None
        if abs(self._lin) > 1e-9:
            nx = self._pose.x + math.cos(self._pose.yaw) * self._lin * dt
            ny = self._pose.y + math.sin(self._pose.yaw) * self._lin * dt
            # Extend the swept segment forward by the footprint radius as well as
            # widening it: `radius` alone is perpendicular to travel, which models
            # the robot's width but gives it no length — so it would stop only
            # once its *centre* entered the wall.
            lead = self._radius * _sign(self._lin)
            ex = nx + math.cos(self._pose.yaw) * lead
            ey = ny + math.sin(self._pose.yaw) * lead
            if self._grid.segment_blocked(self._pose.x, self._pose.y, ex, ey, self._radius):
                # Hard stop against geometry. The nav controller turns this into a
                # failed job; nothing here knows what a job is.
                self._contact = "base"
                self._lin = 0.0
                self._lin_cmd = 0.0
            else:
                self._odometer += abs(self._lin) * dt
                self._pose.x, self._pose.y = nx, ny
        if abs(self._ang) > 1e-9:
            self._pose.yaw = normalize_angle(self._pose.yaw + self._ang * dt)

        for i, target in enumerate(self._joint_targets):
            delta = target - self._joints[i]
            limit = self._joint_speed * dt
            self._joints[i] += _clamp(delta, -limit, limit)

    # ---- observe ------------------------------------------------------

    def state(self) -> dict:
        return {
            "pose": self._pose.copy(),
            "joints": list(self._joints),
            "joint_targets": list(self._joint_targets),
            "lin": self._lin,
            "ang": self._ang,
            "contact": self._contact,
            "odometer": self._odometer,
            "grid": self._grid,
        }

    def sense(self, kinds: list[str]) -> dict:
        out: dict = {}
        for kind in kinds:
            if kind == "scan":
                out["scan"] = self._sense_scan()
            elif kind == "imu":
                out["imu"] = self._sense_imu()
            elif kind == "joints":
                out["joints"] = list(self._joints)
            elif kind == "pose":
                out["pose"] = self._pose.copy()
        return out

    def _sense_scan(self) -> dict:
        n = int(self._scan["n_beams"])
        fov = float(self._scan["fov"])
        max_range = float(self._scan["max_range"])
        start = -fov / 2.0
        increment = fov / (n - 1) if n > 1 else 0.0
        ranges = [
            self._grid.raycast(self._pose.x, self._pose.y,
                               normalize_angle(self._pose.yaw + start + increment * i), max_range)
            for i in range(n)
        ]
        return {"angle_min": start, "angle_max": start + increment * (n - 1),
                "angle_increment": increment, "range_max": max_range, "ranges": ranges}

    def _sense_imu(self) -> dict:
        """Derived from the integrator, never randomised.

        Sensors that disagree with the world are worse than no sensors — a test
        that passes against noise proves nothing.
        """
        return {
            "linear_acceleration": {"x": self._lin_cmd - self._lin, "y": 0.0, "z": self.GRAVITY},
            "angular_velocity": {"x": 0.0, "y": 0.0, "z": self._ang},
            "orientation": {"roll": 0.0, "pitch": 0.0, "yaw": self._pose.yaw},
        }


def _clamp(value: float, low: float, high: float) -> float:
    return low if value < low else high if value > high else value


def _sign(value: float) -> float:
    return 0.0 if value == 0 else (1.0 if value > 0 else -1.0)


def _approach(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + (max_delta if delta > 0 else -max_delta)
