"""Map sensor for Tianyi controlled_spatial.

The Slamtec grid, laser scan, navigation pose, POIs and artifacts share one
coordinate system.  This plugin publishes that coordinate system as the stock
``sensor/mapping`` packet and persists a render cache for every saved map.
"""

from __future__ import annotations

import json
import math
import os
import re
import sqlite3
import struct
import threading
import time
from array import array
from collections import deque


_AREA_TYPES = (
    "forbidden_area", "elevator_area", "dangerous_area", "coverage_area",
    "maintenance_area", "sensor_disable_area", "restricted_area",
)


class _MapDB:
    def __init__(self, path: str):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self._conn = sqlite3.connect(path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._lock = threading.RLock()
        with self._lock:
            self._conn.executescript("""
                CREATE TABLE IF NOT EXISTS maps (
                    name TEXT PRIMARY KEY, pcd_path TEXT NOT NULL,
                    created_at REAL DEFAULT (strftime('%s','now'))
                );
                CREATE TABLE IF NOT EXISTS poi (
                    id INTEGER PRIMARY KEY AUTOINCREMENT, name TEXT NOT NULL,
                    description TEXT DEFAULT '', x REAL NOT NULL, y REAL NOT NULL,
                    yaw REAL DEFAULT 0, map_name TEXT NOT NULL,
                    created_at REAL DEFAULT (strftime('%s','now')),
                    UNIQUE(name, map_name)
                );
                CREATE TABLE IF NOT EXISTS state (
                    key TEXT PRIMARY KEY, value TEXT,
                    updated_at REAL DEFAULT (strftime('%s','now'))
                );
            """)
            columns = {r["name"] for r in self._conn.execute("PRAGMA table_info(maps)")}
            if "visual_path" not in columns:
                self._conn.execute("ALTER TABLE maps ADD COLUMN visual_path TEXT")
            self._conn.commit()

    def maps(self):
        with self._lock:
            rows = self._conn.execute(
                "SELECT name, pcd_path, visual_path, created_at FROM maps ORDER BY created_at DESC"
            ).fetchall()
            result = [dict(r) for r in rows]
            for item in result:
                item["tags"] = [dict(r) for r in self._conn.execute(
                    "SELECT name, description, x, y, yaw FROM poi WHERE map_name=? ORDER BY name",
                    (item["name"],),
                ).fetchall()]
            return result

    def state(self, key: str):
        with self._lock:
            row = self._conn.execute("SELECT value FROM state WHERE key=?", (key,)).fetchone()
            return row["value"] if row else None

    def ensure_map(self, name: str, path: str, visual_path: str):
        with self._lock:
            self._conn.execute(
                "INSERT OR IGNORE INTO maps (name, pcd_path, visual_path) VALUES (?, ?, ?)",
                (name, path, visual_path),
            )
            self._conn.execute("UPDATE maps SET visual_path=? WHERE name=?", (visual_path, name))
            self._conn.commit()


class ControlledSpatialMapPlugin:
    PREFIX = "controlled_spatial_map"
    MAX_POINTS = 40000
    VOXEL = 0.06

    def __init__(self, config: dict, namespace: str, ros2, slamtec_client):
        import numpy as np
        from rclpy.node import Node
        from std_msgs.msg import UInt8MultiArray

        self._np = np
        self._slamtec = slamtec_client
        self._db = _MapDB(config.get("native_slam_db_path", "/data/controlled_spatial/controlled_spatial.db"))
        self._cache_dir = config.get("cache_dir", "/data/controlled_spatial/map-visuals")
        os.makedirs(self._cache_dir, exist_ok=True)
        self._topic = f"/{namespace}/controlled_spatial/map"
        self._poll_hz = max(0.5, float(config.get("poll_hz", 3.0)))
        self._grid_hz = max(0.1, float(config.get("grid_hz", 0.7)))
        self._artifact_hz = max(0.1, float(config.get("artifact_hz", 0.4)))
        # Some Slamtec firmwares expose explore-map cells in a different grid
        # frame from localization. Keep the correction deployment-configurable.
        self._grid_flip_x = bool(config.get("grid_flip_x", False))
        self._grid_flip_y = bool(config.get("grid_flip_y", False))
        self._grid_x_offset = float(config.get("grid_x_offset", 0.0))
        self._grid_y_offset = float(config.get("grid_y_offset", 0.0))
        self._sub_node = Node("tianyi_controlled_spatial_map_sub", context=ros2.ctx_tianyi)
        self._pub_node = Node("tianyi_controlled_spatial_map_pub", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(UInt8MultiArray, self._topic, 10)
        self._lock = threading.RLock()
        self._running = False
        self._thread = None
        self._selected = None
        self._current_map = None
        self._runtime_active = None
        self._recording = None
        self._pose = None
        self._grid = np.zeros((0, 3), dtype=np.float32)
        self._grid_bounds = None
        self._lasers = {}
        self._laser_hits = {}
        self._trajectory = deque(maxlen=6000)
        self._artifacts = {"walls": [], "tracks": [], "areas": {}}
        self._last_grid = 0.0
        self._last_artifacts = 0.0
        self._last_cache = 0.0
        self._last_publish = 0.0
        self._startup_error = None

    def get_tools(self):
        return [self.get_tool()]

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "sensor", "multiInstance": False,
            "description": "Tianyi controlled map: grid, boundary, laser, route, tags and special areas.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": [
                    "list_maps", "select_map",
                ]},
                "map_name": {"type": "string"},
            }},
            "topic_out": [{"topic": self._topic, "format": "sensor/mapping"}],
        }

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._poll, name="tianyi_spatial_map", daemon=True)
        self._thread.start()
        print(f"[ControlledSpatialMap] ready: {self._topic}")

    def stop(self):
        self._save_cache(force=True)
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        for node in (self._sub_node, self._pub_node):
            try:
                node.destroy_node()
            except Exception:
                pass

    def dispatch(self, action: str, args: dict):
        if action in (self.PREFIX, "start", "refresh", "info"):
            self._publish(force=True)
            return self._info("ready")
        if action == "stop":
            return self._info("idle")
        if action == "list_maps":
            return {"maps": self._db.maps(), **self._info("ready")}
        if action == "select_map":
            name = args.get("map_name")
            found = next((m for m in self._db.maps() if m["name"] == name), None)
            if not found:
                return {"error": "map_name not found"}
            with self._lock:
                if self._recording and self._recording != name:
                    return {"error": "cannot select another map while recording"}
                self._selected = name
                self._load_cache(found)
            self._publish(force=True)
            return self._info("selected")
        if action in ("save_map", "record_map", "stop_record_map"):
            return {
                "error": "manual visual recording is not supported; use "
                         "controlled_spatial.start_mapping and stop_mapping"
            }
        return {"error": f"unknown action: {action}"}

    def _poll(self):
        while self._running:
            try:
                self._sync_map()
                pose = self._slamtec.get_pose()
                if isinstance(pose, dict) and not pose.get("error") and "x" in pose:
                    with self._lock:
                        self._pose = {k: float(pose.get(k, 0.0)) for k in ("x", "y", "yaw")}
                        if not self._runtime_active or self._current_map == self._runtime_active:
                            self._append_pose(self._pose)
                scan = self._slamtec.get_laser_scan()
                if isinstance(scan, dict) and not scan.get("error"):
                    self._consume_scan(scan)
                now = time.monotonic()
                if now - self._last_grid >= 1.0 / self._grid_hz:
                    raw, error = self._slamtec.get_explore_map()
                    if raw and not error:
                        self._consume_grid(raw)
                    self._last_grid = now
                if now - self._last_artifacts >= 1.0 / self._artifact_hz:
                    self._refresh_artifacts()
                    self._last_artifacts = now
                self._save_cache()
                self._publish()
            except Exception as exc:
                self._startup_error = str(exc)
            time.sleep(1.0 / self._poll_hz)

    def _sync_map(self):
        active = self._db.state("active_map") or None
        status = self._db.state("map_status") or "idle"
        maps = self._db.maps()
        if status == "mapping" and active:
            desired = active
        else:
            desired = self._selected or active or (maps[0]["name"] if maps else None)
        with self._lock:
            self._runtime_active = active
            if desired and desired != self._current_map:
                found = next((m for m in maps if m["name"] == desired), None)
                if found:
                    self._current_map = desired
                    self._selected = desired
                    if desired != active or status != "mapping":
                        self._load_cache(found)
                    else:
                        self._reset_buffers()
            if status == "mapping" and active:
                self._recording = active
            elif self._recording:
                self._save_cache(force=True)
                self._recording = None

    def _consume_grid(self, raw: bytes):
        with self._lock:
            if self._runtime_active and self._current_map != self._runtime_active:
                return
        if len(raw) < 36:
            return
        min_x, min_y, width, height, resolution = struct.unpack_from("<ffIIf", raw, 0)
        length = struct.unpack_from("<I", raw, 32)[0]
        cells = self._np.frombuffer(raw, dtype=self._np.uint8, offset=36, count=min(length, len(raw) - 36))
        if len(cells) != width * height or width == 0 or height == 0:
            return
        # 0 is unobserved on this device.  127 is the dominant observed floor;
        # non-127 values are retained as a denser relief layer for boundaries.
        known = self._np.flatnonzero(cells != 0)
        if not len(known):
            return
        limit = 26000
        if len(known) > limit:
            known = known[::max(1, len(known) // limit)][:limit]
        rows, cols = known // width, known % width
        values = cells[known]
        z = self._np.where(values == 127, -0.03, 0.04).astype(self._np.float32)
        max_x = min_x + width * resolution
        max_y = min_y + height * resolution
        grid_x = (max_x - (cols.astype(self._np.float32) + 0.5) * resolution
                  if self._grid_flip_x else
                  min_x + (cols.astype(self._np.float32) + 0.5) * resolution)
        grid_y = (max_y - (rows.astype(self._np.float32) + 0.5) * resolution
                  if self._grid_flip_y else
                  min_y + (rows.astype(self._np.float32) + 0.5) * resolution)
        points = self._np.column_stack((
            grid_x + self._grid_x_offset,
            grid_y + self._grid_y_offset,
            z,
        )).astype(self._np.float32)
        with self._lock:
            self._grid = points
            self._grid_bounds = (
                min_x + self._grid_x_offset,
                min_y + self._grid_y_offset,
                width * resolution,
                height * resolution,
            )

    def _consume_scan(self, scan: dict):
        pose = scan.get("pose") or self._pose
        if not isinstance(pose, dict):
            return
        px, py, yaw = (float(pose.get(k, 0.0)) for k in ("x", "y", "yaw"))
        frame = {}
        for point in scan.get("laser_points", []):
            if not point.get("valid"):
                continue
            distance = float(point.get("distance", 0.0))
            if not 0.08 <= distance <= 12.0:
                continue
            angle = yaw + float(point.get("angle", 0.0))
            x, y = px + distance * math.cos(angle), py + distance * math.sin(angle)
            frame[(round(x / self.VOXEL), round(y / self.VOXEL))] = (x, y, 0.09)
        with self._lock:
            if self._recording:
                for key, value in frame.items():
                    if key in self._lasers:
                        self._lasers[key] = value
                    else:
                        hits = self._laser_hits.get(key, 0) + 1
                        if hits >= 2:
                            self._lasers[key] = value
                            self._laser_hits.pop(key, None)
                        else:
                            self._laser_hits[key] = hits

    def _append_pose(self, pose):
        if not self._trajectory:
            self._trajectory.append((pose["x"], pose["y"], pose["yaw"]))
            return
        old = self._trajectory[-1]
        if math.hypot(pose["x"] - old[0], pose["y"] - old[1]) > 0.025 or abs(pose["yaw"] - old[2]) > 0.04:
            self._trajectory.append((pose["x"], pose["y"], pose["yaw"]))

    def _refresh_artifacts(self):
        artifacts = {"walls": self._list_result(self._slamtec.get_lines("walls")),
                     "tracks": self._list_result(self._slamtec.get_lines("tracks")), "areas": {}}
        for kind in _AREA_TYPES:
            result = self._slamtec.get_rectangle_areas(kind)
            # Some firmware returns HTTP 500 for an empty coverage_area.
            artifacts["areas"][kind] = self._list_result(result)
        with self._lock:
            self._artifacts = artifacts

    @staticmethod
    def _list_result(result):
        if isinstance(result, list):
            return result
        if not isinstance(result, dict) or result.get("error"):
            return []
        raw = result.get("raw", result.get("data", result.get("items", [])))
        return raw if isinstance(raw, list) else []

    def _publish(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_publish < 0.35:
            return
        self._last_publish = now
        with self._lock:
            grid = self._grid.copy(); lasers = list(self._lasers.values()); trajectory = list(self._trajectory)
            bounds = self._grid_bounds; pose = dict(self._pose) if self._pose else None
            current_map = self._current_map; artifacts = json.loads(json.dumps(self._artifacts))
        maps = self._db.maps()
        tags = next((m["tags"] for m in maps if m["name"] == current_map), [])
        overlays = self._boundary_points(bounds) + self._trajectory_points(trajectory)
        overlays += self._line_points(artifacts["walls"], 0.20) + self._line_points(artifacts["tracks"], 0.13)
        for index, (_, areas) in enumerate(artifacts["areas"].items()):
            overlays += self._area_points(areas, 0.12 + index * 0.025)
        overlays += self._tag_points(tags)
        cloud = self._np.asarray(lasers + overlays, dtype=self._np.float32) if lasers or overlays else self._np.zeros((0, 3), dtype=self._np.float32)
        points = self._np.vstack((grid, cloud)) if len(grid) else cloud
        if len(points) > self.MAX_POINTS:
            points = points[::max(1, len(points) // self.MAX_POINTS)][:self.MAX_POINTS]
        robot = pose or {"x": 0.0, "y": 0.0, "yaw": 0.0}
        meta = {"version": 3, "active_map": current_map, "robot": {**robot, "pose_available": pose is not None},
                "maps": maps, "tags": tags, "boundary": bounds, "artifacts": artifacts,
                "trajectory_points": len(trajectory), "laser_points": len(lasers), "grid_points": len(grid)}
        raw_meta = json.dumps(meta, ensure_ascii=False).encode()
        payload = struct.pack("<fffBI", robot["x"], robot["y"], robot["yaw"], 7, len(points)) + points.tobytes()
        payload += struct.pack("<I", len(raw_meta)) + raw_meta
        from std_msgs.msg import UInt8MultiArray
        out = UInt8MultiArray(); out.data = array("B", payload); self._pub.publish(out)

    @staticmethod
    def _sample_line(a, b, z, step=0.06):
        x1, y1 = float(a.get("x", 0)), float(a.get("y", 0)); x2, y2 = float(b.get("x", 0)), float(b.get("y", 0))
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / step))
        return [(x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n, z) for i in range(n + 1)]

    def _boundary_points(self, bounds):
        if not bounds:
            return []
        x, y, w, h = bounds
        return self._sample_line({"x": x, "y": y}, {"x": x + w, "y": y}, 0.12) + self._sample_line({"x": x + w, "y": y}, {"x": x + w, "y": y + h}, 0.12) + self._sample_line({"x": x + w, "y": y + h}, {"x": x, "y": y + h}, 0.12) + self._sample_line({"x": x, "y": y + h}, {"x": x, "y": y}, 0.12)

    def _trajectory_points(self, values):
        return [(x, y, 0.16) for x, y, _ in values]

    def _line_points(self, lines, z):
        result = []
        for line in lines:
            if isinstance(line, dict) and isinstance(line.get("start"), dict) and isinstance(line.get("end"), dict):
                result += self._sample_line(line["start"], line["end"], z)
        return result

    def _area_points(self, areas, z):
        result = []
        for item in areas:
            area = item.get("area", item) if isinstance(item, dict) else {}
            start, end = area.get("start"), area.get("end")
            if not isinstance(start, dict) or not isinstance(end, dict):
                continue
            dx, dy = float(end["x"]) - float(start["x"]), float(end["y"]) - float(start["y"])
            length = math.hypot(dx, dy)
            half = float(area.get("half_width", 0.12))
            if length < 1e-5:
                continue
            nx, ny = -dy / length * half, dx / length * half
            corners = [{"x": float(start["x"]) + nx, "y": float(start["y"]) + ny}, {"x": float(end["x"]) + nx, "y": float(end["y"]) + ny}, {"x": float(end["x"]) - nx, "y": float(end["y"]) - ny}, {"x": float(start["x"]) - nx, "y": float(start["y"]) - ny}]
            for i in range(4):
                result += self._sample_line(corners[i], corners[(i + 1) % 4], z)
        return result

    def _tag_points(self, tags):
        result = []
        for tag in tags:
            try:
                x, y, yaw = float(tag["x"]), float(tag["y"]), float(tag.get("yaw", 0))
            except (KeyError, TypeError, ValueError):
                continue
            for radius in (0.12, 0.22):
                result += [(x + radius * math.cos(i * math.pi / 18), y + radius * math.sin(i * math.pi / 18), 0.35) for i in range(36)]
            result += [(x + d * math.cos(yaw), y + d * math.sin(yaw), 0.55) for d in self._np.linspace(0, 0.9, 22)]
        return result

    def _save_cache(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_cache < 5:
            return False
        with self._lock:
            name = self._recording or self._current_map
            if not name or (not self._recording and not force):
                return False
            path = self._cache_path(name)
            self._db.ensure_map(name, os.path.join(self._cache_dir, f"controlled_{self._safe(name)}.stcm"), path)
            self._np.savez_compressed(path, grid=self._grid, lasers=self._np.asarray(list(self._lasers.values()), dtype=self._np.float32), trajectory=self._np.asarray(self._trajectory, dtype=self._np.float32), bounds=self._np.asarray(self._grid_bounds or [], dtype=self._np.float32), artifacts=json.dumps(self._artifacts))
            self._last_cache = now
            return True

    def _load_cache(self, item):
        path = item.get("visual_path") or self._cache_path(item["name"])
        self._reset_buffers()
        if not os.path.exists(path):
            return
        try:
            data = self._np.load(path, allow_pickle=False)
            self._grid = data["grid"].astype(self._np.float32)
            self._lasers = {(round(p[0] / self.VOXEL), round(p[1] / self.VOXEL)): tuple(p) for p in data["lasers"]}
            self._trajectory = deque((tuple(p) for p in data["trajectory"]), maxlen=6000)
            b = data["bounds"].tolist(); self._grid_bounds = tuple(b) if len(b) == 4 else None
            self._artifacts = json.loads(str(data["artifacts"]))
        except Exception as exc:
            print(f"[ControlledSpatialMap] cache load failed: {exc}")

    def _reset_buffers(self):
        self._grid = self._np.zeros((0, 3), dtype=self._np.float32); self._grid_bounds = None
        self._lasers.clear(); self._laser_hits.clear(); self._trajectory.clear()

    def _cache_path(self, name):
        return os.path.join(self._cache_dir, f"{self._safe(name)}.npz")

    @staticmethod
    def _safe(name):
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._") or "map"

    def _info(self, state):
        return {"state": state, "topic_out": [{"topic": self._topic, "format": "sensor/mapping"}],
                "active_map": self._current_map, "recording": self._recording,
                "map_count": len(self._db.maps()), "startup_error": self._startup_error}
