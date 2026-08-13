"""Map sensor for Tianyi controlled_spatial.

The saved Slamtec STCM file is the authoritative map source.  Live laser,
pose, trajectory and artifacts are overlays only; they must not alter a saved
map's rendered grid.
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

    def set_state(self, key: str, value: str | None):
        with self._lock:
            self._conn.execute(
                "INSERT OR REPLACE INTO state (key, value, updated_at) "
                "VALUES (?, ?, strftime('%s','now'))",
                (key, value),
            )
            self._conn.commit()

    def ensure_map(self, name: str, path: str, visual_path: str):
        with self._lock:
            self._conn.execute(
                "INSERT OR IGNORE INTO maps (name, pcd_path, visual_path) VALUES (?, ?, ?)",
                (name, path, visual_path),
            )
            self._conn.execute("UPDATE maps SET visual_path=? WHERE name=?", (visual_path, name))
            self._conn.commit()


class ControlledSpatialMapPlugin:
    PREFIX = "spatial_map"
    # Keep the payload within the existing Agent Core renderer budget while
    # reserving more of it for the persistent head-camera cloud.
    MAX_POINTS = 80000
    VOXEL = 0.06
    HEAD_VOXEL = 0.05
    HEAD_FRAME_LIMIT = 9000
    HEAD_PROCESS_INTERVAL = 0.20
    HEAD_MAP_LIMIT = 300000
    _EXTRINSIC_STATE_KEY = "controlled_spatial_map_head_camera_extrinsic"
    _RECORDING_STATE_KEY = "controlled_spatial_map_head_recording_enabled"

    def __init__(self, config: dict, namespace: str, ros2, slamtec_client):
        import numpy as np
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import UInt8MultiArray

        self._np = np
        self._slamtec = slamtec_client
        self._db = _MapDB(config.get("native_slam_db_path", "/data/controlled_spatial/controlled_spatial.db"))
        self._cache_dir = config.get("cache_dir", "/data/controlled_spatial/map-visuals")
        os.makedirs(self._cache_dir, exist_ok=True)
        self._head_map_dir = config.get(
            "head_map_dir", os.path.join(os.path.dirname(self._cache_dir), "head-3d")
        )
        os.makedirs(self._head_map_dir, exist_ok=True)
        self._topic = f"/{namespace}/controlled_spatial/map"
        self._poll_hz = max(0.5, float(config.get("poll_hz", 3.0)))
        self._artifact_hz = max(0.1, float(config.get("artifact_hz", 0.4)))
        # Kept for compatibility with old deployments.  The saved STCM grid
        # is decoded in its own map frame and does not use live-grid polling.
        self._grid_flip_x = bool(config.get("grid_flip_x", False))
        self._grid_flip_y = bool(config.get("grid_flip_y", False))
        self._grid_x_offset = float(config.get("grid_x_offset", 0.0))
        self._grid_y_offset = float(config.get("grid_y_offset", 0.0))
        self._grid_roll_x_cells = int(config.get("grid_roll_x_cells", 0))
        # A saved STCM already contains the mapped laser evidence.  Replaying
        # mapping-time scan frames on top of it makes a 2D map look like
        # duplicated/shifted clouds in the height-coloured renderer.
        self._show_scan_overlay = bool(config.get("show_scan_overlay", False))
        self._head_pointcloud_enabled = bool(config.get("head_pointcloud_enabled", True))
        self._head_pointcloud_topic = str(
            config.get("head_pointcloud_topic", "/ob_camera_head/depth/points"))
        self._head_recording_enabled = self._load_recording_enabled(config)
        self._head_camera_extrinsic = self._load_extrinsic(config)
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
        self._loaded_map_signature = None
        self._runtime_active = None
        self._recording = None
        self._pose = None
        self._grid = np.zeros((0, 3), dtype=np.float32)
        self._grid_bounds = None
        self._lasers = {}
        self._laser_hits = {}
        self._head_cloud = {}
        self._head_cloud_map_name = None
        self._head_hits = {}
        self._last_head_process = 0.0
        self._trajectory_max_points = max(50, int(config.get("trajectory_max_points", 400)))
        self._trajectory = deque(maxlen=self._trajectory_max_points)
        self._artifacts = {"walls": [], "tracks": [], "areas": {}}
        self._last_artifacts = 0.0
        self._last_cache = 0.0
        self._last_publish = 0.0
        self._startup_error = None
        if self._head_pointcloud_enabled:
            self._sub_node.create_subscription(
                PointCloud2, self._head_pointcloud_topic, self._on_head_cloud,
                qos_profile_sensor_data,
            )

    def get_tools(self):
        return [self.get_tool()]

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": "Tianyi controlled map: grid, boundary, laser, route, tags and special areas.",
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": [
                    "list_maps", "select_map",
                    "set_3d_recording", "clear_3d_cloud", "clear_trajectory",
                ]},
                "map_name": {"type": "string"},
                "enabled": {"type": "boolean"},
            }, "required": ["action"], "x-action-params": {
                "list_maps": {"params": [], "description": "List saved maps."},
                "select_map": {"params": ["map_name"], "description": "Select one saved map for display."},
                "set_3d_recording": {"params": ["enabled"], "description": "Enable or disable persistent 3D point-cloud recording."},
                "clear_3d_cloud": {"params": ["map_name"], "description": "Clear the persistent 3D point cloud for one saved map."},
                "clear_trajectory": {"params": [], "description": "Clear the displayed robot trajectory."},
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

    def _load_extrinsic(self, config):
        configured = config.get("head_camera_extrinsic", {})
        try:
            candidate = self._validate_extrinsic(configured)
        except ValueError:
            # Nominal optical -> robot conversion. It is only a starting point
            # before a persisted transform is available.
            candidate = {
                "translation_m": [0.0, 0.0, 1.50],
                "rotation_rpy_rad": [-math.pi / 2.0, 0.0, -math.pi / 2.0],
            }
        persisted = self._db.state(self._EXTRINSIC_STATE_KEY)
        if persisted:
            try:
                candidate = self._validate_extrinsic(json.loads(persisted))
            except (TypeError, ValueError, json.JSONDecodeError):
                pass
        return candidate

    def _load_recording_enabled(self, config):
        configured = bool(config.get("head_recording_enabled", False))
        persisted = self._db.state(self._RECORDING_STATE_KEY)
        if persisted in ("true", "false"):
            return persisted == "true"
        return configured

    @staticmethod
    def _validate_extrinsic(value):
        if not isinstance(value, dict):
            raise ValueError("head camera extrinsic must be an object")

        def vector(raw, name):
            if isinstance(raw, str):
                raw = [part.strip() for part in raw.split(",")]
            if not isinstance(raw, (list, tuple)) or len(raw) != 3:
                unit = "meters" if name == "translation_m" else "radians"
                raise ValueError(f"{name} must contain exactly three {unit} values")
            return raw

        translation = vector(value.get("translation_m"), "translation_m")
        rotation = vector(value.get("rotation_rpy_rad"), "rotation_rpy_rad")
        try:
            translation = [float(v) for v in translation]
            rotation = [float(v) for v in rotation]
        except (TypeError, ValueError):
            raise ValueError("extrinsic values must be numeric")
        if not all(math.isfinite(v) and abs(v) <= 5.0 for v in translation):
            raise ValueError("translation_m values must be finite and within +/-5 m")
        if not all(math.isfinite(v) and abs(v) <= math.tau for v in rotation):
            raise ValueError("rotation_rpy_rad values must be finite and within +/-2pi")
        return {"translation_m": translation, "rotation_rpy_rad": rotation}

    def _rpy_matrix(self, rpy):
        roll, pitch, yaw = rpy
        sr, cr = math.sin(roll), math.cos(roll)
        sp, cp = math.sin(pitch), math.cos(pitch)
        sy, cy = math.sin(yaw), math.cos(yaw)
        return self._np.asarray([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=self._np.float32)

    def _camera_to_map(self, camera_points, pose, extrinsic):
        """Apply a fixed camera->base transform, then Slamtec's planar pose."""
        rotation = self._rpy_matrix(extrinsic["rotation_rpy_rad"])
        base_points = camera_points @ rotation.T
        base_points += self._np.asarray(extrinsic["translation_m"], dtype=self._np.float32)
        yaw = float(pose["yaw"])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        world = base_points.copy()
        world[:, 0] = float(pose["x"]) + cosine * base_points[:, 0] - sine * base_points[:, 1]
        world[:, 1] = float(pose["y"]) + sine * base_points[:, 0] + cosine * base_points[:, 1]
        return world

    def _on_head_cloud(self, msg):
        """Project downsampled camera frames only while recording."""
        try:
            # PointCloud2 may arrive at camera frame rate. Throttle the
            # expensive map projection and voxel merge.
            with self._lock:
                recording = self._head_recording_enabled
                if (recording and
                        time.monotonic() - self._last_head_process < self.HEAD_PROCESS_INTERVAL):
                    return
            fields = {field.name: int(field.offset) for field in msg.fields}
            if not all(name in fields for name in ("x", "y", "z")) or msg.point_step < 12:
                return
            count = min(int(msg.width) * int(msg.height), len(msg.data) // int(msg.point_step))
            if count <= 0:
                return
            step = max(1, int(math.ceil(count / self.HEAD_FRAME_LIMIT)))
            indexes = self._np.arange(0, count, step, dtype=self._np.intp)
            dtype = ">f4" if msg.is_bigendian else "<f4"
            raw = memoryview(msg.data)
            point_step = int(msg.point_step)

            def component(name):
                values = self._np.ndarray(
                    shape=(count,), dtype=dtype, buffer=raw,
                    offset=fields[name], strides=(point_step,),
                )
                return values[indexes]

            camera = self._np.column_stack((component("x"), component("y"), component("z"))).astype(self._np.float32)
            valid = self._np.isfinite(camera).all(axis=1)
            # Optical Z is forward. Discard invalid depth, near-field noise and
            # distant points that would dominate a small indoor map.
            valid &= (camera[:, 2] >= 0.25) & (camera[:, 2] <= 6.0)
            camera = camera[valid]
            if not len(camera):
                return
            with self._lock:
                pose = dict(self._pose) if self._pose else None
                extrinsic = dict(self._head_camera_extrinsic)
                map_name = self._current_map
                if pose is None or map_name is None:
                    return
                # 3D capture is independent from the 2D map's mapping state.
                # The spatial_map card's recording switch is the sole gate.
                recording = self._head_recording_enabled
            if not recording:
                return
            with self._lock:
                self._last_head_process = time.monotonic()
            world = self._camera_to_map(camera, pose, extrinsic)
            frame = {
                (round(point[0] / self.HEAD_VOXEL), round(point[1] / self.HEAD_VOXEL), round(point[2] / self.HEAD_VOXEL)): tuple(point)
                for point in world
            }
            with self._lock:
                if self._head_cloud_map_name != map_name:
                    # A map change replaces the cloud before recording resumes.
                    return
                for key, point in frame.items():
                    # Voxel deduplication already removes most depth noise.
                    # Insert the first observation so recording becomes
                    # visible immediately instead of waiting for a second
                    # matching frame.
                    self._head_cloud[key] = point
                self._head_hits.clear()
                if len(self._head_cloud) > self.HEAD_MAP_LIMIT:
                    kept = list(self._head_cloud.items())[-self.HEAD_MAP_LIMIT:]
                    self._head_cloud = dict(kept)
        except Exception as exc:
            self._startup_error = f"head point cloud: {exc}"

    def _recording_info(self, state):
        with self._lock:
            recording_enabled = self._head_recording_enabled
            map_name = self._head_cloud_map_name
            cloud_points = len(self._head_cloud)
        return {
            **self._info(state),
            "head_recording_enabled": recording_enabled,
            "head_map_path": self._head_map_path_for(map_name),
            "head_cloud_points": cloud_points,
        }

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
                self._load_saved_map(found)
            self._db.set_state("active_map", name)
            self._publish(force=True)
            return self._info("selected")
        if action == "set_3d_recording":
            if not isinstance(args.get("enabled"), bool):
                return {"error": "enabled must be a boolean"}
            with self._lock:
                if args["enabled"] and self._current_map is None:
                    return {"error": "select a map before enabling 3D recording"}
                self._head_recording_enabled = args["enabled"]
                if args["enabled"]:
                    self._last_head_process = 0.0
            self._db.set_state(
                self._RECORDING_STATE_KEY,
                "true" if args["enabled"] else "false",
            )
            self._save_cache(force=True)
            return self._recording_info(
                "recording_enabled" if args["enabled"] else "recording_disabled")
        if action == "clear_3d_cloud":
            name = args.get("map_name")
            found = next((m for m in self._db.maps() if m["name"] == name), None)
            if not found:
                return {"error": "map_name not found"}
            with self._lock:
                if self._head_recording_enabled and self._head_cloud_map_name == name:
                    return {"error": "disable 3D recording before clearing its map"}
                if self._head_cloud_map_name == name:
                    cleared = len(self._head_cloud)
                    self._head_cloud.clear()
                    self._head_hits.clear()
                else:
                    cleared = self._head_cloud_point_count(name)
                path = self._head_map_path_for(name)
                try:
                    os.remove(path)
                except FileNotFoundError:
                    pass
                except OSError as exc:
                    return {"error": f"failed to clear 3D cloud: {exc}"}
            self._publish(force=True)
            return {"map_name": name, "cleared_3d_points": cleared,
                    **self._info("3d_cloud_cleared")}
        if action == "clear_trajectory":
            with self._lock:
                cleared = len(self._trajectory)
                self._trajectory.clear()
            self._save_cache(force=True)
            self._publish(force=True)
            return {"cleared_trajectory_points": cleared, **self._info("trajectory_cleared")}
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
                if self._show_scan_overlay:
                    scan = self._slamtec.get_laser_scan()
                    if isinstance(scan, dict) and not scan.get("error"):
                        self._consume_scan(scan)
                now = time.monotonic()
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
            # A controlled_spatial load/start operation is authoritative. Do
            # not let a selection from the previous active map keep the
            # viewer attached to stale cache data after that operation.
            active_changed = active != self._runtime_active
            self._runtime_active = active
            if active_changed and active:
                self._selected = active
                desired = active
            if desired:
                found = next((m for m in maps if m["name"] == desired), None)
                if not found:
                    # controlled_spatial can delete a map row while its
                    # active_map state still points at the deleted name.
                    # Never expose that stale state as a selected map.
                    print(
                        f"[ControlledSpatialMap] clearing stale map state: {desired}",
                        flush=True,
                    )
                    self._current_map = None
                    self._selected = None
                    self._recording = None
                    self._loaded_map_signature = None
                    self._reset_buffers()
                elif desired != self._current_map:
                    self._current_map = desired
                    self._selected = desired
                    self._load_saved_map(found)
                elif self._map_signature(found) != self._loaded_map_signature:
                    self._load_saved_map(found)
            elif not desired and self._current_map is not None:
                self._current_map = None
                self._selected = None
                self._loaded_map_signature = None
                self._reset_buffers()
            if status == "mapping" and active:
                self._recording = active
            elif self._recording:
                self._save_cache(force=True)
                self._recording = None

    def _load_saved_map(self, item):
        """Load a map's formal STCM grid, then restore cached overlays only."""
        self._save_3d_map()
        self._reset_buffers()
        loaded = self._load_stcm_grid(item.get("pcd_path", ""))
        self._loaded_map_signature = self._map_signature(item) if loaded else None
        self._load_cache(item)
        self._load_3d_map(item["name"])

    @staticmethod
    def _map_signature(item):
        path = item.get("pcd_path", "")
        try:
            stat = os.stat(path)
        except OSError:
            return None
        return (item.get("name"), path, stat.st_size, stat.st_mtime_ns)

    def _load_stcm_grid(self, path: str):
        """Decode the embedded ``explore`` layer from a Slamtec STCM file.

        In the STCM version emitted by this chassis, the metadata carries the
        grid dimensions/origin/resolution and the final width*height bytes are
        the raw uint8 grid layer.  This avoids using the mutable chassis
        ``/maps/explore`` endpoint as a saved map's visual source.
        """
        try:
            with open(path, "rb") as stream:
                raw = stream.read()
            if not raw.startswith(b"STCM"):
                raise ValueError("not an STCM map")
            metadata = raw[:min(len(raw), 65536)]
            if b"vnd.slamtec.map-layer/vnd.grid-map+binary" not in metadata:
                raise ValueError("STCM does not contain a grid-map layer")

            def property_value(name):
                needle = name.encode("ascii")
                offset = metadata.find(needle)
                if offset < 0:
                    raise ValueError(f"missing STCM property: {name}")
                length_offset = offset + len(needle)
                if length_offset + 2 > len(metadata):
                    raise ValueError(f"truncated STCM property: {name}")
                length = struct.unpack_from("<H", metadata, length_offset)[0]
                start = length_offset + 2
                value = metadata[start:start + length]
                if len(value) != length:
                    raise ValueError(f"truncated STCM value: {name}")
                return value.decode("ascii")

            width = int(property_value("dimension_width"))
            height = int(property_value("dimension_height"))
            min_x = float(property_value("origin_x"))
            min_y = float(property_value("origin_y"))
            resolution_x = float(property_value("resolution_x"))
            resolution_y = float(property_value("resolution_y"))
            if width <= 0 or height <= 0 or resolution_x <= 0 or resolution_y <= 0:
                raise ValueError("invalid STCM grid dimensions")
            cell_count = width * height
            if cell_count > len(raw):
                raise ValueError("STCM grid exceeds file size")
            cells = self._np.frombuffer(raw, dtype=self._np.uint8, count=cell_count,
                                        offset=len(raw) - cell_count).reshape(height, width)
            if self._grid_roll_x_cells:
                cells = self._np.roll(cells, self._grid_roll_x_cells, axis=1)
            # flatnonzero() below returns indices in row-major order. Keep the
            # grid flat for all subsequent indexing; otherwise cells[known]
            # indexes only the first axis of the 2D array and can overflow on
            # any map whose flattened index exceeds its height.
            cells = cells.reshape(-1)
        except (OSError, UnicodeDecodeError, ValueError, struct.error) as exc:
            self._startup_error = f"saved map load failed: {exc}"
            print(f"[ControlledSpatialMap] {self._startup_error}", flush=True)
            return False

        floor = self._np.flatnonzero(cells == 127)
        features = self._np.flatnonzero((cells != 0) & (cells != 127))
        if not len(floor) and not len(features):
            return False

        def spatial_sample(indices, limit):
            """Select a regular 2D lattice rather than a flattened stride."""
            if len(indices) <= limit:
                return indices
            stride = max(1, int(math.ceil(math.sqrt(len(indices) / limit))))
            rows = indices // width
            cols = indices % width
            sampled = indices[(rows % stride == 0) & (cols % stride == 0)]
            return sampled[:limit]

        # A denser floor makes the explored area readable; features retain a
        # finer grid so walls and obstacle edges stay visible.
        floor = spatial_sample(floor, 39000)
        features = spatial_sample(features, 13000)
        known = self._np.concatenate((floor, features))
        rows, cols = known // width, known % width
        values = cells[known]
        z = self._np.where(values == 127, -0.03, 0.04).astype(self._np.float32)
        max_x = min_x + width * resolution_x
        max_y = min_y + height * resolution_y
        grid_x = (max_x - (cols.astype(self._np.float32) + 0.5) * resolution_x
                  if self._grid_flip_x else
                  min_x + (cols.astype(self._np.float32) + 0.5) * resolution_x)
        grid_y = (max_y - (rows.astype(self._np.float32) + 0.5) * resolution_y
                  if self._grid_flip_y else
                  min_y + (rows.astype(self._np.float32) + 0.5) * resolution_y)
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
                width * resolution_x,
                height * resolution_y,
            )
            self._startup_error = None
        print(
            f"[ControlledSpatialMap] loaded saved STCM: {path} "
            f"grid={width}x{height} points={len(points)} "
            f"roll_x={self._grid_roll_x_cells}",
            flush=True,
        )
        return True

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
            grid = self._grid.copy(); head_cloud = list(self._head_cloud.values())
            trajectory = list(self._trajectory)
            bounds = self._grid_bounds; pose = dict(self._pose) if self._pose else None
            current_map = self._current_map; artifacts = json.loads(json.dumps(self._artifacts))
        maps = self._db.maps()
        tags = next((m["tags"] for m in maps if m["name"] == current_map), [])
        overlays = self._boundary_points(bounds) + self._trajectory_points(trajectory)
        overlays += self._line_points(artifacts["walls"], 0.20) + self._line_points(artifacts["tracks"], 0.13)
        for index, (_, areas) in enumerate(artifacts["areas"].items()):
            overlays += self._area_points(areas, 0.12 + index * 0.025)
        overlays += self._tag_points(tags)
        flat_cloud = (self._np.asarray(overlays, dtype=self._np.float32).reshape(-1, 3)
                      if overlays else self._np.zeros((0, 3), dtype=self._np.float32))
        saved_head = (self._np.asarray(head_cloud, dtype=self._np.float32).reshape(-1, 3)
                      if head_cloud else self._np.zeros((0, 3), dtype=self._np.float32))

        def sample(points, limit):
            if len(points) <= limit:
                return points
            step = max(1, int(math.ceil(len(points) / limit)))
            return points[::step][:limit]

        # Keep a guaranteed visual budget for the head cloud. The legacy
        # renderer accepts 80000 points, so reserve 60000 for 3D even when a
        # 2D base grid is present.
        base_parts = [part for part in (grid, flat_cloud) if len(part)]
        base_points = (self._np.vstack(base_parts) if base_parts
                       else self._np.zeros((0, 3), dtype=self._np.float32))
        head_points = saved_head
        head_budget = self.MAX_POINTS if not len(base_points) else min(60000, self.MAX_POINTS)
        head_limit = min(len(head_points), head_budget)
        head_points = sample(head_points, head_limit)
        base_points = sample(base_points, self.MAX_POINTS - len(head_points))
        parts = [part for part in (base_points, head_points) if len(part)]
        points = self._np.vstack(parts) if parts else self._np.zeros((0, 3), dtype=self._np.float32)
        robot = pose or {"x": 0.0, "y": 0.0, "yaw": 0.0}
        meta = {"version": 3, "active_map": current_map, "robot": {**robot, "pose_available": pose is not None},
                "maps": maps, "tags": tags, "boundary": bounds, "artifacts": artifacts,
                "trajectory_points": len(trajectory), "laser_points": 0, "grid_points": len(grid),
                "head_cloud_points": len(saved_head),
                "head_recording_enabled": self._head_recording_enabled}
        raw_meta = json.dumps(meta, ensure_ascii=False).encode()
        display_yaw = -float(robot["yaw"])
        payload = struct.pack("<fffBI", robot["x"], robot["y"], display_yaw, 7, len(points)) + points.tobytes()
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

            # Render POIs as dense 3D point-cloud markers so the existing
            # mapping renderer can show a pillar and an orientation arrow.
            for z in self._np.linspace(0.04, 0.82, 20):
                for i in range(24):
                    angle = i * math.tau / 24.0
                    result.append((x + 0.14 * math.cos(angle),
                                   y + 0.14 * math.sin(angle), float(z)))

            # Keep the direction arrow at the pillar base so it remains
            # readable against the floor and does not hide the marker stem.
            arrow_z = 0.14
            side_x, side_y = -math.sin(yaw), math.cos(yaw)
            # A short 3D tube makes the arrow shaft readable at map scale.
            for d in self._np.linspace(0.0, 0.38, 28):
                for lateral in (-0.035, 0.0, 0.035):
                    for dz in (-0.025, 0.0, 0.025):
                        result.append((
                            x + d * math.cos(yaw) + lateral * side_x,
                            y + d * math.sin(yaw) + lateral * side_y,
                            arrow_z + dz,
                        ))

            # Fill a triangular arrowhead instead of drawing two sparse wings.
            for d in self._np.linspace(0.30, 0.55, 18):
                width = 0.16 * (1.0 - (d - 0.30) / 0.25)
                for lateral in self._np.linspace(-width, width, 9):
                    for dz in (-0.03, 0.0, 0.03):
                        result.append((
                            x + d * math.cos(yaw) + lateral * side_x,
                            y + d * math.sin(yaw) + lateral * side_y,
                            arrow_z + dz,
                        ))
        return result

    def _save_cache(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_cache < 5:
            return False
        with self._lock:
            # The STCM file owns the base grid.  This cache contains only
            # overlays accumulated by this plugin for the selected map.
            name = self._recording or self._current_map
            if not name:
                self._last_cache = now
                return False
            self._save_3d_map()
            path = self._cache_path(name)
            self._np.savez_compressed(path,
                                      trajectory=self._np.asarray(self._trajectory, dtype=self._np.float32),
                                      artifacts=json.dumps(self._artifacts),
                                      source_map_name=self._np.asarray(name))
            self._last_cache = now
            return True

    def _head_map_path_for(self, name):
        if not name:
            return None
        return os.path.join(self._head_map_dir, f"{self._safe(name)}.npz")

    def _save_3d_map(self):
        name = self._head_cloud_map_name
        path = self._head_map_path_for(name)
        if not path:
            return False
        head_cloud = self._np.asarray(
            list(self._head_cloud.values()), dtype=self._np.float32
        ).reshape(-1, 3)
        self._np.savez_compressed(
            path,
            head_cloud=head_cloud,
            source_map_name=self._np.asarray(name),
            updated_at=self._np.asarray(time.time(), dtype=self._np.float64),
        )
        return True

    def _head_cloud_point_count(self, name):
        path = self._head_map_path_for(name)
        if not path or not os.path.exists(path):
            return 0
        try:
            with self._np.load(path, allow_pickle=False) as data:
                points = data["head_cloud"] if "head_cloud" in data.files else ()
                return len(points)
        except Exception:
            return 0

    def _load_3d_map(self, name):
        self._head_cloud = {}
        self._head_cloud_map_name = name
        path = self._head_map_path_for(name)
        if not path:
            return
        if not os.path.exists(path):
            self._migrate_legacy_map_cloud(name)
            return
        try:
            with self._np.load(path, allow_pickle=False) as data:
                source = str(data["source_map_name"].item()) if "source_map_name" in data.files else name
                if source != name:
                    raise ValueError(f"mismatched source map: {source}")
                saved_head = (data["head_cloud"] if "head_cloud" in data.files
                              else self._np.zeros((0, 3), dtype=self._np.float32))
            self._head_cloud = {
                (round(p[0] / self.HEAD_VOXEL), round(p[1] / self.HEAD_VOXEL), round(p[2] / self.HEAD_VOXEL)): tuple(p)
                for p in saved_head
            }
            print(
                f"[ControlledSpatialMap] loaded 3D map: "
                f"{path} points={len(self._head_cloud)}",
                flush=True,
            )
        except Exception as exc:
            print(f"[ControlledSpatialMap] 3D map load failed: {exc}", flush=True)

    def _migrate_legacy_map_cloud(self, name):
        """Move only safely attributable per-map cache clouds to the new store."""
        path = self._cache_path(name)
        if not os.path.exists(path):
            return
        try:
            with self._np.load(path, allow_pickle=False) as data:
                source = (str(data["source_map_name"].item())
                          if "source_map_name" in data.files else None)
                if source != name or "head_cloud" not in data.files:
                    return
                saved_head = data["head_cloud"]
            self._head_cloud = {
                (round(p[0] / self.HEAD_VOXEL), round(p[1] / self.HEAD_VOXEL),
                 round(p[2] / self.HEAD_VOXEL)): tuple(p)
                for p in saved_head
            }
            self._save_3d_map()
            print(
                f"[ControlledSpatialMap] migrated legacy 3D map: {name} "
                f"points={len(self._head_cloud)}",
                flush=True,
            )
        except Exception as exc:
            print(f"[ControlledSpatialMap] legacy 3D migration failed: {exc}", flush=True)

    def _load_cache(self, item):
        path = item.get("visual_path") or self._cache_path(item["name"])
        if not os.path.exists(path):
            return
        try:
            data = self._np.load(path, allow_pickle=False)
            source = (
                str(data["source_map_name"].item())
                if "source_map_name" in data.files else None
            )
            if source != item["name"]:
                print(
                    "[ControlledSpatialMap] ignoring cache with missing or "
                    f"mismatched source map: {path}",
                    flush=True,
                )
                return
            # Ignore legacy saved scan overlays. They were captured while
            # mapping and are already represented by the STCM occupancy grid.
            self._lasers = {}
            self._trajectory = deque(
                (tuple(p) for p in data["trajectory"]),
                maxlen=self._trajectory_max_points,
            )
            self._artifacts = json.loads(str(data["artifacts"]))
        except Exception as exc:
            print(f"[ControlledSpatialMap] cache load failed: {exc}")

    def _reset_buffers(self):
        self._grid = self._np.zeros((0, 3), dtype=self._np.float32); self._grid_bounds = None
        self._lasers.clear(); self._laser_hits.clear(); self._head_hits.clear(); self._trajectory.clear()

    def _cache_path(self, name):
        return os.path.join(self._cache_dir, f"{self._safe(name)}.npz")

    @staticmethod
    def _safe(name):
        return re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._") or "map"

    def _info(self, state):
        return {"state": state, "topic_out": [{"topic": self._topic, "format": "sensor/mapping"}],
                "active_map": self._current_map, "recording": self._recording,
                "map_count": len(self._db.maps()), "startup_error": self._startup_error}
