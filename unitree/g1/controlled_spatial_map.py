"""Map view publisher for the G1 controlled_spatial_map sensor card."""

from __future__ import annotations

import json
import math
import multiprocessing as mp
import os
import queue
import re
import sqlite3
import struct
import threading
import time
from array import array


class _MapDB:
    """SQLite reader for maps and POIs created by controlled_spatial."""

    def __init__(self, db_path: str):
        db_dir = os.path.dirname(db_path)
        if db_dir:
            os.makedirs(db_dir, exist_ok=True)
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._lock = threading.RLock()
        self._migrate()

    def _migrate(self):
        with self._lock:
            self._conn.executescript("""
                CREATE TABLE IF NOT EXISTS maps (
                    name TEXT PRIMARY KEY,
                    pcd_path TEXT NOT NULL,
                    created_at REAL DEFAULT (strftime('%s','now'))
                );
                CREATE TABLE IF NOT EXISTS poi (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    description TEXT DEFAULT '',
                    x REAL NOT NULL, y REAL NOT NULL, yaw REAL DEFAULT 0,
                    map_name TEXT NOT NULL,
                    created_at REAL DEFAULT (strftime('%s','now')),
                    UNIQUE(name, map_name)
                );
                CREATE TABLE IF NOT EXISTS state (
                    key TEXT PRIMARY KEY,
                    value TEXT,
                    updated_at REAL DEFAULT (strftime('%s','now'))
                );
            """)
            cols = {
                row["name"]
                for row in self._conn.execute("PRAGMA table_info(maps)").fetchall()
            }
            if "cloud_path" not in cols:
                self._conn.execute("ALTER TABLE maps ADD COLUMN cloud_path TEXT")
            self._conn.commit()

    def list_maps(self) -> list[dict]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT name, pcd_path, cloud_path, created_at FROM maps ORDER BY created_at DESC"
            ).fetchall()
            return [dict(r) for r in rows]

    def list_pois(self, map_name: str) -> list[dict]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT name, description, x, y, yaw FROM poi WHERE map_name = ? ORDER BY name",
                (map_name,),
            ).fetchall()
            return [dict(r) for r in rows]

    def list_maps_with_pois(self) -> list[dict]:
        maps = self.list_maps()
        for item in maps:
            item["tags"] = self.list_pois(item["name"])
        return maps

    def get_state(self, key: str) -> str | None:
        with self._lock:
            try:
                row = self._conn.execute("SELECT value FROM state WHERE key = ?", (key,)).fetchone()
            except sqlite3.Error:
                return None
            return row["value"] if row else None

    def set_cloud_path(self, map_name: str, cloud_path: str):
        with self._lock:
            self._conn.execute(
                "UPDATE maps SET cloud_path = ? WHERE name = ?",
                (cloud_path, map_name),
            )
            self._conn.commit()

    def ensure_map(self, name: str, pcd_path: str, cloud_path: str | None = None):
        with self._lock:
            row = self._conn.execute("SELECT name FROM maps WHERE name = ?", (name,)).fetchone()
            if row:
                if cloud_path:
                    self._conn.execute(
                        "UPDATE maps SET cloud_path = ? WHERE name = ?",
                        (cloud_path, name),
                    )
            else:
                self._conn.execute(
                    "INSERT INTO maps (name, pcd_path, cloud_path) VALUES (?, ?, ?)",
                    (name, pcd_path, cloud_path),
                )
            self._conn.commit()


class ControlledSpatialMapNode:
    """Publishes a monitor-friendly map snapshot for the controlled spatial card."""

    np = None

    VOXEL_SIZE = 0.06
    # The renderer receives complete snapshots, so prioritise a small latest
    # snapshot over a large stream that can queue behind the live robot pose.
    MAP_PUBLISH_INTERVAL = 0.15
    CLOUD_SNAPSHOT_INTERVAL = 0.50
    META_PUBLISH_INTERVAL = 1.0
    CLOUD_SAVE_INTERVAL = 5.0
    MAX_SEND_POINTS = 16000
    DYNAMIC_MIN_HITS = 2
    VIEW_GUARD_EXTENT = 300.0

    def __init__(self, topic: str, db, cloud_dir: str):
        import numpy as np
        from rclpy.node import Node
        from std_msgs.msg import UInt8MultiArray

        ControlledSpatialMapNode.np = np
        os.makedirs(cloud_dir, exist_ok=True)
        self._node = Node("g1_controlled_spatial_map")
        self._pub = self._node.create_publisher(UInt8MultiArray, topic, 10)
        self._topic = topic
        self._db = db
        self._cloud_dir = cloud_dir
        self._current_pose: dict | None = None
        self._map_status = "idle"
        self._active_map: str | None = None
        self._native_pcd_path: str | None = None
        self._cached_cloud_path: str | None = None
        self._local_pcd_available = False
        self._point_source = "none"
        self._manual_recording = False
        self._recording_map_name: str | None = None
        self._lock = threading.Lock()
        self._map_buffer: dict[tuple, tuple] = {}
        self._pending_voxel_hits: dict[tuple, int] = {}
        self._map_buffer_lock = threading.Lock()
        self._map_buffer_dirty = False
        self._map_buffer_revision = 0
        self._render_cloud_points = np.zeros((0, 3), dtype=np.float32)
        self._render_cloud_revision = -1
        self._last_cloud_snapshot_time = 0.0
        # Mapping frames arrive faster than we can voxelize them. Retaining a
        # backlog makes the displayed map lag behind the robot, while the most
        # recent frames contain all data needed for the visual accumulation.
        self._cloud_queue = queue.Queue(maxsize=1)
        self._running = True
        # DDS callbacks can arrive while the driver is shutting down. Keep
        # publishing serialized with node destruction so no callback touches
        # a publisher whose underlying ROS handle is already gone.
        self._closing = threading.Event()
        self._publish_lock = threading.Lock()
        self._last_map_publish_time = 0.0
        self._last_meta_publish_time = 0.0
        self._last_cloud_save_time = 0.0
        self._last_fallback_log_time = 0.0
        self._cached_maps: list[dict] = []
        self._cloud_source = "unknown"  # "mapping" or "relocation" — tracks which DDS topic fed the buffer
        self._MAX_BUFFER_SIZE = 200000  # cap voxel buffer to prevent unbounded growth
        self._MAX_PENDING_VOXELS = 300000

        self._worker = threading.Thread(
            target=self._cloud_processor_loop,
            daemon=True,
            name="controlled_spatial_cloud",
        )
        self._worker.start()

        self._dds_subs = []
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
            map_sub = ChannelSubscriber("rt/unitree/slam_mapping/points", PointCloud2_)
            map_sub.Init(self._on_mapping_cloud, 10)
            self._dds_subs.append(map_sub)
            reloc_sub = ChannelSubscriber("rt/unitree/slam_relocation/points", PointCloud2_)
            reloc_sub.Init(self._on_relocation_cloud, 10)
            self._dds_subs.append(reloc_sub)
            print("[ControlledSpatialMap] subscribed SLAM point clouds")
        except Exception as e:
            print(f"[ControlledSpatialMap] failed to subscribe point clouds: {e}")

        # Fallback heartbeat. Normal pose/cloud callbacks publish at the
        # higher rate above, while this also keeps an idle view connected.
        self._publish_timer = self._node.create_timer(0.5, self._periodic_publish)

    @property
    def node(self):
        return self._node

    def stop(self):
        if self._closing.is_set():
            return
        self._closing.set()
        self._running = False
        try:
            self._publish_timer.cancel()
        except Exception:
            pass
        self.save_active_map_cache(force=True)
        try:
            self._worker.join(timeout=2)
        except Exception:
            pass
        try:
            # Wait for an already-running publish before destroying the node.
            with self._publish_lock:
                self._node.destroy_node()
        except Exception:
            pass

    def update_pose(self, pose: dict, map_status: str | None = None):
        previous = self.get_map_status()
        with self._lock:
            self._current_pose = dict(pose)
            if map_status is not None and not self._manual_recording:
                self._map_status = map_status
        status_changed = map_status is not None and previous != map_status
        if self.is_recording():
            self._maybe_publish_full_map()
            return
        if map_status is not None and previous != "mapping" and map_status == "mapping":
            self.clear_map_buffer(publish=False)
        if map_status is not None and previous == "mapping" and map_status != "mapping":
            self.save_active_map_cache(force=True)
        if status_changed:
            self.publish_now(force=True)
        else:
            self._maybe_publish_full_map()

    def set_active_map(self, name: str | None):
        with self._lock:
            self._active_map = name
        self.publish_now(force=True)

    def set_active_map_info(
        self,
        name: str | None,
        pcd_path: str | None = None,
        cloud_path: str | None = None,
        load_cached: bool = False,
        clear_for_new: bool = False,
    ):
        previous = self.get_active_map()
        if previous and previous != name:
            self.save_active_map_cache(force=True)
        if name and not cloud_path:
            cloud_path = self._cloud_path_for_map(name)
        with self._lock:
            self._active_map = name
            self._native_pcd_path = pcd_path
            self._cached_cloud_path = cloud_path
            self._local_pcd_available = False
            if clear_for_new:
                self._point_source = "none"
            elif not self._map_buffer:
                self._point_source = "none"
        if clear_for_new:
            self.clear_map_buffer(publish=False)
        if load_cached and cloud_path:
            self.load_cached_cloud_to_buffer(cloud_path)
        self.publish_now(force=True)

    def get_active_map(self) -> str | None:
        with self._lock:
            return self._active_map

    def set_map_status(self, status: str):
        previous = self.get_map_status()
        with self._lock:
            self._map_status = status
        if previous == status:
            self._maybe_publish_full_map()
            return
        if self.is_recording():
            self._maybe_publish_full_map()
            return
        if previous != "mapping" and status == "mapping":
            self.clear_map_buffer(publish=False)
        if previous == "mapping" and status != "mapping":
            self.save_active_map_cache(force=True)
        self.publish_now(force=True)

    def get_map_status(self) -> str:
        with self._lock:
            return self._map_status

    def is_recording(self) -> bool:
        with self._lock:
            return self._manual_recording

    def start_record_cloud(self, map_name: str, overwrite: bool = True) -> dict:
        if not map_name:
            return {"error": "map_name is required"}
        cloud_path = self._cloud_path_for_map(map_name)
        if os.path.exists(cloud_path) and not overwrite:
            return {"error": f"cached cloud already exists for '{map_name}'", "cloud_path": cloud_path}

        if self.get_active_map() != map_name:
            self.save_active_map_cache(force=True)
        with self._lock:
            self._active_map = map_name
            self._native_pcd_path = cloud_path
            self._cached_cloud_path = cloud_path
            self._map_status = "recording"
            self._point_source = "recording"
            self._local_pcd_available = False
            self._manual_recording = True
            self._recording_map_name = map_name
        self.clear_map_buffer(publish=False)
        try:
            self._db.ensure_map(map_name, cloud_path, cloud_path)
        except Exception as e:
            print(f"[ControlledSpatialMap] failed to ensure record map {map_name}: {e}", flush=True)
        self.publish_now(force=True)
        print(f"[ControlledSpatialMap] recording cloud started: {map_name} -> {cloud_path}", flush=True)
        return {"status": "recording", "map_name": map_name, "cloud_path": cloud_path}

    def stop_record_cloud(self) -> dict:
        with self._lock:
            map_name = self._recording_map_name or self._active_map
            was_recording = self._manual_recording
        saved = self.save_active_map_cache(force=True)
        with self._lock:
            self._manual_recording = False
            self._recording_map_name = None
            if self._map_status == "recording":
                self._map_status = "idle"
            if saved:
                self._point_source = "cached_cloud"
        self.publish_now(force=True)
        print(
            f"[ControlledSpatialMap] recording cloud stopped: map={map_name} saved={saved}",
            flush=True,
        )
        return {"status": "stopped", "map_name": map_name, "was_recording": was_recording, "saved": saved}

    def clear_map_buffer(self, publish: bool = True):
        with self._map_buffer_lock:
            self._map_buffer.clear()
            self._pending_voxel_hits.clear()
            self._map_buffer_dirty = False
            self._map_buffer_revision += 1
            self._render_cloud_points = ControlledSpatialMapNode.np.zeros((0, 3), dtype=ControlledSpatialMapNode.np.float32)
            self._render_cloud_revision = self._map_buffer_revision
        with self._lock:
            self._point_source = "none"
        if publish:
            self.publish_now(force=True)

    def load_cached_cloud_to_buffer(self, cloud_path: str):
        points = self._load_cloud_cache(cloud_path)
        if points is None or len(points) == 0:
            print(f"[ControlledSpatialMap] cached cloud not found: {cloud_path}", flush=True)
            self.clear_map_buffer(publish=False)
            with self._lock:
                self._point_source = "none"
                self._local_pcd_available = False
            self.publish_now(force=True)
            return False

        self._replace_buffer_from_points(points)
        with self._lock:
            self._cached_cloud_path = cloud_path
            self._point_source = "cached_cloud"
            self._local_pcd_available = True
        print(
            f"[ControlledSpatialMap] loaded cached cloud: {cloud_path} ({len(points)} points)",
            flush=True,
        )
        self.publish_now(force=True)
        return True

    def load_pcd_to_buffer(self, pcd_path: str):
        with self._lock:
            self._native_pcd_path = pcd_path
            self._local_pcd_available = False
            self._point_source = "none"
        with self._map_buffer_lock:
            self._map_buffer.clear()
            self._pending_voxel_hits.clear()
            self._map_buffer_revision += 1
            self._render_cloud_points = ControlledSpatialMapNode.np.zeros((0, 3), dtype=ControlledSpatialMapNode.np.float32)
            self._render_cloud_revision = self._map_buffer_revision

        points = self._parse_pcd(pcd_path)
        if points is None or len(points) == 0:
            print(
                f"[ControlledSpatialMap] local PCD not visible, waiting for SLAM DDS points: {pcd_path}"
            )
            self.publish_now(force=True)
            return False

        self._replace_buffer_from_points(points)
        print(f"[ControlledSpatialMap] loaded {len(points)} PCD points from {pcd_path}")
        with self._lock:
            self._local_pcd_available = True
            self._point_source = "local_pcd"
        self.publish_now(force=True)
        return True

    def _on_mapping_cloud(self, msg) -> None:
        try:
            data = bytes(msg.data)
            if len(data) < msg.point_step:
                return
            self._enqueue_cloud("mapping", msg.fields, msg.point_step, msg.width * msg.height, data)
        except Exception:
            pass

    def _enqueue_cloud(self, source: str, fields, point_step: int, total: int, data: bytes) -> None:
        item = (source, fields, point_step, total, data)
        try:
            self._cloud_queue.put_nowait(item)
            return
        except queue.Full:
            pass
        try:
            self._cloud_queue.get_nowait()
        except Exception:
            pass
        try:
            self._cloud_queue.put_nowait(item)
        except Exception:
            pass

    def _cloud_processor_loop(self):
        np = ControlledSpatialMapNode.np
        while self._running:
            try:
                source, fields, point_step, total_points, data = self._cloud_queue.get(timeout=1.0)
            except Exception:
                continue
            if total_points == 0:
                continue

            with self._lock:
                status = self._map_status
                recording = self._manual_recording
            if status != "mapping" and not recording:
                cloud_log_counter = getattr(self, "_cloud_log_counter", 0) + 1
                self._cloud_log_counter = cloud_log_counter
                if cloud_log_counter < 5 or cloud_log_counter % 100 == 0:
                    print(
                        f"[ControlledSpatialMap] cloud ignored: source={source} "
                        f"pts={total_points} status={status} (#{cloud_log_counter})",
                        flush=True,
                    )
                continue

            cloud_log_counter = getattr(self, "_cloud_log_counter", 0) + 1
            self._cloud_log_counter = cloud_log_counter
            if cloud_log_counter < 5 or cloud_log_counter % 100 == 0:
                buf_size = len(self._map_buffer) if hasattr(self, '_map_buffer') else 0
                print(f"[ControlledSpatialMap] cloud: source={source} pts={total_points} "
                      f"status={status} buffer={buf_size} (#{cloud_log_counter})", flush=True)

            field_map = {f.name: f.offset for f in fields}
            x_off = field_map.get("x", 0)
            y_off = field_map.get("y", 4)
            z_off = field_map.get("z", 8)
            num_points = min(total_points, 20000)
            if len(data) < num_points * point_step:
                num_points = len(data) // point_step
            if num_points == 0:
                continue

            raw = np.frombuffer(data, dtype=np.uint8, count=num_points * point_step)
            raw = raw.reshape(num_points, point_step)
            x = raw[:, x_off:x_off + 4].view(np.float32).ravel()
            y = raw[:, y_off:y_off + 4].view(np.float32).ravel()
            z = raw[:, z_off:z_off + 4].view(np.float32).ravel()

            valid = (
                np.isfinite(x) & np.isfinite(y) & np.isfinite(z) &
                (np.abs(x) < 50) & (np.abs(y) < 50) & (np.abs(z) < 20)
            )
            pts = np.column_stack([x[valid], y[valid], z[valid]]).astype(np.float32)
            if len(pts) == 0:
                continue

            voxel_size = self.VOXEL_SIZE
            ix = (pts[:, 0] / voxel_size).astype(np.int32)
            iy = (pts[:, 1] / voxel_size).astype(np.int32)
            iz = (pts[:, 2] / voxel_size).astype(np.int32)

            frame_voxels = {}
            for j in range(len(pts)):
                frame_voxels[(int(ix[j]), int(iy[j]), int(iz[j]))] = (
                    float(pts[j, 0]), float(pts[j, 1]), float(pts[j, 2])
                )

            with self._map_buffer_lock:
                prev_size = len(self._map_buffer)
                for key, point in frame_voxels.items():
                    if key in self._map_buffer:
                        self._map_buffer[key] = point
                        continue
                    hits = self._pending_voxel_hits.get(key, 0) + 1
                    if hits >= self.DYNAMIC_MIN_HITS:
                        self._map_buffer[key] = point
                        self._pending_voxel_hits.pop(key, None)
                    else:
                        self._pending_voxel_hits[key] = hits
                new_size = len(self._map_buffer)
                if new_size > prev_size:
                    self._map_buffer_dirty = True
                if len(self._pending_voxel_hits) > self._MAX_PENDING_VOXELS:
                    pending_keys = list(self._pending_voxel_hits.keys())
                    for k in pending_keys[:len(pending_keys) // 2]:
                        self._pending_voxel_hits.pop(k, None)
                # Cap buffer size: if mapping accumulated too many voxels,
                # clear oldest half to make room for new scans.
                if len(self._map_buffer) > self._MAX_BUFFER_SIZE:
                    keys = list(self._map_buffer.keys())
                    for k in keys[:len(keys) // 2]:
                        del self._map_buffer[k]
                    new_size = len(self._map_buffer)
                    self._map_buffer_dirty = True
                # Existing voxels are refreshed too: SLAM loop closure can
                # move their coordinates even when the voxel count is stable.
                if frame_voxels:
                    self._map_buffer_revision += 1
            with self._lock:
                self._cloud_source = source
                self._point_source = "dds"
            if cloud_log_counter < 5 or cloud_log_counter % 100 == 0:
                print(
                    f"[ControlledSpatialMap] cloud buffer: source={source} "
                    f"valid={len(pts)} voxels={len(frame_voxels)} "
                    f"+{max(0, new_size - prev_size)} total={new_size}",
                    flush=True,
                )
            self.save_active_map_cache(force=False)
            self._maybe_publish_full_map()

    def _replace_buffer_from_points(self, points):
        voxel_size = self.VOXEL_SIZE
        with self._map_buffer_lock:
            self._map_buffer.clear()
            self._pending_voxel_hits.clear()
            for i in range(len(points)):
                ix = int(points[i, 0] / voxel_size)
                iy = int(points[i, 1] / voxel_size)
                iz = int(points[i, 2] / voxel_size)
                self._map_buffer[(ix, iy, iz)] = (
                    float(points[i, 0]), float(points[i, 1]), float(points[i, 2])
                )
            self._map_buffer_dirty = False
            self._map_buffer_revision += 1
            self._render_cloud_points = ControlledSpatialMapNode.np.zeros((0, 3), dtype=ControlledSpatialMapNode.np.float32)
            self._render_cloud_revision = -1

    def save_active_map_cache(self, force: bool = False):
        np = ControlledSpatialMapNode.np
        now = time.monotonic()
        if not force and now - self._last_cloud_save_time < self.CLOUD_SAVE_INTERVAL:
            return False
        with self._lock:
            active_map = self._active_map
            cloud_path = self._cached_cloud_path or (self._cloud_path_for_map(active_map) if active_map else None)
        if not active_map or not cloud_path:
            return False

        with self._map_buffer_lock:
            if not self._map_buffer:
                return False
            if not force and not self._map_buffer_dirty:
                return False
            all_points = list(self._map_buffer.values())
            self._map_buffer_dirty = False

        os.makedirs(os.path.dirname(cloud_path), exist_ok=True)
        try:
            pts = np.array(all_points, dtype=np.float32)
            np.savez_compressed(cloud_path, points=pts)
            self._db.set_cloud_path(active_map, cloud_path)
            with self._lock:
                self._cached_cloud_path = cloud_path
            self._last_cloud_save_time = now
            print(
                f"[ControlledSpatialMap] saved cached cloud: {active_map} "
                f"points={len(pts)} path={cloud_path}",
                flush=True,
            )
            return True
        except Exception as e:
            print(f"[ControlledSpatialMap] failed to save cached cloud {cloud_path}: {e}", flush=True)
            return False

    def _load_cloud_cache(self, cloud_path: str):
        np = ControlledSpatialMapNode.np
        if not cloud_path or not os.path.exists(cloud_path):
            return None
        try:
            data = np.load(cloud_path)
            points = data["points"].astype(np.float32)
            if len(points.shape) != 2 or points.shape[1] < 3:
                return None
            return points[:, :3]
        except Exception as e:
            print(f"[ControlledSpatialMap] failed to load cached cloud {cloud_path}: {e}", flush=True)
            return None

    def _cloud_path_for_map(self, map_name: str) -> str:
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", map_name).strip("._")
        if not safe_name:
            safe_name = "map"
        return os.path.join(self._cloud_dir, f"{safe_name}.npz")

    def _maybe_publish_full_map(self):
        now = time.monotonic()
        if now - self._last_map_publish_time < self.MAP_PUBLISH_INTERVAL:
            return
        self._last_map_publish_time = now
        with self._map_buffer_lock:
            has_cloud_points = bool(self._map_buffer)
        # publish_now normally skips an empty cloud. Force the fallback
        # skeleton through so its robot pose stays live as well.
        self.publish_now(force=not has_cloud_points, force_meta=False)

    def _get_render_cloud_points(self):
        """Return a bounded cloud snapshot without rebuilding it for every pose update."""
        np = ControlledSpatialMapNode.np
        now = time.monotonic()
        with self._map_buffer_lock:
            revision = self._map_buffer_revision
            refresh_due = now - self._last_cloud_snapshot_time >= self.CLOUD_SNAPSHOT_INTERVAL
            needs_snapshot = (
                revision != self._render_cloud_revision and
                (refresh_due or len(self._render_cloud_points) == 0)
            )
            if not needs_snapshot:
                return self._render_cloud_points
            values = list(self._map_buffer.values())

        points = np.array(values, dtype=np.float32) if values else np.zeros((0, 3), dtype=np.float32)
        if len(points) > self.MAX_SEND_POINTS:
            step = max(1, math.ceil(len(points) / self.MAX_SEND_POINTS))
            points = points[::step][:self.MAX_SEND_POINTS]

        with self._map_buffer_lock:
            self._render_cloud_points = points
            self._render_cloud_revision = revision
            self._last_cloud_snapshot_time = now
        return points

    def publish_now(self, force: bool = False, force_meta: bool | None = None):
        if self._closing.is_set():
            return
        np = ControlledSpatialMapNode.np
        now = time.monotonic()
        include_meta = (
            (force if force_meta is None else force_meta) or
            now - self._last_meta_publish_time >= self.META_PUBLISH_INTERVAL
        )
        with self._lock:
            pose = dict(self._current_pose) if self._current_pose else None
            active_map = self._active_map
            map_status = self._map_status
            native_pcd_path = self._native_pcd_path
            local_pcd_available = self._local_pcd_available
            point_source = self._point_source

        map_pts = self._get_render_cloud_points()
        if len(map_pts) == 0 and not force:
            return

        if include_meta:
            try:
                maps = self._db.list_maps_with_pois()
                self._cached_maps = maps
                self._last_meta_publish_time = now
            except Exception as e:
                print(f"[ControlledSpatialMap] failed to read map metadata: {e}", flush=True)
                maps = self._cached_maps
        else:
            maps = self._cached_maps
        active_tags = []
        if active_map:
            active_tags = next((m.get("tags", []) for m in maps if m.get("name") == active_map), [])

        overlay_points = self._build_tag_overlay_points(active_tags)
        guard_points = self._build_view_guard_points()
        has_cloud_points = bool(len(map_pts))
        if not has_cloud_points:
            fallback_points = self._build_fallback_points(maps, active_map, pose)
            if point_source == "none":
                point_source = "fallback"
            now = time.monotonic()
            if now - self._last_fallback_log_time > 10:
                self._last_fallback_log_time = now
                print(
                    f"[ControlledSpatialMap] publishing fallback map skeleton: "
                    f"points={len(fallback_points)} tags={len(active_tags)} active_map={active_map}",
                    flush=True,
                )
            map_pts = np.array(fallback_points, dtype=np.float32)
        overlay_pts = (
            np.array(overlay_points, dtype=np.float32)
            if has_cloud_points and overlay_points
            else np.zeros((0, 3), dtype=np.float32)
        )
        guard_pts = np.array(guard_points, dtype=np.float32)
        map_point_count = len(map_pts)
        overlay_point_count = len(overlay_pts)
        max_map_points = max(0, self.MAX_SEND_POINTS - overlay_point_count - len(guard_pts))
        if max_map_points == 0:
            map_pts = np.zeros((0, 3), dtype=np.float32)
        elif len(map_pts) > max_map_points:
            # Stable stride sampling avoids allocating a random index array on
            # every refresh, and prevents the cloud from visually shimmering.
            step = max(1, math.ceil(len(map_pts) / max_map_points))
            map_pts = map_pts[::step][:max_map_points]
        publish_parts = [map_pts]
        if overlay_point_count:
            publish_parts.append(overlay_pts)
        publish_parts.append(guard_pts)
        pts = np.vstack(publish_parts)
        num_points = len(pts)

        robot_x = float(pose["x"]) if pose else 0.0
        robot_y = float(pose["y"]) if pose else 0.0
        slam_yaw = float(pose["yaw"]) if pose else 0.0
        # The stock renderer maps SLAM +Y to Three.js -Z and then applies a
        # second negative rotation. Send the display-frame yaw here, while
        # metadata retains the original SLAM value for map/tag semantics.
        robot_yaw = -slam_yaw

        meta_bytes = b""
        if include_meta:
            meta = {
                "version": 3,
                "active_map": active_map,
                "map_status": map_status,
                "native_pcd_path": native_pcd_path,
                "local_pcd_available": local_pcd_available,
                "point_source": point_source,
                "robot": {"x": robot_x, "y": robot_y, "yaw": slam_yaw, "pose_available": pose is not None},
                "maps": maps,
                "tags": active_tags,
                "map_points": map_point_count,
                "tag_overlay_points": overlay_point_count,
                "view_guard_points": len(guard_pts),
            }
            meta_bytes = json.dumps(meta, ensure_ascii=False).encode("utf-8")

        # v3: v2 mapping packet plus optional trailing metadata JSON.
        flags = 0x03 | (0x04 if include_meta else 0)
        payload = (
            struct.pack('<fffBI', robot_x, robot_y, robot_yaw, flags, num_points) +
            pts.tobytes()
        )
        if include_meta:
            payload += struct.pack('<I', len(meta_bytes)) + meta_bytes

        # The stop path takes this same lock before node destruction. Check
        # _closing again after waiting so late DDS callbacks become no-ops.
        with self._publish_lock:
            if self._closing.is_set():
                return
            from std_msgs.msg import UInt8MultiArray
            ros_msg = UInt8MultiArray()
            try:
                ros_msg.data = array('B', payload)
            except TypeError:
                ros_msg.data = list(payload)
            try:
                self._pub.publish(ros_msg)
            except Exception as e:
                if not self._closing.is_set():
                    print(f"[ControlledSpatialMap] map publish skipped: {e}", flush=True)

    def _build_fallback_points(self, maps: list[dict], active_map: str | None, pose: dict | None) -> list[tuple]:
        """Build a visible map skeleton for the stock mapping renderer when PCD/DDS points are unavailable."""
        points: list[tuple] = []
        selected = None
        if active_map:
            selected = next((m for m in maps if m.get("name") == active_map), None)
        if selected is None and maps:
            selected = maps[0]
        tags = selected.get("tags", []) if selected else []

        xs = [float(t["x"]) for t in tags if _is_finite(t.get("x"))]
        ys = [float(t["y"]) for t in tags if _is_finite(t.get("y"))]
        if pose:
            xs.append(float(pose.get("x", 0.0)))
            ys.append(float(pose.get("y", 0.0)))

        max_extent = 3.0
        if xs and ys:
            max_extent = max(max(abs(x) for x in xs), max(abs(y) for y in ys), 3.0) + 1.0
        max_extent = min(max_extent, 20.0)
        step = 0.25
        n = int((2 * max_extent) / step) + 1

        # Ground cross and border make an empty map visible in the unchanged frontend.
        for i in range(n):
            v = -max_extent + i * step
            points.append((v, 0.0, -0.04))
            points.append((0.0, v, -0.04))
            if i % 2 == 0:
                points.append((v, -max_extent, -0.05))
                points.append((v, max_extent, -0.05))
                points.append((-max_extent, v, -0.05))
                points.append((max_extent, v, -0.05))

        points.extend(self._build_tag_overlay_points(tags))
        return points

    def _build_tag_overlay_points(self, tags: list[dict]) -> list[tuple]:
        points: list[tuple] = []
        for tag in tags:
            if not _is_finite(tag.get("x")) or not _is_finite(tag.get("y")):
                continue
            x = float(tag["x"])
            y = float(tag["y"])
            yaw = float(tag.get("yaw") or 0.0)

            # Dense beacon geometry survives the unchanged point-cloud renderer.
            radii = (0.10, 0.18, 0.28)
            for radius in radii:
                for k in range(48):
                    a = 2.0 * math.pi * k / 48
                    points.append((x + math.cos(a) * radius, y + math.sin(a) * radius, 0.18))
            for k in range(28):
                z = 0.08 + k * 0.07
                points.append((x, y, z))
                points.append((x + 0.12, y, z))
                points.append((x - 0.12, y, z))
                points.append((x, y + 0.12, z))
                points.append((x, y - 0.12, z))
            for k in range(22):
                d = 0.08 + k * 0.05
                hx = x + math.cos(yaw) * d
                hy = y + math.sin(yaw) * d
                points.append((hx, hy, 0.55))
                points.append((hx, hy, 0.75))
            arrow_x = x + math.cos(yaw) * 1.1
            arrow_y = y + math.sin(yaw) * 1.1
            left = yaw + 2.55
            right = yaw - 2.55
            for k in range(10):
                d = 0.04 * k
                points.append((arrow_x + math.cos(left) * d, arrow_y + math.sin(left) * d, 0.65))
                points.append((arrow_x + math.cos(right) * d, arrow_y + math.sin(right) * d, 0.65))
        return points

    def _build_view_guard_points(self) -> list[tuple]:
        extent = self.VIEW_GUARD_EXTENT
        z = -0.02
        # The stock renderer caches the first geometry bounding sphere and
        # does not recalculate it after position updates. These remote points
        # make that sphere cover the practical pan/zoom area, so the real map
        # is not incorrectly frustum-culled while browsing it.
        return [
            (-extent, -extent, z),
            (-extent, extent, z),
            (extent, -extent, z),
            (extent, extent, z),
            (-extent, 0.0, z),
            (extent, 0.0, z),
            (0.0, -extent, z),
            (0.0, extent, z),
        ]

    def _periodic_publish(self):
        """Timer callback: always publish so frontend stays in sync even without DDS events."""
        self._maybe_publish_full_map()

    def _on_relocation_cloud(self, msg) -> None:
        """Queue relocation clouds; they are accumulated like mapping clouds for visualization."""
        try:
            data = bytes(msg.data)
            if len(data) < msg.point_step:
                return
            self._enqueue_cloud("relocation", msg.fields, msg.point_step, msg.width * msg.height, data)
        except Exception:
            pass

    @staticmethod
    def _parse_pcd(path: str):
        np = ControlledSpatialMapNode.np
        if not os.path.exists(path):
            return None
        try:
            with open(path, 'rb') as f:
                header_lines = []
                while True:
                    line = f.readline()
                    if not line:
                        return None
                    line_str = line.decode('ascii', errors='ignore').strip()
                    header_lines.append(line_str)
                    if line_str.startswith('DATA'):
                        break

                fields = []
                field_sizes = []
                num_points = 0
                data_type = "ascii"
                for hl in header_lines:
                    parts = hl.split()
                    if not parts:
                        continue
                    if parts[0] == "FIELDS":
                        fields = parts[1:]
                    elif parts[0] == "SIZE":
                        field_sizes = [int(s) for s in parts[1:]]
                    elif parts[0] == "POINTS":
                        num_points = int(parts[1])
                    elif parts[0] == "DATA":
                        data_type = parts[1].lower()
                if num_points == 0:
                    return None
                try:
                    xi, yi, zi = fields.index("x"), fields.index("y"), fields.index("z")
                except ValueError:
                    return None

                if data_type == "ascii":
                    points = []
                    for _ in range(num_points):
                        line = f.readline().decode('ascii', errors='ignore').strip()
                        vals = line.split()
                        if len(vals) <= max(xi, yi, zi):
                            continue
                        x, y, z = float(vals[xi]), float(vals[yi]), float(vals[zi])
                        if x == x and y == y and z == z:
                            points.append((x, y, z))
                    return np.array(points, dtype=np.float32) if points else None

                if data_type == "binary":
                    point_size = sum(field_sizes)
                    raw = f.read(num_points * point_size)
                    if len(raw) < num_points * point_size:
                        num_points = len(raw) // point_size
                    offsets = [0]
                    for size in field_sizes[:-1]:
                        offsets.append(offsets[-1] + size)
                    x_off, y_off, z_off = offsets[xi], offsets[yi], offsets[zi]
                    points = np.zeros((num_points, 3), dtype=np.float32)
                    for i in range(num_points):
                        base = i * point_size
                        points[i, 0] = struct.unpack_from('<f', raw, base + x_off)[0]
                        points[i, 1] = struct.unpack_from('<f', raw, base + y_off)[0]
                        points[i, 2] = struct.unpack_from('<f', raw, base + z_off)[0]
                    valid = ~np.isnan(points).any(axis=1)
                    return points[valid]
        except Exception as e:
            print(f"[ControlledSpatialMap] failed to parse PCD {path}: {e}")
        return None

class ControlledSpatialMapPlugin:
    """Standalone, read-only sensor plugin for viewing controlled_spatial maps."""

    PREFIX = "controlled_spatial_map"

    def __init__(self, plugin_config: dict, namespace: str, executor, *_, **__):
        self._isolated = _as_bool(plugin_config.get("isolated_process", True))
        self._proc = None
        self._command_queue = None
        self._result_queue = None
        self._ipc_lock = threading.Lock()
        self._request_id = 0

        # Point-cloud decoding, voxelization, compression, and packet creation
        # are CPU-heavy Python work. Run the complete map runtime in a fresh
        # process so it cannot starve the driver process and other cards.
        if self._isolated:
            self._start_isolated_process(plugin_config, namespace)
            return

        self._map_topic = f"/{namespace}/controlled_spatial/map"
        self._db_path = plugin_config.get(
            "native_slam_db_path",
            "/opt/phanthy-motus/data/controlled_spatial.db",
        )
        self._cloud_dir = plugin_config.get(
            "controlled_cloud_dir",
            "/opt/phanthy-motus/data/controlled_spatial_clouds",
        )
        self._db = None
        self._map_node = None
        self._dds_subs = []
        self._slam_info_count = 0
        self._startup_error = None
        self._last_selected_map = None

        try:
            self._db = _MapDB(self._db_path)
            self._map_node = ControlledSpatialMapNode(self._map_topic, self._db, self._cloud_dir)
            executor.add_node(self._map_node.node)
            self._sync_from_db(force=True)
        except Exception as e:
            self._startup_error = str(e)
            print(f"[ControlledSpatialMap] startup degraded: {e}", flush=True)
            try:
                import traceback
                traceback.print_exc()
            except Exception:
                pass
            return

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            info_sub = ChannelSubscriber("rt/slam_info", String_)
            info_sub.Init(self._on_slam_info, 10)
            self._dds_subs.append(info_sub)
            print("[ControlledSpatialMap] subscribed rt/slam_info")
        except Exception as e:
            print(f"[ControlledSpatialMap] failed to subscribe rt/slam_info: {e}", flush=True)

        print(f"[ControlledSpatialMap] standalone plugin ready, topic: {self._map_topic}", flush=True)

    def _start_isolated_process(self, plugin_config: dict, namespace: str):
        self._map_topic = f"/{namespace}/controlled_spatial/map"
        self._cloud_dir = plugin_config.get(
            "controlled_cloud_dir",
            "/opt/phanthy-motus/data/controlled_spatial_clouds",
        )
        self._startup_error = None
        ctx = mp.get_context("spawn")
        self._command_queue = ctx.Queue()
        self._result_queue = ctx.Queue()
        child_config = dict(plugin_config)
        child_config["isolated_process"] = False
        self._proc = ctx.Process(
            target=_controlled_spatial_map_process,
            args=(child_config, namespace, self._command_queue, self._result_queue),
            daemon=True,
            name="controlled_spatial_map",
        )
        self._proc.start()
        try:
            result = self._result_queue.get(timeout=20.0)
        except queue.Empty:
            self._startup_error = "map process startup timed out"
            return
        if not result.get("ready"):
            self._startup_error = result.get("error", "map process failed to start")
            return
        print(f"[ControlledSpatialMap] isolated process ready, topic: {self._map_topic}", flush=True)

    def _call_process(self, action: str, args: dict, timeout: float = 15.0):
        if not self._proc or not self._proc.is_alive() or not self._command_queue or not self._result_queue:
            return {"error": self._startup_error or "map process is not running"}
        with self._ipc_lock:
            self._request_id += 1
            request_id = self._request_id
            self._command_queue.put({"id": request_id, "action": action, "args": dict(args)})
            try:
                while True:
                    result = self._result_queue.get(timeout=timeout)
                    if result.get("id") == request_id:
                        return result.get("result")
            except queue.Empty:
                return {"error": f"map process action '{action}' timed out"}

    def get_tools(self) -> list:
        return [self.get_tool()]

    def get_tool(self) -> dict:
        return {
            "name": self.PREFIX,
            "type": "sensor",
            "multiInstance": False,
            "description": (
                "Controlled spatial map view — saved maps, tags, live robot pose, "
                "and SLAM point cloud when available."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "start", "stop", "info", "refresh", "select_map", "list_maps",
                            "save_cloud", "record_cloud", "stop_record_cloud",
                        ],
                        "description": "Optional map-view control action",
                    },
                    "map_name": {"type": "string", "description": "Map name for select_map"},
                    "overwrite": {"type": "boolean", "description": "Whether record_cloud may overwrite an existing cached cloud"},
                },
            },
            "topic_out": [{"topic": self._map_topic, "format": "sensor/mapping"}],
        }

    def start(self) -> None:
        if self._isolated:
            self._call_process("start", {}, timeout=20.0)
            return
        if not self._map_node:
            return
        self._sync_from_db(force=True)
        self._map_node.publish_now(force=True)

    def stop(self) -> None:
        if self._isolated:
            if self._proc and self._proc.is_alive() and self._command_queue:
                try:
                    self._command_queue.put({"action": "__shutdown__"})
                    self._proc.join(timeout=5.0)
                except Exception:
                    pass
            if self._proc and self._proc.is_alive():
                self._proc.terminate()
                self._proc.join(timeout=2.0)
            return
        if self._map_node:
            self._map_node.stop()

    def dispatch(self, action: str, args: dict) -> dict | None:
        if self._isolated:
            return self._call_process(action, args)
        if action in (self.PREFIX, "start", "refresh"):
            if self._map_node:
                self._sync_from_db(force=True)
                self._map_node.publish_now(force=True)
            return self._info("ready")
        if action == "stop":
            return self._info("idle")
        if action == "info":
            return self._info("running")
        if action == "list_maps":
            return self._list_maps()
        if action == "save_cloud":
            if not self._map_node:
                return self._info("degraded")
            saved = self._map_node.save_active_map_cache(force=True)
            info = self._info("saved" if saved else "ready")
            info["saved"] = saved
            return info
        if action == "record_cloud":
            if not self._db or not self._map_node:
                return self._info("degraded")
            overwrite = _as_bool(args.get("overwrite", True))
            return self._map_node.start_record_cloud(args.get("map_name", ""), overwrite=overwrite)
        if action == "stop_record_cloud":
            if not self._map_node:
                return self._info("degraded")
            return self._map_node.stop_record_cloud()
        if action == "select_map":
            return self._select_map(args.get("map_name", ""))
        return None

    def _info(self, state: str) -> dict:
        maps = []
        if self._db:
            try:
                maps = self._db.list_maps_with_pois()
            except Exception:
                maps = []
        return {
            "state": state if not self._startup_error else "degraded",
            "topic_out": [{"topic": self._map_topic, "format": "sensor/mapping"}],
            "active_map": self._map_node.get_active_map() if self._map_node else None,
            "cloud_dir": self._cloud_dir,
            "map_count": len(maps),
            "recording": self._map_node.is_recording() if self._map_node else False,
            "slam_info_count": self._slam_info_count,
            "startup_error": self._startup_error,
        }

    def _list_maps(self) -> dict:
        if not self._db:
            return self._info("degraded")
        try:
            maps = self._db.list_maps_with_pois()
        except Exception as e:
            return {"error": f"failed to read maps: {e}"}
        return {
            "maps": maps,
            "active_map": self._map_node.get_active_map() if self._map_node else None,
            "cloud_dir": self._cloud_dir,
            "recording": self._map_node.is_recording() if self._map_node else False,
            "map_count": len(maps),
        }

    def _select_map(self, map_name: str) -> dict:
        if not self._db or not self._map_node:
            return self._info("degraded")
        if not map_name:
            return {"error": "map_name is required"}
        try:
            maps = self._db.list_maps_with_pois()
        except Exception as e:
            return {"error": f"failed to read maps: {e}"}
        found = next((m for m in maps if m.get("name") == map_name), None)
        if not found:
            return {"error": f"Map '{map_name}' not found", "maps": [m.get("name") for m in maps]}
        if self._map_node.is_recording() and self._map_node.get_active_map() != map_name:
            return {
                "error": "Cannot select another map while record_cloud is active",
                "active_map": self._map_node.get_active_map(),
                "recording": True,
            }
        status = self._db.get_state("map_status") or self._map_node.get_map_status()
        db_active = self._db.get_state("active_map")
        if status == "mapping" and db_active and db_active != map_name:
            return {
                "error": "Cannot select another map while recording mapping cloud",
                "active_map": db_active,
                "map_status": status,
            }
        self._last_selected_map = map_name
        cloud_path = found.get("cloud_path") or self._map_node._cloud_path_for_map(map_name)
        self._map_node.set_active_map_info(
            found.get("name"),
            found.get("pcd_path"),
            cloud_path=cloud_path,
            load_cached=True,
        )
        if not os.path.exists(cloud_path):
            # This is only a local preview attempt. The real map is normally on the SLAM computer,
            # so failure here is expected and must not affect the control card.
            self._map_node.load_pcd_to_buffer(found.get("pcd_path", ""))
        return self._info("selected")

    def _sync_from_db(self, force: bool = False) -> None:
        if not self._db or not self._map_node:
            return
        if self._map_node.is_recording():
            return
        try:
            maps = self._db.list_maps_with_pois()
            status = self._db.get_state("map_status") or self._map_node.get_map_status()
            db_active = self._db.get_state("active_map")
            desired = self._last_selected_map or self._db.get_state("active_map")
            if status == "mapping" and db_active:
                desired = db_active
                self._last_selected_map = None
            if desired and not any(m.get("name") == desired for m in maps):
                desired = None
            if not desired and maps:
                desired = maps[0].get("name")
            if desired:
                selected = next((m for m in maps if m.get("name") == desired), None)
                active_changed = self._map_node.get_active_map() != desired
                if selected and (force or active_changed):
                    self._map_node.set_active_map_info(
                        selected.get("name"),
                        selected.get("pcd_path"),
                        cloud_path=selected.get("cloud_path"),
                        load_cached=status != "mapping",
                        clear_for_new=status == "mapping" and active_changed,
                    )
            if status:
                self._map_node.set_map_status(status)
        except Exception as e:
            print(f"[ControlledSpatialMap] DB sync skipped: {e}", flush=True)

    def _on_slam_info(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        msg_type = data.get("type", "")
        if msg_type not in ("pos_info", "mapping_info"):
            return

        self._slam_info_count += 1
        if self._slam_info_count <= 5:
            print(f"[ControlledSpatialMap] slam_info: type={msg_type}", flush=True)

        pose_data = data.get("data", {}).get("currentPose")
        if not pose_data or not self._map_node:
            return

        q_x = float(pose_data.get("q_x", 0.0))
        q_y = float(pose_data.get("q_y", 0.0))
        q_z = float(pose_data.get("q_z", 0.0))
        q_w = float(pose_data.get("q_w", 1.0))
        yaw = math.atan2(
            2 * (q_w * q_z + q_x * q_y),
            1 - 2 * (q_y * q_y + q_z * q_z),
        )
        pose = {
            "x": pose_data["x"],
            "y": pose_data["y"],
            "yaw": round(yaw, 3),
        }

        current_status = self._map_node.get_map_status()
        if msg_type == "mapping_info":
            map_status = "mapping"
        elif current_status == "mapping":
            map_status = "mapping"
        else:
            map_status = "localized"
        self._map_node.update_pose(pose, map_status)
        self._sync_from_db(force=False)


def _controlled_spatial_map_process(plugin_config: dict, namespace: str, command_queue, result_queue):
    """Own the heavy mapping runtime outside the main driver process."""
    plugin = None
    executor = None
    try:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(0, plugin_config.get("network_iface", "eth0"))
        rclpy.init(args=None)
        executor = MultiThreadedExecutor(num_threads=2)
        plugin = ControlledSpatialMapPlugin(plugin_config, namespace, executor)
        if plugin._startup_error:
            result_queue.put({"ready": False, "error": plugin._startup_error})
            return

        spin_thread = threading.Thread(target=executor.spin, daemon=True, name="controlled_spatial_map_ros")
        spin_thread.start()
        result_queue.put({"ready": True})

        while True:
            command = command_queue.get()
            if command.get("action") == "__shutdown__":
                break
            request_id = command.get("id")
            try:
                result = plugin.dispatch(command.get("action", ""), command.get("args", {}))
            except Exception as e:
                result = {"error": f"map process action failed: {e}"}
            result_queue.put({"id": request_id, "result": result})
    except Exception as e:
        try:
            result_queue.put({"ready": False, "error": str(e)})
        except Exception:
            pass
    finally:
        try:
            if plugin:
                plugin.stop()
        except Exception:
            pass
        try:
            if executor:
                executor.shutdown()
        except Exception:
            pass
        try:
            if 'rclpy' in locals() and rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def make_plugin(plugin_config, namespace, executor, client=None):
    return ControlledSpatialMapPlugin(plugin_config, namespace, executor)


def _is_finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() not in ("0", "false", "no", "off")
    return bool(value)
