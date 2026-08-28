"""Map view publisher for the T800 controlled_spatial_map sensor card.

Subscribes to T800 ROS2 odometry and SLAM point cloud, voxel-accumulates the
cloud while mapping, and publishes a v3 binary map snapshot (UInt8MultiArray)
for the frontend controlled-spatial renderer. DB is shared with the
controlled_spatial actuator (mapping.db).
"""

from __future__ import annotations

import json
import math
import os
import queue
import re
import struct
import threading
import time
from array import array

from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from device import _MappingDB, _BEST_EFFORT


def _is_finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


class ControlledSpatialMapNode:
    """Publishes a monitor-friendly map snapshot for the controlled spatial card."""

    np = None

    VOXEL_SIZE = 0.06
    MAP_PUBLISH_INTERVAL = 0.15
    CLOUD_SNAPSHOT_INTERVAL = 0.50
    META_PUBLISH_INTERVAL = 1.0
    MAX_SEND_POINTS = 16000
    DYNAMIC_MIN_HITS = 2
    VIEW_GUARD_EXTENT = 300.0

    def __init__(
        self,
        topic: str,
        db: _MappingDB,
        cloud_dir: str,
        ros2,
        odometry_topic: str,
        pointcloud_topic: str,
    ):
        import numpy as np

        ControlledSpatialMapNode.np = np
        os.makedirs(cloud_dir, exist_ok=True)

        # Subscriptions live on the robot domain (odometry + point cloud),
        # while the map publisher must be on the core domain so the frontend
        # / agent-core can receive it.
        self._sub_node = Node("t800_controlled_spatial_map_sub", context=ros2.ctx_robot)
        ros2.executor_robot.add_node(self._sub_node)
        self._pub_node = Node("t800_controlled_spatial_map_pub", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._pub_node)
        self._pub = self._pub_node.create_publisher(UInt8MultiArray, topic, 10)
        self._topic = topic
        self._db = db
        self._cloud_dir = cloud_dir

        # --- state ---
        self._current_pose: dict | None = None
        self._map_status = "idle"
        self._active_map: str | None = None
        self._point_source = "none"
        self._lock = threading.Lock()

        # --- voxel buffer ---
        self._map_buffer: dict[tuple, tuple] = {}
        self._pending_voxel_hits: dict[tuple, int] = {}
        self._map_buffer_lock = threading.Lock()
        self._map_buffer_dirty = False
        self._map_buffer_revision = 0
        self._render_cloud_points = np.zeros((0, 3), dtype=np.float32)
        self._render_cloud_revision = -1
        self._last_cloud_snapshot_time = 0.0
        self._MAX_BUFFER_SIZE = 200000
        self._MAX_PENDING_VOXELS = 300000

        # --- cloud back-pressure queue + worker ---
        self._cloud_queue: queue.Queue = queue.Queue(maxsize=1)
        self._running = True
        self._closing = threading.Event()
        self._publish_lock = threading.Lock()
        self._last_map_publish_time = 0.0
        self._last_meta_publish_time = 0.0
        self._last_fallback_log_time = 0.0
        self._cached_maps: list[dict] = []
        self._cloud_log_counter = 0

        self._worker = threading.Thread(
            target=self._cloud_processor_loop,
            daemon=True,
            name="t800_controlled_spatial_cloud",
        )
        self._worker.start()

        # --- ROS2 subscriptions (robot domain) ---
        self._odom_sub = self._sub_node.create_subscription(
            Odometry, odometry_topic, self._on_odometry, _BEST_EFFORT
        )
        self._cloud_sub = self._sub_node.create_subscription(
            PointCloud2, pointcloud_topic, self._on_pointcloud, _BEST_EFFORT
        )

        # --- fallback heartbeat timer (core domain) ---
        self._publish_timer = self._pub_node.create_timer(0.5, self._periodic_publish)

        print(
            f"[ControlledSpatialMapNode] ready topic={topic} "
            f"odom={odometry_topic} cloud={pointcloud_topic}",
            flush=True,
        )

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    @property
    def node(self):
        return self._pub_node

    def stop(self):
        if self._closing.is_set():
            return
        self._closing.set()
        self._running = False
        try:
            self._publish_timer.cancel()
        except Exception:
            pass
        try:
            self._worker.join(timeout=2)
        except Exception:
            pass
        try:
            with self._publish_lock:
                self._pub_node.destroy_node()
        except Exception:
            pass
        try:
            self._sub_node.destroy_node()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # pose / status accessors
    # ------------------------------------------------------------------
    def update_pose(self, pose: dict, map_status: str | None = None):
        previous = self.get_map_status()
        with self._lock:
            self._current_pose = dict(pose)
            if map_status is not None:
                self._map_status = map_status
        status_changed = map_status is not None and previous != map_status
        if status_changed:
            if previous != "mapping" and map_status == "mapping":
                self.clear_map_buffer(publish=False)
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
        with self._lock:
            self._active_map = name
            if clear_for_new or not self._map_buffer:
                self._point_source = "none"
        if clear_for_new:
            self.clear_map_buffer(publish=False)
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
        if previous != "mapping" and status == "mapping":
            self.clear_map_buffer(publish=False)
        self.publish_now(force=True)

    def get_map_status(self) -> str:
        with self._lock:
            return self._map_status

    # ------------------------------------------------------------------
    # buffer management
    # ------------------------------------------------------------------
    def clear_map_buffer(self, publish: bool = True):
        np = ControlledSpatialMapNode.np
        with self._map_buffer_lock:
            self._map_buffer.clear()
            self._pending_voxel_hits.clear()
            self._map_buffer_dirty = False
            self._map_buffer_revision += 1
            self._render_cloud_points = np.zeros((0, 3), dtype=np.float32)
            self._render_cloud_revision = self._map_buffer_revision
        with self._lock:
            self._point_source = "none"
        if publish:
            self.publish_now(force=True)

    def load_pcd_to_buffer(self, pcd_path: str) -> bool:
        np = ControlledSpatialMapNode.np
        with self._map_buffer_lock:
            self._map_buffer.clear()
            self._pending_voxel_hits.clear()
            self._map_buffer_revision += 1
            self._render_cloud_points = np.zeros((0, 3), dtype=np.float32)
            self._render_cloud_revision = self._map_buffer_revision

        points = self._parse_pcd(pcd_path)
        if points is None or len(points) == 0:
            print(f"[ControlledSpatialMapNode] PCD not found or empty: {pcd_path}", flush=True)
            self.publish_now(force=True)
            return False

        self._replace_buffer_from_points(points)
        print(f"[ControlledSpatialMapNode] loaded {len(points)} PCD points from {pcd_path}", flush=True)
        with self._lock:
            self._point_source = "local_pcd"
        self.publish_now(force=True)
        return True

    def _replace_buffer_from_points(self, points):
        np = ControlledSpatialMapNode.np
        voxel_size = self.VOXEL_SIZE
        with self._map_buffer_lock:
            self._map_buffer.clear()
            self._pending_voxel_hits.clear()
            for i in range(len(points)):
                ix = int(points[i, 0] / voxel_size)
                iy = int(points[i, 1] / voxel_size)
                iz = int(points[i, 2] / voxel_size)
                self._map_buffer[(ix, iy, iz)] = (
                    float(points[i, 0]),
                    float(points[i, 1]),
                    float(points[i, 2]),
                )
            self._map_buffer_dirty = False
            self._map_buffer_revision += 1
            self._render_cloud_points = np.zeros((0, 3), dtype=np.float32)
            self._render_cloud_revision = -1

    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------
    def _on_odometry(self, msg: Odometry):
        try:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            yaw = math.atan2(
                2 * (ori.w * ori.z + ori.x * ori.y),
                1 - 2 * (ori.y * ori.y + ori.z * ori.z),
            )
            with self._lock:
                self._current_pose = {"x": float(pos.x), "y": float(pos.y), "yaw": yaw}
        except Exception as e:
            if not self._closing.is_set():
                print(f"[ControlledSpatialMapNode] odometry callback error: {e}", flush=True)

    def _on_pointcloud(self, msg: PointCloud2):
        try:
            data = bytes(msg.data)
            if len(data) < msg.point_step:
                return
            total = msg.width * msg.height
            self._enqueue_cloud(msg.fields, msg.point_step, total, data)
        except Exception:
            pass

    def _enqueue_cloud(self, fields, point_step: int, total: int, data: bytes):
        item = (fields, point_step, total, data)
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

    # ------------------------------------------------------------------
    # cloud worker
    # ------------------------------------------------------------------
    def _cloud_processor_loop(self):
        np = ControlledSpatialMapNode.np
        while self._running:
            try:
                fields, point_step, total_points, data = self._cloud_queue.get(timeout=1.0)
            except Exception:
                continue
            if total_points == 0:
                continue

            with self._lock:
                status = self._map_status

            if status != "mapping":
                self._cloud_log_counter += 1
                if self._cloud_log_counter < 5 or self._cloud_log_counter % 100 == 0:
                    print(
                        f"[ControlledSpatialMapNode] cloud ignored pts={total_points} "
                        f"status={status} (#{self._cloud_log_counter})",
                        flush=True,
                    )
                continue

            self._cloud_log_counter += 1
            if self._cloud_log_counter < 5 or self._cloud_log_counter % 100 == 0:
                print(
                    f"[ControlledSpatialMapNode] cloud pts={total_points} "
                    f"status={status} buffer={len(self._map_buffer)} "
                    f"(#{self._cloud_log_counter})",
                    flush=True,
                )

            # --- parse xyz from PointCloud2 row layout ---
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
            x = raw[:, x_off : x_off + 4].view(np.float32).ravel()
            y = raw[:, y_off : y_off + 4].view(np.float32).ravel()
            z = raw[:, z_off : z_off + 4].view(np.float32).ravel()

            valid = (
                np.isfinite(x)
                & np.isfinite(y)
                & np.isfinite(z)
                & (np.abs(x) < 50)
                & (np.abs(y) < 50)
                & (np.abs(z) < 20)
            )
            pts = np.column_stack([x[valid], y[valid], z[valid]]).astype(np.float32)
            if len(pts) == 0:
                continue

            # --- voxelize with dynamic hit threshold ---
            voxel_size = self.VOXEL_SIZE
            ix = (pts[:, 0] / voxel_size).astype(np.int32)
            iy = (pts[:, 1] / voxel_size).astype(np.int32)
            iz = (pts[:, 2] / voxel_size).astype(np.int32)

            frame_voxels: dict[tuple, tuple] = {}
            for j in range(len(pts)):
                frame_voxels[(int(ix[j]), int(iy[j]), int(iz[j]))] = (
                    float(pts[j, 0]),
                    float(pts[j, 1]),
                    float(pts[j, 2]),
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
                    for k in pending_keys[: len(pending_keys) // 2]:
                        self._pending_voxel_hits.pop(k, None)
                if len(self._map_buffer) > self._MAX_BUFFER_SIZE:
                    keys = list(self._map_buffer.keys())
                    for k in keys[: len(keys) // 2]:
                        del self._map_buffer[k]
                    new_size = len(self._map_buffer)
                    self._map_buffer_dirty = True
                if frame_voxels:
                    self._map_buffer_revision += 1

            with self._lock:
                self._point_source = "live"

            if self._cloud_log_counter < 5 or self._cloud_log_counter % 100 == 0:
                print(
                    f"[ControlledSpatialMapNode] buffer valid={len(pts)} "
                    f"voxels={len(frame_voxels)} +{max(0, new_size - prev_size)} "
                    f"total={new_size}",
                    flush=True,
                )
            self._maybe_publish_full_map()

    # ------------------------------------------------------------------
    # publish
    # ------------------------------------------------------------------
    def _maybe_publish_full_map(self):
        now = time.monotonic()
        if now - self._last_map_publish_time < self.MAP_PUBLISH_INTERVAL:
            return
        self._last_map_publish_time = now
        with self._map_buffer_lock:
            has_cloud_points = bool(self._map_buffer)
        self.publish_now(force=not has_cloud_points, force_meta=False)

    def _get_render_cloud_points(self):
        np = ControlledSpatialMapNode.np
        now = time.monotonic()
        with self._map_buffer_lock:
            revision = self._map_buffer_revision
            refresh_due = now - self._last_cloud_snapshot_time >= self.CLOUD_SNAPSHOT_INTERVAL
            needs_snapshot = (
                revision != self._render_cloud_revision
                and (refresh_due or len(self._render_cloud_points) == 0)
            )
            if not needs_snapshot:
                return self._render_cloud_points
            values = list(self._map_buffer.values())

        points = (
            np.array(values, dtype=np.float32)
            if values
            else np.zeros((0, 3), dtype=np.float32)
        )
        if len(points) > self.MAX_SEND_POINTS:
            step = max(1, math.ceil(len(points) / self.MAX_SEND_POINTS))
            points = points[::step][: self.MAX_SEND_POINTS]

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
            (force if force_meta is None else force_meta)
            or now - self._last_meta_publish_time >= self.META_PUBLISH_INTERVAL
        )
        with self._lock:
            pose = dict(self._current_pose) if self._current_pose else None
            active_map = self._active_map
            map_status = self._map_status
            point_source = self._point_source

        map_pts = self._get_render_cloud_points()
        if len(map_pts) == 0 and not force:
            return

        # --- metadata (maps + tags) ---
        if include_meta:
            try:
                maps = self._db.list_maps_with_pois()
                self._cached_maps = maps
                self._last_meta_publish_time = now
            except Exception as e:
                print(f"[ControlledSpatialMapNode] failed to read map metadata: {e}", flush=True)
                maps = self._cached_maps
        else:
            maps = self._cached_maps

        # T800 _MappingDB.list_maps_with_pois returns tags as name-list only;
        # fetch full POI records for the active map so overlay geometry works.
        active_tags: list[dict] = []
        if active_map:
            try:
                active_tags = self._db.list_pois(active_map)
            except Exception:
                active_tags = []

        overlay_points = self._build_tag_overlay_points(active_tags)
        guard_points = self._build_view_guard_points()
        has_cloud_points = bool(len(map_pts))

        if not has_cloud_points:
            fallback_points = self._build_fallback_points(maps, active_map, pose, active_tags)
            if point_source == "none":
                point_source = "fallback"
            now = time.monotonic()
            if now - self._last_fallback_log_time > 10:
                self._last_fallback_log_time = now
                print(
                    f"[ControlledSpatialMapNode] fallback skeleton points={len(fallback_points)} "
                    f"tags={len(active_tags)} active_map={active_map}",
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
            # Stable stride sampling avoids shimmer and per-frame random-index alloc.
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
        # Frontend renderer maps SLAM +Y to Three.js -Z and applies a second
        # negative rotation; send the display-frame yaw here while metadata
        # keeps the original SLAM value for map/tag semantics.
        robot_yaw = -slam_yaw

        meta_bytes = b""
        if include_meta:
            meta = {
                "version": 3,
                "active_map": active_map,
                "map_status": map_status,
                "native_pcd_path": None,
                "local_pcd_available": False,
                "point_source": point_source,
                "robot": {
                    "x": robot_x,
                    "y": robot_y,
                    "yaw": slam_yaw,
                    "pose_available": pose is not None,
                },
                "maps": maps,
                "tags": active_tags,
                "map_points": map_point_count,
                "tag_overlay_points": overlay_point_count,
                "view_guard_points": len(guard_pts),
            }
            meta_bytes = json.dumps(meta, ensure_ascii=False).encode("utf-8")

        # v3 binary: header (3xf32 + u8 flags + u32 count) + points + optional meta
        flags = 0x03 | (0x04 if include_meta else 0)
        payload = (
            struct.pack("<fffBI", robot_x, robot_y, robot_yaw, flags, num_points)
            + pts.tobytes()
        )
        if include_meta:
            payload += struct.pack("<I", len(meta_bytes)) + meta_bytes

        with self._publish_lock:
            if self._closing.is_set():
                return
            ros_msg = UInt8MultiArray()
            try:
                ros_msg.data = array("B", payload)
            except TypeError:
                ros_msg.data = list(payload)
            try:
                self._pub.publish(ros_msg)
            except Exception as e:
                if not self._closing.is_set():
                    print(f"[ControlledSpatialMapNode] map publish skipped: {e}", flush=True)

    # ------------------------------------------------------------------
    # geometry builders
    # ------------------------------------------------------------------
    def _build_fallback_points(
        self,
        maps: list[dict],
        active_map: str | None,
        pose: dict | None,
        active_tags: list[dict] | None = None,
    ) -> list[tuple]:
        """Ground cross + border skeleton so an empty map is still visible."""
        points: list[tuple] = []
        tags = active_tags or []
        if not tags and active_map:
            try:
                tags = self._db.list_pois(active_map)
            except Exception:
                tags = []

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
        """Dense beacon geometry that survives the unchanged point-cloud renderer."""
        points: list[tuple] = []
        for tag in tags:
            if not _is_finite(tag.get("x")) or not _is_finite(tag.get("y")):
                continue
            x = float(tag["x"])
            y = float(tag["y"])
            yaw = float(tag.get("yaw") or 0.0)

            # 3 concentric rings
            radii = (0.10, 0.18, 0.28)
            for radius in radii:
                for k in range(48):
                    a = 2.0 * math.pi * k / 48
                    points.append((x + math.cos(a) * radius, y + math.sin(a) * radius, 0.18))
            # vertical pillar (center + cross, 28 layers)
            for k in range(28):
                z = 0.08 + k * 0.07
                points.append((x, y, z))
                points.append((x + 0.12, y, z))
                points.append((x - 0.12, y, z))
                points.append((x, y + 0.12, z))
                points.append((x, y - 0.12, z))
            # direction arrow shaft
            for k in range(22):
                d = 0.08 + k * 0.05
                hx = x + math.cos(yaw) * d
                hy = y + math.sin(yaw) * d
                points.append((hx, hy, 0.55))
                points.append((hx, hy, 0.75))
            # arrow head
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
        """Remote points so the renderer's cached bounding sphere covers pan/zoom."""
        extent = self.VIEW_GUARD_EXTENT
        z = -0.02
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
        self._maybe_publish_full_map()

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _cloud_path_for_map(self, map_name: str) -> str:
        safe_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", map_name).strip("._")
        if not safe_name:
            safe_name = "map"
        return os.path.join(self._cloud_dir, f"{safe_name}.npz")

    @staticmethod
    def _parse_pcd(path: str):
        np = ControlledSpatialMapNode.np
        if not os.path.exists(path):
            return None
        try:
            with open(path, "rb") as f:
                header_lines = []
                while True:
                    line = f.readline()
                    if not line:
                        return None
                    line_str = line.decode("ascii", errors="ignore").strip()
                    header_lines.append(line_str)
                    if line_str.startswith("DATA"):
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
                        line = f.readline().decode("ascii", errors="ignore").strip()
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
                        points[i, 0] = struct.unpack_from("<f", raw, base + x_off)[0]
                        points[i, 1] = struct.unpack_from("<f", raw, base + y_off)[0]
                        points[i, 2] = struct.unpack_from("<f", raw, base + z_off)[0]
                    valid = ~np.isnan(points).any(axis=1)
                    return points[valid]
        except Exception as e:
            print(f"[ControlledSpatialMapNode] failed to parse PCD {path}: {e}", flush=True)
        return None


# ======================================================================
# MCP sensor plugin
# ======================================================================
class ControlledSpatialMapPlugin:
    """Read-only sensor plugin for viewing controlled_spatial maps on T800."""

    PREFIX = "controlled_spatial_map"

    def __init__(self, plugin_config: dict, namespace: str, ros2, *_, **__):
        self._map_topic = f"/{namespace}/controlled_spatial/map"
        db_path = plugin_config.get("db_path", "/opt/phanthy-motus/data/mapping.db")
        self._cloud_dir = plugin_config.get(
            "cloud_dir", "/opt/phanthy-motus/data/controlled_spatial_clouds"
        )
        odometry_topic = plugin_config.get(
            "odometry_topic", "/manifold/ODIN2/device0/odometry"
        )
        pointcloud_topic = plugin_config.get(
            "pointcloud_topic", "/manifold/ODIN2/device0/cloud/slam"
        )

        self._db = None
        self._map_node = None
        self._startup_error = None
        self._last_selected_map = None
        self._closing = threading.Event()
        self._sync_timer = None

        try:
            self._db = _MappingDB(db_path)
            self._map_node = ControlledSpatialMapNode(
                self._map_topic,
                self._db,
                self._cloud_dir,
                ros2,
                odometry_topic,
                pointcloud_topic,
            )
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

        # T800 has no DDS slam_info channel like g1; poll the shared DB once
        # per second so actuator-driven map_status changes are picked up.
        self._schedule_sync()

        print(f"[ControlledSpatialMap] plugin ready, topic: {self._map_topic}", flush=True)

    # ------------------------------------------------------------------
    # tool schema
    # ------------------------------------------------------------------
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
                        "enum": ["start", "stop", "info", "refresh", "select_map", "list_maps"],
                        "description": "Optional map-view control action",
                    },
                    "map_name": {
                        "type": "string",
                        "description": "Map name for select_map",
                    },
                    "overwrite": {"type": "boolean"},
                },
            },
            "topic_out": [{"topic": self._map_topic, "format": "sensor/mapping"}],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def start(self):
        if not self._map_node:
            return
        self._sync_from_db(force=True)
        self._map_node.publish_now(force=True)

    def stop(self):
        self._closing.set()
        if self._sync_timer is not None:
            self._sync_timer.cancel()
            self._sync_timer = None
        if self._map_node:
            self._map_node.stop()

    def _schedule_sync(self):
        if self._closing.is_set():
            return
        self._sync_timer = threading.Timer(1.0, self._periodic_sync)
        self._sync_timer.daemon = True
        self._sync_timer.start()

    def _periodic_sync(self):
        if self._closing.is_set():
            return
        try:
            self._sync_from_db(force=False)
        except Exception as e:
            print(f"[ControlledSpatialMap] periodic sync error: {e}", flush=True)
        self._schedule_sync()

    # ------------------------------------------------------------------
    # dispatch
    # ------------------------------------------------------------------
    def dispatch(self, action: str, args: dict) -> dict | None:
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
        if action == "select_map":
            return self._select_map(args.get("map_name", ""))
        return None

    # ------------------------------------------------------------------
    # action handlers
    # ------------------------------------------------------------------
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
            return {
                "error": f"Map '{map_name}' not found",
                "maps": [m.get("name") for m in maps],
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
        self._map_node.set_active_map_info(
            found.get("name"),
            found.get("pcd_path"),
            load_cached=False,
        )
        # Best-effort PCD preview; failure is non-fatal.
        pcd_path = found.get("pcd_path", "")
        if pcd_path:
            self._map_node.load_pcd_to_buffer(pcd_path)
        return self._info("selected")

    def _sync_from_db(self, force: bool = False) -> None:
        if not self._db or not self._map_node:
            return
        try:
            maps = self._db.list_maps_with_pois()
            status = self._db.get_state("map_status") or self._map_node.get_map_status()
            db_active = self._db.get_state("active_map")
            desired = self._last_selected_map or db_active
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
                        load_cached=False,
                        clear_for_new=status == "mapping" and active_changed,
                    )
            if status:
                self._map_node.set_map_status(status)
        except Exception as e:
            print(f"[ControlledSpatialMap] DB sync skipped: {e}", flush=True)


def make_plugin(plugin_config, namespace, ros2, client=None):
    return ControlledSpatialMapPlugin(plugin_config, namespace, ros2)
