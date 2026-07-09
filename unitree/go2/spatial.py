"""
SpatialPlugin — Go2 自动持续建图与智能导航。

与 ControlledSpatialPlugin 共存：
- ControlledSpatial: 手动控制建图（用户操控机器人走）
- Spatial: 自动持续建图（开机自动识别地图、localized后自动续建、Scan Context指纹）

功能：
- 自动建图：启动时自动识别/创建地图，定位后自动续建
- Scan Context：关键帧指纹存储与查询，自动发现当前所在地图
- 三个 sensor tools：pos_tag (10Hz)、slam_mapping (1Hz)、slam_cloud (5Hz)
- 一个 actuator tool：spatial (tag/navigate/mapping control)

DDS 订阅：
- rt/slam_info (String_): 实时位姿/状态
- rt/slam_key_info (String_): 任务执行反馈
- rt/unitree/slam_mapping/points (PointCloud2_): 建图实时点云
- rt/unitree/slam_relocation/points (PointCloud2_): 定位实时点云
"""

import json
import math
import os
import queue
import multiprocessing
import sqlite3
import struct
import threading
import time

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, UInt8MultiArray

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


# ── RPC Error Codes ──────────────────────────────────────────────────────────

RPC_ERROR_DESCRIPTIONS = {
    3102: "send request failed",
    3103: "API not registered",
    3104: "request timeout",
    3105: "request/response mismatch",
    3106: "invalid response data",
    3107: "invalid lease",
    3201: "server send error",
    3202: "server internal error",
    3203: "server API not implemented",
    3204: "server API parameter error",
    3205: "server lease denied",
}


def _rpc_error(action: str, code: int, resp=None) -> dict:
    desc = RPC_ERROR_DESCRIPTIONS.get(code, "unknown error")
    return {"error": f"{action} failed: {desc} (code={code})", "response": resp}


# ── SLAM RPC Subprocess Proxy ────────────────────────────────────────────────

def _slam_rpc_worker(cmd_queue: multiprocessing.Queue, result_queue: multiprocessing.Queue,
                     network_iface: str):
    """Subprocess: holds a dedicated SlamClient, processes RPC commands."""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    try:
        from unitree_sdk2py.g1.slam.slam_client import SlamClient
    except ImportError:
        print("[SpatialRpcWorker] SlamClient not available in SDK, worker disabled", flush=True)
        while True:
            try:
                cmd = cmd_queue.get()
            except Exception:
                return
            if cmd is None:
                return
            result_queue.put({"code": -1, "resp": "SlamClient not available in this SDK build"})
        return

    ChannelFactoryInitialize(0, network_iface)
    client = SlamClient()
    client.SetTimeout(10.0)
    client.Init()
    time.sleep(0.5)
    print("[SpatialRpcWorker] ready (Go2)", flush=True)

    while True:
        try:
            cmd = cmd_queue.get()
        except Exception:
            break
        if cmd is None:
            break

        method = cmd.get("method")
        args = cmd.get("args", {})
        try:
            if method == "StartMapping":
                code, resp = client.StartMapping()
            elif method == "StopMapping":
                code, resp = client.StopMapping(args["address"])
            elif method == "InitPose":
                code, resp = client.InitPose(**args)
            elif method == "NavigateTo":
                code, resp = client.NavigateTo(**args)
            elif method == "PauseNav":
                code, resp = client.PauseNav()
            elif method == "ResumeNav":
                code, resp = client.ResumeNav()
            else:
                result_queue.put({"code": -1, "resp": f"unknown method: {method}"})
                continue
            result_queue.put({"code": code, "resp": resp})
        except Exception as e:
            result_queue.put({"code": -1, "resp": str(e)})


class _SlamRpcProxy:
    """Proxy that forwards SLAM RPC calls to a subprocess."""

    def __init__(self, network_iface: str = "eth0"):
        ctx = multiprocessing.get_context("spawn")
        self._cmd_q = ctx.Queue()
        self._result_q = ctx.Queue()
        self._proc = ctx.Process(
            target=_slam_rpc_worker,
            args=(self._cmd_q, self._result_q, network_iface),
            daemon=True,
        )
        self._proc.start()
        self._lock = threading.Lock()

    def _call(self, method: str, args: dict | None = None, timeout: float = 15.0) -> tuple:
        with self._lock:
            self._cmd_q.put({"method": method, "args": args or {}})
            try:
                result = self._result_q.get(timeout=timeout)
            except Exception:
                return 3104, None
            return result["code"], result["resp"]

    def StartMapping(self) -> tuple:
        return self._call("StartMapping")

    def StopMapping(self, address: str) -> tuple:
        return self._call("StopMapping", {"address": address})

    def InitPose(self, x=0.0, y=0.0, z=0.0, q_x=0.0, q_y=0.0, q_z=0.0, q_w=1.0, address="") -> tuple:
        return self._call("InitPose", {"x": x, "y": y, "z": z, "q_x": q_x, "q_y": q_y, "q_z": q_z, "q_w": q_w, "address": address})

    def NavigateTo(self, x, y, z=0.0, q_x=0.0, q_y=0.0, q_z=0.0, q_w=1.0, speed=0.5, mode=0) -> tuple:
        return self._call("NavigateTo", {"x": x, "y": y, "z": z, "q_x": q_x, "q_y": q_y, "q_z": q_z, "q_w": q_w, "speed": speed, "mode": mode})

    def PauseNav(self) -> tuple:
        return self._call("PauseNav")

    def ResumeNav(self) -> tuple:
        return self._call("ResumeNav")

    def stop(self):
        try:
            self._cmd_q.put(None)
            self._proc.join(timeout=3)
        except Exception:
            pass


# ── Database ─────────────────────────────────────────────────────────────────

class _SpatialDB:
    """SQLite storage for maps, POIs, and trajectory."""

    def __init__(self, db_path: str):
        os.makedirs(os.path.dirname(db_path) if os.path.dirname(db_path) else '.', exist_ok=True)
        self._conn = sqlite3.connect(db_path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._migrate()

    def _migrate(self):
        c = self._conn.cursor()
        c.executescript("""
            CREATE TABLE IF NOT EXISTS maps (
                name TEXT PRIMARY KEY,
                pcd_path TEXT NOT NULL,
                created_at REAL DEFAULT (strftime('%s','now')),
                last_used_at REAL DEFAULT 0
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
            CREATE TABLE IF NOT EXISTS trajectory (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                x REAL NOT NULL, y REAL NOT NULL, yaw REAL DEFAULT 0,
                ts REAL NOT NULL
            );
            CREATE TABLE IF NOT EXISTS meta (
                key TEXT PRIMARY KEY,
                value TEXT
            );
        """)
        self._conn.commit()

    def get_last_used_map(self) -> str | None:
        row = self._conn.execute(
            "SELECT value FROM meta WHERE key='last_used_map'"
        ).fetchone()
        return row['value'] if row else None

    def set_last_used_map(self, name: str):
        self._conn.execute(
            "INSERT OR REPLACE INTO meta (key, value) VALUES ('last_used_map', ?)", (name,)
        )
        self._conn.execute(
            "UPDATE maps SET last_used_at = strftime('%s','now') WHERE name = ?", (name,)
        )
        self._conn.commit()

    def add_map(self, name: str, pcd_path: str):
        self._conn.execute(
            "INSERT OR REPLACE INTO maps (name, pcd_path) VALUES (?, ?)", (name, pcd_path)
        )
        self._conn.commit()

    def list_maps(self) -> list[dict]:
        rows = self._conn.execute("SELECT name, pcd_path, created_at, last_used_at FROM maps ORDER BY last_used_at DESC").fetchall()
        return [dict(r) for r in rows]

    def get_map(self, name: str) -> dict | None:
        row = self._conn.execute("SELECT * FROM maps WHERE name = ?", (name,)).fetchone()
        return dict(row) if row else None

    def delete_map(self, name: str) -> bool:
        map_info = self.get_map(name)
        if not map_info:
            return False
        pcd_path = map_info["pcd_path"]
        try:
            os.remove(pcd_path)
        except OSError:
            pass
        self._conn.execute("DELETE FROM poi WHERE map_name = ?", (name,))
        self._conn.execute("DELETE FROM maps WHERE name = ?", (name,))
        self._conn.commit()
        return True

    def add_poi(self, name: str, x: float, y: float, yaw: float, map_name: str, description: str = ""):
        self._conn.execute(
            "INSERT OR REPLACE INTO poi (name, description, x, y, yaw, map_name) VALUES (?, ?, ?, ?, ?, ?)",
            (name, description, x, y, yaw, map_name)
        )
        self._conn.commit()

    def delete_poi(self, name: str, map_name: str) -> bool:
        cur = self._conn.execute("DELETE FROM poi WHERE name = ? AND map_name = ?", (name, map_name))
        self._conn.commit()
        return cur.rowcount > 0

    def list_pois(self, map_name: str) -> list[dict]:
        rows = self._conn.execute(
            "SELECT name, description, x, y, yaw FROM poi WHERE map_name = ? ORDER BY name",
            (map_name,)
        ).fetchall()
        return [dict(r) for r in rows]

    def find_poi(self, query: str, map_name: str) -> dict | None:
        row = self._conn.execute(
            "SELECT name, description, x, y, yaw FROM poi WHERE map_name = ? AND name LIKE ?",
            (map_name, f"%{query}%")
        ).fetchone()
        return dict(row) if row else None

    def add_trajectory(self, x: float, y: float, yaw: float, ts: float):
        self._conn.execute(
            "INSERT INTO trajectory (x, y, yaw, ts) VALUES (?, ?, ?, ?)", (x, y, yaw, ts)
        )
        self._conn.commit()

    def prune_trajectory(self, keep_seconds: float = 3600):
        self._conn.execute(
            "DELETE FROM trajectory WHERE ts < ?", (time.time() - keep_seconds,)
        )
        self._conn.commit()


# ── Helpers ──────────────────────────────────────────────────────────────────

SPATIAL_POS_INTERVAL = 0.1      # 10 Hz pos_tag publish
SPATIAL_TRAJ_INTERVAL = 3.0     # trajectory sample every 3s
SPATIAL_TRAJ_MIN_DIST = 0.3     # or if moved > 0.3m


def _bearing_label(dx: float, dy: float) -> str:
    """Convert delta (x=forward, y=left) to bearing label."""
    angle = math.atan2(dy, dx)
    deg = math.degrees(angle)
    if -22.5 <= deg < 22.5:
        return "front"
    elif 22.5 <= deg < 67.5:
        return "left_front"
    elif 67.5 <= deg < 112.5:
        return "left"
    elif 112.5 <= deg < 157.5:
        return "left_behind"
    elif -67.5 <= deg < -22.5:
        return "right_front"
    elif -112.5 <= deg < -67.5:
        return "right"
    elif -157.5 <= deg < -112.5:
        return "right_behind"
    else:
        return "behind"


# ── Spatial Node (ROS2) ──────────────────────────────────────────────────────

class _SpatialNode(Node):
    """Subscribes to SLAM DDS topics, maintains voxel map, publishes sensor streams."""

    VOXEL_SIZE = 0.05            # 5cm voxel grid for deduplication
    MAP_PUBLISH_INTERVAL = 1.0   # 1 Hz full map publish
    MAP_SAVE_INTERVAL = 5.0      # auto-save PCD every 5s
    MAX_SEND_POINTS = 50000      # max points per publish
    RECENT_CLOUD_MAX = 50000     # recent cloud ring buffer capacity
    KF_DIST_THRESH = 2.0         # keyframe every 2m movement
    KF_YAW_THRESH = 0.52         # or 30° rotation
    KF_TIME_THRESH = 30.0        # or every 30s even if stationary
    SLAM_CLOUD_INTERVAL = 0.2    # 5Hz

    def __init__(self, pos_tag_topic: str, mapping_topic: str, slam_cloud_topic: str,
                 grid_map_topic: str, db: _SpatialDB, sc_mgr=None):
        super().__init__("go2_spatial")
        self._db = db
        self._sc_mgr = sc_mgr
        self._auto_mapping_cb = None  # set by SpatialPlugin

        # Publishers
        self._pos_tag_pub = self.create_publisher(String, pos_tag_topic, _LOW_LAT_QOS)
        self._mapping_pub = self.create_publisher(UInt8MultiArray, mapping_topic, _LOW_LAT_QOS)
        self._grid_map_pub = self.create_publisher(UInt8MultiArray, grid_map_topic, _LOW_LAT_QOS)
        self._slam_cloud_pub = self.create_publisher(UInt8MultiArray, slam_cloud_topic, _LOW_LAT_QOS)

        # Overlay data (merged into mapping publish)
        self._nav_path_overlay: np.ndarray | None = None   # Nx3 path points (z=0.3)
        self._grid_overlay: np.ndarray | None = None        # Nx3 obstacle points (z=0.5)

        # Grid map 2D cache (refreshed periodically from planner)
        self._planner_ref = None  # set by SpatialPlugin
        self._grid_pts_cache: np.ndarray | None = None
        self._planner_grid_pts_cache_valid = False

        # Coordinate bias (SLAM frame → map frame transform)
        self._bias_x: float = 0.0
        self._bias_y: float = 0.0
        self._bias_yaw: float = 0.0
        self._bias_set: bool = False

        # State
        self._current_pose: dict | None = None
        self._map_status: str = "idle"    # idle | mapping | localized
        self._nav_status: dict | None = None
        self._nav_target_name: str | None = None
        self._active_map: str | None = None
        self._lock = threading.Lock()

        self._last_pub_time: float = 0.0
        self._last_traj_time: float = 0.0
        self._last_traj_pose: tuple = (0.0, 0.0)
        self._last_slam_cloud_time: float = 0.0

        # 3D voxel map buffer
        self._map_buffer: dict[tuple, tuple] = {}
        self._map_buffer_lock = threading.Lock()
        self._map_buffer_dirty = False

        # Cloud processing queue + background thread
        self._cloud_queue = queue.Queue(maxsize=50)
        self._cloud_processor_running = True
        self._cloud_processor_thread = threading.Thread(
            target=self._cloud_processor_loop, daemon=True, name="spatial_cloud_processor"
        )
        self._cloud_processor_thread.start()

        # Recent cloud ring buffer for fingerprinting
        self._recent_cloud = np.zeros((self.RECENT_CLOUD_MAX, 3), dtype=np.float32)
        self._recent_cloud_count = 0
        self._recent_cloud_write_idx = 0

        # Keyframe tracking
        self._last_kf_pose: tuple = (0.0, 0.0, 0.0)

        # Map publish timing
        self._last_map_publish_time: float = 0.0

        # PCD auto-save
        self._pcd_save_dir: str | None = None
        self._save_timer: threading.Timer | None = None
        self._save_timer_running = False

        # Navigation state
        self._nav_arrived = threading.Event()
        self._nav_error: str | None = None

        # Subscribe DDS topics
        self._dds_subs = []
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_

            info_sub = ChannelSubscriber("rt/slam_info", String_)
            info_sub.Init(self._on_slam_info, 10)
            self._dds_subs.append(info_sub)
            self.get_logger().info("SpatialNode subscribed rt/slam_info")

            key_sub = ChannelSubscriber("rt/slam_key_info", String_)
            key_sub.Init(self._on_slam_key_info, 10)
            self._dds_subs.append(key_sub)
            self.get_logger().info("SpatialNode subscribed rt/slam_key_info")
        except Exception as e:
            self.get_logger().warn(f"SpatialNode: failed to subscribe slam_info: {e}")

        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_

            map_cloud_sub = ChannelSubscriber("rt/unitree/slam_mapping/points", PointCloud2_)
            map_cloud_sub.Init(self._on_mapping_cloud, 10)
            self._dds_subs.append(map_cloud_sub)
            self.get_logger().info("SpatialNode subscribed rt/unitree/slam_mapping/points")

            reloc_cloud_sub = ChannelSubscriber("rt/unitree/slam_relocation/points", PointCloud2_)
            reloc_cloud_sub.Init(self._on_mapping_cloud, 10)
            self._dds_subs.append(reloc_cloud_sub)
            self.get_logger().info("SpatialNode subscribed rt/unitree/slam_relocation/points")
        except Exception as e:
            self.get_logger().warn(f"SpatialNode: failed to subscribe mapping points: {e}")

    # ── DDS Callbacks ────────────────────────────────────────────────────────

    def _on_slam_info(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        msg_type = data.get("type", "")

        if msg_type in ("pos_info", "mapping_info"):
            pose_data = data.get("data", {}).get("currentPose")
            if pose_data:
                yaw = math.atan2(
                    2 * (pose_data.get("q_w", 1) * pose_data.get("q_z", 0)),
                    1 - 2 * pose_data.get("q_z", 0) ** 2
                )
                with self._lock:
                    prev_status = self._map_status
                    self._current_pose = {
                        "x": pose_data["x"],
                        "y": pose_data["y"],
                        "yaw": round(yaw, 3),
                    }
                    if msg_type == "pos_info":
                        self._map_status = "localized"
                    elif msg_type == "mapping_info":
                        self._map_status = "mapping"

                # Auto-transition: idle → localized → trigger auto-mapping
                if msg_type == "pos_info" and prev_status != "mapping" and prev_status != "localized" and self._auto_mapping_cb:
                    self.get_logger().info("[slam_info] Localized! Triggering auto StartMapping...")
                    try:
                        self._auto_mapping_cb()
                    except Exception as e:
                        self.get_logger().warn(f"[slam_info] auto-mapping callback failed: {e}")

            self._maybe_record_trajectory()
            self._maybe_publish_pos_tag()

        elif msg_type == "ctrl_info":
            ctrl = data.get("data", {})
            progress = ctrl.get("progress", {})
            with self._lock:
                self._nav_status = {
                    "target": self._nav_target_name,
                    "progress": progress.get("completion_percentage", 0),
                    "eta_seconds": progress.get("last_time", 0),
                    "is_arrived": ctrl.get("is_arrived", False),
                    "obstacle": ctrl.get("obsInfo", {}).get("state", False),
                    "is_paused": ctrl.get("stateMachine", {}).get("isPause", False),
                }
                if ctrl.get("is_arrived"):
                    self._nav_status = None
                    self._nav_target_name = None
                    self._nav_arrived.set()
            obs = ctrl.get("obsInfo", {})
            if obs.get("state") and obs.get("time", 0) > 10:
                self._nav_error = "blocked by obstacle for >10s"
                self._nav_arrived.set()

    def _on_slam_key_info(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if data.get("type") == "task_result":
            is_arrived = data.get("data", {}).get("is_arrived", False)
            if is_arrived:
                with self._lock:
                    self._nav_status = None
                    self._nav_target_name = None
                self._nav_arrived.set()

    def _on_mapping_cloud(self, msg) -> None:
        """DDS callback: fast enqueue only."""
        try:
            data = bytes(msg.data)
            if len(data) < msg.point_step:
                return

            # Real-time slam_cloud passthrough (throttled)
            now = time.monotonic()
            if now - self._last_slam_cloud_time >= self.SLAM_CLOUD_INTERVAL:
                self._last_slam_cloud_time = now
                self._publish_slam_cloud(msg.fields, msg.point_step, msg.width * msg.height, data)

            self._cloud_queue.put_nowait((msg.fields, msg.point_step, msg.width * msg.height, data))
        except Exception:
            pass

    # ── Cloud Processing ─────────────────────────────────────────────────────

    def _publish_slam_cloud(self, fields, point_step: int, total_points: int, data: bytes) -> None:
        """Parse SLAM PointCloud2, publish as sensor/pointcloud binary."""
        num_points = min(total_points, 20000)
        if len(data) < num_points * point_step:
            num_points = len(data) // point_step
        if num_points == 0:
            return

        field_map = {}
        for f in fields:
            field_map[f.name] = f.offset
        x_off = field_map.get("x", 0)
        y_off = field_map.get("y", 4)
        z_off = field_map.get("z", 8)

        raw = np.frombuffer(data, dtype=np.uint8, count=num_points * point_step)
        raw = raw.reshape(num_points, point_step)
        sx = raw[:, x_off:x_off+4].view(np.float32).ravel()
        sy = raw[:, y_off:y_off+4].view(np.float32).ravel()
        sz = raw[:, z_off:z_off+4].view(np.float32).ravel()

        valid = (
            np.isfinite(sx) & np.isfinite(sy) & np.isfinite(sz) &
            (np.abs(sx) < 50) & (np.abs(sy) < 50) & (np.abs(sz) < 20)
        )
        sx, sy, sz = sx[valid], sy[valid], sz[valid]
        n = len(sx)
        if n == 0:
            return

        out = np.column_stack([sx, sy, sz]).astype(np.float32)
        header = struct.pack('<II', 12, n)
        ros_msg = UInt8MultiArray()
        ros_msg.data = list(header + out.tobytes())
        self._slam_cloud_pub.publish(ros_msg)

    def _cloud_processor_loop(self):
        """Background thread: processes queued point clouds."""
        while self._cloud_processor_running:
            try:
                item = self._cloud_queue.get(timeout=1.0)
            except Exception:
                continue

            fields, point_step, total_points, data = item
            if total_points == 0:
                continue

            field_map = {}
            for f in fields:
                field_map[f.name] = (f.offset, f.datatype)
            x_off = field_map.get("x", (0, 7))[0]
            y_off = field_map.get("y", (4, 7))[0]
            z_off = field_map.get("z", (8, 7))[0]

            num_points = min(total_points, 20000)
            if len(data) < num_points * point_step:
                num_points = len(data) // point_step

            raw = np.frombuffer(data, dtype=np.uint8, count=num_points * point_step)
            raw = raw.reshape(num_points, point_step)

            x = raw[:, x_off:x_off+4].view(np.float32).ravel()
            y = raw[:, y_off:y_off+4].view(np.float32).ravel()
            z = raw[:, z_off:z_off+4].view(np.float32).ravel()

            valid = (
                np.isfinite(x) & np.isfinite(y) & np.isfinite(z) &
                (np.abs(x) < 50) & (np.abs(y) < 50) & (np.abs(z) < 20)
            )
            x, y, z = x[valid], y[valid], z[valid]

            if len(x) == 0:
                continue

            pts_arr = np.column_stack([x, y, z]).astype(np.float32)

            # Merge into voxel map buffer
            voxel_size = self.VOXEL_SIZE
            ix = (pts_arr[:, 0] / voxel_size).astype(np.int32)
            iy = (pts_arr[:, 1] / voxel_size).astype(np.int32)
            iz = (pts_arr[:, 2] / voxel_size).astype(np.int32)

            with self._map_buffer_lock:
                prev_size = len(self._map_buffer)
                for j in range(len(pts_arr)):
                    key = (int(ix[j]), int(iy[j]), int(iz[j]))
                    if key not in self._map_buffer:
                        self._map_buffer[key] = (float(pts_arr[j, 0]), float(pts_arr[j, 1]), float(pts_arr[j, 2]))
                new_size = len(self._map_buffer)
                if new_size > prev_size:
                    self._map_buffer_dirty = True
                    self._planner_grid_pts_cache_valid = False  # refresh grid_map

            # Update recent cloud ring buffer
            n = len(pts_arr)
            start = self._recent_cloud_write_idx
            cap = self.RECENT_CLOUD_MAX
            if n <= cap:
                end = start + n
                if end <= cap:
                    self._recent_cloud[start:end] = pts_arr
                else:
                    first = cap - start
                    self._recent_cloud[start:cap] = pts_arr[:first]
                    self._recent_cloud[0:n - first] = pts_arr[first:]
                self._recent_cloud_write_idx = (start + n) % cap
                self._recent_cloud_count = min(self._recent_cloud_count + n, cap)
            else:
                self._recent_cloud[:] = pts_arr[-cap:]
                self._recent_cloud_write_idx = 0
                self._recent_cloud_count = cap

            self._maybe_add_keyframe(pts_arr)
            self._maybe_publish_full_map()

    # ── Trajectory & Publishing ──────────────────────────────────────────────

    def _maybe_record_trajectory(self):
        with self._lock:
            if self._current_pose is None:
                return
            x, y = self._current_pose["x"], self._current_pose["y"]
            yaw = self._current_pose["yaw"]

        now = time.time()
        dx = x - self._last_traj_pose[0]
        dy = y - self._last_traj_pose[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if (now - self._last_traj_time >= SPATIAL_TRAJ_INTERVAL) or (dist >= SPATIAL_TRAJ_MIN_DIST):
            self._last_traj_time = now
            self._last_traj_pose = (x, y)
            self._db.add_trajectory(x, y, yaw, now)

    def _maybe_publish_pos_tag(self):
        now = time.monotonic()
        if now - self._last_pub_time < SPATIAL_POS_INTERVAL:
            return
        self._last_pub_time = now

        with self._lock:
            pose = self._current_pose
            map_status = self._map_status
            nav_status = dict(self._nav_status) if self._nav_status else None
            active_map = self._active_map

        if pose is None:
            return

        tags_in_range = []
        if active_map:
            pois = self._db.list_pois(active_map)
            for poi in pois:
                dx = poi["x"] - pose["x"]
                dy = poi["y"] - pose["y"]
                dist = math.sqrt(dx * dx + dy * dy)
                if dist <= 20.0:
                    cos_yaw = math.cos(-pose["yaw"])
                    sin_yaw = math.sin(-pose["yaw"])
                    rx = dx * cos_yaw - dy * sin_yaw
                    ry = dx * sin_yaw + dy * cos_yaw
                    tags_in_range.append({
                        "name": poi["name"],
                        "dist": round(dist, 2),
                        "bearing": _bearing_label(rx, ry),
                    })
            tags_in_range.sort(key=lambda t: t["dist"])

        output = {
            "pose": pose,
            "nearest_tag": tags_in_range[0] if tags_in_range else None,
            "tags_in_range": tags_in_range[:5],
            "map_status": map_status,
            "nav_status": nav_status,
        }

        out = String()
        out.data = json.dumps(output)
        self._pos_tag_pub.publish(out)

    def _maybe_add_keyframe(self, pts_arr) -> None:
        """Generate Scan Context keyframe if robot moved/rotated enough."""
        if self._sc_mgr is None:
            return
        with self._lock:
            if self._current_pose is None or self._active_map is None:
                return
            x, y = self._current_pose["x"], self._current_pose["y"]
            yaw = self._current_pose["yaw"]
            active_map = self._active_map

        lx, ly, lyaw = self._last_kf_pose
        dx = x - lx
        dy = y - ly
        dist = math.sqrt(dx * dx + dy * dy)
        dyaw = abs(yaw - lyaw)
        if dyaw > math.pi:
            dyaw = 2 * math.pi - dyaw

        if dist >= self.KF_DIST_THRESH or dyaw >= self.KF_YAW_THRESH or \
                (time.time() - getattr(self, '_last_kf_time', 0)) >= self.KF_TIME_THRESH:
            sc = self._sc_mgr.make_scan_context(pts_arr)
            self._sc_mgr.add_keyframe(active_map, sc, (x, y, 0.0))
            self._last_kf_pose = (x, y, yaw)
            self._last_kf_time = time.time()

    def _maybe_publish_full_map(self) -> None:
        """Publish the full 3D voxel map at 1Hz."""
        now = time.monotonic()
        if now - self._last_map_publish_time < self.MAP_PUBLISH_INTERVAL:
            return
        self._last_map_publish_time = now

        with self._lock:
            pose = self._current_pose
        robot_x = pose["x"] if pose else 0.0
        robot_y = pose["y"] if pose else 0.0
        robot_yaw = pose["yaw"] if pose else 0.0

        with self._map_buffer_lock:
            if not self._map_buffer:
                return
            all_points = list(self._map_buffer.values())

        pts = np.array(all_points, dtype=np.float32)
        num_points = len(pts)

        if num_points > self.MAX_SEND_POINTS:
            indices = np.random.choice(num_points, self.MAX_SEND_POINTS, replace=False)
            pts = pts[indices]
            num_points = self.MAX_SEND_POINTS

        # Merge nav path overlay into the 3D mapping point cloud
        overlay_parts = [pts]
        if self._nav_path_overlay is not None and len(self._nav_path_overlay) > 0:
            overlay_parts.append(self._nav_path_overlay)
        if len(overlay_parts) > 1:
            pts = np.vstack(overlay_parts)
            num_points = len(pts)

        # Binary format: [float32 robot_x, robot_y, robot_yaw][uint8 flags][uint32 N][float32 x,y,z × N]
        flags = 0x03  # bit0=full_map, bit1=has_z
        header = struct.pack('<fffBI', robot_x, robot_y, robot_yaw, flags, num_points)
        body = pts.tobytes()

        ros_msg = UInt8MultiArray()
        ros_msg.data = list(header + body)
        self._mapping_pub.publish(ros_msg)

        # Publish 2D grid map (occupancy grid + nav path, all at z=0)
        self._publish_grid_map_2d(robot_x, robot_y, robot_yaw)

    def _publish_grid_map_2d(self, robot_x: float, robot_y: float, robot_yaw: float):
        """发布 2D 占据栅格鸟瞰图（规划器的障碍物栅格 + 导航路线）。"""
        parts = []

        # 尝试加载/刷新规划器栅格（每次都用最新 voxel buffer）
        if not self._planner_grid_pts_cache_valid:
            with self._map_buffer_lock:
                if self._map_buffer and len(self._map_buffer) > 100:
                    pts_buf = np.array(list(self._map_buffer.values()), dtype=np.float32)
                    # 使用 SpatialPlugin 设置的 planner 引用来刷新
                    if self._planner_ref is not None:
                        self._planner_ref.load_from_buffer(pts_buf)
                        grid_pts = self._planner_ref.get_grid_as_points()
                        if grid_pts is not None:
                            # 压平到 z=0 用于 2D 显示
                            grid_pts_2d = grid_pts.copy()
                            grid_pts_2d[:, 2] = 0.0
                            self._grid_pts_cache = grid_pts_2d
                            self._planner_grid_pts_cache_valid = True

        if self._grid_pts_cache is not None:
            parts.append(self._grid_pts_cache)

        # 导航路线点 (z=0.3, 高于障碍物以便在2D视图中区分)
        if self._nav_path_overlay is not None and len(self._nav_path_overlay) > 0:
            path_2d = self._nav_path_overlay.copy()
            path_2d[:, 2] = 0.3
            parts.append(path_2d)

        if not parts:
            return

        pts = np.vstack(parts)
        num_points = len(pts)
        if num_points > 50000:
            indices = np.random.choice(num_points, 50000, replace=False)
            pts = pts[indices]
            num_points = 50000

        flags = 0x03  # full_map + has_z
        header = struct.pack('<fffBI', robot_x, robot_y, robot_yaw, flags, num_points)

        ros_msg = UInt8MultiArray()
        ros_msg.data = list(header + pts.tobytes())
        self._grid_map_pub.publish(ros_msg)

    # ── PCD Save ─────────────────────────────────────────────────────────────

    def _maybe_save_pcd(self) -> None:
        if not self._pcd_save_dir:
            self._schedule_save_timer()
            return

        with self._lock:
            active_map = self._active_map
        if not active_map:
            self._schedule_save_timer()
            return

        with self._map_buffer_lock:
            if not self._map_buffer or not self._map_buffer_dirty:
                self._schedule_save_timer()
                return
            all_points = list(self._map_buffer.values())
            self._map_buffer_dirty = False

        if len(all_points) < 10:
            self._schedule_save_timer()
            return

        pcd_path = os.path.join(self._pcd_save_dir, f"{active_map}.pcd")
        os.makedirs(os.path.dirname(pcd_path), exist_ok=True)
        try:
            pts = np.array(all_points, dtype=np.float32)
            num = len(pts)
            with open(pcd_path, 'wb') as f:
                header = (
                    f"# .PCD v0.7 - Point Cloud Data\n"
                    f"VERSION 0.7\n"
                    f"FIELDS x y z\n"
                    f"SIZE 4 4 4\n"
                    f"TYPE F F F\n"
                    f"COUNT 1 1 1\n"
                    f"WIDTH {num}\n"
                    f"HEIGHT 1\n"
                    f"VIEWPOINT 0 0 0 1 0 0 0\n"
                    f"POINTS {num}\n"
                    f"DATA binary\n"
                )
                f.write(header.encode('ascii'))
                f.write(pts.tobytes())
            self.get_logger().info(f"Auto-saved PCD: {pcd_path} ({num} points, binary)")
        except Exception as e:
            self.get_logger().warn(f"Failed to save PCD: {e}")

        self._schedule_save_timer()

    def _schedule_save_timer(self):
        if not self._save_timer_running:
            return
        self._save_timer = threading.Timer(self.MAP_SAVE_INTERVAL, self._maybe_save_pcd)
        self._save_timer.daemon = True
        self._save_timer.start()

    def _start_save_timer(self):
        self._save_timer_running = True
        self._schedule_save_timer()

    def _stop_save_timer(self):
        self._save_timer_running = False
        if self._save_timer:
            self._save_timer.cancel()
            self._save_timer = None

    # ── Public API ───────────────────────────────────────────────────────────

    def set_pcd_save_dir(self, path: str):
        self._pcd_save_dir = path
        self._start_save_timer()

    def load_pcd_to_buffer(self, pcd_path: str) -> None:
        """Load a PCD file into the voxel map buffer."""
        if not os.path.exists(pcd_path):
            self.get_logger().warn(f"PCD file not found: {pcd_path}")
            return
        points = self._parse_pcd(pcd_path)
        if points is None or len(points) == 0:
            return
        voxel_size = self.VOXEL_SIZE
        with self._map_buffer_lock:
            for i in range(len(points)):
                ix = int(points[i, 0] / voxel_size)
                iy = int(points[i, 1] / voxel_size)
                iz = int(points[i, 2] / voxel_size)
                self._map_buffer[(ix, iy, iz)] = (points[i, 0], points[i, 1], points[i, 2])
        self.get_logger().info(f"Loaded {len(points)} points from PCD, buffer size: {len(self._map_buffer)}")

    def clear_map_buffer(self) -> None:
        with self._map_buffer_lock:
            self._map_buffer.clear()

    def get_recent_cloud(self):
        count = min(self._recent_cloud_count, self.RECENT_CLOUD_MAX)
        if count == 0:
            return None
        return self._recent_cloud[:count].copy()

    def get_pose(self) -> dict | None:
        """返回当前位姿（map 坐标系，已应用 bias）。"""
        with self._lock:
            if not self._current_pose:
                return None
            raw = self._current_pose
        if not self._bias_set:
            return dict(raw)
        # Apply bias: map = R_bias * slam + T_bias
        cos_b = math.cos(self._bias_yaw)
        sin_b = math.sin(self._bias_yaw)
        x = cos_b * raw["x"] - sin_b * raw["y"] + self._bias_x
        y = sin_b * raw["x"] + cos_b * raw["y"] + self._bias_y
        yaw = raw["yaw"] + self._bias_yaw
        # Normalize yaw
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        return {"x": x, "y": y, "yaw": round(yaw, 3)}

    def get_pose_raw(self) -> dict | None:
        """返回 SLAM 原始位姿（不加 bias）。"""
        with self._lock:
            return dict(self._current_pose) if self._current_pose else None

    def map_to_slam(self, x: float, y: float) -> tuple:
        """旧图坐标 → SLAM 坐标（逆 bias 变换）。"""
        if not self._bias_set:
            return (x, y)
        dx = x - self._bias_x
        dy = y - self._bias_y
        cos_b = math.cos(-self._bias_yaw)
        sin_b = math.sin(-self._bias_yaw)
        return (cos_b * dx - sin_b * dy, sin_b * dx + cos_b * dy)

    def set_active_map(self, name: str | None):
        with self._lock:
            self._active_map = name

    def set_map_status(self, status: str):
        with self._lock:
            self._map_status = status

    def set_nav_target(self, name: str | None):
        with self._lock:
            self._nav_target_name = name

    @staticmethod
    def _parse_pcd(path: str):
        """Parse ASCII/binary PCD file, extract x,y,z. Returns Nx3 numpy array."""
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
                num_points = 0
                data_type = "ascii"
                field_sizes = []
                for hl in header_lines:
                    parts = hl.split()
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
                    xi = fields.index("x")
                    yi = fields.index("y")
                    zi = fields.index("z")
                except ValueError:
                    return None

                if data_type == "ascii":
                    points = []
                    for _ in range(num_points):
                        line = f.readline().decode('ascii', errors='ignore').strip()
                        if not line:
                            break
                        vals = line.split()
                        if len(vals) <= max(xi, yi, zi):
                            continue
                        px = float(vals[xi])
                        py = float(vals[yi])
                        pz = float(vals[zi])
                        if px != px or py != py or pz != pz:
                            continue
                        points.append((px, py, pz))
                    return np.array(points, dtype=np.float32) if points else None

                elif data_type == "binary":
                    point_size = sum(field_sizes)
                    raw = f.read(num_points * point_size)
                    if len(raw) < num_points * point_size:
                        num_points = len(raw) // point_size

                    offsets = [0]
                    for s in field_sizes[:-1]:
                        offsets.append(offsets[-1] + s)

                    x_off = offsets[xi]
                    y_off = offsets[yi]
                    z_off = offsets[zi]

                    points = np.zeros((num_points, 3), dtype=np.float32)
                    for i in range(num_points):
                        base = i * point_size
                        points[i, 0] = struct.unpack_from('<f', raw, base + x_off)[0]
                        points[i, 1] = struct.unpack_from('<f', raw, base + y_off)[0]
                        points[i, 2] = struct.unpack_from('<f', raw, base + z_off)[0]

                    valid = ~np.isnan(points).any(axis=1)
                    return points[valid]

        except Exception:
            return None


# ── SpatialPlugin ────────────────────────────────────────────────────────────

class SpatialPlugin:
    """Go2 autonomous spatial intelligence — auto mapping, Scan Context, navigation."""

    PREFIX = "spatial"

    @staticmethod
    def _ensure_slam_service():
        """Ensure unitree_slam and mid360_driver are running via nsenter."""
        import subprocess as sp

        slam_bin = "/unitree/module/unitree_slam/bin/unitree_slam"
        lidar_bin = "/unitree/module/unitree_slam/bin/mid360_driver"
        work_dir = "/unitree/module/unitree_slam/bin"
        log_dir = "/opt/phanthy-motus/data"

        if not os.path.exists(slam_bin):
            print("[Spatial] unitree_slam binary not found (volume not mounted?), skipping auto-start")
            return

        os.makedirs(log_dir, exist_ok=True)

        if sp.run(["pgrep", "-f", "unitree_slam"], capture_output=True).returncode != 0:
            print("[Spatial] Starting unitree_slam via nsenter...", flush=True)
            sp.Popen(
                ["nsenter", "-t", "1", "-m", "--", "bash", "-c",
                 f"cd {work_dir} && export CYCLONEDDS_URI=file:///home/unitree/cyclonedds_ws/cyclonedds.xml && "
                 f"nohup ./unitree_slam > {log_dir}/unitree_slam.log 2>&1 &"],
            )

        if sp.run(["pgrep", "-f", "mid360_driver"], capture_output=True).returncode != 0:
            print("[Spatial] Starting mid360_driver via nsenter...", flush=True)
            sp.Popen(
                ["nsenter", "-t", "1", "-m", "--", "bash", "-c",
                 f"cd {work_dir} && export CYCLONEDDS_URI=file:///home/unitree/cyclonedds_ws/cyclonedds.xml && "
                 f"nohup ./mid360_driver > {log_dir}/mid360_driver.log 2>&1 &"],
            )

        time.sleep(5)

    def __init__(self, plugin_config: dict, namespace: str, executor, rpc_proxy=None):
        network_iface = plugin_config.get("network_iface", "eth0")
        # Note: _ensure_slam_service() is NOT called here.
        # ControlledSpatialPlugin (loaded first) already starts unitree_slam + mid360_driver.
        # We just wait briefly for it to be ready.
        time.sleep(2)
        self._client = _SlamRpcProxy(network_iface)
        self._rpc_proxy = rpc_proxy  # for obstacles_avoid calls

        self._map_dir = plugin_config.get("native_slam_pcd_dir", "/home/unitree/maps")
        import subprocess as _sp
        _sp.run(["nsenter", "-t", "1", "-m", "--", "mkdir", "-p", self._map_dir],
                capture_output=True)

        db_path = plugin_config.get("db_path", "/opt/phanthy-motus/data/spatial.db")
        self._db = _SpatialDB(db_path)

        # Scan Context manager
        sc_db_path = os.path.join(os.path.dirname(db_path), "scan_context.db")
        from scan_context import ScanContextManager
        self._sc_mgr = ScanContextManager(sc_db_path)

        # Path planner (loaded lazily when first map is available)
        from path_planner import PathPlanner
        self._planner = PathPlanner()
        self._nav_executing = False
        self._nav_waypoints: list[tuple] = []
        self._nav_target_yaw: float = 0.0

        # Topics
        self._pos_tag_topic = f"/{namespace}/spatial/pos_tag" if namespace else "/spatial/pos_tag"
        self._mapping_topic = f"/{namespace}/spatial/mapping" if namespace else "/spatial/mapping"
        self._slam_cloud_topic = f"/{namespace}/spatial/slam_cloud" if namespace else "/spatial/slam_cloud"
        self._grid_map_topic = f"/{namespace}/spatial/grid_map" if namespace else "/spatial/grid_map"

        # Create node
        self._node = _SpatialNode(
            self._pos_tag_topic, self._mapping_topic, self._slam_cloud_topic,
            self._grid_map_topic, self._db, self._sc_mgr
        )
        self._node.set_active_map(self._db.get_last_used_map())
        self._node.set_pcd_save_dir(self._map_dir)
        self._node._auto_mapping_cb = self._on_localized
        self._node._planner_ref = self._planner  # for grid_map 2D publishing
        executor.add_node(self._node)

    def get_tools(self) -> list:
        return [self._spatial_tool(), self._pos_tag_tool(), self._mapping_tool(),
                self._slam_cloud_tool(), self._grid_map_tool()]

    def _pos_tag_tool(self) -> dict:
        return {
            "name": "pos_tag",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Spatial position + nearest tags — current pose, nearby POIs with distance/bearing, map/nav status. 10Hz to {self._pos_tag_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._pos_tag_topic, "format": "data/json"}],
        }

    def _mapping_tool(self) -> dict:
        return {
            "name": "slam_mapping",
            "type": "sensor",
            "multiInstance": False,
            "description": f"SLAM 3D mapping visualization — full 3D point cloud map with robot position. Binary: [float32 robot_x,y,yaw][uint8 flags][uint32 N][float32 x,y,z × N]. 1Hz to {self._mapping_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._mapping_topic, "format": "sensor/mapping"}],
        }

    def _slam_cloud_tool(self) -> dict:
        return {
            "name": "slam_cloud",
            "type": "sensor",
            "multiInstance": False,
            "description": f"Real-time SLAM point cloud at 5Hz. Binary: [uint32 point_step=12][uint32 total_points][float32 x,y,z × N]. Publishes to {self._slam_cloud_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._slam_cloud_topic, "format": "sensor/pointcloud"}],
        }

    def _grid_map_tool(self) -> dict:
        return {
            "name": "grid_map",
            "type": "sensor",
            "multiInstance": False,
            "description": f"2D occupancy grid bird's-eye view — obstacle cells + navigation path. Continuous 1Hz. {self._grid_map_topic}",
            "inputSchema": {"type": "object", "properties": {}},
            "topic_out": [{"topic": self._grid_map_topic, "format": "sensor/mapping"}],
        }

    def _spatial_tool(self) -> dict:
        return {
            "name": "spatial",
            "type": "actuator",
            "multiInstance": False,
            "description": "Spatial intelligence — auto mapping, place tagging, navigation. Mapping is always active automatically.",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["tag_place", "untag_place", "list_tags",
                                 "list_maps", "delete_map",
                                 "relocalize_at_tag",
                                 "navigate_to_tag", "navigate_to_pose",
                                 "wait_navigation_done",
                                 "pause_nav", "resume_nav", "stop_nav"],
                        "description": "Action to perform",
                    },
                    "name":        {"type": "string", "description": "POI tag name"},
                    "description": {"type": "string", "description": "POI description"},
                    "tag_name":    {"type": "string", "description": "Target tag name for navigation"},
                    "x":           {"type": "number", "description": "Target X coordinate (meters)"},
                    "y":           {"type": "number", "description": "Target Y coordinate (meters)"},
                    "yaw":         {"type": "number", "description": "Target yaw (radians)"},
                    "map_name":    {"type": "string", "description": "Map name (for delete_map)"},
                    "speed":       {"type": "number", "description": "Navigation speed 0.2-0.8 m/s (default 0.5)"},
                    "mode":        {"type": "integer", "description": "Obstacle mode: 0=detour(default), 1=stop"},
                    "stall_timeout": {"type": "number", "description": "Seconds without movement before declaring timeout (default 60)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "tag_place":        {"params": ["name", "description"], "description": "Tag current position with a name"},
                    "untag_place":      {"params": ["name"],               "description": "Remove a place tag"},
                    "list_tags":        {"params": [],                     "description": "List all tags with relative positions"},
                    "list_maps":        {"params": [],                     "description": "List all saved maps"},
                    "delete_map":       {"params": ["map_name"],           "description": "Delete a map and its associated data"},
                    "relocalize_at_tag": {"params": ["tag_name"],          "description": "Relocalize near a known tag using ICP point cloud matching"},
                    "navigate_to_tag":  {"params": ["tag_name"], "description": "Navigate to a tagged place (A* path planning + obstacle avoidance)"},
                    "navigate_to_pose": {"params": ["x", "y", "yaw"], "description": "Navigate to coordinates (A* path planning + obstacle avoidance)"},
                    "wait_navigation_done": {"params": ["stall_timeout"], "description": "Block until navigation completes or robot is stuck"},
                    "pause_nav":        {"params": [],                     "description": "Pause navigation"},
                    "resume_nav":       {"params": [],                     "description": "Resume navigation"},
                    "stop_nav":         {"params": [],                     "description": "Stop and cancel navigation"},
                },
            },
        }

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Auto-start mapping on plugin start."""
        print("[Spatial] start() called, scheduling auto-mapping in 3s", flush=True)
        def _auto_start():
            time.sleep(3)
            try:
                self._do_auto_mapping()
            except Exception as e:
                print(f"[Spatial] auto-mapping failed: {e}")
                import traceback
                traceback.print_exc()
        threading.Thread(target=_auto_start, daemon=True).start()

    def stop(self) -> None:
        self._node._stop_save_timer()
        self._client.stop()

    # ── Auto Mapping ─────────────────────────────────────────────────────────

    def _on_localized(self):
        """Called when SLAM reports localization success. Start mapping to extend."""
        if self._node._map_status == "mapping":
            return
        # Debounce: don't re-trigger within 5s of last call
        now = time.time()
        if hasattr(self, '_last_localized_time') and now - self._last_localized_time < 5.0:
            return
        self._last_localized_time = now
        print("[Spatial] _on_localized: starting mapping to extend map", flush=True)
        code, resp = self._client.StartMapping()
        print(f"[Spatial] StartMapping after localization → code={code}", flush=True)
        if code == 0:
            self._node.set_map_status("mapping")
            if not self._node._active_map:
                map_name = f"map_{int(time.time())}"
                pcd_path = f"{self._map_dir}/{map_name}.pcd"
                self._node.set_active_map(map_name)
                self._db.add_map(map_name, pcd_path)

    def _do_auto_mapping(self) -> dict:
        """启动时直接建新图。30s 后启动 merge 检查循环。"""
        with self._node._lock:
            status = self._node._map_status

        if status == "mapping":
            if not self._node._active_map:
                map_name = f"map_{int(time.time())}"
                pcd_path = f"{self._map_dir}/{map_name}.pcd"
                self._node.set_active_map(map_name)
                self._db.add_map(map_name, pcd_path)
            return {"status": "already_mapping"}

        # 直接开始新地图
        print("[Spatial] Starting new map", flush=True)
        code, resp = self._client.StartMapping()
        if code == 0:
            map_name = f"map_{int(time.time())}"
            pcd_path = f"{self._map_dir}/{map_name}.pcd"
            self._node.clear_map_buffer()
            self._node.set_map_status("mapping")
            self._node.set_active_map(map_name)
            self._db.add_map(map_name, pcd_path)
            # 5s 后检查是否有旧图可用
            threading.Thread(target=self._try_recognize_existing_map, daemon=True).start()
            return {"status": "new", "map_name": map_name}
        return {"error": f"StartMapping failed, code={code}"}

    def _try_recognize_existing_map(self):
        """启动 5s 后：用 Scan Context + ICP 检查是否在已知地图中，设置 bias。"""
        time.sleep(5)
        try:
            current_map = self._node._active_map
            if not current_map:
                return

            # 1. 获取当前点云
            recent_cloud = self._node.get_recent_cloud()
            if recent_cloud is None or len(recent_cloud) < 500:
                print("[Spatial] Recognize: not enough cloud data yet, staying on new map", flush=True)
                return

            # 2. Scan Context 粗匹配
            sc = self._sc_mgr.make_scan_context(recent_cloud)
            match = self._sc_mgr.query(sc)
            if not match or match["map_name"] == current_map:
                print("[Spatial] Recognize: no matching old map found, staying on new map", flush=True)
                return

            old_map_name = match["map_name"]
            old_map_info = self._db.get_map(old_map_name)
            if not old_map_info:
                return

            old_pcd = old_map_info["pcd_path"]
            if not os.path.exists(old_pcd):
                print(f"[Spatial] Recognize: PCD not found: {old_pcd}", flush=True)
                return
            print(f"[Spatial] Recognize: matched '{old_map_name}' (score={match['score']:.4f})", flush=True)

            # 3. ICP 精确定位
            from icp import icp_2d
            from path_planner import PathPlanner

            target_cloud = PathPlanner._parse_pcd(old_pcd)
            with self._node._map_buffer_lock:
                source_cloud = np.array(list(self._node._map_buffer.values()), dtype=np.float32) if self._node._map_buffer else None

            bias_x = match["pose"]["x"]
            bias_y = match["pose"]["y"]
            bias_yaw = 0.0

            if target_cloud is not None and source_cloud is not None and len(source_cloud) >= 100 and len(target_cloud) >= 100:
                icp_result = icp_2d(source_cloud, target_cloud, init_x=bias_x, init_y=bias_y,
                                    max_iterations=50, max_correspond_dist=3.0)
                if icp_result and icp_result["score"] < 1.0:
                    bias_x = icp_result["x"]
                    bias_y = icp_result["y"]
                    bias_yaw = icp_result["yaw"]
                    print(f"[Spatial] Recognize ICP: bias_x={bias_x:.3f}, bias_y={bias_y:.3f}, bias_yaw={bias_yaw:.3f}, "
                          f"score={icp_result['score']:.4f}", flush=True)
                else:
                    print(f"[Spatial] Recognize ICP failed, using Scan Context pose as bias", flush=True)
            else:
                print(f"[Spatial] Recognize: can't run ICP (src={len(source_cloud) if source_cloud is not None else 0}, "
                      f"tgt={len(target_cloud) if target_cloud is not None else 0}), using Scan Context pose", flush=True)

            # 4. 设置 bias（不调 InitPose，SLAM 继续建图不受影响）
            self._node._bias_x = bias_x
            self._node._bias_y = bias_y
            self._node._bias_yaw = bias_yaw
            self._node._bias_set = True
            print(f"[Spatial] Bias set: ({bias_x:.3f}, {bias_y:.3f}, yaw={bias_yaw:.3f})", flush=True)

            # 5. 切换到旧图（复用 POI/tags），加载旧图点云到 buffer
            self._node.load_pcd_to_buffer(old_pcd)
            self._node.set_active_map(old_map_name)
            self._db.set_last_used_map(old_map_name)

            # 删除临时新图
            current_pcd = f"{self._map_dir}/{current_map}.pcd"
            self._db.delete_map(current_map)
            self._sc_mgr.clear_map(current_map)
            import subprocess as _sp
            _sp.run(["rm", "-f", current_pcd], capture_output=True)
            print(f"[Spatial] Recognized old map '{old_map_name}', bias applied, deleted temp '{current_map}'", flush=True)

        except Exception as e:
            print(f"[Spatial] Recognize error: {e}", flush=True)
            import traceback
            traceback.print_exc()

    # ── Dispatch ─────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            tool_name = args.get('_tool_name', '')
            if tool_name == 'slam_mapping':
                return {"state": "running", "topic_out": [{"topic": self._mapping_topic, "format": "sensor/mapping"}]}
            if tool_name == 'slam_cloud':
                return {"state": "running", "topic_out": [{"topic": self._slam_cloud_topic, "format": "sensor/pointcloud"}]}
            if tool_name == 'grid_map':
                return {"state": "running", "topic_out": [{"topic": self._grid_map_topic, "format": "sensor/mapping"}]}
            return {"state": "running", "topic_out": [{"topic": self._pos_tag_topic, "format": "data/json"}]}

        if action == "tag_place":
            name = args.get("name", "")
            if not name:
                return {"error": "name is required"}
            pose = self._node.get_pose()
            if not pose:
                return {"error": "No current pose available (SLAM not running?)"}
            active_map = self._node._active_map or "default"
            desc = args.get("description", "")
            self._db.add_poi(name, pose["x"], pose["y"], pose["yaw"], active_map, desc)
            return {"status": "tagged", "name": name, "pose": pose, "map": active_map}

        elif action == "untag_place":
            name = args.get("name", "")
            active_map = self._node._active_map or "default"
            if self._db.delete_poi(name, active_map):
                return {"status": "deleted", "name": name}
            return {"error": f"Tag '{name}' not found in map '{active_map}'"}

        elif action == "list_tags":
            active_map = self._node._active_map or "default"
            pois = self._db.list_pois(active_map)
            pose = self._node.get_pose()
            result = []
            for poi in pois:
                entry = {"name": poi["name"], "description": poi["description"],
                         "x": poi["x"], "y": poi["y"], "yaw": poi["yaw"]}
                if pose:
                    dx = poi["x"] - pose["x"]
                    dy = poi["y"] - pose["y"]
                    dist = math.sqrt(dx * dx + dy * dy)
                    cos_yaw = math.cos(-pose["yaw"])
                    sin_yaw = math.sin(-pose["yaw"])
                    rx = dx * cos_yaw - dy * sin_yaw
                    ry = dx * sin_yaw + dy * cos_yaw
                    entry["distance"] = round(dist, 2)
                    entry["bearing"] = _bearing_label(rx, ry)
                result.append(entry)
            return {"tags": result, "map": active_map}

        elif action == "list_maps":
            maps = self._db.list_maps()
            return {"maps": maps}

        elif action == "delete_map":
            map_name = args.get("map_name", "")
            if not map_name:
                return {"error": "map_name is required"}
            if self._node._active_map == map_name:
                return {"error": f"Cannot delete active map '{map_name}'."}
            if self._db.delete_map(map_name):
                self._sc_mgr.clear_map(map_name)
                return {"status": "deleted", "map_name": map_name}
            return {"error": f"Map '{map_name}' not found"}

        elif action == "relocalize_at_tag":
            tag_name = args.get("tag_name", "")
            if not tag_name:
                return {"error": "tag_name is required"}
            return self._relocalize_at_tag(tag_name)

        elif action == "navigate_to_tag":
            tag_name = args.get("tag_name", "")
            if not tag_name:
                return {"error": "tag_name is required"}
            active_map = self._node._active_map or "default"
            poi = self._db.find_poi(tag_name, active_map)
            if not poi:
                return {"error": f"Tag '{tag_name}' not found", "available": [p["name"] for p in self._db.list_pois(active_map)]}
            yaw = poi.get("yaw", 0)
            return self._navigate_with_planner(poi["x"], poi["y"], yaw, tag_name=tag_name)

        elif action == "navigate_to_pose":
            x = float(args.get("x", 0))
            y = float(args.get("y", 0))
            yaw = float(args.get("yaw", 0))
            return self._navigate_with_planner(x, y, yaw)

        elif action == "wait_navigation_done":
            stall_timeout = float(args.get("stall_timeout", 60))
            start_time = time.time()

            while self._nav_executing:
                time.sleep(1.0)
                if time.time() - start_time > stall_timeout:
                    self._nav_executing = False
                    return {"status": "timeout", "error": f"Navigation timeout after {stall_timeout}s"}

            if self._node._nav_error:
                error = self._node._nav_error
                self._node._nav_error = None
                return {"status": "error", "error": error}
            return {"status": "arrived", "pose": self._node.get_pose()}

        elif action == "pause_nav":
            self._nav_executing = False
            return {"status": "paused"}

        elif action == "resume_nav":
            return {"error": "resume not supported with path planner navigation"}

        elif action == "stop_nav":
            self._nav_executing = False
            return {"status": "stopped"}

        return None

    # ── Relocalization ────────────────────────────────────────────────────────

    def _relocalize_at_tag(self, tag_name: str) -> dict:
        """用 ICP 在已知 tag 附近精确定位 → 设置 bias。"""
        from icp import icp_2d

        # 查找 tag 所在的地图
        all_maps = self._db.list_maps()
        tag_info = None
        target_map = None
        for m in all_maps:
            poi = self._db.find_poi(tag_name, m["name"])
            if poi:
                tag_info = poi
                target_map = m
                break

        if not tag_info or not target_map:
            return {"error": f"Tag '{tag_name}' not found in any map"}

        pcd_path = target_map["pcd_path"]
        if not os.path.exists(pcd_path):
            return {"error": f"PCD file not found: {pcd_path}"}

        # 加载目标地图点云
        from path_planner import PathPlanner
        target_cloud = PathPlanner._parse_pcd(pcd_path)
        if target_cloud is None or len(target_cloud) < 100:
            return {"error": "Failed to load target map PCD"}

        # 获取当前点云
        with self._node._map_buffer_lock:
            if not self._node._map_buffer or len(self._node._map_buffer) < 100:
                return {"error": "Not enough current point cloud data (need at least 100 points, try again later)"}
            source_cloud = np.array(list(self._node._map_buffer.values()), dtype=np.float32)

        print(f"[Spatial] Relocalize: source={len(source_cloud)} pts, target={len(target_cloud)} pts, "
              f"init=({tag_info['x']:.2f}, {tag_info['y']:.2f}, yaw={tag_info['yaw']:.2f})", flush=True)

        # ICP 对齐
        result = icp_2d(
            source_cloud, target_cloud,
            init_x=tag_info["x"], init_y=tag_info["y"], init_yaw=tag_info["yaw"],
            max_iterations=50, tolerance=0.0005, max_correspond_dist=3.0
        )

        if result is None:
            return {"error": "ICP failed to converge"}

        print(f"[Spatial] ICP result: bias=({result['x']:.3f}, {result['y']:.3f}, yaw={result['yaw']:.3f}), "
              f"score={result['score']:.4f}, iters={result['iterations']}", flush=True)

        if result["score"] > 1.0:
            return {"error": f"ICP score too high ({result['score']:.3f}), possibly diverged", "icp_result": result}

        # 设置 bias（不调 InitPose）
        self._node._bias_x = result["x"]
        self._node._bias_y = result["y"]
        self._node._bias_yaw = result["yaw"]
        self._node._bias_set = True

        # 切换到目标地图
        self._node.load_pcd_to_buffer(pcd_path)
        self._node.set_active_map(target_map["name"])
        self._db.set_last_used_map(target_map["name"])

        return {
            "status": "relocalized",
            "map": target_map["name"],
            "bias": {"x": result["x"], "y": result["y"], "yaw": result["yaw"]},
            "icp_score": result["score"],
            "icp_iterations": result["iterations"],
        }

    # ── Path Planner Navigation ──────────────────────────────────────────────

    def _ensure_planner_loaded(self) -> bool:
        """加载/刷新路径规划器（每次导航重新加载，因为地图在持续增长）。"""
        # 每次导航都重新加载最新地图数据
        with self._node._map_buffer_lock:
            if self._node._map_buffer:
                pts = np.array(list(self._node._map_buffer.values()), dtype=np.float32)
                if self._planner.load_from_buffer(pts):
                    grid = self._planner._grid
                    occupied = np.sum(grid > 0) if grid is not None else 0
                    total = grid.size if grid is not None else 0
                    print(f"[Spatial] PathPlanner refreshed: {len(pts)} pts, "
                          f"grid {self._planner._width}x{self._planner._height}, "
                          f"occupied={occupied}/{total} ({100*occupied/max(total,1):.1f}%)", flush=True)
                    return True
        # 尝试从 PCD 文件加载
        active_map = self._node._active_map
        if active_map:
            pcd_path = f"{self._map_dir}/{active_map}.pcd"
            if os.path.exists(pcd_path):
                if self._planner.load_pcd(pcd_path):
                    return True
        return False

    def _navigate_with_planner(self, target_x: float, target_y: float, target_yaw: float,
                               tag_name: str | None = None) -> dict:
        """路径规划 + 导航执行。全程在 SLAM 坐标系下工作。"""
        # 起点：SLAM 原始坐标（planner 栅格也是 SLAM 坐标系）
        pose_raw = self._node.get_pose_raw()
        if not pose_raw:
            return {"error": "No current pose available"}

        if not self._ensure_planner_loaded():
            return {"error": "Path planner: no map loaded"}

        # 目标：map 坐标 → SLAM 坐标
        target_slam = self._node.map_to_slam(target_x, target_y)

        # 设置栅格障碍物（用于 grid_map 2D 显示）
        grid_pts = self._planner.get_grid_as_points()
        if grid_pts is not None:
            print(f"[Spatial] Grid: {len(grid_pts)} obstacle cells", flush=True)

        # 规划路径（SLAM 坐标系）
        waypoints = self._planner.plan((pose_raw["x"], pose_raw["y"]), target_slam)
        if not waypoints:
            return {"error": f"No path found from ({pose_raw['x']:.1f},{pose_raw['y']:.1f}) to ({target_slam[0]:.1f},{target_slam[1]:.1f})"}

        # 生成路径 overlay（SLAM 坐标，因为 mapping 点云也是 SLAM 坐标）
        overlay = self._build_path_overlay_with_bias(waypoints)
        self._node._nav_path_overlay = overlay
        overlay_len = len(overlay) if overlay is not None else 0
        print(f"[Spatial] Nav path: {len(waypoints)} waypoints, overlay={overlay_len} pts, "
              f"start=({pose_raw['x']:.2f},{pose_raw['y']:.2f}), "
              f"target_slam=({target_slam[0]:.2f},{target_slam[1]:.2f})", flush=True)

        # 设置导航状态
        self._nav_waypoints = waypoints
        self._nav_target_yaw = target_yaw
        self._nav_executing = True
        self._node._nav_arrived.clear()
        self._node._nav_error = None
        if tag_name:
            self._node.set_nav_target(tag_name)

        # 后台执行移动
        threading.Thread(target=self._execute_waypoints, daemon=True).start()

        return {
            "status": "navigating",
            "target": tag_name or f"({target_x:.1f},{target_y:.1f})",
            "start_slam": {"x": round(pose_raw["x"], 2), "y": round(pose_raw["y"], 2)},
            "target_slam": {"x": round(target_slam[0], 2), "y": round(target_slam[1], 2)},
            "waypoints": len(waypoints),
            "waypoint_coords": [(round(x, 2), round(y, 2)) for x, y in waypoints],
            "method": "velocity_control",
        }

    def _build_path_overlay(self, waypoints: list[tuple], start_idx: int = 0) -> np.ndarray | None:
        """生成剩余 waypoints 的路径点 overlay（SLAM 坐标，直接用于 mapping 显示）。"""
        remaining = waypoints[start_idx:]
        if len(remaining) < 2:
            return None
        PATH_WIDTH = 0.05
        PATH_Z = 0.3
        PATH_STEP = 0.05
        OFFSETS = [0, -PATH_WIDTH, PATH_WIDTH]
        path_points = []
        for i in range(len(remaining) - 1):
            x1, y1 = remaining[i]
            x2, y2 = remaining[i + 1]
            dx = x2 - x1
            dy = y2 - y1
            seg_len = math.sqrt(dx * dx + dy * dy)
            if seg_len < 1e-6:
                continue
            nx = -dy / seg_len
            ny = dx / seg_len
            steps = max(int(seg_len / PATH_STEP), 1)
            for s in range(steps):
                t = s / steps
                cx = x1 + t * dx
                cy = y1 + t * dy
                for offset in OFFSETS:
                    path_points.append((cx + offset * nx, cy + offset * ny, PATH_Z))
        if not path_points:
            return None
        return np.array(path_points, dtype=np.float32)

    def _build_path_overlay_with_bias(self, waypoints: list[tuple]) -> np.ndarray | None:
        """生成路径 overlay，waypoints 是 SLAM 坐标，不需要额外变换
        （因为 mapping 显示的点云也是 SLAM 坐标）。"""
        return self._build_path_overlay(waypoints, 0)

    @staticmethod
    def _normalize_angle(a: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def _execute_waypoints(self):
        """后台线程：平滑导航 + 动态重规划。

        - 首个 waypoint: 原地转向后前进
        - 后续 waypoints: look-ahead 预瞄，边走边转不停顿
        - 每 1s 检测路径障碍，自动重规划
        """
        final_goal = self._nav_waypoints[-1]  # 最终目标（重规划用）
        last_replan_time = time.time()

        try:
            i = 0
            while i < len(self._nav_waypoints) and self._nav_executing:
                wx, wy = self._nav_waypoints[i]
                is_first = (i == 0)
                is_last = (i == len(self._nav_waypoints) - 1)

                print(f"[Spatial] Waypoint {i+1}/{len(self._nav_waypoints)}: ({wx:.2f}, {wy:.2f})", flush=True)

                # 更新 nav_path overlay
                overlay = self._build_path_overlay(self._nav_waypoints, i)
                self._node._nav_path_overlay = overlay

                # 首个 waypoint: 先原地转向
                if is_first:
                    turn_start = time.time()
                    while self._nav_executing:
                        pose = self._node.get_pose_raw()
                        if not pose:
                            time.sleep(0.1)
                            continue
                        dx = wx - pose["x"]
                        dy = wy - pose["y"]
                        dist = math.sqrt(dx * dx + dy * dy)
                        if dist < 0.3:
                            break
                        heading = math.atan2(dy, dx)
                        err = self._normalize_angle(heading - pose["yaw"])
                        if abs(err) < 0.2:
                            break
                        vyaw = max(-1.5, min(1.5, err * 1.5))
                        self._rpc_proxy.Move(0, 0, vyaw)
                        time.sleep(0.1)
                        if time.time() - turn_start > 8:
                            break

                # 前进到 waypoint（边走边转，look-ahead 预瞄）
                stall_start = time.time()
                last_pose = self._node.get_pose_raw()
                while self._nav_executing:
                    pose = self._node.get_pose_raw()
                    if not pose:
                        time.sleep(0.1)
                        continue

                    dx = wx - pose["x"]
                    dy = wy - pose["y"]
                    dist = math.sqrt(dx * dx + dy * dy)

                    if dist < 0.3:
                        print(f"[Spatial] Waypoint {i+1} reached (dist={dist:.2f}m)", flush=True)
                        break

                    # 计算航向：look-ahead 预瞄下一个 waypoint
                    current_heading = math.atan2(dy, dx)
                    target_heading = current_heading

                    if not is_last and dist < 1.0:
                        next_wx, next_wy = self._nav_waypoints[i + 1]
                        next_heading = math.atan2(next_wy - pose["y"], next_wx - pose["x"])
                        blend = 1.0 - dist / 1.0  # 越近当前 wp，越偏向下一个
                        # 角度混合（处理 -pi/pi 边界）
                        diff = self._normalize_angle(next_heading - current_heading)
                        target_heading = current_heading + blend * diff

                    heading_err = self._normalize_angle(target_heading - pose["yaw"])

                    # 速度控制
                    if abs(heading_err) > 0.8:
                        # 偏航很大：大幅减速转向
                        vx = 0.05
                        vyaw = max(-2.0, min(2.0, heading_err * 2.0))
                    elif abs(heading_err) > 0.3:
                        # 中等偏航：减速+转
                        vx = max(0.1, min(0.3, dist * 0.5))
                        vyaw = max(-1.5, min(1.5, heading_err * 1.5))
                    else:
                        # 方向正确：全速前进+微调
                        vx = max(0.15, min(1.0, dist * 0.8))
                        vyaw = max(-0.8, min(0.8, heading_err * 1.0))

                    self._rpc_proxy.Move(vx, 0, vyaw)
                    time.sleep(0.1)

                    # 动态重规划（每 1s）
                    now = time.time()
                    if now - last_replan_time > 1.0:
                        last_replan_time = now
                        if self._try_replan(i, final_goal):
                            # 路径已更新，重置 waypoint 索引到 0（从新路径开始）
                            i = 0
                            overlay = self._build_path_overlay(self._nav_waypoints, 0)
                            self._node._nav_path_overlay = overlay
                            break  # 跳出内层循环，重新从 while i < len 开始

                    # 卡住检测
                    if last_pose:
                        moved = math.sqrt(
                            (pose["x"] - last_pose["x"])**2 +
                            (pose["y"] - last_pose["y"])**2
                        )
                        if moved > 0.03:
                            stall_start = time.time()
                    last_pose = pose
                    if now - stall_start > 30:
                        print(f"[Spatial] Waypoint {i+1} stalled, skipping", flush=True)
                        break
                else:
                    # while 正常退出（不是 break）→ 重规划触发了，继续外层循环
                    continue

                i += 1  # 下一个 waypoint（只有 break 到达时才递增）

            # 最终转到目标朝向
            if self._nav_executing and self._nav_target_yaw != 0:
                print(f"[Spatial] Final yaw adjustment to {self._nav_target_yaw:.2f}", flush=True)
                for _ in range(50):
                    pose = self._node.get_pose_raw()
                    if not pose:
                        time.sleep(0.1)
                        continue
                    yaw_err = self._normalize_angle(self._nav_target_yaw - pose["yaw"])
                    if abs(yaw_err) < 0.15:
                        break
                    vyaw = max(-1.0, min(1.0, yaw_err * 1.5))
                    self._rpc_proxy.Move(0, 0, vyaw)
                    time.sleep(0.1)

            self._rpc_proxy.Move(0, 0, 0)

            if self._nav_executing:
                self._nav_executing = False
                self._node._nav_path_overlay = None
                self._node._nav_arrived.set()
                print("[Spatial] Navigation complete", flush=True)

        except Exception as e:
            print(f"[Spatial] Navigation error: {e}", flush=True)
            import traceback
            traceback.print_exc()
            self._rpc_proxy.Move(0, 0, 0)
            self._nav_executing = False
            self._node._nav_error = str(e)
            self._node._nav_path_overlay = None
            self._node._nav_arrived.set()

    def _try_replan(self, current_wp_idx: int, final_goal: tuple) -> bool:
        """检查剩余路径是否被阻挡，如果是则重新规划。返回 True 如果路径已更新。"""
        remaining = self._nav_waypoints[current_wp_idx:]
        if len(remaining) < 2:
            return False

        # 刷新 planner 地图数据
        with self._node._map_buffer_lock:
            if not self._node._map_buffer:
                return False
            pts = np.array(list(self._node._map_buffer.values()), dtype=np.float32)
        self._planner.load_from_buffer(pts)

        # 检查剩余 waypoints 是否有被新障碍阻挡的
        blocked = False
        for wx, wy in remaining:
            if not self._planner.is_reachable(wx, wy):
                blocked = True
                break
        if not blocked:
            return False

        # 重新规划
        pose = self._node.get_pose_raw()
        if not pose:
            return False
        new_path = self._planner.plan((pose["x"], pose["y"]), final_goal)
        if new_path and len(new_path) >= 2:
            print(f"[Spatial] Replanned: {len(self._nav_waypoints)} → {len(new_path)} waypoints (obstacle detected)", flush=True)
            self._nav_waypoints = new_path
            return True
        return False
