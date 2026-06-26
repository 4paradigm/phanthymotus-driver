"""
ControlledSpatialPlugin — 人工控制建图与导航 actuator。

用户流程：
1. start_mapping(map_name) → 开始建图，用遥控器控制机器人行走
2. tag_place(name) → 记录当前位置/朝向，关联语义 tag
3. list_tags → 列出当前地图所有 tag
4. stop_mapping → 停止建图并保存
5. list_maps → 列出所有已保存地图
6. delete_map(map_name) → 删除地图及关联数据
7. load_map(map_name) → 载入地图（机器人需站在初始点）
8. navigate_to_tag(tag_name) → 导航到指定 tag
"""

import json
import math
import os
import sqlite3
import threading
import time


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


# ── Database ─────────────────────────────────────────────────────────────────

class _ControlledSpatialDB:
    """SQLite storage for controlled maps and POIs."""

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
        """)
        self._conn.commit()

    def add_map(self, name: str, pcd_path: str):
        self._conn.execute(
            "INSERT OR REPLACE INTO maps (name, pcd_path) VALUES (?, ?)", (name, pcd_path)
        )
        self._conn.commit()

    def get_map(self, name: str) -> dict | None:
        row = self._conn.execute("SELECT * FROM maps WHERE name = ?", (name,)).fetchone()
        return dict(row) if row else None

    def list_maps(self) -> list[dict]:
        rows = self._conn.execute("SELECT name, pcd_path, created_at FROM maps ORDER BY created_at DESC").fetchall()
        return [dict(r) for r in rows]

    def delete_map(self, name: str) -> bool:
        """Delete map record and associated POIs. PCD deletion is best-effort."""
        map_info = self.get_map(name)
        if not map_info:
            return False
        # Try to delete PCD file (may fail if running in container)
        pcd_path = map_info["pcd_path"]
        try:
            os.remove(pcd_path)
        except OSError:
            pass
        # Delete POIs and map record
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


# ── Helpers ──────────────────────────────────────────────────────────────────

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


# ── Plugin ───────────────────────────────────────────────────────────────────

class ControlledSpatialPlugin:
    """Controlled mapping & navigation — user drives the robot manually during mapping."""

    PREFIX = "controlled_spatial"

    def __init__(self, plugin_config: dict, namespace: str, executor, slam_client, smart_motion=None):
        self._client = slam_client
        self._smart_motion = smart_motion
        self._pcd_dir = plugin_config.get("native_slam_pcd_dir", "/home/unitree")  # SLAM 服务写 PCD 的机器人本机路径
        db_path = plugin_config.get("native_slam_db_path", "/opt/phanthy-motus/data/controlled_spatial.db")
        self._db = _ControlledSpatialDB(db_path)

        # State
        self._active_map: str | None = None
        self._is_mapping: bool = False
        self._current_pose: dict | None = None
        self._map_status: str = "idle"  # idle | mapping | localized
        self._lock = threading.Lock()

        # Subscribe DDS for pose updates
        self._dds_subs = []
        try:
            from unitree_sdk2py.core.channel import ChannelSubscriber
            from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_
            info_sub = ChannelSubscriber("rt/slam_info", String_)
            info_sub.Init(self._on_slam_info, 10)
            self._dds_subs.append(info_sub)
            print("[ControlledSpatial] subscribed rt/slam_info")
        except Exception as e:
            print(f"[ControlledSpatial] failed to subscribe rt/slam_info: {e}")

    def get_tools(self) -> list:
        return [self._tool_def()]

    def _tool_def(self) -> dict:
        return {
            "name": "controlled_spatial",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Controlled mapping & navigation — user manually drives the robot during mapping. "
                "Supports: start/stop mapping, tag places, list/delete maps, load map, navigate between tags."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "start_mapping", "stop_mapping",
                            "tag_place", "untag_place", "list_tags",
                            "list_maps", "delete_map",
                            "load_map",
                            "navigate_to_tag", "navigate_to_pose",
                            "pause_nav", "resume_nav", "stop_nav",
                        ],
                        "description": "Action to perform",
                    },
                    "map_name": {"type": "string", "description": "Map name (for start_mapping, delete_map, load_map)"},
                    "name": {"type": "string", "description": "POI tag name"},
                    "description": {"type": "string", "description": "POI description"},
                    "tag_name": {"type": "string", "description": "Target tag name for navigation"},
                    "x": {"type": "number", "description": "Target X coordinate (meters)"},
                    "y": {"type": "number", "description": "Target Y coordinate (meters)"},
                    "yaw": {"type": "number", "description": "Target yaw (radians)"},
                },
                "required": ["action"],
                "x-action-params": {
                    "start_mapping": {"params": ["map_name"], "description": "Start SLAM mapping with given map name"},
                    "stop_mapping": {"params": [], "description": "Stop mapping and save the map"},
                    "tag_place": {"params": ["name", "description"], "description": "Tag current position with a semantic name"},
                    "untag_place": {"params": ["name"], "description": "Remove a place tag"},
                    "list_tags": {"params": [], "description": "List all tags in current map with relative positions"},
                    "list_maps": {"params": [], "description": "List all saved maps"},
                    "delete_map": {"params": ["map_name"], "description": "Delete a map and its associated data"},
                    "load_map": {"params": ["map_name"], "description": "Load a map (robot must be at map origin)"},
                    "navigate_to_tag": {"params": ["tag_name"], "description": "Navigate to a tagged place"},
                    "navigate_to_pose": {"params": ["x", "y", "yaw"], "description": "Navigate to coordinates"},
                    "pause_nav": {"params": [], "description": "Pause navigation"},
                    "resume_nav": {"params": [], "description": "Resume navigation"},
                    "stop_nav": {"params": [], "description": "Stop and cancel navigation"},
                },
            },
        }

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    # ── DDS callback ─────────────────────────────────────────────────────────

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
                    self._current_pose = {
                        "x": pose_data["x"],
                        "y": pose_data["y"],
                        "yaw": round(yaw, 3),
                    }
                    if msg_type == "pos_info":
                        self._map_status = "localized"
                    elif msg_type == "mapping_info":
                        self._map_status = "mapping"

    def _get_pose(self) -> dict | None:
        with self._lock:
            return dict(self._current_pose) if self._current_pose else None

    # ── Dispatch ─────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "info":
            return {"state": "running"}

        elif action == "start_mapping":
            map_name = args.get("map_name", "")
            if not map_name:
                return {"error": "map_name is required"}
            code, resp = self._client.StartMapping()
            if code == 0:
                pcd_path = f"{self._pcd_dir}/controlled_{map_name}.pcd"
                self._active_map = map_name
                self._is_mapping = True
                self._db.add_map(map_name, pcd_path)
                return {"status": "mapping", "map_name": map_name}

            # RPC timeout — check DDS state to see if mapping actually started
            if code == 3104:
                time.sleep(1)
                with self._lock:
                    actually_mapping = self._map_status == "mapping"
                if actually_mapping:
                    print(f"[ControlledSpatial] StartMapping RPC timeout but DDS confirms mapping started")
                    pcd_path = f"{self._pcd_dir}/controlled_{map_name}.pcd"
                    self._active_map = map_name
                    self._is_mapping = True
                    self._db.add_map(map_name, pcd_path)
                    return {"status": "mapping", "map_name": map_name,
                            "warning": "RPC timeout but mapping confirmed via DDS"}

            return _rpc_error("StartMapping", code, resp)

        elif action == "stop_mapping":
            if not self._active_map:
                return {"error": "No active mapping session"}
            pcd_path = f"{self._pcd_dir}/controlled_{self._active_map}.pcd"
            map_name = self._active_map

            # 尝试停止建图，超时则重试一次（保存 PCD 可能耗时较长）
            code, resp = self._client.StopMapping(pcd_path)
            if code == 3104:
                print(f"[ControlledSpatial] StopMapping timeout, retrying...")
                code, resp = self._client.StopMapping(pcd_path)

            if code == 0:
                self._is_mapping = False
                self._active_map = None
                return {"status": "stopped", "map_name": map_name, "pcd_path": pcd_path}

            # 即使 RPC 超时，如果 DDS 回调显示已不在建图状态，视为成功
            with self._lock:
                actually_mapping = self._map_status == "mapping"
            if not actually_mapping:
                print(f"[ControlledSpatial] StopMapping RPC failed (code={code}) but DDS shows not mapping, treating as success")
                self._is_mapping = False
                self._active_map = None
                return {"status": "stopped", "map_name": map_name, "pcd_path": pcd_path,
                        "warning": f"RPC returned code={code} but SLAM already stopped"}

            return _rpc_error("StopMapping", code, resp)

        elif action == "tag_place":
            name = args.get("name", "")
            if not name:
                return {"error": "name is required"}
            pose = self._get_pose()
            if not pose:
                return {"error": "No current pose available (SLAM not running?)"}
            active_map = self._active_map
            if not active_map:
                return {"error": "No active map. Start mapping or load a map first."}
            desc = args.get("description", "")
            self._db.add_poi(name, pose["x"], pose["y"], pose["yaw"], active_map, desc)
            return {"status": "tagged", "name": name, "pose": pose, "map": active_map}

        elif action == "untag_place":
            name = args.get("name", "")
            if not name:
                return {"error": "name is required"}
            active_map = self._active_map
            if not active_map:
                return {"error": "No active map"}
            if self._db.delete_poi(name, active_map):
                return {"status": "deleted", "name": name}
            return {"error": f"Tag '{name}' not found in map '{active_map}'"}

        elif action == "list_tags":
            active_map = self._active_map
            if not active_map:
                return {"error": "No active map. Start mapping or load a map first."}
            pois = self._db.list_pois(active_map)
            pose = self._get_pose()
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
            if self._active_map == map_name:
                return {"error": f"Cannot delete active map '{map_name}'. Stop mapping or unload first."}
            if self._db.delete_map(map_name):
                return {"status": "deleted", "map_name": map_name}
            return {"error": f"Map '{map_name}' not found"}

        elif action == "load_map":
            map_name = args.get("map_name", "")
            if not map_name:
                return {"error": "map_name is required"}
            if self._is_mapping:
                return {"error": "Cannot load map while mapping is active. Stop mapping first."}
            map_info = self._db.get_map(map_name)
            if not map_info:
                return {"error": f"Map '{map_name}' not found"}
            pcd_path = map_info["pcd_path"]
            # InitPose with origin (robot at map start point)
            code, resp = self._client.InitPose(0, 0, 0, 0, 0, 0, 1.0, pcd_path)
            if code == 0:
                self._active_map = map_name
                return {"status": "loaded", "map_name": map_name, "pcd_path": pcd_path}
            return _rpc_error("InitPose (load map)", code, resp)

        elif action == "navigate_to_tag":
            tag_name = args.get("tag_name", "")
            if not tag_name:
                return {"error": "tag_name is required"}
            active_map = self._active_map
            if not active_map:
                return {"error": "No active map. Load a map first."}
            poi = self._db.find_poi(tag_name, active_map)
            if not poi:
                available = [p["name"] for p in self._db.list_pois(active_map)]
                return {"error": f"Tag '{tag_name}' not found", "available": available}
            yaw = poi.get("yaw", 0)

            if self._smart_motion:
                return self._smart_motion.navigate_to(poi["x"], poi["y"], yaw, tag_name)

            # Fallback: direct SLAM navigation
            q_z = math.sin(yaw / 2)
            q_w = math.cos(yaw / 2)
            code, resp = self._client.NavigateTo(poi["x"], poi["y"], 0, 0, 0, q_z, q_w)
            if code == 0:
                return {"status": "navigating", "target": tag_name, "pose": {"x": poi["x"], "y": poi["y"], "yaw": yaw}}
            return _rpc_error("NavigateTo", code, resp)

        elif action == "navigate_to_pose":
            x = float(args.get("x", 0))
            y = float(args.get("y", 0))
            yaw = float(args.get("yaw", 0))

            if self._smart_motion:
                return self._smart_motion.navigate_to(x, y, yaw)

            q_z = math.sin(yaw / 2)
            q_w = math.cos(yaw / 2)
            code, resp = self._client.NavigateTo(x, y, 0, 0, 0, q_z, q_w)
            if code == 0:
                return {"status": "navigating", "target_pose": {"x": x, "y": y, "yaw": yaw}}
            return _rpc_error("NavigateTo", code, resp)

        elif action == "pause_nav":
            if self._smart_motion:
                return self._smart_motion.pause_nav()
            code, resp = self._client.PauseNav()
            return {"status": "paused"} if code == 0 else _rpc_error("PauseNav", code)

        elif action == "resume_nav":
            if self._smart_motion:
                return self._smart_motion.resume_nav()
            code, resp = self._client.ResumeNav()
            return {"status": "resumed"} if code == 0 else _rpc_error("ResumeNav", code)

        elif action == "stop_nav":
            if self._smart_motion:
                return self._smart_motion.stop_nav()
            self._client.PauseNav()
            return {"status": "stopped"}

        return None
