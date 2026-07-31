"""
controlled_spatial — 天轶2.0 人工控制建图与导航插件。

通过 Slamtec HTTP REST API 实现与 G1 controlled_spatial.py 相同的功能：
1. start_mapping(map_name) → 开始建图，用遥控器控制机器人行走
2. tag_place(name) → 记录当前位置/朝向，关联语义 tag
3. list_tags → 列出当前地图所有 tag
4. stop_mapping → 停止建图并保存
5. list_maps → 列出所有已保存地图
6. delete_map(map_name) → 删除地图及关联数据
7. load_map(map_name) → 载入地图
8. navigate_to_tag(tag_name) → 导航到指定 tag

差异：G1 使用 DDS RPC (unitree_sdk2py.SlamClient)，tianyi2.0 使用 HTTP REST API。
"""

import json
import math
import os
import threading
import time
import sqlite3
from typing import Optional


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
        # Only update the map path, do NOT delete POIs — they are tagged during mapping
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


# ── Plugin ───────────────────────────────────────────────────────────────────

class ControlledSpatialPlugin:
    """Controlled mapping & navigation via Slamtec HTTP REST API."""

    def __init__(self, plugin_config: dict, namespace: str, ros2, slamtec_client):
        self._slamtec = slamtec_client
        self._ros2 = ros2
        # Resolve paths relative to this script's directory (works in Docker /work/ and local dev)
        _script_dir = os.path.dirname(os.path.abspath(__file__))
        self._pcd_dir = os.path.join(_script_dir, "maps")
        db_path = os.path.join(_script_dir, "maps", "controlled_spatial.db")
        # Allow config overrides for explicit paths
        if plugin_config.get("native_slam_pcd_dir"):
            pcd_cfg = plugin_config["native_slam_pcd_dir"]
            self._pcd_dir = pcd_cfg if os.path.isabs(pcd_cfg) else os.path.join(_script_dir, pcd_cfg)
        if plugin_config.get("native_slam_db_path"):
            db_cfg = plugin_config["native_slam_db_path"]
            db_path = db_cfg if os.path.isabs(db_cfg) else os.path.join(_script_dir, db_cfg)
        os.makedirs(self._pcd_dir, exist_ok=True)
        self._db = _ControlledSpatialDB(db_path)

        # State
        self._active_map: str | None = None
        self._is_mapping: bool = False
        self._current_pose: dict | None = None
        self._map_status: str = "idle"  # idle | mapping | localized | localizing
        self._nav_arrived = threading.Event()
        self._nav_error: str | None = None
        self._nav_active: bool = False  # True when navigation is in progress
        self._nav_action_id: str | None = None  # Action ID returned by move_to
        self._nav_start_time: float = 0  # monotonic time when navigation was initiated
        self._nav_lost_count: int = 0  # consecutive polls where action was missing on chassis
        self._lock = threading.Lock()
        self._poll_thread: Optional[threading.Thread] = None

        # Start polling thread for pose, nav status, map status
        self._poll_running = False

    def get_tools(self) -> list:
        return [self._tool_def()]

    def _tool_def(self) -> dict:
        return {
            "name": "controlled_spatial",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Controlled mapping & navigation via Slamtec底盘 HTTP REST API — "
                "start/stop mapping, tag places, list/delete maps, load map, navigate between tags."
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
                            "wait_navigation_done",
                            "pause_nav", "stop_nav",
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
                    "speed": {"type": "number", "description": "Navigation speed ratio 0.1-1.0 (default 0.5)"},
                    "mode": {"type": "integer", "description": "Navigation mode: 0=free (default), 1=strict-track (stop on obstacle, requires tracks), 2=track-priority (detour on obstacle)"},
                    "fail_retry_count": {"type": "integer", "description": "Path planning retry count on failure (default 3)"},
                    "acceptable_precision": {"type": "number", "description": "Acceptable arrival distance in meters when target is occupied (default 0.18)"},
                    "stall_timeout": {"type": "number", "description": "Seconds without movement before declaring timeout (default 60)"},
                    "strategy": {"type": "string", "description": "Motion strategy: default, depot, inventory, delivery, low_speed"},
                    "ignore_dynamic_obstacles": {"type": "boolean", "description": "Ignore dynamic obstacles during path planning (default true)"},
                    "precise": {"type": "boolean", "description": "Enable precise navigation mode (default false)"},
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
                    "navigate_to_tag": {"params": ["tag_name", "speed", "mode", "fail_retry_count", "acceptable_precision", "strategy", "ignore_dynamic_obstacles", "precise"], "description": "Navigate to a tagged place. Returns immediately — call wait_navigation_done to wait for arrival."},
                    "navigate_to_pose": {"params": ["x", "y", "yaw", "speed", "mode", "fail_retry_count", "acceptable_precision", "strategy", "ignore_dynamic_obstacles", "precise"], "description": "Navigate to coordinates. Returns immediately — call wait_navigation_done to wait for arrival."},
                    "wait_navigation_done": {"params": ["stall_timeout"], "description": "Block until navigation completes or robot is stuck (no movement for stall_timeout seconds). Must be called after navigate_to_tag or navigate_to_pose."},
                    "pause_nav": {"params": [], "description": "Pause navigation"},
                    "stop_nav": {"params": [], "description": "Stop and cancel navigation"},
                },
            },
        }

    def start(self) -> None:
        self._poll_running = True
        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()
        print("[ControlledSpatial] poll thread started")

    def stop(self) -> None:
        self._poll_running = False

    # ── Polling loop (替代 DDS callback) ─────────────────────────────────────

    def _poll_loop(self):
        """Poll pose, nav status, map status via HTTP at 5Hz. Replaces DDS rt/slam_info subscription."""
        while self._poll_running:
            try:
                # Poll pose
                pose = self._slamtec.get_pose()
                if isinstance(pose, dict) and "x" in pose and "y" in pose:
                    yaw = pose.get("yaw", 0)
                    # Convert quaternion yaw if needed
                    with self._lock:
                        self._current_pose = {
                            "x": pose.get("x", 0),
                            "y": pose.get("y", 0),
                            "yaw": yaw,
                        }
                        if self._is_mapping:
                            self._map_status = "mapping"
                        else:
                            self._map_status = "localized"

                # Poll nav status
                # API ActionState.status: 0=NewBorn, 1=Working, 3=Paused, 4=Done
                # API ActionState.result: 0=Success, -1=Failed, -2=Aborted
                # action_state=-1 means no active action on chassis (not an error)
                nav_status = self._slamtec.get_nav_status()
                if isinstance(nav_status, dict) and not nav_status.get("error"):
                    action_state = nav_status.get("action_state")
                    result_code = nav_status.get("result")
                    if action_state is not None:
                        action_state = int(action_state)
                    if result_code is not None:
                        result_code = int(result_code)

                    # Debug: log action state when nav is active
                    if self._nav_active:
                        print(f"[ControlledSpatial] poll: action_state={action_state}, result={result_code}, "
                              f"stage={nav_status.get('stage')}, action_id={nav_status.get('action_id')}, "
                              f"action_name={nav_status.get('action_name')}, lost_count={self._nav_lost_count}")

                    if action_state == 4 and result_code == 0:
                        # Done + Success → arrived
                        self._nav_arrived.set()
                        self._nav_active = False
                        self._nav_lost_count = 0
                        self._map_status = "localized"
                    elif action_state == 4 and result_code in (-1, -2):
                        # Done + Failed/Aborted
                        label = "failed" if result_code == -1 else "aborted"
                        reason = nav_status.get("reason", "")
                        self._nav_error = f"action {label}: result={result_code}" + (f", reason={reason}" if reason else "")
                        self._nav_arrived.set()
                        self._nav_active = False
                        self._nav_lost_count = 0
                        self._map_status = "localized"
                    elif action_state == 3:
                        # Paused — still active, don't treat as lost
                        self._nav_lost_count = 0
                        self._map_status = "navigating"
                    elif action_state in (0, 1):
                        # NewBorn or Working
                        self._nav_lost_count = 0
                        self._map_status = "navigating"
                        # Verify the current action matches our expected action_id.
                        # If load_map left a RecoverLocalizationAction running,
                        # we'd see action_state=1 but for the WRONG action.
                        current_action_id = nav_status.get("action_id")
                        if (self._nav_action_id is not None
                                and current_action_id is not None
                                and str(current_action_id) != str(self._nav_action_id)):
                            print(f"[ControlledSpatial] WARNING: current action_id={current_action_id} "
                                  f"!= expected action_id={self._nav_action_id}. "
                                  f"action_name={nav_status.get('action_name')}, stage={nav_status.get('stage')}")
                    elif action_state == -1:
                        # No active action on chassis — action may have completed
                        # between polls. Query the specific action by ID to get
                        # the final result (API keeps last 20 actions for query).
                        if self._nav_active and self._nav_action_id is not None:
                            final_status = self._slamtec.get_action_status(str(self._nav_action_id))
                            print(f"[ControlledSpatial] action_state=-1, querying action_id={self._nav_action_id}: {final_status}")
                            if isinstance(final_status, dict) and not final_status.get("error"):
                                # Unwrap nested state like get_nav_status does
                                fs = final_status.pop("state", None) if isinstance(final_status.get("state"), dict) else None
                                if isinstance(fs, dict):
                                    final_action_state = fs.get("status")
                                    final_result = fs.get("result")
                                else:
                                    final_action_state = final_status.get("action_state")
                                    final_result = final_status.get("result")
                                if final_action_state is not None:
                                    final_action_state = int(final_action_state)
                                if final_result is not None:
                                    final_result = int(final_result)

                                if final_action_state == 4 and final_result == 0:
                                    # Done + Success → arrived
                                    self._nav_arrived.set()
                                    self._nav_active = False
                                    self._nav_lost_count = 0
                                    self._map_status = "localized"
                                elif final_action_state == 4 and final_result in (-1, -2):
                                    # Done + Failed/Aborted
                                    label = "failed" if final_result == -1 else "aborted"
                                    reason = fs.get("reason", "") if isinstance(fs, dict) else ""
                                    self._nav_error = f"action {label}: result={final_result}" + (f", reason={reason}" if reason else "")
                                    self._nav_arrived.set()
                                    self._nav_active = False
                                    self._nav_lost_count = 0
                                    self._map_status = "localized"
                                else:
                                    # Action exists but not done yet — shouldn't happen
                                    # since :current returned 404, but handle gracefully
                                    self._nav_lost_count += 1
                                    if self._nav_lost_count >= 5:
                                        self._nav_error = "Action lost on chassis"
                                        self._nav_arrived.set()
                                        self._nav_active = False
                                        self._nav_lost_count = 0
                            else:
                                # Can't query action by ID either — truly lost
                                self._nav_lost_count += 1
                                if self._nav_lost_count >= 5:
                                    self._nav_error = "Action lost on chassis"
                                    self._nav_arrived.set()
                                    self._nav_active = False
                                    self._nav_lost_count = 0
                else:
                    # HTTP error from chassis — don't count as "action lost"
                    # Could be transient network issue; just skip this poll cycle
                    pass

            except Exception:
                pass
            time.sleep(0.2)  # 5Hz

    # ── Pose helpers ─────────────────────────────────────────────────────────

    def _get_pose(self) -> dict | None:
        with self._lock:
            return dict(self._current_pose) if self._current_pose else None

    def _get_localization_quality(self) -> int:
        """Get current localization quality from chassis (0-100)."""
        quality = self._slamtec.get_localization_quality()
        q = quality.get("raw", quality) if isinstance(quality, dict) else quality
        try:
            q = int(q)
        except (TypeError, ValueError):
            q = 0
        return q

    def _wait_for_localization(self, timeout: float = 30.0) -> bool:
        """Wait until SLAM reports localization with sufficient quality (>= 50).

        Returns True if quality >= 50, False if timeout but quality >= 30.
        Quality < 30 means the robot likely isn't at the expected position on the map.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._map_status == "localized":
                q = self._get_localization_quality()
                if q >= 50:
                    print(f"[ControlledSpatial] localization quality={q} ✓")
                    return True
                print(f"[ControlledSpatial] localization quality={q}, waiting...")
            time.sleep(1.0)
        # Final check
        q = self._get_localization_quality()
        print(f"[ControlledSpatial] localization quality final={q}")
        return q >= 30

    # ── Dispatch ─────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "running"}

        # ── Mapping ────────────────────────────────────────────────────────

        elif action == "start_mapping":
            map_name = args.get("map_name", "")
            if not map_name:
                return {"error": "map_name is required"}
            if self._is_mapping:
                return {"error": "Mapping already active"}
            result = self._slamtec.start_mapping()
            if result.get("error"):
                return {"error": f"Start mapping failed: {result.get('error')}", "api_result": result}

            map_path = f"{self._pcd_dir}/controlled_{map_name}.stcm"
            self._active_map = map_name
            self._is_mapping = True
            self._map_status = "mapping"
            self._db.add_map(map_name, map_path)
            return {"status": "mapping", "map_name": map_name, "map_path": map_path}

        elif action == "stop_mapping":
            if not self._active_map:
                return {"error": "No active mapping session"}
            map_name = self._active_map
            map_path = f"{self._pcd_dir}/controlled_{map_name}.stcm"

            # 1. Stop mapping mode on chassis
            result = self._slamtec.stop_mapping()
            if result.get("error"):
                return {"error": f"Stop mapping failed: {result.get('error')}", "api_result": result}

            # 2. Download map binary data from chassis and save to local file
            map_data, dl_error = self._slamtec.get_current_map()
            if dl_error or not map_data:
                self._is_mapping = False
                self._active_map = None
                self._map_status = "idle"
                return {
                    "status": "stopped_no_save",
                    "map_name": map_name,
                    "warning": f"Map data download failed: {dl_error}. Map only exists in chassis memory.",
                }

            try:
                with open(map_path, "wb") as f:
                    f.write(map_data)
            except OSError as e:
                self._is_mapping = False
                self._active_map = None
                self._map_status = "idle"
                return {
                    "status": "stopped_no_save",
                    "map_name": map_name,
                    "warning": f"Map file save failed: {e}. Map only exists in chassis memory.",
                }

            # 3. Update DB with actual file path
            self._db.add_map(map_name, map_path)

            self._is_mapping = False
            self._active_map = None
            self._map_status = "idle"
            return {"status": "stopped", "map_name": map_name, "map_path": map_path, "size_bytes": len(map_data)}

        # ── POI Tagging ────────────────────────────────────────────────────

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
            self._db.add_poi(name, pose["x"], pose["y"], pose.get("yaw", 0), active_map, desc)
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
                entry = {
                    "name": poi["name"],
                    "description": poi["description"],
                    "x": poi["x"],
                    "y": poi["y"],
                    "yaw": poi["yaw"],
                }
                if pose:
                    dx = poi["x"] - pose["x"]
                    dy = poi["y"] - pose["y"]
                    dist = math.sqrt(dx * dx + dy * dy)
                    cos_yaw = math.cos(-pose.get("yaw", 0))
                    sin_yaw = math.sin(-pose.get("yaw", 0))
                    rx = dx * cos_yaw - dy * sin_yaw
                    ry = dx * sin_yaw + dy * cos_yaw
                    entry["distance"] = round(dist, 2)
                    entry["bearing"] = _bearing_label(rx, ry)
                result.append(entry)
            return {"tags": result, "map": active_map}

        # ── Map Management ─────────────────────────────────────────────────

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
            map_path = map_info["pcd_path"]

            # 1. Clear existing map on chassis (required before upload to avoid 403 Forbidden)
            clear_result = self._slamtec.clear_map()
            if clear_result.get("error"):
                # Non-fatal: chassis may have no map to clear
                print(f"[ControlledSpatial] clear_map warning: {clear_result.get('error')}")

            # 2. Upload saved map file to chassis
            if not os.path.isfile(map_path):
                return {"error": f"Map file not found: {map_path}. The map data was not saved locally."}

            try:
                with open(map_path, "rb") as f:
                    map_data = f.read()
            except OSError as e:
                return {"error": f"Failed to read map file: {e}"}

            upload_result = self._slamtec.upload_map(map_data)
            if upload_result.get("error"):
                return {"error": f"Map upload failed: {upload_result.get('error')}", "api_result": upload_result}

            # 3. Set initial pose at origin
            result = self._slamtec.set_pose_init(0, 0, 0)
            if result.get("error"):
                return {"error": f"InitPose failed: {result.get('error')}", "api_result": result}

            # 4. Recover localization
            result = self._slamtec.recover_localization()
            if result.get("error"):
                # Non-fatal: some chassis don't require this
                print(f"[ControlledSpatial] recover_localization warning: {result.get('error')}")
            else:
                # Cancel the RecoverLocalizationAction after a short delay — we only need
                # it to trigger relocalization, not to wait for it to complete.
                # If left running, it will be the "current action" and block
                # subsequent navigation actions.
                time.sleep(1.0)
                self._slamtec.cancel_current_action()

            self._active_map = map_name
            self._map_status = "localizing"

            # Wait for localization to converge (up to 30s)
            if not self._wait_for_localization(timeout=30.0):
                q = self._get_localization_quality()
                if q < 30:
                    return {
                        "status": "loaded_poor",
                        "map_name": map_name,
                        "map_path": map_path,
                        "error": f"Localization quality too low ({q}/100). Robot may not be at the map origin, or the map may be outdated. Try: 1) move robot to map origin and reload, 2) rebuild the map.",
                    }
                return {
                    "status": "loaded",
                    "map_name": map_name,
                    "map_path": map_path,
                    "warning": f"Localization quality is marginal ({q}/100). Navigation may fail. Consider moving robot to map origin or rebuilding the map.",
                }
            self._map_status = "localized"
            return {"status": "loaded", "map_name": map_name, "map_path": map_path}

        # ── Navigation ─────────────────────────────────────────────────────

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

            # Check localization quality before navigating
            q = self._get_localization_quality()
            if q < 30:
                return {"error": f"Localization quality too low ({q}/100). Cannot navigate reliably. Try load_map again or rebuild the map."}
            if q < 50:
                print(f"[ControlledSpatial] WARNING: localization quality={q} is marginal, navigation may fail")
            yaw = poi.get("yaw", 0)
            speed = max(0.1, min(1.0, float(args.get("speed", 0.5))))
            mode = int(args.get("mode", 0))
            fail_retry_count = int(args.get("fail_retry_count", 3))
            acceptable_precision = float(args.get("acceptable_precision", 0.18))
            strategy = args.get("strategy", "default")
            ignore_dynamic = bool(args.get("ignore_dynamic_obstacles", True))
            precise = bool(args.get("precise", False))

            self._slamtec.set_motion_strategy(strategy)

            # Cancel any existing action (e.g. RecoverLocalizationAction from load_map)
            # before creating a new MoveToAction, to avoid monitoring the wrong action.
            cancel_result = self._slamtec.cancel_current_action()
            print(f"[ControlledSpatial] cancel before navigate: {cancel_result}")
            time.sleep(0.3)  # Wait for cancellation to take effect

            # mode: 0=free (default), 1=strict-track, 2=track-priority
            self._nav_arrived.clear()
            self._nav_error = None
            self._nav_active = True
            self._nav_start_time = time.monotonic()
            self._nav_lost_count = 0
            result = self._slamtec.move_to(poi["x"], poi["y"], yaw=yaw, speed_ratio=speed, mode=mode,
                                           fail_retry_count=fail_retry_count,
                                           acceptable_precision=acceptable_precision,
                                           ignore_dynamic_obstacles=ignore_dynamic,
                                           precise=precise)
            if result.get("error"):
                self._nav_active = False
                return {"error": f"NavigateTo failed: {result.get('error')}", "api_result": result}
            self._nav_action_id = result.get("action_id") or result.get("id")
            print(f"[ControlledSpatial] navigate_to_tag '{tag_name}': action_id={self._nav_action_id}, api_result={result}")

            # Immediately verify the action is actually running on the chassis
            time.sleep(0.2)
            verify = self._slamtec.get_nav_status()
            print(f"[ControlledSpatial] verify after move_to: {verify}")

            # If the action already disappeared from :current, query by ID to get the final result
            if isinstance(verify, dict) and verify.get("action_state") == -1:
                final = self._slamtec.get_action_status(str(self._nav_action_id))
                print(f"[ControlledSpatial] action {self._nav_action_id} already ended, query result: {final}")
                if isinstance(final, dict) and not final.get("error"):
                    fs = final.get("state") if isinstance(final.get("state"), dict) else None
                    if isinstance(fs, dict):
                        final_status = int(fs.get("status", -1))
                        final_result = int(fs.get("result", 0))
                        reason = fs.get("reason", "")
                        if final_status == 4:
                            if final_result == 0:
                                self._nav_arrived.set()
                                self._nav_active = False
                                return {"status": "arrived", "pose": self._get_pose()}
                            else:
                                label = "failed" if final_result == -1 else "aborted"
                                self._nav_active = False
                                return {"status": "error", "error": f"Action {label}: result={final_result}, reason={reason}"}

            return {
                "status": "navigating",
                "target": tag_name,
                "pose": {"x": poi["x"], "y": poi["y"], "yaw": yaw},
            }

        elif action == "navigate_to_pose":
            x = float(args.get("x", 0))
            y = float(args.get("y", 0))
            yaw = float(args.get("yaw", 0))

            # Check localization quality before navigating
            q = self._get_localization_quality()
            if q < 30:
                return {"error": f"Localization quality too low ({q}/100). Cannot navigate reliably. Try load_map again or rebuild the map."}
            if q < 50:
                print(f"[ControlledSpatial] WARNING: localization quality={q} is marginal, navigation may fail")
            speed = max(0.1, min(1.0, float(args.get("speed", 0.5))))
            mode = int(args.get("mode", 0))
            fail_retry_count = int(args.get("fail_retry_count", 3))
            acceptable_precision = float(args.get("acceptable_precision", 0.18))
            strategy = args.get("strategy", "default")
            ignore_dynamic = bool(args.get("ignore_dynamic_obstacles", True))
            precise = bool(args.get("precise", False))

            self._slamtec.set_motion_strategy(strategy)

            # Cancel any existing action before creating a new MoveToAction
            cancel_result = self._slamtec.cancel_current_action()
            print(f"[ControlledSpatial] cancel before navigate: {cancel_result}")
            time.sleep(0.3)

            self._nav_arrived.clear()
            self._nav_error = None
            self._nav_active = True
            self._nav_start_time = time.monotonic()
            self._nav_lost_count = 0
            result = self._slamtec.move_to(x, y, yaw=yaw, speed_ratio=speed, mode=mode,
                                           fail_retry_count=fail_retry_count, acceptable_precision=acceptable_precision,
                                           ignore_dynamic_obstacles=ignore_dynamic,
                                           precise=precise)
            if result.get("error"):
                self._nav_active = False
                return {"error": f"NavigateTo failed: {result.get('error')}", "api_result": result}
            self._nav_action_id = result.get("action_id") or result.get("id")
            print(f"[ControlledSpatial] navigate_to_pose ({x},{y}): action_id={self._nav_action_id}, api_result={result}")

            # Immediately verify the action is actually running on the chassis
            time.sleep(0.2)
            verify = self._slamtec.get_nav_status()
            print(f"[ControlledSpatial] verify after move_to: {verify}")

            # If the action already disappeared from :current, query by ID to get the final result
            if isinstance(verify, dict) and verify.get("action_state") == -1:
                final = self._slamtec.get_action_status(str(self._nav_action_id))
                print(f"[ControlledSpatial] action {self._nav_action_id} already ended, query result: {final}")
                if isinstance(final, dict) and not final.get("error"):
                    fs = final.get("state") if isinstance(final.get("state"), dict) else None
                    if isinstance(fs, dict):
                        final_status = int(fs.get("status", -1))
                        final_result = int(fs.get("result", 0))
                        reason = fs.get("reason", "")
                        if final_status == 4:
                            if final_result == 0:
                                self._nav_arrived.set()
                                self._nav_active = False
                                return {"status": "arrived", "pose": self._get_pose()}
                            else:
                                label = "failed" if final_result == -1 else "aborted"
                                self._nav_active = False
                                return {"status": "error", "error": f"Action {label}: result={final_result}, reason={reason}"}

            return {
                "status": "navigating",
                "target_pose": {"x": x, "y": y, "yaw": yaw},
            }

        elif action == "wait_navigation_done":
            stall_timeout = float(args.get("stall_timeout", 60))
            poll_interval = 0.5
            last_pose = self._get_pose()
            stall_start = time.monotonic()

            while True:
                if self._nav_arrived.is_set():
                    if self._nav_error:
                        error = self._nav_error
                        self._nav_error = None
                        self._nav_active = False
                        return {"status": "error", "error": error}
                    self._nav_active = False
                    return {"status": "arrived", "pose": self._get_pose()}

                # If nav is active but no action on chassis, query by ID
                if self._nav_active and self._nav_action_id is not None:
                    nav_status = self._slamtec.get_nav_status()
                    if isinstance(nav_status, dict) and nav_status.get("action_state") == -1:
                        final = self._slamtec.get_action_status(str(self._nav_action_id))
                        if isinstance(final, dict) and not final.get("error"):
                            fs = final.get("state") if isinstance(final.get("state"), dict) else None
                            if isinstance(fs, dict):
                                final_status = int(fs.get("status", -1))
                                final_result = int(fs.get("result", 0))
                                reason = fs.get("reason", "")
                                if final_status == 4:
                                    if final_result == 0:
                                        self._nav_active = False
                                        return {"status": "arrived", "pose": self._get_pose()}
                                    else:
                                        label = "failed" if final_result == -1 else "aborted"
                                        self._nav_active = False
                                        return {"status": "error", "error": f"Action {label}: result={final_result}, reason={reason}"}

                time.sleep(poll_interval)

                # Stall detection: no movement for stall_timeout seconds
                current_pose = self._get_pose()
                if current_pose and last_pose:
                    dx = current_pose["x"] - last_pose["x"]
                    dy = current_pose["y"] - last_pose["y"]
                    moved = math.sqrt(dx * dx + dy * dy)
                    dyaw = abs(current_pose.get("yaw", 0) - last_pose.get("yaw", 0))
                    # Normalize yaw difference to [0, pi]
                    if dyaw > math.pi:
                        dyaw = 2 * math.pi - dyaw
                    if moved > 0.05 or dyaw > 0.05:
                        stall_start = time.monotonic()
                        last_pose = current_pose

                if time.monotonic() - stall_start > stall_timeout:
                    self._slamtec.cancel_current_action()
                    self._nav_active = False
                    return {"status": "timeout", "error": f"No movement for {stall_timeout}s, navigation cancelled"}

        elif action == "pause_nav":
            result = self._slamtec.cancel_current_action()
            if result.get("error"):
                return {"error": f"PauseNav failed: {result.get('error')}", "api_result": result}
            return {"status": "paused"}

        elif action == "stop_nav":
            result = self._slamtec.cancel_current_action()
            self._nav_arrived.clear()
            self._nav_active = False
            self._nav_action_id = None
            return {"status": "stopped", "api_result": result}

        return None
