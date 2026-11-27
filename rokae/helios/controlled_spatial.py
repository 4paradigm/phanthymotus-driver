"""Map-scoped named poses and Helios canvas actions."""

import asyncio
import math
import os
from pathlib import Path
import re
import sqlite3
import ssl
import sys
import time
from urllib.parse import urlsplit

import aiohttp

from common.vendor_runtime import tool

PARAMS = {"info": [], "start": [], "stop": [], "status": [], "list_maps": [],
          "load_map": ["map_name", "confirm"], "start_mapping": ["map_name", "confirm"], "stop_mapping": ["confirm"],
          "tag_place": ["name", "description"], "list_tags": [], "untag_place": ["name"],
          "navigate_to_tag": ["tag_name", "confirm"], "navigate_to_pose": ["x", "y", "yaw", "confirm"], "stop_nav": []}


def definition(topic):
    props = {k: {"type": "string"} for k in ("map_name", "name", "description", "tag_name")}
    props.update({k: {"type": "number"} for k in ("x", "y", "yaw")})
    props["confirm"] = {"type": "boolean", "enum": [False, True], "default": False,
                        "description": "确认本次建图、切换地图或底盘导航操作"}
    props["action"] = {"type": "string", "enum": list(PARAMS)}
    schema = {"type": "object", "properties": props, "required": ["action"],
              "x-action-params": {k: {"params": p, "required": [i for i in p if i != "description"]}
                                  for k, p in PARAMS.items()},
              "x-completion": {"actions": ["navigate_to_tag", "navigate_to_pose"], "timeout": 300},
              "x-is-dangerous": True, "x-resource": "base", "x-hooks": {"on_interrupt_motion": {"action": "stop_nav"},
                                                "on_interrupt_all": {"action": "stop_nav"}}}
    return tool("controlled_spatial", "actuator", "手动建图、加载地图、记录当前位置，按标签或地图坐标导航。"
                "status 查询导航结果，stop_nav 取消当前导航。", schema, topic_out=[topic])


class NavigationCompletion:
    """Authenticated Core callback for navigation accepted by the VMR worker."""

    def __init__(self):
        endpoint = os.getenv("AGENT_CORE_URL", "https://127.0.0.1:15678").rstrip("/")
        core = urlsplit(endpoint)
        if core.scheme != "https" or not core.hostname:
            raise ValueError("Helios navigation completion requires an HTTPS AGENT_CORE_URL")
        self.url = endpoint + "/api/acp/complete"
        # Core's self-signed certificate names phanthy-motus on the loopback transport.
        self.server_hostname = ("phanthy-motus" if core.hostname in ("localhost", "127.0.0.1", "::1")
                                else core.hostname)
        cert = os.getenv("AGENT_CORE_CA_CERT")
        if not cert:
            raise ValueError("AGENT_CORE_CA_CERT is required for Helios navigation completion")
        self.context = ssl.create_default_context(cafile=cert)

    async def send(self, result):
        payload = {"action_id": result["action_id"], "status": result["state"],
                   "result": result, "tool": "controlled_spatial", "ts": time.time()}
        async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=3)) as client:
            for attempt in range(3):
                try:
                    async with client.post(self.url, json=payload, ssl=self.context,
                                           server_hostname=self.server_hostname, allow_redirects=False) as response:
                        if 200 <= response.status < 300:
                            return
                        error = f"HTTP {response.status}"
                except (aiohttp.ClientError, asyncio.TimeoutError) as exc:
                    error = str(exc)
                if attempt < 2:
                    await asyncio.sleep(0.5)
        print(f"[helios] ACP completion {result['action_id']} failed: {error}", file=sys.stderr)


class ControlledSpatial:
    def __init__(self, path):
        self.path = Path(path)
        self.db = None
        self.building = None
        self.selected = None

    def open(self):
        if self.db is None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.db = sqlite3.connect(self.path, check_same_thread=False)
            self.db.row_factory = sqlite3.Row
            with self.db:
                self.db.execute("CREATE TABLE IF NOT EXISTS tags (map TEXT, name TEXT, description TEXT, "
                                "x REAL, y REAL, yaw REAL, PRIMARY KEY(map, name))")
                self.db.execute("CREATE TABLE IF NOT EXISTS mapping_session (name TEXT PRIMARY KEY)")
            row = self.db.execute("SELECT name FROM mapping_session").fetchone()
            self.building = row["name"] if row else None

    def close(self):
        if self.db:
            self.db.close()
            self.db = None

    def tags(self, name):
        if self.db is None or not name:
            return []
        return [dict(row) for row in self.db.execute(
            "SELECT name, description, x, y, yaw FROM tags WHERE map=? ORDER BY name", (name,))]

    def execute(self, action, args, sensor):
        if "confirm" in PARAMS.get(action, ()) and args.get("confirm") is not True:
            raise ValueError(f"{action} requires confirm=true")
        current = sensor.payload()
        active = current["map_name"]
        if action == "list_maps":
            return sensor.command(action)
        if action == "status":
            return {"active_map": active, "display_map": self.selected or active, "building": self.building,
                    "cloud_error": (sensor.selected_cloud if self.selected and self.selected != active else sensor.cloud).error, "navigation": sensor.command("nav_status")}
        if action == "stop_nav":
            return sensor.command(action)
        state = sensor.chassis.snapshot()
        chassis = state["chassis"] if state["available"] else {}
        mapping = self.building or chassis.get("mapping")
        if action in ("start_mapping", "load_map"):
            name = args.get("map_name")
            if not isinstance(name, str) or not name.strip():
                raise ValueError("map_name is required")
            if mapping:
                raise ValueError("Finish the current mapping session first")
            if action == "start_mapping" and not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", name):
                raise ValueError("Use letters, digits or underscores; the map name cannot start with a digit")
            result = sensor.command(action, map_name=name)
            with sensor._lock:
                self.selected = name
                sensor.cloud.clear()
                sensor.selected_cloud.clear()
                sensor.voxels.clear()
                if action == "start_mapping":
                    self.building = name
                    with self.db:
                        self.db.execute("INSERT INTO mapping_session VALUES (?)", (name,))
            return result
        if action == "stop_mapping":
            if not mapping:
                raise ValueError("No mapping session is active")
            result = sensor.command(action, map_name=self.building or active)
            with sensor._lock:
                self.selected = self.building or active
                self.building = None
                with self.db:
                    self.db.execute("DELETE FROM mapping_session")
                sensor.cloud.clear()
                sensor.selected_cloud.clear()
                sensor.voxels.clear()
            return result
        if action not in ("tag_place", "untag_place", "list_tags", "navigate_to_tag", "navigate_to_pose"):
            raise ValueError(f"Unsupported controlled_spatial action: {action}")
        name = self.selected or active
        if not name:
            raise ValueError("No map selected")
        if action in ("list_tags", "untag_place"):
            if action == "untag_place":
                with self.db:
                    if not self.db.execute("DELETE FROM tags WHERE map=? AND name=?", (name, args.get("name"))).rowcount:
                        raise ValueError("Unknown tag on this map")
            return {"map_name": name, "tags": self.tags(name)}
        pose = current["localization"]
        if mapping or active != name or not pose or not chassis.get("located"):
            raise ValueError("Load this map and wait for fresh chassis localization first")
        if action == "tag_place":
            label = args.get("name")
            if not isinstance(label, str) or not label.strip():
                raise ValueError("Tag name is required")
            with self.db:
                self.db.execute("INSERT OR REPLACE INTO tags VALUES (?,?,?,?,?,?)",
                                (name, label, args.get("description", ""), pose["x"], pose["y"], pose["theta"]))
            return {"map_name": name, "tags": self.tags(name)}
        if action == "navigate_to_tag":
            target = next((t for t in self.tags(name) if t["name"] == args.get("tag_name")), None)
            if target is None:
                raise ValueError("Unknown tag on this map")
        else:
            target = args
        coords = {k: float(target[k]) for k in ("x", "y", "yaw")}
        if not all(map(math.isfinite, coords.values())):
            raise ValueError("Target coordinates must be finite")
        return sensor.command("navigate", map_name=name, **coords)
