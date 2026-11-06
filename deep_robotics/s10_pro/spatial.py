"""Controlled S10 map operations through the robot-side drmap CLI."""

from __future__ import annotations

import json
import ipaddress
import fcntl
import math
import os
from pathlib import Path
import shlex
import shutil
import socket
import struct
import subprocess
import time

from common.vendor_runtime import action_schema, tool


def _safe_remote_value(value, depth=0):
    """Bound and escape data returned by a remote CLI before exposing it via MCP."""
    if depth >= 5:
        return "[truncated]"
    if isinstance(value, str):
        escaped = "".join(char if char >= " " and char != "\x7f" else f"\\x{ord(char):02x}"
                          for char in value)
        return escaped[:512]
    if isinstance(value, list):
        return [_safe_remote_value(item, depth + 1) for item in value[:100]]
    if isinstance(value, dict):
        return {str(_safe_remote_value(key, depth + 1))[:128]: _safe_remote_value(item, depth + 1)
                for key, item in list(value.items())[:100]}
    if value is None or isinstance(value, (bool, int, float)):
        return value
    return str(type(value).__name__)


class S10ControlledSpatialPlugin:
    PREFIX = "controlled_spatial"

    ACTIONS = {
        "info": ([], "Report auto-detected local or external execution path."),
        "map_list": ([], "List robot maps using drmap JSON output."),
        "map_disk": ([], "Check free space on the robot map filesystem."),
        "mapping_status": ([], "Read mapping/localization service status and current mapping path."),
        "map_activate": (["confirm", "map_name"],
                         "Activate a newly created map using the documented mapping-result workflow."),
        "map_apply": (["confirm", "map_name"],
                      "Switch to an existing map and restart localization."),
        "mapping_start": (["confirm", "map_name", "no_activate", "no_rviz"],
                          "Start body mapping; closes localization. Before confirming, verify the robot is stationary and standing in normal/agile/basic gait, sensors are online, the area is static and within 50x50 m, and the planned route forms a loop."),
        "mapping_stop": (["confirm"], "Stop the active mapping session."),
        # Compatibility names used by Tianyi/G1 controlled_spatial cards.
        "start_mapping": (["map_name", "confirm"], "Start mapping."),
        "stop_mapping": (["confirm"], "Stop mapping and save the map."),
        "list_maps": ([], "List saved maps."),
        "delete_map": (["map_name", "confirm"], "Delete a saved map."),
        "load_map": (["map_name", "confirm"], "Load an existing map."),
        "tag_place": (["name", "description"], "Save the current localized pose as a named tag."),
        "untag_place": (["name", "confirm"], "Remove a named tag."),
        "list_tags": ([], "List named tags on the current map."),
        "navigate_to_tag": (["tag_name", "speed", "mode", "confirm"], "Navigate to a named tag."),
        "navigate_to_pose": (["x", "y", "yaw", "speed", "mode", "confirm"], "Navigate to a map pose; yaw is radians, matching G1/Tianyi."),
        "stop_nav": (["confirm"], "Stop and cancel navigation."),
        "status": ([], "Read native navigation status."),
        "cancel": (["confirm"], "Cancel the active native navigation task."),
        "pose": ([], "Read the current localized map pose."),
        "perception_status": ([], "Read native localization and obstacle status."),
        "localize": (["x", "y", "z", "yaw_degrees", "confirm"], "Initialize localization at a map pose."),
        "goal": (["x", "y", "z", "yaw_degrees", "point_type", "gait", "speed", "reverse", "avoid_obstacles", "autonomous", "confirm"], "Navigate to one map-coordinate goal."),
        "route": (["waypoints", "gait", "speed", "reverse", "avoid_obstacles", "autonomous", "confirm"], "Navigate through a sequence of map-coordinate waypoints."),
    }
    CONFIRM_ACTIONS = {"map_activate", "map_apply", "mapping_start", "mapping_stop", "cancel", "localize", "goal", "route",
                       "start_mapping", "stop_mapping", "delete_map", "load_map", "untag_place", "navigate_to_tag"}
    CONFIRM_ACTIONS.update({"navigate_to_pose", "stop_nav"})
    NAVIGATION_ACTIONS = {"status", "cancel", "pose", "perception_status", "localize", "goal", "route"}

    def __init__(self, config, navigation=None):
        self.navigation = navigation
        spatial = config.get("controlled_spatial", {})
        self.requested_mode = os.environ.get("S10_DRMAP_MODE", spatial.get("mode", "auto"))
        self.control_host = str(os.environ.get("S10_BASIC_SERVER_HOST") or
                                config.get("basic_server", {}).get("host", "10.21.33.103"))
        self.host = str(os.environ.get("S10_DRMAP_SSH_HOST") or spatial.get("ssh_host") or "10.21.33.106")
        self.user = os.environ.get("S10_DRMAP_SSH_USER", spatial.get("ssh_user", "user"))
        self.password = os.environ.get("S10_DRMAP_PASSWORD", "")
        self.directory = os.environ.get("S10_DRMAP_DIRECTORY", spatial.get("directory", "/var/opt/robot/data/maps"))
        self.timeout = float(spatial.get("timeout", 30))
        self.tags_file = str(os.environ.get("S10_DRMAP_TAGS_FILE") or
                            spatial.get("tags_file", "/opt/phanthy-motus/data/s10_pro/tags.json"))
        self._tags = {}
        self._load_tags()
        if self.requested_mode not in {"auto", "local", "ssh"}:
            raise ValueError("controlled_spatial.mode must be auto, local, or ssh")
        try:
            self.host_address = ipaddress.ip_address(self.host)
        except ValueError as exc:
            raise ValueError("S10 drmap SSH host must be an IP address for deployment detection") from exc
        if not self.directory.startswith("/") or "\n" in self.directory:
            raise ValueError("S10_DRMAP_DIRECTORY must be an absolute path")
        self.local_target = self._is_local_address(self.host_address)
        try:
            self.control_local = self._is_local_address(ipaddress.ip_address(self.control_host))
        except ValueError as exc:
            raise ValueError("S10 basic_server host must be an IP address for deployment detection") from exc
        local_drmap = shutil.which("drmap") is not None and os.path.isdir(self.directory)
        if self.requested_mode == "local" and not local_drmap:
            raise ValueError("local drmap mode requested but drmap or map directory is missing")
        if self.requested_mode == "local" and not self.local_target:
            raise ValueError("refusing local drmap execution: S10 NOS management host is not local")
        # A NOS address may be assigned locally while the driver image lacks
        # the vendor CLI/data mount. In auto mode use the documented SSH path
        # in that case; explicit local mode remains fail-closed above.
        self.mode = ("local" if self.local_target and local_drmap else "ssh") \
            if self.requested_mode == "auto" else self.requested_mode
        if self.mode == "ssh" and shutil.which("ssh") is None:
            raise ValueError("ssh client is required for robot-side drmap execution")

    @staticmethod
    def _is_local_address(address):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            for _, name in socket.if_nameindex():
                request = struct.pack("256s", name.encode()[:15])
                try:
                    raw = fcntl.ioctl(sock.fileno(), 0x8915, request)
                except OSError:
                    continue
                if ipaddress.ip_address(socket.inet_ntoa(raw[20:24])) == address:
                    return True
        return address.is_loopback

    def get_tool(self):
        result = tool(self.PREFIX, "actuator",
                      "山猫 S10 本体建图与地图管理。建图建议限于 50m×50m；开始前需确认机器人静止站立并处于常规模式-敏捷-基础步态、雷达无遮挡、IMU/雷达在线、环境静态且路线规划为闭环。激活/切换地图会重启定位；启动/停止建图也会切换 mapping/localization 服务，均需显式 confirm=true。",
                      action_schema(self.ACTIONS, {
                          "confirm": {"type": "boolean", "description": "Required true for map activation/application and mapping start/stop. For mapping start, confirms the documented robot posture/mode, sensor, environment, area-size, and loop-route preconditions."},
                          "map_name": {"type": "string", "description": "Optional map name; drmap appends a timestamp."},
                          "no_activate": {"type": "boolean", "description": "Create the map without activating it for localization/navigation."},
                          "no_rviz": {"type": "boolean", "description": "Disable RViz when running headless or remotely."},
                          "x": {"type": "number"}, "y": {"type": "number"},
                          "z": {"type": "number", "default": 0},
                          "yaw_degrees": {"type": "number", "description": "Map-frame target yaw in degrees."},
                          "point_type": {"type": "integer", "enum": [0, 1, 2, 3, 4], "default": 1},
                          "gait": {"type": "string", "enum": ["navigation_flat", "navigation_stairs"], "default": "navigation_flat"},
                          "speed": {"type": "number", "minimum": 0.2, "maximum": 0.8, "default": 0.5},
                          "reverse": {"type": "boolean", "default": False},
                          "avoid_obstacles": {"type": "boolean", "default": True},
                          "autonomous": {"type": "boolean", "default": False},
                          "waypoints": {"type": "array", "minItems": 1, "maxItems": 64,
                                         "items": {"type": "object"}},
                          "name": {"type": "string"}, "description": {"type": "string"},
                          "tag_name": {"type": "string"}, "mode": {"type": "integer", "enum": [0, 1]},
                          "fail_retry_count": {"type": "integer", "default": 3},
                          "acceptable_precision": {"type": "number", "default": 0.18},
                          "strategy": {"type": "string", "default": "default"},
                          "ignore_dynamic_obstacles": {"type": "boolean", "default": True},
                          "precise": {"type": "boolean", "default": False},
                          "stall_timeout": {"type": "number", "default": 90},
                      }))
        result["inputSchema"]["x-is-dangerous"] = True
        result["inputSchema"]["x-resource"] = "map"
        if self.navigation is not None:
            result["inputSchema"]["x-resource"] = ["base", "map"]
            result["inputSchema"]["x-completion"] = {"actions": ["goal", "route"], "timeout": 190}
            result["inputSchema"]["x-hooks"] = {"on_interrupt_motion": {"action": "cancel"}}
        return result

    def start(self):
        pass

    def stop(self):
        pass

    def _load_tags(self):
        try:
            with open(self.tags_file, encoding="utf-8") as handle:
                data = json.load(handle)
            if isinstance(data, dict):
                if isinstance(data.get("maps"), dict):
                    self._tags = data
                else:
                    self._tags = {"active_map": "active", "maps": {"active": data}}
        except (OSError, ValueError, TypeError):
            self._tags = {"active_map": "active", "maps": {}}

    def _save_tags(self):
        path = Path(self.tags_file)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            temp = path.with_suffix(path.suffix + ".tmp")
            temp.write_text(json.dumps(self._tags, ensure_ascii=False, sort_keys=True), encoding="utf-8")
            temp.replace(path)
        except OSError:
            # Tagging must not make the robot move if persistence is unavailable.
            return False
        return True

    @staticmethod
    def _pose_from_response(response):
        items = response.get("PatrolDevice", {}).get("Items", {}) if isinstance(response, dict) else {}
        try:
            return {"x": float(items["PosX"]), "y": float(items["PosY"]),
                    "z": float(items.get("PosZ", 0)),
                    "yaw_degrees": float(items.get("YawDegrees", 0))}
        except (KeyError, TypeError, ValueError):
            return None

    def _tag_map(self):
        return str(self._tags.get("active_map") or "active")

    def _run_command(self, command, *, parse_json=False):
        env = None
        if self.mode == "local":
            argv = ["/bin/sh", "-lc", command]
        else:
            target = f"{self.user}@{self.host}" if self.user else self.host
            known_hosts = os.environ.get(
                "S10_SSH_KNOWN_HOSTS",
                str(Path(__file__).with_name("resource") / "ssh_known_hosts"),
            )
            ssh_args = ["ssh", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=yes",
                        "-o", f"UserKnownHostsFile={known_hosts}", "-o", "GlobalKnownHostsFile=/dev/null",
                        "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2", "--", target, command]
            if self.password:
                argv = ["sshpass", "-e", *ssh_args]
                env = os.environ.copy()
                env["SSHPASS"] = self.password
            else:
                argv = [ssh_args[0], "-o", "BatchMode=yes", *ssh_args[1:]]
        try:
            result = subprocess.run(argv, capture_output=True, text=True, timeout=self.timeout,
                                    check=False, env=env)
        except (OSError, subprocess.TimeoutExpired) as exc:
            return {"ok": False, "code": "EXECUTION_FAILED", "error": str(exc), "mode": self.mode}
        payload = {"ok": result.returncode == 0, "mode": self.mode,
                   "exit_code": result.returncode,
                   "stdout": _safe_remote_value(result.stdout[-12000:]),
                   "stderr": _safe_remote_value(result.stderr[-4000:])}
        if result.returncode:
            payload["code"] = "DRMAP_FAILED"
        if parse_json and result.returncode == 0:
            try:
                payload["data"] = _safe_remote_value(json.loads(result.stdout))
            except json.JSONDecodeError:
                payload["code"] = "INVALID_JSON_OUTPUT"
        return payload

    def _run_drmap(self, args):
        command = "cd -- {} && exec drmap {}".format(
            shlex.quote(self.directory), " ".join(shlex.quote(part) for part in args))
        return self._run_command(command, parse_json=args[:2] == ["--format", "json"])

    def _mapping_status(self):
        # All commands are fixed and read-only; do not accept shell fragments from MCP.
        service_command = (
            "printf 'mapping.service='; systemctl is-active mapping.service 2>/dev/null || true; "
            "printf 'localization.service='; systemctl is-active localization.service 2>/dev/null || true; "
            "printf 'mapping_path.env='; cat /tmp/mapping_path.env 2>/dev/null || true"
        )
        status = self._run_drmap(["mapping", "status"])
        services = self._run_command(service_command)
        return {"ok": bool(status.get("ok") and services.get("ok")),
                "mode": self.mode, "mapping": status, "services": services}

    def dispatch(self, action, args):
        aliases = {"start_mapping": "mapping_start", "stop_mapping": "mapping_stop",
                   "list_maps": "map_list", "load_map": "map_apply", "delete_map": "map_delete",
                   "stop_nav": "cancel"}
        if action == "navigate_to_pose":
            args = dict(args)
            try:
                args["yaw_degrees"] = math.degrees(float(args.get("yaw", 0)))
            except (TypeError, ValueError):
                return {"ok": False, "code": "INVALID_ARGUMENT", "error": "yaw must be radians"}
            speed = args.get("speed", 0.5)
            if isinstance(speed, (int, float)) and not isinstance(speed, bool):
                args["speed"] = 0 if speed < 0.4 else 1 if speed < 0.7 else 2
            args["avoid_obstacles"] = int(args.get("mode", 0)) != 1
            action = "goal"
        if action in aliases:
            action = aliases[action]
        if action == "start":
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            return {"state": "ready", "mode": self.mode,
                    "deployment": "aos" if self.control_local else "external",
                    "management_host": self.host,
                    "ssh_host": self.host if self.mode == "ssh" else None,
                    "directory": self.directory}
        if action == "map_delete":
            if args.get("confirm") is not True:
                return {"ok": False, "code": "INVALID_ARGUMENT", "error": "delete_map requires confirm=true"}
            name = args.get("map_name")
            if not isinstance(name, str) or not name.strip():
                return {"ok": False, "code": "INVALID_ARGUMENT", "error": "map_name is required"}
            return self._run_drmap(["map", "delete", name.strip()])
        if action == "tag_place":
            if self.navigation is None:
                return {"ok": False, "code": "UNAVAILABLE", "error": "localization is unavailable"}
            pose = self._pose_from_response(self.navigation.dispatch("pose", {}))
            name = str(args.get("name") or "").strip()
            if not name or len(name) > 64:
                return {"ok": False, "code": "INVALID_ARGUMENT", "error": "name must be 1-64 characters"}
            if pose is None:
                return {"ok": False, "code": "POSE_UNAVAILABLE", "error": "current localized pose is unavailable"}
            self._tags.setdefault("maps", {}).setdefault(self._tag_map(), {})[name] = {
                **pose, "description": str(args.get("description") or ""), "updated_at": time.time()}
            if not self._save_tags():
                return {"ok": False, "code": "PERSISTENCE_FAILED", "error": "tag storage is not writable"}
            return {"ok": True, "state": "tagged", "tag": {"name": name, **self._tags["maps"][self._tag_map()][name]}}
        if action == "list_tags":
            tags = self._tags.get("maps", {}).get(self._tag_map(), {})
            return {"ok": True, "map": self._tag_map(), "tags": [
                {"name": name, **tag} for name, tag in sorted(tags.items())]}
        if action == "untag_place":
            name = str(args.get("name") or "").strip()
            tags = self._tags.get("maps", {}).get(self._tag_map(), {})
            if not name or name not in tags:
                return {"ok": False, "code": "NOT_FOUND", "error": "tag not found"}
            if args.get("confirm") is not True:
                return {"ok": False, "code": "INVALID_ARGUMENT", "error": "untag_place requires confirm=true"}
            del tags[name]
            if not self._save_tags():
                return {"ok": False, "code": "PERSISTENCE_FAILED", "error": "tag storage is not writable"}
            return {"ok": True, "state": "untagged", "name": name}
        if action == "navigate_to_tag":
            name = str(args.get("tag_name") or "").strip()
            tag = self._tags.get("maps", {}).get(self._tag_map(), {}).get(name)
            if tag is None:
                return {"ok": False, "code": "NOT_FOUND", "error": "tag not found", "tags": list(self._tags.get("maps", {}).get(self._tag_map(), {}))}
            if self.navigation is None:
                return {"ok": False, "code": "UNAVAILABLE", "error": "native navigation is unavailable"}
            nav_args = {"x": tag["x"], "y": tag["y"], "z": tag.get("z", 0),
                        "yaw_degrees": tag.get("yaw_degrees", 0), "confirm": args.get("confirm") is True}
            if "gait" in args: nav_args["gait"] = args["gait"]
            if "speed" in args:
                speed = args["speed"]
                if isinstance(speed, (int, float)) and not isinstance(speed, bool):
                    nav_args["speed"] = 0 if speed < 0.4 else 1 if speed < 0.7 else 2
            if "reverse" in args: nav_args["reverse"] = args["reverse"]
            nav_args["avoid_obstacles"] = int(args.get("mode", 0)) != 1
            if "avoid_obstacles" in args: nav_args["avoid_obstacles"] = args["avoid_obstacles"]
            if "autonomous" in args: nav_args["autonomous"] = args["autonomous"]
            return self.navigation.dispatch("goal", nav_args)
        if action in self.NAVIGATION_ACTIONS:
            if self.navigation is None:
                return {"ok": False, "code": "UNAVAILABLE", "error": "native navigation is unavailable"}
            return self.navigation.dispatch(action, args)
        if action == "map_list":
            return self._run_drmap(["--format", "json", "map", "list"])
        if action == "map_disk":
            return self._run_drmap(["map", "disk"])
        if action == "mapping_status":
            return self._mapping_status()
        if action in self.CONFIRM_ACTIONS and args.get("confirm") is not True:
            return {"ok": False, "code": "INVALID_ARGUMENT", "error": f"{action} requires confirm=true"}
        if action in {"map_activate", "map_apply"}:
            map_name = args.get("map_name")
            if not isinstance(map_name, str) or not map_name.strip() or len(map_name) > 64 or any(
                    ord(char) < 32 for char in map_name):
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "error": "map_name must be 1-64 printable characters"}
            if action == "map_activate":
                result = self._run_drmap(["map", "activate", map_name.strip()])
                if result.get("ok"):
                    self._tags["active_map"] = map_name.strip()
                    self._save_tags()
                return result
            command = "cd -- {} && drmap map apply {} && systemctl restart localization.service".format(
                shlex.quote(self.directory), shlex.quote(map_name.strip()))
            result = self._run_command(command)
            if result.get("ok"):
                self._tags["active_map"] = map_name.strip()
                self._save_tags()
            return result
        if action == "mapping_start":
            map_name = args.get("map_name")
            if map_name is not None and (not isinstance(map_name, str) or
                                         not map_name.strip() or len(map_name) > 64 or
                                         any(ord(char) < 32 for char in map_name)):
                return {"ok": False, "code": "INVALID_ARGUMENT",
                        "error": "map_name must be 1-64 printable characters"}
            command = ["mapping", "start"]
            if map_name:
                command.append(map_name.strip())
            if args.get("no_activate") is True:
                command.append("--no-activate")
            if args.get("no_rviz") is True:
                command.append("--no-rviz")
            result = self._run_drmap(command)
            if result.get("ok") and map_name:
                self._tags["active_map"] = map_name.strip()
                self._save_tags()
            return result
        if action == "mapping_stop":
            return self._run_drmap(["mapping", "stop"])
        return None
