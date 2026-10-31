"""Controlled S10 map operations through the robot-side drmap CLI."""

from __future__ import annotations

import json
import ipaddress
import fcntl
import os
import shlex
import shutil
import socket
import struct
import subprocess

from common.vendor_runtime import action_schema, tool


class S10ControlledSpatialPlugin:
    ACTIONS = {
        "info": ([], "Report auto-detected local or external execution path."),
        "map_list": ([], "List robot maps using drmap JSON output."),
        "mapping_start": (["confirm"], "Start mapping on the robot; closes localization."),
        "mapping_stop": (["confirm"], "Stop the active mapping session."),
    }
    CONFIRM_ACTIONS = {"mapping_start", "mapping_stop"}

    def __init__(self, config):
        spatial = config.get("controlled_spatial", {})
        self.requested_mode = os.environ.get("S10_DRMAP_MODE", spatial.get("mode", "auto"))
        self.host = str(os.environ.get("S10_DRMAP_SSH_HOST") or spatial.get("ssh_host") or
                        os.environ.get("S10_BASIC_SERVER_HOST") or
                        config.get("basic_server", {}).get("host", "10.21.33.103"))
        self.user = os.environ.get("S10_DRMAP_SSH_USER", spatial.get("ssh_user", "user"))
        self.password = os.environ.get("S10_DRMAP_PASSWORD", "")
        self.directory = os.environ.get("S10_DRMAP_DIRECTORY", spatial.get("directory", "/var/opt/robot/data/maps"))
        self.timeout = float(spatial.get("timeout", 30))
        if self.requested_mode not in {"auto", "local", "ssh"}:
            raise ValueError("controlled_spatial.mode must be auto, local, or ssh")
        try:
            self.host_address = ipaddress.ip_address(self.host)
        except ValueError as exc:
            raise ValueError("S10 basic_server host must be an IP address for deployment detection") from exc
        if not self.directory.startswith("/") or "\n" in self.directory:
            raise ValueError("S10_DRMAP_DIRECTORY must be an absolute path")
        self.local_target = self._is_local_address(self.host_address)
        local_drmap = shutil.which("drmap") is not None and os.path.isdir(self.directory)
        if self.requested_mode == "local" and not local_drmap:
            raise ValueError("local drmap mode requested but drmap or map directory is missing")
        if self.requested_mode == "local" and not self.local_target:
            raise ValueError("refusing local drmap execution: S10 basic_server is not a local address")
        if self.requested_mode == "auto" and self.local_target and not local_drmap:
            raise ValueError("S10 address is local but drmap or its map directory is missing")
        self.mode = ("local" if self.local_target else "ssh") if self.requested_mode == "auto" else self.requested_mode
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
        result = tool("controlled_spatial", "actuator",
                      "山猫 S10 地图管理。地图列表为只读；启动/停止建图会影响定位和地图数据，需要 confirm=true。",
                      action_schema(self.ACTIONS, {
                          "confirm": {"type": "boolean", "description": "Required true for mapping start/stop."},
                      }))
        result["inputSchema"]["x-is-dangerous"] = True
        return result

    def start(self):
        pass

    def stop(self):
        pass

    def _run_drmap(self, args):
        command = "cd -- {} && exec drmap {}".format(
            shlex.quote(self.directory), " ".join(shlex.quote(part) for part in args))
        env = None
        if self.mode == "local":
            argv = ["/bin/sh", "-lc", command]
        else:
            target = f"{self.user}@{self.host}" if self.user else self.host
            ssh_args = ["ssh", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=accept-new",
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
                   "exit_code": result.returncode, "stdout": result.stdout[-12000:],
                   "stderr": result.stderr[-4000:]}
        if result.returncode:
            payload["code"] = "DRMAP_FAILED"
        if args[:2] == ["--format", "json"] and result.returncode == 0:
            try:
                payload["data"] = json.loads(result.stdout)
            except json.JSONDecodeError:
                payload["code"] = "INVALID_JSON_OUTPUT"
        return payload

    def dispatch(self, action, args):
        if action == "info":
            return {"state": "ready", "mode": self.mode,
                    "deployment": "aos" if self.local_target else "external",
                    "ssh_host": self.host if self.mode == "ssh" else None,
                    "directory": self.directory}
        if action == "map_list":
            return self._run_drmap(["--format", "json", "map", "list"])
        if action in self.CONFIRM_ACTIONS and args.get("confirm") is not True:
            return {"ok": False, "code": "INVALID_ARGUMENT", "error": f"{action} requires confirm=true"}
        if action == "mapping_start":
            return self._run_drmap(["mapping", "start"])
        if action == "mapping_stop":
            return self._run_drmap(["mapping", "stop"])
        raise ValueError(f"unsupported S10 controlled_spatial action: {action}")
