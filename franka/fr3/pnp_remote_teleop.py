"""Canvas interface for the dedicated arm service on the PNP workstation."""
from concurrent.futures import ThreadPoolExecutor
import ipaddress
import json
import re
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid

from common.vendor_runtime import action_schema, tool


class PnpRemoteTeleopPlugin:
    PREFIX = "teleop"

    def __init__(self, acp_callback=None):
        self._lock = threading.RLock()
        self._loaded = False
        self._init = {"state": "not_initialized"}
        self._init_thread = None
        self._password = ""
        self._acp_callback = acp_callback or self._post_completion
        self._acp = None
        self._align_cancel = threading.Event()
        self._align_thread = None
        self._settings = {"pnp_host": "", "ssh_user": "pnp", "right_slave_ip": "172.16.0.2",
                          "left_slave_ip": "172.16.0.3", "read_vendor_slave_ips": True}
        self._unit = "pnp-fr3-right-teleop.service"
        self._hosts = {"right": os.environ.get("FRANKA_PNP_SSH_HOST", "pnp@172.16.0.200")}
        left = os.environ.get("FRANKA_PNP_LEFT_SSH_HOST", "").strip()
        if left:
            if left == self._hosts["right"]:
                raise ValueError("左右两台 PNP 主控机需要不同的 SSH 地址")
            self._hosts["left"] = left

    def get_tools(self):
        schema = action_schema({
            "Align": ([], "Run vendor startup, align slave arms, and complete ACP while holding the aligned pose."),
            "Follow": ([], "Enable continuous vendor following after alignment has completed."),
            "Stop": ([], "Stop PNP teleoperation in reverse vendor startup order and verify shutdown."),
            "Status": ([], "Read PNP service states and vendor execution stages."),
        }, {})
        schema.update({
            "x-completion": {"actions": ["Align"], "timeout": 150},
            "x-resource": "arm",
            "x-is-dangerous": True,
            "x-hooks": {
                "on_interrupt_motion": {"action": "Stop"},
                "on_interrupt_all": {"action": "Stop"},
            },
        })
        definition = tool("teleop", "actuator", "PNP master teleoperation", schema)
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "pnp_host": {"type": "string", "title": "PNP host IP (optional)", "default": "",
                             "description": "Leave empty to discover a wired PNP host with both master USB devices."},
                "right_slave_ip": {"type": "string", "title": "Right FR3 slave IP", "default": self._settings["right_slave_ip"],
                                   "description": "Current right slave IP. Disable reading vendor IPs to use this value as a manual override."},
                "left_slave_ip": {"type": "string", "title": "Left FR3 slave IP", "default": self._settings["left_slave_ip"],
                                  "description": "Current left slave IP. Disable reading vendor IPs to use this value as a manual override."},
                "read_vendor_slave_ips": {"type": "boolean", "title": "Read slave IPs from PNP",
                                          "default": True,
                                          "description": "Read vendor left/right assignments during initialization. Keep current values if unavailable."},
                "ssh_user": {"type": "string", "title": "SSH username", "default": "pnp"},
                "ssh_password": {"type": "string", "title": "SSH initialization password",
                                 "format": "password", "x-sensitive": True, "writeOnly": True,
                                 "description": "Used for installation or updates; removed from driver memory after initialization."},

            },
        }
        return [definition]

    def start(self):
        pass

    def stop(self):
        result = self.dispatch("stop", {})
        if result.get("state") == "error":
            raise RuntimeError(result["message"])

    def _configure(self, args):
        keys = set(self._settings)
        supplied = {key: args[key] for key in keys if key in args}
        settings = {**self._settings, **supplied}
        if settings["pnp_host"]:
            ipaddress.IPv4Address(settings["pnp_host"])
        for key in ("right_slave_ip", "left_slave_ip"):
            ipaddress.IPv4Address(settings[key])
        if settings["right_slave_ip"] == settings["left_slave_ip"]:
            raise ValueError("Left and right slave IP addresses must be different")
        if not re.fullmatch(r"[a-z_][a-z0-9_-]*", settings["ssh_user"]):
            raise ValueError("Invalid SSH user")
        if type(settings["read_vendor_slave_ips"]) is not bool:
            raise ValueError("read_vendor_slave_ips must be boolean")
        changed = settings != self._settings or "ssh_password" in args
        if changed and (self._loaded or self._init.get("state") == "initializing" or
                        (self._align_thread and self._align_thread.is_alive())):
            raise RuntimeError("先停止卡片，再修改初始化配置")
        self._settings = settings
        if settings["pnp_host"]:
            self._hosts["right"] = settings["ssh_user"] + "@" + settings["pnp_host"]
        if "ssh_password" in args:
            self._password = str(args["ssh_password"])
        if changed:
            self._init = {"state": "not_initialized"}

    def _initialize(self):
        if self._init_thread and self._init_thread.is_alive():
            return
        from franka.fr3.pnp_initialize import initialize
        self._init = {"state": "initializing", "stage": "starting"}
        password, self._password = self._password, ""
        settings = dict(self._settings)
        host = self._hosts["right"]

        def progress(stage):
            with self._lock:
                self._init = {"state": "initializing", "stage": stage}

        def work():
            try:
                slave_ips = {"right": settings["right_slave_ip"], "left": settings["left_slave_ip"]}
                selected_host = host
                if not settings["pnp_host"]:
                    from franka.fr3.pnp_discovery import discover
                    address = discover(slave_ips, settings["ssh_user"], password, progress)
                    selected_host = settings["ssh_user"] + "@" + address
                result = initialize(selected_host, password, progress, slave_ips=slave_ips,
                                    read_vendor=settings["read_vendor_slave_ips"])
                with self._lock:
                    self._hosts["right"] = selected_host
                    for side, address in result.get("slave_ips", {}).items():
                        self._settings[f"{side}_slave_ip"] = address
                result["pnp_host"] = selected_host.split("@", 1)[1]
            except Exception as exc:
                message = str(exc).replace(password, "[redacted]") if password else str(exc)
                result = {"state": "error", "message": message}
            with self._lock:
                self._init = result

        self._init_thread = threading.Thread(target=work, name="pnp-initialize", daemon=True)
        self._init_thread.start()

    def _ssh(self, side, *args):
        host = self._hosts[side]
        from franka.fr3.pnp_initialize import paths
        key, known = paths()
        if not key.exists():
            key, known = Path("/run/fr3-pnp-ssh/id_ed25519"), Path("/run/fr3-pnp-ssh/known_hosts")
        result = subprocess.run(
            ["ssh", "-i", str(key), "-o", "IdentitiesOnly=yes",
             "-o", f"UserKnownHostsFile={known}",
             "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
             "-o", "ConnectTimeout=4", "-o", "ServerAliveInterval=2",
             "-o", "ServerAliveCountMax=2", "--", host, *args],
            capture_output=True, text=True, timeout=15,
        )
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or f"SSH exit {result.returncode}")
        return result.stdout

    def _host_status(self, side):
        output = self._ssh(side, "systemctl", "--user", "show", self._unit,
                           "-p", "LoadState", "-p", "ActiveState",
                           "-p", "SubState", "-p", "Result")
        fields = dict(line.split("=", 1) for line in output.splitlines() if "=" in line)
        if fields.get("LoadState") != "loaded":
            raise RuntimeError(f"PNP 主控机缺少专用服务 {self._unit}")
        active = fields.get("ActiveState")
        states = {"active": "aligning", "activating": "aligning",
                  "deactivating": "stopping", "inactive": "loaded" if self._loaded else "idle",
                  "failed": "fault"}
        state = states.get(active, "unknown")
        masters = json.loads(fields.pop("Arms", "{}"))
        execution = json.loads(fields.pop("Execution", "{}"))
        if active == "active":
            stage = execution.get("stage")
            state = "aligning"
            if stage == "failed":
                state = "fault"
            else:
                arms = list(execution.get("arms", {}).values())
                aligned = [arm for arm in arms if arm.get("stage") == "vendor_aligned"]
                following = [arm for arm in arms if arm.get("stage") == "vendor_following"]
                if any(arm.get("stage") == "failed" for arm in arms):
                    state = "fault"
                elif any(not arm.get("heartbeat_fresh") for arm in following):
                    state = "follow_interrupted"
                elif aligned and len(aligned) == len(arms) and all(arm.get("heartbeat_fresh") for arm in aligned):
                    state = "aligned"
                elif following and len(following) == len(arms):
                    state = "following"
                elif following:
                    state = "starting_follow"
        if "left" in self._hosts:
            masters = {side: masters.get(side, {})}
        return {"host": self._hosts[side], "state": state, "remote_service": fields,
                "masters": masters, "execution": execution}

    def _batch(self, sides, operation):
        sides = list(sides)
        if not sides:
            return {}, {}
        results, errors = {}, {}
        with ThreadPoolExecutor(max_workers=len(sides)) as pool:
            futures = {side: pool.submit(operation, side) for side in sides}
            for side, future in futures.items():
                try:
                    results[side] = future.result()
                except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
                    errors[side] = str(exc)
        return results, errors

    def _status(self):
        if self._init.get("state") == "initializing":
            return {"state": "initializing", "initialization": dict(self._init)}
        hosts, errors = self._batch(self._hosts, self._host_status)
        for side, error in errors.items():
            hosts[side] = {"host": self._hosts[side], "state": "unreachable", "error": error}
        states = {host["state"] for host in hosts.values()}
        active = states - {"idle", "loaded"}
        state = "loaded" if self._loaded else "idle"
        if active:
            for candidate in ("unreachable", "fault", "follow_interrupted", "stopping",
                              "aligning", "starting_follow", "following", "aligned", "unknown"):
                if candidate in active:
                    state = candidate
                    break
        right = hosts.get("right", {})
        return {"state": state, "acp": self._acp, "initialization": dict(self._init),
                "slave_ips": {side: self._settings[f"{side}_slave_ip"] for side in ("right", "left")},
                "hosts": hosts,
                "masters": {arm: data for entry in hosts.values()
                            for arm, data in entry.get("masters", {}).items()},
                "remote_service": right.get("remote_service", {}),
                "execution": right.get("execution", {})}

    def _stop_host(self, side):
        self._ssh(side, "systemctl", "--user", "stop", self._unit)
        service = self._host_status(side)["remote_service"]
        if (service.get("ActiveState") not in ("inactive", "failed") or
                any(service.get(key) != "0" for key in ("MainPID", "ControlPID"))):
            raise RuntimeError("PNP 遥操服务停止未确认")

    def _post_completion(self, action_id, status, result):
        from franka.fr3.device import JointControlPlugin
        return JointControlPlugin._post_completion(self, action_id, status, result)

    def _watch_alignment(self, action_id, cancelled):
        terminal = "error"
        result = {"error": "Alignment timed out"}
        deadline = time.monotonic() + 140
        while time.monotonic() < deadline:
            if cancelled.is_set():
                terminal, result = "cancelled", {"state": "paused"}
                break
            try:
                status = self._status()
            except Exception as exc:
                result = {"error": str(exc)}
                break
            if status["state"] == "aligned":
                terminal, result = "completed", status
                break
            if status["state"] in ("fault", "unreachable", "follow_interrupted"):
                result = status
                break
            cancelled.wait(.4)
        if terminal != "completed":
            _, stop_errors = self._batch(self._hosts, self._stop_host)
            result = {**result, "stop_errors": stop_errors}
            if stop_errors:
                terminal = "error"
        with self._lock:
            if self._acp and self._acp["action_id"] == action_id:
                self._acp = {"action_id": action_id, "state": "pending", "status": terminal}
        error = None
        for attempt in range(3):
            try:
                self._acp_callback(action_id, terminal, result)
                error = None
                break
            except Exception as exc:
                error = str(exc)[:240]
                time.sleep(.25 * (attempt + 1))
        with self._lock:
            if self._acp and self._acp["action_id"] == action_id:
                self._acp = {"action_id": action_id, "state": "failed" if error else "accepted",
                             "status": terminal, "error": error}

    def dispatch(self, action, args):
        # Core reserves lowercase stop for the card lifecycle.
        action = {"Align": "align", "Follow": "follow", "Stop": "pause",
                  "Status": "status"}.get(action, action)
        with self._lock:
            try:
                if action == "config":
                    self._configure(args)
                    return {"state": "configured", "initialization": dict(self._init)}
                if action in ("info", "status"):
                    return self._status()
                if action == "start":
                    self._configure(args)
                    self._loaded = True
                    self._initialize()
                    return self._status()
                if action == "follow":
                    if not self._loaded:
                        raise RuntimeError("Start the canvas project first")
                    status = self._status()
                    if status["state"] == "following":
                        return status
                    if status["state"] != "aligned":
                        raise RuntimeError("Execute align and wait for ACP completion before follow")
                    _, errors = self._batch(self._hosts, lambda side: self._ssh(side, "pnp-enable-follow")
                                            if status["hosts"][side]["state"] == "aligned" else None)
                    if errors:
                        return {"state": "error", "errors": errors}
                    return self._status()
                if action == "align":
                    if self._align_thread and self._align_thread.is_alive():
                        return {"state": "aligning", "action_id": self._acp["action_id"]}
                    if not self._loaded:
                        raise RuntimeError("Start the canvas project before alignment")
                    if self._init.get("state") != "ready":
                        return {"state": self._init["state"], "initialization": dict(self._init)}
                    status = self._status()
                    hosts = status["hosts"]
                    unavailable = {side: entry["error"] for side, entry in hosts.items()
                                   if entry["state"] == "unreachable"}
                    if unavailable:
                        return {"state": "error", "message": "PNP 主控机不可达", "errors": unavailable}
                    selected = {
                        side: [arm for arm, master in entry["masters"].items() if master.get("ready")]
                        for side, entry in hosts.items()
                    }
                    selected = {side: arms for side, arms in selected.items() if arms}
                    if not selected:
                        raise RuntimeError("PNP 主控机未识别到可读写的左右主臂")
                    active = {side for side, entry in hosts.items()
                              if entry["remote_service"].get("ActiveState") not in ("inactive", "failed")}
                    if active:
                        raise RuntimeError("Stop the existing session before starting a new alignment")
                    commands = {side: "pnp-align-both" if len(arms) == 2 else f"pnp-align-{arms[0]}"
                                for side, arms in selected.items()}
                    _, errors = self._batch(selected, lambda side: self._ssh(side, commands[side]))
                    if errors:
                        _, rollback = self._batch(selected, self._stop_host)
                        return {"state": "error", "message": "PNP 遥操启动未确认",
                                "errors": errors, "stop_errors": rollback}
                    action_id = "fr3_align_" + uuid.uuid4().hex
                    self._align_cancel = threading.Event()
                    self._acp = {"action_id": action_id, "state": "waiting_for_motion"}
                    self._align_thread = threading.Thread(target=self._watch_alignment,
                                                          args=(action_id, self._align_cancel), daemon=True)
                    self._align_thread.start()
                    return {"state": "aligning", "action_id": action_id}
                if action in ("pause", "stop"):
                    self._align_cancel.set()
                    if self._init.get("state") == "initializing":
                        if action == "stop":
                            self._loaded = False
                        return {"state": "initializing", "message": "初始化安装尚在进行；此阶段未启动遥操"}
                    _, errors = self._batch(self._hosts, self._stop_host)
                    if errors:
                        return {"state": "error", "message": "部分 PNP 遥操服务停止未确认",
                                "errors": errors}
                    if action == "stop":
                        self._loaded = False
                    return {"state": "paused" if self._loaded else "idle"}
            except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
                return {"state": "error", "message": str(exc)}
        return None
