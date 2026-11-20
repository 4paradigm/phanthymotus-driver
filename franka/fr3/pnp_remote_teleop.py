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


TRAJECTORY_NAME = re.compile(r"(?:franka|fr3)-(right|left)-\d{8}T\d{6}(?:\.\d{6})?(?:Z|[+-]\d{4})")


class PnpRemoteTeleopPlugin:
    PREFIX = "teach"
    COMPLETION_ACTIONS = ("stop_trajectory", "stop_motion", "delete_trajectory")

    def __init__(self, acp_callback=None):
        self._lock = threading.RLock()
        self._recording = None
        self._playback = None
        self._loaded = False
        self._stopped = False
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
            "start_trajectory": ([], "Align the arms, start PNP following, and record vendor frames."),
            "stop_trajectory": ([], "End the PNP teleoperation session and verify shutdown."),
            "stop_motion": ([], "Terminate teaching or playback, verify shutdown, and discard playback progress."),
            "list_trajectories": ([], "List saved teaching recordings."),
            "delete_trajectory": (["trajectory"], "Delete a saved teaching recording."),
            "play": (["left_trajectory", "right_trajectory"],
                     "Replay the supplied arms. Leave an arm blank to keep it out of this run."),
            "pause": ([], "Decelerate active playback and hold its position for resume. Use stop_motion during startup or approach."),
            "resume": ([], "Resume playback from the confirmed pause position."),
            "status": ([], "Read PNP service states and vendor execution stages."),
        }, {
            "trajectory": {"type": "string", "description": "Paste the name from list_trajectories."},
            "left_trajectory": {"type": "string", "title": "Left Arm Trajectory",
                                "description": "Optional left-arm recording name from list_trajectories."},
            "right_trajectory": {"type": "string", "title": "Right Arm Trajectory",
                                 "description": "Optional right-arm recording name from list_trajectories."},
        })
        schema.update({
            "x-completion": {"actions": ["start_trajectory", "play", *self.COMPLETION_ACTIONS], "timeout": 900},
            "x-resource": "arm",
            "x-is-dangerous": True,
            "x-hooks": {
                "on_interrupt_motion": {"action": "stop_motion"},
                "on_interrupt_all": {"action": "stop_motion"},
            },
        })
        definition = tool(self.PREFIX, "actuator", "FR3 PNP teaching and trajectory recording", schema)
        definition["configSchema"] = {
            "type": "object",
            "properties": {
                "ssh_user": {"type": "string", "title": "PNP Host Username", "default": "pnp"},
                "ssh_password": {"type": "string", "title": "PNP Host Password",
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

    def _inactive_state(self):
        if not self._loaded:
            return "idle"
        return "stop" if self._stopped else "loaded"

    def _host_status(self, side):
        output = self._ssh(side, "systemctl", "--user", "show", self._unit,
                           "-p", "LoadState", "-p", "ActiveState",
                           "-p", "SubState", "-p", "Result")
        fields = dict(line.split("=", 1) for line in output.splitlines() if "=" in line)
        if fields.get("LoadState") != "loaded":
            raise RuntimeError(f"PNP 主控机缺少专用服务 {self._unit}")
        active = fields.get("ActiveState")
        states = {"active": "aligning", "activating": "aligning",
                  "deactivating": "stopping", "inactive": self._inactive_state(),
                  "failed": "fault"}
        state = states.get(active, "unknown")
        masters = json.loads(fields.pop("Arms", "{}"))
        execution = json.loads(fields.pop("Execution", "{}"))
        if active == "active":
            stage = execution.get("stage")
            state = "aligning"
            if isinstance(stage, str) and stage.startswith("playback_"):
                state = stage
            elif stage == "failed":
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
        active = states - {"idle", "loaded", "stop"}
        state = self._inactive_state()
        if active:
            for candidate in ("unreachable", "fault", "follow_interrupted", "stopping",
                              "playback_starting", "playback_approaching", "playback_running",
                              "playback_paused", "playback_completed", "aligning", "starting_follow", "following", "aligned", "unknown"):
                if candidate in active:
                    state = candidate
                    break
        right = hosts.get("right", {})
        recording = self._recording
        return {"state": state, "acp": self._acp,
                "recording": ({"state": "recording"} if recording is not None else None),
                "initialization": dict(self._init),
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

    def _start_recording(self):
        self._recording = set(self._saved_trajectories())

    def _finish_recording(self, complete=False):
        if self._recording is None:
            return {}
        previous, self._recording = self._recording, None
        try:
            trajectories = [name for name in self._saved_trajectories() if name not in previous]
        except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
            return {"trajectory_error": str(exc)}
        return {"trajectories": trajectories, "complete": bool(trajectories) and complete}

    def _watch_alignment(self, action_id, cancelled, teach=False):
        terminal = "error"
        result = {"error": "Alignment timed out"}
        deadline = time.monotonic() + 140
        released = False
        while time.monotonic() < deadline:
            if cancelled.is_set():
                terminal, result = "cancelled", {"state": "stop"}
                break
            try:
                status = self._status()
            except Exception as exc:
                result = {"error": str(exc)}
                break
            if teach and status["state"] == "aligned" and not released:
                with self._lock:
                    if cancelled.is_set():
                        continue
                    self._acp["message"] = "Alignment complete. Starting Follow."
                    try:
                        self._start_recording()
                    except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
                        result = {"error": str(exc)}
                        break
                    _, errors = self._batch(
                        (side for side, host in status["hosts"].items() if host["state"] == "aligned"),
                        lambda side: self._ssh(side, "pnp-enable-follow-record"),
                    )
                if errors:
                    result = {"error": "Unable to start Follow", "details": errors}
                    break
                released = True
            if (teach and status["state"] == "following") or (not teach and status["state"] == "aligned"):
                if teach:
                    status["message"] = "Alignment complete. Follow is active."
                terminal, result = "completed", status
                break
            if status["state"] in ("fault", "unreachable", "follow_interrupted"):
                result = status
                break
            cancelled.wait(.4)
        if terminal != "completed":
            _, stop_errors = self._batch(self._hosts, self._stop_host)
            with self._lock:
                recording = self._finish_recording()
            result = {**result, "stop_errors": stop_errors, **recording}
            if stop_errors:
                terminal = "error"
        with self._lock:
            if self._acp and self._acp["action_id"] == action_id:
                self._acp = {"action_id": action_id, "state": "pending", "status": terminal}
        error = None
        for attempt in range(3):
            try:
                self._acp_callback(action_id, terminal, {
                    key: result[key] for key in ("state", "message", "error", "details", "stop_errors")
                    if key in result
                } if teach and terminal == "completed" else result)
                error = None
                break
            except Exception as exc:
                error = str(exc)[:240]
                time.sleep(.25 * (attempt + 1))
        with self._lock:
            if self._acp and self._acp["action_id"] == action_id:
                self._acp = {"action_id": action_id, "state": "failed" if error else "accepted",
                             "status": terminal, "error": error,
                             **({"message": result["message"]} if "message" in result else {})}

    def _trajectory_hosts(self):
        if self._init.get("state") != "ready":
            raise RuntimeError("Wait for PNP initialization before accessing trajectories")
        result = {}
        for side in self._hosts:
            names = json.loads(self._ssh(side, "pnp-list-recordings"))
            if not isinstance(names, list) or any(not isinstance(name, str) for name in names):
                raise ValueError("Invalid PNP trajectory listing")
            for name in names:
                if name in result:
                    raise ValueError(f"Duplicate PNP trajectory: {name}")
                result[name] = side
        return result

    def _saved_trajectories(self):
        return sorted(self._trajectory_hosts(), reverse=True)

    def _playback_selection(self, args):
        tracks = {}
        for arm in ("left", "right"):
            name = args.get(f"{arm}_trajectory")
            if name in (None, ""):
                continue
            match = TRAJECTORY_NAME.fullmatch(name) if isinstance(name, str) else None
            if not match or match[1] != arm:
                raise ValueError(f"Choose a {arm}-arm recording for {arm}_trajectory")
            tracks[arm] = name
        legacy = args.get("trajectory")
        if not tracks and isinstance(legacy, str) and (match := TRAJECTORY_NAME.fullmatch(legacy)):
            tracks[match[1]] = legacy
        if not tracks:
            raise ValueError("Choose at least one recording from list_trajectories")
        owners = self._trajectory_hosts()
        if any(name not in owners for name in tracks.values()):
            raise ValueError("A selected PNP recording does not exist")
        hosts = {owners[name] for name in tracks.values()}
        if len(hosts) != 1:
            raise ValueError("Synchronized playback requires both recordings on the same PNP host")
        return hosts.pop(), tracks

    def _watch_playback(self, action_id, side, cancelled):
        result = {"error": "Playback timed out"}
        terminal = "error"
        deadline = time.monotonic() + 870
        while not cancelled.wait(.3) and time.monotonic() < deadline:
            try:
                status = self._host_status(side)
                execution = status["execution"]
                if (execution.get("stage") == "playback_completed"
                        and status["remote_service"].get("ActiveState") == "inactive"):
                    terminal, result = "completed", {"state": "playback_completed", "execution": execution}
                    break
                if status["state"] == "fault":
                    result = {"error": execution.get("error", "PNP playback failed"), "execution": execution}
                    break
            except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
                result = {"error": str(exc)}
                break
        with self._lock:
            if cancelled.is_set():
                terminal, result = "cancelled", {"state": "stop"}
            elif terminal != "completed":
                try:
                    self._stop_host(side)
                except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
                    result["stop_error"] = str(exc)
            self._playback = None
            self._acp = {"action_id": action_id, "state": "pending", "status": terminal, "result": result}
        delivery_error = None
        for attempt in range(3):
            try:
                self._acp_callback(action_id, terminal, result)
                delivery_error = None
                break
            except Exception as exc:
                delivery_error = str(exc)[:240]
                if attempt < 2:
                    time.sleep(.25 * (attempt + 1))
        with self._lock:
            if self._acp and self._acp["action_id"] == action_id:
                self._acp.update(state="failed" if delivery_error else "accepted", error=delivery_error)

    def dispatch(self, action, args):
        result = self._dispatch_action(action, args)
        if action not in self.COMPLETION_ACTIONS or result is None:
            return result
        action_id = "fr3_" + action + "_" + uuid.uuid4().hex
        terminal = "error" if result.get("state") in ("error", "initializing") else "completed"

        def notify():
            for attempt in range(3):
                try:
                    self._acp_callback(action_id, terminal, result)
                    return
                except Exception as exc:
                    if attempt == 2:
                        print(f"[teleop] ACP {action_id} failed: {exc}", flush=True)
                    else:
                        time.sleep(.25 * (attempt + 1))

        threading.Thread(target=notify, name="fr3-teleop-acp", daemon=True).start()
        return {**result, "action_id": action_id}

    def _dispatch_action(self, action, args):
        normal_end = action == "stop_trajectory"
        # Core reserves lowercase stop for the card lifecycle.
        action = {"start_trajectory": "teach", "stop_trajectory": "stop_session",
                  "stop_motion": "stop_session", "Teach": "teach", "Align": "align", "Follow": "follow", "Stop": "stop_session",
                  "Status": "status"}.get(action, action)
        with self._lock:
            try:
                if action == "play":
                    if self._playback or not self._loaded:
                        raise RuntimeError("Load Teach and stop any existing playback first")
                    side, tracks = self._playback_selection(args)
                    status = self._status()
                    if any(entry.get("remote_service", {}).get("ActiveState") not in ("inactive", "failed")
                           for entry in status["hosts"].values()):
                        raise RuntimeError("Stop the active PNP session before playback")
                    self._ssh(side, "pnp-play-recording", *tracks.values())
                    cancelled = threading.Event()
                    action_id = "fr3_play_" + uuid.uuid4().hex
                    self._playback = (side, cancelled)
                    self._acp = {"action_id": action_id, "state": "waiting_for_motion"}
                    threading.Thread(target=self._watch_playback, args=(action_id, side, cancelled),
                                     daemon=True, name="fr3-playback").start()
                    return {"state": "playback_starting", "action_id": action_id, "selected_arms": list(tracks)}
                if action in ("pause", "resume"):
                    if self._playback is None:
                        raise RuntimeError("No active playback")
                    side, _ = self._playback
                    self._ssh(side, "pnp-pause-playback" if action == "pause" else "pnp-resume-playback")
                    return {"state": "pausing" if action == "pause" else "resuming"}
                if action == "list_trajectories":
                    names = self._saved_trajectories()
                    return {"trajectories": names, "trajectories_by_arm": {
                        arm: [name for name in names if name.startswith((f"franka-{arm}-", f"fr3-{arm}-"))]
                        for arm in ("left", "right")
                    }}
                if action == "delete_trajectory":
                    name = args.get("trajectory", "")
                    if not isinstance(name, str) or not TRAJECTORY_NAME.fullmatch(name):
                        raise ValueError("Choose a recording from list_trajectories")
                    side = self._trajectory_hosts().get(name)
                    if side is None:
                        raise ValueError("PNP recording does not exist")
                    self._ssh(side, "pnp-delete-recording", name)
                    return {"deleted": name}
                if action == "config":
                    self._configure(args)
                    return {"state": "configured", "initialization": dict(self._init)}
                if action in ("info", "status"):
                    return self._status()
                if action == "start":
                    self._configure(args)
                    self._loaded = True
                    self._stopped = False
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
                if action in ("align", "teach"):
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
                    self._stopped = False
                    action_id = "fr3_align_" + uuid.uuid4().hex
                    self._align_cancel = threading.Event()
                    self._acp = {"action_id": action_id, "state": "waiting_for_motion"}
                    self._align_thread = threading.Thread(target=self._watch_alignment,
                                                          args=(action_id, self._align_cancel, action == "teach"), daemon=True)
                    self._align_thread.start()
                    return {"state": "aligning", "action_id": action_id, "message": "Aligning. Please wait."}
                if action in ("stop_session", "stop"):
                    self._align_cancel.set()
                    if self._init.get("state") == "initializing":
                        if action == "stop":
                            self._loaded = False
                        return {"state": "initializing", "message": "初始化安装尚在进行；此阶段未启动遥操"}
                    _, errors = self._batch(self._hosts, self._stop_host)
                    if errors:
                        return {"state": "error", "message": "部分 PNP 遥操服务停止未确认",
                                "errors": errors}
                    if self._playback:
                        self._playback[1].set()
                    recording = self._finish_recording(normal_end)
                    if action == "stop":
                        self._loaded = False
                    self._stopped = action != "stop"
                    return {"state": "stop" if self._stopped else "idle", **recording}
            except (OSError, RuntimeError, ValueError, subprocess.TimeoutExpired) as exc:
                return {"state": "error", "message": str(exc)}
        return None
