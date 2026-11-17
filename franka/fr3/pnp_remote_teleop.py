"""Canvas interface for the dedicated arm service on the PNP workstation."""
from concurrent.futures import ThreadPoolExecutor
import json
import os
import subprocess
import threading

from common.vendor_runtime import action_schema, tool


class PnpRemoteTeleopPlugin:
    PREFIX = "teleop"

    def __init__(self):
        self._lock = threading.RLock()
        self._loaded = False
        self._unit = "pnp-fr3-right-teleop.service"
        self._hosts = {"right": os.environ.get("FRANKA_PNP_SSH_HOST", "pnp@172.16.0.200")}
        left = os.environ.get("FRANKA_PNP_LEFT_SSH_HOST", "").strip()
        if left:
            if left == self._hosts["right"]:
                raise ValueError("左右两台 PNP 主控机需要不同的 SSH 地址")
            self._hosts["left"] = left

    def get_tools(self):
        schema = action_schema({
            "follow": ([], "启动已连接主臂对应的 PNP 厂家遥操流程。启动过程中从臂会移到厂家预设姿态，夹爪会执行初始化。"),
            "pause": ([], "按厂家退出顺序停止 PNP 遥操服务并确认退出。"),
            "status": ([], "读取 PNP 服务状态和厂家脚本执行阶段。"),
        }, {})
        schema.update({
            "x-resource": "arm",
            "x-is-dangerous": True,
            "x-hooks": {
                "on_interrupt_motion": {"action": "pause"},
                "on_interrupt_all": {"action": "pause"},
            },
        })
        return [tool("teleop", "actuator", "PNP 主控机遥操", schema)]

    def start(self):
        pass

    def stop(self):
        result = self.dispatch("stop", {})
        if result.get("state") == "error":
            raise RuntimeError(result["message"])

    def _ssh(self, side, *args):
        host = self._hosts[side]
        result = subprocess.run(
            ["ssh", "-i", "/run/fr3-pnp-ssh/id_ed25519", "-o", "IdentitiesOnly=yes",
             "-o", "UserKnownHostsFile=/run/fr3-pnp-ssh/known_hosts",
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
        states = {"active": "starting_follow", "activating": "starting",
                  "deactivating": "stopping", "inactive": "loaded" if self._loaded else "idle",
                  "failed": "fault"}
        state = states.get(active, "unknown")
        masters = json.loads(fields.pop("Arms", "{}"))
        execution = json.loads(fields.pop("Execution", "{}"))
        if active == "active":
            stage = execution.get("stage")
            state = "starting_follow"
            if stage == "failed":
                state = "fault"
            else:
                arms = list(execution.get("arms", {}).values())
                following = [arm for arm in arms if arm.get("stage") == "vendor_following"]
                if any(arm.get("stage") == "failed" for arm in arms):
                    state = "fault"
                elif any(not arm.get("heartbeat_fresh") for arm in following):
                    state = "follow_interrupted"
                elif following and len(following) == len(arms):
                    state = "following"
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
        hosts, errors = self._batch(self._hosts, self._host_status)
        for side, error in errors.items():
            hosts[side] = {"host": self._hosts[side], "state": "unreachable", "error": error}
        states = {host["state"] for host in hosts.values()}
        active = states - {"idle", "loaded"}
        state = "loaded" if self._loaded else "idle"
        if active:
            for candidate in ("unreachable", "fault", "follow_interrupted", "stopping",
                              "starting", "starting_follow", "following", "unknown"):
                if candidate in active:
                    state = candidate
                    break
        right = hosts.get("right", {})
        return {"state": state, "hosts": hosts,
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

    def dispatch(self, action, args):
        with self._lock:
            try:
                if action in ("config", "info", "status"):
                    return self._status()
                if action == "start":
                    self._status()
                    self._loaded = True
                    return self._status()
                if action == "follow":
                    if not self._loaded:
                        raise RuntimeError("先启动画布项目，再执行 follow")
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
                        if active == set(selected):
                            return status
                        raise RuntimeError("请先暂停当前会话，再启动新的主臂组合")
                    commands = {side: "pnp-follow-both" if len(arms) == 2 else f"pnp-follow-{arms[0]}"
                                for side, arms in selected.items()}
                    _, errors = self._batch(selected, lambda side: self._ssh(side, commands[side]))
                    if errors:
                        _, rollback = self._batch(selected, self._stop_host)
                        return {"state": "error", "message": "PNP 遥操启动未确认",
                                "errors": errors, "stop_errors": rollback}
                    return self._status()
                if action in ("pause", "stop"):
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
