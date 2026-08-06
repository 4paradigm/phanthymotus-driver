"""Optional lifecycle manager for an EngineAI Native SDK runtime."""

from __future__ import annotations

import os
import shlex
import signal
import subprocess
import threading
import time
from pathlib import Path


class NativeSdkManager:
    """Manage either a child Native SDK process or the robot systemd service.

    The default ``external`` mode is observation-only.  Process and systemd
    control must be selected explicitly in config.yaml.
    """

    def __init__(self, config: dict):
        self._config = config
        self._mode = str(config.get("mode", "external"))
        self._process: subprocess.Popen | None = None
        self._lock = threading.RLock()
        self._started_at: float | None = None

    def tool(self) -> dict:
        actions = ["status"]
        if self._mode in ("process", "systemd"):
            actions.extend(["start", "stop", "restart"])
        return {
            "name": "native_sdk",
            "type": "actuator",
            "multiInstance": False,
            "description": "EngineAI Native SDK runtime lifecycle and integration status",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": actions},
                },
                "required": ["action"],
            },
        }

    def start(self) -> dict:
        if self._mode == "external":
            return {"state": "external", "managed": False}
        if self._mode == "systemd":
            return self._systemctl("start")
        if self._mode != "process":
            return {"error": f"unsupported Native SDK mode: {self._mode}"}

        with self._lock:
            if self._process is not None and self._process.poll() is None:
                return self.status()
            command = self._command()
            workdir = Path(os.path.expandvars(str(self._config.get("workdir", ".")))).expanduser()
            if not workdir.is_dir():
                return {"error": f"Native SDK workdir does not exist: {workdir}"}
            self._process = subprocess.Popen(
                command,
                cwd=workdir,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self._started_at = time.time()
            return self.status()

    def stop(self) -> dict:
        if self._mode == "external":
            return {"state": "external", "managed": False}
        if self._mode == "systemd":
            return self._systemctl("stop")
        with self._lock:
            process = self._process
            self._process = None
        if process is None or process.poll() is not None:
            return {"state": "stopped", "managed": True}
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=float(self._config.get("stop_timeout", 10)))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        return {"state": "stopped", "managed": True, "exit_code": process.returncode}

    def restart(self) -> dict:
        self.stop()
        return self.start()

    def status(self) -> dict:
        if self._mode == "systemd":
            return self._systemctl("is-active", mutating=False)
        if self._mode == "external":
            return {
                "state": "external",
                "managed": False,
                "source_revision": self._config.get("source_revision", "unknown"),
            }
        with self._lock:
            process = self._process
            running = process is not None and process.poll() is None
            return {
                "state": "running" if running else "stopped",
                "managed": True,
                "pid": process.pid if running else None,
                "exit_code": None if running else (process.returncode if process else None),
                "started_at": self._started_at,
                "source_revision": self._config.get("source_revision", "unknown"),
            }

    def dispatch(self, action: str) -> dict:
        if action in ("info", "status"):
            return self.status()
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "restart":
            return self.restart()
        return {"error": f"unknown Native SDK action: {action}"}

    def _command(self) -> list[str]:
        value = self._config.get("command", [])
        if isinstance(value, str):
            value = shlex.split(value)
        if not isinstance(value, list) or not value:
            raise ValueError("native_sdk.command must be a non-empty string or array")
        return [os.path.expandvars(str(part)) for part in value]

    def _systemctl(self, verb: str, *, mutating: bool = True) -> dict:
        service = str(self._config.get("service", "robotics.service"))
        prefix = self._config.get(
            "systemctl_prefix",
            ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--"],
        )
        command = [*prefix, "systemctl", verb, service]
        result = subprocess.run(command, capture_output=True, text=True, timeout=15)
        output = (result.stdout or result.stderr).strip()
        if verb == "is-active":
            state = output or ("running" if result.returncode == 0 else "stopped")
        else:
            state = "running" if verb in ("start", "restart") and result.returncode == 0 else "stopped"
        payload = {
            "state": state,
            "managed": True,
            "mode": "systemd",
            "service": service,
            "returncode": result.returncode,
        }
        if output:
            payload["output"] = output[-1000:]
        if result.returncode != 0 and mutating:
            payload["error"] = f"systemctl {verb} failed"
        return payload
