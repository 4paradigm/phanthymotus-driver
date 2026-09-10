"""Bounded, cancellable simulation sessions. No robot SDK lives in this process."""

from __future__ import annotations

import base64
import collections
import copy
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import uuid

from upstream import inspect_installation


USER_OPTIONS = frozenset({
    "source", "bvh_path", "max_steps", "human_height", "render", "pico_advertise_host",
})
ACTIVE_STATES = frozenset({"starting", "running", "pausing", "paused", "resuming", "finishing", "stopping"})


def validate_options(options: dict) -> dict:
    """Validate controls independently of JSON Schema (MCP callers may bypass it)."""
    if not isinstance(options, dict):
        raise ValueError("遥操作设置必须是对象")
    unknown = set(options) - USER_OPTIONS
    if unknown:
        raise ValueError(f"不支持的仿真设置: {', '.join(sorted(unknown))}")
    result = dict(options)
    if "source" in result and result["source"] not in {"bvh", "pico"}:
        raise ValueError("source 只能是 bvh 或 pico；此 Driver 不提供真机输出")
    if "max_steps" in result:
        value = result["max_steps"]
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 180000:
            raise ValueError("max_steps 必须是 0–180000 的整数（0 表示直到停止）")
    if "human_height" in result:
        value = result["human_height"]
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not 0.8 <= value <= 2.5:
            raise ValueError("human_height 必须是 0.8–2.5 米的有限数值")
    if "render" in result and not isinstance(result["render"], bool):
        raise ValueError("render 必须是布尔值")
    for name in ("bvh_path", "pico_advertise_host"):
        if name in result and (not isinstance(result[name], str) or len(result[name]) > 4096 or any(ord(c) < 32 for c in result[name])):
            raise ValueError(f"{name} 必须是单行字符串")
    return result


class SimulationManager:
    """One subprocess per run, generation-fenced so stop preempts slow startup."""

    def __init__(self, config: dict | None = None):
        self.config = dict(config or {})
        self._lock = threading.RLock()
        self._stop_lock = threading.Lock()
        self._generation = 0
        self._state = "idle"
        self._process: subprocess.Popen | None = None
        self._session_id: str | None = None
        self._snapshot: dict = {}
        self._preview: bytes | None = None
        self._snapshot_at: float | None = None
        self._error: str | None = None
        self._logs: collections.deque[str] = collections.deque(maxlen=24)
        self._last_preflight: dict | None = None
        self._backend: dict = {}
        self._diagnostics: collections.deque[dict] = collections.deque(maxlen=12)
        self._closed = False

    def _options(self, supplied: dict | None) -> dict:
        supplied = validate_options(supplied or {})
        defaults = {"source": "bvh", "bvh_path": "", "max_steps": 500,
                    "human_height": 1.75, "render": True, "pico_advertise_host": ""}
        defaults.update({k: v for k, v in self.config.items() if k in USER_OPTIONS})
        defaults.update(supplied)
        validate_options(defaults)
        root = self.config.get("upstream_root") or os.environ.get("TELEOPIT_ROOT", "")
        return {**defaults, "upstream_root": str(Path(root).expanduser().resolve()) if root else "",
                "policy_path": self.config.get("policy_path") or "",
                "pico_host": self.config.get("pico_host", "0.0.0.0"),
                "pico_port": self.config.get("pico_port", 63901),
                "input_timeout_s": self.config.get("input_timeout_s", 10.0)}

    def _python(self) -> str:
        return str(self.config.get("worker_python") or os.environ.get("TELEOPIT_PYTHON") or sys.executable)

    def _check(self, options: dict) -> dict:
        if not options["upstream_root"]:
            return {"ready": False, "errors": ["尚未安装 Teleopit；先运行 setup_teleopit.py 并配置 TELEOPIT_ROOT"], "paths": {}}
        result = inspect_installation(
            Path(options["upstream_root"]), policy_path=options["policy_path"] or None,
            bvh_path=options["bvh_path"] or None, source=options["source"],
        )
        result = dict(result)
        errors = list(result.get("errors", []))
        if result.get("ready"):
            # Probe in the selected interpreter, not the ROS2/Core interpreter.
            modules = ["teleopit", "mujoco", "mink", "onnxruntime", "torch", "omegaconf", "daqp",
                       "hydra", "scipy", "qpsolvers", "numpy", "h5py", "zmq", "msgpack",
                       "rich", "loop_rate_limiters", "imageio"]
            versions = {"teleopit": "0.5.0", "mujoco": "3.2.7", "mink": "0.0.13",
                        "onnxruntime": "1.23.2", "torch": "2.2.2", "numpy": "1.26.4",
                        "hydra-core": "1.3.6", "omegaconf": "2.3.1"}
            if options["render"]:
                modules.append("PIL")
            if options["source"] == "pico":
                modules.append("pico_bridge")
                versions["pico-bridge"] = "0.2.1"
            probe = ("import importlib.util,importlib.metadata,json; "
                     f"names={modules!r}; versions={versions!r}; "
                     "installed={d.metadata['Name'].lower():d.version for d in importlib.metadata.distributions() if d.metadata['Name']}; "
                     "print(json.dumps({'missing':[n for n in names if importlib.util.find_spec(n) is None],"
                     "'version_mismatches':[n+': '+installed.get(n,'missing')+' != '+v for n,v in versions.items() if installed.get(n,'').split('+')[0]!=v],"
                     "'origin':str(importlib.util.find_spec('teleopit').origin) if importlib.util.find_spec('teleopit') else ''}))")
            try:
                proc = subprocess.run([self._python(), "-c", probe], capture_output=True, text=True, timeout=10, check=True)
                data = json.loads(proc.stdout)
                if data["missing"]:
                    errors.append("仿真解释器缺少依赖: " + ", ".join(data["missing"]))
                if data["version_mismatches"]:
                    errors.append("仿真依赖与已验证版本不匹配: " + ", ".join(data["version_mismatches"]))
                expected = Path(options["upstream_root"]) / "teleopit" / "__init__.py"
                if data["origin"] and Path(data["origin"]).resolve() != expected.resolve():
                    errors.append("仿真解释器导入了另一份 teleopit；请对选定源码目录执行 editable 安装")
            except (OSError, subprocess.SubprocessError, ValueError, KeyError) as exc:
                errors.append(f"无法检查仿真解释器 {self._python()}: {exc}")
        result.update(ready=not errors, errors=errors, worker_python=self._python())
        return result

    def preflight(self, options: dict | None = None) -> dict:
        try:
            result = self._check(self._options(options))
        except (ValueError, OSError) as exc:
            result = {"ready": False, "errors": [str(exc)], "paths": {}}
        return {**result, "state": "ready" if result["ready"] else "error",
                "hardware_output": False, "profile": "g1_29_sim"}

    def run(self, options: dict | None = None) -> dict:
        resolved = self._options(options)
        with self._lock:
            if self._closed:
                raise ValueError("Driver 已关闭")
            if self._state in ACTIVE_STATES or self._process is not None:
                return {**self.info(), "accepted": False, "reason": "已有仿真会话；请先停止"}
            self._generation += 1
            generation = self._generation
            self._state = "starting"
            self._session_id = uuid.uuid4().hex
            self._snapshot = {}
            self._preview = None
            self._snapshot_at = None
            self._error = None
            self._last_preflight = None
            self._backend = {}
            self._diagnostics.clear()
            self._logs.clear()
            threading.Thread(target=self._launch, args=(generation, resolved), name="teleopit-launch", daemon=True).start()
            return {**self.info(), "accepted": True}

    def _launch(self, generation: int, options: dict) -> None:
        read_fd = write_fd = None
        try:
            check = self._check(options)
            with self._lock:
                if generation != self._generation:
                    return
                self._last_preflight = check
                if not check["ready"]:
                    self._state = "error"
                    self._error = "; ".join(check["errors"])
                    return
                paths = check["paths"]
                options.update(upstream_root=str(paths.get("root", options["upstream_root"])),
                               policy_path=str(paths.get("policy", options["policy_path"])),
                               bvh_path=str(paths.get("bvh") or options["bvh_path"]))
                read_fd, write_fd = os.pipe()
                process = subprocess.Popen(
                    [self._python(), str(Path(__file__).with_name("worker.py")), "--event-fd", str(write_fd)],
                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, encoding="utf-8", errors="replace", bufsize=1,
                    pass_fds=(write_fd,), start_new_session=True,
                )
                self._process = process
                os.close(write_fd)
                write_fd = None
                process.stdin.write(json.dumps(options, allow_nan=False) + "\n")
                process.stdin.flush()
            threading.Thread(target=self._read_logs, args=(generation, process), daemon=True, name="teleopit-logs").start()
            with os.fdopen(read_fd, encoding="utf-8") as stream:
                read_fd = None
                for line in stream:
                    self._receive(generation, json.loads(line))
            return_code = process.wait()
            if process.stdin:
                process.stdin.close()
            with self._lock:
                if generation == self._generation:
                    self._process = None
                    if self._state == "finishing" and return_code == 0:
                        self._state = "completed"
                    elif self._state in ACTIVE_STATES:
                        self._state = "error"
                        self._error = f"仿真子进程意外退出（code={return_code}），未收到完成事件"
                    elif return_code != 0 and self._state == "completed":
                        self._state = "error"
                        self._error = f"仿真子进程退出失败（code={return_code}）"
        except Exception as exc:
            error = f"启动/读取 Teleopit 失败: {exc}"
            with self._lock:
                if generation == self._generation:
                    process = self._process
                    self._state = "stopping" if process is not None else "error"
                    self._error = error
                else:
                    process = None
            self._terminate(process)
            with self._lock:
                if generation == self._generation:
                    self._process = None
                    self._state = "error"
        finally:
            for fd in (read_fd, write_fd):
                if fd is not None:
                    os.close(fd)

    def _read_logs(self, generation: int, process: subprocess.Popen) -> None:
        if process.stdout:
            with process.stdout:
                for line in process.stdout:
                    with self._lock:
                        if generation == self._generation:
                            self._logs.append(line.rstrip()[:1024])

    def _receive(self, generation: int, event: dict) -> None:
        with self._lock:
            if generation != self._generation:
                return
            kind = event.get("event")
            if kind == "ready":
                self._state = "running"
                self._backend = {key: value for key, value in event.items() if key != "event"}
            elif kind == "paused":
                self._state = "paused"
            elif kind == "resumed":
                self._state = "running"
            elif kind == "frame":
                snapshot = event["snapshot"]
                json.dumps(snapshot, allow_nan=False)
                self._snapshot = snapshot
                self._backend["waiting_for_input"] = False
                self._snapshot_at = time.monotonic()
                if event.get("preview_jpeg_b64"):
                    self._preview = base64.b64decode(event["preview_jpeg_b64"], validate=True)
            elif kind == "complete":
                self._state = "finishing"
                self._snapshot = {**self._snapshot, "summary": event.get("summary", {})}
            elif kind == "error":
                self._state = "error"
                self._error = event.get("error", "Teleopit 仿真失败")
            elif kind == "diagnostic":
                self._diagnostics.append({key: value for key, value in event.items() if key != "event"})

    def _command(self, command: str) -> bool:
        process = self._process
        if process is None or process.poll() is not None or process.stdin is None:
            return False
        try:
            process.stdin.write(json.dumps({"command": command}) + "\n")
            process.stdin.flush()
            return True
        except (BrokenPipeError, OSError, ValueError):
            return False

    def pause(self) -> dict:
        with self._lock:
            if self._state == "running" and self._command("pause"):
                self._state = "pausing"
            return self.info()

    def resume(self) -> dict:
        with self._lock:
            if self._state == "paused" and self._command("resume"):
                self._state = "resuming"
            return self.info()

    @staticmethod
    def _terminate(process: subprocess.Popen | None) -> None:
        if process is None:
            return
        if process.poll() is None:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=2.0)
        if process.stdin:
            try:
                process.stdin.close()
            except OSError:
                pass

    def stop(self) -> dict:
        # A second concurrent stop must not announce idle while the first is
        # still reaping its child (which may own the PICO receiver port).
        with self._stop_lock:
            with self._lock:
                self._command("stop")
                self._generation += 1
                generation = self._generation
                process, self._process = self._process, None
                self._state = "stopping"
            self._terminate(process)
            with self._lock:
                if generation == self._generation:
                    self._state = "idle"
                    self._preview = None
                return self.info()

    def info(self) -> dict:
        with self._lock:
            return {"state": self._state, "session_id": self._session_id,
                    "profile": "g1_29_sim", "hardware_output": False,
                    "snapshot": copy.deepcopy(self._snapshot), "error": self._error,
                    "snapshot_age_ms": round((time.monotonic() - self._snapshot_at) * 1000, 1) if self._snapshot_at else None,
                    "preflight": copy.deepcopy(self._last_preflight), "logs": list(self._logs),
                    "backend": copy.deepcopy(self._backend), "diagnostics": copy.deepcopy(list(self._diagnostics))}

    def preview(self) -> bytes | None:
        with self._lock:
            return self._preview

    def close(self) -> None:
        with self._lock:
            self._closed = True
        self.stop()
