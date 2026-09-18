"""Third-generation RM75 drag-teaching action card."""

from __future__ import annotations

import json
import math
import re
import threading
import time
from pathlib import Path
from uuid import uuid4

from common.vendor_runtime import action_schema, tool


class ActionRecord:
    def __init__(self, plugin, config, joint_limits):
        self.plugin = plugin
        self.client = plugin.client
        self.root = Path(config.get("directory", "/var/lib/realman/rm75-actions"))
        self.root.mkdir(parents=True, exist_ok=True)
        self.index_path = self.root / "index.json"
        self.speed = max(1, min(100, int(config.get("speed_percent", 20))))
        self.timeout = max(30, int(config.get("replay_timeout_seconds", 600)))
        self._lock = threading.RLock()
        self._recording = False
        self._record_name = None
        self._active = None
        self._cancel = set()

    def _index(self):
        if not self.index_path.exists():
            return {}
        try:
            data = json.loads(self.index_path.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def _write_index(self, data):
        tmp = self.index_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(self.index_path)

    @staticmethod
    def _name(raw):
        value = str(raw or "").strip()
        if not value or len(value) > 64 or not re.fullmatch(r"[\w.-]+", value, re.UNICODE):
            raise ValueError("name must be 1-64 characters and contain only letters, numbers, _, -, or .")
        return value

    def get_tool(self):
        props = {
            "name": {"type": "string", "description": "Named action"},
            "speed_percent": {"type": "integer", "minimum": 1, "maximum": 100, "default": self.speed},
            "confirm_motion": {"type": "boolean", "description": "Must be true for recording or replay"},
        }
        schema = action_schema({
            "record_start": (["name", "confirm_motion"], "Start drag teaching and record a named action"),
            "record_stop": ([], "Stop drag teaching and save the action"),
            "replay": (["name", "speed_percent", "confirm_motion"], "Replay a saved named action"),
            "list": ([], "List saved actions"),
            "stop": ([], "Stop action replay"),
            "info": ([], "Read recording and replay status"),
        }, props)
        schema["x-completion"] = {"actions": ["replay"], "timeout": self.timeout + 30}
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "stop"}}
        schema["x-is-dangerous"] = True
        return tool("action_record", "actuator", "RM75 third-generation drag teaching recorder and named action replay", schema)

    def _ensure_motion(self, args):
        if not self.client.motion_enabled:
            raise PermissionError("motion is locked; set RM_MOTION_ENABLED=1 for supervised hardware testing")
        if args.get("confirm_motion") is not True:
            raise ValueError("confirm_motion must be true")

    def dispatch(self, action, args):
        if action == "start":
            return {"state": "ready"}
        if action == "info":
            with self._lock:
                return {"state": "recording" if self._recording else "replaying" if self._active else "idle",
                        "recording_name": self._record_name, "active_action_id": self._active,
                        "actions": self._index()}
        if action == "list":
            return {"actions": self._index()}
        if action == "record_start":
            self._ensure_motion(args)
            name = self._name(args.get("name"))
            if not self.client.motion_gate.acquire(blocking=False):
                raise RuntimeError("another arm operation is active")
            try:
                self.client.command("rm_start_drag_teach", 1)
            except Exception:
                self.client.motion_gate.release()
                raise
            with self._lock:
                self._recording, self._record_name = True, name
            return {"state": "recording", "name": name}
        if action == "record_stop":
            return self._stop_recording()
        if action == "replay":
            self._ensure_motion(args)
            return self._start_replay(self._name(args.get("name")), int(args.get("speed_percent", self.speed)))
        if action == "stop":
            return self.stop_active() or {"state": "idle", "action_id": None}
        return None

    def _stop_recording(self):
        with self._lock:
            if not self._recording:
                raise RuntimeError("no recording is active")
            name = self._record_name
        try:
            self.client.command("rm_stop_drag_teach")
            raw = self.root / f"{name}.trajectory.txt"
            result = self.client.call("rm_save_trajectory", str(raw))
            points = int(result) if isinstance(result, (int, float)) else None
            lines = raw.read_text(encoding="utf-8").splitlines(keepends=True)
            project = self.root / f"{name}.project.txt"
            trajectory_count = points if points is not None else len(lines)
            project.write_text(
                f'{{"file":7}}\n{{"name":"Folder","num":1,"type":{trajectory_count},"enabled":true,"parent_number":0}}\n'
                + "".join(lines), encoding="utf-8")
            slot = (max([int(v.get("slot", 0)) for v in self._index().values()] or [0]) + 1)
            self.client.upload_recording(project, self.speed, slot)
            data = self._index(); data[name] = {"name": name, "slot": slot, "points": points, "updated": time.time()}
            self._write_index(data)
            return {"state": "saved", "name": name, "points": points, "slot": slot}
        finally:
            with self._lock:
                self._recording, self._record_name = False, None
            self.client.motion_gate.release()

    def _start_replay(self, name, speed):
        if not 1 <= speed <= 100:
            raise ValueError("speed_percent must be within [1, 100]")
        item = self._index().get(name)
        if not item:
            raise ValueError(f"unknown action: {name}")
        if not self.client.motion_gate.acquire(blocking=False):
            raise RuntimeError("another arm operation is active")
        action_id = f"rm75_replay_{uuid4().hex[:10]}"
        try:
            project = self.root / f"{name}.project.txt"
            if not project.is_file():
                raise RuntimeError(f"saved project file is missing: {project}")
            # Third-generation controllers are more reliable when replay is
            # started by sending the saved project with only_save=0. The
            # program-ID runner may reject locally assigned IDs with code 1.
            self.client.upload_recording(project, speed, int(item["slot"]), run=True)
        except Exception:
            self.client.motion_gate.release()
            raise
        with self._lock:
            self._active = action_id
            self._cancel.discard(action_id)
        threading.Thread(target=self._monitor, args=(action_id, name), daemon=True).start()
        return {"state": "running", "action_id": action_id, "name": name}

    def _monitor(self, action_id, name):
        status, result = "error", {"reason": "replay_timeout"}
        started = time.monotonic()
        seen_running = False
        try:
            while time.monotonic() - started < self.timeout:
                with self._lock:
                    if action_id in self._cancel:
                        self.client.command("rm_set_arm_slow_stop")
                        status, result = "cancelled", {"reason": "stop"}
                        break
                state = self.client.call("rm_get_program_run_state")
                run_state = int(state.get("run_state", 0)) if isinstance(state, dict) else 0
                seen_running = seen_running or run_state in (1, 2)
                if (seen_running and run_state == 0) or (not seen_running and time.monotonic() - started > 3):
                    status, result = "completed", {"reason": "trajectory_finished", "name": name}
                    break
                time.sleep(1)
        except Exception as exc:
            status = "error"
            result = {"reason": str(exc), "name": name}
        finally:
            with self._lock:
                self._active = None; self._cancel.discard(action_id)
            self.client.motion_gate.release()
            self.plugin._acp_callback(action_id, status, result)

    def stop_active(self):
        with self._lock:
            action_id = self._active
            recording = self._recording
            recording_name = self._record_name
            if recording:
                self._recording, self._record_name = False, None
        if recording:
            try:
                self.client.command("rm_stop_drag_teach")
            finally:
                self.client.motion_gate.release()
            return {"state": "recording_stopped", "name": recording_name}
        if action_id:
                self._cancel.add(action_id)
        if action_id:
            return {"state": "stop_requested", "action_id": action_id}
        return None

    def shutdown(self):
        self.stop_active()
        with self._lock:
            recording = self._recording
        if recording:
            try: self.client.command("rm_stop_drag_teach")
            except Exception: pass
