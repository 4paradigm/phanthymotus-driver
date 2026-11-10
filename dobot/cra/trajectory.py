"""Persistent timestamped recordings and an explicitly configured native bridge.

The public TCP manual does not define native CSV columns. No native format is
guessed here: an installation must supply a profile derived from its controller.
The default records locally and refuses native playback until that is supplied.
"""

from __future__ import annotations

import copy
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import queue
import re
import threading
import time
from uuid import uuid4

if __package__:
    from .control import JOINT_LIMITS_DEG, trajectory_filename
    from .native_mounts import readable_profile, writable_controller_directory
else:
    from control import JOINT_LIMITS_DEG, trajectory_filename
    from native_mounts import readable_profile, writable_controller_directory

SAMPLE_MS = 50
MAX_POINTS = 10000
MAX_BYTES = 32 * 1024 * 1024
REST_SPEED_DEG_S = 0.1
# Field capture (CR3A, drag button): 5 -> 0 (enabled, drag_status=1, ~24 ms) -> 6.
DRAG_ENTRY_TRANSITION_MS = 100
# Field capture: one mode-5 packet with running_status=1 right after leaving drag.
DRAG_EXIT_RUNNING_MS = 100


def protection_fault(state):
    """Protection reasons that end a recording regardless of mode 5/6."""
    if not state["enable_status"]:
        return "robot disabled"
    if state["error_status"]:
        return "controller alarm"
    if state["collision_state"]:
        return "collision detected"
    if any(state[f"{part}_approach_state"] for part in ("arm", "j4", "j5", "j6")):
        return "safe skin triggered"
    if any(state["brake_released"]):
        return "brakes released"
    if state["robot_type"] != 113:
        return "controller is not CR3A (type 113)"
    return None


def interval_fault(state):
    """Mode 5 is only an acceptable pause between drags when nothing else runs."""
    if state["running_status"]:
        return "robot is running"
    if state["jog_status"]:
        return "robot is jogging"
    if state["run_queued_cmd"]:
        return "command queue is active"
    if state["pause_cmd_flag"]:
        return "command queue is paused"
    return None


def drag_state_fault(state):
    """Mode 6 without drag_status: the arm stays back-drivable/damped but the
    controller no longer reports drag (seen after StopDrag during a button drag)."""
    if state["robot_mode"] == 6 and not state["drag_status"]:
        return ("controller drag state inconsistent (mode 6 without drag status); "
                "recover drag on the arm or teach pendant")
    return None


def at_rest_after_drag(state):
    return (state["robot_mode"] == 5 and state["enable_status"] and not state["drag_status"]
            and interval_fault(state) is None
            and all(abs(v) <= REST_SPEED_DEG_S for v in state["joint_state"]["qd_actual"]))


def check_joints(values):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError("six joint angles are required")
    if any(type(v) not in (float, int) or not math.isfinite(v) or not lo <= v <= hi
           for v, (lo, hi) in zip(values, JOINT_LIMITS_DEG)):
        raise ValueError("joint angles must be finite and within CR3A travel")
    return [float(v) for v in values]


def atomic_json(path, value):
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as file:
        json.dump(value, file, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
        file.flush()
        os.fsync(file.fileno())
    os.replace(temporary, path)


class NativeCsvProfile:
    """Data-only layout; never execute a format script or presume six columns.

Supported fields are joint1_deg..joint6_deg, time_ms/time_s, user and tool.
Other constant columns/header rows can be specified as literal CSV cells.
No production profile is bundled because no controller sample is available.
"""

    def __init__(self, path):
        profile_path = readable_profile(path)
        if profile_path.stat().st_size > 65536:
            raise ValueError("native CSV profile exceeds 64 KiB")
        self.data = json.loads(profile_path.read_text(encoding="utf-8"))
        self.digest = hashlib.sha256(profile_path.read_bytes()).hexdigest()
        data = self.data
        if data.get("version") != 1 or not isinstance(data.get("format_id"), str) or not data["format_id"]:
            raise ValueError("native CSV profile requires version=1 and format_id")
        columns = data.get("columns")
        if not isinstance(columns, list) or not 6 <= len(columns) <= 64:
            raise ValueError("native CSV profile requires 6..64 columns")
        fields = []
        allowed = {f"joint{i}_deg" for i in range(1, 7)} | {"time_ms", "time_s", "user", "tool"}
        for column in columns:
            if isinstance(column, str) and column in allowed:
                fields.append(column)
            elif (isinstance(column, dict) and set(column) == {"literal"}
                  and type(column["literal"]) in (str, int, float)):
                if isinstance(column["literal"], float) and not math.isfinite(column["literal"]):
                    raise ValueError("invalid CSV literal")
            else:
                raise ValueError("unsupported native CSV column")
        if any(fields.count(f"joint{i}_deg") != 1 for i in range(1, 7)):
            raise ValueError("native CSV profile must map every joint exactly once")
        header = data.get("header", [])
        if not isinstance(header, list) or len(header) > 32 or any(
                not isinstance(row, list) or len(row) > 64 or any(type(v) not in (str, int) for v in row)
                for row in header):
            raise ValueError("invalid CSV header rows")
        if data.get("delimiter", ",") not in (",", ";", "\t"):
            raise ValueError("unsupported CSV delimiter")

    def encode(self, recording):
        output = io.StringIO(newline="")
        writer = csv.writer(output, delimiter=self.data.get("delimiter", ","), lineterminator="\n")
        writer.writerows(self.data.get("header", []))
        for point in recording["points"]:
            joints = check_joints(point["joints_deg"])
            values = {f"joint{i}_deg": format(v, ".9f") for i, v in enumerate(joints, 1)}
            values.update(time_ms=point["time_ms"], time_s=point["time_ms"] / 1000,
                          user=recording["frames"]["user"], tool=recording["frames"]["tool"])
            writer.writerow([values[column] if isinstance(column, str) else column["literal"]
                             for column in self.data["columns"]])
            if output.tell() > MAX_BYTES:
                raise ValueError("native CSV exceeds storage size limit")
        return output.getvalue().encode("utf-8")


class TrajectoryStore:
    def __init__(self, directory, controller_directory="", profile_path="", *, require_network_mount=True):
        self.root = Path(directory).expanduser().resolve()
        self.controller_root = Path(controller_directory).expanduser().resolve() if controller_directory else None
        self.profile_path = profile_path
        self.require_network_mount = require_network_mount
        self._lock = threading.RLock()
        # Lazy directory creation: simply discovering cards has no disk effects.
        self._recovered = False

    def _directory(self, identifier):
        if not isinstance(identifier, str) or not re.fullmatch(r"[a-f0-9]{32}", identifier):
            raise ValueError("invalid trajectory_id")
        path = self.root / identifier
        if path.is_symlink():
            raise ValueError("trajectory directory may not be a symlink")
        return path

    def _read(self, path):
        if path.is_symlink() or path.stat().st_size > MAX_BYTES:
            raise ValueError("invalid or oversized trajectory file")
        return json.loads(path.read_text(encoding="utf-8"))

    def recover(self):
        with self._lock:
            if self._recovered:
                return
            self.root.mkdir(parents=True, exist_ok=True)
            for path in self.root.glob("*/manifest.json"):
                if path.parent.is_symlink() or path.is_symlink():
                    continue
                try:
                    record = self._read(path)
                    if record.get("state") == "recording":
                        record.update(state="incomplete", reason="driver restarted during recording")
                        atomic_json(path, record)
                except (ValueError, OSError):
                    continue
            self._recovered = True

    def create(self, name, address):
        if not isinstance(name, str) or not name.strip() or len(name) > 80 or any(ord(c) < 32 for c in name):
            raise ValueError("name must contain 1..80 printable characters")
        self.recover()
        with self._lock:
            identifier = uuid4().hex
            directory = self._directory(identifier)
            directory.mkdir()
            record = {"trajectory_id": identifier, "name": name.strip(), "version": 1,
                      "controller_ip": address, "robot_type": 113, "state": "recording",
                      "created_at_ms": int(time.time() * 1000), "sample_ms": SAMPLE_MS,
                      "point_count": 0, "duration_s": 0, "native_state": "format_required"}
            atomic_json(directory / "manifest.json", record)
            return record

    def load(self, identifier):
        self.recover()
        with self._lock:
            return self._read(self._directory(identifier) / "manifest.json")

    def save(self, record):
        with self._lock:
            atomic_json(self._directory(record["trajectory_id"]) / "manifest.json", record)

    def samples(self, identifier):
        record = self.load(identifier)
        if record["state"] != "recorded":
            raise RuntimeError(f"trajectory is not complete: {record.get('reason', record['state'])}")
        path = self._directory(identifier) / "samples.json"
        data = self._read(path)
        if hashlib.sha256(path.read_bytes()).hexdigest() != record.get("samples_sha256"):
            raise RuntimeError("recorded trajectory checksum mismatch")
        points = data.get("points")
        if not isinstance(points, list) or not 2 <= len(points) <= MAX_POINTS:
            raise ValueError("trajectory requires 2..10000 points")
        for index, point in enumerate(points):
            if point.get("time_ms") != index * SAMPLE_MS:
                raise ValueError("trajectory sampling times are not uniformly spaced")
            check_joints(point["joints_deg"])
        return data

    def list_records(self):
        self.recover()
        records = []
        with self._lock:
            for path in sorted(self.root.glob("*/manifest.json")):
                try:
                    record = self.load(path.parent.name)
                    records.append(self.summary(record))
                except (ValueError, OSError):
                    continue
        return records

    @staticmethod
    def summary(record):
        keys = ("trajectory_id", "name", "state", "end_reason", "sample_ms", "duration_s", "point_count",
                "native_state", "native_reason", "controller_file", "reason")
        return {key: record[key] for key in keys if key in record}

    def delete(self, identifier):
        record = self.load(identifier)
        if record["state"] == "recording":
            raise RuntimeError("stop recording before deleting it")
        with self._lock:
            directory = self._directory(identifier)
            # Only delete our own local artifacts. A controller copy is not ours
            # to remove while another controller program may reference it.
            for name in ("raw.jsonl", "samples.json", "manifest.json", "native.csv"):
                path = directory / name
                if path.is_symlink():
                    raise ValueError("trajectory files may not be symlinks")
            for name in ("raw.jsonl", "samples.json", "manifest.json", "native.csv"):
                (directory / name).unlink(missing_ok=True)
            directory.rmdir()
        return {"trajectory_id": identifier, "state": "deleted", "controller_copy_deleted": False}

    def export(self, identifier):
        """Publish via an already mounted controller trajectory share.

        No SSH credentials, implicit mount, guessed HTTP route or new SMB
        dependency. The deployment maps the operator's mounted share into us.
        """
        data = self.samples(identifier)
        record = self.load(identifier)
        if not self.profile_path:
            record.update(native_state="format_required")
            self.save(record)
            return record
        profile = NativeCsvProfile(self.profile_path)
        native = profile.encode(data)
        local = self._directory(identifier) / "native.csv"
        with local.open("wb") as file:
            file.write(native)
            file.flush()
            os.fsync(file.fileno())
        if self.controller_root is None or not self.controller_root.is_dir():
            record.update(native_state="transfer_required")
            self.save(record)
            return record
        writable_controller_directory(self.controller_root, require_network_mount=self.require_network_mount)
        filename = trajectory_filename(f"teach_{identifier}.csv")
        destination = self.controller_root / filename
        digest = hashlib.sha256(native).hexdigest()
        if destination.exists() or destination.is_symlink():
            if destination.is_symlink() or destination.stat().st_size > MAX_BYTES or destination.read_bytes() != native:
                raise RuntimeError("refusing to overwrite a different controller trajectory")
        else:
            # O_EXCL: do not overwrite another creator. A failed upload is never
            # reported ready, nor automatically replayed as a motion command.
            with destination.open("xb") as file:
                file.write(native)
                file.flush()
                os.fsync(file.fileno())
        if hashlib.sha256(destination.read_bytes()).hexdigest() != digest:
            raise RuntimeError("controller trajectory readback checksum mismatch")
        record.update(native_state="exported", controller_file=filename, native_sha256=digest,
                      native_format_id=profile.data["format_id"], profile_sha256=profile.digest)
        self.save(record)
        return record

    def playback(self, identifier, address):
        data = self.samples(identifier)
        record = self.load(identifier)
        if record["controller_ip"] != address:
            raise RuntimeError("trajectory belongs to a different controller")
        record = self.export(identifier)
        if record["native_state"] != "exported":
            raise RuntimeError(f"native playback unavailable: {record['native_state']}")
        return record, data


class TrajectoryRecorder:
    """One recording: wait for the first manual drag, then a continuous time axis
    across any number of drag (mode 6) and idle (mode 5) phases until stopped."""

    def __init__(self, store, record, subscription, on_finished, *, max_gap_ms=50, max_duration_s=500,
                 max_wait_s=None, clock=time.monotonic):
        self.store, self.record, self.subscription = store, record, subscription
        self.on_finished = on_finished
        self.max_gap_ms = max_gap_ms
        self.max_duration_s = max_duration_s
        self.max_wait_s = max_duration_s if max_wait_s is None else max_wait_s
        self._clock = clock
        self.stop_event = threading.Event()
        self.finished = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True, name="dobot-teach-recorder")
        self.reason = None
        self._reason_lock = threading.Lock()
        self._sealed = False
        self.limit_reached = False
        self.drag_entries = 0
        self._mode = None
        self._transition_since = None
        self._last_drag_ms = None
        self._points = []
        self._previous = None
        self._origin = None
        self._frames = None

    def start(self):
        self.thread.start()

    def stop(self, reason=None):
        """reason=None is a normal stop_record; any reason marks the take incomplete.
        An interruption is irrevocable: a later normal stop never clears it. Returns
        False once the outcome is sealed (the interruption arrived too late to apply)."""
        with self._reason_lock:
            if reason and self.reason is None and not self._sealed:
                self.reason = reason
            accepted = not self._sealed
        self.stop_event.set()
        return accepted

    def _seal(self, reason):
        """Final, synchronized check: merge any interruption received while exiting drag."""
        with self._reason_lock:
            self._sealed = True
            if self.reason and (reason is None or self.reason not in reason):
                return f"{reason}; {self.reason}" if reason else self.reason
            return reason

    def _consume(self, item):
        """Return the raw sample, or None while still waiting for the first drag."""
        state = item["data"]
        # Protection first: mode 0/5/6 never bypasses it.
        fault = protection_fault(state) or drag_state_fault(state)
        if fault:
            raise RuntimeError(f"recording interrupted: {fault}")
        timestamp = state["timestamp"]
        if type(timestamp) is not int or timestamp < 0:
            raise ValueError("invalid controller timestamp")
        mode = state["robot_mode"]
        if mode not in (0, 5, 6):
            raise RuntimeError(f"recording interrupted by controller mode {mode}")
        if self._mode == 0 and mode != 6 and mode != 0:
            raise RuntimeError(f"recording interrupted by controller mode 0: drag transition ended in mode {mode}")
        if mode == 0:
            # Only the short button-drag entry transition is accepted as mode 0.
            if not state["drag_status"]:
                raise RuntimeError("recording interrupted by controller mode 0 without drag status")
            if self._mode != 0:
                self._transition_since = timestamp
            if timestamp - self._transition_since > DRAG_ENTRY_TRANSITION_MS:
                raise RuntimeError(f"recording interrupted by controller mode 0 lasting over "
                                   f"{DRAG_ENTRY_TRANSITION_MS} ms")
        elif mode == 6:
            self._last_drag_ms = timestamp
        else:
            fault = interval_fault(state)
            # Leaving drag, the controller may report running_status for a packet.
            blip = (fault == "robot is running" and self._last_drag_ms is not None and
                    interval_fault({**state, "running_status": 0}) is None and
                    timestamp - self._last_drag_ms <= DRAG_EXIT_RUNNING_MS)
            if fault and not blip:
                raise RuntimeError(f"recording interrupted: {fault}")
        if mode in (0, 6) and self._mode not in (0, 6):
            self.drag_entries += 1
        self._mode = mode
        if self._previous is None and mode == 5:
            return None
        joints = check_joints(state["joint_state"]["q_actual"])
        frames = {"user": state["user_coordinate"], "tool": state["tool_coordinate"],
                  "user_value": state["user_value"], "tool_value": state["tool_value"]}
        if self._frames is not None and frames != self._frames:
            raise RuntimeError("user/tool frame changed during recording")
        if self._origin is None:
            self._origin, self._frames = timestamp, copy.deepcopy(frames)
        elapsed = timestamp - self._origin
        if self._previous is not None:
            gap = timestamp - self._previous["timestamp_ms"]
            if not 0 < gap <= self.max_gap_ms:
                raise RuntimeError("recording timestamps are discontinuous or feedback is missing")
        raw = {"timestamp_ms": timestamp, "time_ms": elapsed, "robot_mode": mode, "joints_deg": joints}
        if self._previous is None:
            self._points.append({"time_ms": 0, "joints_deg": joints})
        else:
            while len(self._points) < MAX_POINTS and len(self._points) * SAMPLE_MS <= elapsed:
                target_time = len(self._points) * SAMPLE_MS
                alpha = (target_time - self._previous["time_ms"]) / (elapsed - self._previous["time_ms"])
                values = [a + alpha * (b - a) for a, b in zip(self._previous["joints_deg"], joints)]
                self._points.append({"time_ms": target_time, "joints_deg": values})
        self._previous = raw
        # Limits count from the first drag and include every idle pause.
        if len(self._points) >= MAX_POINTS or elapsed >= self.max_duration_s * 1000:
            self.limit_reached = True
        return raw

    def _run(self):
        reason = None
        armed = last_packet = self._clock()
        directory = self.store._directory(self.record["trajectory_id"])
        try:
            with (directory / "raw.jsonl").open("x", encoding="utf-8") as file:
                while not self.stop_event.is_set():
                    try:
                        item = self.subscription.read(timeout=0.05)
                    except queue.Empty:
                        if self._clock() - last_packet > 0.25:
                            raise RuntimeError("feedback missing during recording")
                        continue
                    last_packet = self._clock()
                    if last_packet - item["received_at"] > 0.25:
                        raise RuntimeError("recording consumer fell behind feedback")
                    raw = self._consume(item)
                    if raw is None:
                        if last_packet - armed >= self.max_wait_s:
                            raise RuntimeError(f"no manual drag within {self.max_wait_s:g} s")
                        continue
                    file.write(json.dumps(raw, allow_nan=False, separators=(",", ":")) + "\n")
                    file.flush()
                    if file.tell() > MAX_BYTES:
                        raise RuntimeError("recording exceeds storage size limit")
                    if self.limit_reached:
                        break
                file.flush()
                os.fsync(file.fileno())
            reason = self.reason
        except Exception as exc:
            reason = str(exc)
        finally:
            self.subscription.close()
            if not self._points:
                reason = reason or "no manual drag was detected before the recording ended"
            elif len(self._points) < 2:
                reason = reason or "recording needs at least two samples (50 ms apart)"
            try:
                # Leave drag if still in it and confirm rest from post-stop feedback.
                cleanup_error = self.on_finished()
            except Exception as exc:
                cleanup_error = f"could not confirm exit from drag: {exc}"
            if cleanup_error:
                reason = f"{reason}; {cleanup_error}" if reason else cleanup_error
            # stop_motion/shutdown may arrive while the exit is being confirmed.
            reason = self._seal(reason)
            end_reason = "interrupted" if reason else ("limit_reached" if self.limit_reached else "stop_record")
            try:
                data = {"version": 1, "sample_ms": SAMPLE_MS, "frames": self._frames, "points": self._points}
                path = directory / "samples.json"
                atomic_json(path, data)
                self.record.update(state="incomplete" if reason else "recorded", reason=reason,
                                   end_reason=end_reason, drag_entries=self.drag_entries,
                                   point_count=len(self._points),
                                   duration_s=(len(self._points) - 1) * SAMPLE_MS / 1000 if self._points else 0,
                                   samples_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                self.store.save(self.record)
                if reason is None:
                    # Native CSV is independent: a local take stays recorded even if export fails.
                    try:
                        self.record = self.store.export(self.record["trajectory_id"])
                    except Exception as exc:
                        self.record.update(native_state="export_failed", native_reason=str(exc))
                        self.store.save(self.record)
            except Exception as exc:
                self.record.update(state="incomplete", end_reason="interrupted",
                                   reason=f"recording persistence failed: {exc}")
            finally:
                self.finished.set()
