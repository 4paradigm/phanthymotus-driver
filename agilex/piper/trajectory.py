"""Bounded, non-blocking trajectory recording and strict offline replay validation."""
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import stat
import threading
import time
from uuid import uuid4

SCHEMA = "motus.piper.trajectory/1"
MAX_BYTES = 64 * 1024 * 1024
MAX_FRAMES = 12000
MAX_DURATION_NS = 600_000_000_000
MAX_LINE = 16384
NAME = re.compile(r"[A-Za-z0-9_-]{1,96}\.jsonl")


def unique(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("duplicate_json_field")
        result[key] = value
    return result


def encode(value):
    line = (json.dumps(value, allow_nan=False, separators=(",", ":")) + "\n").encode()
    if len(line) > MAX_LINE:
        raise ValueError("trajectory_line_too_large")
    return line


def numbers(value, count):
    if (not isinstance(value, list) or len(value) != count or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
        raise ValueError("invalid_trajectory_vector")


def validate_frame(frame, previous=None):
    expected = {"type", "t_ns", "target_q", "target_width_mm", "feedback_q", "feedback_width_mm",
                "feedback_time", "send_ok", "send_offset_ns"}
    if not isinstance(frame, dict) or set(frame) != expected or frame["type"] != "frame":
        raise ValueError("invalid_trajectory_frame")
    t = frame["t_ns"]
    if type(t) is not int or not 0 <= t <= MAX_DURATION_NS or previous is not None and t <= previous["t_ns"]:
        raise ValueError("invalid_trajectory_time")
    for key, count in (("target_q", 12), ("feedback_q", 12), ("target_width_mm", 2),
                       ("feedback_width_mm", 2), ("feedback_time", 4)):
        numbers(frame[key], count)
    if frame["send_ok"] != [True, True] or any(type(v) is not bool for v in frame["send_ok"]):
        raise ValueError("partial_pair_command")
    offsets = frame["send_offset_ns"]
    if (not isinstance(offsets, list) or len(offsets) != 2 or
            any(type(v) is not int or not 0 <= v <= 50_000_000 for v in offsets) or offsets[0] > offsets[1]):
        raise ValueError("invalid_pair_send_timing")
    if previous is not None:
        dt = (t - previous["t_ns"]) / 1e9
        if dt < .02:
            raise ValueError("trajectory_rate_exceeded")
        if dt > .3:
            raise ValueError("trajectory_gap_exceeds_watchdog")
        # SDK rounds joints to 0.001 degrees and width to 0.001 mm.
        if any(abs(a-b) > math.radians(1)*dt + math.radians(.002)
               for a, b in zip(frame["target_q"], previous["target_q"])):
            raise ValueError("trajectory_joint_speed_exceeded")
        if any(abs(a-b) > 10*dt + .002 for a, b in zip(frame["target_width_mm"], previous["target_width_mm"])):
            raise ValueError("trajectory_gripper_speed_exceeded")


def read_recording(directory, filename, binding):
    if not isinstance(filename, str) or not NAME.fullmatch(filename):
        raise ValueError("invalid_recording_filename")
    path = Path(directory) / filename
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, "rb") as stream:
        metadata = os.fstat(stream.fileno())
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_size > MAX_BYTES:
            raise ValueError("invalid_recording_file")
        digest = hashlib.sha256()
        frames, header, footer = [], None, None
        total = 0
        for raw in iter(lambda: stream.readline(MAX_LINE + 1), b""):
            total += len(raw)
            if total > MAX_BYTES or len(raw) > MAX_LINE or not raw.endswith(b"\n"):
                raise ValueError("invalid_recording_size_or_truncation")
            value = json.loads(raw, object_pairs_hook=unique)
            if not isinstance(value, dict) or footer is not None:
                raise ValueError("invalid_recording_order")
            if header is None:
                if (set(value) != {"type", "schema", "binding", "units"} or value["type"] != "header"
                        or value["schema"] != SCHEMA or value["binding"] != binding
                        or value["units"] != {"q": "rad", "width": "mm", "time": "ns", "feedback_time": "unix_seconds"}):
                    raise ValueError("recording_binding_or_schema_mismatch")
                header = value
            elif value.get("type") == "complete":
                if (set(value) != {"type", "frames", "sha256"} or type(value["frames"]) is not int
                        or value["frames"] != len(frames) or value["sha256"] != digest.hexdigest() or len(frames) < 2):
                    raise ValueError("recording_checksum_or_count_mismatch")
                footer = value
                continue
            else:
                if len(frames) >= MAX_FRAMES:
                    raise ValueError("recording_too_long")
                validate_frame(value, frames[-1] if frames else None)
                frames.append(value)
            digest.update(raw)
        if footer is None:
            raise ValueError("recording_incomplete")
    return {"filename": filename, "sha256": footer["sha256"], "header": header, "frames": frames,
            "duration_seconds": (frames[-1]["t_ns"] - frames[0]["t_ns"])/1e9}


class Recorder:
    def __init__(self, directory, binding):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.filename = time.strftime("motion-%Y%m%d-%H%M%S-") + uuid4().hex[:12] + ".jsonl"
        self.partial = self.directory / (self.filename + ".partial")
        self.binding = binding
        self.queue = queue.Queue(maxsize=128)
        self.stopping = threading.Event()
        self.ready = threading.Event()
        self.error = None
        self.count = 0
        self.complete = False
        self.origin_ns = None
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._write, name="piper-recorder", daemon=True)
        self.thread.start()
        if not self.ready.wait(2):
            self.fail("recording_open_timeout")
        if self.error:
            raise RuntimeError(self.error)

    def fail(self, reason):
        with self.lock:
            self.error = self.error or str(reason)
            self.stopping.set()

    def append(self, frame):
        with self.lock:
            if self.stopping.is_set():
                return False
            frame = dict(frame)
            if self.origin_ns is None:
                self.origin_ns = frame["t_ns"]
            frame["t_ns"] -= self.origin_ns
            try:
                self.queue.put_nowait(encode(frame))
            except (queue.Full, ValueError) as exc:
                self.error = "recording_queue_full" if isinstance(exc, queue.Full) else str(exc)
                self.stopping.set()
                return False
        return True

    def _write(self):
        try:
            with self.partial.open("xb") as stream:
                os.chmod(self.partial, 0o600)
                header = encode({"type": "header", "schema": SCHEMA, "binding": self.binding,
                                 "units": {"q": "rad", "width": "mm", "time": "ns", "feedback_time": "unix_seconds"}})
                stream.write(header)
                stream.flush()
                self.ready.set()
                digest, total, previous = hashlib.sha256(header), len(header), None
                while not self.stopping.is_set() or not self.queue.empty():
                    if self.error:
                        break
                    try:
                        raw = self.queue.get(timeout=.05)
                    except queue.Empty:
                        continue
                    frame = json.loads(raw)
                    validate_frame(frame, previous)
                    if self.count >= MAX_FRAMES or total + len(raw) + MAX_LINE > MAX_BYTES:
                        raise ValueError("recording_limit_reached")
                    stream.write(raw)
                    digest.update(raw)
                    total += len(raw)
                    self.count += 1
                    previous = frame
                if not self.error:
                    if self.count < 2:
                        raise ValueError("recording_needs_two_frames")
                    stream.write(encode({"type": "complete", "frames": self.count, "sha256": digest.hexdigest()}))
                    stream.flush()
                    os.fsync(stream.fileno())
            if not self.error:
                self.partial.rename(self.directory / self.filename)
                self.complete = True
        except Exception as exc:
            self.fail(str(exc))
        finally:
            self.ready.set()

    def stop(self, timeout=2):
        with self.lock:
            self.stopping.set()
        self.thread.join(timeout)
        return self.status()

    def status(self):
        return {"filename": self.filename if self.complete else self.partial.name,
                "frames": self.count, "complete": self.complete, "error": self.error,
                "recording": not self.stopping.is_set() and self.thread.is_alive(),
                "stopping": self.stopping.is_set() and self.thread.is_alive()}


def list_recordings(directory):
    root = Path(directory)
    if not root.exists():
        return []
    files = []
    for path in sorted(root.iterdir()):
        if (NAME.fullmatch(path.name) or path.name.endswith(".jsonl.partial")) and not path.is_symlink() and path.is_file():
            files.append({"filename": path.name, "bytes": path.stat().st_size,
                          "complete": not path.name.endswith(".partial")})
    return files
