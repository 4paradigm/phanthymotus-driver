"""Record glove frames from UDP into the hand_teach data directory.

Raw frames are appended to raw.jsonl in the same
line format as the desktop recorder, so a take goes through exactly the same
hand_trajectory conversion, resampling and safety rules as the built-in presets.

Layout (as the Dobot CRA TrajectoryStore of PR #492):
    <data_dir>/<sha256(name)[:32]>/manifest.json, raw.jsonl, samples.json
The directory is created with mkdir, which is the atomic duplicate-name check.
A take is playable once its manifest reaches the recorded state.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import socket
import threading
import time
from uuid import uuid4

import hand_trajectory
from teach_messages import error_text

SAVE_END_REASONS = {"stop_trajectory", "limit_reached"}

END_REASONS = {
    "stop_trajectory": "用户已停止录制",
    "gap": "手套数据断流，超过录制允许的帧间隔",
    "limit_reached": "录制达到时长上限",
    "interrupted": "录制被停止动作或驱动退出中止",
    "live_error": "实时遥操失败",
    "no_frames": "没有收到手套数据",
}


def directory_for(root, name):
    digest = hashlib.sha256(hand_trajectory.normalize_name(name).encode("utf-8")).hexdigest()
    return Path(root) / digest[:32]


def _write_manifest(directory, manifest):
    hand_trajectory._atomic_write(Path(directory) / "manifest.json", hand_trajectory.encode(manifest))


class RecordedStore:
    def __init__(self, root):
        self.root = Path(root)

    def find(self, name):
        directory = directory_for(self.root, name)
        if directory.is_symlink() or not (directory / "manifest.json").exists():
            return None
        return directory

    def create(self, name, recorded_at, *, side="right"):
        name = hand_trajectory.normalize_name(name)
        self.root.mkdir(parents=True, exist_ok=True)
        directory = directory_for(self.root, name)
        try:
            directory.mkdir()
        except FileExistsError:
            raise ValueError(f"轨迹「{name}」已存在，请换名或先删除原轨迹") from None
        try:
            _write_manifest(directory, {
                "version": 2, "name": name, "source": "recorded", "side": side,
                "source_format": hand_trajectory.SOURCE_FORMAT, "state": "recording", "recorded_at": recorded_at,
            })
        except Exception:
            shutil.rmtree(directory, ignore_errors=True)
            raise
        return directory

    def list(self):
        if not self.root.is_dir():
            return []
        return hand_trajectory.list_presets(self.root, source="recorded")

    def delete(self, name):
        directory = self.find(name)
        if directory is None:
            raise ValueError(f"找不到已录制轨迹「{hand_trajectory.normalize_name(name)}」")
        shutil.rmtree(directory)

    def recover(self):
        """A take still marked recording was cut off by a driver restart."""
        if not self.root.is_dir():
            return
        for directory in self.root.iterdir():
            if directory.is_symlink() or not directory.is_dir():
                continue
            path = directory / "manifest.json"
            if not path.exists():
                shutil.rmtree(directory, ignore_errors=True)  # Crashed between mkdir and manifest.
                continue
            try:
                manifest = json.loads(path.read_text(encoding="utf-8"))
            except (ValueError, OSError):
                continue
            if isinstance(manifest, dict) and manifest.get("state") == "recording":
                try:
                    shutil.rmtree(directory)
                except OSError as exc:
                    print(f"[hand_teach] 清理未保存的录制失败：{exc!r}", flush=True)


class Recorder:
    """One take: a local or UDP input thread that ends on stop(), a stream gap or the length limit."""

    def __init__(self, directory, name, *, port, max_s, gap_ms, on_end=None, live_control=None, host="0.0.0.0", local_packet=None, side="right"):
        self.directory = Path(directory)
        self.name = name
        self.recording_id = uuid4().hex
        self.side = side
        self.port, self.host = port, host
        self.local_packet = local_packet
        self.source = "local_glove" if local_packet is not None else f"udp:{port}"
        self.max_s, self.gap_s = max_s, gap_ms / 1000
        self.on_end = on_end
        self.live_control = live_control
        self.live_ready = threading.Event()
        self.live_port = None
        self.live_frames = 0
        self.live_error = None
        self.torque_released = None
        self.latest = None
        self.frames = []
        self.sender = None
        self.sender_host = None
        self.end_reason = None
        self.result = None
        self.invalid_frames = 0
        self.startup = threading.Event()
        self.first_frame = threading.Event()
        self.finished = threading.Event()
        self.stopped = threading.Event()
        self._lock = threading.Lock()
        self._socket = None
        self._thread = None

    def start(self):
        if self.local_packet is None:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind((self.host, self.port))
                sock.settimeout(0.02)
            except OSError:
                sock.close()
                raise
            self._socket = sock
        self._thread = threading.Thread(target=self._run, name=f"hand_teach_record_{self.port}", daemon=True)
        try:
            self._thread.start()
        except Exception:
            if self._socket is not None:
                self._socket.close()
            raise

    def stop(self, reason):
        """First reason wins; a take that already ended keeps its own reason."""
        with self._lock:
            if self.end_reason is None:
                self.end_reason = reason
        self.stopped.set()

    def fail_live(self, message):
        self.live_error = message
        self.stop("live_error")
        self.live_ready.set()

    def join(self, timeout=None):
        self.finished.wait(timeout)
        return self.result

    def _run(self):
        started_wall, started = time.time(), time.monotonic()
        last = None
        control = None
        try:
            if self.live_control is not None:
                control = threading.Thread(target=self.live_control, args=(self,), daemon=True,
                                           name=f"hand_teach_live_{self.port}")
                control.start()
            with (self.directory / "raw.jsonl").open("x", encoding="utf-8") as raw:
                while not self.stopped.is_set():
                    now = time.monotonic()
                    if last is not None and now - last > self.gap_s:
                        self.stop("gap")
                        break
                    if self.frames and now - started - self.frames[0][0] / 1000 > self.max_s:
                        self.stop("limit_reached")
                        break
                    if self.local_packet is not None:
                        packet = self.local_packet()
                        if packet is None or packet[0] < started or (last is not None and packet[0] <= last):
                            self.stopped.wait(0.005)
                            continue
                        received, data = packet
                        sender, host = "local_glove", None
                    else:
                        try:
                            data, address = self._socket.recvfrom(65535)
                        except socket.timeout:
                            continue
                        received = time.monotonic()
                        host = address[0]
                        sender = f"{host}:{address[1]}"
                        if self.sender is not None and host != self.sender_host:
                            continue  # Only the first glove host is recorded.
                    try:
                        text = data.decode("utf-8")
                    except UnicodeDecodeError:
                        self.invalid_frames += 1
                        continue
                    values = hand_trajectory.parse_stroke(text, self.side)
                    if values is None:
                        self.invalid_frames += 1
                        continue
                    if last is not None and received - last > self.gap_s:
                        self.stop("gap")
                        break
                    if self.sender is None:
                        self.sender, self.sender_host = sender, host
                    # Rounded as stored, so raw.jsonl rebuilds exactly the saved samples.
                    t_rel = round(received - started, 6)
                    raw.write(json.dumps({"ts": started_wall + t_rel, "t_rel": t_rel, "port": self.port,
                                          "from": sender, "raw": text}, ensure_ascii=False) + "\n")
                    self.frames.append((t_rel * 1000.0, values))
                    self.latest = (received, values)
                    last = received
                    self.first_frame.set()
                    self.startup.set()
                raw.flush()
        except Exception as exc:
            self.stop("interrupted")
            self.result = {"state": "error", "name": self.name, "reason": error_text(exc, "读取或写入录制数据失败，请查看驱动日志")}
        finally:
            self.stopped.set()
            self.startup.set()
            if self._socket is not None:
                self._socket.close()
            if control is not None and control.ident is not None:
                control.join()
            self.live_ready.set()
        try:
            if self.result is None:
                self.result = self._finish()
        except Exception as exc:
            self.result = {"state": "error", "name": self.name, "reason": "保存轨迹失败：" + error_text(exc, "请查看驱动日志")}
        if not self.result.get("valid"):
            problem = self.result.get("reason") or self.result["message"]
            try:
                shutil.rmtree(self.directory)
            except FileNotFoundError:
                deleted = True
            except OSError as exc:
                deleted = False
                problem += "；清理轨迹失败：" + error_text(exc, "请查看驱动日志")
            else:
                deleted = True
            suffix = "，轨迹已自动删除" if deleted else "，轨迹未能自动清理"
            self.result.setdefault("end_reason", self.end_reason or "interrupted")
            if self.live_control is not None:
                self.result.setdefault("torque_released", self.torque_released)
            self.result.update(valid=False, deleted=deleted, reason=problem,
                               message=f"录制失败：{problem}{suffix}")
        self.result["recording_id"] = self.recording_id
        self.finished.set()
        self.startup.set()
        if self.on_end is not None:
            self.on_end(self)

    def _finish(self):
        reason = self.end_reason or "interrupted"
        manifest = json.loads((self.directory / "manifest.json").read_text(encoding="utf-8"))
        if not self.frames:
            return {"state": "discarded", "name": self.name, "end_reason": reason,
                    "reason": self.live_error or "没有收到手套数据"}
        frames = self.frames
        base = {"frame_count": len(frames), "end_reason": reason, "sender": self.sender,
                "received_s": round((frames[-1][0] - frames[0][0]) / 1000, 3)}
        if self.live_control is not None:
            base.update(live_teleop=True, live_frames=self.live_frames, torque_released=self.torque_released)
        problem = self.live_error or (None if reason in SAVE_END_REASONS else END_REASONS[reason])
        if problem is None:
            try:
                built, samples = hand_trajectory.build_trajectory(
                    frames, name=self.name, source="recorded", side=self.side, source_recording=self.source,
                    recorded_at=manifest.get("recorded_at"))
            except ValueError as exc:
                problem = str(exc)
            else:
                written = hand_trajectory.write_trajectory(self.directory, {**built, **base}, samples)
                result = {**hand_trajectory.summary(written), **base, "valid": True}
                if reason == "limit_reached":
                    result["message"] = "录制达到时长上限，轨迹已自动保存"
                return result
        return {**hand_trajectory.summary({**manifest, "state": "discarded"}), **base,
                "reason": problem, "valid": False}
