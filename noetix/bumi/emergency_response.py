"""Bumi-specific emergency handoff primitive.

Stops the Bumi chassis, requests a fresh photo from ``vision_capture``, and
publishes an auditable handoff record. Classification, authentication,
external notification and case closure belong to a Skill/upstream service.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import threading
import time
import uuid


_HANDOFF_ID = re.compile(r"^handoff-[A-Za-z0-9-]{1,80}$")


def _now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def _text(value, name: str, maximum: int, *, required: bool = True) -> str:
    if value is None and not required:
        return ""
    if not isinstance(value, str):
        raise ValueError(f"{name} must be a string")
    value = value.strip()
    if (required and not value) or len(value) > maximum or any(ord(ch) < 32 for ch in value):
        raise ValueError(f"{name} must contain 1..{maximum} printable characters")
    return value


class HandoffJournal:
    """Append-only robot-local handoff journal with bounded reconstruction."""

    def __init__(self, path: str | Path, max_records: int = 200):
        self.path = Path(path)
        self.max_records = max(1, min(2000, int(max_records)))
        self._lock = threading.RLock()
        self._records: list[dict] = []
        self._load()

    def _load(self) -> None:
        if not self.path.exists():
            return
        try:
            lines = self.path.read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            raise RuntimeError(f"cannot read handoff journal: {exc}") from exc
        for line in lines[-self.max_records:]:
            try:
                value = json.loads(line)
                if value.get("schema") == "motus.bumi-emergency-handoff/1":
                    self._records.append(value)
            except (AttributeError, json.JSONDecodeError):
                continue

    def append(self, record: dict) -> dict:
        value = deepcopy(record)
        encoded = json.dumps(value, ensure_ascii=False, separators=(",", ":")) + "\n"
        with self._lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as output:
                output.write(encoded)
                output.flush()
            self._records.append(value)
            self._records = self._records[-self.max_records:]
        return deepcopy(value)

    def list(self, limit: int = 20) -> list[dict]:
        limit = max(1, min(100, int(limit)))
        with self._lock:
            return deepcopy(self._records[-limit:][::-1])

    def latest(self) -> dict | None:
        with self._lock:
            return deepcopy(self._records[-1]) if self._records else None


class EmergencyResponsePlugin:
    PREFIX = "emergency_response"

    def __init__(self, config: dict, namespace: str, executor,
                 loco_plugin=None, vision_capture_plugin=None):
        from rclpy.node import Node
        from std_msgs.msg import String

        output_dir = Path(str(config.get(
            "output_dir", "/opt/phanthy-motus/data/emergency_response")))
        self._journal = HandoffJournal(
            output_dir / "handoffs.jsonl", config.get("max_records", 200))
        self._loco = loco_plugin
        self._capture = vision_capture_plugin
        self._lock = threading.RLock()
        latest = self._journal.latest()
        self._active: dict | None = None
        if latest and latest.get("state") == "engaged":
            photo = latest.get("detail", {}).get("photo", {})
            self._active = {
                "handoff_id": latest["handoff_id"],
                "state": "engaged",
                "reason": latest.get("reason"),
                "source": latest.get("source"),
                "location": latest.get("location"),
                "engaged_at": latest.get("occurred_at"),
                "latest_photo": photo.get("file_path") if photo.get("ok") else None,
            }
        self._node = Node("bumi_emergency_response")
        self._topic = f"/{namespace}/emergency/handoff"
        self._String = String
        self._publisher = self._node.create_publisher(String, self._topic, 10)
        executor.add_node(self._node)

    def get_tool(self) -> dict:
        return {
            "name": self.PREFIX, "type": "actuator", "multiInstance": False,
            "description": (
                "Bumi 本机应急交接基础能力：engage 先停止底盘，再通过现有 vision_capture "
                "补拍新鲜现场照片并发布交接记录；snapshot 可补拍；release 只释放交接标记。"
                "这不是安全认证急停，也不负责事件分类、用户认证、外部推送或呼叫安保。"),
            "inputSchema": {
                "type": "object", "required": ["action"],
                "properties": {
                    "action": {"type": "string", "enum": [
                        "start", "stop", "engage", "snapshot", "release",
                        "info", "history"]},
                    "reason": {"type": "string", "description": "交接原因；由上游保留原始来源"},
                    "source": {"type": "string", "enum": [
                        "manual", "vision", "audio", "system", "skill"]},
                    "location": {"type": "string", "description": "可选现场位置标签"},
                    "handoff_id": {"type": "string"},
                    "confirm": {"type": "boolean", "description": "release 必须显式为 true"},
                    "limit": {"type": "integer", "minimum": 1, "maximum": 100, "default": 20}},
                "additionalProperties": False,
                "x-action-params": {
                    "start": {"params": []}, "stop": {"params": []},
                    "engage": {"params": ["reason", "source", "location"],
                               "required": ["reason", "source"]},
                    "snapshot": {"params": ["handoff_id"], "required": ["handoff_id"]},
                    "release": {"params": ["handoff_id", "confirm"],
                                "required": ["handoff_id", "confirm"]},
                    "info": {"params": []}, "history": {"params": ["limit"]}},
                "x-resource": ["base", "camera"]},
            "topic_out": [{"topic": self._topic, "format": "data/json",
                           "desc": "Bumi 底盘停止、现场照片和交接状态记录"}]}

    def start(self):
        return {"state": "ready", "topic_out": self.get_tool()["topic_out"]}

    def stop(self):
        return {"state": "idle"}

    def _publish(self, record: dict) -> None:
        message = self._String()
        message.data = json.dumps(record, ensure_ascii=False, separators=(",", ":"))
        self._publisher.publish(message)

    @staticmethod
    def _id(value) -> str:
        value = _text(value, "handoff_id", 88)
        if not _HANDOFF_ID.fullmatch(value):
            raise ValueError("invalid handoff_id")
        return value

    def _photo(self) -> dict:
        if self._capture is None:
            return {"ok": False, "code": "CAPTURE_UNAVAILABLE"}
        result = self._capture.dispatch("capture_photo", {})
        return result if isinstance(result, dict) else {
            "ok": False, "code": "CAPTURE_INVALID_RESPONSE"}

    def _record(self, action: str, active: dict, detail: dict) -> dict:
        record = {"schema": "motus.bumi-emergency-handoff/1",
                  "handoff_id": active["handoff_id"], "action": action,
                  "occurred_at": _now(), "state": active["state"],
                  "reason": active["reason"], "source": active["source"],
                  "location": active["location"], "detail": deepcopy(detail)}
        record = self._journal.append(record)
        self._publish(record)
        return record

    def dispatch(self, action: str, args: dict) -> dict | None:
        try:
            if action == "start":
                return {"state": "ready"}
            if action == "stop":
                return {"state": "idle"}
            if action == "engage":
                reason = _text(args.get("reason"), "reason", 500)
                source = args.get("source")
                if source not in {"manual", "vision", "audio", "system", "skill"}:
                    raise ValueError("unsupported source")
                location = _text(args.get("location"), "location", 160, required=False)
                with self._lock:
                    if self._active is not None:
                        return {"state": "error", "error": "handoff_already_active",
                                "active": deepcopy(self._active)}
                    motion = (self._loco.dispatch(
                        "stop_move", {"_tool_name": "loco"}) if self._loco is not None
                              else {"state": "unavailable"})
                    if not isinstance(motion, dict):
                        motion = {"state": "error", "error": "loco_stop_not_dispatched"}
                    photo = self._photo()
                    active = {"handoff_id": (
                                  f"handoff-{int(time.time() * 1000)}-{uuid.uuid4().hex[:8]}"),
                              "state": "engaged", "reason": reason, "source": source,
                              "location": location or None, "engaged_at": _now(),
                              "latest_photo": photo.get("file_path") if photo.get("ok") else None}
                    self._active = active
                    record = self._record("engage", active, {
                        "motion_stop": motion, "photo": photo})
                return {"state": "engaged", "handoff": deepcopy(active),
                        "motion_stop": motion, "photo": photo, "record": record}

            if action == "snapshot":
                handoff_id = self._id(args.get("handoff_id"))
                with self._lock:
                    if self._active is None or self._active["handoff_id"] != handoff_id:
                        return {"state": "error", "error": "handoff_not_active",
                                "handoff_id": handoff_id}
                    photo = self._photo()
                    if photo.get("ok"):
                        self._active["latest_photo"] = photo.get("file_path")
                    record = self._record("snapshot", self._active, {"photo": photo})
                return {"state": "engaged", "photo": photo, "record": record}

            if action == "release":
                handoff_id = self._id(args.get("handoff_id"))
                if args.get("confirm") is not True:
                    return {"state": "error", "error": "confirm_true_required"}
                with self._lock:
                    if self._active is None or self._active["handoff_id"] != handoff_id:
                        return {"state": "error", "error": "handoff_not_active",
                                "handoff_id": handoff_id}
                    released = deepcopy(self._active)
                    released["state"] = "released"
                    record = self._record("release", released, {})
                    self._active = None
                return {"state": "released", "handoff": released, "record": record,
                        "note": "release does not start or resume robot motion"}

            if action == "info":
                with self._lock:
                    active = deepcopy(self._active)
                return {"state": "engaged" if active else "ready", "active": active,
                        "journal_path": str(self._journal.path),
                        "motion_stop_available": self._loco is not None,
                        "photo_available": self._capture is not None,
                        "safety_rated_estop": False}
            if action == "history":
                return {"state": "ok", "records": self._journal.list(args.get("limit", 20))}
        except (OSError, RuntimeError, TypeError, ValueError) as exc:
            return {"state": "error", "error": "invalid_or_unavailable", "detail": str(exc)}
        return None
