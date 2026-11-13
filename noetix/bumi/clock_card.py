"""Bumi clock and reminder card.

Local time queries, countdowns and scheduled reminders.  Reminders are kept in
an on-disk JSON store so they survive a driver restart, and a background thread
publishes due reminders as JSON events on ``/<namespace>/clock/reminder``.

The card is self-contained: it needs neither the Noetix SDK nor the camera, so
it stays usable on a robot whose other Cards are unavailable.
"""

from __future__ import annotations

import json
import threading
import time
import uuid
from datetime import datetime, timedelta, time as dtime
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String


STORE_PATH = Path("/opt/phanthy-motus/data/bumi/reminders.json")
TICK_INTERVAL_S = 0.5
MAX_PENDING = 200
# Fired one-shot history is useful for "did it remind me?", but must not grow
# without bound on a robot that runs for months.
MAX_FINISHED = 100
MAX_CONTENT_LENGTH = 500
_STORE_VERSION = 1

_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

WEEKDAY_ZH = ("星期一", "星期二", "星期三", "星期四", "星期五", "星期六", "星期日")
WEEKDAY_EN = ("Monday", "Tuesday", "Wednesday", "Thursday", "Friday",
              "Saturday", "Sunday")

_WEEKDAY_ALIASES = {
    "mon": 1, "monday": 1, "周一": 1, "星期一": 1,
    "tue": 2, "tues": 2, "tuesday": 2, "周二": 2, "星期二": 2,
    "wed": 3, "wednesday": 3, "周三": 3, "星期三": 3,
    "thu": 4, "thur": 4, "thurs": 4, "thursday": 4, "周四": 4, "星期四": 4,
    "fri": 5, "friday": 5, "周五": 5, "星期五": 5,
    "sat": 6, "saturday": 6, "周六": 6, "星期六": 6,
    "sun": 7, "sunday": 7, "周日": 7, "周天": 7, "星期日": 7, "星期天": 7,
}

_REPEAT_ALIASES = {
    "none": "none", "once": "none", "one-shot": "none", "oneshot": "none",
    "一次": "none", "不重复": "none", "单次": "none",
    "daily": "daily", "day": "daily", "每天": "daily", "每日": "daily",
    "weekly": "weekly", "week": "weekly", "每周": "weekly", "每星期": "weekly",
}

_ACTIONS = ("now", "weekday", "until", "countdown", "create", "list",
            "cancel", "update", "info", "start", "stop")


# ── Time helpers ─────────────────────────────────────────────────────────────

def _iso(value: datetime | None) -> str | None:
    return value.isoformat(timespec="seconds") if value is not None else None


def _parse_time_value(value) -> datetime | None:
    """Parse an ISO-8601 / ``YYYY-MM-DD HH:MM[:SS]`` string; naive = local."""
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text:
        return None
    if text.endswith(("Z", "z")):
        text = text[:-1] + "+00:00"
    candidates = (text, text.replace("/", "-"))
    parsed = None
    for candidate in candidates:
        try:
            parsed = datetime.fromisoformat(candidate)
            break
        except ValueError:
            continue
    if parsed is None:
        for fmt in ("%Y/%m/%d %H:%M:%S", "%Y/%m/%d %H:%M",
                    "%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M"):
            try:
                parsed = datetime.strptime(text, fmt)
                break
            except ValueError:
                continue
    if parsed is None:
        return None
    if parsed.tzinfo is None:
        # Attach the robot's local timezone; a bare timestamp means local time.
        parsed = parsed.astimezone()
    return parsed


def _parse_time_of_day(value) -> tuple[int, int, int] | None:
    if not isinstance(value, str):
        return None
    text = value.strip().replace("：", ":")
    if not text:
        return None
    parts = text.split(":")
    if not 2 <= len(parts) <= 3 or not all(part.isdigit() for part in parts):
        return None
    hour, minute = int(parts[0]), int(parts[1])
    second = int(parts[2]) if len(parts) == 3 else 0
    if not (0 <= hour < 24 and 0 <= minute < 60 and 0 <= second < 60):
        return None
    return hour, minute, second


def _format_time_of_day(tod: tuple[int, int, int] | None) -> str | None:
    return "{:02d}:{:02d}:{:02d}".format(*tod) if tod else None


def _parse_weekday(value) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value if 1 <= value <= 7 else None
    if isinstance(value, str):
        text = value.strip().lower()
        if text.isdigit():
            number = int(text)
            return number if 1 <= number <= 7 else None
        return _WEEKDAY_ALIASES.get(text)
    return None


def _normalize_repeat(value) -> str | None:
    if value is None or value == "":
        return None
    if not isinstance(value, str):
        return None
    return _REPEAT_ALIASES.get(value.strip().lower())


def _weekday_payload(index: int) -> dict:
    return {
        "index": index,
        "name_en": WEEKDAY_EN[index - 1],
        "name_zh": WEEKDAY_ZH[index - 1],
    }


def _duration_payload(seconds: float) -> dict:
    """Decompose a signed interval and render a short Chinese phrase."""
    in_past = seconds < 0
    total = abs(int(round(seconds)))
    days, remainder = divmod(total, 86400)
    hours, remainder = divmod(remainder, 3600)
    minutes, secs = divmod(remainder, 60)
    parts = []
    if days:
        parts.append(f"{days}天")
    if days or hours:
        parts.append(f"{hours}小时")
    if days or hours or minutes:
        parts.append(f"{minutes}分钟")
    if secs or not parts:
        parts.append(f"{secs}秒")
    phrase = "".join(parts)
    return {
        "total_seconds": round(seconds, 3),
        "in_past": in_past,
        "days": days,
        "hours": hours,
        "minutes": minutes,
        "seconds": secs,
        "text": f"{phrase}前" if in_past else f"{phrase}后",
    }


def _now_payload(now: datetime) -> dict:
    return {
        "state": "ok",
        "iso": _iso(now),
        "timestamp_s": int(now.timestamp()),
        "timezone": now.tzname() or str(now.tzinfo),
        "utc_offset": now.strftime("%z"),
        "date": now.strftime("%Y-%m-%d"),
        "time": now.strftime("%H:%M:%S"),
        "year": now.year,
        "month": now.month,
        "day": now.day,
        "hour": now.hour,
        "minute": now.minute,
        "second": now.second,
        "weekday": _weekday_payload(now.isoweekday()),
    }


def _next_daily(now: datetime, tod: tuple[int, int, int]) -> datetime:
    candidate = datetime.combine(now.date(), dtime(*tod), tzinfo=now.tzinfo)
    if candidate <= now:
        candidate = datetime.combine(
            now.date() + timedelta(days=1), dtime(*tod), tzinfo=now.tzinfo)
    return candidate


def _next_weekly(now: datetime, weekday: int,
                 tod: tuple[int, int, int]) -> datetime:
    days_ahead = (weekday - now.isoweekday()) % 7
    candidate = datetime.combine(
        now.date() + timedelta(days=days_ahead), dtime(*tod), tzinfo=now.tzinfo)
    if candidate <= now:
        candidate = candidate + timedelta(days=7)
    return candidate


def _error(state: str, message: str) -> dict:
    return {"state": state, "message": message}


# ── Plugin ───────────────────────────────────────────────────────────────────

class ClockPlugin:
    PREFIX = "clock"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._output_topic = f"/{namespace}/clock/reminder"
        store_path = plugin_config.get("store_path")
        self._store_path = Path(store_path) if store_path else STORE_PATH
        self._node = Node("bumi_clock")
        executor.add_node(self._node)
        self._publisher = self._node.create_publisher(String, self._output_topic, _QOS)
        self._lock = threading.RLock()
        self._reminders: list[dict] = []
        self._loaded = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # ── Clock injection point (tests freeze time by overriding this) ──────

    def _now(self) -> datetime:
        return datetime.now().astimezone()

    # ── Persistence ───────────────────────────────────────────────────────

    def _ensure_loaded(self) -> None:
        if self._loaded:
            return
        self._loaded = True
        try:
            raw = json.loads(self._store_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except (OSError, ValueError) as exc:
            print(f"[clock] ignoring unreadable reminder store: {exc}", flush=True)
            return
        items = raw.get("reminders") if isinstance(raw, dict) else None
        if not isinstance(items, list):
            return
        for item in items:
            reminder = self._deserialize(item)
            if (reminder is not None
                    and len(self._reminders) < MAX_PENDING + MAX_FINISHED):
                self._reminders.append(reminder)
        self._prune_locked()

    def _prune_locked(self) -> bool:
        """Keep only the most recent fired reminders."""
        finished = [reminder for reminder in self._reminders
                    if reminder["state"] != "pending"]
        if len(finished) <= MAX_FINISHED:
            return False
        finished.sort(key=lambda reminder:
                      reminder.get("fired_at") or reminder["created_at"])
        drop = {id(reminder) for reminder in finished[:len(finished) - MAX_FINISHED]}
        self._reminders = [reminder for reminder in self._reminders
                           if id(reminder) not in drop]
        return True

    def _deserialize(self, item) -> dict | None:
        if not isinstance(item, dict):
            return None
        rid = item.get("id")
        content = item.get("content")
        if not isinstance(rid, str) or not rid or not isinstance(content, str):
            return None
        repeat = _normalize_repeat(item.get("repeat")) or "none"
        state = item.get("state")
        if state not in ("pending", "fired"):
            state = "fired"
        next_due = _parse_time_value(item.get("next_due"))
        if state == "pending" and next_due is None:
            return None
        created_at = _parse_time_value(item.get("created_at")) or self._now()
        tod = _parse_time_of_day(item.get("time_of_day"))
        if tod is None and next_due is not None:
            tod = (next_due.hour, next_due.minute, next_due.second)
        weekday = item.get("weekday")
        if not isinstance(weekday, int) or not 1 <= weekday <= 7:
            weekday = None
        if repeat == "weekly" and weekday is None and next_due is not None:
            weekday = next_due.isoweekday()
        if repeat == "weekly" and weekday is None:
            return None
        if repeat == "daily" and tod is None:
            return None
        fired_count = item.get("fired_count")
        return {
            "id": rid,
            "content": content,
            "repeat": repeat,
            "time_of_day": tod,
            "weekday": weekday,
            "state": state,
            "created_at": created_at,
            "updated_at": _parse_time_value(item.get("updated_at")) or created_at,
            "next_due": next_due,
            "fired_at": _parse_time_value(item.get("fired_at")),
            "fired_count": fired_count if isinstance(fired_count, int) and fired_count >= 0 else 0,
        }

    def _serialize(self, reminder: dict) -> dict:
        return {
            "id": reminder["id"],
            "content": reminder["content"],
            "repeat": reminder["repeat"],
            "time_of_day": _format_time_of_day(reminder["time_of_day"]),
            "weekday": reminder["weekday"],
            "state": reminder["state"],
            "created_at": _iso(reminder["created_at"]),
            "updated_at": _iso(reminder.get("updated_at")),
            "next_due": _iso(reminder["next_due"]),
            "fired_at": _iso(reminder.get("fired_at")),
            "fired_count": reminder.get("fired_count", 0),
        }

    def _save_locked(self) -> None:
        payload = {
            "version": _STORE_VERSION,
            "saved_at": _iso(self._now()),
            "reminders": [self._serialize(reminder) for reminder in self._reminders],
        }
        temporary = self._store_path.with_name(self._store_path.name + ".tmp")
        try:
            self._store_path.parent.mkdir(parents=True, exist_ok=True)
            temporary.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8")
            temporary.replace(self._store_path)
        except OSError as exc:
            print(f"[clock] failed to save reminders: {exc}", flush=True)

    # ── Reminder views ────────────────────────────────────────────────────

    @staticmethod
    def _public(reminder: dict) -> dict:
        return {
            "id": reminder["id"],
            "content": reminder["content"],
            "repeat": reminder["repeat"],
            "state": reminder["state"],
            "time_of_day": _format_time_of_day(reminder["time_of_day"]),
            "weekday": (_weekday_payload(reminder["weekday"])
                        if reminder["weekday"] else None),
            "created_at": _iso(reminder["created_at"]),
            "updated_at": _iso(reminder.get("updated_at")),
            "next_due": _iso(reminder["next_due"]),
            "fired_at": _iso(reminder.get("fired_at")),
            "fired_count": reminder.get("fired_count", 0),
        }

    def _find_locked(self, reminder_id) -> dict | None:
        if not isinstance(reminder_id, str):
            return None
        for reminder in self._reminders:
            if reminder["id"] == reminder_id:
                return reminder
        return None

    def _new_id_locked(self) -> str:
        existing = {reminder["id"] for reminder in self._reminders}
        while True:
            candidate = "rmd_" + uuid.uuid4().hex[:8]
            if candidate not in existing:
                return candidate

    # ── Scheduling ────────────────────────────────────────────────────────

    def _build_schedule(self, args: dict, now: datetime,
                        existing: dict | None) -> tuple[dict | None, dict | None]:
        """Resolve repeat/next_due from create/update arguments."""
        raw_repeat = args.get("repeat")
        if raw_repeat not in (None, ""):
            repeat = _normalize_repeat(raw_repeat)
            if repeat is None:
                return None, _error(
                    "invalid_parameters",
                    "repeat must be one of: none, daily, weekly")
        else:
            repeat = existing["repeat"] if existing else "none"

        raw_time = args.get("time")
        parsed_time = None
        if raw_time not in (None, ""):
            parsed_time = _parse_time_value(raw_time)
            if parsed_time is None:
                return None, _error(
                    "invalid_parameters",
                    "time must be an ISO-8601 timestamp, "
                    "e.g. 2026-11-08T09:30:00")

        raw_tod = args.get("time_of_day")
        tod = _parse_time_of_day(raw_tod) if raw_tod not in (None, "") else None
        if raw_tod not in (None, "") and tod is None:
            return None, _error(
                "invalid_parameters", "time_of_day must look like HH:MM or HH:MM:SS")
        if tod is None and parsed_time is not None:
            tod = (parsed_time.hour, parsed_time.minute, parsed_time.second)

        raw_weekday = args.get("weekday")
        weekday = _parse_weekday(raw_weekday) if raw_weekday not in (None, "") else None
        if raw_weekday not in (None, "") and weekday is None:
            return None, _error(
                "invalid_parameters",
                "weekday must be 1-7 (Monday=1) or a weekday name")
        if weekday is None and parsed_time is not None:
            weekday = parsed_time.isoweekday()

        if repeat == "none":
            if parsed_time is not None:
                if parsed_time <= now:
                    return None, _error(
                        "time_in_past", "a one-shot reminder must be in the future")
                due = parsed_time
                tod = (due.hour, due.minute, due.second)
            elif existing is not None and existing["repeat"] == "none" \
                    and existing["next_due"] is not None:
                due = existing["next_due"]
                tod = existing["time_of_day"]
            else:
                return None, _error(
                    "invalid_parameters",
                    "time is required for a one-shot reminder")
            weekday = None
        elif repeat == "daily":
            if tod is None and existing is not None:
                tod = existing["time_of_day"]
            if tod is None:
                return None, _error(
                    "invalid_parameters",
                    "time_of_day is required for a daily reminder")
            due = _next_daily(now, tod)
            weekday = None
        else:  # weekly
            if weekday is None and existing is not None:
                weekday = existing["weekday"]
            if weekday is None:
                return None, _error(
                    "invalid_parameters",
                    "weekday is required for a weekly reminder")
            if tod is None and existing is not None:
                tod = existing["time_of_day"]
            if tod is None:
                return None, _error(
                    "invalid_parameters",
                    "time_of_day is required for a weekly reminder")
            due = _next_weekly(now, weekday, tod)

        return {
            "repeat": repeat,
            "time_of_day": tod,
            "weekday": weekday,
            "next_due": due,
        }, None

    def _advance_locked(self, reminder: dict, now: datetime) -> datetime:
        if reminder["repeat"] == "daily":
            return _next_daily(now, reminder["time_of_day"])
        return _next_weekly(now, reminder["weekday"], reminder["time_of_day"])

    # ── Scheduler ─────────────────────────────────────────────────────────

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._tick()
            except Exception as exc:  # keep the scheduler alive
                print(f"[clock] scheduler error: {exc}", flush=True)
            if self._stop_event.wait(TICK_INTERVAL_S):
                break

    def _tick(self) -> list[dict]:
        """Fire every due reminder and publish it as one JSON event."""
        now = self._now()
        events: list[dict] = []
        with self._lock:
            self._ensure_loaded()
            changed = False
            for reminder in self._reminders:
                if reminder["state"] != "pending":
                    continue
                due = reminder["next_due"]
                if due is None or due > now:
                    continue
                late_seconds = (now - due).total_seconds()
                reminder["fired_count"] += 1
                reminder["fired_at"] = now
                events.append({
                    "type": "reminder",
                    "id": reminder["id"],
                    "content": reminder["content"],
                    "repeat": reminder["repeat"],
                    "due_at": _iso(due),
                    "fired_at": _iso(now),
                    "late_seconds": round(late_seconds, 3),
                    "fired_count": reminder["fired_count"],
                    "timestamp_ms": int(now.timestamp() * 1000),
                })
                if reminder["repeat"] in ("daily", "weekly"):
                    # One event per due reminder, even after a long outage:
                    # reschedule to the next occurrence strictly after now.
                    reminder["next_due"] = self._advance_locked(reminder, now)
                    reminder["updated_at"] = now
                else:
                    reminder["state"] = "fired"
                    reminder["next_due"] = None
                changed = True
            if changed:
                self._prune_locked()
                self._save_locked()
        for event in events:
            try:
                message = String()
                message.data = json.dumps(event, ensure_ascii=False)
                self._publisher.publish(message)
            except Exception as exc:
                print(f"[clock] failed to publish reminder: {exc}", flush=True)
        return events

    # ── Actions ───────────────────────────────────────────────────────────

    def _action_until(self, args: dict, now: datetime) -> dict:
        target = _parse_time_value(args.get("time"))
        if target is None:
            return _error("invalid_parameters",
                          "time must be an ISO-8601 timestamp")
        result = {"state": "ok", "now": _iso(now), "target": _iso(target)}
        result.update(_duration_payload((target - now).total_seconds()))
        return result

    def _action_countdown(self, args: dict, now: datetime) -> dict:
        reminder_id = args.get("id")
        if reminder_id not in (None, ""):
            with self._lock:
                self._ensure_loaded()
                reminder = self._find_locked(reminder_id)
                if reminder is None:
                    return _error("not_found", f"reminder {reminder_id!r} does not exist")
                if reminder["state"] != "pending" or reminder["next_due"] is None:
                    return _error("not_pending",
                                  f"reminder {reminder_id!r} is not scheduled")
                target = reminder["next_due"]
            result = {
                "state": "ok",
                "reminder_id": reminder["id"],
                "content": reminder["content"],
                "now": _iso(now),
                "target": _iso(target),
            }
            result.update(_duration_payload((target - now).total_seconds()))
            return result
        return self._action_until(args, now)

    def _action_create(self, args: dict, now: datetime) -> dict:
        content = args.get("content")
        if not isinstance(content, str) or not content.strip():
            return _error("invalid_parameters", "content is required")
        content = content.strip()
        if len(content) > MAX_CONTENT_LENGTH:
            return _error("invalid_parameters",
                          f"content must be at most {MAX_CONTENT_LENGTH} characters")
        schedule, error = self._build_schedule(args, now, existing=None)
        if error is not None:
            return error
        with self._lock:
            self._ensure_loaded()
            pending = sum(1 for reminder in self._reminders
                          if reminder["state"] == "pending")
            if pending >= MAX_PENDING:
                return _error("limit_reached",
                              f"at most {MAX_PENDING} pending reminders are allowed")
            reminder = {
                "id": self._new_id_locked(),
                "content": content,
                "repeat": schedule["repeat"],
                "time_of_day": schedule["time_of_day"],
                "weekday": schedule["weekday"],
                "state": "pending",
                "created_at": now,
                "updated_at": now,
                "next_due": schedule["next_due"],
                "fired_at": None,
                "fired_count": 0,
            }
            self._reminders.append(reminder)
            self._prune_locked()
            self._save_locked()
            return {
                "state": "created",
                "reminder": self._public(reminder),
                "pending_count": pending + 1,
            }

    def _action_list(self, args: dict, now: datetime) -> dict:
        with self._lock:
            self._ensure_loaded()
            pending = [reminder for reminder in self._reminders
                       if reminder["state"] == "pending"]
            finished = [reminder for reminder in self._reminders
                        if reminder["state"] != "pending"]
            pending.sort(key=lambda reminder: reminder["next_due"])
            finished.sort(key=lambda reminder: reminder.get("fired_at") or now,
                          reverse=True)
            reminders = [self._public(reminder) for reminder in pending + finished]
        return {
            "state": "ok",
            "now": _iso(now),
            "count": len(reminders),
            "pending_count": len(pending),
            "reminders": reminders,
        }

    def _action_cancel(self, args: dict, now: datetime) -> dict:
        reminder_id = args.get("id")
        if not isinstance(reminder_id, str) or not reminder_id.strip():
            return _error("invalid_parameters", "id is required")
        reminder_id = reminder_id.strip()
        with self._lock:
            self._ensure_loaded()
            reminder = self._find_locked(reminder_id)
            if reminder is None:
                return _error("not_found", f"reminder {reminder_id!r} does not exist")
            snapshot = self._public(reminder)
            self._reminders.remove(reminder)
            self._save_locked()
        return {"state": "cancelled", "reminder": snapshot}

    def _action_update(self, args: dict, now: datetime) -> dict:
        reminder_id = args.get("id")
        if not isinstance(reminder_id, str) or not reminder_id.strip():
            return _error("invalid_parameters", "id is required")
        reminder_id = reminder_id.strip()
        with self._lock:
            self._ensure_loaded()
            reminder = self._find_locked(reminder_id)
            if reminder is None:
                return _error("not_found", f"reminder {reminder_id!r} does not exist")
            if reminder["state"] != "pending":
                return _error("not_pending",
                              f"reminder {reminder_id!r} has already fired and cannot be updated")

            content = args.get("content")
            if content is not None:
                if not isinstance(content, str) or not content.strip():
                    return _error("invalid_parameters", "content must not be blank")
                content = content.strip()
                if len(content) > MAX_CONTENT_LENGTH:
                    return _error(
                        "invalid_parameters",
                        f"content must be at most {MAX_CONTENT_LENGTH} characters")

            schedule_keys = ("repeat", "time", "time_of_day", "weekday")
            reschedule = any(args.get(key) not in (None, "") for key in schedule_keys)
            schedule, error = (self._build_schedule(args, now, existing=reminder)
                               if reschedule else (None, None))
            if error is not None:
                return error
            if content is not None:
                reminder["content"] = content
            if schedule is not None:
                reminder["repeat"] = schedule["repeat"]
                reminder["time_of_day"] = schedule["time_of_day"]
                reminder["weekday"] = schedule["weekday"]
                reminder["next_due"] = schedule["next_due"]
            reminder["updated_at"] = now
            self._save_locked()
            return {"state": "updated", "reminder": self._public(reminder)}

    def _action_info(self, now: datetime) -> dict:
        with self._lock:
            self._ensure_loaded()
            pending = sorted(
                (reminder for reminder in self._reminders
                 if reminder["state"] == "pending"),
                key=lambda reminder: reminder["next_due"])
            next_reminder = self._public(pending[0]) if pending else None
        running = self._thread is not None and self._thread.is_alive()
        return {
            "state": "ready" if running else "idle",
            "now": _iso(now),
            "pending_count": len(pending),
            "next_reminder": next_reminder,
            "topic_out": [{"topic": self._output_topic, "format": "data/json"}],
        }

    # ── Plugin lifecycle ──────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "clock",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Bumi clock and reminder card — query local time and weekday, "
                "compute how long until a target time, and create/list/update/"
                "cancel one-shot, daily or weekly reminders. Due reminders are "
                f"published as JSON events on {self._output_topic}."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": list(_ACTIONS),
                        "description": "Action to perform",
                    },
                    "content": {
                        "type": "string",
                        "description": "Reminder text, e.g. 吃药 / 开会",
                    },
                    "time": {
                        "type": "string",
                        "description": (
                            "Target time as ISO-8601 (2026-11-08T09:30:00). "
                            "A timestamp without an offset is local time."
                        ),
                    },
                    "time_of_day": {
                        "type": "string",
                        "description": "Time of day for daily/weekly reminders, HH:MM or HH:MM:SS",
                    },
                    "weekday": {
                        "type": "integer",
                        "minimum": 1,
                        "maximum": 7,
                        "description": "Weekday for a weekly reminder, 1=Monday … 7=Sunday",
                    },
                    "repeat": {
                        "type": "string",
                        "enum": ["none", "daily", "weekly"],
                        "description": "Repeat mode; default none (one-shot)",
                    },
                    "id": {
                        "type": "string",
                        "description": "Reminder ID returned by create/list",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "now": {"params": [],
                            "description": "Query the current local date, time and weekday"},
                    "weekday": {"params": [],
                                "description": "Query which weekday today is"},
                    "until": {"params": ["time"],
                              "description": "Compute how long until a target time"},
                    "countdown": {"params": ["time", "id"],
                                  "description": "Countdown to a target time or an existing reminder"},
                    "create": {"params": ["content", "time", "repeat", "time_of_day", "weekday"],
                               "description": "Create a reminder and write it into the scheduler"},
                    "list": {"params": [],
                             "description": "List reminders created by the user"},
                    "cancel": {"params": ["id"],
                               "description": "Cancel a reminder by ID"},
                    "update": {"params": ["id", "content", "time", "repeat", "time_of_day", "weekday"],
                               "description": "Change a pending reminder's content or time"},
                },
            },
            "topic_out": [{"topic": self._output_topic, "format": "data/json"}],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self) -> bool:
        with self._lock:
            self._ensure_loaded()
        if self._thread is not None and self._thread.is_alive():
            return True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, name=f"bumi_clock_{self._namespace}", daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2)
        self._thread = None

    def dispatch(self, action: str, args: dict) -> dict | None:
        args = args or {}
        now = self._now()
        try:
            if action == "start":
                self.start()
                running = self._thread is not None and self._thread.is_alive()
                return {"state": "ready" if running else "idle"}
            if action == "stop":
                self.stop()
                return {"state": "idle"}
            if action in ("info", "clock"):
                return self._action_info(now)
            if action in ("now", "time", "current_time"):
                return _now_payload(now)
            if action == "weekday":
                return {
                    "state": "ok",
                    "now": _iso(now),
                    "weekday": _weekday_payload(now.isoweekday()),
                }
            if action in ("until", "time_until"):
                return self._action_until(args, now)
            if action == "countdown":
                return self._action_countdown(args, now)
            if action in ("create", "add", "remind"):
                return self._action_create(args, now)
            if action in ("list", "query"):
                return self._action_list(args, now)
            if action == "cancel":
                return self._action_cancel(args, now)
            if action == "update":
                return self._action_update(args, now)
            return _error("invalid_action", f"unknown action {action!r}")
        except Exception as exc:  # the MCP layer must never see a traceback
            return _error("error", f"{type(exc).__name__}: {exc}")
