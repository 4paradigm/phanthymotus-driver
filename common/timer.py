"""Generic monotonic timer card.

The card starts timers synchronously and emits their later alarms as
``data/json`` on a ROS topic.  It deliberately contains no robot-specific
behaviour: consumers decide whether an alarm should speak, move, change a
light, or only update application state.

Wall-clock time is used only for human-readable event timestamps.  Deadlines
use ``time.monotonic`` so NTP and manual clock corrections cannot make a timer
jump or fire twice.
"""

from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
import math
import re
import secrets
import threading
import time
from typing import Any, Callable


SCHEMA = "motus.timer.event/1"
NAME_RE = re.compile(r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
ACTIVE = frozenset({"running", "paused"})
TERMINAL = frozenset({"completed", "cancelled"})


def _number(value: Any, name: str, *, minimum: float = 0.0,
            maximum: float | None = None, allow_zero: bool = True) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name}_must_be_number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name}_must_be_finite")
    if result < minimum or (not allow_zero and result == 0):
        raise ValueError(f"{name}_out_of_range")
    if maximum is not None and result > maximum:
        raise ValueError(f"{name}_out_of_range")
    return result


def _name(value: Any, field: str) -> str:
    if not isinstance(value, str) or not NAME_RE.fullmatch(value):
        raise ValueError(f"invalid_{field}")
    return value


def _json_object(value: Any, field: str) -> dict:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{field}_must_be_object")
    # Reject values which the outgoing JSON publisher could not serialize.
    try:
        return json.loads(json.dumps(value, allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field}_must_be_json") from exc


def _normalise_dashboard_create_args(args: dict) -> dict:
    """Decode Canvas JSON text fields before handing values to TimerEngine."""
    result = dict(args)
    for field in ("alarms", "payload"):
        value = result.get(field)
        if not isinstance(value, str):
            continue
        value = value.strip()
        if not value:
            result.pop(field, None)
            continue
        try:
            result[field] = json.loads(value)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{field}_must_be_valid_json") from exc

    for field in ("replace", "auto_remove"):
        value = result.get(field)
        if not isinstance(value, str):
            continue
        value = value.strip().lower()
        if not value:
            result.pop(field, None)
        elif value == "true":
            result[field] = True
        elif value == "false":
            result[field] = False
        else:
            raise ValueError("boolean_option_required")
    return result


def _round_seconds(value: float | None) -> float | None:
    return None if value is None else round(max(0.0, value), 6)


def _iso8601(unix_seconds: float) -> str:
    return datetime.fromtimestamp(unix_seconds, timezone.utc).isoformat(
        timespec="milliseconds").replace("+00:00", "Z")


class TimerEngine:
    """State machine independent of ROS and worker-thread scheduling."""

    def __init__(self, *, max_timers: int = 64, max_duration_sec: float = 86400.0,
                 max_alarms: int = 64):
        self.max_timers = int(max_timers)
        self.max_duration_sec = float(max_duration_sec)
        self.max_alarms = int(max_alarms)
        if self.max_timers < 1 or self.max_alarms < 1 or self.max_duration_sec <= 0:
            raise ValueError("invalid_timer_limits")
        self.timers: dict[str, dict] = {}

    @staticmethod
    def _elapsed(timer: dict, now: float) -> float:
        if timer["status"] == "running":
            return timer["accumulated_sec"] + max(0.0, now - timer["resumed_at"])
        return timer["accumulated_sec"]

    @staticmethod
    def _remaining(timer: dict, elapsed: float) -> float | None:
        duration = timer["duration_sec"]
        return None if duration is None else max(0.0, duration - elapsed)

    def _snapshot(self, timer: dict, now: float) -> dict:
        elapsed = self._elapsed(timer, now)
        remaining = self._remaining(timer, elapsed)
        pending = [alarm for alarm in timer["alarms"]
                   if alarm["alarm_id"] not in timer["fired"]]
        next_alarm = min(pending, key=lambda item: item["due_elapsed_sec"],
                         default=None)
        result = {
            "timer_id": timer["timer_id"],
            "run_id": timer["run_id"],
            "mode": timer["mode"],
            "status": timer["status"],
            "duration_sec": timer["duration_sec"],
            "elapsed_sec": _round_seconds(elapsed),
            "remaining_sec": _round_seconds(remaining),
            "emit_interval_sec": timer["emit_interval_sec"],
            "started_at": timer["started_at"],
            "payload": copy.deepcopy(timer["payload"]),
        }
        if next_alarm is not None:
            result["next_alarm"] = {
                key: copy.deepcopy(next_alarm[key]) for key in
                ("alarm_id", "trigger_type", "trigger_sec", "event")
            }
        else:
            result["next_alarm"] = None
        return result

    def _normalise_alarms(self, raw: Any, mode: str,
                          duration: float | None) -> list[dict]:
        if raw is None:
            return []
        if not isinstance(raw, list) or len(raw) > self.max_alarms:
            raise ValueError("invalid_alarms")
        alarms, ids = [], set()
        for item in raw:
            if not isinstance(item, dict):
                raise ValueError("invalid_alarm")
            unknown = set(item) - {"alarm_id", "trigger_type", "trigger_sec",
                                   "event", "payload"}
            if unknown:
                raise ValueError("invalid_alarm_fields")
            alarm_id = _name(item.get("alarm_id"), "alarm_id")
            if alarm_id in ids:
                raise ValueError("duplicate_alarm_id")
            ids.add(alarm_id)
            trigger_type = item.get("trigger_type")
            if trigger_type not in ("elapsed", "remaining"):
                raise ValueError("invalid_trigger_type")
            if trigger_type == "remaining" and (mode != "countdown" or duration is None):
                raise ValueError("remaining_alarm_requires_countdown")
            trigger = _number(item.get("trigger_sec"), "trigger_sec",
                              maximum=self.max_duration_sec)
            if duration is not None and trigger > duration:
                raise ValueError("alarm_outside_duration")
            event = item.get("event")
            if not isinstance(event, str) or not event.strip() or len(event) > 128:
                raise ValueError("invalid_alarm_event")
            due = trigger if trigger_type == "elapsed" else duration - trigger
            alarms.append({
                "alarm_id": alarm_id,
                "trigger_type": trigger_type,
                "trigger_sec": trigger,
                "due_elapsed_sec": due,
                "event": event.strip(),
                "payload": _json_object(item.get("payload"), "alarm_payload"),
            })
        return sorted(alarms, key=lambda value: (value["due_elapsed_sec"],
                                                  value["alarm_id"]))

    def start(self, args: dict, now: float, wall: float) -> dict:
        allowed = {"timer_id", "mode", "duration_sec", "alarms",
                   "emit_interval_sec", "replace", "auto_remove", "payload"}
        unknown = set(args) - allowed
        if unknown:
            raise ValueError(f"invalid_start_fields: {', '.join(sorted(unknown))}")
        timer_id = _name(args.get("timer_id"), "timer_id")
        mode = args.get("mode")
        if mode not in ("countup", "countdown"):
            raise ValueError("invalid_timer_mode")
        duration_value = args.get("duration_sec")
        if mode == "countdown" and duration_value is None:
            raise ValueError("countdown_requires_duration_sec")
        duration = (None if duration_value is None else
                    _number(duration_value, "duration_sec", allow_zero=False,
                            maximum=self.max_duration_sec))
        interval = _number(args.get("emit_interval_sec", 0), "emit_interval_sec",
                           maximum=self.max_duration_sec)
        if interval and interval < 0.1:
            raise ValueError("emit_interval_too_small")
        replace = args.get("replace", False)
        auto_remove = args.get("auto_remove", False)
        if not isinstance(replace, bool) or not isinstance(auto_remove, bool):
            raise ValueError("boolean_option_required")
        existing = self.timers.get(timer_id)
        if existing is not None and existing["status"] in ACTIVE and not replace:
            raise ValueError("timer_already_exists")
        active_count = sum(t["status"] in ACTIVE for t in self.timers.values())
        if (existing is None or existing["status"] not in ACTIVE) and active_count >= self.max_timers:
            raise ValueError("timer_limit_reached")
        alarms = self._normalise_alarms(args.get("alarms"), mode, duration)
        if mode == "countup" and duration is None and not alarms and interval == 0:
            raise ValueError("unbounded_timer_has_no_output")
        run_id = f"{timer_id}-{secrets.token_hex(6)}"
        spec = {
            "timer_id": timer_id,
            "mode": mode,
            "duration_sec": duration,
            "alarms": copy.deepcopy(args.get("alarms") or []),
            "emit_interval_sec": interval,
            "auto_remove": auto_remove,
            "payload": _json_object(args.get("payload"), "payload"),
        }
        timer = {
            **spec,
            "spec": spec,
            "run_id": run_id,
            "status": "running",
            "resumed_at": now,
            "accumulated_sec": 0.0,
            "started_at": _iso8601(wall),
            "started_unix": wall,
            "alarms": alarms,
            "fired": set(),
            "next_tick_elapsed": interval if interval else None,
            "event_seq": 0,
        }
        self.timers[timer_id] = timer
        return self._snapshot(timer, now)

    def pause(self, timer_id: str, now: float) -> dict:
        timer = self._get(timer_id)
        if timer["status"] != "running":
            raise ValueError("timer_not_running")
        timer["accumulated_sec"] = self._elapsed(timer, now)
        timer["status"] = "paused"
        return self._snapshot(timer, now)

    def resume(self, timer_id: str, now: float) -> dict:
        timer = self._get(timer_id)
        if timer["status"] != "paused":
            raise ValueError("timer_not_paused")
        timer["resumed_at"] = now
        timer["status"] = "running"
        return self._snapshot(timer, now)

    def cancel(self, timer_id: str, now: float,
               wall: float | None = None) -> tuple[dict, dict | None]:
        timer = self._get(timer_id)
        if timer["status"] not in ACTIVE:
            raise ValueError("timer_not_active")
        timer["accumulated_sec"] = self._elapsed(timer, now)
        timer["status"] = "cancelled"
        event = self._event(timer, "timer_cancelled", now,
                            time.time() if wall is None else wall,
                            event="timer-cancelled")
        return self._snapshot(timer, now), event

    def reset(self, timer_id: str, now: float, wall: float) -> dict:
        old = self._get(timer_id)
        args = copy.deepcopy(old["spec"])
        args["replace"] = True
        return self.start(args, now, wall)

    def info(self, timer_id: str, now: float) -> dict:
        return self._snapshot(self._get(timer_id), now)

    def list(self, now: float) -> list[dict]:
        return [self._snapshot(self.timers[key], now) for key in sorted(self.timers)]

    def _get(self, timer_id: Any) -> dict:
        key = _name(timer_id, "timer_id")
        try:
            return self.timers[key]
        except KeyError:
            raise ValueError("timer_not_found") from None

    def _event(self, timer: dict, event_type: str, now: float, wall: float,
               *, event: str, alarm: dict | None = None,
               scheduled_elapsed: float | None = None,
               actual_elapsed: float | None = None) -> dict:
        timer["event_seq"] += 1
        elapsed = self._elapsed(timer, now)
        remaining = self._remaining(timer, elapsed)
        result = {
            "schema": SCHEMA,
            "source": "timer",
            "type": event_type,
            # Agent Core routes priority>0 JSON to the main agent.  Progress
            # ticks stay in the background so short intervals cannot wake the
            # LLM on every update.
            "priority": 1 if event_type in (
                "timer_alarm", "timer_completed", "timer_cancelled"
            ) else 0,
            "event": event,
            "timer_id": timer["timer_id"],
            "run_id": timer["run_id"],
            "event_seq": timer["event_seq"],
            "mode": timer["mode"],
            "status": timer["status"],
            "duration_sec": timer["duration_sec"],
            "elapsed_sec": _round_seconds(elapsed),
            "remaining_sec": _round_seconds(remaining),
            "emitted_at": _iso8601(wall),
            "emitted_at_unix_ms": round(wall * 1000),
            "payload": copy.deepcopy(timer["payload"]),
        }
        if scheduled_elapsed is not None:
            result["scheduled_elapsed_sec"] = _round_seconds(scheduled_elapsed)
            measured = elapsed if actual_elapsed is None else actual_elapsed
            result["drift_ms"] = round(max(0.0, measured - scheduled_elapsed) * 1000, 3)
        if alarm is not None:
            result.update({
                "alarm_id": alarm["alarm_id"],
                "trigger_type": alarm["trigger_type"],
                "trigger_sec": alarm["trigger_sec"],
                "alarm_payload": copy.deepcopy(alarm["payload"]),
            })
        return result

    def poll(self, now: float, wall: float) -> list[dict]:
        events, remove = [], []
        for timer_id, timer in list(self.timers.items()):
            if timer["status"] != "running":
                continue
            elapsed = self._elapsed(timer, now)
            duration = timer["duration_sec"]
            effective = min(elapsed, duration) if duration is not None else elapsed

            for alarm in timer["alarms"]:
                if (alarm["alarm_id"] not in timer["fired"] and
                        effective >= alarm["due_elapsed_sec"]):
                    timer["fired"].add(alarm["alarm_id"])
                    events.append(self._event(
                        timer, "timer_alarm", now, wall, event=alarm["event"],
                        alarm=alarm, scheduled_elapsed=alarm["due_elapsed_sec"]))

            interval = timer["emit_interval_sec"]
            due_tick = timer["next_tick_elapsed"]
            if (interval and due_tick is not None and effective >= due_tick and
                    (duration is None or elapsed < duration)):
                # Coalesce missed ticks.  A delayed process must not flood the
                # Agent with one stale event for every missed interval.  Once
                # due, completion supersedes a refresh tick for this timer.
                scheduled = due_tick
                timer["next_tick_elapsed"] = (math.floor(effective / interval) + 1) * interval
                events.append(self._event(timer, "timer_tick", now, wall,
                                          event="timer-tick",
                                          scheduled_elapsed=scheduled))

            if duration is not None and elapsed >= duration:
                timer["accumulated_sec"] = duration
                timer["status"] = "completed"
                events.append(self._event(timer, "timer_completed", now, wall,
                                          event="timer-completed",
                                          scheduled_elapsed=duration,
                                          actual_elapsed=elapsed))
                if timer["auto_remove"]:
                    remove.append(timer_id)
        for timer_id in remove:
            self.timers.pop(timer_id, None)
        return events

    def next_delay(self, now: float, default: float = 1.0) -> float:
        deadlines = []
        for timer in self.timers.values():
            if timer["status"] != "running":
                continue
            elapsed = self._elapsed(timer, now)
            duration = timer["duration_sec"]
            if duration is not None:
                deadlines.append(duration - elapsed)
            if timer["next_tick_elapsed"] is not None:
                deadlines.append(timer["next_tick_elapsed"] - elapsed)
            deadlines.extend(alarm["due_elapsed_sec"] - elapsed
                             for alarm in timer["alarms"]
                             if alarm["alarm_id"] not in timer["fired"])
        if not deadlines:
            return default
        return max(0.001, min(default, max(0.0, min(deadlines))))


class TimerPlugin:
    """MCP/ROS wrapper around :class:`TimerEngine`."""

    PREFIX = "timer"

    def __init__(self, config: dict | None, namespace: str, executor=None, *,
                 monotonic: Callable[[], float] = time.monotonic,
                 wall_clock: Callable[[], float] = time.time,
                 event_sink: Callable[[dict], None] | None = None):
        config = config or {}
        self._monotonic = monotonic
        self._wall_clock = wall_clock
        self._event_sink = event_sink
        self._engine = TimerEngine(
            max_timers=int(config.get("max_timers", 64)),
            max_duration_sec=float(config.get("max_duration_sec", 86400)),
            max_alarms=int(config.get("max_alarms", 64)),
        )
        root = f"/{namespace}" if namespace else ""
        self._topic = str(config.get("topic") or f"{root}/timer/events")
        if not self._topic.startswith("/"):
            raise ValueError("timer_topic_must_be_absolute")
        self._condition = threading.Condition(threading.RLock())
        self._shutdown = True
        self._thread: threading.Thread | None = None
        self._node = None
        self._publisher = None
        self._last_event = None
        self._published_count = 0
        self._delivery_log_lock = threading.Lock()
        self._delivery_errors = {
            "event sink": {"active": False, "last_logged": 0.0, "suppressed": 0},
            "event publish": {"active": False, "last_logged": 0.0, "suppressed": 0},
        }
        if executor is not None:
            try:
                from rclpy.node import Node
                from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                                       QoSProfile, ReliabilityPolicy)
                from std_msgs.msg import String
                qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                 history=HistoryPolicy.KEEP_LAST, depth=100,
                                 durability=DurabilityPolicy.VOLATILE)
                suffix = re.sub(r"[^a-zA-Z0-9_]", "_", namespace or "common")
                self._node = Node(f"timer_{suffix}")
                self._publisher = self._node.create_publisher(String, self._topic, qos)
                executor.add_node(self._node)
            except Exception as exc:
                print(f"[timer] ROS publisher unavailable: {exc}", flush=True)
                self._node = None
                self._publisher = None

    def get_tool(self) -> dict:
        # start/stop/info are Canvas lifecycle actions.  Creating a timer uses
        # create so a project start cannot be mistaken for an incomplete timer
        # request.
        actions = ["start", "stop", "create", "pause", "resume", "cancel",
                   "reset", "info", "list"]
        alarm = {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "alarm_id": {"type": "string", "pattern": "^[a-z0-9]+(?:-[a-z0-9]+)*$"},
                "trigger_type": {"type": "string", "enum": ["elapsed", "remaining"]},
                "trigger_sec": {"type": "number", "minimum": 0},
                "event": {"type": "string", "minLength": 1, "maxLength": 128},
                "payload": {"type": "object", "additionalProperties": True},
            },
            "required": ["alarm_id", "trigger_type", "trigger_sec", "event"],
        }
        schema = {
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "action": {"type": "string", "enum": actions},
                "timer_id": {"type": "string", "pattern": "^[a-z0-9]+(?:-[a-z0-9]+)*$"},
                "mode": {"type": "string", "enum": ["countup", "countdown"]},
                "duration_sec": {"type": "number", "exclusiveMinimum": 0,
                                 "maximum": self._engine.max_duration_sec},
                "alarms": {"type": ["array", "string"],
                           "description": "报警列表；画布可输入 JSON 数组文本",
                           "maxItems": self._engine.max_alarms,
                           "items": alarm},
                "emit_interval_sec": {"type": "number", "minimum": 0,
                                      "maximum": self._engine.max_duration_sec,
                                      "default": 0},
                "replace": {
                    "type": "string",
                    "enum": ["false", "true"],
                    "default": "false",
                    "description": "同名运行中 Timer 已存在时是否替换",
                },
                "auto_remove": {
                    "type": "string",
                    "enum": ["false", "true"],
                    "default": "false",
                    "description": "Timer 完成后是否自动从列表移除",
                },
                "payload": {"type": ["object", "string"],
                            "description": "事件附加数据；画布可输入 JSON 对象文本",
                            "additionalProperties": True},
            },
            "required": ["action"],
            "x-action-params": {
                "start": {"params": [], "description": "启动 Timer 卡片的后台调度"},
                "stop": {"params": [], "description": "停止 Timer 卡片的后台调度"},
                "create": {"params": ["timer_id", "mode", "duration_sec", "alarms",
                                       "emit_interval_sec", "replace", "auto_remove", "payload"],
                           "description": "创建并启动正向计时器或倒计时器"},
                "pause": {"params": ["timer_id"], "description": "暂停指定计时器"},
                "resume": {"params": ["timer_id"], "description": "恢复指定计时器"},
                "cancel": {"params": ["timer_id"], "description": "取消指定计时器并发送取消事件"},
                "reset": {"params": ["timer_id"], "description": "按原配置重新启动指定计时器"},
                "info": {"params": ["timer_id"],
                         "description": "查询指定计时器；不填 ID 时查询卡片状态"},
                "list": {"params": [], "description": "列出当前保存的全部计时器"},
            },
        }
        return {
            "name": self.PREFIX,
            "type": "processor",
            "multiInstance": False,
            "description": (
                "通用正向/倒计时器。create立即返回；报警、周期进度和完成事件通过"
                "data/json输出，不执行语音、灯光或机器人动作。"),
            "inputSchema": schema,
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self) -> None:
        with self._condition:
            if not self._shutdown:
                return
            self._shutdown = False
            self._thread = threading.Thread(target=self._worker, daemon=True,
                                            name="common-timer")
            self._thread.start()

    def stop(self) -> None:
        with self._condition:
            self._shutdown = True
            self._condition.notify_all()
            worker = self._thread
        if worker is not None and worker is not threading.current_thread():
            worker.join(3.0)
        with self._condition:
            self._thread = None

    def _delivery_succeeded(self, kind: str) -> None:
        with self._delivery_log_lock:
            self._delivery_errors[kind]["active"] = False
            self._delivery_errors[kind]["suppressed"] = 0

    def _delivery_failed(self, kind: str, exc: Exception) -> None:
        now = self._monotonic()
        with self._delivery_log_lock:
            state = self._delivery_errors[kind]
            if state["active"] and now - state["last_logged"] < 60:
                state["suppressed"] += 1
                return
            suppressed = state["suppressed"]
            state.update(active=True, last_logged=now, suppressed=0)
        safe = str(exc)[:160].encode("unicode_escape").decode("ascii")[:200]
        repeats = f" (suppressed {suppressed} repeats)" if suppressed else ""
        print(f"[timer] {kind} failed: {safe}{repeats}", flush=True)

    def _publish(self, event: dict) -> None:
        self._last_event = copy.deepcopy(event)
        self._published_count += 1
        if self._event_sink is not None:
            try:
                self._event_sink(copy.deepcopy(event))
            except Exception as exc:
                self._delivery_failed("event sink", exc)
            else:
                self._delivery_succeeded("event sink")
        if self._publisher is not None:
            try:
                from std_msgs.msg import String
                message = String()
                message.data = json.dumps(event, ensure_ascii=False, allow_nan=False,
                                          separators=(",", ":"))
                self._publisher.publish(message)
            except Exception as exc:
                self._delivery_failed("event publish", exc)
            else:
                self._delivery_succeeded("event publish")

    def _worker(self) -> None:
        while True:
            with self._condition:
                if self._shutdown:
                    return
                now, wall = self._monotonic(), self._wall_clock()
                events = self._engine.poll(now, wall)
            for event in events:
                self._publish(event)
            with self._condition:
                if self._shutdown:
                    return
                # Recompute after publishing.  A start/reset may have notified
                # us between the two critical sections; using the earlier
                # delay would lose that wake-up and make a short timer late.
                delay = self._engine.next_delay(self._monotonic())
                self._condition.wait(timeout=delay)

    def dispatch(self, action: str, args: dict) -> dict:
        # Agent Core attaches routing/tracing metadata to split action calls.
        # None of these fields are part of the timer specification.
        clean = {key: value for key, value in args.items()
                 if key not in ("_tool_name", "concurrent", "_trace_id", "instance_id")}

        # Do not call stop() while holding _condition: stop waits for the worker,
        # and the worker needs the same condition once more in order to exit.
        if action == "start":
            self.start()
            return {"state": "running"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "create":
            clean = _normalise_dashboard_create_args(clean)

        with self._condition:
            now, wall = self._monotonic(), self._wall_clock()
            if action in ("create", "resume", "reset") and self._shutdown:
                raise ValueError("timer_not_started")
            if action == "create":
                result = self._engine.start(clean, now, wall)
            elif action == "pause":
                result = self._engine.pause(clean.get("timer_id"), now)
            elif action == "resume":
                result = self._engine.resume(clean.get("timer_id"), now)
            elif action == "cancel":
                result, event = self._engine.cancel(clean.get("timer_id"), now, wall)
                if event is not None:
                    # Publish outside the lock below.
                    pass
            elif action == "reset":
                result = self._engine.reset(clean.get("timer_id"), now, wall)
            elif action == "info":
                timer_id = clean.get("timer_id")
                if timer_id is None:
                    result = {
                        "state": "running" if not self._shutdown else "idle",
                        "topic_out": [{"topic": self._topic, "format": "data/json"}],
                    }
                else:
                    result = self._engine.info(timer_id, now)
                    result["topic_out"] = [
                        {"topic": self._topic, "format": "data/json"}]
            elif action == "list":
                result = {"status": "ok", "timers": self._engine.list(now)}
            else:
                raise ValueError("unsupported_timer_action")
            self._condition.notify_all()
        if action == "cancel" and event is not None:
            self._publish(event)
        return result


def make_plugin(config: dict | None, namespace: str, executor=None) -> TimerPlugin:
    return TimerPlugin(config, namespace, executor)
