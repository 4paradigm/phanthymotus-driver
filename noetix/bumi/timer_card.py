"""Repeating timer card for Bumi (and any bundle that copies this file).

Why this card exists: the agent loop cannot create a timer. `task_create` only
tracks a task, `scheduler` is operator configuration, and a skill that claims
"remind me in 10 minutes" with neither is lying. This card is the missing clock:
it publishes a `data/json` tick on a ROS topic, the canvas wires that topic to
the decision core, and the event payload carries `"priority": 1` so the
collector steers it to the main agent instead of filing it as a background
sensor event.

Two deliberate choices:

* `start` only arms the card. The canvas sends `start` when a project boots;
  if that began ticking, every deployment would wake the model once a minute
  forever. `begin` starts the ticks, `halt` stops them while staying armed,
  `stop` disarms.
* The scheduling core has no ROS import. The thread, the payload and the
  stop semantics are testable on a laptop; `_TimerNode` is the only part that
  needs rclpy.
"""

from __future__ import annotations

import json
import math
import threading
import time
import uuid

try:  # ROS is absent on a laptop; the core and the tests must still import.
    from rclpy.node import Node
    from std_msgs.msg import String
except Exception:  # noqa: BLE001 - any import failure means "no ROS here"
    Node = object  # type: ignore[assignment]
    String = None  # type: ignore[assignment]


TICK_EVENT = "timer_tick"
TIMER_NAME = "bumi-timer"
MIN_INTERVAL_S = 1.0
MAX_INTERVAL_S = 3600.0
MAX_TICKS = 1_000_000


class TimerCore:
    """A single repeating timer with no ROS dependency.

    `publish` is called from the worker thread with the tick payload. It may
    raise; the error is retained in the snapshot and the timer keeps ticking,
    because one dropped publish must not silently end the schedule.
    """

    def __init__(self, publish, clock=time.monotonic, wall=time.time):
        self._publish = publish
        self._clock = clock
        self._wall = wall
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._state: dict = {"state": "idle"}

    def start(self, interval_s: float, label: str = "", max_ticks: int = 0) -> dict:
        interval = float(interval_s)
        if not math.isfinite(interval) or not MIN_INTERVAL_S <= interval <= MAX_INTERVAL_S:
            raise ValueError(f"interval_s must be between {MIN_INTERVAL_S:g} and {MAX_INTERVAL_S:g}")
        label = str(label or "").strip()
        if len(label) > 64:
            raise ValueError("label must be at most 64 characters")
        try:
            max_ticks = int(max_ticks or 0)
        except (TypeError, ValueError):
            raise ValueError("max_ticks must be an integer") from None
        if not 0 <= max_ticks <= MAX_TICKS:
            raise ValueError(f"max_ticks must be between 0 and {MAX_TICKS}")

        self.stop()
        stop_event = threading.Event()
        timer_id = uuid.uuid4().hex[:8]
        start_mono = self._clock()
        with self._lock:
            self._stop = stop_event
            self._state = {
                "state": "running",
                "timer_id": timer_id,
                "label": label,
                "interval_s": interval,
                "max_ticks": max_ticks,
                "tick": 0,
                "started_mono": start_mono,
                "started_at": self._wall(),
                "next_due_mono": start_mono + interval,
                "last_tick_mono": None,
                "last_tick_at": None,
                "last_error": "",
            }
            thread = threading.Thread(
                target=self._loop,
                args=(stop_event, timer_id, interval, label, max_ticks, start_mono),
                daemon=True,
                name="bumi-timer",
            )
            self._thread = thread
            thread.start()
        return self.snapshot()

    def stop(self) -> dict:
        with self._lock:
            stop_event = self._stop
            thread = self._thread
            self._thread = None
            self._stop = threading.Event()
        stop_event.set()
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            if self._state.get("state") == "running":
                self._state = {**self._state, "state": "idle"}
        return self.snapshot()

    def snapshot(self) -> dict:
        with self._lock:
            state = dict(self._state)
        state.pop("started_mono", None)
        state.pop("last_tick_mono", None)
        state.pop("next_due_mono", None)
        if state.get("state") == "running":
            now = self._clock()
            started = self._state.get("started_mono", now)
            due = self._state.get("next_due_mono", now)
            state["elapsed_s"] = round(max(0.0, now - started), 3)
            state["next_tick_in_s"] = round(max(0.0, due - now), 3)
        return state

    def _loop(self, stop_event, timer_id, interval, label, max_ticks, start_mono):
        tick = 0
        try:
            while not stop_event.is_set():
                with self._lock:
                    if self._state.get("timer_id") != timer_id:
                        return
                    due = self._state.get("next_due_mono", self._clock() + interval)
                delay = due - self._clock()
                if delay > 0 and stop_event.wait(delay):
                    break
                if stop_event.is_set():
                    break

                tick += 1
                now_mono = self._clock()
                now_wall = self._wall()
                payload = {
                    # Read by collector._extract_priority() before source
                    # matching. Without it this is a P=0 background event.
                    "priority": 1,
                    "event": TICK_EVENT,
                    "timer": TIMER_NAME,
                    "timer_id": timer_id,
                    "label": label,
                    "tick": tick,
                    "interval_s": interval,
                    "elapsed_s": round(now_mono - start_mono, 3),
                    "ts": round(now_wall, 3),
                }
                try:
                    self._publish(payload)
                except Exception as exc:  # noqa: BLE001
                    with self._lock:
                        if self._state.get("timer_id") == timer_id:
                            self._state["last_error"] = str(exc)[:200]

                with self._lock:
                    if self._state.get("timer_id") != timer_id:
                        return
                    self._state.update({
                        "tick": tick,
                        "last_tick_mono": now_mono,
                        "last_tick_at": now_wall,
                        "elapsed_s": round(now_mono - start_mono, 3),
                        "next_due_mono": now_mono + interval,
                    })
                    finished = bool(max_ticks) and tick >= max_ticks
                    if finished:
                        self._state["state"] = "finished"
                if finished:
                    return
        finally:
            with self._lock:
                if (self._state.get("timer_id") == timer_id
                        and self._state.get("state") == "running"):
                    self._state["state"] = "idle"


class _TimerNode(Node):  # type: ignore[misc,valid-type]
    """The only ROS-dependent part: one String publisher."""

    def __init__(self, topic: str):
        if String is None:
            raise RuntimeError("rclpy is not available")
        super().__init__("bumi_timer")
        self._topic = topic
        self._publisher = self.create_publisher(String, topic, 10)

    def publish(self, payload: dict) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._publisher.publish(message)


class TimerPlugin:
    PREFIX = "timer"

    def __init__(self, plugin_config: dict, namespace: str, executor, node_factory=None):
        self._namespace = namespace
        config = plugin_config or {}
        self._topic = str(config.get("topic") or f"/{namespace}/timer/events")
        try:
            self._default_interval = float(config.get("interval_s", 60))
        except (TypeError, ValueError):
            self._default_interval = 60.0
        if not MIN_INTERVAL_S <= self._default_interval <= MAX_INTERVAL_S:
            self._default_interval = 60.0
        self._node = (node_factory or _TimerNode)(self._topic)
        executor.add_node(self._node)
        self._core = TimerCore(self._node.publish)
        self._armed = False

    def get_tool(self) -> dict:
        interval = self._default_interval
        return {
            "name": "timer",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "通用重复定时器：start/stop 只让卡片待命，begin 后按 interval_s 周期发布 "
                f"timer_tick 事件到 {self._topic}（data/json，priority=1），halt 停止 tick，"
                "stop 停止并退出。事件可直接连到 decision core。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "begin", "halt", "stop", "info"],
                    },
                    "interval_s": {
                        "type": "number",
                        "description": "tick 间隔（秒），1–3600",
                        "minimum": MIN_INTERVAL_S,
                        "maximum": MAX_INTERVAL_S,
                        "default": interval,
                    },
                    "label": {
                        "type": "string",
                        "description": "事件标签，便于下游过滤，例如 sedentary",
                        "maxLength": 64,
                        "default": "",
                    },
                    "max_ticks": {
                        "type": "integer",
                        "description": "最多发多少个 tick，0 表示不限",
                        "minimum": 0,
                        "maximum": MAX_TICKS,
                        "default": 0,
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": [], "description": "卡片待命，不产生 tick"},
                    "begin": {"params": ["interval_s", "label", "max_ticks"],
                              "description": "开始周期发布 timer_tick"},
                    "halt": {"params": [], "description": "停止 tick，保持待命"},
                    "stop": {"params": [], "description": "停止 tick 并退出"},
                    "info": {"params": [], "description": "查询计时状态"},
                },
            },
            "configSchema": {
                "type": "object",
                "properties": {
                    "interval_s": {
                        "type": "number",
                        "title": "默认间隔（秒）",
                        "default": interval,
                        "minimum": MIN_INTERVAL_S,
                        "maximum": MAX_INTERVAL_S,
                        "description": "begin 未指定 interval_s 时使用。",
                    },
                },
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    def start(self) -> None:
        # The bundle lifecycle must not start ticks: the canvas sends the card's
        # `start` when a project boots, and an always-ticking timer would wake
        # the model once a minute for every deployment.
        return None

    def stop(self) -> None:
        self._armed = False
        self._core.stop()

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop("_tool_name", None)
        if action == "start":
            self._armed = True
            return {"state": "ready", "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        if action == "begin":
            self._armed = True
            interval = args.get("interval_s", self._default_interval)
            label = args.get("label", "")
            max_ticks = args.get("max_ticks", 0)
            try:
                requested_interval = float(interval)
                requested_label = str(label or "").strip()
                requested_max = int(max_ticks or 0)
            except (TypeError, ValueError):
                requested_interval = None
                requested_label = ""
                requested_max = -1
            current = self._core.snapshot()
            if (requested_interval is not None
                    and current.get("state") == "running"
                    and current.get("interval_s") == requested_interval
                    and current.get("label") == requested_label
                    and current.get("max_ticks") == requested_max):
                return {**current, "topic_out": [{"topic": self._topic, "format": "data/json"}]}
            try:
                state = self._core.start(interval, label, max_ticks)
            except ValueError as exc:
                return {"state": "error", "error": str(exc)}
            return {**state, "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        if action == "halt":
            self._core.stop()
            self._armed = True
            return {"state": "ready", "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        if action == "stop":
            self._core.stop()
            self._armed = False
            return {"state": "idle", "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        if action == "info":
            state = self._core.snapshot()
            if state.get("state") == "idle" and self._armed:
                state["state"] = "ready"
            state["armed"] = self._armed
            return {**state, "topic_out": [{"topic": self._topic, "format": "data/json"}]}
        return None
