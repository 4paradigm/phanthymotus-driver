"""Meeting facilitator card for the Unitree G1."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String

    _HAS_ROS2 = True
    _ASR_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE,
    )
except ImportError:
    _HAS_ROS2 = False
    Node = object


CARD = "meeting_facilitator"
TYPE = "actuator"


@dataclass(frozen=True)
class ActionItem:
    description: str
    owner: str = ""


@dataclass(frozen=True)
class Announcement:
    generation: int
    text: str
    gesture: Optional[str] = None


class MeetingController:
    """Track speakers, timing, transcript, and action items."""

    def __init__(self, transcript_limit: int = 200, action_item_limit: int = 100):
        if transcript_limit < 1:
            raise ValueError("transcript_limit must be at least 1")
        if action_item_limit < 1:
            raise ValueError("action_item_limit must be at least 1")
        self.transcript_limit = transcript_limit
        self.action_item_limit = action_item_limit
        self.reset()

    def reset(self) -> None:
        self.active = False
        self.paused = False
        self.participants: list[str] = []
        self.speaker_index = -1
        self.duration_s = 0.0
        self.deadline: Optional[float] = None
        self.remaining_s: Optional[float] = None
        self.warning_sent = False
        self.timeout_sent = False
        self.transcript: list[dict] = []
        self.action_items: list[ActionItem] = []

    @property
    def current_speaker(self) -> Optional[str]:
        if 0 <= self.speaker_index < len(self.participants):
            return self.participants[self.speaker_index]
        return None

    def start(self, participants: list[str], duration_s: float, now: float) -> str:
        if not participants or any(not isinstance(name, str) for name in participants):
            raise ValueError("participants must contain non-empty strings")
        names = [name.strip() for name in participants]
        if any(not name for name in names):
            raise ValueError("participants must contain non-empty strings")
        if not math.isfinite(duration_s) or duration_s <= 0:
            raise ValueError("duration_s must be finite and positive")
        self.reset()
        self.active = True
        self.participants = names
        self.speaker_index = 0
        self.duration_s = duration_s
        self.deadline = now + duration_s
        return names[0]

    def next_speaker(self, now: float) -> Optional[str]:
        if not self.active:
            raise ValueError("no meeting is active")
        self.speaker_index += 1
        self.paused = False
        self.remaining_s = None
        self.warning_sent = False
        self.timeout_sent = False
        if self.speaker_index >= len(self.participants):
            self.deadline = None
            return None
        self.deadline = now + self.duration_s
        return self.current_speaker

    def pause(self, now: float) -> None:
        if not self.active or self.paused:
            raise ValueError("meeting is not running")
        self.remaining_s = max(0.0, (self.deadline or now) - now)
        self.deadline = None
        self.paused = True

    def resume(self, now: float) -> None:
        if not self.active or not self.paused:
            raise ValueError("meeting is not paused")
        self.deadline = now + (self.remaining_s or 0.0)
        self.remaining_s = None
        self.paused = False

    def add_transcript(self, text: str, now: float) -> None:
        text = text.strip()
        if not self.active or self.paused or not text:
            return
        self.transcript.append({
            "speaker": self.current_speaker,
            "text": text,
            "timestamp": now,
        })
        if len(self.transcript) > self.transcript_limit:
            del self.transcript[:-self.transcript_limit]

    def add_action_item(self, description: str, owner: str = "") -> None:
        description = description.strip()
        if not self.active:
            raise ValueError("no meeting is active")
        if not description:
            raise ValueError("description is required")
        if len(self.action_items) >= self.action_item_limit:
            raise ValueError("action item limit reached")
        self.action_items.append(ActionItem(description, owner.strip()))

    def status(self, now: float) -> dict:
        if self.paused:
            remaining = self.remaining_s
        elif self.deadline is not None:
            remaining = max(0.0, self.deadline - now)
        else:
            remaining = None
        return {
            "active": self.active,
            "paused": self.paused,
            "current_speaker": self.current_speaker,
            "speaker_index": self.speaker_index,
            "participants": list(self.participants),
            "remaining_s": None if remaining is None else round(remaining, 1),
            "transcript_count": len(self.transcript),
            "action_items": [item.__dict__.copy() for item in self.action_items],
        }


class MeetingAsrNode(Node):
    def __init__(self, topic: str, on_text):
        super().__init__("g1_meeting_facilitator")
        self._on_text = on_text
        self._subscription = self.create_subscription(
            String, topic, self._handle_message, _ASR_QOS
        )

    def _handle_message(self, message) -> None:
        try:
            payload = json.loads(message.data)
            text = payload["text"]
        except (KeyError, TypeError, json.JSONDecodeError):
            return
        if isinstance(text, str) and text.strip():
            self._on_text(text)


class Plugin:
    """Guide a timed meeting and retain transcript and action items."""

    PREFIX = CARD

    def __init__(self, plugin_config, namespace, executor, dependencies):
        if not _HAS_ROS2:
            raise RuntimeError("meeting_facilitator requires ROS2 rclpy and std_msgs")
        self._config = dict(plugin_config or {})
        self._executor = executor
        self._tts = dependencies["tts"]
        self._led = dependencies["led"]
        self._arm = dependencies.get("arm")
        asr = dependencies.get("asr")
        default_topic = f"/{namespace}/asr/text"
        if asr is not None:
            topic_out = asr.get_tool().get("topic_out", [])
            if topic_out:
                default_topic = topic_out[0]["topic"]
        self._topic = str(self._config.get("asr_topic", default_topic))
        self._default_duration_s = float(
            self._config.get("speaker_duration_s", 120.0)
        )
        self._warning_before_s = float(
            self._config.get("warning_before_s", 15.0)
        )
        if not math.isfinite(self._default_duration_s) or self._default_duration_s <= 0:
            raise ValueError("speaker_duration_s must be finite and positive")
        if not math.isfinite(self._warning_before_s) or self._warning_before_s < 0:
            raise ValueError("warning_before_s must be finite and non-negative")
        self._controller = MeetingController(
            int(self._config.get("transcript_limit", 200)),
            int(self._config.get("action_item_limit", 100)),
        )
        queue_limit = int(self._config.get("announcement_queue_limit", 20))
        self._led_min_hold_s = float(self._config.get("led_min_hold_s", 1.0))
        if queue_limit < 1:
            raise ValueError("announcement_queue_limit must be at least 1")
        if not math.isfinite(self._led_min_hold_s) or self._led_min_hold_s < 0:
            raise ValueError("led_min_hold_s must be finite and non-negative")
        self._node = MeetingAsrNode(self._topic, self._on_text)
        self._executor.add_node(self._node)
        self._lock = threading.RLock()
        self._action_lock = threading.Lock()
        self._wake = threading.Event()
        self._shutdown = threading.Event()
        self._announcement_queue: queue.Queue[Announcement | None] = queue.Queue(
            maxsize=queue_limit
        )
        self._enabled = False
        self._generation = 0
        self._torn_down = False
        self._timer_thread = threading.Thread(
            target=self._timer_loop,
            daemon=True,
            name="g1_meeting_timer",
        )
        self._announcement_thread = threading.Thread(
            target=self._announcement_loop,
            daemon=True,
            name="g1_meeting_announcement",
        )
        self._timer_thread.start()
        self._announcement_thread.start()

    def get_tool(self) -> dict:
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": (
                "G1 meeting host with speaker timing, ASR transcript, "
                "action items, and spoken reminders"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": [
                            "start_meeting", "next_speaker", "pause", "resume",
                            "add_action_item", "end_meeting", "status", "info",
                        ],
                    },
                    "participants": {
                        "type": "array",
                        "items": {"type": "string"},
                    },
                    "duration_s": {"type": "number", "exclusiveMinimum": 0},
                    "description": {"type": "string"},
                    "owner": {"type": "string"},
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "start_meeting": {"params": ["participants", "duration_s"]},
                    "next_speaker": {"params": []},
                    "pause": {"params": []},
                    "resume": {"params": []},
                    "add_action_item": {"params": ["description", "owner"]},
                    "end_meeting": {"params": []},
                    "status": {"params": []},
                    "info": {"params": []},
                },
            },
        }

    def start(self) -> None:
        with self._lock:
            if self._torn_down:
                raise RuntimeError("meeting facilitator has been torn down")
            self._enabled = bool(self._config.get("auto_start", True))

    def stop(self) -> None:
        with self._lock:
            self._enabled = False
            self._generation += 1
            self._controller.reset()
        self._clear_announcements()
        self._wake.set()
        with self._action_lock:
            self._set_led("idle")

    def teardown(self) -> None:
        with self._lock:
            if self._torn_down:
                return
            self._torn_down = True
        self.stop()
        self._shutdown.set()
        self._wake.set()
        self._clear_announcements()
        try:
            self._announcement_queue.put_nowait(None)
        except queue.Full:
            pass
        self._timer_thread.join(timeout=2.0)
        self._announcement_thread.join(timeout=2.0)
        try:
            self._executor.remove_node(self._node)
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            with self._lock:
                if self._torn_down:
                    return {"error": "meeting facilitator has been torn down"}
                self._enabled = True
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "start_meeting":
            participants = args.get("participants")
            if not isinstance(participants, list):
                return {"error": "participants must be an array"}
            try:
                duration_s = float(args.get("duration_s", self._default_duration_s))
                with self._lock:
                    if not self._enabled:
                        return {"error": "meeting facilitator is not enabled"}
                    if self._controller.active:
                        return {"error": "a meeting is already active"}
                    self._generation += 1
                    generation = self._generation
                    speaker = self._controller.start(
                        participants, duration_s, time.monotonic()
                    )
            except (TypeError, ValueError) as exc:
                return {"error": str(exc)}
            self._wake.set()
            self._announce(
                f"会议开始，请{speaker}发言",
                gesture="right hand up",
                generation=generation,
            )
            return self._status()
        if action == "next_speaker":
            try:
                with self._lock:
                    speaker = self._controller.next_speaker(time.monotonic())
                    generation = self._generation
            except ValueError as exc:
                return {"error": str(exc)}
            self._wake.set()
            if speaker is None:
                return self._end_meeting()
            self._announce(
                f"下面请{speaker}发言",
                gesture="right hand up",
                generation=generation,
            )
            return self._status()
        if action == "pause":
            try:
                with self._lock:
                    self._controller.pause(time.monotonic())
            except ValueError as exc:
                return {"error": str(exc)}
            self._wake.set()
            self._announce("会议计时已暂停")
            return self._status()
        if action == "resume":
            try:
                with self._lock:
                    self._controller.resume(time.monotonic())
            except ValueError as exc:
                return {"error": str(exc)}
            self._wake.set()
            self._announce("会议计时继续")
            return self._status()
        if action == "add_action_item":
            description = args.get("description", "")
            owner = args.get("owner", "")
            if not isinstance(description, str) or not isinstance(owner, str):
                return {"error": "description and owner must be strings"}
            try:
                with self._lock:
                    self._controller.add_action_item(description, owner)
            except ValueError as exc:
                return {"error": str(exc)}
            return self._status()
        if action == "end_meeting":
            return self._end_meeting()
        if action == "status":
            return self._status()
        if action == "info":
            return {"topic_in": [{"topic": self._topic, "format": "data/json"}]}
        return None

    def _status(self) -> dict:
        with self._lock:
            return self._controller.status(time.monotonic())

    def _end_meeting(self) -> dict:
        with self._lock:
            if not self._controller.active:
                return {"error": "no meeting is active"}
            result = self._controller.status(time.monotonic())
            result["transcript"] = list(self._controller.transcript)
            self._controller.active = False
            self._controller.paused = False
            self._controller.deadline = None
            self._generation += 1
            generation = self._generation
        self._clear_announcements()
        self._wake.set()
        count = len(result["action_items"])
        self._announce(
            f"会议结束，共记录{count}项待办",
            gesture="clap",
            generation=generation,
        )
        result["active"] = False
        result["remaining_s"] = None
        return result

    def _on_text(self, text: str) -> None:
        if not self._enabled:
            return
        with self._lock:
            self._controller.add_transcript(text, time.time())

    def _timer_loop(self) -> None:
        while not self._shutdown.is_set():
            self._wake.clear()
            message = None
            generation = None
            wait_s = None
            with self._lock:
                controller = self._controller
                if self._enabled and controller.active and not controller.paused:
                    now = time.monotonic()
                    remaining = max(0.0, (controller.deadline or now) - now)
                    if remaining <= 0 and not controller.timeout_sent:
                        controller.timeout_sent = True
                        message = f"{controller.current_speaker}发言时间已到"
                        generation = self._generation
                    elif (
                        remaining <= self._warning_before_s
                        and not controller.warning_sent
                    ):
                        controller.warning_sent = True
                        seconds = max(1, int(round(remaining)))
                        message = f"还剩{seconds}秒"
                        generation = self._generation
                        wait_s = max(0.05, remaining)
                    elif remaining > 0:
                        wait_s = max(
                            0.05,
                            remaining - self._warning_before_s
                            if not controller.warning_sent
                            else remaining,
                        )
            if message:
                self._announce(message, generation=generation)
                continue
            self._wake.wait(wait_s)

    def _announce(
        self,
        text: str,
        gesture: Optional[str] = None,
        generation: Optional[int] = None,
    ) -> None:
        with self._lock:
            if not self._enabled or self._torn_down:
                return
            if generation is None:
                generation = self._generation
            announcement = Announcement(generation, text, gesture)
        try:
            self._announcement_queue.put_nowait(announcement)
        except queue.Full:
            print("[meeting_facilitator] announcement queue is full", flush=True)

    def _announcement_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                announcement = self._announcement_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if announcement is None:
                self._announcement_queue.task_done()
                break
            try:
                self._run_announcement(announcement)
            finally:
                self._announcement_queue.task_done()

    def _run_announcement(self, announcement: Announcement) -> None:
        with self._action_lock:
            if not self._is_current(announcement.generation):
                return
            speaking_since = time.monotonic()
            self._set_led("speaking")
            try:
                result = self._tts.dispatch(
                    "speak", {"text": announcement.text, "voice": 0}
                )
                self._raise_child_error("tts", result)
                if (
                    announcement.gesture
                    and self._arm is not None
                    and self._is_current(announcement.generation)
                ):
                    result = self._arm.dispatch(
                        "execute", {"gesture": announcement.gesture}
                    )
                    self._raise_child_error("arm", result)
            except Exception as exc:
                print(
                    f"[meeting_facilitator] announcement failed: {exc}",
                    flush=True,
                )
                if self._is_current(announcement.generation):
                    self._set_led("error")
                return
            remaining_hold = self._led_min_hold_s - (
                time.monotonic() - speaking_since
            )
            if remaining_hold > 0:
                self._shutdown.wait(remaining_hold)
            if self._is_current(announcement.generation):
                self._set_led("idle")

    def _is_current(self, generation: int) -> bool:
        with self._lock:
            return (
                self._enabled
                and not self._torn_down
                and generation == self._generation
            )

    def _clear_announcements(self) -> None:
        while True:
            try:
                self._announcement_queue.get_nowait()
            except queue.Empty:
                return
            else:
                self._announcement_queue.task_done()

    def _set_led(self, state: str) -> None:
        try:
            self._led.dispatch("state", {"state": state})
        except Exception as exc:
            print(f"[meeting_facilitator] LED failed: {exc}", flush=True)

    @staticmethod
    def _raise_child_error(name: str, result) -> None:
        if not isinstance(result, dict):
            return
        if result.get("error"):
            raise RuntimeError(f"{name}: {result['error']}")
        if result.get("ret") not in (None, 0):
            raise RuntimeError(f"{name}: ret={result['ret']}")


def make_plugin(plugin_config, namespace, executor, dependencies):
    return Plugin(plugin_config, namespace, executor, dependencies)
