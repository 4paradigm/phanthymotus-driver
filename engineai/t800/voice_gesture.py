"""Optional T800 voice-gesture extension.

This module deliberately contains no ASR model client and no LLM client.  It
consumes final events published by Perception and performs only deterministic,
configuration-whitelisted actions locally.
"""

from __future__ import annotations

import json
import re
import threading
import time

from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from control import action_schema


_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
_PUNCTUATION = re.compile(r"[\s,，.。!！?？:：;；、]+")


def _now_ms() -> int:
    return int(time.time() * 1000)


def _normalise_text(value: str) -> str:
    return _PUNCTUATION.sub("", value).lower()


class VoiceGesturePlugin:
    """Route final Perception ASR events to approved T800 actions.

    The ASR text is matched exactly after whitespace and common punctuation are
    removed.  It is never treated as a tool name, Python expression, or joint
    command.
    """

    def __init__(self, config: dict, namespace: str, ros2, targets: dict[str, object]):
        plugin_config = config.get("plugins", {}).get("voice_gesture", {}) or {}
        self._config = plugin_config
        self._ns = namespace
        self._targets = dict(targets)
        self._asr_topic = str(plugin_config.get("asr_topic") or f"/{namespace}/mic/audio/asr")
        base_topic = f"/{namespace}/voice_gesture"
        self._events_topic = str(plugin_config.get("events_topic") or f"{base_topic}/events")
        self._require_wake_word = bool(plugin_config.get("require_wake_word", True))
        self._cooldown_sec = max(0.0, float(plugin_config.get("cooldown_sec", 3.0)))
        self._actions = self._load_actions(plugin_config.get("actions", {}) or {})
        self._node = Node("t800_voice_gesture", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._node)
        self._events_pub = None
        self._started = False
        self._enabled = False
        self._last_action_at = 0.0
        self._status = {
            "state": "idle",
            "last_text": "",
            "last_action_id": "",
            "last_error": "",
            "events_published": 0,
        }
        self._lock = threading.RLock()

    def _load_actions(self, definitions: dict) -> dict[str, dict]:
        actions: dict[str, dict] = {}
        for action_id, raw_definition in definitions.items():
            if not isinstance(raw_definition, dict):
                continue
            target = raw_definition.get("target", {}) or {}
            tool = str(target.get("tool", ""))
            action = str(target.get("action", ""))
            if not tool or not action or tool not in self._targets:
                continue
            phrases = raw_definition.get("phrases", [])
            if not isinstance(phrases, list):
                phrases = []
            actions[str(action_id)] = {
                "description": str(raw_definition.get("description", action_id)),
                "phrases": [_normalise_text(str(value)) for value in phrases if str(value).strip()],
                "tool": tool,
                "action": action,
                "arguments": dict(target.get("arguments", {}) or {}),
            }
        return actions

    def get_tool(self) -> dict:
        return {
            "name": "voice_gesture",
            "type": "processor",
            "multiInstance": False,
            "description": "T800 ASR 语音指令路由；仅执行配置白名单中的固定动作",
            "inputSchema": action_schema(
                {
                    "start": ([], "开始接收 ASR 最终结果"),
                    "stop": ([], "停止执行语音动作，但保留诊断订阅"),
                    "info": ([], "查询 Topic、已声明动作和运行状态"),
                },
                {},
                "语音控制动作",
            ),
            "topic_in": [{"topic": self._asr_topic, "format": "data/json", "desc": "Perception ASR final result"}],
            "topic_out": [{"topic": self._events_topic, "format": "data/json", "desc": "voice control event"}],
        }

    def start(self) -> None:
        if self._started:
            self._enabled = True
            return
        self._events_pub = self._node.create_publisher(String, self._events_topic, _BEST_EFFORT)
        self._node.create_subscription(String, self._asr_topic, self._on_asr, _BEST_EFFORT)
        self._started = True
        self._enabled = True
        self._set_status(state="running", last_error="")
        self._publish_event("ready", actions=self._action_catalog())

    def stop(self) -> None:
        self._enabled = False
        self._set_status(state="idle")

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            self.start()
            return self._info()
        if action == "stop":
            self.stop()
            return self._info()
        if action == "info":
            return self._info()
        return {"error": f"unknown voice_gesture action: {action}"}

    def _on_asr(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self._reject("invalid_asr_json")
            return
        if not isinstance(payload, dict):
            self._reject("invalid_asr_payload")
            return
        if payload.get("is_final") is False or payload.get("final") is False:
            return
        text = str(payload.get("text") or payload.get("transcript") or "").strip()
        if not text:
            self._reject("empty_transcript")
            return
        if self._require_wake_word and not bool(payload.get("kws_triggered", False)):
            self._reject("wake_word_required", text=text)
            return
        if not self._enabled:
            self._reject("voice_gesture_stopped", text=text)
            return
        self._set_status(last_text=text, last_error="")
        action_id = self._match_action(text)
        if action_id is None:
            self._reject("no_rule_match", text=text)
            return
        self._execute_action(action_id, text=text)

    def _match_action(self, text: str) -> str | None:
        normalized = _normalise_text(text)
        for action_id, definition in self._actions.items():
            if normalized in definition["phrases"]:
                return action_id
        return None

    def _execute_action(self, action_id: str, *, text: str) -> dict:
        definition = self._actions.get(action_id)
        if definition is None:
            self._reject("unknown_action_id", action_id=action_id)
            return {"error": "unknown action_id"}
        now = time.monotonic()
        if now - self._last_action_at < self._cooldown_sec:
            self._reject("cooldown", action_id=action_id, text=text)
            return {"state": "ignored", "reason": "cooldown", "action_id": action_id}
        target = self._targets.get(definition["tool"])
        if target is None:
            self._reject("target_unavailable", action_id=action_id)
            return {"error": "configured target is unavailable"}
        try:
            result = target.dispatch(definition["action"], dict(definition["arguments"]))
        except Exception as exc:  # noqa: BLE001
            self._set_status(last_error=str(exc))
            self._publish_event("action_error", action_id=action_id, error=str(exc), text=text)
            return {"error": str(exc), "action_id": action_id}
        self._last_action_at = now
        self._set_status(last_action_id=action_id, last_error="")
        event = {
            "action_id": action_id,
            "text": text,
            "source": "rules",
            "result": result,
        }
        self._publish_event("action_dispatched", **event)
        return {"state": "dispatched", **event}

    def _action_catalog(self) -> list[dict]:
        return [
            {"action_id": action_id, "description": definition["description"]}
            for action_id, definition in self._actions.items()
        ]

    def _info(self) -> dict:
        with self._lock:
            status = dict(self._status)
        return {
            **status,
            "require_wake_word": self._require_wake_word,
            "actions": self._action_catalog(),
            "topic_in": [{"topic": self._asr_topic, "format": "data/json"}],
            "topic_out": [{"topic": self._events_topic, "format": "data/json"}],
        }

    def _set_status(self, **values) -> None:
        with self._lock:
            self._status.update(values)

    def _reject(self, reason: str, **fields) -> None:
        self._publish_event("ignored", reason=reason, **fields)

    def _publish_event(self, event_type: str, **fields) -> None:
        payload = {"event": event_type, "timestamp_ms": _now_ms(), **fields}
        self._publish_json(self._events_pub, payload)
        with self._lock:
            self._status["events_published"] += 1

    @staticmethod
    def _publish_json(publisher, payload: dict) -> None:
        if publisher is None:
            return
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(message)
