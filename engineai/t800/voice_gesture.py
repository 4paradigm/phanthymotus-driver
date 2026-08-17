"""Optional T800 ASR-to-official-motion-plan extension.

The extension subscribes to final Perception ASR events and executes only the
T800 motion-plan YAML files supplied by EngineAI's ROS2 workspace.  It does not
use the driver's custom GesturePlugin or accept arbitrary joint values from
ASR text.
"""

from __future__ import annotations

import json
import re
import threading
import time
from pathlib import Path

import yaml
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
_RELIABLE_ONE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)
_PUNCTUATION = re.compile(r"[\s,，.。!！?？:：;；、]+")


def _now_ms() -> int:
    return int(time.time() * 1000)


def _normalise_text(value: str) -> str:
    return _PUNCTUATION.sub("", value).lower()


def _flatten(values: object) -> list[float]:
    if not isinstance(values, list):
        raise ValueError("stiffness and damping must be arrays")
    flattened: list[float] = []
    for value in values:
        if isinstance(value, list):
            flattened.extend(_flatten(value))
        else:
            flattened.append(float(value))
    return flattened


class VoiceGesturePlugin:
    """Execute configured official T800 motion plans after wake-word ASR."""

    def __init__(self, config: dict, namespace: str, ros2):
        plugin_config = config.get("plugins", {}).get("voice_gesture", {}) or {}
        self._config = plugin_config
        self._namespace = namespace
        self._topics = config["topics"]
        self._asr_topic = str(plugin_config.get("asr_topic") or f"/{namespace}/mic/audio/asr")
        self._events_topic = str(
            plugin_config.get("events_topic") or f"/{namespace}/voice_gesture/events"
        )
        self._motion_dir = Path(plugin_config.get("motion_dir", "/official-motions"))
        self._bundled_motion_dir = Path(__file__).resolve().parent / "motions"
        self._require_wake_word = bool(plugin_config.get("require_wake_word", True))
        self._cooldown_sec = max(0.0, float(plugin_config.get("cooldown_sec", 3.0)))
        self._ready_timeout = max(1.0, float(plugin_config.get("planner_ready_timeout_sec", 10.0)))
        self._step_timeout = max(1.0, float(plugin_config.get("planner_step_timeout_sec", 15.0)))
        self._motions = self._load_motion_catalog(plugin_config.get("motions", {}) or {})

        self._core_node = Node("t800_voice_gesture", context=ros2.ctx_core)
        self._plan_node = Node("t800_voice_gesture_plan", context=ros2.ctx_robot)
        ros2.executor_core.add_node(self._core_node)
        ros2.executor_robot.add_node(self._plan_node)
        self._events_pub = None
        self._request_pub = None
        self._request_type = None
        self._state_type = None
        self._started = False
        self._enabled = False
        self._planner_state = None
        self._planner_request_id = None
        self._active_request_id = None
        self._last_action_at = 0.0
        self._thread: threading.Thread | None = None
        self._cancel = threading.Event()
        self._lock = threading.RLock()
        self._planner_changed = threading.Condition(self._lock)
        self._status = {
            "state": "idle",
            "last_text": "",
            "last_motion": "",
            "last_error": "",
            "events_published": 0,
        }

    @staticmethod
    def _load_motion_catalog(definitions: dict) -> dict[str, dict]:
        catalog: dict[str, dict] = {}
        for motion_id, definition in definitions.items():
            if not isinstance(definition, dict):
                continue
            filename = str(definition.get("file", ""))
            if not filename or Path(filename).name != filename or not filename.endswith((".yaml", ".yml")):
                continue
            phrases = definition.get("phrases", [])
            if not isinstance(phrases, list):
                continue
            catalog[str(motion_id)] = {
                "description": str(definition.get("description", motion_id)),
                "file": filename,
                "phrases": [_normalise_text(str(item)) for item in phrases if str(item).strip()],
            }
        return catalog

    def get_tool(self) -> dict:
        return {
            "name": "voice_gesture",
            "type": "processor",
            "multiInstance": False,
            "description": "T800 ASR 到众擎官方关节运动规划 YAML 的安全路由",
            "inputSchema": action_schema(
                {
                    "start": ([], "开始处理 ASR 最终结果"),
                    "stop": ([], "取消当前官方运动规划并停止语音动作"),
                    "info": ([], "查询 ASR、官方规划器和动作运行状态"),
                },
                {},
                "语音手势动作",
            ),
            "topic_in": [{"topic": self._asr_topic, "format": "data/json", "desc": "Perception ASR final result"}],
            "topic_out": [{"topic": self._events_topic, "format": "data/json", "desc": "voice gesture event"}],
        }

    def start(self) -> None:
        if self._started:
            self._enabled = True
            return
        from interface_protocol.msg import JointMotionPlanRequest, JointMotionPlanState

        self._request_type = JointMotionPlanRequest
        self._state_type = JointMotionPlanState
        self._events_pub = self._core_node.create_publisher(String, self._events_topic, _BEST_EFFORT)
        self._core_node.create_subscription(String, self._asr_topic, self._on_asr, _BEST_EFFORT)
        self._request_pub = self._plan_node.create_publisher(
            JointMotionPlanRequest, self._topics["joint_plan_request"], _RELIABLE_ONE
        )
        self._plan_node.create_subscription(
            JointMotionPlanState, self._topics["joint_plan_state"], self._on_planner_state, _BEST_EFFORT
        )
        self._started = True
        self._enabled = True
        self._set_status(state="running", last_error="")
        self._publish_event("ready", motions=self._motion_catalog())

    def stop(self) -> None:
        self._enabled = False
        self._cancel.set()
        self._cancel_active_request()
        self._set_status(state="idle")

    def dispatch(self, action: str, _args: dict) -> dict:
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
        if not isinstance(payload, dict) or payload.get("is_final") is False or payload.get("final") is False:
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
        motion_id = self._match_motion(text)
        if motion_id is None:
            self._reject("no_motion_match", text=text)
            return
        self._start_motion(motion_id, text)

    def _on_planner_state(self, message) -> None:
        with self._planner_changed:
            self._planner_state = int(message.status)
            self._planner_request_id = int(message.request_id)
            self._planner_changed.notify_all()

    def _match_motion(self, text: str) -> str | None:
        normalised = _normalise_text(text)
        for motion_id, definition in self._motions.items():
            if normalised in definition["phrases"]:
                return motion_id
        return None

    def _start_motion(self, motion_id: str, text: str) -> None:
        now = time.monotonic()
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                self._reject("motion_already_running", text=text, motion_id=motion_id)
                return
            if now - self._last_action_at < self._cooldown_sec:
                self._reject("cooldown", text=text, motion_id=motion_id)
                return
            self._last_action_at = now
            self._cancel = threading.Event()
            self._status.update({"state": "running", "last_text": text, "last_motion": motion_id, "last_error": ""})
            self._thread = threading.Thread(
                target=self._run_motion, args=(motion_id, text), daemon=True, name="t800-voice-gesture"
            )
            self._thread.start()
        self._publish_event("motion_started", motion_id=motion_id, text=text)

    def _run_motion(self, motion_id: str, text: str) -> None:
        try:
            steps = self._load_official_motion(motion_id)
            self._wait_for_idle(self._ready_timeout)
            for step in steps:
                if self._cancel.is_set():
                    raise RuntimeError("motion cancelled")
                request_id = self._publish_step(step)
                self._wait_for_execution(request_id, self._step_timeout)
                self._wait_for_idle(self._step_timeout, minimum_request_id=request_id)
            self._set_status(state="completed")
            self._publish_event("motion_completed", motion_id=motion_id, text=text)
        except Exception as exc:  # noqa: BLE001
            cancelled = self._cancel.is_set()
            self._set_status(state="cancelled" if cancelled else "error", last_error=str(exc))
            self._publish_event("motion_cancelled" if cancelled else "motion_error",
                                motion_id=motion_id, text=text, error=str(exc))
        finally:
            with self._lock:
                self._active_request_id = None

    def _load_official_motion(self, motion_id: str) -> list[dict]:
        definition = self._motions[motion_id]
        path = self._motion_dir / definition["file"]
        if not path.is_file():
            bundled_path = self._bundled_motion_dir / definition["file"]
            if bundled_path.is_file():
                path = bundled_path
            else:
                raise FileNotFoundError(f"official motion file not found: {path}")
        payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        motion_plan = payload.get("motion_plan")
        if not isinstance(motion_plan, list) or len(motion_plan) < 2:
            raise ValueError("official motion file has no motion_plan tasks")
        header = motion_plan[0]
        indices = header.get("joint_indices") if isinstance(header, dict) else None
        if not isinstance(indices, list) or not indices:
            raise ValueError("official motion file has no joint_indices")
        indices = [int(index) for index in indices]
        if len(set(indices)) != len(indices) or any(index < 0 or index > 24 for index in indices):
            raise ValueError("official motion file has invalid joint_indices")
        steps = []
        for item in motion_plan[1:]:
            if not isinstance(item, dict) or "motion" not in item:
                raise ValueError("official motion file contains an invalid task")
            if str(item.get("request_type", "")).upper() == "RESET":
                steps.append({"reset": True, "name": str(item["motion"])})
                continue
            positions = item.get("target_positions")
            if not isinstance(positions, list) or len(positions) != len(indices):
                raise ValueError(f"motion {item['motion']} has invalid target_positions")
            duration = float(item.get("duration", 2.0))
            if not 0.05 <= duration <= 120.0:
                raise ValueError(f"motion {item['motion']} has invalid duration")
            steps.append({
                "name": str(item["motion"]),
                "joint_indices": indices,
                "target_positions": [float(value) for value in positions],
                "duration": duration,
                "gravity_compensation": bool(item.get("use_gravity_compensation", True)),
                "stiffness": _flatten(item.get("stiffness", [])) if item.get("stiffness") else [],
                "damping": _flatten(item.get("damping", [])) if item.get("damping") else [],
            })
        return steps

    def _wait_for_idle(self, timeout: float, *, minimum_request_id: int | None = None) -> None:
        idle = int(self._state_type.IDLE)
        def ready() -> bool:
            return (
                self._planner_state == idle
                and self._planner_request_id is not None
                and (minimum_request_id is None or self._planner_request_id >= minimum_request_id)
            )
        self._wait_for(ready, timeout, "planner did not become IDLE")

    def _wait_for_execution(self, request_id: int, timeout: float) -> None:
        executing = int(self._state_type.EXECUTING)
        self._wait_for(
            lambda: self._planner_request_id == request_id and self._planner_state == executing,
            timeout,
            f"planner did not accept request {request_id}",
        )

    def _wait_for(self, predicate, timeout: float, error: str) -> None:
        deadline = time.monotonic() + timeout
        with self._planner_changed:
            while not predicate():
                if self._cancel.is_set():
                    raise RuntimeError("motion cancelled")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(error)
                self._planner_changed.wait(timeout=min(remaining, 0.2))

    def _publish_step(self, step: dict) -> int:
        with self._planner_changed:
            if self._planner_request_id is None:
                raise RuntimeError("planner request_id is unavailable")
            request_id = self._planner_request_id + 1
        message = self._request_type()
        message.request_id = request_id
        if step.get("reset"):
            message.request_type = self._request_type.REQUEST_RESET
            message.use_gravity_compensation = False
            message.joint_indices = []
            message.target_positions = []
            message.target_velocities = []
            message.execution_time = 0.0
            message.stiffness = []
            message.damping = []
        else:
            message.request_type = self._request_type.REQUEST_PLAN_EXECUTE
            message.use_gravity_compensation = step["gravity_compensation"]
            message.joint_indices = step["joint_indices"]
            message.target_positions = step["target_positions"]
            message.target_velocities = []
            message.execution_time = step["duration"]
            message.stiffness = step["stiffness"]
            message.damping = step["damping"]
        self._request_pub.publish(message)
        with self._lock:
            self._active_request_id = request_id
        self._publish_event("plan_published", request_id=request_id, step=step["name"])
        return request_id

    def _cancel_active_request(self) -> None:
        if self._request_pub is None or self._request_type is None:
            return
        with self._lock:
            if self._active_request_id is None or self._planner_request_id is None:
                return
            request_id = self._planner_request_id + 1
        message = self._request_type()
        message.request_id = request_id
        message.request_type = self._request_type.REQUEST_CANCEL
        message.use_gravity_compensation = False
        message.joint_indices = []
        message.target_positions = []
        message.target_velocities = []
        message.execution_time = 0.0
        message.stiffness = []
        message.damping = []
        self._request_pub.publish(message)

    def _motion_catalog(self) -> list[dict]:
        return [
            {"motion_id": motion_id, "description": definition["description"], "file": definition["file"]}
            for motion_id, definition in self._motions.items()
        ]

    def _info(self) -> dict:
        with self._lock:
            status = dict(self._status)
            planner_state = self._planner_state
            planner_request_id = self._planner_request_id
        return {
            **status,
            "motions": self._motion_catalog(),
            "motion_dir": str(self._motion_dir),
            "require_wake_word": self._require_wake_word,
            "planner_state": planner_state,
            "planner_request_id": planner_request_id,
            "topic_in": [{"topic": self._asr_topic, "format": "data/json"}],
            "topic_out": [{"topic": self._events_topic, "format": "data/json"}],
        }

    def _set_status(self, **values) -> None:
        with self._lock:
            self._status.update(values)

    def _reject(self, reason: str, **fields) -> None:
        self._publish_event("ignored", reason=reason, **fields)

    def _publish_event(self, event_type: str, **fields) -> None:
        if self._events_pub is None:
            return
        message = String()
        message.data = json.dumps(
            {"event": event_type, "timestamp_ms": _now_ms(), **fields},
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self._events_pub.publish(message)
        with self._lock:
            self._status["events_published"] += 1
