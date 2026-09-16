"""Distance-triggered welcome card for the Unitree G1."""

from __future__ import annotations

import json
import math
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
    _DISTANCE_QOS = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )
except ImportError:
    _HAS_ROS2 = False
    Node = object


CARD = "greeting"
TYPE = "actuator"


@dataclass(frozen=True)
class GreetingObservation:
    distance_m: float


class GreetingController:
    """Debounce distance observations and manage one-shot welcome triggers."""

    def __init__(self, threshold_m: float = 2.0, consecutive_frames: int = 5,
                 cooldown_s: float = 15.0):
        if threshold_m <= 0:
            raise ValueError("threshold_m must be positive")
        if consecutive_frames < 1:
            raise ValueError("consecutive_frames must be at least 1")
        if cooldown_s < 0:
            raise ValueError("cooldown_s must not be negative")
        self.threshold_m = threshold_m
        self.consecutive_frames = consecutive_frames
        self.cooldown_s = cooldown_s
        self._inside_count = 0
        self._armed = True
        self._cooldown_until = 0.0
        self._last_distance_m: Optional[float] = None

    @property
    def last_distance_m(self) -> Optional[float]:
        return self._last_distance_m

    def update(self, observation: Optional[GreetingObservation],
               now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        distance = observation.distance_m if observation is not None else None
        self._last_distance_m = distance
        inside = (
            distance is not None
            and math.isfinite(distance)
            and 0 < distance <= self.threshold_m
        )

        if not inside:
            self._inside_count = 0
            self._armed = True
            return False

        if now < self._cooldown_until:
            return False

        self._inside_count += 1
        if self._armed and self._inside_count >= self.consecutive_frames:
            self._armed = False
            self._cooldown_until = now + self.cooldown_s
            return True
        return False

    def status(self, now: Optional[float] = None) -> dict:
        now = time.monotonic() if now is None else now
        return {
            "state": (
                "armed"
                if self._armed and now >= self._cooldown_until
                else "cooldown"
            ),
            "distance_m": self._last_distance_m,
            "inside_frames": self._inside_count,
            "threshold_m": self.threshold_m,
            "cooldown_remaining_s": round(
                max(0.0, self._cooldown_until - now), 2
            ),
        }


class GreetingDistanceNode(Node):
    def __init__(self, topic: str, on_distance):
        super().__init__("g1_greeting")
        self._on_distance = on_distance
        self._subscription = self.create_subscription(
            String, topic, self._handle_message, _DISTANCE_QOS
        )

    def _handle_message(self, message) -> None:
        try:
            payload = json.loads(message.data)
            distance_m = float(payload["distance_m"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if math.isfinite(distance_m):
            self._on_distance(distance_m)


class Plugin:
    """Welcome when camera distance remains within the configured threshold."""

    PREFIX = CARD

    def __init__(self, plugin_config, namespace, executor, dependencies):
        if not _HAS_ROS2:
            raise RuntimeError("greeting requires ROS2 rclpy and std_msgs")
        self._config = dict(plugin_config or {})
        self._executor = executor
        self._led = dependencies["led"]
        self._tts = dependencies["tts"]
        self._arm = dependencies["arm"]
        self._topic = str(self._config.get(
            "distance_topic", f"/{namespace}/camera/distance"
        ))
        self._controller = GreetingController(
            threshold_m=float(self._config.get("threshold_m", 2.0)),
            consecutive_frames=int(self._config.get("consecutive_frames", 5)),
            cooldown_s=float(self._config.get("cooldown_s", 15.0)),
        )
        self._text = str(self._config.get("text", "欢迎参观"))
        self._gesture = str(self._config.get("gesture", "high wave"))
        self._node = GreetingDistanceNode(self._topic, self._on_distance)
        self._executor.add_node(self._node)
        self._enabled = False
        self._action_lock = threading.Lock()

    def get_tool(self) -> dict:
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": (
                "G1 distance-triggered welcome with LED, speech, and gesture"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["enable", "disable", "status"],
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "enable": {"params": []},
                    "disable": {"params": []},
                    "status": {"params": []},
                },
            },
        }

    def start(self) -> None:
        self._enabled = bool(self._config.get("auto_start", True))

    def stop(self) -> None:
        self._enabled = False
        try:
            self._executor.remove_node(self._node)
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        del args
        if action == "start":
            self._enabled = True
            return {"state": "ready"}
        if action == "stop":
            self._enabled = False
            return {"state": "idle"}
        if action == "enable":
            self._enabled = True
            return {"state": "enabled"}
        if action == "disable":
            self._enabled = False
            return {"state": "disabled"}
        if action == "status":
            result = self._controller.status()
            result["enabled"] = self._enabled
            result["topic"] = self._topic
            return result
        return None

    def _on_distance(self, distance_m: float) -> None:
        if not self._enabled:
            return
        if not self._controller.update(GreetingObservation(distance_m)):
            return
        threading.Thread(
            target=self._welcome,
            daemon=True,
            name="g1_greeting_action",
        ).start()

    def _welcome(self) -> None:
        with self._action_lock:
            errors = []

            def speak():
                try:
                    result = self._tts.dispatch(
                        "speak", {"text": self._text, "voice": 0}
                    )
                    if isinstance(result, dict):
                        if result.get("error"):
                            errors.append(str(result["error"]))
                        elif result.get("ret") and result["ret"] != 0:
                            errors.append(f"tts ret={result['ret']}")
                except Exception as exc:
                    errors.append(f"TTS: {exc}")

            try:
                self._led.dispatch("state", {"state": "speaking"})
                speech_thread = threading.Thread(target=speak, daemon=True)
                speech_thread.start()
                arm_result = self._arm.dispatch(
                    "execute", {"gesture": self._gesture}
                )
                if isinstance(arm_result, dict):
                    if arm_result.get("error"):
                        errors.append(str(arm_result["error"]))
                    elif arm_result.get("ret") and arm_result["ret"] != 0:
                        errors.append(f"arm ret={arm_result['ret']}")
                speech_thread.join()
                if errors:
                    raise RuntimeError("; ".join(errors))
                self._led.dispatch("state", {"state": "idle"})
            except Exception as exc:
                print(f"[greeting] welcome action failed: {exc}", flush=True)
                try:
                    self._led.dispatch("state", {"state": "error"})
                except Exception:
                    pass


def make_plugin(plugin_config, namespace, executor, dependencies):
    """Create the independent greeting card using existing G1 plugins."""
    return Plugin(plugin_config, namespace, executor, dependencies)
