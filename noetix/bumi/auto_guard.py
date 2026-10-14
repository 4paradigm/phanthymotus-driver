"""Automatic, speech-only visitor greeting using Bumi's private whitelist.

The card consumes distinct frames from the existing camera cache. It never
saves visitor images, makes a physical access decision, or sends motor commands.
The canvas connects its JSON text output to Perception TTS and Bumi speaker.
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.request

try:
    from .guard_policy import GuardPolicy
except ImportError:  # the container copies both modules directly into /work
    from guard_policy import GuardPolicy


class AutoGuardPlugin:
    PREFIX = "auto_guard"

    def __init__(
        self, plugin_config: dict, namespace: str, executor, camera, whitelist,
        speaker, *, publisher=None, message_type=None, tts_probe=None,
    ) -> None:
        config = plugin_config or {}
        self._camera = camera
        self._whitelist = whitelist
        self._speaker = speaker
        self._input_topic = f"/{namespace}/camera/color"
        self._output_topic = f"/{namespace}/auto_guard/text"
        self._audio_topic = f"{self._output_topic}/tts"
        self._tts_instance_id = str(config.get("tts_instance_id", "bumi-guard-tts"))
        if not self._tts_instance_id:
            raise ValueError("tts_instance_id must identify the guard's TTS canvas card")
        self._tts_probe = tts_probe or self._perception_tts_running
        self._sample_interval_s = float(config.get("sample_interval_s", 0.5))
        if not 0.1 <= self._sample_interval_s <= 5.0:
            raise ValueError("sample_interval_s must be between 0.1 and 5 seconds")
        self._policy_kwargs = {
            "confirm_frames": int(config.get("confirm_frames", 3)),
            "rearm_no_face_frames": int(config.get("rearm_no_face_frames", 3)),
            "cooldown_s": float(config.get("cooldown_s", 30.0)),
            "max_frame_age_s": float(config.get("max_frame_age_s", 2.0)),
        }
        # A slower configured sample rate must still allow three valid frames
        # to confirm one visitor rather than silently resetting each vote.
        self._policy_kwargs["max_gap_s"] = max(
            2.5, self._sample_interval_s + self._policy_kwargs["max_frame_age_s"] + 0.5,
        )
        GuardPolicy(**self._policy_kwargs)  # validate configuration at startup

        if publisher is None:
            from rclpy.node import Node
            from rclpy.qos import (
                DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
            )
            from std_msgs.msg import String

            node = Node("bumi_auto_guard")
            executor.add_node(node)
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=200,
                durability=DurabilityPolicy.VOLATILE,
            )
            publisher = node.create_publisher(String, self._output_topic, qos)
            message_type = String
        if message_type is None:
            raise ValueError("message_type is required with an injected publisher")
        self._publisher = publisher
        self._message_type = message_type

        self._lifecycle_lock = threading.Lock()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._state = "idle"
        self._last_error: str | None = None
        self._last_event: str | None = None
        self._checked_frames = 0

    def get_tool(self) -> dict:
        return {
            "name": "auto_guard",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "Automatically confirms one visitor across fresh Bumi camera "
                "frames, then sends fixed greeting or neutral registration text "
                "to TTS. Never controls movement or grants physical access."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string", "enum": ["start", "stop", "info"],
                    },
                    "input_topic": {
                        "type": "string",
                        "description": "Connect the Bumi camera card's JPEG output.",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": ["input_topic"], "description": "Monitor the connected camera automatically."},
                    "stop": {"params": [], "description": "Stop monitoring and greeting."},
                    "info": {"params": [], "description": "Show pipeline and monitoring state."},
                },
            },
            "topic_in": [{"format": "image/jpeg"}],
            "topic_out": [{"topic": self._output_topic, "format": "data/json"}],
        }

    def start(self) -> None:
        # Bundle startup must not arm the guard: canvas start supplies the
        # camera connection and starts TTS/speaker downstream.
        pass

    def stop(self) -> dict:
        with self._lifecycle_lock:
            with self._lock:
                self._stop_event.set()
                thread = self._thread
                self._state = "stopping" if thread and thread.is_alive() else "idle"
            if thread and thread.is_alive() and thread is not threading.current_thread():
                thread.join(timeout=3.0)
            with self._lock:
                if self._thread is thread and (not thread or not thread.is_alive()):
                    self._thread = None
                    self._state = "idle"
                return {"state": self._state}

    def _pipeline_ready(self) -> bool:
        return bool(
            self._camera is not None
            and self._camera.is_running()
            and self._whitelist is not None
            and self._whitelist.is_ready()
            and self._speaker is not None
            and self._speaker.is_subscribed_to(self._audio_topic)
            and self._publisher.get_subscription_count() > 0
            and self._tts_probe()
        )

    def _perception_tts_running(self) -> bool:
        """Check this exact TTS instance, not Agent Core's monitoring subscriber."""
        payload = {
            "jsonrpc": "2.0", "id": 1, "method": "tools/call",
            "params": {
                "name": "tts",
                "arguments": {
                    "action": "info",
                    "instance_id": self._tts_instance_id,
                    "input_topic": self._output_topic,
                },
            },
        }
        request = urllib.request.Request(
            "http://127.0.0.1:15720/mcp",
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )
        try:
            with urllib.request.urlopen(request, timeout=0.8) as response:
                rpc = json.load(response)
            content = rpc.get("result", {}).get("content", [])
            if not content:
                return False
            info = json.loads(content[0].get("text", "{}"))
            return info.get("state") == "running" and any(
                entry.get("topic") == self._output_topic
                for entry in info.get("topic_in", [])
            )
        except (OSError, ValueError, KeyError, TypeError):
            return False

    def _run(self) -> None:
        policy = GuardPolicy(**self._policy_kwargs)
        sequence = None
        pending = None
        pending_until = 0.0
        last_valid_frame_ts = None

        def drop_pending() -> None:
            nonlocal pending
            if pending is not None:
                policy.abandon_unpublished()
            pending = None

        while not self._stop_event.is_set():
            try:
                if pending is not None and time.monotonic() > pending_until:
                    # Never replay an old greeting; reconfirm if nothing was sent.
                    drop_pending()
                if not self._pipeline_ready():
                    with self._lock:
                        self._state = "waiting_for_pipeline"
                    # Preserve announcement/cooldown across reconnection.
                    policy.pause()
                    sequence = None
                    self._stop_event.wait(self._sample_interval_s)
                    continue
                frame, sequence = self._camera.wait_for_color_frame(
                    after_sequence=sequence, timeout_s=1.0,
                )
                if self._stop_event.is_set():
                    break
                if not isinstance(frame, dict) or not frame.get("data"):
                    self._stop_event.wait(self._sample_interval_s)
                    continue
                frame_ts = frame.get("received_monotonic")
                now = time.monotonic()
                if (
                    isinstance(frame_ts, bool)
                    or not isinstance(frame_ts, (int, float))
                    or not math.isfinite(frame_ts)
                    or (last_valid_frame_ts is not None
                        and frame_ts <= last_valid_frame_ts)
                    or frame_ts > now + policy.future_slack_s
                    or now - frame_ts > policy.max_frame_age_s
                ):
                    drop_pending()
                    policy.pause()
                    self._stop_event.wait(self._sample_interval_s)
                    continue
                result = self._whitelist.recognize_jpeg(bytes(frame["data"]))
                now = time.monotonic()
                if now - frame_ts > policy.max_frame_age_s:
                    drop_pending()
                    policy.pause()
                    self._stop_event.wait(self._sample_interval_s)
                    continue
                last_valid_frame_ts = frame_ts
                with self._lock:
                    self._checked_frames += 1
                    if result.get("state") == "error":
                        self._last_error = str(result.get("error") or "recognition failed")
                        self._state = "error"
                    else:
                        self._last_error = None
                        self._state = "running"
                if result.get("state") == "error":
                    drop_pending()
                    policy.pause()
                    self._stop_event.wait(self._sample_interval_s)
                    continue
                observation = {"status": result.get("result", "error")}
                if result.get("person_id"):
                    observation["person_id"] = result["person_id"]
                if pending is not None:
                    same_visitor = (
                        (pending.kind == "known_visitor_arrived"
                         and observation["status"] == "matched"
                         and observation.get("person_id") == pending.person_id)
                        or (pending.kind == "visitor_needs_assistance"
                            and observation["status"] == "unknown")
                    )
                    if not same_visitor:
                        drop_pending()
                event = policy.observe(
                    observation, frame_ts=frame_ts, now=now,
                )
                if event is not None:
                    pending = event
                    pending_until = now + min(2.0, policy.max_frame_age_s)
                if pending is not None and now <= pending_until and self._pipeline_ready():
                    message = self._message_type()
                    message.data = json.dumps({"text": pending.text}, ensure_ascii=False)
                    delivered = None
                    with self._lock:
                        if (
                            not self._stop_event.is_set()
                            and time.monotonic() <= pending_until
                            and time.monotonic() - frame_ts <= policy.max_frame_age_s
                        ):
                            self._publisher.publish(message)
                            policy.mark_published()
                            delivered = pending.kind
                            drop_pending()
                            self._last_event = delivered
                    if delivered is not None:
                        print(f"[auto_guard] announced {delivered}", flush=True)
                self._stop_event.wait(self._sample_interval_s)
            except Exception as exc:
                with self._lock:
                    self._last_error = str(exc)
                    self._state = "error"
                # An unexpected camera/model/publisher error invalidates any
                # unsent decision; require three fresh frames on recovery.
                drop_pending()
                policy.pause()
                sequence = None
                print(f"[auto_guard] error: {exc}", flush=True)
                self._stop_event.wait(2.0)
        with self._lock:
            self._state = "idle"

    def _start_monitoring(self, args: dict) -> dict:
        input_topic = str(args.get("input_topic") or "").strip()
        if input_topic != self._input_topic:
            return {
                "state": "error",
                "error": f"connect the Bumi camera output {self._input_topic}",
            }
        if self._camera is None or self._whitelist is None or self._speaker is None:
            return {"state": "error", "error": "camera, face_whitelist and speaker are required"}
        if not self._whitelist.is_ready():
            return {"state": "error", "error": "load valid whitelist photos before starting auto_guard"}
        with self._lifecycle_lock:
            previous = self._thread
            if previous and previous.is_alive() and self._stop_event.is_set():
                previous.join(timeout=3.0)
            with self._lock:
                if self._thread and self._thread.is_alive():
                    if self._stop_event.is_set():
                        return {"state": "error", "error": "previous guard worker is still stopping; retry start"}
                    return {"state": self._state, "topic_out": self.get_tool()["topic_out"]}
                self._stop_event.clear()
                self._last_error = None
                self._last_event = None
                self._checked_frames = 0
                self._state = "waiting_for_pipeline"
                self._thread = threading.Thread(
                    target=self._run, name="bumi_auto_guard", daemon=True,
                )
                self._thread.start()
        return {
            "state": "running",
            "topic_in": [{"topic": self._input_topic, "format": "image/jpeg"}],
            "topic_out": self.get_tool()["topic_out"],
            "note": "Monitoring starts when TTS and speaker are connected; no physical access decision.",
        }

    def _info(self) -> dict:
        with self._lock:
            result = {
                "state": self._state,
                "checked_frames": self._checked_frames,
                "last_event": self._last_event,
                "last_error": self._last_error,
                "topic_in": [{"topic": self._input_topic, "format": "image/jpeg"}],
                "topic_out": self.get_tool()["topic_out"],
            }
        result["pipeline_ready"] = self._pipeline_ready()
        return result

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return self._start_monitoring(args)
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._info()
        return None


class GuardStatusPlugin:
    """Read-only LLM-facing summary of the automatic guard's health.

    Agent Core intentionally hides processor lifecycle actions from the LLM;
    ``auto_guard`` only declares those actions. This separate on-demand card
    exposes one user action, ``status``, without exposing visitor identities.
    Agent Core sends unadvertised ``start``/``stop``/``info`` during canvas
    lifecycle through its direct card endpoint; those remain safe no-ops or
    read-only snapshots, while the LLM schema permits only ``status``.
    """

    PREFIX = "guard_status"
    _STATES = frozenset({
        "idle", "stopping", "waiting_for_pipeline", "running", "error",
    })

    def __init__(self, guard: AutoGuardPlugin, whitelist) -> None:
        self._guard = guard
        self._whitelist = whitelist

    def get_tool(self) -> dict:
        return {
            "name": "guard_status",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Read-only Bumi guard health. `status` reports monitoring, "
                "pipeline and whitelist readiness; no visitor identity, "
                "photos, motion or access decisions. Use status only."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["status"],
                        "description": "Use status to query guard health.",
                    },
                },
                "required": ["action"],
            },
        }

    def start(self) -> None:
        """Canvas lifecycle hook; status queries never arm monitoring."""

    def stop(self) -> None:
        """Canvas lifecycle hook; stopping this card never stops monitoring."""

    def _snapshot(self) -> dict:
        try:
            guard_info = self._guard._info()
            if not isinstance(guard_info, dict):
                raise TypeError("guard info must be a dict")
        except Exception:
            guard_info = {}
            guard_error = True
        else:
            guard_error = bool(guard_info.get("last_error"))

        try:
            whitelist_ready = bool(
                self._whitelist is not None and self._whitelist.is_ready()
            )
        except Exception:
            whitelist_ready = False

        guard_state = guard_info.get("state")
        if not isinstance(guard_state, str) or guard_state not in self._STATES:
            guard_state = "unavailable"
        checked_frames = guard_info.get("checked_frames")
        if type(checked_frames) is not int or checked_frames < 0:
            checked_frames = 0
        return {
            "state": "ready",  # this query card is usable; monitor may not be
            "guard_state": guard_state,
            "pipeline_ready": guard_info.get("pipeline_ready") is True,
            "whitelist_ready": whitelist_ready,
            "checked_frames": checked_frames,
            "guard_has_error": guard_error,
        }

    def dispatch(self, action: str, args: dict) -> dict | None:
        del args
        if action == "start":
            self.start()
            return {"state": "ready"}
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action in ("info", "status"):
            return self._snapshot()
        return None
