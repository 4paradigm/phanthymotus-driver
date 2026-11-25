#!/usr/bin/env python3
"""ROS/device side of the Bumi music beat performance card.

The card publishes the decoded upload as ``audio/pcm-16k`` (consumed by the
existing speaker card) and, on the beats chosen by ``beat_performance_spec``,
sends semantic-action edges plus a small bounded yaw pulse.  All command writes
go through LocoPlugin's shared rate limiter, so the >=2 ms vendor interval holds
even when another card is driving the body.

It deliberately never touches MediaController, volume, or the speaker card's
configuration -- the 500 ms ``set_*`` red line stays entirely inside speaker.
"""

from __future__ import annotations

import json
import os
import ssl
import threading
import time
import urllib.request
import uuid

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from audio_msgs.msg import AudioChunk

from beat_performance_spec import (
    ACTION_COMMANDS,
    DEFAULT_ACTION_CYCLE,
    SAMPLE_RATE,
    BeatSpecError,
    build_choreography,
    decode_audio,
    detect_beats,
    iter_audio_frames,
    normalise_config,
    protection_result,
    status_payload,
)

_LOW_LAT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=200,
    durability=DurabilityPolicy.VOLATILE,
)

# workmode values from which a vendor action command is meaningful.  26 is
# protection and is refused explicitly; everything else is treated as unsafe.
_ACTION_WORKMODES = frozenset((2, 5, 8, 9, 10, 31, 32, 33))
_PROTECTION_MODE = 26


def _acp_notify(action_id: str, status: str, result: dict) -> None:
    """Report the terminal state of a long performance through ACP.

    Same endpoint and envelope as the bundle's other asynchronous actuators;
    kept local so this card module stays independently loadable in tests.
    """
    try:
        url = os.environ.get("AGENT_CORE_URL", "https://localhost:15678").rstrip("/")
        payload = json.dumps({
            "action_id": action_id, "status": status, "result": result,
            "tool": "beat_performance", "ts": time.time(),
        }).encode()
        request = urllib.request.Request(
            f"{url}/api/acp/complete", data=payload,
            headers={"Content-Type": "application/json"}, method="POST")
        context = ssl._create_unverified_context() if url.startswith("https://") else None
        with urllib.request.urlopen(request, timeout=5, context=context) as response:
            response.read()
    except Exception as exc:
        print(f"[Bumi ACP] callback failed for {action_id}: {exc}", flush=True)


class _BeatPerformanceNode(Node):
    """Owns the one output topic; no polling and no hardware access."""

    def __init__(self, namespace: str):
        super().__init__("bumi_beat_performance")
        ns = namespace.strip("/") or "bumi"
        self.topic = f"/{ns}/beat_performance/audio"
        self._publisher = self.create_publisher(AudioChunk, self.topic, 10)

    def publish_frame(self, pcm: bytes, stamp_ns: int) -> None:
        message = AudioChunk()
        message.format = "audio/pcm-16k"
        message.data = pcm
        message.header.stamp.sec = stamp_ns // 1_000_000_000
        message.header.stamp.nanosec = stamp_ns % 1_000_000_000
        self._publisher.publish(message)


class BeatPerformancePlugin:
    PREFIX = "beat_performance"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 high_ctrl, loco_plugin):
        if high_ctrl is None or loco_plugin is None:
            raise ValueError("beat_performance needs both high_ctrl and loco")
        self._high_ctrl = high_ctrl
        self._loco = loco_plugin
        self._namespace = namespace
        self._config = normalise_config(plugin_config)
        self._node = _BeatPerformanceNode(namespace)
        executor.add_node(self._node)

        self._state = "idle"
        self._audio_file: str | None = None
        self._beat_count = 0
        self._beat_source = "none"
        self._event_count = 0
        self._events_fired = 0
        self._action_count = 0
        self._turn_count = 0
        self._frames_published = 0
        self._duration_s = 0.0
        self._elapsed_s = 0.0
        self._last_action: str | None = None
        self._last_action_drift_ms: float | None = None
        self._protection = False
        self._last_error: str | None = None

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._generation = 0
        self._thread: threading.Thread | None = None
        # One ACP action per run.  The worker claims it under the lock before
        # notifying so a concurrent stop cannot produce a second terminal POST.
        self._active_action: dict | None = None

    # ── tool declaration ─────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "beat_performance",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Bumi 音乐节拍表演卡：上传 WAV/16k PCM16，分析节拍并输出两路 —— "
                "audio/pcm-16k 接到 speaker 卡播放；节拍点按 spec 编排挥手/欢呼/舞蹈，"
                "并叠加小幅转身脉冲。仅在 workmode=2 或动作模式执行；保护模式 26 直接拒绝。"
                "所有动作命令边沿触发，其余轮次发 DEFAULT；stop 会取消后续节拍。"
                "使用前确认机器人稳定站立、四肢活动范围内无人和障碍物，舞蹈至少留出 3m×3m 空间。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info", "config"],
                        "description": "start=上传并播放；stop=立即停止并取消后续节拍；info=查询节拍与命令统计",
                    },
                    "audio_file": {
                        "type": "string",
                        "format": "file",
                        "accept": ".wav,.pcm",
                        "uploadTo": "mcp",
                        "description": (
                            "上传的 WAV 或裸 PCM 文件。WAV 必须为 16 kHz、16-bit、单声道或立体声；"
                            "裸 PCM 解释为 16 kHz 16-bit 小端单声道。最长 120 秒。"
                        ),
                    },
                    "confirm": {
                        "type": "boolean",
                        "description": (
                            "必须显式传 true 才会驱动机器人。未确认或为 false 时直接拒绝，"
                            "不准备模式、不发送任何命令。"
                        ),
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {
                        "params": ["audio_file", "confirm"],
                        "description": (
                            "解码上传文件、分析节拍，开始连播音频与踩点动作；"
                            "confirm 必须为 true，确认已留出安全空间并使用吊架"
                        ),
                    },
                    "stop": {"params": [], "description": "立即停止：取消后续节拍并复位命令"},
                    "info": {"params": [], "description": "查询节拍数、已触发动作、漂移与保护模式状态"},
                    "config": {"params": [], "description": "调整动作周期、冷却与转身脉冲参数（仅空闲时生效）"},
                },
                "x-hooks": {
                    "on_interrupt_motion": {"action": "stop"},
                    "on_interrupt_all": {"action": "stop"},
                },
                "x-is-dangerous": True,
                "x-resource": ["base", "arm_l", "arm_r"],
                "x-completion": {
                    "actions": ["start"],
                    "timeout": 165,
                },
            },
            "configSchema": {
                "type": "object",
                "properties": {
                    "action_cycle": {
                        "type": "string",
                        "default": ",".join(DEFAULT_ACTION_CYCLE),
                        "description": (
                            "语义动作顺序，逗号分隔；可选 "
                            + "/".join(ACTION_COMMANDS)
                        ),
                    },
                    "action_every_beats": {"type": "integer", "default": 4, "minimum": 1, "maximum": 32,
                                           "description": "每隔几个节拍触发一个语义动作"},
                    "turn_every_beats": {"type": "integer", "default": 8, "minimum": 1, "maximum": 32,
                                         "description": "每隔几个节拍触发一次转身脉冲"},
                    "action_cooldown_s": {"type": "number", "default": 3.0, "minimum": 0.5, "maximum": 30.0,
                                          "description": "两次语义动作最小间隔（秒）"},
                    "turn_cooldown_s": {"type": "number", "default": 4.0, "minimum": 0.5, "maximum": 30.0,
                                        "description": "两次转身脉冲最小间隔（秒）"},
                    "turn_vyaw": {"type": "number", "default": 0.5, "minimum": -1.0, "maximum": 1.0,
                                  "description": "归一化转身速度；0 表示关闭脉冲"},
                    "turn_frames": {"type": "integer", "default": 3, "minimum": 1, "maximum": 10,
                                    "description": "转身脉冲持续的音频帧数（每帧默认 20 ms）"},
                    "frame_ms": {"type": "integer", "default": 20, "minimum": 10, "maximum": 100,
                                 "description": "音频分帧时长（毫秒）"},
                    "fallback_bpm": {"type": "number", "default": 120.0, "minimum": 40.0, "maximum": 240.0,
                                     "description": "检测不到节拍时的兜底节拍速度"},
                    "max_duration_s": {"type": "number", "default": 120.0, "minimum": 1.0, "maximum": 120.0,
                                       "description": "上传音频最大时长（秒）"},
                },
            },
            "topic_out": [{
                "topic": self._node.topic,
                "format": "audio/pcm-16k",
                "message_type": "audio_msgs/msg/AudioChunk",
            }],
        }

    def start(self) -> None:
        pass  # The card starts per upload through tools/call.

    def stop(self) -> None:
        self._stop()

    # ── lifecycle ────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop("_tool_name", None)
        if action == "config":
            return self._apply_config(args)
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "info":
            return self._info()
        return None

    def _apply_config(self, args: dict) -> dict:
        with self._lock:
            if self._state == "running":
                return {"state": "error", "error": "stop the performance before changing config"}
            try:
                self._config = normalise_config(args)
            except BeatSpecError as exc:
                return {"state": "error", "error": str(exc)}
            return {"state": "configured", "action_cycle": ",".join(self._config["action_cycle"])}

    def _start(self, args: dict) -> dict:
        # An x-is-dangerous card must never move the body on an implicit
        # confirmation.  Refuse before even decoding or preparing workmode.
        if args.get("confirm") is not True:
            return {
                "state": "error", "command_sent": False,
                "error": "start requires confirm=true; no command was sent",
            }
        audio_file = args.get("audio_file")
        if not isinstance(audio_file, str) or not audio_file.strip():
            return {"state": "error", "error": "audio_file is required"}
        action_id = f"bumi_beat_performance_{uuid.uuid4().hex}"
        active = {"id": action_id, "notified": False}
        with self._lock:
            if self._state == "running":
                return {"state": "error", "error": "performance already running; stop it first"}
            # The worker validates and decodes the upload before it reserves
            # or prepares the body: a bad file must never change workmode.
            self._stop_event.clear()
            self._generation += 1
            generation = self._generation
            self._state = "running"
            self._audio_file = audio_file.strip()
            self._active_action = active
            self._reset_statistics()
            self._thread = threading.Thread(
                target=self._run, args=(generation, self._audio_file, active),
                daemon=True, name="bumi_beat_performance",
            )
            self._thread.start()
        return {"state": "running", "action_id": action_id,
                "expected_duration_s": self._config["max_duration_s"]}

    def _stop(self) -> dict:
        with self._lock:
            self._stop_event.set()
            self._generation += 1
            thread = self._thread
        # The worker owns the final DEFAULT and the reservation release, and
        # both happen before join returns -- nothing can write after stop.
        if thread is not None and thread.is_alive():
            thread.join(timeout=8.0)
        with self._lock:
            if self._state == "running":
                self._state = "idle"
            self._thread = None
        return {"state": "idle"}

    def _reset_statistics(self) -> None:
        self._beat_count = 0
        self._beat_source = "none"
        self._event_count = 0
        self._events_fired = 0
        self._action_count = 0
        self._turn_count = 0
        self._frames_published = 0
        self._duration_s = 0.0
        self._elapsed_s = 0.0
        self._last_action = None
        self._last_action_drift_ms = None
        self._protection = False
        self._last_error = None

    def _info(self) -> dict:
        with self._lock:
            state = self._state
            payload = status_payload(
                state=state, audio_file=self._audio_file,
                beat_count=self._beat_count, beat_source=self._beat_source,
                event_count=self._event_count, events_fired=self._events_fired,
                action_count=self._action_count, turn_count=self._turn_count,
                frames_published=self._frames_published, duration_s=self._duration_s,
                elapsed_s=self._elapsed_s, last_action=self._last_action,
                last_action_drift_ms=self._last_action_drift_ms,
                workmode=self._read_workmode(), protection=self._protection,
                last_error=self._last_error,
            )
            payload["topic_out"] = self.get_tool()["topic_out"]
        return payload

    # ── worker ───────────────────────────────────────────────────────────────

    def _cancelled(self, generation: int) -> bool:
        return self._stop_event.is_set() or generation != self._generation

    def _read_workmode(self) -> int | None:
        try:
            return int(self._high_ctrl.get_mode())
        except Exception as exc:
            with self._lock:
                self._last_error = f"get_mode: {exc}"
            return None

    def _fail(self, generation: int, message: str) -> None:
        with self._lock:
            if generation == self._generation:
                self._state = "error"
                self._last_error = message

    def _complete(self, active: dict, status: str, result: dict) -> None:
        """Publish exactly one ACP terminal event for this run."""
        with self._lock:
            if active is not self._active_action or active["notified"]:
                return
            active["notified"] = True
            self._active_action = None
        _acp_notify(active["id"], status, result)

    def _run(self, generation: int, audio_file: str, active: dict) -> None:
        owns_body = False
        try:
            if self._cancelled(generation):
                return
            # Validate the entire plan first.  A missing file, a bad WAV or an
            # over-long clip must not send START/SWITCH/WALK and leave the
            # robot in a different workmode with no performance to run.
            config = dict(self._config)
            audio = decode_audio(
                audio_file, max_duration_s=config["max_duration_s"])
            beats, beat_source = detect_beats(
                audio, fallback_bpm=config["fallback_bpm"])
            events = build_choreography(beats, config)
            with self._lock:
                self._beat_count = len(beats)
                self._beat_source = beat_source
                self._event_count = len(events)
                self._duration_s = audio.duration_s
            if self._cancelled(generation):
                return

            # Protection mode blocks every command.  Refuse before taking the
            # body or writing anything -- not even a DEFAULT in cleanup.
            mode = self._read_workmode()
            if mode == _PROTECTION_MODE:
                with self._lock:
                    self._protection = True
                self._fail(generation,
                           protection_result("beat_performance", mode)["error"])
                return

            # Only a fully validated, unprotected performance touches the body.
            if not self._loco.reserve_body_action("beat_performance"):
                self._fail(generation,
                           "body is reserved by another action; no command was sent")
                return
            owns_body = True
            if self._cancelled(generation):
                return
            self._loco.preempt_for("beat_performance.start")
            if self._cancelled(generation):
                return
            prepared = self._loco.prepare_walking_mode(
                "beat_performance", cancel_event=self._stop_event)
            if prepared.get("cancelled") or self._cancelled(generation):
                return
            if prepared.get("state") == "error":
                if prepared.get("protection"):
                    with self._lock:
                        self._protection = True
                self._fail(generation, prepared.get("error", "body preparation failed"))
                return
            if self._cancelled(generation):
                return

            self._perform(generation, audio.pcm, events, config)
        except BeatSpecError as exc:
            self._fail(generation, str(exc))
        except Exception as exc:  # a worker must never die without a status
            self._fail(generation, f"performance error: {exc}")
        finally:
            # Only a run that actually took the body may emit a DEFAULT or
            # release it.  A rejected upload leaves the robot untouched.
            if owns_body:
                try:
                    self._loco.publish_velocity(0.0, 0.0, 0.0)
                except Exception as exc:  # dead SDK must not wedge cleanup
                    with self._lock:
                        self._last_error = f"final release: {exc}"
                self._loco.release_body_action("beat_performance")
            with self._lock:
                if generation == self._generation and self._state == "running":
                    self._state = "idle"
                terminal = {
                    "state": self._state,
                    "events_fired": self._events_fired,
                    "frames_published": self._frames_published,
                    "error": self._last_error,
                }
            if terminal["state"] == "error":
                status = "error"
            elif self._cancelled(generation):
                status = "cancelled"
            else:
                status = "completed"
            self._complete(active, status, terminal)

    def _perform(self, generation: int, pcm: bytes, events: list, config: dict) -> None:
        """One clock drives both outputs; commands precede each audio frame."""
        frame_ms = config["frame_ms"]
        frame_period_s = frame_ms / 1000.0
        if frame_ms * SAMPLE_RATE // 1000 <= 0:  # defensive
            raise BeatSpecError("invalid frame_ms")
        start = time.monotonic()
        event_index = 0
        pulse_remaining = 0
        pulse_vyaw = 0.0
        action_workmodes = _ACTION_WORKMODES

        for frame_number, chunk in enumerate(iter_audio_frames(pcm, frame_ms)):
            if self._cancelled(generation):
                return
            deadline = start + frame_number * frame_period_s
            # Events whose beat lands in this frame.  Scheduling on the same
            # clock as the audio keeps actions on the beat instead of drifting
            # with SDK call latency.
            due: list = []
            while event_index < len(events) and events[event_index].time_s <= frame_number * frame_period_s:
                due.append(events[event_index])
                event_index += 1

            mode = self._read_workmode()
            if mode == _PROTECTION_MODE:
                with self._lock:
                    self._protection = True
                self._fail(generation, protection_result("beat_performance", mode)["error"])
                return

            for event in due:
                if self._cancelled(generation):
                    return
                if mode is None or mode not in action_workmodes:
                    with self._lock:
                        self._last_error = (
                            f"skipped {event.kind}: workmode={mode} is not an action mode"
                        )
                    continue
                if event.kind == "semantic_action":
                    self._loco.publish_action_edge(event.command)
                    # Measure from the actual beat, not the frame deadline:
                    # the dashboard must see frame quantisation too.
                    drift_ms = (time.monotonic() - (start + event.time_s)) * 1000.0
                    with self._lock:
                        self._action_count += 1
                        self._events_fired += 1
                        self._last_action = event.action
                        self._last_action_drift_ms = drift_ms
                elif event.kind == "turn_pulse":
                    pulse_vyaw = event.vyaw
                    pulse_remaining = max(1, round(event.duration_ms / frame_ms))
                    with self._lock:
                        self._turn_count += 1
                        self._events_fired += 1

            # Edge semantics: every non-action round publishes DEFAULT, while a
            # turn pulse publishes a bounded velocity run released by DEFAULT.
            if self._cancelled(generation):
                return
            if pulse_remaining > 0:
                self._loco.publish_velocity(0.0, 0.0, pulse_vyaw)
                pulse_remaining -= 1
                if pulse_remaining == 0:
                    self._loco.publish_velocity(0.0, 0.0, 0.0)
            else:
                self._loco.publish_velocity(0.0, 0.0, 0.0)

            if self._cancelled(generation):
                return
            self._node.publish_frame(chunk, time.time_ns())
            with self._lock:
                self._frames_published += 1
                self._elapsed_s = max(0.0, time.monotonic() - start)

            remaining = deadline + frame_period_s - time.monotonic()
            if remaining > 0:
                # An interruptible sleep keeps stop from waiting out a frame.
                self._stop_event.wait(remaining)
