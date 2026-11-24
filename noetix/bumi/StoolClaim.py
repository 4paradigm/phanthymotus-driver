#!/usr/bin/env python3
"""Claim announcement card for the Bumi "grab the stool" game.

Subscribes to the chase card's event stream.  When the chase reports
``{"state": "arrived"}`` it plays the bundled Chinese prompt "抢到板凳！" on the
robot speaker.  This is the last link in the pipeline, and it only does one
thing: announce the claim.  It does not move the robot or detect bottles.

The audio is a small MP3 asset under ``resource/``; ffmpeg (already in the
image) converts it to the 16 kHz stereo PCM that MediaController expects.
"""

from __future__ import annotations

import json
import math
import shutil
import struct
import subprocess
import threading
import time
import uuid
from pathlib import Path

_SAMPLE_RATE = 16000
_CHANNELS = 2
_FRAME_MS = 100
_FRAME_SECONDS = _FRAME_MS / 1000.0
_FRAME_BYTES = _SAMPLE_RATE * _CHANNELS * 2 * _FRAME_MS // 1000
_CONFIG_INTERVAL_S = 0.5
_AUTO_COOLDOWN_S = 3.0

CLAIM_TEXT = "抢到板凳！"


class StoolClaimPlugin:
    """MCP card: ``stool_claim`` — play the claim announcement."""

    PREFIX = "stool_claim"

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 media_ctrl):
        plugin_config = plugin_config or {}
        self._namespace = namespace
        self._media_ctrl = media_ctrl
        self._event_topic = f"/{namespace}/bottle/event"
        configured = plugin_config.get("audio_path")
        self._audio_path = (Path(configured).expanduser() if configured else
                            Path(__file__).resolve().parent / "resource"
                            / "claim_stool.mp3")
        self._auto_claim = bool(plugin_config.get("auto_claim", True))
        self._active = None
        self._thread = None
        self._last = None
        self._last_event = None
        self._last_auto_at = 0.0
        self._lock = threading.RLock()
        self._config_lock = threading.Lock()
        self._last_config_change = 0.0
        self._pcm_cache = None
        self._sub = None
        self._subscription_topic = self._event_topic

        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                               DurabilityPolicy)
        from std_msgs.msg import String

        self._String = String
        self._qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=10,
                               durability=DurabilityPolicy.VOLATILE)
        self._node = Node("bumi_stool_claim")
        executor.add_node(self._node)
        self._subscribe(self._event_topic)
        print(f"[stool_claim] event←{self._event_topic} "
              f"audio={self._audio_path}", flush=True)

    # ── Card definition ──────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "stool_claim",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "抢到板凳后播报“抢到板凳！”。订阅 bottle_chase 的到达事件，"
                "收到 arrived 后自动播放内置中文语音；也可以手动 claim 播放。"
                "只播放语音，不移动机器人。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["claim", "stop", "info", "start"],
                        "description": ("claim=播放“抢到板凳！”；stop=停止播放；"
                                        "info=查看状态；start=建立事件输入。"),
                    },
                },
                "required": ["action"],
                "additionalProperties": False,
                "x-action-params": {
                    "claim": {"params": [], "description": "播放内置的抢到板凳语音。"},
                    "stop": {"params": [], "description": "停止当前播报。"},
                    "info": {"params": [], "description": "返回音频可用性和最近事件。"},
                    "start": {"params": [],
                              "description": "订阅连接的到达事件，不播放。"},
                },
            },
            "topic_in": [{"format": "data/json",
                          "message_type": "std_msgs/msg/String"}],
            "topic_out": [],
        }

    def get_tools(self) -> list:
        return [self.get_tool()]

    def start(self, args: dict | None = None):
        args = args or {}
        topic = str(args.get("input_topic") or "").strip()
        self._subscribe(topic or self._event_topic)
        return {"state": "ready", "topic_in": [{
            "topic": self._subscription_topic,
            "format": "data/json",
            "message_type": "std_msgs/msg/String",
        }]}

    def stop(self):
        with self._lock:
            record = self._active
            thread = self._thread
        if record is None:
            return {"state": "idle", "stopped": False}
        record["stop_event"].set()
        if (thread is not None and thread.is_alive()
                and thread is not threading.current_thread()):
            thread.join(timeout=3)
        if self._active is record and self._media_ctrl is not None:
            try:
                self._media_ctrl.pause_audio_playback()
            except Exception:
                pass
        return {"state": "idle", "stopped": True}

    def dispatch(self, action: str, args: dict):
        args = args or {}
        if action == "start":
            return self.start(args)
        if action == "claim":
            return self._start_claim()
        if action == "stop":
            return self.stop()
        if action == "info":
            return self._info()
        return None

    # ── Event input ──────────────────────────────────────────────────────────

    def _subscribe(self, topic: str) -> None:
        if not topic:
            return
        if self._sub is not None and topic == self._subscription_topic:
            return
        if self._sub is not None:
            try:
                self._node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        self._sub = self._node.create_subscription(
            self._String, topic, self._on_event, self._qos)
        self._subscription_topic = topic

    def _on_event(self, msg) -> None:
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if not isinstance(payload, dict) or payload.get("state") != "arrived":
            return
        with self._lock:
            self._last_event = {
                "state": "arrived",
                "distance_m": payload.get("distance_m"),
                "timestamp_ms": payload.get("timestamp_ms"),
                "received_at": time.time(),
            }
        if not self._auto_claim:
            return
        now = time.monotonic()
        with self._lock:
            if now - self._last_auto_at < _AUTO_COOLDOWN_S:
                return
            if self._active is not None:
                return
            self._last_auto_at = now
        try:
            self._start_claim()
        except Exception as exc:
            print(f"[stool_claim] auto claim failed: {exc}", flush=True)

    # ── Playback ─────────────────────────────────────────────────────────────

    def _start_claim(self) -> dict:
        if self._media_ctrl is None:
            return {"state": "unavailable",
                    "message": "MediaController 未初始化，无法播放语音"}
        if not self._audio_path.is_file():
            return {"state": "error",
                    "error": f"缺少语音文件：{self._audio_path}"}
        if shutil.which("ffmpeg") is None:
            return {"state": "error", "error": "缺少 ffmpeg，无法转换语音"}
        with self._lock:
            if self._active is not None:
                return {"state": "speaking",
                        "action_id": self._active["action_id"]}
            record = {
                "action_id": f"bumi_stool_claim_{uuid.uuid4().hex}",
                "started_at": time.time(),
                "stop_event": threading.Event(),
            }
            thread = threading.Thread(target=self._run_claim, args=(record,),
                                      daemon=True, name="bumi_stool_claim")
            self._active = record
            self._thread = thread
            thread.start()
        return {"state": "speaking", "action_id": record["action_id"],
                "text": CLAIM_TEXT}

    def _run_claim(self, record: dict) -> None:
        outcome = {"state": "error", "error": "audio did not start"}
        try:
            pcm = self._load_pcm()
            self._open_route()
            stopped = self._stream(record, pcm)
            outcome = {"state": "stopped" if stopped else "completed"}
        except Exception as exc:
            outcome = {"state": "error", "error": str(exc)}
            print(f"[stool_claim] playback failed: {exc}", flush=True)
        finally:
            try:
                self._publish_zero()
            except Exception:
                pass
            result = {**outcome, "text": CLAIM_TEXT,
                      "duration_s": round(time.time() - record["started_at"], 2)}
            with self._lock:
                if self._active is record:
                    self._active = None
                    self._thread = None
                self._last = result

    def _load_pcm(self) -> bytes:
        with self._lock:
            if self._pcm_cache is not None:
                return self._pcm_cache
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("缺少 ffmpeg")
        converted = subprocess.run(
            [ffmpeg, "-nostdin", "-y", "-loglevel", "error",
             "-i", str(self._audio_path),
             "-ar", str(_SAMPLE_RATE), "-ac", str(_CHANNELS),
             "-f", "s16le", "pipe:1"],
            capture_output=True, timeout=30, check=False)
        if converted.returncode != 0 or not converted.stdout:
            detail = converted.stderr.decode("utf-8", "replace").strip()[:300]
            raise RuntimeError(
                f"ffmpeg 转换失败（退出码 {converted.returncode}）：{detail}")
        pcm = converted.stdout
        if len(pcm) % 4:
            pcm = pcm[:-(len(pcm) % 4)]
        with self._lock:
            self._pcm_cache = pcm
        return pcm

    def _open_route(self) -> None:
        media = self._media_ctrl
        if media is None:
            raise RuntimeError("MediaController 未初始化")
        with self._config_lock:
            remaining = _CONFIG_INTERVAL_S - (
                time.monotonic() - self._last_config_change)
            if remaining > 0:
                time.sleep(remaining)
            media.set_external_custom_audio_data_to_playback_enable(True)
            self._last_config_change = time.monotonic()
            media.resume_audio_playback()

    def _stream(self, record: dict, pcm: bytes) -> bool:
        from mediacontrol_py import AudioStream

        media = self._media_ctrl
        next_at = time.monotonic()
        for offset in range(0, len(pcm), _FRAME_BYTES):
            if record["stop_event"].is_set():
                return True
            chunk = pcm[offset:offset + _FRAME_BYTES]
            if len(chunk) % 2:
                chunk = chunk[:-1]
            if not chunk:
                break
            stream = AudioStream()
            stream.channels = _CHANNELS
            stream.sample_rate = _SAMPLE_RATE
            stream.format = 2
            stream.duration_ms = _FRAME_MS
            stream.timestamp_us = time.time_ns() // 1000
            stream.audio_data = list(
                struct.unpack(f"<{len(chunk) // 2}h", chunk))
            media.publish_external_audio_playback_stream(stream)
            next_at += _FRAME_SECONDS
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        return record["stop_event"].is_set()

    def _publish_zero(self) -> None:
        # Silence is not needed: the audio agent drains what it was given.
        pass

    def _info(self) -> dict:
        with self._lock:
            active = None if self._active is None else {
                "action_id": self._active["action_id"],
                "started_at": self._active["started_at"],
            }
            last = dict(self._last) if self._last else None
            event = dict(self._last_event) if self._last_event else None
        return {
            "state": "speaking" if active else "idle",
            "text": CLAIM_TEXT,
            "audio_path": str(self._audio_path),
            "audio_available": self._audio_path.is_file(),
            "auto_claim": self._auto_claim,
            "event_topic": self._subscription_topic,
            "last_event": event,
            "active": active,
            "last": last,
        }
