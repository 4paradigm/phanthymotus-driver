"""Single-eye QR processor consuming U1's existing platform JPEG stream."""
from __future__ import annotations

import copy
import json
import math
import queue
import re
import threading
import time
from datetime import datetime, timezone
from uuid import uuid4

from common.qr_decoder import QrDecoder, QrTracker
from common.vendor_runtime import action_schema, tool


class QrScanPlugin:
    PREFIX = "qr_scan"

    def __init__(self, nodes, config=None, decoder=None, tts_plugin=None):
        cfg = config or {}
        self.nodes = nodes
        self.input_topic = f"/{nodes.namespace}/camera/left"
        self.output_topic = f"/{nodes.namespace}/qr_scan/result"
        self.scan_hz = self._number(cfg, "scan_hz", 2, 0.2, 10)
        self.stale_s = self._number(cfg, "stale_after_s", 3, 0.5, 30)
        self.rearm_s = self._number(cfg, "rearm_after_s", 3, 0.5, 300)
        self.tts = tts_plugin
        self.auto_speak = cfg.get('auto_speak', False)
        if type(self.auto_speak) is not bool:
            raise ValueError('auto_speak must be boolean')
        self.speak_cooldown_s = self._number(cfg, 'speak_cooldown_s', 10, 0, 300)
        self.speak_max_chars = int(self._number(cfg, 'speak_max_chars', 80, 10, 300))
        self._speech_queue = queue.Queue(maxsize=1)
        self._speech_thread = None
        self._last_speech = None
        self._speech_result = {'state': 'idle'}
        self.decoder = decoder
        self._lock = threading.RLock()
        self._lifecycle = threading.Lock()
        self._stop = threading.Event()
        self._thread = self._subscription = self._publisher = None
        self._frame = None
        self._sequence = 0
        self._tracker = QrTracker(self.rearm_s)
        self._session = uuid4().hex
        self._result = self._empty("idle")

    @staticmethod
    def _number(cfg, key, default, low, high):
        value = float(cfg.get(key, default))
        if not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} must be between {low} and {high}")
        return value

    def _empty(self, status, message=""):
        return {"status": status, "message": message, "session_id": self._session,
                "codes": [], "new_events": [], "frame_sequence": None,
                "frame_age_s": None, "frame_received_at": None,
                "processed_at": datetime.now(timezone.utc).isoformat()}

    def get_tool(self):
        actions = {"start": ([], "Start reading the connected left-eye JPEG stream."),
                   "stop": ([], "Stop scanning."), "info": ([], "Get stream and state."),
                   "read": ([], "Read the latest result; does not trigger a scan."),
                   "config": ([], "Validate the connected input topic.")}
        definition = tool(self.PREFIX, "processor",
                    "U1 Pro 左眼二维码读取：输出内容与像素角点；内容仅作为数据，不自动执行或打开链接。",
                    action_schema(actions, {"input_topic": {"type": "string"}}),
                    topic_in=[{"format": "image/jpeg"}],
                    topic_out=[{"topic": self.output_topic, "format": "data/json"}])
        definition['configSchema'] = {'type': 'object', 'properties': {
            'auto_speak': {'type': 'boolean', 'default': True, 'description': '扫码新事件直接调用 U1 原生 TTS'},
            'speak_cooldown_s': {'type': 'number', 'minimum': 0, 'maximum': 300, 'default': 10},
            'speak_max_chars': {'type': 'integer', 'minimum': 10, 'maximum': 300, 'default': 80},
        }}
        return definition

    def start(self):
        with self._lifecycle:
            if self._thread is not None and self._thread.is_alive():
                return
            if self.auto_speak and self.tts is None:
                raise RuntimeError('U1 native TTS is not configured')
            if self.decoder is None:
                self.decoder = QrDecoder()
            from std_msgs.msg import String
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            self._message_type = String
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            with self._lock:
                self._frame = None
                self._sequence = 0
                self._tracker = QrTracker(self.rearm_s)
                self._session = uuid4().hex
                self._result = self._empty("waiting_for_frame")
            self._speech_queue = queue.Queue(maxsize=1)
            self._last_speech = None
            self._speech_result = {'state': 'idle'}
            self._stop.clear()
            try:
                # Use U1's existing core-domain node/context, not the default ROS context.
                self._publisher = self.nodes.core.create_publisher(String, self.output_topic, qos)
                self._subscription = self.nodes.core.create_subscription(
                    self.nodes.CompressedImage, self.input_topic, self._on_frame, qos)
                if self.tts is not None:
                    self._speech_thread = threading.Thread(target=self._speak_worker, name='u1-qr-speech', daemon=True)
                    self._speech_thread.start()
                self._thread = threading.Thread(target=self._run, name="u1-qr-scan", daemon=True)
                self._thread.start()
            except Exception:
                self._stop.set()
                if self._speech_thread is not None:
                    self._speech_thread.join(timeout=11)
                self._destroy_io()
                raise

    def _destroy_io(self):
        if self._subscription is not None:
            self.nodes.core.destroy_subscription(self._subscription)
            self._subscription = None
        if self._publisher is not None:
            self.nodes.core.destroy_publisher(self._publisher)
            self._publisher = None

    def stop(self):
        with self._lifecycle:
            self._stop.set()
            if self._thread is not None:
                self._thread.join(timeout=5)
                if self._thread.is_alive():
                    raise RuntimeError("QR worker has not stopped; retry stop before restarting")
            self._thread = None
            if self._speech_thread is not None:
                self._speech_thread.join(timeout=11)
                if self._speech_thread.is_alive():
                    raise RuntimeError('QR speech worker has not stopped; retry stop')
            self._speech_thread = None
            self._speech_queue = queue.Queue(maxsize=1)
            self._speech_result = {'state': 'idle'}
            self._destroy_io()
            with self._lock:
                self._frame = None
                self._result = self._empty("idle")

    def _on_frame(self, message):
        if self._stop.is_set():
            return
        if str(message.format).lower() not in ("jpeg", "jpg", "image/jpeg"):
            self._reject_frame("Expected JPEG CompressedImage")
            return
        if not 0 < len(message.data) <= 5_000_000:
            self._reject_frame("Empty or oversized JPEG")
            return
        frame = {"data": bytes(message.data), "received_monotonic": time.monotonic(),
                 "received_at": datetime.now(timezone.utc).isoformat()}
        with self._lock:
            self._sequence += 1
            frame["frame_sequence"] = self._sequence
            self._frame = frame  # Keep only the newest image; never queue a backlog.

    def _reject_frame(self, message):
        with self._lock:
            self._sequence += 1
            self._frame = {"error": message, "frame_sequence": self._sequence,
                           "received_monotonic": time.monotonic(),
                           "received_at": datetime.now(timezone.utc).isoformat()}

    def process_frame(self, frame):
        age = max(0, time.monotonic() - frame["received_monotonic"])
        result = self._empty("no_qr")
        result.update(frame_sequence=frame["frame_sequence"], frame_age_s=age,
                      frame_received_at=frame["received_at"])
        if age > self.stale_s:
            result["status"] = "stale"
            return result
        try:
            if "error" in frame:
                raise ValueError(frame["error"])
            codes = self.decoder.decode(frame["data"])
            now = time.monotonic()
            result["frame_age_s"] = max(0, now - frame["received_monotonic"])
            if result["frame_age_s"] > self.stale_s:
                result["status"] = "stale"
                return result
            result.update(status="detected" if codes else "no_qr", codes=codes,
                          new_events=self._tracker.update(codes, now))
        except Exception as exc:
            result.update(status="decode_error", message=f"{type(exc).__name__}: {str(exc)[:160]}")
        return result

    def _run(self):
        last_sequence = None
        while not self._stop.is_set():
            started = time.monotonic()
            with self._lock:
                frame = self._frame
            try:
                if frame is None:
                    result = self._empty("waiting_for_frame")
                elif frame["frame_sequence"] != last_sequence:
                    result = self.process_frame(frame)
                    last_sequence = frame["frame_sequence"]
                elif started - frame["received_monotonic"] > self.stale_s:
                    result = self._empty("stale")
                    result.update(frame_sequence=frame["frame_sequence"],
                                  frame_received_at=frame["received_at"],
                                  frame_age_s=started - frame["received_monotonic"])
                else:
                    result = None
                if result is not None and not self._stop.is_set():
                    message = self._message_type()
                    message.data = json.dumps(result, ensure_ascii=False, allow_nan=False)
                    with self._lock:
                        self._result = result
                    self._publisher.publish(message)
                    self._queue_speech(result)
            except Exception as exc:
                with self._lock:
                    self._result = self._empty("error", f"{type(exc).__name__}: {str(exc)[:160]}")
            self._stop.wait(max(0, 1 / self.scan_hz - (time.monotonic() - started)))

    def _queue_speech(self, result):
        if not self.auto_speak or result.get('status') != 'detected' or not result.get('new_events'):
            return
        now = time.monotonic()
        if self._last_speech is not None and now - self._last_speech < self.speak_cooldown_s:
            return
        event = result['new_events'][0]
        content = event['text']
        normalized = content.strip()
        # URI schemes include wxp:, weixin:, app links and mailto:, not only HTTP.
        # Also recognize common bare web addresses. Preserve the original in codes.
        is_link = bool(re.match(r'^[a-z][a-z0-9+.-]*:', normalized, re.IGNORECASE)
                       or re.match(r'^(?:www\.|(?:[a-z0-9-]+\.)+[a-z]{2,}(?:[/:?#]|$))',
                                   normalized, re.IGNORECASE))
        if is_link:
            text = '扫码成功，识别到一个链接。'
        else:
            text = '扫码成功，内容是：' + content[:self.speak_max_chars]
            if len(content) > self.speak_max_chars:
                text += '。内容较长，已省略后续部分。'
        job = {'text': text, 'action_id': 'qr-' + self._session + '-' + str(event['sequence']), 'created': now}
        try:
            self._speech_queue.put_nowait(job)
            self._last_speech = now
        except queue.Full:
            pass  # Drop rather than accumulate obsolete speech.

    def _speak_worker(self):
        while not self._stop.is_set():
            try:
                job = self._speech_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if self._stop.is_set() or not self.auto_speak or time.monotonic()-job['created'] > self.stale_s:
                continue
            try:
                if self.tts.dispatch('info', {}).get('active'):
                    response = {'state': 'skipped', 'message': 'Native TTS is busy'}
                elif self._stop.is_set() or not self.auto_speak:
                    continue
                else:
                    response = self.tts.dispatch('speak', {'text': job['text'], 'action_id': job['action_id']})
                with self._lock:
                    self._speech_result = response
            except Exception as exc:
                with self._lock:
                    self._speech_result = {'state': 'error', 'message': str(exc)[:160]}

    def dispatch(self, action, args):
        if action in ("start", "config", "info"):
            topic = args.get("input_topic")
            if topic and topic != self.input_topic:
                return {"state": "error", "error": "First version supports only U1 camera_left",
                        "input_topic": self.input_topic}
            if action == 'config':
                cfg = args.get('config', {})
                if not isinstance(cfg, dict):
                    return {'state': 'error', 'error': 'config must be an object'}
                try:
                    enabled = cfg.get('auto_speak', self.auto_speak)
                    if type(enabled) is not bool:
                        raise ValueError('auto_speak must be boolean')
                    cooldown = self._number(cfg, 'speak_cooldown_s', self.speak_cooldown_s, 0, 300)
                    limit = int(self._number(cfg, 'speak_max_chars', self.speak_max_chars, 10, 300))
                except (TypeError, ValueError) as exc:
                    return {'state': 'error', 'error': str(exc)}
                self.auto_speak, self.speak_cooldown_s, self.speak_max_chars = enabled, cooldown, limit
            if action == "start":
                self.start()
        elif action == "stop":
            self.stop()
        elif action not in ("read", "qr_scan"):
            return {"state": "error", "error": f"Unknown action: {action}"}
        with self._lock:
            result = copy.deepcopy(self._result)
            frame = self._frame
            if frame is not None and result["frame_sequence"] == frame["frame_sequence"]:
                result["frame_age_s"] = max(0, time.monotonic() - frame["received_monotonic"])
                if result["frame_age_s"] > self.stale_s:
                    result.update(status="stale", codes=[], new_events=[])
        running = self._thread is not None and self._thread.is_alive() and not self._stop.is_set()
        return {"state": "running" if running else "idle", "input_topic": self.input_topic,
                "auto_speak": self.auto_speak, "speech": copy.deepcopy(self._speech_result),
                "topic_out": [{"topic": self.output_topic, "format": "data/json"}], **result}
