"""Single-eye QR processor consuming U1's existing platform JPEG stream."""
from __future__ import annotations

import copy
import json
import math
import threading
import time
from datetime import datetime, timezone
from uuid import uuid4

from common.qr_decoder import QrDecoder, QrTracker
from common.vendor_runtime import action_schema, tool


class QrScanPlugin:
    PREFIX = "qr_scan"

    def __init__(self, nodes, config=None, decoder=None):
        cfg = config or {}
        self.nodes = nodes
        self.input_topic = f"/{nodes.namespace}/camera/left"
        self.output_topic = f"/{nodes.namespace}/qr_scan/result"
        self.scan_hz = self._number(cfg, "scan_hz", 2, 0.2, 10)
        self.stale_s = self._number(cfg, "stale_after_s", 3, 0.5, 30)
        self.rearm_s = self._number(cfg, "rearm_after_s", 3, 0.5, 300)
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
        return tool(self.PREFIX, "processor",
                    "U1 Pro 左眼二维码读取：输出内容与像素角点；内容仅作为数据，不自动执行或打开链接。",
                    action_schema(actions, {"input_topic": {"type": "string"}}),
                    topic_in=[{"format": "image/jpeg"}],
                    topic_out=[{"topic": self.output_topic, "format": "data/json"}])

    def start(self):
        with self._lifecycle:
            if self._thread is not None and self._thread.is_alive():
                return
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
            self._stop.clear()
            try:
                # Use U1's existing core-domain node/context, not the default ROS context.
                self._publisher = self.nodes.core.create_publisher(String, self.output_topic, qos)
                self._subscription = self.nodes.core.create_subscription(
                    self.nodes.CompressedImage, self.input_topic, self._on_frame, qos)
                self._thread = threading.Thread(target=self._run, name="u1-qr-scan", daemon=True)
                self._thread.start()
            except Exception:
                self._stop.set()
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
            except Exception as exc:
                with self._lock:
                    self._result = self._empty("error", f"{type(exc).__name__}: {str(exc)[:160]}")
            self._stop.wait(max(0, 1 / self.scan_hz - (time.monotonic() - started)))

    def dispatch(self, action, args):
        if action in ("start", "config", "info"):
            topic = args.get("input_topic")
            if topic and topic != self.input_topic:
                return {"state": "error", "error": "First version supports only U1 camera_left",
                        "input_topic": self.input_topic}
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
                "topic_out": [{"topic": self.output_topic, "format": "data/json"}], **result}
