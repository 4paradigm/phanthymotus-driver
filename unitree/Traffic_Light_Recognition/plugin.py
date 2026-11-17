"""TrafficLightPlugin — the `traffic_light` card for the As2W bundle.

A processor card: it consumes the RGB camera stream on the canvas (the
``sensor_msgs/CompressedImage`` JPEG that the `camera` card publishes), runs
YOLOv8n traffic-light detection plus HSV color classification, and when the
light state changes it publishes two things:

  * a speech AudioChunk on ``/{ns}/traffic_light/audio``
    (format ``audio/pcm-16k``) — wire this to the `speaker` card's input so the
    robot says "红灯，请等待" / "绿灯，请通行" etc.;
  * a JSON event on ``/{ns}/traffic_light/event`` (format ``data/json``) — wire
    this to the monitoring view or the agent to carry the light metadata.

The heavy inference runs on a dedicated worker thread; the ROS subscription
callback only stores the latest JPEG frame, so the executor thread never waits
on YOLO. The model is loaded lazily by that worker so the first canvas `start`
returns immediately.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from audio_msgs.msg import AudioChunk
from sensor_msgs.msg import CompressedImage

from Traffic_Light_Recognition.detection_engine import DetectionEngine
from Traffic_Light_Recognition.light_classifier import (
    LightStateTracker,
    classify_from_scores,
    compute_color_scores,
    UNKNOWN,
)
from Traffic_Light_Recognition.speech import (
    CHUNK_BYTES,
    DEFAULT_TEXTS,
    SAMPLE_RATE,
    SpeechSynthesizer,
)

# Only the latest camera frame matters; anything older is dropped. The camera
# card publishes BEST_EFFORT, so the subscriber must match.
_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

_COLORS = ("red", "yellow", "green")


class TrafficLightPlugin:
    PREFIX = "traffic_light"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        cfg = plugin_config or {}
        self._namespace = namespace
        self._audio_topic = f"/{namespace}/traffic_light/audio"
        self._event_topic = f"/{namespace}/traffic_light/event"

        self._node = Node("as2w_traffic_light")
        executor.add_node(self._node)
        self._audio_pub = self._node.create_publisher(
            AudioChunk, self._audio_topic, _LATEST_QOS)
        self._event_pub = self._node.create_publisher(
            String, self._event_topic, _LATEST_QOS)

        self._engine = DetectionEngine(
            model_path=str(cfg.get("model_path", "yolov8n.pt")),
            device=cfg.get("device"),
            conf=float(cfg.get("conf", 0.35)),
            imgsz=int(cfg.get("imgsz", 640)),
        )
        self._saturation_min = float(cfg.get("saturation_min", 90))
        self._value_min = float(cfg.get("value_min", 100))
        self._min_lit_fraction = float(cfg.get("min_lit_fraction", 0.005))
        self._dominance_ratio = float(cfg.get("dominance_ratio", 1.5))
        self._tracker = LightStateTracker(
            confirm_frames=int(cfg.get("confirm_frames", 2)),
            cooldown_s=float(cfg.get("cooldown_s", 3.0)),
        )

        self._texts = dict(cfg.get("texts") or DEFAULT_TEXTS)
        self._speech = SpeechSynthesizer(
            wav_paths=self._resolve_wav_paths(cfg), texts=self._texts)
        self._rate_hz = max(1.0, float(cfg.get("rate_hz", 5.0)))

        # Runtime state.
        self._sub = None
        self._worker = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest_frame: bytes | None = None
        self._input_topic = ""
        self._status = "idle"
        self._frames_processed = 0
        self._announcements = 0
        self._last_event = None
        self._last_error = None
        self._log_counts: dict[str, int] = {}

    def _resolve_wav_paths(self, cfg) -> dict:
        """Prefer explicit config paths, then auto-discovered resource WAVs."""
        resource_dir = Path(__file__).parent / "resource"
        wav_paths = {}
        for color in _COLORS:
            explicit = cfg.get(f"{color}_wav_path")
            if explicit:
                wav_paths[color] = str(explicit)
            else:
                candidate = resource_dir / f"traffic_light_{color}.wav"
                if candidate.exists():
                    wav_paths[color] = str(candidate)
        return wav_paths

    # ── Card definition ──────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "traffic_light",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "As2W traffic-light recognition — consumes the RGB camera stream, "
                "runs YOLOv8n traffic-light detection plus HSV color "
                "classification, and on a red/yellow/green state change publishes "
                f"a speech AudioChunk ({self._audio_topic}, wire to the speaker "
                f"card) and a JSON event ({self._event_topic})."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info"]},
                    "input_topic": {
                        "type": "string",
                        "description": (
                            "ROS2 topic publishing sensor_msgs/CompressedImage "
                            "(JPEG) — provided by the canvas connection to the "
                            "camera card"
                        ),
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {
                        "params": ["input_topic"],
                        "description": "Subscribe to the connected RGB stream and begin traffic-light recognition",
                    },
                    "stop": {
                        "params": [],
                        "description": "Stop recognition and release the subscription",
                    },
                    "info": {
                        "params": [],
                        "description": "Recognition status, model state, current color and topic wiring",
                    },
                },
            },
            "topic_in": [{
                "format": "image/jpeg",
                "message_type": "sensor_msgs/msg/CompressedImage",
            }],
            "topic_out": [
                {"topic": self._audio_topic, "format": "audio/pcm-16k"},
                {"topic": self._event_topic, "format": "data/json"},
            ],
        }

    # ── Bundle lifecycle ─────────────────────────────────────────────────────

    def start(self) -> None:
        # The bundle starts every card at boot, but this card cannot subscribe
        # until the canvas resolves the upstream camera topic and sends `start`
        # with `input_topic`. Real start happens in dispatch("start").
        pass

    def stop(self) -> None:
        self._teardown()

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop("_tool_name", None)
        if action == "start":
            return self._start_detection(args)
        if action == "stop":
            return self._stop_detection()
        if action == "info":
            return self._info()
        return None

    # ── Card actions ─────────────────────────────────────────────────────────

    def _start_detection(self, args: dict) -> dict:
        input_topic = str(args.get("input_topic") or "").strip()
        if not input_topic:
            return {"error": "input_topic is required"}

        # Idempotent (re)start: tear down any previous run first.
        self._teardown()
        self._input_topic = input_topic
        self._last_error = None
        self._stop_event = threading.Event()

        def _on_frame(msg: CompressedImage):
            # Keep only the newest frame; the worker decodes and infers.
            with self._frame_lock:
                self._latest_frame = bytes(msg.data)

        self._sub = self._node.create_subscription(
            CompressedImage, input_topic, _on_frame, _LATEST_QOS)

        self._worker = threading.Thread(
            target=self._worker_loop, daemon=True, name="traffic_light_worker")
        self._worker.start()

        self._status = "running"
        return {
            "state": "running",
            "input_topic": input_topic,
            "topic_in": [{
                "topic": input_topic,
                "format": "image/jpeg",
                "message_type": "sensor_msgs/msg/CompressedImage",
            }],
        }

    def _stop_detection(self) -> dict:
        self._teardown()
        return {"state": "idle"}

    def _teardown(self) -> None:
        self._stop_event.set()
        worker = self._worker
        self._worker = None
        if worker is not None:
            worker.join(timeout=3.0)
        if self._sub is not None:
            try:
                self._node.destroy_subscription(self._sub)
            except Exception:
                pass
            self._sub = None
        self._status = "idle"

    def _info(self) -> dict:
        topic_in = [{
            "format": "image/jpeg",
            "message_type": "sensor_msgs/msg/CompressedImage",
        }]
        if self._input_topic:
            topic_in[0]["topic"] = self._input_topic

        last = self._last_event
        return {
            "state": self._status,
            "topic_in": topic_in,
            "topic_out": [
                {"topic": self._audio_topic, "format": "audio/pcm-16k"},
                {"topic": self._event_topic, "format": "data/json"},
            ],
            "model": {
                "path": self._engine.model_path,
                "loaded": self._engine.loaded,
                "device": self._engine.device,
            },
            "current_color": self._tracker.announced_color,
            "texts": self._texts,
            "frames_processed": self._frames_processed,
            "announcements": self._announcements,
            "last_event": (
                {
                    "color": last.color,
                    "confidence": last.confidence,
                    "fraction": last.fraction,
                    "bbox": list(last.bbox),
                    "timestamp_ms": last.timestamp_ms,
                } if last is not None else None
            ),
            "last_error": self._last_error,
        }

    # ── Detection worker ─────────────────────────────────────────────────────

    def _worker_loop(self) -> None:
        try:
            self._engine.load()
        except Exception as exc:
            self._last_error = f"model load failed: {exc}"
            self._status = "error"
            return

        try:
            import cv2
            import numpy as np
        except Exception as exc:
            self._last_error = f"cv2/numpy unavailable: {exc}"
            self._status = "error"
            return

        interval = 1.0 / self._rate_hz
        while not self._stop_event.is_set():
            with self._frame_lock:
                frame = self._latest_frame
                self._latest_frame = None
            if frame is not None:
                self._process_frame(frame, cv2, np)
            self._stop_event.wait(interval)

    def _process_frame(self, jpeg_bytes: bytes, cv2, np) -> None:
        try:
            arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if bgr is None:
                return
            boxes = self._engine.detect(bgr)
            self._frames_processed += 1
            now_ms = int(time.time() * 1000)

            if not boxes:
                self._tracker.update(UNKNOWN, now_ms=now_ms)
                return

            # Use the most confident traffic light in the frame.
            best = max(boxes, key=lambda b: b["confidence"])
            x1, y1, x2, y2 = (int(round(v)) for v in best["bbox"])
            h, w = bgr.shape[:2]
            x1, x2 = max(0, min(x1, w)), max(0, min(x2, w))
            y1, y2 = max(0, min(y1, h)), max(0, min(y2, h))
            if x2 <= x1 or y2 <= y1:
                self._tracker.update(UNKNOWN, now_ms=now_ms)
                return

            roi = bgr[y1:y2, x1:x2]
            scores = compute_color_scores(
                roi, saturation_min=self._saturation_min,
                value_min=self._value_min)
            color = classify_from_scores(
                scores, min_lit_fraction=self._min_lit_fraction,
                dominance_ratio=self._dominance_ratio)
            event = self._tracker.update(
                color,
                confidence=best["confidence"],
                fraction=scores.get(color, 0.0),
                bbox=best["bbox"],
                now_ms=now_ms,
            )
            if event is not None:
                self._on_light(event)
        except Exception as exc:
            self._last_error = str(exc)
            self._throttled("frame_error", f"frame processing error: {exc}")

    def _on_light(self, event) -> None:
        self._announcements += 1
        self._last_event = event
        print(
            f"[traffic_light] {event.color} conf={event.confidence:.2f} "
            f"fraction={event.fraction:.3f} bbox={event.bbox}", flush=True,
        )

        payload = {
            "type": "traffic_light",
            "color": event.color,
            "text": self._speech.phrase(event.color),
            "confidence": event.confidence,
            "fraction": event.fraction,
            "bbox": list(event.bbox),
            "timestamp_ms": event.timestamp_ms,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._event_pub.publish(msg)

        self._publish_speech(event.color)

    def _publish_speech(self, color: str) -> None:
        try:
            pcm = self._speech.synthesize(color)
        except Exception as exc:
            self._last_error = f"speech synthesis failed: {exc}"
            return

        if self._speech.source(color) == "chime":
            self._throttled(
                "chime",
                f"speech WAV asset for {color} missing; playing chime instead",
            )

        # Stream the PCM chunks paced to real time, on a short-lived thread so
        # the detection worker keeps running while the announcement plays.
        def _stream() -> None:
            for i in range(0, len(pcm), CHUNK_BYTES):
                if self._stop_event.is_set():
                    break
                chunk = pcm[i:i + CHUNK_BYTES]
                msg = AudioChunk()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.format = "audio/pcm-16k"
                msg.data = list(chunk)
                self._audio_pub.publish(msg)
                time.sleep(len(chunk) / 2.0 / SAMPLE_RATE)

        threading.Thread(target=_stream, daemon=True, name="traffic_light_speech").start()

    def _throttled(self, key: str, message: str) -> None:
        n = self._log_counts.get(key, 0) + 1
        self._log_counts[key] = n
        if n == 1 or n % 100 == 0:
            print(f"[traffic_light] {message} (n={n})", flush=True)
