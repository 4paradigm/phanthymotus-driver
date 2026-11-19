"""FallAlarmPlugin — the `fall_alarm` card for the Bumi bundle.

A processor card: it consumes the RGB camera stream on the canvas (the
``sensor_msgs/CompressedImage`` JPEG that the `camera` card publishes), runs
YOLOv11-pose plus geometric fall detection, and when a fall is confirmed it
publishes two things:

  * an alarm-speech AudioChunk on ``/{ns}/fall_alarm/alarm``
    (format ``audio/pcm-16k``) — wire this to the `speaker` card's input so the
    robot says the phrase;
  * a JSON event on ``/{ns}/fall_alarm/event`` (format ``data/json``) — wire
    this to the monitoring view or the agent to carry the fall metadata.

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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from audio_msgs.msg import AudioChunk
from sensor_msgs.msg import CompressedImage

from fall_alarm.fall_detector import FallDetector
from fall_alarm.pose_engine import PoseEngine
from fall_alarm.speech import ALARM_TEXT, CHUNK_BYTES, SAMPLE_RATE, SpeechSynthesizer

# Only the latest camera frame matters; anything older is dropped.
_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class FallAlarmPlugin:
    PREFIX = "fall_alarm"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        cfg = plugin_config or {}
        self._namespace = namespace
        self._alarm_topic = f"/{namespace}/fall_alarm/alarm"
        self._event_topic = f"/{namespace}/fall_alarm/event"

        self._node = Node("bumi_fall_alarm")
        executor.add_node(self._node)
        self._alarm_pub = self._node.create_publisher(AudioChunk, self._alarm_topic, _LATEST_QOS)
        self._event_pub = self._node.create_publisher(String, self._event_topic, _LATEST_QOS)

        self._engine = PoseEngine(
            model_path=str(cfg.get("model_path", "yolo11n-pose.pt")),
            device=cfg.get("device"),
            conf=float(cfg.get("conf", 0.5)),
            imgsz=int(cfg.get("imgsz", 640)),
        )
        self._detector = FallDetector(
            min_keypoint_conf=float(cfg.get("min_keypoint_conf", 0.5)),
            aspect_ratio_max=float(cfg.get("aspect_ratio_max", 1.15)),
            torso_angle_min_deg=float(cfg.get("torso_angle_min_deg", 45.0)),
            hip_elevation_max=float(cfg.get("hip_elevation_max", 0.25)),
            confirm_frames=int(cfg.get("confirm_frames", 3)),
            cooldown_s=float(cfg.get("cooldown_s", 8.0)),
        )

        default_wav = Path(__file__).parent / "resource" / "fall_alarm.wav"
        self._alarm_text = str(cfg.get("alarm_text", ALARM_TEXT))
        self._speech = SpeechSynthesizer(
            wav_path=cfg.get("alarm_wav_path") or str(default_wav),
            text=self._alarm_text,
        )
        self._rate_hz = max(1.0, float(cfg.get("rate_hz", 10.0)))

        # Runtime state.
        self._sub = None
        self._worker = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest_frame: bytes | None = None
        self._input_topic = ""
        self._status = "idle"
        self._frames_processed = 0
        self._alarms = 0
        self._last_event = None
        self._last_error = None
        self._log_counts: dict[str, int] = {}

    # ── Card definition ──────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "fall_alarm",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "Bumi fall alarm — consumes the RGB camera stream, runs "
                "YOLOv11-pose plus geometric fall detection, and on a "
                "confirmed fall publishes an alarm-speech AudioChunk "
                f"({self._alarm_topic}, wire to the speaker card) and a JSON "
                f"event ({self._event_topic})."
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
                        "description": "Subscribe to the connected RGB stream and begin fall detection",
                    },
                    "stop": {
                        "params": [],
                        "description": "Stop detection and release the subscription",
                    },
                    "info": {
                        "params": [],
                        "description": "Detection status, model state, alarm count and topic wiring",
                    },
                },
            },
            "topic_in": [{
                "format": "image/jpeg",
                "message_type": "sensor_msgs/msg/CompressedImage",
            }],
            "topic_out": [
                {"topic": self._alarm_topic, "format": "audio/pcm-16k"},
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
            target=self._worker_loop, daemon=True, name="fall_alarm_worker")
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
                {"topic": self._alarm_topic, "format": "audio/pcm-16k"},
                {"topic": self._event_topic, "format": "data/json"},
            ],
            "model": {
                "path": self._engine.model_path,
                "loaded": self._engine.loaded,
                "device": self._engine.device,
            },
            "alarm_text": self._alarm_text,
            "speech_source": self._speech.source,
            "frames_processed": self._frames_processed,
            "alarms": self._alarms,
            "last_event": (
                {
                    "confidence": last.confidence,
                    "bbox": list(last.bbox),
                    "aspect_ratio": last.aspect_ratio,
                    "torso_angle_deg": last.torso_angle_deg,
                    "hip_elevation_ratio": last.hip_elevation_ratio,
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
            persons = self._engine.detect(bgr)
            self._frames_processed += 1
            now_ms = int(time.time() * 1000)
            for person in persons:
                event = self._detector.update(person, now_ms)
                if event is not None:
                    self._on_fall(event)
        except Exception as exc:
            self._last_error = str(exc)
            self._throttled("frame_error", f"frame processing error: {exc}")

    def _on_fall(self, event) -> None:
        self._alarms += 1
        self._last_event = event
        print(
            f"[fall_alarm] FALL detected conf={event.confidence:.2f} "
            f"aspect_ratio={event.aspect_ratio} torso_angle={event.torso_angle_deg} "
            f"hip_elevation={event.hip_elevation_ratio}", flush=True,
        )

        payload = {
            "type": "fall",
            "text": self._alarm_text,
            "confidence": event.confidence,
            "bbox": list(event.bbox),
            "aspect_ratio": event.aspect_ratio,
            "torso_angle_deg": event.torso_angle_deg,
            "hip_elevation_ratio": event.hip_elevation_ratio,
            "timestamp_ms": event.timestamp_ms,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._event_pub.publish(msg)

        self._publish_alarm()

    def _publish_alarm(self) -> None:
        try:
            pcm = self._speech.synthesize()
        except Exception as exc:
            self._last_error = f"speech synthesis failed: {exc}"
            return

        if self._speech.source == "chime":
            self._throttled(
                "chime",
                "speech WAV asset missing; playing attention chime instead",
            )

        # Stream the PCM chunks paced to real time, on a short-lived thread so
        # the detection worker keeps running while the alarm plays.
        def _stream() -> None:
            for i in range(0, len(pcm), CHUNK_BYTES):
                if self._stop_event.is_set():
                    break
                chunk = pcm[i:i + CHUNK_BYTES]
                msg = AudioChunk()
                msg.header.stamp = self._node.get_clock().now().to_msg()
                msg.format = "audio/pcm-16k"
                msg.data = list(chunk)
                self._alarm_pub.publish(msg)
                time.sleep(len(chunk) / 2.0 / SAMPLE_RATE)

        threading.Thread(target=_stream, daemon=True, name="fall_alarm_speech").start()

    def _throttled(self, key: str, message: str) -> None:
        n = self._log_counts.get(key, 0) + 1
        self._log_counts[key] = n
        if n == 1 or n % 100 == 0:
            print(f"[fall_alarm] {message} (n={n})", flush=True)
