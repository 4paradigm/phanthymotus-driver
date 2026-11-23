"""UpperBodyTrackPlugin — the `upper_body_track` card for the Bumi bundle.

A processor card that answers one question: *is the robot seeing a person's
upper body, and where would that person's arms be on this robot?*

It consumes the RGB stream and, when available, the registered depth stream
that the `camera` / `depth` cards publish, runs YOLO11-pose on a worker thread,
and publishes two things:

  * `sensor/pose2d` on ``/{ns}/upper_body_track/pose2d`` — the COCO-17
    keypoints the dashboard draws over the frame;
  * `data/json` on ``/{ns}/upper_body_track/angles`` — the six retargeted arm
    joint angles plus the flags that say whether they are trustworthy.

**It commands nothing.** There is not one call to `publish_cmd` or to the
SDK's joint-command entry point anywhere in this file — the only mention is the
one in this paragraph. Driving the arms from these angles is a separate
card, because on this robot that card must take over all 21 motors (the SDK
rejects a command list of any other length) and therefore has to hold its own
balance — a decision that does not belong inside a vision card.

Inference runs on a dedicated worker thread and the subscriptions only keep the
newest frame, so a slow first model load cannot stall the executor that every
other card shares.
"""

from __future__ import annotations

import json
import threading
import time
import zlib

import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import upper_body_track_spec as spec

# Only the newest frame of each stream is useful; a queue would only add age.
_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

# A depth sample older than this is a different scene, not the same one.
# Both streams run at 30 fps, so 0.4 s is roughly twelve frames of drift.
_DEPTH_FRESH_S = 0.4
# Width of the depth patch averaged per keypoint. 5x5 at 640x480 is ~1.5 deg of
# the field of view — small enough to stay on one limb, wide enough to survive
# single-pixel invalid measurements.
_DEPTH_PATCH = 5
_DEFAULT_FOCAL_PX = 460.0

# Named for the person, not for the robot: the mapping mirrors them.
_PERSON_LEFT = (spec.L_SHOULDER, spec.L_ELBOW, spec.L_WRIST)
_PERSON_RIGHT = (spec.R_SHOULDER, spec.R_ELBOW, spec.R_WRIST)


class _DepthCache:
    """Newest decoded-not-yet depth frame, with its receive time."""

    def __init__(self):
        self._lock = threading.Lock()
        self._data = None
        self._received = 0.0
        self._width = 0
        self._height = 0

    def put(self, data, width, height):
        with self._lock:
            self._data = data
            self._width = width
            self._height = height
            self._received = time.monotonic()

    def age_s(self):
        with self._lock:
            if self._data is None:
                return None
            return time.monotonic() - self._received

    def decode(self):
        """→ (uint16 array, width, height) or `(None, 0, 0)` when unusable."""
        with self._lock:
            data = self._data
            width, height = self._width, self._height
        if data is None:
            return None, 0, 0
        try:
            raw = zlib.decompress(data)
            frame = np.frombuffer(raw, dtype=np.uint16)
        except Exception:
            return None, 0, 0
        if width <= 0 or height <= 0:
            return None, 0, 0
        if frame.size != width * height:
            return None, 0, 0
        return frame.reshape((height, width)), width, height


class _PoseEngine:
    """YOLO11-pose wrapper.

    `ultralytics` and its torch dependency are imported on first use so this
    module can be byte-compiled on a machine that has neither — the same lazy
    discipline the rest of the bundle uses for ROS. Official weights, no
    training: the file is downloaded once and cached, or `model_path` points at
    a pre-downloaded copy on an offline robot.
    """

    def __init__(self, model_path="yolo11n-pose.pt", device="cpu", conf=0.5, imgsz=640):
        self.model_path = model_path
        self.device = device
        self.conf = conf
        self.imgsz = imgsz
        self._model = None
        self._error = None

    @property
    def loaded(self):
        return self._model is not None

    @property
    def error(self):
        return self._error

    def load(self):
        if self._model is not None:
            return
        from ultralytics import YOLO
        self._model = YOLO(self.model_path)
        self._error = None

    def detect(self, bgr):
        if self._model is None:
            self.load()
        results = self._model.predict(
            bgr, imgsz=self.imgsz, conf=self.conf, device=self.device,
            verbose=False, stream=False,
        )
        persons = []
        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None or len(boxes) == 0:
                continue
            xyxy = boxes.xyxy.cpu().tolist()
            scores = boxes.conf.cpu().tolist()
            keypoints = getattr(result, "keypoints", None)
            xy = None
            kp_conf = None
            if keypoints is not None:
                if getattr(keypoints, "xy", None) is not None:
                    xy = keypoints.xy.cpu().tolist()
                if getattr(keypoints, "conf", None) is not None:
                    kp_conf = keypoints.conf.cpu().tolist()
            for index in range(len(xyxy)):
                points = []
                for point_index in range(len(spec.COCO17_NAMES)):
                    if xy is not None:
                        px, py = xy[index][point_index][0], xy[index][point_index][1]
                    else:
                        px, py = 0.0, 0.0
                    confidence = 0.0
                    if kp_conf is not None:
                        confidence = kp_conf[index][point_index]
                        if isinstance(confidence, (list, tuple)):
                            confidence = confidence[0]
                    points.append([float(px), float(py), float(confidence)])
                persons.append({
                    "keypoints": points,
                    "bbox": [float(v) for v in xyxy[index]],
                    "confidence": float(scores[index]),
                })
        return persons


class UpperBodyTrackPlugin:
    PREFIX = "upper_body_track"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        cfg = plugin_config or {}
        self._namespace = namespace
        self._pose2d_topic = f"/{namespace}/upper_body_track/pose2d"
        self._angles_topic = f"/{namespace}/upper_body_track/angles"

        self._node = Node("bumi_upper_body_track")
        executor.add_node(self._node)
        self._pose2d_pub = self._node.create_publisher(
            String, self._pose2d_topic, _LATEST_QOS)
        self._angles_pub = self._node.create_publisher(
            String, self._angles_topic, _LATEST_QOS)

        self._engine = _PoseEngine(
            model_path=str(cfg.get("model_path", "yolo11n-pose.pt")),
            device=str(cfg.get("device", "cpu")),
            conf=float(cfg.get("conf", 0.5)),
            imgsz=int(cfg.get("imgsz", 640)),
        )
        self._min_kp_conf = float(cfg.get("min_keypoint_conf", 0.4))
        self._depth_cache = _DepthCache()
        self._bank = spec.AngleBank(
            min_cutoff=float(cfg.get("filter_min_cutoff", 1.0)),
            beta=float(cfg.get("filter_beta", 0.02)),
        )

        # Tunables the operator may override per `start`, so the sign of an axis
        # and the smoothing can be corrected on a robot without rebuilding the
        # image — config.yaml is baked into the container.
        self._signs = {
            "pitch_sign": float(cfg.get("pitch_sign", 1.0)),
            "roll_sign": float(cfg.get("roll_sign", 1.0)),
            "elbow_sign": float(cfg.get("elbow_sign", 1.0)),
        }
        self._focal_setting = cfg.get("focal_px", _DEFAULT_FOCAL_PX)
        self._rate_hz = max(1.0, float(cfg.get("rate_hz", 10.0)))

        self._color_sub = None
        self._depth_sub = None
        self._worker = None
        self._generation = 0
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest_jpeg = None
        self._latest_jpeg_at = 0.0
        self._input_topic = ""
        self._depth_topic = ""

        self._validate()

        self._status = "idle"
        self._frames = 0
        self._last_error = None
        # One corrupt stream is one outage, not ten warnings a second. The
        # first failure and every hundredth keep visibility; the first good
        # frame announces recovery immediately (same pattern as the odom card).
        self._frame_failures = 0
        self._last_payload = None
        self._last_inference_ms = 0.0

    def _validate(self) -> None:
        """Range checks belong here, not as silent clamps on the way out.

        A `rate_hz` of 200 is not "fast", it is a frame budget the card cannot
        meet; a `conf` of 0 turns every shadow into a person. Both would present
        as plausible numbers on the panel, which is why they raise instead.
        """
        checks = (
            ("rate_hz", self._rate_hz, 1.0, 30.0),
            ("min_keypoint_conf", self._min_kp_conf, 0.05, 0.99),
            ("conf", self._engine.conf, 0.05, 0.95),
            ("imgsz", self._engine.imgsz, 320, 1280),
        )
        for name, value, low, high in checks:
            if not low <= float(value) <= high:
                raise ValueError(f"{name} must be in [{low}, {high}], got {value}")
        for name, value in self._signs.items():
            if value not in (1.0, -1.0):
                raise ValueError(f"{name} must be 1.0 or -1.0, got {value}")
        if str(self._focal_setting).strip().lower() == "auto":
            return
        try:
            focal = float(self._focal_setting)
        except (TypeError, ValueError):
            raise ValueError("focal_px must be a number or 'auto'")
        if not 200.0 <= focal <= 1200.0:
            raise ValueError(f"focal_px must be in [200, 1200] or 'auto', got {focal}")

    # ── Card definition ──────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "upper_body_track",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "Upper-body tracking for Bumi: consumes the RGB camera stream "
                "(and the registered depth stream when wired), runs YOLO11-pose, "
                f"publishes 2D keypoints to {self._pose2d_topic} and the six "
                f"retargeted arm joint angles to {self._angles_topic}. "
                "Read-only: it never commands a joint, so the arms do not move. "
                "Angles assume the person faces the robot and one person is in "
                "frame; humeral rotation (palm facing) is not observable and is "
                "reported as zero. `tracked: false` means nobody was seen well "
                "enough to measure, not that the robot is idle."
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                        "description": "Card lifecycle or live status",
                    },
                    "input_topic": {
                        "type": "string",
                        "description": (
                            "ROS2 topic publishing sensor_msgs/CompressedImage "
                            "JPEG — provided by the canvas connection to the "
                            "camera card"),
                    },
                    "depth_topic": {
                        "type": "string",
                        "description": (
                            "Optional ROS2 topic publishing zlib-compressed Z16 "
                            "depth registered to the RGB plane. Without it the "
                            "card falls back to a flat-plane assumption and says "
                            "so in `degraded`"),
                    },
                    "focal_px": {
                        "type": "string",
                        "description": (
                            "Camera focal length in pixels, or `auto` to derive "
                            "it from shoulder width at the measured depth"),
                    },
                    "pitch_sign": {
                        "type": "number",
                        "description": "±1, flips shoulder pitch for this URDF",
                    },
                    "roll_sign": {
                        "type": "number",
                        "description": "±1, flips shoulder roll for this URDF",
                    },
                    "elbow_sign": {
                        "type": "number",
                        "description": "±1, flips elbow flexion for this URDF",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {
                        "params": ["input_topic", "depth_topic", "focal_px",
                                   "pitch_sign", "roll_sign", "elbow_sign"],
                        "description": "Subscribe to the camera streams and begin tracking",
                    },
                    "stop": {"params": [], "description": "Stop tracking and release subscriptions"},
                    "info": {"params": [], "description": "Tracking status and last measured angles"},
                },
            },
            "topic_in": [
                {"format": "image/jpeg", "message_type": "sensor_msgs/msg/CompressedImage"},
                {"format": "image/depth-zlib", "message_type": "sensor_msgs/msg/CompressedImage"},
            ],
            "topic_out": [
                {"topic": self._pose2d_topic, "format": "sensor/pose2d"},
                {"topic": self._angles_topic, "format": "data/json"},
            ],
        }

    # ── Bundle lifecycle ─────────────────────────────────────────────────────

    def start(self) -> None:
        # The bundle starts every card at boot, but this one cannot subscribe
        # until the canvas resolves which camera topic to read. Real start
        # happens in dispatch("start"), exactly like the other processor cards.
        pass

    def stop(self) -> None:
        self._teardown()

    def dispatch(self, action: str, args: dict) -> dict | None:
        args.pop("_tool_name", None)
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "info":
            return self._info()
        return None

    # ── Actions ──────────────────────────────────────────────────────────────

    def _start(self, args: dict) -> dict:
        input_topic = str(args.get("input_topic") or "").strip()
        if not input_topic:
            return {"error": "input_topic is required — wire the camera card's RGB output"}

        error = self._apply_overrides(args)
        if error is not None:
            return {"error": error}
        self._teardown()
        self._input_topic = input_topic
        self._depth_topic = str(args.get("depth_topic") or "").strip()
        self._last_error = None
        self._frame_failures = 0
        self._stop_event = threading.Event()

        def _on_color(msg: CompressedImage):
            with self._frame_lock:
                self._latest_jpeg = bytes(msg.data)
                self._latest_jpeg_at = time.monotonic()

        def _on_depth(msg: CompressedImage):
            # Width/height come from the vendor contract (640x480), not from the
            # message: CompressedImage carries no geometry and the payload is a
            # zlib blob of raw Z16.
            self._depth_cache.put(bytes(msg.data), 640, 480)

        self._color_sub = self._node.create_subscription(
            CompressedImage, input_topic, _on_color, _LATEST_QOS)
        if self._depth_topic:
            self._depth_sub = self._node.create_subscription(
                CompressedImage, self._depth_topic, _on_depth, _LATEST_QOS)

        self._generation += 1
        self._worker = threading.Thread(
            target=self._worker_loop, args=(self._generation,),
            daemon=True, name="upper_body_track")
        self._worker.start()

        self._status = "running"
        result = {
            "state": "running",
            "input_topic": input_topic,
            "topic_in": [{
                "topic": input_topic,
                "format": "image/jpeg",
                "message_type": "sensor_msgs/msg/CompressedImage",
            }],
        }
        if self._depth_topic:
            result["depth_topic"] = self._depth_topic
            result["topic_in"].append({
                "topic": self._depth_topic,
                "format": "image/depth-zlib",
                "message_type": "sensor_msgs/msg/CompressedImage",
            })
        return result

    def _stop(self) -> dict:
        self._teardown()
        return {"state": "idle"}

    def _apply_overrides(self, args: dict):
        """Apply per-`start` overrides, refusing values that cannot be meant.

        Returns an error message instead of raising: a bad canvas parameter is a
        caller mistake to report, not a reason to take the card's executor down.
        """
        for key in ("pitch_sign", "roll_sign", "elbow_sign"):
            if args.get(key) is None:
                continue
            try:
                value = float(args[key])
            except (TypeError, ValueError):
                return f"{key} must be a number, got {args[key]!r}"
            if value not in (1.0, -1.0):
                return f"{key} must be 1.0 or -1.0, got {value}"
            self._signs[key] = value

        focal = args.get("focal_px")
        if focal is None:
            return None
        if str(focal).strip().lower() == "auto":
            self._focal_setting = "auto"
            return None
        try:
            value = float(focal)
        except (TypeError, ValueError):
            return f"focal_px must be a number or 'auto', got {focal!r}"
        if not 200.0 <= value <= 1200.0:
            return f"focal_px must be in [200, 1200] or 'auto', got {value}"
        self._focal_setting = value
        return None

    def _teardown(self) -> None:
        self._stop_event.set()
        worker = self._worker
        self._worker = None
        if worker is not None:
            worker.join(timeout=3.0)
        for attr in ("_color_sub", "_depth_sub"):
            sub = getattr(self, attr)
            if sub is not None:
                try:
                    self._node.destroy_subscription(sub)
                except Exception:
                    pass
                setattr(self, attr, None)
        self._bank.reset()
        self._status = "idle"

    def _info(self) -> dict:
        topic_in = [{
            "format": "image/jpeg",
            "message_type": "sensor_msgs/msg/CompressedImage",
        }]
        if self._input_topic:
            topic_in[0]["topic"] = self._input_topic
        if self._depth_topic:
            topic_in.append({
                "topic": self._depth_topic,
                "format": "image/depth-zlib",
                "message_type": "sensor_msgs/msg/CompressedImage",
            })
        return {
            "state": self._status,
            "topic_in": topic_in,
            "topic_out": [
                {"topic": self._pose2d_topic, "format": "sensor/pose2d"},
                {"topic": self._angles_topic, "format": "data/json"},
            ],
            "model": {
                "path": self._engine.model_path,
                "loaded": self._engine.loaded,
                "device": self._engine.device,
                "error": self._engine.error,
            },
            "focal_px": self._focal_setting,
            "signs": dict(self._signs),
            "frames_processed": self._frames,
            "last_inference_ms": round(self._last_inference_ms, 1),
            "last": self._last_payload,
            "last_error": self._last_error,
        }

    # ── Worker ───────────────────────────────────────────────────────────────

    def _worker_loop(self, generation: int) -> None:
        interval = 1.0 / self._rate_hz
        while not self._stop_event.is_set() and generation == self._generation:
            with self._frame_lock:
                jpeg = self._latest_jpeg
                received_at = self._latest_jpeg_at
            if jpeg is None:
                self._stop_event.wait(0.05)
                continue

            started = time.monotonic()
            try:
                self._process(jpeg, received_at)
            except Exception as exc:  # keep the worker alive; a bad frame is not fatal
                self._frame_failures += 1
                self._last_error = f"{type(exc).__name__}: {exc}"
                if self._frame_failures == 1 or self._frame_failures % 100 == 0:
                    self._node.get_logger().warning(
                        f"upper_body_track frame failed ({self._frame_failures}): "
                        f"{self._last_error}")
            else:
                if self._frame_failures:
                    self._node.get_logger().info(
                        f"upper_body_track frame recovered after "
                        f"{self._frame_failures} failures")
                    self._frame_failures = 0
            self._last_inference_ms = (time.monotonic() - started) * 1000.0

            elapsed = time.monotonic() - started
            if elapsed < interval and self._stop_event.wait(interval - elapsed):
                break

    def _process(self, jpeg: bytes, received_at: float) -> None:
        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("JPEG decode returned no frame")
        height, width = frame.shape[:2]

        started = time.monotonic()
        persons = self._engine.detect(frame)
        self._frames += 1

        labeled = {}
        for index, person in enumerate(persons):
            labeled[index] = spec.posture_name(
                spec.keypoints_of(person), self._min_kp_conf)

        self._pose2d_pub.publish(String(data=json.dumps(
            spec.build_pose2d_payload(persons, width, height, labeled),
            ensure_ascii=False)))

        payload = self._measure(
            persons, width, height, started, time.monotonic() - received_at)
        self._last_payload = payload
        self._angles_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    def _measure(self, persons, width, height, started, frame_age_s):
        """One frame's worth of retargeting, including every reason it may be junk."""
        person = spec.pick_person(persons, self._min_kp_conf)
        # Identity, not equality: two people can produce equal dictionaries, and
        # `list.index` would then label the target as whoever matched first.
        target_id = next(
            (index for index, candidate in enumerate(persons) if candidate is person),
            None)
        if person is None:
            return spec.build_angles_payload(
                tracked=False, person_count=len(persons), target_id=None,
                angles_by_side=None, clamped_by_side=None, depth_used=False,
                depth_coverage=0.0, focal_px=self._numeric_focal(),
                latency_ms=(time.monotonic() - started) * 1000.0,
                frame_age_ms=frame_age_s * 1000.0, reach=None,
                **self._signs, degraded="no_person" if not persons else "low_confidence")

        points = spec.keypoints_of(person)
        left_ok = spec.upper_body_visible(points, _PERSON_LEFT, self._min_kp_conf)
        right_ok = spec.upper_body_visible(points, _PERSON_RIGHT, self._min_kp_conf)
        if not left_ok and not right_ok:
            return spec.build_angles_payload(
                tracked=False, person_count=len(persons),
                target_id=target_id,
                angles_by_side=None, clamped_by_side=None, depth_used=False,
                depth_coverage=0.0, focal_px=self._numeric_focal(),
                latency_ms=(time.monotonic() - started) * 1000.0,
                frame_age_ms=frame_age_s * 1000.0, reach=None,
                **self._signs, degraded="upper_body_occluded")

        depth_frame, _, _ = self._depth_cache.decode()
        depth_fresh = self._depth_cache.age_s()
        usable_depth = (
            depth_frame is not None
            and depth_fresh is not None
            and depth_fresh <= _DEPTH_FRESH_S)

        focal_px = self._resolve_focal(points, depth_frame, usable_depth, width, height)
        scene, coverage = self._scene_points(points, depth_frame, usable_depth, focal_px,
                                             width, height)

        angles_by_side = {}
        clamped_by_side = {}
        for side, needed, (shoulder, elbow, wrist) in (
            ("left", _PERSON_RIGHT, (spec.R_SHOULDER, spec.R_ELBOW, spec.R_WRIST)),
            ("right", _PERSON_LEFT, (spec.L_SHOULDER, spec.L_ELBOW, spec.L_WRIST)),
        ):
            # Mirror mapping: the person's right arm drives the robot's left,
            # because the two face each other and each one's left is on the
            # other's right. See spec.arm_angles for why no axis is flipped.
            if not spec.upper_body_visible(points, needed, self._min_kp_conf):
                continue
            trio = (scene[shoulder], scene[elbow], scene[wrist])
            if any(point is None for point in trio):
                continue
            try:
                raw = spec.arm_angles(scene[shoulder], scene[elbow], scene[wrist])
            except ValueError:
                continue
            signed = spec.apply_signs(raw, **self._signs)
            limited, clamped = spec.limit_angles(signed, side)
            angles_by_side[side] = limited
            clamped_by_side[side] = clamped

        if not angles_by_side:
            return spec.build_angles_payload(
                tracked=False, person_count=len(persons),
                target_id=target_id,
                angles_by_side=None, clamped_by_side=None, depth_used=False,
                depth_coverage=coverage, focal_px=focal_px,
                latency_ms=(time.monotonic() - started) * 1000.0,
                frame_age_ms=frame_age_s * 1000.0, reach=None,
                **self._signs, degraded="degenerate_projection")

        smoothed = self._bank.update(angles_by_side, time.monotonic())
        angles_out = {}
        for side in ("left", "right"):
            prefix = side[0]
            # Emit the held value for every side the bank has ever seen, not
            # just the sides measured in this frame. When an arm is occluded
            # `angles_by_side` drops it but the bank still holds its last
            # filtered angle; sending -999 for that side would turn a brief
            # occlusion into a pose discontinuity — the exact failure this
            # filter exists to prevent. A side never seen at all is still
            # omitted, and the reader sees -999 = "never measured".
            if all(f"{prefix}_{axis}" in smoothed
                   for axis in ("pitch", "roll", "elbow")):
                angles_out[side] = {
                    "pitch_deg": smoothed[f"{prefix}_pitch"],
                    "roll_deg": smoothed[f"{prefix}_roll"],
                    "elbow_deg": smoothed[f"{prefix}_elbow"],
                }

        reach = None
        reach_trio = (scene[spec.R_SHOULDER], scene[spec.R_ELBOW], scene[spec.R_WRIST])
        if (spec.upper_body_visible(points, _PERSON_RIGHT, self._min_kp_conf)
                and not any(point is None for point in reach_trio)):
            reach = spec.reach_indicator(*reach_trio)

        degraded = ""
        if not usable_depth:
            degraded = ("depth_stale" if depth_frame is not None else "no_depth_flat_plane")
        elif coverage < 1.0:
            degraded = "partial_depth"
        elif len(angles_by_side) < 2:
            degraded = "one_arm_only"

        return spec.build_angles_payload(
            tracked=True, person_count=len(persons),
            target_id=target_id,
            angles_by_side=angles_out, clamped_by_side=clamped_by_side,
            depth_used=bool(usable_depth), depth_coverage=coverage,
            focal_px=focal_px,
            latency_ms=(time.monotonic() - started) * 1000.0,
            frame_age_ms=frame_age_s * 1000.0, reach=reach,
            **self._signs, degraded=degraded)

    # ── Depth helpers ────────────────────────────────────────────────────────

    def _numeric_focal(self):
        try:
            return float(self._focal_setting)
        except (TypeError, ValueError):
            return _DEFAULT_FOCAL_PX

    def _resolve_focal(self, points, depth_frame, usable_depth, width, height):
        if str(self._focal_setting).strip().lower() != "auto":
            return self._numeric_focal()
        if not usable_depth:
            return _DEFAULT_FOCAL_PX
        if not spec.upper_body_visible(points, (spec.L_SHOULDER, spec.R_SHOULDER),
                                       self._min_kp_conf):
            return _DEFAULT_FOCAL_PX
        depth = spec.median([
            self._sample_depth(depth_frame, points[spec.L_SHOULDER]),
            self._sample_depth(depth_frame, points[spec.R_SHOULDER]),
        ])
        if depth is None:
            return _DEFAULT_FOCAL_PX
        left, right = points[spec.L_SHOULDER], points[spec.R_SHOULDER]
        shoulder_px = ((left[0] - right[0]) ** 2 + (left[1] - right[1]) ** 2) ** 0.5
        focal = spec.estimate_focal_px(shoulder_px, depth)
        if focal is None:
            return _DEFAULT_FOCAL_PX
        return spec.clamp(focal, 200.0, 1200.0)

    def _sample_depth(self, depth_frame, point):
        """Median depth in metres around a keypoint, or `None` when no echo landed."""
        if depth_frame is None:
            return None
        u = int(round(point[0]))
        v = int(round(point[1]))
        height, width = depth_frame.shape[:2]
        half = _DEPTH_PATCH // 2
        u0, u1 = max(0, u - half), min(width, u + half + 1)
        v0, v1 = max(0, v - half), min(height, v + half + 1)
        if u1 <= u0 or v1 <= v0:
            return None
        patch = depth_frame[v0:v1, u0:u1]
        values = (patch.astype(np.float32) / 1000.0).ravel().tolist()
        return spec.median(values)

    def _scene_points(self, points, depth_frame, usable_depth, focal_px, width, height):
        """Keypoints → body-frame points, with the fallbacks made explicit.

        Preference order per keypoint: its own depth echo, then the median depth
        of the whole person, then the bare ray direction. The last one treats the
        person as a billboard facing the camera — it still recovers abduction and
        elbow flexion, but it cannot see an arm reaching toward the robot, so it
        is reported (coverage 0, `degraded: no_depth_flat_plane`) instead of
        being mixed in silently.
        """
        count = len(points)
        scene = [None] * count
        # `backproject` with depth == focal_px yields the pixel ray direction: the
        # depth factor cancels against the (u - cx)/f term. Reusing it keeps one
        # implementation of the frame conversion instead of two that can drift.
        def flat(x, y):
            return spec.backproject(x, y, focal_px, focal_px, width, height)

        if not usable_depth:
            for index in range(count):
                x, y, _ = points[index]
                scene[index] = flat(x, y)
            return scene, 0.0

        sampled = [self._sample_depth(depth_frame, point) for point in points]
        resolved = sum(1 for value in sampled if value)
        fallback = spec.median(sampled)
        for index in range(count):
            x, y, _ = points[index]
            depth = sampled[index] or fallback
            scene[index] = (
                spec.backproject(x, y, depth, focal_px, width, height)
                if depth else flat(x, y))
        coverage = resolved / float(count) if count else 0.0
        return scene, coverage
