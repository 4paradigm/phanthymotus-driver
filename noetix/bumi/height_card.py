"""`height` card — measure a person's or a floor-standing object's height.

Lives in the Bumi driver bundle rather than in perception for one reason: the
depth stream and the RealSense intrinsics are produced here, and a card in this
process can read the camera's just-negotiated `K` at the moment of a
measurement instead of depending on a `motus.camera/1` declaration that only
travels once, at project start. That removes the cold-start case where a
measurement card starts before the camera has reported its intrinsics.

Read-only: it subscribes to the bundle's own depth topic
(`/<namespace>/camera/depth`), optionally to one pose/vop detection stream, and
answers tool calls. It never publishes control and never declares `x-resource`.

Two ways to get "above the floor", reported in the reply as `mode`:

* **floor** — fit the floor in the same depth frame; the target's distance to
  that plane is its height. Needs the floor in view and absorbs the robot's own
  posture.
* **mount** — the camera's calibrated height and pitch. Needs no floor, but is
  only as good as that one-time calibration.

The card is disabled in `config.yaml` by default: a new measurement capability
must not appear on an existing deployment until an operator enables it.
"""

from __future__ import annotations

import json
import math
import threading
import time

import numpy as np

from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import height_math as geometry

# Sensor streams are BEST_EFFORT end to end on this platform. A RELIABLE
# subscriber against a BEST_EFFORT publisher receives nothing, with no error.
_SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
_RESULT_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)

_MEASURE_TIMEOUT_S = 3.0
_MAX_FRAME_SPREAD_M = 0.03
_MIN_PLANE_HEIGHT_M = 0.20
_MAX_PLANE_HEIGHT_M = 3.0

# Also the `configSchema` defaults. Keep them equal: agent-core sends every
# schema field (default included) on every `config` call, so a schema default
# that disagrees with config.yaml silently wins.
_CONFIG_DEFAULTS = {
    "camera_height_m": 0.0,      # 0 = not calibrated
    "camera_pitch_deg": 0.0,     # positive = optical axis below horizontal
    "camera_roll_deg": 0.0,
    "fx": 0.0, "fy": 0.0, "cx": 0.0, "cy": 0.0,   # 0 = use the camera's K
    "frames": 5,
    "max_range_m": 4.5,
    # Card-only settings, deliberately not in configSchema so a schema default
    # cannot silently override them.
    "max_detection_age_s": 0.8,
    "min_valid_ratio": 0.5,
    "calibration_frames": 12,
}

_TOOL = {
    "name": "height",
    "type": "processor",
    "multiInstance": False,
    "description": (
        "Measure the height of one person or one floor-standing object from "
        "Bumi's depth camera. The card subscribes to this bundle's "
        "`/<namespace>/camera/depth` internally; optionally wire a pose or vop "
        "`data/json` stream to say which target. Returns height in metres with "
        "an uncertainty and the mode used, or a reason it could not measure — "
        "it never guesses."
    ),
    "inputSchema": {
        "type": "object",
        "properties": {
            "action": {
                "type": "string",
                "enum": ["start", "stop", "measure", "calibrate", "info",
                         "config"],
            },
            "target": {
                "type": "string",
                "description": (
                    "What to measure: 'person', 'object', 'auto' (default), or "
                    "a class name from the vop detections (e.g. 'box'). A "
                    "person is located from the pose card's boxes, an object "
                    "from the vop card's boxes."
                ),
            },
            "bbox": {
                "type": "array",
                "items": {"type": "number"},
                "minItems": 4,
                "maxItems": 4,
                "description": (
                    "Optional normalised [x1,y1,x2,y2] in 0..1. For testing or "
                    "when an external caller already knows the target box; "
                    "normally the detection input supplies it."
                ),
            },
        },
        "required": ["action"],
        "x-action-params": {
            "measure": {
                "params": ["target", "bbox"],
                "description": (
                    "Measure one target's height. The target must be still and "
                    "its top (person: head, object: top face) must be inside "
                    "the picture with a few pixels of margin. Returns "
                    "height_m / uncertainty_m / mode / frames_used, or an "
                    "error with a reason (no_camera_info, "
                    "no_ground_reference, target_clipped, too_far, "
                    "ambiguous_target...)."
                ),
            },
            "calibrate": {
                "params": [],
                "description": (
                    "One-time mount calibration from the floor in view: "
                    "returns camera_height_m / camera_pitch_deg / "
                    "camera_roll_deg and a suggested config block. Save those "
                    "values in the card's configuration; the camera is fixed, "
                    "so this is not needed per measurement."
                ),
            },
        },
    },
    "configSchema": {
        "type": "object",
        "properties": {
            "camera_height_m": {
                "type": "number", "default": 0.0, "scope": "shared",
                "minimum": 0.0, "maximum": 3.0,
                "title": "Camera height above floor (m)",
                "description": (
                    "0 = not calibrated (floor mode only). Set this from the "
                    "`calibrate` action or a tape measure; the camera is "
                    "fixed, so it is measured once per robot."
                ),
            },
            "camera_pitch_deg": {
                "type": "number", "default": 0.0, "scope": "shared",
                "minimum": -45.0, "maximum": 45.0,
                "title": "Camera pitch (deg, positive = looking down)",
                "description": "Mounting pitch from the `calibrate` action; "
                               "used by mount mode when the floor is not in "
                               "view.",
            },
            "camera_roll_deg": {
                "type": "number", "default": 0.0, "scope": "shared",
                "minimum": -45.0, "maximum": 45.0,
                "title": "Camera roll (deg)",
                "description": "Mounting roll from the `calibrate` action.",
            },
            "frames": {
                "type": "integer", "default": 5, "scope": "shared",
                "minimum": 2, "maximum": 30,
                "title": "Frames averaged per measurement",
                "description": "The target must hold still for this many "
                               "frames; the spread across them must stay "
                               "under 3 cm.",
            },
            "max_range_m": {
                "type": "number", "default": 4.5, "scope": "shared",
                "minimum": 0.5, "maximum": 10.0,
                "title": "Maximum depth range (m)",
                "description": "Depth beyond this is ignored for the floor fit "
                               "and the target; the D435i is most accurate "
                               "inside about 3 m.",
            },
            "fx": {"type": "number", "default": 0.0, "scope": "shared",
                   "advanced": True, "minimum": 0.0,
                   "title": "Manual fx (px), 0 = use the camera's K"},
            "fy": {"type": "number", "default": 0.0, "scope": "shared",
                   "advanced": True, "minimum": 0.0,
                   "title": "Manual fy (px), 0 = use the camera's K"},
            "cx": {"type": "number", "default": 0.0, "scope": "shared",
                   "advanced": True, "minimum": 0.0,
                   "title": "Manual cx (px), 0 = use the camera's K"},
            "cy": {"type": "number", "default": 0.0, "scope": "shared",
                   "advanced": True, "minimum": 0.0,
                   "title": "Manual cy (px), 0 = use the camera's K"},
        },
        "required": [],
    },
    # The depth input is internal (`/<namespace>/camera/depth`) on purpose: the
    # card lives in the same bundle as the camera, so making an operator wire
    # the camera back into its own driver would add a failure mode without
    # adding capability. Only the optional detection stream is a port.
    "topic_in": [
        {"format": "data/json",
         "desc": "Optional detection: pose /poses (people) or vop /objects "
                 "(objects), from perception"},
    ],
    "topic_out": [
        {"format": "data/json",
         "desc": "Last measurement or calibration result (dashboard / bus)"},
    ],
}

TOOLS = [_TOOL]


# ── ROS node ─────────────────────────────────────────────────────────────────

class _HeightNode(Node):
    """Owns the depth subscription, the optional detection subscription and the
    frame-window condition variable.

    No worker thread: a measurement is a bounded loop over the next N depth
    frames, so the only state while idle is one 614 KB payload and one dict.
    """

    def __init__(self, depth_topic: str, result_topic: str):
        super().__init__("bumi_height")
        self._depth_topic = depth_topic
        self._condition = threading.Condition()
        self._depth_seq = 0
        self._depth_bytes = None
        self._depth_arrival = 0.0
        self._detection_topic = ""
        self._detection = None
        self._detection_arrival = 0.0
        self._depth_sub = None
        self._detection_sub = None
        self._result_pub = self.create_publisher(String, result_topic,
                                                 _RESULT_QOS)

    def start(self) -> dict:
        if self._depth_sub is None:
            self._depth_sub = self.create_subscription(
                CompressedImage, self._depth_topic, self._on_depth,
                _SENSOR_QOS)
        return {"state": "running"}

    def bind_detection(self, topic: str) -> None:
        """(Re)subscribe to one detection stream, or drop it when empty."""
        if topic == self._detection_topic and (
                topic == "" or self._detection_sub is not None):
            return
        previous = self._detection_sub
        self._detection_sub = None
        self._detection_topic = topic
        if previous is not None:
            try:
                self.destroy_subscription(previous)
            except Exception:                                 # noqa: BLE001
                pass
        if topic:
            self._detection_sub = self.create_subscription(
                String, topic, self._on_detection, _SENSOR_QOS)
        else:
            self._detection = None
            self._detection_arrival = 0.0

    def stop(self) -> None:
        # The node itself is destroyed by _dispose_node after it leaves the
        # executor; destroying subscriptions here can race the executor's wait
        # list (see the same note in perception's visual_depth card).
        self._depth_sub = None
        self._detection_sub = None

    def _on_depth(self, message) -> None:
        with self._condition:
            self._depth_bytes = bytes(message.data)
            self._depth_arrival = time.monotonic()
            self._depth_seq += 1
            self._condition.notify_all()

    def _on_detection(self, message) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        with self._condition:
            self._detection = payload
            self._detection_arrival = time.monotonic()

    def depth_sequence(self) -> int:
        with self._condition:
            return self._depth_seq

    def wait_for_depth(self, after_seq: int, timeout_s: float):
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while self._depth_seq <= after_seq:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None, self._depth_seq
                self._condition.wait(remaining)
            return (self._depth_bytes, self._depth_arrival), self._depth_seq

    def latest_detection(self):
        with self._condition:
            return self._detection, self._detection_arrival

    def publish_result(self, result: dict) -> None:
        message = String()
        message.data = json.dumps(result, ensure_ascii=False)
        self._result_pub.publish(message)


# ── card ────────────────────────────────────────────────────────────────────

class HeightPlugin:
    PREFIX = "height"

    def __init__(self, plugin_cfg: dict, namespace: str, executor,
                 camera_plugin=None):
        self._namespace = namespace
        self._camera = camera_plugin
        self._executor = executor
        self._depth_topic = f"/{namespace}/camera/depth"
        self._result_topic = f"/{namespace}/height/last"
        self._cfg = dict(_CONFIG_DEFAULTS)
        self._merge_config(plugin_cfg or {})
        self._node: _HeightNode | None = None
        self._detection_topic = ""
        self._detection_size = (geometry.DEPTH_WIDTH, geometry.DEPTH_HEIGHT)
        self._last_result = None
        self._last_calibration = None
        self._lock = threading.RLock()

    # -- tool contract -----------------------------------------------------

    def get_tools(self) -> list:
        return list(TOOLS)

    def dispatch(self, action: str, args: dict):
        args = args or {}
        if action == "start":
            return self._start(args)
        if action == "stop":
            self._close()
            return {"state": "idle"}
        if action == "measure":
            return self._measure(args)
        if action == "calibrate":
            return self._calibrate(args)
        if action == "info":
            return self._info(args)
        if action == "config":
            return self._config(args)
        return None

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        """Bundle lifecycle: subscribe to the depth stream and stay idle.

        The driver starts every enabled plugin at boot. That is why the depth
        subscription is created here rather than lazily: the camera starts
        immediately above this plugin in the bundle, so frames are already
        flowing by the time anybody calls `measure`.
        """
        if self._node is None:
            self._open("")

    def stop(self) -> None:
        self._close()

    def _open(self, detection_topic: str) -> None:
        node = _HeightNode(self._depth_topic, self._result_topic)
        self._executor.add_node(node)
        node.start()
        node.bind_detection(detection_topic)
        self._node = node
        self._detection_topic = detection_topic
        print(f"[height] node started: depth={self._depth_topic} "
              f"detection={detection_topic or '(none)'}", flush=True)

    def _close(self) -> None:
        node, self._node = self._node, None
        self._detection_topic = ""
        if node is None:
            return
        try:
            node.stop()
        except Exception:                                     # noqa: BLE001
            pass
        _dispose_node(self._executor, node)
        print("[height] node stopped", flush=True)

    def _start(self, args: dict):
        detection_topic, detection_size = self._detection_from_args(args)
        self._detection_size = detection_size
        if self._node is None:
            self._open(detection_topic)
        elif detection_topic != self._detection_topic:
            # The project was re-wired between starts (added, removed or
            # changed detector). Recreate the node so the old subscription
            # cannot keep feeding a detector that is gone.
            self._close()
            self._open(detection_topic)
        degraded = self._degraded_notes()
        return {
            "state": "running",
            "inputs": {"depth": self._depth_topic,
                       "detection": self._detection_topic or None},
            "intrinsics": self._intrinsics_summary(),
            "calibration": self._calibration(),
            "degraded": degraded,
        }

    def _detection_from_args(self, args: dict):
        topics = _input_topics(args)
        detection = next((topic for topic in topics
                          if _is_detection_topic(topic)), "")
        size = (geometry.DEPTH_WIDTH, geometry.DEPTH_HEIGHT)
        if detection:
            declaration = geometry.declaration_for(
                args.get("camera_info") or {}, detection)
            if declaration.get("width") and declaration.get("height"):
                size = (int(declaration["width"]),
                        int(declaration["height"]))
        return detection, size

    # -- configuration -----------------------------------------------------

    def _merge_config(self, incoming: dict) -> None:
        for key, default in _CONFIG_DEFAULTS.items():
            if key not in incoming:
                continue
            value = incoming[key]
            try:
                if isinstance(default, bool):
                    value = bool(value)
                elif isinstance(default, int):
                    value = int(value)
                else:
                    value = float(value)
            except (TypeError, ValueError):
                raise ValueError(f"{key} must be a number")
            self._cfg[key] = value
        self._validate_config()

    def _validate_config(self) -> None:
        if not 0.0 <= self._cfg["camera_height_m"] <= 3.0:
            raise ValueError("camera_height_m must be in [0, 3]")
        for key in ("camera_pitch_deg", "camera_roll_deg"):
            if not -45.0 <= self._cfg[key] <= 45.0:
                raise ValueError(f"{key} must be in [-45, 45]")
        for key in ("fx", "fy", "cx", "cy"):
            if self._cfg[key] < 0.0:
                raise ValueError(f"{key} must be >= 0")
        if not 2 <= self._cfg["frames"] <= 30:
            raise ValueError("frames must be in [2, 30] (a single frame has "
                             "no spread to check)")
        if not 0.5 <= self._cfg["max_range_m"] <= 10.0:
            raise ValueError("max_range_m must be in [0.5, 10]")
        if not 0.05 <= self._cfg["max_detection_age_s"] <= 5.0:
            raise ValueError("max_detection_age_s must be in [0.05, 5]")
        if not 0.0 <= self._cfg["min_valid_ratio"] <= 1.0:
            raise ValueError("min_valid_ratio must be in [0, 1]")
        if not 2 <= self._cfg["calibration_frames"] <= 60:
            raise ValueError("calibration_frames must be in [2, 60]")

    def _public_config(self) -> dict:
        return {key: self._cfg[key] for key in _CONFIG_DEFAULTS}

    def _config(self, args: dict):
        body = {key: value for key, value in args.items()
                if key in _CONFIG_DEFAULTS}
        if not body:
            return {"status": "configured", "config": self._public_config()}
        try:
            self._merge_config(body)
        except ValueError as error:
            return {"status": "error", "message": str(error)}
        return {"status": "configured", "config": self._public_config()}

    # -- intrinsics --------------------------------------------------------

    def _intrinsics(self) -> dict:
        """The camera's depth intrinsics, or the configured fallback.

        Read at call time, not at start: the RealSense subprocess reports its
        intrinsics asynchronously, and a measurement started after the camera
        settles must use the real `K` rather than failing a cold start.
        """
        stream = {}
        if self._camera is not None:
            try:
                stream = self._camera.depth_intrinsics() or {}
            except Exception:                                 # noqa: BLE001
                stream = {}
        if isinstance(stream, dict) and stream:
            try:
                width = int(stream["width"])
                height = int(stream["height"])
                fx = float(stream["fx"])
                fy = float(stream["fy"])
                cx = float(stream["ppx"])
                cy = float(stream["ppy"])
            except (KeyError, TypeError, ValueError):
                width = height = 0
            if width > 0 and height > 0 and fx > 0 and fy > 0 and cx > 0 and cy > 0:
                if (width, height) != (geometry.DEPTH_WIDTH,
                                       geometry.DEPTH_HEIGHT):
                    raise geometry.GeometryError(
                        "intrinsics_size_mismatch",
                        f"camera reports {width}x{height}; the depth contract "
                        f"is {geometry.DEPTH_WIDTH}x{geometry.DEPTH_HEIGHT}")
                return {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                        "width": width, "height": height,
                        "source": "camera-runtime", "notes": []}

        configured = (self._cfg["fx"], self._cfg["fy"],
                      self._cfg["cx"], self._cfg["cy"])
        if all(value > 0 for value in configured):
            fx, fy, cx, cy = configured
            return {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                    "width": geometry.DEPTH_WIDTH,
                    "height": geometry.DEPTH_HEIGHT,
                    "source": "config-manual",
                    "notes": ["using manually configured intrinsics; the "
                              "camera has not reported its own K"]}

        raise geometry.GeometryError(
            "no_camera_info",
            "the camera has not reported its RealSense intrinsics yet. Wait "
            "for the depth stream to start and try again, or set fx/fy/cx/cy "
            "in the card config (they are then marked manual).")

    def _intrinsics_summary(self) -> dict:
        try:
            intrinsics = self._intrinsics()
        except geometry.GeometryError as error:
            return {"source": "unavailable", "reason": error.reason,
                    "message": error.message}
        return {key: intrinsics[key] for key in
                ("fx", "fy", "cx", "cy", "width", "height", "source")}

    # -- measurement -------------------------------------------------------

    def _measure(self, args: dict):
        if self._node is None:
            return _error("not_running", "the height card is not running; "
                                         "start the project first")
        try:
            intrinsics = self._intrinsics()
            target = self._resolve_target(args)
        except geometry.GeometryError as error:
            return _error(error.reason, error.message)

        node = self._node
        samples = []
        last_failure = None
        baseline = node.depth_sequence()
        deadline = time.monotonic() + _MEASURE_TIMEOUT_S
        for _ in range(int(self._cfg["frames"])):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            frame, sequence = node.wait_for_depth(baseline, min(1.5, remaining))
            if frame is None:
                break
            baseline = sequence
            try:
                samples.append(self._sample(frame, target, intrinsics))
            except geometry.GeometryError as error:
                last_failure = error
        if len(samples) < 2:
            return _error(
                last_failure.reason if last_failure else "insufficient_frames",
                last_failure.message if last_failure else
                f"only {len(samples)} usable frame(s); the target must hold "
                f"still while the depth frames arrive")

        _, spread = geometry.median_and_spread(
            [sample["height_m"] for sample in samples])
        if spread > _MAX_FRAME_SPREAD_M:
            return _error(
                "unstable",
                f"the height moved {spread * 100:.1f} cm across "
                f"{len(samples)} frames; ask the target to stand still and "
                f"measure again")
        valid_ratio = float(np.median([sample["valid_ratio"]
                                       for sample in samples]))
        if valid_ratio < float(self._cfg["min_valid_ratio"]):
            return _error(
                "no_depth_at_target",
                f"only {valid_ratio * 100:.0f}% of the sampled top columns "
                f"had a depth reading")
        height_m, _ = geometry.median_and_spread(
            [sample["height_m"] for sample in samples])
        if not geometry.plausible_height(height_m):
            return _error(
                "implausible_height",
                f"measured {height_m:.2f} m, outside the "
                f"{geometry.MIN_HEIGHT_M:.2f}-{geometry.MAX_HEIGHT_M:.2f} m "
                f"this card will report")
        modes = sorted({sample["mode"] for sample in samples})
        result = {
            "state": "ok",
            "height_m": round(height_m, 3),
            "uncertainty_m": round(max(0.02, spread / 2.0), 3),
            "height_definition": ("floor_to_top_of_head"
                                  if target["kind"] == "person"
                                  else "floor_to_top_of_object"),
            "target": {
                "kind": target["kind"],
                "label": target["label"],
                "track_id": target.get("track_id"),
                "bbox_px": [round(float(v), 1) for v in target["bbox_px"]],
                "candidate_count": target.get("candidate_count", 1),
            },
            "mode": modes[0] if len(modes) == 1 else "mixed",
            "camera_height_m": _reported_camera_height(
                self._last_calibration, self._cfg["camera_height_m"]),
            "frames_used": len(samples),
            "spread_m": round(spread, 3),
            "valid_ratio": round(valid_ratio, 3),
            "distance_m": round(float(np.median([sample["z"]
                                                 for sample in samples])), 3),
            "degraded": self._degraded_notes(),
            "measured_at": _timestamp(),
        }
        self._last_result = result
        node.publish_result(result)
        return result

    def _sample(self, frame, target: dict, intrinsics: dict) -> dict:
        payload, _ = frame
        depth = geometry.decode_depth(payload)
        bbox = target["bbox_px"]
        top = geometry.top_point(depth, bbox, intrinsics)
        plane = None
        try:
            plane = geometry.floor_plane_from_depth(
                depth, intrinsics, max_range_m=self._cfg["max_range_m"],
                exclude_bbox=bbox)
            usable = (
                _MIN_PLANE_HEIGHT_M <= plane["height_m"] <= _MAX_PLANE_HEIGHT_M
                and plane["residual_m"] <= geometry.MAX_PLANE_RESIDUAL_M
                and plane["inlier_ratio"] >= geometry.MIN_PLANE_INLIERS)
            if usable:
                height_m = geometry.height_above_plane(top["point"], plane)
                mode = "floor"
            else:
                raise geometry.GeometryError(
                    "no_ground_reference", "the fitted plane is not a floor")
        except geometry.GeometryError:
            if self._cfg["camera_height_m"] > 0.0:
                height_m = geometry.height_from_mount(
                    top["point"], self._cfg["camera_height_m"],
                    math.radians(self._cfg["camera_pitch_deg"]),
                    math.radians(self._cfg["camera_roll_deg"]))
                mode = "mount"
            else:
                raise geometry.GeometryError(
                    "no_ground_reference",
                    "no usable floor in view and no mount calibration; set "
                    "camera_height_m/camera_pitch_deg once (or run `calibrate` "
                    "somewhere the floor is visible)")
        if not geometry.plausible_height(height_m):
            raise geometry.GeometryError(
                "implausible_height",
                f"frame measured {height_m:.2f} m, outside the "
                f"{geometry.MIN_HEIGHT_M:.2f}-{geometry.MAX_HEIGHT_M:.2f} m "
                f"band")
        return {
            "height_m": height_m,
            "mode": mode,
            "z": top["z"],
            "valid_ratio": top["valid_ratio"],
            "plane": plane,
        }

    def _resolve_target(self, args: dict) -> dict:
        bbox = args.get("bbox")
        if bbox is not None:
            try:
                values = [float(v) for v in bbox]
            except (TypeError, ValueError):
                raise geometry.GeometryError(
                    "bad_bbox", "bbox must be four numbers in 0..1")
            if len(values) != 4 or any(not 0.0 <= v <= 1.0 for v in values):
                raise geometry.GeometryError(
                    "bad_bbox", "bbox must be four numbers in 0..1")
            width, height = geometry.DEPTH_WIDTH, geometry.DEPTH_HEIGHT
            return {
                "bbox_px": geometry.clamp_bbox(
                    [values[0] * width, values[1] * height,
                     values[2] * width, values[3] * height], width, height),
                "kind": "manual",
                "label": "manual",
                "track_id": None,
                "candidate_count": 1,
            }

        if not self._detection_topic:
            raise geometry.GeometryError(
                "no_detection",
                "no detection input and no explicit bbox. Wire the pose card "
                "(people) or the vop card (objects) into this card's "
                "data/json input.")
        payload, arrival = self._node.latest_detection()
        if payload is None:
            raise geometry.GeometryError(
                "no_detection",
                "the detection input has not published anything yet")
        if time.monotonic() - arrival > float(self._cfg["max_detection_age_s"]):
            raise geometry.GeometryError(
                "stale_detection",
                f"the newest detection is {time.monotonic() - arrival:.1f} s "
                f"old; the detector may have stopped")
        source = "pose" if self._detection_topic.endswith("/poses") else "vop"
        wanted = str(args.get("target") or "auto").strip()
        if source == "pose" and wanted.lower() not in ("", "auto", "person"):
            wanted = "auto"
        return geometry.pick_detection(
            payload, source, wanted, self._detection_size,
            (geometry.DEPTH_WIDTH, geometry.DEPTH_HEIGHT))

    # -- calibration -------------------------------------------------------

    def _calibrate(self, args: dict):
        if self._node is None:
            return _error("not_running", "the height card is not running; "
                                         "start the project first")
        try:
            intrinsics = self._intrinsics()
        except geometry.GeometryError as error:
            return _error(error.reason, error.message)
        node = self._node
        wanted = int(self._cfg["calibration_frames"])
        planes = []
        baseline = node.depth_sequence()
        deadline = time.monotonic() + 6.0
        while len(planes) < wanted and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            frame, sequence = node.wait_for_depth(baseline,
                                                  min(1.5, remaining))
            if frame is None:
                break
            baseline = sequence
            try:
                depth = geometry.decode_depth(frame[0])
                plane = geometry.floor_plane_from_depth(
                    depth, intrinsics, max_range_m=self._cfg["max_range_m"])
            except geometry.GeometryError:
                continue
            if (_MIN_PLANE_HEIGHT_M <= plane["height_m"] <= _MAX_PLANE_HEIGHT_M
                    and plane["residual_m"] <= geometry.MAX_PLANE_RESIDUAL_M):
                planes.append(plane)
        if len(planes) < 3:
            return _error(
                "no_ground_reference",
                "not enough frames with a visible floor. Put the robot where "
                "the floor in front of it is in view (about 2.5-4 m, no table "
                "or cable in the way), then run `calibrate` again.")
        height_m = float(np.median([plane["height_m"] for plane in planes]))
        pitch = float(np.median([plane["pitch_rad"] for plane in planes]))
        roll = float(np.median([plane["roll_rad"] for plane in planes]))
        residual = float(np.median([plane["residual_m"] for plane in planes]))
        result = {
            "state": "ok",
            "camera_height_m": round(height_m, 4),
            "camera_pitch_deg": round(math.degrees(pitch), 2),
            "camera_roll_deg": round(math.degrees(roll), 2),
            "residual_m": round(residual, 4),
            "frames": len(planes),
            "mode": "floor-fit",
            "suggested_config": {
                "camera_height_m": round(height_m, 4),
                "camera_pitch_deg": round(math.degrees(pitch), 2),
                "camera_roll_deg": round(math.degrees(roll), 2),
            },
            "note": ("Save suggested_config in this card's configuration; the "
                     "camera is fixed, so this is a one-time calibration."),
            "measured_at": _timestamp(),
        }
        self._last_calibration = result
        self._last_result = result
        node.publish_result(result)
        return result

    # -- info --------------------------------------------------------------

    def _info(self, args: dict):
        base = {
            "name": "BumiHeight",
            "manufacture": "Noetix",
            "config": self._public_config(),
        }
        if self._node is None:
            return {**base, "state": "idle", "degraded": ["not_running"]}
        topics_in = []
        if self._detection_topic:
            topics_in.append({"topic": self._detection_topic,
                              "format": "data/json"})
        return {
            **base,
            "state": "running",
            "inputs": {"depth": self._depth_topic,
                       "detection": self._detection_topic or None},
            "topic_in": topics_in,
            "topic_out": [{"topic": self._result_topic,
                           "format": "data/json"}],
            "intrinsics": self._intrinsics_summary(),
            "calibration": self._calibration(),
            "last_result": self._last_result,
            "last_calibration": self._last_calibration,
            "degraded": self._degraded_notes(),
        }

    # -- helpers -----------------------------------------------------------

    def _calibration(self) -> dict:
        return {
            "camera_height_m": self._cfg["camera_height_m"],
            "camera_pitch_deg": self._cfg["camera_pitch_deg"],
            "camera_roll_deg": self._cfg["camera_roll_deg"],
            "source": "configured" if self._cfg["camera_height_m"] > 0
                      else "uncalibrated",
        }

    def _degraded_notes(self) -> list:
        notes = []
        if self._cfg["camera_height_m"] <= 0.0:
            notes.append("no mount calibration: floor mode only; run "
                         "`calibrate` or set camera_height_m")
        summary = self._intrinsics_summary()
        if summary.get("source") == "config-manual":
            notes.extend(summary.get("notes") or [])
        elif summary.get("source") == "unavailable":
            notes.append("camera intrinsics unavailable: "
                         + str(summary.get("message") or ""))
        return notes


# ── module helpers ──────────────────────────────────────────────────────────

def _input_topics(args: dict) -> list:
    topics: list = []
    for topic in args.get("input_topics") or []:
        if isinstance(topic, str) and topic and topic not in topics:
            topics.append(topic)
    single = args.get("input_topic") or ""
    if isinstance(single, str) and single and single not in topics:
        topics.insert(0, single)
    return topics


def _is_detection_topic(topic: str) -> bool:
    return topic.endswith("/objects") or topic.endswith("/poses")


def _reported_camera_height(calibration, configured: float):
    if isinstance(calibration, dict) and "camera_height_m" in calibration:
        return round(float(calibration["camera_height_m"]), 3)
    return round(float(configured), 3) if configured > 0 else None


def _dispose_node(executor, node) -> None:
    try:
        executor.remove_node(node)
    except Exception:                                         # noqa: BLE001
        pass
    try:
        node.destroy_node()
    except Exception:                                         # noqa: BLE001
        pass


def _error(reason: str, message: str) -> dict:
    return {"state": "error", "reason": reason, "message": message}


def _timestamp() -> str:
    from datetime import datetime
    return datetime.now().astimezone().isoformat(timespec="seconds")
