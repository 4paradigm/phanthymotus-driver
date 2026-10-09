"""Read-only, reference-tag-relative Q5 left-hand repeatability measurement.

The AprilTag detector owns image processing. This card consumes its detection
metadata and the transforms stamped with the same camera image time. It never
publishes a robot command.
"""

from __future__ import annotations

from collections import Counter, deque
import math
import statistics
import threading
import time


CARD = "visual_ee_repeatability"
LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
    "left_arm_yaw_joint", "left_elbow_pitch_joint", "left_elbow_yaw_joint",
    "left_wrist_pitch_joint", "left_wrist_roll_joint",
)

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo
    from tf2_ros import Buffer, TransformListener
    _HAS_ROS2 = True
except ImportError:
    _HAS_ROS2 = False


def _unit(q):
    length = math.sqrt(sum(v * v for v in q))
    if length < 1e-12:
        raise ValueError("zero-length quaternion")
    return tuple(v / length for v in q)


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _qinv(q):
    x, y, z, w = _unit(q)
    return (-x, -y, -z, w)


def _rotate(q, v):
    x, y, z, _ = _qmul(_qmul(q, (v[0], v[1], v[2], 0.0)), _qinv(q))
    return (x, y, z)


def relative_pose(reference, hand):
    """Return reference->hand pose from two camera->tag poses."""
    rp, rq = reference
    hp, hq = hand
    qi = _qinv(rq)
    delta = tuple(hp[i] - rp[i] for i in range(3))
    return _rotate(qi, delta), _unit(_qmul(qi, hq))


def _mean_quaternion(quaternions):
    # All quaternions in one settled sample/pose are close. Hemisphere-align
    # before averaging so q and -q cannot cancel each other.
    anchor = _unit(quaternions[0])
    aligned = [tuple(-v for v in q) if sum(q[i] * anchor[i] for i in range(4)) < 0
               else q for q in map(_unit, quaternions)]
    return _unit(tuple(sum(q[i] for q in aligned) for i in range(4)))


def _angle_deg(a, b):
    dot = min(1.0, abs(sum(x * y for x, y in zip(_unit(a), _unit(b)))))
    return math.degrees(2.0 * math.acos(dot))


def _percentile(values, pct):
    ordered = sorted(values)
    pos = (len(ordered) - 1) * pct / 100.0
    low = math.floor(pos)
    high = math.ceil(pos)
    return ordered[low] + (ordered[high] - ordered[low]) * (pos - low)


def _pose_center(poses):
    positions = [p for p, _ in poses]
    return (tuple(statistics.median(p[i] for p in positions) for i in range(3)),
            _mean_quaternion([q for _, q in poses]))


def _distance_mm(a, b):
    return math.dist(a, b) * 1000.0


def aggregate_frames(poses, minimum=15):
    """Robustly combine distinct, settled frames into one arrival sample."""
    if len(poses) < minimum:
        raise ValueError("insufficient distinct settled frames")
    center = _pose_center(poses)
    distances = [_distance_mm(p, center[0]) for p, _ in poses]
    median_distance = statistics.median(distances)
    mad = statistics.median(abs(d - median_distance) for d in distances)
    cutoff_mm = max(3.0, median_distance + 4.0 * 1.4826 * mad)
    kept = [pose for pose, d in zip(poses, distances)
            if d <= cutoff_mm and _angle_deg(pose[1], center[1]) <= 3.0]
    if len(kept) < minimum:
        raise ValueError("too many visual outliers")
    return _pose_center(kept), len(kept), len(poses) - len(kept)


def repeatability_report(samples):
    if len(samples) < 2:
        raise ValueError("at least two arrival samples are required")
    center = _pose_center(samples)
    distances = [_distance_mm(p, center[0]) for p, _ in samples]
    angles = [_angle_deg(q, center[1]) for _, q in samples]
    axes = [[(p[i] - center[0][i]) * 1000.0 for p, _ in samples] for i in range(3)]
    return {
        "center": {"position_m": list(center[0]), "quaternion_xyzw": list(center[1])},
        "translation": {
            "rms_mm": math.sqrt(statistics.mean(d * d for d in distances)),
            "p95_mm": _percentile(distances, 95), "max_mm": max(distances),
            "std_x_mm": statistics.stdev(axes[0]),
            "std_y_mm": statistics.stdev(axes[1]),
            "std_z_mm": statistics.stdev(axes[2]),
        },
        "rotation": {
            "rms_deg": math.sqrt(statistics.mean(a * a for a in angles)),
            "p95_deg": _percentile(angles, 95), "max_deg": max(angles),
        },
    }


def _stamp_ns(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _transform_pose(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    position = (float(t.x), float(t.y), float(t.z))
    quaternion = tuple(float(v) for v in (q.x, q.y, q.z, q.w))
    if not all(math.isfinite(v) for v in position + quaternion):
        raise ValueError("non-finite AprilTag transform")
    return position, _unit(quaternion)


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        del namespace
        self._cfg = plugin_config
        self._client = client
        self._lock = threading.Condition(threading.RLock())
        self._node = None
        self._executor = executor
        self._tf_buffer = None
        self._tf_listener = None
        self._session = None
        self._capture = None
        self._pending = deque(maxlen=32)
        self._latest_detection = None
        self._latest_camera_info = None
        self._latest_tags = {}
        self._stable_since = None
        self._departure_seen = False
        self._latest_joint_positions = None
        self._previous_sample_joint_positions = None
        self._processed_stamps = deque(maxlen=128)
        self._static_poses = deque(maxlen=1200)
        self._reference_id = int(plugin_config.get("reference_tag_id", 0))
        self._hand_id = int(plugin_config.get("hand_tag_id", 1))
        self._reference_frame = str(plugin_config.get("reference_frame", "q5_reference_tag"))
        self._hand_frame = str(plugin_config.get("hand_frame", "q5_left_hand_tag"))
        self._joint_velocity_limit = float(plugin_config.get("joint_velocity_limit_rad_s", 0.02))
        self._joint_age_limit_ms = int(plugin_config.get("joint_age_limit_ms", 300))
        self._capture_timeout_s = float(plugin_config.get("capture_timeout_s", 8.0))
        self._min_frames = int(plugin_config.get("min_frames", 15))
        self._target_frames = int(plugin_config.get("target_frames", 25))
        self._departure_rad = float(plugin_config.get("departure_delta_rad", 0.03))
        self._return_rad = float(plugin_config.get("return_tolerance_rad", 0.05))
        self._tag_size_confirmed = bool(plugin_config.get("tag_size_confirmed", False))
        if self._reference_id == self._hand_id:
            raise ValueError("reference_tag_id and hand_tag_id must differ")
        if not (2 <= self._min_frames <= self._target_frames <= 100):
            raise ValueError("require 2 <= min_frames <= target_frames <= 100")
        if _HAS_ROS2 and executor is not None:
            self._node = Node("q5_visual_ee_repeatability")
            qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
            self._node.create_subscription(
                AprilTagDetectionArray,
                str(plugin_config.get("detections_topic", "/q5/apriltag/detections")),
                self._on_detections, qos)
            self._node.create_subscription(
                CameraInfo,
                str(plugin_config.get("camera_info_topic", "/camera/camera/color/camera_info")),
                self._on_camera_info, qos)
            self._tf_buffer = Buffer(cache_time=None)
            self._tf_listener = TransformListener(self._tf_buffer, self._node, spin_thread=False)
            self._node.create_timer(0.03, self._process_pending)
            executor.add_node(self._node)

    def get_tool(self):
        return {
            "name": CARD, "type": "sensor", "multiInstance": False,
            "description": "Q5 左手相对躯干 AprilTag 视觉重复定位测量；只读，不控制机器人",
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["info", "start_session", "capture_sample", "report", "reset"]},
                    "side": {"type": "string", "enum": ["left"], "default": "left"},
                    "expected_samples": {"type": "integer", "minimum": 2, "maximum": 100, "default": 10},
                },
                "required": ["action"], "additionalProperties": False,
                "x-action-params": {
                    "info": {"params": []},
                    "start_session": {"params": ["side", "expected_samples"]},
                    "capture_sample": {"params": []},
                    "report": {"params": []},
                    "reset": {"params": []},
                },
            },
        }

    def start(self):
        pass

    def stop(self):
        with self._lock:
            if self._capture is not None:
                self._capture["cancelled"] = True
            self._lock.notify_all()
        if self._node is not None:
            try:
                self._executor.remove_node(self._node)
                self._node.destroy_node()
            except Exception:
                pass

    def _on_camera_info(self, msg):
        with self._lock:
            self._latest_camera_info = (time.monotonic(), _stamp_ns(msg.header),
                                        msg.width, msg.height, list(msg.d))

    def _joint_state(self, stamp_ns):
        snap = self._client.snapshot()
        age = snap.get("age_ms")
        if not snap.get("fresh") or age is None or age > self._joint_age_limit_ms:
            return None, "joint_state_stale"
        joint_stamp_ms = snap.get("message_timestamp_ms")
        if joint_stamp_ms is not None and abs(joint_stamp_ms - stamp_ns / 1_000_000) > self._joint_age_limit_ms:
            return None, "joint_image_stamp_mismatch"
        positions = snap.get("joints", {})
        velocities = snap.get("velocities", {})
        if any(j not in positions or j not in velocities for j in LEFT_ARM_JOINTS):
            return None, "arm_position_or_velocity_missing"
        if any(not math.isfinite(positions[j]) or not math.isfinite(velocities[j]) for j in LEFT_ARM_JOINTS):
            return None, "arm_state_non_finite"
        joint_positions = {j: positions[j] for j in LEFT_ARM_JOINTS}
        if any(abs(velocities[j]) > self._joint_velocity_limit for j in LEFT_ARM_JOINTS):
            return joint_positions, "arm_moving"
        return joint_positions, None

    def _on_detections(self, msg):
        now = time.monotonic()
        stamp = _stamp_ns(msg.header)
        found = {int(d.id): d for d in msg.detections}
        with self._lock:
            self._latest_detection = (now, stamp, msg.header.frame_id)
            self._latest_tags = {i: {"hamming": int(d.hamming),
                                     "decision_margin": float(d.decision_margin)}
                                 for i, d in found.items()}
            if self._session is not None:
                q = self._session["quality"]
                q["detection_frames"] += 1
                q["reference_visible_frames"] += int(self._reference_id in found)
                q["hand_visible_frames"] += int(self._hand_id in found)
            if self._reference_id not in found or self._hand_id not in found:
                self._stable_since = None
                self._static_poses.clear()
                self._reject("tag_missing")
                return
            if stamp == 0 or not msg.header.frame_id:
                self._stable_since = None
                self._static_poses.clear()
                self._reject("invalid_image_header")
                return
            if stamp in self._processed_stamps or any(p[0] == stamp for p in self._pending):
                self._reject("duplicate_image_stamp")
                return
            for tag in (found[self._reference_id], found[self._hand_id]):
                if (tag.hamming != 0 or not math.isfinite(tag.decision_margin) or
                        tag.decision_margin < float(self._cfg.get("min_decision_margin", 30.0))):
                    self._stable_since = None
                    self._static_poses.clear()
                    self._reject("low_detection_quality")
                    return
            self._pending.append((stamp, msg.header.frame_id, now))

    def _reject(self, reason):
        if self._session is not None and self._capture is not None:
            self._session["quality"]["rejected_frames"] += 1
            self._session["rejection_reasons"][reason] += 1

    def _process_pending(self):
        if self._tf_buffer is None:
            return
        self._observe_departure()
        with self._lock:
            pending = list(self._pending)
        for stamp, camera_frame, received in pending:
            try:
                stamp_time = Time(nanoseconds=stamp)
                ref = self._tf_buffer.lookup_transform(camera_frame, self._reference_frame, stamp_time)
                hand = self._tf_buffer.lookup_transform(camera_frame, self._hand_frame, stamp_time)
                pose = relative_pose(_transform_pose(ref), _transform_pose(hand))
            except Exception:
                if time.monotonic() - received > 0.6:
                    with self._lock:
                        self._remove_pending(stamp)
                        self._stable_since = None
                        self._static_poses.clear()
                        self._reject("matching_tf_unavailable")
                continue
            now = time.monotonic()
            joints, motion_error = self._joint_state(stamp)
            with self._lock:
                if not self._remove_pending(stamp):
                    continue
                self._processed_stamps.append(stamp)
                if joints is not None:
                    self._latest_joint_positions = joints
                    if self._previous_sample_joint_positions is not None:
                        delta = max(abs(joints[j] - self._previous_sample_joint_positions[j])
                                    for j in LEFT_ARM_JOINTS)
                        if delta >= self._departure_rad:
                            self._departure_seen = True
                if motion_error:
                    self._stable_since = None
                    self._static_poses.clear()
                    self._reject(motion_error)
                    continue
                if self._stable_since is None:
                    self._stable_since = now
                self._static_poses.append((now, pose))
                if self._capture is None:
                    continue
                if now - self._stable_since < 0.5:
                    self._reject("not_stable_for_0_5_s")
                    continue
                if self._previous_sample_joint_positions is not None:
                    if not self._departure_seen:
                        self._reject("departure_not_observed")
                        continue
                    delta = max(abs(joints[j] - self._previous_sample_joint_positions[j])
                                for j in LEFT_ARM_JOINTS)
                    if delta > self._return_rad:
                        self._reject("not_at_measurement_posture")
                        continue
                self._capture["frames"].append((stamp, pose, joints))
                self._lock.notify_all()

    def _observe_departure(self):
        snap = self._client.snapshot()
        if not snap.get("fresh") or snap.get("age_ms", 100000) > self._joint_age_limit_ms:
            return
        joints = snap.get("joints", {})
        with self._lock:
            if self._previous_sample_joint_positions is None:
                return
            if all(j in joints and math.isfinite(joints[j]) for j in LEFT_ARM_JOINTS):
                delta = max(abs(joints[j] - self._previous_sample_joint_positions[j])
                            for j in LEFT_ARM_JOINTS)
                if delta >= self._departure_rad:
                    self._departure_seen = True

    def _remove_pending(self, stamp):
        for item in self._pending:
            if item[0] == stamp:
                self._pending.remove(item)
                return True
        return False

    def _info(self):
        now = time.monotonic()
        snap = self._client.snapshot()
        with self._lock:
            detection = self._latest_detection
            camera = self._latest_camera_info
            tags = dict(self._latest_tags)
            session = self._session
            static = list(self._static_poses)
            static_duration = static[-1][0] - static[0][0] if len(static) > 1 else 0.0
            static_noise = None
            if (static_duration >= 30.0 and len(static) >= 50 and
                    detection and now - detection[0] < 2.0):
                stats = repeatability_report([pose for _, pose in static])
                static_noise = {"duration_s": round(static_duration, 1),
                                "frame_count": len(static),
                                "translation_rms_mm": stats["translation"]["rms_mm"],
                                "translation_p95_mm": stats["translation"]["p95_mm"],
                                "rotation_rms_deg": stats["rotation"]["rms_deg"]}
            return {
                "ok": self._node is not None,
                "detector_online": bool(detection and now - detection[0] < 2.0),
                "camera_info_online": bool(camera and now - camera[0] < 2.0),
                "image_frame": detection[2] if detection else None,
                "reference_tag": {"id": self._reference_id, "visible": self._reference_id in tags and bool(detection and now - detection[0] < 2.0),
                                  **tags.get(self._reference_id, {})},
                "hand_tag": {"id": self._hand_id, "visible": self._hand_id in tags and bool(detection and now - detection[0] < 2.0),
                             **tags.get(self._hand_id, {})},
                "joint_state_age_ms": snap.get("age_ms"),
                "joint_state_fresh_for_measurement": bool(snap.get("fresh") and snap.get("age_ms") is not None and snap["age_ms"] <= self._joint_age_limit_ms),
                "tag_size_confirmed": self._tag_size_confirmed,
                "static_noise_baseline": static_noise,
                "static_stable_duration_s": round(static_duration, 1),
                "session_sample_count": len(session["samples"]) if session else 0,
                "capture_in_progress": self._capture is not None,
            }

    def _start_session(self, args):
        if args.get("side", "left") != "left":
            return {"ok": False, "code": "UNSUPPORTED_SIDE", "message": "First version supports left arm only"}
        expected = int(args.get("expected_samples", 10))
        if not 2 <= expected <= 100:
            return {"ok": False, "code": "INVALID_EXPECTED_SAMPLES"}
        if not self._tag_size_confirmed:
            return {"ok": False, "code": "TAG_SIZE_UNCONFIRMED",
                    "message": "Measure both printed black-square edges, update apriltag_q5.yaml, then set tag_size_confirmed: true"}
        info = self._info()
        if not (info["detector_online"] and info["camera_info_online"] and
                info["reference_tag"]["visible"] and info["hand_tag"]["visible"] and
                info["joint_state_fresh_for_measurement"]):
            return {"ok": False, "code": "INPUT_NOT_READY", "info": info}
        with self._lock:
            if self._capture is not None:
                return {"ok": False, "code": "CAPTURE_IN_PROGRESS"}
            self._session = {"side": "left", "expected_samples": expected,
                             "samples": [], "started_at_ms": int(time.time() * 1000),
                             "quality": Counter(), "rejection_reasons": Counter()}
            self._previous_sample_joint_positions = None
            self._departure_seen = False
            self._stable_since = None
            self._pending.clear()
        return {"ok": True, "side": "left", "expected_samples": expected}

    def _capture_sample(self):
        with self._lock:
            if self._session is None:
                return {"ok": False, "code": "NO_SESSION"}
            if len(self._session["samples"]) >= self._session["expected_samples"]:
                return {"ok": False, "code": "EXPECTED_SAMPLES_REACHED"}
            if self._capture is not None:
                return {"ok": False, "code": "CAPTURE_IN_PROGRESS"}
            info = self._info()
            if not (info["detector_online"] and info["camera_info_online"] and
                    info["reference_tag"]["visible"] and info["hand_tag"]["visible"] and
                    info["joint_state_fresh_for_measurement"]):
                return {"ok": False, "code": "INPUT_NOT_READY", "info": info}
            if self._previous_sample_joint_positions is not None and not self._departure_seen:
                return {"ok": False, "code": "DEPARTURE_NOT_OBSERVED",
                        "message": "Move from A to B and back before the next capture"}
            current = {"frames": [], "cancelled": False}
            self._capture = current
            deadline = time.monotonic() + self._capture_timeout_s
            while len(current["frames"]) < self._target_frames and not current["cancelled"]:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                self._lock.wait(left)
            if self._capture is current:
                self._capture = None
            if current["cancelled"]:
                return {"ok": False, "code": "CAPTURE_CANCELLED"}
            frames = current["frames"]
            if len(frames) < self._min_frames:
                return {"ok": False, "code": "INSUFFICIENT_VALID_FRAMES",
                        "collected_frames": len(frames),
                        "rejection_reasons": dict(self._session["rejection_reasons"])}
            try:
                pose, accepted, outliers = aggregate_frames(
                    [f[1] for f in frames], self._min_frames)
            except ValueError as exc:
                return {"ok": False, "code": "VISUAL_OUTLIERS", "message": str(exc)}
            self._session["samples"].append(pose)
            self._session["quality"]["accepted_frames"] += accepted
            self._session["quality"]["rejected_frames"] += outliers
            self._session["rejection_reasons"]["visual_outlier"] += outliers
            self._previous_sample_joint_positions = frames[-1][2]
            self._departure_seen = False
            return {"ok": True, "sample_index": len(self._session["samples"]),
                    "accepted_frames": accepted, "rejected_outliers": outliers,
                    "position_m": list(pose[0]), "quaternion_xyzw": list(pose[1])}

    def _report(self):
        with self._lock:
            if self._session is None:
                return {"ok": False, "code": "NO_SESSION"}
            samples = list(self._session["samples"])
            quality = dict(self._session["quality"])
            reasons = dict(self._session["rejection_reasons"])
            expected = self._session["expected_samples"]
        if len(samples) < 2:
            return {"ok": False, "code": "TOO_FEW_SAMPLES", "sample_count": len(samples)}
        frames = quality.get("detection_frames", 0)
        quality.update({
            "reference_tag_visible_ratio": quality.get("reference_visible_frames", 0) / frames if frames else 0.0,
            "hand_tag_visible_ratio": quality.get("hand_visible_frames", 0) / frames if frames else 0.0,
            "rejection_reasons": reasons,
        })
        return {"ok": True, "side": "left", "sample_count": len(samples),
                "expected_samples": expected, "complete": len(samples) >= expected,
                "measurement": "hand_tag_relative_to_torso_reference_tag",
                "note": "Engineering repeatability statistics; not absolute TCP accuracy or ISO 9283 certification",
                **repeatability_report(samples), "quality": quality}

    def dispatch(self, action, args):
        if action == "info":
            return self._info()
        if action == "start_session":
            return self._start_session(args)
        if action == "capture_sample":
            return self._capture_sample()
        if action == "report":
            return self._report()
        if action == "reset":
            with self._lock:
                if self._capture is not None:
                    self._capture["cancelled"] = True
                    self._capture = None
                    self._lock.notify_all()
                self._session = None
                self._stable_since = None
                self._previous_sample_joint_positions = None
                self._departure_seen = False
                self._pending.clear()
                self._static_poses.clear()
            return {"ok": True, "state": "idle"}
        return {"ok": False, "code": "UNKNOWN_ACTION"}


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
