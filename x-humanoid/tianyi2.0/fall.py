#!/usr/bin/env python3
"""Tianyi 2.0 Pro fall-detection card.

Fuses two sources that already run on the robot instead of loading a second
detector:

  * ``vop`` (perception container, core domain) says *which pixels* hold a person.
  * The Orbbec head depth image + accelerometer (tianyi domain) say *how high*
    those pixels are above the floor.

Because the decision is made on metres above the floor, it is
distance-invariant: no per-site pixel thresholds, and no need to pitch the head
down to keep a fallen person inside the frame.  It also separates "lying on the
floor" from "sitting on the floor", which a bbox aspect ratio cannot do — a
seated person's head still reaches ~0.9 m.

Geometry.  ``PointCloudPlugin`` levels its points with a Rodrigues rotation that
takes the measured up-vector to world +Y and then adds a floor offset.  Only the
vertical component is needed here, and for that rotation the component collapses
to a dot product::

    height_above_camera = dot(normalised_accel_optical, point_optical)
    height_above_floor  = height_above_camera + camera_height_m

Optical axes are (right, down, forward); an accelerometer at rest reads the
specific force pointing *up*, so the normalised reading is the up-vector in the
camera's own frame.  When no IMU sample has arrived the up-vector falls back to
``(0, -cos(pitch), -sin(pitch))`` for a configured pitch-down angle.

Output topic carries ``std_msgs/String`` JSON in the core domain, so Agent Core
turns it into a ``dds:`` source and a skill can read it with ``raw_input_info``.
"""

import json
import math
import threading
import time

import numpy as np
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

from device import _RELIABLE_QOS

# 640x480 Z16 is 614 KiB per frame and only the newest one matters — keep the
# ingress queue at one, the way DepthCameraPlugin does.
_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class FallPlugin:
    """Person height tracking over the head depth camera."""

    _DEPTH_TOPIC = "/ob_camera_head/depth/image_raw"
    _INFO_TOPIC = "/ob_camera_head/depth/camera_info"
    _ACCEL_TOPIC = "/ob_camera_head/accel/sample"

    # Cap the per-frame point budget; the ROI is strided down to hit it.
    _MAX_ROI_POINTS = 3000
    _MIN_ROI_POINTS = 30
    # Depth values outside this band around the ROI median are background.
    _DEPTH_BAND_M = 0.60
    _VOP_STALE_SEC = 3.0
    _DEPTH_STALE_SEC = 3.0

    def __init__(self, plugin_config: dict, namespace: str, ros2):
        cfg = plugin_config or {}
        self._ns = namespace
        self._topic = f"/{namespace}/camera/head/fall"
        self._vop_topic = str(
            cfg.get("vop_topic", f"/{namespace}/camera/head/objects"))

        # ── tunables ─────────────────────────────────────────────────────────
        # camera_height_m mirrors PointCloudPlugin's floor_offset_m default.
        self._camera_height_m = float(cfg.get("camera_height_m", 1.50))
        self._camera_pitch_deg = float(cfg.get("camera_pitch_deg", 0.0))
        self._fall_top_m = float(cfg.get("fall_top_m", 0.70))
        self._recover_top_m = float(cfg.get("recover_top_m", 1.00))
        self._min_duration_sec = float(cfg.get("min_duration_sec", 5.0))
        self._lost_grace_sec = float(cfg.get("lost_grace_sec", 5.0))
        self._min_confidence = float(cfg.get("min_confidence", 0.35))
        self._roi_shrink = float(cfg.get("roi_shrink", 0.15))
        self._eval_hz = max(0.5, min(float(cfg.get("eval_hz", 2.0)), 10.0))
        # Every String on a core-domain topic becomes an Agent Core event, and
        # every event can wake the LLM.  So publish on state *changes* only, plus
        # a slow heartbeat that proves the card is still alive.  A skill that
        # wants the current numbers between changes calls fall_detect info.
        self._publish_interval = float(cfg.get("publish_interval_sec", 30.0))

        # ── runtime state ────────────────────────────────────────────────────
        self._running = False
        self._lock = threading.Lock()
        self._latest_depth = None          # (stamp, ndarray uint16)
        self._persons = ([], 0.0)          # (boxes, stamp)
        self._intrinsics = None            # (fx, fy, cx, cy)
        self._gravity = None               # normalised accel, optical frame
        self._low_since = None
        self._fallen = False
        self._last_state = None
        self._last_publish = 0.0
        self._last_person_seen = 0.0
        self._eval_timer = None

        self._sub_node = Node("tianyi2_fall_sub", context=ros2.ctx_tianyi)
        self._core_node = Node("tianyi2_fall_core", context=ros2.ctx_core)
        ros2.executor_tianyi.add_node(self._sub_node)
        ros2.executor_core.add_node(self._core_node)
        self._pub = None

    # ── MCP tool ─────────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "fall_detect",
            "type": "processor",
            "description": (
                "天轶2.0 跌倒检测 — 融合 vop 的 person 检测框与头部深度相机，"
                "按离地高度(米)判断有人倒地且长时间未起身。结果以 JSON 发布，"
                "可用 raw_input_info 查询。需要 vop 卡片同时运行。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info", "reset"],
                        "description": "start=开始监测, stop=停止, info=状态, reset=清除当前跌倒状态",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": [], "description": "开始跌倒监测"},
                    "stop": {"params": [], "description": "停止跌倒监测"},
                    "info": {"params": [], "description": "查询运行状态与最近一次判定"},
                    "reset": {"params": [], "description": "清除已确认的跌倒状态，重新开始计时"},
                },
            },
            "configSchema": {
                "type": "object",
                "properties": {
                    "camera_height_m": {
                        "type": "number", "default": 1.50,
                        "description": "头部深度相机离地高度(米)，需现场标定",
                    },
                    "camera_pitch_deg": {
                        "type": "number", "default": 0.0,
                        "description": "相机俯角(度，向下为正)。仅在 IMU 无数据时作为兜底",
                    },
                    "fall_top_m": {
                        "type": "number", "default": 0.70,
                        "description": "人体最高点低于此高度(米)判为低位",
                    },
                    "recover_top_m": {
                        "type": "number", "default": 1.00,
                        "description": "人体最高点高于此高度(米)判为已起身",
                    },
                    "min_duration_sec": {
                        "type": "number", "default": 5.0,
                        "description": "持续低位超过该秒数才确认跌倒",
                    },
                    "min_confidence": {
                        "type": "number", "default": 0.35,
                        "description": "vop person 检测置信度下限",
                    },
                    "eval_hz": {
                        "type": "number", "default": 2.0,
                        "description": "判定频率(Hz)",
                    },
                },
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        from sensor_msgs.msg import Image, CameraInfo, Imu
        from std_msgs.msg import String

        self._pub = self._core_node.create_publisher(
            String, self._topic, _RELIABLE_QOS)
        self._core_node.create_subscription(
            String, self._vop_topic, self._on_vop, _LATEST_QOS)
        self._sub_node.create_subscription(
            Image, self._DEPTH_TOPIC, self._on_depth, _LATEST_QOS)
        self._sub_node.create_subscription(
            CameraInfo, self._INFO_TOPIC, self._on_info, _RELIABLE_QOS)
        self._sub_node.create_subscription(
            Imu, self._ACCEL_TOPIC, self._on_accel, _LATEST_QOS)
        print(f"[FallPlugin] vop={self._vop_topic} depth={self._DEPTH_TOPIC} "
              f"out={self._topic}")

    def stop(self):
        self._running = False
        self._cancel_timer()

    def _cancel_timer(self):
        if self._eval_timer is not None:
            try:
                self._eval_timer.cancel()
            except Exception:
                pass
            self._eval_timer = None

    # ── subscriptions ────────────────────────────────────────────────────────

    def _on_accel(self, msg):
        a = (msg.linear_acceleration.x,
             msg.linear_acceleration.y,
             msg.linear_acceleration.z)
        magnitude = math.sqrt(sum(v * v for v in a))
        if not 8.0 <= magnitude <= 11.5:
            return  # reject dynamic acceleration, same gate as PointCloudPlugin
        a = tuple(v / magnitude for v in a)
        with self._lock:
            previous = self._gravity
            if previous is None:
                self._gravity = a
            else:
                mixed = tuple(0.95 * o + 0.05 * n for o, n in zip(previous, a))
                norm = math.sqrt(sum(v * v for v in mixed))
                self._gravity = tuple(v / norm for v in mixed)

    def _on_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self._intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5],
                                int(msg.width), int(msg.height))

    def _on_depth(self, msg):
        if not self._running or msg.encoding not in ("16UC1", "mono16"):
            return
        if msg.is_bigendian:
            return
        width, height, step = int(msg.width), int(msg.height), int(msg.step)
        if width <= 0 or height <= 0 or step < width * 2:
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        needed = height * step
        if raw.size < needed:
            return
        depth = (raw[:needed].reshape(height, step)[:, :width * 2]
                 .view(np.uint16).reshape(height, width))
        with self._lock:
            self._latest_depth = (time.time(), depth)

    def _on_vop(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        boxes = []
        for obj in data.get("objects", []) or []:
            if obj.get("name") != "person":
                continue
            if float(obj.get("confidence", 0.0)) < self._min_confidence:
                continue
            box = self._normalised_box(obj)
            if box:
                boxes.append(box)
        now = time.time()
        with self._lock:
            self._persons = (boxes, now)
            if boxes:
                self._last_person_seen = now

    def _normalised_box(self, obj):
        """Return (x1, y1, x2, y2) in 0..1 image coordinates."""
        bbox = obj.get("bbox")
        if bbox and len(bbox) == 4:
            x1, y1, x2, y2 = (float(v) for v in bbox)
            if x2 > x1 and y2 > y1:
                return (x1, y1, x2, y2)
        # Fallback: vop currently publishes only a centre point, normalised to
        # -1..1 relative to the image centre.  A fixed-size window around it is
        # a much weaker ROI — add `bbox` to vop's _extract_objects for real use.
        pos = obj.get("position")
        if not pos or len(pos) < 2:
            return None
        cx = (float(pos[0]) + 1.0) / 2.0
        cy = (float(pos[1]) + 1.0) / 2.0
        return (cx - 0.06, cy - 0.14, cx + 0.06, cy + 0.14)

    # ── evaluation ───────────────────────────────────────────────────────────

    def _up_vector(self):
        """Up-vector in the optical frame, plus its provenance."""
        with self._lock:
            gravity = self._gravity
        if gravity is not None:
            return gravity, "imu"
        theta = math.radians(self._camera_pitch_deg)
        return (0.0, -math.cos(theta), -math.sin(theta)), "config"

    def _height_stats(self, box, depth, up):
        """Height percentiles (metres above floor) for one detection box."""
        if self._intrinsics is None:
            return None
        fx, fy, cx, cy, info_w, info_h = self._intrinsics
        height, width = depth.shape
        # CameraInfo may describe a different resolution than the frame that
        # arrived; scale the intrinsics rather than trusting them blindly.
        if info_w and info_h and (info_w != width or info_h != height):
            sx, sy = width / float(info_w), height / float(info_h)
            fx, cx = fx * sx, cx * sx
            fy, cy = fy * sy, cy * sy

        pad_x = (box[2] - box[0]) * self._roi_shrink / 2.0
        pad_y = (box[3] - box[1]) * self._roi_shrink / 2.0
        u0 = int(max(0, min(width - 1, (box[0] + pad_x) * width)))
        u1 = int(max(1, min(width, (box[2] - pad_x) * width)))
        v0 = int(max(0, min(height - 1, (box[1] + pad_y) * height)))
        v1 = int(max(1, min(height, (box[3] - pad_y) * height)))
        if u1 <= u0 or v1 <= v0:
            return None

        roi = depth[v0:v1, u0:u1]
        stride = max(1, int(math.sqrt(roi.size / float(self._MAX_ROI_POINTS))))
        sub = roi[::stride, ::stride]
        z = sub.astype(np.float32) / 1000.0
        mask = z > 0.2
        if int(mask.sum()) < self._MIN_ROI_POINTS:
            return None

        rows, cols = np.nonzero(mask)
        z_valid = z[mask]
        median_z = float(np.median(z_valid))
        # Drop background: keep only the depth band around the person.
        keep = np.abs(z_valid - median_z) <= self._DEPTH_BAND_M
        if int(keep.sum()) < self._MIN_ROI_POINTS:
            return None
        zz = z_valid[keep]
        u_pix = u0 + cols[keep] * stride
        v_pix = v0 + rows[keep] * stride
        x = (u_pix - cx) * zz / fx
        y = (v_pix - cy) * zz / fy

        # height = dot(up, point) + camera height; see module docstring.
        heights = (up[0] * x + up[1] * y + up[2] * zz) + self._camera_height_m
        top, mid, bottom = np.percentile(heights, [90, 50, 10])
        return {
            "top_m": round(float(top), 3),
            "median_m": round(float(mid), 3),
            "bottom_m": round(float(bottom), 3),
            "distance_m": round(float(np.median(zz)), 3),
            "n_points": int(zz.size),
        }

    def _evaluate(self):
        if not self._running:
            return
        now = time.time()
        with self._lock:
            boxes, vop_stamp = self._persons
            depth_entry = self._latest_depth
            last_seen = self._last_person_seen

        if not vop_stamp or now - vop_stamp > self._VOP_STALE_SEC:
            return self._publish(now, "no_data", None,
                                 reason="vop 无数据或已停止")
        if depth_entry is None or now - depth_entry[0] > self._DEPTH_STALE_SEC:
            return self._publish(now, "no_data", None,
                                 reason="深度相机无数据")
        if self._intrinsics is None:
            return self._publish(now, "no_data", None,
                                 reason="尚未收到 CameraInfo 内参")

        up, source = self._up_vector()
        if not boxes:
            # Person gone.  If they were already low, that is suspicious rather
            # than safe: they may have dropped out of the depth frustum.
            if self._low_since and now - last_seen <= self._lost_grace_sec:
                return self._publish(now, "person_lost", None,
                                     gravity=source,
                                     reason="人员消失但此前处于低位，建议复核")
            self._low_since = None
            return self._publish(now, "no_person", None, gravity=source)

        stats = [s for s in (self._height_stats(b, depth_entry[1], up)
                             for b in boxes) if s]
        if not stats:
            return self._publish(now, "no_depth", None, gravity=source,
                                 reason="检测框内没有有效深度点")
        # The lowest person is the one worth reporting.
        best = min(stats, key=lambda s: s["top_m"])
        best["person_count"] = len(stats)

        top = best["top_m"]
        if top >= self._recover_top_m:
            self._low_since = None
            self._fallen = False
            return self._publish(now, "standing", best, gravity=source)
        if top > self._fall_top_m:
            # Between the two thresholds: deliberately undecided.
            return self._publish(now, "uncertain", best, gravity=source)

        if self._low_since is None:
            self._low_since = now
        duration = now - self._low_since
        if duration >= self._min_duration_sec:
            self._fallen = True
            return self._publish(now, "fallen", best, gravity=source,
                                 duration=duration)
        return self._publish(now, "low", best, gravity=source,
                             duration=duration)

    def _publish(self, now, state, stats, gravity=None, duration=0.0,
                 reason=None):
        payload = {
            "timestamp": round(now, 3),
            "fallen": state == "fallen",
            "state": state,
            "low_duration_sec": round(duration, 1),
        }
        if stats:
            payload.update(stats)
        if gravity:
            payload["up_source"] = gravity
        if reason:
            payload["reason"] = reason

        # info must always answer with the freshest evaluation, published or not.
        self._last_payload = payload

        changed = state != self._last_state
        due = now - self._last_publish >= self._publish_interval
        if not (changed or due):
            return
        self._last_state = state
        self._last_publish = now
        if self._pub is not None:
            from std_msgs.msg import String
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._pub.publish(msg)
        self._last_payload = payload

    # ── dispatch ─────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            if not self._running:
                self._running = True
                self._low_since = None
                self._fallen = False
                self._last_state = None
                self._cancel_timer()
                self._eval_timer = self._core_node.create_timer(
                    1.0 / self._eval_hz, self._evaluate)
            return {
                "state": "running",
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
                "vop_topic": self._vop_topic,
                "hint": (f"用 raw_input_info(source=\"dds:{self._topic}\") 读取判定结果；"
                         "需要 vop 卡片同时运行并检测 person"),
            }
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        if action == "reset":
            self._low_since = None
            self._fallen = False
            self._last_state = None
            return {"state": "running" if self._running else "idle",
                    "reset": True}
        # info
        return {
            "state": "running" if self._running else "idle",
            "fallen": self._fallen,
            "last": getattr(self, "_last_payload", None),
            "intrinsics_ready": self._intrinsics is not None,
            "up_source": self._up_vector()[1],
            "camera_height_m": self._camera_height_m,
            "fall_top_m": self._fall_top_m,
            "recover_top_m": self._recover_top_m,
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }
