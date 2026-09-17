#!/usr/bin/env python3
"""Tianyi 2.0 Pro in-place person-following card.

Keeps a person centred in the head camera's field of view by rotating the
chassis on the spot.  It never translates — only yaw.

Two existing cards do the work; this one is the loop between them:

  * ``vop`` (perception container, core domain) publishes each detected
    object's centre, already normalised to -1..1 relative to the image centre.
  * ``chassis_raw`` turns a signed angle in degrees into a Slamtec
    ``RotateAction``, which closes the loop on the wheel encoders.

Why the servo loop lives in the card and not in a skill: ``vop`` publishes at
5 Hz, every ``String`` on a core-domain topic becomes an Agent Core event, and
an LLM round-trip is seconds.  A visual servo cannot converge at that latency,
so the LLM only gets ``start`` / ``stop`` / ``info`` and the tracking runs in a
background thread here.

Geometry.  ``position[0]`` is *not* an angle — it is the horizontal offset as a
fraction of the half-width.  Recovering the bearing needs the pinhole model::

    bearing_deg = degrees(atan(x_norm * tan(radians(hfov_deg) / 2)))

Treating ``x_norm * (hfov/2)`` as the angle overshoots by >15% near the frame
edge.  ``hfov_deg`` must be calibrated on the robot; the default is a placeholder.

Output topic carries ``std_msgs/String`` JSON in the core domain, published on
state *changes* plus a slow heartbeat, so following does not flood Agent Core
with events.  ``info`` always answers with the freshest numbers.
"""

import json
import math
import threading
import time

from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from std_msgs.msg import String

from device import _RELIABLE_QOS

# Only the newest detection frame matters — a queued backlog would steer the
# chassis towards where the person used to be.
_LATEST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)


class FollowPlugin:
    """Rotate-in-place tracking of a vop detection class."""

    # chassis_raw's fixed angular rate, used to estimate how long a rotation
    # will take (device.py ChassisRawPlugin._FIXED_W).
    _CHASSIS_W = 1.0          # rad/s
    _COOLDOWN_MARGIN = 0.3    # s, settle time added to every rotation estimate

    def __init__(self, plugin_config: dict, namespace: str, ros2, chassis=None):
        cfg = plugin_config or {}
        self._ns = namespace
        self._chassis = chassis
        self._topic = f"/{namespace}/follow/state"
        self._vop_topic = str(
            cfg.get("vop_topic", f"/{namespace}/camera/head/objects"))

        # ── tunables ─────────────────────────────────────────────────────────
        # Horizontal field of view of the head RGB camera. MUST be calibrated:
        # stand at the edge of the frame, measure the true bearing, solve back.
        self._hfov_deg = float(cfg.get("hfov_deg", 60.0))
        self._target_class = str(cfg.get("target_class", "person"))
        self._confidence = float(cfg.get("confidence", 0.35))
        # Inside the deadband the person counts as centred and nothing is sent.
        # Small on purpose: a real shift in where the person stands can show up
        # as a modest x_norm change, and offsets above this are never simply
        # dropped — if one persists it is corrected (see _plan_step).
        self._deadband = float(cfg.get("deadband", 0.12))
        # Smallest rotation worth sending. Below roughly a degree the Slamtec
        # RotateAction may not move the chassis at all, and each attempt still
        # costs a brake, so tiny commands buy jitter instead of tracking.
        self._min_step_deg = float(cfg.get("min_step_deg", 1.5))
        self._max_step_deg = float(cfg.get("max_step_deg", 45.0))
        # How long an offset too small for min_step_deg must persist before it
        # is corrected anyway. This is what stops a slowly drifting person from
        # being reported as centred forever.
        self._small_hold_sec = float(cfg.get("small_hold_sec", 2.0))
        self._dwell_release_sec = self._small_hold_sec
        self._gain = float(cfg.get("gain", 0.8))
        self._lost_timeout_sec = float(cfg.get("lost_timeout_sec", 1.0))
        self._stale_sec = float(cfg.get("stale_sec", 1.0))
        self._eval_hz = max(0.5, min(float(cfg.get("eval_hz", 2.0)), 10.0))
        self._publish_interval = float(cfg.get("publish_interval_sec", 30.0))

        # ── runtime state ────────────────────────────────────────────────────
        self._running = False
        self._lock = threading.Lock()
        self._detections = ([], 0.0)   # ([(x_norm, confidence)], stamp)
        self._pending_deg = 0.0        # unsent correction, see _accumulate
        self._last_target_x = None
        self._last_seen = 0.0
        self._cooldown_until = 0.0
        self._braked = False           # brake is idempotent; only send it once
        self._last_state = None
        self._last_publish = 0.0
        self._last_payload = None
        self._thread = None
        self._turn_count = 0

        self._core_node = Node("tianyi2_follow_core", context=ros2.ctx_core)
        ros2.executor_core.add_node(self._core_node)
        self._pub = self._core_node.create_publisher(
            String, self._topic, _RELIABLE_QOS)
        self._subscribed = False

    # ── MCP tool ─────────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "follow",
            "type": "processor",
            "description": (
                "天轶2.0 原地跟随 — 读取 vop 的人体中心坐标，调用 chassis_raw 原地转向，"
                "把人保持在画面中央。只转不走。需要 vop 卡片同时运行并检测 person。"
                "hfov_deg 需现场标定。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info", "config"],
                        "description": "start=开始跟随, stop=停止并刹停, info=状态与实时数值, config=改参数",
                    },
                    "target_class": {
                        "type": "string",
                        "description": "要跟随的 vop 类别，默认 person",
                    },
                    "deadband": {
                        "type": "number",
                        "default": 0.12,
                        "minimum": 0.0, "maximum": 0.49,
                        "description": "死区(0-1)。|归一化横向偏移| 小于此值视为已居中，不转向",
                    },
                    "max_step_deg": {
                        "type": "number",
                        "description": "单次转向角度上限(度)。首次上机建议压到 15",
                    },
                    "hfov_deg": {
                        "type": "number",
                        "default": 60.0,
                        "minimum": 1.0, "maximum": 179.0,
                        "description": "头部相机水平视场角(度)，需现场标定",
                    },
                },
                "required": ["action"],
                # No x-completion: following is a continuous behaviour with no
                # "done" state for the ACP barrier to wait on.
                "x-action-params": {
                    "start": {"params": [], "description": "开始原地跟随"},
                    "stop": {"params": [], "description": "停止跟随并刹停底盘"},
                    "info": {"params": [], "description": "查询运行状态与最近一次判定"},
                    "config": {
                        "params": ["target_class", "deadband", "max_step_deg", "hfov_deg"],
                        "description": "运行中改参数，立即生效",
                    },
                },
            },
            "configSchema": {
                "type": "object",
                "properties": {
                    "hfov_deg": {
                        "type": "number", "default": 60.0,
                        "minimum": 1.0, "maximum": 179.0,
                        "description": "头部相机水平视场角(度)，需现场标定",
                    },
                    "target_class": {
                        "type": "string", "default": "person",
                        "description": "跟随的 vop 类别",
                    },
                    "confidence": {
                        "type": "number", "default": 0.35,
                        "description": "vop 检测置信度下限",
                    },
                    "deadband": {
                        "type": "number", "default": 0.12,
                        "minimum": 0.0, "maximum": 0.49,
                        "description": "死区(0-1)，小于此偏移不转向",
                    },
                    "gain": {
                        "type": "number", "default": 0.8,
                        "description": "转向增益。1.0=一次转到位，小于 1 更稳但收敛慢",
                    },
                    "min_step_deg": {
                        "type": "number", "default": 3.0,
                        "description": "小于此角度不下发，避免抖动",
                    },
                    "max_step_deg": {
                        "type": "number", "default": 45.0,
                        "description": "单次转向上限(度)",
                    },
                    "lost_timeout_sec": {
                        "type": "number", "default": 1.0,
                        "description": "目标消失超过该秒数则刹停",
                    },
                    "eval_hz": {
                        "type": "number", "default": 2.0,
                        "description": "控制频率(Hz)",
                    },
                },
            },
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        # Subscribe once; repeated MCP start actions must not stack callbacks.
        if not self._subscribed:
            self._core_node.create_subscription(
                String, self._vop_topic, self._on_vop, _LATEST_QOS)
            self._subscribed = True
            print(f"[FollowPlugin] vop={self._vop_topic} out={self._topic}")

    def stop(self):
        self._running = False
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._brake("stopped")

    # ── subscription ─────────────────────────────────────────────────────────

    def _on_vop(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        found = []
        for obj in data.get("objects", []) or []:
            if obj.get("name") != self._target_class:
                continue
            try:
                confidence = float(obj.get("confidence", 0.0))
            except (TypeError, ValueError):
                continue
            if confidence < self._confidence:
                continue
            position = obj.get("position")
            if not position or len(position) < 2:
                continue
            try:
                x_norm = float(position[0])
            except (TypeError, ValueError):
                continue
            if not math.isfinite(x_norm):
                continue
            found.append((x_norm, confidence))
        now = time.time()
        with self._lock:
            self._detections = (found, now)
            if found:
                self._last_seen = now

    # ── geometry ─────────────────────────────────────────────────────────────

    def bearing_deg(self, x_norm: float) -> float:
        """Bearing of a normalised horizontal offset, through the pinhole model.

        ``x_norm`` is the offset as a fraction of the half-width, so it scales
        with ``tan``, not linearly with the angle.  Positive = target is to the
        right of centre.
        """
        half_fov = math.radians(self._hfov_deg) / 2.0
        return math.degrees(math.atan(x_norm * math.tan(half_fov)))

    def select_target(self, detections):
        """Pick which detection to chase.

        Highest confidence wins.  Ties break towards the previous target's
        x — cheap frame-to-frame stickiness so two people standing side by side
        do not make the chassis swing left and right forever.
        """
        if not detections:
            return None
        best_confidence = max(c for _, c in detections)
        candidates = [d for d in detections if d[1] >= best_confidence - 1e-9]
        if len(candidates) == 1 or self._last_target_x is None:
            return candidates[0]
        return min(candidates, key=lambda d: abs(d[0] - self._last_target_x))

    def clamp_step(self, bearing: float, dwell_sec: float = 0.0):
        """Turn a bearing into a rotation command, or None to send nothing.

        Two gates, for two different failure modes:

        * Steps below ``min_step_deg`` are not worth a REST round-trip and a
          ``RotateAction`` that small may fall under the encoder's resolution.
          But silently dropping them forever is exactly the "person moved and
          the robot still thinks it is aimed" bug — a small offset that persists
          is real.  So a sub-minimum step is released once it has persisted for
          ``dwell_release_sec``, floored to ``min_step_deg`` so the chassis
          actually moves.
        * Steps above ``max_step_deg`` are clamped.  The remaining error is not
          lost: the next tick sees it again in the fresh ``x_norm``.

        Note the correction is always recomputed from the *current* bearing.
        Accumulating the angle across ticks would be integral windup — bearing
        is an absolute error, not an increment — and would overshoot.
        """
        step = bearing * self._gain
        magnitude = abs(step)
        if magnitude < self._min_step_deg:
            if dwell_sec < self._dwell_release_sec:
                return None
            return math.copysign(self._min_step_deg, step)
        if magnitude > self._max_step_deg:
            step = math.copysign(self._max_step_deg, step)
        return step

    # ── chassis ──────────────────────────────────────────────────────────────

    def _brake(self, state_hint: str = ""):
        """Stop the chassis.  Idempotent — only reaches the chassis once."""
        if self._braked or self._chassis is None:
            return
        self._braked = True
        self._cooldown_until = 0.0
        try:
            self._chassis.dispatch("brake", {})
        except Exception as e:
            print(f"[FollowPlugin] brake failed ({state_hint}): {e}")

    def _rotate(self, step_deg: float) -> dict:
        """Send one closed-loop rotation and start the cooldown.

        chassis_raw's rotate branch returns as soon as the REST call is made and
        — unlike its move branch — does not cancel the previous action, so the
        previous rotation is cancelled here first.  Without that, rotations
        queue up inside Slamtec and the chassis keeps turning after the person
        has already been centred.
        """
        if self._chassis is None:
            return {"error": "chassis_raw card not available"}
        self._brake("pre-rotate")
        self._braked = False
        rotation = "right" if step_deg > 0 else "left"
        result = self._chassis.dispatch(
            "rotate", {"rotation": rotation, "angle": abs(step_deg)})
        # Estimate when the rotation finishes: no x-completion callback exists,
        # so this is the only thing keeping one rotation in flight at a time.
        self._cooldown_until = (
            time.time()
            + math.radians(abs(step_deg)) / self._CHASSIS_W
            + self._COOLDOWN_MARGIN
        )
        self._turn_count += 1
        return result

    # ── control loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        interval = 1.0 / self._eval_hz
        while self._running:
            try:
                self._evaluate()
            except Exception as e:
                print(f"[FollowPlugin] evaluate error: {e}")
            time.sleep(interval)

    def _evaluate(self):
        now = time.time()
        with self._lock:
            detections, stamp = self._detections
            last_seen = self._last_seen

        if not stamp or now - stamp > self._stale_sec:
            self._brake("no_data")
            return self._publish(now, "no_data", reason="vop 无数据或已停止")

        if not detections:
            # Only brake once the target has been gone long enough — a single
            # dropped frame at 5 fps should not stop an in-progress rotation.
            if not last_seen or now - last_seen > self._lost_timeout_sec:
                self._brake("lost")
                return self._publish(now, "lost",
                                     reason=f"{self._target_class} 已丢失，已刹停")
            return self._publish(now, "searching",
                                 reason="目标暂时未检出，等待重新出现")

        target = self.select_target(detections)
        x_norm, confidence = target
        self._last_target_x = x_norm
        bearing = self.bearing_deg(x_norm)

        if abs(x_norm) <= self._deadband:
            self._brake("centered")
            return self._publish(now, "centered", x_norm=x_norm,
                                 bearing_deg=bearing, confidence=confidence)

        if now < self._cooldown_until:
            return self._publish(now, "turning", x_norm=x_norm,
                                 bearing_deg=bearing, confidence=confidence,
                                 reason="上一次转向仍在进行")

        step = self.clamp_step(bearing)
        if step is None:
            return self._publish(now, "centered", x_norm=x_norm,
                                 bearing_deg=bearing, confidence=confidence,
                                 reason="偏移小于最小步长")

        result = self._rotate(step)
        if isinstance(result, dict) and "error" in result:
            return self._publish(now, "error", x_norm=x_norm,
                                 bearing_deg=bearing, confidence=confidence,
                                 step_deg=step, reason=str(result["error"]))
        return self._publish(now, "turning", x_norm=x_norm,
                             bearing_deg=bearing, confidence=confidence,
                             step_deg=step)

    # ── publishing ───────────────────────────────────────────────────────────

    def _publish(self, now, state, x_norm=None, bearing_deg=None,
                 confidence=None, step_deg=None, reason=None):
        payload = {
            "timestamp": round(now, 3),
            "state": state,
            "following": state in ("turning", "centered"),
            "target_class": self._target_class,
        }
        if x_norm is not None:
            payload["x_norm"] = round(x_norm, 3)
        if bearing_deg is not None:
            payload["bearing_deg"] = round(bearing_deg, 2)
        if confidence is not None:
            payload["confidence"] = round(confidence, 2)
        if step_deg is not None:
            payload["step_deg"] = round(step_deg, 2)
            payload["rotation"] = "right" if step_deg > 0 else "left"
        if reason:
            payload["reason"] = reason

        # info must answer with the freshest evaluation whether it was
        # published or not.
        self._last_payload = payload

        changed = state != self._last_state
        due = now - self._last_publish >= self._publish_interval
        if not (changed or due):
            return
        self._last_state = state
        self._last_publish = now
        if self._pub is not None:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._pub.publish(msg)

    # ── dispatch ─────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict) -> dict:
        if action == "start":
            if self._chassis is None:
                return {"error": "chassis_raw 卡片未启用，无法转向。"
                                 "请在 config.yaml 里打开 chassis_raw 后重启容器"}
            if not self._running:
                self._running = True
                self._last_state = None
                self._last_target_x = None
                self._cooldown_until = 0.0
                self._braked = False
                self._turn_count = 0
                self._thread = threading.Thread(
                    target=self._control_loop, daemon=True, name="follow_loop")
                self._thread.start()
            return {
                "state": "running",
                "topic_out": [{"topic": self._topic, "format": "data/json"}],
                "vop_topic": self._vop_topic,
                "hfov_deg": self._hfov_deg,
                "target_class": self._target_class,
                "hint": (
                    f"需要 vop 卡片以 input_topic=/{self._ns}/camera/head 同时运行；"
                    f"用 raw_input_info(source=\"dds:{self._topic}\") 读取跟随状态。"
                    "hfov_deg 未标定时转向角度会有系统偏差"
                ),
            }

        if action == "stop":
            self.stop()
            return {"state": "idle", "braked": True}

        if action == "config":
            changed = {}
            for key, caster in (("target_class", str),
                                ("confidence", float),
                                ("deadband", float),
                                ("gain", float),
                                ("min_step_deg", float),
                                ("max_step_deg", float),
                                ("lost_timeout_sec", float),
                                ("hfov_deg", float)):
                if key not in args or args[key] is None or args[key] == "":
                    continue
                try:
                    value = caster(args[key])
                except (TypeError, ValueError):
                    return {"error": f"{key} 类型不对"}
                if caster is float and not math.isfinite(value):
                    return {"error": f"{key} 必须是有限数值"}
                setattr(self, f"_{key}", value)
                changed[key] = value
            return {"status": "configured", "config": changed,
                    "state": "running" if self._running else "idle"}

        # info
        return {
            "state": "running" if self._running else "idle",
            "last": self._last_payload,
            "chassis_available": self._chassis is not None,
            "vop_topic": self._vop_topic,
            "hfov_deg": self._hfov_deg,
            "target_class": self._target_class,
            "deadband": self._deadband,
            "gain": self._gain,
            "min_step_deg": self._min_step_deg,
            "max_step_deg": self._max_step_deg,
            "turn_count": self._turn_count,
            "topic_out": [{"topic": self._topic, "format": "data/json"}],
        }
