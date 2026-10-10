# -*- coding: utf-8 -*-
"""
trend_processor —— Q5 "来拒去留" 传感器处理卡

订阅：
  * /nvidia_desktop/camera/rgb/objects  (std_msgs/String, JSON, VOP 人体检测输出)
  * /nvidia_desktop/camera/pointcloud   (std_msgs/UInt8MultiArray, Q5 紧凑 XYZ 点云)

Q5 的 pointcloud 不是 sensor_msgs/PointCloud2。参考 q5_media_bridge.py 与
legacy_device.CameraPointCloudPlugin._encode_pointcloud：桥接层用
std_msgs/UInt8MultiArray 承载一段紧凑二进制包体：

    <uint32 point_step>  <uint32 count>  <float32 xyz> * count       (little-endian)

其中 point_step == 12，点在机体系下：
    packet_x = -body_forward   （机器人正前方距离取负）
    packet_y = -camera_x       （相机右方镜像后取负）
    packet_z = -body_up        （相对机身零点向下）

因此某点 (x, y, z) 对应的机器人正前方距离为 forward = -x，水平角正切
近似为 y / forward。

发布：
  * /nvidia_desktop/trend_processor     (std_msgs/String, JSON)

策略：
  1. 从 objects JSON 中挑 confidence 最高的 "person"，取其归一化图像 x_norm。
  2. 依 x_norm 与相机水平 FOV 计算目标方向的角度带，筛选点云。
  3. 用带内点的 forward 距离中位数作为"人距离"样本。
  4. 维护固定时间窗内的 (t, distance) 样本，做最小二乘拟合得斜率。
  5. 斜率超阈值判 approaching/leaving，未超判 stable，样本不足或无数据判 unknown。
  6. 简单磁滞：连续 N 帧同向候选才切换趋势，避免抖动。

只使用现有依赖（rclpy、std_msgs、sensor_contract）+ 标准库。
"""

from __future__ import annotations

import json
import math
import struct
import time
from collections import deque

from sensor_contract import topic_out

try:
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import String, UInt8MultiArray

    _HAS_ROS2 = True

    _JSON_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=1,
                           durability=DurabilityPolicy.VOLATILE)
    _MEDIA_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST, depth=1,
                            durability=DurabilityPolicy.VOLATILE)
except Exception:
    _HAS_ROS2 = False

CARD = "trend_processor"
TYPE = "sensor"
NODE = "q5_trend_processor"
DESC = ("Q5 来拒去留：融合 VOP 人体检测与 D455 紧凑点云，"
        "输出 approaching/leaving/stable/unknown 趋势 JSON")

DEFAULT_OBJECTS_TOPIC = "/nvidia_desktop/camera/rgb/objects"
DEFAULT_POINTCLOUD_TOPIC = "/nvidia_desktop/camera/pointcloud"
DEFAULT_OUTPUT_TOPIC = "/nvidia_desktop/trend_processor"
FMT = "data/json"

# Intel RealSense D455 彩色 848x480 水平 FOV ≈ 87°。
_DEFAULT_HFOV_RAD = math.radians(87.0)


def _now_ms() -> int:
    return int(time.time() * 1000)


def _pick_top_person(objects_json: str):
    """挑 confidence 最高的 person，返回 (confidence, x_norm, y_norm) 或 None。"""
    try:
        payload = json.loads(objects_json)
    except Exception:
        return None
    if not isinstance(payload, dict):
        return None
    objects = payload.get("objects") or []
    if not isinstance(objects, list):
        return None
    best = None
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        if str(obj.get("name", "")).lower() != "person":
            continue
        pos = obj.get("position")
        if not (isinstance(pos, (list, tuple)) and len(pos) >= 2):
            continue
        try:
            conf = float(obj.get("confidence", 0.0))
            x_norm = float(pos[0])
            y_norm = float(pos[1])
        except Exception:
            continue
        if not (0.0 <= x_norm <= 1.0 and 0.0 <= y_norm <= 1.0):
            # 上游若已归一化到别的区间，仍然接受但夹紧到 [0,1] 以避免异常。
            x_norm = min(max(x_norm, 0.0), 1.0)
            y_norm = min(max(y_norm, 0.0), 1.0)
        if best is None or conf > best[0]:
            best = (conf, x_norm, y_norm)
    return best


def _parse_pointcloud(raw: bytes):
    """解析 Q5 紧凑点云: <u32 point_step><u32 count><f32 xyz>*count (little-endian)。

    返回 [(x, y, z), ...] 列表；出错或格式不符返回 None。
    """
    if raw is None or len(raw) < 8:
        return None
    try:
        point_step, count = struct.unpack_from("<II", raw, 0)
    except struct.error:
        return None
    if point_step != 12 or count <= 0 or count > 200000:
        return None
    expected = 8 + point_step * count
    if len(raw) < expected:
        return None
    try:
        return list(struct.iter_unpack("<fff", raw[8:expected]))
    except struct.error:
        return None


def _representative_distance(points, x_norm, hfov_rad, band_rad,
                             min_forward, min_count):
    """求人在图像 x_norm 方向上的代表性 forward 距离 (米)。

    Packet frame (见 legacy_device._encode_pointcloud)：
        forward = -packet_x
        packet_y/forward ≈ -camera_x/z = tan(angle_from_forward_towards_image_left)

    x_norm < 0.5 (图像左) → camera_x < 0 → packet_y > 0，即目标 ratio 为正。
    """
    if not points:
        return None
    theta = (0.5 - x_norm) * hfov_rad
    lo_r = math.tan(theta - band_rad)
    hi_r = math.tan(theta + band_rad)
    if lo_r > hi_r:
        lo_r, hi_r = hi_r, lo_r
    forwards = []
    for x, y, _z in points:
        forward = -x
        if forward < min_forward:
            continue
        if forward <= 1e-3:
            continue
        ratio = y / forward
        if ratio < lo_r or ratio > hi_r:
            continue
        forwards.append(forward)
    if len(forwards) < min_count:
        return None
    forwards.sort()
    mid = len(forwards) // 2
    if len(forwards) % 2:
        return forwards[mid]
    return 0.5 * (forwards[mid - 1] + forwards[mid])


def _linear_fit(samples):
    """(t_s, d_m) 序列的最小二乘拟合，返回 (slope_mps, r_squared)。"""
    n = len(samples)
    if n < 2:
        return 0.0, 0.0
    mean_t = sum(t for t, _ in samples) / n
    mean_d = sum(d for _, d in samples) / n
    num = 0.0
    den = 0.0
    for t, d in samples:
        dt = t - mean_t
        num += dt * (d - mean_d)
        den += dt * dt
    if den <= 1e-9:
        return 0.0, 0.0
    slope = num / den
    ss_tot = sum((d - mean_d) ** 2 for _, d in samples)
    if ss_tot <= 1e-9:
        return slope, 1.0
    intercept = mean_d - slope * mean_t
    ss_res = sum((d - (slope * t + intercept)) ** 2 for t, d in samples)
    r2 = max(0.0, min(1.0, 1.0 - ss_res / ss_tot))
    return slope, r2


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        del client
        cfg = dict(plugin_config or {})
        self._namespace = namespace
        self._objects_topic = str(cfg.get("objects_topic", DEFAULT_OBJECTS_TOPIC))
        self._pointcloud_topic = str(cfg.get("pointcloud_topic",
                                             DEFAULT_POINTCLOUD_TOPIC))
        self._topic = str(cfg.get("output_topic", DEFAULT_OUTPUT_TOPIC))
        self._hz = max(0.5, float(cfg.get("publish_rate_hz", 5.0)))
        self._window_s = max(0.5, float(cfg.get("window_s", 2.0)))
        self._min_samples = max(3, int(cfg.get("min_samples", 5)))
        self._slope_threshold = max(0.02,
                                    float(cfg.get("slope_threshold_mps", 0.15)))
        self._sample_max_age_s = max(self._window_s,
                                     float(cfg.get("sample_max_age_s", 3.0)))
        self._hfov_rad = float(cfg.get("hfov_rad", _DEFAULT_HFOV_RAD))
        self._band_rad = math.radians(float(cfg.get("band_deg", 15.0)))
        self._min_forward = max(0.05, float(cfg.get("min_forward_m", 0.2)))
        self._pc_min_count = max(1, int(cfg.get("min_point_matches", 5)))
        # 磁滞：新趋势需要连续 N 帧才能切换（除切到 unknown 立即生效）。
        self._hysteresis = max(1, int(cfg.get("hysteresis_frames", 2)))

        self._node = None
        self._pub = None
        # (recv_ms, confidence, x_norm, y_norm) 或 (recv_ms, None, None, None)
        self._last_person = None
        self._last_points = None
        self._points_recv_ms = None
        self._person_recv_ms = None
        # (t_monotonic, distance_m)
        self._samples = deque()
        self._last_trend = "unknown"
        self._pending_trend = "unknown"
        self._pending_count = 0
        self._parse_errors = 0
        self._last_error_log_ms = 0

        if _HAS_ROS2 and executor is not None:
            try:
                self._node = Node(NODE)
                self._pub = self._node.create_publisher(
                    String, self._topic, _JSON_QOS)
                self._node.create_subscription(
                    String, self._objects_topic, self._on_objects, _JSON_QOS)
                self._node.create_subscription(
                    UInt8MultiArray, self._pointcloud_topic,
                    self._on_pointcloud, _MEDIA_QOS)
                self._node.create_timer(1.0 / self._hz, self._tick)
                executor.add_node(self._node)
                print(f"[{CARD}] subscribed objects={self._objects_topic} "
                      f"pointcloud={self._pointcloud_topic} -> {self._topic} "
                      f"@ {self._hz:g}Hz", flush=True)
            except Exception as exc:
                print(f"[{CARD}] ROS2 setup failed: {exc}", flush=True)
                self._node = None
                self._pub = None

    # ── Callbacks (must not raise) ─────────────────────────────────────────
    def _on_objects(self, msg):
        try:
            person = _pick_top_person(getattr(msg, "data", ""))
            self._person_recv_ms = _now_ms()
            if person is not None:
                self._last_person = (self._person_recv_ms,
                                     person[0], person[1], person[2])
            else:
                self._last_person = (self._person_recv_ms, None, None, None)
        except Exception:
            # 单帧解析异常绝不能杀死回调。
            pass

    def _on_pointcloud(self, msg):
        try:
            data = getattr(msg, "data", None)
            if data is None:
                return
            if isinstance(data, (bytes, bytearray)):
                raw = bytes(data)
            else:
                raw = bytes(bytearray(data))
            points = _parse_pointcloud(raw)
            self._points_recv_ms = _now_ms()
            if points is not None:
                self._last_points = points
            else:
                self._parse_errors += 1
                now = _now_ms()
                if now - self._last_error_log_ms > 5000:
                    print(f"[{CARD}] pointcloud parse errors so far: "
                          f"{self._parse_errors}", flush=True)
                    self._last_error_log_ms = now
        except Exception:
            self._parse_errors += 1

    # ── Trend logic ────────────────────────────────────────────────────────
    def _prune_samples(self, now_mono):
        while self._samples and now_mono - self._samples[0][0] > self._window_s:
            self._samples.popleft()

    def _current_distance(self):
        person = self._last_person
        points = self._last_points
        recv_person = self._person_recv_ms
        recv_points = self._points_recv_ms
        now_ms = _now_ms()
        max_age_ms = int(self._sample_max_age_s * 1000)
        if person is None or person[1] is None:
            return None, "no_person"
        if recv_person is None or now_ms - recv_person > max_age_ms:
            return None, "person_stale"
        if points is None or not points:
            return None, "no_pointcloud"
        if recv_points is None or now_ms - recv_points > max_age_ms:
            return None, "pointcloud_stale"
        x_norm = person[2]
        try:
            dist = _representative_distance(
                points, x_norm, self._hfov_rad, self._band_rad,
                self._min_forward, self._pc_min_count)
        except Exception:
            dist = None
        if dist is None:
            return None, "no_matching_points"
        return dist, "ok"

    def _update_trend(self, distance):
        now_mono = time.monotonic()
        if distance is not None and math.isfinite(distance):
            self._samples.append((now_mono, distance))
        self._prune_samples(now_mono)
        if distance is None or len(self._samples) < self._min_samples:
            candidate = "unknown"
            slope = 0.0
            r2 = 0.0
        else:
            slope, r2 = _linear_fit(list(self._samples))
            if slope < -self._slope_threshold:
                candidate = "approaching"
            elif slope > self._slope_threshold:
                candidate = "leaving"
            else:
                candidate = "stable"
        # 磁滞：候选与已确认相同 → 保持；切到 unknown 立即生效；其他方向切换
        # 需要 hysteresis 帧连续同向候选。
        if candidate == self._last_trend:
            self._pending_trend = candidate
            self._pending_count = 0
        elif candidate == "unknown":
            self._last_trend = "unknown"
            self._pending_trend = "unknown"
            self._pending_count = 0
        elif candidate == self._pending_trend:
            self._pending_count += 1
            if self._pending_count >= self._hysteresis:
                self._last_trend = candidate
                self._pending_count = 0
        else:
            self._pending_trend = candidate
            self._pending_count = 1
        return self._last_trend, slope, r2

    def _tick(self):
        if self._pub is None:
            return
        try:
            distance, reason = self._current_distance()
            trend, slope, r2 = self._update_trend(distance)
            valid = distance is not None
            person_conf = 0.0
            if self._last_person is not None and self._last_person[1] is not None:
                try:
                    person_conf = float(self._last_person[1])
                except Exception:
                    person_conf = 0.0
            if valid:
                confidence = max(0.0, min(1.0, 0.5 * person_conf + 0.5 * r2))
            else:
                confidence = 0.0
            payload = {
                "trend": trend if valid else "unknown",
                "valid": valid,
                "confidence": round(confidence, 3),
                "timestamp": _now_ms(),
            }
            if valid:
                payload["distance_m"] = round(distance, 3)
                payload["slope_mps"] = round(slope, 4)
                payload["samples"] = len(self._samples)
                payload["r_squared"] = round(r2, 3)
            else:
                payload["reason"] = reason
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self._pub.publish(msg)
        except Exception as exc:
            # tick 抛出会导致 timer 被 executor 移除，这里必须吞掉。
            now = _now_ms()
            if now - self._last_error_log_ms > 5000:
                print(f"[{CARD}] tick error: {exc}", flush=True)
                self._last_error_log_ms = now

    # ── Plugin surface ─────────────────────────────────────────────────────
    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False,
            "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["info", "start", "stop"]}
                },
                "required": ["action"],
                "additionalProperties": False,
            },
            "topic_out": topic_out(self._topic, FMT),
        }

    def _data(self):
        return {
            "trend": self._last_trend,
            "samples": len(self._samples),
            "objects_topic": self._objects_topic,
            "pointcloud_topic": self._pointcloud_topic,
            "output_topic": self._topic,
            "person_last_recv_ms": self._person_recv_ms,
            "pointcloud_last_recv_ms": self._points_recv_ms,
            "parse_errors": self._parse_errors,
        }

    def start(self):
        return {"state": "running" if self._pub else "unavailable"}

    def stop(self):
        return {"state": "idle"}

    def dispatch(self, action, args):
        del args
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action in ("info", "read", "get", CARD):
            return {"state": "running" if self._pub else "unavailable",
                    "data": self._data(),
                    "topic_out": topic_out(self._topic, FMT)}
        return None


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
