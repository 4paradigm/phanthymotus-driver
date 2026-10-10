"""ArUco zone processor for the RealMan canvas. No robot commands."""

import json
import threading
import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


ROLES = ("waiting", "sorting_1", "sorting_2")
LABELS = {"waiting": "待分拣区", "sorting_1": "分拣区 1", "sorting_2": "分拣区 2"}
DICTIONARIES = ("DICT_4X4_50", "DICT_4X4_100", "DICT_5X5_50", "DICT_6X6_50")
DEFAULTS = {"dictionary": "DICT_4X4_50", "waiting_rect": "", "sorting_1_rect": "",
            "sorting_2_rect": "", "sorting_1_color": "red", "sorting_2_color": "blue"}
DRAW_COLORS = {"waiting": (0, 176, 224), "sorting_1": (96, 160, 32),
               "sorting_2": (224, 160, 40)}


def parse_rect(value):
    if value is None or value == "":
        return None
    if isinstance(value, str):
        parts = value.split(",")
    elif isinstance(value, (list, tuple)):
        parts = value
    else:
        raise ValueError("区域须为 x1,y1,x2,y2，坐标范围 0~1")
    if len(parts) != 4:
        raise ValueError("区域须包含四个归一化坐标 x1,y1,x2,y2")
    try:
        rect = [float(part) for part in parts]
    except (TypeError, ValueError) as exc:
        raise ValueError("区域坐标必须是数字") from exc
    if not (0 <= rect[0] < rect[2] <= 1 and 0 <= rect[1] < rect[3] <= 1):
        raise ValueError("区域须满足 0 ≤ x1 < x2 ≤ 1、0 ≤ y1 < y2 ≤ 1")
    return rect


def _overlap(a, b):
    return max(a[0], b[0]) < min(a[2], b[2]) and max(a[1], b[1]) < min(a[3], b[3])


def validate_config(raw):
    unknown = set(raw) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"未知配置项: {', '.join(sorted(unknown))}")
    config = {**DEFAULTS, **raw}
    if config["dictionary"] not in DICTIONARIES:
        raise ValueError("不支持的 ArUco 字典")
    rects = {role: parse_rect(config[f"{role}_rect"]) for role in ROLES}
    present = [(role, rect) for role, rect in rects.items() if rect is not None]
    for index, (role, rect) in enumerate(present):
        for other_role, other in present[index + 1:]:
            if _overlap(rect, other):
                raise ValueError(f"{LABELS[role]}与{LABELS[other_role]}重叠")
    for role in ("sorting_1", "sorting_2"):
        color = config[f"{role}_color"]
        if not isinstance(color, str) or not color.strip():
            raise ValueError("颜色标签不能为空")
        config[f"{role}_color"] = color.strip()
    return config, rects


def assign_zones(markers, rects, width, height, config):
    zones = {}
    for role in ROLES:
        rect = rects[role]
        ids = ([marker["id"] for marker in markers
                if rect and rect[0] <= marker["center_px"][0] / width <= rect[2]
                and rect[1] <= marker["center_px"][1] / height <= rect[3]])
        zones[role] = {"label": LABELS[role], "rect_normalized": rect,
                       "marker_id": ids[0] if len(ids) == 1 else None,
                       "marker_ids_in_region": ids,
                       "color": config[f"{role}_color"] if role != "waiting" else None}
    ids = [zone["marker_id"] for zone in zones.values()]
    ready = all(rects.values()) and all(marker_id is not None for marker_id in ids) and len(set(ids)) == 3
    return {"ready": ready, "zones": zones, "detected_marker_ids": [m["id"] for m in markers],
            "image_width": width, "image_height": height,
            "coordinate_system": "image_top_left_normalized_0_to_1", "motion_enabled": False}


class ArucoZonesPlugin:
    PREFIX = "aruco_zones"

    def __init__(self, plugin_config, namespace, executor):
        self._executor = executor
        self._namespace = namespace.strip("/")
        self._lock = threading.RLock()
        self._config, self._rects = validate_config({key: value for key, value in plugin_config.items()
                                                     if key in DEFAULTS})
        self._node = None
        self._subscription = None
        self._image_pub = None
        self._data_pub = None
        self._input_topic = None
        self._last_data = None
        self._last_frame_at = None
        self._last_error = None
        self._last_processed_at = 0.0
        base = f"/{self._namespace}/aruco_zones" if self._namespace else "/aruco_zones"
        self._outputs = [{"topic": f"{base}/overlay", "format": "image/jpeg"},
                         {"topic": f"{base}/regions", "format": "data/json"}]

    def get_tool(self):
        properties = {
            "dictionary": {"type": "string", "enum": list(DICTIONARIES), "default": DEFAULTS["dictionary"]},
        }
        for role in ROLES:
            properties[f"{role}_rect"] = {"type": "string", "default": "",
                "description": f"{LABELS[role]}: x1,y1,x2,y2；画面左上角为 0,0，右下角为 1,1。"}
        for role in ("sorting_1", "sorting_2"):
            properties[f"{role}_color"] = {"type": "string", "default": DEFAULTS[f"{role}_color"],
                                           "description": f"{LABELS[role]}对应颜色标签"}
        return {"name": self.PREFIX, "type": "processor", "multiInstance": False,
                "description": "识别 RealSense RGB 画面中的 ArUco，并按三个矩形区域分配 ID；仅视觉，不抓取。",
                "topic_in": [{"format": "image/jpeg", "desc": "连接 ext_camera 的 RGB 输出"}],
                "topic_out": self._outputs,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info", "config"]},
                    "input_topic": {"type": "string"}, **properties},
                    "x-action-params": {
                        "start": {"params": ["input_topic", *properties], "description": "订阅 RGB 相机并开始识别"},
                        "config": {"params": list(properties), "description": "更新区域与颜色"},
                        "info": {"params": [], "description": "查看当前识别状态"},
                        "stop": {"params": [], "description": "停止识别"}}},
                "configSchema": {"type": "object", "properties": properties}}

    def start(self):
        # Bundle initialization runs before canvas connections are known.
        return {"state": "idle"}

    def stop(self):
        with self._lock:
            if self._node is not None:
                if self._subscription is not None:
                    self._node.destroy_subscription(self._subscription)
                self._executor.remove_node(self._node)
                self._node.destroy_node()
            self._node = self._subscription = self._image_pub = self._data_pub = None
            self._input_topic = None
            self._last_data = self._last_frame_at = self._last_error = None
        return {"state": "idle"}

    def dispatch(self, action, args):
        if action == "start":
            topic = args.get("input_topic")
            if not isinstance(topic, str) or not topic.startswith("/"):
                raise ValueError("请先将 ext_camera RGB 输出连接到此卡片")
            changes = {key: value for key, value in args.items() if key in DEFAULTS}
            with self._lock:
                config, rects = validate_config({**self._config, **changes})
                if self._node is not None and self._input_topic != topic:
                    self.stop()
                if self._node is None:
                    node = Node("realman_aruco_zones", context=self._executor.context)
                    self._node = node
                    self._image_pub = node.create_publisher(CompressedImage, self._outputs[0]["topic"], 10)
                    self._data_pub = node.create_publisher(String, self._outputs[1]["topic"], 10)
                    self._subscription = node.create_subscription(
                        CompressedImage, topic, self._on_frame, qos_profile_sensor_data)
                    self._executor.add_node(node)
                self._input_topic, self._config, self._rects = topic, config, rects
            return {"state": "running", "topic_in": topic, "topic_out": self._outputs}
        if action == "stop":
            return self.stop()
        if action == "config":
            with self._lock:
                self._config, self._rects = validate_config({**self._config, **args})
            return self.dispatch("info", {})
        if action == "info":
            with self._lock:
                age = time.monotonic() - self._last_frame_at if self._last_frame_at else None
                latest = dict(self._last_data) if self._last_data else None
                if latest is not None and (age is None or age > 3):
                    latest["ready"] = False
                return {"state": "running" if self._node else "idle", "input_topic": self._input_topic,
                        "topic_out": self._outputs, "config": dict(self._config), "latest": latest,
                        "latest_frame_age_s": round(age, 3) if age is not None else None,
                        "last_error": self._last_error}
        raise ValueError(f"Unsupported action: {action}")

    def _on_frame(self, message):
        now = time.monotonic()
        with self._lock:
            if self._node is None or now - self._last_processed_at < 0.2:
                return
            self._last_processed_at = now
            config, rects = dict(self._config), dict(self._rects)
        try:
            import cv2
            import numpy as np
            if "jpeg" not in message.format.lower() and "jpg" not in message.format.lower():
                return
            frame = cv2.imdecode(np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("无法解码 RGB JPEG")
            height, width = frame.shape[:2]
            aruco = cv2.aruco
            dictionary = aruco.getPredefinedDictionary(getattr(aruco, config["dictionary"]))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.ArucoDetector(dictionary).detectMarkers(gray)
            markers = []
            if ids is not None:
                for marker_id, quad in zip(ids.flatten().tolist(), corners):
                    points = [[float(x), float(y)] for x, y in quad.reshape(4, 2)]
                    markers.append({"id": int(marker_id), "center_px": [
                        sum(point[0] for point in points) / 4,
                        sum(point[1] for point in points) / 4]})
                    cv2.polylines(frame, [np.rint(quad).astype(np.int32)], True, (0, 0, 255), 2)
                    center = markers[-1]["center_px"]
                    cv2.putText(frame, str(marker_id), tuple(map(round, center)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            result = assign_zones(markers, rects, width, height, config)
            for role, zone in result["zones"].items():
                rect = zone["rect_normalized"]
                if rect is None:
                    continue
                start = (round(rect[0] * width), round(rect[1] * height))
                end = (round(rect[2] * width), round(rect[3] * height))
                color = DRAW_COLORS[role]
                cv2.rectangle(frame, start, end, color, 2)
                label = f"{role}: {zone['marker_id'] if zone['marker_id'] is not None else '?'}"
                cv2.putText(frame, label, (start[0], max(20, start[1] - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            encoded, jpeg = cv2.imencode(".jpg", frame)
            if not encoded:
                raise ValueError("无法编码预览 JPEG")
            image_message = CompressedImage()
            image_message.header = message.header
            image_message.format = "jpeg"
            image_message.data = jpeg.tobytes()
            json_message = String()
            json_message.data = json.dumps(result, ensure_ascii=False)
            with self._lock:
                if self._node is None:
                    return
                self._image_pub.publish(image_message)
                self._data_pub.publish(json_message)
                self._last_data, self._last_frame_at, self._last_error = result, now, None
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._last_data = None
