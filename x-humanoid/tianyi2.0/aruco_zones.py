"""Read-only ArUco zone card for Tianyi's existing Orbbec head camera."""

import json
import threading
import time


ROLES = ("waiting", "sorting_1", "sorting_2")
LABELS = {"waiting": "待分拣区", "sorting_1": "分拣区 1", "sorting_2": "分拣区 2"}
DICTIONARIES = ("DICT_4X4_50", "DICT_4X4_100", "DICT_5X5_50", "DICT_6X6_50")
DEFAULTS = {
    "dictionary": "DICT_4X4_50",
    "waiting_size": "",
    "sorting_1_size": "",
    "sorting_2_size": "",
    "waiting_marker_id": "",
    "sorting_1_marker_id": "",
    "sorting_2_marker_id": "",
    "sorting_1_color": "red",
    "sorting_2_color": "blue",
}
DRAW_COLORS = {
    "waiting": (0, 176, 224),
    "sorting_1": (96, 160, 32),
    "sorting_2": (224, 160, 40),
}


def parse_size(value):
    if value is None or value == "":
        return None
    parts = value.split(",") if isinstance(value, str) else value
    if not isinstance(parts, (list, tuple)) or len(parts) != 2:
        raise ValueError("区域尺寸须为 宽,高，均为画面比例 0~1")
    try:
        size = [float(part) for part in parts]
    except (TypeError, ValueError) as exc:
        raise ValueError("区域尺寸必须是数字") from exc
    if not all(0 < side <= 1 for side in size):
        raise ValueError("区域宽、高须大于 0 且不超过 1")
    return size


def parse_marker_id(value):
    if value is None or value == "":
        return None
    try:
        marker_id = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("ArUco ID 必须是非负整数") from exc
    if marker_id < 0 or str(marker_id) != str(value).strip():
        raise ValueError("ArUco ID 必须是非负整数")
    return marker_id


def validate_config(raw):
    unknown = set(raw) - set(DEFAULTS)
    if unknown:
        raise ValueError(f"未知配置项: {', '.join(sorted(unknown))}")
    config = {**DEFAULTS, **raw}
    if config["dictionary"] not in DICTIONARIES:
        raise ValueError("不支持的 ArUco 字典")
    sizes = {role: parse_size(config[f"{role}_size"]) for role in ROLES}
    ids = [parse_marker_id(config[f"{role}_marker_id"]) for role in ROLES]
    configured_ids = [marker_id for marker_id in ids if marker_id is not None]
    if len(set(configured_ids)) != len(configured_ids):
        raise ValueError("三个区域的 ArUco ID 必须不同")
    marker_count = int(config["dictionary"].rsplit("_", 1)[1])
    if any(marker_id >= marker_count for marker_id in configured_ids):
        raise ValueError(f"当前字典的 ArUco ID 须小于 {marker_count}")
    for role in ("sorting_1", "sorting_2"):
        color = config[f"{role}_color"]
        if not isinstance(color, str) or not color.strip():
            raise ValueError("颜色标签不能为空")
        config[f"{role}_color"] = color.strip()
    return config, sizes


def assign_zones(markers, sizes, width, height, config):
    selected_ids = {role: parse_marker_id(config[f"{role}_marker_id"]) for role in ROLES}
    assigned = {role: next((marker for marker in markers
                            if marker["id"] == selected_ids[role]), None)
                if selected_ids[role] is not None else None
                for role in ROLES}
    zones = {}
    for role in ROLES:
        marker, size = assigned[role], sizes[role]
        rect = None
        clipped = False
        center = None
        if marker is not None:
            center = [marker["center_px"][0] / width,
                      marker["center_px"][1] / height]
        if center is not None and size is not None:
            raw = [center[0] - size[0] / 2, center[1] - size[1] / 2,
                   center[0] + size[0] / 2, center[1] + size[1] / 2]
            rect = [max(0.0, raw[0]), max(0.0, raw[1]),
                    min(1.0, raw[2]), min(1.0, raw[3])]
            clipped = rect != raw
        zones[role] = {
            "label": LABELS[role],
            "rect_normalized": rect,
            "size_normalized": size,
            "marker_id": marker["id"] if marker is not None else None,
            "configured_marker_id": selected_ids[role],
            "marker_visible": marker is not None,
            "marker_center_normalized": center,
            "clipped": clipped,
            "color": config[f"{role}_color"] if role != "waiting" else None,
        }
    rects = [zone["rect_normalized"] for zone in zones.values()]
    overlap = any(
        max(first[0], second[0]) < min(first[2], second[2])
        and max(first[1], second[1]) < min(first[3], second[3])
        for index, first in enumerate(rects) if first is not None
        for second in rects[index + 1:] if second is not None
    )
    chosen = [zone["marker_id"] for zone in zones.values()]
    ambiguous_ids = [marker_id for marker_id in selected_ids.values()
                     if marker_id is not None
                     and sum(marker["id"] == marker_id for marker in markers) > 1]
    ready = (all(rect is not None for rect in rects)
             and len(set(chosen)) == len(ROLES)
             and not ambiguous_ids and not overlap
             and not any(zone["clipped"] for zone in zones.values()))
    return {
        "ready": ready,
        "zones": zones,
        "detected_marker_ids": [marker["id"] for marker in markers],
        "detected_markers": [
            {"id": marker["id"],
             "center_normalized": [marker["center_px"][0] / width,
                                   marker["center_px"][1] / height]}
            for marker in markers
        ],
        "assignment": "marker_id",
        "ambiguous_detected_ids": ambiguous_ids,
        "zones_overlap": overlap,
        "image_width": width,
        "image_height": height,
        "coordinate_system": "image_top_left_normalized_0_to_1",
        "motion_enabled": False,
    }


def process_image(msg, config, sizes):
    """Decode one ROS RGB Image, detect markers, and return JSON plus overlay JPEG."""
    import cv2
    import numpy as np

    width, height = int(msg.width), int(msg.height)
    if width <= 0 or height <= 0 or msg.encoding.lower() not in ("rgb8", "bgr8"):
        raise ValueError("仅支持非空 rgb8/bgr8 RGB 画面")
    stride = int(msg.step)
    if stride < width * 3:
        raise ValueError("RGB 图像行宽不足")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if raw.size < stride * height:
        raise ValueError("RGB 图像数据不完整")
    image = np.ndarray((height, width, 3), dtype=np.uint8, buffer=raw,
                       strides=(stride, 3, 1))
    frame = (cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
             if msg.encoding.lower() == "rgb8" else image.copy())
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(getattr(aruco, config["dictionary"]))
    if hasattr(aruco, "ArucoDetector"):
        corners, ids, _ = aruco.ArucoDetector(dictionary).detectMarkers(gray)
    else:
        corners, ids, _ = aruco.detectMarkers(gray, dictionary)
    markers = []
    if ids is not None:
        for marker_id, quad in zip(ids.flatten().tolist(), corners):
            points = [[float(x), float(y)] for x, y in quad.reshape(4, 2)]
            center = [sum(point[0] for point in points) / 4,
                      sum(point[1] for point in points) / 4]
            markers.append({"id": int(marker_id), "center_px": center})
            cv2.polylines(frame, [np.rint(quad).astype(np.int32)], True, (0, 0, 255), 2)
            cv2.putText(frame, str(marker_id), tuple(map(round, center)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    result = assign_zones(markers, sizes, width, height, config)
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
    ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if not ok:
        raise ValueError("无法编码预览 JPEG")
    return result, jpeg.tobytes()


class ArucoZonesPlugin:
    def __init__(self, plugin_config, namespace, ros2):
        self._namespace = namespace.strip("/")
        self._ros2 = ros2
        self._config, self._sizes = validate_config({
            key: value for key, value in plugin_config.items() if key in DEFAULTS})
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._latest_frame = None
        self._frame_at = None
        self._last_data = None
        self._last_error = None
        self._input_topic = None
        self._sub_node = None
        self._pub_node = None
        self._subscription = None
        self._overlay_pub = None
        self._regions_pub = None
        prefix = f"/{self._namespace}" if self._namespace else ""
        self._camera_topic = f"{prefix}/camera/head"
        self._outputs = [
            {"topic": f"{prefix}/aruco_zones/overlay", "format": "image/jpeg"},
            {"topic": f"{prefix}/aruco_zones/regions", "format": "data/json"},
        ]

    def get_tool(self):
        properties = {"dictionary": {"type": "string", "enum": list(DICTIONARIES),
                                     "default": DEFAULTS["dictionary"]}}
        for role in ROLES:
            properties[f"{role}_marker_id"] = {
                "type": "string", "enum": [""] + [str(marker_id) for marker_id in range(100)],
                "default": "",
                "description": f"{LABELS[role]}对应的 ArUco ID；先在预览中查看识别结果",
            }
            properties[f"{role}_size"] = {
                "type": "string", "default": "",
                "description": f"{LABELS[role]}宽,高；占画面比例 0~1，以所选码中心为区域中心",
            }
        for role in ("sorting_1", "sorting_2"):
            properties[f"{role}_color"] = {
                "type": "string", "default": DEFAULTS[f"{role}_color"],
                "description": f"{LABELS[role]}对应颜色标签",
            }
        return {
            "name": "aruco_zones",
            "type": "processor",
            "multiInstance": False,
            "description": "按选定 ArUco ID 的实时位置与尺寸生成三区域；仅视觉，不抓取。",
            "topic_in": [{"format": "image/jpeg", "desc": "连接 camera_head 输出"}],
            "topic_out": self._outputs,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info", "config"]},
                    "input_topic": {"type": "string"},
                    **properties,
                },
                "x-action-params": {
                    "start": {"params": ["input_topic", *properties], "description": "使用头部 RGB 画面开始识别"},
                    "config": {"params": list(properties), "description": "更新区域与颜色"},
                    "info": {"params": [], "description": "查看当前识别状态"},
                    "stop": {"params": [], "description": "停止识别"},
                },
            },
            "configSchema": {"type": "object", "properties": properties},
        }

    def start(self):
        # Bundle startup precedes canvas connection; dispatch(start) activates.
        return None

    def _on_image(self, msg):
        if self._stop.is_set():
            return
        with self._lock:
            self._latest_frame = msg

    def _ensure_ros_endpoints(self):
        if self._sub_node is not None:
            return
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image, CompressedImage
        from std_msgs.msg import String

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self._sub_node = Node("tianyi2_aruco_sub", context=self._ros2.ctx_tianyi)
        self._pub_node = Node("tianyi2_aruco_pub", context=self._ros2.ctx_core)
        self._ros2.executor_tianyi.add_node(self._sub_node)
        self._ros2.executor_core.add_node(self._pub_node)
        self._subscription = self._sub_node.create_subscription(
            Image, "/ob_camera_head/color/image_raw", self._on_image, qos)
        self._overlay_pub = self._pub_node.create_publisher(
            CompressedImage, self._outputs[0]["topic"], qos)
        self._regions_pub = self._pub_node.create_publisher(
            String, self._outputs[1]["topic"], qos)

    def stop(self):
        with self._lifecycle_lock:
            self._stop.set()
            thread = self._thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=2)
            with self._lock:
                self._thread = thread if thread is not None and thread.is_alive() else None
                self._latest_frame = self._frame_at = self._last_data = None
                self._input_topic = None
        return {"state": "idle"}

    def dispatch(self, action, args):
        if action == "start":
            if args.get("input_topic") != self._camera_topic:
                raise ValueError(f"请将 camera_head 输出 {self._camera_topic} 连接到此卡片")
            changes = {key: value for key, value in args.items() if key in DEFAULTS}
            with self._lifecycle_lock:
                config, sizes = validate_config({**self._config, **changes})
                self._ensure_ros_endpoints()
                with self._lock:
                    self._config, self._sizes = config, sizes
                    self._input_topic = self._camera_topic
                    self._latest_frame = self._frame_at = self._last_data = None
                    if self._thread is not None and not self._thread.is_alive():
                        self._thread = None
                    if self._thread is not None and self._stop.is_set():
                        raise RuntimeError("ArUco 处理线程仍在停止，请稍后重试")
                    if self._thread is None:
                        self._stop.clear()
                        self._thread = threading.Thread(target=self._loop, daemon=True,
                                                        name="tianyi_aruco_zones")
                        self._thread.start()
            return {"state": "running", "topic_in": self._camera_topic,
                    "topic_out": self._outputs}
        if action == "stop":
            return self.stop()
        if action == "config":
            changes = {key: value for key, value in args.items() if key in DEFAULTS}
            with self._lock:
                self._config, self._sizes = validate_config({**self._config, **changes})
                self._last_data = self._frame_at = None
            return self.dispatch("info", {})
        if action == "info":
            with self._lock:
                age = time.monotonic() - self._frame_at if self._frame_at else None
                latest = dict(self._last_data) if self._last_data else None
                if latest is not None and (age is None or age > 3):
                    latest["ready"] = False
                return {
                    "state": "running" if self._thread is not None and self._thread.is_alive()
                    and not self._stop.is_set() else "idle",
                    "input_topic": self._input_topic,
                    "topic_out": self._outputs,
                    "config": dict(self._config),
                    "latest": latest,
                    "latest_frame_age_s": round(age, 3) if age is not None else None,
                    "last_error": self._last_error,
                }
        raise ValueError(f"Unsupported action: {action}")

    def _loop(self):
        from sensor_msgs.msg import CompressedImage
        from std_msgs.msg import String

        last_processed_at = 0.0
        while not self._stop.wait(0.02):
            now = time.monotonic()
            if now - last_processed_at < 0.2:
                continue
            with self._lock:
                frame = self._latest_frame
                self._latest_frame = None
                config, sizes = dict(self._config), dict(self._sizes)
            if frame is None:
                continue
            last_processed_at = now
            try:
                result, jpeg = process_image(frame, config, sizes)
                if self._stop.is_set():
                    break
                overlay = CompressedImage()
                overlay.format = "jpeg"
                overlay.data = jpeg
                regions = String()
                regions.data = json.dumps(result, ensure_ascii=False)
                self._overlay_pub.publish(overlay)
                self._regions_pub.publish(regions)
                with self._lock:
                    self._last_data, self._frame_at, self._last_error = (
                        result, time.monotonic(), None)
            except Exception as exc:
                with self._lock:
                    self._last_data, self._last_error = None, str(exc)
