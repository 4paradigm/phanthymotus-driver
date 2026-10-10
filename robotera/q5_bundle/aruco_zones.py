"""Q5 ArUco zone canvas card, consuming the existing RGB camera worker."""

import json
import threading
import time


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
    parts = value.split(",") if isinstance(value, str) else value
    if not isinstance(parts, (list, tuple)) or len(parts) != 4:
        raise ValueError("区域须为 x1,y1,x2,y2，坐标范围 0~1")
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


def process_jpeg(data, config, rects):
    import cv2
    import numpy as np

    frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
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
            center = [sum(point[0] for point in points) / 4,
                      sum(point[1] for point in points) / 4]
            markers.append({"id": int(marker_id), "center_px": center})
            cv2.polylines(frame, [np.rint(quad).astype(np.int32)], True, (0, 0, 255), 2)
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
    return result, jpeg.tobytes()


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        del executor
        self._namespace = namespace.strip("/")
        self._worker = getattr(client, "camera_worker", None)
        self._publish_media = getattr(client, "publish_media", None)
        self._config, self._rects = validate_config({
            key: value for key, value in plugin_config.items() if key in DEFAULTS})
        self._lock = threading.RLock()
        self._lifecycle_lock = threading.Lock()
        self._thread = None
        self._stop = threading.Event()
        self._input_topic = None
        self._last_data = None
        self._last_frame_at = None
        self._last_error = None
        base = f"/{self._namespace}/aruco_zones" if self._namespace else "/aruco_zones"
        self._outputs = [{"topic": f"{base}/overlay", "format": "image/jpeg"},
                         {"topic": f"{base}/regions", "format": "data/json"}]

    def get_tool(self):
        properties = {"dictionary": {"type": "string", "enum": list(DICTIONARIES),
                                     "default": DEFAULTS["dictionary"]}}
        for role in ROLES:
            properties[f"{role}_rect"] = {"type": "string", "default": "",
                "description": f"{LABELS[role]}: x1,y1,x2,y2；画面左上角为 0,0，右下角为 1,1。"}
        for role in ("sorting_1", "sorting_2"):
            properties[f"{role}_color"] = {"type": "string", "default": DEFAULTS[f"{role}_color"],
                                           "description": f"{LABELS[role]}对应颜色标签"}
        return {"name": "aruco_zones", "type": "processor", "multiInstance": False,
                "description": "识别 Q5 RGB 画面中的 ArUco 并划分三个区域；仅视觉，不抓取。",
                "topic_in": [{"format": "image/jpeg", "desc": "连接 camera_rgb 输出"}],
                "topic_out": self._outputs,
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "info", "config"]},
                    "input_topic": {"type": "string"}, **properties},
                    "x-action-params": {
                        "start": {"params": ["input_topic", *properties], "description": "使用 Q5 RGB 帧开始识别"},
                        "config": {"params": list(properties), "description": "更新区域与颜色"},
                        "info": {"params": [], "description": "查看当前识别状态"},
                        "stop": {"params": [], "description": "停止识别"}}},
                "configSchema": {"type": "object", "properties": properties}}

    def start(self):
        # Bundle startup precedes canvas connection; activate on canvas start.
        return {"state": "idle"}

    def stop(self):
        with self._lifecycle_lock:
            with self._lock:
                self._stop.set()
                thread = self._thread
                self._input_topic = None
                self._last_data = self._last_frame_at = None
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=2)
            with self._lock:
                self._thread = thread if thread is not None and thread.is_alive() else None
        return {"state": "idle"}

    def dispatch(self, action, args):
        if action == "start":
            expected = f"/{self._namespace}/camera/rgb" if self._namespace else "/camera/rgb"
            if args.get("input_topic") != expected:
                raise ValueError(f"请将 Q5 camera_rgb 输出 {expected} 连接到此卡片")
            if self._worker is None or not callable(self._publish_media):
                raise RuntimeError("Q5 RGB 相机工作进程或媒体桥不可用")
            changes = {key: value for key, value in args.items() if key in DEFAULTS}
            with self._lifecycle_lock:
                with self._lock:
                    config, rects = validate_config({**self._config, **changes})
                    self._config, self._rects = config, rects
                    self._input_topic = expected
                    self._last_data = self._last_frame_at = None
                    if self._thread is not None and not self._thread.is_alive():
                        self._thread = None
                    if self._thread is not None and self._stop.is_set():
                        raise RuntimeError("ArUco 处理线程仍在停止，请稍后重试")
                    if self._thread is None:
                        self._stop.clear()
                        self._thread = threading.Thread(target=self._loop, daemon=True,
                                                        name="q5_aruco_zones")
                        self._thread.start()
            return {"state": "running", "topic_in": expected, "topic_out": self._outputs}
        if action == "stop":
            return self.stop()
        if action == "config":
            with self._lock:
                self._config, self._rects = validate_config({**self._config, **args})
                self._last_data = self._last_frame_at = None
            return self.dispatch("info", {})
        if action == "info":
            with self._lock:
                age = time.monotonic() - self._last_frame_at if self._last_frame_at else None
                latest = dict(self._last_data) if self._last_data else None
                if latest is not None and (age is None or age > 3):
                    latest["ready"] = False
                return {"state": "running" if self._thread is not None else "idle",
                        "input_topic": self._input_topic, "topic_out": self._outputs,
                        "config": dict(self._config), "latest": latest,
                        "latest_frame_age_s": round(age, 3) if age is not None else None,
                        "last_error": self._last_error}
        raise ValueError(f"Unsupported action: {action}")

    def _loop(self):
        sequence = None
        last_processed_at = 0.0
        while not self._stop.is_set():
            frame, sequence = self._worker.wait_for_frame("rgb", sequence, timeout_s=0.5)
            if frame is None or self._stop.is_set():
                continue
            now = time.monotonic()
            if now - last_processed_at < 0.2:
                continue
            last_processed_at = now
            try:
                with self._lock:
                    config, rects = dict(self._config), dict(self._rects)
                result, jpeg = process_jpeg(frame["data"], config, rects)
                self._publish_media({"kind": "aruco_overlay", "data": jpeg})
                self._publish_media({"kind": "aruco_regions", "data": json.dumps(result, ensure_ascii=False)})
                with self._lock:
                    self._last_data, self._last_frame_at, self._last_error = result, time.monotonic(), None
            except Exception as exc:
                with self._lock:
                    self._last_data, self._last_error = None, str(exc)
                # Avoid repeated failure logs and unnecessary CPU use.
                self._stop.wait(0.2)


def make_plugin(config, namespace, executor, client):
    return Plugin(config, namespace, executor, client)
