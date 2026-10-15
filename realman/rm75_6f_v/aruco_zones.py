"""睿曼 RM75 视觉画布与分拣任务 ArUco 标记检测卡片（只读视觉处理，不控制机械臂运动）。

参考实现基于 x-humanoid/tianyi2.0/aruco_zones.py，针对睿曼 RM75 上位机 RealSense 相机
与分拣抓取任务进行了适配：
1. 从相机 RGB 图像流（CompressedImage 或 Image）中识别所有可见 ArUco 码的 ID 数值与几何位置；
2. 提取每个码的中心位置（包含归一化坐标 [0, 1] 与像素坐标）及四个角点坐标；
3. 支持基于标记码 ID 锚定分拣区域（待分拣区、分拣区 1、分拣区 2）或直接按矩形区域划分；
4. 当未配置具体分拣区域时，作为通用 ArUco 检测卡片直接向分拣任务提供 detected_markers 列表；
5. 输出带标注的 JPEG 图像（overlay）与 regions JSON 数据。
"""

import json
import threading
import time

try:
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
except ImportError:
    Node = object
    QoSProfile = ReliabilityPolicy = HistoryPolicy = qos_profile_sensor_data = None
    CompressedImage = String = None


# 分拣区域定义与配置
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
    "waiting_rect": "",
    "sorting_1_rect": "",
    "sorting_2_rect": "",
    "sorting_1_color": "red",
    "sorting_2_color": "blue",
}
DRAW_COLORS = {
    "waiting": (0, 176, 224),
    "sorting_1": (96, 160, 32),
    "sorting_2": (224, 160, 40),
}


def parse_size(value):
    """解析并校验归一化尺寸宽,高（占画面比例 0~1）。"""
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
    """解析并校验 ArUco ID（非负整数）。"""
    if value is None or value == "":
        return None
    try:
        marker_id = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("ArUco ID 必须是非负整数") from exc
    if marker_id < 0 or str(marker_id) != str(value).strip():
        raise ValueError("ArUco ID 必须是非负整数")
    return marker_id


def parse_rect(value):
    """解析并校验归一化矩形区域 [x1, y1, x2, y2]，范围 0~1。"""
    if value is None or value == "":
        return None
    if isinstance(value, str):
        parts = [p.strip() for p in value.split(",")]
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
    """判断两个归一化矩形是否重叠。"""
    return max(a[0], b[0]) < min(a[2], b[2]) and max(a[1], b[1]) < min(a[3], b[3])


def validate_config(raw):
    """校验卡片配置参数（支持通过 marker_id+size 锚定区域，也兼容直接传 rect 矩形框）。"""
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

    # 兼容直接配置矩形框 rect
    rects = {role: parse_rect(config[f"{role}_rect"]) for role in ROLES}
    present_rects = [(role, rect) for role, rect in rects.items() if rect is not None]
    for index, (role, rect) in enumerate(present_rects):
        for other_role, other in present_rects[index + 1:]:
            if _overlap(rect, other):
                raise ValueError(f"{LABELS[role]}与{LABELS[other_role]}重叠")

    for role in ("sorting_1", "sorting_2"):
        color = config[f"{role}_color"]
        if not isinstance(color, str) or not color.strip():
            raise ValueError("颜色标签不能为空")
        config[f"{role}_color"] = color.strip()

    return config, sizes, rects


def assign_zones(markers, sizes, width, height, config, rects=None):
    """根据检测到的标记码分配分拣区域，并汇总提取所有码的 ID 与坐标位置。

    Args:
        markers: 检测到的标记列表，每个元素包含 id、center_px 等
        sizes: 各角色基于码锚定的尺寸字典
        width: 画面宽度（像素）
        height: 画面高度（像素）
        config: 当前卡片配置
        rects: 可选的直接预设矩形区域
    Returns:
        dict: 包含 detected_markers、detected_marker_ids、zones、ready 状态等
    """
    selected_ids = {role: parse_marker_id(config.get(f"{role}_marker_id")) for role in ROLES}
    has_marker_anchors = any(selected_ids.values())
    has_fixed_rects = rects is not None and any(rects.values())

    zones = {}
    if has_fixed_rects:
        # 模式 A：按预设静态矩形框分配区域（中心落入矩形即绑定）
        for role in ROLES:
            rect = rects[role]
            ids_in_box = [
                m["id"] for m in markers
                if rect and rect[0] <= m["center_px"][0] / width <= rect[2]
                and rect[1] <= m["center_px"][1] / height <= rect[3]
            ]
            zones[role] = {
                "label": LABELS[role],
                "rect_normalized": rect,
                "marker_id": ids_in_box[0] if len(ids_in_box) == 1 else None,
                "marker_ids_in_region": ids_in_box,
                "color": config.get(f"{role}_color") if role != "waiting" else None,
            }
        box_ids = [zone["marker_id"] for zone in zones.values()]
        ready = (
            all(rects.values())
            and all(marker_id is not None for marker_id in box_ids)
            and len(set(box_ids)) == 3
        )
        overlap = False
        ambiguous_ids = []
    elif has_marker_anchors:
        # 模式 B：参考 tianyi2.0 按照 ArUco ID 锚定动态区域
        assigned = {
            role: next((m for m in markers if m["id"] == selected_ids[role]), None)
            if selected_ids[role] is not None else None
            for role in ROLES
        }
        for role in ROLES:
            marker, size = assigned[role], sizes[role]
            rect = None
            clipped = False
            center = None
            if marker is not None:
                center = [marker["center_px"][0] / width, marker["center_px"][1] / height]
            if center is not None and size is not None:
                raw = [center[0] - size[0] / 2, center[1] - size[1] / 2,
                       center[0] + size[0] / 2, center[1] + size[1] / 2]
                rect = [max(0.0, raw[0]), max(0.0, raw[1]), min(1.0, raw[2]), min(1.0, raw[3])]
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
                "color": config.get(f"{role}_color") if role != "waiting" else None,
            }
        active_rects = [zone["rect_normalized"] for zone in zones.values()]
        overlap = any(
            max(first[0], second[0]) < min(first[2], second[2])
            and max(first[1], second[1]) < min(first[3], second[3])
            for index, first in enumerate(active_rects) if first is not None
            for second in active_rects[index + 1:] if second is not None
        )
        chosen = [zone["marker_id"] for zone in zones.values()]
        ambiguous_ids = [
            marker_id for marker_id in selected_ids.values()
            if marker_id is not None and sum(m["id"] == marker_id for m in markers) > 1
        ]
        ready = (
            all(rect is not None for rect in active_rects)
            and len(set(chosen)) == len(ROLES)
            and not ambiguous_ids and not overlap
            and not any(zone["clipped"] for zone in zones.values())
        )
    else:
        # 模式 C：通用识别模式，未配置区域时只要检测到码即置 ready=True
        for role in ROLES:
            zones[role] = {
                "label": LABELS[role],
                "rect_normalized": None,
                "marker_id": None,
                "color": config.get(f"{role}_color") if role != "waiting" else None,
            }
        ready = len(markers) > 0
        overlap = False
        ambiguous_ids = []

    # 提取所有识别到的码位置详情供下游分拣和指向调用
    detected_markers = [
        {
            "id": int(m["id"]),
            "center_normalized": [
                round(float(m["center_px"][0] / width), 4),
                round(float(m["center_px"][1] / height), 4),
            ],
            "center_px": [
                round(float(m["center_px"][0]), 2),
                round(float(m["center_px"][1]), 2),
            ],
            "corners_px": m.get("corners_px", []),
            "corners_normalized": m.get("corners_normalized", []),
        }
        for m in markers
    ]

    return {
        "ready": ready,
        "zones": zones,
        "detected_marker_ids": [m["id"] for m in markers],
        "detected_markers": detected_markers,
        "assignment": "marker_id" if has_marker_anchors else "rect",
        "ambiguous_detected_ids": ambiguous_ids,
        "zones_overlap": overlap,
        "image_width": width,
        "image_height": height,
        "coordinate_system": "image_top_left_normalized_0_to_1",
        "motion_enabled": False,
    }


def process_image(msg, config, sizes, rects=None):
    """解码图像数据（JPEG 或原始 RGB），检测 ArUco 标记并绘制标注。"""
    import cv2
    import numpy as np

    if hasattr(msg, "width"):
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
    else:
        if not getattr(msg, "format", "").lower().startswith(("jpeg", "jpg")):
            raise ValueError("画布输入必须是 JPEG")
        encoded = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("无法解码相机 JPEG")
        height, width = frame.shape[:2]

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
            center = [sum(point[0] for point in points) / 4.0,
                      sum(point[1] for point in points) / 4.0]
            markers.append({
                "id": int(marker_id),
                "center_px": center,
                "center_normalized": [center[0] / width, center[1] / height],
                "corners_px": points,
                "corners_normalized": [[p[0] / width, p[1] / height] for p in points],
            })
            cv2.polylines(frame, [np.rint(quad).astype(np.int32)], True, (0, 0, 255), 2)
            cv2.putText(frame, str(marker_id), tuple(map(round, center)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    result = assign_zones(markers, sizes, width, height, config, rects=rects)
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
    """RealMan RM75 视觉 ArUco 检测卡片插件。"""

    PREFIX = "aruco_zones"

    def __init__(self, plugin_config, namespace, executor):
        self._namespace = namespace.strip("/")
        # 兼容不同 ROS2 运行时封装对象
        if hasattr(executor, "executor_core"):
            self._executor = executor.executor_core
            self._context = getattr(executor, "ctx_core", None)
        elif hasattr(executor, "context"):
            self._executor = executor
            self._context = executor.context
        else:
            self._executor = executor
            self._context = getattr(executor, "context", None)

        self._lock = threading.RLock()
        self._config, self._sizes, self._rects = validate_config({
            key: value for key, value in plugin_config.items() if key in DEFAULTS
        })
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
        self._outputs = [
            {"topic": f"{base}/overlay", "format": "image/jpeg"},
            {"topic": f"{base}/regions", "format": "data/json"},
        ]

    def get_tool(self):
        """返回卡片在 Agent Core 和 Motus 中注册的 Tool 描述。"""
        properties = {
            "dictionary": {
                "type": "string",
                "enum": list(DICTIONARIES),
                "default": DEFAULTS["dictionary"],
                "description": "使用的 ArUco 字典类型",
            },
        }
        for role in ROLES:
            properties[f"{role}_marker_id"] = {
                "type": "string",
                "enum": [""] + [str(marker_id) for marker_id in range(100)],
                "default": "",
                "description": f"{LABELS[role]}对应的 ArUco ID；先在预览中查看识别结果",
            }
            properties[f"{role}_size"] = {
                "type": "string",
                "default": "",
                "description": f"{LABELS[role]}宽,高；占画面比例 0~1，以所选码中心为区域中心",
            }
            properties[f"{role}_rect"] = {
                "type": "string",
                "default": "",
                "description": f"{LABELS[role]}: x1,y1,x2,y2；画面左上角为 0,0，右下角为 1,1。",
            }
        for role in ("sorting_1", "sorting_2"):
            properties[f"{role}_color"] = {
                "type": "string",
                "default": DEFAULTS[f"{role}_color"],
                "description": f"{LABELS[role]}对应颜色标签",
            }

        return {
            "name": self.PREFIX,
            "type": "processor",
            "multiInstance": False,
            "description": "识别 RealSense RGB 画面中的 ArUco 码 ID 与位置坐标，支持分拣任务区域规划；仅视觉处理，不控制机械臂。",
            "topic_in": [{"format": "image/jpeg", "desc": "连接 ext_camera 的 RGB 输出"}],
            "topic_out": self._outputs,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info", "detect", "config"],
                    },
                    "input_topic": {
                        "type": "string",
                        "description": "RGB 图像输入 ROS2 主题，默认为 ext_camera 输出",
                    },
                    **properties,
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {
                        "params": ["input_topic", *properties],
                        "description": "订阅 RGB 相机并开始识别 ArUco 码",
                    },
                    "detect": {
                        "params": [],
                        "description": "获取当前画面中检测到的所有 ArUco 码 ID 及位置坐标",
                    },
                    "info": {
                        "params": [],
                        "description": "查看当前识别状态、码 ID 列表与区域数据",
                    },
                    "config": {
                        "params": list(properties),
                        "description": "更新区域坐标与字典配置",
                    },
                    "stop": {
                        "params": [],
                        "description": "停止识别并释放订阅节点",
                    },
                },
            },
            "configSchema": {"type": "object", "properties": properties},
        }

    def start(self):
        return {"state": "idle"}

    def stop(self):
        with self._lock:
            if self._node is not None:
                if self._subscription is not None:
                    self._node.destroy_subscription(self._subscription)
                if self._executor is not None:
                    try:
                        self._executor.remove_node(self._node)
                    except Exception:
                        pass
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
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
                config, sizes, rects = validate_config({**self._config, **changes})
                if self._node is not None and self._input_topic != topic:
                    self.stop()
                if self._node is None:
                    ctx = self._context or getattr(self._executor, "context", None)
                    node = Node("realman_aruco_zones", context=ctx)
                    self._node = node
                    self._image_pub = node.create_publisher(
                        CompressedImage, self._outputs[0]["topic"], 10
                    )
                    self._data_pub = node.create_publisher(
                        String, self._outputs[1]["topic"], 10
                    )
                    self._subscription = node.create_subscription(
                        CompressedImage, topic, self._on_frame, qos_profile_sensor_data
                    )
                    if self._executor is not None:
                        self._executor.add_node(node)
                self._input_topic, self._config, self._sizes, self._rects = topic, config, sizes, rects
            return {"state": "running", "topic_in": topic, "topic_out": self._outputs}

        if action == "stop":
            return self.stop()

        if action == "config":
            with self._lock:
                self._config, self._sizes, self._rects = validate_config({**self._config, **args})
            return self.dispatch("info", {})

        if action in ("info", "detect"):
            with self._lock:
                age = (
                    time.monotonic() - self._last_frame_at
                    if self._last_frame_at
                    else None
                )
                latest = dict(self._last_data) if self._last_data else None
                if latest is not None and (age is None or age > 3):
                    latest["ready"] = False
                res = {
                    "state": "running" if self._node else "idle",
                    "input_topic": self._input_topic,
                    "topic_out": self._outputs,
                    "config": dict(self._config),
                    "latest": latest,
                    "latest_frame_age_s": round(age, 3) if age is not None else None,
                    "last_error": self._last_error,
                }
                if action == "detect" and latest is not None:
                    res["detected_markers"] = latest.get("detected_markers", [])
                    res["detected_marker_ids"] = latest.get("detected_marker_ids", [])
                    res["ready"] = latest.get("ready", False)
                return res

        raise ValueError(f"Unsupported action: {action}")

    def _on_frame(self, message):
        now = time.monotonic()
        with self._lock:
            if self._node is None or now - self._last_processed_at < 0.2:
                return
            self._last_processed_at = now
            config, sizes, rects = dict(self._config), dict(self._sizes), dict(self._rects)

        try:
            result, jpeg = process_image(message, config, sizes, rects=rects)
            image_message = CompressedImage()
            image_message.header = getattr(message, "header", image_message.header)
            image_message.format = "jpeg"
            image_message.data = jpeg

            json_message = String()
            json_message.data = json.dumps(result, ensure_ascii=False)

            with self._lock:
                if self._node is None:
                    return
                self._image_pub.publish(image_message)
                self._data_pub.publish(json_message)
                self._last_data, self._last_frame_at, self._last_error = (
                    result,
                    now,
                    None,
                )
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._last_data = None
