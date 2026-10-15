"""睿曼 RM75 视觉画布与分拣任务 ArUco 码检测卡片（纯视觉处理，不控制机械臂运动）。

本卡片订阅来自 RealSense 上位机相机的 RGB 图像流（CompressedImage/JPEG），
使用 OpenCV 检测画面中所有可见的 ArUco 标记码：
1. 识别并提取所有 ArUco 码的 ID 数值与几何位置（归一化坐标、像素坐标与四角点坐标）；
2. 支持可选的分拣区域划分（待分拣区、分拣区 1、分拣区 2），为后续物体分拣与机械臂抓取任务提供依据；
3. 输出带标注框的 JPEG 预览图像与 regions JSON 数据。
"""

import json
import threading
import time

try:
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
except ImportError:
    Node = object
    qos_profile_sensor_data = None
    CompressedImage = String = None


# 分拣区域定义与配置
ROLES = ("waiting", "sorting_1", "sorting_2")
LABELS = {"waiting": "待分拣区", "sorting_1": "分拣区 1", "sorting_2": "分拣区 2"}
DICTIONARIES = ("DICT_4X4_50", "DICT_4X4_100", "DICT_5X5_50", "DICT_6X6_50")
DEFAULTS = {
    "dictionary": "DICT_4X4_50",
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
    """校验卡片配置参数（字典类型、区域非重叠性、颜色标签等）。"""
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
    """根据检测到的标记码与预设矩形区域分配分拣角色，并汇总所有码的坐标位置。

    Args:
        markers: 检测到的标记列表，每个元素包含 id、center_px、center_normalized 等
        rects: 各角色的归一化矩形区域字典
        width: 画面宽度（像素）
        height: 画面高度（像素）
        config: 当前卡片配置
    Returns:
        dict: 包含 detected_markers（全量码 ID 及归一化/像素坐标）、zones、ready 状态等
    """
    zones = {}
    for role in ROLES:
        rect = rects[role]
        ids = [
            marker["id"] for marker in markers
            if rect and rect[0] <= marker["center_px"][0] / width <= rect[2]
            and rect[1] <= marker["center_px"][1] / height <= rect[3]
        ]
        zones[role] = {
            "label": LABELS[role],
            "rect_normalized": rect,
            "marker_id": ids[0] if len(ids) == 1 else None,
            "marker_ids_in_region": ids,
            "color": config[f"{role}_color"] if role != "waiting" else None,
        }

    # 准备 detected_markers 列表供分拣和指向任务直接调用
    detected_markers = [
        {
            "id": int(m["id"]),
            "center_normalized": [
                round(float(m["center_normalized"][0]), 4),
                round(float(m["center_normalized"][1]), 4),
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

    has_configured_zones = any(rects.values())
    if has_configured_zones:
        # 当配置了区域时：所有区域均须配置有效、各绑定一个唯一码且三码互不相同
        ids = [zone["marker_id"] for zone in zones.values()]
        ready = (
            all(rects.values())
            and all(marker_id is not None for marker_id in ids)
            and len(set(ids)) == 3
        )
    else:
        # 当未配置区域时（纯 ArUco 识别模式）：画面中有识别到的码即表示数据就绪
        ready = len(markers) > 0

    return {
        "ready": ready,
        "zones": zones,
        "detected_marker_ids": [m["id"] for m in markers],
        "detected_markers": detected_markers,
        "image_width": width,
        "image_height": height,
        "coordinate_system": "image_top_left_normalized_0_to_1",
        "motion_enabled": False,
    }


class ArucoZonesPlugin:
    """RealMan RM75 视觉画布与分拣任务 ArUco 处理器插件。"""

    PREFIX = "aruco_zones"

    def __init__(self, plugin_config, namespace, executor):
        self._executor = executor
        self._namespace = namespace.strip("/")
        self._lock = threading.RLock()
        self._config, self._rects = validate_config({
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
        """返回卡片在 Agent Core 和 Motus 画布中注册的 Tool 描述。"""
        properties = {
            "dictionary": {
                "type": "string",
                "enum": list(DICTIONARIES),
                "default": DEFAULTS["dictionary"],
                "description": "使用的 ArUco 字典类型",
            },
        }
        for role in ROLES:
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
        """生命周期启动接口。"""
        return {"state": "idle"}

    def stop(self):
        """生命周期停止接口：销毁 ROS 订阅节点与发布器。"""
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
        """分发并处理外部动作请求。"""
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
                    self._image_pub = node.create_publisher(
                        CompressedImage, self._outputs[0]["topic"], 10
                    )
                    self._data_pub = node.create_publisher(
                        String, self._outputs[1]["topic"], 10
                    )
                    self._subscription = node.create_subscription(
                        CompressedImage, topic, self._on_frame, qos_profile_sensor_data
                    )
                    self._executor.add_node(node)
                self._input_topic, self._config, self._rects = topic, config, rects
            return {"state": "running", "topic_in": topic, "topic_out": self._outputs}

        if action == "stop":
            return self.stop()

        if action == "config":
            with self._lock:
                self._config, self._rects = validate_config({**self._config, **args})
            return self.dispatch("info", {})

        if action in ("info", "detect"):
            with self._lock:
                age = (
                    time.monotonic() - self._last_frame_at
                    if self._last_frame_at
                    else None
                )
                latest = dict(self._last_data) if self._last_data else None
                # 相机断流超过 3 秒时置 ready 为 False
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
                # 如果是 detect 动作，直接扁平化返回 latest 中的关键数据便于调用
                if action == "detect" and latest is not None:
                    res["detected_markers"] = latest.get("detected_markers", [])
                    res["detected_marker_ids"] = latest.get("detected_marker_ids", [])
                    res["ready"] = latest.get("ready", False)
                return res

        raise ValueError(f"Unsupported action: {action}")

    def _on_frame(self, message):
        """接收图像帧回调：进行 ArUco 标记检测、坐标计算与发布。"""
        now = time.monotonic()
        with self._lock:
            # 限制处理帧率不超过 5Hz（0.2s 间隔），避免占用过多算力
            if self._node is None or now - self._last_processed_at < 0.2:
                return
            self._last_processed_at = now
            config, rects = dict(self._config), dict(self._rects)

        try:
            import cv2
            import numpy as np

            if "jpeg" not in message.format.lower() and "jpg" not in message.format.lower():
                return
            frame = cv2.imdecode(
                np.frombuffer(bytes(message.data), np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                raise ValueError("无法解码 RGB JPEG")
            height, width = frame.shape[:2]

            aruco = cv2.aruco
            dictionary = aruco.getPredefinedDictionary(getattr(aruco, config["dictionary"]))
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # 兼容不同版本 OpenCV 的 ArUco 检测接口
            if hasattr(aruco, "ArucoDetector"):
                corners, ids, _ = aruco.ArucoDetector(dictionary).detectMarkers(gray)
            else:
                corners, ids, _ = aruco.detectMarkers(gray, dictionary)

            markers = []
            if ids is not None:
                for marker_id, quad in zip(ids.flatten().tolist(), corners):
                    points = [[float(x), float(y)] for x, y in quad.reshape(4, 2)]
                    cx = sum(point[0] for point in points) / 4.0
                    cy = sum(point[1] for point in points) / 4.0
                    markers.append({
                        "id": int(marker_id),
                        "center_px": [cx, cy],
                        "center_normalized": [cx / width, cy / height],
                        "corners_px": points,
                        "corners_normalized": [
                            [point[0] / width, point[1] / height] for point in points
                        ],
                    })
                    # 在图像上绘制角点连线和标记 ID
                    cv2.polylines(
                        frame, [np.rint(quad).astype(np.int32)], True, (0, 0, 255), 2
                    )
                    cv2.putText(
                        frame,
                        str(marker_id),
                        (round(cx), round(cy)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2,
                    )

            # 分配分拣区域并组织结果字典
            result = assign_zones(markers, rects, width, height, config)

            # 在图像上绘制分拣区域矩形框与文字标签
            for role, zone in result["zones"].items():
                rect = zone["rect_normalized"]
                if rect is None:
                    continue
                start = (round(rect[0] * width), round(rect[1] * height))
                end = (round(rect[2] * width), round(rect[3] * height))
                color = DRAW_COLORS[role]
                cv2.rectangle(frame, start, end, color, 2)
                label = f"{role}: {zone['marker_id'] if zone['marker_id'] is not None else '?'}"
                cv2.putText(
                    frame,
                    label,
                    (start[0], max(20, start[1] - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2,
                )

            # 编码带标注的图像与发布
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
                self._last_data, self._last_frame_at, self._last_error = (
                    result,
                    now,
                    None,
                )
        except Exception as exc:
            with self._lock:
                self._last_error = str(exc)
                self._last_data = None
