"""睿曼 RM75 视觉颜色分类与物体检测卡片（纯视觉处理，不控制机械臂运动）。

功能：
1. 订阅 RealSense 相机 RGB 图像流（CompressedImage 或 Image）；
2. 基于 HSV 色彩空间与自适应阈值，识别画面中的目标颜色（红色、黄色、黑色、蓝色、绿色等）；
3. 提取各颜色物体的中心点坐标（center_px, center_normalized）、外接边界框（bbox）与尺寸面积；
4. 提供可调用的动作接口（detect / classify / info / config / start / stop），
   上层 AgentCore / 决策核心可直接调用它回答诸如“画面中是否有黑色/红色/黄色物体”、“黄色物体在哪里”等问题；
5. 在输出话题发布带颜色标注的预览图（overlay）和检测结果 JSON（objects）。
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


SUPPORTED_COLORS = ("red", "yellow", "black", "blue", "green", "white", "orange", "all")

# 常用颜色的显示与绘制 BGR 颜色
DRAW_COLORS_BGR = {
    "red": (0, 0, 255),
    "yellow": (0, 255, 255),
    "black": (40, 40, 40),
    "blue": (255, 0, 0),
    "green": (0, 255, 0),
    "white": (240, 240, 240),
    "orange": (0, 140, 255),
}


def create_color_masks(cv2_module, image_bgr):
    """根据 HSV 色彩空间与亮度生成各常见颜色的二值掩码。"""
    np = cv2_module
    import numpy as np

    hsv = cv2_module.cvtColor(image_bgr, cv2_module.COLOR_BGR2HSV)
    gray = cv2_module.cvtColor(image_bgr, cv2_module.COLOR_BGR2GRAY)

    masks = {}

    # 1. 红色（跨越色相 0 与 180）
    mask_red1 = cv2_module.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
    mask_red2 = cv2_module.inRange(hsv, np.array([168, 70, 50]), np.array([180, 255, 255]))
    masks["red"] = cv2_module.bitwise_or(mask_red1, mask_red2)

    # 2. 黄色 / 浅黄绿（如网球、黄色包装）
    masks["yellow"] = cv2_module.inRange(hsv, np.array([18, 70, 70]), np.array([38, 255, 255]))

    # 3. 黑色（低明度 V < 70 且亮度 gray < 70）
    mask_v = cv2_module.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, 75]))
    mask_gray = cv2_module.inRange(gray, 0, 70)
    masks["black"] = cv2_module.bitwise_and(mask_v, mask_gray)

    # 4. 蓝色
    masks["blue"] = cv2_module.inRange(hsv, np.array([95, 70, 50]), np.array([130, 255, 255]))

    # 5. 绿色
    masks["green"] = cv2_module.inRange(hsv, np.array([38, 70, 50]), np.array([85, 255, 255]))

    # 6. 橙色
    masks["orange"] = cv2_module.inRange(hsv, np.array([11, 100, 100]), np.array([20, 255, 255]))

    return masks


def detect_color_objects(image_bgr, target_color="all", min_area=300):
    """从 BGR 图像中检测各颜色物体并提取其位置与尺寸。

    返回:
        list[dict]: 检测到的各颜色物体列表，按面积从大到小排序。
    """
    import cv2
    import numpy as np

    height, width = image_bgr.shape[:2]
    total_image_area = float(max(1, width * height))
    masks = create_color_masks(cv2, image_bgr)

    colors_to_check = list(masks.keys()) if (target_color in ("all", "", None)) else [target_color]
    results = []
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    for color_name in colors_to_check:
        if color_name not in masks:
            continue
        mask = masks[color_name]
        # 形态学滤波去除散点噪声
        cleaned = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = float(cv2.contourArea(c))
            # 过滤面积过小或占满全屏的异常斑块
            if area < min_area or area > (total_image_area * 0.85):
                continue

            x, y, w, h = cv2.boundingRect(c)
            cx = float(x + w / 2.0)
            cy = float(y + h / 2.0)

            results.append({
                "color": color_name,
                "center_px": [round(cx, 2), round(cy, 2)],
                "center_normalized": [round(cx / width, 4), round(cy / height, 4)],
                "center": [round(cx / width, 4), round(cy / height, 4)],
                "bbox_px": [int(x), int(y), int(w), int(h)],
                "bbox_normalized": [
                    round(x / width, 4),
                    round(y / height, 4),
                    round(w / width, 4),
                    round(h / height, 4),
                ],
                "width_px": int(w),
                "height_px": int(h),
                "area_px": int(area),
                "area_normalized": round(area / total_image_area, 4),
            })

    # 按面积从大到小排序
    results.sort(key=lambda item: item["area_px"], reverse=True)
    return results


def annotate_color_image(image_bgr, detected_objects):
    """在图像上绘制检测到的颜色物体边框、中心点和颜色文字标签。"""
    import cv2

    canvas = image_bgr.copy()
    for obj in detected_objects:
        color_name = obj["color"]
        draw_color = DRAW_COLORS_BGR.get(color_name, (0, 255, 0))
        x, y, w, h = obj["bbox_px"]
        cx, cy = int(round(obj["center_px"][0])), int(round(obj["center_px"][1]))

        # 1. 矩形外接框
        cv2.rectangle(canvas, (x, y), (x + w, y + h), draw_color, 2)

        # 2. 中心点十字线与圆点
        cv2.circle(canvas, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(canvas, (cx - 6, cy), (cx + 6, cy), (0, 0, 255), 1)
        cv2.line(canvas, (cx, cy - 6), (cx, cy + 6), (0, 0, 255), 1)

        # 3. 颜色标签文字
        label = f"{color_name} ({w}x{h}px)"
        ty = max(16, y - 6)
        cv2.putText(canvas, label, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)

    return canvas


def process_image(msg, target_color="all", min_area=300):
    """解码相机输入的图像消息，执行颜色识别并生成标注预览。"""
    import cv2
    import numpy as np

    if isinstance(msg, (bytes, bytearray)):
        image = cv2.imdecode(np.frombuffer(msg, dtype=np.uint8), cv2.IMREAD_COLOR)
    elif hasattr(msg, "format") and hasattr(msg, "data"):
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    elif hasattr(msg, "width") and hasattr(msg, "height") and hasattr(msg, "data"):
        width, height = int(msg.width), int(msg.height)
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding.lower() == "rgb8":
            image = cv2.cvtColor(raw.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
        elif msg.encoding.lower() == "bgr8":
            image = raw.reshape((height, width, 3)).copy()
        else:
            raise ValueError(f"不支持的图像编码: {msg.encoding}")
    elif isinstance(msg, np.ndarray):
        image = msg
    else:
        raise ValueError(f"无法解析的图像输入类型: {type(msg)}")

    if image is None or image.size == 0:
        raise ValueError("图像解码失败或数据为空")

    height, width = image.shape[:2]
    detected = detect_color_objects(image, target_color=target_color, min_area=min_area)
    distinct_colors = sorted(list(set(obj["color"] for obj in detected)))

    result = {
        "status": "success",
        "detected_colors": distinct_colors,
        "object_count": len(detected),
        "objects": detected,
        "image_width": width,
        "image_height": height,
        "timestamp": time.time(),
    }

    annotated = annotate_color_image(image, detected)
    success, jpeg_buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    jpeg_bytes = jpeg_buf.tobytes() if success else b""
    return result, jpeg_bytes


class ColorDetectorPlugin:
    """睿曼 RM75 颜色分类检测卡片插件。"""

    PREFIX = "color_detector"

    def __init__(self, plugin_config, namespace, executor):
        self._namespace = namespace
        self._executor = executor
        self._input_topic = plugin_config.get(
            "input_topic",
            f"/{namespace}/ext_camera/default/rgb" if namespace else "/ext_camera/default/rgb"
        )
        self._target_color = plugin_config.get("target_color", "all")
        self._min_area = int(plugin_config.get("min_area", 300))

        self._lock = threading.Lock()
        self._frame_condition = threading.Condition(self._lock)
        self._node = None
        self._subscription = None
        self._image_pub = None
        self._data_pub = None
        self._latest_result = None
        self._latest_jpeg = None
        self._last_frame_at = 0.0
        self._last_error = None
        self._last_processed_at = 0.0

        base = f"/{self._namespace}/{self.PREFIX}" if self._namespace else f"/{self.PREFIX}"
        self._outputs = [
            {"topic": f"{base}/overlay", "format": "image/jpeg", "desc": "标注了颜色物体的预览图像"},
            {"topic": f"{base}/objects", "format": "data/json", "desc": "检测到的各颜色物体位置与尺寸 JSON"},
        ]

    def get_tool(self):
        """返回在 Agent Core 和 Motus 注册的 Tool 描述字典。"""
        return {
            "name": self.PREFIX,
            "type": "processor",
            "multiInstance": False,
            "description": "基于色彩空间检测画面中的彩色或黑色物体（如红色、黄色、黑色、蓝色、绿色等），返回目标物体的中心坐标、尺寸和边界框。用于按颜色寻找或分拣物体；纯视觉处理，不控制机械臂运动。",
            "topic_in": [{"format": "image/jpeg", "desc": "RealSense 相机 RGB 图像流"}],
            "topic_out": self._outputs,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["detect", "classify", "config", "info", "start", "stop"],
                        "description": "操作动作：detect（按颜色检测物体）、classify（列出画面中所有颜色物体）、config（配置参数）、info（查看状态）、start（启动订阅）、stop（停止订阅）",
                    },
                    "color": {
                        "type": "string",
                        "enum": list(SUPPORTED_COLORS),
                        "default": "all",
                        "description": "要筛选的目标颜色（如 red, yellow, black, all 等）",
                    },
                    "min_area": {
                        "type": "integer",
                        "minimum": 50,
                        "default": 300,
                        "description": "物体最小像素面积过滤阈值",
                    },
                    "input_topic": {
                        "type": "string",
                        "description": "RGB 图像输入 ROS2 话题",
                    },
                    "timeout_s": {
                        "type": "number",
                        "minimum": 0.1,
                        "maximum": 30.0,
                        "default": 3.0,
                        "description": "等待新图像帧的超时时间（秒）",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "detect": {
                        "params": ["color", "min_area", "timeout_s"],
                        "description": "检测画面中指定颜色或全部颜色的物体中心点与尺寸",
                    },
                    "classify": {
                        "params": ["min_area", "timeout_s"],
                        "description": "列出画面中检测到的所有颜色分类与物体位置",
                    },
                    "config": {
                        "params": ["input_topic", "color", "min_area"],
                        "description": "配置卡片参数（图像输入话题、目标颜色与过滤面积）",
                    },
                    "info": {
                        "params": [],
                        "description": "查看颜色卡片状态与最新检测到的物体列表",
                    },
                    "start": {
                        "params": ["input_topic"],
                        "description": "启动图像订阅并开启持续颜色检测",
                    },
                    "stop": {
                        "params": [],
                        "description": "停止图像订阅并释放资源",
                    },
                },
            },
            "configSchema": {
                "type": "object",
                "properties": {
                    "input_topic": {
                        "type": "string",
                        "default": "",
                        "description": "默认 RGB 图像话题",
                    },
                    "target_color": {
                        "type": "string",
                        "enum": list(SUPPORTED_COLORS),
                        "default": "all",
                        "description": "默认关注的目标颜色",
                    },
                },
            },
        }

    def start(self):
        return self._start_subscription(self._input_topic)

    def stop(self):
        with self._frame_condition:
            if self._subscription is not None and self._node is not None:
                self._node.destroy_subscription(self._subscription)
                self._subscription = None
            if self._node is not None and self._executor is not None:
                try:
                    self._executor.remove_node(self._node)
                except Exception:
                    pass
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
                self._node = None
                self._image_pub = None
                self._data_pub = None
            return {"ok": True, "state": "idle", "status": "idle"}

    def _start_subscription(self, topic: str):
        with self._frame_condition:
            if not topic:
                raise ValueError("图像输入话题 input_topic 不能为空")

            if Node is not object:
                if self._node is None:
                    node_name = f"rm75_{self.PREFIX}"
                    context = getattr(self._executor, "context", None)
                    node = Node(node_name, context=context)
                    self._node = node
                    base = f"/{self._namespace}/{self.PREFIX}" if self._namespace else f"/{self.PREFIX}"
                    self._image_pub = node.create_publisher(CompressedImage, f"{base}/overlay", 1)
                    self._data_pub = node.create_publisher(String, f"{base}/objects", 1)
                    self._subscription = node.create_subscription(
                        CompressedImage, topic, self._on_frame, qos_profile_sensor_data
                    )
                    if self._executor is not None:
                        self._executor.add_node(node)
                elif topic != self._input_topic:
                    if self._subscription is not None:
                        self._node.destroy_subscription(self._subscription)
                    self._subscription = self._node.create_subscription(
                        CompressedImage, topic, self._on_frame, qos_profile_sensor_data
                    )

            self._input_topic = topic
            return {
                "ok": True,
                "state": "running",
                "status": "ready",
                "ready": True,
                "input_topic": topic,
                "topic_out": self._outputs,
            }

    def dispatch(self, action, args):
        if action == "config":
            topic = str(args.get("input_topic") or self._input_topic).strip()
            color = str(args.get("color") or args.get("target_color") or self._target_color).strip()
            if color:
                self._target_color = color
            if "min_area" in args:
                self._min_area = int(args["min_area"])
            if topic:
                old_topic = self._input_topic
                self._input_topic = topic
                with self._frame_condition:
                    if self._subscription is not None and self._node is not None and topic != old_topic:
                        self._node.destroy_subscription(self._subscription)
                        self._subscription = self._node.create_subscription(
                            CompressedImage, topic, self._on_frame, qos_profile_sensor_data
                        )
            info = self._info_action()
            info["ok"] = True
            info["status"] = "ready"
            info["ready"] = True
            return info

        if action == "start":
            topic = str(args.get("input_topic") or self._input_topic).strip()
            return self._start_subscription(topic)

        if action == "stop":
            return self.stop()

        if action in ("detect", "classify"):
            color = str(args.get("color") or self._target_color).strip().lower()
            min_area = int(args.get("min_area", self._min_area))
            timeout_s = float(args.get("timeout_s", 3.0))
            return self._detect_action(color, min_area, timeout_s)

        if action == "info":
            return self._info_action()

        raise ValueError(f"不支持的动作: {action}。支持动作: config, detect, classify, info, start, stop")

    def _detect_action(self, color, min_area, timeout_s):
        with self._frame_condition:
            if self._subscription is None and Node is not object and self._executor is not None:
                try:
                    self._start_subscription(self._input_topic)
                except Exception:
                    pass

            now = time.monotonic()
            frame_stale = (now - self._last_frame_at > 2.0) if self._last_frame_at else True
            if (self._latest_result is None or frame_stale) and timeout_s > 0:
                self._frame_condition.wait(timeout_s)

            if self._latest_result is None:
                return {
                    "status": "timeout",
                    "success": False,
                    "found": False,
                    "error": "在指定超时时间内未接收到相机图像帧",
                    "detected_colors": [],
                    "object_count": 0,
                    "objects": [],
                }

            all_objects = list(self._latest_result.get("objects", []))
            # 过滤面积
            objects = [obj for obj in all_objects if obj["area_px"] >= min_area]

            if color and color != "all":
                matched = [obj for obj in objects if obj["color"] == color]
                found = len(matched) > 0
                target = matched[0] if found else None
                return {
                    "status": "success",
                    "success": True,
                    "found": found,
                    "target_color": color,
                    "target_object": target,
                    "center": target["center_normalized"] if target else None,
                    "center_normalized": target["center_normalized"] if target else None,
                    "center_px": target["center_px"] if target else None,
                    "bbox_px": target["bbox_px"] if target else None,
                    "bbox_normalized": target["bbox_normalized"] if target else None,
                    "objects": matched,
                    "object_count": len(matched),
                    "all_detected_colors": sorted(list(set(obj["color"] for obj in objects))),
                    "image_width": self._latest_result.get("image_width"),
                    "image_height": self._latest_result.get("image_height"),
                    "frame_age_s": round(time.monotonic() - self._last_frame_at, 3) if self._last_frame_at else None,
                }

            return {
                "status": "success",
                "success": True,
                "found": len(objects) > 0,
                "detected_colors": sorted(list(set(obj["color"] for obj in objects))),
                "object_count": len(objects),
                "objects": objects,
                "image_width": self._latest_result.get("image_width"),
                "image_height": self._latest_result.get("image_height"),
                "frame_age_s": round(time.monotonic() - self._last_frame_at, 3) if self._last_frame_at else None,
            }

    def _info_action(self):
        with self._lock:
            age = (
                time.monotonic() - self._last_frame_at
                if self._last_frame_at
                else None
            )
            latest = dict(self._latest_result) if self._latest_result else None
            is_running = self._subscription is not None
            return {
                "ok": True,
                "state": "running" if is_running else "idle",
                "status": "ready",
                "ready": True,
                "readiness": "ready",
                "card_name": self.PREFIX,
                "input_topic": self._input_topic,
                "topic_in": [{"topic": self._input_topic, "format": "image/jpeg"}],
                "topic_out": self._outputs,
                "target_color": self._target_color,
                "latest": latest,
                "detected_colors": latest.get("detected_colors", []) if latest else [],
                "object_count": latest.get("object_count", 0) if latest else 0,
                "latest_frame_age_s": round(age, 3) if age is not None else None,
                "last_error": self._last_error,
            }

    def _on_frame(self, message):
        now = time.monotonic()
        with self._lock:
            if self._node is None or (now - self._last_processed_at < 0.1):
                return
            self._last_processed_at = now
            target_color = self._target_color
            min_area = self._min_area

        try:
            result, jpeg = process_image(message, target_color=target_color, min_area=min_area)
            image_message = CompressedImage()
            image_message.header = getattr(message, "header", image_message.header)
            image_message.format = "jpeg"
            image_message.data = jpeg

            json_message = String()
            json_message.data = json.dumps(result, ensure_ascii=False)

            with self._frame_condition:
                if self._node is None:
                    return
                self._latest_result = result
                self._latest_jpeg = jpeg
                self._last_frame_at = now
                self._last_error = None
                self._frame_condition.notify_all()

                if self._image_pub is not None:
                    self._image_pub.publish(image_message)
                if self._data_pub is not None:
                    self._data_pub.publish(json_message)
        except Exception as exc:
            with self._frame_condition:
                self._last_error = str(exc)
                self._frame_condition.notify_all()
