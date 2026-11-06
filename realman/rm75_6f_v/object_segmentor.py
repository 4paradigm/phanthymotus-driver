"""睿曼 RM75 视觉物体分割、边缘提取与尺寸估计卡片（纯视觉处理，不控制机械臂运动）。

功能：
1. 订阅 RealSense 相机 RGB 图像流（CompressedImage 或 Image）；
2. 执行前景物体轮廓分割，提取画面中独立物体的多边形边缘与边界点；
3. 计算每个物体的几何中心点（center_px, center_normalized）；
4. 精确测量并判断物体的大小与轮廓：
   - 像素尺寸（宽 width_px, 高 height_px, 面积 area_px, 周长 perimeter_px）
   - 归一化比例（width_normalized, height_normalized, area_normalized）
   - 尺寸等级划分：small（小型）、medium（中型）、large（大型）
   - 边缘多边形顶点序列（edge_points_normalized, edge_points_px）
   - 识别物体的长宽比（aspect_ratio）与主要颜色（dominant_color）；
5. 提供可调用的动作接口（segment / measure_size / detect_objects / info / config / start / stop），
   上层 AgentCore / 决策核心可直接调用它回答诸如“你可以识别物体的大小吗”、“画面中物体的边缘和尺寸是多少”等问题；
6. 在输出话题发布带分割轮廓与尺寸标签的预览图（overlay）和物体数据 JSON（objects）。
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


def estimate_dominant_color(cv2_module, roi_bgr):
    """估计感兴趣区域（ROI）的主导颜色名称。"""
    import numpy as np

    if roi_bgr is None or roi_bgr.size == 0:
        return "unknown"

    hsv = cv2_module.cvtColor(roi_bgr, cv2_module.COLOR_BGR2HSV)
    gray = cv2_module.cvtColor(roi_bgr, cv2_module.COLOR_BGR2GRAY)

    mean_v = float(np.mean(hsv[:, :, 2]))
    mean_s = float(np.mean(hsv[:, :, 1]))
    mean_h = float(np.mean(hsv[:, :, 0]))
    mean_gray = float(np.mean(gray))

    # 判断黑色（低明度）
    if mean_v < 75 and mean_gray < 75:
        return "black"

    # 判断白色/高亮灰色
    if mean_v > 190 and mean_s < 40:
        return "white"

    if mean_s < 45:
        return "gray"

    # 根据色相判断
    if (mean_h <= 10 or mean_h >= 165) and mean_s >= 60:
        return "red"
    elif 18 <= mean_h <= 40:
        return "yellow"
    elif 40 < mean_h <= 85:
        return "green"
    elif 85 < mean_h <= 135:
        return "blue"
    elif 10 < mean_h < 18:
        return "orange"

    return "colored"


def segment_objects(image_bgr, min_area=500, max_area=None,
                    x_compensation_mm=30.0, y_compensation_mm=-75.0, camera_z_mm=450.0):
    """在 BGR 图像上执行物体轮廓分割，并计算物体的尺寸、大小等级与边缘。

    坐标系同步说明：
    - position: VOP 统一归一化居中坐标系 [-1.0, 1.0]，图像中心为 [0, 0]，左上角 [-1, -1]，右下角 [1, 1]。
      与 vision_pick_and_drop 完全对齐，可直接作为抓取点传入。
    - compensated_position: 已叠加机械臂物理安装位姿补偿 (x_compensation_mm, y_compensation_mm) 的预补偿归一化坐标。
    - center_px: 原始像素坐标 [cx, cy]。
    - center_normalized: 传统左上角归一化坐标 [0.0, 1.0]。

    返回:
        list[dict]: 检测到的各物体详情列表，包含中心点、尺寸面积、大小等级与边缘点。
    """
    import cv2
    import numpy as np

    height, width = image_bgr.shape[:2]
    total_image_area = float(max(1, width * height))
    half_w = float(max(1, width)) / 2.0
    half_h = float(max(1, height)) / 2.0
    if max_area is None:
        max_area = total_image_area * 0.85

    # 计算机械臂基座补偿在相机像平面上的像素偏移 (俯视标准工作位姿 Z≈450mm)
    z_m = max(50.0, float(camera_z_mm))
    delta_px = -(float(x_compensation_mm) / z_m) * float(width)
    delta_py = (float(y_compensation_mm) / z_m) * float(height)

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # 1. 结合自适应阈值与 Otsu 提取前景物体
    thresh_otsu = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
    thresh_adapt = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                         cv2.THRESH_BINARY_INV, 15, 4)
    combined = cv2.bitwise_or(thresh_otsu, thresh_adapt)

    # 形态学闭运算连接物体内部空隙
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    closed = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kernel, iterations=2)
    opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN, kernel, iterations=1)

    # 2. 轮廓提取
    contours, _ = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    objects = []
    obj_idx = 1

    for c in contours:
        area = float(cv2.contourArea(c))
        if area < min_area or area > max_area:
            continue

        x, y, w, h = cv2.boundingRect(c)
        # 旋转外接矩形以获取更精确的长和宽
        rect = cv2.minAreaRect(c)
        box_w, box_h = sorted([rect[1][0], rect[1][1]])

        # 几何中心点（通过图像矩或边界框）
        moments = cv2.moments(c)
        if moments["m00"] > 0:
            cx = float(moments["m10"] / moments["m00"])
            cy = float(moments["m01"] / moments["m00"])
        else:
            cx = float(x + w / 2.0)
            cy = float(y + h / 2.0)

        # 1. VOP 标准居中归一化坐标 [-1.0, 1.0]
        pos_x = max(-1.0, min(1.0, round(float((cx - half_w) / half_w), 4)))
        pos_y = max(-1.0, min(1.0, round(float((cy - half_h) / half_h), 4)))

        # 2. 考虑基座补偿后的像平面与归一化坐标
        comp_cx = round(cx + delta_px, 2)
        comp_cy = round(cy + delta_py, 2)
        comp_pos_x = max(-1.0, min(1.0, round(float((comp_cx - half_w) / half_w), 4)))
        comp_pos_y = max(-1.0, min(1.0, round(float((comp_cy - half_h) / half_h), 4)))

        perimeter = float(cv2.arcLength(c, True))

        # 尺寸级别划分
        area_ratio = area / total_image_area
        if area_ratio < 0.015:
            size_level = "small"
            size_name_cn = "小型"
        elif area_ratio < 0.06:
            size_level = "medium"
            size_name_cn = "中型"
        else:
            size_level = "large"
            size_name_cn = "大型"

        # 边缘多边形简化顶点序列
        epsilon = 0.012 * perimeter
        approx = cv2.approxPolyDP(c, epsilon, True)
        edge_pts_px = [[int(pt[0][0]), int(pt[0][1])] for pt in approx]
        edge_pts_norm = [[round(pt[0][0] / width, 4), round(pt[0][1] / height, 4)] for pt in approx]

        # 提取物体 ROI 计算主导颜色
        roi = image_bgr[max(0, y):min(height, y + h), max(0, x):min(width, x + w)]
        dominant_color = estimate_dominant_color(cv2, roi)

        aspect_ratio = round(float(w) / float(max(1, h)), 2)

        objects.append({
            "id": obj_idx,
            "name": f"object_{obj_idx}",
            "confidence": 0.90,
            "position": [pos_x, pos_y],
            "compensated_position": [comp_pos_x, comp_pos_y],
            "center": [pos_x, pos_y],
            "center_vop": [pos_x, pos_y],
            "center_px": [round(cx, 2), round(cy, 2)],
            "center_normalized": [pos_x, pos_y],
            "topleft_ratio_uv": [round(cx / width, 4), round(cy / height, 4)],
            "size_level": size_level,
            "size_description": f"{size_name_cn} (宽 {int(w)}px, 高 {int(h)}px, 面积 {int(area)}px)",
            "width_px": int(w),
            "height_px": int(h),
            "area_px": int(area),
            "perimeter_px": round(perimeter, 2),
            "aspect_ratio": aspect_ratio,
            "width_normalized": round(w / width, 4),
            "height_normalized": round(h / height, 4),
            "area_normalized": round(area_ratio, 4),
            "bbox_px": [int(x), int(y), int(w), int(h)],
            "bbox": [int(x), int(y), int(x + w), int(y + h)],
            "bbox_normalized": [
                round(x / width, 4),
                round(y / height, 4),
                round(w / width, 4),
                round(h / height, 4),
            ],
            "dominant_color": dominant_color,
            "edge_points_count": len(edge_pts_px),
            "edge_points_normalized": edge_pts_norm,
            "edge_points_px": edge_pts_px,
            "compensation": {
                "x_compensation_mm": float(x_compensation_mm),
                "y_compensation_mm": float(y_compensation_mm),
            },
        })
        obj_idx += 1

    # 按面积从大到小排序
    objects.sort(key=lambda item: item["area_px"], reverse=True)
    # 重排 id 与 name
    for i, obj in enumerate(objects, 1):
        obj["id"] = i
        obj["name"] = f"object_{i}"

    return objects


def annotate_segmented_image(image_bgr, objects):
    """在图像上绘制分割出的物体边缘轮廓、外接框、中心十字线和尺寸信息。"""
    import cv2
    import numpy as np

    canvas = image_bgr.copy()
    for obj in objects:
        x, y, w, h = obj["bbox_px"]
        cx, cy = int(round(obj["center_px"][0])), int(round(obj["center_px"][1]))

        # 1. 绘制边缘多边形轮廓（亮绿色）
        pts = np.array(obj["edge_points_px"], dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [pts], isClosed=True, color=(0, 255, 128), thickness=2)

        # 2. 绘制外接矩形框（虚/细蓝色线）
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (255, 160, 0), 1)

        # 3. 绘制中心十字线与红点
        cv2.circle(canvas, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(canvas, (cx - 6, cy), (cx + 6, cy), (0, 0, 255), 1)
        cv2.line(canvas, (cx, cy - 6), (cx, cy + 6), (0, 0, 255), 1)

        # 4. 绘制尺寸标签（显示尺寸等级、宽高像素和主色）
        label = f"#{obj['id']} [{obj['size_level']}] {w}x{h} ({obj['dominant_color']})"
        ty = max(16, y - 6)
        cv2.putText(canvas, label, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    return canvas


def process_image(msg, min_area=500, max_area=None,
                  x_compensation_mm=30.0, y_compensation_mm=-75.0, camera_z_mm=450.0):
    """解码图像并执行分割与尺寸计算。"""
    import cv2
    import numpy as np

    t0 = time.time()
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
    objects = segment_objects(
        image, min_area=min_area, max_area=max_area,
        x_compensation_mm=x_compensation_mm, y_compensation_mm=y_compensation_mm,
        camera_z_mm=camera_z_mm,
    )
    t_done = time.time()

    result = {
        "status": "success",
        "object_count": len(objects),
        "count": len(objects),
        "objects": objects,
        "coordinate_system": "vop_centered_normalized_minus1_to_plus1",
        "compensation": {
            "x_compensation_mm": float(x_compensation_mm),
            "y_compensation_mm": float(y_compensation_mm),
        },
        "image_width": width,
        "image_height": height,
        "timestamp": t_done,
        "latency_ms": round((t_done - t0) * 1000, 2),
    }

    annotated = annotate_segmented_image(image, objects)
    success, jpeg_buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    jpeg_bytes = jpeg_buf.tobytes() if success else b""
    return result, jpeg_bytes


class ObjectSegmentorPlugin:
    """睿曼 RM75 物体轮廓分割与尺寸/边缘检测卡片插件。"""

    PREFIX = "object_segmentor"

    def __init__(self, plugin_config, namespace, executor):
        self._namespace = namespace
        self._executor = executor
        self._input_topic = plugin_config.get(
            "input_topic",
            f"/{namespace}/ext_camera/default/rgb" if namespace else "/ext_camera/default/rgb"
        )
        self._min_area = int(plugin_config.get("min_area", 500))
        self._x_compensation_mm = float(plugin_config.get("x_compensation_mm", 30.0))
        self._y_compensation_mm = float(plugin_config.get("y_compensation_mm", -75.0))
        self._camera_z_mm = float(plugin_config.get("camera_z_mm", 450.0))

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
            {"topic": f"{base}/overlay", "format": "image/jpeg", "desc": "标注了物体边缘轮廓与尺寸的预览图"},
            {"topic": f"{base}/objects", "format": "data/json", "desc": "分割出的物体尺寸、面积与边缘点 JSON（VOP 统一格式）"},
        ]

    def get_tool(self):
        """返回在 Agent Core 和 Motus 注册的 Tool 描述字典。"""
        return {
            "name": self.PREFIX,
            "type": "processor",
            "multiInstance": False,
            "description": "分割画面中的物体轮廓，检测物体边缘、几何中心（与 VOP 统一的 [-1, 1] 居中坐标）、宽度、高度、面积与大小等级（小型/中型/大型），并提供物理位姿补偿坐标。可直接用于抓取与分拣任务。",
            "topic_in": [{"format": "image/jpeg", "desc": "RealSense 相机 RGB 图像流"}],
            "topic_out": self._outputs,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["segment", "measure_size", "detect_objects", "config", "info", "start", "stop"],
                        "description": "操作动作：segment（分割物体并获取尺寸大小）、measure_size（测量物体大小）、config（配置参数）、info（查看状态）、start（启动订阅）、stop（停止订阅）",
                    },
                    "min_area": {
                        "type": "integer",
                        "minimum": 50,
                        "default": 500,
                        "description": "物体最小像素面积阈值",
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
                    "apply_compensation": {
                        "type": "boolean",
                        "default": False,
                        "description": "是否在返回的 position 主字段中直接应用物理位姿安装补偿；默认 False（返回 VOP 标准像平面归一化居中坐标，由下游 pick_place 计算补偿）",
                    },
                    "x_compensation_mm": {
                        "type": "number",
                        "default": 30.0,
                        "description": "基座坐标系 X 补偿（mm），用于校准相机与机械臂末端水平偏置",
                    },
                    "y_compensation_mm": {
                        "type": "number",
                        "default": -75.0,
                        "description": "基座坐标系 Y 补偿（mm），用于校准相机与机械臂末端水平偏置",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "segment": {
                        "params": ["min_area", "timeout_s", "apply_compensation", "x_compensation_mm", "y_compensation_mm"],
                        "description": "分割画面中所有物体，获取其大小等级、宽度、高度、面积、VOP 坐标与边缘轮廓",
                    },
                    "measure_size": {
                        "params": ["min_area", "timeout_s", "apply_compensation", "x_compensation_mm", "y_compensation_mm"],
                        "description": "测量画面中物体的大小尺寸、中心坐标与轮廓边界",
                    },
                    "detect_objects": {
                        "params": ["min_area", "timeout_s", "apply_compensation", "x_compensation_mm", "y_compensation_mm"],
                        "description": "检测画面中所有独立物体的位置、尺寸与主要颜色",
                    },
                    "config": {
                        "params": ["input_topic", "min_area", "x_compensation_mm", "y_compensation_mm"],
                        "description": "配置卡片参数（图像输入话题、最小检测面积与位姿补偿）",
                    },
                    "info": {
                        "params": [],
                        "description": "查看分割卡片状态与最新检测到的物体大小列表",
                    },
                    "start": {
                        "params": ["input_topic"],
                        "description": "启动图像订阅并开启持续分割",
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
                    "min_area": {
                        "type": "integer",
                        "default": 500,
                        "description": "默认最小检测像素面积",
                    },
                    "x_compensation_mm": {
                        "type": "number",
                        "default": 30.0,
                        "description": "基坐标 X 补偿（mm）",
                    },
                    "y_compensation_mm": {
                        "type": "number",
                        "default": -75.0,
                        "description": "基坐标 Y 补偿（mm）",
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
            if "min_area" in args:
                self._min_area = int(args["min_area"])
            if "x_compensation_mm" in args:
                self._x_compensation_mm = float(args["x_compensation_mm"])
            if "y_compensation_mm" in args:
                self._y_compensation_mm = float(args["y_compensation_mm"])
            if "camera_z_mm" in args:
                self._camera_z_mm = float(args["camera_z_mm"])
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

        if action in ("segment", "measure_size", "detect_objects"):
            min_area = int(args.get("min_area", self._min_area))
            timeout_s = float(args.get("timeout_s", 3.0))
            apply_comp = bool(args.get("apply_compensation", False))
            x_comp = float(args.get("x_compensation_mm", self._x_compensation_mm))
            y_comp = float(args.get("y_compensation_mm", self._y_compensation_mm))
            return self._segment_action(min_area, timeout_s, apply_comp, x_comp, y_comp)

        if action == "info":
            return self._info_action()

        raise ValueError(f"不支持的动作: {action}。支持动作: config, segment, measure_size, detect_objects, info, start, stop")

    def _segment_action(self, min_area, timeout_s, apply_compensation=False,
                        x_compensation_mm=None, y_compensation_mm=None):
        if x_compensation_mm is None:
            x_compensation_mm = self._x_compensation_mm
        if y_compensation_mm is None:
            y_compensation_mm = self._y_compensation_mm

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
                    "object_count": 0,
                    "count": 0,
                    "objects": [],
                    "coordinate_system": "vop_centered_normalized_minus1_to_plus1",
                    "compensation": {
                        "x_compensation_mm": x_compensation_mm,
                        "y_compensation_mm": y_compensation_mm,
                        "applied": apply_compensation,
                    },
                }

            all_objects = list(self._latest_result.get("objects", []))
            objects = [obj for obj in all_objects if obj["area_px"] >= min_area]
            first_target = objects[0] if objects else None
            pos = (first_target["compensated_position"] if apply_compensation else first_target["position"]) if first_target else None

            return {
                "status": "success",
                "success": True,
                "found": len(objects) > 0,
                "position": pos,
                "target_position": pos,
                "compensated_position": first_target["compensated_position"] if first_target else None,
                "center": pos,
                "center_normalized": first_target["center_normalized"] if first_target else None,
                "center_px": first_target["center_px"] if first_target else None,
                "compensated_center_px": first_target["compensated_center_px"] if first_target else None,
                "object_count": len(objects),
                "count": len(objects),
                "objects": objects,
                "summary": f"共检测到 {len(objects)} 个物体，包含尺寸与边缘信息",
                "coordinate_system": "vop_centered_normalized_minus1_to_plus1",
                "compensation": {
                    "x_compensation_mm": x_compensation_mm,
                    "y_compensation_mm": y_compensation_mm,
                    "applied": apply_compensation,
                },
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
                "min_area": self._min_area,
                "x_compensation_mm": self._x_compensation_mm,
                "y_compensation_mm": self._y_compensation_mm,
                "coordinate_system": "vop_centered_normalized_minus1_to_plus1",
                "latest": latest,
                "object_count": latest.get("object_count", 0) if latest else 0,
                "count": latest.get("count", 0) if latest else 0,
                "latest_frame_age_s": round(age, 3) if age is not None else None,
                "last_error": self._last_error,
            }

    def _on_frame(self, message):
        now = time.monotonic()
        with self._lock:
            if self._node is None or (now - self._last_processed_at < 0.1):
                return
            self._last_processed_at = now
            min_area = self._min_area
            x_comp = self._x_compensation_mm
            y_comp = self._y_compensation_mm
            z_m = self._camera_z_mm

        try:
            result, jpeg = process_image(
                message,
                min_area=min_area,
                x_compensation_mm=x_comp,
                y_compensation_mm=y_comp,
                camera_z_mm=z_m,
            )
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
