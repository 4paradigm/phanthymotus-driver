"""睿曼 RM75 视觉 ArUco 标记码检测卡片（纯视觉处理，不控制机械臂运动）。

功能：
1. 订阅 RealSense 相机 RGB 图像流（CompressedImage 或 Image）；
2. 使用 OpenCV ArUco 检测画面中所有可见的 ArUco 标记码；
3. 提取每个码的 ID 数值与几何中心点坐标（包含像素中心坐标 center_px 与 0~1 归一化坐标 center_normalized）；
4. 提供可调用的动作接口（detect / get_marker_center / info），供下游分拣任务直接调用获取标记中心位置；
5. 在输出话题上发布标注了中心点与 ID 的预览图像及 JSON 数据。

注意：本卡片仅提取标记码 ID 与中心位置坐标，不包含待分拣区/分拣区等额外规则。
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


# 支持的常用 ArUco 字典类型
DICTIONARIES = (
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_4X4_1000",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_5X5_1000",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_6X6_250",
    "DICT_6X6_1000",
    "DICT_7X7_50",
    "DICT_ARUCO_ORIGINAL",
)

DEFAULT_DICTIONARY = "DICT_4X4_50"


def get_aruco_dict(cv2_module, dict_name: str):
    """获取指定名称的 OpenCV ArUco 字典对象。"""
    aruco = cv2_module.aruco
    if hasattr(aruco, dict_name):
        return aruco.getPredefinedDictionary(getattr(aruco, dict_name))
    return aruco.getPredefinedDictionary(aruco.DICT_4X4_50)


def extract_markers_from_corners(corners, ids, image_width: int, image_height: int,
                                 x_compensation_mm: float = 30.0, y_compensation_mm: float = -75.0,
                                 camera_z_mm: float = 450.0):
    """从角点坐标与 ID 列表中提取每个标记码的几何中心点与位置坐标。

    坐标系同步说明：
    - position: VOP 统一归一化居中坐标系 [-1.0, 1.0]，图像中心为 [0, 0]，左上角 [-1, -1]，右下角 [1, 1]。
      与 vision_pick_and_drop 完全对齐，可直接作为抓取点传入。
    - compensated_position: 已叠加机械臂物理安装位姿补偿 (x_compensation_mm, y_compensation_mm) 的预补偿归一化坐标。
    - center_px: 原始像素坐标 [cx, cy]。
    - center_normalized: 传统左上角归一化坐标 [0.0, 1.0]。

    参数:
        corners: OpenCV 检测出的角点列表（numpy ndarray 或嵌套列表）
        ids: OpenCV 检测出的标记 ID 列表
        image_width: 画面宽度（像素）
        image_height: 画面高度（像素）
        x_compensation_mm: 基座 X 补偿（mm）
        y_compensation_mm: 基座 Y 补偿（mm）
        camera_z_mm: 相机观察高度（mm，默认 450mm）

    返回:
        list[dict]: 提取出的标记列表，每个标记包含:
            - id: 标记 ID (int)
            - name: "aruco_{id}" (str)
            - confidence: 1.0 (float)
            - position: [x, y] VOP 归一化居中坐标 (-1.0~1.0)
            - compensated_position: [x, y] 叠加基座位姿补偿的归一化坐标 (-1.0~1.0)
            - center: [x, y] (与 position 对齐)
            - center_normalized: [x, y] 传统左上角归一化坐标 (0.0~1.0)
            - center_px: [x, y] 像素中心坐标
            - compensated_center_px: [x, y] 补偿后像素中心坐标
            - corners_normalized: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]] 归一化角点
            - corners_px: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]] 像素角点
            - compensation: 补偿参数字典
    """
    markers = []
    if ids is None or len(ids) == 0:
        return markers

    width = float(max(1, image_width))
    height = float(max(1, image_height))
    half_w = width / 2.0
    half_h = height / 2.0

    # 计算机械臂基座补偿在相机像平面上的像素偏移 (俯视标准工作位姿 Z≈450mm)
    z_m = max(50.0, float(camera_z_mm))
    delta_px = -(float(x_compensation_mm) / z_m) * width
    delta_py = (float(y_compensation_mm) / z_m) * height

    for i in range(len(ids)):
        raw_id = ids[i]
        marker_id = int(raw_id[0] if hasattr(raw_id, "__getitem__") else raw_id)
        raw_corners = corners[i]
        # 处理形状为 (1, 4, 2) 或 (4, 2) 或纯嵌套列表
        if hasattr(raw_corners, "shape"):
            c = raw_corners[0] if len(raw_corners.shape) == 3 else raw_corners
            pts = [[float(p[0]), float(p[1])] for p in c]
        elif isinstance(raw_corners, (list, tuple)) and len(raw_corners) == 1 and isinstance(raw_corners[0], (list, tuple)):
            pts = [[float(p[0]), float(p[1])] for p in raw_corners[0]]
        else:
            pts = [[float(p[0]), float(p[1])] for p in raw_corners]

        if len(pts) != 4:
            continue

        # 计算几何中心点（四个角点坐标的算术平均值）
        cx = sum(p[0] for p in pts) / 4.0
        cy = sum(p[1] for p in pts) / 4.0

        # 1. VOP 标准居中归一化坐标 [-1.0, 1.0]
        pos_x = max(-1.0, min(1.0, round(float((cx - half_w) / half_w), 4)))
        pos_y = max(-1.0, min(1.0, round(float((cy - half_h) / half_h), 4)))

        # 2. 传统左上角归一化坐标 (0.0~1.0)
        norm_x = round(cx / width, 4)
        norm_y = round(cy / height, 4)

        # 3. 考虑基座补偿后的像平面与归一化坐标
        comp_cx = round(cx + delta_px, 2)
        comp_cy = round(cy + delta_py, 2)
        comp_pos_x = max(-1.0, min(1.0, round(float((comp_cx - half_w) / half_w), 4)))
        comp_pos_y = max(-1.0, min(1.0, round(float((comp_cy - half_h) / half_h), 4)))

        corners_px = [[round(p[0], 2), round(p[1], 2)] for p in pts]
        corners_norm = [
            [round(p[0] / width, 4), round(p[1] / height, 4)]
            for p in pts
        ]

        markers.append({
            "id": marker_id,
            "name": f"aruco_{marker_id}",
            "confidence": 1.0,
            "position": [pos_x, pos_y],
            "compensated_position": [comp_pos_x, comp_pos_y],
            "center": [pos_x, pos_y],
            "center_vop": [pos_x, pos_y],
            "center_normalized": [norm_x, norm_y],
            "center_px": [round(cx, 2), round(cy, 2)],
            "compensated_center_px": [comp_cx, comp_cy],
            "corners_normalized": corners_norm,
            "corners_px": corners_px,
            "compensation": {
                "x_compensation_mm": float(x_compensation_mm),
                "y_compensation_mm": float(y_compensation_mm),
            },
        })

    # 按 marker_id 升序排序输出
    markers.sort(key=lambda m: m["id"])
    return markers


def detect_markers(image_bgr, dictionary_name: str = DEFAULT_DICTIONARY,
                   x_compensation_mm: float = 30.0, y_compensation_mm: float = -75.0,
                   camera_z_mm: float = 450.0):
    """直接在 BGR 图像上运行 ArUco 检测并提取中心点坐标（供 Python 脚本或方法直接调用）。

    参数:
        image_bgr: OpenCV numpy ndarray (BGR 图像)
        dictionary_name: ArUco 字典名称，默认 DICT_4X4_50
        x_compensation_mm: 基座 X 补偿（mm）
        y_compensation_mm: 基座 Y 补偿（mm）
        camera_z_mm: 相机观察高度（mm）

    返回:
        list[dict]: 提取出的标记列表，每项包含 id, position, center_normalized, center_px 等
    """
    import cv2

    height, width = image_bgr.shape[:2]
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    aruco_dict = get_aruco_dict(cv2, dictionary_name)

    # 兼容 OpenCV 4.7+ 的 ArucoDetector API 以及旧版本 detectMarkers
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        detector_params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)

    return extract_markers_from_corners(
        corners, ids, width, height,
        x_compensation_mm=x_compensation_mm,
        y_compensation_mm=y_compensation_mm,
        camera_z_mm=camera_z_mm,
    )


def annotate_image(image_bgr, markers):
    """在图像上绘制 ArUco 标记的边框、中心十字线与 ID 文本标注（不绘制任何分拣区）。

    参数:
        image_bgr: OpenCV numpy ndarray (BGR)
        markers: 提取出的标记列表

    返回:
        numpy ndarray: 绘制完成的标注图像
    """
    import cv2
    import numpy as np

    canvas = image_bgr.copy()
    for marker in markers:
        # 1. 绘制标记四边多边形轮廓（绿色）
        pts = np.array(marker["corners_px"], dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(canvas, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # 2. 绘制中心点十字线与圆点（红色）
        cx, cy = int(round(marker["center_px"][0])), int(round(marker["center_px"][1]))
        cv2.circle(canvas, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(canvas, (cx - 8, cy), (cx + 8, cy), (0, 0, 255), 1)
        cv2.line(canvas, (cx, cy - 8), (cx, cy + 8), (0, 0, 255), 1)

        # 3. 绘制 ID 与 VOP 居中归一化坐标标签（[-1, 1] 范围，供机械臂直接使用）
        vop_pos = marker.get("position", marker.get("center", [0.0, 0.0]))
        text = f"ID:{marker['id']} ({vop_pos[0]:+.2f},{vop_pos[1]:+.2f})"
        tx = int(marker["corners_px"][0][0])
        ty = max(16, int(marker["corners_px"][0][1]) - 6)
        cv2.putText(canvas, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    return canvas


def process_image(msg, dictionary_name: str = DEFAULT_DICTIONARY,
                  x_compensation_mm: float = 30.0, y_compensation_mm: float = -75.0,
                  camera_z_mm: float = 450.0):
    """解码相机输入的图像消息，执行标记检测并提取几何中心坐标。

    参数:
        msg: CompressedImage / sensor_msgs.msg.Image / bytes / numpy.ndarray
        dictionary_name: ArUco 字典名称
        x_compensation_mm: 基座 X 补偿（mm）
        y_compensation_mm: 基座 Y 补偿（mm）
        camera_z_mm: 相机观察高度（mm）

    返回:
        tuple[dict, bytes]: (检测数据字典, 带中心点标注的 JPEG 字节流)
    """
    import cv2
    import numpy as np

    t0 = time.time()
    # 1. 图像解码与格式转换
    if isinstance(msg, (bytes, bytearray)):
        image = cv2.imdecode(np.frombuffer(msg, dtype=np.uint8), cv2.IMREAD_COLOR)
    elif hasattr(msg, "format") and hasattr(msg, "data"):
        # CompressedImage
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
    elif hasattr(msg, "width") and hasattr(msg, "height") and hasattr(msg, "data"):
        # raw sensor_msgs/Image
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

    # 2. 检测标记并提取中心坐标
    markers = detect_markers(
        image, dictionary_name,
        x_compensation_mm=x_compensation_mm,
        y_compensation_mm=y_compensation_mm,
        camera_z_mm=camera_z_mm,
    )
    t_done = time.time()

    result = {
        "status": "success",
        "detected_markers": markers,
        "detected_marker_ids": [m["id"] for m in markers],
        "marker_count": len(markers),
        "count": len(markers),
        "objects": markers,
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

    # 3. 渲染标注图像（轮廓与中心十字线）
    annotated = annotate_image(image, markers)
    success, jpeg_buf = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    jpeg_bytes = jpeg_buf.tobytes() if success else b""

    return result, jpeg_bytes


class ArucoDetectorPlugin:
    """睿曼 RM75 ArUco 标记检测卡片插件。

    负责：
    - 订阅 RealSense RGB 图像流并识别 ArUco 码中心点
    - 发布 /{namespace}/aruco_detector/overlay 预览图像
    - 发布 /{namespace}/aruco_detector/markers 检测结果 JSON
    - 发布 /{namespace}/aruco_detector/objects VOP 兼容格式的标记物体列表 JSON
    - 提供直接可调用的 detect / get_marker_center 动作接口
    """

    PREFIX = "aruco_detector"

    def __init__(self, plugin_config, namespace, executor, card_name=None):
        self._namespace = namespace
        self._executor = executor
        self.PREFIX = card_name or plugin_config.get("card_name", "aruco_detector")
        self._dictionary = plugin_config.get("dictionary", DEFAULT_DICTIONARY)
        self._input_topic = plugin_config.get(
            "input_topic",
            f"/{namespace}/ext_camera/default/rgb" if namespace else "/ext_camera/default/rgb"
        )
        self._x_compensation_mm = float(plugin_config.get("x_compensation_mm", 30.0))
        self._y_compensation_mm = float(plugin_config.get("y_compensation_mm", -75.0))
        self._camera_z_mm = float(plugin_config.get("camera_z_mm", 450.0))

        self._lock = threading.Lock()
        self._frame_condition = threading.Condition(self._lock)
        self._node = None
        self._subscription = None
        self._image_pub = None
        self._data_pub = None
        self._objects_pub = None
        self._latest_result = None
        self._latest_jpeg = None
        self._last_frame_at = 0.0
        self._last_error = None
        self._last_processed_at = 0.0

        base = f"/{self._namespace}/{self.PREFIX}" if self._namespace else f"/{self.PREFIX}"
        self._outputs = [
            {"topic": f"{base}/overlay", "format": "image/jpeg", "desc": "标注了 ArUco 码中心点的预览图像"},
            {"topic": f"{base}/markers", "format": "data/json", "desc": "检测到的 ArUco 标记 ID 与中心点位置 JSON"},
        ]

    def get_tool(self):
        """返回卡片在 Agent Core 和 Motus 中注册的 Tool 描述字典。"""
        return {
            "name": self.PREFIX,
            "type": "processor",
            "multiInstance": False,
            "description": "识别 RealSense 相机画面中的 ArUco 标记码，提取其 ID 数值与几何中心点坐标（与 VOP 统一的 [-1, 1] 居中坐标 position 及物理安装位姿补偿坐标）。可直接用于抓取与放置任务。",
            "topic_in": [{"format": "image/jpeg", "desc": "RealSense 相机 RGB 图像流"}],
            "topic_out": self._outputs,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["config", "start", "stop", "info", "detect", "get_marker_center"],
                        "description": "操作动作：config（配置参数）、start（启动订阅）、stop（停止订阅）、info（查看状态）、detect（获取所有码或指定码中心坐标）、get_marker_center（根据 ID 获取目标码中心）",
                    },
                    "marker_id": {
                        "type": "integer",
                        "minimum": 0,
                        "description": "可选的目标 ArUco 标记码 ID。指定时将精确匹配并返回该码的中心点坐标",
                    },
                    "timeout_s": {
                        "type": "number",
                        "minimum": 0.1,
                        "maximum": 30.0,
                        "default": 3.0,
                        "description": "等待新图像帧的超时时间（秒）",
                    },
                    "input_topic": {
                        "type": "string",
                        "description": "RGB 图像输入 ROS2 话题，默认为 ext_camera 的 RGB 输出",
                    },
                    "dictionary": {
                        "type": "string",
                        "enum": list(DICTIONARIES),
                        "default": DEFAULT_DICTIONARY,
                        "description": "使用的 ArUco 字典类型，默认 DICT_4X4_50",
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
                    "config": {
                        "params": ["input_topic", "dictionary", "x_compensation_mm", "y_compensation_mm"],
                        "description": "配置卡片参数（图像输入话题、字典类型与位姿补偿）",
                    },
                    "detect": {
                        "params": ["marker_id", "timeout_s", "apply_compensation", "x_compensation_mm", "y_compensation_mm"],
                        "description": "调用卡片识别画面中的 ArUco 码，提取 ID 及 VOP 居中坐标（支持传入 marker_id 筛选目标码）",
                    },
                    "get_marker_center": {
                        "params": ["marker_id", "timeout_s", "apply_compensation", "x_compensation_mm", "y_compensation_mm"],
                        "description": "调用卡片获取指定 ID 的 ArUco 码中心位置坐标。返回的 position 字段为机械臂专用的 VOP 居中坐标 [-1, 1]（画面中心为 0,0，左/上为负，右/下为正），请务必直接使用 position 字段作为抓取或移动目标坐标",
                    },
                    "info": {
                        "params": [],
                        "description": "查看识别卡片状态与最新检测到的标记码中心列表",
                    },
                    "start": {
                        "params": ["input_topic", "dictionary"],
                        "description": "启动图像订阅并开启持续检测",
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
                    "dictionary": {
                        "type": "string",
                        "enum": list(DICTIONARIES),
                        "default": DEFAULT_DICTIONARY,
                        "description": "使用的 ArUco 字典类型",
                    },
                    "input_topic": {
                        "type": "string",
                        "default": "",
                        "description": "默认 RGB 图像话题",
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

    def get_tools(self):
        """支持同时注册 aruco_detector 与兼容别名 aruco_zones。"""
        primary = self.get_tool()
        alias = dict(primary)
        alias["name"] = "aruco_zones"
        alias["description"] = "兼容别名：识别画面中 ArUco 标记码并提取 ID 与中心点坐标（纯视觉检测，无待分拣区规则）。"
        return [primary, alias]

    def start(self):
        """卡片启动：初始化节点并开始订阅默认话题。"""
        return self._start_subscription(self._input_topic, self._dictionary)

    def stop(self):
        """卡片停止：销毁订阅与发布节点。"""
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
            return {"state": "idle"}

    def _start_subscription(self, topic: str, dictionary: str):
        """启动对指定图像话题的订阅。"""
        with self._frame_condition:
            if not topic:
                raise ValueError("图像输入话题 input_topic 不能为空")
            if dictionary not in DICTIONARIES:
                raise ValueError(f"未知 ArUco 字典: {dictionary}")

            if Node is not object:
                if self._node is None:
                    node_name = f"rm75_{self.PREFIX}"
                    context = getattr(self._executor, "context", None)
                    node = Node(node_name, context=context)
                    self._node = node
                    base = f"/{self._namespace}/{self.PREFIX}" if self._namespace else f"/{self.PREFIX}"
                    self._image_pub = node.create_publisher(CompressedImage, f"{base}/overlay", 1)
                    self._data_pub = node.create_publisher(String, f"{base}/markers", 1)
                    self._objects_pub = node.create_publisher(String, f"{base}/objects", 1)
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
            self._dictionary = dictionary
            return {
                "ok": True,
                "state": "running",
                "status": "ready",
                "ready": True,
                "input_topic": topic,
                "dictionary": dictionary,
                "topic_out": self._outputs,
            }

    def dispatch(self, action, args):
        """统一动作分发入口，提供可调用的动作执行能力。"""
        if action == "config":
            topic = str(args.get("input_topic") or self._input_topic).strip()
            dictionary = str(args.get("dictionary") or self._dictionary).strip()
            if dictionary:
                if dictionary not in DICTIONARIES:
                    raise ValueError(f"未知 ArUco 字典: {dictionary}")
                self._dictionary = dictionary
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
            dictionary = str(args.get("dictionary") or self._dictionary).strip()
            return self._start_subscription(topic, dictionary)

        if action == "stop":
            return self.stop()

        if action in ("detect", "get_marker_center"):
            raw_id = args.get("marker_id")
            if raw_id is not None and raw_id != "":
                try:
                    marker_id = int(raw_id)
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"marker_id 必须为整数: {raw_id}") from exc
            else:
                marker_id = None

            timeout_s = float(args.get("timeout_s", 3.0))
            apply_comp = bool(args.get("apply_compensation", False))
            x_comp = float(args.get("x_compensation_mm", self._x_compensation_mm))
            y_comp = float(args.get("y_compensation_mm", self._y_compensation_mm))
            return self._detect_action(marker_id, timeout_s, apply_comp, x_comp, y_comp)

        if action == "info":
            return self._info_action()

        raise ValueError(f"不支持的动作: {action}。支持的动作包括: config, start, stop, info, detect, get_marker_center")

    def _detect_action(self, marker_id, timeout_s: float, apply_compensation: bool = False,
                       x_compensation_mm: float = None, y_compensation_mm: float = None):
        """执行 detect 动作：返回当前画面中所有码或指定 ID 码的中心坐标（与 VOP 统一的 [-1, 1] 居中坐标与物理补偿）。"""
        if x_compensation_mm is None:
            x_compensation_mm = self._x_compensation_mm
        if y_compensation_mm is None:
            y_compensation_mm = self._y_compensation_mm

        with self._frame_condition:
            # 若尚未启动订阅且处于 ROS 节点可用环境，自动启动订阅
            if self._subscription is None and Node is not object and self._executor is not None:
                try:
                    self._start_subscription(self._input_topic, self._dictionary)
                except Exception:
                    pass

            now = time.monotonic()
            frame_stale = (now - self._last_frame_at > 2.0) if self._last_frame_at else True

            # 若尚无帧或帧已过期，等待新帧到来
            if (self._latest_result is None or frame_stale) and timeout_s > 0:
                self._frame_condition.wait(timeout_s)

            if self._latest_result is None:
                return {
                    "status": "timeout",
                    "success": False,
                    "found": False,
                    "error": "在指定超时时间内未接收到相机图像帧",
                    "detected_markers": [],
                    "detected_marker_ids": [],
                    "marker_count": 0,
                    "count": 0,
                    "objects": [],
                    "coordinate_system": "vop_centered_normalized_minus1_to_plus1",
                    "compensation": {
                        "x_compensation_mm": x_compensation_mm,
                        "y_compensation_mm": y_compensation_mm,
                        "applied": apply_compensation,
                    },
                }

            markers = list(self._latest_result.get("detected_markers", []))
            width = self._latest_result.get("image_width")
            height = self._latest_result.get("image_height")
            frame_age = round(time.monotonic() - self._last_frame_at, 3) if self._last_frame_at else None

            # 若指定了具体的 marker_id，精确匹配目标码
            if marker_id is not None:
                matched = [m for m in markers if m["id"] == marker_id]
                if matched:
                    target = matched[0]
                    pos = (target.get("compensated_position") if apply_compensation else target.get("position", target.get("center"))) if target else None
                    return {
                        "status": "success",
                        "success": True,
                        "found": True,
                        "marker_id": marker_id,
                        "position": pos,
                        "target_position": pos,
                        "center": pos,
                        "vop_position": pos,
                        "description": "机械臂移动控制请使用 position 字段（VOP 居中归一化坐标 [-1, 1]）",
                        "compensated_position": target.get("compensated_position"),
                        "center_px": target.get("center_px"),
                        "compensated_center_px": target.get("compensated_center_px"),
                        "center_normalized": target.get("center_normalized"),
                        "corners_normalized": target.get("corners_normalized"),
                        "corners_px": target.get("corners_px"),
                        "marker": target,
                        "detected_markers": markers,
                        "detected_marker_ids": [m["id"] for m in markers],
                        "marker_count": len(markers),
                        "count": len(markers),
                        "objects": markers,
                        "coordinate_system": "vop_centered_normalized_minus1_to_plus1",
                        "compensation": {
                            "x_compensation_mm": x_compensation_mm,
                            "y_compensation_mm": y_compensation_mm,
                            "applied": apply_compensation,
                        },
                        "image_width": width,
                        "image_height": height,
                        "frame_age_s": frame_age,
                    }
                else:
                    return {
                        "status": "success",
                        "success": True,
                        "found": False,
                        "marker_id": marker_id,
                        "position": None,
                        "target_position": None,
                        "compensated_position": None,
                        "center": None,
                        "center_normalized": None,
                        "center_px": None,
                        "compensated_center_px": None,
                        "marker": None,
                        "detected_markers": markers,
                        "detected_marker_ids": [m["id"] for m in markers],
                        "marker_count": len(markers),
                        "count": len(markers),
                        "objects": markers,
                        "coordinate_system": "vop_centered_normalized_minus1_to_plus1",
                        "compensation": {
                            "x_compensation_mm": x_compensation_mm,
                            "y_compensation_mm": y_compensation_mm,
                            "applied": apply_compensation,
                        },
                        "image_width": width,
                        "image_height": height,
                        "frame_age_s": frame_age,
                        "message": f"未在当前画面中检测到 ID 为 {marker_id} 的 ArUco 码",
                    }

            # 未指定 marker_id 时，返回检测到的全部标记码列表
            first_target = markers[0] if markers else None
            pos = (first_target.get("compensated_position") if apply_compensation else first_target.get("position", first_target.get("center"))) if first_target else None
            return {
                "status": "success",
                "success": True,
                "found": len(markers) > 0,
                "position": pos,
                "target_position": pos,
                "compensated_position": first_target.get("compensated_position") if first_target else None,
                "center": pos,
                "center_normalized": first_target.get("center_normalized") if first_target else None,
                "center_px": first_target.get("center_px") if first_target else None,
                "compensated_center_px": first_target.get("compensated_center_px") if first_target else None,
                "detected_markers": markers,
                "detected_marker_ids": [m["id"] for m in markers],
                "marker_count": len(markers),
                "count": len(markers),
                "objects": markers,
                "coordinate_system": "vop_centered_normalized_minus1_to_plus1",
                "compensation": {
                    "x_compensation_mm": x_compensation_mm,
                    "y_compensation_mm": y_compensation_mm,
                    "applied": apply_compensation,
                },
                "image_width": width,
                "image_height": height,
                "frame_age_s": frame_age,
            }

    def _info_action(self):
        """执行 info 动作：返回当前卡片运行状态与最新检测到的标记码中心列表。"""
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
                "dictionary": self._dictionary,
                "x_compensation_mm": self._x_compensation_mm,
                "y_compensation_mm": self._y_compensation_mm,
                "coordinate_system": "vop_centered_normalized_minus1_to_plus1",
                "latest": latest,
                "detected_markers": latest.get("detected_markers", []) if latest else [],
                "detected_marker_ids": latest.get("detected_marker_ids", []) if latest else [],
                "marker_count": len(latest.get("detected_markers", [])) if latest else 0,
                "count": len(latest.get("detected_markers", [])) if latest else 0,
                "latest_frame_age_s": round(age, 3) if age is not None else None,
                "last_error": self._last_error,
            }

    def _on_frame(self, message):
        """接收图像帧，处理检测并广播结果。"""
        now = time.monotonic()
        with self._lock:
            if self._node is None or (now - self._last_processed_at < 0.1):
                return
            self._last_processed_at = now
            dictionary = self._dictionary
            x_comp = self._x_compensation_mm
            y_comp = self._y_compensation_mm
            z_m = self._camera_z_mm

        try:
            result, jpeg = process_image(
                message,
                dictionary_name=dictionary,
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
                if self._objects_pub is not None:
                    self._objects_pub.publish(json_message)
        except Exception as exc:
            with self._frame_condition:
                self._last_error = str(exc)
                self._frame_condition.notify_all()
