"""兼容别名模块：将 aruco_canvas 导向纯中心点提取的 aruco_detector 模块。"""

from aruco_detector import (
    ArucoDetectorPlugin,
    detect_markers,
    extract_markers_from_corners,
    process_image,
    annotate_image,
    DICTIONARIES,
    DEFAULT_DICTIONARY,
)

# 兼容导出
ArucoZonesPlugin = ArucoDetectorPlugin
