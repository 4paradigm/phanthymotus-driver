"""traffic_light — Unitree AS2-W 交通灯识别卡片（YOLOv8n 检测 + HSV 颜色判定）。

模块划分（遵循本仓库「纯逻辑与 ROS 分离、无 ROS 即可单测」的约定）：

  detection_engine — YOLOv8n 推理封装（惰性导入 ultralytics，官方 COCO 权重）
  light_classifier — 纯「检测框 → 红/黄/绿」判定 + 状态去抖，无 ROS / 无 YOLO，直接单测
  speech           — 播报短语 → 16 kHz 单声道 S16LE PCM（预录 WAV + 提示音兜底）
  plugin           — TrafficLightPlugin，即 MCP card 本体（导入 rclpy）

画布连线与部署说明见本目录 README.md。
"""

__all__: list[str] = []
