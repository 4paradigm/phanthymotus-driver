"""fall_alarm — Bumi 视觉跌倒检测卡片（YOLOv11-pose + 姿态几何判定）。

模块划分（遵循本仓库「纯逻辑与 ROS 分离、无 ROS 即可单测」的约定）：

  fall_detector — 纯「姿态关键点 → 跌倒判定」逻辑，无 ROS / 无 YOLO，直接单测
  pose_engine   — YOLOv11-pose 推理封装（惰性导入 ultralytics，官方权重）
  speech        — 报警语音 → 16 kHz 单声道 S16LE PCM（预录 WAV + 提示音兜底）
  plugin        — FallAlarmPlugin，即 MCP card 本体（导入 rclpy）

画布连线与部署说明见本目录 README.md。
"""

__all__: list[str] = []
