"""R1 pose_check card: camera frame lifecycle plus reusable pose rules."""

from __future__ import annotations

import threading
import time

from pose_check import POSES, UnavailablePoseEstimator, check_pose


class PoseCheckPlugin:
    PREFIX = "pose_check"

    def __init__(self, plugin_config: dict, namespace: str, executor):
        self._namespace = namespace
        self._executor = executor
        configured_topic = str(plugin_config.get("input_topic", "")).strip()
        self._input_topic = configured_topic or f"/{namespace}/camera/main"
        self._estimator = UnavailablePoseEstimator()
        self._model_error = ""
        if plugin_config.get("model_path"):
            try:
                from pose_estimator import MediaPipePoseEstimator
                self._estimator = MediaPipePoseEstimator(plugin_config["model_path"])
            except Exception as exc:
                self._model_error = str(exc)
        self._lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._latest_frame_at = 0.0
        self._frames = 0
        self._max_frame_age = 2.0
        self._node = None

        try:
            from rclpy.node import Node
            from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                                   ReliabilityPolicy)
            from sensor_msgs.msg import CompressedImage

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._node = Node("r1_pose_check")
            self._node.create_subscription(
                CompressedImage, self._input_topic, self._on_frame, qos
            )
            executor.add_node(self._node)
        except Exception as exc:  # Import/runtime errors must not kill the bundle.
            self._init_error = f"camera subscription unavailable: {exc}"
        else:
            self._init_error = ""

    def get_tool(self) -> dict:
        return {
            "name": "pose_check",
            "type": "processor",
            "multiInstance": False,
            "description": (
                "R1 工位活动动作识别：根据 camera_main 的人体关键点检查"
                " hands_up、arms_open、one_hand_up、squat。check 只返回单帧判断，"
                "不代表保持或下蹲往返完成。仅用于动作模仿反馈，不是医疗评估。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "info", "check"]},
                    "pose": {"type": "string", "enum": list(POSES)},
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {"params": [], "description": "启用姿态检查"},
                    "stop": {"params": [], "description": "停止姿态检查"},
                    "info": {"params": [], "description": "查询相机输入和模型状态"},
                    "check": {"params": ["pose"], "description": "检查当前相机画面的姿态"},
                },
            },
            "topic_in": [{"topic": self._input_topic, "format": "image/jpeg"}],
        }

    def _on_frame(self, message) -> None:
        try:
            jpeg = bytes(message.data)
        except (TypeError, ValueError):
            return
        if not jpeg:
            return
        with self._lock:
            self._latest_jpeg = jpeg
            self._latest_frame_at = time.monotonic()
            self._frames += 1

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def dispatch(self, action: str, args: dict) -> dict | None:
        if action == "start":
            return {"state": "ready", "input_topic": self._input_topic}
        if action == "stop":
            return {"state": "idle"}
        if action == "info":
            with self._lock:
                frames = self._frames
                age_ms = ((time.monotonic() - self._latest_frame_at) * 1000
                          if self._latest_frame_at else None)
            return {
                "state": "ready" if not self._init_error else "error",
                "input_topic": self._input_topic,
                # Canvas wiring queries info(), rather than the static tool
                # descriptor, when it needs to discover a processor input.
                "topic_in": [{"topic": self._input_topic,
                              "format": "image/jpeg"}],
                "format": "image/jpeg",
                "frames": frames,
                "latest_frame_age_ms": age_ms,
                "estimator": self._estimator.name,
                "model_error": self._model_error or None,
                **({"error": self._init_error} if self._init_error else {}),
            }
        if action != "check":
            return None

        pose = args.get("pose")
        if pose not in POSES:
            return {
                "error": "unsupported_pose",
                "pose": pose,
                "supported_poses": list(POSES),
            }

        with self._lock:
            jpeg = self._latest_jpeg
            frame_at = self._latest_frame_at
        if jpeg is None:
            return {
                "error": "camera_no_data",
                "detected": False,
                "matched": False,
                "pose": pose,
                "feedback": "暂时没有收到相机画面",
            }

        if time.monotonic() - frame_at > self._max_frame_age:
            return {"error": "camera_stale", "detected": False, "matched": False,
                    "pose": pose, "feedback": "相机画面已过期，请检查连接"}

        try:
            keypoints = self._estimator.estimate(jpeg)
        except Exception as exc:  # A backend failure is a card result, not a driver crash.
            return {
                "error": "model_failed",
                "detected": False,
                "matched": False,
                "pose": pose,
                "feedback": "动作识别暂时不可用",
                "detail": str(exc),
            }
        if keypoints is None:
            return {
                "error": "model_unavailable",
                "detected": False,
                "matched": False,
                "pose": pose,
                "feedback": "动作识别模型未配置",
                "detail": self._model_error or "Set pose_check.model_path and install requirements-pose.txt",
            }
        return check_pose(keypoints, pose)
