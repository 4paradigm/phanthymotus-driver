"""Optional JPEG -> anatomical keypoints adapter, shared by local and ROS paths."""

from __future__ import annotations

import threading
import time


class MediaPipePoseEstimator:
    name = "mediapipe"
    LANDMARKS = {"left_shoulder": 11, "right_shoulder": 12,
                 "left_elbow": 13, "right_elbow": 14,
                 "left_wrist": 15, "right_wrist": 16,
                 "left_hip": 23, "right_hip": 24,
                 "left_knee": 25, "right_knee": 26,
                 "left_ankle": 27, "right_ankle": 28}

    def __init__(self, model_path: str, *, video: bool = False):
        import cv2
        import mediapipe as mp
        import numpy as np

        self._cv2, self._mp, self._np = cv2, mp, np
        self._lock = threading.Lock()
        self._video = video
        self._last_timestamp_ms = -1
        options = mp.tasks.vision.PoseLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=model_path),
            running_mode=(mp.tasks.vision.RunningMode.VIDEO if video
                          else mp.tasks.vision.RunningMode.IMAGE),
        )
        self._model = mp.tasks.vision.PoseLandmarker.create_from_options(options)

    def estimate(self, jpeg: bytes, *, max_width: int = 0) -> dict:
        """Estimate landmarks from a JPEG frame.

        The ROS compressed-image path still uses this method.  Raw ROS images
        should use :meth:`estimate_bgr` so they do not pay an unnecessary JPEG
        encode/decode round trip.
        """
        frame = self._cv2.imdecode(self._np.frombuffer(jpeg, dtype=self._np.uint8),
                                   self._cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("Invalid JPEG frame")
        return self.estimate_bgr(frame, max_width=max_width)

    def estimate_bgr(self, frame, *, max_width: int = 0) -> dict:
        """Estimate landmarks from an already-decoded BGR image.

        ``max_width=0`` keeps the source resolution.  The landmark coordinates
        are normalized, so inference can safely run on a smaller frame while
        callers keep the same geometric rules.
        """
        if frame is None or len(frame.shape) < 2:
            raise ValueError("Invalid BGR frame")
        height, width = frame.shape[:2]
        if height <= 0 or width <= 0:
            raise ValueError("Invalid BGR frame dimensions")
        if max_width > 0 and width > max_width:
            scaled_height = max(1, round(height * max_width / width))
            frame = self._cv2.resize(
                frame, (max_width, scaled_height),
                interpolation=self._cv2.INTER_AREA,
            )
        rgb = self._cv2.cvtColor(frame, self._cv2.COLOR_BGR2RGB)
        return self._estimate_rgb(rgb, frame.shape[1], frame.shape[0])

    def _estimate_rgb(self, rgb, width: int, height: int) -> dict:
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)
        with self._lock:
            if self._video:
                # Serialize timestamps with inference: even rapid calls must
                # satisfy MediaPipe's strictly increasing video time contract.
                timestamp_ms = max(time.monotonic_ns() // 1_000_000,
                                   self._last_timestamp_ms + 1)
                self._last_timestamp_ms = timestamp_ms
                result = self._model.detect_for_video(image, timestamp_ms)
            else:
                result = self._model.detect(image)
        if not result.pose_landmarks:
            return {}
        landmarks = result.pose_landmarks[0]
        points = {name: (landmarks[i].x, landmarks[i].y,
                       min(landmarks[i].visibility, landmarks[i].presence))
                for name, i in self.LANDMARKS.items()}
        points["__image_size__"] = {"width": width, "height": height}
        return points

    def close(self):
        with self._lock:
            self._model.close()
