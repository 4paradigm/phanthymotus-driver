"""Backend dispatch tests without starting MediaPipe or accessing a camera."""

import sys
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pose_estimator import MediaPipePoseEstimator


class EstimatorTests(unittest.TestCase):
    def make_estimator(self, video):
        estimator = MediaPipePoseEstimator.__new__(MediaPipePoseEstimator)
        estimator._video = video
        estimator._last_timestamp_ms = -1
        estimator._lock = threading.Lock()
        estimator._cv2 = Mock()
        estimator._np = Mock()
        estimator._mp = Mock()
        estimator._model = Mock()
        estimator._cv2.imdecode.return_value = SimpleNamespace(shape=(480, 640, 3))
        empty = SimpleNamespace(pose_landmarks=[])
        estimator._model.detect.return_value = empty
        estimator._model.detect_for_video.return_value = empty
        return estimator

    def test_video_timestamps_increase_even_with_same_clock_reading(self):
        estimator = self.make_estimator(True)
        with patch("pose_estimator.time.monotonic_ns", return_value=100_000_000):
            self.assertEqual(estimator.estimate(b"jpeg"), {})
            self.assertEqual(estimator.estimate(b"jpeg"), {})
        calls = estimator._model.detect_for_video.call_args_list
        self.assertEqual([call.args[1] for call in calls], [100, 101])
        estimator._model.detect.assert_not_called()

    def test_image_mode_does_not_use_tracking(self):
        estimator = self.make_estimator(False)
        self.assertEqual(estimator.estimate(b"jpeg"), {})
        estimator._model.detect.assert_called_once()
        estimator._model.detect_for_video.assert_not_called()

    def test_bad_jpeg_does_not_reuse_previous_result(self):
        estimator = self.make_estimator(True)
        estimator._cv2.imdecode.return_value = None
        with self.assertRaises(ValueError):
            estimator.estimate(b"invalid")
        estimator._model.detect_for_video.assert_not_called()

    def test_bgr_path_skips_jpeg_decode_and_can_downscale(self):
        estimator = self.make_estimator(False)
        source = SimpleNamespace(shape=(720, 1280, 3))
        resized = SimpleNamespace(shape=(360, 640, 3))
        estimator._cv2.INTER_AREA = 7
        estimator._cv2.resize.return_value = resized

        self.assertEqual(estimator.estimate_bgr(source, max_width=640), {})

        estimator._cv2.imdecode.assert_not_called()
        estimator._cv2.resize.assert_called_once_with(
            source, (640, 360), interpolation=7)
