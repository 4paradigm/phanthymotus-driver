"""Headless preview checks; run with the optional pose virtual environment."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None

from pose_local import draw_preview


@unittest.skipIf(cv2 is None, "Requires requirements-pose.txt")
class PreviewTests(unittest.TestCase):
    def test_confidence_colors_and_input_preserved(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        points = {"left_wrist": (.25, .5, .9), "right_wrist": (.75, .5, .2)}
        result = {"pose": "arms_open", "detected": False, "matched": False}
        image = draw_preview(frame, points, result)
        self.assertEqual(image[240, 160].tolist(), [0, 220, 0])
        self.assertEqual(image[240, 480].tolist(), [0, 0, 255])
        self.assertFalse(frame.any())

    def test_invalid_and_offscreen_points_are_not_drawn(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        points = {"left_wrist": (float("nan"), .5, 1),
                  "right_wrist": (2, .5, 1)}
        result = {"pose": "arms_open", "detected": False, "matched": False}
        image = draw_preview(frame, points, result)
        self.assertFalse(image[66:].any())


if __name__ == "__main__":
    unittest.main()
