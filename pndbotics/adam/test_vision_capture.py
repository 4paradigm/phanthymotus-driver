"""ROS-free tests for the Adam ``vision_capture`` card."""

from __future__ import annotations

import base64
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest

# The host-side unit-test environment need not carry the Jetson image's NumPy
# dependency.  These tests exercise only the JPEG cache/card boundary.
sys.modules.setdefault("numpy", types.ModuleType("numpy"))

from device import VisionCapturePlugin, ZedCameraPlugin


class VisionCaptureTests(unittest.TestCase):
    @staticmethod
    def _camera():
        """Make the small cache portion of a camera without ROS/ZED hardware."""
        camera = object.__new__(ZedCameraPlugin)
        camera._lock = threading.Lock()
        camera._photo_condition = threading.Condition(camera._lock)
        camera._photo_waiters = 0
        camera._latest_rgb = {"data": b"old", "timestamp_ms": 0, "sequence": 4}
        camera._rgb_sequence = 4
        camera._running = True
        camera._state = {"state": "running", "available": True, "error": None}
        return camera

    def test_capture_photo_waits_for_a_new_jpeg_and_returns_a_displayable_image(self):
        camera = self._camera()

        def publish_new_frame():
            time.sleep(0.02)
            with camera._photo_condition:
                camera._rgb_sequence += 1
                camera._latest_rgb = {
                    "data": b"new-jpeg", "timestamp_ms": 1, "sequence": camera._rgb_sequence,
                }
                camera._photo_condition.notify_all()

        writer = threading.Thread(target=publish_new_frame)
        writer.start()
        with tempfile.TemporaryDirectory() as directory:
            card = VisionCapturePlugin({"output_dir": directory, "timeout_s": 1}, camera)
            result = card.dispatch("capture_photo", {})
            self.assertTrue(result["ok"])
            self.assertEqual(base64.b64decode(result["image_data_url"].split(",", 1)[1]), b"new-jpeg")
            self.assertEqual(Path(result["file_path"]).read_bytes(), b"new-jpeg")
        writer.join()
        self.assertEqual(camera._photo_waiters, 0)

    def test_capture_photo_reports_camera_timeout(self):
        camera = self._camera()
        card = VisionCapturePlugin({"timeout_s": 1}, camera)
        # Call the camera directly so the test is bounded without waiting for
        # the card's intentionally human-friendly one-second minimum.
        with self.assertRaisesRegex(RuntimeError, "no fresh RGB frame"):
            camera.capture_photo(0.01)


if __name__ == "__main__":
    unittest.main()
