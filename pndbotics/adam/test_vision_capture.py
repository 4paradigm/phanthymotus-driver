"""ROS-free tests for the Adam ``vision_capture`` card."""

from __future__ import annotations

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

    def test_tool_matches_tianyi_action_contract(self):
        card = VisionCapturePlugin({}, self._camera())
        schema = card.get_tool()["inputSchema"]
        self.assertEqual(schema["properties"]["action"]["enum"][:6], [
            "capture_image", "record_video", "start_recording",
            "stop_recording", "list", "delete",
        ])
        self.assertEqual(schema["x-completion"]["actions"], ["record_video"])

    def test_capture_image_waits_for_a_new_jpeg_then_lists_and_deletes_it(self):
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
            result = card.dispatch("capture_image", {"image_name": "adam-test"})
            self.assertEqual(result["state"], "captured")
            self.assertEqual(result["filename"], "adam-test.jpg")
            self.assertEqual(Path(result["path"]).read_bytes(), b"new-jpeg")
            listed = card.dispatch("list", {})
            self.assertEqual(listed["files"][0]["filename"], "adam-test.jpg")
            self.assertEqual(card.dispatch("delete", {"name": "adam-test.jpg"})["state"], "deleted")
            self.assertFalse(Path(result["path"]).exists())
        writer.join()
        self.assertEqual(camera._photo_waiters, 0)

    def test_capture_image_rejects_unsafe_name(self):
        with tempfile.TemporaryDirectory() as directory:
            card = VisionCapturePlugin({"output_dir": directory}, self._camera())
            result = card.dispatch("capture_image", {"image_name": "../escape"})
            self.assertIn("name must be", result["error"])

    def test_capture_image_reports_camera_timeout(self):
        camera = self._camera()
        card = VisionCapturePlugin({"timeout_s": 1}, camera)
        # Call the camera directly so the test is bounded without waiting for
        # the card's intentionally human-friendly one-second minimum.
        with self.assertRaisesRegex(RuntimeError, "no fresh RGB frame"):
            camera.capture_photo(0.01)

    def test_video_frame_count_tracks_wall_clock_when_camera_lags(self):
        card = VisionCapturePlugin({"video_fps": 15}, self._camera())
        count = 0
        # Simulate an 11.5 FPS camera feeding a 15 FPS encoder for two seconds.
        for frame_index in range(23):
            elapsed = min(2.0, (frame_index + 1) / 11.5)
            count = card._target_frame_count(elapsed, count)
        self.assertEqual(count, 30)


if __name__ == "__main__":
    unittest.main()
