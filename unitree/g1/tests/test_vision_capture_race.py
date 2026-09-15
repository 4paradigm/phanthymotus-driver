"""Regression checks for vision_capture terminal-state cancellation races."""

from __future__ import annotations

import ast
from pathlib import Path
import tempfile
import threading
import unittest


ROOT = Path(__file__).resolve().parents[1]


class _CameraFrameNode:
    def __init__(self, topic):
        self.topic = topic
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True


def _load_class(name, notifications=None):
    notifications = notifications if notifications is not None else []
    tree = ast.parse((ROOT / "device.py").read_text())
    class_node = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == name)
    namespace = globals().copy()
    namespace.update({
        "_VISION_FIRST_FRAME_TIMEOUT_S": 5.0,
        "_VISION_MAX_FRAME_AGE_S": 3.0,
        "_vision_acp_notify": lambda *args: notifications.append(args),
    })
    exec(compile(ast.Module(body=[class_node], type_ignores=[]),
                 str(ROOT / "device.py"), "exec"), namespace)
    return namespace[name]


class VisionCaptureRaceTests(unittest.TestCase):
    def test_cancellation_wins_over_already_encoded_success(self):
        notifications = []
        plugin_class = _load_class("VisionCapturePlugin", notifications)
        plugin = plugin_class.__new__(plugin_class)
        plugin._recording_lock = threading.Lock()
        plugin._last_recording = None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "video.mp4"
            path.write_bytes(b"encoded")
            active = {
                "action_id": "record-1", "state": "recording",
                "cancel_event": threading.Event(), "finished": False,
            }
            active["cancel_event"].set()
            plugin._active_recording = active
            plugin._record_video = lambda _active: {
                "ok": True, "file_path": str(path), "media_type": "video"}

            plugin._record_video_async(active)

            self.assertFalse(path.exists())
            self.assertEqual("cancelled", plugin._last_recording["status"])
            self.assertEqual("RECORD_CANCELLED",
                             plugin._last_recording["result"]["code"])
            self.assertEqual("cancelled", notifications[0][1])
            self.assertIsNone(plugin._active_recording)

    def test_stop_does_not_cancel_a_committed_terminal_state(self):
        notifications = []
        plugin_class = _load_class("VisionCapturePlugin", notifications)
        plugin = plugin_class.__new__(plugin_class)
        plugin._recording_lock = threading.Lock()
        cancel = threading.Event()
        thread = type("Thread", (), {
            "join": lambda self, timeout=None: None,
            "is_alive": lambda self: False,
        })()
        active = {
            "action_id": "record-2", "state": "completed",
            "cancel_event": cancel, "process": None, "thread": thread,
            "finished": True,
        }
        plugin._active_recording = active

        result = plugin.stop()

        self.assertFalse(cancel.is_set())
        self.assertEqual("idle", result["state"])
        self.assertEqual("completed", active["state"])

    def test_realsense_stop_removes_and_destroys_cache_node(self):
        plugin_class = _load_class("RealSensePlugin")
        plugin = plugin_class.__new__(plugin_class)
        removed = []
        added = []
        plugin._executor = type("Executor", (), {
            "add_node": lambda self, node: added.append(node),
            "remove_node": lambda self, node: removed.append(node),
        })()
        plugin._color_topic = "/g1/camera/rgb"
        plugin._proc = None
        old_node = _CameraFrameNode(plugin._color_topic)
        plugin._frame_node = old_node

        plugin.stop()

        self.assertEqual([old_node], removed)
        self.assertTrue(old_node.destroyed)
        self.assertIsNone(plugin._frame_node)
        plugin._ensure_frame_node()
        self.assertEqual(1, len(added))
        self.assertIs(plugin._frame_node, added[0])

    def test_missing_camera_returns_structured_precondition(self):
        plugin_class = _load_class("VisionCapturePlugin")
        plugin = plugin_class.__new__(plugin_class)
        plugin._camera = None
        plugin._max_duration_s = 30

        photo = plugin._capture_photo()
        video = plugin._start_video_recording({"duration_s": 5})

        self.assertEqual("PRECONDITION_FAILED", photo["code"])
        self.assertEqual("PRECONDITION_FAILED", video["code"])


if __name__ == "__main__":
    unittest.main()
