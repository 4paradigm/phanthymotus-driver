"""Regression checks for vision_capture terminal-state cancellation races."""

from __future__ import annotations

import ast
from pathlib import Path
import tempfile
import threading
import unittest


ROOT = Path(__file__).resolve().parents[1]


def _load_plugin_class(notifications):
    tree = ast.parse((ROOT / "device.py").read_text())
    plugin_node = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "VisionCapturePlugin")
    namespace = globals().copy()
    namespace.update({
        "_VISION_FIRST_FRAME_TIMEOUT_S": 5.0,
        "_VISION_MAX_FRAME_AGE_S": 3.0,
        "_vision_acp_notify": lambda *args: notifications.append(args),
    })
    exec(compile(ast.Module(body=[plugin_node], type_ignores=[]),
                 str(ROOT / "device.py"), "exec"), namespace)
    return namespace["VisionCapturePlugin"]


class VisionCaptureRaceTests(unittest.TestCase):
    def test_cancellation_wins_over_already_encoded_success(self):
        notifications = []
        plugin_class = _load_plugin_class(notifications)
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
        plugin_class = _load_plugin_class(notifications)
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


if __name__ == "__main__":
    unittest.main()
