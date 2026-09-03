"""Regression tests for superseded controlled-spatial navigation waiters."""

import importlib.util
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]


def load_controlled_spatial():
    spec = importlib.util.spec_from_file_location(
        "controlled_spatial_under_test", ROOT / "controlled_spatial.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


CS = load_controlled_spatial()


class FakeSmartMotion:
    def __init__(self):
        self._arrived = threading.Event()
        self._active_navigation_id = None
        self.wait_started = {}

    def navigate_to(self, *args, navigation_id=None, **kwargs):
        self._active_navigation_id = navigation_id
        return {"status": "navigating"}

    def wait_nav_done(self, stall_timeout=60, navigation_id=None):
        self.wait_started.setdefault(navigation_id, threading.Event()).set()
        self._arrived.wait(timeout=1)
        if navigation_id != self._active_navigation_id:
            return {"status": "superseded"}
        return {"status": "arrived", "pose": {"x": 2.0, "y": 0.0}}

    def arrive(self):
        self._arrived.set()


class FakeClient:
    def NavigateTo(self, *args, **kwargs):
        return 0, {}


class ControlledSpatialSupersedeTests(unittest.TestCase):
    def make_plugin(self, smart_motion):
        db_file = tempfile.NamedTemporaryFile(suffix=".db", delete=False)
        db_file.close()
        self.addCleanup(Path(db_file.name).unlink, missing_ok=True)
        return CS.ControlledSpatialPlugin(
            {"native_slam_db_path": db_file.name}, "g1", None, None, smart_motion)

    def test_smart_motion_arrival_completes_only_latest_navigation(self):
        smart_motion = FakeSmartMotion()
        plugin = self.make_plugin(smart_motion)
        notifications = []

        with patch.object(CS, "_acp_notify", side_effect=lambda *args: notifications.append(args)):
            first = plugin.dispatch("navigate_to_pose", {"x": 1, "y": 0, "yaw": 0})
            self.assertTrue(smart_motion.wait_started[first["action_id"]].wait(timeout=1))
            second = plugin.dispatch("navigate_to_pose", {"x": 2, "y": 0, "yaw": 0})
            self.assertTrue(smart_motion.wait_started[second["action_id"]].wait(timeout=1))
            smart_motion.arrive()
            self.wait_for(lambda: any(item[0] == second["action_id"] and item[1] == "completed"
                                      for item in notifications))

        self.assertEqual(
            [(item[0], item[1]) for item in notifications],
            [(first["action_id"], "cancelled"), (second["action_id"], "completed")],
        )

    def test_fallback_arrival_completes_only_latest_navigation(self):
        plugin = self.make_plugin(None)
        plugin._client = FakeClient()
        notifications = []

        with patch.object(CS, "_acp_notify", side_effect=lambda *args: notifications.append(args)):
            first = plugin.dispatch("navigate_to_pose", {"x": 1, "y": 0, "yaw": 0})
            second = plugin.dispatch("navigate_to_pose", {"x": 2, "y": 0, "yaw": 0})
            plugin._nav_arrived.set()
            self.wait_for(lambda: any(item[0] == second["action_id"] and item[1] == "completed"
                                      for item in notifications))

        self.assertEqual(
            [(item[0], item[1]) for item in notifications],
            [(first["action_id"], "cancelled"), (second["action_id"], "completed")],
        )

    def wait_for(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.01)
        self.fail("condition was not satisfied before timeout")


if __name__ == "__main__":
    unittest.main()
