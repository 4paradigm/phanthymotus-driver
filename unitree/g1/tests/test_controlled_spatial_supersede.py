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


class RaceySmartMotion:
    """Simulates the subprocess race: the second navigate_to RPC is in-flight
    (blocked) when the old waiter's wait_nav_done returns a stale 'arrived'.

    With the old code (navigate_to submitted before _supersede_nav), the old
    waiter could _try_claim_terminal its action_id and fire 'completed' while
    the new navigation was already physically underway.  With reserve-before-
    submit, _nav_action_id is already the new id, so the old claim fails.
    """

    def __init__(self):
        self.old_id = None
        self.new_id = None
        self.nav_in_flight = threading.Event()
        self.nav_continue = threading.Event()
        self.old_arrive = threading.Event()
        self.new_arrive = threading.Event()

    def navigate_to(self, *args, navigation_id=None, **kwargs):
        if self.old_id is None:
            self.old_id = navigation_id
            return {"status": "navigating"}
        # Second navigate_to: block to simulate an in-flight RPC.
        self.new_id = navigation_id
        self.nav_in_flight.set()
        self.nav_continue.wait(timeout=5)
        return {"status": "navigating"}

    def wait_nav_done(self, stall_timeout=60, navigation_id=None):
        if navigation_id == self.new_id:
            self.new_arrive.wait(timeout=5)
            return {"status": "arrived", "pose": {"x": 2.0, "y": 0.0}}
        # Old navigation: wait for the test to inject a stale arrival.
        self.old_arrive.wait(timeout=5)
        return {"status": "arrived", "pose": {"x": 1.0, "y": 0.0}}


class ArrivalInterleavingTests(unittest.TestCase):
    """Regression: an arrival detected on the old navigation must not fire
    'completed' for the old action when a replacement navigation has already
    reserved its action_id (even if the replacement RPC is still in-flight)."""

    def make_plugin(self, smart_motion):
        db_file = tempfile.NamedTemporaryFile(suffix=".db", delete=False)
        db_file.close()
        self.addCleanup(Path(db_file.name).unlink, missing_ok=True)
        return CS.ControlledSpatialPlugin(
            {"native_slam_db_path": db_file.name}, "g1", None, None, smart_motion)

    def test_old_arrival_during_inflight_replacement_does_not_complete_old_action(self):
        sm = RaceySmartMotion()
        plugin = self.make_plugin(sm)
        notifications = []

        with patch.object(CS, "_acp_notify", side_effect=lambda *args: notifications.append(args)):
            # 1. Start first navigation — old waiter blocks on wait_nav_done(old_id).
            first = plugin.dispatch("navigate_to_pose", {"x": 1, "y": 0, "yaw": 0})
            self.assertIsNotNone(first.get("action_id"))
            # Give the old waiter thread time to enter wait_nav_done.
            time.sleep(0.1)

            # 2. Start second navigation in a background thread because navigate_to
            #    blocks (simulating in-flight RPC).  reserve_nav(new_id) runs
            #    BEFORE the blocking navigate_to call.
            second_result = {}
            def _start_second():
                second_result["value"] = plugin.dispatch(
                    "navigate_to_pose", {"x": 2, "y": 0, "yaw": 0})
            t = threading.Thread(target=_start_second, daemon=True)
            t.start()

            # 3. Wait until the second navigate_to is blocked in-flight.
            sm.nav_in_flight.wait(timeout=2)
            # At this point _nav_action_id is already the new id (reserved before
            # the blocking call).  Inject a stale arrival for the old navigation.
            sm.old_arrive.set()
            # Give the old waiter time to process the stale arrival.
            time.sleep(0.2)

            # The old waiter must NOT have fired 'completed' — its _try_claim_terminal
            # should fail because _nav_action_id is the new id.
            old_completed = [n for n in notifications
                             if n[0] == first["action_id"] and n[1] == "completed"]
            self.assertEqual(old_completed, [],
                             f"old action must not complete during replacement; got {notifications}")

            # 4. Release the blocking navigate_to so the second nav completes setup.
            sm.nav_continue.set()
            t.join(timeout=2)
            second = second_result.get("value")
            self.assertIsNotNone(second)
            self.assertEqual(second.get("status"), "navigating")

            # 5. Signal arrival for the new navigation.
            sm.new_arrive.set()
            # Wait for the new action to complete.
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                if any(n[0] == second["action_id"] and n[1] == "completed"
                       for n in notifications):
                    break
                time.sleep(0.01)

        # Final assertions: old action got exactly 'cancelled' (never 'completed'),
        # new action got 'completed'.
        old_events = [(n[0], n[1]) for n in notifications if n[0] == first["action_id"]]
        new_events = [(n[0], n[1]) for n in notifications if n[0] == second["action_id"]]
        self.assertEqual(old_events, [(first["action_id"], "cancelled")],
                         f"old action should only be cancelled, got {old_events}")
        self.assertIn((second["action_id"], "completed"), new_events,
                      f"new action should complete, got {new_events}")


if __name__ == "__main__":
    unittest.main()
