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
        self.wait_started[navigation_id] = threading.Event()
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
        with patch.object(CS, "_SlamRpcProxy", return_value=FakeClient()):
            plugin = CS.ControlledSpatialPlugin(
                {"native_slam_db_path": db_file.name}, "g1", None, None, smart_motion)
        self.addCleanup(plugin._db._conn.close)
        return plugin

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
        self.old_wait_started = threading.Event()
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
        self.old_wait_started.set()
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
        with patch.object(CS, "_SlamRpcProxy", return_value=FakeClient()):
            plugin = CS.ControlledSpatialPlugin(
                {"native_slam_db_path": db_file.name}, "g1", None, None, smart_motion)
        self.addCleanup(plugin._db._conn.close)
        return plugin

    def test_old_arrival_during_inflight_replacement_does_not_complete_old_action(self):
        sm = RaceySmartMotion()
        plugin = self.make_plugin(sm)
        plugin._nav_command_lock = ObservedLock()
        self.addCleanup(sm.nav_continue.set)
        self.addCleanup(sm.old_arrive.set)
        self.addCleanup(sm.new_arrive.set)
        notifications = []

        with patch.object(CS, "_acp_notify", side_effect=lambda *args: notifications.append(args)):
            # 1. Start first navigation — old waiter blocks on wait_nav_done(old_id).
            first = plugin.dispatch("navigate_to_pose", {"x": 1, "y": 0, "yaw": 0})
            self.assertIsNotNone(first.get("action_id"))
            self.assertTrue(sm.old_wait_started.wait(timeout=2))
            plugin._nav_command_lock.contended.clear()

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
            self.assertTrue(sm.nav_in_flight.wait(timeout=2))
            # At this point _nav_action_id is already the new id (reserved before
            # the blocking call).  Inject a stale arrival for the old navigation.
            sm.old_arrive.set()
            self.assertTrue(plugin._nav_command_lock.contended.wait(timeout=2))

            # Terminal claiming waits for the replacement to commit or roll back.
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


class ObservedLock:
    """Expose contention without relying on scheduler sleeps."""

    def __init__(self):
        self.lock = threading.Lock()
        self.contended = threading.Event()

    def __enter__(self):
        if not self.lock.acquire(blocking=False):
            self.contended.set()
            self.lock.acquire()

    def __exit__(self, *args):
        self.lock.release()


class SubmissionContractTests(unittest.TestCase):
    make_plugin = ControlledSpatialSupersedeTests.make_plugin

    def prepare(self, smart_motion):
        plugin = self.make_plugin(smart_motion)
        plugin._active_map = "test"
        plugin._db.add_poi("target", 1, 0, 0, "test")
        return plugin

    def test_navigation_commands_are_serialized(self):
        for smart in (False, True):
            for first_action in ("navigate_to_tag", "navigate_to_pose"):
                for second_action in ("navigate_to_tag", "navigate_to_pose",
                                      "pause_nav", "resume_nav", "stop_nav"):
                    for barrier_at in ("reservation", "rpc", "waiter_setup"):
                        with self.subTest(smart=smart, first=first_action,
                                          second=second_action, barrier=barrier_at):
                            self.check_serialized(smart, first_action, second_action, barrier_at)

    def check_serialized(self, smart, first_action, second_action, barrier_at):
        from unittest.mock import Mock

        backend = Mock()
        plugin = self.prepare(backend if smart else None)
        if not smart:
            plugin._client = backend
        lock = ObservedLock()
        plugin._nav_command_lock = lock
        blocked = threading.Event()
        release = threading.Event()
        calls = []
        results = {}
        errors = []
        waiters = []
        real_thread = threading.Thread

        def barrier():
            blocked.set()
            if not release.wait(timeout=5):
                raise AssertionError("submission barrier was not released")

        def rpc(name, *args, **kwargs):
            # RPC must never hold the state lock.
            self.assertTrue(plugin._lock.acquire(blocking=False))
            plugin._lock.release()
            calls.append((name, plugin._nav_action_id))
            if barrier_at == "rpc" and len(calls) == 1:
                barrier()
            return {"status": "navigating"} if smart else (0, {})

        names = (("navigate_to", "pause_nav", "resume_nav", "stop_nav") if smart
                 else ("NavigateTo", "PauseNav", "ResumeNav"))
        for name in names:
            getattr(backend, name).side_effect = lambda *a, _name=name, **kw: rpc(_name, *a, **kw)

        reserve = plugin._reserve_nav
        reservations = []

        def reserve_nav(action_id):
            value = reserve(action_id)
            reservations.append(action_id)
            if barrier_at == "reservation" and len(reservations) == 1:
                barrier()
            return value

        def waiter_thread(*args, **kwargs):
            if barrier_at == "waiter_setup" and not waiters:
                barrier()
            thread = real_thread(*args, **kwargs)
            waiters.append(thread)
            return thread

        def dispatch(key, action):
            try:
                results[key] = plugin.dispatch(action, {"tag_name": "target", "x": 2})
            except BaseException as exc:
                errors.append(exc)

        first = real_thread(target=dispatch, args=("first", first_action), daemon=True)
        second = real_thread(target=dispatch, args=("second", second_action), daemon=True)
        with patch.object(plugin, "_reserve_nav", side_effect=reserve_nav), \
                patch.object(plugin, "_acp_wait_nav"), \
                patch.object(CS.threading, "Thread", side_effect=waiter_thread), \
                patch.object(CS, "_acp_notify"):
            try:
                first.start()
                self.assertTrue(blocked.wait(timeout=2))
                reserved_id = plugin._nav_action_id
                second.start()
                self.assertTrue(lock.contended.wait(timeout=2))
                self.assertEqual(reservations, [reserved_id])
                self.assertEqual(len(calls), 0 if barrier_at == "reservation" else 1)
                # Read-only dispatch and state access remain available during RPC.
                self.assertEqual(plugin.dispatch("info", {}), {"state": "running"})
                self.assertIsNone(plugin._get_pose())
            finally:
                release.set()
                first.join(timeout=3)
                if second.ident is not None:
                    second.join(timeout=3)
                for waiter in waiters:
                    waiter.join(timeout=3)
            self.assertFalse(first.is_alive())
            self.assertFalse(second.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual(len(calls), 2)
            self.assertEqual(calls[0][1], results["first"]["action_id"])
            expected_id = results["second"].get("action_id", results["first"]["action_id"])
            self.assertEqual(plugin._nav_action_id, expected_id)
            self.assertEqual(calls[1][1], expected_id)

    def test_fallback_failed_replacement_preserves_waiter(self):
        """The waiter must survive a provisional generation change and rollback."""
        for action in ("navigate_to_tag", "navigate_to_pose"):
            with self.subTest(action=action):
                plugin = self.prepare(None)
                _, _, generation = plugin._reserve_nav("old")
                plugin._nav_command_lock = ObservedLock()
                rpc_started = threading.Event()
                release = threading.Event()
                results = []

                def fail_navigation(*args, **kwargs):
                    rpc_started.set()
                    if not release.wait(timeout=5):
                        raise AssertionError("RPC barrier was not released")
                    return 3204, {}

                replacement = threading.Thread(
                    target=lambda: results.append(plugin.dispatch(
                        action, {"tag_name": "target", "x": 2})), daemon=True)
                old_waiter = threading.Thread(
                    target=plugin._acp_wait_nav,
                    args=("old", "old target", 90, (10, 0), generation), daemon=True)
                with patch.object(plugin._client, "NavigateTo", side_effect=fail_navigation), \
                        patch.object(CS, "_acp_notify") as notify:
                    try:
                        replacement.start()
                        self.assertTrue(rpc_started.wait(timeout=2))
                        # Schedule the old waiter while the reservation is provisional.
                        old_waiter.start()
                        self.assertTrue(plugin._nav_command_lock.contended.wait(timeout=2))
                        self.assertTrue(old_waiter.is_alive())
                        notify.assert_not_called()
                    finally:
                        release.set()
                        replacement.join(timeout=2)
                    self.assertFalse(replacement.is_alive())
                    self.assertIn("error", results[0])
                    self.assertEqual(plugin._nav_action_id, "old")
                    self.assertEqual(plugin._nav_generation, generation)
                    notify.assert_not_called()
                    plugin._nav_arrived.set()
                    old_waiter.join(timeout=2)
                    self.assertFalse(old_waiter.is_alive())
                    self.assertEqual([c.args[:2] for c in notify.call_args_list],
                                     [("old", "completed")])
                    self.assertIsNone(plugin._nav_action_id)

    def test_smart_motion_arrival_survives_expired_replacement(self):
        from unittest.mock import Mock

        for action in ("navigate_to_tag", "navigate_to_pose"):
            with self.subTest(action=action):
                backend = Mock()
                plugin = self.prepare(backend)
                _, _, generation = plugin._reserve_nav("old")
                plugin._nav_command_lock = ObservedLock()
                entered = threading.Event()
                release = threading.Event()
                arrival = threading.Event()

                def wait(**kwargs):
                    entered.set()
                    self.assertTrue(arrival.wait(timeout=3))
                    return {"status": "arrived"}

                def navigate(*args, **kwargs):
                    arrival.set()
                    self.assertTrue(release.wait(timeout=3))
                    return {"status": "expired"}

                backend.wait_nav_done.side_effect = wait
                backend.navigate_to.side_effect = navigate
                waiter = threading.Thread(target=plugin._acp_wait_nav,
                                          args=("old", "target", 90, None, generation))
                replacement = threading.Thread(target=plugin.dispatch,
                                               args=(action, {"tag_name": "target", "x": 2}))
                with patch.object(CS, "_acp_notify") as notify:
                    try:
                        waiter.start()
                        self.assertTrue(entered.wait(timeout=2))
                        replacement.start()
                        self.assertTrue(plugin._nav_command_lock.contended.wait(timeout=2))
                        self.assertTrue(waiter.is_alive())
                        notify.assert_not_called()
                    finally:
                        release.set()
                        arrival.set()
                        replacement.join(timeout=3)
                        waiter.join(timeout=3)
                    self.assertFalse(waiter.is_alive())
                    self.assertFalse(replacement.is_alive())
                    self.assertEqual([c.args[:2] for c in notify.call_args_list],
                                     [("old", "completed")])

    def test_unknown_wait_retains_tracking_until_confirmed(self):
        from unittest.mock import Mock

        for terminal in ("stopped", "superseded", "arrived"):
            with self.subTest(terminal=terminal):
                backend = Mock()
                plugin = self.prepare(backend)
                _, _, generation = plugin._reserve_nav("old")
                backend.wait_nav_done.side_effect = [
                    {"status": "unknown"},
                    {"status": "arrived" if terminal == "arrived" else "unknown"}]
                backend._call.side_effect = [{"status": "unknown"}, {"status": terminal}]
                with patch.object(CS, "_acp_notify") as notify:
                    def retry_delay(seconds):
                        self.assertGreaterEqual(seconds, 0.1)
                        self.assertEqual(plugin._nav_action_id, "old")
                        notify.assert_not_called()

                    with patch.object(CS.time, "sleep", side_effect=retry_delay) as sleep:
                        plugin._acp_wait_nav("old", "target", generation=generation)
                    sleep.assert_called_once()
                    self.assertEqual(backend.wait_nav_done.call_count, 2)
                    for call in backend._call.call_args_list:
                        self.assertEqual(call.args, ("cancel_navigation",))
                        self.assertEqual(call.kwargs, {"navigation_id": "old"})
                    backend.stop_nav.assert_not_called()
                    backend.pause_nav.assert_not_called()
                    self.assertIsNone(plugin._nav_action_id)
                    self.assertEqual(notify.call_count, 1)
                    expected = {"arrived": "completed", "stopped": "error",
                                "superseded": "cancelled"}[terminal]
                    self.assertEqual(notify.call_args.args[:2], ("old", expected))

    def test_unknown_cancellation_does_not_terminate_replacement(self):
        from unittest.mock import Mock

        backend = Mock()
        plugin = self.prepare(backend)
        _, _, generation = plugin._reserve_nav("old")
        backend.wait_nav_done.return_value = {"status": "unknown"}
        backend.navigate_to.return_value = {"status": "navigating"}
        waiter = plugin._acp_wait_nav
        replacement = {}

        def cancel(method, navigation_id):
            self.assertEqual((method, navigation_id), ("cancel_navigation", "old"))
            # Submission is allowed while ID-targeted cancellation is in flight.
            with patch.object(plugin, "_acp_wait_nav"):
                replacement.update(plugin.dispatch("navigate_to_pose", {"x": 2}))
            return {"status": "superseded"}

        backend._call.side_effect = cancel
        with patch.object(CS, "_acp_notify") as notify:
            waiter("old", "target", generation=generation)
            self.assertEqual(plugin._nav_action_id, replacement["action_id"])
            self.assertEqual([c.args[:2] for c in notify.call_args_list],
                             [("old", "cancelled")])
            backend.stop_nav.assert_not_called()
            backend.pause_nav.assert_not_called()

    def test_stall_pause_blocks_replacement_submission(self):
        from unittest.mock import Mock

        plugin = self.prepare(None)
        plugin._client = Mock()
        _, _, generation = plugin._reserve_nav("old")
        plugin._nav_command_lock = ObservedLock()
        paused = threading.Event()
        release = threading.Event()
        plugin._nav_arrived = Mock()
        plugin._nav_arrived.wait.return_value = False
        plugin._client.NavigateTo.return_value = (0, {})

        def pause():
            paused.set()
            self.assertTrue(release.wait(timeout=3))
            return 0, {}

        plugin._client.PauseNav.side_effect = pause
        waiter = threading.Thread(target=plugin._acp_wait_nav,
                                  args=("old", "target", -1, None, generation))
        replacement = threading.Thread(target=plugin.dispatch,
                                       args=("navigate_to_pose", {"x": 2}))
        with patch.object(CS, "_acp_notify") as notify:
            try:
                waiter.start()
                self.assertTrue(paused.wait(timeout=2))
                with patch.object(plugin, "_acp_wait_nav"):
                    replacement.start()
                    self.assertTrue(plugin._nav_command_lock.contended.wait(timeout=2))
                    plugin._client.NavigateTo.assert_not_called()
                    release.set()
                    replacement.join(timeout=3)
            finally:
                release.set()
                waiter.join(timeout=3)
            self.assertFalse(waiter.is_alive())
            self.assertFalse(replacement.is_alive())
            plugin._client.PauseNav.assert_called_once()
            plugin._client.NavigateTo.assert_called_once()
            self.assertEqual(notify.call_args.args[:2], ("old", "error"))

    def test_smart_motion_submission_status_contract(self):
        from unittest.mock import Mock

        for action in ("navigate_to_tag", "navigate_to_pose"):
            for status in ("unknown", "expired", "error"):
                with self.subTest(action=action, status=status):
                    backend = Mock()
                    plugin = self.prepare(backend)
                    plugin._reserve_nav("old")
                    old_generation = plugin._nav_generation
                    started = threading.Event()
                    release = threading.Event()
                    threads = []
                    real_thread = threading.Thread

                    def navigate(*args, navigation_id=None, **kwargs):
                        return {"status": status, "navigation_id": navigation_id,
                                "error": "submission timeout"}

                    def wait(**kwargs):
                        started.set()
                        if not release.wait(timeout=5):
                            raise AssertionError("waiter barrier was not released")
                        return {"status": "arrived", "pose": {"x": 1, "y": 0}}

                    def waiter_thread(*args, **kwargs):
                        thread = real_thread(*args, **kwargs)
                        threads.append(thread)
                        return thread

                    backend.navigate_to.side_effect = navigate
                    backend.wait_nav_done.side_effect = wait
                    with patch.object(CS, "_acp_notify") as notify, \
                            patch.object(CS.threading, "Thread", side_effect=waiter_thread):
                        try:
                            result = plugin.dispatch(action, {"tag_name": "target", "x": 1})
                            self.assertEqual(result["status"], status)
                            if status == "unknown":
                                self.assertTrue(started.wait(timeout=2))
                                self.assertEqual(result["action_id"], result["navigation_id"])
                                self.assertEqual(plugin._nav_action_id, result["action_id"])
                                self.assertEqual(plugin._nav_generation, old_generation + 1)
                                backend.wait_nav_done.assert_called_once_with(
                                    stall_timeout=90, navigation_id=result["action_id"])
                                self.assertEqual(result["error"], "submission timeout")
                                self.assertEqual(notify.call_args.args[:2], ("old", "cancelled"))
                            else:
                                self.assertNotIn("action_id", result)
                                backend.wait_nav_done.assert_not_called()
                                self.assertEqual(threads, [])
                                if status == "expired":
                                    self.assertEqual(plugin._nav_action_id, "old")
                                    self.assertEqual(plugin._nav_generation, old_generation)
                                    notify.assert_not_called()
                                    backend.stop_nav.assert_not_called()
                                    backend.pause_nav.assert_not_called()
                                else:
                                    self.assertIsNone(plugin._nav_action_id)
                                    self.assertEqual(notify.call_args.args[:2], ("old", "cancelled"))
                        finally:
                            release.set()
                            for thread in threads:
                                thread.join(timeout=2)
                                self.assertFalse(thread.is_alive())
                        if status == "unknown":
                            self.assertEqual([call.args[:2] for call in notify.call_args_list],
                                             [("old", "cancelled"), (result["action_id"], "completed")])
                            self.assertIsNone(plugin._nav_action_id)


if __name__ == "__main__":
    unittest.main()
