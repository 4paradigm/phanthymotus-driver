"""No ROS or robot: python3 -m unittest unitree/as2w/test_driver_stop.py"""
import ast
import math
from pathlib import Path
import threading
import types
import unittest
from uuid import uuid4


ROOT = Path(__file__).resolve().parent


class Clock:
    def __init__(self):
        self.now = 10.0

    def monotonic(self):
        return self.now

    def time(self):
        return 1791514800.0 + self.now

    def sleep(self, seconds):
        self.now = round(self.now + seconds, 8)


class Event:
    def __init__(self, clock, cancel_at=None):
        self.clock = clock
        self.cancel_at = cancel_at

    def is_set(self):
        return self.cancel_at is not None and self.clock.now >= self.cancel_at

    def wait(self, seconds):
        self.clock.sleep(seconds)
        return self.is_set()


class Proxy:
    def __init__(self, stop_result=0, state_name="AI_FREE_WALK", move_result=0):
        self.stop_result = stop_result
        self.state_name = state_name
        self.move_result = move_result
        self.stops = 0

    def StopMove(self):
        self.stops += 1
        if isinstance(self.stop_result, Exception):
            raise self.stop_result
        return self.stop_result

    def GetState(self):
        if isinstance(self.state_name, Exception):
            raise self.state_name
        return 0, {"fsm_name": self.state_name}

    def Move(self, *args):
        return self.move_result


def load_driver(clock, notifications):
    # Load the real method bodies, excluding imports requiring ROS/Unitree.
    source = ROOT / "device.py"
    tree = ast.parse(source.read_text())
    keep = {"_number", "_finite_number", "_StateNode", "StatePlugin", "LocoPlugin"}
    selected = ast.Module(body=[node for node in tree.body
                               if isinstance(node, (ast.FunctionDef, ast.ClassDef))
                               and node.name in keep], type_ignores=[])
    namespace = {"math": math, "time": clock, "threading": threading,
                 "uuid4": uuid4,
                 "_acp_notify": lambda *args, **kwargs: notifications.append(args)}
    exec(compile(selected, str(source), "exec"), namespace)
    return namespace


class StopTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.events = []
        self.module = load_driver(self.clock, self.events)
        self.calls = 0

    def sample(self, **changes):
        self.calls += 1
        sample = {"velocity": [0.001, -0.002, 0.0], "yaw_speed": 0.001,
                  "received_monotonic": self.clock.now,
                  "timestamp": self.clock.time(), "generation": self.calls,
                  "stream_id": "original"}
        sample.update(changes)
        return sample

    def plugin(self, provider=None, **proxy_args):
        return self.module["LocoPlugin"]({}, "robot", None, Proxy(**proxy_args),
                                          motion_snapshot=provider or self.sample)

    def run_stop(self, plugin):
        plugin._stop_move_worker("stop-test")
        self.assertEqual(len(self.events), 1)
        return self.events[0][1], self.events[0][2]

    def assert_failed(self, provider, **proxy_args):
        status, result = self.run_stop(self.plugin(provider, **proxy_args))
        self.assertEqual(status, "error")
        self.assertIs(result.get("stopped_confirmed", False), False)
        return result

    def test_free_walk_mode_with_fresh_zero_speeds_completes(self):
        status, result = self.run_stop(self.plugin())
        self.assertEqual(status, "completed")
        self.assertEqual(result["current_state"], "AI_FREE_WALK")
        self.assertIs(result["stopped_confirmed"], True)
        self.assertGreaterEqual(result["stable_samples"], 3)
        self.assertGreaterEqual(result["stable_duration_sec"], 0.3)

    def test_zero_linear_speed_with_rotation_does_not_complete(self):
        self.assert_failed(lambda: self.sample(velocity=[0, 0, 0], yaw_speed=0.5))

    def test_standing_mode_with_linear_motion_does_not_complete(self):
        self.assert_failed(lambda: self.sample(velocity=[0.1, 0, 0]), state_name="STAND_UP")

    def test_three_linear_components_are_measured_not_yaw(self):
        self.assert_failed(lambda: self.sample(velocity=[0, 0, 0.1], yaw_speed=0))

    def test_repeated_sample_does_not_complete(self):
        self.assert_failed(lambda: self.sample(generation=1))

    def test_same_receive_time_with_new_generation_does_not_complete(self):
        self.assert_failed(lambda: self.sample(received_monotonic=10.01))

    def test_pre_stop_and_exact_boundary_samples_cannot_complete(self):
        for received in (9.999, 10.0):
            with self.subTest(received=received):
                self.setUp()
                self.assert_failed(lambda: self.sample(received_monotonic=received))

    def test_future_samples_cannot_complete(self):
        self.assert_failed(lambda: self.sample(received_monotonic=self.clock.now + 0.1))

    def test_stale_samples_cannot_complete(self):
        self.assert_failed(lambda: self.sample(received_monotonic=self.clock.now - 1))

    def test_missing_yaw_or_velocity_cannot_complete(self):
        for changes in ({"yaw_speed": None}, {"velocity": None}, {"velocity": [0, 0]}):
            with self.subTest(changes=changes):
                self.setUp()
                self.assert_failed(lambda: self.sample(**changes))

    def test_nan_and_infinite_measurements_cannot_complete(self):
        for changes in ({"yaw_speed": float("nan")}, {"yaw_speed": float("inf")},
                        {"velocity": [0, float("nan"), 0]},
                        {"received_monotonic": float("inf")}):
            with self.subTest(changes=changes):
                self.setUp()
                self.assert_failed(lambda: self.sample(**changes))

    def test_missing_or_failed_provider_cannot_complete(self):
        def broken():
            raise RuntimeError("state node closed")
        for provider in (lambda: None, broken):
            with self.subTest(provider=provider):
                self.setUp()
                self.assert_failed(provider)

    def test_movement_resets_stable_window(self):
        status, result = self.run_stop(self.plugin(
            lambda: self.sample(yaw_speed=0.5 if self.clock.now == 10.3 else 0)))
        self.assertEqual(status, "completed")
        self.assertGreaterEqual(self.clock.now, 10.7)

    def test_stream_restart_resets_window_even_if_generation_reused(self):
        def provider():
            restarted = self.clock.now >= 10.3
            return self.sample(stream_id="new" if restarted else "old",
                               generation=int(round((self.clock.now - (10.3 if restarted else 10)) * 10)) + 1)
        status, result = self.run_stop(self.plugin(provider))
        self.assertEqual(status, "completed")
        self.assertGreaterEqual(self.clock.now, 10.6)

    def test_generation_rollback_also_resets_window(self):
        def provider():
            return self.sample(generation=100 + self.calls if self.clock.now < 10.3 else self.calls)
        status, _ = self.run_stop(self.plugin(provider))
        self.assertEqual(status, "completed")
        self.assertGreaterEqual(self.clock.now, 10.6)

    def test_fsm_rpc_failure_is_diagnostic_when_physical_samples_valid(self):
        status, result = self.run_stop(self.plugin(state_name=RuntimeError("FSM unavailable")))
        self.assertEqual(status, "completed")
        self.assertIn("state_error", result)

    def test_slow_fsm_diagnostic_does_not_consume_stop_confirmation_window(self):
        plugin = self.plugin()
        def slow_state():
            self.clock.sleep(5.1)
            return 0, {"fsm_name": "AI_FREE_WALK"}
        plugin.proxy.GetState = slow_state
        status, result = self.run_stop(plugin)
        self.assertEqual(status, "completed")
        self.assertTrue(result["stopped_confirmed"])
        self.assertGreaterEqual(result["stable_duration_sec"], 0.3)
        self.assertGreaterEqual(self.clock.now, 15.4)

    def test_stop_rpc_rejection_does_not_read_motion(self):
        status, result = self.run_stop(self.plugin(stop_result=3104))
        self.assertEqual(status, "error")
        self.assertEqual(result["ret"], 3104)
        self.assertEqual(self.calls, 0)

    def test_stop_rpc_exception_is_error(self):
        status, result = self.run_stop(self.plugin(stop_result=RuntimeError("unreachable")))
        self.assertEqual(status, "error")
        self.assertIn("unreachable", result["reason"])

    def test_get_state_exposes_fresh_physical_motion(self):
        result = self.plugin().dispatch("get_state", {})
        self.assertTrue(result["motion"]["valid"])
        self.assertTrue(result["motion"]["fresh"])
        self.assertTrue(result["motion"]["stationary_sample"])

    def test_get_state_missing_yaw_or_stale_is_not_stationary(self):
        for changes in ({"yaw_speed": None}, {"received_monotonic": 9.0}):
            motion = self.plugin(lambda: self.sample(**changes)).dispatch("get_state", {})["motion"]
            self.assertIsNone(motion["stationary_sample"])

    def test_timed_move_complete_does_not_claim_physical_stop(self):
        plugin = self.plugin()
        plugin._run_timed_move("move", 0, 0, 0.5, 0.3, Event(self.clock))
        self.assertEqual(len(self.events), 1)
        self.assertEqual(self.events[0][1], "completed")
        self.assertFalse(self.events[0][2]["stopped_confirmed"])
        self.assertEqual(plugin.proxy.stops, 1)

    def test_timed_move_final_stop_rejection_or_exception_never_completes(self):
        for result in (3104, RuntimeError("lost link")):
            with self.subTest(result=result):
                self.setUp()
                plugin = self.plugin(stop_result=result)
                plugin._run_timed_move("move", 0, 0, 0.5, 0.3, Event(self.clock))
                self.assertEqual(len(self.events), 1)
                self.assertEqual(self.events[0][1], "error")
                self.assertEqual(plugin.proxy.stops, 1)

    def test_timed_move_cancellation_retains_original_status(self):
        plugin = self.plugin()
        plugin._run_timed_move("move", 0, 0, 0.5, 1, Event(self.clock, 10.2))
        self.assertEqual(len(self.events), 1)
        self.assertEqual(self.events[0][1], "cancelled")
        self.assertEqual(plugin.proxy.stops, 1)

    def test_timed_move_rpc_error_retains_original_status(self):
        plugin = self.plugin(move_result=3104)
        plugin._run_timed_move("move", 0, 0, 0.5, 1, Event(self.clock))
        self.assertEqual(len(self.events), 1)
        self.assertEqual(self.events[0][1], "error")
        self.assertEqual(self.events[0][2]["error"], "Timed Move failed")
        self.assertEqual(plugin.proxy.stops, 1)

    def make_state_node(self, stream_id="one"):
        node = self.module["_StateNode"].__new__(self.module["_StateNode"])
        node._latest_lock = threading.Lock()
        node._latest_sport = None
        node._latest_sport_received_ms = None
        node._latest_sport_received_monotonic = None
        node._sport_stream_id = stream_id
        node._sport_generation = 0
        node._stop_event = threading.Event()
        node._publisher_thread = object()
        return node

    def test_state_cache_timestamp_and_publication_yaw(self):
        node = self.make_state_node()
        msg = types.SimpleNamespace(velocity=[0.01, 0, 0], yaw_speed=0.4, position=[1, 2, 3])
        node._on_sport(msg)
        self.clock.sleep(0.2)
        sample = node.motion_snapshot()
        self.assertEqual(sample["received_monotonic"], 10.0)
        self.assertEqual(sample["yaw_speed"], 0.4)
        self.assertEqual(sample["generation"], 1)
        published = []
        node.loco = object()
        node._publish = lambda publisher, payload: published.append(payload)
        node._publish_sport(msg, received_ms=sample["timestamp"] * 1000)
        self.assertEqual(published[0]["timestamp"], 1791514810.0)
        self.assertEqual(published[0]["yaw_speed"], 0.4)
        node._stop_event.set()
        self.assertIsNone(node.motion_snapshot())

    def test_state_cache_missing_yaw_stays_missing(self):
        node = self.make_state_node()
        node._on_sport(types.SimpleNamespace(velocity=[0, 0, 0]))
        self.assertIsNone(node.motion_snapshot()["yaw_speed"])

    def test_dynamic_state_provider_follows_restarted_node(self):
        state = self.module["StatePlugin"].__new__(self.module["StatePlugin"])
        state._state = self.make_state_node("old")
        callback = state.motion_snapshot
        state._state._on_sport(types.SimpleNamespace(velocity=[0, 0, 0], yaw_speed=0))
        self.assertEqual(callback()["stream_id"], "old")
        state._state = None
        self.assertIsNone(callback())
        state._state = self.make_state_node("new")
        state._state._on_sport(types.SimpleNamespace(velocity=[0, 0, 0], yaw_speed=0))
        self.assertEqual(callback()["stream_id"], "new")
        self.assertEqual(callback()["generation"], 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
