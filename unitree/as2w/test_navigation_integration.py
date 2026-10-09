"""No-hardware tests of transport ownership and canvas command preemption."""
import importlib.util
import queue
import sys
import threading
import types
import unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rpc = load("navigation_rpc", ROOT / "rpc_proxy.py")


class Channel:
    def __init__(self, *_args, **_kwargs):
        self.ready = True
        self.calls = []
        self.ret = 0

    def call(self, method, *args):
        self.calls.append((method, args))
        return (0, {"fsm_name": "AI_STAND_UP"}) if method == "GetState" else self.ret

    def stop(self):
        self.ready = False


class OwnershipTests(unittest.TestCase):
    def setUp(self):
        with patch.object(rpc, "_RpcChannel", Channel):
            self.proxy = rpc.RpcProxy("robot-test")
        self.servo = rpc._ServoClient(self.proxy, "robot-test", Channel)

    def test_servo_excludes_legacy_moves_and_late_stop_finalizers(self):
        self.assertTrue(self.servo.acquire_control()["ok"])
        self.assertNotEqual(0, self.proxy.Move(.1, 0, 0))
        self.assertNotEqual(0, self.proxy.StopMove())
        self.assertEqual([], self.proxy._sport.calls)
        self.assertEqual(0, self.servo.Move(.1, 0, 0))
        self.assertEqual(0, self.servo.StopMove())
        self.servo.release_control()
        self.assertEqual(0, self.proxy.Move(.1, 0, 0))

    def test_legacy_motion_requires_acknowledged_stop_before_acquire(self):
        self.proxy.Move(.1, 0, 0)
        self.assertFalse(self.servo.acquire_control()["ok"])
        self.proxy._sport.ret = 3104
        self.proxy.StopMove()
        self.assertFalse(self.servo.acquire_control()["ok"])
        self.proxy._sport.ret = 0
        self.proxy.StopMove()
        self.assertTrue(self.servo.acquire_control()["ok"])

    def test_acquire_refuses_in_flight_legacy_rpc(self):
        entered, release = threading.Event(), threading.Event()
        def call(*_):
            entered.set()
            release.wait(1)
            return 0
        self.proxy._sport.call = call
        thread = threading.Thread(target=self.proxy.Move, args=(.1, 0, 0))
        thread.start()
        self.assertTrue(entered.wait(1))
        self.assertFalse(self.servo.acquire_control()["ok"])
        release.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())

    def test_legacy_getstate_does_not_take_control_lane(self):
        self.assertTrue(self.servo.acquire_control()["ok"])
        self.assertEqual(0, self.proxy.GetState()[0])
        self.assertEqual(0, self.servo.Move(.1, 0, 0))
        self.assertEqual([("GetState", ())], self.proxy._sport.calls)

    def test_stop_has_independent_lane_after_motion_worker_fault(self):
        self.assertTrue(self.servo.acquire_control()["ok"])
        self.servo._motion.ready = False
        self.assertEqual(0, self.servo.StopMove())
        self.assertEqual([("StopMove", ())], self.servo._emergency.calls)
        self.assertFalse(self.servo.control_ready)

    def test_first_stop_timeout_immediately_uses_spare_lane(self):
        self.assertTrue(self.servo.acquire_control()["ok"])
        def timed_out(*_):
            self.servo._motion.ready = False
            return 3104
        self.servo._motion.call = timed_out
        self.assertEqual(0, self.servo.StopMove())
        self.assertEqual([("StopMove", ())], self.servo._emergency.calls)
        self.assertEqual({"primary_ret": 3104, "fallback_ret": 0}, self.servo.stop_diagnostics)

    def test_vendor_navigation_blocks_servo_until_accepted_pause(self):
        spatial_module = load("navigation_spatial", ROOT / "controlled_spatial.py")
        spatial = spatial_module.ControlledSpatialPlugin.__new__(spatial_module.ControlledSpatialPlugin)
        spatial.set_chassis_guard(self.proxy)
        reply = {"code": 0, "response": {}}
        spatial._client = types.SimpleNamespace(call=lambda *_: reply)
        self.assertEqual(0, spatial.dispatch("resume_navigation", {})["ret"])
        self.assertFalse(self.servo.acquire_control()["ok"])
        reply["code"] = 3104
        spatial.dispatch("pause_navigation", {})
        self.assertFalse(self.servo.acquire_control()["ok"])
        reply["code"] = 0
        spatial.dispatch("pause_navigation", {})
        self.assertTrue(self.servo.acquire_control()["ok"])
        self.assertNotEqual(0, spatial.dispatch("resume_navigation", {})["ret"])

    def test_uncertain_vendor_resume_keeps_chassis_reserved(self):
        spatial_module = load("navigation_spatial_uncertain", ROOT / "controlled_spatial.py")
        spatial = spatial_module.ControlledSpatialPlugin.__new__(spatial_module.ControlledSpatialPlugin)
        spatial.set_chassis_guard(self.proxy)
        spatial._client = types.SimpleNamespace(call=lambda *_: {"code": 3104, "response": "timeout"})
        spatial.dispatch("resume_navigation", {})
        self.assertFalse(self.servo.acquire_control()["ok"])

    def test_special_motion_conflict_is_checked_before_claim(self):
        self.servo.conflict_check = lambda: "special motion active"
        self.assertFalse(self.servo.acquire_control()["ok"])
        self.assertEqual("legacy", self.proxy._owner)

    def test_release_during_in_flight_control_rpc_is_refused(self):
        self.assertTrue(self.servo.acquire_control()["ok"])
        self.servo._active = 1
        with self.assertRaises(RuntimeError):
            self.servo.release_control()
        self.assertEqual("servo", self.proxy._owner)

    def test_timed_out_control_worker_is_poisoned_and_cannot_send_later(self):
        process = types.SimpleNamespace(alive=True, terminated=False)
        process.is_alive = lambda: process.alive
        def terminate():
            process.alive, process.terminated = False, True
        process.terminate = terminate
        channel = rpc._RpcChannel.__new__(rpc._RpcChannel)
        channel._startup_error = None
        channel._fail_closed = True
        channel._process = process
        channel._lock = threading.Lock()
        channel._commands, channel._results = queue.Queue(), queue.Queue()
        channel._timeout = .005
        channel._last_error = {}
        channel._next_request_id = 0
        self.assertEqual(3104, channel.call("Move", .1, 0, 0))
        self.assertTrue(process.terminated)
        self.assertFalse(channel.ready)
        self.assertEqual(3104, channel.call("Move", .2, 0, 0))
        self.assertEqual(1, channel._commands.qsize())


class CanvasPreemptionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Use the existing SDK/ROS stubs; this file never imports live drivers.
        baseline = load("navigation_baseline_stubs", ROOT / "test_driver.py")
        baseline._install_device_stubs()
        cls.main = load("navigation_main", ROOT / "main.py")

    def bundle(self, name="loco", ok=True):
        calls = []
        plugin = types.SimpleNamespace(
            get_tool=lambda: {"name": name},
            dispatch=lambda action, args: calls.append((action, args)) or {"ret": 0})
        servo = types.SimpleNamespace(
            pause_for_explicit_command=lambda reason: {"ok": ok, "reason": reason})
        bundle = self.main.Bundle.__new__(self.main.Bundle)
        bundle.plugins, bundle._servo = [plugin], servo
        return bundle, calls

    def test_failed_stop_barrier_blocks_new_motion(self):
        bundle, calls = self.bundle(ok=False)
        result = bundle.call("loco", {"action": "move", "vx": .1})
        self.assertFalse(result["accepted"])
        self.assertEqual([], calls)

    def test_successful_barrier_preserves_original_arguments(self):
        bundle, calls = self.bundle()
        args = {"action": "move", "vx": .1}
        bundle.call("loco", args)
        self.assertEqual("move", args["action"])
        self.assertEqual(.1, calls[0][1]["vx"])

    def test_reading_state_does_not_interrupt_navigation(self):
        bundle, calls = self.bundle(ok=False)
        self.assertEqual(0, bundle.call("loco", {"action": "get_state"})["ret"])
        self.assertEqual("get_state", calls[0][0])

    def test_special_motion_also_uses_stop_barrier(self):
        bundle, calls = self.bundle(name="special_motion", ok=False)
        self.assertFalse(bundle.call("special_motion", {"action": "handstand", "confirm": True})["accepted"])
        self.assertEqual([], calls)


if __name__ == "__main__":
    unittest.main()
