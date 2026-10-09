"""No ROS, robot or network: AS2W stream failure and ordering contracts.

Run from the repository root: python3 -m unittest unitree/as2w/test_loco_servo.py
"""
import importlib.util
from pathlib import Path
import sys
import threading
import time
import unittest


ROOT = Path(__file__).resolve().parents[2]
BUNDLE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))


def load(name, filename):
    spec = importlib.util.spec_from_file_location(name, BUNDLE / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


control = load("as2w_control", "as2w_control.py")
servo = load("as2w_loco_servo_test_module", "loco_servo.py")


class Clock:
    def __init__(self):
        self.value = 1000.0

    def now(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class Client:
    max_call_seconds = 0.35
    control_ready = True

    def __init__(self):
        self.events = []
        self.owned = False
        self.move_ret = self.stop_ret = 0
        self.state = "AI_STAND_UP"
        self.claim_ok = True

    def acquire_control(self):
        if not self.claim_ok:
            return {"ok": False, "error": "legacy action in flight"}
        self.events.append("acquire")
        self.owned = True
        return {"ok": True}

    def release_control(self):
        self.events.append("release")
        self.owned = False

    def GetState(self):
        self.events.append("state")
        return 0, {"fsm_name": self.state}

    def Move(self, *values):
        if not self.owned:
            raise AssertionError("write without control ownership")
        self.events.append(("move", values))
        if isinstance(self.move_ret, Exception):
            raise self.move_ret
        return self.move_ret

    def StopMove(self):
        if not self.owned:
            raise AssertionError("stop without ownership")
        self.events.append("stop")
        return self.stop_ret

    @property
    def moves(self):
        return [event[1] for event in self.events if isinstance(event, tuple)]


class StreamTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = Client()
        self.descriptor = servo.build_descriptor()

    def controller(self, **kwargs):
        return control.VelocityController(
            self.client, self.descriptor, clock=self.clock.now,
            wall_clock=self.clock.now, threaded=False, **kwargs)

    def command(self, seq=1, values=None, **kwargs):
        out = dict(schema="motus.control/1", mode="twist", dof=6,
                   source="navi", session_id="one", seq=seq, priority=50,
                   stamp_ms=self.clock.now() * 1000,
                   obs_stamp_ms=self.clock.now() * 1000, ttl_ms=200,
                   values=values if values is not None else [.25, 0, 0, 0, 0, .4])
        out.update(kwargs)
        return out

    def test_default_dry_run_never_claims_or_calls_sdk(self):
        c = self.controller()
        self.assertTrue(c.activate()["ok"])
        self.assertTrue(c.submit(self.command())["ok"])
        c.pump()
        self.assertEqual(self.client.events, [])
        self.assertEqual(c.info()["simulated"], 1)
        self.assertEqual(c.info()["sdk_accepted"], 0)

    def test_idle_card_pause_does_not_wait_for_nonexistent_thread(self):
        c = control.VelocityController(self.client, self.descriptor)
        before = time.monotonic()
        self.assertTrue(c.pause(timeout=0.1)["ok"])
        self.assertLess(time.monotonic() - before, 0.08)
        self.assertEqual(self.client.events, [])

    def test_real_start_clears_previous_command_then_checks_posture(self):
        c = self.controller(dry_run=False)
        self.assertTrue(c.activate()["ok"])
        self.assertEqual(self.client.events, ["acquire", "stop", "state"])
        self.assertIsNone(c.info()["physical_stop_verified"])
        self.assertTrue(c.info()["stop_acknowledged"])

    def test_initial_stop_failure_latches_and_keeps_ownership(self):
        self.client.stop_ret = 3104
        c = self.controller(dry_run=False)
        self.assertFalse(c.activate()["ok"])
        self.assertTrue(c.info()["fault_latched"])
        self.assertTrue(self.client.owned)
        self.assertNotIn("state", self.client.events)

    def test_bad_posture_prevents_nonzero_motion(self):
        self.client.state = "AI_DAMPING"
        c = self.controller(dry_run=False)
        self.assertFalse(c.activate()["ok"])
        self.assertFalse(self.client.owned)
        self.assertEqual(self.client.moves, [])

    def test_existing_motion_and_unbounded_proxy_cannot_acquire(self):
        c = self.controller(dry_run=False, conflict=lambda: True)
        self.assertFalse(c.activate()["ok"])
        self.assertEqual(self.client.events, [])
        self.client.max_call_seconds = 7.0
        c = self.controller(dry_run=False)
        self.assertFalse(c.activate()["ok"])
        self.assertEqual(self.client.events, [])

    def test_finite_pinned_axes_ttl_and_timestamps_are_receiver_requirements(self):
        c = self.controller()
        c.activate()
        cases = [dict(values=[float("nan"), 0, 0, 0, 0, 0]),
                 dict(values=[0, 0, float("inf"), 0, 0, 0]),
                 dict(values=[0, 0, .01, 0, 0, 0]),
                 dict(values=[2, 0, 0, 0, 0, 0]), dict(obs_stamp_ms=None),
                 dict(ttl_ms=301), dict(ttl_ms=0), dict(ttl_ms=float("inf")),
                 dict(stamp_ms=self.clock.now() * 1000 + 51),
                 dict(obs_stamp_ms=self.clock.now() * 1000 - 501),
                 dict(obs_stamp_ms=self.clock.now() * 1000 + 51), dict(seq=True),
                 dict(priority=float("nan")), dict(dof=True)]
        for invalid in cases:
            with self.subTest(invalid=invalid):
                self.assertFalse(c.submit(self.command(**invalid))["ok"])
        c.pump()
        self.assertEqual(c.info()["simulated"], 0)

    def test_latest_only_queue_and_slew_from_rest(self):
        c = self.controller(dry_run=False)
        c.activate()
        self.assertTrue(c.submit(self.command(1, [.2, 0, 0, 0, 0, 0]))["ok"])
        self.assertTrue(c.submit(self.command(2, [0, .15, 0, 0, 0, -.4]))["ok"])
        c.pump()
        self.assertEqual(len(self.client.moves), 1)
        vx, vy, wz = self.client.moves[0]
        self.assertEqual(vx, 0)
        self.assertAlmostEqual(vy, .03)
        self.assertAlmostEqual(wz, -.05)
        self.assertEqual(c.info()["replaced"], 1)

    def test_sdk_failure_is_not_accepted_and_stops_then_latches(self):
        c = self.controller(dry_run=False)
        c.activate()
        self.client.move_ret = 127
        c.submit(self.command())
        c.pump()
        info = c.info()
        self.assertEqual(info["attempted"], 1)
        self.assertEqual(info["sdk_accepted"], 0)
        self.assertEqual(info["sdk_errors"], 1)
        self.assertEqual(info["last_move_ret"], 127)
        self.assertEqual(info["last_stop_ret"], 0)
        self.assertTrue(info["fault_latched"])
        self.assertTrue(info["owner_acquired"])
        self.assertEqual(self.client.events[-1], "stop")
        self.assertFalse(c.submit(self.command(2))["ok"])
        self.assertFalse(c.activate()["ok"])
        self.assertTrue(c.reset_fault()["ok"])
        self.assertFalse(self.client.owned)
        self.assertEqual(c.info()["state"], "paused")

    def test_sdk_exception_also_stops_and_latches(self):
        c = self.controller(dry_run=False)
        c.activate()
        self.client.move_ret = TimeoutError("request timed out")
        c.submit(self.command())
        c.pump()
        self.assertTrue(c.info()["fault_latched"])
        self.assertEqual(self.client.events[-1], "stop")

    def test_pause_failure_does_not_release_control_or_advertise_success(self):
        c = self.controller(dry_run=False)
        c.activate()
        self.client.stop_ret = 3104
        self.assertFalse(c.pause()["ok"])
        self.assertTrue(c.info()["fault_latched"])
        self.assertTrue(self.client.owned)
        self.assertFalse(c.reset_fault()["ok"])

    def test_pause_discards_old_frames_and_resume_changes_epoch(self):
        c = self.controller(dry_run=False)
        c.activate()
        old = self.command()
        c.submit(old)
        self.assertTrue(c.pause()["ok"])
        self.clock.advance(.01)
        self.assertTrue(c.activate()["ok"])
        self.assertFalse(c.submit(old)["ok"])
        self.assertTrue(c.submit(self.command())["ok"])
        c.pump()
        self.assertEqual(len(self.client.moves), 1)

    def test_source_restart_requires_explicit_new_epoch(self):
        c = self.controller()
        c.activate()
        self.assertTrue(c.submit(self.command(seq=10))["ok"])
        self.assertFalse(c.submit(self.command(seq=1, session_id="two"))["ok"])
        self.assertFalse(c.submit(self.command(seq=9))["ok"])
        c.pause()
        self.clock.advance(.01)
        c.activate()
        self.assertTrue(c.submit(self.command(seq=1, session_id="two"))["ok"])

    def test_equal_priority_other_source_cannot_steal_lease(self):
        c = self.controller()
        c.activate()
        self.assertTrue(c.submit(self.command())["ok"])
        self.assertFalse(c.submit(self.command(source="other"))["ok"])
        self.assertTrue(c.submit(self.command(source="other", priority=60))["ok"])

    def test_queue_expiry_is_rechecked_after_posture_rpc(self):
        c = self.controller(dry_run=False)
        c.activate()
        self.clock.advance(.26)
        original = self.client.GetState

        def delayed_state():
            self.clock.advance(.21)
            return original()

        self.client.GetState = delayed_state
        c.submit(self.command())
        c.pump()
        c.pump()
        self.assertEqual(self.client.moves, [])
        self.assertEqual(c.info()["state"], "paused")
        self.assertFalse(self.client.owned)

    def test_watchdog_stops_and_requires_resume(self):
        c = self.controller(dry_run=False)
        c.activate()
        c.submit(self.command())
        c.pump()
        self.clock.advance(.301)
        c.pump()
        self.assertEqual(c.info()["state"], "paused")
        self.assertFalse(c.submit(self.command(2))["ok"])
        self.assertFalse(self.client.owned)

    def test_executed_ttl_stops_before_later_watchdog(self):
        c = self.controller(dry_run=False)
        c.activate()
        c.submit(self.command(ttl_ms=200))
        c.pump()
        self.clock.advance(.201)
        c.pump()
        self.assertEqual(c.info()["state"], "paused")
        self.assertEqual(c.info()["reason"], "executed command TTL expired")
        self.assertEqual(self.client.events[-2:], ["stop", "release"])

    def test_fresh_successful_frame_replaces_executed_deadline(self):
        c = self.controller(dry_run=False)
        c.activate()
        c.submit(self.command(ttl_ms=200))
        c.pump()
        self.clock.advance(.11)
        c.submit(self.command(seq=2, ttl_ms=200))
        c.pump()
        self.clock.advance(.10)
        c.pump()
        self.assertEqual(c.info()["state"], "running")
        self.clock.advance(.101)
        c.pump()
        self.assertEqual(c.info()["state"], "paused")

    def test_slow_move_ack_after_ttl_is_followed_immediately_by_stop(self):
        c = self.controller(dry_run=False)
        c.activate()
        original = self.client.Move

        def delayed_move(*args):
            self.clock.advance(.21)
            return original(*args)

        self.client.Move = delayed_move
        c.submit(self.command(ttl_ms=200))
        c.pump()
        self.assertEqual(self.client.events[-2:], ["stop", "release"])
        self.assertEqual(c.info()["state"], "paused")

    def test_no_frame_yet_is_not_stream_failure(self):
        c = self.controller(dry_run=False)
        c.activate()
        before = list(self.client.events)
        self.clock.advance(10)
        c.pump()
        self.assertEqual(c.info()["state"], "running")
        self.assertEqual(self.client.events, before)

    def test_posture_loss_stops_previous_velocity_and_latches(self):
        c = self.controller(dry_run=False)
        c.activate()
        c.submit(self.command(ttl_ms=300))
        c.pump()
        self.clock.advance(.26)
        self.client.state = "AI_FALLEN"
        c.submit(self.command(2))
        c.pump()
        self.assertEqual(len(self.client.moves), 1)
        self.assertEqual(self.client.events[-1], "stop")
        self.assertTrue(c.info()["fault_latched"])

    def test_full_zero_bypasses_slew_and_calls_stop(self):
        c = self.controller(dry_run=False)
        c.activate()
        c.submit(self.command())
        c.pump()
        self.clock.advance(.01)
        c.submit(self.command(2, [0] * 6))
        c.pump()
        self.assertEqual(len(self.client.moves), 1)
        self.assertEqual(self.client.events[-1], "stop")

    def test_real_to_dry_run_stops_before_switch_and_remains_paused(self):
        c = self.controller(dry_run=False)
        c.activate()
        c.submit(self.command())
        c.pump()
        c.configure(dry_run=True)
        self.assertEqual(self.client.events[-2:], ["stop", "release"])
        self.assertTrue(c.info()["dry_run"])
        self.assertEqual(c.info()["state"], "paused")
        self.assertFalse(c.submit(self.command(2))["ok"])

    def test_dry_run_transition_cannot_hide_stop_failure(self):
        c = self.controller(dry_run=False)
        c.activate()
        self.client.stop_ret = 3104
        c.configure(dry_run=True)
        self.assertFalse(c.info()["dry_run"])
        self.assertTrue(c.info()["fault_latched"])

    def test_rest_needs_fresh_known_axes_after_ack_over_multiple_samples(self):
        sample = {}
        c = self.controller(dry_run=False, odom_provider=lambda: sample)
        c.activate()
        c.pause()
        sample.update(schema="motus.odom/1", frame="body", stamp_ms=self.clock.now() * 1000,
                      twist=[None] * 6)
        c.pump()
        self.assertIsNone(c.info()["physical_stop_verified"])
        for delta in (.01, .11, .11):
            self.clock.advance(delta)
            sample.update(stamp_ms=self.clock.now() * 1000, twist=[0, 0, None, None, None, 0])
            c.pump()
        self.assertTrue(c.info()["physical_stop_verified"])
        self.clock.advance(.6)
        c.pump()
        self.assertIsNone(c.info()["physical_stop_verified"])

    def test_descriptor_does_not_invent_deadband_or_footprint(self):
        self.assertNotIn("min_magnitude", self.descriptor["limits"])
        self.assertNotIn("footprint", self.descriptor)
        self.assertEqual(self.descriptor["limits"]["upper"][2:5], [0, 0, 0])
        for key in ("vx_limit", "expected_hz", "watchdog_ms"):
            with self.assertRaises(ValueError):
                servo.build_descriptor({key: float("nan")})


class ThreadOrderingTests(unittest.TestCase):
    def test_resume_cannot_rearm_between_stop_ack_and_disconnect(self):
        client = Client()
        plugin = servo.LocoServoPlugin({}, "test", None, client)
        plugin._node = object()
        plugin._controller.activate()
        entered, finish, resumed = threading.Event(), threading.Event(), threading.Event()
        results = {}
        def disconnect():
            entered.set()
            finish.wait(1)
            plugin._node = None
        plugin._disconnect = disconnect
        stop = threading.Thread(target=lambda: results.update(stop=plugin.dispatch("stop", {})))
        def resume():
            results["resume"] = plugin.dispatch("resume", {})
            resumed.set()
        start = threading.Thread(target=resume)
        try:
            stop.start()
            self.assertTrue(entered.wait(.5))
            start.start()
            self.assertFalse(resumed.wait(.05))
            finish.set()
            stop.join(1)
            start.join(1)
            self.assertEqual("idle", results["stop"]["state"])
            self.assertFalse(results["resume"]["ok"])
            self.assertEqual("paused", plugin._controller.info()["state"])
            self.assertFalse(client.moves)
        finally:
            finish.set()
            plugin._controller.close()

    def test_inflight_move_then_pause_is_stop_last_and_callback_is_nonblocking(self):
        client = Client()
        entered, finish = threading.Event(), threading.Event()
        original = client.Move

        def blocked_move(*args):
            entered.set()
            if not finish.wait(1.0):
                raise TimeoutError("test worker was not released")
            return original(*args)

        client.Move = blocked_move
        c = control.VelocityController(client, servo.build_descriptor(), dry_run=False)
        try:
            self.assertTrue(c.activate()["ok"])

            def frame(seq):
                now = time.time() * 1000
                return dict(schema="motus.control/1", mode="twist", dof=6,
                            source="navi", seq=seq, stamp_ms=now, obs_stamp_ms=now,
                            ttl_ms=200, values=[.2, 0, 0, 0, 0, 0])

            c.submit(frame(1))
            self.assertTrue(entered.wait(.5))
            before = time.monotonic()
            for seq in range(2, 20):
                self.assertTrue(c.submit(frame(seq))["ok"])
            self.assertLess(time.monotonic() - before, .1)
            self.assertFalse(c.pause(timeout=.01)["ok"])
            self.assertTrue(client.owned)
            finish.set()
            self.assertTrue(c._stop_event.wait(.5))
            self.assertEqual(len(client.moves), 1)
            self.assertEqual(client.events[-2:], ["stop", "release"])
            self.assertFalse(c.submit(frame(20))["ok"])
        finally:
            finish.set()
            c.close()


if __name__ == "__main__":
    unittest.main()
