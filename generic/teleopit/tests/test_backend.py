"""Adapter contracts plus an opt-in REAL Teleopit / GMR / ONNX / MuJoCo smoke.

Run lightweight checks with unittest discovery. Set TELEOPIT_TEST_ROOT to a
bootstrapped v0.5.0 source root in its dependency environment to include the
real three-policy-step smoke; a skipped smoke is NOT simulation evidence.
"""

from __future__ import annotations

from dataclasses import dataclass
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


SPEC = importlib.util.spec_from_file_location(
    "teleopit_driver_backend", Path(__file__).resolve().parents[1] / "backend.py"
)
backend = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(backend)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeEvent:
    def __init__(self, clock, *, set_=False):
        self.clock = clock
        self.value = set_
        self.waits = []

    def is_set(self):
        return self.value

    def wait(self, seconds):
        self.waits.append(seconds)
        self.clock.now += seconds
        return self.value


class GateTests(unittest.TestCase):
    def test_regular_pacing_is_50_hz(self):
        clock = FakeClock()
        stop = FakeEvent(clock)
        gate = backend._Gate(stop, FakeEvent(clock), lambda event: None, clock=clock)
        clock.now = 0.008
        gate.next_step()
        self.assertAlmostEqual(clock.now, 0.02)
        self.assertEqual(len(stop.waits), 1)

    def test_slow_step_resets_deadline_and_never_catches_up(self):
        clock = FakeClock()
        stop = FakeEvent(clock)
        gate = backend._Gate(stop, FakeEvent(clock), lambda event: None, clock=clock)
        clock.now = 10.0
        gate.next_step()
        self.assertEqual(stop.waits, [])
        clock.now += 0.005
        gate.next_step()
        self.assertAlmostEqual(stop.waits[-1], 0.015)

    def test_stop_preempts_pause(self):
        clock = FakeClock()
        gate = backend._Gate(FakeEvent(clock, set_=True), FakeEvent(clock, set_=True), lambda event: None)
        with self.assertRaises(backend._Stopped):
            gate.check()


@dataclass(frozen=True)
class Packet:
    timestamp_s: float
    seq: int = 1
    control_events: tuple = ("toggle_arms", "toggle_pause")


class PicoInputTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.gate = backend._Gate(FakeEvent(self.clock), FakeEvent(self.clock), lambda event: None, clock=self.clock)
        self.packet = Packet(timestamp_s=0.0)
        self.provider = SimpleNamespace(
            has_frame=lambda: True,
            is_available=lambda: True,
            get_realtime_input_packet=lambda: self.packet,
        )

    def test_strips_headset_controls_without_changing_body_packet(self):
        monitor = backend._PicoInput(self.provider, self.gate, 10.0)
        packet = monitor.read()
        self.assertEqual(packet.control_events, ())
        self.assertEqual(packet.timestamp_s, self.packet.timestamp_s)
        self.assertEqual(self.packet.control_events, ("toggle_arms", "toggle_pause"))
        self.assertFalse(monitor.freshness()["input_stale"])

    def test_reports_stale_then_times_out_if_stream_stops(self):
        monitor = backend._PicoInput(self.provider, self.gate, 10.0)
        monitor.read()
        self.clock.now = 0.6
        self.assertEqual(monitor.freshness()["input_age_ms"], 600.0)
        self.assertTrue(monitor.freshness()["input_stale"])
        self.clock.now = 10.1
        with self.assertRaisesRegex(TimeoutError, "stopped updating"):
            monitor.read()

    def test_no_body_frame_times_out_while_remain_cancellable(self):
        self.provider.has_frame = lambda: False
        monitor = backend._PicoInput(self.provider, self.gate, 0.05)
        with self.assertRaisesRegex(TimeoutError, "full-body tracking"):
            monitor.read()
        self.assertGreaterEqual(self.clock.now, 0.05)
        self.assertLess(self.clock.now, 0.1)

    def test_input_timestamp_must_be_finite(self):
        self.packet = Packet(timestamp_s=float("nan"))
        with self.assertRaisesRegex(ValueError, "finite"):
            backend._PicoInput(self.provider, self.gate, 1.0).read()


class FakeBus:
    def __init__(self):
        self.callbacks = {}

    def subscribe(self, topic, callback):
        self.callbacks[topic] = callback

    def publish(self, topic, state):
        self.callbacks[topic](state)


class FakePipeline:
    """Test double of the pinned bus/API seam, NOT an algorithm smoke."""

    def __init__(self, *, bad_state=False, bad_action=False):
        self.closed = False
        self.state = SimpleNamespace(
            qpos=[float("nan") if bad_state else 0.4] * 29,
            quat=[1.0, 0.0, 0.0, 0.0], base_pos=[0.0, 0.0, 0.76], timestamp=0.0,
        )
        self.robot = SimpleNamespace(
            num_actions=29, default_dof_pos=[0.0] * 29,
            model=SimpleNamespace(nq=36, njnt=30, jnt_qposadr=[0] + list(range(7, 36)),
                                  joint=lambda i: SimpleNamespace(name=f"joint_{i}")),
        )
        self.bus = FakeBus()
        self.controller = SimpleNamespace(compute_action=lambda obs: [float("inf") if bad_action else 0.2] * 29)
        self.retargeter = SimpleNamespace(retarget=lambda frame: [0.0] * 36)
        self.input_provider = SimpleNamespace(close=self.close)
        self.runner = SimpleNamespace(
            build_observation=lambda: [0.0] * 167,
            compute_target_dof_pos=lambda action: [0.7] * 29,
            apply_control=self.apply_control,
        )
        self.loop = SimpleNamespace(_step_runner=self.runner)

    def apply_control(self, target):
        self.state.timestamp += 0.02
        return [99.0] * 29, self.state

    def close(self):
        self.closed = True

    def run(self, num_steps):
        for _ in range(num_steps):
            self.retargeter.retarget(None)
            obs = self.runner.build_observation()
            action = self.controller.compute_action(obs)
            target = self.runner.compute_target_dof_pos(action)
            _, state = self.runner.apply_control(target)
            self.bus.publish("state", state)


class PipelineAdapterTests(unittest.TestCase):
    def setUp(self):
        self.topics = ModuleType("teleopit.bus.topics")
        self.topics.TOPIC_ROBOT_STATE = "state"
        self.options = {"source": "bvh", "max_steps": 3, "render": False}
        self.events = []
        self.stop = threading.Event()
        self.pause = threading.Event()

    def run_fake(self, pipeline, emit=None):
        with patch.object(backend, "_make_pipeline", return_value=pipeline), patch.dict(
            sys.modules, {"teleopit.bus.topics": self.topics}
        ):
            backend.run_backend(self.options, emit or self.events.append, self.stop, self.pause)

    def test_real_bus_seam_reports_targets_not_torques_or_measured_positions(self):
        pipeline = FakePipeline()
        self.run_fake(pipeline)
        self.assertTrue(pipeline.closed)
        self.assertEqual(self.events[0]["event"], "ready")
        frames = [event["snapshot"] for event in self.events if event["event"] == "frame"]
        self.assertEqual(len(frames), 3)
        self.assertEqual(frames[-1]["target_positions"], [0.7] * 29)
        self.assertEqual(frames[-1]["joint_positions"], [0.4] * 29)
        self.assertEqual(frames[-1]["sim_time_s"], 0.06)
        self.assertFalse(frames[-1]["hardware_output"])
        self.assertEqual(self.events[-1]["summary"]["steps"], 3)
        json.dumps(self.events, allow_nan=False)

    def test_stop_from_first_frame_unwinds_and_reports_one_step(self):
        def emit(event):
            self.events.append(event)
            if event["event"] == "frame":
                self.stop.set()

        pipeline = FakePipeline()
        self.run_fake(pipeline, emit)
        self.assertTrue(pipeline.closed)
        self.assertEqual(self.events[-1]["summary"]["reason"], "stopped")
        self.assertEqual(self.events[-1]["summary"]["steps"], 1)

    def test_stop_before_init_does_not_load_dependencies(self):
        self.stop.set()
        with patch.object(backend, "_make_pipeline") as factory:
            backend.run_backend(self.options, self.events.append, self.stop, self.pause)
        factory.assert_not_called()
        self.assertEqual(self.events[-1]["summary"]["reason"], "stopped")

    def test_bad_policy_output_rejected_before_physics(self):
        pipeline = FakePipeline(bad_action=True)
        with self.assertRaisesRegex(ValueError, "finite"):
            self.run_fake(pipeline)
        self.assertEqual(pipeline.state.timestamp, 0.0)
        self.assertTrue(pipeline.closed)

    def test_bad_state_rejected_without_false_complete(self):
        pipeline = FakePipeline(bad_state=True)
        with self.assertRaisesRegex(ValueError, "finite"):
            self.run_fake(pipeline)
        self.assertTrue(pipeline.closed)
        self.assertFalse(any(event["event"] == "complete" for event in self.events))

    def test_pause_ack_freezes_physics_until_resume(self):
        acknowledged = threading.Event()
        pipeline = FakePipeline()
        errors = []

        def emit(event):
            self.events.append(event)
            if event["event"] == "frame" and event["snapshot"]["step"] == 1:
                self.pause.set()
            if event["event"] == "paused":
                acknowledged.set()

        def run():
            try:
                self.run_fake(pipeline, emit)
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=run)
        worker.start()
        try:
            self.assertTrue(acknowledged.wait(1.0))
            paused_at = pipeline.state.timestamp
            time.sleep(0.06)
            self.assertEqual(pipeline.state.timestamp, paused_at)
            self.pause.clear()
            worker.join(1.0)
            self.assertFalse(worker.is_alive())
            self.assertEqual(errors, [])
            self.assertEqual([e["event"] for e in self.events].count("paused"), 1)
            self.assertEqual([e["event"] for e in self.events].count("resumed"), 1)
        finally:
            self.stop.set()
            self.pause.clear()
            worker.join(1.0)


class ValidationTests(unittest.TestCase):
    def test_no_padding_or_truncation(self):
        for count in (23, 28, 30):
            with self.subTest(count=count), self.assertRaises(ValueError):
                backend._vector([0.0] * count, 29, "target")

    def test_nonfinite_vectors_rejected(self):
        for value in (float("nan"), float("inf"), -float("inf"), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                backend._vector([value] * 29, 29, "target")


@unittest.skipUnless(os.environ.get("TELEOPIT_TEST_ROOT"), "set TELEOPIT_TEST_ROOT for real GMR/ONNX/MuJoCo smoke")
class RealUpstreamSmoke(unittest.TestCase):
    def test_bvh_to_gmr_to_onnx_to_mujoco(self):
        root = Path(os.environ["TELEOPIT_TEST_ROOT"]).resolve()
        options = {
            "upstream_root": str(root), "source": "bvh", "render": False,
            "bvh_path": str(root / "data/sample_bvh/aiming1_subject1.bvh"),
            "policy_path": str(root / "ckpt/track_g1.onnx"), "max_steps": 3,
        }
        events = []
        backend.run_backend(options, events.append, threading.Event(), threading.Event())
        frames = [event["snapshot"] for event in events if event["event"] == "frame"]
        self.assertEqual(len(frames), 3)
        self.assertEqual(events[-1]["summary"]["reason"], "completed")
        self.assertAlmostEqual(frames[-1]["sim_time_s"], 0.06)
        self.assertEqual(len(frames[-1]["target_positions"]), 29)
        self.assertGreater(frames[-1]["policy_ms"], 0.0)
        self.assertGreater(frames[-1]["physics_ms"], 0.0)
        self.assertNotEqual(frames[0]["joint_positions"], frames[-1]["joint_positions"])
        json.dumps(events, allow_nan=False)
        forbidden = ("g1_bridge_sdk", "unitree_sdk2py", "teleopit.sim2real.unitree_g1")
        self.assertFalse(any(module in sys.modules for module in forbidden))


if __name__ == "__main__":
    unittest.main()
