import threading
import time
import unittest

import numpy as np
from teleop.adapter import G1ControllerPoseMapper, G1DualArmAdapter
from teleop.dispatch import AdapterAck
from teleop.protocol import ProtocolError
from teleop.runtime import G1TeleopRuntime

from tests.helpers import FakeIkSolver, FakeLowStateReader, frame, session


class G1ShadowRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.lowstate = FakeLowStateReader()
        self.ik = FakeIkSolver()
        self.adapter = G1DualArmAdapter(
            mode="shadow",
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=self.ik,
            low_state_reader=self.lowstate,
        )
        self.runtime = G1TeleopRuntime(
            mode="shadow",
            adapter=self.adapter,
            auto_watchdog=False,
        )
        self.identity = session()

    def tearDown(self):
        self.runtime.close()

    def _wait_for_output(self, state, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self.runtime.status()
            if snapshot["output"]["state"] == state:
                return snapshot
            time.sleep(0.002)
        self.fail(f"output did not reach {state}")

    def _assert_joint_shape(self, snapshot, expected=10):
        target = snapshot["output"]["target_joint_positions_rad"]
        measured = snapshot["output"]["measured_joint_positions_rad"]
        self.assertEqual(expected, len(target), snapshot["state"])
        self.assertEqual(len(target), len(measured), snapshot["state"])

    def test_pose_mapper_matches_g1_23_controller_profile_golden_pose(self):
        raw = frame(self.runtime, self.identity, sequence=0, clutch_sequence=0, deadman=False)
        left, right = G1ControllerPoseMapper().map_frame(raw)
        np.testing.assert_allclose(left[:3, 3], [0.55, 0.2, 0.05], atol=1e-9)
        np.testing.assert_allclose(right[:3, 3], [0.55, -0.2, 0.05], atol=1e-9)
        np.testing.assert_allclose(left[:3, :3], [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [0.0, 1.0, 0.0],
        ], atol=1e-9)
        np.testing.assert_allclose(right[:3, :3], [
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
        ], atol=1e-9)

        # The deployed G1 path explicitly uses arm_reference_mode=head_yaw.
        raw["head"]["orientation"] = [0.0, 2 ** -0.5, 0.0, 2 ** -0.5]
        yawed_left, yawed_right = G1ControllerPoseMapper().map_frame(raw)
        np.testing.assert_allclose(yawed_left[:3, 3], [0.35, -0.4, 0.05], atol=1e-9)
        np.testing.assert_allclose(yawed_right[:3, 3], [-0.05, -0.4, 0.05], atol=1e-9)
        np.testing.assert_allclose(yawed_left[:3, :3], [
            [0.0, 0.0, -1.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ], atol=1e-9)
        np.testing.assert_allclose(yawed_right[:3, :3], [
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ], atol=1e-9)

    def test_shadow_requires_neutral_then_reclutch_and_never_has_publisher(self):
        self._assert_joint_shape(self.runtime.status())
        prepared = self.runtime.prepare("shadow", self.identity)
        self.assertEqual(prepared["state"], "prepared_shadow")
        self.assertFalse(prepared["actuation_enabled"])
        self.assertFalse(prepared["output"]["hardware_output"])
        self._assert_joint_shape(prepared)

        held = self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=0, clutch_sequence=7, deadman=True),
            source="test",
        )
        self.assertEqual(held["state"], "prepared_shadow")
        self.assertEqual(len(self.ik.calls), 0)

        neutral = self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=1, clutch_sequence=7, deadman=False),
            source="test",
        )
        self.assertEqual(neutral["state"], "prepared_shadow")

        active = self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=2, clutch_sequence=8, deadman=True),
            source="test",
        )
        self.assertEqual(active["state"], "active_shadow")
        snapshot = self._wait_for_output("would_apply")
        self.assertEqual(snapshot["dispatch"]["last_would_apply_sequence"], 2)
        self.assertNotIn("last_published_sequence", snapshot["dispatch"])
        self.assertGreaterEqual(self.lowstate.reads, 4)
        self.assertEqual(len(self.ik.calls), 1)
        self.assertEqual(len(snapshot["output"]["target_joint_positions_rad"]), 10)
        self.assertEqual(len(snapshot["output"]["measured_joint_positions_rad"]), 10)
        self.assertAlmostEqual(snapshot["output"]["max_abs_error_rad"], 0.1)
        self.assertEqual(
            set(snapshot["diagnostics"]["latency_ms"]),
            {"receive_to_admit", "mailbox_wait", "ik", "adapter_apply", "robot_follow"},
        )

        paused = self.runtime.pause({
            "boot_id": self.runtime.boot_id,
            **self.identity,
        })
        self.assertEqual(paused["state"], "paused")
        self.assertTrue(paused["dispatch"]["stop_acknowledged"])
        self.assertEqual(paused["output"]["state"], "stopped")
        self._assert_joint_shape(paused)

        released = self.runtime.release({
            "boot_id": self.runtime.boot_id,
            **self.identity,
        })
        self.assertEqual("released", released["state"])
        self._assert_joint_shape(released)

    def test_hold_and_fault_preserve_equal_ten_joint_vectors(self):
        self.runtime.prepare("shadow", self.identity)
        self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=1, clutch_sequence=1, deadman=False),
            source="test",
        )
        self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=2, clutch_sequence=2, deadman=True),
            source="test",
        )
        self._wait_for_output("would_apply")
        held = self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=3, clutch_sequence=2, deadman=False),
            source="test",
        )
        self.assertEqual("hold", held["state"])
        stopped = self._wait_for_output("stopped")
        self._assert_joint_shape(stopped)

        class FailingIk(FakeIkSolver):
            def solve(self, *args):
                raise RuntimeError("deterministic IK failure")

        failing_adapter = G1DualArmAdapter(
            mode="shadow",
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=FailingIk(),
            low_state_reader=FakeLowStateReader(),
        )
        failing = G1TeleopRuntime(
            mode="shadow",
            adapter=failing_adapter,
            auto_watchdog=False,
        )
        failing_identity = session()
        try:
            failing.prepare("shadow", failing_identity)
            failing.submit_frame(
                frame(failing, failing_identity, sequence=1, clutch_sequence=1, deadman=False),
                source="test",
            )
            failing.submit_frame(
                frame(failing, failing_identity, sequence=2, clutch_sequence=2, deadman=True),
                source="test",
            )
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                fault = failing.status()
                if fault["state"] == "fault":
                    break
                time.sleep(0.002)
            else:
                self.fail("IK failure did not latch the runtime fault")
            self._assert_joint_shape(fault)
        finally:
            failing.close()

    def test_mode_mismatch_fails_before_mapping_or_ik(self):
        self.runtime.prepare("shadow", self.identity)
        wrong = frame(self.runtime, self.identity, sequence=0, clutch_sequence=0, deadman=False)
        wrong["mode"] = "live"
        with self.assertRaisesRegex(ProtocolError, "mode must be 'shadow'"):
            self.runtime.submit_frame(wrong, source="test")
        self.assertEqual(len(self.ik.calls), 0)

    def test_mailbox_expiry_is_projected_to_runtime_hold_without_new_frame(self):
        class Clock:
            def __init__(self):
                self.value = 0.0

            def __call__(self):
                return self.value

        class BlockingRecordingAdapter:
            hardware_output = False

            def __init__(self):
                self.entered = threading.Event()
                self.release = threading.Event()

            def startup_safe(self, deadline):
                return AdapterAck(True)

            def apply(self, intent):
                if intent.sequence == 2:
                    self.entered.set()
                    self.release.wait(1.0)
                return AdapterAck(True)

            def safe_stop(self, request):
                return AdapterAck(True)

            def snapshot(self):
                output = {
                    "profile_id": "unitree_g1_23_dual_arm_controller_v1",
                    "hardware_output": False,
                    "state": "safe",
                    "target_joint_positions_rad": [0.0] * 10,
                    "measured_joint_positions_rad": [0.0] * 10,
                    "max_abs_error_rad": 0.0,
                    "arm_sdk_weight": None,
                    "command_age_ms": None,
                    "fault_reason": None,
                }
                return {
                    "kind": "recording",
                    "closed": False,
                    "current": {"kind": "safe"},
                    "records": [],
                    "diagnostics": {"latency_ms": {}},
                    "output": output,
                }

            def close(self):
                return AdapterAck(True)

        clock = Clock()
        adapter = BlockingRecordingAdapter()
        runtime = G1TeleopRuntime(
            mode="shadow",
            adapter=adapter,
            clock=clock,
            pose_timeout_ms=1000,
            auto_watchdog=False,
        )
        identity = session()
        try:
            runtime.prepare("shadow", identity)
            runtime.submit_frame(
                frame(runtime, identity, sequence=1, clutch_sequence=1, deadman=False),
                source="test",
            )
            runtime.submit_frame(
                frame(runtime, identity, sequence=2, clutch_sequence=2, deadman=True),
                source="test",
            )
            self.assertTrue(adapter.entered.wait(1.0))
            runtime._pose_timeout = 0.2
            runtime.submit_frame(
                frame(runtime, identity, sequence=3, clutch_sequence=2, deadman=True),
                source="test",
            )
            clock.value = 0.3
            adapter.release.set()
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                status = runtime.status()
                if status["state"] == "hold":
                    break
                time.sleep(0.002)
            else:
                self.fail("expired mailbox intent was not projected to HOLD")
            self.assertEqual("pose_timeout", status["reason"])
            self.assertEqual("safe_reclutch_required", status["dispatch"]["state"])
            self.assertTrue(status["dispatch"]["stop_acknowledged"])
        finally:
            adapter.release.set()
            runtime.close()


if __name__ == "__main__":
    unittest.main()
