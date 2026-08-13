from __future__ import annotations

import threading
import time
import unittest
import uuid

import numpy as np
from teleop.adapter import G1ControllerPoseMapper, G1DualArmAdapter
from teleop.hardware import ARM_INDICES, G1ArmSdkPort
from teleop.protocol import ProtocolError
from teleop.runtime import G1TeleopRuntime

from tests.helpers import FakeIkSolver, FakeLowStateReader, frame, session


class FakeMotorCommand:
    def __init__(self):
        self.mode = 0
        self.q = 0.0
        self.dq = 0.0
        self.tau = 0.0
        self.kp = 0.0
        self.kd = 0.0


class FakeLowCommand:
    def __init__(self):
        self.mode_pr = 0
        self.mode_machine = 0
        self.motor_cmd = [FakeMotorCommand() for _ in range(35)]
        self.crc = 0


class FakeCrc:
    def Crc(self, message):
        return 1234


class FakePublisher:
    def __init__(self):
        self.initialized = 0
        self.closed = 0
        self._lock = threading.Lock()
        self.records = []

    def Init(self):
        self.initialized += 1

    def Write(self, message, timeout=None):
        with self._lock:
            self.records.append({
                "weight": message.motor_cmd[29].q,
                "q": [message.motor_cmd[index].q for index in ARM_INDICES],
                "tau": [message.motor_cmd[index].tau for index in ARM_INDICES],
                "mode_machine": message.mode_machine,
                "crc": message.crc,
            })
        return True

    def Close(self):
        self.closed += 1

    def snapshot(self):
        with self._lock:
            return list(self.records)


class FailingZeroPublisher(FakePublisher):
    def __init__(self):
        super().__init__()
        self.fail_zero = False

    def Write(self, message, timeout=None):
        if self.fail_zero and message.motor_cmd[29].q == 0.0:
            return False
        return super().Write(message, timeout=timeout)


class StaleableLowState(FakeLowStateReader):
    def __init__(self):
        super().__init__()
        self.stale = False

    def read_arm_state(self):
        state = dict(super().read_arm_state())
        if self.stale:
            state["sample_monotonic"] = time.monotonic() - 1.0
        return state


class G1LiveRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.lowstate = FakeLowStateReader()
        self.publisher = FakePublisher()
        self.port = G1ArmSdkPort(
            self.lowstate,
            control_hz=250.0,
            ramp_seconds=0.0,
            release_seconds=0.0,
            velocity_limit_rad_s=30.0,
            command_timeout_s=0.2,
            publisher=self.publisher,
            message_factory=FakeLowCommand,
            crc=FakeCrc(),
        )
        self.adapter = G1DualArmAdapter(
            mode="live",
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=FakeIkSolver(),
            low_state_reader=self.lowstate,
            arm_sdk=self.port,
        )
        self.runtime = G1TeleopRuntime(
            mode="live",
            adapter=self.adapter,
            pose_timeout_ms=1000,
            dispatch_io_timeout_ms=150,
            dispatch_ack_timeout_ms=200,
            auto_watchdog=False,
        )
        self.identity = session()

    def tearDown(self):
        self.runtime.close()

    def _wait_until(self, predicate, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = predicate()
            if result:
                return result
            time.sleep(0.002)
        self.fail("timed out waiting for live output")

    def test_one_publisher_publish_ack_and_five_zero_weight_stop_frames(self):
        prepared = self.runtime.prepare("live", self.identity)
        self.assertEqual("prepared_live", prepared["state"])
        self.assertTrue(prepared["actuation_enabled"])
        self.assertEqual(10, len(prepared["output"]["target_joint_positions_rad"]))
        self.assertEqual(1, self.publisher.initialized)

        with self.assertRaisesRegex(RuntimeError, "already exists"):
            G1ArmSdkPort(
                FakeLowStateReader(),
                publisher=FakePublisher(),
                message_factory=FakeLowCommand,
                crc=FakeCrc(),
            )

        self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=1, clutch_sequence=7, deadman=False),
            source="test",
        )
        active = self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=2, clutch_sequence=8, deadman=True),
            source="test",
        )
        self.assertEqual("active_live", active["state"])
        published = self._wait_until(
            lambda: (
                state
                if (state := self.runtime.status())["dispatch"].get(
                    "last_published_sequence"
                ) == 2
                else None
            )
        )
        self.assertEqual("published", published["output"]["state"])
        self.assertTrue(published["output"]["hardware_output"])
        self.assertNotIn("last_would_apply_sequence", published["dispatch"])
        records_before_stop = self.publisher.snapshot()
        self.assertGreaterEqual(len(records_before_stop), 1)
        self.assertEqual(1.0, records_before_stop[-1]["weight"])
        self.assertEqual(4, records_before_stop[-1]["mode_machine"])
        self.assertEqual(1234, records_before_stop[-1]["crc"])
        self.assertEqual(10, len(records_before_stop[-1]["q"]))
        self.assertEqual(10, len(records_before_stop[-1]["tau"]))

        paused = self.runtime.pause({
            "boot_id": self.runtime.boot_id,
            **self.identity,
        })
        self.assertEqual("paused", paused["state"])
        self.assertTrue(paused["dispatch"]["stop_acknowledged"])
        records = self.publisher.snapshot()
        self.assertGreaterEqual(len(records), len(records_before_stop) + 5)
        self.assertEqual([0.0] * 5, [record["weight"] for record in records[-5:]])
        self.assertEqual(0.0, paused["output"]["arm_sdk_weight"])
        self.assertEqual(10, len(paused["output"]["target_joint_positions_rad"]))
        self.assertEqual(
            len(paused["output"]["target_joint_positions_rad"]),
            len(paused["output"]["measured_joint_positions_rad"]),
        )

        writes_after_pause = len(records)
        repeated = self.port.safe_stop(
            reason="repeated_safe_stop",
            deadline_monotonic=time.monotonic() + 0.15,
        )
        self.assertTrue(repeated.ok)
        self.assertEqual(writes_after_pause, len(self.publisher.snapshot()))

        # A completed zero-weight release may be deliberately prepared again;
        # historical writes never make startup/current safety fail.
        second = {
            "session_id": str(uuid.uuid4()),
            "epoch": 2,
            "fence": "s" * 32,
        }
        prepared_again = self.runtime.prepare("live", second)
        self.assertEqual("prepared_live", prepared_again["state"])
        self.runtime.submit_frame(
            frame(self.runtime, second, sequence=1, clutch_sequence=1, deadman=False),
            source="test",
        )
        self.runtime.submit_frame(
            frame(self.runtime, second, sequence=2, clutch_sequence=2, deadman=True),
            source="test",
        )
        self._wait_until(
            lambda: self.runtime.status()["dispatch"].get("last_published_sequence") == 2
        )
        released = self.runtime.release({
            "boot_id": self.runtime.boot_id,
            **second,
        })
        self.assertEqual("released", released["state"])
        self.assertTrue(released["dispatch"]["stop_acknowledged"])

    def test_mode_change_latches_fault_and_no_write_occurs_after_fault(self):
        self.runtime.prepare("live", self.identity)
        rtc_generation = self.runtime.session_generation()
        self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=1, clutch_sequence=1, deadman=False),
            source="test",
        )
        self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=2, clutch_sequence=2, deadman=True),
            source="test",
        )
        self._wait_until(lambda: len(self.publisher.snapshot()) >= 1)
        self.lowstate.mode_machine = 3
        fault = self._wait_until(
            lambda: (
                snapshot
                if (snapshot := self.port.snapshot())["fault_reason"] is not None
                else None
            )
        )
        self.assertEqual("hard_fault", fault["control_state"])
        terminal = self.runtime.watchdog_tick()
        self.assertEqual("fault", terminal["state"])
        self.assertEqual("dispatch_fault", terminal["reason"])
        self.assertFalse(terminal["authority_valid"])
        self.assertIsNone(terminal["session_id"])
        self.assertIsNone(terminal["lease"]["age_ms"])
        self.assertFalse(terminal["lease"]["fresh"])
        self.assertFalse(terminal["rtc"]["connected"])
        self.assertEqual("fault_latched", terminal["dispatch"]["state"])
        self.assertFalse(terminal["dispatch"]["ready"])
        self.assertFalse(terminal["dispatch"]["stop_acknowledged"])
        self.assertEqual("arm_sdk_async_fault", terminal["dispatch"]["fault_code"])
        self.assertEqual("fault", terminal["output"]["state"])
        self.assertEqual(0.0, terminal["output"]["arm_sdk_weight"])
        self.assertFalse(self.runtime.generation_matches(rtc_generation))
        with self.assertRaises(ProtocolError):
            self.runtime.submit_frame(
                frame(
                    self.runtime,
                    self.identity,
                    sequence=3,
                    clutch_sequence=3,
                    deadman=True,
                ),
                source="test",
            )
        writes_at_fault = len(self.publisher.snapshot())
        time.sleep(0.03)
        self.assertEqual(writes_at_fault, len(self.publisher.snapshot()))

    def test_autonomous_timeout_requires_neutral_then_new_clutch(self):
        self.runtime.prepare("live", self.identity)
        self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=1, clutch_sequence=1, deadman=False),
            source="test",
        )
        self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=2, clutch_sequence=2, deadman=True),
            source="test",
        )
        self._wait_until(
            lambda: self.runtime.status()["dispatch"].get("last_published_sequence") == 2
        )
        released = self._wait_until(
            lambda: (
                snapshot
                if (snapshot := self.port.external_release_signal())["acknowledged"]
                and snapshot["generation"] == 1
                else None
            ),
            timeout=1.0,
        )
        self.assertEqual("command_timeout", released["reason"])
        hold = self.runtime.watchdog_tick()
        self.assertEqual("hold", hold["state"])
        self.assertEqual("command_timeout", hold["reason"])
        self.assertEqual("safe_reclutch_required", hold["dispatch"]["state"])
        self.assertTrue(hold["dispatch"]["stop_acknowledged"])
        writes_at_release = len(self.publisher.snapshot())

        still_held = self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=3, clutch_sequence=3, deadman=True),
            source="test",
        )
        self.assertEqual("hold", still_held["state"])
        time.sleep(0.02)
        self.assertEqual(writes_at_release, len(self.publisher.snapshot()))

        neutral = self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=4, clutch_sequence=3, deadman=False),
            source="test",
        )
        self.assertEqual("hold", neutral["state"])
        resumed = self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=5, clutch_sequence=4, deadman=True),
            source="test",
        )
        self.assertEqual("active_live", resumed["state"])
        self._wait_until(
            lambda: self.runtime.status()["dispatch"].get("last_published_sequence") == 5
        )
        self.assertGreater(len(self.publisher.snapshot()), writes_at_release)

    def test_late_timeout_signal_cannot_weaken_pause_or_release(self):
        self.runtime.prepare("live", self.identity)
        paused = self.runtime.pause({"boot_id": self.runtime.boot_id, **self.identity})
        self.assertEqual("paused", paused["state"])
        with self.port._condition:
            self.port._external_release_generation += 1
            self.port._external_release_reason = "command_timeout"
            self.port._external_release_acknowledged = True
        still_paused = self.runtime.status()
        self.assertEqual("paused", still_paused["state"])
        self.assertEqual("safe_latched", still_paused["dispatch"]["state"])

        second = {
            "session_id": str(uuid.uuid4()),
            "epoch": 2,
            "fence": "r" * 32,
        }
        self.runtime.prepare("live", second)
        released = self.runtime.release({"boot_id": self.runtime.boot_id, **second})
        self.assertEqual("released", released["state"])
        with self.port._condition:
            self.port._external_release_generation += 1
            self.port._external_release_reason = "intent_expired"
            self.port._external_release_acknowledged = True
        still_released = self.runtime.status()
        self.assertEqual("released", still_released["state"])
        self.assertEqual("safe_revoked", still_released["dispatch"]["state"])

    def test_missing_or_short_full_lowstate_never_writes(self):
        self.runtime.close()

        class InvalidFullState(FakeLowStateReader):
            def __init__(self, *, missing):
                super().__init__()
                self.missing = missing

            def read_arm_state(self):
                value = dict(super().read_arm_state())
                if self.missing:
                    value.pop("all_joint_positions")
                else:
                    value["all_joint_positions"] = np.zeros(34)
                return value

        for missing in (True, False):
            with self.subTest(missing=missing):
                publisher = FakePublisher()
                port = G1ArmSdkPort(
                    InvalidFullState(missing=missing),
                    ramp_seconds=0.0,
                    release_seconds=0.0,
                    publisher=publisher,
                    message_factory=FakeLowCommand,
                    crc=FakeCrc(),
                )
                try:
                    self.assertFalse(port.startup_safe(time.monotonic() + 0.1).ok)
                    ack = port.apply_target(
                        np.zeros(10),
                        np.zeros(10),
                        expires_monotonic=time.monotonic() + 0.1,
                        required_mode_machine=4,
                        allow_arm=True,
                    )
                    self.assertFalse(ack.ok)
                    self.assertEqual([], publisher.snapshot())
                finally:
                    port.close()

    def test_velocity_limit_preserves_whole_vector_direction(self):
        self.runtime.close()
        lowstate = FakeLowStateReader()
        publisher = FakePublisher()
        port = G1ArmSdkPort(
            lowstate,
            control_hz=100.0,
            ramp_seconds=0.0,
            release_seconds=0.0,
            velocity_limit_rad_s=0.5,
            publisher=publisher,
            message_factory=FakeLowCommand,
            crc=FakeCrc(),
        )
        try:
            delta = np.linspace(0.1, 1.0, 10)
            ack = port.apply_target(
                lowstate.q + delta,
                np.zeros(10),
                expires_monotonic=time.monotonic() + 0.2,
                required_mode_machine=4,
                allow_arm=True,
            )
            self.assertTrue(ack.ok)
            actual_delta = np.asarray(publisher.snapshot()[-1]["q"]) - lowstate.q
            np.testing.assert_allclose(actual_delta / delta, np.full(10, 0.005), atol=1e-8)
        finally:
            port.close()

    def test_stale_lowstate_latches_fault_before_another_write(self):
        self.runtime.close()
        self.lowstate = StaleableLowState()
        self.publisher = FakePublisher()
        self.port = G1ArmSdkPort(
            self.lowstate,
            ramp_seconds=0.0,
            release_seconds=0.0,
            velocity_limit_rad_s=30.0,
            publisher=self.publisher,
            message_factory=FakeLowCommand,
            crc=FakeCrc(),
        )
        self.adapter = G1DualArmAdapter(
            mode="live",
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=FakeIkSolver(),
            low_state_reader=self.lowstate,
            arm_sdk=self.port,
        )
        self.runtime = G1TeleopRuntime(
            mode="live",
            adapter=self.adapter,
            pose_timeout_ms=1000,
            dispatch_io_timeout_ms=150,
            dispatch_ack_timeout_ms=200,
            auto_watchdog=False,
        )
        self.identity = session()
        self.runtime.prepare("live", self.identity)
        self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=1, clutch_sequence=1, deadman=False),
            source="test",
        )
        self.runtime.submit_frame(
            frame(self.runtime, self.identity, sequence=2, clutch_sequence=2, deadman=True),
            source="test",
        )
        self._wait_until(lambda: len(self.publisher.snapshot()) >= 1)
        self.lowstate.stale = True
        self._wait_until(lambda: self.port.snapshot()["fault_reason"] is not None)
        writes_at_fault = len(self.publisher.snapshot())
        time.sleep(0.03)
        self.assertEqual(writes_at_fault, len(self.publisher.snapshot()))

    def test_failed_zero_weight_write_is_not_counted_as_confirmed(self):
        self.runtime.close()
        lowstate = FakeLowStateReader()
        publisher = FailingZeroPublisher()
        port = G1ArmSdkPort(
            lowstate,
            ramp_seconds=0.0,
            release_seconds=0.0,
            velocity_limit_rad_s=30.0,
            publisher=publisher,
            message_factory=FakeLowCommand,
            crc=FakeCrc(),
        )
        try:
            self.assertTrue(port.startup_safe(time.monotonic() + 0.1).ok)
            applied = port.apply_target(
                np.linspace(-0.1, 0.1, 10),
                np.zeros(10),
                expires_monotonic=time.monotonic() + 0.5,
                required_mode_machine=4,
                allow_arm=True,
            )
            self.assertTrue(applied.ok)
            publisher.fail_zero = True
            stopped = port.safe_stop(
                reason="test_failed_zero",
                deadline_monotonic=time.monotonic() + 0.15,
            )
            self.assertFalse(stopped.ok)
            snapshot = port.snapshot()
            self.assertEqual("hard_fault", snapshot["control_state"])
            self.assertEqual(5, snapshot["final_zero_weight_frames_remaining"])
            self.assertNotEqual(0.0, publisher.snapshot()[-1]["weight"])
        finally:
            port.close()


if __name__ == "__main__":
    unittest.main()
