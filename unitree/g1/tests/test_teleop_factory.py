from __future__ import annotations

import json
import threading
import time
import unittest
from collections import deque
from unittest.mock import patch

import numpy as np
from teleop.dispatch import AdapterAck
from teleop.factory import (
    G1TeleopPreflightError,
    build_g1_teleop_service,
    project_preflight_error,
)
from teleop.ik import G123PinocchioIk


class FactoryLowState:
    def __init__(self, *args, **kwargs):
        self.closed = False

    def read_arm_state(self):
        return {
            "joint_positions": np.zeros(10),
            "joint_velocities": np.zeros(10),
            "all_joint_positions": np.zeros(35),
            "mode_machine": 4,
            "sample_monotonic": time.monotonic(),
        }

    def close(self):
        self.closed = True


class FactoryIk:
    instances = []

    def __init__(self, path):
        self.warmed = False
        self.resets = []
        type(self).instances.append(self)

    def warm_up(self, q, dq):
        self.warmed = True
        return {"ready": True, "warmup_ms": 1.0}

    def ready(self):
        return self.warmed

    def reset(self, q):
        self.resets.append(np.asarray(q).copy())

    def solve(self, left, right, q, dq):
        return np.asarray(q).copy(), np.zeros(10)

    def current_targets(self, q):
        return np.eye(4), np.eye(4)


class FactoryPort:
    publisher_count = 1
    constructed_after_warmup = False

    def __init__(self, low_state, *args, **kwargs):
        type(self).constructed_after_warmup = bool(
            FactoryIk.instances and FactoryIk.instances[-1].warmed
        )

    def startup_safe(self, deadline):
        return AdapterAck(True)

    def apply_target(self, *args, **kwargs):
        return AdapterAck(True)

    def safe_stop(self, *args, **kwargs):
        return AdapterAck(True)

    def snapshot(self):
        return {"arm_sdk_weight": 0.0, "fault_reason": None}

    def external_fault_code(self):
        return None

    def external_release_signal(self):
        return {"generation": 0, "reason": None, "acknowledged": True}

    def close(self):
        return AdapterAck(True)


class CapturingService:
    def __init__(self, runtime, **kwargs):
        self.runtime = runtime
        self.startup_preflight = kwargs["startup_preflight"]
        self.live_low_state_probe = kwargs["live_low_state_probe"]
        self.ik_diagnostic = kwargs["ik_diagnostic"]
        self.ticket_secret = kwargs["ticket_secret"]

    def close(self):
        self.runtime.close()


class G1TeleopFactoryTests(unittest.TestCase):
    def setUp(self):
        FactoryIk.instances.clear()
        FactoryPort.constructed_after_warmup = False
        self.env = patch.dict(
            "os.environ",
            {
                "MOTUS_TELEOP_TICKET_SECRET": "t" * 32,
                "MOTUS_CAPTURE_WSS_URL": (
                    "wss://127.0.0.1:15702/ws/teleop-capture"
                ),
            },
            clear=False,
        )
        self.env.start()

    def tearDown(self):
        self.env.stop()

    def test_missing_or_disabled_teleop_is_a_noop(self):
        with patch("teleop.factory.G1LowStateReader") as low_state:
            self.assertIsNone(build_g1_teleop_service({}))
            self.assertIsNone(build_g1_teleop_service({"teleop": {}}))
            self.assertIsNone(
                build_g1_teleop_service({"teleop": {"enabled": False}})
            )
        low_state.assert_not_called()

    def test_factory_does_not_require_an_external_ticket_secret(self):
        config = {"teleop": {
            "enabled": True,
            "mode": "shadow",
            "capture": {
                "public_wss_url": (
                    "wss://127.0.0.1:15702/ws/teleop-capture"
                ),
            },
        }}
        with (
            patch.dict("os.environ", {}, clear=True),
            patch("teleop.factory.G1LowStateReader", FactoryLowState),
            patch("teleop.factory.G123PinocchioIk", FactoryIk),
            patch("teleop.factory.G1ArmSdkPort") as publisher,
            patch("teleop.factory.G1TeleopService", CapturingService),
        ):
            service = build_g1_teleop_service(config)
        try:
            publisher.assert_not_called()
            self.assertIsNone(service.ticket_secret)
        finally:
            service.close()

    def test_shadow_warms_real_ik_boundary_without_constructing_publisher(self):
        config = {"teleop": {"enabled": True, "mode": "shadow"}}
        with (
            patch("teleop.factory.G1LowStateReader", FactoryLowState),
            patch("teleop.factory.G123PinocchioIk", FactoryIk),
            patch("teleop.factory.G1ArmSdkPort") as publisher,
            patch("teleop.factory.G1TeleopService", CapturingService),
        ):
            service = build_g1_teleop_service(config)
        try:
            self.assertTrue(FactoryIk.instances[-1].warmed)
            self.assertIs(FactoryIk.instances[-1], service.ik_diagnostic._ik)
            publisher.assert_not_called()
            self.assertFalse(service.runtime.actuation_enabled)
            self.assertIsNone(service.live_low_state_probe)
            self.assertEqual("shadow", service.startup_preflight["mode"])
            self.assertFalse(service.startup_preflight["hardware_output"])
            self.assertFalse(service.startup_preflight["publisher_created"])
            self.assertEqual(4, service.startup_preflight["low_state"]["mode_machine"])
            self.assertLessEqual(
                service.startup_preflight["low_state"]["sample_age_ms"],
                100.0,
            )
            self.assertEqual(1.0, service.startup_preflight["ik"]["warmup_ms"])
            self.assertEqual(
                service.runtime.capability_digest,
                service.startup_preflight["identity"]["capability_digest"],
            )
        finally:
            service.close()

    def test_arm_gesture_configuration_conflict_fails_before_dependencies(self):
        config = {
            "teleop": {"enabled": True, "mode": "shadow"},
            "plugins": {"arm": {"enabled": True}},
        }
        with patch("teleop.factory.G1LowStateReader") as low_state:
            with self.assertRaisesRegex(ValueError, "arm authority must be exclusive"):
                build_g1_teleop_service(config)
        low_state.assert_not_called()

    def test_core_lease_and_fixed_velocity_contracts_fail_before_dependencies(self):
        invalid = (
            {"teleop": {"enabled": True, "mode": "shadow", "lease_timeout_ms": 749}},
            {
                "teleop": {
                    "enabled": True,
                    "mode": "live",
                    "live": {"enabled": True, "velocity_limit_rad_s": 0.6},
                }
            },
        )
        for config in invalid:
            with self.subTest(config=config):
                with patch("teleop.factory.G1LowStateReader") as low_state:
                    with self.assertRaises(ValueError):
                        build_g1_teleop_service(config)
                low_state.assert_not_called()

    def test_wrong_mode_and_failed_startup_never_return_a_ready_service(self):
        class WrongModeLowState(FactoryLowState):
            def read_arm_state(self):
                value = dict(super().read_arm_state())
                value["mode_machine"] = 3
                return value

        class ChangesModeAfterProbe(FactoryLowState):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.reads = 0

            def read_arm_state(self):
                self.reads += 1
                value = dict(super().read_arm_state())
                value["mode_machine"] = 4 if self.reads == 1 else 3
                return value

        config = {"teleop": {"enabled": True, "mode": "shadow"}}
        for low_state_type, expected_stage in (
            (WrongModeLowState, "low_state_probe"),
            (ChangesModeAfterProbe, "runtime_startup"),
        ):
            with self.subTest(low_state=low_state_type.__name__):
                with (
                    patch("teleop.factory.G1LowStateReader", low_state_type),
                    patch("teleop.factory.G123PinocchioIk", FactoryIk),
                    patch("teleop.factory.G1ArmSdkPort") as publisher,
                    patch("teleop.factory.G1TeleopService", CapturingService),
                ):
                    with self.assertRaises(G1TeleopPreflightError) as caught:
                        build_g1_teleop_service(config)
                self.assertEqual(
                    expected_stage,
                    caught.exception.preflight_status["stage"],
                )
                publisher.assert_not_called()

    def test_failed_shadow_preflight_has_stable_operator_projection(self):
        class StaleLowState(FactoryLowState):
            def read_arm_state(self):
                value = dict(super().read_arm_state())
                value["sample_monotonic"] = time.monotonic() - 1.0
                return value

        config = {"teleop": {"enabled": True, "mode": "shadow"}}
        with (
            patch("teleop.factory.G1LowStateReader", StaleLowState),
            patch("teleop.factory.G123PinocchioIk", FactoryIk),
            patch("teleop.factory.G1ArmSdkPort") as publisher,
        ):
            with self.assertRaises(G1TeleopPreflightError) as caught:
                build_g1_teleop_service(config)
        publisher.assert_not_called()
        projected = project_preflight_error(caught.exception, mode="shadow")
        self.assertFalse(projected["ready"])
        self.assertEqual("low_state_probe", projected["stage"])
        self.assertEqual("low_state_preflight_failed", projected["code"])
        self.assertEqual(
            "G1 LowState startup safety probe failed",
            projected["message"],
        )
        self.assertFalse(projected["hardware_output"])
        self.assertNotIn("d" * 32, projected["message"])

    def test_arbitrary_configuration_error_is_not_reflected_publicly(self):
        secret = "driver-secret-that-must-not-be-reflected"
        projected = project_preflight_error(
            ValueError(f"invalid /private/operator/config.yaml token={secret}"),
            mode="live",
        )
        self.assertEqual("teleop_configuration_invalid", projected["code"])
        self.assertEqual(
            "G1 teleoperation configuration is invalid",
            projected["message"],
        )
        self.assertNotIn(secret, json.dumps(projected, sort_keys=True))
        self.assertNotIn("/private/operator", json.dumps(projected, sort_keys=True))

    def test_live_publisher_is_constructed_only_after_successful_warmup(self):
        config = {
            "teleop": {
                "enabled": True,
                "mode": "live",
                "live": {"enabled": True},
            }
        }
        with (
            patch("teleop.factory.G1LowStateReader", FactoryLowState),
            patch("teleop.factory.G123PinocchioIk", FactoryIk),
            patch("teleop.factory.G1ArmSdkPort", FactoryPort),
            patch("teleop.factory.G1TeleopService", CapturingService),
        ):
            service = build_g1_teleop_service(config)
        try:
            self.assertTrue(FactoryPort.constructed_after_warmup)
            self.assertTrue(service.runtime.actuation_enabled)
            self.assertTrue(callable(service.live_low_state_probe))
            self.assertTrue(service.startup_preflight["hardware_output"])
            self.assertTrue(service.startup_preflight["publisher_created"])
        finally:
            service.close()

    def test_real_ik_reset_discards_previous_session_filter_history(self):
        solver = object.__new__(G123PinocchioIk)
        solver._lock = threading.Lock()
        solver._history = deque(
            [np.full(10, -1.0), np.full(10, 1.0)],
            maxlen=4,
        )
        solver._last_q = np.full(10, 1.0)
        measured = np.linspace(-0.2, 0.2, 10)
        solver.reset(measured)
        self.assertEqual(0, len(solver._history))
        np.testing.assert_array_equal(measured, solver._last_q)


if __name__ == "__main__":
    unittest.main()
