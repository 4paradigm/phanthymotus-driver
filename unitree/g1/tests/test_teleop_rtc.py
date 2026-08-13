from __future__ import annotations

import asyncio
import json
import threading
import time
import unittest

from aiortc import RTCPeerConnection, RTCSessionDescription
from teleop.adapter import G1ControllerPoseMapper, G1DualArmAdapter
from teleop.descriptor import SIGNALING_AUDIENCE
from teleop.dispatch import AdapterAck
from teleop.protocol import (
    ProtocolError,
    TicketCodec,
    TicketError,
    TicketVerifier,
    bind_rtc_frame_v1,
    make_ticket_claims,
)
from teleop.rtc import RtcManager, RtcRequestError
from teleop.runtime import G1TeleopRuntime
from teleop.service import G1TeleopService

from tests.helpers import (
    capture_config,
    FakeIkDiagnostic,
    FakeIkSolver,
    FakeLowStateReader,
    rtc_frame,
    session,
    startup_preflight,
)

DRIVER_TOKEN = "driver-token-for-local-g1-rtc-tests"
TICKET_SECRET = "ticket-secret-for-local-g1-rtc-tests-0123456789"


class HealthArmSdk:
    publisher_count = 1

    def __init__(self):
        self.output_calls = []

    def startup_safe(self, deadline):
        self.output_calls.append("startup_safe")
        return AdapterAck(True)

    def apply_target(self, *args, **kwargs):
        self.output_calls.append("apply_target")
        return AdapterAck(True)

    def safe_stop(self, *args, **kwargs):
        self.output_calls.append("safe_stop")
        return AdapterAck(True)

    def snapshot(self):
        return {"arm_sdk_weight": 0.0, "fault_reason": None}

    def external_fault_code(self):
        return None

    def external_release_signal(self):
        return {"generation": 0, "reason": None, "acknowledged": True}

    def close(self):
        self.output_calls.append("close")
        return AdapterAck(True)


class G1LocalAiortcTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.low_state = FakeLowStateReader()
        adapter = G1DualArmAdapter(
            mode="shadow",
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=FakeIkSolver(),
            low_state_reader=self.low_state,
        )
        self.runtime = G1TeleopRuntime(
            mode="shadow",
            adapter=adapter,
            lease_timeout_ms=15_000,
            pose_timeout_ms=1_000,
            auto_watchdog=False,
        )
        self.service = G1TeleopService(
            self.runtime,
            ticket_secret=TICKET_SECRET,
            startup_preflight=startup_preflight(self.runtime),
            ik_diagnostic=FakeIkDiagnostic(),
            capture_config=capture_config(),
            start_capture_listener=False,
            offer_timeout_s=10.0,
        )
        self.peer = RTCPeerConnection()

    async def asyncTearDown(self):
        await self.peer.close()
        await asyncio.to_thread(self.service.close)

    async def _wait_until(self, predicate, *, timeout: float = 10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            await asyncio.sleep(0.01)
        self.fail("timed out waiting for RTC state")

    async def test_startup_preflight_binds_descriptor_identity_and_running_rtc(self):
        preflight = self.service.preflight_status()
        self.assertTrue(preflight["ready"])
        self.assertEqual("complete", preflight["stage"])
        self.assertEqual("shadow", preflight["mode"])
        self.assertFalse(preflight["hardware_output"])
        self.assertFalse(preflight["publisher_created"])
        self.assertTrue(preflight["low_state"]["ready"])
        self.assertTrue(preflight["ik"]["ready"])
        self.assertTrue(preflight["descriptor"]["ready"])
        self.assertEqual(
            self.runtime.capability_digest,
            preflight["descriptor"]["capability_digest"],
        )
        self.assertEqual("unitree-g1", preflight["descriptor"]["driver_id"])
        self.assertTrue(preflight["rtc"]["event_loop_running"])
        self.assertEqual("motus-teleop-rtc", preflight["rtc"]["audience"])
        self.assertEqual(
            "/ws/teleop-capture",
            preflight["rtc"]["capture_path"],
        )
        self.assertIsNone(preflight["rtc"]["offer_path"])
        health = self.service.health()
        self.assertTrue(health["preflight"]["ready"])
        self.assertFalse(health["ready"])
        self.assertEqual("not_started", health["registration"]["state"])
        self.assertIsNone(health["live_low_state"])
        reads_before = self.low_state.reads
        self.service.health()
        self.assertEqual(reads_before, self.low_state.reads)

    async def test_stock_core_lifecycle_actions_and_instance_id_are_compatible(self):
        started = self.service.dispatch(
            "teleop_session",
            {"action": "start", "instance_id": "card-instance-1"},
        )
        self.assertEqual("prepared_shadow", started["state"])
        self.assertEqual("card-instance-1", started["instance_id"])
        self.assertFalse(started["actuation_enabled"])
        self.assertFalse(started["publisher_present"])
        self.assertFalse(started["output_active"])
        self.assertTrue(self.runtime.status()["authority_valid"])
        self.assertFalse(started["lease"]["armed"])

        info = self.service.dispatch(
            "teleop_session",
            {"action": "info", "instance_id": "card-instance-1"},
        )
        self.assertEqual("prepared_shadow", info["state"])
        status = self.service.dispatch(
            "teleop_session",
            {"action": "status", "instance_id": "card-instance-1"},
        )
        self.assertEqual("prepared_shadow", status["state"])
        resource = self.service.dispatch(
            "teleop_state",
            {"instance_id": "card-instance-1"},
        )
        self.assertEqual("prepared_shadow", resource["state"])
        ik = self.service.dispatch(
            "teleop_ik",
            {"action": "status", "instance_id": "card-instance-1"},
        )
        self.assertFalse(ik["diagnostic_hardware_output"])
        self.assertFalse(ik["publisher_present"])

        stopped = self.service.dispatch(
            "teleop_session",
            {"action": "stop", "instance_id": "card-instance-1"},
        )
        self.assertEqual("released", stopped["state"])

    async def test_stock_project_lifecycle_matrix_covers_all_three_cards(self):
        tools = ("teleop_session", "teleop_state", "teleop_ik")
        for action in ("start", "info"):
            for tool in tools:
                with self.subTest(tool=tool, action=action):
                    result = self.service.dispatch(
                        tool,
                        {"action": action, "instance_id": f"{tool}-card"},
                    )
                    self.assertEqual([], result["topic_out"])
                    self.assertEqual(f"{tool}-card", result["instance_id"])
                    if tool == "teleop_session":
                        self.assertEqual(action, result["lifecycle_action"])
                    else:
                        self.assertTrue(result["compatibility_lifecycle_only"])
                        self.assertFalse(result["lifecycle_action_output_applied"])
        self.assertEqual("prepared_shadow", self.runtime.status()["state"])
        self.assertTrue(self.runtime.status()["authority_valid"])
        for tool in ("teleop_state", "teleop_ik"):
            with self.subTest(tool=tool, action="stop"):
                result = self.service.dispatch(
                    tool,
                    {"action": "stop", "instance_id": f"{tool}-card"},
                )
                self.assertEqual([], result["topic_out"])
                self.assertTrue(self.runtime.status()["authority_valid"])
                self.assertEqual("prepared_shadow", self.runtime.status()["state"])

        stopped = self.service.dispatch(
            "teleop_session",
            {"action": "stop", "instance_id": "teleop_session-card"},
        )
        self.assertEqual([], stopped["topic_out"])
        self.assertEqual("teleop_session-card", stopped["instance_id"])
        self.assertEqual("released", stopped["state"])
        self.assertFalse(stopped["authority_valid"])

    async def test_authenticated_offer_ticket_replay_and_neutral_reclutch(self):
        identity = session()
        prepared = self.runtime.prepare("shadow", identity)

        control_open = asyncio.Event()
        pose_open = asyncio.Event()
        control = self.peer.createDataChannel("teleop-control", ordered=True)
        pose = self.peer.createDataChannel(
            "teleop-pose",
            ordered=False,
            maxRetransmits=0,
        )

        @control.on("open")
        def on_control_open():
            control_open.set()

        @pose.on("open")
        def on_pose_open():
            pose_open.set()

        await self.peer.setLocalDescription(await self.peer.createOffer())
        offer_sdp = self.peer.localDescription.sdp
        claims = make_ticket_claims(
            session={
                "boot_id": self.runtime.boot_id,
                "session_id": identity["session_id"],
                "epoch": identity["epoch"],
                "fence": identity["fence"],
                "capability_digest": prepared["capability_digest"],
            },
            sdp=offer_sdp,
            jti="g1_local_rtc_ticket_1234",
        )
        offer = {
            "type": "offer",
            "sdp": offer_sdp,
            "ticket": TicketCodec(TICKET_SECRET).sign(claims),
        }
        answer = await asyncio.to_thread(self.service._accept_capture_offer, offer)
        self.assertEqual("shadow", answer["mode"])
        self.assertFalse(answer["actuation_enabled"])
        self.assertNotIn("fence", answer)

        with self.assertRaises(RtcRequestError) as replay:
            await asyncio.to_thread(self.service._accept_capture_offer, offer)
        self.assertEqual(401, replay.exception.status)
        self.assertEqual("ticket_replayed", replay.exception.code)

        await self.peer.setRemoteDescription(
            RTCSessionDescription(sdp=answer["sdp"], type=answer["type"])
        )
        await asyncio.wait_for(
            asyncio.gather(control_open.wait(), pose_open.wait()),
            timeout=10.0,
        )
        await self._wait_until(lambda: self.runtime.status()["rtc"]["connected"])

        # A newly prepared or reconnected peer must first show both grips
        # released; an already-held grip can never activate hardware output.
        pose.send(json.dumps(rtc_frame(
            self.runtime,
            identity,
            sequence=1,
            clutch_sequence=7,
            deadman=True,
        )))
        await self._wait_until(
            lambda: self.runtime.status()["pose"]["latest_sequence"] == 1
        )
        self.assertEqual("prepared_shadow", self.runtime.status()["state"])

        pose.send(json.dumps(rtc_frame(
            self.runtime,
            identity,
            sequence=2,
            clutch_sequence=7,
            deadman=False,
        )))
        await self._wait_until(
            lambda: self.runtime.status()["pose"]["latest_sequence"] == 2
        )
        self.assertEqual("prepared_shadow", self.runtime.status()["state"])

        pose.send(json.dumps(rtc_frame(
            self.runtime,
            identity,
            sequence=3,
            clutch_sequence=8,
            deadman=True,
        )))
        state = await self._wait_until(
            lambda: (
                snapshot
                if (snapshot := self.runtime.status())["dispatch"].get(
                    "last_would_apply_sequence"
                ) == 3
                else None
            )
        )
        self.assertEqual("active_shadow", state["state"])
        self.assertEqual("would_apply", state["output"]["state"])
        self.assertFalse(state["output"]["hardware_output"])


class G1RtcNegativeContractTests(unittest.TestCase):
    def test_ticket_audience_sdp_and_session_binding_are_independent(self):
        identity = session()
        expected = {
            "boot_id": "00000000-0000-0000-0000-000000000001",
            **identity,
            "capability_digest": "a" * 64,
        }
        codec = TicketCodec(TICKET_SECRET)

        cases = (
            (
                "invalid_audience",
                make_ticket_claims(
                    session=expected,
                    sdp="offer-a",
                    audience="teleop-shadow-rtc",
                    jti="wrong_audience_001",
                ),
                expected,
                "offer-a",
            ),
            (
                "sdp_mismatch",
                make_ticket_claims(
                    session=expected,
                    sdp="offer-a",
                    audience=SIGNALING_AUDIENCE,
                    jti="wrong_sdp_value_01",
                ),
                expected,
                "offer-b",
            ),
            (
                "binding_mismatch",
                make_ticket_claims(
                    session=expected,
                    sdp="offer-a",
                    audience=SIGNALING_AUDIENCE,
                    jti="wrong_binding_0001",
                ),
                {**expected, "session_id": session()["session_id"]},
                "offer-a",
            ),
        )
        for code, claims, binding, sdp in cases:
            with self.subTest(code=code):
                verifier = TicketVerifier(codec, audience=SIGNALING_AUDIENCE)
                with self.assertRaises(TicketError) as caught:
                    verifier.verify_and_consume(
                        codec.sign(claims),
                        expected=binding,
                        sdp=sdp,
                    )
                self.assertEqual(code, caught.exception.code)

    def test_browser_frame_cannot_supply_private_authority(self):
        adapter = G1DualArmAdapter(
            mode="shadow",
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=FakeIkSolver(),
            low_state_reader=FakeLowStateReader(),
        )
        runtime = G1TeleopRuntime(mode="shadow", adapter=adapter, auto_watchdog=False)
        identity = session()
        try:
            public = rtc_frame(
                runtime,
                identity,
                sequence=1,
                clutch_sequence=1,
                deadman=False,
            )
            public["session_id"] = identity["session_id"]
            authority = {
                "boot_id": runtime.boot_id,
                **identity,
            }
            with self.assertRaises(ProtocolError) as caught:
                bind_rtc_frame_v1(
                    public,
                    authority=authority,
                    expected_mode="shadow",
                )
            self.assertEqual("unknown_field", caught.exception.code)
        finally:
            runtime.close()


    def test_rtc_heartbeat_never_renews_core_lease(self):
        class Clock:
            def __init__(self):
                self.value = time.monotonic()

            def __call__(self):
                return self.value

        class Channel:
            readyState = "open"

            def __init__(self):
                self.sent = []

            def send(self, value):
                self.sent.append(json.loads(value))

        clock = Clock()

        class ClockedLowState(FakeLowStateReader):
            def read_arm_state(self):
                value = dict(super().read_arm_state())
                value["sample_monotonic"] = clock()
                return value

        adapter = G1DualArmAdapter(
            mode="shadow",
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=FakeIkSolver(),
            low_state_reader=ClockedLowState(),
            clock=clock,
        )
        runtime = G1TeleopRuntime(
            mode="shadow",
            adapter=adapter,
            lease_timeout_ms=100,
            clock=clock,
            auto_watchdog=False,
        )
        identity = session()
        channel = Channel()
        try:
            runtime.prepare("shadow", identity)
            RtcManager(runtime, None)._handle_control_message(
                channel,
                json.dumps({"type": "heartbeat", "request_id": "rtc-heartbeat"}),
            )
            self.assertFalse(channel.sent[-1]["ok"])
            self.assertEqual(
                "rtc_cannot_renew_lease",
                channel.sent[-1]["error"]["code"],
            )
            self.assertFalse(channel.sent[-1]["lease_renewed"])
            clock.value += 0.101
            expired = runtime.watchdog_tick()
            self.assertFalse(expired["authority_valid"])
            self.assertIsNone(expired["session_id"])
            self.assertEqual("lease_timeout", expired["reason"])
        finally:
            runtime.close()


class G1LiveHealthTests(unittest.TestCase):
    def setUp(self):
        class HealthLowState(FakeLowStateReader):
            stale = False

            def read_arm_state(low_state_self):
                sample = super().read_arm_state()
                if low_state_self.stale:
                    sample["sample_monotonic"] = time.monotonic() - 1.0
                return sample

        self.low_state = HealthLowState()
        self.arm_sdk = HealthArmSdk()
        adapter = G1DualArmAdapter(
            mode="live",
            pose_mapper=G1ControllerPoseMapper(),
            ik_solver=FakeIkSolver(),
            low_state_reader=self.low_state,
            arm_sdk=self.arm_sdk,
        )
        self.runtime = G1TeleopRuntime(
            mode="live",
            adapter=adapter,
            auto_watchdog=False,
        )
        self.service = G1TeleopService(
            self.runtime,
            ticket_secret=TICKET_SECRET,
            startup_preflight=startup_preflight(self.runtime),
            ik_diagnostic=FakeIkDiagnostic(),
            capture_config=capture_config(),
            start_capture_listener=False,
            live_low_state_probe=self.low_state.read_arm_state,
        )
        self.service.update_registration_status(state="registered")

    def tearDown(self):
        self.service.close()

    def test_live_card_start_reports_configured_output_but_applies_no_command(self):
        output_calls = list(self.arm_sdk.output_calls)

        result = self.service.dispatch(
            "teleop_session",
            {"action": "start", "instance_id": "live-card"},
        )

        self.assertTrue(result["actuation_enabled"])
        self.assertTrue(result["publisher_present"])
        self.assertFalse(result["output_active"])
        self.assertEqual("prepared_live", result["state"])
        self.assertTrue(self.runtime.status()["authority_valid"])
        self.assertNotIn("apply_target", self.arm_sdk.output_calls)
        self.assertGreaterEqual(
            self.arm_sdk.output_calls.count("safe_stop"),
            output_calls.count("safe_stop"),
        )
        ik = self.service.dispatch(
            "teleop_ik",
            {"action": "status", "instance_id": "live-ik-card"},
        )
        self.assertFalse(ik["diagnostic_hardware_output"])
        self.assertFalse(ik["diagnostic_publisher_present"])
        self.assertFalse(ik["diagnostic_output_active"])
        self.assertTrue(ik["actuation_enabled"])
        self.assertTrue(ik["publisher_present"])
        self.assertFalse(ik["output_active"])

    def test_live_health_turns_red_when_mode_machine_leaves_ai_without_output(self):
        before = self.service.health()
        self.assertTrue(before["ready"])
        self.assertTrue(before["live_low_state"]["ready"])
        output_calls = list(self.arm_sdk.output_calls)

        self.low_state.mode_machine = 3
        after = self.service.health()

        self.assertFalse(after["ready"])
        self.assertEqual("mode_machine_not_ai", after["live_low_state"]["code"])
        self.assertEqual(3, after["live_low_state"]["mode_machine"])
        self.assertEqual(output_calls, self.arm_sdk.output_calls)

    def test_live_health_turns_red_for_stale_low_state_without_output(self):
        self.low_state.stale = True
        output_calls = list(self.arm_sdk.output_calls)

        health = self.service.health()

        self.assertFalse(health["ready"])
        self.assertEqual("low_state_stale", health["live_low_state"]["code"])
        self.assertGreaterEqual(health["live_low_state"]["sample_age_ms"], 999.0)
        self.assertEqual(output_calls, self.arm_sdk.output_calls)

    def test_live_health_cannot_return_green_after_concurrent_close(self):
        probe_entered = threading.Event()
        release_probe = threading.Event()
        original_read = self.low_state.read_arm_state

        def blocked_read():
            probe_entered.set()
            if not release_probe.wait(1.0):
                raise RuntimeError("test probe was not released")
            return original_read()

        self.service._live_low_state_probe = blocked_read
        results = []
        worker = threading.Thread(target=lambda: results.append(self.service.health()))
        worker.start()
        self.assertTrue(probe_entered.wait(1.0))

        # close() must not wait for a diagnostics read; once it has returned,
        # the blocked health call may only publish a red service_closed result.
        self.service.close()
        output_calls_after_close = list(self.arm_sdk.output_calls)
        release_probe.set()
        worker.join(1.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual(1, len(results))
        self.assertFalse(results[0]["ready"])
        self.assertEqual(
            "service_closed",
            results[0]["live_low_state"]["code"],
        )
        self.assertEqual(output_calls_after_close, self.arm_sdk.output_calls)


if __name__ == "__main__":
    unittest.main()
