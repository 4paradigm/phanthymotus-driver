from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from protocol import (
    CAPABILITIES,
    CAPABILITY_DIGEST,
    FRAME_V1_DESCRIPTION,
    MAX_SEQUENCE,
    RTC_FRAME_V1_DESCRIPTION,
    ProtocolError,
    TicketCodec,
    TicketError,
    TicketVerifier,
    bind_rtc_frame_v1,
    make_ticket_claims,
    validate_frame_v1,
)
from rtc import RtcManager
from runtime import ShadowRuntime

from tests.helpers import (
    TEST_SECRET,
    FakeClock,
    new_session,
    rtc_wire_frame,
    valid_frame,
)


class FrameV1Tests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.runtime = ShadowRuntime(clock=self.clock, auto_watchdog=False)
        self.addCleanup(self.runtime.close)
        self.session = new_session()
        self.runtime.prepare_shadow(self.session)

    def test_valid_frame_is_normalized(self):
        frame = validate_frame_v1(valid_frame(self.runtime, self.session))
        self.assertEqual(1, frame["schema_version"])
        self.assertEqual("shadow", frame["mode"])
        self.assertEqual([0.0, 0.0], frame["controllers"]["left"]["axes"])

    def test_unknown_fields_are_rejected(self):
        frame = valid_frame(self.runtime, self.session)
        frame["vendor_extension"] = True
        with self.assertRaisesRegex(ProtocolError, "unknown fields"):
            validate_frame_v1(frame)

    def test_nan_and_bad_quaternion_are_rejected(self):
        frame = valid_frame(self.runtime, self.session)
        frame["base_twist"]["linear"][0] = math.nan
        with self.assertRaisesRegex(ProtocolError, "finite"):
            validate_frame_v1(frame)
        frame = valid_frame(self.runtime, self.session)
        frame["head"]["orientation"] = [0.0, 0.0, 0.0, 0.2]
        with self.assertRaisesRegex(ProtocolError, "normalized"):
            validate_frame_v1(frame)

    def test_tracking_false_requires_null_pose(self):
        frame = valid_frame(self.runtime, self.session, tracking=False)
        frame["head"] = {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]}
        with self.assertRaisesRegex(ProtocolError, "tracking is invalid"):
            validate_frame_v1(frame)
        frame = valid_frame(self.runtime, self.session, tracking=True)
        frame["head"] = None
        with self.assertRaisesRegex(ProtocolError, "tracking is valid"):
            validate_frame_v1(frame)

    def test_sequence_fields_accept_signed_64_bit_maximum(self):
        frame = valid_frame(self.runtime, self.session)
        frame["sequence"] = MAX_SEQUENCE
        frame["clutch_sequence"] = MAX_SEQUENCE

        normalized = validate_frame_v1(frame)

        self.assertEqual(MAX_SEQUENCE, normalized["sequence"])
        self.assertEqual(MAX_SEQUENCE, normalized["clutch_sequence"])

    def test_sequence_fields_reject_values_above_signed_64_bit_maximum(self):
        for field in ("sequence", "clutch_sequence"):
            with self.subTest(field=field):
                frame = valid_frame(self.runtime, self.session)
                frame[field] = MAX_SEQUENCE + 1

                with self.assertRaises(ProtocolError) as raised:
                    validate_frame_v1(frame)

                self.assertEqual("out_of_range", raised.exception.code)

    def test_capability_schema_advertises_sequence_bounds(self):
        expected = {"minimum": 0, "maximum": MAX_SEQUENCE}
        bounds = FRAME_V1_DESCRIPTION["integer_bounds"]
        self.assertEqual(expected, bounds["sequence"])
        self.assertEqual(expected, bounds["clutch_sequence"])
        self.assertEqual(bounds, CAPABILITIES["frame"]["integer_bounds"])

    def test_rtc_wire_frame_is_bound_server_side_without_private_identity(self):
        wire = rtc_wire_frame(self.runtime, self.session, sequence=7)
        authority, _generation = self.runtime.rtc_authority_snapshot()

        normalized = bind_rtc_frame_v1(wire, authority=authority)

        self.assertEqual(7, normalized["sequence"])
        self.assertEqual(self.runtime.boot_id, normalized["boot_id"])
        self.assertEqual(self.session["session_id"], normalized["session_id"])
        self.assertEqual(self.session["fence"], normalized["fence"])

    def test_rtc_wire_frame_rejects_client_supplied_private_identity(self):
        wire = rtc_wire_frame(self.runtime, self.session)
        wire["fence"] = self.session["fence"]
        authority, _generation = self.runtime.rtc_authority_snapshot()

        with self.assertRaises(ProtocolError) as raised:
            bind_rtc_frame_v1(wire, authority=authority)

        self.assertEqual("unknown_field", raised.exception.code)

    def test_rtc_capability_contract_has_no_private_identity_fields(self):
        private = {"boot_id", "session_id", "epoch", "fence"}
        self.assertTrue(private.isdisjoint(RTC_FRAME_V1_DESCRIPTION["required"]))
        self.assertEqual(
            sorted(private),
            sorted(RTC_FRAME_V1_DESCRIPTION["forbidden_private_fields"]),
        )
        self.assertEqual(
            RTC_FRAME_V1_DESCRIPTION,
            CAPABILITIES["rtc_frame"],
        )
        self.assertEqual(
            "motus.teleop.rtc-frame.v1",
            CAPABILITIES["rtc_frame"]["protocol"],
        )
        self.assertEqual(
            ["peer_ping", "status"],
            CAPABILITIES["rtc_control"]["allowed"],
        )
        self.assertFalse(
            CAPABILITIES["rtc_control"]["private_authority_fields_allowed"]
        )


class RtcMessageDecodingTests(unittest.TestCase):
    def setUp(self):
        self.runtime = ShadowRuntime(auto_watchdog=False)
        self.addCleanup(self.runtime.close)
        self.rtc = RtcManager(self.runtime, None)

    def test_pose_huge_integer_is_stable_invalid_json(self):
        message = '{"sequence":' + ('9' * 5000) + '}'

        with self.assertRaises(ProtocolError) as raised:
            self.rtc._decode_message(message, maximum=64 * 1024)

        self.assertEqual("invalid_json", raised.exception.code)

    def test_control_deep_json_is_stable_invalid_json(self):
        with (
            mock.patch("rtc.json.loads", side_effect=RecursionError),
            self.assertRaises(ProtocolError) as raised,
        ):
            self.rtc._decode_message("[]", maximum=8 * 1024)

        self.assertEqual("invalid_json", raised.exception.code)

    def test_unencodable_text_is_stable_invalid_encoding(self):
        message = '"' + chr(0xD800) + '"'

        with self.assertRaises(ProtocolError) as raised:
            self.rtc._decode_message(message, maximum=8 * 1024)

        self.assertEqual("invalid_encoding", raised.exception.code)


class TicketTests(unittest.TestCase):
    def setUp(self):
        self.wall = FakeClock(1_000)
        self.codec = TicketCodec(TEST_SECRET)
        self.verifier = TicketVerifier(self.codec, wall_clock=self.wall)
        self.session = {
            "boot_id": "11111111-1111-1111-1111-111111111111",
            "session_id": "22222222-2222-2222-2222-222222222222",
            "epoch": 7,
            "fence": "f" * 32,
            "capability_digest": CAPABILITY_DIGEST,
        }
        self.sdp = "v=0\r\nt=test-offer\r\n"

    def ticket(self, *, jti: str = "ticket_identifier_1234", sdp: str | None = None) -> str:
        claims = make_ticket_claims(
            session=self.session,
            sdp=sdp or self.sdp,
            wall_clock=self.wall,
            jti=jti,
        )
        return self.codec.sign(claims)

    def test_ticket_is_bound_and_one_time(self):
        ticket = self.ticket()
        claims = self.verifier.verify_and_consume(ticket, expected=self.session, sdp=self.sdp)
        self.assertEqual(self.session["fence"], claims["fence"])
        with self.assertRaisesRegex(TicketError, "already been used"):
            self.verifier.verify_and_consume(ticket, expected=self.session, sdp=self.sdp)

    def test_wrong_sdp_and_expired_ticket_are_rejected(self):
        with self.assertRaisesRegex(TicketError, "SDP"):
            self.verifier.verify_and_consume(self.ticket(), expected=self.session, sdp="other")
        ticket = self.ticket(jti="ticket_identifier_5678")
        self.wall.advance(31)
        with self.assertRaisesRegex(TicketError, "expired"):
            self.verifier.verify_and_consume(ticket, expected=self.session, sdp=self.sdp)

    def test_short_secret_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "32 bytes"):
            TicketCodec("too-short")

    def test_ticket_secret_must_be_valid_utf8_without_disclosure(self):
        secret = "private-prefix-" + ("\ud800" * 32)
        with self.assertRaisesRegex(ValueError, "valid UTF-8") as raised:
            TicketCodec(secret)
        self.assertNotIn("private-prefix", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
