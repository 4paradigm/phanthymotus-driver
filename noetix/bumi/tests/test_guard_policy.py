"""Run with `python3 -m unittest -v test_guard_policy` or pytest."""

import unittest

from noetix.bumi.guard_policy import GuardEvent, GuardPolicy, NEUTRAL_TEXT, WELCOME_TEXT


class GuardPolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = GuardPolicy()

    def obs(self, status, ts, person_id=None, now=None):
        result = {"status": status}
        if person_id is not None:
            result["person_id"] = person_id
        return self.policy.observe(result, frame_ts=ts, now=ts if now is None else now)

    def test_matched_requires_three_distinct_frames(self):
        self.assertIsNone(self.obs("matched", 1, "p-1"))
        self.assertIsNone(self.obs("matched", 1, "p-1"))
        self.assertIsNone(self.obs("matched", 2, "p-1"))
        self.assertEqual(
            self.obs("matched", 3, "p-1"),
            GuardEvent("known_visitor_arrived", WELCOME_TEXT, "p-1"),
        )

    def test_unknown_has_neutral_fixed_text(self):
        self.obs("unknown", 1)
        self.obs("unknown", 2)
        event = self.obs("unknown", 3)
        self.assertEqual(event, GuardEvent("visitor_needs_assistance", NEUTRAL_TEXT))
        self.assertNotIn("person_id", event.as_agent_event())
        self.assertEqual(event.as_agent_event()["priority"], 1)

    def test_no_repeat_while_person_remains(self):
        for ts in (1, 2, 3):
            self.obs("matched", ts, "p-1")
        self.policy.pause()
        for ts in (4, 5, 6, 40, 41, 42):
            self.assertIsNone(self.obs("matched", ts, "p-1"))

    def test_unpublished_event_can_be_abandoned_and_reconfirmed(self):
        for ts in (1, 2, 3):
            event = self.obs("matched", ts, "p-1")
        self.assertIsNotNone(event)
        self.policy.abandon_unpublished()
        self.assertIsNone(self.obs("matched", 4, "p-1"))
        self.assertIsNone(self.obs("matched", 5, "p-1"))
        self.assertIsNotNone(self.obs("matched", 6, "p-1"))

    def test_published_event_cannot_be_abandoned_into_duplicate(self):
        for ts in (1, 2, 3):
            self.obs("matched", ts, "p-1")
        self.policy.mark_published()
        self.policy.abandon_unpublished()
        for ts in (4, 5, 6):
            self.assertIsNone(self.obs("matched", ts, "p-1"))

    def test_three_no_face_frames_and_cooldown_both_required(self):
        for ts in (1, 2, 3):
            self.obs("matched", ts, "p-1")
        for ts in (4, 5, 6):
            self.assertIsNone(self.obs("no_face", ts))
        for ts in (7, 8, 9, 32):
            self.assertIsNone(self.obs("matched", ts, "p-1"))
        self.assertIsNone(self.obs("matched", 33, "p-1"))
        self.assertIsNone(self.obs("matched", 34, "p-1"))
        self.assertEqual(self.obs("matched", 35, "p-1").kind, "known_visitor_arrived")

    def test_no_face_below_rearm_threshold_does_not_repeat(self):
        for ts in (1, 2, 3):
            self.obs("matched", ts, "p-1")
        self.obs("no_face", 4)
        self.obs("no_face", 5)
        for ts in (34, 35, 36):
            self.assertIsNone(self.obs("matched", ts, "p-1"))

    def test_identity_change_resets_confirmation(self):
        self.obs("matched", 1, "p-1")
        self.obs("matched", 2, "p-2")
        self.obs("matched", 3, "p-2")
        self.assertEqual(self.obs("matched", 4, "p-2").person_id, "p-2")

    def test_unknown_to_match_resets_confirmation(self):
        self.obs("unknown", 1)
        self.obs("unknown", 2)
        self.obs("matched", 3, "p-1")
        self.obs("matched", 4, "p-1")
        self.assertEqual(self.obs("matched", 5, "p-1").kind, "known_visitor_arrived")

    def test_multiple_faces_and_error_fail_closed(self):
        self.obs("matched", 1, "p-1")
        self.obs("matched", 2, "p-1")
        self.assertIsNone(self.obs("multiple_faces", 3))
        self.assertIsNone(self.obs("matched", 4, "p-1"))
        self.assertIsNone(self.obs("error", 5))
        self.assertIsNone(self.obs("matched", 6, "p-1"))
        self.assertIsNone(self.obs("matched", 7, "p-1"))
        self.assertIsNotNone(self.obs("matched", 8, "p-1"))

    def test_invalid_match_and_disagreeing_status_fail_closed(self):
        self.obs("matched", 1, "p-1")
        self.obs("matched", 2, "p-1")
        self.assertIsNone(self.obs("matched", 3))
        self.assertIsNone(self.policy.observe(
            {"status": "matched", "state": "unknown", "person_id": "p-1"},
            frame_ts=4, now=4,
        ))
        self.assertIsNone(self.obs("matched", 5, "p-1"))
        self.assertIsNone(self.obs("matched", 6, "p-1"))
        self.assertIsNotNone(self.obs("matched", 7, "p-1"))

    def test_stale_future_and_out_of_order_frames(self):
        self.obs("matched", 1, "p-1")
        self.assertIsNone(self.obs("matched", 2, "p-1", now=6))
        self.assertIsNone(self.obs("matched", 7, "p-1", now=6))
        self.assertIsNone(self.obs("matched", 6, "p-1", now=6))
        self.assertIsNone(self.obs("matched", 5, "p-1", now=7))
        self.assertIsNone(self.obs("matched", 7, "p-1", now=7))
        self.assertIsNotNone(self.obs("matched", 8, "p-1", now=8))
        self.assertIsNone(self.obs("matched", 9, "p-1", now=9))

    def test_long_gap_requires_new_confirmation(self):
        self.obs("unknown", 1)
        self.obs("unknown", 2)
        self.assertIsNone(self.obs("unknown", 10))
        self.assertIsNone(self.obs("unknown", 11))
        self.assertIsNotNone(self.obs("unknown", 12))


if __name__ == "__main__":
    unittest.main()
