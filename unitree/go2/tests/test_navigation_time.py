import unittest

from navigation_time import ClockOffsetEstimator, split_ns, stamp_to_ns


class TimestampHelpersTest(unittest.TestCase):
    def test_round_trip(self):
        timestamp_ns = stamp_to_ns(1_769_941_368, 987_654_321)
        self.assertEqual(split_ns(timestamp_ns), (1_769_941_368, 987_654_321))

    def test_invalid_nanoseconds(self):
        with self.assertRaises(ValueError):
            stamp_to_ns(1, 1_000_000_000)


class ClockOffsetEstimatorTest(unittest.TestCase):
    def make_estimator(self):
        return ClockOffsetEstimator(
            warmup_samples=3,
            window_samples=5,
            reset_threshold_ns=1_000_000_000,
            reset_confirm_samples=3,
        )

    def test_uses_minimum_arrival_latency_after_warmup(self):
        estimator = self.make_estimator()
        offset = 15_869_770_000_000_000
        source = 1_769_941_368_000_000_000

        self.assertIsNone(estimator.correct_observation(source, source + offset + 8_000_000))
        self.assertIsNone(
            estimator.correct_observation(source + 5_000_000, source + 5_000_000 + offset + 2_000_000)
        )
        corrected = estimator.correct_observation(
            source + 10_000_000, source + 10_000_000 + offset + 5_000_000
        )

        self.assertEqual(corrected, source + 10_000_000 + offset + 2_000_000)
        self.assertTrue(estimator.snapshot().ready)

    def test_one_late_callback_is_rejected_without_reset(self):
        estimator = self.make_estimator()
        source = 10_000_000_000
        offset = 20_000_000_000
        for index in range(3):
            estimator.correct_observation(
                source + index * 10_000_000,
                source + index * 10_000_000 + offset + 1_000_000,
            )

        late = estimator.correct_observation(
            source + 30_000_000,
            source + 30_000_000 + offset + 2_000_000_000,
        )
        recovered = estimator.correct_observation(
            source + 40_000_000,
            source + 40_000_000 + offset + 2_000_000,
        )

        self.assertIsNone(late)
        self.assertIsNotNone(recovered)
        self.assertEqual(estimator.snapshot().resets, 0)

    def test_persistent_backward_clock_jump_restarts_warmup(self):
        estimator = self.make_estimator()
        source = 10_000_000_000
        offset = 20_000_000_000
        for index in range(3):
            estimator.correct_observation(
                source + index * 10_000_000,
                source + index * 10_000_000 + offset,
            )

        for index in range(3):
            self.assertIsNone(
                estimator.correct_observation(
                    source + 30_000_000 + index * 10_000_000,
                    source + 30_000_000 + index * 10_000_000 + offset + 2_000_000_000,
                )
            )

        snapshot = estimator.snapshot()
        self.assertFalse(snapshot.ready)
        self.assertEqual(snapshot.resets, 1)
        self.assertEqual(snapshot.samples, 1)

    def test_forward_clock_jump_restarts_immediately(self):
        estimator = self.make_estimator()
        source = 10_000_000_000
        offset = 20_000_000_000
        for index in range(3):
            estimator.correct_observation(
                source + index * 10_000_000,
                source + index * 10_000_000 + offset,
            )

        self.assertIsNone(
            estimator.correct_observation(source + 2_000_000_000, source + offset + 100_000_000)
        )
        self.assertEqual(estimator.snapshot().resets, 1)


if __name__ == "__main__":
    unittest.main()
