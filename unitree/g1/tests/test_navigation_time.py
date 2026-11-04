import random
import threading
import unittest

from navigation_time import ClockOffsetEstimator, NavigationClockOffsetEstimator, split_ns, stamp_to_ns


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


class SlidingClockTest(unittest.TestCase):
    SOURCE = 10_000_000_000_000
    OFFSET = 20_000_000_000

    def observe(self, estimator, mono, delay=0):
        source = self.SOURCE + mono
        return estimator.correct_observation(source, source + self.OFFSET + delay, mono)

    def test_3_second_load_does_not_raise_30_second_minimum(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        self.observe(est, 0)
        for i in range(1, 631):
            self.observe(est, i * 5_000_000, 30_000_000)
        self.assertEqual(est.snapshot().offset_ns, self.OFFSET)
        self.assertEqual(est.snapshot().samples, 631)

    def test_sixty_second_load_limits_slew_to_three_milliseconds(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        self.observe(est, 0)
        for i in range(1, 12001):
            self.observe(est, i * 5_000_000, 30_000_000)
        self.assertLessEqual(est.snapshot().offset_ns - self.OFFSET, 3_000_000)
        self.assertGreater(est.snapshot().offset_ns, self.OFFSET)
        self.assertTrue(est.snapshot().ready)
        self.assertLessEqual(len(est._candidates), 12000)

    def test_mixed_streams_share_one_offset_and_outputs_increase(self):
        est = NavigationClockOffsetEstimator(warmup_samples=2)
        last = {"imu": 0, "cloud": 0}
        for i in range(1000):
            stream = "cloud" if i % 20 == 0 else "imu"
            stamp = self.observe(est, i * 5_000_000, (i % 3) * 1000)
            if stamp is not None:
                self.assertGreater(stamp, last[stream])
                last[stream] = stamp
        self.assertGreater(last["imu"], 0)
        self.assertGreater(last["cloud"], 0)

    def test_rising_20us_per_second_window_lags_600us(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        for second in range(61):
            self.observe(est, second * 1_000_000_000, second * 20_000)
        self.assertEqual(est.snapshot().offset_ns, self.OFFSET + 600_000)
        self.assertEqual(60 * 20_000 - (est.snapshot().offset_ns - self.OFFSET), 600_000)

    def test_negative_direction_and_target_no_overshoot(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        self.observe(est, 0)
        self.observe(est, 1_000_000_000, -100_000)
        self.assertEqual(est.snapshot().offset_ns, self.OFFSET - 50_000)
        self.observe(est, 2_000_000_000, -100_000)
        self.assertEqual(est.snapshot().offset_ns, self.OFFSET - 100_000)
        self.observe(est, 3_000_000_000, -100_000)
        self.assertEqual(est.snapshot().offset_ns, self.OFFSET - 100_000)

    def test_status_distinguishes_warmup_filled_and_reset(self):
        est = NavigationClockOffsetEstimator(warmup_samples=2, reset_confirm_samples=2)
        self.observe(est, 0)
        self.observe(est, 1)
        self.assertTrue(est.snapshot().capacity_untruncated)
        self.assertFalse(est.snapshot().window_filled)
        self.assertEqual(est.snapshot().window_quality, "warming")
        self.observe(est, 30_000_000_000)
        self.assertTrue(est.snapshot().window_filled)
        self.assertEqual(est.snapshot().retained_sample_span_seconds, 30.0)
        self.assertEqual(est.snapshot().window_quality, "sample_span_full")
        self.observe(est, 30_000_000_001, 2_000_000_000)
        self.observe(est, 30_000_000_002, 2_000_000_000)
        self.assertFalse(est.snapshot().window_filled)
        self.assertEqual(est.snapshot().window_quality, "warming")

    def test_pending_after_long_gap_expires_old_samples_and_marks_status(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        self.observe(est, 0)
        self.observe(est, 30_000_000_000)
        self.assertTrue(est.snapshot().window_filled)
        self.assertIsNone(self.observe(est, 61_000_000_000, 2_000_000_000))
        status = est.snapshot()
        self.assertFalse(status.ready)
        self.assertEqual(status.samples, 0)
        self.assertFalse(status.window_filled)
        self.assertEqual(status.window_quality, "reset_pending")

    def test_long_gap_does_not_claim_full_retained_span(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        for i in range(100):
            self.observe(est, i * 1_000_000)
        self.observe(est, 40_000_000_000)
        status = est.snapshot()
        self.assertEqual(status.samples, 1)
        self.assertEqual(status.epoch_elapsed_seconds, 40.0)
        self.assertEqual(status.retained_sample_span_seconds, 0.0)
        self.assertFalse(status.window_filled)
        self.assertEqual(status.window_quality, "warming")

    def test_window_boundary_is_inclusive_and_bounded(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        self.observe(est, 0)
        self.observe(est, 30_000_000_000, 100_000)
        self.assertEqual(est.snapshot().offset_ns, self.OFFSET)
        self.observe(est, 30_000_000_001, 100_000)
        self.assertEqual(est._minima[0][1], self.OFFSET + 100_000)

    def test_stale_sample_cannot_touch_offset_window_or_reset(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        self.observe(est, 100)
        self.observe(est, 200, 10_000)
        before = est.snapshot()
        self.assertIsNone(self.observe(est, 150, -2_000_000_000))
        self.assertIsNone(self.observe(est, 200, 2_000_000_000))
        after = est.snapshot()
        self.assertEqual((before.offset_ns, before.samples, before.resets),
                         (after.offset_ns, after.samples, after.resets))
        self.assertEqual(after.stale_observations, 2)

    def test_legacy_400_config_and_high_rate_continue_with_bounded_window(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1, window_samples=400)
        for i in range(100000):
            self.assertIsNotNone(self.observe(est, i * 1_000_000))
        self.assertTrue(est.snapshot().ready)
        self.assertEqual(len(est._candidates), 12000)
        self.assertGreater(est.snapshot().window_evicted, 0)
        self.assertFalse(est.snapshot().capacity_untruncated)
        self.assertEqual(est.snapshot().window_quality, "incomplete")
        self.assertIsNotNone(self.observe(est, 130_000_000_001))
        self.assertTrue(est.snapshot().capacity_untruncated)

    def test_threshold_strict_and_confirmed_positive_reset(self):
        est = NavigationClockOffsetEstimator(warmup_samples=2, reset_confirm_samples=3)
        self.observe(est, 0)
        self.observe(est, 1)
        self.assertIsNotNone(self.observe(est, 2, 1_000_000_000))
        self.assertIsNone(self.observe(est, 3, 1_000_000_001))
        self.assertEqual(est.snapshot().pending_reset_samples, 1)
        self.assertIsNotNone(self.observe(est, 4))
        self.assertEqual(est.snapshot().pending_reset_samples, 0)
        for i in range(5, 8):
            self.assertIsNone(self.observe(est, i, 1_000_000_001))
        self.assertEqual(est.snapshot().resets, 1)
        self.assertFalse(est.snapshot().ready)

    def test_negative_jump_needs_confirmation_and_rewarms(self):
        est = NavigationClockOffsetEstimator(warmup_samples=2, reset_confirm_samples=3)
        self.observe(est, 0)
        self.observe(est, 1)
        self.assertIsNotNone(self.observe(est, 2, -1_000_000_000))
        self.assertIsNone(self.observe(est, 3, -1_000_000_001))
        self.assertEqual(est.snapshot().resets, 0)
        self.assertIsNotNone(self.observe(est, 4))
        self.assertEqual(est.snapshot().pending_reset_samples, 0)
        for i in range(5, 8):
            self.assertIsNone(self.observe(est, i, -1_000_000_001))
        self.assertEqual(est.snapshot().resets, 1)
        self.assertFalse(est.snapshot().ready)
        self.assertIsNotNone(self.observe(est, 8, -1_000_000_001))

    def test_randomized_minimum_matches_exact_reference(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        random_source = random.Random(73)
        reference = []
        mono = 0
        for _ in range(30000):
            mono += random_source.randrange(1, 20_000_000)
            delta = random_source.randrange(-100_000, 100_001)
            self.observe(est, mono, delta)
            reference.append((mono, self.OFFSET + delta))
            reference = [item for item in reference if item[0] >= mono - 30_000_000_000]
            self.assertEqual(est._minima[0][1], min(v for _, v in reference))
            self.assertTrue(est.snapshot().capacity_untruncated)

    def test_forced_cross_thread_arrival_order_discards_older_sample(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1)
        sampled = threading.Event()
        release = threading.Event()
        result = []
        def older_callback():
            sampled.set()
            self.assertTrue(release.wait(2))
            result.append(self.observe(est, 100, -2_000_000_000))
        thread = threading.Thread(target=older_callback)
        thread.start()
        self.assertTrue(sampled.wait(2))
        self.assertIsNotNone(self.observe(est, 200))
        release.set()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(result, [None])
        self.assertEqual(est.snapshot().resets, 0)
        self.assertEqual(est.snapshot().stale_observations, 1)

    def test_conflicting_or_dispersed_jumps_restart_confirmation(self):
        est = NavigationClockOffsetEstimator(warmup_samples=1, reset_confirm_samples=3)
        self.observe(est, 0)
        for i, delta in enumerate([2_000_000_000, -2_000_000_000,
                                   2_000_000_000, 4_000_000_000], 1):
            self.assertIsNone(self.observe(est, i, delta))
            self.assertEqual(est.snapshot().pending_reset_samples, 1)
        self.assertEqual(est.snapshot().resets, 0)


if __name__ == "__main__":
    unittest.main()
