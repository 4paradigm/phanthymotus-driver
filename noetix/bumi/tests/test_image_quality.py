"""Image-quality metrics and debounce contracts, using real synthetic images.

These tests exercise the pure module only. They do not load ROS, the Bumi
bundle, vendor SDKs, camera devices, or a network service, and save no images.
Invalid input keeps confirmed reasons visible while current metrics are unknown.

Run from the repository root:
    py -B -m unittest discover -s noetix/bumi/tests -p test_image_quality.py -v
"""

from __future__ import annotations

import ast
from dataclasses import FrozenInstanceError, asdict
import json
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np


BUMI = Path(__file__).resolve().parents[1]
if str(BUMI) not in sys.path:
    sys.path.insert(0, str(BUMI))

import image_quality as quality  # noqa: E402
from image_quality import (  # noqa: E402
    ImageQualityAnalyzer,
    ImageQualityError,
    QualityConfig,
    compute_metrics,
)


METRIC_KEYS = {
    "mean_luminance", "dark_pixel_ratio", "bright_pixel_ratio",
    "laplacian_variance",
}
RESULT_KEYS = {"state", "metrics", "reasons", "candidate_reasons", "error"}


def texture(low=80, high=176, *, height=96, width=128):
    """One-pixel checkerboard: unambiguous contrast without clipping."""
    parity = np.indices((height, width)).sum(axis=0) % 2
    return np.where(parity, high, low).astype(np.uint8)


def jpeg_bytes(image):
    ok, encoded = cv2.imencode(".jpg", image,
                               [cv2.IMWRITE_JPEG_QUALITY, 95])
    if not ok:
        raise AssertionError("synthetic JPEG encoding failed")
    return encoded.tobytes()


def feed(analyzer, image, count):
    result = analyzer.snapshot()
    for _ in range(count):
        result = analyzer.analyze_image(image)
    return result


class MetricsTests(unittest.TestCase):
    def test_constants_and_metric_names_match_public_contract(self):
        for name in ("HEALTHY", "DEGRADED", "UNKNOWN", "LOW_LIGHT",
                     "OVEREXPOSED", "BLURRED"):
            self.assertEqual(getattr(quality, name), name)
        values = compute_metrics(texture())
        self.assertEqual(set(values), METRIC_KEYS)
        for value in values.values():
            self.assertIs(type(value), float)
            self.assertTrue(np.isfinite(value))

    def test_black_and_white_have_exact_brightness_and_pixel_ratios(self):
        for level, dark, bright in ((0, 1.0, 0.0), (255, 0.0, 1.0)):
            with self.subTest(level=level):
                metrics = compute_metrics(np.full((64, 80), level,
                                                 dtype=np.uint8))
                self.assertEqual(metrics["mean_luminance"], float(level))
                self.assertEqual(metrics["dark_pixel_ratio"], dark)
                self.assertEqual(metrics["bright_pixel_ratio"], bright)
                self.assertEqual(metrics["laplacian_variance"], 0.0)

    def test_extreme_pixel_thresholds_are_inclusive(self):
        pixels = np.tile(np.array([39, 40, 41, 239, 240, 241],
                                  dtype=np.uint8), (64, 16))
        metrics = compute_metrics(pixels)
        self.assertAlmostEqual(metrics["dark_pixel_ratio"], 2 / 6)
        self.assertAlmostEqual(metrics["bright_pixel_ratio"], 2 / 6)
        self.assertAlmostEqual(metrics["mean_luminance"], 140.0)

    def test_bgr_is_converted_to_gray_with_opencv(self):
        bgr = np.empty((64, 80, 3), dtype=np.uint8)
        bgr[:, :40] = (20, 80, 180)
        bgr[:, 40:] = (160, 100, 40)
        actual = compute_metrics(bgr)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        expected = compute_metrics(gray)
        self.assertEqual(actual, expected)

    def test_metrics_use_native_dimensions_without_hidden_resize(self):
        image = texture(height=37, width=53)
        actual = compute_metrics(image)
        reference = float(cv2.Laplacian(image, cv2.CV_64F).var())
        self.assertAlmostEqual(actual["laplacian_variance"], reference)

    def test_real_gaussian_blur_reduces_laplacian_variance(self):
        sharp = texture()
        blurred = cv2.GaussianBlur(sharp, (15, 15), 3)
        sharp_metrics = compute_metrics(sharp)
        blurred_metrics = compute_metrics(blurred)
        self.assertGreater(sharp_metrics["laplacian_variance"], 100.0)
        self.assertLess(blurred_metrics["laplacian_variance"], 100.0)
        self.assertLess(blurred_metrics["laplacian_variance"],
                        sharp_metrics["laplacian_variance"] / 100)

    def test_noncontiguous_uint8_views_remain_valid(self):
        image = texture(height=96, width=256)[:, 1::2]
        self.assertFalse(image.flags.c_contiguous)
        self.assertEqual(compute_metrics(image),
                         compute_metrics(np.ascontiguousarray(image)))

    def test_invalid_image_types_and_shapes_raise_typed_error(self):
        invalid_images = [
            None, [[0, 255]], b"not an array",
            np.empty((0, 10), dtype=np.uint8),
            np.empty((10, 0), dtype=np.uint8),
            np.zeros((10,), dtype=np.uint8),
            np.zeros((10, 10, 1), dtype=np.uint8),
            np.zeros((10, 10, 4), dtype=np.uint8),
            np.zeros((2, 10, 10, 3), dtype=np.uint8),
            np.zeros((10, 10), dtype=np.float32),
            np.zeros((10, 10), dtype=np.uint16),
        ]
        for image in invalid_images:
            with self.subTest(shape=getattr(image, "shape", None),
                              kind=type(image).__name__):
                with self.assertRaises(ImageQualityError) as caught:
                    compute_metrics(image)
                self.assertIsInstance(caught.exception, ValueError)
                self.assertIsInstance(caught.exception.code, str)
                self.assertTrue(caught.exception.code)

    def test_input_pixels_are_not_mutated(self):
        image = texture()
        before = image.copy()
        compute_metrics(image)
        np.testing.assert_array_equal(image, before)


class ConfigTests(unittest.TestCase):
    def test_defaults_are_explicit_and_config_is_frozen(self):
        expected = {
            "dark_pixel_threshold": 40,
            "bright_pixel_threshold": 240,
            "low_light_mean_threshold": 60.0,
            "low_light_ratio_threshold": 0.6,
            "overexposed_mean_threshold": 220.0,
            "overexposed_ratio_threshold": 0.3,
            "blur_variance_threshold": 100.0,
            "analysis_hz": 2.0,
            "confirm_samples": 3,
            "recover_samples": 5,
        }
        config = QualityConfig()
        self.assertEqual(asdict(config), expected)
        self.assertEqual(QualityConfig.from_mapping(None), config)
        with self.assertRaises(FrozenInstanceError):
            config.confirm_samples = 4

    def test_partial_mapping_and_analyzer_config_forms(self):
        config = QualityConfig.from_mapping({"confirm_samples": 2})
        self.assertEqual(config.confirm_samples, 2)
        self.assertEqual(config.recover_samples, 5)
        self.assertEqual(ImageQualityAnalyzer(config).config, config)
        self.assertEqual(ImageQualityAnalyzer({"confirm_samples": 2}).config,
                         config)
        self.assertEqual(ImageQualityAnalyzer().config, QualityConfig())

    def test_unknown_config_fields_are_rejected(self):
        with self.assertRaises(ValueError):
            QualityConfig.from_mapping({"blur_threshold_typo": 40})
        analyzer = ImageQualityAnalyzer()
        before = analyzer.snapshot()
        with self.assertRaises(ValueError):
            analyzer.configure({"input_topic": "/should/not/be/here"})
        self.assertEqual(analyzer.snapshot(), before)

    def test_nonfinite_configuration_is_rejected_for_every_numeric_field(self):
        for field in asdict(QualityConfig()):
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ValueError):
                        QualityConfig.from_mapping({field: value})
                    with self.assertRaises(ValueError):
                        QualityConfig(**{field: value})

    def test_booleans_are_not_accepted_as_numeric_configuration(self):
        for field in asdict(QualityConfig()):
            for value in (True, False):
                with self.subTest(field=field, value=value):
                    with self.assertRaises(ValueError):
                        QualityConfig.from_mapping({field: value})

    def test_integer_fields_reject_fractional_values(self):
        for field, value in (("dark_pixel_threshold", 40.5),
                             ("bright_pixel_threshold", 240.5),
                             ("confirm_samples", 3.5),
                             ("recover_samples", 5.5)):
            with self.subTest(field=field):
                with self.assertRaises(ValueError):
                    QualityConfig.from_mapping({field: value})

    def test_out_of_range_and_inconsistent_configurations_are_rejected(self):
        invalid = [
            {"dark_pixel_threshold": -1}, {"dark_pixel_threshold": 255},
            {"bright_pixel_threshold": 0}, {"bright_pixel_threshold": 256},
            {"dark_pixel_threshold": 240, "bright_pixel_threshold": 240},
            {"dark_pixel_threshold": 241, "bright_pixel_threshold": 240},
            {"low_light_mean_threshold": -0.01},
            {"overexposed_mean_threshold": 255.01},
            {"low_light_mean_threshold": 220.0},
            {"low_light_mean_threshold": 221.0},
            {"low_light_ratio_threshold": 0.0},
            {"overexposed_ratio_threshold": 1.01},
            {"blur_variance_threshold": -0.01},
            {"analysis_hz": 0.0}, {"analysis_hz": 30.01},
            {"confirm_samples": 0}, {"confirm_samples": 1001},
            {"recover_samples": 0}, {"recover_samples": 1001},
        ]
        for values in invalid:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    QualityConfig.from_mapping(values)
                with self.assertRaises(ValueError):
                    QualityConfig(**values)

    def test_valid_boundary_configurations_are_accepted(self):
        config = QualityConfig.from_mapping({
            "dark_pixel_threshold": 0, "bright_pixel_threshold": 255,
            "low_light_mean_threshold": 0.0,
            "overexposed_mean_threshold": 255.0,
            "low_light_ratio_threshold": 1.0,
            "overexposed_ratio_threshold": 1.0,
            "blur_variance_threshold": 0.0,
            "analysis_hz": 30.0, "confirm_samples": 1,
            "recover_samples": 1000,
        })
        self.assertEqual(config.confirm_samples, 1)
        self.assertEqual(config.recover_samples, 1000)

    def test_failed_configure_is_atomic_and_keeps_debounced_state(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, texture(5, 35), 3)
        before_config = analyzer.config
        before_snapshot = analyzer.snapshot()
        with self.assertRaises(ValueError):
            analyzer.configure({"confirm_samples": 7,
                                "analysis_hz": float("nan")})
        self.assertEqual(analyzer.config, before_config)
        self.assertEqual(analyzer.snapshot(), before_snapshot)

    def test_successful_changed_configure_resets_all_measurements_and_latches(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, texture(5, 35), 3)
        analyzer.configure({"confirm_samples": 2})
        self.assertEqual(analyzer.config.confirm_samples, 2)
        result = analyzer.snapshot()
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertIsNone(result["metrics"])
        self.assertEqual(result["reasons"], [])
        self.assertEqual(result["candidate_reasons"], [])
        self.assertEqual(analyzer.analyze_image(texture())["state"], "UNKNOWN")

    def test_unchanged_configure_is_a_noop(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, texture(5, 35), 3)
        before = analyzer.snapshot()
        analyzer.configure({})
        self.assertEqual(analyzer.snapshot(), before)
        analyzer.configure({"confirm_samples": 3})
        self.assertEqual(analyzer.snapshot(), before)


class StateMachineTests(unittest.TestCase):
    def setUp(self):
        self.normal = texture()
        self.dark = texture(5, 35)
        self.bright = texture(241, 255)
        self.blurred = cv2.GaussianBlur(self.normal, (15, 15), 3)

    def test_initial_snapshot_is_unknown_without_metrics_or_confirmed_reasons(self):
        result = ImageQualityAnalyzer().snapshot()
        self.assertEqual(set(result), RESULT_KEYS)
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertIsNone(result["metrics"])
        self.assertEqual(result["reasons"], [])
        self.assertEqual(result["candidate_reasons"], [])
        self.assertIsNone(result["error"])

    def test_initial_health_requires_five_consecutive_clean_samples(self):
        analyzer = ImageQualityAnalyzer()
        for _ in range(4):
            result = analyzer.analyze_image(self.normal)
            self.assertEqual(result["state"], "UNKNOWN")
            self.assertEqual(result["reasons"], [])
        result = analyzer.analyze_image(self.normal)
        self.assertEqual(result["state"], "HEALTHY")
        self.assertEqual(result["candidate_reasons"], [])
        self.assertIsNone(result["error"])

    def test_single_bad_sample_does_not_degrade_confirmed_health(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.normal, 5)
        result = analyzer.analyze_image(self.dark)
        self.assertEqual(result["state"], "HEALTHY")
        self.assertEqual(result["reasons"], [])
        self.assertEqual(result["candidate_reasons"], ["LOW_LIGHT"])
        result = analyzer.analyze_image(self.normal)
        self.assertEqual(result["state"], "HEALTHY")
        self.assertEqual(result["candidate_reasons"], [])

    def test_individual_real_image_faults_confirm_on_third_sample(self):
        for image, reason in ((self.dark, "LOW_LIGHT"),
                              (self.bright, "OVEREXPOSED"),
                              (self.blurred, "BLURRED")):
            with self.subTest(reason=reason):
                analyzer = ImageQualityAnalyzer()
                for _ in range(2):
                    result = analyzer.analyze_image(image)
                    self.assertEqual(result["state"], "UNKNOWN")
                    self.assertEqual(result["reasons"], [])
                    self.assertEqual(result["candidate_reasons"], [reason])
                result = analyzer.analyze_image(image)
                self.assertEqual(result["state"], "DEGRADED")
                self.assertEqual(result["reasons"], [reason])

    def test_constant_black_and_white_can_confirm_multiple_reasons(self):
        for level, expected in ((0, {"LOW_LIGHT", "BLURRED"}),
                                (255, {"OVEREXPOSED", "BLURRED"})):
            with self.subTest(level=level):
                analyzer = ImageQualityAnalyzer()
                result = feed(analyzer, np.full((64, 80), level,
                                                dtype=np.uint8), 3)
                self.assertEqual(result["state"], "DEGRADED")
                self.assertEqual(set(result["reasons"]), expected)
                self.assertEqual(set(result["candidate_reasons"]), expected)

    def test_uniform_midgray_is_low_detail_not_proof_of_camera_fault(self):
        analyzer = ImageQualityAnalyzer()
        result = feed(analyzer, np.full((64, 80), 128, dtype=np.uint8), 3)
        self.assertEqual(result["state"], "DEGRADED")
        self.assertEqual(result["reasons"], ["BLURRED"])
        self.assertEqual(result["metrics"]["laplacian_variance"], 0.0)

    def test_recovery_requires_five_clean_samples_and_keeps_confirmed_reasons(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.dark, 3)
        for _ in range(4):
            result = analyzer.analyze_image(self.normal)
            self.assertEqual(result["state"], "DEGRADED")
            self.assertEqual(result["reasons"], ["LOW_LIGHT"])
            self.assertEqual(result["candidate_reasons"], [])
        result = analyzer.analyze_image(self.normal)
        self.assertEqual(result["state"], "HEALTHY")
        self.assertEqual(result["reasons"], [])

    def test_interleaved_different_faults_do_not_share_a_confirmation_counter(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.normal, 5)
        for image in [self.dark, self.bright] * 6:
            result = analyzer.analyze_image(image)
            self.assertEqual(result["state"], "HEALTHY")
            self.assertEqual(result["reasons"], [])

    def test_each_confirmed_reason_recovers_independently(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.dark, 3)
        result = feed(analyzer, self.bright, 3)
        self.assertEqual(result["state"], "DEGRADED")
        self.assertEqual(set(result["reasons"]), {"LOW_LIGHT", "OVEREXPOSED"})
        result = feed(analyzer, self.bright, 2)
        self.assertEqual(result["reasons"], ["OVEREXPOSED"])
        self.assertEqual(result["state"], "DEGRADED")

    def test_new_pending_reason_blocks_health_when_last_old_reason_clears(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.dark, 3)
        feed(analyzer, self.normal, 4)
        # LOW_LIGHT earns its fifth recovery while OVEREXPOSED has only one
        # confirmation sample. Neither a false HEALTHY nor an early alarm is due.
        result = analyzer.analyze_image(self.bright)
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertEqual(result["reasons"], [])
        self.assertEqual(result["candidate_reasons"], ["OVEREXPOSED"])
        self.assertEqual(analyzer.analyze_image(self.bright)["state"], "UNKNOWN")
        result = analyzer.analyze_image(self.bright)
        self.assertEqual(result["state"], "DEGRADED")
        self.assertEqual(result["reasons"], ["OVEREXPOSED"])

    def test_clean_sample_breaks_an_unconfirmed_fault_streak(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.normal, 5)
        feed(analyzer, self.dark, 2)
        analyzer.analyze_image(self.normal)
        result = feed(analyzer, self.dark, 2)
        self.assertEqual(result["state"], "HEALTHY")
        self.assertEqual(result["reasons"], [])
        self.assertEqual(analyzer.analyze_image(self.dark)["state"], "DEGRADED")

    def test_invalid_input_reports_unknown_and_clears_visible_current_measurements(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.normal, 5)
        result = analyzer.analyze_image(None)
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertIsNone(result["metrics"])
        self.assertEqual(result["reasons"], [])
        self.assertEqual(result["candidate_reasons"], [])
        self.assertIsInstance(result["error"]["code"], str)
        self.assertIsInstance(result["error"]["message"], str)

    def test_invalid_input_resets_initial_health_and_unconfirmed_streaks(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.normal, 4)
        analyzer.analyze_image(None)
        self.assertEqual(feed(analyzer, self.normal, 4)["state"], "UNKNOWN")
        self.assertEqual(analyzer.analyze_image(self.normal)["state"], "HEALTHY")
        feed(analyzer, self.dark, 2)
        analyzer.analyze_image(None)
        result = feed(analyzer, self.dark, 2)
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertEqual(result["reasons"], [])

    def test_invalid_input_keeps_confirmed_fault_but_restarts_recovery(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.dark, 3)
        feed(analyzer, self.normal, 4)
        result = analyzer.analyze_image(None)
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertEqual(result["reasons"], ["LOW_LIGHT"])
        for _ in range(4):
            result = analyzer.analyze_image(self.normal)
            self.assertEqual(result["state"], "DEGRADED")
            self.assertEqual(result["reasons"], ["LOW_LIGHT"])
        self.assertEqual(analyzer.analyze_image(self.normal)["state"], "HEALTHY")

    def test_invalid_input_after_health_requires_fresh_health_warmup(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.normal, 5)
        analyzer.analyze_image(None)
        self.assertEqual(feed(analyzer, self.normal, 4)["state"], "UNKNOWN")
        self.assertEqual(analyzer.analyze_image(self.normal)["state"], "HEALTHY")

    def test_reset_clears_confirmed_reasons_and_restarts_warmup(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.dark, 3)
        analyzer.reset()
        self.assertEqual(analyzer.snapshot(), ImageQualityAnalyzer().snapshot())
        self.assertEqual(feed(analyzer, self.normal, 4)["state"], "UNKNOWN")
        self.assertEqual(analyzer.analyze_image(self.normal)["state"], "HEALTHY")

    def test_no_input_does_not_trigger_time_based_stream_or_online_detection(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, self.normal, 5)
        before = analyzer.snapshot()
        for _ in range(10):
            self.assertEqual(analyzer.snapshot(), before)

    def test_mean_comparisons_are_strict_and_pixel_ratio_comparisons_inclusive(self):
        analyzer = ImageQualityAnalyzer({"low_light_mean_threshold": 20.0})
        self.assertEqual(analyzer.analyze_image(self.dark)["candidate_reasons"], [])
        analyzer = ImageQualityAnalyzer({"overexposed_mean_threshold": 248.0})
        self.assertEqual(analyzer.analyze_image(self.bright)["candidate_reasons"], [])
        pixels = np.tile(np.array([0, 10, 20, 30, 40, 100, 100, 100, 100, 100],
                                  dtype=np.uint8), (64, 16))
        analyzer = ImageQualityAnalyzer({"low_light_mean_threshold": 70.0,
                                         "low_light_ratio_threshold": 0.5,
                                         "blur_variance_threshold": 0.0})
        self.assertEqual(analyzer.analyze_image(pixels)["candidate_reasons"],
                         ["LOW_LIGHT"])

    def test_brightness_flags_require_both_mean_and_extreme_pixel_ratio(self):
        samples = [
            np.full((64, 80), 50, dtype=np.uint8),   # low mean, no dark pixels
            np.full((64, 80), 230, dtype=np.uint8),  # high mean, no bright pixels
            np.tile(np.array([39] * 6 + [239] * 4, dtype=np.uint8), (64, 8)),
            np.tile(np.array([250] * 6 + [0] * 4, dtype=np.uint8), (64, 8)),
        ]
        for image in samples:
            with self.subTest(mean=float(image.mean())):
                analyzer = ImageQualityAnalyzer({"blur_variance_threshold": 0.0})
                result = feed(analyzer, image, 5)
                self.assertEqual(result["candidate_reasons"], [])
                self.assertEqual(result["state"], "HEALTHY")


class JpegAndOutputTests(unittest.TestCase):
    def test_invalid_jpeg_preserves_confirmed_low_light_in_unknown(self):
        for invalid in (b"invalid", b"\xff\xd8garbage\xff\xd9"):
            with self.subTest(payload=invalid):
                analyzer = ImageQualityAnalyzer()
                feed(analyzer, texture(5, 35), 3)
                result = analyzer.analyze_jpeg(invalid)
                self.assertEqual(result["state"], "UNKNOWN")
                self.assertIsNone(result["metrics"])
                self.assertEqual(result["reasons"], ["LOW_LIGHT"])
                self.assertEqual(result["candidate_reasons"], [])
                self.assertTrue(result["error"]["code"])
                self.assertTrue(result["error"]["message"])
                result["reasons"].clear()
                self.assertEqual(analyzer.snapshot()["reasons"], ["LOW_LIGHT"])

    def test_invalid_jpeg_preserves_all_confirmed_reasons_in_order(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, texture(5, 35), 3)
        before = feed(analyzer, texture(241, 255), 3)
        self.assertEqual(before["reasons"], ["LOW_LIGHT", "OVEREXPOSED"])
        result = analyzer.analyze_jpeg(b"invalid")
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertIsNone(result["metrics"])
        self.assertEqual(result["reasons"], ["LOW_LIGHT", "OVEREXPOSED"])
        self.assertEqual(result["candidate_reasons"], [])
        self.assertEqual(result["error"]["code"], "INVALID_JPEG")

    def test_invalid_jpeg_without_confirmed_fault_has_empty_reasons(self):
        for pending_samples in (0, 1, 2):
            with self.subTest(pending_samples=pending_samples):
                analyzer = ImageQualityAnalyzer()
                feed(analyzer, texture(5, 35), pending_samples)
                result = analyzer.analyze_jpeg(b"invalid")
                self.assertEqual(result["state"], "UNKNOWN")
                self.assertIsNone(result["metrics"])
                self.assertEqual(result["reasons"], [])
                self.assertEqual(result["candidate_reasons"], [])
                self.assertEqual(result["error"]["code"], "INVALID_JPEG")

    def test_invalid_jpegs_cannot_count_as_recovery_samples(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, texture(5, 35), 3)
        feed(analyzer, texture(), 4)
        for _ in range(10):
            result = analyzer.analyze_jpeg(b"invalid")
            self.assertEqual(result["state"], "UNKNOWN")
            self.assertEqual(result["reasons"], ["LOW_LIGHT"])
            self.assertIsNone(result["metrics"])
        result = analyzer.analyze_image(texture())
        self.assertEqual(result["state"], "DEGRADED")
        self.assertEqual(result["reasons"], ["LOW_LIGHT"])

    def test_invalid_jpeg_requires_a_full_new_recovery_run(self):
        for previous_good_samples in (0, 1, 4):
            with self.subTest(previous_good_samples=previous_good_samples):
                analyzer = ImageQualityAnalyzer()
                feed(analyzer, texture(5, 35), 3)
                feed(analyzer, texture(), previous_good_samples)
                result = analyzer.analyze_jpeg(b"invalid")
                self.assertEqual(result["reasons"], ["LOW_LIGHT"])
                for _ in range(4):
                    result = analyzer.analyze_image(texture())
                    self.assertEqual(result["state"], "DEGRADED")
                    self.assertEqual(result["reasons"], ["LOW_LIGHT"])
                result = analyzer.analyze_image(texture())
                self.assertEqual(result["state"], "HEALTHY")
                self.assertEqual(result["reasons"], [])

    def test_full_recovery_after_invalid_jpeg_clears_every_confirmed_reason(self):
        analyzer = ImageQualityAnalyzer()
        feed(analyzer, np.zeros((64, 80), dtype=np.uint8), 3)
        result = analyzer.analyze_jpeg(b"invalid")
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertEqual(result["reasons"], ["LOW_LIGHT", "BLURRED"])
        for _ in range(4):
            result = analyzer.analyze_image(texture())
            self.assertEqual(result["state"], "DEGRADED")
            self.assertEqual(result["reasons"], ["LOW_LIGHT", "BLURRED"])
        result = analyzer.analyze_image(texture())
        self.assertEqual(result["state"], "HEALTHY")
        self.assertEqual(result["reasons"], [])
        self.assertEqual(result["candidate_reasons"], [])
        self.assertIsNotNone(result["metrics"])
        self.assertIsNone(result["error"])

    def test_supported_jpeg_format_spellings_decode_real_jpeg(self):
        data = jpeg_bytes(texture())
        for image_format in ("jpeg", "jpg", "image/jpeg",
                             "bgr8; jpeg compressed bgr8"):
            with self.subTest(image_format=image_format):
                analyzer = ImageQualityAnalyzer()
                result = analyzer.analyze_jpeg(data, image_format=image_format)
                self.assertIsNone(result["error"])
                self.assertEqual(result["candidate_reasons"], [])
                self.assertEqual(set(result["metrics"]), METRIC_KEYS)

    def test_jpeg_decoding_and_direct_decoded_pixels_have_same_metrics(self):
        data = jpeg_bytes(texture())
        decoded = cv2.imdecode(np.frombuffer(data, dtype=np.uint8),
                               cv2.IMREAD_COLOR)
        self.assertIsNotNone(decoded)
        analyzer = ImageQualityAnalyzer()
        result = analyzer.analyze_jpeg(data)
        self.assertEqual(result["metrics"], compute_metrics(decoded))

    def test_unsupported_format_is_not_silently_decoded(self):
        for image_format in ("png", "image/png", "16UC1; compressedDepth zlib",
                             "rgb8", "unknown"):
            with self.subTest(image_format=image_format):
                result = ImageQualityAnalyzer().analyze_jpeg(
                    jpeg_bytes(texture()), image_format=image_format)
                self.assertEqual(result["state"], "UNKNOWN")
                self.assertEqual(result["error"]["code"], "UNSUPPORTED_FORMAT")
                self.assertIsNone(result["metrics"])

    def test_png_disguised_as_jpeg_is_rejected(self):
        ok, encoded = cv2.imencode(".png", texture())
        self.assertTrue(ok)
        result = ImageQualityAnalyzer().analyze_jpeg(encoded.tobytes())
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertEqual(result["error"]["code"], "INVALID_JPEG")

    def test_missing_jpeg_start_or_end_marker_is_invalid_jpeg(self):
        valid = jpeg_bytes(texture())
        for invalid in (b"", b"not jpeg", valid[2:], valid[:-2],
                        b"\xff\xd8", b"\xff\xd9"):
            with self.subTest(length=len(invalid)):
                result = ImageQualityAnalyzer().analyze_jpeg(invalid)
                self.assertEqual(result["state"], "UNKNOWN")
                self.assertEqual(result["error"]["code"], "INVALID_JPEG")
                self.assertEqual(result["reasons"], [])
                self.assertEqual(result["candidate_reasons"], [])
                self.assertIsNone(result["metrics"])

    def test_marked_but_undecodable_jpeg_reports_decode_failed(self):
        result = ImageQualityAnalyzer().analyze_jpeg(b"\xff\xd8garbage\xff\xd9")
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertEqual(result["error"]["code"], "DECODE_FAILED")

    def test_corrupt_jpeg_does_not_reuse_previous_valid_metrics(self):
        analyzer = ImageQualityAnalyzer()
        analyzer.analyze_jpeg(jpeg_bytes(texture()))
        result = analyzer.analyze_jpeg(b"broken")
        self.assertIsNone(result["metrics"])
        self.assertIsNotNone(result["error"])
        result = analyzer.analyze_jpeg(jpeg_bytes(texture(5, 35)))
        self.assertIsNone(result["error"])
        self.assertIn("LOW_LIGHT", result["candidate_reasons"])

    def test_invalid_jpeg_restarts_recovery_of_confirmed_fault(self):
        analyzer = ImageQualityAnalyzer()
        dark = jpeg_bytes(texture(5, 35))
        normal = jpeg_bytes(texture())
        for _ in range(3):
            result = analyzer.analyze_jpeg(dark)
        self.assertEqual(result["reasons"], ["LOW_LIGHT"])
        for _ in range(4):
            self.assertEqual(analyzer.analyze_jpeg(normal)["state"], "DEGRADED")
        result = analyzer.analyze_jpeg(b"invalid")
        self.assertEqual(result["state"], "UNKNOWN")
        self.assertIsNone(result["metrics"])
        for _ in range(4):
            result = analyzer.analyze_jpeg(normal)
            self.assertEqual(result["state"], "DEGRADED")
            self.assertEqual(result["reasons"], ["LOW_LIGHT"])
        self.assertEqual(analyzer.analyze_jpeg(normal)["state"], "HEALTHY")

    def test_all_result_variants_serialize_as_finite_small_json(self):
        analyzer = ImageQualityAnalyzer()
        snapshots = [analyzer.snapshot(),
                     analyzer.analyze_image(texture()),
                     feed(analyzer, np.zeros((64, 80), dtype=np.uint8), 3),
                     analyzer.analyze_jpeg(b"invalid")]
        for result in snapshots:
            with self.subTest(state=result["state"]):
                self.assertEqual(set(result), RESULT_KEYS)
                encoded = json.dumps(result, allow_nan=False)
                self.assertLess(len(encoded), 4096)
                self.assertEqual(json.loads(encoded), result)
                self.assertNotIn("base64", encoded)

    def test_snapshot_and_returned_results_do_not_share_mutable_state(self):
        analyzer = ImageQualityAnalyzer()
        result = feed(analyzer, texture(5, 35), 3)
        expected = analyzer.snapshot()
        result["metrics"]["mean_luminance"] = -999.0
        result["reasons"].append("injected")
        result["candidate_reasons"].clear()
        snapshot = analyzer.snapshot()
        snapshot["metrics"]["dark_pixel_ratio"] = -1.0
        snapshot["reasons"].clear()
        self.assertEqual(analyzer.snapshot(), expected)
        analyzer.analyze_jpeg(b"broken")
        expected_error = analyzer.snapshot()
        snapshot = analyzer.snapshot()
        snapshot["error"]["message"] = "injected"
        self.assertEqual(analyzer.snapshot(), expected_error)

    def test_pure_module_does_not_import_ros_robot_sdk_or_bundle(self):
        source = Path(quality.__file__).read_text(encoding="utf-8")
        imports = set()
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imports.add(node.module.split(".")[0])
        forbidden = {"rclpy", "sensor_msgs", "std_msgs", "highcontrol_py",
                     "mediacontrol_py", "pyrealsense2", "device", "main"}
        self.assertFalse(imports & forbidden)


if __name__ == "__main__":
    unittest.main()
