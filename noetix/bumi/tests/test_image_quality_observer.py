"""Metrics-only observer parsing, without ROS or filesystem writes."""
import importlib.util
import json
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("iqg_observer", Path(__file__).resolve().parents[1]
                                            / "tools" / "image_quality_observe.py")
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


class ObserverTests(unittest.TestCase):
    def result(self, **extra):
        return json.dumps({"schema": "motus.image_quality/1", "run_id": "observed-ticket",
                           "state": "UNKNOWN", "metrics": None, "reasons": ["LOW_LIGHT"],
                           "candidate_reasons": [], "error": {"code": "INVALID_JPEG"}, **extra})

    def test_unknown_keeps_confirmed_reason_and_omits_unknown_fields(self):
        value = observer.parse_result(self.result(unexpected_payload="discard me"))
        self.assertEqual(value["reasons"], ["LOW_LIGHT"])
        self.assertIsNone(value["metrics"])
        self.assertNotIn("unexpected_payload", value)

    def test_old_run_is_filtered(self):
        self.assertIsNone(observer.parse_result(self.result(), "new-ticket"))

    def test_invalid_schema_state_and_nonfinite_json_are_rejected(self):
        for data in ("null", "[]", self.result(state="ONLINE"),
                     self.result(schema="unknown"), self.result(error=float("nan"))):
            with self.subTest(data=data), self.assertRaises(ValueError):
                observer.parse_result(data)

    def test_quality_metrics_are_finite_and_complete(self):
        metrics = {"mean_luminance": 128., "dark_pixel_ratio": 0.,
                   "bright_pixel_ratio": 0., "laplacian_variance": 200.}
        self.assertEqual(observer.parse_result(self.result(metrics=metrics))["metrics"], metrics)
        for bad in ({}, {**metrics, "mean_luminance": "nan"}):
            with self.subTest(metrics=bad), self.assertRaises(ValueError):
                observer.parse_result(self.result(metrics=bad))
