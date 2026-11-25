"""Contract tests for the G1 image_diff card; synthetic images only."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import image_diff as image_diff_module


def make_scene(width=640, height=480, brightness=0, seed=7):
    rng = np.random.default_rng(seed)
    texture = (rng.random((height, width)) * 60 + 100).astype(np.uint8)
    image = cv2.cvtColor(cv2.GaussianBlur(texture, (5, 5), 0), cv2.COLOR_GRAY2BGR)
    cv2.rectangle(image, (200, 180), (300, 280), (255, 255, 255), -1)
    cv2.rectangle(image, (400, 100), (430, 130), (0, 0, 0), -1)
    if brightness:
        image = np.clip(image.astype(np.int16) + brightness, 0, 255).astype(np.uint8)
    return image


def encode(image, extension=".jpg"):
    ok, buffer = cv2.imencode(extension, image)
    if not ok:
        raise AssertionError("cannot encode synthetic image")
    return buffer.tobytes()


class FakeCamera:
    def __init__(self, frames, running=True):
        self.frames = list(frames)
        self.running = running
        self.index = 0

    def is_running(self):
        return self.running

    def wait_for_color_frame(self, after_sequence=None, timeout_s=5.0):
        frame = self.frames[min(self.index, len(self.frames) - 1)]
        self.index += 1
        sequence = self.index
        return {
            "data": frame,
            "received_monotonic": time.monotonic(),
            "frame_sequence": sequence,
        }, sequence


class ImageDiffTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.source = self.root / "data"
        self.output = self.root / "out"
        self.source.mkdir()
        self.plugin = image_diff_module.ImageDiffPlugin({
            "output_dir": str(self.output),
            "source_root": str(self.source),
            "retention_days": 7,
        }, "test", None)

    def tearDown(self):
        self.temporary.cleanup()

    def write(self, name, image, extension=".jpg"):
        path = self.source / name
        path.write_bytes(encode(image, extension))
        return path

    def compare(self, before, after, **kwargs):
        return self.plugin.dispatch("compare_files", {
            "before_path": str(before),
            "after_path": str(after),
            **kwargs,
        })

    def test_tool_contract(self):
        tool = self.plugin.get_tool()
        self.assertEqual(tool["name"], "image_diff")
        self.assertEqual(tool["type"], "actuator")
        self.assertIs(tool["multiInstance"], False)
        schema = tool["inputSchema"]
        self.assertEqual(schema["required"], ["action"])
        self.assertIs(schema["additionalProperties"], False)
        self.assertIn("compare_files", schema["properties"]["action"]["enum"])
        self.assertEqual(
            set(schema["x-action-params"]),
            {"start", "stop", "info", "capture_baseline", "compare",
             "compare_files", "list", "delete"},
        )
        self.assertNotIn("x-completion", schema)

    def test_card_is_wired_into_config_marketplace_and_bundle(self):
        config = yaml.safe_load((ROOT / "config.yaml").read_text(encoding="utf-8"))
        self.assertIs(config["plugins"]["image_diff"]["enabled"], True)
        driver = yaml.safe_load((ROOT / "driver.yaml").read_text(encoding="utf-8"))
        self.assertIn({"name": "image_diff", "type": "actuator"}, driver["cards"])
        main = (ROOT / "main.py").read_text(encoding="utf-8")
        self.assertIn("from image_diff import ImageDiffPlugin", main)
        self.assertIn('plugins_cfg.get("image_diff"', main)

    def test_info_reports_storage_and_camera_state(self):
        result = self.plugin.dispatch("info", {})
        self.assertTrue(result["ok"])
        self.assertFalse(result["camera_ready"])
        self.assertEqual(result["output_dir"], str(self.output))
        self.assertEqual(result["baseline_count"], 0)
        self.assertEqual(result["run_count"], 0)

    def test_unknown_action_is_declined(self):
        self.assertIsNone(self.plugin.dispatch("nope", {}))

    def test_identical_images_are_unchanged(self):
        before = self.write("before.jpg", make_scene())
        after = self.write("after.jpg", make_scene())
        result = self.compare(before, after)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["verdict"], "no_change")
        self.assertEqual(result["metrics"]["changed_ratio"], 0.0)
        self.assertEqual(result["metrics"]["regions"], [])
        self.assertTrue(Path(result["diff_path"]).is_file())
        self.assertTrue(Path(result["mask_path"]).is_file())
        self.assertTrue(Path(result["result_path"]).is_file())

    def test_moved_object_is_changed(self):
        before = self.write("before.jpg", make_scene())
        after_image = make_scene()
        cv2.rectangle(after_image, (200, 180), (300, 280), (60, 60, 60), -1)
        after = self.write("after.jpg", after_image)
        result = self.compare(before, after, align="none")
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["verdict"], "changed")
        self.assertGreater(result["metrics"]["changed_ratio"], 0.005)
        regions = result["metrics"]["regions"]
        self.assertTrue(regions)
        x, y, w, h = regions[0]["bbox"]
        self.assertLessEqual(x, 205)
        self.assertLessEqual(y, 185)
        self.assertGreaterEqual(x + w, 295)
        self.assertGreaterEqual(y + h, 275)

    def test_brightness_change_stays_unchanged(self):
        before = self.write("before.jpg", make_scene())
        after = self.write("after.jpg", make_scene(brightness=-30))
        result = self.compare(before, after)
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["verdict"], "no_change", result["metrics"])

    def test_translation_is_aligned_before_comparison(self):
        before_image = make_scene()
        after_image = cv2.warpAffine(
            before_image, np.float32([[1, 0, 8], [0, 1, 0]]),
            (before_image.shape[1], before_image.shape[0]),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)
        before = self.write("before.jpg", before_image)
        after = self.write("after.jpg", after_image)
        result = self.compare(before, after, align="translation")
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["alignment"]["mode"], "translation")
        self.assertAlmostEqual(result["alignment"]["shift_px"][0], 8.0, delta=1.0)
        self.assertNotEqual(result["verdict"], "changed")

    def test_high_resolution_is_resized_and_rendered(self):
        before_image = make_scene(width=1280, height=720)
        after_image = before_image.copy()
        cv2.rectangle(after_image, (500, 300), (700, 500), (30, 30, 30), -1)
        before = self.write("before.jpg", before_image)
        after = self.write("after.jpg", after_image)
        result = self.compare(before, after, align="none")
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["image_size"], [960, 540])
        triptych = cv2.imread(result["diff_path"])
        self.assertEqual(triptych.shape[1], 640 * 3 + 8 * 2)
        self.assertEqual(triptych.shape[0], 540 * 640 // 960 + 30)

    def test_same_aspect_different_resolution_is_fitted(self):
        before_image = make_scene()
        after_image = cv2.resize(before_image, (1280, 960))
        before = self.write("before.jpg", before_image)
        after = self.write("after.jpg", after_image)
        result = self.compare(before, after, align="none")
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["image_size"], [640, 480])
        self.assertEqual(result["verdict"], "no_change")

    def test_low_texture_is_uncertain_not_silent(self):
        blank = np.full((480, 640, 3), 128, np.uint8)
        before = self.write("before.jpg", blank)
        after = self.write("after.jpg", blank)
        result = self.compare(before, after, align="none")
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["verdict"], "uncertain")
        self.assertTrue(result["metrics"]["low_texture"])

    def test_aspect_mismatch_is_rejected(self):
        before = self.write("before.jpg", make_scene())
        after = self.write("after.jpg", make_scene(width=800, height=480))
        result = self.compare(before, after)
        self.assertFalse(result["ok"])
        self.assertEqual(result["code"], "SIZE_MISMATCH")

    def test_corrupt_image_is_rejected(self):
        before = self.write("before.jpg", make_scene())
        after = self.source / "after.jpg"
        after.write_bytes(b"not an image")
        result = self.compare(before, after)
        self.assertFalse(result["ok"])
        self.assertEqual(result["code"], "IMAGE_UNREADABLE")

    def test_path_outside_source_root_is_rejected(self):
        outside = self.root / "outside.jpg"
        outside.write_bytes(encode(make_scene()))
        inside = self.write("inside.jpg", make_scene())
        result = self.compare(outside, inside)
        self.assertFalse(result["ok"])
        self.assertEqual(result["code"], "PATH_NOT_ALLOWED")

    def test_capture_baseline_and_compare_persist_across_instances(self):
        camera = FakeCamera([encode(make_scene())])
        plugin = image_diff_module.ImageDiffPlugin({
            "output_dir": str(self.output),
            "source_root": str(self.source),
        }, "test", None, camera)
        captured = plugin.dispatch("capture_baseline", {"label": "desk"})
        self.assertTrue(captured["ok"], captured)
        baseline_id = captured["baseline_id"]
        self.assertTrue(Path(captured["file_path"]).is_file())

        changed = make_scene()
        cv2.rectangle(changed, (200, 180), (300, 280), (60, 60, 60), -1)
        second = image_diff_module.ImageDiffPlugin({
            "output_dir": str(self.output),
            "source_root": str(self.source),
        }, "test", None, FakeCamera([encode(changed)]))
        result = second.dispatch("compare", {
            "baseline_id": baseline_id, "align": "none"})
        self.assertTrue(result["ok"], result)
        self.assertEqual(result["baseline_id"], baseline_id)
        self.assertEqual(result["verdict"], "changed")

        listed = second.dispatch("list", {"kind": "baselines"})
        self.assertEqual(listed["items"][0]["baseline_id"], baseline_id)
        deleted = second.dispatch("delete", {"kind": "baselines", "id": baseline_id})
        self.assertTrue(deleted["ok"], deleted)
        self.assertEqual(second.dispatch("info", {})["baseline_count"], 0)

    def test_live_capture_requires_camera(self):
        result = self.plugin.dispatch("capture_baseline", {})
        self.assertFalse(result["ok"])
        self.assertEqual(result["code"], "CAMERA_UNAVAILABLE")

    def test_retention_removes_old_entries(self):
        baseline_id = "b_20200101_000000_abcd"
        image = self.write("old.jpg", make_scene())
        self.output.mkdir(parents=True, exist_ok=True)
        (self.output / "baselines").mkdir(parents=True, exist_ok=True)
        path = self.output / "baselines" / f"{baseline_id}.jpg"
        path.write_bytes(image.read_bytes())
        metadata = self.output / "baselines" / f"{baseline_id}.json"
        metadata.write_text(json.dumps({"baseline_id": baseline_id}))
        old = time.time() - 30 * 86400
        os.utime(path, (old, old))
        os.utime(metadata, (old, old))
        self.plugin.dispatch("start", {})
        self.assertFalse(path.exists())
        self.assertFalse(metadata.exists())

    def test_delete_rejects_invalid_id(self):
        result = self.plugin.dispatch("delete", {"kind": "baselines", "id": "../../etc"})
        self.assertFalse(result["ok"])
        self.assertEqual(result["code"], "INVALID_ARGUMENT")


if __name__ == "__main__":
    unittest.main()
