"""Hardware-free Q5 ArUco canvas and media-bridge contract tests."""

from pathlib import Path
import sys
import threading
import time
import unittest
from unittest import mock


Q5 = Path(__file__).resolve().parents[1] / "robotera/q5_bundle"
if str(Q5) not in sys.path:
    sys.path.insert(0, str(Q5))
import aruco_zones


class _CameraWorker:
    def __init__(self):
        self.sent = False

    def wait_for_frame(self, kind, after_sequence, timeout_s):
        assert kind == "rgb"
        if not self.sent:
            self.sent = True
            return {"kind": "rgb", "data": b"jpeg"}, 1
        time.sleep(min(timeout_s, 0.01))
        return None, 1


class _Client:
    def __init__(self):
        self.camera_worker = _CameraWorker()
        self.media = []
        self.ready = threading.Event()

    def publish_media(self, item):
        self.media.append(item)
        if item["kind"] == "aruco_regions":
            self.ready.set()


class Q5ArucoZonesTests(unittest.TestCase):
    def test_registration_and_no_motion_contract(self):
        self.assertIn("aruco_zones:\n    enabled: true", (Q5 / "config.yaml").read_text())
        self.assertIn("name: aruco_zones,       type: processor", (Q5 / "driver.yaml").read_text())
        self.assertIn("COPY aruco_zones.py", (Q5 / "Dockerfile").read_text())
        client = _Client()
        card = aruco_zones.make_plugin({}, "q5", None, client)
        tool = card.get_tool()
        self.assertEqual(tool["type"], "processor")
        self.assertEqual(tool["topic_in"][0]["format"], "image/jpeg")
        self.assertEqual([out["format"] for out in tool["topic_out"]],
                         ["image/jpeg", "data/json"])
        with self.assertRaises(ValueError):
            card.dispatch("start", {"input_topic": "/q5/camera/depth_preview"})

    def test_three_regions_bind_unordered_ids(self):
        config, rects = aruco_zones.validate_config({
            "waiting_rect": "0,0,0.3,1", "sorting_1_rect": "0.35,0,0.65,1",
            "sorting_2_rect": "0.7,0,1,1"})
        markers = [{"id": 9, "center_px": [80, 50]},
                   {"id": 2, "center_px": [20, 50]},
                   {"id": 7, "center_px": [50, 50]}]
        result = aruco_zones.assign_zones(markers, rects, 100, 100, config)
        self.assertTrue(result["ready"])
        self.assertEqual([result["zones"][role]["marker_id"] for role in aruco_zones.ROLES],
                         [2, 7, 9])
        self.assertFalse(result["motion_enabled"])
        with self.assertRaises(ValueError):
            aruco_zones.validate_config({"waiting_rect": "0,0,0.5,1",
                                         "sorting_1_rect": "0.4,0,0.8,1"})

    def test_rgb_worker_frame_reaches_both_canvas_outputs(self):
        client = _Client()
        card = aruco_zones.make_plugin({}, "q5", None, client)
        result = {"ready": False, "motion_enabled": False, "zones": {}}
        with mock.patch.object(aruco_zones, "process_jpeg", return_value=(result, b"overlay")):
            started = card.dispatch("start", {"input_topic": "/q5/camera/rgb"})
            self.assertEqual(started["state"], "running")
            self.assertTrue(client.ready.wait(1))
            self.assertEqual(card.dispatch("info", {})["latest"], result)
            self.assertEqual(card.dispatch("stop", {})["state"], "idle")
        self.assertEqual([item["kind"] for item in client.media],
                         ["aruco_overlay", "aruco_regions"])


if __name__ == "__main__":
    unittest.main()
