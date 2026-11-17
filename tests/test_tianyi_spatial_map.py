"""Offline STCM layer and map-frame regressions (no ROS or chassis)."""

import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from test_stcm import SEMANTIC, container, grid_layer


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "tianyi_spatial_map", ROOT / "x-humanoid/tianyi2.0/controlled_spatial_map.py")
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SpatialMapTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.card = MODULE.ControlledSpatialMapPlugin.__new__(MODULE.ControlledSpatialMapPlugin)
        self.card._np = np
        self.card._lock = threading.RLock()
        self.card._grid_flip_x = self.card._grid_flip_y = False
        self.card._grid_x_offset = self.card._grid_y_offset = 0.0
        self.card._grid_roll_x_cells = 0

    def load_grid(self, cells, origin=(-3.0, 2.0), resolution=(0.05, 0.08)):
        raw = container(grid_layer(cells.astype(np.uint8).tobytes(),
                                   dimension_width=cells.shape[1], dimension_height=cells.shape[0],
                                   origin_x=origin[0], origin_y=origin[1],
                                   resolution_x=resolution[0], resolution_y=resolution[1]), SEMANTIC)
        path = Path(self.directory.name) / "map.stcm"
        path.write_bytes(raw)
        self.assertTrue(self.card._load_stcm_grid(str(path)))
        return self.card._grid

    def test_small_asymmetric_grid_preserves_map_frame_and_cell_values(self):
        cells = np.array([[127, 0, 255], [0, 50, 127]], dtype=np.uint8)
        points = self.load_grid(cells, origin=(-2.0, 3.0), resolution=(0.2, 0.4))
        np.testing.assert_allclose(points, [
            [-1.9, 3.2, -0.03], [-1.5, 3.6, -0.03],
            [-1.5, 3.2, 0.04], [-1.7, 3.6, 0.04],
        ], atol=1e-6)
        np.testing.assert_allclose(self.card._grid_bounds, [-2.0, 3.0, 0.6, 0.8])
        # A POI pillar is centered on the same world position as its floor cell.
        tag = {"x": -1.9, "y": 3.2, "yaw": 0.0}
        marker = np.asarray(self.card._tag_points([tag]))
        np.testing.assert_allclose(marker[:480, :2].mean(axis=0), points[0, :2], atol=1e-6)

    def test_malformed_layer_is_reported_without_publishing_partial_grid(self):
        path = Path(self.directory.name) / "truncated.stcm"
        path.write_bytes(container(grid_layer(), SEMANTIC)[:-1])
        self.card._grid = np.zeros((0, 3), dtype=np.float32)
        self.assertFalse(self.card._load_stcm_grid(str(path)))
        self.assertIn("saved map load failed", self.card._startup_error)
        self.assertEqual(len(self.card._grid), 0)

    def test_published_map_with_trailing_layer_keeps_pois_aligned(self):
        cells = np.zeros((20, 30), dtype=np.uint8)
        cells[:, 8:13] = 127
        self.load_grid(cells)
        tags = [{"name": str(row), "x": -2.475, "y": 2.0 + (row + 0.5) * 0.08,
                 "yaw": 0.7} for row in (2, 10, 18)]
        self.card._db = SimpleNamespace(maps=lambda: [{"name": "test", "tags": tags}])
        self.card._current_map = "test"
        self.card._pose = {"x": tags[-1]["x"], "y": tags[-1]["y"], "yaw": 0.7}
        self.card._head_cloud = {}
        self.card._head_recording_enabled = False
        self.card._trajectory = []
        self.card._artifacts = {"walls": [], "tracks": [], "areas": {}}
        self.card._pub = Mock()
        with patch.dict(sys.modules, {"std_msgs.msg": SimpleNamespace(UInt8MultiArray=SimpleNamespace)}):
            self.card._publish(force=True)
        payload = bytes(self.card._pub.publish.call_args.args[0].data)
        x, y, yaw, flags, count = struct.unpack_from("<fffBI", payload)
        np.testing.assert_allclose([x, y, yaw], [tags[-1]["x"], tags[-1]["y"], -0.7])
        self.assertEqual(flags, 7)
        self.assertLessEqual(count, self.card.MAX_POINTS)
        points = np.frombuffer(payload, dtype="<f4", count=count * 3, offset=17).reshape(-1, 3)
        meta_at = 17 + count * 12
        meta_size, = struct.unpack_from("<I", payload, meta_at)
        meta = json.loads(payload[meta_at + 4:meta_at + 4 + meta_size])
        self.assertEqual(meta["tags"], tags)
        self.assertEqual(meta["robot"], {**self.card._pose, "pose_available": True})
        floor = points[points[:, 2] < 0]
        for tag in tags:
            distance = np.linalg.norm(floor[:, :2] - [tag["x"], tag["y"]], axis=1)
            self.assertLess(distance.min(), 1e-5, "POI must stay on its original floor cell")


if __name__ == "__main__":
    unittest.main()
