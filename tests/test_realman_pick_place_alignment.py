"""SDK projection regression using software frames, never a physical camera."""
import importlib.util
from pathlib import Path
import unittest
import zlib

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / "realman/rm75_6f_v/pick_place/alignment.py"
spec = importlib.util.spec_from_file_location("pick_place_alignment", SOURCE)
alignment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(alignment)


class AlignmentSDKTests(unittest.TestCase):
    def test_native_resolution_alignment_preserves_holes_and_xy_translation(self):
        try:
            import pyrealsense2
        except ImportError:
            self.skipTest("RealSense SDK unavailable; run this test in the Driver image")

        def intr(width, height):
            return dict(width=width, height=height, fx=500., fy=500.,
                        ppx=width/2, ppy=height/2, model="distortion.none", coeffs=[0]*5)
        metadata = dict(version=2, depth_aligned_to="depth", depth_scale_m=.001,
                        serial_number="synthetic", session_id="offline",
                        rgb_intrinsics=intr(1280, 720), depth_intrinsics=intr(640, 480),
                        depth_to_color=dict(rotation=[1., 0, 0, 0, 1., 0, 0, 0, 1.],
                                            translation=[0., 0, 0]))
        raw = np.full((480, 640), 500, dtype="<u2")
        raw[200:240, 300:340] = 0
        encoded = zlib.compress(raw.tobytes())
        baseline = alignment.align_depth(encoded, metadata)
        self.assertEqual(baseline.shape, (720, 1280))
        self.assertEqual(set(np.unique(baseline)), {0, 500})
        self.assertEqual(baseline[340, 640], 0)
        self.assertEqual(baseline[200, 640], 500)
        # SDK projects pixel footprints, including the boundary pixels. Test
        # translation against its baseline instead of treating it as resize.
        metadata["depth_to_color"]["translation"] = [.02, 0, 0]
        right = alignment.align_depth(encoded, metadata)
        np.testing.assert_array_equal(right[:, 20:], baseline[:, :-20])
        self.assertFalse(right[:, :20].any())
        metadata["depth_to_color"]["translation"] = [0, -.01, 0]
        up = alignment.align_depth(encoded, metadata)
        np.testing.assert_array_equal(up[:-10, :], baseline[10:, :])
        self.assertFalse(up[-10:, :].any())


if __name__ == "__main__":
    unittest.main()
