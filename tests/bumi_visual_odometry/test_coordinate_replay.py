"""A known optical-to-odom point observed after both translation and rotation."""

from contextlib import closing
import csv

import numpy as np
import pytest

pytest.importorskip("cv2")
from noetix.bumi.tools import capture_coordinates as capture, replay_coordinates as replay


class Recording:
    def __init__(self, db):
        self.recorder = capture.Recorder(db, "test", 1024**2)
        self.base_stamp = 1_000_000_000_000
        self.mounts = set()

    def frame(self, index, *, segment="bumi_visual_first", position=(0., 0., 0.),
              quaternion=(0., 0., 0., 1.), skew_ns=2_000_000, stamp_mismatch=False):
        t = self.base_stamp + index * 100_000_000
        write = self.recorder.write
        optical = {"frame_id": "test_rgb_optical", "width": 5, "height": 5}
        parent, child = segment + "_odom", segment + "_base"
        if child not in self.mounts:
            self.mounts.add(child)
            write("tf_static", t, {
                "parent": child, "child": optical["frame_id"], "translation": [.2, 0., .1],
                "quaternion": [0., 0., 2**-.5, 2**-.5],
            })
        write("rgb", t, {**optical, "encoding": "bgr8", "step": 16},
              bytes([20]) * 80)  # Include a real image-row padding byte.
        write("depth", t + skew_ns, {**optical, "encoding": "32FC1",
                                    "step": 20, "is_bigendian": False},
              np.full((5, 5), 2., dtype="<f4").tobytes())
        write("info", t, {**optical, "distortion_model": "plumb_bob", "d": [0.] * 5,
                          "k": [4., 0., 2., 0., 4., 2., 0., 0., 1.]})
        write("camera_meta", t, {"epoch": "camera-a", "rgb_stamp_ns": t,
                                 "depth_stamp_ns": t + skew_ns})
        write("odom", t, {"schema": "motus.odom/1",
                          "stamp_ms": t // 1_000_000 - int(stamp_mismatch),
                          "pose": {"position": position, "quaternion": quaternion,
                                   "frame_id": parent, "child_frame_id": child},
                          "vendor": {"source": "rtabmap_rgbd_odometry", "status": "tracking",
                                     "visual_stamp_ns": t, "segment_id": segment,
                                     "pose_reference": "body", "calibration_id": "measured",
                                     "camera_time_provenance": {"epoch": "camera-a"}}})


@pytest.fixture
def dataset(tmp_path):
    path = tmp_path / "测试.sqlite3"
    with closing(capture.open_capture(path, "test")) as db:
        rec = Recording(db)
        rec.frame(0)
        rec.frame(1, position=(.4, 0, 0), quaternion=(0., 0., 1., 0.))
        rec.frame(2, stamp_mismatch=True)
        rec.frame(3, skew_ns=21_000_000)
        rec.frame(4, segment="bumi_visual_second")
        db.commit()
    return path


def annotate(path, images):
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    for row in rows:
        if row["image"] in images:
            row["station"], row["u"], row["v"] = images[row["image"]]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=replay.CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def test_capture_replay_preserves_one_world_point_and_rejects_mixed_segments(dataset, tmp_path):
    preview = tmp_path / "预览"
    with closing(replay.connect(dataset)) as db:
        frames, stats = replay.paired_frames(db)
        assert stats["matched_frames"] == 3
        assert stats["skipped_rgb"] == {"no_matching_tracking_pose": 1, "missing_or_unsynced_depth": 1}
        first, second, reset = frames
        replay.prepare(db, frames, stats, preview, every=1)
        assert (preview / first).read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
        csv_path = preview / "observations.csv"
        positions = {first: ("A", 3, 2), second: ("B", 1, 2)}
        annotate(csv_path, positions)
        result = replay.analyze(db, frames, csv_path, radius=1, tolerance=.001)
        assert result["within_tolerance"] is True
        assert result["max_point_separation_m"] < 1e-10
        assert result["stations"]["B"]["median_point_odom_m"] == pytest.approx([.2, .5, 2.1])
        annotate(csv_path, {reset: ("after-reset", 3, 2)})
        with pytest.raises(ValueError, match="must share a segment"):
            replay.analyze(db, frames, csv_path, radius=1, tolerance=None)
