"""Pick-place-owned camera tests; other cards are not imported."""
import queue
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock
import zlib

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "realman/rm75_6f_v"))
from pick_place import camera as rs


class SnapshotTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch("common.logsafe.install")
        self.install_logsafe = patch.start()
        self.addCleanup(patch.stop)
        self.raw = np.array([[0, 1000], [2000, 3000]], dtype=np.uint16)
        self.rgb = np.zeros((2, 2, 3), dtype=np.uint8)
        intrinsics = types.SimpleNamespace(width=2, height=2, fx=100, fy=101,
                                          ppx=1, ppy=1, model="brown", coeffs=[0]*5)
        profile = types.SimpleNamespace(as_video_stream_profile=lambda: types.SimpleNamespace(
            get_intrinsics=lambda: intrinsics))
        self.color = types.SimpleNamespace(get_frame_timestamp_domain=lambda: "global", get_timestamp=lambda: 100100,
                                           get_data=lambda: self.rgb, profile=profile)
        self.depth = types.SimpleNamespace(get_frame_timestamp_domain=lambda: "global", get_timestamp=lambda: 100100,
                                           get_data=lambda: self.raw)
        self.frames = types.SimpleNamespace(get_color_frame=lambda: self.color, get_depth_frame=lambda: self.depth)
        self.align = mock.Mock(return_value=types.SimpleNamespace(process=mock.Mock(return_value=self.frames)))
        self.sdk = types.SimpleNamespace(timestamp_domain=types.SimpleNamespace(system_time="system", global_time="global"),
                                         stream=types.SimpleNamespace(color="color"), align=self.align)
        self.cv = types.SimpleNamespace(IMWRITE_JPEG_QUALITY=1, imencode=mock.Mock(
            return_value=(True, np.array([255,216,255,217], dtype=np.uint8))))
        patch = mock.patch.object(rs.time, "time", return_value=100.2)
        patch.start()
        self.addCleanup(patch.stop)

    def test_one_aligned_pair_preserves_invalid_depth_and_intrinsics(self):
        photo = rs.snapshot_rgbd(self.sdk, self.cv, self.frames, 0.001, "D435", after=100)
        np.testing.assert_array_equal(np.frombuffer(zlib.decompress(photo["depth_zlib"]), dtype="<u2").reshape(2,2), self.raw)
        self.assertEqual(photo["intrinsics"]["fx"], 100)
        self.assertEqual(photo["serial_number"], "D435")
        self.assertEqual(photo["depth_scale_m"], 0.001)
        self.align.assert_called_once_with("color")
        self.cv.imencode.assert_called_once()

    def test_old_pair_waits_without_encoding(self):
        self.assertIsNone(rs.snapshot_rgbd(self.sdk, self.cv, self.frames, 0.001, "D435", after=100.15))
        self.cv.imencode.assert_not_called()

    def test_unmapped_hardware_clock_is_rejected(self):
        self.depth.get_frame_timestamp_domain = lambda: "hardware"
        with self.assertRaisesRegex(RuntimeError, "timestamps"):
            rs.snapshot_rgbd(self.sdk, self.cv, self.frames, 0.001, "D435", after=100)
        self.cv.imencode.assert_not_called()

    def test_misaligned_depth_and_encoding_failure_are_rejected(self):
        self.depth.get_data = lambda: self.raw[:1]
        with self.assertRaisesRegex(RuntimeError, "aligned"):
            rs.snapshot_rgbd(self.sdk, self.cv, self.frames, 0.001, "D435", after=100)
        self.depth.get_data = lambda: self.raw
        self.cv.imencode.return_value = False, None
        with self.assertRaisesRegex(RuntimeError, "encoding"):
            rs.snapshot_rgbd(self.sdk, self.cv, self.frames, 0.001, "D435", after=100)

    def test_capture_worker_emits_one_snapshot_and_has_no_stream_publisher(self):
        requests, results, statuses = queue.Queue(), queue.Queue(), queue.Queue()
        requests.put(("one-request", 100))
        stopped = threading.Event()
        device = mock.Mock()
        device.get_info.side_effect = lambda key: "D435" if key == "serial" else "3.2"
        device.first_depth_sensor.return_value.get_depth_scale.return_value = 0.001
        device.query_sensors.return_value = []
        pipeline = mock.Mock()
        frame_count = 0

        def next_frame(timeout):
            nonlocal frame_count
            frame_count += 1
            if frame_count == 4:
                stopped.set()
            return self.frames

        pipeline.wait_for_frames.side_effect = next_frame
        self.sdk.camera_info = types.SimpleNamespace(serial_number="serial", usb_type_descriptor="usb")
        self.sdk.stream.depth = "depth"
        self.sdk.format = types.SimpleNamespace(bgr8="bgr8", z16="z16")
        self.sdk.context = mock.Mock(return_value=types.SimpleNamespace(query_devices=lambda: [device]))
        self.sdk.config = mock.Mock()
        self.sdk.pipeline = mock.Mock(return_value=pipeline)
        with mock.patch.dict(sys.modules, {"pyrealsense2": self.sdk, "cv2": self.cv,
                                           "rclpy": None, "camera": None, "realsense": None}), \
             mock.patch.object(rs, "CameraLease") as lease:
            rs._capture("D435", requests, results, statuses, stopped)
        self.assertEqual(frame_count, 4)
        self.install_logsafe.assert_called_once_with(check_fd=False)
        self.assertEqual(results.qsize(), 1)
        self.assertEqual(results.get_nowait()["request_id"], "one-request")
        self.cv.imencode.assert_called_once()
        self.assertTrue(all("error" not in item for item in list(statuses.queue)))
        pipeline.start.assert_called_once()
        pipeline.stop.assert_called_once()
        lease.assert_called_once_with("D435")
        lease.return_value.__exit__.assert_called_once()

    def test_capture_worker_does_not_open_an_occupied_camera(self):
        statuses = queue.Queue()
        self.sdk.context = mock.Mock()
        with mock.patch.dict(sys.modules, {"pyrealsense2": self.sdk, "cv2": self.cv}), \
             mock.patch.object(rs, "CameraLease") as lease:
            lease.return_value.__enter__.side_effect = RuntimeError("camera in use")
            rs._capture("D435", queue.Queue(), queue.Queue(), statuses, threading.Event())
        self.assertEqual(statuses.get_nowait()["error"], "camera in use")
        self.sdk.context.assert_not_called()

    def test_native_camera_open_failure_aborts_without_retry(self):
        device = mock.Mock()
        device.get_info.side_effect = lambda key: "D435" if key == "serial" else "3.2"
        device.first_depth_sensor.return_value.get_depth_scale.return_value = 0.001
        device.query_sensors.return_value = []
        pipeline = mock.Mock()
        pipeline.start.side_effect = RuntimeError("Device or resource busy")
        self.sdk.camera_info = types.SimpleNamespace(serial_number="serial", usb_type_descriptor="usb")
        self.sdk.stream.depth = "depth"
        self.sdk.format = types.SimpleNamespace(bgr8="bgr8", z16="z16")
        self.sdk.context = lambda: types.SimpleNamespace(query_devices=lambda: [device])
        self.sdk.config = mock.Mock()
        self.sdk.pipeline = mock.Mock(return_value=pipeline)
        statuses, results = queue.Queue(), queue.Queue()
        with mock.patch.dict(sys.modules, {"pyrealsense2": self.sdk, "cv2": self.cv}), \
             mock.patch.object(rs, "CameraLease"):
            rs._capture("D435", queue.Queue(), results, statuses, threading.Event())
        self.assertEqual(statuses.get_nowait()["error"], "Device or resource busy")
        pipeline.start.assert_called_once()
        pipeline.wait_for_frames.assert_not_called()
        self.assertTrue(results.empty())

    def test_selection_fails_for_ambiguous_camera_without_using_other_card_configuration(self):
        sdk = types.SimpleNamespace(context=lambda: types.SimpleNamespace(query_devices=lambda: [object(), object()]))
        with mock.patch.dict(sys.modules, {"pyrealsense2": sdk}):
            with self.assertRaisesRegex(RuntimeError, "exactly one"):
                rs.SnapshotCameras().select()


class CaptureLogsafeTests(unittest.TestCase):
    def test_spawned_capture_protects_both_streams_before_sdk_startup(self):
        script = '''
import multiprocessing as mp
from pathlib import Path
import queue
import sys
import types
from unittest import mock

root = Path(sys.argv[1])
sys.path[:0] = [str(root), str(root / "realman/rm75_6f_v")]

def child():
    from common import logsafe
    from pick_place.camera import _capture
    assert not logsafe._installed

    def context():
        assert isinstance(sys.stdout, logsafe.LineAtomicStream)
        assert isinstance(sys.stderr, logsafe.LineAtomicStream)
        print("capture-stdout\\x00-safe", flush=True)
        print("capture-stderr\\x00-safe", file=sys.stderr, flush=True)
        raise RuntimeError("SDK startup checked")

    statuses = queue.Queue()
    with mock.patch.dict(sys.modules, {
        "cv2": types.SimpleNamespace(),
        "pyrealsense2": types.SimpleNamespace(context=context),
    }), mock.patch("pick_place.camera.CameraLease"):
        _capture("test", queue.Queue(), queue.Queue(), statuses, mp.Event())
    assert statuses.get_nowait() == {"error": "SDK startup checked"}

if __name__ == "__main__":
    worker = mp.get_context("spawn").Process(target=child)
    worker.start()
    worker.join(10)
    if worker.is_alive():
        worker.terminate()
        worker.join(2)
        raise AssertionError("capture child timed out")
    assert worker.exitcode == 0, worker.exitcode
'''
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            probe = Path(directory, "capture_probe.py")
            probe.write_text(script)
            result = subprocess.run([sys.executable, "-I", str(probe), str(root)],
                                    capture_output=True, timeout=20)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(b"capture-stdout-safe", result.stdout)
        self.assertIn(b"capture-stderr-safe", result.stderr)
        self.assertNotIn(b"\x00", result.stdout + result.stderr)
