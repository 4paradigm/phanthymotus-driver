"""RealMan RealSense wire format and shared RGB-D-IR ownership tests."""
import importlib.util
from pathlib import Path
import queue
import sys
import threading
import types
import unittest
from unittest import mock
import zlib

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / "realman/rm75_6f_v/realsense.py"
spec = importlib.util.spec_from_file_location("realman_realsense", SOURCE)
rs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rs)


class FakeDevice:
    def __init__(self, serial):
        self.serial = serial

    def supports(self, field):
        return field == "serial"

    def get_info(self, field):
        if field == "serial":
            return self.serial
        raise RuntimeError("unsupported")


class DeviceSelectionTests(unittest.TestCase):
    SDK = types.SimpleNamespace(
        camera_info=types.SimpleNamespace(serial_number="serial"))

    def test_selects_exact_serial_instead_of_first_usb_device(self):
        context = types.SimpleNamespace(
            query_devices=lambda: [FakeDevice("first"), FakeDevice("wanted")])
        selected = rs.find_device_by_serial(self.SDK, context, "wanted")
        self.assertEqual(selected.serial, "wanted")

    def test_missing_or_duplicate_serial_is_rejected(self):
        for devices in ([], [FakeDevice("other")], [FakeDevice("same"), FakeDevice("same")]):
            with self.subTest(devices=len(devices)), self.assertRaisesRegex(
                RuntimeError, "unavailable"):
                rs.find_device_by_serial(
                    self.SDK,
                    types.SimpleNamespace(query_devices=lambda: devices),
                    "same",
                )


class DepthEncodingTests(unittest.TestCase):
    def test_converts_device_units_to_millimetres_and_preserves_invalid(self):
        raw = np.zeros((480, 640), dtype=np.uint16)
        raw[0, :4] = [0, 4000, 8000, 65535]
        decoded = np.frombuffer(
            zlib.decompress(rs.encode_depth(raw, 0.00025)), dtype="<u2")
        self.assertEqual(decoded.size, 640 * 480)
        np.testing.assert_array_equal(decoded[:4], [0, 1000, 2000, 16384])

    def test_depth_overflow_does_not_wrap_to_a_near_object(self):
        raw = np.full((480, 640), 40000, dtype=np.uint16)
        decoded = np.frombuffer(
            zlib.decompress(rs.encode_depth(raw, 0.002)), dtype="<u2")
        self.assertFalse(np.any(decoded))

    def test_bad_depth_shape_dtype_and_scale_are_rejected(self):
        raw = np.zeros((480, 640), dtype=np.uint16)
        for scale in (0, -1, float("nan"), float("inf")):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                rs.encode_depth(raw, scale)
        for bad in (raw[:240], raw.astype(np.uint8)):
            with self.assertRaises(ValueError):
                rs.encode_depth(bad, 0.001)


class StatusQueue(queue.Queue):
    def cancel_join_thread(self):
        pass

    def close(self):
        pass


class FakeProcess:
    def __init__(self, *, target, args, **kwargs):
        self.quit = args[4]
        self.serial_number = args[1]
        self.alive = False
        self.closed = False

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive

    def join(self, timeout):
        if self.quit.is_set():
            self.alive = False

    def terminate(self):
        self.alive = False

    kill = terminate

    def close(self):
        self.closed = True


class RGBDLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.clock = mock.patch.object(rs.time, "monotonic", return_value=100.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.session = rs.RealSenseSession("robot_a", "serial-1234")
        self.session._ctx = types.SimpleNamespace(
            Event=threading.Event, Queue=StatusQueue, Process=FakeProcess)
        for instance_id in ("rgb-a", "depth-a", "ir-a"):
            self.addCleanup(self.session.stop, instance_id)

    def report(self, routes, error=None):
        self.session._queue.put({
            "frames": {key: 1 for key in routes},
            "last_frame": {key: self.now.return_value for key in routes},
            "channels": dict(routes),
            "error": error,
            "depth_scale_m": 0.001,
            "device_name": "D435",
            "usb_type": "3.2",
            "profiles": {
                "rgb": {"width": 1280, "height": 720, "fps": 15},
                "depth": {"width": 640, "height": 480, "fps": 15},
                "infrared": {"width": 640, "height": 480, "fps": 15},
            },
        })

    def test_all_modalities_share_owner_and_last_stop_releases_it(self):
        routes = {"rgb-a": "rgb", "depth-a": "depth", "ir-a": "infrared"}
        for instance_id, channel in routes.items():
            self.session.start(instance_id, channel)
        process = self.session._proc
        self.report(routes)
        self.assertIs(self.session._proc, process)
        for instance_id, channel in routes.items():
            self.assertEqual(self.session.info(instance_id, channel)["state"], "running")
        self.session.stop("depth-a")
        self.assertTrue(process.alive)
        self.session.stop("rgb-a")
        self.assertTrue(process.alive)
        self.session.stop("ir-a")
        self.assertTrue(process.closed)
        self.assertIsNone(self.session._proc)

    def test_rgb_status_reports_real_profile_and_topic(self):
        self.session.start("rgb-a", "rgb")
        self.report({"rgb-a": "rgb"})
        info = self.session.info("rgb-a", "rgb")
        self.assertTrue(info["fresh"])
        self.assertEqual((info["width"], info["height"], info["fps"]), (1280, 720, 15))
        self.assertEqual(info["encoding"], "bgr8")
        self.assertEqual(info["topic_out"], [{
            "topic": "/robot_a/ext_camera/rgb_a/rgb", "format": "image/jpeg",
        }])

    def test_start_and_stale_status_require_current_channel_frames(self):
        self.assertEqual(self.session.start("depth-a", "depth")["state"], "starting")
        self.report({"depth-a": "depth"})
        self.assertTrue(self.session.info("depth-a", "depth")["fresh"])
        process = self.session._proc
        self.now.return_value += 1
        self.assertEqual(
            self.session.start("depth-a", "infrared")["state"], "starting")
        self.report({"depth-a": "depth"})
        self.assertFalse(self.session.info("depth-a", "infrared")["fresh"])
        self.report({"depth-a": "infrared"})
        self.assertTrue(self.session.info("depth-a", "infrared")["fresh"])
        self.assertIs(self.session._proc, process)
        self.now.return_value += 3.1
        self.assertEqual(
            self.session.info("depth-a", "infrared")["state"], "error")

    def test_retry_restores_every_route_after_worker_failure(self):
        self.session.start("rgb-a", "rgb")
        self.session.start("depth-a", "depth")
        self.report({}, error="Device unavailable")
        self.assertEqual(self.session.info("rgb-a", "rgb")["state"], "error")
        old = self.session._proc
        self.session.start("rgb-a", "rgb")
        self.assertTrue(old.closed)
        self.assertEqual(
            self.session._wanted, {"rgb-a": "rgb", "depth-a": "depth"})

    def test_startup_timeout_is_not_running(self):
        self.session.start("rgb-a", "rgb")
        self.now.return_value += 10.1
        self.assertEqual(self.session.info("rgb-a", "rgb")["state"], "error")


class FakeConfig:
    def __init__(self):
        self.device = None
        self.streams = []

    def enable_device(self, serial):
        self.device = serial

    def enable_stream(self, *args):
        self.streams.append(args)


class FakeFrame:
    def __init__(self, data):
        self.data = data

    def get_data(self):
        return self.data


class FakeFrameSet:
    def __init__(self, kinds, data=None):
        self.kinds = set(kinds)
        data = data or {}
        self.frames = {
            kind: FakeFrame(data.get(kind)) for kind in self.kinds
        }

    def get_color_frame(self):
        return self.frames.get("rgb")

    def get_depth_frame(self):
        return self.frames.get("depth")

    def get_infrared_frame(self, index):
        return self.frames.get("infrared") if index == 1 else None


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.publishers = {}

    def create_publisher(self, _, topic, __):
        publisher = FakePublisher(topic)
        self.publishers[topic] = publisher
        return publisher

    def destroy_publisher(self, publisher):
        self.publishers.pop(publisher.topic, None)

    def get_clock(self):
        stamp = types.SimpleNamespace(to_msg=lambda: "stamp")
        return types.SimpleNamespace(now=lambda: stamp)

    def destroy_node(self):
        pass


class FakeCompressedImage:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id=None)
        self.format = None
        self.data = None


class RGBDCaptureTests(unittest.TestCase):
    """Exercise pipeline selection and health timing without RealSense hardware."""

    def capture(
        self, samples, usb_type="3.2", serial="serial-1234", routes=None,
    ):
        clock = [100.0]
        quit_event = threading.Event()
        statuses = queue.Queue()
        samples = iter(samples)
        config = FakeConfig()

        rgb_shape = (720, 1280, 3) if usb_type.startswith("3") else (480, 640, 3)
        frame_data = {
            "rgb": np.zeros(rgb_shape, dtype=np.uint8),
            "depth": np.ones((480, 640), dtype=np.uint16),
            "infrared": np.zeros((480, 640), dtype=np.uint8),
        }

        def wait_for_frames(timeout):
            self.assertEqual(timeout, 200)
            try:
                elapsed, kinds = next(samples)
            except StopIteration:
                quit_event.set()
                return FakeFrameSet(())
            clock[0] = 100.0 + elapsed
            return FakeFrameSet(kinds, frame_data)

        pipeline = types.SimpleNamespace(
            start=mock.Mock(), stop=mock.Mock(), wait_for_frames=wait_for_frames)
        depth_sensor = types.SimpleNamespace(get_depth_scale=lambda: 0.001)

        class Device:
            values = {"serial": serial, "usb": usb_type, "name": "D435"}

            def supports(self, key):
                return key in self.values

            def get_info(self, key):
                return self.values[key]

            def first_depth_sensor(self):
                return depth_sensor

        device = Device()
        context = types.SimpleNamespace(query_devices=lambda: [device])
        sdk = types.SimpleNamespace(
            context=lambda: context,
            config=lambda: config,
            pipeline=lambda selected_context: pipeline,
            camera_info=types.SimpleNamespace(
                serial_number="serial", usb_type_descriptor="usb", name="name"),
            stream=types.SimpleNamespace(
                color="color", depth="depth", infrared="infrared"),
            format=types.SimpleNamespace(bgr8="bgr8", z16="z16", y8="y8"),
        )
        node = FakeNode()
        ros = types.SimpleNamespace(
            init=mock.Mock(), create_node=mock.Mock(return_value=node), shutdown=mock.Mock())
        logsafe = types.SimpleNamespace(install=mock.Mock())
        modules = {
            "common": types.SimpleNamespace(logsafe=logsafe),
            "cv2": types.SimpleNamespace(
                IMWRITE_JPEG_QUALITY=1,
                imencode=mock.Mock(
                    return_value=(True, np.array([1, 2, 3], dtype=np.uint8))),
            ),
            "pyrealsense2": sdk,
            "rclpy": ros,
            "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=None),
            "sensor_msgs.msg": types.SimpleNamespace(
                CompressedImage=FakeCompressedImage),
        }
        with mock.patch.dict(sys.modules, modules), mock.patch.object(
            rs.time, "monotonic", side_effect=lambda: clock[0]
        ):
            rs._capture(
                "robot_a", serial, routes or {}, queue.Queue(), quit_event, statuses)
        errors = [item["error"] for item in list(statuses.queue) if item.get("error")]
        return errors, config, pipeline, ros.create_node.call_args.args[0], node

    def test_usb3_pipeline_enables_rgb_depth_and_infrared_once(self):
        errors, config, pipeline, node_name, _ = self.capture([
            (0.1, rs.STREAMS), (0.2, rs.STREAMS),
        ])
        self.assertEqual(errors, [])
        self.assertEqual(config.device, "serial-1234")
        self.assertEqual(config.streams, [
            ("color", 1280, 720, "bgr8", 15),
            ("depth", 640, 480, "z16", 15),
            ("infrared", 1, 640, 480, "y8", 15),
        ])
        pipeline.start.assert_called_once_with(config)
        pipeline.stop.assert_called_once_with()
        self.assertRegex(node_name, r"^robot_a_realsense_rgbd_[a-z0-9]+$")

    def test_usb2_uses_conservative_shared_profile(self):
        errors, config, _, _, _ = self.capture(
            [(0.1, rs.STREAMS)], usb_type="2.1")
        self.assertEqual(errors, [])
        self.assertEqual(config.streams, [
            ("color", 640, 480, "bgr8", 6),
            ("depth", 640, 480, "z16", 6),
            ("infrared", 1, 640, 480, "y8", 6),
        ])

    def test_slow_first_frames_get_startup_window(self):
        errors, _, _, _, _ = self.capture([
            (4.0, ()), (4.5, ("rgb",)),
            (5.0, ("depth",)), (5.5, ("infrared",)),
        ])
        self.assertEqual(errors, [])

    def test_missing_first_stream_eventually_times_out(self):
        errors, _, _, _, _ = self.capture([
            (0.1, ("rgb", "depth")), (10.1, ("rgb", "depth")),
        ])
        self.assertEqual(
            errors, ["RealSense RGB/depth/infrared startup timed out"])

    def test_stream_stall_after_startup_uses_shorter_deadline(self):
        errors, _, _, _, _ = self.capture([
            (0.1, rs.STREAMS), (3.2, ("rgb", "depth")),
        ])
        self.assertEqual(
            errors, ["RealSense RGB/depth/infrared frames stopped arriving"])

    def test_one_frameset_fans_out_all_three_canvas_streams(self):
        routes = {
            "rgb-card": "rgb",
            "depth-card": "depth",
            "infrared-card": "infrared",
        }
        errors, _, _, _, node = self.capture(
            [(0.1, rs.STREAMS)], routes=routes)
        self.assertEqual(errors, [])
        expected = {
            "/robot_a/ext_camera/rgb_card/rgb": "jpeg",
            "/robot_a/ext_camera/depth_card/depth":
                "16UC1; compressedDepth zlib",
            "/robot_a/ext_camera/infrared_card/infrared": "jpeg",
        }
        self.assertEqual(set(node.publishers), set(expected))
        for topic, image_format in expected.items():
            messages = node.publishers[topic].messages
            self.assertEqual(len(messages), 1)
            self.assertEqual(messages[0].format, image_format)
            self.assertTrue(messages[0].data)


if __name__ == "__main__":
    unittest.main()
