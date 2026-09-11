"""Go2 capture contract tests; real FFmpeg encoding, with a simulated ROS source."""

import __future__
import ast
from datetime import datetime
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


class FakeNode:
    def __init__(self, name):
        self.name = name
        self.callbacks = {}

    def create_subscription(self, message_type, topic, callback, qos):
        self.topic = topic
        self.callback = callback
        self.callbacks[topic] = callback
        return object()

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(time.time() * 1e9)))


def load_module():
    stubs = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.node", "rclpy.qos", "sensor_msgs", "sensor_msgs.msg")}
    stubs["rclpy.node"].Node = FakeNode
    stubs["rclpy.qos"].qos_profile_sensor_data = object()
    stubs["sensor_msgs.msg"].CompressedImage = object
    spec = importlib.util.spec_from_file_location("go2_vision_capture_test", ROOT / "unitree/go2/vision_capture.py")
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, stubs):
        spec.loader.exec_module(module)
    return module


capture = load_module()


def external_plugin(instances):
    """Run the real ext_camera info contract with hardware nodes replaced."""
    tree = ast.parse((ROOT / "unitree/go2/ext_devices.py").read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "ExtCameraPlugin")
    ns = {}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), "ext_devices.py", "exec",
                 flags=__future__.annotations.compiler_flag), ns)
    plugin = ns["ExtCameraPlugin"].__new__(ns["ExtCameraPlugin"])
    plugin._namespace = "go2"
    plugin._available_devices = []
    plugin._lock = threading.RLock()
    plugin._instance_configs = {key: {"channel": value.get("channel", "rgb"),
                                      "device_path": value.get("device_path", "")}
                               for key, value in instances.items()}
    plugin._nodes = {key: types.SimpleNamespace(_status_dict=lambda value=value: value)
                     for key, value in instances.items()}
    return plugin


def external_status(topic="/go2/ext_camera/usb_one/rgb"):
    return {"state": "running", "device_name": "RealSense (Color)", "device_path": "/dev/video4",
            "topic_out": [{"topic": topic, "format": "image/jpeg"}]}


class CaptureHarness(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.executor = mock.Mock()
        self.plugin = capture.VisionCapturePlugin({"output_dir": self.directory.name}, "go2", self.executor)
        self.front = self.plugin._resolve_source({})
        self.notifications = []
        self.plugin._notify_complete = lambda *args: self.notifications.append(args)

    def tearDown(self):
        self.plugin.stop()
        self.directory.cleanup()

    def feed(self, data=b"\xff\xd8test\xff\xd9", age=0, fmt="jpeg", topic="/go2/camera/front"):
        timestamp = time.time() - age
        msg = types.SimpleNamespace(
            format=fmt, data=data,
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(
                sec=int(timestamp), nanosec=int((timestamp % 1) * 1e9))))
        callback = self.plugin._node.callbacks.get(topic)
        if callback:
            callback(msg)

    def publish(self, data, topic="/go2/camera/front", fps=15):
        halt = threading.Event()

        def produce():
            while not halt.is_set():
                self.feed(data, topic=topic)
                halt.wait(1 / fps)

        thread = threading.Thread(target=produce, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 2)
        self.addCleanup(halt.set)
        return halt

    def wait_recording(self, timeout=8):
        deadline = time.monotonic() + timeout
        while not self.notifications and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertEqual(len(self.notifications), 1)
        return self.notifications[0]

    def wait_last_recording(self, timeout=12):
        """Wait for the terminal outcome in info.last_recording (no ACP)."""
        deadline = time.monotonic() + timeout
        timeline = None
        while time.monotonic() < deadline:
            info = self.plugin._info()
            record = info.get("last_recording")
            if record and record.get("status") in ("completed", "cancelled", "error"):
                return record
            time.sleep(0.02)
        self.fail(f"recording did not finish: {info.get('last_recording')!r}")


class CaptureTest(CaptureHarness):
    def test_external_selection_excludes_depth_and_infrared_jpeg(self):
        self.plugin._external_camera = external_plugin({
            "color": {**external_status("/go2/ext_camera/color/rgb"), "channel": "rgb"},
            "ir": {**external_status("/go2/ext_camera/ir/infrared"), "channel": "infrared"},
            "depth": {**external_status("/go2/ext_camera/depth/depth"), "channel": "depth"},
        })
        source = self.plugin._resolve_source({"camera": "external"})
        self.assertEqual(source["external_instance_id"], "color")
        self.assertEqual(len(self.plugin.dispatch("list_cameras", {})["cameras"]), 2)
        with self.assertRaisesRegex(ValueError, "infrared.*only RGB"):
            self.plugin._resolve_source({"camera": "external", "external_instance_id": "ir"})

    def test_selects_each_camera_and_uses_actual_external_topic(self):
        topic = "/go2/ext_camera/usb_one/rgb"
        self.plugin._external_camera = external_plugin({"usb-one": external_status(topic)})
        front_jpeg, external_jpeg = b"\xff\xd8front\xff\xd9", b"\xff\xd8external\xff\xd9"
        self.publish(front_jpeg)
        self.publish(external_jpeg, topic)
        result = self.plugin.dispatch("config", {"camera": "external"})
        self.assertTrue(result["ok"])
        external = self.plugin.dispatch("capture_photo", {})
        self.assertTrue(external["ok"], external)
        self.assertEqual(external["source"]["external_instance_id"], "usb-one")
        self.assertEqual(external["source"]["topic"], topic)
        self.assertEqual(Path(external["file_path"]).read_bytes(), external_jpeg)
        front = self.plugin.dispatch("capture_photo", {"camera": "front"})
        self.assertTrue(front["ok"], front)
        self.assertEqual(Path(front["file_path"]).read_bytes(), front_jpeg)
        self.assertEqual(self.plugin._camera, "external", "per-call override must not change configured default")

    def test_multiple_external_cameras_require_explicit_instance(self):
        self.plugin._external_camera = external_plugin({
            "usb-one": external_status(), "usb-two": external_status("/go2/ext_camera/usb_two/rgb")})
        listed = self.plugin.dispatch("list_cameras", {})["cameras"]
        self.assertEqual(len(listed), 3)
        self.assertEqual(self.plugin.dispatch("capture_photo", {"camera": "external"})["code"], "CAPTURE_FAILED")
        source = self.plugin._resolve_source({"camera": "external", "external_instance_id": "usb-two"})
        self.assertEqual(source["topic"], "/go2/ext_camera/usb_two/rgb")

    def test_missing_external_camera_never_falls_back_to_front(self):
        self.publish(b"\xff\xd8front\xff\xd9")
        self.plugin._external_camera = external_plugin({})
        self.assertFalse(self.plugin.dispatch("capture_photo", {"camera": "external"})["ok"])
        with mock.patch.object(capture.shutil, "which", return_value="ffmpeg"):
            result = self.plugin.dispatch("record_video", {"camera": "external"})
        self.assertFalse(result["ok"])
        self.assertNotIn("action_id", result)
        self.assertEqual(list(Path(self.directory.name).rglob("*.jpg")), [])

    def test_bundle_wires_existing_external_plugin(self):
        tree = ast.parse((ROOT / "unitree/go2/main.py").read_text())
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Go2DeviceBundle")
        ext = external_plugin({"usb-one": external_status()})
        stub = types.ModuleType("ext_devices")
        stub.ExtCameraPlugin = lambda *args: ext
        ns = {}
        with mock.patch.dict(sys.modules, {"vision_capture": capture, "ext_devices": stub}):
            exec(compile(ast.Module(body=[cls], type_ignores=[]), "main.py", "exec",
                         flags=__future__.annotations.compiler_flag), ns)
            bundle = ns["Go2DeviceBundle"]({"plugins": {"ext_camera": {"enabled": True},
                                            "vision_capture": {"enabled": True}}}, "go2", mock.Mock(), None)
        plugin = bundle._plugins[-1]
        self.assertIs(plugin._external_camera, ext)
        self.assertEqual(plugin.dispatch("list_cameras", {})["cameras"][1]["external_instance_id"], "usb-one")

    def test_registration_and_configured_source(self):
        # Exercise the actual bundle class without importing hardware SDKs.
        tree = ast.parse((ROOT / "unitree/go2/main.py").read_text())
        bundle = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Go2DeviceBundle")
        module = ast.Module(body=[bundle], type_ignores=[])
        ns = {"RpcProxy": object}
        with mock.patch.dict(sys.modules, {"vision_capture": capture}):
            exec(compile(module, "go2/main.py", "exec", flags=__future__.annotations.compiler_flag), ns)
            instance = ns["Go2DeviceBundle"]({"plugins": {"vision_capture": {
                "enabled": True, "camera": "external"}}}, "go2", mock.Mock(), None)
        try:
            tools = instance.get_all_tools()
            self.assertIn("vision_capture", [tool["name"] for tool in tools])
            info = instance.dispatch("vision_capture", {"action": "info"})
            self.assertEqual(info["camera"], "external")
            self.assertFalse(info["ok"])
        finally:
            instance.stop_all()

    def test_start_returns_lifecycle_state_and_info_tracks_freshness(self):
        # The actuator contract requires the start action to return the bare
        # lifecycle state; camera freshness stays on the info action.
        self.assertEqual(self.plugin.start(), {"state": "ready"})
        self.assertEqual(self.plugin.dispatch("start", {}), {"state": "ready"})
        self.assertFalse(self.plugin.dispatch("info", {})["ok"])
        self.feed()
        self.assertTrue(self.plugin.dispatch("info", {})["ok"])
        json.dumps(self.plugin.dispatch("info", {}))

    def test_tool_does_not_advertise_async_completion(self):
        # record_video is admission-synchronous: without x-completion, Agent
        # Core never registers a pending action nor awaits /api/acp/complete.
        self.assertNotIn("x-completion", self.plugin.get_tool()["inputSchema"])

    def test_driver_yaml_marketplace_lists_vision_capture(self):
        # config.yaml enables the card, so driver.yaml's hand-synced
        # marketplace list must expose it too.
        text = (ROOT / "unitree" / "go2" / "driver.yaml").read_text()
        self.assertIn("{ name: vision_capture, type: actuator }", text)

    def test_photo_bytes_and_persistence_across_plugin_restart(self):
        data = b"\xff\xd8photo\xff\xd9"
        self.publish(data)
        result = self.plugin.dispatch("capture_photo", {})
        self.assertTrue(result["ok"])
        path = Path(result["file_path"])
        self.assertEqual(path.parent.name, "photos")
        self.plugin.stop()
        self.plugin.start()
        self.assertEqual(path.read_bytes(), data)

    def test_invalid_and_delayed_images_are_rejected(self):
        for data, age, fmt in [(b"bad", 0, "jpeg"), (b"\xff\xd8x\xff\xd9", 10, "jpeg"),
                               (b"\xff\xd8x\xff\xd9", 0, "png")]:
            self.feed(data, age, fmt)
            with self.assertRaises(RuntimeError):
                self.plugin._frame(self.front, timeout_s=0)

    def test_cache_expires_and_sequence_cannot_be_reused(self):
        self.feed()
        frame = self.plugin._frame(self.front, timeout_s=0)
        with self.assertRaises(RuntimeError):
            self.plugin._frame(self.front, after_sequence=frame[2], timeout_s=0)
        self.plugin._streams[self.front["topic"]]["latest"] = (frame[0], time.monotonic() - 4, frame[2])
        with self.assertRaises(RuntimeError):
            self.plugin._frame(self.front, timeout_s=0)

    def test_photo_failure_leaves_no_file(self):
        with mock.patch.object(self.plugin, "_frame", side_effect=RuntimeError("no camera")):
            self.assertEqual(self.plugin.dispatch("capture_photo", {})["code"], "CAPTURE_FAILED")
        self.assertEqual(list(Path(self.directory.name).rglob("*.jpg")), [])

    def test_duration_validation_and_schema_agree(self):
        # Default config: max 30 s, default 5 s.
        schema = self.plugin.get_tool()["inputSchema"]
        self.assertEqual(schema["properties"]["duration_s"]["maximum"], 30)
        self.assertEqual(schema["properties"]["duration_s"]["default"], 5)
        for value in (True, 1.5, "2", None, 0, 31):
            self.assertEqual(self.plugin.dispatch("record_video", {"duration_s": value})["code"], "INVALID_DURATION")
        # The literal default stays 5 even when the configured cap changes;
        # the validation cap follows the config independently.
        self.plugin._max_duration_s = 2
        schema = self.plugin.get_tool()["inputSchema"]
        self.assertEqual(schema["properties"]["duration_s"]["maximum"], 2)
        self.assertEqual(schema["properties"]["duration_s"]["default"], 5)
        self.assertEqual(self.plugin.dispatch("record_video", {"duration_s": 6})["code"], "INVALID_DURATION")

    def test_missing_encoder_is_immediate_error(self):
        with mock.patch.object(capture.shutil, "which", return_value=None):
            result = self.plugin.dispatch("record_video", {})
        self.assertEqual(result["code"], "RECORD_FAILED")
        self.assertEqual(self.notifications, [])

    def test_cancel_waiting_for_first_frame_and_reject_overlap(self):
        with mock.patch.object(capture.shutil, "which", return_value="ffmpeg"):
            started = self.plugin.dispatch("record_video", {})
            self.assertTrue(started["ok"])
            json.dumps(self.plugin.dispatch("info", {}))
            duplicate = self.plugin.dispatch("record_video", {})
            self.assertEqual(duplicate["code"], "RECORD_IN_PROGRESS")
            self.assertNotIn("action_id", duplicate)
            self.assertEqual(self.plugin.dispatch("config", {"camera": "external"})["code"], "RECORD_IN_PROGRESS")
            self.assertTrue(self.plugin.dispatch("config", {"camera": "front"})["ok"])
            self.assertEqual(self.plugin.stop()["state"], "idle")
        self.assertIsNone(self.plugin._active_recording)
        record = self.wait_last_recording()
        self.assertEqual(record["action_id"], started["action_id"])
        self.assertEqual(record["status"], "cancelled")
        self.assertEqual(record["result"]["code"], "RECORD_CANCELLED")

    def test_no_camera_reports_async_error_without_action_id(self):
        with mock.patch.object(capture.shutil, "which", return_value="ffmpeg"), \
             mock.patch.object(self.plugin, "_frame", side_effect=RuntimeError("no camera")):
            started = self.plugin.dispatch("record_video", {})
        # The queued response carries the destination file_path synchronously,
        # like capture_photo, even when the background encode later fails.
        self.assertTrue(started["ok"])
        self.assertIn("file_path", started)
        self.assertEqual(Path(started["file_path"]).parent.name, "videos")

    def test_stop_waits_for_slow_worker_before_returning(self):
        # stop() joins the worker thread: it must block until the background
        # encode fully settles (active cleared, last_recording committed).
        entered, release = threading.Event(), threading.Event()

        def slow_record(active):
            entered.set()
            release.wait(3)
            return {"ok": True, "file_path": str(Path(self.directory.name) / "videos" / "slow.mp4")}

        self.plugin._record_video = slow_record
        with mock.patch.object(capture.shutil, "which", return_value="ffmpeg"):
            self.plugin.dispatch("record_video", {})
            self.assertTrue(entered.wait(2))
            stopper = threading.Thread(target=self.plugin.stop)
            stopper.start()
            try:
                self.assertTrue(stopper.is_alive(), "stop must block until the worker finishes")
                release.set()
                stopper.join(3)
            finally:
                release.set()
        self.assertFalse(stopper.is_alive())
        # stop() set the cancel flag while the worker was still running, so the
        # terminal outcome is cancelled even though encoding would have succeeded.
        self.assertEqual(self.plugin._info()["last_recording"]["status"], "cancelled")
        self.assertIsNone(self.plugin._active_recording)


@unittest.skipUnless(shutil.which("ffmpeg") and shutil.which("ffprobe"), "FFmpeg integration requires ffmpeg and ffprobe")
class EncoderTest(CaptureHarness):
    @classmethod
    def setUpClass(cls):
        cls.jpeg = subprocess.check_output([
            "ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "lavfi",
            "-i", "testsrc=size=160x120:rate=15", "-frames:v", "1", "-f", "image2pipe", "-c:v", "mjpeg", "pipe:1"])

    def source(self, fps=15):
        return self.publish(self.jpeg, fps=fps)

    def test_real_photo_and_video(self):
        self.source()
        photo = self.plugin.dispatch("capture_photo", {})
        subprocess.run(["ffmpeg", "-v", "error", "-i", photo["file_path"], "-f", "null", "-"], check=True)
        self.assertEqual(self.notifications, [], "capture_photo must not trigger ACP completion")
        self.plugin.dispatch("record_video", {"duration_s": 1})
        record = self.wait_last_recording()
        self.assertEqual(record["status"], "completed")
        result = record["result"]
        probe = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-show_streams", "-show_format", "-of", "json", result["file_path"]]))
        self.assertEqual(probe["streams"][0]["codec_name"], "h264")
        self.assertEqual(probe["streams"][0]["pix_fmt"], "yuv420p")
        self.assertAlmostEqual(float(probe["format"]["duration"]), 1, delta=0.001)
        self.assertGreater(result["frames"], 8)
        self.assertEqual(self.plugin._info()["last_recording"]["status"], "completed")
        # The destination path was already returned synchronously on admission.
        self.assertEqual(self.notifications, [])

    def test_no_notify_for_record_video(self):
        # No ACP terminal notification is posted for record_video any more;
        # the destination path is returned synchronously like capture_photo.
        self.source()
        self.plugin.dispatch("record_video", {"duration_s": 1})
        record = self.wait_last_recording()
        self.assertEqual(self.notifications, [])
        self.assertEqual(record["status"], "completed")

    def test_video_result_matches_q5_contract(self):
        # The completed video result is retained for the info card; no ACP
        # terminal notification is posted. The slim q5 field set is unchanged.
        self.source()
        self.plugin.dispatch("record_video", {"duration_s": 1})
        record = self.wait_last_recording()
        result = record["result"]
        self.assertTrue(result["ok"])
        self.assertEqual(result["media_type"], "video")
        self.assertEqual(result["recorded_duration_s"], 1)
        self.assertGreaterEqual(result["frames"], 1)
        self.assertIn("captured_at", result)
        fields = {"ok", "media_type", "file_path", "recorded_duration_s",
                  "frames", "captured_at"}
        self.assertFalse(set(result) - fields)
        self.assertIn("file_path", result)

    def test_slow_source_preserves_video_timing(self):
        self.source(fps=5)
        self.plugin.dispatch("record_video", {"duration_s": 2})
        record = self.wait_last_recording()
        self.assertEqual(record["status"], "completed", record)
        result = record["result"]
        duration = float(subprocess.check_output([
            "ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", result["file_path"]]))
        self.assertAlmostEqual(duration, 2, delta=0.001)

    def test_five_second_video_has_exact_duration_and_complete_last_frame(self):
        self.source(fps=7)
        self.plugin.dispatch("record_video", {"duration_s": 5})
        record = self.wait_last_recording(timeout=12)
        self.assertEqual(record["status"], "completed", record)
        result = record["result"]
        probe = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-count_frames", "-show_entries",
            "format=duration:stream=duration,nb_read_frames,avg_frame_rate", "-of", "json", result["file_path"]]))
        self.assertAlmostEqual(float(probe["format"]["duration"]), 5, delta=0.001)
        self.assertAlmostEqual(float(probe["streams"][0]["duration"]), 5, delta=0.001)
        self.assertEqual(int(probe["streams"][0]["nb_read_frames"]), 75)
        self.assertEqual(probe["streams"][0]["avg_frame_rate"], "15/1")
        self.assertEqual(result["recorded_duration_s"], float(probe["format"]["duration"]))

    def test_one_second_recording_at_one_fps(self):
        self.plugin._fps = 1
        self.source()
        self.plugin.dispatch("record_video", {"duration_s": 1})
        record = self.wait_last_recording()
        self.assertEqual(record["status"], "completed", record)
        result = record["result"]
        duration = float(subprocess.check_output([
            "ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", result["file_path"]]))
        self.assertAlmostEqual(duration, 1, delta=0.1)

    def test_encoder_rejects_corrupt_frames_and_removes_partial_file(self):
        self.publish(b"\xff\xd8corrupt\xff\xd9")
        self.plugin.dispatch("record_video", {"duration_s": 1})
        record = self.wait_last_recording()
        self.assertEqual(record["status"], "error", record)
        result = record["result"]
        self.assertEqual(result["code"], "RECORD_FAILED")
        self.assertEqual(list(Path(self.directory.name).rglob("*.mp4")), [])

    def test_finished_recording_cleared_only_after_validation_finishes(self):
        self.source()
        entered, release = threading.Event(), threading.Event()
        probe = self.plugin._probe_video

        def held_probe(*args):
            entered.set()
            if not release.wait(5):
                raise RuntimeError("test validation gate timed out")
            return probe(*args)

        with mock.patch.object(self.plugin, "_probe_video", side_effect=held_probe):
            self.plugin.dispatch("record_video", {"duration_s": 1})
            try:
                self.assertTrue(entered.wait(4))
                # No ACP notification and the recording is not yet final.
                self.assertEqual(self.notifications, [])
                self.assertIsNotNone(self.plugin._info()["active_recording"])
            finally:
                release.set()
            record = self.wait_last_recording()
        self.assertEqual(record["status"], "completed")

    def test_failed_file_validation_reports_error_and_removes_output(self):
        self.source()
        with mock.patch.object(self.plugin, "_probe_video", side_effect=RuntimeError("duration mismatch")):
            self.plugin.dispatch("record_video", {"duration_s": 1})
            record = self.wait_last_recording()
        self.assertEqual(record["status"], "error")
        result = record["result"]
        self.assertEqual(result["code"], "RECORD_FAILED")
        # Failure result carries the q5 slim failure contract: ok/code/message only.
        self.assertEqual(set(result), {"ok", "code", "message"})
        self.assertFalse(result["ok"])
        self.assertEqual(list(Path(self.directory.name).rglob("*.mp4")), [])

    def test_external_video_keeps_selected_camera_while_front_is_publishing(self):
        topic = "/go2/ext_camera/usb_one/rgb"
        self.plugin._external_camera = external_plugin({"usb-one": external_status(topic)})
        blue = subprocess.check_output([
            "ffmpeg", "-v", "error", "-f", "lavfi", "-i", "color=c=blue:s=160x120",
            "-frames:v", "1", "-f", "image2pipe", "-c:v", "mjpeg", "pipe:1"])
        self.source()
        self.publish(blue, topic)
        self.plugin.dispatch("record_video", {"camera": "external", "duration_s": 1})
        record = self.wait_last_recording()
        self.assertEqual(record["status"], "completed", record)
        result = record["result"]
        # The external instance id is no longer echoed in the q5-slim result;
        # the blue-frame decode still proves the selected camera was used.
        self.assertNotIn("source", result)
        # Decode all frames: a front-camera test pattern would not remain blue.
        pixels = subprocess.check_output([
            "ffmpeg", "-v", "error", "-i", result["file_path"], "-vf", "scale=1:1",
            "-pix_fmt", "rgb24", "-f", "rawvideo", "pipe:1"])
        self.assertGreater(len(pixels), 15)
        for i in range(0, len(pixels), 3):
            red, green, blue = pixels[i:i+3]
            self.assertLess(red, 20)
            self.assertLess(green, 20)
            self.assertGreater(blue, 220)

    def test_cancel_active_encoder_cleans_partial_video(self):
        self.source()
        self.plugin.dispatch("record_video", {"duration_s": 5})
        deadline = time.monotonic() + 3
        process = None
        while time.monotonic() < deadline:
            with self.plugin._recording_lock:
                process = self.plugin._active_recording.get("process") if self.plugin._active_recording else None
            if process:
                break
            time.sleep(0.02)
        self.assertIsNotNone(process)
        self.assertEqual(self.plugin.stop()["state"], "idle")
        record = self.wait_last_recording()
        self.assertEqual(record["status"], "cancelled")
        self.assertIsNotNone(process.poll())
        self.assertEqual(list(Path(self.directory.name).rglob("*.mp4")), [])

    def test_stalled_source_errors_and_deletes_partial_video(self):
        halt = self.source()
        self.plugin.dispatch("record_video", {"duration_s": 5})
        time.sleep(0.4)
        halt.set()
        record = self.wait_last_recording()
        self.assertEqual(record["status"], "error", record)
        self.assertEqual(list(Path(self.directory.name).rglob("*.mp4")), [])


if __name__ == "__main__":
    unittest.main()
