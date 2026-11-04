"""Exercise the hardware-depth wire contract without ROS or an Orbbec camera."""

import ast
from pathlib import Path
import sys
import threading
import time as real_time
import types
import zlib

import numpy as np
import pytest


class Node:
    def __init__(self, *args, **kwargs):
        self.messages = []

    def create_publisher(self, message_type, topic, qos):
        self.message_type = message_type
        return self

    def publish(self, message):
        assert isinstance(message, self.message_type)
        self.messages.append(message)

    def create_subscription(self, message_type, topic, callback, qos):
        self.input_type, self.input_topic = message_type, topic
        return callback

    def create_timer(self, period, callback):
        self.timer_period = period
        return callback

    def destroy_timer(self, handle):
        pass

    def destroy_subscription(self, handle):
        pass

    def destroy_publisher(self, handle):
        pass


@pytest.fixture
def depth_plugin(monkeypatch):
    # Isolate the production class from unrelated joint/ROS imports in device.py.
    source = Path(__file__).parents[1] / "device.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    definition = next(node for node in tree.body
                      if isinstance(node, ast.ClassDef) and node.name == "DepthCameraPlugin")
    clock = [10.0]
    enum = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, KEEP_LAST=1, VOLATILE=1)
    namespace = dict(Node=Node, threading=threading,
                     time=types.SimpleNamespace(monotonic=lambda: clock[0]),
                     QoSProfile=lambda **kwargs: kwargs,
                     ReliabilityPolicy=enum, HistoryPolicy=enum, DurabilityPolicy=enum)
    exec(compile(ast.Module(body=[definition], type_ignores=[]), str(source), "exec"), namespace)
    messages = types.ModuleType("sensor_msgs.msg")
    messages.Image = type("Image", (), {})
    messages.CompressedImage = type("CompressedImage", (), {})
    package = types.ModuleType("sensor_msgs")
    package.msg = messages
    monkeypatch.setitem(sys.modules, "sensor_msgs", package)
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", messages)
    # These tests use native 640x480 frames; resizing is not part of this change.
    monkeypatch.setitem(sys.modules, "cv2", types.ModuleType("cv2"))
    monkeypatch.setitem(sys.modules, "numpy", np)
    executor = types.SimpleNamespace(add_node=lambda node: None)
    ros = types.SimpleNamespace(ctx_tianyi=object(), ctx_core=object(),
                                executor_tianyi=executor, executor_core=executor)
    plugin = namespace["DepthCameraPlugin"]({}, "nvidia_desktop", ros)
    # Wire-format tests drive one pending frame synchronously. Background
    # scheduling has its own real-clock fixture below.
    plugin._start_worker = lambda: None
    plugin.start()
    yield plugin, clock, messages
    plugin.stop()


def frame(value=1200, padding=0):
    pixels = np.full((480, 640), value, dtype="<u2")
    pixels[0, :4] = [0, 1, 1200, 65535]
    rows = np.full((480, 1280 + padding), 255, dtype=np.uint8)
    rows[:, :1280] = pixels.view(np.uint8).reshape(480, 1280)
    message = types.SimpleNamespace(
        width=640, height=480, step=1280 + padding, encoding="16UC1",
        is_bigendian=0, data=rows.tobytes(),
        header=types.SimpleNamespace(stamp="source-stamp", frame_id="depth_optical"))
    return message, pixels


@pytest.mark.parametrize("padding", [0, 8])
def test_lossless_depth_and_header_reach_consumer(depth_plugin, padding):
    plugin, _, messages = depth_plugin
    message, expected = frame(padding=padding)
    plugin._on_image(message)
    plugin._publish_latest()
    output, = plugin._pub.messages
    assert isinstance(output, messages.CompressedImage)
    assert plugin._sub_node.input_type is messages.Image
    assert plugin._sub_node.input_topic == "/ob_camera_head/depth/image_raw"
    assert output.header is message.header
    assert output.format == "16UC1; compressedDepth zlib"
    raw = zlib.decompress(output.data)
    assert len(raw) == 640 * 480 * 2
    decoded = np.frombuffer(raw, dtype="<u2").reshape(480, 640)
    np.testing.assert_array_equal(decoded, expected)
    assert decoded[1, 1] / 1000 == 1.2
    expected_port = [{"topic": "/nvidia_desktop/camera/head/depth/visual_depth", "format": "image/depth-zlib"}]
    assert plugin.get_tool()["topic_out"] == expected_port
    assert plugin.dispatch("info", {})["topic_out"] == expected_port


def test_depth_uses_fast_lossless_rle_zlib_strategy(depth_plugin, monkeypatch):
    plugin, _, _ = depth_plugin
    original = zlib.compressobj
    called = {}

    def record_compressor(*args, **kwargs):
        called.update(kwargs)
        return original(*args, **kwargs)

    monkeypatch.setattr(zlib, "compressobj", record_compressor)
    message, expected = frame(1700)
    plugin._on_image(message)
    plugin._publish_latest()

    output, = plugin._pub.messages
    decoded = np.frombuffer(zlib.decompress(output.data), dtype="<u2").reshape(480, 640)
    np.testing.assert_array_equal(decoded, expected)
    assert called["level"] == 1
    assert called["strategy"] == zlib.Z_RLE
    info = plugin.dispatch("info", {})
    assert info["compression_strategy"] == "zlib-level1-rle"
    assert "last_transform_ms" in info and "last_compress_ms" in info


def test_wide_source_is_center_cropped_and_sampled_without_opencv(depth_plugin):
    plugin, _, _ = depth_plugin
    height, width = 720, 1280
    yy, xx = np.mgrid[:height, :width]
    source = ((xx + yy * 7) % 60000).astype("<u2")
    message = types.SimpleNamespace(
        width=width, height=height, step=width * 2, encoding="16UC1",
        is_bigendian=0, data=source.tobytes(),
        header=types.SimpleNamespace(stamp="source-stamp", frame_id="depth_optical"))

    actual = plugin._to_dashboard_depth(message)
    rows = np.arange(480, dtype=np.intp) * height // 480
    # 1280x720 is centre-cropped to 960x720 before sampling.
    cols = 160 + np.arange(640, dtype=np.intp) * 960 // 640
    expected = source[rows[:, None], cols[None, :]]
    np.testing.assert_array_equal(actual, expected)


@pytest.mark.parametrize("changes", [
    {"is_bigendian": 1}, {"encoding": "32FC1"}, {"width": 0},
    {"step": 1279}, {"data": b"short"},
])
def test_invalid_frame_is_not_published(depth_plugin, changes):
    plugin, _, _ = depth_plugin
    message, _ = frame()
    for key, value in changes.items():
        setattr(message, key, value)
    plugin._on_image(message)
    plugin._publish_latest()
    assert plugin._pub.messages == []


def test_backlog_is_replaced_and_stop_suppresses_output(depth_plugin):
    plugin, clock, _ = depth_plugin
    first, _ = frame(1000)
    newest, _ = frame(2000)
    plugin._on_image(first)
    plugin._on_image(newest)
    plugin._publish_latest()
    raw = zlib.decompress(plugin._pub.messages[0].data)
    assert np.frombuffer(raw, dtype="<u2")[640] == 2000
    plugin._on_image(first)
    plugin._publish_latest()
    assert len(plugin._pub.messages) == 2
    publisher = plugin._pub
    clock[0] += 0.2
    plugin.stop()
    plugin._publish_latest()
    assert len(publisher.messages) == 2
    assert plugin._latest_image is None


def test_manual_slots_preserve_15hz_accounting_without_a_ros_timer(depth_plugin):
    plugin, clock, _ = depth_plugin
    publisher = plugin._pub
    assert not hasattr(plugin._pub_node, "timer_period")
    beginning = clock[0]
    for index in range(16):
        clock[0] = beginning + index / 15
        plugin._on_image(frame(1500)[0])
        plugin._publish_latest()
        clock[0] += 0.01  # Processing must not postpone the next timer slot.
    assert len(publisher.messages) == 16
    assert plugin.dispatch("info", {})["target_hz"] == 15
    assert plugin.dispatch("info", {})["average_hz"] == pytest.approx(15, abs=0.2)


def test_no_source_replay_and_old_pending_frame_is_discarded(depth_plugin):
    plugin, clock, _ = depth_plugin
    plugin._on_image(frame()[0])
    clock[0] += 0.51
    plugin._publish_latest()
    plugin._publish_latest()
    assert not plugin._pub.messages
    assert plugin.dispatch("info", {})["stale_frames"] == 1
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    plugin._publish_latest()
    assert len(plugin._pub.messages) == 1


def test_repeated_start_is_idempotent_and_restart_reuses_endpoints(depth_plugin):
    plugin, _, _ = depth_plugin
    publisher, subscription = plugin._pub, plugin._subscription
    plugin.start()
    assert (plugin._pub, plugin._subscription) == (publisher, subscription)
    plugin.stop()
    assert (plugin._pub, plugin._subscription) == (publisher, subscription)
    plugin.start()
    assert (plugin._pub, plugin._subscription) == (publisher, subscription)
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    assert plugin.dispatch("info", {})["state"] == "running"


def test_frame_that_ages_during_conversion_is_not_published(depth_plugin, monkeypatch):
    plugin, clock, _ = depth_plugin
    original = plugin._to_dashboard_depth
    def delayed_conversion(message):
        result = original(message)
        clock[0] += 0.51
        return result
    monkeypatch.setattr(plugin, "_to_dashboard_depth", delayed_conversion)
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    assert not plugin._pub.messages
    assert plugin.dispatch("info", {})["stale_frames"] == 1


def test_old_generation_cannot_publish_across_restart(depth_plugin, monkeypatch):
    plugin, _, _ = depth_plugin
    original = plugin._to_dashboard_depth

    def restart_during_conversion(message):
        result = original(message)
        plugin.stop()
        plugin.start()
        return result

    monkeypatch.setattr(plugin, "_to_dashboard_depth", restart_during_conversion)
    plugin._on_image(frame()[0])
    plugin._publish_latest()
    assert plugin._pub.messages == []
    assert plugin.dispatch("info", {})["published_frames"] == 0


def test_single_publish_error_is_reported_and_next_frame_recovers(depth_plugin):
    plugin, _, _ = depth_plugin
    original_publish = plugin._pub.publish

    def fail(_message):
        return False

    plugin._pub.publish = fail
    plugin._on_image(frame(1000)[0])
    plugin._publish_latest()
    info = plugin.dispatch("info", {})
    assert info["publish_errors"] == 1
    assert info["last_error"] == "socket bridge did not accept depth frame"
    assert info["published_frames"] == 0

    plugin._pub.publish = original_publish
    plugin._on_image(frame(2000)[0])
    plugin._publish_latest()
    info = plugin.dispatch("info", {})
    assert info["published_frames"] == 1
    assert info["publish_errors"] == 1
    assert info["last_error"] == ""


def test_worker_coalesces_while_bridge_publish_is_busy(depth_plugin):
    plugin, _, _ = depth_plugin
    plugin.stop()
    # This fixture normally uses a deterministic clock and disables the
    # worker. Switch this isolated class namespace to a real clock here.
    plugin._publish_loop.__globals__["time"] = real_time
    del plugin.__dict__["_start_worker"]
    plugin.start()
    worker = plugin._worker
    plugin.start()
    assert plugin._worker is worker

    entered = threading.Event()
    release = threading.Event()
    original_publish = plugin._pub.publish

    def slow_first_publish(message):
        if not entered.is_set():
            entered.set()
            if not release.wait(2):
                raise TimeoutError("test bridge was not released")
        original_publish(message)

    plugin._pub.publish = slow_first_publish
    try:
        plugin._on_image(frame(1000)[0])
        assert entered.wait(1)
        plugin._on_image(frame(2000)[0])
        plugin._on_image(frame(3000)[0])
        release.set()
        deadline = real_time.monotonic() + 2
        while len(plugin._pub.messages) < 2 and real_time.monotonic() < deadline:
            real_time.sleep(0.01)
        assert len(plugin._pub.messages) == 2
        values = [np.frombuffer(zlib.decompress(item.data), dtype="<u2")[640]
                  for item in plugin._pub.messages]
        assert values == [1000, 3000]
        info = plugin.dispatch("info", {})
        assert info["coalesced_frames"] >= 1
        assert info["worker_alive"] is True
        assert info["last_payload_bytes"] > 0
        assert info["publish_errors"] == 0
    finally:
        release.set()
        plugin.stop()
        with plugin._frame_ready:
            plugin._worker_shutdown = True
            plugin._frame_ready.notify_all()
        plugin._worker.join(timeout=1)
