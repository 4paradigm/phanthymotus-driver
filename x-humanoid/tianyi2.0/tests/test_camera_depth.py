"""Exercise the hardware-depth wire contract without ROS or an Orbbec camera."""

import ast
from pathlib import Path
import sys
import threading
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
        return callback


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
    expected_port = [{"topic": "/nvidia_desktop/camera/head/depth", "format": "image/depth-zlib"}]
    assert plugin.get_tool()["topic_out"] == expected_port
    assert plugin.dispatch("info", {})["topic_out"] == expected_port


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
    assert len(plugin._pub.messages) == 1  # 8 Hz rate limit
    clock[0] += 0.2
    plugin.stop()
    plugin._publish_latest()
    assert len(plugin._pub.messages) == 1
    assert plugin._latest_image is None
