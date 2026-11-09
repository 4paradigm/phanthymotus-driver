"""Real OpenCV geometry and processor lifecycle; no robot or camera is opened."""

from __future__ import annotations

import importlib
import json
import sys
import threading
import time
import types
from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml
from jsonschema import validate


ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix" / "bumi"


@pytest.fixture
def rect(monkeypatch):
    monkeypatch.syspath_prepend(str(BUMI))
    return importlib.import_module("planar_rectify")


def encode(image):
    ok, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 98])
    assert ok
    return data.tobytes()


def scene():
    """Synthetic printed page projected into a known tilted quadrilateral."""
    page = np.full((300, 220, 3), 245, np.uint8)
    for y in range(50, 270, 30):
        cv2.line(page, (35, y), (185, y), (30, 30, 30), 2)
    cv2.circle(page, (30, 30), 10, (0, 0, 230), -1)
    cv2.circle(page, (190, 30), 10, (0, 200, 0), -1)
    cv2.circle(page, (190, 270), 10, (220, 0, 0), -1)
    quad = np.float32([[150, 55], [430, 100], [470, 420], [100, 390]])
    source = np.float32([[0, 0], [219, 0], [219, 299], [0, 299]])
    transform = cv2.getPerspectiveTransform(source, quad)
    image = cv2.warpPerspective(page, transform, (640, 480), borderValue=(30, 30, 30))
    return page, image, quad


def manual_config(rect):
    _, _, quad = scene()
    return rect.settings({"mode": "manual", "corners": (quad / [639, 479]).tolist(),
                          "output_width": 220, "output_height": 300, "max_fps": 5})


def test_manual_rectification_restores_pixels_orientation_and_homography(rect):
    page, image, quad = scene()
    report, output = rect.rectify(encode(image), manual_config(rect))
    decoded = cv2.imdecode(np.frombuffer(output, np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == page.shape
    assert report["valid"] and report["status"] == "ok"
    assert report["metric_geometry"] is False
    mapped = cv2.perspectiveTransform(quad[None], np.asarray(report["homography"]))[0]
    assert np.allclose(mapped, [[0, 0], [219, 0], [219, 299], [0, 299]], atol=0.001)
    error = np.mean(np.abs(decoded[10:-10, 10:-10].astype(float) - page[10:-10, 10:-10]))
    assert error < 12
    assert decoded[30, 30, 2] > 180  # red top-left retained; no mirror/rotation
    assert decoded[30, 190, 1] > 150
    assert decoded[270, 190, 0] > 170


def test_auto_detects_tilted_boundary_without_claiming_document_identity(rect):
    _, image, quad = scene()
    report, output = rect.rectify(encode(image), rect.settings({}))
    assert output is not None and report["valid"]
    assert report["selection"] == "geometric_candidate"
    assert np.max(np.linalg.norm(np.asarray(report["corners_px"]) - quad, axis=1)) < 5


def test_no_boundary_and_ambiguous_pages_never_emit_a_fake_scan(rect):
    blank = np.full((480, 640, 3), 80, np.uint8)
    report, output = rect.rectify(encode(blank), rect.settings({}))
    assert report["status"] == "no_boundary" and not report["valid"]
    assert output is None and report["homography"] is None
    cv2.rectangle(blank, (35, 55), (275, 425), (245, 245, 245), -1)
    cv2.rectangle(blank, (350, 70), (600, 420), (245, 245, 245), -1)
    report, output = rect.rectify(encode(blank), rect.settings({}))
    assert report["status"] == "ambiguous_boundary" and not report["valid"]
    assert output is None


def test_cropped_page_is_not_filled_in_from_an_invented_boundary(rect):
    image = np.full((480, 640, 3), 30, np.uint8)
    cv2.rectangle(image, (-10, 50), (450, 430), (245, 245, 245), -1)
    report, output = rect.rectify(encode(image), rect.settings({}))
    assert not report["valid"] and output is None


@pytest.mark.parametrize("cfg", [
    {"mode": "manual"}, {"mode": "other"}, {"max_fps": float("nan")},
    {"max_fps": True}, {"output_width": 50000}, {"output_height": 64.5},
    {"corners": [[0, 0], [1, 1], [1, 0], [0, 1]]},
    {"corners": [[0, 0], [1, 0], [0.5, 0], [0, 1]]},
    {"corners": [[0, 0], [1, 0], [1, 1], [-1, 1]]},
    {"corners": [[True, 0], [1, 0], [1, 1], [0, 1]]},
    {"corners": None},
])
def test_invalid_configuration(rect, cfg):
    with pytest.raises(ValueError):
        rect.settings(cfg)


def test_jpeg_is_bounded_before_decode_and_invalid_data_fails(rect, monkeypatch):
    data = encode(np.zeros((480, 640, 3), np.uint8))
    assert rect.jpeg_size(data) == (640, 480)
    oversized = bytearray(data)
    index = oversized.find(b"\xff\xc0")
    oversized[index + 5:index + 9] = b"\x7f\xff\x7f\xff"
    monkeypatch.setattr(rect.cv2, "imdecode", lambda *_: pytest.fail("must reject before allocation"))
    for bad in (bytes(oversized), b"not jpeg", data[:20], b"x" * (rect.MAX_JPEG_BYTES + 1)):
        with pytest.raises(ValueError):
            rect.rectify(bad, rect.settings({}))


class Node:
    def __init__(self):
        self.images = []
        self.reports = []
        self.destroyed = False

    def send_image(self, data, stamp):
        self.images.append((data, stamp))

    def send_status(self, report):
        self.reports.append(report)

    def destroy_node(self):
        self.destroyed = True


class Executor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        self.nodes.remove(node)


def message(data, sec=123):
    return types.SimpleNamespace(data=data, header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(sec=sec, nanosec=0)))


def wait_for(predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        assert time.monotonic() < deadline, "processor did not reach expected state"
        time.sleep(0.01)


@pytest.fixture
def plugin(rect, monkeypatch):
    executor, node = Executor(), Node()
    obj = rect.PlanarRectifyPlugin(manual_config(rect), "test", executor)
    monkeypatch.setattr(obj, "_make_node", lambda: node)
    yield obj, node, executor
    obj.stop()


def test_worker_is_latest_only_preserves_stamp_and_clears_on_stop(rect, plugin):
    obj, node, executor = plugin
    obj.start()
    first = obj._thread
    obj.start()
    assert first is obj._thread and len(executor.nodes) == 1
    data = encode(scene()[1])
    for sec in range(123, 128):
        obj._on_frame(message(data, sec))
    wait_for(lambda: bool(node.images))
    assert len(node.images) == 1 and node.images[0][1] == (127, 0)
    assert obj.info()["frame_sequence"] == 5
    assert obj.info()["capture_time_verified"] is False
    assert obj.info()["camera_info"] == []
    with obj._lock:
        seq, data, received, stamp = obj._latest
        obj._latest = (seq, data, received - 10, stamp)
    assert obj.info()["status"] == "stale_input"
    assert obj.info()["valid"] is False
    assert obj.info()["homography"] is None
    assert obj.dispatch("stop", {}) == {"state": "idle"}
    assert not first.is_alive() and not executor.nodes and node.destroyed
    assert node.reports[-1]["status"] == "idle"
    assert node.reports[-1]["valid"] is False
    assert obj._latest is None and obj.info()["status"] == "idle"
    obj._on_frame(message(data))
    assert obj._latest is None


def test_bad_config_preserves_running_instance_and_replayed_frames_invalidate(rect, plugin):
    obj, _, _ = plugin
    obj.start()
    thread = obj._thread
    result = obj.dispatch("config", {"output_width": 99999})
    assert result["error"] == "invalid_parameters"
    assert obj._thread is thread and thread.is_alive()
    result = obj.dispatch("start", {"input_topics": ["/one", "/two"]})
    assert result["error"] == "invalid_parameters" and obj._thread is thread
    assert obj.dispatch("start", {"input_topic": obj._image_topic})["state"] == "error"
    data = encode(scene()[1])
    obj._on_frame(message(data, 3))
    obj._on_frame(message(data, 3))
    assert obj.info()["status"] == "replayed_input" and obj._latest is None
    obj._on_frame(message(data, 2))
    assert obj.info()["status"] == "replayed_input"
    obj._on_frame(message(data, 4))
    wait_for(lambda: obj.info()["valid"])


def test_stop_during_processing_cancels_publication(rect, plugin, monkeypatch):
    obj, node, executor = plugin
    entered, release = threading.Event(), threading.Event()
    real_rectify = rect.rectify

    def paused(*args):
        entered.set()
        assert release.wait(3)
        return real_rectify(*args)

    monkeypatch.setattr(rect, "rectify", paused)
    obj.start()
    obj._on_frame(message(encode(scene()[1])))
    assert entered.wait(2)
    stopper = threading.Thread(target=obj.stop)
    stopper.start()
    wait_for(obj._cancel.is_set)
    release.set()
    stopper.join(3)
    assert not stopper.is_alive()
    assert not node.images and not executor.nodes and node.destroyed
    assert not obj.info()["valid"]


def test_rejected_newer_frame_cannot_be_overwritten_by_old_work(rect, plugin, monkeypatch):
    obj, node, _ = plugin
    entered, release = threading.Event(), threading.Event()
    real_rectify = rect.rectify

    def paused(*args):
        entered.set()
        assert release.wait(3)
        return real_rectify(*args)

    monkeypatch.setattr(rect, "rectify", paused)
    obj.start()
    obj._on_frame(message(encode(scene()[1])))
    assert entered.wait(2)
    obj._on_frame(message(b"x" * (rect.MAX_JPEG_BYTES + 1), 124))
    release.set()
    time.sleep(0.1)
    assert obj.info()["status"] == "invalid_image"
    assert not node.images


def test_tool_metadata_config_and_bundle_wiring(rect, monkeypatch):
    obj = rect.PlanarRectifyPlugin({}, "test", Executor())
    tool = obj.get_tool()
    validate({"action": "start"}, tool["inputSchema"])
    validate(rect.DEFAULTS, tool["configSchema"])
    assert tool["type"] == "processor" and tool["multiInstance"] is False
    assert tool["topic_in"] == [{"format": "image/jpeg", "desc": "Planar scene"}]
    assert obj.dispatch("unknown", {}) is None
    json.dumps(obj.dispatch("info", {}), allow_nan=False)
    cfg = yaml.safe_load((BUMI / "config.yaml").read_text(encoding="utf-8"))
    assert cfg["plugins"]["planar_rectify"]["enabled"] is False
    meta = yaml.safe_load((BUMI / "driver.yaml").read_text(encoding="utf-8"))
    assert {"name": "planar_rectify", "type": "processor"} in meta["cards"]
    assert "COPY planar_rectify.py /work/planar_rectify.py" in (BUMI / "Dockerfile").read_text()

    rclpy, executors = types.ModuleType("rclpy"), types.ModuleType("rclpy.executors")
    rclpy.executors = executors
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.executors", executors)
    module = types.ModuleType("bumi_planar_bundle_test")
    module.__file__ = str(BUMI / "main.py")
    exec(compile((BUMI / "main.py").read_text(encoding="utf-8"), module.__file__, "exec"), module.__dict__)
    config = {"plugins": {"planar_rectify": {"enabled": True}}}
    bundle = module.BumiDeviceBundle(config, "test", Executor(), None, None)
    assert [t["name"] for t in bundle.get_all_tools()] == ["planar_rectify"]
    assert bundle.dispatch("planar_rectify", {"action": "info"})["state"] == "idle"
    config["plugins"]["planar_rectify"]["mode"] = "invalid"
    assert module.BumiDeviceBundle(config, "test", Executor(), None, None).get_all_tools() == []


def test_ros_node_contract_serialization_qos_and_shutdown(rect, monkeypatch):
    class String:
        def __init__(self, data):
            self.data = data

    class Compressed:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=types.SimpleNamespace(sec=0, nanosec=0))

    class RosNode(Node):
        def __init__(self, name):
            super().__init__()
            self.timers = []
            self.publishers = {}

        def create_publisher(self, cls, topic, qos):
            assert qos["depth"] == 1 and qos["reliability"] == "best_effort"
            publisher = types.SimpleNamespace(messages=[])
            publisher.publish = publisher.messages.append
            self.publishers[topic] = publisher
            return publisher

        def create_subscription(self, cls, topic, callback, qos):
            self.subscription = (cls, topic, callback)

        def create_timer(self, period, callback):
            self.timers.append(callback)

    names = ("rclpy", "rclpy.node", "rclpy.qos", "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg")
    stubs = {name: types.ModuleType(name) for name in names}
    stubs["rclpy.node"].Node = RosNode
    qos = stubs["rclpy.qos"]
    qos.QoSProfile = lambda **kwargs: kwargs
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best_effort")
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE="volatile")
    stubs["sensor_msgs.msg"].CompressedImage = Compressed
    stubs["std_msgs.msg"].String = String
    for name, module in stubs.items():
        monkeypatch.setitem(sys.modules, name, module)
    obj = rect.PlanarRectifyPlugin(manual_config(rect), "test", Executor())
    try:
        obj.start()
        node = obj._node
        assert node.subscription[1] == "/test/camera/color"
        obj._on_frame(message(encode(scene()[1])))
        wait_for(lambda: bool(node.image_pub.messages))
        node.timers[0]()
        report = json.loads(node.state_pub.messages[-1].data)
        assert report["valid"] and report["input_ros_stamp"] == {"sec": 123, "nanosec": 0}
        image = node.image_pub.messages[-1]
        assert image.header.stamp.sec == 123 and image.format == "jpeg"
        assert image.header.frame_id == "planar_rectified_pixels"
        assert rect.jpeg_size(image.data) == (220, 300)
    finally:
        obj.stop()
    assert json.loads(node.state_pub.messages[-1].data)["status"] == "idle"
    count = len(node.state_pub.messages)
    node.timers[0]()
    assert len(node.state_pub.messages) == count and node.destroyed


def test_failed_executor_registration_releases_created_node(rect, monkeypatch):
    node = Node()
    executor = Executor()
    monkeypatch.setattr(executor, "add_node", lambda node: (_ for _ in ()).throw(RuntimeError("test failure")))
    obj = rect.PlanarRectifyPlugin({}, "test", executor)
    monkeypatch.setattr(obj, "_make_node", lambda: node)
    with pytest.raises(RuntimeError, match="test failure"):
        obj.start()
    assert node.destroyed and obj._node is None and obj._thread is None


def test_publisher_failure_invalidates_result_and_still_destroys_node(rect, plugin, monkeypatch):
    obj, node, executor = plugin
    monkeypatch.setattr(node, "send_image", lambda *args: (_ for _ in ()).throw(RuntimeError("publisher failure")))
    obj.start()
    obj._on_frame(message(encode(scene()[1])))
    wait_for(lambda: obj.info()["status"] == "worker_error")
    assert obj.info()["valid"] is False and obj._latest is None
    obj.stop()
    assert not executor.nodes and node.destroyed
