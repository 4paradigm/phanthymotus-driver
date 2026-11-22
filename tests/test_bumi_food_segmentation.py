"""ROS-free integration tests for the Bumi Food Segmentation processor card."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from noetix.bumi.food_segmentation import (
    FoodSegmentationPlugin,
    SCHEMA_VERSION,
    _OpenCvCodec,
)
from noetix.bumi.food_segmentation_contract import (
    COCO_FOOD_CLASSES,
    RawInstance,
    decode_binary_rle,
)


ROOT = Path(__file__).resolve().parents[1]
BUMI = ROOT / "noetix" / "bumi"


class FakeStamp:
    sec = 123
    nanosec = 456


class FakeHeader:
    def __init__(self):
        self.stamp = FakeStamp()
        self.frame_id = "camera_color"


class FakeCompressedImage:
    def __init__(self, data=b"", fmt="jpeg"):
        self.header = FakeHeader()
        self.format = fmt
        self.data = data


class FakeString:
    def __init__(self):
        self.data = ""


class FakePublisher:
    def __init__(self, message_type, topic, qos):
        self.message_type = message_type
        self.topic = topic
        self.qos = qos
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    instances = []

    def __init__(self, name):
        self.name = name
        self.publishers = []
        self.subscriptions = []
        self.destroyed = False
        self.__class__.instances.append(self)

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher(message_type, topic, qos)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos, **kwargs):
        subscription = SimpleNamespace(
            message_type=message_type,
            topic=topic,
            callback=callback,
            qos=qos,
            callback_group=kwargs.get("callback_group"),
        )
        self.subscriptions.append(subscription)
        return subscription

    def destroy_node(self):
        self.destroyed = True


class FakeExecutor:
    def __init__(self):
        self.nodes = []
        self.removed = []

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        self.removed.append(node)
        if node in self.nodes:
            self.nodes.remove(node)


class FakeCodec:
    def __init__(self):
        self.encoded_images = []
        self.rectangles = []
        self.labels = []

    def decode(self, data):
        if data == b"broken":
            raise ValueError("JPEG payload could not be decoded")
        return np.zeros((5, 6, 3), dtype=np.uint8)

    def encode(self, image, quality):
        self.encoded_images.append((image.copy(), quality))
        return b"overlay-jpeg"

    def rectangle(self, _image, point1, point2, color):
        self.rectangles.append((point1, point2, color))

    def label(self, _image, text, origin, color):
        self.labels.append((text, origin, color))


def fake_bindings():
    return SimpleNamespace(
        Node=FakeNode,
        String=FakeString,
        CompressedImage=FakeCompressedImage,
        qos_profile=lambda: {
            "reliability": "best_effort",
            "history": "keep_last",
            "depth": 1,
            "durability": "volatile",
        },
        callback_group=lambda: "mutually_exclusive",
    )


class FakeRuntime:
    def __init__(self, results=None, load_error=None, predict_error=None):
        self.names = {0: "person", **COCO_FOOD_CLASSES}
        self.results = results
        self.load_error = load_error
        self.predict_error = predict_error
        self.load_calls = []
        self.predict_calls = 0
        self.last_diagnostics = {"backend": "fake"}

    def load(self, model_path, **options):
        self.load_calls.append((model_path, options))
        if self.load_error:
            raise self.load_error

    def predict(self, image):
        self.predict_calls += 1
        if self.predict_error:
            error = self.predict_error
            self.predict_error = None
            raise error
        if self.results is not None:
            return self.results
        apple = np.zeros(image.shape[:2], dtype=np.uint8)
        apple[1, 1] = apple[2, 1] = apple[2, 2] = 1  # non-rectangular L
        banana = np.zeros(image.shape[:2], dtype=np.uint8)
        banana[1:4, 4] = 1
        person = np.zeros(image.shape[:2], dtype=np.uint8)
        person[0, 0] = 1
        return [
            RawInstance(47, 0.91, [1, 1, 3, 3], apple.tolist()),
            RawInstance(46, 0.82, [4, 1, 5, 4], banana.tolist()),
            RawInstance(0, 0.99, [0, 0, 1, 1], person.tolist()),
        ]


class BlockingRuntime(FakeRuntime):
    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def predict(self, image):
        self.entered.set()
        assert self.release.wait(3), "test did not release blocked inference"
        return super().predict(image)


class BlockingLoadRuntime(FakeRuntime):
    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def load(self, model_path, **options):
        self.load_calls.append((model_path, options))
        self.entered.set()
        assert self.release.wait(3), "test did not release blocked model load"


def config(**overrides):
    return {
        "model_path": "C:/local/yolo11n-seg.pt",
        "max_rate_hz": 30,
        "stop_timeout_s": 2,
        **overrides,
    }


def make_plugin(runtime=None, codec=None, **overrides):
    runtime = runtime or FakeRuntime()
    codec = codec or FakeCodec()
    executor = FakeExecutor()
    plugin = FoodSegmentationPlugin(
        config(**overrides),
        "bumi_test",
        executor,
        runtime_factory=lambda: runtime,
        ros_bindings=fake_bindings(),
        image_codec=codec,
    )
    return plugin, runtime, codec, executor


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition was not reached before timeout")


def publishers(plugin):
    node = plugin._node
    return {publisher.topic: publisher for publisher in node.publishers}


def send(plugin, data=b"good", fmt="jpeg"):
    plugin._node.subscriptions[0].callback(FakeCompressedImage(data, fmt))


def test_tool_schema_prefix_card_type_and_topics():
    plugin, _, _, _ = make_plugin()
    tool = plugin.get_tool()
    assert plugin.PREFIX == "food_segmentation"
    assert plugin.get_tools() == [tool]
    assert tool["name"] == "food_segmentation"
    assert tool["type"] == "processor"
    assert tool["multiInstance"] is False
    assert tool["inputSchema"]["properties"]["action"]["enum"] == [
        "start", "stop", "info"]
    assert tool["inputSchema"]["x-action-params"]["start"]["params"] == [
        "input_topic"]
    assert tool["topic_in"] == [{
        "topic": "/bumi_test/camera/color", "format": "image/jpeg"}]
    assert tool["topic_out"] == [
        {"topic": "/bumi_test/food_segmentation/metadata", "format": "data/json"},
        {"topic": "/bumi_test/food_segmentation/overlay", "format": "image/jpeg"},
    ]


def test_custom_topic_derivation_and_invalid_topics():
    plugin, _, _, _ = make_plugin(
        input_topic="/{namespace}/custom/jpeg",
        metadata_topic="custom/food_json",
        overlay_topic="/custom/food_overlay",
    )
    assert plugin.get_tool()["topic_in"][0]["topic"] == "/bumi_test/custom/jpeg"
    assert plugin.get_tool()["topic_out"][0]["topic"] == "/custom/food_json"
    with pytest.raises(ValueError, match="invalid ROS 2 topic"):
        make_plugin(input_topic="/bad topic")
    with pytest.raises(ValueError, match="must be distinct"):
        make_plugin(metadata_topic="/bumi_test/camera/color")


def test_start_is_lazy_idempotent_and_creates_real_dds_shapes():
    plugin, runtime, _, executor = make_plugin()
    assert runtime.load_calls == []
    assert plugin.start() is True
    assert plugin.start() is True
    assert len(runtime.load_calls) == 1
    assert runtime.load_calls[0] == (
        "C:/local/yolo11n-seg.pt", {"imgsz": 640, "conf": 0.25})
    assert executor.nodes == [plugin._node]
    assert plugin._node.subscriptions[0].message_type is FakeCompressedImage
    assert plugin._node.subscriptions[0].callback_group == "mutually_exclusive"
    assert all(pub.qos["depth"] == 1 for pub in plugin._node.publishers)
    assert {pub.message_type for pub in plugin._node.publishers} == {
        FakeString, FakeCompressedImage}
    assert plugin.stop() is True


def test_canvas_start_can_remap_input_topic_but_not_while_running():
    plugin, runtime, _, _ = make_plugin()
    started = plugin.dispatch("start", {"input_topic": "/canvas/camera/jpeg"})
    assert started["ok"] is True
    assert plugin._node.subscriptions[0].topic == "/canvas/camera/jpeg"
    assert started["topic_in"] == [{
        "topic": "/canvas/camera/jpeg", "format": "image/jpeg"}]
    assert plugin.dispatch(
        "start", {"input_topics": ["/canvas/camera/jpeg"]})["ok"] is True

    rejected = plugin.dispatch("start", {"input_topic": "/other/camera"})
    assert rejected["ok"] is False
    assert rejected["state"] == "running"
    assert "stop before switching" in rejected["last_error"]
    assert plugin._node.subscriptions[0].topic == "/canvas/camera/jpeg"

    assert plugin.dispatch("stop", {})["ok"] is True
    restarted = plugin.dispatch("start", {"input_topics": ["other/camera"]})
    assert restarted["ok"] is True
    assert restarted["topic_in"][0]["topic"] == "/other/camera"
    assert len(runtime.load_calls) == 2  # deliberate unload/reload across stop
    plugin.stop()


def test_stop_cancels_start_that_is_still_loading_model():
    runtime = BlockingLoadRuntime()
    plugin, _, _, executor = make_plugin(runtime=runtime)
    result = []
    starter = threading.Thread(target=lambda: result.append(plugin.start()))
    starter.start()
    assert runtime.entered.wait(1)

    assert plugin.stop() is True
    assert plugin.info()["state"] == "idle"
    runtime.release.set()
    starter.join(2)

    assert result == [False]
    assert plugin.info()["state"] == "idle"
    assert plugin.info()["runtime_loaded"] is False
    assert executor.nodes == []


def test_frame_publishes_contract_json_real_rle_and_overlay():
    plugin, _, codec, _ = make_plugin()
    assert plugin.start()
    pubs = publishers(plugin)
    send(plugin)
    wait_for(lambda: plugin.info()["processed_frames"] == 1)
    metadata_message = pubs["/bumi_test/food_segmentation/metadata"].messages[0]
    overlay_message = pubs["/bumi_test/food_segmentation/overlay"].messages[0]
    payload = json.loads(metadata_message.data)

    assert payload["schema_version"] == SCHEMA_VERSION
    assert payload["frame_timestamp"] == {"sec": 123, "nanosec": 456}
    assert payload["image_size"] == [6, 5]
    assert payload["processed_instance_count"] == 2
    assert [item["class_name"] for item in payload["instances"]] == [
        "apple", "banana"]
    assert [item["mask_area_px"] for item in payload["instances"]] == [3, 3]
    assert sum(map(sum, decode_binary_rle(payload["instances"][0]["mask"]))) == 3
    assert payload["processing"]["runtime"] == {"backend": "fake"}
    assert overlay_message.data == b"overlay-jpeg"
    assert overlay_message.format == "jpeg"
    assert overlay_message.header.frame_id == "camera_color"
    changed = np.any(codec.encoded_images[0][0] != 0, axis=2)
    # Only the six true mask pixels change: bbox rectangles are not masks.
    assert int(changed.sum()) == 6
    assert len(codec.rectangles) == len(codec.labels) == 2
    assert plugin.stop()


def test_empty_detection_is_a_valid_output():
    plugin, _, codec, _ = make_plugin(runtime=FakeRuntime(results=[]))
    assert plugin.start()
    pubs = publishers(plugin)
    send(plugin)
    wait_for(lambda: plugin.info()["processed_frames"] == 1)
    payload = json.loads(
        pubs["/bumi_test/food_segmentation/metadata"].messages[0].data)
    assert payload["processed_instance_count"] == 0
    assert payload["instances"] == []
    assert payload["errors"] == []
    assert not np.any(codec.encoded_images[0][0])
    plugin.stop()


@pytest.mark.parametrize("data,fmt", [(b"", "jpeg"), (b"good", "png")])
def test_invalid_compressed_message_isolated(data, fmt):
    plugin, runtime, _, _ = make_plugin()
    assert plugin.start()
    send(plugin, data, fmt)
    wait_for(lambda: plugin.info()["decode_errors"] == 1)
    assert runtime.predict_calls == 0
    assert plugin.info()["state"] == "running"
    plugin.stop()


def test_corrupt_jpeg_and_inference_failure_do_not_kill_worker():
    runtime = FakeRuntime(predict_error=RuntimeError("fake inference failure"))
    plugin, _, _, _ = make_plugin(runtime=runtime)
    assert plugin.start()
    send(plugin, b"broken")
    wait_for(lambda: plugin.info()["decode_errors"] == 1)
    send(plugin)
    wait_for(lambda: plugin.info()["inference_errors"] == 1)
    send(plugin)
    wait_for(lambda: plugin.info()["processed_frames"] == 1)
    assert runtime.predict_calls == 2
    assert plugin.info()["state"] == "running"
    plugin.stop()


def test_missing_model_and_load_failure_report_clear_error():
    constructed = []
    plugin = FoodSegmentationPlugin(
        config(model_path=""), "bumi_test", FakeExecutor(),
        runtime_factory=lambda: constructed.append(True),
        ros_bindings=fake_bindings(), image_codec=FakeCodec())
    assert plugin.dispatch("start", {})["ok"] is False
    assert "model_path" in plugin.info()["last_error"]
    assert constructed == []

    failed, _, _, _ = make_plugin(
        runtime=FakeRuntime(load_error=RuntimeError("checkpoint rejected")))
    result = failed.dispatch("start", {})
    assert result["state"] == "error"
    assert result["ok"] is False
    assert result["runtime_loaded"] is False
    assert "checkpoint rejected" in result["last_error"]
    assert failed.dispatch("stop", {})["state"] == "idle"


def test_latest_frame_queue_drops_old_pending_frames():
    runtime = BlockingRuntime()
    plugin, _, _, _ = make_plugin(runtime=runtime)
    assert plugin.start()
    send(plugin, b"first")
    assert runtime.entered.wait(1)
    send(plugin, b"second")
    send(plugin, b"third")
    assert plugin.info()["dropped_frames"] >= 1
    runtime.release.set()
    wait_for(lambda: plugin.info()["processed_frames"] >= 2)
    assert runtime.predict_calls == 2
    plugin.stop()


def test_stop_during_inference_never_publishes_after_node_destroy():
    runtime = BlockingRuntime()
    plugin, _, _, executor = make_plugin(runtime=runtime)
    assert plugin.start()
    node = plugin._node
    pubs = publishers(plugin)
    send(plugin)
    assert runtime.entered.wait(1)
    result = {}
    stopper = threading.Thread(target=lambda: result.setdefault("ok", plugin.stop()))
    stopper.start()
    time.sleep(0.03)
    assert stopper.is_alive()
    assert node.destroyed is False
    runtime.release.set()
    stopper.join(2)
    assert result == {"ok": True}
    assert node.destroyed is True
    assert executor.nodes == []
    assert all(not publisher.messages for publisher in pubs.values())
    assert plugin.info()["state"] == "idle"
    assert plugin.stop() is True


def test_stop_timeout_defers_resource_destruction_until_inference_exits():
    runtime = BlockingRuntime()
    plugin, _, _, executor = make_plugin(
        runtime=runtime, stop_timeout_s=0.1)
    assert plugin.start()
    node = plugin._node
    send(plugin)
    assert runtime.entered.wait(1)

    assert plugin.stop() is False
    assert plugin.info()["state"] == "stopping"
    assert "retained until it exits" in plugin.info()["last_error"]
    assert node.destroyed is False
    assert executor.nodes == [node]

    runtime.release.set()
    wait_for(lambda: node.destroyed)
    assert executor.nodes == []
    assert plugin.info()["state"] == "idle"


def test_info_reports_input_interruption_without_stopping_plugin():
    plugin, _, _, _ = make_plugin(input_timeout_s=0.1)
    assert plugin.start()
    assert plugin.info()["input_state"] == "never_received"
    send(plugin)
    wait_for(lambda: plugin.info()["processed_frames"] == 1)
    assert plugin.info()["input_state"] == "fresh"
    time.sleep(0.12)
    assert plugin.info()["input_state"] == "stale"
    assert plugin.info()["state"] == "running"
    plugin.stop()


def test_dispatch_unknown_action_and_repeated_lifecycle():
    plugin, runtime, _, _ = make_plugin()
    assert plugin.dispatch("unknown", {}) is None
    assert plugin.dispatch("info", {})["state"] == "idle"
    assert plugin.dispatch("start", {})["ok"] is True
    assert plugin.dispatch("start", {})["ok"] is True
    assert plugin.dispatch("stop", {})["ok"] is True
    assert plugin.dispatch("stop", {})["ok"] is True
    assert len(runtime.load_calls) == 1


def test_manifest_config_and_docker_registration_are_safe_by_default():
    cfg = yaml.safe_load((BUMI / "config.yaml").read_text(encoding="utf-8"))
    assert cfg["plugins"]["food_segmentation"]["enabled"] is False
    assert cfg["plugins"]["food_segmentation"]["model_path"] == ""
    manifest = yaml.safe_load((BUMI / "driver.yaml").read_text(encoding="utf-8"))
    assert {"name": "food_segmentation", "type": "processor"} in manifest["cards"]
    dockerfile = (BUMI / "Dockerfile").read_text(encoding="utf-8")
    for name in (
        "food_segmentation.py", "food_segmentation_backend.py",
        "food_segmentation_contract.py", "food_segmentation_ultralytics.py",
    ):
        assert f"COPY {name} /work/{name}" in dockerfile
    assert "pip3 install" not in dockerfile[dockerfile.index("COPY food_segmentation.py"):]


def test_real_opencv_jpeg_codec_when_dependency_is_available():
    cv2 = pytest.importorskip("cv2")
    source = np.zeros((7, 9, 3), dtype=np.uint8)
    source[:, :, 1] = 180
    ok, encoded = cv2.imencode(".jpg", source)
    assert ok
    decoded = _OpenCvCodec.decode(encoded.tobytes())
    assert decoded.shape == source.shape
    assert _OpenCvCodec.encode(decoded, 85).startswith(b"\xff\xd8")
