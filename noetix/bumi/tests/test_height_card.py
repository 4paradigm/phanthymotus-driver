"""tests/test_height_card.py — the Bumi `height` card's wiring and refusals.

Geometry is covered in test_height_math.py; this file loads the card with fake
rclpy/message modules (the same pattern as test_mic_card.py) and asserts what a
caller sees: the bundle lifecycle, the internal depth subscription, the optional
detection binding, measure/calibrate replies, and the cases where the card must
say "no" instead of returning a number.
"""

from __future__ import annotations

import importlib.util
import json
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np
import pytest
import yaml

BUNDLE = Path(__file__).resolve().parents[1]
W, H = 640, 480
K = {"fx": 603.85986328125, "fy": 603.5826416015625,
     "cx": 324.97772216796875, "cy": 250.24855041503906}
DEPTH_K = {"width": W, "height": H, "fx": K["fx"], "fy": K["fy"],
           "ppx": K["cx"], "ppy": K["cy"], "model": "none", "coeffs": []}
POSE_TOPIC = "/robot/camera/color/poses"
POSE_DECL = {"schema": "motus.camera/1", "topic": POSE_TOPIC,
             "format": "data/json", "width": W, "height": H, "K": None}


class _Node:
    def __init__(self, name):
        self._name = name
        self.subscriptions = []
        self.published = []
        self.destroyed = False

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = types.SimpleNamespace(
            topic=topic, callback=callback, message_type=message_type)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        if subscription in self.subscriptions:
            self.subscriptions.remove(subscription)

    def create_publisher(self, message_type, topic, qos):
        return types.SimpleNamespace(
            topic=topic, publish=lambda message: self.published.append(message))

    def destroy_node(self):
        self.destroyed = True


class _Executor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)

    def remove_node(self, node):
        self.nodes.remove(node)


class _CompressedImage:
    def __init__(self, data: bytes = b""):
        self.data = data


class _String:
    def __init__(self):
        self.data = ""


class _FakeCamera:
    def __init__(self, stream=None):
        self.stream = dict(stream or {})

    def depth_intrinsics(self):
        return dict(self.stream)


def _load_module(monkeypatch):
    """Load height_card.py with the ROS modules it imports faked."""
    qos = types.ModuleType("rclpy.qos")
    qos.QoSProfile = lambda **kwargs: object()
    qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = _Node
    rclpy = types.ModuleType("rclpy")
    rclpy.node = node_mod
    rclpy.qos = qos
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msg.CompressedImage = _CompressedImage
    std_msgs = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.String = _String
    for name, module in {
        "rclpy": rclpy, "rclpy.node": node_mod, "rclpy.qos": qos,
        "sensor_msgs": sensor_msgs, "sensor_msgs.msg": sensor_msg,
        "std_msgs": std_msgs, "std_msgs.msg": std_msg,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.syspath_prepend(str(BUNDLE))
    path = BUNDLE / "height_card.py"
    spec = importlib.util.spec_from_file_location("bumi_height_card_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _plugin(monkeypatch, cfg=None, stream=DEPTH_K):
    module = _load_module(monkeypatch)
    executor = _Executor()
    plugin = module.HeightPlugin(cfg or {}, "robot", executor,
                                 _FakeCamera(stream))
    plugin.start()
    return plugin, executor


def _node(plugin):
    return plugin._node


def _floor_depth(camera_height_m: float = 1.0, max_range_m: float = 6.0):
    depth = np.full((H, W), np.nan, dtype=np.float32)
    for v in range(int(K["cy"]) + 1, H):
        z = camera_height_m * K["fy"] / (v - K["cy"])
        if 0.3 <= z <= max_range_m:
            depth[v, :] = z
    return depth


def _person_frame(person_height_m=1.70, distance_m=3.0,
                  camera_height_m=1.0):
    depth = _floor_depth(camera_height_m)
    v_top = int(round(K["cy"] + K["fy"] *
                      (camera_height_m - person_height_m) / distance_m))
    v_bottom = int(round(K["cy"] + K["fy"] * camera_height_m / distance_m))
    bbox = [200, v_top, 440, min(H - 1, v_bottom)]
    depth[bbox[1]:bbox[3], bbox[0]:bbox[2]] = distance_m
    return depth, bbox


def _feed_depth(node, depths, interval=0.01):
    payloads = [_encode(depth) for depth in depths]

    def run():
        for payload in payloads:
            node._on_depth(_CompressedImage(payload))
            time.sleep(interval)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def _encode(depth_m):
    import zlib
    millimetres = np.nan_to_num(depth_m * 1000.0, nan=0.0)
    millimetres = np.clip(np.rint(millimetres), 0, 65535).astype("<u2")
    return zlib.compress(millimetres.tobytes(), 1)


def _set_pose(node, bbox, track_id=1):
    payload = {"timestamp": time.time(), "count": 1,
               "persons": [{"id": track_id, "bbox": [float(v) for v in bbox]}]}
    message = _String()
    message.data = json.dumps(payload)
    node._on_detection(message)


def _start_with_detection(plugin):
    return plugin.dispatch("start", {
        "input_topics": [POSE_TOPIC],
        "camera_info": {POSE_TOPIC: POSE_DECL},
    })


# ── tool contract and bundle lifecycle ─────────────────────────────────────

def test_tool_is_a_driver_measurement_card(monkeypatch):
    module = _load_module(monkeypatch)
    tool = module.TOOLS[0]
    assert tool["name"] == "height"
    assert tool["type"] == "processor"
    assert tool["multiInstance"] is False
    enum = tool["inputSchema"]["properties"]["action"]["enum"]
    assert "info" in enum and "measure" in enum and "calibrate" in enum
    assert set(tool["inputSchema"]["x-action-params"]) == {"measure", "calibrate"}
    assert [port["format"] for port in tool["topic_in"]] == ["data/json"]
    assert tool["topic_out"][0]["format"] == "data/json"


def test_bundle_start_subscribes_to_the_internal_depth_topic(monkeypatch):
    plugin, executor = _plugin(monkeypatch)
    node = _node(plugin)
    assert executor.nodes == [node]
    assert node.subscriptions[0].topic == "/robot/camera/depth"
    info = plugin.dispatch("info", {})
    assert info["state"] == "running"
    assert info["inputs"]["depth"] == "/robot/camera/depth"
    assert info["topic_out"][0]["topic"] == "/robot/height/last"


def test_camera_runtime_intrinsics_are_preferred(monkeypatch):
    plugin, _ = _plugin(monkeypatch)
    info = plugin.dispatch("info", {})
    assert info["intrinsics"]["source"] == "camera-runtime"
    assert info["intrinsics"]["fx"] == pytest.approx(K["fx"])


def test_missing_intrinsics_are_a_reason_not_a_fallback_number(monkeypatch):
    plugin, _ = _plugin(monkeypatch, stream={})
    _feed_depth(_node(plugin), [_floor_depth()] * 3)
    result = plugin.dispatch("measure", {"bbox": [0.3, 0.2, 0.7, 0.9]})
    assert result["state"] == "error"
    assert result["reason"] == "no_camera_info"


def test_manual_intrinsics_unblock_a_camera_that_reports_none(monkeypatch):
    plugin, _ = _plugin(monkeypatch, cfg={
        "fx": 603.86, "fy": 603.58, "cx": 324.98, "cy": 250.25}, stream={})
    assert plugin.dispatch("info", {})["intrinsics"]["source"] == "config-manual"


def test_detection_binding_is_reported_and_rescaled(monkeypatch):
    plugin, _ = _plugin(monkeypatch)
    result = _start_with_detection(plugin)
    assert result["state"] == "running"
    assert result["inputs"]["detection"] == POSE_TOPIC
    info = plugin.dispatch("info", {})
    assert info["topic_in"] == [{"topic": POSE_TOPIC, "format": "data/json"}]


# ── measure ────────────────────────────────────────────────────────────────

def test_measure_a_person_from_pose_boxes_and_the_floor_plane(monkeypatch):
    plugin, _ = _plugin(monkeypatch, cfg={"frames": 2})
    assert _start_with_detection(plugin)["state"] == "running"
    depth, bbox = _person_frame(1.70)
    _set_pose(_node(plugin), bbox)
    _feed_depth(_node(plugin), [depth] * 4)
    result = plugin.dispatch("measure", {})
    assert result["state"] == "ok", result
    assert result["height_m"] == pytest.approx(1.70, abs=0.03)
    assert result["mode"] == "floor"
    assert result["target"]["kind"] == "person"
    assert result["height_definition"] == "floor_to_top_of_head"


def test_measure_with_a_manual_bbox_and_a_calibrated_mount(monkeypatch):
    plugin, _ = _plugin(monkeypatch, cfg={"frames": 2, "camera_height_m": 1.0})
    depth, bbox = _person_frame(1.70)
    normalized = [bbox[0] / W, bbox[1] / H, bbox[2] / W, bbox[3] / H]
    blank = np.full((H, W), 3.0, dtype=np.float32)
    _feed_depth(_node(plugin), [blank] * 4)
    result = plugin.dispatch("measure", {"bbox": normalized})
    assert result["state"] == "ok", result
    assert result["mode"] == "mount"
    assert result["height_m"] == pytest.approx(1.70, abs=0.03)


def test_measure_without_detection_or_bbox_is_a_reason(monkeypatch):
    plugin, _ = _plugin(monkeypatch, cfg={"frames": 2})
    _feed_depth(_node(plugin), [_floor_depth()] * 3)
    result = plugin.dispatch("measure", {})
    assert result["state"] == "error"
    assert result["reason"] == "no_detection"


def test_a_clipped_detection_is_refused_before_any_number(monkeypatch):
    plugin, _ = _plugin(monkeypatch, cfg={"frames": 2})
    _start_with_detection(plugin)
    _set_pose(_node(plugin), [100, 0, 300, 470])
    _feed_depth(_node(plugin), [_floor_depth()] * 3)
    result = plugin.dispatch("measure", {})
    assert result["state"] == "error"
    assert result["reason"] == "target_clipped"


def test_a_stale_detection_is_refused(monkeypatch):
    plugin, _ = _plugin(monkeypatch, cfg={"frames": 2})
    _start_with_detection(plugin)
    _set_pose(_node(plugin), [180, 100, 440, 470])
    _node(plugin)._detection_arrival -= 10.0
    _feed_depth(_node(plugin), [_floor_depth()] * 3)
    result = plugin.dispatch("measure", {})
    assert result["state"] == "error"
    assert result["reason"] == "stale_detection"


def test_frames_that_disagree_are_unstable(monkeypatch):
    plugin, _ = _plugin(monkeypatch, cfg={"frames": 2})
    _start_with_detection(plugin)
    depth, bbox = _person_frame(1.70)
    _set_pose(_node(plugin), bbox)
    shifted = depth.copy()
    shifted[int(bbox[1]):int(bbox[3]), int(bbox[0]):int(bbox[2])] = 2.8
    _feed_depth(_node(plugin), [depth, shifted, depth, shifted])
    result = plugin.dispatch("measure", {})
    assert result["state"] == "error"
    assert result["reason"] == "unstable"


# ── calibrate / stop / config ──────────────────────────────────────────────

def test_calibrate_reads_the_floor_height_and_pitch(monkeypatch):
    plugin, _ = _plugin(monkeypatch, cfg={"calibration_frames": 3})
    _feed_depth(_node(plugin), [_floor_depth(1.0)] * 6)
    result = plugin.dispatch("calibrate", {})
    assert result["state"] == "ok", result
    assert result["camera_height_m"] == pytest.approx(1.0, abs=0.02)
    assert result["camera_pitch_deg"] == pytest.approx(0.0, abs=1.0)
    assert result["suggested_config"]["camera_height_m"] == \
        pytest.approx(1.0, abs=0.02)


def test_stop_retires_the_node_and_info_returns_idle(monkeypatch):
    plugin, executor = _plugin(monkeypatch)
    assert plugin.dispatch("stop", {})["state"] == "idle"
    assert executor.nodes == []
    info = plugin.dispatch("info", {})
    assert info["state"] == "idle"
    assert "topic_in" not in info


def test_info_is_answerable_before_start(monkeypatch):
    module = _load_module(monkeypatch)
    plugin = module.HeightPlugin({}, "robot", _Executor(), _FakeCamera({}))
    info = plugin.dispatch("info", {})
    assert info["state"] == "idle"
    assert info["config"]["frames"] == 5


def test_a_single_frame_is_refused(monkeypatch):
    module = _load_module(monkeypatch)
    with pytest.raises(ValueError, match="at least 2|frames"):
        module.HeightPlugin({"frames": 1}, "robot", _Executor(), _FakeCamera())


# ── bundle registration / manifests ────────────────────────────────────────

def test_camera_depth_intrinsics_accessor(monkeypatch):
    from test_mic_card import _load_device

    device, _ = _load_device(monkeypatch)
    camera = device.CameraPlugin({}, "robot", _Executor())
    assert camera.depth_intrinsics() == {}
    camera._intrinsics = {"depth": dict(DEPTH_K)}
    assert camera.depth_intrinsics()["fx"] == pytest.approx(K["fx"])


def test_main_bundle_registers_height_and_manifest_lists_it(monkeypatch):
    from test_mic_card import _load_device

    device, _ = _load_device(monkeypatch)
    height_module = _load_module(monkeypatch)
    monkeypatch.setitem(sys.modules, "device", device)
    monkeypatch.setitem(sys.modules, "height_card", height_module)
    executors = types.ModuleType("rclpy.executors")
    executors.MultiThreadedExecutor = object
    monkeypatch.setitem(sys.modules, "rclpy.executors", executors)

    path = BUNDLE / "main.py"
    spec = importlib.util.spec_from_file_location("bumi_height_bundle_test", path)
    main = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(main)

    cfg = {"plugins": {"camera": {"enabled": True},
                       "height": {"enabled": True}}}
    bundle = main.BumiDeviceBundle(cfg, "robot", _Executor(), None, None)
    tools = {tool["name"] for tool in bundle.get_all_tools()}
    assert {"camera", "depth", "height"} <= tools

    manifest = yaml.safe_load((BUNDLE / "driver.yaml").read_text())
    assert "height" in {card["name"] for card in manifest["cards"]}
    config = yaml.safe_load((BUNDLE / "config.yaml").read_text())
    assert config["plugins"]["height"]["enabled"] is False
