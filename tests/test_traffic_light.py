"""Real OpenCV images; source provenance, independent targets and ROS/MCP fakes."""
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import types

import cv2
import numpy as np
import pytest

from common.traffic_light import classify_color
from common.traffic_light.processor import TrafficLightProcessor
from common.traffic_light.plugin import TrafficLightPlugin, topics_for

ROOT = Path(__file__).resolve().parents[1]
COLORS = {"red": (0, 0, 255), "green": (0, 255, 0), "yellow": (0, 255, 255)}


def lamp(color="red", symbol="circle", *, size=160):
    image = np.zeros((size, size, 3), np.uint8)
    if symbol == "circle":
        cv2.circle(image, (size//2, size//2), size//5, COLORS[color], -1)
    else:
        # Independently drawn arrow with a head and shaft, scaled to this housing.
        points = np.array([[0, -32], [30, -3], [11, -3], [11, 32],
                           [-11, 32], [-11, -3], [-30, -3]], np.float32)
        if symbol == "arrow_left":
            points = points @ np.array([[0, -1], [1, 0]], np.float32)
        elif symbol == "arrow_right":
            points = points @ np.array([[0, 1], [-1, 0]], np.float32)
        points = np.rint(points*(size/160) + size/2).astype(np.int32)
        cv2.fillPoly(image, [points], COLORS[color])
    return image


@pytest.mark.parametrize("color", list(COLORS))
def test_colors_are_emitting_regions_not_housing_means(color):
    image = lamp(color)
    result = classify_color(image)
    assert result.color == color and result.color_confidence > .8
    assert len(result.bulbs) == 1
    assert result.symbol == "circle"


@pytest.mark.parametrize("symbol", ["circle", "arrow_left", "arrow_straight", "arrow_right"])
def test_actual_pattern_distinguishes_symbols_independently_of_lamp_position(symbol):
    image = lamp("green", symbol)
    result = classify_color(image)
    assert result.symbol == symbol and result.symbol_confidence >= .8
    # Translate the housing pixels; the direction must stay the same.
    shifted = cv2.warpAffine(image, np.float32([[1, 0, 12], [0, 1, -8]]), (160, 160))
    assert classify_color(shifted).symbol == symbol


@pytest.mark.parametrize("kind", ["off", "black", "white", "dark_red", "weak_saturation"])
def test_off_and_reflections_are_unknown(kind):
    image = np.zeros((160, 160, 3), np.uint8)
    color = {"off": (20, 20, 20), "black": (0, 0, 0), "white": (255, 255, 255),
             "dark_red": (0, 0, 40), "weak_saturation": (240, 240, 255)}[kind]
    cv2.circle(image, (80, 80), 30, color, -1)
    assert classify_color(image).color == "unknown"


@pytest.mark.parametrize("hue", [0, 2, 178, 179])
def test_red_hue_wrap_is_preserved(hue):
    hsv = np.uint8([[[hue, 255, 255]]])
    bgr = tuple(map(int, cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]))
    image = np.zeros((160, 160, 3), np.uint8)
    cv2.circle(image, (80, 80), 25, bgr, -1)
    assert classify_color(image).color == "red"


def test_colored_background_and_white_highlight_do_not_choose_the_light_color():
    image = lamp("red")
    image[:15] = COLORS["green"]
    cv2.circle(image, (80, 80), 6, (255, 255, 255), -1)
    result = classify_color(image)
    assert result.color == "red" and all(b["color"] == "red" for b in result.bulbs)


@pytest.mark.parametrize("box", [[-20, -30, 180, 190], [160, 160, 200, 200], [30, 30, 10, 10], [0, 0, float("inf"), 20]])
def test_outside_or_invalid_boxes_are_bounded(box):
    result = classify_color(lamp(), box)
    assert result.color in ("red", "unknown")
    assert isinstance(result.to_dict(), dict)


def test_simultaneous_colors_are_preserved_as_independent_bulbs():
    image = np.zeros((240, 120, 3), np.uint8)
    cv2.circle(image, (60, 55), 20, COLORS["red"], -1)
    cv2.circle(image, (60, 180), 20, COLORS["green"], -1)
    result = classify_color(image)
    assert result.color == "unknown" and result.reason == "ambiguous_colors"
    assert [b["color"] for b in result.bulbs] == ["red", "green"]


@pytest.mark.parametrize("kind", ["low_resolution", "blur", "incomplete_arrow", "clipped", "rectangle"])
def test_incomplete_weak_or_blurred_shapes_are_unknown(kind):
    image = lamp("red", "arrow_left")
    if kind == "low_resolution":
        image = cv2.resize(image, (24, 24), interpolation=cv2.INTER_AREA)
    elif kind == "blur":
        image = cv2.GaussianBlur(image, (31, 31), 6)
    elif kind == "incomplete_arrow":
        image[:, :75] = 0
    elif kind == "clipped":
        image = image[:, 70:]
    else:
        image[:] = 0
        cv2.rectangle(image, (65, 45), (90, 110), COLORS["red"], -1)
    result = classify_color(image)
    assert result.symbol == "unknown"
    if result.color != "unknown":
        assert result.color_confidence > 0 and result.symbol_confidence == 0


class Clock:
    value = 1000.

    def now(self):
        return self.value

    def ns(self):
        return round(self.value*10**9)

    def advance(self, seconds=.2):
        self.value += seconds


@pytest.fixture
def engine():
    clock = Clock()
    return TrafficLightProcessor(clock=clock.now, ros_clock=clock.ns, wall_clock=clock.now), clock


def jpeg(image):
    ok, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    assert ok
    return data.tobytes()


def stamp(value):
    return {"sec": value//10**9, "nanosec": value % 10**9}


def detection(clock, boxes=None, *, source=None, stream="camera_epoch", seq=1):
    boxes = boxes if boxes is not None else [[0, 0, 160, 160]]
    return {"source_mode": "stream", "stream_id": stream, "frame_seq": seq,
            "source_stamp": stamp(clock.ns() if source is None else source),
            "source_clock_status": "ok", "temporal_eligible": True,
            "objects": [{"name": "traffic light", "bbox": b, "confidence": .9,
                         "position": [0., 0.]} for b in boxes]}


def observe(engine, image=None, boxes=None, **kwargs):
    processor, clock = engine
    image = lamp() if image is None else image
    payload = detection(clock, boxes, **kwargs)
    processor.image(jpeg(image), payload["source_stamp"])
    result = processor.detection(payload)
    clock.advance()
    return result


def test_three_of_five_votes_and_duplicate_frames_never_count_twice(engine):
    processor, clock = engine
    for index in range(5):
        source = clock.ns()
        result = observe(engine, seq=index)
        signal = result["signals"][0]
        assert signal["observations"] == index+1
        assert signal["color"] == ("unknown" if index < 2 else "red")
        duplicate = processor.detection(detection(clock, source=source, seq=index))
        assert duplicate["signals"][0]["observations"] == index+1
    assert signal["stability"] == "stable"


def test_current_unknown_or_changed_color_cannot_reuse_old_red(engine):
    for _ in range(3):
        observe(engine)
    changed = observe(engine, lamp("green"))["signals"][0]
    assert changed["color"] == "unknown"
    dark = observe(engine, np.zeros((160, 160, 3), np.uint8))["signals"][0]
    assert dark["color"] == "unknown" and dark["bulbs"] == []


def test_voting_is_three_of_five_not_three_repeated_reads(engine):
    image = lamp()
    dark = np.zeros_like(image)
    for pixels in (image, dark, image, dark, image):
        result = observe(engine, pixels)
    assert result["signals"][0]["color"] == "red"
    assert result["signals"][0]["observations"] == 5


@pytest.mark.parametrize("symbols", [("circle", "circle"), ("arrow_left", "arrow_straight"), ("arrow_left", "arrow_right")])
def test_two_different_targets_keep_separate_ids_colors_and_directions(engine, symbols):
    image = np.hstack((lamp("red", symbols[0]), lamp("green", symbols[1])))
    boxes = [[0, 0, 160, 160], [160, 0, 320, 160]]
    for _ in range(3):
        result = observe(engine, image, boxes)
    signals = result["signals"]
    assert len(signals) == 2 and len({s["signal_id"] for s in signals}) == 2
    assert [s["color"] for s in signals] == ["red", "green"]
    assert [s["symbol"] for s in signals] == list(symbols)


def test_occluded_target_is_unknown_without_affecting_other_target(engine):
    boxes = [[0, 0, 160, 160], [160, 0, 320, 160]]
    for _ in range(3):
        observe(engine, np.hstack((lamp(), lamp("green"))), boxes)
    result = observe(engine, np.hstack((np.zeros((160, 160, 3), np.uint8), lamp("green"))), boxes)
    assert [s["color"] for s in result["signals"]] == ["unknown", "green"]


def test_missing_target_clears_votes_and_reappearance_needs_confirmation(engine):
    processor, clock = engine
    for _ in range(3):
        first = observe(engine)
    assert observe(engine, boxes=[])["state"] == "no_signal"
    assert observe(engine)["signals"][0]["color"] == "unknown"
    clock.advance(2)
    assert processor.latest()["state"] == "stale" and not processor.tracks
    again = observe(engine)
    assert again["signals"][0]["signal_id"] != first["signals"][0]["signal_id"]


def test_unmatched_frames_and_provenance_never_produce_color(engine):
    processor, clock = engine
    processor.image(jpeg(lamp()), stamp(clock.ns()))
    payload = detection(clock, source=clock.ns()+1)
    assert processor.detection(payload)["signals"][0]["color"] == "unknown"
    payload = detection(clock)
    del payload["source_stamp"]
    payload["timestamp"] = clock.now()
    payload["objects"][0]["color_name"] = "red"
    result = processor.detection(payload)
    assert result["state"] == "waiting" and result["signals"][0]["color"] == "unknown"


def test_detection_before_image_is_matched_later_without_future_substitution(engine):
    processor, clock = engine
    payload = detection(clock)
    assert processor.detection(payload)["state"] == "waiting"
    clock.advance(.01)
    assert processor.image(jpeg(lamp()), stamp(clock.ns())) == []
    result = processor.image(jpeg(lamp()), payload["source_stamp"])
    assert len(result) == 1 and result[0]["signals"][0]["observations"] == 1


@pytest.mark.parametrize("seconds", [-2., .1])
def test_stale_and_future_source_times_are_rejected(engine, seconds):
    processor, clock = engine
    payload = detection(clock, source=clock.ns()+int(seconds*10**9))
    assert processor.image(jpeg(lamp()), payload["source_stamp"]) == []
    result = processor.detection(payload)
    assert result["state"] == "stale" and result["signals"][0]["color"] == "unknown"


def test_clock_regression_stream_restart_and_latest_staleness(engine):
    processor, clock = engine
    for _ in range(3):
        observe(engine)
    clock.advance(2)
    result = processor.latest()
    assert result["state"] == "stale" and result["signals"][0]["color"] == "unknown"
    payload = detection(clock)
    payload["source_clock_status"] = "regressed"
    assert processor.detection(payload)["signals"][0]["color"] == "unknown"
    result = observe(engine, stream="new_epoch")
    assert result["signals"][0]["observations"] == 1


def test_cache_and_pending_capacity_are_bounded(engine):
    processor, clock = engine
    processor.config = processor.config.from_dict({"cache_size": 4})
    for index in range(20):
        processor.image(jpeg(lamp()), stamp(clock.ns()))
        processor.detection(detection(clock, source=clock.ns()-1))
        clock.advance(.01)
    assert len(processor.frames) <= 4 and len(processor.pending) <= 8


def test_motion_restart_barrier_rejects_pre_start_frames(engine):
    processor, clock = engine
    processor.not_before_ns = clock.ns()+1
    result = observe(engine)
    assert result["reason"] == "before_observation_start"
    assert result["signals"][0]["color"] == "unknown"


def test_missing_bbox_confidence_and_bad_json_are_normal_results(engine):
    processor, clock = engine
    payload = detection(clock)
    processor.image(jpeg(lamp()), payload["source_stamp"])
    payload["objects"][0].pop("bbox")
    result = processor.detection(payload)
    assert result["signals"][0]["color"] == "unknown"
    assert processor.detection({"objects": None})["state"] == "error"
    assert processor.detection([])["state"] == "error"


@pytest.fixture
def ros(monkeypatch):
    """Stub only middleware, not OpenCV or the processor/HTTP implementation."""
    clock, events, created = Clock(), [], []

    class Node:
        def __init__(self, name):
            self.name, self.subscriptions, self.timers = name, [], []
            self.messages = []
            self.destroy_fails = False
            self.subscription_types = []
            self.publisher_topics = []
            created.append(self)

        def get_clock(self):
            return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=clock.ns()))

        def create_publisher(self, cls, topic, qos):
            self.publisher_topics.append((cls, topic, qos))
            return types.SimpleNamespace(publish=lambda msg: self.messages.append(json.loads(msg.data)))

        def create_subscription(self, cls, topic, callback, qos):
            assert qos.depth == 2
            self.subscriptions.append((topic, callback))
            self.subscription_types.append((cls, topic, qos))

        def create_timer(self, interval, callback):
            self.timers.append(callback)

        def destroy_node(self):
            events.append("destroy")
            return False if self.destroy_fails else None

    class Executor:
        def __init__(self):
            self.nodes = []

        def add_node(self, node):
            self.nodes.append(node)

        def remove_node(self, node):
            events.append("remove")
            self.nodes.remove(node)

        def get_nodes(self):
            return self.nodes

    class String:
        def __init__(self, data=""):
            self.data = data

    modules = {name: types.ModuleType(name) for name in (
        "rclpy", "rclpy.node", "rclpy.qos", "rclpy.executors", "sensor_msgs", "sensor_msgs.msg",
        "std_msgs", "std_msgs.msg")}
    modules["rclpy.node"].Node = Node
    modules["rclpy.executors"].MultiThreadedExecutor = Executor
    qos = modules["rclpy.qos"]
    qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    for name in ("ReliabilityPolicy", "HistoryPolicy", "DurabilityPolicy"):
        setattr(qos, name, types.SimpleNamespace(BEST_EFFORT=1, KEEP_LAST=1, VOLATILE=1))
    modules["sensor_msgs.msg"].CompressedImage = type("CompressedImage", (), {})
    modules["std_msgs.msg"].String = String
    modules["rclpy"].executors = modules["rclpy.executors"]
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    return types.SimpleNamespace(clock=clock, executor=Executor(), events=events,
                                 created=created, String=String)


def load_bumi(monkeypatch):
    """Import the real MCP entry point, never its hardware-initialising main()."""
    import common
    logsafe = types.ModuleType("common.logsafe")
    logsafe.install = lambda **kwargs: None
    monkeypatch.setattr(common, "logsafe", logsafe, raising=False)
    monkeypatch.setitem(sys.modules, "common.logsafe", logsafe)
    # Bundle constructors adjust these camera flags; restore their prior values
    # with the rest of the fixture instead of affecting unrelated test modules.
    for key in ("BUMI_VISUAL_ODOMETRY", "BUMI_VISUAL_ODOMETRY_MAX_FPS"):
        monkeypatch.setenv(key, os.environ.get(key, ""))
    if "rclpy.executors" not in sys.modules:
        monkeypatch.setitem(sys.modules, "rclpy.executors", types.ModuleType("rclpy.executors"))
    spec = importlib.util.spec_from_file_location("bumi_traffic_light_test", ROOT/"noetix"/"bumi"/"main.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def rpc_for(module, bundle):
    """Use the production handler with in-memory HTTP streams, no server/socket."""
    module._bundle = bundle
    handler_type = module.make_handler()

    def rpc(method, params=None):
        request = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method,
                              "params": params or {}}).encode()
        handler = object.__new__(handler_type)
        handler.path, handler.headers, handler.rfile = "/mcp", {"Content-Length": str(len(request))}, io.BytesIO(request)
        captured = []
        handler._send = lambda status, body: captured.append((status, json.loads(body)))
        handler.do_POST()
        assert captured[0][0] == 200
        return captured[0][1]

    return rpc


def test_card_lifecycle_binding_and_domain_actions(ros):
    plugin = TrafficLightPlugin({}, "", ros.executor)
    assert plugin.dispatch("latest", {})["state"] == "waiting"
    args = {"instance_id": "card1", "input_topic": "/another_ns/camera/color/objects"}
    started = plugin.dispatch("start", args)
    assert started["rgb_topic"] == "/another_ns/camera/color"
    assert started["topic_out"][0]["topic"] == args["input_topic"]+"/traffic_light"
    assert plugin.dispatch("start", args)["state"] == "running" and len(ros.executor.nodes) == 1
    runtime = plugin._instances["card1"]
    assert len(runtime.node.subscriptions) == 2
    assert plugin.dispatch("info", {"instance_id": "card1"})["has_synchronized_frame"] is False
    assert plugin.dispatch("finish_observation", {"instance_id": "card1"})["state"] == "idle"
    assert ros.events == ["remove", "destroy"] and not ros.executor.nodes
    assert plugin.dispatch("stop", {"instance_id": "card1"})["state"] == "idle"
    ros.clock.advance()
    assert plugin.dispatch("observe", {"instance_id": "card1"})["state"] == "running"
    assert not plugin._instances["card1"].engine.frames
    assert not plugin._instances["card1"].engine.tracks
    assert plugin.shutdown()["state"] == "idle"


def test_card_input_errors_recover_and_late_callbacks_after_stop_do_nothing(ros):
    plugin = TrafficLightPlugin({}, "", ros.executor)
    plugin.dispatch("start", {"input_topic": "/r/camera/color/objects", "instance_id": "c"})
    runtime = plugin._instances["c"]
    runtime.detection(ros.String("not json"))
    assert plugin.dispatch("latest", {"instance_id": "c"})["state"] == "error"
    payload = detection(ros.clock, boxes=[])
    runtime.image(types.SimpleNamespace(data=jpeg(lamp()), header=types.SimpleNamespace(
        stamp=types.SimpleNamespace(**payload["source_stamp"]))))
    runtime.detection(ros.String(json.dumps(payload)))
    assert plugin.dispatch("latest", {"instance_id": "c"})["state"] == "no_signal"
    before = len(runtime.node.messages)
    plugin.dispatch("stop", {"instance_id": "c"})
    runtime.detection(ros.String(json.dumps(payload)))
    assert len(runtime.node.messages) == before


def test_multi_instance_isolation_and_failed_disposal_remains_retryable(ros):
    plugin = TrafficLightPlugin({}, "", ros.executor)
    for key in ("a", "b"):
        plugin.dispatch("start", {"instance_id": key, "input_topic": f"/{key}/camera/color/objects"})
    assert plugin.dispatch("latest", {})["state"] == "error"
    runtime = plugin._instances["a"]
    runtime.node.destroy_fails = True
    assert plugin.dispatch("stop", {"instance_id": "a"})["state"] == "error"
    assert "a" in plugin._instances and not runtime.running
    assert plugin.dispatch("start", {"instance_id": "a", "input_topic": runtime.topic})["state"] == "error"
    runtime.node.destroy_fails = False
    assert plugin.dispatch("stop", {"instance_id": "a"})["state"] == "idle"
    assert "b" in plugin._instances
    plugin.shutdown()


@pytest.mark.parametrize("topic", ["/objects", "relative/objects", "/r/camera/color", "/r//objects", None])
def test_invalid_topic_cannot_create_subscriptions(ros, topic):
    result = TrafficLightPlugin({}, "", ros.executor).dispatch("start", {"input_topic": topic})
    assert result["state"] == "error" and not ros.executor.nodes


def test_real_bumi_mcp_handler_registers_and_calls_processor_without_robot(ros, monkeypatch):
    module = load_bumi(monkeypatch)
    bundle = module.BumiDeviceBundle({"plugins": {"traffic_light": {"enabled": True}}},
                                     "r", ros.executor, None, None)
    rpc = rpc_for(module, bundle)
    tools = rpc("tools/list")["result"]["tools"]
    assert [(t["name"], t["type"]) for t in tools] == [("traffic_light", "processor")]
    for action in ("start", "latest", "info", "stop"):
        response = rpc("tools/call", {"name": "traffic_light", "arguments": {
            "action": action, "instance_id": "test", "input_topic": "/r/camera/color/objects"}})
        result = json.loads(response["result"]["content"][0]["text"])
        assert result["state"] in ("running", "waiting", "idle")
    assert not ros.executor.nodes


def test_configuration_is_checked_and_requires_stopped_instance(ros):
    plugin = TrafficLightPlugin({}, "", ros.executor)
    assert plugin.dispatch("config", {"cache_size": -1})["state"] == "error"
    assert plugin.dispatch("config", {"cache_size": 4})["state"] == "idle"
    plugin.dispatch("start", {"instance_id": "c", "input_topic": "/r/camera/objects"})
    assert plugin.dispatch("config", {"instance_id": "c", "max_age_s": .2})["state"] == "error"
    plugin.shutdown()


def test_conflicting_images_same_stamp_and_retired_stream_cannot_be_evidence(engine):
    processor, clock = engine
    original = detection(clock)
    processor.image(jpeg(lamp()), original["source_stamp"])
    processor.image(jpeg(lamp("green")), original["source_stamp"])
    assert processor.detection(original)["signals"][0]["color"] == "unknown"
    clock.advance()
    observe(engine, stream="one")
    observe(engine, stream="two")
    result = observe(engine, stream="one")
    assert result["reason"] == "retired_stream" and result["signals"][0]["color"] == "unknown"


def test_start_failure_retains_cleanup_handle_until_explicit_stop(ros, monkeypatch):
    from rclpy.node import Node
    original = Node.create_subscription
    attempts = []

    def fail(node, *args, **kwargs):
        attempts.append(node)
        node.destroy_fails = True
        raise RuntimeError("subscription creation failed")

    monkeypatch.setattr(Node, "create_subscription", fail)
    plugin = TrafficLightPlugin({}, "", ros.executor)
    result = plugin.dispatch("start", {"input_topic": "/r/camera/objects", "instance_id": "c"})
    assert result["state"] == "error"
    assert "c" in plugin._instances and not plugin._instances["c"].running
    assert plugin.dispatch("info", {"instance_id": "c"})["error"]
    attempts[0].destroy_fails = False
    assert plugin.dispatch("stop", {"instance_id": "c"})["state"] == "idle"
    monkeypatch.setattr(Node, "create_subscription", original)
    assert plugin.dispatch("observe", {"instance_id": "c"})["state"] == "running"
    plugin.shutdown()


def test_observe_requires_preexisting_canvas_binding_and_config_toggle(ros, monkeypatch):
    plugin = TrafficLightPlugin({}, "", ros.executor)
    assert plugin.dispatch("observe", {})["state"] == "error"
    module = load_bumi(monkeypatch)
    disabled = module.BumiDeviceBundle({"plugins": {"traffic_light": {"enabled": False}}},
                                       "r", ros.executor, None, None)
    assert not disabled._plugins
    enabled = module.BumiDeviceBundle({"plugins": {"traffic_light": {"enabled": True}}},
                                      "r", ros.executor, None, None)
    assert [p.PREFIX for p in enabled._plugins] == ["traffic_light"]


def test_simultaneous_lamps_each_acquire_their_own_votes(engine):
    image = np.zeros((240, 160, 3), np.uint8)
    cv2.circle(image, (80, 50), 22, COLORS["red"], -1)
    cv2.circle(image, (80, 185), 22, COLORS["green"], -1)
    for _ in range(3):
        result = observe(engine, image, [[0, 0, 160, 240]])
    signal = result["signals"][0]
    assert signal["color"] == "unknown"
    assert [b["color"] for b in signal["bulbs"]] == ["red", "green"]
    assert all(b["stability"] == "stable" for b in signal["bulbs"])


def test_cache_also_has_a_byte_budget_and_oversized_jpeg_is_not_decoded(engine, monkeypatch):
    processor, clock = engine
    processor.config = processor.config.from_dict({"max_cache_bytes": 1048576})
    for _ in range(3):
        processor.image(b"x"*700000, stamp(clock.ns()))
        clock.advance(.01)
    assert processor.info()["cache_bytes"] <= 1048576
    # A tiny file claiming huge dimensions must be rejected before OpenCV allocates.
    header = b"\xff\xd8\xff\xc0\x00\x08\x08\x7f\xff\x7f\xff\x01"
    payload = detection(clock)
    processor.image(header, payload["source_stamp"])
    monkeypatch.setattr(cv2, "imdecode", lambda *args: pytest.fail("must check SOF before decoding"))
    assert processor.detection(payload)["state"] == "error"


def test_result_expiring_during_processing_is_not_reported_as_fresh(engine, monkeypatch):
    from common.traffic_light import processor as implementation
    processor, clock = engine
    analyze = implementation.classify_color

    def delayed(*args):
        result = analyze(*args)
        clock.advance(1.1)
        return result

    monkeypatch.setattr(implementation, "classify_color", delayed)
    result = observe(engine)
    assert result["state"] == "stale" and result["signals"][0]["color"] == "unknown"


def test_interrupt_hooks_stop_every_observation_and_keep_bindings(ros):
    plugin = TrafficLightPlugin({}, "", ros.executor)
    for key in ("a", "b"):
        plugin.dispatch("start", {"instance_id": key, "input_topic": f"/{key}/color/objects"})
    hooks = plugin.get_tools()[0]["inputSchema"]["x-hooks"]
    assert hooks["on_interrupt_all"]["action"] == "stop"
    assert plugin.dispatch(hooks["on_interrupt_all"]["action"], {})["state"] == "idle"
    assert not ros.executor.nodes and not plugin._instances
    assert plugin.dispatch("info", {"instance_id": "a"})["rgb_topic"] == "/a/color"
    assert plugin.dispatch("observe", {"instance_id": "a"})["state"] == "running"
    plugin.shutdown()


def test_conflicting_stamp_invalidates_an_already_published_observation(engine):
    processor, clock = engine
    for _ in range(2):
        observe(engine)
    source = clock.ns()
    result = observe(engine)
    assert result["signals"][0]["color"] == "red"
    invalidated = processor.image(jpeg(lamp("green")), stamp(source))
    assert invalidated[0]["state"] == "waiting"
    assert processor.latest()["signals"] == []
    assert processor.info()["has_synchronized_frame"] is False
