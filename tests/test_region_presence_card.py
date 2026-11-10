"""Real Card and Bundle, substituting only ROS at its boundary."""
import ast
import importlib
import json
from pathlib import Path
import sys
import types

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.region_presence_card import RegionPresencePlugin
from test_region_presence import frame


class Node:
    def __init__(self):
        self.destroyed = False
    def destroy_node(self):
        self.destroyed = True


class Executor:
    def __init__(self):
        self.nodes = []
    def add_node(self, node):
        self.nodes.append(node)
    def remove_node(self, node):
        if node in self.nodes:
            self.nodes.remove(node)


@pytest.fixture
def running():
    executor, events, clock = Executor(), [], [0.]
    card = RegionPresencePlugin({}, "robot", executor)
    card._clock = lambda: clock[0]
    def factory(topic, generation):
        node = Node()
        node.receive = lambda data: card._on_message(types.SimpleNamespace(data=data), generation)
        node.tick = lambda: card._tick(generation)
        return node, lambda event: events.append(json.loads(json.dumps(event)))
    card._make_node = factory
    assert card.dispatch("start", {"input_topic": "/camera/objects"})["state"] == "running"
    yield card, executor, events, clock
    assert card.stop()["state"] == "idle"


def observe(executor, clock, stamp, at, count=1):
    clock[0] = at
    executor.nodes[0].receive(json.dumps(frame(stamp, count)))


def test_cold_discovery_and_invalid_start_do_not_open_ros():
    card = RegionPresencePlugin({}, "robot", Executor())
    assert card.start() is True
    assert card.check()["state"] == "idle"
    assert card.check()["present"] is None
    assert card.dispatch("start", {})["state"] == "error"
    assert card.dispatch("start", {"input_topic": "/robot/region_presence/events"})["state"] == "error"


def test_target_guidance_combines_roi_position_and_camera_quality(running):
    import cv2
    import numpy as np
    card, executor, _, clock = running
    jpeg = cv2.imencode(".jpg", np.full((100, 100), 250, np.uint8))[1].tobytes()
    card._camera = types.SimpleNamespace(is_running=lambda: True, wait_for_color_frame=lambda timeout_s:
                                         ({"data": jpeg, "received_monotonic": 0}, 7))
    card.dispatch("config", {"x_min": 0.2, "x_max": 0.8})
    receive = executor.nodes[0].receive
    receive(json.dumps(frame(1, x=-0.5, y=-0.9)))
    result = card.dispatch("target_guidance", {})
    assert card.check()["detected_count"] == 0
    assert result["position_guidance"]["hints"] == ["请让目标在画面中往下移一些。", "请让目标在画面中往右移一些。"]
    assert set(result["image_quality"]["issues"]) == {"overexposed", "low_sharpness"}
    receive(json.dumps(frame(2, x=.5)))
    assert card.check()["position_guidance"]["state"] == "aligned"
    receive(json.dumps(frame(3, count=2)))
    assert card.check()["position_guidance"]["state"] == "multiple"
    receive(json.dumps(frame(4, confidence=.1)))
    assert card.check()["position_guidance"]["state"] == "not_found"
    clock[0] = 4
    result = card.dispatch("target_guidance", {})
    assert result["image_quality"]["state"] == result["position_guidance"]["state"] == "unavailable"
    assert not result["hints"]


def test_scan_guidance_uses_fresh_camera_frame_without_vop():
    import cv2
    import numpy as np

    qr = cv2.QRCodeEncoder_create().encode("test")
    qr = cv2.resize(qr, (100, 100), interpolation=cv2.INTER_NEAREST)
    image = np.full((480, 640), 160, dtype=np.uint8)
    image[20:120, 270:370] = qr
    frame = {"received_monotonic": 10, "data": cv2.imencode(".jpg", image)[1].tobytes()}
    camera = types.SimpleNamespace(is_running=lambda: True, wait_for_color_frame=lambda timeout_s: (frame, 1))
    card = RegionPresencePlugin({}, "robot", Executor(), camera)
    card._clock = lambda: 11
    result = card.dispatch("scan_guidance", {})
    assert result["qr_located"] and result["hints"] == ["请把二维码往下移一些。"]
    image.fill(160)
    image[215:265, 295:345] = cv2.resize(qr, (50, 50), interpolation=cv2.INTER_NEAREST)
    frame["data"] = cv2.imencode(".jpg", image)[1].tobytes()
    result = card.dispatch("scan_guidance", {})
    assert result["size_ratio"] < .15 and "靠近" in result["hints"][0]
    image[190:290, 270:370] = qr
    frame["data"] = cv2.imencode(".jpg", image)[1].tobytes()
    result = card.dispatch("scan_guidance", {})
    assert result["framing_ok"] and result["hints"] == ["二维码已对齐，请保持不动。"]
    image.fill(250)
    frame["data"] = cv2.imencode(".jpg", image)[1].tobytes()
    result = card.dispatch("scan_guidance", {})
    assert not result["qr_located"] and "调暗" in result["hints"][0]
    card._clock = lambda: 13
    assert card.dispatch("scan_guidance", {})["state"] == "unavailable"


def test_stream_output_query_and_publication_metadata(running):
    card, executor, events, clock = running
    observe(executor, clock, 100, 0, 0)
    observe(executor, clock, 101, 1.5, 0)
    observe(executor, clock, 102, 2)
    observe(executor, clock, 103, 2.7)
    assert [event["event"] for event in events] == ["initialized", "appeared"]
    assert events[-1]["present"] == card.dispatch("check", {})["present"] is True
    assert events[-1]["input_topic"] == "/camera/objects"
    assert events[-1]["capture_time_verified"] is False
    assert events[-1]["event_id"] != events[0]["event_id"]


def test_stale_timer_publishes_unknown_once(running):
    card, executor, events, clock = running
    observe(executor, clock, 100, 0)
    observe(executor, clock, 101, .7)
    clock[0] = 4
    executor.nodes[0].tick()
    executor.nodes[0].tick()
    assert len(events) == 2
    assert events[-1]["event"] == "unavailable"
    assert events[-1]["present"] is None
    assert card.check()["state"] == "stale"


@pytest.mark.parametrize("text", ["{", "[]", '{"soc":20}', '{"error":"model unavailable"}', "x" * 262145],
                         ids=["malformed", "list", "wrong_card", "upstream_error", "oversized"])
def test_bad_messages_never_become_empty(running, text):
    card, executor, events, clock = running
    observe(executor, clock, 100, 0)
    observe(executor, clock, 101, .7)
    executor.nodes[0].receive(text)
    assert card.check()["state"] == "invalid_data"
    assert card.check()["present"] is None
    assert events[-1]["event"] == "unavailable"


def test_config_is_atomic_and_success_resets_observations(running):
    card, executor, events, clock = running
    observe(executor, clock, 100, 0)
    observe(executor, clock, 101, .7)
    old = card.info()["parameters"]
    assert card.dispatch("config", {"x_min": .9, "x_max": -.9})["state"] == "error"
    assert card.info()["parameters"] == old
    assert card.check()["present"] is True
    assert card.dispatch("config", {"x_min": .1})["state"] == "configured"
    assert card.check()["present"] is None
    assert events[-1]["state"] == "reconfigured"
    observe(executor, clock, 102, 1)
    observe(executor, clock, 103, 2.5)
    assert events[-1]["event"] == "initialized"
    assert events[-1]["present"] is False
    assert events[-1]["config_revision"] == 1


def test_rebind_stop_and_late_callbacks_do_not_revive_old_observations(running):
    card, executor, events, clock = running
    old = executor.nodes[0]
    observe(executor, clock, 100, 0)
    observe(executor, clock, 101, .7)
    assert card.dispatch("start", {"input_topic": "/camera/objects"})["state"] == "running"
    assert executor.nodes == [old]
    assert card.dispatch("start", {"input_topic": "/other/objects"})["state"] == "running"
    assert old.destroyed
    assert len(executor.nodes) == 1
    old.receive(json.dumps(frame(200)))
    old.tick()
    assert card.check()["state"] == "no_data"
    new = executor.nodes[0]
    assert card.stop()["state"] == "idle"
    before = len(events)
    new.receive(json.dumps(frame(201)))
    new.tick()
    assert len(events) == before
    assert card.check()["present"] is None
    assert executor.nodes == []


def test_multiple_or_conflicting_inputs_fail_without_rebinding(running):
    card, executor, events, clock = running
    old = executor.nodes[0]
    for args in ({"input_topics": ["/a", "/b"]},
                 {"input_topic": "/a", "input_topics": ["/b"]}):
        assert card.dispatch("start", args)["state"] == "error"
    assert executor.nodes == [old]
    assert card.info()["input_topic"] == "/camera/objects"


def test_cleanup_failure_blocks_replacement_and_can_be_retried(running):
    card, executor, events, clock = running
    node = executor.nodes[0]
    remove = executor.remove_node
    def fail(_):
        raise RuntimeError("executor unavailable")
    executor.remove_node = fail
    assert card.dispatch("start", {"input_topic": "/other/objects"})["state"] == "error"
    assert executor.nodes == [node]
    assert not node.destroyed
    executor.remove_node = remove
    assert card.stop()["state"] == "idle"
    assert node.destroyed


def test_partial_start_failure_destroys_node():
    executor, node = Executor(), Node()
    def fail(_):
        raise RuntimeError("add failed")
    executor.add_node = fail
    card = RegionPresencePlugin({}, "robot", executor)
    card._make_node = lambda topic, generation: (node, lambda event: None)
    result = card.dispatch("start", {"input_topic": "/camera/objects"})
    assert result["state"] == "error"
    assert node.destroyed


def test_executor_failure_after_attachment_also_removes_node():
    executor, node = Executor(), Node()
    def fail(item):
        executor.nodes.append(item)
        raise RuntimeError("failure after attachment")
    executor.add_node = fail
    card = RegionPresencePlugin({}, "robot", executor)
    card._make_node = lambda topic, generation: (node, lambda event: None)
    assert card.dispatch("start", {"input_topic": "/camera/objects"})["state"] == "error"
    assert node.destroyed
    assert executor.nodes == []


def test_publish_failure_is_visible_but_query_still_works(running):
    card, executor, events, clock = running
    def fail(_):
        raise RuntimeError("publish unavailable")
    card._publish = fail
    observe(executor, clock, 100, 0)
    observe(executor, clock, 101, .7)
    assert card.check()["present"] is True
    assert "publish unavailable" in card.info()["publish_error"]


def test_real_bundle_registration_dispatch_and_packaging():
    root = Path(__file__).resolve().parents[1]
    path = root / "noetix/bumi/main.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "BumiDeviceBundle")
    context = {"__name__": "region_presence_bundle_contract", "os": importlib.import_module("os")}
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), "exec"), context)
    for enabled in (False, True):
        bundle = context["BumiDeviceBundle"]({"plugins": {"region_presence": {"enabled": enabled}}},
                                            "robot", Executor(), None, None)
        assert [t["name"] for t in bundle.get_all_tools()] == (["region_presence"] if enabled else [])
        bundle.start_all()
        if enabled:
            assert bundle.dispatch("region_presence", {"action": "check"})["state"] == "idle"
        bundle.stop_all()
    cfg = yaml.safe_load((path.parent / "config.yaml").read_text(encoding="utf-8"))
    RegionPresencePlugin(cfg["plugins"]["region_presence"], "robot", Executor())
    manifest = yaml.safe_load((path.parent / "driver.yaml").read_text(encoding="utf-8"))
    assert {"name": "region_presence", "type": "processor"} in manifest["cards"]
    assert "COPY common/ /work/common/" in (path.parent / "Dockerfile").read_text(encoding="utf-8")


def test_actual_ros_factory_uses_string_qos_and_timer(monkeypatch):
    class RosNode(Node):
        def __init__(self, name, namespace):
            super().__init__()
            self.name, self.namespace, self.sent = name, namespace, []
        def create_publisher(self, message_type, topic, qos):
            self.output_topic, self.output_type = topic, message_type
            return types.SimpleNamespace(publish=lambda msg: self.sent.append(json.loads(msg.data)))
        def create_subscription(self, message_type, topic, callback, qos):
            self.input_topic, self.input_type, self.receive, self.qos = topic, message_type, callback, qos
        def create_timer(self, interval, callback):
            self.interval, self.tick = interval, callback
    class String:
        def __init__(self, data): self.data = data
    qos = {"QoSProfile": lambda **kwargs: types.SimpleNamespace(**kwargs),
           "ReliabilityPolicy": types.SimpleNamespace(BEST_EFFORT="best_effort"),
           "HistoryPolicy": types.SimpleNamespace(KEEP_LAST="keep_last"),
           "DurabilityPolicy": types.SimpleNamespace(VOLATILE="volatile")}
    for name, attrs in [("rclpy", {}), ("rclpy.node", {"Node": RosNode}), ("rclpy.qos", qos),
                        ("std_msgs", {}), ("std_msgs.msg", {"String": String})]:
        fake = types.ModuleType(name)
        fake.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, fake)
    executor = Executor()
    clock = [0]
    card = RegionPresencePlugin({}, "robot", executor)
    card._clock = lambda: clock[0]
    try:
        assert card.dispatch("start", {"input_topic": "/camera/objects"})["state"] == "running"
        node = executor.nodes[0]
        assert node.input_type is node.output_type is String
        assert node.input_topic == "/camera/objects"
        assert node.output_topic == "/robot/region_presence/events"
        assert node.qos.reliability == "best_effort"
        assert node.interval == .2
        node.receive(String(json.dumps(frame(100))))
        clock[0] = .7
        node.receive(String(json.dumps(frame(101))))
        assert node.sent[-1]["present"] is True
        clock[0] = 4
        node.tick()
        assert node.sent[-1]["state"] == "stale"
    finally:
        assert card.stop()["state"] == "idle"
    assert executor.nodes == []
