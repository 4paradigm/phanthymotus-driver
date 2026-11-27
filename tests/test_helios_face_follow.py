"""ROS-free policy tests for the Helios tracking card."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS
import zlib

import numpy as np

FILE = Path(__file__).resolve().parents[1] / "rokae/helios-face-follow/tracking.py"
SPEC = importlib.util.spec_from_file_location("helios_tracking", FILE)
tracking = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tracking)

LEFT, RIGHT = (80, 100, 120, 120), (460, 100, 80, 80)


def frame(stamp=10, optical="color", data=b""):
    return NS(header=NS(frame_id=optical, stamp=NS(sec=int(stamp),
              nanosec=round((stamp % 1) * 1e9))), format="image/depth-zlib", data=data)


def test_nearest_face_uses_depth_and_switches_to_new_nearest_face():
    policy = tracking.FollowPolicy("nearest_face")
    depth = np.full((480, 640), 2400, dtype="<u2")
    depth[100:180, 460:540] = 900
    rgb = frame()
    target, reason = policy.faces([LEFT, RIGHT], (640, 480), rgb,
                                  frame(data=zlib.compress(depth.tobytes())), np, 10, 10)
    assert reason == "tracking" and target["distance_mm"] == 900
    assert target["bbox"] == list(RIGHT)
    depth[100:180, 460:540] = 3400
    target, _ = policy.faces([LEFT, RIGHT], (640, 480), rgb,
                             frame(data=zlib.compress(depth.tobytes())), np, 10.2, 10.2)
    assert target["bbox"] == list(LEFT) and target["track_id"] == 2


def test_depth_must_be_aligned_fresh_and_well_formed():
    data = zlib.compress(np.full((480, 640), 900, dtype="<u2").tobytes())
    for depth in (None, frame(stamp=10.2, data=data), frame(optical="native_depth", data=data),
                  frame(data=b"bad"), frame(data=zlib.compress(b"x" * (640*480*2 + 1)))):
        assert tracking.nearest_face([LEFT], (640, 480), frame(), depth, np) is None
    assert tracking.nearest_face([LEFT], (640, 400), frame(), frame(data=data), np) is None


def test_speaker_waits_for_end_even_when_another_person_starts_or_vision_is_lost():
    policy = tracking.FollowPolicy("speaker")

    def event(identifier, ts, active=True, image_x=-0.55):
        return policy.speech_event(dict(timestamp=ts, active=active, image_x=image_x,
                                        utterance_id=identifier), ts)

    def target(boxes, now):
        return policy.faces(boxes, (640, 480), None, None, np, now, now)

    assert event("a", 12)
    assert not event("b", 12.001, image_x=0.56)
    assert target([LEFT, RIGHT], 12.01)[0]["bbox"] == list(LEFT)
    assert not event("b", 12.02, image_x=0.56)
    assert target([LEFT, RIGHT], 12.03)[0]["bbox"] == list(LEFT)
    assert event("a", 12.4)
    assert target([RIGHT], 12.62) == (None, "speaker_face_lost")
    assert not event("b", 12.65, image_x=0.56)
    assert not event("b", 12.66, active=False)
    assert event("a", 12.67, active=False)
    assert target([LEFT, RIGHT], 12.69) == (None, "waiting_for_speech")
    assert event("b", 12.71, image_x=0.56)
    assert target([LEFT, RIGHT], 12.72)[0]["bbox"] == list(RIGHT)


def test_vop_selects_label_and_rejects_stale_low_confidence_and_duplicate_packets():
    policy = tracking.FollowPolicy("object", "cup")
    packet = dict(timestamp=2, objects=[
        {"name": "bottle", "position": [0, 0], "confidence": 1},
        {"name": "cup", "position": [-0.1, 0], "confidence": 0.2},
        {"name": "cup", "position": [0.5, 0.1], "confidence": .95},
    ])
    assert policy.objects(packet, 3, 3)[0] is None
    target, reason = policy.objects(packet, 2.1, 2.1)
    assert reason == "tracking" and target["offset"] == (0.5, 0.1)
    assert policy.objects(packet, 2.2, 2.2)[0] is None
    packet.update(timestamp=2.2, latency_ms=600)
    assert policy.objects(packet, 2.3, 2.3)[0] is None


def test_canvas_lifecycle_and_object_mode(monkeypatch):
    import json
    import sys
    from types import ModuleType

    for name, attrs in {
        "rclpy.qos": {"QoSProfile": NS, "ReliabilityPolicy": NS(BEST_EFFORT=1, RELIABLE=2)},
        "rclpy.validate_full_topic_name": {"validate_full_topic_name": lambda name: None},
        "std_msgs.msg": {"String": NS},
        "sensor_msgs.msg": {"CompressedImage": NS},
    }.items():
        module = ModuleType(name)
        vars(module).update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setitem(sys.modules, "tracking", tracking)
    monkeypatch.syspath_prepend(str(FILE.parent))
    spec = importlib.util.spec_from_file_location("helios_follow_driver", FILE.with_name("main.py"))
    driver = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(driver)
    monkeypatch.setattr(driver, "_Detector", lambda: NS())
    calls, subscriptions, publications = [], {}, []
    monkeypatch.setattr(driver, "start_vop", calls.append)
    clock = NS(monotonic=lambda: 10, time=lambda: 100)
    monkeypatch.setattr(driver, "time", clock)

    def subscribe(_type, topic, callback, _qos):
        subscriptions[topic] = callback
        return topic

    node = NS(create_publisher=lambda *_: NS(publish=publications.append),
              create_subscription=subscribe, destroy_subscription=subscriptions.pop,
              create_timer=lambda *_: None, count_publishers=lambda _: 0)
    motion_calls = []

    def request(operation, **fields):
        motion_calls.append((operation, fields))
        return {"ok": True, "camera_topic": fields.get("camera_topic"), "axes": AXES}

    card = driver.HeliosFaceFollow(node, NS(request=request))
    bundle = driver.DriverBundle([card])
    rgb = "/test/rgb"
    name = "helios_face_follow"
    assert bundle.dispatch(name, {"action": "start", "input_topic": rgb})["state"] == "running"
    assert bundle.dispatch(name, {"action": "set_target", "object_name": "Cup"})["mode"] == "object"
    assert calls == [rgb]
    subscriptions[rgb](frame(stamp=100))
    subscriptions[driver.JOINTS](NS(data=json.dumps({"available": True, "received_at_ms": 100000,
        "joints": [{"name": joint, "q": 0} for joint in driver.HEAD_JOINTS]})))
    vop = subscriptions[rgb + "/objects"]
    vop(NS(data=json.dumps({"timestamp": 100, "objects": [
        {"name": "cup", "position": [0.25, 0], "confidence": 0.9}]})))
    assert bundle.dispatch(name, {"action": "info"})["target"]["offset"] == (0.25, 0)
    assert json.loads(publications[-1].data)["available"]
    card.move_head()
    assert [op for op, _ in motion_calls] == ["acquire", "position"]
    assert bundle.dispatch(name, {"action": "info"})["motion_available"]
    card.move_head()
    assert len(motion_calls) == 2
    clock.monotonic = lambda: 10.4
    card.move_head()
    assert motion_calls[-1][0] == "halt"
    assert not bundle.dispatch(name, {"action": "info"})["motion_available"]
    clock.monotonic = lambda: 11
    card.expire()
    assert not json.loads(publications[-1].data)["available"]
    assert bundle.dispatch(name, {"action": "stop"})["state"] == "idle"
    assert set(subscriptions) == {driver.JOINTS}
    vop(NS(data="{}"))
    assert card.info()["reason"] == "idle"


AXES = {
    "head_joint_1": dict(image_axis="x", image_sign=-1, lower_rad=-0.5, upper_rad=0.5, max_rad_s=0.2),
    "head_joint_2": dict(image_axis="y", image_sign=1, lower_rad=-0.2, upper_rad=0.2, max_rad_s=0.1),
}


def test_head_bridge_limits_faults_and_stop(monkeypatch, tmp_path):
    import json
    import socketserver
    import sys
    import threading

    import pytest

    monkeypatch.setitem(sys.modules, "tracking", tracking)
    monkeypatch.syspath_prepend(str(FILE.parent))
    import head_control

    requests = []

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            request = json.loads(self.rfile.readline())
            requests.append(request)
            reject = request["operation"] == "position" and len(requests) == 3
            self.wfile.write(json.dumps({"version": 1, "ok": not reject,
                "camera_topic": request.get("camera_topic"), "axes": AXES}).encode() + b"\n")

    # macOS AF_UNIX paths have a 104-byte limit.
    path = str(tmp_path.resolve() / "h")
    if len(path.encode()) >= 104:
        monkeypatch.chdir(tmp_path)
        path = "h"
    with socketserver.UnixStreamServer(path, Handler) as server:
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            controller = head_control.HeadFollower(head_control.UnixHeadBridge(path))
            target = {"track_id": 1, "offset": (1, 0.8)}
            feedback = {"head_joint_1": -0.498, "head_joint_2": 0.1}
            controller.step("/head/rgb", target, feedback, 1, 1.0, 1.3)
            position = requests[-1]
            assert position["positions_rad"] == pytest.approx({"head_joint_1": -0.5, "head_joint_2": 0.108})
            assert position["max_rad_s"] == {"head_joint_1": 0.15, "head_joint_2": 0.1}
            assert position["valid_until_monotonic_ns"] == 1300000000
            assert position["ttl_ms"] == 250
            controller.step("/head/rgb", target, feedback, 2, 1.1, 1.3)
            assert [r["operation"] for r in requests] == ["acquire", "position", "position", "halt"]
            assert requests[0]["lease"] == requests[-1]["lease"]
            assert controller.status()["state"] == "fault"
            controller.step("/head/rgb", target, feedback, 3, 2.1, 2.3)
            assert len(requests) == 4
            controller.reset()
            controller.step("/head/rgb", target, feedback, 4, 2.2, 2.4)
            controller.step("/head/rgb", target, None, 5, 2.3, 2.4)
            assert requests[-1]["operation"] == "halt"
            assert controller.status()["state"] == "waiting_for_feedback"
        finally:
            server.shutdown()
            worker.join()
