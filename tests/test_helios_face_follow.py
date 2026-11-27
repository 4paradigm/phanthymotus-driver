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


def test_person_tracks_closest_by_same_camera_depth():
    policy = tracking.FollowPolicy("person", "person")
    pixels = np.full((480, 640), 3100, dtype="<u2")
    pixels[:, 420:600] = 1400
    objects = [
        {"name": "person", "position": [-0.1, 0], "confidence": 1, "bbox": [220, 80, 350, 300]},
        {"name": "person", "position": [0.6, 0], "confidence": 1, "bbox": [450, 80, 575, 300]},
    ]
    assert policy.objects(dict(timestamp=1, objects=objects), 1, 1) == (None, "waiting_for_person_depth")
    target, reason = policy.objects(dict(timestamp=2, objects=objects), 2, 2, pixels, (640, 400))
    assert reason == "tracking" and target["offset"] == (0.6, 0)
    assert target["distance_estimate_mm"] == 1400
    pixels[:, 220:350] = 800
    closer, reason = policy.objects(dict(timestamp=2.1, objects=objects), 2.1, 2.1, pixels, (640, 400))
    assert reason == "tracking" and closer["offset"] == (-0.1, 0)
    assert closer["track_id"] != target["track_id"]


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
    def unexpected_face_detector():
        raise AssertionError("VOP modes should not initialize the face detector")

    monkeypatch.setattr(driver, "_Detector", unexpected_face_detector)
    calls, subscriptions, publications = [], {}, []
    monkeypatch.setattr(driver, "start_vop", calls.append)
    monkeypatch.setattr(driver, "start_perception", lambda *_: None)
    clock = NS(monotonic=lambda: 10, time=lambda: 100)
    monkeypatch.setattr(driver, "time", clock)

    def subscribe(_type, topic, callback, _qos):
        subscriptions[topic] = callback
        return topic

    node = NS(create_publisher=lambda *_: NS(publish=publications.append),
              create_subscription=subscribe, destroy_subscription=subscriptions.pop,
              create_timer=lambda *_: None, count_publishers=lambda topic: int(topic.removesuffix("/objects") in calls))
    motion_calls = []

    def request(operation, **fields):
        motion_calls.append((operation, fields))
        return {"ok": True, "camera_topic": fields.get("camera_topic"), "axes": AXES}

    card = driver.HeliosFaceFollow(node, NS(request=request))
    bundle = driver.DriverBundle([card])
    rgb = "/test/rgb"
    name = "helios_face_follow"
    started = bundle.dispatch(name, {"action": "start", "input_topic": rgb})
    assert started["state"] == "running" and started["mode"] == "person"
    subscriptions[rgb](frame(stamp=100))
    subscriptions[rgb + "/objects"](NS(data=json.dumps({"timestamp": 100, "objects": [
        {"name": "person", "position": [-0.55, 0.25], "confidence": 0.9},
        {"name": "cup", "position": [0.02, 0.01], "confidence": 0.9}]})))
    assert card.info()["target"]["offset"] == (-0.55, 0.25)
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
    card.move_motion()
    assert [op for op, _ in motion_calls] == ["acquire", "position"]
    assert bundle.dispatch(name, {"action": "info"})["motion_available"]
    card.move_motion()
    assert len(motion_calls) == 2
    clock.monotonic = lambda: 10.4
    card.move_motion()
    assert motion_calls[-1][0] == "halt"
    assert not bundle.dispatch(name, {"action": "info"})["motion_available"]
    clock.monotonic = lambda: 11
    card.expire()
    assert not json.loads(publications[-1].data)["available"]
    assert bundle.dispatch(name, {"action": "stop"})["state"] == "idle"
    assert set(subscriptions) == {driver.JOINTS}
    vop(NS(data="{}"))
    assert card.info()["reason"] == "idle"

    turn_calls = []
    base = NS(camera_ok=False, action_id=None, state="idle", status=lambda: {}, reset=lambda: None,
              halt=lambda _: True, step=lambda target, *_: turn_calls.append(target))
    card = driver.HeliosFaceFollow(node, base=base)
    clock.monotonic = lambda: 20
    camera = {rgb: {"topic": rgb, "id": driver.CHEST_CAMERA}}
    card.start({"input_topic": rgb, "camera_info": camera})
    assert base.camera_ok
    card.dispatch("set_target", {"object_name": "cup"})
    subscriptions[rgb](frame(stamp=100))
    subscriptions[rgb + "/objects"](NS(data=json.dumps({"timestamp": 100, "objects": [
        {"name": "cup", "position": [-0.25, 0], "confidence": 0.9}]})))
    card.move_motion()
    assert turn_calls[-1]["offset"] == (-0.25, 0)
    clock.monotonic = lambda: 20.4
    card.move_motion()
    assert turn_calls[-1] is None
    card.stop()
    card.start({"input_topic": rgb, "camera_info": {rgb: {"topic": rgb, "id": "head"}}})
    assert not base.camera_ok
    card.stop()

    camera_calls = []
    body_rgb = "/rokae_helios/camera/card_xyz/rgb"

    def camera_request(port, tool, action, **parameters):
        if port == 15782:
            assert (tool, action, parameters) == ("base_drive", "info", {})
            return {"worker_running": False, "connect_error": "chassis reports no navigation device"}
        assert (port, tool, parameters) == (15747, "camera_rgb", {"instance_id": "card-xyz"})
        camera_calls.append(action)
        if action == "info":
            return {"state": "idle", "camera": "chest_rgb",
                    "camera_info": [{"id": driver.CHEST_CAMERA, "topic": body_rgb}]}
        return {"state": "online" if action == "start" else "idle"}

    monkeypatch.setattr(driver, "call_local", camera_request)
    result = card.dispatch("toggle_person_follow", {"input_topic": body_rgb})
    assert result["state"] == "running" and "no navigation device" in result["启动说明"]
    assert base.camera_ok
    assert card.dispatch("pause_follow", {})["follow_paused"]
    assert not card.dispatch("resume_follow", {})["follow_paused"]
    assert card.dispatch("toggle_person_follow", {})["state"] == "idle"
    assert card.dispatch("set_target", {"object_name": "bottle"})["mode"] == "object"
    assert card.dispatch("follow_status", {})["object_name"] == "bottle"
    schemas = card.get_tools()[0]["inputSchema"]["x-action-params"]
    assert schemas["set_mode"]["required"] == ["mode"] and "toggle_person_follow" not in schemas
    assert camera_calls == ["info", "start", "stop"]


AXES = {
    "head_joint_1": dict(image_axis="x", image_sign=-1, lower_rad=-0.5, upper_rad=0.5, max_rad_s=0.2),
    "head_joint_2": dict(image_axis="y", image_sign=1, lower_rad=-0.2, upper_rad=0.2, max_rad_s=0.1),
}


def test_base_turns_in_place_with_fresh_targets_and_respects_existing_motion(monkeypatch):
    import sys

    monkeypatch.setitem(sys.modules, "tracking", tracking)
    monkeypatch.syspath_prepend(str(FILE.parent))
    from base_follow import BaseFollower

    clock = NS(now=10.0)
    drive = NS(active=None, nav="idle", level=0, calls=[])

    def call(port, tool, action, **params):
        drive.calls.append((port, tool, action, params))
        if tool == "controlled_spatial":
            return {"navigation": {"state": drive.nav}}
        if action == "info":
            return {"state": "ready", "motion_enabled": True, "sdk_available": True, "worker_running": True,
                    "chassis_exception_level": drive.level, "active_move": drive.active,
                    "odometry": {"age_s": 0.2, "vx": 0.0, "vy": 0.0, "w": 0.0}}
        if action == "move":
            drive.active = "our-turn"
            return {"state": "running", "action_id": drive.active}
        active, drive.active = drive.active, None
        return {"stopped": True, "cancelled_move": active}

    base = BaseFollower(call, lambda: clock.now)
    target = {"track_id": 1, "offset": (0.9, -0.5)}
    base.step(target, 1, 10.3)
    assert base.state == "waiting_for_chest_rgb" and drive.calls == []
    base.camera_ok = True
    base.step(target, 1, 10.3)
    base.step(target, 1, 10.3)
    assert drive.calls == []
    drive.nav = "moving"
    base.step(target, 2, 10.3)
    assert base.state == "base_busy"
    drive.nav, drive.active = "idle", "a-manual-move"
    base.step(target, 3, 10.3)
    base.halt("idle")
    assert drive.active == "a-manual-move"
    drive.active, drive.level = None, 1
    base.step(target, 4, 10.3)
    base.step(target, 5, 10.3)
    assert base.state == "waiting_for_base"
    drive.level = 0
    base.step(target, 6, 9.9)
    assert base.state == "waiting_for_target"
    base.step(target, 7, 10.3)
    base.step(target, 8, 10.3)
    assert drive.calls[-1] == (15782, "base_drive", "move",
                                {"vx": 0.0, "vy": 0.0, "vyaw": -8.0, "duration_s": 1.2})
    assert base.state == "controlling"
    clock.now = 11.8  # The first segment may still be waking the chassis.
    base.step(target, 9, 12.0)
    assert base.state == "controlling"
    drive.active = None  # A completed segment is followed by another if still off center.
    closer = {"track_id": 1, "offset": (0.25, -0.5)}
    base.step(closer, 10, 12.0)
    assert base.state == "settling"
    base.step(closer, 11, 12.0)
    base.step(closer, 12, 12.0)
    assert drive.calls[-1][3]["vyaw"] == -3.5
    centered = {"track_id": 1, "offset": (0.06, -0.5)}
    base.step(centered, 13, 12.0)
    assert drive.calls[-1][2] == "stop_move" and base.state == "centered"
    calls = len(drive.calls)
    jitter = {"track_id": 1, "offset": (0.09, -0.5)}
    base.step(jitter, 14, 12.0)
    assert len(drive.calls) == calls and base.state == "centered"

    base.step(target, 15, 12.0)
    base.step(target, 16, 12.0)
    base.step(None, 17, 12.0)
    assert drive.calls[-1][2] == "stop_move" and base.state == "waiting_for_target"

    other_side = {"track_id": 1, "offset": (-0.4, 0)}
    base.step(other_side, 18, 12.0)
    base.step(other_side, 19, 12.0)
    assert drive.calls[-1][3]["vyaw"] == 14 * 0.4
    clock.now = 15.6
    base.step(other_side, 20, 15.9)
    assert drive.calls[-1][2] == "stop_move" and base.state == "fault" and base.error


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
