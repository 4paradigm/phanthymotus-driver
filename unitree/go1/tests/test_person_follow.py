"""Go1 person-follow decisions; no robot or model is needed for these tests."""

import math
import importlib
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from unitree.go1.person_follow import (Detection, TargetTracker, PersonFollowPlugin,
                                      YoloXOnnxDetector, decode_yolox,
                                      extract_latest_frame, follow_command)


def box(kind, x1, y1, x2, y2, score=0.9):
    return Detection(kind, x1, y1, x2, y2, score)


def test_selects_nearest_visible_person_then_keeps_lock():
    tracker = TargetTracker()
    far = box("person", .1, .2, .3, .55)
    near = box("person", .6, .2, .9, .8)
    assert tracker.update([far, near]) == near
    assert tracker.update([box("person", .12, .2, .35, .9),
                           box("person", .62, .25, .91, .82)]).x1 == .62


def test_shoes_can_acquire_when_no_body_is_visible():
    tracker = TargetTracker()
    left = box("shoe", .42, .7, .49, .8)
    right = box("shoe", .52, .71, .59, .81)
    target = tracker.update([left, right])
    assert target.kind == "shoe"
    assert .49 < target.center_x < .52
    assert target.y2 == .81


def test_shoe_pair_does_not_depend_on_detector_output_order():
    tracker = TargetTracker()
    right = box("shoe", .52, .71, .59, .81)
    left = box("shoe", .42, .7, .49, .8)
    target = tracker.update([right, left])
    assert target is not None
    assert .49 < target.center_x < .52


def test_ambiguous_people_do_not_start_motion():
    tracker = TargetTracker()
    assert tracker.update([box("person", .1, .2, .3, .8),
                           box("person", .6, .2, .8, .81)]) is None


def test_lost_target_never_switches_to_another_person():
    tracker = TargetTracker()
    assert tracker.update([box("shoe", .1, .7, .2, .8)]) is not None
    assert tracker.update([box("shoe", .7, .7, .8, .9)]) is None
    assert tracker.update([box("shoe", .7, .7, .8, .9)]) is None


def test_keeps_original_person_when_another_enters_matching_area():
    tracker = TargetTracker()
    original = box("person", .40, .2, .60, .70)
    assert tracker.update([original]) == original
    moved_original = box("person", .42, .2, .62, .71)
    newcomer = box("person", .65, .2, .85, .73)
    assert tracker.update([newcomer, moved_original]) == moved_original
    assert not tracker.lost
    assert tracker.update([box("person", .44, .2, .64, .72), newcomer]).x1 == .44


def test_stops_if_two_people_are_equally_likely_to_be_locked_target():
    tracker = TargetTracker()
    tracker.update([box("person", .40, .2, .60, .70)])
    assert tracker.update([box("person", .36, .2, .56, .71),
                           box("person", .44, .2, .64, .71)]) is None
    assert tracker.lost


def test_does_not_accept_a_distant_replacement_inside_old_matching_radius():
    tracker = TargetTracker()
    tracker.update([box("person", .40, .2, .60, .70)])
    assert tracker.update([box("person", .60, .2, .80, .72)]) is None
    assert tracker.lost


def test_same_position_with_different_appearance_does_not_take_lock():
    tracker = TargetTracker()
    original = Detection("person", .40, .2, .60, .70, .9, (220, 20, 20))
    newcomer = Detection("person", .41, .2, .61, .71, .9, (20, 20, 220))
    assert tracker.update([original]) == original
    assert tracker.update([newcomer]) is None
    assert tracker.lost


def test_appearance_keeps_original_when_bystander_is_closer_to_old_box():
    tracker = TargetTracker()
    original = Detection("person", .40, .2, .60, .70, .9, (220, 20, 20))
    moved = Detection("person", .47, .2, .67, .71, .9, (215, 25, 20))
    bystander = Detection("person", .41, .2, .61, .71, .9, (20, 20, 220))
    tracker.update([original])
    assert tracker.update([bystander, moved]) == moved
    assert not tracker.lost


def test_follow_command_stops_for_missing_far_or_close_target():
    assert follow_command(None) == (0.0, 0.0)
    assert follow_command(box("shoe", .4, .8, .6, .92)) == (0.0, 0.0)
    assert follow_command(box("shoe", .6, .6, .8, .74)) == (0.0, 0.0)
    # 远景样本 9–12 的脚点约在画面高度 0.53–0.56，不应触发转向或前进。
    assert follow_command(box("person", .1, .2, .3, .53)) == (0.0, 0.0)
    assert follow_command(box("shoe", .6, .4, .8, .55)) == (0.0, 0.0)
    assert follow_command(box("shoe", .4, .4, .6, .66)) == (0.0, 0.0)
    vx, yaw = follow_command(box("shoe", .6, .4, .8, .60))
    assert 0 < vx <= .15
    assert yaw < 0


def test_crossing_targets_are_ambiguous_and_end_lock():
    tracker = TargetTracker()
    tracker.update([box("person", .2, .3, .4, .7)])
    assert tracker.update([box("person", .22, .3, .42, .71),
                           box("person", .25, .3, .45, .72)]) is None
    assert tracker.lost


class Client:
    available = True

    def __init__(self):
        self.moves = []
        self.stops = 0

    def diagnostics(self):
        return {"accessible": True, "recv_count": self.stops + len(self.moves) + 1}

    def snapshot(self):
        return {"fresh": True, "observed_monotonic": time.monotonic(),
                "mode": 2, "velocity": [.1, 0, 0]}

    def move(self, vx, vy, yaw, gait=1):
        self.moves.append((vx, vy, yaw))
        return {"vx": vx, "vy": vy, "yaw": yaw}

    def stop_move(self):
        self.stops += 1

    def set_posture(self, mode, **kwargs):
        return {"mode": mode}


def test_card_requires_model_before_following():
    client = Client()
    card = PersonFollowPlugin({"model_path": ""}, client=client)
    assert card.dispatch("follow", {"confirm": True})["code"] == "MODEL_UNAVAILABLE"
    assert not client.moves


def test_follow_advertises_physical_motion_as_dangerous():
    schema = PersonFollowPlugin({"model_path": ""}, client=Client()).get_tool()["inputSchema"]
    assert schema["x-is-dangerous"] is True


def test_default_config_keeps_experimental_follow_disabled():
    config = Path(__file__).resolve().parents[1] / "config.yaml"
    assert yaml.safe_load(config.read_text(encoding="utf-8"))["plugins"]["person_follow"]["enabled"] is False


def test_sdk_snapshot_records_successful_parse_time():
    from unitree.go1.go1_sdk_client import Go1HighSdkClient

    client = object.__new__(Go1HighSdkClient)
    client._lock = threading.Lock()
    client._snapshot = {}
    before = time.monotonic()
    client._parse_state(SimpleNamespace())
    assert before <= client.snapshot()["observed_monotonic"] <= time.monotonic()


@pytest.mark.parametrize("action,args", [
    ("stop", {}),
    ("move", {"vx": .1}),
    ("stand_down", {}),
])
def test_other_loco_action_cancels_follow_before_taking_control(tmp_path, monkeypatch, action, args):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    client = Client()
    model = tmp_path / "shoes.onnx"
    model.write_bytes(b"model")
    bundle = main.Go1Bundle({"plugins": {"loco": {"enabled": True},
                                        "person_follow": {"enabled": True,
                                                          "model_path": str(model)}}},
                            "test_go1", None, client)
    follow = next(p for p in bundle._plugins if p.get_tool()["name"] == "person_follow")
    entered = threading.Event()

    def run():
        entered.set()
        while not follow._cancel.wait(.01):
            client.move(.1, 0, 0)

    monkeypatch.setattr(follow, "_run", run)
    try:
        assert bundle.dispatch("person_follow", {"action": "follow", "confirm": True})["ok"]
        assert entered.wait(1)
        assert bundle.dispatch("loco", {"action": action, **args})["ok"]
        assert follow._cancel.is_set()
        assert not follow._worker.is_alive()
    finally:
        follow.stop()


def test_card_stop_cancels_worker_and_stops_robot(tmp_path, monkeypatch):
    client = Client()
    model = tmp_path / "shoes.onnx"
    model.write_bytes(b"model")
    card = PersonFollowPlugin({"model_path": str(model)}, client=client)
    entered = threading.Event()

    def run():
        entered.set()
        while not card._cancel.wait(.01):
            pass

    monkeypatch.setattr(card, "_run", run)
    assert card.dispatch("follow", {"confirm": True})["ok"]
    assert entered.wait(1)
    assert card.dispatch("stop", {})["ok"]
    assert client.stops >= 1
    assert not card._worker.is_alive()


def test_starting_follow_cancels_running_loco_timed_move(tmp_path, monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    client = Client()
    model = tmp_path / "shoes.onnx"
    model.write_bytes(b"model")
    bundle = main.Go1Bundle({"plugins": {"loco": {"enabled": True},
                                        "person_follow": {"enabled": True,
                                                          "model_path": str(model)}}},
                            "test_go1", None, client)
    follow = next(p for p in bundle._plugins if p.get_tool()["name"] == "person_follow")
    loco = next(p for p in bundle._plugins if p.get_tool()["name"] == "loco")
    entered = threading.Event()

    def run():
        entered.set()
        while not follow._cancel.wait(.01):
            pass

    monkeypatch.setattr(follow, "_run", run)
    try:
        # 反向交接：loco 定时 move 线程运行期间启动 follow，该线程必须被取消，
        # 否则会以 100ms 周期继续覆盖跟随卡的保守命令。
        assert bundle.dispatch("loco", {"action": "move", "vx": .3, "duration": 60})["ok"]
        time.sleep(.3)
        moves_before = len(client.moves)
        assert moves_before >= 1
        assert bundle.dispatch("person_follow", {"action": "follow", "confirm": True})["ok"]
        assert entered.wait(1)
        time.sleep(.3)
        assert len(client.moves) == moves_before
        assert client.stops >= 1
        assert loco._thread is None
    finally:
        follow.stop()


def test_starting_follow_interrupts_running_gesture():
    from unitree.go1.controllers import GesturePlugin

    client = Client()
    card = GesturePlugin({}, "test_go1", None, client)
    assert card.dispatch("greet", {"times": 2})["ok"]
    assert card._lock.locked()
    card.preempt_motion()
    assert not card._lock.locked()
    thread = card._thread
    assert thread is None or not thread.is_alive()


def test_cleanup_releases_camera_when_stop_move_fails(tmp_path, monkeypatch):
    from unitree.go1 import camera

    class BoomClient(Client):
        def stop_move(self):
            raise RuntimeError("sdk proxy gone")

    model = tmp_path / "shoes.onnx"
    model.write_bytes(b"model")
    card = PersonFollowPlugin({"model_path": str(model)}, client=BoomClient())

    def run():
        raise RuntimeError("camera failed")

    monkeypatch.setattr(card, "_run", run)
    try:
        assert card.dispatch("follow", {"confirm": True})["ok"]
        card._worker.join(1)
        assert not card._worker.is_alive()
        with camera._CAMERA_LOCK:
            assert "front" not in camera._SNAPSHOT_POSITIONS
        info = card.dispatch("info", {})
        assert info["state"] == "error"
        assert "camera failed" in info["reason"]
    finally:
        with camera._CAMERA_LOCK:
            camera._SNAPSHOT_POSITIONS.discard("front")


def test_cancelled_follow_cleanup_does_not_stop_new_owner(tmp_path, monkeypatch):
    client = Client()
    model = tmp_path / "shoes.onnx"
    model.write_bytes(b"model")
    card = PersonFollowPlugin({"model_path": str(model)}, client=client)
    entered = threading.Event()

    def run():
        card.stop()
        client.move(.1, 0, 0)
        entered.set()

    monkeypatch.setattr(card, "_run", run)
    assert card.dispatch("follow", {"confirm": True})["ok"]
    assert entered.wait(1)
    card._worker.join(1)
    assert client.stops == 1
    assert client.moves == [(.1, 0, 0)]


def test_yolox_decoder_maps_person_and_shoe_boxes():
    raw = np.zeros((1, 3549, 7), dtype=np.float32)
    raw[0, 10 * 52 + 10] = [0, 0, math.log(2), math.log(2), .9, .9, .1]
    raw[0, 10 * 52 + 11] = [0, 0, math.log(2), math.log(2), .9, .1, .9]
    found = decode_yolox(raw, 416, 416, 1.0)
    assert {item.kind for item in found} == {"person", "shoe"}
    person = next(item for item in found if item.kind == "person")
    assert abs(person.center_x - 80 / 416) < .001
    assert abs(person.y2 - 88 / 416) < .001


def test_yolox_decoder_rejects_unexpected_model_shape():
    try:
        decode_yolox(np.zeros((1, 3549, 85), dtype=np.float32), 416, 416, 1.0)
    except ValueError as exc:
        assert "two classes" in str(exc)
    else:
        raise AssertionError("COCO model must not be interpreted as a shoe model")


def test_bundle_registers_follow_card_without_starting_motion(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    main = importlib.import_module("unitree.go1.main")
    client = Client()
    bundle = main.Go1Bundle({"plugins": {"person_follow": {"enabled": True}}},
                            "test_go1", None, client)
    assert [tool["name"] for tool in bundle.get_all_tools()] == ["person_follow"]
    assert bundle.dispatch("person_follow", {"action": "start"}) == {"state": "ready"}
    assert not client.moves


def test_follow_refuses_busy_front_camera(tmp_path):
    from unitree.go1 import camera

    model = tmp_path / "shoes.onnx"
    model.write_bytes(b"model")
    card = PersonFollowPlugin({"model_path": str(model)}, client=Client())
    with camera._CAMERA_LOCK:
        camera._SNAPSHOT_POSITIONS.add("front")
    try:
        assert card.dispatch("follow", {"confirm": True})["code"] == "RESOURCE_BUSY"
    finally:
        with camera._CAMERA_LOCK:
            camera._SNAPSHOT_POSITIONS.discard("front")


def test_receiver_discards_complete_old_frames_but_keeps_partial_tail():
    from struct import pack

    pending = bytearray(pack(">I", 3) + b"old" + pack(">I", 3) + b"new" +
                        pack(">I", 4) + b"pa")
    assert extract_latest_frame(pending) == b"new"
    assert pending == bytearray(pack(">I", 4) + b"pa")
    pending.extend(b"rt")
    assert extract_latest_frame(pending) == b"part"


def test_detector_accepts_jpeg_and_two_class_onnx_output(monkeypatch):
    from io import BytesIO
    from PIL import Image

    raw = np.zeros((1, 3549, 7), dtype=np.float32)
    raw[0, 10 * 52 + 10] = [0, 0, math.log(2), math.log(2), .9, .1, .9]

    class Session:
        def get_inputs(self):
            return [SimpleNamespace(name="images", shape=[1, 3, 416, 416])]

        def run(self, unused, inputs):
            assert inputs["images"].shape == (1, 3, 416, 416)
            return [raw]

    monkeypatch.setitem(sys.modules, "onnxruntime", SimpleNamespace(
        SessionOptions=lambda: SimpleNamespace(),
        InferenceSession=lambda *args, **kwargs: Session()))
    encoded = BytesIO()
    Image.new("RGB", (416, 416), "white").save(encoded, format="JPEG")
    found = YoloXOnnxDetector("unused.onnx").detect(encoded.getvalue())
    assert len(found) == 1 and found[0].kind == "shoe"
    assert len(found[0].appearance) == 48


def test_runtime_stops_when_locked_shoe_disappears(tmp_path, monkeypatch):
    from struct import pack
    from unitree.go1 import person_follow

    class Connection:
        def __init__(self):
            self.chunks = [pack(">I", 1) + b"a", None,
                           pack(">I", 1) + b"b", None]

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def setblocking(self, value):
            pass

        def recv(self, size):
            value = self.chunks.pop(0)
            if value is None:
                raise BlockingIOError()
            return value

    class Detector:
        def __init__(self, path):
            pass

        def detect(self, jpeg):
            return ([box("shoe", .4, .4, .6, .62)] if jpeg == b"a" else
                    [box("shoe", .8, .4, .95, .6)])

    client = Client()
    card = PersonFollowPlugin({"model_path": str(tmp_path / "model.onnx")}, client=client)
    card._owns_motion = True
    monkeypatch.setattr(person_follow, "YoloXOnnxDetector", Detector)
    monkeypatch.setattr(person_follow.socket, "create_connection", lambda *args, **kwargs: Connection())
    monkeypatch.setattr(person_follow.select, "select", lambda connections, *args: (connections, [], []))
    with pytest.raises(RuntimeError, match="target lost"):
        card._run()
    assert client.moves and client.moves[0][0] > 0
    assert client.stops >= 1


def test_cancel_during_diagnostics_prevents_late_follow_move(tmp_path, monkeypatch):
    from struct import pack
    from unitree.go1 import person_follow

    class Connection:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def setblocking(self, value):
            pass

        def recv(self, size):
            if not hasattr(self, "sent"):
                self.sent = True
                return pack(">I", 1) + b"a"
            raise BlockingIOError()

    class Detector:
        def __init__(self, path):
            pass

        def detect(self, jpeg):
            return [box("shoe", .4, .4, .6, .62)]

    client = Client()
    card = PersonFollowPlugin({"model_path": str(tmp_path / "model.onnx")}, client=client)
    card._owns_motion = True

    def diagnostics():
        card.stop()
        return {"accessible": True, "recv_count": 1}

    monkeypatch.setattr(client, "diagnostics", diagnostics)
    monkeypatch.setattr(person_follow, "YoloXOnnxDetector", Detector)
    monkeypatch.setattr(person_follow.socket, "create_connection", lambda *args, **kwargs: Connection())
    monkeypatch.setattr(person_follow.select, "select", lambda connections, *args: (connections, [], []))
    card._run()
    assert client.moves == []


def test_rising_packet_count_cannot_override_stale_robot_feedback(tmp_path, monkeypatch):
    from struct import pack
    from unitree.go1 import person_follow

    class Connection:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def setblocking(self, value):
            pass

        def recv(self, size):
            if not hasattr(self, "sent"):
                self.sent = True
                return pack(">I", 1) + b"a"
            raise BlockingIOError()

    class Detector:
        def __init__(self, path):
            pass

        def detect(self, jpeg):
            return [box("shoe", .4, .4, .6, .62)]

    client = Client()
    client.snapshot = lambda: {"fresh": True, "observed_monotonic": time.monotonic() - 2,
                               "mode": 2, "velocity": [.1, 0, 0]}
    client.move = lambda *args, **kwargs: pytest.fail("stale feedback must prevent motion")
    card = PersonFollowPlugin({"model_path": str(tmp_path / "model.onnx")}, client=client)
    card._owns_motion = True
    monkeypatch.setattr(person_follow, "YoloXOnnxDetector", Detector)
    monkeypatch.setattr(person_follow.socket, "create_connection", lambda *args, **kwargs: Connection())
    monkeypatch.setattr(person_follow.select, "select", lambda connections, *args: (connections, [], []))
    with pytest.raises(RuntimeError, match="feedback"):
        card._run()
    assert client.moves == []
    assert client.stops >= 1


@pytest.mark.parametrize("mode,velocity", [(1, [0, 0, 0]), (2, [.1, 0, 0])])
def test_follow_stops_when_robot_reports_no_motion_after_command(tmp_path, monkeypatch,
                                                                mode, velocity):
    from struct import pack
    from unitree.go1 import person_follow

    class Connection:
        ready = True

        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def setblocking(self, value):
            pass

        def recv(self, size):
            if self.ready:
                self.ready = False
                return pack(">I", 1) + b"a"
            self.ready = True
            raise BlockingIOError()

    class Detector:
        def __init__(self, path):
            pass

        def detect(self, jpeg):
            return [box("shoe", .4, .4, .6, .62)]

    client = Client()
    client.snapshot = lambda: {"fresh": True, "observed_monotonic": time.monotonic(),
                               "mode": mode, "velocity": velocity, "position": [0, 0, 0]}
    card = PersonFollowPlugin({"model_path": str(tmp_path / "model.onnx")}, client=client)
    card._owns_motion = True
    monkeypatch.setattr(person_follow, "YoloXOnnxDetector", Detector)
    monkeypatch.setattr(person_follow.socket, "create_connection", lambda *args, **kwargs: Connection())

    deadline = time.monotonic() + 1.2

    def select(connections, *args):
        if time.monotonic() > deadline:
            raise TimeoutError("test stopped an unbounded follow loop")
        time.sleep(.06)
        return connections, [], []

    monkeypatch.setattr(person_follow.select, "select", select)
    with pytest.raises(RuntimeError, match="motion feedback"):
        card._run()
    assert client.moves
    assert client.stops >= 1
