"""Host interaction and measured/cancellable arm execution, without hardware."""
import importlib.util
import math
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1] / "robotera/q5_bundle"


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


game_module = load("wooden_man")
motion = load("wooden_man_motion")


def pose(dx=0, joint=None):
    points = [[100 + i * 3 + (dx if joint is None or i == joint else 0),
               100 + (100 if i >= 11 else 0), .99] for i in range(17)]
    return {"image_size": [320, 320], "persons": [{"id": 1, "keypoints": points}]}


class Host:
    def __init__(self, **config):
        self.now = 0.
        self.words, self.actions = [], []
        self.game = game_module.Game(self.words.append, config, clock=lambda: self.now, emit_action=self.actions.append)

    def step(self, seconds):
        self.now += seconds
        self.game.tick()

    def ready(self):
        for _ in range(17):
            self.game.pose(pose())
            self.step(.2)

    def complete(self):
        rid = self.game.pending_motion["request_id"]
        self.step(3.01)
        self.game.motion_feedback({"kind": "wooden_man_motion_feedback", "request_id": "old-id", "state": "completed"})
        assert self.game.phase == "robot_action"
        self.game.motion_feedback({"kind": "wooden_man_motion_feedback", "request_id": rid, "state": "completed"})
        assert self.game.phase == "robot_settle"
        self.step(1.01)

    def watch(self):
        self.game.text("游戏开始")
        self.ready()
        assert self.game.phase == "robot_action" and self.actions[-1]["gesture"] == "welcome"
        self.complete()
        assert self.game.phase == "countdown"
        self.step(5)
        self.step(3.5)
        assert self.game.phase == "watch"

    def still(self):
        for _ in range(31):
            self.game.pose(pose())
            self.step(.1)


def test_ready_guidance_needs_stable_visibility_and_never_auto_starts():
    h = Host()
    for _ in range(50):
        h.game.pose({"persons": []})
        h.step(.1)
    assert h.words == ["没有看清玩家，请一个人站在镜头前。"]
    assert h.game.phase == "waiting" and not h.actions
    h.ready()
    assert h.words[-1] == "准备好了，等你的开始口令。"
    assert h.game.phase == "waiting"
    h.step(2)
    h.game.pose(pose())
    assert not h.game.ready


def test_preparation_never_plays_with_multiple_people_or_missing_hands():
    h = Host(detect_fingers=True)
    h.game.text("游戏开始")
    h.ready()
    assert h.game.phase == "preparing" and not h.actions
    assert any("双手" in text for text in h.words)
    h.step(21)
    assert h.game.result == "error" and h.game.reason == "preparation_timeout"
    # Error response may gesture but never progresses to countdown/watch.
    h.complete()
    assert h.game.phase == "error"


def test_gesture_does_not_advance_until_matching_completion_and_settle():
    h = Host()
    h.game.text("游戏开始")
    h.ready()
    original = h.actions[-1]
    h.step(2)
    assert h.game.phase == "robot_action" and h.game.round == 0
    assert h.actions[-1] == original
    h.game.motion_feedback({"kind": "wooden_man_motion_feedback", "request_id": original["request_id"], "state": "running"})
    assert h.game.phase == "robot_action"
    h.complete()
    assert h.game.phase == "countdown" and h.words[-1] == "游戏开始，五秒后开始。"


def test_full_three_round_host_sequence_and_voice_replay():
    h = Host()
    h.watch()
    for round_number in range(1, 4):
        h.still()
        if round_number < 3:
            assert h.actions[-1]["gesture"] == "round"
            # Movement and camera shake during the arm gesture never count.
            h.game.pose(pose(70))
            h.complete()
            assert h.game.phase == "cue" and h.game.round == round_number + 1
            h.step(3.5)
        else:
            assert h.game.result == "won" and h.actions[-1]["gesture"] == "celebrate"
            h.complete()
            assert h.game.phase == "won" and h.game.round == 3
    h.step(3.01)
    assert h.game.replay_prompted and "再挑战一次" in h.words[-1]
    previous_session = h.game.session
    h.game.text("再来")
    assert h.game.phase == "preparing" and h.game.session != previous_session


def test_part_speech_highlight_and_point_use_the_same_frozen_frame():
    h = Host()
    h.watch()
    h.game.pose(pose())
    for _ in range(3):
        h.step(.1)
        h.game.pose(pose(20, joint=9))
    assert h.words[-1] == "你的左手臂动了，游戏结束。"
    assert h.game.result == "lost" and h.game.moved_part == "left_arm"
    request = h.actions[-1]
    assert request["gesture"] == "point"
    # Highlight is left elbow/wrist (indices 7,9); use its centre, not a
    # different person's position or an anatomical-left/right shortcut.
    assert request["target"]["x"] == pytest.approx((121 + 147) / 2 / 320)
    frame = h.game.frame
    h.game.pose(pose(100))
    assert h.game.frame is frame
    h.complete()
    assert h.game.phase == "lost"
    h.step(3.1)
    h.game.text("不玩了")
    assert h.game.phase == "waiting" and not h.game.moved_part


def test_fault_or_timeout_never_opens_a_watch_window():
    for state in ("error", "cancelled", "timeout"):
        h = Host()
        h.game.text("游戏开始")
        h.ready()
        rid = h.game.pending_motion["request_id"]
        if state == "timeout":
            h.step(31)
            assert h.actions[-1]["kind"] == "wooden_man_motion_cancel"
        else:
            h.game.motion_feedback({"kind": "wooden_man_motion_feedback", "request_id": rid, "state": state})
        assert h.game.phase == "error" and h.game.round == 0


def test_reset_cancels_in_flight_request_and_rejects_late_feedback():
    h = Host()
    h.game.text("游戏开始")
    h.ready()
    rid = h.game.pending_motion["request_id"]
    h.game.reset()
    assert h.actions[-1]["kind"] == "wooden_man_motion_cancel"
    h.game.motion_feedback({"kind": "wooden_man_motion_feedback", "request_id": rid, "state": "completed"})
    assert h.game.phase == "waiting"


def test_motion_ports_and_registration_are_canvas_compatible(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT))
    plugin = motion.make_plugin({}, "q5", None, None)
    plugin.start()
    assert plugin.process is None
    tool = plugin.get_tool()
    game = game_module.WoodenManPlugin({}, "q5")
    commands = next(p for p in game.get_tool()["topic_out"] if p["topic"].endswith("/actions"))
    assert tool["topic_in"][0]["format"] == commands["format"] == "data/json"
    assert game.configured_topics["motion_feedback_topic"] == tool["topic_out"][0]["topic"]
    assert plugin.dispatch("start", {})["state"] == "error"
    assert plugin.dispatch("config", {})["status"] == "configured"
    assert plugin.dispatch("info", {})["state"] == "idle"


@pytest.mark.parametrize("x,side", [(.1, "left"), (.5, "right"), (.9, "right")])
@pytest.mark.parametrize("y,row", [(.1, "upper"), (.5, "middle"), (.9, "lower")])
def test_camera_sector_maps_to_only_arms_with_urdf_valid_targets(monkeypatch, x, side, y, row):
    monkeypatch.syspath_prepend(str(ROOT))
    from joint_limits import JOINT_LIMITS
    sector, positions = motion.trajectory({"gesture": "point", "target": {"x": x, "y": y}})
    assert sector.startswith(row) and all(k.startswith(side) for k in positions)
    assert len(positions) == 3
    assert all(JOINT_LIMITS[k][0] <= v <= JOINT_LIMITS[k][1] for k, v in positions.items())
    if x != .5:
        _, mirrored = motion.trajectory({"gesture": "point", "target": {"x": x, "y": y}}, True)
        assert all(k.startswith("right" if side == "left" else "left") for k in mirrored)


class FakeClient:
    q5_position_control_prepared = True
    fresh = True
    def __init__(self):
        self.joints = {f"{s}_{j}": v for s in ("left", "right") for j, v in zip(motion.JOINT_SUFFIXES, (-math.pi/6, 0, -.2))}
        self.joints.update({f"{s}_{j}": 0. for s in ("left", "right") for j in motion.PASSIVE_SUFFIXES})
    def snapshot(self):
        return {"fresh": self.fresh, "age_ms": getattr(self, "age", 0), "joints": dict(self.joints)}
    def get_lifecycle_state(self):
        return "active"
    def sensor_snapshot(self, _):
        return {"available": True, "fresh": True, "age_ms": 0, "state": 4}


class FakeRouter:
    def __init__(self, client):
        self.client = client
        self.owner = None
        self.writes = []
        self.follow = True
        self.other = []
    def status(self):
        return {"ros_publisher_available": True, "endpoint_query_available": True,
                "other_publishers": self.other, "active_owner": self.owner, "same_name_publisher_count": 1}
    def acquire(self, owner):
        if self.owner is not None:
            return False
        self.owner = owner
        return True
    def release(self, _):
        self.owner = None
    def publish(self, point):
        self.writes.append(dict(point))
        if self.follow:
            self.client.joints.update(point)
        return True


@pytest.fixture
def runtime(monkeypatch):
    monkeypatch.syspath_prepend(str(ROOT))
    clock = SimpleNamespace(now=0., renew=True, cancel_at=None, stale_at=None)
    client = FakeClient()
    router = FakeRouter(client)
    reports = []
    runner = motion.MotionRunner(client, router, reports.append)
    class Event:
        flag = False
        def is_set(self):
            return self.flag
        def set(self):
            self.flag = True
        def wait(self, seconds):
            clock.now += seconds
            if clock.renew:
                runner.lease = clock.now
            if clock.cancel_at is not None and clock.now >= clock.cancel_at:
                self.flag = True
            if clock.stale_at is not None and clock.now >= clock.stale_at:
                client.fresh = False
    class Thread:
        def __init__(self, target, args, **_):
            self.target, self.args = target, args
        def start(self):
            self.target(*self.args)
        def is_alive(self):
            return False
        def join(self, **_):
            pass
    monkeypatch.setattr(motion, "threading", SimpleNamespace(RLock=threading.RLock, Event=Event, Thread=Thread))
    monkeypatch.setattr(motion, "time", SimpleNamespace(monotonic=lambda: clock.now, time=lambda: 1700000000 + clock.now))
    return runner, client, router, reports, clock


def request(**extra):
    return {"kind": "wooden_man_motion_request", "request_id": "session:1", "issued_at": 1700000000., "gesture": "welcome", **extra}


def test_measured_return_duplicate_dedup_and_bounded_interpolation(runtime):
    runner, client, router, reports, clock = runtime
    original = dict(client.joints)
    runner.submit(request())
    assert reports[-1]["state"] == "completed"
    assert client.joints == pytest.approx(original) and router.owner is None
    assert all(k.startswith("right_") for point in router.writes for k in point)
    for a, b in zip(router.writes, router.writes[1:]):
        assert all(abs(b[k] - a[k]) <= .010001 for k in b)
    count = len(router.writes)
    runner.submit(request())
    assert len(router.writes) == count and reports[-1]["state"] == "completed"


@pytest.mark.parametrize("fault", ["unprepared", "stale", "competition", "lease", "cancel", "no_arrival", "excursion", "yaw", "age", "mid_stale", "owner"])
def test_motion_failure_never_claims_completion(runtime, fault):
    runner, client, router, reports, clock = runtime
    if fault == "unprepared":
        client.q5_position_control_prepared = False
    elif fault == "stale":
        client.fresh = False
    elif fault == "competition":
        router.other = [{"node_name": "another_controller"}]
    elif fault == "lease":
        clock.renew = False
    elif fault == "cancel":
        clock.cancel_at = .5
    elif fault == "no_arrival":
        router.follow = False
    elif fault == "excursion":
        client.joints["right_shoulder_pitch_joint"] = 2.
    elif fault == "yaw":
        client.joints["right_arm_yaw_joint"] = 1.
    elif fault == "age":
        client.age = 501
    elif fault == "mid_stale":
        clock.stale_at = .5
    elif fault == "owner":
        router.owner = "another_card"
    runner.submit(request())
    assert reports[-1]["state"] in ("error", "cancelled")
    assert router.owner == ("another_card" if fault == "owner" else None)
    if fault in ("unprepared", "stale", "competition", "excursion", "yaw", "age", "owner"):
        assert not router.writes


def test_cancel_before_delivery_tombstones_request_and_invalid_commands_do_not_move(runtime):
    runner, client, router, reports, clock = runtime
    runner.submit({"kind": "wooden_man_motion_cancel", "request_id": "session:1"})
    runner.submit(request())
    assert reports[-1]["state"] == "cancelled" and not router.writes
    runner.submit(request(request_id="invalid-target", gesture="point", target={"x": float("nan"), "y": .5}))
    assert reports[-1]["state"] == "error" and not router.writes
    runner.submit(request(request_id="old", issued_at=1699999900.))
    assert reports[-1]["state"] == "error" and not router.writes
