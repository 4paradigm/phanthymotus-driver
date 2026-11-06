"""Motion geometry and command-boundary tests; no ROS or robot connection."""

import importlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest


BUMI = Path(__file__).resolve().parents[1]


@pytest.fixture
def modules(monkeypatch):
    monkeypatch.syspath_prepend(str(BUMI))
    return importlib.import_module("detaforce_motion"), importlib.import_module("detaforce_rps")


class Controller:
    mode = 2

    def get_mode(self):
        return self.mode


class Playback:
    def __init__(self, ctrl):
        self.ctrl = ctrl
        self.calls = []
        self.moving = False
        self.confirm_start = True

    def is_moving(self):
        return self.moving

    def dispatch(self, action, args):
        self.calls.append((action, args.copy()))
        assert args["_tool_name"] == "action_recording"
        if action == "play_recording":
            if self.confirm_start:
                self.ctrl.mode = 23
            return {"state": "running" if self.confirm_start else "accepted",
                    "command_sent": True, "workmode": self.ctrl.mode}
        assert action == "stop_playback"
        self.ctrl.mode = 2
        return {"state": "completed", "command_sent": True, "confirmed": True, "workmode": 2}


def plugin(module, **config):
    ctrl = Controller()
    backend = Playback(ctrl)
    return module.DetaforceRpsPlugin(config, backend, ctrl), ctrl, backend


def frame_points(pose):
    """Independent FK using the source URDF, including its nonzero joint origins."""
    def rotation(axis, angle):
        axis = np.asarray(axis, dtype=float)
        axis /= np.linalg.norm(axis)
        x, y, z = axis
        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
        result = np.eye(4)
        result[:3, :3] = np.eye(3)+math.sin(angle)*cross+(1-math.cos(angle))*(cross@cross)
        return result

    frames = {"base_link": np.eye(4)}
    pending = list(ET.parse(BUMI/"resource/bumi_model.urdf").getroot().findall("joint"))
    while pending:
        ready = [j for j in pending if j.find("parent").get("link") in frames]
        assert ready, "URDF contains an unresolved joint chain"
        for joint in ready:
            origin = joint.find("origin")
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
            roll, pitch, yaw = [float(v) for v in origin.get("rpy", "0 0 0").split()]
            transform = rotation([0, 0, 1], yaw)@rotation([0, 1, 0], pitch)@rotation([1, 0, 0], roll)
            transform[:3, 3] = xyz
            axis = [float(v) for v in joint.find("axis").get("xyz").split()]
            frames[joint.find("child").get("link")] = (
                frames[joint.find("parent").get("link")]@transform@rotation(axis, pose.get(joint.get("name"), 0)))
            pending.remove(joint)
    return {side: (frames[f"{side}_elbow_pitch_link"]@np.array([0, 0, -.16, 1]))[:3]
            for side in ("l", "r")}


@pytest.mark.parametrize("action", ["invite_one", "invite_both", "countdown", "double_punch", "mouse", "confident"])
def test_motion_is_arm_only_smooth_bounded_and_returns_to_rest(modules, action):
    motion, _ = modules
    clip = motion.build_motion(action, tempo=1.25, cycles=3)
    urdf = ET.parse(BUMI/"resource/bumi_model.urdf").getroot()
    limits = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
              for j in urdf.findall("joint")}
    step = .002
    times = np.linspace(0, clip.duration_s, math.ceil(clip.duration_s/step)+1)
    samples = np.array([[clip.sample(t)[0][name] for name in motion.ARM_JOINTS] for t in times])
    for index, name in enumerate(motion.ARM_JOINTS):
        assert samples[:, index].min() >= limits[name][0]
        assert samples[:, index].max() <= limits[name][1]
    dt = times[1]-times[0]
    velocity = np.diff(samples, axis=0)/dt
    acceleration = np.diff(velocity, axis=0)/dt
    assert abs(velocity).max() <= motion.MAX_VELOCITY+1e-3
    assert abs(acceleration).max() <= motion.MAX_ACCELERATION+1e-2
    assert set(clip.sample(0)[0]) == set(motion.ARM_JOINTS)
    assert np.allclose(samples[[0, -1]], 0)


def test_hand_paths_match_invitation_rotation_and_front_back_punch(modules):
    motion, _ = modules
    invite = motion.build_motion("invite_one", cycles=1)
    out = frame_points(invite.sample(invite.keyframes[1].time_s)[0])
    back = frame_points(invite.sample(invite.keyframes[2].time_s)[0])
    assert out["r"][0] > back["r"][0]+.03
    assert back["r"][2] > out["r"][2]
    assert np.allclose(out["l"], back["l"])
    punch = motion.build_motion("double_punch", cycles=1)
    left = frame_points(punch.sample(punch.keyframes[1].time_s)[0])
    right = frame_points(punch.sample(punch.keyframes[2].time_s)[0])
    assert left["l"][0] > left["r"][0]+.03
    assert right["r"][0] > right["l"][0]+.03
    orbit = motion.build_motion("countdown", cycles=1)
    points = [frame_points(orbit.sample(f.time_s)[0]) for f in orbit.keyframes[1:10]]
    heights = [p["l"][2]-p["r"][2] for p in points]
    assert min(heights) < -.04 and max(heights) > .04
    assert min(np.linalg.norm(p["l"]-p["r"]) for p in points) > .10


@pytest.mark.parametrize("action", ["invite_one", "invite_both"])
def test_invitation_reaches_face_height_twice(modules, action):
    clip = modules[0].build_motion(action)
    reaches = [frame_points(clip.sample(frame.time_s)[0]) for frame in clip.keyframes
               if frame.phase == "reach"]
    beckons = [frame_points(clip.sample(frame.time_s)[0]) for frame in clip.keyframes
               if frame.phase == "beckon_to_face"]
    assert len(reaches) == len(beckons) == 2
    active_sides = ("l", "r") if action == "invite_both" else ("r",)
    for reach, beckon in zip(reaches, beckons):
        for side in active_sides:
            # Coordinates are relative to the pelvis in the bundled URDF.
            assert beckon[side][2] > .38
            assert .10 < beckon[side][0] < .14
            assert .05 < abs(beckon[side][1]) < .10
            assert reach[side][0] - beckon[side][0] > .10
            assert beckon[side][2] > reach[side][2] + .025
    if action == "invite_one":
        rest = frame_points({})["l"]
        assert all(np.allclose(frame_points(clip.sample(frame.time_s)[0])["l"], rest)
                   for frame in clip.keyframes)


def test_mouse_hands_are_raised_beside_face(modules):
    clip = modules[0].build_motion("mouse")
    raised = next(frame for frame in clip.keyframes if frame.phase == "mouse_hold")
    hands = frame_points(clip.sample(raised.time_s)[0])
    assert min(hands[side][2] for side in ("l", "r")) > .42
    assert hands["l"][1] > .12
    assert hands["r"][1] < -.12


def test_hualai_makes_one_downstroke_then_restores_raised_fist(modules):
    clip = modules[0].build_motion("confident")
    frames = {frame.phase: frame for frame in clip.keyframes}
    assert sum(frame.phase == "affirm_downstroke" for frame in clip.keyframes) == 1
    points = {phase: frame_points(clip.sample(frames[phase].time_s)[0])
              for phase in ("fist_before_face", "affirm_downstroke", "restore_fist")}
    raised = points["fist_before_face"]["r"]
    lowered = points["affirm_downstroke"]["r"]
    assert raised[2] > .38
    assert .04 < raised[2] - lowered[2] < .10
    assert np.linalg.norm(raised[:2] - lowered[:2]) < .04
    np.testing.assert_allclose(points["restore_fist"]["r"], raised)
    rest = frame_points({})["l"]
    assert all(np.allclose(points[phase]["l"], rest) for phase in points)


@pytest.mark.parametrize("kwargs", [{"cycles": True}, {"cycles": 0}, {"cycles": 4}, {"tempo": float("nan")}, {"tempo": True}, {"tempo": 2}])
def test_invalid_motion_options_are_rejected(modules, kwargs):
    with pytest.raises(ValueError):
        modules[0].build_motion("countdown", **kwargs)


def test_boot_info_config_and_unbound_actions_never_move(modules):
    motion, card = modules
    p, _, backend = plugin(card)
    p.start()
    for action in ("info", "start", "status", "stop", *motion.LABELS):
        p.dispatch(action, {})
    assert p.dispatch("config", {"hardware_enabled": True})["state"] == "configured"
    assert p.dispatch("countdown", {})["state"] == "unavailable"
    p.stop()
    assert not backend.calls


def test_verified_recording_is_used_without_inventing_completion(modules):
    _, card = modules
    p, ctrl, backend = plugin(card, hardware_enabled=True, countdown_recording_id=37)
    result = p.dispatch("countdown", {})
    assert result["state"] == "running" and result["recording_id"] == 37
    assert not result["motion_completion_verified"]
    assert backend.calls == [("play_recording", {"_tool_name": "action_recording", "recording_id": 37})]
    assert p.dispatch("countdown", {})["command_sent"] is False
    assert p.dispatch("config", {"countdown_recording_id": 38})["state"] == "error"
    ctrl.mode = 2
    assert p.dispatch("status", {})["state"] == "playback_exited"
    p.stop()
    assert len(backend.calls) == 1


@pytest.mark.parametrize("mode", [0, 1, 3, 8, 23, 26])
def test_wrong_workmode_never_automatically_prepares_or_moves(modules, mode):
    p, ctrl, backend = plugin(modules[1], hardware_enabled=True, mouse_recording_id=12)
    ctrl.mode = mode
    assert p.dispatch("mouse", {})["command_sent"] is False
    assert not backend.calls


def test_moving_robot_and_extra_parameters_are_rejected(modules):
    p, _, backend = plugin(modules[1], hardware_enabled=True, mouse_recording_id=12)
    backend.moving = True
    assert not p.dispatch("mouse", {})["command_sent"]
    backend.moving = False
    assert not p.dispatch("mouse", {"recording_id": 7})["command_sent"]
    assert not backend.calls


def test_stop_only_exits_this_cards_playback_and_is_idempotent(modules):
    p, _, backend = plugin(modules[1], hardware_enabled=True, mouse_recording_id=12)
    assert not p.dispatch("stop_action", {})["command_sent"]
    p.dispatch("mouse", {})
    assert p.dispatch("stop_action", {})["state"] == "stopped"
    p.dispatch("stop_action", {})
    assert [action for action, _ in backend.calls] == ["play_recording", "stop_playback"]


def test_unconfirmed_start_is_not_forgotten_or_retried(modules):
    p, ctrl, backend = plugin(modules[1], hardware_enabled=True, mouse_recording_id=12)
    backend.confirm_start = False
    assert p.dispatch("mouse", {})["state"] == "start_unconfirmed"
    assert not p.dispatch("mouse", {})["command_sent"]
    assert not p.dispatch("stop_action", {})["command_sent"]
    assert len(backend.calls) == 1
    ctrl.mode = 23
    assert p.dispatch("status", {})["state"] == "running"
    assert p.dispatch("stop_action", {})["state"] == "stopped"


@pytest.mark.parametrize("value", [True, 1.5, "7", -2, 65536, float("nan")])
def test_recording_config_is_strict_and_transactional(modules, value):
    p, _, backend = plugin(modules[1])
    result = p.dispatch("config", {"hardware_enabled": True, "mouse_recording_id": value})
    assert result["state"] == "error"
    assert not p.dispatch("info", {})["hardware_enabled"]
    assert not backend.calls


def test_tool_contract_and_interrupt_are_shared_with_preview(modules):
    motion, card = modules
    for simulation in (False, True):
        spec = card.tool_spec(simulation=simulation)
        actions = spec["inputSchema"]["x-action-params"]
        assert set(actions) == set(motion.LABELS) | {"status", "stop_action"}
        assert spec["inputSchema"]["x-hooks"]["on_interrupt_motion"]["action"] == "stop_action"
        assert "x-completion" not in spec["inputSchema"]
