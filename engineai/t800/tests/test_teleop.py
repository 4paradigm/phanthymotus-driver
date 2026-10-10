"""Pure model/session tests: no ROS runtime, network or hardware outputs."""
import copy
import json
from pathlib import Path
import sys
import threading

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parents[1]))

from common.teleop_contract import COMMAND_SCHEMA, TRACKING_FRAME, validate_feedback
from control import T800_JOINT_NAMES
from teleop_control import TeleopControl
from teleop_kinematics import DualArmModel, RelativeMapping, rotation, smooth_reference


Q = np.array([0., .25, 0., -.6, 0., 0., -.25, 0., -.6, 0.])


class Rig:
    def __init__(self, mode="live"):
        self.now = 1_000_000_000
        self.sequence = 0
        self.messages = []
        full = np.zeros(25)
        full[13:23] = Q
        self.snapshot = {"joints": {"age_sec": 0., "joints": [
            {"idx": i, "name": name, "q": float(full[i])} for i, name in enumerate(T800_JOINT_NAMES)]},
            "motion": {"age_sec": 0., "current_motion_task": "lower_body_balance"},
            "planner": {"age_sec": 0., "status": 1}}
        self.control = TeleopControl({"mode": mode}, lambda: self.snapshot,
            lambda q, v, w: self.messages.append((q.copy(), v.copy(), w)),
            clock=lambda: self.now, clock_id="host-boot")

    def frame(self, grip=0., dx=0., **changes):
        self.now += 10_000_000
        self.sequence += 1
        pose = {"tracked": True, "position": [0., 0., 1.], "orientation_xyzw": [0., 0., 0., 1.]}
        value = {"schema": COMMAND_SCHEMA, "kind": "input", "device_id": "pico-1",
            "instance_id": "pico_input", "clock_id": "host-boot", "sequence": self.sequence,
            "connection_epoch": 0, "space_epoch": 0, "received_monotonic_ns": self.now,
            "source_monotonic_ns": self.now-1_000_000, "tracking_frame": TRACKING_FRAME,
            "head_reference": copy.deepcopy(pose)}
        for side, sign in (("left", 1), ("right", -1)):
            value[side] = {**copy.deepcopy(pose), "position": [dx, .2*sign, 1.],
                "grip": grip, "trigger": .4,
                "controls": {"buttons": {"primary": {"available": True, "pressed": False}},
                             "axes": {"thumbstick": {"available": True, "value": [0., 0.]}}}}
        return {**value, **changes}

    def start(self):
        self.control.begin({"input_topic": "/teleop/command"})
        assert self.control.receive(self.frame())
        self.control.tick()

    def step(self, grip=1., dx=.1):
        assert self.control.receive(self.frame(grip, dx))
        self.control.solve_once()
        self.control.tick()
        # Test plant follows only successfully published command references.
        if self.messages and self.messages[-1][2] == 1:
            for i, q in enumerate(self.messages[-1][0]):
                self.snapshot["joints"]["joints"][13+i]["q"] = float(q)


def test_urdf_jacobian_matches_finite_difference_and_mirrored_arm_positions():
    model = DualArmModel()
    poses = model.poses(Q)
    np.testing.assert_allclose(poses[0][0] * [1, -1, 1], poses[1][0], atol=1e-10)
    for i, arm in enumerate(model.arms):
        q = Q[5*i:5*i+5].copy()
        p, _, jacobian, _ = arm.forward(q)
        for j in range(5):
            perturbed = q.copy()
            perturbed[j] += 1e-6
            np.testing.assert_allclose((arm.forward(perturbed)[0]-p)/1e-6, jacobian[:, j], atol=1e-6)


def test_ik_position_priority_reachable_and_unreachable_recovery():
    model = DualArmModel()
    poses = model.poses(Q)
    targets = [(p + [.05, 0., .02], r @ rotation([0, 0, 1], .3)) for p, r in poses]
    answer, residual = model.solve(targets, Q)
    assert max(residual) < .005
    far = [(p + [2., 0., 0.], r) for p, r in poses]
    boundary, errors = model.solve(far, answer)
    assert min(errors) > 1.
    assert np.isfinite(boundary).all()
    assert np.all(boundary >= model.lower) and np.all(boundary <= model.upper)
    # Warm-started recovery can take several bounded solves; there is no latch
    # that forces regrip or recalibration just because a hand was out of range.
    for _ in range(8):
        boundary, errors = model.solve(poses, boundary, reference=Q)
    assert max(errors) < .005


def test_mapping_removes_initial_head_yaw_and_uses_relative_positions():
    rig = Rig()
    frame = rig.frame()
    frame["head_reference"]["orientation_xyzw"] = [0., 0., 2**-.5, 2**-.5]
    poses = rig.control.model.poses(Q)
    mapping = RelativeMapping(frame, poses, .5)
    changed = copy.deepcopy(frame)
    for side in ("left", "right"):
        changed[side]["position"][1] += .2
    targets = mapping.targets(changed)
    np.testing.assert_allclose(targets[0][0]-poses[0][0], [.1, 0, 0], atol=1e-10)


def test_reference_velocity_acceleration_and_reversal_are_bounded():
    model = DualArmModel()
    q, v = Q.copy(), np.zeros(10)
    for i in range(300):
        target = Q + (.25 if i < 140 else -.1)
        next_q, next_v = smooth_reference(q, v, target, .01, model.lower, model.upper)
        assert np.max(np.abs(next_v)) <= .5 + 1e-10
        assert np.max(np.abs(next_v-v)) <= .020000001
        np.testing.assert_allclose(next_q-q, next_v*.01, atol=1e-12)
        q, v = next_q, next_v


def test_start_and_calibration_do_not_publish_and_shadow_never_publishes():
    rig = Rig()
    rig.start()
    assert rig.control.mapping is not None and not rig.messages
    rig.step()
    assert rig.messages[-1][2] == 1.
    shadow = Rig("shadow")
    shadow.start()
    for _ in range(4):
        shadow.step()
    assert not shadow.messages
    assert shadow.control.state == "following"
    assert not np.array_equal(shadow.control.q, Q)
    shadow.control.halt()
    assert not shadow.messages


@pytest.mark.parametrize("bad", ["partial", "wrong_mode", "planner_stale"])
def test_start_rejects_missing_readiness_without_outputs(bad):
    rig = Rig()
    if bad == "partial":
        rig.snapshot["joints"]["joints"] = rig.snapshot["joints"]["joints"][:13]
    elif bad == "wrong_mode":
        rig.snapshot["motion"]["current_motion_task"] = "pd_stand"
    else:
        rig.snapshot["planner"]["age_sec"] = None
    with pytest.raises(ValueError):
        rig.control.begin({"input_topic": "/teleop/command"})
    assert not rig.control.running and not rig.messages


def test_held_grips_at_start_require_release_and_repeated_start_is_idempotent():
    rig = Rig()
    rig.control.begin({"input_topic": "/teleop/command"})
    rig.step()
    assert not rig.messages and rig.control.mapping is None
    rig.step(grip=0.)
    mapping = rig.control.mapping
    rig.step()
    rig.control.begin({"input_topic": "/teleop/command"})
    assert rig.control.mapping is mapping


def test_release_one_grip_holds_and_regrip_preserves_anchor():
    rig = Rig()
    rig.start()
    rig.step()
    mapping = rig.control.mapping
    reference = rig.control.q.copy()
    value = rig.frame(grip=1., dx=.3)
    value["left"]["grip"] = 0.
    rig.control.receive(value)
    rig.control.tick()
    np.testing.assert_array_equal(reference, rig.messages[-1][0])
    np.testing.assert_array_equal(np.zeros(10), rig.messages[-1][1])
    rig.step(dx=.2)
    assert rig.control.mapping is mapping
    assert not np.array_equal(reference, rig.control.q)


@pytest.mark.parametrize("change", [
    {"clock_id": "another-host"}, {"device_id": "other-pico"},
    {"instance_id": "other_card"}, {"kind": "operation"},
    {"tracking_frame": "openxr"}, {"sequence": True},
    {"received_monotonic_ns": 0}, {"sequence": 1}, {"source_monotonic_ns": 1},
])
def test_invalid_or_replayed_input_cannot_replace_latest_frame(change):
    rig = Rig()
    rig.start()
    rig.step()
    latest = rig.control.frame
    assert not rig.control.receive(rig.frame(grip=1., **change))
    assert rig.control.frame is latest


def test_input_queued_before_project_start_is_not_admitted():
    rig = Rig()
    old = rig.frame()
    rig.now += 10_000_000
    rig.control.begin({"input_topic": "/teleop/command"})
    assert not rig.control.receive(old)


@pytest.mark.parametrize("failure", ["input_stale", "tracking_lost", "command_loop_gap", "solver_stale"])
def test_recoverable_hold_requires_release_before_resuming(failure):
    rig = Rig()
    rig.start()
    rig.step()
    mapping = rig.control.mapping
    if failure == "tracking_lost":
        value = rig.frame(grip=1.)
        value["right"].update(tracked=False, position=None, orientation_xyzw=None)
        rig.control.receive(value)
    elif failure == "input_stale":
        rig.now += 310_000_000
    elif failure == "command_loop_gap":
        rig.now += 60_000_000
    else:
        for _ in range(16):
            rig.control.receive(rig.frame(grip=1.))
            rig.control.tick()  # Fresh input but deliberately no solver progress.
    rig.control.tick()
    reference = rig.control.q.copy()
    assert not rig.control.release_seen
    rig.step()
    np.testing.assert_array_equal(reference, rig.control.q)
    rig.step(grip=0.)
    rig.step()
    assert rig.control.mapping is mapping
    assert rig.control.state == "following"


def test_reconnection_keeps_mapping_but_recenter_requires_new_released_anchor():
    rig = Rig()
    rig.start()
    rig.step()
    mapping = rig.control.mapping
    rig.control.receive(rig.frame(grip=1., connection_epoch=1))
    assert not rig.control.release_seen and rig.control.mapping is mapping
    rig.control.receive(rig.frame(grip=0., connection_epoch=1))
    rig.control.tick()
    rig.control.receive(rig.frame(grip=1., connection_epoch=1, space_epoch=1))
    assert rig.control.mapping is None
    assert not rig.control.receive(rig.frame(grip=0., connection_epoch=0, space_epoch=0))
    rig.control.receive(rig.frame(grip=0., connection_epoch=1, space_epoch=1))
    rig.control.tick()
    assert rig.control.mapping is not mapping


@pytest.mark.parametrize("failure", ["partial", "nan", "joints_stale", "motion_stale", "mode", "planner", "torso", "tracking"])
def test_robot_fault_releases_and_does_not_resume_automatically(failure):
    rig = Rig()
    rig.start()
    rig.step()
    if failure == "partial":
        rig.snapshot["joints"]["joints"].pop()
    elif failure == "nan":
        rig.snapshot["joints"]["joints"][13]["q"] = float("nan")
    elif failure in ("joints_stale", "motion_stale"):
        rig.snapshot[failure.split("_")[0]]["age_sec"] = 1.
    elif failure == "mode":
        rig.snapshot["motion"]["current_motion_task"] = "walk"
    elif failure == "planner":
        rig.snapshot["planner"]["status"] = 2
    elif failure == "torso":
        rig.snapshot["joints"]["joints"][12]["q"] = .2
    else:
        rig.snapshot["joints"]["joints"][13]["q"] += .4
    rig.now += 10_000_000
    rig.control.tick()
    assert rig.control.state == "error" and rig.messages[-1][2] == 0.
    assert rig.control.motion_active()
    assert not rig.control.receive(rig.frame(grip=0.))


def test_stop_invalidates_an_inflight_solve_and_no_late_command_is_published():
    rig = Rig()
    rig.start()
    rig.step()
    entered, finish = threading.Event(), threading.Event()
    original = rig.control.model.solve
    def blocked(targets, seed, **kwargs):
        entered.set()
        assert finish.wait(2)
        return original(targets, seed, **kwargs)
    rig.control.model.solve = blocked
    rig.control.receive(rig.frame(grip=1.))
    worker = threading.Thread(target=rig.control.solve_once)
    worker.start()
    assert entered.wait(1)
    rig.control.halt()
    assert rig.messages[-1][2] == 0.
    count = len(rig.messages)
    finish.set()
    worker.join(2)
    assert not worker.is_alive()
    rig.control.tick()
    assert len(rig.messages) == count and rig.control.solution is None


def test_publish_failure_retries_release_and_keeps_motion_reserved():
    rig = Rig()
    rig.start()
    original = rig.control.publish
    def failed(*args):
        raise RuntimeError("publisher disconnected")
    rig.control.publish = failed
    rig.step()
    assert rig.control.release_pending and rig.control.motion_active()
    with pytest.raises(ValueError, match="override_release_pending"):
        rig.control.begin({"input_topic": "/teleop/command"})
    rig.control.publish = original
    rig.control.tick()
    assert rig.messages[-1][2] == 0. and not rig.control.release_pending
    assert rig.control.state == "error"


def test_monitor_uses_shared_pico_contract():
    rig = Rig()
    rig.start()
    feedback = rig.control.feedback()
    validate_feedback(feedback, instance_id="pico_input", clock_id="host-boot", now_ns=rig.now)
    assert feedback["execution"]["mode"] == "live"
    rig.control.reason = "<script>"
    assert "<script>" not in rig.control.feedback()["text"]
    assert "&lt;script&gt;" in rig.control.feedback()["text"]


def test_failed_stop_release_retries_to_idle_without_rearming():
    rig = Rig()
    rig.start()
    rig.step()
    original = rig.control.publish
    rig.control.publish = lambda *args: (_ for _ in ()).throw(RuntimeError("offline"))
    assert rig.control.halt()["override_release_pending"]
    rig.control.publish = original
    rig.control.tick()
    assert not rig.control.motion_active()
    assert rig.control.state == "idle" and rig.messages[-1][2] == 0.


def test_accepts_pico_pr329_producer_fixture():
    # Unmodified producer fixture from PR329 77a9fa3a, Apache-2.0:
    # pico/4ultra/ext_vr/fixtures/input-v1.json. Change only local clock context.
    value = json.loads((ROOT / "tests/fixtures/pico-input-v1.json").read_text())
    rig = Rig()
    rig.now = value["received_monotonic_ns"]-1
    rig.control.clock_id = value["clock_id"]
    rig.control.begin({"input_topic": "/teleop/command"})
    rig.now += 1
    assert rig.control.receive(value)
    assert rig.control.source == "vr_1"
    rig.control.tick()
    assert not rig.messages  # Fixture grips are held; calibration needs release.
