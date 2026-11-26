import math
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

import vision_pick_and_drop as pick
from pick_ik import plan


def test_pixel_to_base_and_image_relative_displacement():
    transform = np.eye(4)
    transform[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    transform[:3, 3] = [.2, .3, 0.]
    photo = {"depth": np.full((8, 10), 1000), "K": np.diag([100, 100, 1]), "transform": transform}
    args = dict(start_point_x=0, start_point_y=0, delta_x=30, delta_y=-20)
    points, _, details = pick.resolve_targets("grab_by", args, photo, {"xy_compensation_m": [.01, 0.]})
    assert points[0] == pytest.approx([.17, .35])
    assert points[1] == pytest.approx([.19, .38])
    assert details["pick_pixel"] == [5, 4]
    photo["depth"][4, 5] = 0
    with pytest.raises(ValueError, match="valid depth"):
        pick.resolve_targets("grab_by", args, photo, {"xy_compensation_m": [0., 0.]})


def test_motion_confirmation_and_uncommissioned_default_never_reserve_arm():
    device = Mock()
    card = pick.VisionPickAndDrop(device, {"allow_motion": False})
    assert card.dispatch("observe", {})["code"] == "CONFIRMATION_REQUIRED"
    with pytest.raises(ValueError, match="not commissioned"):
        card.dispatch("observe", {"confirm_motion": True})
    device.reserve_pick.assert_not_called()
    device.arm.assert_not_called()


def test_failure_after_close_keeps_ownership_and_reports_possible_object(monkeypatch):
    device = Mock()
    card = pick.VisionPickAndDrop(device, {})
    card._base_pose = Mock(return_value=np.zeros(6))
    card._same_base = Mock()
    card._joints = Mock(return_value={})
    card._plan = Mock(return_value=([], [("open", None), ("close", None), ("lift", [[]]), ("release", None)]))
    card._grip = Mock()
    card._move = Mock(side_effect=RuntimeError("controller failed"))
    notify = Mock()
    monkeypatch.setattr(pick, "_notify_completion", notify)
    snapshot = dict(expiry=time.monotonic() + 1, joints={}, base=np.zeros(6), summary={"observation_id": "one"})
    card._run("grab_by", "test-action", snapshot, ([], 0, {}))
    assert [c.args[0] for c in card._grip.call_args_list] == ["open", "close"]
    assert card.info()["state"] == "blocked"
    device.release_pick.assert_not_called()
    _, status, result, _ = notify.call_args.args
    assert status == "error" and result["recovery_required"] and result["holding_object_possible"]
    assert not result["release_completed"]


def test_arm_waypoint_uses_existing_p2p_and_waits_for_success():
    device = Mock()
    card = pick.VisionPickAndDrop(device, {"side": "left"})
    card._same_base = Mock()
    device._get.return_value = (SimpleNamespace(name=["j"], position=[math.radians(2)]), time.monotonic())
    device.arm.side_effect = lambda args, owner, completion: completion("completed", {"controllers_restored": True})
    card._move(["j"], [2.], np.zeros(6))
    assert device.arm.call_args.args[0] == {"action": "move", "side": "left", "positions_deg": [2.]}


def test_pink_plans_a_tabletop_cycle_and_rejects_work_boundary():
    pytest.importorskip("pink")
    pytest.importorskip("daqp")
    axes = ("1 0 0", "0 1 0", "0 0 1", "1 0 0", "0 1 0", "0 0 1", "0 0 1", "0 0 1")
    urdf = ['<robot name="fixture"><link name="base_link"/>']
    previous = "base_link"
    names = [f"j{i}" for i in range(8)]
    for i, (name, axis) in enumerate(zip(names, axes)):
        child = f"link{i}"
        kind = "prismatic" if i < 3 else "revolute"
        urdf.append(f'<link name="{child}"/><joint name="{name}" type="{kind}">'
                    f'<parent link="{previous}"/><child link="{child}"/><axis xyz="{axis}"/>'
                    '<limit lower="-2" upper="2" effort="10" velocity="2"/></joint>')
        previous = child
    urdf.append(f'<link name="tcp"/><joint name="tool" type="fixed"><parent link="{previous}"/>'
                '<child link="tcp"/><origin xyz="0 0 0.5" rpy="3.141592653589793 0 0"/></joint></robot>')
    cfg = {"tcp_frame": "tcp", "base_frame": "base_link", "workspace_min_m": [-.1, -.1, .3],
           "workspace_max_m": [.1, .1, .7], "observation_joints_deg": [0.] * 8,
           "pick_descent_m": .04, "place_descent_m": .03}
    req = dict(urdf="".join(urdf), action="grab_to", config=cfg, names=names,
               measured=dict.fromkeys(names, 0.), limits=dict.fromkeys(names, [-2, 2]),
               targets=[[.02, .01], [-.03, -.01]], rotation_deg=15., max_delta_deg=20.)
    assert [name for name, _ in plan(req)] == ["approach", "open", "descend_pick", "close", "lift",
                                               "rotate_gripper", "transfer", "descend_place", "release",
                                               "retract", "return_observation"]
    req["targets"][1] = [.2, 0.]
    with pytest.raises(ValueError, match="reach|workspace"):
        plan(req)
