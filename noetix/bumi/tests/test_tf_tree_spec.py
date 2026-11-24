"""Tests for the tf_tree card's dependency-free maths layer.

The two bugs these tests exist to pin:

1. A TF transform is parent→child **local** to the parent frame. Publishing the
   root-relative FK pose with a parent frame_id misplaces every joint below the
   first one. `joint_local_pose` is the fix; the "absolute" comparison below
   documents how wrong the old behaviour was.
2. Vision transforms carry their own stamp. An expired one must be rejected,
   not silently republished as fresh.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import tf_tree_spec as spec  # noqa: E402


URDF = """<?xml version="1.0"?>
<robot name="test">
  <link name="base_link"/>
  <link name="a"/>
  <link name="b"/>
  <joint name="j1" type="fixed">
    <parent link="base_link"/><child link="a"/>
    <origin xyz="0.1 0.2 0.3" rpy="0 0 0"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="a"/><child link="b"/>
    <origin xyz="0.4 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
</robot>
"""


# ── URDF parsing ─────────────────────────────────────────────────────────────


def test_parses_links_and_joints():
    tree = spec.parse_urdf(URDF)
    assert tree["root"] == "base_link"
    assert set(tree["links"]) == {"base_link", "a", "b"}
    assert tree["joints"]["j1"]["type"] == "fixed"
    assert tree["joints"]["j2"]["axis"] == (0.0, 0.0, 1.0)


@pytest.mark.parametrize("bad, fragment", [
    (URDF.replace("</robot>", "</robot> extra"), "not well-formed"),
    (URDF.replace('<link name="a"/>', ""), "unknown child"),
    (URDF.replace('type="revolute"', 'type="continuous"'), "does not implement"),
    (URDF.replace('<axis xyz="0 0 1"/>', ""), "missing <axis>"),
])
def test_rejects_malformed_urdf(bad, fragment):
    with pytest.raises(ValueError, match=fragment):
        spec.parse_urdf(bad)


def test_rejects_two_roots():
    bad = URDF.replace('<link name="b"/>', '<link name="b"/><link name="orphan"/>')
    with pytest.raises(ValueError, match="exactly one root"):
        spec.parse_urdf(bad)


# ── the local-vs-absolute bug ────────────────────────────────────────────────


def test_local_pose_is_not_root_relative():
    tree = spec.parse_urdf(URDF)
    joint = tree["joints"]["j2"]
    xyz, _ = spec.joint_local_pose(joint, 0.0)
    assert xyz == pytest.approx((0.4, 0.0, 0.0))
    # The root-relative FK pose is (0.5, 0.2, 0.3): publishing that with
    # frame_id="a" would put `b` 20 cm too far in y and 30 cm too high.
    absolute = spec.forward_kinematics(tree, {"j2": 0.0})["b"][0]
    assert absolute == pytest.approx((0.5, 0.2, 0.3))
    assert xyz != pytest.approx(absolute)


def test_local_pose_rotates_about_the_joint_axis():
    tree = spec.parse_urdf(URDF)
    joint = tree["joints"]["j2"]
    _, quat = spec.joint_local_pose(joint, math.pi / 2)
    # 90° about +z → (x,y,z,w) = (0, 0, sin45, cos45).
    assert quat == pytest.approx(
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)), abs=1e-9)


def test_fixed_and_revolute_local_xyz_are_independent_of_joint_values():
    tree = spec.parse_urdf(URDF)
    for q in (0.0, 0.7, -1.2):
        xyz, _ = spec.joint_local_pose(tree["joints"]["j2"], q)
        assert xyz == pytest.approx((0.4, 0.0, 0.0))


# ── forward kinematics ───────────────────────────────────────────────────────


def test_fk_composes_rotation_not_just_translation():
    tree = spec.parse_urdf(URDF)
    poses = spec.forward_kinematics(tree, {"j2": math.pi / 2})
    # URDF semantics: a revolute origin translation is a fixed mount offset,
    # so only the rotation changes with q — `b` stays at (0.5, 0.2, 0.3).
    assert poses["b"][0] == pytest.approx((0.5, 0.2, 0.3), abs=1e-9)


def test_fk_defaults_missing_joint_values_to_zero():
    tree = spec.parse_urdf(URDF)
    assert spec.forward_kinematics(tree, {}) == spec.forward_kinematics(
        tree, {"j2": 0.0})


def test_fk_on_shipped_urdf_covers_all_21_revolute_joints():
    urdf = (Path(__file__).resolve().parents[1] / "resource" / "bumi_model.urdf").read_text(
        encoding="utf-8")
    tree = spec.parse_urdf(urdf)
    poses = spec.forward_kinematics(tree, {})
    assert tree["root"] == "base_link"
    joints = [j for j in tree["joints"].values() if j["type"] == "revolute"]
    assert len(joints) == 21
    # The shipped URDF declares the camera mount as two fixed joints. They must
    # be parsed (and published on /tf_static) so vision targets hang off the
    # body tree instead of floating; forward kinematics must place both links.
    fixed = [j for j in tree["joints"].values() if j["type"] == "fixed"]
    assert {j["child"] for j in fixed} == {"camera_link", "camera_optical_frame"}
    assert all(j["child"] in poses for j in joints)
    assert all(j["child"] in poses for j in fixed)


# ── vision merge ─────────────────────────────────────────────────────────────


def _vision_entry(**overrides):
    entry = {
        "parent": "camera_optical_frame",
        "child": "person_head",
        "stamp_ms": 10000,
        "translation": [0.1, 0.2, 0.5],
        "rotation": [0.0, 0.0, 0.0, 1.0],
    }
    entry.update(overrides)
    return entry


def test_merge_vision_keeps_fresh_rejects_expired():
    live, report = spec.merge_vision(
        [_vision_entry(), _vision_entry(child="old", stamp_ms=8000)],
        10000, 500)
    assert [e["child"] for e in live] == ["person_head"]
    assert report["expired"] == 1


def test_merge_vision_rejects_unknown_parent_and_bad_child():
    live, report = spec.merge_vision(
        [_vision_entry(parent="mystery_frame"),
         _vision_entry(child="a/b"),
         _vision_entry(child="9bad")],
        10000, 500)
    assert live == []
    assert report["unknown_parent"] == 1
    assert report["invalid_child"] == 2


def test_merge_vision_treats_missing_stamp_as_fresh():
    live, report = spec.merge_vision(
        [_vision_entry(stamp_ms=None)], 999999, 500)
    assert len(live) == 1
    assert report["expired"] == 0


def test_merge_vision_rejects_far_future_stamp():
    """P1 regression: a clock-skewed/malformed future stamp must not stay live
    until wall time catches up, nor be broadcast as a future tf2 stamp."""
    live, report = spec.merge_vision(
        [_vision_entry(child="future", stamp_ms=10000 + 3_600_000)],
        10000, 500)
    assert live == []
    assert report["expired"] == 1


def test_merge_vision_accepts_small_future_skew():
    live, report = spec.merge_vision(
        [_vision_entry(child="skew", stamp_ms=10000 + 400)], 10000, 500)
    assert [e["child"] for e in live] == ["skew"]
    assert report["expired"] == 0


def test_merge_vision_rejects_malformed_payload():
    """P1 regression: a wired-in bad edge must never reach the ROS callback.

    `_vision_transform` indexes translation[0:3] and rotation[0:4] directly.
    A short vector raises IndexError in the subscription thread; a non-numeric
    or NaN/inf value produces an invalid TF that poisons every tf2 lookup; a
    zero quaternion is not a rotation. All must be rejected at the boundary.
    """
    bad = [
        _vision_entry(child="empty", translation=[], rotation=[]),
        _vision_entry(child="short_t", translation=[0.1, 0.2]),
        _vision_entry(child="short_r", rotation=[0.0, 0.0, 0.0]),
        _vision_entry(child="text", translation=["a", "b", "c"]),
        _vision_entry(child="nan", translation=[float("nan"), 0, 0]),
        _vision_entry(child="inf", rotation=[0, 0, 0, float("inf")]),
        _vision_entry(child="zero_q", rotation=[0.0, 0.0, 0.0, 0.0]),
    ]
    live, report = spec.merge_vision(bad, 10000, 500)
    assert live == []
    assert report["invalid_payload"] == len(bad)


def test_merge_vision_keeps_valid_payload_and_defaults_missing_fields():
    """A valid edge passes with finite values; absent vectors use identity."""
    live, report = spec.merge_vision(
        [_vision_entry(),
         _vision_entry(child="no_vectors", translation=None, rotation=None)],
        10000, 500)
    assert [e["child"] for e in live] == ["person_head", "no_vectors"]
    assert all(math.isfinite(v) for v in live[0]["translation"])
    # Absent fields keep the historical identity default.
    assert live[1]["translation"] == [0.0, 0.0, 0.0]
    assert live[1]["rotation"] == [0.0, 0.0, 0.0, 1.0]
    assert report["invalid_payload"] == 0


# ── consumer-side freshness check ────────────────────────────────────────────


def test_transform_is_fresh_rejects_stale_missing_future_and_nonfinite():
    """A downstream TF lookup must be validated before treating it as a pose.

    `/tf` cannot retract a published edge, so a cached transform can be stale;
    the consumer-side check is what makes expiry a real, verifiable property.
    """
    assert spec.transform_is_fresh(9500, 10000, 500) is True   # 500 ms old
    assert spec.transform_is_fresh(10_000, 10_000, 500) is True
    assert spec.transform_is_fresh(9400, 10000, 500) is False  # > ttl
    assert spec.transform_is_fresh(None, 10000, 500) is False
    assert spec.transform_is_fresh(float("nan"), 10000, 500) is False
    assert spec.transform_is_fresh(float("inf"), 10000, 500) is False
    # Future-dated beyond skew is unusable (clock disagreement).
    assert spec.transform_is_fresh(10_000 + 10_000, 10_000, 500) is False


# ── status payload ───────────────────────────────────────────────────────────


def test_status_payload_is_flat_and_reports_degradation():
    payload = spec.status_payload(
        state="running", stamp_ms=123, root_frame="base_link",
        link_count=24, dynamic_frames=21, static_frames=2,
        tf_hz=10.04, last_joint_age_ms=120, joints_stale=False,
        vision_targets=1, rejected_unknown_parent=0, expired_vision=2,
        conflicting_parent=0, degraded="joints_stale", last_error="")
    assert all(not isinstance(v, (dict, list)) for v in payload.values())
    assert payload["degraded"] == "joints_stale"
    assert payload["tf_hz"] == 10.0
    assert payload["expired_vision"] == 2
