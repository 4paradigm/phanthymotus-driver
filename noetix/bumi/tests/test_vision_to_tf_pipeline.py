"""End-to-end contract between the two cards delivered together.

vision_tf converts vision-localisation JSON into tf-shaped edges; tf_tree consumes
those edges and publishes them on /tf_static (parent chain) and /tf (targets).
The two cards were authored and reviewed in isolation, so the seam — the JSON
dict they exchange — is exactly where a regression can hide without any single
card's own tests going red. These tests pin that seam.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

BUMI = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUMI))

import vision_tf_spec as vspec  # noqa: E402
import tf_tree_spec as tspec  # noqa: E402


def _vision_cfg(**overrides):
    cfg = vspec.validate_config({
        "adapter": "pose",
        "input_frame": "optical",
        "output_frame": "optical",
        "source_name": "ar",
        "units": "m",
        "ttl_ms": 500,
    })
    cfg.update(overrides)
    return cfg


def test_vision_tf_output_is_consumed_by_tf_tree_without_rejection():
    """The default vision_tf edge must pass tf_tree's merge_vision unchanged."""
    batch = vspec.transforms_from_json(
        {"marker_id": 17, "position": [0.412, -0.108, 1.352],
         "orientation": [0.0, 0.7071068, 0.0, 0.7071068]},
        _vision_cfg(), receive_ms=10_000, targets={})

    live, report = tspec.merge_vision(batch["transforms"], 10_000, 500)

    assert len(live) == 1
    assert report == {"unknown_parent": 0, "invalid_child": 0, "expired": 0}
    edge = live[0]
    assert edge["parent"] == "camera_optical_frame"
    assert edge["child"] == "vision_ar_17"
    assert edge["translation"] == [0.412, -0.108, 1.352]


def test_all_vision_tf_parent_choices_are_in_tf_tree_whitelist():
    """Every parent frame vision_tf can emit must be accepted by tf_tree."""
    for input_frame, output_frame, expected in (
            ("optical", "optical", "camera_optical_frame"),
            ("body", "optical", "camera_link"),
            ("base", "optical", "base_link"),
            ("optical", "body", "camera_link")):
        batch = vspec.transforms_from_json(
            {"marker_id": 1, "position": [0.1, 0.2, 0.5],
             "orientation": [0.0, 0.0, 0.0, 1.0]},
            _vision_cfg(input_frame=input_frame, output_frame=output_frame),
            receive_ms=10_000, targets={})
        parent = batch["transforms"][0]["parent"]
        assert parent == expected
        live, report = tspec.merge_vision(batch["transforms"], 10_000, 500)
        assert len(live) == 1, (input_frame, output_frame, report)
        assert report["unknown_parent"] == 0


def test_camera_fixed_chain_is_published_on_tf_static():
    """The camera mount added in the URDF is what connects vision to the body.

    tf_tree reads those two fixed joints from the URDF and publishes them as the
    static chain base_link -> camera_link -> camera_optical_frame. Without them,
    the vision target edges above would be a floating, disconnected tree.
    """
    urdf = (BUMI / "resource" / "bumi_model.urdf").read_text(encoding="utf-8")
    tree = tspec.parse_urdf(urdf)

    fixed = {j["child"]: j for j in tree["joints"].values()
             if j["type"] == "fixed"}
    assert set(fixed) == {"camera_link", "camera_optical_frame"}
    assert fixed["camera_link"]["parent"] == "base_link"
    assert fixed["camera_optical_frame"]["parent"] == "camera_link"
    # Optical->body rotation must be the documented rpy (-pi/2, 0, -pi/2).
    rpy = fixed["camera_optical_frame"]["origin"][1]
    assert abs(rpy[0] + 1.5707963267948966) < 1e-9
    assert abs(rpy[2] + 1.5707963267948966) < 1e-9


def test_full_chain_from_vision_target_to_body_root():
    """Compose the published transforms: a vision target is reachable from root."""
    urdf = (BUMI / "resource" / "bumi_model.urdf").read_text(encoding="utf-8")
    tree = tspec.parse_urdf(urdf)
    poses = tspec.forward_kinematics(tree, {})

    batch = vspec.transforms_from_json(
        {"marker_id": 17, "position": [0.412, -0.108, 1.352],
         "orientation": [0.0, 0.0, 0.0, 1.0]},
        _vision_cfg(), receive_ms=10_000, targets={})
    live, _ = tspec.merge_vision(batch["transforms"], 10_000, 500)

    # Every fixed frame in the URDF has a root-relative pose, and the vision
    # edge's parent is one of them -> the target is connected, not floating.
    assert "camera_optical_frame" in poses
    assert live[0]["parent"] in poses
    assert "base_link" in poses
