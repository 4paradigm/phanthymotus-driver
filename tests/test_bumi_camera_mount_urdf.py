"""The Bumi camera mount frames — URDF vs config consistency, offline.

Vision results are expressed in a camera optical frame (x right / y down / z
forward) whose origin and attitude differ from the body tree rooted at
`base_link` (x forward / y left / z up). This task adds two frames to
`resource/bumi_model.urdf` — `camera_link` (body axes, origin at the optical
centre) and `camera_optical_frame` (optical axes), plus the `head_center_link`
datum they are measured against — and locks them against the existing
`calibration.camera_to_body` in `config.yaml`.

The whole point is that these two files must never drift: the URDF `rpy` is the
same rotation as the config quaternion, and the URDF `xyz` is the same
translation. The tests below are the regression lock — change either side alone
and they go red.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_bumi_camera_mount_urdf.py -q
"""

from __future__ import annotations

import ast
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
BUNDLE = ROOT / "noetix" / "bumi"
URDF_PATH = BUNDLE / "resource" / "bumi_model.urdf"
CONFIG_PATH = BUNDLE / "config.yaml"
DEVICE_PATH = BUNDLE / "device.py"

def _parse_urdf() -> ET.Element:
    return ET.parse(URDF_PATH).getroot()


def _joint_map() -> dict[str, ET.Element]:
    """joint name → <joint> element, from the URDF."""
    return {j.get("name"): j for j in _parse_urdf().findall("joint")}


def _bumi_joint_names_from_device_py() -> list[str]:
    """Extract `_BUMI_JOINT_NAMES` from device.py without importing rclpy.

    device.py imports rclpy at module top level, which only runs inside the
    robot container. We lift the literal list out of the AST instead.
    """
    tree = ast.parse(DEVICE_PATH.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == "_BUMI_JOINT_NAMES":
                    return ast.literal_eval(node.value)
    raise AssertionError("_BUMI_JOINT_NAMES not found in device.py")


def _rpy_to_quaternion(rpy: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """Fixed-axis RPY (x, y, z) → unit quaternion (x, y, z, w)."""
    r, p, y = rpy
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    x = sr * cp * cy - cr * sp * sy
    yy = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, yy, z, w)


def _parent_chain(name: str, joint_map: dict[str, ET.Element]) -> list[tuple[str, str]]:
    """[(child_link, joint_type), ...] walking up from `name` to the root."""
    chain = []
    current = name
    while True:
        joint = next(
            (j for j in joint_map.values()
             if j.find("child") is not None
             and j.find("child").get("link") == current),
            None,
        )
        if joint is None:
            break
        chain.append((current, joint.get("type")))
        current = joint.find("parent").get("link")
    return chain


# ── 1. the 21 joints are untouched ────────────────────────────────────────────

def test_urdf_still_declares_exactly_the_21_revolute_joints():
    joint_map = _joint_map()
    expected = _bumi_joint_names_from_device_py()

    # The only revolute joints are exactly the 21 declared in device.py —
    # the camera joints are `fixed` and must not enter the actuated set.
    revolute = sorted(
        name for name, j in joint_map.items() if j.get("type") == "revolute"
    )
    assert revolute == sorted(expected)


# ── 2. the camera hangs off the waist through fixed joints only ──────────────

def test_the_camera_chain_reaches_waist_yaw_link():
    joint_map = _joint_map()
    chain = _parent_chain("camera_optical_frame", joint_map)

    links = [link for link, _ in chain]
    types = [typ for _, typ in chain]

    # All three camera/head joints are fixed; the chain terminates at
    # waist_yaw_link, which is a root in this motion-control URDF.
    assert links == ["camera_optical_frame", "camera_link",
                     "head_center_link", "waist_yaw_link"]
    # The chain walk continues one step past waist_yaw_link via its revolute
    # joint to base_link; the camera/head segment itself is entirely fixed.
    assert types[:3] == ["fixed", "fixed", "fixed"]
    assert types[-1] == "revolute"
    assert joint_map["camera_optical_joint"].get("type") == "fixed"
    assert joint_map["camera_mount_joint"].get("type") == "fixed"
    assert joint_map["head_center_joint"].get("type") == "fixed"
    # The camera is measured relative to the head geometric centre, so its
    # immediate parent must be head_center_link, not waist_yaw_link/base_link.
    assert joint_map["camera_mount_joint"].find("parent").get("link") == "head_center_link"
    # On-robot A5 finding: the head camera yaws with the waist, so the head
    # datum must ultimately be fixed to waist_yaw_link, not base_link.
    assert joint_map["head_center_joint"].find("parent").get("link") == "waist_yaw_link"


def test_head_center_datum_matches_the_vendor_mesh_derivation():
    """The head datum is derived, not measured; pin the derivation inputs."""
    joint_map = _joint_map()
    origin = joint_map["head_center_joint"].find("origin")
    xyz = [float(v) for v in origin.get("xyz").split()]

    # Midpoint of the vendor waist_yaw_link.STL head-region bounding box
    # (vertices with z >= 0.24 m). If the mesh or the datum changes, this test
    # forces the URDF comment and README to be updated together.
    assert xyz == pytest.approx([0.0055, 0.0, 0.3089], abs=1e-6)


# ── 3. the optical-frame rotation matches the calibration quaternion ──────────

def test_the_optical_frame_rotation_matches_the_calibration_quaternion():
    joint_map = _joint_map()
    optical_joint = joint_map["camera_optical_joint"]
    origin = optical_joint.find("origin")
    rpy = tuple(float(v) for v in origin.get("rpy").split())

    got = _rpy_to_quaternion(rpy)

    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    want = tuple(
        config["plugins"]["visual_odometry"]["calibration"]["camera_to_body"][
            "quaternion"]
    )

    # Quaternions are equal up to one *global* sign flip. Compare the whole
    # tuple at once — a per-component sign tolerance would also admit mixed
    # signs such as (0.5, 0.5, 0.5, 0.5), which is a different orientation.
    got_neg = tuple(-v for v in got)
    assert _close_quat(got, want) or _close_quat(got_neg, want), (
        f"rpy {rpy} → quaternion {got} does not match {want}"
    )


def _close_quat(a: tuple, b: tuple, tol: float = 1e-9) -> bool:
    return all(abs(x - y) < tol for x, y in zip(a, b))


# ── 4. the mount translation matches the calibration translation ─────────────

def test_the_mount_translation_matches_the_calibration_translation():
    joint_map = _joint_map()
    mount_joint = joint_map["camera_mount_joint"]
    origin = mount_joint.find("origin")
    xyz = [float(v) for v in origin.get("xyz").split()]

    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    translation = config["plugins"]["visual_odometry"]["calibration"][
        "camera_to_body"]["translation"]

    for got, want in zip(xyz, translation):
        assert abs(got - want) < 1e-6


def test_the_mount_translation_is_the_measured_head_relative_offset():
    """2026-11-24 measurement: +5 cm forward, +3.5 cm left, +0.5 cm up."""
    joint_map = _joint_map()
    origin = joint_map["camera_mount_joint"].find("origin")
    xyz = [float(v) for v in origin.get("xyz").split()]
    assert xyz == pytest.approx([0.05, 0.035, 0.005], abs=1e-9)


# ── 5. the camera frames carry no mass ────────────────────────────────────────

def test_the_camera_link_carries_no_mass():
    link_map = {l.get("name"): l for l in _parse_urdf().findall("link")}
    for name in ("head_center_link", "camera_link", "camera_optical_frame"):
        link = link_map[name]
        assert link.find("inertial") is None, f"{name} must not declare <inertial>"
