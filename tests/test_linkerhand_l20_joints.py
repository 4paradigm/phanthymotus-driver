"""Tests for the LinkerHand L20 joints card (data/json + sensor/skeleton) and its model."""

import json
import math
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))
sys.path.insert(0, str(ROOT / "tests"))

from common.vendor_runtime import DriverBundle
from can_client import JOINT_NAMES_16, L20CanClient, PositionQueryTimeout, vector20_to_named16
import hand_device
import joints_device
from hand_device import build_plugins
from joints_device import POLL_HZ, JointsPlugin, _JointsStream
import joints_model
from joints_model import (ANCHOR_X, ANCHOR_YAW, CALIBRATION_FILES, DISPLAY_SCALE, MODEL_DIR, RESERVED_VECTOR_INDEXES,
                          URDF_JOINT, VECTOR_INDEX, dual_urdf, model, raw_to_angle, side_from_topic, side_joints,
                          single_urdf, skeleton_joints)
from test_linkerhand_l20 import MockCanBus


GLOBAL_CONFIG = {"side": "left", "can_channel": "can0", "left_can_id": 0x28, "right_can_id": 0x27}


# ── mapping ────────────────────────────────────────────────────────────

# Index 19 = 300 checks the 0..255 clamp.
REFERENCE_VECTOR = [0, 64, 128, 192, 255, 0, 0, 255, 100, 200, 51, 9, 9, 9, 9, 255, 0, 128, 30, 300]


def _vendor_calibration(side):
    """The shipped vendor file, read independently of joints_model."""
    return json.loads((MODEL_DIR / CALIBRATION_FILES[side]).read_text(encoding="utf-8"))["joints"]


@pytest.mark.parametrize("side", ["left", "right"])
def test_angles_are_looked_up_in_the_vendor_calibration_table(side):
    vendor = _vendor_calibration(side)
    for name, index in VECTOR_INDEX.items():
        entry = vendor[URDF_JOINT[name]]
        assert entry["motor_index"] == index and not entry.get("passive"), name
        sign = -1.0 if URDF_JOINT[name] in joints_model.NEGATED_TABLES[side] else 1.0
        raw = REFERENCE_VECTOR[index]
        assert raw_to_angle(side, name, raw) == sign * entry["angle_rad"][min(raw, 255)], name
        assert raw_to_angle(side, name, -5) == sign * entry["angle_rad"][0]


def test_only_left_finger_spread_tables_are_negated():
    assert joints_model.NEGATED_TABLES["right"] == frozenset()
    assert joints_model.NEGATED_TABLES["left"] == {
        f"{finger}_mcp_roll" for finger in ("index", "middle", "ring", "pinky")}
    for name in ("index_yaw", "middle_yaw", "ring_yaw", "pinky_yaw"):
        # Mirrored hands: the same raw spreads the fingers to opposite model angles.
        assert raw_to_angle("left", name, 0) < 0 < raw_to_angle("right", name, 0)


def test_driver_thumb_yaw_and_roll_motors_turn_urdf_cmc_roll_and_cmc_yaw():
    # From the vendor motor assignment; the driver's names are the other way round.
    assert URDF_JOINT["thumb_yaw"] == "thumb_cmc_roll" and URDF_JOINT["thumb_roll"] == "thumb_cmc_yaw"


def test_sixteen_active_joints_skip_reserved_slots_and_match_driver_layout():
    assert set(VECTOR_INDEX) == set(JOINT_NAMES_16) == set(URDF_JOINT)
    assert sorted(VECTOR_INDEX.values()) == [i for i in range(20) if i not in RESERVED_VECTOR_INDEXES]
    # Same layout as the driver's own 20 -> 16 conversion.
    assert vector20_to_named16(list(range(20))) == VECTOR_INDEX


@pytest.mark.parametrize("side", ["left", "right"])
def test_every_model_joint_has_a_table_on_a_measured_motor(side):
    joints = {j["name"]: j for j in side_joints(side)}
    vendor = _vendor_calibration(side)
    assert len(joints) == 21 and set(vendor) == set(joints)
    assert set(URDF_JOINT.values()) <= set(joints)
    unmeasured = set(joints) - set(URDF_JOINT.values())
    # The five distal joints are passive and follow a measured motor.
    assert len(unmeasured) == 5
    assert all(vendor[name]["passive"] and vendor[name]["motor_index"] in VECTOR_INDEX.values() for name in unmeasured)


def test_calibration_that_disagrees_with_the_mapping_is_rejected(tmp_path, monkeypatch):
    for name in ("linkerhand_g20_right.urdf", CALIBRATION_FILES["right"]):
        (tmp_path / name).write_bytes((MODEL_DIR / name).read_bytes())
    data = json.loads((tmp_path / CALIBRATION_FILES["right"]).read_text())
    data["joints"]["thumb_cmc_roll"]["motor_index"], data["joints"]["thumb_cmc_yaw"]["motor_index"] = 10, 5
    (tmp_path / CALIBRATION_FILES["right"]).write_text(json.dumps(data))
    monkeypatch.setattr(joints_model, "MODEL_DIR", tmp_path)
    joints_model.side_joints.cache_clear()
    joints_model.calibration.cache_clear()
    try:
        with pytest.raises(ValueError, match="motor assignment"):
            joints_model.calibration("right")
    finally:
        monkeypatch.undo()
        joints_model.side_joints.cache_clear()
        joints_model.calibration.cache_clear()


def test_skeleton_clamps_to_model_limits_and_marks_passive_joints_as_derived():
    raw = {name: 128 for name in JOINT_NAMES_16}
    raw["index_pitch"] = 0  # The table reaches 1.24 rad, past the URDF limit of 1.22.
    entries = {e["name"]: e for e in skeleton_joints("right", raw)}
    assert len(entries) == 21
    limits = {j["name"]: j for j in side_joints("right")}
    pitch = entries["right_hand_index_mcp_pitch"]
    assert pitch["q"] == limits["index_mcp_pitch"]["upper"] and pitch["clamped_to_model"] is True
    assert pitch["source"] == "measured" and pitch["group"] == "右手"
    vendor = _vendor_calibration("right")
    derived = [e for e in entries.values() if e["source"] == "derived_calibration"]
    assert len(derived) == 5
    for entry in derived:
        name = entry["name"].removeprefix("right_hand_")
        joint = limits[name]
        expected = min(joint["upper"], max(joint["lower"], vendor[name]["angle_rad"][128]))
        assert entry["q"] == pytest.approx(expected)
    # Right-hand indexes follow the left hand's in the dual model.
    assert min(e["idx"] for e in entries.values()) >= len(side_joints("left"))


# ── model ──────────────────────────────────────────────────────────────

def _rot(axis, angle):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _apply(r, v):
    return [sum(r[i][k] * v[k] for k in range(3)) for i in range(3)]


def _rpy_fixed_axis(rpy):  # URDF: Rz(yaw)·Ry(pitch)·Rx(roll)
    return _mul(_rot((0, 0, 1), rpy[2]), _mul(_rot((0, 1, 0), rpy[1]), _rot((1, 0, 0), rpy[0])))


def _rpy_intrinsic_xyz(rpy):  # Current Core renderer: Rx·Ry·Rz
    return _mul(_rot((1, 0, 0), rpy[0]), _mul(_rot((0, 1, 0), rpy[1]), _rot((0, 0, 1), rpy[2])))


def _floats(text, default="0 0 0"):
    return [float(v) for v in (text or default).split()]


def _forward(root, rpy_rule, q, scale=1.0):
    """World position/rotation of every link; revolute joint angles come from q by joint name."""
    joints = root.findall("joint")
    links = {l.get("name") for l in root.findall("link")}
    children = {j.find("child").get("link") for j in joints}
    poses = {name: ([0.0, 0.0, 0.0], _rot((0, 0, 1), 0.0)) for name in links - children}
    pending = list(joints)
    while pending:
        progressed = False
        for joint in list(pending):
            parent = joint.find("parent").get("link")
            if parent not in poses:
                continue
            origin = joint.find("origin")
            xyz = [v * scale for v in _floats(origin.get("xyz") if origin is not None else None)]
            rpy = _floats(origin.get("rpy") if origin is not None else None)
            p, r = poses[parent]
            r_child = _mul(r, rpy_rule(rpy))
            if joint.get("type") == "revolute":
                axis = joint.find("axis")
                r_child = _mul(r_child, _rot(_floats(axis.get("xyz") if axis is not None else None, "1 0 0"),
                                             q[joint.get("name")]))
            poses[joint.find("child").get("link")] = ([a + b for a, b in zip(p, _apply(r, xyz))], r_child)
            pending.remove(joint)
            progressed = True
        assert progressed, "unreachable joints"
    return poses


def _sample_q(side):
    return {j["name"]: j["lower"] + (j["upper"] - j["lower"]) * ((i * 0.37) % 1.0)
            for i, j in enumerate(side_joints(side))}


@pytest.mark.parametrize("side", ["left", "right"])
def test_split_dual_model_under_core_rotation_order_matches_vendor_kinematics(side):
    vendor = joints_model._vendor_root(side)
    dual = ET.fromstring(dual_urdf())
    q = _sample_q(side)
    both = {f"{s}_hand_{name}": value for s in ("left", "right") for name, value in _sample_q(s).items()}
    expected = _forward(vendor, _rpy_fixed_axis, q, scale=DISPLAY_SCALE)
    actual = _forward(dual, _rpy_intrinsic_xyz, both)
    turn = _rot((0, 0, 1), ANCHOR_YAW[side])
    for link, (p, r) in expected.items():
        p2, r2 = actual[f"{side}_{link}"]
        p = _apply(turn, p)
        assert p2 == pytest.approx([p[0] + ANCHOR_X[side], p[1], p[2]], abs=1e-9), link
        r = _mul(turn, r)
        assert [v for row in r2 for v in row] == pytest.approx([v for row in r for v in row], abs=1e-9), link
    # The split is needed: without it Core's order would move at least one link.
    naive = _forward(vendor, _rpy_intrinsic_xyz, q, scale=DISPLAY_SCALE)
    assert any(abs(a - b) > 1e-6 for link in expected for a, b in zip(naive[link][0], expected[link][0]))


def _link_gap(side, raw20, link_a, link_b):
    """Distance in mm on the real hand between two link origins of one hand for a raw vector."""
    raw = {name: raw20[index] for name, index in VECTOR_INDEX.items()}
    q = {f"{s}_hand_{j['name']}": 0.0 for s in ("left", "right") for j in side_joints(s)}
    q.update({entry["name"]: entry["q"] for entry in skeleton_joints(side, raw)})
    poses = _forward(ET.fromstring(dual_urdf()), _rpy_intrinsic_xyz, q)
    return math.dist(poses[f"{side}_{link_a}"][0], poses[f"{side}_{link_b}"][0]) * 1000 / DISPLAY_SCALE


def _right_link_gap(raw20, link_a, link_b):
    return _link_gap("right", raw20, link_a, link_b)


def _preset(name, **raw):
    from gesture_device import DEFAULT_POSES_20
    vector = list(DEFAULT_POSES_20[name])
    for joint, value in raw.items():
        vector[VECTOR_INDEX[joint]] = value
    return vector


def test_right_ok_sign_closes_thumb_tip_on_index_tip():
    # Tip points are on each finger's centre line, not on the pads, so touching
    # tips stay ~23-28 mm apart (6.7 mm between the vendor tip meshes, checked offline).
    ok = _right_link_gap(_preset("ok_gesture"), "thumb_tip", "index_tip")
    natural = _right_link_gap(_preset("natural_gesture"), "thumb_tip", "index_tip")
    assert ok < 35 and natural > 100


@pytest.mark.parametrize("side", ["left", "right"])
def test_every_finger_ends_in_a_drawn_tip_beyond_its_last_joint(side):
    # Core draws bones only up to joint origins; the tip needs its own leaf link.
    dual = ET.fromstring(dual_urdf())
    joints = {j.get("name"): j for j in dual.findall("joint")}
    parents = {j.find("parent").get("link") for j in joints.values()}
    for finger in ("thumb", "index", "middle", "ring", "pinky"):
        last = joints[f"{side}_hand_{finger}_ip" if finger == "thumb" else f"{side}_hand_{finger}_dip"]
        tip = joints[f"{side}_hand_{finger}_tip_fixed"]
        assert tip.get("type") == "fixed"
        assert tip.find("parent").get("link") == last.find("child").get("link")
        assert tip.find("child").get("link") == f"{side}_{finger}_tip" and f"{side}_{finger}_tip" not in parents
        length = math.hypot(*_floats(tip.find("origin").get("xyz"))) * 1000 / DISPLAY_SCALE
        assert 20 < length < 35, finger


@pytest.mark.parametrize("joint, reference", [
    ("thumb_pitch", "index_proximal"),      # 0: toward the index finger
    ("thumb_yaw", "middle_metacarpals"),    # 0: turns toward the palm
    ("thumb_tip", "thumb_metacarpals"),     # 0: bent
])
def test_right_thumb_moves_like_the_real_hand(joint, reference):
    # Real hand, each joint moved alone from the natural pose (2026-11-12): raw 0
    # brings the thumb closer to the reference than raw 255. thumb_roll is not
    # checked: "folds toward the palm" has no unambiguous distance in the model.
    near = _right_link_gap(_preset("natural_gesture", **{joint: 0}), "thumb_tip", reference)
    far = _right_link_gap(_preset("natural_gesture", **{joint: 255}), "thumb_tip", reference)
    assert near < far


def _display_positions(raw20=None):
    """Positions of every link in the dual model as Core places them (both hands)."""
    q = {f"{s}_hand_{j['name']}": 0.0 for s in ("left", "right") for j in side_joints(s)}
    if raw20 is not None:
        raw = {name: raw20[index] for name, index in VECTOR_INDEX.items()}
        for side in ("left", "right"):
            q.update({entry["name"]: entry["q"] for entry in skeleton_joints(side, raw)})
    return {link: p for link, (p, _) in _forward(ET.fromstring(dual_urdf()), _rpy_intrinsic_xyz, q).items()}


def test_hands_are_shown_as_mirror_images_with_palms_to_the_camera():
    # User, 2026-11-13: both hands used to face the same way; palms should face the camera.
    pos = _display_positions(_preset("natural_gesture"))
    for side, outward in (("left", -1), ("right", 1)):
        # Palms towards the viewer: thumb and index on the outer side, pinky towards the other hand.
        assert outward * (pos[f"{side}_thumb_tip"][0] - pos[f"{side}_pinky_tip"][0]) > 0
        assert outward * (pos[f"{side}_index_tip"][0] - pos[f"{side}_pinky_tip"][0]) > 0
        # Fingers up.
        assert pos[f"{side}_middle_tip"][2] > pos[f"{side}_middle_metacarpals"][2]
    for link in ("thumb_tip", "index_tip", "middle_tip", "ring_tip", "pinky_tip", "middle_metacarpals"):
        left, right = pos[f"left_{link}"], pos[f"right_{link}"]
        assert left == pytest.approx([-right[0], right[1], right[2]], abs=0.02), link
    fist = _display_positions(_preset("natural_gesture", index_pitch=0, index_tip=0))
    for side in ("left", "right"):
        # Core's camera looks from -y (URDF z-up becomes three.js y-up, camera at +z there),
        # so a finger curling into the palm comes towards it.
        assert fist[f"{side}_index_tip"][1] < pos[f"{side}_index_tip"][1]


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("finger, last, previous", [
    ("thumb", "thumb_ip", "thumb_mcp"), ("index", "index_dip", "index_pip"), ("pinky", "pinky_dip", "pinky_pip")])
def test_straight_finger_tip_continues_the_last_bone(side, finger, last, previous):
    # User, 2026-11-13: tips looked bent outward when the finger was straight.
    pos = _display_positions()
    links = {j.get("name"): j.find("child").get("link") for j in ET.fromstring(dual_urdf()).findall("joint")}
    a, b = pos[links[f"{side}_hand_{previous}"]], pos[links[f"{side}_hand_{last}"]]
    tip = pos[f"{side}_{finger}_tip"]
    bone = [y - x for x, y in zip(a, b)]
    end = [y - x for x, y in zip(b, tip)]
    cos = sum(x * y for x, y in zip(bone, end)) / (math.hypot(*bone) * math.hypot(*end))
    assert cos > math.cos(math.radians(3))


@pytest.mark.parametrize("side", ["left", "right"])
def test_v_sign_spreads_index_and_middle_wider_than_natural_pose(side):
    # Real hands (right 2026-11-12, left 2026-11-13): V sign visibly spreads index and middle.
    v_sign = _link_gap(side, _preset("v_sign_gesture"), "index_tip", "middle_tip")
    natural = _link_gap(side, _preset("natural_gesture"), "index_tip", "middle_tip")
    assert v_sign > 1.5 * natural


def test_left_ok_sign_closes_thumb_tip_on_index_tip():
    ok = _link_gap("left", _preset("ok_gesture"), "thumb_tip", "index_tip")
    natural = _link_gap("left", _preset("natural_gesture"), "thumb_tip", "index_tip")
    assert ok < 35 and natural > 100


def test_dual_model_keeps_joint_names_types_limits_and_has_one_root():
    dual = ET.fromstring(dual_urdf())
    revolute = {j.get("name"): j for j in dual.findall("joint") if j.get("type") == "revolute"}
    assert set(revolute) == {f"{side}_hand_{j['name']}" for side in ("left", "right") for j in side_joints(side)}
    for side in ("left", "right"):
        for joint in side_joints(side):
            limit = revolute[f"{side}_hand_{joint['name']}"].find("limit")
            assert float(limit.get("lower")) == joint["lower"] and float(limit.get("upper")) == joint["upper"]
    children = {j.find("child").get("link") for j in dual.findall("joint")}
    roots = [l.get("name") for l in dual.findall("link") if l.get("name") not in children]
    assert roots == ["dual_hand_base"]
    # Kinematics only: no meshes are referenced.
    assert dual.find(".//mesh") is None and dual.find(".//visual") is None


# ── model: one hand per panel ──────────────────────────────────────────

def test_single_model_has_one_root_and_only_that_hand():
    for side in ("left", "right"):
        robot = ET.fromstring(single_urdf(side))
        assert robot.get("name") == f"linkerhand_l20_{side}_skeleton"
        revolute = {j.get("name") for j in robot.findall("joint") if j.get("type") == "revolute"}
        assert revolute == {f"{side}_hand_{j['name']}" for j in side_joints(side)}
        children = {j.find("child").get("link") for j in robot.findall("joint")}
        roots = [l.get("name") for l in robot.findall("link") if l.get("name") not in children]
        assert roots == [f"{side}_base"]
        other = "right_" if side == "left" else "left_"
        assert all(not l.get("name").startswith(other) for l in robot.findall("link"))
        origin = robot.find(f"joint[@name='{side}_anchor']/origin")
        assert origin.get("xyz") == "0.0 0.0 0.0"
        assert robot.find(".//mesh") is None and robot.find(".//visual") is None


def test_topic_selects_the_model_side_and_falls_back_to_both_hands():
    assert side_from_topic("/nvidia_desktop/hand/left/joints_ab12_skeleton") == "left"
    assert side_from_topic("/nvidia_desktop/hand/right/joints_ab12_skeleton") == "right"
    assert side_from_topic("/nvidia_desktop/hand/both/left_can_x/right_can_y/joints_skeleton") is None
    assert side_from_topic("") is None and side_from_topic(None) is None
    assert ET.fromstring(model("/x/hand/left/y")["urdf"]).get("name") == "linkerhand_l20_left_skeleton"
    assert ET.fromstring(model("/x/hand/right/y")["urdf"]).get("name") == "linkerhand_l20_right_skeleton"
    assert ET.fromstring(model("/x/hand/both/y")["urdf"]).get("name") == "linkerhand_l20_dual_skeleton"
    assert ET.fromstring(model()["urdf"]).get("name") == "linkerhand_l20_dual_skeleton"


def test_model_resource_serves_the_topic_side_through_driver_bundle():
    plugin = JointsPlugin(None, {}, global_config=GLOBAL_CONFIG)
    bundle = DriverBundle([plugin])
    tools = {t["name"]: t for t in bundle.get_all_tools()}
    assert tools["model"]["type"] == "resource" and tools["joints"]["type"] == "sensor"
    topic_prop = tools["model"]["inputSchema"]["properties"]["topic"]
    assert topic_prop["type"] == "string"
    left = ET.fromstring(bundle.dispatch("model", {"topic": "/hand/left/joints_a_skeleton"})["urdf"])
    assert left.get("name") == "linkerhand_l20_left_skeleton"
    dual = ET.fromstring(bundle.dispatch("model", {})["urdf"])
    assert dual.get("name") == "linkerhand_l20_dual_skeleton"


# ── acquisition ────────────────────────────────────────────────────────

class FakeClient:
    def __init__(self, vector=None):
        self.vector = vector or list(REFERENCE_VECTOR[:19]) + [200]
        self.error = None
        self.calls = 0
        self.timeouts = []

    def get_positions_vector(self, timeout_s=None):
        self.calls += 1
        self.timeouts.append(timeout_s)
        if self.error is not None:
            raise self.error
        return list(self.vector)


def make_plugin(clients=None, config=None, ros2=None, channels=("can0",), hands=None):
    """A plugin whose CAN world is injected: no sysfs, no socketcan."""
    clients = clients if clients is not None else {}
    hands = set(hands) if hands is not None else {
        (side, channel) for side in ("left", "right") for channel in channels}
    plugin = JointsPlugin(None, dict(config or {}), global_config=GLOBAL_CONFIG, ros2=ros2)
    plugin._channels = lambda: list(channels)
    plugin._channel_usable = lambda channel: True
    plugin._probe = lambda side, channel: (side, channel) in hands

    def get_controller(side, channel):
        client = clients.setdefault((side, channel), FakeClient())
        plugin._used_targets.add((side, channel))
        return SimpleNamespace(get=lambda: client)

    plugin._get_controller = get_controller
    return plugin, clients, hands


def one_round(plugin, side="right", channel="can0"):
    return _JointsStream(plugin, "probe", side, channel).read()


def test_one_round_reports_raw_and_looked_up_angles_for_sixteen_joints():
    plugin, clients, _ = make_plugin()
    result = one_round(plugin)
    assert result["state"] == "ok" and result["valid"] is True
    assert result["side"] == "right" and result["can_id"] == "0x27"
    assert result["joint_count"] == 16 and len(result["joints"]) == 16
    assert result["angle_unit"] == "rad" and result["angle_source"] == "vendor_g20_calibration_table_adjusted"
    assert result["reserved_vector_indexes"] == [11, 12, 13, 14]
    assert result["timestamp"] is not None and result["source"] == "can_position_query"
    vector = clients[("right", "can0")].vector
    for joint in result["joints"]:
        assert joint["raw"] == vector[joint["vector_index"]]
        assert joint["angle_rad"] == pytest.approx(raw_to_angle("right", joint["name"], joint["raw"]))
        assert joint["model_joint"] == "right_hand_" + URDF_JOINT[joint["name"]]
    assert clients[("right", "can0")].timeouts == [plugin.query_timeout_s]


@pytest.mark.parametrize("failure", [
    PositionQueryTimeout("位置回复超时", {"missing_frames": ["0x45"]}),
    "short",
    "out_of_range",
])
def test_failed_or_partial_rounds_are_invalid_without_values_or_timestamp(failure):
    plugin, clients, _ = make_plugin()
    client = clients.setdefault(("left", "can0"), FakeClient())
    if failure == "short":
        client.vector = client.vector[:16]
    elif failure == "out_of_range":
        client.vector[VECTOR_INDEX["ring_tip"]] = 256
    else:
        client.error = failure
    result = one_round(plugin, "left")
    assert result["valid"] is False and result["state"] == "error" and result["reason"]
    assert result["timestamp"] is None
    assert all(j["raw"] is None and j["angle_rad"] is None for j in result["joints"])
    if isinstance(failure, PositionQueryTimeout):
        assert result["error_details"] == {"missing_frames": ["0x45"]}


def test_reserved_slot_contents_are_ignored():
    plugin, clients, _ = make_plugin()
    client = clients.setdefault(("left", "can0"), FakeClient())
    client.vector[11:15] = [-1, 999, 0, 0]
    assert one_round(plugin, "left")["valid"] is True


class PartialBus(MockCanBus):
    """Replies to every position request except the little finger's."""

    def send(self, msg):
        if list(msg.data)[:1] == [0x45]:
            with self._lock:
                self.sent_messages.append(msg)
            return
        super().send(msg)


@pytest.mark.parametrize("bus_type", [MockCanBus, PartialBus])
def test_real_client_sends_only_position_queries(bus_type):
    bus = bus_type(can_id=0x27)
    bus.sim_pitch, bus.sim_yaw, bus.sim_roll, bus.sim_tip = [10, 20, 30, 40, 50], [60, 70, 80, 90, 100], [110, 0, 0, 0, 0], [120, 130, 140, 150, 160]
    client = L20CanClient("can0", "right", can_id=0x27, recv_timeout_s=0.05, bus_factory=lambda *_: bus)
    plugin, _, _ = make_plugin({("right", "can0"): client}, config={"query_timeout_s": 0.1})
    try:
        result = one_round(plugin)
    finally:
        client.close()
    sent = [list(m.data) for m in bus.sent_messages]
    # Opening a client queries UID / hardware / software version (read-only);
    # everything else must be an empty-payload position request.
    version_queries = [[0xC0, 0], [0xC1, 0], [0xC2, 0]]
    queries = [frame for frame in sent if frame not in version_queries]
    assert sorted({frame[0] for frame in queries}) == [0x41, 0x42, 0x43, 0x44, 0x45]
    assert all(len(frame) == 1 for frame in queries)
    if bus_type is PartialBus:
        assert result["valid"] is False and result["timestamp"] is None
        return
    raw = {j["name"]: j["raw"] for j in result["joints"]}
    assert result["valid"] is True
    assert (raw["thumb_pitch"], raw["thumb_yaw"], raw["thumb_roll"], raw["thumb_tip"]) == (10, 60, 110, 120)
    assert (raw["pinky_pitch"], raw["pinky_yaw"], raw["pinky_tip"]) == (50, 100, 160)


# ── topics, matching, waiting ──────────────────────────────────────────

def test_topics_are_per_instance_and_keep_the_side():
    plugin, _, _ = make_plugin()
    assert plugin.topic_out("a") == []
    plugin.dispatch("config", {"instance_id": "card-1", "bound_side": "left"})
    assert plugin.topic_out("card-1") == [
        {"topic": "/hand/left/joints_card1", "format": "data/json"},
        {"topic": "/hand/left/joints_card1_skeleton", "format": "sensor/skeleton"},
    ]
    plugin.dispatch("config", {"instance_id": "card-2", "bound_side": "left"})
    assert plugin.topic_out("card-1")[0]["topic"] != plugin.topic_out("card-2")[0]["topic"]
    # A different namespace is honoured.
    ns = JointsPlugin(None, {}, global_config=GLOBAL_CONFIG, namespace="robot1")
    ns.dispatch("config", {"instance_id": "x", "bound_side": "right"})
    assert ns.topic_out("x")[0]["topic"] == "/robot1/hand/right/joints_x"


def test_no_side_selected_reports_unset_without_topic_or_can():
    plugin, clients, _ = make_plugin()
    assert plugin.dispatch("start", {"instance_id": "a"})["state"] == "unset"
    assert plugin._streams == {} and clients == {}
    assert plugin.dispatch("config", {"instance_id": "a", "bound_side": "both"})["bound_side"] is None
    assert plugin._bindings["a"] is None
    assert plugin.topic_out("a") == []


def test_start_claims_a_free_hand():
    plugin, clients, _ = make_plugin(channels=("can0", "can1"), hands={("left", "can1")})
    info = plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
    assert info["state"] == "running" and info["can_channel"] == "can1"
    assert plugin._claims["a"] == ("left", "can1")
    wait_for(lambda: clients[("left", "can1")].calls > 0)


def test_waiting_card_publishes_invalid_json_and_no_skeleton(ros_runtime, monkeypatch):
    ros2, published, _ = ros_runtime
    monkeypatch.setattr(joints_device, "WAITING_PUBLISH_INTERVAL_S", 0.02)
    plugin, _, _ = make_plugin(ros2=ros2, hands=set())
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        wait_for(lambda: any(t == "/hand/left/joints_a" and p["state"] == "waiting" for t, p in published))
        info = plugin.dispatch("info", {"instance_id": "a"})
    finally:
        plugin.stop()
    assert info["state"] == "waiting" and info["can_channel"] is None
    assert info["measured_hz"] is None
    frame = next(p for t, p in published if t == "/hand/left/joints_a")
    assert frame["valid"] is False and frame["joints"][0]["raw"] is None and frame["timestamp"] is None
    assert not any(t.endswith("_skeleton") for t, _ in published)


def test_waiting_card_takes_a_hand_that_appears_later(ros_runtime, monkeypatch):
    ros2, _, _ = ros_runtime
    monkeypatch.setattr(joints_device, "MATCH_RETRY_INTERVAL_S", 0.05)
    plugin, clients, hands = make_plugin(ros2=ros2, hands=set())
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        assert plugin.dispatch("info", {"instance_id": "a"})["state"] == "waiting"
        hands.add(("left", "can0"))
        wait_for(lambda: plugin.dispatch("info", {"instance_id": "a"})["state"] == "running")
        assert plugin._claims["a"] == ("left", "can0")
        wait_for(lambda: clients[("left", "can0")].calls > 0)
    finally:
        plugin.stop()


def test_second_card_of_the_same_side_waits_and_takes_over_after_stop(ros_runtime, monkeypatch):
    ros2, _, _ = ros_runtime
    monkeypatch.setattr(joints_device, "MATCH_RETRY_INTERVAL_S", 0.05)
    plugin, clients, _ = make_plugin(ros2=ros2, hands={("left", "can0")})
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        wait_for(lambda: plugin.dispatch("info", {"instance_id": "a"})["state"] == "running")
        plugin.dispatch("start", {"instance_id": "b", "bound_side": "left"})
        assert plugin.dispatch("info", {"instance_id": "b"})["state"] == "waiting"
        assert plugin._claims == {"a": ("left", "can0")}
        plugin.dispatch("stop", {"instance_id": "a"})
        wait_for(lambda: plugin.dispatch("info", {"instance_id": "b"})["state"] == "running")
        assert plugin._claims == {"b": ("left", "can0")}
        wait_for(lambda: clients[("left", "can0")].calls > 0)
    finally:
        plugin.stop()


def test_left_and_right_cards_can_share_one_bus(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, clients, _ = make_plugin(ros2=ros2, channels=("can1",),
                                     hands={("left", "can1"), ("right", "can1")})
    try:
        plugin.dispatch("start", {"instance_id": "L", "bound_side": "left"})
        plugin.dispatch("start", {"instance_id": "R", "bound_side": "right"})
        assert plugin.dispatch("info", {"instance_id": "L"})["state"] == "running"
        assert plugin.dispatch("info", {"instance_id": "R"})["state"] == "running"
        assert plugin._claims == {"L": ("left", "can1"), "R": ("right", "can1")}
        assert plugin.topic_out("L")[0]["topic"] == "/hand/left/joints_l"
        assert plugin.topic_out("R")[0]["topic"] == "/hand/right/joints_r"
        wait_for(lambda: clients[("left", "can1")].calls > 0 and clients[("right", "can1")].calls > 0)
    finally:
        plugin.stop()


def test_waiting_reason_names_the_occupied_channel():
    plugin, _, _ = make_plugin(hands={("left", "can0")})
    plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
    for instance in ("b", "c"):
        plugin.dispatch("start", {"instance_id": instance, "bound_side": "left"})
    assert "已被其他卡片占用" in plugin._match_reason("left")
    assert "未在" in plugin._match_reason("right")
    plugin.stop()


# ── streaming ──────────────────────────────────────────────────────────

@pytest.fixture
def ros_runtime(monkeypatch):
    published = []

    class String:
        data = ""

    class Publisher:
        def __init__(self, topic):
            self.topic = topic

        def publish(self, msg):
            published.append((self.topic, json.loads(msg.data)))

        def get_subscription_count(self):
            return 1

    class Node:
        created = []

        def __init__(self, *args, **kwargs):
            self.destroyed = False
            Node.created.append(self)

        def create_publisher(self, msg_type, topic, qos):
            return Publisher(topic)

        def destroy_node(self):
            self.destroyed = True

    class QoS:
        def __init__(self, **kwargs):
            self.settings = kwargs

    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        QoSProfile=QoS, ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
        DurabilityPolicy=SimpleNamespace(VOLATILE=1), HistoryPolicy=SimpleNamespace(KEEP_LAST=1)))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=String))
    ros2 = SimpleNamespace(ctx_core=object(), executor_core=mock.Mock())
    return ros2, published, Node.created


def wait_for(predicate, timeout=3.0):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        time.sleep(0.005)
    assert predicate()


def test_one_round_feeds_both_outputs_with_the_same_sequence(ros_runtime):
    ros2, published, _ = ros_runtime
    plugin, _, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "right"})
        wait_for(lambda: sum(1 for t, _ in published if t.endswith("_skeleton")) >= 2)
    finally:
        plugin.stop()
    json_frames = {p["sequence"]: p for t, p in published if t == "/hand/right/joints_a"}
    skeleton_frames = [p for t, p in published if t == "/hand/right/joints_a_skeleton"]
    for frame in skeleton_frames:
        sample = json_frames[frame["status"]["sequence"]]
        assert frame["status"]["timestamp"] == sample["timestamp"] and frame["status"]["valid"] is True
        by_name = {e["name"]: e for e in frame["joints"]}
        for joint in sample["joints"]:
            entry = by_name[joint["model_joint"]]
            assert entry["source"] == "measured"
            limits = next(j for j in side_joints("right") if f"right_hand_{j['name']}" == joint["model_joint"])
            assert entry["q"] == pytest.approx(min(limits["upper"], max(limits["lower"], joint["angle_rad"])))


def test_invalid_rounds_publish_json_but_no_skeleton(ros_runtime):
    ros2, published, _ = ros_runtime
    client = FakeClient()
    client.error = PositionQueryTimeout("位置回复超时", {})
    plugin, _, _ = make_plugin({("left", "can0"): client}, ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        wait_for(lambda: len(published) >= 3)
        info = plugin.dispatch("info", {"instance_id": "a"})
    finally:
        plugin.stop()
    assert all(t == "/hand/left/joints_a" and p["valid"] is False for t, p in published)
    assert info["published_skeleton_frames"] == 0 and info["sample_state"] == "error"


def test_stop_sends_one_final_invalid_frame_and_no_skeleton(ros_runtime):
    ros2, published, _ = ros_runtime
    plugin, _, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        wait_for(lambda: sum(1 for t, _ in published if t == "/hand/left/joints_a") >= 2)
        published.clear()
        plugin.dispatch("stop", {"instance_id": "a"})
    finally:
        plugin.stop()
    frames = [p for t, p in published if t == "/hand/left/joints_a"]
    assert [f for f in frames if f["state"] == "idle"], published
    final = frames[-1]
    assert final["state"] == "idle" and final["valid"] is False and final["reason"]
    assert sum(1 for f in frames if f["state"] == "idle") == 1
    assert final["timestamp"] is None and final["measured_hz"] is None
    assert final["query_elapsed_s"] is None and final["joint_count"] == 16
    assert all(j["raw"] is None and j["angle_rad"] is None and j["angle_deg"] is None
               for j in final["joints"])
    assert not [p for t, p in published if t.endswith("_skeleton") and p["status"]["valid"] is False]


def test_fixed_rate_is_thirty_hz(ros_runtime):
    assert POLL_HZ == 30.0
    ros2, published, _ = ros_runtime
    plugin, _, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        start = time.monotonic()
        wait_for(lambda: sum(1 for t, _ in published if t == "/hand/left/joints_a") >= 6, timeout=2.0)
        elapsed = time.monotonic() - start
        count = sum(1 for t, _ in published if t == "/hand/left/joints_a")
        assert count / elapsed >= 15, (count, elapsed)
        assert plugin.dispatch("info", {"instance_id": "a"})["poll_hz"] == 30.0
    finally:
        plugin.stop()


def test_stale_running_sample_is_not_reported_as_current(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, _, _ = make_plugin(ros2=ros2, config={"stale_after_s": 0.05})
    stream = None
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        stream = plugin._streams["a"]
        wait_for(lambda: stream.latest.get("valid"))
        stream._stop_event.set()
        wait_for(lambda: not stream.running())
        stream._thread = SimpleNamespace(is_alive=lambda: True)
        time.sleep(0.1)
        result = plugin.dispatch("get_state", {"instance_id": "a"})
    finally:
        if stream is not None:
            stream._thread = None
        plugin.stop()
    assert result["valid"] is False and result["sample_state"] == "stale" and result["reason"]


# ── lifecycle ──────────────────────────────────────────────────────────

def test_switching_side_stops_the_old_stream_and_moves_the_claim(ros_runtime):
    ros2, published, _ = ros_runtime
    plugin, _, _ = make_plugin(ros2=ros2, hands={("left", "can0"), ("right", "can0")})
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        wait_for(lambda: plugin.dispatch("info", {"instance_id": "a"})["state"] == "running")
        old = plugin._streams["a"]
        plugin.dispatch("config", {"instance_id": "a", "bound_side": "right"})
        assert not old.running()
        assert plugin._claims["a"] == ("right", "can0")
        published.clear()
        wait_for(lambda: any("/hand/right/" in t for t, _ in published))
        assert all("/hand/right/" in t for t, _ in published)
    finally:
        plugin.stop()


def test_config_ignores_legacy_channel_and_rate_fields():
    plugin, _, _ = make_plugin()
    result = plugin.dispatch("config", {
        "instance_id": "a", "bound_side": "left",
        "can_channel": "can9", "left_can_channel": "can8", "poll_hz": 5,
    })
    assert result["state"] == "configured" and result["bound_side"] == "left"
    assert not hasattr(plugin, "poll_hz")
    plugin.dispatch("start", {"instance_id": "a"})
    assert plugin._claims["a"] == ("left", "can0")
    plugin.stop()


def test_config_with_a_removed_side_stops_the_stream():
    plugin, _, _ = make_plugin()
    plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
    assert plugin._streams
    plugin.dispatch("config", {"instance_id": "a", "bound_side": "both"})
    assert plugin._streams == {} and plugin._claims == {}
    assert plugin.dispatch("info", {"instance_id": "a"})["state"] == "unset"


def test_get_state_does_not_query_can_before_the_card_starts():
    plugin, clients, _ = make_plugin()
    plugin.dispatch("config", {"instance_id": "a", "bound_side": "left"})
    result = plugin.dispatch("get_state", {"instance_id": "a"})
    assert result["state"] == "idle" and result["valid"] is False and result["topic_out"]
    assert clients == {}


def test_repeated_start_stop_and_config_replay_do_not_accumulate_workers(ros_runtime):
    ros2, _, nodes = ros_runtime
    plugin, _, _ = make_plugin(ros2=ros2)
    try:
        for _ in range(3):
            plugin.dispatch("config", {"instance_id": "a", "bound_side": "left"})
            plugin.dispatch("start", {"instance_id": "a"})
            plugin.dispatch("start", {"instance_id": "a"})
            plugin.dispatch("config", {"instance_id": "a", "bound_side": "left"})
            assert len(plugin._streams) == 1
            assert sum(1 for n in nodes if not n.destroyed) == 1
            assert plugin.dispatch("stop", {"instance_id": "a"})["state"] == "idle"
            assert plugin.dispatch("stop", {"instance_id": "a"})["state"] == "idle"
            assert plugin._streams == {} and plugin._claims == {}
            assert all(n.destroyed for n in nodes)
    finally:
        plugin.stop()


class BlockingClient(FakeClient):
    def __init__(self):
        super().__init__()
        self.enter = threading.Event()
        self.release = threading.Event()

    def get_positions_vector(self, timeout_s=None):
        self.calls += 1
        self.enter.set()
        self.release.wait(10)
        return super().get_positions_vector(timeout_s)


def test_stop_that_times_out_keeps_the_card_owning_its_worker(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, clients, _ = make_plugin(ros2=ros2, config={"query_timeout_s": 0.02})
    client = clients[("left", "can0")] = BlockingClient()
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        wait_for(lambda: client.enter.is_set())
        with pytest.raises(RuntimeError):
            plugin.dispatch("stop", {"instance_id": "a"})
        assert "a" in plugin._streams and plugin._claims["a"] == ("left", "can0")
        client.release.set()
        wait_for(lambda: not plugin._streams["a"].running())
        plugin.dispatch("stop", {"instance_id": "a"})
        assert plugin._streams == {} and plugin._claims == {}
    finally:
        client.release.set()
        plugin.stop()


def test_driver_stop_keeps_a_worker_it_cannot_stop_and_its_can_client(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, clients, _ = make_plugin(ros2=ros2, config={"query_timeout_s": 0.02})
    client = clients[("left", "can0")] = BlockingClient()
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        wait_for(lambda: client.enter.is_set())
        with pytest.raises(RuntimeError):
            plugin.stop()
        assert "a" in plugin._streams and plugin._claims["a"] == ("left", "can0")
        client.release.set()
        wait_for(lambda: not plugin._streams["a"].running())
        plugin.stop()
        assert plugin._streams == {} and plugin._claims == {}
    finally:
        client.release.set()
        try:
            plugin.stop()
        except RuntimeError:
            pass


def test_unknown_action_is_left_to_the_mcp_handler_as_an_error():
    plugin, clients, _ = make_plugin()
    assert plugin.dispatch("calibrate", {"instance_id": "A", "bound_side": "left"}) is None
    assert clients == {}


def test_driver_does_not_start_acquisition_on_boot():
    plugin, clients, _ = make_plugin()
    plugin.start()
    time.sleep(0.05)
    assert plugin._streams == {} and clients == {}


def test_shared_controller_is_only_trusted_when_the_hand_answers(monkeypatch):
    """A registry entry alone is not evidence the hand is still plugged in."""
    registry = {}
    monkeypatch.setattr(hand_device, "GLOBAL_CONTROLLERS", registry)
    monkeypatch.setattr(joints_device, "GLOBAL_CONTROLLERS", registry)

    class Controller:
        def __init__(self, alive):
            self.alive = alive
            self.asked = 0

        def get(self):
            def probe(timeout):
                self.asked += 1
                return self.alive
            return SimpleNamespace(probe_positions=probe)

    plugin, _, _ = make_plugin(channels=("can1",), hands={("right", "can1")})
    plugin._probe = lambda side, channel: pytest.fail(
        "an existing controller must be queried, not probed with a second client")

    dead = Controller(False)
    registry[("right", "can1")] = dead
    assert plugin._match("a", "right") is None
    assert dead.asked == 1 and "a" not in plugin._claims

    alive = Controller(True)
    registry[("right", "can1")] = alive
    assert plugin._match("b", "right") == ("right", "can1")
    assert alive.asked == 1 and plugin._claims["b"] == ("right", "can1")


def test_start_reads_bound_side_from_nested_instance_config():
    """Some platform paths send the gear settings as arguments.config."""
    plugin, _, _ = make_plugin(hands={("left", "can0")})
    try:
        info = plugin.dispatch("start", {"instance_id": "a", "config": {"bound_side": "left"}})
        assert info["state"] == "running" and info["bound_side"] == "left"
        assert plugin._claims["a"] == ("left", "can0")
    finally:
        plugin.stop()


def test_colliding_topic_slugs_are_rejected(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, _, _ = make_plugin(ros2=ros2, hands={("left", "can0")})
    try:
        plugin.dispatch("start", {"instance_id": "left-card-a", "bound_side": "left"})
        with pytest.raises(ValueError, match="后缀冲突"):
            plugin.dispatch("start", {"instance_id": "left_card_a", "bound_side": "left"})
        assert len(plugin._streams) == 1
        # The same suffix on the other side is fine: the topics differ by side.
        plugin.dispatch("start", {"instance_id": "left_card_a", "bound_side": "right"})
        assert len(plugin._streams) == 2
    finally:
        plugin.stop()


# ── integration with the other cards ───────────────────────────────────

def test_schema_exposes_only_the_side_and_no_rate_or_channel():
    plugin = JointsPlugin(None, {}, global_config=GLOBAL_CONFIG)
    definition = plugin.get_tools()[0]
    props = definition["configSchema"]["properties"]
    assert set(props) == {"bound_side"}
    assert props["bound_side"]["enum"] == ["left", "right"]
    assert not definition.get("topic_out")


def test_shipped_config_has_no_rate_setting_and_registers_joints_last():
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    joints_config = config["plugins"]["joints"]
    assert "poll_hz" not in joints_config
    plugins = build_plugins(config, "", None)
    assert [p.PREFIX for p in plugins] == ["hand", "hand_gesture", "hand_grasp", "hand_tactile", "joints"]
    joints = plugins[-1]
    assert joints.query_timeout_s == joints_config["query_timeout_s"]
    assert JOINT_NAMES_16 and POLL_HZ == 30.0


def test_disabled_joints_section_registers_nothing():
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    config["plugins"]["joints"]["enabled"] = False
    assert "joints" not in [p.PREFIX for p in build_plugins(config, "", None)]


def test_a_stopped_card_closes_and_removes_only_the_controller_it_created(monkeypatch):
    """A closed controller must not keep a hand looking occupied (PR #546 review)."""
    registry = {}

    class Controller:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    created = Controller()
    monkeypatch.setattr(hand_device, "GLOBAL_CONTROLLERS", registry)
    monkeypatch.setattr(joints_device, "GLOBAL_CONTROLLERS", registry)
    monkeypatch.setattr(joints_device, "get_shared_controller",
                        lambda config, side, channel, **kw: registry.setdefault((side, channel), created))
    plugin = JointsPlugin(None, {}, global_config=GLOBAL_CONFIG)
    plugin._get_controller("right", "can1")
    assert plugin._created_targets == {("right", "can1")}
    assert registry == {("right", "can1"): created}

    plugin._close_if_unused(("right", "can1"))
    assert created.closed and registry == {}

    # The next card must probe the hand again instead of trusting a stale entry.
    probed = []
    plugin._channels = lambda: ["can1"]
    plugin._channel_usable = lambda channel: True
    plugin._probe = lambda side, channel: probed.append((side, channel)) or True
    plugin._get_controller = lambda side, channel: SimpleNamespace(get=lambda: FakeClient())
    plugin.dispatch("start", {"instance_id": "next", "bound_side": "right"})
    try:
        assert probed == [("right", "can1")]
        assert plugin._claims["next"] == ("right", "can1")
    finally:
        plugin.stop()


def test_a_controller_created_by_another_plugin_is_never_closed(monkeypatch):
    """joints must not close a hand/tactile/gesture/grasp consumer's controller."""
    registry = {}

    class Controller:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    shared = Controller()
    registry[("right", "can1")] = shared
    monkeypatch.setattr(hand_device, "GLOBAL_CONTROLLERS", registry)
    monkeypatch.setattr(joints_device, "GLOBAL_CONTROLLERS", registry)
    plugin = JointsPlugin(None, {}, global_config=GLOBAL_CONFIG)
    plugin._used_targets.add(("right", "can1"))

    plugin._close_if_unused(("right", "can1"))
    assert shared.closed is False
    assert registry == {("right", "can1"): shared}
    assert plugin._used_targets == set()


def test_plugin_stop_closes_only_controllers_the_hand_card_does_not_own(monkeypatch):
    owned, own = mock.Mock(), mock.Mock()
    registry = {("left", "can0"): owned}
    monkeypatch.setattr(hand_device, "GLOBAL_CONTROLLERS", registry)
    import joints_device
    monkeypatch.setattr(joints_device, "GLOBAL_CONTROLLERS", registry)
    # The right controller does not exist yet, so this plugin creates it; the
    # left one belongs to the hand card and must not be touched.
    monkeypatch.setattr(joints_device, "get_shared_controller",
                        lambda config, side, channel, **kw: registry.setdefault((side, channel), own))
    hand = SimpleNamespace(hand_config={}, config=GLOBAL_CONFIG, _controllers={("left", "can0"): owned})
    plugin = JointsPlugin(hand, {}, global_config=GLOBAL_CONFIG)
    assert plugin._get_controller("left", "can0") is owned
    assert plugin._get_controller("right", "can1") is own
    # Read-only card: never registered on the hand card, so its shutdown cannot release torque here.
    assert hand._controllers == {("left", "can0"): owned}
    plugin.stop()
    owned.close.assert_not_called()
    own.close.assert_called_once()
    owned.release_torque.assert_not_called()
    own.release_torque.assert_not_called()
