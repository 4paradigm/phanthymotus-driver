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
from hand_device import build_plugins
from joints_device import JointsPlugin
import joints_model
from joints_model import (ANCHOR_X, ANCHOR_YAW, CALIBRATION_FILES, DISPLAY_SCALE, MODEL_DIR, RESERVED_VECTOR_INDEXES,
                          URDF_JOINT, VECTOR_INDEX, dual_urdf, raw_to_angle, side_joints, skeleton_joints)
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
    pitch = entries["right_index_mcp_pitch"]
    assert pitch["q"] == limits["index_mcp_pitch"]["upper"] and pitch["clamped_to_model"] is True
    assert pitch["source"] == "measured" and pitch["group"] == "右手"
    vendor = _vendor_calibration("right")
    derived = [e for e in entries.values() if e["source"] == "derived_calibration"]
    assert len(derived) == 5
    for entry in derived:
        name = entry["name"].removeprefix("right_")
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
    both = {f"{s}_{name}": value for s in ("left", "right") for name, value in _sample_q(s).items()}
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
    q = {f"{s}_{j['name']}": 0.0 for s in ("left", "right") for j in side_joints(s)}
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
        last = joints[f"{side}_{finger}_ip" if finger == "thumb" else f"{side}_{finger}_dip"]
        tip = joints[f"{side}_{finger}_tip_fixed"]
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
    q = {f"{s}_{j['name']}": 0.0 for s in ("left", "right") for j in side_joints(s)}
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
    a, b = pos[links[f"{side}_{previous}"]], pos[links[f"{side}_{last}"]]
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
    assert set(revolute) == {f"{side}_{j['name']}" for side in ("left", "right") for j in side_joints(side)}
    for side in ("left", "right"):
        for joint in side_joints(side):
            limit = revolute[f"{side}_{joint['name']}"].find("limit")
            assert float(limit.get("lower")) == joint["lower"] and float(limit.get("upper")) == joint["upper"]
    children = {j.find("child").get("link") for j in dual.findall("joint")}
    roots = [l.get("name") for l in dual.findall("link") if l.get("name") not in children]
    assert roots == ["dual_hand_base"]
    # Kinematics only: no meshes are referenced.
    assert dual.find(".//mesh") is None and dual.find(".//visual") is None


def test_model_resource_is_served_through_driver_bundle():
    plugin = JointsPlugin(None, {}, global_config=GLOBAL_CONFIG)
    bundle = DriverBundle([plugin])
    tools = {t["name"]: t for t in bundle.get_all_tools()}
    assert tools["model"]["type"] == "resource" and tools["joints"]["type"] == "sensor"
    result = bundle.dispatch("model", {})
    assert set(result) == {"urdf"}
    assert ET.fromstring(result["urdf"]).get("name") == "linkerhand_l20_dual_skeleton"


def test_topic_out_lists_json_first_then_skeleton():
    plugin = JointsPlugin(None, {}, global_config=GLOBAL_CONFIG, namespace="robot1")
    assert plugin.topic_out("right", "can0") == [
        {"topic": "/robot1/hand/right/joints", "format": "data/json"},
        {"topic": "/robot1/hand/right/joints_skeleton", "format": "sensor/skeleton"},
    ]
    other = plugin.topic_out("right", "can1")[0]["topic"]
    assert other == "/robot1/hand/right/can_" + b"can1".hex() + "/joints"


def test_both_hands_topic_out_has_each_json_and_one_shared_skeleton():
    plugin = JointsPlugin(None, {}, global_config=GLOBAL_CONFIG, namespace="robot1")
    can1 = "can_" + b"can1".hex()
    assert plugin.topic_out("both", "can0", (("left", "can0"), ("right", "can1"))) == [
        {"topic": "/robot1/hand/left/joints", "format": "data/json"},
        {"topic": f"/robot1/hand/right/{can1}/joints", "format": "data/json"},
        {"topic": f"/robot1/hand/both/right_{can1}/joints_skeleton", "format": "sensor/skeleton"},
    ]
    assert plugin.dual_topic("can0", "can0") == "/robot1/hand/both/joints_skeleton"
    # Different hand pairs never share a skeleton topic (Core rejects shared outputs).
    assert len({plugin.dual_topic(l, r) for l in ("can0", "can1") for r in ("can0", "can1")}) == 4
    # Single-hand cards keep per-side skeleton topics, so a left and a right card do not clash.
    assert plugin.topic_out("left")[1]["topic"] != plugin.topic_out("right")[1]["topic"]


def test_schema_offers_both_hands_mode_with_a_can_interface_per_hand():
    props = JointsPlugin(None, {}, global_config=GLOBAL_CONFIG).get_tools()[0]["configSchema"]["properties"]
    assert props["bound_side"]["enum"] == ["left", "right", "both"]
    for key in ("left_can_channel", "right_can_channel"):
        assert props[key]["scope"] == "instance" and "default" not in props[key]


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


def make_plugin(clients=None, config=None, ros2=None):
    clients = clients if clients is not None else {}
    plugin = JointsPlugin(None, {"poll_hz": 30, **(config or {})}, global_config=GLOBAL_CONFIG, ros2=ros2)

    def get_controller(side, channel):
        client = clients.setdefault((side, channel), FakeClient())
        plugin._used_targets.add((side, channel))
        return SimpleNamespace(get=lambda: client)

    plugin._get_controller = get_controller
    return plugin, clients


def test_get_state_reports_raw_and_unverified_angles_for_sixteen_joints():
    plugin, clients = make_plugin()
    result = plugin.dispatch("get_state", {"bound_side": "right"})
    assert result["valid"] is True and result["state"] == "ok" and result["sample_valid"] is True
    assert result["side"] == "right" and result["can_id"] == "0x27"
    assert result["joint_count"] == 16 and len(result["joints"]) == 16
    assert result["angle_unit"] == "rad" and result["angle_source"] == "vendor_g20_calibration_table_adjusted"
    assert result["reserved_vector_indexes"] == [11, 12, 13, 14]
    assert result["timestamp"] is not None and result["source"] == "can_position_query"
    vector = clients[("right", "can0")].vector
    for joint in result["joints"]:
        assert joint["raw"] == vector[joint["vector_index"]]
        assert joint["angle_rad"] == pytest.approx(raw_to_angle("right", joint["name"], joint["raw"]))
        assert joint["model_joint"] == "right_" + URDF_JOINT[joint["name"]]
    assert clients[("right", "can0")].timeouts == [plugin.query_timeout_s]


@pytest.mark.parametrize("failure", [
    PositionQueryTimeout("位置回复超时", {"missing_frames": ["0x45"]}),
    "short",
    "out_of_range",
])
def test_failed_or_partial_rounds_are_invalid_without_values_or_timestamp(failure):
    plugin, clients = make_plugin()
    client = clients.setdefault(("left", "can0"), FakeClient())
    if failure == "short":
        client.vector = client.vector[:16]
    elif failure == "out_of_range":
        client.vector[VECTOR_INDEX["ring_tip"]] = 256
    else:
        client.error = failure
    result = plugin.dispatch("get_state", {"bound_side": "left"})
    assert result["valid"] is False and result["state"] == "error" and result["sample_valid"] is False
    assert result["timestamp"] is None and result["reason"]
    assert all(j["raw"] is None and j["angle_rad"] is None for j in result["joints"])
    if isinstance(failure, PositionQueryTimeout):
        assert result["error_details"] == {"missing_frames": ["0x45"]}


def test_reserved_slot_contents_are_ignored():
    plugin, clients = make_plugin()
    client = clients.setdefault(("left", "can0"), FakeClient())
    client.vector[11:15] = [-1, 999, 0, 0]
    assert plugin.dispatch("get_state", {"bound_side": "left"})["valid"] is True


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
    plugin, _ = make_plugin({("right", "can0"): client}, config={"query_timeout_s": 0.1})
    try:
        result = plugin.dispatch("get_state", {"bound_side": "right"})
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
    plugin, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "right"})
        wait_for(lambda: sum(1 for t, _ in published if t.endswith("_skeleton")) >= 2)
    finally:
        plugin.stop()
    json_frames = {p["sequence"]: p for t, p in published if t == "/hand/right/joints"}
    skeleton_frames = [p for t, p in published if t == "/hand/right/joints_skeleton"]
    for frame in skeleton_frames:
        sample = json_frames[frame["status"]["sequence"]]
        assert frame["status"]["timestamp"] == sample["timestamp"] and frame["status"]["valid"] is True
        by_name = {e["name"]: e for e in frame["joints"]}
        for joint in sample["joints"]:
            entry = by_name[joint["model_joint"]]
            assert entry["source"] == "measured"
            limits = next(j for j in side_joints("right") if f"right_{j['name']}" == joint["model_joint"])
            assert entry["q"] == pytest.approx(min(limits["upper"], max(limits["lower"], joint["angle_rad"])))


def test_invalid_rounds_publish_json_but_no_skeleton(ros_runtime):
    ros2, published, _ = ros_runtime
    client = FakeClient()
    client.error = PositionQueryTimeout("位置回复超时", {})
    plugin, _ = make_plugin({("left", "can0"): client}, ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        wait_for(lambda: len(published) >= 3)
        info = plugin.dispatch("info", {"instance_id": "a"})
    finally:
        plugin.stop()
    assert all(t == "/hand/left/joints" and p["valid"] is False for t, p in published)
    assert info["published_skeleton_frames"] == 0 and info["sample_state"] == "error"


def test_stop_sends_one_final_invalid_frame_and_no_skeleton(ros_runtime):
    ros2, published, _ = ros_runtime
    plugin, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        wait_for(lambda: sum(1 for t, _ in published if t == "/hand/left/joints") >= 2)
        published.clear()
        plugin.dispatch("stop", {"instance_id": "a"})
    finally:
        plugin.stop()
    frames = [p for t, p in published if t == "/hand/left/joints"]
    # The worker may finish one more round before the stop joins it.
    assert [f for f in frames if f["state"] == "idle"], published
    final = frames[-1]
    assert final["state"] == "idle" and final["valid"] is False and final["reason"]
    assert sum(1 for f in frames if f["state"] == "idle") == 1
    assert final["timestamp"] is None and final["measured_hz"] is None
    assert final["query_elapsed_s"] is None
    assert final["joint_count"] == 16
    assert all(j["raw"] is None and j["angle_rad"] is None and j["angle_deg"] is None
               for j in final["joints"])
    # A stopped stream must not leave the renderer a frame with a pose in it.
    assert not [p for t, p in published if t.endswith("_skeleton") and p["status"]["valid"] is False]


def test_stale_running_sample_is_not_reported_as_current(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, _ = make_plugin(ros2=ros2, config={"stale_after_s": 0.05})
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        stream = plugin._streams[("left", "can0")]
        wait_for(lambda: stream.latest.get("valid"))
        stream._stop_event.set()  # Freeze the worker; the cached frame ages.
        wait_for(lambda: not stream.running())
        stream._thread = SimpleNamespace(is_alive=lambda: True)
        time.sleep(0.1)
        result = plugin.dispatch("get_state", {"instance_id": "a"})
    finally:
        stream._thread = None
        plugin.stop()
    assert result["valid"] is False and result["sample_state"] == "stale" and result["reason"]


def test_left_and_right_instances_are_isolated(ros_runtime):
    ros2, published, _ = ros_runtime
    left, right = FakeClient(), FakeClient([255] * 20)
    plugin, _ = make_plugin({("left", "can0"): left, ("right", "can1"): right}, ros2=ros2)
    try:
        plugin.dispatch("config", {"instance_id": "L", "bound_side": "left"})
        plugin.dispatch("config", {"instance_id": "R", "bound_side": "right", "can_channel": "can1"})
        plugin.dispatch("start", {"instance_id": "L"})
        plugin.dispatch("start", {"instance_id": "R"})
        assert set(plugin._streams) == {("left", "can0"), ("right", "can1")}
        wait_for(lambda: {t for t, _ in published} >= {"/hand/left/joints", "/hand/right/can_" + b"can1".hex() + "/joints"})
        plugin.dispatch("stop", {"instance_id": "L"})
        assert set(plugin._streams) == {("right", "can1")}
        assert plugin._streams[("right", "can1")].running()
    finally:
        plugin.stop()
    for topic, payload in published:
        if payload.get("state") == "idle":  # stop notice, no measurement
            continue
        side = "left" if "/left/" in topic else "right"
        assert (payload.get("side") or payload["status"]["side"]) == side
        if not topic.endswith("_skeleton"):
            expected = (left if side == "left" else right).vector
            assert [j["raw"] for j in payload["joints"]] == [expected[VECTOR_INDEX[n]] for n in JOINT_NAMES_16]
        else:
            assert all(e["name"].startswith(side + "_") for e in payload["joints"])


def test_switching_side_moves_the_stream_and_releases_the_old_one(ros_runtime):
    ros2, published, _ = ros_runtime
    plugin, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        old = plugin._streams[("left", "can0")]
        plugin.dispatch("config", {"instance_id": "a", "bound_side": "right"})
        assert set(plugin._streams) == {("right", "can0")} and not old.running()
        published.clear()
        wait_for(lambda: len(published) >= 2)
        assert all("/hand/right/" in t for t, _ in published)
    finally:
        plugin.stop()


def test_cards_on_the_same_hand_share_one_worker(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, clients = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "a", "bound_side": "left"})
        plugin.dispatch("start", {"instance_id": "b", "bound_side": "left"})
        assert len(plugin._streams) == 1
        plugin.dispatch("stop", {"instance_id": "a"})
        assert plugin._streams[("left", "can0")].running()
        plugin.dispatch("stop", {"instance_id": "b"})
        assert plugin._streams == {}
    finally:
        plugin.stop()


def test_anonymous_card_cannot_silently_switch_target(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"bound_side": "left"})
        with pytest.raises(ValueError):
            plugin.dispatch("start", {"bound_side": "right"})
    finally:
        plugin.stop()


def test_shared_config_does_not_bind_a_side():
    plugin, _ = make_plugin()
    result = plugin.dispatch("config", {"instance_id": "a", "poll_hz": 5})
    assert result["state"] == "configured" and plugin.poll_hz == 5
    assert "a" not in plugin._bindings and plugin._streams == {}
    with pytest.raises(ValueError):
        plugin.dispatch("config", {"instance_id": "a", "bound_side": "neither"})
    with pytest.raises(ValueError):
        plugin.dispatch("config", {"instance_id": "a", "poll_hz": 0})


def test_repeated_start_stop_and_config_replay_do_not_accumulate_workers(ros_runtime):
    ros2, _, nodes = ros_runtime
    plugin, _ = make_plugin(ros2=ros2)
    try:
        for _ in range(3):
            plugin.dispatch("config", {"instance_id": "a", "bound_side": "left"})
            plugin.dispatch("start", {"instance_id": "a"})
            plugin.dispatch("start", {"instance_id": "a"})
            plugin.dispatch("config", {"instance_id": "a", "bound_side": "left"})
            assert len(plugin._streams) == 1
            stream = plugin._streams[("left", "can0")]
            assert sum(1 for n in nodes if not n.destroyed) == 1
            assert plugin.dispatch("stop", {"instance_id": "a"})["state"] == "idle"
            assert plugin.dispatch("stop", {"instance_id": "a"})["state"] == "idle"
            assert not stream.running() and plugin._streams == {}
            assert all(n.destroyed for n in nodes)
    finally:
        plugin.stop()


BOTH_DUAL = "/hand/both/right_can_" + b"can1".hex() + "/joints_skeleton"


def test_both_hands_card_reads_each_hand_on_its_own_interface_into_one_skeleton(ros_runtime):
    ros2, published, _ = ros_runtime
    left, right = FakeClient(), FakeClient([255] * 20)
    plugin, clients = make_plugin({("left", "can0"): left, ("right", "can1"): right}, ros2=ros2)
    try:
        plugin.dispatch("config", {"instance_id": "B", "bound_side": "both", "can_channel": "can0",
                                   "left_can_channel": "", "right_can_channel": "can1"})
        info = plugin.dispatch("start", {"instance_id": "B"})
        assert set(plugin._streams) == {("left", "can0"), ("right", "can1")}
        assert info["bound_side"] == "both" and info["state"] == "running"
        assert (info["left_can_channel"], info["right_can_channel"]) == ("can0", "can1")
        assert [o["topic"] for o in info["topic_out"]][-1] == BOTH_DUAL
        wait_for(lambda: {p["status"]["side"] for t, p in published if t == BOTH_DUAL} == {"left", "right"})
        state = plugin.dispatch("get_state", {"instance_id": "B"})
        assert state["valid"] is True and set(state["hands"]) == {"left", "right"}
    finally:
        plugin.stop()
    assert set(clients) == {("left", "can0"), ("right", "can1")}
    for topic, payload in published:
        if topic == BOTH_DUAL:
            side = payload["status"]["side"]
            assert payload["status"]["can_channel"] == ("can0" if side == "left" else "can1")
            assert all(e["name"].startswith(side + "_") for e in payload["joints"])


def test_both_hands_card_keeps_one_failing_hand_out_of_the_skeleton(ros_runtime):
    ros2, published, _ = ros_runtime
    right = FakeClient()
    right.error = PositionQueryTimeout("位置回复超时", {})
    plugin, _ = make_plugin({("right", "can1"): right}, ros2=ros2)
    args = {"instance_id": "B", "bound_side": "both", "can_channel": "can1"}
    try:
        plugin.dispatch("start", args)
        wait_for(lambda: sum(1 for t, _ in published if t == "/hand/both/left_can_" + b"can1".hex()
                             + "/right_can_" + b"can1".hex() + "/joints_skeleton") >= 2)
        wait_for(lambda: any(t.startswith("/hand/right/") for t, _ in published))
        info = plugin.dispatch("info", args)
    finally:
        plugin.stop()
    skeleton_sides = {p["status"]["side"] for t, p in published if t.startswith("/hand/both/")}
    assert skeleton_sides == {"left"}
    assert info["sample_state"] == "error" and info["sample_valid"] is False
    assert info["hands"]["left"]["sample_state"] == "ok"


def test_both_hands_card_shares_workers_with_single_hand_cards_and_releases_its_own(ros_runtime):
    ros2, published, nodes = ros_runtime
    plugin, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "L", "bound_side": "left"})
        plugin.dispatch("start", {"instance_id": "B", "bound_side": "both"})
        assert set(plugin._streams) == {("left", "can0"), ("right", "can0")}
        left = plugin._streams[("left", "can0")]
        assert set(left._dual_pubs) == {"/hand/both/joints_skeleton"}
        plugin.dispatch("stop", {"instance_id": "B"})
        assert set(plugin._streams) == {("left", "can0")} and left.running()
        assert left._dual_pubs == {} and left.dual_topics == set()
        published.clear()
        wait_for(lambda: any(t == "/hand/left/joints_skeleton" for t, _ in published))
        assert not any(t.startswith("/hand/both/") for t, _ in published)
        # Switching a both-hands card to one side releases the other hand.
        plugin.dispatch("start", {"instance_id": "B", "bound_side": "both"})
        plugin.dispatch("config", {"instance_id": "B", "bound_side": "right"})
        assert set(plugin._streams) == {("left", "can0"), ("right", "can0")}
        assert left._dual_pubs == {} and plugin._streams[("right", "can0")]._dual_pubs == {}
        plugin.dispatch("stop", {"instance_id": "L"})
        plugin.dispatch("stop", {"instance_id": "B"})
        assert plugin._streams == {} and all(n.destroyed for n in nodes)
    finally:
        plugin.stop()


def test_both_hands_card_follows_per_hand_interface_changes(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "B", "bound_side": "both"})
        plugin.dispatch("config", {"instance_id": "B", "right_can_channel": "can1"})
        assert set(plugin._streams) == {("left", "can0"), ("right", "can1")}
        plugin.dispatch("config", {"instance_id": "B", "right_can_channel": ""})
        assert set(plugin._streams) == {("left", "can0"), ("right", "can0")}
        with pytest.raises(ValueError):
            plugin.dispatch("config", {"instance_id": "B", "left_can_channel": "bad name"})
    finally:
        plugin.stop()


def test_both_hands_card_started_directly_keeps_its_interface(ros_runtime):
    ros2, _, _ = ros_runtime
    plugin, _ = make_plugin(ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "B", "bound_side": "both", "can_channel": "can1"})
        info = plugin.dispatch("info", {"instance_id": "B"})
        assert (info["can_channel"], info["left_can_channel"], info["right_can_channel"]) == ("can1",) * 3
        assert info["topic_out"][-1]["topic"] == plugin.dual_topic("can1", "can1")
        # A partial config moves only the hand it names.
        plugin.dispatch("config", {"instance_id": "B", "right_can_channel": "can2"})
        assert set(plugin._streams) == {("left", "can1"), ("right", "can2")}
    finally:
        plugin.stop()


class BlockingClient(FakeClient):
    """Position query that does not return until released."""

    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def get_positions_vector(self, timeout_s=None):
        self.entered.set()
        self.release.wait(10)
        return super().get_positions_vector(timeout_s)


def test_both_hands_state_before_a_hand_finishes_its_first_round_is_waiting(ros_runtime):
    ros2, _, _ = ros_runtime
    left = BlockingClient()
    plugin, _ = make_plugin({("left", "can0"): left}, ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "B", "bound_side": "both"})
        assert left.entered.wait(3)
        wait_for(lambda: plugin._streams[("right", "can0")].latest.get("valid"))
        state = plugin.dispatch("get_state", {"instance_id": "B"})
    finally:
        left.release.set()
        plugin.stop()
    assert state["valid"] is False and state["hands"]["right"]["valid"] is True
    waiting = state["hands"]["left"]
    assert waiting["valid"] is False and waiting["state"] == "waiting" and waiting["reason"]
    assert waiting["sample_state"] == "waiting" and waiting["timestamp"] is None
    assert all(j["raw"] is None for j in waiting["joints"])


def test_stop_that_times_out_keeps_the_card_owning_its_worker(ros_runtime):
    ros2, published, _ = ros_runtime
    client = BlockingClient()
    plugin, _ = make_plugin({("left", "can0"): client}, ros2=ros2, config={"query_timeout_s": 0.02})
    json_topic = plugin.topics("left", "can0")[0]
    try:
        plugin.dispatch("start", {"instance_id": "A", "bound_side": "left"})
        assert client.entered.wait(3)
        worker = plugin._streams[("left", "can0")]
        # Blocked longer than the join timeout: the stop fails...
        with pytest.raises(RuntimeError):
            plugin.dispatch("stop", {"instance_id": "A"})
        # ...but the card still owns the worker, so retrying reaches it.
        assert plugin._streams[("left", "can0")] is worker and worker.running()
        client.release.set()
        plugin.dispatch("stop", {"instance_id": "A"})
        assert plugin._owners == {} and plugin._streams == {} and not worker.running()
        assert [p for t, p in published if t == json_topic and p["state"] == "idle"]
    finally:
        client.release.set()
        plugin.stop()


def test_target_switch_that_cannot_stop_the_old_worker_keeps_it_reachable(ros_runtime):
    ros2, _, _ = ros_runtime
    blocked = BlockingClient()
    plugin, _ = make_plugin({("left", "can0"): blocked}, ros2=ros2,
                            config={"query_timeout_s": 0.02})
    try:
        plugin.dispatch("start", {"instance_id": "A", "bound_side": "left"})
        assert blocked.entered.wait(3)
        abandoned = plugin._streams[("left", "can0")]
        # The switch cannot stop the left worker, so it reports the failure...
        with pytest.raises(RuntimeError):
            plugin.dispatch("config", {"instance_id": "A", "bound_side": "right"})
        # ...and the worker it could not stop stays reachable: a stop of the same
        # card still ends it instead of leaving an ownerless thread behind.
        blocked.release.set()
        plugin.dispatch("stop", {"instance_id": "A"})
        assert plugin._owners == {} and plugin._streams == {}
        assert not abandoned.running()
    finally:
        blocked.release.set()
        plugin.stop()


def test_partially_completed_dual_stop_leaves_the_stopped_hand_to_the_next_card(ros_runtime):
    ros2, _, _ = ros_runtime
    blocked = BlockingClient()
    plugin, _ = make_plugin({("right", "can0"): blocked}, ros2=ros2,
                            config={"query_timeout_s": 0.02})
    try:
        plugin.dispatch("start", {"instance_id": "B", "bound_side": "both"})
        assert blocked.entered.wait(3)
        with pytest.raises(RuntimeError):
            plugin.dispatch("stop", {"instance_id": "B"})
        # The left hand did stop, so B no longer owes anything for it.
        assert set(plugin._streams) == {("right", "can0")}
        # Another card takes the freed hand and stops it: that stop must work.
        plugin.dispatch("start", {"instance_id": "C", "bound_side": "left"})
        plugin.dispatch("stop", {"instance_id": "C"})
        assert set(plugin._streams) == {("right", "can0")}
        blocked.release.set()
        plugin.dispatch("stop", {"instance_id": "B"})
        assert plugin._owners == {} and plugin._streams == {}
    finally:
        blocked.release.set()
        plugin.stop()


def test_start_while_a_hand_is_still_stopping_does_not_take_it_over(ros_runtime):
    ros2, _, _ = ros_runtime
    blocked = BlockingClient()
    plugin, _ = make_plugin({("left", "can0"): blocked}, ros2=ros2,
                            config={"query_timeout_s": 0.02})
    try:
        plugin.dispatch("start", {"instance_id": "A", "bound_side": "left"})
        assert blocked.entered.wait(3)
        with pytest.raises(RuntimeError):
            plugin.dispatch("stop", {"instance_id": "A"})
        # The worker is still shutting down: a second card must not treat it as
        # its own running stream.
        with pytest.raises(RuntimeError):
            plugin.dispatch("start", {"instance_id": "B", "bound_side": "left"})
        assert "B" not in plugin._owners and "B" not in plugin._channels
        blocked.release.set()
        plugin.dispatch("stop", {"instance_id": "A"})
        plugin.dispatch("start", {"instance_id": "B", "bound_side": "left"})
        assert plugin._owners["B"][0] == (("left", "can0"),)
        assert plugin._streams[("left", "can0")].running()
    finally:
        blocked.release.set()
        plugin.stop()


def test_driver_stop_keeps_a_worker_it_cannot_stop_and_its_can_client(ros_runtime, monkeypatch):
    ros2, _, _ = ros_runtime
    blocked = BlockingClient()
    ctrl = mock.Mock(get=mock.Mock(return_value=blocked))
    monkeypatch.setattr(hand_device, "GLOBAL_CONTROLLERS", {("left", "can0"): ctrl})
    import joints_device
    monkeypatch.setattr(joints_device, "GLOBAL_CONTROLLERS", hand_device.GLOBAL_CONTROLLERS)
    hand = SimpleNamespace(hand_config={}, config=GLOBAL_CONFIG, _lock=threading.RLock(), _controllers={})
    plugin = JointsPlugin(hand, {"poll_hz": 30, "query_timeout_s": 0.02}, global_config=GLOBAL_CONFIG, ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "A", "bound_side": "left"})
        assert blocked.entered.wait(3)
        worker = plugin._streams[("left", "can0")]
        # The worker is blocked beyond the join timeout: the driver stop says so...
        with pytest.raises(RuntimeError):
            plugin.stop()
        # ...keeps the worker reachable and does not close the client under it.
        assert plugin._streams.get(("left", "can0")) is worker and worker.running()
        ctrl.close.assert_not_called()
        blocked.release.set()
        # The card's own stop still reaches it once the query returns.
        plugin.dispatch("stop", {"instance_id": "A"})
        assert plugin._streams == {} and not worker.running()
        ctrl.close.assert_called_once()
    finally:
        blocked.release.set()
        plugin.stop()


def test_unknown_action_is_left_to_the_mcp_handler_as_an_error():
    plugin, clients = make_plugin()
    assert plugin.dispatch("calibrate", {"instance_id": "A", "bound_side": "left"}) is None
    assert clients == {}


def test_driver_does_not_start_acquisition_on_boot():
    plugin, clients = make_plugin()
    plugin.start()
    time.sleep(0.05)
    assert plugin._streams == {} and clients == {}


# ── integration with the other cards ───────────────────────────────────

def test_shipped_config_registers_joints_after_existing_cards():
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    plugins = build_plugins(config, "", None)
    assert [p.PREFIX for p in plugins] == ["hand", "hand_gesture", "hand_grasp", "hand_tactile", "joints"]
    joints = plugins[-1]
    assert joints.hand is plugins[0] and joints.poll_hz == config["plugins"]["joints"]["poll_hz"]


def test_disabled_joints_section_registers_nothing():
    config = yaml.safe_load((DRIVER / "config.yaml").read_text())
    config["plugins"]["joints"]["enabled"] = False
    assert "joints" not in [p.PREFIX for p in build_plugins(config, "", None)]


def test_plugin_stop_closes_only_controllers_the_hand_card_does_not_own(monkeypatch):
    owned, own = mock.Mock(), mock.Mock()
    monkeypatch.setattr(hand_device, "GLOBAL_CONTROLLERS", {("left", "can0"): owned, ("right", "can1"): own})
    import joints_device
    monkeypatch.setattr(joints_device, "GLOBAL_CONTROLLERS", hand_device.GLOBAL_CONTROLLERS)
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


def test_last_card_to_release_a_hand_closes_the_can_client_it_opened(ros_runtime, monkeypatch):
    ros2, _, _ = ros_runtime
    clients = {}

    def controller(target):
        client = clients[target] = FakeClient()
        return mock.Mock(get=mock.Mock(return_value=client))

    left, right, other = controller(("left", "can0")), controller(("right", "can0")), controller(("left", "can1"))
    monkeypatch.setattr(hand_device, "GLOBAL_CONTROLLERS",
                        {("left", "can0"): left, ("right", "can0"): right, ("left", "can1"): other})
    import joints_device
    monkeypatch.setattr(joints_device, "GLOBAL_CONTROLLERS", hand_device.GLOBAL_CONTROLLERS)
    # The hand card already uses the right hand for its own actions.
    hand = SimpleNamespace(hand_config={}, config=GLOBAL_CONFIG, _lock=threading.RLock(),
                           _controllers={("right", "can0"): right})
    plugin = JointsPlugin(hand, {"poll_hz": 30}, global_config=GLOBAL_CONFIG, ros2=ros2)
    try:
        plugin.dispatch("start", {"instance_id": "A", "bound_side": "left"})
        plugin.dispatch("start", {"instance_id": "B", "bound_side": "left"})
        wait_for(lambda: clients[("left", "can0")].calls)
        plugin.dispatch("stop", {"instance_id": "A"})
        left.close.assert_not_called()  # B still reads this hand
        plugin.dispatch("stop", {"instance_id": "B"})
        left.close.assert_called_once()
        # Moving a card to another interface closes the client it left behind.
        plugin.dispatch("start", {"instance_id": "A", "bound_side": "left", "can_channel": "can1"})
        plugin.dispatch("config", {"instance_id": "A", "can_channel": "can0"})
        other.close.assert_called_once()
        plugin.dispatch("stop", {"instance_id": "A"})
        assert left.close.call_count == 2
        # A one-off read on an idle hand does not leave its client open either.
        assert plugin.dispatch("get_state", {"instance_id": "C", "bound_side": "left"})["valid"] is True
        assert left.close.call_count == 3
        # A hand the hand card uses is never closed by this card.
        plugin.dispatch("start", {"instance_id": "R", "bound_side": "right"})
        plugin.dispatch("stop", {"instance_id": "R"})
        plugin.dispatch("get_state", {"instance_id": "R"})
    finally:
        plugin.stop()
    right.close.assert_not_called()
    assert plugin._used_targets == set()
