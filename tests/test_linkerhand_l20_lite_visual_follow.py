"""Offline checks for the L20 Lite continuous visual-follow card.

No serial port, ROS node or hand is involved: the stream admission and the
pose interpolation are plain functions, and the Modbus client is injected.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "linkerhand" / "l20_lite"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(DRIVER))

from common.vendor_runtime import DriverBundle, load_config  # noqa: E402
from lite_tactile import FINGER_NAMES  # noqa: E402
from modbus_client import TACTILE_CELL_COUNT  # noqa: E402
from modbus_device import build_plugins  # noqa: E402
from visual_follow import (VisualFollowPlugin, clamp_step,  # noqa: E402
                           target_from_flexions)

OPEN = [254, 254, 254, 254, 254, 254, 127, 126, 127, 254]
FIST = [93, 255, 0, 0, 0, 0, 128, 128, 128, 255]


class FakeClient:
    def __init__(self, positions=None):
        self.positions = list(positions or OPEN)
        self.writes = []
        self.torques = []
        self.speeds = []

    def get_positions(self):
        return list(self.positions)

    def set_positions(self, values):
        self.writes.append(list(values))
        self.positions = list(values)

    def set_torques(self, values):
        self.torques.append(list(values))

    def set_speeds(self, values):
        self.speeds.append(list(values))


class FakeConnection:
    def __init__(self, client, side="right", port="/dev/fake"):
        self._client = client
        self.side = side
        self.port = port

    def get(self, side=None):
        return self._client

    def close(self):
        pass


class FakeHand:
    def __init__(self, sides=None):
        self.config = {"side": "right", "serial_port": "/dev/fake"}
        self._bus_lock = threading.RLock()
        self.sides = sides or {"right": "/dev/fake"}

    def _discover(self):
        return dict(self.sides)


def make_plugin(sides=None):
    cfg = {
        "right_open_pose": OPEN, "right_fist_pose": FIST,
        "left_open_pose": OPEN, "left_fist_pose": FIST,
        "max_hz": 10, "max_step": 8, "watchdog_ms": 600,
        "max_obs_age_ms": 500,
    }
    return VisualFollowPlugin(FakeHand(sides), cfg)


def sample(**changes):
    payload = {
        "schema": "motus.hand/1",
        "stamp_ms": int(time.time() * 1000),
        "obs_stamp_ms": int(time.time() * 1000),
        "handedness": "Right",
        "flexions": {name: 0.0 for name in
                     ("thumb", "index", "middle", "ring", "pinky")},
    }
    payload.update(changes)
    return payload


def start_with_fake_client(plugin, client=None):
    client = client or FakeClient()

    def connection(instance_id, side, port):
        made = FakeConnection(client, side, port)
        plugin._connections[instance_id] = made
        return made

    plugin._connection = connection
    plugin._subscribe = lambda topic: None
    result = plugin.dispatch("start", {
        "instance_id": "card1", "input_topic": "/actucore/handdrive/keypoints",
        "confirm_motion": True,
    })
    assert result["state"] == "armed", result
    return client


# ── registration and geometry ───────────────────────────────────────────────

def test_card_is_registered_with_a_control_input():
    full = load_config(DRIVER / "config.yaml")
    plugins = build_plugins(full, "", None)
    tools = {item["name"]: item for item in DriverBundle(plugins).get_all_tools()}
    assert "hand_visual_follow" in tools
    tool = tools["hand_visual_follow"]
    assert tool["type"] == "actuator"
    assert tool["topic_in"][0]["format"] == "control/hand_flex"
    assert tool["inputSchema"]["x-is-dangerous"] is True
    enum = tool["inputSchema"]["properties"]["action"]["enum"]
    assert "start" in enum and "stop" in enum and "info" in enum


def test_flexions_interpolate_calibrated_poses_only_on_flexion_joints():
    curl = {"thumb": 1.0, "index": 0.5, "middle": 0.0,
            "ring": 1.0, "pinky": 0.25}
    target = target_from_flexions(curl, OPEN, FIST)
    # Four fingers still interpolate their own curl, unchanged.
    assert target[2] == round((OPEN[2] + FIST[2]) / 2)  # index root
    assert target[3] == OPEN[3]              # middle root
    assert target[4] == FIST[4]              # ring root
    assert target[6] == OPEN[6] and target[7] == OPEN[7]  # finger sway held
    # Thumb is driven by the MEAN four-finger grasp, not its own curl.
    # mean(0.5,0,1,0.25)=0.4375 -> just past onset 0.35 -> small opposition.
    assert target[0] < OPEN[0]               # root started to bend
    assert target[1] < OPEN[1]               # sway started to oppose


def test_clamp_step_limits_every_joint():
    assert clamp_step([0] * 10, [100] * 10, 8) == [8] * 10
    assert clamp_step([100] * 10, [0] * 10, 8) == [92] * 10


# ── admission and safety ────────────────────────────────────────────────────

def test_start_disarms_and_resume_is_the_confirmation_gate():
    plugin = make_plugin()
    result = plugin.dispatch("start", {"instance_id": "card1",
                                       "confirm_motion": True})
    assert result["state"] == "error"
    assert "input_topic" in result["error"]

    plugin._connection = lambda i, s, p: FakeConnection(FakeClient(), s, p)
    plugin._subscribe = lambda topic: None
    result = plugin.dispatch("start", {"instance_id": "card1",
                                       "input_topic": "/x"})
    assert result["state"] == "armed", result          # no confirm needed
    assert plugin._streams["card1"]["paused"] is True
    assert plugin.accept(sample()) is False            # data ignored while armed

    # Canvas renders the confirm_motion field for resume via x-action-params.
    # A missing or false confirmation must be refused; true energizes.
    result = plugin.dispatch("resume", {"instance_id": "card1"})
    assert result["state"] == "error", result
    assert "confirm" in result["error"]
    assert plugin._streams["card1"]["paused"] is True
    result = plugin.dispatch("resume", {"instance_id": "card1",
                                        "confirm_motion": False})
    assert result["state"] == "error", result
    result = plugin.dispatch("resume", {"instance_id": "card1",
                                        "confirm_motion": True})
    assert result["state"] == "running", result
    assert plugin._streams["card1"]["paused"] is False


def test_accept_writes_a_clamped_command_and_rejects_wrong_data():
    plugin = make_plugin()
    client = start_with_fake_client(plugin)
    plugin.dispatch("resume", {"instance_id": "card1", "confirm_motion": True})
    curl = dict(sample()["flexions"])
    curl["index"] = 1.0
    assert plugin.accept(sample(flexions=curl)) is True
    assert len(client.writes) == 1
    assert client.writes[0][2] == OPEN[2] - 8     # clamped from 254 towards 0

    before = len(client.writes)
    assert plugin.accept(sample(schema="motus.hand/2")) is False
    assert plugin.accept(sample(handedness="Left")) is False
    assert plugin.accept(sample(obs_stamp_ms=int(time.time() * 1000) - 5000)) is False
    assert plugin.accept(sample(flexions={"thumb": 1.4})) is False
    assert len(client.writes) == before


def test_watchdog_releases_torque_then_auto_resumes_on_fresh_data():
    plugin = make_plugin()
    client = start_with_fake_client(plugin)
    plugin.dispatch("resume", {"instance_id": "card1", "confirm_motion": True})
    assert plugin.accept(sample()) is True
    stream = plugin._streams["card1"]
    # Watchdog silence is now owned by the shared ControlSink; advance its
    # monotonic clock past the configured window instead of poking the old
    # stream field. The wall clock stays real so fresh-data checks still pass.
    sink = stream["sink"]
    real_clock = sink._clock
    fake = {"now": real_clock() + plugin.watchdog_ms + 1000}
    sink._clock = lambda: fake["now"]
    try:
        plugin._on_watchdog()
        assert "card1" in plugin._streams      # subscription kept alive
        assert stream["starved"] is True
        assert client.torques[-1] == [0] * 10  # torque released while silent
    finally:
        sink._clock = real_clock
    # Fresh valid data re-energizes and resumes (canvas has no re-confirm UI).
    stream["last_command_monotonic"] = 0.0
    curl = dict(sample()["flexions"]); curl["index"] = 1.0
    assert plugin.accept(sample(flexions=curl)) is True
    assert stream["starved"] is False
    assert client.torques[-1] != [0] * 10      # torque re-applied


def test_pause_releases_torque_and_resume_needs_confirmation():
    plugin = make_plugin()
    client = start_with_fake_client(plugin)
    plugin.dispatch("resume", {"instance_id": "card1", "confirm_motion": True})
    assert plugin.accept(sample()) is True
    assert plugin.dispatch("pause", {"instance_id": "card1"})["state"] == "paused"
    assert client.torques[-1] == [0] * 10
    assert plugin.accept(sample()) is False
    # resume requires explicit confirmation now that the canvas can pass it.
    assert plugin.dispatch("resume", {"instance_id": "card1"})["state"] == "error"
    assert plugin.dispatch("resume", {"instance_id": "card1",
                                      "confirm_motion": False})["state"] == "error"
    assert plugin.dispatch("resume", {"instance_id": "card1",
                                      "confirm_motion": True})["state"] == "running"


def test_confirmation_accepts_canvas_string_but_nothing_else():
    """The canvas may send "true" as a string; that is still an explicit yes."""
    from modbus_device import confirm_motion_ok

    assert confirm_motion_ok({"confirm_motion": True}) is True
    assert confirm_motion_ok({"confirm_motion": "true"}) is True
    for bad in ("false", "True", "TRUE", "1", 1, None, "", []):
        assert confirm_motion_ok({"confirm_motion": bad}) is False
    assert confirm_motion_ok({}) is False


def test_resume_accepts_canvas_string_confirmation():
    plugin = make_plugin()
    plugin._connection = lambda i, s, p: FakeConnection(FakeClient(), s, p)
    plugin._subscribe = lambda topic: None
    plugin.dispatch("start", {"instance_id": "card1",
                              "input_topic": "/actucore/handdrive/keypoints"})
    result = plugin.dispatch("resume", {"instance_id": "card1",
                                        "confirm_motion": "true"})
    assert result["state"] == "running", result


def test_canvas_action_without_instance_id_finds_the_only_stream():
    """Canvas custom actions omit instance_id; project start supplies it."""
    plugin = make_plugin()
    client = FakeClient()
    plugin._connection = lambda i, s, p: FakeConnection(client, s, p)
    plugin._subscribe = lambda topic: None
    plugin.dispatch("start", {"instance_id": "canvas-card-id",
                              "input_topic": "/actucore/handdrive/keypoints"})
    assert plugin.dispatch("info", {})["state"] == "armed"
    result = plugin.dispatch("resume", {"confirm_motion": "true"})
    assert result["state"] == "running", result
    assert plugin._streams["canvas-card-id"]["paused"] is False

    # Two streams must not be guessed.
    plugin2 = make_plugin()
    plugin2._streams["a"] = {"paused": True, "side": "left"}
    plugin2._streams["b"] = {"paused": True, "side": "left"}
    assert plugin2._resolve_instance("") == ""


def test_throttled_frames_still_keep_watchdog_alive():
    """A 10 Hz source must not look dead just because write slots were missed."""
    plugin = make_plugin()
    start_with_fake_client(plugin)
    plugin.dispatch("resume", {"instance_id": "card1", "confirm_motion": True})
    stream = plugin._streams["card1"]
    # Force every sample to be throttled, then advance past watchdog_ms.
    stream["last_command_monotonic"] = time.monotonic()
    for _ in range(8):
        assert plugin.accept(sample()) is False
        stream["last_command_monotonic"] = time.monotonic()  # keep throttling
    assert "throttled" in plugin._rejects
    plugin._on_watchdog()
    assert "card1" in plugin._streams, "throttled-but-fresh frames must not trip watchdog"


def test_thumb_is_fully_independent_of_four_fingers():
    """A thumbs-up (four fingers curled, thumb out) must NOT bend the thumb.

    Thumb follows only its own vision curl; the four fingers never drive it.
    """
    plugin = make_plugin()
    plugin.config['fist_reference'] = 1.0
    plugin.config['max_step'] = 64
    client = FakeClient()
    import visual_follow as vf
    p2 = vf.VisualFollowPlugin(plugin.hand, plugin.config)

    def connection(instance_id, side, port):
        made = FakeConnection(client, side, port)
        p2._connections[instance_id] = made
        return made
    p2._connection = connection
    p2._subscribe = lambda topic: None
    p2.dispatch('start', {'instance_id': 'card1', 'input_topic': '/x'})
    p2.dispatch('resume', {'instance_id': 'card1', 'confirm_motion': True})

    # Thumbs-up: four fingers fully curled, thumb vision = 0 -> thumb stays open.
    payload = sample()
    payload['flexions'] = dict(payload['flexions'])
    payload['flexions']['thumb'] = 0.0
    for n in ("index", "middle", "ring", "pinky"):
        payload['flexions'][n] = 1.0
    assert p2.accept(payload) is True, p2._rejects[-3:]
    target = p2._streams['card1']['target']
    assert target[0] == 254 and target[1] == 254 and target[9] == 254, \
        f"thumbs-up must keep thumb open, got {target[0]},{target[1]},{target[9]}"

    # Thumb curls on its own (even with four fingers open) -> thumb moves.
    payload['flexions']['thumb'] = 1.0
    for n in ("index", "middle", "ring", "pinky"):
        payload['flexions'][n] = 0.0
    p2._streams['card1']['last_command_monotonic'] = 0.0
    assert p2.accept(payload) is True, p2._rejects[-3:]
    t2 = p2._streams['card1']['target']
    # Thumb must move away from open on all three joints. With the travel
    # limit (thumb_travel=0.6) it stops short of the collision pose, so do
    # not assert full extension here -- just that it bent noticeably.
    assert t2[0] < 160 and t2[1] < 160 and t2[9] < 230, \
        f"thumb curl must bend thumb, got {t2[0]},{t2[1]},{t2[9]}"

def test_speed_is_capped_at_verified_max_and_thumb_is_geared_down():
    """Never exceed the operator-verified speed; thumb keeps its gear ratio."""
    import visual_follow as vf
    speeds = vf.joint_speeds(110)
    assert speeds[2] == 110                      # fingers at full verified speed
    assert speeds[0] < 110 and speeds[1] < 110 and speeds[9] < 110
    assert speeds[0] == round(110 * 29.11 / 63.11)

    plugin = make_plugin()
    plugin.config['speed'] = 255
    import pytest as _pytest
    with _pytest.raises(ValueError, match="verified maximum"):
        vf.VisualFollowPlugin(plugin.hand, plugin.config)


def test_deadband_ignores_jitter_but_slow_drift_eventually_moves():
    """10 deg deadband vs last accepted pose, not vs previous frame."""
    plugin = make_plugin()
    plugin.config['angle_deadband_deg'] = 10.0
    plugin.config['thumb_gain'] = 1.0
    plugin.config['fist_reference'] = 1.0
    plugin.config['synergy_weight'] = 0.0
    import visual_follow as vf
    p = vf.VisualFollowPlugin(plugin.hand, plugin.config)
    client = FakeClient()

    def connection(i, side, port):
        made = FakeConnection(client, side, port)
        p._connections[i] = made
        return made
    p._connection = connection
    p._subscribe = lambda t: None
    p.dispatch('start', {'instance_id': 'card1', 'input_topic': '/x'})
    p.dispatch('resume', {'instance_id': 'card1', 'confirm_motion': True})

    base = sample()
    assert p.accept(base) is True                    # first pose accepted
    n0 = p._streams['card1']['commands']

    # small jitter (5 deg = 0.028) below deadband -> rejected, no write
    jitter = sample()
    jitter['flexions'] = dict(base['flexions'])
    jitter['flexions']['index'] = 0.028
    p._streams['card1']['last_command_monotonic'] = 0.0  # bypass throttle
    assert p.accept(jitter) is False
    assert 'deadband' in p._rejects[-3:]
    assert p._streams['card1']['commands'] == n0

    # slow drift: each frame +6 deg (<10) but accumulates past threshold
    # from the last ACCEPTED pose -> must eventually move
    for i in range(1, 4):
        drift = sample()
        drift['flexions'] = dict(base['flexions'])
        drift['flexions']['index'] = 0.0556 * i
        p._streams['card1']['last_command_monotonic'] = 0.0  # bypass throttle
        assert p.accept(drift) is True, 'slow drift must eventually pass deadband'


def test_synergy_mapping_maps_verified_full_fist_to_fist_pose():
    """Human fist (vision=0.6) must reach robot fist, not stop halfway."""
    import visual_follow as vf
    fist = {'thumb': 0.84, 'index': 0.60, 'middle': 0.60,
            'ring': 0.60, 'pinky': 0.58}
    out = vf.normalized_grasp(fist, fist_reference=0.6, synergy_weight=0.6)
    assert all(v >= 0.95 for v in out.values()), out
    target = vf.target_from_flexions(out, OPEN, FIST, gamma=1.0)
    assert target[2] <= FIST[2] + 2  # index root effectively closed
    assert target[3] <= FIST[3] + 2  # middle root effectively closed

    # A single finger moving alone must stay independent: adaptive synergy
    # drops to zero when the spread is large, so no cross-finger coupling.
    single = dict(fist); single['index'] = 0.20
    fused = vf.normalized_grasp(single, fist_reference=0.6, synergy_weight=0.6)
    assert fused['index'] < 0.5, fused          # index stays independent
    assert fused['middle'] > 0.9, fused         # others not dragged by it

    # Whole-hand grasp stays smoothed (all four close together).
    together = vf.normalized_grasp(fist, fist_reference=0.6, synergy_weight=0.6)
    assert min(together.values()) > 0.9, together


def test_per_finger_deadband_keeps_jittering_finger_frozen():
    """One finger moving must not drag a jittering neighbour's motor."""
    plugin = make_plugin()
    plugin.config['angle_deadband_deg'] = 10.0
    plugin.config['fist_reference'] = 1.0
    plugin.config['synergy_weight'] = 0.0
    import visual_follow as vf
    p = vf.VisualFollowPlugin(plugin.hand, plugin.config)
    client = FakeClient()
    def connection(i, side, port):
        made = FakeConnection(client, side, port)
        p._connections[i] = made
        return made
    p._connection = connection
    p._subscribe = lambda t: None
    p.dispatch('start', {'instance_id': 'card1', 'input_topic': '/x'})
    p.dispatch('resume', {'instance_id': 'card1', 'confirm_motion': True})

    base = sample()
    assert p.accept(base) is True
    idx_before = client.writes[-1][2]     # index_root
    mid_before = client.writes[-1][3]     # middle_root

    # index bends a lot, middle only jitters 0.02 (<10deg=0.0556)
    moved = sample()
    moved['flexions'] = dict(base['flexions'])
    moved['flexions']['index'] = 0.5
    moved['flexions']['middle'] = 0.02
    p._streams['card1']['last_command_monotonic'] = 0.0
    assert p.accept(moved) is True
    cmd = client.writes[-1]
    assert cmd[2] != idx_before, 'index must move'
    assert cmd[3] == mid_before, 'jittering middle must stay frozen'


def test_thumb_palm_plane_mapping_is_graded_and_monotonic():
    """Thumb bend must be graded across the whole range, not just 0/1."""
    import numpy as np
    import visual_follow as vf
    # Self-contained fixtures: 21-point open/fist poses. Kept here so the
    # test runs in any checkout/CI without an author-specific local path.
    OPEN = np.array([
        [0.50, 0.90, 0.0],
        [0.42, 0.78, 0.0], [0.35, 0.68, 0.0], [0.28, 0.60, 0.0], [0.22, 0.52, 0.0],
        [0.45, 0.72, 0.0], [0.44, 0.58, 0.0], [0.43, 0.47, 0.0], [0.42, 0.36, 0.0],
        [0.51, 0.71, 0.0], [0.51, 0.55, 0.0], [0.51, 0.44, 0.0], [0.51, 0.31, 0.0],
        [0.57, 0.72, 0.0], [0.58, 0.57, 0.0], [0.59, 0.46, 0.0], [0.59, 0.34, 0.0],
        [0.62, 0.75, 0.0], [0.64, 0.62, 0.0], [0.65, 0.53, 0.0], [0.66, 0.44, 0.0],
    ], dtype=np.float32)
    FIST = np.array([
        [0.50, 0.90, 0.0],
        [0.42, 0.78, 0.0], [0.37, 0.70, 0.0], [0.39, 0.66, 0.0], [0.44, 0.70, 0.0],
        [0.45, 0.72, 0.0], [0.46, 0.66, 0.0], [0.47, 0.69, 0.0], [0.46, 0.71, 0.0],
        [0.51, 0.71, 0.0], [0.52, 0.65, 0.0], [0.53, 0.68, 0.0], [0.52, 0.70, 0.0],
        [0.57, 0.72, 0.0], [0.58, 0.66, 0.0], [0.59, 0.69, 0.0], [0.58, 0.71, 0.0],
        [0.62, 0.75, 0.0], [0.64, 0.69, 0.0], [0.65, 0.72, 0.0], [0.64, 0.74, 0.0],
    ], dtype=np.float32)
    weights = []
    for k in (0.0, 0.25, 0.5, 0.75, 1.0):
        lm = (OPEN * (1 - k) + FIST * k).tolist()
        weights.append(vf.thumb_opposition_from_landmarks(lm))
    assert weights[0] < 0.05, f"open must not oppose: {weights}"
    assert weights[-1] > 0.95, f"fist must fully oppose: {weights}"
    # Each step grows -- graded control across the full range.
    for i in range(1, len(weights)):
        assert weights[i] > weights[i-1], f"not monotonic: {weights}"
    # Roughly linear (no big dead zone or jump).
    assert 0.15 < weights[1] < 0.45, weights
    assert 0.45 < weights[2] < 0.75, weights
    assert 0.75 < weights[3] < 1.0, weights

    # Bad input fails safe (no motion).
    assert vf.thumb_opposition_from_landmarks([[0,0,0]]*5) == 0.0


def test_accept_with_landmarks_drives_four_fingers_without_crashing():
    """Regression: a payload carrying landmarks must still move the 4 fingers.

    The landmark-based thumb path must use the current function signature;
    an argument mismatch here previously killed the whole frame.
    """
    import visual_follow as vf
    plugin = make_plugin()
    p = vf.VisualFollowPlugin(plugin.hand, plugin.config)
    client = FakeClient()
    def connection(instance_id, side, port):
        made = FakeConnection(client, side, port)
        p._connections[instance_id] = made
        return made
    p._connection = connection
    p._subscribe = lambda topic: None
    p.dispatch('start', {'instance_id': 'card1', 'input_topic': '/x'})
    p.dispatch('resume', {'instance_id': 'card1', 'confirm_motion': True})

    payload = sample()
    payload['flexions'] = dict(payload['flexions'])
    for n in ("index", "middle", "ring", "pinky"):
        payload['flexions'][n] = 1.0
    # 21 landmarks with a plausible layout (palm + thumb + finger tips).
    lm = [[0.5, 0.9, 0]] * 21
    for i, xy in {0:(0.5,0.9),5:(0.4,0.7),9:(0.5,0.7),13:(0.6,0.7),
                  17:(0.7,0.7),3:(0.35,0.78),4:(0.32,0.68)}.items():
        lm[i] = [xy[0], xy[1], 0]
    for i in (8,12,16,20):
        lm[i] = [0.5, 0.3, 0]
    payload['landmarks'] = lm

    assert p.accept(payload) is True, p._rejects[-5:]
    target = p._streams['card1']['target']
    # Four fingers must be driven to fist despite the landmark path running.
    assert target[2] == 0 and target[3] == 0 and target[4] == 0 and target[5] == 0, (
        f"four fingers did not move: {target}")


def test_resume_exposes_confirm_motion_param_to_canvas():
    """The canvas must render a confirm_motion field for the resume action.

    Without x-action-params the dangerous resume cannot be confirmed from the
    canvas (ACTION dropdown only), so the safety gate would deadlock.
    """
    plugin = make_plugin()
    definition = plugin.get_tool()
    schema = definition.get("inputSchema", definition.get("schema", {}))
    params = schema.get("x-action-params", {})
    assert "resume" in params
    assert "confirm_motion" in params["resume"].get("params", [])
    prop = schema.get("properties", {}).get("confirm_motion", {})
    assert prop.get("type") == "boolean"
    assert prop.get("const") is None, "const hides the field; must be editable"


def test_thumb_travel_limits_all_three_joints_without_touching_fingers():
    """thumb_travel shrinks the thumb's endpoint but leaves four fingers intact."""
    import visual_follow as vf
    full = {n: 1.0 for n in vf.FLEXION_JOINTS}
    t_full = vf.target_from_flexions(full, OPEN, FIST, thumb_root_fist=0,
                                     thumb_override=1.0, thumb_travel=1.0)
    t_half = vf.target_from_flexions(full, OPEN, FIST, thumb_root_fist=0,
                                     thumb_override=1.0, thumb_travel=0.5)
    # Thumb joints are pulled back toward open when travel is reduced.
    for i in (0, 1, 9):
        assert t_half[i] > t_full[i], (i, t_half[i], t_full[i])
    # Four fingers are byte-identical regardless of thumb_travel.
    for i in (2, 3, 4, 5, 6, 7, 8):
        assert t_half[i] == t_full[i], (i, t_half[i], t_full[i])
    # travel=0 -> no thumb motion at all.
    t_zero = vf.target_from_flexions(full, OPEN, FIST, thumb_root_fist=0,
                                     thumb_override=1.0, thumb_travel=0.0)
    assert t_zero[0] == OPEN[0] and t_zero[1] == OPEN[1] and t_zero[9] == OPEN[9]


def test_sway_roll_travel_are_independent_and_spare_fingers():
    """Sway/roll can be limited more than root to avoid thumb-index overlap."""
    import visual_follow as vf
    full = {n: 1.0 for n in vf.FLEXION_JOINTS}
    base = dict(thumb_root_fist=0, thumb_override=1.0, thumb_travel=0.6)
    # Same root travel, but smaller sway/roll -> thumb rolls/sweeps less.
    t_a = vf.target_from_flexions(full, OPEN, FIST, **base,
                                  thumb_sway_travel=0.5, thumb_roll_travel=0.4)
    t_b = vf.target_from_flexions(full, OPEN, FIST, **base,
                                  thumb_sway_travel=0.3, thumb_roll_travel=0.2)
    assert t_a[0] == t_b[0], "root must not change"
    assert t_b[1] > t_a[1], "smaller sway travel -> sway pulled toward open"
    assert t_b[9] > t_a[9], "smaller roll travel -> roll pulled toward open"
    for i in (2, 3, 4, 5, 6, 7, 8):
        assert t_a[i] == t_b[i], "four fingers must be untouched"


def test_input_declares_control_hand_flex_format():
    """Driver must declare the same control format as hand_landmarks output."""
    plugin = make_plugin()
    definition = plugin.get_tool()
    tin = definition.get('topic_in', [])
    assert any(t.get('format') == 'control/hand_flex' for t in tin), tin
    assert not any(t.get('format') == 'data/json' for t in tin)


# ── tactile read path (regression: deleted MultiHandPlugin.read_tactile) ────

class FakeTactileClient:
    """Modbus stand-in that answers one 96-cell read per fingertip."""

    def __init__(self):
        self.reads = []

    def read_fingertip(self, finger, settle_s=0.03, *, deadline=None):
        self.reads.append(finger)
        return [FINGER_NAMES.index(finger) + 1] * TACTILE_CELL_COUNT


class FakeTactileConnection:
    def __init__(self, client, config):
        self._client = client
        self.port = config.get("serial_port", "/dev/fake")
        self.side = config.get("side", "right")
        self.closed = False

    def get(self, side=None):
        return self._client

    def detect_sides(self, **_kwargs):
        return [self.side]

    def close(self):
        self.closed = True


def build_tactile_plugins(monkeypatch, tmp_path, *, client=None, sides=("right",)):
    """Build hand + tactile cards with a fake RS-485 descriptor."""
    import modbus_device as md

    port = tmp_path / "fake-rs485"
    port.write_text("")
    client = client or FakeTactileClient()
    cfg = {
        "side": "right",
        "serial_port": str(port),
        "plugins": {
            "hand": {
                "enabled": True,
                "open_pose": OPEN, "fist_pose": FIST,
                "left_open_pose": OPEN, "left_fist_pose": FIST,
            },
            "hand_tactile": {"enabled": True},
        },
    }
    hand, tactile = build_plugins(cfg, "", None)

    def connection(conf):
        made = FakeTactileConnection(client, conf)
        if made.side not in sides:
            made.detect_sides = lambda **_kwargs: []
        return made

    monkeypatch.setattr(md, "L20LiteConnection", connection)
    hand._ports = lambda: [str(port)]
    return hand, tactile, client, str(port)


def test_tactile_card_reads_through_multi_hand_plugin(monkeypatch, tmp_path):
    """The tactile card must reach read_tactile on MultiHandPlugin.

    This is the exact call site that raised AttributeError after the
    multi-instance refactor deleted MultiHandPlugin.read_tactile.
    """
    hand, tactile, client, port = build_tactile_plugins(monkeypatch, tmp_path)
    assert tactile.hand is hand
    assert hasattr(hand, "read_tactile")

    data = tactile._read_tactile_snapshot("right")
    assert data["state"] == "ok", data["errors"]
    assert data["serial_port"] == port
    assert data["scalar_valid"] is True
    assert all(value is not None for value in data["touch_scalar"].values())
    assert all(len(cells) == TACTILE_CELL_COUNT
               for cells in data["touch_cells"].values())
    assert client.reads == FINGER_NAMES           # one select+read per finger
    assert hand._tactile_ports["right"] == port   # discovered port cached

    # Direct calls use the same contract and keep the cached port.
    read_port, results = hand.read_tactile("right", FINGER_NAMES,
                                           budget_s=1.0, settle_s=0.0)
    assert read_port == port
    assert set(results) == set(FINGER_NAMES)
    assert all(isinstance(value, tuple) and len(value) == 2
               for value in results.values())
    assert client.reads == FINGER_NAMES * 2


def test_tactile_discovery_retry_and_cancel(monkeypatch, tmp_path):
    """A missing port is probed at most once per 2 s; cancel aborts a read."""
    hand, _, client, _ = build_tactile_plugins(monkeypatch, tmp_path, sides=())
    for _ in range(2):
        with pytest.raises(ValueError, match="未连接"):
            hand.read_tactile("right", FINGER_NAMES, budget_s=0.5, settle_s=0.0)
    assert hand._tactile_retry_at["right"] > time.monotonic()
    assert client.reads == []

    hand, _, client, port = build_tactile_plugins(monkeypatch, tmp_path)
    hand._tactile_ports["right"] = port
    cancel = threading.Event()
    cancel.set()
    with pytest.raises(InterruptedError):
        hand.read_tactile("right", FINGER_NAMES, budget_s=1.0, settle_s=0.0,
                          cancel=cancel)
    assert client.reads == []


def test_shared_control_sink_owns_admission():
    """accept() must route through the shared ControlSink, not a private chain.

    A malformed control envelope (bad schema/mode/dof or stale stamp) must be
    dropped by the sink before any valve/motor write, exactly as every other
    motus.control/1 servo path does.
    """
    from common.control import parse_descriptor
    import visual_follow as vf

    # The card's descriptor is a valid shared-control descriptor.
    desc = parse_descriptor(vf.FLEX_DESCRIPTOR)
    assert desc.dof == 5

    plugin = make_plugin()
    client = start_with_fake_client(plugin)
    plugin.dispatch("resume", {"instance_id": "card1", "confirm_motion": True})
    stream = plugin._streams["card1"]
    sink = stream["sink"]
    assert sink is not None and sink.descriptor.dof == 5

    before = len(client.writes)
    # Stale envelope: the sink's freshness check must drop it.
    stale = sample()
    stale["stamp_ms"] = int(time.time() * 1000) - 60_000
    stale["obs_stamp_ms"] = stale["stamp_ms"]
    assert plugin.accept(stale) is False
    assert len(client.writes) == before
    # The stream's own seq counter advanced only on admitted frames.
    assert stream["sink_seq"] >= 1
