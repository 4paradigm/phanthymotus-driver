"""Wire-format guard for the simulator's sensor cards.

These formats are platform contracts, not bundle details, and every one of them
fails *silently* when it is wrong: the card appears on the canvas, the panel
stays blank, and nothing lands in any log. That is why they are pinned here
rather than discovered on a robot.

The decoder below is written independently of the encoder on purpose — a
round-trip through the card's own helper would pass even if both sides agreed on
the wrong layout.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_sensor_formats.py -q
"""

from __future__ import annotations

import json
import math
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.generic.backend import LocalBackend  # noqa: E402
from simulator.generic.card_base import BUS_RENDERABLE_FORMATS, Card  # noqa: E402
from simulator.generic.cards_sensors import (  # noqa: E402
    BatteryCard,
    ImuCard,
    LaserScanCard,
    MapCard,
    ModelCard,
    OdomCard,
)
from simulator.generic.clock import FakeClock  # noqa: E402
from simulator.generic.geometry import OCCUPIED, OccupancyGrid, Pose  # noqa: E402
from simulator.generic.world import VirtualWorld  # noqa: E402

ALL_SENSOR_CARDS = (OdomCard, ImuCard, LaserScanCard, BatteryCard, MapCard, ModelCard)

EMBODIMENT = {"kind": "arm", "dof": 3, "joint_names": ["shoulder_pan", "shoulder_lift", "elbow"]}


def build(grid=None, spawn=(0.0, 0.0, 0.0), config=None):
    grid = grid if grid is not None else OccupancyGrid.blank(0.05, (-5.0, -5.0), 240, 240)
    clock = FakeClock()
    backend = LocalBackend()
    backend.reset({"grid": grid, "spawn": {"x": spawn[0], "y": spawn[1], "yaw": spawn[2]},
                   "motion": {"max_lin": 0.5, "max_ang": 0.6, "accel": 0.4, "radius": 0.2},
                   "dof": 3})
    world = VirtualWorld(backend, clock, {"tick_hz": 20.0})
    return world, clock, {"embodiment": EMBODIMENT, **(config or {})}


def decode_mapping(buffer: bytes) -> dict:
    """Independent reader for `<fffBI` + float32[n][3] + `<I` + JSON."""
    robot_x, robot_y, display_yaw, flags, count = struct.unpack_from("<fffBI", buffer, 0)
    offset = struct.calcsize("<fffBI")
    floats = struct.unpack_from(f"<{count * 3}f", buffer, offset)
    offset += count * 3 * 4
    (meta_len,) = struct.unpack_from("<I", buffer, offset)
    offset += 4
    meta = json.loads(buffer[offset:offset + meta_len].decode())
    return {"robot": (robot_x, robot_y, display_yaw), "flags": flags, "count": count,
            "points": [tuple(floats[i:i + 3]) for i in range(0, len(floats), 3)],
            "meta": meta, "consumed": offset + meta_len}


# ── format allowlist ─────────────────────────────────────────────────────────

@pytest.mark.parametrize("card_cls", ALL_SENSOR_CARDS)
def test_every_declared_format_actually_renders_off_the_bus(card_cls):
    """A renderer with no `onData` cannot draw a topic. Declaring one of those on
    a `topic_out` gives a permanently blank panel and no log line anywhere."""
    world, _, config = build()
    card = card_cls(world, config, "sim")
    for entry in card.get_tool().get("topic_out", []):
        assert entry["format"] in BUS_RENDERABLE_FORMATS, (
            f"{card_cls.__name__} publishes {entry['format']}, which no renderer reads from the bus")


def test_laser_scan_avoids_sensor_lidar():
    """`sensor/lidar` has a renderer, but it only handles `mcp_result` activity
    events — it has no `onData`. Nothing in this repo publishes it, and this card
    must not become the first to try."""
    world, _, config = build()
    formats = {entry["format"] for entry in LaserScanCard(world, config, "sim").get_tool()["topic_out"]}
    assert "sensor/lidar" not in formats
    assert formats == {"data/json"}


def test_no_card_claims_sensor_imu():
    """There is no `sensor/imu` renderer at all."""
    world, _, config = build()
    for card_cls in ALL_SENSOR_CARDS:
        card = card_cls(world, config, "sim")
        assert all(entry["format"] != "sensor/imu" for entry in card.get_tool().get("topic_out", []))


# ── sensor/mapping wire format ───────────────────────────────────────────────

def test_mapping_header_round_trips():
    grid = OccupancyGrid.blank(0.05, (-2.0, -2.0), 120, 120)
    grid.border(OCCUPIED)
    world, _, config = build(grid, spawn=(0.5, -0.25, 1.0))
    card = MapCard(world, config, "sim")

    decoded = decode_mapping(card.payload())

    assert decoded["robot"][0] == pytest.approx(0.5, abs=1e-5)
    assert decoded["robot"][1] == pytest.approx(-0.25, abs=1e-5)
    assert decoded["count"] == len(decoded["points"]) > 0
    assert decoded["consumed"] == len(card.payload()), "trailing bytes or a short read"


def test_mapping_flags_have_the_has_z_bit_set():
    """`mapping.js` sniffs the protocol by testing `flags & 0x02`. Clear it and
    the renderer silently takes the legacy branch: blank panel, empty log."""
    world, _, config = build()
    decoded = decode_mapping(MapCard(world, config, "sim").payload())

    assert decoded["flags"] & 0x02, f"flags={decoded['flags']} — has_z is clear"
    assert decoded["flags"] == 7


def test_mapping_yaw_is_negated_for_the_renderer():
    """The renderer's world is y-flipped; an un-negated yaw points the robot cone
    the wrong way and nothing complains."""
    world, _, config = build(spawn=(0.0, 0.0, 1.2))
    decoded = decode_mapping(MapCard(world, config, "sim").payload())

    assert decoded["robot"][2] == pytest.approx(-1.2, abs=1e-5)


def test_mapping_stays_within_the_renderer_point_budget():
    grid = OccupancyGrid.blank(0.02, (-10.0, -10.0), 1000, 1000)
    grid.fill_rect(-9.0, -9.0, 9.0, 9.0, OCCUPIED)          # ~810k occupied cells
    world, _, config = build(grid)
    card = MapCard(world, config, "sim")

    decoded = decode_mapping(card.payload())

    assert 0 < decoded["count"] <= MapCard.MAX_POINTS


def test_mapping_carries_grid_and_waypoint_metadata():
    world, _, config = build()
    card = MapCard(world, config, "sim")
    card.set_waypoints_provider(lambda: [{"name": "一号展区", "x": 2.0, "y": 1.0}])

    decoded = decode_mapping(card.payload())

    assert decoded["meta"]["simulated"] is True
    assert decoded["meta"]["tags"] == [{"name": "一号展区", "x": 2.0, "y": 1.0}]
    assert decoded["meta"]["resolution"] == 0.05


def test_mapping_draws_trail_and_waypoints_at_distinct_heights():
    """The renderer colours by z, so the map, the path and the waypoints have to
    sit at different heights or they read as one undifferentiated blob."""
    grid = OccupancyGrid.blank(0.05, (-5.0, -5.0), 240, 240)
    grid.border(OCCUPIED)
    world, clock, config = build(grid)
    world.submit_job("navigate_to", Pose(2.0, 0.0, 0.0))
    for _ in range(200):
        clock.advance(0.05)
        world.step(0.05)

    card = MapCard(world, config, "sim")
    card.set_waypoints_provider(lambda: [{"name": "P", "x": 2.0, "y": 0.0}])
    heights = {round(z, 3) for _, _, z in decode_mapping(card.payload())["points"]}

    assert {MapCard.Z_GRID, MapCard.Z_TRAIL, MapCard.Z_WAYPOINT} <= heights


def test_mapping_survives_an_empty_map():
    world, _, config = build(OccupancyGrid.blank(0.05, (0.0, 0.0), 20, 20))
    decoded = decode_mapping(MapCard(world, config, "sim").payload())

    assert decoded["count"] == 0
    assert decoded["meta"]["size"] == [20, 20]


# ── laser scan agrees with the map ───────────────────────────────────────────

def test_scan_ranges_match_the_grid_it_was_cast_against():
    """A sensor that disagrees with the world tests nothing."""
    grid = OccupancyGrid.blank(0.05, (-5.0, -5.0), 240, 240)
    grid.fill_rect(2.0, -3.0, 2.2, 3.0, OCCUPIED)           # wall 2 m dead ahead
    world, _, config = build(grid)
    card = LaserScanCard(world, config, "sim")

    payload = card.payload()
    forward = payload["ranges"][len(payload["ranges"]) // 2]

    assert forward == pytest.approx(2.0, abs=0.1)
    assert payload["count"] == len(payload["ranges"])


def test_scan_read_summarises_instead_of_dumping_every_beam():
    """`read` goes into an LLM prompt; 60 floats crowd out the actual reasoning."""
    grid = OccupancyGrid.blank(0.05, (-5.0, -5.0), 240, 240)
    grid.fill_rect(1.0, -3.0, 1.2, 3.0, OCCUPIED)
    world, _, config = build(grid)

    result = LaserScanCard(world, config, "sim").do_read()

    assert "ranges" not in result
    assert result["min_range"] == pytest.approx(1.0, abs=0.1)


# ── imu / odom / battery ─────────────────────────────────────────────────────

def test_imu_is_derived_not_randomised():
    world, _, config = build(spawn=(0.0, 0.0, 0.7))
    card = ImuCard(world, config, "sim")

    first, second = card.payload(), card.payload()

    assert first["angular_velocity"] == second["angular_velocity"]
    assert first["orientation"]["yaw"] == pytest.approx(0.7, abs=1e-6)


def test_imu_angular_velocity_tracks_an_actual_rotation():
    world, clock, config = build()
    card = ImuCard(world, config, "sim")
    world.submit_job("rotate_to", Pose(0.0, 0.0, math.pi / 2))
    for _ in range(10):
        clock.advance(0.05)
        world.step(0.05)

    assert card.payload()["angular_velocity"]["z"] > 0.1


def test_odom_reports_pose_and_accumulated_distance():
    world, clock, config = build()
    card = OdomCard(world, config, "sim")
    world.submit_job("navigate_to", Pose(2.0, 0.0, 0.0))
    for _ in range(200):
        clock.advance(0.05)
        world.step(0.05)

    payload = card.payload()

    assert payload["pose"]["x"] == pytest.approx(2.0, abs=0.15)
    assert payload["odometer_m"] == pytest.approx(2.0, abs=0.2)


def test_battery_drains_with_distance():
    world, clock, config = build()
    card = BatteryCard(world, config, "sim")
    card.start()
    before = card.payload()["percent"]

    world.submit_job("navigate_to", Pose(8.0, 0.0, 0.0))
    for _ in range(600):
        clock.advance(0.05)
        world.step(0.05)

    after = card.payload()["percent"]
    assert after < before
    assert 0.0 <= after <= 100.0


def test_battery_low_flag_is_scriptable():
    world, _, config = build(config={"battery": {"start_percent": 15.0, "low_threshold": 20.0}})
    card = BatteryCard(world, config, "sim")

    assert card.payload()["low"] is True


# ── model card / URDF ────────────────────────────────────────────────────────

def test_urdf_joint_names_match_the_embodiment_exactly():
    """The skeleton renderer matches joints by name. One mismatch draws nothing
    and says nothing about why."""
    world, _, config = build()
    result = ModelCard(world, config, "sim").dispatch("model", {})

    assert result["joint_names"] == EMBODIMENT["joint_names"]
    for name in EMBODIMENT["joint_names"]:
        assert f'<joint name="{name}"' in result["urdf"]


def test_model_card_is_a_resource_so_it_stays_barrier_exempt():
    """`_needs_barrier` exempts sensor and resource. As an actuator, every
    skeleton fetch would queue behind a 90 second navigation."""
    world, _, config = build()
    assert ModelCard(world, config, "sim").get_tool()["type"] == "resource"


# ── lifecycle / info ─────────────────────────────────────────────────────────

@pytest.mark.parametrize("card_cls", ALL_SENSOR_CARDS)
def test_info_reports_the_authoritative_topic_out(card_cls):
    """agent-core binds what `info` says, not what `tools/list` happened to say."""
    world, _, config = build()
    card = card_cls(world, config, "sim")
    if card.KIND == "resource":
        pytest.skip("resource cards publish no topic")

    assert card.dispatch("info", {})["topic_out"] == card.get_tool()["topic_out"]


@pytest.mark.parametrize("card_cls", ALL_SENSOR_CARDS)
def test_dispatch_returns_a_flat_dict(card_cls):
    """Never a pre-wrapped [{"type": "text", ...}] array — the HTTP layer wraps
    it, and double-wrapping breaks frontend parsing."""
    world, _, config = build()
    card = card_cls(world, config, "sim")
    action = card.NAME if card.KIND == "resource" else "info"

    result = card.dispatch(action, {})

    assert isinstance(result, dict) and "type" not in result


def test_unknown_action_is_declined_in_the_form_lifecycle_expects():
    """`common.lifecycle.is_declined` only accepts an `unknown action` error with
    no `state` — anything else is read as a real failure and rolls the project back."""
    from common import lifecycle

    world, _, config = build()
    result = OdomCard(world, config, "sim").dispatch("wiggle", {})

    assert lifecycle.is_declined(result) is True


def test_cards_start_and_stop_without_ros():
    world, _, config = build()
    card = OdomCard(world, config, "sim", ros2=None)

    assert card.dispatch("start", {})["state"] == "running"
    assert card.dispatch("stop", {})["state"] == "idle"


def test_topic_names_are_namespaced():
    world, _, config = build()
    assert MapCard(world, config, "robot7").topic == "/robot7/map"


def test_base_card_publishes_nothing_when_it_has_no_topic():
    world, _, config = build()

    class Bare(Card):
        NAME = "bare"
        TOPIC = ""

    card = Bare(world, config, "sim")
    card.start()

    assert card.get_tool().get("topic_out") is None
    assert card.info()["topic_out"] == []
