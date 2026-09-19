"""Every simulator card conforms to README_dev.md's tool spec.

The bundle is meant to be the first executable reference for that spec — today
the format table and the schema rules exist only as prose, and nothing checks
that any driver follows them. So these assertions are deliberately written
against the *spec*, not against the simulator's own choices.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_tool_spec.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.generic.backend import LocalBackend  # noqa: E402
from simulator.generic.card_base import BUS_RENDERABLE_FORMATS  # noqa: E402
from simulator.generic.cards_motion import (  # noqa: E402
    ArmCard,
    LedCard,
    ControlledSpatialCard,
    LocoCard,
    SwitchModeCard,
)
from simulator.generic.cards_sensors import (  # noqa: E402
    BatteryCard,
    ImuCard,
    LaserScanCard,
    MapCard,
    ModelCard,
    OdomCard,
)
from simulator.generic.clock import FakeClock  # noqa: E402
from simulator.generic.geometry import OccupancyGrid, Pose  # noqa: E402
from simulator.generic.world import VirtualWorld  # noqa: E402

ALL_CARDS = (OdomCard, ImuCard, LaserScanCard, BatteryCard, MapCard, ModelCard,
             LocoCard, ControlledSpatialCard, SwitchModeCard, LedCard, ArmCard)

VALID_KINDS = {"sensor", "actuator", "processor", "resource"}
CONFIG = {"embodiment": {"kind": "wheeled", "dof": 2, "joint_names": ["a", "b"]}}


def build():
    clock = FakeClock()
    backend = LocalBackend()
    backend.reset({"grid": OccupancyGrid.blank(0.05, (-5.0, -5.0), 240, 240), "dof": 2,
                   "motion": {"max_lin": 0.5, "max_ang": 0.6, "accel": 0.4, "radius": 0.2}})
    return VirtualWorld(backend, clock, {"tick_hz": 20.0}), clock


def card(cls):
    world, clock = build()
    return cls(world, CONFIG, "sim"), world, clock


def tools():
    world, _ = build()
    return [cls(world, CONFIG, "sim").get_tool() for cls in ALL_CARDS]


# ── schema shape ─────────────────────────────────────────────────────────────

@pytest.mark.parametrize("definition", tools(), ids=[cls.NAME for cls in ALL_CARDS])
def test_tool_has_the_required_keys(definition):
    for key in ("name", "type", "description", "inputSchema"):
        assert key in definition, f"{definition.get('name')} is missing {key}"
    assert definition["type"] in VALID_KINDS
    assert definition["description"].strip()


@pytest.mark.parametrize("definition", tools(), ids=[cls.NAME for cls in ALL_CARDS])
def test_action_params_cover_every_action_in_the_enum(definition):
    """An action the model can pick but that carries no parameter description is
    an action it will call wrong."""
    schema = definition["inputSchema"]
    enum = schema.get("properties", {}).get("action", {}).get("enum")
    if not enum:
        return
    documented = set(schema.get("x-action-params", {}))
    assert set(enum) == documented, (
        f"{definition['name']}: enum {sorted(set(enum) - documented)} undocumented, "
        f"{sorted(documented - set(enum))} documented but unreachable")


@pytest.mark.parametrize("definition", tools(), ids=[cls.NAME for cls in ALL_CARDS])
def test_completion_actions_are_a_subset_of_the_enum(definition):
    """An `x-completion` naming an action that does not exist is a barrier that
    never releases."""
    schema = definition["inputSchema"]
    completion = schema.get("x-completion")
    if not completion:
        return
    enum = set(schema.get("properties", {}).get("action", {}).get("enum", []))
    assert set(completion["actions"]) <= enum
    assert completion["timeout"] > 0


@pytest.mark.parametrize("definition", tools(), ids=[cls.NAME for cls in ALL_CARDS])
def test_declared_topic_formats_render_off_the_bus(definition):
    for entry in definition.get("topic_out", []):
        assert entry["topic"].startswith("/sim/")
        assert entry["format"] in BUS_RENDERABLE_FORMATS


@pytest.mark.parametrize("definition", tools(), ids=[cls.NAME for cls in ALL_CARDS])
def test_hooks_use_the_dict_form_agent_core_parses(definition):
    """`hooks.register` reads `spec.get('action')` — a bare string silently
    registers an empty action and the hook fires into nothing."""
    for hook_id, spec in definition["inputSchema"].get("x-hooks", {}).items():
        assert isinstance(spec, dict), f"{definition['name']}.{hook_id} must be a dict, not {type(spec)}"
        assert spec.get("action"), f"{definition['name']}.{hook_id} has no action"
        enum = set(definition["inputSchema"].get("properties", {}).get("action", {}).get("enum", []))
        assert spec["action"] in enum, f"{definition['name']}.{hook_id} binds a non-existent action"


@pytest.mark.parametrize("definition", tools(), ids=[cls.NAME for cls in ALL_CARDS])
def test_extension_keys_live_inside_input_schema(definition):
    """agent-core reads x-completion, x-resource and x-hooks from `inputSchema`
    and nowhere else (mcp_client.py:407-408, api/mcp_manage.py:685,725). Declared
    one level up they are silently invisible: no hook registered, no resource
    conflict detected, nothing logged."""
    for key in ("x-completion", "x-resource", "x-hooks"):
        assert key not in definition, f"{definition['name']}: {key} is beside inputSchema, not inside it"


def test_tool_names_are_unique():
    names = [definition["name"] for definition in tools()]
    assert len(names) == len(set(names))


# ── dispatch contract ────────────────────────────────────────────────────────

@pytest.mark.parametrize("cls", ALL_CARDS, ids=[cls.NAME for cls in ALL_CARDS])
def test_dispatch_returns_a_flat_dict_never_a_wrapped_array(cls):
    """The HTTP layer wraps the reply into JSON-RPC content. Returning a
    pre-wrapped [{"type": "text", ...}] double-encodes it and breaks the
    frontend — the single most common driver bug in README_dev."""
    instance, _, _ = card(cls)
    action = instance.NAME if instance.KIND == "resource" else "read"

    result = instance.dispatch(action, {})

    assert isinstance(result, dict)
    assert not isinstance(result, list)
    assert "type" not in result


@pytest.mark.parametrize("cls", ALL_CARDS, ids=[cls.NAME for cls in ALL_CARDS])
def test_lifecycle_verbs_are_answered_by_every_non_resource_card(cls):
    """start-project is strict: one card that fails `start` rolls the entire
    project back and the robot comes up with nothing running."""
    instance, _, _ = card(cls)
    if instance.KIND == "resource":
        return

    assert instance.dispatch("start", {}).get("state") == "running"
    assert instance.dispatch("info", {}).get("state") == "running"
    assert instance.dispatch("stop", {}).get("state") == "idle"


def test_dispatch_strips_the_injected_tool_name():
    """`DriverBundle.dispatch` injects `_tool_name`; a handler that forwards
    **args into a typed signature would choke on it."""
    instance, _, _ = card(LocoCard)

    result = instance.dispatch("move", {"lin": 0.3, "ang": 0.0, "_tool_name": "loco"})

    assert result["lin"] == 0.3


# ── nav / ACP handshake ──────────────────────────────────────────────────────

def test_navigate_returns_an_action_id_immediately():
    """Asynchronous by contract: the LLM keeps reasoning, and the barrier holds
    the next actuator call until the callback lands."""
    instance, world, _ = card(ControlledSpatialCard)

    result = instance.dispatch("navigate_to_pose", {"x": 5.0, "y": 0.0, "yaw": 0.0})

    assert result["state"] == "running"
    assert result["action_id"].startswith("sim-nav-")
    assert result["estimated_distance_m"] == pytest.approx(5.0, abs=0.01)


def test_navigate_to_tag_rejects_an_unknown_tag_with_the_known_list():
    instance, world, _ = card(ControlledSpatialCard)
    world.set_tags([{"name": "入口", "x": 1.0, "y": 0.0}])

    result = instance.dispatch("navigate_to_tag", {"name": "月球"})

    assert "error" in result
    assert result["known_tags"] == ["入口"]


def test_stop_nav_reports_progress_rather_than_a_bare_ok():
    instance, world, clock = card(ControlledSpatialCard)
    instance.dispatch("navigate_to_pose", {"x": 12.0, "y": 0.0, "yaw": 0.0})
    for _ in range(160):
        clock.advance(0.05)
        world.step(0.05)

    result = instance.dispatch("stop_nav", {})

    assert result["stopped"] is True
    assert result["status"] == "cancelled"
    assert 0.0 < result["progress"]["fraction"] < 1.0


def test_stop_nav_with_nothing_running_is_not_an_error():
    instance, _, _ = card(ControlledSpatialCard)

    result = instance.dispatch("stop_nav", {})

    assert result["stopped"] is False
    assert "error" not in result


def test_tag_place_names_the_current_pose():
    """真机上导览前也是先把展区打好点，之后每段走 navigate_to_tag。"""
    instance, world, clock = card(ControlledSpatialCard)
    instance.dispatch("navigate_to_pose", {"x": 2.0, "y": 0.0, "yaw": 0.0})
    for _ in range(200):
        clock.advance(0.05)
        world.step(0.05)

    tag = instance.dispatch("tag_place", {"name": "一号展区", "description": "早期产品"})["tag"]

    assert tag["name"] == "一号展区"
    assert tag["x"] == pytest.approx(2.0, abs=0.15)
    assert [t["name"] for t in world.tags()] == ["一号展区"]


def test_protected_actions_are_refused_without_the_password():
    """真卡会拒，仿真也必须拒 —— 一个不检查密码的仿真，会让人以为真机上也不用给。"""
    instance, _, _ = card(ControlledSpatialCard)

    denied = instance.dispatch("add_wall", {"x1": 0, "y1": 1, "x2": 2, "y2": 1})
    allowed = instance.dispatch("add_wall", {"x1": 0, "y1": 1, "x2": 2, "y2": 1,
                                             "password": "123456"})

    assert "error" in denied and "password" in denied["error"]
    assert allowed["state"] == "running"


def test_a_virtual_wall_changes_the_grid_and_the_laser_sees_it():
    """障碍物因此有了诚实的来源：加一道墙 → 栅格改变 → 激光雷达立刻反映。"""
    from simulator.generic.cards_sensors import LaserScanCard

    instance, world, _ = card(ControlledSpatialCard)
    scan = LaserScanCard(world, CONFIG, "sim")
    before = min(scan.payload()["ranges"])

    instance.dispatch("add_wall", {"x1": 1.0, "y1": -2.0, "x2": 1.0, "y2": 2.0,
                                   "password": "123456"})
    after = min(scan.payload()["ranges"])

    assert after < before
    assert after == pytest.approx(1.0, abs=0.2)


def test_removing_a_wall_restores_the_grid():
    instance, world, _ = card(ControlledSpatialCard)
    grid = world._backend.state()["grid"]  # noqa: SLF001
    wall = instance.dispatch("add_wall", {"x1": 1.0, "y1": -2.0, "x2": 1.0, "y2": 2.0,
                                          "password": "123456"})["wall"]
    occupied = grid.is_occupied(1.0, 0.0)

    instance.dispatch("remove_wall", {"wall_id": wall["id"], "password": "123456"})

    assert occupied is True
    assert grid.is_occupied(1.0, 0.0) is False
    assert instance.dispatch("list_walls", {})["walls"] == []


def test_mutating_the_grid_bumps_its_revision():
    """map 卡的占用扫描按 revision 缓存 —— 虚拟墙是原地改栅格，identity 不变，
    只按 id 缓存会一直画出加墙之前的地图。"""
    instance, world, _ = card(ControlledSpatialCard)
    grid = world._backend.state()["grid"]  # noqa: SLF001
    before = grid.revision

    instance.dispatch("add_wall", {"x1": 1.0, "y1": -1.0, "x2": 1.0, "y2": 1.0,
                                   "password": "123456"})

    assert grid.revision > before


def test_arm_rejects_a_joint_vector_of_the_wrong_length():
    instance, _, _ = card(ArmCard)

    assert "error" in instance.dispatch("move_joints", {"positions": [0.1]})
    assert instance.dispatch("move_joints", {"positions": [0.1, 0.2]})["state"] == "running"


def test_switch_mode_declares_no_interrupt_hook():
    """Aborting a posture change partway is how a controlled descent becomes a
    fall — the same reason llm.py's fallback deliberately skips switch_mode."""
    instance, _, _ = card(SwitchModeCard)

    assert instance.get_tool()["inputSchema"].get("x-hooks", {}) == {}


# ── resource declarations ────────────────────────────────────────────────────

def test_cards_sharing_a_physical_resource_declare_it():
    world, _ = build()
    by_name = {cls.NAME: cls(world, CONFIG, "sim").get_tool() for cls in ALL_CARDS}

    assert by_name["loco"]["inputSchema"]["x-resource"] == ["base"]
    assert by_name["controlled_spatial"]["inputSchema"]["x-resource"] == ["base"]
    assert by_name["arm"]["inputSchema"]["x-resource"] == ["arm"]


def test_read_only_cards_are_typed_so_the_barrier_exempts_them():
    """`_needs_barrier` exempts sensor and resource. Typing a status read as an
    actuator queues it behind a 90 second navigation."""
    world, _ = build()
    for cls in (OdomCard, ImuCard, LaserScanCard, BatteryCard, MapCard):
        assert cls(world, CONFIG, "sim").get_tool()["type"] == "sensor"
    assert ModelCard(world, CONFIG, "sim").get_tool()["type"] == "resource"
