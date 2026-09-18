"""A user barge-in has to actually stop the robot. Today, on Tianyi, it does not.

This file pins the defect as an executable assertion, and pins the simulator's
card naming as the contrast that makes it visible.

## The mechanism, read out of agent-core on 2026-09-18

`_interrupt_active_outputs` (`agent-core/src/event/llm.py:1457`) is what a
TurnCancelled — a user talking over the robot — reaches:

    results = await hooks.fire('on_interrupt_all')
    if results:
        ...
        return                                    # <-- early return
    # fallback: hardcoded lookup
    for short_name, action in (('tts', 'interrupt'), ('loco', 'stop_move')):

Three facts turn that into a silent failure:

1. `hooks.register` (`src/hooks.py:35`) registers bindings **straight from the
   `x-hooks` field of a tool schema**. Declaring a hook *is* the binding; no
   canvas wiring and no user action is involved.
2. `hooks.fire` (`src/hooks.py:167`) appends an entry for every binding it ran,
   **including ones that raised** — errors become `{'error': ...}` results
   rather than being dropped. So `results` is non-empty whenever a binding
   merely *exists*, whether or not it did anything.
3. Therefore one card declaring `on_interrupt_all` suppresses the hardcoded
   fallback for **every** card in the fleet.

On Tianyi the `servo` card declares `on_interrupt_all` and is enabled by default.
Its navigation cards (`nav`, `controlled_spatial`) declare only
`on_interrupt_motion`, which a barge-in never fires. Neither is named `loco`, and
`tts` is only reachable through the fallback that never runs.

Net effect of a barge-in on Tianyi: the servo pauses, and **speech and navigation
both continue**. Nothing is logged that says so.

The simulator's locomotion card is therefore named `loco`, with a `stop_move`
action, and declares `on_interrupt_all` itself — so it stops whichever path the
code takes. Running the same barge-in against both is what makes the defect
visible as a difference rather than an argument.

These assertions read the real driver sources structurally with `ast` rather than
executing them: tianyi's `device.py` is 7k lines behind vendor ROS imports, and a
grep would not survive reformatting.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_interrupt_naming.py -q
"""

from __future__ import annotations

import ast
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.generic.backend import LocalBackend  # noqa: E402
from simulator.generic.cards_audio import TtsCard  # noqa: E402
from simulator.generic.cards_motion import LocoCard, NavCard  # noqa: E402
from simulator.generic.clock import FakeClock  # noqa: E402
from simulator.generic.geometry import OccupancyGrid, Pose  # noqa: E402
from simulator.generic.world import VirtualWorld  # noqa: E402

TIANYI = ROOT / "x-humanoid" / "tianyi2.0"

# What llm.py falls back to when no `on_interrupt_all` binding exists.
FALLBACK_LOOKUP = (("tts", "interrupt"), ("loco", "stop_move"))


# ── structural reader for real driver sources ────────────────────────────────

def declared_tools(path: Path) -> dict[str, dict]:
    """Every tool-definition dict literal in a source file, keyed by tool name.

    A tool definition is a dict literal carrying both a string `name` and a
    `type` — enough to pick them out without executing vendor imports.
    """
    found: dict[str, dict] = {}
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if not isinstance(node, ast.Dict):
            continue
        keys = {k.value: v for k, v in zip(node.keys, node.values)
                if isinstance(k, ast.Constant) and isinstance(k.value, str)}
        name = keys.get("name")
        if "type" not in keys or not isinstance(name, ast.Constant) or not isinstance(name.value, str):
            continue
        found[name.value] = keys
    return found


def hook_ids(tool_node: dict) -> set[str]:
    """`x-hooks` lives *inside* `inputSchema`, which is where agent-core reads it
    from (`api/mcp_manage.py:725`). Looked for one level up it is always empty —
    which is exactly the mistake that makes a declared hook never register."""
    schema = tool_node.get("inputSchema")
    hooks = None
    if isinstance(schema, ast.Dict):
        for key, value in zip(schema.keys, schema.values):
            if isinstance(key, ast.Constant) and key.value == "x-hooks":
                hooks = value
    if not isinstance(hooks, ast.Dict):
        return set()
    return {k.value for k in hooks.keys if isinstance(k, ast.Constant)}


def all_tianyi_tools() -> dict[str, dict]:
    merged: dict[str, dict] = {}
    for source in sorted(TIANYI.glob("*.py")):
        merged.update(declared_tools(source))
    return merged


# ── the defect ───────────────────────────────────────────────────────────────

def test_reader_actually_finds_tianyi_tools():
    """Guard the guard: a structural reader that silently matches nothing would
    make every assertion below vacuously true."""
    tools = all_tianyi_tools()

    assert len(tools) > 15, f"only found {len(tools)} tianyi tools — the AST reader is broken"
    assert {"nav", "tts", "controlled_spatial", "servo"} <= set(tools)


def test_tianyi_servo_declares_on_interrupt_all_which_suppresses_the_fallback():
    """The single fact that makes a barge-in a no-op on that robot."""
    servo = all_tianyi_tools()["servo"]

    assert "on_interrupt_all" in hook_ids(servo), (
        "if this ever becomes false, re-check the defect — the fallback would run again")


def test_tianyi_navigation_cards_are_deaf_to_a_barge_in():
    """They bind only `on_interrupt_motion`, which a TurnCancelled never fires,
    and neither is named `loco`, so the suppressed fallback would not have
    reached them either."""
    tools = all_tianyi_tools()
    fallback_names = {name for name, _ in FALLBACK_LOOKUP}

    for card_name in ("nav", "controlled_spatial"):
        hooks = hook_ids(tools[card_name])
        assert "on_interrupt_all" not in hooks, f"{card_name} now handles barge-in — update this test"
        assert "on_interrupt_motion" in hooks
        assert card_name not in fallback_names


def test_tianyi_speech_only_stops_through_the_suppressed_fallback():
    """`tts` is named in the fallback and declares `on_interrupt_speak` — but
    neither path runs on a barge-in while servo's `on_interrupt_all` exists."""
    tts = all_tianyi_tools()["tts"]
    hooks = hook_ids(tts)

    assert "on_interrupt_all" not in hooks
    assert "on_interrupt_speak" in hooks
    assert "tts" in {name for name, _ in FALLBACK_LOOKUP}


# ── the contrast: what the simulator does instead ────────────────────────────

def build():
    backend = LocalBackend()
    backend.reset({"grid": OccupancyGrid.blank(0.05, (-5.0, -5.0), 400, 240), "dof": 2,
                   "motion": {"max_lin": 0.5, "max_ang": 0.6, "accel": 0.4, "radius": 0.2}})
    clock = FakeClock()
    world = VirtualWorld(backend, clock, {"tick_hz": 20.0, "chars_per_sec": 5.0})
    config = {"embodiment": {"kind": "wheeled", "dof": 2, "joint_names": ["a", "b"]}}
    return world, clock, config


def test_simulator_locomotion_card_is_reachable_by_the_hardcoded_fallback():
    world, _, config = build()
    card = LocoCard(world, config, "sim")
    definition = card.get_tool()
    actions = set(definition["inputSchema"]["properties"]["action"]["enum"])

    assert (definition["name"], "stop_move") in FALLBACK_LOOKUP
    assert "stop_move" in actions


def test_simulator_speech_card_is_reachable_by_the_hardcoded_fallback():
    world, _, config = build()
    definition = TtsCard(world, config, "sim").get_tool()
    actions = set(definition["inputSchema"]["properties"]["action"]["enum"])

    assert (definition["name"], "interrupt") in FALLBACK_LOOKUP
    assert "interrupt" in actions


def test_simulator_cards_also_bind_on_interrupt_all_so_the_hook_path_works():
    """Belt and braces: whichever branch llm.py takes, this robot stops."""
    world, _, config = build()

    for card in (LocoCard(world, config, "sim"), TtsCard(world, config, "sim")):
        assert "on_interrupt_all" in card.get_tool()["inputSchema"]["x-hooks"]


def test_the_fallback_action_names_really_stop_the_simulated_robot():
    """Naming alone proves nothing — the actions the fallback sends have to work."""
    world, clock, config = build()
    loco, tts = LocoCard(world, config, "sim"), TtsCard(world, config, "sim")
    loco.dispatch("move", {"lin": 0.4, "ang": 0.0})
    tts.dispatch("speak", {"text": "一段很长的讲解词" * 6})
    for _ in range(20):
        clock.advance(0.05)
        world.step(0.05)
    assert world.snapshot()["lin"] > 0.1 and world.snapshot()["speech"] is not None

    for short_name, action in FALLBACK_LOOKUP:
        {"loco": loco, "tts": tts}[short_name].dispatch(action, {})
    # A stop is a command, not teleportation: the base decelerates at `accel`,
    # so give it the ~1 s that 0.4 m/s at 0.4 m/s^2 actually takes.
    for _ in range(40):
        clock.advance(0.05)
        world.step(0.05)

    snapshot = world.snapshot()
    assert abs(snapshot["lin"]) < 1e-6, "loco.stop_move did not stop the base"
    assert snapshot["speech"] is None, "tts.interrupt did not stop speech"


def test_a_barge_in_equivalent_also_stops_an_active_navigation():
    """The case Tianyi gets wrong: a leg in progress when the user speaks."""
    world, clock, config = build()
    loco, nav = LocoCard(world, config, "sim"), NavCard(world, config, "sim")
    job_id = nav.dispatch("move_to", {"x": 12.0, "y": 0.0, "yaw": 0.0})["action_id"]
    for _ in range(100):
        clock.advance(0.05)
        world.step(0.05)

    loco.dispatch("stop_move", {})

    snapshot = world.snapshot()
    assert snapshot["job"]["action_id"] == job_id
    assert snapshot["job"]["status"] == "cancelled"
    assert 0.0 < snapshot["job"]["progress"]["fraction"] < 1.0
