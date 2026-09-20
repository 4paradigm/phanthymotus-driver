"""ACP: what the driver posts when an asynchronous action ends.

Two invariants, both of which fail *quietly* on the agent-core side and so can
only be caught here:

* **A preempted action reports `cancelled`, never `completed`.** By the time a
  cancel lands, `llm.py` has already force-marked the pendings and released the
  barrier, so posting nothing looks harmless. But `/api/acp/complete` has a
  second channel (`start.py:616-624`) that enqueues an `action_complete`
  steering event — which is how the LLM learns the leg ended and how far it got.
  A late `cancelled` is harmless. A late `completed` tells the model it reached
  a waypoint it abandoned, which is precisely the long-horizon failure the
  exhibition tour is built to detect.
* **Exactly one post per `action_id`.** `mark_action_complete` overwrites
  `_pending_results` without complaint, so a duplicate neither errors nor logs;
  it just makes the transcript wrong.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_acp.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.generic import acp  # noqa: E402
from simulator.generic.backend import LocalBackend  # noqa: E402
from simulator.generic.cards_audio import TtsCard  # noqa: E402
from simulator.generic.cards_motion import ControlledSpatialCard, SwitchModeCard  # noqa: E402
from simulator.generic.clock import FakeClock  # noqa: E402
from simulator.generic.geometry import OccupancyGrid  # noqa: E402
from simulator.generic.world import VirtualWorld  # noqa: E402

CONFIG = {"embodiment": {"kind": "wheeled", "dof": 2, "joint_names": ["a", "b"]}}

# What `start.py:606-608` actually reads off the body.
REQUIRED_BODY_KEYS = ("action_id", "status", "result")


@pytest.fixture
def rig():
    posts: list[dict] = []
    acp.set_transport(lambda url, payload: posts.append({"url": url, **payload}))
    backend = LocalBackend()
    backend.reset({"grid": OccupancyGrid.blank(0.05, (-5.0, -5.0), 600, 240), "dof": 2,
                   "motion": {"max_lin": 0.5, "max_ang": 0.6, "accel": 0.4, "radius": 0.2}})
    clock = FakeClock()
    world = VirtualWorld(backend, clock, {"tick_hz": 20.0, "chars_per_sec": 5.0})
    yield world, clock, posts
    acp.set_transport(None)


def run(world, clock, seconds, dt=0.05):
    for _ in range(int(round(seconds / dt))):
        clock.advance(dt)
        world.step(dt)


# ── payload shape ────────────────────────────────────────────────────────────

def test_payload_carries_every_field_agent_core_reads(rig):
    world, clock, posts = rig
    TtsCard(world, CONFIG, "sim").dispatch("speak", {"text": "到了"})
    run(world, clock, 3.0)

    assert len(posts) == 1
    for key in REQUIRED_BODY_KEYS:
        assert key in posts[0], f"/api/acp/complete reads {key} off the body"
    assert posts[0]["url"].endswith("/api/acp/complete")
    assert posts[0]["tool"] == "tts"


def test_result_is_a_dict_not_a_string(rig):
    """`start.py` forwards `result` straight into the steering payload; a string
    there reaches the LLM as escaped source text instead of fields."""
    world, clock, posts = rig
    ControlledSpatialCard(world, CONFIG, "sim").dispatch("navigate_to_pose", {"x": 1.0, "y": 0.0, "yaw": 0.0})
    run(world, clock, 20.0)

    assert isinstance(posts[0]["result"], dict)


# ── cancelled, never completed ───────────────────────────────────────────────

def test_preempted_navigation_posts_cancelled_with_partial_progress(rig):
    world, clock, posts = rig
    nav = ControlledSpatialCard(world, CONFIG, "sim")
    nav.dispatch("navigate_to_pose", {"x": 12.0, "y": 0.0, "yaw": 0.0})
    run(world, clock, 8.0)

    nav.dispatch("stop_nav", {})

    assert len(posts) == 1
    assert posts[0]["status"] == "cancelled"
    assert 0.0 < posts[0]["result"]["progress"]["fraction"] < 1.0


def test_interrupted_speech_posts_cancelled(rig):
    world, clock, posts = rig
    tts = TtsCard(world, CONFIG, "sim")
    tts.dispatch("speak", {"text": "一段很长很长的讲解词" * 5})
    run(world, clock, 1.0)

    tts.dispatch("interrupt", {})

    assert [post["status"] for post in posts] == ["cancelled"]


def test_a_completed_action_posts_completed(rig):
    world, clock, posts = rig
    ControlledSpatialCard(world, CONFIG, "sim").dispatch("navigate_to_pose", {"x": 1.0, "y": 0.0, "yaw": 0.0})
    run(world, clock, 20.0)

    assert [post["status"] for post in posts] == ["completed"]


def test_a_superseded_leg_is_reported_so_the_llm_can_resume_it(rig):
    """The abandoned waypoint has to be identifiable, or the tour resumes at the
    wrong one — the single most common long-horizon mistake."""
    world, clock, posts = rig
    nav = ControlledSpatialCard(world, CONFIG, "sim")
    world.set_tags([{"name": "二号展区", "x": 12.0, "y": 0.0},
                    {"name": "洗手间", "x": 0.0, "y": 3.0}])
    nav.dispatch("navigate_to_tag", {"name": "二号展区"})
    run(world, clock, 6.0)

    nav.dispatch("navigate_to_tag", {"name": "洗手间"})

    assert posts[0]["status"] == "cancelled"
    assert posts[0]["result"]["label"] == "二号展区"


# ── exactly once ─────────────────────────────────────────────────────────────

def test_one_post_per_action_id_when_cancel_races_arrival(rig):
    world, clock, posts = rig
    nav = ControlledSpatialCard(world, CONFIG, "sim")
    action_id = nav.dispatch("navigate_to_pose", {"x": 0.5, "y": 0.0, "yaw": 0.0})["action_id"]
    run(world, clock, 2.0)
    nav.dispatch("stop_nav", {})
    run(world, clock, 3.0)

    assert [post["action_id"] for post in posts].count(action_id) == 1


def test_interrupting_an_already_finished_utterance_posts_nothing_extra(rig):
    world, clock, posts = rig
    tts = TtsCard(world, CONFIG, "sim")
    tts.dispatch("speak", {"text": "短"})
    run(world, clock, 3.0)

    tts.dispatch("interrupt", {})

    assert len(posts) == 1 and posts[0]["status"] == "completed"

def test_switch_mode_is_not_reported_as_a_navigation(rig):
    world, clock, posts = rig
    ControlledSpatialCard(world, CONFIG, "sim")
    switch = SwitchModeCard(world, CONFIG, "sim")

    switch.dispatch("stand", {})
    run(world, clock, 3.0)

    assert [post["tool"] for post in posts] == ["switch_mode"]


# ── robustness ───────────────────────────────────────────────────────────────

def test_a_failing_callback_does_not_kill_the_tick(rig):
    """The callback POSTs over HTTP from the tick thread. A dead Agent Core must
    degrade to a log line, not stop the world."""
    world, clock, _ = rig

    def explode(url, payload):
        raise ConnectionError("agent core is down")

    acp.set_transport(explode)
    ControlledSpatialCard(world, CONFIG, "sim").dispatch("navigate_to_pose", {"x": 1.0, "y": 0.0, "yaw": 0.0})
    run(world, clock, 20.0)

    assert world.snapshot()["job"]["status"] == "completed"

def test_empty_text_is_rejected_rather_than_posting_a_zero_length_action(rig):
    world, _, posts = rig

    assert "error" in TtsCard(world, CONFIG, "sim").dispatch("speak", {"text": "   "})
    assert posts == []
