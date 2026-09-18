"""The simulated world, exercised with no ROS, no HTTP and no robot.

This file pins the riskiest logic in the bundle while nothing else is in the
frame: the lock discipline, cancellation, and the guarantee that a navigation
reports its terminal state exactly once.

That last one matters more than it looks. A duplicate terminal callback does not
fail on the agent-core side — `mark_action_complete` overwrites `_pending_results`
without complaint — so a double post is invisible at runtime and merely makes the
transcript wrong. The only place it can be caught is here.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_sim_world.py -q
"""

from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from simulator.generic.backend import LocalBackend  # noqa: E402
from simulator.generic.clock import FakeClock  # noqa: E402
from simulator.generic.geometry import OCCUPIED, OccupancyGrid, Pose, normalize_angle  # noqa: E402
from simulator.generic.world import (  # noqa: E402
    RESULT_CANCELLED,
    RESULT_FAILED,
    RESULT_OK,
    STATE_RUNNING,
    VirtualWorld,
)

DT = 0.05


# ── fixtures ─────────────────────────────────────────────────────────────────

def make_world(grid=None, spawn=(0.0, 0.0, 0.0), **cfg):
    grid = grid if grid is not None else OccupancyGrid.blank(0.05, (-5.0, -5.0), 600, 400)
    clock = FakeClock()
    backend = LocalBackend()
    backend.reset({
        "grid": grid,
        "spawn": {"x": spawn[0], "y": spawn[1], "yaw": spawn[2]},
        "motion": {"max_lin": 0.5, "max_ang": 0.6, "accel": 0.4, "radius": 0.2},
        "dof": 2,
    })
    world = VirtualWorld(backend, clock, {"tick_hz": 1.0 / DT, **cfg})
    return world, clock, backend


def run_for(world, clock, seconds, dt=DT):
    """Advance then integrate, so a terminal stamped `t` really is `t` elapsed."""
    for _ in range(int(round(seconds / dt))):
        clock.advance(dt)
        world.step(dt)


def run_until(world, clock, predicate, limit=120.0, dt=DT):
    elapsed = 0.0
    while elapsed < limit:
        clock.advance(dt)
        world.step(dt)
        elapsed += dt
        if predicate():
            return elapsed
    return None


# ── integration ──────────────────────────────────────────────────────────────

def test_reaches_target_and_settles_on_final_heading():
    world, clock, _ = make_world()
    done = []
    world.set_nav_callback(done.append)

    job = world.submit_job("navigate_to", Pose(3.0, 2.0, math.pi / 2), label="一号展区")
    elapsed = run_until(world, clock, lambda: job.terminal_posted)

    assert elapsed is not None, "navigation never terminated"
    assert job.result == RESULT_OK
    snap = world.snapshot()
    assert math.hypot(snap["pose"]["x"] - 3.0, snap["pose"]["y"] - 2.0) <= 0.15
    assert abs(normalize_angle(snap["pose"]["yaw"] - math.pi / 2)) <= 0.08
    assert done[0]["status"] == "completed"
    assert done[0]["progress"]["fraction"] == 1.0


def test_rotate_in_place_does_not_translate():
    world, clock, _ = make_world()
    job = world.submit_job("rotate_to", Pose(0.0, 0.0, math.pi / 2))
    run_until(world, clock, lambda: job.terminal_posted)

    snap = world.snapshot()
    assert job.result == RESULT_OK
    assert abs(snap["pose"]["x"]) < 1e-6 and abs(snap["pose"]["y"]) < 1e-6
    assert snap["odometer"] == 0.0


def test_a_24_second_leg_runs_in_microseconds_of_wall_clock():
    """The injected clock is what makes a 366-test suite affordable — and what
    keeps `step()` faster than real time once a physics backend is attached."""
    world, clock, _ = make_world()
    job = world.submit_job("navigate_to", Pose(12.0, 0.0, 0.0))

    began = time.monotonic()
    simulated = run_until(world, clock, lambda: job.terminal_posted, limit=200.0)
    wall = time.monotonic() - began

    assert simulated is not None and simulated > 20.0, "12 m at 0.5 m/s must take real simulated time"
    assert wall < 1.0, f"wall clock {wall:.3f}s — the fake clock is not being used"


# ── collision ────────────────────────────────────────────────────────────────

def test_driving_into_an_occupied_cell_fails_the_job():
    grid = OccupancyGrid.blank(0.05, (-5.0, -5.0), 600, 400)
    grid.fill_rect(2.0, -1.0, 2.2, 1.0, OCCUPIED)          # wall across the path
    world, clock, _ = make_world(grid)
    done = []
    world.set_nav_callback(done.append)

    job = world.submit_job("navigate_to", Pose(4.0, 0.0, 0.0))
    run_until(world, clock, lambda: job.terminal_posted)

    assert job.result == RESULT_FAILED
    assert "blocked" in job.reason
    assert done[0]["status"] == "failed"
    assert world.snapshot()["pose"]["x"] < 2.0, "stopped short of the wall, not through it"


def test_segment_check_catches_a_one_cell_wall():
    """A full-cell sampling stride straddles a thin wall and misses it — which
    reads as walking through geometry with nothing in any log."""
    grid = OccupancyGrid.blank(0.05, (-1.0, -1.0), 200, 200)
    cx, cy = grid.world_to_cell(1.0, 0.0)
    for dy in range(-10, 11):
        grid.set_cell(cx, cy + dy, OCCUPIED)

    assert grid.segment_blocked(0.0, 0.0, 2.0, 0.0) is True
    assert grid.segment_blocked(0.0, 0.5, 0.5, 0.5) is False


def test_outside_the_grid_is_a_wall_not_empty_space():
    grid = OccupancyGrid.blank(0.05, (0.0, 0.0), 40, 40)   # 2m x 2m
    assert grid.is_occupied(5.0, 5.0) is True
    assert grid.at(-1, -1) == OCCUPIED


# ── cancellation ─────────────────────────────────────────────────────────────

def test_cancel_midway_reports_cancelled_with_partial_progress():
    world, clock, _ = make_world()
    done = []
    world.set_nav_callback(done.append)

    job = world.submit_job("navigate_to", Pose(12.0, 0.0, 0.0))
    run_for(world, clock, 8.0)
    assert job.state == STATE_RUNNING, "8 s into a 24 s leg it must still be running"

    payload = world.cancel_job("interrupted by user instruction")

    assert payload["status"] == "cancelled", "a preempted leg must not report completed"
    assert 0.0 < payload["progress"]["fraction"] < 1.0
    assert payload["reason"] == "interrupted by user instruction"
    assert payload["elapsed"] > 0
    assert len(done) == 1


def test_cancelled_job_stops_the_base():
    world, clock, _ = make_world()
    world.submit_job("navigate_to", Pose(12.0, 0.0, 0.0))
    run_for(world, clock, 5.0)
    world.cancel_job("stop")
    run_for(world, clock, 2.0)

    snap = world.snapshot()
    assert abs(snap["lin"]) < 1e-6 and abs(snap["ang"]) < 1e-6


def test_a_new_job_supersedes_the_old_one_as_cancelled():
    world, clock, _ = make_world()
    done = []
    world.set_nav_callback(done.append)

    first = world.submit_job("navigate_to", Pose(12.0, 0.0, 0.0), label="二号展区")
    run_for(world, clock, 6.0)
    second = world.submit_job("navigate_to", Pose(0.0, 4.0, 0.0), label="洗手间")

    assert first.terminal_posted and first.result == RESULT_CANCELLED
    assert second.state == STATE_RUNNING
    assert [event["status"] for event in done] == ["cancelled"]
    assert done[0]["label"] == "二号展区", "the abandoned leg must be identifiable to resume it"


def test_direct_velocity_preempts_navigation():
    world, clock, _ = make_world()
    job = world.submit_job("navigate_to", Pose(12.0, 0.0, 0.0))
    run_for(world, clock, 3.0)

    world.set_velocity(0.2, 0.0)

    assert job.terminal_posted and job.result == RESULT_CANCELLED
    assert "direct velocity" in job.reason


# ── exactly-once terminal ────────────────────────────────────────────────────

def test_terminal_is_posted_exactly_once_when_arrival_races_cancel():
    world, clock, _ = make_world()
    done = []
    world.set_nav_callback(done.append)

    job = world.submit_job("navigate_to", Pose(0.4, 0.0, 0.0))
    run_until(world, clock, lambda: job.fraction > 0.5, limit=10.0)

    # Both paths now reachable in the same instant.
    world.cancel_job("late cancel")
    run_for(world, clock, 3.0)

    assert len(done) == 1, f"terminal posted {len(done)} times"
    assert done[0]["status"] == "cancelled"


def test_concurrent_cancels_produce_one_terminal():
    """The real shape of the race: HTTP threads cancelling while the tick runs."""
    world, clock, _ = make_world()
    done = []
    world.set_nav_callback(done.append)
    job = world.submit_job("navigate_to", Pose(12.0, 0.0, 0.0))
    run_for(world, clock, 4.0)

    barrier = threading.Barrier(9)

    def racer():
        barrier.wait()
        world.cancel_job("racing cancel")

    threads = [threading.Thread(target=racer) for _ in range(8)]
    for thread in threads:
        thread.start()
    barrier.wait()
    for _ in range(20):
        world.step(DT)
    for thread in threads:
        thread.join(timeout=5.0)

    assert len(done) == 1, f"terminal posted {len(done)} times under 8 concurrent cancels"
    assert not any(thread.is_alive() for thread in threads)


def test_cancelling_a_finished_job_is_a_no_op():
    world, clock, _ = make_world()
    done = []
    world.set_nav_callback(done.append)
    job = world.submit_job("navigate_to", Pose(1.0, 0.0, 0.0))
    run_until(world, clock, lambda: job.terminal_posted)

    assert world.cancel_job("too late") is None
    assert len(done) == 1
    assert done[0]["status"] == "completed"


# ── lock discipline ──────────────────────────────────────────────────────────

def test_terminal_callback_runs_with_the_lock_released():
    """Rule 2. The callback POSTs over HTTP; holding the lock across it would let
    one slow network call freeze every card in the bundle."""
    world, clock, _ = make_world()
    observed = {}

    def callback(_payload):
        result = []

        def probe():
            began = time.monotonic()
            world.snapshot()                       # needs the lock
            result.append(time.monotonic() - began)

        thread = threading.Thread(target=probe)
        thread.start()
        thread.join(timeout=2.0)
        observed["acquired"] = bool(result)
        observed["waited"] = result[0] if result else None

    world.set_nav_callback(callback)
    job = world.submit_job("navigate_to", Pose(0.6, 0.0, 0.0))
    run_until(world, clock, lambda: job.terminal_posted)

    assert observed.get("acquired") is True, "another thread could not take the lock during the callback"
    assert observed["waited"] < 1.0


def test_job_is_registered_before_it_runs():
    """Rule 3. Registered-then-running; backwards you get an orphan job that
    nothing can cancel and that keeps reporting completion."""
    world, clock, _ = make_world()
    job = world.submit_job("navigate_to", Pose(5.0, 0.0, 0.0))

    assert world.snapshot()["job"]["action_id"] == job.id
    assert world.cancel_job("found it") is not None


def test_cancel_signals_before_taking_the_lock():
    """Rule 4. The event has to be set even if the lock is contended, or a cancel
    arriving mid-tick is lost."""
    world, clock, _ = make_world()
    job = world.submit_job("navigate_to", Pose(12.0, 0.0, 0.0))

    held = threading.Event()
    released = threading.Event()

    def hog():
        with world._lock:      # noqa: SLF001 - deliberately testing the lock itself
            held.set()
            released.wait(timeout=2.0)

    thread = threading.Thread(target=hog)
    thread.start()
    held.wait(timeout=2.0)

    canceller = threading.Thread(target=world.cancel_job, args=("blocked cancel",))
    canceller.start()
    time.sleep(0.05)
    assert job.cancel_event.is_set(), "cancellation must be signalled before the lock is taken"

    released.set()
    thread.join(timeout=2.0)
    canceller.join(timeout=2.0)
    assert job.terminal_posted


# ── speech ───────────────────────────────────────────────────────────────────

def test_speech_duration_follows_text_length():
    world, clock, _ = make_world(chars_per_sec=5.0)
    done = []
    world.set_speech_callback(done.append)

    world.speak("十二个字的一句讲解词啊")               # 11 chars -> 2.2 s
    run_for(world, clock, 1.0)
    assert not done, "finished too early"
    run_for(world, clock, 1.5)

    assert len(done) == 1 and done[0]["status"] == "completed"


def test_interrupting_speech_reports_cancelled_not_completed():
    world, clock, _ = make_world(chars_per_sec=5.0)
    done = []
    world.set_speech_callback(done.append)
    world.speak("这是一段很长很长的讲解词" * 5)
    run_for(world, clock, 1.0)

    world.interrupt_speech("user barge-in")

    assert len(done) == 1
    assert done[0]["status"] == "cancelled"
    assert world.snapshot()["speech"] is None


def test_queued_speech_plays_in_order_and_is_dropped_on_interrupt():
    world, clock, _ = make_world(chars_per_sec=10.0)
    done = []
    world.set_speech_callback(done.append)

    world.speak("第一句")
    world.speak("第二句")
    assert world.snapshot()["speech_queued"] == 1

    run_for(world, clock, 1.0)
    assert [event["text"] for event in done] == ["第一句"]
    assert world.snapshot()["speech"]["text"] == "第二句"

    world.interrupt_speech("stop")
    assert [event["status"] for event in done] == ["completed", "cancelled"]


def test_speech_terminal_is_posted_exactly_once():
    world, clock, _ = make_world(chars_per_sec=10.0)
    done = []
    world.set_speech_callback(done.append)
    world.speak("一句话")
    run_for(world, clock, 2.0)

    world.interrupt_speech("after it already finished")

    assert len(done) == 1 and done[0]["status"] == "completed"


# ── event log ────────────────────────────────────────────────────────────────

def test_event_log_orders_arrival_before_its_announcement():
    """The whole point of the exhibition scenario: announcing a waypoint you have
    not reached is wrong, and only a timestamped log can prove ordering."""
    world, clock, _ = make_world(chars_per_sec=10.0)
    job = world.submit_job("navigate_to", Pose(1.0, 0.0, 0.0), label="入口")
    run_until(world, clock, lambda: job.terminal_posted)
    clock.advance(0.4)          # the LLM round-trip between arriving and deciding to speak
    world.speak("入口到了")
    run_for(world, clock, 1.0)

    events = [record["event"] for record in world.events()]
    arrive_at = next(r["t"] for r in world.events() if r["event"] == "arrive")
    speak_at = next(r["t"] for r in world.events() if r["event"] == "speak_start")

    assert events[0] == "nav_start"
    assert speak_at > arrive_at


def test_snapshot_does_not_leak_mutable_world_state():
    world, _, _ = make_world()
    snap = world.snapshot()
    snap["pose"]["x"] = 999.0
    snap["joints"].append(1.23)

    assert world.snapshot()["pose"]["x"] == 0.0
    assert len(world.snapshot()["joints"]) == 2


def test_reset_clears_jobs_speech_and_events():
    world, clock, _ = make_world()
    world.submit_job("navigate_to", Pose(5.0, 0.0, 0.0))
    world.speak("something")
    run_for(world, clock, 1.0)

    world.reset({"spawn": {"x": 1.0, "y": 1.0, "yaw": 0.0}, "dof": 2})

    snap = world.snapshot()
    assert snap["job"] is None and snap["speech"] is None
    assert world.events() == []
    assert snap["pose"] == {"x": 1.0, "y": 1.0, "yaw": 0.0}


# ── grid serialisation ───────────────────────────────────────────────────────

def test_grid_round_trips_through_its_dict_form():
    grid = OccupancyGrid.blank(0.05, (-2.0, -3.0), 80, 60)
    grid.fill_rect(0.0, 0.0, 0.5, 0.5, OCCUPIED)
    grid.border(OCCUPIED)

    restored = OccupancyGrid.from_dict(grid.to_dict())

    assert restored.resolution == grid.resolution
    assert restored.origin == grid.origin
    assert (restored.width, restored.height) == (grid.width, grid.height)
    assert restored.cells == grid.cells


def test_grid_rejects_a_cell_count_that_does_not_match():
    with pytest.raises(ValueError):
        OccupancyGrid(0.05, (0.0, 0.0), 10, 10, bytearray(99))
