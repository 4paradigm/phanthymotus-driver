"""G1 navigation arrival — the signal the SLAM service does *not* send.

`rt/slam_info` on a G1 carries `robot_data`, `pos_info` and `mapping_info`, and
nothing else. `ctrl_info` — the message that would carry `is_arrived` — is never
published; sampled live on the robot, and noted in two places in the driver.

So there is exactly one arrival signal: the pose stream says the robot is within
`NAV_ARRIVAL_RADIUS_M` of where it was sent. SmartMotion has always applied that
rule, but `controlled_spatial` runs with `smart_motion=None` whenever
`isolated_process: true` (which is the shipped config), and *that* path only
waited on `is_arrived`. Every navigation therefore ran to `stall_timeout` and was
reported to the agent as an error — the robot standing at its destination while
the ACP barrier held the turn open for the full 180 s.

These tests pin the fallback path specifically. Nothing here imports the Unitree
SDK, ROS or DDS: the plugin is exercised with its DDS callback driven by hand, so
a regression is caught on a laptop rather than by a robot that never says it
arrived.

Run: PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_g1_nav_arrival.py -q
"""

from __future__ import annotations

import importlib.util
import json
import sys
import time
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def _load():
    """`controlled_spatial.py` imports its sibling `safety_harness`, which is how
    the arrival radius stays a single number. In the container both sit in /work;
    here the bundle directory has to go on the path for the duration of the load.

    The module is registered under a prefixed name because more than one bundle in
    this repo has a file called `controlled_spatial.py`, and leaking a bare one
    into `sys.modules` fails an unrelated test file somewhere else.
    """
    bundle = ROOT / "unitree" / "g1"
    sys.path.insert(0, str(bundle))
    try:
        spec = importlib.util.spec_from_file_location(
            "g1_controlled_spatial", bundle / "controlled_spatial.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules["g1_controlled_spatial"] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(bundle))


cs = _load()


# ── fixtures ─────────────────────────────────────────────────────────────────


@pytest.fixture
def plugin(tmp_path, monkeypatch):
    """A plugin with no SmartMotion — i.e. exactly what the isolated subprocess
    builds — and with its DDS subscription and ACP callback replaced.

    `_SlamRpcProxy` is stubbed because the real one spawns a process that loads
    the Unitree SDK; `ChannelSubscriber` because there is no DDS here. Poses are
    fed through the plugin's own `_on_slam_info`, so the test drives the same
    code path a real `pos_info` message would.
    """
    notices = []
    monkeypatch.setattr(cs, "_acp_notify",
                        lambda aid, status, result, tool="controlled_spatial":
                        notices.append((aid, status, result)))
    monkeypatch.setattr(cs, "_SlamRpcProxy", lambda iface: types.SimpleNamespace(
        PauseNav=lambda: (0, None), NavigateTo=lambda *a, **k: (0, None)))

    p = cs.ControlledSpatialPlugin(
        {"native_slam_db_path": str(tmp_path / "cs.db"), "network_iface": "lo"},
        "test", None, slam_client=None, smart_motion=None)
    p.notices = notices
    return p


def _feed_pose(p, x, y, yaw=0.0):
    """Hand the plugin a `pos_info` the way the SLAM service would."""
    p._on_slam_info(types.SimpleNamespace(data=json.dumps({
        "type": "pos_info",
        "data": {"currentPose": {"x": x, "y": y, "q_x": 0.0, "q_y": 0.0,
                                 "q_z": 0.0, "q_w": 1.0}},
    })))


# ── the isolated child takes the fallback path at all ────────────────────────


def test_no_smart_motion_means_the_fallback_path(plugin):
    """The isolated subprocess constructs the plugin exactly like this. If this
    ever stops being true the rest of this file is testing nothing."""
    assert plugin._smart_motion is None
    assert plugin._client is not None


def test_ctrl_info_is_not_what_we_rely_on(plugin):
    """Kept wired for the day the SLAM service starts publishing it — but a robot
    that never sends it must still be able to arrive."""
    plugin._on_slam_info(types.SimpleNamespace(data=json.dumps({
        "type": "ctrl_info", "data": {"is_arrived": True}})))
    assert plugin._nav_arrived.is_set()


# ── the arrival rule ─────────────────────────────────────────────────────────


def test_arrival_is_decided_by_distance(plugin):
    target = {"x": 3.0, "y": 4.0}
    _feed_pose(plugin, 3.0, 4.0)
    assert plugin._arrived_at(target) == pytest.approx(0.0)

    _feed_pose(plugin, 3.0 + cs.NAV_ARRIVAL_RADIUS_M / 2, 4.0)
    assert plugin._arrived_at(target) is not None

    _feed_pose(plugin, 3.0 + cs.NAV_ARRIVAL_RADIUS_M * 2, 4.0)
    assert plugin._arrived_at(target) is None


def test_a_stale_pose_decides_nothing(plugin, monkeypatch):
    """`pos_info` stops the moment localization is lost, and `_current_pose` then
    reports the last fix forever. Trusting it would declare a navigation that
    never started complete, whenever the robot happened to be parked near the
    target.
    """
    target = {"x": 0.0, "y": 0.0}
    _feed_pose(plugin, 0.0, 0.0)
    assert plugin._arrived_at(target) is not None

    plugin._pose_ts = time.monotonic() - (cs.POSE_FRESH_S + 1)
    assert plugin._get_fresh_pose() is None
    assert plugin._arrived_at(target) is None


def test_the_two_wait_paths_agree_on_the_radius():
    """controlled_spatial imports the number from safety_harness rather than
    repeating it — two processes watching the same navigation must not disagree
    about where the destination is."""
    bundle = ROOT / "unitree" / "g1"
    sys.path.insert(0, str(bundle))
    try:
        spec = importlib.util.spec_from_file_location(
            "g1_safety_harness", bundle / "safety_harness.py")
        sh = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(sh)
    finally:
        sys.path.remove(str(bundle))

    assert cs.NAV_ARRIVAL_RADIUS_M == sh.NAV_ARRIVAL_RADIUS_M
    # Equality alone would still pass if someone re-typed the literal, which is
    # how the two would drift. Assert the import is what ties them together.
    source = (bundle / "controlled_spatial.py").read_text()
    assert "from safety_harness import NAV_ARRIVAL_RADIUS_M" in source


# ── what the agent is told ───────────────────────────────────────────────────


def test_reaching_the_target_completes_the_acp_action(plugin):
    """The regression itself: pose reaches the target, and the agent is told the
    action completed — not that it stalled, and not 180 s later."""
    target = {"x": 1.0, "y": 0.0}
    _feed_pose(plugin, 0.0, 0.0)

    done = []
    import threading
    t = threading.Thread(
        target=lambda: (plugin._acp_wait_nav("act-1", "kitchen", 30, target),
                        done.append(True)),
        daemon=True)
    t.start()
    time.sleep(0.2)
    assert not plugin.notices, "must not report arrival before the robot moves"

    _feed_pose(plugin, 1.0, 0.0)
    t.join(timeout=5)
    assert done, "_acp_wait_nav did not return after arrival"

    (action_id, status, result), = plugin.notices
    assert (action_id, status) == ("act-1", "completed")
    assert result["via"] == "pose"
    assert result["target"] == "kitchen"
    assert result["distance"] == pytest.approx(0.0)


def test_not_arriving_is_reported_as_a_stall_not_as_success(plugin):
    """The other half of the contract: a robot that stops short must not be
    reported as having got there."""
    target = {"x": 10.0, "y": 0.0}
    _feed_pose(plugin, 0.0, 0.0)
    plugin._acp_wait_nav("act-2", "far", 1.5, target)

    (action_id, status, result), = plugin.notices
    assert (action_id, status) == ("act-2", "error")
    assert "stall_timeout" in result["error"]


def test_a_superseded_navigation_does_not_fire(plugin):
    """A second navigate cancels the first; the first thread is still looping and
    must not later announce that the *old* target was reached."""
    target = {"x": 1.0, "y": 0.0}
    _feed_pose(plugin, 0.0, 0.0)

    import threading
    t = threading.Thread(
        target=plugin._acp_wait_nav, args=("act-3", "old", 30, target), daemon=True)
    t.start()
    time.sleep(0.2)

    plugin._nav_action_id = "act-4"   # a new navigate took over
    _feed_pose(plugin, 1.0, 0.0)
    t.join(timeout=5)

    assert not t.is_alive()
    assert plugin.notices == []
