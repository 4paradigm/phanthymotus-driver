"""
R1 switch_mode: the standing/lying FSM sequences must not block the dispatch thread.

Measured on R1 (agent-core log, 2026-09-02): a `switch_mode(lie2standup)` call held
the MCP dispatch for 12.14s -- `_run_fsm_sequence` runs 4 steps 1s apart and waits
for each. Because every MCP `tools/call` is what the agent turn is awaiting, the
whole turn froze for those 12s: no reasoning, no speech, nothing overlapped.

So the two sequence modes now return an action_id immediately and report via the
ACP callback, the same shape G1 has used since it went async.

The single-RPC modes (damp / zero_torque / emergency_stop / get_current_mode)
deliberately stay synchronous and return *no* action_id -- that absence is what
keeps agent-core from arming a barrier for them, since this tool's parameter is
named `mode` and agent-core's x-completion filter matches on `action`
(mcp_client.py:504).

Run:  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest unitree/r1/tests -q
"""

import importlib.util
import sys
import threading
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class Message:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")


class FakeNode:
    def __init__(self, name):
        self._name = name

    def create_publisher(self, *a, **kw):
        return types.SimpleNamespace(publish=lambda msg: None)

    def create_subscription(self, *a, **kw):
        return object()

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a: None, warn=lambda *a: None,
                                     error=lambda *a: None, debug=lambda *a: None)


def install_stubs():
    rclpy = types.ModuleType("rclpy")
    sys.modules.setdefault("rclpy", rclpy)
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kw: types.SimpleNamespace(**kw)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos

    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.String = type("String", (Message,),
                           {"__init__": lambda self: setattr(self, "data", "")})
    sys.modules["std_msgs.msg"] = std_msgs

    audio_msgs = types.ModuleType("audio_msgs.msg")
    audio_msgs.AudioChunk = type("AudioChunk", (Message,), {})
    sys.modules["audio_msgs.msg"] = audio_msgs

    # device.py only needs AudioClient to exist at import time.
    for name in ("unitree_sdk2py", "unitree_sdk2py.g1", "unitree_sdk2py.g1.audio"):
        sys.modules.setdefault(name, types.ModuleType(name))
    audio_mod = types.ModuleType("unitree_sdk2py.g1.audio.g1_audio_client")
    audio_mod.AudioClient = type("AudioClient", (), {})
    sys.modules["unitree_sdk2py.g1.audio.g1_audio_client"] = audio_mod


def load_device():
    install_stubs()
    spec = importlib.util.spec_from_file_location("r1_device_under_test", ROOT / "device.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["r1_device_under_test"] = module
    spec.loader.exec_module(module)
    return module


DEV = load_device()


# ── Fakes ────────────────────────────────────────────────────────────────────

class FakeLocoClient:
    """Stands in for the R1 LocoClient. RunFsmSequence blocks like the real one."""

    def __init__(self, fsm_id=0, seq_delay=0.0, seq_result=None, seq_raises=None):
        self.fsm_id = fsm_id
        self.fsm_code = 0
        self.seq_delay = seq_delay
        self.seq_result = seq_result if seq_result is not None else {"ok": True}
        self.seq_raises = seq_raises
        self.calls = []
        self.sequence_thread = None

    def GetFsmId(self):
        return self.fsm_code, self.fsm_id

    def StopMove(self):
        self.calls.append("StopMove")
        return 0

    def Damp(self):
        self.calls.append("Damp")
        return 0

    def ZeroTorque(self):
        self.calls.append("ZeroTorque")
        return 0

    def RunFsmSequence(self, steps, interval=1.0, step_timeout=15.0):
        self.calls.append("RunFsmSequence")
        self.sequence_thread = threading.current_thread()
        if self.seq_delay:
            import time
            time.sleep(self.seq_delay)
        if self.seq_raises:
            raise self.seq_raises
        return self.seq_result


def _healthy(mode):
    """What the worker returns for a real, controlled transition."""
    transit = {"standup2lie": 702, "lie2standup": 701}[mode]
    target = {"standup2lie": 1, "lie2standup": 811}[mode]
    return {"ret": 0, "steps": [mode], "fsm_id": target, "fsm_target": target,
            "fsm_measured": True, "fsm_seen": {mode: [transit, target]}}


@pytest.fixture
def notified(monkeypatch):
    """Capture ACP callbacks instead of POSTing them."""
    seen = []
    monkeypatch.setattr(DEV, '_loco_acp_notify',
                        lambda aid, status, result, tool="loco":
                        seen.append({'action_id': aid, 'status': status,
                                     'result': result, 'tool': tool}))
    return seen


def make_plugin(**kw):
    client = FakeLocoClient(**kw)
    return DEV.LocoPlugin({}, "ubuntu", None, client), client


def wait_for(predicate, timeout=5.0):
    import time
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


# ── the regression: dispatch must not block ──────────────────────────────────

@pytest.mark.parametrize('mode,start_fsm', [
    ('lie2standup', 0),
    ('standup2lie', 811),
    ('standup2lie', 4),
])
def test_sequence_modes_return_before_the_sequence_finishes(mode, start_fsm, notified):
    """dispatch returns while RunFsmSequence is still running -- that is the point."""
    plugin, client = make_plugin(fsm_id=start_fsm, seq_delay=1.0,
                                 seq_result=_healthy(mode))

    import time
    t0 = time.monotonic()
    result = plugin.dispatch('switch_mode', {'mode': mode})
    elapsed = time.monotonic() - t0

    assert elapsed < 0.5, f'dispatch blocked for {elapsed:.2f}s'
    assert result['status'] == 'executing'
    assert result['mode'] == mode
    assert result['action_id'].startswith('r1_fsm_')

    assert wait_for(lambda: notified), 'ACP callback never fired'
    assert notified[0]['action_id'] == result['action_id']
    assert notified[0]['status'] == 'completed'
    assert notified[0]['tool'] == 'switch_mode'
    assert notified[0]['result']['mode'] == mode


def test_sequence_runs_off_the_dispatch_thread():
    plugin, client = make_plugin(fsm_id=0, seq_result=_healthy('lie2standup'))
    plugin.dispatch('switch_mode', {'mode': 'lie2standup'})
    assert wait_for(lambda: client.sequence_thread is not None)
    assert client.sequence_thread is not threading.current_thread()


def test_standup2lie_from_loco_stops_movement_in_the_worker(notified):
    """StopMove + the 1s settle used to run inline, so dispatch blocked a second
    even before the sequence started."""
    plugin, client = make_plugin(fsm_id=811, seq_result=_healthy('standup2lie'))

    import time
    t0 = time.monotonic()
    plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    elapsed = time.monotonic() - t0

    assert elapsed < 0.5, f'dispatch blocked for {elapsed:.2f}s on StopMove'
    assert wait_for(lambda: notified)
    assert client.calls == ['StopMove', 'RunFsmSequence']


def test_standup2lie_from_stance_does_not_stop_move(notified):
    """FSM 4 is not walking; StopMove there would be a spurious RPC."""
    plugin, client = make_plugin(fsm_id=4, seq_result=_healthy('standup2lie'))
    plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    assert wait_for(lambda: notified)
    assert 'StopMove' not in client.calls


# ── failures must still report ───────────────────────────────────────────────

def test_sequence_error_is_reported_as_error(notified):
    plugin, _ = make_plugin(fsm_id=0, seq_result={'error': 'step standup timed out'})
    plugin.dispatch('switch_mode', {'mode': 'lie2standup'})
    assert wait_for(lambda: notified)
    assert notified[0]['status'] == 'error'
    assert notified[0]['result']['error'] == 'step standup timed out'


def test_rpc_timeout_is_reported_as_error(notified):
    """RunFsmSequence returning None is the RPC-timeout path."""
    plugin, client = make_plugin(fsm_id=0)
    client.seq_result = None   # the default kwarg coerces None, so set it directly
    plugin.dispatch('switch_mode', {'mode': 'lie2standup'})
    assert wait_for(lambda: notified)
    assert notified[0]['status'] == 'error'
    assert 'RPC timeout' in notified[0]['result']['error']


def test_exception_in_the_worker_still_fires_the_callback(notified):
    """Without this the agent-core barrier waits out the full 90s x-completion
    timeout for a sequence that already died."""
    plugin, _ = make_plugin(fsm_id=0, seq_raises=RuntimeError('sdk exploded'))
    plugin.dispatch('switch_mode', {'mode': 'lie2standup'})
    assert wait_for(lambda: notified)
    assert notified[0]['status'] == 'error'
    assert 'RuntimeError: sdk exploded' in notified[0]['result']['error']


# ── the synchronous modes must stay synchronous ──────────────────────────────

@pytest.mark.parametrize('mode,fsm', [
    ('emergency_stop', 811),
    ('get_current_mode', 811),
])
def test_single_rpc_modes_carry_no_action_id(mode, fsm, notified):
    """agent-core arms a barrier iff the response has an action_id
    (mcp_client.py:511). These must not get one, or every query would block the
    turn waiting for a callback that never comes."""
    plugin, _ = make_plugin(fsm_id=fsm)
    result = plugin.dispatch('switch_mode', {'mode': mode})
    assert 'action_id' not in result
    assert not notified


@pytest.mark.parametrize('mode,fsm,key', [
    ('lie2standup', 811, 'info'),    # already standing
    ('standup2lie', 1, 'info'),      # already lying
])
def test_no_op_shortcuts_carry_no_action_id(mode, fsm, key, notified):
    """Same reason: nothing runs, so nothing will ever call back."""
    plugin, _ = make_plugin(fsm_id=fsm)
    result = plugin.dispatch('switch_mode', {'mode': mode})
    assert key in result
    assert 'action_id' not in result
    assert not notified


# ── damp / zero_torque are gone from the model's surface ─────────────────────

@pytest.mark.parametrize('mode', ['damp', 'zero_torque'])
def test_raw_motor_modes_are_not_offered(mode):
    """They were the only way for the model to go limp directly, and four ways to
    'lie down' in one enum invited picking a motor mode for a posture change."""
    plugin, _ = make_plugin()
    assert mode not in plugin._switch_mode_tool()['inputSchema']['properties']['mode']['enum']


@pytest.mark.parametrize('mode', ['damp', 'zero_torque'])
@pytest.mark.parametrize('fsm', [0, 1, 4, 811])
def test_raw_motor_modes_are_refused_from_every_state(mode, fsm, notified):
    """A stale cached schema or a hand-made call must get a clear refusal, not a
    silently dropped robot -- including from the ground states where they used to
    be allowed."""
    plugin, client = make_plugin(fsm_id=fsm)
    result = plugin.dispatch('switch_mode', {'mode': mode})
    assert 'error' in result
    assert 'standup2lie' in result['error']
    assert client.calls == [], 'no motor RPC may be issued'
    assert not notified


def test_the_posture_modes_survive():
    enum = make_plugin()[0]._switch_mode_tool()['inputSchema']['properties']['mode']['enum']
    assert enum == ['lie2standup', 'standup2lie', 'emergency_stop', 'get_current_mode']


def test_emergency_stop_still_damps_from_standing(notified):
    """The one remaining deliberate way to go limp. Must not be gated on state."""
    plugin, client = make_plugin(fsm_id=811)
    result = plugin.dispatch('switch_mode', {'mode': 'emergency_stop'})
    assert result['mode'] == 'emergency_stop'
    assert client.calls == ['Damp']


# ── an unreadable FSM must not be acted on ───────────────────────────────────

@pytest.mark.parametrize('mode', ['lie2standup', 'standup2lie'])
def test_failed_fsm_read_refuses_instead_of_guessing(mode, notified):
    """Every branch below keys off current_fsm. Acting on a failed read is how a
    'safe' sequence turns into a collapse -- and RpcProxy returns (3104, None) for
    a timed-out call, which used to fall straight through to the else branch."""
    plugin, client = make_plugin(fsm_id=0)
    client.fsm_code = 3104
    result = plugin.dispatch('switch_mode', {'mode': mode})
    assert 'error' in result
    assert 'action_id' not in result
    assert client.calls == []
    assert not notified


# ── schema ───────────────────────────────────────────────────────────────────

def test_schema_declares_x_completion_without_an_action_filter():
    """An `actions` filter would be matched against args["action"]
    (mcp_client.py:504), but this tool's parameter is named `mode` -- so a filter
    would never match and the barrier would never arm."""
    plugin, _ = make_plugin()
    schema = plugin._switch_mode_tool()['inputSchema']
    completion = schema['x-completion']
    assert 'actions' not in completion
    assert completion['timeout'] >= 60, 'must outlast 4 steps x 15s step_timeout'
    assert 'action' not in schema['properties']


# ── mid-transition and unknown states must be refused ────────────────────────
#
# 701/702 are "motion in progress". Every guard used to ignore them, and
# lie2standup's `fsm_to_start.get(current_fsm, 0)` defaulted anything it did not
# recognise to step 0 -- ZeroTorque() on a robot that may be halfway up.
#
# While switch_mode was synchronous the robot could not be caught in those states:
# dispatch blocked for the whole sequence and the ACP barrier held back the next
# actuator. Async dispatch leaves the window open for 6-13s.

@pytest.mark.parametrize('mode', ['lie2standup', 'standup2lie'])
@pytest.mark.parametrize('fsm', [701, 702])
def test_transitional_states_are_refused(mode, fsm, notified):
    plugin, client = make_plugin(fsm_id=fsm)
    result = plugin.dispatch('switch_mode', {'mode': mode})
    assert 'error' in result
    assert str(fsm) in result['error']
    assert 'action_id' not in result
    assert client.calls == [], 'no motor RPC may be issued mid-transition'
    assert not notified


def test_lie2standup_no_longer_zero_torques_on_an_unknown_state(notified):
    """The dangerous default. fsm=999 used to select step 0 = ZeroTorque()."""
    plugin, client = make_plugin(fsm_id=999)
    result = plugin.dispatch('switch_mode', {'mode': 'lie2standup'})
    assert 'error' in result
    assert 'Unrecognised' in result['error']
    assert 'ZeroTorque' not in client.calls
    assert client.calls == []
    assert not notified


def test_standup2lie_refuses_an_unknown_state(notified):
    plugin, client = make_plugin(fsm_id=999)
    result = plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    assert 'error' in result
    assert client.calls == []
    assert not notified


@pytest.mark.parametrize('fsm,expect_steps', [
    (0, ['damp', 'stance', 'lie2standup']),   # from zero_torque, skip step 0
    (1, ['stance', 'lie2standup']),
    (4, ['lie2standup']),
])
def test_lie2standup_still_skips_completed_steps(fsm, expect_steps, notified):
    """The recognised states must behave exactly as before."""
    captured = {}
    plugin, _ = make_plugin(fsm_id=fsm)

    def fake_async(mode, steps, stop_first=False):
        captured['steps'] = [s[2] for s in steps]
        return {'status': 'executing', 'action_id': 'test'}

    plugin._async_fsm = fake_async
    plugin.dispatch('switch_mode', {'mode': 'lie2standup'})
    assert captured['steps'] == expect_steps


# ── only one sequence at a time ──────────────────────────────────────────────

def test_a_second_posture_change_is_refused_while_one_runs(notified):
    """Two sequences moving the robot at once is how a safe call becomes a fall."""
    plugin, client = make_plugin(fsm_id=811, seq_delay=1.0,
                                 seq_result=_healthy('standup2lie'))
    first = plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    assert first['status'] == 'executing'

    # Same reading, so only the busy flag can stop the second one.
    second = plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    assert 'error' in second
    assert 'already running' in second['error']
    assert 'action_id' not in second

    assert wait_for(lambda: notified)
    assert client.calls.count('RunFsmSequence') == 1


def test_the_lock_is_released_after_a_sequence_finishes(notified):
    plugin, _ = make_plugin(fsm_id=811, seq_result=_healthy('standup2lie'))
    plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    assert wait_for(lambda: notified)
    assert not plugin._fsm_busy.locked()
    assert plugin._fsm_active is None


def test_the_lock_is_released_after_a_crash(notified):
    """A wedged lock would refuse every future posture change for the process life."""
    plugin, _ = make_plugin(fsm_id=811, seq_raises=RuntimeError('sdk exploded'))
    plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    assert wait_for(lambda: notified)
    assert notified[0]['status'] == 'error'
    assert not plugin._fsm_busy.locked()


# ── "arrived" is not the same as "moved there" ───────────────────────────────

@pytest.mark.parametrize('mode,transit,target', [
    ('standup2lie', 702, 1),
    ('lie2standup', 701, 811),
])
def test_reaching_the_target_without_the_transitional_state_is_an_error(
        mode, transit, target, notified):
    """The 2026-09-02 signature: standup2lie hit damp at the first poll, having
    never entered 702, and was reported as a clean "completed" -- so the model went
    on to say it had lain down while the robot was on the floor.

    A healthy transition sits in 702 for ~6s / 701 for ~3s, so 1s polling cannot
    miss it.
    """
    start = 811 if mode == 'standup2lie' else 4
    plugin, _ = make_plugin(fsm_id=start, seq_result={
        "ret": 0, "steps": [mode], "fsm_id": target, "fsm_target": target,
        "fsm_measured": True, "fsm_seen": {mode: [target]}})   # target only
    plugin.dispatch('switch_mode', {'mode': mode})

    assert wait_for(lambda: notified)
    assert notified[0]['status'] == 'error', 'a fall must not report as completed'
    r = notified[0]['result']
    assert r['anomaly'] == 'missing_transitional_state'
    assert r['expected_transitional'] == transit
    assert str(transit) in r['error']


@pytest.mark.parametrize('mode', ['standup2lie', 'lie2standup'])
def test_a_controlled_transition_is_reported_as_completed(mode, notified):
    start = 811 if mode == 'standup2lie' else 4
    plugin, _ = make_plugin(fsm_id=start, seq_result=_healthy(mode))
    plugin.dispatch('switch_mode', {'mode': mode})
    assert wait_for(lambda: notified)
    assert notified[0]['status'] == 'completed'
    assert 'anomaly' not in notified[0]['result']


def test_a_step_failure_is_not_relabelled_by_the_transit_check(notified):
    """An error must keep its own message, not be overwritten by the anomaly text."""
    plugin, _ = make_plugin(fsm_id=811, seq_result={
        "error": "Step 'standup2lie' failed: code=7", "fsm_seen": {}})
    plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    assert wait_for(lambda: notified)
    assert notified[0]['status'] == 'error'
    assert 'code=7' in notified[0]['result']['error']
    assert 'anomaly' not in notified[0]['result']


def test_stop_move_return_code_reaches_the_acp_result(notified):
    """StopMove clears residual motion before the lie-down. It returns non-zero on
    healthy runs too, so it is reported rather than acted on -- but it must not be
    silently dropped, since "lie down while still moving" is the leading theory for
    a controlled descent degrading into a collapse."""
    plugin, client = make_plugin(fsm_id=811, seq_result=_healthy('standup2lie'))
    client.StopMove = lambda: 127
    plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    assert wait_for(lambda: notified)
    assert notified[0]['result']['stop_move_ret'] == 127
