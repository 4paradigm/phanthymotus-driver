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
    plugin, client = make_plugin(fsm_id=start_fsm, seq_delay=1.0)

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
    plugin, client = make_plugin(fsm_id=0)
    plugin.dispatch('switch_mode', {'mode': 'lie2standup'})
    assert wait_for(lambda: client.sequence_thread is not None)
    assert client.sequence_thread is not threading.current_thread()


def test_standup2lie_from_loco_stops_movement_in_the_worker(notified):
    """StopMove + the 1s settle used to run inline, so dispatch blocked a second
    even before the sequence started."""
    plugin, client = make_plugin(fsm_id=811)

    import time
    t0 = time.monotonic()
    plugin.dispatch('switch_mode', {'mode': 'standup2lie'})
    elapsed = time.monotonic() - t0

    assert elapsed < 0.5, f'dispatch blocked for {elapsed:.2f}s on StopMove'
    assert wait_for(lambda: notified)
    assert client.calls == ['StopMove', 'RunFsmSequence']


def test_standup2lie_from_stance_does_not_stop_move(notified):
    """FSM 4 is not walking; StopMove there would be a spurious RPC."""
    plugin, client = make_plugin(fsm_id=4)
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
