"""Offline R1-shaped G1 stream + actual worker/SDK-method tests; no DDS/hardware."""
import ast
import importlib.util
import json
import multiprocessing
from pathlib import Path
import queue
import runpy
import sys
import threading
import time
from types import SimpleNamespace as NS

import pytest

ROOT = Path(__file__).resolve().parents[1]
G1 = ROOT / 'unitree/g1'
sys.path.insert(0, str(ROOT))

def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

m = load('g1_loco_servo', G1 / 'loco_servo.py')
rpc = load('g1_servo_rpc', G1 / 'rpc_proxy.py')


class Client:
    def __init__(self):
        self.moves, self.stops = [], 0
        self.move_ret = self.stop_ret = 0
    def Move(self, *args): self.moves.append(args); return self.move_ret
    def StopMove(self): self.stops += 1; return self.stop_ret


def command(seq=1, values=None):
    now = int(time.time() * 1000)
    return {'schema': 'motus.control/1', 'mode': 'twist', 'dof': 6,
            'values': values if values is not None else [.2, .1, 0, 0, 0, -.3],
            'seq': seq, 'source': 'test', 'priority': 50,
            'stamp_ms': now, 'obs_stamp_ms': now, 'ttl_ms': 200}


def card(client=None, reader=None, **cfg):
    client = client or Client()
    cfg.setdefault('dry_run', False)
    obj = m.LocoServoPlugin(cfg, 'g1', NS(), client,
                           reader or (lambda: {'fsm_id': 500, 'ts': time.time()}))
    obj._open = lambda topic: None
    assert obj.dispatch('start', {'input_topic': '/test'})['state'] == 'running'
    return obj, client


def send(obj, cmd=None):
    obj._on_message(NS(data=json.dumps(cmd if cmd is not None else command())))


def watchdog(obj):
    obj._sink._last_apply_ms -= m.WATCHDOG_MS + 1
    obj._tick()


def test_descriptor_keeps_g1_units_groups_fsm_and_default_dry_run():
    obj = m.LocoServoPlugin({}, 'g1', None, Client(), lambda: {})
    descriptor = obj._descriptor_raw
    assert descriptor['joint_names'] == ['vx', 'vy', 'vz', 'wx', 'wy', 'wz']
    assert descriptor['groups'] == [
        {'name': 'base_linear', 'offset': 0, 'count': 3, 'unit': 'm/s', 'resource': 'base'},
        {'name': 'base_angular', 'offset': 3, 'count': 3, 'unit': 'rad/s', 'resource': 'base'}]
    assert descriptor['rate']['watchdog_ms'] == 300
    assert descriptor['rate']['max_obs_age_ms'] == 500
    assert m.parse_descriptor(descriptor).resources == ('base',)
    assert obj._dry_run is True
    obj.start()
    assert not obj._running and not obj._client.moves


def test_every_valid_command_is_one_continuous_move():
    obj, client = card()
    send(obj)
    assert client.moves == [(.2, .1, -.3, True)]
    cmd = command(2); del cmd['obs_stamp_ms']; send(obj, cmd)
    assert len(client.moves) == 2


@pytest.mark.parametrize('invalid', ['stale', 'pinned', 'sequence'])
def test_control_sink_still_rejects_invalid_input(invalid):
    obj, client = card()
    cmd = command()
    if invalid == 'stale': cmd['stamp_ms'] -= 1000
    elif invalid == 'pinned': cmd['values'][2] = 1
    else:
        send(obj, cmd)
        assert len(client.moves) == 1
    before = len(client.moves); send(obj, cmd)
    assert len(client.moves) == before


@pytest.mark.parametrize('snapshot', [{}, {'fsm_id': 500, 'ts': 0},
    {'fsm_id': 811, 'ts': time.time()}, {'fsm_id': 500, 'ts': float('nan')}])
def test_g1_fsm_refusal_is_preserved(snapshot):
    obj, client = card(reader=lambda: snapshot)
    send(obj)
    assert not client.moves and obj._refused == 1 and obj._fsm_problem


@pytest.mark.parametrize('stop_ret', [0, 7, 3104])
def test_watchdog_stop_does_not_latch_and_next_legal_command_recovers(stop_ret):
    obj, client = card(); client.stop_ret = stop_ret
    send(obj); watchdog(obj)
    assert client.stops == 1 and not obj._paused
    stale = command(2); stale['stamp_ms'] -= 1000
    send(obj, stale); assert len(client.moves) == 1
    send(obj, command(3)); assert len(client.moves) == 2
    assert obj._info()['physical_stop_confirmed'] is False


def test_abort_callback_uses_same_hold_without_manual_pause():
    obj, client = card()
    obj._sink._on_abort()
    assert client.stops == 1 and not obj._paused


def test_manual_pause_requires_resume_but_stop_return_code_is_not_a_latch():
    obj, client = card(); send(obj); client.stop_ret = 3104
    assert obj.dispatch('pause', {})['state'] == 'paused'
    send(obj, command(2)); assert len(client.moves) == 1
    assert obj.dispatch('resume', {})['state'] == 'running'
    send(obj, command(2)); assert len(client.moves) == 2
    result = obj.dispatch('stop', {})
    assert result == {'state': 'stopped', 'physical_stop_confirmed': False}
    send(obj, command(3)); assert len(client.moves) == 2


def test_stop_exception_is_best_effort_not_a_recovery_latch():
    obj, client = card()
    def fail(): raise RuntimeError('RPC timeout')
    client.StopMove = fail
    send(obj); watchdog(obj)
    send(obj, command(2)); assert len(client.moves) == 2
    assert obj.dispatch('stop', {})['state'] == 'stopped'


def test_dry_run_and_rotate_only_follow_r1_switches():
    obj, client = card(dry_run=True); send(obj)
    assert not client.moves
    obj.dispatch('stop', {}); assert client.stops == 0
    obj, client = card(rotate_only=True); send(obj)
    assert client.moves == [(0., 0., -.3, True)]
    # R1 config changes only future sends; it is not a stop request.
    obj.dispatch('config', {'dry_run': True})
    assert obj._dry_run and client.stops == 0


def test_move_error_code_is_visible_and_does_not_latch(capsys):
    obj, client = card(); client.move_ret = 3104; send(obj)
    assert obj._info()['last_ret'] == 3104 and obj._sdk_errors == 1
    assert 'execution unconfirmed' in capsys.readouterr().out
    client.move_ret = 0; send(obj, command(2))
    assert obj._last_ret == 0 and obj._sdk_errors == 0


def test_zero_command_is_step_clamped_not_an_atomic_stop():
    obj, client = card()
    send(obj, command(values=[.8, 0, 0, 0, 0, 0]))
    send(obj, command(2, values=[0]*6))
    assert client.moves[-1][0] == pytest.approx(.65)
    assert client.stops == 0  # Same ControlSink behavior as R1.


def test_bundle_shutdown_seals_future_start_and_resume():
    obj, _ = card()
    assert obj.stop()['state'] == 'stopped'
    assert obj.dispatch('start', {'input_topic': '/test'})['state'] == 'error'
    assert obj.dispatch('resume', {})['state'] == 'error'


@pytest.mark.parametrize('hz', [0, -1, 21, float('nan'), float('inf')])
def test_invalid_expected_frequency(hz):
    with pytest.raises(ValueError): m.LocoServoPlugin({'expected_hz': hz}, 'g1', None, Client(), lambda: {})


def test_low_expected_frequency_is_allowed_like_r1():
    obj, _ = card(expected_hz=1)
    assert obj._expected_hz == 1


@pytest.fixture
def real_worker(monkeypatch):
    """Actual _rpc_worker and G1 LocoClient methods, fake transport/base Client only."""
    sdk_root = G1 / 'unitree_sdk2py/g1/loco'
    state = {'writes': [], 'move_ret': 0, 'stop_ret': 0, 'gate': None,
             'entered': threading.Event(), 'timed_calls': []}
    class BaseClient:
        def __init__(self, *args): pass
        def SetTimeout(self, timeout): state['sdk_timeout'] = timeout
        def _SetApiVerson(self, value): pass
        def _RegistApi(self, *args): pass
        def _BeginCall(self, api, parameter):
            body = json.loads(parameter); state['writes'].append((api, body))
            is_stop = body['velocity'] == [0., 0., 0.]
            if not is_stop: state['entered'].set()
            code = state['stop_ret'] if is_stop else state['move_ret']
            def wait():
                if not is_stop and state['gate'] is not None:
                    assert state['gate'].wait(3), 'test SDK reply gate timed out'
                return code, ''
            return wait
    namespace = dict(runpy.run_path(str(sdk_root / 'g1_loco_api.py')),
                     Client=BaseClient, json=json, threading=threading)
    tree = ast.parse((sdk_root / 'g1_loco_client.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LocoClient')
    exec(compile(ast.Module(body=[cls], type_ignores=[]), 'actual_g1_loco_client', 'exec'), namespace)
    sdk_class = namespace['LocoClient']
    timed = load('g1_servo_timed_motion', G1 / 'timed_motion.py')
    class Motion(timed.TimedMotion):
        def start(self, *args, **kw):
            state['timed_calls'].append(('start', args))
            return super().start(*args, **kw)
        def stop(self, *args, **kw):
            state['timed_calls'].append(('stop', args))
            return super().stop(*args, **kw)
    import common
    monkeypatch.setattr(common, 'logsafe', NS(install=lambda **kw: None), raising=False)
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.core.channel', NS(ChannelFactoryInitialize=lambda *a: None))
    monkeypatch.setitem(sys.modules, 'unitree_sdk2py.g1.loco.g1_loco_client', NS(LocoClient=sdk_class))
    monkeypatch.setitem(sys.modules, 'timed_motion', NS(TimedMotion=Motion))
    source = (G1 / 'rpc_proxy.py').read_text()
    worker_node = next(n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name == '_rpc_worker')
    worker_ns = {'multiprocessing': multiprocessing, 'time': NS(sleep=lambda _: None)}
    exec(ast.get_source_segment(source, worker_node), worker_ns)
    proxy = rpc.RpcProxy.__new__(rpc.RpcProxy)
    proxy._lock, proxy._req_counter = threading.Lock(), 0
    proxy._cmd_q, proxy._result_q = queue.Queue(), queue.Queue()
    thread = threading.Thread(target=worker_ns['_rpc_worker'], args=(proxy._cmd_q, proxy._result_q, 'fake'))
    thread.start()
    state.update(proxy=proxy, client=rpc.DirectLocoClient(proxy))
    yield NS(**state, state=state)
    if state['gate'] is not None: state['gate'].set()
    proxy._cmd_q.put(None); thread.join(3)
    assert not thread.is_alive()


def test_direct_worker_sends_actual_sdk_ten_days_and_stop_one_second(real_worker):
    w = real_worker
    assert w.client.Move(.2, -.1, .3, True) == 0
    assert w.client.StopMove() == 0
    assert w.state['writes'] == [
        (7105, {'velocity': [.2, -.1, .3], 'duration': 864000.0}),
        (7105, {'velocity': [0., 0., 0.], 'duration': 1.0})]
    assert w.state['timed_calls'] == [] and w.state['sdk_timeout'] == 10.0


def test_existing_proxy_move_stop_still_use_timed_motion(real_worker):
    w = real_worker
    assert w.proxy.Move(.2, 0, 0, True) == 0
    assert w.state['timed_calls'][0][0] == 'start'
    assert w.proxy.StopMove() == 0
    assert w.state['timed_calls'][-1][0] == 'stop'


def test_direct_timeout_and_late_reply_keep_actual_stop_code(real_worker):
    w = real_worker; w.state['gate'] = threading.Event(); w.state['stop_ret'] = 7
    original = w.proxy._call
    w.proxy._call = lambda method, *args, **kw: original(method, *args, timeout=.02, **kw)
    assert w.client.Move(.2, 0, 0, True) == 3104
    assert w.state['writes'][0][1]['duration'] == 864000.0  # Sent despite timeout.
    w.state['gate'].set()
    w.proxy._call = original
    assert w.client.StopMove() == 7  # Late Move reply 0 cannot become StopMove ACK.


@pytest.mark.parametrize('action', ['pause', 'stop'])
def test_inflight_rpc_does_not_hold_card_lock_and_stop_follows_committed_move(real_worker, action):
    w = real_worker; w.state['gate'] = threading.Event()
    obj, _ = card(w.client)
    sender = threading.Thread(target=send, args=(obj,)); sender.start()
    assert w.state['entered'].wait(1)
    entered_stop = threading.Event()
    original_stop = w.client.StopMove
    def stop(): entered_stop.set(); return original_stop()
    w.client.StopMove = stop
    result = []
    stopper = threading.Thread(target=lambda: result.append(obj.dispatch(action, {}))); stopper.start()
    assert entered_stop.wait(1)
    assert obj._lock.acquire(blocking=False)
    obj._lock.release()
    assert obj._info()['state'] == ('paused' if action == 'pause' else 'idle')
    if action == 'stop': assert obj.dispatch('start', {'input_topic': '/other'})['state'] == 'error'
    w.state['gate'].set(); sender.join(2); stopper.join(2)
    assert not sender.is_alive() and not stopper.is_alive()
    assert [body['duration'] for _, body in w.state['writes']] == [864000.0, 1.0]
    assert result[0]['physical_stop_confirmed'] is False


@pytest.mark.parametrize('action', ['pause', 'stop'])
def test_documented_r1_pre_rpc_gap_can_send_old_move_after_stop(real_worker, action):
    w = real_worker; obj, _ = card(w.client)
    entered, release = threading.Event(), threading.Event()
    original = w.client.Move
    def delayed(*args):
        entered.set(); assert release.wait(2)
        return original(*args)
    w.client.Move = delayed
    sender = threading.Thread(target=send, args=(obj,)); sender.start()
    assert entered.wait(1)  # Card checks passed; proxy lock not yet acquired.
    assert obj.dispatch(action, {})['physical_stop_confirmed'] is False
    release.set(); sender.join(2); assert not sender.is_alive()
    assert [body['duration'] for _, body in w.state['writes']] == [1.0, 864000.0]
    count = len(w.state['writes']); send(obj, command(2))
    assert len(w.state['writes']) == count  # Only future consumption is blocked.
