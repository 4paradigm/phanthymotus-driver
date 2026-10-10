"""SDK and card integration contracts with extracted code and fake transport."""
import ast
import contextlib
import json
import io
import logging
import math
from pathlib import Path
import queue
import threading
import multiprocessing
import types
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_g1_loco_timing import ROOT, World, extract_class
from unitree.g1.timed_motion import TimedMotion


class FakeThread:
    def __init__(self, target, args=(), daemon=False):
        self.target, self.args, self.daemon = target, args, daemon
    def start(self):
        pass


def card_namespace():
    return dict(json=json, math=math, time=time,
                _SMS=SimpleNamespace(LOCO_STATES={500,501,801,802}, LIMP_STATES={0,1},
                                     BALANCED_SQUAT=706, LIE_TO_STAND=702),
                threading=SimpleNamespace(Thread=FakeThread))


class TestLocoCard(unittest.TestCase):
    def card(self, smart=None, fallback=None):
        namespace = card_namespace()
        cls = extract_class(ROOT / 'device.py', 'LocoPlugin', namespace)
        card = cls.__new__(cls)
        card._smart_motion = smart
        card._fallback_motion = fallback
        card._client = None
        card._slam_client = None
        card._namespace = 'g1'
        return card, namespace

    def test_card_passes_action_identity_and_limits_fallback(self):
        w = World()
        card, _ = self.card(fallback=w.controller)
        result = card.dispatch('move', dict(vx=10, vy=-10, vyaw=10, duration=1))
        self.assertEqual(w.calls[0][2], (1., -1., 2., True))
        self.assertEqual(result['action_id'], w.controller.active['id'])
        self.assertEqual(result['ret'], 0)

    def test_smart_motion_receives_card_action_identity(self):
        calls = []
        def move(vx, vy, vyaw, duration, action_id):
            calls.append(action_id)
            return {'ret': 0, 'action_id': action_id}
        card, _ = self.card(smart=SimpleNamespace(move=move))
        result = card.dispatch('move', dict(vyaw=1, duration=1))
        self.assertEqual(calls, [result['action_id']])

    def test_completion_comes_from_stop_outcome(self):
        w = World()
        w.start()
        pending = w.controller.get_result('a')
        w.timers[0].fire()
        outcome = w.controller.get_result('a')
        responses = iter([pending, outcome])
        card, namespace = self.card(smart=SimpleNamespace(get_motion_result=lambda _: next(responses)))
        notifications = []
        namespace['_loco_acp_notify'] = lambda *args, **kw: notifications.append((args, kw))
        namespace['time'] = SimpleNamespace(sleep=lambda _: None)
        card._acp_wait_move('a')
        self.assertEqual(notifications[0][0], ('a', 'completed', outcome['result']))
        self.assertEqual(len(notifications), 1)

    def test_stop_failure_notifies_error(self):
        w = World()
        w.stop_ret = 3104
        w.start()
        w.timers[0].fire()
        card, namespace = self.card(fallback=w.controller)
        notifications = []
        namespace['_loco_acp_notify'] = lambda *args, **kw: notifications.append(args)
        card._acp_wait_move('a')
        self.assertEqual(notifications[0][1], 'error')
        self.assertEqual(notifications[0][2]['stop_ret'], 3104)

    def test_replaced_action_notifies_cancelled(self):
        w = World()
        w.start('a')
        w.start('b')
        card, namespace = self.card(fallback=w.controller)
        notifications = []
        namespace['_loco_acp_notify'] = lambda *args, **kw: notifications.append(args)
        card._acp_wait_move('a')
        self.assertEqual(notifications[0][1], 'cancelled')
        self.assertEqual(notifications[0][2]['reason'], 'replaced')

    def test_explicit_stop_returns_actual_failure(self):
        w = World()
        w.start()
        w.stop_ret = 3104
        card, _ = self.card(fallback=w.controller)
        result = card.dispatch('stop_move', {})
        self.assertEqual(result['ret'], 3104)
        self.assertEqual(result['state'], 'stop_failed')


class TestSDKSplitCall(unittest.TestCase):
    def sdk(self, response_code=0, future_code=0, send_failure=False, wrong_api=False):
        sent = []
        waits = []
        removed = []
        def request(header, parameter, binary):
            return SimpleNamespace(header=header, parameter=parameter, binary=binary)
        class Stub:
            def __init__(self, _):
                pass
            def Init(self):
                pass
            def SendRequest(self, req, timeout):
                sent.append(req)
                if send_failure:
                    return None
                def get_result(timeout):
                    waits.append(req.header.identity.id)
                    return SimpleNamespace(code=future_code, value=SimpleNamespace(
                        header=SimpleNamespace(identity=SimpleNamespace(api_id=-1 if wrong_api else req.header.identity.api_id),
                                               status=SimpleNamespace(code=response_code)), data='reply'))
                return SimpleNamespace(GetResult=get_result)
            def RemoveFuture(self, request_id):
                removed.append(request_id)
        namespace = dict(time=time, json=json, threading=threading, _log=logging.getLogger('g1-test'),
                         ClientStub=Stub, Request=request, RPC_DEBUG=False,
                         RequestIdentity=lambda ident, api: SimpleNamespace(id=ident, api_id=api),
                         RequestLease=lambda ident: ident, RequestPolicy=lambda *args: args,
                         RequestHeader=lambda identity, lease, policy: SimpleNamespace(identity=identity),
                         FutureResult=SimpleNamespace(FUTURE_SUCC=0, FUTUTE_ERR_TIMEOUT=1))
        exec((ROOT / 'unitree_sdk2py/rpc/internal.py').read_text(), namespace)
        base = extract_class(ROOT / 'unitree_sdk2py/rpc/client_base.py', 'ClientBase', namespace)
        namespace['ClientBase'] = base
        client = extract_class(ROOT / 'unitree_sdk2py/rpc/client.py', 'Client', namespace)
        namespace['Client'] = client
        exec((ROOT / 'unitree_sdk2py/g1/loco/g1_loco_api.py').read_text(), namespace)
        loco = extract_class(ROOT / 'unitree_sdk2py/g1/loco/g1_loco_client.py', 'LocoClient', namespace)()
        loco.Init()
        return loco, sent, waits, removed

    def test_send_does_not_wait_and_stop_uses_same_writer_in_order(self):
        loco, sent, waits, _ = self.sdk()
        move = loco.BeginMove(0, 0, 1, True)
        stop = loco.BeginStopMove()
        self.assertEqual(waits, [])
        self.assertEqual([json.loads(req.parameter)['velocity'] for req in sent], [[0, 0, 1], [0., 0., 0.]])
        self.assertNotEqual(sent[0].header.identity.id, sent[1].header.identity.id)
        self.assertEqual(stop(), 0)  # responses may be received/waited out of order
        self.assertEqual(move(), 0)
        self.assertEqual(waits, [sent[1].header.identity.id, sent[0].header.identity.id])

    def test_move_and_stop_preserve_server_failure_code(self):
        loco, _, _, _ = self.sdk(response_code=3104)
        self.assertEqual(loco.Move(0, 0, 1, True), 3104)
        self.assertEqual(loco.StopMove(), 3104)

    def test_transport_failures_and_api_mismatch(self):
        for kwargs, expected in [({'send_failure': True}, 3102),
                                 ({'future_code': 1}, 3104),
                                 ({'wrong_api': True}, 3105)]:
            with self.subTest(kwargs=kwargs):
                loco, sent, _, removed = self.sdk(**kwargs)
                with self.assertLogs('g1-test', level='WARNING') if not kwargs.get('wrong_api') else contextlib.nullcontext():
                    self.assertEqual(loco.Move(0, 0, 1), expected)
                if kwargs.get('future_code'):
                    self.assertEqual(removed, [sent[0].header.identity.id])

    def test_unregistered_api_does_not_send(self):
        loco, sent, _, _ = self.sdk()
        self.assertEqual(loco._BeginCall(9999, '{}')(), (3103, None))
        self.assertEqual(sent, [])

    def test_future_cleanup_removes_correct_request(self):
        cls = extract_class(ROOT / 'unitree_sdk2py/rpc/request_future.py', 'RequestFutureQueue',
                            dict(Lock=threading.Lock, RequestFuture=object))
        futures = cls()
        futures.Set(123, object())
        futures.Remove(123)
        self.assertIsNone(futures.Get(123))


class TestRpcResponseIdentity(unittest.TestCase):
    def test_late_reply_cannot_be_used_as_stop_result(self):
        cls = extract_class(ROOT / 'rpc_proxy.py', 'RpcProxy', dict(time=time, threading=threading))
        proxy = cls.__new__(cls)
        proxy._lock = threading.Lock()
        proxy._req_counter = 0
        proxy._cmd_q = queue.Queue()
        proxy._result_q = queue.Queue()
        proxy._result_q.put({'_req_id': 999, 'result': 0})
        proxy._result_q.put({'_req_id': 1, 'result': 3104})
        self.assertEqual(proxy.StopMove(), 3104)


class TestRpcWorkerMotion(unittest.TestCase):
    def test_worker_fallback_uses_deadline_controller_and_returns_action_outcome(self):
        w = World()
        w.controller.client.Init = lambda: None
        w.controller.client.SetTimeout = lambda timeout: None
        w.move_hook = lambda: w.timers[0].fire()
        source = (ROOT / 'rpc_proxy.py').read_text()
        node = next(n for n in ast.parse(source).body
                    if isinstance(n, ast.FunctionDef) and n.name == '_rpc_worker')
        namespace = dict(multiprocessing=multiprocessing,
                         time=SimpleNamespace(sleep=lambda _: None))
        exec(ast.get_source_segment(source, node), namespace)
        def module(name, **attrs):
            result = types.ModuleType(name)
            result.__dict__.update(attrs)
            return result
        stubs = {
            'common': module('common', logsafe=SimpleNamespace(install=lambda **kw: None)),
            'unitree_sdk2py.core.channel': module('channel', ChannelFactoryInitialize=lambda *args: None),
            'unitree_sdk2py.g1.loco.g1_loco_client': module('loco', LocoClient=lambda: w.controller.client),
            'timed_motion': module('timed_motion', TimedMotion=lambda *args, **kw: w.controller),
            'safety_harness': module('safety_harness', _LocoTiming=lambda: None),
        }
        commands, results = queue.Queue(), queue.Queue()
        commands.put({'method': 'TimedMove', 'args': [0, 0, 1, 1, 'a'], '_req_id': 1})
        commands.put({'method': 'GetMotionResult', 'args': ['a'], '_req_id': 2})
        commands.put(None)
        with patch.dict('sys.modules', stubs), contextlib.redirect_stdout(io.StringIO()):
            namespace['_rpc_worker'](commands, results, 'fake-interface')
        move = results.get_nowait()
        outcome = results.get_nowait()
        self.assertEqual(move['_req_id'], 1)
        self.assertEqual(move['result']['ret'], 0)
        self.assertEqual(outcome['_req_id'], 2)
        self.assertEqual(outcome['result']['status'], 'completed')
        self.assertEqual(w.calls[1][1], 1.)


class TestSafetyHarnessIntegration(unittest.TestCase):
    def harness(self):
        w = World()
        source = (ROOT / 'safety_harness.py').read_text()
        names = ['clamp', 'on_motion_stop', 'do_stop', 'handle_move']
        nodes = {n.name: n for n in ast.walk(ast.parse(source))
                 if isinstance(n, ast.FunctionDef) and n.name in names}
        idle = SimpleNamespace(value='idle')
        moving = SimpleNamespace(value='moving')
        namespace = dict(
            math=math, time=SimpleNamespace(monotonic=lambda: w.now),
            MotionState=SimpleNamespace(IDLE=idle, MOVING=moving, NAVIGATING=object(), NAV_PAUSED=object()),
            SpeedZone=SimpleNamespace(NORMAL='normal', DECELERATED='decelerated', STOPPED='stopped'),
            limits=SimpleNamespace(vx_max=1., vy_max=1., vyaw_max=2., vx_decel=.5, vy_decel=.1, vyaw_decel=.2),
            obstacle_lock=threading.Lock(), obstacle_dist=1., obstacle_angle=0.,
            publish_event=lambda name, data: events.append((name, data)),
            controller=w.controller)
        events = []
        factory = '''def factory():
    state = MotionState.IDLE
    current_cmd = None
    speed_zone = SpeedZone.NORMAL
    stop_repeat_count = 0
    stop_repeat_action_id = None
    motion = controller
'''
        for name in names:
            factory += '\n'.join('    ' + line for line in ast.get_source_segment(source, nodes[name]).splitlines()) + '\n'
        factory += '''    motion.on_stop = on_motion_stop
    return handle_move, do_stop, lambda: (state.value, current_cmd, speed_zone), on_motion_stop
'''
        exec(factory, namespace)
        return w, events, namespace['factory']()

    def test_slow_move_does_not_post_start_after_stop(self):
        w, events, (move, stop, state, callback) = self.harness()
        def delayed():
            w.timers[0].fire()
            w.now = 4.2
        w.move_hook = delayed
        result = move(0, 0, 1, 1, 'a')
        self.assertEqual(result['state'], 'idle')
        self.assertEqual(state()[0], 'idle')
        self.assertEqual([e[0] for e in events], ['motion_stop'])
        self.assertEqual(w.calls[1][1], 1.)

    def test_failure_keeps_unknown_motion_active_and_reports_actual_code(self):
        w, events, (move, stop, state, callback) = self.harness()
        move(0, 0, 1, 1, 'a')
        w.stop_ret = 3104
        result = stop('command')
        self.assertEqual(result['ret'], 3104)
        self.assertEqual(state()[0], 'moving')
        self.assertEqual(events[-1][0], 'motion_stop_failed')

    def test_old_stop_callback_cannot_clear_new_command(self):
        w, events, (move, stop, state, callback) = self.harness()
        move(0, 0, 1, 1, 'a')
        move(0, 0, 1, 1, 'b')
        callback({'action_id': 'a', 'ret': 0, 'reason': 'duration_expired'})
        self.assertEqual(state()[0], 'moving')
        self.assertEqual(state()[1]['action_id'], 'b')
        self.assertEqual(events[-1][0], 'new_command')

    def test_speed_limits_match_card(self):
        w, events, (move, stop, state, callback) = self.harness()
        result = move(100, -100, 100, 1, 'a')
        self.assertEqual((result['vx'], result['vy'], result['vyaw']), (1., -1., 2.))


if __name__ == '__main__':
    unittest.main()
