"""Offline worker-level coverage for the merged RPC contracts."""
import queue
import sys
import threading
import time
import types
from unittest.mock import patch

import rpc_proxy
from timed_motion import TimedMotion


class Loco:
    def __init__(self):
        self.calls = []
        self.stop_ret = 0

    def SetTimeout(self, value): pass
    def Init(self): pass
    def SetVelocity(self, *args):
        self.calls.append(('velocity', args))
        return 0
    def StopMove(self):
        self.calls.append(('stop',))
        return self.stop_ret
    def GetFsmId(self): return (0, 500)


class Motion:
    def __init__(self, loco):
        self.loco = loco
        self.idle = True
        self.actions = {}
    def is_idle(self): return self.idle
    def start(self, vx, vy, yaw, duration, action_id=None):
        self.idle = False
        self.actions[action_id] = 'pending'
        return {'ret': 0, 'action_id': action_id}
    def get_result(self, action_id):
        return {'action_id': action_id, 'status': self.actions[action_id]}
    def stop(self, reason='command'):
        self.idle = True
        self.loco.calls.append(('motion_stop', reason))
        return {'ret': 0}


def test_worker_action_and_proposal_arbitration():
    loco = Loco()
    channel = types.ModuleType('unitree_sdk2py.core.channel')
    channel.ChannelFactoryInitialize = lambda *args: None
    client = types.ModuleType('unitree_sdk2py.g1.loco.g1_loco_client')
    client.LocoClient = lambda: loco
    timed = types.ModuleType('timed_motion')
    timed.TimedMotion = Motion
    commands, results = queue.Queue(), queue.Queue()
    modules = {'unitree_sdk2py.core.channel': channel,
               'unitree_sdk2py.g1.loco.g1_loco_client': client,
               'timed_motion': timed}
    with patch.dict(sys.modules, modules), patch.object(rpc_proxy.time, 'sleep', lambda _: None):
        worker = threading.Thread(target=rpc_proxy._rpc_worker, args=(commands, results, 'fake'))
        worker.start()
        def call(seq, method, *args):
            commands.put({'request_id': seq, 'method': method, 'args': args})
            response = results.get(timeout=2)
            assert response['request_id'] == seq
            return response
        try:
            assert call(1, 'TimedMove', 0.1, 0, 0, 1, 'act')['result']['action_id'] == 'act'
            assert call(2, 'GetMotionResult', 'act')['result']['status'] == 'pending'
            proposal = (0.1, 0, 0, time.monotonic() + 20, 'nav', 1, 77)
            assert call(3, '__apply_velocity_proposal', *proposal)['result']['applied']
            assert not call(4, 'MotionIdle')['result']['available']
            assert call(5, 'GetFsmId')['result'] == (0, 500)
            loco.stop_ret = 3104
            assert call(6, 'TimedMove', 0.2, 0, 0, 1, 'next')['result']['action_id'] == 'next'
            assert loco.calls[-1] == ('stop',)
            # A failed proposal stop does not persistently block later commands.
            assert call(7, '__apply_velocity_proposal', *proposal)['result']['applied']
            assert call(8, 'StopMove')['result'] == 3104
            assert loco.calls[-1] == ('motion_stop', 'command')
            assert call(9, 'MotionIdle')['result']['available']
            assert call(10, '__apply_velocity_proposal', *proposal)['result']['applied']
            original_stop = rpc_proxy.ProposalRpcExecutor.stop
            def stop_with_error(executor, reason, force=False):
                original_stop(executor, reason, force)
                return {'ret': 0, 'error': 'ambiguous stop'}
            with patch.object(rpc_proxy.ProposalRpcExecutor, 'stop', stop_with_error):
                assert call(11, 'StopMove')['result'] is None
            assert loco.calls[-1] == ('motion_stop', 'command')
        finally:
            commands.put(None)
            worker.join(timeout=2)
        assert not worker.is_alive()


def test_real_timed_motion_worker_allows_writes_and_stop_clears_both():
    class AsyncLoco(Loco):
        def __init__(self):
            super().__init__()
            self.stop_entered = threading.Event()
            self.stop_release = threading.Event()
            self.proposal_stop_sent = threading.Event()
        def StopMove(self):
            self.proposal_stop_sent.set()
            return super().StopMove()
        def BeginMove(self, *args):
            self.calls.append(('begin_move', args))
            return lambda: 0
        def BeginStopMove(self):
            self.calls.append(('begin_stop',))
            def wait():
                self.stop_entered.set()
                assert self.stop_release.wait(2)
                return self.stop_ret
            return wait
    loco = AsyncLoco()
    loco.stop_ret = 17
    channel = types.ModuleType('unitree_sdk2py.core.channel')
    channel.ChannelFactoryInitialize = lambda *args: None
    client = types.ModuleType('unitree_sdk2py.g1.loco.g1_loco_client')
    client.LocoClient = lambda: loco
    timed = types.ModuleType('timed_motion')
    timed.TimedMotion = TimedMotion
    commands, results = queue.Queue(), queue.Queue()
    modules = {'unitree_sdk2py.core.channel': channel,
               'unitree_sdk2py.g1.loco.g1_loco_client': client,
               'timed_motion': timed}
    with patch.dict(sys.modules, modules):
        worker = threading.Thread(target=rpc_proxy._rpc_worker, args=(commands, results, 'fake'))
        worker.start()
        seq = 0
        def call(method, *args):
            nonlocal seq
            seq += 1
            commands.put({'request_id': seq, 'method': method, 'args': args})
            response = results.get(timeout=2)
            assert response['request_id'] == seq
            return response
        try:
            assert call('TimedMove', 0.1, 0, 0, 0.05, 'a')['result']['ret'] == 0
            assert loco.stop_entered.wait(2)
            assert call('SetVelocity', 0.1, 0, 0, 1)['result'] == 0
            proposal = (0.1, 0, 0, time.monotonic() + 20, 'nav', 1, 77)
            assert call('__apply_velocity_proposal', *proposal)['result']['applied']
            assert not call('MotionIdle')['result']['available']
            # A newer proposal must receive a stop even while an older timed
            # stop is still waiting for its response.
            stop_result = []
            stopper = threading.Thread(target=lambda: stop_result.append(call('StopMove')))
            stopper.start()
            try:
                assert loco.proposal_stop_sent.wait(2)
            finally:
                loco.stop_release.set()
                stopper.join(2)
            assert not stopper.is_alive()
            assert stop_result[0]['result'] == 17
            outcome = call('GetMotionResult', 'a')['result']
            assert outcome['status'] == 'error' and outcome['result']['stop_ret'] == 17
            assert call('SetVelocity', 0.1, 0, 0, 1)['result'] == 0
            assert call('TimedMove', 0.1, 0, 0, 0, 'b')['result']['ret'] == 0
            assert call('GetMotionResult', 'a')['result'] == outcome
            assert call('StopMove')['result'] == 17
        finally:
            loco.stop_release.set()
            commands.put(None)
            worker.join(timeout=2)
        assert not worker.is_alive()


def test_concurrent_calls_keep_replies_bound_to_their_request():
    proxy = rpc_proxy.RpcProxy.__new__(rpc_proxy.RpcProxy)
    proxy._lock = threading.Lock()
    proxy._request_id = 0
    proxy._cmd_q = queue.Queue()
    proxy._result_q = queue.Queue()
    responses = {}
    def responder():
        for _ in range(2):
            cmd = proxy._cmd_q.get(timeout=2)
            proxy._result_q.put({'request_id': cmd['request_id'], 'result': cmd['method']})
    worker = threading.Thread(target=responder)
    callers = [threading.Thread(target=lambda name=name: responses.update({name: proxy._call(name, timeout=2)}))
               for name in ('first', 'second')]
    worker.start()
    for caller in callers: caller.start()
    for caller in callers: caller.join(timeout=3)
    worker.join(timeout=3)
    assert responses == {'first': 'first', 'second': 'second'}
    assert proxy._request_id == 2


def test_proposal_timeout_does_not_latch_subsequent_writes():
    proxy = rpc_proxy.RpcProxy.__new__(rpc_proxy.RpcProxy)
    proxy._lock = threading.Lock()
    proxy._request_id = 0
    proxy._cmd_q = queue.Queue()
    proxy._result_q = queue.Queue()
    proxy._motion_rpc_timeout = 0.001
    proxy._replace_timed_out_worker = lambda: None
    result = proxy.ApplyVelocityProposal(0.1, 0, 0, time.monotonic() + 1, 'nav', 1)
    assert result['error'] == 'parent_velocity_proposal_timeout'
    old = proxy._cmd_q.get_nowait()
    def move_reply():
        cmd = proxy._cmd_q.get(timeout=2)
        assert cmd['method'] == 'TimedMove'
        proxy._result_q.put({'request_id': old['request_id'], 'result': {'ret': 99}})
        proxy._result_q.put({'request_id': cmd['request_id'], 'result': {'ret': 0, 'action_id': 'action'}})
    responder = threading.Thread(target=move_reply)
    responder.start()
    assert proxy.TimedMove(0.1, 0, 0, 1, 'action')['ret'] == 0
    responder.join(timeout=2)
    assert not responder.is_alive()


def test_timed_out_worker_that_survives_kill_closes_proxy():
    class Process:
        pid = 123
        def is_alive(self): return True
        def terminate(self): pass
        def join(self, timeout): pass
        def kill(self): pass
    proxy = rpc_proxy.RpcProxy.__new__(rpc_proxy.RpcProxy)
    proxy._proc = Process()
    proxy._cmd_q = queue.Queue()
    proxy._result_q = queue.Queue()
    proxy._closed = False
    proxy._start_worker = lambda: (_ for _ in ()).throw(AssertionError('unsafe replacement'))
    proxy._replace_timed_out_worker()
    assert proxy._closed
