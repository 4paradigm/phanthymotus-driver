"""Offline contracts for sequential loco_point movement."""
import json
import types
from types import SimpleNamespace
from unittest.mock import patch
import math
import threading
import unittest

from unitree.g1.loco_point import LocoPointPlugin
from unitree.g1.point_motion import PointMotion
from unitree.g1.point_rpc import PointRpc


class Backend:
    def __init__(self):
        self.calls = []
        self.status = "completed"
        self.ret = 0
        self.stop_ret = 0
        self.sent = threading.Event()
        self.stop_count = 0

    def _start_motion(self, vx, vy, vyaw, duration, action_id):
        self.calls.append((vx, vy, vyaw, duration, action_id))
        self.sent.set()
        return {"ret": self.ret, "action_id": action_id}

    def _motion_result(self, action_id):
        return {"status": self.status, "result": {"stop_acknowledged": True}}

    def _stop_motion(self):
        self.stop_count += 1
        return {"ret": self.stop_ret}


class TestLocoPoint(unittest.TestCase):
    def setup_point(self, backend=None):
        backend = backend or Backend()
        notifications, done = [], threading.Event()
        def notify(*args, **kwargs):
            notifications.append((args, kwargs))
            done.set()
        return LocoPointPlugin({}, backend, notify), backend, notifications, done

    def test_plan_has_no_compensation_and_uses_body_axis_signs(self):
        point, _, _, _ = self.setup_point()
        for angle in (30, 45, 60, 90, 180):
            stages = point.plan(dict(x=-0.6, y=0.3, angle=angle, direction="right"))
            self.assertEqual([s['stage'] for s in stages], ['straight', 'lateral', 'rotation'])
            self.assertEqual((stages[0]['vx'], stages[0]['vy'], stages[0]['vyaw']), (-0.3, 0., 0.))
            self.assertEqual((stages[1]['vx'], stages[1]['vy'], stages[1]['vyaw']), (0., 0.3, 0.))
            self.assertAlmostEqual(stages[0]['duration'], 2.)
            self.assertAlmostEqual(stages[1]['duration'], 1.)
            self.assertAlmostEqual(stages[2]['duration'], math.radians(angle) / 0.5)
            self.assertEqual(stages[2]['vyaw'], -0.5)

    def test_whole_sequence_notifies_once_with_distinct_internal_ids(self):
        point, backend, notices, done = self.setup_point()
        result = point.dispatch('move', dict(x=0.3, y=-0.3, angle=90))
        self.assertTrue(done.wait(2))
        self.assertEqual(len(backend.calls), 3)
        self.assertEqual(len({call[4] for call in backend.calls}), 3)
        self.assertEqual(len(notices), 1)
        self.assertEqual(notices[0][0][:2], (result['action_id'], 'completed'))
        self.assertEqual(notices[0][1]['tool'], 'loco_point')
        self.assertEqual(notices[0][0][2]['completed_stages'], ['straight', 'lateral', 'rotation'])

    def test_cancel_stops_and_prevents_remaining_stages(self):
        point, backend, notices, done = self.setup_point()
        backend.status = 'pending'
        point.dispatch('move', dict(x=0.3, y=0.3, angle=90))
        self.assertTrue(backend.sent.wait(2))
        self.assertIn('error', point.dispatch('move', dict(x=0.3)))
        self.assertEqual(point.dispatch('stop_move', {})['ret'], 0)
        self.assertTrue(done.wait(2))
        self.assertEqual(len(backend.calls), 1)
        self.assertEqual(backend.stop_count, 1)
        self.assertEqual(notices[0][0][1], 'cancelled')

    def test_failed_cancel_stop_reports_error(self):
        point, backend, notices, done = self.setup_point()
        backend.status = 'pending'
        backend.stop_ret = 3104
        point.dispatch('move', dict(x=0.3, y=0.3))
        self.assertTrue(backend.sent.wait(2))
        self.assertEqual(point.cancel_and_stop()['ret'], 3104)
        self.assertTrue(done.wait(2))
        self.assertEqual(notices[0][0][1], 'error')
        self.assertEqual(len(backend.calls), 1)

    def test_safety_stop_or_failure_does_not_advance(self):
        for status in ('cancelled', 'error'):
            with self.subTest(status=status):
                point, backend, notices, done = self.setup_point()
                backend.status = status
                point.dispatch('move', dict(x=0.3, y=0.3, angle=90))
                self.assertTrue(done.wait(2))
                self.assertEqual(len(backend.calls), 1)
                self.assertEqual(notices[0][0][1], status)

    def test_failed_submission_preserves_error_and_requests_stop(self):
        point, backend, notices, done = self.setup_point()
        backend.ret = 3104
        point.dispatch('move', dict(x=0.3, y=0.3))
        self.assertTrue(done.wait(2))
        self.assertEqual(len(backend.calls), 1)
        self.assertEqual(backend.stop_count, 1)
        self.assertEqual(notices[0][0][1], 'error')
        self.assertIn('3104', notices[0][0][2]['error'])

    def test_invalid_inputs_and_zero_plan_do_not_move(self):
        point, backend, notices, _ = self.setup_point()
        for args in (dict(x=float('nan')), dict(y=float('inf')), dict(angle=10),
                     dict(angle=True), dict(direction='bad'), dict(x=100), dict(x=None)):
            with self.subTest(args=args):
                self.assertIn('error', point.dispatch('move', args))
        reply = point.dispatch('move', {})
        self.assertEqual(reply['state'], 'idle')
        self.assertNotIn('action_id', reply)
        self.assertEqual(backend.calls, [])
        self.assertEqual(notices, [])

    def test_base_dispatch_blocks_move_and_allows_stop(self):
        point, backend, notices, done = self.setup_point()
        backend.status = 'pending'
        point.dispatch('move', dict(x=0.3, y=0.3))
        self.assertTrue(backend.sent.wait(2))
        class Other:
            def dispatch(self, action, args):
                return {'action': action}
        self.assertIn('error', point.dispatch_base(Other(), 'move', {}))
        self.assertEqual(point.dispatch_base(Other(), 'stop_move', {})['action'], 'stop_move')
        self.assertTrue(done.wait(2))
        self.assertEqual(len(backend.calls), 1)

    def test_schema_declares_base_and_completion(self):
        point, _, _, _ = self.setup_point()
        schema = point.get_tools()[0]['inputSchema']
        self.assertEqual(schema['x-resource'], 'base')
        self.assertEqual(schema['x-completion']['actions'], ['move'])
        self.assertEqual(schema['properties']['angle']['enum'], [0, 30, 45, 60, 90, 180])


class TestPointMotion(unittest.TestCase):
    def test_stop_without_start_sends_zero_velocity(self):
        calls = []
        class Client:
            def BeginStopMove(self):
                calls.append('stop')
                return lambda: 0
        controller = PointMotion(Client())
        self.assertEqual(controller.stop()['ret'], 0)
        self.assertEqual(calls, ['stop'])

    def test_deadline_stop_is_independent_of_move_response(self):
        sent, stopped, release = threading.Event(), threading.Event(), threading.Event()
        class Client:
            def BeginMove(self, *args):
                sent.set()
                def wait():
                    if not release.wait(2):
                        raise TimeoutError('test release missing')
                    return 0
                return wait
            def BeginStopMove(self):
                stopped.set()
                return lambda: 0
        controller = PointMotion(Client())
        result = []
        thread = threading.Thread(target=lambda: result.append(controller.start(0.3, 0, 0, .05, 'step')))
        thread.start()
        try:
            self.assertTrue(sent.wait(1))
            self.assertTrue(stopped.wait(1))
            self.assertTrue(thread.is_alive())
            self.assertEqual(controller.get_result('step')['status'], 'pending')
        finally:
            release.set()
            thread.join(2)
        self.assertEqual(controller.get_result('step')['status'], 'completed')

    def test_move_failure_is_not_reported_completed(self):
        class Client:
            def BeginMove(self, *args):
                return lambda: 3104
            def BeginStopMove(self):
                return lambda: 0
        controller = PointMotion(Client())
        self.assertEqual(controller.start(.3, 0, 0, 1, 'failed')['ret'], 3104)
        self.assertEqual(controller.get_result('failed')['status'], 'error')


class TestPointRpc(unittest.TestCase):
    def test_transport_preserves_velocity_codes_and_cleans_pending(self):
        def module(name, **attrs):
            result = types.ModuleType(name)
            result.__dict__.update(attrs)
            return result
        class Future:
            def SetRequestId(self, request_id):
                self.id = request_id
            def Ready(self, response):
                self.response = response
            def GetResult(self, timeout):
                if hasattr(self, 'response'):
                    return SimpleNamespace(code=0, value=self.response)
                return SimpleNamespace(code=1)
        stubs = {
            'unitree_sdk2py.idl.unitree_api.msg.dds_': module('idl',
                Request_=lambda header, parameter, binary: SimpleNamespace(header=header, parameter=parameter),
                RequestHeader_=lambda identity, lease, policy: SimpleNamespace(identity=identity),
                RequestIdentity_=lambda request_id, api: SimpleNamespace(id=request_id, api_id=api),
                RequestLease_=lambda value: value, RequestPolicy_=lambda *args: args),
            'unitree_sdk2py.g1.loco.g1_loco_api': module('api', ROBOT_API_ID_LOCO_SET_VELOCITY=7105),
            'unitree_sdk2py.rpc.request_future': module('future', RequestFuture=Future),
            'unitree_sdk2py.utils.future': module('results',
                FutureResult=SimpleNamespace(FUTURE_SUCC=0, FUTUTE_ERR_TIMEOUT=1)),
            'unitree_sdk2py.rpc.internal': module('codes', RPC_ERR_CLIENT_SEND=3102,
                RPC_ERR_CLIENT_API_TIMEOUT=3104, RPC_ERR_CLIENT_API_NOT_MATCH=3105, RPC_ERR_UNKNOWN=3001),
        }
        for case, expected in [('success', 0), ('server', 3104), ('send', 3102), ('timeout', 3104), ('api', 3105)]:
            with self.subTest(case=case), patch.dict('sys.modules', stubs):
                rpc = PointRpc.__new__(PointRpc)
                rpc.lock, rpc.pending = threading.Lock(), {}
                requests = []
                def write(request, timeout):
                    requests.append(request)
                    if case == 'send':
                        return False
                    if case != 'timeout':
                        identity = SimpleNamespace(id=request.header.identity.id,
                                                   api_id=999 if case == 'api' else 7105)
                        rpc._receive(SimpleNamespace(header=SimpleNamespace(identity=identity,
                            status=SimpleNamespace(code=3104 if case == 'server' else 0))))
                    return True
                rpc.writer = SimpleNamespace(Write=write)
                wait = rpc.BeginMove(.3, -.2, .5, True)
                self.assertEqual(json.loads(requests[0].parameter),
                                 {'velocity': [.3, -.2, .5], 'duration': 864000.})
                self.assertEqual(wait(), expected)
                self.assertEqual(rpc.pending, {})
