"""Offline contracts for sequential loco_relative_move movement."""
import ast
from pathlib import Path
import time
import json
import types
from types import SimpleNamespace
from unittest.mock import patch
import math
import threading
import unittest

from unitree.g1.loco_relative_move import LocoRelativeMovePlugin, PointBackend
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


class TestLocoRelativeMove(unittest.TestCase):
    def setup_point(self, backend=None):
        backend = backend or Backend()
        notifications, done = [], threading.Event()
        def notify(*args, **kwargs):
            notifications.append((args, kwargs))
            done.set()
        return LocoRelativeMovePlugin({}, backend, notify), backend, notifications, done

    def test_plan_uses_separate_rotation_compensation_and_body_axis_signs(self):
        point, _, _, _ = self.setup_point()
        for angle in (30, 60, 90, 180):
            stages = point.plan(dict(x=-0.6, y=0.3, angle=angle, direction="right"))
            self.assertEqual([s['stage'] for s in stages], ['straight', 'lateral', 'rotation'])
            self.assertEqual((stages[0]['vx'], stages[0]['vy'], stages[0]['vyaw']), (-0.3, 0., 0.))
            self.assertEqual((stages[1]['vx'], stages[1]['vy'], stages[1]['vyaw']), (0., 0.3, 0.))
            self.assertAlmostEqual(stages[0]['duration'], 2. / 0.6)
            self.assertAlmostEqual(stages[1]['duration'], 1. / 0.6 / 1.2)
            factor = {90: 1.35, 180: 48. / 35.}.get(angle, 1.)
            self.assertAlmostEqual(stages[2]['duration'], math.radians(angle) / 0.5 * factor)
            self.assertEqual(stages[2]['vyaw'], -0.5)

    def test_compensation_is_limited_to_measured_speed(self):
        point, _, _, _ = self.setup_point()
        point.yaw_speed = 0.7
        for angle in (90, 180):
            for direction in ('left', 'right'):
                stage = point.plan(dict(angle=angle, direction=direction))[0]
                self.assertAlmostEqual(stage['duration'], math.radians(angle) / 0.7)
                self.assertEqual(abs(stage['vyaw']), 0.7)

    def test_translation_compensation_keeps_command_speed_and_applies_to_both_axes(self):
        point, _, _, _ = self.setup_point()
        stages = point.plan(dict(x=1, y=-1))
        self.assertEqual((stages[0]['vx'], stages[1]['vy']), (0.3, -0.3))
        self.assertAlmostEqual(stages[0]['duration'], 1 / 0.3 / 0.6)
        self.assertAlmostEqual(stages[1]['duration'], 1 / 0.3 / 0.6 / 1.2)
        point.speed = 0.5
        stages = point.plan(dict(x=1, y=-1))
        self.assertEqual((stages[0]['vx'], stages[1]['vy']), (0.5, -0.5))
        self.assertAlmostEqual(stages[0]['duration'], 2.)
        self.assertAlmostEqual(stages[1]['duration'], 2.)

    def test_canvas_string_angles_match_numeric_plan(self):
        point, _, _, _ = self.setup_point()
        for angle in (0, 30, 60, 90, 180):
            with self.subTest(angle=angle):
                self.assertEqual(point.plan(dict(x='1', y='1', angle=str(angle), direction='left')),
                                 point.plan(dict(x=1, y=1, angle=angle, direction='left')))
        for angle in (45, '45', '45.5', 'bad', '', 'nan', 'inf', None, True):
            with self.subTest(invalid=angle):
                self.assertIn('error', point.dispatch('move', dict(angle=angle)))

    def test_distance_bounds_allow_zero_and_both_signed_endpoints(self):
        point, backend, notices, _ = self.setup_point()
        for axis in ('x', 'y'):
            for distance in (0, 0.3, -0.3, 5, -5):
                with self.subTest(axis=axis, distance=distance):
                    stages = point.plan({axis: distance})
                    self.assertEqual(len(stages), int(distance != 0))
            for distance in (0.299, -0.299, 5.001, -5.001):
                with self.subTest(axis=axis, distance=distance):
                    self.assertIn('error', point.dispatch('move', {axis: distance}))
        self.assertEqual(backend.calls, [])
        self.assertEqual(notices, [])

    def test_whole_sequence_notifies_once_with_distinct_internal_ids(self):
        point, backend, notices, done = self.setup_point()
        result = point.dispatch('move', dict(x=0.3, y=-0.3, angle=90))
        self.assertTrue(done.wait(2))
        self.assertEqual(len(backend.calls), 3)
        self.assertEqual(len({call[4] for call in backend.calls}), 3)
        self.assertEqual(len(notices), 1)
        self.assertEqual(notices[0][0][:2], (result['action_id'], 'completed'))
        self.assertEqual(notices[0][1]['tool'], 'loco_relative_move')
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

    def test_stop_failure_blocks_new_jobs_until_successful_retry(self):
        point, backend, _, done = self.setup_point()
        backend.status = 'pending'
        backend.stop_ret = 3104
        point.dispatch('move', dict(x=.3))
        self.assertTrue(backend.sent.wait(2))
        point.cancel_and_stop()
        self.assertTrue(done.wait(2))
        self.assertIn('error', point.dispatch('move', dict(x=.3)))
        backend.stop_ret = 0
        self.assertEqual(point.cancel_and_stop()['ret'], 0)
        self.assertFalse(point.faulted)

    def test_manual_stop_does_not_wait_for_move_ack(self):
        release, sent, stopped = threading.Event(), threading.Event(), threading.Event()
        backend = Backend()
        def begin(*args):
            backend.calls.append(args)
            sent.set()
            def wait():
                release.wait(2)
                return {'ret': 0}
            return wait
        backend._begin_motion = begin
        original_stop = backend._stop_motion
        def stop():
            stopped.set()
            return original_stop()
        backend._stop_motion = stop
        point, _, notices, done = self.setup_point(backend)
        point.dispatch('move', dict(x=.3, y=.3))
        try:
            self.assertTrue(sent.wait(1))
            caller = threading.Thread(target=point.cancel_and_stop)
            caller.start()
            self.assertTrue(stopped.wait(1))
            self.assertFalse(release.is_set())
            caller.join(1)
            self.assertFalse(caller.is_alive())
        finally:
            release.set()
        self.assertTrue(done.wait(2))
        self.assertEqual(len(backend.calls), 1)
        self.assertEqual(notices[0][0][1], 'cancelled')

    def test_shutdown_cancels_joins_and_closes(self):
        point, backend, notices, done = self.setup_point()
        backend.status = 'pending'
        closed = threading.Event()
        backend.close = closed.set
        point.dispatch('move', dict(x=.3, y=.3))
        self.assertTrue(backend.sent.wait(1))
        point.stop()
        self.assertTrue(done.is_set())
        self.assertTrue(closed.is_set())
        self.assertIn('error', point.dispatch('move', dict(x=.3)))
        self.assertEqual(notices[0][0][1], 'cancelled')

    def test_prepare_failure_never_sends_a_movement(self):
        point, backend, notices, done = self.setup_point()
        point.prepare = lambda: {'ret': 3104}
        point.dispatch('move', dict(x=.3))
        self.assertTrue(done.wait(1))
        self.assertEqual(backend.calls, [])
        self.assertEqual(notices[0][0][1], 'error')

    def test_schema_declares_base_and_completion(self):
        point, _, _, _ = self.setup_point()
        schema = point.get_tools()[0]['inputSchema']
        self.assertEqual(schema['x-resource'], 'base')
        self.assertEqual(schema['x-completion']['actions'], ['move'])
        self.assertEqual(schema['properties']['angle']['enum'], [0, 30, 60, 90, 180])
        for axis in ('x', 'y'):
            field = schema['properties'][axis]
            self.assertEqual(field['title'], f'{axis.upper()} (m)')
            self.assertEqual((field['minimum'], field['maximum']), (-5, 5))
            self.assertEqual(field['anyOf'], [{'const': 0}, {'minimum': 0.3}, {'maximum': -0.3}])
        self.assertEqual(schema['properties']['angle']['title'], 'Angle (°)')


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

    def test_stop_timeout_recovers_before_terminal_outcome(self):
        replies, stops = iter([3104, 0]), []
        class Client:
            def BeginMove(self, *args):
                return lambda: 0
            def BeginStopMove(self):
                stops.append('stop')
                return lambda: next(replies)
        class Timer:
            def __init__(self, *args, **kwargs): pass
            def start(self): pass
            def cancel(self): pass
        controller = PointMotion(Client(), timer_factory=Timer)
        controller.start(.3, 0, 0, 1, 'step')
        controller._expire('step', 1)
        outcome = controller.get_result('step')
        self.assertEqual(len(stops), 2)
        self.assertEqual(outcome['status'], 'completed')
        self.assertTrue(outcome['result']['stop_acknowledged'])
        self.assertIsNone(outcome['result']['error'])

    def test_stop_timeout_retries_are_bounded_and_fail_closed(self):
        stops = []
        class Client:
            def BeginStopMove(self):
                stops.append('stop')
                return lambda: 3104
        controller = PointMotion(Client())
        self.assertEqual(controller.stop()['ret'], 3104)
        self.assertEqual(len(stops), 3)

    def test_old_stop_timeout_does_not_send_retry_after_replacement(self):
        stops, controller = [], None
        class Timer:
            def __init__(self, *args, **kwargs): pass
            def start(self): pass
            def cancel(self): pass
        class Client:
            def BeginMove(self, *args):
                return lambda: 0
            def BeginStopMove(self):
                stops.append('stop')
                def wait():
                    controller.start(.3, 0, 0, 1, 'new')
                    return 3104
                return wait
        controller = PointMotion(Client(), timer_factory=Timer)
        controller.start(.3, 0, 0, 1, 'old')
        controller._expire('old', 1)
        self.assertEqual(len(stops), 1)
        self.assertEqual(controller.get_result('new')['status'], 'pending')

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

    def test_close_fails_pending_waiters_and_closes_channels_once(self):
        rpc = PointRpc.__new__(PointRpc)
        rpc.lock, rpc.closed = threading.Lock(), False
        failures, closes = [], []
        rpc.pending = {1: SimpleNamespace(Fail=failures.append)}
        rpc.reader = SimpleNamespace(Close=lambda: closes.append('reader'))
        rpc.writer = SimpleNamespace(Close=lambda: closes.append('writer'))
        rpc.close()
        rpc.close()
        self.assertEqual(len(failures), 1)
        self.assertEqual(closes, ['reader', 'writer'])
        self.assertEqual(rpc.pending, {})


class TestSmartMotionPointIntegration(unittest.TestCase):
    def harness(self):
        source = (Path(__file__).resolve().parents[1] / 'unitree/g1/safety_harness.py').read_text()
        process = next(n for n in ast.parse(source).body
                       if isinstance(n, ast.FunctionDef) and n.name == '_run_smart_motion_process')
        selected = ('cancel_point', 'point_stopped', 'do_stop', 'emergency_stop',
                    'handle_point_move', 'handle_point_result', 'update_velocity')
        functions = [n for n in process.body if isinstance(n, ast.FunctionDef) and n.name in selected]
        calls, events = [], []
        class Client:
            def BeginMove(self, *args):
                calls.append(('move', args))
                return lambda: 0
            def BeginStopMove(self):
                calls.append(('stop', ()))
                return lambda: 0
            def StopMove(self):
                calls.append(('legacy_stop', ()))
                return 0
            def SetFsmId(self, value):
                calls.append(('fsm', value))
        class Timer:
            def __init__(self, *args, **kwargs):
                pass
            def start(self):
                pass
            def cancel(self):
                pass
        class Thread:
            def __init__(self, target, **kwargs):
                self.target = target
            def start(self):
                self.target()
        client = Client()
        namespace = dict(time=time, threading=SimpleNamespace(Thread=Thread),
            MotionState=SimpleNamespace(IDLE='idle', MOVING='moving', NAVIGATING='nav', NAV_PAUSED='paused'),
            SpeedZone=SimpleNamespace(NORMAL='normal', STOPPED='stopped'),
            clamp=lambda vx, vy, vyaw, zone: (vx, vy, vyaw),
            publish_event=lambda kind, data: events.append((kind, data)), loco_client=client)
        text = """def factory():
    point_motion = None
    point_sequence = None
    point_cancelled = {}
    state = MotionState.IDLE
    current_cmd = None
    speed_zone = SpeedZone.NORMAL
    stop_repeat_count = 0
    move_timer = None
    tilt_triggered = False
    process_safety_checks = lambda: None
"""
        for node in functions:
            text += '\n'.join('    ' + line for line in ast.get_source_segment(source, node).splitlines()) + '\n'
        text += """    def snapshot():
        return point_motion, point_sequence, state, point_cancelled
    return locals()
"""
        exec(text, namespace)
        env = namespace['factory']()
        modules = {
            'point_motion': SimpleNamespace(PointMotion=lambda c, **kw: PointMotion(c, timer_factory=Timer, **kw)),
            'point_rpc': SimpleNamespace(PointRpc=lambda: client),
        }
        return env, calls, events, modules

    def command(self, index=0):
        return dict(vx=.3, vy=0., vyaw=0., duration=1.,
                    action_id=f'point_job_{index}', sequence_id='point_job')

    def test_emergency_interrupt_cancels_sequence_and_rejects_next_step(self):
        for reason in ('tilt', 'foot_airborne', 'comm_timeout', 'joint_overheat'):
            with self.subTest(reason=reason):
                env, calls, events, modules = self.harness()
                with patch.dict('sys.modules', modules):
                    self.assertEqual(env['handle_point_move'](self.command())['ret'], 0)
                    env['emergency_stop'](reason)
                    result = env['handle_point_result'](self.command())
                    self.assertEqual(result['status'], 'cancelled')
                    self.assertTrue(result['result']['stop_acknowledged'])
                    self.assertIn('error', env['handle_point_move'](self.command(1)))
                self.assertEqual(len([c for c in calls if c[0] == 'move']), 1)
                self.assertTrue(any(kind == 'safety_stop' for kind, _ in events))

    def test_safety_interrupt_between_steps_latches_entire_sequence(self):
        env, calls, _, modules = self.harness()
        with patch.dict('sys.modules', modules):
            env['handle_point_move'](self.command())
            motion = env['snapshot']()[0]
            motion._expire('point_job_0', 1.)
            self.assertEqual(motion.get_result('point_job_0')['status'], 'completed')
            env['emergency_stop']('tilt')
            self.assertIn('error', env['handle_point_move'](self.command(1)))
            self.assertEqual(env['handle_point_result'](self.command())['status'], 'cancelled')
        self.assertEqual(len([c for c in calls if c[0] == 'move']), 1)

    def test_obstacle_stop_is_tracked_and_speed_updates_use_same_owner(self):
        env, calls, _, modules = self.harness()
        with patch.dict('sys.modules', modules):
            env['handle_point_move'](self.command())
            env['update_velocity'](.1, 0, 0)
            self.assertEqual(calls[-1][1][:3], (.1, 0, 0))
            self.assertEqual(env['do_stop']('obstacle')['ret'], 0)
            self.assertEqual(env['handle_point_result'](self.command())['status'], 'cancelled')
            self.assertIn('error', env['handle_point_move'](self.command(1)))

    def test_deadline_stop_retry_recovers_harness_state(self):
        env, calls, _, modules = self.harness()
        with patch.dict('sys.modules', modules):
            env['handle_point_move'](self.command())
            motion = env['snapshot']()[0]
            replies = iter([3104, 0])
            attempts = []
            def begin_stop():
                attempts.append(1)
                return lambda: next(replies)
            motion.client.BeginStopMove = begin_stop
            motion._expire('point_job_0', 1.)
            self.assertEqual(len(attempts), 2)
            self.assertEqual(env['snapshot']()[2], 'idle')
            self.assertEqual(env['handle_point_result'](self.command())['status'], 'completed')

    def test_safety_stop_retry_never_resumes_sequence(self):
        env, calls, _, modules = self.harness()
        with patch.dict('sys.modules', modules):
            env['handle_point_move'](self.command())
            motion = env['snapshot']()[0]
            replies = iter([3104, 0])
            motion.client.BeginStopMove = lambda: (lambda: next(replies))
            self.assertEqual(env['do_stop']('obstacle')['ret'], 0)
            self.assertEqual(env['handle_point_result'](self.command())['status'], 'cancelled')
            self.assertIn('error', env['handle_point_move'](self.command(1)))
        self.assertEqual(len([c for c in calls if c[0] == 'move']), 1)

    def test_failed_safety_stop_reports_error_and_still_rejects_next_step(self):
        env, calls, _, modules = self.harness()
        with patch.dict('sys.modules', modules):
            env['handle_point_move'](self.command())
            motion = env['snapshot']()[0]
            motion.client.BeginStopMove = lambda: (lambda: 3104)
            self.assertEqual(env['do_stop']('obstacle')['ret'], 3104)
            result = env['handle_point_result'](self.command())
            self.assertEqual(result['status'], 'error')
            self.assertFalse(result['result']['stop_acknowledged'])
            self.assertIn('error', env['handle_point_move'](self.command(1)))
        self.assertEqual(len([c for c in calls if c[0] == 'move']), 1)

    def test_point_backend_only_calls_safety_proxy(self):
        calls = []
        proxy = SimpleNamespace(
            point_move=lambda *args: calls.append(('move', args)) or {'ret': 0},
            point_result=lambda *args: {'status': 'cancelled'},
            point_stop=lambda *args: calls.append(('stop', args)) or {'ret': 0},
            point_finish=lambda *args: calls.append(('finish', args)) or {'ret': 0})
        backend = PointBackend(proxy)
        backend._begin_motion(.3, 0, 0, 1, 'point_job_0')
        self.assertEqual(calls[0][1][-1], 'point_job')
        self.assertEqual(backend._motion_result('point_job_0')['status'], 'cancelled')
        backend._stop_motion()
        backend.finish_sequence()
        self.assertEqual([c[0] for c in calls], ['move', 'stop', 'finish'])
        with self.assertRaises(ValueError):
            PointBackend(None)
