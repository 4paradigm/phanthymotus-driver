"""Closed-loop relative movement and existing locomotion regressions (no hardware)."""
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
from unitree.g1.relative_feedback import PoseFeedback, RelativeController, MotionSafety, SafetyViolation, wrap
from unitree.g1.point_motion import PointMotion
from unitree.g1.point_rpc import PointRpc

class TestBundleShutdown(unittest.TestCase):
    def test_cards_stop_before_safety_peer_even_when_card_cleanup_fails(self):
        source = Path(__file__).resolve().parents[1] / 'unitree/g1/main.py'
        tree = ast.parse(source.read_text())
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'main')
        server_loop = next(node for node in reversed(main.body) if isinstance(node, ast.Try))
        cleanup = compile(ast.Module(body=server_loop.finalbody, type_ignores=[]), str(source), 'exec')
        for fail in (False, True):
            with self.subTest(card_cleanup_fails=fail):
                calls = []
                def stop_cards():
                    calls.append('cards')
                    if fail:
                        raise RuntimeError('card cleanup failed')
                namespace = {
                    '_bundle': SimpleNamespace(stop_all=stop_cards),
                    'smart_motion': SimpleNamespace(shutdown=lambda: calls.append('safety')),
                    'loco_client': SimpleNamespace(stop=lambda: calls.append('rpc')),
                    'executor': SimpleNamespace(shutdown=lambda: calls.append('executor')),
                    'rclpy': SimpleNamespace(shutdown=lambda: calls.append('ros')),
                }
                if fail:
                    with self.assertRaisesRegex(RuntimeError, 'card cleanup failed'):
                        exec(cleanup, namespace)
                else:
                    exec(cleanup, namespace)
                self.assertEqual(calls, ['cards', 'safety', 'rpc', 'executor', 'ros'])


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



def pose(x=0., y=0., yaw=0., odom_yaw=0., now=0., speed=0., rate=0.):
    return dict(x=x, y=y, yaw=yaw, roll=0., pitch=0., angular_speed=abs(rate), odom_yaw=odom_yaw, speed=speed, yaw_rate=rate,
                odom_stamp=now, imu_stamp=now)


class TestPoseFeedback(unittest.TestCase):
    def test_freshness_validation_alignment_and_yaw_unwrap(self):
        now = [10.]
        feedback = PoseFeedback(clock=lambda: now[0])
        with self.assertRaises(RuntimeError):
            feedback.snapshot()
        odom = dict(position=[1, 2, 0], velocity=[0, 0, 0], imu=dict(rpy=[0, 0, .2]))
        feedback.update_odom(odom)
        feedback.update_imu(dict(rpy=[0, 0, 3.1], gyroscope=[0, 0, 0]))
        feedback.update_imu(dict(rpy=[0, 0, -3.1], gyroscope=[0, 0, 0]))
        self.assertGreater(feedback.snapshot()['yaw'], math.pi)
        now[0] += .3
        feedback.update_imu(dict(rpy=[0, 0, -3.], gyroscope=[0, 0, 0]))
        with self.assertRaisesRegex(RuntimeError, 'synchronized'):
            feedback.snapshot()
        now[0] += .3
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            feedback.snapshot()
        feedback.update_odom(dict(position=[float('nan'), 0], velocity=[0, 0], imu=dict(rpy=[0, 0, 0])))
        with self.assertRaises(RuntimeError):
            feedback.snapshot()


class TestRelativeController(unittest.TestCase):
    def test_each_active_translation_axis_exceeds_dead_zone_and_respects_cap(self):
        for x, y in ((.06, 0), (-.06, 0), (0, .06), (0, -.06), (.04, .04), (1, -.04), (.04, -1)):
            for cap in (.2, .25, .3, 1.):
                with self.subTest(x=x, y=y, cap=cap):
                    controller = RelativeController(pose(), dict(x=x, y=y, yaw=0), cap, .5, 0.)
                    (vx, vy, _), done = controller.step(pose(), 0.)
                    self.assertFalse(done)
                    self.assertTrue(vx or vy)
                    self.assertLessEqual(math.hypot(vx, vy), cap + 1e-9)
                    for command, error in ((vx, x), (vy, y)):
                        if command:
                            self.assertGreaterEqual(abs(command), .2 - 1e-9)
                            self.assertGreater(command * error, 0)

    def test_translation_stops_inside_position_tolerance_even_while_turning(self):
        controller = RelativeController(pose(), dict(x=.03, y=.03, yaw=1.), .3, .5, 0.)
        command, done = controller.step(pose(), 0.)
        self.assertEqual(command[:2], (0., 0.))
        self.assertGreater(command[2], 0.)
        self.assertFalse(done)
        with self.assertRaises(ValueError):
            RelativeController(pose(), dict(x=1, y=0, yaw=0), .1, .5, 0.)

    def test_wrap_boundary_half_turn_keeps_requested_direction(self):
        for start in (math.pi - .02, -math.pi + .02):
            for direction in (1, -1):
                with self.subTest(start=start, direction=direction):
                    controller = RelativeController(pose(yaw=start),
                        dict(x=0, y=0, yaw=direction * math.pi), .3, .5, 0.)
                    command, _ = controller.step(pose(yaw=start), 0.)
                    self.assertGreater(command[2] * direction, 0)
                    # A wrapped sample on either side of the boundary must not
                    # turn the remaining ~169 degrees into almost a full turn.
                    current = wrap(start + direction * .2)
                    command, _ = controller.step(pose(yaw=current, now=.1), .1)
                    self.assertGreater(command[2] * direction, 0)
                    self.assertAlmostEqual(controller.error['angle_error_deg'], direction * math.degrees(math.pi - .2))
                    current = wrap(start + direction * (math.pi - .01))
                    controller.step(pose(yaw=current, now=.2), .2)
                    self.assertLess(abs(controller.error['angle_error_deg']), 1.)

    def test_half_turn_tie_ignores_small_opposite_startup_drift(self):
        for direction in (1, -1):
            controller = RelativeController(pose(yaw=3.1),
                dict(x=0, y=0, yaw=direction * math.pi), .3, .5, 0.)
            command, _ = controller.step(pose(yaw=wrap(3.1 - direction * .01), now=.1), .1)
            self.assertGreater(command[2] * direction, 0)

    def test_simulated_slip_and_rotation_drift_converge_to_pose(self):
        for angle in (0, math.pi, -math.pi, math.pi / 2):
            with self.subTest(angle=angle):
                initial = pose(2., -1., yaw=2.9, odom_yaw=.6)
                controller = RelativeController(initial, dict(x=.6, y=-.4, yaw=angle), .3, .5, 0.)
                current = dict(initial)
                for step in range(2400):
                    now = step * .05
                    current['odom_stamp'] = current['imu_stamp'] = now
                    (vx, vy, w), done = controller.step(current, now)
                    self.assertLessEqual(math.hypot(vx, vy), .300001)
                    self.assertLessEqual(abs(w), .500001)
                    if done:
                        break
                    heading = .6 + current['yaw'] - initial['yaw']
                    # Unequal axis gains and translation while turning simulate the observed robot.
                    bx, by = .6 * vx + .015 * w, .8 * vy - .01 * w
                    current['x'] += (math.cos(heading) * bx - math.sin(heading) * by) * .05
                    current['y'] += (math.sin(heading) * bx + math.cos(heading) * by) * .05
                    rate = .85 * w - .015 * abs(vx)
                    current['yaw'] += rate * .05
                    current['speed'], current['yaw_rate'] = math.hypot(bx, by), rate
                    current['angular_speed'] = abs(rate)
                self.assertTrue(done)
                self.assertLessEqual(controller.error['position_error_m'], .05)
                self.assertLessEqual(abs(controller.error['angle_error_deg']), 3.)

    def test_pure_rotation_corrects_position_drift(self):
        initial = pose()
        controller = RelativeController(initial, dict(x=0, y=0, yaw=math.pi/2), .3, .5, 0)
        velocity, done = controller.step(pose(x=.1, y=-.1, yaw=.2, now=.1), .1)
        self.assertLess(velocity[0], 0)
        self.assertGreater(velocity[1], 0)
        self.assertGreater(velocity[2], 0)
        self.assertFalse(done)

    def test_frozen_feedback_never_completes_and_no_progress_times_out(self):
        controller = RelativeController(pose(), dict(x=0, y=0, yaw=0), .3, .5, 0)
        for now in (0., .6, 1.):
            self.assertFalse(controller.step(pose(now=0.), now)[1])
        controller = RelativeController(pose(), dict(x=1, y=0, yaw=0), .3, .5, 0)
        controller.step(pose(), 0)
        with self.assertRaisesRegex(RuntimeError, 'progress'):
            controller.step(pose(now=11), 11)

    def test_reset_is_an_error(self):
        safety = MotionSafety(.3, .5)
        safety.check(pose(), 0)
        with self.assertRaises(SafetyViolation) as caught:
            safety.check(pose(x=2, now=.1), .1)
        self.assertEqual(caught.exception.reason, 'position_jump')


class FeedbackBackend:
    def __init__(self):
        self.calls = []
        self.position = pose()
        self.failure = None
        self.sent = threading.Event()
        self.send_ret = 0
        self.arrive = False

    def snapshot(self):
        if self.failure:
            if isinstance(self.failure, Exception):
                raise self.failure
            raise RuntimeError(self.failure)
        return {**self.position, 'odom_stamp': time.monotonic(), 'imu_stamp': time.monotonic()}

    def command(self, *velocity):
        self.calls.append(velocity)
        if any(velocity):
            self.sent.set()
            if self.arrive:
                self.position = pose(x=min(.3, self.position['x'] + .03))
        return self.send_ret


class TestClosedLoopCard(unittest.TestCase):
    def card(self, backend=None):
        backend = backend or FeedbackBackend()
        notifications, done = [], threading.Event()
        def notify(*args, **kwargs):
            notifications.append((args, kwargs))
            done.set()
        return LocoRelativeMovePlugin({}, backend, notify), backend, notifications, done

    def test_validation_schema_and_relative_target_without_calibration(self):
        card, _, _, _ = self.card()
        for axis in ('x', 'y'):
            for value in (0, .3, -.3, 5, -5):
                self.assertEqual(card.plan({axis: value})[axis], value)
            for value in (.29, -.29, 5.01, -5.01, float('nan')):
                self.assertIn('error', card.dispatch('move', {axis: value}))
        for angle in (True, 45, 'bad', float('inf')):
            self.assertIn('error', card.dispatch('move', dict(angle=angle)))
        self.assertAlmostEqual(card.plan(dict(angle='180', direction='right'))['yaw'], -math.pi)
        tool = card.get_tools()[0]
        self.assertEqual(tool['inputSchema']['x-resource'], 'base')
        self.assertIn('正向左', tool['inputSchema']['properties']['y']['title'])

    def test_no_feedback_rejects_before_sending(self):
        card, backend, _, _ = self.card()
        backend.failure = 'stale feedback'
        self.assertIn('error', card.dispatch('move', dict(x=.3)))
        self.assertEqual(backend.calls, [])

    def test_feedback_completion_not_rpc_ack(self):
        card, backend, notices, done = self.card()
        backend.arrive = True
        reply = card.dispatch('move', dict(x=.3))
        self.assertEqual(reply['control'], 'closed_loop')
        self.assertTrue(done.wait(3))
        self.assertEqual(notices[0][0][1], 'completed')
        self.assertTrue(notices[0][0][2]['stop_confirmed_by_feedback'])
        self.assertEqual(backend.calls[-1], (0., 0., 0.))

    def test_interrupt_cancels_and_lifecycle_response_is_idle(self):
        card, backend, notices, done = self.card()
        card.dispatch('move', dict(x=.3, y=.3, angle=90))
        self.assertTrue(backend.sent.wait(1))
        job = card.active
        def dispatch(action, args):
            self.assertTrue(job['cancel'].is_set())
            return {'ret': 0}
        source = Path(__file__).resolve().parents[1] / 'unitree/g1/main.py'
        tree = ast.parse(source.read_text())
        bundle = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'G1DeviceBundle')
        method = next(n for n in bundle.body if isinstance(n, ast.FunctionDef) and n.name == 'dispatch')
        future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
        module = ast.fix_missing_locations(ast.Module(body=[future, method], type_ignores=[]))
        env = {'_lifecycle': SimpleNamespace(LIFECYCLE_ACTIONS=())}
        exec(compile(module, str(source), 'exec'), env)
        peer = SimpleNamespace(dispatch=dispatch, get_tool=lambda: {
            'name': 'smart_motion', 'type': 'actuator', 'inputSchema': {}})
        env['dispatch'](SimpleNamespace(_plugins=[peer], _point_card=card),
                        'smart_motion', {'action': 'interrupt_motion'})
        index = len(backend.calls)
        self.assertTrue(done.wait(3))
        self.assertEqual(notices[0][0][1], 'cancelled')
        self.assertTrue(all(command == (0., 0., 0.) for command in backend.calls[index:]))
        self.assertEqual(card.dispatch('stop', {}), {'state': 'idle'})

    def test_stale_feedback_and_send_failure_report_error(self):
        for failure in ('stale', 'send'):
            with self.subTest(failure=failure):
                card, backend, notices, done = self.card()
                card.dispatch('move', dict(x=.3))
                self.assertTrue(backend.sent.wait(1))
                if failure == 'stale':
                    backend.failure = 'feedback stale'
                else:
                    backend.send_ret = 3102
                self.assertTrue(done.wait(3))
                self.assertEqual(notices[0][0][1], 'error')
                self.assertTrue(card.faulted)
                self.assertEqual(backend.calls[-1], (0., 0., 0.))
                self.assertIn('error', card.dispatch('move', dict(x=.3)))

    def test_expired_goal_sends_stop_and_errors(self):
        card, backend, notices, done = self.card()
        card.MAX_MOTION_SECONDS = .01
        card.dispatch('move', dict(x=.3))
        self.assertTrue(done.wait(3))
        self.assertEqual(notices[0][0][1], 'error')
        self.assertIn('deadline', notices[0][0][2]['error'])

    def test_adapter_uses_no_reply_and_short_command_duration(self):
        calls = []
        client = SimpleNamespace(_CallNoReply=lambda api, payload: calls.append((api, json.loads(payload))) or 0)
        module = 'unitree_sdk2py.g1.loco.g1_loco_api'
        with patch.dict('sys.modules', {module: SimpleNamespace(ROBOT_API_ID_LOCO_SET_VELOCITY=7105)}):
            backend = PointBackend(client, FeedbackBackend())
            self.assertEqual(backend.command(.1, -.2, .3), 0)
        self.assertEqual(calls, [(7105, {'velocity': [.1, -.2, .3], 'duration': .3})])


class TestClosedLoopWiring(unittest.TestCase):
    def test_feedback_subscribes_to_card_topics(self):
        from unitree.g1.relative_feedback import create_feedback_node
        subscriptions = {}
        class Node:
            def __init__(self, name):
                pass
            def create_subscription(self, msg_type, topic, callback, qos):
                subscriptions[topic] = callback
        modules = {
            'rclpy.node': SimpleNamespace(Node=Node),
            'rclpy.qos': SimpleNamespace(QoSProfile=lambda **kwargs: kwargs,
                ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1), HistoryPolicy=SimpleNamespace(KEEP_LAST=1)),
            'std_msgs.msg': SimpleNamespace(String=object),
        }
        feedback = PoseFeedback()
        with patch.dict('sys.modules', modules):
            create_feedback_node('g1', feedback)
        subscriptions['/g1/loco/state'](SimpleNamespace(data=json.dumps(dict(
            position=[1, 2, 0], velocity=[0, 0, 0], imu=dict(rpy=[0, 0, .2])))))
        subscriptions['/g1/state/imu'](SimpleNamespace(data=json.dumps(dict(
            rpy=[0, 0, .4], gyroscope=[0, 0, 0]))))
        self.assertEqual(feedback.snapshot()['x'], 1)
        self.assertEqual(feedback.snapshot()['yaw'], .4)
        subscriptions['/g1/state/imu'](SimpleNamespace(data='invalid json'))
        with self.assertRaises(RuntimeError):
            feedback.snapshot()

    def test_bundle_installs_and_closes_feedback_node_with_bounded_no_reply_client(self):
        from unitree.g1.relative_feedback import PoseFeedback
        source = Path(__file__).resolve().parents[1] / 'unitree/g1/main.py'
        tree = ast.parse(source.read_text())
        bundle = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'G1DeviceBundle')
        init = next(n for n in bundle.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        registration = next(n for n in init.body if isinstance(n, ast.If) and 'loco_relative_move' in ast.unparse(n.test))
        code = compile(ast.Module(body=[registration], type_ignores=[]), str(source), 'exec')
        calls = []
        client = SimpleNamespace(SetTimeout=lambda timeout: calls.append(('timeout', timeout)),
                                 Init=lambda: calls.append(('init',)))
        node = SimpleNamespace(destroy_node=lambda: calls.append(('destroy',)))
        executor = SimpleNamespace(add_node=lambda n: calls.append(('add', n)),
                                   remove_node=lambda n: calls.append(('remove', n)))
        modules = {
            'loco_relative_move': SimpleNamespace(LocoRelativeMovePlugin=LocoRelativeMovePlugin, PointBackend=PointBackend),
            'relative_feedback': SimpleNamespace(PoseFeedback=PoseFeedback, create_feedback_node=lambda ns, fb: node),
            'device': SimpleNamespace(_loco_acp_notify=lambda *args, **kwargs: None),
            'unitree_sdk2py.g1.loco.g1_loco_client': SimpleNamespace(LocoClient=lambda: client),
        }
        instance = SimpleNamespace(_plugins=[])
        cfg = {key: {'enabled': True} for key in ('loco', 'state', 'loco_relative_move')}
        env = dict(self=instance, plugins_cfg=cfg, executor=executor, namespace='g1', print=lambda *args: None)
        with patch.dict('sys.modules', modules):
            exec(code, env)
            self.assertEqual(calls[:3], [('timeout', .05), ('init',), ('add', node)])
            instance._point_card.stop()
            self.assertEqual(calls[-2:], [('remove', node), ('destroy',)])
            cfg['state']['enabled'] = False
            with self.assertRaisesRegex(ValueError, 'publishers'):
                exec(code, env)
            for override in ({}, {'loco_relative_move': {}},
                             {'loco_relative_move': {'enabled': False}}):
                calls.clear()
                fresh = SimpleNamespace(_plugins=[])
                env.update(self=fresh, plugins_cfg=override)
                exec(code, env)
                self.assertEqual(calls, [])
                self.assertEqual(fresh._plugins, [])


class TestLocalMotionSafety(unittest.TestCase):
    def test_stale_skew_invalid_and_jumps_latch(self):
        cases = [
            ('feedback_stale', pose(now=0), .6),
            ('feedback_skew', {**pose(now=.3), 'odom_stamp': 0.}, .3),
            ('feedback_invalid', {**pose(now=.1), 'roll': float('nan')}, .1),
            ('position_jump', pose(x=.3, now=.1), .1),
            ('attitude_jump', pose(yaw=.4, now=.1), .1),
            ('attitude_jump', {**pose(now=.1), 'pitch': .4}, .1),
        ]
        for reason, sample, now in cases:
            with self.subTest(reason=reason):
                safety = MotionSafety(.3, .5)
                safety.check(pose(), 0.)
                with self.assertRaises(SafetyViolation) as caught:
                    safety.check(sample, now)
                self.assertEqual(caught.exception.reason, reason)
                with self.assertRaises(SafetyViolation):
                    safety.check(pose(now=now), now)

    def test_tilt_debounce_and_severe_threshold(self):
        safety = MotionSafety(.3, .5)
        tilted = lambda stamp: {**pose(now=stamp), 'roll': math.radians(26)}
        safety.check(tilted(0), 0)
        safety.check(tilted(.1), .1)
        with self.assertRaises(SafetyViolation) as caught:
            safety.check(tilted(.15), .15)
        self.assertEqual(caught.exception.reason, 'tilt')
        with self.assertRaises(SafetyViolation):
            MotionSafety(.3, .5).check({**pose(), 'pitch': math.radians(35)}, 0)
        # A brief excursion does not accumulate after a below-threshold sample.
        safety = MotionSafety(.3, .5)
        safety.check(tilted(0), 0)
        safety.check({**pose(now=.1), 'roll': math.radians(24)}, .1)
        safety.check(tilted(.2), .2)
        safety.check(tilted(.3), .3)

    def test_overspeed_debounce_uses_distinct_sensor_samples(self):
        for reason, extra in [('linear_overspeed', {'speed': .6}),
                              ('angular_overspeed', {'angular_speed': 1.})]:
            with self.subTest(reason=reason):
                safety = MotionSafety(.3, .5)
                initial = {**pose(), **extra}
                safety.check(initial, 0)
                safety.check(initial, .2)  # Re-reading a held sample cannot prove persistence.
                safety.check({**pose(now=.1), **extra}, .2)
                with self.assertRaises(SafetyViolation) as caught:
                    safety.check({**pose(now=.2), **extra}, .2)
                self.assertEqual(caught.exception.reason, reason)

    def test_normal_motion_and_yaw_boundary_do_not_trigger(self):
        safety = MotionSafety(.3, .5)
        for step in range(20):
            sample = pose(x=.03 * step, yaw=3.1 + .04 * step, now=.1 * step, speed=.3, rate=.4)
            safety.check(sample, .1 * step)
        for config in ({'tilt_stop_deg': -1}, {'tilt_stop_deg': 20}, {'speed_factor': float('nan')}, {'unknown': 1}):
            with self.assertRaises(ValueError):
                MotionSafety(.3, .5, config)

    def test_active_job_stops_latches_and_never_resumes_after_each_safety_event(self):
        cases = [
            ('tilt', {'roll': math.radians(40)}),
            ('position_jump', {'x': 1.}),
            ('attitude_jump', {'yaw': 1.}),
            ('linear_overspeed', {'speed': .7}),
            ('angular_overspeed', {'angular_speed': 1.1}),
            ('feedback_stale', None),
        ]
        for reason, changed in cases:
            with self.subTest(reason=reason):
                backend = FeedbackBackend()
                original_command = backend.command
                def command(*velocity):
                    ret = original_command(*velocity)
                    if not any(velocity):
                        backend.position = pose()  # Simulated plant stops after receiving zero.
                    return ret
                backend.command = command
                done, notices = threading.Event(), []
                def notify(action, status, result, **kwargs):
                    notices.append((status, result))
                    done.set()
                card = LocoRelativeMovePlugin({}, backend, notify)
                card.dispatch('move', dict(x=1, y=1, angle=90))
                self.assertTrue(backend.sent.wait(1))
                # Mutating feedback happens between commands; subsequent commands must all be zero.
                with card.command_lock:
                    boundary = len(backend.calls)
                    if changed is None:
                        backend.failure = SafetyViolation('feedback_stale', 'feedback stale')
                    else:
                        backend.position.update(changed)
                self.assertTrue(done.wait(5))
                self.assertEqual(notices[0][0], 'error')
                self.assertEqual(notices[0][1]['safety_reason'], reason)
                after_event = backend.calls[boundary:]
                stopped_at = next(i for i, command in enumerate(after_event) if not any(command))
                self.assertTrue(all(not any(command) for command in after_event[stopped_at:]))
                self.assertTrue(card.faulted)
                backend.failure = None
                backend.position = pose()
                self.assertIn('error', card.dispatch('move', dict(x=.3)))
                self.assertEqual(card.dispatch('stop_move', {})['state'], 'idle')
                self.assertFalse(card.faulted)
