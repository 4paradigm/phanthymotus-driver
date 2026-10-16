"""Closed-loop relative movement and existing locomotion regressions (no hardware)."""
import ast
import tempfile
from pathlib import Path
import time
import json
import types
from types import SimpleNamespace
from unittest.mock import patch
import math
import threading
import unittest
from unitree.g1.loco_relative_move import LocoRelativeMovePlugin, PointBackend, MotionTrace
from unitree.g1.relative_feedback import PoseFeedback, RelativeController, MotionSafety, SafetyViolation, wrap

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


def pose(x=0., y=0., yaw=0., odom_yaw=0., now=0., speed=0., rate=0.):
    return dict(x=x, y=y, yaw=yaw, roll=0., pitch=0., angular_speed=abs(rate), odom_yaw=odom_yaw, speed=speed, yaw_rate=rate,
                odom_stamp=now)


def odom_data(yaw=0., rate=0., x=0., y=0.):
    return dict(position=[x, y, 0], velocity=[0, 0, 0], yaw_speed=rate,
                imu=dict(rpy=[0, 0, yaw], gyroscope=[0, 0, rate]))


class TestPoseFeedback(unittest.TestCase):
    def test_invalid_odom_recovery_invalidates_old_task(self):
        now = [10.]
        feedback = PoseFeedback(clock=lambda: now[0])
        feedback.update_odom(odom_data(3.1))
        feedback.update_odom(odom_data(-3.1))
        initial = feedback.snapshot()
        self.assertGreater(initial['yaw'], math.pi)
        safety = MotionSafety(.3, .5)
        safety.check(initial, now[0])
        feedback.update_odom({})
        self.assertIsNone(feedback.odom)
        self.assertIsNone(feedback.last_yaw)
        feedback.update_odom(odom_data(-1.))
        recovered = feedback.snapshot()
        self.assertAlmostEqual(recovered['yaw'], -1.)
        with self.assertRaises(SafetyViolation) as caught:
            safety.check(recovered, now[0])
        self.assertEqual(caught.exception.reason, 'odom_reset')
        MotionSafety(.3, .5).check(recovered, now[0])

    def test_valid_odom_after_gap_also_rebaselines(self):
        now = [0.]
        feedback = PoseFeedback(clock=lambda: now[0])
        feedback.update_odom(odom_data(3.))
        now[0] = 1.
        feedback.update_odom(odom_data(-2.))
        self.assertEqual(feedback.odom_generation, 1)
        self.assertAlmostEqual(feedback.snapshot()['yaw'], -2.)

    def test_odometry_alone_supplies_heading_rate_and_freshness(self):
        now = [10.]
        feedback = PoseFeedback(clock=lambda: now[0])
        with self.assertRaises(RuntimeError):
            feedback.snapshot()
        feedback.update_odom(odom_data(3.1, .4))
        now[0] += .1
        feedback.update_odom(odom_data(-3.1, .4))
        sample = feedback.snapshot()
        self.assertGreater(sample['yaw'], math.pi)
        self.assertEqual(sample['yaw_rate'], .4)
        self.assertEqual(sample['angular_speed'], .4)
        self.assertNotIn('imu_stamp', sample)
        now[0] += .6
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            feedback.snapshot()
        for bad in (dict(yaw_speed=float('nan')), dict(imu={}), dict(position=[float('nan'), 0])):
            feedback.update_odom({**odom_data(), **bad})
            with self.assertRaises(RuntimeError):
                feedback.snapshot()


class TestRelativeController(unittest.TestCase):
    def test_rotation_is_not_interrupted_by_small_reported_linear_velocity(self):
        for direction in (1, -1):
            controller = RelativeController(pose(), dict(x=0, y=0, yaw=direction * math.pi/2), .3, .5, 0.)
            safety = MotionSafety(.3, .5)
            for step, speed in enumerate((0., .04, .08, .02, .06)):
                now = step * .1
                sample = pose(yaw=direction * step * .04, speed=speed, rate=direction * .4, now=now)
                safety.check(sample, now)
                command, done = controller.step(sample, now)
                self.assertEqual(command[:2], (0., 0.))
                self.assertGreater(command[2] * direction, 0.)
                self.assertFalse(done)
            # Reaching the angle must still stop; drift must not qualify as settled.
            sample = pose(yaw=direction * math.pi/2, speed=.06, now=1.)
            self.assertEqual(controller.step(sample, 1.), ((0., 0., 0.), False))
            self.assertEqual(controller.phase, 'rotate')

    def test_rotation_floor_handles_both_directions_and_stops_within_tolerance(self):
        for direction in (1, -1):
            for remaining in (6.2, 12.75, 90):
                with self.subTest(direction=direction, remaining=remaining):
                    controller = RelativeController(pose(),
                        dict(x=0, y=0, yaw=direction * math.pi/2), .3, .5, 0.)
                    current = direction * math.radians(90 - remaining)
                    command, _ = controller.step(pose(yaw=current), 0.)
                    self.assertEqual(command[:2], (0., 0.))
                    self.assertGreaterEqual(abs(command[2]), .3)
                    self.assertLessEqual(abs(command[2]), .5)
                    self.assertGreater(command[2] * direction, 0)
                    if remaining < 13:
                        self.assertAlmostEqual(abs(command[2]), .3)
                    command, done = controller.step(pose(yaw=direction * math.radians(88), now=.1), .1)
                    self.assertEqual(command, (0., 0., 0.))
                    self.assertFalse(done)
        for value in (0, -.1, .6, float('nan')):
            with self.assertRaises(ValueError):
                RelativeController(pose(), dict(x=0, y=0, yaw=1), .3, .5, 0., min_yaw_speed=value)

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
        self.assertEqual(command[2], 0.)
        self.assertFalse(done)
        controller.step(pose(now=.6), .6)
        self.assertGreater(controller.step(pose(now=.7), .7)[0][2], 0.)
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
                    current['odom_stamp'] = now
                    (vx, vy, w), done = controller.step(current, now)
                    self.assertFalse((vx or vy) and w, 'Rotation and translation must never be commanded together')
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
        self.assertEqual(velocity[:2], (0., 0.))
        self.assertGreater(velocity[2], 0)
        self.assertFalse(done)
        for now in (.2, .8):
            velocity, done = controller.step(pose(x=.1, y=-.1, yaw=math.pi/2, now=now), now)
            self.assertEqual(velocity, (0., 0., 0.))
            self.assertFalse(done)
        velocity, _ = controller.step(pose(x=.1, y=-.1, yaw=math.pi/2, now=.9), .9)
        # In the rotated body frame both corrections are positive.
        self.assertGreater(velocity[0], 0)
        self.assertGreater(velocity[1], 0)
        self.assertEqual(velocity[2], 0.)
        self.assertFalse(controller.step(pose(yaw=math.pi/2, now=1.), 1.)[1])
        self.assertTrue(controller.step(pose(yaw=math.pi/2, now=1.6), 1.6)[1])

    def test_nonzero_goal_translates_then_turns_and_corrects_drift_separately(self):
        controller = RelativeController(pose(), dict(x=1, y=1, yaw=math.pi/2), .3, .5, 0)
        command, _ = controller.step(pose(), 0.)
        self.assertTrue(command[0] or command[1])
        self.assertEqual(command[2], 0.)
        for now in (.1, .7):
            command, done = controller.step(pose(x=1, y=1, now=now), now)
            self.assertEqual(command, (0., 0., 0.))
            self.assertFalse(done)
        command, _ = controller.step(pose(x=1, y=1, now=.8), .8)
        self.assertEqual(command[:2], (0., 0.))
        self.assertGreater(command[2], 0.)
        for now in (.9, 1.5):
            command, _ = controller.step(pose(x=1.1, y=.9, yaw=math.pi/2, now=now), now)
            self.assertEqual(command, (0., 0., 0.))
        command, _ = controller.step(pose(x=1.1, y=.9, yaw=math.pi/2, now=1.6), 1.6)
        self.assertTrue(command[0] or command[1])
        self.assertEqual(command[2], 0.)
        drifting = pose(x=1.1, y=.9, yaw=math.pi/2 + .1, now=1.7, speed=.2, rate=.1)
        command, done = controller.step(drifting, 1.7)
        self.assertTrue(command[0] or command[1])
        self.assertEqual(command[2], 0.)
        self.assertFalse(done)
        self.assertEqual(controller.phase, 'translate')
        for now in (1.8, 2.4):
            command, done = controller.step(pose(x=1, y=1, yaw=math.pi/2 + .1, now=now), now)
            self.assertEqual(command, (0., 0., 0.))
            self.assertFalse(done)
        command, _ = controller.step(pose(x=1, y=1, yaw=math.pi/2 + .1, now=2.5), 2.5)
        self.assertEqual(command[:2], (0., 0.))
        self.assertLess(command[2], 0.)

    def test_straight_goal_defers_heading_correction_until_endpoint_and_stop(self):
        for turn in (0., math.pi / 2):
            controller = RelativeController(pose(), dict(x=1, y=0, yaw=turn), .3, .5, 0)
            for i, x in enumerate((0., .3, .6, .9)):
                sample = pose(x=x, yaw=math.radians(15), now=i*.1, speed=.2)
                command, done = controller.step(sample, i*.1)
                self.assertGreater(command[0], 0.)
                self.assertEqual(command[2], 0.)
                self.assertFalse(done)
                self.assertEqual(controller.phase, 'translate')
                self.assertEqual((controller.x, controller.y), (1., 0.))
            # At the endpoint residual movement must not trigger rotation yet.
            command, _ = controller.step(pose(x=1, yaw=math.radians(15), speed=.1, now=.5), .5)
            self.assertEqual(command, (0., 0., 0.))
            self.assertEqual(controller.phase, 'translate')
            for now in (.6, 1.2):
                command, done = controller.step(pose(x=1, yaw=math.radians(15), now=now), now)
                self.assertEqual(command, (0., 0., 0.))
                self.assertFalse(done)
            self.assertEqual(controller.phase, 'rotate')
            command, _ = controller.step(pose(x=1, yaw=math.radians(15), now=1.3), 1.3)
            self.assertEqual(command[:2], (0., 0.))
            self.assertGreater(command[2] * (1 if turn else -1), 0.)

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
        return {**self.position, 'odom_stamp': time.monotonic()}

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

    def test_local_diagnostics_preserve_full_history_and_compact_callback(self):
        card, _, _, _ = self.card()
        trace = MotionTrace(pose(), {}, {})
        for i in range(450):
            trace.add('control', index=i)
        with tempfile.TemporaryDirectory() as directory:
            card.diagnostics_dir = Path(directory)
            original = dict(error='No progress in rotate phase', motion_trace=trace.export(full=True),
                            stop_confirmed_by_feedback=False)
            with patch('builtins.print'):
                summary = card._save_diagnostics('g1_point_test', 'error', original)
            self.assertLess(len(json.dumps(summary, ensure_ascii=True)), 600)
            saved = json.loads(Path(summary['diagnostics_file']).read_text())
            self.assertEqual(saved['result'], original)
            self.assertEqual(len(saved['result']['motion_trace']['records']), 450)
            self.assertEqual(saved['result']['motion_trace']['dropped_records'], 0)
            self.assertFalse(list(Path(directory).glob('*.tmp')))

    def test_diagnostics_disk_failure_has_reconstructable_log_fallback(self):
        card, _, _, _ = self.card()
        original = dict(error='test', data='abc' * 1000)
        with patch.object(Path, 'mkdir', side_effect=OSError('disk full')), patch('builtins.print') as output:
            summary = card._save_diagnostics('g1_point_test', 'error', original)
        self.assertIn('diagnostics_error', summary)
        self.assertLess(len(json.dumps(summary, ensure_ascii=True)), 600)
        chunks = [call.args[0].split(' ', 3)[3] for call in output.call_args_list
                  if call.args[0].startswith('[RelativeMotionData]')]
        self.assertEqual(json.loads(''.join(chunks))['result'], original)

    def test_rotation_completes_with_only_odometry_samples(self):
        # Exercise actual cache + safety + controller + stop without an IMU callback.
        feedback = PoseFeedback()
        feedback.update_odom(odom_data())
        class OdomBackend:
            COMMAND_DURATION = .3
            def __init__(self):
                self.yaw = 0.
                self.calls = []
            def snapshot(self):
                return feedback.snapshot()
            def command(self, vx, vy, rate):
                self.calls.append((vx, vy, rate))
                self.yaw += rate * .05
                feedback.update_odom(odom_data(wrap(self.yaw), rate))
                return 0
        backend = OdomBackend()
        card, _, notices, done = self.card(backend)
        card.dispatch('move', dict(angle=30))
        self.assertTrue(done.wait(5))
        self.assertEqual(notices[0][0][1], 'completed')
        self.assertTrue(notices[0][0][2]['stop_confirmed_by_feedback'])
        self.assertLessEqual(abs(backend.yaw - math.pi / 6), math.radians(3))
        self.assertEqual(backend.calls[-1], (0., 0., 0.))

    def test_trace_is_bounded_serializable_and_marks_missing_source_time(self):
        trace = MotionTrace(pose(), {'yaw': 1.}, {})
        for i in range(450):
            trace.add('control', sequence=i, invalid=float('nan'))
        result = trace.export()
        self.assertEqual(result['record_count'], 450)
        self.assertEqual(result['dropped_records'], 50)
        self.assertEqual(len(result['records']), 400)
        self.assertEqual(result['records'][0]['sequence'], 50)
        self.assertIsNone(result['records'][0]['invalid'])
        self.assertIn('source_capture_time_unavailable', result['time_basis'])
        json.dumps(result, allow_nan=False)

    def test_no_progress_trace_contains_current_feedback_and_stop_attempts(self):
        card, backend, notices, done = self.card()
        def fail_step(controller, sample, now):
            controller.update_error(sample)
            raise RuntimeError('No progress in rotate phase')
        with patch.object(RelativeController, 'step', fail_step):
            card.dispatch('move', dict(angle=60))
            self.assertTrue(done.wait(3))
        args, _ = notices[0]
        self.assertEqual(args[1], 'error')
        result = args[2]
        self.assertLess(len(json.dumps(result, ensure_ascii=True)), 600)
        result = json.loads(Path(result['diagnostics_file']).read_text())['result']
        trace = result['motion_trace']
        events = [r['event'] for r in trace['records']]
        for event in ('start', 'control', 'exception', 'stop_send', 'stop_feedback', 'terminal'):
            self.assertIn(event, events)
        control = next(r for r in trace['records'] if r['event'] == 'control')
        self.assertIn('odom_age_s', control)
        self.assertIn('pose', control)
        self.assertIn('error', control)
        self.assertIn('exception', control)
        self.assertNotIn('command', control)  # Failure occurred before any velocity send.
        self.assertAlmostEqual(result['angle_error_deg'], 60.)
        self.assertEqual(trace['records'][-1]['status'], 'error')
        json.dumps(result, allow_nan=False)

    def test_heading_jump_callback_reports_trigger_error_and_last_command(self):
        card, backend, notices, done = self.card()
        original_command = backend.command
        def command(*velocity):
            ret = original_command(*velocity)
            if any(velocity):
                backend.position.update(yaw=1., raw_yaw=1., yaw_rate=.4)
            return ret
        backend.command = command
        with patch.object(card, '_confirm_stop', return_value=True):
            card.dispatch('move', dict(angle=180))
            self.assertTrue(done.wait(2))
        args, _ = notices[0]
        self.assertEqual(args[1], 'error')
        result = args[2]
        self.assertEqual(result['safety_reason'], 'attitude_jump')
        result = json.loads(Path(result['diagnostics_file']).read_text())['result']
        self.assertAlmostEqual(result['angle_error_deg'], 180 - math.degrees(1.))
        self.assertEqual(result['error_sample']['basis'], 'safety_trigger_feedback_not_final_pose')
        control = [r for r in result['motion_trace']['records'] if r['event'] == 'control']
        self.assertEqual(control[0]['command'], [0., 0., .5])
        self.assertEqual(control[0]['send_ret'], 0)
        self.assertIn('send_elapsed_s', control[0])
        self.assertIn('exception', control[-1])
        jump = result['odom_heading_jump']
        self.assertAlmostEqual(jump['actual_change_deg'], math.degrees(1.))
        self.assertGreater(jump['actual_change_deg'], jump['allowed_change_deg'])
        self.assertEqual(jump['commanded_vyaw_rad_s'], .5)
        self.assertEqual(jump['yaw_speed_rad_s'], .4)
        json.dumps(result, allow_nan=False)

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

    def test_unknown_action_is_declined_without_sending_motion(self):
        card, backend, notices, _ = self.card()
        self.assertIsNone(card.dispatch('unsupported_action', {}))
        self.assertEqual(backend.calls, [])
        self.assertEqual(notices, [])
        self.assertFalse(card.busy)

    def test_no_feedback_rejects_before_sending(self):
        card, backend, _, _ = self.card()
        backend.failure = 'stale feedback'
        self.assertIn('error', card.dispatch('move', dict(x=.3)))
        self.assertEqual(backend.calls, [])

    def test_zero_target_returns_noop_without_feedback_or_completion(self):
        card, backend, notices, _ = self.card()
        backend.failure = 'no feedback'
        card.can_start = lambda: self.fail('No-op must not query ownership')
        for args in ({}, dict(x=0, y=0, angle=0), dict(x='0', y='0', angle='0')):
            reply = card.dispatch('move', args)
            self.assertEqual(reply, {'ret': 0, 'state': 'idle', 'noop': True})
            self.assertNotIn('action_id', reply)
        self.assertIsNone(card.active)
        self.assertEqual(backend.calls, [])
        self.assertEqual(notices, [])

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
            position=[1, 2, 0], velocity=[0, 0, 0], yaw_speed=0., imu=dict(rpy=[0, 0, .2])))))
        self.assertEqual(list(subscriptions), ['/g1/loco/state'])
        self.assertEqual(feedback.snapshot()['x'], 1)
        self.assertEqual(feedback.snapshot()['yaw'], .2)
        subscriptions['/g1/loco/state'](SimpleNamespace(data='invalid json'))
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
        env = dict(self=instance, plugins_cfg=cfg, executor=executor, namespace='g1', print=lambda *args: None,
                   smart_motion=SimpleNamespace(get_state=lambda: {'state': 'idle', 'base_available': False}))
        with patch.dict('sys.modules', modules):
            exec(code, env)
            self.assertEqual(calls[:3], [('timeout', .05), ('init',), ('add', node)])
            self.assertFalse(instance._point_card.can_start()['available'])
            env['smart_motion'] = SimpleNamespace(get_state=lambda: {'state': 'idle', 'base_available': True})
            self.assertTrue(instance._point_card.can_start()['available'])
            env['smart_motion'] = None
            env['loco_client'] = SimpleNamespace(MotionIdle=lambda: {'available': False})
            self.assertFalse(instance._point_card.can_start()['available'])
            instance._point_card.stop()
            self.assertEqual(calls[-2:], [('remove', node), ('destroy',)])
            cfg['state']['enabled'] = False
            exec(code, env)  # Independent IMU publisher is no longer required.
            cfg['loco']['enabled'] = False
            with self.assertRaisesRegex(ValueError, 'publisher'):
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

    def test_raw_yaw_wrap_is_continuous_and_jump_details_use_receive_interval(self):
        now = [1.]
        feedback = PoseFeedback(clock=lambda: now[0])
        def update(yaw):
            feedback.update_odom(odom_data(yaw, .5))
            return feedback.snapshot()
        safety = MotionSafety(.3, .5)
        safety.check(update(3.13), now[0])
        now[0] += .05
        sample = update(-3.13)
        self.assertEqual(sample['raw_yaw'], -3.13)
        self.assertGreater(sample['yaw'], math.pi)
        safety.check(sample, now[0])
        now[0] += .05
        with self.assertRaises(SafetyViolation) as caught:
            safety.check(update(-2.5), now[0])
        detail = caught.exception.details['odom_heading_jump']
        self.assertAlmostEqual(detail['sample_interval_s'], .05)
        self.assertAlmostEqual(detail['actual_change_deg'], math.degrees(.63))
        self.assertAlmostEqual(detail['allowed_change_deg'], 10 + math.degrees(.95 * .05))
        self.assertAlmostEqual(detail['previous_raw_yaw_deg'], math.degrees(-3.13))
        self.assertEqual(detail['time_basis'], 'local_callback_receive')

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
            ('odom_reset', {'odom_generation': 1}),
            ('linear_overspeed', {'speed': .7}),
            ('angular_overspeed', {'angular_speed': 1.1}),
            ('feedback_stale', None),
        ]
        for reason, changed in cases:
            with self.subTest(reason=reason):
                backend = FeedbackBackend()
                original_command = backend.command
                cancelled_stop = []
                def command(*velocity):
                    ret = original_command(*velocity)
                    if not any(velocity) and card.active and card.active['cancel'].is_set():
                        if not cancelled_stop:
                            cancelled_stop.append(len(backend.calls) - 1)
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
                self.assertTrue(cancelled_stop)
                self.assertTrue(all(not any(command) for command in backend.calls[cancelled_stop[0]:]))
                self.assertTrue(card.faulted)
                backend.failure = None
                backend.position = pose()
                self.assertIn('error', card.dispatch('move', dict(x=.3)))
                self.assertEqual(card.dispatch('stop_move', {})['state'], 'idle')
                self.assertFalse(card.faulted)


class TestBaseOwnership(unittest.TestCase):
    def test_existing_loco_timed_and_continuous_motion_block_direct_goal(self):
        from test_g1_loco_timing import World, ROOT, extract_class
        from test_g1_loco_contract import card_namespace
        for duration in (0, 1):
            with self.subTest(duration=duration):
                world = World()
                namespace = card_namespace()
                original_class = extract_class(ROOT / 'device.py', 'LocoPlugin', namespace)
                original = original_class.__new__(original_class)
                original._smart_motion = None
                original._fallback_motion = world.controller
                original._client = original._slam_client = None
                backend = FeedbackBackend()
                done = threading.Event()
                card = LocoRelativeMovePlugin({}, backend, lambda *args, **kwargs: done.set(),
                    can_start=lambda: {'available': world.controller.is_idle()})
                original.dispatch('move', dict(vx=.3, duration=duration))
                self.assertIn('error', card.dispatch('move', dict(x=.3)))
                self.assertEqual(backend.calls, [])
                self.assertIsNone(card.active)
                original.dispatch('stop_move', {})
                backend.arrive = True
                self.assertEqual(card.dispatch('move', dict(x=.3))['ret'], 0)
                self.assertTrue(done.wait(3))
                self.assertTrue(backend.calls)

    def test_ownership_timeout_or_failed_stop_does_not_start_direct_writes(self):
        for state in ({'available': False}, {'error': 'timeout'}, None):
            backend = FeedbackBackend()
            card = LocoRelativeMovePlugin({}, backend, lambda *args, **kwargs: None, can_start=lambda: state)
            self.assertIn('error', card.dispatch('move', dict(x=.3)))
            self.assertEqual(backend.calls, [])

    def test_pending_stop_ack_and_updates_do_not_release_owner(self):
        from test_g1_loco_timing import World
        world = World()
        self.assertTrue(world.controller.is_idle())
        world.start()
        self.assertFalse(world.controller.is_idle())
        world.controller.stop()
        self.assertTrue(world.controller.is_idle())
        with world.controller.lock:
            world.controller.active['updates_pending'] = 1
        self.assertFalse(world.controller.is_idle())
        with world.controller.lock:
            world.controller.active['updates_pending'] = 0
            world.controller.active['stop_done'] = False
        self.assertFalse(world.controller.is_idle())

    def test_base_submission_and_relative_start_are_serialized(self):
        backend = FeedbackBackend()
        entered, release = threading.Event(), threading.Event()
        owner = {'available': True}
        card = LocoRelativeMovePlugin({}, backend, lambda *args, **kwargs: None,
                                      can_start=lambda: dict(owner))
        def legacy_move(action, args):
            entered.set()
            release.wait(2)
            owner['available'] = False
            return {'ret': 0}
        legacy = threading.Thread(target=lambda: card.dispatch_base(SimpleNamespace(dispatch=legacy_move), 'move', {}))
        legacy.start()
        self.assertTrue(entered.wait(1))
        result = []
        direct = threading.Thread(target=lambda: result.append(card.dispatch('move', dict(x=.3))))
        direct.start()
        self.assertEqual(backend.calls, [])
        release.set()
        legacy.join(2)
        direct.join(2)
        self.assertEqual(len(result), 1)
        self.assertIn('error', result[0])
        self.assertEqual(backend.calls, [])

    def test_safety_owner_reports_unavailable_until_stop_repeats_are_drained(self):
        from test_g1_loco_timing import World
        path = Path(__file__).resolve().parents[1] / 'unitree/g1/safety_harness.py'
        tree = ast.parse(path.read_text())
        worker = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == '_run_smart_motion_process')
        method = next(n for n in worker.body if isinstance(n, ast.FunctionDef) and n.name == 'handle_get_state')
        idle = SimpleNamespace(value='idle')
        world = World()
        namespace = dict(state=idle, MotionState=SimpleNamespace(IDLE=idle, MOVING=object(),
            NAVIGATING=object(), NAV_PAUSED=object()), speed_zone=SimpleNamespace(value='normal'),
            current_cmd=None, nav_cmd=None, stop_repeat_count=0, motion=world.controller,
            obstacle_lock=threading.Lock(), obstacle_dist=float('inf'), lateral_obstacle=False)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        get_state = namespace['handle_get_state']
        self.assertTrue(get_state()['base_available'])
        world.start()
        self.assertFalse(get_state()['base_available'])
        world.controller.stop()
        namespace['stop_repeat_count'] = 3
        self.assertFalse(get_state()['base_available'])
        namespace['stop_repeat_count'] = 0
        self.assertTrue(get_state()['base_available'])


class TestCancelledHandoff(unittest.TestCase):
    def test_peer_move_waits_for_old_worker_and_receives_no_late_zero(self):
        backend = FeedbackBackend()
        finished = threading.Event()
        card = LocoRelativeMovePlugin({}, backend, lambda *args, **kwargs: finished.set())
        confirming, release = threading.Event(), threading.Event()
        def confirm():
            confirming.set()
            if not release.wait(2):
                return False
            with card.command_lock:
                backend.command(0., 0., 0.)
            return True
        card._confirm_stop = confirm
        card.dispatch('move', dict(x=1))
        self.assertTrue(backend.sent.wait(1))
        job = card.active
        results = []
        def peer_dispatch(action, args):
            self.assertTrue(job['motion_done'].is_set())
            self.assertTrue(job['stop_confirmed'])
            if action == 'move':
                backend.calls.append('peer_move')
            return {'ret': 0}
        peer = SimpleNamespace(dispatch=peer_dispatch)
        interrupt = threading.Thread(target=lambda: results.append(card.dispatch_base(peer, 'interrupt_motion', {})))
        interrupt.start()
        self.assertTrue(confirming.wait(1))
        requested = threading.Event()
        def move():
            requested.set()
            results.append(card.dispatch_base(peer, 'move', {}))
        next_move = threading.Thread(target=move)
        next_move.start()
        self.assertTrue(requested.wait(1))
        self.assertEqual(results, [])
        self.assertNotIn('peer_move', backend.calls)
        release.set()
        interrupt.join(2)
        next_move.join(2)
        job['worker'].join(2)
        self.assertFalse(interrupt.is_alive())
        self.assertFalse(next_move.is_alive())
        self.assertEqual([reply['ret'] for reply in results], [0, 0])
        index = backend.calls.index('peer_move')
        self.assertEqual(backend.calls[index + 1:], [])
        self.assertTrue(finished.is_set())

    def test_unconfirmed_stop_rejects_following_motion(self):
        backend = FeedbackBackend()
        card = LocoRelativeMovePlugin({}, backend, lambda *args, **kwargs: None)
        card._confirm_stop = lambda: False
        card.dispatch('move', dict(x=1))
        self.assertTrue(backend.sent.wait(1))
        peer_calls = []
        peer = SimpleNamespace(dispatch=lambda action, args: peer_calls.append(action) or {'ret': 0})
        stopped = card.dispatch_base(peer, 'interrupt_motion', {})
        self.assertNotEqual(stopped['ret'], 0)
        self.assertTrue(card.faulted)
        self.assertIn('error', card.dispatch_base(peer, 'move', {}))
        self.assertEqual(peer_calls, ['interrupt_motion'])

    def test_handoff_timeout_does_not_release_running_worker(self):
        backend = FeedbackBackend()
        card = LocoRelativeMovePlugin({}, backend, lambda *args, **kwargs: None)
        card.HANDOFF_TIMEOUT = .05
        release = threading.Event()
        card._confirm_stop = lambda: release.wait(2)
        card.dispatch('move', dict(x=1))
        self.assertTrue(backend.sent.wait(1))
        job = card.active
        try:
            result = card.dispatch('stop_move', {})
            self.assertNotEqual(result['ret'], 0)
            self.assertFalse(job['motion_done'].is_set())
            peer = SimpleNamespace(dispatch=lambda *args: self.fail('Unconfirmed handoff must reject peer motion'))
            self.assertIn('error', card.dispatch_base(peer, 'move', {}))
        finally:
            release.set()
            job['worker'].join(2)
