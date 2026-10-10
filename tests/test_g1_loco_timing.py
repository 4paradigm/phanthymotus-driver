"""Offline motion regression tests; no ROS, DDS, or robot connection."""
import ast
import contextlib
import io
import json
import math
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from unitree.g1.timed_motion import TimedMotion

ROOT = Path(__file__).resolve().parents[1] / 'unitree/g1'


def extract_class(path, name, namespace):
    source = path.read_text()
    node = next(n for n in ast.parse(source).body
                if isinstance(n, ast.ClassDef) and n.name == name)
    exec("from __future__ import annotations\n" + ast.get_source_segment(source, node), namespace)
    return namespace[name]


class World:
    def __init__(self):
        self.now = 0.
        self.timers = []
        self.calls = []
        self.move_ret = 0
        self.stop_ret = 0
        self.move_hook = lambda: None
        self.stop_hook = lambda: None
        self.stop_send_hook = lambda: None
        self.stops = []
        world = self

        class Timer:
            def __init__(self, duration, callback, args=()):
                self.duration, self.callback, self.args = duration, callback, args
                self.cancelled = False
            def start(self):
                self.deadline = world.now + self.duration
                world.timers.append(self)
            def cancel(self):
                self.cancelled = True
            def fire(self, lag=0):
                world.now = max(world.now, self.deadline + lag)
                self.callback(*self.args)

        class Client:
            def BeginMove(self, *args):
                world.calls.append(('move_sent', world.now, args))
                def wait():
                    world.move_hook()
                    return world.move_ret
                return wait
            def BeginStopMove(self):
                world.calls.append(('stop_sent', world.now, ()))
                world.stop_send_hook()
                def wait():
                    world.stop_hook()
                    return world.stop_ret
                return wait

        self.controller = TimedMotion(Client(), clock=lambda: self.now,
                                      timer_factory=Timer, on_stop=self.stops.append)

    def start(self, action_id='a', duration=1):
        return self.controller.start(0, 0, 1, duration, action_id)


class TestTimedMotion(unittest.TestCase):
    def test_stop_sent_at_deadline_during_slow_move_response(self):
        w = World()
        def slow_response():
            w.timers[0].fire()
            w.now = 4.2
        w.move_hook = slow_response
        result = w.start()
        self.assertEqual([c[:2] for c in w.calls], [('move_sent', 0.), ('stop_sent', 1.)])
        self.assertEqual(result['ret'], 0)
        self.assertEqual(result['state'], 'idle')
        self.assertEqual(w.controller.get_result('a')['status'], 'completed')

    def test_no_completion_before_stop_response(self):
        w = World()
        w.start()
        def slow_stop():
            self.assertEqual(w.controller.get_result('a')['status'], 'pending')
            self.assertEqual(w.stops, [])
            w.now += 5.14
        w.stop_hook = slow_stop
        w.timers[0].fire()
        self.assertEqual(w.calls[1][1], 1.)
        self.assertEqual(w.now, 6.14)
        self.assertEqual(w.controller.get_result('a')['status'], 'completed')

    def test_stop_failure_is_not_completed(self):
        w = World()
        w.stop_ret = 3104
        w.start()
        w.timers[0].fire()
        result = w.controller.get_result('a')
        self.assertEqual(result['status'], 'error')
        self.assertEqual(result['result']['stop_ret'], 3104)
        self.assertFalse(result['result']['stop_acknowledged'])
        self.assertEqual(w.controller.stop()['state'], 'stop_failed')

    def test_move_failure_sends_stop_and_preserves_code(self):
        w = World()
        w.move_ret = 3104
        self.assertEqual(w.start()['ret'], 3104)
        self.assertEqual([c[0] for c in w.calls], ['move_sent', 'stop_sent'])
        self.assertTrue(w.timers[0].cancelled)
        result = w.controller.get_result('a')
        self.assertEqual(result['status'], 'error')
        self.assertEqual(result['result']['move_ret'], 3104)

    def test_move_exception_still_stops(self):
        w = World()
        def fail():
            raise RuntimeError('fake SDK timeout')
        w.move_hook = fail
        self.assertIn('error', w.start())
        self.assertEqual(w.calls[-1][0], 'stop_sent')
        self.assertEqual(w.controller.get_result('a')['status'], 'error')

    def test_stop_send_exception_is_not_success(self):
        w = World()
        w.start()
        def fail():
            raise RuntimeError('fake send error')
        w.stop_send_hook = fail
        result = w.controller.stop()
        self.assertEqual(result['state'], 'stop_failed')
        self.assertIsNone(result['ret'])
        self.assertEqual(w.controller.get_result('a')['status'], 'error')

    def test_old_timer_cannot_stop_replacement(self):
        w = World()
        w.start('a')
        old_timer = w.timers[0]
        w.now = .2
        w.start('b')
        old_timer.fire()  # even a cancelled callback already running is harmless
        self.assertEqual([c[0] for c in w.calls], ['move_sent', 'move_sent'])
        self.assertEqual(w.controller.get_result('a')['status'], 'cancelled')
        self.assertEqual(w.controller.get_result('b')['status'], 'pending')
        w.timers[-1].fire()
        self.assertEqual(w.controller.get_result('b')['status'], 'completed')

    def test_stop_reply_for_old_action_cannot_reset_new_action(self):
        w = World()
        w.start('a')
        def replacement():
            w.stop_hook = lambda: None
            w.start('b')
        w.stop_hook = replacement
        w.timers[0].fire()
        self.assertEqual(w.stops, [])
        self.assertEqual(w.controller.active['id'], 'b')
        self.assertEqual(w.controller.get_result('b')['status'], 'pending')
        self.assertEqual([c[0] for c in w.calls], ['move_sent', 'stop_sent', 'move_sent'])

    def test_speed_update_keeps_deadline_and_cannot_revive_expired_action(self):
        w = World()
        w.start()
        w.now = .5
        self.assertEqual(w.controller.update('a', 0, 0, .2)['ret'], 0)
        self.assertEqual(w.timers[0].deadline, 1.)
        w.now = 1.1
        w.controller.update('a', 0, 0, 1)
        self.assertEqual([c[0] for c in w.calls], ['move_sent', 'move_sent', 'stop_sent'])
        w.controller.update('a', 0, 0, 1)
        self.assertEqual(len(w.calls), 3)

    def test_failed_speed_update_reports_error(self):
        w = World()
        w.start()
        w.move_ret = 3104
        self.assertEqual(w.controller.update('a', 0, 0, .2)['ret'], 3104)
        self.assertEqual(w.controller.get_result('a')['status'], 'error')

    def test_completion_waits_for_inflight_speed_update(self):
        w = World()
        w.start()
        def delayed_update():
            w.timers[0].fire()
            self.assertEqual(w.controller.get_result('a')['status'], 'pending')
            w.move_ret = 3104
        w.move_hook = delayed_update
        w.controller.update('a', 0, 0, .2)
        outcome = w.controller.get_result('a')
        self.assertEqual(outcome['status'], 'error')
        self.assertEqual(outcome['result']['update_ret'], 3104)

    def test_continuous_motion_requires_explicit_stop(self):
        w = World()
        w.start(duration=0)
        self.assertEqual(w.timers, [])
        w.controller.stop()
        self.assertEqual(w.controller.get_result('a')['status'], 'cancelled')

    def test_nonfinite_input_never_sends(self):
        for value in [math.nan, math.inf, -math.inf]:
            w = World()
            self.assertIn('error', w.controller.start(0, 0, value, 1))
            self.assertEqual(w.calls, [])

    def test_explicit_stop_can_retry_failure(self):
        w = World()
        w.stop_ret = 3104
        w.start()
        w.timers[0].fire()
        w.stop_ret = 0
        self.assertEqual(w.controller.stop()['ret'], 0)
        self.assertEqual(len(w.calls), 3)

    def test_guarded_repeat_cannot_stop_new_action_or_change_old_outcome(self):
        w = World()
        w.start('a')
        w.timers[0].fire()
        outcome = w.controller.get_result('a')
        w.stop_ret = 3104
        w.controller.stop('stop_repeat', expected_id='a', retry=True)
        self.assertEqual(w.controller.get_result('a'), outcome)
        w.stop_ret = 0
        w.start('b')
        count = len(w.calls)
        w.controller.stop('stop_repeat', expected_id='a', retry=True)
        self.assertEqual(len(w.calls), count)

    def test_timer_cannot_overtake_inflight_move_send(self):
        entered = threading.Event()
        release_send = threading.Event()
        expire_entered = threading.Event()
        stop_sent = threading.Event()
        calls = []
        timers = []
        class Timer:
            def __init__(self, duration, callback, args=()):
                self.callback, self.args = callback, args
            def start(self):
                timers.append(self)
            def cancel(self):
                pass
        class Client:
            def BeginMove(self, *args):
                entered.set()
                if not release_send.wait(2):
                    raise RuntimeError('test did not release send')
                calls.append('move')
                return lambda: 0
            def BeginStopMove(self):
                calls.append('stop')
                stop_sent.set()
                return lambda: 0
        controller = TimedMotion(Client(), timer_factory=Timer)
        worker = threading.Thread(target=lambda: controller.start(0, 0, 1, 1, 'a'))
        worker.start()
        def expire():
            expire_entered.set()
            timers[0].callback(*timers[0].args)
        stopper = None
        try:
            self.assertTrue(entered.wait(1))
            stopper = threading.Thread(target=expire)
            stopper.start()
            self.assertTrue(expire_entered.wait(1))
            self.assertFalse(stop_sent.is_set())
        finally:
            release_send.set()
            worker.join(2)
            if stopper:
                stopper.join(2)
        self.assertFalse(worker.is_alive())
        self.assertFalse(stopper.is_alive())
        self.assertEqual(calls, ['move', 'stop'])

    def test_real_timer_stops_while_move_wait_is_blocked(self):
        sent = threading.Event()
        release = threading.Event()
        stop_sent = threading.Event()
        calls = []
        class Client:
            def BeginMove(self, *args):
                calls.append('move')
                sent.set()
                def wait():
                    if not release.wait(2):
                        raise RuntimeError('test did not release response')
                    return 0
                return wait
            def BeginStopMove(self):
                calls.append('stop')
                stop_sent.set()
                return lambda: 0
        controller = TimedMotion(Client())
        errors = []
        def start():
            try:
                controller.start(0, 0, 1, .03, 'a')
            except Exception as exc:
                errors.append(exc)
        worker = threading.Thread(target=start)
        worker.start()
        try:
            self.assertTrue(sent.wait(1))
            self.assertTrue(stop_sent.wait(1))
            self.assertTrue(worker.is_alive())
            self.assertEqual(calls, ['move', 'stop'])
            self.assertEqual(controller.get_result('a')['status'], 'pending')
        finally:
            release.set()
            worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(controller.get_result('a')['status'], 'completed')


class TestTimingLogs(unittest.TestCase):
    def make_timing(self, enabled):
        cls = extract_class(ROOT / 'safety_harness.py', '_LocoTiming',
                            dict(os=os, time=time, threading=threading, json=json))
        with patch.dict(os.environ, {'G1_LOCO_TIMING': '1' if enabled else '0'}):
            return cls()

    def test_logs_distinguish_response_delay_and_timer_lateness(self):
        w = World()
        timing = self.make_timing(True)
        w.controller.timing = timing
        def slow_stop():
            w.now += 5.14
        w.stop_hook = slow_stop
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            w.start()
            w.timers[0].fire(lag=.2)
        rows = [json.loads(line.removeprefix('[LocoTiming] '))
                for line in output.getvalue().splitlines() if line.startswith("[LocoTiming] ")]
        fired = next(r for r in rows if r['stage'] == 'timer_fired')
        self.assertAlmostEqual(fired['lateness_s'], .2)
        stop = next(r for r in rows if r['stage'] == 'rpc_return' and r['method'] == 'StopMove')
        self.assertAlmostEqual(stop['elapsed_s'], 5.14)
        self.assertEqual({r['timing_id'] for r in rows}, {'a'})

    def test_send_logs_work_without_optional_timing_flag(self):
        w = World()
        w.controller.timing = self.make_timing(False)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            w.start()
            w.timers[0].fire()
        self.assertNotIn('[LocoTiming]', output.getvalue())
        self.assertEqual(output.getvalue().count('[LocoSend]'), 2)

    def test_send_logs_measure_local_interval_without_response_wait(self):
        w = World()
        def delayed_move_response():
            w.timers[0].fire(lag=.1)
            w.now = 4.2
        w.move_hook = delayed_move_response
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            w.start()
        rows = [json.loads(line.removeprefix('[LocoSend] '))
                for line in output.getvalue().splitlines()]
        self.assertEqual([r['method'] for r in rows], ['Move', 'StopMove'])
        stop = rows[1]
        self.assertAlmostEqual(stop['write_gap_lower_s'], 1.1)
        self.assertAlmostEqual(stop['write_gap_upper_s'], 1.1)
        self.assertAlmostEqual(stop['timer_lateness_s'], .1)
        self.assertEqual(stop['timer_to_stop_call_s'], 0.)
        self.assertEqual(stop['requested_duration_s'], 1)
        self.assertEqual(stop['stop_send_attempt'], 1)

    def test_send_interval_bounds_include_send_call_duration(self):
        w = World()
        begin = w.controller.client.BeginMove
        def slow_send(*args):
            w.now += .2
            return begin(*args)
        w.controller.client.BeginMove = slow_send
        w.stop_send_hook = lambda: setattr(w, 'now', w.now + .1)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            w.start()
            w.timers[0].fire()
        rows = [json.loads(line.removeprefix('[LocoSend] '))
                for line in output.getvalue().splitlines()]
        self.assertAlmostEqual(rows[0]['send_call_elapsed_s'], .2)
        self.assertAlmostEqual(rows[1]['send_call_elapsed_s'], .1)
        self.assertAlmostEqual(rows[1]['write_gap_lower_s'], .8)
        self.assertAlmostEqual(rows[1]['write_gap_upper_s'], 1.1)

    def test_broken_log_output_does_not_prevent_stop(self):
        w = World()
        w.controller.timing = self.make_timing(True)
        with patch('builtins.print', side_effect=OSError('fake broken pipe')):
            w.start()
            w.timers[0].fire()
        self.assertEqual(w.calls[-1][0], 'stop_sent')
        self.assertEqual(w.controller.get_result('a')['status'], 'completed')


if __name__ == '__main__':
    unittest.main()
