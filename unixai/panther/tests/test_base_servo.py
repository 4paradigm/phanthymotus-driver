"""Exercise stream expiry, motor ownership and independent watchdog output."""
import queue
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT.parents[1])]
from base_servo import BaseServoPlugin, descriptor, ZERO
from device import PantherDevice


class Device:
    base_check_available = PantherDevice.base_check_available
    drive = PantherDevice.drive

    def __init__(self):
        self._base_gate = threading.RLock()
        self._cfg = {'plugins': {'base_drive': {'max_linear_mps': .5, 'max_angular_deg_s': 45.}}}
        self._base_servo = self._controlled_spatial = None
        self._drive_action_id = self._drive_stop_request = None
        self._closed = False
        self.output = []
        self.matched = True
        self.servo_prepare = Mock()
        self._ensure_motion_ready = Mock()
        self._drive_impl = Mock(return_value={'state': 'queued'})

    def servo_publish(self, values):
        if any(values) and not self.matched:
            raise RuntimeError('subscriber lost')
        self.output.append(tuple(values))
        return self.matched


class Clock:
    def __init__(self):
        self.now = 10.
    def mono(self):
        return self.now
    def wall(self):
        return self.now + 1000.
    def advance(self, seconds):
        self.now += seconds


def command(clock, seq=1, values=None, **overrides):
    wall = int(clock.wall()*1000)
    return dict(schema='motus.control/1', mode='twist', dof=6, source='actucore_navi',
                seq=seq, values=values if values is not None else [.2, 0., 0., 0., 0., .3],
                stamp_ms=wall, obs_stamp_ms=wall, ttl_ms=300, **overrides)


class StreamTest(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.patch_mono = patch('base_servo.time.monotonic', self.clock.mono)
        self.patch_wall = patch('base_servo.time.time', self.clock.wall)
        self.patch_mono.start(); self.patch_wall.start()
        self.addCleanup(self.patch_mono.stop); self.addCleanup(self.patch_wall.stop)
        self.device = Device()
        self.servo = BaseServoPlugin(self.device, {})
        self.device._base_servo = self.servo
        self.servo._running = True
        self.servo._arm()
        self.servo._inbox = queue.Queue()

    def send(self, message):
        self.servo._inbox.put((message, self.clock.mono()))
        self.servo._step()

    def test_command_is_bounded_and_explicit_zero_is_immediate(self):
        self.send(command(self.clock))
        self.assertAlmostEqual(self.device.output[-1][0], .03)
        self.assertAlmostEqual(self.device.output[-1][5], .06)
        self.send(command(self.clock, seq=2, values=list(ZERO)))
        self.assertEqual(self.device.output[-1], ZERO)

    def test_rejects_stale_future_invalid_and_unsupported_axes(self):
        bad = []
        for updates in ({'stamp_ms': 0}, {'obs_stamp_ms': 0}, {'ttl_ms': 10000},
                        {'obs_stamp_ms': None}, {'stamp_ms': self.clock.wall()*1000+100},
                        {'values': [0, 0, .01, 0, 0, 0]}, {'values': [float('nan')]*6},
                        {'values': [0]*5}, {'schema': 'other'}, {'ttl_ms': True}):
            message = command(self.clock); message.update(updates); bad.append(message)
        for i, message in enumerate(bad):
            message['seq'] = i+1
            self.send(message)
        self.assertEqual(self.servo._applied, 0)
        self.assertTrue(all(not any(v) for v in self.device.output))

    def test_sequence_regression_and_source_priority_are_checked(self):
        self.send(command(self.clock))
        self.clock.advance(.06)
        self.send(command(self.clock))
        self.assertEqual(self.servo._applied, 1)
        message = command(self.clock, seq=2); message.update(source='other', priority=-1)
        self.clock.advance(.06)
        self.send(message)
        self.assertEqual(self.servo._applied, 1)

    def test_ttl_stops_before_watchdog_and_never_replays_old_motion(self):
        message = command(self.clock); message['ttl_ms'] = 80
        self.send(message)
        self.clock.advance(.1); self.servo._step()
        self.assertEqual(self.device.output[-1], ZERO)
        count = len(self.device.output)
        for _ in range(8):
            self.clock.advance(.1); self.servo._step()
        self.assertTrue(all(v == ZERO for v in self.device.output[count:]))

    def test_receiver_exit_pauses_and_confirms_zero_before_releasing(self):
        self.send(command(self.clock))
        self.servo._process = NS(is_alive=lambda: False)
        self.clock.advance(.06); self.servo._step()
        self.assertTrue(self.servo._paused)
        self.assertTrue(self.servo._reserved)
        for _ in range(3):
            self.clock.advance(.06); self.servo._step()
        self.assertFalse(self.servo._reserved)
        self.assertTrue(self.servo._stop_confirmed)

    def test_unmatched_stop_keeps_ownership_and_blocks_manual_motion(self):
        self.send(command(self.clock)); self.device.matched = False
        with patch('base_servo.time.sleep', self.clock.advance):
            with self.assertRaisesRegex(RuntimeError, 'unconfirmed'):
                self.device.drive({'action': 'move'})
        self.assertTrue(self.servo._reserved)
        self.assertFalse(self.servo._stop_confirmed)
        self.device._drive_impl.assert_not_called()

    def test_zero_input_without_reader_cannot_confirm_stop(self):
        self.device.matched = False
        for seq in range(1, 5):
            self.clock.advance(.06)
            self.send(command(self.clock, seq=seq, values=list(ZERO)))
        self.assertFalse(self.servo._stop_confirmed)
        self.assertEqual(self.servo._zero_count, 0)

    def test_manual_motion_pauses_policy_and_precedes_fresh_resume(self):
        self.send(command(self.clock))
        old = command(self.clock, seq=2)
        with patch('base_servo.time.sleep', self.clock.advance):
            self.device.drive({'action': 'move'})
        self.assertTrue(self.servo._paused)
        self.assertFalse(self.servo._reserved)
        self.assertEqual(self.device.output[-3:], [ZERO]*3)
        self.device._drive_impl.assert_called_once()
        self.clock.advance(.2)
        self.servo._arm()
        self.send(old)
        self.assertEqual(self.servo._last_command['verdict'], 'REJECTED')

    def test_nav2_and_call_motion_block_stream_acquisition(self):
        self.servo._reserved = False
        self.device._controlled_spatial = NS(nav={'state': 'canceling'})
        with self.assertRaisesRegex(RuntimeError, 'Nav2'):
            self.device.base_check_available('base_servo')
        self.device._controlled_spatial.nav = None
        self.device._drive_stop_request = {'state': 'pending'}
        with self.assertRaisesRegex(RuntimeError, 'still active'):
            self.device.base_check_available('base_servo')

    def test_config_requires_confirmed_pause_and_atomic_boolean_validation(self):
        self.assertEqual(self.servo.dispatch('config', {'dry_run': True})['state'], 'error')
        self.servo._reserved = False
        result = self.servo.dispatch('config', {'dry_run': True, 'rotate_only': 'false'})
        self.assertEqual(result['state'], 'error')
        self.assertFalse(self.servo._dry_run)


class LifecycleTest(unittest.TestCase):
    def test_bundle_start_is_inert_and_descriptor_matches_navi(self):
        d = Device(); p = BaseServoPlugin(d, {})
        p.start()
        self.assertIsNone(p._process)
        self.assertFalse(p._reserved)
        d.servo_prepare.assert_not_called()
        tool = p.get_tool()
        self.assertEqual(tool['topic_in'][0]['format'], 'control/velocity')
        self.assertEqual(tool['control_interface']['frame'], 'base_link')
        self.assertTrue(tool['control_interface']['limits']['zero_immediate'])
        self.assertEqual(tool['inputSchema']['x-resource'], 'base')

    def test_watchdog_runs_without_ros_executor_progress(self):
        d = Device(); p = BaseServoPlugin(d, {}); d._base_servo = p
        p._running = True; p._arm()
        got_zero = threading.Event()
        def record(values):
            d.output.append(tuple(values))
            if not any(values): got_zero.set()
            return True
        d.servo_publish = record
        now = int(time.time()*1000)
        message = dict(schema='motus.control/1', mode='twist', dof=6, source='navi', seq=1,
                       values=[.1, 0, 0, 0, 0, .1], stamp_ms=now+1, obs_stamp_ms=now+1, ttl_ms=80)
        p._accept(message, time.monotonic())
        self.assertTrue(any(d.output[-1]))
        thread = threading.Thread(target=p._loop)
        thread.start()
        try:
            self.assertTrue(got_zero.wait(1))
            self.assertEqual(d.output[-1], ZERO)
        finally:
            p._thread_end.set(); thread.join(timeout=1)

    def test_descriptor_acceleration_steps_follow_expected_rate(self):
        drive = Device()._cfg['plugins']['base_drive']
        for config, hz, linear, angular in (({}, 10., .03, .06),
                                            ({'expected_hz': 15.}, 15., .02, .04)):
            with self.subTest(config=config):
                result = descriptor(config, drive)
                self.assertEqual(result['rate']['expected_hz'], hz)
                self.assertEqual(result['rate']['max_hz'], 20.)
                steps = result['limits']['max_delta_per_step']
                self.assertAlmostEqual(steps[0], linear)
                self.assertAlmostEqual(steps[1], linear)
                self.assertAlmostEqual(steps[5], angular)

    def test_descriptor_cannot_widen_base_limits(self):
        for cfg in ({'max_linear_mps': .6}, {'max_angular_rad_s': 1.}, {'expected_hz': 25},
                    {'max_linear_mps': True}, {'linear_acceleration_mps2': float('nan')}):
            with self.subTest(config=cfg), self.assertRaises(ValueError):
                descriptor(cfg, Device()._cfg['plugins']['base_drive'])


if __name__ == '__main__':
    unittest.main()
