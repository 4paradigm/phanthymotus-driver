"""Mode gating uses live FSM data and never infers standing from a mode match."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch


import _point_test_helpers
import directional_stationary as MODULE
Guard = MODULE.DirectionalStationaryGuard


class ModeGuardTests(unittest.TestCase):
    def setUp(self):
        self.now = 1000.
        self.wall = 1700001000.
        low_state = SimpleNamespace(snapshot=self.unexpected_lowstate_read)
        self.guard = Guard(low_state, {'hold_s': .5}, clock=lambda: self.now,
                           wall_clock=lambda: self.wall)

    def unexpected_lowstate_read(self):
        self.fail('mode guard read body feedback')

    def mode(self, fsm_id=500, fsm_mode=0, **extra):
        self.guard.update_fsm(dict(fsm_id=fsm_id, fsm_mode=fsm_mode, ts=self.wall, **extra))
        return self.guard.check()

    def test_first_mode_sample_admits_both_normal_submodes_without_dwell(self):
        for submode in (0, 1):
            with self.subTest(submode=submode):
                self.setUp()
                result = self.mode(fsm_mode=submode)
                self.assertTrue(result['stationary'])
                self.assertTrue(result['mode_allowed'])
                self.assertEqual(result['guard_kind'], 'locomotion_mode')
                self.assertEqual(result['reason'], 'normal_loco_mode')
                self.assertEqual(result['fsm_mode'], submode)

    def test_walking_turning_and_absent_body_feedback_do_not_veto_mode(self):
        self.assertTrue(self.mode()['mode_allowed'])
        for odometry in ({'position': [4., 5., 0.], 'velocity': [1., .5, 0.],
                          'yaw_speed': .7, 'imu': {'rpy': [.3, .3, .3]}}, {}, None):
            self.guard.update_odom(odometry)
            self.assertTrue(self.guard.check()['mode_allowed'])

    def test_switching_out_of_allowed_mode_rejects_current_call(self):
        for fsm_id, mode in ((0, 0), (4, 0), (501, 0), (500, 2)):
            self.setUp()
            self.assertTrue(self.mode()['mode_allowed'])
            self.now += .01
            self.wall += .01
            result = self.mode(fsm_id, mode)
            self.assertFalse(result['stationary'])
            self.assertEqual(result['reason'], 'fsm_not_operational')

    def test_missing_mode_has_no_permission(self):
        result = self.guard.check()
        self.assertFalse(result['stationary'])
        self.assertEqual(result['reason'], 'fsm_missing')

    def test_receipt_and_source_clocks_each_expire_the_sample(self):
        for local_delta, source_delta in ((.251, .1), (.1, .251), (-.01, 0.), (0., -.01)):
            self.setUp()
            self.mode()
            self.now += local_delta
            self.wall += source_delta
            self.assertFalse(self.guard.check()['stationary'])
            self.assertEqual(self.guard.check()['reason'], 'fsm_stale')

    def test_exact_timeout_is_inclusive_and_polling_cannot_refresh_it(self):
        self.mode()
        self.now += .25
        self.wall += .25
        self.assertTrue(self.guard.check()['stationary'])
        self.now += .001
        self.assertFalse(self.guard.check()['stationary'])

    def test_repeat_and_older_timestamp_invalidate_until_an_advancing_frame(self):
        for delta in (0., -.01):
            self.setUp()
            self.mode()
            self.guard.update_fsm(dict(fsm_id=500, fsm_mode=1, ts=self.wall+delta))
            self.assertEqual(self.guard.check()['reason'], 'fsm_time_not_advancing')
            self.now += .01
            self.wall += .01
            self.assertTrue(self.mode(fsm_mode=1)['stationary'])

    def test_future_timestamp_does_not_block_recovery_by_valid_frame(self):
        self.guard.update_fsm(dict(fsm_id=500, fsm_mode=0, ts=self.wall+100))
        self.assertEqual(self.guard.check()['reason'], 'fsm_stale')
        self.assertTrue(self.mode()['stationary'])

    def test_invalid_fields_revoke_previous_permission(self):
        for fields in ({}, None, {'fsm_id': '500', 'fsm_mode': 0, 'ts': self.wall},
                       {'fsm_id': 500, 'fsm_mode': False, 'ts': self.wall},
                       {'fsm_id': 500, 'fsm_mode': 0, 'ts': float('nan')},
                       {'fsm_id': 500, 'fsm_mode': 0, 'ts': True}):
            self.setUp()
            self.mode()
            self.guard.update_fsm(fields)
            self.assertFalse(self.guard.check()['stationary'])
            self.assertEqual(self.guard.check()['reason'], 'fsm_invalid')

    def test_close_is_final(self):
        self.mode()
        self.guard.close()
        self.now += .01
        self.wall += .01
        self.mode()
        self.assertEqual(self.guard.check()['reason'], 'closed')
        self.assertFalse(self.guard.check()['stationary'])

    def test_node_subscribes_only_to_existing_mode_topic(self):
        subscriptions = []
        node = SimpleNamespace(create_subscription=lambda *args: subscriptions.append(args))
        modules = {
            'rclpy.node': SimpleNamespace(Node=lambda name: node),
            'rclpy.qos': SimpleNamespace(QoSProfile=lambda **kwargs: kwargs,
                ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1),
                HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
                DurabilityPolicy=SimpleNamespace(VOLATILE=1)),
            'std_msgs.msg': SimpleNamespace(String=object),
        }
        with patch.dict('sys.modules', modules):
            self.assertIs(MODULE.create_stationary_node('ubuntu', self.guard), node)
        self.assertEqual(len(subscriptions), 1)
        self.assertEqual(subscriptions[0][1], '/ubuntu/loco/motion_state')
        receive = subscriptions[0][2]
        receive(SimpleNamespace(data=json.dumps(dict(fsm_id=500, fsm_mode=1, ts=self.wall))))
        self.assertTrue(self.guard.check()['mode_allowed'])
        receive(SimpleNamespace(data='broken json'))
        self.assertFalse(self.guard.check()['mode_allowed'])


if __name__ == '__main__':
    unittest.main()
