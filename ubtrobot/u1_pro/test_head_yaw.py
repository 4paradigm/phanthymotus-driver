import json
import math
import types
import unittest
from unittest import mock
from head_yaw import HeadYaw, wrap_degrees
import test_direction_card as card_fixture


def joint(angle=15, stamp=1700000000):
    return types.SimpleNamespace(name=['neck_base_joint'],position=[math.radians(angle)],
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(sec=stamp,nanosec=0)))


class YawFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.patch=mock.patch('head_yaw.time.time',return_value=1700000000.1)
        self.patch.start();self.addCleanup(self.patch.stop)
    def test_pose_sign_offset_freshness_and_reset(self):
        p=HeadYaw({'enabled':True,'yaw_sign':-1,'zero_offset_deg':2})
        p.update(joint(),100)
        self.assertAlmostEqual(p.snapshot(100.1)['head_yaw_deg'],-13)
        self.assertEqual(p.snapshot(100.4)['state'],'unavailable')
        p.reset();self.assertEqual(p.snapshot(100.1)['state'],'unavailable')
    def test_invalid_or_old_feedback_clears_previous_pose(self):
        p=HeadYaw({'enabled':True})
        for bad in (joint(float('nan')),joint(10,1699999990),types.SimpleNamespace(name=[],position=[])):
            p.update(joint(),100);p.update(bad,100.1)
            self.assertEqual(p.snapshot(100.2)['state'],'unavailable')
    def test_configuration_and_wrap(self):
        for cfg in ({'enabled':1},{'yaw_sign':0},{'yaw_sign':True},{'zero_offset_deg':float('inf')}):
            with self.assertRaises(ValueError):HeadYaw(cfg)
        self.assertEqual(wrap_degrees(-180),180)
        self.assertEqual(wrap_degrees(190),-170)


class CardCompensationTests(unittest.TestCase):
    def setUp(self):
        self.fixture=card_fixture.SoundDirectionTests()
        self.fixture.setUp()
        self.addCleanup(self.fixture.doCleanups)
        self.card=self.fixture.plugin;self.card.backend='native'
        self.card.running=True
        self.card._publisher=self.fixture.nodes.core.create_publisher.return_value
    def observation(self):
        return json.loads(self.card._publisher.publish.call_args.args[0].data)
    def event(self, yaw=15, state='estimated'):
        self.card._on_event({'azimuth':15,'confidence':.9,'frame_compensation':
            {'enabled':True,'state':state,'head_yaw_deg':yaw,
             'basis':'joint_feedback_at_doa_receipt_approximation'}})
    def test_front_compensation_and_frozen_heartbeat(self):
        self.event();d=self.observation()
        self.assertEqual(d['head_angle_deg'],-15)
        self.assertEqual(d['angle_deg'],0)
        self.assertEqual(d['reference_frame'],'torso_front')
        self.card._tick();self.assertEqual(self.observation()['angle_deg'],0)
        self.event(yaw=-15);self.assertEqual(self.observation()['angle_deg'],-30)
        self.assertEqual(self.observation()['audio_window_start_us'],0)
    def test_missing_pose_never_mislabels_head_angle_as_torso(self):
        self.event();self.event(state='unavailable')
        d=self.observation();self.assertEqual(d['state'],'no_event');self.assertIsNone(d['angle_deg'])
        self.assertEqual(d['head_angle_deg'],-15)
    def test_rear_wrap(self):
        self.card._on_event({'azimuth':-175,'frame_compensation':{'state':'estimated','head_yaw_deg':15}})
        self.assertEqual(self.observation()['angle_deg'],-170)
