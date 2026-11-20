"""Joint/Hand routing from the shared controller configuration."""
import sys
from pathlib import Path
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'franka/fr3')]
import test_franka_joint_control as joints
import test_franka_gripper as hands
from common.vendor_runtime import DriverBundle

OLD = {'left': '172.16.0.3', 'right': '172.16.0.2'}
NEW = {'left': '172.16.0.8', 'right': '172.16.0.9'}


class MotionArmIPTests(unittest.TestCase):
    def plugins(self):
        discovery = Mock()
        discovery.snapshot.return_value = {'state': 'ready', 'robots': []}
        self.configuration = Mock()
        self.configuration.snapshot.side_effect = RuntimeError('Discovering the PNP controller')
        jm = joints.hardware.SessionManager(session_factory=joints.FakeSession)
        gm = hands.fixtures.hardware.SessionManager(session_factory=hands.FakeGripper)
        options = dict(discovery=discovery, acp_callback=lambda *args: None,
                       controller_configuration=self.configuration)
        joint = joints.device.JointControlPlugin({'motion_arm_ips': OLD}, manager=jm, **options)
        hand = hands.gripper.GripperPlugin({'motion_arm_ips': OLD}, manager=gm, **options)
        joint._arm_ip_peer, hand._arm_ip_peer = hand, joint
        for p in (joint, hand):
            p.READY_TIMEOUT = .1
            self.addCleanup(p.stop)
        return joint, hand

    def call(self, card, action, **args):
        return DriverBundle([card]).dispatch(card.PREFIX, {'action': action, **args})

    def action(self, card, arm='left'):
        if card.PREFIX == 'joint_control':
            return self.call(card, 'set', arm=arm, joint1_deg=1, speed_deg_s=5)
        return self.call(card, 'set_position', arm=arm, width_mm=30, speed_mm_s=20)

    def verified(self, arms):
        self.configuration.snapshot.side_effect = None
        self.configuration.snapshot.return_value = ('172.16.0.200', arms, 'pnp', 'ros')

    def test_motion_waits_for_verified_addresses_and_routes_without_wiring(self):
        cards = self.plugins()
        for card in cards:
            tool = card.get_tools()[0]
            self.assertEqual(tool['inputSchema']['properties']['arm']['enum'], ['left', 'right'])
            self.assertNotIn('topic_in', tool)
            self.assertEqual(self.action(card)['code'], 'CONTROLLER_CONFIGURATION_NOT_READY')
            self.assertFalse(card._manager._sessions)
        self.verified(NEW)
        for card in cards:
            for arm in ('left', 'right'):
                result = self.action(card, arm)
                self.assertEqual(result['state'], 'running')
                session = card._manager._sessions[NEW[arm]][0]
                session.finish(result['action_id'])

    def test_new_mapping_waits_for_active_or_held_motion_and_stop_stays_available(self):
        joint, hand = self.plugins()
        self.verified(OLD)
        result = self.action(hand)
        session = hand._manager._sessions[OLD['left']][0]
        self.verified(NEW)
        self.assertEqual(self.action(joint)['code'], 'CONTROLLER_CONFIGURATION_NOT_READY')
        session.finish(result['action_id'])
        session.value['is_grasped'] = True
        self.assertIn('holding an object', self.action(joint)['message'])
        self.assertEqual(joint._arm_ips, OLD)
        stopped = self.call(hand, 'stopmotion', arm='left')
        self.assertNotEqual(stopped.get('code'), 'CONTROLLER_CONFIGURATION_NOT_READY')
