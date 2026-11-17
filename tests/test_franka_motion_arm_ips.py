"""Joint/Hand stream routing with deterministic fake hardware only."""
import json
import sys
from pathlib import Path
import time
import unittest
from unittest.mock import Mock, patch

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
        jm = joints.hardware.SessionManager(session_factory=joints.FakeSession)
        gm = hands.fixtures.hardware.SessionManager(session_factory=hands.FakeGripper)
        joint = joints.device.JointControlPlugin({'motion_arm_ips': OLD}, manager=jm,
            discovery=discovery, acp_callback=lambda *args: None)
        hand = hands.gripper.GripperPlugin({'motion_arm_ips': OLD}, manager=gm,
            discovery=discovery, acp_callback=lambda *args: None)
        joint._arm_ip_peer, hand._arm_ip_peer = hand, joint
        for p in (joint, hand):
            p.READY_TIMEOUT = .1
            self.addCleanup(p.stop)
        return joint, hand

    def call(self, p, action, **args):
        return DriverBundle([p]).dispatch(p.PREFIX, {'action': action, **args})

    def action(self, p, arm='left'):
        if p.PREFIX == 'joint_control':
            return self.call(p, 'set', arm=arm, joint1_deg=1, speed_deg_s=5)
        return self.call(p, 'set_position', arm=arm, width_mm=30, speed_mm_s=20)

    def connect(self, p, addresses=NEW):
        p._arm_ip_input.topic = '/franka/arm_ip_sync'
        p._arm_ip_input.receive(json.dumps(addresses))
        self.assertTrue(p._arm_ip_input.info()['ready'])

    def test_options_unchanged_and_no_mapping_never_opens_hardware(self):
        for p in self.plugins():
            tool = p.get_tools()[0]
            self.assertEqual(tool['inputSchema']['properties']['arm']['enum'], ['left', 'right'])
            self.assertEqual(tool['topic_in'][0]['format'], 'data/json')
            self.assertNotIn('configSchema', tool)
            result = self.action(p)
            self.assertEqual(result['code'], 'ARM_IP_INPUT_NOT_READY')
            self.assertFalse(p._manager._sessions)
            self.assertFalse(p._instances)

    def test_both_sides_route_using_received_mapping_even_after_heartbeat_pause(self):
        for p in self.plugins():
            self.connect(p)
            p._arm_ip_input.received_at = time.monotonic() - 60
            for side in ('left', 'right'):
                result = self.action(p, side)
                self.assertEqual(result['state'], 'running')
                self.assertIn(NEW[side], p._manager._sessions)
                session = p._manager._sessions[NEW[side]][0]
                session.finish(result['action_id'])
            info = self.call(p, 'info')
            self.assertEqual(info['motion_arm_ips'], NEW)
            self.assertTrue(info['arm_ip_input']['ready'])
            self.assertFalse(info['arm_ip_input']['fresh'])

    def test_updates_reject_busy_motion_then_apply_when_idle(self):
        for p in self.plugins():
            self.connect(p, OLD)
            result = self.action(p)
            session = p._manager._sessions[OLD['left']][0]
            p._arm_ip_input.receive(json.dumps(NEW))
            self.assertEqual(p._arm_ips, OLD)
            self.assertFalse(p._arm_ip_input.info()['ready'])
            session.finish(result['action_id'])
            p._arm_ip_input.receive(json.dumps(NEW))
            self.assertEqual(p._arm_ips, NEW)
            self.assertTrue(p._arm_ip_input.info()['ready'])
            self.assertFalse(p._instances)
            self.assertFalse(p._manager._sessions)

    def test_held_object_blocks_endpoint_changes_on_both_cards(self):
        joint, hand = self.plugins()
        self.connect(joint, OLD)
        self.connect(hand, OLD)
        result = self.action(hand)
        session = hand._manager._sessions[OLD['left']][0]
        session.finish(result['action_id'])
        session.value['is_grasped'] = True
        for p in (joint, hand):
            p._arm_ip_input.receive(json.dumps(NEW))
            self.assertEqual(p._arm_ips, OLD)
            self.assertIn('holding an object', p._arm_ip_input.info()['error'])

    def test_start_stop_lifecycle_and_stopmotion_remain_available(self):
        for p in self.plugins():
            def start(topic): p._arm_ip_input.topic = topic
            with patch.object(p._arm_ip_input, 'start', side_effect=start):
                result = self.call(p, 'start', input_topic='/franka/arm_ip_sync')
            self.assertEqual(result['state'], 'ready')
            p._arm_ip_input.receive(json.dumps(NEW))
            motion = self.action(p)
            p._arm_ip_input.stop()
            stopped = self.call(p, 'stopmotion', arm='left')
            self.assertNotEqual(stopped.get('code'), 'ARM_IP_INPUT_NOT_READY')
            self.assertEqual(self.call(p, 'stop')['state'], 'idle')
            self.assertFalse(p._arm_ip_input.info()['connected'])

    def test_unchanged_heartbeats_remain_ready_while_dispatch_lock_is_busy(self):
        import threading
        for p in self.plugins():
            self.connect(p)
            locked, release = threading.Event(), threading.Event()
            def hold():
                with p._endpoint_lock:
                    locked.set()
                    release.wait(2)
            thread = threading.Thread(target=hold)
            thread.start()
            self.assertTrue(locked.wait(1))
            try:
                p._arm_ip_input.receive(json.dumps(NEW))
                self.assertTrue(p._arm_ip_input.info()['ready'])
            finally:
                release.set()
                thread.join(2)
