"""Wired endpoint input contracts; no network or robot motion."""
import copy
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'franka/fr3'))
from franka.fr3.arm_ips import ArmIPInput, mapping, split_topics
import test_franka_observe as fixtures


class ArmInputTests(unittest.TestCase):
    def test_topic_resolution_distinguishes_both_json_inputs_in_any_order(self):
        topics = ['/test/rgb/objects', '/franka/arm_ip_sync', '/test/depth', '/test/rgb']
        ip, vision = split_topics({'input_topics': topics})
        self.assertEqual(ip, '/franka/arm_ip_sync')
        self.assertEqual(vision, ['/test/rgb/objects', '/test/depth', '/test/rgb'])
        self.assertEqual(split_topics({'input_topic': ip}), (ip, []))
        self.assertEqual(split_topics({'input_topics': vision}), (None, vision))
        for value in [['/a/arm_ip_sync', '/b/arm_ip_sync'], ['relative'], ['/x', '/x']]:
            with self.assertRaises(ValueError): split_topics({'input_topics': value})

    def test_only_valid_mapping_reaches_the_consumer(self):
        apply = Mock()
        reader = ArmIPInput(None, apply)
        reader.topic = '/franka/arm_ip_sync'
        for value in [{'objects': []}, {'left': 'bad', 'right': '172.16.0.2'},
                      {'left': '172.16.0.2', 'right': '172.16.0.2'}]:
            reader.receive(json.dumps(value))
            self.assertFalse(reader.info()['fresh'])
        reader.receive('x' * 4097)
        apply.assert_not_called()
        ips = {'left': '172.16.0.3', 'right': '172.16.0.2'}
        reader.receive(json.dumps(ips))
        apply.assert_called_once_with(ips)
        self.assertTrue(reader.info()['fresh'])
        reader.received_at = time.monotonic() - 4
        self.assertFalse(reader.info()['fresh'])
        self.assertTrue(reader.info()['ready'])
        reader.stop()
        self.assertFalse(reader.info()['connected'])

    def test_failed_apply_is_not_reported_as_a_fresh_mapping(self):
        reader = ArmIPInput(None, Mock(side_effect=RuntimeError('busy')))
        reader.topic = '/franka/arm_ip_sync'
        reader.receive('{"left":"172.16.0.3","right":"172.16.0.2"}')
        self.assertEqual(reader.info()['error'], 'busy')
        self.assertFalse(reader.info()['fresh'])
        self.assertFalse(reader.info()['ready'])
        self.assertIsNone(reader.info()['motion_arm_ips'])


class VisionEndpointTests(unittest.TestCase):
    def plugin(self):
        plugin = fixtures.ObserveTests().dual_plugin()
        plugin.inputs.start = Mock()
        plugin.inputs.stop = Mock()
        return plugin

    def test_wired_mapping_updates_both_choices_preserves_calibration_and_clears_photo(self):
        p = self.plugin()
        p.observation = {'old': 'photo'}
        before = copy.deepcopy(p.profiles)
        new = {'left': '172.16.0.8', 'right': '172.16.0.9'}
        p.apply_arm_ip_input(new)
        self.assertEqual(p.addresses, new)
        self.assertEqual(p.address, new['right'])
        self.assertEqual(p.config['arm'], 'right')
        schema = p.get_tools()[0]['configSchema']['properties']['arm']
        self.assertEqual(schema['enum'], ['left', 'right'])
        self.assertEqual(schema['enumNames'], ['Left hand', 'Right hand'])
        self.assertIsNone(p.observation)
        for side in new:
            before[side]['planning']['handeye']['robot_ip'] = new[side]
        self.assertEqual(p.profiles, before)
        # Previously saved canvas labels still select the same physical arm.
        r = p.dispatch('config', {'arm': '172.16.0.2（左手）', 'force_n': 30})
        self.assertTrue(r['ok'])
        self.assertEqual(p.address, new['left'])
        self.assertEqual(p.info()['motion_arm_ips'], new)
        ports = p.get_tools()[0]['topic_in']
        self.assertEqual(len(ports), 4)
        self.assertEqual([v['format'] for v in ports].count('data/json'), 2)

    def test_active_or_uncertain_grasp_rejects_endpoint_change(self):
        p = self.plugin()
        original = dict(p.addresses)
        p.active = {'action_id': 'test'}
        with self.assertRaises(RuntimeError):
            p.apply_arm_ip_input({'left': '172.16.0.8', 'right': '172.16.0.9'})
        p.apply_arm_ip_input(original)  # Normal heartbeats are safe during motion.
        p.active = None
        p.last_result = {'result': {'holding_object_possible': True}}
        with self.assertRaises(RuntimeError):
            p.apply_arm_ip_input({'left': '172.16.0.8', 'right': '172.16.0.9'})
        self.assertEqual(p.addresses, original)

    def test_full_start_passes_only_camera_topics_to_the_existing_reader(self):
        p = self.plugin()
        topics = ['/test/rgb', '/franka/arm_ip_sync', '/test/depth', '/test/rgb/objects']
        with patch.object(p.arm_ip_input, 'start') as start:
            p.dispatch('start', {'input_topics': topics})
        start.assert_called_once_with('/franka/arm_ip_sync')
        self.assertEqual(p.inputs.start.call_args.args[0]['input_topics'],
                         ['/test/rgb', '/test/depth', '/test/rgb/objects'])
        self.assertTrue(p.enabled)

    def test_ip_only_start_and_stale_input_never_command_motion(self):
        p = self.plugin()
        def started(topic): p.arm_ip_input.topic = topic
        with patch.object(p.arm_ip_input, 'start', side_effect=started):
            p.dispatch('start', {'input_topic': '/franka/arm_ip_sync'})
        p.inputs.start.assert_not_called()
        self.assertFalse(p.enabled)
        for action in ('observe', 'grab_to', 'grab_by'):
            result = p.dispatch(action, {'confirm_motion': True})
            self.assertEqual(result['code'], 'ARM_IP_INPUT_NOT_READY')
        p.arm_ip_input.receive(json.dumps(p.addresses))
        self.assertTrue(p.arm_ip_input.info()['fresh'])
        p.stop()
        self.assertIsNone(p.arm_ip_input.topic)
