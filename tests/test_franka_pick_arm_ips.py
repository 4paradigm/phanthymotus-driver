"""Shared controller endpoint contracts; no network or robot motion."""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'franka/fr3'))
import test_franka_observe as fixtures


class VisionEndpointTests(unittest.TestCase):
    def plugin(self):
        plugin = fixtures.ObserveTests().dual_plugin()
        plugin.inputs.start = Mock()
        plugin.inputs.stop = Mock()
        return plugin

    def test_shared_mapping_updates_both_choices_preserves_calibration_and_clears_photo(self):
        p = self.plugin()
        p.observation = {'old': 'photo'}
        before = copy.deepcopy(p.profiles)
        new = {'left': '172.16.0.8', 'right': '172.16.0.9'}
        p.apply_arm_mapping(new)
        self.assertEqual(p.addresses, new)
        self.assertEqual(p.address, new['right'])
        self.assertEqual(p.config['arm'], 'right')
        properties = p.get_tools()[0]['configSchema']['properties']
        self.assertFalse({'username', 'password'} & properties.keys())
        schema = properties['arm']
        self.assertEqual(list(properties), ['arm', 'speed_deg_s', 'speed_mm_s', 'force_n', 'observe_after_transfer'])
        self.assertTrue(all(field['scope'] == 'instance' for field in properties.values()))
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
        ports = p.get_tools()[0]['topic_in']
        self.assertEqual(len(ports), 3)
        self.assertEqual([v['format'] for v in ports].count('data/json'), 1)

    def test_active_or_uncertain_grasp_rejects_endpoint_change(self):
        p = self.plugin()
        original = dict(p.addresses)
        p.active = {'action_id': 'test'}
        with self.assertRaises(RuntimeError):
            p.apply_arm_mapping({'left': '172.16.0.8', 'right': '172.16.0.9'})
        p.apply_arm_mapping(original)  # An unchanged mapping is safe during motion.
        p.active = None
        p.last_result = {'result': {'holding_object_possible': True}}
        with self.assertRaises(RuntimeError):
            p.apply_arm_mapping({'left': '172.16.0.8', 'right': '172.16.0.9'})
        self.assertEqual(p.addresses, original)

    def test_full_start_passes_only_camera_topics_to_the_existing_reader(self):
        p = self.plugin()
        topics = ['/test/rgb', '/franka/arm_ip_sync', '/test/depth', '/test/rgb/objects']
        config = dict(arm='left', speed_deg_s=8, speed_mm_s=25, force_n=35, observe_after_transfer=True)
        p.dispatch('start', dict(input_topics=topics, **config))
        self.assertEqual({key: p.config[key] for key in config}, config)
        self.assertEqual(p.inputs.start.call_args.args[0]['input_topics'],
                         ['/test/rgb', '/test/depth', '/test/rgb/objects'])
        self.assertTrue(p.enabled)

    def test_unverified_mapping_blocks_all_visual_motion(self):
        p = self.plugin()
        p.controller_configuration = Mock()
        p.controller_configuration.snapshot.side_effect = RuntimeError('Discovering the PNP controller')
        for action in ('observe', 'grab_to', 'grab_by'):
            result = p.dispatch(action, {'confirm_motion': True})
            self.assertEqual(result['code'], 'CONTROLLER_CONFIGURATION_NOT_READY')
        self.assertIsNone(p.active)
