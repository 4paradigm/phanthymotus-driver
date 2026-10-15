"""Public two-card acceptance using a recording SDK; no robot access."""
import json
import math
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from agilex.piper.device import build_plugins
from agilex.piper.intent_teaching import exit_teaching_and_hold
from agilex.piper.user_motion import (Transform, rotation, euler, orientation_error, vector,
                                      execute_target, target_pose)
from agilex.piper.path_capability import path_capability, driver_digest, configuration_digest
from agilex.piper.preset import capture_position
from common.vendor_runtime import DriverBundle
from test_piper_intent import IntentDriver, FastClock


def fast_execute(*args, **kwargs):
    clock = FastClock()
    return execute_target(*args, **kwargs, clock=clock.now, sleep=clock.sleep)


def fast_exit(*args, **kwargs):
    clock = FastClock()
    return exit_teaching_and_hold(*args, **kwargs, clock=clock.now, sleep=clock.sleep)


def fast_capture(*args, **kwargs):
    clock = FastClock()
    return capture_position(*args, **kwargs, clock=clock.now, sleep=clock.sleep)


class UnifiedTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        gate = patch.dict('os.environ', {'PIPER_MOTION_ENABLED': '1'})
        gate.start(); self.addCleanup(gate.stop)
        self.config = {'data_dir': self.tmp.name, 'async_actions': False, 'intent_control': {
            'tcp_verified': True, 'tcp_offset_mm_deg': [0, 0, 0, 0, 0, 0],
            'preset_joints_deg': [1, 10, -10, 0, 20, 0]}}
        self.plugins = build_plugins(self.config, 'piper_arm1', None)
        self.runtime, self.background = self.plugins[:2]
        # No unit test should accidentally post to a locally running Core.
        self.completion_mock = patch.object(self.runtime, 'acp_complete', return_value=None)
        self.completion_mock.start(); self.addCleanup(self.completion_mock.stop)
        self.runtime.factory = IntentDriver
        self.background.auto_connect = False
        for plugin in self.plugins:
            self.addCleanup(plugin.stop)
        self.driver = self.runtime.connected()
        self.sdk = self.driver.arm
        self.bundle = DriverBundle(self.plugins)
        for name, function in [('execute_target', fast_execute), ('exit_teaching_and_hold', fast_exit)]:
            mock = patch('agilex.piper.unified_control.' + name, side_effect=function)
            mock.start(); self.addCleanup(mock.stop)

    def tearDown(self):
        self.runtime.completions.wait(12)

    def call(self, action, **args):
        return self.bundle.dispatch('arm_control', {'action': action, **args})

    def targets(self, kind='end_pose'):
        return [call[1] for call in self.sdk.calls if call[0] == kind]

    def test_two_card_schema_topics_and_legacy_request_rejection(self):
        tools = {t['name']: t for t in self.bundle.get_all_tools()}
        self.assertEqual(set(tools), {'arm_control', 'joints', 'model', 'piper_photo', 'piper_camera_live'})
        arm = tools['arm_control']
        self.assertEqual(arm['title'], 'Arm control')
        schema = arm['inputSchema']
        self.assertEqual(schema['x-completion']['actions'], sorted(self.runtime.task_actions))
        self.assertNotRegex(json.dumps(tools, ensure_ascii=False), r'[\u3400-\u9fff]')
        self.assertEqual(arm['topic_out'], [])
        self.assertEqual(tools['joints']['topic_out'][0]['format'], 'sensor/skeleton')
        self.assertEqual(tools['joints']['title'], 'Joints')
        self.assertEqual(tools['joints']['type'], 'sensor')
        self.assertFalse(any(t['format'] == 'data/json' for tool in tools.values() for t in tool.get('topic_out', [])))
        self.assertIn('tip-relative', arm['description'])
        self.assertIn('Every control action first exits', arm['description'])
        for old in ('move_forward', 'exit_teaching_and_hold', 'enable', 'switch', 'move_joint', 'set_position', 'open_gripper', 'close_gripper'):
            with self.subTest(old=old), self.assertRaisesRegex(ValueError, 'replaced'):
                self.call(old)
        self.assertEqual(self.sdk.calls, [])

    def test_driver_catalog_matches_runtime_tools(self):
        import yaml
        from pathlib import Path
        definition = yaml.safe_load((Path(__file__).resolve().parents[1] / 'agilex/piper/driver.yaml').read_text())
        self.assertEqual({(card['name'], card['type']) for card in definition['cards']},
                         {(tool['name'], tool['type']) for tool in self.bundle.get_all_tools()})

    def test_start_status_and_default_selection_are_passive(self):
        self.call('start')
        self.assertFalse(self.call('select_action')['executed'])
        info = self.call('get_status', width_mm=50, pose_mm_deg='retained invalid', input_topic='')
        self.assertTrue(info['snapshot']['tcp_verified'])
        self.assertEqual(len(info['snapshot']['tip_pose_mm_deg']), 6)
        self.assertEqual(len(info['snapshot']['joint_limits_deg']), 6)
        self.assertIn('urdf', self.bundle.dispatch('model', {}))
        self.assertEqual(self.sdk.calls, [])

    def test_tip_axes_follow_rotation_in_all_six_directions(self):
        # Yaw 90: local +X becomes base +Y, local +Y becomes base -X.
        self.sdk.pose = (200000, 0, 250000, 0, 0, 90000)
        vectors = [([1,0,0], (0,1000,0)), ([-1,0,0], (0,-1000,0)),
                   ([0,1,0], (-1000,0,0)), ([0,-1,0], (1000,0,0)),
                   ([0,0,1], (0,0,1000)), ([0,0,-1], (0,0,-1000))]
        for requested, offset in vectors:
            with self.subTest(requested=requested):
                self.sdk.calls.clear()
                self.sdk.pose = (200000, 0, 250000, 0, 0, 90000)
                result = self.call('move', translation_mm=json.dumps(requested))
                expected = tuple(a+b for a,b in zip((200000, 0, 250000), offset)) + (0,0,90000)
                self.assertEqual(self.targets(), [expected])
                self.assertEqual(result['state'], 'completed')

    def test_empty_optional_parameters_use_declared_defaults(self):
        self.sdk.pose = (200000, 0, 250000, 0, 0, 0)
        self.call('move', translation_mm='', rotation_deg='', speed_percent='', width_mm=50)
        self.assertEqual(self.targets(), [(201000,0,250000,0,0,0)])
        self.assertEqual([c[3] for c in self.sdk.calls if c[0] == 'mode'], [1])

    def test_nullable_absolute_joints_keep_initial_feedback(self):
        result = self.call('set_joints', joints_deg='[null,11,null,null,null,null]')
        self.assertEqual(self.targets('joints'), [(1000,11000,-10000,0,20000,0)])
        self.assertEqual(result['target'], (1,11,-10,0,20,0))
        self.assertEqual(result['reference_frame'], 'joint_angles')
        self.assertEqual(result['motion_mode'], 'joint')

    def test_reset_uses_official_zero_and_preserves_old_saved_file(self):
        from pathlib import Path
        old = Path(self.tmp.name) / 'return-preset.json'
        old.write_text('{invalid legacy data, deliberately retained')
        before = old.read_bytes()
        self.config['intent_control']['preset_joints_deg'] = [2, 11, -12, 1, 21, 1]
        result = self.call('reset')
        self.assertEqual(result['target'], (0.,) * 6)
        self.assertEqual(self.targets('joints'), [(0,) * 6])
        self.assertEqual(result['return_position']['source'], 'official_zero')
        self.assertEqual(result['return_position']['physical_acceptance'], 'pending')
        self.assertEqual(old.read_bytes(), before)
        self.assertFalse(any(c[0] in {'reset', 'gripper', 'teach', 'enable_arm'} for c in self.sdk.calls))

    def test_fixed_reset_survives_restart_and_ignores_preset_configuration(self):
        self.runtime.stop()
        self.config['intent_control']['preset_joints_deg'] = None
        plugins = build_plugins(self.config, 'piper_arm1', None)
        try:
            self.assertEqual(plugins[0].return_preset()['joints_deg'], [0.] * 6)
            labels = plugins[0].get_tools()[0]['inputSchema']['properties']['action']['oneOf']
            self.assertIn({'const': 'reset', 'title': 'Reset'}, labels)
        finally:
            for plugin in plugins: plugin.stop()

    def test_reset_exits_teaching_then_moves_to_official_zero(self):
        self.sdk.mode = 'TEACHING_MODE(0x2)'
        original = tuple(self.sdk.raw)
        result = self.call('reset')
        self.assertEqual(result['target'], (0.,) * 6)
        self.assertEqual([c[2] for c in self.sdk.calls if c[0] == 'mode'], [1, 1])
        self.assertTrue(all(target == original for target in self.targets('joints')[:-1]))
        self.assertEqual(self.targets('joints')[-1], (0,) * 6)
        self.assertFalse(any(c[0] in {'gripper', 'reset'} for c in self.sdk.calls))

    def test_reset_respects_effective_joint_limits_without_fallback(self):
        with patch.object(self.driver, 'joint_bounds_deg', return_value=[(0.5, 150), (0, 180), (-170, 0), (-100, 100), (-70, 70), (-120, 120)]):
            with self.assertRaisesRegex(RuntimeError, 'within'):
                self.call('reset')
        self.assertEqual(self.sdk.calls, [])

    def test_legacy_capture_helper_is_retained_without_a_public_action(self):
        record = fast_capture(self.driver, interface='can0', serial='fake-arm1')
        self.assertEqual(record['joints_deg'], tuple(q / 1000 for q in self.sdk.raw))
        self.assertEqual(self.sdk.calls, [])
        with self.assertRaisesRegex(ValueError, 'replaced'):
            self.call('set_position')

    def test_nullable_pose_uses_calibrated_tip_not_sdk_reference(self):
        self.config['intent_control']['tcp_offset_mm_deg'] = [0,0,100,0,0,0]
        self.sdk.pose = (200000,0,250000,0,0,0)
        result = self.call('pose', pose_mm_deg=[201,None,None,None,None,None])
        self.assertEqual(self.targets(), [(201000,0,250000,0,0,0)])
        self.assertEqual(tuple(result['tip_target_mm_deg']), (201,0,350,0,0,0))
        self.assertEqual(result['motion_mode'], 'linear')
        self.assertEqual([c[2] for c in self.sdk.calls if c[0]=='mode'], [2])

    def test_null_all_empty_and_relative_null_rejected_without_commands(self):
        cases = [('pose', {'pose_mm_deg':[None]*6}), ('pose', {'pose_mm_deg':''}),
                 ('set_joints', {'joints_deg':[None]*6}), ('set_joints', {'joints_deg':[]}),
                 ('set_joints', {'joints_deg':[None,1,0,0,0,0], 'joint_mode':'relative'}),
                 ('move', {'translation_mm':[None,0,0]}), ('move', {'rotation_deg':[0,None,0]}),
                 ('pose', {'pose_mm_deg':[None,True,None,None,None,None]}),
                 ('pose', {'pose_mm_deg':[None,float('nan'),None,None,None,None]})]
        for action, args in cases:
            with self.subTest(action=action,args=args), self.assertRaises(ValueError):
                self.call(action, **args)
            self.assertEqual(self.sdk.calls, [])

    def test_noop_does_not_connect_prepare_enable_or_require_motion_gate(self):
        self.runtime.motion_enabled = False
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,)*6
        with patch.object(self.runtime, 'connected', side_effect=AssertionError('No connection allowed')):
            for action, args in [('move', {'translation_mm':[0,0,0]}),
                                 ('set_joints', {'joint_mode':'relative','joints_deg':[0]*6})]:
                with self.subTest(action=action):
                    result = self.call(action, **args)
                    self.assertFalse(result['executed'])
                    self.assertEqual(result['state'], 'completed')
                    self.assertEqual(result['message'], 'No movement requested.')
        self.assertEqual(self.sdk.calls, [])

    def test_nullable_target_not_recomputed_after_prepare(self):
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,)*6
        prepare = self.driver.prepare_control
        def drift(**kwargs):
            result=prepare(**kwargs)
            self.sdk.raw=(1000,12000,*self.sdk.raw[2:])
            return result
        self.driver.prepare_control=drift
        with self.assertRaisesRegex(RuntimeError, 'moved during preparation'):
            self.call('set_joints', joints_deg=[2,None,None,None,None,None])
        self.assertEqual(self.targets('joints'), [])
        self.assertEqual(self.runtime.task['diagnostics']['joint_target_deg'], (2,10,-10,0,20,0))

    def test_capability_schema_hides_unconfigured_actions_and_rotation(self):
        self.config['intent_control'].update(tcp_verified=False,preset_joints_deg=None)
        tool=self.runtime.get_tools()[0]
        schema=tool['inputSchema']
        self.assertNotIn('pose',schema['properties']['action']['enum'])
        self.assertIn('reset',schema['properties']['action']['enum'])
        self.assertNotIn('pose_mm_deg',schema['properties'])
        self.assertNotIn('rotation_deg',schema['properties'])
        self.assertNotIn('motion_mode',schema['properties'])
        self.assertNotIn('pose',schema['x-completion']['actions'])
        self.assertNotIn('Return to preset',json.dumps(tool))
        self.assertEqual(schema['x-action-params']['move']['params'],['translation_mm','speed_percent'])
        caps=self.call('get_status')['snapshot']['capabilities']
        self.assertFalse(caps['pose']['available'])
        self.assertIn('Calibrate',caps['pose']['reason'])
        self.assertEqual(self.sdk.calls, [])
        self.config['intent_control'].update(tcp_verified=True,preset_joints_deg=[1,10,-10,0,20,0])
        schema=self.runtime.get_tools()[0]['inputSchema']
        self.assertIn('pose',schema['properties']['action']['enum'])
        self.assertIn('reset',schema['properties']['action']['enum'])
        self.assertIn('rotation_deg',schema['properties'])

    def test_malformed_calibration_does_not_advertise_tip_capability(self):
        self.config['intent_control']['tcp_offset_mm_deg']=None
        self.assertNotIn('pose',self.runtime.get_tools()[0]['inputSchema']['properties']['action']['enum'])
        self.assertFalse(self.call('get_status')['snapshot']['tcp_verified'])

    def test_hidden_legacy_zero_rotation_is_accepted_nonzero_never_discarded(self):
        self.config['intent_control']['tcp_verified']=False
        self.call('move',translation_mm=[1,0,0],rotation_deg='[0,0,0]')
        self.assertEqual(len(self.targets()),1)

    def accepted_path(self):
        record={'accepted':True,'device_serial':'fake-arm1','firmware':self.driver.firmware(),
                'driver_sha256':driver_digest(),'configuration_sha256':configuration_digest(self.config),
                'evidence':'simulated fixture only; not hardware acceptance'}
        self.config['intent_control']['point_to_point_acceptance']=record
        mock=patch('agilex.piper.unified_control.adapter_serial',return_value='fake-arm1')
        mock.start();self.addCleanup(mock.stop)
        return record

    def test_point_to_point_default_closed_and_incomplete_record_rejected(self):
        self.assertEqual(self.runtime.get_tools()[0]['inputSchema']['properties']['motion_mode']['enum'],['linear'])
        with self.assertRaisesRegex(RuntimeError,'supervised hardware acceptance'):
            self.call('pose',pose_mm_deg=[201,0,250,0,0,0],motion_mode='point_to_point')
        self.assertEqual(self.sdk.calls,[])
        self.config['intent_control']['point_to_point_acceptance']={'accepted':True}
        self.assertFalse(self.runtime.capabilities()['point_to_point']['available'])

    def test_path_acceptance_binds_device_firmware_driver_and_configuration(self):
        record=self.accepted_path()
        self.assertTrue(self.runtime.capabilities()['point_to_point']['available'])
        for key in ('device_serial','firmware','driver_sha256','configuration_sha256','evidence'):
            with self.subTest(key=key):
                value=record[key];record[key]=''
                self.assertFalse(self.runtime.capabilities()['point_to_point']['available'])
                record[key]=value
        self.config['intent_control']['tcp_offset_mm_deg']=[0,0,10,0,0,0]
        self.assertFalse(self.runtime.capabilities()['point_to_point']['available'])
        self.assertEqual(self.sdk.calls,[])

    def test_accepted_point_to_point_uses_move_p_end_pose_never_move_j(self):
        self.accepted_path()
        self.assertEqual(self.runtime.get_tools()[0]['inputSchema']['properties']['motion_mode']['enum'],['linear','point_to_point'])
        result=self.call('pose',pose_mm_deg=[201,0,250,0,0,0],motion_mode='point_to_point')
        self.assertEqual([c[2] for c in self.sdk.calls if c[0]=='mode'],[0])
        self.assertEqual(self.targets(),[(201000,0,250000,0,0,0)])
        self.assertEqual(self.targets('joints'),[])
        self.assertEqual(result['motion_mode'],'point_to_point')

    def test_path_rejection_is_not_retried_or_switched_to_linear(self):
        self.accepted_path()
        submit=self.sdk.EndPoseCtrl
        def reject(*target):
            submit(*target);self.sdk.status='TARGET_POS_EXCEEDS_LIMIT(0x4)'
        self.sdk.EndPoseCtrl=reject
        with self.assertRaisesRegex(RuntimeError,'outside its limits'):
            self.call('pose',pose_mm_deg=[201,0,250,0,0,0],motion_mode='point_to_point')
        self.assertEqual([c[2] for c in self.sdk.calls if c[0]=='mode'],[0])
        self.assertEqual(len(self.targets()),1)

    def test_point_to_point_timeout_holds_with_verified_linear_path(self):
        self.accepted_path();self.sdk.follow_pose=False
        with self.assertRaisesRegex(RuntimeError,'timed out'):
            self.call('pose',pose_mm_deg=[201,0,250,0,0,0],motion_mode='point_to_point')
        self.assertEqual([c[2] for c in self.sdk.calls if c[0]=='mode'],[0,2])
        self.assertEqual(len(self.targets()),2)
        self.assertEqual(self.targets()[-1],self.sdk.pose)

    def test_path_revoked_between_mode_and_target_blocks_original_goal(self):
        record=self.accepted_path()
        mode=self.sdk.MotionCtrl_2
        def revoke(*args):
            mode(*args);record['accepted']=False
        self.sdk.MotionCtrl_2=revoke
        with self.assertRaisesRegex(RuntimeError,'supervised hardware acceptance'):
            self.call('pose',pose_mm_deg=[201,0,250,0,0,0],motion_mode='point_to_point')
        self.assertEqual(self.targets(),[self.sdk.pose])
        self.assertEqual([c[2] for c in self.sdk.calls if c[0]=='mode'],[0,2])

    def test_internal_path_without_acceptance_callback_sends_nothing(self):
        target,_=target_pose(self.driver,'pose',{'pose_mm_deg':[201,0,250,0,0,0]},self.config)
        with self.assertRaisesRegex(RuntimeError,'supervised hardware acceptance'):
            fast_execute(self.driver,target,1,'pose',require_permission=lambda:None,motion_mode='point_to_point')
        self.assertEqual(self.sdk.calls,[])

    def test_pose_and_tcp_offset_rotation_keep_tip_fixed(self):
        self.config['intent_control']['tcp_offset_mm_deg'] = [0, 0, 100, 0, 0, 0]
        self.sdk.pose = (200000,0,250000,0,0,0)
        self.call('pose', pose_mm_deg=[200,0,350,0,0,0])
        self.assertEqual(self.targets(), [(200000,0,250000,0,0,0)])
        self.sdk.calls.clear()
        result = self.call('move', translation_mm=[0,0,0], rotation_deg=[0,5,0])
        # Rotation around the tip changes the SDK position, preserving the tip.
        target = self.targets()[0]
        sdk = Transform.from_pose([v/1000 for v in target])
        tip = sdk.compose(Transform.from_pose([0,0,100,0,0,0])).values()
        self.assertLess(math.dist(tip[:3], (200,0,350)), .001)
        self.assertLess(orientation_error(tip[3:], (0,5,0)), .001)
        self.assertLess(math.dist(result['tip_target_mm_deg'][:3], (200,0,350)), .001)

    def test_joint_absolute_relative_and_preset_targets(self):
        self.call('set_joints', joints_deg='[2,11,-11,1,21,1]')
        self.assertEqual(self.targets('joints'), [(2000,11000,-11000,1000,21000,1000)])
        self.sdk.calls.clear()
        self.call('set_joints', joint_mode='relative', joints_deg=[1,1,-1,1,1,1])
        self.assertEqual(self.targets('joints'), [(3000,12000,-12000,2000,22000,2000)])
        self.sdk.calls.clear()
        self.call('reset')
        self.assertEqual(self.targets('joints'), [(0,0,0,0,0,0)])

    def test_invalid_arrays_ranges_and_unknown_fields_never_send(self):
        cases = [('move', {'translation_mm': [True,0,0]}), ('move', {'translation_mm': [1,2]}),
                 ('move', {'translation_mm': '[1,"2",3]'}), ('move', {'translation_mm': '1,2,3'}),
                 ('move', {'translation_mm': [float('nan'),0,0]}), ('move', {'translation_mm': [11,0,0]}),
                 ('move', {'rotation_deg': [0,6,0]}), ('move', {'speed_percent': 0}),
                 ('move', {'speed_percent': True}), ('move', {'mystery': 1}),
                 ('pose', {}), ('pose', {'pose_mm_deg': [1,2,3,0,181,0]}),
                 ('pose', {'pose_mm_deg': [10**10,2,3,0,0,0]}),
                 ('pose', {'pose_mm_deg': [1e308,2,3,0,0,0]}),
                 ('move', {'speed_percent': 10**400}),
                 ('set_joints', {'joints_deg': [151,10,-10,0,0,0]}),
                 ('set_joints', {'joint_mode': 'relative', 'joints_deg': [150,0,0,0,0,0]})]
        for action, args in cases:
            with self.subTest(action=action, args=args), self.assertRaises((ValueError, RuntimeError)):
                self.call(action, **args)
            self.assertEqual(self.sdk.calls, [])

    def test_unconfigured_tip_blocks_before_preparation(self):
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,)*6
        self.config['intent_control']['tcp_verified'] = False
        with self.assertRaisesRegex(RuntimeError, 'not calibrated'):
            self.call('pose', pose_mm_deg=[200,0,250,0,0,0])
        with self.assertRaisesRegex(RuntimeError, 'rotation requires TCP calibration'):
            self.call('move', rotation_deg=[1,0,0])
        self.config['intent_control']['preset_joints_deg'] = None
        self.assertTrue(self.runtime.capabilities()['return_preset']['available'])
        self.assertEqual(self.sdk.calls, [])

    def test_uncalibrated_translation_preserves_sdk_axes_and_orientation(self):
        self.config['intent_control']['tcp_verified'] = False
        self.config['intent_control']['tcp_offset_mm_deg'] = None
        self.sdk.pose = (200000,0,250000,0,0,90000)
        result = self.call('move', translation_mm=[1,0,0])
        self.assertEqual(self.targets(), [(200000,1000,250000,0,0,90000)])
        self.assertEqual(result['state'], 'completed')
        self.assertEqual(result['reference_frame'], 'sdk_end_axes')
        self.assertFalse(result['tcp_verified'])
        self.assertNotIn('tip_target_mm_deg', result)
        self.assertNotIn('actual_tip_mm_deg', result)
        self.assertIsNone(self.call('get_status')['snapshot']['tip_pose_mm_deg'])

    def test_parallel_tip_translation_is_independent_of_unknown_tool_length(self):
        from agilex.piper.user_motion import target_pose
        args={'translation_mm':[1,2,3],'rotation_deg':[0,0,0]}
        self.sdk.pose = (200000,10000,250000,103579,76749,102160)
        self.config['intent_control']['tcp_verified'] = False
        uncalibrated, tip = target_pose(self.driver, 'move', args, self.config)
        self.assertIsNone(tip)
        self.assertEqual(uncalibrated.orientation_deg, (103.579,76.749,102.16))
        for length in (0,145,200):
            with self.subTest(length=length):
                self.config['intent_control'].update(tcp_verified=True,
                    tcp_offset_mm_deg=[0,0,length,0,0,0])
                calibrated, tip = target_pose(self.driver, 'move', args, self.config)
                self.assertLess(math.dist(calibrated.position_mm, uncalibrated.position_mm), 1e-8)
                self.assertEqual(calibrated.orientation_deg, uncalibrated.orientation_deg)

    def test_uncalibrated_nonzero_rotation_is_never_silently_discarded(self):
        self.config['intent_control']['tcp_verified'] = False
        for axis in range(3):
            angles=[0,0,0];angles[axis]=.001
            with self.subTest(axis=axis), self.assertRaisesRegex(RuntimeError,'requires TCP calibration'):
                self.call('move', rotation_deg=angles)
        self.assertEqual(self.sdk.calls, [])

    def test_ready_not_reprepared_and_disabled_prepared_once(self):
        self.call('move')
        self.assertNotIn('prepare', [c[0] for c in self.sdk.calls])
        self.sdk.calls.clear()
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,)*6
        self.call('move')
        names = [c[0] for c in self.sdk.calls]
        self.assertEqual(names.count('prepare'), 1)
        self.assertLess(names.index('prepare'), names.index('end_pose'))

    def test_teaching_auto_exit_holds_before_target(self):
        self.sdk.mode = 'TEACHING_MODE(0x2)'
        self.call('move')
        names = [c[0] for c in self.sdk.calls]
        self.assertLess(names.index('firmware_query'), names.index('joints'))
        self.assertLess(names.index('joints'), names.index('end_pose'))
        self.assertFalse(any(c[0] in {'enable_arm', 'prepare', 'teach'} for c in self.sdk.calls))

    def test_teaching_exit_precedes_target_composition_and_uses_post_exit_pose(self):
        self.sdk.mode='TEACHING_MODE(0x2)'
        exit_result=fast_exit
        original_target=target_pose
        order=[]
        def leave(*args,**kwargs):
            result=exit_result(*args,**kwargs)
            # Model a permitted final measured correction during position hold.
            self.sdk.pose=(210000,0,250000,0,0,0)
            order.append('exit_confirmed')
            return result
        def compose(*args,**kwargs):
            order.append('target_composed')
            self.assertTrue(self.sdk.mode.startswith('CAN_CTRL'))
            return original_target(*args,**kwargs)
        with patch('agilex.piper.unified_control.exit_teaching_and_hold',side_effect=leave), \
             patch('agilex.piper.unified_control.target_pose',side_effect=compose):
            result=self.call('move',translation_mm=[1,0,0])
        self.assertEqual(order,['exit_confirmed','target_composed'])
        self.assertEqual(self.targets(),[(211000,0,250000,0,0,0)])
        self.assertTrue(self.runtime.task['preparation']['teaching_exit']['executed'])
        self.assertEqual(result['state'],'completed')

    def test_every_control_action_takes_over_teaching_before_its_target(self):
        cases=[('move',{'translation_mm':[1,0,0]}),
               ('pose',{'pose_mm_deg':[201,0,250,0,0,0]}),
               ('set_joints',{'joints_deg':[None,11,None,None,None,None]}),
               ('reset',{}),
               ('set_gripper_opening',{'width_mm':25})]
        for action,args in cases:
            with self.subTest(action=action):
                self.sdk.calls.clear();self.sdk.mode='TEACHING_MODE(0x2)'
                result=self.call(action,**args)
                self.assertEqual(result['state'],'completed')
                names=[call[0] for call in self.sdk.calls]
                self.assertEqual(names.count('firmware_query'),1)
                self.assertLess(names.index('firmware_query'),names.index('joints'))
                self.assertEqual([c[2] for c in self.sdk.calls if c[0]=='mode'][0],1)
                if 'gripper' in names:self.assertLess(names.index('joints'),names.index('gripper'))
                if 'end_pose' in names:self.assertLess(names.index('joints'),names.index('end_pose'))

    def test_failed_exit_never_resolves_or_sends_following_user_target(self):
        self.sdk.mode='TEACHING_MODE(0x2)'
        with patch('agilex.piper.unified_control.exit_teaching_and_hold',side_effect=RuntimeError('Exit not confirmed')), \
             patch('agilex.piper.unified_control.target_pose',side_effect=AssertionError('Target must wait')):
            with self.assertRaisesRegex(RuntimeError,'Exit not confirmed'):
                self.call('move')
        self.assertEqual(self.sdk.calls,[])

    def test_passive_core_start_info_and_stop_do_not_take_over_teaching(self):
        self.sdk.mode='TEACHING_MODE(0x2)'
        for action in ('start','get_status','info','select_action','stop_motion'):
            with self.subTest(action=action):self.call(action)
        self.assertEqual(self.sdk.calls,[])
        self.assertTrue(self.sdk.mode.startswith('TEACHING_MODE'))

    def test_stop_during_teaching_exit_blocks_following_target_and_releases_task(self):
        self.sdk.mode='TEACHING_MODE(0x2)'
        original=self.sdk.MotionCtrl_2
        def cancel(*args):
            original(*args);self.call('stop_motion')
        self.sdk.MotionCtrl_2=cancel
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            self.call('move')
        self.assertEqual(self.targets(),[])
        self.assertFalse(self.runtime.gate.locked())
        self.assertEqual(self.call('get_status')['state'],'cancelled')

    def test_unknown_firmware_and_recording_never_take_over(self):
        self.sdk.mode = 'TEACHING_MODE(0x2)'
        with patch.object(self.driver, 'firmware', return_value='unknown'), self.assertRaisesRegex(RuntimeError, 'firmware'):
            self.call('move')
        self.assertEqual(self.sdk.calls, [])
        self.sdk.teach = 'START_RECORDING(0x1)'
        with self.assertRaisesRegex(RuntimeError, 'Teaching'):
            self.call('move')
        self.assertEqual(self.sdk.calls, [])

    def test_fault_partial_enable_and_stale_frames_never_send(self):
        for change in ('fault', 'partial', 'stale_end'):
            with self.subTest(change=change):
                self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)' if change == 'fault' else 'NORMAL(0x0)'
                self.sdk.enabled = (True,False,True,True,True,True) if change == 'partial' else (True,)*6
                self.sdk.pose_frozen = change == 'stale_end'
                with self.assertRaises(RuntimeError):
                    self.call('move')
                self.assertEqual(self.sdk.calls, [])

    def test_permission_revoked_during_mode_send_blocks_target_and_hold(self):
        mode = self.sdk.MotionCtrl_2
        def revoke(*args):
            mode(*args)
            self.runtime.motion_enabled = False
        self.sdk.MotionCtrl_2 = revoke
        with self.assertRaisesRegex(RuntimeError, 'Motion is disabled'):
            self.call('move')
        self.assertEqual(self.targets(), [])
        self.assertEqual(self.targets('joints'), [])

    def test_permission_revoked_in_prepare_feedback_blocks_all_commands(self):
        from agilex.piper.arm import PiperDriver
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,)*6
        self.driver.prepare_control = lambda **kw: PiperDriver.prepare_control(self.driver, **kw)
        healthy = self.driver._healthy
        def revoke():
            value = healthy()
            self.runtime.motion_enabled = False
            return value
        self.driver._healthy = revoke
        with self.assertRaisesRegex(RuntimeError, 'Motion is disabled'):
            self.call('move')
        self.assertEqual(self.sdk.calls, [])

    def test_stop_marked_in_mode_send_cannot_submit_original_target(self):
        mode = self.sdk.MotionCtrl_2
        original_pose = self.sdk.pose
        def cancel(*args):
            mode(*args)
            self.runtime.cancel.set()
        self.sdk.MotionCtrl_2 = cancel
        with self.assertRaisesRegex(RuntimeError, 'cancelled'):
            self.call('move')
        self.assertEqual(self.targets(), [original_pose])  # Measured hold only.
        self.assertEqual(self.call('get_status')['state'], 'cancelled')

    def test_controller_reject_after_submission_stops_without_reset_or_retry(self):
        submit = self.sdk.EndPoseCtrl
        def reject(*args):
            submit(*args)
            self.sdk.status = 'TARGET_POS_EXCEEDS_LIMIT(0x4)'
        self.sdk.EndPoseCtrl = reject
        with self.assertRaisesRegex(RuntimeError, 'outside its limits'):
            self.call('move')
        self.assertEqual(len(self.targets()), 1)
        self.assertFalse(any(c[0] in {'reset','enable_arm','prepare','teach','joints'} for c in self.sdk.calls))
        self.assertEqual(self.call('get_status')['state'], 'failed')

    def test_failed_preparation_does_not_submit_target(self):
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,)*6
        self.driver.prepare_control = lambda **kw: (_ for _ in ()).throw(RuntimeError('Preparation timed out.'))
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('move')
        self.assertEqual(self.sdk.calls, [])

    def test_post_prepare_drift_does_not_recompute_relative_target(self):
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,)*6
        prepare = self.driver.prepare_control
        def drift(**kwargs):
            result = prepare(**kwargs)
            self.sdk.raw = (2000,*self.sdk.raw[1:])
            return result
        self.driver.prepare_control = drift
        with self.assertRaisesRegex(RuntimeError, 'moved during preparation'):
            self.call('set_joints', joint_mode='relative', joints_deg=[1,0,0,0,0,0])
        self.assertEqual(self.targets('joints'), [])

    def test_stop_and_concurrent_request_do_not_submit_old_target(self):
        entered, release = threading.Event(), threading.Event()
        prepare = self.driver.prepare_control
        def waiting(**kwargs):
            entered.set(); release.wait(2)
            return prepare(**kwargs)
        self.sdk.mode, self.sdk.enabled = 'STANDBY(0x0)', (False,)*6
        self.driver.prepare_control = waiting
        self.runtime.config['async_actions'] = True
        result = self.call('move')
        self.assertEqual(result['state'], 'running')
        self.assertTrue(entered.wait(1))
        with self.assertRaises(RuntimeError):
            self.call('reset')
        self.assertEqual(self.call('stop_motion')['state'], 'stopping')
        release.set()
        deadline = time.monotonic()+2
        while self.runtime.gate.locked() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertFalse(self.runtime.gate.locked())
        self.assertEqual(self.targets(), [])
        self.assertEqual(self.call('get_status')['state'], 'cancelled')

    def test_timeout_requires_arrival_and_only_holds_measured_pose(self):
        self.sdk.follow_pose = False
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('move', translation_mm=[10,0,0])
        self.assertEqual(len(self.targets()), 2)  # One goal, one measured hold.
        self.assertEqual(self.targets()[-1], self.sdk.pose)

    def test_joint_near_without_controller_completion_is_not_success(self):
        submit = self.sdk.JointCtrl
        def moving_without_completion(*target):
            submit(*target)
            self.sdk.reached = False
        self.sdk.JointCtrl = moving_without_completion
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('set_joints', joints_deg=[1,10,-10,0,20,0])

    def test_stop_during_movement_holds_without_repeating_goal(self):
        self.sdk.follow_pose = False
        clock = FastClock()
        def sleep(seconds):
            self.call('stop_motion')
            clock.sleep(seconds)
        with patch('agilex.piper.unified_control.execute_target', side_effect=lambda *a, **kw:
                execute_target(*a, **kw, clock=clock.now, sleep=sleep)):
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                self.call('move')
        self.assertEqual(len(self.targets()), 2)
        self.assertEqual(self.targets()[-1], self.sdk.pose)
        self.assertEqual(self.call('get_status')['state'], 'cancelled')
        self.assertFalse(self.runtime.gate.locked())
        self.assertEqual(self.call('stop_motion')['state'], 'stopped')

    def test_stop_during_gripper_holds_measured_opening(self):
        self.sdk.gripper_enabled = True
        self.sdk.follow = False
        with patch('agilex.piper.arm.time.sleep', side_effect=lambda seconds: self.call('stop_motion')):
            with self.assertRaisesRegex(RuntimeError, 'cancelled'):
                self.call('set_gripper_opening', width_mm=50)
        commands = [c[1] for c in self.sdk.calls if c[0] == 'gripper']
        self.assertEqual(commands, [50000,50000,20000])
        self.assertEqual(self.call('get_status')['state'], 'cancelled')

    def test_component_frame_stale_despite_fresh_aggregate_blocks_move(self):
        self.sdk.frame_lock = threading.Lock()
        self.sdk.frame_counts = {}
        self.sdk._arm_can = SimpleNamespace(tx_frames=[])
        self.sdk.frame_times = {0x2A2:time.time(), 0x2A3:time.time()-1, 0x2A4:time.time()}
        with self.assertRaisesRegex(RuntimeError, 'stale'):
            self.call('move')
        self.assertEqual(self.sdk.calls, [])

    def test_pre_submission_cached_arrival_is_not_completion(self):
        original = self.sdk.GetArmEndPoseMsgs
        cached = original()
        self.sdk.GetArmEndPoseMsgs = lambda: cached
        self.sdk.follow_pose = False
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('pose', pose_mm_deg=[200,0,250,180,0,90])

    def test_tip_arrival_uses_rotational_offset_not_only_sdk_reference(self):
        self.config['intent_control']['tcp_offset_mm_deg'] = [0,0,100,0,0,0]
        original = self.sdk.EndPoseCtrl
        def bias(*target):
            original(*target)
            self.sdk.pose = (*target[:4],target[4]+500,target[5])
        self.sdk.EndPoseCtrl = bias
        # SDK reference position is exact and orientation error is only 0.5°,
        # but the actual tip is displaced 0.87 mm: cannot report completion.
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            self.call('pose', pose_mm_deg=[200,0,350,0,0,0])

    def test_independent_joints_and_dual_namespace_isolation(self):
        self.background.start()
        data = self.background.monitor.read()
        self.assertEqual(len(data['skeleton']['joints']), 6)
        self.assertTrue(data['parameters']['Gripper-tip calibrated'])
        self.assertEqual(self.sdk.calls, [])
        plugins = build_plugins({'data_dir': self.tmp.name+'/arm2'}, 'piper_arm2', None)
        try:
            self.assertNotEqual(self.background.get_tools()[0]['topic_out'][0]['topic'],
                                plugins[1].get_tools()[0]['topic_out'][0]['topic'])
        finally:
            for plugin in plugins:
                plugin.stop()

    def test_joints_only_connects_once_without_control_and_survives_arm_card_stop(self):
        self.runtime.stop()
        self.sdk.calls.clear()
        self.background.auto_connect = True
        with patch.object(self.runtime, 'factory', wraps=IntentDriver) as factory:
            self.bundle.dispatch('joints', {'action': 'start'})
            deadline = time.monotonic() + 2
            while self.runtime.driver is None and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertIsNotNone(self.runtime.driver)
            self.bundle.dispatch('joints', {'action': 'start'})
            self.assertEqual(factory.call_count, 1)
            self.assertEqual(self.runtime.driver.arm.calls, [])
            self.call('stop')
            self.assertTrue(self.background.monitor.read()['skeleton'])
            self.assertEqual(self.bundle.dispatch('joints', {'action': 'info'})['state'], 'running')
            self.assertEqual(self.runtime.driver.arm.calls, [])
            self.bundle.dispatch('joints', {'action': 'stop'})
            self.assertEqual(self.background.info()['state'], 'idle')

    def test_joints_rejects_control_parameters_and_unsupported_actions(self):
        for args in ({'action': 'move'}, {'action': 'info', 'width_mm': 20},
                     {'action': 'info', 'input_topic': '/another/device'}):
            with self.subTest(args=args), self.assertRaisesRegex(ValueError, 'Joints'):
                self.bundle.dispatch('joints', args)
        self.assertEqual(self.sdk.calls, [])

    def test_joints_cannot_reopen_a_driver_with_a_stopping_action(self):
        with self.runtime.operation('arm_control', 'move'):
            self.runtime.closed = True
            try:
                with self.assertRaisesRegex(RuntimeError, 'driver is stopping'):
                    self.bundle.dispatch('joints', {'action': 'start'})
                self.assertTrue(self.runtime.closed)
                self.assertIsNone(self.background._connection_thread)
            finally:
                self.runtime.closed = False
        self.assertEqual(self.sdk.calls, [])

    def test_gripper_menu_is_one_width_field_with_effective_bounds(self):
        schema = self.runtime.get_tools()[0]['inputSchema']
        self.assertIn({'const': 'set_gripper_opening', 'title': 'Set gripper'},
                      schema['properties']['action']['oneOf'])
        prop = schema['properties']['width_mm']
        self.assertEqual((prop['minimum'], prop['maximum'], prop['default']), (0, 70, 20))
        for width in (0, 20, 70):
            self.sdk.gripper_enabled = True
            self.sdk.calls.clear()
            result = self.call('set_gripper_opening', width_mm=width)
            self.assertEqual(result['state'], 'completed')
            self.assertEqual(self.sdk.width, width)
        self.sdk.calls.clear()
        for width in (-1, 71, float('nan'), True):
            with self.subTest(width=width), self.assertRaises(ValueError):
                self.call('set_gripper_opening', width_mm=width)
        self.assertEqual(self.sdk.calls, [])


class TransformTests(unittest.TestCase):
    def test_euler_round_trip_and_gimbal_equivalence(self):
        for angles in [(0,0,0), (30,40,50), (-120,-70,150), (35,90,60), (35,-90,60)]:
            with self.subTest(angles=angles):
                self.assertLess(orientation_error(angles, euler(rotation(angles))), 1e-5)

    def test_rotation_composes_in_tip_frame_and_inverse_round_trip(self):
        first = Transform.from_pose([100,200,300,30,40,50])
        second = Transform.from_pose([10,20,30,2,3,4])
        roundtrip = first.compose(second).compose(second.inverse()).values()
        self.assertLess(math.dist(roundtrip[:3], first.position), 1e-8)
        self.assertLess(orientation_error(roundtrip[3:], (30,40,50)), 1e-5)
        composed = first.compose(second).values()[3:]
        self.assertGreater(orientation_error(composed, (32,43,54)), .5)


if __name__ == '__main__':
    unittest.main()
