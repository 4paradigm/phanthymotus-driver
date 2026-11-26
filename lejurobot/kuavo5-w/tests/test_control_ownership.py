"""Cross-card controller handoff and independent-resource regressions."""
import time
import unittest
from unittest.mock import patch

import test_driver as fixtures
from test_driver import driver, wait_until


class ControlOwnershipTests(unittest.TestCase):
    def setUp(self):
        fixtures.DriverContractTests.setUp(self)
        self.auto_mode_patch.stop()
        self.base_mode_patch.stop()
        self.auto_observation_patch = patch.object(self.bundle.spatial.client, 'observe', return_value={
            'agvState': {'runningMode':'AUTO','isMapping':False,'estopButtonEvent':False,
                         'bumperEvent':False,'driverError':False}, 'state':{'driving':False}})
        self.auto_observation_patch.start()
        self.addCleanup(self.auto_observation_patch.stop)

    tearDown = fixtures.DriverContractTests.tearDown
    call = fixtures.DriverContractTests.call

    def reserve(self, tool, channel):
        return self.bundle.start_motion(channel, tool, (3,), lambda: None, notify=False)

    def release_all(self):
        self.bundle.spatial._task = None
        self.bundle.spatial._uncertain = False
        self.bundle.tracker.cancel_all('test_done')

    def test_upper_card_verifies_chassis_route_before_first_target_even_when_upper_enabled(self):
        self.bundle._upper_route_key=None
        calls=[]
        def handoff(operation,**args):
            calls.append(operation)
            self.assertFalse(any(p[0]=='/cmd_vel' and p[2]['linear']['x'] for p in self.ros.published))
            return {'success':True,'stationary':True,'software_estop':False,
                    'velocity_source':'nav_mpc_move_base','delivery_uncertain':False}
        with patch.object(self.bundle.spatial.client,'native',side_effect=handoff):
            lease=self.bundle.start_motion('base','base_drive',(3,),lambda:self.bundle.twist('/cmd_vel',.1))
        self.assertEqual(calls,['upper_handoff'])
        self.assertEqual(lease[-1]['chassis_velocity_source'],'nav_mpc_move_base')
        self.release_all()
        with patch.object(self.bundle.spatial.client,'native',side_effect=AssertionError('already selected')):
            self.reserve('base_drive','base')
        self.release_all()

    def test_cached_upper_route_is_revalidated_when_chassis_changed_to_manual(self):
        self.assertIsNotNone(self.bundle._upper_route_key)
        order, mode = [], ['MANUAL']
        def observe(**kwargs):
            return {'agvState':{'runningMode':mode[0], 'isMapping':False,
                'estopButtonEvent':False,'bumperEvent':False,'driverError':False},
                'state':{'driving':False}}
        def command(method, params, timeout):
            order.append(method)
            self.assertEqual(params, {'mode':'AUTO'})
            mode[0] = 'AUTO'
        def native(operation, **args):
            order.append(operation)
            if operation == 'manual_auto_guard':
                return {'success':True,'stationary':True,'native_tracker_idle':True,'upper_zero':True}
            if operation == 'manual_stop':
                return {'success':True,'stationary':True,'manual_zero_sent':True,
                    'completion_basis':'factory_manual_zero_and_fresh_stationary_odometry'}
            self.assertEqual(mode[0], 'AUTO')
            return {'success':True,'stationary':True,'software_estop':False,
                'velocity_source':'nav_mpc_move_base','delivery_uncertain':False}
        with patch.object(self.bundle.spatial.client, 'observe', side_effect=observe), \
                patch.object(self.bundle.spatial.client, 'command', side_effect=command), \
                patch.object(self.bundle.spatial.client, 'native', side_effect=native):
            self.bundle.start_motion('base','base_drive',(3,),lambda:order.append('target'))
        self.assertEqual(order, ['manual_stop','manual_auto_guard','ChangeMode','upper_handoff','target'])
        self.release_all()

    def test_cached_upper_route_is_invalidated_by_another_selected_source(self):
        self.ros.emit('/twist_mux_new/out/nav_source_used', 'mpc_path_track')
        with patch.object(self.bundle.spatial.client, 'native', return_value={
                'success':True,'stationary':True,'software_estop':False,
                'velocity_source':'nav_mpc_move_base','delivery_uncertain':False}) as native:
            self.reserve('base_servo', 'base')
        native.assert_called_once_with('upper_handoff')
        self.release_all()

    def test_native_navigation_changes_manual_to_auto_with_upper_velocity_disabled(self):
        mode, order = ['MANUAL'], []
        def observe(**kwargs):
            return {'agvState':{'runningMode':mode[0], 'isMapping':False,
                'estopButtonEvent':False,'bumperEvent':False,'driverError':False},
                'state':{'driving':False}}
        def command(method, params, timeout):
            self.assertFalse(self.bundle.state.read('upper_velocity_enabled')[0])
            order.append(method)
            mode[0] = params['mode']
        def native(operation, **args):
            order.append(operation)
            return {'success':True,'stationary':True,'manual_zero_sent':True,
                    'completion_basis':'factory_manual_zero_and_fresh_stationary_odometry',
                    'native_tracker_idle':True,'upper_zero':True}
        with patch.object(self.bundle.spatial.client, 'observe', side_effect=observe), \
                patch.object(self.bundle.spatial.client, 'command', side_effect=command), \
                patch.object(self.bundle.spatial.client, 'native', side_effect=native):
            self.bundle.start_motion('base', 'controlled_spatial', (3,), lambda:order.append('target'))
        self.assertEqual(order, ['manual_stop','manual_auto_guard','ChangeMode','target'])
        self.assertFalse(self.bundle.state.read('upper_velocity_enabled')[0])
        self.release_all()

    def test_manual_startup_recovers_only_a_resolved_preparation_without_enabling(self):
        spatial = self.bundle.spatial
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        self.ros.emit('/enable_vel_control_state', False)
        self.bundle.state.motion_blocked = True
        spatial._preparation = {'delivery_uncertain':False}
        spatial._persist()
        observation = {'agvState':{'runningMode':'MANUAL','isMapping':False,
            'softwareEstop':False,'estopButtonEvent':False,'bumperEvent':False,'driverError':False},
            'state':{'driving':False,'nodeStates':[],'edgeStates':[],'actionStates':[]}}
        calls = []
        def native(operation, **args):
            calls.append(operation)
            if operation == 'held_stationary':
                raise RuntimeError('no software stop in manual')
            if operation == 'manual_stop':
                return {'success':True,'stationary':True,'manual_zero_sent':True,
                    'completion_basis':'factory_manual_zero_and_fresh_stationary_odometry'}
            if operation == 'manual_idle_stationary':
                return {'success':True,'stationary':True,'manual_mode':True,'native_tracker_idle':True}
            self.assertEqual(operation, 'stationary')
            return {'success':True,'stationary':True}
        before = list(self.ros.sent_services)
        with patch.object(spatial.client, 'observe', return_value=observation), \
                patch.object(spatial.client, 'native', side_effect=native), \
                patch.object(spatial.client, 'command', side_effect=AssertionError('no mode/map writes')):
            result = spatial.prepare_chassis()
        self.assertTrue(result['auto_transition_deferred'])
        self.assertIsNone(spatial._preparation)
        self.assertIsNone(spatial.store.state('pending')['chassis_preparation'])
        self.assertIn('manual_stop', calls)
        self.assertIn('manual_idle_stationary', calls)
        self.assertEqual(self.ros.sent_services, before)
        self.assertFalse(self.bundle.state.confirmed_enable())
        self.assertTrue(self.bundle.state.motion_blocked)
        self.assertFalse(spatial.store.state('last_preparation_recovery')['enabled_motion'])

    def test_manual_recovery_never_clears_uncertain_or_owned_operations(self):
        spatial = self.bundle.spatial
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        self.ros.emit('/enable_vel_control_state', False)
        for field, value in (('_preparation',{'delivery_uncertain':True}), ('_task',{'task_id':'old'}),
                             ('_map_pending',{'operation':'stop_mapping'}), ('_keyboard_pending',True)):
            with self.subTest(field=field):
                spatial._preparation = {'delivery_uncertain':False}
                original = getattr(spatial, field)
                setattr(spatial, field, value)
                expected = spatial._preparation
                with patch.object(spatial.client, 'native', side_effect=AssertionError('no RPC')), \
                        patch.object(spatial.client, 'observe', side_effect=AssertionError('no IO')):
                    self.assertFalse(spatial._recover_manual_preparation(self.bundle._stop_epoch))
                self.assertEqual(spatial._preparation, expected)
                setattr(spatial, field, original)
        spatial._preparation = None

    def test_manual_recovery_stop_supersedes_zero_ack_before_clearing_record(self):
        spatial = self.bundle.spatial
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        self.ros.emit('/enable_vel_control_state', False)
        spatial._preparation = {'delivery_uncertain':False}
        observation = {'agvState':{'runningMode':'MANUAL','isMapping':False,
            'estopButtonEvent':False,'bumperEvent':False,'driverError':False},
            'state':{'driving':False,'nodeStates':[],'edgeStates':[],'actionStates':[]}}
        def zero(*args, **kwargs):
            self.bundle._stop_epoch += 1
            self.bundle._transition['cancelled'].set()
            return {'success':True,'stationary':True,'manual_zero_sent':True,
                'completion_basis':'factory_manual_zero_and_fresh_stationary_odometry'}
        with patch.object(spatial.client, 'observe', return_value=observation), \
                patch.object(spatial.client, 'native', side_effect=zero):
            with self.assertRaises(InterruptedError):
                spatial._recover_manual_preparation(self.bundle._stop_epoch)
        self.assertEqual(spatial._preparation, {'delivery_uncertain':False})
        self.assertIsNone(spatial.store.state('last_preparation_recovery'))
        spatial._preparation = None

    def test_native_navigation_does_not_start_upper_bridge(self):
        with patch.object(self.bundle.upper_bridge,'ensure',side_effect=AssertionError('native navigation must not start upper bridge')):
            self.reserve('controlled_spatial','base')
        self.release_all()

    def test_upper_handoff_disables_old_velocity_before_resuming_body(self):
        self.bundle._upper_route_key=None
        self.ros.emit('/enable_control_state',False)
        self.bundle.state.verify_enable()
        self.bundle.state.motion_blocked=True
        before=len(self.ros.sent_services)
        self.reserve('base_drive','base')
        services=[(name,args) for name,_,args in self.ros.sent_services[before:]]
        disabled=services.index(('/enable_vel_control',{'data':False}))
        resumed=services.index(('/enable_control',{'data':True}))
        enabled=services.index(('/enable_vel_control',{'data':True}))
        self.assertLess(disabled,resumed)
        self.assertLess(resumed,enabled)
        self.release_all()

    def test_failed_chassis_handoff_does_not_publish_target(self):
        self.bundle._upper_route_key=None
        sent=[]
        def native(operation,**args):
            if operation=='upper_handoff':
                return {'success':False,'delivery_uncertain':False}
            return {'success':True,'stationary':True,'software_estop':True}
        with patch.object(self.bundle.spatial.client,'native',side_effect=native):
            with self.assertRaisesRegex(RuntimeError,'handoff_unconfirmed'):
                self.bundle.start_motion('base','base_drive',(3,),lambda:sent.append(True))
        self.assertFalse(sent)

    def test_native_owner_invalidates_upper_route_for_the_next_upper_action(self):
        self.reserve('base_drive','base');self.release_all()
        self.reserve('controlled_spatial','base');self.release_all()
        self.assertIsNone(self.bundle._upper_route_key)
        with patch.object(self.bundle.spatial.client,'native',return_value={
                'success':True,'stationary':True,'software_estop':False,
                'velocity_source':'nav_mpc_move_base','delivery_uncertain':False}) as native:
            self.reserve('base_drive','base')
        native.assert_called_once_with('upper_handoff')
        self.release_all()

    def test_stop_during_chassis_handoff_cannot_publish_an_old_target(self):
        self.bundle._upper_route_key=None
        sent=[]
        def native(operation,**args):
            self.bundle._stop_epoch+=1
            self.bundle._transition['cancelled'].set()
            return {'success':True,'stationary':True,'software_estop':False,
                    'velocity_source':'nav_mpc_move_base','delivery_uncertain':False}
        with patch.object(self.bundle.spatial.client,'native',side_effect=native):
            with self.assertRaises(InterruptedError):
                self.bundle.start_motion('base','base_drive',(3,),lambda:sent.append(True))
        self.assertFalse(sent)
        self.assertIsNone(self.bundle._upper_route_key)

    def test_every_motion_card_normalizes_all_vendor_modes_to_shared_base_arm(self):
        for tool, channel in (('base_drive', 'base'), ('base_servo', 'base'),
                              ('controlled_spatial', 'base'), ('arm', 'arms'),
                              ('leg', 'lower_body'), ('head', 'head')):
            for mode in range(5):
                with self.subTest(tool=tool, mode=mode):
                    self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', mode)
                    self.ros.sent_services.clear()
                    lease = self.reserve(tool, channel)
                    self.assertEqual(lease[-1]['mode'], 3)
                    changes = [s for s in self.ros.sent_services if s[0] == '/mobile_manipulator_mpc_control']
                    self.assertEqual([s[2]['control_mode'] for s in changes], [] if mode == 3 else [3])
                    self.assertTrue(self.bundle.tracker.current(channel, lease[0]))
                    self.release_all()

    def test_latched_velocity_state_does_not_expire_and_block_a_parallel_arm(self):
        arm = self.reserve('arm', 'arms')
        self.bundle.state._seen['upper_velocity_enabled'] = time.monotonic()-60
        base = self.reserve('base_drive', 'base')
        self.assertFalse(self.ros.sent_services)
        self.assertTrue(self.bundle.tracker.current('arms', arm[0]))
        self.assertTrue(self.bundle.tracker.current('base', base[0]))

    def test_reconnect_drops_latched_velocity_authority_and_requires_new_feedback(self):
        self.ros.connection = (True, 2)
        self.assertIsNone(self.bundle.state.read('upper_velocity_enabled')[0])
        self.bundle.sensors.discover()
        self.ros.raw()
        self.ros.emit('/enable_control_state', True)
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 3)
        self.bundle.state.verify_enable()
        original = self.ros.call_service
        def missing_velocity_feedback(service, *args):
            if service == '/enable_vel_control':
                return {'success': True}
            return original(service, *args)
        with patch.object(self.ros, 'call_service', side_effect=missing_velocity_feedback), \
             patch.object(driver, 'CONTROL_TIMEOUT_S', .03):
            result = self.call('base_drive', 'move', vx=.1, duration=1)
        self.assertEqual(result['state'], 'error')
        self.assertIn('upper_velocity_enabled', result['error'])
        self.assertFalse(any(p[0]=='/cmd_vel' and p[2]['linear']['x'] for p in self.ros.published))

    def test_base_handoffs_preserve_all_independent_resources_in_both_directions(self):
        for tool, enabled in (('controlled_spatial', False), ('base_drive', True), ('base_servo', True)):
            with self.subTest(tool=tool):
                self.ros.emit('/enable_vel_control_state', not enabled)
                leases = {channel: self.reserve(name, channel) for name, channel in
                          (('arm', 'arms'), ('head', 'head'), ('leg', 'lower_body'))}
                original = self.ros.wait_service
                def wait(pending, timeout, cancelled=None):
                    # Independent workers must remain authorized during the service wait.
                    self.bundle.state.require_motion((3,))
                    return original(pending, timeout, cancelled)
                with patch.object(self.ros, 'wait_service', side_effect=wait):
                    base = self.reserve(tool, 'base')
                self.assertEqual(base[-1]['mode'], 3)
                self.assertIs(self.bundle.state.read('upper_velocity_enabled')[0], enabled)
                for channel, lease in leases.items():
                    self.assertTrue(self.bundle.tracker.current(channel, lease[0]))
                self.assertFalse(any(s[0] in ('/enable_control', '/mobile_manipulator_mpc_control')
                                     for s in self.ros.sent_services))
                self.release_all()
                self.ros.sent_services.clear()

    def test_confirmed_native_navigation_allows_independent_cards_without_taking_base_back(self):
        nav = self.reserve('controlled_spatial', 'base')
        spatial = self.bundle.spatial
        spatial._task = {'action_id': nav[0], 'task_id': 'native-1'}
        self.ros.sent_services.clear()
        try:
            for name, channel in (('arm', 'arms'), ('head', 'head'), ('leg', 'lower_body')):
                self.reserve(name, channel)
            self.assertFalse(self.ros.sent_services)
            self.assertIs(self.bundle.state.read('upper_velocity_enabled')[0], False)
            owners = self.bundle.control_ownership()
            self.assertEqual(owners['active_resources'], {'base': 'controlled_spatial',
                             'arms': 'arm', 'head': 'head', 'lower_body': 'leg'})
            self.assertTrue(self.bundle.tracker.current('base', nav[0]))
        finally:
            self.release_all()

    def test_real_card_workers_can_run_alongside_confirmed_native_navigation(self):
        import tempfile
        from controlled_spatial import SpatialStore
        from test_controlled_spatial import Client
        with tempfile.TemporaryDirectory() as folder:
            self.bundle.spatial.client = client = Client()
            self.bundle.spatial.store = SpatialStore(folder+'/db')
            client.check = {'success': True, 'status': 1, 'arrived': False}
            nav = self.call('controlled_spatial', 'navigate_to_pose', x=1, y=2, yaw=.3)
            self.assertEqual(nav['state'], 'executing')
            self.assertTrue(wait_until(lambda: (self.bundle.spatial._task or {}).get('task_id')))
            results = [self.call('arm', 'set', left_joint_2_rad=.2),
                       self.call('head', 'move_pos', yaw=5),
                       self.call('leg', 'set', waist_yaw_deg=5)]
            self.assertTrue(all(r['state']=='executing' for r in results), results)
            time.sleep(.15)
            self.assertFalse(self.callbacks)
            self.assertTrue(self.bundle.tracker.current('base', nav['action_id']))
            self.assertIs(self.bundle.state.read('upper_velocity_enabled')[0], False)
            self.assertFalse(any(s[0] in ('/enable_control', '/mobile_manipulator_mpc_control')
                                 for s in self.ros.sent_services))
            self.assertEqual(self.call('controlled_spatial', 'stop_nav')['state'], 'paused')

    def test_uncertain_or_orphan_native_navigation_still_blocks_new_motion_without_pausing(self):
        spatial = self.bundle.spatial
        for uncertain, active, task_id in ((True, True, 'native-1'), (False, False, 'native-1'),
                                           (False, True, None)):
            with self.subTest(uncertain=uncertain, active=active, task_id=task_id):
                nav = self.reserve('controlled_spatial', 'base')
                spatial._task = {'action_id': nav[0], 'task_id': task_id}
                spatial._uncertain = uncertain
                if not active:
                    self.bundle.tracker.cancel_all('orphan')
                before = list(self.ros.sent_services)
                try:
                    with self.assertRaisesRegex(RuntimeError, 'native navigation'):
                        self.reserve('head', 'head')
                    self.assertEqual(self.ros.sent_services, before)
                    self.assertFalse(self.bundle.state.motion_blocked)
                finally:
                    self.release_all()

    def test_base_only_startup_and_mapping_handoff_do_not_pause_independent_actions(self):
        arm = self.reserve('arm', 'arms')
        token, _, _ = self.bundle._begin_control('prepare_chassis')
        try:
            self.bundle.state.require_motion((3,))
            self.assertTrue(self.bundle.tracker.current('arms', arm[0]))
        finally:
            self.bundle._end_control(token)
        self.ros.emit('/enable_vel_control_state', False)
        self.bundle.prepare_mapping_velocity(self.bundle._stop_epoch)
        self.bundle.state.require_motion((3,))
        self.assertTrue(self.bundle.tracker.current('arms', arm[0]))
        self.assertIs(self.bundle.state.read('upper_velocity_enabled')[0], False)

    def test_mapping_route_cannot_override_uncertain_navigation(self):
        self.bundle.spatial._uncertain = True
        before = list(self.ros.sent_services)
        try:
            with self.assertRaisesRegex(RuntimeError, 'native navigation'):
                self.bundle.prepare_mapping_velocity(self.bundle._stop_epoch)
            self.assertEqual(self.ros.sent_services, before)
        finally:
            self.bundle.spatial._uncertain = False

    def test_an_external_velocity_handoff_aborts_base_drive_before_more_velocity_commands(self):
        result = self.call('base_drive', 'move', vx=.1, duration=1)
        self.assertEqual(result['state'], 'executing')
        self.ros.emit('/enable_vel_control_state', False)
        self.assertTrue(wait_until(lambda: bool(self.callbacks)))
        self.assertEqual(self.callbacks[0][2], 'error')
        self.assertIn('ownership changed', self.callbacks[0][3]['reason'])
        self.assertIs(self.bundle.state.read('control_enabled')[0], False)

    def test_ownership_info_is_read_only_for_all_motion_cards(self):
        for name in ('base_drive', 'base_servo', 'controlled_spatial', 'arm', 'leg', 'head'):
            # Spatial info reads the factory web state; the ownership itself must not perform I/O.
            if name == 'controlled_spatial':
                info = self.bundle.control_ownership()
            else:
                info = self.call(name, 'info')['control_ownership']
            self.assertEqual(info['mode_policy'], 'BaseArm')
            self.assertEqual(info['required_mode'], 3)
        self.assertFalse(self.ros.sent_services)
        self.assertFalse(self.ros.published)


if __name__ == '__main__':
    unittest.main()
