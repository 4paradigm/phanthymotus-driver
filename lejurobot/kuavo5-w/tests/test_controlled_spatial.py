"""Native task ownership, map identity and interruption regressions (no hardware)."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from controlled_spatial import ControlledSpatialPlugin, SpatialStore
from spatial_client import JatenClient, MapCommandRejected, NavigationStateUnavailable


def wait(predicate, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError('worker did not reach expected state')


class Tracker:
    def __init__(self):
        self.lock = threading.RLock()
        self._active, self.events = {}, []
        self.counter = 0

    def begin(self, channel, tool, notify=True):
        self.counter += 1
        aid, event = str(self.counter), threading.Event()
        self._active[channel] = aid, tool, event
        return aid, event

    def current(self, channel, aid):
        return channel in self._active and self._active[channel][0] == aid

    def finish(self, channel, aid, tool, status, result):
        with self.lock:
            if self.current(channel, aid):
                self.events.append((status, result))
                self._active.pop(channel)

    def cancel_all(self, reason, status='cancelled'):
        with self.lock:
            for channel, (aid, tool, event) in list(self._active.items()):
                event.set()
                self.finish(channel, aid, 'controlled_spatial', status, {'reason': reason})


class Bundle:
    def control_ownership(self):
        return {'mode_policy': 'BaseArm'}

    def require_base_control(self, tool):
        self.state.require_motion((3,))
        if self.state.read('upper_velocity_enabled')[0] is not False:
            raise RuntimeError('base velocity ownership changed')

    def prepare_mapping_velocity(self, request_epoch):
        if request_epoch != self._stop_epoch:
            raise RuntimeError('superseded')

    def __init__(self):
        self.tracker = Tracker()
        self._stop_epoch, self._transition = 0, None
        self.state = types.SimpleNamespace(motion_blocked=False, require_motion=lambda modes: None,
                                           snapshot=lambda: {'controller_boot': None},
                                           motion_send_uncertain=False,
                                           require_motion_enabled=lambda: None,
                                           read=lambda key: (False if key == 'upper_velocity_enabled'
                                                             else 'nav_mpc_path_track', time.monotonic()))
        self.ros1 = types.SimpleNamespace(connection=(True, 1))
        self.base_servo = types.SimpleNamespace(preempt=lambda: None, mark_paused=lambda: None)
        self.pauses = []

    def _resume_unresolved(self):
        return False

    def _mode_unresolved(self):
        return False

    def _begin_control(self, action):
        with self.tracker.lock:
            self.state.require_motion_enabled()
            if self._transition or self.tracker._active:
                raise RuntimeError('busy')
            token = {'cancelled': threading.Event()}
            self._transition = token
            previous = self.state.motion_blocked
            self.state.motion_blocked = True
            return token, None, previous

    def _check_transition(self, token):
        if self._transition is not token or token['cancelled'].is_set():
            raise RuntimeError('superseded')

    def confirm_control(self, token, name, kind, args, success, field, expected):
        self._check_transition(token)
        assert name == '/enable_vel_control' and args == {'data':False}

    def _end_control(self, token):
        with self.tracker.lock:
            if self._transition is token:
                self._transition = None

    def start_motion(self, channel, tool, modes, publish, validate, request_epoch):
        with self.tracker.lock:
            if request_epoch != self._stop_epoch:
                raise RuntimeError('superseded')
            if self.spatial.unresolved() or channel in self.tracker._active:
                raise RuntimeError('busy')
            validate()
            aid, cancelled = self.tracker.begin(channel, tool)
            publish()
            return aid, cancelled, 1, time.monotonic(), {}

    def pause(self, reason, status='cancelled', expected=None, expected_transition=None):
        with self.tracker.lock:
            if expected_transition is not None and self._transition is not expected_transition:
                return {'state': 'idle'}
            if self._transition is not None:
                self._transition['cancelled'].set()
                self._transition = None
            if expected and not self.tracker.current(*expected):
                return {'state': 'idle'}
            self._stop_epoch += 1
            self.tracker.cancel_all(reason, status)
        self.pauses.append(reason)
        try:
            self.spatial.cancel_remote()
            return {'state': 'paused'}
        except Exception:
            return {'state': 'error', 'physical_stop_confirmed': False}


class Client:
    def __init__(self):
        self.identity = ('factory', 'grid-A:road-A')
        self.grid_maps = [{'fileName': 'factory.pgm'}, {'fileName': 'other.pgm'}]
        self.calls, self.commands = [], []
        self.check = {'success': True, 'status': 2, 'arrived': True}
        self.navigate = {'success': True, 'task_id': 'native-task-1'}
        self.feedback = {
            'pose2d': {'x': 1., 'y': 2., 'theta': .3},
            'state': {'driving': False, 'agvPosition': {'positionInitialized': True, 'mapId': 'factory'},
                      'velocity': {'vx': 0., 'vy': 0., 'omega': 0.}},
            'agvState': {'isMapping': False, 'runningMode': 'AUTO', 'softwareEstop': False,
                         'estopButtonEvent': False, 'bumperEvent': False, 'driverError': False},
            'localizationStatus': {'cartoStatusId': 1}}

    def native(self, operation, **args):
        if operation == 'standby_voice':
            self.standby_requests = getattr(self, 'standby_requests', []) + [args]
            return {'state': 'disabled'}
        if operation == 'motion_music':
            self.music_requests = getattr(self, 'music_requests', []) + [args]
            return {'state': 'disabled'}
        self.calls.append((operation, args))
        if operation == 'manual_auto_guard':
            return {'success':True,'stationary':True,'native_tracker_idle':True,'upper_zero':True}
        if operation == 'manual_stop':
            return {'success':True,'stationary':True,'manual_zero_sent':True,
                    'completion_basis':'factory_manual_zero_and_fresh_stationary_odometry'}
        if operation == 'upper_handoff':
            return {'success':True,'stationary':True,'software_estop':False,
                    'velocity_source':'nav_mpc_move_base','delivery_uncertain':False}
        if operation == 'navigate':
            return self.navigate
        if operation == 'check':
            return self.check
        if operation == 'prepare':
            released = self.feedback['agvState']['softwareEstop']
            self.feedback['agvState']['softwareEstop'] = False
            return {'success': True, 'stationary': True, 'software_estop': False,
                    'old_task_cancelled': True, 'released': released}
        if operation in ('soft_stop', 'manual_hold'):
            self.feedback['agvState']['softwareEstop'] = True
            return {'success': True, 'stationary': True, 'software_estop': True}
        if operation == 'stationary':
            return {'success': True, 'stationary': getattr(self, 'stationary', True)}
        if operation == 'map_save_status':
            return {'complete': True, 'files': {'final': [2, 1]}}
        if operation == 'map_runtime_status':
            return {'boot_id': '00000000-0000-0000-0000-000000000001',
                    'uptime_s': 60., 'observed_at': time.time()}
        if operation == 'cancel':
            return {'success': True, 'stationary': True}
        return {'owner': '/leju_node'}

    def current_map(self):
        return {'mapId': self.identity[0], 'mapMd5': 'road-A'}

    def map_identity(self):
        return self.identity

    def saved_map_identity(self, name):
        return name, 'grid-new:road-new'

    def maps(self):
        return self.grid_maps

    def observe(self, required=None):
        return copy.deepcopy(self.feedback)

    def command(self, method, params, timeout):
        self.commands.append((method, params))
        if method == 'ChangeMode':
            self.feedback['agvState']['runningMode'] = params['mode']
        elif method == 'StartCartoMapping':
            self.feedback['agvState']['isMapping'] = True
        elif method == 'SaveCartoMap':
            self.grid_maps.append({'fileName': params['mapName'] + '.pgm'})
        elif method == 'AgvSystemRestart':
            self.feedback['agvState']['isMapping'] = False
        elif method == 'ChangeRoadMap':
            self.identity = (params['mapName'], 'grid-new:road-new')
            self.feedback['state']['agvPosition']['mapId'] = params['mapName']


class SpatialTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = str(Path(self.temp.name) / 'state.db')
        self.bundle, self.client = Bundle(), Client()
        self.plugin = ControlledSpatialPlugin(self.bundle, {'db_path': self.path}, self.client)
        self.bundle.spatial = self.plugin

    def tearDown(self):
        self.bundle.pause('test_cleanup')
        self.plugin.close()
        self.temp.cleanup()

    def call(self, action, **args):
        result = self.plugin.dispatch(action, args)
        if action == 'start' and result['state'] == 'loading':
            self.plugin._startup_thread.join(4)
            return dict(self.plugin._startup)
        return result

    def navigate(self, **args):
        return self.call('navigate_to_pose', x=1., y=2., yaw=.3, **args)

    def completed(self):
        wait(lambda: bool(self.bundle.tracker.events))
        return self.bundle.tracker.events[-1]

    def test_preview_stream_updates_pose_and_clears_it_on_map_mismatch(self):
        first = copy.deepcopy(self.client.feedback)
        second = copy.deepcopy(first)
        second['pose2d'].update(x=1.4, y=2.1, theta=-.2)
        other_map = copy.deepcopy(second)
        other_map['state']['agvPosition']['mapId'] = 'other'
        self.client.pose_observations = lambda active: iter(
            [(first, 4.), (second, 4.2), (other_map, 4.4)])
        samples = list(self.plugin.preview_pose_updates(lambda: True))
        self.assertEqual(samples, [
            (self.client.identity, {'x': 1., 'y': 2., 'yaw': .3}, 4.),
            (self.client.identity, {'x': 1.4, 'y': 2.1, 'yaw': -.2}, 4.2),
            None,
        ])

    def test_get_pose_requires_localized_matching_map_but_not_auto_control(self):
        self.client.feedback['agvState'].update(runningMode='MANUAL', softwareEstop=True)
        response = self.call('get_pose')
        self.assertEqual(response['pose'], {'x': 1., 'y': 2., 'yaw': .3})
        self.assertEqual(response['frame'], 'map')
        self.assertEqual(response['map_fingerprint'], 'grid-A:road-A')
        self.assertFalse(self.client.commands)

        self.client.feedback['state']['agvPosition']['mapId'] = 'different'
        self.assertEqual(self.call('get_pose')['error'], 'pose_map_identity_mismatch')
        self.client.feedback['state']['agvPosition']['mapId'] = 'factory'
        self.client.feedback['localizationStatus']['cartoStatusId'] = 0
        self.assertEqual(self.call('get_pose')['error'], 'cartographer_localization_not_ready')

    def test_start_retries_read_only_state_before_preparing_exactly_once(self):
        original = self.client.observe
        calls = []
        def delayed(required=None):
            calls.append(required)
            if len(calls) <= 2:
                raise NavigationStateUnavailable(['state'])
            return original(required)
        self.client.observe = delayed
        self.assertEqual(self.call('start')['state'], 'running')
        self.assertEqual([c[0] for c in self.client.calls], ['prepare'])
        self.assertEqual(calls, [('agvState', 'state')] * 4)

    def test_info_missing_feedback_is_loading_and_does_not_hide_background_start(self):
        self.plugin._startup_key=(self.bundle._stop_epoch,self.bundle.ros1.connection)
        self.plugin._startup={'state':'running'}
        self.client.observe=lambda **kw: (_ for _ in ()).throw(NavigationStateUnavailable(['agvState','pose2d','state','localizationStatus']))
        result=self.call('info')
        self.assertEqual(result['state'],'loading')
        self.assertEqual(result['startup']['state'],'running')
        self.assertIsNone(result['feedback'])
        self.assertEqual(result['feedback_status']['state'],'loading')
        self.assertFalse(self.client.calls)
        self.client.observe=lambda **kw: copy.deepcopy(Client().feedback)
        self.assertEqual(self.call('info')['state'],'ready')

    def test_persistent_missing_state_has_diagnostic_without_control_mutation(self):
        self.client.observe = lambda **kw: (_ for _ in ()).throw(NavigationStateUnavailable(['agvState']))
        result = self.call('start')
        self.assertEqual(result['state'], 'error')
        self.assertIn('missing=agvState', result['error'])
        self.assertFalse(self.client.calls)
        self.assertFalse(self.plugin.unresolved())

    def test_stop_during_missing_state_retry_cannot_later_prepare(self):
        self.client.observe = lambda **kw: (_ for _ in ()).throw(NavigationStateUnavailable(['state']))
        self.plugin.start_chassis()
        wait(lambda: self.plugin.startup_status().get('missing_state') == ['state'])
        self.bundle._stop_epoch += 1
        self.plugin._startup_thread.join(2)
        self.assertFalse(self.client.calls)

    def test_post_prepare_state_retry_does_not_repeat_remote_commands(self):
        original = self.client.observe
        reads = []
        def observe(required=None):
            reads.append(required)
            if len(reads) == 2:
                raise NavigationStateUnavailable(['state'])
            return original(required)
        self.client.observe = observe
        self.assertEqual(self.call('start')['state'], 'running')
        self.assertEqual([c[0] for c in self.client.calls], ['prepare'])

    def test_contract_lifecycle_resource_and_acp(self):
        schema = self.plugin.get_tool()['inputSchema']
        self.assertEqual(schema['x-resource'], 'base')
        self.assertEqual(schema['x-hooks']['on_interrupt_motion'], {'action': 'stop_nav'})
        self.assertEqual(set(schema['properties']['action']['enum']),
                         set(schema['x-action-params']) | (set(schema['x-workbench']['actions']) - {'info', 'stop'}))
        self.assertIn('load_map', schema['x-completion']['actions'])
        self.assertEqual(self.call('start')['state'], 'running')
        self.assertEqual(self.call('stop')['state'], 'idle')
        self.assertFalse(any(c[0] == 'navigate' for c in self.client.calls))

    def test_start_cancels_then_releases_chassis_without_navigation(self):
        self.client.feedback['agvState']['softwareEstop'] = True
        result = self.call('start')
        self.assertEqual(result['state'], 'running')
        self.assertTrue(result['chassis_preparation']['old_task_cancelled'])
        self.assertTrue(result['chassis_preparation']['released'])
        self.assertFalse(self.client.feedback['agvState']['softwareEstop'])
        self.assertFalse(self.plugin.unresolved())
        self.assertEqual([c[0] for c in self.client.calls], ['prepare'])

    def test_background_start_deduplicates_and_ignores_only_web_velocity_cache(self):
        entered, release = threading.Event(), threading.Event()
        original = self.client.native
        def blocked(operation, **args):
            if operation == 'prepare':
                entered.set();release.wait(2)
            return original(operation, **args)
        self.client.native = blocked
        self.client.feedback['state']['velocity'] = {'vx': -.051, 'vy': -.051, 'omega': 0.}
        try:
            first = self.plugin.dispatch('start', {})
            self.assertEqual(first['state'], 'loading')
            self.assertIn('Checking chassis',first['message'])
            self.assertTrue(entered.wait(1))
            second = self.plugin.dispatch('start', {})
            self.assertEqual(second['state'], 'loading')
            release.set();self.plugin._startup_thread.join(2)
            self.assertEqual(self.plugin.startup_status()['state'],'running')
            self.assertEqual([c[0] for c in self.client.calls],['prepare'])
        finally:
            release.set()

    def test_start_fault_or_readonly_does_not_release(self):
        for field in ('estopButtonEvent', 'bumperEvent', 'driverError'):
            self.client.feedback['agvState'][field] = True
            self.assertEqual(self.call('start')['state'], 'error')
            self.client.feedback['agvState'][field] = False
        self.bundle.state.require_motion_enabled = lambda: (_ for _ in ()).throw(RuntimeError('disabled'))
        self.assertEqual(self.call('start')['state'], 'error')
        self.assertFalse(self.client.calls)

    def test_start_does_not_cancel_an_active_local_action(self):
        self.bundle.tracker.begin('base', 'base_drive')
        result = self.call('start')
        self.assertEqual(result['state'], 'running')
        self.assertEqual(result['active_base_controller'], 'base_drive')
        self.assertFalse(self.client.calls)

    def test_known_rejection_is_stopped_and_can_be_retried(self):
        original = self.client.native
        def rejected(operation, **args):
            if operation == 'prepare':
                return {'success': False, 'delivery_uncertain': False}
            return original(operation, **args)
        self.client.native = rejected
        self.assertEqual(self.call('start')['state'], 'error')
        self.assertTrue(self.client.feedback['agvState']['softwareEstop'])
        self.assertFalse(self.plugin.unresolved())
        self.client.native = original
        self.assertEqual(self.call('start')['state'], 'running')

    def test_info_never_releases_software_stop(self):
        self.client.feedback['agvState']['softwareEstop'] = True
        self.assertEqual(self.call('info')['state'], 'ready')
        self.assertTrue(self.client.feedback['agvState']['softwareEstop'])
        self.assertFalse(self.client.calls)

    def test_uncertain_release_survives_restart_and_blocks_motion(self):
        original = self.client.native
        def lost(operation, **args):
            if operation == 'prepare':
                raise TimeoutError('possibly delivered')
            return original(operation, **args)
        self.client.native = lost
        self.assertEqual(self.call('start')['state'], 'error')
        self.assertTrue(self.plugin.unresolved())
        self.assertIn(('soft_stop', {}), self.client.calls)
        restored = ControlledSpatialPlugin(self.bundle, {'db_path': self.path}, self.client)
        self.assertTrue(restored.unresolved())
        self.assertTrue(restored._preparation['delivery_uncertain'])
        restored.close()
        self.assertEqual(self.call('start')['state'], 'error')

    def test_stop_orders_after_blocked_release_and_wins(self):
        entered, release = threading.Event(), threading.Event()
        original = self.client.native
        def blocked(operation, **args):
            if operation == 'prepare':
                entered.set()
                release.wait(3)
            return original(operation, **args)
        self.client.native = blocked
        starts, stops = [], []
        start = threading.Thread(target=lambda: starts.append(self.call('start')))
        start.start()
        self.assertTrue(entered.wait(1))
        stop = threading.Thread(target=lambda: stops.append(self.call('stop_nav')))
        stop.start()
        wait(lambda: self.bundle._stop_epoch > 0)
        self.assertFalse(stops)
        release.set()
        start.join(2); stop.join(2)
        self.assertEqual(starts[0]['state'], 'error')
        self.assertEqual(stops[0]['state'], 'paused')
        self.assertTrue(self.client.feedback['agvState']['softwareEstop'])
        self.assertFalse(self.plugin.unresolved())
        self.assertEqual([c[0] for c in self.client.calls], ['prepare', 'soft_stop'])

    def test_arrival_uses_exact_native_task_id(self):
        result = self.navigate()
        self.assertEqual(result['state'], 'executing')
        status, details = self.completed()
        self.assertEqual(status, 'completed')
        self.assertEqual(details['task_id'], 'native-task-1')
        self.assertIn(('check', {'task_id': 'native-task-1'}), self.client.calls)
        self.assertFalse(self.plugin.unresolved())

    def test_native_handoff_rejects_unchanged_latched_move_base_source(self):
        self.bundle.state.read = lambda key: (
            False if key == 'upper_velocity_enabled' else 'nav_mpc_move_base',
            time.monotonic() - 60.,
        )
        self.plugin.NAV_HANDOFF_GRACE = .3
        self.client.check = {'success': True, 'status': 1, 'arrived': False}
        self.assertEqual(self.navigate()['state'], 'executing')
        status, result = self.completed()
        self.assertEqual(status, 'error')
        self.assertIn('chassis_selected_nav_mpc_move_base', result['reason'])
        wait(lambda: ('cancel', {}) in self.client.calls)

    def test_failure_after_native_acceptance_still_cancels_the_goal(self):
        self.client.navigate = {'success': False, 'stage': 'resume_new_tracker',
                                'goal_submitted': True, 'delivery_uncertain': False}
        self.assertEqual(self.navigate()['state'], 'executing')
        status, result = self.completed()
        self.assertEqual(status, 'error')
        self.assertIn('failed_at_resume_new_tracker', result['reason'])
        wait(lambda: ('cancel', {}) in self.client.calls)
        wait(lambda: not self.plugin.unresolved())

    def test_uncertain_native_handoff_is_not_cleared_as_a_rejected_goal(self):
        self.client.navigate = {'success': False, 'stage': 'goal_submit',
                                'goal_submitted': True, 'delivery_uncertain': True}
        self.assertEqual(self.navigate()['state'], 'executing')
        self.assertEqual(self.completed()[0], 'error')
        wait(lambda: ('cancel', {}) in self.client.calls)
        self.assertTrue(self.plugin.unresolved())

    def test_pending_task_is_not_completed_by_close_pose(self):
        self.client.check = {'success': True, 'status': 0, 'arrived': False}
        self.navigate()
        wait(lambda: any(c[0] == 'check' for c in self.client.calls))
        self.assertFalse(self.bundle.tracker.events)
        self.assertEqual(self.call('stop_nav')['state'], 'paused')
        self.assertIn(('cancel', {}), self.client.calls)
        self.assertEqual(self.completed()[0], 'cancelled')

    def test_unknown_or_failed_status_causes_pause(self):
        for code in (3, 99):
            with self.subTest(code=code):
                self.bundle.tracker.events.clear()
                self.client.check = {'success': True, 'status': code, 'arrived': False}
                self.navigate()
                self.assertEqual(self.completed()[0], 'error')
                wait(lambda: not self.plugin.unresolved())

    def test_arrival_requires_pose_and_stationary_feedback(self):
        self.client.feedback['pose2d']['x'] = 9
        self.navigate()
        self.assertEqual(self.completed()[0], 'error')
        wait(lambda: not self.plugin.unresolved())
        self.bundle.tracker.events.clear()
        self.client.feedback['pose2d']['x'] = 1
        self.client.feedback['state']['velocity']['vx'] = .2
        self.client.stationary = False
        self.navigate()
        wait(lambda: any(c[0] == 'check' for c in self.client.calls))
        time.sleep(.3)
        self.assertFalse(self.bundle.tracker.events)
        self.client.stationary = True
        self.assertEqual(self.completed()[0], 'completed')

    def test_navigation_can_prepare_from_a_held_auto_chassis_without_bypassing_localization(self):
        self.client.feedback['agvState']['softwareEstop'] = True
        original = self.client.native
        def native(operation, **args):
            if operation == 'navigate':
                self.assertTrue(self.client.feedback['agvState']['softwareEstop'])
                self.client.feedback['agvState']['softwareEstop'] = False
            return original(operation, **args)
        self.client.native = native
        result = self.navigate()
        self.assertEqual(result['state'], 'executing', result)
        self.assertEqual(self.completed()[0], 'completed')
        self.assertIn('navigate', [op for op, _ in self.client.calls])

    def test_navigation_preparation_may_change_mode_but_still_requires_valid_localization(self):
        self.client.feedback['agvState'].update(runningMode='MANUAL', softwareEstop=True)
        self.assertEqual(self.plugin._observation(navigable=True, preparing=True)[0]['x'], 1.)
        with self.assertRaisesRegex(RuntimeError, 'localized_auto_mode'):
            self.plugin._observation(navigable=True, held=True)
        self.client.feedback['localizationStatus']['cartoStatusId'] = 4
        self.assertEqual(self.navigate()['state'], 'error')
        self.assertFalse(any(op == 'navigate' for op, _ in self.client.calls))

    def test_safety_faults_prevent_submit(self):
        for field in ('estopButtonEvent', 'bumperEvent', 'driverError'):
            with self.subTest(field=field):
                self.client.feedback['agvState'][field] = True
                self.assertEqual(self.navigate()['state'], 'error')
                self.client.feedback['agvState'][field] = False
        self.assertFalse(any(c[0] == 'navigate' for c in self.client.calls))

    def test_localization_and_map_must_match(self):
        self.client.feedback['state']['agvPosition']['positionInitialized'] = False
        self.assertEqual(self.navigate()['state'], 'error')
        self.client.feedback['state']['agvPosition']['positionInitialized'] = True
        self.client.feedback['state']['agvPosition']['mapId'] = 'other'
        self.assertEqual(self.navigate()['state'], 'error')
        self.client.feedback['state']['agvPosition']['mapId'] = 'factory'
        self.client.feedback['localizationStatus']['cartoStatusId'] = 0
        self.assertEqual(self.navigate()['state'], 'error')

    def test_invalid_inputs_never_submit(self):
        for args in ({'x': float('nan'), 'y': 2, 'yaw': 0},
                     {'x': 1, 'y': 2, 'yaw': 0, 'speed': 10}):
            self.assertEqual(self.call('navigate_to_pose', **args)['state'], 'error')
        self.assertFalse(any(c[0] == 'navigate' for c in self.client.calls))

    def test_tag_coordinates_and_fingerprint_persist(self):
        self.assertEqual(self.call('tag_place', name='P1', description='door')['state'], 'completed')
        self.assertEqual(self.call('list_tags')['tags'][0]['distance'], 0)
        store = SpatialStore(self.path)
        self.assertEqual(store.tags(self.client.identity)[0]['yaw'], .3)
        store.close()
        self.client.identity = ('factory', 'replaced-grid:same-road')
        self.assertEqual(self.call('list_tags')['tags'], [])
        self.assertEqual(self.call('navigate_to_tag', tag_name='P1')['state'], 'error')

    def test_tag_name_is_exact_and_untag_removes_only_one(self):
        self.call('tag_place', name='P10')
        self.assertEqual(self.call('navigate_to_tag', tag_name='P1')['state'], 'error')
        self.assertFalse(self.call('untag_place', name='P1')['removed'])
        self.assertTrue(self.call('untag_place', name='P10')['removed'])

    def test_inflight_map_change_aborts_navigation(self):
        self.client.check = {'success': True, 'status': 1, 'arrived': False}
        self.navigate()
        wait(lambda: any(c[0] == 'navigate' for c in self.client.calls))
        self.client.identity = ('factory', 'new-grid')
        self.assertEqual(self.completed()[0], 'error')
        wait(lambda: any(c[0] == 'cancel' for c in self.client.calls))

    def test_submit_failure_stays_uncertain_after_cancel_and_restart(self):
        original = self.client.native
        def fail(operation, **args):
            if operation == 'navigate':
                raise TimeoutError('possibly delivered')
            return original(operation, **args)
        self.client.native = fail
        self.navigate()
        self.assertEqual(self.completed()[0], 'error')
        wait(lambda: not self.plugin._workers)
        self.assertTrue(self.plugin.unresolved())
        self.assertEqual(self.call('stop_nav')['state'], 'error')
        restored = ControlledSpatialPlugin(self.bundle, {'db_path': self.path}, self.client)
        self.assertTrue(restored.unresolved())
        self.assertTrue(restored._uncertain)
        restored.close()

    def test_stop_waits_for_older_submit_then_cancels(self):
        entered, release = threading.Event(), threading.Event()
        original = self.client.native
        def blocked(operation, **args):
            if operation == 'navigate':
                entered.set()
                release.wait(2)
            return original(operation, **args)
        self.client.native = blocked
        self.navigate()
        self.assertTrue(entered.wait(1))
        result = []
        thread = threading.Thread(target=lambda: result.append(self.call('stop_nav')))
        thread.start()
        time.sleep(.05)
        self.assertFalse(result)
        self.assertFalse(any(c[0] == 'cancel' for c in self.client.calls))
        release.set()
        thread.join(2)
        self.assertEqual(result[0]['state'], 'paused')
        self.assertFalse(self.plugin.unresolved())
        self.assertEqual([c[0] for c in self.client.calls if c[0] in ('navigate', 'cancel')], ['navigate', 'cancel'])

    def test_cancel_ack_without_stationary_is_unconfirmed(self):
        self.client.check = {'success': True, 'status': 1, 'arrived': False}
        self.navigate()
        wait(lambda: self.plugin._task and self.plugin._task['task_id'])
        original = self.client.native
        self.client.native = lambda op, **args: ({'success': True, 'stationary': False}
                                                if op == 'cancel' else original(op, **args))
        self.assertEqual(self.call('stop_nav')['state'], 'error')
        self.assertTrue(self.plugin.unresolved())

    def test_explicit_rejection_is_not_unknown_delivery(self):
        self.client.navigate = {'success': False}
        self.navigate()
        self.assertEqual(self.completed()[0], 'error')
        wait(lambda: not self.plugin._workers)
        self.assertFalse(self.plugin.unresolved())

    def test_map_workflow_is_async_and_rebinds_provisional_tags(self):
        started = self.call('start_mapping', map_name='new')
        self.assertEqual(started['state'], 'executing')
        self.assertIn('mapping_console', started['message'])
        status, reply = self.completed()
        self.assertEqual(status, 'completed')
        self.assertIn('Please call mapping_console', reply['message'])
        self.assertIn('instead of the G12 remote controller', reply['message'])
        self.assertEqual(self.plugin.store.state('mapping')['phase'], 'mapping')
        self.assertEqual(self.client.commands, [
            ('ChangeMode', {'mode': 'MANUAL'}), ('StartCartoMapping', {})])
        self.assertEqual(self.client.feedback['agvState']['runningMode'], 'MANUAL')
        self.assertTrue(self.client.feedback['agvState']['softwareEstop'])
        self.assertEqual(self.navigate()['state'], 'error')
        self.call('tag_place', name='entry')
        self.bundle.tracker.events.clear()
        self.assertEqual(self.call('stop_mapping')['state'], 'executing')
        self.assertEqual(self.completed()[0], 'completed')
        self.assertIsNone(self.plugin.store.state('mapping'))
        identity = ('new', 'grid-new:road-new')
        self.assertEqual(self.plugin.store.state(self.plugin.store.anchor_key(identity)),
                         {'x': 1., 'y': 2., 'yaw': .3})
        self.assertEqual(self.plugin.store.tags(identity)[0]['name'], 'entry')
        self.assertEqual(self.client.commands[-1], ('ChangeMode', {'mode':'AUTO'}))
        self.assertFalse(any(method in ('ChangeCartoMap','ChangeRoadMap') for method, _ in self.client.commands))
        self.assertEqual(self.client.identity[0], 'factory')
        self.assertFalse(self.plugin.unresolved())

    def test_console_opened_during_start_is_authorized_only_for_that_completion(self):
        entered,release=threading.Event(),threading.Event()
        original=self.client.command
        def command(method,params,timeout):
            if method=='StartCartoMapping':
                entered.set();release.wait(2)
            return original(method,params,timeout)
        self.client.command=command
        started=self.call('start_mapping',map_name='new')
        try:
            self.assertTrue(entered.wait(1))
            self.call('mapping_console')
            self.assertEqual(self.call('keyboard_open')['mapping_state'],'starting')
            self.assertEqual(self.call('keyboard_status')['mapping_state'],'starting')
        finally:
            release.set()
        self.assertEqual(self.completed()[0],'completed')
        status=self.call('keyboard_status')
        self.assertEqual(status['mapping_state'],'mapping')
        self.assertTrue(status['console_authorized'])
        self.assertIsNone(self.plugin.keyboard.session)

    def test_mapping_requires_both_active_slam_and_manual_web_route(self):
        original = self.client.command
        def command(method, params, timeout):
            original(method, params, timeout)
            if method == 'StartCartoMapping':
                self.client.feedback['agvState']['isMapping'] = False
        self.client.command = command
        def confirm(predicate, *args, **kwargs):
            if not predicate():
                raise RuntimeError('map_transition_not_confirmed_inspect_info')
        with patch.object(self.plugin, '_wait', side_effect=confirm):
            self.call('start_mapping', map_name='new')
            self.assertEqual(self.completed()[0], 'error')
        self.assertEqual(self.plugin.store.state('mapping')['phase'], 'starting')
        self.assertTrue(self.plugin.unresolved())

    def test_lost_mapping_remote_route_ack_is_not_reported_ready(self):
        original = self.client.command
        def command(method, params, timeout):
            original(method, params, timeout)
            if method == 'ChangeMode' and params['mode'] == 'MANUAL':
                raise TimeoutError('lost_manual_ack')
        self.client.command = command
        self.call('start_mapping', map_name='new')
        self.assertEqual(self.completed()[0], 'error')
        self.assertEqual(self.plugin.store.state('mapping')['phase'], 'starting')
        self.assertTrue(self.plugin.unresolved())

    def test_loading_does_not_clear_stop_or_forge_localization(self):
        self.plugin.LOAD_LOCALIZATION_WAIT = .01
        self.client.feedback['agvState']['softwareEstop'] = True
        self.client.feedback['state']['agvPosition']['positionInitialized'] = False
        self.assertEqual(self.call('load_map', map_name='other')['state'], 'executing')
        status, result = self.completed()
        self.assertEqual(status, 'completed')
        self.assertTrue(result['localization_required'])
        self.assertEqual(result['localization']['state'], 'unconfirmed')
        self.assertTrue(self.client.feedback['agvState']['softwareEstop'])
        self.assertFalse(self.client.feedback['state']['agvPosition']['positionInitialized'])
        self.assertEqual(self.navigate()['state'], 'error')

    def test_removed_unload_action_and_legacy_state(self):
        self.plugin.store.state('map_unloaded', True)
        schema = self.plugin.get_tool()['inputSchema']
        self.assertNotIn('upload_map', schema['properties']['action']['enum'])
        self.assertNotIn('unload_map', schema['properties']['action']['enum'])
        self.assertEqual(self.call('unload_map')['error'], 'unknown_controlled_spatial_action')
        self.assertEqual(self.call('list_maps')['current_map']['mapId'], 'factory')
        self.assertEqual(self.call('get_pose')['map'], 'factory')
        self.assertEqual(self.client.commands, [])

    def test_archive_only_map_can_be_loaded_without_separate_upload(self):
        archive = self.plugin.archive = unittest.mock.Mock()
        archive.names.return_value = ['archive', 'other']
        original = self.client.native

        def native(operation, **args):
            if operation == 'map_upload_commit':
                self.client.grid_maps.append({'fileName': 'archive.pgm'})
                return {'success': True}
            return original(operation, **args)

        self.client.native = native
        self.assertIn({'fileName': 'archive.pgm', 'status': 'archive_only'},
                      self.call('list_maps')['maps'])
        self.assertEqual(self.call('load_map', map_name='archive')['state'], 'executing')
        self.assertEqual(self.completed()[0], 'completed')
        archive.stage_upload.assert_called_once()
        self.assertEqual(self.client.identity[0], 'archive')

    def test_load_uses_only_the_anchor_for_the_loaded_map_revision(self):
        anchor = {'x': .2, 'y': -.1, 'yaw': .3}
        identity = ('other', 'grid-new:road-new')
        self.plugin.store.state(self.plugin.store.anchor_key(('other', 'old:revision')),
                                {'x': 8., 'y': 9., 'yaw': 1.})
        self.plugin.store.state(self.plugin.store.anchor_key(identity), anchor)
        native = self.client.native
        observed = []

        def native_with_relocalization(operation, **args):
            if operation == 'relocalize':
                observed.append((self.client.identity, args['anchor']))
                return {'state': 'corrected'}
            return native(operation, **args)

        self.client.native = native_with_relocalization
        self.assertEqual(self.call('load_map', map_name='other')['state'], 'executing')
        status, result = self.completed()
        self.assertEqual(status, 'completed')
        self.assertEqual(observed, [(identity, anchor)])
        self.assertFalse(result['localization_required'])

    def test_unverified_anchor_match_is_reported_and_blocks_navigation(self):
        identity = ('other', 'grid-new:road-new')
        self.plugin.store.state(self.plugin.store.anchor_key(identity), {'x': 0., 'y': 0., 'yaw': 0.})
        native = self.client.native
        self.client.native = lambda op, **args: (
            {'state': 'no_distinct_anchor_match'} if op == 'relocalize' else native(op, **args))
        self.assertEqual(self.call('load_map', map_name='other')['state'], 'executing')
        _, result = self.completed()
        self.assertTrue(result['localization_required'])
        self.assertIn('map_relocalization_unconfirmed', self.navigate()['error'])

    def test_loading_waits_for_mode_feedback_after_command_ack(self):
        original_command, original_observe = self.client.command, self.client.observe
        pending, observed = [], []
        def command(method, params, timeout):
            if method == 'ChangeMode':
                self.client.commands.append((method, params))
                pending[:] = [params['mode']]
            else:
                if method == 'ChangeCartoMap':
                    self.assertEqual(self.client.feedback['agvState']['runningMode'], 'MANUAL')
                    self.assertIn('MANUAL', observed)
                original_command(method, params, timeout)
        def observe(required=None):
            if pending and required == ('agvState',):
                self.client.feedback['agvState']['runningMode'] = pending.pop()
                observed.append(self.client.feedback['agvState']['runningMode'])
            return original_observe(required)
        self.client.command, self.client.observe = command, observe
        self.call('load_map', map_name='other')
        status, result = self.completed()
        self.assertEqual(status, 'completed')
        self.assertFalse(result['localization_required'])
        self.assertEqual(result['localization']['state'], 'initialized')
        self.assertEqual(observed, ['MANUAL', 'AUTO'])

    def test_explicit_mode_precondition_rejection_does_not_latch_map_busy(self):
        original = self.client.command
        def command(method, params, timeout):
            if method == 'ChangeCartoMap':
                raise MapCommandRejected(method, 8211)
            original(method, params, timeout)
        self.client.command = command
        self.call('load_map', map_name='other')
        status, result = self.completed()
        self.assertEqual(status, 'error')
        self.assertEqual(result['reason'], 'factory_map_command_rejected')
        self.assertEqual(result['failed_stage'], 'ChangeCartoMap')
        self.assertFalse(self.plugin.unresolved())
        self.assertEqual(self.client.identity[0], 'factory')

    def test_manual_lifecycle_preflight_defers_mode_and_release_until_execution(self):
        self.client.feedback['agvState']['runningMode'] = 'MANUAL'
        result = self.plugin.prepare_chassis()
        self.assertTrue(result['auto_transition_deferred'])
        self.assertFalse(self.client.commands)
        self.assertEqual(self.client.calls, [('stationary', {})])
        self.assertIsNone(self.plugin._preparation)
        self.assertFalse(self.plugin.base_preparation_required)

    def test_auto_transition_stops_manual_and_waits_for_actual_mode_without_loading_map(self):
        self.client.feedback['agvState']['runningMode'] = 'MANUAL'
        token, _, _ = self.bundle._begin_control('test_auto')
        original = self.client.command
        def command(method, params, timeout):
            self.assertIn(('manual_stop', {}), self.client.calls)
            self.assertEqual(self.plugin._preparation['delivery_uncertain'], True)
            return original(method, params, timeout)
        self.client.command = command
        self.plugin.ensure_auto_for_motion(token)
        self.assertEqual(self.client.commands, [('ChangeMode', {'mode':'AUTO'})])
        self.assertEqual(self.plugin._preparation['delivery_uncertain'], False)
        self.bundle._end_control(token)

    def test_read_only_auto_guard_failure_does_not_create_uncertain_mode_write(self):
        self.client.feedback['agvState']['runningMode']='MANUAL'
        token,_,_=self.bundle._begin_control('test_auto')
        original=self.client.native
        def native(operation,**args):
            if operation=='manual_auto_guard':raise RuntimeError('boot_remote_fresh_neutral_or_zero_output_unconfirmed')
            return original(operation,**args)
        self.client.native=native
        with self.assertRaisesRegex(RuntimeError,'zero_output_unconfirmed'):
            self.plugin.ensure_auto_for_motion(token)
        self.assertFalse(self.plugin._preparation['delivery_uncertain'])
        self.assertEqual(self.plugin._preparation['stage'],'observe_manual_auto_guard')
        self.assertFalse(self.client.commands)
        self.bundle._end_control(token)

    def test_auto_transition_ack_without_mode_feedback_does_not_confirm_preparation(self):
        self.client.feedback['agvState']['runningMode'] = 'MANUAL'
        token, _, _ = self.bundle._begin_control('test_auto')
        self.client.command = lambda *args, **kwargs: None
        with self.assertRaisesRegex(RuntimeError, 'auto_mode_feedback_unconfirmed'):
            self.plugin.ensure_auto_for_motion(token, timeout=.01)
        self.assertTrue(self.plugin._preparation['delivery_uncertain'])
        self.bundle._end_control(token)

    def test_auto_transition_never_overrides_mapping_or_a_superseding_stop(self):
        for cause in ('mapping', 'stop'):
            with self.subTest(cause=cause):
                self.bundle._transition = None
                self.plugin.store.state('mapping', None)
                token, _, _ = self.bundle._begin_control('test_auto')
                if cause == 'mapping':
                    self.plugin.store.state('mapping', {'name':'new','token':'active','phase':'mapping'})
                else:
                    token['cancelled'].set()
                    self.bundle._transition = None
                with self.assertRaises((RuntimeError, InterruptedError)):
                    self.plugin.ensure_auto_for_motion(token)
                self.assertFalse(self.client.commands)
                self.assertFalse(self.client.calls)
                self.bundle._end_control(token)

    def test_map_start_from_manual_uses_manual_zero_instead_of_dispatch_stop(self):
        self.client.feedback['agvState']['runningMode'] = 'MANUAL'
        self.call('start_mapping', map_name='new')
        self.assertEqual(self.completed()[0], 'completed')
        self.assertIn('manual_stop', [op for op, _ in self.client.calls])
        self.assertFalse(any(op == 'soft_stop' for op, _ in self.client.calls))
        self.assertEqual(self.client.feedback['agvState']['runningMode'], 'MANUAL')

    def test_map_load_from_manual_confirms_auto_after_loading_without_requiring_dispatch_stop(self):
        self.client.feedback['agvState']['runningMode'] = 'MANUAL'
        self.call('load_map', map_name='other')
        self.assertEqual(self.completed()[0], 'completed')
        self.assertIn('manual_stop', [op for op, _ in self.client.calls])
        self.assertFalse(any(op == 'soft_stop' for op, _ in self.client.calls))
        self.assertEqual(self.client.feedback['agvState']['runningMode'], 'AUTO')

    def test_worker_preflight_failure_does_not_latch_map_busy(self):
        original = self.plugin._require_stationary
        calls = []
        def stationary():
            calls.append(True)
            if len(calls) == 2:
                raise RuntimeError('temporary_feedback_loss')
            return original()
        self.plugin._require_stationary = stationary
        self.call('load_map', map_name='other')
        status, result = self.completed()
        self.assertEqual(status, 'error')
        self.assertEqual(result['reason'], 'map_preflight_failed_no_command_sent')
        self.assertFalse(self.plugin.unresolved())
        self.assertFalse(self.client.commands)

    def test_stop_mapping_preflight_failure_preserves_session_and_allows_retry(self):
        self.call('start_mapping', map_name='new')
        self.completed()
        self.bundle.tracker.events.clear()
        original = self.client.native
        def native(operation, **args):
            if operation == 'manual_stop':
                raise RuntimeError('temporary_manual_stop_feedback_loss')
            return original(operation, **args)
        self.client.native = native
        self.call('stop_mapping')
        status, result = self.completed()
        self.assertEqual(status, 'error')
        self.assertEqual(result['failed_stage'], None)
        self.assertFalse(self.plugin.unresolved())
        self.assertEqual(self.plugin.store.state('mapping')['phase'], 'mapping')
        self.client.native = original
        self.bundle.tracker.events.clear()
        self.call('stop_mapping')
        status, result = self.completed()
        self.assertEqual(status, 'completed', result)
        self.assertIsNone(self.plugin.store.state('mapping'))
        self.assertIn('manual_stop', [op for op, args in self.client.calls])

    def test_save_waits_for_factory_optimization_without_short_rpc_timeout(self):
        self.call('start_mapping', map_name='new')
        self.completed()
        self.bundle.tracker.events.clear()
        original = self.client.command
        timeouts = []
        def command(method, params, timeout):
            if method == 'SaveCartoMap':
                timeouts.append(timeout)
            original(method, params, timeout)
        self.client.command = command
        self.call('stop_mapping')
        self.assertEqual(self.completed()[0], 'completed')
        self.assertGreaterEqual(timeouts[0], 1490)

    def test_resume_save_waits_for_files_without_replaying_save_and_preserves_tags(self):
        self.call('start_mapping', map_name='new')
        self.completed()
        self.call('tag_place', name='P1')
        wait(lambda: not self.plugin._workers)
        self.bundle.tracker.events.clear()
        original = self.client.command
        def lost_save(method, params, timeout):
            original(method, params, timeout)
            if method == 'SaveCartoMap':
                raise TimeoutError('old_save_still_executing')
        self.client.command = lost_save
        self.call('stop_mapping')
        self.assertEqual(self.completed()[0], 'error')
        wait(lambda: not self.plugin._workers)
        self.bundle.tracker.events.clear()
        self.client.commands.clear()
        self.client.command = original
        # Production retains completed startup/music thread objects. These
        # background workers must not prevent resuming an unrelated save.
        background = threading.Thread(target=lambda: None, name='motion-music-start')
        background.start()
        background.join()
        self.plugin._workers.add(background)
        self.call('stop_mapping')
        wait(lambda: bool(self.bundle.tracker.events), timeout=5)
        self.assertEqual(self.completed()[0], 'completed')
        self.assertFalse(any(m == 'SaveCartoMap' for m, _ in self.client.commands))
        self.assertEqual(self.plugin.store.tags(('new', 'grid-new:road-new'))[0]['name'], 'P1')
        self.assertFalse(self.plugin.unresolved())

    def test_save_recovery_does_not_restart_on_partial_grid_files(self):
        self.plugin.store.state('mapping', {'name': 'new', 'token': 'x', 'phase': 'saving'})
        self.plugin._map_pending = {'operation': 'stop_mapping', 'stage': 'SaveCartoMap'}
        self.plugin._persist()
        self.client.grid_maps.append({'fileName': 'new.pgm'})
        original = self.client.native
        reached = threading.Event()
        def native(operation, **args):
            if operation == 'map_save_status':
                reached.set()
                return {'complete': False}
            return original(operation, **args)
        self.client.native = native
        self.assertEqual(self.call('stop_mapping')['state'], 'executing')
        self.assertTrue(reached.wait(1))
        self.assertFalse(self.client.commands)
        self.bundle.tracker.cancel_all('test_end')
        wait(lambda: not self.plugin._workers)

    def test_saving_info_explains_missing_tf_and_blocks_load(self):
        self.plugin.store.state('mapping', {'name': 'new', 'token': 'x', 'phase': 'saving'})
        self.client.observe = lambda **kwargs: (_ for _ in ()).throw(NavigationStateUnavailable(['pose2d']))
        info = self.call('info')
        self.assertEqual(info['state'], 'saving')
        self.assertIn('Optimizing', info['message'])
        self.assertIn('finish_or_reconcile', self.call('load_map', map_name='other')['error'])

    def test_invalid_maps_are_rejected_without_latching_busy(self):
        for action, args in (('load_map', {'map_name': 'missing'}),
                             ('start_mapping', {'map_name': 'factory'}),
                             ('start_mapping', {'map_name': '../escape'}), ('stop_mapping', {})):
            self.assertEqual(self.call(action, **args)['state'], 'error')
        self.assertFalse(self.plugin.unresolved())
        self.assertFalse(self.client.commands)

    def test_map_stationary_uses_native_odom_despite_stale_web_velocity(self):
        self.client.feedback['state']['velocity'] = {'vx': .1, 'vy': -.1, 'omega': -.05}
        self.plugin._require_stationary()
        self.assertEqual(self.client.calls, [('stationary', {})])
        self.assertFalse(self.client.commands)

    def test_missing_or_moving_native_odom_rejects_map_mutations(self):
        for response in ({'success': True, 'stationary': False}, {'success': False}):
            self.client.native = lambda *a, **k: response
            result = self.call('start_mapping', map_name='new')
            self.assertEqual(result['state'], 'error')
            self.assertFalse(self.client.commands)
            self.assertFalse(self.plugin.unresolved())

    def test_external_chassis_motion_prevents_map_change(self):
        self.client.feedback['state']['driving'] = True
        self.assertEqual(self.call('load_map', map_name='other')['state'], 'error')
        self.assertFalse(self.client.commands)
        self.assertFalse(self.plugin.unresolved())

    def test_lost_map_ack_is_persisted_and_blocks_motion(self):
        self.client.command = lambda *a, **k: (_ for _ in ()).throw(TimeoutError())
        self.call('start_mapping', map_name='new')
        self.assertEqual(self.completed()[0], 'error')
        wait(lambda: not self.plugin._workers)
        self.assertTrue(self.plugin.unresolved())
        self.assertEqual(self.call('tag_place', name='unsafe')['state'], 'error')
        self.assertEqual(self.call('stop_nav')['state'], 'error')
        restored = ControlledSpatialPlugin(self.bundle, {'db_path': self.path}, self.client)
        self.assertTrue(restored.unresolved())
        restored.close()

    def test_base_resource_prevents_overlapping_map_and_drive(self):
        self.bundle.tracker.begin('base', 'base_drive')
        self.assertEqual(self.call('load_map', map_name='other')['state'], 'error')
        self.assertFalse(self.client.commands)


class ClientTests(unittest.TestCase):
    def test_preview_releases_factory_socket_before_dispatch(self):
        client = object.__new__(JatenClient)
        client._ws_lock = threading.Lock()
        snapshot = {'pose2d': {'x': 1.}}
        client._observe_socket = lambda fields: (snapshot, {'pose2d': 2.})

        def command(method, params, timeout):
            self.assertTrue(client._ws_lock.locked())
            return {'method': method}

        client._command_socket = command
        stream = client.pose_observations(lambda: True)
        try:
            self.assertEqual(next(stream), (snapshot, 2.))
            self.assertEqual(client.command('ChangeMode', {}, timeout=.01), {'method': 'ChangeMode'})
        finally:
            stream.close()

    def test_base_observation_does_not_require_map_frames_but_navigation_does(self):
        frames = [json.dumps({'type': 'state', 'data': {'driving': False}}),
                  json.dumps({'type': 'agvState', 'data': {'data': {'softwareEstop': True}}})]
        class Socket:
            def __init__(self): self.frames = iter(frames)
            def close(self): pass
            def recv(self):
                try: return next(self.frames)
                except StopIteration: raise ConnectionError('closed')
        ws = types.SimpleNamespace(create_connection=lambda *a, **k: Socket(),
            WebSocketTimeoutException=TimeoutError, WebSocketException=ConnectionError)
        client = object.__new__(JatenClient);client.ws_url='ws://test';client._ws_lock=threading.Lock()
        with patch.dict(sys.modules, {'websocket': ws}):
            result=client.observe(required=('agvState','state'))
            self.assertIs(result['state']['driving'],False)
            with self.assertRaises(NavigationStateUnavailable) as caught:
                client.observe()
            self.assertEqual(caught.exception.missing, ('localizationStatus','pose2d'))

    def test_old_received_frame_does_not_satisfy_snapshot_freshness(self):
        clock = [0.]
        frames = iter([(0., {'type':'state','data':{'driving':False}}),
                       (3., {'type':'agvState','data':{'data':{'softwareEstop':False}}})])
        class Socket:
            def close(self): pass
            def recv(self):
                try:
                    clock[0], frame = next(frames)
                    return json.dumps(frame)
                except StopIteration: raise ConnectionError('closed')
        ws=types.SimpleNamespace(create_connection=lambda *a, **kw:Socket(),
            WebSocketTimeoutException=TimeoutError,WebSocketException=ConnectionError)
        client=object.__new__(JatenClient);client.ws_url='ws://test';client._ws_lock=threading.Lock()
        with patch.dict(sys.modules,{'websocket':ws}), patch('spatial_client.time.monotonic',side_effect=lambda:clock[0]):
            with self.assertRaises(NavigationStateUnavailable) as caught:
                client.observe(required=('agvState','state'))
            self.assertEqual(caught.exception.missing,('state',))

    def test_grid_content_identity_not_road_md5_alone(self):
        client = object.__new__(JatenClient)
        client.current_map = lambda: {'mapId': 'factory', 'mapMd5': 'unchanged-road'}
        client.native = lambda *a, **k: {'sha256': 'a' * 64}
        self.assertEqual(client.map_identity(), ('factory', 'a' * 64 + ':unchanged-road'))
        client.native = lambda *a, **k: {'sha256': 'b' * 64}
        self.assertNotEqual(client.map_identity()[1], 'a' * 64 + ':unchanged-road')

    def test_map_change_during_hash_is_rejected(self):
        client = object.__new__(JatenClient)
        client.current_map = unittest.mock.Mock(side_effect=[{'mapId': 'a'}, {'mapId': 'b'}])
        client.native = lambda *a, **k: {'sha256': 'a' * 64}
        with self.assertRaises(RuntimeError):
            client.map_identity()

    def test_websocket_malformed_or_oversized_frames_rejected(self):
        for raw in ('[]', 'bad json', b'{}', ' ' * 2_000_001):
            with self.subTest(raw=str(raw)[:20]), self.assertRaises((ValueError, RuntimeError)):
                JatenClient._frame(raw)

    def test_mutation_matches_both_request_id_and_method(self):
        class Socket:
            def close(self): pass
            def send(self, raw):
                self.request, self.received = json.loads(raw), 0
            def recv(self):
                self.received += 1
                r = self.request
                return json.dumps({'type': 'taskResult', 'data': {
                    'id': 'other' if self.received == 1 else r['id'],
                    'method': 'OtherMethod' if self.received == 2 else r['method']}})
        ws = types.SimpleNamespace(create_connection=lambda *a, **k: Socket(),
                                   WebSocketTimeoutException=TimeoutError)
        client = object.__new__(JatenClient)
        client.ws_url = 'ws://test'
        client._ws_lock = threading.Lock()
        with patch.dict(sys.modules, {'websocket': ws}):
            result = client.command('ChangeMode', {'mode': 'MANUAL'})
            self.assertEqual(result['method'], 'ChangeMode')
            self.assertNotEqual(result['id'], 'other')
            with self.assertRaises(ValueError): client.command('ClearEstop', {})

    def test_all_native_command_ids_fit_decimal_stoi_contract(self):
        requests = []
        class Socket:
            def close(self): pass
            def send(self, raw):
                self.request = json.loads(raw)
                requests.append(self.request)
                value = self.request['id']
                # Model the strict range expected by the native C++ wrapper.
                if not value.isascii() or not value.isdecimal() or not 1 <= int(value) <= 2_147_483_647:
                    raise AssertionError('native API would abort on request ID')
            def recv(self):
                return json.dumps({'type': 'taskResult', 'data': {
                    'id': self.request['id'], 'method': self.request['method']}})
        ws = types.SimpleNamespace(create_connection=lambda *a, **k: Socket(),
                                   WebSocketTimeoutException=TimeoutError)
        client = object.__new__(JatenClient)
        client.ws_url = 'ws://test'
        client._ws_lock = threading.Lock()
        with patch.dict(sys.modules, {'websocket': ws}):
            for edge in (0, 2_147_483_646):
                with patch('spatial_client.secrets.randbelow', return_value=edge):
                    for method in sorted(JatenClient.COMMANDS):
                        client.command(method, {})
        self.assertEqual({r['id'] for r in requests}, {'1', '2147483647'})


class BundleIntegrationTests(unittest.TestCase):
    def test_container_start_configures_music_without_starting_control_cards(self):
        from test_driver import driver, FakeRos1, FakeNode
        for enabled, automation in ((False, True), (True, True), (False, False)):
            with self.subTest(enabled=enabled, automation=automation), tempfile.TemporaryDirectory() as folder:
                ros = FakeRos1()
                bundle = driver.KuavoBundle({}, 'test', ros, FakeNode())
                bundle.spatial.close()
                client = Client()
                bundle.spatial = ControlledSpatialPlugin(bundle, {
                    'db_path': folder + '/db', 'motion_music_enabled': enabled}, client)
                bundle.startup_checks.provisioner.enabled = automation
                bundle._auto_start = []
                try:
                    with patch.object(bundle.startup_checks, 'start'), patch.object(bundle.peripherals, 'start'):
                        bundle.start_background()
                        bundle.start_background()
                    if automation:
                        bundle.spatial._music_thread.join(2)
                        self.assertEqual(client.music_requests, [{'enabled': enabled}])
                        self.assertEqual(client.standby_requests, [{'enabled': False}])
                        self.assertEqual(bundle.spatial.startup_status()['standby_voice']['state'], 'disabled')
                        self.assertEqual(bundle.spatial.startup_status()['motion_music']['state'], 'disabled')
                    else:
                        self.assertFalse(getattr(client, 'music_requests', []))
                        self.assertEqual(bundle.spatial.startup_status()['motion_music']['state'], 'skipped')
                    self.assertEqual(bundle.spatial.startup_status()['state'], 'idle')
                    self.assertFalse(client.calls)
                    self.assertFalse(ros.services)
                    self.assertFalse(ros.published)
                finally:
                    bundle.close()

    def test_standby_is_configured_even_if_movement_track_fails(self):
        with tempfile.TemporaryDirectory() as folder:
            bundle = Bundle()
            bundle.startup_checks = types.SimpleNamespace(provisioner=types.SimpleNamespace(
                enabled=True, _target=lambda role: {}))
            client = Client()
            def native(operation, **args):
                if operation == 'motion_music':
                    raise ConnectionError('offline')
                self.assertEqual((operation, args), ('standby_voice', {'enabled': False}))
                return {'state': 'disabled'}
            client.native = native
            plugin = ControlledSpatialPlugin(bundle, {'db_path': folder + '/db'}, client)
            try:
                plugin.start_background()
                wait(lambda: plugin.startup_status()['standby_voice']['state'] == 'disabled')
                self.assertEqual(plugin.startup_status()['motion_music']['state'], 'unavailable')
            finally:
                plugin.close()

    def test_music_failure_is_visible_and_retry_wait_is_cancellable(self):
        with tempfile.TemporaryDirectory() as folder:
            bundle = Bundle()
            bundle.startup_checks = types.SimpleNamespace(provisioner=types.SimpleNamespace(
                enabled=True, _target=lambda role: {}))
            client = Client()
            client.native = lambda *a, **k: (_ for _ in ()).throw(ConnectionError('offline'))
            plugin = ControlledSpatialPlugin(bundle, {'db_path': folder + '/db'}, client)
            plugin.start_background()
            wait(lambda: plugin.startup_status()['motion_music']['state'] == 'unavailable')
            plugin.close()
            self.assertFalse(plugin._music_thread.is_alive())

    def test_both_card_starts_share_preparation_without_enabling_whole_body(self):
        from test_driver import driver, FakeRos1, FakeNode
        with tempfile.TemporaryDirectory() as folder, patch.object(driver, '_acp_complete'):
            ros = FakeRos1()
            bundle = driver.KuavoBundle({}, 'test', ros, FakeNode())
            bundle.spatial.close()
            client = Client()
            client.feedback['agvState']['softwareEstop'] = True
            bundle.spatial = ControlledSpatialPlugin(bundle, {'db_path': folder + '/db'}, client)
            try:
                for card in ('base_drive', 'controlled_spatial'):
                    result = bundle.dispatch(card, {'action': 'start'})
                    self.assertIn(result['state'], ('loading', 'running'))
                    bundle.spatial._startup_thread.join(2)
                    self.assertTrue(bundle.spatial.startup_status()['chassis_preparation']['old_task_cancelled'])
                self.assertEqual([c[0] for c in client.calls], ['prepare'])
                self.assertFalse(getattr(client, 'music_requests', []))
                self.assertFalse(ros.services)
                self.assertFalse(ros.published)
            finally:
                bundle.close()

    def test_base_drive_cannot_overlap_native_navigation_and_stop_disables_body_first(self):
        from test_driver import driver, FakeRos1, FakeNode, fixture_joint_limits
        with tempfile.TemporaryDirectory() as folder, patch.object(driver, '_acp_complete'):
            ros = FakeRos1()
            bundle = driver.KuavoBundle({'sensors': {'joint_limits': fixture_joint_limits(ros)}},
                                       'test', ros, FakeNode())
            bundle.spatial.close()
            client = Client()
            bundle.spatial = ControlledSpatialPlugin(bundle, {'db_path': folder + '/db'}, client)
            bundle.sensors.discover()
            ros.raw()
            ros.emit('/enable_control_state', True)
            ros.emit('/mobile_manipulator/lb_mpc_control_mode', 3)
            bundle.state.verify_enable()
            entered, release = threading.Event(), threading.Event()
            original = client.native
            def blocked(operation, **args):
                if operation == 'navigate':
                    entered.set()
                    release.wait(3)
                return original(operation, **args)
            client.native = blocked
            try:
                result = bundle.dispatch('controlled_spatial', {'action': 'navigate_to_pose',
                                          'x': 1, 'y': 2, 'yaw': .3})
                self.assertEqual(result['state'], 'executing')
                self.assertTrue(entered.wait(1))
                result = bundle.dispatch('base_drive', {'action': 'move', 'vx': .1, 'vy': 0,
                                                        'vyaw': 0, 'duration': .1})
                self.assertEqual(result['state'], 'error')
                stopped = []
                thread = threading.Thread(target=lambda: stopped.append(bundle.pause('test')))
                thread.start()
                wait(lambda: ros.latched['/enable_control_state'] is False)
                self.assertFalse(stopped)
                release.set()
                thread.join(3)
                self.assertEqual(stopped[0]['state'], 'paused')
            finally:
                release.set()
                bundle.close()


class NativePreparationTests(unittest.TestCase):
    def setUp(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('native_prepare_test',
                      Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.state = types.SimpleNamespace(estop=False, bumper=False, driver_error=False, soft_estop=True)
        self.events, self.reject, self.fail_after_release = [], None, False
        classes = {name: type(name, (), {'_type': 'leju_mobile_base_msgs/' + name, '_md5sum': name})
                   for name in ('MoveToTarget', 'CheckArrived', 'SetDispatchMode')}
        names = {'/move_base/move_to_target': classes['MoveToTarget']._type,
                 '/move_base/check_arrived': classes['CheckArrived']._type,
                 '/move_base/set_dispatch_mode': classes['SetDispatchMode']._type}
        def command(control_mode):
            self.events.append(control_mode)
            if self.reject == 'transport': raise ConnectionError('remote service reply lost')
            if control_mode == 1: self.state.soft_estop = False
            if control_mode == 0: self.state.soft_estop = True
            return types.SimpleNamespace(success=control_mode != self.reject)
        def sample(*a, **k):
            if self.fail_after_release and 1 in self.events:
                raise TimeoutError('feedback lost')
            return copy.copy(self.state)
        def proxy(name, _type):
            if name != '/move_base/move_to_target':
                return command
            def move(**kwargs):
                self.events.append(('goal', kwargs))
                return types.SimpleNamespace(__slots__=('success', 'task_id'),
                                             success=True, task_id='task-1')
            return move
        ros = types.SimpleNamespace(ERROR=1, init_node=lambda *a, **k: None,
                                   ServiceProxy=proxy, wait_for_message=sample)
        modules = {'rospy': ros,
            'rosgraph': types.SimpleNamespace(Master=lambda *a: types.SimpleNamespace(
                getSystemState=lambda: [[], [], [(n, ['/leju_node']) for n in names]], lookupService=lambda n: n)),
            'rosservice': types.SimpleNamespace(get_service_type=lambda n: names[n],
                get_service_headers=lambda n, uri: {'type': names[n], 'md5sum': names[n].split('/')[-1]}),
            'leju_mobile_base_msgs': types.ModuleType('leju_mobile_base_msgs'),
            'leju_mobile_base_msgs.srv': types.SimpleNamespace(**classes),
            'leju_mobile_base_msgs.msg': types.SimpleNamespace(MoveToTargetOptions=types.SimpleNamespace),
            'nav_msgs': types.ModuleType('nav_msgs'), 'nav_msgs.msg': types.SimpleNamespace(Odometry=object),
            'jaten_msgs': types.ModuleType('jaten_msgs'),
            'jaten_msgs.msg': types.SimpleNamespace(RobotChassisState=object)}
        self.module_patch = patch.dict(sys.modules, modules)
        self.module_patch.start(); self.addCleanup(self.module_patch.stop)
        self.env_patch = patch.dict(self.module.os.environ)
        self.env_patch.start(); self.addCleanup(self.env_patch.stop)
        self.stationary_patch = patch.object(self.module, 'confirm_stationary',
                                              side_effect=lambda *a, **kw: self.events.append('stationary'))
        self.stationary_patch.start(); self.addCleanup(self.stationary_patch.stop)
        self.tracker_patch = patch.object(self.module, 'tracker_command',
                                          side_effect=lambda ros, op, request: self.events.append(('tracker', op)))
        self.tracker_patch.start(); self.addCleanup(self.tracker_patch.stop)
        self.mux_patch = patch.object(self.module, 'select_navigation_source',
                                      side_effect=lambda ros: self.events.append('mux'))
        self.mux_patch.start(); self.addCleanup(self.mux_patch.stop)
        self.stop_feedback_patch = patch.object(self.module, 'confirm_software_stop')
        self.stop_feedback_patch.start(); self.addCleanup(self.stop_feedback_patch.stop)
        self.path_feedback_patch = patch.object(self.module, 'confirm_tracker_target',
                                                side_effect=lambda *a: self.events.append('new_path'))
        self.path_feedback_patch.start(); self.addCleanup(self.path_feedback_patch.stop)

    def call(self, operation='prepare'):
        return self.module.main({'master': 'http://test:11311', 'ros_ip': '127.0.0.1', 'operation': operation})

    def test_navigation_returns_dispatch_control_before_sending_goal(self):
        request = {'master': 'http://test:11311', 'ros_ip': '127.0.0.1', 'operation': 'navigate',
                   'args': {'x': 1., 'y': 2., 'yaw': .3, 'speed': .5}}
        self.reject = 4
        self.assertEqual(self.module.main(request), {'success': False, 'stage': 'dispatch_handoff'})
        self.assertEqual(self.events, [0, 2, ('tracker', 3), 'stationary', 4])

        self.reject = None
        self.events.clear()
        self.assertEqual(self.module.main(request), {'success': True, 'task_id': 'task-1'})
        self.assertEqual(self.events[:6], [0, 2, ('tracker', 3), 'stationary', 4, 'mux'])
        goal, kwargs = self.events[6]
        self.assertEqual(goal, 'goal')
        self.assertEqual(self.events[7:], ['new_path', ('tracker', 4), 'mux', 1])
        self.assertEqual(kwargs['options'].linear_velocity, .5)
        self.assertTrue(kwargs['options'].avoid_enabled)

    def test_rpc_distinguishes_completed_rejection_from_uncertain_transport(self):
        import io
        import runpy
        for rejection, uncertain in ((2, False), ('transport', True)):
            self.reject = rejection
            output = io.StringIO()
            request = {'master': 'http://test:11311', 'ros_ip': '127.0.0.1', 'operation': 'prepare'}
            with patch.object(sys, 'stdin', io.StringIO(json.dumps(request))), patch.object(sys, 'stdout', output):
                runpy.run_path(str(Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py'), run_name='__main__')
            response = json.loads(output.getvalue())
            self.assertTrue(response['ok'])
            self.assertFalse(response['result']['success'])
            self.assertIs(response['result']['delivery_uncertain'], uncertain)

    def test_manual_auto_guard_reads_zero_sources_without_releasing_or_switching(self):
        self.state.manual_mode, self.state.mapping, self.state.driver_enable = True, False, True
        self.state.motion_type = 2
        with patch.object(sys.modules['jaten_msgs.msg'], 'MotionState', object, create=True), \
                patch.object(self.module, 'tracker_live_message', return_value=True), \
                patch.object(self.module, 'remote_idle_guard', return_value=(lambda:None,lambda:self.events.append('closed'))):
            result = self.call('manual_auto_guard')
            self.assertTrue(result['upper_zero'])
            self.assertTrue(result['native_tracker_idle'])
            self.assertEqual(self.events, ['stationary','closed'])
            self.state.motion_type = 8
            self.events.clear()
            with self.assertRaisesRegex(RuntimeError, 'native_tracker_must_finish_before_auto'):
                self.call('manual_auto_guard')
            self.assertFalse(self.events)

    def test_stationary_probe_reads_odom_without_cancel_or_release(self):
        result = self.call('stationary')
        self.assertTrue(result['stationary'])
        self.assertEqual(self.events, ['stationary'])

    def test_upper_handoff_selects_chassis_mux_before_release(self):
        self.state.manual_mode,self.state.mapping,self.state.driver_enable=False,False,True
        with patch.object(self.module,'remote_idle_guard',return_value=(lambda:None,lambda:None)) as guard, \
                patch.object(self.module,'select_navigation_source',side_effect=lambda *args:self.events.append('upper_mux')):
            result=self.module.main({'master':'http://chassis:11311','upper_master':'http://upper:11311',
                                     'ros_ip':'127.0.0.1','operation':'upper_handoff'})
        self.assertEqual(result['velocity_source'],'nav_mpc_move_base')
        self.assertLess(self.events.index('upper_mux'),self.events.index(1))
        self.assertFalse(guard.call_args.kwargs['require_joy'])

    def test_manual_idle_probe_is_read_only_without_requiring_software_stop(self):
        self.state.manual_mode, self.state.mapping, self.state.driver_enable = True, False, True
        self.state.soft_estop, self.state.motion_type = False, 2
        with patch.object(sys.modules['jaten_msgs.msg'], 'MotionState', object, create=True), \
                patch.object(self.module, 'tracker_live_message', return_value=True):
            result = self.call('manual_idle_stationary')
            self.assertTrue(result['manual_mode'])
            self.assertTrue(result['native_tracker_idle'])
            self.assertEqual(self.events, ['stationary'])
            self.events.clear()
            self.state.motion_type = 8
            with self.assertRaisesRegex(RuntimeError, 'native_tracker_not_finished'):
                self.call('manual_idle_stationary')
            self.assertFalse(self.events)
            self.state.motion_type, self.state.mapping = 2, True
            with self.assertRaisesRegex(RuntimeError, 'chassis_manual_idle_state_invalid'):
                self.call('manual_idle_stationary')

    def test_held_stationary_probe_is_read_only_and_requires_finished_tracker(self):
        message = types.SimpleNamespace(motion_type=2,
            _connection_header={'callerid':'/lqr_path_track','latching':'0'})
        with patch.dict(sys.modules, {'jaten_msgs.msg':types.SimpleNamespace(MotionState=object)}), \
                patch.object(sys.modules['rospy'], 'wait_for_message', return_value=message):
            result = self.call('held_stationary')
        self.assertTrue(result['software_estop'])
        self.assertEqual(self.events,['stationary'])
        for motion_type,header in [(1,message._connection_header),(2,{'callerid':'/other','latching':'0'})]:
            message.motion_type,message._connection_header=motion_type,header
            with patch.dict(sys.modules, {'jaten_msgs.msg':types.SimpleNamespace(MotionState=object)}), \
                    patch.object(sys.modules['rospy'], 'wait_for_message', return_value=message):
                with self.assertRaisesRegex(RuntimeError,'not_finished'):
                    self.call('held_stationary')

    def test_cancel_and_stationary_before_release_then_stationary_again(self):
        result = self.call()
        self.assertEqual(self.events, [0, 2, ('tracker', 3), 'stationary', 1, 'stationary'])
        self.assertTrue(result['old_task_cancelled'])
        self.assertFalse(result['software_estop'])

    def test_clear_state_is_held_while_stale_tracker_is_paused(self):
        self.state.soft_estop = False
        self.assertTrue(self.call()['released'])
        self.assertEqual(self.events, [0, 2, ('tracker', 3), 'stationary', 1, 'stationary'])

    def test_hardware_fault_is_not_bypassed(self):
        for flag in ('estop', 'bumper', 'driver_error'):
            setattr(self.state, flag, True)
            with self.assertRaises(RuntimeError): self.call()
            setattr(self.state, flag, False)
        self.assertFalse(self.events)

    def test_rejected_cancel_never_releases(self):
        self.reject = 2
        with self.assertRaises(RuntimeError): self.call()
        self.assertEqual(self.events, [0, 2])

    def test_missing_stationary_samples_never_releases(self):
        with patch.object(self.module, 'confirm_stationary', side_effect=TimeoutError()):
            with self.assertRaises(TimeoutError): self.call()
        self.assertEqual(self.events, [0, 2, ('tracker', 3)])

    def test_feedback_loss_after_release_reasserts_stop(self):
        self.fail_after_release = True
        with self.assertRaises(TimeoutError): self.call()
        self.assertEqual(self.events, [0, 2, ('tracker', 3), 'stationary', 1, 0])
        self.assertTrue(self.state.soft_estop)

    def test_interrupt_stop_asserts_stop_and_cancels_before_confirmation(self):
        self.state.soft_estop = False
        result = self.call('soft_stop')
        self.assertEqual(self.events, [0, 2, ('tracker', 3), 'stationary'])
        self.assertTrue(result['software_estop'])

    def test_tracker_pause_failure_never_selects_mux_or_releases_stop(self):
        with patch.object(self.module, 'tracker_command', side_effect=RuntimeError('pause_unconfirmed')):
            for operation in ('prepare', 'navigate'):
                self.events.clear()
                request = {'master': 'http://test:11311', 'ros_ip': '127.0.0.1', 'operation': operation,
                           'args': {'x': 1., 'y': 2., 'yaw': .3, 'speed': .1}}
                with self.assertRaisesRegex(RuntimeError, 'pause_unconfirmed'):
                    self.module.main(request)
                self.assertNotIn(1, self.events)
                self.assertNotIn('mux', self.events)
                self.assertTrue(self.state.soft_estop)

    def test_mux_failure_never_submits_goal_and_retains_stop(self):
        with patch.object(self.module, 'select_navigation_source', side_effect=TimeoutError('mux')):
            with self.assertRaises(TimeoutError):
                self.module.main({'master': 'http://test:11311', 'ros_ip': '127.0.0.1',
                                  'operation': 'navigate',
                                  'args': {'x': 1., 'y': 2., 'yaw': .3, 'speed': .1}})
        self.assertEqual(self.events, [0, 2, ('tracker', 3), 'stationary', 4, 0])
        self.assertTrue(self.state.soft_estop)

    def test_unconfirmed_software_stop_never_switches_source(self):
        with patch.object(self.module, 'confirm_software_stop', side_effect=TimeoutError('stop')):
            with self.assertRaises(TimeoutError):
                self.module.main({'master': 'http://test:11311', 'ros_ip': '127.0.0.1',
                                  'operation': 'navigate',
                                  'args': {'x': 1., 'y': 2., 'yaw': .3, 'speed': .1}})
        self.assertEqual(self.events, [0, 0])
        self.assertTrue(self.state.soft_estop)

    def test_resume_failure_after_accepted_goal_reasserts_stop(self):
        def tracker(ros, op, request):
            if op == 4:
                raise RuntimeError('resume_unconfirmed')
        with patch.object(self.module, 'tracker_command', side_effect=tracker):
            with self.assertRaisesRegex(RuntimeError, 'resume_unconfirmed'):
                self.module.main({'master': 'http://test:11311', 'ros_ip': '127.0.0.1',
                                  'operation': 'navigate',
                                  'args': {'x': 1., 'y': 2., 'yaw': .3, 'speed': .1}})
        self.assertNotIn(1, self.events)
        self.assertEqual(self.events[-1], 0)
        self.assertTrue(self.state.soft_estop)


class DispatchAbiTests(unittest.TestCase):
    def test_factory_header_fallback_requires_matching_hash_and_exact_fields(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('dispatch_abi_test',
                      Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        expected = '09d13be4b7fff0abe4e61d99b81ca03b'
        installed = types.SimpleNamespace(_md5sum='stale')
        generated = []
        def generate(name, definition):
            generated.append((name, definition))
            return {name: type(name.split('/')[-1], (), {})}
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'SetDispatchMode.h').write_text('return "' + expected + '";')
            def header(name, definition):
                (root / name).write_text('struct Definition< T > { return ' + json.dumps(definition) + '; }')
            header('SetDispatchModeRequest.h', '# command\nuint8 control_mode\n')
            header('SetDispatchModeResponse.h', 'bool success\nstring message\n')
            modules = {'genpy': types.ModuleType('genpy'), 'genpy.dynamic': types.SimpleNamespace(generate_dynamic=generate)}
            with patch.object(module, 'Path', return_value=root), patch.dict(sys.modules, modules):
                service = module.dispatch_service_class(installed, expected)
                self.assertEqual(service._md5sum, expected)
                self.assertEqual(service._type, 'leju_mobile_base_msgs/SetDispatchMode')
                self.assertEqual(generated[0][1], 'uint8 control_mode\n')
                with self.assertRaises(RuntimeError):
                    module.dispatch_service_class(installed, 'unrecognized')
                header('SetDispatchModeRequest.h', 'string control_mode\n')
                with self.assertRaises(RuntimeError):
                    module.dispatch_service_class(installed, expected)

    def test_matching_installed_class_is_used_without_reading_headers(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('dispatch_abi_match',
                      Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
        module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        service = types.SimpleNamespace(_md5sum='matching')
        with patch.object(module, 'Path', side_effect=AssertionError('no fallback needed')):
            self.assertIs(module.dispatch_service_class(service, 'matching'), service)


class NativeIdentityTests(unittest.TestCase):
    def test_native_cancel_requires_distinct_post_cancel_stationary_samples(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('native_cancel_test',
                      Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        classes = {name: type(name, (), {'_type': 'leju_mobile_base_msgs/' + name, '_md5sum': name})
                   for name in ('MoveToTarget', 'CheckArrived', 'SetDispatchMode')}
        names = {'/move_base/move_to_target': classes['MoveToTarget']._type,
                 '/move_base/check_arrived': classes['CheckArrived']._type,
                 '/move_base/set_dispatch_mode': classes['SetDispatchMode']._type}
        vector = types.SimpleNamespace(x=0., y=0., z=0.)
        def odom(stamp):
            return types.SimpleNamespace(header=types.SimpleNamespace(
                stamp=types.SimpleNamespace(to_sec=lambda: stamp)),
                twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=vector, angular=vector)))
        ros = types.SimpleNamespace(ERROR=1, init_node=lambda *a, **k: None,
            ServiceProxy=lambda *a: lambda **k: types.SimpleNamespace(success=True))
        modules = {'rospy': ros,
            'rosgraph': types.SimpleNamespace(Master=lambda *a: types.SimpleNamespace(
                getSystemState=lambda: [[], [], [(n, ['/leju_node']) for n in names]], lookupService=lambda n: n)),
            'rosservice': types.SimpleNamespace(get_service_type=lambda n: names[n],
                get_service_headers=lambda n, uri: {'type': names[n], 'md5sum': names[n].split('/')[-1]}),
            'leju_mobile_base_msgs': types.ModuleType('leju_mobile_base_msgs'),
            'leju_mobile_base_msgs.srv': types.SimpleNamespace(**classes),
            'leju_mobile_base_msgs.msg': types.SimpleNamespace(MoveToTargetOptions=object),
            'nav_msgs': types.ModuleType('nav_msgs'),
            'nav_msgs.msg': types.SimpleNamespace(Odometry=object)}
        request = {'master': 'http://test:11311', 'ros_ip': '127.0.0.1', 'operation': 'cancel'}
        with patch.dict(sys.modules, modules), patch.dict(module.os.environ), \
             patch.object(module, 'tracker_command'), patch.object(module, 'confirm_software_stop'):
            for stamps, accepted in (([9.9] * 4, False), ([10.1] * 4, False),
                                     ([10.1, 10.2, 10.41], True)):
                ros.wait_for_message = unittest.mock.Mock(side_effect=[odom(t) for t in stamps])
                with patch.object(module.time, 'time', side_effect=[10.] + [10.45] * 10), \
                     patch.object(module.time, 'monotonic', side_effect=range(7)):
                    if accepted:
                        self.assertTrue(module.main(request)['stationary'])
                    else:
                        with self.assertRaisesRegex(RuntimeError, 'stop_not_confirmed'):
                            module.main(request)

    def test_grid_and_coordinate_config_both_affect_identity(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('navigation_rpc_test',
                      Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'map.pgm').write_bytes(b'P5 image')
            (root / 'map.yaml').write_text('resolution: 0.05')
            with patch.object(module, 'Path', return_value=root):
                request = {'operation': 'map_identity', 'args': {'map_name': 'map'}}
                first = module.main(request)['sha256']
                (root / 'map.yaml').write_text('resolution: 0.10')
                self.assertNotEqual(first, module.main(request)['sha256'])
                with self.assertRaises(ValueError):
                    module.main({'operation': 'map_identity', 'args': {'map_name': '../escape'}})

    def test_save_completion_requires_final_valid_path_after_all_outputs(self):
        import importlib.util
        import os
        spec = importlib.util.spec_from_file_location('navigation_save_test',
                      Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for suffix in ('.pgm', '.yaml', '.pbstream'):
                (root / ('map' + suffix)).write_bytes(b'output')
            self.assertFalse(module.map_save_status('map', root)['complete'])
            path = root / 'map.path'
            path.write_text('{')
            self.assertFalse(module.map_save_status('map', root)['complete'])
            path.write_text('{}')
            self.assertTrue(module.map_save_status('map', root)['complete'])
            os.utime(path, ns=(1, 1))
            self.assertFalse(module.map_save_status('map', root)['complete'])
            with self.assertRaises(ValueError):
                module.map_save_status('../map', root)


if __name__ == '__main__':
    unittest.main()
