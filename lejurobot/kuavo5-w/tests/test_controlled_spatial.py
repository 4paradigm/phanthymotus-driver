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
from spatial_client import JatenClient


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

    def begin(self, channel, tool):
        self.counter += 1
        aid, event = str(self.counter), threading.Event()
        self._active[channel] = aid, event
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
            for channel, (aid, event) in list(self._active.items()):
                event.set()
                self.finish(channel, aid, 'controlled_spatial', status, {'reason': reason})


class Bundle:
    def __init__(self):
        self.tracker = Tracker()
        self._stop_epoch, self._transition = 0, None
        self.state = types.SimpleNamespace(require_motion=lambda modes: None,
                                           require_motion_enabled=lambda: None)
        self.ros1 = types.SimpleNamespace(connection=(True, 1))
        self.pauses = []

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

    def pause(self, reason, status='cancelled', expected=None):
        with self.tracker.lock:
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
        self.calls.append((operation, args))
        if operation == 'navigate':
            return self.navigate
        if operation == 'check':
            return self.check
        if operation == 'cancel':
            return {'success': True, 'stationary': True}
        return {'owner': '/leju_node'}

    def current_map(self):
        return {'mapId': self.identity[0], 'mapMd5': 'road-A'}

    def map_identity(self):
        return self.identity

    def maps(self):
        return self.grid_maps

    def observe(self):
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
        return self.plugin.dispatch(action, args)

    def navigate(self, **args):
        return self.call('navigate_to_pose', x=1., y=2., yaw=.3, **args)

    def completed(self):
        wait(lambda: bool(self.bundle.tracker.events))
        return self.bundle.tracker.events[-1]

    def test_contract_lifecycle_resource_and_acp(self):
        schema = self.plugin.get_tool()['inputSchema']
        self.assertEqual(schema['x-resource'], 'base')
        self.assertEqual(schema['x-hooks']['on_interrupt_motion'], {'action': 'stop_nav'})
        self.assertEqual(set(schema['properties']['action']['enum']), set(schema['x-action-params']))
        self.assertIn('load_map', schema['x-completion']['actions'])
        self.assertEqual(self.call('start')['state'], 'running')
        self.assertEqual(self.call('stop')['state'], 'idle')
        self.assertFalse(any(c[0] == 'navigate' for c in self.client.calls))

    def test_arrival_uses_exact_native_task_id(self):
        result = self.navigate()
        self.assertEqual(result['state'], 'executing')
        status, details = self.completed()
        self.assertEqual(status, 'completed')
        self.assertEqual(details['task_id'], 'native-task-1')
        self.assertIn(('check', {'task_id': 'native-task-1'}), self.client.calls)
        self.assertFalse(self.plugin.unresolved())

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
        self.navigate()
        wait(lambda: any(c[0] == 'check' for c in self.client.calls))
        time.sleep(.3)
        self.assertFalse(self.bundle.tracker.events)

    def test_safety_faults_prevent_submit(self):
        for field in ('softwareEstop', 'estopButtonEvent', 'bumperEvent', 'driverError'):
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
        self.assertEqual(self.call('start_mapping', map_name='new')['state'], 'executing')
        self.assertEqual(self.completed()[0], 'completed')
        self.assertEqual(self.plugin.store.state('mapping')['phase'], 'mapping')
        self.assertEqual(self.navigate()['state'], 'error')
        self.call('tag_place', name='entry')
        self.bundle.tracker.events.clear()
        self.assertEqual(self.call('stop_mapping')['state'], 'executing')
        self.assertEqual(self.completed()[0], 'completed')
        self.assertIsNone(self.plugin.store.state('mapping'))
        self.assertEqual(self.call('list_tags')['tags'][0]['name'], 'entry')
        self.assertEqual(self.client.commands[-1], ('ChangeMode', {'mode': 'AUTO'}))
        self.assertFalse(self.plugin.unresolved())

    def test_loading_does_not_clear_stop_or_forge_localization(self):
        self.client.feedback['agvState']['softwareEstop'] = True
        self.client.feedback['state']['agvPosition']['positionInitialized'] = False
        self.assertEqual(self.call('load_map', map_name='other')['state'], 'executing')
        self.assertEqual(self.completed()[0], 'completed')
        self.assertTrue(self.client.feedback['agvState']['softwareEstop'])
        self.assertFalse(self.client.feedback['state']['agvPosition']['positionInitialized'])
        self.assertEqual(self.navigate()['state'], 'error')

    def test_invalid_maps_are_rejected_without_latching_busy(self):
        for action, args in (('load_map', {'map_name': 'missing'}),
                             ('start_mapping', {'map_name': 'factory'}),
                             ('start_mapping', {'map_name': '../escape'}), ('stop_mapping', {})):
            self.assertEqual(self.call(action, **args)['state'], 'error')
        self.assertFalse(self.plugin.unresolved())
        self.assertFalse(self.client.commands)

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
        with patch.dict(sys.modules, {'websocket': ws}):
            result = client.command('ChangeMode', {'mode': 'MANUAL'})
            self.assertEqual(result['method'], 'ChangeMode')
            self.assertNotEqual(result['id'], 'other')
            with self.assertRaises(ValueError): client.command('ClearEstop', {})


class BundleIntegrationTests(unittest.TestCase):
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


class NativeIdentityTests(unittest.TestCase):
    def test_native_cancel_requires_distinct_post_cancel_stationary_samples(self):
        import importlib.util
        spec = importlib.util.spec_from_file_location('native_cancel_test',
                      Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        classes = {name: type(name, (), {'_type': 'leju_mobile_base_msgs/' + name})
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
                getSystemState=lambda: [[], [], [(n, ['/leju_node']) for n in names]])),
            'rosservice': types.SimpleNamespace(get_service_type=lambda n: names[n]),
            'leju_mobile_base_msgs': types.ModuleType('leju_mobile_base_msgs'),
            'leju_mobile_base_msgs.srv': types.SimpleNamespace(**classes),
            'leju_mobile_base_msgs.msg': types.SimpleNamespace(MoveToTargetOptions=object),
            'nav_msgs': types.ModuleType('nav_msgs'),
            'nav_msgs.msg': types.SimpleNamespace(Odometry=object)}
        request = {'master': 'http://test:11311', 'ros_ip': '127.0.0.1', 'operation': 'cancel'}
        with patch.dict(sys.modules, modules), patch.dict(module.os.environ):
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


if __name__ == '__main__':
    unittest.main()
