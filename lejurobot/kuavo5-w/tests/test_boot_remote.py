"""First-boot G12 handoff is one-shot and subordinate to every card/stop."""
import copy
import importlib.util
import json
import sys
import tempfile
import threading
import time
import types
from pathlib import Path
from unittest.mock import patch

import test_driver as fixtures
from test_controlled_spatial import Client
from controlled_spatial import SpatialStore
from boot_remote import BootRemote

def host_module(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).resolve().parents[1]/'host'/f'{name}.py')
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value

native = host_module('navigation_rpc')
controller = host_module('controller_boot')


class BootRemoteTests(fixtures.unittest.TestCase):
    setUpDriver = fixtures.DriverContractTests.setUp
    tearDownDriver = fixtures.DriverContractTests.tearDown

    def setUp(self):
        self.setUpDriver()
        self.temp = tempfile.TemporaryDirectory()
        self.old_store = self.bundle.spatial.store
        self.bundle.spatial.store = SpatialStore(self.temp.name+'/db')
        self.client = Client()
        self.node.create_subscription = lambda kind, topic, callback, depth: callback
        self.node.destroy_subscription = lambda subscription: None
        self.bundle.spatial.client = self.client
        self.remote = self.bundle.boot_remote
        self.read_unavailable = self.remote.attempt.__globals__['NavigationStateUnavailable']
        self.remote.enabled = True
        self.remote.status = {'state': 'waiting'}
        # The ordinary fixture disables automatic boot behavior; emulate the
        # Linux host identity only for this feature's explicitly enabled cases.
        self.host_identity = patch('boot_remote.Path.read_text',
            return_value='b3e4392d-0d0e-43a2-bbe3-1f6d8a81f24a')
        self.host_identity.start()
        self.addCleanup(self.host_identity.stop)
        self.boot = {'state': 'complete', 'boot_pause_confirmed': True,
            'initialization_requested': True, 'initialization_accepted': True,
            'launch_ownership': 'service', 'initial_control_mode': 3,
            'boot_id': 'e0f4fcc6-f92f-431b-b874-d927074f915b',
            'ready_age_s': 1., 'g12_owner': '/actual_sdk_converter', 'g12_age_s': .01, 'g12_neutral': True}
        self.feedback()
        self.ros.emit('/enable_control_state', False)
        original = self.client.native
        def native(operation, **kwargs):
            if operation == 'remote_ready':
                self.client.calls.append((operation, kwargs))
                self.client.feedback['agvState']['softwareEstop'] = False
                return {'success': True, 'stationary': True, 'software_estop': False, 'old_task_cancelled': True}
            return original(operation, **kwargs)
        self.native_patch = patch.object(self.client, 'native', side_effect=native)
        self.native_patch.start()

    def tearDown(self):
        self.native_patch.stop()
        self.tearDownDriver()
        self.old_store.close()
        self.temp.cleanup()

    def feedback(self, **values):
        self.boot.update(values)
        self.bundle.state._boot_update({'data': json.dumps(self.boot)})

    def test_first_boot_prepares_control_without_any_nonzero_or_joint_target(self):
        self.assertTrue(self.remote.attempt())
        self.assertEqual(self.remote.info()['state'], 'ready', self.remote.info())
        self.assertEqual([call[0] for call in self.client.calls], ['soft_stop', 'remote_ready'])
        self.assertEqual(self.client.calls[-1][1]['g12_owner'], '/actual_sdk_converter')
        self.assertEqual([s[:3] for s in self.ros.sent_services], [
            ('/enable_vel_control', 'std_srvs/SetBool', {'data': False}),
            ('/enable_control', 'std_srvs/SetBool', {'data': True}),
            ('/enable_vel_control', 'std_srvs/SetBool', {'data': True})])
        for topic, _, message in self.ros.published:
            self.assertEqual(topic, '/cmd_vel')
            self.assertTrue(all(value == 0 for group in message.values() for value in group.values()))
        self.assertFalse(self.bundle.state.motion_blocked)
        self.bundle.upper_bridge.ensure.assert_called_once()
        self.assertEqual(self.bundle._upper_route_key,
                         (self.bundle._stop_epoch, self.bundle.ros1.connection))
        self.assertEqual(self.bundle.upper_bridge.owner(), 'remote')

    def test_missing_velocity_bridge_never_enables_remote_control(self):
        self.bundle._upper_route_key = None
        self.bundle.upper_bridge.ensure.side_effect = RuntimeError('upper_velocity_bridge_not_ready')
        self.assertTrue(self.remote.attempt())
        self.assertEqual(self.remote.info()['state'], 'error')
        self.assertFalse(self.ros.sent_services)
        self.assertIsNone(self.bundle._upper_route_key)

    def test_transient_chassis_read_failure_retries_before_control_writes(self):
        original = self.client.observe
        calls = []
        def observe(**kwargs):
            calls.append(kwargs)
            if len(calls) == 1:
                self.assertFalse(self.ros.sent_services)
                self.assertFalse(self.client.calls)
                raise self.read_unavailable(('agvState', 'state'), 'connection')
            return original(**kwargs)
        with patch.object(self.client, 'observe', side_effect=observe), \
                patch.object(self.remote.closed, 'wait', return_value=False):
            self.remote.attempt()
        self.assertEqual(self.remote.info()['state'], 'ready')
        self.assertEqual(len(calls), 2)
        self.assertEqual([c[0] for c in self.client.calls], ['soft_stop', 'remote_ready'])

    def test_operator_stop_during_read_retry_prevents_enable(self):
        def interrupted_wait(_):
            self.remote.cancel('software_pause_requested')
        with patch.object(self.client, 'observe', side_effect=self.read_unavailable(('state',), 'connection')), \
                patch.object(self.remote.closed, 'wait', side_effect=interrupted_wait):
            self.remote.attempt()
        self.assertFalse(self.ros.sent_services)
        self.assertFalse(self.client.calls)
        self.assertNotEqual(self.remote.info()['state'], 'ready')

    def test_chassis_read_retry_is_bounded_and_sends_no_control(self):
        self.remote.CHASSIS_READ_WAIT_S = 0.
        with patch.object(self.client, 'observe', side_effect=self.read_unavailable(('state',), 'connection')):
            self.remote.attempt()
        self.assertEqual(self.remote.info()['state'], 'error')
        self.assertFalse(self.ros.sent_services)
        self.assertFalse(self.client.calls)

    def test_container_restart_and_repeated_calls_never_replay_consumed_permit(self):
        self.remote.attempt()
        before = list(self.ros.sent_services)
        second = BootRemote(self.bundle, True)
        second.attempt()
        self.remote.attempt()
        self.assertEqual(self.ros.sent_services, before)
        self.assertEqual(second.info()['reason'], 'boot_permit_already_consumed')
        other = SpatialStore(self.temp.name+'/db')
        try:
            self.assertFalse(other.claim_boot_remote(self.boot['boot_id']))
        finally:
            other.close()

    def test_restart_before_controller_feedback_cannot_enroll_again(self):
        with patch('boot_remote.Path.read_text', return_value='4b3729c5-d872-4b89-9ae6-46c9db12b408'):
            first = BootRemote(self.bundle, True)
            second = BootRemote(self.bundle, True)
        self.assertTrue(first.enrolled)
        self.assertFalse(second.enrolled)
        second.start()
        self.assertIsNone(second.thread)
        self.assertEqual(second.info()['reason'], 'driver_already_started_this_host_boot')

    def test_missing_host_boot_identity_disables_automatic_handoff(self):
        with patch('boot_remote.Path.read_text', side_effect=OSError('not available')):
            remote = BootRemote(self.bundle, True)
        remote.start()
        self.assertFalse(remote.enrolled)
        self.assertIsNone(remote.thread)

    def test_bridge_reconnect_while_waiting_never_enables_later(self):
        self.feedback(state='initializing', boot_pause_confirmed=False)
        self.assertFalse(self.remote.attempt())
        self.ros.connection = (True, 2)
        self.remote.attempt()
        self.assertEqual(self.remote.info()['reason'], 'control_connection_changed_no_automatic_retry')
        self.assertEqual(self.ros.sent_services, [])

    def test_lifecycle_starts_preserve_ready_g12_and_servo_attaches_paused(self):
        self.remote.attempt()
        before = list(self.client.calls), list(self.ros.sent_services)
        for name in ('base_drive', 'controlled_spatial'):
            result = self.bundle.dispatch(name, {'action': 'start'})
            self.assertEqual(result['state'], 'ready', result)
            self.assertTrue(result['preparation_deferred'])
        with patch.dict(sys.modules, {'std_msgs.msg': fixtures.std}):
            result = self.bundle.dispatch('base_servo', {'action': 'start', 'input_topic': '/actucore/navi/cmd'})
        self.assertEqual(result['state'], 'ready', result)
        self.assertTrue(self.bundle.base_servo._paused)
        self.assertEqual((self.client.calls, self.ros.sent_services), before)

    def test_cancel_before_boot_feedback_is_persisted_when_boot_arrives(self):
        self.remote.cancel('stop_before_boot_feedback')
        self.remote.attempt()
        self.assertEqual(self.ros.sent_services, [])
        second = BootRemote(self.bundle, True)
        second.attempt()
        self.assertEqual(second.info()['reason'], 'boot_permit_already_consumed')

    def test_missing_old_external_failed_or_expired_permits_never_enable(self):
        for values in ({'boot_id': None}, {'launch_ownership': 'external'},
                       {'state': 'failed'}, {'ready_age_s': 61.}, {'ready_age_s': float('nan')}):
            with self.subTest(values=values):
                old = copy.deepcopy(self.boot)
                self.feedback(**values)
                self.remote.attempt()
                self.boot = old
        self.assertEqual(self.ros.sent_services, [])
        self.assertEqual(self.client.calls, [])

    def test_factory_mapping_manual_or_moving_state_has_no_control_side_effect(self):
        for field, value in (('isMapping', True), ('runningMode', 'MANUAL'), ('driving', True)):
            with self.subTest(field=field), tempfile.TemporaryDirectory() as folder:
                previous = self.bundle.spatial.store
                self.bundle.spatial.store = SpatialStore(folder+'/db')
                feedback = copy.deepcopy(self.client.feedback)
                try:
                    feedback['state' if field == 'driving' else 'agvState'][field] = value
                    with patch.object(self.client, 'observe', return_value=feedback):
                        BootRemote(self.bundle, True).attempt()
                    self.assertEqual(self.ros.sent_services, [])
                    self.assertEqual(self.client.calls, [])
                    self.assertIsNone(self.bundle._transition)
                finally:
                    self.bundle.spatial.store.close()
                    self.bundle.spatial.store = previous

    def test_stale_or_non_neutral_g12_skips_without_any_native_writes(self):
        self.feedback(g12_neutral=False)
        self.remote.attempt()
        self.assertEqual(self.client.calls, [])
        self.assertEqual(self.ros.sent_services, [])
        self.assertIsNone(self.bundle._transition)

    def test_active_mapping_navigation_or_any_joint_reservation_blocks_boot_handoff(self):
        for kind in ('mapping', 'navigation', 'arm'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as folder:
                store = self.bundle.spatial.store
                self.bundle.spatial.store = SpatialStore(folder+'/db')
                try:
                    if kind == 'mapping':
                        self.bundle.spatial.store.state('mapping', {'phase': 'mapping'})
                    elif kind == 'navigation':
                        self.bundle.spatial._task = {'task_id': 'existing'}
                    else:
                        self.bundle.tracker.begin('arms', 'arm', notify=False)
                    BootRemote(self.bundle, True).attempt()
                    self.assertEqual(self.ros.sent_services, [])
                    self.assertEqual(self.client.calls, [])
                finally:
                    self.bundle.spatial._task = None
                    self.bundle.tracker.cancel_all('test_end')
                    self.bundle.spatial.store.close()
                    self.bundle.spatial.store = store

    def test_lifecycle_starts_do_not_compete_with_or_cancel_first_boot_handoff(self):
        for name in ('base_drive', 'base_servo', 'controlled_spatial', 'head', 'arm', 'leg'):
            with self.subTest(name=name):
                self.remote.cancelled = False
                with patch.object(self.bundle.spatial, 'start_chassis', return_value={'state': 'ready'}):
                    self.bundle.dispatch(name, {'action': 'start'})
                self.assertFalse(self.remote.cancelled)

    def test_base_card_start_during_boot_waiting_only_reports_deferred_preparation(self):
        for name in ('base_drive', 'controlled_spatial'):
            result = self.bundle.dispatch(name, {'action': 'start'})
            self.assertEqual(result['state'], 'ready')
            self.assertTrue(result['preparation_deferred'])
        self.assertEqual(self.client.calls, [])
        self.assertEqual(self.ros.sent_services, [])

    def test_mode_change_or_neutral_loss_after_hold_repauses_without_release(self):
        def lose_neutral(*args, **kwargs):
            self.feedback(g12_neutral=False)
            return {'success': True, 'stationary': True, 'software_estop': True}
        with patch.object(self.client, 'native', side_effect=lose_neutral):
            self.remote.attempt()
        self.assertEqual(self.remote.info()['state'], 'error')
        self.assertFalse(any(s[0] == '/enable_control' and s[2]['data'] is True for s in self.ros.sent_services))
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_stop_supersedes_inflight_handoff_before_enable(self):
        entered, release = threading.Event(), threading.Event()
        original = self.client.native
        def hold(operation, **kwargs):
            if operation == 'soft_stop' and not entered.is_set():
                entered.set()
                release.wait(2)
            return original(operation, **kwargs)
        with patch.object(self.client, 'native', side_effect=hold):
            worker = threading.Thread(target=self.remote.attempt)
            worker.start()
            self.assertTrue(entered.wait(1))
            stopped = threading.Thread(target=lambda: self.bundle.pause('operator_stop'))
            stopped.start()
            self.assertTrue(fixtures.wait_until(lambda: self.remote.cancelled))
            release.set()
            worker.join(3)
            stopped.join(3)
        self.assertFalse(worker.is_alive())
        self.assertFalse(stopped.is_alive())
        self.assertFalse(any(s[0] == '/enable_control' and s[2]['data'] is True for s in self.ros.sent_services))

    def test_servo_start_during_boot_preparation_only_attaches_paused(self):
        token, _, _ = self.bundle._begin_control('boot_remote_ready')
        self.remote.token = token
        self.remote.status = {'state': 'preparing'}
        try:
            with patch.dict(sys.modules, {'std_msgs.msg': fixtures.std}):
                result = self.bundle.dispatch('base_servo', {'action': 'start', 'input_topic': '/actucore/navi/cmd'})
            self.assertEqual(result['state'], 'ready', result)
            self.assertTrue(self.bundle.base_servo._paused)
            self.assertFalse(self.remote.cancelled)
            self.assertIs(self.bundle._transition, token)
            self.assertEqual(self.client.calls, [])
            self.assertEqual(self.ros.sent_services, [])
        finally:
            self.bundle._end_control(token)
            self.remote.token = None

    def test_mapping_request_cancels_boot_and_cannot_issue_map_or_mode_commands(self):
        entered, release = threading.Event(), threading.Event()
        original = self.client.native
        def hold(operation, **kwargs):
            if operation == 'soft_stop' and not entered.is_set():
                entered.set()
                release.wait(2)
            return original(operation, **kwargs)
        with patch.object(self.client, 'native', side_effect=hold):
            worker = threading.Thread(target=self.remote.attempt)
            worker.start()
            self.assertTrue(entered.wait(1))
            try:
                result = self.bundle.dispatch('controlled_spatial', {'action': 'start_mapping', 'map_name': 'new'})
                self.assertEqual(result['state'], 'error', result)
                self.assertTrue(self.remote.cancelled)
                self.assertEqual(self.client.commands, [])
            finally:
                release.set()
                worker.join(3)
        self.assertFalse(worker.is_alive())
        self.assertFalse(any(s[0] == '/enable_control' and s[2]['data'] is True for s in self.ros.sent_services))

    def test_final_release_failure_repauses_and_has_no_automatic_retry(self):
        original = self.client.native
        def fail(operation, **kwargs):
            if operation == 'remote_ready':
                raise RuntimeError('local_chassis_guard_failed')
            return original(operation, **kwargs)
        with patch.object(self.client, 'native', side_effect=fail):
            self.remote.attempt()
        self.assertEqual(self.remote.info()['state'], 'error')
        self.assertIs(self.ros.latched.get('/enable_control_state'), False)
        count = len(self.ros.sent_services)
        BootRemote(self.bundle, True).attempt()
        self.assertEqual(len(self.ros.sent_services), count)


class G12GuardTests(fixtures.unittest.TestCase):
    def message(self, owner='/sdk', axes=None, buttons=None, latching='0'):
        return types.SimpleNamespace(axes=axes if axes is not None else [0., 0., 1., 0., 0., 1., 0., 0.],
            buttons=buttons if buttons is not None else [0]*11,
            _connection_header={'callerid': owner, 'latching': latching})

    def test_both_guards_use_vendor_stick_mapping_and_reject_held_buttons(self):
        for check in (native.remote_joy_neutral, controller.g12_neutral):
            self.assertTrue(check(self.message(), '/sdk'))
            for axis in (0, 1, 3, 4, 6, 7):
                axes = list(self.message().axes)
                axes[axis] = .04
                self.assertFalse(check(self.message(axes=axes), '/sdk'))
            for button in range(11):
                buttons = [0]*11
                buttons[button] = 1
                self.assertFalse(check(self.message(buttons=buttons), '/sdk'))
            for message in (self.message(owner='/other'), self.message(latching='1'),
                            self.message(axes=[0]*7), self.message(buttons=[0]*8),
                            self.message(axes=[float('nan')]*8)):
                self.assertFalse(check(message, '/sdk'))

    def run_guard(self, repeats=True, output=.0, interruption=None, cross_master=False):
        clock, callbacks, closed = [10.], {}, []
        def subscribe(topic, _, callback, **kwargs):
            callbacks[topic] = callback
            return types.SimpleNamespace(unregister=lambda: closed.append(topic))
        def emit():
            callbacks['/joy'](self.message())
            callbacks['/move_base/base_cmd_vel'](types.SimpleNamespace(
                linear=types.SimpleNamespace(x=output, y=0., z=0.),
                angular=types.SimpleNamespace(x=0., y=0., z=0.),
                _connection_header={'callerid': '/nodelet_manager', 'latching': '1'}))
        def sleep(_):
            clock[0] += .05
            if repeats or len(closed) == 0 and clock[0] < 10.1:
                emit()
        upper_subscriptions = []
        def upper_subscribe(master, topic, kind, owner, callback):
            self.assertEqual(master, 'http://upper:11311')
            self.assertEqual(owner, '/sdk' if topic == '/joy' else '/nodelet_manager')
            subscription = subscribe(topic, kind, callback)
            subscription.failed = threading.Event()
            upper_subscriptions.append(subscription)
            return subscription
        rospy = types.SimpleNamespace(Subscriber=subscribe if not cross_master else None)
        master = 'http://upper:11311' if cross_master else None
        with patch.dict(sys.modules, {'sensor_msgs.msg': types.SimpleNamespace(Joy=object),
                                     'geometry_msgs.msg': types.SimpleNamespace(Twist=object),
                                     'std_msgs.msg': types.SimpleNamespace(Bool=object)}), \
                patch.object(native, 'UpperTopicObservation', side_effect=upper_subscribe), \
                patch.object(native.time, 'monotonic', side_effect=lambda: clock[0]), \
                patch.object(native.time, 'sleep', side_effect=sleep):
            if not repeats or output:
                with self.assertRaisesRegex(RuntimeError, 'neutral_or_zero_output_unconfirmed'):
                    native.remote_idle_guard(rospy, '/sdk', master)
                self.assertEqual(len(closed), 2)
                return
            check, close = native.remote_idle_guard(rospy, '/sdk', master)
            check()
            if cross_master:
                upper_subscriptions[-1].failed.set()
                with self.assertRaises(RuntimeError):
                    check()
                upper_subscriptions[-1].failed.clear()
            if interruption:
                if interruption == 'motion':
                    output = .01
                    emit()
                    with self.assertRaises(RuntimeError):
                        check()
                    output = 0.
                # A previous zero period or the last nonzero timestamp must
                # not qualify three new callbacks as 0.3s of quiet output.
                clock[0] += .4
                for _ in range(3):
                    clock[0] += .01
                    emit()
                with self.assertRaisesRegex(RuntimeError, 'neutral_or_zero_output_unconfirmed'):
                    check()
                for _ in range(7):
                    clock[0] += .05
                    emit()
                check()
            callbacks['/joy'](self.message(owner='/wrong'))
            with self.assertRaises(RuntimeError):
                check()
            emit()
            clock[0] += .31
            with self.assertRaises(RuntimeError):
                check()
            close()
            self.assertEqual(len(closed), 2)

    def test_disabled_upper_accepts_one_zero_but_live_enable_nonzero_or_disconnect_revokes_it(self):
        clock, callbacks, subscriptions = [10.], {}, []
        def subscribe(master, topic, kind, owner, callback):
            self.assertEqual(owner, '/nodelet_manager')
            callbacks[topic] = callback
            subscription = types.SimpleNamespace(failed=threading.Event(), unregister=lambda:None)
            subscriptions.append(subscription)
            return subscription
        def zero(value=0.):
            callbacks['/move_base/base_cmd_vel'](types.SimpleNamespace(
                linear=types.SimpleNamespace(x=value,y=0.,z=0.),
                angular=types.SimpleNamespace(x=0.,y=0.,z=0.),
                _connection_header={'callerid':'/nodelet_manager','latching':'1'}))
        def enable(value, owner='/nodelet_manager'):
            callbacks['/enable_vel_control_state'](types.SimpleNamespace(data=value,
                _connection_header={'callerid':owner,'latching':'1'}))
        def sleep(_):
            if clock[0] == 10.:
                zero(); enable(False)
            clock[0] += .05
        with patch.dict(sys.modules, {'sensor_msgs.msg':types.SimpleNamespace(Joy=object),
                'geometry_msgs.msg':types.SimpleNamespace(Twist=object),
                'std_msgs.msg':types.SimpleNamespace(Bool=object)}), \
                patch.object(native,'UpperTopicObservation',side_effect=subscribe), \
                patch.object(native.time,'monotonic',side_effect=lambda:clock[0]), \
                patch.object(native.time,'sleep',side_effect=sleep):
            check, close = native.remote_idle_guard(object(), None, 'http://upper:11311',
                require_joy=False, allow_disabled_output=True)
            clock[0] += 3.
            check()  # Disabled output is latched, not a heartbeat.
            for cause in ('enabled','wrong_owner','nonzero','disconnect'):
                with self.subTest(cause=cause):
                    if cause == 'enabled':enable(True)
                    elif cause == 'wrong_owner':enable(False,'/wrong')
                    elif cause == 'nonzero':zero(.1)
                    else:subscriptions[0].failed.set()
                    with self.assertRaisesRegex(RuntimeError,'zero_output_unconfirmed'):check()
                    subscriptions[0].failed.clear()
                    enable(False);zero();clock[0]+=.31
                    check()
            close()

    def test_missing_disable_feedback_cannot_accept_a_single_zero(self):
        clock, callbacks = [10.], {}
        def subscribe(topic, kind, callback, **kwargs):
            callbacks[topic]=callback
            return types.SimpleNamespace(unregister=lambda:None)
        def sleep(_):
            if clock[0] == 10.:
                callbacks['/move_base/base_cmd_vel'](types.SimpleNamespace(
                    linear=types.SimpleNamespace(x=0.,y=0.,z=0.),
                    angular=types.SimpleNamespace(x=0.,y=0.,z=0.),
                    _connection_header={'callerid':'/nodelet_manager'}))
            clock[0]+=.05
        with patch.dict(sys.modules, {'sensor_msgs.msg':types.SimpleNamespace(Joy=object),
                'geometry_msgs.msg':types.SimpleNamespace(Twist=object),
                'std_msgs.msg':types.SimpleNamespace(Bool=object)}), \
                patch.object(native.time,'monotonic',side_effect=lambda:clock[0]), \
                patch.object(native.time,'sleep',side_effect=sleep):
            with self.assertRaisesRegex(RuntimeError,'zero_output_unconfirmed'):
                native.remote_idle_guard(types.SimpleNamespace(Subscriber=subscribe),None,
                    require_joy=False,allow_disabled_output=True)

    def test_final_guard_requires_continuous_zero_output_and_neutral_input(self):
        self.run_guard()

    def test_a_single_latched_zero_is_insufficient(self):
        self.run_guard(repeats=False)

    def test_nonzero_upper_output_is_never_released(self):
        self.run_guard(output=.01)

    def test_nonzero_then_rapid_zeros_require_a_new_continuous_quiet_period(self):
        self.run_guard(interruption='motion')

    def test_stale_zero_stream_cannot_reuse_its_previous_quiet_period(self):
        self.run_guard(interruption='gap')

    def test_cross_master_guard_retains_quiet_window_and_fails_on_stream_loss(self):
        self.run_guard(interruption='motion', cross_master=True)

    def test_g12_owner_is_verified_against_local_sdk_process_not_node_name(self):
        master = types.SimpleNamespace(lookupNode=lambda *a: (1, '', 'http://localhost:1'))
        proxy = types.SimpleNamespace(getPid=lambda *a: (1, '', 123))
        expected = '/sdk/src/humanoid-control/h12pro_controller_node/src/h12pro_node/h12ChannelToJoymsg.py'
        with patch.object(controller.xmlrpc.client, 'ServerProxy') as factory, \
                patch.object(controller.Path, 'read_bytes', return_value=('python3\0'+expected+'\0').encode()):
            factory.return_value.__enter__.return_value = proxy
            self.assertEqual(controller.g12_owner([[('/joy', ['/arbitrary_node_name'])], [], []], master, '/sdk'),
                             '/arbitrary_node_name')
            self.assertIsNone(controller.g12_owner([[('/joy', ['/a', '/b'])], [], []], master, '/sdk'))
        with patch.object(controller.xmlrpc.client, 'ServerProxy') as factory, \
                patch.object(controller.Path, 'read_bytes', return_value=b'python3\0/tmp/fake_h12.py\0'):
            factory.return_value.__enter__.return_value = proxy
            self.assertIsNone(controller.g12_owner([[('/joy', ['/sdk'])], [], []], master, '/sdk'))

    def test_catkin_g12_relay_requires_exact_template_and_allowlisted_source(self):
        master = types.SimpleNamespace(lookupNode=lambda *a: (1, '', 'http://localhost:1'))
        proxy = types.SimpleNamespace(getPid=lambda *a: (1, '', 123))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root/'src/humanoid-control/h12pro_controller_node/src/h12pro_node/ocs2_h12pro_node.py'
            relay = root/'devel/.private/h12pro_controller_node/lib/h12pro_controller_node/ocs2_h12pro_node.py'
            relay.parent.mkdir(parents=True)
            template = """python_script = TARGET
with open(python_script, 'r') as fh:
    context = {'__builtins__': __builtins__, '__doc__': None, '__file__': python_script,
               '__name__': __name__, '__package__': None}
    exec(compile(fh.read(), python_script, 'exec'), context)
"""
            for target, suffix, accepted in ((source, '', True),
                    (Path('/tmp/untrusted.py'), '', False), (source, '\nprint(123)', False)):
                relay.write_text(template.replace('TARGET', repr(str(target))) + suffix)
                with patch.object(controller.xmlrpc.client, 'ServerProxy') as factory, \
                        patch.object(controller.Path, 'read_bytes', return_value=('python3\0'+str(relay)+'\0').encode()):
                    factory.return_value.__enter__.return_value = proxy
                    actual = controller.g12_owner([[('/joy', ['/joy_node'])], [], []], master, directory)
                    self.assertEqual(actual, '/joy_node' if accepted else None)

    def test_controller_exit_is_persisted_as_terminal_failure_before_ros_publish(self):
        sequence = controller.BootSequence(63, lambda action: None)
        with tempfile.TemporaryDirectory() as directory, \
                patch.object(controller, 'STATUS_PATH', Path(directory)/'controller/status.json'):
            status = controller.record_controller_exit(sequence, 17)
            saved = json.loads(controller.STATUS_PATH.read_text())
        self.assertEqual(status['state'], 'failed')
        self.assertEqual(status['error'], 'controller_process_exited')
        self.assertEqual(status['controller_exit_code'], 17)
        self.assertEqual(saved, status)


if __name__ == '__main__':
    fixtures.unittest.main()
