"""Factory tracker/mux feedback gates; no hardware or ROS installation needed."""
import importlib.util
import itertools
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch


def helper():
    spec = importlib.util.spec_from_file_location(
        'navigation_handoff_test', Path(__file__).resolve().parents[1] / 'host/navigation_rpc.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TrackerFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.module = helper()
        self.motion, self.speed, self.empty = 8, 0., False
        self.stamp = 101.
        self.connection = {'callerid': '/lqr_path_track', 'latching': '0'}
        self.owner, self.md5 = ['/lqr_path_track'], 'tracker-abi'
        self.response = types.SimpleNamespace(request_id=5, num_edge_remain=1, success=True)
        self.command = Mock(return_value=self.response)
        service = types.SimpleNamespace(_type='jaten_msgs/PathTrackRequestService', _md5sum='tracker-abi')
        def message():
            return types.SimpleNamespace(header=types.SimpleNamespace(), op=types.SimpleNamespace())
        def sample(topic, cls, timeout):
            if topic.endswith('/motion_state'):
                return types.SimpleNamespace(header=types.SimpleNamespace(
                    stamp=types.SimpleNamespace(to_sec=lambda: self.stamp)), motion_type=self.motion,
                    _connection_header=self.connection)
            vector = types.SimpleNamespace(x=self.speed, y=0., z=0.)
            return types.SimpleNamespace(linear=vector, angular=vector, _connection_header=self.connection)
        self.ros = types.SimpleNamespace(Time=types.SimpleNamespace(now=lambda: None),
            ServiceProxy=lambda *a: self.command, wait_for_message=sample)
        modules = {
            'rosgraph': types.SimpleNamespace(Master=lambda *a: types.SimpleNamespace(
                getSystemState=lambda: [[], [], [('/path_track_request_service', self.owner)]],
                lookupService=lambda n: n)),
            'rosservice': types.SimpleNamespace(get_service_headers=lambda *a: {
                'type': service._type, 'md5sum': self.md5}),
            'jaten_msgs.srv': types.SimpleNamespace(PathTrackRequestService=service),
            'jaten_msgs.msg': types.SimpleNamespace(PathTrackRequest=message, TrackEdge=types.SimpleNamespace,
                                                  MotionState=object),
            'geometry_msgs.msg': types.SimpleNamespace(Twist=object)}
        modules['nav_msgs.msg'] = types.SimpleNamespace(Odometry=object)
        p = patch.dict(sys.modules, modules)
        p.start(); self.addCleanup(p.stop)
        for replacement in (
                patch.object(self.module.os, 'urandom', return_value=b'\x00\x05'),
                patch.object(self.module.time, 'monotonic', side_effect=itertools.count(0., .1))):
            replacement.start(); self.addCleanup(replacement.stop)

    def test_pause_requires_quiet_tracker_and_uses_correlated_request(self):
        self.module.tracker_command(self.ros, 3)
        message = self.command.call_args.kwargs['request']
        self.assertEqual((message.request_id, message.op.operation), (5, 3))
        self.assertEqual(message.edge_num, 0)
        self.assertEqual(len(message.edge_to_add), 1)
        self.assertFalse(self.command.call_args.kwargs['next_area'])

    def test_tracker_helper_never_accepts_add_or_delete_operations(self):
        for operation in (1, 2):
            with self.assertRaises(ValueError):
                self.module.tracker_command(self.ros, operation)
        self.command.assert_not_called()

    def test_empty_tracker_can_be_held_without_pause_acceptance(self):
        self.response.num_edge_remain = 0
        self.response.success = False
        self.motion = 2
        with patch.object(self.module, 'confirm_software_stop') as held, \
                patch.object(self.module, 'confirm_stationary') as stationary:
            self.module.tracker_command(self.ros, 3)
        held.assert_called_once_with(self.ros, True)
        stationary.assert_called_once()

    def test_empty_finished_tracker_does_not_wait_for_unpublished_idle_velocity(self):
        self.response.num_edge_remain = 0
        self.motion = 2
        original = self.ros.wait_for_message
        def receive(topic, *args, **kwargs):
            if not topic.endswith('/motion_state'):
                raise AssertionError('finished tracker has no idle velocity stream')
            return original(topic, *args, **kwargs)
        self.ros.wait_for_message = receive
        with patch.object(self.module, 'confirm_software_stop'), \
                patch.object(self.module, 'confirm_stationary'):
            self.module.tracker_command(self.ros, 3)

    def test_empty_finished_tracker_still_requires_software_stop_and_stationarity(self):
        self.response.num_edge_remain = 0
        self.motion = 2
        for helper in ('confirm_software_stop', 'confirm_stationary'):
            with self.subTest(helper=helper), patch.object(self.module, 'confirm_software_stop'), \
                    patch.object(self.module, 'confirm_stationary'):
                with patch.object(self.module, helper, side_effect=RuntimeError('unconfirmed')):
                    with self.assertRaisesRegex(RuntimeError, 'unconfirmed'):
                        self.module.tracker_command(self.ros, 3)

    def test_service_success_does_not_replace_motion_feedback(self):
        self.motion = 1
        with self.assertRaisesRegex(RuntimeError, 'pause_unconfirmed'):
            self.module.tracker_command(self.ros, 3)

    def test_paused_flag_does_not_replace_zero_velocity_feedback(self):
        self.speed = .0168
        with self.assertRaisesRegex(RuntimeError, 'pause_unconfirmed'):
            self.module.tracker_command(self.ros, 3)

    def test_vendor_event_stamp_does_not_reject_new_nonlatched_feedback(self):
        self.stamp = 99.
        self.module.tracker_command(self.ros, 3)

    def test_retained_or_wrong_publisher_feedback_is_rejected(self):
        for connection in ({'callerid': '/lqr_path_track', 'latching': '1'},
                           {'callerid': '/other', 'latching': '0'}, {}):
            self.connection = connection
            with self.assertRaisesRegex(RuntimeError, 'pause_unconfirmed'):
                self.module.tracker_command(self.ros, 3)

    def test_wrong_request_id_cannot_acknowledge_pause(self):
        self.response.request_id = 6
        with self.assertRaisesRegex(RuntimeError, 'request_unconfirmed'):
            self.module.tracker_command(self.ros, 3)

    def test_resume_requires_fresh_nonpaused_motion(self):
        with self.assertRaisesRegex(RuntimeError, 'resume_unconfirmed'):
            self.module.tracker_command(self.ros, 4)

    def test_resume_accepts_tracking_or_factory_obstacle_stop(self):
        for state in (1, 3):
            with self.subTest(state=state):
                self.motion = state
                self.module.tracker_command(self.ros, 4)

    def test_wrong_service_owner_or_abi_is_never_called(self):
        for owner, md5 in ((['/another_node'], 'tracker-abi'), (['/lqr_path_track'], 'wrong')):
            self.owner, self.md5 = owner, md5
            with self.assertRaisesRegex(RuntimeError, 'identity_mismatch'):
                self.module.tracker_command(self.ros, 3)
        self.command.assert_not_called()


class EmptyHeldTrackerTests(unittest.TestCase):
    def setUp(self):
        self.module = helper()
        connection = {'callerid': '/lqr_path_track', 'latching': '0'}
        self.path = types.SimpleNamespace(num_edge_remain=0, edges_remain=[],
                                          _connection_header=connection.copy())
        self.motion = types.SimpleNamespace(motion_type=2, _connection_header=connection.copy())
        self.ros = types.SimpleNamespace(wait_for_message=Mock(
            side_effect=lambda topic, *a, **kw: self.path if topic.endswith('path_track_state') else self.motion))
        p = patch.dict(sys.modules, {'jaten_msgs.msg': types.SimpleNamespace(
            PathTrackState=object, MotionState=object)})
        p.start(); self.addCleanup(p.stop)
        self.held = patch.object(self.module, 'confirm_software_stop').start()
        self.stationary = patch.object(self.module, 'confirm_stationary').start()
        self.addCleanup(patch.stopall)

    def check(self):
        self.module.confirm_empty_held_tracker(self.ros, object)

    def test_empty_finished_feedback_requires_hold_and_fresh_rest(self):
        self.check()
        self.assertEqual(self.held.call_count, 2)
        self.stationary.assert_called_once_with(self.ros, object)
        self.assertEqual(self.ros.wait_for_message.call_count, 4)

    def test_pending_edges_or_unfinished_motion_are_rejected(self):
        for field, value in (( 'num_edge_remain', 1), ('edges_remain', [object()])):
            with self.subTest(field=field):
                setattr(self.path, field, value)
                with self.assertRaisesRegex(RuntimeError, 'not_empty'):
                    self.check()
                setattr(self.path, field, 0 if field == 'num_edge_remain' else [])
        self.motion.motion_type = 1
        with self.assertRaisesRegex(RuntimeError, 'not_empty'):
            self.check()

    def test_retained_or_untrusted_samples_are_rejected(self):
        for message in (self.path, self.motion):
            for connection in ({'callerid': '/other', 'latching': '0'},
                               {'callerid': '/lqr_path_track', 'latching': '1'}, {}):
                original = message._connection_header
                message._connection_header = connection
                with self.assertRaisesRegex(RuntimeError, 'not_empty'):
                    self.check()
                message._connection_header = original

    def test_new_task_during_stationarity_check_is_rejected(self):
        self.stationary.side_effect = lambda *a: setattr(self.path, 'num_edge_remain', 1)
        with self.assertRaisesRegex(RuntimeError, 'not_empty'):
            self.check()

    def test_missing_feedback_hold_or_motion_failure_stays_blocked(self):
        for target in (self.held, self.stationary, self.ros.wait_for_message):
            original = target.side_effect
            target.side_effect = RuntimeError('unconfirmed')
            with self.assertRaisesRegex(RuntimeError, 'unconfirmed'):
                self.check()
            target.side_effect = original


class MuxAcknowledgementTests(unittest.TestCase):
    def run_switch(self, acknowledgement):
        module = helper()
        callbacks = []
        subscriber = Mock()
        publisher = Mock()
        publisher.get_num_connections.return_value = 1
        def subscribe(topic, cls, callback, **kwargs):
            callbacks.append(callback)
            # Even an already matching retained value must not satisfy the gate.
            callback(types.SimpleNamespace(data='nav_mpc_path_track'))
            return subscriber
        def publish(message):
            if acknowledgement is not None:
                callbacks[0](types.SimpleNamespace(data=acknowledgement))
        publisher.publish.side_effect = publish
        ros = types.SimpleNamespace(Subscriber=subscribe, Publisher=Mock(return_value=publisher))
        with patch.dict(sys.modules, {'std_msgs.msg': types.SimpleNamespace(String=types.SimpleNamespace)}), \
             patch.object(module.time, 'monotonic', side_effect=itertools.count(0., 4.)):
            if acknowledgement == 'nav_mpc_path_track':
                module.select_navigation_source(ros)
            else:
                with self.assertRaisesRegex(RuntimeError, 'handoff_unconfirmed'):
                    module.select_navigation_source(ros)
        self.assertFalse(ros.Publisher.call_args.kwargs['latch'])
        publisher.publish.assert_called_once()
        subscriber.unregister.assert_called_once()
        publisher.unregister.assert_called_once()

    def test_new_matching_acknowledgement_succeeds(self):
        self.run_switch('nav_mpc_path_track')

    def test_retained_value_alone_does_not_confirm_switch(self):
        self.run_switch(None)

    def test_competing_source_does_not_confirm_switch(self):
        self.run_switch('nav_mpc_move_base')


class ReplacementPathTests(unittest.TestCase):
    def run_path(self, x=1., frame='map', latched='0', expect_success=False,
                 initial_timeouts=0, always_timeout=False):
        module = helper()
        path = types.SimpleNamespace(
            _connection_header={'callerid': '/lqr_path_track', 'latching': latched},
            header=types.SimpleNamespace(frame_id=frame),
            poses=[types.SimpleNamespace(pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=x, y=2.)))])
        class ROSException(Exception):
            pass
        samples = itertools.chain([ROSException('no path yet')] * initial_timeouts,
                                  itertools.repeat(ROSException('no path') if always_timeout else path))
        ros = types.SimpleNamespace(wait_for_message=Mock(side_effect=samples), ROSException=ROSException)
        with patch.dict(sys.modules, {'nav_msgs.msg': types.SimpleNamespace(Path=object)}), \
             patch.object(module.time, 'monotonic', side_effect=itertools.count(0., .1)):
            if expect_success:
                module.confirm_tracker_target(ros, {'x': 1., 'y': 2.})
                self.assertGreaterEqual(ros.wait_for_message.call_count, 3)
            else:
                with self.assertRaisesRegex(RuntimeError, 'target_unconfirmed'):
                    module.confirm_tracker_target(ros, {'x': 1., 'y': 2.})

    def test_matching_live_path_is_confirmed_across_several_samples(self):
        self.run_path(expect_success=True)

    def test_first_sample_timeout_does_not_shorten_planning_window(self):
        self.run_path(initial_timeouts=2, expect_success=True)

    def test_missing_path_exhausts_bounded_window_without_releasing(self):
        self.run_path(always_timeout=True)

    def test_old_path_to_another_goal_is_not_released(self):
        self.run_path(x=3.)

    def test_local_frame_path_is_not_confused_with_map_coordinates(self):
        self.run_path(frame='odom')

    def test_retained_path_does_not_confirm_replacement(self):
        self.run_path(latched='1')


class StationaryGuardTests(unittest.TestCase):
    def setUp(self):
        self.module = helper()
        class ROSException(Exception):
            pass
        self.ros = types.SimpleNamespace(ROSException=ROSException, wait_for_message=Mock())

    @staticmethod
    def odometry(stamp):
        vector = types.SimpleNamespace(x=0., y=0., z=0.)
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: stamp)),
            twist=types.SimpleNamespace(twist=types.SimpleNamespace(linear=vector, angular=vector)))

    def test_guard_is_rechecked_after_blocking_odometry_read(self):
        self.ros.wait_for_message.return_value = self.odometry(100.1)
        guard = Mock(side_effect=[None, RuntimeError('joystick_changed')])
        with patch.object(self.module.time, 'time', return_value=100.), \
             self.assertRaisesRegex(RuntimeError, 'joystick_changed'):
            self.module.confirm_stationary(self.ros, object, guard=guard)
        self.assertEqual(guard.call_count, 2)
        self.assertEqual(self.ros.wait_for_message.call_args.kwargs['timeout'], .1)

    def test_missing_odometry_keeps_checking_guard(self):
        self.ros.wait_for_message.side_effect = self.ros.ROSException('missing')
        guard = Mock(side_effect=[None, RuntimeError('neutral_feedback_stale')])
        with self.assertRaisesRegex(RuntimeError, 'neutral_feedback_stale'):
            self.module.confirm_stationary(self.ros, object, guard=guard)
        self.assertEqual(guard.call_count, 2)
        self.assertEqual(self.ros.wait_for_message.call_args.kwargs['timeout'], .1)

    def test_unguarded_timeout_still_propagates(self):
        self.ros.wait_for_message.side_effect = self.ros.ROSException('missing')
        with self.assertRaises(self.ros.ROSException):
            self.module.confirm_stationary(self.ros, object)
        self.assertEqual(self.ros.wait_for_message.call_args.kwargs['timeout'], 1.)

    def test_guarded_success_requires_distinct_stationary_samples(self):
        self.ros.wait_for_message.side_effect = [self.odometry(100.1), self.odometry(100.5)]
        guard = Mock()
        with patch.object(self.module.time, 'time', side_effect=[100., 100.2, 100.5]):
            self.module.confirm_stationary(self.ros, object, guard=guard)
        self.assertEqual(guard.call_count, 4)
        self.assertEqual(self.ros.wait_for_message.call_count, 2)


class PointNavigationBindingTests(unittest.TestCase):
    def setUp(self):
        import xml.etree.ElementTree as ET
        self.module = helper()
        self.direct = '/path_sequence_executor/dispatch_direct_move_target'
        self.key = '/leju_node/service_move_to_target_driver'
        self.marker = '/motus_kuavo/direct_navigation_binding'
        self.params = {self.key: '/path_sequence_executor/dispatch_move_target', '/run_id': 'boot-one'}
        self.pid = 10
        self.events = []
        self.names = {a: '/move_base/' + n for a, n in (
            ('navigate', 'move_to_target'), ('check', 'check_arrived'), ('cancel', 'set_dispatch_mode'))}
        self.types = {a: types.SimpleNamespace(_type='test/' + a, _md5sum=a) for a in self.names}
        self.direct_type = types.SimpleNamespace(_type='jaten_msgs/leju_MoveToTarget', _md5sum='direct')
        self.headers = {n: {'type': self.types[a]._type, 'md5sum': a} for a, n in self.names.items()}
        self.headers[self.direct] = {'type': self.direct_type._type, 'md5sum': 'direct'}
        self.owners = {n: ['/leju_node'] for n in self.names.values()}
        self.owners[self.direct] = ['/path_sequence_executor']
        def set_param(k, v):
            self.events.append(('param', k)); self.params[k] = v
        self.master = types.SimpleNamespace(lookupNode=lambda n: 'http://wrapper:' + str(self.pid),
            lookupService=lambda n: n, getParam=lambda n: self.params[n],
            hasParam=lambda n: n in self.params, setParam=set_param,
            getSystemState=lambda: [[], [], list(self.owners.items())])
        self.respawn = True
        self.shutdown_code = 1
        def shutdown(*args):
            self.events.append('shutdown')
            if self.respawn: self.pid += 1
            return self.shutdown_code, '', 0
        proxy = Mock()
        proxy.__enter__ = Mock(return_value=proxy)
        proxy.__exit__ = Mock(return_value=False)
        proxy.getPid.side_effect = lambda *a: (1, '', self.pid)
        proxy.shutdown.side_effect = shutdown
        self.launch = ET.ElementTree(ET.fromstring('<launch><node name="leju_node" '
            'pkg="leju_mobile_base_msgs" type="leju_node" respawn="true"/></launch>'))
        patches = [patch.dict(sys.modules, {
            'rosgraph': types.SimpleNamespace(Master=lambda *a: self.master),
            'rosservice': types.SimpleNamespace(get_service_headers=lambda n, uri: self.headers[n]),
            'jaten_msgs.srv': types.SimpleNamespace(leju_MoveToTarget=self.direct_type),
            'nav_msgs.msg': types.SimpleNamespace(Odometry=object)}),
            patch('xmlrpc.client.ServerProxy', return_value=proxy),
            patch('xml.etree.ElementTree.parse', return_value=self.launch),
            patch.object(self.module, 'confirm_software_stop', side_effect=lambda *a, **k: self.events.append('held')),
            patch.object(self.module, 'confirm_stationary', side_effect=lambda *a: self.events.append('stationary')),
            patch.object(self.module.time, 'monotonic', side_effect=itertools.count(0, 1)),
            patch.object(self.module.time, 'sleep')]
        for p in patches: p.start(); self.addCleanup(p.stop)

    def call(self):
        self.module.ensure_point_navigation(object(), self.names, self.types)

    def test_binding_restarts_only_wrapper_while_held_and_keeps_wrapper_task_contract(self):
        self.call()
        self.assertEqual(self.events[:4], ['held', 'stationary', ('param', self.key), 'shutdown'])
        self.assertEqual(self.params[self.key], self.direct)
        self.assertEqual(self.params[self.marker]['pid'], 11)
        self.events.clear()
        self.call()
        self.assertEqual(self.events, [])  # No restart for the verified live process.

    def test_respawn_or_boot_change_invalidates_binding_marker(self):
        self.call()
        self.pid += 1
        self.events.clear()
        self.call()
        self.assertIn('shutdown', self.events)
        self.params['/run_id'] = 'boot-two'
        self.events.clear()
        self.call()
        self.assertIn('shutdown', self.events)

    def test_parameter_write_without_process_confirmation_is_insufficient(self):
        self.params[self.key] = self.direct
        self.call()
        self.assertIn('shutdown', self.events)

    def test_wrong_direct_abi_or_service_owner_never_restarts(self):
        for headers, owner in (({'type': 'wrong', 'md5sum': 'direct'}, ['/path_sequence_executor']),
                               (self.headers[self.direct], ['/unknown'])):
            self.headers[self.direct] = headers
            self.owners[self.direct] = owner
            with self.assertRaisesRegex(RuntimeError, 'direct_contract'):
                self.call()
            self.assertNotIn('shutdown', self.events)

    def test_unknown_binding_and_nonrespawning_wrapper_are_preserved(self):
        self.params[self.key] = '/custom_service'
        with self.assertRaisesRegex(RuntimeError, 'unrecognized_point_binding'): self.call()
        self.params[self.key] = '/path_sequence_executor/dispatch_move_target'
        self.launch.getroot()[0].set('respawn', 'false')
        with self.assertRaisesRegex(RuntimeError, 'respawn_required'): self.call()
        self.assertNotIn('shutdown', self.events)

    def test_respawn_failure_never_records_binding_success(self):
        self.respawn = False
        with self.assertRaisesRegex(RuntimeError, 'restart_unconfirmed'): self.call()
        self.assertNotIn(self.marker, self.params)

    def test_rejected_shutdown_restores_parameter(self):
        self.shutdown_code = -1
        with self.assertRaisesRegex(RuntimeError, 'restart_rejected'): self.call()
        self.assertEqual(self.params[self.key], '/path_sequence_executor/dispatch_move_target')
        self.assertNotIn(self.marker, self.params)

    def test_hold_failure_prevents_reconfiguration(self):
        with patch.object(self.module, 'confirm_software_stop', side_effect=RuntimeError('not_held')):
            with self.assertRaisesRegex(RuntimeError, 'not_held'): self.call()
        self.assertNotIn('shutdown', self.events)
        self.assertNotIn(self.marker, self.params)


if __name__ == '__main__':
    unittest.main()
