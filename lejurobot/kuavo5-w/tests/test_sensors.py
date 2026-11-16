"""Exercise vendor-to-Agent mappings without ROS or hardware dependencies."""
import json
import math
import threading
import time
import types
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import patch

from test_driver import driver, FakeNode, FakeRos1, LINK_ORDER, JOINT_NAMES, ROOT, sensors_module


class SensorTests(unittest.TestCase):
    def setUp(self):
        self.ros, self.node = FakeRos1(), FakeNode()
        self.hub = driver.KuavoSensors({}, 'test', self.ros, self.node)
        self.hub.discover()

    def tearDown(self):
        self.hub.close()

    def odom(self, vx=1., vy=2., wz=3., child='base_link', secs=0):
        self.ros.subscriptions['/odom'][1]({
            'header': {'stamp': {'secs': secs, 'nsecs': 0}, 'frame_id': 'odom'},
            'child_frame_id': child,
            'pose': {'pose': {'position': dict(x=0., y=0., z=0.)}},
            'twist': {'twist': {'linear': dict(x=vx, y=vy, z=0.),
                                'angular': dict(x=0., y=0., z=wz)}}})

    def test_discovers_wheel_profile_and_local_model(self):
        self.hub.require_profile()
        self.assertEqual(self.hub.health()['profile']['robot_type'], 1)
        self.assertEqual(self.hub._names, JOINT_NAMES)
        self.assertEqual(self.hub.dispatch('model', 'read'), {'urdf': self.hub._model_path.read_text()})
        self.assertTrue(self.hub.health()['skeleton_model']['runtime_joint_names_verified'])
        self.assertEqual(self.ros.services, [])
        self.assertEqual(self.ros.published, [])

    def test_raw_field_mapping_units_and_urdf_joint_names(self):
        self.ros.raw(list(range(20)))
        data = self.hub.snapshot('joints')
        self.assertTrue(data['valid'])
        joint = data['joints'][7]
        self.assertEqual(joint, dict(idx=7, name=JOINT_NAMES[7], q=7, dq=1., ddq=2., tau=None))
        self.assertEqual(data['units']['q'], 'rad')
        self.assertIsNone(data['units']['tau'])
        self.assertEqual(data['vendor']['joint_torque'], [3.] * 20)

    def test_torque_exposed_only_when_unit_is_verified(self):
        hub = driver.KuavoSensors({'joint_torque_unit': 'N*m'}, 'test', self.ros, FakeNode())
        try:
            hub.discover()
            self.ros.raw()
            data = hub.snapshot('joints')
            self.assertEqual(data['joints'][0]['tau'], 3.)
            self.assertEqual(data['joints'][0]['ddq'], 2.)
            self.assertEqual(data['units']['tau'], 'N*m')
        finally:
            hub.close()

    def test_imu_quaternion_order_and_units(self):
        self.ros.raw()
        sample = self.hub.snapshot('imu')
        self.assertTrue(sample['valid'])
        self.assertEqual(sample['quaternion'], [1., 0., 0., 0.])
        self.assertEqual(sample['gyroscope'], [1., 2., 3.])
        self.assertEqual(sample['accelerometer'], [0., 0., 9.81])
        self.assertEqual(sample['units']['gyroscope'], 'rad/s')
        self.assertIsNone(sample['frame_id'])

    def test_standard_imu_operates_without_controller_profile(self):
        self.hub._profile = None
        self.hub._on_standard_imu({'header': {'frame_id': 'imu_link'},
            'orientation': dict(w=1., x=0., y=0., z=0.),
            'orientation_covariance': [0.] * 9,
            'angular_velocity': dict(x=1., y=2., z=3.),
            'linear_acceleration': dict(x=0., y=0., z=9.81)})
        sample = self.hub.snapshot('imu')
        self.assertTrue(sample['valid'])
        self.assertEqual(sample['vendor']['topic'], '/imu')
        self.assertEqual(sample['quaternion'], [1., 0., 0., 0.])
        self.assertEqual(sample['rpy'], [0., 0., 0.])
        self.assertIsNone(sample['free_acceleration'])
        self.ros.params.clear()
        self.hub.discover()
        self.assertTrue(self.hub.snapshot('imu')['valid'])
        self.assertTrue(self.hub.dispatch('joints', 'info')['skeleton_model']['reference_only'])

    def test_missing_controller_parameters_report_not_ready_not_wrong_hardware(self):
        self.ros.params.clear()
        self.hub.discover()
        self.assertIn('controller_not_ready', self.hub.health()['discovery_error'])
        self.assertTrue(self.hub.health()['skeleton_model']['reference_only'])

    def test_missing_optional_torque_not_replaced_by_acceleration(self):
        self.ros.raw(joint_data={'joint_q': [0.] * 20, 'joint_vd': [9.] * 20})
        j = self.hub.snapshot('joints')['joints'][0]
        self.assertIsNone(j['tau'])
        self.assertIsNone(j['dq'])
        self.assertEqual(j['ddq'], 9.)

    def test_invalid_q_does_not_leave_last_measurement_valid(self):
        for value in ([0.] * 19, [math.nan] * 20, [True] * 20):
            self.ros.raw()
            self.ros.raw(value)
            self.assertFalse(self.hub.snapshot('joints')['valid'])
            with self.assertRaises(RuntimeError):
                self.hub.joint_positions()

    def test_bad_imu_quaternion_rejected(self):
        self.ros.raw(imu_data={'quat': dict(w=0, x=0, y=0, z=0)})
        self.assertFalse(self.hub.snapshot('imu')['valid'])

    def test_joint_capture_timestamp_must_be_valid_and_recent(self):
        now = int(time.time())
        for capture in (None, {}, {'secs': 0, 'nsecs': 0}, {'secs': True, 'nsecs': 0},
                        {'secs': now, 'nsecs': 1_000_000_000}, {'secs': now - 2, 'nsecs': 0},
                        {'secs': now + 2, 'nsecs': 0}):
            with self.subTest(capture=capture):
                self.ros.raw(sensor_time=capture)
                self.assertFalse(self.hub.snapshot('joints')['valid'])
                with self.assertRaises(RuntimeError):
                    self.hub.joint_positions()
                self.assertTrue(self.hub.snapshot('imu')['valid'])

    def test_reordered_capture_does_not_replace_newer_joint_sample(self):
        ns = time.time_ns()
        captured = lambda value: {'secs': value // 1_000_000_000, 'nsecs': value % 1_000_000_000}
        self.ros.raw([1.] * 20, sensor_time=captured(ns))
        seen, measured = self.hub._seen['joints'], self.hub.joint_positions()
        self.ros.raw([9.] * 20, sensor_time=captured(ns - 1_000_000))
        self.assertEqual(self.hub._seen['joints'], seen)
        self.assertEqual(self.hub.joint_positions(), measured)

    def test_periodic_identity_refresh_keeps_raw_subscription_and_fresh_sample(self):
        self.ros.raw()
        callback = self.ros.subscriptions['/sensors_data_raw'][1]
        with patch.object(self.ros, 'subscribe', wraps=self.ros.subscribe) as subscribe:
            self.hub._retry_at = 0
            self.hub.tick()
            self.hub._thread.join(1)
        self.assertFalse(self.hub._thread.is_alive())
        self.assertEqual(subscribe.call_count, 0)
        self.assertIs(self.ros.subscriptions['/sensors_data_raw'][1], callback)
        self.assertTrue(self.hub.snapshot('joints')['valid'])

    def test_slow_parameter_round_does_not_grant_a_new_identity_lease_at_completion(self):
        clock = [time.monotonic()]
        original = self.ros.get_param
        def slow(*args, **kwargs):
            clock[0] += 1.
            return original(*args, **kwargs)
        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0])
        with patch.object(sensors_module, 'time', fake_time), patch.object(self.ros, 'get_param', side_effect=slow):
            self.hub._retry_at = 0
            self.hub.tick()
            self.hub._thread.join(1)
            self.assertFalse(self.hub._thread.is_alive())
            self.assertFalse(self.hub.health()['identity_verification']['valid'])
            with self.assertRaisesRegex(RuntimeError, 'identity verification is stale'):
                self.hub.require_profile()

    def test_stale_or_reconnected_measurements_are_not_zeros(self):
        self.ros.raw()
        self.hub._seen['joints'] = time.monotonic() - 1
        self.assertEqual(self.hub.snapshot('joints')['joints'], [])
        self.ros.connection = (True, 2)
        self.assertFalse(self.hub.snapshot('imu')['valid'])
        with self.assertRaises(RuntimeError):
            self.hub.require_profile()
        self.assertIn('urdf', self.hub.dispatch('model', 'read'))

    def test_sensor_lifecycle_and_disconnected_publication(self):
        self.hub.tick()
        self.assertEqual(self.hub._pubs['imu'].messages, [])
        self.hub.dispatch('imu', 'start')
        self.ros.raw()
        self.hub.tick()
        self.assertTrue(json.loads(self.hub._pubs['imu'].messages[-1].data)['valid'])
        self.ros.connection = (False, 1)
        self.hub.tick()
        disconnected = json.loads(self.hub._pubs['imu'].messages[-1].data)
        self.assertFalse(disconnected['connected'])
        self.assertNotIn('quaternion', disconnected)
        self.hub.dispatch('imu', 'stop')
        before = len(self.hub._pubs['imu'].messages)
        self.hub.tick()
        self.assertEqual(len(self.hub._pubs['imu'].messages), before)

    def test_shm_selection_unsubscribes_previous_source(self):
        self.ros.raw()
        old_callback = self.ros.subscriptions['/sensors_data_raw'][1]
        self.ros.params['use_shm_communication'] = True
        self.hub.discover()
        self.assertNotIn('/sensors_data_raw', self.ros.subscriptions)
        self.assertIn('/sensors_data_raw_shm', self.ros.subscriptions)
        self.assertFalse(self.hub.snapshot('joints')['valid'])
        old_callback({'joint_data': {'joint_q': [99.] * 20}})
        self.assertFalse(self.hub.snapshot('joints')['valid'])
        self.ros.raw()
        self.assertTrue(self.hub.snapshot('joints')['valid'])

    def test_old_discovery_cannot_leave_trusted_profile_after_mismatch(self):
        self.ros.raw()
        self.ros.params['robot_type'] = 2
        self.hub.discover()
        with self.assertRaises(RuntimeError):
            self.hub.require_profile()
        self.assertIn('urdf', self.hub.dispatch('model', 'read'))
        self.assertFalse(self.hub.health()['skeleton_model']['runtime_joint_names_verified'])
        self.assertFalse(self.hub.snapshot('imu')['valid'])

    def test_model_recovery_requires_new_correctly_named_measurement(self):
        urdf = self.ros.params.pop('humanoid_description')
        self.hub.discover()
        self.ros.raw()
        self.ros.params['humanoid_description'] = urdf
        self.hub.discover()
        self.assertFalse(self.hub.snapshot('joints')['valid'])
        self.ros.raw()
        self.assertEqual(self.hub.snapshot('joints')['joints'][0]['name'], JOINT_NAMES[0])

    def test_wrong_robot_or_dof_never_enables_motion(self):
        for key, value in [('robot_type', 2), ('armRealDof', 8), ('legRealDof', 12), ('headRealDof', True)]:
            ros = FakeRos1()
            ros.params[key] = value
            hub = driver.KuavoSensors({}, 'test', ros, FakeNode())
            try:
                hub.discover()
                with self.assertRaises(RuntimeError):
                    hub.require_profile()
            finally:
                hub.close()

    def test_missing_runtime_urdf_preserves_model_without_inventing_measurements(self):
        del self.ros.params['humanoid_description']
        self.hub.discover()
        self.ros.raw()
        self.assertFalse(self.hub.snapshot('joints')['valid'])
        self.assertEqual(self.hub.snapshot('joints')['joints'], [])
        self.assertIn('urdf', self.hub.dispatch('model', 'read'))
        self.assertIsNotNone(self.hub.health()['model_error'])
        self.assertTrue(self.hub.snapshot('imu')['valid'])

    def test_biped_or_unsafe_runtime_xml_never_validates_measured_joints(self):
        for urdf in ('<robot name="biped"/>', '<!DOCTYPE robot><robot/>',
                     self.ros.params['humanoid_description'].replace('zhead_2_link', 'other')):
            self.ros.params['humanoid_description'] = urdf
            self.hub.discover()
            self.ros.raw()
            self.assertIsNone(self.hub._names)
            self.assertFalse(self.hub.snapshot('joints')['valid'])
            self.assertIn('urdf', self.hub.dispatch('model', 'read'))

    def test_local_model_available_before_discovery_and_without_ros_parameters(self):
        ros = FakeRos1()
        ros.connection = (False, 1)
        ros.params.clear()
        hub = driver.KuavoSensors({}, 'test', ros, FakeNode())
        try:
            model = hub.dispatch('model', 'read')
            self.assertEqual(model, {'urdf': hub._model_path.read_text()})
            self.assertFalse(hub.snapshot('joints')['valid'])
            self.assertEqual(hub.snapshot('joints')['joints'], [])
            hub.discover()
            self.assertEqual(hub.dispatch('model', 'read'), model)
            self.assertEqual(ros.services, [])
            self.assertEqual(ros.published, [])
        finally:
            hub.close()

    def test_local_model_read_errors_are_explicit(self):
        with patch.object(type(self.hub._model_path), 'read_text', side_effect=FileNotFoundError('missing')):
            self.assertIn('local skeleton URDF unavailable', self.hub.dispatch('model', 'read')['error'])
        for xml in ('<robot>', '<robot/>', '<!DOCTYPE robot><robot/>'):
            with patch.object(type(self.hub._model_path), 'read_text', return_value=xml):
                self.assertIn('error', self.hub.dispatch('model', 'read'))

    def test_runtime_joint_names_must_match_displayed_model(self):
        self.ros.raw()
        self.ros.params['humanoid_description'] = self.ros.params['humanoid_description'].replace(
            f'name="{JOINT_NAMES[0]}"', 'name="different_knee"')
        self.hub.discover()
        self.ros.raw()
        self.assertIn('runtime joint names differ', self.hub.health()['model_error'])
        self.assertEqual(self.hub.snapshot('joints')['joints'], [])
        self.assertFalse(self.hub.snapshot('joints')['valid'])
        self.assertIn('urdf', self.hub.dispatch('model', 'read'))

    def test_packaged_skeleton_is_one_connected_tree_without_mesh_dependencies(self):
        root = ET.fromstring((ROOT / 'resource' / 'kuavo5w_model.urdf').read_text())
        links, joints = root.findall('link'), root.findall('joint')
        names = {link.get('name') for link in links}
        self.assertEqual(len(names), len(links))
        self.assertEqual(len(joints), len(links) - 1)
        self.assertEqual(len({joint.get('name') for joint in joints}), len(joints))
        children, adjacency = set(), {name: [] for name in names}
        for joint in joints:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            self.assertIn(parent, names)
            self.assertIn(child, names)
            self.assertNotIn(child, children)
            children.add(child)
            adjacency[parent].append(child)
        self.assertEqual(names - children, {'base_link'})
        visited, pending = set(), ['base_link']
        while pending:
            link = pending.pop()
            self.assertNotIn(link, visited)
            visited.add(link)
            pending.extend(adjacency[link])
        self.assertEqual(visited, names)
        self.assertTrue(set(LINK_ORDER) <= names)
        self.assertTrue(set(JOINT_NAMES) <= {joint.get('name') for joint in joints})
        for tag in ('mesh', 'visual', 'collision', 'inertial'):
            self.assertEqual(root.findall('.//' + tag), [])

    def test_odom_averages_window_null_axes_and_reports_provenance(self):
        self.odom(1., 2., 3.)
        self.odom(3., 4., 5.)
        sample = self.hub.snapshot('loco_state')
        self.assertTrue(sample['valid'])
        self.assertEqual(sample['schema'], 'motus.odom/1')
        self.assertEqual(sample['frame'], 'body')
        self.assertEqual(sample['twist'], [2., 3., None, None, None, 4.])
        self.assertIsNone(sample['pose'])
        self.assertEqual(sample['vendor']['samples'], 2)
        self.assertEqual(sample['vendor']['stamp_source'], 'received')
        self.assertEqual(self.hub.dispatch('loco_state', 'info')['odom_interface']['provides'], ['vx', 'vy', 'wz'])
        self.assertEqual(next(t for t in self.hub.tools() if t['name'] == 'loco_state')['topic_out'][0]['format'], 'state/odom')

    def test_odom_reading_does_not_require_whole_body_controller(self):
        self.hub._profile = None
        self.hub._discovery_error = 'whole-body controller unlaunch'
        self.odom(vx=0.12, vy=0.03, wz=0.01, child='base_footprint')
        sample = self.hub.snapshot('loco_state')
        self.assertTrue(sample['valid'])
        self.assertIsNone(sample['reason'])
        self.assertEqual(sample['twist'], [0.12, 0.03, None, None, None, 0.01])
        with self.assertRaises(RuntimeError):
            self.hub.require_profile()  # Reading data must never enable motion.
        for key in ('robot_type', 'armRealDof', 'headRealDof', 'legRealDof', 'waistRealDof'):
            self.ros.params.pop(key, None)
        self.hub.discover()
        self.assertTrue(self.hub.snapshot('loco_state')['valid'])
        with self.assertRaises(RuntimeError):
            self.hub.require_profile()

    def test_switching_raw_source_preserves_independent_odometry(self):
        self.odom()
        self.ros.params['use_shm_communication'] = True
        self.hub.discover()
        self.assertTrue(self.hub.snapshot('loco_state')['valid'])
        self.ros.connection = (True, 2)
        self.assertFalse(self.hub.snapshot('loco_state')['valid'])

    def test_odom_empty_window_does_not_republish_old_velocity(self):
        self.hub.dispatch('loco_state', 'start')
        self.odom()
        self.hub.tick()
        self.hub.tick()
        sample = json.loads(self.hub._pubs['loco_state'].messages[-1].data)
        self.assertFalse(sample['valid'])
        self.assertEqual(sample['twist'], [None] * 6)
        self.assertEqual(sample['vendor']['samples'], 0)

    def test_odom_unknown_frame_nan_or_missing_twist_is_unavailable(self):
        self.odom(child='map')
        self.assertFalse(self.hub.snapshot('loco_state')['valid'])
        self.odom(vx=math.nan)
        self.assertFalse(self.hub.snapshot('loco_state')['valid'])
        self.ros.subscriptions['/odom'][1]({})
        self.assertEqual(self.hub.snapshot('loco_state')['twist'], [None] * 6)

    def test_synchronized_vendor_timestamp_preserved(self):
        now = int(time.time())
        self.odom(secs=now)
        sample = self.hub.snapshot('loco_state')
        self.assertEqual(sample['stamp_ms'], now * 1000)
        self.assertEqual(sample['vendor']['stamp_source'], 'robot')

    def test_discovery_runs_off_receiver_and_discards_old_session_result(self):
        entered, release = threading.Event(), threading.Event()
        original = self.ros.get_param
        def blocking(name, default=None):
            entered.set()
            release.wait(1)
            return original(name, default)
        self.hub._profile = None
        with patch.object(self.ros, 'get_param', side_effect=blocking):
            self.hub._retry_at = 0
            self.hub.tick()
            self.assertTrue(entered.wait(1))
            self.ros.connection = (True, 2)
            self.hub.snapshot('imu')
            release.set()
            self.hub._thread.join(1)
        self.assertIsNone(self.hub._profile)
        self.assertFalse(self.hub._thread.is_alive())


if __name__ == '__main__':
    unittest.main()
