import importlib.util
import hashlib
import copy
import json
import math
import sys
import threading
import time
import types
import unittest
import tempfile
from contextlib import contextmanager
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parents[1]))
spec = importlib.util.spec_from_file_location("kuavo_device", ROOT / "device.py")
driver = importlib.util.module_from_spec(spec)
std = types.ModuleType("std_msgs.msg")
std.String = type("String", (), {})
std.UInt8MultiArray = type('UInt8MultiArray', (), {})
qos = types.ModuleType('rclpy.qos')
qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT='best_effort')
qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST='keep_last')
qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE='volatile')
sensor_msg = types.ModuleType('sensor_msgs.msg')
class FakeCompressedImage:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id='', stamp=types.SimpleNamespace(sec=0, nanosec=0))
sensor_msg.CompressedImage = FakeCompressedImage
with patch.dict(sys.modules, {"std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": std,
                              'rclpy': types.ModuleType('rclpy'), 'rclpy.qos': qos,
                              'sensor_msgs': types.ModuleType('sensor_msgs'), 'sensor_msgs.msg': sensor_msg}):
    spec.loader.exec_module(driver)
    from kuavo_sensors import LINK_ORDER, model_names
    import kuavo_sensors as sensors_module
    import peripherals as peripherals_module
    JOINT_NAMES = model_names((ROOT / 'resource' / 'kuavo5w_model.urdf').read_text())


class FakeRos1:
    def __init__(self):
        self.published, self.services, self.subscriptions, self.advertised = [], [], {}, []
        self.connection, self.motion_ready = (True, 1), True
        self.service_accepts, self.confirm = True, True
        self.latched, self.sent_services, self.publisher_checks = {}, [], []
        self.enable_publishers, self.replay_enable = ['/wheel_controller'], True
        self.service_results = {}
        self.params = {'robot_type': 1, 'robot_version': 400062, 'armRealDof': 14,
                       'headRealDof': 2, 'legRealDof': 4, 'waistRealDof': 0,
                       'use_shm_communication': False,
                       'humanoid_description': '<robot name="fixture">' + ''.join(
                           f'<joint name="{JOINT_NAMES[i]}" type="revolute"><child link="{link}"/></joint>'
                           for i, link in enumerate(LINK_ORDER)) + '</robot>'}

    def get_param(self, name, default=None):
        return self.params.get(name.lstrip('/'), default)

    def get_params(self, defaults, timeout=2.):
        return {name: self.get_param(name, default) for name, default in defaults.items()}

    def unsubscribe(self, topic):
        self.subscriptions.pop(topic, None)

    def advertise(self, topic, message_type):
        self.advertised.append((topic, message_type))

    def publish(self, topic, message_type, message):
        if not self.connection[0]:
            raise ConnectionError("disconnected")
        self.published.append((topic, message_type, message))

    def call_service(self, service, service_type, args, timeout):
        if service == '/rosapi/publishers':
            self.publisher_checks.append((service, service_type, args, timeout))
            return {'publishers': self.enable_publishers}
        self.services.append((service, service_type, args, timeout))
        if not self.connection[0]:
            raise ConnectionError("disconnected")
        if self.service_accepts and self.confirm:
            if service == "/enable_control":
                self.emit("/enable_control_state", args["data"])
                if args['data'] and '/mobile_manipulator/lb_mpc_control_mode' in self.subscriptions:
                    self.emit('/mobile_manipulator/lb_mpc_control_mode',
                              self.latched.get('/mobile_manipulator/lb_mpc_control_mode', 3))
            elif service == "/mobile_manipulator_mpc_control":
                self.emit("/mobile_manipulator/lb_mpc_control_mode", args["control_mode"])
            elif service == '/enable_vel_control':
                self.emit('/enable_vel_control_state', args['data'])
        return {"success": self.service_accepts, "result": self.service_accepts, "message": "test service"}

    def start_service(self, service, service_type, args):
        self.sent_services.append((service, service_type, args))
        return service, service_type, args

    def wait_service(self, pending, timeout, cancelled=None):
        if cancelled is not None and cancelled.is_set():
            raise InterruptedError('control request superseded by stop')
        response = self.call_service(*pending, timeout)
        self.service_results[id(pending)] = response
        if cancelled is not None and cancelled.is_set():
            raise InterruptedError('control request superseded by stop')
        return response

    def retain_service(self, pending):
        pass

    def service_result(self, pending):
        return self.service_results.get(id(pending))

    def subscribe(self, topic, message_type, callback, throttle_ms=0):
        self.subscriptions[topic] = (message_type, callback)
        if topic == '/enable_control_state' and self.replay_enable and topic in self.latched:
            callback({'data': self.latched[topic]})

    def emit(self, topic, value):
        self.latched[topic] = value
        self.subscriptions[topic][1]({"data": value})

    def raw(self, q=None, **overrides):
        capture_ns = time.time_ns()
        msg = {'sensor_time': {'secs': capture_ns // 1_000_000_000, 'nsecs': capture_ns % 1_000_000_000},
               'joint_data': {'joint_q': q if q is not None else [0.] * 20,
                              'joint_v': [1.] * 20, 'joint_vd': [2.] * 20, 'joint_torque': [3.] * 20},
               'imu_data': {'quat': dict(x=0., y=0., z=0., w=1.),
                            'gyro': dict(x=1., y=2., z=3.), 'acc': dict(x=0., y=0., z=9.81),
                            'free_acc': dict(x=0., y=0., z=0.)}}
        msg.update(overrides)
        topic = '/sensors_data_raw_shm' if self.params['use_shm_communication'] else '/sensors_data_raw'
        self.subscriptions[topic][1](msg)


class FakePublisher:
    def __init__(self, topic):
        self.topic_name, self.messages = topic, []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    def create_publisher(self, _type, topic, _depth):
        self.publisher = FakePublisher(topic)
        return self.publisher

    def create_timer(self, _period, callback):
        return callback


def wait_until(predicate, timeout=1):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return bool(predicate())


def fixture_joint_limits(ros):
    """Synthetic bounds for offline tests, NOT limits for a real robot."""
    return {'verification': 'OFFLINE TEST FIXTURE ONLY; NOT HARDWARE LIMITS',
            'robot_version': ros.params['robot_version'],
            'runtime_urdf_sha256': hashlib.sha256(ros.params['humanoid_description'].encode()).hexdigest(),
            **{card: {'joint_names': JOINT_NAMES[offset:offset + size],
                      'lower_deg': [-90.] * size, 'upper_deg': [90.] * size}
               for card, offset, size in (('arm', 4, 14), ('leg', 0, 4))}}


class DriverContractTests(unittest.TestCase):
    def setUp(self):
        self.callbacks = []
        self.callback_patch = patch.object(driver, "_acp_complete", side_effect=lambda *a: self.callbacks.append(a))
        self.callback_patch.start()
        self.ros, self.node = FakeRos1(), FakeNode()
        self.bundle = driver.KuavoBundle({'sensors': {'joint_limits': fixture_joint_limits(self.ros)}},
                                         "test_robot", self.ros, self.node)
        self.bundle.sensors.discover()
        self.ros.raw()
        self.ros.emit("/enable_control_state", True)
        self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 3)
        self.bundle.state.verify_enable()
        self.ros.emit('/enable_vel_control_state', True)

    def tearDown(self):
        self.bundle.close()
        self.callback_patch.stop()

    def call(self, card, action, **args):
        if card == '_control':  # Test internal control primitives; never an MCP card.
            try:
                if action == 'get_current_mode':
                    return {'state': 'ready', 'feedback': self.bundle.state.snapshot()}
                if action == 'set_mode':
                    return self.bundle.set_mode(args['mode'])
                if action == 'pause':
                    return self.bundle.pause('test_pause')
                return self.bundle.resume()
            except Exception as exc:
                return {'state': 'error', 'error': str(exc)}
        if card == 'state':  # Internal safety state remains tested, no public card.
            return self.bundle.state.dispatch(action, args)
        return self.bundle.dispatch(card, {"action": action, **args})

    @contextmanager
    def held_chassis_start(self):
        from controlled_spatial import SpatialStore
        from test_controlled_spatial import Client
        spatial = self.bundle.spatial
        previous_store, previous_client = spatial.store, spatial.client
        entered, release = threading.Event(), threading.Event()
        with tempfile.TemporaryDirectory() as folder:
            spatial.store, spatial.client = SpatialStore(folder + '/db'), Client()
            original = spatial.client.native
            def native(operation, **args):
                if operation == 'prepare':
                    entered.set()
                    release.wait(2)
                return original(operation, **args)
            with patch.object(spatial.client, 'native', side_effect=native):
                spatial.start_chassis()
                try:
                    self.assertTrue(entered.wait(1))
                    yield release
                finally:
                    release.set()
                    spatial._startup_thread.join(2)
                    spatial.store.close()
                    spatial.store, spatial.client = previous_store, previous_client

    def test_leg_motion_waits_for_shared_chassis_start_then_revalidates_target(self):
        targets, validated, results = [], [], []
        def motion():
            results.append(self.bundle.start_motion('lower_body', 'leg', (3,),
                lambda: targets.append('target'), lambda: validated.append('live'),
                request_epoch=self.bundle._stop_epoch, notify=False))
        with self.held_chassis_start() as release:
            thread = threading.Thread(target=motion)
            thread.start()
            time.sleep(.06)
            self.assertTrue(thread.is_alive())
            self.assertFalse(targets)
            self.assertFalse(validated)
            release.set()
            thread.join(1)
            self.assertFalse(thread.is_alive())
            self.assertEqual(targets, ['target'])
            self.assertGreaterEqual(len(validated), 1)
            self.assertGreater(results[0][-1]['waited_for_chassis_start_s'], 0)
            self.bundle.tracker.cancel_all('test_done')

    def test_stop_while_waiting_for_chassis_start_never_replays_the_joint_request(self):
        targets, errors = [], []
        def motion():
            try:
                self.bundle.start_motion('lower_body', 'leg', (3,), lambda: targets.append(True),
                                         request_epoch=self.bundle._stop_epoch)
            except Exception as exc:
                errors.append(exc)
        with self.held_chassis_start() as release:
            thread = threading.Thread(target=motion)
            thread.start()
            time.sleep(.05)
            stop = threading.Thread(target=lambda: self.bundle.pause('test_stop'))
            stop.start()
            thread.join(1)
            self.assertFalse(thread.is_alive())
            self.assertIsInstance(errors[0], InterruptedError)
            release.set()
            stop.join(1)
        self.assertFalse(targets)

    def test_reconnect_and_timeout_while_waiting_never_send_a_joint_target(self):
        targets, errors = [], []
        def motion():
            try:
                self.bundle.start_motion('lower_body', 'leg', (3,), lambda: targets.append(True))
            except Exception as exc:
                errors.append(exc)
        with self.held_chassis_start():
            with patch.object(driver, 'CHASSIS_PREPARATION_WAIT_S', .04):
                motion()
            self.assertIsInstance(errors[-1], TimeoutError)
            self.assertIn('No motion target was sent', str(errors[-1]))
            thread = threading.Thread(target=motion)
            thread.start()
            time.sleep(.04)
            self.ros.connection = (True, 2)
            thread.join(1)
            self.assertIsInstance(errors[-1], ConnectionError)
            self.assertFalse(thread.is_alive())
        self.assertFalse(targets)

    def test_other_motion_preparation_still_rejects_with_its_owner(self):
        with self.bundle.tracker.lock:
            token = self.bundle._new_transition('prepare_motion')
            token['tool'] = 'head'
        try:
            result = self.call('leg', 'move_joints', positions=[0.] * 4)
            self.assertIn('preparing=prepare_motion; owner=head', result['error'])
            ownership = self.bundle.control_ownership()
            self.assertEqual(ownership['transition_owner'], 'head')
            self.assertGreaterEqual(ownership['transition_age_s'], 0.)
            self.assertFalse(self.ros.published)
        finally:
            self.bundle._end_control(token)

    def test_joint_limits_are_revalidated_after_waiting_before_any_enable_or_target(self):
        results = []
        with self.held_chassis_start() as release:
            thread = threading.Thread(target=lambda: results.append(
                self.call('leg', 'move_joints', positions=[0.] * 4)))
            thread.start()
            time.sleep(.05)
            self.ros.params['robot_version'] += 1
            self.bundle.sensors.discover()
            self.ros.raw()
            release.set()
            thread.join(1)
            self.assertFalse(thread.is_alive())
            self.assertEqual(results[0]['state'], 'error')
            self.assertIn('robot_version', results[0]['error'])
            self.assertFalse(self.ros.sent_services)
            self.assertFalse(self.ros.published)

    def test_pending_map_rejects_leg_without_pause_or_target(self):
        self.bundle.spatial._restored = True
        self.bundle.spatial._map_pending = {'operation': 'stop_mapping', 'stage': 'SaveCartoMap'}
        with patch.object(self.bundle, 'pause') as pause:
            result = self.call('leg', 'set_height', height=.9)
            self.assertEqual(result['state'], 'error')
            self.assertIn('map transition', result['error'])
            self.assertIn('stop_nav does not finish', result['error'])
            pause.assert_not_called()
            self.assertFalse(self.ros.sent_services)
            self.assertFalse(self.ros.published)
        self.bundle.spatial._map_pending = None

    def test_native_navigation_disables_upper_velocity_and_base_drive_restores_it(self):
        observed = []
        def publish():
            observed.append(self.bundle.state.read('upper_velocity_enabled')[0])
        first = self.bundle.start_motion('base', 'controlled_spatial', (2, 3), publish)
        self.assertEqual(observed, [False])
        self.assertEqual(first[-1]['velocity_owner'], 'native_navigation')
        self.assertIn(('/enable_vel_control', 'std_srvs/SetBool', {'data': False}), self.ros.sent_services)
        self.bundle.tracker.cancel_all('test_done')
        second = self.bundle.start_motion('base', 'base_drive', (2, 3), publish)
        self.assertEqual(observed, [False, True])
        self.assertEqual(second[-1]['velocity_owner'], 'upper_mpc')
        self.assertEqual(self.ros.sent_services[-1],
                         ('/enable_vel_control', 'std_srvs/SetBool', {'data': True}))

    def test_velocity_handoff_requires_feedback_before_any_navigation_target(self):
        original = self.ros.call_service
        def missing_feedback(service, service_type, args, timeout):
            if service == '/enable_vel_control':
                return {'success': True}
            return original(service, service_type, args, timeout)
        self.ros.call_service = missing_feedback
        published = []
        with patch.object(driver, 'CONTROL_TIMEOUT_S', .03):
            with self.assertRaisesRegex(TimeoutError, 'upper_velocity_enabled'):
                self.bundle.start_motion('base', 'controlled_spatial', (2, 3),
                                         lambda: published.append(True))
        self.assertFalse(published)
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_native_navigation_disables_upper_velocity_before_resuming(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        self.bundle.start_motion('base', 'controlled_spatial', (3,), lambda: None)
        self.assertEqual(self.ros.sent_services[:2], [
            ('/enable_vel_control', 'std_srvs/SetBool', {'data': False}),
            ('/enable_control', 'std_srvs/SetBool', {'data': True}),
        ])

    def test_stop_during_velocity_handoff_retains_the_older_remote_request(self):
        entered, release, errors = threading.Event(), threading.Event(), []
        original = self.ros.wait_service
        def wait_service(pending, timeout, cancelled=None):
            if pending[0] == '/enable_vel_control':
                entered.set()
                release.wait(2)
                self.ros.emit('/enable_vel_control_state', pending[2]['data'])
                self.ros.service_results[id(pending)] = {'success': True}
            return original(pending, timeout, cancelled)
        def start():
            try:
                self.bundle.start_motion('base', 'controlled_spatial', (2, 3), lambda: None)
            except Exception as exc:
                errors.append(exc)
        with patch.object(self.ros, 'wait_service', side_effect=wait_service):
            thread = threading.Thread(target=start)
            thread.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('base_drive', 'stop_move')['state'], 'paused')
                self.assertTrue(self.bundle._mode_unresolved())
            finally:
                release.set()
                thread.join(2)
        self.assertTrue(errors)
        self.assertFalse(self.bundle._mode_unresolved())
        self.assertIs(self.bundle.state.read('control_enabled')[0], False)

    def test_mapping_restores_remote_velocity_without_resuming_motors(self):
        self.ros.emit('/enable_vel_control_state', False)
        self.ros.emit('/enable_control_state', False)
        self.bundle.prepare_mapping_velocity(self.bundle._stop_epoch)
        self.assertIs(self.bundle.state.read('upper_velocity_enabled')[0], True)
        self.assertIs(self.bundle.state.read('control_enabled')[0], False)
        self.assertEqual(self.ros.sent_services,
                         [('/enable_vel_control', 'std_srvs/SetBool', {'data': True})])

    def test_metadata_and_schema_actions_agree(self):
        import yaml
        self.assertTrue(self.bundle.state.snapshot()['motion_enabled'])
        tools = {tool["name"]: tool for tool in self.bundle.get_all_tools()}
        metadata = yaml.safe_load((ROOT / "driver.yaml").read_text())
        self.assertEqual(set(tools), {c["name"] for c in metadata["cards"]})
        for name, tool in tools.items():
            if tool['type'] != 'actuator':
                continue
            schema = tool["inputSchema"]
            self.assertEqual(set(schema["properties"]["action"]["enum"]) - ({"start", "stop", "info"} if name == "base_servo" else set()),
                             set(schema["x-action-params"]))
            self.assertTrue(all(set(a["params"]) <= set(schema["properties"])
                                for a in schema["x-action-params"].values()))
            if not tool.get('topic_in'):
                self.assertIn("x-completion", schema)
            self.assertIn("x-resource", schema)
        self.assertNotIn("waist", tools)

    def test_actuator_info_preserves_diagnostics_without_public_control_cards(self):
        self.bundle.startup_checks.status = {'state': 'ready'}
        result = self.call('base_drive', 'info')
        self.assertEqual(result['feedback']['host_runtime'], {'state': 'ready'})
        self.assertIn('robot_discovery', result['feedback'])
        self.assertTrue({'state', 'switch_mode'}.isdisjoint(
            {tool['name'] for tool in self.bundle.get_all_tools()}))

    def test_connection_settings_are_not_exposed_on_cards(self):
        for tool in self.bundle.get_all_tools():
            if tool['name'] in ('mic', 'ext_mic'):
                self.assertEqual(set(tool['configSchema']['properties']), {'device_index'})
            else:
                self.assertNotIn('configSchema', tool)
                self.assertNotIn('config', tool['inputSchema'].get('properties', {}).get('action', {}).get('enum', []))

    def test_model_without_action_remains_a_resource_fetch_across_lifecycle(self):
        model = {'urdf': self.bundle.sensors._model_path.read_text()}
        self.assertEqual(self.bundle.dispatch('model', {}), model)
        self.assertEqual(self.call('model', 'start'), {'state': 'running'})
        self.assertEqual(self.call('model', 'info'), {'state': 'running'})
        self.assertEqual(self.bundle.dispatch('model', {}), model)
        self.assertEqual(self.call('model', 'stop'), {'state': 'idle'})
        self.assertEqual(self.bundle.dispatch('model', {}), model)
        self.assertFalse(self.ros.published)
        self.assertFalse(self.ros.sent_services)

    def test_removed_loco_config_cannot_change_connections_or_send_commands(self):
        manager = self.bundle.startup_checks.provisioner
        before = {role: manager._target(role) for role in ('host', 'controller', 'chassis')}
        with patch.object(manager.transport, 'run') as run:
            result = self.call('base_drive', 'config', provision_host_address='192.0.2.1',
                               provision_controller_password='unused')
        self.assertEqual(result['state'], 'error')
        self.assertEqual(before, {role: manager._target(role) for role in before})
        run.assert_not_called()
        self.assertFalse(self.ros.published)
        self.assertFalse(self.ros.sent_services)

    def test_removed_control_card_cannot_be_reenabled_by_legacy_configuration(self):
        legacy = driver.KuavoBundle({'plugins': {'switch_mode': {'enabled': True},
                                                'base_drive': {'enabled': True}}},
                                    'legacy', FakeRos1(), FakeNode())
        try:
            self.assertNotIn('switch_mode', legacy._enabled)
            self.assertEqual([tool['name'] for tool in legacy.get_all_tools()], ['base_drive'])
        finally:
            legacy.close()
        for configure in (False, True):
            if configure:
                self.bundle._enabled.add('switch_mode')  # Old operator config cannot recreate a plugin.
            for action in ('start', 'resume', 'pause', 'set_mode', 'get_current_mode'):
                self.assertIsNone(self.bundle.dispatch('switch_mode', {'action': action, 'mode': 3}))
        self.assertEqual(self.ros.sent_services, [])
        self.assertEqual(self.ros.published, [])

    def test_state_start_stop_and_ros2_forwarding(self):
        self.bundle.state._publish()
        self.assertEqual(self.node.publisher.messages, [])
        self.assertEqual(self.call("state", "start")["state"], "running")
        self.bundle.state._publish()
        state = json.loads(self.node.publisher.messages[-1].data)
        self.assertTrue(state["connected"])
        self.assertIs(state["fields"]["control_enabled"]["value"], True)
        self.assertEqual(self.bundle.state.tool()["topic_out"], [{"topic": "/test_robot/state/control", "format": "data/json"}])
        self.call("state", "stop")
        before = len(self.node.publisher.messages)
        self.bundle.state._publish()
        self.assertEqual(before, len(self.node.publisher.messages))
        self.assertEqual(self.call("state", "start")["state"], "running")
        self.assertIsNone(self.bundle.dispatch("state", {"action": "info"}))
        self.assertNotIn('state', {tool['name'] for tool in self.bundle.get_all_tools()})

    def test_reconnection_invalidates_old_feedback_even_if_disconnect_missed(self):
        self.ros.connection = (True, 2)
        self.assertEqual(self.bundle.state.snapshot()["fields"], {})
        result = self.call("arm", "move_joints", positions=[0] * 14)
        self.assertEqual(result["state"], "error")
        self.assertEqual(self.ros.published, [])

    def test_disconnected_state_not_reported_ready(self):
        self.call('state', 'start')
        self.ros.connection = (False, 1)
        self.bundle.state._publish()
        self.assertFalse(json.loads(self.node.publisher.messages[-1].data)['connected'])
        self.assertFalse(self.call("state", "info")["feedback"]["connected"])
        self.assertEqual(self.call("base_drive", "start")["state"], "error")  # Chassis preparation needs a live connection.
        self.assertEqual(self.call("base_drive", "info")["state"], "disconnected")

    def test_non_chassis_lifecycle_never_waits_for_control_lock_or_calls_hardware(self):
        entered, release = threading.Event(), threading.Event()
        def hold_lock():
            with self.bundle.tracker.lock:
                entered.set()
                release.wait(2)
        thread = threading.Thread(target=hold_lock)
        thread.start()
        self.assertTrue(entered.wait(1))
        try:
            with patch.object(self.bundle, 'pause', side_effect=AssertionError('lifecycle must not pause')):
                for plugin in self.bundle.plugins:
                    if plugin.PREFIX == 'base_drive':
                        continue  # Explicit start now prepares the chassis.
                    self.assertEqual(self.call(plugin.PREFIX, 'start'), {'state': 'ready'})
                    self.assertEqual(self.call(plugin.PREFIX, 'stop'), {'state': 'idle'})
            self.assertTrue(thread.is_alive())
            self.assertEqual(self.ros.services, [])
            self.assertEqual(self.ros.published, [])
        finally:
            release.set()
            thread.join(1)

    def test_lifecycle_stop_keeps_active_action_tracked_until_explicit_interrupt(self):
        action = self.call('arm', 'set_pose', frame='world', left_position=[0, .4, 0],
                           right_position=[0, -.4, 0], left_quaternion=[0, 0, 0, 1],
                           right_quaternion=[0, 0, 0, 1])
        before = len(self.ros.published)
        self.assertEqual(self.call('arm', 'stop'), {'state': 'idle'})
        self.assertTrue(self.bundle.tracker.current('arms', action['action_id']))
        self.assertEqual(len(self.ros.published), before)
        self.assertEqual(self.ros.services, [])
        self.assertEqual(self.call('arm', 'stop_move')['state'], 'paused')
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual([c[:3] for c in self.callbacks], [(action['action_id'], 'arm', 'cancelled')])

    def test_boot_initialization_gate_blocks_motion_resume_and_mode_but_not_pause(self):
        for state in ('waiting_hardware', 'initializing', 'pausing', 'failed'):
            self.bundle.state._boot_update({'data': json.dumps({'state': state, 'boot_pause_confirmed': False})})
            for action, args in [('resume', {}), ('set_mode', {'mode': 3})]:
                result = self.call('_control', action, **args)
                self.assertIn('boot initialization/pause is not confirmed', result['error'])
            with self.assertRaisesRegex(RuntimeError, 'boot initialization'):
                self.bundle.state.require_motion()
        self.assertEqual(self.ros.sent_services, [])
        self.assertEqual(self.call('_control', 'pause')['state'], 'paused')

    def test_boot_completion_is_not_a_current_enable_signal_and_stales_closed(self):
        self.bundle.state._boot_update({'data': json.dumps({'state': 'complete', 'boot_pause_confirmed': True})})
        self.bundle.state.require_boot_complete()
        self.assertTrue(self.bundle.state.confirmed_enable())  # Boot report never overwrites live state.
        self.bundle.state._controller_boot_at -= 4
        with self.assertRaisesRegex(RuntimeError, 'boot initialization'):
            self.bundle.state.require_boot_complete()
        self.ros.connection = (False, 2)
        self.bundle.state.snapshot()
        self.ros.connection = (True, 3)
        self.bundle.state.snapshot()
        with self.assertRaisesRegex(RuntimeError, 'boot initialization'):
            self.bundle.state.require_boot_complete()

    def test_explicit_boot_requirement_blocks_before_first_owner_message(self):
        self.bundle.state._boot_required = True
        with self.assertRaisesRegex(RuntimeError, 'boot initialization'):
            self.bundle.state.require_boot_complete()

    def test_internal_mode_and_resume_keep_missing_controller_gate(self):
        self.ros.params.clear()
        self.bundle.sensors.discover()
        for action, args in [('set_mode', {'mode': 3}), ('resume', {})]:
            with self.subTest(action=action):
                result = self.call('_control', action, **args)
                self.assertIn('controller_not_ready', result['error'])
        self.assertEqual(self.ros.sent_services, [])
        self.assertEqual(self.ros.published, [])

    def test_control_lock_contention_returns_busy_instead_of_waiting_unbounded(self):
        entered, release = threading.Event(), threading.Event()
        def hold_lock():
            with self.bundle.tracker.lock:
                entered.set()
                release.wait(2)
        thread = threading.Thread(target=hold_lock)
        thread.start()
        self.assertTrue(entered.wait(1))
        try:
            result = self.call('_control', 'set_mode', mode=3)
            self.assertIn('busy', result['error'])
            self.assertTrue(thread.is_alive())
            self.assertEqual(self.ros.services, [])
        finally:
            release.set()
            thread.join(1)

    def test_control_transitions_share_one_service_and_feedback_deadline(self):
        for action in ('pause', 'set_mode', 'resume'):
            with self.subTest(action=action):
                self.ros.confirm = False
                self.ros.emit('/enable_control_state', action != 'resume')
                started = time.monotonic()
                clock = [started]
                def sleep(seconds):
                    clock[0] += seconds
                def delayed_service(*args):
                    self.assertAlmostEqual(args[-1], driver.CONTROL_TIMEOUT_S)
                    clock[0] += 1.5  # Consumes the same budget as the following wait.
                    return {'success': True, 'result': True}
                fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], sleep=sleep)
                with patch.object(driver, 'time', fake_time), patch.object(self.ros, 'call_service', side_effect=delayed_service):
                    result = self.call('_control', action, mode=3)
                self.assertEqual(result['state'], 'error')
                self.assertIn('not confirmed', result['error'])
                self.assertAlmostEqual(clock[0] - started, driver.CONTROL_TIMEOUT_S)
        self.ros.confirm = True

    def test_control_confirmation_rejects_feedback_from_reconnected_service_session(self):
        original = self.ros.call_service
        def reconnect(*args):
            self.ros.connection = (True, 2)
            return original(*args)
        with patch.object(self.ros, 'call_service', side_effect=reconnect):
            result = self.call('_control', 'set_mode', mode=3)
        self.assertEqual(result['state'], 'error')
        self.assertIn('connection changed', result['error'])

    def test_stops_preempt_blocked_resume_and_mode_service_without_waiting(self):
        for transition, card, stop_action in [('resume', 'base_drive', 'stop_move'),
                                               ('set_mode', '_control', 'pause')]:
            with self.subTest(transition=transition):
                self.ros.emit('/enable_control_state', transition != 'resume')
                self.bundle.state.verify_enable()
                entered, release = threading.Event(), threading.Event()
                results = []
                original = self.ros.wait_service
                def blocked(pending, timeout, cancelled=None):
                    if (pending[0] == '/mobile_manipulator_mpc_control'
                            or (pending[0] == '/enable_control' and pending[2]['data'])):
                        entered.set()
                        release.wait(2)
                    return original(pending, timeout, cancelled)
                with patch.object(self.ros, 'wait_service', side_effect=blocked):
                    worker = threading.Thread(target=lambda: results.append(
                        self.call('_control', transition, mode=3)))
                    worker.start()
                    try:
                        self.assertTrue(entered.wait(1))
                        before = len(self.ros.published)
                        stopped = self.call(card, stop_action)
                        self.assertEqual(stopped['state'], 'error' if transition == 'resume' else 'paused')
                        if transition == 'resume':
                            self.assertFalse(stopped['physical_stop_confirmed'])
                            self.assertTrue(stopped['remote_resume_unresolved'])
                        self.assertTrue(worker.is_alive())  # Old service is STILL blocked.
                        self.assertFalse(release.is_set())
                        self.assertEqual(self.ros.sent_services[-1][0::2], ('/enable_control', {'data': False}))
                        self.assertEqual(len(self.ros.published) - before, 2)
                        self.assertEqual(self.ros.published[-2][2]['linear']['x'], 0.)
                        self.assertEqual(self.ros.published[-1][2]['joint_data'], [0., 0.])
                        # Even a late True notification cannot release the local stop.
                        self.ros.emit('/enable_control_state', True)
                    finally:
                        release.set()
                        worker.join(1)
                self.assertFalse(worker.is_alive())
                self.assertIn('superseded', results[0]['error'])
                self.assertTrue(self.bundle.state.motion_blocked)
                self.assertEqual(self.call('base_drive', 'move', duration=.1)['state'], 'error')
                if transition == 'resume':
                    pending = self.bundle._remote_resume['pending']
                    self.ros.service_results[id(pending)] = {'success': False}
                    self.assertEqual(self.call('_control', 'pause')['state'], 'paused')

    def _timeout_remote_resume(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        with patch.object(self.ros, 'wait_service', side_effect=TimeoutError('remote request unresolved')):
            self.assertEqual(self.call('_control', 'resume')['state'], 'error')
        self.assertIsNone(self.bundle._transition)
        self.assertTrue(self.bundle.state.snapshot()['remote_resume_unresolved'])
        return self.bundle._remote_resume['pending']

    def test_late_resume_after_disable_feedback_never_confirms_that_pause(self):
        pending = self._timeout_remote_resume()
        original = self.bundle.confirm_control
        def disable_then_late_enable(token, *args, **kwargs):
            original(token, *args, **kwargs)
            # Older enable finishes AFTER the new disable was confirmed.
            self.ros.service_results[id(pending)] = {'success': True}
            self.ros.emit('/enable_control_state', True)
        with patch.object(self.bundle, 'confirm_control', side_effect=disable_then_late_enable):
            stopped = self.call('_control', 'pause')
        self.assertEqual(stopped['state'], 'error')
        self.assertFalse(stopped['physical_stop_confirmed'])
        self.assertTrue(self.bundle.state.confirmed_enable())
        self.assertTrue(self.bundle.state.motion_blocked)
        # Explicitly pausing again sends disable AFTER the old resume is settled.
        self.assertEqual(self.call('_control', 'pause')['state'], 'paused')
        self.assertFalse(self.bundle.state.confirmed_enable())
        self.assertFalse(self.bundle.state.snapshot()['remote_resume_unresolved'])
        self.assertEqual([s[2]['data'] for s in self.ros.sent_services], [True, False, False])

    def test_resume_acceptance_without_terminal_true_does_not_settle_hazard(self):
        pending = self._timeout_remote_resume()
        self.ros.service_results[id(pending)] = {'success': True}
        self.assertEqual(self.call('_control', 'pause')['state'], 'error')
        self.assertFalse(self.bundle.state.confirmed_enable())
        count = len(self.ros.sent_services)
        for action in ('resume', 'set_mode'):
            self.assertIn('remote resume is unresolved', self.call('_control', action, mode=3)['error'])
        self.assertEqual(len(self.ros.sent_services), count)
        self.assertTrue(self.bundle.state.remote_resume_unresolved)

    def test_explicit_resume_rejection_allows_confirmed_pause(self):
        pending = self._timeout_remote_resume()
        self.ros.service_results[id(pending)] = {'success': False}
        self.assertEqual(self.call('_control', 'pause')['state'], 'paused')
        self.assertFalse(self.bundle.state.remote_resume_unresolved)
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_precommand_true_cannot_settle_unresolved_resume(self):
        pending = self._timeout_remote_resume()
        self.ros.service_results[id(pending)] = {'success': True}
        with self.bundle.state._lock:
            self.bundle.state._values['control_enabled'] = True
            self.bundle.state._seen['control_enabled'] = self.bundle._remote_resume['issued'] - .01
        stopped = self.call('_control', 'pause')
        self.assertEqual(stopped['state'], 'error')
        self.assertFalse(stopped['physical_stop_confirmed'])
        self.assertTrue(self.bundle.state.remote_resume_unresolved)

    def test_reconnect_cannot_clear_unresolved_remote_resume(self):
        pending = self._timeout_remote_resume()
        self.ros.service_results[id(pending)] = {'success': True}
        self.ros.connection = (True, 2)
        self.bundle.sensors.discover()
        self.ros.raw()
        self.ros.emit('/enable_control_state', True)
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 3)
        self.bundle.state.verify_enable()
        self.assertEqual(self.call('_control', 'pause')['state'], 'error')
        self.assertTrue(self.bundle.state.remote_resume_unresolved)
        self.assertIn('unresolved', self.call('_control', 'resume')['error'])

    def test_resume_send_failure_cannot_be_mistaken_for_remote_cancellation(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        with patch.object(self.ros, 'start_service', side_effect=ConnectionError('partial send')):
            self.assertEqual(self.call('_control', 'resume')['state'], 'error')
        stopped = self.call('_control', 'pause')
        self.assertEqual(stopped['state'], 'error')
        self.assertFalse(stopped['physical_stop_confirmed'])
        self.assertTrue(stopped['remote_resume_unresolved'])

    def test_normal_resume_settles_remote_hazard_before_unblocking(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        self.assertEqual(self.call('_control', 'resume')['state'], 'ready')
        self.assertFalse(self.bundle.state.remote_resume_unresolved)
        self.assertFalse(self.bundle.state.motion_blocked)

    def test_stop_before_resume_send_prevents_the_old_command_from_being_sent(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        entered, release = threading.Event(), threading.Event()
        results = []
        original = self.bundle.confirm_control
        def delay(token, *args, **kwargs):
            if token['action'] == 'resume':
                entered.set()
                release.wait(2)
            return original(token, *args, **kwargs)
        with patch.object(self.bundle, 'confirm_control', side_effect=delay):
            worker = threading.Thread(target=lambda: results.append(self.call('_control', 'resume')))
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('_control', 'pause')['state'], 'paused')
                self.assertTrue(worker.is_alive())
            finally:
                release.set()
                worker.join(1)
        self.assertIn('superseded', results[0]['error'])
        self.assertEqual([r[2] for r in self.ros.sent_services], [{'data': False}])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_stop_preempts_service_wait_and_cancels_other_actions_exactly_once(self):
        # A pause service can itself be waiting while ACP actions are cancelled.
        action = self.call('base_drive', 'move', duration=1)
        entered, release = threading.Event(), threading.Event()
        results = []
        original = self.ros.wait_service
        first = [True]
        def blocked(pending, timeout, cancelled=None):
            if first[0]:
                first[0] = False
                entered.set()
                release.wait(2)
            return original(pending, timeout, cancelled)
        with patch.object(self.ros, 'wait_service', side_effect=blocked):
            worker = threading.Thread(target=lambda: results.append(self.call('_control', 'pause')))
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('base_drive', 'stop_move')['state'], 'paused')
                self.assertTrue(worker.is_alive())
                self.assertTrue(wait_until(lambda: bool(self.callbacks)))
                self.assertEqual([c[:3] for c in self.callbacks], [(action['action_id'], 'base_drive', 'cancelled')])
            finally:
                release.set()
                worker.join(1)
        self.assertIn('superseded', results[0]['error'])

    def test_stale_enable_never_allows_motion_resume_or_mode_shortcuts(self):
        self.bundle.state._enable_verified_at = time.monotonic() - 3600
        self.bundle.state._seen['control_enabled'] = time.monotonic() - 3600
        for card, args in [('base_drive', dict(action='move', duration=.1)),
                           ('_control', dict(action='resume')),
                           ('_control', dict(action='set_mode', mode=3))]:
            self.assertIn('verification is missing or stale', self.call(card, **args)['error'])
        self.assertEqual(self.ros.published, [])
        self.assertEqual(self.ros.services, [])

    def test_stale_false_does_not_skip_pause_confirmation(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state._seen['control_enabled'] = time.monotonic() - 3600
        self.bundle.state._enable_verified_at = time.monotonic() - 3600
        result = self.call('_control', 'pause')
        self.assertEqual(result['state'], 'paused')
        self.assertEqual(self.ros.services[-1][:3], ('/enable_control', 'std_srvs/SetBool', {'data': False}))

    def test_latched_enable_can_be_revalidated_without_enabling_the_robot(self):
        self.bundle.state._seen['control_enabled'] = time.monotonic() - 3600
        self.bundle.state._enable_verified_at = time.monotonic() - 3600
        self.bundle.state.verify_enable()
        self.assertIs(self.bundle.state.confirmed_enable(), True)
        self.assertTrue(self.bundle.state.snapshot()['enable_verification']['valid'])
        self.assertEqual(self.ros.publisher_checks[-1][2], {'topic': '/enable_control_state'})
        self.assertEqual(self.ros.services, [])
        self.assertEqual(self.ros.published, [])

    def test_enable_probe_requires_one_publisher_and_actual_redelivery(self):
        for publishers in ([], ['/one', '/two'], None):
            self.ros.enable_publishers = publishers
            self.bundle.state.verify_enable()
            with self.assertRaises(RuntimeError):
                self.bundle.state.confirmed_enable()
        self.ros.enable_publishers = ['/wheel_controller']
        self.ros.replay_enable = False
        self.bundle.state.verify_enable()
        self.assertIn('no control-enable latched delivery', self.bundle.state.snapshot()['enable_verification']['error'])
        with self.assertRaises(RuntimeError):
            self.bundle.state.confirmed_enable()

    def test_enable_probe_runs_off_callback_and_discards_a_reconnected_session(self):
        entered, release = threading.Event(), threading.Event()
        original = self.ros.call_service
        def blocked(*args):
            if args[0] == '/rosapi/publishers':
                entered.set()
                release.wait(2)
            return original(*args)
        with patch.object(self.ros, 'call_service', side_effect=blocked):
            self.bundle.state._enable_tick()
            try:
                self.assertTrue(entered.wait(1))
                first_thread = self.bundle.state._enable_probe_thread
                self.bundle.state._enable_tick()
                self.assertIs(self.bundle.state._enable_probe_thread, first_thread)
                self.ros.connection = (True, 2)
                self.assertFalse(self.bundle.state.snapshot()['enable_verification']['valid'])
            finally:
                release.set()
                self.bundle.state._enable_probe_thread.join(1)
        self.assertFalse(self.bundle.state.snapshot()['enable_verification']['valid'])

    def test_recent_enable_message_cannot_hide_an_expired_verification_lease(self):
        self.bundle.state._enable_verified_at = time.monotonic() - driver.ENABLE_LEASE_S - 1
        self.ros.emit('/enable_control_state', True)
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 3)
        self.assertIn('verification is missing or stale', self.call('base_drive', 'move', duration=.1)['error'])
        self.assertEqual(self.ros.published, [])

    def test_enable_verification_loss_aborts_inflight_action(self):
        action = self.call('base_drive', 'move', duration=1)
        self.ros.enable_publishers = []
        self.bundle.state.verify_enable()
        self.assertTrue(wait_until(lambda: bool(self.callbacks)))
        self.assertEqual(self.callbacks[0][:3], (action['action_id'], 'base_drive', 'error'))
        self.assertIn('control-enable', self.callbacks[0][3]['reason'])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_action_and_service_errors_are_bounded_single_line_logs(self):
        hostile = '\x00\x1b[31m\n' + 'x' * 100_000
        with patch('builtins.print') as log:
            result = self.call('arm', hostile)
        self.assertEqual(result['state'], 'error')
        for call in log.call_args_list:
            line = call[0][0]
            self.assertLess(len(line), 500)
            self.assertFalse(any(c in line for c in ('\n', '\x00', '\x1b')))
        self.ros.service_accepts = False
        with patch.object(self.ros, 'call_service', return_value={'success': False, 'message': hostile}), patch('builtins.print') as log:
            result = self.call('_control', 'pause')
        self.assertEqual(result['state'], 'error')
        self.assertLessEqual(len(result['error']), 200)
        self.assertTrue(log.called)
        for call in log.call_args_list:
            line = call[0][0]
            self.assertLess(len(line), 500)
            self.assertFalse(any(c in line for c in ('\n', '\x00', '\x1b')))

    def test_cartesian_arm_normalizes_mode_four_and_keeps_reach_time_completion(self):
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 4)
        self.bundle.sensors._data.pop('joints', None)
        result = self.call('arm', 'set_pose', frame='local', left_position=[0, .4, 0],
                           right_position=[0, -.4, 0], left_quaternion=[0, 0, 0, 1],
                           right_quaternion=[0, 0, 0, 1])
        self.assertIn('action_id', result)
        self.assertEqual(self.ros.published[-1][2]['frame'], 2)
        self.ros.emit('/lb_arm_ee_reach_time', 0)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result['action_id'], 'arm', 'completed'))
        self.assertFalse(self.callbacks[0][3]['measured_convergence'])
        self.assertEqual(self.ros.sent_services[0][2], {'control_mode': 3})

    def test_motion_prepares_enable_stale_or_incompatible_mode_but_still_requires_warmup(self):
        for setup in (
            lambda: self.ros.emit("/enable_control_state", False),
            lambda: self.bundle.state._seen.update(mpc_mode=time.monotonic() - 3),
            lambda: self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 1),
        ):
            self.ros.emit("/enable_control_state", True)
            self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 3)
            setup()
            result = self.call('base_drive', 'move', duration=.1)
            self.assertEqual(result['state'], 'executing')
            self.assertEqual(result['control_preparation']['mode'], 3)
            self.call('base_drive', 'stop_move')
        self.ros.motion_ready = False
        boundary, calls = len(self.ros.published), len(self.ros.sent_services)
        self.assertIn('warming up', self.call('base_drive', 'move', duration=.1)['error'])
        self.assertEqual((len(self.ros.published), len(self.ros.sent_services)), (boundary, calls))

    def test_rejects_invalid_values_before_hardware_calls(self):
        for args in ({"vx": 1, "duration": 1}, {"vx": True, "duration": 1},
                     {"vx": float("nan"), "duration": 1}, {"duration": 0}, {"duration": "1"}):
            self.assertEqual(self.call("base_drive", "move", **args)["state"], "error")
        self.assertEqual(self.call("arm", "move_joints", positions=[0] * 13)["state"], "error")
        self.assertIsNone(self.call("waist", "set_pose", x=0))
        self.assertEqual(self.ros.published, [])
        self.assertEqual(self.ros.services, [])

    def test_paused_action_resumes_then_confirms_mode_before_target_publication(self):
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 1)
        self.ros.emit('/enable_control_state', False)
        order, send, publish = [], self.ros.start_service, self.ros.publish
        def service(name, kind, args):
            order.append((name, args))
            return send(name, kind, args)
        def target(*args):
            order.append((args[0], args[2]))
            return publish(*args)
        with patch.object(self.ros, 'start_service', side_effect=service), patch.object(self.ros, 'publish', side_effect=target):
            result = self.call('base_drive', 'move', vx=.05, duration=.1)
        self.assertEqual(result['state'], 'executing')
        self.assertEqual(order[:2], [('/enable_control', {'data': True}),
                                    ('/mobile_manipulator_mpc_control', {'control_mode': 3})])
        self.assertEqual(order[2][0], '/cmd_vel')
        self.assertEqual(order[2][1]['linear']['x'], .05)

    def test_arm_automatically_changes_base_only_to_base_arm(self):
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 2)
        result = self.call('arm', 'move_joints', positions=[0.] * 14)
        self.assertEqual(result['state'], 'executing')
        self.assertEqual(result['control_preparation'], {'automatic': True, 'resumed': False,
                                                       'mode_changed': True, 'mode': 3})
        self.assertEqual(self.ros.sent_services[0][2], {'control_mode': 3})
        self.assertEqual(self.ros.published[0][0], '/kuavo_arm_traj')

    def test_arm_only_is_normalized_to_shared_base_arm_before_motion(self):
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 1)
        result = self.call('arm', 'move_joints', positions=[0.] * 14)
        self.assertEqual(result['state'], 'executing')
        self.assertEqual(result['control_preparation']['mode'], 3)
        self.assertEqual(self.ros.sent_services, [('/mobile_manipulator_mpc_control',
                                                'kuavo_msgs/changeTorsoCtrlMode', {'control_mode': 3})])

    def test_all_motion_cards_prepare_after_pause_and_invalid_joint_target_never_resumes(self):
        for card, action, args in (
            ('arm', 'move_joints', {'positions': [0.] * 14}),
            ('leg', 'move_joints', {'positions': [0.] * 4}),
            ('head', 'move_pos', {'yaw': 0., 'pitch': 0.}),
            ('arm', 'set_pose', {'frame': 'local', 'left_position': [0., .4, 0.],
                'right_position': [0., -.4, 0.], 'left_quaternion': [0., 0., 0., 1.],
                'right_quaternion': [0., 0., 0., 1.]})):
            with self.subTest(card=card, action=action):
                self.call('base_drive', 'stop_move')
                self.ros.raw()
                result = self.call(card, action, **args)
                self.assertEqual(result['state'], 'executing')
                self.assertTrue(result['control_preparation']['resumed'])
                self.call(card, 'stop_move')
        self.ros.sent_services.clear()
        self.assertIn('outside', self.call('arm', 'move_joints', positions=[10000.] * 14)['error'])
        self.bundle.sensors._joint_limit_config = None
        self.assertIn('not configured', self.call('arm', 'move_joints', positions=[0.] * 14)['error'])
        self.assertEqual(self.ros.sent_services, [])

    def test_failed_automatic_mode_preparation_pauses_without_sending_target(self):
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 1)
        self.ros.emit('/enable_control_state', False)
        original = self.ros.call_service
        def reject_mode(service, *args):
            if service == '/mobile_manipulator_mpc_control':
                return {'result': False, 'message': 'mode rejected'}
            return original(service, *args)
        with patch.object(self.ros, 'call_service', side_effect=reject_mode):
            result = self.call('base_drive', 'move', vx=.2, duration=.1)
        self.assertEqual(result['state'], 'error')
        self.assertFalse(self.bundle.state.confirmed_enable())
        self.assertFalse(self.bundle.tracker._active)
        self.assertEqual([s[2] for s in self.ros.sent_services],
                         [{'data': True}, {'control_mode': 3}, {'data': False}])
        self.assertFalse(any(p[0] == '/cmd_vel' and p[2]['linear']['x'] for p in self.ros.published))

    def test_automatic_mode_needs_post_request_feedback(self):
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 1)
        self.ros.confirm = False
        with patch.object(driver, 'CONTROL_TIMEOUT_S', .04):
            result = self.call('base_drive', 'move', vx=.2, duration=.1)
        self.assertEqual(result['state'], 'error')
        self.assertIn('not confirmed', result['error'])
        self.assertFalse(self.bundle.tracker._active)
        self.assertFalse(any(p[0] == '/cmd_vel' and p[2]['linear']['x'] for p in self.ros.published))

    def test_automatic_resume_waits_for_mpc_before_any_mode_write_or_target(self):
        self.ros.emit('/enable_control_state', False)
        original = self.ros.call_service
        def enable_without_mpc(service, *args):
            if service == '/enable_control' and args[1].get('data') is True:
                callback = self.ros.subscriptions.pop('/mobile_manipulator/lb_mpc_control_mode')
                try:
                    return original(service, *args)
                finally:
                    self.ros.subscriptions['/mobile_manipulator/lb_mpc_control_mode'] = callback
            return original(service, *args)
        with patch.object(self.ros, 'call_service', side_effect=enable_without_mpc), patch.object(driver, 'CONTROL_TIMEOUT_S', .04):
            result = self.call('base_drive', 'move', vx=.2, duration=.1)
        self.assertIn('resumed MPC mode feedback not confirmed', result['error'])
        self.assertEqual([s[2] for s in self.ros.sent_services], [{'data': True}, {'data': False}])
        self.assertFalse(any(p[0] == '/cmd_vel' and p[2]['linear']['x'] for p in self.ros.published))

    def test_action_never_auto_resumes_before_boot_confirmation(self):
        self.bundle.state._boot_update({'data': json.dumps({'state': 'initializing', 'boot_pause_confirmed': False})})
        self.ros.emit('/enable_control_state', False)
        self.assertIn('boot initialization/pause is not confirmed', self.call('base_drive', 'move', duration=.1)['error'])
        self.assertEqual(self.ros.sent_services, [])

    def test_global_reconfiguration_does_not_change_an_inflight_action(self):
        arm = self.call('arm', 'move_joints', positions=[0.] * 14)
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 1)
        before = list(self.ros.sent_services)
        self.assertIn('another action', self.call('base_drive', 'move', duration=.1)['error'])
        self.assertEqual(self.ros.sent_services, before)
        self.assertTrue(self.bundle.tracker.current('arms', arm['action_id']))

    def test_base_arm_mode_allows_independent_channels_without_reconfiguration(self):
        self.assertEqual(self.call('arm', 'move_joints', positions=[0.] * 14)['state'], 'executing')
        self.assertEqual(self.call('base_drive', 'move', duration=.1)['state'], 'executing')
        self.assertEqual(self.ros.sent_services, [])

    def test_stop_between_auto_resume_and_mode_cannot_recover_or_cancel_a_new_action(self):
        self.ros.emit('/enable_control_state', False)
        original = self.bundle.confirm_control
        entered, release, results = threading.Event(), threading.Event(), []
        first = [True]
        def confirm(token, service, *args):
            result = original(token, service, *args)
            if service == '/enable_control' and args[1].get('data') is True and first[0]:
                first[0] = False
                entered.set()
                release.wait(2)
            return result
        with patch.object(self.bundle, 'confirm_control', side_effect=confirm):
            thread = threading.Thread(target=lambda: results.append(self.call('base_drive', 'move', vx=.2, duration=1)))
            thread.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('base_drive', 'stop_move')['state'], 'paused')
                fresh = self.call('base_drive', 'move', duration=1)
                self.assertEqual(fresh['state'], 'executing')
                boundary = len(self.ros.sent_services)
            finally:
                release.set()
                thread.join(1)
        self.assertIn('superseded', results[0]['error'])
        self.assertEqual(len(self.ros.sent_services), boundary)
        self.assertTrue(self.bundle.tracker.current('base', fresh['action_id']))
        self.assertFalse(any(p[0] == '/cmd_vel' and p[2]['linear']['x'] == .2 for p in self.ros.published))

    def test_stop_during_auto_mode_wait_blocks_new_motion_until_old_write_settles(self):
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 1)
        original = self.ros.wait_service
        entered, release, results = threading.Event(), threading.Event(), []
        first = [True]
        def wait(pending, timeout, cancelled=None):
            if pending[0] == '/mobile_manipulator_mpc_control' and first[0]:
                first[0] = False
                entered.set()
                release.wait(2)
                # Emulate a remote write that executes despite local cancellation.
                self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 3)
                self.ros.service_results[id(pending)] = {'result': True, 'mode': 3}
            return original(pending, timeout, cancelled)
        with patch.object(self.ros, 'wait_service', side_effect=wait):
            thread = threading.Thread(target=lambda: results.append(self.call('base_drive', 'move', vx=.2, duration=1)))
            thread.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('base_drive', 'stop_move')['state'], 'paused')
                before = len(self.ros.sent_services)
                self.assertIn('mode change is unresolved', self.call('base_drive', 'move', duration=1)['error'])
                self.assertEqual(len(self.ros.sent_services), before)
            finally:
                release.set()
                thread.join(1)
        self.assertIn('superseded', results[0]['error'])
        self.assertEqual(self.call('base_drive', 'move', duration=.1)['state'], 'executing')

    def test_stop_during_argument_validation_cannot_be_undone_by_the_old_action(self):
        entered, release, result = threading.Event(), threading.Event(), []
        original = driver._number
        def number(value, name, *args):
            if name == 'vx':
                entered.set()
                release.wait(2)
            return original(value, name, *args)
        with patch.object(driver, '_number', side_effect=number):
            thread = threading.Thread(target=lambda: result.append(self.call('base_drive', 'move', vx=.2, duration=1)))
            thread.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('base_drive', 'stop_move')['state'], 'paused')
                boundary = len(self.ros.sent_services)
            finally:
                release.set()
                thread.join(1)
        self.assertIn('superseded by stop', result[0]['error'])
        self.assertEqual(len(self.ros.sent_services), boundary)
        self.assertFalse(any(p[0] == '/cmd_vel' and p[2]['linear']['x'] for p in self.ros.published))

    def test_legacy_loco_config_enables_only_renamed_base_drive(self):
        bundle = driver.KuavoBundle({'plugins': {'loco': {'enabled': True}}},
                                   'renamed_robot', self.ros, self.node)
        try:
            tools = bundle.get_all_tools()
            self.assertEqual([t['name'] for t in tools], ['base_drive'])
            self.assertEqual(bundle.dispatch('base_drive', {'action': 'info'})['state'], 'ready')
        finally:
            bundle.close()

    def test_explicit_base_drive_disable_overrides_legacy_loco_enable(self):
        bundle = driver.KuavoBundle({'plugins': {'loco': {'enabled': True},
                                               'base_drive': {'enabled': False}}},
                                   'renamed_robot', self.ros, self.node)
        try:
            self.assertEqual(bundle.get_all_tools(), [])
        finally:
            bundle.close()

    def test_arm_measured_completion_needs_no_vendor_reach_time(self):
        result = self.call("arm", "move_joints", positions=list(range(14)))
        self.assertIn("action_id", result)
        self.assertEqual(self.ros.services, [])
        self.assertEqual(self.ros.published[-1], ("/kuavo_arm_traj", "sensor_msgs/JointState", {
            "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
            "name": [f"joint{i}" for i in range(1, 15)], "position": list(range(14)), "velocity": [], "effort": []}))
        time.sleep(0.06)
        self.assertEqual(self.callbacks, [])  # No measured convergence yet.
        self.ros.raw([0] * 4 + [math.radians(v) for v in range(14)] + [0] * 2)
        time.sleep(0.06)
        self.assertEqual(self.callbacks, [])  # A single in-tolerance sample is insufficient.
        time.sleep(0.17)
        self.ros.raw([0] * 4 + [math.radians(v) for v in range(14)] + [0] * 2)
        self.assertTrue(wait_until(lambda: len(self.callbacks) == 1))
        self.assertEqual(self.callbacks[0][0:3], (result["action_id"], "arm", "completed"))
        self.assertIs(self.callbacks[0][3]["measured_convergence"], True)

    def test_leg_and_waist_share_busy_channel(self):
        first = self.call("leg", "move_joints", positions=[1, 2, 3, 4])
        self.assertIn("action_id", first)
        second = self.call("leg", "set", knee_deg=10.)
        self.assertIn("busy", second["error"])
        self.assertEqual(len(self.ros.published), 1)

    def test_cartesian_wire_type_frame_quaternion(self):
        args = dict(frame="world", left_position=[0, 0.4, 0], right_position=[0, -0.4, 0],
                    left_quaternion=[0, 0, 0, 1], right_quaternion=[0, 0, 0, 1])
        bad = self.call("arm", "set_pose", **{**args, "left_quaternion": [0] * 4})
        self.assertIn("normalized", bad["error"])
        result = self.call("arm", "set_pose", **args)
        self.assertIn("action_id", result)
        topic, msg_type, msg = self.ros.published[-1]
        self.assertEqual((topic, msg_type, msg["frame"]), ("/mm/two_arm_hand_pose_cmd", "kuavo_msgs/twoArmHandPoseCmd", 1))
        self.assertFalse(msg["use_custom_ik_param"])
        self.ros.emit("/lb_arm_ee_reach_time", 0)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result['action_id'], 'arm', 'completed'))
        self.assertEqual(self.callbacks[0][3]['completion_basis'], 'vendor_reach_time_estimate')
        self.assertFalse(self.callbacks[0][3]['measured_convergence'])
        self.assertEqual(self.ros.services, [])

    def test_loco_velocity_acp_waits_for_vendor_stop_window(self):
        result = self.call("base_drive", "move", vx=0.1, duration=0.1)
        self.assertIn("action_id", result)
        self.assertGreater(self.ros.published[0][2]["linear"]["x"], 0)
        self.assertTrue(wait_until(lambda: self.ros.published[-1][2]["linear"]["x"] == 0))
        self.assertEqual(self.callbacks, [])
        self.assertTrue(wait_until(lambda: self.callbacks, 1.3))
        self.assertEqual(self.callbacks[0][2], "completed")
        self.assertEqual(self.callbacks[0][3]["stop_delay_s"], 1.0)

    def test_base_busy_or_invalid_command_preserves_active_motion(self):
        velocity = self.call("base_drive", "move", vx=0.1, duration=1)
        bad = self.call("base_drive", "move_relative", x=0.2)
        self.assertEqual(bad['state'], 'error')
        pose = self.call("base_drive", "move_relative", x=0.2, y=0, yaw=0)
        self.assertIn('busy', pose['error'])
        self.assertTrue(self.bundle.tracker.current('base', velocity['action_id']))
        self.assertFalse(any(p[0] == '/cmd_pose' for p in self.ros.published))
        self.assertEqual(self.callbacks, [])

    def test_split_action_schemas_are_action_specific(self):
        tools = {t['name']: t for t in self.bundle.get_all_tools()}
        self.assertEqual(tools['base_drive']['inputSchema']['x-action-params']['move']['required'], ['duration'])
        self.assertEqual(set(tools['arm']['inputSchema']['properties']['action']['enum']), {'set', 'zero', 'stop_move'})

    def test_joint_command_rejected_without_fresh_measurement(self):
        self.bundle.sensors._seen['joints'] = time.monotonic() - 1
        result = self.call('arm', 'move_joints', positions=[0.] * 14)
        self.assertIn('stale', result['error'])
        self.assertEqual(self.ros.published, [])

    def test_missing_joint_limits_only_disables_joint_target_actions(self):
        self.bundle.sensors._joint_limit_config = None
        for card, size in (('arm', 14), ('leg', 4)):
            result = self.call(card, 'move_joints', positions=[0.] * size)
            self.assertIn('hardware-verified joint limits are not configured', result['error'])
        self.assertEqual(self.ros.published, [])
        self.assertEqual(self.callbacks, [])
        self.assertEqual(self.bundle.sensors.health()['joint_limits'],
                         {'arm': {'available': False}, 'leg': {'available': False}})
        self.assertTrue(self.bundle.state.motion_enabled)
        self.assertEqual(self.call('base_drive', 'move', duration=.1)['state'], 'executing')

    def test_vendor_runtime_arm_can_publish_and_missing_limits_abort_in_flight(self):
        self.bundle.sensors._joint_limit_config = None
        self.bundle.sensors._arm_limit_source = 'vendor_runtime'
        self.ros.params['humanoid_description'] = (ROOT / 'resource' / 'kuavo5w_model.urdf').read_text()
        self.ros.params['kuavo_configuration'] = {
            'NUM_JOINT': 20, 'NUM_ARM_JOINT': 14, 'NUM_HEAD_JOINT': 2,
            'min_joint_position_limits': [-180.] * 20,
            'max_joint_position_limits': [180.] * 20}
        self.bundle.sensors.discover()
        self.ros.raw()
        result = self.call('arm', 'set', left_joint_1_rad=.1)
        self.assertEqual(result['state'], 'executing')
        self.assertEqual(self.ros.published[-1][0], '/kuavo_arm_traj')
        self.assertAlmostEqual(self.ros.published[-1][2]['position'][0], -math.degrees(.1))
        self.ros.params['kuavo_configuration'] = None
        self.bundle.sensors._retry_at = 0
        self.bundle.sensors.tick()
        self.bundle.sensors._thread.join(1)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result['action_id'], 'arm', 'error'))
        self.assertIn('limits unavailable', self.callbacks[0][3]['reason'])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_extreme_arm_and_leg_targets_are_rejected_before_publication(self):
        for card, size in (('arm', 14), ('leg', 4)):
            for value in (-10000., 10000., -90.01, 90.01):
                with self.subTest(card=card, value=value):
                    result = self.call(card, 'move_joints', positions=[value] * size)
                    self.assertIn('outside hardware-verified limits', result['error'])
                    self.assertEqual(self.ros.published, [])
                    self.assertFalse(self.bundle.tracker._active)
        self.assertEqual(self.callbacks, [])
        self.assertFalse(self.bundle.state.motion_send_uncertain)

    def test_joint_limit_boundaries_are_inclusive(self):
        for card, size in (('arm', 14), ('leg', 4)):
            for value in (-90., 90.):
                with self.subTest(card=card, value=value):
                    self.assertEqual(self.call(card, 'move_joints', positions=[value] * size)['state'], 'executing')
                    self.assertEqual(self.ros.published[-1][2]['position'], [value] * size)
                    self.assertEqual(self.call(card, 'stop_move')['state'], 'paused')
                    self.assertEqual(self.call('_control', 'resume')['state'], 'ready')

    def test_joint_limit_records_must_match_runtime_and_be_well_formed(self):
        original = fixture_joint_limits(self.ros)
        invalid = []
        for field, value in (('verification', ''), ('robot_version', 1),
                             ('runtime_urdf_sha256', 'wrong fingerprint')):
            policy = copy.deepcopy(original)
            policy[field] = value
            invalid.append(policy)
        for card, size in (('arm', 14), ('leg', 4)):
            for field, value in (('joint_names', list(reversed(original[card]['joint_names']))),
                                 ('lower_deg', [False] * size), ('lower_deg', [math.nan] * size),
                                 ('upper_deg', [math.inf] * size), ('upper_deg', [0.] * (size - 1)),
                                 ('lower_deg', [100.] * size)):
                policy = copy.deepcopy(original)
                policy[card][field] = value
                invalid.append(policy)
        for policy in invalid:
            self.bundle.sensors._joint_limit_config = policy
            card = 'leg' if policy['leg'] != original['leg'] else 'arm'
            size = 4 if card == 'leg' else 14
            with self.subTest(card=card, policy=policy):
                self.assertEqual(self.call(card, 'move_joints', positions=[0.] * size)['state'], 'error')
                self.assertEqual(self.ros.published, [])

    def test_joint_limits_can_be_commissioned_for_one_card_only(self):
        del self.bundle.sensors._joint_limit_config['leg']
        self.assertIn('not configured for leg', self.call('leg', 'move_joints', positions=[0.] * 4)['error'])
        self.assertEqual(self.call('arm', 'move_joints', positions=[0.] * 14)['state'], 'executing')

    def test_urdf_fingerprint_change_invalidates_joint_limits_even_with_same_names(self):
        self.ros.params['humanoid_description'] += '\n'
        self._refresh_identity_via_timer()
        self.ros.raw()
        self.assertTrue(self.bundle.sensors.snapshot('joints')['valid'])
        for card, size in (('arm', 14), ('leg', 4)):
            self.assertIn('fingerprint does not match', self.call(card, 'move_joints', positions=[0.] * size)['error'])
        self.assertEqual(self.ros.published, [])

    def test_limit_identity_change_aborts_inflight_joint_action(self):
        action = self.call('arm', 'move_joints', positions=[0.] * 14)
        self.ros.params['humanoid_description'] += '\n'
        self._refresh_identity_via_timer()
        self.ros.raw()
        self.assertTrue(wait_until(lambda: bool(self.callbacks)))
        self.assertEqual(self.callbacks[0][:3], (action['action_id'], 'arm', 'error'))
        self.assertIn('fingerprint', self.callbacks[0][3]['reason'])
        self.assertTrue(self.bundle.state.motion_blocked)

    def _refresh_identity_via_timer(self):
        hub = self.bundle.sensors
        hub._retry_at = time.monotonic() - 1  # Make the scheduled refresh due.
        previous = hub._thread
        hub.tick()
        self.assertIsNot(hub._thread, previous)
        hub._thread.join(1)
        self.assertFalse(hub._thread.is_alive())

    def test_periodic_robot_version_change_rejects_arm_and_leg_targets(self):
        self.ros.params['robot_version'] += 1
        self._refresh_identity_via_timer()
        self.ros.raw()
        for card, size in (('arm', 14), ('leg', 4)):
            self.assertIn('robot_version does not match',
                          self.call(card, 'move_joints', positions=[0.] * size)['error'])
        self.assertEqual(self.ros.published, [])

    def test_periodic_missing_urdf_aborts_inflight_joint_action(self):
        action = self.call('leg', 'move_joints', positions=[0.] * 4)
        del self.ros.params['humanoid_description']
        self._refresh_identity_via_timer()
        self.assertTrue(wait_until(lambda: bool(self.callbacks)))
        self.assertEqual(self.callbacks[0][:3], (action['action_id'], 'leg', 'error'))
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_stalled_periodic_identity_refresh_expires_and_aborts_motion(self):
        action = self.call('arm', 'move_joints', positions=[0.] * 14)
        hub = self.bundle.sensors
        entered, release = threading.Event(), threading.Event()
        original = self.ros.get_param
        def blocked(*args, **kwargs):
            entered.set()
            release.wait(2)
            return original(*args, **kwargs)
        with patch.object(self.ros, 'get_param', side_effect=blocked):
            hub._retry_at = 0
            hub.tick()
            try:
                self.assertTrue(entered.wait(1))
                # Simulate lease age while the normal timer's read is blocked.
                hub._identity_checked_at -= sensors_module.IDENTITY_LEASE_S + 1
                self.assertTrue(wait_until(lambda: bool(self.callbacks)))
                self.assertEqual(self.callbacks[0][:3], (action['action_id'], 'arm', 'error'))
                self.assertIn('identity verification is stale', self.callbacks[0][3]['reason'])
                self.assertTrue(self.bundle.state.motion_blocked)
            finally:
                release.set()
                hub._thread.join(1)

    def test_invalid_joint_target_does_not_cancel_another_active_channel(self):
        action = self.call('leg', 'move_joints', positions=[0.] * 4)
        result = self.call('arm', 'move_joints', positions=[10000.] * 14)
        self.assertEqual(result['state'], 'error')
        self.assertTrue(self.bundle.tracker.current('lower_body', action['action_id']))
        self.assertEqual(self.callbacks, [])
        self.assertEqual(len(self.ros.published), 1)

    def test_joint_commands_require_matching_runtime_model(self):
        model = self.ros.params['humanoid_description']
        for description in (None, model.replace(JOINT_NAMES[0], 'unexpected_joint')):
            for card, count in (('arm', 14), ('leg', 4)):
                with self.subTest(model='missing' if description is None else 'mismatched', card=card):
                    self.ros.params['humanoid_description'] = description
                    self.bundle.sensors.discover()
                    self.ros.raw()
                    self.assertFalse(self.bundle.sensors.snapshot('joints')['valid'])
                    result = self.call(card, 'move_joints', positions=[0.] * count)
                    self.assertIn('runtime joint model is not validated', result['error'])
                    self.assertEqual(self.ros.published, [])
                    self.assertEqual(self.callbacks, [])

    def test_runtime_model_loss_aborts_joint_action_without_completing(self):
        for mismatch in (False, True):
            with self.subTest(mismatch=mismatch):
                model = self.ros.params['humanoid_description']
                action = self.call('arm', 'move_joints', positions=[0.] * 14)
                self.assertEqual(action['state'], 'executing')
                self.ros.emit('/lb_arm_joint_reach_time', 0.)
                self.ros.params['humanoid_description'] = (
                    model.replace(JOINT_NAMES[0], 'unexpected_joint') if mismatch else None)
                self.bundle.sensors.discover()
                self.ros.raw()
                self.assertTrue(wait_until(lambda: any(c[0] == action['action_id'] for c in self.callbacks)))
                outcome = [c for c in self.callbacks if c[0] == action['action_id']]
                self.assertEqual(len(outcome), 1)
                self.assertEqual(outcome[0][2], 'error')
                self.assertIn('runtime joint model', outcome[0][3]['reason'])
                self.assertTrue(self.bundle.state.motion_blocked)
                self.ros.params['humanoid_description'] = model
                self.bundle.sensors.discover()
                self.ros.raw()
                self.assertEqual(self.call('_control', 'resume')['state'], 'ready')

    def test_reach_time_alone_cannot_complete_unconverged_joints(self):
        result = self.call('leg', 'move_joints', positions=[20.] * 4)
        self.ros.emit('/lb_leg_joint_reach_time', 0.)
        for _ in range(6):
            self.ros.raw()  # Fresh measurements show that the leg has not moved.
            time.sleep(0.05)
        self.assertEqual(self.callbacks, [])
        self.assertTrue(self.bundle.tracker.current('lower_body', result['action_id']))
        self.bundle.sensors._seen['joints'] = time.monotonic() - 1
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][2], 'error')
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_leg_measured_convergence_degrees_to_radians(self):
        result = self.call('leg', 'move_joints', positions=[10.] * 4)
        self.ros.emit('/lb_leg_joint_reach_time', 0.)
        self.ros.raw([math.radians(10)] * 4 + [0.] * 16)
        time.sleep(0.04)
        time.sleep(0.18)
        self.ros.raw([math.radians(10)] * 4 + [0.] * 16)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result['action_id'], 'leg', 'completed'))
        self.assertTrue(self.callbacks[0][3]['measured_convergence'])

    def _run_joint_measurement_trace(self, samples):
        """Run the real motion worker with independent poll/sample timestamps.

        Each entry is (poll elapsed, measurement elapsed, measured leg degrees).
        Stop polling at the end so a non-completing action can be inspected.
        """
        started = time.monotonic()
        clock = [started]
        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0], time=time.time)
        with patch.object(driver, 'time', fake_time):
            with patch.object(driver.threading, 'Thread') as thread:
                result = self.call('leg', 'move_joints', positions=[10.] * 4)
            self.assertEqual(result['state'], 'executing')
            thread.assert_called_once()
            worker = thread.call_args.kwargs['target']
            cancelled = self.bundle.tracker._active['lower_body'][2]
            clock[0] += 0.001
            self.ros.emit('/lb_leg_joint_reach_time', 0.)
            entries = iter(samples)
            current = [None]

            def poll(_timeout):
                entry = next(entries, None)
                if entry is None:
                    return True
                elapsed, measured_elapsed, degrees = entry
                clock[0] = started + elapsed
                current[0] = ([math.radians(degrees)] * 4 + [0.] * 16,
                              started + measured_elapsed, int(measured_elapsed * 1e9))
                return False

            with patch.object(cancelled, 'wait', side_effect=poll), patch.object(
                    self.bundle.sensors, 'joint_positions', side_effect=lambda: current[0]) as positions:
                worker()
            # A premature completion would consume fewer trace entries.
            self.assertEqual(positions.call_count, len(samples))
        self.bundle.tracker._callbacks.join()
        return result

    def test_unconverged_joint_target_times_out_without_vendor_estimate(self):
        with patch.object(self.bundle.state, 'require_motion'):
            result = self._run_joint_measurement_trace([(0.02, 0.02, 0.), (115.01, 115.01, 0.)])
        self.assertEqual(self.callbacks[0][:3], (result['action_id'], 'leg', 'error'))
        self.assertIn('115s', self.callbacks[0][3]['reason'])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_cached_joint_sample_does_not_accumulate_convergence(self):
        result = self._run_joint_measurement_trace([
            (0.02, 0.02, 10.), (0.10, 0.02, 10.),
            (0.24, 0.02, 10.), (0.32, 0.02, 10.),
        ])
        self.assertEqual(self.callbacks, [])
        self.assertTrue(self.bundle.tracker.current('lower_body', result['action_id']))

    def test_precommand_joint_sample_does_not_start_convergence(self):
        self._run_joint_measurement_trace([
            (0.02, -0.01, 10.), (0.24, -0.01, 10.), (0.30, 0.30, 10.),
        ])
        self.assertEqual(self.callbacks, [])

    def test_distinct_joint_samples_must_span_convergence_window(self):
        self._run_joint_measurement_trace([
            (0.02, 0.02, 10.), (0.17, 0.17, 10.), (0.32, 0.17, 10.),
        ])
        self.assertEqual(self.callbacks, [])

    def test_out_of_tolerance_joint_sample_restarts_convergence(self):
        result = self._run_joint_measurement_trace([
            (0.02, 0.02, 10.), (0.18, 0.18, 0.), (0.24, 0.24, 10.),
            (0.30, 0.30, 10.), (0.46, 0.46, 10.),
        ])
        self.assertEqual(len(self.callbacks), 1)
        self.assertEqual(self.callbacks[0][:3], (result['action_id'], 'leg', 'completed'))
        self.assertTrue(self.callbacks[0][3]['measured_convergence'])

    def _run_stamped_joint_trace(self, samples, cycle_profile=False):
        """Exercise actual ROS callbacks + motion worker, with separate clocks.

        Samples are (receipt elapsed, sensor capture elapsed, leg degrees).
        Header seq changes even when the sensor capture itself is replayed.
        """
        if cycle_profile:
            self.bundle.close()
            self.ros = FakeRos1()
            self.ros.params['robot_version'] = 63
            self.bundle = driver.KuavoBundle({'sensors': {'joint_limits': fixture_joint_limits(self.ros)}},
                                             'test_robot', self.ros, self.node)
            self.bundle.sensors.discover()
            ns = time.time_ns()
            self.ros.raw(sensor_time={'secs': (ns - 824_000_000) // 1_000_000_000,
                                     'nsecs': (ns - 824_000_000) % 1_000_000_000},
                         header={'stamp': {'secs': ns // 1_000_000_000, 'nsecs': ns % 1_000_000_000}})
            self.ros.emit('/enable_control_state', True)
            self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 3)
            self.bundle.state.verify_enable()
        started, wall = time.monotonic(), time.time() + .001
        clock = [started]
        fake_time = types.SimpleNamespace(monotonic=lambda: clock[0],
                                          time=lambda: wall + clock[0] - started)
        with patch.object(driver, 'time', fake_time), patch.object(sensors_module, 'time', fake_time):
            with patch.object(driver.threading, 'Thread') as thread:
                result = self.call('leg', 'move_joints', positions=[10.] * 4)
            self.assertEqual(result['state'], 'executing')
            worker = thread.call_args.kwargs['target']
            cancelled = self.bundle.tracker._active['lower_body'][2]
            clock[0] = started + .001
            self.ros.emit('/lb_leg_joint_reach_time', 0.)
            entries = iter(enumerate(samples))
            def poll(_timeout):
                entry = next(entries, None)
                if entry is None:
                    return True
                seq, sample = entry
                received, captured, degrees = sample[:3]
                clock[0] = started + received
                ns = int((wall + captured - (.824 if cycle_profile else 0)) * 1e9)
                header = {'seq': seq}
                if cycle_profile:
                    header_ns = int((wall + (sample[3] if len(sample) > 3 else received)) * 1e9)
                    header['stamp'] = {'secs': header_ns // 1_000_000_000,
                                       'nsecs': header_ns % 1_000_000_000}
                self.ros.raw([math.radians(degrees)] * 4 + [0.] * 16,
                             sensor_time={'secs': ns // 1_000_000_000, 'nsecs': ns % 1_000_000_000},
                             header=header)
                return False
            with patch.object(cancelled, 'wait', side_effect=poll):
                worker()
        self.bundle.tracker._callbacks.join()
        return result

    def test_v63_cycle_clock_and_fresh_headers_complete_measured_joint_action(self):
        result = self._run_stamped_joint_trace([
            (.02, .02, 10.), (.12, .12, 10.), (.24, .24, 10.)], cycle_profile=True)
        self.assertEqual([c[:3] for c in self.callbacks], [(result['action_id'], 'leg', 'completed')])

    def test_v63_repeated_cycle_with_new_headers_expires_instead_of_completing(self):
        result = self._run_stamped_joint_trace([
            (.02, .02, 10.), (.12, .02, 10.), (.24, .02, 10.), (.6, .02, 10.)], cycle_profile=True)
        self.assertEqual([c[:3] for c in self.callbacks], [(result['action_id'], 'leg', 'error')])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_v63_precommand_header_cannot_start_convergence(self):
        result = self._run_stamped_joint_trace([
            (.02, .02, 10., -.01), (.12, .12, 10., .09), (.24, .24, 10., .21)], cycle_profile=True)
        self.assertFalse(self.callbacks)
        self.assertTrue(self.bundle.tracker.current('lower_body', result['action_id']))

    def test_replayed_stamped_sensor_frame_cannot_complete_joints(self):
        result = self._run_stamped_joint_trace([
            (.02, .02, 10.), (.10, .02, 10.), (.24, .02, 10.), (.32, .02, 10.)])
        self.assertEqual(self.callbacks, [])
        self.assertTrue(self.bundle.tracker.current('lower_body', result['action_id']))

    def test_replayed_stamped_sensor_frame_eventually_expires_and_pauses(self):
        result = self._run_stamped_joint_trace([
            (.02, .02, 10.), (.30, .02, 10.), (.60, .02, 10.)])
        self.assertEqual([c[:3] for c in self.callbacks], [(result['action_id'], 'leg', 'error')])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_fresh_capture_timestamps_can_complete_joints(self):
        result = self._run_stamped_joint_trace([
            (.02, .02, 10.), (.13, .13, 10.), (.25, .25, 10.)])
        self.assertEqual([c[:3] for c in self.callbacks], [(result['action_id'], 'leg', 'completed')])

    def test_capture_span_not_receipt_span_controls_convergence(self):
        result = self._run_stamped_joint_trace([(.02, .02, 10.), (.30, .10, 10.)])
        self.assertEqual(self.callbacks, [])
        self.assertTrue(self.bundle.tracker.current('lower_body', result['action_id']))

    def test_future_clock_tolerance_cannot_inflate_capture_convergence_span(self):
        result = self._run_stamped_joint_trace([(.02, .06, 10.), (.23, .23, 10.)])
        self.assertEqual(self.callbacks, [])  # 0.21s receipt span, only 0.17s capture span.
        self.assertTrue(self.bundle.tracker.current('lower_body', result['action_id']))

    def test_delayed_precommand_capture_cannot_start_convergence(self):
        self._run_stamped_joint_trace([(.02, -.01, 10.), (.24, -.01, 10.), (.30, .30, 10.)])
        self.assertEqual(self.callbacks, [])

    def test_stop_blocks_old_velocity_thread_and_cancels_acp_once(self):
        result = self.call("base_drive", "move", vx=0.1, duration=1)
        stopped = self.call("base_drive", "stop_move")
        self.assertEqual(stopped["state"], "paused")
        boundary = len(self.ros.published)
        time.sleep(0.1)
        self.assertEqual(len(self.ros.published), boundary)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual([c[:3] for c in self.callbacks], [(result["action_id"], "base_drive", "cancelled")])
        fresh = self.call('base_drive', 'move', duration=.1)
        self.assertEqual(fresh['state'], 'executing')
        self.assertNotEqual(fresh['action_id'], result['action_id'])
        self.assertTrue(fresh['control_preparation']['resumed'])

    def _partial_initial_send(self, card, action, topic, **args):
        original = self.ros.publish
        failed = [False]
        def publish(name, message_type, message):
            original(name, message_type, message)
            if name == topic and not failed[0]:
                failed[0] = True
                raise OSError('send failed after simulated delivery')
        with patch.object(self.ros, 'publish', side_effect=publish):
            result = self.call(card, action, **args)
        self.assertEqual(result['state'], 'error')
        self.assertEqual(result['command_delivery'], 'uncertain')
        self.assertFalse(result['physical_stop_confirmed'])
        self.assertTrue(self.bundle.state.motion_blocked)
        self.assertTrue(self.bundle.state.snapshot()['motion_send_uncertain'])
        self.assertFalse(self.bundle.tracker._active)
        self.assertEqual(self.ros.sent_services[-1][2], {'data': False})
        self.assertTrue(wait_until(lambda: bool(self.callbacks)))
        self.assertEqual([c[:3] for c in self.callbacks], [(result['action_id'], card, 'error')])
        # A False feedback/accepted pause is not proof the old target was cancelled.
        self.assertFalse(self.bundle.state.confirmed_enable())
        self.assertIn('delivery is uncertain', self.call('_control', 'resume')['error'])
        self.assertEqual(self.call('base_drive', 'move', duration=.1)['state'], 'error')
        count = len(self.ros.sent_services)
        self.bundle.close()
        self.assertEqual(len(self.ros.sent_services), count + 1)
        self.assertEqual(self.ros.sent_services[-1][2], {'data': False})
        self.assertEqual(len(self.callbacks), 1)

    def test_partial_initial_position_send_fails_closed_and_shutdown_retries_stop(self):
        self.bundle.sensors.odom_pose = lambda: ((0., 0., 0.), time.time(), 'odom')
        self._partial_initial_send('base_drive', 'move_relative', '/cmd_vel', x=.01, y=0., yaw=0.)

    def test_partial_initial_velocity_send_fails_closed(self):
        self._partial_initial_send('base_drive', 'move', '/cmd_vel', vx=.01, duration=.2)

    def test_partial_initial_joint_send_fails_closed(self):
        self._partial_initial_send('arm', 'move_joints', '/kuavo_arm_traj', positions=[0.] * 14)

    def test_initial_send_disconnect_remains_uncertain_after_reconnect(self):
        def disconnect(*_args):
            self.ros.connection = (False, 1)
            raise ConnectionError('delivery unknown')
        with patch.object(self.ros, 'publish', side_effect=disconnect):
            result = self.call('base_drive', 'move', duration=.2)
        self.assertEqual(result['command_delivery'], 'uncertain')
        self.assertFalse(result['physical_stop_confirmed'])
        self.ros.connection = (True, 2)
        self.bundle.sensors.discover()
        self.ros.raw()
        self.ros.emit('/enable_control_state', True)
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 3)
        self.bundle.state.verify_enable()
        self.assertIn('delivery is uncertain', self.call('_control', 'resume')['error'])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_initial_send_failure_pause_wait_does_not_hold_publication_lock(self):
        original_publish, original_wait = self.ros.publish, self.ros.wait_service
        entered, release = threading.Event(), threading.Event()
        first_publish, first_wait, result = [True], [True], []
        def publish(*args):
            original_publish(*args)
            if first_publish[0]:
                first_publish[0] = False
                raise OSError('delivery unknown')
        def wait(pending, timeout, cancelled=None):
            if first_wait[0]:
                first_wait[0] = False
                entered.set()
                release.wait(2)
            return original_wait(pending, timeout, cancelled)
        with patch.object(self.ros, 'publish', side_effect=publish), patch.object(self.ros, 'wait_service', side_effect=wait):
            worker = threading.Thread(target=lambda: result.append(self.call('base_drive', 'move', duration=.2)))
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                stopped = self.call('head', 'stop_move')
                self.assertTrue(worker.is_alive())
                self.assertNotIn('busy', stopped['error'])
                self.assertFalse(stopped['physical_stop_confirmed'])
                self.assertEqual([s[2] for s in self.ros.sent_services], [{'data': False}, {'data': False}])
            finally:
                release.set()
                worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(result[0]['command_delivery'], 'uncertain')
        self.assertTrue(wait_until(lambda: bool(self.callbacks)))
        self.assertEqual(len(self.callbacks), 1)

    def test_input_validation_failure_does_not_mark_delivery_uncertain(self):
        self.assertEqual(self.call('base_drive', 'move', duration=-2)['state'], 'error')
        self.assertFalse(self.bundle.state.motion_send_uncertain)
        self.assertFalse(self.bundle.state.motion_blocked)
        self.assertEqual(self.ros.sent_services, [])

    def test_pause_service_acceptance_without_feedback_not_success(self):
        self.ros.confirm = False
        with patch.object(self.bundle, "wait_state", side_effect=TimeoutError("not confirmed")):
            result = self.call("_control", "pause")
        self.assertEqual(result["state"], "error")
        self.assertFalse(result["physical_stop_confirmed"])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_pause_rejection_and_resume_feedback(self):
        self.ros.service_accepts = False
        self.assertEqual(self.call("_control", "pause")["state"], "error")
        self.ros.service_accepts = True
        self.assertEqual(self.call("_control", "pause")["state"], "paused")
        self.assertEqual(self.call("_control", "resume")["state"], "ready")
        self.assertFalse(self.bundle.state.motion_blocked)
        self.assertEqual(self.ros.services[-1][:3], ("/enable_control", "std_srvs/SetBool", {"data": True}))

    def test_mode_switch_can_leave_no_control_and_is_confirmed(self):
        self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 0)
        result = self.call("_control", "set_mode", mode=3)
        self.assertEqual(result["state"], "completed")
        self.assertEqual(self.ros.services[-1][:3], ("/mobile_manipulator_mpc_control", "kuavo_msgs/changeTorsoCtrlMode", {"control_mode": 3}))

    def test_feedback_loss_causes_error_not_false_completed(self):
        result = self.call("arm", "move_joints", positions=[0] * 14)
        self.ros.connection = (False, 1)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result["action_id"], "arm", "error"))
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_named_arm_angle_preserves_other_joints_and_converts_radians(self):
        q = [v / 100 for v in range(20)]
        self.ros.raw(q=q)
        result = self.call('arm', 'set', left_joint_3_rad=.25)
        self.assertEqual(result['state'], 'executing')
        command = self.ros.published[-1][2]['position']
        expected = q[4:18]
        expected[2] = .25
        for value, target in zip(command, expected):
            self.assertAlmostEqual(value, math.degrees(target))

    def test_named_joint_targets_refresh_unset_joints_after_resume(self):
        self.call('base_drive', 'stop_move')
        original = self.bundle.confirm_control
        def confirm(*args, **kwargs):
            result = original(*args, **kwargs)
            if args[1] == '/enable_control' and args[3].get('data'):
                self.ros.raw(q=[.1] * 20)
            return result
        with patch.object(self.bundle, 'confirm_control', side_effect=confirm):
            result = self.call('arm', 'set', right_joint_7_rad=.2)
        self.assertEqual(result['state'], 'executing')
        positions = self.ros.published[-1][2]['position']
        self.assertAlmostEqual(positions[0], math.degrees(.1))
        self.assertAlmostEqual(positions[-1], -math.degrees(.2))

    def test_arm_set_blank_fields_hold_and_zero_is_a_target(self):
        self.ros.raw(q=[.1] * 20)
        result = self.call('arm', 'set', left_joint_1_rad=0., left_joint_2_rad=None,
                           left_joint_3_rad='', right_joint_7_rad='   ')
        self.assertEqual(result['state'], 'executing')
        positions = self.ros.published[-1][2]['position']
        self.assertEqual(positions[0], 0.)
        for value in positions[1:]:
            self.assertAlmostEqual(value, math.degrees(.1))

    def test_arm_set_rejects_all_blank_or_invalid_fields_without_publishing(self):
        for args in ({'left_joint_1_rad': None, 'right_joint_7_rad': ' '},
                     {'left_joint_1_rad': 'ten'}, {'left_joint_1_rad': True}):
            self.assertEqual(self.call('arm', 'set', **args)['state'], 'error')
        for action in ('left_joint_1', 'right_joint_7'):
            self.assertEqual(self.call('arm', action)['state'], 'error')
        self.assertFalse(self.ros.published)

    def test_named_leg_and_waist_targets_use_correct_lower_body_indices(self):
        self.ros.raw(q=[.1] * 20)
        result = self.call('leg', 'set', waist_yaw_deg=math.degrees(.2))
        self.assertEqual(result['state'], 'executing')
        topic, _, message = self.ros.published[-1]
        self.assertEqual(topic, '/lb_leg_traj')
        self.assertAlmostEqual(message['position'][0], math.degrees(.1))
        self.assertAlmostEqual(message['position'][3], math.degrees(.2))
        conflict = self.call('leg', 'set', knee_deg=5.)
        self.assertIn('busy', conflict['error'])

    def test_leg_set_accepts_degrees_and_preserves_omitted_angles(self):
        self.ros.raw(q=[.1] * 20)
        result = self.call('leg', 'set', knee_deg=10., waist_pitch_deg=20.)
        self.assertEqual(result['state'], 'executing')
        positions = self.ros.published[-1][2]['position']
        self.assertAlmostEqual(positions[0], 10.)
        self.assertAlmostEqual(positions[1], math.degrees(.1))
        self.assertAlmostEqual(positions[2], 20.)

    def test_new_named_joint_actions_cannot_bypass_commissioning_limits(self):
        self.bundle.sensors._joint_limit_config = None
        for card, action, args in [('arm', 'set', {'left_joint_1_rad': .1}),
                                   ('leg', 'set', {'knee_deg': 1.}),
                                   ('leg', 'zero', {})]:
            result = self.call(card, action, **args)
            self.assertIn('not configured', result['error'])
        self.assertFalse(self.ros.published)

    def test_new_joint_fields_reject_missing_or_out_of_range_angles(self):
        for card, action, args in [('arm', 'set', {}), ('leg', 'set', {}),
                                   ('arm', 'left_joint_1', {}),
                                   ('arm', 'set', {'left_joint_1_rad': 100.}),
                                   ('leg', 'set', {'waist_pitch_deg': float('nan')})]:
            self.assertEqual(self.call(card, action, **args)['state'], 'error')
        self.assertFalse(self.ros.published)

    def test_head_rejects_velocity_api_bad_angles_and_missing_feedback(self):
        for action, args in [('move', {'yaw': 1., 'duration': 1.}),
                             ('move_pos', {'yaw': 81., 'pitch': 0.}),
                             ('move_pos', {'yaw': 0., 'pitch': -26.}),
                             ('move_pos', {}),
                             ('move_pos', {'yaw': True, 'pitch': 0.})]:
            self.assertEqual(self.call('head', action, **args)['state'], 'error')
        self.bundle.sensors._seen['joints'] = time.monotonic() - 1.
        self.assertIn('stale', self.call('head', 'zero')['error'])
        self.assertFalse(self.ros.published)

    def test_head_exact_completion_requires_new_measurements_and_reports_error(self):
        result = self.call('head', 'zero')
        self.assertEqual(result['state'], 'executing')
        self.assertEqual(result['completion_basis'], 'measured_head_or_stable_residual')
        self.assertTrue(self.bundle.tracker.current('head', result['action_id']))
        end = time.monotonic()+1.
        while not self.callbacks and time.monotonic()<end:
            self.ros.raw()
            time.sleep(.05)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result['action_id'], 'head', 'completed'))
        self.assertTrue(self.callbacks[0][3]['measured_convergence'])
        self.assertFalse(self.callbacks[0][3]['approximate'])
        self.assertEqual(self.callbacks[0][3]['error'], {'yaw_deg': 0., 'pitch_deg': 0.})
        self.assertFalse(self.bundle.tracker._active)

    def test_head_stable_offset_completes_approximately_with_card_pitch_direction(self):
        result = self.call('head', 'move_pos', yaw=10., pitch=4.)
        end = time.monotonic()+1.6
        while not self.callbacks and time.monotonic()<end:
            self.ros.raw(q=[0.]*18+[math.radians(7), math.radians(-1)])
            time.sleep(.05)
        self.assertTrue(wait_until(lambda: self.callbacks))
        event = self.callbacks[0]
        self.assertEqual(event[:3], (result['action_id'], 'head', 'completed'))
        self.assertTrue(event[3]['approximate'])
        self.assertFalse(event[3]['measured_convergence'])
        self.assertEqual(event[3]['error'], {'yaw_deg': 3., 'pitch_deg': 3.})
        self.assertAlmostEqual(event[3]['measured']['pitch_deg'], 1.)
        self.assertIn('approximately',event[3]['message'])

    def test_head_large_stable_error_does_not_claim_success(self):
        result = self.call('head', 'move_pos', yaw=20.)
        end = time.monotonic()+1.25
        while time.monotonic()<end:
            self.ros.raw()
            time.sleep(.05)
        self.assertFalse(self.callbacks)
        self.assertTrue(self.bundle.tracker.current('head',result['action_id']))

    def test_head_missing_post_command_feedback_errors_instead_of_success(self):
        result = self.call('head', 'move_pos', yaw=4.)
        self.assertTrue(wait_until(lambda:self.callbacks,1.))
        self.assertEqual(self.callbacks[0][:3], (result['action_id'], 'head', 'error'))
        self.assertIn('stale',self.callbacks[0][3]['reason'])

    def test_head_stop_during_acp_handoff_cancels_exactly_once(self):
        result = self.call('head', 'move_pos', yaw=5.)
        self.assertEqual(result['state'], 'executing')
        self.call('head', 'stop_move')
        self.assertTrue(wait_until(lambda: self.callbacks))
        time.sleep(.25)
        callbacks = [c for c in self.callbacks if c[0] == result['action_id']]
        self.assertEqual(len(callbacks), 1)
        self.assertEqual(callbacks[0][2], 'cancelled')

    def test_head_uses_5w_limits_instead_of_generic_sdk_limits(self):
        tool = next(p.tool() for p in self.bundle.plugins if p.PREFIX == 'head')
        yaw = tool['inputSchema']['properties']['yaw']
        self.assertEqual((yaw['minimum'], yaw['maximum']), (-30, 30))
        for value in (-30.01, 30.01, 80.):
            self.assertEqual(self.call('head', 'move_pos', yaw=value)['state'], 'error')
        self.ros.raw(q=[0.] * 18 + [math.radians(31), 0.])
        self.assertEqual(self.call('head', 'move_pos', pitch=1.)['state'], 'error')
        self.assertFalse(self.ros.published)

    def test_head_dispatch_returns_without_waiting_for_stable_feedback(self):
        self.ros.raw(q=[0.] * 18 + [math.radians(6.63), 0.])
        before = time.monotonic()
        result = self.call('head', 'move_pos', yaw=5.)
        self.assertLess(time.monotonic()-before, .5)
        self.assertEqual(result['state'], 'executing')
        self.assertFalse(result['measured_convergence'])
        self.assertFalse(self.bundle.state.motion_blocked)

    def test_head_yaw_only_holds_measured_pitch(self):
        self.ros.raw(q=[0.] * 18 + [math.radians(3), math.radians(7)])
        result = self.call('head', 'move_pos', yaw=10.)
        self.assertEqual(result['state'], 'executing')
        self.assertAlmostEqual(result['pitch'], -7.)
        self.assertEqual(self.ros.published[-1][0], '/robot_head_motion_data')
        self.assertAlmostEqual(self.ros.published[-1][2]['joint_data'][1], 7.)

    def test_head_pitch_only_holds_measured_yaw_and_blank_is_omitted(self):
        self.ros.raw(q=[0.] * 18 + [math.radians(9), math.radians(7)])
        result = self.call('head', 'move_pos', yaw=' ', pitch=0.)
        self.assertEqual(result['state'], 'executing')
        self.assertAlmostEqual(result['yaw'], 9.)
        self.assertEqual(result['pitch'], 0.)

    def test_head_all_blank_rejected_and_axes_not_required_in_schema(self):
        self.assertEqual(self.call('head', 'move_pos', yaw=None, pitch='')['state'], 'error')
        tool = next(p.tool() for p in self.bundle.plugins if p.PREFIX == 'head')
        self.assertEqual(tool['inputSchema']['x-action-params']['move_pos']['required'], [])
        self.assertFalse(self.ros.published)

    def test_head_refreshes_omitted_axis_after_resume(self):
        self.call('base_drive', 'stop_move')
        original = self.bundle.confirm_control
        def confirm(*args, **kwargs):
            result = original(*args, **kwargs)
            if args[1] == '/enable_control' and args[3].get('data'):
                self.ros.raw(q=[0.] * 18 + [0., math.radians(5)])
            return result
        with patch.object(self.bundle, 'confirm_control', side_effect=confirm):
            result = self.call('head', 'move_pos', yaw=10., pitch=None)
        self.assertEqual(result['state'], 'executing')
        self.assertAlmostEqual(result['pitch'], -5.)

    def test_head_pitch_direction_and_acp_schema(self):
        result = self.call('head', 'move_pos', yaw=12, pitch=4)
        self.assertEqual(result['state'], 'executing')
        self.assertEqual(self.ros.published[1], ('/robot_head_motion_data', 'kuavo_msgs/robotHeadMotionData',
                                                {'joint_data': [12, -4]}))
        self.assertEqual(result['pitch'], 4)
        self.assertIn('action_id', result)
        self.assertFalse(self.callbacks)
        tool = next(t for t in self.bundle.get_all_tools() if t['name'] == 'head')
        self.assertEqual(tool['inputSchema']['x-completion']['actions'], ['move_pos', 'zero'])

    def test_slow_acp_delivery_does_not_block_stop(self):
        entered, release = threading.Event(), threading.Event()
        def slow_callback(*args):
            entered.set()
            release.wait(1)
        with patch.object(driver, "_acp_complete", side_effect=slow_callback):
            self.call("arm", "move_joints", positions=[0] * 14)
            self.ros.emit("/lb_arm_joint_reach_time", 0)
            self.ros.raw()
            time.sleep(0.05)
            time.sleep(0.18)
            self.ros.raw()
            self.assertTrue(entered.wait(1))
            try:
                result = self.call("base_drive", "stop_move")
                self.assertEqual(result["state"], "paused")
                self.assertFalse(release.is_set())
            finally:
                release.set()
                self.bundle.tracker._callbacks.join()

    def test_excessive_cartesian_reach_time_aborts_and_pauses(self):
        result = self.call("arm", "set_pose", frame="world", left_position=[0, .4, 0],
                           right_position=[0, -.4, 0], left_quaternion=[0, 0, 0, 1],
                           right_quaternion=[0, 0, 0, 1])
        self.ros.emit("/lb_arm_ee_reach_time", 111)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result["action_id"], "arm", "error"))
        self.assertIn("110s", self.callbacks[0][3]["reason"])
        self.assertFalse(self.bundle.state.read("control_enabled")[0])

    def test_bad_end_effector_state_is_dropped(self):
        self.ros.emit("/humanoid_wheel/eePoses", [0] * 11)
        self.assertIsNone(self.bundle.state.read("ee_world_poses")[0])
        self.ros.emit("/humanoid_wheel/eePoses", list(range(12)))
        self.assertEqual(self.bundle.state.read("ee_world_poses")[0], list(range(12)))

    def test_dispatch_does_not_mutate_caller_arguments(self):
        args = {"action": "info"}
        self.bundle.dispatch("state", args)
        self.assertEqual(args, {"action": "info"})

    def test_shutdown_pauses_and_rejects_new_motion(self):
        self.call("base_drive", "move", duration=1)
        self.bundle.close()
        self.assertEqual(self.ros.services[-1][0], "/enable_control")
        self.assertEqual(self.call("base_drive", "move", duration=1)["state"], "error")
        # close is idempotent, including the callback worker.
        self.assertTrue(self.bundle._closing)


class CardRevisionTests(unittest.TestCase):
    setUp = DriverContractTests.setUp
    tearDown = DriverContractTests.tearDown
    call = DriverContractTests.call

    def vendor_model(self):
        sensors = self.bundle.sensors
        sensors._joint_limit_config = None
        sensors._arm_limit_source = sensors._leg_limit_source = 'vendor_runtime'
        self.ros.params['humanoid_description'] = (ROOT / 'resource/kuavo5w_model.urdf').read_text()
        self.ros.params['kuavo_configuration'] = {
            'NUM_JOINT': 20, 'NUM_ARM_JOINT': 14, 'NUM_HEAD_JOINT': 2,
            'min_joint_position_limits': [-5.5, -161, -25, -190]+[-180.]*16,
            'max_joint_position_limits': [83, 5.5, 181, 190]+[180.]*16}
        sensors.discover()
        self.ros.raw()

    def test_arm_six_directions_change_only_supplied_card_commands(self):
        values = {field: .1 for _, field, _ in driver.ArmPlugin.JOINTS.values()}
        result = self.call('arm', 'set', **values)
        self.assertEqual(result['state'], 'executing')
        actual = self.ros.published[-1][2]['position']
        for i, value in enumerate(actual):
            self.assertAlmostEqual(value, math.degrees(-.1 if i in (0,3,6,7,10,13) else .1))
        self.assertEqual(self.bundle.sensors.joint_positions()[0], [0.]*20)

    def test_arm_zero_and_schema_transformed_asymmetric_limits(self):
        policy = self.bundle.sensors._joint_limit_config['arm']
        policy['lower_deg'][0], policy['upper_deg'][0] = -20., 10.
        tool = next(t for t in self.bundle.get_all_tools() if t['name'] == 'arm')
        field = tool['inputSchema']['properties']['left_joint_1_rad']
        self.assertAlmostEqual(field['minimum'], math.radians(-10))
        self.assertAlmostEqual(field['maximum'], math.radians(20))
        self.assertIn('范围', field['description'])
        result = self.call('arm', 'zero')
        self.assertEqual(result['state'], 'executing')
        self.assertEqual(self.ros.published[-1][2]['position'], [0.]*14)

    def test_leg_uses_live_intersection_and_invalid_refresh_revokes_it(self):
        self.vendor_model()
        bounds = self.bundle.sensors.joint_bounds('leg')
        for actual, expected in zip(bounds, [(0,83),(-161,0),(-9,171),(-180,180)]):
            for a,b in zip(actual,expected): self.assertAlmostEqual(math.degrees(a), b, places=5)
        self.assertEqual(self.call('leg', 'set', knee_deg=1.)['state'], 'executing')
        self.ros.params['kuavo_configuration'] = None
        self.bundle.sensors._retry_at = 0
        self.bundle.sensors.tick()
        self.bundle.sensors._thread.join(1)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][2], 'error')

    def test_leg_field_limits_are_live_and_front_loaded(self):
        self.vendor_model()
        schema = next(t for t in self.bundle.get_all_tools() if t['name']=='leg')
        bounds = self.bundle.sensors.joint_bounds('leg')
        for field,(index,_) in driver.LowerBodyPlugin.FIELDS.items():
            entry = schema['inputSchema']['properties'][field]
            self.assertTrue(entry['description'].startswith('['))
            self.assertAlmostEqual(entry['minimum'], math.degrees(bounds[index][0]))
            self.assertAlmostEqual(entry['maximum'], math.degrees(bounds[index][1]))

    def test_height_ik_preserves_x_pitch_yaw_including_fixed_base_transform(self):
        self.vendor_model()
        q = [math.radians(1), math.radians(-2), math.radians(1), .2]
        self.ros.raw(q=q+[0.]*16)
        target, pose = self.bundle.sensors.height_target(1.02, with_pose=True)
        def fk(q):
            a,b,c = q[:3]
            return (.093-.46521*math.cos(a)+.18325*math.sin(a)
                    +.48296*math.cos(a+b)+.12941*math.sin(a+b)+.25*math.sin(a+b+c),
                    .40635+.46521*math.sin(a)+.18325*math.cos(a)
                    -.48296*math.sin(a+b)+.12941*math.cos(a+b)+.25*math.cos(a+b+c))
        self.assertAlmostEqual(fk(q)[0], fk(target)[0])
        self.assertAlmostEqual(fk(target)[1], 1.02)
        self.assertAlmostEqual(sum(q[:3]),sum(target[:3]))
        self.assertAlmostEqual(target[3], q[3])
        result = self.call('leg', 'set_height', height=1.02)
        self.assertEqual(result['state'], 'executing')
        self.assertEqual(self.ros.published[-1][0], '/cmd_lb_torso_pose')
        self.assertAlmostEqual(self.ros.published[-1][2]['linear']['z'], 1.02)
        self.assertAlmostEqual(self.ros.published[-1][2]['linear']['x'], fk(q)[0])

    def test_height_query_reports_actual_origin_and_pose_dependent_range_without_motion(self):
        self.vendor_model()
        status = self.call('leg', 'get_height')
        self.assertEqual(status['state'], 'ready')
        self.assertEqual(status['frame_id'], 'base_link')
        self.assertEqual(status['target_frame'], 'waist_yaw_link')
        self.assertAlmostEqual(status['height_m'], .96901)
        intervals = status['reachable_height_intervals_m']
        self.assertGreater(intervals[-1][1], 1.06)
        self.assertGreater(intervals[0][0], .74)
        error = self.call('leg', 'set_height', height=.8)
        self.assertEqual(error['state'], 'error')
        self.assertIn('current=', error['error'])
        self.assertIn('reachable_intervals_m=', error['error'])
        self.assertFalse(self.ros.published)
        self.assertFalse(self.ros.sent_services)
        self.ros.raw(q=[.2,-.4,.2,0.]+[0.]*16)
        self.assertNotEqual(self.call('leg', 'get_height')['reachable_height_intervals_m'], intervals)

    def test_height_head_top_uses_live_envelope_and_current_pose(self):
        self.vendor_model()
        model = self.ros.params['humanoid_description'].replace(
            '<link name="zhead_1_link">', '<link name="zhead_1_link"><collision>'
            '<origin xyz="0.03 0 0.09"/><geometry><sphere radius="0.1"/></geometry></collision>')
        self.ros.params['humanoid_description'] = model
        self.bundle.sensors.discover();self.ros.raw()
        status = self.call('leg', 'get_height')
        self.assertAlmostEqual(status['head_top_height_m'], .96901+.43935+.09+.1)
        self.assertTrue(status['head_top_is_estimate'])
        self.assertEqual(status['head_top_frame_id'], 'base_link')
        result = self.call('leg', 'set_height', height=1.02)
        self.assertEqual(result['state'], 'executing')
        self.assertAlmostEqual(result['head_top_height_m'], status['head_top_height_m'])
        self.assertAlmostEqual(result['target_head_top_height_m'], 1.02+.43935+.19)
        self.assertEqual(result['height_sample_phase'], 'before_motion')
        self.ros.raw(q=[.1,-.2,.2,.3]+[0.]*14+[.4,0.])
        tilted = self.call('leg', 'get_height')
        self.assertAlmostEqual(tilted['head_top_height_m'], tilted['height_m']
            -math.sin(.1)*math.cos(.7)*.03+math.cos(.1)*(.43935+.09)+.1)

    def test_height_missing_head_geometry_returns_unknown_not_nominal_height(self):
        self.vendor_model()
        status = self.call('leg', 'get_height')
        self.assertIsNone(status['head_top_height_m'])
        self.assertIn('unavailable', status['head_top_reason'])

    def test_height_interval_endpoints_and_interior_match_fk_and_joint_limits(self):
        self.vendor_model()
        for q in ([0.,0.,0.,0.], [.2,-.4,.2,.1], [.7,-1.2,.5,0.]):
            self.ros.raw(q=q+[0.]*16)
            status = self.bundle.sensors.height_state()
            bounds = self.bundle.sensors.joint_bounds('leg')
            for lo, hi in status['reachable_height_intervals_m']:
                for z in (lo, (lo+hi)/2, hi):
                    target = self.bundle.sensors.height_target(z)
                    a,b,c = target[:3]
                    actual_z = (.40635+.46521*math.sin(a)+.18325*math.cos(a)
                                -.48296*math.sin(a+b)+.12941*math.cos(a+b)+.25*math.cos(a+b+c))
                    actual_x = (.093-.46521*math.cos(a)+.18325*math.sin(a)
                                +.48296*math.cos(a+b)+.12941*math.sin(a+b)+.25*math.sin(a+b+c))
                    self.assertAlmostEqual(actual_z, z, places=7)
                    self.assertAlmostEqual(actual_x, status['x_m'], places=7)
                    self.assertAlmostEqual(sum(target[:3]), sum(q[:3]), places=7)
                    self.assertTrue(all(low <= v <= high for v,(low,high) in zip(target,bounds)))

    def test_height_rejects_unreachable_or_unknown_geometry_without_enabling(self):
        self.vendor_model()
        self.assertEqual(self.call('leg', 'set_height', height=.74)['state'], 'error')
        self.assertFalse(self.ros.published)
        self.assertFalse(self.ros.sent_services)
        self.ros.params['humanoid_description'] = self.ros.params['humanoid_description'].replace(
            'xyz="-0.46521 0 0.18325"', 'xyz="-0.46521 0.01 0.18325"')
        self.bundle.sensors.discover();self.ros.raw()
        self.assertIn('unsupported', self.call('leg', 'set_height', height=1.)['error'])

    def test_continuous_move_keeps_heartbeat_until_stop_and_angles_are_degrees(self):
        result = self.call('base_drive', 'move', vx=.1, vyaw=10., duration=-1)
        self.assertEqual(result['state'], 'running')
        self.assertNotIn('expected_duration_s', result)
        self.assertNotIn('action_id', result)
        self.assertTrue(result['continuous'])
        time.sleep(.15)
        self.assertGreaterEqual(len(self.ros.published), 2)
        self.assertAlmostEqual(self.ros.published[-1][2]['angular']['z'], math.radians(10.))
        self.assertFalse(self.callbacks)
        self.assertEqual(self.call('base_drive', 'stop_move')['state'], 'paused')
        boundary = len(self.ros.published);time.sleep(.1)
        self.assertEqual(len(self.ros.published), boundary)
        schema = next(t for t in self.bundle.get_all_tools() if t['name']=='base_drive')['inputSchema']
        self.assertEqual(schema['x-completion']['timeout'], 125)
        self.assertNotIn('background', schema['x-completion'])
        self.assertFalse(self.callbacks)

    def test_position_profiles_limit_actual_velocity_and_relative_frame(self):
        for speed, vmax, wmax in [(s, driver.LINEAR_LIMIT*f, math.degrees(driver.ANGULAR_LIMIT*f)) for s,f in driver.SPEED_FRACTIONS.items()]:
            with self.subTest(speed=speed):
                self.bundle.sensors.odom_pose = lambda: ((2.,3.,math.pi/2), time.time(), 'odom')
                result = self.call('base_drive','move_relative',x=1.,y=0.,yaw=90.,speed=speed)
                self.assertEqual(result['state'], 'executing')
                self.assertTrue(wait_until(lambda: any(abs(p[2]['linear']['x'])>.001 for p in self.ros.published if p[0]=='/cmd_vel')))
                command = self.ros.published[-1][2]
                self.assertGreater(command['linear']['x'], 0)
                self.assertAlmostEqual(command['linear']['y'],0.,places=6)
                self.assertLessEqual(math.hypot(command['linear']['x'],command['linear']['y']),vmax+1e-9)
                self.assertLessEqual(abs(command['angular']['z']),math.radians(wmax)+1e-9)
                self.call('base_drive','stop_move');self.ros.raw();self.ros.published.clear()

    def test_relative_target_uses_body_frame_and_feedback_loss_stops(self):
        self.bundle.sensors.odom_pose = lambda: ((2.,3.,math.pi/2), time.time(), 'odom')
        result = self.call('base_drive','move_relative',x=0.,y=-1.,yaw=0.)
        self.assertEqual(result['state'],'executing')
        self.assertTrue(wait_until(lambda: len(self.ros.published)>1))
        self.assertLess(self.ros.published[-1][2]['linear']['y'],0.)
        self.bundle.sensors.odom_pose = lambda: (_ for _ in ()).throw(RuntimeError('stale odometry'))
        self.assertTrue(wait_until(lambda:self.callbacks))
        self.assertEqual(self.callbacks[0][2],'error')
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_pose_repeated_capture_cannot_complete_but_distinct_samples_can(self):
        capture = time.time()
        self.bundle.sensors.odom_pose = lambda: ((0.,0.,0.), capture, 'odom')
        result = self.call('base_drive','move_relative',x=0.,y=0.,yaw=0.)
        self.assertEqual(result['state'],'executing')
        time.sleep(.36)
        self.assertFalse(self.callbacks)
        self.assertTrue(self.bundle.tracker.current('base',result['action_id']))
        self.bundle.sensors.odom_pose = lambda: ((0.,0.,0.), time.time(), 'odom')
        self.assertTrue(wait_until(lambda:self.callbacks))
        self.assertEqual(self.callbacks[0][2],'completed')
        self.assertTrue(self.callbacks[0][3]['measured_convergence'])

    def test_pose_servo_overcomes_small_command_deadband_without_relaxing_tolerance(self):
        # A plant that ignores small per-axis speeds reproduces the observed
        # P-only stall. Integral correction must finish within the same 2cm/1deg.
        pose = [0.,0.,0.]
        servo = driver._PoseServo(.24,.3)
        settled = 0
        for step in range(600):
            dx,dy = .2-pose[0], .2-pose[1]
            da = math.atan2(math.sin(math.pi/2-pose[2]), math.cos(math.pi/2-pose[2]))
            vx,vy,w = servo.command(dx,dy,da,pose[2],10.+step*.05)
            self.assertLessEqual(math.hypot(vx,vy), .24+1e-9)
            self.assertLessEqual(abs(w), .3+1e-9)
            vx,vy,w = [v if abs(v)>=.05 else 0. for v in (vx,vy,w)]
            pose[0] += (math.cos(pose[2])*vx-math.sin(pose[2])*vy)*.05
            pose[1] += (math.sin(pose[2])*vx+math.cos(pose[2])*vy)*.05
            pose[2] += w*.05
            close = math.hypot(.2-pose[0],.2-pose[1])<=.02 and abs(math.pi/2-pose[2])<=math.radians(1.)
            settled = settled+1 if close else 0
            if settled >= 8:
                break
        self.assertGreaterEqual(settled, 8)

    def test_pose_servo_replayed_samples_do_not_accumulate_and_bias_is_cleared_at_target(self):
        servo = driver._PoseServo(.24,.3)
        first = servo.command(.05,.05,.04,0.,10.)
        for _ in range(100):
            self.assertEqual(servo.command(.05,.05,.04,0.,10.),first)
        servo.command(.05,.05,.04,0.,10.1)
        self.assertTrue(any(servo.bias))
        self.assertEqual(servo.command(.001,.001,.001,0.,10.2),(0.,0.,0.))
        self.assertEqual(servo.bias,[0.,0.,0.])
        for step in range(100):
            vx,vy,w = servo.command(5.,5.,3.,0.,20.+step*.05)
            self.assertLessEqual(math.hypot(vx,vy),.24+1e-9)
            self.assertLessEqual(abs(w),.3+1e-9)
        self.assertEqual(servo.bias,[0.,0.,0.])

    def test_pose_info_reports_target_measured_residual_and_command(self):
        self.bundle.sensors.odom_pose = lambda: ((2.,3.,0.), time.time(), 'odom')
        result = self.call('base_drive', 'move_relative', x=.2, y=.2, yaw=90.)
        self.assertTrue(wait_until(lambda: self.call('base_drive', 'info')['motion'].get('command')))
        progress = self.call('base_drive', 'info')['motion']
        self.assertEqual(progress['action_id'], result['action_id'])
        self.assertEqual(progress['target'], {'x_m': 2.2, 'y_m': 3.2, 'yaw_deg': 90.})
        self.assertAlmostEqual(progress['error']['position_m'], math.sqrt(.08), places=5)
        self.assertEqual(progress['error']['yaw_deg'], 90.)
        self.assertLessEqual(abs(progress['command']['yaw_rad_s']), .3)

    def test_base_stable_small_residual_stops_before_approximate_acp(self):
        self.bundle.sensors.odom_pose = lambda: ((0.,0.,0.),time.time(),'odom')
        result = self.call('base_drive','move_relative',x=.2,y=0.,yaw=0.)
        self.bundle.sensors.odom_pose = lambda: ((.13,0.,math.radians(2.)),time.time(),'odom')
        end = time.monotonic()+2.8
        zero_started = None
        while not self.callbacks and time.monotonic()<end:
            self.ros.emit('/mobile_manipulator/lb_mpc_control_mode',3)
            progress = self.call('base_drive','info')['motion']
            if progress['state']=='stopping' and zero_started is None:
                zero_started=time.monotonic()
            time.sleep(.05)
        self.assertTrue(wait_until(lambda:self.callbacks))
        event=self.callbacks[0]
        self.assertEqual(event[:3],(result['action_id'],'base_drive','completed'))
        self.assertTrue(event[3]['approximate'])
        self.assertFalse(event[3]['measured_convergence'])
        self.assertAlmostEqual(event[3]['error']['position_m'],.07)
        self.assertAlmostEqual(event[3]['error']['yaw_deg'],-2.)
        self.assertIsNotNone(zero_started)
        self.assertGreaterEqual(time.monotonic()-zero_started,1.)
        self.assertEqual(self.ros.published[-1][2]['linear']['x'],0.)
        self.assertIn('0.0700 m',event[3]['message'])

    def test_base_large_residual_does_not_approximate(self):
        self.bundle.sensors.odom_pose = lambda: ((0.,0.,0.),time.time(),'odom')
        result = self.call('base_drive','move_relative',x=.3,y=0.,yaw=0.)
        self.bundle.sensors.odom_pose = lambda: ((.1,0.,0.),time.time(),'odom')
        time.sleep(1.2)
        self.assertFalse(self.callbacks)
        self.assertTrue(self.bundle.tracker.current('base',result['action_id']))
        self.call('base_drive','stop_move')

    def test_base_near_residual_with_replayed_capture_does_not_approximate(self):
        self.bundle.sensors.odom_pose = lambda: ((0.,0.,0.),time.time(),'odom')
        result = self.call('base_drive','move_relative',x=.2,y=0.,yaw=0.)
        captured=time.time()
        self.bundle.sensors.odom_pose = lambda: ((.13,0.,0.),captured,'odom')
        time.sleep(1.25)
        self.assertFalse(self.callbacks)
        self.assertTrue(self.bundle.tracker.current('base',result['action_id']))

    def test_base_approximate_completion_requires_stopping_after_zero(self):
        self.bundle.sensors.odom_pose = lambda: ((0.,0.,0.),time.time(),'odom')
        result = self.call('base_drive','move_relative',x=.2,y=0.,yaw=0.)
        self.bundle.sensors.odom_pose = lambda: ((.13,0.,0.),time.time(),'odom')
        self.assertTrue(wait_until(lambda:self.call('base_drive','info')['motion']['state']=='stopping',1.7))
        start=time.monotonic()
        self.bundle.sensors.odom_pose = lambda: ((.13+.02*(time.monotonic()-start),0.,0.),time.time(),'odom')
        end=start+1.25
        while time.monotonic()<end:
            self.ros.emit('/mobile_manipulator/lb_mpc_control_mode',3)
            time.sleep(.05)
        self.assertFalse(self.callbacks)
        self.assertTrue(self.bundle.tracker.current('base',result['action_id']))
        self.assertEqual(self.ros.published[-1][2]['linear']['x'],0.)
        self.call('base_drive','stop_move')
        self.assertTrue(wait_until(lambda:self.callbacks))
        self.assertEqual(self.callbacks[0][2],'cancelled')

    def test_pose_requires_fresh_stamped_odom_before_resume(self):
        self.assertIn('odometry',self.call('base_drive','move_relative',x=0.,y=0.,yaw=0.)['error'])
        self.assertFalse(self.ros.sent_services)
        self.assertFalse(self.ros.published)


class MotionConfigurationTests(unittest.TestCase):
    def test_explicit_disable_blocks_motion_and_enable_but_preserves_reads_and_pause(self):
        ros = FakeRos1()
        bundle = driver.KuavoBundle({'motion_enabled': False}, 'test', ros, FakeNode())
        try:
            bundle.sensors.discover()
            ros.raw()
            ros.emit('/enable_control_state', True)
            ros.emit('/mobile_manipulator/lb_mpc_control_mode', 3)
            bundle.state.verify_enable()
            cases = [
                ('base_drive', dict(action='move', duration=.1)),
                ('base_drive', dict(action='move_relative', x=0, y=0, yaw=0)),
                ('base_drive', dict(action='move_relative', x=0, y=0, yaw=0)),
                ('arm', dict(action='move_joints', positions=[0.] * 14)),
                ('arm', dict(action='set_pose', frame='world', left_position=[0, .4, 0],
                             right_position=[0, -.4, 0], left_quaternion=[0, 0, 0, 1],
                             right_quaternion=[0, 0, 0, 1])),
                ('leg', dict(action='move_joints', positions=[0.] * 4)),
                ('head', dict(action='move_pos', yaw=0., pitch=0.)),
            ]
            for name, args in cases:
                with self.subTest(card=name, action=args['action']):
                    self.assertIn('motion is disabled', bundle.dispatch(name, args)['error'])
            self.assertEqual(ros.published, [])
            self.assertEqual(ros.services, [])
            self.assertFalse(bundle.state.snapshot()['motion_enabled'])
            self.assertTrue(bundle.sensors.snapshot('joints')['valid'])
            self.assertIn('urdf', bundle.dispatch('model', {}))
            for request in (bundle.resume, lambda: bundle.set_mode(3)):
                with self.assertRaisesRegex(RuntimeError, 'motion is disabled'):
                    request()
            self.assertIsNone(bundle.dispatch('switch_mode', {'action': 'resume'}))
            self.assertEqual(bundle.dispatch('base_drive', {'action': 'stop_move'})['state'], 'paused')
            self.assertFalse(ros.services[-1][2]['data'])
        finally:
            bundle.close()

    def test_motion_flag_rejects_ambiguous_string_configuration(self):
        with self.assertRaisesRegex(ValueError, 'YAML boolean'):
            driver.KuavoBundle({'motion_enabled': 'false'}, 'test', FakeRos1(), FakeNode())


if __name__ == "__main__":
    unittest.main()


class StableFeedbackTests(unittest.TestCase):
    def test_distinct_capture_and_receipt_time_are_both_required(self):
        window=driver._StableFeedback(1.,angular_epsilon=.2)
        for i in range(20):
            self.assertFalse(window.update([0.,0.],10.,20.+i*.1))
        for i in range(1,11):
            self.assertFalse(window.update([0.,0.],10.+i*.1,22.+i*.1))
        self.assertTrue(window.update([0.,0.],11.1,23.1))
        burst=driver._StableFeedback(1.,angular_epsilon=.2)
        for i in range(20):
            self.assertFalse(burst.update([0.,0.],10.+i*.1,20.+i*.001))

    def test_motion_and_large_sampling_gaps_cannot_look_like_plateau(self):
        window=driver._StableFeedback(1.,angular_epsilon=.2)
        for i in range(20):
            self.assertFalse(window.update([i*.1,0.],10.+i*.1,20.+i*.1))
        self.assertFalse(window.update([2.,0.],15.,25.))
        for i in range(1,10):
            self.assertFalse(window.update([2.,0.],15.+i*.1,25.+i*.1))
        self.assertTrue(window.update([2.,0.],16.,26.))

    def test_base_yaw_wrap_is_stable_but_linear_drift_is_not(self):
        window=driver._StableFeedback(1.,2,angular_epsilon=math.radians(.2),angular_period=2*math.pi)
        for i in range(12):
            ready=window.update([1.,2.,math.radians(179.99 if i%2 else -179.99)],10.+i*.1,20.+i*.1)
        self.assertTrue(ready)
        self.assertFalse(window.update([1.01,2.,math.pi],11.2,21.2))
