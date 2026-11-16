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
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parents[1]))
spec = importlib.util.spec_from_file_location("kuavo_device", ROOT / "device.py")
driver = importlib.util.module_from_spec(spec)
std = types.ModuleType("std_msgs.msg")
std.String = type("String", (), {})
with patch.dict(sys.modules, {"std_msgs": types.ModuleType("std_msgs"), "std_msgs.msg": std}):
    spec.loader.exec_module(driver)
    from kuavo_sensors import LINK_ORDER, model_names
    import kuavo_sensors as sensors_module
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
            elif service == "/mobile_manipulator_mpc_control":
                self.emit("/mobile_manipulator/lb_mpc_control_mode", args["control_mode"])
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

    def subscribe(self, topic, message_type, callback):
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

    def tearDown(self):
        self.bundle.close()
        self.callback_patch.stop()

    def call(self, card, action, **args):
        return self.bundle.dispatch(card, {"action": action, **args})

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
            self.assertEqual(set(schema["properties"]["action"]["enum"]), set(schema["x-action-params"]))
            self.assertTrue(all(set(a["params"]) <= set(schema["properties"])
                                for a in schema["x-action-params"].values()))
            if name != "switch_mode":
                self.assertIn("x-completion", schema)
                self.assertIn("x-resource", schema)
            else:
                self.assertNotIn("x-resource", schema)  # Whole-body exclusion.
        self.assertEqual(tools["leg"]["inputSchema"]["x-resource"], tools["waist"]["inputSchema"]["x-resource"])

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
        self.assertIn("topic_out", self.bundle.dispatch("state", {"action": "info"}))

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
        self.assertEqual(self.call("loco", "start")["state"], "ready")  # Lifecycle only.
        self.assertEqual(self.call("loco", "info")["state"], "disconnected")

    def test_actuator_lifecycle_never_waits_for_control_lock_or_calls_hardware(self):
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
            result = self.call('switch_mode', 'set_mode', mode=3)
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
                    result = self.call('switch_mode', action, mode=3)
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
            result = self.call('switch_mode', 'set_mode', mode=3)
        self.assertEqual(result['state'], 'error')
        self.assertIn('connection changed', result['error'])

    def test_stops_preempt_blocked_resume_and_mode_service_without_waiting(self):
        for transition, card, stop_action in [('resume', 'loco', 'stop_move'),
                                               ('set_mode', 'switch_mode', 'pause')]:
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
                        self.call('switch_mode', transition, mode=3)))
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
                self.assertEqual(self.call('loco', 'move', duration=.1)['state'], 'error')
                if transition == 'resume':
                    pending = self.bundle._remote_resume['pending']
                    self.ros.service_results[id(pending)] = {'success': False}
                    self.assertEqual(self.call('switch_mode', 'pause')['state'], 'paused')

    def _timeout_remote_resume(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        with patch.object(self.ros, 'wait_service', side_effect=TimeoutError('remote request unresolved')):
            self.assertEqual(self.call('switch_mode', 'resume')['state'], 'error')
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
            stopped = self.call('switch_mode', 'pause')
        self.assertEqual(stopped['state'], 'error')
        self.assertFalse(stopped['physical_stop_confirmed'])
        self.assertTrue(self.bundle.state.confirmed_enable())
        self.assertTrue(self.bundle.state.motion_blocked)
        # Explicitly pausing again sends disable AFTER the old resume is settled.
        self.assertEqual(self.call('switch_mode', 'pause')['state'], 'paused')
        self.assertFalse(self.bundle.state.confirmed_enable())
        self.assertFalse(self.bundle.state.snapshot()['remote_resume_unresolved'])
        self.assertEqual([s[2]['data'] for s in self.ros.sent_services], [True, False, False])

    def test_resume_acceptance_without_terminal_true_does_not_settle_hazard(self):
        pending = self._timeout_remote_resume()
        self.ros.service_results[id(pending)] = {'success': True}
        self.assertEqual(self.call('switch_mode', 'pause')['state'], 'error')
        self.assertFalse(self.bundle.state.confirmed_enable())
        count = len(self.ros.sent_services)
        for action in ('resume', 'set_mode'):
            self.assertIn('remote resume is unresolved', self.call('switch_mode', action, mode=3)['error'])
        self.assertEqual(len(self.ros.sent_services), count)
        self.assertTrue(self.bundle.state.remote_resume_unresolved)

    def test_explicit_resume_rejection_allows_confirmed_pause(self):
        pending = self._timeout_remote_resume()
        self.ros.service_results[id(pending)] = {'success': False}
        self.assertEqual(self.call('switch_mode', 'pause')['state'], 'paused')
        self.assertFalse(self.bundle.state.remote_resume_unresolved)
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_precommand_true_cannot_settle_unresolved_resume(self):
        pending = self._timeout_remote_resume()
        self.ros.service_results[id(pending)] = {'success': True}
        with self.bundle.state._lock:
            self.bundle.state._values['control_enabled'] = True
            self.bundle.state._seen['control_enabled'] = self.bundle._remote_resume['issued'] - .01
        stopped = self.call('switch_mode', 'pause')
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
        self.assertEqual(self.call('switch_mode', 'pause')['state'], 'error')
        self.assertTrue(self.bundle.state.remote_resume_unresolved)
        self.assertIn('unresolved', self.call('switch_mode', 'resume')['error'])

    def test_resume_send_failure_cannot_be_mistaken_for_remote_cancellation(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        with patch.object(self.ros, 'start_service', side_effect=ConnectionError('partial send')):
            self.assertEqual(self.call('switch_mode', 'resume')['state'], 'error')
        stopped = self.call('switch_mode', 'pause')
        self.assertEqual(stopped['state'], 'error')
        self.assertFalse(stopped['physical_stop_confirmed'])
        self.assertTrue(stopped['remote_resume_unresolved'])

    def test_normal_resume_settles_remote_hazard_before_unblocking(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        self.assertEqual(self.call('switch_mode', 'resume')['state'], 'ready')
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
            worker = threading.Thread(target=lambda: results.append(self.call('switch_mode', 'resume')))
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('switch_mode', 'pause')['state'], 'paused')
                self.assertTrue(worker.is_alive())
            finally:
                release.set()
                worker.join(1)
        self.assertIn('superseded', results[0]['error'])
        self.assertEqual([r[2] for r in self.ros.sent_services], [{'data': False}])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_stop_preempts_service_wait_and_cancels_other_actions_exactly_once(self):
        # A pause service can itself be waiting while ACP actions are cancelled.
        action = self.call('loco', 'move', duration=1)
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
            worker = threading.Thread(target=lambda: results.append(self.call('switch_mode', 'pause')))
            worker.start()
            try:
                self.assertTrue(entered.wait(1))
                self.assertEqual(self.call('loco', 'stop_move')['state'], 'paused')
                self.assertTrue(worker.is_alive())
                self.assertTrue(wait_until(lambda: bool(self.callbacks)))
                self.assertEqual([c[:3] for c in self.callbacks], [(action['action_id'], 'loco', 'cancelled')])
            finally:
                release.set()
                worker.join(1)
        self.assertIn('superseded', results[0]['error'])

    def test_stale_enable_never_allows_motion_resume_or_mode_shortcuts(self):
        self.bundle.state._enable_verified_at = time.monotonic() - 3600
        self.bundle.state._seen['control_enabled'] = time.monotonic() - 3600
        for card, args in [('loco', dict(action='move', duration=.1)),
                           ('switch_mode', dict(action='resume')),
                           ('switch_mode', dict(action='set_mode', mode=3))]:
            self.assertIn('verification is missing or stale', self.bundle.dispatch(card, args)['error'])
        self.assertEqual(self.ros.published, [])
        self.assertEqual(self.ros.services, [])

    def test_stale_false_does_not_skip_pause_confirmation(self):
        self.ros.emit('/enable_control_state', False)
        self.bundle.state._seen['control_enabled'] = time.monotonic() - 3600
        self.bundle.state._enable_verified_at = time.monotonic() - 3600
        result = self.call('switch_mode', 'pause')
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
        self.assertIn('verification is missing or stale', self.call('loco', 'move', duration=.1)['error'])
        self.assertEqual(self.ros.published, [])

    def test_enable_verification_loss_aborts_inflight_action(self):
        action = self.call('loco', 'move', duration=1)
        self.ros.enable_publishers = []
        self.bundle.state.verify_enable()
        self.assertTrue(wait_until(lambda: bool(self.callbacks)))
        self.assertEqual(self.callbacks[0][:3], (action['action_id'], 'loco', 'error'))
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
            result = self.call('switch_mode', 'pause')
        self.assertEqual(result['state'], 'error')
        self.assertLessEqual(len(result['error']), 200)
        self.assertTrue(log.called)
        for call in log.call_args_list:
            line = call[0][0]
            self.assertLess(len(line), 500)
            self.assertFalse(any(c in line for c in ('\n', '\x00', '\x1b')))

    def test_cartesian_arm_mode_four_completes_without_joint_measurements(self):
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
        self.assertEqual(self.ros.services, [])

    def test_motion_requires_enabled_fresh_supported_mode_and_warmup(self):
        for setup in (
            lambda: self.ros.emit("/enable_control_state", False),
            lambda: self.bundle.state._seen.update(mpc_mode=time.monotonic() - 3),
            lambda: self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 1),
            lambda: setattr(self.ros, "motion_ready", False),
        ):
            self.ros.emit("/enable_control_state", True)
            self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 3)
            setup()
            self.assertEqual(self.call("loco", "move", duration=0.1)["state"], "error")
        self.assertEqual(self.ros.published, [])

    def test_rejects_invalid_values_before_hardware_calls(self):
        for args in ({"vx": 1, "duration": 1}, {"vx": True, "duration": 1},
                     {"vx": float("nan"), "duration": 1}, {"duration": 0}, {"duration": "1"}):
            self.assertEqual(self.call("loco", "move", **args)["state"], "error")
        self.assertEqual(self.call("arm", "move_joints", positions=[0] * 13)["state"], "error")
        self.assertEqual(self.call("waist", "set_pose", x=0)["state"], "error")
        self.assertEqual(self.ros.published, [])
        self.assertEqual(self.ros.services, [])

    def test_arm_does_not_enable_quick_mode_and_requires_fresh_estimate(self):
        self.ros.emit("/lb_arm_joint_reach_time", 0)
        result = self.call("arm", "move_joints", positions=list(range(14)))
        self.assertIn("action_id", result)
        self.assertEqual(self.ros.services, [])
        self.assertEqual(self.ros.published[-1], ("/kuavo_arm_traj", "sensor_msgs/JointState", {
            "header": {"seq": 0, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": ""},
            "name": [f"joint{i}" for i in range(1, 15)], "position": list(range(14)), "velocity": [], "effort": []}))
        time.sleep(0.06)
        self.assertEqual(self.callbacks, [])  # Cached estimate cannot complete new motion.
        self.ros.emit("/lb_arm_joint_reach_time", 0)
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
        second = self.call("waist", "set_pose", x=0, z=0.8, pitch=0, yaw=0)
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
        result = self.call("loco", "move", vx=0.1, duration=0.1)
        self.assertIn("action_id", result)
        self.assertGreater(self.ros.published[0][2]["linear"]["x"], 0)
        self.assertTrue(wait_until(lambda: self.ros.published[-1][2]["linear"]["x"] == 0))
        self.assertEqual(self.callbacks, [])
        self.assertTrue(wait_until(lambda: self.callbacks, 1.3))
        self.assertEqual(self.callbacks[0][2], "completed")
        self.assertEqual(self.callbacks[0][3]["stop_delay_s"], 1.0)

    def test_base_busy_or_invalid_command_preserves_active_motion(self):
        velocity = self.call("loco", "move", vx=0.1, duration=1)
        bad = self.call("loco", "move_relative", x=0.2)
        self.assertEqual(bad['state'], 'error')
        pose = self.call("loco", "move_relative", x=0.2, y=0, yaw=0)
        self.assertIn('busy', pose['error'])
        self.assertTrue(self.bundle.tracker.current('base', velocity['action_id']))
        self.assertFalse(any(p[0] == '/cmd_pose' for p in self.ros.published))
        self.assertEqual(self.callbacks, [])

    def test_split_action_schemas_are_action_specific(self):
        tool = next(t for t in self.bundle.get_all_tools() if t["name"] == "switch_mode")
        specs = tool["inputSchema"]["x-action-params"]
        self.assertIn("mode", tool["inputSchema"]["properties"])
        self.assertEqual(specs["set_mode"]["required"], ["mode"])
        self.assertEqual(specs["resume"]["required"], [])
        tools = {t['name']: t for t in self.bundle.get_all_tools()}
        self.assertEqual(tools['loco']['inputSchema']['x-action-params']['move']['required'], ['duration'])
        self.assertEqual(tools['arm']['inputSchema']['x-action-params']['move_joints']['required'], ['positions'])

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
        self.assertEqual(self.call('loco', 'move', duration=.1)['state'], 'executing')

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
                    self.assertEqual(self.call('switch_mode', 'resume')['state'], 'ready')

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
                self.assertEqual(self.call('switch_mode', 'resume')['state'], 'ready')

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

    def _run_stamped_joint_trace(self, samples):
        """Exercise actual ROS callbacks + motion worker, with separate clocks.

        Samples are (receipt elapsed, sensor capture elapsed, leg degrees).
        Header seq changes even when the sensor capture itself is replayed.
        """
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
                seq, (received, captured, degrees) = entry
                clock[0] = started + received
                ns = int((wall + captured) * 1e9)
                self.ros.raw([math.radians(degrees)] * 4 + [0.] * 16,
                             sensor_time={'secs': ns // 1_000_000_000, 'nsecs': ns % 1_000_000_000},
                             header={'seq': seq})
                return False
            with patch.object(cancelled, 'wait', side_effect=poll):
                worker()
        self.bundle.tracker._callbacks.join()
        return result

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
        result = self.call("loco", "move", vx=0.1, duration=1)
        stopped = self.call("loco", "stop_move")
        self.assertEqual(stopped["state"], "paused")
        boundary = len(self.ros.published)
        time.sleep(0.1)
        self.assertEqual(len(self.ros.published), boundary)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual([c[:3] for c in self.callbacks], [(result["action_id"], "loco", "cancelled")])
        self.assertEqual(self.call("loco", "move", duration=0.1)["state"], "error")

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
        self.assertIn('delivery is uncertain', self.call('switch_mode', 'resume')['error'])
        self.assertEqual(self.call('loco', 'move', duration=.1)['state'], 'error')
        count = len(self.ros.sent_services)
        self.bundle.close()
        self.assertEqual(len(self.ros.sent_services), count + 1)
        self.assertEqual(self.ros.sent_services[-1][2], {'data': False})
        self.assertEqual(len(self.callbacks), 1)

    def test_partial_initial_position_send_fails_closed_and_shutdown_retries_stop(self):
        self._partial_initial_send('loco', 'move_relative', '/cmd_pose', x=.01, y=0., yaw=0.)

    def test_partial_initial_velocity_send_fails_closed(self):
        self._partial_initial_send('loco', 'move', '/cmd_vel', vx=.01, duration=.2)

    def test_partial_initial_joint_send_fails_closed(self):
        self._partial_initial_send('arm', 'move_joints', '/kuavo_arm_traj', positions=[0.] * 14)

    def test_initial_send_disconnect_remains_uncertain_after_reconnect(self):
        def disconnect(*_args):
            self.ros.connection = (False, 1)
            raise ConnectionError('delivery unknown')
        with patch.object(self.ros, 'publish', side_effect=disconnect):
            result = self.call('loco', 'move', duration=.2)
        self.assertEqual(result['command_delivery'], 'uncertain')
        self.assertFalse(result['physical_stop_confirmed'])
        self.ros.connection = (True, 2)
        self.bundle.sensors.discover()
        self.ros.raw()
        self.ros.emit('/enable_control_state', True)
        self.ros.emit('/mobile_manipulator/lb_mpc_control_mode', 3)
        self.bundle.state.verify_enable()
        self.assertIn('delivery is uncertain', self.call('switch_mode', 'resume')['error'])
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
            worker = threading.Thread(target=lambda: result.append(self.call('loco', 'move', duration=.2)))
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
        self.assertEqual(self.call('loco', 'move', duration=-1)['state'], 'error')
        self.assertFalse(self.bundle.state.motion_send_uncertain)
        self.assertFalse(self.bundle.state.motion_blocked)
        self.assertEqual(self.ros.sent_services, [])

    def test_pause_service_acceptance_without_feedback_not_success(self):
        self.ros.confirm = False
        with patch.object(self.bundle, "wait_state", side_effect=TimeoutError("not confirmed")):
            result = self.call("switch_mode", "pause")
        self.assertEqual(result["state"], "error")
        self.assertFalse(result["physical_stop_confirmed"])
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_pause_rejection_and_resume_feedback(self):
        self.ros.service_accepts = False
        self.assertEqual(self.call("switch_mode", "pause")["state"], "error")
        self.ros.service_accepts = True
        self.assertEqual(self.call("switch_mode", "pause")["state"], "paused")
        self.assertEqual(self.call("switch_mode", "resume")["state"], "ready")
        self.assertFalse(self.bundle.state.motion_blocked)
        self.assertEqual(self.ros.services[-1][:3], ("/enable_control", "std_srvs/SetBool", {"data": True}))

    def test_mode_switch_can_leave_no_control_and_is_confirmed(self):
        self.ros.emit("/mobile_manipulator/lb_mpc_control_mode", 0)
        result = self.call("switch_mode", "set_mode", mode=3)
        self.assertEqual(result["state"], "completed")
        self.assertEqual(self.ros.services[-1][:3], ("/mobile_manipulator_mpc_control", "kuavo_msgs/changeTorsoCtrlMode", {"control_mode": 3}))

    def test_feedback_loss_causes_error_not_false_completed(self):
        result = self.call("arm", "move_joints", positions=[0] * 14)
        self.ros.connection = (False, 1)
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.callbacks[0][:3], (result["action_id"], "arm", "error"))
        self.assertTrue(self.bundle.state.motion_blocked)

    def test_head_uses_degrees_per_second_and_has_acp(self):
        result = self.call("head", "move", yaw=12, pitch=4, duration=0.1)
        self.assertIn("action_id", result)
        self.assertEqual(self.ros.published[0], ("/cmd_head_vel", "kuavo_msgs/robotHeadMotionData", {"joint_data": [12, 4]}))
        self.assertTrue(wait_until(lambda: self.callbacks))
        self.assertEqual(self.ros.published[-1][2], {"joint_data": [0, 0]})
        self.assertEqual(self.callbacks[0][3]["stop_delay_s"], 0.3)

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
                result = self.call("loco", "stop_move")
                self.assertEqual(result["state"], "paused")
                self.assertFalse(release.is_set())
            finally:
                release.set()
                self.bundle.tracker._callbacks.join()

    def test_excessive_reach_time_aborts_and_pauses(self):
        result = self.call("arm", "move_joints", positions=[0] * 14)
        self.ros.emit("/lb_arm_joint_reach_time", 111)
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
        self.call("loco", "move", duration=1)
        self.bundle.close()
        self.assertEqual(self.ros.services[-1][0], "/enable_control")
        self.assertEqual(self.call("loco", "move", duration=1)["state"], "error")
        # close is idempotent, including the callback worker.
        self.assertTrue(self.bundle._closing)


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
                ('loco', dict(action='move', duration=.1)),
                ('loco', dict(action='move_relative', x=0, y=0, yaw=0)),
                ('loco', dict(action='move_to', x=0, y=0, yaw=0)),
                ('arm', dict(action='move_joints', positions=[0.] * 14)),
                ('arm', dict(action='set_pose', frame='world', left_position=[0, .4, 0],
                             right_position=[0, -.4, 0], left_quaternion=[0, 0, 0, 1],
                             right_quaternion=[0, 0, 0, 1])),
                ('leg', dict(action='move_joints', positions=[0.] * 4)),
                ('waist', dict(action='set_pose', x=0, z=.8, pitch=0, yaw=0)),
                ('head', dict(action='move', duration=.1)),
                ('switch_mode', dict(action='resume')),
                ('switch_mode', dict(action='set_mode', mode=3)),
            ]
            for name, args in cases:
                with self.subTest(card=name, action=args['action']):
                    self.assertIn('motion is disabled', bundle.dispatch(name, args)['error'])
            self.assertEqual(ros.published, [])
            self.assertEqual(ros.services, [])
            self.assertFalse(bundle.dispatch('state', {'action': 'info'})['feedback']['motion_enabled'])
            self.assertTrue(bundle.sensors.snapshot('joints')['valid'])
            self.assertIn('urdf', bundle.dispatch('model', {}))
            self.assertEqual(bundle.dispatch('switch_mode', {'action': 'pause'})['state'], 'paused')
            self.assertFalse(ros.services[-1][2]['data'])
        finally:
            bundle.close()

    def test_motion_flag_rejects_ambiguous_string_configuration(self):
        with self.assertRaisesRegex(ValueError, 'YAML boolean'):
            driver.KuavoBundle({'motion_enabled': 'false'}, 'test', FakeRos1(), FakeNode())


if __name__ == "__main__":
    unittest.main()
