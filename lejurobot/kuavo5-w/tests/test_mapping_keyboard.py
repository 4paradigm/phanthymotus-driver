"""Manual web mapping: ownership, replay/timeout fences and stop confirmation."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import types
import sys
import base64
import hashlib
import socket
import struct
import unittest
from unittest.mock import patch

import test_controlled_spatial as spatial_fixtures
from controlled_spatial import ControlledSpatialPlugin, SpatialStore
import test_driver as driver_fixtures
from test_driver import wait_until

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('keyboard_remote', ROOT/'host/manual_keyboard.py')
remote = importlib.util.module_from_spec(spec)
spec.loader.exec_module(remote)


class Stream:
    def __init__(self):
        self.sent, self.closed = [], False
        self.held, self.stopping, self.confirm_stop = False, False, True

    def read(self, timeout):
        if not self.held:
            self.held = True
            return {'state': 'held', 'nonce': 'first'}
        if self.stopping:
            if not self.confirm_stop:
                raise ConnectionError('stop ack lost')
            return {'state': 'stopped', 'stationary': True, 'software_estop': True}
        return {'state': 'ready', 'nonce': 'next', 'seq': self.sent[-1]['seq']}

    def send(self, value):
        self.sent.append(value)

    def stop(self):
        self.stopping = True

    def close(self):
        self.closed = True


class MappingKeyboardTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.bundle, self.client = spatial_fixtures.Bundle(), spatial_fixtures.Client()
        self.spatial = ControlledSpatialPlugin(self.bundle, {'db_path': self.temp.name+'/db'}, self.client)
        self.bundle.spatial = self.spatial
        self.stream = Stream()
        self.client.keyboard_stream = lambda: self.stream
        self.spatial.store.state('mapping', {'name': 'new', 'token': 'mapping-id', 'phase': 'mapping'})
        self.keyboard = self.spatial.keyboard

    def tearDown(self):
        self.keyboard.stop()
        if self.keyboard.thread:
            self.keyboard.thread.join(2)
        self.spatial.close()
        self.temp.cleanup()

    def open(self):
        result = self.spatial.dispatch('keyboard_open', {})
        self.assertNotEqual(result['state'], 'error', result)
        self.session, self.seq = result['session_id'], 0
        self.assertTrue(wait_until(lambda: self.keyboard.info()['state'] == 'ready'))

    def send(self, values):
        self.seq += 1
        return self.spatial.dispatch('keyboard_command',
            {'session_id': self.session, 'seq': self.seq, 'values': values})

    def test_panel_open_has_no_side_effects_and_exposes_existing_canvas_contract(self):
        page = self.spatial.dispatch('mapping_console', {})
        self.assertEqual(page['protocol'], 'motus.workbench/1')
        self.assertIn('Alt+W / I', page['html'])
        self.assertIn("window.addEventListener('blur'", page['html'])
        self.assertIn("document.addEventListener('visibilitychange'", page['html'])
        self.assertFalse(self.client.calls)
        self.assertFalse(self.bundle.tracker._active)
        self.assertFalse(self.spatial.unresolved())
        schema = self.spatial.get_tool()['inputSchema']
        self.assertEqual(schema['x-workbench']['entry'], 'mapping_console')
        for action, params in {'keyboard_open': [], 'keyboard_command': ['session_id', 'seq', 'values'],
                               'keyboard_release': ['session_id'], 'keyboard_status': []}.items():
            self.assertEqual(schema['x-action-params'][action]['params'], params)
            self.assertEqual(schema['x-action-params'][action]['required'], params)
            self.assertTrue(set(params) <= set(schema['properties']))

    def test_zero_activation_and_browser_timeout_hold_and_release_resource(self):
        self.open()
        self.assertEqual(self.stream.sent[0]['values'], [0., 0.])
        self.assertTrue(self.spatial.unresolved())
        self.assertEqual(self.bundle.tracker._active['base'][1], 'mapping_keyboard')
        self.assertEqual(self.send([.15, -.3])['state'], 'ready')
        self.assertTrue(wait_until(lambda: any(v['values'] == [.15, -.3] for v in self.stream.sent)))
        self.assertTrue(wait_until(lambda: self.keyboard.session is None, timeout=2))
        self.assertTrue(self.stream.stopping)
        self.assertFalse(self.spatial.unresolved())
        self.assertNotIn('base', self.bundle.tracker._active)
        self.assertTrue(self.spatial.mapping_active())  # watchdog never ends SLAM
        self.assertEqual(self.send([.15, 0])['state'], 'error')

    def test_replayed_expired_wrong_session_and_nonfinite_commands_are_rejected(self):
        self.open()
        self.assertEqual(self.send([0, 0])['state'], 'ready')
        args = {'session_id': self.session, 'seq': self.seq, 'values': [.1, 0]}
        self.assertEqual(self.spatial.dispatch('keyboard_command', args)['state'], 'error')
        for update in ({'session_id': 'old', 'seq': 500}, {'values': [.151, 0], 'seq': 501},
                       {'values': [0, .301], 'seq': 502}, {'values': [True, 0], 'seq': 503},
                       {'values': [float('nan'), 0], 'seq': 504}):
            self.assertEqual(self.spatial.dispatch('keyboard_command', {**args, **update})['state'], 'error')
        self.assertFalse(any(any(v['values']) for v in self.stream.sent))

    def test_stop_map_load_and_navigation_cannot_overwrite_keyboard(self):
        self.open()
        for action, args in [('stop_mapping', {}), ('load_map', {'map_name': 'other'}),
                             ('navigate_to_pose', {'x': 1, 'y': 2, 'yaw': 0}),
                             ('delete_map', {'map_name': 'other', 'confirm_delete': True})]:
            self.assertEqual(self.spatial.dispatch(action, args)['state'], 'error')
        self.assertFalse(self.client.commands)
        self.assertFalse(self.client.calls)

    def test_second_panel_and_old_panel_release_cannot_take_new_control(self):
        self.open()
        self.assertEqual(self.spatial.dispatch('keyboard_open', {})['state'], 'error')
        self.assertEqual(self.spatial.dispatch('keyboard_release', {'session_id': 'old'})['state'], 'error')
        self.assertEqual(self.keyboard.info()['state'], 'ready')

    def test_pending_shared_mode_enable_or_uncertain_send_rejects_keyboard_before_io(self):
        for attribute in ('_resume_unresolved', '_mode_unresolved', 'motion_send_uncertain'):
            with self.subTest(attribute=attribute):
                target = self.bundle.state if attribute == 'motion_send_uncertain' else self.bundle
                value = True if attribute == 'motion_send_uncertain' else lambda: True
                with patch.object(target, attribute, value):
                    self.assertEqual(self.spatial.dispatch('keyboard_open', {})['state'], 'error')
                self.assertFalse(self.stream.sent)
                self.assertFalse(self.bundle.tracker._active)

    def test_stop_epoch_reconnect_and_mode_route_change_each_invalidate_lease(self):
        for cause in ('epoch', 'connection', 'upper_velocity', 'mapping_token'):
            with self.subTest(cause=cause):
                self.stream = Stream()
                self.open()
                if cause == 'epoch':
                    self.bundle._stop_epoch += 1
                elif cause == 'connection':
                    self.bundle.ros1.connection = (True, 2)
                elif cause == 'upper_velocity':
                    self.bundle.state.read = lambda key: (True, time.monotonic())
                else:
                    self.spatial.store.state('mapping', {'phase': 'mapping', 'token': 'changed'})
                self.assertTrue(wait_until(lambda: self.keyboard.session is None))
                self.assertTrue(self.stream.stopping)
                self.bundle.state.read = lambda key: (False, time.monotonic())

    def test_unconfirmed_stop_is_persisted_and_requires_confirmed_reconciliation(self):
        self.stream.confirm_stop = False
        self.open()
        self.keyboard.stop()
        self.assertTrue(wait_until(lambda: self.keyboard.session is None))
        self.assertTrue(self.spatial.unresolved())
        self.assertTrue(self.spatial.store.state('pending')['keyboard_stop_unconfirmed'])
        self.spatial.cancel_remote()
        self.assertEqual(self.client.calls[-1][0], 'soft_stop')
        self.assertFalse(self.spatial.unresolved())

    def test_preparation_directions_are_discarded_and_disconnected_browser_never_activates(self):
        gate, entered = threading.Event(), threading.Event()
        def stream():
            entered.set()
            gate.wait(2)
            return self.stream
        self.client.keyboard_stream = stream
        result = self.spatial.dispatch('keyboard_open', {})
        self.session, self.seq = result['session_id'], 0
        self.assertTrue(entered.wait(1))
        self.assertEqual(self.send([.15, 0])['state'], 'loading')
        time.sleep(.33)
        gate.set()
        self.assertTrue(wait_until(lambda: self.keyboard.session is None))
        self.assertFalse(self.stream.sent)
        self.assertTrue(self.stream.stopping)


class MappingCardCoexistenceTests(unittest.TestCase):
    def setUp(self):
        driver_fixtures.DriverContractTests.setUp(self)
        self.temp = tempfile.TemporaryDirectory()
        self.bundle.spatial.store = SpatialStore(self.temp.name+'/db')
        self.bundle.spatial.store.state('mapping', {'name': 'new', 'token': 'mapping-id', 'phase': 'mapping'})
        self.bundle.spatial.client = spatial_fixtures.Client()
        self.ros.emit('/enable_vel_control_state', False)

    def tearDown(self):
        driver_fixtures.DriverContractTests.tearDown(self)
        self.temp.cleanup()

    call = driver_fixtures.DriverContractTests.call

    def test_card_start_during_manual_mapping_does_not_prepare_or_change_modes(self):
        import sys
        for card in ('base_drive', 'controlled_spatial'):
            result = self.call(card, 'start')
            self.assertEqual(result['state'], 'ready')
            self.assertTrue(result['preparation_deferred'])
        with patch.dict(sys.modules, {'std_msgs.msg': driver_fixtures.std}):
            self.node.create_subscription = lambda *args: None
            result = self.call('base_servo', 'start', input_topic='/actucore/navi/cmd')
        self.assertEqual(result['state'], 'ready')
        self.assertTrue(self.bundle.base_servo._paused)
        self.assertFalse(self.bundle.spatial.client.calls)
        self.assertFalse(self.ros.sent_services)
        self.assertFalse(self.ros.published)

    def test_start_and_joint_motion_during_active_keyboard_do_not_change_its_route(self):
        stream = Stream()
        self.bundle.spatial.client.keyboard_stream = lambda: stream
        opened = self.call('controlled_spatial', 'keyboard_open')
        self.assertNotEqual(opened['state'], 'error', opened)
        self.assertTrue(wait_until(lambda: self.bundle.spatial.keyboard.info()['state'] == 'ready'))
        for card in ('base_drive', 'controlled_spatial'):
            self.assertTrue(self.call(card, 'start')['preparation_deferred'])
        for channel, card in [('head', 'head'), ('arms', 'arm'), ('lower_body', 'leg')]:
            sent = []
            lease = self.bundle.start_motion(channel, card, (3,), lambda: sent.append(True))
            self.assertEqual(sent, [True])
            self.bundle.tracker.finish(channel, lease[0], card, 'completed', {})
        self.assertFalse(self.ros.sent_services)
        self.assertEqual(self.bundle.control_ownership()['base_velocity_route'], 'factory_manual')

    def test_completed_mapping_invalidates_startup_before_upper_mpc_can_send(self):
        self.bundle.spatial.store.state('mapping', None)
        self.bundle.spatial.base_preparation_required = True
        order = []
        def prepare():
            order.append('fresh_preparation')
            self.bundle.spatial.base_preparation_required = False
            return {'state': 'running'}
        with patch.object(self.bundle.spatial, 'start_chassis', side_effect=prepare):
            lease = self.bundle.start_motion('base', 'base_drive', (3,), lambda: order.append('target'),
                request_epoch=self.bundle._stop_epoch)
        self.assertEqual(order, ['fresh_preparation', 'target'])
        self.bundle.tracker.finish('base', lease[0], 'base_drive', 'completed', {})

    def test_base_execution_and_global_mode_changes_reject_without_side_effects(self):
        for action, args in [('move', {'vx': .1, 'duration': 1}),
                             ('move_relative', {'x': .2, 'y': 0, 'yaw': 0})]:
            self.assertEqual(self.call('base_drive', action, **args)['state'], 'error')
        self.assertEqual(self.call('_control', 'set_mode', mode=2)['state'], 'error')
        self.assertEqual(self.call('_control', 'resume')['state'], 'error')
        self.assertEqual(self.call('base_servo', 'resume')['state'], 'error')
        self.assertFalse(self.ros.sent_services)
        self.assertFalse(self.ros.published)

    def test_joint_cards_can_share_ready_basearm_but_cannot_change_shared_enable_or_mode(self):
        for channel, card in [('head', 'head'), ('arms', 'arm'), ('lower_body', 'leg')]:
            sent = []
            lease = self.bundle.start_motion(channel, card, (3,), lambda: sent.append(True))
            self.assertEqual(sent, [True])
            self.bundle.tracker.finish(channel, lease[0], card, 'completed', {})
        self.assertFalse(self.ros.sent_services)
        self.ros.emit('/enable_control_state', False)
        self.bundle.state.verify_enable()
        with self.assertRaisesRegex(RuntimeError, 'shared_enable_or_mode'):
            self.bundle.start_motion('head', 'head', (3,), lambda: self.fail('published'))
        self.assertFalse(self.ros.sent_services)


class ChassisGuardTests(unittest.TestCase):
    def test_factory_websocket_masks_commands_answers_ping_and_rejects_api_errors(self):
        class Wire:
            def __init__(self):
                self.incoming, self.frames, self.closed = bytearray(), [], False
                self.condition = threading.Condition()

            def push(self, value):
                with self.condition:
                    self.incoming.extend(value)
                    self.condition.notify_all()

            def sendall(self, value):
                if value.startswith(b'GET '):
                    self.request = value.decode()
                    key = self.request.split('Sec-WebSocket-Key: ')[1].split('\r\n')[0]
                    accept = base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest())
                    self.push(b'HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: '+accept+b'\r\n\r\n')
                else:
                    length, offset = value[1] & 127, 2
                    if length == 126:
                        length, offset = struct.unpack('!H', value[2:4])[0], 4
                    assert value[1] & 128
                    mask, payload = value[offset:offset+4], value[offset+4:]
                    assert len(payload) == length
                    self.frames.append((value[0] & 15, bytes(v ^ mask[i % 4] for i, v in enumerate(payload))))

            def recv(self, size):
                with self.condition:
                    if not self.incoming and not self.closed:
                        self.condition.wait(.01)
                    if self.closed:
                        return b''
                    if not self.incoming:
                        raise socket.timeout()
                    value = self.incoming[:size]
                    del self.incoming[:size]
                    return bytes(value)

            def shutdown(self, *args):
                self.close()

            def close(self):
                with self.condition:
                    self.closed = True
                    self.condition.notify_all()

        wire = Wire()
        with patch.object(remote.socket, 'create_connection', return_value=wire):
            ws = remote.FactorySocket()
            try:
                self.assertTrue(wire.request.startswith('GET /ws/app'))
                message = json.dumps({'method': 'RobotMotion', 'id': '1', 'params': {'vx': .1, 'vy': 0, 'vw': 0}})
                ws.send(message)
                self.assertEqual(wire.frames[0], (1, message.encode()))
                wire.push(b'\x89\x04ping')
                self.assertTrue(wait_until(lambda: (10, b'ping') in wire.frames))
                error = json.dumps({'type': 'taskResult', 'data': {'method': 'RobotMotion', 'error': {'code': 1}}}).encode()
                wire.push(bytes([0x81, len(error)])+error)
                self.assertTrue(ws.failed.wait(1))
                with self.assertRaises(ConnectionError):
                    ws.send(message)
            finally:
                ws.close()
        self.assertTrue(wire.closed)

    def test_full_remote_guard_orders_zero_release_and_stop_on_eof_or_mode_loss(self):
        for cause in ('eof', 'mode', 'timeout', 'early_release', 'release_on_zero'):
            with self.subTest(cause=cause):
                read_fd, write_fd = os.pipe()
                state = types.SimpleNamespace(manual_mode=True, mapping=True, estop=False,
                    bumper=False, driver_error=False, driver_enable=True, soft_estop=True)
                commands, outputs, errors = [], [], []
                def dispatch(**args):
                    commands.append(('dispatch', args['control_mode']))
                    state.soft_estop = args['control_mode'] == 0
                    return types.SimpleNamespace(success=True)
                def check_stop(rospy, expected):
                    self.assertEqual(state.soft_estop, expected)
                fake = types.ModuleType('keyboard_fixture')
                fake.main = lambda request: {'software_estop': True, 'stationary': True}
                fake.dispatch_service_class = lambda *args: object
                fake.confirm_stationary = lambda *args: None
                fake.confirm_software_stop = check_stop
                ros = types.SimpleNamespace(ServiceProxy=lambda *args: dispatch,
                    Subscriber=lambda *args, **kw: types.SimpleNamespace(unregister=lambda: None),
                    wait_for_message=lambda *args, **kw: state)
                modules = {'keyboard_fixture': fake, 'rospy': ros,
                    'rosgraph': types.SimpleNamespace(Master=lambda *args: types.SimpleNamespace(lookupService=lambda *a: 'uri')),
                    'rosservice': types.SimpleNamespace(get_service_headers=lambda *a: {}),
                    'jaten_msgs.msg': types.SimpleNamespace(RobotChassisState=object),
                    'leju_mobile_base_msgs.srv': types.SimpleNamespace(SetDispatchMode=object),
                    'nav_msgs.msg': types.SimpleNamespace(Odometry=object)}
                def motion(value):
                    commands.append(('motion', json.loads(value)['params']))
                    if cause == 'release_on_zero' and len(commands) == 2:
                        state.soft_estop = False
                ws = types.SimpleNamespace(send=motion, close=lambda: None)
                def run():
                    try:
                        remote.run({'master': 'unused', 'ros_ip': 'unused'})
                    except Exception as exc:
                        errors.append(exc)
                with os.fdopen(read_fd, 'r') as stream, patch.dict(sys.modules, modules), \
                        patch.object(remote, 'NAVIGATION_SOURCE', 'from keyboard_fixture import *', create=True), \
                        patch.object(remote, 'FactorySocket', return_value=ws), \
                        patch.object(remote, 'print', side_effect=lambda value, **kw: outputs.append(json.loads(value)), create=True), \
                        patch.object(sys, 'stdin', stream):
                    worker = threading.Thread(target=run)
                    worker.start()
                    try:
                        self.assertTrue(wait_until(lambda: outputs and outputs[-1]['state'] == 'held'))
                        if cause == 'early_release':
                            state.soft_estop = False
                        def send(seq, values):
                            os.write(write_fd, (json.dumps({'nonce': outputs[-1]['nonce'], 'seq': seq, 'values': values})+'\n').encode())
                        send(1, [0, 0])
                        if cause in ('early_release', 'release_on_zero'):
                            worker.join(1)
                            self.assertFalse(worker.is_alive())
                            self.assertEqual(outputs[-1]['state'], 'stopped')
                            self.assertNotIn(('dispatch', 1), commands)
                            self.assertEqual(commands[-1], ('dispatch', 0))
                            self.assertTrue(state.soft_estop)
                            self.assertTrue(errors)
                            self.assertFalse(any(value.get('state') == 'ready' for value in outputs))
                            continue
                        self.assertTrue(wait_until(lambda: outputs[-1]['state'] == 'ready'))
                        if cause == 'mode':
                            state.manual_mode = False
                        send(2, [.1, .2])
                        if cause == 'eof':
                            self.assertTrue(wait_until(lambda: outputs[-1].get('seq') == 2))
                            os.close(write_fd)
                            write_fd = None
                        worker.join(1)
                        self.assertFalse(worker.is_alive())
                        self.assertEqual(outputs[-1]['state'], 'stopped')
                        self.assertEqual(commands[0], ('motion', {'vx': 0., 'vy': 0., 'vw': 0.}))
                        self.assertEqual(commands[1], ('motion', {'vx': 0., 'vy': 0., 'vw': 0.}))
                        self.assertEqual(commands[2], ('dispatch', 1))
                        self.assertEqual(commands[-2], ('motion', {'vx': 0., 'vy': 0., 'vw': 0.}))
                        self.assertEqual(commands[-1], ('dispatch', 0))
                        if cause == 'mode':
                            self.assertFalse(any(item[0] == 'motion' and item[1]['vx'] for item in commands))
                        self.assertTrue(errors)
                    finally:
                        if write_fd is not None:
                            os.close(write_fd)
                        worker.join(2)

    def test_remote_limits_and_nonce_are_enforced_independently(self):
        self.assertEqual(remote.validate_frame({'nonce': 'n', 'seq': 1, 'values': [.15, -.3]}, 'n', 0), [.15, -.3])
        for frame in ({'nonce': 'old', 'seq': 1, 'values': [0, 0]},
                      {'nonce': 'n', 'seq': 0, 'values': [0, 0]},
                      {'nonce': 'n', 'seq': 1, 'values': [.2, 0]},
                      {'nonce': 'n', 'seq': 1, 'values': [False, 0]}):
            with self.assertRaises(ValueError):
                remote.validate_frame(frame, 'n', 0)

    def test_remote_watchdog_bounds_partial_json_and_disconnect(self):
        read_fd, write_fd = os.pipe()
        with os.fdopen(read_fd, 'rb', buffering=0) as stream:
            try:
                os.write(write_fd, b'{"seq":')
                start = time.monotonic()
                with self.assertRaises(TimeoutError):
                    remote.receive(stream, .03)
                self.assertLess(time.monotonic()-start, .15)
                os.close(write_fd)
                write_fd = None
                with self.assertRaises(EOFError):
                    remote.receive(stream, .03)
            finally:
                if write_fd is not None:
                    os.close(write_fd)


if __name__ == '__main__':
    unittest.main()
