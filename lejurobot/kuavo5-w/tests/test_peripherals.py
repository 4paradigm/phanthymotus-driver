import base64
import importlib.util
import io
import json
import math
import struct
import threading
import tempfile
import sys
from pathlib import Path
import time
import types
import unittest
import zlib
from unittest.mock import patch

from test_driver import driver, FakeRos1, FakeNode, ROOT, peripherals_module, wait_until
Peripherals, battery_sample = peripherals_module.Peripherals, peripherals_module.battery_sample


class PeripheryTests(unittest.TestCase):
    def setUp(self):
        self.ros = FakeRos1()
        self.periphery = Peripherals(self.ros, FakeNode(), 'test')

    def tearDown(self):
        self.periphery.close()

    def test_battery_preserves_sources_and_does_not_claim_zero_voltage(self):
        now = int(time.time() * 1000)
        sources = {topic: {'received_ms': now, 'values': {'percentage': 96.5, 'voltage': 0.,
                   'current': -3.2, 'status': status}} for topic, status in (
                   ('/battery', 4), ('/jaten_behavior_manager/out/leju/battery', 2))}
        sample = battery_sample({'sources': sources}, now)
        self.assertTrue(sample['valid'])
        self.assertEqual(sample['main_soc_percent'], 96.5)
        self.assertIsNone(sample['main_voltage_v'])
        self.assertEqual(sample['main_current_a'], -3.2)
        self.assertEqual(sample['main_status_code'], 4)
        self.assertEqual(sample['chassis_status_code'], 2)
        self.assertTrue(all(not isinstance(value, (dict, list)) for value in sample.values()))

    def test_stale_battery_aggregate_cannot_refresh_old_source(self):
        now = int(time.time() * 1000)
        result = battery_sample({'sources': {'/battery': {'received_ms': now - 6000,
                                 'values': {'percentage': 96.}}}}, now)
        self.assertFalse(result['valid'])

    def test_malformed_battery_and_nonfinite_vendor_fields_are_bounded(self):
        self.periphery._on_battery({'data': '[]'})
        self.assertFalse(self.periphery.snapshot('battery')['valid'])
        now = int(time.time() * 1000)
        result = battery_sample({'sources': {'/battery': {'received_ms': now, 'values':
                                {'percentage': 90., 'voltage': math.nan, 'health': math.inf}}}}, now)
        json.dumps(result, allow_nan=False)
        self.assertIsNone(result['main_voltage_v'])

    def test_laser_null_ranges_and_points_use_actual_laser_frame(self):
        self.periphery.dispatch('laser_scan', 'start')
        self.periphery._on_scan({'header': {'frame_id': 'laser'}, 'angle_min': 0.,
            'angle_increment': math.pi/2, 'range_min': .05, 'range_max': 50.,
            'ranges': [1., 2., math.inf, math.nan, 0.]})
        self.periphery.tick()
        raw = json.loads(self.periphery._pubs['laser_scan'].messages[-1].data)
        self.assertEqual(raw['frame_id'], 'laser')
        self.assertEqual(raw['ranges'], [1., 2., None, None, None])
        payload = self.periphery._points_pub.messages[-1].data
        step, count = struct.unpack_from('<II', payload)
        self.assertEqual(step, 16)
        points = [struct.unpack_from('<ffff', payload, 8+i*step) for i in range(count)]
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0][0], 1.)
        self.assertAlmostEqual(points[1][1], 2.)

    def test_bad_laser_invalidates_previous_measurement(self):
        self.periphery._on_scan({'ranges': [1.]})
        self.assertFalse(self.periphery.snapshot('laser_scan')['valid'])

    def test_pointcloud_is_first_output_and_has_native_binary_envelope(self):
        tool = next(t for t in self.periphery.tools() if t['name'] == 'laser_scan')
        self.assertEqual([t['format'] for t in tool['topic_out']], ['sensor/pointcloud', 'data/json'])
        self.periphery.dispatch('laser_scan', 'start')
        self.periphery.tick()
        self.assertEqual(struct.unpack('<II', self.periphery._points_pub.messages[-1].data), (16, 0))
        qos = peripherals_module.latest_qos()
        self.assertEqual((qos.depth, qos.reliability), (1, 'best_effort'))

    def test_camera_publishes_native_compressed_image_once_per_received_frame(self):
        self.periphery.dispatch('camera_left', 'start')
        payload = {'data': base64.b64encode(b'\xff\xd8test\xff\xd9').decode(), 'header': {'frame_id': 'camera'}}
        self.ros.subscriptions['/camera_1/color/image_raw/compressed'][1](payload)
        self.periphery.tick()
        self.periphery.tick()
        messages = self.periphery._image_pubs['camera_left'].messages
        self.assertEqual(len(messages), 1)
        self.assertEqual(bytes(messages[0].data), b'\xff\xd8test\xff\xd9')
        self.assertEqual(messages[0].format, 'jpeg')
        self.assertEqual(messages[0].header.frame_id, 'camera')

    def test_camera_stop_drops_late_frames_without_disabling_shared_hardware(self):
        self.periphery.dispatch('camera_left', 'start')
        callback = self.ros.subscriptions['/camera_1/color/image_raw/compressed'][1]
        self.periphery.dispatch('camera_left', 'stop')
        callback({'data': base64.b64encode(b'\xff\xd8x\xff\xd9').decode()})
        self.periphery.tick()
        self.assertFalse(self.periphery.snapshot('camera_left')['valid'])
        self.assertTrue(all(call[:3] == ('/camera_left/set_camera_enabled',
                            'std_srvs/SetBool', {'data': True}) for call in self.ros.services))

    def test_camera_reconnect_rebinds_and_only_invokes_camera_enable(self):
        with patch.object(peripherals_module.threading, 'Thread') as thread:
            self.periphery.dispatch('camera_left', 'start')
            initial = thread.call_args
        initial.kwargs['target'](*initial.kwargs['args'])
        self.ros.connection = (True, 2)
        with patch.object(peripherals_module.threading, 'Thread') as thread:
            self.periphery.tick()
            target = thread.call_args.kwargs['target']
            args = thread.call_args.kwargs['args']
        target(*args)
        self.assertEqual(len(self.ros.services), 2)
        self.assertEqual(self.ros.services[-1][:3],
            ('/camera_left/set_camera_enabled', 'std_srvs/SetBool', {'data': True}))
        self.assertEqual(self.ros.published, [])

    def test_invalid_or_oversized_camera_payload_never_publishes(self):
        self.periphery.dispatch('camera_left', 'start')
        for value in ('invalid-base64', 'x' * 1_500_001, base64.b64encode(b'notjpeg').decode()):
            self.periphery._on_image('camera_left', {'data': value}, 1)
            self.assertFalse(self.periphery.snapshot('camera_left')['valid'])

    def test_camera_preserves_source_timestamp_and_rejects_bad_header(self):
        self.periphery.dispatch('camera_left', 'start')
        payload = {'data': base64.b64encode(b'\xff\xd8test\xff\xd9').decode(),
                   'header': {'frame_id': '', 'stamp': {'secs': 123, 'nsecs': 456}}}
        self.periphery._on_image('camera_left', payload, 1)
        self.periphery.tick()
        stamp = self.periphery._image_pubs['camera_left'].messages[-1].header.stamp
        self.assertEqual((stamp.sec, stamp.nanosec), (123, 456))
        payload['header'] = []
        self.periphery._on_image('camera_left', payload, 1)
        self.assertFalse(self.periphery.snapshot('camera_left')['valid'])

    def test_background_start_cannot_start_actuator_cards(self):
        bundle = driver.KuavoBundle({'sensors': {'auto_start': ['arm', 'base_drive', 'switch_mode', 'battery', 'imu']}},
                                   'test', self.ros, FakeNode())
        try:
            with patch.object(bundle.startup_checks, 'start'), patch.object(bundle, 'dispatch') as dispatch:
                bundle.start_background()
                self.assertEqual([c.args[0] for c in dispatch.call_args_list], ['battery', 'imu'])
            self.assertEqual(self.ros.services, [])
            self.assertEqual(self.ros.published, [])
        finally:
            bundle.close()

    def test_head_camera_reads_jpeg_without_chassis_enable_service(self):
        self.periphery.dispatch('camera_head', 'start')
        self.periphery.tick()
        self.assertIn('/camera_head/color/image_rect/compressed', self.ros.subscriptions)
        self.assertEqual(self.ros.services, [])

    def calibrate_head(self):
        info = {'width': 640, 'height': 480, 'K': [370., 0., 320., 0., 370., 240., 0., 0., 1.],
                'D': [0.] * 8, 'distortion_model': 'rational_polynomial',
                'header': {'frame_id': 'camera_head_color_optical_frame'}}
        for card in ('camera_head', 'camera_depth'):
            self.periphery._on_camera_info(card, info)
        self.periphery._depth_registration = True

    def test_depth_publishes_rgb_registered_lossless_format_with_provenance(self):
        self.periphery.dispatch('camera_depth', 'start')
        self.calibrate_head()
        raw = struct.pack('<HH', 0, 1500) * (640*480//2)
        compressed = zlib.compress(raw, 1)
        self.periphery._on_image('camera_depth', {'format': 'image/depth-zlib',
            'data': base64.b64encode(compressed).decode(),
            'header': {'frame_id': 'camera_head_color_optical_frame', 'stamp': {'secs': 12, 'nsecs': 34}}}, 1)
        self.periphery.tick()
        msg = self.periphery._image_pubs['camera_depth'].messages[-1]
        self.assertEqual(msg.format, 'image/depth-zlib')
        self.assertEqual(zlib.decompress(bytes(msg.data)), raw)
        self.assertEqual(msg.header.frame_id, 'camera_head_color_optical_frame')
        self.assertEqual(msg.header.stamp.sec, 12)
        self.assertEqual(next(t for t in self.periphery.tools() if t['name']=='camera_depth')['topic_out'][0]['format'], 'image/depth-zlib')
        self.assertEqual(self.periphery.snapshot('camera_depth')['unit'], 'mm')
        self.assertTrue(self.periphery.snapshot('camera_depth')['aligned_to_rgb'])
        rgb = self.periphery.dispatch('camera_head', 'info')['camera_info'][0]
        depth = self.periphery.dispatch('camera_depth', 'info')['camera_info'][0]
        self.assertEqual((depth['id'], depth['K']), (rgb['id'], rgb['K']))
        self.assertEqual(self.periphery.dispatch('camera_depth', 'info')['state'], 'running')

    def test_depth_declares_legacy_navi_suffix_even_before_any_source_is_ready(self):
        tool = next(t for t in self.periphery.tools() if t['name']=='camera_depth')
        self.assertEqual(tool['topic_out'], [{'topic':'/test/state/camera_head/visual_depth',
                                             'format':'image/depth-zlib'}])
        info = self.periphery.dispatch('camera_depth', 'info')
        self.assertEqual(info['state'], 'idle')
        self.assertEqual(info['topic_out'], tool['topic_out'])
        self.assertEqual([d['topic'] for d in info['camera_info']],
                         ['/test/state/camera_head/visual_depth','/test/state/camera_depth'])

    def test_depth_alias_preserves_bytes_capture_header_and_lifecycle(self):
        self.periphery.dispatch('camera_depth', 'start')
        self.calibrate_head()
        data = zlib.compress(struct.pack('<H',1500)*(640*480))
        frame = {'format':'image/depth-zlib','data':base64.b64encode(data).decode(),
                 'header':{'frame_id':'camera_head_color_optical_frame',
                           'stamp':{'secs':123,'nsecs':456}}}
        self.periphery._on_image('camera_depth',frame,1)
        self.periphery.tick()
        primary = self.periphery._image_pubs['camera_depth'].messages
        legacy = self.periphery._legacy_depth_pub.messages
        self.assertEqual(len(primary),1)
        self.assertEqual(len(legacy),1)
        self.assertIs(primary[0],legacy[0])
        self.assertEqual(bytes(legacy[0].data),data)
        self.assertEqual((legacy[0].header.stamp.sec,legacy[0].header.stamp.nanosec),(123,456))
        declarations = self.periphery.dispatch('camera_depth','info')['camera_info']
        self.assertEqual(declarations[0]['K'],declarations[1]['K'])
        self.assertEqual(declarations[0]['id'],declarations[1]['id'])
        self.periphery.dispatch('camera_depth','stop')
        self.periphery._on_image('camera_depth',frame,1)
        self.periphery.tick()
        self.assertEqual(len(legacy),1)

    def test_start_with_fresh_depth_does_not_request_hardware_again(self):
        with patch.object(peripherals_module.threading, 'Thread'):
            self.periphery.dispatch('camera_depth', 'start')
        self.calibrate_head()
        raw = b'\x00\x00' * (640 * 480)
        self.periphery._on_image('camera_depth', {'format': 'image/depth-zlib',
            'data': base64.b64encode(zlib.compress(raw)).decode(),
            'header': {'frame_id': 'camera_head_color_optical_frame'}}, 1)
        self.periphery._enabling.clear()
        self.periphery._camera_ensure = lambda card: self.fail('fresh source was enabled again')
        with patch.object(peripherals_module.threading, 'Thread') as worker:
            self.periphery.dispatch('camera_depth', 'start')
            worker.assert_not_called()

    def test_native_or_misregistered_depth_is_not_published_as_aligned(self):
        self.periphery.dispatch('camera_depth', 'start')
        self.calibrate_head()
        payload = {'format': 'image/depth-zlib',
                   'data': base64.b64encode(zlib.compress(bytes(640*480*2))).decode(),
                   'header': {'frame_id': 'camera_head_depth_optical_frame'}}
        self.periphery._on_image('camera_depth', payload, 1)
        self.assertFalse(self.periphery.snapshot('camera_depth')['valid'])
        self.assertEqual(self.periphery.dispatch('camera_depth', 'info')['state'], 'loading')
        payload['header']['frame_id'] = 'camera_head_color_optical_frame'
        self.periphery._depth_registration = False
        self.periphery._on_image('camera_depth', payload, 1)
        self.assertFalse(self.periphery.snapshot('camera_depth')['valid'])

    def test_idle_camera_declares_unknown_optics_without_borrowing_head_calibration(self):
        self.calibrate_head()
        left = self.periphery.dispatch('camera_left', 'info')
        self.assertEqual(left['state'], 'idle')
        self.assertIsNone(left['camera_info'][0]['K'])
        self.assertIsNone(left['camera_info'][0]['half_fov_rad'])
        self.assertEqual(left['camera_info'][0]['source'], 'unknown')
        self.assertNotEqual(left['camera_info'][0]['id'],
                            self.periphery.dispatch('camera_head', 'info')['camera_info'][0]['id'])

    def test_reconnect_invalidates_camera_geometry_and_footprint(self):
        self.calibrate_head()
        self.periphery._footprint = {'half_width': .35}
        self.ros.connection = (True, self.ros.connection[1] + 1)
        self.periphery.tick()
        self.assertFalse(self.periphery._depth_aligned())
        self.assertIsNone(self.periphery.footprint())

    def test_depth_decoder_rejects_bad_size_trailing_data_and_expansion_bombs(self):
        self.periphery.dispatch('camera_depth', 'start')
        valid = zlib.compress(bytes(640*480*2), 1)
        for data in (b'not zlib', valid[:-1], valid+b'trailing', valid+valid,
                     zlib.compress(bytes(640*480*2+1)), zlib.compress(bytes(640*480*8)), zlib.compress(b'short')):
            self.periphery._on_image('camera_depth', {'format': 'image/depth-zlib',
                'data': base64.b64encode(data).decode()}, 1)
            self.assertFalse(self.periphery.snapshot('camera_depth')['valid'])

    def test_head_services_are_requested_on_start_and_not_for_idle_cards(self):
        called = []
        p = Peripherals(self.ros, FakeNode(), 'test', camera_ensure=called.append)
        try:
            p.tick()
            self.assertEqual(called, [])
            p.dispatch('camera_depth', 'start')
            self.assertTrue(wait_until(lambda: called == ['camera_depth']))
            p.dispatch('camera_depth', 'stop')
            p._retry_at.clear()
            p.tick()
            self.assertEqual(called, ['camera_depth'])
            self.assertEqual(self.ros.published, [])
        finally:
            p.close()

    def test_cameras_use_independent_clients_and_publish_latest_without_blocking_each_other(self):
        clients = []
        class Client(FakeRos1):
            def start(self):
                self.started = True
            def close(self):
                self.closed = True
        def factory():
            client = Client()
            clients.append(client)
            return client
        p = Peripherals(self.ros, FakeNode(), 'test', media_factory=factory)
        entered, release = threading.Event(), threading.Event()
        left = p._image_pubs['camera_left']
        publish = left.publish
        def blocked(msg):
            entered.set()
            release.wait(2)
            publish(msg)
        left.publish = blocked
        payload = lambda data: {'data': base64.b64encode(b'\xff\xd8'+data+b'\xff\xd9').decode()}
        try:
            p.start()
            for card in ('camera_left', 'camera_right'):
                p.dispatch(card, 'start')
            p._on_image('camera_left', payload(b'first'), 1)
            self.assertTrue(entered.wait(1))
            for i in range(10):
                p._on_image('camera_left', payload(str(i).encode()), 1)
            p._on_image('camera_right', payload(b'right'), 1)
            self.assertTrue(wait_until(lambda: len(p._image_pubs['camera_right'].messages) == 1))
            self.assertNotIn('/camera_1/color/image_raw/compressed', self.ros.subscriptions)
            self.assertTrue(all(c.started for c in clients))
            release.set()
            self.assertTrue(wait_until(lambda: len(left.messages) == 2))
            self.assertEqual(bytes(left.messages[-1].data), b'\xff\xd89\xff\xd9')
        finally:
            release.set()
            p.close()
        self.assertTrue(all(c.closed for c in clients))


class HostManagerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('kuavo_host_manager', ROOT / 'host/runtime_manager.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def request(self, host='127.0.0.1:15748', origin=None, body=b'{}', peer='127.0.0.1', path='/ensure'):
        head = [f'POST {path} HTTP/1.1', f'Host: {host}', 'Content-Type: application/json', f'Content-Length: {len(body)}']
        if origin:
            head.append('Origin: ' + origin)
        wire = '\r\n'.join(head).encode() + b'\r\n\r\n' + body
        class Connection:
            output = bytearray()
            def settimeout(self, value):
                pass
            def makefile(self, *args):
                return io.BytesIO(wire)
            def sendall(self, value):
                self.output.extend(value)
        connection = Connection()
        self.module.Handler(connection, (peer, 1000), types.SimpleNamespace())
        return int(bytes(connection.output).split(b' ')[1])

    def test_browser_remote_and_arbitrary_commands_are_rejected_before_start(self):
        with patch.object(self.module, 'ensure_services') as ensure:
            for args in ({'origin': 'https://evil.example'}, {'host': 'evil.example:15748'},
                         {'peer': '192.0.2.1'}, {'body': b'{"command":"anything"}'}):
                self.assertEqual(self.request(**args), 403)
            ensure.assert_not_called()

    def test_only_loopback_empty_ensure_can_start_fixed_services(self):
        with patch.object(self.module, 'ensure_services', return_value={'requested': True}) as ensure:
            self.assertEqual(self.request(), 200)
            ensure.assert_called_once_with()

    def test_manager_only_starts_fixed_telemetry_units(self):
        with patch.object(self.module.subprocess, 'run', return_value=types.SimpleNamespace(returncode=0)) as run:
            with patch.object(self.module.socket, 'create_connection'):
                self.assertTrue(self.module.ensure_services()['requested'])
        self.assertEqual(run.call_args_list[0].args[0], ['systemctl', 'start', '--no-block',
                         'kuavo5w-rosbridge.service', 'kuavo5w-battery-relay.service'])

    def test_manager_can_only_add_fixed_installed_head_camera_unit(self):
        with patch.object(self.module.subprocess, 'run', return_value=types.SimpleNamespace(returncode=0)) as run:
            with patch.object(self.module.socket, 'create_connection'), patch.object(self.module.Path, 'is_file', return_value=True):
                self.module.ensure_services('head')
        self.assertEqual(run.call_args_list[0].args[0][-3:], ['kuavo5w-head-camera.service', 'kuavo5w-head-jpeg-relay.service', 'kuavo5w-head-imu-relay.service'])

    def test_depth_scope_uses_only_its_fixed_services_and_rejects_arbitrary_paths(self):
        with patch.object(self.module.subprocess, 'run', return_value=types.SimpleNamespace(returncode=0)) as run:
            with patch.object(self.module.socket, 'create_connection'), patch.object(self.module.Path, 'is_file', return_value=True):
                self.module.ensure_services('depth')
        self.assertEqual(run.call_args_list[0].args[0], ['systemctl', 'start', '--no-block',
            'kuavo5w-rosbridge.service', 'kuavo5w-head-camera.service', 'kuavo5w-head-jpeg-relay.service', 'kuavo5w-head-imu-relay.service'])
        with patch.object(self.module, 'ensure_services', return_value={'requested': True}) as ensure:
            self.assertEqual(self.request(path='/ensure/depth'), 200)
            ensure.assert_called_once_with('depth')
            self.assertEqual(self.request(path='/ensure/enable_control'), 403)

    def test_missing_camera_installation_does_not_start_partial_services(self):
        with patch.object(self.module.Path, 'is_file', return_value=False), patch.object(self.module.subprocess, 'run') as run:
            result = self.module.ensure_services('depth')
            self.assertFalse(result['requested'])
            run.assert_not_called()

    def test_bridge_tcp_latency_fix_keeps_all_odometry_samples_and_is_process_local(self):
        spec = importlib.util.spec_from_file_location('kuavo_loopback', ROOT / 'host/rosbridge_loopback.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        calls = []
        original = lambda *args, **kwargs: calls.append((args, kwargs))
        rospy = types.SimpleNamespace(Subscriber=original)
        subscribers = types.SimpleNamespace(Subscriber=original)
        module.configure_subscribers(rospy, subscribers)
        subscribers.Subscriber('/odom', 'nav_msgs/Odometry', 'callback')
        self.assertIs(rospy.Subscriber, original)
        self.assertEqual(calls, [(('/odom', 'nav_msgs/Odometry', 'callback'),
                                 {'tcp_nodelay': True, 'buff_size': 2_000_000})])
        self.assertNotIn('queue_size', calls[0][1])
        module._listen = lambda *args, **kwargs: kwargs
        self.assertEqual(module.listen_loopback(None, interface='0.0.0.0'), {'interface': '127.0.0.1'})

    def test_controller_messages_override_is_optional_and_process_local(self):
        spec = importlib.util.spec_from_file_location('kuavo_message_overlay', ROOT / 'host/rosbridge_loopback.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        original = list(sys.path)
        with tempfile.TemporaryDirectory() as root, patch.object(sys, 'path', original.copy()):
            self.assertIsNone(module.configure_messages(root))
            self.assertEqual(sys.path, original)
            overlay = Path(root) / 'ros1_messages'
            for part in ('msg', 'srv'):
                package = overlay / 'kuavo_msgs' / part
                package.mkdir(parents=True)
                (package / '__init__.py').write_text('')
                if part == 'msg':
                    self.assertIsNone(module.configure_messages(root))
            self.assertEqual(module.configure_messages(root), str(overlay))
            self.assertEqual(sys.path, [str(overlay), *original])
        launch = (ROOT / 'host/rosbridge.launch').read_text()
        self.assertEqual(launch.count('launch-prefix="/usr/bin/python3 /opt/phanthy-motus/kuavo5w-host/rosbridge_loopback.py"'), 2)


if __name__ == '__main__':
    unittest.main()
