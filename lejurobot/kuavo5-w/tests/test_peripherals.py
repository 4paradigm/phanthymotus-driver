import base64
import importlib.util
import io
import json
import math
from pathlib import Path
import time
import types
import unittest
from unittest.mock import patch

from test_driver import driver, FakeRos1, FakeNode, ROOT, peripherals_module
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
        self.assertEqual(sample['soc'], 96.5)
        self.assertIsNone(sample['voltage'])
        self.assertEqual(sample['current'], -3.2)
        self.assertEqual(sample['sources']['/battery']['status'], 4)
        self.assertEqual(sample['sources']['/jaten_behavior_manager/out/leju/battery']['status'], 2)

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
        self.assertIsNone(result['voltage'])

    def test_laser_null_ranges_and_points_use_actual_laser_frame(self):
        self.periphery.dispatch('laser_scan', 'start')
        self.periphery._on_scan({'header': {'frame_id': 'laser'}, 'angle_min': 0.,
            'angle_increment': math.pi/2, 'range_min': .05, 'range_max': 50.,
            'ranges': [1., 2., math.inf, math.nan, 0.]})
        self.periphery.tick()
        raw = json.loads(self.periphery._pubs['laser_scan'].messages[-1].data)
        self.assertEqual(raw['frame_id'], 'laser')
        self.assertEqual(raw['ranges'], [1., 2., None, None, None])
        points = json.loads(self.periphery._points_pub.messages[-1].data)
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[0]['x'], 1.)
        self.assertAlmostEqual(points[1]['y'], 2.)

    def test_bad_laser_invalidates_previous_measurement(self):
        self.periphery._on_scan({'ranges': [1.]})
        self.assertFalse(self.periphery.snapshot('laser_scan')['valid'])

    def test_camera_publishes_native_compressed_image_once_per_received_frame(self):
        self.periphery.dispatch('camera_rgb', 'start')
        payload = {'data': base64.b64encode(b'\xff\xd8test\xff\xd9').decode(), 'header': {'frame_id': 'camera'}}
        self.ros.subscriptions['/camera_1/color/image_raw/compressed'][1](payload)
        self.periphery.tick()
        self.periphery.tick()
        messages = self.periphery._image_pubs['camera_rgb'].messages
        self.assertEqual(len(messages), 1)
        self.assertEqual(messages[0].data, b'\xff\xd8test\xff\xd9')
        self.assertEqual(messages[0].format, 'jpeg')
        self.assertEqual(messages[0].header.frame_id, 'camera')

    def test_camera_stop_drops_late_frames_without_disabling_shared_hardware(self):
        self.periphery.dispatch('camera_rgb', 'start')
        callback = self.ros.subscriptions['/camera_1/color/image_raw/compressed'][1]
        self.periphery.dispatch('camera_rgb', 'stop')
        callback({'data': base64.b64encode(b'\xff\xd8x\xff\xd9').decode()})
        self.periphery.tick()
        self.assertFalse(self.periphery.snapshot('camera_rgb')['valid'])
        self.assertEqual(self.ros.services, [])

    def test_camera_reconnect_rebinds_and_only_invokes_camera_enable(self):
        self.periphery.dispatch('camera_rgb', 'start')
        self.ros.connection = (True, 2)
        with patch.object(peripherals_module.threading, 'Thread') as thread:
            self.periphery.tick()
            target = thread.call_args.kwargs['target']
            args = thread.call_args.kwargs['args']
        target(*args)
        self.assertEqual(self.ros.services[0][:3],
            ('/camera_left/set_camera_enabled', 'std_srvs/SetBool', {'data': True}))
        self.assertEqual(self.ros.published, [])

    def test_invalid_or_oversized_camera_payload_never_publishes(self):
        self.periphery.dispatch('camera_rgb', 'start')
        for value in ('invalid-base64', 'x' * 1_500_001, base64.b64encode(b'notjpeg').decode()):
            self.periphery._on_image('camera_rgb', {'data': value}, 1)
            self.assertFalse(self.periphery.snapshot('camera_rgb')['valid'])

    def test_camera_preserves_source_timestamp_and_rejects_bad_header(self):
        self.periphery.dispatch('camera_rgb', 'start')
        payload = {'data': base64.b64encode(b'\xff\xd8test\xff\xd9').decode(),
                   'header': {'frame_id': '', 'stamp': {'secs': 123, 'nsecs': 456}}}
        self.periphery._on_image('camera_rgb', payload, 1)
        self.periphery.tick()
        stamp = self.periphery._image_pubs['camera_rgb'].messages[-1].header.stamp
        self.assertEqual((stamp.sec, stamp.nanosec), (123, 456))
        payload['header'] = []
        self.periphery._on_image('camera_rgb', payload, 1)
        self.assertFalse(self.periphery.snapshot('camera_rgb')['valid'])

    def test_background_start_cannot_start_actuator_cards(self):
        bundle = driver.KuavoBundle({'sensors': {'auto_start': ['arm', 'loco', 'switch_mode', 'battery', 'imu']}},
                                   'test', self.ros, FakeNode())
        try:
            with patch.object(bundle.startup_checks, 'start'), patch.object(bundle, 'dispatch') as dispatch:
                bundle.start_background()
                self.assertEqual([c.args[0] for c in dispatch.call_args_list], ['battery', 'imu'])
            self.assertEqual(self.ros.services, [])
            self.assertEqual(self.ros.published, [])
        finally:
            bundle.close()


class HostManagerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location('kuavo_host_manager', ROOT / 'host/runtime_manager.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def request(self, host='127.0.0.1:15748', origin=None, body=b'{}', peer='127.0.0.1'):
        head = [f'POST /ensure HTTP/1.1', f'Host: {host}', 'Content-Type: application/json', f'Content-Length: {len(body)}']
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
        self.assertEqual(run.call_args.args[0], ['systemctl', 'start', '--no-block',
                         'kuavo5w-rosbridge.service', 'kuavo5w-battery-relay.service'])


if __name__ == '__main__':
    unittest.main()
