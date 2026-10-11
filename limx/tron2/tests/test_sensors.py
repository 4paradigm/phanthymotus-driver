"""Offline sensor contracts and real loopback HTTP/WebSocket transport."""
import ast
import asyncio
import copy
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import patch, MagicMock

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))

import aiohttp
from aiohttp import web
import yaml
from device import Adapter, TOPICS, VERSION
from vendor import RobotReader
import state
import ros2


def settings(**updates):
    return {'serial': 'TEST_TRON2A', 'url': 'ws://127.0.0.1:5000',
            'poll_interval_seconds': .5, 'response_timeout_seconds': 5,
            'stale_after_seconds': 5, 'enabled_cards': list(TOPICS), **updates}


def info(serial='TEST_TRON2A'):
    return {'title': 'notify_robot_info', 'accid': serial,
            'data': {'imu': 'OK', 'motor': 'OK', 'camera': 'UNKNOW',
                     'working_mode': 'controller_mode', 'battery': '85',
                     'bms_voltage': '52650', 'bms_temp1': '94'}}


def joints(**updates):
    return {'result': 'success', 'names': [], 'q': [.1]*16,
            'dq': [0]*16, 'tau': [0]*16, 'timestamp': 1234, **updates}


class StateTests(unittest.TestCase):
    def test_index_labels_and_degree_display_do_not_invent_names(self):
        result = state.joint_state(joints())
        self.assertEqual(result['joint_count'], 16)
        self.assertEqual(result['names'], [])
        self.assertEqual(result['labels'][0], 'joint_index_0')
        self.assertTrue(result['labels_are_indices'])
        self.assertAlmostEqual(result['positions_deg'][0], math.degrees(.1))
        self.assertFalse(result['measurement_time_verified'])

    def test_vendor_names_are_preserved_when_complete(self):
        names = ['vendor_joint_' + str(i) for i in range(16)]
        result = state.joint_state(joints(names=names))
        self.assertEqual(result['labels'], names)
        self.assertFalse(result['labels_are_indices'])

    def test_missing_optional_arrays_and_measurement_timestamp_are_accepted(self):
        result = state.joint_state({'result': 'success', 'q': [0, .1]})
        self.assertEqual(result['dq'], [])
        self.assertEqual(result['tau'], [])
        self.assertIsNone(result['robot_timestamp'])

    def test_empty_nonfinite_mismatched_and_ambiguous_names_are_rejected(self):
        cases = [joints(q=[]), joints(q=[math.nan]*16), joints(q=[True]*16),
                 joints(dq=[0]), joints(tau='bad'), joints(names=['same']*16),
                 joints(names=['only-one']), joints(timestamp=math.inf),
                 joints(result='failure'), joints(q=[0]*65)]
        for data in cases:
            with self.subTest(data=data), self.assertRaises(ValueError):
                state.joint_state(data)

    def test_status_keeps_unknown_values_and_does_not_require_idle(self):
        data = state.select_fields(info()['data'], state.STATUS_FIELDS)
        self.assertEqual(data['camera'], 'UNKNOW')
        self.assertEqual(data['working_mode'], 'controller_mode')
        self.assertIsNone(data['sw_version'])

    def test_battery_scaling_is_not_invented(self):
        data = state.battery_state(info()['data'])
        self.assertEqual(data['battery_percent'], 85)
        self.assertEqual(data['raw']['bms_voltage'], '52650')
        self.assertEqual(data['raw']['bms_temp1'], '94')
        self.assertEqual(data['bms_units'], 'vendor_raw_unverified')
        for raw in ('UNKNOW', '101', '-1', None, True, 'nan'):
            with self.subTest(raw=raw):
                self.assertIsNone(state.battery_state({'battery': raw})['battery_percent'])


class CacheTests(unittest.TestCase):
    def setUp(self):
        self.reader = RobotReader(settings())

    def test_disconnected_and_unavailable_samples_do_not_show_values(self):
        self.assertEqual(self.reader.sample('joint_state')['sample_status'], 'offline')
        self.reader.accept_info(info())
        joint = self.reader.sample('joint_state')
        self.assertFalse(joint['available'])
        self.assertEqual(joint['sample_status'], 'unavailable')
        self.assertIsNone(joint['data'])

    def test_reception_age_does_not_claim_measurement_age(self):
        self.reader.accept_info(info())
        self.reader.joints = state.joint_state(joints())
        self.reader.joints_at, self.reader.joints_received = time.monotonic(), time.time()
        packet = self.reader.sample('joint_state')
        self.assertTrue(packet['available'])
        self.assertEqual(packet['sample_status'], 'recently_received')
        self.assertFalse(packet['measurement_time_verified'])
        self.assertNotIn('joint_age_seconds', packet)
        self.assertNotIn('measurement_age_seconds', packet)
        self.reader.joints_at -= 10
        stale = self.reader.sample('joint_state')
        self.assertEqual(stale['sample_status'], 'stale')
        self.assertIsNone(stale['data'])
        self.assertTrue(self.reader.sample('robot_status')['available'])

    def test_disconnect_clears_all_cached_values(self):
        self.reader.accept_info(info())
        self.reader.joints = state.joint_state(joints())
        self.reader.joints_at, self.reader.joints_received = time.monotonic(), time.time()
        self.reader.disconnect('Disconnected')
        for card in TOPICS:
            self.assertFalse(self.reader.sample(card)['available'])
            self.assertIsNone(self.reader.sample(card)['data'])
        self.assertIsNone(self.reader.info)
        self.assertIsNone(self.reader.joints)

    def test_identity_lock_and_notification_consistency(self):
        with self.assertRaises(ValueError):
            self.reader.accept_info(info('OTHER'))
        bad = info()
        bad['data']['accid'] = 'OTHER'
        with self.assertRaises(ValueError):
            self.reader.accept_info(bad)
        self.assertFalse(self.reader.connected)

    def test_auto_identity_is_bound_once_per_process(self):
        reader = RobotReader(settings(serial=''))
        reader.accept_info(info())
        self.assertEqual(reader.serial, 'TEST_TRON2A')
        reader.disconnect('Disconnected')
        with self.assertRaises(ValueError):
            reader.accept_info(info('OTHER'))


class TransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_guid_correlation_and_query_only_wire_payload(self):
        reader = RobotReader(settings())
        reader.accept_info(info())
        queue, sent = [], []
        class WS:
            async def send_json(self, body):
                sent.append(body)
                response = {'title': 'response_get_joint_state', 'accid': 'TEST_TRON2A'}
                queue.extend([{**response, 'guid': 'wrong', 'data': joints(q=[9]*16)},
                    {**response, 'guid': body['guid'], 'data': joints()}])
            async def receive(self):
                return types.SimpleNamespace(type=aiohttp.WSMsgType.TEXT,
                                             data=json.dumps(queue.pop(0)))
        await reader.read_joints(WS())
        self.assertEqual(reader.joints['q'], [.1]*16)
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0]['title'], 'request_get_joint_state')
        self.assertEqual(sent[0]['data'], {})
        self.assertEqual(sent[0]['accid'], 'TEST_TRON2A')

    async def test_response_serial_mismatch_is_not_cached(self):
        reader = RobotReader(settings())
        reader.accept_info(info())
        class WS:
            async def send_json(self, body): self.guid = body['guid']
            async def receive(self):
                return types.SimpleNamespace(type=aiohttp.WSMsgType.TEXT,
                    data=json.dumps({'title': 'response_get_joint_state', 'guid': self.guid,
                                     'accid': 'OTHER', 'data': joints()}))
        with self.assertRaises(ValueError):
            await reader.read_joints(WS())
        self.assertIsNone(reader.joints)

    async def test_closed_and_nonfinite_wire_data_are_rejected(self):
        reader = RobotReader(settings())
        class WS:
            async def receive(self): return message
        message = types.SimpleNamespace(type=aiohttp.WSMsgType.CLOSED)
        with self.assertRaises(ConnectionError):
            await reader.receive(WS(), time.monotonic()+1)
        message = types.SimpleNamespace(type=aiohttp.WSMsgType.TEXT, data='{"bad":NaN}')
        with self.assertRaises(ValueError):
            await reader.receive(WS(), time.monotonic()+1)

    async def test_read_timeout_is_bounded(self):
        reader = RobotReader(settings(response_timeout_seconds=.03))
        reader.accept_info(info())
        class WS:
            async def send_json(self, body): pass
            async def receive(self): await asyncio.sleep(10)
        with self.assertRaises(asyncio.TimeoutError):
            await reader.read_joints(WS())

    async def test_real_websocket_poll_disconnect_and_reconnect(self):
        requests, connections = [], []
        second = asyncio.Event()
        async def robot(request):
            ws = web.WebSocketResponse()
            await ws.prepare(request)
            connections.append(ws)
            await ws.send_json(info())
            async for message in ws:
                body = json.loads(message.data)
                requests.append(body)
                await ws.send_json({'title': 'response_get_joint_state',
                    'accid': 'TEST_TRON2A', 'guid': body['guid'], 'data': joints()})
                if len(connections) == 1:
                    await ws.close()
                    break
                second.set()
            return ws
        app = web.Application()
        app.router.add_get('/robot', robot)
        runner = web.AppRunner(app)
        await runner.setup()
        server = await asyncio.get_running_loop().create_server(runner.server, '127.0.0.1', 0)
        port = server.sockets[0].getsockname()[1]
        reader = RobotReader(settings(url='ws://127.0.0.1:' + str(port) + '/robot',
                                      poll_interval_seconds=.02))
        try:
            async with aiohttp.ClientSession(trust_env=False) as session:
                task = asyncio.create_task(reader.run(session))
                try:
                    await asyncio.wait_for(second.wait(), 4)
                    for _ in range(30):
                        if reader.sample('joint_state')['available']: break
                        await asyncio.sleep(.01)
                    self.assertTrue(reader.sample('joint_state')['available'])
                    self.assertGreaterEqual(len(connections), 2)
                    self.assertEqual({p['title'] for p in requests}, {'request_get_joint_state'})
                    self.assertTrue(all(p['data'] == {} for p in requests))
                finally:
                    task.cancel()
                    await asyncio.gather(task, return_exceptions=True)
                    self.assertFalse(reader.connected)
                    self.assertIsNone(reader.sample('joint_state')['data'])
        finally:
            server.close()
            await server.wait_closed()
            await runner.cleanup()


class MCPTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.published = []
        self.adapter = Adapter(lambda topic, data: self.published.append((topic, data)), settings())
        self.adapter.reader.accept_info(info())
        self.adapter.reader.joints = state.joint_state(joints())
        self.adapter.reader.joints_at = time.monotonic()
        self.adapter.reader.joints_received = time.time()
        app = web.Application()
        app.router.add_post('/mcp', self.adapter.rpc)
        app.router.add_get('/health', self.adapter.health)
        self.runner = web.AppRunner(app)
        await self.runner.setup()
        self.server = await asyncio.get_running_loop().create_server(self.runner.server, '127.0.0.1', 0)
        self.url = 'http://127.0.0.1:' + str(self.server.sockets[0].getsockname()[1])
        self.session = aiohttp.ClientSession(trust_env=False)

    async def asyncTearDown(self):
        await self.session.close()
        self.server.close()
        await self.server.wait_closed()
        await self.runner.cleanup()

    async def rpc(self, method, params=None):
        async with self.session.post(self.url+'/mcp', json={
                'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params or {}}) as response:
            self.assertEqual(response.status, 200)
            return await response.json()

    async def call(self, name, **args):
        reply = await self.rpc('tools/call', {'name': name, 'arguments': args})
        result = reply['result']
        return result['isError'], json.loads(result['content'][0]['text'])

    async def test_handshake_and_three_sensors_plus_resource(self):
        reply = await self.rpc('initialize')
        self.assertEqual(reply['result']['serverInfo']['version'], VERSION)
        tools = (await self.rpc('tools/list'))['result']['tools']
        self.assertEqual({t['name'] for t in tools}, set(TOPICS) | {'read_state'})
        self.assertEqual({t['type'] for t in tools}, {'sensor', 'resource'})
        for tool in tools:
            if tool['type'] == 'sensor':
                self.assertEqual(tool['topic_out'][0]['topic'], TOPICS[tool['name']])
                self.assertEqual(tool['topic_out'][0]['format'], 'data/json')

    async def test_canvas_lifecycle_and_cache_read_have_no_robot_write_path(self):
        for name in TOPICS:
            for action in ('start', 'info', 'stop'):
                error, result = await self.call(name, action=action, instance_id='canvas-card')
                self.assertFalse(error)
                self.assertTrue(result['read_only'])
                self.assertTrue(result['always_on'])
        error, result = await self.call('read_state')
        self.assertFalse(error)
        self.assertTrue(result['samples']['joint_state']['available'])
        self.assertEqual(result['samples']['joint_state']['data']['joint_count'], 16)

    async def test_reset_execute_custom_angles_and_bad_metadata_are_rejected(self):
        for name, args in [('reset_pose', {'action': 'execute'}),
                ('joint_state', {'action': 'execute'}),
                ('joint_state', {'action': 'info', 'target_q': [0]*14}),
                ('read_state', {'action': 'move'}),
                ('joint_state', {'action': 'info', 'instance_id': 'bad\ncard'})]:
            with self.subTest(name=name, args=args):
                error, result = await self.call(name, **args)
                self.assertTrue(error)
                self.assertTrue(result['read_only'])

    async def test_periodic_publication_and_disconnect_clear_packets(self):
        task = asyncio.create_task(self.adapter.publish_loop())
        try:
            await asyncio.sleep(.02)
            self.assertEqual({t for t,d in self.published}, set(TOPICS.values()))
            self.assertTrue(all(d['available'] for t,d in self.published))
            self.published.clear()
            self.adapter.reader.disconnect('Disconnected')
            await asyncio.sleep(.51)
            self.assertTrue(self.published)
            self.assertTrue(all(d['data'] is None and not d['available'] for t,d in self.published))
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)

    async def test_health_reports_failure_after_disconnect(self):
        self.adapter.reader.disconnect('Disconnected')
        async with self.session.get(self.url+'/health') as response:
            self.assertEqual(response.status, 503)
            self.assertFalse((await response.json())['ready'])

    async def test_disabled_card_is_not_discovered(self):
        adapter = Adapter(lambda *args: None, settings(enabled_cards=['joint_state']))
        self.assertEqual({t['name'] for t in adapter.tools()}, {'joint_state', 'read_state'})
        with self.assertRaises(ValueError):
            adapter.call('battery_state', {'action': 'start'})

    async def test_json_rpc_notification_and_parse_errors(self):
        async with self.session.post(self.url+'/mcp', json={
                'jsonrpc': '2.0', 'method': 'notifications/initialized'}) as response:
            self.assertEqual(response.status, 204)
        async with self.session.post(self.url+'/mcp', data='bad-json') as response:
            self.assertEqual((await response.json())['error']['code'], -32700)


class PackagingTests(unittest.TestCase):
    def function(self, name):
        source = ast.parse((DRIVER/'main.py').read_text())
        node = next(n for n in source.body if isinstance(n, ast.FunctionDef) and n.name == name)
        from urllib.parse import urlparse
        import re
        namespace = {'Path': Path, 'yaml': yaml, 'os': os, 'math': math,
                     're': re, 'urlparse': urlparse, '__file__': str(DRIVER/'main.py')}
        exec(compile(ast.Module(body=[node], type_ignores=[]), 'main.py', 'exec'), namespace)
        return namespace[name]

    def test_work_image_path_does_not_index_shallow_parents(self):
        with patch.object(Path, 'is_file', autospec=True,
                          side_effect=lambda p: str(p) == '/work/common/logsafe.py'):
            self.assertEqual(self.function('common_root')('/work/main.py'), Path('/work'))

    def test_staged_entry_imports_without_repository_layout(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for name in ('main.py','device.py','vendor.py','state.py','ros2.py','config.yaml'):
                shutil.copyfile(DRIVER/name, root/name)
            shutil.copytree(DRIVER.parents[1]/'common', root/'common')
            script = ('from pathlib import Path; import main, device, vendor, state, ros2, common; '
                'assert device.VERSION == "0.4.0"; assert Path(common.__file__).parent.resolve() == Path('+
                repr(str(root/'common'))+').resolve()')
            result = subprocess.run([sys.executable,'-c',script], cwd=temp,
                env={**os.environ,'PYTHONPATH':temp}, text=True, capture_output=True, timeout=8)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_only_read_query_exists_in_wire_send_ast(self):
        source = ast.parse((DRIVER/'vendor.py').read_text())
        calls = [n for n in ast.walk(source) if isinstance(n,ast.Call)
                 and isinstance(n.func,ast.Attribute) and n.func.attr == 'send_json']
        self.assertEqual(len(calls),1)
        payload = calls[0].args[0]
        fields = {k.value:v for k,v in zip(payload.keys,payload.values)}
        self.assertEqual(fields['title'].value, 'request_get_joint_state')
        self.assertEqual(fields['data'].keys, [])
        for filename in ('motion.py','pose.py','control.py','record.py','teach.py'):
            self.assertFalse((DRIVER/filename).exists())

    def test_config_defaults_and_invalid_poll_or_write_plugin(self):
        load = self.function('load_settings')
        with patch.dict(os.environ, {'TRON2_SERIAL':'','TRON2_ROBOT_URL':'ws://127.0.0.1:5000'}):
            config = yaml.safe_load((DRIVER/'config.yaml').read_text())
            with tempfile.TemporaryDirectory() as temp:
                path = Path(temp)/'config.yaml'
                for key,value in [('poll_interval_seconds',False),('poll_interval_seconds',math.nan),
                                  ('poll_interval_seconds',.001),('response_timeout_seconds',0)]:
                    changed = copy.deepcopy(config)
                    changed['robot'][key] = value
                    path.write_text(yaml.safe_dump(changed))
                    with patch.dict(os.environ,CONFIG_PATH=str(path)), self.assertRaises(ValueError): load()
                changed = copy.deepcopy(config)
                changed['plugins']['reset_pose'] = {'enabled':True}
                path.write_text(yaml.safe_dump(changed))
                with patch.dict(os.environ,CONFIG_PATH=str(path)), self.assertRaises(ValueError): load()
                path.write_text(yaml.safe_dump(config))
                with patch.dict(os.environ,CONFIG_PATH=str(path)):
                    self.assertEqual(set(load()['enabled_cards']),set(TOPICS))

    def test_missing_dds_profile_refuses_participant_creation(self):
        with patch.dict(os.environ,FASTRTPS_DEFAULT_PROFILES_FILE='/missing/tron2-dds.xml'):
            with self.assertRaisesRegex(ValueError,'DDS profile'):
                ros2.JsonPublisher(TOPICS.values())

    def test_ros2_publisher_serializes_each_topic_and_closes_participant(self):
        rclpy = types.ModuleType('rclpy')
        rclpy.init = MagicMock()
        rclpy.ok = MagicMock(return_value=True)
        rclpy.shutdown = MagicMock()
        node = MagicMock()
        sinks = {topic: MagicMock() for topic in TOPICS.values()}
        node.create_publisher.side_effect = lambda message, topic, qos: sinks[topic]
        rclpy.create_node = MagicMock(return_value=node)
        qos = types.ModuleType('rclpy.qos')
        qos.QoSProfile = MagicMock()
        qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT='best_effort')
        qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST='keep_last')
        qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE='volatile')
        messages = types.ModuleType('std_msgs.msg')
        messages.String = type('String', (), {})
        modules = {'rclpy': rclpy, 'rclpy.qos': qos,
                   'std_msgs': types.ModuleType('std_msgs'), 'std_msgs.msg': messages}
        with tempfile.TemporaryDirectory() as temp:
            profile = Path(temp)/'dds.xml'
            profile.write_text('<profiles/>')
            with patch.dict(sys.modules, modules), patch.dict(os.environ, {
                    'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile), 'ROS_DOMAIN_ID': '42',
                    'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp', 'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'}):
                publisher = ros2.JsonPublisher(TOPICS.values())
                self.assertNotIn('FASTDDS_BUILTIN_TRANSPORTS', os.environ)
                qos.QoSProfile.assert_called_once_with(depth=5, reliability='best_effort',
                    history='keep_last', durability='volatile')
                for topic in TOPICS.values():
                    publisher.publish(topic, {'available': False, 'data': None})
                    message = sinks[topic].publish.call_args[0][0]
                    self.assertEqual(json.loads(message.data), {'available': False, 'data': None})
                publisher.close()
                node.destroy_node.assert_called_once()
                rclpy.shutdown.assert_called_once()

    def test_service_has_loopback_dds_profile_and_no_hardware_write_mounts(self):
        fragment = yaml.safe_load((DRIVER/'deploy/service.yml').read_text())['limx-tron2']
        self.assertIn('/opt/phanthy-motus/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro',fragment['volumes'])
        self.assertEqual(fragment['restart'],'always')
        self.assertNotIn('privileged',fragment)
        self.assertFalse(any('/dev' in v or 'docker.sock' in v or 'runstate' in v for v in fragment['volumes']))


if __name__ == '__main__':
    unittest.main()
