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
from unittest.mock import patch, MagicMock, AsyncMock

DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))

import aiohttp
from aiohttp import web
import yaml
from device import Adapter, TOPICS, VERSION
from vendor import RobotReader
import state
import ros2
import skeleton
from reset import ARM_NAMES


def settings(**updates):
    return {'serial': 'TEST_TRON2A', 'url': 'ws://127.0.0.1:5000',
            'poll_interval_seconds': .5, 'response_timeout_seconds': 5,
            'stale_after_seconds': 5, 'enabled_cards': list(TOPICS), **updates}


def info(serial='TEST_TRON2A'):
    return {'title': 'notify_robot_info', 'accid': serial,
            'data': {'imu': 'OK', 'motor': 'OK', 'camera': 'UNKNOW',
                     'working_mode': 'developer_mode', 'working_mode_valid': True,
                     'collision_enable': True, 'collision_enable_valid': True,
                     'compliant_enable': False, 'compliant_enable_valid': True}}


def joints(**updates):
    return {'result': 'success', 'names': [], 'q': [.1]*16,
            'dq': [0]*16, 'tau': [0]*16, 'timestamp': 1234, **updates}


class StateTests(unittest.TestCase):
    def test_skeleton_motor_order_matches_official_model_including_head(self):
        import xml.etree.ElementTree as ET
        root = ET.fromstring(skeleton.model()['urdf'])
        moving = {joint.get('name') for joint in root.findall('joint') if joint.get('type') != 'fixed'}
        self.assertEqual(moving, set(skeleton.JOINT_NAMES))
        self.assertEqual(skeleton.JOINT_NAMES[14:], ['head_pitch_Joint', 'head_yaw_Joint'])
        self.assertFalse(root.findall('.//mesh'))
        sample = {'connected': True, 'available': True, 'sample_status': 'recently_received',
                  'received_at': 1, 'reception_age_seconds': .1, 'measurement_time_verified': False,
                  'data': state.joint_state(joints(q=[i/100 for i in range(16)]))}
        packet = skeleton.skeleton_stream(sample)
        self.assertEqual([j['name'] for j in packet['joints']], skeleton.JOINT_NAMES)
        self.assertEqual(packet['joints'][14]['q'], .14)
        self.assertEqual(packet['joints'][15]['q'], .15)
        self.assertEqual(packet['joints'][3]['label'], '肘')
        self.assertEqual(packet['joints'][3]['group'], '左臂')
        self.assertEqual(packet['joints'][10]['group'], '右臂')
        self.assertEqual(packet['joints'][14]['group'], '头部')
        self.assertAlmostEqual(packet['joints'][3]['q_deg'], math.degrees(.03))
        self.assertNotIn('imu_quat', packet)
        sample['data'] = state.joint_state(joints(names=list(reversed(skeleton.JOINT_NAMES))))
        packet = skeleton.skeleton_stream(sample)
        self.assertEqual(packet['joints'][0]['name'], 'head_yaw_Joint')
        self.assertEqual(packet['joints'][0]['idx'], 15)
        sample['data'] = state.joint_state(joints(names=['unknown_'+str(i) for i in range(16)]))
        packet = skeleton.skeleton_stream(sample)
        self.assertFalse(packet['status']['available'])
        self.assertEqual(packet['joints'], [])
        sample.update(available=False, data=None, sample_status='offline')
        self.assertEqual(skeleton.skeleton_stream(sample)['joints'], [])

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
        self.assertEqual(data['working_mode'], 'developer_mode')
        self.assertIsNone(data['sw_version'])







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
        for card in ('joint_state', 'robot_status'):
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


class JointPublicationTests(unittest.IsolatedAsyncioTestCase):
    async def first_packet(self, adapter, expected_topic=TOPICS['joint_state']):
        ready = asyncio.Event()
        packets = []
        def capture(topic, data):
            self.assertEqual(topic, expected_topic)
            packets.append(data)
            ready.set()
        adapter.publish = capture
        task = asyncio.create_task(adapter.publish_loop())
        try:
            await asyncio.wait_for(ready.wait(), .5)
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        return packets[0]




    async def test_new_joint_feedback_updates_skeleton_without_invalid_orientation(self):
        adapter = Adapter(lambda *args: None, settings(enabled_cards=['joint_state']))
        adapter.reader.accept_info(info())
        packets = []
        for elbow in (-.0032, -.3522657):
            q = [0.0]*16
            q[3] = elbow
            adapter.reader.joints = joints(q=q)
            adapter.reader.joints_at = time.monotonic()
            adapter.reader.joints_received = time.time()
            adapter.reader.joints = state.joint_state(adapter.reader.joints)
            packet = await self.first_packet(adapter, TOPICS['joint_state'])
            self.assertNotIn('imu_quat', packet)
            self.assertTrue(packet['status']['available'])
            angle = next(j['q'] for j in packet['joints'] if j['name'] == 'elbow_L_Joint')
            self.assertEqual(angle, elbow)
            packets.append(packet)
        self.assertNotEqual(packets[0]['joints'][3]['q'], packets[1]['joints'][3]['q'])
        adapter.reader.disconnect('test')
        packet = await self.first_packet(adapter, TOPICS['joint_state'])
        self.assertNotIn('imu_quat', packet)
        self.assertEqual(packet['joints'], [])



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


    async def test_real_websocket_movej_ack_correlation_reject_and_disconnect(self):
        requests = []
        mode = ['success']
        async def robot(request):
            ws = web.WebSocketResponse()
            await ws.prepare(request)
            await ws.send_json(info())
            async for msg in ws:
                body = json.loads(msg.data)
                requests.append(body)
                if body['title'] == 'request_get_joint_state':
                    await ws.send_json({'title': 'response_get_joint_state', 'accid': 'TEST_TRON2A',
                        'guid': body['guid'], 'data': joints()})
                else:
                    ack = {'title': 'response_movej', 'accid': 'TEST_TRON2A', 'guid': body['guid']}
                    # These wrong ACKs must never satisfy the pending command.
                    await ws.send_json({**ack, 'guid': 'wrong', 'data': {'result': 'success'}})
                    await ws.send_json({**ack, 'accid': 'OTHER', 'data': {'result': 'success'}})
                    if mode[0] == 'disconnect':
                        await ws.close()
                    elif mode[0] != 'timeout':
                        await ws.send_json({**ack, 'data': {'result': mode[0]}})
            return ws
        app = web.Application()
        app.router.add_get('/', robot)
        runner = web.AppRunner(app)
        await runner.setup()
        server = await asyncio.get_running_loop().create_server(runner.server, '127.0.0.1', 0)
        reader = RobotReader(settings(url='ws://127.0.0.1:'+str(server.sockets[0].getsockname()[1]),
                                      response_timeout_seconds=.15, poll_interval_seconds=.02))
        async with aiohttp.ClientSession(trust_env=False) as session:
            task = asyncio.create_task(reader.run(session))
            try:
                for _ in range(100):
                    if reader.sample('joint_state')['available']: break
                    await asyncio.sleep(.01)
                result = await reader.send_movej([0]*14, 2)
                self.assertEqual(result['status'], 'accepted')
                self.assertFalse(result['arrival_verified'])
                mode[0] = 'fail_invalid_cmd'
                with self.assertRaisesRegex(ValueError, 'rejected'):
                    await reader.send_movej([0]*14, 2)
                mode[0] = 'timeout'
                with self.assertRaisesRegex(ValueError, 'timed out'):
                    await reader.send_movej([0]*14, 2)
                mode[0] = 'disconnect'
                with self.assertRaisesRegex(ValueError, 'Disconnected'):
                    await reader.send_movej([0]*14, 2)
                self.assertIsNone(reader.move_pending)
                self.assertEqual(len([r for r in requests if r['title']=='request_movej']), 4)
            finally:
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
        server.close()
        await server.wait_closed()
        await runner.cleanup()


class MCPTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.published = []
        self.adapter = Adapter(lambda topic, data: self.published.append((topic, data)), settings())
        # These tests cover admission/ACK wire behavior; feedback verification
        # and real ACP HTTP delivery are exercised separately in test_completion.
        self.adapter.motion.wait_for_arrival = AsyncMock(return_value={'arrival_verified': True})
        self.adapter.motion.report = AsyncMock(return_value=True)
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
        await self.adapter.motion.close()
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

    async def test_handshake_skeleton_and_model_resource(self):
        reply = await self.rpc('initialize')
        self.assertEqual(reply['result']['serverInfo']['version'], VERSION)
        tools = (await self.rpc('tools/list'))['result']['tools']
        self.assertEqual({t['name'] for t in tools}, set(TOPICS) | {'model'})
        self.assertEqual({t['type'] for t in tools}, {'sensor', 'resource'})
        self.assertEqual((await self.rpc('resources/list'))['result']['resources'], [])
        for tool in tools:
            if tool['type'] == 'sensor':
                self.assertEqual(tool['topic_out'][0]['topic'], TOPICS[tool['name']])
                self.assertEqual(tool['topic_out'][0]['format'],
                                 'sensor/skeleton' if tool['name'] == 'joint_state' else 'data/json')

    async def test_canvas_lifecycle_and_cache_read_have_no_robot_write_path(self):
        for name in TOPICS:
            for action in ('start', 'info', 'stop'):
                error, result = await self.call(name, action=action, instance_id='canvas-card')
                self.assertFalse(error)
                self.assertTrue(result['read_only'])
                self.assertTrue(result['always_on'])
        error, result = await self.call('joint_state', action='info')
        self.assertFalse(error)
        self.assertTrue(result['sample']['available'])
        self.assertEqual(result['sample']['data']['joint_count'], 16)

    async def test_removed_read_state_is_not_callable(self):
        for action in ('read', 'info', 'start', 'stop'):
            with self.subTest(action=action):
                error, result = await self.call('read_state', action=action,
                    instance_id='old-resource', _trace_id='valid')
                self.assertTrue(error)
                self.assertIn('Unknown tool', result['error'])

    async def test_core_trace_and_instance_metadata_are_declared_and_ignored(self):
        tools = (await self.rpc('tools/list'))['result']['tools']
        trace = 'b2d39576-6402-4855-8367-3b19df60e76f'
        # Exercise the actual HTTP tools/call path used after Core injection.
        for tool in tools:
            props = tool['inputSchema']['properties']
            self.assertEqual(props['_trace_id']['maxLength'], 128)
            self.assertEqual(props['instance_id']['type'], 'string')
            if tool['name'] == 'model':
                error, result = await self.call('model', instance_id='model', _trace_id=trace)
                self.assertFalse(error)
                self.assertIn('<robot', result['urdf'])
                continue
            actions = ('info', 'start', 'stop')
            for action in actions:
                with self.subTest(tool=tool['name'], action=action):
                    error, original = await self.call(tool['name'], action=action)
                    self.assertFalse(error)
                    error, traced = await self.call(tool['name'], action=action,
                        instance_id='tron2-sensor-card', _trace_id=trace)
                    self.assertFalse(error)
                    self.assertTrue(traced['read_only'])
                    # A different receipt age is natural; the sampled robot data
                    # and lifecycle result must otherwise be independent of metadata.
                    if action == 'info':
                        self.assertEqual(traced['sample']['data'], original['sample']['data'])
                    else:
                        self.assertEqual(traced, original)
                    self.assertNotIn(trace, json.dumps(traced))

    async def test_core_metadata_validation_does_not_allow_control_payloads(self):
        for args in ({'_trace_id': None}, {'_trace_id': 10}, {'_trace_id': {}},
                     {'_trace_id': 'x'*129}, {'_trace_id': 'bad\ntrace'},
                     {'_trace_id': 'bad\x7ftrace'}, {'instance_id': ['card']},
                     {'_trace_id': 'valid', 'target_q': [0]*14},
                     {'_trace_id': 'valid', 'unexpected': True}):
            with self.subTest(args=args):
                error, result = await self.call('joint_state', action='info', **args)
                self.assertTrue(error)
                self.assertTrue(result['read_only'])
        error, result = await self.call('reset_pose', action='execute', _trace_id='valid')
        self.assertTrue(error)

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
            for topic,packet in self.published:
                self.assertEqual(topic,TOPICS['joint_state'])
                self.assertTrue(packet['status']['available'])
                self.assertEqual(len(packet['joints']),16)
            self.published.clear();self.adapter.reader.disconnect('Disconnected')
            await asyncio.sleep(.51)
            self.assertTrue(self.published)
            for topic,packet in self.published:
                self.assertEqual(packet['joints'],[])
                self.assertFalse(packet['status']['available'])
        finally:
            task.cancel();await asyncio.gather(task,return_exceptions=True)

    async def test_health_reports_failure_after_disconnect(self):
        self.adapter.reader.disconnect('Disconnected')
        async with self.session.get(self.url+'/health') as response:
            self.assertEqual(response.status, 503)
            self.assertFalse((await response.json())['ready'])


    async def test_disabled_card_is_not_discovered(self):
        adapter = Adapter(lambda *args: None, settings(enabled_cards=['joint_state']))
        self.assertEqual({t['name'] for t in adapter.tools()}, {'joint_state', 'model'})
        with self.assertRaises(ValueError):
            adapter.call('battery_state', {'action': 'start'})

    async def test_arm_control_tool_appears_when_enabled(self):
        adapter = Adapter(lambda *args: None, settings(enabled_cards=['arm_control']))
        tools = adapter.tools()
        names = {t['name'] for t in tools}
        self.assertIn('arm_control', names)
        arm = next(t for t in tools if t['name'] == 'arm_control')
        self.assertEqual(arm['type'], 'actuator')
        props = arm['inputSchema']['properties']
        self.assertEqual(props['action']['enum'], ['move', 'reset'])
        self.assertEqual(props['joint']['minItems'], 14)
        self.assertEqual(props['joint']['maxItems'], 14)
        self.assertEqual(props['joint']['type'], ['array', 'string'])
        self.assertEqual(props['joint']['maxLength'], 4096)
        self.assertEqual(arm['inputSchema']['x-action-params']['reset']['params'], [])
        self.assertNotIn('pose_name', props)
        self.assertEqual(set(arm['inputSchema']['x-action-params']), {'move', 'reset'})

    async def test_removed_cards_are_not_discovered_or_callable(self):
        self.adapter.enabled.append('arm_control')
        names = {tool['name'] for tool in self.adapter.tools()}
        self.assertEqual(names, {'joint_state', 'arm_control', 'model'})
        for name in ('robot_status', 'robot_state', 'reset_pose', 'joint_angles', 'joint_angle', 'battery_state'):
            error, result = await self.call(name, action='info')
            self.assertTrue(error)
            self.assertFalse(result['motion_sent'])

    async def test_removed_pose_and_pose_name_reject_without_motion(self):
        self.adapter.enabled.append('arm_control')
        reader = self.adapter.reader
        reader.ws = types.SimpleNamespace(closed=False, send_json=AsyncMock())
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'home.json'
            self.reset_pose_fixture(path)
            raw = path.read_bytes()
            for args in ({'action':'pose'}, {'action':'pose','pose_name':'left_elbow_20'},
                         {'action':'reset','pose_name':'arms_down'},
                         {'action':'move','joint':[.1]*14,'time':8,'pose_name':'arms_down'}):
                error, result = await self.call('arm_control', **args)
                self.assertTrue(error)
                self.assertFalse(result['motion_sent'])
            self.assertEqual(path.read_bytes(), raw)
        reader.ws.send_json.assert_not_called()
        self.assertIsNone(self.adapter.motion.active)

    async def test_arm_control_move_rejects_wrong_joint_count(self):
        adapter = Adapter(lambda *args: None, settings(enabled_cards=['arm_control']))
        adapter.reader.accept_info(info())
        adapter.reader.ws = MagicMock()
        with self.assertRaises(ValueError):
            adapter.call('arm_control', {'action': 'move', 'joint': [0.0] * 16})

    async def test_arm_control_move_rejects_non_numeric_joints(self):
        adapter = Adapter(lambda *args: None, settings(enabled_cards=['arm_control']))
        adapter.reader.accept_info(info())
        adapter.reader.ws = MagicMock()
        with self.assertRaises(ValueError):
            adapter.call('arm_control', {'action': 'move', 'joint': [0.0] * 13 + ['bad']})

    async def test_arm_control_move_sends_once_and_waits_for_ack(self):
        self.adapter.enabled.append('arm_control')
        reader = self.adapter.reader
        sent = []
        async def send(body):
            sent.append(body)
            packet = {'title': 'response_movej', 'guid': body['guid'],
                      'accid': reader.serial, 'data': {'result': 'success'}}
            ws = types.SimpleNamespace(receive=AsyncMock(return_value=types.SimpleNamespace(
                type=aiohttp.WSMsgType.TEXT, data=json.dumps(packet))))
            await reader.receive(ws, time.monotonic()+1)
        reader.ws = types.SimpleNamespace(closed=False, send_json=send)
        ids = set()
        for joint in ([.1]*14, json.dumps([.1]*14)):
            with self.subTest(joint_type=type(joint).__name__):
                sent.clear()
                error, result = await self.call('arm_control', action='move', joint=joint,
                                               time=1.5, _trace_id='valid')
                self.assertFalse(error)
                self.assertEqual(result['status'], 'accepted')
                self.assertFalse(result['arrival_verified'])
                self.assertTrue(result['motion_sent'])
                self.assertEqual(len(sent), 1)
                self.assertEqual(sent[0]['title'], 'request_movej')
                self.assertEqual(sent[0]['data'], {'joint': [.1]*14, 'time': 1.5})
                self.assertNotIn('_trace_id', sent[0])
                self.assertNotIn(result['action_id'], ids)
                ids.add(result['action_id'])
                await self.adapter.motion.task
                self.assertEqual(self.adapter.motion.report.await_args.args[:2],
                                 (result['action_id'], 'completed'))

    async def test_arm_canvas_invalid_json_never_sends(self):
        self.adapter.enabled.append('arm_control')
        ws = types.SimpleNamespace(closed=False, send_json=AsyncMock())
        self.adapter.reader.ws = ws
        values = ['[0,', '0,0', 'null', '{}', '"[]"', '['*1100,
                  ' '*4097, json.dumps([0]*13), json.dumps([0]*16),
                  json.dumps([True]*14), json.dumps(['0']*14),
                  json.dumps([10]*14), json.dumps([math.nan]*14),
                  json.dumps([math.inf]*14)]
        for value in values:
            with self.subTest(value=value[:50]):
                error, result = await self.call('arm_control', action='move', joint=value, time=8)
                self.assertTrue(error)
                self.assertFalse(result['motion_sent'])
        ws.send_json.assert_not_called()

    async def test_arm_control_invalid_offline_and_stale_never_send(self):
        self.adapter.enabled.append('arm_control')
        reader = self.adapter.reader
        ws = types.SimpleNamespace(closed=False, send_json=AsyncMock())
        reader.ws = ws
        for values in ([0]*16, [0]*13, [math.nan]*14, [True]*14, [10]*14):
            error, result = await self.call('arm_control', action='move', joint=values)
            self.assertTrue(error)
            self.assertFalse(result['motion_sent'])
        for duration in (True, '2', math.inf, 0, 61):
            error, result = await self.call('arm_control', action='move', joint=[0]*14, time=duration)
            self.assertTrue(error)
        reader.info_at = time.monotonic()-10
        error, result = await self.call('arm_control', action='move', joint=[0]*14)
        self.assertTrue(error)
        self.assertFalse(result['motion_sent'])
        reader.disconnect('test')
        error, result = await self.call('arm_control', action='move', joint=[0]*14)
        self.assertTrue(error)
        self.assertFalse(result['motion_sent'])
        ws.send_json.assert_not_called()

    async def test_arm_info_and_periodic_sensor_publication_with_actuator_enabled(self):
        self.adapter.enabled.append('arm_control')
        error, result = await self.call('arm_control', action='info')
        self.assertFalse(error)
        self.assertEqual(result['joint_count'], 14)
        self.assertFalse(result['motion_sent'])
        self.assertNotIn('arm_control', self.adapter.snapshot())
        task = asyncio.create_task(self.adapter.publish_loop())
        try:
            await asyncio.sleep(.02)
            self.assertEqual({topic for topic, _ in self.published}, set(TOPICS.values()))
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        error, result = await self.call('arm_control', action='execute')
        self.assertTrue(error)
        self.assertIn('Unsupported actuator action', result['error'])

    async def test_all_actuator_lifecycle_calls_never_send_even_with_move_fields(self):
        self.adapter.enabled.append('arm_control')
        reader = self.adapter.reader
        reader.ws = types.SimpleNamespace(closed=False, send_json=AsyncMock())
        tools = {tool['name']: tool for tool in self.adapter.tools()}
        for name in ('arm_control',):
            for action in ('info', 'start', 'stop'):
                self.assertNotIn(action, tools[name]['inputSchema']['properties']['action']['enum'])
                kwargs = {'joint': [.1]*14, 'time': 8} if name == 'arm_control' else {}
                error, result = await self.call(name, action=action,
                    instance_id='canvas', _trace_id='valid', **kwargs)
                self.assertFalse(error)
                self.assertFalse(result['motion_sent'])
        reader.ws.send_json.assert_not_called()
        self.assertEqual(set(self.adapter.snapshot()), set(TOPICS))

    def reset_pose_fixture(self, path, **updates):
        pose = {'schema': 'tron2-arms-reset-pose-v1', 'scope': 'arms_only',
                'unit': 'rad', 'joint_names': ARM_NAMES, 'accid': 'TEST_TRON2A',
                'firmware': 'TEST_FW', 'target_q': [.1]*14,
                'head_policy': 'unchanged', 'gripper_policy': 'unchanged', **updates}
        path.write_text(json.dumps(pose))
        self.adapter.reset.path = path
        self.adapter.reader.info['sw_version'] = 'TEST_FW'

    async def test_reset_missing_pose_does_not_block_start_but_execute_rejects(self):
        self.adapter.enabled.append('arm_control')
        with tempfile.TemporaryDirectory() as temp:
            self.adapter.reset.path = Path(temp)/'missing.json'
            error, result = await self.call('arm_control', action='start')
            self.assertFalse(error)
            self.assertFalse(result['motion_sent'])
            error, result = await self.call('arm_control', action='info')
            self.assertFalse(error)
            self.assertFalse(result['home']['configured'])
            error, result = await self.call('arm_control', action='reset')
            self.assertTrue(error)
            self.assertFalse(result['motion_sent'])

    async def test_removed_actions_and_invalid_reset_identity_never_send_motion(self):
        self.adapter.enabled.append('arm_control')
        reader = self.adapter.reader
        reader.ws = types.SimpleNamespace(closed=False, send_json=AsyncMock())
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'home.json'
            self.reset_pose_fixture(path)
            for action in ('preview', 'list', 'verify_at_home'):
                error, result = await self.call('arm_control', action=action)
                self.assertTrue(error)
                self.assertFalse(result['motion_sent'])
                self.assertIn('Unsupported actuator action', result['error'])
            for updates in ({'accid': 'OTHER'}, {'firmware': 'OTHER'},
                            {'unit': 'deg'}, {'target_q': [0]*16}, {'target_q': [10]*14},
                            {'joint_names': list(reversed(ARM_NAMES))}):
                self.reset_pose_fixture(path, **updates)
                error, result = await self.call('arm_control', action='reset')
                self.assertTrue(error)
                self.assertFalse(result['motion_sent'])
        reader.ws.send_json.assert_not_called()

    async def test_reset_execute_uses_only_file_target_and_awaits_ack(self):
        self.adapter.enabled.append('arm_control')
        reader = self.adapter.reader
        sent = []
        async def send(body):
            sent.append(body)
            packet = {'title': 'response_movej', 'guid': body['guid'],
                      'accid': reader.serial, 'data': {'result': 'success'}}
            ws = types.SimpleNamespace(receive=AsyncMock(return_value=types.SimpleNamespace(
                type=aiohttp.WSMsgType.TEXT, data=json.dumps(packet))))
            await reader.receive(ws, time.monotonic()+1)
        reader.ws = types.SimpleNamespace(closed=False, send_json=send)
        with tempfile.TemporaryDirectory() as temp:
            self.reset_pose_fixture(Path(temp)/'home.json')
            error, result = await self.call('arm_control', action='reset', target_q=[0]*14)
            self.assertTrue(error)
            self.assertEqual(sent, [])
            error, result = await self.call('arm_control', action='reset', _trace_id='valid')
            self.assertFalse(error)
            self.assertEqual(result['action'], 'reset')
            self.assertEqual(result['status'], 'accepted')
            self.assertFalse(result['arrival_verified'])
            self.assertEqual(len(sent), 1)
            self.assertEqual(sent[0]['data'], {'joint': [.1]*14, 'time': 8})
            await self.adapter.motion.task
            self.assertEqual(self.adapter.motion.report.await_args.args[:2],
                             (result['action_id'], 'completed'))

    async def test_reset_verification_named_feedback_uses_arm_order_excludes_head(self):
        self.adapter.enabled.append('arm_control')
        with tempfile.TemporaryDirectory() as temp:
            self.reset_pose_fixture(Path(temp)/'home.json')
            names = list(reversed(skeleton.JOINT_NAMES))
            self.adapter.reader.joints = state.joint_state(joints(
                names=names, q=list(reversed([.1]*14+[1, -1]))))
            self.adapter.reader.ws = types.SimpleNamespace(closed=False)
            self.adapter.reader.send_movej = AsyncMock(return_value={'status': 'accepted', 'motion_sent': True})
            error, result = await self.call('arm_control', action='reset')
            self.assertFalse(error)
            self.assertIsNone(result['at_home'])
            await self.adapter.motion.task
            terminal = self.adapter.motion.report.await_args.args[2]
            self.assertTrue(terminal['at_home'])
            self.assertTrue(terminal['reset_verified'])
            self.adapter.reader.send_movej.assert_awaited_once_with([.1]*14, 8)

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

    def test_removed_battery_has_no_config_card_topic_or_sample(self):
        config=yaml.safe_load((DRIVER/'config.yaml').read_text())
        manifest=yaml.safe_load((DRIVER/'driver.yaml').read_text())
        self.assertNotIn('battery_state',config['plugins'])
        self.assertNotIn('battery_state', {c['name'] for c in manifest['cards']})
        self.assertEqual(set(TOPICS),{'joint_state'})
        config['plugins']['battery_state']={'enabled':True}
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'config.yaml';path.write_text(yaml.safe_dump(config))
            with patch.dict(os.environ,CONFIG_PATH=str(path)):
                with self.assertRaisesRegex(ValueError,'Only TRON2'):
                    self.function('load_settings')()
        reader=RobotReader(settings())
        for connected in (False,True):
            if connected: reader.accept_info(info())
            with self.assertRaisesRegex(ValueError,'Unknown sensor'):reader.sample('battery_state')
        with self.assertRaisesRegex(ValueError,'Unknown TRON2 card'):
            Adapter(lambda *args:None,settings(enabled_cards=['battery_state']))

    def test_removed_angle_plugin_is_rejected_in_configuration(self):
        config=yaml.safe_load((DRIVER/'config.yaml').read_text())
        self.assertNotIn('joint_angles',config['plugins'])
        config['plugins']['joint_angles']={'enabled':True}
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'config.yaml';path.write_text(yaml.safe_dump(config))
            with patch.dict(os.environ,CONFIG_PATH=str(path)):
                with self.assertRaisesRegex(ValueError,'Only TRON2'):
                    self.function('load_settings')()
        with self.assertRaisesRegex(ValueError,'Unknown TRON2 card'):
            Adapter(lambda *args:None,settings(enabled_cards=['joint_angles']))

    def test_work_image_path_does_not_index_shallow_parents(self):
        with patch.object(Path, 'is_file', autospec=True,
                          side_effect=lambda p: str(p) == '/work/common/logsafe.py'):
            self.assertEqual(self.function('common_root')('/work/main.py'), Path('/work'))

    def test_staged_entry_imports_without_repository_layout(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for name in ('main.py','device.py','vendor.py','state.py','reset.py','skeleton.py','completion.py','camera.py','ros2.py','config.yaml'):
                shutil.copyfile(DRIVER/name, root/name)
            shutil.copytree(DRIVER/'resource', root/'resource')
            shutil.copytree(DRIVER.parents[1]/'common', root/'common')
            script = ('from pathlib import Path; import main, device, vendor, state, ros2, common, skeleton; '
                'assert "<robot" in skeleton.model()["urdf"]; '
                'assert device.VERSION == "0.5.12"; assert Path(common.__file__).parent.resolve() == Path('+
                repr(str(root/'common'))+').resolve()')
            result = subprocess.run([sys.executable,'-c',script], cwd=temp,
                env={**os.environ,'PYTHONPATH':temp}, text=True, capture_output=True, timeout=8)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_only_read_query_exists_in_wire_send_ast(self):
        source = ast.parse((DRIVER/'vendor.py').read_text())
        calls = [n for n in ast.walk(source) if isinstance(n,ast.Call)
                 and isinstance(n.func,ast.Attribute) and n.func.attr == 'send_json']
        self.assertEqual(len(calls),2)
        payloads = [{k.value:v for k,v in zip(c.args[0].keys,c.args[0].values)} for c in calls]
        self.assertEqual({p['title'].value for p in payloads}, {'request_get_joint_state', 'request_movej'})
        query = next(p for p in payloads if p['title'].value == 'request_get_joint_state')
        self.assertEqual(query['data'].keys, [])
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
                changed['plugins']['unknown_motion'] = {'enabled':True}
                path.write_text(yaml.safe_dump(changed))
                with patch.dict(os.environ,CONFIG_PATH=str(path)), self.assertRaises(ValueError): load()
                changed['plugins'] = {'arm_control': {'enabled': True}}
                path.write_text(yaml.safe_dump(changed))
                with patch.dict(os.environ,CONFIG_PATH=str(path)), self.assertRaisesRegex(ValueError, 'read-only sensor'): load()
                path.write_text(yaml.safe_dump(config))
                with patch.dict(os.environ,CONFIG_PATH=str(path)):
                    self.assertEqual(set(load()['enabled_cards']),set(TOPICS) | {'arm_control', 'camera', 'camera_left', 'camera_right'})

    def test_camera_only_and_fixed_source_configuration(self):
        load = self.function('load_settings')
        config = yaml.safe_load((DRIVER/'config.yaml').read_text())
        config['plugins'] = {'camera': {'enabled': True}}
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'config.yaml'; path.write_text(yaml.safe_dump(config))
            with patch.dict(os.environ, CONFIG_PATH=str(path), TRON2_ROBOT_URL='ws://[::1]:5000'):
                settings = load()
                self.assertEqual(settings['enabled_cards'], ['camera'])
                self.assertEqual(settings['camera_url'], 'http://[::1]:59001/stream?topic=/camera/top/color/image_raw/compressed&quality=80&rotate=0')
                self.assertEqual(settings['camera_urls']['camera_left'], 'http://[::1]:59001/stream?topic=/camera/left/color/image_rect_raw/compressed&quality=80&rotate=0')
                self.assertEqual(settings['camera_urls']['camera_right'], 'http://[::1]:59001/stream?topic=/camera/right/color/image_rect_raw/compressed&quality=80&rotate=0')

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
        self.assertIn('ROS_LOG_DIR=/tmp/ros-log', fragment['environment'])
        self.assertIn('ENV ROS_LOG_DIR=/tmp/ros-log', (DRIVER/'Dockerfile').read_text())
        self.assertNotIn('privileged',fragment)
        self.assertFalse(any('/dev' in v or 'docker.sock' in v or 'runstate' in v for v in fragment['volumes']))


if __name__ == '__main__':
    unittest.main()
