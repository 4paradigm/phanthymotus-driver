"""MCP discovery and always-on read-only sensor cards."""
import asyncio
import json
import logging
import os

import aiohttp
from aiohttp import web
from vendor import RobotReader

LOG = logging.getLogger('tron2')
VERSION = '0.4.0'
PORT = 15791
SERVER_NAME = 'LimX TRON2 Read-only Sensors'
TOPICS = {name: '/limx_tron2/' + name for name in
          ('joint_state', 'robot_status', 'battery_state')}
DESCRIPTIONS = {
    'joint_state': 'TRON2 实时读取关节位置（rad/deg）、速度和力矩；名称缺失时显示索引',
    'robot_status': 'TRON2 模式、固件与 IMU/电机/相机状态（只读）',
    'battery_state': 'TRON2 电量百分比及厂家原始 BMS 参数（只读，不推测温度/电压单位）',
}


class Adapter:
    def __init__(self, publish, settings, reader=None):
        self.publish = publish
        self.settings = settings
        self.enabled = settings['enabled_cards']
        self.reader = reader or RobotReader(settings)
        self.registered_id = None
        self.core_ssl = True

    def tools(self):
        tools = []
        for name in self.enabled:
            tools.append({'name': name, 'type': 'sensor', 'multiInstance': False,
                'description': DESCRIPTIONS[name],
                'inputSchema': {'type': 'object', 'additionalProperties': False,
                    'properties': {'action': {'type': 'string', 'enum': ['info', 'start', 'stop']}},
                    'required': ['action']},
                'topic_out': [{'topic': TOPICS[name], 'format': 'data/json'}]})
        tools.append({'name': 'read_state', 'type': 'resource',
            'description': '读取三张传感器卡片的当前缓存与接收时效；不发送机器人控制指令',
            'inputSchema': {'type': 'object', 'additionalProperties': False,
                'properties': {'action': {'type': 'string',
                    'enum': ['info', 'read', 'start', 'stop'], 'default': 'read'}}}})
        return tools

    def snapshot(self):
        return {name: self.reader.sample(name) for name in self.enabled}

    def call(self, name, args):
        if not isinstance(args, dict) or set(args) - {'action', 'instance_id'}:
            raise ValueError('Only action and canvas instance_id are accepted')
        instance = args.get('instance_id', '')
        if not isinstance(instance, str) or len(instance) > 128 or any(
                ord(c) < 32 or ord(c) == 127 for c in instance):
            raise ValueError('Invalid canvas instance_id')
        action = args.get('action', 'read' if name == 'read_state' else None)
        if not isinstance(action, str):
            raise ValueError('An action string is required')
        if name == 'read_state':
            if action in ('start', 'stop'):
                return {'state': 'ready' if action == 'start' else 'idle', 'read_only': True}
            if action not in ('info', 'read'):
                raise ValueError('Unsupported read-only action')
            return {'read_only': True, 'samples': self.snapshot()}
        if name not in self.enabled:
            raise ValueError('Unknown tool; this driver exposes only parameter readers')
        if action not in ('info', 'start', 'stop'):
            raise ValueError('Unsupported sensor action')
        # Single-instance sensors remain on; lifecycle calls do not affect the robot.
        result = {'state': 'idle' if action == 'stop' else 'running', 'read_only': True,
                  'always_on': True, 'topic_out': [{'topic': TOPICS[name], 'format': 'data/json'}]}
        if action == 'info':
            result['sample'] = self.reader.sample(name)
        return result

    async def publish_loop(self):
        while True:
            for name, data in self.snapshot().items():
                self.publish(TOPICS[name], data)
            await asyncio.sleep(self.settings['poll_interval_seconds'])

    async def rpc(self, request):
        try:
            body = await request.json()
        except (ValueError, UnicodeError):
            return web.json_response({'jsonrpc': '2.0', 'id': None,
                'error': {'code': -32700, 'message': 'Parse error'}})
        if not isinstance(body, dict) or body.get('jsonrpc') != '2.0':
            return web.json_response({'jsonrpc': '2.0', 'id': None,
                'error': {'code': -32600, 'message': 'Invalid request'}})
        if 'id' not in body:
            return web.Response(status=204)
        rid, method = body['id'], body.get('method')
        params = body.get('params', {})
        try:
            if not isinstance(params, dict):
                raise ValueError('Parameters must be an object')
            if method == 'initialize':
                result = {'protocolVersion': '2024-11-05', 'capabilities': {'tools': {}},
                          'serverInfo': {'name': SERVER_NAME, 'version': VERSION}}
            elif method == 'tools/list':
                result = {'tools': self.tools()}
            elif method == 'resources/list':
                result = {'resources': []}
            elif method == 'ping':
                result = {}
            elif method == 'tools/call':
                try:
                    data = self.call(params.get('name'), params.get('arguments', {}))
                    error = False
                except (ValueError, TypeError) as exc:
                    data, error = {'error': str(exc)[:240], 'read_only': True}, True
                result = {'content': [{'type': 'text', 'text': json.dumps(data,
                           ensure_ascii=False, allow_nan=False)}], 'isError': error}
            else:
                return web.json_response({'jsonrpc': '2.0', 'id': rid,
                    'error': {'code': -32601, 'message': 'Method not found'}})
            return web.json_response({'jsonrpc': '2.0', 'id': rid, 'result': result})
        except (ValueError, TypeError) as exc:
            return web.json_response({'jsonrpc': '2.0', 'id': rid,
                'error': {'code': -32602, 'message': str(exc)[:240]}})

    async def health(self, request):
        samples = self.snapshot()
        ready = all(s['available'] for s in samples.values())
        return web.json_response({'version': VERSION, 'read_only': True, 'ready': ready,
            'robot_connected': self.reader.connected, 'registered_mcp_id': self.registered_id,
            'last_connection_error': self.reader.last_error,
            'round_trip_seconds': self.reader.last_round_trip, 'samples': samples},
            status=200 if ready else 503)

    async def register_loop(self, session):
        last = None
        url = os.environ.get('AGENT_CORE_URL', 'https://phanthy-motus:15678').rstrip('/')
        while True:
            try:
                async with session.post(url + '/api/mcp', ssl=self.core_ssl,
                        allow_redirects=False, json={'name': SERVER_NAME, 'transport': 'http',
                        'url': 'http://127.0.0.1:15791/mcp', 'category': 'driver',
                        'render_hint': 'data/json'}) as response:
                    if response.status != 200:
                        raise ValueError('Core registration HTTP ' + str(response.status))
                    body = await response.json()
                if body.get('code') != 200 or not isinstance(body.get('data'), dict) \
                        or not body['data'].get('id'):
                    raise ValueError('Core registration did not return an ID')
                self.registered_id = body['data']['id']
                if last != 'ok':
                    LOG.info('Registered sensor driver with Motus')
                last = 'ok'
            except (OSError, ValueError, TypeError, AttributeError, aiohttp.ClientError,
                    asyncio.TimeoutError) as exc:
                self.registered_id = None
                if last != 'error':
                    LOG.warning('Core registration pending (%s)', type(exc).__name__)
                last = 'error'
            await asyncio.sleep(30 if last == 'ok' else 5)
