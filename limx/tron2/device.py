"""MCP discovery and always-on read-only sensor cards."""
import asyncio
import json
import logging
import os

import aiohttp
from aiohttp import web
from vendor import RobotReader
from state import BATTERY_FIELDS

LOG = logging.getLogger('tron2')
VERSION = '0.4.3'
PORT = 15791
SERVER_NAME = 'LimX TRON2 Read-only Sensors'
TOPICS = {name: '/limx_tron2/' + name for name in
          ('joint_state', 'robot_status', 'battery_state')}
DESCRIPTIONS = {
    'joint_state': 'TRON2 实时读取关节位置（rad/deg）、速度和力矩；名称缺失时显示索引',
    'robot_status': 'TRON2 模式、固件与 IMU/电机/相机状态（只读）',
    'battery_state': 'TRON2 电量 soc（%）、电压 voltage（mV）、电流 current（mA）、功率 power（W）；单位未确认时相关字段为 null（只读）',
}


def metadata_properties():
    """Reserved Core metadata; neither field changes a robot request."""
    return {key: {'type': 'string', 'maxLength': 128,
                  'pattern': r'^[^\x00-\x1f\x7f]*$',
                  'description': description}
            for key, description in (
                ('instance_id', '画布调用层传入的卡片标识；可省略'),
                ('_trace_id', 'Agent Core 自动注入的追踪标识；无需用户填写'))}


class Adapter:
    def __init__(self, publish, settings, reader=None):
        self.publish = publish
        self.settings = settings
        self.enabled = settings['enabled_cards']
        self.reader = reader or RobotReader(settings)
        self.registered_id = None
        self.core_ssl = True

    def _battery_tool(self):
        """Battery sensor — empty inputSchema, auto-publishes via topic_out (Unitree pattern)."""
        return {'name': 'battery_state', 'type': 'sensor', 'multiInstance': False,
            'description': DESCRIPTIONS['battery_state'],
            'inputSchema': {'type': 'object', 'additionalProperties': False,
                'properties': {**metadata_properties()}},
            'topic_out': [{'topic': TOPICS['battery_state'], 'format': 'data/json'}]}

    def tools(self):
        tools = []
        # Battery card uses Unitree pattern: no action required, auto-publish
        if 'battery_state' in self.enabled:
            tools.append(self._battery_tool())
        for name in self.enabled:
            if name == 'battery_state':
                continue
            tools.append({'name': name, 'type': 'sensor', 'multiInstance': False,
                'description': DESCRIPTIONS[name],
                'inputSchema': {'type': 'object', 'additionalProperties': False,
                    'properties': {'action': {'type': 'string', 'enum': ['info', 'start', 'stop']},
                                   **metadata_properties()},
                    'required': ['action']},
                'topic_out': [{'topic': TOPICS[name], 'format': 'data/json'}]})
        return tools

    def snapshot(self):
        return {name: self.reader.sample(name) for name in self.enabled}

    def call(self, name, args):
        if not isinstance(args, dict) or set(args) - {'action', 'instance_id', '_trace_id'}:
            raise ValueError('Only action and Core instance_id/_trace_id metadata are accepted')
        # Core injects both fields after the model has selected a tool. Accept
        # bounded identifiers, but do not forward, log, or echo them as robot data.
        for key in ('instance_id', '_trace_id'):
            value = args.get(key, '')
            if not isinstance(value, str) or len(value) > 128 or any(
                    ord(c) < 32 or ord(c) == 127 for c in value):
                raise ValueError('Invalid Core metadata: ' + key)
        action = args.get('action')
        if not isinstance(action, str):
            raise ValueError('An action string is required')
        if name not in self.enabled:
            raise ValueError('Unknown tool; this driver exposes only parameter readers')
        if name == 'battery_state':
            # Battery card: only 'info' returns sample data; start/stop are no-ops
            if action not in ('info', 'start', 'stop'):
                raise ValueError('Unsupported sensor action')
            result = {'state': 'idle' if action == 'stop' else 'running', 'read_only': True,
                      'always_on': True, 'topic_out': [{'topic': TOPICS[name], 'format': 'data/json'}]}
            if action == 'info':
                result['sample'] = self.reader.sample(name)
            return result
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
                # Compact battery stream, like Unitree's flat BMS display.
                # Detailed connection/receipt metadata remains available via info.
                if name == 'battery_state':
                    data = data['data'] if data['available'] else \
                        dict.fromkeys(BATTERY_FIELDS)
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
