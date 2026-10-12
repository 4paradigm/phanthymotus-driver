"""MCP discovery, sensor publication and explicit arm MoveJ calls."""
import asyncio
import json
import logging
import os

import aiohttp
from aiohttp import web
from vendor import RobotReader, validate_movej
from reset import ResetPose
from state import BATTERY_FIELDS

LOG = logging.getLogger('tron2')
VERSION = '0.4.5'
PORT = 15791
SERVER_NAME = 'LimX TRON2 Sensors and Arm Control'
TOPICS = {name: '/limx_tron2/' + name for name in
          ('joint_state', 'robot_status', 'battery_state')}
DESCRIPTIONS = {
    'joint_state': 'TRON2 实时读取关节位置（rad/deg）、速度和力矩；名称缺失时显示索引',
    'robot_status': 'TRON2 模式、固件与 IMU/电机/相机状态（只读）',
    'battery_state': 'TRON2 电量 soc（%）、电压 voltage（mV）、电流 current（mA）、功率 power（W）、温度 temp（℃）；单位未确认时相关字段为 null（只读）',
    'arm_control': 'TRON2 双臂关节运动控制 — MoveJ 设置双臂 14 个关节目标（rad），头部不参与；响应仅确认受理',
    'reset_pose': 'TRON2 双臂固定姿态复位：preview 只读，execute 使用本机保存的14项目标；start/stop不发送运动，stop不是急停',
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
        self.reset = ResetPose(settings.get('home_file', '/config/tron2/home.json'), self.reader)

    def _battery_tool(self):
        """Battery sensor — empty inputSchema, auto-publishes via topic_out (Unitree pattern)."""
        return {'name': 'battery_state', 'type': 'sensor', 'multiInstance': False,
            'description': DESCRIPTIONS['battery_state'],
            'inputSchema': {'type': 'object', 'additionalProperties': False,
                'properties': {'action': {'type': 'string', 'enum': ['info', 'start', 'stop']}, **metadata_properties()}},
            'topic_out': [{'topic': TOPICS['battery_state'], 'format': 'data/json'}]}

    def _arm_control_tool(self):
        """Arm actuator — movej joint-space motion control."""
        return {
            'name': 'arm_control',
            'type': 'actuator',
            'multiInstance': False,
            'description': DESCRIPTIONS['arm_control'],
            'inputSchema': {
                'type': 'object',
                'additionalProperties': False,
                'properties': {
                    'action': {'type': 'string', 'enum': ['info', 'start', 'stop', 'move']},
                    'joint': {
                        'type': 'array',
                        'minItems': 14,
                        'maxItems': 14,
                        'items': {'type': 'number'},
                        'description': '14 arm angles in rad: left arm 7, then right arm 7; excludes head',
                    },
                    'time': {
                        'type': 'number',
                        'minimum': 0.1,
                        'maximum': 60,
                        'default': 2.0,
                        'description': 'Movement duration in seconds',
                    },
                    **metadata_properties(),
                },
                'required': ['action'],
            },
        }

    def tools(self):
        tools = []
        # Battery card uses Unitree pattern: no action required, auto-publish
        if 'battery_state' in self.enabled:
            tools.append(self._battery_tool())
        if 'arm_control' in self.enabled:
            tools.append(self._arm_control_tool())
        if 'reset_pose' in self.enabled:
            tools.append({'name': 'reset_pose', 'type': 'actuator', 'multiInstance': False,
                'description': DESCRIPTIONS['reset_pose'],
                'inputSchema': {'type': 'object', 'additionalProperties': False,
                    'properties': {'action': {'type': 'string',
                        'enum': ['info', 'start', 'stop', 'preview', 'verify_at_home', 'execute']},
                        **metadata_properties()}, 'required': ['action']}})
        for name in self.enabled:
            if name in ('battery_state', 'arm_control', 'reset_pose'):
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
        return {name: self.reader.sample(name) for name in self.enabled if name in TOPICS}

    def call(self, name, args):
        allowed = {'action', 'instance_id', '_trace_id'}
        if name == 'arm_control':
            allowed |= {'joint', 'time'}
        if not isinstance(args, dict) or set(args) - allowed:
            raise ValueError('Only action and Core instance_id/_trace_id metadata are accepted')
        # Core injects both fields after the model has selected a tool. Accept
        # bounded identifiers, but do not forward, log, or echo them as robot data.
        for key in ('instance_id', '_trace_id'):
            value = args.get(key, '')
            if not isinstance(value, str) or len(value) > 128 or any(
                    ord(c) < 32 or ord(c) == 127 for c in value):
                raise ValueError('Invalid Core metadata: ' + key)
        action = args.get('action', 'info' if name == 'battery_state' else None)
        if not isinstance(action, str):
            raise ValueError('An action string is required')
        if name not in self.enabled:
            raise ValueError('Unknown tool; this driver exposes only parameter readers')
        if name == 'reset_pose':
            return self.reset.call(action)
        if name == 'battery_state':
            # Battery card: only 'info' returns sample data; start/stop are no-ops
            if action not in ('info', 'start', 'stop'):
                raise ValueError('Unsupported sensor action')
            result = {'state': 'idle' if action == 'stop' else 'running', 'read_only': True,
                      'always_on': True, 'topic_out': [{'topic': TOPICS[name], 'format': 'data/json'}]}
            if action == 'info':
                result['sample'] = self.reader.sample(name)
            return result
        if name == 'arm_control':
            if action in ('start', 'stop'):
                return {'state': 'running' if action == 'start' else 'idle',
                        'motion_sent': False, 'lifecycle_only': True}
            if action == 'info':
                return {'state': 'ready' if self.reader.connected else 'offline',
                        'joint_count': 14, 'position_unit': 'rad', 'motion_sent': False}
            if action != 'move':
                raise ValueError('Unsupported actuator action')
            joint, duration = validate_movej(args.get('joint'), args.get('time', 2.0))
            # Pure validation. Only the async RPC path performs and awaits the send.
            return {'action': 'move', 'joint': joint, 'time': duration,
                    'joint_count': 14, 'status': 'validated', 'motion_sent': False}
        if action not in ('info', 'start', 'stop'):
            raise ValueError('Unsupported sensor action')
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
                    if params.get('name') == 'arm_control' and data.get('action') == 'move':
                        data = await self.reader.send_movej(data['joint'], data['time'])
                    elif params.get('name') == 'reset_pose' and data.get('action') == 'execute':
                        reply = await self.reader.send_movej(data['target_q'], data['duration_seconds'])
                        data = {**reply, 'action': 'execute', 'pose_sha256': data['pose_sha256']}
                    error = False
                except (ValueError, TypeError) as exc:
                    data, error = {'error': str(exc)[:240],
                        'read_only': params.get('name') not in self.enabled or params.get('name') not in ('arm_control', 'reset_pose'),
                        'motion_sent': getattr(exc, 'motion_sent', False)}, True
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
        return web.json_response({'version': VERSION, 'read_only': not set(self.enabled) & {'arm_control', 'reset_pose'}, 'ready': ready,
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
