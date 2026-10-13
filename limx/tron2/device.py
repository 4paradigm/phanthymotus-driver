"""MCP discovery, sensor publication and explicit arm MoveJ calls."""
import asyncio
import json
import logging
import os

import aiohttp
from aiohttp import web
from vendor import RobotReader, validate_movej, reject_constant
from reset import ResetPose, POSES
from skeleton import skeleton_stream, model
from state import battery_stream

LOG = logging.getLogger('tron2')
VERSION = '0.5.1'
PORT = 15791
SERVER_NAME = 'LimX TRON2 Sensors and Arm Control'
TOPICS = {name: '/limx_tron2/' + name for name in
          ('joint_state', 'battery_state')}
DESCRIPTIONS = {
    'joint_state': 'TRON2 双臂与头部16关节实时骨架；配套厂家URDF，原始rad/deg及速度力矩可通过info查看',
    'battery_state': 'TRON2 电量（%）、电压原始值/电流绝对值、估算功率 estimated_power_w（按 mV/mA 假设）、单个温度原始值 temperature_raw（五路有效值中最高值，非℃）、循环次数；随厂家通知更新，诊断含接收年龄及五路原值（只读）',
    'arm_control': 'TRON2 双臂控制：move自定义14项rad目标，pose命名姿态，reset回到本机自然垂下目标；preview/list/verify_at_home只读；头部与夹爪不参与，响应仅确认受理，stop不是急停',
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
                    'action': {'type': 'string', 'enum': ['info', 'start', 'stop', 'move',
                        'pose', 'reset', 'preview', 'list', 'verify_at_home']},
                    'pose_name': {'type': 'string', 'enum': list(POSES),
                        'description': 'arms_down自然垂下；left_elbow_20左肘、right_elbow_20右肘、both_elbows_20双肘弯曲20度，均基于本机保存姿态'},
                    'joint': {
                        'type': ['array', 'string'],
                        'maxLength': 4096,
                        'minItems': 14,
                        'maxItems': 14,
                        'items': {'type': 'number'},
                        'description': '14 arm angles in rad: left arm 7, then right arm 7; excludes head. Canvas: enter a JSON array with square brackets.',
                    },
                    'time': {
                        'type': 'number',
                        'minimum': 0.1,
                        'maximum': 60,
                        'default': 8.0,
                        'description': 'Movement duration in seconds; pose requires at least 8 seconds and may need longer for a larger change',
                    },
                    **metadata_properties(),
                },
                'required': ['action'],
                'x-action-params': {
                    'move': {'params': ['joint', 'time'], 'description': '自定义双臂14项绝对关节目标'},
                    'pose': {'params': ['pose_name', 'time'], 'description': '执行命名姿态；时长至少8秒且满足角度差要求'},
                    'reset': {'params': [], 'description': '回到本机保存的自然垂下姿态'},
                    'preview': {'params': ['pose_name'], 'description': '预览姿态目标与角度差，不发送运动'},
                    'list': {'params': [], 'description': '列出姿态名称，不发送运动'},
                    'verify_at_home': {'params': [], 'description': '检查当前单个最近样本是否在复位目标1度内'},
                },
            },
        }

    def tools(self):
        tools = []
        # Battery card uses Unitree pattern: no action required, auto-publish
        if 'battery_state' in self.enabled:
            tools.append(self._battery_tool())
        if 'arm_control' in self.enabled:
            tools.append(self._arm_control_tool())
        for name in self.enabled:
            if name in ('battery_state', 'arm_control'):
                continue
            tools.append({'name': name, 'type': 'sensor', 'multiInstance': False,
                'description': DESCRIPTIONS[name],
                'inputSchema': {'type': 'object', 'additionalProperties': False,
                    'properties': {'action': {'type': 'string', 'enum': ['info', 'start', 'stop']},
                                   **metadata_properties()},
                    'required': ['action']},
                'topic_out': [{'topic': TOPICS[name], 'format': 'sensor/skeleton' if name == 'joint_state' else 'data/json'}]})
        if 'joint_state' in self.enabled:
            tools.append({'name': 'model', 'type': 'resource', 'multiInstance': False,
                'description': 'TRON2 DACH dual-arm/head URDF for the skeleton renderer',
                'inputSchema': {'type': 'object', 'additionalProperties': False,
                    'properties': metadata_properties()}})
        return tools

    def snapshot(self):
        return {name: self.reader.sample(name) for name in self.enabled if name in TOPICS}

    def call(self, name, args):
        allowed = {'action', 'instance_id', '_trace_id'}
        if name == 'arm_control':
            allowed |= {'joint', 'time', 'pose_name'}
        if not isinstance(args, dict) or set(args) - allowed:
            raise ValueError('Only action and Core instance_id/_trace_id metadata are accepted')
        # Core injects both fields after the model has selected a tool. Accept
        # bounded identifiers, but do not forward, log, or echo them as robot data.
        for key in ('instance_id', '_trace_id'):
            value = args.get(key, '')
            if not isinstance(value, str) or len(value) > 128 or any(
                    ord(c) < 32 or ord(c) == 127 for c in value):
                raise ValueError('Invalid Core metadata: ' + key)
        if name == 'model' and 'joint_state' in self.enabled:
            if 'action' in args:
                raise ValueError('Model has no action')
            return model()
        action = args.get('action', 'info' if name in TOPICS else None)
        if not isinstance(action, str):
            raise ValueError('An action string is required')
        if name not in self.enabled:
            raise ValueError('Unknown tool')
        if name == 'battery_state':
            # Battery card: only 'info' returns sample data; start/stop are no-ops
            if action not in ('info', 'start', 'stop'):
                raise ValueError('Unsupported sensor action')
            result = {'state': 'idle' if action == 'stop' else 'running', 'read_only': True,
                      'always_on': True, 'topic_out': [{'topic': TOPICS[name], 'format': 'sensor/skeleton' if name == 'joint_state' else 'data/json'}]}
            if action == 'info':
                result['sample'] = self.reader.sample(name)
            return result
        if name == 'arm_control':
            if action in ('start', 'stop'):
                return {'state': 'ready' if action == 'start' else 'idle',
                        'motion_sent': False, 'lifecycle_only': True}
            if action == 'info':
                return {'state': 'ready' if self.reader.connected else 'offline',
                        'joint_count': 14, 'position_unit': 'rad', 'motion_sent': False,
                        'home': self.reset.call('info'), 'poses': list(POSES)}
            if action == 'list':
                return {'poses': list(POSES), 'reference': 'saved_home', 'motion_sent': False}
            if action in ('reset', 'pose', 'preview', 'verify_at_home'):
                pose_name = args.get('pose_name', 'arms_down') if action in ('pose', 'preview') else 'arms_down'
                if not isinstance(pose_name, str) or pose_name not in POSES:
                    raise ValueError('Select a known pose_name; no motion sent')
                if action == 'pose' and 'pose_name' not in args:
                    raise ValueError('pose requires pose_name; no motion sent')
                operation = 'execute' if action in ('pose', 'reset') else action
                result = self.reset.call(operation, pose_name,
                                         args.get('time') if action == 'pose' else None)
                return {**result, 'action': action}
            if action != 'move':
                raise ValueError('Unsupported actuator action')
            joint = args.get('joint')
            # Motus canvas text inputs submit arrays as JSON strings. Decode only
            # this field, then use the same strict motion validation as API arrays.
            if isinstance(joint, str):
                if len(joint) > 4096:
                    raise ValueError('JOINT JSON text exceeds 4096 characters; no motion sent')
                try:
                    joint = json.loads(joint, parse_constant=reject_constant)
                except (ValueError, RecursionError) as exc:
                    raise ValueError('JOINT must be a JSON array of 14 numbers in rad; no motion sent') from exc
            joint, duration = validate_movej(joint, args.get('time', 8.0))
            # Pure validation. Only the async RPC path performs and awaits the send.
            return {'action': 'move', 'joint': joint, 'time': duration,
                    'joint_count': 14, 'status': 'validated', 'motion_sent': False}
        if action not in ('info', 'start', 'stop'):
            raise ValueError('Unsupported sensor action')
        result = {'state': 'idle' if action == 'stop' else 'running', 'read_only': True,
                  'always_on': True, 'topic_out': [{'topic': TOPICS[name], 'format': 'sensor/skeleton' if name == 'joint_state' else 'data/json'}]}
        if action == 'info':
            result['sample'] = self.reader.sample(name)
        return result

    async def publish_loop(self):
        while True:
            for name, data in self.snapshot().items():
                # Keep flat battery measurements; expose validity/receipt age
                # instead of making a fresh notification imply fresh BMS data.
                if name == 'battery_state':
                    data = battery_stream(data)
                elif name == 'joint_state':
                    data = skeleton_stream(data)
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
                    if params.get('name') == 'arm_control' and data.get('action') in ('move', 'pose', 'reset'):
                        action = data['action']
                        target = data['joint'] if action == 'move' else data['target_q']
                        duration = data['time'] if action == 'move' else data['duration_seconds']
                        details = {key: data[key] for key in ('pose_sha256', 'pose_name') if key in data}
                        reply = await self.reader.send_movej(target, duration)
                        data = {**reply, 'action': action, **details}
                    error = False
                except (ValueError, TypeError) as exc:
                    data, error = {'error': str(exc)[:240],
                        'read_only': params.get('name') not in self.enabled or params.get('name') not in ('arm_control',),
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
        return web.json_response({'version': VERSION, 'read_only': not set(self.enabled) & {'arm_control'}, 'ready': ready,
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
