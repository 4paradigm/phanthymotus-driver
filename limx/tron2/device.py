"""MCP discovery, sensor publication and explicit arm MoveJ calls."""
import asyncio
import json
import logging
import os
import time

import aiohttp
from aiohttp import web
from vendor import RobotReader, validate_movej, reject_constant, motion_readiness
from reset import ResetPose
from skeleton import skeleton_stream, model
from completion import ArmCompletion, ACP_TIMEOUT
from camera import CameraStream, CAMERA_SOURCES

LOG = logging.getLogger('tron2')
VERSION = '0.5.11'
PORT = 15791
SERVER_NAME = 'LimX TRON2 Sensors and Arm Control'
TOPICS = {name: '/limx_tron2/' + name for name in
          ('joint_state',)}
DESCRIPTIONS = {
    'camera': 'TRON2 头部RGB实时画面；start连接厂家MJPEG并发布JPEG图像，stop释放连接；限10fps，不录像、不控制头部，接收时间不是测量时间（只读）',
    'camera_left': 'TRON2 左腕RGB实时画面；厂家left校正图像，独立start/stop/info；限10fps，不录像、不控制机械臂，接收时间不是测量时间（只读）',
    'camera_right': 'TRON2 右腕RGB实时画面；厂家right校正图像，独立start/stop/info；限10fps，不录像、不控制机械臂，接收时间不是测量时间（只读）',
    'joint_state': 'TRON2 双臂与头部16关节实时骨架，左右夹爪轮廓随手腕运动；夹爪默认为固定闭合示意，未接入开合反馈；原始rad/deg及速度力矩可通过info查看',
    'arm_control': 'TRON2 双臂控制：move自定义14项rad目标，reset自然垂下；响应含action_id，连续实时反馈确认到位后通知Core完成；reset自动验证自然垂下复位，两个动作均通过ACP报告完成',
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
    def __init__(self, publish, settings, reader=None, session=None):
        self.publish = publish
        self.settings = settings
        self.enabled = settings['enabled_cards']
        if set(self.enabled) - (set(TOPICS) | set(CAMERA_SOURCES) | {'arm_control'}):
            raise ValueError('Unknown TRON2 card in configuration')
        self.reader = reader or RobotReader(settings)
        self.registered_id = None
        self.core_ssl = True
        self.session = session
        self.reset = ResetPose(settings.get('home_file', '/config/tron2/home.json'), self.reader)
        self.motion = ArmCompletion(self.reader, self.report_completion)
        urls = settings.get('camera_urls', {})
        self.cameras = {name: CameraStream(urls.get(name, settings.get('camera_url', '') if name == 'camera' else ''),
                                          publish, session, topic=source[0])
                        for name, source in CAMERA_SOURCES.items()}
        self.camera = self.cameras['camera']

    async def report_completion(self, action_id, status, result):
        if self.session is None:
            return False
        url = os.environ.get('AGENT_CORE_URL', 'https://phanthy-motus:15678').rstrip('/')
        payload = {'action_id': action_id, 'status': status, 'result': result,
                   'tool': 'arm_control', 'ts': time.time()}
        for attempt in range(3):
            try:
                async with self.session.post(url + '/api/acp/complete',
                        ssl=self.core_ssl, allow_redirects=False,
                        timeout=aiohttp.ClientTimeout(total=3), json=payload) as response:
                    body = await response.json() if response.status == 200 else {}
                    if isinstance(body, dict) and body.get('ok') is True \
                            and body.get('action_id', action_id) == action_id:
                        return True
            except (OSError, ValueError, aiohttp.ClientError, asyncio.TimeoutError):
                pass
            if attempt < 2:
                await asyncio.sleep(.5 * 2 ** attempt)
        LOG.warning('Arm completion delivery failed; local motion owner retained')
        return False

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
                    'action': {'type': 'string', 'enum': ['move', 'reset']},
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
                        'description': 'Movement duration in seconds for move; reset computes its duration from the saved home',
                    },
                    **metadata_properties(),
                },
                'required': ['action'],
                'x-completion': {'actions': ['move', 'reset'], 'timeout': ACP_TIMEOUT},
                'x-resource': ['arm_l', 'arm_r'],
                'x-action-params': {
                    'move': {'params': ['joint', 'time'], 'description': '自定义双臂14项绝对关节目标'},
                    'reset': {'params': [], 'description': '回到自然垂下目标；连续新反馈自动确认复位后，通过ACP通知完成'},
                },
            },
        }

    def tools(self):
        tools = []
        if 'arm_control' in self.enabled:
            tools.append(self._arm_control_tool())
        for name in CAMERA_SOURCES:
            if name not in self.enabled:
                continue
            tools.append({'name': name, 'type': 'sensor', 'multiInstance': False,
                'description': DESCRIPTIONS[name],
                'inputSchema': {'type': 'object', 'additionalProperties': False,
                    'properties': {'action': {'type': 'string', 'enum': ['info', 'start', 'stop']},
                                   **metadata_properties()}},
                'topic_out': [{'topic': CAMERA_SOURCES[name][0], 'format': 'image/jpeg'}]})
        for name in self.enabled:
            if name == 'arm_control' or name in CAMERA_SOURCES:
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
                'description': 'TRON2 DACH dual-arm/head URDF with fixed gripper outlines for the skeleton renderer',
                'inputSchema': {'type': 'object', 'additionalProperties': False,
                    'properties': metadata_properties()}})
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
        if name == 'model' and 'joint_state' in self.enabled:
            if 'action' in args:
                raise ValueError('Model has no action')
            return model()
        action = args.get('action', 'info' if name in TOPICS or name in CAMERA_SOURCES else None)
        if not isinstance(action, str):
            raise ValueError('An action string is required')
        if name not in self.enabled:
            raise ValueError('Unknown tool')
        if name in CAMERA_SOURCES:
            if action not in ('info', 'start', 'stop'):
                raise ValueError('Unsupported camera action')
            return self.cameras[name].info()  # Lifecycle I/O is awaited in the RPC path.
        if name == 'arm_control':
            if action in ('start', 'stop'):
                return {'state': 'ready' if action == 'start' else 'idle',
                        'motion_sent': False, 'lifecycle_only': True}
            if action == 'info':
                return {'state': self.motion.active['state'] if self.motion.active else
                        'ready' if self.reader.connected else 'offline',
                        'joint_count': 14, 'position_unit': 'rad', 'motion_sent': False,
                        'motion': self.motion.summary(),
                        'motion_readiness': motion_readiness(self.reader.sample('robot_status')['data'] or {}),
                        'home': self.reset.call('info')}
            if action == 'reset':
                return {**self.reset.call('execute'), 'action': 'reset'}
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
            result['sample'] = self.snapshot()[name]
        return result

    async def publish_loop(self):
        while True:
            for name, data in self.snapshot().items():
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
                    if params.get('name') in CAMERA_SOURCES:
                        data = await self.cameras[params['name']].call(params.get('arguments', {}).get('action', 'info'))
                    if params.get('name') == 'arm_control' and data.get('action') in ('move', 'reset'):
                        action = data['action']
                        target = data['joint'] if action == 'move' else data['target_q']
                        duration = data['time'] if action == 'move' else data['duration_seconds']
                        details = {key: data[key] for key in ('pose_sha256',) if key in data}
                        data = await self.motion.dispatch(action, target, duration, details)
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
        ready = self.reader.connected and bool(samples) and all(s['available'] for s in samples.values())
        cameras = {name: stream.info() for name, stream in self.cameras.items() if name in self.enabled}
        if not samples and cameras:
            ready = all(info['available'] for info in cameras.values())
        return web.json_response({'version': VERSION, 'read_only': not set(self.enabled) & {'arm_control'}, 'ready': ready,
            'arm_motion': self.motion.summary(),
            'camera': self.camera.info() if 'camera' in self.enabled else None,
            'cameras': cameras,
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
