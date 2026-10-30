"""MCP discovery, sensor publication and guarded TRON2 motion calls."""
import asyncio
import json
import logging
import os
import time

import aiohttp
from aiohttp import web
from vendor import MoveError, RobotReader, validate_movej, reject_constant, motion_readiness
from reset import ResetPose
from skeleton import skeleton_stream, model
from completion import ArmCompletion, MotionPeers, ACP_TIMEOUT
from pose import Pose, tool as pose_tool, arguments as pose_arguments
from head import HeadCompletion, tool as head_tool, arguments as head_arguments, card_degrees, COORDINATE_CONVENTION
from camera import CameraStream, CAMERA_SOURCES
from gripper import Gripper, tool as gripper_tool, command as gripper_command, MOTION_ACTIONS, recovery_arguments, normalize_arguments, GRIPPER_SIDES, combined_tool, combined_arguments

LOG = logging.getLogger('tron2')
VERSION = '0.14.0'
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
        if set(self.enabled) - (set(TOPICS) | set(CAMERA_SOURCES) | ({'arm_control', 'move_head', 'gripper', 'pose', 'head_visual_follow'} | set(GRIPPER_SIDES))):
            raise ValueError('Unknown TRON2 card in configuration')
        if 'head_visual_follow' in self.enabled and not {'joint_state','camera','move_head'} <= set(self.enabled):
            raise ValueError('头部跟随需要joint_state、camera和move_head')
        self.follow_runtime = None
        self.reader = reader or RobotReader(settings)
        self.registered_id = None
        self.core_ssl = True
        self.session = session
        self.reset = ResetPose(settings.get('home_file', '/config/tron2/home.json'), self.reader)
        self.motion = ArmCompletion(self.reader, self.report_completion)
        self.pose = Pose(self.reader,self.motion,self.reset)
        self.head = HeadCompletion(self.reader, self.report_head_completion)
        self.motion.pose_head = self.head
        self.grippers = {name: Gripper(self.reader, self.report_gripper_completion, side)
                         for name, side in GRIPPER_SIDES.items()}
        self.motion.peers = list(self.grippers.values()) + [self.head]
        self.head.peers = list(self.grippers.values()) + [self.motion]
        for owner in self.grippers.values():
            owner.peer = MotionPeers(self.motion, self.head)
            owner.pose_owner = self.motion
        self.pose.gripper_prepare = self.grippers['gripper_right'].prepare_pose
        self.motion.gripper_step = self.grippers['gripper_right'].execute_pose_step
        urls = settings.get('camera_urls', {})
        self.cameras = {name: CameraStream(urls.get(name, settings.get('camera_url', '') if name == 'camera' else ''),
                                          publish, session, topic=source[0])
                        for name, source in CAMERA_SOURCES.items()}
        self.camera = self.cameras['camera']

    async def report_completion(self, action_id, status, result):
        return await self._report_completion(result.get('motion_tool','arm_control'), action_id, status, result)

    async def report_gripper_completion(self, action_id, status, result):
        return await self._report_completion('gripper' if 'gripper' in self.enabled else 'gripper_' + result['side'], action_id, status, result)

    async def report_head_completion(self, action_id, status, result):
        return await self._report_completion('move_head', action_id, status, result)

    async def _report_completion(self, tool, action_id, status, result):
        if self.session is None:
            return False
        url = os.environ.get('AGENT_CORE_URL', 'https://phanthy-motus:15678').rstrip('/')
        payload = {'action_id': action_id, 'status': status, 'result': result,
                   'tool': tool, 'ts': time.time()}
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
        LOG.warning('Completion delivery failed; local motion owner retained')
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
        if 'pose' in self.enabled:
            tools.append(pose_tool())
        if 'move_head' in self.enabled:
            # Core may still inject metadata internally; it is not customer input.
            tools.append(head_tool())
        if 'gripper' in self.enabled:
            tools.append(combined_tool())
        for name, side in GRIPPER_SIDES.items():
            if name in self.enabled and 'gripper' not in self.enabled:
                tools.append(gripper_tool(metadata_properties(), side))
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
            if name in ({'arm_control', 'move_head', 'gripper', 'pose', 'head_visual_follow'} | set(GRIPPER_SIDES)) or name in CAMERA_SOURCES:
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
        if 'head_visual_follow' in self.enabled:
            from head_follow.engine import tool as follow_tool
            tools.append(follow_tool())
        return tools

    def snapshot(self):
        return {name: self.reader.sample(name) for name in self.enabled if name in TOPICS}

    def call(self, name, args):
        allowed = {'action', 'instance_id', '_trace_id'}
        if name == 'head_visual_follow':
            allowed |= {'input_topic','input_topics'}
        if name == 'arm_control':
            allowed |= {'joint', 'time'}
        if name == 'pose':
            allowed |= {'pose'}
        if name == 'move_head':
            allowed |= {'pitch_deg', 'yaw_deg', 'time'}
        if name == 'gripper':
            allowed |= {'side','opening_mm','speed','force'}
        if name in GRIPPER_SIDES:
            allowed |= {'opening_mm', 'opening', 'speed', 'force'}
        if not isinstance(args, dict) or set(args) - allowed:
            raise ValueError('Only action and Core instance_id/_trace_id metadata are accepted')
        # Core injects both fields after the model has selected a tool. Accept
        # bounded identifiers, but do not forward, log, or echo them as robot data.
        for key in ('instance_id', '_trace_id'):
            value = args.get(key, '')
            if not isinstance(value, str) or len(value) > 128 or any(
                    ord(c) < 32 or ord(c) == 127 for c in value):
                raise ValueError('Invalid Core metadata: ' + key)
        if name == 'head_visual_follow':
            if name not in self.enabled:
                raise ValueError('Unknown tool')
            return {'status':'validated','motion_sent':False}
        if name == 'model' and 'joint_state' in self.enabled:
            if 'action' in args:
                raise ValueError('Model has no action')
            return model()
        if name == 'pose' and name in self.enabled:
            if args.get('action') in ('info','start','stop'):
                # Some dashboards retain declared form values on lifecycle calls.
                # They are inactive here and must never become motor inputs.
                if args['action']=='stop':
                    return self.pose.stop()
                result=self.pose.info()
                if args['action']=='start':
                    result.update(state='ready',lifecycle_only=True)
                return result
            pose_arguments({k:v for k,v in args.items()})
            return {'status':'validated','motion_sent':False}
        if name == 'gripper' and name in self.enabled:
            if args.get('action') in ('info','start','stop'):
                if set(args)-{'action','side','instance_id','_trace_id'}:
                    raise ValueError('状态与生命周期操作不接受运动参数')
                side=args.get('side')
                if side is not None and side not in ('left','right'):
                    raise ValueError('请选择left左爪或right右爪')
                if args['action']=='stop':
                    for owner in self.grippers.values():
                        if side is None or owner.side==side: owner.cancel_waiting()
                result={'read_only':True,'motion_sent':False,'lifecycle_only':True,
                        'grippers':{k:v.info() for k,v in self.grippers.items()}}
                if args['action'] in ('start','stop'):
                    result['state']='ready' if args['action']=='start' else 'idle'
                return result
            side,action,values=combined_arguments(args)
            return {'action':action,'side':side,'arguments':values,'motion_sent':False,'status':'validated'}
        action = args.get('action', 'move' if name == 'move_head' else
                          'info' if name in TOPICS or name in CAMERA_SOURCES else None)
        if not isinstance(action, str):
            raise ValueError('An action string is required')
        if name not in self.enabled:
            raise ValueError('Unknown tool')
        if name == 'move_head':
            if action in ('info','start','stop'):
                if set(args) - {'action','instance_id','_trace_id'}:
                    raise ValueError('头部状态与生命周期操作不接受运动参数')
                if action == 'info':
                    _, q, dq = self.head.feedback()
                    pitch,yaw=card_degrees(q)
                    return {'read_only':True,'motion_sent':False,
                        'pitch_deg':pitch,'yaw_deg':yaw,'vendor_joint_rad':q,
                        'coordinate_convention':COORDINATE_CONVENTION,
                        'velocity_raw':dq,'motion':self.head.summary(),
                        'state':self.head.active['state'] if self.head.active else 'ready'}
                return {'motion_sent':False,'lifecycle_only':True,
                        'state':'ready' if action=='start' else 'idle',
                        'message':'生命周期操作不转动头部，也不构成硬件急停'}
            if action != 'move':
                raise ValueError('Unsupported head action')
            try:
                target, duration = head_arguments(args)
            except (ValueError, TypeError) as exc:
                raise MoveError(str(exc), False, 'head_invalid_command') from exc
            return {'action':'move','joint':target,'time':duration,
                    'target_pitch_deg':args['pitch_deg'],'target_yaw_deg':args['yaw_deg'],
                    'motion_sent':False,'status':'validated'}
        if name in GRIPPER_SIDES:
            args = normalize_arguments(action, args)
            if action == 'recover':
                recovery_arguments(args)
                return {'action': 'recover', 'read_only': True, 'motion_sent': False}
            if action in ('start', 'stop', 'info'):
                if set(args) - {'action', 'instance_id', '_trace_id'}:
                    raise ValueError('读取状态和系统操作无需开合参数')
                if action in ('start', 'stop'):
                    if action == 'stop':
                        self.grippers[name].cancel_waiting()
                    return {'state': 'ready' if action == 'start' else 'idle',
                            'motion_sent': False, 'lifecycle_only': True,
                            'message': 'stop取消本侧未受理的等待指令；不发送夹爪开合或硬件急停'}
                return self.grippers[name].info()
            gripper_command(action, args, GRIPPER_SIDES[name])
            return {'action': action, 'motion_sent': False, 'status': 'validated'}
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

    async def execute_tool(self, name, args, *, follow_guard=None):
        data = self.call(name, args)
        if name == 'head_visual_follow':
            if self.follow_runtime is None:
                raise ValueError('头部跟随尚未初始化')
            return await self.follow_runtime.call(args)
        if name in CAMERA_SOURCES:
            data = await self.cameras[name].call(args.get('action', 'info'))
        if name == 'arm_control' and data.get('action') in ('move', 'reset'):
            action = data['action']
            target = data['joint'] if action == 'move' else data['target_q']
            duration = data['time'] if action == 'move' else data['duration_seconds']
            details = {key: data[key] for key in ('pose_sha256',) if key in data}
            data = await self.motion.dispatch(action, target, duration, details)
        if name == 'pose' and data.get('status')=='validated':
            data=await self.pose.execute(args)
        if name == 'move_head' and data.get('action') == 'move':
            details = {key:data[key] for key in ('target_pitch_deg','target_yaw_deg')}
            if follow_guard is not None:
                details['head_follow_guard'] = follow_guard
            data = await self.head.dispatch('move',data['joint'],data['time'],details)
        if name == 'gripper':
            if data.get('status')=='validated':
                owner=self.grippers['gripper_'+data['side']]
                data=await owner.dispatch(data['action'],data['arguments'])
            elif args.get('action')=='info':
                side=args.get('side')
                if side: data=await self.grippers['gripper_'+side].read_state()
        if name in GRIPPER_SIDES:
            owner = self.grippers[name]
            action = args.get('action')
            if action == 'info':
                data = await owner.read_state()
            elif action == 'recover':
                data = await owner.recover(args)
            elif action in MOTION_ACTIONS:
                data = await owner.dispatch(action, args)
        return data

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
                    data = await self.execute_tool(params.get('name'), params.get('arguments', {}))
                    error = False
                except (ValueError, TypeError) as exc:
                    data, error = {'error': str(exc)[:240],
                        'read_only': params.get('name') not in self.enabled or params.get('name') not in ({'arm_control', 'move_head', 'gripper', 'pose', 'head_visual_follow'} | set(GRIPPER_SIDES)) or
                            params.get('name') in GRIPPER_SIDES and params.get('arguments', {}).get('action') not in MOTION_ACTIONS,
                        'motion_sent': getattr(exc, 'motion_sent', False)}, True
                    if params.get('name') == 'gripper' and 'gripper' in self.enabled:
                        arguments=params.get('arguments',{})
                        side=arguments.get('side') if isinstance(arguments,dict) else None
                        if side in ('left','right'):
                            data['gripper']=self.grippers['gripper_'+side].info()
                    if params.get('name') in self.grippers and params['name'] in self.enabled:
                        data['gripper'] = self.grippers[params['name']].info()
                    if params.get('name')=='pose' and 'pose' in self.enabled:
                        data['motion']=self.motion.summary()
                    if params.get('name') == 'move_head' and 'move_head' in self.enabled:
                        data['error_code'] = getattr(exc,'code',None)
                        data['head_motion'] = self.head.summary()
                result = {'content': [{'type': 'text', 'text': json.dumps(data,
                           ensure_ascii=False, allow_nan=False)}], 'isError': error}
            else:
                return web.json_response({'jsonrpc': '2.0', 'id': rid,
                    'error': {'code': -32601, 'message': 'Method not found'}})
            return web.json_response({'jsonrpc': '2.0', 'id': rid, 'result': result})
        except (ValueError, TypeError) as exc:
            return web.json_response({'jsonrpc': '2.0', 'id': rid,
                'error': {'code': -32602, 'message': str(exc)[:240]}})

    def health_data(self):
        samples = self.snapshot()
        ready = self.reader.connected and bool(samples) and all(s['available'] for s in samples.values())
        cameras = {name: stream.info() for name, stream in self.cameras.items() if name in self.enabled}
        if not samples and cameras:
            ready = all(info['available'] for info in cameras.values())
        return {'version': VERSION, 'read_only': not set(self.enabled) & ({'arm_control', 'move_head', 'gripper', 'pose'} | set(GRIPPER_SIDES)), 'ready': ready,
            'arm_motion': self.motion.summary(),
            'head_motion': self.head.summary(),
            'pose_motion': self.pose.motion_summary(),
            'grippers': {name: owner.info() for name, owner in self.grippers.items() if name in self.enabled or 'gripper' in self.enabled},
            'camera': self.camera.info() if 'camera' in self.enabled else None,
            'cameras': cameras,
            'robot_connected': self.reader.connected, 'registered_mcp_id': self.registered_id,
            'last_connection_error': self.reader.last_error,
            'round_trip_seconds': self.reader.last_round_trip, 'samples': samples}

    async def health(self, request):
        data = self.health_data()
        if self.follow_runtime is not None:
            data['head_visual_follow'] = await asyncio.to_thread(self.follow_runtime.info)
        return web.json_response(data,status=200 if data['ready'] else 503)

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
