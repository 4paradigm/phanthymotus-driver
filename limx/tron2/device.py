"""MCP discovery, sensor publication and guarded TRON2 motion calls."""
import asyncio
import json
import logging
import math
import os
import time

import aiohttp
from aiohttp import web
from vendor import MoveError, RobotReader, validate_movej, reject_constant, motion_readiness
from walk import Walk, form_number
from sit_down import SitDown
from stand import Stand
from reset import ResetPose
from skeleton import skeleton_stream, joints_stream, model
from state import battery_stream
from completion import ArmCompletion, MotionPeers, ACP_TIMEOUT
from pose import Pose, tool as pose_tool, arguments as pose_arguments
from head import HeadCompletion, tool as head_tool, arguments as head_arguments, card_degrees, COORDINATE_CONVENTION
from camera import CameraStream, CAMERA_SOURCES
from gripper import Gripper, tool as gripper_tool, command as gripper_command, MOTION_ACTIONS, recovery_arguments, normalize_arguments, GRIPPER_SIDES, combined_tool, combined_arguments

LOG = logging.getLogger('tron2')
VERSION = '0.25.0'
PORT = 15791
SERVER_NAME = 'LimX TRON2 Sensors and Arm Control'
TOPICS = {name: '/limx_tron2/' + name for name in
          ('joint_state',)}
# Existing skeleton topics remain public; all JSON sensors use this map.
SENSOR_TOPICS = {'joints':'/limx_tron2/state/joints','battery':'/limx_tron2/state/battery',**TOPICS}
DESCRIPTIONS = {
    'battery':'TRON2电池状态（只读，1Hz）：soc为剩余电量0～100%（首项）；current为有符号电流mA；cycle为非负整数充电循环数。bms_voltage为厂家原始电压读数；bms_current_valid为电流有效标志；bms_current_updated_at为厂家电流更新时间原值；bms_temp1～bms_temp5为5路温度采样原值（未确认倍率，不换算为摄氏度）。其他bms_字段按固件原名输出；不推测未公开字段语义。null表示暂无有效数据。',
    'joints':'TRON2当前构型骨架：按厂家ACCID自动识别上肢DACH（16关节）或下肢WF/SF（10关节），发布官方关节名及q/dq/tau；使用平台原生sensor/skeleton与独立model模型资源；下肢接入厂家真实IMU四元数imu_quat（w,x,y,z），无有效姿态时省略。',
    'camera': 'TRON2 头部RGB实时画面；start连接厂家MJPEG并发布JPEG图像，stop释放连接；限10fps，不录像、不控制头部，接收时间不是测量时间（只读）',
    'camera_left': 'TRON2 左腕RGB实时画面；厂家left校正图像，独立start/stop/info；限10fps，不录像、不控制机械臂，接收时间不是测量时间（只读）',
    'camera_right': 'TRON2 右腕RGB实时画面；厂家right校正图像，独立start/stop/info；限10fps，不录像、不控制机械臂，接收时间不是测量时间（只读）',
    'joint_state': '旧版骨架话题兼容入口；使用当前构型关节与官方URDF模型，推荐joints卡片。',
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
        if set(self.enabled) - (set(SENSOR_TOPICS) | set(CAMERA_SOURCES) | ({'loco_servo_arm', 'loco_leg', 'arm_control', 'move_head', 'gripper', 'pose', 'head_visual_follow'} | set(GRIPPER_SIDES))):
            raise ValueError('Unknown TRON2 card in configuration')
        if 'head_visual_follow' in self.enabled and (not {'camera','move_head'} <= set(self.enabled) or not {'joint_state','joints'} & set(self.enabled)):
            raise ValueError('头部跟随需要joint_state、camera和move_head')
        self._lower_stopping = False
        self._lower_closed = False
        self._lower_stop_task = None
        self.follow_runtime = None
        self.reader = reader or RobotReader(settings)
        self.stand = Stand(self.reader, self._report_lower_completion)
        self.sit_down = SitDown(self.reader, self._report_lower_completion)
        self.walk = Walk(self.reader, self._report_walk_completion)
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
        from arm_servo import ArmServoPlugin
        self.arm_servo = ArmServoPlugin(self, settings.get('loco_servo_arm', {}))

    async def _report_lower_completion(self, tool, action_id, status, result):
        # Core associates each instance with the unique action_id returned to it.
        public_tool='loco_leg' if 'loco_leg' in self.enabled else (tool)
        return await self._report_completion(public_tool,action_id,status,{**result,'action':({'stand':'stand_up','sit_down':'sit_down'}.get(tool,tool) if public_tool=='loco_leg' else tool)})

    async def _report_walk_completion(self,tool,action_id,status,result):
        return await self._report_completion('loco_leg' if 'loco_leg' in self.enabled else tool,action_id,status,result)

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

    def _loco_tool(self):
        return {'name':'loco_leg','type':'actuator','multiInstance':False,
            'description':'TRON2下肢运动：stand_up完整起立，sit_down停止并坐下，move定时移动，stop_move停止移动并保持站立。仅支持下肢构型；轮足无横移。vx范围-3～3 m/s（正前负后），vyaw约±22.5 deg/s（正左负右），默认均为0；duration为-1持续移动、0停止，或0.1～60秒，默认2秒。计时结束另发约1秒零速度，vyaw×duration不是目标转角，不保证精确距离或实际转角。',
            'inputSchema':{'type':'object','additionalProperties':False,
                'properties':{'action':{'type':'string','enum':['stand_up','sit_down','move','stop_move','start','stop','info'],'description':'选择要执行的动作'},
                    'vx':{'type':'number','minimum':-3,'maximum':3,'default':0,'title':'vx','description':'前后速度，单位m/s，范围-3～3，默认0；正值前进、负值后退、0不前后移动。'},
                    'vyaw':{'type':'number','minimum':-22.5,'maximum':22.5,'default':0,'title':'vyaw','description':'转向角速度，单位deg/s，范围约-22.5～22.5，默认0；正值左转、负值右转、0不转向。按现场标定z=vyaw/15换算，22.5deg/s对应厂家z=1.5；不是目标转角。'},
                    'duration':{'type':'number','minimum':-1,'maximum':60,'anyOf':[{'const':-1},{'const':0},{'minimum':.1,'maximum':60}],'default':2,'title':'duration','description':'运动时间，单位秒；-1持续移动直到stop_move或生命周期stop；0停止，0.1～60定时，默认2。停止后另发约1秒零速度。'},
                    **metadata_properties()},'required':['action'],
                'x-resource':['lower_body'],'x-completion':{'actions':['stand_up','sit_down','move'],'timeout':150},
                'x-action-params':{'stand_up':{'params':[],'description':'从启动/准备姿态完整起立并保持站立，厂家状态WALK表示完成；无速度和时间参数，不命令前进或转向。'},
                    'sit_down':{'params':[],'description':'停止移动并折叠坐回启动姿态；厂家状态SIT或DAMPING表示完成，无速度和时间参数。'},
                    'move':{'params':['vx','vyaw','duration'],'description':'轮足定时移动：vx为-3～3 m/s（正前负后），vyaw约±22.5 deg/s（正左负右），默认0；duration为-1持续移动、0停止或0.1～60秒，默认2秒。需要先起立进入WALK，结束后发送零速度，无横移。'},
                    'stop_move':{'params':[],'description':'请求零速度停止移动，保持站立，不执行坐下。'}}}}


    def tools(self):
        tools = []
        if 'loco_servo_arm' in self.enabled:
            tools.append(self.arm_servo.get_tool())
        if 'loco_leg' in self.enabled:
            tools.append(self._loco_tool())
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
            if name in ({'loco_servo_arm', 'loco_leg', 'arm_control', 'move_head', 'gripper', 'pose', 'head_visual_follow'} | set(GRIPPER_SIDES)) or name in CAMERA_SOURCES:
                continue
            tools.append({'name': name, 'type': 'sensor', 'multiInstance': False,
                'description': DESCRIPTIONS[name],
                'inputSchema': {'type': 'object', 'additionalProperties': False,
                    'properties': {'action': {'type': 'string', 'enum': ['info', 'start', 'stop']},
                                   **metadata_properties()},
                    'required': ['action']},
                'topic_out': [{'topic': SENSOR_TOPICS[name],
                               'format': 'sensor/skeleton' if name in ('joint_state', 'joints') else 'data/json'}]})
        if {'joint_state','joints'} & set(self.enabled):
            tools.append({'name': 'model', 'type': 'resource', 'multiInstance': False,
                'description': 'TRON2 URDF selected from the reported startup configuration: DACH dual arms/head, WF wheel-foot, SF sole-foot',
                'inputSchema': {'type': 'object', 'additionalProperties': False,
                    'properties': metadata_properties()}})
        if 'head_visual_follow' in self.enabled:
            from head_follow.engine import tool as follow_tool
            tools.append(follow_tool())
        return tools

    def _skeleton_display(self):
        """Return (state, reference_q) for the lower-limb display frame.

        The frame is captured once after startup and once after a completed
        sit_down, then held. A stand_up request switches to the live native
        frame. Live joint samples never recalculate an existing frame.
        """
        if self.reader.configuration.get('mode') not in ('wheel_foot', 'sole_foot'):
            return None, None
        frame = getattr(self.reader, '_skeleton_display_frame', None)
        if frame is None:
            frame = {'state': 'captured', 'reference_q': None,
                     'after': 0., 'source': 'startup'}
            self.reader._skeleton_display_frame = frame
        if frame.get('state') != 'captured':
            return 'live', None
        if frame.get('reference_q') is not None:
            return 'captured', list(frame['reference_q'])
        sample = self.reader.sample('joint_state')
        data = sample.get('data') if sample.get('available') else None
        q = (data or {}).get('q')
        if (sample.get('received_at') or 0) <= frame.get('after', 0.) \
                or not isinstance(q, list) or len(q) != 10 or any(
                    isinstance(v, bool) or not isinstance(v, (int, float))
                    or not math.isfinite(v) for v in q):
            return 'pending', None
        frame['reference_q'] = list(q)
        return 'captured', list(q)

    def skeleton_configuration(self):
        profile = self.reader.configuration
        if profile.get('state') == 'pending' and self.reader.info_at is not None:
            # Cover the short interval before main's one-shot task records it.
            from state import configuration
            return configuration(self.reader.serial)
        if profile.get('mode') in ('wheel_foot','sole_foot'):
            state, reference = self._skeleton_display()
            return {**profile, 'display_frame': state,
                    'display_reference_q': reference}
        return profile

    def snapshot(self):
        return {name: self.reader.sample({'battery':'battery_state','joints':'joint_state'}.get(name,name)) for name in self.enabled if name in SENSOR_TOPICS}

    def call(self, name, args):
        allowed = {'action', 'instance_id', '_trace_id'}
        if name == 'loco_servo_arm':
            allowed |= {'input_topic', 'input_topics', 'dry_run'}
        if name == 'head_visual_follow':
            allowed |= {'input_topic','input_topics','non_follow_action','speed'}
        if name == 'loco_leg':allowed |= {'vx','vy','vyaw','duration'}
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
        if name == 'loco_servo_arm' and name in self.enabled:
            if args.get('action', 'info') not in ('info', 'start', 'stop', 'pause', 'resume', 'config'):
                raise ValueError('Unsupported loco_servo_arm action')
            return {'status': 'validated', 'motion_sent': False}
        if name=='loco_leg' and name in self.enabled:
            action=args.get('action','info')
            if action=='stand_down':action='sit_down'  # Legacy saved canvas alias only.
            if action in ('info','start','stop'):
                result=self.walk.info()
                return {**result,'state':'ready' if action=='start' else 'idle' if action=='stop' else result['state'],'motion_sent':False}
            if action not in ('stand_up','sit_down','move','stop_move'):raise ValueError('Unsupported locomotion action')
            if action=='move':
                value=form_number(args.get('duration'),'duration',2)
                if isinstance(value,bool) or not isinstance(value,(int,float)) or not (value == -1 or 0<=value<=60):raise ValueError('duration must be -1, 0 or 0.1 to 60 seconds')
                if value not in (-1, 0) and value<.1:raise ValueError('duration must be -1, 0 or 0.1 to 60 seconds')
                if form_number(args.get('vy'),'vy',0)!=0:raise ValueError('Lateral travel is not supported; remove the legacy vy field or set it to 0')
                return {'status':'validated','duration':value,**{k:form_number(args.get(k),k,0) for k in ('vx','vyaw')}}
            return {'status':'validated'}
        if name == 'head_visual_follow':
            if name not in self.enabled:
                raise ValueError('Unknown tool')
            return {'status':'validated','motion_sent':False}
        if name == 'model' and {'joint_state','joints'} & set(self.enabled):
            if 'action' in args:
                raise ValueError('Model has no action')
            return {'urdf':model(self.skeleton_configuration())['urdf']}
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
                          'info' if name in SENSOR_TOPICS or name in CAMERA_SOURCES else None)
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
                  'always_on': True, 'topic_out': [{'topic': SENSOR_TOPICS[name],
                               'format': 'sensor/skeleton' if name in ('joint_state', 'joints') else 'data/json'}]}
        if action == 'info':
            result['sample'] = self.snapshot()[name]
        return result


    async def publish_loop(self):
        next_battery = 0.0
        while True:
            for name, data in self.snapshot().items():
                if name=='joints':
                    data=joints_stream(data,self.skeleton_configuration())
                elif name=='joint_state':
                    data=skeleton_stream(data,self.skeleton_configuration())
                elif name == 'battery':
                    now = time.monotonic()
                    if now < next_battery:
                        continue
                    next_battery = now + 1.0
                    data = battery_stream(data)
                self.publish(SENSOR_TOPICS[name], data)
            await asyncio.sleep(self.settings['poll_interval_seconds'])

    async def _finish_lower_tasks(self):
        owners = (self.stand, self.sit_down, self.walk)
        active = [o for o in owners if o.task is not None and not o.task.done()]
        for owner in active:
            if owner is not self.walk:
                owner._transition_stop_requested = True
        self.walk.stop()
        tasks = [o.task for o in active]
        for task in tasks:
            task.cancel()
        try:
            if tasks:
                # Let Walk send its bounded zero-velocity cleanup before joining.
                _, pending = await asyncio.wait(tasks, timeout=6)
                for task in pending:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
            for owner in active:
                if owner.state not in ('completed', 'stopped', 'unconfirmed'):
                    owner.pending = None
                    owner.state = 'unconfirmed'
                    owner.result = {'error': 'Lifecycle stop', 'motion_sent': getattr(owner, 'motion_sent', False),
                                    'physical_stop_verified': False}
            return {'state': 'idle', 'physical_stop_verified': False,
                    'motion_sent': any(bool((o.result or {}).get('zero_velocity_sent')) for o in active),
                    'owners': {name: owner.state for name, owner in
                               zip(('stand', 'sit_down', 'walk'), owners)},
                    'message': 'Driver任务已结束；厂家已接收的动作不可撤销，物理姿态未确认'}
        finally:
            if not self._lower_closed:
                self._lower_stopping = False

    async def close_lower(self, shutting_down=False):
        self._lower_closed = self._lower_closed or shutting_down
        self._lower_stopping = True
        if self._lower_stop_task is None or self._lower_stop_task.done():
            self._lower_stop_task = asyncio.create_task(self._finish_lower_tasks())
        return await asyncio.shield(self._lower_stop_task)

    async def execute_tool(self, name, args, *, follow_guard=None):
        if name == 'model':
            deadline = time.monotonic() + self.settings['response_timeout_seconds']
            while time.monotonic() < deadline:
                profile = self.skeleton_configuration()
                if profile.get('state') != 'pending' and (
                        profile.get('mode') not in ('wheel_foot','sole_foot') or
                        profile.get('display_frame') in ('captured','live') or
                        not getattr(self.reader, 'connected', False)):
                    break
                await asyncio.sleep(.05)
        data = self.call(name, args)
        if name == 'loco_leg':
            if args.get('action') == 'stop':
                return await self.close_lower()
            if self._lower_stopping and args.get('action') != 'info':
                raise ValueError('下肢任务正在停止，暂不接受新动作')
        if name == 'loco_servo_arm' and name in self.enabled:
            return await self.arm_servo.call(args)
        # Explicit commands outrank the stream. Await its in-flight send and
        # release writer ownership before admitting another motor owner.
        if getattr(self, 'arm_servo', None) is not None and (
                name == 'pose' and data.get('status') == 'validated' or
                name == 'arm_control' and data.get('action') in ('move', 'reset') or
                name == 'move_head' and data.get('action') == 'move' or
                (name == 'gripper' or name in GRIPPER_SIDES) and data.get('action') in ('open', 'close', 'recover') or
                name == 'head_visual_follow' and (args.get('action', 'follow') == 'follow' or
                    args.get('action') == 'not_follow' and args.get('non_follow_action') == 'reset')):
            await self.arm_servo.preempt()
        if name=='loco_leg':
            action=args.get('action','info')
            if action=='stand_down':action='sit_down'
            if action=='stand_up':return self.stand.dispatch()
            if action=='sit_down':return self.sit_down.dispatch()
            if action=='stop_move' or (action=='move' and data['duration']==0):return await self.walk.stop_move()
            if action=='move':return self.walk.dispatch_velocity(data['vx'],0.0,data['vyaw'],data['duration'])
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
                        'read_only': params.get('name') not in self.enabled or params.get('name') not in ({'loco_servo_arm', 'loco_leg', 'arm_control', 'move_head', 'gripper', 'pose', 'head_visual_follow'} | set(GRIPPER_SIDES)) or
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
        # An optional battery field must not regress existing joint readiness.
        required = {k: v for k, v in samples.items() if k in ('joint_state', 'joints')} or samples
        ready = self.reader.connected and bool(required) and all(s['available'] for s in required.values())
        cameras = {name: stream.info() for name, stream in self.cameras.items() if name in self.enabled}
        if not samples and cameras:
            ready = all(info['available'] for info in cameras.values())
        return {'walk':self.walk.info(), 'sit_down':self.sit_down.info(), 'stand': self.stand.info(), 'version': VERSION, 'read_only': not set(self.enabled) & ({'loco_servo_arm', 'loco_leg', 'arm_control', 'move_head', 'gripper', 'pose'} | set(GRIPPER_SIDES)), 'ready': ready,
            'loco_servo_arm': self.arm_servo.info(),
            'arm_motion': self.motion.summary(),
            'head_motion': self.head.summary(),
            'pose_motion': self.pose.motion_summary(),
            'grippers': {name: owner.info() for name, owner in self.grippers.items() if name in self.enabled or 'gripper' in self.enabled},
            'camera': self.camera.info() if 'camera' in self.enabled else None,
            'cameras': cameras,
            'configuration': dict(self.reader.configuration),
            'imu_subscription': getattr(self.reader, 'imu_subscription', None),
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
