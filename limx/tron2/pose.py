"""TRON2-only semantic arm frames; no motor I/O during planning or discovery."""
import asyncio
import errno
import math
import xml.etree.ElementTree as ET

import aiohttp
from completion import arm_feedback, POSE_ACP_TIMEOUT
from reset import ARM_NAMES
from skeleton import MODEL_PATH, skeleton_stream
from vendor import MoveError, motion_readiness, validate_movej
from trajectory import blocks

ACP_TIMEOUT = POSE_ACP_TIMEOUT
TELEOP_HEALTH_URL = 'http://127.0.0.1:15794/health'
LABELS = {'salute':'salute', 'wave':'wave', 'welcome':'welcome'}
PLANNED_DEG_PER_SECOND = 12.  # Unified planned travel/duration; not measured velocity.
WAVE_DEG_PER_SECOND = 24.  # Faster yaw-only swings; raising/returning stay at 12.
# Designed against TRON2 FK, not borrowed joint values from another robot.
LEFT_FRAMES = {
    'salute': [-75, 60, 0, -120, 0, 0, 0],
    'wave': [-65, 25, 0, -100, 0, 0, 0],
    'wave_out': [-65, 25, 20, -100, 0, 0, 0],
    'wave_in': [-65, 25, -20, -100, 0, 0, 0],
    'welcome': [-35, 45, 40, -65, 0, 0, 0],
}
NOMINAL_HOME = [0,0,84.5,0,0,0,0, 0,0,-84.5,0,0,0,0]
HEAD_CENTER = (.10, 0., .37)
HEAD_KEEP_OUT = .20  # Skeletal keep-out heuristic, not a collision CAD model.


def tool():
    return {'name':'pose','type':'actuator','multiInstance':False,
        'description':'TRON2预设轨迹：salute右臂敬礼、wave右臂挥手、welcome双臂欢迎。自检后自动将头部回正至pitch0/yaw0并确认到位，再执行平滑轨迹。挥手后右夹爪开合一次；敬礼、欢迎保持1秒，最后回原home。双臂须在home附近，区域清空。',
        'inputSchema':{'type':'object','additionalProperties':False,
            'properties':{
                'pose':{'type':'string','enum':list(LABELS.values()), 'title':'预设动作',
                    'oneOf':[{'const':label,'title':label} for label in LABELS.values()]}},
            'required':['pose'], 'x-resource':['arm_l','arm_r','head'],
            'x-completion':{'timeout':ACP_TIMEOUT}}}


def arguments(args):
    if not isinstance(args,dict) or set(args)-{'pose','instance_id','_trace_id'}:
        raise ValueError('pose只接受预设动作；手臂和速度由动作统一设定')
    label=args.get('pose')
    if not isinstance(label,str) or label not in LABELS.values():
        raise ValueError('请选择有效的预设动作')
    name=next(key for key,value in LABELS.items() if value==label)
    return name, 'both' if name=='welcome' else 'right'


def _mm(a,b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mv(a,b):
    return [sum(a[i][k]*b[k] for k in range(3)) for i in range(3)]


def _rot(axis,t):
    x,y,z=axis; length=math.sqrt(x*x+y*y+z*z);x,y,z=x/length,y/length,z/length
    c,s=math.cos(t),math.sin(t);v=1-c
    return [[c+x*x*v,x*y*v-z*s,x*z*v+y*s],
            [y*x*v+z*s,c+y*y*v,y*z*v-x*s],
            [z*x*v-y*s,z*y*v+x*s,c+z*z*v]]


class Geometry:
    """Pure FK and sampled skeletal exclusions; no environment/collision model."""
    def __init__(self):
        self.joints=[]
        for joint in ET.parse(MODEL_PATH).getroot().findall('joint'):
            origin=joint.find('origin')
            xyz=list(map(float,origin.get('xyz','0 0 0').split()))
            roll,pitch,yaw=map(float,origin.get('rpy','0 0 0').split())
            rotation=_mm(_mm(_rot([0,0,1],yaw),_rot([0,1,0],pitch)),_rot([1,0,0],roll))
            axis=(list(map(float,joint.find('axis').get('xyz').split()))
                  if joint.get('type')!='fixed' else None)
            self.joints.append((joint.get('name'),joint.find('parent').get('link'),
                joint.find('child').get('link'),xyz,rotation,axis))

    def positions(self,q):
        angles=dict(zip(ARM_NAMES,q))
        eye=[[1.,0,0],[0,1.,0],[0,0,1.]]
        world={'base_Link':(eye,[0.,0,0])}
        for name,parent,child,xyz,local,axis in self.joints:
            rotation,base=world[parent]
            position=[a+b for a,b in zip(base,_mv(rotation,xyz))]
            rotation=_mm(rotation,local)
            if axis is not None:
                rotation=_mm(rotation,_rot(axis,angles.get(name,0)))
            world[child]=(rotation,position)
        return {key:value[1] for key,value in world.items()}

    def validate_path(self,first,frames):
        last=list(first);minimum=math.inf; samples=0
        for frame in frames:
            target=frame['joint']
            count=max(1,math.ceil(max(abs(math.degrees(a-b)) for a,b in zip(last,target))/2))
            for i in range(0 if samples==0 else 1,count+1):
                q=[a+(b-a)*i/count for a,b in zip(last,target)]
                points=self.positions(q);samples+=1
                for suffix,sign in (('L',1),('R',-1)):
                    names=['elbow_'+suffix+'_Link','wrist_yaw_'+suffix+'_Link',
                        'wrist_pitch_'+suffix+'_Link','wrist_roll_'+suffix+'_Link','grasper_'+suffix+'_Link',
                        'display_hand_gripper_'+suffix+'_a_tip_Link']
                    chain=[points[name] for name in names]
                    if any(sign*p[1]<.14 for p in chain):
                        raise ValueError('预设参考路径过于靠近躯干中线；未发送动作')
                    for a,b in zip(chain,chain[1:]):
                        delta=[y-x for x,y in zip(a,b)];length=sum(x*x for x in delta)
                        t=0 if not length else max(0,min(1,sum((h-x)*d for h,x,d in zip(HEAD_CENTER,a,delta))/length))
                        distance=math.sqrt(sum((x+t*d-h)**2 for x,d,h in zip(a,delta,HEAD_CENTER)))
                        minimum=min(minimum,distance)
                        if distance<HEAD_KEEP_OUT:
                            raise ValueError('预设参考路径接近头部禁入区；未发送动作')
            last=list(target)
        return {'sampled_fk_points':samples,'minimum_head_reference_distance_m':round(minimum,4),
            'screening':'joint_linear_skeletal_reference_only','collision_checked':False}


class Pose:
    def __init__(self,reader,motion,reset):
        self.reader,self.motion,self.reset=reader,motion,reset
        self.geometry=Geometry()
        self.lock=asyncio.Lock()
        self.generation=0
        self.gripper_prepare=None

    def motion_summary(self):
        summary=self.motion.summary()
        return summary if (summary or {}).get('motion_tool')=='pose' else None

    def info(self):
        return {'read_only':True,'motion_sent':False,'catalog':LABELS,
            'planned_deg_per_second':PLANNED_DEG_PER_SECOND,
            'wave_planned_deg_per_second':WAVE_DEG_PER_SECOND,'motion':self.motion.summary(),
            'home':self.reset.call('info'),'returns_to_home':True,'collision_checked':False,
            'execution':'time_indexed_quintic_servoj','head_policy':'center_before_arm_trajectory',
            'servo_target_hz':300,'hard_realtime_guaranteed':False,
            'trajectory_peak_deg_per_second':{'arm':22.5,'wave_yaw':45.},
            'physical_presets_verified':False}

    def plan(self,args):
        name,side=arguments(args)
        home=self.reset.call('execute')  # Validates original file, identity and firmware.
        target_home=home['target_q']
        sample,current,velocities=arm_feedback(self.reader)
        stamp=(sample.get('data') or {}).get('robot_timestamp')
        if type(stamp) not in (int,float) or not math.isfinite(stamp):
            raise ValueError('预设动作需要有效的厂家反馈时间戳')
        if any(v is None or abs(v)>.05 for v in velocities) or len(velocities)!=14:
            raise ValueError('预设动作要求新鲜、完整且静止的双臂反馈')
        status=self.reader.sample('robot_status')
        if not status['available'] or not motion_readiness(status['data'] or {})['ready'] or status['data'].get('motor')!='OK':
            raise ValueError('机器人未处于可用的上层开发状态')
        nominal=list(map(math.radians,NOMINAL_HOME))
        if max(abs(math.degrees(a-b)) for a,b in zip(target_home,nominal))>3:
            raise ValueError('当前home不符合本套TRON2动作基准；保持原home，不自动改写')
        if max(abs(math.degrees(a-b)) for a,b in zip(current,target_home))>1:
            raise ValueError('请先使用arm_control的reset回到自然垂下home，再执行预设动作')
        packet=skeleton_stream(self.reader.sample('joint_state'))
        head={j['idx']:j for j in packet['joints'] if j['idx']>=14}
        if set(head)!={14,15} or any(j.get('dq') is None or abs(j['dq'])>.05 for j in head.values()):
            raise ValueError('预设动作要求新鲜且静止的头部反馈')
        from vendor import HEAD_RAD_LIMITS
        head_start=[head[i]['q'] for i in (14,15)]
        if any(not lo-math.radians(1)<=q<=hi+math.radians(1)
               for q,(lo,hi) in zip(head_start,HEAD_RAD_LIMITS)):
            raise ValueError('头部反馈超出机械范围，未发送预设动作')
        # Raise/rotate shoulders with the elbow and wrists unchanged, THEN
        # bend the elbow. Return by extending the elbow before lowering.
        goal=list(LEFT_FRAMES[name]);shoulder=goal[:3]+[0,0,0,0]
        targets=[('shoulder',shoulder),(name,goal)]
        if name=='wave':
            targets += [(label,list(LEFT_FRAMES[label])) for label in
                        ('wave_out','wave_in')*3]
        targets.append(('extend',targets[-1][1][:3]+[0,0,0,0]))
        frames=[];previous=list(current)
        for target_index,(frame_name,values) in enumerate(targets):
            q=list(current)
            for label,offset in (('left',0),('right',7)):
                if side not in (label,'both'):continue
                angles=values if label=='left' else [-v if i in (1,2,4,6) else v for i,v in enumerate(values)]
                q[offset:offset+7]=list(map(math.radians,angles))
                if frame_name in ('shoulder','extend'):
                    reference=current if frame_name=='shoulder' else target_home
                    q[offset+3:offset+7]=reference[offset+3:offset+7]
            rate=WAVE_DEG_PER_SECOND if frame_name in ('wave_out','wave_in') else PLANNED_DEG_PER_SECOND
            duration=max(1.,math.ceil(max(abs(math.degrees(a-b)) for a,b in zip(q,previous))/rate*10)/10)
            validate_movej(q,duration)
            hold=1. if frame_name==name and name!='wave' else 0
            frames.append({'label':frame_name,'joint':q,'time':duration,'hold':hold});previous=q
            if name=='wave' and target_index==len(targets)-2:
                # Hold the last yaw waypoint for one open/close cycle only.
                for action,opening in (('open',85),('close',0)):
                    frames.append({'kind':'gripper','label':'gripper_'+action+'_1',
                        'joint':list(q),'time':0,'hold':0,
                        'gripper':{'side':'right','opening_mm':opening,'speed':10,'force':10}})
        final_target=list(target_home)
        if side=='right':final_target[:7]=current[:7]  # No idle-left correction during right-only gestures.
        duration=max(2.,math.ceil(max(abs(math.degrees(a-b)) for a,b in zip(previous,final_target))/PLANNED_DEG_PER_SECOND*10)/10)
        validate_movej(final_target,duration)
        frames.append({'label':'home','joint':final_target,'time':duration,'hold':0})
        waypoints=frames
        frames=blocks(current,waypoints)
        screening_frames=[]
        for frame in frames:
            screening_frames.extend(frame['path'].screening_frames() if frame.get('kind')=='trajectory' else [frame])
        screening=self.geometry.validate_path(current,screening_frames)
        # A known centered head stays untouched. Otherwise center it inside the
        # same parent pose owner, before any arm/gripper command is possible.
        if any(abs(math.degrees(q))>1 for q in head_start):
            duration=max(2.,max(abs(math.degrees(q)) for q in head_start)/12.)
            frames.insert(0,{'kind':'head','label':'center_head','joint':list(current),
                'head_joint':[0.,0.],'time':duration,'hold':0})
        return {'frames':frames,'waypoints':waypoints,'start_head_q':head_start,'start_q':list(current),'preset':name,'side':side,'planned_deg_per_second':PLANNED_DEG_PER_SECOND,
            'wave_planned_deg_per_second':WAVE_DEG_PER_SECOND if name=='wave' else None,
            'home_q':list(target_home),'pose_sha256':home['pose_sha256'],'motion_tool':'pose',
            'execution':'time_indexed_quintic_servoj','head_policy':'center_before_arm_trajectory','gripper_policy':'right_open_close_once_ends_closed' if name=='wave' else 'unchanged','returns_to_home':True,
            'physical_presets_verified':False,
            'planned_motion_seconds':round(sum(f['time']+f['hold'] for f in frames),1),
            'planned_motion_seconds_excludes_gripper_waits':name=='wave',**screening}

    async def guard(self):
        # Local known teleop source must be inactive; never toggle its lifecycle.
        try:
            async with aiohttp.ClientSession(trust_env=False,timeout=aiohttp.ClientTimeout(total=2)) as session:
                async with session.get(TELEOP_HEALTH_URL,allow_redirects=False) as response:
                    body=await response.json()
                    if response.status not in (200,503) or not isinstance(body,dict) or type(body.get('active')) is not bool or body['active'] or body.get('output_active') is not False:
                        raise ValueError('遥操运行中或状态不明；未发送新的动作段')
        except aiohttp.ClientConnectorError as exc:
            if getattr(exc.os_error,'errno',None)!=errno.ECONNREFUSED:
                raise ValueError('无法确认本机遥操状态；未发送新的动作段') from exc
        except (aiohttp.ClientError,asyncio.TimeoutError,OSError) as exc:
            raise ValueError('无法确认本机遥操状态；未发送新的动作段') from exc
        await self.reader.head_control_status()  # Reject native VR following; never change it.

    async def execute(self,args):
        arguments(args)  # Reject bad requests before any asynchronous check or task.
        if self.lock.locked():raise MoveError('正在审核另一条预设动作',False)
        async with self.lock:
            ticket=self.generation
            ws,serial=self.reader.ws,self.reader.serial
            def same_connection():
                if not self.reader.connected or self.reader.ws is not ws or self.reader.serial!=serial:
                    raise MoveError('预设审核期间连接已改变；请重新发起请求',False)
            same_connection()
            await self.guard()
            same_connection()
            if ticket!=self.generation:raise MoveError('预设审核已被停止请求取消',False)
            plan=await asyncio.to_thread(self.plan,dict(args))
            same_connection()
            if ticket!=self.generation:raise MoveError('预设规划已被停止请求取消',False)
            if plan['preset']=='wave':
                if self.gripper_prepare is None:raise MoveError('挥手夹爪步骤未配置，未发送动作',False)
                await self.gripper_prepare()
                same_connection()
                if ticket!=self.generation:raise MoveError('夹爪审核已被停止请求取消',False)
            async def stage_guard():
                same_connection()
                if ticket!=self.generation and (self.motion.active or {}).get('stage',0)==0:
                    raise MoveError('停止请求已取消预设首段',False)
                await self.guard()
                same_connection()
                if ticket!=self.generation and (self.motion.active or {}).get('stage',0)==0:
                    raise MoveError('停止请求已取消预设首段',False)
                if self.reset.load()[2]!=plan['pose_sha256']:
                    raise ValueError('home文件已变化；取消剩余动作，不自动使用新home')
            return await self.motion.dispatch_sequence(plan,stage_guard)

    def stop(self):
        self.generation+=1
        pending=self.lock.locked()
        requested=self.motion.cancel_sequence()
        return {'state':'idle','motion_sent':False,'lifecycle_only':True,'cancel_requested':requested or pending,
            'message':'取消尚未发送的后续动作段；已发送的实体动作不能由此急停'}
