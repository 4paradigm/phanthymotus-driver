"""TRON2-only semantic arm frames; no motor I/O during planning or discovery."""
import asyncio
import errno
import math
import xml.etree.ElementTree as ET

import aiohttp
from completion import arm_feedback
from reset import ARM_NAMES
from skeleton import MODEL_PATH, skeleton_stream
from vendor import MoveError, motion_readiness, validate_movej

VERSION = 1
ACP_TIMEOUT = 300
TELEOP_HEALTH_URL = 'http://127.0.0.1:15794/health'
LABELS = {'salute':'敬礼', 'wave':'挥手打招呼', 'welcome':'欢迎', 'reset':'自然垂下回正'}
SPEEDS = {'slow':4., 'normal':6.}  # Planned maximum joint travel / duration, deg/s.
# Designed against TRON2 FK, not borrowed joint values from another robot.
LEFT_FRAMES = {
    'fold': [0, 0, 84.5, -45, 0, 0, 0],
    'prepare': [-25, 25, 65, -70, 0, 0, 0],
    'salute': [-75, 25, 0, -120, 0, 0, 0],
    'wave': [-65, 25, 0, -100, 0, 0, 0],
    'wave_out': [-65, 40, 0, -100, 0, 0, 0],
    'wave_in': [-65, 20, 0, -100, 0, 0, 0],
    'welcome': [-35, 45, 40, -65, 0, 0, 0],
}
NOMINAL_HOME = [0,0,84.5,0,0,0,0, 0,0,-84.5,0,0,0,0]
HEAD_CENTER = (.10, 0., .37)
HEAD_KEEP_OUT = .20  # Skeletal keep-out heuristic, not a collision CAD model.


def tool():
    return {'name':'pose','type':'actuator','multiInstance':False,
        'description':'TRON2预设双臂动作：敬礼、挥手、欢迎及自然垂下回正。完成动作后回原home；保持头部和夹爪开口。执行前双臂需在home附近、头部朝正前方，运动范围无人或障碍。',
        'inputSchema':{'type':'object','additionalProperties':False,
            'properties':{
                'pose':{'type':'string','enum':list(LABELS), 'title':'预设动作',
                    'oneOf':[{'const':key,'title':label} for key,label in LABELS.items()]},
                'side':{'type':'string','enum':['left','right','both'],'default':'right','title':'手臂',
                    'description':'敬礼、挥手选left左臂或right右臂；欢迎、回正固定双臂'},
                'speed':{'type':'string','enum':['slow','normal'],'default':'slow','title':'速度',
                    'description':'slow慢速；normal标准。仅决定计划时长，不代表已测得物理速度'}},
            'required':['pose'], 'x-resource':['arm_l','arm_r','head'],
            'x-completion':{'timeout':ACP_TIMEOUT}}}


def arguments(args):
    if not isinstance(args,dict) or set(args)-{'pose','side','speed','instance_id','_trace_id'}:
        raise ValueError('pose只接受预设动作、手臂和速度；不接受自定义关节或action参数')
    name,side,speed=args.get('pose'),args.get('side','right'),args.get('speed','slow')
    if name not in LABELS or side not in ('left','right','both') or speed not in SPEEDS:
        raise ValueError('请选择有效的预设动作、手臂和速度')
    if name in ('salute','wave') and side=='both':
        raise ValueError('敬礼和挥手只支持单臂；请选择left或right')
    return name, 'both' if name in ('welcome','reset') else side, speed


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
        self.joints=ET.parse(MODEL_PATH).getroot().findall('joint')

    def positions(self,q):
        angles=dict(zip(ARM_NAMES,q))
        eye=[[1.,0,0],[0,1.,0],[0,0,1.]]
        world={'base_Link':(eye,[0.,0,0])}
        for joint in self.joints:
            rotation,base=world[joint.find('parent').get('link')]
            origin=joint.find('origin')
            xyz=list(map(float,origin.get('xyz','0 0 0').split()))
            roll,pitch,yaw=map(float,origin.get('rpy','0 0 0').split())
            position=[a+b for a,b in zip(base,_mv(rotation,xyz))]
            local=_mm(_mm(_rot([0,0,1],yaw),_rot([0,1,0],pitch)),_rot([1,0,0],roll))
            rotation=_mm(rotation,local)
            if joint.get('type')!='fixed':
                axis=list(map(float,joint.find('axis').get('xyz').split()))
                rotation=_mm(rotation,_rot(axis,angles.get(joint.get('name'),0)))
            world[joint.find('child').get('link')]=(rotation,position)
        return {key:value[1] for key,value in world.items()}

    def validate_path(self,first,frames):
        last=list(first);minimum=math.inf; samples=0
        for frame in frames:
            target=frame['joint']
            count=max(1,math.ceil(max(abs(math.degrees(a-b)) for a,b in zip(last,target))/2))
            for i in range(count+1):
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

    def motion_summary(self):
        summary=self.motion.summary()
        return summary if (summary or {}).get('motion_tool')=='pose' else None

    def info(self):
        return {'read_only':True,'motion_sent':False,'catalog':LABELS,
            'speed_options':list(SPEEDS),'motion':self.motion.summary(),
            'home':self.reset.call('info'),'returns_to_home':True,'collision_checked':False,
            'physical_presets_verified':False}

    def plan(self,args):
        name,side,speed=arguments(args)
        home=self.reset.call('execute')  # Validates original file, identity and firmware.
        target_home=home['target_q']
        sample,current,velocities=arm_feedback(self.reader)
        stamp=(sample.get('data') or {}).get('robot_timestamp')
        if type(stamp) not in (int,float) or not math.isfinite(stamp):
            raise ValueError('预设动作需要有效的厂家反馈时间戳')
        if any(v is None or abs(v)>.05 for v in velocities) or len(velocities)!=14:
            raise ValueError('预设动作要求新鲜、完整且静止的双臂反馈')
        if not motion_readiness(self.reader.sample('robot_status')['data'] or {})['ready']:
            raise ValueError('机器人未处于可用的上层开发状态')
        nominal=list(map(math.radians,NOMINAL_HOME))
        if name!='reset':
            if max(abs(math.degrees(a-b)) for a,b in zip(target_home,nominal))>3:
                raise ValueError('当前home不符合本套TRON2动作基准；保持原home，不自动改写')
            if max(abs(math.degrees(a-b)) for a,b in zip(current,target_home))>1:
                raise ValueError('请先使用回正回到自然垂下home，再执行预设动作')
            packet=skeleton_stream(self.reader.sample('joint_state'))
            head={j['idx']:j for j in packet['joints'] if j['idx']>=14}
            if set(head)!={14,15} or any(abs(math.degrees(j['q']))>10 or
                    j.get('dq') is None or abs(j['dq'])>.05 for j in head.values()):
                raise ValueError('预设动作要求头部静止且俯仰、偏航均在正前方±10°以内')
        names=[]
        if name!='reset':
            names=['fold','prepare',name]
            if name=='wave':names+=['wave_out','wave_in','wave_out','wave_in']
            names+=['prepare','fold']
        frames=[];previous=list(current)
        for frame_name in names:
            q=list(current)
            for label,offset in (('left',0),('right',7)):
                if side not in (label,'both'):continue
                values=LEFT_FRAMES[frame_name]
                if label=='right':values=[-v if i in (1,2,4,6) else v for i,v in enumerate(values)]
                q[offset:offset+7]=list(map(math.radians,values))
            duration=max(2.,math.ceil(max(abs(math.degrees(a-b)) for a,b in zip(q,previous))/SPEEDS[speed]*10)/10)
            validate_movej(q,duration)
            hold=.8 if frame_name==name and name!='wave' else 0
            frames.append({'label':frame_name,'joint':q,'time':duration,'hold':hold});previous=q
        duration=max(8.,math.ceil(max(abs(math.degrees(a-b)) for a,b in zip(previous,target_home))/SPEEDS[speed]*10)/10)
        validate_movej(target_home,duration)
        frames.append({'label':'home','joint':list(target_home),'time':duration,'hold':0})
        screening=self.geometry.validate_path(current,frames) if name!='reset' else {
            'screening':'existing_home_reset_only','collision_checked':False}
        return {'frames':frames,'start_q':list(current),'preset':name,'side':side,'speed':speed,
            'home_q':list(target_home),'pose_sha256':home['pose_sha256'],'motion_tool':'pose',
            'head_policy':'unchanged','gripper_policy':'unchanged','returns_to_home':True,
            'physical_presets_verified':False,
            'planned_motion_seconds':round(sum(f['time']+f['hold'] for f in frames),1),**screening}

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
