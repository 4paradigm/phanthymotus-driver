"""Owned two-axis MoveH tracking. Never writes an arm or whole-body setpoint."""
import asyncio
import math
import time
from .client import require, pose
from head import card_degrees
from vendor import motion_readiness, MoveError

CONTROL_SECONDS = .02
SEND_SECONDS = .1
MOVE_SECONDS = .25
LIMITS = ((-55.,30.),(-45.,45.))


class Limiter:
    """Velocity, acceleration and jerk bounded command integration in degrees."""
    def __init__(self, position):
        self.position = list(position)
        self.velocity = [0.,0.]
        self.acceleration = [0.,0.]

    def step(self, wanted, dt):
        require(type(dt) in (int,float) and math.isfinite(dt) and 0<dt<=.25, '控制周期失效；不追发积压目标')
        for i,(speed,(low,high)) in enumerate(zip((8.,12.),LIMITS)):
            goal=max(-speed,min(speed,wanted[i]))
            requested=max(-32.,min(32.,(goal-self.velocity[i])/max(.12,dt)))
            change=max(-120.*dt,min(120.*dt,requested-self.acceleration[i]))
            self.acceleration[i]+=change
            previous=self.velocity[i]
            self.velocity[i]=max(-speed,min(speed,previous+self.acceleration[i]*dt))
            if (goal-previous)*(goal-self.velocity[i])<=0:
                self.velocity[i]=goal
                # Retain bounded deceleration state instead of discontinuously
                # assigning a new position or sending a catch-up burst.
            self.position[i]=max(low,min(high,self.position[i]+self.velocity[i]*dt))
            if self.position[i] in (low,high):
                self.velocity[i]=self.acceleration[i]=0.
        return list(self.position)


class Stream:
    """Own the original HeadCompletion channel for an entire tracking session."""
    def __init__(self, adapter, guard, clock=time.monotonic):
        self.adapter,self.head,self.reader=adapter,adapter.head,adapter.reader
        self.guard,self.clock=guard,clock
        self.job=None
        self.reference=None
        self.limiter=None
        self.last_send=None
        self.last_target=None
        self.sent=0
        self.unknown=False
        self.verified=False
        self.samples=0
        self.error=None
        self.lease=None

    def read(self):
        h=self.adapter.health_data()
        status=self.reader.sample('robot_status')
        require(status.get('available') is True and motion_readiness(status.get('data') or {})['ready'],
                '机器人开发模式或保护状态无效；停止跟随')
        p=pose(h,stationary=self.job is None)
        if self.job is not None:
            require(self.head.active is self.job and self.reader.ws is self.job['ws'] and self.reader.serial==self.job['serial'],
                    '头部连接或会话所有权改变；不发送')
            require(max(abs(a-b) for a,b in zip(p['q'][:14],self.reference))<=math.radians(.5),
                    '机械臂参考姿态改变；停止跟随')
            require(not any(owner.active or getattr(owner,'recovering',False) or getattr(owner,'waiting',None)
                    for owner in self.head.peers),'其它执行器进入控制；停止跟随')
            if self.last_target is not None:
                require(max(abs(a-b) for a,b in zip((p['pitch'],p['yaw']),self.last_target))<=4.,
                        '头部反馈未跟上已发送目标；不累积新目标')
        return p

    async def start(self, action_id):
        await self.head.ensure_available()
        self.head.validate_admission_feedback()
        p=self.read()
        await self.reader.head_control_status()
        self.guard(False)
        self.head.validate_admission_feedback()
        again=self.read()
        require(max(abs(a-b) for a,b in zip(p['q'],again['q']))<=math.radians(.2),'模式核对期间姿态改变；未启用跟随')
        self.reference=list(again['q'][:14])
        self.limiter=Limiter([again['pitch'],again['yaw']])
        future=asyncio.get_running_loop().create_future();future.set_result(None)
        self.job={'action_id':action_id+'_head','action':'visual_follow','state':'tracking',
            'target':[-math.radians(again['pitch']),math.radians(again['yaw'])],
            'duration':MOVE_SECONDS,'details':{'motion_tool':'head_visual_follow'},
            'admission':future,'ws':self.reader.ws,'serial':self.reader.serial,
            'motion_sent':False,'vendor_acknowledged':False}
        self.head.active=self.job
        self.head.task=None  # Runtime attaches its one task before yielding again.
        self.lease=self.reader.pose_feedback();self.lease.__enter__()
        return again

    def before_send(self):
        try:
            self.guard(True)
            self.read()
        except ValueError as exc:
            # This guard runs before the reader creates/sends a WS request.
            # Preserve that definite no-send fact when pause races a mode probe.
            raise MoveError(str(exc),False,'head_follow_guard_rejected') from exc

    async def send(self, target):
        self.before_send()
        vendor,duration=self.head.validate([-math.radians(target[0]),math.radians(target[1])],MOVE_SECONDS)
        # Exact ACK serialization and mode recheck come from the existing
        # reader. Newer targets replace an old future target, never its ACK.
        try:
            result=await self.reader.send_moveh(vendor,duration,guard=self.before_send)
        except Exception as exc:
            if getattr(exc,'motion_sent',None) is not False:
                self.unknown=True
                self.job['motion_sent']=None
            raise
        require(result.get('motion_sent') is True,'头部发送结果未确认；不重发')
        self.last_target=list(target);self.last_send=self.clock();self.sent+=1
        self.job.update(target=vendor,motion_sent=True,vendor_acknowledged=True)

    async def finish(self):
        """No extra motor write: verify the last accepted target has settled."""
        require(not self.unknown,'上次发送结果未知；不自动解除头部所有权')
        self.job['state']='settling'
        deadline=self.clock()+MOVE_SECONDS+6.
        after=(self.last_send if self.last_send is not None else self.clock())+MOVE_SECONDS
        window=[];last_stamp=None
        while self.clock()<deadline:
            p=self.read()
            sample,q,dq=self.head.feedback()
            stamp=sample['data']['robot_timestamp'];now=self.clock()
            require(last_stamp is None or stamp>=last_stamp,'反馈时间回退；结果不明')
            target=self.last_target or self.limiter.position
            at_target=max(abs(a-b) for a,b in zip((p['pitch'],p['yaw']),target))<=1.
            if now>=after and at_target and all(abs(v)<=.01 for v in dq):
                if last_stamp is None or stamp>last_stamp:
                    if window and max(abs(a-b) for a,b in zip(q,window[0][1]))>math.radians(.2):window=[]
                    window.append((now,q))
                    if len(window)>=3 and now-window[0][0]>=.5:
                        self.verified=True;self.samples=len(window)
                        return p
            else:window=[]
            last_stamp=stamp
            await asyncio.sleep(.05)
        raise ValueError('最后头部目标未取得持续新反馈到位；保持保护，不重试')

    def release(self, delivered, error=None):
        if self.lease is not None:self.lease.__exit__(None,None,None);self.lease=None
        if self.job is None:return
        self.error=error
        safe=self.verified and not self.unknown and error is None
        self.head.last={'action_id':self.job['action_id'],'action':'visual_follow',
            'status':'completed' if safe else 'error','arrival_verified':safe,
            'motion_may_continue':not safe,'motion_sent':self.sent>0 or self.unknown,
            'callback_delivered':delivered,'verified_samples':self.samples,
            'verification':'advancing_head_feedback_after_tracking_stop','completion_protocol':'acp',
            'completion_scope':'parent_follow_session','completion_action_id':self.job['action_id'].removesuffix('_head'),
            **({'error':str(error)[:200]} if error else {})}
        if safe and delivered and self.head.active is self.job:self.head.active=None
        elif self.head.active is self.job:self.job['state']='blocked_requires_operator_check'
