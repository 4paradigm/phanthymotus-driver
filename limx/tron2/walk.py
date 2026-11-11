"""Timed wheel-foot twist commands (SDK 3.8.3), no files or pose changes."""
import asyncio
import math
import logging
import time
import uuid
from stand import Stand
from vendor import MoveError

LOG = logging.getLogger('tron2')

# Operator observation: vendor z=1.5 for 4 s produced ~360 degrees.
# Empirical average mapping, not verified steady-state or angle feedback control.
YAW_DEG_S_PER_VENDOR_UNIT = 60.0
MAX_YAW_DEG_S = 90.0

DIRECTIONS = {'forward':(.2,0.,0.), 'backward':(-.2,0.,0.),
              'turn_left':(0.,0.,.3), 'turn_right':(0.,0.,-.3)}


def form_number(value,name,default):
    if value is None or (isinstance(value,str) and not value.strip()):return float(default)
    if isinstance(value,bool):raise ValueError(name+' must be a finite number')
    if isinstance(value,str):
        if len(value)>64:raise ValueError(name+' must be a finite number')
        try:value=float(value.strip())
        except ValueError:raise ValueError(name+' must be a finite number') from None
    if not isinstance(value,(int,float)):raise ValueError(name+' must be a finite number')
    try:valid=math.isfinite(value)
    except OverflowError:valid=False
    if not valid:raise ValueError(name+' must be a finite number')
    return float(value)


def duration(value):
    if isinstance(value,bool) or not isinstance(value,(int,float)):
        raise ValueError('行走时间须为-1（持续移动）或0.1至60秒')
    try: valid=math.isfinite(value) and (value == -1 or .1<=value<=60)
    except OverflowError:valid=False
    if not valid:raise ValueError('行走时间须为-1或0.1至60秒的有限数字')
    return float(value)


class Walk:
    PERIOD=1/35
    STOP_SECONDS=1.

    def __init__(self,reader,report):
        self.reader,self.report=reader,report
        self.state,self.result,self.task='idle',None,None
        self.error=None;self.stop_requested=False
        self.first_nonzero_at=self.first_zero_at=None
        self.nonzero_frames=self.zero_frames=0
        reader.walk=self

    def info(self):
        return {'state':self.state,'result':self.result,'automatic_retry':False,
                'linear_speed_m_s':.2,'angular_speed_deg_s':.3*YAW_DEG_S_PER_VENDOR_UNIT}

    def stop(self):
        if self.task is not None and not self.task.done():self.stop_requested=True
        return {**self.info(),'stop_requested':self.stop_requested,'motion_sent':False}

    def receive(self,body):
        if self.state not in ('moving','stopping') or body.get('accid')!=self.reader.serial:return
        if body.get('title') in ('notify_twist','notify_invalid_request'):
            data=body.get('data') or {}
            if not isinstance(data,dict) or data.get('result')!='success':
                self.error='厂家速度控制失败: '+str(data.get('result') if isinstance(data,dict) else 'invalid_reply')

    def disconnect(self):
        if self.task is not None and not self.task.done():self.error='机器人断连，移动结果未确认'

    def _ready(self):
        sample=Stand._read_status(self)
        if sample['data'].get('status')!='WALK':
            raise MoveError('需要先执行stand完成起立，厂家status须为WALK',False)
        return sample

    def dispatch(self,action,seconds):
        if action not in DIRECTIONS:raise ValueError('Unknown direction')
        return self._dispatch(action,duration(seconds),DIRECTIONS[action])

    def dispatch_velocity(self,vx,vy,vyaw,seconds):
        values=[]
        seconds=form_number(seconds,'duration',2)
        for name,value,limit in [('vx',vx,3.),('vy',vy,0.),('vyaw',vyaw,MAX_YAW_DEG_S)]:
            value=form_number(value,name,0)
            try:valid=math.isfinite(value) and abs(value)<=limit
            except OverflowError:valid=False
            if not valid:raise ValueError(name+' exceeds the supported wheel-foot range')
            values.append(float(value))
        return self._dispatch('move',duration(seconds),(values[0],values[1],values[2]/YAW_DEG_S_PER_VENDOR_UNIT))

    def _dispatch(self,action,seconds,velocity):
        if self.state not in ('idle','completed','stopped'):
            raise MoveError('行走正在进行或结果未确认，不重复发送',False)
        if self.reader.configuration.get('mode')!='wheel_foot':
            raise MoveError('当前行走卡片仅支持轮足构型',False)
        self._ready()
        for name in ('stand','sit_down'):
            owner=getattr(self.reader,name,None)
            if owner is not None and owner.task is not None and not owner.task.done():
                raise MoveError('下肢动作正在执行，请等待完成',False)
        if self.reader.move_pending or self.reader.head_pending or self.reader._pose_servo is not None:
            raise MoveError('其他运动请求正在执行',False)
        self.state,self.result,self.error,self.stop_requested='accepted',None,None,False
        self.first_nonzero_at=self.first_zero_at=None
        self.nonzero_frames=self.zero_frames=0
        action_id=str(uuid.uuid4())
        self.task=asyncio.create_task(self._run(action_id,action,seconds,velocity))
        LOG.info('Walk accepted action_id=%s duration=%s vyaw_deg_s=%s vendor_z=%s',
                 action_id,seconds,velocity[2]*YAW_DEG_S_PER_VENDOR_UNIT,velocity[2])
        result = {'status':'accepted','action_id':action_id,'motion_sent':False,
                'direction':action,'time_seconds':seconds,'vx':velocity[0],'vy':velocity[1],
                'vyaw':velocity[2]*YAW_DEG_S_PER_VENDOR_UNIT,'duration':seconds,'vendor_z_command':velocity[2]}
        if seconds == -1:
            # A continuous stream has no finite ACP completion deadline.
            result.pop('action_id')
            result.update(continuous=True, completion='explicit_stop_required')
        return result

    async def stop_move(self):
        if self.task is not None and not self.task.done():
            return self.stop()
        if self.reader.configuration.get('mode')!='wheel_foot':raise MoveError('Wheel-foot mode required',False)
        self._ready()
        for name in ('stand','sit_down'):
            owner=getattr(self.reader,name,None)
            if owner is not None and owner.task is not None and not owner.task.done():raise MoveError('Another lower-body action is active',False)
        self.state='stopping'
        try:
            sent=await self._zero(self.reader.ws)
        except Exception:
            self.state='unconfirmed';raise MoveError('Zero velocity send failed; do not retry automatically',None)
        self.state='stopped'
        return {'state':'stopped','ret':0,'motion_sent':sent,'physical_stop_verified':False}

    async def _send(self,velocity,timeout=.5):
        attempted_at=time.monotonic()
        await asyncio.wait_for(self.reader.ws.send_json({'title':'request_twist',
            'accid':self.reader.serial,'guid':str(uuid.uuid4()),'timestamp':int(time.time()*1000),
            'data':dict(zip(('x','y','z'),velocity))}),timeout)
        if any(velocity):
            self.nonzero_frames+=1
            if self.first_nonzero_at is None:self.first_nonzero_at=attempted_at
        else:
            self.zero_frames+=1
            if self.first_zero_at is None:self.first_zero_at=attempted_at

    async def _zero(self,ws):
        if self.reader.ws is not ws or ws.closed or not self.reader.connected:
            raise MoveError('连接已变化，无法确认零速度发送；不会重连重放',None)
        mode=(self.reader.sample('robot_status').get('data') or {}).get('status')
        if mode in ('STAND','SIT','DAMPING'):return False
        deadline=time.monotonic()+self.STOP_SECONDS
        # Always send at least one zero frame, including a failed first move send.
        first=True
        while first or time.monotonic()<deadline:
            first=False
            if self.reader.ws is not ws or ws.closed or not self.reader.connected:
                raise MoveError('停止时断连，物理停止未确认',None)
            await self._send((0.,0.,0.))
            await asyncio.sleep(self.PERIOD)
        return True

    async def _run(self,action_id,action,seconds,velocity):
        ws=self.reader.ws;sent=False;failure=None;zero_sent=False;started=time.monotonic()
        try:
            self.state='moving';deadline=math.inf if seconds == -1 else started+seconds;next_frame=started
            while time.monotonic()<deadline and not self.stop_requested:
                if self.error:raise MoveError(self.error,sent)
                self._ready()
                if self.reader.ws is not ws:raise MoveError('连接已变化，未继续移动',sent)
                sent=None  # Attempted send may have reached the robot even if it times out.
                remaining=deadline-time.monotonic()
                if remaining<=0:break
                await self._send(velocity,timeout=min(.5,remaining));sent=True
                next_frame=max(next_frame+self.PERIOD,time.monotonic())
                await asyncio.sleep(max(0,min(next_frame,deadline)-time.monotonic()))
            if self.error:raise MoveError(self.error,sent)
        except (Exception,asyncio.CancelledError) as exc:
            failure=str(exc) or type(exc).__name__
        finally:
            self.state='stopping'
            try:
                if sent is not False:
                    zero_sent=await self._zero(ws)
            except (Exception,asyncio.CancelledError) as exc:
                failure=(failure+'; ' if failure else '')+'零速度发送未确认: '+(str(exc) or type(exc).__name__)
        if not failure and sent is not False:
            try:
                self._ready()
                if self.error:raise MoveError(self.error,sent)
            except Exception as exc:failure=str(exc)
        stopped=self.stop_requested
        self.result={'direction':action,'vx':velocity[0],'vy':velocity[1],'vyaw':velocity[2]*YAW_DEG_S_PER_VENDOR_UNIT,'duration':seconds,'time_seconds':seconds,'motion_sent':sent,
            'zero_velocity_sent':zero_sent,'physical_stop_verified':False,
            'vendor_z_command':velocity[2], 'commanded_vyaw_deg_s':velocity[2]*YAW_DEG_S_PER_VENDOR_UNIT, 'nonzero_frames':self.nonzero_frames,
            'zero_velocity_frames':self.zero_frames,
            'command_stream_seconds':None if self.first_nonzero_at is None or self.first_zero_at is None else self.first_zero_at-self.first_nonzero_at,
            'angle_control':False,
            'automatic_retry':False,'stopped_early':stopped,
            'completion':'timed_command_stream_finished_and_zero_velocity_sent'}
        if failure:self.result['error']=failure
        LOG.info('Walk ended action_id=%s command_stream_seconds=%s nonzero_frames=%s zero_frames=%s failure=%s',
                 action_id,self.result['command_stream_seconds'],self.nonzero_frames,self.zero_frames,failure)
        self.state='reporting'
        delivered=True if seconds == -1 else await self.report('walk',action_id,'error' if failure or stopped else 'completed',self.result)
        self.state='unconfirmed' if failure or not delivered else ('stopped' if stopped else 'completed')
