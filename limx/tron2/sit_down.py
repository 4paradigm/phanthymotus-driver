"""SDK 3.8.3 / 3.8.5: zero velocity then one sitdown, no physical auto-retry."""
import asyncio
import os
import time
import uuid
from pathlib import Path
from stand import Stand, startup_match
from vendor import MoveError

class SitDown(Stand):
    STOP_SECONDS = 1.0
    BRAKE_INTERVAL = 1/35

    def __init__(self, reader, report):
        self.reader,self.report=reader,report
        self.state,self.result,self.task,self.pending='idle',None,None,None
        self.stages,self.events,self.motion_sent=[],[],False
        self.zero_velocity_frames=0
        reader.sit_down=self

    def dispatch(self):
        r=self.reader
        walking=getattr(r,'walk',None)
        if walking is not None and walking.task is not None and not walking.task.done():
            raise MoveError('行走正在执行，请先停止行走再进行起立或坐下',False)
        if self.task is not None and not self.task.done():
            raise MoveError('坐下恢复正在进行，不重复发送',False)
        if self.state=='unconfirmed' and (r.sample('robot_status').get('data') or {}).get('status') not in ('SIT','DAMPING') and not startup_match(r)['matched']:
            raise MoveError('上次坐下结果未确认，禁止自动重试；请现场检查或遥控恢复折叠',False)
        peer=getattr(r,'stand',None)
        if peer is not None and peer.task is not None and not peer.task.done():
            raise MoveError('起立流程仍在进行，请先等待其结束，不能并发发送坐下',False)
        if Path(os.environ.get('TRON2_MOTION_MAINTENANCE_FILE','/config/tron2/motion-maintenance.json')).exists():
            raise MoveError('驱动处于维护保护中，未发送坐下指令',False)
        if r.configuration.get('mode') != 'wheel_foot':
            raise MoveError('坐下仅支持已识别的轮足构型', False)
        data=self._read_status()['data']
        if data.get('status') not in ('STAND','WALK','SIT','DAMPING') and not startup_match(r)['matched']:
            raise MoveError('当前不在准备/站立/行走或已记录折叠状态，未发送坐下',False)
        if r.move_pending or r.head_pending or r._pose_servo is not None:
            raise MoveError('其他动作占用，未发送坐下',False)
        self._transition_stop_requested = False
        action_id=str(uuid.uuid4())
        self.state,self.result,self.stages,self.events,self.motion_sent='accepted',None,[],[],False
        self.zero_velocity_frames=0
        self.task=asyncio.create_task(self._run(action_id))
        return {'status':'accepted','action_id':action_id,'motion_sent':False,
                'completion':'fresh_stable_folded_startup_prepare','automatic_retry':False}

    def receive(self, body):
        p=self.pending
        if not p or body.get('accid')!=self.reader.serial:return
        title=body.get('title');data=body.get('data');result=data.get('result') if isinstance(data,dict) else None
        if title=='notify_twist' and self.state=='braking' and result!='success':
            p['error']='Zero velocity rejected: '+str(result);return
        if title not in ('response_sitdown','notify_sitdown','notify_invalid_request'):return
        correlated=body.get('guid')==p['guid']
        self.events.append({'phase':'sitdown','title':title,'result':result,'guid_matched':correlated})
        self.events=self.events[-16:]
        if not correlated:
            # Never use this alone for completion: folded pose and fresh frames required.
            if title=='notify_sitdown':p['uncorrelated_notify']=result
            return
        if result!='success' or title=='notify_invalid_request':p['error']='Vendor '+str(title)+': '+str(result)
        elif title=='response_sitdown':p['ack']=True
        else:p['notified']=True

    async def _stop_velocity(self):
        deadline=time.monotonic()+self.STOP_SECONDS
        while time.monotonic()<deadline:
            if self.pending and self.pending.get('error'):raise MoveError(self.pending['error'],self.motion_sent)
            mode=self._read_status()['data'].get('status')
            if mode in ('SIT','DAMPING'):return
            if mode not in ('WALK','STAND'):
                raise MoveError('停止过程中模式改变: '+str(mode),self.motion_sent)
            if mode=='WALK':
                self.motion_sent=None
                await asyncio.wait_for(self.reader.ws.send_json({'accid':self.reader.serial,
                    'title':'request_twist','guid':str(uuid.uuid4()),'timestamp':int(time.time()*1000),
                    'data':{'x':0.,'y':0.,'z':0.}}),1)
                self.motion_sent=True;self.zero_velocity_frames+=1
            await asyncio.sleep(self.BRAKE_INTERVAL)

    async def _wait_sit(self, after):
        deadline=time.monotonic()+self.PHASE_TIMEOUT
        mode=None
        while time.monotonic()<deadline:
            if self.pending and self.pending.get('error'):raise MoveError(self.pending['error'],self.motion_sent)
            status=self._read_status();mode=status['data'].get('status')
            if mode in ('SIT','DAMPING') and (status.get('received_at') or 0)>after:
                return {'target':'startup_prepare','vendor_status':mode,
                        'completion_source':'notify_robot_info.data.status'}
            if mode not in ('WALK','STAND','SIT','DAMPING','UNKNOW'):
                raise MoveError('坐下被异常模式中断: '+str(mode),self.motion_sent)
            await asyncio.sleep(self.POLL_SECONDS)
        raise MoveError('坐下超时: actual='+str(mode)+'；物理结果未确认，禁止自动重试',self.motion_sent)

    async def _run(self, action_id):
        try:
            mode=self._read_status()['data'].get('status')
            if mode in ('SIT','DAMPING') or startup_match(self.reader)['matched']:
                self.stages.append({'target':'startup_prepare','vendor_status':mode,
                                    'command_sent':False,'completion_source':'current_vendor_status'})
            else:
                self.state='braking'
                self.pending={'guid':str(uuid.uuid4()),'ack':False,'notified':False,'error':None}
                if mode=='WALK':await self._stop_velocity()
                mode=self._read_status()['data'].get('status')
                if mode in ('SIT','DAMPING'):
                    self.stages.append({'target':'startup_prepare','vendor_status':mode,
                                        'completion_source':'notify_robot_info.data.status'})
                else:
                    if mode not in ('STAND','WALK'):
                        raise MoveError('未匹配坐下入口: '+str(mode),self.motion_sent)
                    self.state='folding';after=time.time();self.motion_sent=None
                    await asyncio.wait_for(self.reader.ws.send_json({'accid':self.reader.serial,
                        'title':'request_sitdown','guid':self.pending['guid'],'timestamp':int(after*1000),'data':{}}),5)
                    self.motion_sent=True
                    self.stages.append(await self._wait_sit(after))
            self.result={'motion_sent':self.motion_sent,'target':'startup_prepare','completion_source':'vendor_status',
                         'stages':list(self.stages),'zero_velocity_frames':self.zero_velocity_frames,'automatic_retry':False}
            status='completed'
        except (Exception,asyncio.CancelledError) as exc:
            self.result={'motion_sent':self.motion_sent,'error':str(exc) or type(exc).__name__,
                         'phase':self.state,'stages':list(self.stages),'automatic_retry':False}
            status='error'
        finally:self.pending=None
        self.state='reporting'
        delivered=await self.report('sit_down',action_id,status,self.result)
        self.state='completed' if delivered and status=='completed' else 'unconfirmed'
        if self.state=='completed':
            peer=getattr(self.reader,'stand',None)
            if peer is not None:
                peer.state='idle';peer.result={'recovered_by':'sit_down','completion_source':'vendor_status','no_command_replayed':True}
