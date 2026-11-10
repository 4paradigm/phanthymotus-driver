"""SDK 3.8.1/3.8.2: prepare then vendor balance, with fresh mode verification."""
import asyncio
import json
import math
import os
import time
import uuid
from pathlib import Path
from vendor import MoveError


def startup_match(reader):
    """Match the operator-recorded folded pose; never rename vendor UNKNOW to READY."""
    try:
        path = os.environ.get('TRON2_STARTUP_PROFILE')
        if not path:
            return {'matched': False, 'reason': 'profile_not_configured'}
        profile = json.loads(Path(path).read_text())
        status, joints = reader.sample('robot_status'), reader.sample('joint_state')
        data, feedback = status.get('data') or {}, joints.get('data') or {}
        if not status.get('available') or not joints.get('available'):
            return {'matched': False, 'reason': 'stale_feedback'}
        if reader.serial != profile['accid'] or reader.configuration.get('mode') != profile['mode']:
            return {'matched': False, 'reason': 'different_robot_or_configuration'}
        expected = {k:profile[k] for k in ('sw_version','working_mode')}
        if data.get('status') not in profile.get('folded_statuses',[profile['status']]):
            return {'matched':False,'reason':'not_folded_mode'}
        expected.update(imu='OK', motor='OK', working_mode_valid=True,
                        collision_enable_valid=True, collision_enable=True,
                        compliant_enable_valid=True, compliant_enable=False)
        if any(type(data.get(k)) is not type(v) or data.get(k) != v for k,v in expected.items()):
            return {'matched': False, 'reason': 'different_status_or_firmware'}
        q, dq = feedback.get('q'), feedback.get('dq')
        if feedback.get('joint_count') != 10 or not isinstance(q,list) or not isinstance(dq,list) or len(q)!=10 or len(dq)!=10:
            return {'matched': False, 'reason': 'invalid_joints'}
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in q+dq):
            return {'matched': False, 'reason': 'invalid_joints'}
        if any(abs(v)>profile['stationary_dq_limit_rad_s'] for v in dq):
            return {'matched': False, 'reason': 'joints_not_stationary'}
        ranges = profile['pose_ranges']
        if any(not ranges[str(i)][0] <= q[i] <= ranges[str(i)][1] for i in profile['pose_indices']):
            return {'matched': False, 'reason': 'different_folded_pose'}
        return {'matched': True, 'state': profile['name'], 'vendor_status':data['status'],
                'vendor_ready_verified': False, 'permission': 'one_official_stand_request'}
    except (OSError, ValueError, TypeError, KeyError, AttributeError, IndexError):
        return {'matched': False, 'reason': 'profile_unavailable'}

class Stand:
    PHASE_TIMEOUT = 60
    STABLE_SECONDS = 1.0
    POLL_SECONDS = .1

    def __init__(self, reader, report):
        self.reader, self.report = reader, report
        self.state, self.result, self.task, self.pending = 'idle', None, None, None
        self.stages, self.events = [], []
        self.motion_sent = False
        reader.stand = self

    def info(self):
        return {'startup_match': startup_match(self.reader), 'state': self.state,
                'result': self.result, 'stages': list(self.stages), 'vendor_events':list(self.events),
                'robot_status': self.reader.sample('robot_status'),
                'hold_controller': 'vendor', 'automatic_retry': False,
                'stop_is_sitdown': False}

    def receive(self, body):
        p = self.pending
        if not p or body.get('accid') != self.reader.serial:
            return
        title = body.get('title')
        if title not in ('response_'+p['mode']+'_mode', 'notify_'+p['mode']+'_mode', 'notify_invalid_request'):
            return
        data = body.get('data')
        result = data.get('result') if isinstance(data,dict) else None
        correlated = body.get('guid') == p['guid']
        self.events.append({'phase':p['mode'],'title':title,'result':result,
                            'guid_matched':correlated})
        self.events = self.events[-16:]
        if not correlated:
            data_now=(self.reader.sample('robot_status').get('data') or {})
            # This firmware generates a new guid for completion notifications.
            # Require the owned request's correlated ACK plus the expected live mode.
            if (title=='notify_'+p['mode']+'_mode' and p.get('ack')
                    and data_now.get('sw_version')=='robot-tron2-r-2.4.6.20260901211405'
                    and data_now.get('status')==('STAND' if p['mode']=='stand' else 'WALK')):
                if result=='success':p['notified']=True;p['notify_guid_compatibility']=True
                else:p['error']='Vendor notification failure: '+str(result)
            return
        if result != 'success' or title == 'notify_invalid_request':
            p['error'] = 'Vendor '+str(title)+': '+str(result)
        elif title.startswith('response_'):
            p['ack'] = True
        else:
            p['notified'] = True

    def disconnect(self):
        if self.pending:
            self.pending['error'] = 'Connection lost; result unconfirmed; no retry'

    def _read_status(self):
        if getattr(self, "_transition_stop_requested", False) and asyncio.current_task() is self.task:
            raise MoveError("Lifecycle stop; physical result unconfirmed", self.motion_sent)
        r = self.reader
        sample = r.sample('robot_status')
        data = sample.get('data') or {}
        if not sample.get('available') or not r.connected or r.ws is None or r.ws.closed:
            raise MoveError('机器人连接或新鲜状态不可用，未继续发送动作',False)
        if data.get('working_mode') != 'developer_mode' or data.get('working_mode_valid') is not True:
            raise MoveError('需要有效上层developer_mode，未继续发送动作',False)
        if data.get('imu') != 'OK' or data.get('motor') != 'OK':
            raise MoveError('IMU或电机未报告OK，未继续发送动作',False)
        if data.get('collision_enable') is not True or data.get('collision_enable_valid') is not True or data.get('compliant_enable') is not False or data.get('compliant_enable_valid') is not True:
            raise MoveError('碰撞保护或拖动状态不满足起立条件',False)
        return sample

    def dispatch(self):
        r = self.reader
        walking=getattr(r,'walk',None)
        if walking is not None and walking.task is not None and not walking.task.done():
            raise MoveError('行走正在执行，请先停止行走再进行起立或坐下',False)
        peer=getattr(r,'sit_down',None)
        if peer is not None and peer.state not in ('idle','completed','rejected'):
            raise MoveError('坐下恢复任务占用或未确认，禁止起立重试',False)
        if self.state in ('unconfirmed','recovery_required'):
            # A confirmed WALK goal makes a repeat verification-only.
            # A failed request in the same starting mode is never replayed.
            current=self._read_status()['data'].get('status')
            if current=='WALK':
                self.state='completed'
        if self.state not in ('idle','completed','rejected'):
            raise MoveError('起立进行中或结果未确认；不会重复发送',False)
        if Path(os.environ.get('TRON2_MOTION_MAINTENANCE_FILE','/config/tron2/motion-maintenance.json')).exists():
            raise MoveError('驱动处于维护保护中，未发送起立指令',False)
        if r.configuration.get('mode') not in ('wheel_foot','sole_foot'):
            raise MoveError('起立仅支持已识别的轮足/足式构型',False)
        data = self._read_status()['data']
        mode = data.get('status')
        if mode not in ('DAMPING','SIT','STAND','WALK') and not startup_match(r)['matched']:
            raise MoveError('当前状态未匹配起立入口: '+str(mode),False)
        if r.move_pending or r.head_pending or r._pose_servo is not None:
            raise MoveError('其他运动请求正在进行，未发送起立指令',False)
        self._transition_stop_requested = False
        action_id = str(uuid.uuid4())
        self.state, self.result, self.stages, self.events, self.motion_sent = 'accepted',None,[],[],False
        self.task = asyncio.create_task(self._run(action_id))
        return {'status':'accepted','action_id':action_id,'motion_sent':False,
                'completion':'fresh_stable_WALK_after_prepare','hold_controller':'vendor'}

    async def _monitor(self, target, after, timeout, *, sent):
        # One completion parameter: the fresh vendor status, not wheel/joint stillness.
        deadline=time.monotonic()+timeout
        mode=None
        while time.monotonic()<deadline:
            if self.pending and self.pending.get('error'):
                raise MoveError(self.pending['error'],self.motion_sent)
            sample=self._read_status();mode=sample['data'].get('status')
            if mode==target and (sample.get('received_at') or 0)>after:
                return {'target':target,'command_sent':sent,'fresh_samples':1,
                        'ack_received':bool(self.pending and self.pending.get('ack')),
                        'notify_received':bool(self.pending and self.pending.get('notified')),
                        'completion_source':'notify_robot_info.data.status',
                        'notify_guid_compatibility':bool(self.pending and self.pending.get('notify_guid_compatibility')),
                        'firmware_status_compatibility':bool(sent and self.pending and not self.pending.get('notified'))}
            if mode not in ('UNKNOW','DAMPING','SIT','STAND','WALK'):
                raise MoveError('起立被异常模式中断: '+str(mode),self.motion_sent)
            if target=='WALK' and mode in ('DAMPING','SIT'):
                raise MoveError('机器人已退出准备/站立状态，未继续动作',self.motion_sent)
            await asyncio.sleep(self.POLL_SECONDS)
        raise MoveError('阶段超时: expected='+target+', actual='+str(mode)+'；已发送指令，物理结果未确认，禁止自动重试',self.motion_sent)

    async def _phase(self, command, target):
        self._read_status()
        p={'mode':command,'guid':str(uuid.uuid4()),'ack':False,'notified':False,'error':None}
        self.pending=p;after=time.time()
        self.state='preparing' if command=='stand' else 'standing_up'
        # Once attempted, a send failure is uncertain and must not be retried.
        self.motion_sent=None
        await asyncio.wait_for(self.reader.ws.send_json({'accid':self.reader.serial,
            'title':'request_'+command+'_mode','timestamp':int(after*1000),
            'guid':p['guid'],'data':{}}),5)
        self.motion_sent=True
        stage=await self._monitor(target,after,self.PHASE_TIMEOUT,sent=True)
        self.stages.append(stage);self.pending=None

    async def _run(self, action_id):
        try:
            mode=self._read_status()['data'].get('status')
            if mode=='WALK':
                self.state='verifying_standing'
                self.stages.append(await self._monitor('WALK',time.time(),5,sent=False))
            else:
                if mode!='STAND':
                    await self._phase('stand','STAND')
                else:
                    self.state='verifying_prepare'
                    self.stages.append(await self._monitor('STAND',time.time(),5,sent=False))
                # Recheck immediately before changing into vendor balance/walk mode.
                if self._read_status()['data'].get('status')!='STAND':
                    raise MoveError('准备状态已改变，未发送第二阶段',self.motion_sent)
                await self._phase('walk','WALK')
            self.result={'motion_sent':self.motion_sent,'vendor_status':'WALK',
                         'stages':list(self.stages),'hold_controller':'vendor',
                         'velocity_commands_sent':0,'automatic_retry':False}
            status='completed'
        except (Exception,asyncio.CancelledError) as exc:
            self.result={'motion_sent':self.motion_sent,'error':str(exc) or type(exc).__name__,
                         'phase':self.state,'stages':list(self.stages),'automatic_retry':False,
                         'retry_allowed':False,'recovery_required':True,'recovery_tool':'sit_down'}
            status='error'
        finally:
            self.pending=None
        self.state='reporting'
        delivered=await self.report('stand',action_id,status,self.result)
        self.state='completed' if delivered and status=='completed' else 'unconfirmed'
