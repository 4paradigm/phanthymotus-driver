"""Strict measured trajectories and cancellable, feedback-checked replay.

Without a vendor control guard, each execution requires operator confirmation.
This confirmation is not a hardware lease or detection of other processes.
"""
import asyncio
import hashlib
import json
import math
import time
from uuid import uuid4

from episode import TRAJECTORY_SCHEMA, STATE_NAMES, UNITS, MAX_TRAJECTORY, encode
from state import vector
from vendor import validate_movej, motion_readiness
from gripper import state_data, MAX_OPENING

MAX_SPEED = .8  # rad/s; reject faster recordings without changing their speed.
MAX_GAP_NS = 100_000_000
START_RAD = math.radians(2)
GRIP_TOLERANCE = 3


def pack_trajectory(binding, states):
    start = states[0]['received_ns']
    header = dict(type='header', schema=TRAJECTORY_SCHEMA, binding=binding, state_names=STATE_NAMES,
                  units=UNITS, source='measured_state', head_policy='hold_live_start')
    raw = bytearray(encode(header))
    for state in states:
        if state.get('gripper') is None:
            raise ValueError('complete_gripper_feedback_required')
        raw.extend(encode(dict(type='frame',t_ns=state['received_ns']-start,
            q=state['q'][:14],gripper=state['gripper'])))
    raw.extend(encode(dict(type='complete',frames=len(states),sha256=hashlib.sha256(raw).hexdigest())))
    return bytes(raw)


def read_trajectory(raw, binding=None):
    if not 1 <= len(raw) <= MAX_TRAJECTORY:
        raise ValueError('trajectory_size_limit')
    lines=raw.splitlines(keepends=True)
    if len(lines)<4 or any(len(line)>32768 for line in lines):
        raise ValueError('trajectory_layout')
    from common.teaching.package import unique
    values=[json.loads(line,object_pairs_hook=unique,parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite'))) for line in lines]
    if not all(isinstance(value,dict) for value in values):
        raise ValueError('trajectory_object_required')
    header,frames,footer=values[0],values[1:-1],values[-1]
    if (not isinstance(header.get('binding'),dict) or header.get('type')!='header' or header.get('schema')!=TRAJECTORY_SCHEMA or header.get('state_names')!=STATE_NAMES
            or header.get('units')!=UNITS or header.get('source')!='measured_state' or header.get('head_policy')!='hold_live_start'
            or footer.get('type')!='complete' or footer.get('frames')!=len(frames)
            or footer.get('sha256')!=hashlib.sha256(b''.join(lines[:-1])).hexdigest()):
        raise ValueError('trajectory_integrity')
    if binding is not None and header['binding']!=binding:
        raise ValueError('trajectory_binding_mismatch')
    previous=None
    for frame in frames:
        if frame.get('type')!='frame' or type(frame.get('t_ns')) is not int or not 0<=frame['t_ns']<=1800_000_000_000:
            raise ValueError('trajectory_frame')
        validate_movej(frame.get('q'), .1)
        vector(frame.get('gripper'),2,'gripper')
        if any(not 0<=v<=MAX_OPENING for v in frame['gripper']):
            raise ValueError('gripper_replay_limit')
        if previous:
            dt=frame['t_ns']-previous['t_ns']
            if not 0<dt<=MAX_GAP_NS:
                raise ValueError('trajectory_time_gap')
            if any(abs(a-b)*1e9/dt>MAX_SPEED for a,b in zip(frame['q'],previous['q'])):
                raise ValueError('trajectory_speed_limit')
        elif frame['t_ns']!=0:
            raise ValueError('trajectory_start_time')
        previous=frame
    return header,frames


class Replay:
    closed = False
    def __init__(self, reader, exclusive_guard, peers_idle):
        self.reader,self.exclusive_guard,self.peers_idle=reader,exclusive_guard,peers_idle
        self.task=None
        self.outcome=None
        self.preview=None
        self.epoch=0
        self.motion_sent=False

    @property
    def busy(self):
        return self.task is not None and not self.task.done()

    @property
    def active(self):
        if self.busy:
            return {'state':'running'}
        if self.outcome and self.outcome['result'].get('motion_may_continue'):
            return {'state':'blocked_requires_operator_check'}
        return None

    async def guard(self):
        if self.exclusive_guard is not None and await self.exclusive_guard() is not True:
            raise ValueError('exclusive_control_unverified')
        if not self.peers_idle():
            raise ValueError('other_motion_active')
        token=getattr(self.reader,'_pose_servo',None)
        if token and token.get('error'):
            raise ValueError('replay_vendor_fault')
        status=self.reader.sample('robot_status')
        if not status['available'] or not motion_readiness(status['data'])['ready'] or status['data'].get('motor')!='OK':
            raise ValueError('robot_mode_not_ready')
        sample=self.reader.sample('joint_state')
        if not sample['available'] or sample['reception_age_seconds']>.1 or len(sample['data']['q'])!=16:
            raise ValueError('replay_feedback_stale')
        from skeleton import JOINT_NAMES
        data=sample['data']
        names=data.get('names') or JOINT_NAMES
        if len(names)!=16 or set(names)!=set(JOINT_NAMES):
            raise ValueError('unsupported_joint_layout')
        order=[names.index(name) for name in JOINT_NAMES]
        return {**data,'q':[data['q'][i] for i in order],
                'dq':[data['dq'][i] for i in order] if len(data.get('dq',[]))==16 else []}

    async def preflight(self, raw, binding):
        if self.active:
            raise ValueError('replay_busy')
        _,frames=read_trajectory(raw,binding)
        feedback=await self.guard()
        grip=state_data(await self.reader.read_gripper())
        delta=[a-b for a,b in zip(feedback['q'][:14],frames[0]['q'])]
        if max(map(abs,delta))>START_RAD or any(abs(grip[s]['opening']-v)>GRIP_TOLERANCE for s,v in zip(('left','right'),frames[0]['gripper'])):
            raise ValueError('replay_start_mismatch_no_automatic_return')
        return frames,feedback['q'][14:]

    async def play(self, raw, binding, args):
        frames,head=await self.preflight(raw,binding)
        sha=hashlib.sha256(raw).hexdigest()
        if args.get('execute') is not True:
            self.preview={'id':uuid4().hex,'sha':sha,'expires':time.monotonic()+60,'epoch':self.epoch}
            return dict(executed=False,preview_id=self.preview['id'],duration_seconds=frames[-1]['t_ns']/1e9,
                        binding=binding,description='按原速度回放实测双臂和双夹爪轨迹；头部保持当前角度。')
        preview,self.preview=self.preview,None
        if not preview or args.get('preview_id')!=preview['id'] or preview['sha']!=sha or preview['epoch']!=self.epoch or time.monotonic()>preview['expires']:
            raise ValueError('paired_preview_required')
        if args.get('workspace_clear') is not True or args.get('take_control') is not True:
            raise ValueError('user_confirmation_required')
        if self.exclusive_guard is None and args.get('exclusive_control_confirmed') is not True:
            raise ValueError('operator_exclusive_confirmation_required')
        # Measured feedback is not a command speed/force. Require explicit settings.
        from gripper import percentage
        speed=percentage(args.get('gripper_speed'),'gripper_speed')
        force=percentage(args.get('gripper_force'),'gripper_force')
        if speed<=0:
            raise ValueError('gripper_speed_must_be_positive')
        ident='tron2_teach_'+uuid4().hex
        self.outcome={'action_id':ident,'status':'running','result':{}}
        self.motion_sent=False
        self.task=asyncio.create_task(self._run(ident,frames,head,speed,force))
        return {'state':'running','action_id':ident}

    async def _run(self, ident, frames, head, speed, force):
        result={'physical_stop_verified':False,'source':'measured_state','head_policy':'hold_live_start'}
        status='error'
        sent=0
        try:
            with self.reader.teaching_feedback(), self.reader.pose_servo(ident) as token:
                token['head_q']=head
                start=time.monotonic()
                last_grip=frames[0]['gripper']
                previous=frames[0]
                for frame in frames:
                    target=start+frame['t_ns']/1e9
                    await asyncio.sleep(max(0,target-time.monotonic()))
                    if time.monotonic()-target>.05:
                        raise ValueError('replay_deadline_missed_no_catchup')
                    feedback=await self.guard()
                    grip=state_data(await self.reader.read_gripper())
                    if time.monotonic()-target>.05:
                        raise ValueError('replay_deadline_missed_no_catchup')
                    if max(abs(a-b) for a,b in zip(feedback['q'][14:],head))>START_RAD:
                        raise ValueError('replay_head_hold_error')
                    if any(abs(grip[side]['opening']-v)>15 for side,v in zip(('left','right'),last_grip)):
                        raise ValueError('replay_gripper_tracking_error')
                    if max(abs(a-b) for a,b in zip(feedback['q'][:14],previous['q']))>math.radians(8):
                        raise ValueError('replay_tracking_error')
                    self.motion_sent=True
                    await self.reader.send_pose_servoj(frame['q']+head,token)
                    sent+=1
                    for i,side in enumerate(('left','right')):
                        if frame['gripper'][i]!=last_grip[i]:
                            await self.reader.send_gripper({side+'_opening':frame['gripper'][i],side+'_speed':speed,side+'_force':force})
                    last_grip=frame['gripper'];previous=frame
                # Sending the final frame is not completion. Require advancing feedback.
                deadline=time.monotonic()+5
                stable=0;last_stamp=None
                while time.monotonic()<deadline:
                    feedback=await self.guard()
                    grip=state_data(await self.reader.read_gripper())
                    at_target=max(abs(a-b) for a,b in zip(feedback['q'][:14],frames[-1]['q']))<=START_RAD
                    stationary=len(feedback.get('dq',[]))==16 and max(map(abs,feedback['dq']))<math.radians(1)
                    closed=all(abs(grip[s]['opening']-v)<=GRIP_TOLERANCE and grip[s]['speed']==0 for s,v in zip(('left','right'),last_grip))
                    stamp=feedback.get('robot_timestamp')
                    if stamp is not None and stamp!=last_stamp:
                        stable=stable+1 if at_target and stationary and closed else 0
                        last_stamp=stamp
                    if stable>=3:
                        status='completed';result['physical_stop_verified']=True
                        break
                    await asyncio.sleep(.02)
                if status!='completed':
                    raise ValueError('terminal_feedback_not_verified')
        except asyncio.CancelledError:
            status='cancelled';result['reason']='operator_stop'
        except (ValueError,OSError,asyncio.TimeoutError) as exc:
            result['reason']=str(exc)
        finally:
            self.preview=None
            self.outcome={'action_id':ident,'status':status,'result':dict(result,motion_sent=self.motion_sent,
                samples_sent=sent,output_stopped=True,motion_may_continue=not result['physical_stop_verified'])}

    async def stop(self):
        self.epoch+=1;self.preview=None
        if self.busy:
            self.task.cancel()
            await self.task
        return {'output_stopped':True,'physical_stop_verified':False,'motion_may_continue':self.motion_sent,
                'busy':bool(self.active),'authority_released':not bool(self.active),
                'message':'已停止发送；未确认机器人已物理停止','last_action':self.outcome}

    async def wait(self, ident):
        if self.task:
            await asyncio.shield(self.task)
        if not self.outcome or self.outcome['action_id']!=ident:
            raise ValueError('replay_action_changed')
        return self.outcome
