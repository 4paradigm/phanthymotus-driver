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
from vendor import ARM_LOWER, ARM_UPPER, MoveError, validate_movej, motion_readiness
from gripper import state_data, MAX_OPENING
from reset import duration_for_change

MAX_GAP_NS = 100_000_000
START_RAD = math.radians(2)
GRIP_TOLERANCE = 3
RECORDING_LIMIT_TOLERANCE = math.radians(5)
START_STABILITY_SECONDS = .5
START_ARRIVAL_TIMEOUT = 10
START_GRIPPER_TIMEOUT = 30


# LimX tron2_manipulation_controller 482ebbc: robot_model.CONTROLLER_ARM_LIMITS.
# MoveJ uses narrower wrist limits than the SDK's recorded/ServoJ arm range.
MOVEJ_START_LOWER = (-3.14159,-.261799,-3.66519,-2.61799,-1.74533,-.7505,-.7505,
                     -3.14159,-2.96706,-1.48353,-2.61799,-1.39626,-.7505,-.7505)
MOVEJ_START_UPPER = (2.60054,2.96706,1.48353,.523599,1.39626,.7505,.7505,
                     2.60054,.261799,3.66519,.523599,1.74533,.7505,.7505)


def movej_start_target(q):
    target=[min(high,sdk_high,max(low,sdk_low,value)) for value,low,high,sdk_low,sdk_high
        in zip(q,MOVEJ_START_LOWER,MOVEJ_START_UPPER,ARM_LOWER,ARM_UPPER)]
    for index,(value,planned) in enumerate(zip(q,target)):
        if abs(value-planned)>START_RAD:
            raise ValueError('录制起点超出 MoveJ 可达范围超过 2°，关节 '+str(index))
    return target


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
        q=vector(frame.get('q'),14,'q')
        for index,(value,low,high) in enumerate(zip(q,ARM_LOWER,ARM_UPPER)):
            if not low-RECORDING_LIMIT_TOLERANCE<=value<=high+RECORDING_LIMIT_TOLERANCE:
                raise ValueError('Recorded joint angle exceeds 5 degree tolerance at index '+str(index))
        vector(frame.get('gripper'),2,'gripper')
        if any(not 0<=v<=MAX_OPENING for v in frame['gripper']):
            raise ValueError('gripper_replay_limit')
        if previous:
            dt=frame['t_ns']-previous['t_ns']
            if not 0<dt<=MAX_GAP_NS:
                raise ValueError('trajectory_time_gap')
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
        self.phase=None
        self.connection=None
        self.recovering=False
        self.movej_not_before=0
        self.movej_acknowledged=False
        self.command_pending=False
        self.release_task=None

    @property
    def busy(self):
        return self.task is not None and not self.task.done()

    @property
    def active(self):
        if self.recovering:
            return {'state':'checking_release'}
        if self.busy:
            return {'state':'running'}
        if self.outcome and self.outcome['result'].get('motion_may_continue'):
            return {'state':'blocked_requires_operator_check'}
        return None

    async def guard(self):
        if self.connection is not None and self.connection!=(getattr(self.reader,'ws',None),self.reader.serial):
            raise ValueError('replay_start_connection_changed')
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
        vector(data['q'],16,'q')
        vector(data.get('dq'),16,'dq')
        return {**data,'q':[data['q'][i] for i in order],
                'dq':[data['dq'][i] for i in order] if len(data.get('dq',[]))==16 else []}

    async def fresh_feedback(self):
        deadline=time.monotonic()+.5
        while True:
            sample=self.reader.sample('joint_state')
            if sample['available'] and sample['reception_age_seconds']<=.1:
                return await self.guard()
            if time.monotonic()>=deadline:
                raise ValueError('replay_feedback_stale')
            await asyncio.sleep(.01)

    async def preflight(self, raw, binding):
        if self.active:
            raise ValueError('replay_busy')
        _,frames=read_trajectory(raw,binding)
        # Clamp only execution targets; package import/export retains measured values.
        for frame in frames:
            frame['q']=[min(high,max(low,value)) for value,low,high in zip(frame['q'],ARM_LOWER,ARM_UPPER)]
            validate_movej(frame['q'],.1)
        # Idle polling is slower than the replay freshness limit; wake the sole reader.
        with self.reader.pose_feedback():
            grip=state_data(await self.reader.read_gripper())
            feedback=await self.fresh_feedback()
        if max(map(abs,feedback['dq']))>=math.radians(1):
            raise ValueError('replay_start_not_stationary')
        target=movej_start_target(frames[0]['q'])
        duration=duration_for_change(max(abs(math.degrees(a-b)) for a,b in zip(feedback['q'][:14],target)))
        validate_movej(target,duration)
        return frames,feedback['q'][14:]

    async def play(self, raw, binding, args):
        frames,head=await self.preflight(raw,binding)
        sha=hashlib.sha256(raw).hexdigest()
        if args.get('execute') is not True:
            self.preview={'id':uuid4().hex,'sha':sha,'expires':time.monotonic()+60,'epoch':self.epoch}
            return dict(executed=False,preview_id=self.preview['id'],duration_seconds=frames[-1]['t_ns']/1e9,
                        binding=binding,description='先低速移动双臂并调整夹爪到录制起点，确认到位后按原时间回放；头部保持当前角度。请确认移向起点的路径及回放活动空间畅通。')
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
        self.movej_not_before=0
        self.movej_acknowledged=False
        self.command_pending=False
        self.phase='checking_start'
        self.task=asyncio.create_task(self._run(ident,frames,head,speed,force))
        return {'state':'running','action_id':ident}

    async def _wait_at_start(self, target, head, *, minimum=0, timeout=None, grippers=None):
        timeout=START_ARRIVAL_TIMEOUT if timeout is None else timeout
        not_before=time.monotonic()+minimum
        deadline=not_before+timeout
        stamp=None;window=[]
        while time.monotonic()<deadline:
            feedback=await self.guard()
            if max(abs(a-b) for a,b in zip(feedback['q'][14:],head))>START_RAD:
                raise ValueError('replay_head_hold_error')
            current=feedback.get('robot_timestamp')
            if type(current) not in (int,float) or not math.isfinite(current) or stamp is not None and current<stamp:
                raise ValueError('replay_start_timestamp_invalid')
            at_target=max(abs(a-b) for a,b in zip(feedback['q'][:14],target))<=START_RAD
            stationary=max(map(abs,feedback['dq']))<math.radians(1)
            if grippers is not None:
                grip=state_data(await self.reader.read_gripper())
                at_target=at_target and all(abs(grip[s]['opening']-v)<=GRIP_TOLERANCE and grip[s]['speed']==0
                    for s,v in zip(('left','right'),grippers))
            now=time.monotonic()
            if current!=stamp:
                stamp=current
                if now<not_before or not at_target or not stationary:
                    window=[]
                else:
                    if window and max(abs(a-b) for a,b in zip(feedback['q'],window[0][1]))>math.radians(.2):
                        window=[]
                    window.append((now,feedback['q']))
                    if len(window)>=3 and now-window[0][0]>=START_STABILITY_SECONDS:
                        return feedback
            await asyncio.sleep(.02)
        raise ValueError('replay_start_arrival_not_verified')

    async def _prepare_start(self, first, head, speed, force):
        feedback=await self.fresh_feedback()
        feedback=await self._wait_at_start(feedback['q'][:14],head,timeout=3)
        target=movej_start_target(first['q'])
        duration=duration_for_change(max(abs(math.degrees(a-b)) for a,b in zip(feedback['q'][:14],target)))
        validate_movej(target,duration)
        self.phase='moving_to_start'
        # Establish the vendor position target with MoveJ before entering ServoJ.
        self.motion_sent=True
        self.movej_not_before=time.monotonic()+duration+5+getattr(self.reader,'settings',{}).get('response_timeout_seconds',5)
        try:
            await self.reader.send_movej(target,duration)
            self.movej_acknowledged=True
            self.movej_not_before=time.monotonic()+duration
        except MoveError as exc:
            if exc.motion_sent is False:self.motion_sent=False
            raise
        await self._wait_at_start(first['q'],head,minimum=duration)
        self.phase='adjusting_grippers'
        grip=state_data(await self.reader.read_gripper())
        for side,value in zip(('left','right'),first['gripper']):
            if abs(grip[side]['opening']-value)>GRIP_TOLERANCE:
                await self.guard()
                self.command_pending=True
                await self.reader.send_gripper({side+'_opening':value,side+'_speed':speed,side+'_force':force})
                self.command_pending=False
        self.phase='verifying_start'
        await self._wait_at_start(first['q'],head,timeout=START_GRIPPER_TIMEOUT,grippers=first['gripper'])

    async def _run(self, ident, frames, head, speed, force):
        result={'physical_stop_verified':False,'source':'measured_state','head_policy':'hold_live_start'}
        status='error'
        sent=0
        self.connection=(getattr(self.reader,'ws',None),self.reader.serial)
        try:
            with self.reader.teaching_feedback():
                await self._prepare_start(frames[0],head,speed,force)
                result['start_verified']=True
                self.phase='replaying'
                with self.reader.pose_servo(ident) as token:
                    token['head_q']=head
                    await self.fresh_feedback()
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
                                self.command_pending=True
                                await self.reader.send_gripper({side+'_opening':frame['gripper'][i],side+'_speed':speed,side+'_force':force})
                                self.command_pending=False
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
            result['phase']=self.phase
            self.phase=None
            self.preview=None
            self.outcome={'action_id':ident,'status':status,'result':dict(result,motion_sent=self.motion_sent,
                samples_sent=sent,output_stopped=True,motion_may_continue=self.motion_sent and not result['physical_stop_verified'])}
            if self.motion_sent and not result['physical_stop_verified'] and self.movej_acknowledged and not self.command_pending:
                self.recovering=True
                self.release_task=asyncio.create_task(self._release_after_motion())
            else:
                self.connection=None

    async def _release_after_motion(self):
        try:
            await asyncio.sleep(max(0,self.movej_not_before-time.monotonic()))
            await self._verify_stopped()
            self.outcome['result'].update(physical_stop_verified=True,motion_may_continue=False,
                release_method='automatic_stable_feedback')
        except (ValueError,OSError,asyncio.TimeoutError) as exc:
            self.outcome['result']['release_error']=str(exc)
        finally:
            self.connection=None
            self.recovering=False

    async def _verify_stopped(self):
        with self.reader.teaching_feedback():
            feedback=await self.fresh_feedback()
            grip=state_data(await self.reader.read_gripper())
            await self._wait_at_start(feedback['q'][:14],feedback['q'][14:],timeout=3,
                grippers=[grip[side]['opening'] for side in ('left','right')])

    async def unlock(self, args):
        if self.busy or self.recovering:
            raise ValueError('replay_running_cannot_unlock')
        if args.get('physical_stop_confirmed') is not True or args.get('exclusive_control_confirmed') is not True:
            raise ValueError('unlock_operator_confirmation_required')
        if self.motion_sent and time.monotonic()<self.movej_not_before:
            raise ValueError('movej_duration_not_elapsed')
        self.recovering=True
        self.connection=(getattr(self.reader,'ws',None),self.reader.serial)
        try:
            await self._verify_stopped()
            if self.outcome:
                self.outcome['result'].update(physical_stop_verified=True,motion_may_continue=False,
                    release_method='operator_confirmed_stable_feedback')
            self.epoch+=1;self.preview=None
            return dict(unlocked=True,motion_sent=False,authority_released=True)
        finally:
            self.connection=None
            self.recovering=False

    async def stop(self):
        self.epoch+=1;self.preview=None
        if self.busy:
            self.task.cancel()
            await self.task
        result=(self.outcome or {}).get('result',{})
        return {'output_stopped':True,'physical_stop_verified':result.get('physical_stop_verified',False),
                'motion_may_continue':result.get('motion_may_continue',self.motion_sent),
                'busy':bool(self.active),'authority_released':not bool(self.active),
                'message':'已停止发送；已核验反馈静止' if result.get('physical_stop_verified') else '已停止发送；未确认机器人已物理停止',
                'last_action':self.outcome}

    async def wait(self, ident):
        if self.task:
            await asyncio.shield(self.task)
        if not self.outcome or self.outcome['action_id']!=ident:
            raise ValueError('replay_action_changed')
        return self.outcome
