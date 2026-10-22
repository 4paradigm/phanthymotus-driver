"""Measured trajectory validation and SDK replay lifecycle."""
import asyncio
import hashlib
import json
import math
import time
from uuid import uuid4

from episode import TRAJECTORY_SCHEMA, STATE_NAMES, UNITS, MAX_TRAJECTORY, encode
from state import vector
from vendor import ARM_LOWER, ARM_UPPER, motion_readiness
from gripper import MAX_OPENING, percentage, state_data
from teaching_sdk import SDKReplay, StableWindow

MAX_GAP_NS = 100_000_000
START_RAD = math.radians(2)
RECORDING_LIMIT_TOLERANCE = math.radians(5)


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
    recovering = False

    def __init__(self, reader, exclusive_guard, peers_idle):
        self.reader, self.exclusive_guard, self.peers_idle = reader, exclusive_guard, peers_idle
        self.task = self.worker = self.outcome = self.preview = None
        self.epoch = 0

    @property
    def busy(self):
        return self.task is not None and not self.task.done()

    @property
    def phase(self):
        return self.worker.phase if self.busy and self.worker else None

    @property
    def active(self):
        if self.busy or self.recovering:
            return {'state': 'running'}
        if self.outcome and self.outcome['result'].get('motion_may_continue'):
            return {'state': 'blocked_requires_operator_check'}
        return None

    async def preflight(self, raw, binding):
        if self.active or not self.peers_idle():
            raise ValueError('other_motion_active')
        if self.exclusive_guard and await self.exclusive_guard() is not True:
            raise ValueError('exclusive_control_unverified')
        _, frames = read_trajectory(raw, binding)
        for frame in frames:
            frame['q'] = [min(hi, max(lo, q)) for q, lo, hi in zip(frame['q'], ARM_LOWER, ARM_UPPER)]
        movej_start_target(frames[0]['q'])
        status = self.reader.sample('robot_status')
        if not status['available'] or not motion_readiness(status['data'])['ready'] or status['data'].get('motor') != 'OK':
            raise ValueError('robot_mode_not_ready')
        with self.reader.pose_feedback():
            deadline = time.monotonic()+.5
            while True:
                sample = self.reader.sample('joint_state')
                age = sample.get('reception_age_seconds')
                if sample['available'] and type(age) in (int, float) and math.isfinite(age) and 0 <= age <= .1:
                    break
                if time.monotonic() >= deadline:
                    raise ValueError('replay_feedback_stale')
                await asyncio.sleep(.01)
        from skeleton import JOINT_NAMES
        names = sample['data'].get('names') or JOINT_NAMES
        if list(names) != list(JOINT_NAMES):
            raise ValueError('unsupported_joint_layout')
        q = vector(sample['data']['q'], 16, 'q')
        return frames, q[14:]

    async def play(self, raw, binding, args):
        frames, head = await self.preflight(raw, binding)
        sha = hashlib.sha256(raw).hexdigest()
        if args.get('execute') is not True:
            self.preview = dict(id=uuid4().hex, sha=sha, expires=time.monotonic()+60, epoch=self.epoch)
            return dict(executed=False, preview_id=self.preview['id'], binding=binding,
                duration_seconds=frames[-1]['t_ns']/1e9,
                description='原厂 SDK 初始化到录制起点（可能经过中间姿态并张开夹爪），再按原时间回放。请确认整个活动空间畅通。')
        preview, self.preview = self.preview, None
        if not preview or args.get('preview_id') != preview['id'] or preview['sha'] != sha or preview['epoch'] != self.epoch or time.monotonic() > preview['expires']:
            raise ValueError('paired_preview_required')
        if args.get('workspace_clear') is not True or args.get('take_control') is not True:
            raise ValueError('user_confirmation_required')
        if self.exclusive_guard is None and args.get('exclusive_control_confirmed') is not True:
            raise ValueError('operator_exclusive_confirmation_required')
        speed = percentage(args.get('gripper_speed'), 'gripper_speed')
        force = percentage(args.get('gripper_force'), 'gripper_force')
        if speed <= 0:
            raise ValueError('gripper_speed_must_be_positive')
        # Recheck peers after the only await; reserve this task before yielding again.
        if self.active or not self.peers_idle():
            raise ValueError('other_motion_active')
        ident = 'tron2_teach_'+uuid4().hex
        self.worker = SDKReplay(self.reader.settings['url'], frames, head,
                                movej_start_target(frames[0]['q']), speed, force)
        self.outcome = dict(action_id=ident, status='running', result={})
        self.task = asyncio.create_task(self._run(ident))
        return dict(state='running', action_id=ident)

    async def _run(self, ident):
        try:
            result = await asyncio.to_thread(self.worker.run)
        except Exception as exc:
            result = dict(status='error', reason=str(exc), output_stopped=False, motion_may_continue=True)
        status = result.pop('status')
        self.outcome = dict(action_id=ident, status=status, result=result)
        self.preview = None

    async def stop(self):
        self.epoch += 1
        self.preview = None
        if self.busy:
            self.worker.stop.set()
            # Do not cancel to_thread: cancellation does not stop the SDK thread.
            try:
                await asyncio.wait_for(asyncio.shield(self.task), 10)
            except asyncio.TimeoutError:
                return dict(output_stopped=False, physical_stop_verified=False,
                            motion_may_continue=True, busy=True, authority_released=False,
                            message='SDK 尚未结束清理，保持占用')
        result = (self.outcome or {}).get('result', {})
        return dict(output_stopped=result.get('output_stopped', True),
            physical_stop_verified=result.get('physical_stop_verified', False),
            motion_may_continue=result.get('motion_may_continue', False),
            busy=bool(self.active), authority_released=not bool(self.active), last_action=self.outcome)

    async def wait(self, ident):
        if self.task:
            await asyncio.shield(self.task)
        if not self.outcome or self.outcome['action_id'] != ident:
            raise ValueError('replay_action_changed')
        return self.outcome

    async def unlock(self, args):
        if self.busy or self.recovering:
            raise ValueError('replay_running_cannot_unlock')
        if args.get('physical_stop_confirmed') is not True or args.get('exclusive_control_confirmed') is not True:
            raise ValueError('unlock_operator_confirmation_required')
        if self.outcome and not self.outcome['result'].get('output_stopped'):
            raise ValueError('sdk_cleanup_not_verified')
        self.recovering = True
        try:
            window = StableWindow()
            deadline = time.monotonic()+3
            with self.reader.teaching_feedback():
                while time.monotonic() < deadline:
                    if not self.peers_idle():
                        raise ValueError('other_motion_active')
                    grip = state_data(await self.reader.read_gripper())
                    sample = self.reader.sample('joint_state')
                    now = time.monotonic()
                    if sample['available'] and sample.get('reception_age_seconds', 1) <= .1:
                        q = vector(sample['data']['q'], 16, 'q')
                        g = [grip[s]['opening'] for s in ('left', 'right')]
                        if window.add(now, (sample['data']['robot_timestamp'], grip['vendor_timestamp_ms']), q, g):
                            if self.outcome:
                                self.outcome['result'].update(physical_stop_verified=True, motion_may_continue=False)
                            self.epoch += 1
                            self.preview = None
                            return dict(unlocked=True, motion_sent=False, authority_released=True)
                    else:
                        window.reset()
                    await asyncio.sleep(.02)
            raise ValueError('replay_release_stability_not_verified')
        finally:
            self.recovering = False
