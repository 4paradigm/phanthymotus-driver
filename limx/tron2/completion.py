"""Single arm motion owner, with feedback verification and bounded ACP delivery."""
import asyncio
import math
import time
from uuid import uuid4

from skeleton import skeleton_stream
from vendor import MoveError, validate_movej

ACP_TIMEOUT = 120
ARRIVAL_GRACE = 10
TOLERANCE_DEG = 1
STABILITY_DEG = .2
STABILITY_SECONDS = .5
# Vendor dq is preserved without a verified unit definition. This is an
# additional raw near-zero gate; stationary position is checked from q itself.
VELOCITY_RAW_TOLERANCE = .05


def arm_feedback(reader):
    sample = reader.sample('joint_state')
    packet = skeleton_stream(sample)
    if not packet['status']['available']:
        raise ValueError('Fresh, supported joint feedback is required')
    joints = {j['idx']: j for j in packet['joints']}
    return sample, [joints[i]['q'] for i in range(14)], [joints[i].get('dq') for i in range(14)]


class ArmCompletion:
    joint_count = 14
    action_prefix = 'tron2_arm_'
    require_ack = False  # Preserve the existing MoveJ completion behavior.

    def __init__(self, reader, report):
        self.reader, self.report = reader, report
        self.active = self.last = self.task = None
        self.closed = False
        self.peer = None
        self.peers = []

    def validate(self, target, duration):
        return validate_movej(target, duration)

    def feedback(self):
        return arm_feedback(self.reader)

    async def send(self, target, duration):
        return await self.reader.send_movej(target, duration)

    def blocked_error(self):
        return MoveError('Arms are busy or require operator verification; no additional motion sent', False)

    def summary(self):
        return ({**{key:self.active[key] for key in ('action_id','action','state')},
                 **({k:self.active[k] for k in ('stage','completed_stages')} if 'sequence' in self.active else {}),
                 **({k:self.active['details'][k] for k in ('motion_tool','preset','stages_total')} if 'sequence' in self.active else {})}
                if self.active else self.last)

    async def dispatch(self, action, target, duration, details, *, sequence=None, guard=None):
        # Core can wake its next action before the callback HTTP response has
        # reached us. Wait for that bounded delivery only after verified arrival;
        # reject immediately while moving or when completion is uncertain.
        if self.active and self.active['state'] == 'reporting' \
                and self.active.get('arrival_verified'):
            await asyncio.shield(self.task)
        peers = self.peers + ([self.peer] if self.peer else [])
        for peer in peers:
            if peer.active and peer.active['state'] == 'reporting' and (peer.active.get('target_opening_verified') or peer.active.get('arrival_verified')):
                await asyncio.shield(peer.task)
        if any(peer.closed or peer.active or getattr(peer, 'recovering', False) or getattr(peer, 'waiting', None) for peer in peers):
            raise MoveError('Gripper or other motion is active or uncertain; no additional motion sent', False)
        if self.closed or self.active:
            raise self.blocked_error()
        target, duration = self.validate(target, duration)
        self.feedback()
        future = asyncio.get_running_loop().create_future()
        # A disconnected RPC caller must not cancel the one physical send or
        # abandon its owner. Retrieve a late admission error without hiding it.
        future.add_done_callback(lambda f: f.exception() if not f.cancelled() else None)
        job = {'action_id': ('tron2_pose_' if sequence is not None else self.action_prefix) + uuid4().hex, 'action': action,
               'state': 'awaiting_ack', 'target': list(target), 'duration': duration,
               'details': details, 'admission': future, 'ws': self.reader.ws,
                'serial': self.reader.serial, 'motion_sent': None, 'vendor_acknowledged': False}
        self.active = job
        if sequence is not None:
            job.update(sequence=sequence,guard=guard,cancel_sequence=False,stage=0,completed_stages=0)
        self.task = asyncio.create_task(self._run_sequence(job) if sequence is not None else self._run(job))
        return await asyncio.shield(future)

    async def dispatch_sequence(self, plan, guard):
        import copy
        frames=copy.deepcopy(plan['frames'])
        if not frames or len(frames)>10:
            raise ValueError('Invalid pose frame count')
        for frame in frames:
            self.validate(frame['joint'],frame['time'])
            if type(frame['hold']) not in (int,float) or not 0<=frame['hold']<=2:
                raise ValueError('Invalid pose dwell')
        # Include every arrival grace window and bounded ACP delivery budget.
        if sum(f['time']+f['hold']+ARRIVAL_GRACE for f in frames)+10>=300:
            raise ValueError('Pose exceeds ACP time budget; no motion sent')
        details={k:copy.deepcopy(v) for k,v in plan.items() if k!='frames'}
        details['stages_total']=len(frames)
        return await self.dispatch('pose',frames[0]['joint'],frames[0]['time'],details,
                                   sequence=frames,guard=guard)

    def cancel_sequence(self):
        if self.active and self.active.get('sequence') is not None and self.task and not self.task.done() and self.active['state'] not in ('reporting','blocked_requires_operator_check'):
            self.active['cancel_sequence']=True
            return True
        return False

    def _sequence_feedback(self, job, expected):
        if self.closed or self.active is not job or not self.reader.connected or \
                self.reader.ws is not job['ws'] or self.reader.serial!=job['serial']:
            raise ValueError('Pose connection/owner changed; remaining frames cancelled')
        sample,q,dq=self.feedback()
        if job['details']['preset']!='reset':
            packet=skeleton_stream(sample)
            head=[j for j in packet['joints'] if j['idx']>=14]
            if len(head)!=2 or any(abs(math.degrees(j['q']))>10 or j.get('dq') is None or abs(j['dq'])>.05 for j in head):
                raise ValueError('Head moved outside preset admission range; remaining frames cancelled')
        if len(dq)!=14 or any(v is None or abs(v)>.05 for v in dq) or \
                max(abs(math.degrees(a-b)) for a,b in zip(q,expected))>1:
            raise ValueError('Pose start/waypoint feedback changed; remaining frames cancelled')

    async def _run_sequence(self, job):
        admitted=False;status='error';safe_end=False;any_sent=False
        result={'arrival_verified':False}
        expected=job['details']['start_q']
        try:
            for index,frame in enumerate(job['sequence']):
                # A lifecycle stop never creates another motor command.
                if job['cancel_sequence']:
                    self._sequence_feedback(job,expected)
                    status='cancelled';safe_end=True
                    break
                await job['guard']()
                self._sequence_feedback(job,expected)
                if job['cancel_sequence']:
                    status='cancelled';safe_end=True
                    break
                job.update(stage=index+1,target=list(frame['joint']),duration=frame['time'],
                           state='awaiting_ack',vendor_acknowledged=False)
                previous_sent=any_sent
                any_sent=True if previous_sent is True else None  # Outcome unknown until exact ACK.
                try:
                    reply=await self.send(job['target'],job['duration'])
                except (ValueError,OSError) as exc:
                    if not admitted and (getattr(exc,'motion_sent',None) is False or getattr(exc,'code',None)=='vendor_rejected'):
                        self.active=None
                        job['admission'].set_exception(exc)
                        return
                    if getattr(exc,'motion_sent',None) is False:any_sent=previous_sent
                    elif getattr(exc,'motion_sent',None) is True:any_sent=True
                    job['motion_sent']=any_sent
                    if not admitted:
                        admitted=True
                        job['admission'].set_result({'action_id':job['action_id'],'action':'pose',
                            'preset':job['details']['preset'],'state':'blocked_requires_operator_check',
                            'motion_sent':any_sent,'arrival_verified':False,'completion_protocol':'acp'})
                    raise ValueError('Pose stage acknowledgement failed; remaining frames cancelled: '+str(exc)) from exc
                any_sent=True if reply.get('motion_sent') is True or previous_sent is True else None
                job['motion_sent']=any_sent
                job['vendor_acknowledged']=reply.get('status')=='accepted'
                if not job['vendor_acknowledged']:
                    raise ValueError('Pose requires an exact positive acknowledgement for every stage')
                job['state']='moving'
                if not admitted:
                    admitted=True
                    job['admission'].set_result({**reply,'action_id':job['action_id'],'action':'pose',
                        'preset':job['details']['preset'],'side':job['details']['side'],
                        'state':'moving','motion_sent':any_sent,'arrival_verified':False,
                        'stages_total':len(job['sequence']),'completion_protocol':'acp'})
                await self.wait_for_arrival(job)
                job['completed_stages']=index+1
                expected=list(job['target'])
                until=time.monotonic()+frame['hold']
                while time.monotonic()<until and not job['cancel_sequence']:
                    self._sequence_feedback(job,expected)
                    await asyncio.sleep(min(.05,max(0,until-time.monotonic())))
            else:
                self._sequence_feedback(job,job['details']['home_q'])
                status='completed';safe_end=True
                result.update(arrival_verified=True,reset_verified=True,at_home=True,
                    verification='every_stage_post_duration_stable_feedback_then_home')
            if status=='cancelled':
                result.update(cancelled_remaining_stages=True,segment_arrival_verified=True,
                    message='后续动作段已取消；反馈确认停在最近已到位动作段，未自动回正')
        except asyncio.CancelledError:
            result['error']='Driver stopping; pose may continue; no remaining frames sent'
        except (ValueError,OSError) as exc:
            result.update(error=str(exc),error_code=getattr(exc,'code',None),
                          cancelled_remaining_stages=True)
        finally:
            if self.active is not job:return
            if not admitted:
                # Pure pre-send failure creates no ACP job or sticky owner.
                if any_sent is False:
                    self.active=None
                    if not job['admission'].done():job['admission'].set_exception(MoveError(result.get('error','Pose stopped before dispatch'),False))
                    return
                if not job['admission'].done():job['admission'].set_exception(MoveError('Pose result unknown',None))
            result.update({k:v for k,v in job['details'].items() if k not in ('start_q','home_q')})
            result.update(action='pose',joint_count=14,completion_protocol='acp',
                motion_sent=any_sent,vendor_acknowledged=job['vendor_acknowledged'],
                completed_stages=job['completed_stages'],motion_may_continue=not safe_end)
            job['arrival_verified']=safe_end
            job['state']='reporting'
            delivered=False
            try:delivered=await self.report(job['action_id'],status,result)
            except Exception:pass
            self.last={'action_id':job['action_id'],'action':'pose','status':status,
                       'callback_delivered':delivered,**result}
            if safe_end and delivered:self.active=None
            else:job['state']='blocked_requires_operator_check'

    async def wait_for_arrival(self, job):
        # Wait the nominal duration AFTER ACK, then require distinct new frames
        # over a stationary window. Cached pre-command values cannot complete it.
        started = time.monotonic()
        not_before = started + job['duration']
        deadline = not_before + ARRIVAL_GRACE
        last_at = self.reader.joints_at
        last_vendor = (self.reader.sample('joint_state').get('data') or {}).get('robot_timestamp')
        window = []
        while time.monotonic() < deadline:
            if not self.reader.connected or self.reader.ws is not job['ws'] \
                    or self.reader.serial != job['serial']:
                raise ValueError('Robot disconnected or connection changed during motion')
            sample, q, dq = self.feedback()
            at = self.reader.joints_at
            if at != last_at:
                last_at = at
                if job.get('sequence') is not None:
                    stamp=(sample.get('data') or {}).get('robot_timestamp')
                    if type(stamp) not in (int,float) or not math.isfinite(stamp) or last_vendor is None or stamp<=last_vendor:
                        window=[]
                        await asyncio.sleep(.05)
                        continue
                    last_vendor=stamp
                now = time.monotonic()
                error = max(abs(math.degrees(a-b)) for a, b in zip(q, job['target']))
                slow = (len(dq)==14 and all(v is not None and abs(v)<=VELOCITY_RAW_TOLERANCE for v in dq)) if job.get('sequence') is not None else all(v is None or abs(v) <= VELOCITY_RAW_TOLERANCE for v in dq)
                if now < not_before or error > TOLERANCE_DEG or not slow:
                    window = []
                else:
                    if window and max(abs(math.degrees(a-b)) for a,b in zip(q, window[0][1])) > STABILITY_DEG:
                        window = []
                    window.append((now, q))
                    if len(window) >= 3 and now-window[0][0] >= STABILITY_SECONDS:
                        return {'arrival_verified': True, 'max_error_deg': error,
                                'tolerance_deg': TOLERANCE_DEG, 'verified_samples': len(window),
                                'verification': 'post_duration_stable_feedback'}
            await asyncio.sleep(.05)
        raise ValueError('Arrival verification timed out; motion may still be active')

    async def _run(self, job):
        admitted = False
        result = {'arrival_verified': False}
        status = 'error'
        try:
            try:
                reply = await self.send(job['target'], job['duration'])
            except MoveError as exc:
                if exc.motion_sent is False or exc.code == 'vendor_rejected':
                    self.active = None
                    job['admission'].set_exception(exc)
                    return
                # The send/ACK result is unknown. Keep ownership, return a
                # tracking ID, verify feedback, and never resend the command.
                reply = {'status': 'ack_unverified', 'motion_sent': exc.motion_sent,
                         'arrival_verified': False, 'message': str(exc)}
            job['state'] = 'moving'
            job['motion_sent'] = reply.get('motion_sent')
            job['vendor_acknowledged'] = reply.get('status') != 'ack_unverified'
            admitted = True
            job['admission'].set_result({**reply, **job['details'], 'action': job['action'],
                'action_id': job['action_id'], 'state': 'moving', 'joint_count': self.joint_count,
                'time': job['duration'], 'arrival_verified': False,
                'completion_protocol': 'acp',
                **({'reset_verified': False, 'at_home': None} if job['action'] == 'reset' else {})})
            if self.require_ack and reply.get('status') == 'ack_unverified':
                raise ValueError('Head acknowledgement unknown; position alone cannot confirm this command')
            result = await self.wait_for_arrival(job)
            status = 'completed'
        except asyncio.CancelledError:
            result = {'arrival_verified': False, 'error': 'Driver stopping; motion may continue'}
        except (ValueError, OSError) as exc:
            result = {'arrival_verified': False, 'error': str(exc), 'error_code':getattr(exc,'code',None)}
        finally:
            if self.active is not job:
                return
            if not admitted and not job['admission'].done():
                job['admission'].set_exception(MoveError('Motion result unknown; driver stopping', None))
            result.update(job['details'])
            result.update(action=job['action'], joint_count=self.joint_count, completion_protocol='acp')
            result.update(motion_sent=job['motion_sent'],vendor_acknowledged=job['vendor_acknowledged'])
            if job['action'] == 'reset':
                # The dispatched reset target came from the validated home file.
                # Verify it using the same multi-frame, post-duration feedback;
                # an ACK or a cached pre-motion sample cannot mark it at home.
                result['reset_verified'] = status == 'completed' and result['arrival_verified'] is True
                result['at_home'] = True if result['reset_verified'] else None
            result['motion_may_continue'] = status != 'completed'
            job['arrival_verified'] = result['arrival_verified']
            job['state'] = 'reporting'
            delivered = False
            try:
                delivered = await self.report(job['action_id'], status, result)
            except Exception:
                # Delivery failure cannot crash the sensor/robot reader tasks.
                pass
            self.last = {'action_id': job['action_id'], 'action': job['action'],
                         'status': status, 'callback_delivered': delivered, **result}
            if status == 'completed' and delivered:
                self.active = None
            else:
                # Core may release its barrier after an error/timeout. Retain
                # the local owner because that does not physically stop arms.
                job['state'] = 'blocked_requires_operator_check'

    async def close(self):
        self.closed = True
        if self.task and not self.task.done():
            self.task.cancel()
            await asyncio.gather(self.task, return_exceptions=True)
        if self.active and not self.active['admission'].done():
            self.active['admission'].set_exception(MoveError('Driver stopped before dispatch', False))
            self.active = None


class MotionPeers:
    """Present the arm/head owners as one peer to existing gripper guards."""
    def __init__(self, *owners):
        self.owners = owners

    @property
    def active(self):
        return next((owner.active for owner in self.owners if owner.active), None)

    @property
    def task(self):
        return next((owner.task for owner in self.owners if owner.active), None)

    @property
    def closed(self):
        return any(owner.closed for owner in self.owners)
