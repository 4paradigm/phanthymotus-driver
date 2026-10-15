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
    def __init__(self, reader, report):
        self.reader, self.report = reader, report
        self.active = self.last = self.task = None
        self.closed = False
        self.peer = None
        self.peers = []

    def summary(self):
        return ({key: self.active[key] for key in ('action_id', 'action', 'state')}
                if self.active else self.last)

    async def dispatch(self, action, target, duration, details):
        # Core can wake its next action before the callback HTTP response has
        # reached us. Wait for that bounded delivery only after verified arrival;
        # reject immediately while moving or when completion is uncertain.
        if self.active and self.active['state'] == 'reporting' \
                and self.active.get('arrival_verified'):
            await asyncio.shield(self.task)
        peers = self.peers + ([self.peer] if self.peer else [])
        for peer in peers:
            if peer.active and peer.active['state'] == 'reporting' and peer.active.get('target_opening_verified'):
                await asyncio.shield(peer.task)
        if any(peer.closed or peer.active or peer.recovering or peer.waiting for peer in peers):
            raise MoveError('Gripper action is active or uncertain; no arm motion sent', False)
        if self.closed or self.active:
            raise MoveError('Arms are busy or require operator verification; no additional motion sent', False)
        target, duration = validate_movej(target, duration)
        arm_feedback(self.reader)
        future = asyncio.get_running_loop().create_future()
        # A disconnected RPC caller must not cancel the one physical send or
        # abandon its owner. Retrieve a late admission error without hiding it.
        future.add_done_callback(lambda f: f.exception() if not f.cancelled() else None)
        job = {'action_id': 'tron2_arm_' + uuid4().hex, 'action': action,
               'state': 'awaiting_ack', 'target': list(target), 'duration': duration,
               'details': details, 'admission': future, 'ws': self.reader.ws,
               'serial': self.reader.serial}
        self.active = job
        self.task = asyncio.create_task(self._run(job))
        return await asyncio.shield(future)

    async def wait_for_arrival(self, job):
        # Wait the nominal duration AFTER ACK, then require distinct new frames
        # over a stationary window. Cached pre-command values cannot complete it.
        started = time.monotonic()
        not_before = started + job['duration']
        deadline = not_before + ARRIVAL_GRACE
        last_at = self.reader.joints_at
        window = []
        while time.monotonic() < deadline:
            if not self.reader.connected or self.reader.ws is not job['ws'] \
                    or self.reader.serial != job['serial']:
                raise ValueError('Robot disconnected or connection changed during motion')
            sample, q, dq = arm_feedback(self.reader)
            at = self.reader.joints_at
            if at != last_at:
                last_at = at
                now = time.monotonic()
                error = max(abs(math.degrees(a-b)) for a, b in zip(q, job['target']))
                slow = all(v is None or abs(v) <= VELOCITY_RAW_TOLERANCE for v in dq)
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
                reply = await self.reader.send_movej(job['target'], job['duration'])
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
            admitted = True
            job['admission'].set_result({**reply, **job['details'], 'action': job['action'],
                'action_id': job['action_id'], 'state': 'moving', 'joint_count': 14,
                'time': job['duration'], 'arrival_verified': False,
                'completion_protocol': 'acp',
                **({'reset_verified': False, 'at_home': None} if job['action'] == 'reset' else {})})
            result = await self.wait_for_arrival(job)
            status = 'completed'
        except asyncio.CancelledError:
            result = {'arrival_verified': False, 'error': 'Driver stopping; motion may continue'}
        except (ValueError, OSError) as exc:
            result = {'arrival_verified': False, 'error': str(exc)}
        finally:
            if self.active is not job:
                return
            if not admitted and not job['admission'].done():
                job['admission'].set_exception(MoveError('Motion result unknown; driver stopping', None))
            result.update(job['details'])
            result.update(action=job['action'], joint_count=14, completion_protocol='acp')
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
