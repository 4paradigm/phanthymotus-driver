"""Bounded, time-indexed TRON2 pose paths and feedback-supervised ServoJ output.

ServoJ has no positive ACK. Socket submission is never arrival or acceptance
proof. Only final advancing, stationary feedback can complete a trajectory.
"""
import asyncio
import math
import time

from completion import arm_feedback, STABILITY_SECONDS, STABILITY_DEG, TOLERANCE_DEG
from skeleton import skeleton_stream
from vendor import MoveError, validate_movej, motion_readiness

OUTPUT_HZ = 300
MAX_OUTPUT_GAP = .05
MAX_FEEDBACK_AGE = .35
TRACKING_DEG = 8.
MAX_ACCEL_DEG = 60.
MAX_MOTION_SCALE = 1.44
FINAL_TIMEOUT = 10.
CONTROL_GUARD_INTERVAL = 1.


def smooth(t):
    t = max(0., min(1., t))
    return t*t*t*(10+t*(-15+6*t))


class Path:
    """C2 quintic ramps; only disjoint joint ramps overlap (25 percent).

    A shoulder ramp leads an elbow ramp. Reversals have zero velocity without
    waiting for a stationary feedback window at every reversal. No overshoot:
    overlapping ramps affect disjoint joints and smooth() is monotonic.
    """
    def __init__(self, start, frames, motion_scale=1.):
        if type(motion_scale) not in (int,float) or not math.isfinite(motion_scale) or not 1.<=motion_scale<=MAX_MOTION_SCALE:
            raise ValueError('Preset motion scale must be between 1 and 1.44')
        self.motion_scale = motion_scale
        self.start = list(start)
        self.ramps = []
        self.holds = []
        previous = list(start)
        end = 0.
        previous_axes = set()
        previous_hold = 0.
        previous_duration = 0.
        for frame in frames:
            target, duration = validate_movej(frame['joint'], frame['time'])
            delta = [b-a for a,b in zip(previous,target)]
            axes = {i for i,v in enumerate(delta) if abs(v)>1e-10}
            travel = max(abs(math.degrees(v)) for v in delta)
            # Quintic peak acceleration is 10/sqrt(3) * travel / duration^2.
            duration = max(duration, math.sqrt((10/math.sqrt(3))*travel/MAX_ACCEL_DEG))
            # Scale the complete ramp, including acceleration-limited swings.
            # Presentation holds retain their requested duration. Peak speed
            # scales linearly; peak acceleration scales by motion_scale^2.
            duration /= motion_scale
            overlap = (.25*min(previous_duration,duration)
                       if previous_axes and axes and not axes & previous_axes and previous_hold==0 else 0.)
            begins = end-overlap
            self.ramps.append((begins,duration,delta,frame['label']))
            if frame['hold']:
                self.holds.append((begins+duration,frame['hold'],list(target)))
            previous = list(target)
            previous_hold = frame['hold']
            previous_duration = duration
            previous_axes = axes
            end = begins+duration+previous_hold
        if not self.ramps:
            raise ValueError('Empty preset trajectory')
        self.duration = end
        self.target = previous

    def at(self, elapsed):
        if elapsed>=self.duration:
            return list(self.target)
        q = list(self.start)
        for begins,duration,delta,_ in self.ramps:
            factor = smooth((elapsed-begins)/duration)
            q = [a+factor*b for a,b in zip(q,delta)]
        return q

    def screening_frames(self):
        # Bound both temporal spacing and nominal angular spacing. Geometry's
        # existing <=2deg linear screening refines these samples further.
        count = math.ceil(self.duration*20)
        return [{'joint':self.at(self.duration*i/count)} for i in range(count+1)]


def blocks(start, frames, motion_scale=1.):
    """Keep gripper barriers; continuous arm output between those barriers."""
    result = []
    pending = []
    current = list(start)
    def flush():
        nonlocal pending,current
        if pending:
            path = Path(current,pending,motion_scale)
            result.append({'kind':'trajectory','label':'trajectory_'+pending[-1]['label'],
                'joint':list(path.target),'time':path.duration,'hold':0,'path':path})
            current = list(path.target)
            pending = []
    for frame in frames:
        if frame.get('kind')=='gripper':
            flush()
            result.append(frame)
        else:
            pending.append(frame)
    flush()
    return result


async def run(owner, job, frame, admitted):
    """No retries, no queued burst after a scheduler stall, no second WS reader."""
    reader = owner.reader
    path = frame['path']
    started = last_output = time.monotonic()
    first = True
    stamp = None
    advanced_at = started
    window = []
    hold_windows = [[] for _ in path.holds]
    verified_holds = set()
    guard_task = None
    next_guard = started+CONTROL_GUARD_INTERVAL
    sent = 0
    diagnostics = {'stage_label':frame['label'],'target_hz':OUTPUT_HZ,
        'samples_sent':0,'max_output_gap_seconds':0.,'max_socket_send_seconds':0.,
        'max_feedback_age_seconds':0.,'max_tracking_error_deg':0.}
    job.setdefault('trajectory_diagnostics',[]).append(diagnostics)
    try:
        with reader.pose_servo(job) as token:
            while True:
                now = time.monotonic()
                if not first:
                    diagnostics['max_output_gap_seconds']=max(diagnostics['max_output_gap_seconds'],now-last_output)
                if owner.closed or owner.active is not job or job['cancel_sequence']:
                    raise MoveError('Trajectory output cancelled; stationary stop is not yet verified',bool(sent),'trajectory_cancelled')
                if reader.ws is not job['ws'] or reader.serial!=job['serial'] or not reader.connected:
                    raise MoveError('Trajectory connection changed',bool(sent),'trajectory_connection')
                if not first and now-last_output>MAX_OUTPUT_GAP:
                    raise MoveError('Trajectory scheduler stalled; no catch-up commands sent',True,'trajectory_output_gap')
                sample,q,dq = arm_feedback(reader)
                packet = skeleton_stream(sample)
                head = [j for j in packet['joints'] if j['idx']>=14]
                if len(head)!=2 or any(abs(math.degrees(j['q']))>TOLERANCE_DEG or j.get('dq') is None or abs(j['dq'])>.05 for j in head):
                    raise MoveError('Centered head feedback changed during trajectory',bool(sent),'trajectory_head_drift')
                current = sample['data'].get('robot_timestamp')
                if type(current) not in (int,float) or not math.isfinite(current) or (stamp is not None and current<stamp):
                    raise MoveError('Trajectory feedback timestamp invalid/regressed',bool(sent),'trajectory_feedback')
                fresh = stamp is None or current>stamp
                if fresh:
                    stamp = current
                    advanced_at = now
                age = sample.get('reception_age_seconds')
                if type(age) in (int,float) and math.isfinite(age):
                    diagnostics['max_feedback_age_seconds']=max(diagnostics['max_feedback_age_seconds'],age)
                if age is None or age>MAX_FEEDBACK_AGE or now-advanced_at>MAX_FEEDBACK_AGE:
                    raise MoveError('Trajectory feedback stale/frozen',bool(sent),'trajectory_feedback')
                status = reader.sample('robot_status')
                if not status['available'] or not motion_readiness(status['data'])['ready'] or status['data'].get('motor')!='OK':
                    raise MoveError('Robot no longer ready for trajectory',bool(sent),'trajectory_not_ready')
                elapsed = now-started
                target = path.at(elapsed)
                # Feedback lag is bounded by the same freshness gate. Compare to
                # the commanded history at receipt age, not only a future target.
                reference = path.at(max(0.,elapsed-age))
                errors = [abs(math.degrees(a-b)) for a,b in zip(q,reference)]
                worst = max(range(len(errors)),key=errors.__getitem__)
                tracking_error = errors[worst]
                if tracking_error>=diagnostics['max_tracking_error_deg']:
                    diagnostics.update(max_tracking_error_deg=tracking_error,
                        worst_tracking_joint_index=worst,
                        worst_tracking_actual_deg=math.degrees(q[worst]),
                        worst_tracking_reference_deg=math.degrees(reference[worst]),
                        worst_tracking_command_deg=math.degrees(target[worst]))
                if tracking_error>TRACKING_DEG:
                    raise MoveError('Trajectory tracking error exceeds 8 degrees',bool(sent),'trajectory_tracking')
                for index,(begins,dwell,goal) in enumerate(path.holds):
                    if index in verified_holds or elapsed<begins:
                        continue
                    if elapsed>begins+dwell:
                        raise MoveError('Preset presentation hold was not verified before return',bool(sent),'target_not_reached')
                    if fresh:
                        stable = hold_windows[index]
                        if max(abs(math.degrees(a-b)) for a,b in zip(q,goal))>TOLERANCE_DEG or any(v is None or abs(v)>.05 for v in dq):
                            stable.clear()
                        else:
                            if stable and max(abs(math.degrees(a-b)) for a,b in zip(q,stable[0][1]))>STABILITY_DEG:
                                stable.clear()
                            stable.append((now,q))
                            if len(stable)>=3 and now-stable[0][0]>=STABILITY_SECONDS:
                                verified_holds.add(index)
                if guard_task and guard_task.done():
                    guard_task.result()
                    guard_task = None
                if guard_task and now-guard_started>5:
                    raise MoveError('Trajectory control-source check timed out',bool(sent),'trajectory_guard')
                if not guard_task and now>=next_guard:
                    guard_started = now
                    guard_task = asyncio.create_task(job['guard']())
                    next_guard = now+CONTROL_GUARD_INTERVAL
                # Always hold native head axes at zero in the 16D protocol.
                # Pure 14D arm validation also runs in the transport per sample.
                job['motion_sent'] = True if sent else None
                send_started=time.monotonic()
                try:
                    await reader.send_pose_servoj(target+[0.,0.],token)
                finally:
                    diagnostics['max_socket_send_seconds']=max(diagnostics['max_socket_send_seconds'],time.monotonic()-send_started)
                sent += 1
                last_output = now  # Include time spent blocked in submission.
                if first:
                    first = False
                    admitted({'status':'submitted','motion_sent':True,'vendor_acknowledged':False,
                        'transport':'request_servoj','positive_ack_supported':False})
                job['trajectory_samples_sent'] = job.get('trajectory_samples_sent',0)+1
                if elapsed>=path.duration:
                    error = max(abs(math.degrees(a-b)) for a,b in zip(q,path.target))
                    if fresh:
                        if error>TOLERANCE_DEG or any(v is None or abs(v)>.05 for v in dq):
                            window = []
                        else:
                            if window and max(abs(math.degrees(a-b)) for a,b in zip(q,window[0][1]))>STABILITY_DEG:
                                window = []
                            window.append((now,q))
                            if len(window)>=3 and now-window[0][0]>=STABILITY_SECONDS:
                                if guard_task:
                                    await guard_task
                                if token.get('error'):
                                    raise MoveError(token['error'],True,'trajectory_vendor_fault')
                                return {'arrival_verified':True,'verified_samples':len(window),
                                    'max_error_deg':error,'verification':'trajectory_terminal_advancing_stationary_feedback'}
                    if elapsed>path.duration+FINAL_TIMEOUT:
                        raise MoveError('Trajectory terminal arrival not verified',True,'target_not_reached')
                # Rebase to actual completion: late ticks never create a burst.
                await asyncio.sleep(max(0.,1/OUTPUT_HZ-(time.monotonic()-now)))
    finally:
        diagnostics.update(samples_sent=sent,elapsed_seconds=time.monotonic()-started)
        diagnostics['average_output_hz']=sent/diagnostics['elapsed_seconds'] if diagnostics['elapsed_seconds'] else 0.
        if guard_task and not guard_task.done():
            guard_task.cancel()
            await asyncio.gather(guard_task,return_exceptions=True)
