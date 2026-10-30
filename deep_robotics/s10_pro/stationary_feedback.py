"""Shared bounded stationary tolerance for reported motion feedback."""
from collections import deque
import math
import threading
import time

LINEAR_TOLERANCE_MPS = .05
YAW_TOLERANCE_RADPS = .06
STABLE_DURATION_S = .3
STABLE_SAMPLES = 3
MAX_SAMPLE_GAP_S = .35
BIAS_SPEED_LIMIT_MPS = .20
BIAS_VARIATION_LIMIT_MPS = .02
ODOMETRY_SPEED_LIMIT_MPS = .02
ODOMETRY_YAW_RATE_LIMIT_RADPS = .03
ODOMETRY_POSITION_SPAN_M = .01
ODOMETRY_YAW_SPAN_RAD = math.radians(1.)
CORROBORATED_DURATION_S = .5
CORROBORATED_SAMPLES = 4


def stationary_velocity(observed):
    speed = observed.get('velocity')
    if observed.get('fresh') is not True or not isinstance(speed, dict):
        return False
    if any(type(speed.get(k)) not in (int, float) or not math.isfinite(speed[k])
           for k in ('vx', 'vy', 'yaw_rate')):
        return False
    return (math.hypot(speed['vx'], speed['vy']) < LINEAR_TOLERANCE_MPS
            and abs(speed['yaw_rate']) < YAW_TOLERANCE_RADPS)


class StationaryWindow:
    """Use distinct samples; cached frames and one quiet frame cannot qualify."""
    def __init__(self, *, clock=time.monotonic):
        self._clock = clock
        self._samples = deque(maxlen=32)
        self._corroborated = deque(maxlen=32)
        self._lock = threading.RLock()

    def reset(self):
        with self._lock:
            self._samples.clear()
            self._corroborated.clear()

    def _odometry_check(self, observed, odometry, states, now):
        """Bounded bias exception; never rewrite measured motion or learn a zero."""
        odometry = odometry if isinstance(odometry, dict) else {}
        speed = observed.get('velocity') or {}
        fields = ('vx', 'vy', 'yaw_rate', 'x', 'y', 'yaw', 'received_monotonic', 'source_stamp_s')
        valid = (odometry.get('fresh') is True
            and type(odometry.get('sequence')) is int and odometry['sequence'] > 0
            and all(type(odometry.get(k)) in (int, float) and math.isfinite(odometry[k]) for k in fields)
            and odometry['source_stamp_s'] > 0
            and 0 <= now - odometry['received_monotonic'] <= MAX_SAMPLE_GAP_S
            and observed.get('fresh') is True
            and type(observed.get('sequence')) is int and observed['sequence'] > 0
            and type(observed.get('received_monotonic')) in (int, float)
            and math.isfinite(observed['received_monotonic'])
            and 0 <= now - observed['received_monotonic'] <= MAX_SAMPLE_GAP_S
            and observed.get('motion_state') in states
            and isinstance(speed, dict)
            and all(type(speed.get(k)) in (int, float) and math.isfinite(speed[k]) for k in ('vx', 'vy', 'yaw_rate')))
        quiet = (valid and math.hypot(speed['vx'], speed['vy']) < BIAS_SPEED_LIMIT_MPS
            and abs(speed['yaw_rate']) < YAW_TOLERANCE_RADPS
            and math.hypot(odometry['vx'], odometry['vy']) < ODOMETRY_SPEED_LIMIT_MPS
            and abs(odometry['yaw_rate']) < ODOMETRY_YAW_RATE_LIMIT_RADPS)
        if not quiet:
            self._corroborated.clear()
        else:
            sample = {'motion_sequence': observed['sequence'],
                'motion_received': observed['received_monotonic'],
                'odom_sequence': odometry['sequence'], 'odom_received': odometry['received_monotonic'],
                'odom_stamp': odometry['source_stamp_s'],
                'state': observed['motion_state'], 'vx': speed['vx'], 'vy': speed['vy'],
                'x': odometry['x'], 'y': odometry['y'], 'yaw': odometry['yaw']}
            if self._corroborated:
                last = self._corroborated[-1]
                # Rewinds, gaps, changed posture and changed bias retire all old evidence.
                if (sample['state'] != last['state'] or any(
                        sample[k] < last[k] for k in ('motion_sequence', 'odom_sequence', 'motion_received', 'odom_received', 'odom_stamp'))
                        or any(sample[k] - last[k] > MAX_SAMPLE_GAP_S for k in ('motion_received', 'odom_received'))
                        or any(math.hypot(sample['vx'] - p['vx'], sample['vy'] - p['vy']) > BIAS_VARIATION_LIMIT_MPS
                               for p in self._corroborated)):
                    self._corroborated.clear()
            # Each credited pair must advance both independent source sequences/times.
            if not self._corroborated or all(sample[k] > self._corroborated[-1][k]
                    for k in ('motion_sequence', 'odom_sequence', 'motion_received', 'odom_received', 'odom_stamp')):
                self._corroborated.append(sample)
            while self._corroborated and any(now - self._corroborated[0][k] > .8
                    for k in ('motion_received', 'odom_received')):
                self._corroborated.popleft()
        samples = list(self._corroborated)
        position_span = max((math.hypot(a['x'] - b['x'], a['y'] - b['y'])
                             for a in samples for b in samples), default=0.)
        yaw_span = max((abs(math.atan2(math.sin(a['yaw'] - b['yaw']), math.cos(a['yaw'] - b['yaw'])))
                       for a in samples for b in samples), default=0.)
        duration = min((samples[-1][k] - samples[0][k] for k in ('motion_received', 'odom_received')), default=0.) if samples else 0.
        within_pose = position_span < ODOMETRY_POSITION_SPAN_M and yaw_span < ODOMETRY_YAW_SPAN_RAD
        # Movement anywhere in the window invalidates it, including out-and-back motion.
        if not within_pose:
            self._corroborated.clear()
        ready = quiet and within_pose and len(samples) >= CORROBORATED_SAMPLES and duration + 1e-9 >= CORROBORATED_DURATION_S
        return {'ready': bool(ready), 'fresh': bool(valid), 'matching_samples': len(samples),
            'stable_duration_s': max(0., duration), 'required_duration_s': CORROBORATED_DURATION_S,
            'required_samples': CORROBORATED_SAMPLES, 'position_span_m': position_span,
            'position_span_limit_m': ODOMETRY_POSITION_SPAN_M,
            'yaw_span_rad': yaw_span, 'yaw_span_limit_rad': ODOMETRY_YAW_SPAN_RAD,
            'motion_speed_limit_mps': BIAS_SPEED_LIMIT_MPS,
            'odometry_speed_limit_mps': ODOMETRY_SPEED_LIMIT_MPS}

    def observe(self, observed, states, odometry=None):
        with self._lock:
            sequence, received = observed.get('sequence'), observed.get('received_monotonic')
            now = self._clock()
            valid = (observed.get('fresh') is True and type(sequence) is int and sequence > 0
                and type(received) in (int, float) and math.isfinite(received)
                and 0 <= now - received <= MAX_SAMPLE_GAP_S)
            if not valid:
                self._samples.clear()
            else:
                if self._samples and (sequence < self._samples[-1]['sequence']
                        or received < self._samples[-1]['received']
                        or received - self._samples[-1]['received'] > MAX_SAMPLE_GAP_S):
                    self._samples.clear()
                if not self._samples or sequence != self._samples[-1]['sequence']:
                    self._samples.append({'sequence': sequence, 'received': received,
                        'state': observed.get('motion_state'),
                        'within_tolerance': stationary_velocity(observed)})
                while self._samples and now - self._samples[0]['received'] > .8:
                    self._samples.popleft()
            tail = []
            for sample in reversed(self._samples):
                if not sample['within_tolerance'] or sample['state'] not in states:
                    break
                tail.append(sample)
            duration = tail[0]['received'] - tail[-1]['received'] if tail else 0.
            ready = valid and len(tail) >= STABLE_SAMPLES and duration + 1e-9 >= STABLE_DURATION_S
            speed = observed.get('velocity') or {}
            if not isinstance(speed, dict):
                speed = {}
            corroborated = self._odometry_check(observed, odometry, states, now)
            basis = 'motion_feedback'
            if not ready and corroborated['ready']:
                ready, basis = True, 'motion_and_odometry'
            return {'ready': bool(ready), 'matching_samples': len(tail) if basis == 'motion_feedback' else corroborated['matching_samples'],
                'feedback_fresh': bool(valid),
                'required_samples': STABLE_SAMPLES if basis == 'motion_feedback' else CORROBORATED_SAMPLES,
                'stable_duration_s': max(0., duration) if basis == 'motion_feedback' else corroborated['stable_duration_s'],
                'required_duration_s': STABLE_DURATION_S if basis == 'motion_feedback' else CORROBORATED_DURATION_S,
                'linear_tolerance_mps': LINEAR_TOLERANCE_MPS,
                'yaw_tolerance_radps': YAW_TOLERANCE_RADPS,
                'reported_velocity': dict(speed), 'sequence': sequence,
                'motion_state': observed.get('motion_state'), 'confirmation_basis': basis,
                'odometry_corroboration': corroborated}
