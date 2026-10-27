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
        self._lock = threading.RLock()

    def reset(self):
        with self._lock:
            self._samples.clear()

    def observe(self, observed, states):
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
            return {'ready': bool(ready), 'matching_samples': len(tail),
                'required_samples': STABLE_SAMPLES, 'stable_duration_s': max(0., duration),
                'required_duration_s': STABLE_DURATION_S,
                'linear_tolerance_mps': LINEAR_TOLERANCE_MPS,
                'yaw_tolerance_radps': YAW_TOLERANCE_RADPS,
                'reported_velocity': dict(speed), 'sequence': sequence,
                'motion_state': observed.get('motion_state')}
