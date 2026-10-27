"""Read-only, request-scoped feedback from the documented S10 /MOTION_INFO.

No ROS participant, publisher, SDK or socket is constructed here. This module
correlates observations in local software; the vendor message does not carry a
command/request ID. Motion state and velocity do not establish control ownership,
HES, usage mode, SDK permission, or a robot-side deadman guarantee.
"""
from __future__ import annotations

import copy
import math
import threading
import time


SOFTWARE_GUIDE = 'https://support.deeprobotics.cn/docs/#/259676015'
POSTURE_TARGETS = {'stand_pose': 1, 'stand': 17, 'lie': 4, 'lie_idle': 0}
ACTIONS = frozenset((*POSTURE_TARGETS, 'velocity', 'stopmotion'))
UNKNOWN_EXECUTION_CONDITIONS = (
    'control_usage_mode', 'hard_emergency_stop', 'control_ownership',
    'robot_side_loss_of_command_stop', 'firmware_protocol_compatibility',
)


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _bounded(value, low, high, name):
    if not _finite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be finite and in {low}..{high}')
    return float(value)


def _sequence(value):
    return value if type(value) is int and value > 0 else 0


class MotionFeedback:
    """Require fresh, distinct, post-request samples before confirming a goal.

    ``snapshot`` is a no-argument callable returning a ReaderBridge snapshot for
    source ``motion_info``. ``begin`` must run immediately before a backend's
    first publish, and only once per action stage. Repeated begin calls for an
    unchanged action/request/generation are idempotent, so velocity refreshes do
    not continually move the observation baseline. ``stand_pose`` is an internal
    DDS stage for state 1; ``stand`` confirms final RL state 17. Changing stages
    creates a new baseline and invalidates confirmation of the previous stage.
    ``lie_idle`` waits for the vendor state machine to finish LieDown and return
    to Idle (state 0); observing the intermediate command state 4 is not enough.

    Callers must not infer physical completion from ``confirmed`` alone: this
    reports only a matching measured state after a local request marker. No
    vendor acknowledgement or exclusive authority is implied.
    """

    def __init__(self, snapshot, *, clock=time.monotonic, max_age_s=.5,
                 stable_samples=2, stop_linear_tolerance=.02,
                 stop_yaw_tolerance=.03, velocity_linear_tolerance=.03,
                 velocity_yaw_tolerance=.05, context_ttl_s=15.):
        if not callable(snapshot):
            raise ValueError('snapshot must be callable')
        if type(stable_samples) is not int or not 2 <= stable_samples <= 20:
            raise ValueError('stable_samples must be an integer in 2..20')
        self._snapshot, self._clock = snapshot, clock
        self.max_age_s = _bounded(max_age_s, .01, .5, 'max_age_s')
        self.stable_samples = stable_samples
        self.stop_linear_tolerance = _bounded(stop_linear_tolerance, .001, .05, 'stop_linear_tolerance')
        self.stop_yaw_tolerance = _bounded(stop_yaw_tolerance, .001, .1, 'stop_yaw_tolerance')
        self.velocity_linear_tolerance = _bounded(velocity_linear_tolerance, .001, .1, 'velocity_linear_tolerance')
        self.velocity_yaw_tolerance = _bounded(velocity_yaw_tolerance, .001, .2, 'velocity_yaw_tolerance')
        self.context_ttl_s = _bounded(context_ttl_s, .5, 30., 'context_ttl_s')
        self._lock = threading.RLock()
        self._generation = 0
        self._owner_generation = -1
        self._context = None

    @staticmethod
    def _request(action, request):
        if action not in ACTIONS:
            raise ValueError('unsupported feedback action')
        if request is None:
            request = {}
        if not isinstance(request, dict):
            raise ValueError('request must be an object')
        if action != 'velocity':
            return {}
        # Documented physical bounds. The action controller may impose smaller
        # per-session limits; this observer never raises those limits.
        result = {}
        for key, maximum in [('vx', 1.67), ('vy', .4), ('yaw_rate', 1.)]:
            result[key] = _bounded(request.get(key, 0), -maximum, maximum, key)
        return result

    def _raw(self):
        try:
            value = self._snapshot()
            return value if isinstance(value, dict) else {}
        except Exception:
            return {}

    def _observation(self, raw, now):
        """Validate one source record, never comparing vendor wall-clock stamps."""
        result = {'fresh': False, 'sequence': _sequence(raw.get('sequence')),
                  'age_s': None, 'received_monotonic': None,
                  'motion_state': None, 'gait': None, 'velocity': None,
                  'reason': 'no_source_data'}
        age = raw.get('age_s')
        if _finite(age) and age >= 0:
            result['age_s'] = float(age)
        if raw.get('error'):
            result['reason'] = 'source_error'
            return result
        if raw.get('fresh') is not True or not _finite(age) or not 0 <= age <= self.max_age_s:
            result['reason'] = 'source_not_fresh'
            return result
        received = raw.get('received_monotonic')
        if received is None:
            # Compatibility for older ReaderBridge snapshots. Sequence and the
            # begin marker remain independent guards against pre-request cache.
            received = now - age
        if (not _finite(received) or received > now or now - received > self.max_age_s
                or abs((now - received) - age) > .1):
            result['reason'] = 'invalid_receive_time'
            return result
        result['received_monotonic'] = float(received)
        if not result['sequence']:
            result['reason'] = 'invalid_sequence'
            return result
        message = raw.get('data')
        values = message.get('data') if isinstance(message, dict) else None
        if not isinstance(values, dict):
            result['reason'] = 'invalid_motion_message'
            return result
        state = values.get('motion_state')
        state = state.get('state') if isinstance(state, dict) else None
        if type(state) is not int:
            result['reason'] = 'invalid_motion_state'
            return result
        speed = [values.get(name) for name in ('vel_x', 'vel_y', 'vel_yaw')]
        if not all(_finite(value) for value in speed):
            result['reason'] = 'invalid_velocity'
            return result
        gait = values.get('gait_state')
        gait = gait.get('gait') if isinstance(gait, dict) else None
        result.update(fresh=True, motion_state=state,
                      gait=gait if type(gait) is int else None,
                      velocity=dict(zip(('vx', 'vy', 'yaw_rate'), map(float, speed))),
                      reason='observed')
        return result

    def observation(self):
        raw = self._raw()
        return self._observation(raw, self._clock())

    def preflight(self):
        observed = self.observation()
        return {**observed, 'read_only': True, 'source': '/MOTION_INFO',
                'message_type': 'drdds/msg/MotionInfo',
                'units': {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'},
                'execution_ready': False, 'control_usage_mode': None,
                'hard_emergency_stop': None, 'control_ownership': None,
                'robot_deadman_verified': False,
                'unknown_conditions': list(UNKNOWN_EXECUTION_CONDITIONS),
                'source_reference': SOFTWARE_GUIDE,
                'note': 'Motion feedback is observable; execution authority and stopping guarantees are not inferred.'}

    def begin(self, action, request_id, request=None, *, generation=None):
        target = self._request(action, request)
        if not isinstance(request_id, str) or not request_id.strip() or len(request_id) > 128:
            raise ValueError('invalid request_id')
        if generation is not None and (type(generation) is not int or generation < 0):
            raise ValueError('generation must be a nonnegative integer')
        with self._lock:
            if generation is not None and generation < self._owner_generation:
                raise ValueError('retired owner generation cannot begin feedback')
            if generation is not None:
                self._owner_generation = generation
            old = self._context
            if old and old['action'] == action and old['request_id'] == request_id and old['owner_generation'] == generation:
                if old['target'] != target:
                    raise ValueError('an active feedback context cannot change targets')
                return self._public_context(old)
            raw = self._raw()
            now = self._clock()
            self._generation += 1
            self._context = {'action': action, 'request_id': request_id,
                'owner_generation': generation, 'generation': self._generation,
                'target': target, 'begin_monotonic': now,
                'baseline_sequence': _sequence(raw.get('sequence')),
                'last_sequence': _sequence(raw.get('sequence')), 'matching_samples': 0,
                'last_receive_monotonic': None,
                'failed_reason': None}
            return self._public_context(self._context)

    @staticmethod
    def _public_context(context):
        return {key: copy.deepcopy(context[key]) for key in (
            'action', 'request_id', 'generation', 'owner_generation',
            'target', 'begin_monotonic', 'baseline_sequence')}

    def invalidate(self, request_id=None, *, generation=None):
        """Retire only the matching local context; old cancels cannot retire new work."""
        with self._lock:
            context = self._context
            if context is None:
                return False
            if request_id is not None and context['request_id'] != request_id:
                return False
            if generation is not None and context['generation'] != generation:
                return False
            self._generation += 1
            self._context = None
            return True

    def _matches(self, action, observed, target):
        state, speed = observed['motion_state'], observed['velocity']
        if action in POSTURE_TARGETS:
            return state == POSTURE_TARGETS[action]
        if action == 'stopmotion':
            return all(abs(speed[key]) <= (self.stop_yaw_tolerance if key == 'yaw_rate' else self.stop_linear_tolerance)
                       for key in speed)
        if state != 17:
            return False
        for key, requested in target.items():
            actual = speed[key]
            tolerance = self.velocity_yaw_tolerance if key == 'yaw_rate' else self.velocity_linear_tolerance
            if abs(actual - requested) > tolerance:
                return False
            # Even a small nonzero request must have a matching direction and
            # measurable amplitude. A near-zero sensor value cannot prove a
            # small positive/negative movement merely by absolute tolerance.
            if requested != 0 and (actual * requested <= 0 or abs(actual) < abs(requested) * .5):
                return False
        return True

    def poll(self, action, request_id):
        with self._lock:
            raw = self._raw()
            now = self._clock()
            observed = self._observation(raw, now)
            result = {**observed, 'confirmed': False, 'request_id': request_id,
                'vendor_request_id': None, 'correlation': 'local_request_marker_only',
                'generation': None, 'matching_samples': 0,
                'required_samples': self.stable_samples, 'action': action,
                'zero_target': False}
            context = self._context
            if context is None:
                result['reason'] = 'no_request_context'
                return result
            result['generation'] = context['generation']
            if context['request_id'] != request_id or context['action'] != action:
                result['reason'] = 'request_context_mismatch'
                return result
            if now < context['begin_monotonic'] or now - context['begin_monotonic'] > self.context_ttl_s:
                context['failed_reason'] = 'context_expired'
            if context['failed_reason']:
                result['reason'] = context['failed_reason']
                return result
            if not observed['fresh']:
                context['matching_samples'] = 0
                context['last_receive_monotonic'] = None
                return result
            sequence = observed['sequence']
            if sequence < context['last_sequence']:
                context['failed_reason'] = 'source_sequence_regressed'
                result['reason'] = context['failed_reason']
                return result
            if (sequence <= context['baseline_sequence']
                    or observed['received_monotonic'] < context['begin_monotonic']):
                result['reason'] = 'not_after_request'
                return result
            if sequence != context['last_sequence']:
                context['last_sequence'] = sequence
                previous_receive = context['last_receive_monotonic']
                if (previous_receive is not None
                        and observed['received_monotonic'] - previous_receive > self.max_age_s):
                    context['matching_samples'] = 0
                context['last_receive_monotonic'] = observed['received_monotonic']
                if self._matches(action, observed, context['target']):
                    context['matching_samples'] += 1
                else:
                    context['matching_samples'] = 0
            result['matching_samples'] = context['matching_samples']
            result['confirmed'] = context['matching_samples'] >= self.stable_samples
            result['reason'] = 'observed_goal' if result['confirmed'] else 'awaiting_stable_goal'
            result['zero_target'] = action == 'velocity' and not any(context['target'].values())
            return result

    __call__ = poll
