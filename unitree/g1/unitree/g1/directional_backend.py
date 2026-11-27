"""Point with the right arm while holding the left at its measured start.

Each call plans from fresh feedback and uses the shared ten-joint executor.
"""
import copy
import math
import threading
import time

from common.motion.protocol import envelope, vector
from arm_stream import require_directional_standing
from directional_kinematics import DirectionalKinematics, normalize_direction


class DirectionalEnvelopeError(RuntimeError):
    def __init__(self, reason, evidence):
        super().__init__(reason)
        self.envelope_failure = evidence


class DirectionalBackend:
    def __init__(self, motion, *, stationary, config=None, kinematics=None):
        self.motion = motion
        self.arm = motion.executor
        self.stationary = stationary
        self.config = dict(config or {})
        if self.config.get('left_arm_mode') != 'hold_measured':
            raise RuntimeError('directional_requires_explicit_left_hold')
        if self.config.get('solver_mode') != 'bounded_ik':
            raise RuntimeError('directional_requires_bounded_ik')
        self.kinematics = kinematics or DirectionalKinematics(self.config)
        self._run_lock = threading.Lock()

    def _geometry_matches(self):
        if self.arm.versions['model_version'] != self.kinematics.model_sha256:
            raise RuntimeError('directional_execution_model_mismatch')
        hand = self.arm.profile.get('palm_frames', {}).get('right')
        if hand != {'position': [.2, 0., 0.], 'orientation': [0., 0., 0., 1.]}:
            raise RuntimeError('directional_execution_hand_calibration_mismatch')

    def _number(self, key, default, low, high):
        value = self.config.get(key, default)
        if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
            raise RuntimeError('invalid_' + key)
        return float(value)

    def snapshot(self):
        with self.arm.lock:
            sample, q, dq = self.arm._fresh()
            state = self.arm.status()
        return {'q': q[5:], 'dq': dq[5:], 'arm_ns': sample['arm_ns'],
                'state': state['state'], 'ownership_held': state['ownership_held']}

    def _read(self, lease=None, *, unowned=False):
        with self.arm.lock:
            state = self.arm.status()
            if unowned and (state['ownership_held'] or self.arm._legacy_busy()):
                raise RuntimeError('directional_arm_owned')
            if lease is not None:
                self._require_lease(lease, state=state)
            sample, q, dq = self.arm._fresh()
            return sample, q, dq

    def _require_lease(self, lease, *, state=None, unstarted=False):
        """Keep a same-lease hold/fault distinct from another owner's claim."""
        state = self.arm.status() if state is None else state
        same = state['session_id'] == lease['session_id']
        release = state.get('release') or {}
        own_release = (state['session_id'] is None
                       and release.get('session_id') == lease['session_id'])
        if not same and not own_release:
            raise RuntimeError('directional_ownership_changed')
        if state.get('directional_fault'):
            raise RuntimeError(state['directional_fault'])
        if own_release:
            raise RuntimeError('directional_execution_released:' + str(
                release.get('reason') or release.get('state', 'unknown')))
        if state['state'] not in ('ready', 'active') or state['release_requested']:
            raise RuntimeError(state.get('reason') or 'directional_execution_interrupted')
        if unstarted and (state['sequence'] != -1 or state['applied_sequence'] != -1):
            raise RuntimeError('directional_initial_sequence_changed')
        if unstarted and state['weight'] != 0.:
            raise RuntimeError('directional_initial_weight_changed')
        if unstarted and state['state'] != 'ready':
            raise RuntimeError(state.get('reason') or 'directional_initial_state_changed')

    @staticmethod
    def _finite_vector(values):
        if (isinstance(values, (list, tuple)) and len(values) == 10
                and all(type(value) in (float, int) and math.isfinite(value) for value in values)):
            return list(values)
        return None

    def _feedback_evidence(self, sample):
        stamp = sample.get('arm_ns')
        return {'feedback_ns': stamp if type(stamp) is int else None,
                'feedback_age_ns': self.arm.clock()-stamp if type(stamp) is int else None,
                'q_rad': self._finite_vector(sample.get('q')),
                'dq_rad_s': self._finite_vector(sample.get('dq'))}

    def _startup_evidence(self, lease, phase, reason):
        """Diagnostic only; never replace the original exception on failure."""
        try:
            with self.arm.lock:
                state = self.arm.status()
                return {'phase': phase, 'reason': str(reason),
                        'same_session': bool(lease and state['session_id'] == lease['session_id']),
                        'state': state['state'], 'state_reason': state.get('reason'),
                        'directional_fault': state.get('directional_fault'),
                        'sequence': state['sequence'], 'applied_sequence': state['applied_sequence'],
                        'release_requested': state['release_requested'],
                        'first_fault': copy.deepcopy(state.get('first_fault')),
                        **self._feedback_evidence(state.get('feedback') or {})}
        except Exception as exc:
            return {'phase': phase, 'reason': str(reason), 'evidence_error': type(exc).__name__}

    def _release_evidence(self, lease, reason):
        """Read only this lease's operation, retaining its actual timeout phase."""
        evidence = {'reason': str(reason), 'operation_match': False}
        try:
            with self.arm.lock:
                operations = [operation for operation in self.arm._operations.values()
                              if operation.get('session_id') == lease['session_id']]
                current = self.arm._operation
                if current and current.get('session_id') == lease['session_id']:
                    operations.append(current)
                if operations:
                    operation = max(operations, key=lambda item: item.get('started_ns', 0))
                    for key in ('operation_id', 'state', 'reason', 'timed_out_state', 'action_id',
                                'started_ns', 'handback_started_ns', 'zero_confirmed_ns',
                                'zero_writes', 'handback_confirmed', 'physical_confirmed',
                                'authority_released', 'sdk_return'):
                        evidence[key] = copy.deepcopy(operation.get(key))
                    evidence['operation_match'] = True
                else:
                    evidence['state'] = 'no_matching_operation'
                sample = self.arm._snapshot()
                evidence.update(self._feedback_evidence(sample))
                evidence.update(rest_velocity_limit_rad_s=.02,
                                rest_position_delta_limit_rad=.002, rest_window_ns=100_000_000)
                dq = evidence['dq_rad_s']
                if dq is not None:
                    index = max(range(10), key=lambda i: abs(dq[i]))
                    evidence.update(max_abs_dq_rad_s=abs(dq[index]),
                                    max_dq_motor_id=(15,16,17,18,19,22,23,24,25,26)[index])
                # _settle belongs to the current operation; never attach a
                # different lease's confirmation window to this error.
                settle = self.arm._settle if current and current.get('session_id') == lease['session_id'] else None
                if settle and evidence['q_rad'] is not None:
                    reference = self._finite_vector(settle[1])
                    if reference is not None:
                        delta = [abs(a-b) for a,b in zip(evidence['q_rad'], reference)]
                        index = max(range(10), key=delta.__getitem__)
                        evidence.update(settle_since_ns=settle[0], settle_reference_q_rad=reference,
                                        max_settle_delta_rad=delta[index],
                                        max_settle_delta_motor_id=(15,16,17,18,19,22,23,24,25,26)[index])
        except Exception as exc:
            evidence['evidence_error'] = type(exc).__name__
        return evidence

    def _submit(self, lease, seq, reference, sample, bounds):
        """Called under the actuator lock; contains no geometry work or IPC."""
        self._require_lease(lease)
        generated = self.arm.clock()
        packet = envelope(lease, seq=seq, source_seq=seq, mapping_epoch=0,
                          generated_ns=generated, valid_until_ns=generated+150_000_000,
                          mode='joint_position', dof=10, values=reference, **self.arm.versions)
        record = {key: packet[key] for key in ('boot_id', 'session_id', 'seq', 'source_seq',
                  'mapping_epoch', 'model_version', 'calibration_version', 'frame', 'valid_until_ns')}
        record.update(schema='motus.motion.envelope/1', measured_ns=sample['arm_ns'],
                      lower=bounds['lower'], upper=bounds['upper'])
        self.arm.install_motion_envelope(record)
        if not self.arm.accept_control(packet):
            raise RuntimeError('directional_command_rejected')

    def _check_start(self, q, expected, tolerance):
        if max(abs(a-b) for a, b in zip(q[5:], expected)) > tolerance:
            raise RuntimeError('directional_start_pose_mismatch')

    def _parameters(self, target_q5, hold_s, planned_start_q5=None):
        target = vector(target_q5, 5, 'directional_target')
        if planned_start_q5 is None:
            _, q, _ = self._read(unowned=True)
            planned_start_q5 = q[5:]
        expected = vector(planned_start_q5, 5, 'directional_planned_start')
        tolerance = self._number('start_tolerance_rad', .05, .001, .10)
        speed = self._number('joint_speed_rad_s', .15, .01, 1.0)
        target_tolerance = self._number('target_tolerance_rad', .04, .005, .10)
        if type(hold_s) not in (int, float) or not math.isfinite(hold_s) or not 0 <= hold_s <= 10:
            raise RuntimeError('invalid_hold_s')
        # FK is local and read-only; it also checks the fixed model's limits.
        self.kinematics.direction(expected)
        self.kinematics.direction(target)
        return {'target': target, 'expected': expected, 'tolerance': tolerance,
                'speed': speed, 'target_tolerance': target_tolerance}

    def _admission(self, parameters, lease=None):
        """Read current conditions without preparing, claiming or commanding."""
        require_directional_standing(self.stationary)
        with self.arm.lock:
            if self.arm.cfg.get('live_enabled') is not True:
                raise RuntimeError('live_disabled')
            if self.arm.cfg.get('servo_position') is not True:
                raise RuntimeError('directional_requires_servo_position')
            sample, q, dq = self._read(lease, unowned=lease is None)
            self._geometry_matches()
            self._check_start(q, parameters['expected'], parameters['tolerance'])
            target = q[:5] + parameters['target']
            if (len(self.arm.limits) != 10 or any(
                    not lower <= measured <= upper or not lower <= requested <= upper
                    for measured, requested, (lower, upper) in zip(q, target, self.arm.limits))):
                raise RuntimeError('motion_envelope_joint_limit')
            self.kinematics.check_measured_pose(q[5:], left_q5=q[:5])
            distance = max(abs(a-b) for a, b in zip(q[5:], parameters['target']))
            # Smoothstep's peak derivative is 1.5. This is a duration check,
            # not collision checking or a reservation of the observed state.
            duration = max(1.0, 1.5*distance/parameters['speed'])
            if duration > 30:
                raise RuntimeError('directional_approach_too_long')
        return sample, q, dq, duration

    def _admit(self, parameters, stop=None, *, claim=False, lease=None, execution_guard=None):
        """Check the current sample and optionally acquire the shared arms."""
        if stop is not None and stop.is_set():
            raise RuntimeError('cancelled')
        admission = self._admission(parameters, lease)
        if stop is not None and stop.is_set():
            raise RuntimeError('cancelled')
        if claim:
            with self.arm._management_lock:
                if stop is not None and stop.is_set():
                    raise RuntimeError('cancelled')
                # The executor seeds its filter from fresh measured positions
                # taken after the channel has opened.
                return self.arm._claim(guard=execution_guard or self.stationary,
                                       velocity=parameters['speed'])
        return admission

    def plan(self, direction, hold_s):
        """Solve against this call's measured right-arm start without ownership."""
        if not self._run_lock.acquire(False):
            raise RuntimeError('directional_busy')
        try:
            require_directional_standing(self.stationary)
            _, q, _ = self._read(unowned=True)
            target = self.kinematics.solve(direction, start_q5=q[5:], left_q5=q[:5])
            parameters = self._parameters(target, hold_s, q[5:])
            sample, current, _, duration = self._admit(parameters)
            self.kinematics.check_path(current[5:], target, left_q5=current[:5])
            return {'target_q_rad': target, 'start_q_rad': q[5:],
                    'planned_direction': self.kinematics.direction(target),
                    'feedback_ns': sample['arm_ns'], 'approach_s': duration}
        finally:
            self._run_lock.release()

    def check_ready(self, target_q5, hold_s, *, planned_start_q5=None):
        """Read-only admission before an action ID is issued; reserves nothing.

        Never prepares a numerical worker, opens a channel or acquires arm
        authority. Execution repeats these checks around its actual claim.
        """
        if not self._run_lock.acquire(False):
            raise RuntimeError('directional_busy')
        try:
            parameters = self._parameters(target_q5, hold_s, planned_start_q5)
            sample, q, _, duration = self._admit(parameters)
            if self.config.get('solver_mode') == 'bounded_ik':
                self.kinematics.check_path(q[5:], parameters['target'], left_q5=q[:5])
            return {'ready': True, 'approach_s': duration, 'feedback_ns': sample['arm_ns']}
        except (ValueError, RuntimeError, OverflowError) as exc:
            raise RuntimeError(str(exc)) from exc
        finally:
            self._run_lock.release()

    def _release(self, lease):
        """Release only this lease, preserving an unresolved owner on failure."""
        # Management paths take management -> actuator; preserve that order
        # when atomically inspecting ownership and requesting handback.
        with self.arm._management_lock, self.arm.lock:
            state = self.arm.status()
            if state['session_id'] == lease['session_id']:
                result = self.arm.dispatch('stop', lease)
            elif state['session_id'] is None:
                result = state.get('release') or {}
                if result.get('session_id') != lease['session_id']:
                    raise RuntimeError('directional_release_owner_changed')
            else:
                raise RuntimeError('directional_release_owner_changed')
        if result.get('error'):
            raise RuntimeError('directional_release_unconfirmed:' + str(result['error']))
        deadline = time.monotonic() + 11.0
        while True:
            if (result.get('authority_released') is True
                    and result.get('physical_confirmed') is True
                    and result.get('state') == 'completed'):
                return
            if result.get('state') in ('failed', 'unknown', 'error') or time.monotonic() >= deadline:
                raise RuntimeError('directional_release_unconfirmed:' + str(result.get('reason', result.get('state'))))
            operation_id = result.get('operation_id')
            if not operation_id:
                raise RuntimeError('directional_release_unconfirmed:no_operation')
            time.sleep(.02)
            result = self.arm.dispatch('finish_status', {'operation_id': operation_id})

    def execute(self, target_q5, hold_s, stop_event, observe, *, planned_start_q5=None,
                target_direction=None):
        """Run in the caller's worker; success means measured rest and handback.

        ``observe(phase, q5)`` receives only advancing, fresh measured samples;
        ``observe('releasing', None)`` announces handback without inventing one.
        Cancellation raises ``RuntimeError('cancelled')`` only after confirmed
        release. Callback, feedback and release errors stay errors.
        """
        if not self._run_lock.acquire(False):
            raise RuntimeError('directional_busy')
        lease = None
        error = None
        envelope_failure = None
        left_hold = None
        last_left_ns = -1
        initial = None
        target = None
        returning = False
        measured_pose_key = None
        tracking_diagnostics = None
        arrival_diagnostics = None
        longest_hold_s = 0.
        startup_phase = 'admission'
        startup_complete = False
        startup_failure = None
        release_failure = None

        def record_left(sample, q, phase):
            nonlocal left_hold, last_left_ns
            if initial is not None and sample['arm_ns'] > last_left_ns:
                error_now = max(abs(a-b) for a, b in zip(q[:5], initial[:5]))
                left_hold = {'target_q_rad': list(initial[:5]), 'actual_q_rad': list(q[:5]),
                             'max_error_rad': max(left_hold['max_error_rad'] if left_hold else 0., error_now),
                             'feedback_ns': sample['arm_ns'], 'sample_phase': phase,
                             'sampling_scope': 'active_control_ticks_and_release_endpoint'}
                last_left_ns = sample['arm_ns']

        def check_measured(sample, q):
            """Check this exact consumed sample while holding the actuator lock."""
            nonlocal envelope_failure, measured_pose_key
            stamp = sample['arm_ns']
            if type(stamp) is not int or not 0 <= self.arm.clock()-stamp <= 200_000_000:
                raise RuntimeError('arm_feedback_stale')
            for i, (measured, limits) in enumerate(zip(q, self.arm.limits)):
                if not limits[0] <= measured <= limits[1]:
                    envelope_failure = {'predicate': 'measured_joint_limit',
                        'motor_id': (15,16,17,18,19,22,23,24,25,26)[i],
                        'failing_value_rad': measured, 'lower_rad': limits[0], 'upper_rad': limits[1],
                        'feedback_ns': stamp, 'monotonic_ns': self.arm.clock()}
                    raise RuntimeError('motion_envelope_joint_limit')
            # This is a sampled pose check, not a tracking-tube certificate.
            # Repeated reads of the same unchanged sample do no geometry work.
            # Both arms came from this exact encoder sample. A left-only
            # change must invalidate the cache even if the right hand is still.
            pose_key = (stamp, tuple(q))
            if pose_key != measured_pose_key:
                self.kinematics.check_measured_pose(q[5:], left_q5=q[:5])
                measured_pose_key = pose_key

        def execution_guard():
            nonlocal tracking_diagnostics
            require_directional_standing(self.stationary)
            with self.arm.lock:
                if initial is None:
                    return {'stationary': True}
                sample, q, _ = self.arm._fresh()
                previous = vector(self.arm.last_q, 10, 'directional_last_reference')
                record_left(sample, q, 'controlled')
                errors = [measured-commanded for measured, commanded in zip(q, previous)]
                maximum = tracking_diagnostics['max_abs_error_rad'] if tracking_diagnostics else [0.]*10
                tracking_diagnostics = {
                    'motor_ids': [15,16,17,18,19,22,23,24,25,26],
                    'latest_error_rad': errors,
                    'max_abs_error_rad': [max(old, abs(error)) for old,error in zip(maximum, errors)],
                    'feedback_ns': sample['arm_ns'], 'feedback_age_ns': self.arm.clock()-sample['arm_ns'],
                    'sample_phase': 'returning' if returning else 'moving', 'weight': self.arm._weight,
                    'policy': 'diagnostic_only',
                }
                check_measured(sample, q)
                # The executor uses a scalar filter for this point lease. Its
                # command must stay on the geometry-checked joint-linear segment.
                span = [b-a for a, b in zip(initial[5:], target)]
                axis = max(range(5), key=lambda i: abs(span[i]))
                fraction = 0. if abs(span[axis]) < 1e-12 else (previous[5+axis]-initial[5+axis])/span[axis]
                expected = initial[:5] + [a+fraction*d for a, d in zip(initial[5:], span)]
                if (not -1e-9 <= fraction <= 1.+1e-9
                        or max(abs(a-b) for a, b in zip(previous, expected)) > 1e-9):
                    raise RuntimeError('directional_command_left_validated_path')
            return {'stationary': True}
        try:
            parameters = self._parameters(target_q5, hold_s, planned_start_q5)
            target = parameters['target']
            goal_direction = (list(self.kinematics.direction(target)) if target_direction is None
                              else vector(target_direction, 3, 'directional_target_direction'))
            normalize_direction(goal_direction)  # Validate, retaining the exact requested vector.
            angular_tolerance = self.kinematics.angular_tolerance_deg
            return_to_start = self.config.get('solver_mode') == 'bounded_ik'
            target_tolerance = parameters['target_tolerance']
            if not callable(observe):
                raise RuntimeError('invalid_observer')
            if stop_event.is_set():
                raise RuntimeError('cancelled')
            self._admit(parameters, stop_event)
            startup_phase = 'prepare'
            prepared = self.motion.dispatch('prepare_operator_session', {})
            if prepared.get('error'):
                raise RuntimeError(prepared['error'])
            if stop_event.is_set():
                raise RuntimeError('cancelled')
            startup_phase = 'claim'
            lease = self._admit(parameters, stop_event, claim=True, execution_guard=execution_guard)
            startup_phase = 'claim_feedback'
            sample, measured, dq, duration = self._admit(parameters, stop_event, lease=lease)
            with self.arm.lock:
                self._require_lease(lease, unstarted=True)
                # The executor seeds the servo filter from the sample taken at claim.
                # The measured arm pose above can already be a newer encoder sample.
                previous = vector(self.arm.last_q, 10, 'directional_claim_reference')
            self._check_start(previous, parameters['expected'], parameters['tolerance'])
            startup_phase = 'path_validation'
            if return_to_start:
                if previous[5:] != measured[5:]:
                    self.kinematics.check_path(previous[5:], measured[5:], left_q5=previous[:5])
                # The nominal left command is constant at the claim seed;
                # actual left and right positions are also checked together
                # on every consumed feedback sample by execution_guard.
                self.kinematics.check_path(previous[5:], target, left_q5=previous[:5])
            # The real numerical worker certifies the URDF hard-limit box.
            # Per-sample geometry and the nominal command path are checked
            # separately; there is no small command-tracking envelope here.
            lower = [limits[0] for limits in self.arm.limits]
            upper = [limits[1] for limits in self.arm.limits]
            startup_phase = 'envelope_validation'
            bounds = self.motion.solver.joint_envelope(measured, lower, upper)
            duration = max(1., 1.5 * max(abs(a-b) for a, b in zip(previous[5:], target)) / parameters['speed'])
            if duration > 30:
                raise RuntimeError('directional_approach_too_long')
            startup_phase = 'first_reference'
            with self.arm.lock:
                self._require_lease(lease, unstarted=True)
                sample, q, dq = self.arm._fresh()
                require_directional_standing(self.stationary)
                if stop_event.is_set():
                    raise RuntimeError('cancelled')
                # Seed continuity and the first packet share this short lock.
                # Path search and numerical IPC finished outside it. Trajectory
                # time begins now; there is no initial-pose holding phase.
                initial = list(previous)
                execution_guard()
                began = self.arm.clock()
                self.arm._takeover_ns = began
                seq = 1
                self._submit(lease, seq, initial, sample, bounds)
            startup_phase = 'first_observation'
            settled_since = None
            hold_since = None
            last_sample_ns = -1
            leg_start, leg_end = initial[5:], target
            endpoint_seq = endpoint_applied_ns = None
            while True:
                sample, q, dq = self._read(lease)
                require_directional_standing(self.stationary)
                if stop_event.is_set():
                    raise RuntimeError('cancelled')
                now = self.arm.clock()
                elapsed = (now-began)/1e9
                fraction = min(1., elapsed/duration)
                alpha = fraction*fraction*(3-2*fraction)
                reference = initial[:5] + [a+alpha*(b-a) for a, b in zip(leg_start, leg_end)]
                # Motion starts during authority ramp-up, but a terminal hold
                # requires an actual full-authority output from this lease.
                with self.arm.lock:
                    self._require_lease(lease)
                    # This frame may be newer than the executor's last guard
                    # tick. Validate it before consuming it for a terminal hold.
                    check_measured(sample, q)
                    authority_ready = self.arm._weight >= 1. and self.arm.applied_seq >= 1
                    if not returning and not authority_ready:
                        endpoint_applied_ns = None
                        # Authority can change while the source repeats its
                        # last frame. Clear continuity immediately, not only
                        # when another measured frame reaches the loop below.
                        hold_since = None
                        if arrival_diagnostics is not None:
                            arrival_diagnostics.update(
                                authority_ready=False, reference_at_target=False,
                                endpoint_applied_ns=None, hold_start_feedback_ns=None,
                                hold_elapsed_s=0., hold_completed=False)
                    if (not returning and endpoint_seq is not None and authority_ready
                            and self.arm.applied_seq >= endpoint_seq and endpoint_applied_ns is None):
                        # Capture once: newer output ticks must not keep moving
                        # the goalpost ahead of a valid, slightly delayed encoder.
                        endpoint_applied_ns = self.arm.last_emit
                    endpoint_ready = (endpoint_applied_ns is not None
                                      and sample['arm_ns'] >= endpoint_applied_ns)
                    applied_seq = self.arm.applied_seq
                if returning:
                    at_target = (authority_ready and fraction == 1.
                                 and max(abs(a-b) for a, b in zip(q[5:], leg_end)) <= target_tolerance
                                 and max(map(abs, dq[5:])) <= .03
                                 and max(map(abs, dq)) <= .02 and
                                 max(abs(a-b) for a,b in zip(q[5:], initial[5:])) <= target_tolerance)
                    if elapsed > duration + 8:
                        raise RuntimeError('directional_return_not_reached')
                    if sample['arm_ns'] > last_sample_ns:
                        if not at_target:
                            settled_since = hold_since = None
                        elif settled_since is None:
                            settled_since = sample['arm_ns']
                        elif sample['arm_ns']-settled_since >= 200_000_000 and hold_since is None:
                            hold_since = sample['arm_ns']
                        last_sample_ns = sample['arm_ns']
                        if hold_since is not None:
                            break
                elif sample['arm_ns'] > last_sample_ns:
                    # Pointing constrains the observed hand axis, not a unique
                    # five-joint IK configuration. Every out-of-tolerance frame
                    # resets this continuous hold; it is not an immediate abort.
                    measured = self.kinematics.evaluate(q[5:], goal_direction)
                    angular_error = measured['angular_error_deg']
                    within_direction = angular_error <= angular_tolerance
                    eligible = authority_ready and endpoint_ready and within_direction
                    if eligible:
                        if hold_since is None:
                            hold_since = sample['arm_ns']
                        held_s = (sample['arm_ns']-hold_since)/1e9
                        longest_hold_s = max(longest_hold_s, held_s)
                    else:
                        hold_since = None
                        held_s = 0.
                    joint_error = [actual-requested for actual,requested in zip(q[5:], target)]
                    arrival_diagnostics = {
                        'criterion': 'measured_direction', 'target_direction': list(goal_direction),
                        'angular_error_deg': angular_error, 'tolerance_deg': angular_tolerance,
                        'within_tolerance': within_direction, 'reference_at_target': endpoint_ready,
                        'trajectory_at_end': fraction == 1., 'endpoint_sequence': endpoint_seq,
                        'applied_sequence': applied_seq, 'endpoint_applied_ns': endpoint_applied_ns,
                        'authority_ready': authority_ready, 'required_hold_s': hold_s,
                        'hold_elapsed_s': held_s, 'longest_hold_s': longest_hold_s,
                        'hold_completed': False, 'hold_start_feedback_ns': hold_since,
                        'joint_error_rad': joint_error, 'max_abs_joint_error_rad': max(map(abs, joint_error)),
                        'joint_velocity_rad_s': list(dq[5:]),
                        'feedback_ns': sample['arm_ns'], 'feedback_age_ns': self.arm.clock()-sample['arm_ns'],
                    }
                    observe('holding' if eligible else 'moving', list(q[5:]))
                    startup_complete = True
                    last_sample_ns = sample['arm_ns']
                    if elapsed > duration + 8 + hold_s:
                        raise RuntimeError('directional_target_not_reached')
                    if eligible and held_s >= hold_s:
                        arrival_diagnostics['hold_completed'] = True
                        if not return_to_start:
                            break
                        # Retrace the original command path and preserve the
                        # verified outgoing hold evidence through handback.
                        returning = True
                        leg_start, leg_end = target, initial[5:]
                        began = now
                        settled_since = hold_since = None
                        observe('returning', None)
                        continue
                if stop_event.is_set():
                    raise RuntimeError('cancelled')
                seq += 1
                with self.arm.lock:
                    self._submit(lease, seq, reference, sample, bounds)
                    if not returning and fraction == 1. and endpoint_seq is None:
                        endpoint_seq = seq
                stop_event.wait(.02)
        except Exception as exc:
            error = exc
            if not startup_complete:
                startup_failure = self._startup_evidence(lease, startup_phase, exc)
            if lease is not None:
                # Copy the rejected tick before handback can retire this lease.
                with self.arm.lock:
                    failure = getattr(self.arm, '_directional_envelope_failure', None)
                    if (isinstance(failure, dict) and failure.get('session_id') == lease['session_id']
                            and (str(exc) == 'motion_envelope_exceeded'
                                 or self.arm.reason == 'motion_envelope_exceeded')):
                        envelope_failure = {k: v for k, v in failure.items() if k != 'session_id'}
        finally:
            try:
                if lease is not None:
                    try:
                        observe('releasing', None)
                    except Exception as observer_error:
                        if error is None or str(error) == 'cancelled':
                            error = RuntimeError('directional_observer:' + str(observer_error))
                    try:
                        self._release(lease)
                        with self.arm.lock:
                            final_sample, final_q, _ = self.arm._fresh()
                            record_left(final_sample, final_q, 'released')
                    except Exception as release_error:
                        release_failure = self._release_evidence(lease, release_error)
                        # Preserve both facts; cancellation is not success if
                        # ownership or physical rest could not be confirmed.
                        error = RuntimeError(str(release_error) + (('; cause=' + str(error)) if error else ''))
            finally:
                self._run_lock.release()
        if error is None and stop_event.is_set():
            error = RuntimeError('cancelled')
        if error is not None:
            failure = DirectionalEnvelopeError(str(error), envelope_failure)
            failure.left_hold = left_hold
            failure.tracking_diagnostics = tracking_diagnostics
            failure.arrival_diagnostics = arrival_diagnostics
            failure.startup_failure = startup_failure
            failure.release_failure = release_failure
            raise failure from error
        return {'left_hold': left_hold, 'tracking_diagnostics': tracking_diagnostics,
                'arrival_diagnostics': arrival_diagnostics}
