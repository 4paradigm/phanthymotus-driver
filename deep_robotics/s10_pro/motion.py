"""Basic motion lifecycle with an explicit, fail-closed execution boundary.

No SDK or transport is constructed here. The default backend cannot send. A
real backend requires robot-specific evidence for authority, feedback and the
robot-side loss-of-command stop guarantee, plus process-level authorization.
"""
from __future__ import annotations

import copy
import math
import threading
import time
from collections import deque

try:
    from .parameters import (motion_defaults, validate_motion_settings, motion_config_schema,
        motion_action_schema, validate_motion_request, normalize_config_updates, validate_empty_arguments)
    from .motion_tasks import MotionTasks
except ImportError:
    from parameters import (motion_defaults, validate_motion_settings, motion_config_schema,
        motion_action_schema, validate_motion_request, normalize_config_updates, validate_empty_arguments)
    from motion_tasks import MotionTasks


ACTIONS = ('stand', 'lie', 'velocity', 'stopmotion')
REQUIRED_EVIDENCE = ('identity', 'firmware', 'authority', 'feedback', 'stop', 'watchdog')


class UnsupportedBackend:
    def contract(self):
        return {'verified': False, 'missing': list(REQUIRED_EVIDENCE)}

    def control_status(self):
        return {'owned': False, 'fresh': False, 'age_s': None}


def _number(value, minimum, maximum, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f'{name} must be finite and within {minimum}..{maximum}')
    return float(value)


def validate_contract(contract):
    """Evidence must describe this machine, rather than a generic verified flag."""
    if not isinstance(contract, dict) or contract.get('verified') is not True:
        return False
    evidence = contract.get('evidence')
    if not isinstance(evidence, dict) or any(not isinstance(evidence.get(k), str) or not evidence[k].strip() for k in REQUIRED_EVIDENCE):
        return False
    if contract.get('robot_identity_matched') is not True or contract.get('firmware_matched') is not True:
        return False
    if contract.get('robot_deadman_verified') is not True:
        return False
    if contract.get('velocity_units') != {'vx': 'm/s', 'vy': 'm/s', 'yaw_rate': 'rad/s'}:
        return False
    if not isinstance(contract.get('actions'), list) or not set(ACTIONS) <= set(contract['actions']):
        return False
    try:
        deadman = _number(contract.get('robot_deadman_s'), .1, 1., 'robot_deadman_s')
        send_timeout = _number(contract.get('send_timeout_s'), .01, .2, 'send_timeout_s')
    except ValueError:
        return False
    return (send_timeout * 2 < deadman and contract.get('stop_semantics') == 'zero_velocity_until_feedback'
        and isinstance(contract.get('stop_on_authority_loss'), bool))


class MotionPlugin:
    name = PREFIX = 'motion'

    def __init__(self, config, namespace, backend=None, *, feedback=None, publish=None,
                 execution_authorized=False, clock=time.monotonic, wall_time=time.time,
                 autostart_worker=True, notifier=None, tasks=None):
        if not isinstance(execution_authorized, bool):
            raise ValueError('execution_authorized must be boolean')
        self._backend = backend or UnsupportedBackend()
        self._feedback = feedback
        self._publish = publish or (lambda *args: None)
        self._authorized = execution_authorized
        self._notifier = notifier
        self._tasks = tasks if tasks is not None else MotionTasks()
        self._owns_tasks = tasks is None
        self._tasks.recover_interrupted(wall_time)
        self._worker = None
        self._shutdown_done = False
        self._cancel_reason = None
        self._clock, self._wall = clock, wall_time
        self._lock = threading.RLock()
        self._io_lock = threading.Lock()
        self._notification_lock = threading.Lock()
        self._busy = threading.Lock()
        self._cancel = threading.Event()
        self._closed = threading.Event()
        self._started = False
        self._phase = 'idle'
        self._active = None
        self._generation = 0
        self._last_command = clock()
        self._last = None
        self._events = deque(maxlen=64)
        self._error = None
        self._last_tick = clock()
        self._settings = self._validate(config or {})
        base = '/' + '/'.join(p for p in (str(namespace).strip('/'), 's10_pro') if p)
        self.status_topic, self.events_topic = base + '/motion_status', base + '/motion_events'
        self._thread = threading.Thread(target=self._monitor, name='s10-motion-watchdog', daemon=True)
        if autostart_worker:
            self._thread.start()

    @staticmethod
    def _validate(settings):
        return validate_motion_settings(settings)

    def _ports(self):
        return [{'topic': self.status_topic, 'format': 'data/json'}, {'topic': self.events_topic, 'format': 'data/json'}]

    def get_tool(self):
        with self._lock:
            schema = motion_action_schema(self._settings)
            schema['x-completion'] = {'actions': ['stand', 'lie', 'velocity'],
                'timeout': 90, 'silent_completion': False}
            schema['x-resource'] = 'base'
            return {'name': self.name, 'title': 'Basic Motion', 'type': 'actuator', 'multiInstance': False,
                'description': 'Stand, lie down, finite velocity and application stop. Physical execution remains locked. Accepted tasks complete through ACP; application stop is not a hardware emergency stop.',
                'inputSchema': schema, 'configSchema': motion_config_schema(self._settings),
                'topic_in': [], 'topic_out': self._ports()}

    def get_tools(self):
        return [self.get_tool()]

    def _contract(self):
        try:
            contract = self._backend.contract()
            return copy.deepcopy(contract) if isinstance(contract, dict) else {}
        except Exception:
            return {}

    def _gate(self, *, for_stop=False):
        missing = []
        if self._closed.is_set() and not for_stop:
            missing.append('controller_closed')
        if not self._authorized:
            missing.append('execution_authorization')
        if not validate_contract(self._contract()):
            missing.append('robot_control_contract')
        if self._feedback is None:
            missing.append('fresh_feedback_adapter')
        if self._notifier is None:
            missing.append('acp_notifier')
        return missing

    def _authority_ok(self):
        try:
            value = self._backend.control_status()
            age = value.get('age_s')
            return (value.get('owned') is True and value.get('fresh') is True
                and isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age) and 0 <= age <= .5)
        except Exception:
            return False

    def _control_ok(self):
        try:
            return self._authority_ok() and self._backend.control_status().get('feedback_fresh') is True
        except Exception:
            return False

    def _event(self, kind, **values):
        with self._lock:
            event = {'card': 'motion', 'event': kind, 'timestamp_ms': int(self._wall() * 1000), **values}
            self._events.append(event)
            if self._started:
                try:
                    self._publish(self.events_topic, 'data/json', copy.deepcopy(event))
                except Exception:
                    self._error = 'status_publish_failed'

    def status(self):
        with self._lock:
            missing = self._gate()
            diagnostics = {}
            if hasattr(self._backend, 'diagnostics'):
                try:
                    diagnostics = self._backend.diagnostics()
                except Exception:
                    diagnostics = {'preflight_error': 'observation_unavailable'}
            return {'card': 'motion', 'state': 'running' if self._started else 'idle', 'phase': self._phase,
                'execution_available': not missing and self._settings['enabled'], 'execution_authorized': self._authorized,
                'contract_verified': validate_contract(self._contract()), 'blocked_by': missing,
                'availability_detail': {
                    'protocol_implemented': 'S10 software guide V1.0.2',
                    'production_backend': 'unbound',
                    'missing_evidence': self._contract().get('missing', []),
                    'explanation': 'Robot version, control ownership and loss-of-command stopping remain unverified. Canvas settings cannot authorize execution.',
                    **diagnostics},
                'settings': dict(self._settings), 'active_request': copy.deepcopy(self._active),
                'last_result': copy.deepcopy(self._last), 'last_error': self._error,
                'completion_delivery': self._delivery_status(),
                'topic_in': [], 'topic_out': self._ports(), 'supported_actions': list(ACTIONS),
                'stop_semantics': 'application_stop_not_emergency_stop', 'events': list(self._events)}

    def start(self):
        with self._lock:
            if self._closed.is_set():
                return self.status()
            if hasattr(self._backend, 'start_observation'):
                self._backend.start_observation()
            self._started = True
            self._phase = 'locked' if self._gate() or not self._settings['enabled'] else 'ready'
        return self.status()

    def stop(self):
        with self._lock:
            self._started = False
            self._phase = 'idle'
        result = self.cancel('card_stopped')
        with self._lock:
            if self._active is None and hasattr(self._backend, 'stop_observation'):
                self._backend.stop_observation()
        return {**self.status(), 'cancellation': result}

    def close(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._closed.set()
        self.stop()
        worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(2)
        if hasattr(self._backend, 'close'):
            self._backend.close()
        if worker is not None and worker is not threading.current_thread() and worker.is_alive():
            worker.join(1)
        if self._thread.ident is not None and self._thread is not threading.current_thread():
            self._thread.join(1)
        self._flush_notifications()
        if self._owns_tasks and (worker is None or not worker.is_alive()):
            self._tasks.close()

    def configure(self, options):
        updates = normalize_config_updates(options, motion_defaults())
        with self._lock:
            updated = self._validate({**self._settings, **updates})
            active = self._active is not None
            self._generation += 1
            self._settings = updated
        if active:
            self.cancel('configuration_changed')
        return self.status()

    def _delivery_status(self):
        if self._notifier is None:
            return {'delivery_state': 'unavailable', 'acknowledged': False}
        task = self._active or self._last
        if task is None:
            return {'delivery_state': 'idle', 'acknowledged': False}
        try:
            return self._notifier.status(task['action_id'])
        except Exception:
            return {'delivery_state': 'status_unavailable', 'acknowledged': False}

    def _flush_notifications(self):
        if self._notifier is None or not self._notification_lock.acquire(blocking=False):
            return
        try:
            for record in self._tasks.pending_notifications():
                result = record['result']
                payload = {'action_id': record['action_id'], 'request_id': record['request_id'],
                    'status': result['status'], 'result': result, 'tool': 'motion', 'ts': result['finished_at']}
                try:
                    self._notifier.submit(payload)
                    self._tasks.mark_enqueued(record['request_id'])
                except Exception:
                    self._error = 'completion_enqueue_failed_retained_for_retry'
        finally:
            self._notification_lock.release()

    def _stop_backend(self, reason, *, request_id, authority_lost=False):
        if self._gate(for_stop=True):
            return {'accepted': False, 'confirmed': False, 'reason': 'unavailable'}
        contract = self._contract()
        if authority_lost and not contract['stop_on_authority_loss']:
            return {'accepted': None, 'confirmed': False, 'reason': 'authority_released'}
        if not self._io_lock.acquire(timeout=.25):
            return {'accepted': None, 'confirmed': False, 'reason': 'stop_transport_busy'}
        try:
            with self._lock:
                if self._active is None or self._active['request_id'] != request_id:
                    return {'accepted': False, 'confirmed': False, 'reason': 'request_replaced'}
            # Recheck ownership inside the transport critical section. Never
            # send a delayed stop over a later request or manual takeover.
            if not self._authority_ok() and not contract['stop_on_authority_loss']:
                return {'accepted': None, 'confirmed': False, 'reason': 'authority_released'}
            # An already-set movement cancellation token must not cancel its own stop.
            return self._backend.stop(reason, threading.Event())
        except Exception:
            return {'accepted': None, 'confirmed': False, 'reason': 'stop_transport_failed'}
        finally:
            self._io_lock.release()

    def cancel(self, reason='user_stop'):
        # This lock never surrounds transport I/O. The action worker owns stop
        # and the sole terminal commit; the caller never waits for feedback.
        with self._lock:
            active = copy.deepcopy(self._active)
            if active is None:
                return {'accepted': False, 'confirmed': False, 'completed': False,
                        'reason': 'no_application_motion'}
            first = not self._cancel.is_set()
            self._cancel.set()
            if first:
                self._cancel_reason = reason
                self._generation += 1
                self._event('cancel_requested', request_id=active['request_id'],
                    original_action_id=active['action_id'], reason=reason, stop_confirmed=False)
            return {'accepted': True, 'confirmed': False, 'completed': False,
                'state': 'cancellation_requested', 'request_id': active['request_id'],
                'original_action_id': active['action_id'], 'reason': self._cancel_reason,
                'stop_confirmed': False}

    def _confirmation(self, action, request_id, after):
        try:
            value = self._feedback(action, request_id)
            age = value.get('age_s')
            received = value.get('received_monotonic')
            now = self._clock()
            return (value.get('confirmed') is True and value.get('fresh') is True
                and isinstance(age, (int, float)) and not isinstance(age, bool) and math.isfinite(age) and 0 <= age <= .5
                and isinstance(received, (int, float)) and not isinstance(received, bool) and math.isfinite(received)
                and after <= received <= now and abs((now - received) - age) <= .1
                and value.get('request_id') == request_id)
        except Exception:
            return False

    @staticmethod
    def _receipt(record, replayed=False):
        if record['result'] is not None:
            # Returning action_id here would make Core register a fresh pending
            # after the original barrier has reaped it. Keep identity nested.
            result = copy.deepcopy(record['result'])
            result['original_action_id'] = result.pop('action_id')
            return {**result, 'replayed': True, 'terminal': True}
        return {'action_id': record['action_id'], 'request_id': record['request_id'],
                'action': record['request']['action'], 'accepted': True,
                'confirmed': False, 'completed': False, 'stop_confirmed': False,
                'state': 'accepted', 'status': 'running', 'replayed': replayed}

    def execute(self, action, arguments):
        if action not in ('stand', 'lie', 'velocity'):
            raise ValueError('Unsupported motion action')
        with self._lock:
            settings, generation = dict(self._settings), self._generation
            request = validate_motion_request(action, arguments, settings, self._wall, check_freshness=False)
            if self._closed.is_set():
                return {'accepted': False, 'confirmed': False, 'completed': False,
                        'state': 'unavailable', 'blocked_by': ['controller_closed']}
            previous = self._tasks.get(request['request_id'])
            if previous:
                if self._tasks.fingerprint(request) != previous['fingerprint']:
                    raise ValueError('request_id already belongs to a different motion target')
                return self._receipt(previous, replayed=True)
            # Only first acceptance is subject to request TTL. Replays keep the
            # original identity even when their original acceptance TTL elapsed.
            request = validate_motion_request(action, {**arguments, 'request_id': request['request_id']},
                                              settings, self._wall)
            missing = self._gate()
            if missing or not self._started or not settings['enabled']:
                return {'accepted': False, 'confirmed': False, 'completed': False, 'state': 'unavailable',
                        'blocked_by': missing + ([] if self._started and settings['enabled'] else ['card_not_enabled'])}
            if not self._control_ok():
                return {'accepted': False, 'confirmed': False, 'completed': False,
                        'state': 'unavailable', 'blocked_by': ['fresh_control_authority']}
            if not self._busy.acquire(blocking=False):
                return {'accepted': False, 'confirmed': False, 'completed': False, 'state': 'busy'}
            if generation != self._generation or not self._started or self._closed.is_set():
                self._busy.release()
                return {'accepted': False, 'confirmed': False, 'completed': False,
                        'state': 'cancelled', 'reason': 'lifecycle_changed'}
            try:
                record, replayed = self._tasks.accept(request)
            except Exception:
                self._busy.release()
                raise
            if replayed:
                self._busy.release()
                return self._receipt(record, replayed=True)
            started = self._clock()
            request['expires_monotonic'] = started + (request['duration'] if action == 'velocity' else settings['action_timeout_s'])
            self._cancel, self._cancel_reason = threading.Event(), None
            self._active = {**request, 'action_id': record['action_id'], 'started_monotonic': started}
            self._last_command, self._phase = started, 'active'
            receipt = self._receipt(record)
            self._worker = threading.Thread(target=self._run_action,
                args=(record, request, started), name='s10-action-' + record['action_id'], daemon=True)
            self._event('requested', request_id=request['request_id'], action_id=record['action_id'], action=action)
            try:
                self._worker.start()
            except Exception:
                result = {'request_id': request['request_id'], 'action_id': record['action_id'],
                    'action': action, 'status': 'error', 'state': 'error', 'accepted': True,
                    'confirmed': False, 'stop_confirmed': False, 'completed': False,
                    'reason': 'action_worker_start_failed', 'finished_at': self._wall()}
                self._tasks.finish(request['request_id'], result)
                self._last, self._active, self._phase = result, None, 'error'
                self._busy.release()
        self._flush_notifications()
        return receipt

    def _wait_for_stop(self, reason, request_id, interval):
        with self._lock:
            if self._active and self._active['request_id'] == request_id:
                self._phase = 'stopping'
        after = self._clock()
        receipt = self._stop_backend(reason, request_id=request_id, authority_lost=not self._authority_ok())
        deadline, next_send = self._clock() + 1., self._clock() + interval
        while self._clock() < deadline and receipt.get('reason') != 'authority_released':
            if self._confirmation('stopmotion', request_id, after):
                return True
            if self._clock() >= next_send:
                receipt = self._stop_backend(reason, request_id=request_id, authority_lost=not self._authority_ok())
                next_send = self._clock() + interval
            time.sleep(.02)
        return False

    def _run_action(self, record, request, started):
        action, rid = request['action'], request['request_id']
        deadline = request['expires_monotonic']
        receipt, confirmed, stop_confirmed = {'accepted': None}, False, False
        reason, stop_attempted = 'feedback_timeout', False
        interval = .1
        try:
            contract = self._contract()
            interval = min(.1, (contract['robot_deadman_s'] - contract['send_timeout_s']) / 3)
            next_send = started
            while self._clock() < deadline:
                if self._cancel.is_set():
                    reason = self._cancel_reason or 'cancelled'
                    break
                if not self._control_ok():
                    reason = 'authority_unavailable'
                    break
                if self._clock() >= next_send:
                    with self._io_lock:
                        if self._cancel.is_set() or not self._control_ok() or not self._started:
                            reason = self._cancel_reason or 'authority_unavailable'
                            break
                        if next_send == started and self._wall() * 1000 > request['issued_at_ms'] + request['ttl_ms']:
                            reason = 'stale_request'
                            break
                        if self._clock() >= deadline:
                            reason = 'action_deadline_expired'
                            break
                        receipt = self._backend.send(action, request, self._cancel)
                        self._last_command = self._clock()
                    if receipt.get('accepted') is False:
                        reason = 'rejected'
                        break
                    next_send = self._clock() + interval if action == 'velocity' else float('inf')
                    self._event('sent', request_id=rid, action_id=record['action_id'],
                                command_accepted=receipt.get('accepted'), confirmed=False)
                if self._confirmation(action, rid, started):
                    confirmed = True
                    if action != 'velocity':
                        reason = 'feedback_confirmed'
                        break
                if hasattr(self._backend, 'progress') and self._clock() < deadline:
                    with self._io_lock:
                        if not self._cancel.is_set() and self._control_ok() and self._clock() < deadline:
                            advanced = self._backend.progress(action, request, self._cancel)
                            if advanced is not None:
                                receipt = advanced
                                if receipt.get('accepted') is False:
                                    reason = 'rejected'
                                    break
                self._cancel.wait(.02)
            else:
                # Only a natural deadline exit completes a finite velocity.
                # A rejected send or lost authority can also advance time past
                # the deadline; retain its failure instead of hiding it.
                if action == 'velocity':
                    reason = 'duration_elapsed'
            if action == 'velocity' or not confirmed or self._cancel.is_set():
                stop_confirmed = self._wait_for_stop(reason, rid, interval)
                stop_attempted = True
        except Exception as exc:
            reason = 'execution_failed_' + type(exc).__name__
            confirmed = False
            try:
                stop_confirmed = self._wait_for_stop(reason, rid, interval)
            except Exception:
                stop_confirmed = False
            stop_attempted = True
        finally:
            # Cancel and completion compete at this one commit lock. A late
            # cancellation first gets a real stop attempt, never a fake success.
            while True:
                with self._lock:
                    need_stop = self._cancel.is_set() and not stop_attempted
                    if not need_stop:
                        cancelled = self._cancel.is_set()
                        if cancelled:
                            reason = self._cancel_reason or 'cancelled'
                        completed = (not cancelled and confirmed
                            and reason in ('feedback_confirmed', 'duration_elapsed')
                            and (action != 'velocity' or stop_confirmed))
                        status = ('completed' if completed else 'cancelled'
                                  if cancelled and stop_confirmed else 'error')
                        if (cancelled or action == 'velocity') and not stop_confirmed:
                            reason += '_stop_unconfirmed'
                        result = {'request_id': rid, 'action_id': record['action_id'], 'action': action,
                            'accepted': True, 'command_accepted': receipt.get('accepted'),
                            'confirmed': confirmed and not cancelled, 'stop_confirmed': stop_confirmed,
                            'completed': completed, 'state': status, 'status': status,
                            'reason': reason, 'finished_at': self._wall()}
                        try:
                            if self._tasks.finish(rid, result):
                                self._last = result
                                self._phase = status if self._started else 'idle'
                                self._event('finished', **result)
                        except Exception:
                            self._error = 'terminal_record_failed_restart_recovery_required'
                        self._active = None
                        break
                stop_confirmed = self._wait_for_stop(self._cancel_reason or 'cancelled', rid, interval)
                stop_attempted = True
            try:
                if hasattr(self._backend, 'finish'):
                    self._backend.finish(rid)
                if not self._started and hasattr(self._backend, 'stop_observation'):
                    self._backend.stop_observation()
            finally:
                self._busy.release()
            self._flush_notifications()

    def dispatch(self, action, args):
        if action in ('info', 'status'):
            validate_empty_arguments(args)
            return self.status()
        if action == 'start':
            updates = normalize_config_updates(args, motion_defaults())
            if updates:
                self.configure(updates)
            return self.start()
        if action == 'stop':
            validate_empty_arguments(args)
            return self.stop()
        if action == 'config':
            return self.configure(args)
        if action == 'stopmotion':
            validate_motion_request(action, args, self._settings, self._wall)
            return self.cancel()
        return self.execute(action, args)

    def _monitor(self):
        while not self._closed.wait(.05):
            self._flush_notifications()
            now = self._clock()
            with self._lock:
                active = self._active is not None
                started = self._started
                overdue = now - self._last_tick > .5
                if active and self._active['action'] == 'velocity' and self._phase == 'active':
                    overdue = overdue or now - self._last_command > self._contract().get('robot_deadman_s', .3) / 2
                self._last_tick = now
            if active and (overdue or not self._control_ok()):
                self.cancel('watchdog_late' if overdue else 'authority_unavailable')
            if started and now >= getattr(self, '_next_status', 0):
                with self._lock:
                    if self._started:
                        try:
                            payload = self.status()
                            payload['timestamp_ms'] = int(self._wall() * 1000)
                            self._publish(self.status_topic, 'data/json', payload)
                        except Exception:
                            self._error = 'status_publish_failed'
                        self._next_status = now + 1. / self._settings['refresh_hz']
