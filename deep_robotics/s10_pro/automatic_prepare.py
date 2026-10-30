"""Prepare idle, healthy drivers under explicit machine-bound startup policy.

This supervisor can only start read-only observation and call prepare. It never
creates a motion task, selects a posture, publishes velocity, or replays a task.
"""
from __future__ import annotations

import copy
import logging
import os
import threading
import time


class AutomaticPreparation:
    def __init__(self, plugin, *, interval_s=5., clock=time.monotonic):
        self.plugin = plugin
        self.authorization = plugin._backend.authorization
        self.interval_s, self._clock = interval_s, clock
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._failure_latched = False
        self._startup_observation_started = False
        self._last_logged_status = None
        self._status = {'enabled': True, 'state': 'waiting', 'blocked_by': [],
                        'attempts': 0, 'sessions_prepared': 0,
                        'last_session_id': None, 'physical_command_sent': False}
        plugin._automatic_preparation = self

    def policy_enabled(self):
        check = getattr(self.authorization, 'automatic_policy_enabled', None)
        return callable(check) and check() is True

    def status(self):
        with self._lock:
            return copy.deepcopy(self._status)

    def _record(self, state, blocked=(), **fields):
        with self._lock:
            self._status.update(state=state, blocked_by=list(blocked), **fields)
            # A periodic retry transitions through preparing every five seconds.
            # Log the meaningful outcome only when it changes, not every poll.
            outcome = (state, tuple(blocked))
            changed = state != 'preparing' and outcome != self._last_logged_status
            if changed:
                self._last_logged_status = outcome
        if changed:
            logging.info('Automatic prepare: %s%s', state,
                         ' — ' + ', '.join(blocked) if blocked else '')

    def start(self):
        if self._thread is not None or not self.policy_enabled():
            return
        if self.plugin._closed.is_set():
            self._record('stopped')
            return
        # This startup exception belongs only to the explicitly enabled driver
        # policy. A later user/card stop is respected and never auto-restarted.
        with self.plugin._lock:
            if self.plugin._settings['enabled'] and not self.plugin._closed.is_set():
                self.plugin.start()
                self._startup_observation_started = True
        self._thread = threading.Thread(target=self._run, daemon=True, name='s10-automatic-prepare')
        self._thread.start()

    def tick(self):
        with self.plugin._lock:
            if self._stop.is_set() or self.plugin._closed.is_set():
                self._record('stopped')
                return
            if not self.policy_enabled():
                self._record('disabled', ['automatic_prepare_policy_unavailable'], enabled=False)
                return
            if (not self._startup_observation_started and self.plugin._settings['enabled']):
                self.plugin.start()
                self._startup_observation_started = True
            if not self.plugin._started or not self.plugin._settings['enabled']:
                self._record('paused', ['card_not_enabled'])
                return
            if self.plugin._active is not None or (
                    self.plugin._worker is not None and self.plugin._worker.is_alive()):
                self._record('busy', ['active_motion_task_in_progress'])
                return
            # A latched command failure needs review or an explicit new action;
            # the supervisor must not repeatedly restart a failed worker.
            if self.plugin._error or self.plugin._backend.safety_status().get('failed_closed'):
                self._record('needs_review', ['driver_error_requires_review'])
                return
            if self.authorization.authorized_now():
                self._failure_latched = False
                self._record('session_active')
                return
            if self._failure_latched:
                self._record('needs_review', ['automatic_prepare_failed'])
                return
            if not self.authorization.automatic_prepare_allowed():
                self._record('explicit_prepare_required', ['automatic_prepare_not_authorized'])
                return
            before = self.status()
            self._record('preparing', attempts=before['attempts'] + 1,
                         last_attempt_monotonic=self._clock())
            try:
                result = self.plugin.dispatch('prepare_automatic', {})
            except Exception:
                # Do not reset errors, change limits, or replay a previous task.
                self._failure_latched = True
                self._record('needs_review', ['automatic_prepare_failed'])
                return
            if result.get('prepared') and self.authorization.authorized_now():
                session = self.authorization.public_status()
                self._record('session_active', sessions_prepared=before['sessions_prepared'] + 1,
                             last_session_id=session.get('session_id'))
            else:
                blocked = result.get('blocked_by', ['prepare_not_ready'])
                if any('requires_review' in reason for reason in blocked):
                    self._failure_latched = True
                self._record('needs_review' if self._failure_latched else 'waiting', blocked)

    def _run(self):
        while not self._stop.is_set():
            try:
                self.tick()
            except Exception:
                self._failure_latched = True
                self._record('needs_review', ['automatic_prepare_failed'])
            self._stop.wait(self.interval_s)

    def close(self):
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=10.)
        self._record('stopped')


def start_automatic_preparation(plugins):
    """Deployment opt-in; protected authorization scope and preflight stay intact."""
    if os.environ.get('S10_AUTO_PREPARE', '1') != '1':
        return None
    for plugin in plugins:
        authorization = getattr(getattr(plugin, '_backend', None), 'authorization', None)
        if getattr(authorization, 'supports_prepare', False):
            supervisor = AutomaticPreparation(plugin)
            supervisor.start()
            return supervisor
    logging.warning('Automatic prepare unavailable: valid prepare policy required')
    return None
