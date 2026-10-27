"""Explicit, bounded private-pipe client for the vendor-domain motion worker.

No child process or ROS import occurs at construction. Commands never start or
restart a worker implicitly. Failure/cancellation permanently closes this
session; late responses cannot be accepted by a later action. There is no
motion retry, automatic heartbeat, SDK mode change, or stop-on-close claim.
"""
from __future__ import annotations

import json
import copy
import math
import os
import select
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    from .bridge import reader_environment
    from .motion_worker import MAX_REQUEST_BYTES, MAX_RESPONSE_BYTES, RPC_BUDGET_S, strict_json
    from .motion_dds import (OPERATOR_PERMIT_SCHEMAS, SUPERVISED_POSTURE_ACTIONS,
        SUPERVISED_DISTANCE_ACTIONS, AuthorityUnavailable, CommandChannelNotReady,
        validate_operator_permit)
except ImportError:
    from bridge import reader_environment
    from motion_worker import MAX_REQUEST_BYTES, MAX_RESPONSE_BYTES, RPC_BUDGET_S, strict_json
    from motion_dds import (OPERATOR_PERMIT_SCHEMAS, SUPERVISED_POSTURE_ACTIONS,
        SUPERVISED_DISTANCE_ACTIONS, AuthorityUnavailable, CommandChannelNotReady,
        validate_operator_permit)


class MotionProcessDisabled(RuntimeError):
    pass


class MotionProcessClosed(RuntimeError):
    pass


class MotionProcessNotStarted(RuntimeError):
    pass


class MotionWorkerRejected(RuntimeError):
    pass


class MotionProcessCancelled(RuntimeError):
    pass


class DDSProcessTransport:
    def __init__(self, config, config_path=None, *, execution_enabled=False, supervised_posture=False,
                 rpc_budget_s=RPC_BUDGET_S,
                 process_factory=subprocess.Popen, clock=time.monotonic):
        if not isinstance(execution_enabled, bool):
            raise ValueError('execution_enabled must be boolean')
        if type(supervised_posture) is not bool:
            raise ValueError('supervised_posture must be boolean')
        reader = config.get('reader', {})
        if int(reader.get('domain_id', 0)) != 0:
            raise ValueError('motion vendor domain must be 0')
        if type(rpc_budget_s) not in (int, float) or not math.isfinite(rpc_budget_s) or not .01 <= rpc_budget_s <= RPC_BUDGET_S:
            raise ValueError('rpc_budget_s must be within .01..0.2 seconds')
        self._rpc_budget = float(rpc_budget_s)
        self._config = copy.deepcopy(config)
        self.config_path = str(config_path) if config_path is not None else None
        self._enabled = execution_enabled
        self._supervised_posture = supervised_posture
        self._factory, self._clock = process_factory, clock
        self._state_lock, self._io_lock = threading.Lock(), threading.Lock()
        self._process = None
        self._closed = False
        self._ready = False
        self._probe_only = False
        self._buffer = bytearray()
        self._next_id = 1

    def _state(self):
        with self._state_lock:
            if self._closed:
                raise MotionProcessClosed('private motion session is closed')
            process = self._process
            if process is None or not self._ready:
                raise MotionProcessNotStarted('explicit start is required')
            return process

    def status(self):
        with self._state_lock:
            return {'started': self._ready, 'closed': self._closed,
                    'execution_enabled': self._enabled and not self._probe_only,
                    'supervised_posture': self._supervised_posture,
                    'worker_running': self._process is not None and self._process.poll() is None}

    def _abort(self):
        with self._state_lock:
            self._closed, self._ready = True, False
            process, self._process = self._process, None
        if process is not None:
            # A timeout/cancel may race a worker type import. Kill the private
            # process before returning; it cannot begin a later publish. A
            # command already handed to DDS is never claimed to be retractable.
            if process.poll() is None:
                process.kill()
                try:
                    process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    pass
            for stream in (process.stdin, process.stdout):
                if stream is not None:
                    try:
                        stream.close()
                    except OSError:
                        pass

    def close(self):
        self._abort()

    def _check_cancel(self, event):
        if event is not None and event.is_set():
            raise MotionProcessCancelled('private motion request was cancelled')

    def _read(self, process, deadline, cancel_event=None):
        while True:
            self._check_cancel(cancel_event)
            with self._state_lock:
                if self._closed:
                    raise MotionProcessClosed('private motion session is closed')
            if b'\n' in self._buffer:
                raw, _, rest = self._buffer.partition(b'\n')
                self._buffer = bytearray(rest)
                if not raw or len(raw) > MAX_RESPONSE_BYTES:
                    raise ValueError('invalid private response size')
                return strict_json(raw)
            remaining = deadline - self._clock()
            if remaining <= 0:
                raise TimeoutError('private motion response deadline exceeded')
            ready, _, _ = select.select([process.stdout.fileno()], [], [], min(.01, remaining))
            if ready:
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk:
                    raise ConnectionError('private motion worker exited')
                self._buffer.extend(chunk)
                if len(self._buffer) > MAX_RESPONSE_BYTES:
                    raise ValueError('private response exceeds bound')

    def start(self, *, probe_only=False, timeout_s=5.):
        if not isinstance(probe_only, bool):
            raise ValueError('probe_only must be boolean')
        if self._supervised_posture and probe_only:
            raise ValueError('supervised posture and read-only probe modes are distinct')
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or not .05 <= timeout_s <= 5:
            raise ValueError('startup timeout must be in .05..5 seconds')
        if not self._enabled and not probe_only:
            raise MotionProcessDisabled('motion worker execution is disabled')
        deadline = self._clock() + timeout_s
        if not self._io_lock.acquire(timeout=timeout_s):
            raise TimeoutError('private motion startup I/O is busy')
        try:
            with self._state_lock:
                if self._closed:
                    raise MotionProcessClosed('private motion session is closed')
                if self._ready:
                    if probe_only != self._probe_only:
                        raise ValueError('cannot change private worker execution mode')
                    return self.status_unlocked()
                command = [sys.executable, '-u', str(Path(__file__).with_name('motion_worker.py'))]
                if self._enabled and not probe_only:
                    command.append('--allow-supervised-posture' if self._supervised_posture else '--allow-execution')
                process = self._factory(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL, env=reader_environment(self._config), start_new_session=True, bufsize=0)
                self._process, self._probe_only = process, probe_only
            try:
                os.set_blocking(process.stdin.fileno(), False)
                ready = self._read(process, deadline)
                if ready != {'ready': True}:
                    raise ValueError('invalid private motion startup acknowledgement')
                with self._state_lock:
                    if self._closed:
                        raise MotionProcessClosed('startup cancelled')
                    self._ready = True
                return self.status()
            except BaseException:
                self._abort()
                raise
        finally:
            self._io_lock.release()

    def status_unlocked(self):
        return {'started': self._ready, 'closed': self._closed,
                'execution_enabled': self._enabled and not self._probe_only,
                'supervised_posture': self._supervised_posture,
                'worker_running': self._process is not None and self._process.poll() is None}

    def _rpc(self, operation, payload, *, authority=None, cancel_event=None):
        deadline = self._clock() + self._rpc_budget
        if self._supervised_posture and operation != 'graph':
            permit = validate_operator_permit(authority, now=self._clock())
            deadline = min(deadline, permit['action_deadline_monotonic'])
            if permit['expires_monotonic'] is not None:
                deadline = min(deadline, permit['expires_monotonic'])
        self._check_cancel(cancel_event)
        while not self._io_lock.acquire(timeout=.01):
            self._check_cancel(cancel_event)
            if self._clock() >= deadline:
                # Do not abort a different in-flight request merely because a
                # concurrent call could not enter this session's serial I/O.
                raise TimeoutError('private motion I/O is busy')
        process = None
        try:
            process = self._state()
            self._check_cancel(cancel_event)
            request_id = self._next_id
            self._next_id += 1
            request = {'id': request_id, 'op': operation, 'payload': payload,
                       'authority': authority, 'deadline_monotonic': deadline}
            raw = json.dumps(request, allow_nan=False, separators=(',', ':')).encode() + b'\n'
            if len(raw) > MAX_REQUEST_BYTES:
                raise ValueError('private request exceeds bound')
            offset = 0
            while offset < len(raw):
                self._check_cancel(cancel_event)
                remaining = deadline - self._clock()
                if remaining <= 0:
                    raise TimeoutError('private motion write deadline exceeded')
                _, writable, _ = select.select([], [process.stdin.fileno()], [], min(.01, remaining))
                if writable:
                    try:
                        offset += os.write(process.stdin.fileno(), raw[offset:])
                    except BlockingIOError:
                        pass
            reply = self._read(process, deadline, cancel_event)
            if self._supervised_posture and self._clock() >= deadline:
                raise TimeoutError('supervised posture response exceeded its bounded deadline')
            if (not isinstance(reply, dict) or type(reply.get('id')) is not int
                    or reply.get('id') != request_id or type(reply.get('ok')) is not bool):
                raise ValueError('private response does not match the active request')
            if not reply['ok']:
                if reply.get('error_type') == 'CommandChannelNotReady':
                    raise CommandChannelNotReady('private worker has no matched command receiver')
                raise MotionWorkerRejected('private motion worker rejected the request')
            if set(reply) != {'id', 'ok', 'result'} or not isinstance(reply['result'], dict):
                raise ValueError('invalid private response shape')
            return reply['result']
        except BaseException:
            if process is not None:
                self._abort()
            raise
        finally:
            self._io_lock.release()

    def publish_state(self, state, *, authority, cancel_event=None):
        if not self._enabled or self._probe_only:
            raise MotionProcessDisabled('motion worker execution is disabled')
        if self._supervised_posture:
            permit = validate_operator_permit(authority, now=self._clock())
            if (permit['action'] not in SUPERVISED_POSTURE_ACTIONS or type(state) is not int
                    or state != {'stand': 1, 'lie': 4, 'move_mode': 17}[permit['action']]):
                raise AuthorityUnavailable('supervised posture action does not match its state command')
        elif (isinstance(authority, dict) and isinstance(authority.get('schema'), str)
              and authority['schema'] in OPERATOR_PERMIT_SCHEMAS):
            raise AuthorityUnavailable('operator permits cannot enter the normal vendor-authority worker')
        return self._rpc('state', {'state': state}, authority=authority, cancel_event=cancel_event)

    def publish_velocity(self, vx=0., vy=0., yaw_rate=0., *, authority, cancel_event=None):
        if not self._enabled or self._probe_only:
            raise MotionProcessDisabled('motion worker execution is disabled')
        if self._supervised_posture:
            permit = validate_operator_permit(authority, now=self._clock())
            if permit['action'] not in SUPERVISED_DISTANCE_ACTIONS:
                raise AuthorityUnavailable('operator scope does not authorize fixed-distance NAV_CMD')
        elif (isinstance(authority, dict) and isinstance(authority.get('schema'), str)
              and authority['schema'] in OPERATOR_PERMIT_SCHEMAS):
            raise AuthorityUnavailable('operator permits cannot enter the normal vendor-authority worker')
        return self._rpc('velocity', {'vx': vx, 'vy': vy, 'yaw_rate': yaw_rate}, authority=authority, cancel_event=cancel_event)

    def graph_snapshot(self):
        return self._rpc('graph', {})
