"""Bounded private process for a separately verified posture interruption adapter.

No child, factory import, ROS participant or control endpoint is created by the
constructor. A deployment-selected factory must construct a command-free adapter
with ``send`` and read-only ``feedback`` methods. Only explicit ``prepare`` starts
it. Failure/cancellation permanently closes this session; commands are never
replayed and a killed worker is never restarted implicitly.
"""
from __future__ import annotations

import argparse
import copy
import importlib
import json
import math
import os
from pathlib import Path
import re
import select
import signal
import subprocess
import sys
import threading
import time

try:
    from .bridge import reader_environment
except ImportError:
    from bridge import reader_environment

MAX_BYTES = 65536
REFERENCE = re.compile(r'[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*:[A-Za-z_][A-Za-z0-9_]*\Z')


class PostureProcessUnavailable(RuntimeError):
    pass


def _timeout(value, maximum, name, minimum=.001):
    if type(value) not in (int, float) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(name + ' is outside its bounded range')
    return float(value)


def _json(raw):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate private field')
            result[key] = value
        return result
    return json.loads(raw, object_pairs_hook=pairs,
        parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite private field')))


def _encode(value):
    raw = json.dumps(value, allow_nan=False, separators=(',', ':')).encode() + b'\n'
    if len(raw) > MAX_BYTES:
        raise ValueError('private posture record exceeds bound')
    return raw


class BoundedPostureProcess:
    """Private adapter session; creating this object performs no I/O."""
    bounded_private_process = True

    def __init__(self, reference, config, *, send_timeout_s=.1, feedback_timeout_s=.05,
                 process_factory=subprocess.Popen, clock=time.monotonic):
        if not isinstance(reference, str) or not REFERENCE.fullmatch(reference):
            raise ValueError('invalid locally installed posture adapter reference')
        if not isinstance(config, dict):
            raise ValueError('posture adapter configuration must be a mapping')
        self._reference, self._config = reference, copy.deepcopy(config)
        self.send_timeout_s = _timeout(send_timeout_s, .2, 'send_timeout_s', .01)
        self.feedback_timeout_s = _timeout(feedback_timeout_s, .05, 'feedback_timeout_s')
        self._factory, self._clock = process_factory, clock
        self._lock, self._state_lock = threading.Lock(), threading.Lock()
        self._process, self._buffer = None, bytearray()
        self._closed = self._prepared = False
        self._next_id = 1

    def status(self):
        with self._state_lock:
            return {'prepared': self._prepared and not self._closed,
                'closed': self._closed, 'bounded_private_process': True,
                'worker_running': self._process is not None and self._process.poll() is None,
                'send_timeout_s': self.send_timeout_s, 'feedback_timeout_s': self.feedback_timeout_s}

    def _abort(self):
        with self._state_lock:
            self._closed, self._prepared = True, False
            process, self._process = self._process, None
        if process is None:
            return
        try:
            # Popen creates a dedicated session. Reap adapter-owned children
            # even if their worker already exited, preventing a late callback.
            os.killpg(process.pid, signal.SIGKILL)
        except (OSError, AttributeError):
            if process.poll() is None:
                process.kill()
        if process.poll() is None:
            try:
                process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=.5)
        for stream in (process.stdin, process.stdout):
            if stream is not None:
                try:
                    stream.close()
                except OSError:
                    pass

    def close(self):
        self._abort()

    def _cancel(self, event, deadline):
        if event is not None and event.is_set():
            raise InterruptedError('private posture operation cancelled')
        if self._clock() >= deadline:
            raise TimeoutError('private posture operation exceeded deadline')
        with self._state_lock:
            if self._closed:
                raise PostureProcessUnavailable('private posture session is closed')

    def _exchange(self, process, request, deadline, cancel_event=None):
        raw = _encode(request)
        offset = 0
        while offset < len(raw):
            self._cancel(cancel_event, deadline)
            _, writable, _ = select.select([], [process.stdin.fileno()], [], min(.005, max(0, deadline-self._clock())))
            if writable:
                try:
                    offset += os.write(process.stdin.fileno(), raw[offset:])
                except BlockingIOError:
                    pass
        while True:
            self._cancel(cancel_event, deadline)
            if b'\n' in self._buffer:
                raw, _, remainder = self._buffer.partition(b'\n')
                self._buffer = bytearray(remainder)
                if not raw or len(raw) > MAX_BYTES:
                    raise ValueError('invalid private posture response')
                reply = _json(raw)
                if (not isinstance(reply, dict) or set(reply) != {'id', 'ok', 'result'}
                        or type(reply.get('id')) is not int or reply['id'] != request['id']
                        or reply.get('ok') is not True or not isinstance(reply['result'], dict)):
                    raise PostureProcessUnavailable('private posture adapter rejected operation')
                return reply['result']
            readable, _, _ = select.select([process.stdout.fileno()], [], [], min(.005, max(0, deadline-self._clock())))
            if readable:
                chunk = os.read(process.stdout.fileno(), 4096)
                if not chunk:
                    raise PostureProcessUnavailable('private posture worker exited')
                self._buffer.extend(chunk)
                if len(self._buffer) > MAX_BYTES:
                    raise ValueError('private posture response exceeds bound')

    def prepare(self, *, timeout_s=5.):
        timeout_s = _timeout(timeout_s, 5., 'startup timeout', .05)
        if not self._lock.acquire(timeout=timeout_s):
            raise TimeoutError('private posture startup is busy')
        try:
            with self._state_lock:
                if self._closed:
                    raise PostureProcessUnavailable('private posture session is closed')
                if self._prepared:
                    return self.status_unlocked()
                command = [sys.executable, '-u', str(Path(__file__).resolve()), '--worker', self._reference]
                self._process = self._factory(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL, env=reader_environment(self._config),
                    start_new_session=True, bufsize=0)
                process = self._process
            try:
                os.set_blocking(process.stdin.fileno(), False)
                deadline = self._clock() + timeout_s
                ready = self._exchange(process, {'id': 0, 'op': 'initialize', 'payload': self._config}, deadline)
                if ready != {'ready': True, 'command_callbacks_invoked': 0}:
                    raise ValueError('invalid private posture startup acknowledgement')
                with self._state_lock:
                    self._prepared = True
                return self.status()
            except BaseException:
                self._abort()
                raise
        finally:
            self._lock.release()

    def status_unlocked(self):
        return {'prepared': self._prepared, 'closed': self._closed, 'bounded_private_process': True,
                'worker_running': self._process is not None and self._process.poll() is None}

    def _rpc(self, op, payload, timeout_s, cancel_event=None):
        deadline = self._clock() + timeout_s
        while not self._lock.acquire(timeout=min(.005, max(.001, deadline-self._clock()))):
            self._cancel(cancel_event, deadline)
        process = None
        try:
            self._cancel(cancel_event, deadline)
            with self._state_lock:
                if not self._prepared or self._process is None:
                    raise PostureProcessUnavailable('explicit posture preparation is required')
                process = self._process
            wire_id = self._next_id
            self._next_id += 1
            return self._exchange(process, {'id': wire_id, 'op': op, 'payload': payload,
                'deadline_monotonic': deadline}, deadline, cancel_event)
        except BaseException:
            if process is not None:
                self._abort()
            raise
        finally:
            self._lock.release()

    def send(self, request_id, action, reason, authority, cancel_event):
        _send_arguments(request_id, action, reason, authority)
        budget = min(self.send_timeout_s, authority['expires_monotonic'] - self._clock())
        return self._rpc('send', {'request_id': request_id, 'action': action, 'reason': reason,
            'authority': authority}, budget, cancel_event)

    def feedback(self, request_id, action):
        _request_arguments(request_id, action)
        return self._rpc('feedback', {'request_id': request_id, 'action': action}, self.feedback_timeout_s)


class _DeadlineCancellation:
    def __init__(self, deadline):
        self.deadline = deadline

    def is_set(self):
        return time.monotonic() >= self.deadline


def _request_arguments(request_id, action):
    if (not isinstance(request_id, str) or not request_id.strip() or len(request_id) > 128
            or action not in ('stand', 'lie')):
        raise ValueError('invalid private posture request identity')


def _send_arguments(request_id, action, reason, authority):
    _request_arguments(request_id, action)
    if not isinstance(reason, str) or not reason or len(reason) > 128:
        raise ValueError('invalid private posture stop reason')
    try:
        from .motion_dds import validate_authority
    except ImportError:
        from motion_dds import validate_authority
    validate_authority(authority, now=time.monotonic())
    if (authority.get('request_id') != request_id or authority.get('regular_mode_confirmed') is not True
            or not isinstance(authority.get('allowed_actions'), list) or action not in authority['allowed_actions']):
        raise PermissionError('private posture authority does not cover this action')


def _read_record(stream):
    line = stream.readline(MAX_BYTES + 1)
    if not line or len(line) > MAX_BYTES or not line.endswith(b'\n'):
        raise ValueError('invalid private posture request')
    value = _json(line)
    if not isinstance(value, dict):
        raise ValueError('invalid private posture request')
    return value


def _worker(reference):
    init = _read_record(sys.stdin.buffer)
    if set(init) != {'id', 'op', 'payload'} or init['id'] != 0 or init['op'] != 'initialize' or not isinstance(init['payload'], dict):
        raise ValueError('invalid private posture initialization')
    module, factory_name = reference.split(':', 1)
    factory = getattr(importlib.import_module(module), factory_name)
    adapter = factory(copy.deepcopy(init['payload']))
    if not callable(getattr(adapter, 'send', None)) or not callable(getattr(adapter, 'feedback', None)):
        raise ValueError('invalid private posture adapter')
    sys.stdout.buffer.write(_encode({'id': 0, 'ok': True, 'result': {'ready': True, 'command_callbacks_invoked': 0}}))
    sys.stdout.buffer.flush()
    last_id = 0
    while True:
        request = _read_record(sys.stdin.buffer)
        wire_id, op, payload = request.get('id'), request.get('op'), request.get('payload')
        if (set(request) != {'id', 'op', 'payload', 'deadline_monotonic'}
                or type(wire_id) is not int or wire_id <= last_id or not isinstance(payload, dict)):
            raise ValueError('invalid private posture operation')
        last_id = wire_id
        deadline, now = request['deadline_monotonic'], time.monotonic()
        if type(deadline) not in (int, float) or not math.isfinite(deadline) or not now < deadline <= now+.2:
            raise TimeoutError('private posture operation expired')
        if op == 'send' and set(payload) == {'request_id', 'action', 'reason', 'authority'}:
            _send_arguments(payload['request_id'], payload['action'], payload['reason'], payload['authority'])
            result = adapter.send(payload['request_id'], payload['action'], payload['reason'],
                                  payload['authority'], _DeadlineCancellation(deadline))
        elif op == 'feedback' and set(payload) == {'request_id', 'action'}:
            _request_arguments(payload['request_id'], payload['action'])
            result = adapter.feedback(payload['request_id'], payload['action'])
        else:
            raise ValueError('unknown private posture operation')
        if time.monotonic() >= deadline or not isinstance(result, dict):
            raise TimeoutError('private posture callback exceeded deadline')
        sys.stdout.buffer.write(_encode({'id': wire_id, 'ok': True, 'result': result}))
        sys.stdout.buffer.flush()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worker', required=True)
    args = parser.parse_args(argv)
    if not REFERENCE.fullmatch(args.worker):
        raise ValueError('invalid private posture factory')
    _worker(args.worker)


if __name__ == '__main__':
    main()
