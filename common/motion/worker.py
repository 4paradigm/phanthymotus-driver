"""Bounded numerical IPC; no ROS, publisher, lease secret or hardware API.

Only the coordinator waits here, never the execution/watchdog thread. One RPC
is in flight; MotionControl retains only the latest pending input.
"""
import json
import logging
import math
import importlib
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

LOG = logging.getLogger(__name__)

MAX_BYTES = 2 * 1024 * 1024


def send(sock, value):
    raw = json.dumps(value, allow_nan=False, separators=(',', ':')).encode()
    if len(raw) > MAX_BYTES:
        raise ValueError('ik_ipc_oversized')
    sock.sendall(struct.pack('!I', len(raw)) + raw)


def receive(sock):
    def exact(size):
        result = bytearray()
        while len(result) < size:
            chunk = sock.recv(size-len(result))
            if not chunk:
                raise EOFError('ik_worker_exited')
            result.extend(chunk)
        return result
    size, = struct.unpack('!I', exact(4))
    if not 0 < size <= MAX_BYTES:
        raise ValueError('ik_ipc_oversized')
    try:
        value = json.loads(exact(size))
    except (ValueError, UnicodeError) as exc:
        raise ValueError('ik_ipc_invalid') from exc
    if not isinstance(value, dict):
        raise ValueError('ik_ipc_invalid')
    return value


class NumericalWorker:
    """Lazy restart drops the triggering request; never retries old work."""
    def __init__(self, path, velocity=None, *, driver_dir, solver_module,
                 solver_class, visualization_module, python_executable=None):
        self.python_executable = str(python_executable or sys.executable)
        self.driver_dir = str(Path(driver_dir).resolve())
        self.solver_module, self.solver_class = solver_module, solver_class
        self.visualization_module = visualization_module
        self.path, self.requested_velocity = str(path), velocity
        self._lock = threading.Lock()
        self._process = self._socket = None
        self._serial = 0
        self.last_ms = 0.
        self.last_envelope = None
        self._closed = False
        self._start()

    def _start(self):
        parent, child = socket.socketpair()
        self._socket = parent
        self._process = subprocess.Popen(
            [self.python_executable, str(Path(__file__).resolve()), str(child.fileno()),
             self.driver_dir, self.solver_module, self.solver_class, self.visualization_module],
            cwd=self.driver_dir, pass_fds=(child.fileno(),),
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=None)
        child.close()
        try:
            data = self._exchange('load', {'path': self.path, 'velocity': self.requested_velocity},
                                  {}, time.monotonic()+10.)
            if hasattr(self, 'profile_sha256') and data['profile_sha256'] != self.profile_sha256:
                raise ValueError('ik_worker_calibration_changed')
            self.profile, self.profile_sha256 = data['profile'], data['profile_sha256']
            self.hands_enabled, self.velocity = data['hands_enabled'], data['velocity']
            self.collision_checks_enabled = data.get('collision_checks_enabled')
            self.model = SimpleNamespace(velocityLimit=data['velocity_limits'])
            self.lower, self.upper = data['lower'], data['upper']
        except BaseException:
            self._retire()
            raise

    def _retire(self):
        if self._socket:
            self._socket.close()
            self._socket = None
        process, self._process = self._process, None
        if process:
            if process.poll() is None:
                process.terminate()
            try:
                process.wait(timeout=.15)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=.15)

    def interrupt(self):
        self._closed = True
        sock = self._socket
        if sock:
            try:sock.shutdown(socket.SHUT_RDWR)
            except OSError:pass

    def close(self):
        self.interrupt()
        with self._lock:
            self._closed = True
            self._retire()

    def _exchange(self, op, payload, identity, deadline):
        remaining = deadline-time.monotonic()
        if remaining <= 0:
            raise ValueError('command_expired')
        self._serial += 1
        tag = {'request': self._serial, **identity}
        self._socket.settimeout(remaining)
        send(self._socket, {'op': op, 'payload': payload, 'identity': tag, 'deadline': deadline})
        self._socket.settimeout(max(.001, deadline-time.monotonic()))
        reply = receive(self._socket)
        if reply.get('identity') != tag:
            raise ValueError('ik_worker_identity_mismatch')
        if time.monotonic() >= deadline:
            raise ValueError('command_expired')
        if reply.get('error'):
            raise ValueError(reply['error'])
        return reply['result']

    def call(self, op, payload, *, identity=None, deadline=None):
        deadline = time.monotonic()+.15 if deadline is None else deadline
        if not self._lock.acquire(timeout=max(0., min(.15, deadline-time.monotonic()))):
            raise ValueError('ik_worker_busy')
        try:
            if self._closed:
                raise ValueError('ik_worker_closed')
            if not self._process or self._process.poll() is not None:
                self._retire()
                self._start()
                raise ValueError('ik_worker_restarted_fresh_input_required')
            try:
                return self._exchange(op, payload, identity or {}, deadline)
            except (OSError, EOFError, KeyError, TypeError, json.JSONDecodeError) as exc:
                # A timed-out framed stream cannot be reused: its late reply
                # belongs to the old request. Retire it, then accept fresh work.
                reason = ('ik_worker_timeout' if isinstance(exc, TimeoutError) else
                          'ik_worker_exited' if isinstance(exc, EOFError) else
                          'ik_worker_unavailable')
                LOG.warning('IK IPC failed op=%s request=%s exception=%s exitcode=%s reason=%s',
                            op, self._serial, type(exc).__name__, self._process.poll(), reason,
                            exc_info=True)
                self._retire()
                raise ValueError(reason) from exc
            except ValueError as exc:
                if str(exc) in ('ik_worker_identity_mismatch', 'ik_ipc_oversized', 'ik_ipc_invalid'):
                    self._retire()
                raise
        finally:
            self._lock.release()

    def self_test(self, q):
        return self.call('self_test', {'q': q}, deadline=time.monotonic()+5.)

    def solve_frame(self, body, state, generation):
        identity = {k: body[k] for k in ('boot_id', 'session_id', 'seq', 'source_seq',
                    'mapping_epoch', 'model_version', 'calibration_version', 'frame', 'valid_until_ns')}
        identity['generation'] = generation
        result = self.call('solve', {'values': body['values'], 'q': state['feedback']['q'],
            'commanded': state.get('commanded_q'),
            'measured_ns': state['feedback'].get('arm_ns')}, identity=identity,
            deadline=body['valid_until_ns']/1e9)
        self.last_ms = result['last_ms']
        self.last_envelope = result['envelope']
        return result['q']

    def render(self, state, output, hardware, generation):
        return self.call('render', {'state': state, 'output': output, 'hardware': hardware},
                         identity={'generation': generation})

    def return_step(self, q, previous, deadline, measured_ns):
        result = self.call('return_step', {'q': list(q), 'previous': list(previous), 'measured_ns': measured_ns}, deadline=deadline)
        self.last_envelope = result['envelope']
        return result['q']


def serve(sock, solver_module, solver_class, visualization_module):
    solver = None
    key = None
    while True:
        request = receive(sock)
        identity = request['identity']
        try:
            op, data, deadline = request['op'], request['payload'], request['deadline']
            def budget():
                if time.monotonic() >= deadline:
                    raise ValueError('command_expired')
            budget()
            if op == 'load':
                module = importlib.import_module(solver_module)
                solver = getattr(module, solver_class)(data['path'])
                if data['velocity'] is not None:
                    velocity = data['velocity']
                    maximum = min(float(min(solver.model.velocityLimit)), getattr(solver, 'velocity_limit', 5.))
                    if type(velocity) not in (int, float) or not math.isfinite(velocity) or not 0 < velocity <= maximum:
                        raise ValueError('joint_velocity_limit')
                    solver.velocity = float(velocity)
                result = dict(profile=solver.profile, profile_sha256=solver.profile_sha256,
                    hands_enabled=solver.hands_enabled, velocity=solver.velocity,
                    collision_checks_enabled=getattr(solver, 'collision_checks_enabled', None),
                    velocity_limits=solver.model.velocityLimit.tolist(),
                    lower=solver.model.lowerPositionLimit[solver.indices].tolist(),
                    upper=solver.model.upperPositionLimit[solver.indices].tolist())
            elif op == 'self_test':
                solver.self_test(data['q'])
                result = True
            elif op == 'solve':
                transform = importlib.import_module(solver_module).transform
                current = tuple(identity[k] for k in ('boot_id', 'session_id', 'mapping_epoch', 'generation'))
                if current != key:
                    solver.visualization_sample = solver.last_valid_visualization = None
                    key = current
                    if hasattr(solver, 'ik'):
                        solver.ik.reset(data['q'])
                values = data['values']
                targets = [transform({'position': values[i:i+3], 'orientation': values[i+3:i+7]}) for i in (0, 7)]
                q = solver.solve(targets, data['q'], data['commanded'], deadline_monotonic=deadline)
                result = {'q': list(q), 'last_ms': solver.last_ms, 'envelope': {**solver.last_envelope, 'measured_ns': data['measured_ns']}}
            elif op == 'render':
                from scipy.spatial.transform import Rotation
                snapshot = importlib.import_module(visualization_module).snapshot
                if key and key[-1] != identity['generation']:
                    solver.visualization_sample = solver.last_valid_visualization = None
                    key = None
                state = data['state']
                feedback = state['feedback']
                fresh = 0 <= time.monotonic_ns()-feedback.get('arm_ns', 0) <= 100_000_000
                poses = [t[:3, 3].tolist()+Rotation.from_matrix(t[:3, :3]).as_quat().tolist()
                         for t in solver.palms(feedback['q'])] if fresh else None
                adapter = SimpleNamespace(solver=solver, link=SimpleNamespace(feedback=lambda: state),
                    output=data['output'], hardware_output=data['hardware'])
                result = {'poses': poses, 'visualization': snapshot(adapter)}
            elif op == 'return_step':
                import numpy as np
                q, previous = np.asarray(data['q']), np.asarray(data['previous'])
                lower, upper = solver.model.lowerPositionLimit[solver.indices], solver.model.upperPositionLimit[solver.indices]
                if np.any(lower > 0) or np.any(upper < 0):
                    raise ValueError('neutral_outside_limits')
                target = np.zeros_like(q)
                result = {'q': target.tolist(),
                          'envelope': {**solver.motion_envelope(q, previous, target, budget), 'measured_ns': data['measured_ns']}}
            else:
                raise ValueError('ik_worker_unknown_operation')
            budget()
            reply = {'identity': identity, 'result': result}
        except Exception as exc:
            reply = {'identity': identity, 'error': str(exc)[:160] or type(exc).__name__}
        send(sock, reply)


if __name__ == '__main__':
    try:
        sys.path.insert(0, sys.argv[2])
        serve(socket.socket(fileno=int(sys.argv[1])), *sys.argv[3:6])
    except (EOFError, OSError):
        pass
