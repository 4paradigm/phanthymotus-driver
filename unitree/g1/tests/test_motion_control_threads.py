"""Real control threads and numerical IPC; synthetic solver and SDK-free gate."""
import copy
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation  # Load before the render RPC deadline.

from common.motion.worker import NumericalWorker
from test_motion_control_contract import controller, Worker
from test_teleop_settings import input_card


def transform(pose):
    return pose


def snapshot(adapter):
    return {'available': True}


class BlockingSolver:
    """Child-side algorithm substitute; blocking must cross the real socket."""
    profile = Worker.profile
    profile_sha256 = Worker.profile_sha256
    hands_enabled = False
    velocity = 1.
    last_ms = 0.
    indices = list(range(10))

    def __init__(self, path):
        self.directory = Path(path)
        self.model = SimpleNamespace(velocityLimit=np.ones(10),
            lowerPositionLimit=np.full(10, -2.), upperPositionLimit=np.full(10, 2.))
        self.last_envelope = {'lower': [-.2]*10, 'upper': [.2]*10}

    def solve(self, targets, q, commanded, **kwargs):
        if (self.directory / 'block').exists():
            (self.directory / 'entered').touch()
            deadline = time.monotonic() + 5.
            while not (self.directory / 'release').exists():
                if time.monotonic() >= deadline:
                    raise RuntimeError('test_release_missing')
                time.sleep(.002)
        return [.1]*10

    def palms(self, q):
        return [np.eye(4), np.eye(4)]


def wait_for(predicate):
    deadline = time.monotonic() + 5.
    while not predicate():
        assert time.monotonic() < deadline, 'thread integration condition timed out'
        time.sleep(.002)


def test_threads_timeout_restart_discard_old_work_and_resume(tmp_path, monkeypatch):
    motion, published, proofs = controller()
    card, sample = input_card(tmp_path)
    motion.executor.cfg = {}
    card.motion, card.executor = motion, motion.executor
    card.clock = time.monotonic_ns
    card.cfg['mode'] = 'live'
    def dispatch(action, args):
        assert action == 'recoverable_hold'
        motion.gate.hold('input_stale')
        return {}
    motion.executor.dispatch = dispatch
    card._operator, card._release_seen = 'prepared-test-session', True
    card.mapping.calibrate(sample, [[0, 0, 0, 0, 0, 0, 1]]*2)
    status = motion.gate.status
    monkeypatch.setattr(motion.gate, 'status', lambda: dict(status(),
        continuation_ready=True, continuation_after_ns=1))
    worker = NumericalWorker(tmp_path, driver_dir=Path(__file__).parent,
        solver_module=__name__, solver_class='BlockingSolver', visualization_module=__name__)
    motion.solver = worker
    first_process = worker._process
    calls, errors, completed = [], [], []
    call, solve = worker.call, worker.solve_frame

    def recorded_call(*args, **kwargs):
        try:
            return call(*args, **kwargs)
        except ValueError as exc:
            errors.append(str(exc))
            raise

    def recorded_solve(body, *args):
        calls.append(body['source_seq'])
        result = solve(body, *args)
        completed.append(body['source_seq'])
        return result

    monkeypatch.setattr(worker, 'call', recorded_call)
    monkeypatch.setattr(worker, 'solve_frame', recorded_solve)

    def feed(sequence):
        value = copy.deepcopy(sample)
        value['sequence'] = sequence
        value['received_monotonic_ns'] = value['source_monotonic_ns'] = time.monotonic_ns()
        value['left']['grip'] = value['right']['grip'] = 1.
        assert card.receive(value)
        wait_for(lambda: card._processed is not None and card._processed[-1] == sequence)

    (tmp_path / 'block').touch()
    card._thread = threading.Thread(target=card._run, name='test-teleop-control')
    try:
        motion.start()
        card._thread.start()
        feed(1)
        wait_for(lambda: (tmp_path / 'entered').exists())
        for sequence in (2, 3, 4):
            feed(sequence)
        with motion._lock:
            assert motion._pending[0]['source_seq'] == 4
        wait_for(lambda: 'ik_worker_timeout' in errors)
        wait_for(lambda: 'ik_worker_restarted_fresh_input_required' in errors)
        wait_for(lambda: motion._snapshot is not None)
        assert first_process.poll() is not None
        assert worker._process.pid != first_process.pid
        assert motion._pending is None and calls == [1]
        assert not published and not proofs
        # Input expiry can fence the solve while the timed-out child is reaped.
        assert motion.gate.holds and set(motion.gate.holds) <= {'ik_recoverable', 'input_stale'}

        # A successfully returned solution must still lose to a cancellation.
        (tmp_path / 'entered').unlink()
        feed(5)
        wait_for(lambda: (tmp_path / 'entered').exists())
        motion.cancel_pending()
        (tmp_path / 'release').touch()
        wait_for(lambda: 5 in completed)
        (tmp_path / 'block').unlink()
        feed(6)
        wait_for(lambda: len(published) == 1)
        assert calls == [1, 5, 6]
        assert published[0]['source_seq'] == 6 and len(proofs) == 1
        assert published[0]['session_id'] == 'session'
        assert card._thread.is_alive() and motion._worker.is_alive()
    finally:
        card._closed.set()
        card._wake.set()
        if card._thread.ident is not None:
            card._thread.join(2.)
        coordinator = motion._worker
        process = worker._process
        motion.stop()
        assert not card._thread.is_alive()
        assert coordinator is None or not coordinator.is_alive()
        assert process is None or process.poll() is not None
