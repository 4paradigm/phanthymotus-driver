"""Card lifecycle with real IK/gate/lagging plant; no ROS or hardware.

The shared executor's socket/process is replaced, but its lifecycle, controller
worker, numeric solver, lease validation and physical receipts remain real.
"""
import threading

import pytest

from test_motion_control import MotionControl, TeleopExecutor, chain, packet, wait_for


@pytest.fixture
def running_chain(chain, monkeypatch):
    c, e, p = chain.c, chain.e, chain.p

    class Bus:
        closed = False

        def poll(self):
            return 0 if self.closed else None

        def terminate(self):
            self.closed = True

        def wait(self, timeout=None):
            return 0

    def open_transport():
        e._bus_process = Bus()
        e._closed.clear()
        e._thread = threading.Thread(target=e._closed.wait, daemon=True)
        e._thread.start()

    monkeypatch.setattr(e, '_start', open_transport)
    monkeypatch.setattr(e, 'start', TeleopExecutor.start.__get__(e))
    monkeypatch.setattr(c, 'start', MotionControl.start.__get__(c))
    p.run(e.gate)
    yield chain
    c.stop()
    e.stop()


@pytest.mark.parametrize('tool', ['controller', 'executor'])
def test_actuator_start_reports_ready_without_claim_or_output(running_chain, tool):
    c, e, p = running_chain.c, running_chain.e, running_chain.p
    item = c if tool == 'controller' else e
    started = item.dispatch('start', {})
    assert started['state'] == 'ready'
    assert started['execution_state'] == 'idle'
    assert item.info()['state'] == 'idle'
    assert not e.gate.session_id and not p.writes
    assert item.dispatch('stop', {})['state'] == 'idle'
    assert item.dispatch('stop', {})['state'] == 'idle'
    assert not e.gate.session_id and not p.writes


def test_motion_stop_ends_only_its_worker_and_restart_is_explicit(running_chain):
    c, e, p = running_chain.c, running_chain.e, running_chain.p
    assert c.dispatch('start', {})['state'] == 'ready'
    worker, shared = c._worker, e._thread
    assert c.dispatch('start', {})['state'] == 'ready'
    assert c._worker is worker
    assert c.dispatch('stop', {})['state'] == 'idle'
    assert not worker.is_alive() and c._worker is None
    assert e._thread is shared and shared.is_alive()
    assert not e._closed.is_set() and e._bus_process.poll() is None
    with e.gate.legacy():
        assert not e.gate.session_id  # Legacy arm admission remains usable.
    for receive in (c.receive_eef, c.receive_joint):
        with pytest.raises(ValueError, match='motion_control_stopped'):
            receive({})
    assert c.dispatch('start', {})['state'] == 'ready'
    assert c._worker is not worker and c._worker.is_alive()
    assert not p.writes


def test_preview_stop_retires_credentials_and_pending_input(running_chain):
    c, e, p = running_chain.c, running_chain.e, running_chain.p
    lease = c.dispatch('prepare_preview', {})
    worker = c._worker
    c.receive_eef(packet(c, lease))
    result = c.dispatch('stop', {})
    assert result['state'] == 'idle' and result['preview']
    assert not worker.is_alive() and c._pending is None
    assert not c.info()['preview'] and not e.gate.session_id and not p.writes
    again = c.dispatch('prepare_preview', {})
    assert again['session_id'] != lease['session_id']
    with pytest.raises(ValueError, match='stale_session'):
        c.receive_eef(packet(c, lease))


def test_slow_ik_result_is_discarded_when_card_stops(running_chain, monkeypatch):
    c = running_chain.c
    entered, release = threading.Event(), threading.Event()
    solve = c.solver.solve

    def slow(*args, **kwargs):
        entered.set()
        assert release.wait(2)
        return solve(*args, **kwargs)

    monkeypatch.setattr(c.solver, 'solve', slow)
    lease = c.dispatch('prepare_preview', {})
    c.receive_eef(packet(c, lease))
    assert entered.wait(1)
    worker = c._worker
    replies = []
    stopper = threading.Thread(target=lambda: replies.append(c.dispatch('stop', {})))
    stopper.start()
    try:
        assert c._closed.wait(1)
        release.set()
        stopper.join(2)
        assert not stopper.is_alive() and replies[0]['state'] == 'idle'
        assert not worker.is_alive() and c._pending is None
        assert not running_chain.commands and not running_chain.p.writes
        assert c._preview_state == 'idle'
    finally:
        release.set()
        stopper.join(2)


def test_stuck_worker_reports_failure_and_blocks_restart(running_chain, monkeypatch):
    c = running_chain.c
    entered, release = threading.Event(), threading.Event()

    def stuck(*args, **kwargs):
        entered.set()
        assert release.wait(3)
        raise ValueError('injected_solver_failure')

    monkeypatch.setattr(c.solver, 'solve', stuck)
    lease = c.dispatch('prepare_preview', {})
    c.receive_eef(packet(c, lease))
    assert entered.wait(1)
    worker = c._worker
    try:
        stopped = c.dispatch('stop', {})
        assert stopped['state'] == 'error'
        assert stopped['code'] == 'motion_control_thread_stop_unconfirmed'
        assert worker.is_alive() and c._closed.is_set()
        assert c.dispatch('start', {})['code'] == 'motion_control_thread_stop_unconfirmed'
        with pytest.raises(ValueError, match='motion_control_stopped'):
            c.receive_eef(packet(c, lease, seq=1))
        assert running_chain.e._thread.is_alive()
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert c.dispatch('stop', {})['state'] == 'idle'
    assert c.dispatch('start', {})['state'] == 'ready'


def test_shutdown_cancels_start_that_has_not_opened_worker(running_chain, monkeypatch):
    c, e = running_chain.c, running_chain.e
    entered, release = threading.Event(), threading.Event()

    def slow_start():
        entered.set()
        assert release.wait(2)

    monkeypatch.setattr(e, 'start', slow_start)
    replies = []
    starter = threading.Thread(target=lambda: replies.append(c.dispatch('start', {})))
    starter.start()
    try:
        assert entered.wait(1)
        c.stop()  # Bundle shutdown must invalidate an in-flight start as well.
        release.set()
        starter.join(2)
        assert replies[0]['code'] == 'motion_control_start_cancelled'
        assert c._worker is None and c._closed.is_set()
    finally:
        release.set()
        starter.join(2)


@pytest.mark.parametrize('tool', ['controller', 'executor'])
def test_start_diagnostics_preserve_execution_hold(running_chain, tool):
    c, e = running_chain.c, running_chain.e
    lease = c.dispatch('claim', {})
    assert c.dispatch('pause', lease)['state'] == 'hold'
    item = c if tool == 'controller' else e
    result = item.dispatch('start', {})
    assert result['state'] == 'ready' and result['execution_state'] == 'hold'
    assert result['ownership_held'] and e.gate.session_id == lease['session_id']
    assert not e.gate.release_requested


def test_pause_and_release_do_not_stop_shared_controller(running_chain):
    c = running_chain.c
    lease = c.dispatch('claim', {})
    worker = c._worker
    assert c.dispatch('pause', lease)['state'] == 'hold'
    assert c._worker is worker and worker.is_alive() and not c._closed.is_set()
    assert not c.dispatch('release', lease).get('error')
    wait_for(lambda: not running_chain.e.gate.session_id)
    assert worker.is_alive() and not c._closed.is_set()


def test_unauthorized_stop_does_not_cancel_another_session(running_chain):
    c, e = running_chain.c, running_chain.e
    lease = c.dispatch('claim', {})
    worker, generation = c._worker, c._generation
    result = c.dispatch('stop', {})
    assert result['code'] == 'invalid_lease'
    assert worker.is_alive() and not c._closed.is_set()
    assert c._generation == generation and e.gate.session_id == lease['session_id']


def test_stopping_preview_does_not_release_a_new_legacy_owner(running_chain):
    c, e = running_chain.c, running_chain.e
    c.dispatch('prepare_preview', {})
    lease = e.dispatch('claim', {})
    result = c.dispatch('stop', {})
    assert c._worker is None and not c.info()['preview']
    assert result['ownership_held'] and not result['stop_confirmed']
    assert e.gate.session_id == lease['session_id'] and not e.gate.release_requested


@pytest.mark.parametrize('tool', ['controller', 'executor'])
def test_failed_transport_start_does_not_report_ready(running_chain, monkeypatch, tool):
    c, e = running_chain.c, running_chain.e

    def failed():
        raise OSError('injected_transport_failure')

    monkeypatch.setattr(e, '_start', failed)
    result = (c if tool == 'controller' else e).dispatch('start', {})
    assert result['state'] == 'error' and result['code'] == 'injected_transport_failure'
    assert c._worker is None and e._thread is None
    assert not e.gate.session_id and not running_chain.p.writes


def test_unconfirmed_hold_is_not_reported_as_idle_or_released(running_chain):
    c, e, p = running_chain.c, running_chain.e, running_chain.p
    lease = c.dispatch('claim', {})
    p.frozen = True
    result = c.dispatch('stop', lease)
    assert result['state'] in ('hold', 'fault')
    assert result['ownership_held'] and not result['stop_confirmed']
    assert e.gate.session_id == lease['session_id']
    assert c._worker is None and e._thread.is_alive()


def test_stop_cancels_return_worker_without_shutdown_of_other_cards(running_chain):
    c, e, p = running_chain.c, running_chain.e, running_chain.p
    with p.lock:
        p.q[3] = p.target[3] = .3
    lease = c.dispatch('claim', {})
    c.dispatch('finish', lease)
    wait_for(lambda: len(running_chain.commands) > 0)
    returning, solver, shared = c._finish_thread, c._worker, e._thread
    assert not c.dispatch('stop', lease).get('error')
    assert not returning.is_alive() and not solver.is_alive()
    assert shared.is_alive() and e._thread is shared
    finish = c.dispatch('finish_status', {})
    assert finish['state'] == 'error' and not finish['return_completed']
    wait_for(lambda: not e.gate.session_id)
    assert e.gate.status()['stop_confirmed']
