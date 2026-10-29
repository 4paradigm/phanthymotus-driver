"""Offline state-machine checks; no ROS, SDK, or physical-stop assertion."""
import threading

import pytest

from base_owner import BaseOwner, Grant, StopToken


def test_grant_submit_and_stop_revoke_before_external_result():
    base = BaseOwner()
    assert base.snapshot().state == "idle"
    grant = base.acquire("servo")
    assert grant is not None
    sent = []
    assert base.submit(grant, lambda: sent.append(grant))
    assert sent == [grant]
    token = base.begin_stop()
    assert token is not None
    assert base.snapshot().state == "stopping"
    assert not base.is_current(grant)
    assert not base.submit(grant, lambda: sent.append("stale"))
    assert base.acquire("loco") is None
    assert base.finish_stop(token, success=True)
    assert base.snapshot().state == "idle"
    replacement = base.acquire("loco")
    assert replacement is not None and replacement.generation > grant.generation
    assert not base.submit(grant, lambda: sent.append("late"))
    assert sent == [grant]


def test_preemption_does_not_implicitly_grant_or_resurrect_old_owner():
    base = BaseOwner()
    original = base.acquire("servo")
    assert base.acquire("loco") is None
    stop = base.begin_stop()
    assert base.acquire("loco") is None
    assert base.begin_stop() is None
    assert base.finish_stop(stop, success=True)
    new = base.acquire("servo")
    assert new != original
    assert not base.is_current(original)
    assert base.is_current(new)


def test_failure_or_unconfirmed_stop_needs_successful_retry_and_explicit_reset():
    base = BaseOwner()
    old = base.acquire("servo")
    stop = base.begin_stop()
    # While no result has arrived the state itself stays blocked.
    assert base.acquire("loco") is None
    assert not base.reset_fault()
    assert base.finish_stop(stop, success=False)
    assert base.snapshot().state == "fault"
    assert base.acquire("loco") is None
    assert not base.reset_fault()
    retry = base.begin_stop()
    assert not base.finish_stop(stop, success=True)
    assert base.finish_stop(retry, success=True)
    assert base.snapshot().state == "fault"
    assert base.snapshot().fault_stop_succeeded
    assert base.acquire("loco") is None
    assert base.reset_fault()
    assert not base.reset_fault()
    grant = base.acquire("loco")
    assert grant is not None
    assert not base.is_current(old)
    assert not base.finish_stop(retry, success=False)
    assert base.is_current(grant)


def test_failed_retry_does_not_enable_reset():
    base = BaseOwner()
    base.acquire("one")
    assert base.finish_stop(base.begin_stop(), success=False)
    assert base.finish_stop(base.begin_stop(), success=False)
    assert not base.reset_fault()
    assert base.snapshot().state == "fault"


def test_only_exact_grant_and_stop_token_are_accepted():
    base = BaseOwner()
    grant = base.acquire("one")
    assert not base.submit(Grant("two", grant.generation, grant.epoch), lambda: None)
    assert not base.submit(Grant("one", grant.generation + 1, grant.epoch), lambda: None)
    stop = base.begin_stop()
    assert not base.finish_stop(StopToken(stop.generation + 1, stop.epoch), success=True)
    assert base.snapshot().state == "stopping"
    with pytest.raises(ValueError):
        base.finish_stop(stop, success=1)
    assert base.finish_stop(stop, success=True)
    with pytest.raises(ValueError):
        base.acquire("  ")


def test_tokens_from_another_instance_do_not_match_restarted_owner():
    first, second = BaseOwner(), BaseOwner()
    old = first.acquire("servo")
    current = second.acquire("servo")
    assert old.generation == current.generation
    assert not second.is_current(old)
    first_stop = first.begin_stop()
    second_stop = second.begin_stop()
    assert not second.finish_stop(first_stop, success=True)
    assert second.finish_stop(second_stop, success=True)


def test_concurrent_acquisition_and_preemption_are_serialized():
    base = BaseOwner()
    barrier = threading.Barrier(9)
    results = []
    def contender(i):
        barrier.wait()
        results.append(base.acquire(f"source-{i}"))
    workers = [threading.Thread(target=contender, args=(i,)) for i in range(8)]
    for worker in workers:
        worker.start()
    barrier.wait()
    for worker in workers:
        worker.join(2)
        assert not worker.is_alive()
    grants = [grant for grant in results if grant is not None]
    assert len(grants) == 1
    stop_barrier = threading.Barrier(9)
    stops = []
    def preempt():
        stop_barrier.wait()
        stops.append(base.begin_stop())
    workers = [threading.Thread(target=preempt) for _ in range(8)]
    for worker in workers:
        worker.start()
    stop_barrier.wait()
    for worker in workers:
        worker.join(2)
        assert not worker.is_alive()
    tokens = [token for token in stops if token is not None]
    assert len(tokens) == 1
    assert not base.is_current(grants[0])
    assert base.acquire("contender") is None
    assert base.finish_stop(tokens[0], success=True)
    assert base.acquire("contender") is not None


def test_submit_linearizes_before_stop_and_late_callback_is_rejected():
    base = BaseOwner()
    grant = base.acquire("servo")
    entered = threading.Event()
    release = threading.Event()
    stop_started = threading.Event()
    sent = []
    def enqueue():
        entered.set()
        assert release.wait(2)
        sent.append(grant)
    sender = threading.Thread(target=lambda: base.submit(grant, enqueue))
    sender.start()
    assert entered.wait(2)
    stops = []
    def preempt():
        stop_started.set()
        stops.append(base.begin_stop())
    stopper = threading.Thread(target=preempt)
    stopper.start()
    assert stop_started.wait(2)
    release.set()
    sender.join(2)
    stopper.join(2)
    assert not sender.is_alive() and not stopper.is_alive()
    assert sent == [grant]
    assert stops[0] is not None
    assert not base.submit(grant, lambda: sent.append("late"))
    # The already enqueued item still needs a final fence in the executor.
    assert not base.is_current(sent[0])
