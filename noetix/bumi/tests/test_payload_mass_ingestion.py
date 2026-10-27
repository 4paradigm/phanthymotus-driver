"""Window work must not block the producer or bridge feedback interruptions."""
import copy
import threading
import time
from types import SimpleNamespace

import pytest

from test_payload_mass import raw_frame
import payload_mass_sampling as sampling
from payload_mass_quick import profile_from_home
from payload_mass_sampling import MassError, SharedSampler


@pytest.mark.parametrize("method", ["recent", "read_since"])
def test_slow_window_snapshot_does_not_block_new_feedback(monkeypatch, method):
    sampler = SharedSampler()
    for seq in range(1, 1201):
        sampler.push(raw_frame(seq=seq))
    copying, release, received = threading.Event(), threading.Event(), threading.Event()
    original_copy = copy.deepcopy
    result = []

    def slow_copy(value):
        if threading.current_thread().name == "window_snapshot":
            copying.set()
            assert release.wait(3)
        return original_copy(value)

    monkeypatch.setattr(sampling, "copy", SimpleNamespace(deepcopy=slow_copy))
    reader = threading.Thread(name="window_snapshot", target=lambda: result.extend(
        sampler.recent(3, 0) if method == "recent" else sampler.read_since(0, 0)))
    writer = threading.Thread(target=lambda: (sampler.push(raw_frame(seq=1201)), received.set()))
    reader.start()
    try:
        assert copying.wait(2)
        writer.start()
        # Deliberately keep copying paused: the producer must finish anyway.
        assert received.wait(1), "window copy holds the ingestion lock"
    finally:
        release.set()
        reader.join(3)
        if writer.ident:
            writer.join(3)
    assert len(result) == 1200
    result[-1]["tau"][0] = 999
    assert sampler.read_since(1199, 0)[0]["tau"][0] != 999


@pytest.mark.parametrize("method", ["recent", "read_since"])
def test_gap_during_snapshot_cannot_return_old_epoch(monkeypatch, method):
    sampler = SharedSampler()
    sampler.push(raw_frame())
    original_copy = copy.deepcopy

    def interrupted_copy(value):
        result = original_copy(value)
        sampler.record_error(MassError("STALE_SOURCE", "pipe gap"), recoverable=True)
        return result

    monkeypatch.setattr(sampling, "copy", SimpleNamespace(deepcopy=interrupted_copy))
    with pytest.raises(MassError) as err:
        if method == "recent":
            sampler.recent(3, 0)
        else:
            sampler.read_since(0, 0)
    assert err.value.code == "DATA_ERROR"


@pytest.mark.parametrize("kind", ["recoverable", "fatal_then_recoverable", "fixed_profile"])
def test_empty_capture_restarts_full_window_only_for_recoverable_gap(kind):
    sampler = SharedSampler()
    frame = raw_frame()
    frame["q"][3] = -.6
    p = profile_from_home(frame, "left", 300)
    p.update(min_samples=5, settle_timeout_s=.5)
    stop = threading.Event()
    interrupted = threading.Event()

    def feed():
        started = time.monotonic()
        seq = 0
        while not stop.is_set():
            seq += 1
            if not interrupted.is_set() and time.monotonic()-started > .07:
                if kind == "fatal_then_recoverable":
                    sampler.record_error(MassError("BAD_DATA", "bad frame"))
                sampler.record_error(MassError("STALE_SOURCE", "pipe gap"), recoverable=True)
                interrupted.set()
            current = copy.deepcopy(frame)
            current["source_seq"] = seq
            current["tau"] = [2. if interrupted.is_set() else 1.] * 21
            sampler.push(current)
            stop.wait(.005)

    producer = threading.Thread(target=feed)
    producer.start()
    try:
        if kind == "recoverable":
            frames = sampler.collect(.15, p, stop, learn_pose_from=[0.] * 21)
            assert interrupted.is_set()
            assert {f["error_epoch"] for f in frames} == {1}
            assert all(f["tau"][0] == 2 for f in frames)
            assert frames[-1]["host_monotonic_s"]-frames[0]["host_monotonic_s"] >= .15
        else:
            with pytest.raises(MassError) as err:
                sampler.collect(.15, p, stop,
                                learn_pose_from=None if kind == "fixed_profile" else [0.] * 21)
            assert err.value.code == "DATA_ERROR"
    finally:
        stop.set()
        producer.join(2)


def test_repeated_prepare_gaps_do_not_extend_the_timeout():
    sampler = SharedSampler()
    frame = raw_frame()
    frame["q"][3] = -.6
    p = profile_from_home(frame, "left", 300)
    p.update(min_samples=5, settle_timeout_s=.1)
    stop = threading.Event()

    def feed():
        seq = 0
        while not stop.is_set():
            seq += 1
            if seq % 5 == 0:
                sampler.record_error(MassError("STALE_SOURCE", "pipe gap"), recoverable=True)
            sampler.push({**frame, "source_seq": seq})
            stop.wait(.005)

    producer = threading.Thread(target=feed)
    producer.start()
    start = time.monotonic()
    try:
        with pytest.raises(MassError):
            sampler.collect(.15, p, stop, learn_pose_from=[0.] * 21)
        assert time.monotonic()-start < 1
    finally:
        stop.set()
        producer.join(2)
