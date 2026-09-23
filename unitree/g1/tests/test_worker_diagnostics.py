"""IPC fault classification without starting a solver or hardware."""
import threading
from types import SimpleNamespace
import pytest
from common.motion.worker import NumericalWorker

@pytest.mark.parametrize('failure,reason', [
    (TimeoutError('timed out'), 'ik_worker_timeout'),
    (EOFError('closed'), 'ik_worker_exited'),
    (OSError('broken'), 'ik_worker_unavailable'),
])
def test_ipc_cause_is_logged_and_stream_retired(failure, reason, caplog):
    worker = object.__new__(NumericalWorker)
    worker._lock = threading.Lock()
    worker._closed = False
    worker._serial = 9
    worker._process = SimpleNamespace(poll=lambda: None)
    retired = []
    worker._retire = lambda: retired.append(True)
    def exchange(*args): raise failure
    worker._exchange = exchange
    with pytest.raises(ValueError, match=reason) as caught:
        worker.call('render', {})
    assert caught.value.__cause__ is failure
    assert retired == [True]
    assert 'op=render request=9' in caplog.text
    assert type(failure).__name__ in caplog.text
