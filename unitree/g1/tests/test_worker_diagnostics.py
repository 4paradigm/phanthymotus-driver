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


def test_real_worker_sanitizes_child_stderr_before_solver_import(tmp_path):
    from pathlib import Path
    import socket
    import subprocess
    import sys
    import time
    from common.motion import worker as module

    (tmp_path / 'noisy_solver.py').write_text(
        "import sys\nsys.stderr.write('\\x1b[31m' + 'x'*12000 + '\\x00' + '\\n')\n"
        "class Solver:\n    def __init__(self, path): raise ValueError('expected_test_error')\n"
    )
    parent, child = socket.socketpair()
    process = subprocess.Popen(
        [sys.executable, str(Path(module.__file__).resolve()), str(child.fileno()),
         str(tmp_path), 'noisy_solver', 'Solver', 'unused_visualization'],
        pass_fds=(child.fileno(),), stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
    )
    child.close()
    parent.settimeout(5.)
    try:
        module.send(parent, {'op': 'load', 'payload': {'path': 'unused', 'velocity': None},
                            'identity': {'request': 1}, 'deadline': time.monotonic()+5.})
        # Drain stderr concurrently: the bounded log writer must not block on a
        # full pipe while this test waits for the IPC reply.
        output = []
        reader = threading.Thread(target=lambda: output.append(process.stderr.read()))
        reader.start()
        reply = module.receive(parent)
        assert reply['error'] == 'expected_test_error'
        parent.close()
        process.wait(timeout=5.)
        reader.join(5.)
        data = b''.join(output)
        assert data and b'\x1b' not in data and b'\x00' not in data
        assert max(map(len, data.splitlines())) < 4096
        assert process.returncode == 0
    finally:
        parent.close()
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5.)


def test_g1_transport_selection_does_not_override_xml_profile():
    from pathlib import Path
    import xml.etree.ElementTree as ET
    root = Path(__file__).resolve().parents[1]
    assert 'ENV FASTDDS_BUILTIN_TRANSPORTS=' not in (root / 'Dockerfile').read_text()
    profile = ET.parse(root / 'dds-local.xml').getroot()
    ns = {'dds': 'http://www.eprosima.com'}
    assert profile.find('.//dds:useBuiltinTransports', ns).text == 'false'
    assert profile.find('.//dds:interfaceWhiteList/dds:address', ns).text == '127.0.0.1'
