"""Fresh-controller log protection, without ROS, sockets or an SDK."""

from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
WORKER = ROOT / "x-humanoid/tianyi2.0/loco_servo_process.py"


def run_probe(body):
    # Dependency attributes are resolved when the production module imports
    # them. This lets the test observe its earliest protected import without
    # changing the worker or connecting to any robot.
    script = r'''
import os, runpy, sys, threading, types
sys.path.insert(0, sys.argv[2])
core = types.ModuleType("loco_servo")
core.TOGGLES = ()
core._validated_min_vx = lambda value, limit: value
core.LocoServoPlugin = type("Core", (), {})
backend = types.ModuleType("slamtec_velocity")
backend.SlamtecVelocityBackend = type("Backend", (), {})
sys.modules["loco_servo"] = core
sys.modules["slamtec_velocity"] = backend
''' + body
    return subprocess.run([sys.executable, "-I", "-u", "-c", script, str(WORKER), str(ROOT)],
                          capture_output=True, check=True, timeout=10)


def test_fresh_script_protects_import_logs_and_concurrent_long_lines():
    result = run_probe(r'''
del core.LocoServoPlugin
def import_core(name):
    if name != "LocoServoPlugin":
        raise AttributeError(name)
    from common import logsafe
    assert logsafe._installed
    assert isinstance(sys.stdout, logsafe.LineAtomicStream)
    assert isinstance(sys.stderr, logsafe.LineAtomicStream)
    assert sys.stdout.fileno() == 1 and sys.stderr.fileno() == 2
    print("\x1b[31mcore-import\x1b[0m\x00\r")
    count = 4
    barrier = threading.Barrier(count)
    def log(index):
        sys.stdout.write("thread-%d:\x1b[32m\x00" % index)
        barrier.wait()
        sys.stdout.write("x" * (logsafe.PIPE_BUF * 3) + "\x1b[0m\n")
    threads = [threading.Thread(target=log, args=(i,)) for i in range(count)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    print("pipe-limit=%d" % logsafe.PIPE_BUF)
    return type("Core", (), {})
core.__getattr__ = import_core
del backend.SlamtecVelocityBackend
def import_backend(name):
    if name != "SlamtecVelocityBackend":
        raise AttributeError(name)
    sys.stderr.write("\x1b[33mbackend-import\x1b[0m\x00\r\n")
    raise SystemExit(0)
backend.__getattr__ = import_backend
runpy.run_path(sys.argv[1], run_name="__main__")
''')
    lines = result.stdout.splitlines()
    assert lines[0] == b"core-import"
    pipe_limit = int(lines[-1].removeprefix(b"pipe-limit="))
    concurrent = lines[1:-1]
    assert len(concurrent) == 4
    for index in range(4):
        matched = [line for line in concurrent if line.startswith(f"thread-{index}:".encode())]
        assert len(matched) == 1
        assert matched[0].count(b"thread-") == 1
        assert b"[truncated " in matched[0]
        assert len(matched[0]) + 1 <= pipe_limit
    assert result.stderr == b"backend-import\n"
    assert not any(byte < 32 and byte not in (9, 10) or byte == 127
                   for byte in result.stdout + result.stderr)


def test_direct_child_entry_installs_before_socket_setup_without_changing_parent_import():
    result = run_probe(r'''
stdout, stderr = sys.stdout, sys.stderr
module = runpy.run_path(sys.argv[1], run_name="controller_import_probe")
assert sys.stdout is stdout and sys.stderr is stderr
try:
    module["_child"](-1, -1)
except (OSError, ValueError):
    pass
else:
    raise AssertionError("invalid descriptors unexpectedly opened")
from common import logsafe
assert logsafe._installed
assert isinstance(sys.stdout, logsafe.LineAtomicStream)
assert isinstance(sys.stderr, logsafe.LineAtomicStream)
print("\x1b[31mdirect-child\x1b[0m\x00\r")
sys.stderr.write("\x1b[32mdirect-error\x1b[0m\x00\r\n")
''')
    assert result.stdout == b"direct-child\n"
    assert result.stderr == b"direct-error\n"


def test_flat_work_path_does_not_index_beyond_filesystem_root():
    result = run_probe(r'''
del core.LocoServoPlugin
def import_core(name):
    if name != "LocoServoPlugin":
        raise AttributeError(name)
    from common import logsafe
    assert logsafe._installed
    print("flat-work-bootstrap-ok")
    raise SystemExit(0)
core.__getattr__ = import_core
source = open(sys.argv[1]).read()
exec(compile(source, "/work/loco_servo_process.py", "exec"),
     {"__name__": "__main__", "__file__": "/work/loco_servo_process.py"})
''')
    assert result.stdout == b"flat-work-bootstrap-ok\n"
    assert result.stderr == b""


def test_flat_image_finds_its_own_common_without_repository_pythonpath():
    result = run_probe(r'''
from pathlib import Path
import shutil, tempfile
del core.LocoServoPlugin
def import_core(name):
    if name != "LocoServoPlugin":
        raise AttributeError(name)
    from common import logsafe
    assert logsafe._installed
    assert isinstance(sys.stderr, logsafe.LineAtomicStream)
    print("flat-image-common-ok")
    raise SystemExit(0)
core.__getattr__ = import_core
with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work / "common").mkdir()
    shutil.copy(Path(sys.argv[2]) / "common/logsafe.py", work / "common/logsafe.py")
    shutil.copy(sys.argv[1], work / "loco_servo_process.py")
    sys.path.remove(sys.argv[2])
    runpy.run_path(str(work / "loco_servo_process.py"), run_name="__main__")
''')
    assert result.stdout == b"flat-image-common-ok\n"
    assert result.stderr == b""
