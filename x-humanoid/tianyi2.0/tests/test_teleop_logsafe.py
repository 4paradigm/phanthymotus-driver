"""Run the real DDS child entry point with ROS stubs, never robot I/O."""
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys

import pytest


DRIVER = Path(__file__).resolve().parents[1]
ROOT = DRIVER.parents[1]


@pytest.mark.parametrize('layout', ['checkout', 'staged-image'])
@pytest.mark.parametrize('import_failure', [False, True])
def test_bus_child_installs_logsafe_before_ros_import(tmp_path, layout, import_failure):
    if layout == 'checkout':
        driver = DRIVER
    else:
        driver = tmp_path / 'work'
        driver.mkdir()
        for name in ('teleop_executor.py', 'motion_stream.py', 'dds-local.xml'):
            shutil.copy2(DRIVER / name, driver / name)
        shutil.copytree(ROOT / 'common', driver / 'common')

    stubs = tmp_path / 'stubs'
    (stubs / 'rclpy').mkdir(parents=True)
    (stubs / 'rclpy/__init__.py').write_text('''
import os
import sys
from common import logsafe

# Checking during import proves the child did not wait until rclpy.init().
assert logsafe._installed
assert isinstance(sys.stdout, logsafe.LineAtomicStream)
assert isinstance(sys.stderr, logsafe.LineAtomicStream)
assert sys.stdout.fileno() == 1 and sys.stderr.fileno() == 2
calls = []
original_write = os.write
def observed_write(fd, data):
    calls.append((fd, data))
    return original_write(fd, data)
os.write = observed_write
print('out\\x00\\x1b[31m-safe\\x1b[0m')
print('err\\x00\\x1b[31m-safe\\x1b[0m', file=sys.stderr)
assert calls == [(1, b'out-safe\\n'), (2, b'err-safe\\n')]
if os.environ['ROS_STUB_FAIL'] == '1':
    raise RuntimeError('startup\\x00\\x1b[31m-failed\\x1b[0m')
def init(*, domain_id):
    assert domain_id == 42
def ok():
    return False
def try_shutdown():
    pass
''')
    (stubs / 'rclpy/executors.py').write_text('class ExternalShutdownException(Exception): pass\n')
    (stubs / 'rclpy/node.py').write_text('''
class Node:
    def __init__(self, name):
        assert name == 'tianyi_teleop_local_bus'
    def create_subscription(self, *args): pass
    def create_publisher(self, *args): return None
    def destroy_node(self): pass
''')
    (stubs / 'rclpy/qos.py').write_text('''
class QoSProfile:
    def __init__(self, **kwargs): pass
class ReliabilityPolicy: BEST_EFFORT = 1
class HistoryPolicy: KEEP_LAST = 1
class DurabilityPolicy: VOLATILE = 1
''')
    (stubs / 'std_msgs').mkdir()
    (stubs / 'std_msgs/__init__.py').write_text('')
    (stubs / 'std_msgs/msg.py').write_text('class String: pass\n')
    env = {**os.environ, 'PYTHONPATH': str(stubs), 'PYTHONNOUSERSITE': '1',
           'PYTHONDONTWRITEBYTECODE': '1', 'ROS_STUB_FAIL': str(int(import_failure)),
           'FASTRTPS_DEFAULT_PROFILES_FILE': str(driver / 'dds-local.xml')}
    # Anonymous local IPC only: no DDS participants or external sockets exist.
    parent, child = socket.socketpair(type=socket.SOCK_DGRAM)
    try:
        result = subprocess.run(
            [sys.executable, str(driver / 'teleop_executor.py'), '--bus',
             str(child.fileno()), 'isolated'], cwd=tmp_path, env=env,
            pass_fds=(child.fileno(),), capture_output=True, timeout=10)
    finally:
        parent.close()
        child.close()
    assert result.returncode == (1 if import_failure else 0), result.stderr
    assert result.stdout == b'out-safe\n'
    assert result.stderr.startswith(b'err-safe\n')
    assert b'\x00' not in result.stderr and b'\x1b' not in result.stderr
    if import_failure:
        assert b'RuntimeError: startup-failed\n' in result.stderr
    else:
        assert result.stderr == b'err-safe\n'
