"""Actual bus child and parent decoder over anonymous IPC; ROS is a stand-in."""
import json
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

from motion_stream import sign
from teleop_executor import TeleopExecutor
from test_motion_stream import rig


DRIVER = Path(__file__).resolve().parents[1]


@pytest.fixture(params=[False, True], ids=['legacy-only', 'with-control-v2'])
def bus_child(tmp_path, request):
    stubs = tmp_path / 'stubs'
    (stubs / 'rclpy').mkdir(parents=True)
    (stubs / 'rclpy/__init__.py').write_text('''
import json
import sys
from types import SimpleNamespace
callbacks = {}
running = True
def init(*, domain_id):
    assert domain_id == 42
def ok(): return running
def spin_once(node, timeout_sec):
    global running
    print(json.dumps({'spin': True}), flush=True)
    line = sys.stdin.readline()
    if not line:
        running = False
        return
    event = json.loads(line)
    if 'legacy' in event:
        callbacks['/isolated/motion/teleop/command'](SimpleNamespace(data=event['legacy']))
def try_shutdown(): pass
''')
    (stubs / 'rclpy/node.py').write_text('''
import json
from types import SimpleNamespace
import rclpy
class Node:
    def __init__(self, name): pass
    def create_subscription(self, message, topic, callback, qos):
        rclpy.callbacks[topic] = callback
    def create_publisher(self, message, topic, qos):
        def publish(msg):
            print(json.dumps({'topic': topic, 'data': json.loads(msg.data)}), flush=True)
        return SimpleNamespace(publish=publish)
    def destroy_node(self): pass
''')
    (stubs / 'rclpy/executors.py').write_text('class ExternalShutdownException(Exception): pass\n')
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
    parent, child = socket.socketpair(type=socket.SOCK_DGRAM)
    for endpoint in (parent, child):
        endpoint.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        endpoint.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 65536)
    parent.settimeout(3)
    env = {**os.environ, 'PYTHONPATH': str(stubs), 'PYTHONNOUSERSITE': '1',
           'PYTHONDONTWRITEBYTECODE': '1',
           'FASTRTPS_DEFAULT_PROFILES_FILE': str(DRIVER / 'dds-local.xml')}
    process = subprocess.Popen([sys.executable, str(DRIVER / 'teleop_executor.py'),
        '--bus', str(child.fileno()), 'isolated'] + (['--control-v2'] if request.param else []),
        env=env, cwd=tmp_path, pass_fds=(child.fileno(),),
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
    child.close()

    def event():
        # Read bytes directly: no buffered readline can hide an already-read
        # next event from select. Bound failures instead of hanging pytest.
        deadline = time.monotonic() + 3
        line = bytearray()
        while not line.endswith(b'\n'):
            timeout = deadline - time.monotonic()
            assert timeout > 0 and select.select([process.stdout], [], [], timeout)[0], 'bus child stalled'
            data = os.read(process.stdout.fileno(), 1)
            assert data, f'bus child exited: {process.poll()}'
            line.extend(data)
        return json.loads(line)

    def cycle(value=None):
        process.stdin.write((json.dumps(value or {}) + '\n').encode())
        output = []
        while True:
            received = event()
            if received == {'spin': True}:return output
            output.append(received)

    try:
        assert event() == {'spin': True}
        yield SimpleNamespace(process=process, wire=parent, cycle=cycle, v2=request.param)
    finally:
        process.stdin.close()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        stderr = process.stderr.read()
        process.stdout.close()
        process.stderr.close()
        parent.close()
        assert process.returncode == 0, stderr.decode(errors='replace')


@pytest.mark.parametrize('bad', ['{', '[1,', '[' * 1100 + '0' + ']' * 1100],
                         ids=['truncated-object', 'truncated-array', 'deep-json'])
def test_legacy_bad_command_keeps_bus_and_parent_decoder_alive(bus_child, bad):
    gate, _, writes, advance, packet = rig()
    executor = TeleopExecutor({}, 'isolated', None, None, None, [])
    executor.gate = gate
    assert bus_child.cycle({'legacy': bad}) == []
    executor._command(SimpleNamespace(data=bus_child.wire.recv(16385)))
    assert gate.state == 'hold' and gate.latest is None and not writes
    assert bus_child.process.poll() is None

    # Invalid v1 input keeps the existing HOLD contract. Only a confirmed
    # release and explicit fresh claim can authorize the following target.
    gate.hold('operator_pause', release=True)
    for _ in range(3):
        advance(20)
        gate.tick()
    assert gate.session_id is None and gate.status()['stop_confirmed']
    lease = gate.claim()
    valid = packet(seq=1, boot_id=lease['boot_id'], session_id=lease['session_id'])
    valid['mac'] = sign({k: v for k, v in valid.items() if k != 'mac'}, lease['secret'])
    assert bus_child.cycle({'legacy': json.dumps(valid)}) == []
    executor._command(SimpleNamespace(data=bus_child.wire.recv(16385)))
    assert gate.seq == 1 and gate.latest['q'] == valid['q']
    advance(20)
    gate.tick()
    assert gate.applied_seq == 1 and gate.state == 'active'
    assert bus_child.process.poll() is None


@pytest.mark.parametrize('bad', [
    b'{', b'\xff', b'null', b'[]', b'{"state":1,"state":2}',
    b'{"value":NaN}', b'{"value":1e999}',
    ('{"nested":' + '[' * 10000 + '0' + ']' * 10000 + '}').encode(),
    b'{"_motion_route":"joint_output"}',
    b'{"_motion_route":"joint_output","packet":[]}',
    b'{"_motion_route":"joint_output","packet":{"value":NaN}}',
    b'{"_motion_route":"unknown","packet":{}}',
], ids=['truncated', 'invalid-utf8', 'null', 'array', 'duplicate-key', 'nan', 'infinity',
        'deep-object', 'missing-joint', 'joint-array', 'joint-nan', 'unknown-route'])
def test_invalid_parent_frame_does_not_kill_bus_or_replace_valid_latest(bus_child, bad):
    expected = []
    if bus_child.v2:
        joint = {'seq': 7, 'values': [0.] * 14}
        bus_child.wire.send(json.dumps({'_motion_route': 'joint_output', 'packet': joint}).encode())
        expected.append({'topic': '/isolated/motion/arm/command', 'data': joint})
    feedback = {'state': 'ready', 'seq': 7}
    bus_child.wire.send(json.dumps(feedback).encode())
    bus_child.wire.send(bad)
    assert bus_child.cycle() == expected + [{'topic': '/isolated/motion/teleop/feedback', 'data': feedback}]
    assert bus_child.process.poll() is None
    next_feedback = {'state': 'active', 'seq': 8}
    bus_child.wire.send(bad)
    bus_child.wire.send(json.dumps(next_feedback).encode())
    assert bus_child.cycle() == [{'topic': '/isolated/motion/teleop/feedback', 'data': next_feedback}]
    assert bus_child.process.poll() is None
