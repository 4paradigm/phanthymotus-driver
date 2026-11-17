"""Raw cloud transport contracts without SSH accounts or a ROS installation."""
import importlib
import io
import json
import os
import select
import shlex
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest


def relay_module():
    return importlib.import_module('deep_robotics.s10_pro.cloud_relay')


def metadata(**changes):
    return {'kind': 'cloud', 'topic': '/LIDAR/POINTS', 'frame_id': 'lidar_link',
            'stamp': [123, 456789], 'width': 1, 'height': 1, 'point_step': 26,
            'row_step': 26, 'is_bigendian': True, 'is_dense': True,
            'fields': [['x', 0, 7, 1], ['y', 4, 7, 1], ['z', 8, 7, 1],
                       ['intensity', 12, 7, 1], ['ring', 16, 4, 1], ['timestamp', 18, 8, 1]],
            'age_s': .02, **changes}


def packet(meta=None, data=None):
    encoded = json.dumps(metadata() if meta is None else meta, separators=(',', ':')).encode()
    data = struct.pack('>ffffHd', 1, 2, 3, 4, 5, 123.5) if data is None else data
    return struct.pack('!8sII', b'S10CLD1\n', len(encoded), len(data)) + encoded + data


def test_decode_preserves_vendor_layout_stamp_and_exact_bytes():
    raw = packet()
    message, age = relay_module().read_packet(io.BytesIO(raw).read, '/LIDAR/POINTS')
    assert message.header.frame_id == 'lidar_link'
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (123, 456789)
    assert (message.width, message.height, message.point_step, message.row_step) == (1, 1, 26, 26)
    assert message.is_bigendian is True and message.is_dense is True
    assert [(f.name, f.offset, f.datatype, f.count) for f in message.fields] == [
        ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1),
        ('intensity', 12, 7, 1), ('ring', 16, 4, 1), ('timestamp', 18, 8, 1)]
    assert message.data == struct.pack('>ffffHd', 1, 2, 3, 4, 5, 123.5)
    assert age == .02


def test_writer_matches_independently_constructed_wire_packet_and_handles_short_writes():
    received = bytearray()
    def short_write(value):
        received.extend(value[:3])
        return min(3, len(value))
    relay_module().write_packet(short_write, metadata(), struct.pack('>ffffHd', 1, 2, 3, 4, 5, 123.5))
    assert bytes(received) == packet()


def test_reader_handles_partial_pipe_reads_and_idle_response():
    raw = io.BytesIO(packet())
    message, _ = relay_module().read_packet(lambda n: raw.read(min(n, 3)), '/LIDAR/POINTS')
    assert message.header.stamp.nanosec == 456789
    idle = packet({'kind': 'idle', 'topic': '/LIDAR/POINTS'}, b'')
    assert relay_module().read_packet(io.BytesIO(idle).read, '/LIDAR/POINTS') == (None, 0.)


@pytest.mark.parametrize('value', [
    b'', b'S10', packet()[:-1],
    struct.pack('!8sII', b'INVALID!', 1, 0),
    struct.pack('!8sII', b'S10CLD1\n', 16385, 0),
    struct.pack('!8sII', b'S10CLD1\n', 1, 64 * 1024 * 1024 + 1),
    packet(metadata(topic='/ALIGNED_POINTS')), packet(metadata(stamp=[0, 0])),
    packet(metadata(stamp=[1, 1000000000])), packet(metadata(age_s=-1)),
    packet(metadata(age_s=float('inf'))), packet(metadata(width=2_000_001)),
    packet(metadata(fields=[['x', 0, 7, 1]] * 257)),
    packet(metadata(frame_id='')), packet(metadata(is_bigendian=1)),
    packet(metadata(row_step=27)), packet(metadata(kind='unknown')),
])
def test_rejects_malformed_truncated_or_unbounded_packets(value):
    with pytest.raises((ValueError, EOFError)):
        relay_module().read_packet(io.BytesIO(value).read, '/LIDAR/POINTS')


def test_ssh_launch_reuses_nos_credentials_without_exposing_them(monkeypatch):
    monkeypatch.setenv('S10_DRMAP_PASSWORD', 'fixture-secret')
    monkeypatch.setenv('S10_SSH_KNOWN_HOSTS', '/tmp/test-pinned-hosts')
    monkeypatch.setattr(relay_module().shutil, 'which', lambda name: '/usr/bin/' + name)
    args, env = relay_module().ssh_command({}, '/LIDAR/POINTS')
    assert args[:3] == ['sshpass', '-e', 'ssh']
    assert 'fixture-secret' not in repr(args)
    assert env['SSHPASS'] == 'fixture-secret'
    assert 'StrictHostKeyChecking=yes' in args
    assert 'UserKnownHostsFile=/tmp/test-pinned-hosts' in args
    assert args[-2] == 'user@10.21.33.106'


def test_key_auth_has_no_password_fallback_or_prompt(monkeypatch):
    monkeypatch.delenv('S10_DRMAP_PASSWORD', raising=False)
    args, env = relay_module().ssh_command({}, '/LIDAR/POINTS')
    assert args[0] == 'ssh' and 'BatchMode=yes' in args


def wait_until(predicate, seconds=3):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return True
        threading.Event().wait(.01)
    return False


def fake_ssh(monkeypatch, script):
    # Replace only the external SSH boundary; exercise real pipes and child cleanup.
    module = relay_module()
    real_popen = subprocess.Popen
    children = []
    def spawn(*args, **kwargs):
        child = real_popen([sys.executable, '-u', '-c', script], **kwargs)
        children.append(child)
        return child
    monkeypatch.setattr(module.subprocess, 'Popen', spawn)
    monkeypatch.setattr(module, 'ssh_command', lambda *a: (['ssh', 'fixture'], None))
    return children


def test_credit_protocol_delivers_one_raw_frame_and_closes_real_child(monkeypatch):
    raw = packet()
    children = fake_ssh(monkeypatch, "import sys\nwhile sys.stdin.buffer.readline():\n sys.stdout.buffer.write(" + repr(raw) + "); sys.stdout.buffer.flush()\n")
    messages, errors = [], []
    relay = relay_module().CloudRelay({}, '/LIDAR/POINTS', 5, messages.append,
                                       lambda code, reason: errors.append((code, reason)))
    try:
        relay.start()
        assert wait_until(lambda: len(messages) >= 1)
        assert messages[0].data == struct.pack('>ffffHd', 1, 2, 3, 4, 5, 123.5)
        assert not errors
        assert relay.status()['state'] == 'streaming'
    finally:
        relay.close()
    assert not relay.is_alive() and all(p.poll() is not None for p in children)


def test_stale_remote_samples_are_dropped_before_callback(monkeypatch):
    raw = packet(metadata(age_s=2.))
    fake_ssh(monkeypatch, "import sys\nwhile sys.stdin.buffer.readline():\n sys.stdout.buffer.write(" + repr(raw) + "); sys.stdout.buffer.flush()\n")
    messages, errors = [], []
    relay = relay_module().CloudRelay({}, '/LIDAR/POINTS', 5, messages.append,
                                       lambda code, reason: errors.append(code))
    try:
        relay.start()
        assert wait_until(lambda: errors)
        assert errors[0] == 'CLOUD_RELAY_STALE' and not messages
    finally:
        relay.close()


def test_slow_ssh_ros_startup_has_separate_budget_then_uses_fresh_samples(monkeypatch):
    module = relay_module()
    monkeypatch.setattr(module, 'RESPONSE_TIMEOUT_S', .1)
    monkeypatch.setattr(module, 'STARTUP_TIMEOUT_S', .8, raising=False)
    monkeypatch.setattr(module, 'MAX_AGE_S', .15)
    raw = packet(metadata(age_s=0.))
    children = fake_ssh(monkeypatch, "import sys,time\ntime.sleep(.3)\nwhile sys.stdin.buffer.readline():\n sys.stdout.buffer.write(" + repr(raw) + "); sys.stdout.buffer.flush()\n")
    messages, errors = [], []
    relay = module.CloudRelay({}, '/LIDAR/POINTS', 15, messages.append,
                              lambda code, reason: errors.append(code))
    try:
        relay.start()
        assert wait_until(lambda: messages)
        assert errors == ['CLOUD_RELAY_STALE']
        assert len(children) == 1
    finally:
        relay.close()


def test_close_interrupts_stalled_response_and_does_not_leak_thread(monkeypatch):
    children = fake_ssh(monkeypatch, "import sys,time\nsys.stdin.buffer.readline(); time.sleep(30)")
    relay = relay_module().CloudRelay({}, '/LIDAR/POINTS', 5, lambda m: None, lambda *a: None)
    relay.start()
    assert wait_until(lambda: children)
    relay.close()
    assert not relay.is_alive() and all(p.poll() is not None for p in children)


def test_close_can_reap_owned_child_when_group_signalling_is_denied(monkeypatch):
    children = fake_ssh(monkeypatch, "import time;time.sleep(30)")
    def deny_group(*args):
        raise PermissionError('process-group signals unavailable')
    monkeypatch.setattr(relay_module().os, 'killpg', deny_group)
    relay = relay_module().CloudRelay({}, '/LIDAR/POINTS', 5, lambda m: None, lambda *a: None)
    relay.start()
    assert wait_until(lambda: children)
    relay.close()
    assert not relay.is_alive() and all(p.poll() is not None for p in children)


def test_disconnect_retries_without_exposing_remote_stderr(monkeypatch):
    children = fake_ssh(monkeypatch, "import sys\nsys.stderr.write('fixture-secret');sys.exit(255)")
    errors = []
    relay = relay_module().CloudRelay({}, '/LIDAR/POINTS', 5, lambda m: None,
                                       lambda code, reason: errors.append((code, reason)))
    try:
        relay.start()
        assert wait_until(lambda: len(children) >= 2, 4)
        assert errors and all('fixture-secret' not in reason for _, reason in errors)
    finally:
        relay.close()
    assert all(p.poll() is not None for p in children)


def test_roundtrip_bound_drops_delayed_frame_then_accepts_fresh_response(monkeypatch):
    raw = packet()
    script = ("import sys,time\ncount=0\nwhile sys.stdin.buffer.readline():\n"
              " count+=1\n if count==1: time.sleep(1.1)\n"
              " sys.stdout.buffer.write(" + repr(raw) + ");sys.stdout.buffer.flush()\n")
    fake_ssh(monkeypatch, script)
    messages, errors = [], []
    relay = relay_module().CloudRelay({}, '/LIDAR/POINTS', 5, messages.append,
                                      lambda code, reason: errors.append(code))
    try:
        relay.start()
        assert wait_until(lambda: messages, 4)
        assert errors == ['CLOUD_RELAY_STALE']
        assert len(messages) == 1
    finally:
        relay.close()


def test_partial_response_deadline_reconnects_instead_of_waiting_forever(monkeypatch):
    monkeypatch.setattr(relay_module(), 'RESPONSE_TIMEOUT_S', .2)
    monkeypatch.setattr(relay_module(), 'STARTUP_TIMEOUT_S', .2)
    children = fake_ssh(monkeypatch, "import sys,time\nsys.stdin.buffer.readline();sys.stdout.buffer.write(b'S10');sys.stdout.buffer.flush();time.sleep(30)")
    errors = []
    relay = relay_module().CloudRelay({}, '/LIDAR/POINTS', 5, lambda m: None,
                                      lambda code, reason: errors.append(code))
    try:
        relay.start()
        assert wait_until(lambda: len(children) >= 2, 4)
        assert errors[0] == 'CLOUD_RELAY_FAILED'
    finally:
        relay.close()
    assert all(p.poll() is not None for p in children)


@pytest.mark.parametrize('launch', ['file', 'ssh-bootstrap'])
def test_remote_reader_waits_for_credit_keeps_latest_and_shuts_down_on_eof(tmp_path, launch):
    # Fake only the unavailable ROS dependency; execute the real remote entry point.
    ros = tmp_path / 'rclpy'
    ros.mkdir()
    (ros / '__init__.py').write_text('''
import os,sys,time,struct
from types import SimpleNamespace as NS
callback=None
sequence=0
def init(domain_id):
    raise AssertionError('ordinary init enables persistent file logging')
class Context:
    def init(self, *, args, domain_id, initialize_logging):
        assert args==[] and domain_id==0 and initialize_logging is False
context=Context()
class Node:
    def __init__(self, name, **kwargs):
        assert kwargs['enable_rosout'] is False and kwargs['start_parameter_services'] is False
        assert kwargs['context'] is context and kwargs['use_global_arguments'] is False
        overrides={p.name:p.value for p in kwargs['parameter_overrides']}
        assert overrides=={'use_sim_time':False,'start_type_description_service':False}
        # Jazzy constructs this publisher even when parameter services are off.
        self.create_publisher(object, '/parameter_events', None).publish(object())
    def create_publisher(self, *args, **kwargs):
        raise AssertionError('no DDS publisher may be created')
    def create_subscription(self, typ, topic, cb, qos):
        global callback
        assert topic=='/LIDAR/POINTS'
        assert qos.depth==1 and qos.reliability=='best_effort' and qos.durability=='volatile'
        callback=cb
    def destroy_node(self):
        print('NODE_DESTROYED',file=sys.stderr,flush=True)
def create_node(*args,**kwargs):
    assert kwargs['enable_rosout'] is False and kwargs['start_parameter_services'] is False
    return Node()
def spin_once(node,timeout_sec):
    global sequence
    if sequence<8:
        sequence+=1
        callback(NS(header=NS(frame_id='lidar_link',stamp=NS(sec=100+sequence,nanosec=9)),
          fields=[NS(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate('xyz')],
          width=1,height=1,point_step=12,row_step=12,is_bigendian=False,is_dense=True,
          data=struct.pack('<fff',sequence,2,3)))
        print('ROS_LOG_NOISE',flush=True)
        os.write(1, b'NATIVE_ROS_STDOUT\\n')
        os.write(2, b'NATIVE_ROS_STDERR\\n')
        if sequence==8: print('LATEST_READY',file=sys.stderr,flush=True)
    time.sleep(.01)
def shutdown():
    print('ROS_SHUTDOWN',file=sys.stderr,flush=True)
''')
    (ros / 'qos.py').write_text('''
from types import SimpleNamespace as NS
qos_profile_sensor_data=NS(depth=5,reliability='best_effort',durability='volatile')
class QoSProfile:
    def __init__(self,**kwargs):self.__dict__.update(kwargs)
class ReliabilityPolicy:BEST_EFFORT='best_effort'
class DurabilityPolicy:VOLATILE='volatile'
''')
    (ros / 'node.py').write_text('from . import Node\n')
    (ros / 'utilities.py').write_text('from . import context\ndef get_default_context(): return context\n')
    (ros / 'parameter.py').write_text('class Parameter:\n def __init__(self,name,value):self.name,self.value=name,value\n')
    sensor = tmp_path / 'sensor_msgs'
    sensor.mkdir()
    (sensor / '__init__.py').write_text('')
    (sensor / 'msg.py').write_text('class PointCloud2: pass\n')
    env = {**os.environ, 'PYTHONPATH': str(tmp_path)}
    command = [sys.executable, '-u', relay_module().__file__, '--topic', '/LIDAR/POINTS']
    if launch == 'ssh-bootstrap':
        ssh_args, _ = relay_module().ssh_command({}, '/LIDAR/POINTS')
        setup = shlex.split(ssh_args[-1])[-1]
        command = shlex.split(setup.split('; exec ', 1)[1])
        command[0] = sys.executable
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.PIPE, bufsize=0, env=env)
    errors = []
    try:
        def source_ready():
            if select.select([process.stderr], [], [], .05)[0]:
                line = process.stderr.readline()
                errors.append(line)
                return line.strip() == b'LATEST_READY'
            return False
        assert wait_until(source_ready)
        assert not select.select([process.stdout], [], [], .05)[0]
        process.stdin.write(b'N\n')
        message, _ = relay_module().read_packet(process.stdout.read, '/LIDAR/POINTS')
        assert message.header.stamp.sec == 108
        assert message.data == struct.pack('<fff', 8, 2, 3)
        process.stdin.write(b'N\n')
        assert relay_module().read_packet(process.stdout.read, '/LIDAR/POINTS') == (None, 0.)
        process.stdin.close()
        assert process.wait(timeout=3) == 0
        tail = process.stderr.read()
        assert b'NODE_DESTROYED' in tail and b'ROS_SHUTDOWN' in tail
        diagnostics = b''.join(errors) + tail
        assert b'ROS_LOG_NOISE' in diagnostics
        assert b'NATIVE_ROS_STDOUT' in diagnostics and b'NATIVE_ROS_STDERR' in diagnostics
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3)
        process.stdout.close()
        process.stderr.close()


def test_remote_reader_uses_explicit_protocol_fd_without_changing_standard_descriptors():
    # Exercise real pipes; stub only the unavailable ROS dependency.
    script = r'''
import builtins, os, sys, time
from types import SimpleNamespace as NS
from deep_robotics.s10_pro.cloud_relay import remote_main
class Node:
    def __init__(self, *args, **kwargs): pass
    def create_subscription(self, *args, **kwargs): return object()
    def destroy_node(self): pass
sys.modules.update({
    'rclpy': NS(spin_once=lambda *a, **k: time.sleep(.01), shutdown=lambda: None),
    'rclpy.node': NS(Node=Node),
    'rclpy.parameter': NS(Parameter=lambda *a, **k: None),
    'rclpy.qos': NS(QoSProfile=lambda **k: None,
                   ReliabilityPolicy=NS(BEST_EFFORT=1), DurabilityPolicy=NS(VOLATILE=1)),
    'rclpy.utilities': NS(get_default_context=lambda: NS(init=lambda **k: None)),
    'sensor_msgs.msg': NS(PointCloud2=object),
})
original_import = builtins.__import__
def noisy_import(name, *a, **k):
    if name == 'rclpy':
        print('python diagnostic', flush=True)
        os.write(1, b'native stdout diagnostic\n')
        os.write(2, b'native stderr diagnostic\n')
    return original_import(name, *a, **k)
builtins.__import__ = noisy_import
def forbid_standard_descriptor_surgery(*args, **kwargs):
    raise AssertionError('do not duplicate or replace standard descriptors')
os.dup = os.dup2 = forbid_standard_descriptor_surgery
remote_main()
'''
    reader, writer = os.pipe()
    try:
        result = subprocess.run([sys.executable, '-u', '-c', script,
            '--topic', '/LIDAR/POINTS', '--output-fd', str(writer)], input=b'N\n',
            capture_output=True, pass_fds=(writer,), timeout=5)
        assert result.returncode == 0, result.stderr.decode()
        os.close(writer)
        writer = None
        assert os.read(reader, 4096) == packet({'kind': 'idle', 'topic': '/LIDAR/POINTS'}, b'')
    finally:
        os.close(reader)
        if writer is not None:
            os.close(writer)
    assert b'python diagnostic\n' in result.stdout
    assert b'native stdout diagnostic\n' in result.stdout
    assert b'native stderr diagnostic\n' in result.stderr


@pytest.mark.parametrize('exit_code', [0, 23])
def test_transport_forwards_large_binary_data_and_preserves_child_exit_status(exit_code):
    source = f'''
def remote_main():
    import argparse, os, sys
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-fd', type=int)
    args = parser.parse_args()
    print('python diagnostic', flush=True)
    os.write(1, b'native stdout diagnostic\\n')
    os.write(2, b'native stderr diagnostic\\n')
    assert sys.stdin.buffer.readline() == b'N\\n'
    with os.fdopen(args.output_fd, 'wb') as output:
        output.write(bytes(range(256)) * 8192)
    return {exit_code}
'''
    script = ('from deep_robotics.s10_pro.cloud_relay import remote_transport; '
              f'raise SystemExit(remote_transport({source!r}, []))')
    result = subprocess.run([sys.executable, '-u', '-c', script], input=b'N\n',
                            capture_output=True, timeout=5)
    assert result.returncode == exit_code
    assert result.stdout == bytes(range(256)) * 8192
    assert b'python diagnostic' in result.stderr
    assert b'native stdout diagnostic' in result.stderr
    assert b'native stderr diagnostic' in result.stderr
