"""On-demand, read-only PointCloud2 transport through the existing NOS SSH path.

This file also runs from memory on NOS. Keep imports standard-library-only;
the remote entry point imports its already-installed ROS runtime lazily.
"""
from __future__ import annotations

import base64
import json
import math
import os
from pathlib import Path
import re
import select
import shlex
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

HEADER = struct.Struct('!8sII')
MAGIC = b'S10CLD1\n'
MAX_METADATA = 16384
MAX_DATA = 64 * 1024 * 1024
MAX_AGE_S = 1.
RESPONSE_TIMEOUT_S = 3.
STARTUP_TIMEOUT_S = 15.


def _exact(read, size):
    result = bytearray()
    while len(result) < size:
        part = read(size - len(result))
        if not part:
            raise EOFError('cloud relay stream ended mid-packet')
        result.extend(part)
    return bytes(result)


def _integer(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError('invalid cloud relay integer')


def _validate(metadata, data_size, topic):
    if not isinstance(metadata, dict) or metadata.get('topic') != topic:
        raise ValueError('cloud relay topic mismatch')
    if metadata.get('kind') == 'idle' and data_size == 0:
        return
    if metadata.get('kind') != 'cloud':
        raise ValueError('invalid cloud relay packet kind')
    frame = metadata.get('frame_id')
    if not isinstance(frame, str) or not frame or len(frame) > 128 or any(c.isspace() for c in frame):
        raise ValueError('invalid cloud relay frame')
    stamp = metadata.get('stamp')
    if not isinstance(stamp, list) or len(stamp) != 2:
        raise ValueError('invalid cloud relay stamp')
    _integer(stamp[0], 0, 2147483647)
    _integer(stamp[1], 0, 999999999)
    if stamp == [0, 0]:
        raise ValueError('missing cloud relay stamp')
    for key, low, high in [('width', 0, 2000000), ('height', 0, 2000000),
                           ('point_step', 1, MAX_DATA), ('row_step', 0, MAX_DATA)]:
        _integer(metadata.get(key), low, high)
    if (metadata['width'] * metadata['height'] > 2000000 or
            metadata['row_step'] < metadata['width'] * metadata['point_step'] or
            metadata['row_step'] * metadata['height'] != data_size):
        raise ValueError('invalid cloud relay layout')
    if any(type(metadata.get(key)) is not bool for key in ['is_bigendian', 'is_dense']):
        raise ValueError('invalid cloud relay boolean')
    fields = metadata.get('fields')
    if not isinstance(fields, list) or len(fields) > 256:
        raise ValueError('invalid cloud relay fields')
    for f in fields:
        if not isinstance(f, list) or len(f) != 4 or not isinstance(f[0], str) or len(f[0]) > 128:
            raise ValueError('invalid cloud relay field descriptor')
        _integer(f[1], 0, MAX_DATA)
        _integer(f[2], 1, 8)
        _integer(f[3], 0, MAX_DATA)
    age = metadata.get('age_s')
    if type(age) not in (int, float) or not math.isfinite(age) or age < 0:
        raise ValueError('invalid cloud relay age')


def write_packet(write, metadata, data=b''):
    encoded = json.dumps(metadata, separators=(',', ':'), allow_nan=False).encode('utf-8')
    if len(encoded) > MAX_METADATA or len(data) > MAX_DATA:
        raise ValueError('cloud relay packet exceeds size limits')
    _validate(metadata, len(data), metadata.get('topic'))
    for value in [HEADER.pack(MAGIC, len(encoded), len(data)), encoded, data]:
        view = memoryview(value)
        while view:
            size = write(view)
            if not size:
                raise BrokenPipeError('cloud relay output closed')
            view = view[size:]


def read_packet(read, topic):
    magic, metadata_size, data_size = HEADER.unpack(_exact(read, HEADER.size))
    if magic != MAGIC or not 0 < metadata_size <= MAX_METADATA or data_size > MAX_DATA:
        raise ValueError('invalid cloud relay packet header')
    metadata = json.loads(_exact(read, metadata_size))
    _validate(metadata, data_size, topic)
    if metadata['kind'] == 'idle':
        return None, 0.
    data = _exact(read, data_size)
    fields = [SimpleNamespace(name=f[0], offset=f[1], datatype=f[2], count=f[3])
              for f in metadata['fields']]
    sec, nanosec = metadata['stamp']
    message = SimpleNamespace(
        header=SimpleNamespace(frame_id=metadata['frame_id'],
                               stamp=SimpleNamespace(sec=sec, nanosec=nanosec)),
        fields=fields, data=data, **{k: metadata[k] for k in
            ['width', 'height', 'point_step', 'row_step', 'is_bigendian', 'is_dense']})
    return message, float(metadata['age_s'])


def ssh_command(config, topic):
    if not isinstance(topic, str) or not re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', topic):
        raise ValueError('cloud relay requires an absolute ROS topic')
    domain = int(config.get('ros', {}).get('robot_domain_id', 0))
    encoded = base64.b64encode(Path(__file__).read_bytes()).decode('ascii')
    bootstrap = (f'source=__import__("base64").b64decode("{encoded}"); '
                 'scope={"__name__":"s10_cloud_transport"}; exec(source,scope); '
                 'raise SystemExit(scope["remote_transport"](source,__import__("sys").argv[1:]))')
    python = ['python3', '-u', '-c', bootstrap,
              '--topic', topic, '--domain', str(domain)]
    setup = ('set -e; source /opt/ros/jazzy/setup.bash 1>&2; '
             'export PYTHONDONTWRITEBYTECODE=1 ROS_LOCALHOST_ONLY=1; exec ' + shlex.join(python))
    command = 'exec bash --noprofile --norc -c ' + shlex.quote(setup)
    return _ssh_command(config, command)


def _ssh_command(config, command):
    spatial = config.get('controlled_spatial', {})
    host = os.environ.get('S10_DRMAP_SSH_HOST') or spatial.get('ssh_host', '10.21.33.106')
    user = os.environ.get('S10_DRMAP_SSH_USER') or spatial.get('ssh_user', 'user')
    password = os.environ.get('S10_DRMAP_PASSWORD', '')
    known_hosts = os.environ.get('S10_SSH_KNOWN_HOSTS') or str(
        Path(__file__).with_name('resource') / 'ssh_known_hosts')
    args = ['ssh', '-T', '-o', 'ConnectTimeout=5', '-o', 'StrictHostKeyChecking=yes',
            '-o', f'UserKnownHostsFile={known_hosts}', '-o', 'GlobalKnownHostsFile=/dev/null',
            '-o', 'ServerAliveInterval=5', '-o', 'ServerAliveCountMax=2', '--',
            f'{user}@{host}', command]
    if password:
        if not shutil.which('sshpass'):
            raise RuntimeError('SSH cloud relay requires the deployment sshpass client')
        env = os.environ.copy()
        env['SSHPASS'] = password
        return ['sshpass', '-e', *args], env
    return ['ssh', '-o', 'BatchMode=yes', *args[1:]], None


def _signal(process, value):
    try:
        os.killpg(process.pid, value)
    except PermissionError:
        # A concurrently exiting process group can reject a signal on macOS;
        # Popen checks/reaps its owned child before signalling the individual PID.
        process.send_signal(value)
    except ProcessLookupError:
        pass


def _terminate(process):
    if process.poll() is None:
        try:
            _signal(process, signal.SIGTERM)
            process.wait(timeout=1.)
        except subprocess.TimeoutExpired:
            _signal(process, signal.SIGKILL)
            process.wait(timeout=1.)
        except ProcessLookupError:
            process.wait(timeout=1.)
    for stream in [process.stdin, process.stdout]:
        stream.close()


class CloudRelay:
    def __init__(self, config, topic, rate_hz, on_message, on_error):
        if type(rate_hz) not in (int, float) or not math.isfinite(rate_hz) or not 0 < rate_hz <= 15:
            raise ValueError('invalid cloud relay rate')
        self.config, self.topic, self.rate_hz = config, topic, rate_hz
        self.on_message, self.on_error = on_message, on_error
        self._closed = threading.Event()
        self._lock = threading.Lock()
        self._process = None
        self._state, self._received = 'idle', 0
        self._thread = threading.Thread(target=self._run, name='s10-cloud-ssh', daemon=True)

    def start(self):
        self._thread.start()

    def is_alive(self):
        return not self._closed.is_set() and self._thread.is_alive()

    def status(self):
        with self._lock:
            return {'state': self._state, 'received': self._received, 'topic': self.topic}

    def close(self):
        self._closed.set()
        with self._lock:
            process = self._process
        if process is not None and process.poll() is None:
            _signal(process, signal.SIGTERM)
        if self._thread.ident is not None:
            self._thread.join(timeout=RESPONSE_TIMEOUT_S + 2.)
        if self._thread.is_alive():
            raise RuntimeError('cloud relay did not stop')
        with self._lock:
            self._state = 'stopped'

    def _read(self, process, size, deadline):
        while not self._closed.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('cloud relay response stalled')
            ready, _, _ = select.select([process.stdout], [], [], min(.2, remaining))
            if ready:
                return os.read(process.stdout.fileno(), min(size, 65536))
        raise EOFError('cloud relay stopped')

    def _run(self):
        while not self._closed.is_set():
            process = None
            try:
                with self._lock:
                    self._state = 'connecting'
                args, env = ssh_command(self.config, self.topic)
                process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL, bufsize=0, env=env, start_new_session=True)
                with self._lock:
                    self._process = process
                first_response = True
                while not self._closed.is_set():
                    requested_at = time.monotonic()
                    process.stdin.write(b'N\n')
                    process.stdin.flush()
                    timeout = STARTUP_TIMEOUT_S if first_response else RESPONSE_TIMEOUT_S
                    message, age = read_packet(lambda n: self._read(
                        process, n, requested_at + timeout), self.topic)
                    first_response = False
                    if self._closed.is_set():
                        break
                    if message is not None:
                        if age + time.monotonic() - requested_at > MAX_AGE_S:
                            self.on_error('CLOUD_RELAY_STALE', 'SSH cloud response exceeded the freshness limit')
                        else:
                            with self._lock:
                                self._state, self._received = 'streaming', self._received + 1
                            self.on_message(message)
                    self._closed.wait(max(0., 1 / self.rate_hz - (time.monotonic() - requested_at)))
            except Exception:
                if not self._closed.is_set():
                    self.on_error('CLOUD_RELAY_FAILED',
                        'SSH cloud relay disconnected or returned invalid data; check existing NOS SSH access and ROS runtime')
            finally:
                if process is not None:
                    _terminate(process)
                with self._lock:
                    self._process = None
                    self._state = 'stopped' if self._closed.is_set() else 'retrying'
            self._closed.wait(1.)


def remote_transport(source, arguments):
    """Forward only the reader's dedicated pipe to the private SSH channel."""
    reader, writer = os.pipe()
    try:
        code = ('scope={"__name__":"s10_cloud_reader"}; '
                f'exec({source!r},scope); raise SystemExit(scope["remote_main"]())')
        try:
            process = subprocess.Popen([sys.executable, '-u', '-c', code, *arguments,
                '--output-fd', str(writer)], stdin=sys.stdin.buffer,
                stdout=sys.stderr, stderr=sys.stderr, pass_fds=(writer,))
        finally:
            os.close(writer)
        try:
            while True:
                chunk = os.read(reader, 65536)
                if not chunk:
                    return process.wait()
                view = memoryview(chunk)
                while view:
                    count = os.write(sys.stdout.fileno(), view)
                    if not count:
                        raise BrokenPipeError('SSH cloud transport closed')
                    view = view[count:]
        finally:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.)
    finally:
        os.close(reader)


def remote_main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--output-fd', type=int, required=True)
    args = parser.parse_args()
    if args.output_fd <= 2:
        parser.error('cloud protocol requires a dedicated output descriptor')
    output = os.fdopen(args.output_fd, 'wb', buffering=0)
    import rclpy
    from sensor_msgs.msg import PointCloud2
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.utilities import get_default_context
    # Default rclpy.init creates remote log files. Jazzy Node also constructs
    # a parameter-event writer even with start_parameter_services=False.
    context = get_default_context()
    context.init(args=[], domain_id=args.domain, initialize_logging=False)
    class ReadOnlyNode(Node):
        def create_publisher(self, message_type, topic, *args, **kwargs):
            if topic != '/parameter_events':
                raise RuntimeError('cloud reader cannot create publishers')
            return SimpleNamespace(publish=lambda event: None)
    node = ReadOnlyNode('s10_cloud_readonly_ssh', context=context, use_global_arguments=False,
        enable_rosout=False, start_parameter_services=False,
        parameter_overrides=[Parameter('use_sim_time', value=False),
                             Parameter('start_type_description_service', value=False)])
    condition, stopped = threading.Condition(), threading.Event()
    latest = [None, 0., 0]
    def receive(message):
        with condition:
            latest[:] = [message, time.monotonic(), latest[2] + 1]
            condition.notify_all()
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    subscription = node.create_subscription(PointCloud2, args.topic, receive, qos)
    def spin():
        while not stopped.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    thread = threading.Thread(target=spin, name='s10-cloud-readonly-ros', daemon=True)
    thread.start()
    sent = 0
    try:
        while sys.stdin.buffer.readline(3) == b'N\n':
            with condition:
                condition.wait_for(lambda: latest[2] != sent, timeout=.5)
                message, received_at, sequence = latest
            age = time.monotonic() - received_at
            if message is None or sequence == sent or age > MAX_AGE_S:
                write_packet(output.write, {'kind': 'idle', 'topic': args.topic})
                continue
            metadata = {key: getattr(message, key) for key in
                        ['width', 'height', 'point_step', 'row_step', 'is_bigendian', 'is_dense']}
            metadata.update(kind='cloud', topic=args.topic, frame_id=message.header.frame_id,
                stamp=[message.header.stamp.sec, message.header.stamp.nanosec], age_s=age,
                fields=[[f.name, f.offset, f.datatype, f.count] for f in message.fields])
            write_packet(output.write, metadata, memoryview(message.data).cast('B'))
            sent = sequence
    except BrokenPipeError:
        pass
    finally:
        stopped.set()
        thread.join(timeout=1.)
        node.destroy_node()
        rclpy.shutdown()
        output.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(remote_transport(Path(__file__).read_bytes(), sys.argv[1:]))
