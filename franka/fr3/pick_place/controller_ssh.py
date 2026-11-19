"""Invoke an in-memory worker on the verified PNP host."""

from contextlib import contextmanager
import ipaddress
import json
import queue
import re
import shlex
import subprocess
import tempfile
import threading
import time
from itertools import count

from arm_ip_sync import ACCESS


def command_for(controller_ip, username):
    address = str(ipaddress.IPv4Address(controller_ip))
    if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_.-]{0,63}', username):
        raise ValueError('Invalid SSH username')
    return [
        'ssh', '-T', '-F', '/dev/null', '-o', 'StrictHostKeyChecking=yes',
        '-o', 'GlobalKnownHostsFile=/dev/null',
        '-o', f'UserKnownHostsFile={ACCESS / "known_hosts"}',
        '-o', 'HostKeyAlias=franka-pnp-controller',
        '-o', 'ConnectTimeout=5', '-o', 'NumberOfPasswordPrompts=1',
        '-o', 'PubkeyAuthentication=no', '-o', 'PreferredAuthentications=password',
        f'{username}@{address}',
        'source /opt/ros/jazzy/setup.bash && '
        'source /home/pnp/franka_ros2_ws/install/setup.bash && exec python3 -',
    ]


_WORKER = '''
import contextlib
import io
import json
import sys
import rclpy

rclpy.init()
# The existing request handlers own their nodes. Keep the context across calls
# so DDS discovery need not start from scratch with a new Python process.
rclpy.init = lambda: None
rclpy.shutdown = lambda: None
for line in sys.stdin:
    try:
        sequence, source, request = json.loads(line)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            exec(compile(source, "<franka-planning>", "exec"), {"request": request, "json": json})
        result = next(line[11:] for line in output.getvalue().splitlines() if line.startswith("FR3_RESULT "))
        reply = {"result": json.loads(result)}
    except Exception as error:
        reply = {"error": f"{type(error).__name__}: {error}"}
    print("FR3_RPC " + json.dumps([sequence, reply]), flush=True)
'''


@contextmanager
def planning_worker(endpoint, username, password, domain, cancel):
    command = command_for(endpoint, username)
    command[-1] = command[-1].removesuffix('exec python3 -') + shlex.join([
        'env', '-u', 'ROS_LOCALHOST_ONLY', f'ROS_DOMAIN_ID={domain}',
        'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST', 'python3', '-u', '-c', _WORKER,
    ])
    responses = queue.Queue()
    requests = count()
    with tempfile.TemporaryFile() as secret:
        secret.write(password.encode() + b'\n')
        secret.seek(0)
        with subprocess.Popen(
            ['sshpass', '-d', str(secret.fileno()), *command], stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
            pass_fds=(secret.fileno(),),
        ) as process:
            def read_responses():
                for line in process.stdout:
                    if line.startswith('FR3_RPC '):
                        responses.put(line[8:])
                responses.put(None)

            reader = threading.Thread(target=read_responses, daemon=True)
            reader.start()

            def call(source, payload, timeout):
                sequence = next(requests)
                try:
                    process.stdin.write(json.dumps([sequence, source, payload], allow_nan=False) + '\n')
                    process.stdin.flush()
                except BrokenPipeError as error:
                    raise RuntimeError('PNP planning worker disconnected') from error
                deadline = time.monotonic() + timeout
                while not cancel.is_set():
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('PNP planning request timed out')
                    try:
                        line = responses.get(timeout=min(.1, remaining))
                    except queue.Empty:
                        continue
                    if line is None:
                        raise RuntimeError('PNP planning worker disconnected')
                    reply_sequence, reply = json.loads(line)
                    if reply_sequence != sequence:
                        continue
                    if 'error' in reply:
                        raise RuntimeError(reply['error'])
                    return reply['result']
                raise RuntimeError('Pick cancelled')

            try:
                yield call
            finally:
                try:
                    process.stdin.close()
                except BrokenPipeError:
                    pass
                try:
                    process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                reader.join(timeout=2)
