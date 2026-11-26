"""Read the PNP control server's existing slave joint stream over SSH."""

from contextlib import contextmanager
import json
import math
import shlex
import subprocess
import tempfile
import threading
import time

_IDLE = """
import json, select, sys, time
from pylibfranka import Robot, RealtimeConfig
robot = Robot(sys.argv[1], RealtimeConfig.kIgnore)
while not select.select([sys.stdin], [], [], 0)[0]:
    state = robot.read_once()
    print(json.dumps(dict(q=list(state.q), timestamp_ms=round(time.time()*1000))), flush=True)
    time.sleep(.1)
"""

_READ = r'''
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import time

import grpc
import polymetis_pb2 as pb
import polymetis_pb2_grpc as rpc

side, expected_ip, idle_source = sys.argv[1:]
runtime = Path.home() / ".local/lib/fr3-pnp"
config = json.loads((runtime / "robot_ips.json").read_text())
if config.get(side) != expected_ip:
    sys.exit("PNP runtime robot IP does not match Controller Configuration")
port = {"right": 50051, "left": 50052}[side]
idle = None
enabled = False
next_check = 0.
commands = b""

def stop_idle():
    global idle
    if idle is not None:
        if idle.poll() is None:
            idle.terminate()
            try:
                idle.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                idle.kill()
                idle.wait()
        idle.stdin.close()
        idle.stdout.close()
        idle = None

try:
    with grpc.insecure_channel(f"127.0.0.1:{port}") as channel:
        robot = rpc.PolymetisControllerServerStub(channel)
        while True:
            if select.select([sys.stdin], [], [], .065)[0]:
                chunk = os.read(sys.stdin.fileno(), 4096)
                if not chunk:
                    break
                commands += chunk
                while b"\n" in commands:
                    line, commands = commands.split(b"\n", 1)
                    mode, generation = line.split()
                    enabled = mode == b"idle"
                    if enabled:
                        next_check = 0.
                    else:
                        stop_idle()
                        print(json.dumps({"released": int(generation)}), flush=True)
                continue
            try:
                state = robot.GetRobotState(pb.Empty(), timeout=.04)
            except grpc.RpcError:
                state = None
            if state is not None:
                stamp = state.timestamp.seconds + state.timestamp.nanos * 1e-9
                q = list(state.joint_positions)
                if 0 <= time.time() - stamp <= .3 and len(q) == 7 and all(map(math.isfinite, q)):
                    stop_idle()
                    print(json.dumps({"q": q, "timestamp_ms": round(stamp*1000)}), flush=True)
                    continue
            now = time.monotonic()
            if enabled and now >= next_check:
                status = subprocess.run(["systemctl", "show", "pnp-fr3-right-teleop.service",
                                         "-p", "ActiveState", "--value"], capture_output=True,
                                        text=True, timeout=.5)
                inactive = status.returncode == 0 and status.stdout.strip() in ("inactive", "failed")
                other_only = False
                if status.returncode == 0 and status.stdout.strip() == "active":
                    selection = runtime / "requested-arm"
                    playback = runtime / "requested-playback"
                    if selection.is_file():
                        other_only = selection.read_text().strip() in ({"left", "right"} - {side})
                    elif playback.is_file():
                        tracks = playback.read_text().splitlines()
                        other_only = bool(tracks) and all(
                            name.startswith(tuple(f"{prefix}-{other}-" for prefix in ("franka", "fr3")
                                                  for other in ({"left", "right"} - {side})))
                            for name in tracks)
                if not (inactive or other_only):
                    stop_idle()
                elif idle is None or idle.poll() is not None:
                    stop_idle()
                    python = Path.home() / ".venv/pylibfranka/bin/python"
                    idle = subprocess.Popen([str(python), "-u", "-c", idle_source, expected_ip],
                                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.DEVNULL)
                next_check = now + 1.
            if idle is not None and select.select([idle.stdout], [], [], 0)[0]:
                line = idle.stdout.readline()
                if line:
                    print(line.decode().strip(), flush=True)
finally:
    stop_idle()
'''


class PnpJointStream:
    def __init__(self, host, user, password, side, address, *, idle=True):
        self.address = address
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._sample = None
        self._process = None
        self._idle = idle
        self._released = threading.Event()
        self._generation = 0
        self._may_hold_fci = False
        self._thread = threading.Thread(target=self._read, args=(host, user, password, side, address),
                                        name='fr3-pnp-joints', daemon=True)
        self._thread.start()

    def _read(self, host, user, password, side, address):
        from .pick_place.controller_ssh import command_for

        command = command_for(host, user)
        command[-1] = (
            'prefix="${PNP_CONDA_BASE:-$HOME/miniconda3}/envs/polymetis"; '
            'export LD_LIBRARY_PATH="$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; '
            'exec "$prefix/bin/python" -u -c ' + shlex.join([_READ, side, address, _IDLE]))
        while not self._stop.is_set():
            try:
                with tempfile.TemporaryFile() as secret:
                    secret.write(password.encode() + b'\n')
                    secret.seek(0)
                    with subprocess.Popen(
                        ['sshpass', '-d', str(secret.fileno()), *command], pass_fds=(secret.fileno(),),
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                        text=True,
                    ) as process:
                        with self._lock:
                            self._process = process
                            if self._stop.is_set():
                                process.stdin.close()
                            else:
                                self._send_idle()
                        for line in process.stdout:
                            try:
                                sample = json.loads(line)
                                if 'released' in sample:
                                    with self._lock:
                                        if sample['released'] == self._generation and not self._idle:
                                            self._may_hold_fci = False
                                            self._released.set()
                                    continue
                                q, stamp = sample['q'], sample['timestamp_ms']
                                if len(q) != 7 or not all(map(math.isfinite, q)) or type(stamp) is not int:
                                    continue
                            except (ValueError, TypeError, KeyError):
                                continue
                            with self._lock:
                                if self._sample is None or stamp > self._sample[0]['timestamp_ms']:
                                    self._sample = sample, time.monotonic()
            except OSError:
                pass
            finally:
                with self._lock:
                    self._process = self._sample = None
            self._stop.wait(1.)

    def _send_idle(self):
        mode = 'idle' if self._idle else 'control'
        self._process.stdin.write(f'{mode} {self._generation}\n')
        self._process.stdin.flush()
        if self._idle:
            self._may_hold_fci = True

    def set_idle(self, enabled):
        with self._lock:
            if enabled != self._idle:
                self._idle = enabled
                self._generation += 1
                self._released.clear()
                if self._process is not None and self._process.poll() is None:
                    try:
                        self._send_idle()
                    except OSError:
                        pass  # A reconnect will send the latest requested mode.
            needs_ack = not enabled and self._may_hold_fci
        if needs_ack and not self._released.wait(2):
            raise RuntimeError('PNP idle FCI release was not confirmed')

    def snapshot(self):
        with self._lock:
            if self._sample is not None:
                sample, received = self._sample
                age = round((time.monotonic() - received) * 1000)
                if age <= 350:
                    return dict(sample, connected=True, state='streaming', data_age_ms=age)
        return dict(connected=False, state='waiting',
                    error='Waiting for fresh joint state from the PNP host')

    def close(self):
        self.set_idle(False)
        self._stop.set()
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                try:
                    self._process.stdin.close()
                except OSError:
                    self._process.terminate()
        self._thread.join(timeout=2)


class JointStreams:
    """Reference-count readers and pause idle FCI while a local owner controls."""

    def __init__(self):
        self.lock = threading.RLock()
        self.streams = {}
        self.blocked = {}

    def open(self, host, user, password, side, address):
        with self.lock:
            if address not in self.streams:
                stream = PnpJointStream(host, user, password, side, address,
                                        idle=address not in self.blocked)
                self.streams[address] = stream, 0
            stream, count = self.streams[address]
            self.streams[address] = stream, count + 1
            return stream

    def close(self, stream):
        with self.lock:
            _, count = self.streams[stream.address]
            if count == 1:
                stream.close()
                del self.streams[stream.address]
            else:
                self.streams[stream.address] = stream, count - 1

    def pause(self, address):
        with self.lock:
            count = self.blocked.get(address, 0)
            if not count and address in self.streams:
                self.streams[address][0].set_idle(False)
            self.blocked[address] = count + 1

    def resume(self, address):
        with self.lock:
            count = self.blocked.get(address, 0)
            if count <= 1:
                self.blocked.pop(address, None)
                if address in self.streams:
                    self.streams[address][0].set_idle(True)
            else:
                self.blocked[address] = count - 1

    @contextmanager
    def paused(self, addresses):
        paused = []
        try:
            for address in addresses:
                self.pause(address)
                paused.append(address)
            yield
        finally:
            for address in reversed(paused):
                self.resume(address)


def display_quaternion(up):
    """Rotate the measured upward direction in base coordinates onto display Z."""
    import numpy as np
    from scipy.spatial.transform import Rotation

    up = np.asarray(up, dtype=float)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-6:
        raise ValueError('Invalid FR3 display up direction')
    up /= np.linalg.norm(up)
    axis = np.cross(up, [0., 0., 1.])
    length = np.linalg.norm(axis)
    rotation = (Rotation.from_rotvec(axis * (math.atan2(length, up[2]) / length))
                if length > 1e-9 else Rotation.from_rotvec([math.pi if up[2] < 0 else 0., 0., 0.]))
    x, y, z, w = rotation.as_quat()
    return [float(w), float(x), float(y), float(z)]
