"""Opt-in estimated body TF. Read-only feedback; never claims measured calibration."""
import bisect
import uuid
from collections import deque
import json
import math
import os
from pathlib import Path
import select
import shlex
import subprocess
import threading
import time

import numpy as np

IDS = {'head': {1, 2, 3}, 'waist': {31, 32}, 'leg': {51, 52}}
BUDGET_NS = 50_000_000


def joints(m):
    return dict(head_roll_joint=m[1], head_pitch_joint=m[2], head_yaw_joint=m[3],
                waist_yaw_joint=m[31], waist_pitch_joint=m[32],
                first_leg_pitch_joint=m[52], second_leg_pitch_joint=m[51]-m[52])


def rotation(axis, angle):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    x, y, z = a
    k = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*k + (1-math.cos(angle))*(k@k)


def quaternion_matrix(q):
    x, y, z, w = q
    if not np.isfinite(q).all() or not np.isclose(np.dot(q, q), 1., atol=1e-5, rtol=0.):
        raise ValueError('invalid quaternion')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix_quaternion(r):
    # Eigenvector form remains stable at 180 degree rotations.
    k = np.array([[r[0,0]-r[1,1]-r[2,2], r[1,0]+r[0,1], r[2,0]+r[0,2], r[2,1]-r[1,2]],
                  [r[1,0]+r[0,1], r[1,1]-r[0,0]-r[2,2], r[2,1]+r[1,2], r[0,2]-r[2,0]],
                  [r[2,0]+r[0,2], r[2,1]+r[1,2], r[2,2]-r[0,0]-r[1,1], r[1,0]-r[0,1]],
                  [r[2,1]-r[1,2], r[0,2]-r[2,0], r[1,0]-r[0,1], np.trace(r)]])/3
    _, v = np.linalg.eigh(k)
    q = v[:, -1]
    return q if q[3] >= 0 else -q


class Estimator:
    def __init__(self, config, *, wall_ns=time.time_ns, mono_ns=time.monotonic_ns):
        self.config, self.wall_ns, self.mono_ns = config, wall_ns, mono_ns
        self.lock = threading.RLock()
        self.samples = {name: deque(maxlen=2048) for name in IDS}
        self.clock = None

    def calibrate(self, before, remote, after, elapsed):
        if elapsed <= 0 or elapsed > 8_000_000:
            raise ValueError('clock RTT exceeds 8 ms')
        if abs(after-before-elapsed) > 1_000_000:
            raise ValueError('host clock jumped during calibration')
        offset = remote-(before+after)//2
        with self.lock:
            if self.clock and abs(offset-self.clock['offset']) > 10_000_000:
                for rows in self.samples.values():
                    rows.clear()
            self.clock = dict(offset=offset, error=elapsed//2, mono=self.mono_ns(), wall=after)

    def observe(self, group, stamp, positions):
        with self.lock:
            rows = self.samples[group]
            try:
                if type(stamp) is not int or stamp <= 0 or set(positions) != IDS[group]:
                    raise ValueError('invalid joint identity or timestamp')
                if not all(math.isfinite(v) for v in positions.values()):
                    raise ValueError('nonfinite joint feedback')
                if rows and stamp < rows[-1][0]:
                    self.clock = None
                    for history in self.samples.values():
                        history.clear()
                    raise ValueError('joint timestamp order reset')
                if rows and stamp == rows[-1][0]:
                    return
            except ValueError:
                rows.clear()
                raise
            rows.append((stamp, dict(positions)))

    def at(self, stamp):
        with self.lock:
            c = self.clock
            age = self.mono_ns()-c['mono'] if c else -1
            if c is None or age < 0 or age > 2_000_000_000:
                raise ValueError('motion clock mapping expired')
            if abs(self.wall_ns()-c['wall']-age) > 2_000_000:
                raise ValueError('host clock jumped')
            if not -50_000_000 <= self.wall_ns()-stamp <= 250_000_000:
                raise ValueError('capture stale or future')
            error = c['error']+age//10000  # 100 ppm drift allowance until expiry.
            target = stamp+c['offset']
            motors, gap_max = {}, 0
            for rows in self.samples.values():
                times = [r[0] for r in rows]
                i = bisect.bisect_left(times, target)
                if i == 0 or i == len(rows):
                    raise ValueError('waiting for joint bracket')
                a, b = rows[i-1], rows[i]
                gap = b[0]-a[0]
                gap_max = max(gap_max, gap)
                if gap+error > BUDGET_NS:
                    raise ValueError('joint and clock synchronization budget exceeded')
                weight = (target-a[0])/gap
                motors.update({k: v+(b[1][k]-v)*weight for k, v in a[1].items()})
            q = joints(motors)
        t = np.eye(4)
        for entry in self.config['chain']:
            r, p, y = entry['rpy']
            a = np.eye(4)
            a[:3, 3] = entry['xyz']
            a[:3, :3] = rotation([0,0,1], y)@rotation([0,1,0], p)@rotation([1,0,0], r)
            if entry['type'] != 'fixed':
                a[:3, :3] = a[:3, :3]@rotation(entry['axis'], q[entry['joint']])
            t = t@a
        mount = self.config['head_from_optical']
        a = np.eye(4)
        a[:3, 3] = mount['position']
        a[:3, :3] = quaternion_matrix(mount['quaternion'])
        return t@a, error+gap_max


class MotionClock:
    """An SSH child answers read-only clock queries; no remote files or clock changes."""
    def __init__(self, estimator):
        self.estimator = estimator
        self.stop_event = threading.Event()
        self.reason = 'waiting for motion clock'
        self.worker = threading.Thread(target=self.run, daemon=True)

    def run(self):
        config = self.estimator.config
        server = 'import sys,time\nfor line in sys.stdin: print(time.time_ns(),flush=True)'
        command = ['ssh', '-T', '-o', 'StrictHostKeyChecking=accept-new',
                   '-o', 'ConnectTimeout=3', '-o', 'ServerAliveInterval=2',
                   config['motion_user']+'@'+config['motion_host'],
                   'python3 -u -c '+shlex.quote(server)]
        env = dict(os.environ)
        password = env.pop('MOTION_CLOCK_PASSWORD', None)
        if password:
            env['SSHPASS'] = password
            command = ['sshpass', '-e']+command
        else:
            command[1:1] = ['-o', 'BatchMode=yes']
        while not self.stop_event.is_set():
            process = None
            try:
                process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                           stderr=subprocess.DEVNULL, text=True, env=env)
                first = True
                while not self.stop_event.is_set():
                    before, mono = time.time_ns(), time.monotonic_ns()
                    process.stdin.write('\n')
                    process.stdin.flush()
                    deadline = time.monotonic()+(5. if first else .5)
                    first = False
                    while not select.select([process.stdout], [], [], .1)[0]:
                        if self.stop_event.is_set() or time.monotonic() > deadline:
                            raise RuntimeError('motion clock response timeout')
                    remote = int(process.stdout.readline())
                    after, elapsed = time.time_ns(), time.monotonic_ns()-mono
                    try:
                        self.estimator.calibrate(before, remote, after, elapsed)
                        self.reason = None
                    except ValueError as error:
                        self.reason = str(error)
                    self.stop_event.wait(.25)
            except (OSError, ValueError, RuntimeError) as error:
                self.reason = str(error)[:160]
            finally:
                if process is not None:
                    process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    process.stdin.close()
                    process.stdout.close()
            self.stop_event.wait(.5)

    def close(self):
        self.stop_event.set()
        self.worker.join(timeout=4)


def clock_pair(wall_ns=time.time_ns, mono_ns=time.monotonic_ns):
    # Bracket the wall read so thread preemption cannot masquerade as a clock step.
    for _ in range(3):
        before = mono_ns()
        wall = wall_ns()
        after = mono_ns()
        if 0 <= after-before <= 1_000_000:
            return wall, (before+after)//2
    raise ValueError('host clock sampling delayed')


class CaptureEvidence:
    """Exact-capture certificates; never fills rejected gaps through TF interpolation."""
    def __init__(self, config, *, wall_ns=time.time_ns, mono_ns=time.monotonic_ns):
        self.provenance = config['provenance']
        self.wall_ns, self.mono_ns = wall_ns, mono_ns
        self.last_clock = clock_pair(wall_ns, mono_ns)
        self.packets, self.epoch = {}, None
        self.retired_epoch = None
        self.lock = threading.Condition(threading.RLock())

    def _clock_check(self):
        try:
            wall, mono = clock_pair(self.wall_ns, self.mono_ns)
        except ValueError:
            self.packets.clear()
            return False
        stepped = abs((wall-self.last_clock[0])-(mono-self.last_clock[1])) > 2_000_000
        self.last_clock = wall, mono
        if stepped:
            self.packets.clear()
            self.retired_epoch = self.epoch
        return not stepped

    def observe(self, packet):
        with self.lock:
            if not self._clock_check():
                return
            if self.retired_epoch is not None and packet.get('epoch') == self.retired_epoch:
                return
            if packet.get('epoch') != self.epoch:
                self.packets.clear()
                self.epoch = packet.get('epoch')
            stamp = packet.get('stamp_ns')
            if type(stamp) is int:
                self.packets[stamp] = (packet, self.mono_ns())
                self.lock.notify_all()
                while len(self.packets) > 256:
                    del self.packets[min(self.packets)]

    def lookup(self, stamp, parent, child, clock):
        with self.lock:
            if not self._clock_check():
                raise ValueError('host clock jumped; capture evidence discarded')
            self.lock.wait_for(lambda: stamp in self.packets, timeout=.03)
            self._clock_check()
            entry = self.packets.get(stamp)
            if entry is None:
                raise ValueError('no exact estimated capture certificate; cached={} range={}..{} retired={}'.format(
                    len(self.packets), min(self.packets, default=0), max(self.packets, default=0),
                    self.retired_epoch == self.epoch and self.epoch is not None))
        p, received = entry
        if not 0 <= self.mono_ns()-received <= 250_000_000:
            raise ValueError('estimated capture certificate expired')
        if (p.get('schema') != 'motus.body_tf_estimate/1' or not p.get('epoch') or
                p.get('parent_frame') != parent or p.get('child_frame') != child or
                p.get('clock_id') != clock or p.get('provenance') != self.provenance):
            raise ValueError('estimated capture identity/provenance mismatch')
        bound = p.get('synchronization_bound_ns')
        if type(bound) is not int or not 0 <= bound <= BUDGET_NS:
            raise ValueError('estimated synchronization budget exceeded')
        if not -50_000_000 <= self.wall_ns()-stamp <= 250_000_000:
            raise ValueError('estimated capture certificate stale')
        a = np.asarray(p.get('matrix'), dtype=float)
        if (a.shape != (4,4) or not np.isfinite(a).all() or
                not np.allclose(a[3], [0,0,0,1], atol=1e-8) or
                not np.allclose(a[:3,:3].T@a[:3,:3], np.eye(3), atol=1e-5) or
                not np.isclose(np.linalg.det(a[:3,:3]), 1., atol=1e-5)):
            raise ValueError('invalid estimated rigid transform')
        return a.tolist()


def queue_capture(stamp, pending, seen):
    # Color can arrive after the slightly newer depth capture was published.
    # Deduplicate exact stamps, never impose a cross-stream arrival order.
    if stamp in seen:
        return 0
    seen.add(stamp)
    if len(seen) > 256:
        seen.remove(min(seen))
    pending.add(stamp)
    if len(pending) > 64:
        pending.remove(min(pending))
        return 1
    return 0


def main():
    import argparse
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from bodyctrl_msgs.msg import MotorStatusMsg
    from std_msgs.msg import String
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import Buffer, TransformListener, TransformBroadcaster
    from common.camera_mount import _matrix

    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default=str(Path(__file__).parent/'resource/body_tf_estimate.json'))
    args = parser.parse_args()
    config = json.loads(Path(args.config).read_text())
    if config['provenance']['calibration_kind'] != 'estimate' or config['provenance']['accuracy_validated']:
        raise ValueError('this source only supports explicitly unvalidated estimated calibration')
    estimator = Estimator(config)
    clock = MotionClock(estimator)
    rclpy.init(domain_id=0)
    node = Node('tianyi_estimated_body_tf')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    broadcaster = TransformBroadcaster(node)
    status_pub = node.create_publisher(String, '/tianyi_estimated_body_tf/status', 1)
    evidence_pub = node.create_publisher(String, '/tianyi_estimated_body_tf/captures', 100)
    epoch = [uuid.uuid4().hex]
    last_clock = list(clock_pair())
    pending, seen = set(), set()
    state = dict(status='unavailable', reason='waiting for capture and joints', published=0,
                 dropped=0, last_stamp_ns=0, max_sync_bound_ns=0,
                 provenance=config['provenance'], synchronization_budget_ns=BUDGET_NS)

    def feedback(msg, group):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        try:
            if any(s.error for s in msg.status):
                raise ValueError('motor feedback reports error')
            estimator.observe(group, stamp, {int(s.name): float(s.pos) for s in msg.status})
        except ValueError as error:
            with estimator.lock:
                estimator.samples[group].clear()
            state.update(status='unavailable', reason=str(error))

    def capture(msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        # The depth image may have its own frame; TF is computed for the declared optical frame.
        state['dropped'] += queue_capture(stamp, pending, seen)

    def publish_ready():
        try:
            wall, mono = clock_pair()
        except ValueError:
            return
        if abs((wall-last_clock[0])-(mono-last_clock[1])) > 2_000_000:
            pending.clear()
            seen.clear()
            state['last_stamp_ns'] = 0
            epoch[0] = uuid.uuid4().hex
            with estimator.lock:
                estimator.clock = None
                for rows in estimator.samples.values():
                    rows.clear()
        last_clock[:] = wall, mono
        for stamp in sorted(pending)[:8]:
            try:
                estimated, bound = estimator.at(stamp)
                native = buffer.lookup_transform(config['camera_root'], config['optical_frame'],
                    Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME))
                root_from_optical = np.array(_matrix(native, config['camera_root'], config['optical_frame'], stamp))
                body_from_root = estimated@np.linalg.inv(root_from_optical)
                msg = TransformStamped()
                msg.header.frame_id = config['body_frame']
                msg.child_frame_id = config['camera_root']
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
                t = body_from_root[:3, 3]
                q = matrix_quaternion(body_from_root[:3, :3])
                msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, t)
                msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
                broadcaster.sendTransform(msg)
                certificate = dict(schema='motus.body_tf_estimate/1', epoch=epoch[0],
                    stamp_ns=stamp, parent_frame=config['body_frame'], child_frame=config['optical_frame'],
                    clock_id='orbbec_head_global_ros_header', matrix=estimated.tolist(),
                    synchronization_bound_ns=bound, provenance=config['provenance'])
                evidence_pub.publish(String(data=json.dumps(certificate, allow_nan=False)))
                state.update(status='available', reason=None, last_stamp_ns=max(stamp, state['last_stamp_ns']),
                             synchronization_bound_ns=bound,
                             max_sync_bound_ns=max(bound, state['max_sync_bound_ns']))
                state['published'] += 1
                pending.remove(stamp)
            except Exception as error:
                state.update(status='unavailable', reason=str(error)[:240])
                if time.time_ns()-stamp > 200_000_000:
                    pending.remove(stamp)
                    state['dropped'] += 1
                else:
                    break

    def report():
        packet = dict(state, pending=len(pending), clock_reason=clock.reason)
        if time.time_ns()-state['last_stamp_ns'] > 250_000_000:
            packet.update(status='unavailable', reason=state['reason'] or 'capture TF is stale')
        with estimator.lock:
            if estimator.clock:
                packet['motion_clock'] = dict(estimator.clock)
                packet['motion_clock']['age_ns'] = time.monotonic_ns()-estimator.clock['mono']
        status_pub.publish(String(data=json.dumps(packet, allow_nan=False)))

    for group in IDS:
        node.create_subscription(MotorStatusMsg, '/'+group+'/status',
            lambda msg, group=group: feedback(msg, group), qos_profile_sensor_data)
    for stream in ('color', 'depth'):
        node.create_subscription(Image, '/ob_camera_head/'+stream+'/image_raw', capture, qos_profile_sensor_data)
    node.create_timer(.005, publish_ready)
    node.create_timer(1., report)
    clock.worker.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        clock.close()
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
