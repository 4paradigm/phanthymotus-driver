#!/usr/bin/env python3
"""Lower-host boot owner: launch once, initialize once, confirm software pause.

Installed explicitly on the commissioned ROS1 controller host, not in the
driver container. Never sends enable_control=true or retries initialization.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client

STATUS_PATH = Path('/run/kuavo5w-controller/status.json')
EXPECTED_DOF = {'robot_type': 1, 'armRealDof': 14, 'headRealDof': 2,
                'legRealDof': 4, 'waistRealDof': 0}
CONTROLLER_NODES = {'/nodelet_manager', '/nodelet_controller',
                    '/mobile_manipulator_sqp_mpc_node'}


def check_no_controller(system_state):
    """Fail closed on stale registrations as well as live competing owners."""
    for entries in system_state:
        for name, nodes in entries:
            if CONTROLLER_NODES.intersection(nodes) or (name == '/sensors_data_raw' and nodes):
                raise RuntimeError('existing_controller_owner')


class BootSequence:
    """Pure state machine; service handles remain retained across timeouts."""
    def __init__(self, version, request, existing=False):
        self.version, self.request = version, request
        self.existing = existing
        self.state, self.error = 'waiting_hardware', None
        self.pending = None
        self.issued = None
        self.initialization_requested = False
        self.initialization_accepted = False
        self.pause_confirmed = False
        self.initial_mode = None

    def status(self):
        return {'state': self.state, 'error': self.error, 'robot_version': self.version,
                'initialization_requested': self.initialization_requested,
                'initialization_accepted': self.initialization_accepted,
                'boot_pause_confirmed': self.pause_confirmed,
                'initial_control_mode': self.initial_mode, 'operating_mode': 'BaseArm',
                'launch_ownership': 'external' if self.existing else 'service',
                'current_control_state': 'query_enable_control_state', 'auto_resume': False}

    def tick(self, now, sample):
        if self.state in ('complete', 'failed'):
            return
        expected = {**EXPECTED_DOF, 'robot_version': self.version}
        if not all(type(sample.get(k)) is int and sample[k] == v for k, v in expected.items()):
            if self.initialization_requested:
                self.state, self.error = 'failed', 'controller_identity_changed'
            return
        raw_at = sample.get('raw_at', float('-inf'))
        fresh_raw = sample.get('raw_valid') is True and 0 <= now - raw_at <= .5
        if self.state == 'waiting_hardware':
            if self.existing:
                # Installation can observe a manually commissioned controller
                # without killing/restarting it or sending initialization.
                # Only an already initialized, freshly measured pause qualifies.
                if (fresh_raw and sample.get('pause_service_valid') and sample.get('is_ready') == 1
                        and sample.get('enabled') is False
                        and 0 <= now - sample.get('enabled_at', float('-inf')) <= 2):
                    self.pause_confirmed, self.state, self.error = True, 'complete', None
                else:
                    self.error = 'existing_controller_not_initialized_and_paused'
                return
            if not fresh_raw or sample.get('ready_to_start') != 1 or not sample.get('services_valid'):
                return
            self.initialization_requested = True
            self.issued, self.state = now, 'initializing'
            self.pending = self.request('initialize')
            return
        if now - self.issued > 120:
            # Do not retry or discard an in-flight request: it may take effect
            # late. Keep waiting, and pause only after its terminal response
            # AND post-request initialized feedback have both arrived.
            self.error = 'initialization_timeout_waiting' if self.state == 'initializing' else 'pause_timeout_waiting'
        if not self.pending.done.is_set():
            return
        if self.pending.error or getattr(self.pending.response, 'success', None) is not True:
            self.state, self.error = 'failed', 'initialization_unconfirmed' if self.state == 'initializing' else 'pause_unconfirmed'
            return
        if self.state == 'initializing':
            self.initialization_accepted = True
            mode_at = sample.get('mode_at', float('-inf'))
            if (not fresh_raw or sample.get('is_ready') != 1 or
                    mode_at < self.issued or not 0 <= now - mode_at <= 2 or
                    type(sample.get('mode')) is not int or sample['mode'] != 3):
                return
            self.initial_mode = 3  # Confirm actual BaseArm feedback, not service acceptance.
            self.issued, self.state, self.error = now, 'pausing', None
            self.pending = self.request('pause')
        elif (sample.get('enabled') is False and sample.get('enabled_at', float('-inf')) >= self.issued
              and 0 <= now - sample['enabled_at'] <= 2):
            self.pause_confirmed = True
            self.state, self.error = 'complete', None


class Pending:
    def __init__(self, callback):
        self.done, self.response, self.error = threading.Event(), None, False
        def run():
            try:
                self.response = callback()
            except Exception:
                self.error = True
            finally:
                self.done.set()
        threading.Thread(target=run, daemon=True, name='kuavo-boot-service').start()


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2
        return connection


def wait_for_master(stop, timeout=180, adopt_existing=False):
    deadline = time.monotonic() + timeout
    while not stop.is_set() and time.monotonic() < deadline:
        try:
            with xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'], transport=TimeoutTransport()) as master:
                code, _, state = master.getSystemState('/kuavo5w_controller_boot_preflight')
            if code == 1:
                try:
                    check_no_controller(state)
                except RuntimeError:
                    if adopt_existing:
                        return True
                    raise
                return False
        except (OSError, xmlrpc.client.Error):
            pass
        stop.wait(1)
    raise RuntimeError('ros_master_unavailable')


def run(version, adopt_existing=False):
    stop = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: stop.set())
    existing = wait_for_master(stop, adopt_existing=adopt_existing)
    if stop.is_set():
        return
    # Import the actual lower-host workspace classes, never the upper app's
    # incompatible generated messages. The systemd unit sources that workspace.
    import rospy
    import rosgraph
    from std_msgs.msg import Bool, Int8, String
    from std_srvs.srv import Trigger, SetBool
    from kuavo_msgs.msg import sensorsData
    rospy.init_node('kuavo5w_controller_boot', disable_signals=True)
    pub = rospy.Publisher('/kuavo5w/controller_boot_state', String, queue_size=1, latch=True)
    lock, measured = threading.Lock(), {}
    def update(**values):
        with lock:
            measured.update(values)
    def raw(msg):
        q = msg.joint_data.joint_q
        valid = len(q) == 20 and all(math.isfinite(v) for v in q)
        update(raw_valid=valid, raw_at=time.monotonic())
    subscriptions = [
        rospy.Subscriber('/sensors_data_raw', sensorsData, raw, queue_size=1, tcp_nodelay=True),
        rospy.Subscriber('/mobile_manipulator/lb_mpc_control_mode', Int8,
            lambda msg: update(mode=msg.data, mode_at=time.monotonic()), queue_size=1),
        rospy.Subscriber('/enable_control_state', Bool,
            lambda msg: update(enabled=msg.data, enabled_at=time.monotonic()), queue_size=1)]
    def request(action):
        if action == 'initialize':
            return Pending(lambda: rospy.ServiceProxy('/humanoid_controller/real_initial_start', Trigger)())
        return Pending(lambda: rospy.ServiceProxy('/enable_control', SetBool)(False))
    sequence = BootSequence(version, request, existing)
    next_params, params, params_at, last_state = 0, {}, float('-inf'), None
    # Publish the startup gate before launching hardware, including to bridges
    # that connect later through latched redelivery.
    pub.publish(String(data=json.dumps(sequence.status())))
    child = None if existing else subprocess.Popen(['roslaunch', '--wait', 'humanoid_controllers',
        'load_kuavo_real_wheel.launch', 'joystick_type:=h12', 'start_way:=auto'],
        stdin=subprocess.DEVNULL, start_new_session=True)
    failure_pause = None
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            now = time.monotonic()
            if child is not None and child.poll() is not None:
                sequence.state, sequence.error = 'failed', 'controller_process_exited'
                pub.publish(String(data=json.dumps(sequence.status())))
                raise RuntimeError('controller_process_exited_no_restart')
            if now >= next_params and sequence.state != 'failed':
                try:
                    candidate = {k: rospy.get_param('/' + k, None) for k in (*EXPECTED_DOF, 'robot_version')}
                    candidate.update(is_ready=rospy.get_param('/hardware/is_ready', None),
                                  ready_to_start=rospy.get_param('/hardware/ready_to_start', None))
                    services = dict(rosgraph.Master(rospy.get_name()).getSystemState()[2])
                    candidate['pause_service_valid'] = services.get('/enable_control') == ['/nodelet_manager']
                    candidate['services_valid'] = all(services.get(name) == ['/nodelet_manager'] for name in
                        ('/enable_control', '/humanoid_controller/real_initial_start'))
                    params, params_at = candidate, now
                except Exception:
                    pass  # Do not renew a failed/slow identity read's 3s lease.
                next_params = time.monotonic() + 1
            now = time.monotonic()
            with lock:
                sample = {**(params if now - params_at <= 3 else {}), **measured}
            sequence.tick(time.monotonic(), sample)
            if sequence.state == 'complete' and (now - sample.get('raw_at', float('-inf')) > 3
                    or now - params_at > 3 or not all(params.get(k) == v
                        for k, v in {**EXPECTED_DOF, 'robot_version': version}.items())):
                sequence.state, sequence.error = 'failed', 'controller_feedback_or_identity_lost'
            if sequence.state == 'failed' and child is not None and failure_pause is None:
                failure_pause = request('pause')  # Best effort, never report this as confirmed.
            status = sequence.status()
            STATUS_PATH.parent.mkdir(parents=True, exist_ok=True)
            temporary = STATUS_PATH.with_suffix('.tmp')
            temporary.write_text(json.dumps(status) + '\n')
            temporary.replace(STATUS_PATH)
            pub.publish(String(data=json.dumps(status)))
            state = (sequence.state, sequence.error)
            if state != last_state:
                print('[controller-boot] ' + json.dumps(status), flush=True)
                last_state = state
            stop.wait(.2)
    finally:
        # A service stop is explicit lifecycle shutdown, not a recovery loop.
        # Try the software pause before asking our own launch tree to exit.
        # It is not a replacement for the robot's hardware emergency stop.
        if child is not None and child.poll() is None:
            pending = request('pause')
            pending.done.wait(3)
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=20)
            except subprocess.TimeoutExpired:
                print('[controller-boot] controller_shutdown_pending', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-version', type=int, required=True)
    parser.add_argument('--adopt-existing-paused', action='store_true')
    args = parser.parse_args()
    if args.robot_version != 63:
        parser.error('Only the commissioned Kuavo 5-W version63 is supported')
    run(args.robot_version, args.adopt_existing_paused)
