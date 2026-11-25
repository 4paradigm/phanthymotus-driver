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


def g12_neutral(message, owner):
    """Verified v63 H12-to-Joy mapping; triggers 2/5 are not stick axes."""
    header = getattr(message, '_connection_header', {}) or {}
    axes, buttons = getattr(message, 'axes', ()), getattr(message, 'buttons', ())
    return (bool(owner) and header.get('callerid') == owner and header.get('latching') == '0'
            and len(axes) == 8 and len(buttons) == 11
            and all(type(v) in (int, float) and math.isfinite(v) and abs(v) <= 1.05 for v in axes)
            and all(abs(axes[i]) <= .03 for i in (0, 1, 3, 4, 6, 7))
            and all(type(v) is int and v == 0 for v in buttons))


def g12_owner(system_state, master, workspace):
    """Trust only the actual local SDK converter process, never a node-name guess."""
    publishers = dict(system_state[0]).get('/joy', [])
    if len(publishers) != 1:
        return None
    owner = publishers[0]
    code, _, uri = master.lookupNode('/kuavo5w_controller_boot', owner)
    if code != 1:
        return None
    with xmlrpc.client.ServerProxy(uri, transport=TimeoutTransport()) as node:
        code, _, pid = node.getPid('/kuavo5w_controller_boot')
    if code != 1 or type(pid) is not int or pid <= 0:
        return None
    expected = (Path(workspace) / 'src/humanoid-control/h12pro_controller_node/src/h12pro_node/h12ChannelToJoymsg.py').resolve()
    args = Path(f'/proc/{pid}/cmdline').read_bytes().decode().split('\0')
    return owner if any(arg.startswith('/') and Path(arg).resolve() == expected for arg in args) else None


def check_no_controller(system_state):
    """Detect controller publishers/services without mistaking our own subscribers for ownership."""
    if not isinstance(system_state, (list, tuple)) or len(system_state) != 3:
        raise RuntimeError('invalid_ros_master_system_state')
    publishers, subscribers, services = system_state
    for entries in (publishers, subscribers, services):
        for _, nodes in entries:
            if CONTROLLER_NODES.intersection(nodes):
                raise RuntimeError('existing_controller_owner')
    # A raw sensor subscriber is not evidence that the controller is running;
    # this boot monitor itself subscribes here while waiting for hardware.
    if any(name == '/sensors_data_raw' and nodes for name, nodes in publishers):
        raise RuntimeError('existing_controller_owner')


def controller_environment(environment):
    """Noninteractive systemd does not source the vendor's ~/.bashrc."""
    library = Path('/opt/drake/lib/libdrake.so')
    if not library.is_file():
        raise RuntimeError('missing_controller_runtime: /opt/drake/lib/libdrake.so')
    result = dict(environment)
    paths = [part for part in result.get('LD_LIBRARY_PATH', '').split(':') if part]
    result['LD_LIBRARY_PATH'] = ':'.join(['/opt/drake/lib', *(
        part for part in paths if part.rstrip('/') != '/opt/drake/lib')])
    return result


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
        try:
            self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        except OSError:
            self.boot_id = None  # No first-boot permit on an unsupported host.
        self.ready_at = None

    def status(self):
        return {'state': self.state, 'error': self.error, 'robot_version': self.version,
                'boot_id': self.boot_id,
                'ready_age_s': None if self.ready_at is None else max(0., time.monotonic()-self.ready_at),
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
                    self.ready_at = time.monotonic()
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
            self.ready_at = time.monotonic()


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
                except RuntimeError as exc:
                    if adopt_existing and str(exc) == 'existing_controller_owner':
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
    from sensor_msgs.msg import Joy
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
    def joy(msg):
        with lock:
            owner = measured.get('g12_owner')
        update(g12_neutral=g12_neutral(msg, owner), g12_at=time.monotonic())
    def monitor_g12_owner():
        # This optional XMLRPC/process probe must never stall the 5Hz boot
        # heartbeat or turn a joystick-node timeout into a controller fault.
        while not stop.is_set():
            try:
                with xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'], transport=TimeoutTransport()) as master:
                    code, _, system_state = master.getSystemState('/kuavo5w_g12_readonly')
                    owner = g12_owner(system_state, master,
                        os.environ.get('KUAVO_ROS_CONTROL_WS_PATH', '')) if code == 1 else None
            except Exception:
                owner = None
            with lock:
                if measured.get('g12_owner') != owner:
                    measured['g12_neutral'] = False
                measured.update(g12_owner=owner, g12_owner_at=time.monotonic())
            stop.wait(1.)
    threading.Thread(target=monitor_g12_owner, daemon=True, name='g12-source-probe').start()
    subscriptions = [
        rospy.Subscriber('/joy', Joy, joy, queue_size=1, tcp_nodelay=True),
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
        stdin=subprocess.DEVNULL, start_new_session=True, env=controller_environment(os.environ))
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
                    system_state = rosgraph.Master(rospy.get_name()).getSystemState()
                    services = dict(system_state[2])
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
            with lock:
                age = max(0., now-measured['g12_at']) if 'g12_at' in measured else None
                status.update(g12_owner=measured.get('g12_owner'), g12_age_s=age,
                    g12_neutral=bool(measured.get('g12_owner')) and measured.get('g12_neutral') is True
                        and age is not None and age <= .3
                        and 0 <= now-measured.get('g12_owner_at', 0) <= 3.)
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
