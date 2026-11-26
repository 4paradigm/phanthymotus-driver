"""Ephemeral, unprivileged factory-web keyboard guard on the chassis.

Received over the existing SSH stream; never installed or started at boot.
The watchdog lives beside the factory API and requests zero on browser, driver,
or SSH loss, with a separate ROS software-stop fallback if zero cannot be
confirmed. Failure of both paths remains blocked; physical stop is not assumed.
NAVIGATION_SOURCE is injected from the same image as this helper.
"""
import json
import math
import os
import select
import sys
import time
import base64
import hashlib
import socket
import struct
import threading

LEASE_S = .3
LINEAR_LIMIT, ANGULAR_LIMIT = .15, .3


class FactorySocket:
    """Small send-only RFC6455 client to the fixed loopback factory endpoint.

    No remote pip/package installation. Incoming broadcasts are drained; they
    are not treated as safety feedback (fresh ROS chassis state is required).
    """
    def __init__(self):
        self.socket = socket.create_connection(('127.0.0.1', 8888), timeout=.15)
        self.failed = threading.Event()
        self.write_lock = threading.Lock()
        try:
            key = base64.b64encode(os.urandom(16)).decode()
            request = (f'GET /ws/app{time.time_ns()} HTTP/1.1\r\nHost: 127.0.0.1:8888\r\n'
                       'Origin: http://127.0.0.1:8888\r\n'
                       f'Upgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n'
                       'Sec-WebSocket-Version: 13\r\n\r\n')
            self.socket.sendall(request.encode())
            response = bytearray()
            while not response.endswith(b'\r\n\r\n') and len(response) < 8192:
                part = self.socket.recv(1)
                if not part:
                    raise ConnectionError('factory_websocket_closed')
                response.extend(part)
            lines = response.decode('ascii').split('\r\n')
            headers = dict(line.split(':', 1) for line in lines[1:] if ':' in line)
            headers = {k.lower(): v.strip() for k, v in headers.items()}
            expected = base64.b64encode(hashlib.sha1((key+'258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
            if (lines[0].split()[1] != '101' or headers.get('sec-websocket-accept') != expected
                    or headers.get('upgrade', '').lower() != 'websocket'):
                raise RuntimeError('invalid_factory_websocket_handshake')
        except Exception:
            self.socket.close()
            raise
        self.reader = threading.Thread(target=self._drain, daemon=True)
        self.reader.start()

    def _drain(self):
        def read(size):
            data = bytearray()
            while len(data) < size and not self.failed.is_set():
                try:
                    part = self.socket.recv(size-len(data))
                except socket.timeout:
                    continue
                if not part:
                    raise EOFError('factory_websocket_closed')
                data.extend(part)
            if len(data) != size:
                raise EOFError('factory_websocket_closed')
            return data
        try:
            while not self.failed.is_set():
                head = read(2)
                opcode, length = head[0] & 15, head[1] & 127
                if head[1] & 128:
                    raise ValueError('masked_factory_server_frame')
                if length == 126:
                    length = struct.unpack('!H', read(2))[0]
                elif length == 127:
                    length = struct.unpack('!Q', read(8))[0]
                if length > 2_000_000:
                    raise ValueError('oversized_factory_frame')
                payload = read(length)
                if opcode == 8:
                    break
                if opcode == 9:
                    if len(payload) > 125:
                        raise ValueError('oversized_factory_ping')
                    self._write(10, payload)
                elif opcode == 1 and head[0] & 128:
                    value = json.loads(payload)
                    result = value.get('data', {}) if isinstance(value, dict) else {}
                    if isinstance(result, dict) and result.get('method') == 'RobotMotion' and result.get('error'):
                        raise RuntimeError('factory_motion_rejected')
        except Exception:
            pass
        finally:
            self.failed.set()

    def send(self, value):
        self._write(1, value.encode())

    def _write(self, opcode, data):
        if self.failed.is_set():
            raise ConnectionError('factory_websocket_disconnected')
        mask = os.urandom(4)
        if len(data) > 65535:
            raise ValueError('oversized_factory_command')
        length = bytes([0x80 | len(data)]) if len(data) < 126 else b'\xfe'+struct.pack('!H', len(data))
        masked = bytes(value ^ mask[i % 4] for i, value in enumerate(data))
        if not self.write_lock.acquire(timeout=.15):
            raise TimeoutError('factory_websocket_write_busy')
        try:
            self.socket.sendall(bytes([128 | opcode])+length+mask+masked)
        finally:
            self.write_lock.release()

    def close(self):
        self.failed.set()
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()
        self.reader.join(timeout=.3)


def validate_frame(frame, nonce, last_seq):
    if (not isinstance(frame, dict) or frame.get('nonce') != nonce
            or type(frame.get('seq')) is not int or frame['seq'] <= last_seq):
        raise ValueError('invalid_keyboard_lease')
    values = frame.get('values')
    if (not isinstance(values, list) or len(values) != 2
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
            or abs(values[0]) > LINEAR_LIMIT or abs(values[1]) > ANGULAR_LIMIT):
        raise ValueError('invalid_keyboard_velocity')
    return values


def receive(stream, timeout, data):
    # The caller owns this buffer for the entire connection. A parser timeout
    # must not discard a fragmented line, nor extend the motion lease.
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        newline = data.find(b'\n')
        if newline >= 0:
            if newline > 1024:
                raise ValueError('oversized_keyboard_frame')
            line = bytes(data[:newline])
            del data[:newline+1]
            return json.loads(line)
        if len(data) > 1024:
            raise ValueError('oversized_keyboard_frame')
        if not select.select([stream], [], [], max(0., deadline-time.monotonic()))[0]:
            break
        # Bound one read while retaining any following complete frames.
        part = os.read(stream.fileno(), 1025)
        if not part:
            raise EOFError('keyboard_stream_closed')
        data.extend(part)
    raise TimeoutError('keyboard_watchdog_expired')


def dispatch_proxy(namespace, rospy):
    """Resolve the chassis-local dispatcher with the verified vendor ABI."""
    import rosgraph
    import rosservice
    from leju_mobile_base_msgs.srv import SetDispatchMode
    name = '/move_base/set_dispatch_mode'
    headers = rosservice.get_service_headers(name, rosgraph.Master('/motus_keyboard').lookupService(name))
    kind = namespace['dispatch_service_class'](SetDispatchMode, headers.get('md5sum'))
    return rospy.ServiceProxy(name, kind)


def confirm_manual_stop(ws, namespace, rospy, Odometry, dispatch=None):
    """Normal zero/rest; independent ROS stop when the factory route fails."""
    zero_sent = False
    try:
        if ws is None:
            raise ConnectionError('factory_websocket_disconnected')
        ws.send(json.dumps({'method': 'RobotMotion', 'id': '1',
                            'params': {'vx': 0., 'vy': 0., 'vw': 0.}}))
        zero_sent = True
        namespace['confirm_stationary'](rospy, Odometry)
        if getattr(ws, 'failed', None) is not None and ws.failed.is_set():
            raise ConnectionError('factory_websocket_disconnected')
        return {'success': True, 'stationary': True, 'manual_zero_sent': True,
                'completion_basis': 'factory_manual_zero_and_fresh_stationary_odometry'}
    except Exception as exc:
        zero_error = guard_error_code(exc)
    # The failed WebSocket is never used for this fallback. A response alone
    # does not prove stopping: require the held flag before AND after rest.
    try:
        dispatch = dispatch if dispatch is not None else dispatch_proxy(namespace, rospy)
        if dispatch(control_mode=0).success is not True:
            raise RuntimeError('keyboard_stop_rejected')
        namespace['confirm_software_stop'](rospy, True)
        namespace['confirm_stationary'](rospy, Odometry)
        namespace['confirm_software_stop'](rospy, True)
    except Exception:
        raise RuntimeError('keyboard_fallback_stop_unconfirmed') from None
    return {'success': True, 'stationary': True, 'software_estop': True,
            'manual_zero_sent': zero_sent, 'stop_fallback': 'local_ros_software_stop',
            'zero_error': zero_error,
            'completion_basis': 'local_ros_software_stop_and_fresh_stationary_odometry'}


def manual_zero_stop(request):
    """Factory MANUAL stop: zero RobotMotion plus fresh physical rest."""
    namespace = {'__name__': 'motus_keyboard_navigation'}
    exec(compile(NAVIGATION_SOURCE, 'navigation_rpc.py', 'exec'), namespace)
    # A stop must reach RobotMotion even when the chassis is still moving.
    os.environ['ROS_MASTER_URI'] = request['master']
    os.environ['ROS_IP'] = request['ros_ip']
    os.environ.pop('ROS_HOSTNAME', None)
    for path in ('/opt/ros/noetic/lib/python3/dist-packages', '/opt/jaten/lib/python3/dist-packages'):
        if path not in sys.path:
            sys.path.insert(0, path)
    import rospy
    from jaten_msgs.msg import RobotChassisState
    from nav_msgs.msg import Odometry
    rospy.init_node('motus_manual_stop', anonymous=True, disable_signals=True,
                    disable_rosout=True, log_level=rospy.ERROR)
    state = rospy.wait_for_message('/jaten_behavior_manager/robot_state', RobotChassisState, timeout=2.)
    if state.manual_mode is not True:
        raise RuntimeError('keyboard_manual_stop_requires_manual_mode')
    try:
        ws = FactorySocket()
    except Exception:
        ws = None
    try:
        return confirm_manual_stop(ws, namespace, rospy, Odometry)
    finally:
        if ws is not None:
            ws.close()


def run(request):
    import secrets
    namespace = {'__name__': 'motus_keyboard_navigation'}
    exec(compile(NAVIGATION_SOURCE, 'navigation_rpc.py', 'exec'), namespace)
    # Mapping startup already cancelled native tasks and disabled upper output.
    # MANUAL idle uses RobotMotion zero, not a latched dispatcher soft stop.
    held = namespace['main'](dict(request, operation='stationary', args={}))
    if held.get('stationary') is not True:
        raise RuntimeError('keyboard_initial_stop_unconfirmed')
    import rospy
    from jaten_msgs.msg import RobotChassisState
    from nav_msgs.msg import Odometry
    dispatch = dispatch_proxy(namespace, rospy)

    latest = [None, 0.]
    def feedback(state):
        latest[:] = [state, time.monotonic()]
    subscription = rospy.Subscriber('/jaten_behavior_manager/robot_state', RobotChassisState, feedback, queue_size=1)
    feedback(rospy.wait_for_message('/jaten_behavior_manager/robot_state', RobotChassisState, timeout=2.))
    def manual():
        state, received = latest
        if time.monotonic()-received > LEASE_S:
            raise RuntimeError('keyboard_chassis_feedback_is_stale')
        if (state.manual_mode is not True or state.mapping is not True
                or state.estop or state.bumper or state.driver_error or not state.driver_enable):
            raise RuntimeError('keyboard_requires_healthy_manual_mapping')
        return state

    def emit(value):
        print(json.dumps(value), flush=True)

    ws, released = None, False
    try:
        manual()
        # Exactly the endpoint and method used by the installed factory page.
        ws = FactorySocket()
        def motion(vx=0., vw=0.):
            ws.send(json.dumps({'method': 'RobotMotion', 'id': '1',
                                'params': {'vx': vx, 'vy': 0., 'vw': vw}}))
        motion()
        namespace['confirm_stationary'](rospy, Odometry)
        nonce, seq = secrets.token_hex(16), -1
        emit({'state': 'held', 'nonce': nonce})
        # A new challenge per acknowledgement rejects queued/replayed frames.
        deadline = time.monotonic()+LEASE_S
        receive_buffer = bytearray()
        while True:
            frame = receive(sys.stdin, max(0., deadline-time.monotonic()), receive_buffer)
            vx, vw = validate_frame(frame, nonce, seq)
            seq = frame['seq']
            state = manual()
            if time.monotonic() >= deadline:
                raise TimeoutError('keyboard_command_expired')
            if not released:
                if vx or vw:
                    raise RuntimeError('keyboard_activation_requires_zero')
                motion()
                # Another factory client may have released the held chassis
                # while the lease/zero frame was in flight. Recheck fresh
                # feedback at the release boundary; never adopt that release.
                if state.soft_estop:
                    if manual().soft_estop is not True:
                        raise RuntimeError('keyboard_initial_stop_ownership_lost')
                    if not dispatch(control_mode=1).success:
                        raise RuntimeError('keyboard_release_rejected')
                namespace['confirm_software_stop'](rospy, False)
                namespace['confirm_stationary'](rospy, Odometry)
                released = True
                # Discard every direction selected during preparation.
                motion()
            else:
                if state.soft_estop:
                    raise RuntimeError('keyboard_software_stop_is_active')
                motion(vx, vw)
            nonce = secrets.token_hex(16)
            emit({'state': 'ready', 'nonce': nonce, 'seq': seq})
            deadline = time.monotonic()+LEASE_S
    finally:
        # Neither stop route depends on the failed SSH/browser link. The ROS
        # fallback is also independent of a broken factory WebSocket.
        failure = sys.exc_info()[1]
        try:
            reply = dict(confirm_manual_stop(ws, namespace, rospy, Odometry, dispatch), state='stopped')
            if failure is not None and not isinstance(failure, EOFError):
                reply['error'] = guard_error_code(failure)
            elif reply.get('zero_error'):
                reply['error'] = reply['zero_error']
            emit(reply)
        finally:
            try:
                if ws is not None:
                    ws.close()
            finally:
                subscription.unregister()


def guard_error_code(exc):
    code = str(exc)
    known = {'keyboard_initial_stop_unconfirmed', 'keyboard_chassis_feedback_is_stale',
        'keyboard_requires_healthy_manual_mapping', 'keyboard_activation_requires_zero',
        'keyboard_initial_stop_ownership_lost', 'keyboard_release_rejected',
        'keyboard_command_expired', 'keyboard_watchdog_expired',
        'keyboard_software_stop_is_active', 'keyboard_stop_rejected',
        'keyboard_manual_stop_requires_manual_mode', 'keyboard_zero_stop_send_unconfirmed',
        'keyboard_fallback_stop_unconfirmed',
        'invalid_factory_websocket_handshake', 'factory_websocket_closed',
        'factory_websocket_api_error', 'factory_motion_rejected',
        'factory_websocket_disconnected', 'factory_websocket_write_busy'}
    return code if code in known else 'factory_keyboard_guard_failed'


if __name__ == '__main__':
    try:
        # The transport's first line is a configuration, not a velocity frame.
        run(json.loads(sys.stdin.buffer.readline(16385)))
    except Exception as exc:
        print(json.dumps({'state': 'error', 'error': guard_error_code(exc)}), flush=True)
