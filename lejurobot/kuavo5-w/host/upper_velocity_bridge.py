"""Leased replacement for the absent factory two-master wheel bridge.

Executed on the chassis over SSH. The MPC remains the velocity producer;
this helper never switches modes, selects a mux input or enables motion.
NAVIGATION_SOURCE and the manual keyboard protocol/stop helpers are injected.
"""
import multiprocessing
import queue
import os
import sys
import json
import math
import time
import threading


def latest_put(channel, value):
    try:
        channel.put_nowait(value)
    except queue.Full:
        try:
            channel.get_nowait()
        except queue.Empty:
            pass
        try:
            channel.put_nowait(value)
        except queue.Full:
            pass


def upper_observer(request, velocities, statuses, stopping):
    # SSH process loss must not leave the observer child orphaned.
    import ctypes
    import signal
    parent = os.getppid()
    ctypes.CDLL(None).prctl(1, signal.SIGTERM)
    if os.getppid() != parent:
        return
    os.environ['ROS_MASTER_URI'] = request['upper_master']
    os.environ['ROS_IP'] = request['ros_ip']
    os.environ.pop('ROS_HOSTNAME', None)
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from leju_mobile_base_msgs.msg import BaseCmdVelStatus
    import rosgraph
    rospy.init_node('motus_upper_velocity_bridge_observer', anonymous=True, disable_signals=True,
                    disable_rosout=True, log_level=rospy.ERROR)
    publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
    if any(publishers.get(topic) != ['/nodelet_manager'] for topic in
           ('/enable_control_state','/enable_vel_control_state','/move_base/base_cmd_vel')):
        stopping.set()
        return
    lock, state = threading.Lock(), {}
    def enable(field, message):
        if (message._connection_header or {}).get('callerid') == '/nodelet_manager':
            with lock:
                state[field] = message.data is True
    def velocity(message):
        if (message._connection_header or {}).get('callerid') != '/nodelet_manager':
            stopping.set()
            return
        with lock:
            allowed = state.get('control') is True and state.get('velocity') is True
        values = [message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z]
        latest_put(velocities, (time.monotonic(), allowed, values))
    subscribers = [
        rospy.Subscriber('/enable_control_state', Bool, lambda m:enable('control', m), queue_size=1),
        rospy.Subscriber('/enable_vel_control_state', Bool, lambda m:enable('velocity', m), queue_size=1),
        rospy.Subscriber('/move_base/base_cmd_vel', Twist, velocity, queue_size=1, tcp_nodelay=True)]
    publisher = rospy.Publisher('/move_base/base_cmd_vel_status', BaseCmdVelStatus, queue_size=1, latch=False)
    try:
        while not stopping.wait(.02) and not rospy.is_shutdown():
            try:
                stamp, message = statuses.get_nowait()
            except queue.Empty:
                continue
            if time.monotonic()-stamp <= .3:
                publisher.publish(message)
    finally:
        stopping.set()
        for sub in subscribers:
            sub.unregister()
        publisher.unregister()


def bridge_values(sample, chassis, allowed, now):
    if not allowed or sample is None or chassis is None:
        return [0.]*6
    stamp, enabled, values = sample
    state, received = chassis
    if (not enabled or not 0 <= now-stamp <= .2 or not 0 <= now-received <= .3
            or state.manual_mode or state.mapping or state.soft_estop or state.estop
            or state.bumper or state.driver_error or not state.driver_enable):
        return [0.]*6
    if (len(values) != 6 or any(not math.isfinite(v) for v in values)
            or math.hypot(values[0], values[1]) > .8+1e-6 or abs(values[5]) > 1.+1e-6
            or any(abs(values[i]) > 1e-6 for i in (2,3,4))):
        raise RuntimeError('upper_bridge_invalid_velocity')
    return values


def run_bridge(request):
    import secrets
    import fcntl
    ownership = open('/tmp/motus-upper-velocity-bridge.lock', 'a')
    try:
        fcntl.flock(ownership, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('upper_bridge_already_owned') from None
    for path in ('/opt/ros/noetic/lib/python3/dist-packages','/opt/jaten/lib/python3/dist-packages'):
        sys.path.insert(0, path)
    # Fork BEFORE either process initializes rospy: one node per real master.
    context = multiprocessing.get_context('fork')
    velocities, statuses, stopping = context.Queue(1), context.Queue(1), context.Event()
    observer = context.Process(target=upper_observer, args=(request, velocities, statuses, stopping))
    observer.start()
    os.environ['ROS_MASTER_URI'] = request['master']
    os.environ['ROS_IP'] = request['ros_ip']
    os.environ.pop('ROS_HOSTNAME', None)
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from jaten_msgs.msg import RobotChassisState
    from leju_mobile_base_msgs.msg import BaseCmdVelStatus
    namespace = {'__name__':'motus_bridge_navigation'}
    exec(compile(NAVIGATION_SOURCE,'navigation_rpc.py','exec'),namespace)
    rospy.init_node('motus_upper_velocity_bridge', anonymous=True, disable_signals=True,
                    disable_rosout=True, log_level=rospy.ERROR)
    publisher = rospy.Publisher('/move_base/base_cmd_vel', Twist, queue_size=1, latch=False)
    latest, lock = [None], threading.Lock()
    def feedback(message):
        if (message._connection_header or {}).get('callerid') != '/jaten_behavior_manager':
            return
        with lock:
            latest[0] = (message, time.monotonic())
    def status(message):
        if (message._connection_header or {}).get('callerid') == '/leju_node':
            latest_put(statuses, (time.monotonic(), message))
    subscribers = [rospy.Subscriber('/jaten_behavior_manager/robot_state', RobotChassisState, feedback, queue_size=1),
        rospy.Subscriber('/move_base/base_cmd_vel_status', BaseCmdVelStatus, status, queue_size=1)]
    def publish(values):
        message = Twist()
        message.linear.x, message.linear.y, message.linear.z = values[:3]
        message.angular.x, message.angular.y, message.angular.z = values[3:]
        publisher.publish(message)
    try:
        deadline = time.monotonic()+5.
        while not publisher.get_num_connections():
            if time.monotonic() >= deadline or not observer.is_alive():
                raise RuntimeError('upper_bridge_chassis_subscriber_missing')
            time.sleep(.02)
        publish([0.]*6)
        nonce, seq, buffer, sample = secrets.token_hex(16), -1, bytearray(), None
        while sample is None:
            if time.monotonic() >= deadline or not observer.is_alive() or stopping.is_set():
                raise RuntimeError('upper_bridge_upper_publisher_missing')
            try:sample = velocities.get(timeout=.05)
            except queue.Empty:pass
        print(json.dumps({'state':'held','nonce':nonce}),flush=True)
        was_allowed = False
        while not stopping.is_set() and observer.is_alive():
            frame = receive(sys.stdin, .3, buffer)
            if (frame.get('nonce') != nonce or type(frame.get('seq')) is not int
                    or frame['seq'] <= seq or type(frame.get('allow')) is not bool):
                raise RuntimeError('upper_bridge_invalid_lease')
            seq = frame['seq']
            while True:
                try:sample = velocities.get_nowait()
                except queue.Empty:break
            with lock:
                values = bridge_values(sample, latest[0], frame['allow'], time.monotonic())
            # Do not continuously inject even zeros into a native navigation
            # task. Send one zero when relinquishing upper control, then keep
            # the data path silent until another upper lease owns the base.
            if frame['allow'] or was_allowed:
                publish(values)
            was_allowed = frame['allow']
            nonce = secrets.token_hex(16)
            print(json.dumps({'state':'ready','nonce':nonce,'seq':seq,'forwarded_nonzero':any(values)}),flush=True)
        raise RuntimeError('upper_bridge_observer_stopped')
    finally:
        try:
            publish([0.]*6)
            try:
                namespace['confirm_stationary'](rospy, Odometry)
            except Exception:
                dispatch = dispatch_proxy(namespace, rospy)
                if not dispatch(control_mode=0).success:
                    raise RuntimeError('upper_bridge_stop_rejected')
                namespace['confirm_software_stop'](rospy, True)
                namespace['confirm_stationary'](rospy, Odometry)
            print(json.dumps({'state':'stopped','stationary':True,'bridge_zero_sent':True}),flush=True)
        finally:
            stopping.set()
            observer.join(timeout=1.)
            if observer.is_alive():observer.terminate();observer.join(timeout=1.)
            for sub in subscribers:sub.unregister()
            publisher.unregister()


if __name__ == '__main__':
    try:
        run_bridge(json.loads(sys.stdin.buffer.readline(16385)))
    except Exception as exc:
        print(json.dumps({'state':'error','error':str(exc) if str(exc).startswith('upper_bridge_') else type(exc).__name__}),flush=True)
