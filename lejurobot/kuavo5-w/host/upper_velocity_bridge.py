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


def upper_observer(request, velocities, statuses, stopping, ownership, acknowledged, remote_ready):
    # SSH process loss must not leave the observer child orphaned.
    import ctypes
    import signal
    parent = os.getppid()
    ctypes.CDLL(None).prctl(1, signal.SIGTERM)
    signal.signal(signal.SIGTERM,lambda *args:stopping.set())
    if os.getppid() != parent:
        return
    os.environ['ROS_MASTER_URI'] = request['upper_master']
    os.environ['ROS_IP'] = request['ros_ip']
    os.environ.pop('ROS_HOSTNAME', None)
    import rospy
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Bool
    from sensor_msgs.msg import Joy
    from leju_mobile_base_msgs.msg import BaseCmdVelStatus
    import rosgraph
    rospy.init_node('motus_upper_velocity_bridge_observer', anonymous=True, disable_signals=True,
                    disable_rosout=True, log_level=rospy.ERROR)
    publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
    if any(publishers.get(topic) != ['/nodelet_manager'] for topic in
           ('/enable_control_state','/enable_vel_control_state','/move_base/base_cmd_vel')):
        stopping.set()
        return
    if publishers.get('/joy') != ['/joy_node']:
        stopping.set()
        return
    gate = UpperInputGate(request['upper_master'])
    navigation = {'__name__':'remote_neutral_check'}
    exec(compile(NAVIGATION_SOURCE,'navigation_rpc.py','exec'),navigation)
    lock, state = threading.Lock(), {}
    def enable(field, message):
        if (message._connection_header or {}).get('callerid') == '/nodelet_manager':
            with lock:
                state[field] = message.data is True
    def velocity(message):
        if (message._connection_header or {}).get('callerid') != '/nodelet_manager':
            stopping.set()
            return
        values = [message.linear.x, message.linear.y, message.linear.z,
                  message.angular.x, message.angular.y, message.angular.z]
        now = time.monotonic()
        with lock:
            quiet = all(math.isfinite(v) and abs(v)<=1e-6 for v in values)
            if not quiet:state.pop('quiet_since',None)
            elif 'quiet_since' not in state:state['quiet_since']=now
            state['velocity_at']=now
            allowed = (state.get('control') is True and state.get('velocity') is True
                and acknowledged.value==ownership.value
                and (ownership.value!=2 or remote_ready.value==1))
        latest_put(velocities, (now, allowed, values, ownership.value))
    def joy(message):
        now=time.monotonic()
        neutral=navigation['remote_joy_neutral'](message,'/joy_node')
        with lock:
            state.update(joy_at=now,neutral=neutral)
            if not neutral:state.pop('neutral_since',None)
            elif 'neutral_since' not in state:state['neutral_since']=now
    subscribers = [
        rospy.Subscriber('/enable_control_state', Bool, lambda m:enable('control', m), queue_size=1),
        rospy.Subscriber('/enable_vel_control_state', Bool, lambda m:enable('velocity', m), queue_size=1),
        rospy.Subscriber('/move_base/base_cmd_vel', Twist, velocity, queue_size=1, tcp_nodelay=True),
        rospy.Subscriber('/joy',Joy,joy,queue_size=1,tcp_nodelay=True)]
    publisher = rospy.Publisher('/move_base/base_cmd_vel_status', BaseCmdVelStatus, queue_size=1, latch=False)
    try:
        while not stopping.wait(.02) and not rospy.is_shutdown():
            desired={0:'hold',1:'card',2:'remote'}[ownership.value]
            now=time.monotonic()
            with lock:
                neutral=(state.get('neutral') is True and now-state.get('joy_at',0.)<=.3
                    and now-state.get('neutral_since',now)>=.3)
                joy_fresh=now-state.get('joy_at',0.)<=.3
                quiet=(now-state.get('velocity_at',0.)<=.3
                    and now-state.get('quiet_since',now)>=.3)
            if desired=='remote' and remote_ready.value!=1 and not neutral:
                desired='hold'
            if desired=='remote' and not joy_fresh:
                desired='hold'
            if gate.owner!=desired or now-gate.checked_at>=.1:
                changed=gate.owner!=desired
                if changed:
                    acknowledged.value=-1
                    remote_ready.value=0
                    with lock:state.pop('quiet_since',None)
                    quiet=False
                gate.select(desired)
                acknowledged.value={'hold':0,'card':1,'remote':2}[desired]
            if desired=='remote' and neutral and quiet:
                remote_ready.value=1
            if desired!='remote':remote_ready.value=0
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
        try:gate.restore()
        except Exception:pass


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
    import fcntl
    with open('/tmp/motus-upper-velocity-bridge.lock', 'a') as ownership_lock:
        try:
            fcntl.flock(ownership_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('upper_bridge_already_owned') from None
        return run_owned_bridge(request)


def run_owned_bridge(request):
    import secrets
    for path in ('/opt/ros/noetic/lib/python3/dist-packages','/opt/jaten/lib/python3/dist-packages'):
        sys.path.insert(0, path)
    # Fork BEFORE either process initializes rospy: one node per real master.
    context = multiprocessing.get_context('fork')
    velocities, statuses, stopping = context.Queue(1), context.Queue(1), context.Event()
    ownership,acknowledged,remote_ready=context.Value('i',0),context.Value('i',-1),context.Value('i',0)
    observer = context.Process(target=upper_observer, args=(request, velocities, statuses, stopping,
                                                          ownership,acknowledged,remote_ready))
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
                    or frame['seq'] <= seq or type(frame.get('allow')) is not bool
                    or frame.get('owner') not in ('hold','card','remote')):
                raise RuntimeError('upper_bridge_invalid_lease')
            seq = frame['seq']
            ownership.value={'hold':0,'card':1,'remote':2}[frame['owner']]
            while True:
                try:sample = velocities.get_nowait()
                except queue.Empty:break
            with lock:
                matching=sample is not None and sample[3]==ownership.value and acknowledged.value==ownership.value
                permit=frame['allow'] and matching and (ownership.value!=2 or remote_ready.value==1)
                values = bridge_values(sample[:3] if sample else None, latest[0], permit, time.monotonic())
            # Do not continuously inject even zeros into a native navigation
            # task. Send one zero when relinquishing upper control, then keep
            # the data path silent until another upper lease owns the base.
            if frame['allow'] or was_allowed:
                publish(values)
            was_allowed = frame['allow']
            nonce = secrets.token_hex(16)
            print(json.dumps({'state':'ready','nonce':nonce,'seq':seq,'forwarded_nonzero':any(values),
                'input_owner':{0:'hold',1:'card',2:'remote'}.get(acknowledged.value),
                'remote_state':'ready' if remote_ready.value==1 else 'waiting_for_neutral' if ownership.value==2 else 'blocked'}),flush=True)
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
            if observer.is_alive():observer.kill();observer.join(timeout=1.)
            for sub in subscribers:sub.unregister()
            publisher.unregister()


if __name__ == '__main__':
    try:
        run_bridge(json.loads(sys.stdin.buffer.readline(16385)))
    except Exception as exc:
        print(json.dumps({'state':'error','error':str(exc) if str(exc).startswith('upper_bridge_') else type(exc).__name__}),flush=True)
