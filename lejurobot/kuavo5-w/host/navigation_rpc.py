"""Fixed native ROS operations, executed over authenticated SSH on the chassis.

Uses the factory-generated classes on THAT robot; no copied message ABI, daemon,
network listener, workspace modification or downloaded SDK is required.
"""
import json
import hashlib
from pathlib import Path
import math
import os
import sys
import time


def main(request):
    if request['operation'] == 'map_identity':
        name = request.get('args', {}).get('map_name')
        if (not isinstance(name, str) or not name or len(name) > 80
                or any(c in name for c in '/\\') or any(ord(c) < 32 for c in name)):
            raise ValueError('invalid_map_name')
        digest = hashlib.sha256()
        root = Path('/home/jt/jmap_config/jmaps').resolve()
        for suffix in ('.pgm', '.yaml'):
            path = (root / (name + suffix)).resolve()
            if path.parent != root or not 0 < path.stat().st_size <= 32_000_000:
                raise RuntimeError('invalid_grid_map_file')
            with path.open('rb') as stream:
                digest.update(suffix.encode())
                while True:
                    chunk = stream.read(65536)
                    if not chunk:
                        break
                    digest.update(chunk)
        return {'sha256': digest.hexdigest()}
    os.environ['ROS_MASTER_URI'] = request['master']
    os.environ['ROS_IP'] = request['ros_ip']
    os.environ.pop('ROS_HOSTNAME', None)
    for path in ('/opt/ros/noetic/lib/python3/dist-packages', '/opt/jaten/lib/python3/dist-packages'):
        sys.path.insert(0, path)
    import rospy
    import rosgraph
    import rosservice
    from leju_mobile_base_msgs.srv import MoveToTarget, CheckArrived, SetDispatchMode
    from leju_mobile_base_msgs.msg import MoveToTargetOptions
    from nav_msgs.msg import Odometry
    rospy.init_node('motus_navigation_rpc', anonymous=True, disable_signals=True,
                    disable_rosout=True, log_level=rospy.ERROR)
    names = {'navigate': '/move_base/move_to_target', 'check': '/move_base/check_arrived',
             'cancel': '/move_base/set_dispatch_mode'}
    types = {'navigate': MoveToTarget, 'check': CheckArrived, 'cancel': SetDispatchMode}
    operation = request['operation']
    args = request.get('args', {})
    # Verify the native cancellation and submission really have one owner.
    services = dict(rosgraph.Master('/motus_navigation_rpc').getSystemState()[2])
    owners = [services.get(name, []) for name in names.values()]
    if any(len(owner) != 1 for owner in owners) or len({owner[0] for owner in owners}) != 1:
        raise RuntimeError('native_navigation_services_need_one_common_owner')
    for action, name in names.items():
        if rosservice.get_service_type(name) != types[action]._type:
            raise RuntimeError('native_navigation_service_type_mismatch')
    if operation == 'probe':
        return {'owner': owners[0][0], 'services': names}
    if operation not in names:
        raise ValueError('unsupported_navigation_operation')
    proxy = rospy.ServiceProxy(names[operation], types[operation])
    if operation == 'navigate':
        options = MoveToTargetOptions()
        options.avoid_enabled = True  # Never expose an obstacle-disable switch.
        options.allow_rotation = True
        options.avoid_distance = .5
        options.linear_velocity = args['speed']
        options.angular_velocity = .3
        options.position_threshold = .1
        options.angle_threshold = .1
        response = proxy(x=args['x'], y=args['y'], theta=args['yaw'], options=options)
    elif operation == 'check':
        response = proxy(task_id=args['task_id'], blocking=False, timeout=0.)
    else:
        response = proxy(control_mode=2)  # SetDispatchMode.CANCEL_CURRENT_TASK
        if not response.success:
            raise RuntimeError('native_navigation_cancel_rejected')
        # A successful cancel response alone is not evidence of stopping.
        # Require new, distinctly stamped, stationary odometry spanning .3s.
        issued = time.time()
        end, since, previous = time.monotonic() + 5., None, None
        while time.monotonic() < end:
            msg = rospy.wait_for_message('/odom', Odometry, timeout=1.)
            stamp = msg.header.stamp.to_sec()
            if stamp <= issued or not 0 <= time.time() - stamp <= .5 or (previous is not None and stamp <= previous):
                continue
            previous = stamp
            twist = msg.twist.twist
            values = (twist.linear.x, twist.linear.y, twist.linear.z,
                      twist.angular.x, twist.angular.y, twist.angular.z)
            stationary = all(math.isfinite(v) and abs(v) <= .01 for v in values)
            since = stamp if stationary and since is None else since if stationary else None
            if since is not None and stamp - since >= .3:
                return {'success': True, 'stationary': True,
                        'completion_basis': 'native_cancel_and_fresh_stationary_odometry'}
        raise RuntimeError('native_navigation_stop_not_confirmed')
    return {name: getattr(response, name) for name in response.__slots__}


if __name__ == '__main__':
    try:
        print(json.dumps({'ok': True, 'result': main(json.load(sys.stdin))}), flush=True)
    except Exception:
        # No ROS peer messages, credentials or arbitrary paths in SSH errors.
        print(json.dumps({'ok': False, 'error': 'native_navigation_rpc_failed'}), flush=True)
