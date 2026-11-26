"""Use a saved mapping origin to seed Cartographer while the chassis is stopped."""
import json
import math
import os
import sys
import time


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def separation(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1]), abs(math.remainder(a[2]-b[2], math.tau))


def stationary_hold(chassis, odom, now):
    """Chassis-local hold; upper enable_control_state lives on another master."""
    twist = odom.twist.twist
    values = (twist.linear.x, twist.linear.y, twist.angular.z)
    return (chassis.soft_estop is True
            and 0 <= now-odom.header.stamp.to_sec() <= .5
            and all(math.isfinite(v) and abs(v) < .01 for v in values))


def scan_with_transform(read_scan, transform, errors):
    # A new TF listener may not yet cover the first buffered laser frame.
    # Wait for a newer scan rather than substituting a transform at another time.
    deadline = time.monotonic() + 3.
    while time.monotonic() < deadline:
        scan = read_scan()
        try:
            return scan, transform('base_link', scan.header.frame_id, scan.header.stamp)
        except errors:
            continue
    return None, None


def estimate(score, anchor, current):
    """Search only around the saved starting pose. Keep a good live pose."""
    if score(current) >= .8:
        return 'already_aligned', current
    candidates = []
    for ix in range(-6, 7):
        for iy in range(-6, 7):
            for ia in range(-3, 4):
                pose = (anchor[0]+ix*.05, anchor[1]+iy*.05, anchor[2]+math.radians(ia*3))
                if separation(anchor, pose)[0] <= .31:
                    candidates.append((score(pose), pose))
    best, pose = max(candidates)
    def distinct(other):
        distance, angle = separation(other, pose)
        return distance > .20 or angle > math.radians(9)

    if best < .8 or any(value >= best-.05 and distinct(other) for value, other in candidates):
        return 'no_distinct_anchor_match', None
    return 'matched', pose


def main(request):
    os.environ['ROS_MASTER_URI'] = request['master']
    os.environ['ROS_IP'] = request['ros_ip']
    os.environ.pop('ROS_HOSTNAME', None)
    sys.path[:0] = ['/opt/ros/noetic/lib/python3/dist-packages', '/opt/jaten/lib/python3/dist-packages']
    import rospy
    import rosgraph
    import tf2_ros
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from jaten_msgs.msg import RobotChassisState
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import LaserScan

    rospy.init_node('motus_map_relocalize', anonymous=True, disable_signals=True, disable_rosout=True)
    transforms = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(transforms)

    def transform(parent, child, stamp=rospy.Time(0)):
        transform = transforms.lookup_transform(parent, child, stamp, rospy.Duration(1)).transform
        return transform.translation.x, transform.translation.y, yaw(transform.rotation)

    def stopped():
        chassis = rospy.wait_for_message('/jaten_behavior_manager/robot_state', RobotChassisState, timeout=1)
        odom = rospy.wait_for_message('/odom_encoder', Odometry, timeout=1)
        return stationary_hold(chassis, odom, time.time())

    if not stopped():
        return {'state': 'stationary_pause_required'}

    anchor = tuple(request['args']['anchor'][key] for key in ('x', 'y', 'yaw'))
    grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=2)
    scan, mounting = scan_with_transform(
        lambda: rospy.wait_for_message('/scan', LaserScan, timeout=1), transform,
        (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException))
    if scan is None:
        return {'state': 'scan_transform_unavailable'}
    info = grid.info
    if (grid.header.frame_id != 'map' or not .02 <= info.resolution <= .1
            or not 0 < len(grid.data) == info.width*info.height <= 2_000_000
            or not 0 <= (rospy.Time.now()-scan.header.stamp).to_sec() <= .5):
        return {'state': 'invalid_map_or_scan'}
    lx, ly, angle = mounting
    beams = []
    for i in range(0, len(scan.ranges), max(1, len(scan.ranges)//240)):
        radius = scan.ranges[i]
        if math.isfinite(radius) and max(.25, scan.range_min) <= radius <= min(5., scan.range_max):
            bearing = scan.angle_min+i*scan.angle_increment+angle
            beams.append((lx+radius*math.cos(bearing), ly+radius*math.sin(bearing)))
    if len(beams) < 80:
        return {'state': 'insufficient_scan'}

    ox, oy = info.origin.position.x, info.origin.position.y
    origin_yaw = yaw(info.origin.orientation)
    co, so = math.cos(origin_yaw), math.sin(origin_yaw)
    occupied = set()
    for offset, value in enumerate(grid.data):
        if value >= 65:
            column, row = offset % info.width, offset // info.width
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    occupied.add((column+dx, row+dy))

    def score(pose):
        x, y, heading = pose
        c, s = math.cos(heading-origin_yaw), math.sin(heading-origin_yaw)
        gx, gy = co*(x-ox)+so*(y-oy), -so*(x-ox)+co*(y-oy)
        count = sum((math.floor((gx+c*px-s*py)/info.resolution),
                     math.floor((gy+s*px+c*py)/info.resolution)) in occupied for px, py in beams)
        return count/len(beams)

    state, target = estimate(score, anchor, transform('map', 'base_link'))
    if state != 'matched':
        return {'state': state}
    subscribers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[1])
    if subscribers.get('/initialpose') != ['/cartographer_node']:
        return {'state': 'unsupported_localization_receiver'}
    publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    deadline = time.monotonic()+2
    while not publisher.get_num_connections() and time.monotonic() < deadline:
        time.sleep(.05)
    if publisher.get_num_connections() != 1 or not stopped():
        return {'state': 'relocalization_precondition_lost'}

    def publish(pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id, msg.header.stamp = 'map', rospy.Time.now()
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
        msg.pose.pose.orientation.z = math.sin(pose[2]/2)
        msg.pose.pose.orientation.w = math.cos(pose[2]/2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .10**2
        msg.pose.covariance[35] = math.radians(5)**2
        publisher.publish(msg)
        time.sleep(1)
        return transform('map', 'base_link')

    measured = publish(target)
    if score(measured) < .8:
        # Some JaTen builds add a frame offset when applying initialpose.
        # Correct a bounded, observed translation once while still stopped.
        dx, dy = target[0]-measured[0], target[1]-measured[1]
        distance, angle = separation(target, measured)
        if .05 < distance <= .35 and angle <= math.radians(4) and stopped():
            measured = publish((target[0]+dx, target[1]+dy, target[2]))
    distance, angle = separation(target, measured)
    if score(measured) >= .8 and distance <= .15 and angle <= math.radians(5) and stopped():
        return {'state': 'corrected', 'pose': list(measured)}
    return {'state': 'correction_unconfirmed'}


if __name__ == '__main__':
    try:
        print(json.dumps({'ok': True, 'result': main(json.load(sys.stdin))}))
    except Exception:
        print(json.dumps({'ok': True, 'result': {'state': 'relocalization_unavailable'}}))
