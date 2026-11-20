#!/usr/bin/env python3
"""Camera-owned gyro/accel packets, using SDK internal TF and original stamps."""
from collections import deque
import json
import math
import threading
import time
import uuid


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    if not all(math.isfinite(v) for v in (x, y, z, w)) or abs(x*x+y*y+z*z+w*w-1) > 1e-5:
        raise ValueError('invalid camera IMU TF rotation')
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


def make_packet(gyro, accel, transforms, epoch):
    gs, acs = gyro.header.stamp.to_nsec(), accel.header.stamp.to_nsec()
    if min(gs, acs) <= 0 or abs(gs-acs) > 20_000_000:
        raise ValueError('camera gyro/accel timestamps differ by more than 20ms')
    g, a = gyro.angular_velocity, accel.linear_acceleration
    if not all(math.isfinite(v) for v in (g.x, g.y, g.z, a.x, a.y, a.z)):
        raise ValueError('invalid camera IMU vectors')
    packet = dict(schema='motus.imu/1', source_id='/camera_head/gyro+accel', epoch=epoch,
                  stamp_ns=gs, accel_stamp_ns=acs, stamp_source='measurement',
                  clock_id='orbbec_head_system_ros_header',
                  timestamp_semantics='sdk_frame_system_timestamp',
                  clock_mapping=dict(verified=False, uncertainty_ns=None),
                  camera_frame_id='camera_head_color_optical_frame',
                  gyro_frame_id=gyro.header.frame_id, accel_frame_id=accel.header.frame_id,
                  angular_velocity=[g.x, g.y, g.z], linear_acceleration=[a.x, a.y, a.z],
                  units=dict(angular_velocity='rad/s', linear_acceleration='m/s^2'),
                  rotation_source='orbbec_sdk_internal_tf', translation_source='orbbec_sdk_internal_tf')
    for key, transform in zip(('gyro', 'accel'), transforms):
        packet['camera_from_'+key] = rotation(transform.transform.rotation)
        t = transform.transform.translation
        values = [t.x, t.y, t.z]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('invalid camera IMU origin')
        packet['camera_from_'+key+'_translation'] = values
    return packet


def take_samples(gyros, accels):
    """Drain bounded work, waiting for the accel watermark before association."""
    samples = []
    while gyros and accels and len(samples) < 32:
        gyro = gyros[0]
        ns = gyro.header.stamp.to_nsec()
        if accels[-1].header.stamp.to_nsec() < ns:
            break
        gyros.popleft()
        accel = min(accels, key=lambda m: abs(m.header.stamp.to_nsec()-ns))
        samples.append((gyro, accel))
        while len(accels) > 1 and accels[1].header.stamp.to_nsec() <= ns:
            accels.popleft()
    return samples


def main():
    import rospy
    import tf2_ros
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    rospy.init_node('motus_head_imu')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    publisher = rospy.Publisher('/motus/kuavo5w/camera_imu', String, queue_size=20)
    status_pub = rospy.Publisher('/motus/kuavo5w/camera_imu/status', String, queue_size=1, latch=True)
    lock = threading.Lock()
    gyros, accels = deque(maxlen=512), deque(maxlen=512)
    def receive(kind, message):
        with lock:
            (gyros if kind == 'gyro' else accels).append(message)
    subs = [rospy.Subscriber('/camera_head/'+kind+'/sample', Imu,
            lambda msg, k=kind: receive(k, msg), queue_size=100, tcp_nodelay=True)
            for kind in ('gyro', 'accel')]
    epoch, last, reason, received = uuid.uuid4().hex, 0, 'waiting for native camera gyro/accel', 0
    published_status, status_at = None, 0.
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        with lock:
            samples = take_samples(gyros, accels)
        # One sample per 200 Hz timer falls behind whenever delivery batches or
        # TF/JSON work overruns a tick. Drain available captures without changing
        # their timestamps or turning repeated readings into new measurements.
        for gyro, accel in samples:
            try:
                ns = gyro.header.stamp.to_nsec()
                if ns < last:
                    epoch, last = uuid.uuid4().hex, 0
                if ns == last:
                    continue
                transforms = [buffer.lookup_transform('camera_head_color_optical_frame', m.header.frame_id,
                                                       m.header.stamp, rospy.Duration(0)) for m in (gyro, accel)]
                packet = make_packet(gyro, accel, transforms, epoch)
                publisher.publish(String(data=json.dumps(packet, allow_nan=False)))
                last, reason, received = ns, None, received+1
            except Exception as error:
                reason = type(error).__name__+': '+str(error)[:250]
        now = time.monotonic()
        status = dict(state='available' if reason is None else 'unavailable', reason=reason,
                      packets=received, source='orbbec_camera_imu',
                      fusion=dict(state='available' if reason is None else 'unavailable', reason=reason))
        signature = (status['state'], reason)
        if signature != published_status or now-status_at >= 2:
            status_pub.publish(String(data=json.dumps(status)))
            published_status, status_at = signature, now
        rate.sleep()


if __name__ == '__main__':
    main()
