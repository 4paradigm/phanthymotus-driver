"""Optional capture-time TF evidence; never a camera owner or calibration claim."""
import json
import uuid

import numpy as np


def _matrix(transform, parent, child, stamp_ns):
    if transform.header.frame_id != parent or transform.child_frame_id != child:
        raise ValueError('TF response frames do not match requested body and optical frames')
    stamp = transform.header.stamp
    if stamp.sec * 10**9 + stamp.nanosec != stamp_ns:
        raise ValueError('TF response timestamp differs from capture timestamp; zero/latest TF is forbidden')
    t, q = transform.transform.translation, transform.transform.rotation
    translation = np.array([t.x, t.y, t.z], dtype=float)
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if (not np.isfinite(translation).all() or not np.isfinite(quaternion).all() or
            not np.isclose(quaternion @ quaternion, 1., rtol=0., atol=1e-5)):
        raise ValueError('TF must contain finite translation and a unit quaternion')
    x, y, z, w = quaternion / np.linalg.norm(quaternion)
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = translation
    return result.tolist()


class BodyMountSource:
    def __init__(self, node, root, *, body_frame, camera_frame, clock_id):
        self.node, self.topic = node, root + '/body_mount'
        self.body_frame, self.camera_frame, self.clock_id = body_frame, camera_frame, clock_id
        self.publisher = self.buffer = self.listener = None
        self._closed = False
        self._setup_reason = None
        self.reset()
        try:
            from std_msgs.msg import String
            from rclpy.qos import qos_profile_sensor_data
            self.publisher = node.create_publisher(String, self.topic, qos_profile_sensor_data)
        except Exception as error:
            self._setup_reason = 'body mount publisher unavailable: ' + str(error)
        try:
            from tf2_ros import Buffer, TransformListener
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, node)
        except Exception as error:
            reason = 'body mount tf2_ros unavailable: ' + str(error)
            self._setup_reason = (self._setup_reason + '; ' + reason) if self._setup_reason else reason
        if self._setup_reason:
            self._reason = self._setup_reason

    @staticmethod
    def _value(value):
        return value() if callable(value) else value

    def declaration(self):
        return dict(schema='motus.camera_mount/1', topic=self.topic,
                    parent_frame=self._value(self.body_frame), child_frame=self._value(self.camera_frame),
                    source='tf', version='capture_tf_v1', epoch=self.epoch, mode='dynamic',
                    length_unit='m', transform_direction='body_from_camera',
                    status='unavailable' if self._reason else 'available', reason=self._reason,
                    accuracy_validated=False)

    def status(self):
        return dict(self.declaration(), stamp_ns=self._stamp_ns)

    def publish(self, stamp_ns):
        if self._closed:
            return False
        # A failed lookup publishes a new unavailable packet, never a held matrix.
        self._reason = self._setup_reason
        self._stamp_ns = stamp_ns
        packet = None
        try:
            packet = self.declaration()
            packet.pop('topic')
            packet.update(stamp_ns=stamp_ns, clock_id=self._value(self.clock_id), stamp_source='measurement')
            if type(stamp_ns) is not int or stamp_ns <= 0:
                raise ValueError('positive integer capture timestamp required; latest TF is forbidden')
            if not packet['parent_frame'] or not packet['child_frame'] or not packet['clock_id']:
                raise ValueError('body frame, actual camera optical frame and capture clock are required')
            if packet['parent_frame'] == packet['child_frame']:
                raise ValueError('body frame must differ from camera optical frame')
            if self._setup_reason:
                raise RuntimeError(self._setup_reason)
            from rclpy.time import Time
            from rclpy.clock import ClockType
            capture_time = Time(nanoseconds=stamp_ns, clock_type=ClockType.ROS_TIME)
            transform = self.buffer.lookup_transform(packet['parent_frame'], packet['child_frame'], capture_time)
            packet['matrix'] = _matrix(transform, packet['parent_frame'], packet['child_frame'], stamp_ns)
        except Exception as error:
            self._reason = 'body mount unavailable: ' + str(error)
            if packet is not None:
                self._reason += ' ({} <- {})'.format(packet['parent_frame'], packet['child_frame'])
        try:
            packet.update(status='unavailable' if self._reason else 'available', reason=self._reason)
            from std_msgs.msg import String
            if self.publisher is None:
                return False
            self.publisher.publish(String(data=json.dumps(packet, allow_nan=False)))
        except Exception as error:
            self._reason = 'body mount publish unavailable: ' + str(error)
        return self._reason is None

    def reset(self):
        self.epoch = uuid.uuid4().hex
        self._stamp_ns = None
        self._reason = self._setup_reason or 'waiting for capture-time body-camera TF'

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._reason = 'body mount source stopped'
        if self.listener is not None:
            try:
                self.listener.unregister()
            except Exception:
                pass
        if self.publisher is not None:
            try:
                self.node.destroy_publisher(self.publisher)
            except Exception:
                pass
        self.listener = self.buffer = self.publisher = None
