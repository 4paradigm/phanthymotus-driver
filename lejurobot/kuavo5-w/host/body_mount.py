"""Read-only articulated camera mount, from runtime URDF and measured joints.

The SDK root is its depth-stream origin, NOT the CAD mounting-screw origin.
The model head_camera_depth matches Gemini335L CAD depth origin. This is a
model estimate; neither installation nor cross-device clocks are calibrated.
"""
from collections import deque
import hashlib
import json
import math
import threading
import uuid
import xml.etree.ElementTree as ET

LINKS = (['knee_link', 'leg_link', 'waist_link', 'waist_yaw_link'] +
         [f'zarm_{side}{i}_link' for side in ('l', 'r') for i in range(1, 8)] +
         ['zhead_1_link', 'zhead_2_link'])


class Kinematics:
    def __init__(self, urdf):
        import numpy as np
        self.np = np
        if (not isinstance(urdf, str) or len(urdf) > 5_000_000 or
                '<!DOCTYPE' in urdf.upper() or '<!ENTITY' in urdf.upper()):
            raise ValueError('invalid runtime URDF')
        root = ET.fromstring(urdf)
        if root.tag != 'robot':
            raise ValueError('runtime description is not a robot')
        by_child = {}
        for j in root.findall('joint'):
            child = j.find('child').get('link')
            if child in by_child:
                raise ValueError('ambiguous model parent')
            by_child[child] = j
        self.indices = {by_child[link].get('name'): i for i, link in enumerate(LINKS)}
        chain, child = [], 'head_camera_depth'
        while child != 'base_link':
            if child in [j.find('child').get('link') for j in chain]:
                raise ValueError('cyclic camera chain')
            j = by_child[child]
            chain.append(j)
            child = j.find('parent').get('link')
        self.chain = []
        for j in reversed(chain):
            origin = j.find('origin')
            xyz = self.vector(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0')
            rpy = self.vector(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0')
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = (self.axis_rotation([0,0,1], rpy[2]) @
                                    self.axis_rotation([0,1,0], rpy[1]) @
                                    self.axis_rotation([1,0,0], rpy[0]))
            kind = j.get('type')
            if kind == 'fixed':
                index, axis = None, None
            elif kind in ('revolute', 'continuous'):
                index = self.indices[j.get('name')]
                a = j.find('axis')
                axis = self.vector(a.get('xyz') if a is not None else '1 0 0')
                if np.linalg.norm(axis) < 1e-9:
                    raise ValueError('zero joint axis')
            else:
                raise ValueError('unsupported camera chain joint')
            self.chain.append((transform, index, axis))
        # Exact CAD correspondence checked against Orbbec gemini335L_336L.
        depth = by_child['head_camera_depth']
        if (depth.get('type') != 'fixed' or depth.find('parent').get('link') != 'head_camera_base'
                or not np.allclose(self.vector(depth.find('origin').get('xyz')),
                                   [.011715, .0475, .014311], atol=1e-7, rtol=0)
                or not np.allclose(self.vector(depth.find('origin').get('rpy', '0 0 0')), 0)):
            raise ValueError('runtime camera model does not match Gemini335L depth origin')
        self.sha256 = hashlib.sha256(urdf.encode()).hexdigest()

    def vector(self, text):
        v = self.np.array([float(x) for x in text.split()])
        if v.shape != (3,) or not self.np.isfinite(v).all():
            raise ValueError('invalid model vector')
        return v

    def axis_rotation(self, axis, angle):
        np = self.np
        v = np.asarray(axis, dtype=float); v = v/np.linalg.norm(v)
        x,y,z = v
        k = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        return np.eye(3) + math.sin(angle)*k + (1-math.cos(angle))*(k@k)

    def forward(self, q):
        np = self.np
        q = np.asarray(q, dtype=float)
        if q.shape != (20,) or not np.isfinite(q).all():
            raise ValueError('expected 20 finite measured joint angles')
        result = np.eye(4)
        for origin, index, axis in self.chain:
            result = result @ origin
            if index is not None:
                motion = np.eye(4); motion[:3,:3] = self.axis_rotation(axis, q[index])
                result = result @ motion
        return result


def interpolate(samples, stamp_ns):
    """No extrapolation/latest substitution; bound both association and gaps."""
    for (a, qa), (b, qb) in zip(samples, list(samples)[1:]):
        if a <= stamp_ns <= b and 0 < b-a <= 50_000_000:
            if min(stamp_ns-a, b-stamp_ns) > 25_000_000:
                break
            f = (stamp_ns-a)/(b-a)
            return [x+(y-x)*f for x,y in zip(qa,qb)], (a,b)
    raise ValueError('capture has no bracketing measured joints within 50ms')


class BodyMount:
    def __init__(self):
        import rospy
        import tf2_ros
        from pathlib import Path
        from rosbridge_loopback import configure_messages
        configure_messages(Path(__file__).resolve().parent)
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        self.rospy, self.String = rospy, String
        self.lock = threading.Lock()
        self.samples = deque(maxlen=4096)
        self.model = None
        self.reason = 'waiting for runtime model and measured joints'
        self.conflict = False
        self.epoch = uuid.uuid4().hex
        self.last_tf = 0
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.publisher = rospy.Publisher('/motus/kuavo5w/body_mount', String, queue_size=4)
        self.subs = [rospy.Subscriber('/tf', TFMessage, self.check_authority, queue_size=20),
                     rospy.Subscriber('/tf_static', TFMessage, self.check_authority, queue_size=20)]
        self.joint_sub = None
        self.refresh(None)
        self.timer = rospy.Timer(rospy.Duration(2), self.refresh)

    def refresh(self, _):
        try:
            r = self.rospy
            if (r.get_param('/robot_version', None) != 63 or
                    [r.get_param('/'+k, None) for k in ('legRealDof','armRealDof','headRealDof','waistRealDof')] != [4,14,2,0]):
                raise ValueError('unsupported runtime 5-W joint layout')
            urdf = r.get_param('/humanoid_description')
            digest = hashlib.sha256(urdf.encode()).hexdigest()
            model = self.model if self.model and self.model.sha256 == digest else Kinematics(urdf)
            shared = r.get_param('/use_shm_communication', False)
            if type(shared) is not bool:
                raise ValueError('invalid joint transport configuration')
            topic = '/sensors_data_raw_shm' if shared else '/sensors_data_raw'
            if self.joint_sub is None or self.joint_sub.resolved_name != topic:
                from kuavo_msgs.msg import sensorsData
                if self.joint_sub:
                    self.joint_sub.unregister()
                self.joint_sub = r.Subscriber(topic, sensorsData, self.receive, queue_size=100, tcp_nodelay=True)
            with self.lock:
                if self.model is not model:
                    self.samples.clear(); self.epoch = uuid.uuid4().hex
                self.model, self.reason = model, None
        except Exception as e:
            with self.lock:
                self.model, self.reason = None, str(e)[:250]

    def check_authority(self, msg):
        if getattr(msg, '_connection_header', {}).get('callerid') == self.rospy.get_name():
            return
        if any(t.child_frame_id.lstrip('/') == 'camera_head_link' for t in msg.transforms):
            self.conflict = True  # Do not compete with another TF parent/authority.

    def receive(self, msg):
        from geometry_msgs.msg import TransformStamped
        from tf.transformations import quaternion_from_matrix
        try:
            ns = msg.header.stamp.to_nsec()  # v63 sensor_time is a cycle clock.
            age = (self.rospy.Time.now()-msg.header.stamp).to_sec()
            if not -.05 <= age <= .5:
                raise ValueError('stale controller read-completion timestamp')
            q = list(msg.joint_data.joint_q)
            if len(q) != 20 or not all(math.isfinite(x) for x in q):
                raise ValueError('invalid measured joint angles')
            with self.lock:
                if self.samples and ns <= self.samples[-1][0]:
                    return
                self.samples.append((ns,q))
                model = self.model
            if model is None or self.conflict or ns-self.last_tf < 20_000_000:
                return
            matrix = model.forward(q)
            t = TransformStamped(); t.header.stamp = msg.header.stamp
            t.header.frame_id, t.child_frame_id = 'base_link', 'camera_head_link'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = matrix[:3,3]
            quat = quaternion_from_matrix(matrix)
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w = quat
            self.broadcaster.sendTransform(t); self.last_tf = ns
        except Exception as e:
            self.rospy.logwarn_throttle(10, 'model camera mount: %s', str(e)[:200])

    def publish(self, header):
        from tf.transformations import quaternion_matrix
        r = self.rospy
        packet = dict(schema='motus.camera_mount/1', parent_frame='base_link', child_frame=header.frame_id,
                      mode='dynamic', length_unit='m', transform_direction='body_from_camera',
                      source='runtime_urdf_measured_joints_and_sdk_tf', version='kuavo_v63_model_v1',
                      epoch=self.epoch, stamp_ns=header.stamp.to_nsec(), stamp_source='measurement',
                      clock_id='orbbec_head_system_ros_header', accuracy_validated=False,
                      provenance={'mount':'Gemini335L CAD depth origin matched to live head_camera_depth',
                          'joint_time':'controller_ros_read_completion; interpolated; not calibrated acquisition',
                          'body_alignment':'vendor base_link convention; not installation-calibrated',
                          'clock_mapping_verified':False})
        try:
            with self.lock:
                model, samples = self.model, list(self.samples)
            if model is None:
                raise ValueError(self.reason)
            if self.conflict:
                raise ValueError('camera root already has another TF authority')
            if header.frame_id != 'camera_head_color_optical_frame':
                raise ValueError('unsupported camera optical frame')
            q, bracket = interpolate(samples, packet['stamp_ns'])
            internal = self.buffer.lookup_transform('camera_head_link', header.frame_id, header.stamp, r.Duration(0))
            t = internal.transform
            camera = quaternion_matrix([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
            camera[:3,3] = [t.translation.x,t.translation.y,t.translation.z]
            matrix = model.forward(q) @ camera
            if not model.np.isfinite(matrix).all():
                raise ValueError('invalid camera transform')
            packet.update(matrix=matrix.tolist(),status='available',reason=None,
                          runtime_urdf_sha256=model.sha256, joint_bracket_ns=list(bracket))
        except Exception as e:
            packet.update(status='unavailable', reason=str(e)[:250])
        self.publisher.publish(self.String(data=json.dumps(packet,allow_nan=False)))
