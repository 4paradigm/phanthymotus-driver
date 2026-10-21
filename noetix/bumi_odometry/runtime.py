"""ROS sidecar: shared camera -> RTAB-Map RGB-D -> body gyro EKF.

No USB SDK and no motion API are imported here. Each reset uses private topics
and TF frame names; queued old estimates cannot enter the replacement filter.
"""
from __future__ import annotations
from collections import OrderedDict
from copy import deepcopy
import json
import math
from pathlib import Path
import tempfile
import threading
import time
import uuid
from common.fusion_odometry import (Calibration, Health, GravityWindow, finite,
                                     quat_matrix, rotate, make_sample)
try:
    from .pipeline import ChildProcesses
except ImportError:
    from pipeline import ChildProcesses


def stamp(msg):
    return msg.header.stamp.sec*10**9+msg.header.stamp.nanosec


def estimate(msg, segment, require_twist=True):
    import numpy as np
    if msg.header.frame_id != segment+'_odom' or msg.child_frame_id != segment+'_base':
        raise ValueError('wrong fusion reference frames')
    p,q = msg.pose.pose.position, msg.pose.pose.orientation
    position=finite([p.x,p.y,p.z],3); quaternion=finite([q.x,q.y,q.z,q.w],4)
    quat_matrix(quaternion)
    for cov in ((msg.pose.covariance,msg.twist.covariance) if require_twist else (msg.pose.covariance,)):
        matrix=np.asarray(finite(list(cov),36)).reshape(6,6)
        if (not np.allclose(matrix,matrix.T,atol=1e-8) or np.min(np.linalg.eigvalsh(matrix)) < -1e-8
                or min(np.diag(matrix)) <= 0):
            raise ValueError('invalid estimate covariance')
    if max(msg.pose.covariance[i] for i in (0,7,14)) > .04:
        raise ValueError('position uncertainty exceeds 0.2 m standard deviation')
    v,w = msg.twist.twist.linear,msg.twist.twist.angular
    twist=finite([v.x,v.y,v.z,w.x,w.y,w.z],6)
    return dict(stamp_ns=stamp(msg), twist=twist,
                pose=dict(position=position, quaternion=quaternion,
                          frame_id='odom', child_frame_id='base_link',
                          covariance=list(msg.pose.covariance)),
                twist_covariance=list(msg.twist.covariance))


class FusionRuntime:
    def __init__(self, config=None):
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        self.node=Node('bumi_rgbd_imu_odometry')
        self.qos=qos_profile_sensor_data
        self.String=String
        self.lock=threading.RLock()
        self.config=dict(config or {})
        self.active=False; self.closed=False
        self.output=self.config.get('output_topic','/bumi_rgbd_imu_odometry/state/odom')
        self.publisher=self.node.create_publisher(String,self.output,self.qos)
        self.directory=tempfile.TemporaryDirectory(prefix='bumi-fusion-')
        self.children=ChildProcesses(self.directory.name)
        self.input_subs=[]; self.result_subs=[]; self.segment_pubs=[]
        self.source_namespace=None; self.calibration=None; self.estimate=None
        self._new_segment()
        self.node.create_timer(.05,self._guard(self.tick))

    def _guard(self, callback):
        def guarded(*args):
            with self.lock:
                try: return callback(*args)
                except Exception as exc:
                    self.health.fault(str(exc))
                    self.publish()
        return guarded

    def now(self): return self.node.get_clock().now().nanoseconds

    def _new_segment(self):
        self.segment='bumi_fusion_'+uuid.uuid4().hex
        self.health=Health(self.segment,'unconfigured',self.now())
        self.gravity=GravityWindow(); self.initial=None; self.estimate=None
        self.epochs={}; self.sequences={}; self.cache=OrderedDict(); self.images=OrderedDict()
        self.maps=None; self.optics=None
        self.visual_pending=OrderedDict(); self.diagnostic_pending=OrderedDict(); self.diagnostic_started=False
        self.last_filtered_monotonic=None

    def publish(self):
        sample=make_sample(self.health,self.now(),self.estimate)
        if not self.active:
            sample['vendor'].update(status='initializing',reason='sensor stopped')
            sample['pose']=None; sample['twist']=[None]*6
        elif self.calibration is None and not self.health.reason:
            sample['vendor'].update(status='initializing',reason='measured installation and IMU timing calibration required')
        self.publisher.publish(self.String(data=json.dumps(sample,allow_nan=False)))
        return sample

    def info(self):
        from common.odom import build_interface
        sample=self.publish()
        return dict(state='running' if self.active else 'idle', topic_out=[{'topic':self.output,'format':'state/odom',
                   'message_type':'std_msgs/msg/String'}],
                    odom_interface=build_interface(provides=['vx','vy','vz','wx','wy','wz'],rate_hz=20,pose_drift='unbounded'),
                    quality=sample['vendor'],source_namespace=self.source_namespace,
                    calibration_id=self.calibration.id if self.calibration else None)

    def start(self):
        if not self.active:
            self._new_segment(); self.active=True
            try:
                self.calibration=Calibration(self.config.get('calibration',{}))
                self.health.calibration_id=self.calibration.id
            except ValueError:
                self.calibration=None
        return self.info()

    def stop(self, reason='sensor stopped'):
        if self.active:
            self.health.fault(reason); self.publish()
        self.active=False
        self.children.stop()
        for sub in self.input_subs+self.result_subs: self.node.destroy_subscription(sub)
        for pub in self.segment_pubs: self.node.destroy_publisher(pub)
        self.input_subs=[]; self.result_subs=[]; self.segment_pubs=[]
        self.source_namespace=None
        return self.info()

    def reset(self):
        self.stop('coordinate reset requested')
        return self.start()

    def configure(self, args):
        permitted={'calibration','source_namespace'}
        proposed={k:v for k,v in args.items() if k in permitted}
        if 'calibration' in proposed:
            Calibration(proposed['calibration'])  # validate before interrupting old segment
        if 'source_namespace' in proposed:
            import re
            if not re.fullmatch('[A-Za-z_][A-Za-z0-9_]*',proposed['source_namespace']):
                raise ValueError('invalid source namespace')
        if all(self.config.get(k)==v for k,v in proposed.items()):
            return self.info()
        active=self.active
        self.stop('calibration or source configuration changed')
        self.config.update(deepcopy(proposed))
        return self.start() if active else self.info()

    def discover(self):
        """Only sensor inputs are discovered; actuator source selection uses graph binding."""
        names={topic[:-len('/odometry_input/imu')].lstrip('/') for topic,_ in
               self.node.get_topic_names_and_types() if topic.endswith('/odometry_input/imu')}
        wanted=self.config.get('source_namespace')
        if wanted:
            return wanted if wanted in names else None
        return next(iter(names)) if len(names)==1 else None

    def _subscribe(self, cls, topic, callback, destination):
        destination.append(self.node.create_subscription(cls,topic,self._guard(callback),self.qos))

    def bind_inputs(self, namespace):
        from sensor_msgs.msg import Image, CameraInfo
        from std_msgs.msg import String
        import message_filters
        self.source_namespace=namespace
        root='/'+namespace+'/odometry_input/'
        self._subscribe(String,root+'imu',self.imu,self.input_subs)
        self._subscribe(String,root+'camera_meta',self.metadata,self.input_subs)
        # Keep wrappers alive. ROS subscriptions are also explicitly removed on stop.
        self.filters=[message_filters.Subscriber(self.node,cls,root+name,qos_profile=self.qos)
                      for cls,name in ((Image,'rgb'),(Image,'depth'),(CameraInfo,'info'))]
        self.input_subs.extend(f.sub for f in self.filters)
        self.sync=message_filters.ApproximateTimeSynchronizer(self.filters,5,.02)
        self.sync.registerCallback(self._guard(self.rgbd))

    def _epoch(self, name, value):
        if not isinstance(value,str) or not value: raise ValueError('missing input epoch')
        if name in self.epochs and self.epochs[name] != value:
            raise ValueError(name+' input restarted; reset required')
        self.epochs[name]=value

    def imu(self, msg):
        from sensor_msgs.msg import Imu
        data=json.loads(msg.data); now=self.now(); c=self.calibration.data
        self._epoch('imu',data['epoch'])
        if data.get('source_stamp_ns') is not None or data.get('stamp_source') != 'host_read':
            raise ValueError('unexpected IMU time provenance')
        seq=data['sequence']
        if seq<=self.sequences.get('imu',-1): raise ValueError('IMU read sequence reset')
        self.sequences['imu']=seq
        begin,end=data['read_begin_ns'],data['read_end_ns']
        if end<begin or end-begin>50_000_000: raise ValueError('IMU SDK read took too long or clock reset')
        if not 0 <= now-end <= int(c['timing']['max_age_s']*1e9): raise ValueError('IMU read transport stale')
        t=int((begin+end)/2-c['timing']['offset_s']*1e9)
        q=finite(data['quaternion'],4); quat_matrix(q)
        a=finite(data['acceleration'],3)
        w=[v-b for v,b in zip(finite(data['gyro'],3),c['gyro_bias'])]
        body_r=quat_matrix(c['imu_to_body']['quaternion'])
        initial=self.gravity.observe(t,rotate(body_r,a),rotate(body_r,w))
        if self.initial is None and initial is not None: self.initial=initial
        self.health.observe('imu',t,now,True)
        self.health.details['timing']=deepcopy(c['timing'])
        if self.children.children:
            out=Imu(); out.header.frame_id=self.segment+'_imu'
            out.header.stamp.sec,out.header.stamp.nanosec=divmod(t,10**9)
            out.orientation_covariance[0]=-1.  # attitude is not fused twice
            out.angular_velocity.x,out.angular_velocity.y,out.angular_velocity.z=w
            out.linear_acceleration.x,out.linear_acceleration.y,out.linear_acceleration.z=a
            for i,v in zip((0,4,8),c['gyro_variance']): out.angular_velocity_covariance[i]=v
            self.imu_pub.publish(out)

    def metadata(self,msg):
        data=json.loads(msg.data)
        self._epoch('camera',data['epoch'])
        t=data['rgb_stamp_ns']; now=self.now()
        if abs(t-data['depth_stamp_ns'])>20_000_000: raise ValueError('RGB/depth skew')
        if not 0<=now-data['received_ns']<=250_000_000: raise ValueError('camera transport stale')
        for key in ('rgb','depth'):
            if data[key]['domain'] not in ('hardware_clock','system_time','global_time'):
                raise ValueError('unrecognized camera timestamp domain')
        self.cache[t]=data
        while len(self.cache)>5: self.cache.popitem(last=False)
        self._prepare(t)

    def rgbd(self,rgb,depth,info):
        t=stamp(rgb)
        if stamp(info)!=t or abs(t-stamp(depth))>20_000_000: raise ValueError('RGB-D/calibration time mismatch')
        if len({rgb.header.frame_id,depth.header.frame_id,info.header.frame_id})!=1:
            raise ValueError('aligned depth frame mismatch')
        self.images[t]=(rgb,depth,info)
        while len(self.images)>5: self.images.popitem(last=False)
        self._prepare(t)

    def _prepare(self,t):
        if t not in self.cache or t not in self.images: return
        meta=self.cache.pop(t); rgb,depth,info=self.images.pop(t)
        if stamp(depth)!=meta['depth_stamp_ns']: raise ValueError('metadata does not describe paired depth')
        self.health.observe('camera',t,self.now(),True)
        self.health.details['camera_time_provenance']=meta
        if not self.children.children: return
        import cv2
        import numpy as np
        from sensor_msgs.msg import Image
        if rgb.encoding!='bgr8' or depth.encoding!='32FC1' or depth.is_bigendian:
            raise ValueError('unsupported shared image encoding')
        if (rgb.width,rgb.height)!=(depth.width,depth.height) or (rgb.width,rgb.height)!=(info.width,info.height):
            raise ValueError('camera intrinsics dimensions mismatch')
        key=(info.width,info.height,tuple(info.k),tuple(info.d),info.distortion_model)
        if self.optics is not None and key!=self.optics: raise ValueError('camera intrinsics changed; reset required')
        self.optics=key
        if info.distortion_model!='plumb_bob': raise ValueError('unsupported distortion')
        k=np.asarray(finite(list(info.k),9)).reshape(3,3)
        if k[0,0]<=0 or k[1,1]<=0: raise ValueError('invalid camera focal length')
        if self.maps is None:
            self.maps=cv2.initUndistortRectifyMap(k,np.asarray(info.d),None,k,(info.width,info.height),cv2.CV_32FC1)
        def pixels(msg,dtype,channels):
            if msg.step<msg.width*channels*np.dtype(dtype).itemsize or len(msg.data)!=msg.height*msg.step:
                raise ValueError('invalid shared image stride or size')
            rows=np.frombuffer(bytes(msg.data),dtype=np.uint8).reshape(msg.height,msg.step)
            return rows[:,:msg.width*channels*np.dtype(dtype).itemsize].copy().view(dtype).reshape(msg.height,msg.width,channels).squeeze()
        color=cv2.remap(pixels(rgb,'u1',3),*self.maps,cv2.INTER_LINEAR)
        metres=cv2.remap(pixels(depth,'<f4',1),*self.maps,cv2.INTER_NEAREST)
        if np.count_nonzero(np.isfinite(metres)&(metres>.1)&(metres<10))<metres.size*.1:
            raise ValueError('insufficient usable depth')
        for image,data,encoding,bpp,pub in ((rgb,color,'bgr8',3,self.rgb_pub),(depth,metres,'32FC1',4,self.depth_pub)):
            out=Image(); out.header=deepcopy(image.header); out.header.frame_id=self.segment+'_optical'
            out.width,out.height,out.encoding,out.step=rgb.width,rgb.height,encoding,rgb.width*bpp
            out.data=data.tobytes(); pub.publish(out)
        info=deepcopy(info); info.header.frame_id=self.segment+'_optical'; info.d=[0.]*5
        self.info_pub.publish(info)

    def launch(self):
        from sensor_msgs.msg import Image,CameraInfo,Imu
        from nav_msgs.msg import Odometry
        from rtabmap_msgs.msg import OdomInfo
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        root='/'+self.segment
        self.rgb_pub=self.node.create_publisher(Image,root+'/rgb',self.qos)
        self.depth_pub=self.node.create_publisher(Image,root+'/depth',self.qos)
        self.info_pub=self.node.create_publisher(CameraInfo,root+'/info',self.qos)
        self.imu_pub=self.node.create_publisher(Imu,root+'/imu',self.qos)
        self.visual_pub=self.node.create_publisher(Odometry,root+'/visual_validated',self.qos)
        self.segment_pubs=[self.rgb_pub,self.depth_pub,self.info_pub,self.imu_pub,self.visual_pub]
        self.broadcaster=StaticTransformBroadcaster(self.node)
        transforms=[]
        for key,child in (('camera_to_body','optical'),('imu_to_body','imu')):
            t=TransformStamped(); t.header.frame_id=self.segment+'_base'; t.child_frame_id=self.segment+'_'+child
            t.header.stamp=self.node.get_clock().now().to_msg()
            v=self.calibration.data[key]
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=v['translation']
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=v['quaternion']
            transforms.append(t)
        self.broadcaster.sendTransform(transforms)
        self._subscribe(Odometry,root+'/visual',lambda msg:self.odometry('visual',msg),self.result_subs)
        self._subscribe(Odometry,root+'/filtered',lambda msg:self.odometry('filtered',msg),self.result_subs)
        self._subscribe(OdomInfo,root+'/diagnostic',self.diagnostic,self.result_subs)
        self.children.start(self.segment,self.initial,root+'/imu')

    def odometry(self,kind,msg):
        try: data=estimate(msg,self.segment,require_twist=kind=='filtered')
        except ValueError:
            if kind not in self.health.samples: return  # uninitialized ROS estimates
            raise
        if kind=='visual':
            self.visual_pending[data['stamp_ns']]=msg
            while len(self.visual_pending)>20: self.visual_pending.popitem(last=False)
            self._release_visual(data['stamp_ns'])
        if kind=='filtered':
            self.health.observe(kind,data['stamp_ns'],self.now(),True)
            if self.estimate is not None:
                dt=(data['stamp_ns']-self.estimate['stamp_ns'])/1e9
                shift=math.dist(data['pose']['position'],self.estimate['pose']['position'])
                if dt<=0 or shift>.03+1.5*dt: raise ValueError('filtered pose jumped or reset')
            self.estimate=data
            self.health.details['twist_covariance']=data['twist_covariance']
            self.last_filtered_monotonic=time.monotonic()

    def _release_visual(self,t):
        if t not in self.visual_pending or t not in self.diagnostic_pending:
            return
        msg=self.visual_pending.pop(t); self.diagnostic_pending.pop(t)
        self.health.observe('visual',t,self.now(),True)
        self.health.observe('diagnostic',t,self.now(),True)
        if not self.health.reason:
            self.visual_pub.publish(msg)

    def diagnostic(self,msg):
        valid=not msg.lost and msg.inliers>=10 and msg.matches>=msg.inliers
        if not valid and not self.diagnostic_started:
            return  # first registration has no motion estimate yet
        if not valid:
            self.health.details['fault_stamp_ns']=stamp(msg)
            self.health.fault('visual tracking lost or insufficient inliers')
            return
        self.diagnostic_started=True
        self.health.details['visual_diagnostic']=dict(inliers=msg.inliers,matches=msg.matches,lost=msg.lost)
        t=stamp(msg)
        self.diagnostic_pending[t]=msg
        while len(self.diagnostic_pending)>20: self.diagnostic_pending.popitem(last=False)
        self._release_visual(t)

    def tick(self):
        if self.closed: return
        if self.active and self.calibration is not None and not self.health.reason:
            if self.source_namespace is None:
                namespace=self.discover()
                if namespace: self.bind_inputs(namespace)
            if not self.children.children and self.initial is not None and 'camera' in self.health.samples:
                self.launch()
            if self.children.failed(): self.health.fault('RTAB-Map or EKF process exited')
            if self.last_filtered_monotonic is not None and time.monotonic()-self.last_filtered_monotonic>.5:
                self.health.fault('filtered receipt watchdog expired')
        self.publish()

    def close(self):
        with self.lock:
            if self.closed: return
            self.stop('service shutting down'); self.closed=True
            self.node.destroy_node(); self.directory.cleanup()
