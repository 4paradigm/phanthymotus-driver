"""Saved-graph binding and atomic motus.odom/1 distance feedback."""
from __future__ import annotations
from collections import deque
from copy import deepcopy
import json
import math
import threading
import time
from common.fusion_odometry import finite, quat_matrix


class DistanceFeedback:
    def __init__(self,executor,*,on_fault=None,now_ns=None,monotonic=None):
        self.executor=executor; self.on_fault=on_fault or (lambda reason:None)
        self.now_ns=now_ns or time.time_ns; self.monotonic=monotonic or time.monotonic
        self.lock=threading.RLock(); self.node=None
        self.managed=False; self.binding=None; self.deadline=0.; self.issued=-1; self.generation=0
        self.topic=''; self.sample=None; self.received=0.; self.fault=None
        self.binding_error=None; self.failed_segment=None; self.wait_reason='initializing'
        self.stopped=threading.Event(); self.thread=None

    def _subscribe(self,topic):
        if not topic: return
        if self.executor is None: raise ValueError('feedback requires ROS executor')
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import String
        node=Node('bumi_destination_odometry')
        try:
            generation = self.generation
            node.create_subscription(String,topic,lambda msg: self._message(msg,generation),qos_profile_sensor_data)
            self.executor.add_node(node); self.node=node
        except Exception:
            node.destroy_node(); raise

    def _message(self,msg,generation):
        with self.lock:
            if generation != self.generation:
                return  # already dispatched callback from a removed subscription
            try: self.update(json.loads(msg.data))
            except Exception as exc:
                self.fault = self.fault or str(exc)
                self.on_fault(self.fault)

    def _unsubscribe(self):
        if self.node is not None:
            self.executor.remove_node(self.node); self.node.destroy_node(); self.node=None

    def apply(self,binding):
        with self.lock:
            self.managed=True
            try:
                now=self.now_ns()
                if binding.get('version')!=1 or not isinstance(binding.get('revision'),str) or not binding['revision']:
                    raise ValueError('missing platform connection revision')
                issued,expires=binding['issued_ns'],binding['expires_ns']
                if not all(type(v) is int for v in (issued,expires)) or issued<=self.issued:
                    raise ValueError('binding out of order')
                if not -50_000_000<=now-issued<=1_000_000_000 or not 0<expires-now<=1_050_000_000 or not 0<expires-issued<=1_000_000_000:
                    raise ValueError('binding lease expired or invalid')
                if binding.get('resolved') is not True: raise ValueError('saved odometry connection unresolved or ambiguous')
                topic=binding.get('input_topic')
                if not isinstance(topic,str) or (topic and not topic.startswith('/')):
                    raise ValueError('invalid saved odometry topic')
                previous=self.binding
                changed=previous is not None and (previous['revision']!=binding['revision'] or topic!=self.topic)
                if changed:
                    self.on_fault('feedback connection changed')
                if topic!=self.topic or changed:
                    self.generation += 1
                    self._unsubscribe(); self.sample=None; self.fault=None; self.failed_segment=None
                    self._subscribe(topic)
                self.topic=topic; self.binding=deepcopy(binding); self.issued=issued
                self.binding_error=None
                self.deadline=self.monotonic()+min(1.,(expires-now)/1e9)
                return self.info()
            except Exception as exc:
                self.deadline=0; self.binding_error=str(exc); self.on_fault(str(exc)); raise

    def info(self):
        return dict(feedback_binding_required=True,
                    feedback_binding=deepcopy(self.binding),
                    topic_in=[dict(format='state/odom',message_type='std_msgs/msg/String',**({'topic':self.topic} if self.topic else {}))])

    def update(self,data):
        with self.lock:
            try:
                if data.get('schema')!='motus.odom/1' or data.get('frame')!='body': raise ValueError('unsupported odometry schema/frame')
                if data.get('units')!={'linear':'m/s','angular':'rad/s','length':'m'}: raise ValueError('odometry units mismatch')
                v=data['vendor']; p=data['pose']; now=self.now_ns()
                if v['status']=='initializing' and self.sample is None and self.fault is None:
                    self.wait_reason=str(v.get('reason') or 'initializing')
                    return
                if v['status']!='tracking':
                    self.failed_segment=v.get('segment_id')
                    raise ValueError('odometry '+str(v['status'])+': '+str(v.get('reason')))
                if self.fault and self.failed_segment and self.failed_segment!=v['segment_id']:
                    self.fault=None; self.sample=None; self.failed_segment=None
                if v['feedback_source']!='rgbd_imu_ekf' or not v['segment_id'] or not v['calibration_id']:
                    raise ValueError('missing fusion provenance')
                if not 0<=now-v['published_ns']<=500_000_000 or not 0<=now-v['visual_stamp_ns']<=500_000_000:
                    raise ValueError('odometry visual evidence stale')
                if type(data['stamp_ms']) is not int or not 0<=now-data['stamp_ms']*1_000_000<=500_000_000:
                    raise ValueError('odometry pose time stale')
                if p['frame_id']!='odom' or p['child_frame_id']!='base_link': raise ValueError('wrong body pose reference')
                finite(p['position'],3); quat_matrix(p['quaternion']); finite(data['twist'],6)
                cov=finite(p['covariance'],36)
                if any(not 0<cov[i]<=.04 for i in (0,7,14)): raise ValueError('odometry position uncertainty invalid')
                if self.sample is not None:
                    old=self.sample
                    if (v['segment_id'],v['calibration_id'])==(old['vendor']['segment_id'],old['vendor']['calibration_id']):
                        if data['stamp_ms']<old['stamp_ms']: raise ValueError('odometry timestamp reset')
                        if data['stamp_ms']==old['stamp_ms']:
                            if data['pose']!=old['pose']: raise ValueError('pose changed without new time')
                            return  # a repeated publish cannot refresh the estimate
                    else:
                        self.on_fault('odometry segment or calibration changed')
                        self.fault=None  # new action may use the new segment; old context cannot
                self.sample=deepcopy(data); self.received=self.monotonic()
            except Exception as exc:
                self.fault=self.fault or str(exc); self.on_fault(self.fault)

    def freeze(self):
        with self.lock:
            context=dict(source='rgbd_imu_ekf' if self.topic else 'imu_integrated_experimental',
                         revision=self.binding['revision'] if self.binding else None,
                         segment=None,calibration_id=None)
            if self.topic and self.sample:
                context.update(segment=self.sample['vendor']['segment_id'],calibration_id=self.sample['vendor']['calibration_id'])
            self.validate(context)
            return context

    def validate(self,context):
        with self.lock:
            if self.managed:
                if self.binding is None or self.monotonic()>=self.deadline:
                    raise ValueError('feedback binding lease expired')
                if context['revision']!=self.binding['revision']: raise ValueError('feedback connection changed')
            if ('rgbd_imu_ekf' if self.topic else 'imu_integrated_experimental')!=context['source']:
                raise ValueError('feedback source changed')
            if self.binding_error: raise ValueError(self.binding_error)
            if self.fault: raise ValueError(self.fault)
            if self.topic:
                if self.sample is None: raise ValueError('connected odometry unavailable: '+self.wait_reason)
                v=self.sample['vendor']
                if context['segment']!=v['segment_id'] or context['calibration_id']!=v['calibration_id']:
                    raise ValueError('odometry segment changed')
                age=self.monotonic()-self.received
                if not 0<=age<=.5 or not 0<=self.now_ns()-v['visual_stamp_ns']<=500_000_000:
                    raise ValueError('odometry feedback stale')
                return deepcopy(self.sample)
            return None

    def start_watchdog(self,context_provider):
        if self.thread is not None: return
        def loop():
            while not self.stopped.wait(.02):
                context=context_provider()
                if context is not None:
                    try: self.validate(context)
                    except Exception as exc: self.on_fault(str(exc))
        self.thread=threading.Thread(target=loop,daemon=True,name='bumi_feedback_watchdog'); self.thread.start()

    def invalidate(self,reason):
        with self.lock:
            if self.managed:
                self.deadline=0
                self.on_fault(reason)

    def close(self):
        self.stopped.set()
        if self.thread: self.thread.join(timeout=1)
        with self.lock: self._unsubscribe()


def run_fused_distance(owner,distance,context):
    """Direct SDK control; errors, cancellation and lease expiry always brake."""
    started=time.monotonic(); moved=0.; brake=None; quiet=deque()
    result=dict(state='error',feedback_source='rgbd_imu_ekf',accuracy_validated=False,
                requested_distance_m=distance,segment_id=context['segment'],binding_revision=context['revision'])
    try:
        origin=owner._distance_feedback.validate(context)
        p0=origin['pose']['position']; q=origin['pose']['quaternion']; r=quat_matrix(q)
        norm=math.hypot(r[0][0],r[1][0])
        if norm<.9: raise ValueError('starting body forward axis is not approximately horizontal')
        c,s=r[0][0]/norm,r[1][0]/norm
        previous=origin; last_progress=0.; progress_at=started
        while True:
            now=time.monotonic()
            if owner._distance_fault: raise ValueError(owner._distance_fault)
            if owner._rotate_stop_event.is_set(): result['state']='stopped'; break
            problem=owner._rotate_mode_error()
            if problem: raise ValueError(problem['error'])
            sample=owner._distance_feedback.validate(context)
            dt=(sample['stamp_ms']-previous['stamp_ms'])/1000
            if dt<0: raise ValueError('odometry time reset')
            p=sample['pose']['position']; dx,dy=p[0]-p0[0],p[1]-p0[1]
            moved=dx*c+dy*s
            rb=quat_matrix(sample['pose']['quaternion']); yaw=math.atan2(rb[1][0],rb[0][0])
            yaw0=math.atan2(s,c); error=math.atan2(math.sin(yaw0-yaw),math.cos(yaw0-yaw))
            if abs(-dx*s+dy*c)>.15 or abs(error)>math.radians(20): raise ValueError('distance trajectory deviated')
            if dt>0:
                speed=math.dist(p,previous['pose']['position'])/dt
                if speed>1.5: raise ValueError('odometry pose jump or excessive speed')
                if brake is not None: quiet.append((sample['stamp_ms'],moved))
                previous=sample
            if now-started>15+15*distance: raise ValueError('fused distance timed out')
            remaining=distance-moved
            if remaining<-.05: raise ValueError('estimated overshoot exceeds 0.05m')
            if brake is None:
                if moved-last_progress>=.01: last_progress,progress_at=moved,now
                elif now-progress_at>3: raise ValueError('no odometry progress')
                vx=max(0.,sample['twist'][0])
                if remaining<=max(.025,min(.10,vx*.15)): brake=now
                else: owner._distance_publish(.6 if remaining>.2 else .5,max(-.4,min(.4,1.5*error-.1*sample['twist'][5])))
            if brake is not None:
                owner._distance_publish(0)
                while len(quiet)>1 and quiet[-1][0]-quiet[1][0]>=600: quiet.popleft()
                if len(quiet)>=5 and quiet[-1][0]-quiet[0][0]>=600:
                    values=[row[1] for row in quiet]
                    if max(values)-min(values)<=.015 and abs(values[-1]-values[0])/((quiet[-1][0]-quiet[0][0])/1000)<=.01:
                        if abs(remaining)>.05: raise ValueError('settled outside estimated target band')
                        result['state']='done'; break
                if now-brake>4: raise ValueError('odometry endpoint did not settle')
            time.sleep(.02)
        result['estimated_quality']=deepcopy(sample['vendor'])
    except Exception as exc:
        result['error']=str(exc)
        result['estimated_quality']={'status':'lost','reason':str(exc)}
    finally: owner._distance_publish(0)
    result.update(estimated_distance_m=moved,estimated_error_m=distance-moved,elapsed_s=time.monotonic()-started)
    return result
