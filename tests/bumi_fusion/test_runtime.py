"""Run real lifecycle code; replace only ROS transport and process boundaries."""
import json
import sys
import threading
from types import SimpleNamespace as NS,ModuleType
import pytest
from test_contract import calibration
from noetix.bumi_odometry.runtime import FusionRuntime
from noetix.bumi_odometry.service import SensorService


@pytest.fixture
def runtime(monkeypatch):
    class String:
        def __init__(self,data=''): self.data=data
    class Node:
        def __init__(self,*args): self.messages=[]; self.time=10**10; self.removed=[]
        def get_clock(self): return NS(now=lambda:NS(nanoseconds=self.time))
        def create_publisher(self,cls,topic,qos): return NS(publish=self.messages.append)
        def create_timer(self,*args): pass
        def destroy_subscription(self,sub): self.removed.append(sub)
        def destroy_publisher(self,pub): self.removed.append(pub)
        def destroy_node(self): pass
        def get_topic_names_and_types(self): return [('/robot/odometry_input/imu',['std_msgs/msg/String'])]
    for name,attrs in [('rclpy.node',{'Node':Node}),('rclpy.qos',{'qos_profile_sensor_data':object()}),
                       ('std_msgs.msg',{'String':String})]:
        mod=ModuleType(name);mod.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,mod)
    r=FusionRuntime()
    yield r
    r.close()


def test_missing_calibration_is_visible_and_never_launches_children(runtime):
    r=runtime;r.start()
    for _ in range(10): r.tick()
    msg=json.loads(r.node.messages[-1].data)
    assert msg['vendor']['status']=='initializing' and 'calibration' in msg['vendor']['reason']
    assert msg['pose'] is None and not r.children.children


def test_reset_uses_new_segment_and_reaps_only_its_children(runtime):
    r=runtime; r.start(); segment=r.segment
    r.reset()
    assert r.segment!=segment and not r.children.children
    samples=[json.loads(m.data) for m in r.node.messages]
    assert any(m['vendor']['segment_id']==segment and m['vendor']['status']=='lost' for m in samples)


def test_invalid_config_preserves_prior_calibration_and_service_state(runtime,tmp_path):
    r=runtime;r.config['calibration']=calibration();r.start()
    service=SensorService(r,tmp_path/'config.json')
    with pytest.raises(ValueError): service.dispatch({'action':'config','calibration':{}})
    assert r.active and r.calibration.id=='fixture'
    service.dispatch({'action':'config','source_namespace':'robot'})
    assert json.loads((tmp_path/'config.json').read_text())['source_namespace']=='robot'


def test_startup_lost_diagnostic_cannot_brick_initialization(runtime):
    r=runtime;r.config['calibration']=calibration();r.start()
    msg=NS(lost=True,inliers=0,matches=0,header=NS(stamp=NS(sec=10,nanosec=0)))
    r.diagnostic(msg)
    assert r.health.status(10**10)['status']=='initializing'
    msg.lost=False;msg.inliers=20;msg.matches=30;r.diagnostic(msg)
    msg.lost=True;r.diagnostic(msg)
    assert r.health.status(10**10)['status']=='lost'


def test_ekf_never_receives_visual_pose_before_matching_diagnostic(runtime,monkeypatch):
    from noetix.bumi_odometry import runtime as module
    r=runtime;r.config['calibration']=calibration();r.start()
    forwarded=[];r.visual_pub=NS(publish=forwarded.append)
    r.segment_pubs=[]
    msg=NS(header=NS(stamp=NS(sec=10,nanosec=0)))
    monkeypatch.setattr(module,'estimate',lambda *args,**kwargs:{'stamp_ns':10**10})
    r.odometry('visual',msg)
    assert forwarded==[]
    diag=NS(lost=False,inliers=20,matches=30,header=msg.header)
    r.diagnostic(diag)
    assert forwarded==[msg]


def test_reapplying_same_saved_configuration_does_not_reset_an_active_segment(runtime):
    r=runtime;r.config['calibration']=calibration();r.start();old=r.segment
    r.configure({'calibration':calibration()})
    assert r.segment==old and r.active


def test_launch_tf_maps_nonidentity_sensor_coordinates_into_body(runtime,monkeypatch):
    """TF parent B / child S stores T_B_S: p_B = R_B_S p_S + t_B_S.

    Expected points/axes below are literal +90 degree examples, independent of
    the production geometry helpers. Inverting either mount must fail.
    """
    import math
    import numpy as np
    emitted=[]
    class TransformStamped:
        def __init__(self):
            self.header=NS(frame_id='',stamp=None); self.child_frame_id=''
            self.transform=NS(translation=NS(x=0.,y=0.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=1.))
    class Broadcaster:
        def __init__(self,node): pass
        def sendTransform(self,transforms): emitted.extend(transforms)
    for name,attrs in [('sensor_msgs.msg',{'Image':object,'CameraInfo':object,'Imu':object}),
                       ('nav_msgs.msg',{'Odometry':object}),('rtabmap_msgs.msg',{'OdomInfo':object}),
                       ('geometry_msgs.msg',{'TransformStamped':TransformStamped}),
                       ('tf2_ros',{'StaticTransformBroadcaster':Broadcaster})]:
        mod=ModuleType(name);mod.__dict__.update(attrs);monkeypatch.setitem(sys.modules,name,mod)
    r=runtime; c=calibration(); h=math.sqrt(.5)
    c['camera_to_body']=dict(translation=[.4,0,.2],quaternion=[0,0,h,h])  # +90 Z
    c['imu_to_body']=dict(translation=[-.1,.3,.2],quaternion=[h,0,0,h])  # +90 X
    r.config['calibration']=c; r.start(); r.initial=(0.,0.)
    monkeypatch.setattr(r.node,'get_clock',lambda:NS(now=lambda:NS(nanoseconds=10**10,to_msg=lambda:NS(sec=10,nanosec=0))))
    monkeypatch.setattr(r.node,'create_subscription',lambda *args:NS(),raising=False)
    monkeypatch.setattr(r.children,'start',lambda *args:None)  # only the external process boundary
    r.launch()
    assert {t.child_frame_id for t in emitted}=={r.segment+'_optical',r.segment+'_imu'}
    camera,imu=emitted
    assert camera.header.frame_id==imu.header.frame_id==r.segment+'_base'
    def apply(t,point,*,vector=False):
        q=t.transform.rotation; xyz=np.array([q.x,q.y,q.z]); p=np.array(point,dtype=float)
        # Standard quaternion vector action; translation only applies to points.
        uv=2*np.cross(xyz,p)
        out=p+q.w*uv+np.cross(xyz,uv)
        v=t.transform.translation
        return out if vector else out+np.array([v.x,v.y,v.z])
    assert apply(camera,[0,0,0])==pytest.approx([.4,0,.2])
    assert apply(camera,[1,0,0])==pytest.approx([.4,1,.2])
    assert apply(imu,[0,0,0])==pytest.approx([-.1,.3,.2])
    assert apply(imu,[0,1,0],vector=True)==pytest.approx([0,0,1])
