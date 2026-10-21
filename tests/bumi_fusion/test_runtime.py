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
