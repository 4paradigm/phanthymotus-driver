"""Actual isolated bus and parent routing over socketpair; ROS allocation stubs."""
import json
from pathlib import Path
import socket
import sys
from types import SimpleNamespace

import teleop_executor
from teleop_executor import TeleopExecutor


def test_device_input_operation_feedback_topics_share_binding_and_reliable_qos(monkeypatch):
    parent, child = socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
    parent.settimeout(1)
    subscriptions, publications, quality = {}, [], {}
    calls=[0]
    class Node:
        def __init__(self,name):pass
        def create_subscription(self,message,topic,callback,qos):
            subscriptions[topic]=callback;quality[topic]=qos;return topic
        def create_publisher(self,message,topic,qos):
            quality[topic]=qos
            return SimpleNamespace(publish=lambda msg:publications.append((topic,json.loads(msg.data))))
        def destroy_subscription(self,topic):subscriptions.pop(topic)
        def destroy_publisher(self,pub):pass
        def destroy_node(self):pass
    command='/robot/teleop/pico_1/command';feedback='/robot/teleop/pico_1/feedback'
    def spin(node,timeout_sec):
        calls[0]+=1
        if calls[0]==1:
            parent.send(json.dumps({'teleop_device_binding':{'command_topic':command,'feedback_topic':feedback},
                                   'teleop_feedback':{'state':'ready','receipts':[]}}).encode())
        elif calls[0]==2:
            for packet in ({'kind':'input','sequence':10},{'kind':'operation','action':'stop','request_id':'stop-1'}):
                subscriptions[command](SimpleNamespace(data=json.dumps(packet)))
                result=json.loads(parent.recv(4096))
                assert result['packet']==packet
                assert result['_motion_route']=='teleop_'+packet['kind']
    monkeypatch.setitem(sys.modules,'rclpy',SimpleNamespace(init=lambda **kw:None,
        ok=lambda:calls[0]<2,spin_once=spin,try_shutdown=lambda:None))
    monkeypatch.setitem(sys.modules,'rclpy.node',SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules,'rclpy.executors',SimpleNamespace(ExternalShutdownException=RuntimeError))
    monkeypatch.setitem(sys.modules,'rclpy.qos',SimpleNamespace(QoSProfile=lambda **kw:kw,
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=0,RELIABLE=1),HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
        DurabilityPolicy=SimpleNamespace(VOLATILE=1)))
    monkeypatch.setitem(sys.modules,'std_msgs.msg',SimpleNamespace(String=SimpleNamespace))
    from common import logsafe
    monkeypatch.setattr(logsafe,'install',lambda **kw:None)
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE',str(Path(teleop_executor.__file__).with_name('dds-local.xml')))
    try:teleop_executor.run_local_bus(child.detach(),'robot',True)
    finally:parent.close();child.close()
    assert quality[command]['depth']==quality[feedback]['depth']==16
    assert quality[command]['reliability']==quality[feedback]['reliability']==1
    assert (feedback,{'state':'ready','receipts':[]}) in publications


def test_latest_pose_is_separate_from_operations_and_stop_is_first():
    item=TeleopExecutor({},'test',None,None,None,[])
    seen=[];item.teleop_control=SimpleNamespace(receive=lambda value:seen.append(value))
    receive,send=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
    receive.setblocking(False);item._bus_socket=receive
    messages=[('teleop_input',{'sequence':1}),('teleop_operation',{'action':'begin','sequence':1}),
              ('teleop_input',{'sequence':100}),('teleop_operation',{'action':'stop','sequence':2})]
    try:
        for route,packet in messages:send.send(json.dumps({'_motion_route':route,'packet':packet}).encode())
        item._receive_latest_command()
    finally:receive.close();send.close()
    assert seen==[{'sequence':100},{'action':'stop','sequence':2},{'action':'begin','sequence':1}]


def test_sustained_input_flood_does_not_discard_stop():
    item=TeleopExecutor({},'test',None,None,None,[])
    seen=[];item.teleop_control=SimpleNamespace(receive=seen.append)
    class Flood:
        count=0
        def recv(self,size):
            self.count+=1
            op=self.count==5
            return json.dumps({'_motion_route':'teleop_operation' if op else 'teleop_input',
                'packet':{'action':'stop','request_id':'s'} if op else {'sequence':self.count}}).encode()
    item._bus_socket=Flood();item._communication_hold=lambda reason:None
    item._receive_latest_command()
    assert seen==[{'action':'stop','request_id':'s'}]


def test_public_contract_size_survives_internal_route_wrapper_without_truncation():
    item=TeleopExecutor({},'test',None,None,None,[])
    seen=[];item.teleop_control=SimpleNamespace(receive=lambda value:seen.append(value))
    packet={'kind':'input','sequence':1,'text':'x'*20000}
    stop={'kind':'operation','action':'stop','request_id':'s'}
    # macOS caps AF_UNIX datagrams below the Linux production socket limit.
    # Emulate recv's actual truncation semantics here; the other tests retain
    # real socketpairs, and the container harness tests real Linux/DDS IPC.
    payloads=[json.dumps({'_motion_route':route,'packet':value}).encode()
              for route,value in [('teleop_input',packet),('teleop_operation',stop)]]
    class Wire:
        def recv(self,size):
            if not payloads:raise BlockingIOError
            return payloads.pop(0)[:size]
    item._bus_socket=Wire();item._receive_latest_command()
    assert seen==[packet,stop]
