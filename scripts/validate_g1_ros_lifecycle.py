"""Run only inside a network-isolated test container; never imports hardware SDK."""
import os
import sys
import types
import json
import time
import threading
from pathlib import Path
if os.environ.get('G1_TELEOP_ISOLATED_TEST') != '1':
    raise SystemExit('Use --network none --read-only, then set G1_TELEOP_ISOLATED_TEST=1')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'unitree/g1'))
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation
InvalidHandle = rclpy_implementation.InvalidHandle
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from teleop_bus import TeleopBus
ctx=Context();rclpy.init(context=ctx,domain_id=42)
shared=SingleThreadedExecutor(context=ctx)
class Control:
 def __init__(self):
  self._lock=threading.RLock();self.binding={'command_topic':'/test/a','feedback_topic':'/test/state','instance_id':None}
  self.executor=types.SimpleNamespace(set_joint_publisher=lambda cb:None,accept_control=lambda *a:None)
  self.received=[]
 def receive(self,value):
  with self._lock:
   self.binding['instance_id']='pinned';self.received.append(value['sequence']);return True
 def feedback(self):return {'count':len(self.received)}
control=Control();bus=TeleopBus(shared,control)
node=Node('isolated_input_test',context=ctx)
qos=QoSProfile(depth=16,reliability=ReliabilityPolicy.RELIABLE,history=HistoryPolicy.KEEP_LAST)
pubs={name:node.create_publisher(String,name,qos) for name in ['/test/a','/test/b']}
seq=0
owner_events=[]
for method in ['create_subscription','destroy_subscription']:
 original=getattr(bus.node,method)
 def wrapper(*a,_method=method,_original=original,**kw):
  owner_events.append((_method,threading.current_thread().name));return _original(*a,**kw)
 setattr(bus.node,method,wrapper)
def send(seconds,topic='/test/a'):
 global seq
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  seq+=1;message=String();message.data=json.dumps({'sequence':seq});pubs[topic].publish(message);time.sleep(.01)
try:
 send(2)
 assert len(control.received)>20,(len(control.received),bus.status())
 assert bus.subscription_generation==1,bus.status()
 first=len(control.received)
 for i in range(20):
  topic='/test/b' if i%2==0 else '/test/a'
  with control._lock:control.binding={'command_topic':topic,'feedback_topic':'/test/state','instance_id':None}
  send(.12,topic)
 assert len(control.received)>first and bus.health.status()['healthy']
 assert owner_events and all(name=='g1-teleop-ros' for _,name in owner_events),owner_events
 before=len(control.received); original=bus.executor.spin_once; failures=[3]
 def injected(*a,**kw):
  if failures[0]:failures[0]-=1;raise InvalidHandle('injected retirement')
  return original(*a,**kw)
 bus.executor.spin_once=injected
 send(1)
 assert bus.health.status()['invalid_handle_count']==3,bus.status()
 assert len(control.received)>before and bus.health.status()['healthy'],bus.status()
 message=String();message.data='invalid json';pubs['/test/a'].publish(message);time.sleep(.3)
 error=bus.status()['receive_error'];assert error,bus.status()
 time.sleep(.3);assert bus.status()['receive_error']==error
 send(.5);assert bus.status()['receive_error'] is None
 print(json.dumps({'result':'ROS ISOLATED PASS','received':len(control.received),'subscription_generation':bus.subscription_generation,'invalid_handle_count':3,'lifecycle_owner':sorted(set(x[1] for x in owner_events)),'hardware_output':False}))
finally:
 bus.close();assert not bus.thread.is_alive()
 node.destroy_node();shared.shutdown();ctx.shutdown()
