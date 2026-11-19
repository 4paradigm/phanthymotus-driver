"""TEST ONLY: in-process ROS API transport for the real Core integration harness.

No DDS sockets, devices, discovery or robot SDK. Never imported by production.
This is protocol evidence, not a real ROS or hardware acceptance result.
"""
import sys
import threading
import time
import types
from collections import defaultdict


def install():
    if 'rclpy' in sys.modules:
        raise RuntimeError('Refusing to replace an imported real ROS context')
    lock=threading.RLock()
    subscriptions=defaultdict(list)
    topics=set()
    running=[False]
    class Message:
        def __init__(self,**kwargs):
            self.data=''
            self.__dict__.update(kwargs)
    class Node:
        def __init__(self,name,*args,**kwargs):
            self.name=name;self.subs=[];self.timers=[];self.destroyed=False
        def create_subscription(self,kind,topic,callback,qos):
            sub=(self,topic,callback)
            with lock:subscriptions[topic].append(sub);self.subs.append(sub);topics.add(topic)
            return sub
        def create_publisher(self,kind,topic,qos):
            topics.add(topic)
            def publish(msg):
                with lock:callbacks=list(subscriptions.get(topic,[]))
                for owner,_,callback in callbacks:
                    if not owner.destroyed:callback(msg)
            return types.SimpleNamespace(publish=publish)
        def create_timer(self,interval,callback):
            timer=[time.monotonic()+interval,interval,callback]
            self.timers.append(timer);return timer
        def destroy_subscription(self,sub):
            with lock:
                if sub in subscriptions[sub[1]]:subscriptions[sub[1]].remove(sub)
                if sub in self.subs:self.subs.remove(sub)
        def destroy_node(self):
            self.destroyed=True
            for sub in list(self.subs):self.destroy_subscription(sub)
            self.timers.clear()
        def get_topic_names_and_types(self):return [(topic,['std_msgs/msg/String']) for topic in topics]
        def get_name(self):return self.name
        def get_logger(self):
            return types.SimpleNamespace(info=lambda *a:None,warning=lambda *a:None,error=lambda *a:None)
    class Executor:
        def __init__(self,*args,**kwargs):self.nodes=[];self.halt=threading.Event()
        def add_node(self,node):self.nodes.append(node)
        def remove_node(self,node):
            if node in self.nodes:self.nodes.remove(node)
        def wake(self):pass
        def spin_once(self,timeout_sec=.01):
            for node in list(self.nodes):
                for timer in list(node.timers):
                    now=time.monotonic()
                    if not node.destroyed and now>=timer[0]:timer[0]=now+timer[1];timer[2]()
            self.halt.wait(min(timeout_sec,.01))
        def spin(self):
            while not self.halt.is_set():self.spin_once()
        def shutdown(self,*args,**kwargs):self.halt.set()
    modules={name:types.ModuleType(name) for name in ('rclpy','rclpy.node','rclpy.qos','rclpy.executors','std_msgs','std_msgs.msg','sensor_msgs','sensor_msgs.msg','audio_msgs','audio_msgs.msg')}
    modules['rclpy'].init=lambda *a,**kw:running.__setitem__(0,True)
    modules['rclpy'].shutdown=lambda *a,**kw:running.__setitem__(0,False)
    modules['rclpy'].ok=lambda:running[0]
    modules['rclpy'].create_node=Node
    modules['rclpy'].executors=modules['rclpy.executors']
    modules['rclpy.node'].Node=Node
    modules['rclpy.executors'].MultiThreadedExecutor=Executor
    modules['rclpy.qos'].QoSProfile=lambda **kw:types.SimpleNamespace(**kw)
    for name in ('ReliabilityPolicy','HistoryPolicy','DurabilityPolicy'):
        setattr(modules['rclpy.qos'],name,types.SimpleNamespace(BEST_EFFORT=1,RELIABLE=2,KEEP_LAST=1,VOLATILE=1))
    modules['rclpy.qos'].qos_profile_sensor_data=object()
    for package,names in [('std_msgs.msg',['String']),('audio_msgs.msg',['AudioChunk']),('sensor_msgs.msg',['Image','CompressedImage'])]:
        for name in names:setattr(modules[package],name,Message)
    sys.modules.update(modules)
