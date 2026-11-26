"""ROS transport for the simulation-only card bundle."""
import json
import threading
import time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def deny_hardware_sockets():
    """Keep SocketCAN and raw link-layer sockets unreachable while DDS uses UDP."""
    import ctypes as ct
    import errno
    import socket
    lib = ct.CDLL('libseccomp.so.2')
    class Match(ct.Structure):
        _fields_ = [('arg', ct.c_uint), ('op', ct.c_uint), ('value', ct.c_uint64), ('unused', ct.c_uint64)]
    lib.seccomp_init.argtypes = [ct.c_uint32]
    lib.seccomp_init.restype = ct.c_void_p
    lib.seccomp_syscall_resolve_name.argtypes = [ct.c_char_p]
    lib.seccomp_rule_add_array.argtypes = [ct.c_void_p,ct.c_uint32,ct.c_int,ct.c_uint,ct.POINTER(Match)]
    lib.seccomp_load.argtypes = lib.seccomp_release.argtypes = [ct.c_void_p]
    ctx = lib.seccomp_init(0x7fff0000)  # SCMP_ACT_ALLOW, layered over Docker's own filter.
    if not ctx: raise RuntimeError('Cannot initialize simulation socket restrictions')
    try:
        syscall = lib.seccomp_syscall_resolve_name(b'socket')
        for family in (socket.AF_CAN,socket.AF_PACKET):
            if lib.seccomp_rule_add_array(ctx,0x00050000|errno.EPERM,syscall,1,ct.byref(Match(0,4,int(family),0))):
                raise RuntimeError('Cannot limit hardware socket family')
        if lib.seccomp_load(ctx):raise RuntimeError('Cannot apply simulation socket restrictions')
    finally:lib.seccomp_release(ctx)


class SimulationBus:
    def __init__(self, domain=42):
        rclpy.init(domain_id=domain)
        self.node=rclpy.create_node('l20_mujoco_trajectory')
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.pubs={};self.subs={};self.images={};self.lock=threading.RLock()
        self.worker=threading.Thread(target=self.executor.spin,daemon=True)
        self.worker.start()

    def bind(self,key,topics,receiver,output):
        with self.lock:
            self.unbind(key)
            self.subs[key]=[]
            for topic in topics:
                def consume(msg,topic=topic):
                    if len(msg.data)>100000:return
                    try:packet=json.loads(msg.data)
                    except (ValueError,RecursionError):return
                    receiver(topic,packet)
                self.subs[key].append(self.node.create_subscription(String,topic,consume,10))
            self.images[key]=self.node.create_publisher(CompressedImage,output,qos_profile_sensor_data)

    def unbind(self,key):
        with self.lock:
            for sub in self.subs.pop(key,[]):self.node.destroy_subscription(sub)
            pub=self.images.pop(key,None)
            if pub:self.node.destroy_publisher(pub)

    def publish(self,topic,packet):
        with self.lock:
            if topic not in self.pubs:self.pubs[topic]=self.node.create_publisher(String,topic,10)
            pub=self.pubs[topic]
        end=time.monotonic()+2
        while not pub.get_subscription_count() and time.monotonic()<end:time.sleep(.025)
        msg=String();msg.data=json.dumps(packet,separators=(',',':'));pub.publish(msg)

    def image(self,key,jpeg):
        with self.lock:
            pub=self.images.get(key)
            if pub:
                msg=CompressedImage();msg.header.stamp=self.node.get_clock().now().to_msg()
                msg.format='jpeg';msg.data=jpeg;pub.publish(msg)

    def shutdown(self):
        self.executor.shutdown();self.worker.join(3);self.node.destroy_node();rclpy.shutdown()
