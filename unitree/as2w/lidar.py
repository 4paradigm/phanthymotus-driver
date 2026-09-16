"""AS2W JT128 point-cloud sensor, following the Go2 lidar_cloud contract."""
import struct
from std_msgs.msg import UInt8MultiArray
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_


class _LidarNode:
    def __init__(self, topic, executor):
        from rclpy.node import Node
        self.node = Node("as2w_lidar")
        self.pub = self.node.create_publisher(UInt8MultiArray, topic, 10)
        try:
            self.sub = ChannelSubscriber("rt/utlidar/cloud_deskewed", PointCloud2_)
            self.sub.Init(self._on_cloud, 1)
            self.node.get_logger().info("AS2W lidar subscribed to rt/utlidar/cloud_deskewed")
        except Exception as exc:
            self.sub = None
            self.node.get_logger().warning(f"AS2W lidar unavailable: {exc}")
        executor.add_node(self.node)

    def _on_cloud(self, msg):
        data = msg.data if isinstance(msg.data, (bytes, bytearray)) else bytes(msg.data)
        payload = struct.pack("<II", int(msg.point_step), int(msg.width) * int(msg.height)) + data
        out = UInt8MultiArray()
        out.data = list(payload)
        self.pub.publish(out)

class LidarPlugin:
    PREFIX = "lidar"
    def __init__(self, config, namespace, executor):
        self.topic = f"/{namespace}/lidar/cloud"
        self.node = _LidarNode(self.topic, executor)
    def get_tools(self):
        return [self._cloud_tool()]
    def _cloud_tool(self):
        return {"name": "lidar_cloud", "type": "sensor", "multiInstance": False,
                "description": f"AS2W/Livox PointCloud2 passthrough. Binary format [uint32 point_step][uint32 point_count][raw data], published to {self.topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self.topic, "format": "sensor/pointcloud"}]}
    def start(self): pass
    def stop(self):
        if getattr(self.node, "sub", None):
            self.node.sub.Close()
        self.node.node.destroy_node()
    def dispatch(self, action, args):
        if action in ("start", "info", "lidar_cloud"): return {"state": "running", "topic_out": [{"topic": self.topic, "format": "sensor/pointcloud"}]}
        if action == "stop": return {"state": "idle"}
        return None
