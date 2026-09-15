"""AS2W JT128 point-cloud sensor, following the Go2 lidar_cloud contract."""
import struct
import json
from std_msgs.msg import String, UInt8MultiArray
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LidarState_


class _LidarNode:
    def __init__(self, topic, executor):
        from rclpy.node import Node
        self.node = Node("as2w_lidar")
        self.pub = self.node.create_publisher(UInt8MultiArray, topic, 10)
        self.state_pub = self.node.create_publisher(String, topic.replace('/cloud', '/state'), 10)
        try:
            self.sub = ChannelSubscriber("rt/utlidar/cloud_deskewed", PointCloud2_)
            self.sub.Init(self._on_cloud, 1)
            self.node.get_logger().info("AS2W lidar subscribed to rt/utlidar/cloud_deskewed")
        except Exception as exc:
            self.sub = None
            self.node.get_logger().warning(f"AS2W lidar unavailable: {exc}")
        try:
            self.state_sub = ChannelSubscriber("rt/utlidar/state", LidarState_)
            self.state_sub.Init(self._on_state, 1)
        except Exception as exc:
            self.state_sub = None
            self.node.get_logger().warning(f"AS2W lidar state unavailable: {exc}")
        executor.add_node(self.node)

    def _on_cloud(self, msg):
        data = msg.data if isinstance(msg.data, (bytes, bytearray)) else bytes(msg.data)
        payload = struct.pack("<II", int(msg.point_step), int(msg.width) * int(msg.height)) + data
        out = UInt8MultiArray()
        out.data = list(payload)
        self.pub.publish(out)

    def _on_state(self, msg):
        out = String()
        out.data = json.dumps({"firmware_version": msg.firmware_version, "software_version": msg.software_version,
                               "sdk_version": msg.sdk_version, "error_state": int(msg.error_state),
                               "cloud_frequency": float(msg.cloud_frequency),
                               "cloud_packet_loss_rate": float(msg.cloud_packet_loss_rate),
                               "cloud_size": int(msg.cloud_size), "imu_frequency": float(msg.imu_frequency),
                               "imu_packet_loss_rate": float(msg.imu_packet_loss_rate), "imu_rpy": list(msg.imu_rpy)})
        self.state_pub.publish(out)


class LidarPlugin:
    PREFIX = "lidar"
    def __init__(self, config, namespace, executor):
        self.topic = f"/{namespace}/lidar/cloud"
        self.state_topic = f"/{namespace}/lidar/state"
        self.node = _LidarNode(self.topic, executor)
    def get_tools(self):
        return [self._cloud_tool(), self._state_tool()]
    def _cloud_tool(self):
        return {"name": "lidar_cloud", "type": "sensor", "multiInstance": False,
                "description": f"AS2W/Livox PointCloud2 passthrough. Binary format [uint32 point_step][uint32 point_count][raw data], published to {self.topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self.topic, "format": "sensor/pointcloud"}]}
    def _state_tool(self):
        return {"name": "lidar_state", "type": "sensor", "multiInstance": False,
                "description": f"AS2W JT128 LiDAR health and packet diagnostics, published to {self.state_topic}",
                "inputSchema": {"type": "object", "properties": {}},
                "topic_out": [{"topic": self.state_topic, "format": "data/json"}]}
    def start(self): pass
    def stop(self): pass
    def dispatch(self, action, args):
        if action in ("start", "info", "lidar_cloud"): return {"state": "running", "topic_out": [{"topic": self.topic, "format": "sensor/pointcloud"}]}
        if action == "lidar_state": return {"state": "running", "topic_out": [{"topic": self.state_topic, "format": "data/json"}]}
        if action == "stop": return {"state": "idle"}
        return None
