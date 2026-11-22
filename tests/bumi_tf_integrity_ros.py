"""Run with the driver ROS environment sourced, on an isolated ROS domain."""
import json
from pathlib import Path
import sys
import tempfile
from threading import Thread
from urllib.request import Request, urlopen
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rosgraph_msgs.msg import Clock
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "noetix" / "bumi"))
import main as driver
from tf_integrity_monitor import PREFIX, TfIntegrityMonitor


def main():
    rclpy.init()
    source = rclpy.create_node("tf_integrity_fixture")
    executor = SingleThreadedExecutor()
    executor.add_node(source)
    driver._bundle = driver.BumiDeviceBundle({"plugins": {PREFIX: {
        "enabled": True, "required_chains": [["base_link", "tool"]], "publish_hz": 10,
    }}}, "integrity_test", executor, None, None)
    card = driver._bundle._plugins[0]
    driver._bundle.start_all()
    server = driver.ThreadingHTTPServer(("127.0.0.1", 0), driver.make_handler())
    http = Thread(target=server.serve_forever, daemon=True)
    http.start()

    def rpc(method, params=None):
        request = Request(f"http://127.0.0.1:{server.server_port}/mcp",
                          data=json.dumps(dict(jsonrpc="2.0", id=1, method=method,
                                               params=params)).encode(),
                          headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=10) as response:
            body = json.load(response)
        assert "error" not in body, body
        return body["result"]

    def call(action):
        result = rpc("tools/call", {"name": PREFIX, "arguments": {"action": action}})
        return json.loads(result["content"][0]["text"])

    reports = []
    source.create_subscription(String, card.output, lambda msg: reports.append(json.loads(msg.data)), 10)
    dynamic = TransformBroadcaster(source)
    static = StaticTransformBroadcaster(source)

    def edge(parent, child):
        msg = TransformStamped()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.header.stamp = source.get_clock().now().to_msg()
        msg.transform.rotation.w = 1.0
        return msg

    try:
        assert rpc("initialize")["serverInfo"]["name"] == "bumi-device-bundle"
        tool, = rpc("tools/list")["tools"]
        assert tool["name"] == PREFIX and tool["type"] == "sensor"
        assert tool["topic_out"] == [{"topic": card.output, "format": "data/json"}]
        # Publish before starting the subscriber to exercise transient-local delivery.
        static.sendTransform(edge("joint", "tool"))
        for _ in range(3):
            reports.clear()
            assert call("start")["state"] == "running"
            assert call("start")["state"] == "running"
            assert len(executor.get_nodes()) == 2
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                dynamic.sendTransform(edge("base_link", "joint"))
                executor.spin_once(timeout_sec=.03)
                if reports and reports[-1]["state"] == "healthy":
                    break
                time.sleep(.02)
            assert reports and reports[-1]["state"] == "healthy", reports[-1:]
            assert call("info")["diagnostics"]["state"] == "healthy"
            assert reports[-1]["checked_frames"] == ["base_link", "joint", "tool"]
            assert call("stop")["state"] == "idle"
            assert call("stop")["state"] == "idle"
            assert len(executor.get_nodes()) == 1
        card = TfIntegrityMonitor({
            "required_chains": [["base_link", "joint"]],
            "publish_hz": 10, "use_sim_time": True, "visual_check": True,
            "vision": {
                "calibration_id": "fixture-intrinsics-extrinsics",
                "survey_id": "fixture-survey",
                "time_sync_id": "fixture-clock",
                "camera_frame": "camera_optical_frame",
                "surveyed_marker_frame": "surveyed_tag",
                "observation_topic": "/test_tag_pose",
            },
        }, "integrity_test", executor)
        statics = [edge("joint", "tool"), edge("base_link", "camera_optical_frame"),
                   edge("base_link", "surveyed_tag")]
        statics[-1].transform.translation.z = 2.0
        static.sendTransform(statics)
        clock_pub = source.create_publisher(Clock, "/clock", 10)
        pose_pub = source.create_publisher(PoseStamped, "/test_tag_pose", 10)
        assert card.dispatch("start", {})["state"] == "running"
        for second, position, resets, result in ((30, 2.0, 0, "consistent"),
                                                  (2, 2.0, 1, "consistent"),
                                                  (3, 3.0, 1, "inconsistent")):
            reports.clear()
            clock = Clock()
            clock.clock.sec = second
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                clock_pub.publish(clock)
                executor.spin_once(timeout_sec=.03)
                joint = edge("base_link", "joint")
                joint.header.stamp = clock.clock
                dynamic.sendTransform(joint)
                pose = PoseStamped()
                pose.header.frame_id = "camera_optical_frame"
                pose.header.stamp = clock.clock
                pose.pose.orientation.w = 1.0
                pose.pose.position.z = position
                pose_pub.publish(pose)
                executor.spin_once(timeout_sec=.03)
                if reports and reports[-1]["visual_state"] == result and reports[-1]["clock_resets"] == resets:
                    break
                time.sleep(.02)
            assert reports and reports[-1]["visual_state"] == result, reports[-1:]
            assert reports[-1]["clock_resets"] == resets
        card.stop()
        card = TfIntegrityMonitor({"required_chains": [["base_link", "joint"]],
                                   "publish_hz": 10, "use_sim_time": True},
                                  "integrity_test", executor)
        assert card.dispatch("start", {})["state"] == "running"
        reports.clear()
        with tempfile.TemporaryDirectory() as directory:
            bag = str(Path(directory) / "tf")
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                        rosbag2_py.ConverterOptions("", ""))
            writer.create_topic(rosbag2_py.TopicMetadata(
                name="/tf", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"))
            for index in range(40):
                stamp = 100_000_000_000 + index * 100_000_000
                joint = edge("base_link", "joint")
                joint.header.stamp.sec, joint.header.stamp.nanosec = divmod(stamp, 10**9)
                writer.write("/tf", serialize_message(TFMessage(transforms=[joint])), stamp)
            del writer
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                        rosbag2_py.ConverterOptions("", ""))
            while reader.has_next():
                topic, blob, stamp = reader.read_next()
                assert topic == "/tf"
                clock = Clock()
                clock.clock.sec, clock.clock.nanosec = divmod(stamp, 10**9)
                clock_pub.publish(clock)
                executor.spin_once(timeout_sec=.05)
                dynamic.sendTransform(deserialize_message(blob, TFMessage).transforms)
                executor.spin_once(timeout_sec=.05)
                time.sleep(.1)
            playback = [r for r in reports if r["state"] == "healthy"
                        and r["timestamp"]["sec"] >= 101]
            assert len(playback) >= 3, reports[-5:]
            assert all(r["translation_jump_count"] == r["rotation_jump_count"] == 0
                       for r in playback)
    finally:
        server.shutdown()
        http.join(timeout=5)
        server.server_close()
        driver._bundle.stop_all()
        card.stop()
        executor.shutdown()
        source.destroy_node()
        rclpy.shutdown()
    print("MCP discovery, ROS TF and JSON, three lifecycle cycles, clock reset, vision and rosbag replay passed")


if __name__ == "__main__":
    main()
