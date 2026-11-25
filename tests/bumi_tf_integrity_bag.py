"""Independent ROS 2 bag player and late-joining TF monitor acceptance."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.serialization import serialize_message
import rosbag2_py
from tf2_msgs.msg import TFMessage

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "noetix" / "bumi"))
from tf_integrity_monitor import TfIntegrityMonitor


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    card = TfIntegrityMonitor({"use_sim_time": True, "publish_hz": 10,
                               "required_chains": [["base_link", "tool"]]},
                              "bag_test", executor)
    with tempfile.TemporaryDirectory() as directory:
        bag = str(Path(directory) / "tf")
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("", ""))
        writer.create_topic(rosbag2_py.TopicMetadata(
            name="/tf", type="tf2_msgs/msg/TFMessage", serialization_format="cdr"))
        for i in range(50):
            stamp = 100_000_000_000 + i * 100_000_000
            msg = TransformStamped()
            msg.header.frame_id, msg.child_frame_id = "base_link", "joint"
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 10**9)
            msg.transform.rotation.w = 1.0
            writer.write("/tf", serialize_message(TFMessage(transforms=[msg])), stamp)
        del writer
        static = subprocess.Popen(["ros2", "run", "tf2_ros", "static_transform_publisher",
                                   "--frame-id", "joint", "--child-frame-id", "tool"])
        try:
            time.sleep(2)
            assert static.poll() is None
            assert card.dispatch("start", {})["state"] == "running"
            player = subprocess.Popen(["ros2", "bag", "play", bag, "--clock", "40", "--delay", "2"],
                                      stdin=subprocess.DEVNULL)
            try:
                deadline = time.monotonic() + 25
                healthy_seconds = set()
                while player.poll() is None and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.1)
                    report = card.dispatch("info", {})["diagnostics"]
                    if report["state"] == "healthy":
                        assert report["checked_frames"] == ["base_link", "joint", "tool"]
                        assert report["translation_jump_count"] == report["rotation_jump_count"] == 0
                        healthy_seconds.add(report["timestamp"]["sec"])
                assert player.poll() == 0, player.poll()
                assert len(healthy_seconds) >= 3, (healthy_seconds, report)
                print(f"INDEPENDENT_ROSBAG_PASS monitor={os.getpid()} player={player.pid} "
                      f"static={static.pid} ROS seconds={sorted(healthy_seconds)}")
            finally:
                if player.poll() is None:
                    player.terminate()
                player.wait(timeout=5)
        finally:
            static.terminate()
            static.wait(timeout=5)
            card.stop()
            executor.shutdown()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
