"""Run inside the built image: real Domain-42 DDS, simulated ASR and Pose.

No cameras, inference services, motors or speaker are used by this check.
python3 /work/games/wooden_man/smoke_ros.py
"""
import json
import os
import time


def main():
    os.environ["ROS_DOMAIN_ID"] = "42"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ.pop("CYCLONEDDS_URI", None)
    from fastdds_transport import configure_bundled_fastdds_transport
    configure_bundled_fastdds_transport()
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from wooden_man import WoodenManPlugin

    rclpy.init()
    node = Node("wooden_man_smoke")
    asr = node.create_publisher(String, "/wooden_man_test/asr", 10)
    pose = node.create_publisher(String, "/wooden_man_test/pose", 10)
    words = []
    sub = node.create_subscription(String, "/smoke/wooden_man/speech",
                                   lambda msg: words.append(msg.data), 10)
    plugin = WoodenManPlugin({}, "smoke")

    def pump(seconds, dx=0, publish_pose=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish_pose:
                points = [[100 + i * 3 + dx, 100 + (100 if i >= 11 else 0), 0.99]
                          for i in range(17)]
                msg = String()
                msg.data = json.dumps({"persons": [{"id": 1, "keypoints": points}]})
                pose.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.04)
            time.sleep(0.02)

    def speak(text):
        msg = String()
        msg.data = json.dumps({"text": text, "is_final": True}, ensure_ascii=False)
        asr.publish(msg)

    def await_phase(phase, seconds, publish_pose=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            pump(0.1, publish_pose=publish_pose)
            if plugin.dispatch("info", {}).get("phase") == phase:
                return
        raise AssertionError((phase, plugin.info(), words))

    try:
        result = plugin.dispatch("start", {"input_topics": ["/wooden_man_test/asr", "/wooden_man_test/pose"]})
        assert result["state"] == "running", result
        pump(2)
        assert not words and plugin.info()["phase"] == "waiting"
        speak("不要游戏开始")
        pump(0.3)
        assert plugin.info()["phase"] == "waiting"
        speak("游戏开始")
        await_phase("won", 29, publish_pose=True)
        assert words.count("一，二，三，木头人！") == 3, words
        assert words[-1] == "三轮完成，你赢了，游戏结束！", words
        print("PASS: DDS ASR wake, five-second wait, three stationary rounds", flush=True)

        speak("游戏开始")
        await_phase("watch", 12, publish_pose=True)
        pump(0.2, publish_pose=True)
        pump(0.8, dx=20, publish_pose=True)
        assert plugin.info()["phase"] == "lost", plugin.info()
        assert words[-1] == "你动了，游戏结束。", words
        print("PASS: DDS Pose movement -> loss speech", flush=True)

        plugin.dispatch("reset", {})
        await_phase("waiting", 2)
        plugin.dispatch("stop", {})
        assert plugin.info()["state"] == "idle" and plugin.process is None
        print("PASS: reset and stop release worker", flush=True)
    finally:
        plugin.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
