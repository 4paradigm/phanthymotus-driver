"""Synthetic ROS integration; run in a spare ROS_DOMAIN_ID, never the robot's 42."""

import json
import os
from pathlib import Path
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from audio_environment import AudioEnvironmentPlugin


def main():
    assert os.environ.get("ROS_DOMAIN_ID") not in (None, "0", "42")
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from audio_msgs.msg import AudioChunk
    from std_msgs.msg import String

    rclpy.init()
    executor = SingleThreadedExecutor()
    peer = Node("audio_environment_probe")
    executor.add_node(peer)
    events, states = [], []
    pub = peer.create_publisher(AudioChunk, "/test_pcm", 10)
    peer.create_subscription(
        String,
        "/probe/audio_environment/events",
        lambda m: events.append(json.loads(m.data)),
        10,
    )
    peer.create_subscription(
        String,
        "/probe/audio_environment/state",
        lambda m: states.append(json.loads(m.data)),
        10,
    )
    card = AudioEnvironmentPlugin(
        {"calibration_s": 0.5, "hold_s": 0.5, "recover_s": 0.5}, "probe", executor
    )

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    try:
        assert (
            card.dispatch("start", {"input_topic": "/test_pcm"})["state"] == "running"
        )
        spin(1)
        for index, level in enumerate([100] * 4 + [1000] * 4 + [100] * 4):
            msg = AudioChunk()
            msg.header.stamp = peer.get_clock().now().to_msg()
            msg.format = "audio/pcm-16k" if index % 2 else "pcm_16k_16bit_mono"
            msg.data = list(struct.pack("<4000h", *([level] * 4000)))
            pub.publish(msg)
            spin(0.25)
        assert [e["event"] for e in events] == ["loud", "recovered"], events
        assert states[-1]["status"] == "normal", states[-1]
        spin(2.3)
        assert states[-1]["status"] == "unknown" and states[-1]["rms"] is None
        card.stop()
        count = len(states)
        spin(0.5)
        assert len(states) == count
        print(
            json.dumps(
                {
                    "passed": True,
                    "events": events,
                    "state_messages": count,
                    "stale_unknown": True,
                    "stop_released": True,
                }
            )
        )
    finally:
        card.stop()
        executor.remove_node(peer)
        peer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
