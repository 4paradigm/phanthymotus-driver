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
        for comparison, levels in (
            ("above", [100] * 2 + [1000] * 3 + [100] * 3),
            ("below", [1000] * 2 + [100] * 3 + [1000] * 3),
        ):
            response = card.dispatch(
                "config",
                {
                    "mode": "threshold",
                    "threshold_dbfs": -40,
                    "comparison": comparison,
                },
            )
            assert response["ok"] and response["state"] == "running", response
            for level in levels:
                msg = AudioChunk()
                msg.header.stamp = peer.get_clock().now().to_msg()
                msg.format = "audio/pcm-16k"
                msg.data = list(struct.pack("<4000h", *([level] * 4000)))
                pub.publish(msg)
                spin(0.25)
            assert [e["event"] for e in events[-2:]] == [
                "threshold_entered",
                "threshold_cleared",
            ], events
            assert events[-1]["comparison"] == comparison
        response = card.dispatch(
            "config",
            {
                "mode": "relative",
                "baseline_mode": "manual",
                "manual_baseline_dbfs": -40,
            },
        )
        assert response["ok"] and response["state"] == "running", response
        for level in [100] * 2 + [2000] * 3 + [100] * 3:
            msg = AudioChunk()
            msg.header.stamp = peer.get_clock().now().to_msg()
            msg.format = "audio/pcm-16k"
            msg.data = list(struct.pack("<4000h", *([level] * 4000)))
            pub.publish(msg)
            spin(0.25)
        assert [e["event"] for e in events[-2:]] == ["loud", "recovered"]
        assert card.dispatch("read", {})["baseline_dbfs"] == -40
        assert [e["sequence"] for e in events] == list(range(1, 9))
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
