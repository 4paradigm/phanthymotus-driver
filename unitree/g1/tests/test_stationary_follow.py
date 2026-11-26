import unittest
import sys
import json
from types import ModuleType, SimpleNamespace
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stationary_follow import FollowCard, StationaryFollowPlugin, pose2d_to_follow_frame

def frame(stamp, persons):
    return {"timestamp": stamp, "count": len(persons), "persons": persons}

class TestMotusFollow(unittest.TestCase):
    def test_documented_pose2d_bbox_becomes_normalized_position(self):
        out = pose2d_to_follow_frame({"image_size": [640, 480],
            "persons": [{"id": 7, "bbox": [400, 10, 560, 400]}]}, 12.0)
        self.assertEqual(out["persons"], [{"id": 7, "position": [.5, 0]}])

    def test_pose2d_keypoints_fallback_and_invalid_input(self):
        out = pose2d_to_follow_frame({"image_size": [100, 100],
            "persons": [{"keypoints": [[10, 20, 1], [30, 20, 1]]}]}, 1.0)
        self.assertEqual(out["persons"][0]["position"][0], -.6)
        with self.assertRaises(ValueError):
            pose2d_to_follow_frame({"persons": [{}]}, 1.0)

    def test_registered_card_preview_does_not_call_motion(self):
        plugin = StationaryFollowPlugin({}, "g1", object())
        tool = plugin.get_tool()
        self.assertEqual(tool["name"], "g1_stationary_follow")
        self.assertEqual(tool["topic_in"][0]["format"], "sensor/pose2d")
        self.assertEqual(plugin.dispatch("start", {})["state"], "error")
        self.assertEqual(plugin.dispatch("info", {})["mode"], "preview")

    def test_canvas_start_pose_input_and_stop(self):
        clock = [10.0]
        subscriptions = []
        class FakeNode:
            def __init__(self, name):
                self.name = name
                self.destroyed = False
            def create_subscription(self, msg_type, topic, callback, qos):
                subscriptions.append((topic, callback))
            def destroy_node(self):
                self.destroyed = True
        class Executor:
            def __init__(self): self.nodes = []
            def add_node(self, node): self.nodes.append(node)
            def remove_node(self, node): self.nodes.remove(node)
        node_mod = ModuleType("rclpy.node")
        node_mod.Node = FakeNode
        qos_mod = ModuleType("rclpy.qos")
        qos_mod.QoSProfile = lambda **kwargs: kwargs
        qos_mod.ReliabilityPolicy = SimpleNamespace(BEST_EFFORT=1)
        qos_mod.HistoryPolicy = SimpleNamespace(KEEP_LAST=1)
        std_mod = ModuleType("std_msgs")
        std_msg_mod = ModuleType("std_msgs.msg")
        std_msg_mod.String = str
        with patch.dict(sys.modules, {"rclpy": ModuleType("rclpy"),
                                      "rclpy.node": node_mod, "rclpy.qos": qos_mod,
                                      "std_msgs": std_mod, "std_msgs.msg": std_msg_mod}):
            executor = Executor()
            plugin = StationaryFollowPlugin({}, "g1", executor, clock=lambda: clock[0])
            self.assertEqual(plugin.dispatch("start", {"input_topic": "/pose"})["state"], "running")
            self.assertEqual(subscriptions[0][0], "/pose")
            subscriptions[0][1](SimpleNamespace(data=json.dumps({"image_size": [100, 100],
                "persons": [{"bbox": [65, 0, 85, 90]}]})))
            info = plugin.dispatch("info", {})
            self.assertEqual(info["last_decision"]["command"], "move")
            self.assertEqual(info["mode"], "preview")
            clock[0] = 10.31
            self.assertEqual(plugin.dispatch("info", {})["last_decision"]["reason"], "pose_timeout")
            self.assertEqual(plugin.dispatch("stop", {})["state"], "idle")
            self.assertEqual(executor.nodes, [])

    def test_left_right_and_center(self):
        c = FollowCard()
        c.start()
        self.assertLess(c.ingest(frame(1,[{"id":1,"position":[.6,0]}]))["vyaw"],0)
        self.assertGreater(c.ingest(frame(2,[{"id":1,"position":[-.6,0]}]))["vyaw"],0)
        self.assertEqual(c.ingest(frame(3,[{"id":1,"position":[0,0]}]))["command"],"stop_move")

    def test_ambiguous_start_and_target_loss(self):
        c = FollowCard()
        c.start()
        self.assertEqual(c.ingest(frame(1,[{"id":1,"position":[0,0]},{"id":2,"position":[0,0]}]))["command"],"none")
        c.ingest(frame(2,[{"id":1,"position":[.3,0]}]))
        self.assertEqual(c.ingest(frame(3,[{"id":2,"position":[.3,0]}]))["reason"],"target_lost")
        self.assertEqual(c.ingest(frame(4,[{"id":1,"position":[.3,0]}]))["command"],"none")

    def test_timeout_and_manual_stop(self):
        c = FollowCard()
        c.start()
        c.ingest(frame(1,[{"id":1,"position":[.5,0]}]))
        self.assertEqual(c.tick(.31)["command"],"stop_move")
        c.start()
        self.assertEqual(c.stop()["command"],"stop_move")

    def test_repeated_frame_stops(self):
        c = FollowCard()
        c.start()
        c.ingest(frame(1,[{"id":1,"position":[.4,0]}]))
        self.assertEqual(c.ingest(frame(1,[{"id":1,"position":[.4,0]}]))["reason"],"stale_pose")

if __name__ == "__main__":
    unittest.main()
