import unittest
import sys
import json
import time
import zlib
from array import array
from types import ModuleType, SimpleNamespace
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stationary_follow import FollowCard, StationaryFollowPlugin, pose2d_to_follow_frame
from leg_detection import DepthLegDetector

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

    def test_registered_card_requires_pose_input(self):
        plugin = StationaryFollowPlugin({}, "g1", object())
        tool = plugin.get_tool()
        self.assertEqual(tool["name"], "g1_stationary_follow")
        self.assertEqual(tool["topic_in"][0]["format"], "image/depth-zlib")
        self.assertEqual(plugin.dispatch("start", {})["state"], "error")
        self.assertEqual(tool["topic_out"][0]["format"], "data/json")
        self.assertEqual(plugin.dispatch("info", {})["state"], "idle")

    def test_canvas_start_pose_input_and_stop(self):
        clock = [10.0]
        subscriptions = []
        published = []
        class FakeNode:
            def __init__(self, name):
                self.name = name
                self.destroyed = False
            def create_subscription(self, msg_type, topic, callback, qos):
                subscriptions.append((topic, callback))
            def create_publisher(self, msg_type, topic, qos):
                return SimpleNamespace(publish=lambda msg: published.append(msg.data))
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
        std_msg_mod.String = lambda data="": SimpleNamespace(data=data)
        with patch.dict(sys.modules, {"rclpy": ModuleType("rclpy"),
                                      "rclpy.node": node_mod, "rclpy.qos": qos_mod,
                                      "std_msgs": std_mod, "std_msgs.msg": std_msg_mod}):
            executor = Executor()
            plugin = StationaryFollowPlugin({}, "g1", executor, clock=lambda: clock[0])
            self.assertEqual(plugin.dispatch("start", {"input_topic": "/pose", "input_mode": "pose"})["state"], "running")
            self.assertEqual(subscriptions[0][0], "/pose")
            subscriptions[0][1](SimpleNamespace(data=json.dumps({"image_size": [100, 100],
                "persons": [{"bbox": [65, 0, 85, 90]}]})))
            info = plugin.dispatch("info", {})
            self.assertEqual(info["last_decision"]["command"], "move")
            self.assertEqual(info["topic_out"][0]["format"], "data/json")
            clock[0] = 10.31
            time.sleep(.15)
            self.assertEqual(plugin.dispatch("info", {})["last_decision"]["reason"], "pose_timeout")
            self.assertEqual(plugin.dispatch("stop", {})["state"], "idle")
            self.assertEqual(executor.nodes, [])

    def test_turn_command_is_bounded_and_timeout_stops(self):
        clock = [10.0]
        commands = []
        class Motion:
            def get_state(self): return {"base_available": True}
            def move(self, vx, vy, vyaw, duration):
                commands.append(("move", vx, vy, vyaw, duration))
                return {"ret": 0}
            def stop(self, reason="command"):
                commands.append(("stop", reason))
                return {"ret": 0}
        plugin = StationaryFollowPlugin({}, "g1", object(), motion=Motion(),
                                        admission=lambda: {"admitted": True},
                                        clock=lambda: clock[0])
        plugin._node = object()
        plugin._controller.start()
        plugin._last_decision = plugin._controller.ingest(frame(10.0, [{"id": 1, "position": [.5, 0]}]))
        plugin._last_received = clock[0]
        plugin._shutdown.clear()
        worker = __import__('threading').Thread(target=plugin._control_loop, daemon=True)
        worker.start()
        time.sleep(.15)
        self.assertTrue(any(c[0] == "move" and c[1:3] == (0.0, 0.0)
                            and c[4] == .12 for c in commands))
        clock[0] = 10.4
        time.sleep(.15)
        plugin._shutdown.set()
        worker.join(1)
        self.assertEqual(plugin.dispatch("info", {})["last_decision"]["reason"], "pose_timeout")
        self.assertFalse(any(c[0] == "stop" for c in commands))

    def test_busy_base_never_receives_turn(self):
        calls = []
        class Motion:
            def get_state(self): return {"base_available": False}
            def move(self, *args): calls.append(args)
        plugin = StationaryFollowPlugin({}, "g1", object(), motion=Motion(),
                                        admission=lambda: {"admitted": True})
        plugin._node = object()
        plugin._controller.start()
        plugin._last_decision = plugin._controller.ingest(frame(1.0, [{"id": 1, "position": [.5, 0]}]))
        plugin._last_received = plugin._clock()
        worker = __import__('threading').Thread(target=plugin._control_loop, daemon=True)
        worker.start()
        time.sleep(.15)
        plugin._shutdown.set()
        worker.join(1)
        self.assertEqual(calls, [])

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

    def test_depth_pair_requires_three_stable_frames_and_rejects_clutter(self):
        detector = DepthLegDetector()
        pixels = array('H', [2000]) * (640 * 480)
        for top, bottom, left, right in ((210, 400, 220, 260),
                                          (210, 400, 330, 370),
                                          (210, 400, 500, 620)):
            for y in range(top, bottom):
                for x in range(left, right):
                    pixels[y * 640 + x] = 800
        data = zlib.compress(pixels.tobytes())
        self.assertEqual(detector.detect(data)[1], "leg_candidate_unconfirmed")
        self.assertEqual(detector.detect(data)[1], "leg_candidate_unconfirmed")
        center, reason = detector.detect(data)
        self.assertEqual(reason, "legs_tracked")
        self.assertLess(abs(center), .2)
        self.assertEqual(detector.detect(b"broken")[1], "invalid_depth")
        for y in range(210, 400):
            for x in range(330, 370):
                pixels[y * 640 + x] = 2000
        self.assertEqual(detector.detect(zlib.compress(pixels.tobytes()))[1],
                         "no_unique_leg_pair")

if __name__ == "__main__":
    unittest.main()
