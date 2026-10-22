"""Real HTTP + DDS host loop, with an in-memory Q5 client/router only.

Does not connect to hardware, publish vendor commands or load inference models.
Run with ROS Humble and Q5 interface packages sourced, in an isolated runtime.
"""
import json
import math
import os
import threading
import time
import urllib.request
from http.server import ThreadingHTTPServer


class SimulatedClient:
    q5_position_control_prepared = True
    def __init__(self):
        from wooden_man_motion import JOINT_SUFFIXES, PASSIVE_SUFFIXES
        self.joints = {f"{side}_{name}": value for side in ("left", "right")
                       for name, value in zip(JOINT_SUFFIXES, (-math.pi/6, 0., -.2))}
        self.joints.update({f"{side}_{name}": 0. for side in ("left", "right") for name in PASSIVE_SUFFIXES})
    def snapshot(self):
        return {"fresh": True, "age_ms": 0, "joints": dict(self.joints)}
    def get_lifecycle_state(self):
        return "active"
    def sensor_snapshot(self, _):
        return {"available": True, "fresh": True, "age_ms": 0, "state": 4}


class SimulatedRouter:
    def __init__(self, client):
        self.client, self.owner, self.writes = client, None, []
    def status(self):
        return {"ros_publisher_available": True, "endpoint_query_available": True,
                "active_owner": self.owner, "other_publishers": [{"node_name": "mpc_policy_node", "node_namespace": "/"}],
                "same_name_publisher_count": 1}
    def acquire(self, owner):
        if self.owner is not None:
            return False
        self.owner = owner
        return True
    def release(self, _):
        self.owner = None
    def publish(self, point):
        assert all(k.startswith(("left_shoulder_", "right_shoulder_", "left_elbow_", "right_elbow_")) for k in point)
        self.writes.append(dict(point))
        self.client.joints.update(point)
        return True


def main():
    os.environ.update(ROS_DOMAIN_ID="42", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    os.environ.pop("CYCLONEDDS_URI", None)
    from fastdds_transport import configure_bundled_fastdds_transport
    configure_bundled_fastdds_transport()
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import main as driver
    from wooden_man import WoodenManPlugin
    from wooden_man_motion import Plugin

    rclpy.init()
    node = Node("wooden_man_host_smoke")
    asr = node.create_publisher(String, "/host_test/asr", 10)
    poses = node.create_publisher(String, "/host_test/pose", 10)
    words, feedbacks = [], []
    speech_sub = node.create_subscription(String, "/host_test/wooden_man/speech",
                                         lambda msg: words.append(json.loads(msg.data)["text"]), 10)
    feedback_sub = node.create_subscription(String, "/host_test/wooden_man_motion/feedback",
                                           lambda msg: feedbacks.append(json.loads(msg.data)), 10)
    client = SimulatedClient()
    driver._bundle = driver.Q5Bundle({"plugins": {"wooden_man": {"enabled": True}, "wooden_man_motion": {"enabled": True}}},
                                    "host_test", None, client)
    game = next(p for p in driver._bundle._plugins if isinstance(p, WoodenManPlugin))
    actuator = next(p for p in driver._bundle._plugins if isinstance(p, Plugin))
    router = SimulatedRouter(client)
    actuator.router = router
    server = ThreadingHTTPServer(("127.0.0.1", 0), driver.make_handler())
    threading.Thread(target=server.serve_forever, daemon=True).start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def call(tool, action, **args):
        request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
            data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": "tools/call",
                             "params": {"name": tool, "arguments": {"action": action, **args}}}).encode(),
            headers={"Content-Type": "application/json"})
        with opener.open(request, timeout=15) as response:
            result = json.load(response)
        assert "error" not in result, result
        return json.loads(result["result"]["content"][0]["text"])

    def pump(seconds, moving=False):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            points = [[100 + i * 3, 100 + (100 if i >= 11 else 0), .99] for i in range(17)]
            if moving:
                points[9][0] += 20
            poses.publish(String(data=json.dumps({"image_size": [320, 320], "persons": [{"id": 1, "keypoints": points}]})))
            rclpy.spin_once(node, timeout_sec=.03)
            time.sleep(.02)

    def await_phase(phase, timeout=35):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            pump(.1)
            if call("wooden_man", "info").get("phase") == phase:
                return
        raise AssertionError((phase, game.info(), actuator.info(), words[-8:]))

    def speak(text):
        asr.publish(String(data=json.dumps({"text": text, "background": True})))

    try:
        assert call("wooden_man", "config", host_interaction=True, robot_actions=True, cue_seconds=.15,
                    settle_seconds=.1, watch_seconds=1.4, pose_timeout=.7, robot_settle_seconds=.1)["status"] == "configured"
        assert call("wooden_man_motion", "config", image_mirrored=False)["status"] == "configured"
        assert call("wooden_man", "start", input_topics=["/host_test/asr", "/host_test/pose"])["state"] == "running"
        assert call("wooden_man_motion", "start", input_topic="/host_test/wooden_man/actions")["state"] == "running"
        pump(2)
        speak("游戏开始")
        await_phase("won", 40)
        assert game.info()["round"] == 3 and router.writes
        completed = {f["request_id"]: f for f in feedbacks if f["state"] == "completed"}
        gestures = [f["gesture"] for f in completed.values()]
        assert gestures.count("welcome") == 1 and gestures.count("round") == 2 and gestures.count("celebrate") == 1, gestures
        pump(.3)
        assert any("再挑战一次" in text for text in words), words
        print("PASS: HTTP/DDS preparation -> welcome -> three rounds -> measured gesture return -> celebration -> replay prompt", flush=True)

        speak("再来")
        await_phase("watch", 20)
        pump(.2)
        pump(.4, moving=True)
        await_phase("lost", 20)
        assert game.info()["moved_part"] == "left_arm"
        assert any("你的左手臂动了" in text for text in words)
        assert any(f.get("gesture") == "point" and f["state"] == "completed" for f in feedbacks)
        print("PASS: ASR replay -> body-part loss speech -> point -> returned-arm feedback -> frozen verdict", flush=True)

        pump(.3)
        speak("再来")
        await_phase("robot_action", 10)
        call("wooden_man", "reset")
        await_phase("waiting", 5)
        pump(.5)
        assert router.owner is None, actuator.info()
        assert any(f["state"] == "cancelled" for f in feedbacks)
        print("PASS: reset during gesture cancels hardware worker and releases shared router", flush=True)
    finally:
        game.stop()
        actuator.stop()
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
