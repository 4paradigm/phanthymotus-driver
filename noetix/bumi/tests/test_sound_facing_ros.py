"""Opt-in REAL ROS + HTTP MCP + production Loco loop, with a fake plant only.

Run with BUMI_SOUND_FACING_ROS_TEST=1 in an isolated loopback ROS environment.
No SDK module is imported. Source health and all robot feedback are synthetic.
"""

import json
import math
import os
import sys
import threading
import time
import types
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(os.environ.get("BUMI_SOUND_FACING_ROS_TEST") != "1",
                                reason="requires isolated ROS2 and generated audio_msgs")


def test_real_ros_mcp_loco_chain(monkeypatch):
    assert os.environ.get("ROS_DOMAIN_ID") not in (None, "0", "42")
    assert os.environ.get("RMW_IMPLEMENTATION") == "rmw_fastrtps_cpp"
    profile = Path(os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]).read_text()
    assert "127.0.0.1" in profile and "<useBuiltinTransports>false" in profile
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    import device
    import main
    from sound_facing import CONVENTION
    from sound_facing_card import SoundFacingPlugin
    from sound_authorization_fixtures import approved_authority

    class Plant:
        def __init__(self):
            self.yaw = 0
            self.cmds = []

        def get_mode(self):
            return 2

        def get_imu_data(self):
            half = math.radians(self.yaw) / 2
            return types.SimpleNamespace(ori=[0, 0, math.sin(half), math.cos(half)], angular_vel=[0, 0, 0])

        def publish_cmd(self, x, y, z, cmd, index):
            assert x == y == 0
            self.cmds.append(z)
            self.yaw += z * 3

    calls = []
    monkeypatch.setattr(device, "_get_control_cmd", lambda name: name)
    monkeypatch.setattr(device, "_acp_notify", lambda *args: calls.append(args))
    plant = Plant()
    source_health = types.SimpleNamespace(dispatch=lambda *args: {
        "state": "running", "calibrated_directions": ["front", "right"]})
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=3)
    source = Node("sound_facing_synthetic_source")
    executor.add_node(source)
    loco = device.LocoPlugin({}, "sound_facing_sim", executor, plant)
    card = SoundFacingPlugin({}, "sound_facing_sim", executor, loco, source_plugin=source_health,
                             authority=approved_authority(device="sound_facing_sim"))
    bundle = main.BumiDeviceBundle({"plugins": {}}, "sound_facing_sim", executor, None, None)
    monkeypatch.setattr(main, "_bundle", bundle)
    bundle._plugins = [loco, card]
    http = ThreadingHTTPServer(("127.0.0.1", 0), main.make_handler())
    http_thread = threading.Thread(target=http.serve_forever, daemon=True)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    states = []
    source.create_subscription(String, "/sound_facing_sim/bumi_sound_facing/state",
                               lambda msg: states.append(json.loads(msg.data)), 1)
    pub = source.create_publisher(String, "/sound_facing_sim/sound_direction", qos_profile_sensor_data)

    def rpc(method, params):
        payload = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode()
        req = urllib.request.Request(f"http://127.0.0.1:{http.server_port}/mcp", data=payload,
                                     headers={"Content-Type": "application/json"})
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(req, timeout=3) as response:
            result = json.load(response)["result"]
        return result

    def call(action, **kwargs):
        return json.loads(rpc("tools/call", {"name": "bumi_sound_facing", "arguments": {
            "action": action, **kwargs}})["content"][0]["text"])

    def wait(predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        raise AssertionError(card._engine.snapshot())

    stop_source = threading.Event()
    source_angle = [None]

    def publish():
        while not stop_source.wait(0.25):
            end = time.time_ns() // 1000
            data = {"state": "no_event"} if source_angle[0] is None else {
                "state": "fresh", "angle_deg": source_angle[0], "angle_convention": CONVENTION,
                "audio_window_start_us": end - 200_000, "audio_window_end_us": end,
                "timestamp_ms": end // 1000}
            pub.publish(String(data=json.dumps(data)))

    source_thread = threading.Thread(target=publish, daemon=True)
    try:
        card.start()
        http_thread.start()
        spin_thread.start()
        source_thread.start()
        wait(lambda: pub.get_subscription_count() > 0)
        assert call("start")["enabled"] is False
        assert not plant.cmds
        assert rpc("tools/list", {})["tools"][-1]["name"] == "bumi_sound_facing"
        assert call("enable_follow", confirm_motion=True)["state"] == "error"
        enabled = call("enable_follow")
        assert enabled["enabled"] is True
        assert call("renew_follow", session_id=enabled["session_id"])["enabled"] is True
        for target in (35, -35, 90):
            source_angle[0] = target
            wait(lambda: card._engine.state == "turning", timeout=7)
            source_angle[0] = None
            wait(lambda: card._engine.state == "settling")
            wait(lambda: card._engine.state in ("observing", "paused"))
        assert card._engine.turns == 3
        assert any(z > 0 for z in plant.cmds) and any(z < 0 for z in plant.cmds)
        assert plant.cmds[-1] == 0 and not calls
        source_angle[0] = 170
        wait(lambda: card._engine.state == "turning")
        result = call("disable_follow")
        assert result["enabled"] is False
        wait(lambda: card._engine.state == "idle")
        assert plant.cmds[-1] == 0
        assert any(state["state"] == "turning" for state in states)
        assert any(state["state"] == "settling" for state in states)
        assert call("stop")["running"] is False
        assert not card._worker.is_alive() and card._node is None
        print(json.dumps({"motion": "fake plant", "real_ros": True, "real_http_mcp": True,
                          "internal_acp_callbacks": len(calls), "completed_turns": card._engine.turns,
                          "state_messages": len(states), "fake_velocity_commands": len(plant.cmds)}))
    finally:
        stop_source.set()
        if source_thread.is_alive():
            source_thread.join(2)
        card.stop()
        http.shutdown()
        http.server_close()
        http_thread.join(2)
        executor.remove_node(source)
        source.destroy_node()
        executor.shutdown(timeout_sec=3)
        spin_thread.join(3)
        rclpy.shutdown()
