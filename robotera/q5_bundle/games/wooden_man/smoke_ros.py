"""Run inside the built image: real Domain-42 DDS, simulated ASR and Pose.

No cameras, inference services, motors or speaker are used by this check.
python3 /work/games/wooden_man/smoke_ros.py
"""
import json
import os
import time
import threading
import urllib.request
from http.server import ThreadingHTTPServer


def main(host_interaction=False):
    os.environ["ROS_DOMAIN_ID"] = "42"
    os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    os.environ.pop("CYCLONEDDS_URI", None)
    from fastdds_transport import configure_bundled_fastdds_transport
    configure_bundled_fastdds_transport()
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import CompressedImage
    from wooden_man import DEFAULTS, HAND_NAMES, clean_text
    import main as driver

    rclpy.init()
    node = Node("wooden_man_smoke")
    asr = node.create_publisher(String, "/wooden_man_test/asr", 10)
    background_asr = node.create_publisher(String, "/wooden_man_test/asr_background", 10)
    pose = node.create_publisher(String, "/wooden_man_test/pose", 10)
    words = []
    images = []
    hand_mode = False
    finger_delta = 0
    registering = True
    def receive_image(msg):
        from io import BytesIO
        from PIL import Image
        assert msg.format == "jpeg"
        with Image.open(BytesIO(bytes(msg.data))) as im:
            assert im.size == (800, 600)
        images.append(bytes(msg.data))
    image_sub = node.create_subscription(CompressedImage, "/smoke/wooden_man/highlight", receive_image, 1)
    def receive_tts(msg):
        # Match perception/plugins/tts.py:_text_cb, rather than accepting raw
        # text as the previous smoke test did. An invalid JSON wire fails here.
        payload = json.loads(msg.data)
        assert isinstance(payload, dict) and isinstance(payload.get("text"), str), payload
        words.append(payload["text"])

    sub = node.create_subscription(String, "/smoke/wooden_man/speech", receive_tts, 10)
    driver._bundle = driver.Q5Bundle({"plugins": {"wooden_man": {"enabled": True}}},
                                    "smoke", None, None)
    plugin = driver._bundle._plugins[0]
    server = ThreadingHTTPServer(("127.0.0.1", 0), driver.make_handler())
    threading.Thread(target=server.serve_forever, daemon=True).start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def control(action, args=None):
        arguments = {"action": action, "instance_id": "card-smoke", **(args or {})}
        req = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
                                     data=json.dumps({"jsonrpc": "2.0", "id": 1,
                                                      "method": "tools/call", "params": {
                                                          "name": "wooden_man", "arguments": arguments}}).encode(),
                                     headers={"Content-Type": "application/json"})
        with opener.open(req, timeout=15) as response:
            reply = json.load(response)
        assert "error" not in reply, reply
        return json.loads(reply["result"]["content"][0]["text"])

    def pump(seconds, dx=0, publish_pose=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish_pose:
                points = [[100 + i * 3 + dx, 100 + (100 if i >= 11 else 0), 0.99]
                          for i in range(17)]
                payload = {"persons": [{"id": 1, "keypoints": points}], "image_size": [320, 320]}
                if host_interaction:
                    people = []
                    for identity, x in ((1,180), (2,440), (99,720)):
                        body = [[x,70,.99] for _ in range(17)]
                        for i, px, py in ((5,-20,110), (6,20,110), (7,-30,155), (8,30,155),
                                         (9,-30,200), (10,30,200), (11,-15,210), (12,15,210),
                                         (13,-15,290), (14,15,290), (15,-15,370), (16,15,370)):
                            body[i] = [x+px,py,.99]
                        if registering and identity != 99:
                            body[9][1] = 65
                        if identity == 2:
                            body[9][0] += dx
                        if identity == 99:
                            # A seated, moving background person never opts in.
                            body[13][:2], body[14][:2] = [x+80,210], [x+105,210]
                            body[15][:2], body[16][:2] = [x+80,290], [x+105,290]
                            body[9][0] += int(time.monotonic()*25) % 100
                        people.append({"id": identity, "keypoints": body})
                    payload = {"persons": list(reversed(people)), "image_size": [1000,500]}
                if hand_mode:
                    import math
                    jitter = math.sin(time.monotonic() * 3) * 0.02
                    names = [f"body_{i}" for i in range(17)]
                    for side, wrist in (("left", 9), ("right", 10)):
                        x, y = points[wrist][:2]
                        hand = [[x + jitter, y, 0.99]]
                        for finger in range(5):
                            hand.extend([[x + (finger - 2) * 8 + jitter, y - 20 - joint * 9, 0.99] for joint in range(4)])
                        if side == "left":
                            hand[8][0] += finger_delta
                        points.extend(hand)
                        names.extend(f"{side}_hand_{name}" for name in HAND_NAMES)
                    payload["keypoint_names"] = names
                msg = String()
                msg.data = json.dumps(payload)
                pose.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.04)
            time.sleep(0.02)

    def speak(text, background=False):
        msg = String()
        msg.data = json.dumps({"text": text, "background": background, "priority": 0,
                               "log_type": True, "audio_start_ts": time.time(),
                               "audio_end_ts": time.time(), "audio_duration_ms": 1200}, ensure_ascii=False)
        (background_asr if background else asr).publish(msg)

    def await_phase(phase, seconds, publish_pose=False):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            pump(0.1, publish_pose=publish_pose)
            if control("info").get("phase") == phase:
                return
        raise AssertionError((phase, plugin.info(), words))

    try:
        saved = {**DEFAULTS, "asr_topic": "", "pose_topic": "", "host_interaction": host_interaction}
        assert control("config", saved)["status"] == "configured"
        result = control("start", {"input_topic": "/wooden_man_test/asr",
                                   "input_topics": ["/wooden_man_test/asr", "/wooden_man_test/pose"]})
        assert result["state"] == "running", result
        worker_pid = plugin.process.pid
        assert control("config", saved)["state"] == "running"
        assert plugin.process.pid == worker_pid, "Repeated auto-config restarted the game"
        pump(2)
        assert not node.get_publishers_info_by_topic("/smoke/wooden_man/actions"), "Game must never publish robot actions"
        assert not words and plugin.info()["phase"] == "waiting"
        if host_interaction:
            assert "/wooden_man_test/asr_background" in plugin.info()["input_topics"]
            speak("我们来玩木头人游戏吧", background=True)
            pump(.3)
            assert plugin.info()["phase"] == "waiting", "Background must not wake the game"

            def wait_speech():
                pump(max(saved["cue_seconds"], len(clean_text(words[-1]))/4)+.4, publish_pose=True)

            def start_group():
                nonlocal registering
                registering = True
                speak("小范小范，我们来玩一二三木头人游戏吧")
                await_phase("rules_question", 4, publish_pose=True)
                wait_speech()
                speak("不需要，直接开始", background=True)
                await_phase("player_count", 4, publish_pose=True)
                wait_speech()
                speak("两个人", background=True)
                await_phase("preparing", 22, publish_pose=True)
                assert plugin.info()["registered_players"] == 2
                registering = False
                await_phase("countdown", 15, publish_pose=True)
                await_phase("watch", 12, publish_pose=True)

            start_group()
            await_phase("won", 36, publish_pose=True)
            assert words.count("一，二，三，木头人！") == 3
            assert plugin.info()["registered_players"] == 2
            print("PASS: real DDS natural invitation -> background replies -> two-player registration -> three rounds; moving seated bystander ignored", flush=True)
            wait_speech()
            # wait_speech may have received the replay prompt; wait for it too.
            pump(5, publish_pose=True)
            start_group()
            pump(.3, publish_pose=True)
            pump(.8, dx=20, publish_pose=True)
            info = plugin.info()
            assert info["phase"] == "lost" and info["moved_player"] == 2 and info["moved_part"] == "left_arm", info
            assert any("2号玩家，你的左手臂" in word for word in words)
            assert images and not node.get_publishers_info_by_topic("/smoke/wooden_man/actions")
            print("PASS: player 2 anatomical loss speech + multi-person JPEG highlight, no robot actions", flush=True)
            control("reset")
            await_phase("waiting", 2)
            control("stop")
            print("PASS: real HTTP multiplayer config/start/info/reset/stop", flush=True)
            return
        speak("不要游戏开始")
        pump(0.3)
        assert plugin.info()["phase"] == "waiting"
        speak("游戏开始")
        await_phase("won", 42 if host_interaction else 29, publish_pose=True)
        assert words.count("一，二，三，木头人！") == 3, words
        assert words[-1] == "三轮完成，你赢了，游戏结束！", words
        print("PASS: DDS ASR wake, five-second wait, three stationary rounds", flush=True)

        speak("游戏开始")
        await_phase("watch", 20 if host_interaction else 12, publish_pose=True)
        pump(0.2, publish_pose=True)
        pump(0.8, dx=20, publish_pose=True)
        assert plugin.info()["phase"] == "lost", plugin.info()
        assert words[-1].startswith("你的") and words[-1].endswith("动了，游戏结束。"), words
        assert images and plugin.info()["moved_part"] and plugin.info()["highlight_indices"]
        print("PASS: DDS Pose movement -> loss speech", flush=True)

        control("stop")
        control("config", {"detect_fingers": True, "sensitivity": "standard",
                           "cue_seconds": 0.2, "settle_seconds": 0.1, "watch_seconds": 2.4})
        assert control("start", {"input_topics": ["/wooden_man_test/asr", "/wooden_man_test/pose"]})["state"] == "running"
        hand_mode = True
        # Restarted DDS endpoints can take time to rediscover, especially
        # under ARM emulation. Retry only while still waiting for the utterance.
        for _ in range(25):
            speak("游戏开始")
            pump(0.2, publish_pose=True)
            if control("info")["phase"] != "waiting":
                break
        else:
            raise AssertionError("Restarted worker did not discover ASR publisher")
        await_phase("watch", 8, publish_pose=True)
        pump(0.25, publish_pose=True)
        finger_delta = 12
        pump(0.8, publish_pose=True)
        info = control("info")
        assert info["phase"] == "lost" and info["moved_part"] == "left_fingers", info
        assert "你的左手手指动了，游戏结束。" in words, words
        assert images
        print("PASS: DDS 59-joint fingers -> anatomical speech + CompressedImage JPEG highlight", flush=True)

        control("reset")
        await_phase("waiting", 2)
        control("stop")
        assert plugin.info()["state"] == "idle" and plugin.process is None
        print("PASS: real HTTP config -> start -> info -> reset -> stop, repeated config keeps worker", flush=True)
    finally:
        plugin.stop()
        server.shutdown()
        server.server_close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
