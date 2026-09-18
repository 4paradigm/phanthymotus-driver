"""UBTECH U1 Pro ROS 2 adapter.

The cards in this module are Agent capabilities, not a mirror of every SDK
management call. The robot's public ROS graph provides useful contracts:
16 kHz microphone PCM, live speaker PCM input, motion playback, and audio events.
"""

from __future__ import annotations

import json
import time
import uuid
from typing import Any

from common.vendor_runtime import action_schema, jsonable, tool


SERVICE_TIMEOUT = 3.0
MIC_TOPIC = "/audio/sense/audio_data_to_asr"
SPEAKER_TOPIC = "/sys/device/audio_out/raw"


def _sensor_schema() -> dict:
    return {
        "type": "object",
        "properties": {"action": {"type": "string", "enum": ["start", "stop", "info"]}},
        "required": ["action"],
    }


def _event_json(message: Any) -> str:
    return json.dumps(jsonable(message), ensure_ascii=False)


class U1Nodes:
    def __init__(self, config: dict, namespace: str, ros) -> None:
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from audio_msgs.msg import AudioChunk, AudioInData, AudioOutData, DoaEvent, FinishList, MainWakeupWord, WakeupEvent, WakeupState
        from audio_msgs.srv import EnableAudioIn, SetAudioVolume
        from coze_msgs.srv import InterruptActionAudio, PlayResources
        from uworld_action_msgs.srv import GetMotionInfoList, PlayMotion

        self.robot = Node("u1_pro_driver", context=ros.ctx_robot)
        self.core = Node("u1_pro_bridge", namespace=namespace, context=ros.ctx_core)
        ros.executor_robot.add_node(self.robot)
        ros.executor_core.add_node(self.core)
        self.namespace = namespace
        self.mic_topic = f"/{namespace}/mic/audio"
        self.AudioChunk = AudioChunk
        self.AudioOutData = AudioOutData
        self.String = String
        self._speaker_publisher = self.robot.create_publisher(AudioOutData, SPEAKER_TOPIC, 10)
        self._speaker_subscription = None
        self._speaker_uuid = ""

        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._mic_publisher = self.core.create_publisher(AudioChunk, self.mic_topic, best_effort)
        self._event_publishers = {}
        self._robot_subscriptions = []
        for name, msg_type, topic in (
            ("main_wakeup_word", MainWakeupWord, "/audio/sense/main_wakeup_word"),
            ("wakeup_event", WakeupEvent, "/audio/sense/wakeup_event"),
            ("wakeup_state", WakeupState, "/audio/sense/wakeup_state"),
            ("doa_event", DoaEvent, "/audio/sense/doa_event"),
            ("playback_state", FinishList, "/sys/device/audio_out/finish_list"),
        ):
            output_topic = f"/{namespace}/u1_pro/{name}"
            self._event_publishers[name] = self.core.create_publisher(String, output_topic, reliable)
            self._robot_subscriptions.append(self.robot.create_subscription(msg_type, topic, self._event_callback(name), reliable))
        self._robot_subscriptions.append(self.robot.create_subscription(AudioInData, MIC_TOPIC, self._mic_callback, best_effort))

        self._clients = {
            "mic_enable": self.robot.create_client(EnableAudioIn, "/sys/device/audio_in/enable"),
            "volume": self.robot.create_client(SetAudioVolume, "/sys/device/audio_out/set_volume"),
            "motion_list": self.robot.create_client(GetMotionInfoList, "/action/controller/get_motion_info_list"),
            "play_motion": self.robot.create_client(PlayMotion, "/action/controller/pay_motion"),
            "play_resource": self.robot.create_client(PlayResources, "/audio/stream/coze/play_resources"),
            "interrupt": self.robot.create_client(InterruptActionAudio, "/audio/stream/coze/interrupt_action_audio"),
        }

    def _event_callback(self, name: str):
        def callback(message):
            output = self.String()
            output.data = _event_json(message)
            self._event_publishers[name].publish(output)
        return callback

    def _mic_callback(self, message) -> None:
        if message.sample_rate != 16000 or message.channels != 1:
            return
        chunk = self.AudioChunk()
        chunk.format = "audio/pcm-16k"
        chunk.data = list(message.data.data)
        self._mic_publisher.publish(chunk)

    def call(self, name: str, request) -> Any:
        client = self._clients[name]
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            raise RuntimeError(f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        deadline = time.monotonic() + SERVICE_TIMEOUT
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise TimeoutError(f"service timeout: {client.srv_name}")
        return future.result()

    def set_mic_enabled(self, enabled: bool) -> dict:
        from std_msgs.msg import Header
        from audio_msgs.srv import EnableAudioIn
        request = EnableAudioIn.Request()
        request.header = Header()
        request.enable = enabled
        return jsonable(self.call("mic_enable", request))

    def set_volume(self, volume: int) -> dict:
        from audio_msgs.srv import SetAudioVolume
        request = SetAudioVolume.Request()
        request.volume = max(0, min(100, int(volume)))
        return jsonable(self.call("volume", request))

    def close_speaker_subscription(self) -> None:
        if self._speaker_subscription is not None:
            self.core.destroy_subscription(self._speaker_subscription)
            self._speaker_subscription = None

    def connect_speaker(self, input_topic: str) -> dict:
        self.close_speaker_subscription()
        self._speaker_uuid = f"u1-{uuid.uuid4().hex}"
        self._speaker_subscription = self.core.create_subscription(self.AudioChunk, input_topic, self._speaker_callback, 10)
        return {"state": "running", "input_topic": input_topic, "robot_topic": SPEAKER_TOPIC}

    def _speaker_callback(self, message) -> None:
        output = self.AudioOutData()
        output.uuid = self._speaker_uuid
        output.data.data = list(message.data)
        self._speaker_publisher.publish(output)

    def close(self) -> None:
        self.close_speaker_subscription()
        self.robot.destroy_node()
        self.core.destroy_node()


class MicPlugin:
    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = False

    def get_tool(self):
        return tool("mic", "sensor", "U1 Pro microphone array: live 16 kHz mono PCM audio for ASR.", _sensor_schema(), topic_out=[{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}])

    def start(self):
        self.nodes.set_mic_enabled(True)
        self.running = True

    def stop(self):
        self.nodes.set_mic_enabled(False)
        self.running = False

    def dispatch(self, action, args):
        if action == "start":
            self.start()
        elif action == "stop":
            self.stop()
        return {"state": "running" if self.running else "idle", "topic_out": [{"topic": self.nodes.mic_topic, "format": "audio/pcm-16k"}]}


class SpeakerPlugin:
    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes
        self.running = False
        self.input_topic = ""

    def get_tool(self):
        actions = {
            "start": (["input_topic"], "Start playing the connected PCM audio stream."),
            "set_volume": (["volume"], "Set U1 Pro speaker volume from 0 to 100."),
            "stop": ([], "Stop consuming the connected audio stream."),
            "info": ([], "Read speaker connection state."),
        }
        return {"name": "speaker", "type": "actuator", "multiInstance": False, "description": "U1 Pro speaker. Connect an audio/pcm-16k stream such as TTS or mic audio, then start playback.", "inputSchema": action_schema(actions, {"input_topic": {"type": "string", "description": "Connected audio/pcm-16k input topic"}, "volume": {"type": "integer", "minimum": 0, "maximum": 100}}), "topic_in": [{"format": "audio/pcm-16k"}]}

    def start(self):
        pass

    def stop(self):
        self.nodes.close_speaker_subscription()
        self.running = False

    def dispatch(self, action, args):
        if action == "start":
            topic = str(args.get("input_topic", "")).strip()
            if not topic:
                raise ValueError("speaker.start requires input_topic")
            self.input_topic = topic
            result = self.nodes.connect_speaker(topic)
            self.running = True
            return result
        if action == "set_volume":
            return self.nodes.set_volume(args.get("volume", 100))
        if action == "stop":
            self.stop()
            return {"state": "idle"}
        return {"state": "running" if self.running else "idle", "input_topic": self.input_topic}


class AudioPlugin:
    def __init__(self, nodes: U1Nodes):
        self.nodes = nodes

    def get_tool(self):
        actions = {
            "list_actions": ([], "List U1 Pro motions that can be played."),
            "play_action": (["motion_type", "motion_name"], "Play a named U1 Pro motion."),
            "play_resource": (["path"], "Play a vendor audio resource directory."),
            "stop": ([], "Interrupt the current U1 Pro audio or motion."),
        }
        properties = {"motion_type": {"type": "integer", "enum": [0, 1, 2], "description": "0 chat, 1 special, 2 command motion"}, "motion_name": {"type": "string", "description": "Motion name returned by list_actions"}, "path": {"type": "string", "description": "Robot audio resource directory"}}
        return tool("audio", "actuator", "U1 Pro preset audio and motion playback. List actions first, then play a returned motion or vendor audio resource.", action_schema(actions, properties))

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        if action == "list_actions":
            from uworld_action_msgs.srv import GetMotionInfoList
            return jsonable(self.nodes.call("motion_list", GetMotionInfoList.Request()))
        if action == "play_action":
            from uworld_action_msgs.srv import PlayMotion
            request = PlayMotion.Request()
            request.motion_type = int(args["motion_type"])
            request.motion_name = str(args["motion_name"])
            return jsonable(self.nodes.call("play_motion", request))
        if action == "play_resource":
            from coze_msgs.srv import PlayResources
            request = PlayResources.Request()
            request.path = str(args["path"])
            return jsonable(self.nodes.call("play_resource", request))
        if action == "stop":
            from coze_msgs.srv import InterruptActionAudio
            return jsonable(self.nodes.call("interrupt", InterruptActionAudio.Request()))
        raise ValueError(f"unknown audio action: {action}")


class EventPlugin:
    def __init__(self, nodes: U1Nodes, name: str, description: str):
        self.nodes, self.name, self.description = nodes, name, description

    def get_tool(self):
        return tool(self.name, "sensor", self.description, _sensor_schema(), topic_out=[{"topic": f"/{self.nodes.namespace}/u1_pro/{self.name}", "format": "data/json"}])

    def start(self):
        pass

    def stop(self):
        pass

    def dispatch(self, action, args):
        return {"state": "idle" if action == "stop" else "running", "topic_out": [{"topic": f"/{self.nodes.namespace}/u1_pro/{self.name}", "format": "data/json"}]}


def build_plugins(config: dict, namespace: str, ros) -> list:
    nodes = U1Nodes(config, namespace, ros)
    plugins = [MicPlugin(nodes), SpeakerPlugin(nodes), AudioPlugin(nodes)]
    descriptions = {
        "main_wakeup_word": "Main wake-word event recognized by the U1 Pro.",
        "wakeup_event": "U1 Pro wake-up recognition event; does not guarantee a follow-up conversation.",
        "wakeup_state": "Current U1 Pro wake-up state.",
        "doa_event": "Microphone-array sound direction with azimuth and confidence.",
        "playback_state": "U1 Pro audio completion or interruption events for current audio UUIDs.",
    }
    plugins.extend(EventPlugin(nodes, name, description) for name, description in descriptions.items())
    return plugins
