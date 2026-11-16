"""Card schema, user guidance, explicit operations, secret absence."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from live_stream import LiveStreamPlugin

URL = "rtmp://tx.direct.huya.com/huyalive/secret?seq=123&type=simple"


class Store:
    def __init__(self, value=None):
        self.value = value
        self.saves = 0

    def read(self):
        return self.value

    def save(self, value):
        self.value = value
        self.saves += 1


class Worker:
    def __init__(self):
        self.state = "idle"
        self.calls = []
        self.received_url = None

    def info(self):
        return {"state": self.state, "video_frames": 0}

    def start(self, url):
        self.received_url = url
        self.calls.append("go_live")
        if self.state == "streaming":
            return {"state": "streaming"}
        self.state = "streaming"
        return {"state": "starting"}

    def stop(self):
        self.state = "idle"
        self.calls.append("end_live")
        return {"state": "idle"}


def card(store=None, worker=None):
    return LiveStreamPlugin(
        {}, "bumi", None, None, store=store or Store(), worker=worker or Worker())


def test_card_is_huya_only_actuator_and_user_facing():
    schema = card().get_tool()
    assert schema["name"] == "live_stream"
    assert schema["type"] == "actuator"
    assert schema["multiInstance"] is False
    field = schema["configSchema"]["properties"]["rtmp_url"]
    assert field["scope"] == "instance"
    assert field["format"] == "password"
    assert field["x-sensitive"] is True
    assert "虎牙" in field["description"]
    assert "复制" in schema["description"]
    assert "go_live" in schema["inputSchema"]["x-action-params"]
    assert "end_live" in schema["inputSchema"]["x-action-params"]


def test_start_is_lifecycle_only_not_public_broadcast():
    worker = Worker()
    subject = card(worker=worker)
    assert subject.start() == {"state": "ready"}
    assert subject.dispatch("start", {})["state"] == "ready"
    assert worker.calls == []


def test_config_and_explicit_broadcast_do_not_leak_url():
    store, worker = Store(), Worker()
    subject = card(store, worker)
    assert subject.dispatch("go_live", {})["error_code"] == "MISSING_URL"
    assert subject.dispatch("config", {"rtmp_url": "rtmp://other.site/x"})["error_code"] == "INVALID_HUYA_URL"
    result = subject.dispatch("config", {"rtmp_url": URL, "instance_id": "card-1"})
    assert result["configured"] is True
    assert store.value == URL
    assert store.saves == 1
    assert subject.dispatch("go_live", {})["state"] == "starting"
    assert worker.received_url == URL
    assert subject.dispatch("config", {"rtmp_url": URL})["error_code"] == "STREAM_ACTIVE"
    assert subject.dispatch("end_live", {})["state"] == "idle"
    assert subject.dispatch("info", {})["configured"] is True
    assert URL not in str(result)
    assert URL not in str(subject.dispatch("info", {}))
    assert subject.dispatch("stop", {})["state"] == "idle"


def test_restart_loads_saved_url_without_go_live():
    saved = Store(URL)
    worker = Worker()
    subject = card(saved, worker)
    assert subject.dispatch("start", {})["configured"] is True
    assert worker.calls == []


def test_unknown_action_is_safe():
    assert card().dispatch("unknown", {})["error_code"] == "UNKNOWN_ACTION"

def test_canvas_stop_start_recreates_audio_subscription(monkeypatch):
    import live_stream.card as module

    class Node:
        def __init__(self):
            self.destroyed = False

        def destroy_node(self):
            self.destroyed = True

    class Executor:
        def __init__(self):
            self.removed = []

        def remove_node(self, node):
            self.removed.append(node)

    executor = Executor()
    nodes = []

    def connect_audio(_executor, _namespace, _audio):
        node = Node()
        nodes.append(node)
        return node

    monkeypatch.setattr(module, "attach_audio_subscription", connect_audio)
    worker = Worker()
    subject = LiveStreamPlugin({}, "bumi", executor, None,
                               store=Store(), worker=worker)
    assert subject.dispatch("start", {})["state"] == "ready"
    assert len(nodes) == 1
    assert subject.dispatch("end_live", {})["state"] == "idle"
    assert not nodes[0].destroyed
    assert subject.dispatch("stop", {})["state"] == "idle"
    assert nodes[0].destroyed
    assert executor.removed == nodes[:1]
    assert subject.dispatch("start", {})["state"] == "ready"
    assert len(nodes) == 2
    assert not nodes[1].destroyed
    subject.stop()
    assert nodes[1].destroyed
