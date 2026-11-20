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
        {}, "bumi", None, None, store=store or Store(), worker=worker or Worker(),
        clock=lambda: 100.0, approval_verifier=lambda *_: True)


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
    assert schema["inputSchema"]["x-action-params"]["go_live"]["params"] == []
    assert "approval_code" not in schema["inputSchema"]["properties"]
    assert "end_live" in schema["inputSchema"]["x-action-params"]


def test_canvas_gear_config_reaches_driver_but_never_becomes_llm_tool():
    """Mirror Agent Core's configSchema -> saved tool-config -> tools/call contract.

    Agent Core's _to_openai_schema turns EVERY x-action-params entry into an
    LLM-callable tool; Canvas config is sent separately by apply_tool_config.
    """
    store, worker = Store(), Worker()
    subject = card(store, worker)
    tool = subject.get_tool()
    input_schema = tool["inputSchema"]
    config_schema = tool["configSchema"]
    params = input_schema["x-action-params"]

    # Keep config action dispatchable by Agent Core's private config transport,
    # but never advertise it as a callable LLM action.
    assert "config" in input_schema["properties"]["action"]["enum"]
    assert "config" not in params
    assert "rtmp_url" not in input_schema["properties"]
    assert all("rtmp_url" not in action.get("params", []) for action in params.values())
    assert set(params) == {"start", "stop", "info", "go_live", "end_live"}

    gear = config_schema["properties"]["rtmp_url"]
    assert config_schema["required"] == []
    assert gear["scope"] == "instance"
    assert gear["x-sensitive"] and gear["format"] == "password"

    # Canvas stores the instance-scoped value, and Agent Core replays the
    # saved config directly to the MCP action before the card is started.
    saved_config = {"rtmp_url": URL}
    args = {"instance_id": "card-1", **saved_config}
    configured = subject.dispatch("config", args)
    assert configured["ok"] is True
    assert store.saves == 1 and store.value == URL
    assert URL not in str(configured)
    assert URL not in str(subject.dispatch("info", {}))
    assert worker.calls == []  # Config does not start a public live broadcast.

    # Agent Core restores the same saved settings when the driver restarts.
    restored_worker = Worker()
    restored = card(store, restored_worker)
    assert restored.dispatch("start", {})["configured"] is True
    assert restored_worker.calls == []
    # Restored credentials must not re-authorize a new public broadcast.
    assert restored.dispatch("go_live", {})["error_code"] == "APPROVAL_REQUIRED"
    assert restored_worker.received_url is None


def test_saved_url_alone_cannot_start_broadcast():
    store, worker = Store(URL), Worker()
    subject = LiveStreamPlugin({}, "bumi", None, None, store=store,
                               worker=worker, clock=lambda: 100.0,
                               approval_verifier=lambda *_: True)
    result = subject.dispatch("go_live", {})
    assert result["error_code"] == "APPROVAL_REQUIRED"
    assert worker.calls == []
    assert URL not in str(result)


def test_paste_new_url_arms_once_without_extra_code():
    store, worker = Store(), Worker()
    now = [100.0]
    subject = LiveStreamPlugin({}, "bumi", None, None, store=store,
                               worker=worker, clock=lambda: now[0],
                               approval_verifier=lambda *_: True)
    assert subject.dispatch("config", {"rtmp_url": URL})["ok"]
    assert subject.dispatch("go_live", {})["state"] == "starting"
    assert subject.dispatch("end_live", {})["state"] == "idle"
    assert subject.dispatch("go_live", {})["error_code"] == "APPROVAL_REQUIRED"
    # Canvas config replay cannot arm again with the same URL.
    assert subject.dispatch("config", {"rtmp_url": URL})["ok"]
    assert subject.dispatch("go_live", {})["error_code"] == "APPROVAL_REQUIRED"
    changed_url = URL.replace("seq=123", "seq=456")
    assert subject.dispatch("config", {"rtmp_url": changed_url})["ok"]
    now[0] += 301
    assert subject.dispatch("go_live", {})["error_code"] == "APPROVAL_REQUIRED"


def test_explicit_operator_rearm_same_url_does_not_allow_background_replay():
    store, worker = Store(), Worker()
    now = [100.0]
    subject = LiveStreamPlugin({}, "bumi", None, None, store=store, worker=worker,
                               clock=lambda: now[0], approval_verifier=lambda *_: True)
    first = subject.dispatch("config", {"rtmp_url": URL})
    assert first["ready_to_go_live"] is True
    now[0] += 301
    assert subject.dispatch("go_live", {})["error_code"] == "APPROVAL_REQUIRED"
    replay = subject.dispatch("config", {"rtmp_url": URL})
    assert replay["ready_to_go_live"] is False
    assert subject.dispatch("go_live", {})["error_code"] == "APPROVAL_REQUIRED"
    allowed = subject.dispatch("config", {"rtmp_url": URL, "authorize_start": True})
    assert allowed["ok"] and allowed["ready_to_go_live"]
    assert subject.dispatch("go_live", {})["state"] == "starting"
    assert subject.dispatch("end_live", {})["state"] == "idle"
    assert subject.dispatch("go_live", {})["error_code"] == "APPROVAL_REQUIRED"


def test_raw_mcp_cannot_configure_or_publish_without_approval():
    store, worker = Store(), Worker()
    def verify(action, url, proof):
        return proof == "signed:" + action

    subject = LiveStreamPlugin(
        {}, "bumi", None, None, store=store, worker=worker,
        clock=lambda: 100.0, approval_verifier=verify,
    )
    assert subject.dispatch("config", {"rtmp_url": URL})["error_code"] == "APPROVAL_REQUIRED"
    assert store.saves == 0
    assert subject.dispatch("config", {
        "rtmp_url": URL, "operator_proof": "signed:config",
        "authorize_start": True,
    })["ready_to_go_live"] is True
    # The five-minute approval window alone is insufficient to start.
    assert subject.dispatch("go_live", {})["error_code"] == "APPROVAL_REQUIRED"
    assert worker.calls == []
    assert subject.dispatch("go_live", {"operator_proof": "signed:go_live"})["state"] == "starting"
    assert subject.dispatch("end_live", {})["state"] == "idle"
    assert subject.dispatch("go_live", {"operator_proof": "signed:go_live"})["error_code"] == "APPROVAL_REQUIRED"
    # Replays without signed approval do not permit a second session.
    assert subject.dispatch("config", {"rtmp_url": URL})["ready_to_go_live"] is False
    assert subject.dispatch("config", {"rtmp_url": URL, "authorize_start": True})["error_code"] == "APPROVAL_REQUIRED"
    assert subject.dispatch("config", {"rtmp_url": URL.replace("seq=123", "seq=999")})["error_code"] == "APPROVAL_REQUIRED"


def test_xiaohongshu_gear_configuration_without_broadcast():
    xhs_url = ("rtmp://live-push.xhscdn.com/live/123456789"
               "?txSecret=example&txTime=ffffffff")
    store, worker = Store(), Worker()
    subject = card(store, worker)
    result = subject.dispatch("config", {"rtmp_url": xhs_url})
    assert result["ok"]
    assert subject.dispatch("info", {})["platform"] == "xiaohongshu"
    assert worker.calls == []
    assert xhs_url not in str(subject.dispatch("info", {}))
    assert subject.dispatch("go_live", {})["state"] == "starting"
    assert worker.received_url == xhs_url


def test_preflight_failure_does_not_consume_recent_paste():
    class NotReadyWorker(Worker):
        def __init__(self):
            super().__init__()
            self.ready = False

        def start(self, url):
            if not self.ready:
                return {"state": "error", "error_code": "MIC_UNAVAILABLE"}
            return super().start(url)

    worker = NotReadyWorker()
    subject = card(worker=worker)
    assert subject.dispatch("config", {"rtmp_url": URL})["ok"]
    assert subject.dispatch("go_live", {})["error_code"] == "MIC_UNAVAILABLE"
    worker.ready = True
    assert subject.dispatch("go_live", {})["state"] == "starting"


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
    assert subject.dispatch("config", {"rtmp_url": "rtmp://other.site/x"})["error_code"] == "INVALID_STREAM_URL"
    result = subject.dispatch("config", {"rtmp_url": URL, "instance_id": "card-1"})
    assert result["configured"] is True
    assert store.value == URL
    assert store.saves == 1
    assert subject.dispatch("go_live", {})["state"] == "starting"
    assert worker.received_url == URL
    same = subject.dispatch("config", {"rtmp_url": URL})
    assert same["ok"] is True and same["state"] == "streaming"
    assert store.saves == 1
    assert subject.dispatch("config", {"rtmp_url": URL.replace("seq=123", "seq=456")})["error_code"] == "STREAM_ACTIVE"
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
                               store=Store(), worker=worker, clock=lambda: 100.0)
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

def test_auth_status_exposes_expiry_without_url():
    store, worker = Store(), Worker()
    now = [100.0]
    subject = LiveStreamPlugin({}, "bumi", None, None, store=store,
                               worker=worker, clock=lambda: now[0],
                               approval_verifier=lambda *_: True)
    assert subject.dispatch("info", {})["ready_to_go_live"] is False
    result = subject.dispatch("config", {"rtmp_url": URL})
    assert result["ready_to_go_live"] is True
    assert subject.dispatch("info", {})["authorization_expires_in_s"] == 300.0
    now[0] += 301
    assert subject.dispatch("info", {})["ready_to_go_live"] is False
    repeated = subject.dispatch("config", {"rtmp_url": URL})
    assert repeated["ok"] is True
    assert repeated["ready_to_go_live"] is False
    assert "重新授权" in repeated["message"]
    assert URL not in str(repeated)


def test_go_live_initializes_audio_subscription_when_canvas_start_was_skipped(monkeypatch):
    import live_stream.card as module
    audio = module.AudioBuffer()
    calls = []
    def attach(executor, namespace, buffer):
        calls.append((executor, namespace))
        buffer.push(b"\\x00\\x00" * 640, "audio/pcm-16k")
        return object()
    monkeypatch.setattr(module, "attach_audio_subscription", attach)
    executor = object()
    subject = LiveStreamPlugin({}, "bumi", executor, None, store=Store(),
                               audio=audio, worker=Worker(), clock=lambda: 100.0,
                               approval_verifier=lambda *_: True)
    assert subject.dispatch("config", {"rtmp_url": URL})["ready_to_go_live"]
    assert subject.dispatch("go_live", {})["state"] == "starting"
    assert calls == [(executor, "bumi")]
