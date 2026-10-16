import asyncio
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'pico/4ultra'))
from ext_vr.runtime import DeviceRuntime
from ext_vr.protocol import ProtocolError
from ext_vr.plugin import ExtVrPlugin
from ext_vr.transport import BoundedWriter
from common.teleop_contract import validate_input


def frame(sequence=1, grip=1.0, x=1.0):
    pose = {"position": [x, 2.0, 3.0], "orientation": [0.0, 0.0, 0.0, 1.0]}
    return {
        "schema_version": 1,
        "sequence": sequence,
        "client_monotonic_ns": sequence * 50_000_000,
        "mode": "shadow",
        "deadman": bool(grip),
        "clutch_sequence": 0,
        "tracking": {"head": True, "left_controller": True, "right_controller": True},
        "head": pose,
        "left_controller": pose,
        "right_controller": pose,
        "controllers": {
            s: {"axes": [], "buttons": [0.2, grip]} for s in ("left", "right")
        },
    }


def ready_runtime():
    r = DeviceRuntime("vr-1")
    r.start()
    binding, epoch = r.bind_capture("device")
    return r, binding, epoch


def put(runtime, binding, epoch, *args):
    return runtime.submit_rtc_frame(
        frame(*args), authority=binding, rtc_generation=epoch
    )


def feedback(runtime, *, receipts=None, sequence=1):
    return {
        "schema": "motus.teleop.feedback/1",
        "instance_id": runtime.instance_id,
        "control_instance_id": "tianyi",
        "server_epoch": "server-1",
        "sequence": sequence,
        "emitted_monotonic_ns": runtime.clock_ns(),
        "clock_id": runtime.clock_id,
        "source_sequence": 1,
        "connection_epoch": runtime.generation,
        "space_epoch": runtime._space_epoch,
        "operator_session_id": None,
        "mapping_epoch": 0,
        "state": "ready",
        "reason": None,
        "capabilities": ["dual_arm"],
        "execution": {"armed": True, "started": False, "mode": "shadow"},
        "receipts": receipts or [],
    }


def test_regrip_is_next_pose_without_epoch_or_mapping_reset():
    runtime, binding, epoch = ready_runtime()
    a = put(runtime, binding, epoch, 1, 1.0, 1.0)
    released = put(runtime, binding, epoch, 2, 0.0, 5.0)
    again = put(runtime, binding, epoch, 3, 1.0, 6.0)
    assert (
        a["connection_epoch"]
        == released["connection_epoch"]
        == again["connection_epoch"]
    )
    assert a["space_epoch"] == again["space_epoch"]
    assert again["left"]["position"] == [-3.0, -6.0, 2.0]
    assert again["left"]["grip"] == 1
    assert "mapping_epoch" not in again
    assert runtime.take_latest()["sequence"] == 3
    assert runtime.take_latest() is None
    validate_input(
        again, instance_id="vr_1", clock_id=runtime.clock_id, now_ns=runtime.clock_ns()
    )


def test_new_space_rejects_old_rtc_and_never_reuses_stale_pose():
    runtime, binding, epoch = ready_runtime()
    old = put(runtime, binding, epoch)
    runtime.mark_capture_disconnected("device", epoch)
    assert runtime.take_latest() is None
    with pytest.raises(ProtocolError):
        put(runtime, binding, epoch, 2)
    other, new_epoch = runtime.bind_capture("device")
    new = put(runtime, other, new_epoch, 3)
    assert new["connection_epoch"] > old["connection_epoch"]
    assert new["space_epoch"] != old["space_epoch"]


def test_pose_input_is_unfiltered_across_grip_and_tracking_changes():
    runtime, binding, epoch = ready_runtime()
    put(runtime, binding, epoch, 1, 1.0, 1.0)
    out = put(runtime, binding, epoch, 2, 0.0, 5.0)
    assert out["left"]["position"][1] == -5
    assert out["left"]["grip"] == 0
    value = frame(3)
    value["tracking"]["left_controller"] = False
    value["left_controller"] = None
    out = runtime.submit_rtc_frame(value, authority=binding, rtc_generation=epoch)
    assert not out["left"]["tracked"] and out["left"]["position"] is None
    resumed = put(runtime, binding, epoch, 4, 1.0, 7.0)
    assert resumed["left"]["position"][1] == -7
    assert resumed["space_epoch"] == runtime._space_epoch


def test_bounded_writer_stalled_dds_keeps_only_latest():
    entered, release = threading.Event(), threading.Event()
    sent = []
    cleaned = []

    def send(value):
        sent.append(value)
        if len(sent) == 1:
            entered.set()
            release.wait(3)

    writer = BoundedWriter(send, lambda: cleaned.append(True))
    writer.publish({"kind": "input", "sequence": 0})
    assert entered.wait(1)
    for seq in range(1, 101):
        writer.publish({"kind": "input", "sequence": seq})
    writer.publish({"kind": "input", "sequence": 101})
    release.set()
    for _ in range(100):
        if len(sent) >= 2:
            break
        time.sleep(0.01)
    writer.close()
    assert [v["sequence"] for v in sent] == [0, 101]
    assert cleaned == [True]


def load_driver_module(name):
    path = Path(__file__).parents[1] / "pico/4ultra" / f"{name}.py"
    spec = importlib.util.spec_from_file_location("pico_" + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("port", [None, 17441])
def test_standalone_metadata_and_info_have_no_core_custom_dependency(tmp_path, port):
    import yaml

    root = Path(__file__).parents[1]
    metadata = yaml.safe_load((root / "pico/4ultra/driver.yaml").read_text())
    assert metadata["id"] == "pico-driver" and metadata["port"] == 15742
    plugin = ExtVrPlugin(
        {
            "state_dir": str(tmp_path),
            "public_wss_url": "wss://127.0.0.1:15741/ws/teleop-capture",
            **({"port": port} if port is not None else {}),
        },
        "pico",
    )
    try:
        tool = plugin.get_tool()
        assert tool["name"] == "teleop_device"
        assert "management_binding" not in json.dumps(tool)
        info = plugin.dispatch("info", {"instance_id": "card-vr"})
        assert info["topic_out"][0]["topic"] == "/teleop/command"
        assert "feedback_topic" not in info
        assert info["config"]["driver_installed"]
        assert set(tool["configSchema"]["properties"]) == {"management_pin", "install"}
        qr = tool["configSchema"]["properties"]["install"]
        assert qr["format"] == "qr" and qr["scope"] == "instance"
        assert qr["x-qr-options"] == [{"label": "下载与配对", "url": f"https://{{host}}:{port or 15741}/onboarding"}]
        assert "install" not in tool["configSchema"].get("required", [])
        assert "install" not in info["config"]
        assert tool["configSchema"]["properties"]["management_pin"]["x-sensitive"] is True
        assert plugin.instances == {}
    finally:
        plugin.close()


def test_tls_identity_is_persistent_and_not_core_key(tmp_path):
    prepare = load_driver_module("identity").prepare_config
    first = prepare({"state_dir": str(tmp_path), "public_host": "127.0.0.1"})
    cert = Path(first["tls_cert_file"]).read_bytes()
    second = prepare({"state_dir": str(tmp_path), "public_host": "127.0.0.1"})
    assert cert == Path(second["tls_cert_file"]).read_bytes()
    assert Path(second["tls_key_file"]).stat().st_mode & 0o777 == 0o600


@pytest.mark.parametrize("mode", [None, 0o755, 0o700])
def test_startup_prepares_private_state_and_preserves_identity(tmp_path, mode):
    from ext_vr.management_pin import ManagementPin

    root = tmp_path / "state"
    if mode is not None:
        root.mkdir(mode=mode)
        root.chmod(mode)
    prepare = load_driver_module("identity").prepare_config
    config = {"state_dir": str(root), "public_host": "127.0.0.1"}
    prepare(config)
    assert root.stat().st_mode & 0o777 == 0o700
    auth = ManagementPin(root / "management-pin.json")
    auth.configure("0123")
    pairing = root / "device.json"
    pairing.write_text('{"schema_version": 1, "capture": {}}')
    pairing.chmod(0o600)
    before = {p.relative_to(root): (p.read_bytes(), p.stat().st_mode) for p in root.rglob("*") if p.is_file()}
    root.chmod(0o755)
    prepare(config)
    assert root.stat().st_mode & 0o777 == 0o700
    assert before == {p.relative_to(root): (p.read_bytes(), p.stat().st_mode) for p in root.rglob("*") if p.is_file()}
    restored = ManagementPin(auth.path)
    restored.authorize(restored.login("0123"))


@pytest.mark.parametrize("invalid", ["symlink", "file", "owner"])
def test_startup_rejects_unsafe_state_without_changing_target(tmp_path, monkeypatch, invalid):
    identity = load_driver_module("identity")
    root = tmp_path / "state"
    target = tmp_path / "target"
    target.mkdir(mode=0o755)
    marker = target / "keep"
    marker.write_text("unchanged")
    if invalid == "symlink":
        root.symlink_to(target, target_is_directory=True)
    elif invalid == "file":
        root.write_text("unchanged")
    else:
        root.mkdir(mode=0o755)
        monkeypatch.setattr(identity.os, "geteuid", lambda: root.stat().st_uid + 1)
    with pytest.raises(ValueError, match="pico_state_directory_"):
        identity.prepare_config({"state_dir": str(root), "public_host": "127.0.0.1"})
    assert target.stat().st_mode & 0o777 == 0o755
    assert marker.read_text() == "unchanged"
    if invalid == "file":
        assert root.read_text() == "unchanged"
    elif invalid == "owner":
        assert root.stat().st_mode & 0o777 == 0o755
        assert list(root.iterdir()) == []


def test_missing_or_nonlocal_dds_profile_cannot_silently_start(tmp_path):
    validate = load_driver_module("identity").validate_dds_profile
    source = Path(__file__).parents[1] / "pico/4ultra/dds-local.xml"
    validate(source)
    with pytest.raises(OSError):
        validate(tmp_path / "absent.xml")
    for old, new in (
        ("127.0.0.1", "192.168.1.2"),
        ("<useBuiltinTransports>false", "<useBuiltinTransports>true"),
    ):
        candidate = tmp_path / "candidate.xml"
        candidate.write_text(source.read_text().replace(old, new))
        with pytest.raises(ValueError, match="loopback_profile"):
            validate(candidate)


def test_named_controls_and_extensions_survive_rtc_to_dds_without_filtering(monkeypatch):
    runtime, binding, epoch = ready_runtime()
    wire = frame(1)
    controls = {
        "buttons": {
            "trigger": {"available": True, "value": 0.4, "pressed": False, "touched": True},
            "x": {"available": True, "pressed": True, "touched": False},
            "thumbstick": {"available": False},
        },
        "axes": {"thumbstick": {"available": True, "value": [-0.7, 0.3]}},
        "future": {"vendor": "preserved"},
    }
    wire["controllers"]["left"]["controls"] = controls
    wire["extensions"] = {"future_device": {"battery": 0.8}}
    out = runtime.submit_rtc_frame(wire, authority=binding, rtc_generation=epoch)
    assert out["left"]["controls"] == controls
    assert out["extensions"] == wire["extensions"]
    assert "controls" not in out["right"]
    assert out["left"]["grip"] == 1.0
    wire["controllers"]["left"]["controls"]["axes"]["thumbstick"]["value"][0] = 0
    assert out["left"]["controls"]["axes"]["thumbstick"]["value"][0] == -0.7
    # Replace only middleware classes: run the actual card publish loop,
    # bounded writer and ROS String serializer before validating the wire JSON.
    published, bindings, sent = [], [], threading.Event()

    class Node:
        def __init__(self, *args, **kwargs):
            pass

        def create_publisher(self, message_type, topic, qos):
            bindings.append((topic, qos.depth))

            def publish(message):
                published.append(json.loads(message.data))
                sent.set()
            return SimpleNamespace(publish=publish)

        def create_subscription(self, message_type, topic, callback, qos):
            return SimpleNamespace(topic=topic, receive=callback)

        def destroy_node(self):
            pass

    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=Node))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        QoSProfile=lambda **kwargs: SimpleNamespace(**kwargs),
        ReliabilityPolicy=SimpleNamespace(RELIABLE="reliable"),
        HistoryPolicy=SimpleNamespace(KEEP_LAST="keep_last"),
        DurabilityPolicy=SimpleNamespace(VOLATILE="volatile")))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=SimpleNamespace))
    from ext_vr.transport import RosTransport
    executor = SimpleNamespace(context=SimpleNamespace(get_domain_id=lambda: 42),
                               add_node=lambda node: None, remove_node=lambda node: None)
    received = []
    transport = RosTransport(executor, "ignored-brand", "vr-1", received.append)
    assert transport.sub.topic == "/teleop/state"
    transport.sub.receive(SimpleNamespace(data='{"state":"preview"}'))
    transport.sub.receive(SimpleNamespace(data='invalid json'))
    assert received == [{"state": "preview"}, {}]

    async def publish():
        task = asyncio.create_task(ExtVrPlugin._publish(None, {"runtime": runtime, "transport": transport}))
        try:
            assert await asyncio.to_thread(sent.wait, 2)
        finally:
            runtime.stop()
            await task
            transport.close()
    asyncio.run(publish())
    assert bindings == [("/teleop/command", 16)]
    assert published == [out]
    validate_input(published[0], instance_id="vr_1", clock_id=runtime.clock_id,
                   now_ns=published[0]["received_monotonic_ns"])


@pytest.mark.parametrize("controls", [
    {"buttons": {"a": {"available": False, "pressed": False}}},
    {"buttons": {"a": {"available": True, "pressed": 1}}},
    {"axes": {"thumbstick": {"available": True, "value": [1, 2]}}},
    {"axes": {"thumbstick": {"available": True, "value": [0]}}},
])
def test_invalid_optional_controls_do_not_bypass_wire_validation(controls):
    runtime, binding, epoch = ready_runtime()
    wire = frame(1)
    wire["controllers"]["left"]["controls"] = controls
    with pytest.raises(ProtocolError):
        runtime.submit_rtc_frame(wire, authority=binding, rtc_generation=epoch)
