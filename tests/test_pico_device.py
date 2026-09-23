import asyncio
import copy
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
from ext_vr.management import OperatorCommands
from ext_vr.transport import BoundedWriter
from common.teleop_contract import validate_input, validate_operation


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


def test_bounded_writer_stalled_dds_keeps_only_latest_and_stop_priority():
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
    for index in range(16):
        writer.publish(
            {"kind": "operation", "request_id": str(index), "action": "begin"}
        )
    with pytest.raises(ValueError, match="operator_queue_full"):
        writer.publish(
            {"kind": "operation", "request_id": "overflow", "action": "begin"}
        )
    writer.publish({"kind": "operation", "request_id": "stop", "action": "stop"})
    writer.publish({"kind": "input", "sequence": 101})
    release.set()
    for _ in range(100):
        if len(sent) >= 3:
            break
        time.sleep(0.01)
    writer.close()
    assert [v.get("request_id", v.get("sequence")) for v in sent] == [0, "stop", 101]
    assert cleaned == [True]


def test_operation_retry_is_identical_and_stop_bypasses_finish():
    async def run():
        runtime, binding, epoch = ready_runtime()
        put(runtime, binding, epoch)
        connection = SimpleNamespace(
            capture_id="device", connection_id="conn", events=asyncio.Queue(maxsize=8)
        )
        manager = SimpleNamespace(
            _connection=connection, presence_expired=lambda c: False
        )
        sent = []
        ops = OperatorCommands(manager, runtime, lambda v: sent.append(v))
        ops.bind(True)
        ops.feedback(feedback(runtime))
        request = {
            "connection_id": "conn",
            "request_id": "finish-1",
            "action": "finish",
        }
        first = await ops.submit(connection, request)
        assert await ops.submit(connection, request) == first
        await asyncio.sleep(0.13)
        assert len(sent) >= 2 and sent[0] == sent[1]
        validate_operation(
            sent[0],
            instance_id="vr-1",
            clock_id=runtime.clock_id,
            now_ns=runtime.clock_ns(),
        )
        await ops.submit(
            connection,
            {"connection_id": "conn", "request_id": "stop-1", "action": "stop"},
        )
        assert sent[-1]["action"] == "stop"
        assert ops.receipts[("device", "finish-1")]["error"] == "superseded_by_stop"
        ops.feedback(
            feedback(
                runtime,
                receipts=[
                    {
                        "request_id": "stop-1",
                        "action": "stop",
                        **{
                            k: sent[-1][k]
                            for k in ("device_id", "connection_epoch", "space_epoch")
                        },
                        "status": "completed",
                        "error": None,
                        "result": {"state": "idle", "lease": "private"},
                    }
                ],
                sequence=2,
            )
        )
        receipt = ops.receipts[("device", "stop-1")]
        assert receipt["state"] == "completed" and "lease" not in receipt["result"]
        await ops.close()

    asyncio.run(run())


def test_old_connection_operation_not_replayed_after_reconnect():
    async def run():
        runtime, binding, epoch = ready_runtime()
        put(runtime, binding, epoch)
        connection = SimpleNamespace(
            capture_id="device", connection_id="conn", events=asyncio.Queue()
        )
        manager = SimpleNamespace(
            _connection=connection, presence_expired=lambda c: False
        )
        sent = []
        ops = OperatorCommands(manager, runtime, sent.append)
        ops.bind(True)
        ops.feedback(feedback(runtime))
        await ops.submit(
            connection,
            {"connection_id": "conn", "request_id": "begin-1", "action": "start"},
        )
        runtime.mark_capture_disconnected("device", epoch)
        replacement = SimpleNamespace(
            capture_id="device", connection_id="new-conn", events=asyncio.Queue()
        )
        manager._connection = replacement
        await asyncio.sleep(0.04)
        assert len(sent) == 1
        assert (
            ops.receipts[("device", "begin-1")]["error"]
            == "operator_connection_changed"
        )
        assert replacement.events.empty()
        await ops.close()

    asyncio.run(run())


def load_driver_module(name):
    path = Path(__file__).parents[1] / "pico/4ultra" / f"{name}.py"
    spec = importlib.util.spec_from_file_location("pico_" + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_standalone_metadata_and_info_have_no_core_custom_dependency(tmp_path):
    import yaml

    root = Path(__file__).parents[1]
    metadata = yaml.safe_load((root / "pico/4ultra/driver.yaml").read_text())
    assert metadata["id"] == "pico-driver" and metadata["port"] == 15742
    plugin = ExtVrPlugin(
        {
            "state_dir": str(tmp_path),
            "public_wss_url": "wss://127.0.0.1:15741/ws/teleop-capture",
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
        assert set(tool["configSchema"]["properties"]) == {"usage_guide"}
        assert info["installation_url"] in tool["configSchema"]["description"]
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


def test_transport_write_failure_does_not_kill_request_recovery():
    async def run():
        runtime, binding, epoch = ready_runtime()
        put(runtime, binding, epoch)
        connection = SimpleNamespace(
            capture_id="device", connection_id="conn", events=asyncio.Queue()
        )
        manager = SimpleNamespace(
            _connection=connection, presence_expired=lambda c: False
        )
        calls = []

        def publish(value):
            calls.append(copy.deepcopy(value))
            if len(calls) == 1:
                raise ValueError("operator_queue_full")

        ops = OperatorCommands(manager, runtime, publish)
        ops.bind(True)
        ops.feedback(feedback(runtime))
        receipt = await ops.submit(
            connection,
            {"connection_id": "conn", "request_id": "begin-retry", "action": "start"},
        )
        assert receipt["state"] == "accepted"
        await asyncio.sleep(0.13)
        assert len(calls) >= 2 and calls[0] == calls[1]
        assert not ops._task.done()
        await ops.close()

    asyncio.run(run())


def test_stop_survives_saturated_retired_filter_and_receipt_capacity():
    async def run():
        runtime, binding, epoch = ready_runtime()
        put(runtime, binding, epoch)
        connection = SimpleNamespace(
            capture_id="device", connection_id="conn", events=asyncio.Queue(maxsize=1)
        )
        manager = SimpleNamespace(
            _connection=connection, presence_expired=lambda c: False
        )
        sent = []
        ops = OperatorCommands(manager, runtime, sent.append)
        ops.bind(True)
        ops._seen[:] = b"\xff" * len(ops._seen)
        for index in range(256):
            key = ("device", str(index))
            ops.receipts[key] = {
                "type": "operator_result",
                "request_id": str(index),
                "action": "finish",
                "state": "accepted",
            }
            ops.pending[key] = {"connection": connection}
        request = {"connection_id": "conn", "request_id": "new-stop", "action": "stop"}
        result = await ops.submit(connection, request)
        assert result["state"] == "accepted" and sent[-1]["action"] == "stop"
        assert len(ops.pending) == 1 and len(ops.receipts) == 256
        count = len(sent)
        assert await ops.submit(connection, request) == result
        assert len(sent) == count
        await ops.close()

    asyncio.run(run())


def test_stop_after_rtc_loss_does_not_require_tracking_or_feedback():
    async def run():
        runtime, binding, epoch = ready_runtime()
        put(runtime, binding, epoch)
        connection = SimpleNamespace(
            capture_id="device", connection_id="conn", events=asyncio.Queue()
        )
        manager = SimpleNamespace(
            _connection=connection, presence_expired=lambda c: False
        )
        sent = []
        ops = OperatorCommands(manager, runtime, sent.append)
        ops.bind(True)
        runtime.mark_rtc_disconnected(epoch, "focus_lost")
        assert runtime._binding is None
        result = await ops.submit(
            connection,
            {
                "connection_id": "conn",
                "request_id": "stop-no-tracking",
                "action": "stop",
            },
        )
        assert result["state"] == "accepted" and sent[0]["action"] == "stop"
        validate_operation(
            sent[0],
            instance_id="vr-1",
            clock_id=runtime.clock_id,
            now_ns=runtime.clock_ns(),
        )
        old_status = copy.deepcopy(ops.status)
        receipt = {
            **{
                k: sent[0][k]
                for k in (
                    "request_id",
                    "action",
                    "device_id",
                    "connection_epoch",
                    "space_epoch",
                )
            },
            "status": "completed",
            "error": None,
            "result": {"state": "idle"},
        }
        value = feedback(runtime, receipts=[receipt])
        value["connection_epoch"] = epoch
        # Same request ID cannot be completed by another device or space.
        wrong = copy.deepcopy(value)
        wrong["receipts"][0]["device_id"] = "another-device"
        ops.feedback(wrong, update_status=False)
        assert ops.pending
        wrong["receipts"][0]["device_id"] = "device"
        wrong["receipts"][0]["space_epoch"] += 1
        ops.feedback(wrong, update_status=False)
        assert ops.pending
        ops.feedback(value, update_status=False)
        assert not ops.pending
        assert ops.receipts[("device", "stop-no-tracking")]["state"] == "completed"
        assert ops.status == old_status
        await ops.close()

    asyncio.run(run())


def test_device_plugin_does_not_consume_robot_feedback():
    assert not hasattr(ExtVrPlugin, 'feedback')
    assert not hasattr(ExtVrPlugin, '_accept_feedback')


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


def test_named_controls_and_extensions_survive_rtc_to_dds_without_filtering():
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
