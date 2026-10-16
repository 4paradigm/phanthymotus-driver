"""Run in the isolated ROS container: actual DDS and TLS WebSocket, no hardware."""
import asyncio
import json
import socket
import ssl
import threading
import time
import sys
from pathlib import Path
from types import SimpleNamespace

import aiohttp
import pytest

from test_pico_device import frame, load_driver_module
from test_pico_feedback import display_feedback
from ext_vr.capture import CAPTURE_PROTOCOL, RTC_FRAME_PROTOCOL
from ext_vr.plugin import ExtVrPlugin
from ext_vr.runtime import DeviceRuntime
from ext_vr.feedback import RobotFeedback
from ext_vr.transport import RosTransport
from common.teleop_contract import COMMAND_TOPIC, STATE_TOPIC


def test_dds_feedback_reaches_headset_socket_and_stop_invalidates_it(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String

    context = Context()
    rclpy.init(context=context, domain_id=42)
    executor = SingleThreadedExecutor(context=context)
    publisher_node = rclpy.create_node("feedback_fixture", context=context)
    publisher = publisher_node.create_publisher(String, "/teleop/state", 16)
    executor.add_node(publisher_node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    config = load_driver_module("identity").prepare_config({
        "public_host": "127.0.0.1", "state_dir": str(tmp_path / "state"),
        "port": port, "bind_host": "127.0.0.1", "discovery_enabled": False,
    })
    plugin = ExtVrPlugin(config, "pico", ros2=executor)
    try:
        plugin.dispatch("start", {"instance_id": "test-vr"})
        item = plugin.instances["test-vr"]
        invitation = asyncio.run_coroutine_threadsafe(item["manager"].create_pairing(), plugin.loop).result(3)

        async def receive():
            tls = ssl.create_default_context(cafile=config["tls_cert_file"])
            async with aiohttp.ClientSession() as client:
                async with client.ws_connect(f"https://127.0.0.1:{port}/ws/teleop-capture", ssl=tls) as ws:
                    await ws.send_json({"type": "pair", "pairing_id": invitation["pairing_id"],
                        "pairing_code": invitation["pairing_code"], "capture_protocol": CAPTURE_PROTOCOL,
                        "frame_protocol": RTC_FRAME_PROTOCOL, "client_kind": "native_openxr",
                        "app_version": "0.4.5-pico-feedback"})
                    ack = await ws.receive_json(timeout=3)
                    assert ack["type"] == "paired"
                    runtime = item["runtime"]
                    binding, epoch = runtime.bind_capture(ack["capture_id"])
                    ready = None
                    for sequence in range(1, 151):
                        runtime.submit_rtc_frame(frame(sequence), authority=binding, rtc_generation=epoch)
                        value = display_feedback(runtime, sequence)
                        value["source_sequence"] = sequence
                        publisher.publish(String(data=json.dumps(value)))
                        message = await ws.receive_json(timeout=1)
                        if message["type"] == "visualization" and message["visualization"]["alignment"]["ready"]:
                            ready = message["visualization"]
                            break
                    assert ready is not None, "real DDS feedback did not reach WSS"
                    assert ready["visualization"]["measured"] == value["visualization"]["measured"]
                    assert not runtime.actuation_enabled
                    plugin.dispatch("stop", {})
                    for _ in range(20):
                        message = await ws.receive_json(timeout=1)
                        if message["type"] == "visualization" and not message["visualization"]["alignment"]["ready"]:
                            (tmp_path / "feedback.jsonl").write_text(json.dumps(ready) + "\n" + json.dumps(message["visualization"]) + "\n")
                            return
                    pytest.fail("stopped preview remained ready")

        asyncio.run(receive())
    finally:
        plugin.close()
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        publisher_node.destroy_node()
        rclpy.shutdown(context=context)


@pytest.mark.skipif(
    not (Path(__file__).resolve().parents[1] / "unitree/g1/teleop_control.py").is_file(),
    reason="Requires the companion G1 source from PR #330 in the integration checkout",
)
def test_pico_input_and_g1_feedback_use_the_same_contract_over_dds():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "unitree/g1"))
    from teleop_control import TeleopControl
    from teleop_bus import TeleopBus

    runtime = DeviceRuntime("integration-pico")
    runtime.start()
    binding, epoch = runtime.bind_capture("paired-headset")
    preview = RobotFeedback(runtime)
    messages, hardware_commands = [], []
    arm = SimpleNamespace(cfg={}, set_joint_publisher=lambda callback: None,
                          accept_control=hardware_commands.append)
    state = dict(state="idle", reason=None, output_active=False, ownership_held=False,
                 hold_confirmed=False, stop_confirmed=True, applied_sequence=-1, feedback={})
    motion = SimpleNamespace(executor=arm, clock_id=runtime.clock_id,
                             gate=SimpleNamespace(status=lambda: state), _decision={}, _finish={})
    control = TeleopControl({"mode": "shadow"}, motion)
    control.binding = {"command_topic": COMMAND_TOPIC, "feedback_topic": STATE_TOPIC}
    context = Context()
    rclpy.init(context=context, domain_id=42)
    executor = SingleThreadedExecutor(context=context)
    bus = TeleopBus(executor, control)

    def receive(value):
        messages.append(value)
        preview.accept(value)

    transport = RosTransport(executor, "pico", runtime.instance_id, receive)

    def wait_for(condition):
        deadline = time.monotonic() + 5
        while not condition() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert condition(), bus.status()

    try:
        wait_for(lambda: transport.pub.get_subscription_count() > 0
                 and transport.node.count_publishers(STATE_TOPIC) > 0)
        deadline = time.monotonic() + .15
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert messages == [], "G1 must not publish an unbound instance_id"

        value = runtime.submit_rtc_frame(frame(1, grip=0.), authority=binding, rtc_generation=epoch)
        transport.publish(runtime.take_latest())
        wait_for(lambda: preview.value is not None)
        assert bus.status()["last_accepted_sequence"] == value["sequence"]
        assert preview.snapshot()["instance_id"] == runtime.instance_id
        assert preview.snapshot()["source_sequence"] == value["sequence"]

        transport.publish(dict(value, sequence=2, received_monotonic_ns=runtime.clock_ns()-301_000_000))
        wait_for(lambda: bus.status()["receive_error"] == "input_stale")
        assert bus.status()["last_accepted_sequence"] == value["sequence"]

        other = DeviceRuntime("replacement-pico", clock_id=runtime.clock_id)
        other.start()
        other_binding, other_epoch = other.bind_capture("replacement-headset")
        other.submit_rtc_frame(frame(1, grip=0.), authority=other_binding, rtc_generation=other_epoch)
        transport.publish(other.take_latest())
        wait_for(lambda: control._latest["instance_id"] == other.instance_id
                 and preview.value is None)
        assert control._needs_calibration and control._input_changed
        replacement = RobotFeedback(other)
        replacement.accept(control.feedback())
        assert replacement.snapshot()["instance_id"] == other.instance_id
        assert preview.reason == "feedback_invalid"
        assert hardware_commands == [] and control._operator is None
    finally:
        transport.close()
        bus.close()
        executor.shutdown()
        rclpy.shutdown(context=context)
