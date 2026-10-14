"""Run in the isolated ROS container: actual DDS and TLS WebSocket, no hardware."""
import asyncio
import json
import socket
import ssl
import threading

import aiohttp
import pytest

from test_pico_device import frame, load_driver_module
from test_pico_feedback import display_feedback
from ext_vr.capture import CAPTURE_PROTOCOL, RTC_FRAME_PROTOCOL
from ext_vr.plugin import ExtVrPlugin


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
