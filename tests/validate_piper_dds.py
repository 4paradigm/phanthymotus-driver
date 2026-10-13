"""Run in an isolated ROS2 container: real DDS/MCP/IK, simulated CAN only.

No SDK connection, hardware mount or external network is needed. This is not
headset or physical robot acceptance. The simulation fixture never opens CAN.
"""
import copy
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import urllib.request

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from agilex.piper.dual import DualPiperPlugin
from agilex.piper.motion.kinematics import MODEL
from common.teleop_contract import COMMAND_SCHEMA, TRACKING_FRAME, validate_feedback
from common.vendor_runtime import DriverBundle, make_handler
from test_piper_controls import Driver


def run():
    context = Context()
    rclpy.init(context=context, domain_id=42)
    executor = SingleThreadedExecutor(context=context)
    node = Node("piper_test_source", context=context)
    executor.add_node(node)
    input_topic = "/canvas/piper_test/command"
    publisher = node.create_publisher(String, input_topic, 1)
    unrelated = node.create_publisher(String, "/teleop/command", 1)
    received = []
    node.create_subscription(String, "/teleop/state", lambda m: received.append(json.loads(m.data)), 10)
    sequence = 0
    plugin = server = None
    with tempfile.TemporaryDirectory(prefix="piper-dds-test-") as root:
        try:
            document = json.loads((MODEL.parent.parent/"calibration.example.json").read_text())
            document.update(hardware_verified=True, calibration_id="SIMULATED-DDS-TEST-ONLY")
            for arm in document["arms"].values():
                arm["workspace_m"] = [[-1, -1, -1], [1, 1, 1]]
            path = Path(root)/"calibration.json"
            path.write_text(json.dumps(document))
            plugin = DualPiperPlugin({"data_dir": root, "calibration_path": str(path), "async_actions": False,
                                      "arms": {"left": {"can_interface": "can0"}, "right": {"can_interface": "can1"}}},
                                     ros2=SimpleNamespace(ctx_core=context), driver_factory=Driver)
            plugin.start()
            for arm in plugin.arms.values():
                arm.connected().arm.gripper_enabled = True
                arm.motion_enabled = arm.manual_motion_permission = True
            bundle = DriverBundle([plugin])
            server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(lambda: bundle, "piper-test", "piper-test"))
            threading.Thread(target=server.serve_forever, daemon=True).start()

            def rpc(method, params):
                request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}/mcp",
                    data=json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params}).encode(),
                    headers={"Content-Type": "application/json"})
                with urllib.request.urlopen(request, timeout=10) as response:
                    result = json.loads(response.read())
                assert "error" not in result, result
                return result["result"]

            def call(card, action, **args):
                result = rpc("tools/call", {"name": card, "arguments": {"action": action, **args}})
                return json.loads(result["content"][0]["text"])

            assert len(rpc("tools/list", {})["tools"]) == 12
            assert call("piper_teleop", "start")["state"] == "waiting_binding"
            assert plugin.bus is None
            assert publisher.get_subscription_count() == unrelated.get_subscription_count() == 0
            call("piper_teleop", "start", input_topic=input_topic)
            until = time.monotonic()+10
            while publisher.get_subscription_count() == 0 and time.monotonic() < until:
                executor.spin_once(timeout_sec=.05)
            assert publisher.get_subscription_count() > 0, "DDS discovery failed"
            assert unrelated.get_subscription_count() == 0, "unexpected default-topic subscription"
            preview = call("piper_teleop", "begin")
            call("piper_teleop", "begin", execute=True, preview_id=preview["preview_id"], workspace_clear=True, take_control=True)

            def publish(held):
                nonlocal sequence
                sequence += 1
                pose = {"tracked": True, "position": [0., 0., 1.], "orientation_xyzw": [0., 0., 0., 1.]}
                value = {"schema": COMMAND_SCHEMA, "kind": "input", "instance_id": "pico-test",
                    "device_id": "simulated-headset", "clock_id": plugin.motion.clock_id,
                    "sequence": sequence, "connection_epoch": 1, "space_epoch": 1,
                    "source_monotonic_ns": time.monotonic_ns(), "received_monotonic_ns": time.monotonic_ns(),
                    "tracking_frame": TRACKING_FRAME, "head_reference": copy.deepcopy(pose),
                    "left": dict(pose, grip=float(held), trigger=0.), "right": dict(pose, grip=float(held), trigger=0.)}
                msg = String(); msg.data = json.dumps(value)
                publisher.publish(msg)
                executor.spin_once(timeout_sec=.02)
                time.sleep(.03)

            until = time.monotonic()+8
            while plugin.motion.reason != "mapped_waiting_for_both_grips" and time.monotonic() < until:
                publish(False)
            assert plugin.motion.reason == "mapped_waiting_for_both_grips", plugin.motion.status()
            call("piper_motion", "record_start")
            until = time.monotonic()+2
            while time.monotonic() < until:
                publish(True)
            assert plugin.motion.output, plugin.motion.status()
            recording = call("piper_motion", "record_stop")
            assert recording["complete"] and recording["frames"] >= 3, recording
            call("piper_teleop", "finish")
            plugin.motion.thread.join(4)
            assert not plugin.motion.thread.is_alive()
            assert received, "no DDS monitoring received"
            validate_feedback(received[-1], instance_id=received[-1]["instance_id"],
                              clock_id=plugin.motion.clock_id, now_ns=time.monotonic_ns())
            preview = call("piper_motion", "play", filename=recording["filename"])
            call("piper_motion", "play", filename=recording["filename"], execute=True,
                 preview_id=preview["preview_id"], workspace_clear=True, take_control=True)
            plugin.motion.thread.join(10)
            assert not plugin.motion.thread.is_alive()
            assert plugin.motion.last_action["status"] == "completed", plugin.motion.last_action
            print(json.dumps({"evidence": "real_loopback_DDS_MCP_NumPy_worker_simulated_CAN",
                              "frames": recording["frames"], "monitor_messages": len(received),
                              "replay": plugin.motion.last_action["status"], "canvas_input_topic": input_topic,
                              "default_topic_subscribers": unrelated.get_subscription_count(), "physical_acceptance": False}))
        finally:
            if plugin is not None:
                plugin.stop()
            if server is not None:
                server.shutdown(); server.server_close()
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown(context=context)


if __name__ == "__main__":
    run()
