"""Opt-in REAL HTTP → manager → worker → Teleopit/GMR/ONNX/MuJoCo scenarios.

TELEOPIT_TEST_ROOT must point to a bootstrapped, pinned Teleopit installation.
Run this file with that installation's Python, or set TELEOPIT_TEST_PYTHON to
its interpreter. If opted in, missing assets/dependencies/rendering FAIL; they
do not silently skip. The publisher below captures the stock Core transport
boundary only: these tests do not claim a running ROS/Core/browser was tested.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from urllib.request import Request, urlopen


DRIVER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DRIVER))
sys.path.insert(0, str(DRIVER.parents[1]))
spec = importlib.util.spec_from_file_location("teleopit_e2e_entrypoint", DRIVER / "main.py")
entrypoint = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entrypoint)


class CapturingPublisher:
    """Capture actual worker output at the card's ROS message producer seam."""

    def __init__(self):
        self.lock = threading.Lock()
        self.states = []
        self.frames = []

    def publish_state(self, state):
        # JSON serialization is part of the production Core-topic contract.
        with self.lock:
            self.states.append(json.loads(json.dumps(state, allow_nan=False)))
            self.states[:] = self.states[-50:]

    def publish_preview(self, jpeg):
        with self.lock:
            self.frames.append(bytes(jpeg))
            self.frames[:] = self.frames[-50:]

    def last_frame(self):
        with self.lock:
            return self.frames[-1] if self.frames else None


@unittest.skipUnless(os.environ.get("TELEOPIT_TEST_ROOT"),
                     "opt-in real scenario: set TELEOPIT_TEST_ROOT and use the Teleopit dependency interpreter")
class RealTeleopitMCPTests(unittest.TestCase):
    def setUp(self):
        self.publisher = CapturingPublisher()
        self.config = {
            "ros_namespace": "teleopit_e2e",
            "teleopit": {
                "upstream_root": os.environ["TELEOPIT_TEST_ROOT"],
                "worker_python": os.environ.get("TELEOPIT_TEST_PYTHON", sys.executable),
                "source": "bvh", "render": False, "max_steps": 500,
            },
        }
        self.bundle = entrypoint.build_bundle(self.config, publisher=self.publisher)
        self.bundle.start_all()
        self.server = entrypoint.create_server(self.config, self.bundle, host="127.0.0.1", port=0)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True,
                                       name="teleopit-e2e-http")
        self.thread.start()
        self.url = f"http://127.0.0.1:{self.server.server_port}/mcp"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.bundle.stop_all()
        self.bundle.manager.close()

    def rpc(self, method, params=None):
        payload = {"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}
        request = Request(self.url, data=json.dumps(payload).encode(),
                          headers={"Content-Type": "application/json"})
        with urlopen(request, timeout=20) as response:
            result = json.load(response)
        self.assertNotIn("error", result, result)
        return result["result"]

    def call(self, action="info", name="teleopit_sim", **options):
        result = self.rpc("tools/call", {"name": name, "arguments": {"action": action, **options}})
        content = result["content"]
        self.assertEqual(len(content), 1)
        self.assertEqual(content[0]["type"], "text")
        parsed = json.loads(content[0]["text"])
        self.assertIsInstance(parsed, dict)
        return parsed

    def wait_for(self, predicate, description, timeout=60):
        deadline = time.monotonic() + timeout
        last = {}
        while time.monotonic() < deadline:
            last = self.call()
            self.assertNotEqual(last["state"], "error",
                                f"{description}: {json.dumps(last, ensure_ascii=False)}")
            if predicate(last):
                return last
            time.sleep(0.05)
        self.fail(f"Timed out waiting for {description}: {json.dumps(last, ensure_ascii=False)}")

    def assert_real_snapshot(self, snapshot):
        self.assertEqual(snapshot["source"], "bvh")
        self.assertEqual(snapshot["mode"], "simulation")
        self.assertEqual(snapshot["robot_profile"], "unitree_g1_29dof")
        self.assertFalse(snapshot["hardware_output"])
        for key in ("target_positions", "joint_positions", "joint_names"):
            self.assertEqual(len(snapshot[key]), 29, key)
        self.assertEqual(len(set(snapshot["joint_names"])), 29)
        for key in ("target_positions", "joint_positions"):
            self.assertTrue(all(math.isfinite(value) for value in snapshot[key]), key)
        for key in ("retarget_ms", "policy_ms", "physics_ms", "step_compute_ms"):
            self.assertGreater(snapshot[key], 0.0, key)
        self.assertGreater(snapshot["sim_time_s"], 0.0)
        self.assertTrue(any(abs(value) > 1e-4 for value in snapshot["target_positions"]))
        json.dumps(snapshot, allow_nan=False)

    def test_bvh_pause_resume_stop_and_restart_over_real_mcp(self):
        self.assertEqual(self.rpc("initialize")["serverInfo"]["name"], "teleopit-simulation")
        tools = {tool["name"]: tool for tool in self.rpc("tools/list")["tools"]}
        self.assertEqual(tools["teleopit_state"]["topic_out"][0]["format"], "data/json")
        self.call("config", source="bvh", render=False, max_steps=500)
        self.assertTrue(self.call("preflight")["ready"])
        self.assertEqual(self.call("start")["state"], "ready")
        idle = self.call()
        self.assertEqual(idle["state"], "idle")
        self.assertIsNone(idle["session_id"])
        self.assertEqual(idle["snapshot"], {})
        self.assertEqual(self.call("start", name="teleopit_state")["state"], "running")

        began = time.monotonic()
        accepted = self.call("run")
        self.assertTrue(accepted["accepted"])
        self.assertEqual(accepted["state"], "starting")
        active = self.wait_for(lambda info: info["snapshot"].get("step", 0) >= 5,
                               "at least five actual GMR/ONNX/MuJoCo frames")
        startup_s = time.monotonic() - began
        self.assert_real_snapshot(active["snapshot"])
        first_session = active["session_id"]
        self.call("pause")
        paused = self.wait_for(lambda info: info["state"] == "paused", "worker pause acknowledgement", timeout=5)
        paused_step = paused["snapshot"]["step"]
        paused_sim_time = paused["snapshot"]["sim_time_s"]
        time.sleep(0.35)
        still_paused = self.call()
        self.assertEqual(still_paused["state"], "paused")
        self.assertEqual(still_paused["snapshot"]["step"], paused_step)
        self.assertEqual(still_paused["snapshot"]["sim_time_s"], paused_sim_time)

        # Turning a status card off cannot terminate or resume the simulation.
        self.assertEqual(self.call("stop", name="teleopit_state")["state"], "idle")
        self.assertEqual(self.call()["state"], "paused")
        self.call("resume")
        advanced = self.wait_for(lambda info: info["state"] == "running"
                                 and info["snapshot"].get("step", 0) > paused_step,
                                 "worker resume and advancing physics", timeout=5)
        self.assertGreater(advanced["snapshot"]["sim_time_s"], paused_sim_time)
        self.assertEqual(self.call("stop")["state"], "idle")

        # A new short run must allocate a fresh child/session and complete.
        self.call("start", source="bvh", render=False, max_steps=5)
        restarted = self.call("run")
        self.assertTrue(restarted["accepted"])
        self.assertNotEqual(restarted["session_id"], first_session)
        completed = self.wait_for(lambda info: info["state"] == "completed", "five-step restarted run completion")
        self.assertEqual(completed["snapshot"]["step"], 5)
        self.assertEqual(completed["snapshot"]["summary"]["steps"], 5)
        self.assertEqual(completed["snapshot"]["summary"]["reason"], "completed")
        self.assert_real_snapshot(completed["snapshot"])
        print("[e2e] BVH HTTP lifecycle " + json.dumps({
            "startup_to_five_frames_s": round(startup_s, 2),
            "paused_step": paused_step, "resumed_step": advanced["snapshot"]["step"],
            "restart_steps": completed["snapshot"]["step"],
            "target_positions_first3": completed["snapshot"]["target_positions"][:3],
            "step_compute_ms": completed["snapshot"]["step_compute_ms"],
            "hardware_output": completed["hardware_output"],
        }), flush=True)

    def test_rendered_bvh_completion_reaches_card_jpeg_producer(self):
        self.call("config", source="bvh", render=True, max_steps=10)
        self.assertTrue(self.call("preflight")["ready"])
        self.call("start")
        self.assertEqual(self.call("start", name="teleopit_preview")["state"], "running")
        self.assertTrue(self.call("run")["accepted"])
        completed = self.wait_for(lambda info: info["state"] == "completed", "rendered BVH completion")
        self.assertEqual(completed["snapshot"]["summary"]["steps"], 10)
        self.assert_real_snapshot(completed["snapshot"])
        self.wait_for(lambda info: self.publisher.last_frame() is not None,
                      "actual MuJoCo JPEG at the card transport producer", timeout=3)
        jpeg = self.publisher.last_frame()
        self.assertTrue(jpeg.startswith(b"\xff\xd8"), "preview is not a JPEG")
        self.assertTrue(jpeg.endswith(b"\xff\xd9"), "JPEG is truncated")
        self.assertGreater(len(jpeg), 1000)
        self.assertTrue(self.call("info", name="teleopit_preview")["preview_available"])
        print("[e2e] BVH rendered HTTP run " + json.dumps({
            "steps": completed["snapshot"]["step"],
            "jpeg_bytes": len(jpeg), "captured_jpegs": len(self.publisher.frames),
            "hardware_output": completed["hardware_output"],
            "real_ros_core_tested": False,
        }), flush=True)


if __name__ == "__main__":
    unittest.main()
