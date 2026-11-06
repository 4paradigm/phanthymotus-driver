"""ARM64 image validation with actual bundled modules and actual ROS resources.

Run via run_container_validation.sh in a device-free, network-none container.
Import main without invoking main(): only ImageQualityGuard is configured.
--serve is a bounded SIGTERM/normal-exit resource-cleanup probe, not a driver.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import platform
import signal
import sys
import threading
import unittest
from uuid import uuid4

import cv2
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import image_quality
import image_quality_guard
import image_quality_runtime
from main import BumiDeviceBundle


def configured_bundle(executor):
    # No other plugin is enabled; no camera, SDK initialization or controller.
    return BumiDeviceBundle({"plugins": {"image_quality_guard": {"enabled": True}}},
                            "iqg_container", executor, None, None)


def start_guard(bundle):
    status = bundle.lifecycle_dispatch("motus.lifecycle.status", {"tool": "image_quality_guard"})
    epoch = str(uuid4())
    bound = bundle.lifecycle_dispatch("motus.lifecycle.bind", {
        "tool": "image_quality_guard", "driver_boot_id": status["driver_boot_id"],
        "expected_epoch": None, "proposed_epoch": epoch})
    assert bound["admission_epoch"] == epoch, bound
    # Only this isolated fixture synthesizes a ticket; production Core issues it.
    run_id = epoch + "." + str(uuid4())
    result = bundle.dispatch("image_quality_guard", {"action": "start", "run_id": run_id,
                             "input_topic": "/iqg_container/camera/color"})
    assert result["state"] == "running", result
    return run_id


class ImageContainerTests(unittest.TestCase):
    def setUp(self):
        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=2, context=self.context)
        self.bundle = configured_bundle(self.executor)
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.bundle.stop_all()
        self.assertEqual(self.executor.get_nodes(), [])
        self.assertTrue(self.executor.shutdown(timeout_sec=5))
        self.context.shutdown()

    def test_image_modules_and_real_ros_message_types_load(self):
        self.assertEqual(platform.machine(), "aarch64")
        for module in (image_quality, image_quality_guard, image_quality_runtime):
            self.assertEqual(Path(module.__file__).resolve().parent, Path("/work"))
        self.assertEqual(CompressedImage().format, "")
        self.assertEqual(String(data="loaded").data, "loaded")
        self.assertNotIn("highcontrol_py", sys.modules)
        self.assertNotIn("mediacontrol_py", sys.modules)
        self.assertNotIn("pyrealsense2", sys.modules)
        self.assertTrue(cv2.__version__)
        self.assertTrue(np.__version__)
        self.assertNotIn("", os.environ["LD_LIBRARY_PATH"].split(":"))

    def test_actual_bundle_boot_registers_guard_without_a_ros_node(self):
        self.bundle.start_all()
        tools = self.bundle.get_all_tools()
        self.assertEqual([tool["name"] for tool in tools], ["image_quality_guard"])
        self.assertEqual(tools[0]["type"], "processor")
        self.assertEqual(tools[0]["x-lifecycle"]["id_parameter"], "run_id")
        self.assertEqual(self.executor.get_nodes(), [])

    def test_real_start_stop_and_duplicate_stop_release_only_owned_node(self):
        run_id = start_guard(self.bundle)
        guard = self.bundle._plugins[0]
        runtime = guard._runtime
        self.assertIsInstance(runtime, image_quality_runtime.ImageQualityRosRuntime)
        self.assertEqual(len(self.executor.get_nodes()), 1)
        stopped = self.bundle.dispatch("image_quality_guard", {"action": "stop", "run_id": run_id})
        self.assertEqual(stopped["state"], "idle")
        self.assertEqual(self.executor.get_nodes(), [])
        self.assertTrue(self.context.ok())
        self.assertIsNone(runtime._node)
        self.assertIsNone(runtime._subscription)
        self.assertIsNone(runtime._publisher)
        self.assertIsNone(runtime._timer)
        repeated = self.bundle.dispatch("image_quality_guard", {"action": "stop", "run_id": run_id})
        self.assertEqual(repeated["code"], "ALREADY_STOPPED")

    def test_bundle_shutdown_is_idempotent_and_closes_admission(self):
        start_guard(self.bundle)
        self.bundle.stop_all()
        self.bundle.stop_all()
        self.assertEqual(self.executor.get_nodes(), [])
        info = self.bundle.dispatch("image_quality_guard", {"action": "info"})
        self.assertTrue(info["shutdown"])
        self.assertFalse(info["resources_pending"])
        self.assertTrue(self.context.ok())


def serve_probe():
    event = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: event.set())
    signal.signal(signal.SIGINT, lambda *_: event.set())
    context = Context()
    rclpy.init(context=context)
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    bundle = configured_bundle(executor)
    worker = None
    try:
        start_guard(bundle)
        worker = threading.Thread(target=executor.spin, name="iqg-validation-executor")
        worker.start()
        print(json.dumps({"probe": "READY", "nodes": len(executor.get_nodes())}), flush=True)
        event.wait(90)
    finally:
        bundle.stop_all()
        bundle.stop_all()
        node_count = len(executor.get_nodes())
        resources = bundle._plugins[0].info()["resources_pending"]
        stopped = executor.shutdown(timeout_sec=10)
        if worker is not None:
            worker.join(10)
        thread_alive = worker is not None and worker.is_alive()
        context.shutdown()
        result = {"probe": "CLEANUP", "nodes": node_count,
                  "resources_pending": resources, "executor_stopped": stopped,
                  "worker_alive": thread_alive}
        print(json.dumps(result), flush=True)
        assert node_count == 0 and not resources and stopped and not thread_alive, result


if __name__ == "__main__":
    if "--serve" in sys.argv:
        serve_probe()
    else:
        metadata = {"platform": platform.machine(), "opencv": cv2.__version__,
                    "numpy": np.__version__, "ros_distro": os.environ.get("ROS_DISTRO"),
                    "ld_library_path": os.environ.get("LD_LIBRARY_PATH"),
                    "artifact_sha256": {Path(module.__file__).name:
                        hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()
                        for module in (image_quality, image_quality_guard, image_quality_runtime)}}
        print(json.dumps(metadata), flush=True)
        unittest.main()
