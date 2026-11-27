"""Host-side relay preflight tests; no ROS installation or robot required."""

from __future__ import annotations

import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from agibot.A3 import jazzy_relay


class JazzyRelayPreflightTests(unittest.TestCase):
    def test_localization_relay_preserves_vendor_frame_and_timestamp(self):
        msg = SimpleNamespace(
            header=SimpleNamespace(frame_id="map", stamp=SimpleNamespace(sec=17, nanosec=4)),
            pose=SimpleNamespace(pose=SimpleNamespace(
                position=SimpleNamespace(x=2.0, y=3.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))))
        qos = object()
        calls = {}

        def subscribe(msg_type, topic, callback, profile):
            calls.update(topic=topic, profile=profile)
            callback(msg)

        node = SimpleNamespace(create_subscription=subscribe, destroy_node=lambda: None)
        rclpy = SimpleNamespace(init=lambda: None, create_node=lambda _: node,
                                spin=lambda _: None, try_shutdown=lambda: None)
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ), patch.dict(sys.modules, {
                "rclpy": rclpy, "rclpy.qos": SimpleNamespace(qos_profile_sensor_data=qos),
                "nav_msgs.msg": SimpleNamespace(Odometry=object)}):
            path = Path(folder) / "localization.json"
            jazzy_relay._relay_localization("/vendor/dds.xml", "host:11811", path)
            self.assertEqual(calls, {"topic": "/slam/localization/odometry", "profile": qos})
            self.assertEqual(json.loads(path.read_text())[1:], ["map", 17, 4, 2, 3, 0, 0, 0, 1])
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "232")
            self.assertEqual(os.environ["ROS_DISCOVERY_SERVER"], "host:11811")

    def test_image_dependencies_are_imported_and_reported(self):
        calls = []
        cv2 = SimpleNamespace(
            __version__="4.test", setNumThreads=lambda count: calls.append(count))
        numpy = SimpleNamespace(__version__="2.test")
        output = io.StringIO()
        with patch.dict(sys.modules, {"cv2": cv2, "numpy": numpy}), \
                contextlib.redirect_stdout(output):
            jazzy_relay._check_image_dependencies()
        self.assertEqual(calls, [1])
        self.assertIn("numpy=2.test opencv=4.test", output.getvalue())

    def test_missing_host_image_dependency_fails_with_actionable_message(self):
        real_import = __import__

        def without_image_packages(name, *args, **kwargs):
            if name in {"cv2", "numpy"}:
                raise ImportError(f"No module named {name}")
            return real_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=without_image_packages):
            with self.assertRaisesRegex(RuntimeError, "container's pip packages"):
                jazzy_relay._check_image_dependencies()


if __name__ == "__main__":
    unittest.main()
