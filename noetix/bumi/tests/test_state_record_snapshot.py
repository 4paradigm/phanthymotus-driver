"""Regression tests for StateRecordPlugin's wrapper-to-node snapshot path."""
from __future__ import annotations

import ast
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
from types import SimpleNamespace
import tempfile
import threading
import unittest
import uuid


def load_plugin_classes():
    path = Path(__file__).resolve().parents[1] / "device.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    selected = [
        node for node in tree.body
        if isinstance(node, ast.ImportFrom) and node.module == "__future__"
        or isinstance(node, ast.FunctionDef) and node.name == "_finite_number"
        or isinstance(node, ast.ClassDef) and node.name in {
            "StatePlugin", "MotionStatePlugin", "StateRecordPlugin",
        }
    ]
    namespace = {
        "Any": object,
        "Path": Path,
        "datetime": datetime,
        "timezone": timezone,
        "json": json,
        "math": math,
        "os": os,
        "re": re,
        "threading": threading,
        "uuid": uuid,
    }
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), "exec"), namespace)
    return (
        namespace["StatePlugin"],
        namespace["MotionStatePlugin"],
        namespace["StateRecordPlugin"],
    )


class SnapshotForwardingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.StatePlugin, cls.MotionStatePlugin, cls.StateRecordPlugin = load_plugin_classes()

    def test_state_plugin_forwards_source_and_max_age(self):
        calls = []
        plugin = self.StatePlugin.__new__(self.StatePlugin)
        plugin._node = SimpleNamespace(
            snapshot=lambda source, max_age: calls.append((source, max_age)) or {
                "payload": {"soc": 90}, "fresh": True,
            }
        )

        result = plugin.snapshot("battery", 3.0)

        self.assertEqual(calls, [("battery", 3.0)])
        self.assertEqual(result["payload"]["soc"], 90)

    def test_motion_plugin_forwards_max_age(self):
        calls = []
        plugin = self.MotionStatePlugin.__new__(self.MotionStatePlugin)
        plugin._node = SimpleNamespace(
            snapshot=lambda max_age: calls.append(max_age) or {
                "payload": {"activity": "stationary"}, "fresh": True,
            }
        )

        result = plugin.snapshot(3.0)

        self.assertEqual(calls, [3.0])
        self.assertEqual(result["payload"]["activity"], "stationary")

    def test_every_telemetry_record_action_reaches_the_node_and_saves(self):
        state = self.StatePlugin.__new__(self.StatePlugin)
        state._node = SimpleNamespace(snapshot=lambda source, max_age: {
            "payload": {"source": source}, "fresh": True,
        })
        motion = self.MotionStatePlugin.__new__(self.MotionStatePlugin)
        motion._node = SimpleNamespace(snapshot=lambda max_age: {
            "payload": {"source": "motion_state"}, "fresh": True,
        })

        with tempfile.TemporaryDirectory() as folder:
            recorder = self.StateRecordPlugin(
                {"storage_dir": folder, "max_age_s": 3.0}, state, motion,
            )
            for action in (
                "record_imu", "record_battery", "record_joints",
                "record_motion_state", "record_all",
            ):
                with self.subTest(action=action):
                    result = recorder.dispatch(action, {"label": "regression"})
                    self.assertEqual(result["state"], "completed")
                    self.assertTrue(result["saved"])
                    self.assertTrue(Path(result["file"]).is_file())


if __name__ == "__main__":
    unittest.main()
