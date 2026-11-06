import ast
import __future__
import copy
import json
import os
from pathlib import Path
import tempfile
import threading
import time
import types
import sys
import unittest
from unittest.mock import patch

BUNDLE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BUNDLE_ROOT))
import foot_pose as card


def sample(mode=2, position=0, moving=False, fault=False):
    return {"fresh": True, "workmode": {"code": mode, "protection": mode == 26},
            "activity": "moving" if moving else "stationary", "motor_faults": [1] if fault else [],
            "joint_states": [{"motor_id": i, "position": position, "velocity": 0.0} for i in card.LEG_IDS]}


def fixture_manifest():
    # These fabricated IDs and zero positions exist only in tests, never in a deployment package.
    return {"schema": "bumi.foot-pose-assets/1", "robot_model": "Bumi-EDU",
            "validated_by": "TEST FIXTURE ONLY", "verification_record": "simulated-test",
            "poses": {name: {"recording_id": i, "verified_on_device": True, "safe_stop_verified": True,
                              "joint_signature_rad": {str(k): 0.0 for k in card.LEG_IDS},
                              "tolerance_rad": 0.02, "min_elapsed_s": 0.2, "timeout_s": 1.2}
                      for i, name in enumerate(card.POSES)}}


class High:
    def __init__(self):
        self.mode, self.callback = 2, None
    def get_mode(self):
        return self.mode
    def subscribe_robot_hardware_status(self, callback):
        self.callback = callback
        callback(None)


class Loco:
    def __init__(self, high):
        self.high = high
        self._rotate_lock = threading.Lock()
        self._body_action = None
        self.calls = []
    def is_moving(self):
        return bool(self._body_action)
    def _preempt_servo(self, reason):
        self.calls.append(("pause_stream", reason))
    def _trigger_user_action(self, *args, **kwargs):
        self.calls.append(("play", kwargs["recording_id"]))
        self.high.mode = 23
        return {"command_sent": True, "confirmed_started": True}
    def _do_stop_playback(self, safety):
        self.calls.append(("stop",))
        self.high.mode = 2
        return {"command_sent": True, "workmode": 2}


class CardTests(unittest.TestCase):
    def make(self, directory, *, assets=True, malformed=False, moving=False):
        path = Path(directory) / "assets.json"
        if assets:
            path.write_text("[]" if malformed else json.dumps(fixture_manifest()), encoding="utf-8")
        high = High()
        loco = Loco(high)
        results = []
        def reader():
            if high.callback:
                high.callback(None)
            return sample(high.mode, moving=moving)
        plugin = card.FootPosePlugin({"assets_file": str(path)}, "test", None, high, loco,
                                     snapshot_reader=reader, notifier=lambda *args: results.append(args))
        plugin.start()
        return plugin, high, loco, results

    def test_all_four_defaults_are_explicit_previews_and_never_send_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, high, loco, _ = self.make(directory, assets=False)
            for pose in card.POSES:
                result = plugin.dispatch(pose, {})
                self.assertEqual(result["state"], "preview")
                self.assertFalse(result["command_sent"])
                self.assertFalse(result["pose_confirmed"])
            self.assertEqual(loco.calls, [])
            self.assertFalse(plugin.info()["physical_execution_ready"])
            self.assertEqual(plugin.dispatch("config", {"dry_run": False, "confirm": True})["state"], "error")

    def test_malformed_manifest_does_not_take_down_the_device_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, _, loco, _ = self.make(directory, malformed=True)
            self.assertEqual(plugin.info()["state"], "preview_only")
            self.assertEqual(loco.calls, [])

    def test_manifest_requires_all_legs_distinct_assets_and_stop_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "assets.json"
            modifications = [lambda d: d["poses"].pop("neutral"),
                             lambda d: d["poses"]["feet_together"].update(safe_stop_verified=False),
                             lambda d: d["poses"]["feet_together"]["joint_signature_rad"].pop("4"),
                             lambda d: d["poses"]["feet_together"].update(recording_id=True),
                             lambda d: d["poses"]["feet_apart"].update(recording_id=0)]
            for mutate in modifications:
                data = fixture_manifest()
                mutate(data)
                path.write_text(json.dumps(data), encoding="utf-8")
                with self.assertRaises(ValueError):
                    card.load_assets(path)

    def test_real_mode_needs_confirm_and_fresh_stationary_telemetry_and_is_atomic(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, high, loco, _ = self.make(directory)
            for args in ({"dry_run": False}, {"dry_run": "false"}, {"dry_run": False, "confirm": False}):
                self.assertEqual(plugin.dispatch("config", args)["state"], "error")
                self.assertTrue(plugin.info()["dry_run"])
            plugin._received = time.monotonic() - 2
            self.assertEqual(plugin.dispatch("config", {"dry_run": False, "confirm": True})["state"], "error")
            high.callback(None)
            result = plugin.dispatch("config", {"dry_run": False, "confirm": True})
            self.assertTrue(result["physical_execution_ready"])
            self.assertEqual(loco.calls, [])

    def test_play_is_sent_once_and_completes_only_after_signature_holds(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, high, loco, results = self.make(directory)
            plugin.configure({"dry_run": False, "confirm": True})
            accepted = plugin.execute("feet_apart")
            self.assertEqual(accepted["state"], "accepted")
            self.assertFalse(accepted["pose_confirmed"])
            plugin._worker.join(2)
            self.assertEqual([x for x in loco.calls if x[0] == "play"], [("play", 1)])
            self.assertEqual(plugin.info()["task"]["state"], "completed")
            self.assertTrue(plugin.info()["task"]["pose_confirmed"])
            self.assertEqual(len(results), 1)
            self.assertEqual(results[0][1], "completed")
            self.assertTrue(plugin._owned)
            stopped = plugin.stop()
            self.assertTrue(stopped["stop_result"]["returned_to_walking"])
            self.assertFalse(plugin._owned)
            self.assertIsNone(loco._body_action)

    def test_other_body_owner_is_not_preempted_or_stopped(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, _, loco, _ = self.make(directory)
            plugin.configure({"dry_run": False, "confirm": True})
            loco._body_action = "another_action"
            result = plugin.dispatch("feet_together", {})
            self.assertEqual(result["state"], "error")
            plugin.stop()
            self.assertEqual(loco._body_action, "another_action")
            self.assertEqual(loco.calls, [])

    def test_stop_cancels_only_owned_playback_without_replaying(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, _, loco, results = self.make(directory)
            plugin.configure({"dry_run": False, "confirm": True})
            plugin.execute("feet_staggered")
            time.sleep(0.05)
            plugin.stop()
            self.assertEqual(len([x for x in loco.calls if x[0] == "play"]), 1)
            self.assertEqual(results[0][1], "cancelled")
            self.assertFalse(plugin.info()["task"]["pose_confirmed"])
            self.assertIsNone(loco._body_action)

    def test_joint_match_never_accepts_missing_nan_moving_or_wrong_positions(self):
        profile = {"joint_signature_rad": {4: 0.2}, "tolerance_rad": 0.01}
        for payload in ({}, {"joint_states": []}, sample(position=float("nan")),
                        sample(position=0.2, moving=True), sample(position=0.22)):
            self.assertFalse(card.signature_matches(payload, profile))
        self.assertTrue(card.signature_matches(sample(position=0.2), profile))

    def test_no_auto_arming_from_yaml_and_protection_blocks_dispatch(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, high, loco, _ = self.make(directory)
            plugin._dry_run = False
            self.assertEqual(plugin.dispatch("feet_together", {})["state"], "error")
            high.mode = 26
            self.assertEqual(plugin.dispatch("config", {"dry_run": False, "confirm": True})["state"], "error")
            self.assertEqual(loco.calls, [])

    def test_timeout_does_not_report_success_or_retry_the_recording(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, high, loco, results = self.make(directory)
            plugin.configure({"dry_run": False, "confirm": True})
            for profile in plugin._profiles.values():
                profile["joint_signature_rad"] = {k: 0.4 for k in card.LEG_IDS}
            plugin.execute("feet_together")
            plugin._worker.join(2)
            state = plugin.info()["task"]
            self.assertEqual(state["state"], "failed")
            self.assertFalse(state["pose_confirmed"])
            self.assertEqual(len([x for x in loco.calls if x[0] == "play"]), 1)
            self.assertEqual(results[0][1], "failed")
            self.assertEqual(high.mode, 2)
            self.assertIsNone(loco._body_action)

    def test_boolean_configuration_flags_are_not_silently_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, _, loco, _ = self.make(directory)
            self.assertEqual(plugin.dispatch("config", {"joint_angles": [0]})["state"], "error")
            self.assertTrue(plugin.info()["dry_run"])
            self.assertEqual(loco.calls, [])

    def test_unconfirmed_stop_keeps_body_reserved_until_walking_is_observed(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, high, loco, _ = self.make(directory)
            plugin._claim()
            high.mode = 23
            def failed_stop(_):
                raise TimeoutError("SDK stop result unknown")
            loco._do_stop_playback = failed_stop
            result = plugin.stop()
            self.assertEqual(result["state"], "stop_unconfirmed")
            self.assertFalse(result["stopped"])
            self.assertTrue(plugin._owned)
            self.assertEqual(loco._body_action, plugin._lease)
            high.mode = 2
            self.assertTrue(plugin.stop()["stopped"])
            self.assertIsNone(loco._body_action)

    def test_cancellation_with_failed_physical_stop_also_keeps_the_lease(self):
        with tempfile.TemporaryDirectory() as directory:
            plugin, high, loco, _ = self.make(directory)
            plugin.configure({"dry_run": False, "confirm": True})
            def failed_stop(_):
                raise TimeoutError("Stop unconfirmed")
            loco._do_stop_playback = failed_stop
            plugin.execute("feet_staggered")
            time.sleep(0.05)
            self.assertFalse(plugin.stop()["stopped"])
            self.assertTrue(plugin._owned)
            self.assertEqual(loco._body_action, plugin._lease)
            high.mode = 2
            self.assertTrue(plugin.stop()["stopped"])

    def test_unknown_leg_velocity_cannot_confirm_a_pose(self):
        state = sample(position=0.2)
        profile = {"joint_signature_rad": {4: 0.2}, "tolerance_rad": 0.01}
        state["joint_states"][0]["velocity"] = float('nan')
        self.assertFalse(card.signature_matches(state, profile))


class RegistrationTests(unittest.TestCase):
    def test_real_bundle_constructor_registers_and_dispatches_the_new_card(self):
        source = (BUNDLE_ROOT / "main.py").read_text(encoding="utf-8")
        node = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name == "BumiDeviceBundle")
        device = types.ModuleType("device")
        class FakeLoco(Loco):
            def __init__(self, config, namespace, executor, high):
                super().__init__(high)
            def get_tools(self): return [{"name": "loco", "type": "actuator"}]
        device.LocoPlugin = FakeLoco
        device._acp_notify = lambda *args: None
        namespace = {"os": os}
        with patch.dict(sys.modules, {"device": device, "foot_pose": card}), patch.dict(os.environ):
            exec(compile(ast.Module(body=[node], type_ignores=[]), "bundle-test", "exec", flags=__future__.annotations.compiler_flag), namespace)
            bundle = namespace["BumiDeviceBundle"]({"plugins": {"loco": {"enabled": True}, "foot_pose": {"enabled": True}}}, "test", None, High(), None)
            self.assertIn("foot_pose", [t["name"] for t in bundle.get_all_tools()])
            self.assertFalse(bundle.dispatch("foot_pose", {"action": "feet_together"})["command_sent"])
            self.assertIsNone(bundle.dispatch("unregistered", {}))

    def test_motion_state_reader_is_used_by_the_registered_card(self):
        source = (BUNDLE_ROOT / "main.py").read_text(encoding="utf-8")
        node = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name == "BumiDeviceBundle")
        device = types.ModuleType("device")
        class FakeLoco(Loco):
            def __init__(self, config, namespace, executor, high):
                super().__init__(high)
            def get_tools(self): return [{"name": "loco", "type": "actuator"}]
        class FakeMotion:
            def __init__(self, config, namespace, executor, high):
                self._node = types.SimpleNamespace(_read_once=lambda: sample(high.mode))
            def get_tool(self): return {"name": "motion_state", "type": "sensor"}
        device.LocoPlugin = FakeLoco
        device.MotionStatePlugin = FakeMotion
        device._acp_notify = lambda *args: None
        namespace = {"os": os}
        with tempfile.TemporaryDirectory() as directory, patch.dict(sys.modules, {"device": device, "foot_pose": card}), patch.dict(os.environ):
            assets = Path(directory) / "assets.json"
            assets.write_text(json.dumps(fixture_manifest()), encoding="utf-8")
            exec(compile(ast.Module(body=[node], type_ignores=[]), "bundle-test", "exec", flags=__future__.annotations.compiler_flag), namespace)
            config = {"plugins": {"loco": {"enabled": True}, "motion_state": {"enabled": True},
                                   "foot_pose": {"enabled": True, "assets_file": str(assets)}}}
            bundle = namespace["BumiDeviceBundle"](config, "test", None, High(), None)
            result = bundle.dispatch("foot_pose", {"action": "start"})
            self.assertFalse(result["physical_execution_ready"])
            configured = bundle.dispatch("foot_pose", {"action": "config", "dry_run": False, "confirm": True})
            self.assertTrue(configured["physical_execution_ready"])
            self.assertFalse(bundle._plugins[0].calls)


if __name__ == "__main__":
    unittest.main(verbosity=2)
