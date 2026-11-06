"""Offline direction-card contracts. No ROS discovery or hardware operations."""
import concurrent.futures
import tempfile
import json
import sys
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).parent))
import test_device  # existing ROS-free runtime stubs
import device
import direction


def neck(actual=0, desired=0):
    return types.SimpleNamespace(joint_names=["neck_base_joint"],
                                 actual=types.SimpleNamespace(positions=[actual]),
                                 desired=types.SimpleNamespace(positions=[desired]))


class DirectionTests(unittest.TestCase):
    def setUp(self):
        patch = mock.patch.object(device, "_acp_notify")
        self.notify = patch.start()
        self.addCleanup(patch.stop)
        self.completion = threading.Event()
        self.notify.side_effect = lambda *args: self.completion.set()

    def nodes(self):
        nodes = mock.Mock()
        nodes.namespace = "test"
        nodes.String = type("String", (), {})
        nodes._sensor_qos = object()
        nodes.get_system_enabled.return_value = {"code": "OK", "data": {"enabled": False}}
        return nodes

    def sound(self):
        nodes = self.nodes()
        plugin = direction.SoundDirectionPlugin(nodes)
        plugin.start()
        return plugin, nodes

    def head(self):
        nodes = self.nodes()
        audio = device.AudioPlugin(nodes)
        plugin = direction.HeadOrientationPlugin(nodes, audio)
        # This fixture exercises the explicitly opted-in experimental path.
        # Production defaults are covered separately below.
        plugin.experimental_turn_enabled = True
        plugin.running = True
        plugin._client = mock.Mock()
        plugin._request_type = lambda: types.SimpleNamespace(
            revolute_setup=types.SimpleNamespace(), prismatic_setup=types.SimpleNamespace())
        plugin._on_feedback(neck())
        return plugin, audio, nodes

    def submit(self, plugin, **args):
        # A synchronous thread double makes worker/future tests deterministic.
        with mock.patch.object(direction.threading, "Thread") as thread:
            thread.side_effect = lambda target, args, **kw: types.SimpleNamespace(start=lambda: target(*args))
            plugin.turn(dict(degree=5, max_vel=.1, max_acc=.1, confirm=True, **args))

    def test_actual_string_envelope_and_missing_confidence(self):
        event = types.SimpleNamespace(data=json.dumps({"code": "EVENT", "data": {"azimuth": 12, "confidence": .8}}))
        self.assertEqual(direction.parse_doa(event), (12, .8, True))
        self.assertEqual(direction.parse_doa('{"azimuth":-10}'), (-10, None, True))
        self.assertEqual(direction.parse_doa({"data": '{"azimuth":1}'}), (1, None, True))

    def test_bad_messages_never_become_zero(self):
        for event in ('null', '{}', 'bad', {"azimuth": True}, {"azimuth": float("nan")},
                      {"azimuth": 180}, {"azimuth": -181}, {"azimuth": 0, "confidence": 2},
                      {"azimuth": 0, "valid": "false"}):
            with self.subTest(event=event), self.assertRaises((ValueError, TypeError)):
                direction.parse_doa(event)

    def test_no_data_stale_expiry_and_restart(self):
        plugin, nodes = self.sound()
        self.assertFalse(plugin.dispatch("info", {})["valid"])
        self.assertIsNone(plugin.dispatch("info", {})["direction_degrees"])
        plugin._on_event('{"azimuth":20,"confidence":0.8}')
        self.assertTrue(plugin._state()["valid"])
        plugin._received -= 3
        plugin._expire()
        published = json.loads(plugin._publisher.publish.call_args.args[0].data)
        self.assertFalse(published["valid"])
        self.assertIsNone(published["direction_degrees"])
        self.assertEqual(published["last_direction_degrees"], 20)
        count = plugin._publisher.publish.call_count
        plugin._expire()
        self.assertEqual(plugin._publisher.publish.call_count, count)
        plugin.stop()
        plugin._on_event('{"azimuth":30}')
        plugin.start()
        self.assertIsNone(plugin._state()["received_at"])
        nodes.robot.create_subscription.assert_called_with(nodes.String, direction.DOA_TOPIC,
                                                           plugin._on_event, nodes._sensor_qos)
        nodes.set_system_enabled.assert_not_called()

    def test_wraparound_smoothing_invalid_and_recovery(self):
        plugin, _ = self.sound()
        plugin._on_event('{"azimuth":179,"confidence":0.5}')
        plugin._on_event('{"azimuth":-179,"confidence":0.8}')
        self.assertGreater(abs(plugin._state()["direction_degrees"]), 178)
        self.assertEqual(plugin._state()["confidence"], .8)
        plugin._on_event('invalid')
        self.assertFalse(plugin._state()["valid"])
        plugin._on_event('{"azimuth":0,"valid":false}')
        self.assertFalse(plugin._state()["valid"])
        plugin._on_event('{"azimuth":15}')
        self.assertEqual(plugin._state()["direction_degrees"], 15)
        self.assertIsNone(plugin._state()["confidence"])

    def test_sensor_validation_and_partial_start_cleanup(self):
        plugin = direction.SoundDirectionPlugin(self.nodes())
        for args in ({"alpha": 0}, {"alpha": True}, {"stale_after": float("inf")}, {"stale_after": -1}):
            with self.assertRaises(ValueError):
                plugin.start(args)
        plugin.nodes.robot.create_timer.side_effect = RuntimeError("timer unavailable")
        with self.assertRaisesRegex(RuntimeError, "timer unavailable"):
            plugin.start()
        self.assertFalse(plugin.running)
        self.assertIsNone(plugin._subscription)
        self.assertIsNone(plugin._publisher)

    def test_parameter_validation_and_no_feedback(self):
        plugin, _, _ = self.head()
        for args in ({"degree": 21}, {"degree": True}, {"degree": float("nan")},
                     {"max_vel": 0}, {"max_acc": -1}, {"timeout": 0}):
            values = dict(degree=5, max_vel=.1, max_acc=.1, confirm=True)
            values.update(args)
            with self.assertRaises(ValueError):
                plugin.turn(values)
        plugin._feedback_at -= 2
        self.assertIn("fresh neck", plugin.turn(dict(degree=5, max_vel=.1, max_acc=.1, confirm=True))["message"])
        plugin._client.call_async.assert_not_called()

    def test_turn_requires_strict_explicit_confirmation(self):
        plugin, audio, _ = self.head()
        for confirm in (None, False, "true", 1):
            args = dict(degree=5, max_vel=.1, max_acc=.1)
            if confirm is not None:
                args["confirm"] = confirm
            self.assertEqual(plugin.turn(args)["state"], "error")
            self.assertIsNone(audio._orientation_owner)
        plugin._client.call_async.assert_not_called()
        self.assertTrue(plugin.get_tool()["inputSchema"]["x-is-dangerous"])

    def test_single_backend_request_and_acceptance_not_arrival(self):
        plugin, audio, nodes = self.head()
        future = concurrent.futures.Future()
        plugin._client.call_async.return_value = future
        self.submit(plugin)
        self.assertEqual(plugin._state()["state"], "request_pending")
        request = plugin._client.call_async.call_args.args[0]
        self.assertEqual(request.degree, 5)
        self.assertFalse(request.with_waist)
        self.assertEqual(request.revolute_setup.max_vel, .1)
        self.assertEqual(request.prismatic_setup.max_vel, 0)
        future.set_result(types.SimpleNamespace(success=True, status=0, message="accepted"))
        self.assertEqual(plugin._state()["state"], "accepted_unverified")
        self.assertFalse(plugin._state()["arrival_confirmed"])
        self.assertIs(audio._orientation_owner, plugin)
        self.assertEqual(audio._queue("play_action", {}, "id")["state"], "error")
        nodes.string_call.assert_not_called()

    def test_timeout_stop_retain_reservation_and_late_response(self):
        plugin, audio, _ = self.head()
        future = concurrent.futures.Future()
        plugin._client.call_async.return_value = future
        self.submit(plugin)
        plugin._task["deadline"] = 0
        plugin._tick()
        self.assertEqual(plugin._state()["state"], "timeout_unconfirmed")
        self.assertIs(audio._orientation_owner, plugin)
        self.assertEqual(plugin.stop()["state"], "stop_unconfirmed")
        future.set_result(types.SimpleNamespace(success=True, status=0, message="accepted late"))
        self.assertEqual(plugin._state()["state"], "stop_unconfirmed")
        self.assertIs(audio._orientation_owner, plugin)
        future.cancel = mock.Mock()
        self.assertFalse(plugin.dispatch("center", {})["state"] == "completed")
        self.assertEqual(plugin._client.call_async.call_count, 1)

    def test_feedback_settles_only_after_changed_target_and_fresh_samples(self):
        plugin, audio, _ = self.head()
        future = concurrent.futures.Future()
        plugin._client.call_async.return_value = future
        self.submit(plugin)
        future.set_result(types.SimpleNamespace(success=True, status=0, message="accepted"))
        now = time.monotonic()
        with mock.patch.object(direction.time, "monotonic") as clock:
            for i in range(15):
                clock.return_value = now + i * .1
                plugin._on_feedback(neck())
            self.assertIs(audio._orientation_owner, plugin)
            for i in range(15):
                clock.return_value = now + 2 + i * .1
                plugin._on_feedback(neck(.1, .1))
        self.assertIsNone(audio._orientation_owner)
        self.assertEqual(plugin._state()["state"], "settled_unverified")
        self.assertFalse(plugin._state()["arrival_confirmed"])

    def test_vendor_rejection_exception_and_busy(self):
        for success, status in ((False, 0), (True, 2)):
            plugin, audio, _ = self.head()
            future = concurrent.futures.Future()
            plugin._client.call_async.return_value = future
            self.submit(plugin)
            future.set_result(types.SimpleNamespace(success=success, status=status, message="rejected"))
            self.assertEqual(plugin._state()["state"], "error")
            self.assertIsNone(audio._orientation_owner)
        plugin, audio, _ = self.head()
        audio._active = {"action_id": "existing"}
        self.assertEqual(plugin.turn(dict(degree=5, max_vel=.1, max_acc=.1, confirm=True))["state"], "error")
        plugin._client.call_async.assert_not_called()
        audio._active = None
        plugin._client.call_async.side_effect = RuntimeError("transport error")
        self.submit(plugin)
        self.assertEqual(plugin._state()["state"], "outcome_unknown")
        self.assertIs(audio._orientation_owner, plugin)

    def test_behavior_gate_and_check_can_be_stopped(self):
        plugin, audio, nodes = self.head()
        nodes.get_system_enabled.return_value = {"code": "OK", "data": {"enabled": True}}
        self.submit(plugin)
        self.assertEqual(plugin._state()["state"], "error")
        self.assertIsNone(audio._orientation_owner)
        plugin._client.call_async.assert_not_called()
        self.notify.reset_mock()
        self.completion.clear()
        entered, resume = threading.Event(), threading.Event()
        def read_state(*args):
            entered.set()
            resume.wait(2)
            return {"code": "OK", "data": {"enabled": False}}
        nodes.get_system_enabled.side_effect = read_state
        plugin.turn(dict(degree=5, max_vel=.1, max_acc=.1, confirm=True))
        self.assertTrue(entered.wait(1))
        self.assertEqual(plugin.stop()["state"], "cancelled_before_submission")
        resume.set()
        self.assertTrue(self.completion.wait(1))
        self.assertEqual(self.notify.call_args.args[1], "cancelled")
        self.assertIsNone(audio._orientation_owner)
        plugin._client.call_async.assert_not_called()

    def test_audio_stop_retains_ownership_until_matching_vendor_terminal(self):
        for response in ({"success": True}, {"success": False}, RuntimeError("interrupt failed")):
            plugin, audio, nodes = self.head()
            audio._active = {"action_id": "old", "vendor_uuid": "old-vendor", "tool_name": "head"}
            if isinstance(response, Exception):
                nodes.trigger_call.side_effect = response
            else:
                nodes.trigger_call.return_value = response
            with mock.patch.object(device, "_acp_notify") as notify:
                self.assertEqual(audio.stop()["state"], "interrupt_unconfirmed")
                self.assertEqual(plugin.turn(dict(degree=5, max_vel=.1, max_acc=.1, confirm=True))["state"], "error")
                notify.assert_not_called()
                audio._on_playback_state({"phase": "result", "uuid": "unrelated", "state_name": "COMPLETED"})
                self.assertIsNotNone(audio._active)
                audio._on_playback_state({"phase": "result", "uuid": "old-vendor", "state_name": "COMPLETED"})
                self.assertIsNone(audio._active)

    def test_system_enable_blocked_by_shared_reservation(self):
        plugin, audio, nodes = self.head()
        nodes._actuator_audio = audio
        audio._orientation_owner = plugin
        with self.assertRaisesRegex(RuntimeError, "head orientation"):
            device.U1Nodes.set_system_enabled(nodes, "vision_enabled", True)
        nodes.string_call.assert_not_called()

    def test_start_is_read_only_and_close_cleans_domain_two_resources(self):
        nodes = self.nodes()
        audio = device.AudioPlugin(nodes)
        plugin = direction.HeadOrientationPlugin(nodes, audio)
        service = types.ModuleType("uworld_motion_msgs.srv")
        service.SoundSource = types.SimpleNamespace(Request=object)
        messages = types.ModuleType("control_msgs.msg")
        messages.JointTrajectoryControllerState = object
        with mock.patch.dict(sys.modules, {"uworld_motion_msgs.srv": service, "control_msgs.msg": messages}), mock.patch.object(direction, "HeadServiceTransport") as transport:
            self.assertEqual(plugin.start()["state"], "ready")
            plugin.start()
        transport.assert_called_once_with()
        nodes.audio_device.create_client.assert_not_called()
        nodes.audio_device.create_subscription.assert_called_once()
        plugin._client.call_async.assert_not_called()
        plugin.close()
        transport.return_value.close.assert_called_once()
        nodes.audio_device.destroy_subscription.assert_called_once()
        nodes.audio_device.destroy_timer.assert_called_once()
        nodes.core.destroy_publisher.assert_called_once()

    def test_head_status_stream_updates_after_initial_checking_snapshot(self):
        plugin, _, nodes = self.head()
        plugin._publisher = mock.Mock()
        future = concurrent.futures.Future()
        plugin._client.call_async.return_value = future
        self.submit(plugin)
        plugin._tick()
        pending = json.loads(plugin._publisher.publish.call_args.args[0].data)
        self.assertEqual(pending["state"], "request_pending")
        future.set_result(types.SimpleNamespace(success=True, status=0, message=""))
        plugin._last_status_publish = 0
        plugin._tick()
        accepted = json.loads(plugin._publisher.publish.call_args.args[0].data)
        self.assertEqual(accepted["state"], "accepted_unverified")
        self.assertTrue(accepted["vendor_reported_success"])
        self.assertFalse(accepted["arrival_confirmed"])
        self.assertIn("尚未验证物理", accepted["status_message"])
        self.assertEqual(plugin.get_tool()["topic_out"][0]["topic"], "/test/u1_pro/head_orientation")

    def test_source_diagnostics_count_sdk_messages_and_only_read_switch_on_status(self):
        plugin, nodes = self.sound()
        nodes.get_system_enabled.assert_not_called()
        plugin._on_event("malformed")
        self.assertEqual(plugin.dispatch("info", {})["sdk_events_received"], 1)
        nodes.get_system_enabled.assert_not_called()
        state = plugin.dispatch("status", {})
        self.assertIs(state["vendor_wakeup_enabled"], False)
        self.assertIsNotNone(state["source_checked_at"])
        self.assertEqual(state["source_topic"], direction.DOA_TOPIC)
        nodes.set_system_enabled.assert_not_called()

    def test_feedback_must_still_be_fresh_at_submission(self):
        plugin, audio, nodes = self.head()
        def read_state(*args):
            plugin._feedback_at = time.monotonic() - 2
            return {"code": "OK", "data": {"enabled": False}}
        nodes.get_system_enabled.side_effect = read_state
        self.submit(plugin)
        plugin._client.call_async.assert_not_called()
        self.assertIsNone(audio._orientation_owner)
        self.assertIn("stale", plugin._state()["error"])

    def test_completion_is_correlated_once_and_never_claims_arrival(self):
        plugin, audio, _ = self.head()
        future = concurrent.futures.Future()
        plugin._client.call_async.return_value = future
        self.submit(plugin, action_id="web-turn-1")
        self.notify.assert_not_called()
        self.assertEqual(plugin._state()["action_id"], "web-turn-1")
        plugin._task["deadline"] = 0
        plugin._tick()
        self.assertTrue(self.completion.wait(1))
        self.notify.assert_called_once()
        action_id, status, result, tool = self.notify.call_args.args
        self.assertEqual((action_id, status, tool), ("web-turn-1", "error", "head_orientation"))
        self.assertFalse(result["arrival_confirmed"])
        self.assertEqual(result["state"], "timeout_unconfirmed")
        self.assertIs(audio._orientation_owner, plugin)
        future.set_exception(RuntimeError("late transport failure"))
        plugin._tick()
        plugin.stop()
        self.assertEqual(self.notify.call_count, 1)

    def test_sound_status_configure_and_no_data_heartbeat(self):
        plugin, _ = self.sound()
        self.assertEqual(plugin.dispatch("status", {})["data_state"], "no_data")
        actions = plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
        self.assertIn("status", actions)
        self.assertIn("configure", actions)
        result = plugin.dispatch("configure", {"alpha": .5, "stale_after": 3})
        self.assertEqual(result["settings"], {"alpha": .5, "stale_after": 3})
        plugin._last_publish -= 2
        count = plugin._publisher.publish.call_count
        plugin._expire()
        self.assertEqual(plugin._publisher.publish.call_count, count + 1)
        self.assertFalse(json.loads(plugin._publisher.publish.call_args.args[0].data)["valid"])
        with self.assertRaises(ValueError):
            plugin.dispatch("configure", {"alpha": 0})

    def test_unresolved_task_survives_restart_and_requires_explicit_ack(self):
        with tempfile.TemporaryDirectory() as temp:
            plugin, audio, nodes = self.head()
            plugin._state_path = Path(temp) / "guard.json"
            future = concurrent.futures.Future()
            plugin._client.call_async.return_value = future
            self.submit(plugin)
            self.assertTrue(json.loads(plugin._state_path.read_text())["unresolved"])
            nodes.config = {"head_orientation_state_file": str(plugin._state_path)}
            replacement_audio = device.AudioPlugin(nodes)
            restored = direction.HeadOrientationPlugin(nodes, replacement_audio)
            self.assertEqual(restored._state()["state"], "prior_outcome_unknown")
            self.assertIs(replacement_audio._orientation_owner, restored)
            self.assertEqual(restored.acknowledge({})["state"], "error")
            restored._client = mock.Mock(inflight=False)
            self.assertEqual(restored.acknowledge({"confirm": True})["state"], "error")
            now = time.monotonic()
            with mock.patch.object(direction.time, "monotonic") as clock:
                for i in range(26):
                    clock.return_value = now + i * .1
                    restored._on_feedback(neck())
                restored.acknowledge({"confirm": True})
            self.assertIsNone(replacement_audio._orientation_owner)
            self.assertFalse(restored._state()["arrival_confirmed"])
            self.assertFalse(json.loads(plugin._state_path.read_text())["unresolved"])
            self.assertEqual(restored._state()["state"], "operator_acknowledged_unverified")
            self.assertEqual(restored._generation, 1)
            self.assertIn("操作者确认后", restored._state()["error"])

    def test_center_is_not_advertised_and_busy_result_explains_owner(self):
        plugin, audio, _ = self.head()
        actions = plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
        self.assertNotIn("center", actions)
        audio._active = {"action_id": "prior", "tool_name": "head"}
        result = plugin.turn(dict(degree=5, max_vel=.1, max_acc=.1, confirm=True))
        self.assertEqual(result["blocking"]["kind"], "audio")
        self.assertEqual(result["blocking"]["task"]["action_id"], "prior")

    def test_unaccepted_turn_is_disabled_by_default_and_cannot_bypass_schema(self):
        nodes = self.nodes()
        plugin = direction.HeadOrientationPlugin(nodes, device.AudioPlugin(nodes))
        plugin._client = mock.Mock()
        schema = plugin.get_tool()["inputSchema"]
        self.assertNotIn("turn", schema["properties"]["action"]["enum"])
        self.assertNotIn("x-completion", schema)
        self.assertEqual(schema["x-resource"], "u1_action")
        result = plugin.dispatch("turn", {"degree": 5, "max_vel": .1, "max_acc": .1, "confirm": True})
        self.assertEqual(result["code"], "experimental_disabled")
        plugin._client.call_async.assert_not_called()
        self.assertFalse(plugin._state()["experimental_turn_enabled"])

    def test_schemas_and_runtime_dependencies(self):
        plugin, _, _ = self.head()
        self.assertEqual(plugin.get_tool()["type"], "actuator")
        self.assertEqual(plugin.get_tool()["inputSchema"]["x-completion"], {"actions": ["turn"], "timeout": 75})
        sensor, _ = self.sound()
        self.assertEqual(sensor.get_tool()["topic_out"][0]["format"], "data/json")
        root = Path(__file__).parent
        docker = (root / "Dockerfile").read_text()
        self.assertIn("direction.py", docker)
        self.assertEqual(docker.count("ros-humble-control-msgs"), 1)
        self.assertNotIn("ros-humble-control-msgs", docker.split("FROM ${ROS_BASE_IMAGE} AS runtime")[0])
        self.assertIn("float32 degree", (root / "vendor/uworld_motion_msgs/srv/SoundSource.srv").read_text())


if __name__ == "__main__":
    unittest.main()
