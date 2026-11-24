"""Stage B Card contracts with the real analyzer and an inert fake runtime.

No ROS or vendor SDK is loaded. FakeRuntime represents owned subscriptions;
it deliberately performs no image analysis, I/O, or background work itself.
"""

from __future__ import annotations

import ast
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch
from uuid import uuid4

import numpy as np


BUMI = Path(__file__).resolve().parents[1]
if str(BUMI) not in sys.path:
    sys.path.insert(0, str(BUMI))

from image_quality import ImageQualityAnalyzer, QualityConfig  # noqa: E402
from image_quality_guard import ImageQualityGuardPlugin  # noqa: E402


INPUT = "/robot/camera/color"
OUTPUT = INPUT + "/image_quality_guard"
EPOCH = "0beed014-9e46-4f4f-9169-cac2da31b0d7"


def bind_card(card, epoch=EPOCH):
    status = card.lifecycle_status()
    result = card.bind_epoch(driver_boot_id=status["driver_boot_id"],
                             expected_epoch=status["admission_epoch"], proposed_epoch=epoch)
    if result.get("state") == "error":
        raise AssertionError(result)
    return result


def ticket(card):
    active = card.info()["active_run_id"]
    if active:
        card._test_ticket = active
        return active
    previous = getattr(card, "_test_ticket", None)
    if previous is None or card._history.get(previous, {}).get("state") in {"closed", "cancelled", "failed"}:
        card._test_ticket = EPOCH + "." + str(uuid4())
    return card._test_ticket


def dispatch_start(card, args):
    return card.dispatch("start", {"run_id": ticket(card), **args})


def stop_card(card):
    run_id = card.info()["active_run_id"] or getattr(card, "_test_ticket", None)
    return card.stop(run_id)


def dark_texture():
    rows, columns = np.indices((96, 128))
    return np.where((rows + columns) % 2, 4, 36).astype(np.uint8)


def bundle_for_card(card):
    """Use the actual aggregation/dispatch methods without running main."""
    tree = ast.parse((BUMI / "main.py").read_text(encoding="utf-8"))
    bundle_class = next(node for node in tree.body
                        if isinstance(node, ast.ClassDef)
                        and node.name == "BumiDeviceBundle")
    module = ast.Module(body=[ast.ImportFrom(
        module="__future__", names=[ast.alias(name="annotations")], level=0),
        bundle_class], type_ignores=[])
    namespace = {}
    exec(compile(ast.fix_missing_locations(module), "bumi_bundle_only", "exec"),
         namespace)
    bundle_type = namespace["BumiDeviceBundle"]
    bundle = bundle_type.__new__(bundle_type)
    bundle._plugins = [card]
    return bundle


class FakeRuntime:
    """No resource allocation in the constructor; start/close own the fake one."""

    def __init__(self):
        self.start_calls = []
        self.close_calls = 0
        self.active = False
        self.gate = None
        self.fail_start = False
        self.close_failures = 0

    def start(self, *, input_topic, output_topic, run_if_current, run_id, analysis_hz):
        self.run_id, self.analysis_hz = run_id, analysis_hz
        self.start_calls.append({"input_topic": input_topic,
                                 "output_topic": output_topic})
        self.gate = run_if_current
        self.active = True
        if self.fail_start:
            raise RuntimeError("synthetic start failure")

    def close(self):
        self.close_calls += 1
        if self.close_failures:
            self.close_failures -= 1
            raise RuntimeError("synthetic close failure")
        self.active = False


class RuntimeFactory:
    def __init__(self, maker=FakeRuntime):
        self.maker = maker
        self.calls = []
        self.instances = []

    def __call__(self, *, namespace, executor):
        self.calls.append({"namespace": namespace, "executor": executor})
        runtime = self.maker()
        self.instances.append(runtime)
        return runtime


class SlowRuntime(FakeRuntime):
    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()
        self.accepted_while_starting = None

    def start(self, *, input_topic, output_topic, run_if_current, run_id, analysis_hz):
        super().start(input_topic=input_topic, output_topic=output_topic,
                      run_if_current=run_if_current, run_id=run_id, analysis_hz=analysis_hz)
        self.accepted_while_starting = run_if_current(
            lambda analyzer: analyzer.analyze_image(dark_texture()))
        self.entered.set()
        if not self.release.wait(3):
            raise RuntimeError("test failed to release slow start")


class BlockingCloseRuntime(FakeRuntime):
    """Pause cleanup after stop has captured its runtime ownership."""

    def __init__(self):
        super().__init__()
        self.close_entered = threading.Event()
        self.close_release = threading.Event()

    def close(self):
        self.close_entered.set()
        if not self.close_release.wait(3):
            raise RuntimeError("test failed to release close")
        super().close()


class GatedLifecycleLock:
    """Test-only scheduling at lock admission; never changes plugin state."""

    def __init__(self, *thread_names):
        self._lock = threading.Lock()
        self.gates = {name: (threading.Event(), threading.Event())
                      for name in thread_names}

    def __enter__(self):
        gate = self.gates.get(threading.current_thread().name)
        if gate is not None:
            entered, release = gate
            entered.set()
            if not release.wait(3):
                raise RuntimeError("test failed to release lifecycle admission")
        self._lock.acquire()
        return self

    def __exit__(self, *_):
        self._lock.release()


class GuardTestCase(unittest.TestCase):
    def card(self, config=None, factory=None):
        if factory is None:
            factory = RuntimeFactory()
        executor = object()
        card = ImageQualityGuardPlugin(config, "robot", executor,
                                       runtime_factory=factory)
        bind_card(card)
        self.addCleanup(card.shutdown)
        return card, factory, executor

    def start_card(self, card, **extra):
        return dispatch_start(card, {"input_topic": INPUT, **extra})

    def assert_rejected_without_running(self, result):
        self.assertIsInstance(result, dict)
        self.assertNotEqual(result.get("state"), "running")


class SchemaAndPreviewTests(GuardTestCase):
    def test_constructor_boot_start_and_schema_do_not_allocate_runtime(self):
        card, factory, _ = self.card()
        self.assertEqual(factory.calls, [])
        self.assertIsNone(card.start())
        card.get_tool()
        card.get_tools()
        card.info()
        self.assertEqual(factory.calls, [])
        self.assertEqual(card.info()["state"], "idle")
        self.assertFalse(card.info()["resources_pending"])

    def test_default_runtime_reports_unavailable_ros_without_false_subscription(self):
        card = ImageQualityGuardPlugin({}, "robot", object())
        self.addCleanup(card.shutdown)
        bind_card(card)
        run_id = EPOCH + "." + str(uuid4())
        with patch("image_quality_runtime._ros_bindings", side_effect=ImportError("ROS2 unavailable")):
            result = card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})
        self.assertEqual(result["state"], "error")
        self.assertEqual(result["code"], "START_FAILED")
        self.assertIn("ROS2 unavailable", result["error"])
        self.assertTrue(card.info()["runtime_configured"])
        self.assertFalse(card.info()["resources_pending"])
        self.assertEqual(card.stop(run_id)["code"], "ALREADY_STOPPED")
        self.assertEqual(card.stop(run_id)["code"], "ALREADY_STOPPED")

    def test_tool_declares_processor_single_instance_and_only_system_actions(self):
        card, _, _ = self.card()
        tool = card.get_tool()
        self.assertEqual(card.PREFIX, "image_quality_guard")
        self.assertEqual(tool["name"], card.PREFIX)
        self.assertEqual(tool["type"], "processor")
        self.assertIs(tool["multiInstance"], False)
        properties = tool["inputSchema"]["properties"]
        self.assertEqual(set(properties["action"]["enum"]),
                         {"start", "stop", "info", "config"})
        self.assertEqual(properties["input_topic"]["type"], "string")
        self.assertEqual(properties["input_topics"]["type"], "array")
        self.assertEqual(properties["input_topics"]["items"]["type"], "string")
        self.assertEqual(tool["topic_in"][0]["format"], "image/jpeg")
        self.assertEqual(tool["topic_out"][0]["format"], "data/json")
        self.assertNotIn("topic", tool["topic_in"][0])
        self.assertNotIn("topic", tool["topic_out"][0])
        split_actions = tool["inputSchema"].get("x-action-params", {})
        self.assertLessEqual(set(split_actions), {"start", "stop", "info", "config"})
        for metadata in ("x-completion", "x-resource"):
            self.assertNotIn(metadata, tool["inputSchema"])
            self.assertNotIn(metadata, tool)

    def test_config_schema_matches_ten_real_algorithm_fields_and_defaults(self):
        card, _, _ = self.card()
        defaults = asdict(QualityConfig())
        properties = card.get_tool()["configSchema"]["properties"]
        self.assertEqual(set(properties), set(defaults))
        self.assertEqual(len(properties), 10)
        for name, value in defaults.items():
            with self.subTest(name=name):
                self.assertEqual(properties[name]["default"], value)
                self.assertEqual(properties[name]["scope"], "shared")
        self.assertEqual(card.info()["config"], defaults)

    def test_get_tool_and_get_tools_return_independent_nested_metadata(self):
        card, _, _ = self.card()
        original = card.get_tool()
        edited = card.get_tool()
        edited["inputSchema"]["properties"]["action"]["enum"].append("injected")
        edited["topic_out"][0]["format"] = "injected"
        group = card.get_tools()
        group[0]["configSchema"]["properties"]["analysis_hz"]["default"] = -1
        self.assertEqual(card.get_tool(), original)
        self.assertEqual(card.get_tools(), [original])

    def test_idle_info_preview_derives_topics_without_binding_or_starting(self):
        card, factory, _ = self.card()
        baseline = card.info()
        self.assertEqual(baseline["topic_in"], [])
        self.assertEqual(baseline["topic_out"], [])
        preview = card.info(input_topic=INPUT)
        self.assertEqual(preview["state"], "idle")
        self.assertEqual(preview["input_topic"], INPUT)
        self.assertEqual(preview["output_topic"], OUTPUT)
        self.assertEqual(preview["topic_in"][0]["topic"], INPUT)
        self.assertEqual(preview["topic_out"][0]["topic"], OUTPUT)
        self.assertEqual(preview["quality_state"], "UNKNOWN")
        self.assertEqual(card.info(), baseline)
        self.assertEqual(factory.calls, [])

    def test_dispatch_info_preview_and_bare_name_have_no_runtime_side_effects(self):
        card, factory, _ = self.card()
        args = {"input_topic": INPUT, "instance_id": "card-1",
                "_tool_name": card.PREFIX, "_trace_id": "trace-1",
                "camera_info": {"source": {"width": 640}},
                "input_bindings": [{"topic": INPUT, "format": "image/jpeg"}]}
        before = deepcopy(args)
        result = card.dispatch("info", args)
        self.assertEqual(result["output_topic"], OUTPUT)
        self.assertEqual(args, before)
        self.assertEqual(card.dispatch(card.PREFIX), card.info())
        self.assertEqual(factory.calls, [])

    def test_plugin_config_accepts_enabled_flag_without_polluting_algorithm_fields(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                card, factory, _ = self.card({"enabled": enabled,
                                              "confirm_samples": 2})
                self.assertEqual(card.info()["config"]["confirm_samples"], 2)
                self.assertNotIn("enabled", card.info()["config"])
                self.assertEqual(factory.calls, [])
        for invalid in ({"enabled": 1}, {"enabled": "true"},
                        {"output_topic": "/override"}, {"unknown": True}):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    ImageQualityGuardPlugin(invalid, "robot", object())


class BindingAndDispatchTests(GuardTestCase):
    def test_successful_start_calls_factory_and_runtime_with_actual_binding(self):
        card, factory, executor = self.card()
        args = {"input_topic": INPUT, "instance_id": "card-1",
                "_tool_name": card.PREFIX, "_trace_id": "trace-1",
                "camera_info": {"intrinsics": [1, 2, 3]},
                "input_bindings": [{"topic": INPUT, "format": "image/jpeg"}]}
        before = deepcopy(args)
        result = dispatch_start(card, args)
        self.assertEqual(result["state"], "running")
        self.assertEqual(args, before)
        self.assertEqual(factory.calls, [{"namespace": "robot",
                                          "executor": executor}])
        runtime = factory.instances[0]
        self.assertEqual(runtime.start_calls,
                         [{"input_topic": INPUT, "output_topic": OUTPUT}])
        self.assertTrue(runtime.active)
        info = card.info()
        self.assertTrue(info["runtime_configured"])
        self.assertEqual(info["input_topic"], INPUT)
        self.assertEqual(info["output_topic"], OUTPUT)
        self.assertEqual(info["quality_state"], "UNKNOWN")
        self.assertEqual(info["quality"], ImageQualityAnalyzer().snapshot())

    def test_running_info_does_not_replace_binding_with_preview_argument(self):
        card, factory, _ = self.card()
        self.start_card(card)
        result = card.info(input_topic="/another/camera")
        self.assertEqual(result["input_topic"], INPUT)
        self.assertEqual(result["output_topic"], OUTPUT)
        self.assertEqual(len(factory.calls), 1)

    def test_duplicate_same_binding_start_is_idempotent(self):
        card, factory, _ = self.card()
        self.start_card(card, instance_id="card-1")
        result = self.start_card(card, instance_id="card-1")
        self.assertEqual(result["state"], "running")
        self.assertEqual(len(factory.calls), 1)
        self.assertEqual(len(factory.instances[0].start_calls), 1)

    def test_changed_running_topic_or_instance_is_rejected_and_binding_stays_actual(self):
        card, factory, _ = self.card()
        self.start_card(card, instance_id="card-1")
        for args in ({"input_topic": "/robot/another", "instance_id": "card-1"},
                     {"input_topic": INPUT, "instance_id": "card-2"}):
            with self.subTest(args=args):
                before = deepcopy(args)
                result = dispatch_start(card, args)
                self.assertTrue(result.get("error") or result.get("code"))
                self.assertEqual(args, before)
                self.assertEqual(card.info()["state"], "running")
                self.assertEqual(card.info()["input_topic"], INPUT)
                self.assertEqual(len(factory.calls), 1)

    def test_single_duplicate_topic_list_is_deduplicated(self):
        card, factory, _ = self.card()
        args = {"input_topics": [INPUT, INPUT], "input_topic": INPUT}
        before = deepcopy(args)
        result = dispatch_start(card, args)
        self.assertEqual(result["state"], "running")
        self.assertEqual(args, before)
        self.assertEqual(factory.instances[0].start_calls[0]["input_topic"], INPUT)

    def test_missing_invalid_and_multiple_distinct_topics_are_rejected_before_factory(self):
        invalid_args = [
            {}, {"input_topic": ""}, {"input_topic": "camera/color"},
            {"input_topic": "/camera//color"}, {"input_topic": "/camera/../color"},
            {"input_topic": "/camera/color/"}, {"input_topic": "/camera/color image"},
            {"input_topic": "/camera/*"}, {"input_topic": True},
            {"input_topics": INPUT}, {"input_topics": [INPUT, 4]},
            {"input_topics": [INPUT, "/other/camera"]},
            {"input_topic": INPUT, "input_topics": ["/other/camera"]},
            {"input_topic": INPUT, "instance_id": 123},
        ]
        for args in invalid_args:
            with self.subTest(args=args):
                card, factory, _ = self.card()
                before = deepcopy(args)
                self.assert_rejected_without_running(dispatch_start(card, args))
                self.assertEqual(args, before)
                self.assertEqual(factory.calls, [])
                self.assertFalse(card.info()["resources_pending"])

    def test_unknown_dispatch_keys_and_output_override_are_rejected_before_factory(self):
        for key, value in (("unknown", 1), ("output_topic", "/forbidden")):
            with self.subTest(key=key):
                card, factory, _ = self.card()
                self.assert_rejected_without_running(card.dispatch(
                    "start", {"input_topic": INPUT, key: value}))
                self.assertEqual(factory.calls, [])

    def test_unknown_action_returns_none_without_modifying_args(self):
        card, factory, _ = self.card()
        args = {"nested": {"value": [1, 2]}}
        before = deepcopy(args)
        self.assertIsNone(card.dispatch("take_photo", args))
        self.assertEqual(args, before)
        self.assertEqual(factory.calls, [])

    def test_real_bundle_can_aggregate_and_dispatch_card_without_sdk_or_main(self):
        card, _, _ = self.card()
        bundle = bundle_for_card(card)
        self.assertEqual(bundle.get_all_tools(), card.get_tools())
        self.assertEqual(bundle.dispatch(card.PREFIX, {"action": "info"}),
                         card.info())

    def test_bundle_accepts_canvas_disconnect_stop_with_topic_and_instance(self):
        card, factory, _ = self.card()
        bundle = bundle_for_card(card)
        binding = {"input_topic": INPUT, "instance_id": "canvas-card-1", "run_id": ticket(card)}
        started = bundle.dispatch(card.PREFIX, {"action": "start", **binding})
        self.assertEqual(started["state"], "running")
        runtime = factory.instances[0]
        stopped = bundle.dispatch(card.PREFIX, {"action": "stop", **binding})
        self.assertEqual(stopped["state"], "idle")
        self.assertEqual(card.info()["state"], "idle")
        self.assertFalse(runtime.active)
        self.assertEqual(runtime.close_calls, 1)
        repeated = bundle.dispatch(card.PREFIX, {"action": "stop", **binding})
        self.assertEqual(repeated["state"], "idle")
        self.assertEqual(runtime.close_calls, 1)

    def test_canvas_stop_mismatch_keeps_current_run_until_matching_disconnect(self):
        card, factory, _ = self.card()
        bundle = bundle_for_card(card)
        binding = {"input_topic": INPUT, "instance_id": "canvas-card-1", "run_id": ticket(card)}
        bundle.dispatch(card.PREFIX, {"action": "start", **binding})
        runtime = factory.instances[0]
        for mismatch in ({"input_topic": "/robot/old_camera", "instance_id": "canvas-card-1"},
                         {"input_topic": INPUT, "instance_id": "canvas-card-old"}):
            with self.subTest(mismatch=mismatch):
                result = bundle.dispatch(card.PREFIX, {"action": "stop", **mismatch})
                self.assertTrue(result.get("error") or result.get("code"))
                self.assertEqual(card.info()["state"], "running")
                self.assertEqual(card.info()["input_topic"], INPUT)
                self.assertTrue(runtime.active)
                self.assertEqual(runtime.close_calls, 0)
        matched = bundle.dispatch(card.PREFIX, {"action": "stop", **binding})
        self.assertEqual(matched["state"], "idle")
        self.assertFalse(runtime.active)
        self.assertEqual(runtime.close_calls, 1)


class GateAndLifecycleTests(GuardTestCase):
    def test_runtime_gate_runs_real_analyzer_and_debounces_low_light(self):
        card, factory, _ = self.card()
        self.start_card(card)
        runtime = factory.instances[0]
        for _ in range(3):
            self.assertTrue(runtime.gate(
                lambda analyzer: analyzer.analyze_image(dark_texture())))
        info = card.info()
        self.assertEqual(info["state"], "running")
        self.assertEqual(info["quality_state"], "DEGRADED")
        self.assertEqual(info["quality"]["reasons"], ["LOW_LIGHT"])
        self.assertEqual(info["quality"]["metrics"]["mean_luminance"], 20.0)
        encoded = json.dumps(info, allow_nan=False)
        self.assertLess(len(encoded), 8192)
        self.assertEqual(json.loads(encoded), info)

    def test_stop_is_idempotent_and_old_gate_cannot_run_after_stop(self):
        card, factory, _ = self.card()
        self.start_card(card)
        runtime = factory.instances[0]
        old_gate = runtime.gate
        self.assertEqual(stop_card(card)["state"], "idle")
        self.assertFalse(runtime.active)
        calls = []
        self.assertFalse(old_gate(lambda analyzer: calls.append(analyzer)))
        self.assertEqual(calls, [])
        self.assertEqual(card.dispatch("stop", {"run_id": getattr(card, "_test_ticket", None)})["state"], "idle")
        self.assertEqual(runtime.close_calls, 1)
        self.assertFalse(card.info()["resources_pending"])

    def test_old_generation_gate_stays_invalid_after_new_start(self):
        card, factory, _ = self.card()
        self.start_card(card)
        old_gate = factory.instances[0].gate
        stop_card(card)
        self.start_card(card)
        new_gate = factory.instances[1].gate
        calls = []
        self.assertFalse(old_gate(lambda analyzer: calls.append("old")))
        self.assertTrue(new_gate(lambda analyzer: calls.append("new")))
        self.assertEqual(calls, ["new"])
        self.assertEqual(card.info()["quality_state"], "UNKNOWN")

    def test_start_failure_closes_partial_runtime_and_allows_clean_retry(self):
        first = FakeRuntime()
        first.fail_start = True
        second = FakeRuntime()
        instances = iter([first, second])
        card, factory, _ = self.card(factory=RuntimeFactory(lambda: next(instances)))
        result = self.start_card(card)
        self.assertEqual(result["state"], "error")
        self.assertEqual(first.close_calls, 1)
        self.assertFalse(first.active)
        self.assertFalse(card.info()["resources_pending"])
        self.assertEqual(self.start_card(card)["state"], "running")
        self.assertEqual(len(factory.calls), 2)
        self.assertTrue(second.active)

    def test_close_failure_retains_owner_blocks_replacement_and_stop_retries(self):
        first = FakeRuntime()
        first.close_failures = 1
        second = FakeRuntime()
        instances = iter([first, second])
        card, factory, _ = self.card(factory=RuntimeFactory(lambda: next(instances)))
        self.start_card(card)
        gate = first.gate
        result = stop_card(card)
        self.assertEqual(result["state"], "error")
        self.assertTrue(card.info()["resources_pending"])
        calls = []
        self.assertFalse(gate(lambda analyzer: calls.append(True)))
        self.assertEqual(calls, [])
        self.assert_rejected_without_running(self.start_card(card))
        self.assertEqual(len(factory.calls), 1)
        self.assertTrue(first.active)
        self.assertEqual(stop_card(card)["state"], "idle")
        self.assertEqual(first.close_calls, 2)
        self.assertFalse(first.active)
        self.assertFalse(card.info()["resources_pending"])
        self.assertEqual(self.start_card(card)["state"], "running")
        self.assertEqual(len(factory.calls), 2)

    def test_start_failure_and_cleanup_failure_require_later_stop_retry(self):
        runtime = FakeRuntime()
        runtime.fail_start = True
        runtime.close_failures = 1
        card, factory, _ = self.card(factory=RuntimeFactory(lambda: runtime))
        self.assertEqual(self.start_card(card)["state"], "error")
        self.assertEqual(runtime.close_calls, 1)
        self.assertTrue(card.info()["resources_pending"])
        self.assert_rejected_without_running(self.start_card(card))
        self.assertEqual(len(factory.calls), 1)
        self.assertEqual(stop_card(card)["state"], "idle")
        self.assertFalse(card.info()["resources_pending"])

    def test_runtime_factory_must_supply_callable_start_and_close(self):
        class MissingStart:
            def close(self):
                pass

        class MissingClose:
            def start(self, **kwargs):
                pass

        for maker in (lambda: object(), MissingStart, MissingClose):
            with self.subTest(maker=maker):
                card, _, _ = self.card(factory=RuntimeFactory(maker))
                self.assert_rejected_without_running(self.start_card(card))
                self.assertNotEqual(card.info()["state"], "running")

    def test_slow_start_cannot_analyze_while_starting_and_config_requires_stop(self):
        runtime = SlowRuntime()
        card, _, _ = self.card(factory=RuntimeFactory(lambda: runtime))
        result = {}
        errors = []

        def start():
            try:
                result.update(self.start_card(card))
            except BaseException as error:
                errors.append(error)

        thread = threading.Thread(target=start, daemon=True)
        thread.start()
        try:
            self.assertTrue(runtime.entered.wait(1))
            self.assertEqual(card.info()["state"], "starting")
            self.assertFalse(runtime.accepted_while_starting)
            configured = card.dispatch("config", {"analysis_hz": 3.0})
            self.assertEqual(configured["code"], "STOP_REQUIRED")
            self.assertEqual(card.info()["config"]["analysis_hz"], 2.0)
        finally:
            runtime.release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(result["state"], "running")

    def test_stop_cancels_slow_start_and_late_start_cannot_revive_running(self):
        runtime = SlowRuntime()
        card, factory, _ = self.card(factory=RuntimeFactory(lambda: runtime))
        started = threading.Event()
        stopped = threading.Event()
        results = {}
        errors = []

        def start():
            try:
                results["start"] = self.start_card(card)
            except BaseException as error:
                errors.append(error)

        def stop():
            started.set()
            try:
                results["stop"] = stop_card(card)
            except BaseException as error:
                errors.append(error)
            finally:
                stopped.set()

        start_thread = threading.Thread(target=start, daemon=True)
        stop_thread = threading.Thread(target=stop, daemon=True)
        start_thread.start()
        try:
            self.assertTrue(runtime.entered.wait(1))
            stop_thread.start()
            self.assertTrue(started.wait(1))
            deadline = time.monotonic() + 1.0
            while card.info()["state"] != "stopping" and time.monotonic() < deadline:
                stopped.wait(min(0.01, max(0.0, deadline - time.monotonic())))
            self.assertEqual(card.info()["state"], "stopping")
            self.assertFalse(stopped.is_set())
            self.assertEqual(runtime.close_calls, 0)
        finally:
            runtime.release.set()
            start_thread.join(2)
            if stop_thread.ident is not None:
                stop_thread.join(2)
        self.assertFalse(start_thread.is_alive())
        self.assertFalse(stop_thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(card.info()["state"], "idle")
        self.assertEqual(results["stop"]["state"], "idle")
        self.assertFalse(runtime.active)
        self.assertEqual(runtime.close_calls, 1)
        self.assertEqual(len(factory.calls), 1)
        calls = []
        self.assertFalse(runtime.gate(lambda analyzer: calls.append(True)))
        self.assertEqual(calls, [])


class ConcurrentStopTests(GuardTestCase):
    def launch(self, name, operation, threads):
        outcome = {}

        def run():
            try:
                outcome["result"] = operation()
            except BaseException as error:
                outcome["error"] = error

        thread = threading.Thread(target=run, name=name, daemon=True)
        threads.append((thread, outcome))
        thread.start()
        return thread, outcome

    def finish(self, worker):
        thread, outcome = worker
        thread.join(2)
        self.assertFalse(thread.is_alive(), f"worker did not finish: {thread.name}")
        self.assertNotIn("error", outcome, repr(outcome.get("error")))
        return outcome["result"]

    def release_and_finish(self, releases, threads):
        # Registered after card.stop, so unittest releases every test gate first.
        for release in releases:
            release.set()
        for worker in threads:
            self.finish(worker)

    def overlapping_closes(self, *, fail_first_close=False):
        first, second = BlockingCloseRuntime(), FakeRuntime()
        first.close_failures = int(fail_first_close)
        instances = iter([first, second])
        card, factory, _ = self.card(factory=RuntimeFactory(lambda: next(instances)))
        self.start_card(card)
        lock = GatedLifecycleLock("new-stop")
        card._lifecycle_lock = lock
        admitted, resume = lock.gates["new-stop"]
        threads = []
        self.addCleanup(self.release_and_finish,
                        [first.close_release, resume], threads)
        old = self.launch("old-stop", lambda: stop_card(card), threads)
        self.assertTrue(first.close_entered.wait(1))
        new = self.launch("new-stop", lambda: stop_card(card), threads)
        self.assertTrue(admitted.wait(1))
        self.assertEqual(card.info()["state"], "stopping")
        first.close_release.set()
        old_result = self.finish(old)
        return card, factory, first, second, resume, new, old_result

    def test_pending_second_stop_keeps_restart_busy_until_cleanup_owner_finishes(self):
        card, factory, first, second, resume, new, _ = self.overlapping_closes()
        attempted = self.start_card(card)
        self.assertEqual(attempted.get("code"), "BUSY")
        self.assertEqual(card.info()["state"], "stopping")
        self.assertEqual(first.close_calls, 1)
        self.assertEqual(len(factory.calls), 1)
        resume.set()
        self.assertEqual(self.finish(new)["state"], "idle")
        self.assertEqual(self.start_card(card)["state"], "running")
        self.assertTrue(second.active)
        self.assertEqual(second.close_calls, 0)

    def restart_before_old_stop_resumes(self, *, instance_id, stop_with_binding):
        first, second = FakeRuntime(), FakeRuntime()
        instances = iter([first, second])
        card, factory, _ = self.card(factory=RuntimeFactory(lambda: next(instances)))
        binding = {"input_topic": INPUT, "run_id": ticket(card)}
        if instance_id is not None:
            binding["instance_id"] = instance_id
        self.assertEqual(card.dispatch("start", binding)["state"], "running")
        lock = GatedLifecycleLock("old-stop", "new-stop")
        card._lifecycle_lock = lock
        old_entered, old_resume = lock.gates["old-stop"]
        new_entered, new_resume = lock.gates["new-stop"]
        threads = []
        self.addCleanup(self.release_and_finish, [old_resume, new_resume], threads)
        stop = (lambda: card.dispatch("stop", binding)) if stop_with_binding else lambda: stop_card(card)
        old = self.launch("old-stop", stop, threads)
        self.assertTrue(old_entered.wait(1))
        new = self.launch("new-stop", stop, threads)
        self.assertTrue(new_entered.wait(1))
        new_resume.set()
        self.assertEqual(self.finish(new)["state"], "idle")
        self.assertEqual(first.close_calls, 1)
        self.assertEqual(card.dispatch("start", {**binding, "run_id": ticket(card)})["state"], "running")
        expected = card.info()
        old_resume.set()
        self.finish(old)
        self.assertEqual(card.info(), expected)
        self.assertEqual(len(factory.calls), 2)
        self.assertTrue(second.active)
        self.assertEqual(second.close_calls, 0)
        self.assertTrue(second.gate(lambda analyzer: analyzer.snapshot()))

    def test_old_stop_cannot_close_restart_with_same_topic_and_instance_id(self):
        self.restart_before_old_stop_resumes(instance_id="canvas-card-1",
                                            stop_with_binding=True)

    def test_old_unqualified_stop_cannot_close_restart_without_instance_id(self):
        self.restart_before_old_stop_resumes(instance_id=None, stop_with_binding=False)

    def test_repeated_stops_release_runtime_once_and_remain_idle(self):
        card, factory, _ = self.card()
        self.start_card(card)
        runtime = factory.instances[0]
        for _ in range(4):
            self.assertEqual(stop_card(card)["state"], "idle")
        self.assertEqual(runtime.close_calls, 1)
        self.assertFalse(runtime.active)
        self.assertFalse(card.info()["resources_pending"])
        self.assertEqual(card.info()["state"], "idle")

    def test_superseded_failed_cleanup_preserves_pending_stop_and_owner_for_retry(self):
        card, factory, first, second, resume, new, old_result = self.overlapping_closes(
            fail_first_close=True)
        self.assertEqual(old_result["code"], "CLEANUP_FAILED")
        self.assertEqual(card.info()["state"], "stopping")
        self.assertTrue(card.info()["resources_pending"])
        self.assertTrue(first.active)
        self.assertEqual(first.close_calls, 1)
        self.assertEqual(self.start_card(card).get("code"), "BUSY")
        self.assertEqual(len(factory.calls), 1)
        resume.set()
        self.assertEqual(self.finish(new)["state"], "idle")
        self.assertEqual(first.close_calls, 2)
        self.assertFalse(first.active)
        self.assertFalse(card.info()["resources_pending"])
        self.assertEqual(self.start_card(card)["state"], "running")
        self.assertTrue(second.active)
        self.assertEqual(second.close_calls, 0)

    def test_superseded_shutdown_cannot_finalize_newer_shutdown(self):
        for fail_first_close in (False, True):
            with self.subTest(fail_first_close=fail_first_close):
                runtime = BlockingCloseRuntime()
                runtime.close_failures = int(fail_first_close)
                card, _, _ = self.card(factory=RuntimeFactory(lambda: runtime))
                self.start_card(card)
                run_id = card.info()["run_id"]
                lock = GatedLifecycleLock("new-shutdown")
                card._lifecycle_lock = lock
                entered, resume = lock.gates["new-shutdown"]
                threads = []
                self.addCleanup(self.release_and_finish,
                                [runtime.close_release, resume], threads)

                old = self.launch("old-shutdown", card.shutdown, threads)
                self.assertTrue(runtime.close_entered.wait(1))
                new = self.launch("new-shutdown", card.shutdown, threads)
                self.assertTrue(entered.wait(1))
                runtime.close_release.set()
                self.assertEqual(self.finish(old)["state"], "stopping")

                # The newer shutdown still owns finalization, including when
                # the older close failed and its resource must be retried.
                pending = card.info()
                self.assertEqual(pending["state"], "stopping")
                self.assertEqual(pending["run_id"], run_id)
                self.assertTrue(pending["shutdown"])
                self.assertIsNone(pending["error"])
                self.assertEqual(pending["resources_pending"], fail_first_close)
                self.assertEqual(runtime.close_calls, 1)

                resume.set()
                self.assertEqual(self.finish(new)["state"], "idle")
                self.assertEqual(runtime.close_calls, 1 + int(fail_first_close))
                self.assertFalse(runtime.active)
                self.assertIsNone(card.info()["run_id"])
                self.assertFalse(card.info()["resources_pending"])

    def test_two_stops_cancel_slow_start_and_release_partial_runtime_only_once(self):
        runtime = SlowRuntime()
        card, factory, _ = self.card(factory=RuntimeFactory(lambda: runtime))
        lock = GatedLifecycleLock("old-stop", "new-stop")
        card._lifecycle_lock = lock
        old_entered, old_resume = lock.gates["old-stop"]
        new_entered, new_resume = lock.gates["new-stop"]
        threads = []
        self.addCleanup(self.release_and_finish,
                        [runtime.release, old_resume, new_resume], threads)
        start = self.launch("slow-start", lambda: self.start_card(card), threads)
        self.assertTrue(runtime.entered.wait(1))
        old = self.launch("old-stop", lambda: stop_card(card), threads)
        self.assertTrue(old_entered.wait(1))
        new = self.launch("new-stop", lambda: stop_card(card), threads)
        self.assertTrue(new_entered.wait(1))
        runtime.release.set()
        self.assertEqual(self.finish(start)["code"], "START_CANCELLED")
        old_resume.set()
        self.finish(old)
        self.assertEqual(card.info()["state"], "stopping")
        self.assertEqual(runtime.close_calls, 1)
        self.assertFalse(runtime.active)
        new_resume.set()
        self.assertEqual(self.finish(new)["state"], "idle")
        self.assertEqual(runtime.close_calls, 1)
        self.assertEqual(len(factory.calls), 1)
        self.assertFalse(card.info()["resources_pending"])

    def test_cancelled_start_cleanup_failure_cannot_overwrite_newer_stop(self):
        runtime = SlowRuntime()
        runtime.close_failures = 1
        close_entered, close_release = threading.Event(), threading.Event()
        original_close = runtime.close

        def close():
            close_entered.set()
            if not close_release.wait(3):
                raise RuntimeError("test failed to release cancelled-start cleanup")
            original_close()

        runtime.close = close
        card, _, _ = self.card(factory=RuntimeFactory(lambda: runtime))
        lock = GatedLifecycleLock("old-stop", "new-stop")
        card._lifecycle_lock = lock
        old_entered, old_resume = lock.gates["old-stop"]
        new_entered, new_resume = lock.gates["new-stop"]
        threads = []
        self.addCleanup(self.release_and_finish,
                        [runtime.release, close_release, old_resume, new_resume], threads)
        start = self.launch("slow-start", lambda: self.start_card(card), threads)
        self.assertTrue(runtime.entered.wait(1))
        old = self.launch("old-stop", lambda: stop_card(card), threads)
        self.assertTrue(old_entered.wait(1))
        runtime.release.set()
        self.assertTrue(close_entered.wait(1))
        new = self.launch("new-stop", lambda: stop_card(card), threads)
        self.assertTrue(new_entered.wait(1))
        close_release.set()
        self.assertEqual(self.finish(start)["code"], "CLEANUP_FAILED")
        self.assertEqual(card.info()["state"], "stopping")
        self.assertTrue(card.info()["resources_pending"])
        self.assertTrue(runtime.active)
        old_resume.set()
        self.finish(old)
        self.assertEqual(card.info()["state"], "stopping")
        new_resume.set()
        self.assertEqual(self.finish(new)["state"], "idle")
        self.assertEqual(runtime.close_calls, 2)
        self.assertFalse(runtime.active)
        self.assertFalse(card.info()["resources_pending"])


class ConfigureAndIsolationTests(GuardTestCase):
    def test_idle_configuration_uses_real_analyzer_and_preserves_input_dictionary(self):
        card, factory, _ = self.card()
        args = {"confirm_samples": 2, "analysis_hz": 3.0,
                "instance_id": "card-1", "_tool_name": card.PREFIX,
                "_trace_id": "config-1", "camera_info": {},
                "input_bindings": []}
        before = deepcopy(args)
        result = card.dispatch("config", args)
        self.assertEqual(result["state"], "idle")
        self.assertEqual(args, before)
        self.assertEqual(card.info()["config"]["confirm_samples"], 2)
        self.assertEqual(factory.calls, [])
        self.start_card(card)
        runtime = factory.instances[0]
        for _ in range(2):
            runtime.gate(lambda analyzer: analyzer.analyze_image(dark_texture()))
        self.assertEqual(card.info()["quality_state"], "DEGRADED")

    def test_effective_configuration_change_requires_stop_without_restart(self):
        card, factory, _ = self.card()
        self.start_card(card)
        runtime = factory.instances[0]
        for _ in range(3):
            runtime.gate(lambda analyzer: analyzer.analyze_image(dark_texture()))
        before = card.info()
        result = card.dispatch("config", {"confirm_samples": 2})
        self.assertEqual(result["code"], "STOP_REQUIRED")
        self.assertEqual(card.info(), before)
        self.assertEqual(len(factory.calls), 1)
        self.assertEqual(runtime.close_calls, 0)

    def test_empty_and_same_configuration_are_noops_while_running(self):
        card, factory, _ = self.card()
        self.start_card(card)
        before = card.info()
        for args in ({}, {"confirm_samples": 3}, {"analysis_hz": 2.0}):
            with self.subTest(args=args):
                result = card.dispatch("config", args)
                self.assertEqual(result["state"], "running")
                self.assertEqual(card.info(), before)
        self.assertEqual(len(factory.calls), 1)

    def test_invalid_configuration_is_atomic_while_idle_or_running(self):
        invalid = [{"analysis_hz": float("nan")},
                   {"blur_variance_threshold": float("inf")},
                   {"confirm_samples": True}, {"recover_samples": 0},
                   {"unknown": 3}, {"output_topic": "/forbidden"},
                   {"confirm_samples": 2, "analysis_hz": -1}]
        for running in (False, True):
            for args in invalid:
                with self.subTest(running=running, args=args):
                    card, factory, _ = self.card()
                    if running:
                        self.start_card(card)
                        runtime = factory.instances[0]
                        runtime.gate(lambda analyzer: analyzer.analyze_image(dark_texture()))
                    before = card.info()
                    result = card.dispatch("config", args)
                    self.assertTrue(result.get("error") or result.get("code"))
                    self.assertEqual(card.info(), before)
                    self.assertEqual(len(factory.calls), int(running))
                    if running:
                        self.assertEqual(factory.instances[0].close_calls, 0)

    def test_configuration_after_stop_resets_real_quality_and_future_thresholds(self):
        card, factory, _ = self.card()
        self.start_card(card)
        for _ in range(3):
            factory.instances[0].gate(
                lambda analyzer: analyzer.analyze_image(dark_texture()))
        self.assertEqual(card.info()["quality_state"], "DEGRADED")
        stop_card(card)
        result = card.dispatch("config", {"confirm_samples": 2})
        self.assertEqual(result["state"], "idle")
        self.assertEqual(card.info()["quality"],
                         ImageQualityAnalyzer({"confirm_samples": 2}).snapshot())
        self.start_card(card)
        for _ in range(2):
            factory.instances[1].gate(
                lambda analyzer: analyzer.analyze_image(dark_texture()))
        self.assertEqual(card.info()["quality_state"], "DEGRADED")

    def test_info_returns_independent_config_quality_and_port_snapshots(self):
        card, factory, _ = self.card()
        self.start_card(card)
        for _ in range(3):
            factory.instances[0].gate(
                lambda analyzer: analyzer.analyze_image(dark_texture()))
        expected = card.info()
        edited = card.info()
        edited["config"]["confirm_samples"] = 999
        edited["quality"]["metrics"]["mean_luminance"] = -1
        edited["quality"]["reasons"].clear()
        edited["topic_in"][0]["topic"] = "/injected"
        self.assertEqual(card.info(), expected)


class StrictRunIdentityTests(GuardTestCase):
    def run_id(self, epoch=EPOCH):
        return epoch + "." + str(uuid4())

    def test_capability_is_top_level_and_ticket_is_not_algorithm_configuration(self):
        card, _, _ = self.card()
        tool = card.get_tool()
        self.assertEqual(tool["x-lifecycle"], {
            "schema": "motus.lifecycle/1", "id_parameter": "run_id",
            "required_for": ["start", "stop"]})
        self.assertNotIn("run_id", tool["configSchema"]["properties"])
        self.assertIn("run_id", tool["inputSchema"]["properties"])
        self.assertNotIn("shutdown", tool["inputSchema"]["properties"]["action"]["enum"])

    def test_unbound_ticket_is_rejected_without_resources_or_generation_change(self):
        factory = RuntimeFactory()
        card = ImageQualityGuardPlugin({}, "robot", object(), runtime_factory=factory)
        self.addCleanup(card.shutdown)
        before = card._generation
        result = card.dispatch("start", {"input_topic": INPUT, "run_id": self.run_id()})
        self.assertEqual(result["code"], "ADMISSION_REQUIRED")
        self.assertEqual(card._generation, before)
        self.assertEqual(factory.calls, [])

    def test_missing_and_invalid_ticket_cannot_stop_or_reset_active_run(self):
        card, factory, _ = self.card()
        run_id = self.run_id()
        card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})
        factory.instances[0].gate(lambda analyzer: analyzer.analyze_image(dark_texture()))
        before = card.info()
        generation = card._generation
        for bad, code in ((None, "MISSING_RUN_ID"), ("", "MISSING_RUN_ID"),
                          (True, "INVALID_RUN_ID"), ("not-a-ticket", "INVALID_RUN_ID"),
                          (run_id.upper(), "INVALID_RUN_ID")):
            with self.subTest(ticket=bad):
                result = card.dispatch("stop", {"run_id": bad})
                self.assertEqual(result["code"], code)
                self.assertEqual(card.info(), before)
                self.assertEqual(card._generation, generation)
        self.assertEqual(factory.instances[0].close_calls, 0)

    def test_same_ticket_start_is_idempotent_and_different_ticket_is_busy(self):
        card, factory, _ = self.card()
        run_id = self.run_id()
        args = {"input_topic": INPUT, "instance_id": "same-card", "run_id": run_id}
        self.assertEqual(card.dispatch("start", args)["state"], "running")
        generation = card._generation
        self.assertEqual(card.dispatch("start", args)["state"], "running")
        self.assertEqual(card._generation, generation)
        self.assertEqual(card.dispatch("start", {**args, "run_id": self.run_id()})["code"], "BUSY")
        self.assertEqual(len(factory.calls), 1)
        self.assertEqual(factory.instances[0].run_id, run_id)
        self.assertEqual(factory.instances[0].analysis_hz, 2.0)

    def test_same_ticket_binding_is_immutable(self):
        card, factory, _ = self.card()
        run_id = self.run_id()
        card.dispatch("start", {"input_topic": INPUT, "instance_id": "card", "run_id": run_id})
        before = card.info()
        for change in ({"input_topic": "/other/camera"}, {"instance_id": "other"}):
            result = card.dispatch("start", {"input_topic": INPUT, "instance_id": "card",
                                             "run_id": run_id, **change})
            self.assertEqual(result["code"], "INVALID_ARGUMENT")
            self.assertEqual(card.info(), before)
        self.assertEqual(len(factory.calls), 1)

    def test_delayed_stop_before_plugin_entry_cannot_close_same_topic_same_card_restart(self):
        card, factory, _ = self.card()
        old_id, new_id = self.run_id(), self.run_id()
        binding = {"input_topic": INPUT, "instance_id": "same-card"}
        card.dispatch("start", {**binding, "run_id": old_id})
        delayed = threading.Event()
        release = threading.Event()
        results = []

        def old_stop():
            delayed.set()
            if release.wait(3):
                results.append(card.dispatch("stop", {**binding, "run_id": old_id}))

        thread = threading.Thread(target=old_stop)
        thread.start()
        try:
            self.assertTrue(delayed.wait(1))
            card.dispatch("stop", {**binding, "run_id": old_id})
            self.assertEqual(card.dispatch("start", {**binding, "run_id": new_id})["state"], "running")
        finally:
            release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(results[0]["code"], "STALE_RUN")
        self.assertEqual(card.info()["run_id"], new_id)
        self.assertTrue(factory.instances[1].active)
        self.assertEqual(factory.instances[1].close_calls, 0)

    def test_old_ticket_stop_cannot_close_restart_without_instance_id(self):
        card, factory, _ = self.card()
        old_id, new_id = self.run_id(), self.run_id()
        card.dispatch("start", {"input_topic": INPUT, "run_id": old_id})
        card.stop(old_id)
        card.dispatch("start", {"input_topic": INPUT, "run_id": new_id})
        self.assertEqual(card.stop(old_id)["code"], "STALE_RUN")
        self.assertEqual(card.info()["state"], "running")
        self.assertEqual(factory.instances[1].close_calls, 0)

    def test_stop_before_start_tombstone_rejects_delayed_start(self):
        card, factory, _ = self.card()
        run_id = self.run_id()
        self.assertEqual(card.stop(run_id)["code"], "ALREADY_STOPPED")
        self.assertEqual(card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})["code"], "STALE_RUN")
        self.assertEqual(factory.calls, [])

    def test_stop_before_start_of_other_run_remembers_cancellation_without_closing_owner(self):
        card, factory, _ = self.card()
        owner, cancelled = self.run_id(), self.run_id()
        card.dispatch("start", {"input_topic": INPUT, "run_id": owner})
        self.assertEqual(card.stop(cancelled)["code"], "RUN_ID_MISMATCH")
        self.assertEqual(factory.instances[0].close_calls, 0)
        card.stop(owner)
        self.assertEqual(card.dispatch("start", {"input_topic": INPUT, "run_id": cancelled})["code"], "STALE_RUN")

    def test_terminal_ticket_never_restarts_and_repeated_stop_is_safe(self):
        card, factory, _ = self.card()
        run_id = self.run_id()
        card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})
        self.assertEqual(card.stop(run_id)["state"], "idle")
        for _ in range(3):
            self.assertEqual(card.stop(run_id)["code"], "ALREADY_STOPPED")
            self.assertEqual(card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})["code"], "STALE_RUN")
        self.assertEqual(factory.instances[0].close_calls, 1)

    def test_cleanup_failure_keeps_exact_ticket_owner_and_allows_same_ticket_retry(self):
        runtime = FakeRuntime()
        runtime.close_failures = 1
        card, factory, _ = self.card(factory=RuntimeFactory(lambda: runtime))
        run_id = self.run_id()
        card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})
        self.assertEqual(card.stop(run_id)["code"], "CLEANUP_FAILED")
        self.assertEqual(card.info()["run_id"], run_id)
        self.assertTrue(card.info()["resources_pending"])
        self.assertEqual(card.stop(self.run_id())["code"], "RUN_ID_MISMATCH")
        self.assertEqual(runtime.close_calls, 1)
        self.assertEqual(card.stop(run_id)["state"], "idle")
        self.assertEqual(runtime.close_calls, 2)
        self.assertFalse(runtime.active)

    def test_history_capacity_rejects_new_runs_but_existing_stop_still_works(self):
        factory = RuntimeFactory()
        card = ImageQualityGuardPlugin({}, "robot", object(), runtime_factory=factory,
                                       history_capacity=1)
        self.addCleanup(card.shutdown)
        bind_card(card)
        run_id = self.run_id()
        card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})
        self.assertEqual(card.stop(self.run_id())["code"], "RUN_HISTORY_FULL")
        self.assertEqual(card.stop(run_id)["state"], "idle")
        self.assertEqual(card.stop(run_id)["code"], "ALREADY_STOPPED")
        self.assertEqual(card.dispatch("start", {"input_topic": INPUT, "run_id": self.run_id()})["code"], "RUN_HISTORY_FULL")
        self.assertEqual(card.info()["history_count"], 1)
        self.assertEqual(len(factory.calls), 1)

    def test_epoch_bind_challenge_cas_and_exact_retry(self):
        card, _, _ = self.card()
        status = card.lifecycle_status()
        proposed = str(uuid4())
        args = {"driver_boot_id": status["driver_boot_id"], "expected_epoch": EPOCH,
                "proposed_epoch": proposed}
        self.assertEqual(card.bind_epoch(**{**args, "driver_boot_id": str(uuid4())})["code"], "DRIVER_BOOT_MISMATCH")
        self.assertEqual(card.bind_epoch(**{**args, "expected_epoch": None})["code"], "EPOCH_MISMATCH")
        self.assertEqual(card.bind_epoch(**args)["admission_epoch"], proposed)
        generation = card._generation
        self.assertEqual(card.bind_epoch(**args)["admission_epoch"], proposed)
        self.assertEqual(card._generation, generation)
        self.assertEqual(card.stop(self.run_id())["code"], "STALE_RUN")

    def test_epoch_rotation_requires_confirmed_idle_and_invalidates_old_tombstones(self):
        card, _, _ = self.card()
        run_id = self.run_id()
        card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})
        args = {"driver_boot_id": card.info()["driver_boot_id"],
                "expected_epoch": EPOCH, "proposed_epoch": str(uuid4())}
        self.assertEqual(card.bind_epoch(**args)["code"], "BUSY")
        card.stop(run_id)
        self.assertEqual(card.bind_epoch(**args)["history_count"], 0)
        self.assertEqual(card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})["code"], "STALE_RUN")

    def test_shutdown_closes_admission_before_cleanup_and_retries_failed_owner(self):
        runtime = BlockingCloseRuntime()
        runtime.close_failures = 1
        card, _, _ = self.card(factory=RuntimeFactory(lambda: runtime))
        run_id = self.run_id()
        card.dispatch("start", {"input_topic": INPUT, "run_id": run_id})
        results = []
        thread = threading.Thread(target=lambda: results.append(card.shutdown()))
        thread.start()
        try:
            self.assertTrue(runtime.close_entered.wait(1))
            self.assertEqual(card.dispatch("start", {"input_topic": INPUT, "run_id": self.run_id()})["code"], "SHUTDOWN")
            self.assertFalse(runtime.gate(lambda analyzer: analyzer.analyze_image(dark_texture())))
        finally:
            runtime.close_release.set()
            thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(results[0]["code"], "CLEANUP_FAILED")
        self.assertEqual(card.info()["state"], "error")
        self.assertEqual(card.info()["error"], results[0]["error"])
        self.assertTrue(card.info()["resources_pending"])
        self.assertEqual(card.info()["run_id"], run_id)
        self.assertEqual(card.shutdown()["state"], "idle")
        self.assertEqual(card.shutdown()["state"], "idle")
        self.assertEqual(runtime.close_calls, 2)
        self.assertIsNone(card.dispatch("shutdown"))


if __name__ == "__main__":
    unittest.main()
