"""Offline behavioral tests. No ROS initialization or physical robot access."""
from __future__ import annotations
import ast
from collections import Counter
import concurrent.futures
import importlib.util
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from telemetry_recorder_core import Recorder
from telemetry_recorder_store import ClipStore
from telemetry_recorder_report import render_report
from telemetry_recorder import TelemetryRecorderPlugin, create_ros_subscriber


class Clock:
    def __init__(self):
        self.value = 100.0
    def __call__(self):
        return self.value


def sources():
    return {"imu": {"topic": "/test/state/imu", "stale_s": .5},
            "battery": {"topic": "/test/state/battery", "stale_s": 3},
            "motion_events": {"topic": "/test/safety/motion_events", "event_stream": True}}


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
    def recorder(self, **kwargs):
        obj = Recorder(sources(), clock=self.clock, wall=lambda: 1700000000, **kwargs)
        obj.start()
        return obj
    def test_pre_and_post_window_select_actual_received_samples(self):
        r = self.recorder()
        for t in (90, 95, 99, 100):
            self.clock.value = t
            r.ingest_json("imu", json.dumps({"t": t}))
        r.capture("marker", 5, 2)
        for t in (101, 102, 103):
            self.clock.value = t
            r.ingest_json("imu", json.dumps({"t": t}))
        doc, = r.drain_ready()
        self.assertEqual([95, 99, 100, 101, 102], [s["payload"]["t"] for s in doc["samples"]])
        self.assertTrue(doc["post_window_elapsed"])
        self.assertEqual(-5, doc["samples"][0]["receive_offset_s"])
    def test_missing_and_stale_sources_are_not_healthy(self):
        r = self.recorder()
        self.assertEqual("never_received", r.status()["sources"]["imu"]["state"])
        self.assertEqual("no_events_observed", r.status()["sources"]["motion_events"]["state"])
        r.ingest_json("imu", '{}')
        self.clock.value += 2
        self.assertEqual("stale", r.status()["sources"]["imu"]["state"])
        r.capture(post_s=0)
        doc, = r.drain_ready()
        self.assertEqual("stale", doc["quality"]["imu"]["trigger_state"])
        self.assertEqual(0, doc["quality"]["battery"]["sample_count"])
    def test_invalid_messages_and_nonfinite_numbers_rejected(self):
        r = self.recorder()
        for message in ('no json', '[]', 'null', '{"x":NaN}', '{"x":Infinity}', '{"x":1e999}', '{"x":"\ud800"}'):
            self.assertFalse(r.ingest_json("imu", message))
        self.assertTrue(r.ingest_json("imu", '{"x":"NaN is a label"}'))
        self.assertFalse(r.ingest_json("unknown", '{}'))
        self.assertFalse(r.ingest_json("imu", 'x' * 65537))
    def test_ring_bounded_by_time_count_and_bytes(self):
        r = self.recorder(max_buffer_bytes=1024, max_samples=3)
        for i in range(30):
            r.ingest_json("imu", json.dumps({"x": "z" * 150}))
        status = r.status()
        self.assertLessEqual(status["buffer_samples"], 3)
        self.assertLessEqual(status["buffer_payload_budget_bytes"], 1024)
        self.clock.value += 31
        self.assertEqual(0, r.status()["buffer_samples"])
    def test_clip_truncation_is_reported(self):
        r = self.recorder(max_clip_bytes=1024)
        r.capture(post_s=1)
        for i in range(10):
            r.ingest_json("imu", json.dumps({"x": "z" * 500}))
        self.clock.value += 1
        doc, = r.drain_ready()
        self.assertTrue(doc["truncated"])
        self.assertLess(len(doc["samples"]), 10)
    def test_auto_event_debounce_and_pending_limit(self):
        r = self.recorder(auto_events=["safety_stop"], max_pending=1)
        self.assertTrue(r.ingest_json("motion_events", '{"type":[]}'))
        r.ingest_json("motion_events", '{"type":"safety_stop"}')
        r.ingest_json("motion_events", '{"type":"safety_stop"}')
        self.assertEqual(1, len(r.status()["pending"]))
        self.assertEqual(1, r.status()["auto_suppressed"])
        with self.assertRaises(ValueError):
            r.capture()
        self.clock.value += 6
        self.assertEqual(1, len(r.drain_ready()))
        r.ingest_json("motion_events", '{"type":"safety_stop"}')
        self.assertEqual(0, len(r.status()["pending"]))
        self.clock.value += 5
        r.ingest_json("motion_events", '{"type":"safety_stop"}')
        self.assertEqual(1, len(r.status()["pending"]))
    def test_stopping_marks_unfinished_window_and_rejects_more_data(self):
        r = self.recorder()
        r.capture(post_s=5)
        self.clock.value += 1
        doc, = r.stop()
        self.assertTrue(doc["interrupted"])
        self.assertFalse(r.ingest_json("imu", '{}'))
        self.assertEqual([], r.drain_ready())
        r.start()
        self.assertEqual("never_received", r.status()["sources"]["imu"]["state"])
    def test_original_source_timestamp_preserved_without_using_as_clock(self):
        r = self.recorder()
        r.ingest_json("imu", '{"timestamp":-9999,"rpy":[1,2,3]}')
        r.capture(post_s=0)
        doc, = r.drain_ready()
        self.assertEqual(-9999, doc["samples"][0]["payload"]["timestamp"])
        self.assertEqual(0, doc["samples"][0]["receive_offset_s"])
    def test_observed_gap_and_concurrent_callbacks(self):
        r = self.recorder()
        r.ingest_json("imu", '{}')
        self.clock.value += 2
        r.ingest_json("imu", '{}')
        with concurrent.futures.ThreadPoolExecutor(4) as pool:
            self.assertTrue(all(pool.map(lambda _: r.ingest_json("battery", '{}'), range(200))))
        r.capture(post_s=0)
        doc, = r.drain_ready()
        self.assertEqual(2, doc["quality"]["imu"]["max_observed_gap_s"])
        self.assertEqual(200, doc["quality"]["battery"]["sample_count"])
    def test_numeric_limits_validate_without_coercing_bools(self):
        for params in ({"buffer_s": float('nan')}, {"max_pending": True}, {"pre_s": 31}, {"max_samples": 0}):
            with self.assertRaises(ValueError):
                self.recorder(**params)


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=os.environ.get('G1_RECORDER_TEST_TMP'))
        self.addCleanup(self.tmp.cleanup)
        self.store = ClipStore(self.tmp.name)
        self.store.prepare()
        r = Recorder(sources())
        r.start()
        r.ingest_json("imu", '{"rpy":[1,2,3]}')
        r.capture('<script>alert("x")</script>', post_s=0)
        self.doc, = r.drain_ready()
    def test_atomic_pair_metadata_list_and_html_escaping(self):
        result = self.store.write(self.doc)
        self.assertEqual("saved", result["status"])
        report = Path(result["report_path"]).read_text(encoding="utf-8")
        self.assertNotIn('<script>alert', report)
        self.assertIn('&lt;script&gt;', report)
        self.assertNotIn('src="http', report)
        self.assertEqual(self.doc["clip_id"], self.store.list()[0]["clip_id"])
        self.assertNotIn("samples", self.store.get(self.doc["clip_id"]))
    def test_quota_failure_leaves_no_success_or_temp_directory(self):
        tiny = ClipStore(self.tmp.name, max_disk_bytes=1024)
        with self.assertRaises(OSError):
            tiny.write(self.doc)
        self.assertEqual([], list(Path(self.tmp.name).iterdir()))
    def test_failed_write_removes_only_own_temporary_files(self):
        with patch('telemetry_recorder_store.os.fsync', side_effect=OSError('disk failed')):
            with self.assertRaises(OSError):
                self.store.write(self.doc)
        self.assertEqual([], list(Path(self.tmp.name).iterdir()))
    def test_clip_limit_and_path_traversal_rejected(self):
        self.store.max_clips = 1
        self.store.write(self.doc)
        with self.assertRaises(OSError):
            self.store.write({**self.doc, "clip_id": "a" * 32})
        for key in ("../../secret", "x", None):
            with self.assertRaises(ValueError):
                self.store.get(key)
    def test_invalid_numeric_shapes_do_not_break_report(self):
        doc = {**self.doc, "samples": [
            {"source": "imu", "receive_offset_s": 0, "payload": {"rpy": [None, {}, "x"]}},
            {"source": "joints", "receive_offset_s": 0, "payload": {"joints": [None, {"temp": [None]}]}}
        ]}
        self.assertIn("No usable numeric samples", render_report(doc))
    def test_report_does_not_draw_continuous_line_across_receive_outage(self):
        doc = {**self.doc, 'samples': [
            {'source': 'imu', 'receive_offset_s': -2, 'payload': {'rpy': [0]}},
            {'source': 'imu', 'receive_offset_s': 0, 'payload': {'rpy': [1]}}
        ]}
        self.assertRegex(render_report(doc), r'd="M[^"\n]+ M')


class PluginTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=os.environ.get('G1_RECORDER_TEST_TMP'))
        self.addCleanup(self.tmp.cleanup)
        self.opens, self.closes = [], []
        def factory(sources, callback, executor):
            self.opens.append((sources, callback))
            return lambda: self.closes.append(True)
        self.plugin = TelemetryRecorderPlugin({"output_dir": self.tmp.name}, 'test', None,
                                               subscriber_factory=factory)
        self.addCleanup(self.plugin.stop)
    def test_lifecycle_idempotent_and_source_health_honest(self):
        self.assertEqual("ready", self.plugin.start()["state"])
        self.plugin.start()
        self.assertEqual(1, len(self.opens))
        self.assertEqual("never_received", self.plugin.info()["sources"]["imu"]["state"])
        self.plugin.stop()
        self.plugin.stop()
        self.assertEqual(1, len(self.closes))
        self.plugin.start()
        self.assertEqual(2, len(self.opens))
    def test_worker_saves_and_get_clip_exposes_metadata(self):
        signal = threading.Event()
        remember = self.plugin._remember
        def observed(result):
            remember(result)
            if result["status"] == "saved":
                signal.set()
        self.plugin._remember = observed
        self.plugin.start()
        self.opens[0][1]("imu", '{"rpy":[0,0,0]}')
        response = self.plugin.dispatch("capture", {"post_s": 0})
        self.assertEqual("capturing", response["status"])
        self.assertTrue(signal.wait(2), self.plugin.info())
        result = self.plugin.dispatch("get_clip", {"clip_id": response["clip_id"]})
        self.assertEqual("saved", result["status"])
        self.assertTrue(Path(result["json_path"]).is_file())
    def test_storage_error_surfaces_without_saying_saved(self):
        signal = threading.Event()
        remember = self.plugin._remember
        def observed(result):
            remember(result)
            if result["status"] == "failed":
                signal.set()
        self.plugin._remember = observed
        self.plugin.store.max_disk_bytes = 1024
        self.plugin.start()
        key = self.plugin.dispatch("capture", {"post_s": 0})["clip_id"]
        self.assertTrue(signal.wait(2))
        self.assertEqual("failed", self.plugin.dispatch("get_clip", {"clip_id": key})["status"])
        self.assertIn("quota", self.plugin.info()["last_storage_or_worker_error"])
    def test_failed_subscription_start_reports_error(self):
        self.plugin._factory = lambda *args: (_ for _ in ()).throw(RuntimeError('no ROS transport'))
        result = self.plugin.start()
        self.assertEqual("error", result["state"])
        self.assertFalse(self.plugin.core.status()["recording"])
    def test_terminal_result_cannot_be_overwritten_by_late_capture_ack(self):
        self.plugin._remember({"clip_id": 'a'*32, "status": "saved"})
        self.plugin._remember({"clip_id": 'a'*32, "status": "capturing"})
        self.assertEqual("saved", self.plugin.dispatch('get_clip', {'clip_id': 'a'*32})['status'])
    def test_ros_adapter_only_subscribes_to_existing_string_topics(self):
        subscriptions, destroyed, nodes = [], [], []
        class Node:
            def __init__(self, name):
                nodes.append(self)
            def create_subscription(self, message, topic, callback, qos):
                subscriptions.append((message, topic, callback, qos))
            def destroy_node(self):
                destroyed.append(True)
        class Qos:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)
        policy = types.SimpleNamespace(BEST_EFFORT='best_effort', KEEP_LAST='last', VOLATILE='volatile')
        modules = {
            'rclpy': types.SimpleNamespace(ok=lambda: True),
            'rclpy.node': types.SimpleNamespace(Node=Node),
            'rclpy.qos': types.SimpleNamespace(QoSProfile=Qos, ReliabilityPolicy=policy, HistoryPolicy=policy, DurabilityPolicy=policy),
            'std_msgs.msg': types.SimpleNamespace(String=str),
        }
        added, removed, received = [], [], []
        executor = types.SimpleNamespace(add_node=added.append, remove_node=removed.append)
        with patch.dict(sys.modules, modules):
            close = create_ros_subscriber(sources(), lambda key, text: received.append((key, text)), executor)
        for _, topic, callback, qos in subscriptions:
            callback(types.SimpleNamespace(data='{}'))
            self.assertEqual('best_effort', qos.reliability)
        self.assertEqual(set(sources()), {key for key, _ in received})
        self.assertEqual({s['topic'] for s in sources().values()}, {s[1] for s in subscriptions})
        close()
        self.assertEqual(added, removed)
        self.assertEqual([True], destroyed)
    def test_config_rejected_while_running_and_validated_when_idle(self):
        self.plugin.start()
        self.assertIn("error", self.plugin.dispatch("config", {"buffer_s": 20}))
        self.plugin.stop()
        self.assertIn("error", self.plugin.dispatch("config", {"imu_topic": "bad topic"}))
        self.assertEqual(30, self.plugin.core.buffer_s)
        self.assertTrue(self.plugin.dispatch("config", {"buffer_s": 20})["configured"])
    def test_bundle_real_dispatch_and_schema_contract(self):
        tree = ast.parse((ROOT / 'main.py').read_text(encoding="utf-8"))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'G1DeviceBundle')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                   and n.name in ('get_all_tools', 'dispatch')]
        tiny = ast.Module(body=[ast.ClassDef(name='Bundle', bases=[], keywords=[], body=methods, decorator_list=[])], type_ignores=[])
        class Lifecycle:
            LIFECYCLE_ACTIONS = ()
        ns = {"_lifecycle": Lifecycle}
        exec(compile(ast.fix_missing_locations(tiny), str(ROOT/'main.py'), 'exec'), ns)
        bundle = ns['Bundle']()
        bundle._plugins = [self.plugin]
        self.assertEqual('telemetry_recorder', bundle.get_all_tools()[0]['name'])
        self.assertIn('error', bundle.dispatch('telemetry_recorder', {"action": "capture"}))
        self.assertEqual('ready', bundle.dispatch('telemetry_recorder', {"action": "start"})['state'])
        result = bundle.dispatch('telemetry_recorder', {"action": "status"})
        self.assertEqual('never_received', result['sources']['imu']['state'])
    def test_full_bundle_constructor_registers_only_when_enabled(self):
        tree = ast.parse((ROOT / 'main.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'G1DeviceBundle')
        future = ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0)
        module = ast.fix_missing_locations(ast.Module(body=[future, cls], type_ignores=[]))
        ns = {'print': lambda *args: None}
        exec(compile(module, str(ROOT/'main.py'), 'exec'), ns)
        config = {'plugins': {'smart_motion': {'enabled': False}}}
        constructor = ns['G1DeviceBundle']
        empty = constructor(config, 'test', None, None, None, None, None)
        self.assertEqual([], empty.get_all_tools())
        config['plugins']['telemetry_recorder'] = {'enabled': True, 'output_dir': self.tmp.name}
        enabled = constructor(config, 'test', None, None, None, None, None)
        self.assertEqual(['telemetry_recorder'], [t['name'] for t in enabled.get_all_tools()])


if __name__ == '__main__':
    unittest.main()
