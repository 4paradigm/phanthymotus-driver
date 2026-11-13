"""Read-only health tests: synthetic Linux files and existing driver objects."""
import io
import json
import os
import subprocess
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path
from unittest import mock

import test_device  # Installs ROS message stubs, never starts the driver.
import device
from system_health import DEFAULTS, SystemHealthPlugin, SystemProbe, observation
from common.vendor_runtime import DriverBundle, make_handler


class ProbeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.probe = SystemProbe(self.root)
        self.config = {**DEFAULTS, "disk_path": "/"}
        self.write("/proc/loadavg", "0.2 0.3 0.4 1/100 1")
        self.write("/proc/stat", "cpu 1\ncpu0 1\ncpu1 1\n")
        self.write("/proc/meminfo", "MemTotal: 1000 kB\nMemAvailable: 800 kB\n")
        self.write("/proc/self/cgroup", "0::/work")
        self.write("/proc/self/mountinfo", "1 0 0:1 / /sys/fs/cgroup rw - cgroup2 cgroup rw")
        self.write("/sys/fs/cgroup/work/memory.current", "200")
        self.write("/sys/fs/cgroup/work/memory.max", "1000")
        self.write("/sys/fs/cgroup/memory.current", "200")
        self.write("/sys/fs/cgroup/memory.max", "max")

    def write(self, path, value):
        target = self.root / path.lstrip("/")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(value)

    def collect(self):
        stat = types.SimpleNamespace(f_blocks=1000, f_frsize=1024, f_bavail=800)
        with mock.patch("system_health.os.statvfs", return_value=stat):
            return self.probe.collect(self.config)

    def test_normal_metrics_and_missing_thermal(self):
        result = self.collect()
        for name in ("cpu_load", "memory", "memory_limit", "disk"):
            self.assertEqual(result[name]["status"], "ok")
            self.assertIsInstance(result[name]["sampled_at"], float)
        self.assertEqual(result["temperature"]["status"], "unreadable")

    def test_unreadable_and_invalid_metrics_do_not_become_ok(self):
        for path in ("/proc/loadavg", "/proc/meminfo", "/sys/fs/cgroup/work/memory.max"):
            self.write(path, "invalid")
        with mock.patch("system_health.os.statvfs", side_effect=PermissionError("denied")):
            result = self.probe.collect(self.config)
        for name in ("cpu_load", "memory", "memory_limit", "disk"):
            self.assertEqual(result[name]["status"], "unreadable")
            self.assertTrue(result[name]["reasons"])

    def test_cpu_memory_disk_and_cgroup_thresholds(self):
        self.write("/proc/loadavg", "2 2 2")
        self.write("/proc/meminfo", "MemTotal: 1000 kB\nMemAvailable: 100 kB\n")
        self.write("/sys/fs/cgroup/work/memory.current", "900")
        stat = types.SimpleNamespace(f_blocks=1000, f_frsize=1, f_bavail=100)
        with mock.patch("system_health.os.statvfs", return_value=stat):
            result = self.probe.collect(self.config)
        for name in ("cpu_load", "memory", "memory_limit", "disk"):
            self.assertEqual(result[name]["status"], "warning")

    def test_ancestor_limit_uses_ancestor_usage(self):
        self.write("/sys/fs/cgroup/memory.max", "2000")
        self.write("/sys/fs/cgroup/memory.current", "1900")
        result = self.collect()["memory_limit"]
        self.assertEqual(result["status"], "warning")
        self.assertEqual(result["value"]["used_percent"], 95)

    def test_hidden_cgroup_ancestors_are_unknown(self):
        self.write("/proc/self/cgroup", "0::/work")
        self.write("/proc/self/mountinfo", "1 0 0:1 /work /sys/fs/cgroup rw - cgroup2 cgroup rw")
        result = self.collect()["memory_limit"]
        self.assertEqual(result["status"], "unknown")
        self.assertFalse(result["value"]["hierarchy_complete"])

    def test_namespaced_cgroup_root_is_read_without_claiming_host_limit(self):
        self.write("/proc/self/cgroup", "0::/")
        self.write("/proc/self/mountinfo", "1 0 0:1 /docker/id /sys/fs/cgroup rw - cgroup2 cgroup rw")
        item = self.collect()["memory_limit"]
        self.assertEqual(item["status"], "unknown")
        self.assertEqual(item["value"]["usage_bytes"], 200)

    def test_v1_memory_membership(self):
        self.write("/proc/self/cgroup", "4:cpu,memory:/work")
        self.write("/proc/self/mountinfo", "1 0 0:1 / /sys/fs/cgroup rw - cgroup cgroup rw,memory")
        for directory in ("", "/work"):
            self.write("/sys/fs/cgroup" + directory + "/memory.usage_in_bytes", "200")
            self.write("/sys/fs/cgroup" + directory + "/memory.limit_in_bytes", "1000")
        item = self.collect()["memory_limit"]
        self.assertEqual(item["value"]["version"], 1)
        self.assertEqual(item["status"], "ok")
        self.assertTrue(item["value"]["hierarchy_complete"])
        with mock.patch.dict("os.environ", {"U1_PRO_HEALTH_CONTAINER": "1"}):
            container_item = self.collect()["memory_limit"]
        self.assertEqual(container_item["status"], "unknown")
        self.assertFalse(container_item["value"]["hierarchy_complete"])

    def test_unlimited_is_explicit(self):
        self.write("/sys/fs/cgroup/work/memory.max", "max")
        self.assertIsNone(self.collect()["memory_limit"]["value"]["limit_bytes"])

    def test_each_temperature_retains_label_and_unknown_identity(self):
        self.write("/sys/class/thermal/thermal_zone0/temp", "50000")
        self.write("/sys/class/thermal/thermal_zone0/type", "board")
        self.write("/sys/class/thermal/thermal_zone1/temp", "bad")
        result = self.collect()
        self.assertEqual(result["thermal_zone0"]["status"], "unknown")
        self.assertEqual(result["thermal_zone0"]["value"]["celsius"], 50)
        self.assertEqual(result["thermal_zone1"]["status"], "unreadable")
        self.config["temperature_warn_celsius"] = 45
        self.assertEqual(self.collect()["thermal_zone0"]["status"], "warning")

    def test_hwmon_unlabeled_temperature_is_retained(self):
        self.write("/sys/class/hwmon/hwmon0/temp1_input", "42000")
        item = self.collect()["hwmon0_temp1_input"]
        self.assertEqual(item["status"], "unknown")
        self.assertEqual(item["value"]["label"], "unlabeled")
        self.assertEqual(item["value"]["celsius"], 42)

    def test_container_source_attribution(self):
        with mock.patch.dict("os.environ", {"U1_PRO_HEALTH_CONTAINER": "1"}):
            result = self.collect()
        self.assertEqual(result["memory"]["scope"], "container_visible")
        self.assertEqual(result["memory_limit"]["scope"], "process_cgroup")
        self.assertIn("not host disk", result["disk"]["value"]["note"])

    def make_blocked_load(self):
        path = self.root / "proc/loadavg"
        path.unlink()
        os.mkfifo(path)
        return path

    def test_isolated_metric_reader_returns_real_fixture_observations(self):
        config = {**self.config, "probe_timeout_seconds": 2.0}
        result = self.probe.collect_bounded(config, threading.Event())
        self.assertEqual(result["cpu_load"]["status"], "ok")
        self.assertEqual(result["memory"]["status"], "ok")
        self.assertFalse(self.probe.has_pending_read)
        self.assertIsNone(self.probe._reader)

    def test_blocked_filesystem_read_times_out_and_recovers(self):
        path = self.make_blocked_load()
        config = {**self.config, "probe_timeout_seconds": 0.15}
        began = time.monotonic()
        with self.assertRaises(TimeoutError):
            self.probe.collect_bounded(config, threading.Event())
        self.assertLess(time.monotonic() - began, 1.0)
        self.assertFalse(self.probe.has_pending_read)
        self.assertIsNone(self.probe._reader)
        path.unlink()
        path.write_text("0.2 0.3 0.4")
        config["probe_timeout_seconds"] = 2.0
        result = self.probe.collect_bounded(config, threading.Event())
        self.assertEqual(result["cpu_load"]["status"], "ok")

    def test_stop_cancels_and_reaps_blocked_filesystem_reader(self):
        self.make_blocked_load()
        stop_event = threading.Event()
        errors = []
        def read():
            try:
                self.probe.collect_bounded(
                    {**self.config, "probe_timeout_seconds": 5.0}, stop_event)
            except InterruptedError as exc:
                errors.append(exc)
        worker = threading.Thread(target=read)
        worker.start()
        deadline = time.monotonic() + 1
        while not self.probe.has_pending_read and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(self.probe.has_pending_read)
        began = time.monotonic()
        stop_event.set()
        worker.join(1.0)
        self.assertLess(time.monotonic() - began, 1.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertFalse(self.probe.has_pending_read)

    def test_uninterruptible_reader_is_retained_and_prevents_overlap(self):
        process = mock.Mock()
        process.poll.return_value = None
        process.communicate.side_effect = subprocess.TimeoutExpired("fixture", 0.01)
        process.wait.side_effect = subprocess.TimeoutExpired("fixture", 0.2)
        with mock.patch("system_health.subprocess.Popen", return_value=process) as launch:
            with self.assertRaises(TimeoutError):
                self.probe.collect_bounded(
                    {**self.config, "probe_timeout_seconds": 0.01}, threading.Event())
            self.assertTrue(self.probe.has_pending_read)
            with self.assertRaisesRegex(OSError, "previous metric reader"):
                self.probe.collect_bounded(self.config, threading.Event())
            self.assertEqual(launch.call_count, 1)
        process.kill.assert_called_once()
        for stream in (process.stdin, process.stdout, process.stderr):
            stream.close.assert_called_once()
        card = SystemHealthPlugin([], mock.Mock(), probe=self.probe)
        self.assertEqual(card.start(), {"state": "stopping"})
        self.assertEqual(card.stop(), {"state": "stopping"})
        process.poll.return_value = 0
        self.assertFalse(self.probe.has_pending_read)

    def test_invalid_reader_response_is_reaped(self):
        process = mock.Mock(returncode=0)
        process.poll.return_value = 0
        process.communicate.return_value = ("not JSON", "")
        with mock.patch("system_health.subprocess.Popen", return_value=process):
            with self.assertRaises(ValueError):
                self.probe.collect_bounded(self.config, threading.Event())
        process.wait.assert_called_once_with(timeout=0.2)
        self.assertIsNone(self.probe._reader)

    def test_timed_out_compute_does_not_hide_existing_driver_state(self):
        self.make_blocked_load()
        audio = types.SimpleNamespace(
            PREFIX="tts", health_snapshot=lambda config: observation(
                "ok", "fixture:queue", "driver", {}))
        card = SystemHealthPlugin([], audio, {"probe_timeout_seconds": 0.15}, probe=self.probe)
        self.addCleanup(card.stop)
        card.start()
        deadline = time.monotonic() + 1.5
        while card.dispatch("info", {})["samples_collected"] < 1 and time.monotonic() < deadline:
            time.sleep(0.005)
        result = card.dispatch("diagnose", {})
        self.assertEqual(result["checks"]["compute"]["status"], "unreadable")
        self.assertEqual(result["checks"]["tts"]["status"], "ok")
        self.assertFalse(result["complete"])
        self.assertEqual(card.stop(), {"state": "idle"})
        self.assertIsNone(card._worker)
        self.assertFalse(self.probe.has_pending_read)


class CardTests(unittest.TestCase):
    def setUp(self):
        self.nodes = mock.Mock(namespace="test")
        self.nodes.namespace = "test"
        self.nodes.motion_catalog.return_value = {}
        self.cameras = [device.EyeCameraPlugin(self.nodes, eye) for eye in ("left", "right")]
        self.audio = device.AudioPlugin(self.nodes)
        self.nodes.reset_mock()
        self.probe = mock.Mock()
        self.probe.collect.return_value = {"cpu": observation("ok", "fixture", "test", {}, sampled_at=1)}
        self.card = SystemHealthPlugin(self.cameras, self.audio, probe=self.probe)
        self.addCleanup(self.card.stop)

    def wait_for(self, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail("sampler did not reach expected state")
            time.sleep(0.005)

    def report(self, action="robot_info"):
        self.card.start()
        self.wait_for(lambda: self.card.dispatch("info", {})["samples_collected"] >= 1)
        return self.card.dispatch(action, {})

    def test_queries_and_lifecycle_never_call_robot_or_leak_resources(self):
        before = {t.ident for t in threading.enumerate()}
        self.report()
        for action in ("start", "start", "robot_info", "diagnose", "stop", "stop", "robot_info", "start", "stop"):
            self.assertIsInstance(self.card.dispatch(action, {}), dict)
        self.assertEqual({t.ident for t in threading.enumerate()}, before)
        self.assertEqual(self.nodes.mock_calls, [])
        for camera in self.cameras:
            self.assertFalse(camera.running)
            self.assertIsNone(camera._reader)
            self.assertIsNone(camera._subscription)
            self.assertIsNone(camera._frame_worker)
        self.assertFalse(self.card.running)

    def test_not_started_is_incomplete(self):
        result = self.card.dispatch("robot_info", {})
        self.assertEqual(result["checks"]["monitoring"]["status"], "not_started")
        self.probe.collect.assert_not_called()
        self.assertEqual(result["overall"], "unknown")
        self.assertFalse(result["complete"])
        json.dumps(result, allow_nan=False)

    def test_fresh_stale_and_error_camera_snapshots(self):
        camera = self.cameras[0]
        camera.running = True
        with camera._frame_condition:
            camera._last_frame_monotonic = time.monotonic()
            camera._last_frame_at = time.time()
        self.assertEqual(camera.health_snapshot(DEFAULTS)["status"], "ok")
        with mock.patch("device.time.monotonic", return_value=camera._last_frame_monotonic + 6):
            self.assertEqual(camera.health_snapshot(DEFAULTS)["status"], "stale")
        camera._last_error = "decode failed"
        self.assertEqual(camera.health_snapshot(DEFAULTS)["status"], "error")
        camera._last_error = ""
        self.assertEqual(camera.health_snapshot(DEFAULTS)["value"]["recent_error"]["message"], "decode failed")
        camera.running = False
        camera.running = True
        self.assertEqual(camera.health_snapshot(DEFAULTS)["status"], "stale")

    def test_real_publish_updates_health_timestamp(self):
        camera = self.cameras[0]
        camera._publisher = mock.Mock()
        self.nodes.CompressedImage = test_device.sys.modules["sensor_msgs.msg"].CompressedImage
        with mock.patch.object(device, "_jpeg_from_frame", return_value=b"jpeg"):
            camera._publish_frame(b"frame", {}, 0)
        # Camera start marks running only after the first valid frame arrives.
        camera.running = True
        item = camera.health_snapshot(DEFAULTS)
        self.assertEqual(item["status"], "ok")
        self.assertEqual(item["value"]["frames_published"], 1)
        self.assertIsNotNone(item["value"]["last_frame_at"])

    def test_shared_playback_and_stale_task(self):
        self.audio._active = {"kind": "play_action", "tool_name": "head", "action_id": "id",
                              "vendor_uuid": "vendor", "queued_monotonic": time.monotonic() - 121}
        result = self.report("diagnose")
        item = result["checks"]["tts"]
        self.assertEqual(item["status"], "stale")
        self.assertEqual(item["value"]["active"]["tool_name"], "head")
        self.assertEqual(result["overall"], "warning")
        self.assertIsNotNone(self.audio._active)
        self.assertEqual(self.nodes.mock_calls, [])

    def test_matched_playback_failure_is_observable(self):
        self.audio._active = {"action_id": "id", "vendor_uuid": "vendor", "tool_name": "tts"}
        with mock.patch.object(device, "_acp_notify"):
            self.audio._on_playback_state({"phase": "result", "uuid": "vendor", "state_name": "FAILED"})
        self.assertEqual(self.audio.health_snapshot(DEFAULTS)["status"], "error")

    def test_failed_snapshot_and_unknown_are_not_all_normal(self):
        self.cameras[0].health_snapshot = mock.Mock(side_effect=RuntimeError("closed"))
        result = self.report()
        self.assertEqual(result["checks"]["camera_left"]["status"], "unreadable")
        self.assertNotEqual(result["overall"], "ok")

    def test_normal_driver_snapshots_and_sensor_mcp_contract(self):
        for component in (*self.cameras, self.audio):
            component.health_snapshot = mock.Mock(return_value=observation("ok", "fixture", "test", {}))
        bundle = DriverBundle([self.card])
        definition = bundle.get_all_tools()[0]
        self.assertEqual(definition["type"], "sensor")
        self.assertFalse(definition["multiInstance"])
        self.report()
        self.assertEqual(bundle.dispatch("system_health", {"action": "robot_info"})["overall"], "ok")

    def assert_device_error_is_incomplete(self, failed_prefix):
        for component in (*self.cameras, self.audio):
            status = "error" if component.PREFIX == failed_prefix else "ok"
            component.health_snapshot = mock.Mock(return_value=observation(
                status, "fixture:" + component.PREFIX, "driver", {},
                reasons=["device failed"] if status == "error" else [],
                sampled_at=time.time()))
        result = self.report()
        self.assertEqual(result["overall"], "warning")
        self.assertFalse(result["complete"])
        self.assertEqual(result["checks"][failed_prefix]["status"], "error")
        self.assertEqual(result["problems"], [{
            "check": failed_prefix, "status": "error",
            "reasons": ["device failed"]}])
        self.assertEqual(self.nodes.mock_calls, [])

    def test_camera_error_with_otherwise_normal_checks_is_incomplete(self):
        self.assert_device_error_is_incomplete("camera_left")

    def test_playback_error_with_otherwise_normal_checks_is_incomplete(self):
        self.assert_device_error_is_incomplete("tts")

    def test_stop_during_sampling_cancels_publication_and_prevents_overlap(self):
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def sample(config):
            entered.set()
            if not release.wait(2):
                raise TimeoutError("fixture not released")
            return {}
        self.probe.collect.side_effect = sample
        self.card.start()
        self.assertTrue(entered.wait(1))
        worker = self.card._worker
        result = []
        stopper = threading.Thread(target=lambda: result.append(self.card.stop()))
        stopper.start()
        self.wait_for(lambda: self.card._stop_event.is_set())
        self.assertEqual(self.card.start(), {"state": "stopping"})
        release.set()
        stopper.join(2)
        self.assertFalse(stopper.is_alive())
        self.assertFalse(worker.is_alive())
        self.assertIsNone(self.card._worker)
        self.assertEqual(result, [{"state": "idle"}])
        self.assertEqual(self.card._samples, 0)
        self.assertFalse(self.card.running)

    def test_default_one_second_sampling_continues_without_queries(self):
        stamps = []
        third = threading.Event()
        def collect(config):
            stamps.append(time.monotonic())
            if len(stamps) >= 3:
                third.set()
            return {}
        self.probe.collect.side_effect = collect
        self.assertEqual(self.card.config["sample_interval_seconds"], 1.0)
        self.card.start()
        worker = self.card._worker
        self.assertEqual(self.card.start(), {"state": "running"})
        self.assertIs(self.card._worker, worker)
        self.assertTrue(third.wait(3.5))
        self.card.stop()
        self.assertFalse(worker.is_alive())
        self.assertIsNone(self.card._worker)
        for first, second in zip(stamps, stamps[1:]):
            self.assertGreaterEqual(second - first, 0.85)
            self.assertLess(second - first, 1.6)
        self.assertEqual(self.nodes.mock_calls, [])

    def test_queries_only_copy_cache_and_stopped_sample_is_not_healthy(self):
        self.report()
        calls = self.probe.collect.call_count
        result = self.card.dispatch("robot_info", {})
        result["checks"].clear()
        again = self.card.dispatch("diagnose", {})
        self.assertIn("cpu", again["checks"])
        self.assertEqual(self.probe.collect.call_count, calls)
        self.card.stop()
        stopped = self.card.dispatch("robot_info", {})
        self.assertFalse(stopped["complete"])
        self.assertNotEqual(stopped["overall"], "ok")
        self.assertEqual(stopped["state"], "idle")
        self.assertEqual(self.probe.collect.call_count, calls)

    def test_stale_sampler_cache_is_flagged(self):
        self.report()
        with self.card._lock:
            self.card._latest_monotonic -= 3
        result = self.card.dispatch("robot_info", {})
        self.assertEqual(result["checks"]["monitoring"]["status"], "stale")
        self.assertEqual(result["overall"], "warning")
        self.assertFalse(result["complete"])

    def test_sampling_failure_recovers_on_next_interval(self):
        self.probe.collect.side_effect = [RuntimeError("read failed"), {}]
        self.card.config["sample_interval_seconds"] = 0.05
        first = self.report()
        self.assertEqual(first["checks"]["sampler"]["status"], "unreadable")
        self.wait_for(lambda: self.card._samples >= 2)
        self.assertNotIn("sampler", self.card.dispatch("diagnose", {})["checks"])

    def test_stop_interrupts_wait_and_restart_clears_cache(self):
        self.report()
        worker = self.card._worker
        began = time.monotonic()
        self.card.stop()
        self.assertLess(time.monotonic() - began, 0.5)
        self.assertFalse(worker.is_alive())
        gate = threading.Event()
        self.addCleanup(gate.set)
        self.probe.collect.side_effect = lambda config: (gate.wait(1), {})[1]
        self.card.start()
        result = self.card.dispatch("diagnose", {})
        self.assertIsNone(result["sampled_at"])
        self.assertEqual(result["overall"], "unknown")
        gate.set()
        self.card.stop()

    def test_blocked_reader_stop_timeout_is_reported_and_no_second_worker(self):
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def collect(config):
            entered.set()
            release.wait(2)
            return {}
        self.probe.collect.side_effect = collect
        self.card.start()
        self.assertTrue(entered.wait(1))
        worker = self.card._worker
        with mock.patch.object(worker, "join"):
            self.assertEqual(self.card.stop(), {"state": "stopping"})
        self.assertIs(self.card._worker, worker)
        self.assertEqual(self.card.start(), {"state": "stopping"})
        release.set()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertIsNone(self.card._worker)
        self.assertEqual(self.card._samples, 0)

    def test_worker_start_failure_leaves_no_lifecycle_resources(self):
        with mock.patch("system_health.threading.Thread.start", side_effect=RuntimeError("failed")):
            with self.assertRaises(RuntimeError):
                self.card.start()
        self.assertFalse(self.card.running)
        self.assertIsNone(self.card._worker)
        self.assertIsNone(self.card._stop_event)

    def test_mcp_initialize_list_call_single_wrap(self):
        bundle = DriverBundle([self.card])
        handler_class = make_handler(lambda: bundle, "u1-health-test", "u1-test")
        def request(method, params=None):
            handler = object.__new__(handler_class)
            body = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method,
                               "params": params or {}}).encode()
            handler.path = "/mcp"
            handler.headers = {"Content-Length": str(len(body))}
            handler.rfile = io.BytesIO(body)
            handler.send_json = mock.Mock()
            handler.do_POST()
            code, response = handler.send_json.call_args.args
            self.assertEqual(code, 200)
            return response["result"]
        self.assertEqual(request("initialize")["serverInfo"]["name"], "u1-health-test")
        self.assertEqual(request("tools/list")["tools"][0]["name"], "system_health")
        result = request("tools/call", {"name": "system_health", "arguments": {"action": "robot_info"}})
        self.assertIsInstance(json.loads(result["content"][0]["text"]), dict)
        self.assertEqual(self.nodes.mock_calls, [])

    def test_threshold_validation(self):
        for value in (0, -1, float("nan"), True):
            with self.assertRaises(ValueError):
                SystemHealthPlugin([], self.audio, {"frame_stale_seconds": value})

    def test_enabled_registration_matches_metadata_and_image(self):
        import yaml
        directory = Path(__file__).parent
        config = yaml.safe_load((directory / "config.yaml").read_text())
        meta = yaml.safe_load((directory / "driver.yaml").read_text())
        with mock.patch.object(device, "U1Nodes", return_value=self.nodes):
            plugins = device.build_plugins(config, "test", object())
        self.assertEqual(sum(p.PREFIX == "system_health" for p in plugins), 1)
        self.assertIn({"name": "system_health", "type": "sensor"}, meta["cards"])
        config["plugins"]["system_health"]["enabled"] = False
        with mock.patch.object(device, "U1Nodes", return_value=self.nodes):
            plugins = device.build_plugins(config, "test", object())
        self.assertNotIn("system_health", [p.PREFIX for p in plugins])
        self.assertIn("system_health.py", (directory / "Dockerfile").read_text())


if __name__ == "__main__":
    unittest.main()
