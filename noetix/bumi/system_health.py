"""Read-only Bumi host observations; importing this module needs no ROS or SDK."""

from __future__ import annotations

import copy
import json
import math
import os
import posixpath
import re
import threading
import time
from pathlib import Path


_READ_ERRORS = (OSError, ValueError, IndexError, KeyError, ZeroDivisionError)
_ACTIONS = ["start", "stop", "info", "robot_info"]
_DEFAULTS = {
    "sample_interval_s": 2.0,
    "cpu_load_per_core_warn": 1.0,
    "memory_used_percent_warn": 90.0,
    "disk_used_percent_warn": 90.0,
    "temperature_warn_celsius": None,
    "disk_paths": ["/opt/phanthy-motus/data"],
}


def _error(exc):
    return f"{type(exc).__name__}: {exc}"[:200]


def parse_policy(cfg):
    """Validate deployment policy without silently accepting misspelled keys."""
    if cfg is None:
        cfg = {}
    if not isinstance(cfg, dict):
        raise ValueError("system_health config must be a mapping")
    unknown = set(cfg) - set(_DEFAULTS) - {"enabled"}
    if unknown:
        raise ValueError(f"unknown system_health key: {next(iter(unknown))}")
    policy = copy.deepcopy(_DEFAULTS)
    policy.update({key: value for key, value in cfg.items() if key != "enabled"})
    ranges = {
        "sample_interval_s": (0.5, 60),
        "cpu_load_per_core_warn": (0.01, 100),
        "memory_used_percent_warn": (1, 100),
        "disk_used_percent_warn": (1, 100),
        "temperature_warn_celsius": (1, 150),
    }
    for key, (lower, upper) in ranges.items():
        value = policy[key]
        if key == "temperature_warn_celsius" and value is None:
            continue
        try:
            valid = (not isinstance(value, bool)
                     and isinstance(value, (int, float))
                     and math.isfinite(value) and lower <= value <= upper)
        except OverflowError:
            valid = False
        if not valid:
            raise ValueError(f"{key} must be a finite number in [{lower}, {upper}]")
        policy[key] = float(value)
    paths = policy["disk_paths"]
    if not isinstance(paths, list) or not 1 <= len(paths) <= 8:
        raise ValueError("disk_paths must contain 1 to 8 absolute Linux paths")
    if any(not isinstance(path, str) or not path.startswith("/")
           or "\x00" in path for path in paths):
        raise ValueError("disk_paths must contain absolute Linux paths")
    # These are container paths, including when unit tests run on Windows.
    paths = [posixpath.normpath(path) for path in paths]
    if len(set(paths)) != len(paths):
        raise ValueError("disk_paths contains duplicate normalized paths")
    policy["disk_paths"] = paths
    return policy


def _check(status, source, scope, value=None, reasons=()):
    return {"status": status, "source": source, "scope": scope,
            "value": value, "reasons": [str(reason)[:200] for reason in reasons]}


def _verdict(checks):
    """Keep observed warnings even when other observations are incomplete."""
    warnings = [key for key, item in checks.items()
                if item["status"] in ("warning", "stale")]
    unknown = [key for key, item in checks.items()
               if item["status"] not in ("ok", "warning", "stale")]
    complete = bool(checks) and all(
        item["status"] in ("ok", "warning")
        and (item.get("value") or {}).get("coverage_complete", True)
        for item in checks.values())
    overall = "warning" if warnings else "unknown" if unknown or not complete else "ok"
    if overall == "ok":
        summary = "全部已评估项正常"
    else:
        parts = []
        if warnings:
            parts.append("告警：" + "、".join(warnings))
        if unknown:
            parts.append("未知或未评级：" + "、".join(unknown))
        summary = "；".join(parts or ["未知或未评级：checks"]) + "（不代表全部正常）"
    return {
        "overall": overall, "complete": complete, "summary": summary,
        "fresh": checks.get("monitoring", {}).get("status") == "ok",
        "problems": [{"check": key, "status": item["status"],
                      "reasons": list(item["reasons"])}
                     for key, item in checks.items() if item["status"] != "ok"],
    }


class SystemProbe:
    """Linux proc/sysfs reader. Only the sampling thread owns its CPU baseline."""

    def __init__(self, root="/", statvfs=None):
        self._root = Path(root)
        # os.statvfs is absent on Windows; injected filesystem fixtures still work.
        self._statvfs = statvfs if statvfs is not None else getattr(os, "statvfs", None)
        self._previous_cpu = None

    def _path(self, path):
        return self._root / path.lstrip("/")

    def _read(self, path):
        return self._path(path).read_text(encoding="utf-8").strip()

    def _proc_scopes(self):
        try:
            mounts = self._read("/proc/self/mounts").splitlines()
            if not mounts:
                raise ValueError("empty mount table")
            covered = set()
            for line in mounts:
                fields = line.split()
                if len(fields) < 3:
                    raise ValueError("invalid mount table entry")
                if fields[0] == "lxcfs" or fields[2].startswith("fuse.lxcfs"):
                    covered.add(re.sub(r"\\([0-7]{3})",
                                       lambda match: chr(int(match[1], 8)), fields[1]))
            return covered, None
        except _READ_ERRORS as exc:
            return None, _error(exc)

    def _temperature(self, policy):
        zones = [path for path in self._path("/sys/class/thermal").glob("thermal_zone*")
                 if re.fullmatch(r"thermal_zone\d+", path.name)]
        zones.sort(key=lambda path: int(path.name[len("thermal_zone"):]))
        if not zones:
            raise OSError("no thermal zones available")
        readings = []
        for zone in zones:
            try:
                kind = (zone / "type").read_text(encoding="utf-8").strip()[:64]
            except (OSError, ValueError):
                kind = "unlabeled"
            item = {"zone": zone.name, "type": kind}
            try:
                celsius = int((zone / "temp").read_text(encoding="utf-8").strip()) / 1000
                if not -50 <= celsius <= 200:
                    raise ValueError("temperature outside plausible range [-50, 200]")
                item["celsius"] = round(celsius, 1)
            except _READ_ERRORS as exc:
                item["error"] = _error(exc)
            readings.append(item)
        valid = [item for item in readings if "celsius" in item]
        missing = len(readings) - len(valid)
        value = {"zones": readings, "hardware_identity_confirmed": False,
                 "coverage_complete": missing == 0,
                 "max_celsius": None, "hottest_zone": None, "hottest_type": None}
        reasons = []
        if missing:
            reasons.append(f"{missing} thermal zone(s) unreadable")
        if not valid:
            return _check("unreadable", "/sys/class/thermal/thermal_zone*",
                          "host_sysfs", value, reasons)
        hottest = max(valid, key=lambda item: item["celsius"])
        value.update(max_celsius=hottest["celsius"], hottest_zone=hottest["zone"],
                     hottest_type=hottest["type"])
        limit = policy["temperature_warn_celsius"]
        if limit is None:
            status = "unrated"
            reasons.append("no verified Bumi thermal limit is configured; "
                           "values are reported, not judged")
        elif hottest["celsius"] >= limit:
            status = "warning"
            reasons.append(f"maximum observed temperature >= {limit:g} C")
        else:
            status = "ok"
        # Retain a known hot reading, but never call partially observed zones OK.
        if missing and status != "warning":
            status = "unreadable"
        return _check(status, "/sys/class/thermal/thermal_zone*", "host_sysfs", value, reasons)

    def _cpu(self, policy, scope):
        loads = [float(value) for value in self._read("/proc/loadavg").split()[:3]]
        if len(loads) != 3 or not all(math.isfinite(x) and x >= 0 for x in loads):
            raise ValueError("loadavg requires three finite nonnegative values")
        rows = self._read("/proc/stat").splitlines()
        if not rows or not rows[0].startswith("cpu "):
            raise ValueError("missing aggregate cpu row")
        ticks = [int(value) for value in rows[0].split()[1:]]
        cores = sum(bool(re.match(r"^cpu\d+\s", row)) for row in rows[1:])
        if len(ticks) < 4 or any(tick < 0 for tick in ticks) or not cores:
            raise ValueError("invalid CPU counters or missing cores")
        total, idle = sum(ticks[:8]), ticks[3] + (ticks[4] if len(ticks) > 4 else 0)
        usage = None
        if self._previous_cpu is not None:
            delta = total - self._previous_cpu[0]
            idle_delta = idle - self._previous_cpu[1]
            if delta > 0 and 0 <= idle_delta <= delta:
                usage = round(100 * (delta - idle_delta) / delta, 1)
        self._previous_cpu = total, idle
        ratio = loads[0] / cores
        warning = ratio >= policy["cpu_load_per_core_warn"]
        return _check("warning" if warning else "ok", "/proc/loadavg, /proc/stat", scope,
                      {"load_1_5_15": loads, "cores": cores,
                       "load_per_core": round(ratio, 3), "usage_percent": usage},
                      ["1-minute load per core exceeds observation policy"] if warning else [])

    def _memory(self, policy, scope):
        values = {}
        for line in self._read("/proc/meminfo").splitlines():
            key, _, raw = line.partition(":")
            if key in ("MemTotal", "MemAvailable"):
                fields = raw.split()
                if len(fields) != 2 or fields[1] != "kB":
                    raise ValueError(f"{key} must use kB")
                values[key] = int(fields[0]) * 1024
        total, available = values["MemTotal"], values["MemAvailable"]
        if total <= 0 or not 0 <= available <= total:
            raise ValueError("inconsistent memory totals")
        percent = round(100 * (total - available) / total, 1)
        warning = percent >= policy["memory_used_percent_warn"]
        return _check("warning" if warning else "ok", "/proc/meminfo", scope,
                      {"total_bytes": total, "available_bytes": available,
                       "used_percent": percent},
                      ["memory usage exceeds observation policy"] if warning else [])

    def _disk(self, path, policy):
        if self._statvfs is None:
            raise OSError("statvfs is unavailable on this platform")
        fs = self._statvfs(self._path(path))
        size = fs.f_frsize or fs.f_bsize
        used = (fs.f_blocks - fs.f_bfree) * size
        available = fs.f_bavail * size
        if (size <= 0 or fs.f_blocks <= 0 or used < 0 or available < 0
                or used + available <= 0 or fs.f_bavail > fs.f_bfree):
            raise ValueError("inconsistent filesystem counters")
        percent = round(100 * used / (used + available), 1)
        warning = percent >= policy["disk_used_percent_warn"]
        return _check("warning" if warning else "ok", f"statvfs({path})",
                      "mounted_filesystem", {"path": path, "total_bytes": fs.f_blocks * size,
                                             "available_bytes": available, "used_percent": percent},
                      ["filesystem usage exceeds observation policy"] if warning else [])

    def collect(self, policy):
        covered, scope_error = self._proc_scopes()

        def scope(path):
            return ("unknown" if covered is None else
                    "container_lxcfs" if path in covered else "host_kernel")

        checks = {}

        def read(name, source, metric_scope, callback):
            try:
                checks[name] = callback()
            except _READ_ERRORS as exc:
                checks[name] = _check("unreadable", source, metric_scope, reasons=[_error(exc)])
            if metric_scope == "unknown":
                checks[name]["reasons"].append(("proc scope could not be established: "
                                                + scope_error)[:200])

        read("temperature", "/sys/class/thermal/thermal_zone*", "host_sysfs",
             lambda: self._temperature(policy))
        cpu_scope = scope("/proc/loadavg")
        if scope("/proc/stat") == "container_lxcfs":
            cpu_scope = "container_lxcfs"
        read("cpu_load", "/proc/loadavg, /proc/stat", cpu_scope,
             lambda: self._cpu(policy, cpu_scope))
        read("memory", "/proc/meminfo", scope("/proc/meminfo"),
             lambda: self._memory(policy, scope("/proc/meminfo")))
        for path in policy["disk_paths"]:
            read(f"disk:{path}", f"statvfs({path})", "mounted_filesystem",
                 lambda path=path: self._disk(path, policy))
        host = {"uptime_s": None, "kernel": None, "proc_scope": scope("/proc/uptime")}
        try:
            uptime = float(self._read("/proc/uptime").split()[0])
            if math.isfinite(uptime) and uptime >= 0:
                host["uptime_s"] = round(uptime, 1)
        except _READ_ERRORS:
            pass
        try:
            host["kernel"] = self._read("/proc/sys/kernel/osrelease")[:64]
        except _READ_ERRORS:
            pass
        return {"checks": checks, "host": host}


class SystemHealthPlugin:
    """One sampler, cache-only queries, and an executor timer for stalled samples.

    publisher_factory() returns a callable accepting a JSON string. Its default
    creates one ROS node lazily; tests can inject list.append without using ROS.
    Lifecycle serialization never holds the cache lock across I/O or join().
    """

    PREFIX = "system_health"
    _JOIN_TIMEOUT_S = 2.0

    def __init__(self, plugin_config, namespace, executor, *,
                 probe=None, publisher_factory=None, clock=time.monotonic):
        self._policy = parse_policy(plugin_config)
        self._topic = f"/{namespace}/state/system_health"
        self._executor = executor
        self._probe = probe if probe is not None else SystemProbe()
        self._publisher_factory = (publisher_factory if publisher_factory is not None
                                   else self._create_publisher)
        self._clock = clock
        self._lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._publish_lock = threading.Lock()
        self._worker = None
        self._stop_event = None
        self._sample = None
        self._sample_time = None
        self._samples = 0
        self._publisher = None
        self._publisher_error = "not started"
        self._node = None
        self._publish_failures = 0
        self._last_overall = None

    def get_tool(self):
        return {
            "name": self.PREFIX, "type": "sensor", "multiInstance": False,
            "description": (
                "Bumi 主机（计算板）只读健康状态：芯片温度、CPU 负载与使用率、内存、"
                "数据分区剩余空间。后台每 "
                f"{self._policy['sample_interval_s']:g} 秒采样一次并发布到 {self._topic}；"
                "robot_info 只读最近一次缓存，不现场采样，不控制机器人。"
                "用它回答主机状态、是否过热、内存或磁盘是否够用。overall=ok 才表示"
                "全部已评估项正常；unknown、unrated、unreadable、stale 都不代表正常。"
                "电池、关节和运动状态请看 battery、motion_state 卡片。"),
            "inputSchema": {"type": "object", "properties": {
                "action": {"type": "string", "enum": list(_ACTIONS), "description":
                           "start/stop/info 是生命周期动作；robot_info 读取最近一次主机健康报告"}},
                "required": ["action"]},
            "topic_out": self._topic_out(),
        }

    def _topic_out(self):
        return [{"topic": self._topic, "format": "data/json"}]

    def _create_publisher(self):
        from rclpy.node import Node
        from std_msgs.msg import String

        node = Node("bumi_system_health")
        try:
            publisher = node.create_publisher(String, self._topic, 10)
            # This callback runs on the executor, independently of blocked I/O in
            # the sampler. It only republishes unknown/stale cache observations.
            node.create_timer(self._policy["sample_interval_s"], self._publish_stale)
            self._executor.add_node(node)
        except Exception:
            node.destroy_node()
            raise
        self._node = node

        def publish(payload):
            msg = String()
            msg.data = payload
            publisher.publish(msg)

        return publish

    def start(self):
        with self._lifecycle_lock:
            with self._lock:
                if self._worker is not None and self._worker.is_alive():
                    return
            if self._publisher is None:
                try:
                    publisher = self._publisher_factory()
                    if not callable(publisher):
                        raise ValueError("publisher factory must return a callable")
                    with self._lock:
                        self._publisher = publisher
                        self._publisher_error = None
                except Exception as exc:
                    with self._lock:
                        self._publisher_error = _error(exc)
                    print(f"[system_health] WARNING: publisher unavailable: {_error(exc)}", flush=True)
            with self._lock:
                self._stop_event = threading.Event()
                self._worker = threading.Thread(
                    target=self._sample_loop, args=(self._stop_event,),
                    name="bumi-system-health", daemon=True)
                self._worker.start()
            print(f"[system_health] sampling every {self._policy['sample_interval_s']:g}s"
                  f" -> {self._topic}", flush=True)

    def stop(self):
        with self._lifecycle_lock:
            with self._lock:
                worker = self._worker
                if self._stop_event is not None:
                    self._stop_event.set()
            if worker is not None:
                worker.join(timeout=self._JOIN_TIMEOUT_S)
            with self._lock:
                if worker is None or not worker.is_alive():
                    self._worker = None
                    self._stop_event = None
                    stopping = False
                else:
                    stopping = True
            if stopping:
                print("[system_health] WARNING: sampler still stopping", flush=True)

    def _sample_loop(self, stop_event):
        deadline = time.monotonic()
        while not stop_event.is_set():
            # Age includes collection latency, rather than relabeling a slow
            # observation as fresh when the final file read eventually returns.
            sample_time = self._clock()
            timestamp_ms = int(time.time() * 1000)
            try:
                result = self._probe.collect(self._policy)
            except Exception as exc:
                result = {"checks": {"sampler": _check(
                    "unreadable", "SystemProbe.collect", "driver_process", reasons=[_error(exc)])},
                    "host": {}}
            with self._lock:
                if stop_event.is_set():
                    return
                self._sample = copy.deepcopy(result)
                self._sample["timestamp_ms"] = timestamp_ms
                self._sample_time = sample_time
                self._samples += 1
            self._publish_report()
            deadline += self._policy["sample_interval_s"]
            now = time.monotonic()
            if deadline <= now:
                deadline = now + self._policy["sample_interval_s"]
            stop_event.wait(deadline - now)

    def _report(self):
        with self._lock:
            alive = self._worker is not None and self._worker.is_alive()
            stopping = alive and self._stop_event.is_set()
            state = "stopping" if stopping else "running" if alive else "idle"
            sample = copy.deepcopy(self._sample)
            age = None if sample is None else max(0.0, self._clock() - self._sample_time)
            samples = self._samples
        checks = sample["checks"] if sample is not None else {}
        if state != "running":
            status, reasons = "not_started", ["sampler is not running"]
        elif sample is None:
            status, reasons = "unknown", ["no sample collected yet"]
        elif age >= 3 * self._policy["sample_interval_s"]:
            status, reasons = "stale", ["sample age exceeds 3 sampling intervals"]
        else:
            status, reasons = "ok", []
        checks["monitoring"] = _check(status, "sampler cache", "driver_process",
                                      {"samples": samples}, reasons)
        return {
            "card": self.PREFIX, "timestamp_ms": sample["timestamp_ms"] if sample else None,
            "checks": checks, "host": sample["host"] if sample else {}, "state": state,
            "sample_age_s": round(age, 2) if age is not None else None,
            "sample_interval_s": self._policy["sample_interval_s"], **_verdict(checks),
        }

    def _compact(self, report):
        # The generic data/json viewer clips each top-level value at 200
        # characters. Keep independently useful measurements as scalar fields;
        # the complete structured checks belong only in robot_info.
        compact = {key: copy.deepcopy(report[key]) for key in
                   ("card", "timestamp_ms", "fresh", "overall", "complete", "summary", "problems")}
        fields = {
            "temperature": {
                "temperature_celsius": "max_celsius",
                "temperature_coverage_complete": "coverage_complete",
            },
            "cpu_load": {
                "cpu_load_1_5_15": "load_1_5_15",
                "cpu_load_per_core": "load_per_core",
                "cpu_usage_percent": "usage_percent",
            },
            "memory": {
                "memory_used_percent": "used_percent",
                "memory_available_bytes": "available_bytes",
            },
        }
        for name, measurements in fields.items():
            check = report["checks"].get(name, {})
            value = check.get("value") or {}
            compact[f"{name}_status"] = check.get("status", "unknown")
            # Explicit nulls also clear earlier readings after a failed sample.
            for field, source in measurements.items():
                compact[field] = copy.deepcopy(value.get(source))
        for path in self._policy["disk_paths"]:
            name = f"disk:{path}"
            check = report["checks"].get(name, {})
            value = check.get("value") or {}
            compact[f"{name}:status"] = check.get("status", "unknown")
            compact[f"{name}:used_percent"] = value.get("used_percent")
            compact[f"{name}:available_bytes"] = value.get("available_bytes")
        monitoring = report["checks"]["monitoring"]
        compact["monitoring_status"] = monitoring["status"]
        compact["samples"] = monitoring["value"]["samples"]
        compact["sample_age_s"] = report["sample_age_s"]
        return compact

    def _publish_report(self, *, stale_only=False):
        with self._publish_lock:
            # Snapshot in publication order so a delayed watchdog callback cannot
            # publish an old stale verdict after a newer fresh sample.
            report = self._report()
            if stale_only and (report["state"] != "running" or report["fresh"]):
                return
            with self._lock:
                if self._stop_event is None or self._stop_event.is_set():
                    return
                publisher = self._publisher
            if report["overall"] != self._last_overall:
                self._last_overall = report["overall"]
                problems = [item["check"] for item in report["problems"]]
                print(f"[system_health] overall={report['overall']} "
                      f"complete={report['complete']} problems={problems}", flush=True)
            if publisher is None:
                return
            try:
                publisher(json.dumps(self._compact(report), allow_nan=False))
            except Exception as exc:
                self._publish_failures += 1
                if self._publish_failures == 1 or self._publish_failures % 100 == 0:
                    print(f"[system_health] publish failed ({self._publish_failures}): "
                          f"{_error(exc)}", flush=True)
            else:
                if self._publish_failures:
                    print("[system_health] publisher recovered", flush=True)
                self._publish_failures = 0

    def _publish_stale(self):
        self._publish_report(stale_only=True)

    def dispatch(self, action, args):
        if action == "start":
            # Always-on sensor: canvas actions only report state. The bundle
            # owns start()/stop(), including creation of ROS objects and threads.
            return {"state": self._report()["state"], "topic_out": self._topic_out()}
        if action == "stop":
            # Canvas lifecycle does not shut down a persistent sensor.
            return {"state": "idle"}
        if action == "robot_info":
            return self._report()
        if action == "info":
            report = self._report()
            with self._lock:
                publisher = ("ready" if self._publisher is not None else
                             f"unavailable: {self._publisher_error}")
            return {"state": report["state"], "topic_out": self._topic_out(),
                    "samples_collected": report["checks"]["monitoring"]["value"]["samples"],
                    "sample_age_s": report["sample_age_s"], "publisher": publisher,
                    "policy": copy.deepcopy(self._policy)}
        return None
