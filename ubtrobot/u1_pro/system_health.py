"""Periodic read-only health observations; no ROS or robot calls."""
from __future__ import annotations

import copy
import math
import os
import threading
import time
from pathlib import Path

from common.vendor_runtime import action_schema, tool


DEFAULTS = {"sample_interval_seconds": 1.0, "cpu_load_per_core_warn": 1.0, "memory_used_percent_warn": 90.0,
            "disk_used_percent_warn": 90.0, "frame_stale_seconds": 5.0,
            "playback_stale_seconds": 120.0, "temperature_warn_celsius": None}


def observation(status, source, scope, value=None, reasons=(), sampled_at=None):
    return {"status": status, "source": source, "scope": scope, "value": value,
            "sampled_at": sampled_at, "reasons": list(reasons)}


class SystemProbe:
    """Read only the current process's Linux view, never infer host capacity."""

    def __init__(self, root=Path("/")):
        self.root = Path(root)

    def read(self, path):
        return (self.root / path.lstrip("/")).read_text().strip()

    def cgroup_memory(self):
        # Resolve membership against mounts rather than assuming cgroup root.
        memberships = self.read("/proc/self/cgroup").splitlines()
        mounts = self.read("/proc/self/mountinfo").splitlines()
        for version, controller, member in (line.split(":", 2) for line in memberships):
            for line in mounts:
                before, after = line.split(" - ", 1)
                fields, fs = before.split(), after.split()
                v2 = version == "0" and fs[0] == "cgroup2"
                v1 = "memory" in controller.split(",") and fs[0] == "cgroup" and "memory" in fs[2].split(",")
                if not (v1 or v2):
                    continue
                mount_root, mount_point = fields[3:5]
                if member == "/":
                    # A cgroup namespace can expose its own root as / while
                    # mountinfo still names the host subtree. Parents are hidden.
                    relative = "/"
                elif mount_root != "/":
                    if not (member == mount_root or member.startswith(mount_root + "/")):
                        continue
                    relative = member[len(mount_root):]
                else:
                    relative = member
                directory = Path(mount_point) / relative.lstrip("/")
                usage_name = "memory.current" if v2 else "memory.usage_in_bytes"
                limit_name = "memory.max" if v2 else "memory.limit_in_bytes"
                usage = int(self.read(str(directory / usage_name)))
                limits, sources, pressures = [], [], []
                while True:
                    path = str(directory / limit_name)
                    raw = self.read(path)
                    sources.append(path)
                    if raw != "max":
                        limit = int(raw)
                        if limit < (1 << 60):
                            if limit <= 0:
                                raise ValueError("invalid cgroup memory limit")
                            limits.append(limit)
                            ancestor_usage = int(self.read(str(directory / usage_name)))
                            if ancestor_usage < 0:
                                raise ValueError("invalid cgroup memory usage")
                            pressures.append(100 * ancestor_usage / limit)
                    if directory == Path(mount_point):
                        break
                    directory = directory.parent
                # Hidden ancestors above a namespaced mount cannot be verified.
                return {"usage_bytes": usage, "limit_bytes": min(limits) if limits else None,
                        "sources": sources, "hierarchy_complete": mount_root == "/" and v2,
                        "version": 2 if v2 else 1,
                        "max_hierarchy_used_percent": max(pressures) if pressures else None}
        raise OSError("memory cgroup membership/mount could not be resolved")

    def collect(self, config):
        sampled = time.time()
        container = os.environ.get("U1_PRO_HEALTH_CONTAINER") == "1" or (self.root / ".dockerenv").exists()
        scope = "container_visible" if container else "process_visible_host_unconfirmed"
        checks = {}

        def read_check(name, source, func):
            try:
                value, warning = func()
                checks[name] = observation("warning" if warning else "ok", source, scope,
                                           value, [warning] if warning else [], sampled)
            except (OSError, ValueError, IndexError, KeyError, ZeroDivisionError) as exc:
                checks[name] = observation("unreadable", source, scope, reasons=[str(exc)[:256]], sampled_at=sampled)

        def cpu():
            loads = [float(x) for x in self.read("/proc/loadavg").split()[:3]]
            cores = sum(line.startswith("cpu") and line[3:4].isdigit()
                        for line in self.read("/proc/stat").splitlines())
            if len(loads) != 3 or cores <= 0 or any(not math.isfinite(x) or x < 0 for x in loads):
                raise ValueError("invalid load/core sample")
            ratio = loads[0] / cores
            return {"load_1_5_15": loads, "visible_cores": cores, "load_per_visible_core": ratio,
                    "note": "Linux load average, not CPU utilization; not normalized to container CPU quota"}, (
                        "load per visible core exceeds policy" if ratio >= config["cpu_load_per_core_warn"] else "")

        def memory():
            data = {k: int(v.split()[0]) * 1024 for k, v in
                    (line.split(":", 1) for line in self.read("/proc/meminfo").splitlines())}
            total, available = data["MemTotal"], data["MemAvailable"]
            if not 0 <= available <= total or total <= 0:
                raise ValueError("invalid memory capacity")
            percent = 100 * (total - available) / total
            return {"total_bytes": total, "available_bytes": available, "used_percent": percent,
                    "note": "/proc view; does not establish container memory limit"}, (
                        "memory use exceeds policy" if percent >= config["memory_used_percent_warn"] else "")

        def disk():
            path = config["disk_path"]
            stat = os.statvfs(self.root / path.lstrip("/"))
            total, free = stat.f_blocks * stat.f_frsize, stat.f_bavail * stat.f_frsize
            if total <= 0 or not 0 <= free <= total:
                raise ValueError("invalid filesystem capacity")
            percent = 100 * (total - free) / total
            return {"path": path, "total_bytes": total, "available_bytes": free, "used_percent": percent,
                    "note": "capacity of this mounted filesystem; not host disk inventory"}, (
                        "filesystem use exceeds policy" if percent >= config["disk_used_percent_warn"] else "")

        read_check("cpu_load", "/proc/loadavg + /proc/stat", cpu)
        read_check("memory", "/proc/meminfo", memory)
        read_check("disk", "statvfs:" + config["disk_path"], disk)
        try:
            group = self.cgroup_memory()
            if container:
                group["hierarchy_complete"] = False
            percent = group["max_hierarchy_used_percent"]
            group["used_percent"] = percent
            warning = percent is not None and percent >= config["memory_used_percent_warn"]
            status = "warning" if warning else "ok"
            reasons = ["cgroup memory use exceeds policy"] if warning else []
            if not group["hierarchy_complete"]:
                status = "unknown" if not warning else "warning"
                reasons.append("host ancestor limits cannot be confirmed from this cgroup view")
            checks["memory_limit"] = observation(status, group["sources"], "process_cgroup", group, reasons, sampled)
        except (OSError, ValueError, IndexError, ZeroDivisionError) as exc:
            checks["memory_limit"] = observation("unreadable", "/proc/self/cgroup + /proc/self/mountinfo", "process_cgroup",
                                                 reasons=[str(exc)[:256]], sampled_at=sampled)
        thermal = self.root / "sys/class/thermal"
        hwmon = self.root / "sys/class/hwmon"
        sensors = [(zone.name, zone / "temp", zone / "type")
                   for zone in sorted(thermal.glob("thermal_zone*"))]
        sensors.extend((monitor.name + "_" + value.stem, value,
                        value.with_name(value.name.replace("_input", "_label")))
                       for monitor in sorted(hwmon.glob("hwmon*"))
                       for value in sorted(monitor.glob("temp*_input")))
        if not sensors:
            checks["temperature"] = observation("unreadable", "/sys/class/thermal + /sys/class/hwmon", scope,
                                                 reasons=["no readable temperature sensors discovered"], sampled_at=sampled)
        for name, path, label_path in sensors:
            source = str(path)
            try:
                temperature = float(path.read_text()) / 1000
                try:
                    label = label_path.read_text().strip()
                except OSError:
                    label = "unlabeled"
                if not math.isfinite(temperature) or not -100 <= temperature <= 250:
                    raise ValueError("invalid thermal sample")
                threshold = config["temperature_warn_celsius"]
                warning = threshold is not None and temperature >= threshold
                # A sysfs label alone cannot prove this is the U1 CPU sensor.
                checks[name] = observation("warning" if warning else "unknown", source, scope,
                    {"celsius": temperature, "label": label, "hardware_identity_confirmed": False},
                    (["temperature exceeds configured policy"] if warning else []) +
                    ["sensor hardware identity unconfirmed; no vendor thermal limit assumed"], sampled)
            except (OSError, ValueError) as exc:
                checks[name] = observation("unreadable", source, scope, reasons=[str(exc)[:256]], sampled_at=sampled)
        return checks


class SystemHealthPlugin:
    PREFIX = "system_health"

    def __init__(self, cameras, audio, config=None, probe=None):
        self.cameras = tuple(cameras)
        self.audio = audio
        self.probe = probe or SystemProbe()
        self.config = {**DEFAULTS, "disk_path": "/", **(config or {})}
        for key in DEFAULTS:
            value = self.config[key]
            if key == "temperature_warn_celsius" and value is None:
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if not isinstance(self.config["disk_path"], str) or not self.config["disk_path"].startswith("/"):
            raise ValueError("disk_path must be absolute")
        self._lock = threading.Lock()
        self.running = False
        self._worker = None
        self._stop_event = None
        self._latest = None
        self._latest_monotonic = None
        self._samples = 0

    def get_tool(self):
        actions = {name: ([], description) for name, description in {
            "start": "Start continuous read-only sampling every configured interval.",
            "stop": "Stop and join the sampler; no robot behavior.",
            "info": "Read card lifecycle state.",
            "robot_info": "Read the latest cached observation without starting devices or sampling on demand.",
            "diagnose": "Alias of robot_info."}.items()}
        return tool(self.PREFIX, "sensor", "U1 Pro 只读系统健康：计算资源、可见温度、已有相机及播放状态；未知项不表示正常。",
                    action_schema(actions, {}))

    def _state_locked(self):
        if self.running:
            return "running"
        return "stopping" if self._worker is not None else "idle"

    def start(self):
        with self._lock:
            if self._worker is not None:
                return {"state": self._state_locked()}
            self._latest = None
            self._latest_monotonic = None
            self._samples = 0
            self._stop_event = threading.Event()
            self.running = True
            self._worker = threading.Thread(
                target=self._run, args=(self._stop_event,),
                name="u1-system-health", daemon=True)
            try:
                self._worker.start()
            except Exception:
                self.running = False
                self._worker = None
                self._stop_event = None
                raise
        return {"state": "running"}

    def stop(self):
        with self._lock:
            self.running = False
            worker = self._worker
            if self._stop_event is not None:
                self._stop_event.set()
        # Do not hold the snapshot lock while joining. A blocked OS read cannot
        # be forcibly cancelled: keep its worker registered and refuse restart
        # until it exits, rather than spawning overlapping collectors.
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2.0)
        with self._lock:
            return {"state": self._state_locked()}

    def _run(self, stop_event):
        interval = self.config["sample_interval_seconds"]
        deadline = time.monotonic()
        try:
            while not stop_event.is_set():
                started = time.monotonic()
                try:
                    report = self._sample()
                except Exception as exc:
                    report = {
                        "card": self.PREFIX, "sampled_at": time.time(),
                        "checks": {"sampler": observation(
                            "unreadable", "SystemHealthPlugin sampler", "driver",
                            reasons=[str(exc)[:256]], sampled_at=time.time())},
                        "overall": "unknown", "complete": False,
                        "problems": [{"check": "sampler", "status": "unreadable",
                                      "reasons": [str(exc)[:256]]}],
                        "policy": dict(self.config),
                        "summary": "sampling failed; not all normal"}
                with self._lock:
                    if stop_event.is_set():
                        break
                    self._latest = report
                    self._latest_monotonic = started
                    self._samples += 1
                deadline = max(deadline + interval, time.monotonic())
                if stop_event.wait(max(0.0, deadline - time.monotonic())):
                    break
        finally:
            with self._lock:
                if self._worker is threading.current_thread():
                    self.running = False
                    self._worker = None
                    self._stop_event = None

    def _cached_report(self):
        with self._lock:
            result = copy.deepcopy(self._latest)
            stamp = self._latest_monotonic
            state = self._state_locked()
            samples = self._samples
        age = None if stamp is None else max(0.0, time.monotonic() - stamp)
        if result is None:
            result = {"card": self.PREFIX, "sampled_at": None,
                      "overall": "unknown", "complete": False, "checks": {},
                      "problems": [], "policy": dict(self.config),
                      "summary": "no current sample; not all normal"}
        result.update(state=state, sample_age_seconds=age, samples_collected=samples,
                      sample_interval_seconds=self.config["sample_interval_seconds"])
        status, reasons = "ok", []
        if state != "running":
            status, reasons = "not_started", ["continuous monitoring is stopped"]
        elif age is None:
            status, reasons = "unknown", ["waiting for first sample"]
        elif age >= 2 * self.config["sample_interval_seconds"]:
            status, reasons = "stale", ["sampler has not produced a fresh observation"]
        if status != "ok":
            result["checks"]["monitoring"] = observation(
                status, "SystemHealthPlugin sampler", "driver",
                reasons=reasons, sampled_at=time.time())
            result["problems"].append({"check": "monitoring", "status": status, "reasons": reasons})
            result["complete"] = False
            if status == "stale":
                result["overall"] = "warning"
            elif result["overall"] == "ok":
                result["overall"] = "unknown"
            result["summary"] = "monitoring incomplete or stopped; not all normal"
        return result

    def dispatch(self, action, args):
        del args
        if action == "start":
            return self.start()
        if action == "stop":
            return self.stop()
        if action == "info":
            with self._lock:
                return {"state": self._state_locked(), "samples_collected": self._samples,
                        "sample_interval_seconds": self.config["sample_interval_seconds"]}
        if action in {"robot_info", "diagnose"}:
            return self._cached_report()
        return None

    def _sample(self):
        checks = dict(self.probe.collect(self.config))
        for component in (*self.cameras, self.audio):
            try:
                item = component.health_snapshot(self.config)
            except Exception as exc:
                item = observation("unreadable", "driver:" + component.PREFIX, "driver",
                                   reasons=[str(exc)[:256]], sampled_at=time.time())
            checks[component.PREFIX] = item
        problems = [{"check": name, "status": item["status"], "reasons": item["reasons"]}
                    for name, item in checks.items() if item["status"] != "ok"]
        incomplete = any(
            item["status"] in {"unknown", "unreadable", "stale", "not_started", "error"}
            or (isinstance(item["value"], dict) and (
                item["value"].get("hardware_identity_confirmed") is False
                or item["value"].get("hierarchy_complete") is False))
            for item in checks.values())
        overall = "warning" if any(item["status"] in {"warning", "error", "stale"} for item in checks.values()) else ("unknown" if incomplete else "ok")
        return {"card": self.PREFIX, "sampled_at": time.time(),
                "overall": overall, "complete": not incomplete, "checks": checks,
                "problems": problems, "policy": dict(self.config),
                "summary": "all observed checks normal" if overall == "ok" else "warnings or incomplete observations; not all normal"}
