"""
system_health.py — Go1 self-diagnostic card (actuator).

decision_core calls action=`diagnose` when the robot misbehaves; returns a one-shot health check
with per-item OK/WARNING/CRITICAL and an overall verdict pointing at the problem.
Covers the compute board (CPU temp/load, memory, disk, power throttle, network, key process)
and robot subsystems (battery, comm link, motion state).
- OS metrics: read /proc, /sys, vcgencmd (visible to the container under --pid host --privileged).
- Battery: subscribe MQTT bms/state (dog's on-board broker); does not rely on standard-SDK bms.
- Comm/motion: read the shared client.snapshot() fresh / mode_name.
Read-only, no control commands, no locomotion UDP — zero conflict.
"""

from __future__ import annotations

import os
import struct
import subprocess
import threading
import time

try:
    import paho.mqtt.client as mqtt
    _HAS_MQTT = True
except Exception:
    _HAS_MQTT = False

CARD = "system_health"
TYPE = "actuator"
DESC = ("Run a full self-diagnostic when the robot misbehaves. Checks the compute board "
        "(CPU temp/load, memory, disk, power throttle, network, key process) and robot subsystems "
        "(battery, comm link, motion state); returns per-item OK/WARNING/CRITICAL + overall verdict.")

_MARK = {"OK": "[OK]", "WARNING": "[WARN]", "CRITICAL": "[CRIT]", "INFO": "[i]", "UNKNOWN": "[?]"}
_RK = {"OK": 0, "INFO": 0, "UNKNOWN": 0, "WARNING": 1, "CRITICAL": 2}


def _run(cmd, timeout=3):
    out = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                         timeout=timeout, universal_newlines=True)
    return out.stdout.strip()


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        c = plugin_config or {}
        self._host = c.get("mqtt_host", "localhost")
        self._port = int(c.get("mqtt_port", 1883))
        self._mqtt = None
        self._bms = None
        self._lock = threading.Lock()

    def start(self):
        if not _HAS_MQTT:
            return
        try:
            self._mqtt = mqtt.Client()
            self._mqtt.on_message = self._on_msg
            self._mqtt.connect(self._host, self._port, 60)
            self._mqtt.subscribe("bms/state")
            self._mqtt.loop_start()
            print("[system_health] MQTT connected (subscribed bms/state)", flush=True)
        except Exception as e:  # noqa: BLE001
            print(f"[system_health] MQTT connect failed: {e}", flush=True)
            self._mqtt = None

    def stop(self):
        try:
            if self._mqtt:
                self._mqtt.loop_stop()
                self._mqtt.disconnect()
        except Exception:
            pass

    def _on_msg(self, cl, userdata, msg):
        with self._lock:
            self._bms = bytes(msg.payload)

    def get_tool(self):
        return {"name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
                "inputSchema": {"type": "object",
                                "properties": {"action": {"type": "string", "enum": ["diagnose"],
                                                           "description": "Run full diagnostic"}},
                                "required": ["action"]}}

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action != "diagnose":
            return None

        report, problems, counts, worst = {}, [], {}, [0]

        def add(label, status, disp):
            report[label] = f"{_MARK.get(status, '')} {disp}"
            counts[status] = counts.get(status, 0) + 1
            if _RK.get(status, 0) > worst[0]:
                worst[0] = _RK.get(status, 0)
            if status in ("WARNING", "CRITICAL"):
                problems.append(f"{_MARK[status]} {label}: {disp}")

        # ── compute board ──
        try:
            temps = []
            for z in os.listdir("/sys/class/thermal"):
                if z.startswith("thermal_zone"):
                    try:
                        with open(f"/sys/class/thermal/{z}/temp") as f:
                            temps.append(int(f.read().strip()) / 1000.0)
                    except Exception:
                        pass
            if temps:
                t = round(max(temps), 1)
                add("cpu_temp", "OK" if t < 70 else ("WARNING" if t < 80 else "CRITICAL"),
                    f"{t}C" + ("" if t < 80 else " hot"))
            else:
                add("cpu_temp", "UNKNOWN", "read failed")
        except Exception as e:  # noqa: BLE001
            add("cpu_temp", "UNKNOWN", str(e))

        try:
            with open("/proc/loadavg") as f:
                load1 = float(f.read().split()[0])
            n = os.cpu_count() or 1
            r = load1 / n
            add("cpu_load", "OK" if r < 0.8 else ("WARNING" if r < 1.2 else "CRITICAL"),
                f"{load1} / {n}cores" + (" overloaded" if r >= 0.8 else ""))
        except Exception as e:  # noqa: BLE001
            add("cpu_load", "UNKNOWN", str(e))

        try:
            mi = {}
            with open("/proc/meminfo") as f:
                for line in f:
                    k, _, v = line.partition(":")
                    if v:
                        mi[k] = int(v.strip().split()[0])
            total, avail = mi.get("MemTotal", 0), mi.get("MemAvailable", 0)
            up = round(100 * (total - avail) / total, 1) if total else 0
            add("memory", "OK" if up < 80 else ("WARNING" if up < 92 else "CRITICAL"),
                f"{up}% used (avail {avail // 1024}M / total {total // 1024}M)")
        except Exception as e:  # noqa: BLE001
            add("memory", "UNKNOWN", str(e))

        try:
            vfs = os.statvfs("/")
            total = vfs.f_blocks * vfs.f_frsize
            free = vfs.f_bavail * vfs.f_frsize
            up = round(100 * (total - free) / total, 1) if total else 0
            add("disk", "OK" if up < 85 else ("WARNING" if up < 95 else "CRITICAL"),
                f"{up}% used (free {free // (1024 ** 3)}G)")
        except Exception as e:  # noqa: BLE001
            add("disk", "UNKNOWN", str(e))

        try:
            raw = _run(["vcgencmd", "get_throttled"])
            val = raw.split("=")[-1] if "=" in raw else raw
            flags = int(val, 16) if val.startswith("0x") else 0
            if flags == 0:
                add("power", "OK", "normal")
            else:
                add("power", "CRITICAL" if (flags & 0x5) else "WARNING", f"{val} undervolt/throttle")
        except Exception:
            add("power", "UNKNOWN", "vcgencmd unavailable")

        try:
            ips = _run(["hostname", "-I"]).split()
            add("network", "OK" if ips else "CRITICAL", f"{len(ips)} IP" if ips else "no IP")
        except Exception as e:  # noqa: BLE001
            add("network", "UNKNOWN", str(e))

        try:
            running = bool(_run(["pgrep", "-x", "Legged_sport"]))
            add("sport_process", "OK" if running else "WARNING",
                "Legged_sport running" if running else "not running")
        except Exception as e:  # noqa: BLE001
            add("sport_process", "UNKNOWN", str(e))

        # ── robot subsystems ──
        try:
            snap = self._client.snapshot() if self._client else {"fresh": False}
            fresh = bool(snap.get("fresh", False))
            if not fresh:
                add("robot_comm", "WARNING", "no fresh HighState (std SDK cannot read this dog / lying / STUB)")
            else:
                add("robot_comm", "OK", f"HighState fresh (mode={snap.get('mode_name', '?')})")
            add("motion_mode", "INFO", str(snap.get("mode_name", "unknown")))
        except Exception as e:  # noqa: BLE001
            add("robot_comm", "UNKNOWN", str(e))

        try:
            with self._lock:
                raw = self._bms
            if not raw or len(raw) < 8:
                add("battery", "UNKNOWN", "no bms/state (MQTT) yet")
            else:
                soc = raw[3]
                cur = struct.unpack_from("<i", raw, 4)[0]
                add("battery", "OK" if soc > 30 else ("WARNING" if soc > 15 else "CRITICAL"),
                    f"{soc}% {'charging' if cur > 0 else 'discharging'} {abs(cur)}mA")
        except Exception as e:  # noqa: BLE001
            add("battery", "UNKNOWN", str(e))

        overall = ["OK", "WARNING", "CRITICAL"][worst[0]]
        return {
            "ok": True, "action": "diagnose", "card": CARD,
            "control_level": "HIGHLEVEL", "timestamp_ms": int(time.time() * 1000),
            "overall": overall,
            "summary": f"{sum(counts.values())} checks: {counts.get('OK', 0)} OK / "
                       f"{counts.get('WARNING', 0)} warn / {counts.get('CRITICAL', 0)} crit",
            "problems": problems if problems else ["none, all OK"],
            "report": report,
        }


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
