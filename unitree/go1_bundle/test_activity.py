"""
test_activity.py — Go1 activity-statistics card (actuator). [UNACCEPTED · test-prefixed]

NOTE: file/module/card name is `test_activity` until acceptance (team rule: unverified
cards carry a `test` prefix). Rename to `activity` (file + CARD + config key together)
once verified on the real dog.

Fills the one dimension the rest of the bundle leaves empty: TIME.
fall_alarm / odometry / system_health all report the *instantaneous* state ("is it
tilted now", "how far from origin now", "is everything healthy now"). None of them
answer "what has the robot been doing over the last N seconds".

This card keeps a background sampler that reads the shared client.snapshot() at a
fixed rate and pushes lightweight samples into a fixed-length sliding window
(collections.deque). When decision_core needs a summary, it calls action=`report`
and gets a one-shot activity digest: distance travelled, moving ratio, idle time,
avg/peak speed, mode switches, and a one-line verdict.

Read-only: it only *reads* velocity/mode from the shared snapshot. No control
commands, no locomotion UDP, no new data source, no camera — zero conflict with any
existing card.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

CARD = "test_activity"   # must match file/module/config-key; drop `test_` prefix after acceptance
TYPE = "actuator"
DESC = ("Report what the robot has been doing over a recent time window. A background "
        "sampler continuously records speed / motion-state / gait-mode into a sliding "
        "window; action=report returns distance travelled, moving-ratio, idle time, "
        "avg/peak speed, mode switches and a one-line verdict. Read-only, no control.")

# ── tunables (overridable via plugin_config) ──
_HZ = 5.0             # sampling frequency (samples per second)
_WINDOW_SEC = 60.0    # how much history to keep
_MOVE_EPS = 0.05      # speed magnitude (m/s) below which we count the dog as "idle"


def _speed_mag(vel):
    """L2 magnitude of a linear velocity dict/list; robust to missing fields."""
    if vel is None:
        return None
    try:
        if isinstance(vel, dict):
            vx = float(vel.get("vx", vel.get("x", 0.0)) or 0.0)
            vy = float(vel.get("vy", vel.get("y", 0.0)) or 0.0)
        elif isinstance(vel, (list, tuple)) and len(vel) >= 2:
            vx, vy = float(vel[0] or 0.0), float(vel[1] or 0.0)
        else:
            return None
        return math.hypot(vx, vy)
    except (TypeError, ValueError):
        return None


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        c = plugin_config or {}
        self._hz = float(c.get("sample_hz", _HZ))
        self._window_sec = float(c.get("window_sec", _WINDOW_SEC))
        self._move_eps = float(c.get("move_eps", _MOVE_EPS))

        maxlen = max(1, int(self._hz * self._window_sec))
        # each sample: (ts_sec, speed_mag_or_None, is_moving_bool, mode_name_str)
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._started_at = None

    # ── lifecycle ──
    def start(self):
        if self._running:
            return
        self._running = True
        self._started_at = time.time()
        self._thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()
        print(f"[activity] sampler started ({self._hz}Hz, window={self._window_sec}s)", flush=True)

    def stop(self):
        self._running = False
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=2.0)
        self._thread = None

    def _sample_loop(self):
        period = 1.0 / self._hz if self._hz > 0 else 0.2
        while self._running:
            ts = time.time()
            try:
                snap = self._client.snapshot() if self._client else None
            except Exception:  # noqa: BLE001 — never let the sampler die
                snap = None

            speed, moving, mode = None, False, "unknown"
            if snap and snap.get("fresh", False):
                speed = _speed_mag(snap.get("velocity"))
                mode = str(snap.get("mode_name", "unknown"))
                if speed is not None:
                    moving = speed >= self._move_eps

            # only record frames that carried a usable speed reading; skip STUB/stale
            if speed is not None:
                with self._lock:
                    self._buf.append((ts, speed, moving, mode))

            # sleep the remainder of the period (account for work time)
            dt = time.time() - ts
            time.sleep(max(0.0, period - dt))

    # ── tool contract ──
    def get_tool(self):
        return {
            "name": CARD, "type": TYPE, "multiInstance": False, "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["report", "reset"],
                        "description": "report = activity digest over the current window; "
                                       "reset = clear the window and start counting fresh",
                    }
                },
                "required": ["action"],
            },
        }

    def dispatch(self, action, args):
        if action in ("start", "info"):
            return {"state": "ready"}
        if action == "stop":
            return {"state": "idle"}
        if action == "reset":
            with self._lock:
                self._buf.clear()
            self._started_at = time.time()
            return {"ok": True, "action": "reset", "card": CARD, "msg": "window cleared"}
        if action == "report":
            return self._report()
        return None

    # ── the statistics ──
    def _report(self):
        with self._lock:
            samples = list(self._buf)

        if len(samples) < 2:
            return {
                "ok": True, "action": "report", "card": CARD,
                "control_level": "HIGHLEVEL", "timestamp_ms": int(time.time() * 1000),
                "verdict": "no_data",
                "summary": "not enough samples yet (need the dog awake with fresh HighState)",
                "sample_count": len(samples),
            }

        t0, tN = samples[0][0], samples[-1][0]
        window_sec = max(0.0, tN - t0)

        distance = 0.0
        moving_cnt = 0
        speed_sum = 0.0
        peak = 0.0
        mode_switches = 0
        prev_mode = samples[0][3]

        for i, (ts, speed, moving, mode) in enumerate(samples):
            speed_sum += speed
            peak = max(peak, speed)
            if moving:
                moving_cnt += 1
            if mode != prev_mode:
                mode_switches += 1
                prev_mode = mode
            # integrate distance using the *actual* gap to the previous sample,
            # so scheduler jitter doesn't bias the result (more honest than a fixed dt)
            if i > 0:
                dt = ts - samples[i - 1][0]
                if 0 < dt < 2.0:  # guard against gaps (paused thread, resume)
                    distance += speed * dt

        n = len(samples)
        moving_ratio = moving_cnt / n
        avg_speed = speed_sum / n
        idle_sec = round((1.0 - moving_ratio) * window_sec, 1)

        if moving_ratio < 0.1:
            verdict = "idle"          # basically standing still
        elif moving_ratio < 0.5:
            verdict = "light"         # occasional movement
        else:
            verdict = "active"        # moving most of the time

        return {
            "ok": True, "action": "report", "card": CARD,
            "control_level": "HIGHLEVEL", "timestamp_ms": int(time.time() * 1000),
            "verdict": verdict,
            "window_sec": round(window_sec, 1),
            "sample_count": n,
            "distance_m": round(distance, 2),
            "moving_ratio": round(moving_ratio, 2),
            "idle_sec": idle_sec,
            "avg_speed": round(avg_speed, 3),
            "peak_speed": round(peak, 3),
            "mode_switches": mode_switches,
            "current_mode": samples[-1][3],
            "summary": (f"over last {round(window_sec, 1)}s: {verdict}, "
                        f"moved ~{round(distance, 2)}m, "
                        f"{int(moving_ratio * 100)}% of time moving, "
                        f"idle {idle_sec}s, peak {round(peak, 2)}m/s"),
        }


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
