"""
activity_monitor.py — Go1 activity statistics card.

This card is read-only. It samples the shared Go1 HighState snapshot in the
background. A single action=report returns two activity snapshots at once:

- last_30s     — activity statistics for the last 30 seconds
- since_start  — activity statistics from card/sampler start to now

Distance is integrated from horizontal velocity with a deadband. Speeds below
move_eps are treated as zero so standing noise does not accumulate into fake
travel distance.  avg_speed = deadbanded distance / window duration (m/s).
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

CARD = "activity_monitor"
TYPE = "actuator"
DESC = (
    "Report Go1 activity statistics. A single action=report returns both "
    "last_30s and since_start snapshots: distance, moving ratio, idle seconds, "
    "avg/peak speed, current mode, and a one-line summary per range. "
    "Read-only, no robot control commands."
)

# ── tunables (overridable via plugin_config) ──
_HZ = 5.0                 # sampling frequency (samples per second)
_RECENT_WINDOW_SEC = 30.0 # short-window report duration
_HISTORY_SEC = 86400.0    # keep up to 24h of samples for since-start reports
_MOVE_EPS = 0.05          # speed magnitude (m/s) below this is counted as idle
_MAX_INTEGRATION_DT = 2.0 # ignore larger gaps to avoid resume/scheduler spikes


def _speed_mag(vel):
    """Return horizontal speed magnitude from a velocity dict/list."""
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


def _classify(moving_ratio):
    if moving_ratio < 0.1:
        return "idle"
    if moving_ratio < 0.5:
        return "light"
    return "active"


class Plugin:
    def __init__(self, plugin_config, namespace, executor, client):
        self._client = client
        c = plugin_config or {}
        self._hz = float(c.get("sample_hz", _HZ))
        self._recent_window_sec = float(c.get("recent_window_sec", _RECENT_WINDOW_SEC))
        self._history_sec = float(c.get("history_sec", _HISTORY_SEC))
        self._move_eps = float(c.get("move_eps", _MOVE_EPS))
        self._max_integration_dt = float(c.get("max_integration_dt", _MAX_INTEGRATION_DT))

        maxlen = max(2, int(self._hz * self._history_sec))
        # each sample: (ts_sec, raw_speed_mps, effective_speed_mps, is_moving_bool, mode_name_str)
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._running = False
        self._thread = None
        self._started_at = time.time()

    # ── lifecycle ──
    def start(self):
        if self._running:
            return
        self._running = True
        self._started_at = time.time()
        self._thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()
        print(
            f"[activity_monitor] sampler started ({self._hz}Hz, "
            f"recent_window={self._recent_window_sec}s, history={self._history_sec}s)",
            flush=True,
        )

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

            speed, effective_speed, moving, mode = None, 0.0, False, "unknown"
            if snap and snap.get("fresh", False):
                speed = _speed_mag(snap.get("velocity"))
                mode = str(snap.get("mode_name", "unknown"))
                if speed is not None:
                    moving = speed >= self._move_eps
                    effective_speed = speed if moving else 0.0

            # only record frames with usable fresh velocity; skip STUB/stale data
            if speed is not None:
                with self._lock:
                    self._buf.append((ts, speed, effective_speed, moving, mode))

            dt = time.time() - ts
            time.sleep(max(0.0, period - dt))

    # ── tool contract ──
    def get_tool(self):
        return {
            "name": CARD,
            "type": TYPE,
            "multiInstance": False,
            "description": DESC,
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["report"],
                        "description": "report = returns both last_30s and since_start activity statistics",
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
        if action == "report":
            return self._report()
        return None

    # ── statistics ──

    def _select_samples(self, window_sec):
        """Return a copy of recent samples within the given time window."""
        now = time.time()
        with self._lock:
            samples = list(self._buf)
        if window_sec is None:
            return samples
        cutoff = now - window_sec
        return [s for s in samples if s[0] >= cutoff]

    def _compute_stats(self, samples, label):
        """Compute activity statistics from a list of samples."""
        if len(samples) < 2:
            return {
                "range": label,
                "verdict": "no_data",
                "window_sec": 0.0,
                "sample_count": len(samples),
                "distance_m": 0.0,
                "moving_ratio": 0.0,
                "idle_sec": 0.0,
                "avg_speed": 0.0,
                "peak_speed": 0.0,
                "summary": "not enough fresh HighState samples yet",
            }

        t0, tN = samples[0][0], samples[-1][0]
        actual_window_sec = max(0.0, tN - t0)

        distance = 0.0
        moving_cnt = 0
        peak = 0.0

        for i, (ts, raw_speed, effective_speed, moving, mode) in enumerate(samples):
            peak = max(peak, raw_speed)
            if moving:
                moving_cnt += 1

            # Trapezoidal integration with deadbanded speeds.
            if i > 0:
                prev_ts, _prev_raw, prev_effective, _prev_moving, _prev_mode = samples[i - 1]
                dt = ts - prev_ts
                if 0 < dt <= self._max_integration_dt:
                    distance += 0.5 * (prev_effective + effective_speed) * dt

        n = len(samples)
        moving_ratio = moving_cnt / n
        idle_sec = round((1.0 - moving_ratio) * actual_window_sec, 1)
        avg_speed = distance / actual_window_sec if actual_window_sec > 0 else 0.0
        verdict = _classify(moving_ratio)

        return {
            "range": label,
            "verdict": verdict,
            "window_sec": round(actual_window_sec, 1),
            "sample_count": n,
            "distance_m": round(distance, 2),
            "moving_ratio": round(moving_ratio, 2),
            "idle_sec": idle_sec,
            "avg_speed": round(avg_speed, 3),
            "peak_speed": round(peak, 3),
            "summary": (
                f"{label}: {verdict}, moved ~{round(distance, 2)}m, "
                f"{int(moving_ratio * 100)}% of time moving, "
                f"idle {idle_sec}s, peak {round(peak, 2)}m/s"
            ),
        }

    def _report(self):
        """Run both ranges and return merged result."""
        now_samples = self._select_samples(None)
        recent_samples = self._select_samples(self._recent_window_sec)
        current_mode = now_samples[-1][4] if now_samples else "unknown"

        return {
            "ok": True,
            "action": "report",
            "card": CARD,
            "control_level": "HIGHLEVEL",
            "timestamp_ms": int(time.time() * 1000),
            "current_mode": current_mode,
            "move_eps": self._move_eps,
            "last_30s": self._compute_stats(recent_samples, "last_30s"),
            "since_start": self._compute_stats(now_samples, "since_start"),
        }


def make_plugin(plugin_config, namespace, executor, client):
    return Plugin(plugin_config, namespace, executor, client)
