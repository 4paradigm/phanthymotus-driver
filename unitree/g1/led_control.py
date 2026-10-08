"""G1 LED ownership and monotonic RGB sequences; no ROS dependency.

One worker owns all LedControl calls. Manual colours/sequences suppress ordinary
agent hooks; the error hook preempts them. SDK calls are serialized with commands
so a completed stop cannot be followed by an old sequence write.
"""
import bisect
import json
import logging
import math
import threading
import time

_LOG = logging.getLogger(__name__)


class LedPlugin:
    PREFIX = "led"
    _REFRESH_HZ = 5
    _PRIORITY = {"idle": 0, "hearing": 1, "thinking": 3, "speaking": 4, "error": 5}
    _TIMEOUT = {"idle": None, "hearing": 1.2, "thinking": 60, "speaking": 120, "error": 5}

    def __init__(self, plugin_config, namespace, executor, audio_client):
        self._client = audio_client
        self._clock = time.monotonic
        self._release_pending = False
        self._cv = threading.Condition(threading.RLock())
        self._commands = threading.RLock()
        self._thread = None
        self._enabled = False
        self._mode = "state"
        self._state = "idle"
        self._state_ts = 0.0
        self._status = "idle"
        self._rgb = (0, 0, 0)
        self._stages = []
        self._bounds = []
        self._repeats = 1
        self._end_behavior = "release"
        self._origin = 0.0
        self._paused_elapsed = 0.0
        self._elapsed = 0.0
        self._stage = 0
        self._cycle = 0
        self._next_due = 0.0
        self._last_error = None
        self._error_log_ts = float("-inf")

    def get_tool(self):
        rgb = {c: {"type": "integer", "minimum": 0, "maximum": 255,
                   "description": f"{c.upper()} channel intensity (0-255)."} for c in "rgb"}
        stage = {"type": "object", "properties": {
            **rgb, "duration_sec": {"type": "number", "minimum": 0.2, "maximum": 3600}},
            "required": ["r", "g", "b", "duration_sec"], "additionalProperties": False}
        actions = {
            "start": ([], "Start the LED output service and enable state effects. Does not start a color sequence."),
            "state": (["state"], "Request a semantic LED state. Ordinary states are ignored during manual or paused output. The error state terminates the sequence and displays the error effect."),
            "set": (["r", "g", "b"], "Replace the current sequence with a solid RGB color, refreshed continuously at 5 Hz until another command or stop."),
            "cycle": (["sequence", "repeat_count", "end_behavior"],
                      "Start a color sequence asynchronously, replacing the previous sequence. Supply sequence as an array or JSON text with r, g, b, and duration_sec for each stage. repeat_count: 1 for one cycle, N for N cycles, or 0 to loop indefinitely. The driver controls timing and refreshes at 5 Hz; repeated Agent calls are not required."),
            "pause": ([], "Pause sequence timing while continuously refreshing the current color. Use resume to continue."),
            "resume": ([], "Resume the paused sequence from its saved position and remaining duration. Does not restart the sequence."),
            "off": ([], "End the current sequence and continuously refresh black at 5 Hz to keep the LEDs off. Use stop to stop refreshing."),
            "stop": ([], "Terminate the sequence, attempt one black output, and stop all LED refreshing. Cannot be resumed. Firmware may restore its default effect. Use start, set, cycle, or off to restart the service."),
            "info": ([], "Read service and sequence status, stage and cycle indices, RGB values, timing, and hardware errors. Does not start output."),
        }
        return {"name": "led", "type": "actuator", "multiInstance": False,
                "description": "G1 LED control: semantic state effects, continuous RGB colors, and timed color sequences with pause, resume, and stop controls.",
                "inputSchema": {"type": "object", "properties": {
                    "action": {"type": "string", "enum": list(actions)}, **rgb,
                    "state": {"type": "string", "enum": list(self._PRIORITY)},
                    "sequence": {"anyOf": [{"type": "array", "items": stage, "minItems": 1, "maxItems": 256},
                                            {"type": "string"}],
                                 "description": 'Color stages as an array or JSON text. Example: [{"r":0,"g":255,"b":0,"duration_sec":5},{"r":255,"g":0,"b":0,"duration_sec":3}]'},
                    "repeat_count": {"type": "integer", "minimum": 0, "maximum": 10000,
                                     "default": 1, "description": "Number of complete sequence cycles: 1 for once, N for N cycles, or 0 to loop indefinitely until explicitly ended."},
                    "end_behavior": {"type": "string", "enum": ["release", "hold", "off"], "default": "release",
                                     "description": "Behavior after completion: release sends black once and allows subsequent state effects; hold continuously refreshes the final color; off continuously refreshes black."}},
                    "required": ["action"],
                    "x-action-params": {k: {"params": p, "description": d} for k, (p, d) in actions.items()},
                    "x-hooks": {f"on_{s}": {"action": "state", "params": {"state": s}} for s in self._PRIORITY}}}

    @staticmethod
    def _integer(value, low, high, name):
        if type(value) is not int or not low <= value <= high:
            raise ValueError(f"invalid_{name}")
        return value

    @classmethod
    def _color(cls, args):
        return tuple(cls._integer(args.get(c, 0), 0, 255, c) for c in "rgb")

    @classmethod
    def _sequence(cls, value):
        if isinstance(value, str):
            try:
                value = json.loads(value)
            except (ValueError, TypeError):
                raise ValueError("invalid_sequence_json") from None
        if not isinstance(value, list) or not 1 <= len(value) <= 256:
            raise ValueError("sequence_requires_1_to_256_stages")
        stages = []
        for item in value:
            if not isinstance(item, dict) or set(item) != {"r", "g", "b", "duration_sec"}:
                raise ValueError("stage_requires_rgb_and_duration_sec")
            duration = item["duration_sec"]
            if type(duration) not in (int, float) or not math.isfinite(duration) or not 0.2 <= duration <= 3600:
                raise ValueError("invalid_duration_sec")
            stages.append((cls._color(item), float(duration)))
        return stages

    def start(self):
        with self._commands, self._cv:
            if self._enabled:
                return
            if self._thread and self._thread.is_alive():
                raise RuntimeError("led_worker_still_stopping")
            self._enabled = True
            # Fresh lifecycle session; repeated start above is intentionally a no-op.
            self._mode, self._state = "state", "idle"
            self._status = "idle"
            self._stages, self._bounds = [], []
            self._stage = self._cycle = 0
            self._elapsed = self._paused_elapsed = 0.0
            self._origin = self._state_ts = 0.0
            self._repeats = 1
            self._end_behavior = "release"
            self._release_pending = False
            self._rgb = (0, 0, 0)
            self._next_due = 0.0
            self._last_error = None
            self._thread = threading.Thread(target=self._worker, name="g1-led", daemon=True)
            self._thread.start()

    def stop(self):
        with self._commands:
            with self._cv:
                was_enabled = self._enabled
                self._enabled = False
                if self._mode == "cycle":
                    self._advance(self._clock())
                self._release_pending = False
                self._status = "stopped"
                self._mode, self._state = "state", "idle"
                self._cv.notify_all()
                if was_enabled:
                    self._write((0, 0, 0))
                thread = self._thread
            if thread:
                thread.join(timeout=2)

    def dispatch(self, action, args):
        with self._commands:
            if action == "start":
                self.start()
                return {"state": "ready"}
            if action == "stop":
                self.stop()
                return self._info()
            if action == "info":
                return self._info()
            # Validate completely before replacing the current effect.
            if action in ("cycle", "set", "off"):
                # Ignore refresh_hz left by older Canvas configurations.
                # Frequency is fixed, never selected by user input.
                rgb = self._color(args) if action == "set" else (0, 0, 0)
                if action == "cycle":
                    stages = self._sequence(args.get("sequence"))
                    repeats = self._integer(args.get("repeat_count", 1), 0, 10000, "repeat_count")
                    ending = args.get("end_behavior", "release")
                    if ending not in ("release", "hold", "off"):
                        raise ValueError("invalid_end_behavior")
                self.start()
                with self._cv:
                    self._last_error = None
                    self._release_pending = False
                    self._rgb = rgb
                    self._elapsed = self._paused_elapsed = 0.0
                    self._stage = self._cycle = 0
                    self._stages, self._bounds = [], []
                    if action == "cycle":
                        self._stages, self._repeats, self._end_behavior = stages, repeats, ending
                        total = 0.0
                        for _, duration in stages:
                            total += duration
                            self._bounds.append(total)
                        self._origin = self._clock()
                        self._mode, self._status = "cycle", "running"
                        self._rgb = stages[0][0]
                        self._stage = self._cycle = 1
                    else:
                        self._mode, self._status = "hold", "idle"
                    self._next_due = 0.0
                    self._cv.notify_all()
                    return self._info()
            with self._cv:
                now = self._clock()
                if action == "pause":
                    if self._mode == "paused":
                        return self._info()
                    if self._mode != "cycle":
                        raise ValueError("no_running_led_cycle")
                    self._advance(now)
                    if self._mode != "cycle":
                        raise ValueError("led_cycle_already_completed")
                    self._paused_elapsed = self._elapsed
                    self._mode, self._status = "paused", "paused"
                elif action == "resume":
                    if self._mode != "paused":
                        raise ValueError("no_paused_led_cycle")
                    self._origin = now - self._paused_elapsed
                    self._mode, self._status = "cycle", "running"
                elif action == "state":
                    state = args.get("state", "idle")
                    if state not in self._PRIORITY:
                        raise ValueError("invalid_led_state")
                    if not self._enabled or (self._mode != "state" and state != "error"):
                        return {**self._info(), "ignored": True}
                    if self._mode == "state":
                        if self._state == "speaking" and state == "idle" and now - self._state_ts < 1:
                            return {**self._info(), "ignored": True}
                        if state not in ("idle", "error") and self._PRIORITY[state] <= self._PRIORITY[self._state]:
                            return {**self._info(), "ignored": True}
                    elif self._status in ("running", "paused"):
                        self._status = "interrupted"
                    self._release_pending = False
                    self._mode, self._state, self._state_ts = "state", state, now
                else:
                    raise ValueError("unsupported_led_action")
                self._next_due = 0.0
                self._cv.notify_all()
                return self._info()

    def _info(self):
        with self._cv:
            elapsed = self._elapsed
            if self._mode == "cycle":
                elapsed = max(0.0, self._clock() - self._origin)
            total = self._bounds[-1] * self._repeats if self._bounds and self._repeats else None
            if total is not None:
                elapsed = min(elapsed, total)
            return {"state": "ready" if self._enabled else "idle", "mode": self._mode,
                    "semantic_state": self._state, "cycle_status": self._status,
                    "stage_index": self._stage, "cycle_index": self._cycle,
                    "repeat_count": self._repeats if self._stages else None,
                    "elapsed_sec": elapsed, "remaining_sec": max(0, total - elapsed) if total is not None else None,
                    "r": self._rgb[0], "g": self._rgb[1], "b": self._rgb[2],
                    "refresh_hz": self._REFRESH_HZ, "last_error": self._last_error}

    def _advance(self, now):
        self._elapsed = max(0.0, now - self._origin)
        period = self._bounds[-1]
        if self._repeats and self._elapsed >= period * self._repeats:
            self._elapsed = period * self._repeats
            self._cycle, self._stage = self._repeats, len(self._stages)
            self._status = "completed"
            self._rgb = self._stages[-1][0] if self._end_behavior == "hold" else (0, 0, 0)
            self._mode = "state" if self._end_behavior == "release" else "hold"
            self._state = "idle"
            self._release_pending = self._end_behavior == "release"
            return None
        self._cycle = int(self._elapsed // period) + 1
        offset = self._elapsed % period
        index = bisect.bisect_right(self._bounds, offset)
        self._stage = index + 1
        self._rgb = self._stages[index][0]
        return self._bounds[index] - offset

    def _write(self, rgb):
        try:
            ret = self._client.LedControl(*rgb)
            if ret != 0:
                raise RuntimeError(f"LedControl code={ret}")
            self._rgb = rgb
            return True
        except Exception as exc:
            # Stop the failed effect, rather than retry/log at 5 Hz indefinitely.
            self._last_error = ascii(str(exc)[:200])[:240]
            self._status, self._mode, self._state = "failed", "state", "idle"
            now = self._clock()
            if now - self._error_log_ts >= 5:
                _LOG.warning("LED output stopped: %s", self._last_error)
                self._error_log_ts = now
            return False

    def _semantic_rgb(self, now):
        age = now - self._state_ts
        timeout = self._TIMEOUT[self._state]
        if timeout is not None and age >= timeout:
            self._state = "idle"
        if self._state == "idle":
            return None
        if self._state == "hearing":
            return (0, 255, 80)
        if self._state == "speaking":
            return (129, 216, 208)
        if self._state == "error":
            return (255, 0, 0)
        palette = [(0, 200, 255), (80, 40, 255), (180, 0, 255), (255, 0, 180)]
        step = age / .03
        pos = (step % 200) / 50
        idx, frac = int(pos), pos % 1
        brightness = .3 + .7 * (.5 + .5 * math.sin(step * .06))
        return tuple(int((palette[idx][i] * (1 - frac) + palette[(idx + 1) % 4][i] * frac) * brightness) for i in range(3))

    def _worker(self):
        with self._cv:
            while self._enabled:
                now = self._clock()
                if now < self._next_due:
                    self._cv.wait(self._next_due - now)
                    continue
                boundary = self._advance(now) if self._mode == "cycle" else None
                # A just-completed release still writes black once.
                released = self._release_pending
                rgb = self._semantic_rgb(now) if self._mode == "state" else self._rgb
                if released:
                    self._release_pending = False
                    rgb = (0, 0, 0)
                if rgb is None:
                    self._cv.wait()
                    continue
                self._write(rgb)
                interval = .03 if self._mode == "state" else 1 / self._REFRESH_HZ
                due = now + min(interval, boundary) if boundary is not None else now + interval
                # Always yield the lock after a slow RPC, so stop/pause cannot
                # starve while the SDK takes longer than the refresh interval.
                self._next_due = max(due, self._clock() + .001)
                if released:
                    # Wait for a new command; avoid repeated writes after release.
                    self._cv.wait()
