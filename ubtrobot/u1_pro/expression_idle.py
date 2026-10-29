"""Background scheduling of existing SDK expression presets, without native motion."""

from __future__ import annotations

import random
import threading
import time
import uuid

from native_idle_actions import safe_native_detail


class ExpressionIdleLoop:
    WEIGHTS = {"blink": 70, "gaze": 20, "distracted": 10}
    GAP_MIN = 3.0
    GAP_MAX = 7.0
    ACTION_TIMEOUT = 120.0
    POLL_INTERVAL = 0.25

    def __init__(self, audio, expressions, control, notify):
        self.audio = audio
        self.expressions = {name: value for name, value in expressions.items() if name in self.WEIGHTS}
        self.control = control
        self.notify = notify
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._enabled = False
        self._state = "stopped"
        self._current = None
        self._last_expression = None
        self._last_error = None
        self._completed = 0
        self._rng = random.Random()

    def info(self):
        with self._lock:
            return self._snapshot()

    def _snapshot(self):
        return {"enabled": self._enabled, "state": self._state,
                "current_action_id": self._current, "last_expression": self._last_expression,
                "completed_count": self._completed, "last_error": self._last_error,
                "available_expressions": sorted(self.expressions)}

    def start(self, action_id):
        with self._lock:
            if not self.expressions:
                raise ValueError("No blink/gaze/distracted preset is available on this firmware")
            if self.control is None:
                raise RuntimeError("expression idle requires the vendor idle preference reader")
            if self._enabled or (self._thread is not None and self._thread.is_alive()):
                raise ValueError("expression idle is already starting/running or still stopping; query info first")
            self._stop = threading.Event()
            self._enabled = True
            self._state = "starting"
            self._last_error = None
            self._last_expression = None
            self._completed = 0
            worker = threading.Thread(target=self._run, args=(action_id, self._stop),
                                      name="u1-expression-idle", daemon=True)
            self._thread = worker
            try:
                worker.start()
            except Exception:
                self._enabled = False
                self._state = "stopped"
                raise
        return {"state": "starting", "action": "idle_start", "action_id": action_id,
                "message": "正在检查原厂待机开关，随后周期性播放眨眼、注视和发呆预设。"}

    def stop(self):
        with self._lock:
            self._stop.set()
            self._enabled = False
            if self._state != "error":
                self._state = "stopped"
            return {"state": "stopped", "action": "idle_stop",
                    "message": "已停止安排新的待机表情，已接收的单次动作允许执行完成。",
                    "idle_loop": self._snapshot()}

    def _pick(self, previous):
        names = [name for name in self.expressions if name != previous or name == "blink"]
        if not names:
            names = list(self.expressions)
        return self._rng.choices(names, weights=[self.WEIGHTS[name] for name in names], k=1)[0]

    def _run(self, start_action_id, stop):
        startup_reported = False
        try:
            state = self.control.status()
            if stop.is_set():
                self.notify(start_action_id, "cancelled", {"state": "stopped", "action": "idle_start"})
                startup_reported = True
                return
            if state.get("ok") is not True or state.get("data", {}).get("enabled") is not False:
                raise RuntimeError("请先通过 idle_actions.disable 关闭原厂自动待机，并等待当前动作结束；本循环不修改该开关")
            with self._lock:
                if stop.is_set():
                    current = "stopped"
                else:
                    self._state = "running"
                    current = "running"
            self.notify(start_action_id, "completed" if current == "running" else "cancelled",
                        {"state": current, "action": "idle_start", "idle_loop": self.info()})
            startup_reported = True
            previous = None
            while not stop.wait(self._rng.uniform(self.GAP_MIN, self.GAP_MAX)):
                with self.audio._lock:
                    busy = self.audio._active is not None
                if busy:
                    continue
                name = self._pick(previous)
                result = self._play_once(name, stop)
                if result == "stopped":
                    return
                if result == "busy":
                    continue
                previous = name
                with self._lock:
                    self._completed += 1
        except Exception as exc:
            message = safe_native_detail(exc, 512)
            with self._lock:
                self._state = "error"
                self._last_error = message
            if not startup_reported:
                self.notify(start_action_id, "error", {"state": "error", "action": "idle_start", "message": message})
            else:
                print(f"[U1 expression idle] stopped: {message}")
        finally:
            with self._lock:
                self._enabled = False
                if self._state != "error":
                    self._state = "stopped"

    def _play_once(self, name, stop):
        done = threading.Event()
        outcome = {}
        outcome_lock = threading.Lock()
        action_id = "expression-idle-" + str(uuid.uuid4())

        def completed(status, result):
            with outcome_lock:
                if done.is_set():
                    return
                outcome.update(status=status, result=result)
                done.set()
            with self._lock:
                if self._current == action_id:
                    self._current = None

        with self._lock:
            if stop.is_set():
                return "stopped"
            self._current = action_id
        try:
            response = self.audio._queue(
                "play_action", {"action": self.expressions[name][0]}, action_id, "expression",
                on_complete=completed, cancel_event=stop)
        except Exception:
            with self._lock:
                if self._current == action_id:
                    self._current = None
            raise
        if response.get("state") != "queued":
            with self._lock:
                if self._current == action_id:
                    self._current = None
            if response.get("state") == "cancelled":
                return "stopped"
            if response.get("state") == "error" and response.get("message") == "another U1 Pro audio action is active":
                return "busy"
            raise RuntimeError("Unexpected idle expression response")
        with self._lock:
            self._last_expression = name
        deadline = time.monotonic() + self.ACTION_TIMEOUT
        while not stop.is_set():
            if done.wait(self.POLL_INTERVAL):
                if outcome["status"] != "completed":
                    raise RuntimeError("Idle expression ended with status " + str(outcome["status"]))
                return "completed"
            if time.monotonic() >= deadline:
                raise RuntimeError("Idle expression completion timed out; no automatic retry or interrupt")
        return "stopped"
