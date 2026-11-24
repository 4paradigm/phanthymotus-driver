#!/usr/bin/env python3
"""Waist sway for Noetix Bumi — a standing patrol sway through the low-level joint stream.

`waist_sway` is a call-shaped action like `loco.move`, not a deploy-time
configured toggle: the card is dropped onto a canvas bare, and every parameter
— how long, how far, how fast — is given at call time (`start`). While
running, the robot rotates its upper body left and right around the waist yaw,
both when standing still and while the chassis cards (`loco`,
`loco_relative_move`, `loco_servo`) are walking. Any action that needs the
waist itself — the preset animations, teaching, posture changes, protection
and fall states — stops the sway automatically, exactly like `loco` outranks
`loco_servo`.

── why this card has to be low-level, and what that costs ────────────────────

The high-level controller has no waist channel at all: `publish_cmd(x, y, z,
action, index)` either walks or plays a whole-body firmware animation, and the
two are mutually exclusive workmodes. The only execution surface for "waist
while standing (or walking)" is `LowController.set_joint`, which takes all 21
joints at once — there is no per-joint API.

So the stream is the vendor's own standing hold: the official
`example_policy_walk.cpp` `handleStandMode` commands every joint to a fixed
stand-pose table with kp=10/kd=5e-1 at 500 Hz and the robot stands. This card
copies that table verbatim, adds a slow sine on the waist joint, and blends in
from the measured pose over the first second so takeover is not a step.

Three firmware behaviours are **unknown** (the controller is closed-source and
no example mixes high- and low-level):

1. Whether a low-level stream can take over while the robot stands in walking
   mode at all.
2. How the firmware arbitrates a low-level stream against high-level walking.
   Per-joint merge gives the intended result; "high wins" shows up as the sway
   pausing while the robot walks; "low wins" shows up as the robot standing
   still while `loco` believes it is walking. Both endpoints leave the robot
   upright; an alternating arbitration would not.
3. Whether the WALK mode edge restores high-level control after the stream
   stops.

The card is built to make all three observable from `info`: a waist tracking
self-check reports whether the physical joint follows the command, and the
release path records what the restore attempt saw. First runs belong at low
amplitude on the slow speed, with a person holding the e-stop.

── arbitration, mirroring the `loco` / `loco_servo` split ─────────────────────

Chassis cards coexist with the sway (the whole point is swaying while
walking). Body cards never coexist: `LocoPlugin` notifies this card from its
dispatch entry points, and the supervisor also watches `workmode` so actions
started from the vendor App get the same release, and ends a timed sway when
its duration elapses. An explicit instruction outranks a running sway, never
the other way round.
"""

from __future__ import annotations

import math
import threading
import time

# The stand pose, verbatim from the vendor example (`example_policy_walk.cpp`
# init(), `test_policy.py` standJointAngles_) in the example's jointNames
# order. `getJointsIndex` maps each name to the hardware motor index at
# runtime; the driver's own table (device.py `_BUMI_JOINT_NAMES`) says the
# waist is motor 20, which `_EXPECTED_WAIST_HW_INDEX` cross-checks.
_SDK_JOINT_NAMES = [
    "leg_l1_joint", "leg_r1_joint", "waist_1_joint", "leg_l2_joint",
    "leg_r2_joint", "arm_l1_joint", "arm_r1_joint", "leg_l3_joint",
    "leg_r3_joint", "arm_l2_joint", "arm_r2_joint", "leg_l4_joint",
    "leg_r4_joint", "arm_l3_joint", "arm_r3_joint", "leg_l5_joint",
    "leg_r5_joint", "arm_l4_joint", "arm_r4_joint", "leg_l6_joint",
    "leg_r6_joint",
]
_STAND_ANGLES = [
    -0.1495, -0.1495,   # leg_l1, leg_r1
    0.0,                # waist_1
    0.0, 0.0,           # leg_l2, leg_r2
    0.0, 0.0,           # arm_l1, arm_r1
    0.0, 0.0,           # leg_l3, leg_r3
    0.2618, -0.2618,    # arm_l2, arm_r2
    0.3215, 0.3215,     # leg_l4, leg_r4
    0.0, 0.0,           # arm_l3, arm_r3
    -0.1720, -0.1720,   # leg_l5, leg_r5
    0.0, 0.0,           # arm_l4, arm_r4
    0.0, 0.0,           # leg_l6, leg_r6
]
_WAIST_SDK_NAME = "waist_1_joint"
_EXPECTED_WAIST_HW_INDEX = 20

# The vendor standing hold, applied to every non-waist joint.
_HOLD_KP = 10.0
_HOLD_KD = 0.5

# URDF waist_yaw_joint limit is ±1.57 rad; the operator ceiling stays below it.
_WAIST_LIMIT_RAD = 1.57
_MAX_AMPLITUDE_RAD = 1.2

# Operator-facing parameters ride on the `start` action (configured on the
# canvas after the card is dropped in, like `loco.move`'s velocities) — there
# is deliberately no deploy-time configSchema. Speed is two presets rather
# than a free period: a patrol sway has a "how fast does it feel" answer, not
# a tuned number.
_SPEED_PERIODS = {"slow": 8.0, "medium": 4.0}   # seconds per full left-right cycle
_DEFAULT_SPEED = "slow"
_DEFAULT_AMPLITUDE_DEG = 20.0
_MIN_AMPLITUDE_DEG = 5.0
_MAX_AMPLITUDE_DEG = 60.0
_MAX_DURATION_S = 600.0
_RATE_HZ = 500.0   # vendor example loop rate; fixed, not operator-facing

# Workmode gate and the modes that mean somebody else owns the waist. Names
# mirror device.py `_WORKMODE_NAMES` (kept local so this file loads without
# device.py on a laptop). Engage is walking-mode only: 0/1 can be a lying
# enabled robot (stand_up is legal there) and a waist stream there would drag
# the upper body on the floor.
_ENGAGE_MODES = {2}
_WORKMODE_NAMES = {
    0: "enabled", 1: "ready", 2: "walking", 5: "dance",
    8: "greet", 9: "shake", 10: "cheer", 11: "start_teach",
    12: "end_teach", 14: "save_teach_1", 23: "play_teach",
    26: "protection", 27: "fall_to_stand", 28: "stand_to_fall",
    29: "save_teach_2", 30: "disabled", 31: "dance1", 32: "dance2", 33: "tear",
}
# Presets, dances, teaching, protection and fall transitions all move or own
# the waist; so do ready/enabled (the robot may be lying there). The
# supervisor releases the sway on every mode the engage gate would refuse.

_SUPERVISOR_HZ = 10.0
_BLEND_IN_S = 1.0        # measured pose -> stand table
_WIND_DOWN_S = 1.0       # operator stop: amplitude decays to zero
_PREEMPT_WIND_DOWN_S = 0.5   # body action / chassis suspend: snappier decay
_RELEASE_JOIN_S = 3.0        # after the graceful window, insist on a join
_RELEASE_FINAL_JOIN_S = 2.0  # after insisting: the thread is declared stuck
_TRACKING_TOLERANCE_RAD = 0.35
_TRACKING_GRACE_S = 0.5


class WaistSwayPlugin:
    PREFIX = "waistsway"

    def __init__(self, plugin_config: dict, namespace: str, executor, high_ctrl,
                 loco_plugin=None):
        self._high_ctrl = high_ctrl
        self._namespace = namespace
        self._executor = executor
        self._loco = loco_plugin

        # No deploy-time tuning: the operator parameters below are the `start`
        # action's arguments, and these are just their pre-start defaults.
        self._amplitude_rad = math.radians(_DEFAULT_AMPLITUDE_DEG)
        self._period_s = _SPEED_PERIODS[_DEFAULT_SPEED]
        self._speed = _DEFAULT_SPEED
        self._end_time: float | None = None   # monotonic deadline; None = until stop

        self._lock = threading.RLock()
        self._enabled = False
        # Bumped by every disable (operator stop, body-action preemption,
        # bundle teardown). A start whose slow release window overlapped a
        # disable rechecks it before re-enabling — the lifecycle-token rule
        # from loco_servo, so a stop can never be silently overwritten by the
        # restart it raced.
        self._generation = 0
        self._state = "off"          # off|streaming|error
        self._preempted_by: str | None = None
        self._last_error: str | None = None
        self._last_restore: dict | None = None
        self._last_preempt_restore: dict | None = None
        self._chassis_active: str | None = None
        self._wind_down_end: float | None = None
        self._cmds_sent = 0
        self._waist_tracking = "unknown"
        self._workmode: int | None = None
        self._confirmed = False        # runtime gate: no command without it
        self._restore_needed = False   # a real set_joint went out since last restore

        self._stream_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._supervisor_thread: threading.Thread | None = None
        self._supervisor_stop = threading.Event()

        # LowController singleton state; resolved lazily inside the stream
        # thread so neither module import nor card construction needs the SDK.
        self._low = None
        self._MotorCmd = None
        self._low_init_failed: str | None = None

    # ── tool ─────────────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "waist_sway",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Bumi 腰部巡视摆动（实验性）：开始后上身腰部 yaw 持续小幅来回转动，"
                f"持续 duration_s 秒（0=直到 stop）。幅度、快慢、时长在调用时给定：单侧"
                f"幅度 {_MIN_AMPLITUDE_DEG:g}–{_MAX_AMPLITUDE_DEG:g} 度，快慢分慢"
                f"（约 {_SPEED_PERIODS['slow']:g} 秒一个来回）/中（约 "
                f"{_SPEED_PERIODS['medium']:g} 秒一个来回）两档。站立静止时摆动，"
                "loco / loco_relative_move / loco_servo 行走期间继续发送；"
                "semantic_action / action_recording / stand_up_lie_prone 或保护、摔倒等"
                "占用腰部的动作一旦出现，自动停止并释放腰部（preempted_by 记录原因）。"
                "实现走 SDK 低层 set_joint（高层无腰部通道，行走与上身动画在固件互斥），"
                "以官方示例站立角表做全 21 关节位置保持。注意：固件对高层行走与低层关节"
                "流并存的仲裁未知——若低层被忽略则行走正常、腰部停转；若低层生效则机器"
                "人保持站立、底盘命令可能失效。confirm 在运行时校验：非 true 时卡片"
                "静默停止且不发出任何关节指令（智能控制/Agent 调用自动携带布尔 true）。"
                "首次真机请小幅度、慢档、有人看护并确保 App 急停可用。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["start", "stop", "info"],
                        "description": "start=按参数开始摆动；stop=幅度归零后停止并释放；"
                                       "info=查看状态",
                    },
                    "duration_s": {
                        "type": "number", "minimum": 0, "maximum": _MAX_DURATION_S,
                        "description": "来回转身持续时间（秒，默认 0=持续到 stop）；"
                                       "到时自动收尾并释放腰部",
                    },
                    "amplitude_deg": {
                        "type": "number",
                        "minimum": _MIN_AMPLITUDE_DEG, "maximum": _MAX_AMPLITUDE_DEG,
                        "description": f"转身幅度（度，单侧，默认 {_DEFAULT_AMPLITUDE_DEG:g}）。"
                                       "首周期从 0 渐入；首次真机建议不超过 30 度",
                    },
                    "speed": {
                        "type": "string", "enum": list(_SPEED_PERIODS),
                        "description": "转身快慢两档：slow=慢（约 8 秒一个来回，默认），"
                                       "medium=中（约 4 秒一个来回）",
                    },
                    "confirm": {
                        "type": "boolean",
                        "description": "运行时校验：非 true 时卡片静默停止、不发出任何"
                                       "关节指令（info.preempted_by=confirm_missing 可查）；"
                                       "Agent 调用自动传布尔 true，字符串 \"true\" 亦认可",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "start": {
                        "params": ["duration_s", "amplitude_deg", "speed", "confirm"],
                        "description": "按参数开始摆动（confirm 在运行时校验，非 true 卡片"
                                       "静默停机不发指令）；再次调用会先收尾当前摆动再按"
                                       "新参数重启。要求机器人处于行走模式站立（workmode=2）",
                    },
                    "stop": {"params": [], "description": "幅度归零后停止并释放腰部"},
                    "info": {"params": [], "description": "状态、剩余时间、腰跟踪自检、"
                                                         "恢复结果"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "stop"},
                             "on_interrupt_all": {"action": "stop"}},
                "x-is-dangerous": True,
                "x-resource": ["waist"],
            },
            "topic_out": [],
        }

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Bundle lifecycle: watch workmode only, never start swaying.

        A card that moves a robot must not start moving because a container
        restarted. The supervisor thread exists so that vendor-App-triggered
        dances or falls still release an enabled sway; enabling itself is a
        person's action.
        """
        if self._supervisor_thread is not None:
            return
        self._supervisor_stop.clear()
        self._supervisor_thread = threading.Thread(
            target=self._supervisor_loop, daemon=True, name="bumi_waist_sway_supervisor")
        self._supervisor_thread.start()

    def stop(self):
        # Bundle teardown (SIGTERM/redeploy): no graceful wind-down — stop the
        # stream now, best-effort restore walking mode, join everything.
        self._supervisor_stop.set()
        with self._lock:
            self._enabled = False
            self._end_time = None
            self._generation += 1
        self._release(0.0, "plugin_stop")
        thread = self._supervisor_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._supervisor_thread = None

    # ── actions ──────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict):
        if action == "start":
            return self._start_action(args)
        if action == "stop":
            return self._stop_action()
        if action == "info":
            return self._info()
        return None

    def _start_action(self, args: dict) -> dict:
        # A 21-joint low-level stream must not execute on an action name
        # alone. The confirmation gate runs in the stream itself (runtime, by
        # the card owner's design): a start without an explicit true is
        # accepted, but the stream silences the card before its first joint
        # command — zero set_joint calls, no error — so only an explicit true
        # (the agent's boolean, or the canvas's string "true") ever moves the
        # robot.
        confirm = args.get("confirm")
        if isinstance(confirm, str) and confirm.strip().lower() == "true":
            confirm = True
        raw_duration = args.get("duration_s", 0)
        try:
            duration_s = float(raw_duration)
        except (TypeError, ValueError):
            return {"state": "error",
                    "error": f"duration_s 必须是数字，收到 {raw_duration!r}"}
        if isinstance(raw_duration, bool) or not math.isfinite(duration_s) \
                or not 0.0 <= duration_s <= _MAX_DURATION_S:
            return {"state": "error",
                    "error": f"duration_s 必须在 0 与 {_MAX_DURATION_S:g} 秒之间"
                             "（0=持续到 stop）"}

        raw_amplitude = args.get("amplitude_deg", _DEFAULT_AMPLITUDE_DEG)
        try:
            amplitude_deg = float(raw_amplitude)
        except (TypeError, ValueError):
            return {"state": "error",
                    "error": f"amplitude_deg 必须是数字，收到 {raw_amplitude!r}"}
        if isinstance(raw_amplitude, bool) or not math.isfinite(amplitude_deg) \
                or not _MIN_AMPLITUDE_DEG <= amplitude_deg <= _MAX_AMPLITUDE_DEG:
            return {"state": "error",
                    "error": f"amplitude_deg 必须在 {_MIN_AMPLITUDE_DEG:g} 与 "
                             f"{_MAX_AMPLITUDE_DEG:g} 度之间（单侧）"}

        speed = args.get("speed", _DEFAULT_SPEED)
        if speed not in _SPEED_PERIODS:
            return {"state": "error",
                    "error": f"speed 只能是 slow（慢）或 medium（中），收到 {speed!r}"}

        with self._lock:
            streaming = (self._stream_thread is not None
                         and self._stream_thread.is_alive())
            generation = self._generation
        if streaming:
            # A start while streaming replaces the running sway: quick
            # wind-down first, then the fresh parameters engage a new stream —
            # the same "a new instruction replaces the old one" rule as
            # `loco.move` restarting the chassis.
            self._release(_PREEMPT_WIND_DOWN_S, "restart")

        with self._lock:
            if generation != self._generation:
                # A stop or a body-action preemption landed while the old
                # stream was still winding down (an interrupt hook racing the
                # agent's restart is the realistic pairing). It wins; this
                # start is dropped instead of silently re-enabling over it.
                return {"state": self._state,
                        "message": "启动已被并发的 stop 或抢占取消"}
            self._enabled = True
            self._preempted_by = None
            self._last_error = None
            self._waist_tracking = "unknown"
            self._amplitude_rad = math.radians(amplitude_deg)
            self._period_s = _SPEED_PERIODS[speed]
            self._speed = speed
            self._confirmed = confirm is True
            self._end_time = (time.monotonic() + duration_s
                              if duration_s > 0 else None)
        return self._try_engage(generation)

    def _stop_action(self) -> dict:
        with self._lock:
            self._enabled = False
            self._generation += 1
            self._preempted_by = None
            self._end_time = None
        release = self._release(_WIND_DOWN_S, "stop")
        with self._lock:
            state = self._state
            error = self._last_error if state == "error" else None
        # Honest stop: a stream thread that refused to die leaves state
        # "error"; answering "off" here would tell the operator the waist is
        # free while a 500 Hz stream may still be live.
        result = {"state": state, "restore": self._restore_summary(release["restore"])}
        if error is not None:
            result["error"] = error
        return result

    # ── engage / release ─────────────────────────────────────────────────────

    def _try_engage(self, generation: int) -> dict:
        """Start the stream if the robot stands in walking mode and is free."""
        mode = self._read_mode()
        if mode is None:
            return self._engage_refused("cannot read workmode from the controller")
        if mode not in _ENGAGE_MODES:
            name = _WORKMODE_NAMES.get(mode, "unknown")
            return self._engage_refused(
                f"waist sway needs the robot standing in walking mode "
                f"(workmode=2); current mode is {mode} ({name}). Bring the robot "
                "to a standing walk mode first, then enable again.")
        with self._lock:
            if generation != self._generation:
                # The mode read above is an SDK call; a stop may have landed
                # during it. Never spawn a stream nobody asked for.
                return {"state": self._state,
                        "message": "启动已被并发的 stop 或抢占取消"}
            if self._stream_thread is not None and self._stream_thread.is_alive():
                self._state = "streaming"
                return {"state": "streaming"}
            if self._stream_thread is not None:
                # A previous release already stopped it but has not finished
                # its bookkeeping; reap it so two loops can never overlap.
                self._stream_thread.join(timeout=0.5)
                self._stream_thread = None
            self._state = "streaming"
            self._wind_down_end = None
            stop_event = threading.Event()
            self._stop_event = stop_event
            thread = threading.Thread(
                target=self._stream_loop, args=(stop_event,),
                daemon=True, name="bumi_waist_sway_stream")
            self._stream_thread = thread
        thread.start()
        print(f"[waist_sway] stream engaged (amplitude={self._amplitude_rad:.3f} rad, "
              f"period={self._period_s:g} s, rate={_RATE_HZ:g} Hz, "
              f"duration={'until stop' if self._end_time is None else 'timed'})",
              flush=True)
        return {"state": "streaming"}

    def _engage_refused(self, message: str) -> dict:
        with self._lock:
            self._enabled = False
            self._state = "error"
            self._last_error = message
        return {"state": "error", "enabled": False, "error": message}

    def _release(self, graceful_s: float, reason: str) -> dict:
        """Stop the stream thread (optionally winding amplitude down), then
        restore high-level walking mode if this card ever really streamed.

        Returns {'stopped': bool, 'restore': dict|None}; `stopped` is False
        only when the thread refused to die, and every caller must surface
        that instead of reporting a clean stop.
        """
        with self._lock:
            thread = self._stream_thread
            stop_event = self._stop_event
            alive = thread is not None and thread.is_alive()
            if alive and graceful_s > 0.0:
                self._wind_down_end = time.monotonic() + graceful_s
            elif alive:
                stop_event.set()
            else:
                self._stream_thread = None
        if not alive:
            # Nothing to join; restore first, then settle the state so a slow
            # mode-preparation sequence does not run under the lock.
            restore = self._restore(reason)
            with self._lock:
                self._settle_off()
            return {"stopped": True, "restore": restore}
        thread.join(timeout=graceful_s + _RELEASE_JOIN_S)
        if thread.is_alive():
            # The loop refused to stop (a blocking SDK call, most likely).
            # Insist once, and refuse to hand the waist back silently dirty.
            stop_event.set()
            thread.join(timeout=_RELEASE_FINAL_JOIN_S)
        with self._lock:
            if thread.is_alive():
                self._enabled = False
                self._state = "error"
                self._last_error = "stream thread did not stop during release"
                return {"stopped": False, "restore": None}
            self._stream_thread = None
            self._settle_off()
        return {"stopped": True, "restore": self._restore(reason)}

    def _settle_off(self) -> None:
        """After a release: keep errors visible, otherwise land on off.

        Caller holds the lock."""
        if self._state == "error":
            return
        if not self._enabled:
            self._state = "off"

    def _restore(self, reason: str) -> dict | None:
        """Hand the robot back to the high-level walking controller.

        Consumed exactly once per streaming session: `_restore_needed` is set
        by the first real `set_joint` and swapped atomically here, so the
        stream thread's own error path and a dispatch-thread release cannot
        both run a mode-preparation sequence.
        """
        with self._lock:
            if not self._restore_needed:
                return None
            self._restore_needed = False
        mode = self._read_mode()
        if mode == 2:
            result = {"state": "completed", "workmode": 2,
                      "note": "already in walking mode"}
        elif self._loco is not None and hasattr(self._loco, "prepare_walk_mode"):
            result = self._loco.prepare_walk_mode(f"waist_sway.{reason}")
        else:
            result = {"state": "skipped",
                      "error": "no loco plugin available to restore walking mode"}
        self._last_restore = {"reason": reason, **result}
        if isinstance(result, dict) and result.get("state") not in ("completed", "skipped"):
            print(f"[waist_sway] restore after {reason}: "
                  f"{result.get('state')} ({result.get('error', '')})", flush=True)
        return result

    # ── notifications from the sibling cards ─────────────────────────────────

    def disable_for_body_action(self, label: str) -> dict | None:
        """An action that needs the waist is about to run: stop now.

        Returns None when this card was neither enabled nor streaming, and
        otherwise a report of whether the release actually finished: a 500 Hz
        stream that refused to stop is a fact the caller's own result must
        carry (device.py surfaces it as `waist_release: "failed"`), not
        something this card absorbs. Mirrors
        `loco_servo.pause_for_explicit_command`.
        """
        with self._lock:
            was_active = self._enabled or (
                self._stream_thread is not None and self._stream_thread.is_alive())
            if not was_active:
                return None
            self._enabled = False
            self._generation += 1
            self._preempted_by = label
            self._end_time = None
        print(f"[waist_sway] disabled by {label}", flush=True)
        release = self._release(_PREEMPT_WIND_DOWN_S, label)
        report = {"preempted": True,
                  "release": "ok" if release["stopped"] else "failed"}
        if not release["stopped"]:
            report["error"] = ("stream thread did not stop; low-level joint "
                               "stream may still be live")
        self._last_preempt_restore = self._restore_summary(release["restore"])
        return report

    def notify_chassis_active(self, label: str) -> None:
        """A chassis card started driving the robot.

        The sway keeps streaming (the whole point is swaying while walking);
        the label only gates the tracking self-check, because a firmware that
        ignores the low-level stream *during* walking is an expected outcome,
        not a fault.
        """
        with self._lock:
            self._chassis_active = label

    def notify_chassis_idle(self, label: str) -> None:
        """A chassis card stopped driving the robot.

        The idle side never repeats the active side's action suffix —
        `loco.move` is ended by `loco.stop_move` (dispatch) or a lifecycle
        `loco.stop`, and `loco_servo.start` by `loco_servo.stop` — so
        matching whole strings leaves the label stuck and the tracking
        self-check permanently skipped. The tool prefix is the stable part
        and stays distinct between chassis cards (`loco` vs
        `loco_relative_move`), so an idle from one card cannot clear
        another card's active label.
        """
        tool = label.split(".")[0]
        with self._lock:
            if self._chassis_active is not None \
                    and self._chassis_active.split(".")[0] == tool:
                self._chassis_active = None

    # ── supervisor ───────────────────────────────────────────────────────────

    def _supervisor_loop(self) -> None:
        while not self._supervisor_stop.wait(1.0 / _SUPERVISOR_HZ):
            mode = self._read_mode()
            with self._lock:
                self._workmode = mode
                enabled = self._enabled
                state = self._state
                end_time = self._end_time
            if mode is not None and mode not in _ENGAGE_MODES:
                # Fail closed on every mode the engage gate would refuse, not
                # just the named body modes: an App/operator WALK->ready or
                # WALK->enabled edge can leave the robot lying down while a
                # 21-joint stand-pose stream keeps running, and no dispatch
                # hook sees an App-triggered transition. An unreadable mode
                # (None) is not a transition — the stream's own error paths
                # cover a dying SDK, and one flaky read must not tear down a
                # healthy sway.
                if enabled or state == "streaming":
                    self.disable_for_body_action(
                        f"workmode_{mode}_{_WORKMODE_NAMES.get(mode, 'unknown')}")
                continue
            if not enabled:
                continue
            if (end_time is not None and time.monotonic() > end_time
                    and state == "streaming"):
                # Duration elapsed: graceful wind-down and hand-back, the same
                # path an operator `stop` takes.
                with self._lock:
                    self._enabled = False
                    self._end_time = None
                self._release(_WIND_DOWN_S, "duration_elapsed")

    # ── the stream ───────────────────────────────────────────────────────────

    def _stream_loop(self, stop_event: threading.Event) -> None:
        with self._lock:
            confirmed = self._confirmed
        if not confirmed:
            # The confirmation gate, at runtime by design: without an explicit
            # true the card goes silent — not one joint command is sent, the
            # state settles to off, and no error is raised. `info` records
            # why, so a silenced start stays diagnosable without nagging.
            with self._lock:
                self._enabled = False
                self._state = "off"
                self._stream_thread = None
                self._preempted_by = "confirm_missing"
            print("[waist_sway] unconfirmed start silenced; no joint command sent",
                  flush=True)
            return
        low = self._ensure_low()
        if low is None:
            self._on_stream_error(
                f"LowController unavailable: {self._low_init_failed}")
            return
        table, waist = self._resolve_stand_table(low)
        if table is None:
            self._on_stream_error(
                "cannot map the vendor stand pose through getJointsIndex; "
                "refusing to command an unmapped joint table")
            return
        tick_s = 1.0 / _RATE_HZ
        start = time.perf_counter()
        measured = self._read_positions(low)
        if len(measured) != 21 or not all(math.isfinite(v) for v in measured):
            # Zero is the vendor's *lie* pose: a missing or malformed
            # measurement must not be synthesised into a commanded pose —
            # during the blend-in that would fold a standing robot. Refuse.
            self._on_stream_error(
                "cannot read a valid 21-joint initial state; refusing to engage")
            return
        start_pose = list(measured)
        tracking_bad_since: float | None = None
        try:
            while not stop_event.is_set():
                now = time.perf_counter()
                t = now - start
                with self._lock:
                    amplitude = self._amplitude_rad
                    sway_period = self._period_s
                    chassis_busy = self._chassis_active is not None
                    wind_down_end = self._wind_down_end
                scale = min(1.0, t / sway_period) if sway_period > 0 else 1.0
                blend = min(1.0, t / _BLEND_IN_S)
                if wind_down_end is not None:
                    remaining = (wind_down_end - time.monotonic()) / _WIND_DOWN_S
                    if remaining <= 0.0:
                        return
                    scale *= max(0.0, remaining)
                waist_cmd = table[waist] + amplitude * scale * math.sin(
                    2.0 * math.pi * t / sway_period if sway_period > 0 else 0.0)
                waist_cmd = max(-_WAIST_LIMIT_RAD, min(_WAIST_LIMIT_RAD, waist_cmd))

                fresh = self._read_positions(low)
                if len(fresh) == 21 and all(math.isfinite(v) for v in fresh):
                    measured = fresh

                cmds = []
                for i in range(21):
                    cmd = self._MotorCmd()
                    cmd.motor_id = i
                    if i == waist:
                        cmd.pos = float(waist_cmd)
                        cmd.vel = 0.0
                        cmd.kp = _HOLD_KP
                        cmd.kd = _HOLD_KD
                        cmd.tau = 0.0
                    else:
                        cmd.pos = float(start_pose[i] * (1.0 - blend)
                                        + table[i] * blend)
                        cmd.vel = 0.0
                        cmd.kp = _HOLD_KP
                        cmd.kd = _HOLD_KD
                        cmd.tau = 0.0
                    cmds.append(cmd)
                low.set_joint(cmds)
                with self._lock:
                    self._cmds_sent += 1
                    self._restore_needed = True
                # Self-check only while the chassis is idle: during walking the
                # firmware ignoring the stream is an expected outcome, not a
                # fault worth tearing the card down for.
                if not chassis_busy:
                    error = abs(float(measured[waist]) - waist_cmd)
                    if error > _TRACKING_TOLERANCE_RAD:
                        if tracking_bad_since is None:
                            tracking_bad_since = time.monotonic()
                        elif (time.monotonic() - tracking_bad_since
                              > _TRACKING_GRACE_S):
                            with self._lock:
                                self._waist_tracking = "not_tracking"
                            self._on_stream_error(
                                f"waist is not tracking (|measured-command| = "
                                f"{error:.2f} rad for over {_TRACKING_GRACE_S:g} s); "
                                "the firmware is likely ignoring the low-level "
                                "stream")
                            return
                    else:
                        tracking_bad_since = None
                        with self._lock:
                            self._waist_tracking = "ok"
                spent = time.perf_counter() - now
                if spent < tick_s:
                    time.sleep(tick_s - spent)
        except Exception as exc:                              # noqa: BLE001
            self._on_stream_error(f"{type(exc).__name__}: {exc}")

    def _on_stream_error(self, message: str) -> None:
        with self._lock:
            self._enabled = False
            self._state = "error"
            self._last_error = message
            self._stream_thread = None
        print(f"[waist_sway] stream error: {message}", flush=True)
        self._restore("stream_error")

    # ── SDK access ───────────────────────────────────────────────────────────

    def _ensure_low(self):
        """Init the LowController singleton once, on the stream thread.

        The import stays inside the function on purpose: no SDK exists on a
        laptop, and this file has to load there for its tests. init() timing
        is undocumented (the high-level twin can block), so it never runs on
        a dispatch thread.
        """
        if self._low is not None:
            return self._low
        if self._low_init_failed:
            return None
        try:
            from lowcontrol_py import LowController, MotorCmd
            low = LowController.instance()
            if not low.init():
                raise RuntimeError("LowController.init() returned false")
        except Exception as exc:                              # noqa: BLE001
            self._low_init_failed = f"{type(exc).__name__}: {exc}"
            print(f"[waist_sway] LowController init failed: "
                  f"{self._low_init_failed}", flush=True)
            return None
        self._low = low
        self._MotorCmd = MotorCmd
        return low

    def _resolve_stand_table(self, low):
        """Map the vendor stand pose into hardware-motor order, once.

        The example stores angles in its jointNames order and re-indexes every
        tick with getJointsIndex; resolving once at engage is the same mapping
        without the per-tick calls. A missing mapping is a hard refusal, not a
        zero table: zeros are the vendor's *lie* pose and would fold a
        standing robot.
        """
        try:
            table = [None] * 21
            seen: set[int] = set()
            for name, angle in zip(_SDK_JOINT_NAMES, _STAND_ANGLES):
                idx = int(low.getJointsIndex(name))
                if idx in seen:
                    print(f"[waist_sway] getJointsIndex maps multiple joint names "
                          f"to hardware index {idx}; refusing an ambiguous table",
                          flush=True)
                    return None, -1
                seen.add(idx)
                if 0 <= idx < 21:
                    table[idx] = angle
            if any(value is None for value in table):
                return None, -1
            waist = int(low.getJointsIndex(_WAIST_SDK_NAME))
            if not (0 <= waist < 21):
                return None, -1
            if waist != _EXPECTED_WAIST_HW_INDEX:
                print(f"[waist_sway] getJointsIndex('{_WAIST_SDK_NAME}')="
                      f"{waist}, driver table expected "
                      f"{_EXPECTED_WAIST_HW_INDEX}; commanding index {waist}",
                      flush=True)
            return [float(value) for value in table], waist
        except Exception as exc:                              # noqa: BLE001
            print(f"[waist_sway] getJointsIndex failed: {exc}", flush=True)
            return None, -1

    @staticmethod
    def _read_positions(low) -> list:
        """The 21 joint positions, or an empty list when the read fails."""
        try:
            state = low.get_joint_state()
            return [float(joint.pos) for joint in state]
        except Exception:                                     # noqa: BLE001
            return []

    def _read_mode(self) -> int | None:
        try:
            return int(self._high_ctrl.get_mode())
        except Exception:                                     # noqa: BLE001
            return None

    # ── info ─────────────────────────────────────────────────────────────────

    def _info(self) -> dict:
        with self._lock:
            mode = self._workmode
            remaining = (None if self._end_time is None
                         else max(0.0, self._end_time - time.monotonic()))
            return {
                "state": self._state,
                "amplitude_deg": round(math.degrees(self._amplitude_rad), 1),
                "speed": self._speed,
                "period_s": self._period_s,
                "remaining_s": (None if remaining is None else round(remaining, 1)),
                "preempted_by": self._preempted_by,
                "last_error": self._last_error,
                "last_restore": self._last_restore,
                "chassis_active": self._chassis_active,
                "workmode": ({"code": mode,
                              "name": _WORKMODE_NAMES.get(mode, "unknown")}
                             if mode is not None else None),
                "waist_tracking": self._waist_tracking,
                "cmds_sent": self._cmds_sent,
                "low_controller": ("ok" if self._low is not None
                                   else (self._low_init_failed or "not initialised")),
            }

    @staticmethod
    def _restore_summary(restore) -> dict | None:
        if not isinstance(restore, dict):
            return None
        return {key: restore.get(key) for key in ("state", "workmode", "error", "note")
                if key in restore}
