#!/usr/bin/env python3
"""Waist sway for Noetix Bumi — a standing patrol sway through the low-level joint stream.

`waist_sway` is a background toggle, not a one-shot action: while enabled the
robot keeps rotating its upper body left and right around the waist yaw, both
when standing still and while the chassis cards (`loco`, `loco_relative_move`,
`loco_servo`) are walking. Any action that needs the waist itself — the preset
animations, teaching, posture changes, protection and fall states — switches
the card off automatically, exactly like `loco` outranks `loco_servo`.

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
amplitude with a person holding the e-stop, `dry_run` cleared deliberately.

── arbitration, mirroring the `loco` / `loco_servo` split ─────────────────────

Chassis cards coexist with the sway by default (the whole point is swaying
while walking). `suspend_during_loco` flips that to pause-and-resume for
operators who want the chassis alone to own the robot while it moves. Body
cards never coexist: `LocoPlugin` notifies this card from its dispatch entry
points, and the supervisor also watches `workmode` so actions started from the
vendor App get the same release. An explicit instruction outranks a background
sway, never the other way round.
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

# The vendor standing hold. `_ECHO_*` are the vendor damping-mode gains,
# used for the experimental echo leg policy while the chassis walks.
_HOLD_KP = 10.0
_HOLD_KD = 0.5
_ECHO_KP = 0.0
_ECHO_KD = 0.1

# URDF waist_yaw_joint limit is ±1.57 rad; the operator ceiling stays below it.
_WAIST_LIMIT_RAD = 1.57
_MAX_AMPLITUDE_RAD = 1.2
_MIN_AMPLITUDE_RAD = 0.05
_MIN_PERIOD_S = 2.0
_MAX_PERIOD_S = 60.0
_MIN_RATE_HZ = 50.0
_MAX_RATE_HZ = 500.0

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
# the waist; observing any of them releases the sway and clears `enabled`.
_BODY_MODES = {5, 8, 9, 10, 11, 12, 14, 23, 26, 27, 28, 29, 30, 31, 32, 33}

_SUPERVISOR_HZ = 10.0
_BLEND_IN_S = 1.0        # measured pose -> stand table
_WIND_DOWN_S = 1.0       # operator stop: amplitude decays to zero
_PREEMPT_WIND_DOWN_S = 0.5   # body action / chassis suspend: snappier decay
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

        self._dry_run = bool(plugin_config.get("dry_run", True))
        self._amplitude_rad = self._clamp_number(
            plugin_config.get("amplitude_rad", 0.3),
            _MIN_AMPLITUDE_RAD, _MAX_AMPLITUDE_RAD, "amplitude_rad", 0.3)
        self._period_s = self._clamp_number(
            plugin_config.get("period_s", 6.0),
            _MIN_PERIOD_S, _MAX_PERIOD_S, "period_s", 6.0)
        self._rate_hz = self._clamp_number(
            plugin_config.get("rate_hz", 500.0),
            _MIN_RATE_HZ, _MAX_RATE_HZ, "rate_hz", 500.0)
        self._suspend_during_loco = bool(plugin_config.get("suspend_during_loco", False))
        self._concurrent_legs = plugin_config.get("concurrent_legs", "stand_hold")
        if self._concurrent_legs not in ("stand_hold", "echo"):
            print(f"[waist_sway] unknown concurrent_legs "
                  f"{self._concurrent_legs!r}; using stand_hold", flush=True)
            self._concurrent_legs = "stand_hold"

        self._lock = threading.RLock()
        self._enabled = False
        self._state = "off"          # off|waiting_chassis|suspended|streaming|error
        self._preempted_by: str | None = None
        self._last_error: str | None = None
        self._last_restore: dict | None = None
        self._last_preempt_restore: dict | None = None
        self._chassis_active: str | None = None
        self._wind_down_end: float | None = None
        self._cmds_sent = 0
        self._waist_tracking = "unknown"
        self._workmode: int | None = None
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
                "Bumi 腰部巡视摆动（实验性）。enabled=true 期间机器人持续小幅来回转动"
                "上身腰部 yaw：站立静止时摆动，loco / loco_relative_move / loco_servo 行走"
                "期间默认继续发送。semantic_action / action_recording / stand_up_lie_prone "
                "或保护、摔倒等占用腰部的动作一旦出现，卡片自动把 enabled 置为 false 并"
                "释放腰部。实现走 SDK 低层 set_joint（高层 API 无腰部通道，行走与上身动画"
                "在固件里互斥），以官方示例的站立角表做全 21 关节位置保持。注意：固件对"
                "高层行走与低层关节流并存的仲裁未知——若低层被忽略则行走正常、腰部停转；"
                "若低层生效则机器人保持站立、底盘命令可能失效。首次真机请保持小幅度、"
                "有人看护并确保 App 急停可用；默认 dry_run=true 只计数不驱动电机。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "enum": ["set", "start", "stop", "info", "config"],
                        "description": "set=开关摆动；start/stop=set true/false 的别名；"
                                       "info=查看状态；config=运行时改参数",
                    },
                    "enabled": {
                        "type": "boolean",
                        "description": "set 动作：true 开启持续摆动（机器人须处于行走模式"
                                       "站立），false 释放腰部并恢复高层行走模式",
                    },
                    "confirm": {
                        "type": "boolean",
                        "description": "config 动作关闭 dry_run 时必须传 true，确认允许"
                                       "真实电机输出",
                    },
                },
                "required": ["action"],
                "x-action-params": {
                    "set": {
                        "params": ["enabled"],
                        "description": "开启/关闭持续摆动。开启要求机器人已站立"
                                       "（workmode=2）；被占用腰部的动作抢占后会自动变回 "
                                       "false，info 里 preempted_by 记录原因",
                    },
                    "start": {"params": [], "description": "等同 set enabled=true"},
                    "stop": {"params": [], "description": "等同 set enabled=false：幅度归零"
                                                         "后停止并释放腰部"},
                    "info": {"params": [], "description": "状态、腰跟踪自检、恢复结果"},
                    "config": {
                        "params": ["amplitude_rad", "period_s", "rate_hz",
                                   "suspend_during_loco", "concurrent_legs",
                                   "dry_run", "confirm"],
                        "description": "运行时改参数，立即生效（rate_hz 下次开启生效）",
                    },
                },
                "x-hooks": {"on_interrupt_motion": {"action": "stop"},
                             "on_interrupt_all": {"action": "stop"}},
                "x-is-dangerous": True,
                "x-resource": ["waist"],
            },
            "configSchema": {
                "type": "object",
                "properties": {
                    "dry_run": {
                        "type": "boolean", "default": True,
                        "description": "空跑：照常读状态、构造并发计数，但不调用 SDK —— "
                                       "机器人不会动。真机确认场地与看护后在卡片上关闭"
                                       "（需 confirm=true，立即生效）",
                    },
                    "amplitude_rad": {
                        "type": "number", "default": 0.3,
                        "minimum": _MIN_AMPLITUDE_RAD, "maximum": _MAX_AMPLITUDE_RAD,
                        "description": "腰部摆动幅度（弧度，单侧）。硬上限 "
                                       f"{_MAX_AMPLITUDE_RAD}（URDF 限位 ±1.57），"
                                       "首周期从 0 渐入。首次真机建议 ≤0.3",
                    },
                    "period_s": {
                        "type": "number", "default": 6.0,
                        "minimum": _MIN_PERIOD_S, "maximum": _MAX_PERIOD_S,
                        "description": "一个完整来回的周期（秒）",
                    },
                    "rate_hz": {
                        "type": "number", "default": 500.0,
                        "minimum": _MIN_RATE_HZ, "maximum": _MAX_RATE_HZ,
                        "description": "低层指令发送频率，对齐官方示例 500 Hz；下次开启生效",
                    },
                    "suspend_during_loco": {
                        "type": "boolean", "default": False,
                        "description": "true 时底盘动作（loco / loco_relative_move / "
                                       "loco_servo）启动即暂停摆动、结束后自动恢复；"
                                       "false（默认）行走期间继续发送低层流 —— 并存行为"
                                       "属固件未知，见卡片描述",
                    },
                    "concurrent_legs": {
                        "type": "string", "default": "stand_hold",
                        "description": "并发期非腰关节策略：stand_hold=继续发站立角表"
                                       "（若固件低层生效，机器人保持站立、底盘命令可能"
                                       "失效，但不失稳）；echo=随实测位姿近零增益回发"
                                       "（唯一可能真正边走边转的档，但若固件是全局低层"
                                       "模式，腿部失撑可能摔倒 —— 仅吊架/看护下实验）",
                    },
                },
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
        self._release(0.0, "plugin_stop", full=True, target="off")
        thread = self._supervisor_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._supervisor_thread = None

    # ── actions ──────────────────────────────────────────────────────────────

    def dispatch(self, action: str, args: dict):
        if action == "set":
            return self._set_action(args)
        if action == "start":
            return self._set_enabled(True)
        if action == "stop":
            return self._set_enabled(False)
        if action == "info":
            return self._info()
        if action == "config":
            return self._config(args)
        return None

    def _set_action(self, args: dict) -> dict:
        raw = args.get("enabled")
        if not isinstance(raw, bool):
            return {"state": "error",
                    "error": "set requires enabled=true or enabled=false"}
        return self._set_enabled(raw)

    def _set_enabled(self, enabled: bool) -> dict:
        if enabled:
            with self._lock:
                self._enabled = True
                self._preempted_by = None
                self._last_error = None
                self._waist_tracking = "unknown"
            return self._try_engage()
        with self._lock:
            self._enabled = False
            self._preempted_by = None
        restore = self._release(_WIND_DOWN_S, "stop", full=True, target="off")
        return {"state": "off", "restore": self._restore_summary(restore)}

    # ── engage / release ─────────────────────────────────────────────────────

    def _try_engage(self) -> dict:
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
            if self._stream_thread is not None and self._stream_thread.is_alive():
                self._state = "streaming"
                return {"state": "streaming"}
            if self._stream_thread is not None:
                # A previous release already stopped it but has not finished
                # its bookkeeping; reap it so two loops can never overlap.
                self._stream_thread.join(timeout=0.5)
                self._stream_thread = None
            if self._suspend_during_loco and self._chassis_busy_now_locked():
                self._state = "waiting_chassis"
                return {"state": "waiting_chassis",
                        "message": "chassis card active and suspend_during_loco "
                                   "is on; the sway will engage once it is idle"}
            self._state = "streaming"
            self._wind_down_end = None
            stop_event = threading.Event()
            self._stop_event = stop_event
            thread = threading.Thread(
                target=self._stream_loop, args=(stop_event,),
                daemon=True, name="bumi_waist_sway_stream")
            self._stream_thread = thread
        thread.start()
        print(f"[waist_sway] stream engaged (dry_run={self._dry_run}, "
              f"amplitude={self._amplitude_rad} rad, period={self._period_s} s, "
              f"rate={self._rate_hz:g} Hz)", flush=True)
        return {"state": "streaming", "dry_run": self._dry_run,
                "amplitude_rad": self._amplitude_rad, "period_s": self._period_s}

    def _engage_refused(self, message: str) -> dict:
        with self._lock:
            self._enabled = False
            self._state = "error"
            self._last_error = message
        return {"state": "error", "enabled": False, "error": message}

    def _release(self, graceful_s: float, reason: str, *, full: bool,
                 target: str) -> dict | None:
        """Stop the stream thread (optionally winding amplitude down), then
        restore high-level walking mode if this card ever really streamed.

        `full=False` limits the restore to the fast path (mode already 2): it
        is used when a chassis action is about to run and must not have a
        mode-preparation sequence cancel rotations out from under it.
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
            restore = self._restore(reason, full=full)
            with self._lock:
                self._settle_state(target)
            return restore
        thread.join(timeout=graceful_s + 3.0)
        if thread.is_alive():
            # The loop refused to stop (a blocking SDK call, most likely).
            # Insist once, and refuse to hand the waist back silently dirty.
            stop_event.set()
            thread.join(timeout=2.0)
        with self._lock:
            if thread.is_alive():
                self._enabled = False
                self._state = "error"
                self._last_error = "stream thread did not stop during release"
                return None
            self._stream_thread = None
            self._settle_state(target)
        return self._restore(reason, full=full)

    def _settle_state(self, target: str) -> None:
        """After a release: keep errors visible, otherwise land on `target`.

        Caller holds the lock. An enabled card that released for a chassis
        action parks in `suspended`; a disabled one goes to the target state.
        """
        if self._state == "error":
            return
        if not self._enabled or target == "suspended":
            self._state = target

    def _restore(self, reason: str, *, full: bool) -> dict | None:
        """Hand the robot back to the high-level walking controller.

        Consumed exactly once per streaming session: `_restore_needed` is set
        by the first real `set_joint` and swapped atomically here, so the
        stream thread's own error path and a dispatch-thread release cannot
        both run a mode-preparation sequence.
        """
        with self._lock:
            if not self._restore_needed or self._dry_run:
                return None
            self._restore_needed = False
        mode = self._read_mode()
        if mode == 2:
            result = {"state": "completed", "workmode": 2,
                      "note": "already in walking mode"}
        elif not full:
            result = {"state": "skipped", "workmode": mode,
                      "note": "fast path only: not walking, leaving the mode "
                              "to the chassis action about to run"}
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

    def disable_for_body_action(self, label: str) -> bool:
        """An action that needs the waist is about to run: switch off now.

        Returns whether this card was enabled or streaming, so the caller can
        say so in its own result. Mirrors `loco_servo.pause_for_explicit_command`.
        """
        with self._lock:
            was_active = self._enabled or (
                self._stream_thread is not None and self._stream_thread.is_alive())
            if not was_active:
                return False
            self._enabled = False
            self._preempted_by = label
        print(f"[waist_sway] disabled by {label}", flush=True)
        restore = self._release(_PREEMPT_WIND_DOWN_S, label, full=True, target="off")
        self._last_preempt_restore = self._restore_summary(restore)
        return True

    def notify_chassis_active(self, label: str) -> None:
        """A chassis card started driving the robot."""
        with self._lock:
            if self._chassis_active == label:
                return
            self._chassis_active = label
            suspend = self._suspend_during_loco
            streaming = (self._stream_thread is not None
                         and self._stream_thread.is_alive())
        if suspend and streaming:
            print(f"[waist_sway] suspended for {label}", flush=True)
            self._release(_PREEMPT_WIND_DOWN_S, f"chassis:{label}",
                          full=False, target="suspended")

    def notify_chassis_idle(self, label: str) -> None:
        """A chassis card stopped driving the robot."""
        with self._lock:
            if self._chassis_active not in (None, label):
                return   # another chassis action still owns the slot
            self._chassis_active = None
            enabled = self._enabled
            state = self._state
        if enabled and state in ("waiting_chassis", "suspended"):
            self._try_engage()

    # ── supervisor ───────────────────────────────────────────────────────────

    def _supervisor_loop(self) -> None:
        while not self._supervisor_stop.wait(1.0 / _SUPERVISOR_HZ):
            mode = self._read_mode()
            with self._lock:
                self._workmode = mode
                enabled = self._enabled
                state = self._state
                suspend = self._suspend_during_loco
            if mode is not None and mode in _BODY_MODES:
                # Covers everything the dispatch hooks cannot see: preset
                # actions and teaching started from the vendor App, protection,
                # and fall transitions.
                if enabled or state == "streaming":
                    self.disable_for_body_action(
                        f"workmode_{mode}_{_WORKMODE_NAMES.get(mode, 'unknown')}")
                continue
            if not enabled:
                continue
            if state in ("waiting_chassis", "suspended"):
                if not self._chassis_busy_now():
                    self._try_engage()
            elif state == "streaming" and suspend and self._chassis_busy_now():
                self._release(_PREEMPT_WIND_DOWN_S, "chassis_busy",
                              full=False, target="suspended")

    def _chassis_busy_now(self) -> bool:
        with self._lock:
            if self._chassis_active is not None:
                return True
        loco = self._loco
        return bool(loco is not None and getattr(loco, "is_chassis_busy", lambda: False)())

    def _chassis_busy_now_locked(self) -> bool:
        # Caller holds self._lock (RLock); reuse the same facts.
        if self._chassis_active is not None:
            return True
        loco = self._loco
        return bool(loco is not None and getattr(loco, "is_chassis_busy", lambda: False)())

    # ── the stream ───────────────────────────────────────────────────────────

    def _stream_loop(self, stop_event: threading.Event) -> None:
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
        tick_s = 1.0 / self._rate_hz
        start = time.perf_counter()
        measured = self._read_positions(low) or [0.0] * 21
        start_pose = list(measured)
        tracking_bad_since: float | None = None
        try:
            while not stop_event.is_set():
                now = time.perf_counter()
                t = now - start
                with self._lock:
                    amplitude = self._amplitude_rad
                    sway_period = self._period_s
                    legs_mode = self._concurrent_legs
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
                if fresh is not None:
                    measured = fresh
                echo = chassis_busy and legs_mode == "echo"

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
                    elif echo:
                        # Experimental: near-zero gains tracking the measured
                        # pose, so a per-joint firmware merge leaves the legs
                        # to the walk controller. If the firmware is global
                        # low-mode this damps the legs instead of holding them.
                        cmd.pos = float(measured[i])
                        cmd.vel = 0.0
                        cmd.kp = _ECHO_KP
                        cmd.kd = _ECHO_KD
                        cmd.tau = 0.0
                    else:
                        cmd.pos = float(start_pose[i] * (1.0 - blend)
                                        + table[i] * blend)
                        cmd.vel = 0.0
                        cmd.kp = _HOLD_KP
                        cmd.kd = _HOLD_KD
                        cmd.tau = 0.0
                    cmds.append(cmd)
                if self._dry_run:
                    with self._lock:
                        self._cmds_sent += 1
                        self._waist_tracking = "dry_run"
                else:
                    low.set_joint(cmds)
                    with self._lock:
                        self._cmds_sent += 1
                        self._restore_needed = True
                    # Self-check only while idle: during concurrent walking the
                    # firmware ignoring the stream is an expected outcome, not
                    # a fault worth tearing the card down for.
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
        self._restore("stream_error", full=True)

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
            for name, angle in zip(_SDK_JOINT_NAMES, _STAND_ANGLES):
                idx = int(low.getJointsIndex(name))
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
    def _read_positions(low) -> list[float] | None:
        try:
            state = low.get_joint_state()
            return [float(joint.pos) for joint in state]
        except Exception:                                     # noqa: BLE001
            return None

    def _read_mode(self) -> int | None:
        try:
            return int(self._high_ctrl.get_mode())
        except Exception:                                     # noqa: BLE001
            return None

    # ── config / info ────────────────────────────────────────────────────────

    _NUMERIC_FIELDS = {
        "amplitude_rad": (_MIN_AMPLITUDE_RAD, _MAX_AMPLITUDE_RAD),
        "period_s": (_MIN_PERIOD_S, _MAX_PERIOD_S),
        "rate_hz": (_MIN_RATE_HZ, _MAX_RATE_HZ),
    }

    def _config(self, args: dict) -> dict:
        changed = {}
        with self._lock:
            if ("dry_run" in args and self._dry_run
                    and not bool(args["dry_run"])
                    and args.get("confirm") is not True):
                return {"ok": False, "state": self._state,
                        "error": "关闭 dry_run 需要确认：场地无人、有人看护、"
                                 "App 急停可用时传 confirm=true"}
            for field, (low_bound, high_bound) in self._NUMERIC_FIELDS.items():
                if field not in args:
                    continue
                try:
                    value = float(args[field])
                except (TypeError, ValueError):
                    return {"ok": False, "error": f"{field} 必须是数字"}
                if not math.isfinite(value) or not low_bound <= value <= high_bound:
                    return {"ok": False,
                            "error": f"{field} 必须在 {low_bound} 与 {high_bound} 之间"}
                attr = f"_{field}"
                if getattr(self, attr) != value:
                    setattr(self, attr, value)
                    changed[field] = value
            if "dry_run" in args:
                new = bool(args["dry_run"])
                if self._dry_run != new:
                    self._dry_run = new
                    changed["dry_run"] = new
            if "suspend_during_loco" in args:
                new = bool(args["suspend_during_loco"])
                if self._suspend_during_loco != new:
                    self._suspend_during_loco = new
                    changed["suspend_during_loco"] = new
            if "concurrent_legs" in args:
                new = args["concurrent_legs"]
                if new not in ("stand_hold", "echo"):
                    return {"ok": False,
                            "error": "concurrent_legs 只能是 stand_hold 或 echo"}
                if self._concurrent_legs != new:
                    self._concurrent_legs = new
                    changed["concurrent_legs"] = new
            state = self._state
        if changed:
            print(f"[waist_sway] config changed while {state}: "
                  + ", ".join(f"{k}={v}" for k, v in changed.items()), flush=True)
        return {"ok": True, "state": state, **changed,
                "note": "rate_hz 在下次开启时生效" if "rate_hz" in changed else ""}

    def _info(self) -> dict:
        with self._lock:
            mode = self._workmode
            return {
                "state": self._state,
                "enabled": self._enabled,
                "preempted_by": self._preempted_by,
                "last_error": self._last_error,
                "last_restore": self._last_restore,
                "dry_run": self._dry_run,
                "amplitude_rad": self._amplitude_rad,
                "period_s": self._period_s,
                "rate_hz": self._rate_hz,
                "suspend_during_loco": self._suspend_during_loco,
                "concurrent_legs": self._concurrent_legs,
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

    @staticmethod
    def _clamp_number(raw, low_bound, high_bound, name, default):
        try:
            value = float(raw)
        except (TypeError, ValueError):
            return default
        if not math.isfinite(value):
            return default
        return max(low_bound, min(high_bound, value))
