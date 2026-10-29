#!/usr/bin/env python3
"""Continuous joint control for the RM75 — the first consumer of motus.control/1.

`joint_control` in device.py is the call-shaped path: one absolute target per
`tools/call`, an ACP completion, a `confirm_motion` per request. That is right
for an LLM deciding to move an arm somewhere, and wrong for an execution model,
which produces tens of commands per second and is not asking a question each
time.

This card is the stream-shaped path. It subscribes to a `control/joint` topic,
runs every message through `common.control.ControlSink`, and passes what
survives to the RealMan passthrough API (`rm_movej_canfd`). The arm's safety
properties come from the sink, not from this file: freshness, arbitration
between sources, step clamping, hard limits, watchdog, and escalation all live
there and are tested without a robot.

What is specific to this arm, and therefore here:

- **Units.** The descriptor is in radians, matching the skeleton publisher this
  driver already produces. The SDK passthrough takes degrees. The conversion
  happens in one place, at the boundary, immediately before the SDK call.
- **The motion gate.** `RM_MOTION_ENABLED` and an explicit confirmation guard
  every movement in this driver. A card that streams commands does not get to
  skip that because the commands are frequent — it is checked once at `start`,
  which is the point where a human is present.
- **Stopping.** `rm_set_arm_slow_stop` is what the watchdog and abort paths
  call, the same call the existing plugin's interrupt hook uses.
"""

from __future__ import annotations

import json
import math
import threading
import time

from common.control import ControlSink, Verdict, parse_descriptor
from common.vendor_runtime import action_schema, tool

from device import JOINT_LIMITS_DEG, JOINT_MAX_SPEED_DEG_S, JOINT_NAMES

# Passthrough rate. The vendor requires a period under 10 ms for high-follow
# mode; we run low-follow at a much calmer rate, so `follow=False`.
DEFAULT_EXPECTED_HZ = 30.0
MAX_HZ = 100.0
WATCHDOG_MS = 200
# An observation older than this cannot be acted on. Remote inference routinely
# produces a command generated just now from a much older picture; see
# README_dev § "Continuous Control".
MAX_OBS_AGE_MS = 300


def build_descriptor(expected_hz: float = DEFAULT_EXPECTED_HZ) -> dict:
    """The action space this arm accepts, in radians.

    Derived from the same vendor limits `joint_control` enforces, so the two
    paths cannot disagree about what this arm can do.
    """
    lower = [math.radians(low) for low, _ in JOINT_LIMITS_DEG]
    upper = [math.radians(high) for _, high in JOINT_LIMITS_DEG]
    max_velocity = [math.radians(speed) for speed in JOINT_MAX_SPEED_DEG_S]
    # One step may not exceed what the joint could travel in one period at its
    # rated speed. Anything larger is a jump the arm cannot follow anyway; the
    # sink clamps it rather than refusing, because it is usually one noisy
    # sample rather than a mistake.
    period = 1.0 / expected_hz
    max_delta = [speed * period for speed in max_velocity]
    return {
        "control_interface": "motus.control/1",
        "mode": "joint_position",
        "dof": len(JOINT_NAMES),
        "joint_names": list(JOINT_NAMES),
        "units": {"angle": "rad", "time": "s"},
        "limits": {
            "lower": lower,
            "upper": upper,
            "max_velocity": max_velocity,
            "max_delta_per_step": max_delta,
        },
        "frame": "base_link",
        "rate": {
            "max_hz": MAX_HZ,
            "expected_hz": expected_hz,
            "watchdog_ms": WATCHDOG_MS,
            "max_obs_age_ms": MAX_OBS_AGE_MS,
        },
        # Declared as null rather than omitted: this arm has no force-torque
        # sensing wired to this driver, and a missing protection should be
        # visible instead of assumed. See common/control/descriptor.py.
        "force_torque": None,
        "urdf_ref": "model",
    }


class RM75ServoPlugin:
    """Stream-shaped joint control. One card, one input topic, one sink."""

    PREFIX = "servo"

    def __init__(self, client, config, namespace="rm75", ros2=None):
        self.client = client
        self._ros2 = ros2
        self._namespace = namespace.strip("/") or "rm75"
        servo_config = config.get("servo", {}) or {}
        self._expected_hz = float(servo_config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not math.isfinite(self._expected_hz) or not 0 < self._expected_hz <= MAX_HZ:
            raise ValueError(f"servo.expected_hz must be in (0, {MAX_HZ}]")
        self._descriptor_raw = build_descriptor(self._expected_hz)
        self._descriptor = parse_descriptor(self._descriptor_raw)

        # `start`/`stop`/`config` arrive on separate threads (ThreadingHTTPServer).
        # The lock guards the bookkeeping only — never a start(), a stop() or an
        # SDK call, or a stop would queue behind the start it is meant to cancel.
        self._lock = threading.RLock()
        self._sink: ControlSink | None = None
        self._node = None
        self._input_topic = ""
        self._running = False
        self._last_outcome: dict | None = None
        self._rejects: list[str] = []
        # Paused means subscribed but not applying. See `_halt`.
        self._paused = False
        self._motion_gate_held = False

    # ── tools ────────────────────────────────────────────────────────────────

    def get_tools(self):
        schema = action_schema(
            {
                "pause": ([], "Stop driving the arm and hold the current pose, "
                              "staying subscribed; resume continues"),
                "resume": ([], "Continue applying commands"),
            },
            {
                "input_topic": {"type": "string",
                                "description": "control/joint topic to consume"},
            },
        )
        # `start`/`stop`/`info` are deliberately absent from the action list
        # above, and that is what decides who may call them: agent-core splits a
        # tool into one LLM-callable function per action (mcp_client.py
        # `_to_openai_schema`), so an action not listed is reachable by the
        # canvas and not by the model. `stop` belongs to the project lifecycle —
        # a model calling it would take this card out of a running project
        # without the project knowing, and could not put it back, since `start`
        # needs the input topic and the downstream descriptor that only
        # agent-core has.
        #
        # `pause` is therefore the model's halt, and what the interrupt hooks
        # fire. It is not a weaker stop: the arm stops just as immediately. It
        # differs in staying subscribed, so the project's view of what is
        # running stays true and `resume` can continue.
        schema["x-hooks"] = {"on_interrupt_motion": {"action": "pause"},
                             "on_interrupt_all": {"action": "pause"}}
        schema["x-is-dangerous"] = True
        # No x-completion: this card has no end. It streams until stopped, so
        # there is nothing for an ACP pending action to wait for — holding one
        # open would block every other actuator for as long as the card runs.
        schema["x-resource"] = "arm"
        return [
            tool(
                "servo",
                "actuator",
                "Continuous RM75 joint control from a motus.control/1 stream "
                f"(passthrough at up to {self._expected_hz:g} Hz)",
                schema,
                topic_in=[{"format": "control/joint",
                           "desc": "motus.control/1 joint_position commands"}],
            )
        ]

    def dispatch(self, action, args):
        if action == "start":
            return self._start(args)
        if action == "stop":
            return self._stop()
        if action == "pause":
            return self._halt(True)
        if action == "resume":
            return self._halt(False)
        if action == "info":
            return self._info()
        return None

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Bundle lifecycle. Deliberately does nothing.

        A card that streams motion must not come up streaming because the
        container restarted. It starts when agent-core starts the project,
        which is an operator action, and not before.
        """

    def stop(self):
        self._stop()

    # ── actions ──────────────────────────────────────────────────────────────

    def _start(self, args):
        if not self.client.motion_enabled:
            return {"state": "error",
                    "message": "RM_MOTION_ENABLED is not set; this driver is read-only"}
        # Arguments before environment: a caller who left out the topic should
        # be told that, whether or not the arm happens to be plugged in.
        topic = (args.get("input_topic") or "").strip()
        if not topic:
            topic = (args.get("input_topics") or [""])[0].strip()
        if not topic:
            return {"state": "error",
                    "message": "input_topic is required — wire a control/joint "
                               "source to this card on the canvas"}

        if not self.client.connected:
            return {"state": "error", "message": "RM75 SDK is not connected"}
        if self._ros2 is None:
            return {"state": "error", "message": "no ROS context; cannot subscribe"}
        sink = ControlSink(
            self._descriptor,
            self._apply,
            on_watchdog=self._slow_stop,
            on_abort=self._slow_stop,
        )
        # Check, acquire, reserve and subscribe under one lock. This prevents
        # a duplicate start from leaking the shared gate and prevents stop()
        # from racing between gate acquisition and _motion_gate_held=true.
        with self._lock:
            if self._running:
                return {"state": "error", "message": f"already running on {self._input_topic}"}
            if not self.client.motion_gate.acquire(blocking=False):
                return {"state": "error", "message": "another arm operation is active"}
            self._sink = sink
            self._input_topic = topic
            self._running = True
            self._paused = False
            self._motion_gate_held = True
            try:
                self._subscribe(topic)
            except Exception as exc:
                self._running = False
                self._sink = None
                self._input_topic = ""
                self._motion_gate_held = False
                self.client.motion_gate.release()
                return {"state": "error", "message": f"subscribe failed: {exc}"}

        print(f"[rm75] servo streaming from {topic}", flush=True)
        return {"state": "running", "input": topic,
                "control_interface": self._descriptor_raw}

    def _halt(self, halted: bool):
        """`pause` and `resume`. Stops the arm; keeps the subscription."""
        with self._lock:
            if not self._running:
                return {"state": "idle", "message": "card is not running"}
            self._paused = bool(halted)
        if halted:
            self._slow_stop()
        return {"state": "paused" if halted else "running",
                "input": self._input_topic}

    def _stop(self):
        with self._lock:
            node, self._node = self._node, None
            self._sink = None
            was_running = self._running
            self._running = False
            topic, self._input_topic = self._input_topic, ""

        try:
            if node is not None:
                try:
                    self._ros2.executor_core.remove_node(node)
                finally:
                    node.destroy_node()
            if was_running:
                self._slow_stop()
        finally:
            # Teardown can fail; the shared gate must be released even then.
            with self._lock:
                gate_held = self._motion_gate_held
                self._motion_gate_held = False
            if gate_held:
                self.client.motion_gate.release()
        if was_running:
            print(f"[rm75] servo stopped ({topic})", flush=True)
        return {"state": "idle"}

    def _info(self):
        with self._lock:
            sink = self._sink
            running = self._running
            topic = self._input_topic
            last = dict(self._last_outcome) if self._last_outcome else None
            rejects = list(self._rejects)
            paused = self._paused
        return {
            # Paused is its own state, not running: an operator reading
            # "running" beside a motionless arm goes looking for a fault that
            # is not there.
            "state": ("paused" if paused else "running") if running else "idle",
            "input": topic,
            "control_interface": self._descriptor_raw,
            # The window this card commits to the controller before it can be
            # revoked. One passthrough period: the arm is never holding more
            # than a single point, so e-stop latency is bounded by the period
            # rather than by a chunk length. See README_dev § "Continuous Control".
            "committed_window_ms": int(1000.0 / self._expected_hz),
            "motion_enabled": self.client.motion_enabled,
            "sink": sink.stats() if sink is not None else None,
            "last_outcome": last,
            # Rejections are contract or limit violations — somebody wired
            # something wrong — so they are surfaced, unlike drops.
            "recent_rejects": rejects,
        }

    # ── the stream ───────────────────────────────────────────────────────────

    def _subscribe(self, topic: str):
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,                      # a queued command is a stale command
            durability=DurabilityPolicy.VOLATILE,
        )
        node = Node("rm75_servo", context=self._ros2.ctx_core)
        node.create_subscription(String, topic, self._on_message, qos)
        # The watchdog needs a heartbeat of its own: silence produces no
        # callbacks, and silence is exactly what it exists to notice.
        node.create_timer(WATCHDOG_MS / 2000.0, self._tick)
        self._ros2.executor_core.add_node(node)
        with self._lock:
            self._node = node

    def _on_message(self, message):
        try:
            payload = json.loads(message.data)
        except Exception as exc:
            self._record(Verdict.REJECTED.value, f"undecodable payload: {exc}")
            return
        # Hold the lifecycle lock through admission and application. stop()
        # and pause() therefore cannot issue slow-stop and return while an
        # already-admitted callback is still able to command the arm.
        with self._lock:
            sink = self._sink
            if sink is None or self._paused:
                # Dropped, not queued: a command held through a pause was
                # computed from a world that has moved on.
                return
            outcome = sink.submit(payload)
        self._record(outcome.verdict.value, outcome.reason, outcome.warnings)

    def _tick(self):
        with self._lock:
            sink = self._sink
            if sink is None or self._paused:
                return
            outcome = sink.tick()
        if outcome is not None:
            self._record(outcome.verdict.value, outcome.reason)

    def _record(self, verdict: str, reason: str = "", warnings=None):
        entry = {"verdict": verdict, "reason": reason,
                 "at_ms": int(time.time() * 1000)}
        if warnings:
            entry["warnings"] = list(warnings)
        with self._lock:
            self._last_outcome = entry
            if verdict in (Verdict.REJECTED.value, Verdict.ABORTED.value):
                self._rejects.append(f"{verdict}: {reason}")
                del self._rejects[:-10]

    # ── the arm ──────────────────────────────────────────────────────────────

    def _apply(self, values, gripper):
        """Only reached by commands that passed every check in the sink."""
        degrees = [math.degrees(value) for value in values]
        # follow=False — low-follow mode. High follow requires a period under
        # 10 ms, which this card does not promise.
        self.client.command("rm_movej_canfd", degrees, False)

    def _slow_stop(self):
        if not self.client.connected:
            return
        try:
            self.client.command("rm_set_arm_slow_stop")
        except Exception as exc:
            print(f"[rm75] servo stop failed: {exc}", flush=True)
