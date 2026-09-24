#!/usr/bin/env python3
"""Continuous chassis control for Noetix Bumi — the `motus.control/1` twist sink.

`loco` in device.py is the call-shaped path: one `tools/call` per motion, chosen
by a person or an LLM. Right for "walk forward for two seconds", wrong for a
navigation policy, which emits a velocity ten times a second and is not waiting
for an answer to each one. Same split as R1's `loco` / `loco_servo` and G1's
`arm` / `servo`; the reasoning is in `unitree/r1/loco_servo.py`.

This card is the stream-shaped path, and the action space is the same six
numbers in the same body frame: `[vx, vy, vz, wx, wy, wz]`. Bumi actuates three.

── twist is a velocity space, and the sink was built for position spaces ─────

`ControlSink`'s chain is unchanged but two limit fields change meaning:

* `limits.lower/upper` bound the **velocity** — still exactly right.
* `max_delta_per_step` is the difference between consecutive commands, so here
  it is an **acceleration** cap. Also right, and the field that keeps a policy
  from kicking the chassis.
* `max_velocity` would be delta over dt, i.e. **jerk**, which has no useful
  meaning. Deliberately not declared — `parse_descriptor` takes it as optional.

── two things Bumi does differently from R1, both structural ─────────────────

**1. The chassis command is normalised, not metric.** `publish_cmd(x, y, z, …)`
takes three numbers that the vendor's own example feeds straight from a
joystick's axes, and the existing `loco` card declares them `[-1, 1]`. The
protocol requires m/s and rad/s, so this file owns the conversion — and the
conversion factor is a **measurement nobody has taken**. See `Calibration`.

**2. A command does not persist.** R1's `Move(…, True)` sets a velocity that
stands until `StopMove`. `publish_cmd` is a single DDS message: the vendor
example and `LocoPlugin._do_move` both keep a thread republishing at 50 Hz. So
this card carries a repeat thread, and "hold" means *keep publishing zero*
rather than stop publishing. That is the one way a chassis differs from every
arm card in this repo — an arm holds by being left alone, a chassis keeps going.

── one body, several cards ───────────────────────────────────────────────────

`loco`, `stand_up_lie_prone`, `semantic_action` and `action_recording` all move
the same robot, and the last three also change `workmode` out from under a
running stream. A matching `x-resource` does **not** keep them apart — ACP
arbitrates its own scheduling, not several routes to one set of motors. So every
one of them pauses this card before it acts, and this card refuses to start
while `loco` has a motion in flight. An explicit instruction from a person
outranks a running policy, never the other way round.
"""

from __future__ import annotations

import json
import threading
import time

from common.control import ControlSink, parse_descriptor

AXIS_NAMES = ["vx", "vy", "vz", "wx", "wy", "wz"]
DOF = 6

# ── the calibration, and why it is a config block rather than five constants ─
#
# Everything below describes **this chassis' velocity space**, and not one of
# these numbers has been measured on a Bumi. They are grouped because they are
# measured together, in one session, with the robot walking — and because a
# consumer needs to be told, as one fact, that the space it is commanding into
# is an estimate.
#
# `full_scale_*` answer "what does x = 1.0 mean in m/s". Chosen to be plausible
# for a ~0.95 m humanoid and **deliberately low**: reading the full scale as
# slower than it is makes the card ask for a smaller normalised number than it
# meant, so the robot under-runs the policy. Reading it as faster does the
# opposite, and a chassis that travels at twice the commanded speed towards a
# person is the failure that is not recoverable.
#
# `min_magnitude*` are the deadband — below these a legged robot does nothing at
# all, because from a standstill it has to assemble a whole gait cycle and there
# is no "creep slowly" regime the way a wheeled base has. **This number's
# failure modes are not one-sided**, which is why it is declared as an estimate
# rather than left at zero: too high and every small correction overshoots, too
# low and the policy's commands are accepted, counted, and produce no motion.
# Zero is not the safe choice here, it is the claim that this robot can creep.
#
# On R1 the same deadband collapsed by 20x once the robot was already walking
# (1.0 rad/s standing, 0.05 rad/s mid-stride), which is why there are two.
# Bumi's have not been measured either way; the moving figures are the standing
# ones until somebody walks the robot, because inventing a smaller one would be
# the same mistake in the other direction.
#
# README.md § "标定 loco_servo" has the procedure. It is about twenty minutes.
DEFAULTS = {
    "full_scale_vx_mps": 0.5,
    "full_scale_vy_mps": 0.3,
    "full_scale_wz_rads": 1.0,
    "min_vx_mps": 0.15,
    "min_vy_mps": 0.15,
    "min_wz_rads": 0.3,
    "min_wz_moving_rads": 0.3,
}

# `measured` | `estimate`. Rides along in `control_interface` so the policy
# upstream can say so in its own `degraded` list — it has no other way to find
# out that the velocity space it is commanding into is a guess.
CALIBRATION_SOURCES = ("estimate", "measured")

# Acceleration caps, per step at `expected_hz`, in m/s and rad/s. Conservative
# and chosen here rather than derived from the hardware maximum: the SDK will
# accept a jump from rest to full speed, and "the chassis can survive it" is not
# the same statement as "a policy may ask for it".
VX_ACCEL = 0.15
VY_ACCEL = 0.15
WZ_ACCEL = 0.30

# Pinned axes still need a positive `max_delta_per_step` — `parse_descriptor`
# rejects zero, and their real bound is the `lower == upper == 0` above anyway.
PINNED_ACCEL = 1e-6

# ── the space this robot occupies ────────────────────────────────────────────
#
# Declared for the same reason the deadband is: it is a fact about the robot,
# and a policy that hard-codes it is a policy that is wrong on the next chassis.
# navi's avoidance corridor is metric, so it needs this to turn a range of image
# columns into "is there room for my shoulders".
#
# **`source: "estimate"`, and it matters that it says so.** `resource/
# bumi_model.urdf` has had its meshes stripped, so there is no envelope anywhere
# in this repository — this half-width is inferred from the shoulder joint
# origins (+-0.084 m) plus an arm. navi's `_adopt_footprint` reads the `source`
# field and repeats the caveat in its `degraded` list.
#
# Unlike the deadband, every failure mode of *this* number is one-sided:
# believing the robot is wider than it is costs some unnecessary slowing,
# believing it is narrower puts a shoulder into a doorframe. So it errs wide.
FOOTPRINT_HALF_WIDTH = 0.22      # m — arms at rest, estimated, erring wide
FOOTPRINT_FRONT = 0.12
FOOTPRINT_REAR = 0.12
FOOTPRINT_HEIGHT = 0.95

DEFAULT_EXPECTED_HZ = 10.0
MAX_HZ = 20.0
# Generous next to an arm's 200 ms because a chassis at 0.4 m/s travels 12 cm in
# this window, and because the upstream is a perception pipeline whose frame
# cadence is lumpier than a policy's.
WATCHDOG_MS = 300
MAX_OBS_AGE_MS = 500

# How often the repeat thread re-sends the standing target. Matches
# `LocoPlugin._do_move`, which is what the vendor's own example does.
REPEAT_HZ = 50.0

# workmode 2 — walking. Anything else and the robot is disabled, getting ready,
# mid-preset-action, or in protection.
WALKING_MODE = 2
PROTECTION_MODE = 26

# How long a workmode reading stays good for. See `_posture_problem`.
MODE_CACHE_S = 1.0

# The largest normalised value the SDK takes on any axis.
FULL_SCALE = 1.0


class CalibrationError(ValueError):
    """A velocity calibration that would silently misdrive the chassis."""


class Calibration:
    """m/s and rad/s in, normalised `publish_cmd` arguments out.

    Kept as a small object rather than three floats so that the descriptor, the
    conversion and the "is this measured" declaration cannot drift apart — the
    limits a policy is told about have to be the same numbers the commands are
    divided by, or the policy is clamped against a ceiling that does not exist.
    """

    def __init__(self, config: dict = None):
        config = config or {}
        values = {}
        for key, default in DEFAULTS.items():
            raw = config.get(key, default)
            try:
                value = float(raw)
            except (TypeError, ValueError):
                raise CalibrationError(f"loco_servo.{key} must be a number, got {raw!r}")
            if not value > 0:
                # Zero is rejected rather than read as "no limit". A full scale
                # of zero would divide by zero; a deadband of zero would claim
                # this legged robot can creep, which is the claim that makes a
                # policy's small corrections vanish without a trace.
                raise CalibrationError(
                    f"loco_servo.{key} must be positive, got {value!r} — "
                    "zero is not 'no threshold' here, see DEFAULTS")
            values[key] = value
        self._values = values

        source = str(config.get("calibration_source", "estimate"))
        if source not in CALIBRATION_SOURCES:
            raise CalibrationError(
                f"loco_servo.calibration_source must be one of "
                f"{', '.join(CALIBRATION_SOURCES)}, got {source!r} — whether "
                "these numbers were measured is most of what says whether the "
                "robot will do what the policy asked")
        self.source = source

        for axis, floor, ceiling in (
            ("vx", "min_vx_mps", "full_scale_vx_mps"),
            ("vy", "min_vy_mps", "full_scale_vy_mps"),
            ("wz", "min_wz_rads", "full_scale_wz_rads"),
        ):
            if values[floor] >= values[ceiling]:
                # A floor at or above the ceiling leaves exactly one commandable
                # speed, which is a switch and not a controller. It also makes
                # `_lift` in the policy produce a value the sink then rejects on
                # the hard limit, which reads as "every command refused".
                raise CalibrationError(
                    f"loco_servo: {floor}={values[floor]:g} is not below "
                    f"{ceiling}={values[ceiling]:g} — the {axis} axis would have "
                    "a single commandable speed")

    def __getitem__(self, key: str) -> float:
        return self._values[key]

    @property
    def measured(self) -> bool:
        return self.source == "measured"

    def to_normalised(self, vx: float, vy: float, wz: float) -> tuple:
        """Three metric axes as the `[-1, 1]` arguments `publish_cmd` takes.

        Clamped, because the descriptor's `upper` is the full scale and the sink
        has already enforced it — a value past `1.0` here would mean the two
        disagree, and sending it on would let the SDK decide what a chassis does
        with an out-of-range stick.
        """
        return (_clamp(vx / self._values["full_scale_vx_mps"], FULL_SCALE),
                _clamp(vy / self._values["full_scale_vy_mps"], FULL_SCALE),
                _clamp(wz / self._values["full_scale_wz_rads"], FULL_SCALE))

    def as_dict(self) -> dict:
        out = dict(self._values)
        out["calibration_source"] = self.source
        return out


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _step_limit(accel: float, floor: float) -> float:
    """The acceleration cap, widened to at least the axis' deadband.

    **An acceleration cap finer than the deadband is not a cap — it is dead
    time.** The step clamp applies to the command the policy sends, so a
    0.30 rad/s cap against a 1.0 rad/s floor ramps 0.30 → 0.60 → 0.90 → 1.0 and
    the robot executes precisely none of the first three: three ticks of
    silence, then the turn starts at full speed. That asymmetry — slow to start,
    instant to stop, because the ramp *down* crosses the floor on its first
    step — is what a lurch is made of, and nothing reports it, because from the
    sink's side every command was accepted.

    Shaping acceleration below the floor is the gait controller's job, not ours.
    """
    return max(accel, floor)


def build_descriptor(calibration: Calibration,
                     expected_hz: float = DEFAULT_EXPECTED_HZ) -> dict:
    """The action space, in SI units, whatever the SDK happens to take.

    A pure function over the calibration so it can be loaded and asserted
    without rclpy, the Noetix SDK or a robot — same shape as R1's.
    """
    vx = calibration["full_scale_vx_mps"]
    vy = calibration["full_scale_vy_mps"]
    wz = calibration["full_scale_wz_rads"]
    min_vx = calibration["min_vx_mps"]
    min_vy = calibration["min_vy_mps"]
    min_wz = calibration["min_wz_rads"]
    min_wz_moving = calibration["min_wz_moving_rads"]
    return {
        "control_interface": "motus.control/1",
        "mode": "twist",
        "dof": DOF,
        "joint_names": list(AXIS_NAMES),
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {
            # The full stick, in metric. Asking for more than this is not a
            # thing the chassis can be told, so it is a hard limit rather than
            # something quietly clamped.
            "lower": [-vx, -vy, 0.0, 0.0, 0.0, -wz],
            "upper": [vx, vy, 0.0, 0.0, 0.0, wz],
            # Acceleration, not velocity — see the module docstring. Never finer
            # than the deadband on the same axis; see `_step_limit`.
            "max_delta_per_step": [_step_limit(VX_ACCEL, min_vx),
                                   _step_limit(VY_ACCEL, min_vy),
                                   PINNED_ACCEL, PINNED_ACCEL, PINNED_ACCEL,
                                   _step_limit(WZ_ACCEL, min_wz)],
            # The deadband, per axis, standing still. Consumers should either
            # command 0 or at least this much.
            "min_magnitude": [min_vx, min_vy, 0.0, 0.0, 0.0, min_wz],
            # ...and the same thing while translation is already under way,
            # which on a legged robot is a different number entirely.
            "min_magnitude_moving": [min_vx, min_vy, 0.0, 0.0, 0.0,
                                     min_wz_moving],
            # no max_velocity: it would be jerk here, and a declared limit
            # nobody can interpret is worse than an absent one.
        },
        "groups": [{"name": "base", "offset": 0, "count": DOF,
                    "unit": "m/s", "resource": "base"}],
        "frame": "base_link",
        "rate": {
            "max_hz": MAX_HZ,
            "expected_hz": expected_hz,
            "watchdog_ms": WATCHDOG_MS,
            "max_obs_age_ms": MAX_OBS_AGE_MS,
        },
        # The chassis reports no force-torque. Declared null rather than omitted
        # so the absent protection is visible; `parse_descriptor` requires it.
        "force_torque": None,
        # What this robot will hit things with. See FOOTPRINT_HALF_WIDTH.
        "footprint": {
            "shape": "box",
            "half_width": FOOTPRINT_HALF_WIDTH,
            "front": FOOTPRINT_FRONT,
            "rear": FOOTPRINT_REAR,
            "height": FOOTPRINT_HEIGHT,
            # "vendor-spec" | "measured" | "estimate".
            "source": "estimate",
            "arms": "at-rest",
        },
        # Whether the three metric ceilings above mean anything. See
        # `Calibration` — the whole velocity space is a guess until somebody
        # walks the robot, and the policy upstream has no other way to find out.
        "calibration": calibration.as_dict(),
    }


class LocoServoPlugin:
    """One card, one input topic, one sink, one repeat thread."""

    PREFIX = "locoservo"      # no underscore — dispatch routes on partition("_")

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 high_ctrl, loco_plugin=None):
        self._ns = namespace
        self._executor = executor
        self._high_ctrl = high_ctrl
        self._loco = loco_plugin
        config = plugin_config or {}

        self._expected_hz = float(config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not 0 < self._expected_hz <= MAX_HZ:
            raise ValueError(f"loco_servo.expected_hz must be in (0, {MAX_HZ}]")

        # **Defaults to on, unlike R1's.** R1's `dry_run` default was flipped off
        # because a deployed chassis that silently refuses to move is worse than
        # one that moves: `applied: 33, refused: 0, sdk_errors: 0` on screen
        # while the robot stood still cost an afternoon there. That reasoning
        # holds once the numbers are right. Here they are not: nothing has
        # measured what `x = 1.0` means on a Bumi, so the first connection has
        # to be an observation of what this card *wants* to send. Flip it off —
        # from the card, at runtime, no redeploy — once the log's signs and
        # magnitudes look right, and set `calibration_source: measured` in the
        # same breath.
        self._dry_run = bool(config.get("dry_run", True))
        # Whether a command requires the robot to be walking. Checked per
        # command rather than at start — see `_posture_problem`. Configurable
        # only because a bench with no legs attached cannot reach workmode 2.
        self._require_standing = bool(config.get("require_standing", True))
        # Turn in place only: vx and vy are zeroed before they reach the SDK.
        #
        # A deliberate restriction of the robot, not a rejection of the policy —
        # which is why it clamps rather than refusing. Pinning vx/vy in the
        # descriptor instead would make `ControlSink` reject the *whole* command
        # whenever the policy asked to move forward, taking the yaw with it, so
        # the robot would not even turn. The point of this switch is that it
        # still turns.
        self._rotate_only = bool(config.get("rotate_only", False))
        self._suppressed = 0

        self._calibration = Calibration(config)
        self._descriptor_raw = build_descriptor(self._calibration, self._expected_hz)
        self._descriptor = parse_descriptor(self._descriptor_raw)

        self._lock = threading.RLock()
        self._sink = None
        self._sub_node = None
        self._input_topic = ""
        self._running = False
        self._paused = False
        self._last_command = None
        self._applied = 0
        self._holds = 0
        self._refused = 0
        self._sdk_errors = 0
        self._posture_problem_text = ""
        # -inf rather than 0: the first command must actually read the workmode.
        self._mode_checked_at = float("-inf")

        # The standing target the repeat thread keeps sending, already
        # normalised. Zero is a real command here, not an absence — see
        # `_hold`.
        self._target = (0.0, 0.0, 0.0)
        self._repeat_thread = None
        self._repeat_stop = threading.Event()
        self._published = 0
        self._last_dry_run_target = None

        if loco_plugin is not None and hasattr(loco_plugin, "attach_servo"):
            # Lets every call-shaped card pause this one before it acts.
            loco_plugin.attach_servo(self)

    # ── tool ─────────────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "loco_servo",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "Bumi 底盘的连续速度控制：订阅一路 motus.control/1 twist 指令流"
                f"（6 维，≤{self._expected_hz:g} Hz）驱动底盘。"
                "普通走动用 loco 的 move，这张卡片是给导航/执行模型用的。"
            ),
            "inputSchema": {
                "type": "object",
                "properties": {
                    "action": {"type": "string",
                               "enum": ["start", "stop", "pause", "resume", "info"]},
                    "input_topic": {"type": "string",
                                    "description": "control/velocity 指令流话题"},
                },
                "required": ["action"],
                # `start`/`stop` are absent on purpose: agent-core expands each
                # entry into an LLM-callable function, and this card's `start`
                # needs an input topic the model does not have. A model that
                # stopped it could not start it again.
                "x-action-params": {
                    "pause": {"params": [],
                              "description": "立即停止底盘并保持订阅；resume 可继续"},
                    "resume": {"params": [], "description": "继续执行"},
                },
                "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                            "on_interrupt_all": {"action": "pause"}},
                "x-is-dangerous": True,
                "x-resource": ["base"],
            },
            "topic_in": [{"format": "control/velocity",
                          "desc": "motus.control/1 twist，6 维 [vx,vy,vz,wx,wy,wz]"}],
            # Three switches deciding **how much of a command reaches the
            # motors**. They are settable at runtime rather than only in
            # `config.yaml`, because `dry_run` is what an operator reaches for in
            # the second before a new policy is tried on a real robot, not after
            # a redeploy.
            "configSchema": {
                "type": "object",
                "properties": {
                    "dry_run": {
                        "type": "boolean",
                        # On, until somebody measures this chassis. See
                        # `__init__` — the default is also what a form sends for
                        # a field nobody touched, so it matters twice over.
                        "default": True,
                        "description": "空跑：照常接收、检查、计数，但不调用 SDK —— "
                                       "机器人不会动。Bumi 的速度标定还没人量过，"
                                       "所以出厂是开的：先连上去看它想发什么",
                    },
                    "rotate_only": {
                        "type": "boolean",
                        "default": False,
                        "description": "只转不走：vx / vy 在到达 SDK 前被清零，"
                                       "保留 vyaw。场地窄或只想验转向时用",
                    },
                    "require_standing": {
                        "type": "boolean",
                        "default": True,
                        "description": "要求机器人处于 workmode=2（walking）才执行"
                                       "指令。关掉它只在没有接腿的台架上有意义",
                    },
                },
            },
        }

    def dispatch(self, action: str, args: dict):
        if action == "config":
            return self._config(args)
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

    # `configSchema` fields, and the only keys `config` is allowed to touch.
    # Named rather than "whatever arrived": `config` and `start` share an
    # argument dict on this card, so an unfiltered assignment would let a stray
    # `input_topic` or `action` become an attribute.
    _TOGGLES = ("dry_run", "rotate_only", "require_standing")

    def _config(self, args: dict) -> dict:
        """Apply the operator's switches, **including while streaming**.

        Taking effect only at the next `start` is the tempting implementation
        and the wrong one: `dry_run` is reached for when a robot is doing
        something the operator wants stopped *now*, and a toggle that flips in
        the UI, reports success and changes nothing until a restart is the exact
        failure this bundle keeps running into.

        The reverse direction is live too, and is the one to be careful about:
        clearing `dry_run` on a card that is already subscribed hands a running
        command stream to the motors with no further deliberate act. That is the
        operator's decision to make, so it is honoured — and logged, because a
        robot that starts moving with nothing in the log saying why is worse.

        Only keys actually present are applied. A form rendering an unchecked
        box for a field the caller never set would otherwise send
        `require_standing: false` and silently drop a posture check.
        """
        changed = {}
        with self._lock:
            for key in self._TOGGLES:
                if key not in args:
                    continue
                new = bool(args[key])
                attr = f"_{key}"
                if getattr(self, attr) != new:
                    changed[key] = new
                setattr(self, attr, new)
            state = ("running" if self._running and not self._paused else
                     "paused" if self._running else "idle")
        if changed:
            print(f"[loco_servo] config changed while {state}: "
                  + ", ".join(f"{k}={v}" for k, v in changed.items()), flush=True)
        return {"ok": True, "state": state,
                **{key: getattr(self, f"_{key}") for key in self._TOGGLES}}

    # ── lifecycle ────────────────────────────────────────────────────────────

    def start(self):
        """Bundle lifecycle. Deliberately inert.

        A card that moves a chassis must not start moving because a container
        restarted. It subscribes when someone starts the project, and that is a
        person's action.
        """

    def stop(self):
        self._stop()

    # ── actions ──────────────────────────────────────────────────────────────

    def _start(self, args: dict):
        topic = (args.get("input_topic") or "").strip()
        if not topic:
            topic = ((args.get("input_topics") or [""])[0] or "").strip()
        if not topic:
            return {"state": "error",
                    "message": "缺少 input_topic —— 请在画布上把一路 control/velocity "
                               "源连到这张卡片"}
        # The gate about the *robot* comes before the one about this process. An
        # operator whose robot is mid-dance, told "没有 ROS 上下文", goes looking
        # in the wrong place entirely.
        conflict = self._chassis_conflict()
        if conflict:
            return {"state": "error", "message": conflict}

        if self._executor is None:
            return {"state": "error", "message": "没有 ROS 上下文，无法订阅"}

        sink = ControlSink(self._descriptor, self._apply,
                           on_watchdog=self._hold, on_abort=self._hold)
        # Register before starting, so a concurrent stop can find and cancel it.
        with self._lock:
            if self._running:
                return {"state": "error",
                        "message": f"已经在运行（{self._input_topic}）"}
            self._sink = sink
            self._input_topic = topic
            self._running = True
            self._paused = False
            self._applied = 0
            self._holds = 0
            self._published = 0
            self._target = (0.0, 0.0, 0.0)

        try:
            self._open(topic)
        except Exception as exc:                              # noqa: BLE001
            with self._lock:
                self._running = False
                self._sink = None
            return {"state": "error", "message": f"启动失败: {exc}"}

        self._start_repeat()
        print(f"[loco_servo] streaming from {topic} "
              f"(dry_run={self._dry_run}, "
              f"calibration={self._calibration.source})", flush=True)
        return {"state": "running", "input": topic, "dry_run": self._dry_run,
                "rotate_only": self._rotate_only,
                "control_interface": self._interface()}

    def _halt(self, halted: bool):
        """`pause` / `resume`. Stops the chassis but keeps the subscription.

        Unlike an arm — which holds its last joint targets, so "stop sending" is
        itself the hold — a chassis given a velocity keeps travelling. So a
        pause here has to command zero, and keep commanding it.
        """
        with self._lock:
            if not self._running:
                return {"state": "idle", "message": "卡片未在运行"}
            self._paused = bool(halted)
        if halted:
            self._hold()
        return {"state": "paused" if halted else "running",
                "input": self._input_topic}

    def _stop(self):
        with self._lock:
            if not self._running:
                return {"state": "idle"}
            self._running = False
            self._paused = False
            sink, node = self._sink, self._sub_node
            self._sink = None
            self._sub_node = None

        # Stop the chassis *before* tearing down, not after: between removing
        # the subscription and the last publish the robot is still travelling at
        # whatever it was last told.
        self._hold()
        self._stop_repeat()
        # `_stop_repeat` joined the thread that was sending zeros, so the last
        # word to the chassis has to be said explicitly here. Losing it leaves a
        # robot walking with nothing left in the process to stop it.
        self._publish_zero()

        if node is not None:
            try:
                self._executor.remove_node(node)
                node.destroy_node()
            except Exception:                                 # noqa: BLE001
                pass
        del sink
        return {"state": "stopped"}

    def _info(self):
        with self._lock:
            return {
                "state": ("running" if self._running and not self._paused else
                          "paused" if self._running else "idle"),
                "input": self._input_topic,
                "dry_run": self._dry_run,
                # Visible, always — an operator looking at a robot that only
                # spins needs this on the same screen as the command values.
                "rotate_only": self._rotate_only,
                "require_standing": self._require_standing,
                "suppressed_translations": self._suppressed,
                "applied": self._applied,
                "holds": self._holds,
                "refused": self._refused,
                "sdk_errors": self._sdk_errors,
                # Commands actually handed to the SDK. Unlike `applied` this
                # counts the repeat thread's work, so a card that is "applying"
                # while nothing reaches the robot is visible as the two numbers
                # disagreeing.
                "published": self._published,
                # Empty when the posture is fine. A card that is subscribed and
                # running but refusing every command looks identical from the
                # canvas to one that is working, and this is the difference.
                "posture_problem": self._posture_problem_text,
                "last": self._last_command,
                "control_interface": self._interface(),
            }

    def _interface(self) -> dict:
        """The descriptor, plus whether this chassis is currently a no-op.

        `dry_run` and `rotate_only` ride along because the card upstream has no
        other way to find out. A policy whose commands are being swallowed looks
        exactly like one that is working — same verdicts, same counters, same
        silence — and the upstream card's `degraded` list is the only place that
        difference can reach an operator. `calibration` is in the descriptor for
        the same reason.
        """
        out = dict(self._descriptor_raw)
        if self._dry_run:
            out["dry_run"] = True
        if self._rotate_only:
            out["rotate_only"] = True
        return out

    # ── arbitration with the call-shaped cards ───────────────────────────────

    def _chassis_conflict(self) -> str:
        """Refuse to start while `loco` is driving the same chassis.

        Starting anyway produces two threads publishing velocities at different
        rates into one SDK, which reads on the robot as a stutter and in the
        logs as nothing at all.
        """
        if self._loco is None or not getattr(self._loco, "is_moving", None):
            return ""
        if self._loco.is_moving():
            return ("loco 卡片正在驱动底盘（move 未 stop_move）—— 两张卡片同时"
                    "写同一个底盘会互相打架。请先调用 loco 的 stop_move。")
        return ""

    def _posture_problem(self) -> str:
        """Why this chassis must not be driven right now, or "".

        **Checked per command, not at start.** Starting is a wiring event: a
        project comes up when someone opens the canvas, and the robot is very
        often lying down or disabled at that moment. Refusing to start then
        blocks the whole canvas — every other card with it — over a posture that
        says nothing about whether the wiring is right, and by the time a
        command arrives the robot may well have stood up anyway.

        Cached, because at `expected_hz` a DDS read per command would put a
        round-trip to the robot's own controller in the path of every velocity.
        """
        if not self._require_standing:
            return ""

        now = time.monotonic()
        if now - self._mode_checked_at < MODE_CACHE_S:
            return self._posture_problem_text

        self._mode_checked_at = now
        try:
            mode = int(self._high_ctrl.get_mode())
        except Exception as exc:                              # noqa: BLE001
            # Acting on a failed read is how a "safe" call becomes a fall.
            self._posture_problem_text = f"读不到 workmode（{exc}）"
        else:
            if mode == PROTECTION_MODE:
                self._posture_problem_text = (
                    "机器人处于保护模式（workmode=26），不执行速度指令。"
                    "请把它平放、面朝上，再用 stand_up_lie_prone 的 stand_up 起身")
            elif mode != WALKING_MODE:
                self._posture_problem_text = (
                    f"机器人当前 workmode={mode}，不是 walking({WALKING_MODE})，"
                    "不执行速度指令。请先用 stand_up_lie_prone 的 stand_up 让它站起来")
            else:
                self._posture_problem_text = ""
        return self._posture_problem_text

    def pause_for_explicit_command(self, reason: str = "") -> bool:
        """Called by the call-shaped cards before they move the robot themselves.

        A person or the LLM saying "move" outranks a policy that is streaming.
        Returns whether this card was actually running, so the caller can say so.
        """
        with self._lock:
            if not self._running or self._paused:
                return False
            self._paused = True
        self._hold()
        print(f"[loco_servo] paused by an explicit command"
              f"{': ' + reason if reason else ''}", flush=True)
        return True

    # ── wiring ───────────────────────────────────────────────────────────────

    def _open(self, topic: str):
        from rclpy.node import Node
        from std_msgs.msg import String

        node = Node(f"bumi_loco_servo_{abs(hash(topic)) % 100000}")
        node.create_subscription(String, topic, self._on_message, 1)
        self._executor.add_node(node)
        with self._lock:
            self._sub_node = node

    def _on_message(self, message):
        sink = self._sink
        if sink is None or self._paused:
            return
        try:
            command = json.loads(message.data)
        except Exception:                                     # noqa: BLE001
            return          # one bad frame must not take the stream down
        outcome = sink.submit(command)
        if outcome is not None and getattr(outcome, "verdict", None) is not None:
            self._last_command = {
                "verdict": str(getattr(outcome.verdict, "name", outcome.verdict)),
                "reason": getattr(outcome, "reason", ""),
                "at": time.time(),
            }

    # ── the repeat thread ────────────────────────────────────────────────────

    def _start_repeat(self):
        self._repeat_stop.clear()
        thread = threading.Thread(target=self._repeat_loop, daemon=True,
                                  name="bumi_loco_servo")
        self._repeat_thread = thread
        thread.start()

    def _stop_repeat(self):
        self._repeat_stop.set()
        thread = self._repeat_thread
        self._repeat_thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    def _repeat_loop(self):
        """Re-send the standing target, and drive the watchdog off the same clock.

        The watchdog lives here rather than on a ROS timer for the same reason
        the repeat does: both have to keep running when the executor is busy,
        and noticing that commands have *stopped* arriving is not something to
        do on the thread that would also be blocked by whatever stopped them.

        A failure to publish does not end the loop. The chassis is moving; the
        one thing worse than an SDK that rejects commands is a robot travelling
        with nothing left in the process trying to stop it.
        """
        period = 1.0 / REPEAT_HZ
        while not self._repeat_stop.is_set():
            sink = self._sink
            if sink is not None:
                try:
                    sink.tick()
                except Exception as exc:                      # noqa: BLE001
                    print(f"[loco_servo] watchdog tick failed: {exc}", flush=True)
            if not self._dry_run:
                with self._lock:
                    target = self._target
                self._publish(*target)
            self._repeat_stop.wait(period)

    def _publish(self, nx: float, ny: float, nz: float):
        """Hand one normalised command to the SDK, through `loco`'s rate limiter.

        Routed via the `loco` plugin rather than straight at `high_ctrl` so the
        two cards share one lock and one >=2 ms spacing. Two writers with
        separate limiters can interleave inside a single publish, and the SDK
        has no way to tell us that happened.
        """
        try:
            if self._loco is not None and hasattr(self._loco, "publish_velocity"):
                self._loco.publish_velocity(nx, ny, nz)
            elif self._high_ctrl is not None:
                self._high_ctrl.publish_cmd(nx, ny, nz, _default_cmd(), 0)
            else:
                return
        except Exception as exc:                              # noqa: BLE001
            self._sdk_errors += 1
            # Logged on the transition and every hundredth after: at 50 Hz a
            # line per command buries every other log the robot produces, and
            # the first one already says everything the rest would.
            if self._sdk_errors == 1 or self._sdk_errors % 100 == 0:
                print(f"[loco_servo] publish_cmd 失败（第 {self._sdk_errors} 条）："
                      f"({nx:+.3f}, {ny:+.3f}, {nz:+.3f}) -> {exc}", flush=True)
            return
        if self._sdk_errors:
            print(f"[loco_servo] SDK 恢复接受指令（此前失败 "
                  f"{self._sdk_errors} 条）", flush=True)
            self._sdk_errors = 0
        self._published += 1

    def _publish_zero(self):
        """One zero, now, outside the repeat loop. Used at teardown."""
        if self._dry_run:
            print("[loco_servo] DRY RUN publish_cmd(0, 0, 0)", flush=True)
            return
        self._publish(0.0, 0.0, 0.0)

    # ── the command path ─────────────────────────────────────────────────────

    def _apply(self, values, _gripper=None):
        vx, vy, wz = float(values[0]), float(values[1]), float(values[5])

        if self._rotate_only and (vx or vy):
            if self._suppressed == 0:
                print(f"[loco_servo] rotate_only: 抑制平移 vx={vx:+.3f} "
                      f"vy={vy:+.3f}，只执行 vyaw={wz:+.3f}", flush=True)
            self._suppressed += 1
            vx = vy = 0.0

        # The posture gate lives here, on the command, not on `start`.
        posture = self._posture_problem()
        if posture:
            self._refused += 1
            # Announced on the transition only. At `expected_hz` a line per
            # refused command would bury every other log the robot produces.
            if self._refused == 1 or self._refused % 100 == 0:
                print(f"[loco_servo] 拒绝执行（第 {self._refused} 条）：{posture}",
                      flush=True)
            self._last_command = {"verdict": "REFUSED", "reason": posture,
                                  "at": time.time()}
            # A refused command must also stop the robot, not merely fail to
            # update the target: the repeat thread is still sending whatever was
            # accepted last, so "we stopped accepting" would otherwise mean
            # "it keeps walking at the last speed it was told".
            self._hold()
            return

        self._refused = 0
        self._applied += 1
        target = self._calibration.to_normalised(vx, vy, wz)
        with self._lock:
            self._target = target

        if self._dry_run:
            # Only when the target changes. The metric values are the useful
            # half — they are what the policy asked for — and the normalised
            # ones are what would reach the SDK, so both are printed: this log
            # is the whole point of `dry_run` on an unmeasured chassis.
            if target != self._last_dry_run_target:
                self._last_dry_run_target = target
                print(f"[loco_servo] DRY RUN vx={vx:+.3f} m/s vy={vy:+.3f} m/s "
                      f"wz={wz:+.3f} rad/s -> publish_cmd("
                      f"{target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f})",
                      flush=True)
            return

        # Sent now rather than waiting up to 20 ms for the repeat thread's next
        # turn: at `expected_hz` the policy's own period is 100 ms, so letting
        # the repeat rate set the latency would add a fifth of it for nothing.
        self._publish(*target)

    def _hold(self, *_args):
        """Watchdog, abort, pause, refusal and teardown all land here.

        A chassis holds by being told to stop, **and told again** — which is the
        one way this differs from every arm card in the repo, and from R1's
        version of this file, where a single `StopMove` latches.
        """
        self._holds += 1
        with self._lock:
            self._target = (0.0, 0.0, 0.0)
        self._last_dry_run_target = None
        if self._dry_run:
            print("[loco_servo] DRY RUN hold: publish_cmd(0, 0, 0)", flush=True)
            return
        self._publish(0.0, 0.0, 0.0)


def _default_cmd():
    """`ControlCmd.DEFAULT`, imported only when there is no `loco` to route through.

    Kept out of the module body on purpose: the SDK does not exist on a laptop,
    and this file has to be loadable there or its descriptor can only be
    asserted on a robot.
    """
    from highcontrol_py import ControlCmd

    return ControlCmd.DEFAULT
