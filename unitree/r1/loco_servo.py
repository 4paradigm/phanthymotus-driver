#!/usr/bin/env python3
"""Continuous chassis control for Unitree R1 — the first `twist` card in this repo.

`loco` in device.py is the call-shaped path: one `tools/call` per motion, chosen
by an LLM or a person. Right for "walk forward for two seconds", wrong for a
navigation policy, which emits a velocity ten times a second and is not waiting
for an answer to each one. Same split as `arm` versus `servo` on G1, and the
reasoning is in that file's module docstring.

This card is the stream-shaped path. The action space is `motus.control/1`'s
`twist`: six numbers, body frame, `[vx, vy, vz, wx, wy, wz]`. R1 actuates three
of them.

── twist is a velocity space, and the sink was built for position spaces ─────

`ControlSink`'s chain is the same, but two limit fields change meaning, which is
the thing to get right here and is easy to get wrong silently:

* `limits.lower/upper` bound the **velocity** — still exactly right.
* `max_delta_per_step` is the difference between consecutive commands, so here
  it is an **acceleration** cap. Also right, and the field that keeps a policy
  from kicking the chassis.
* `max_velocity` is delta over dt, which in a velocity space is **jerk**. It has
  no useful meaning, so it is deliberately **not declared** — `parse_descriptor`
  takes it as optional, so omitting it is the correct action rather than a
  workaround.

── the three axes R1 does not have ───────────────────────────────────────────

`vz`, `wx` and `wy` are pinned `lower == upper == 0` rather than left wide. A
policy that believes it is commanding vertical motion is then **rejected** by
the sink, loudly, instead of having a third of its output silently dropped. The
vector is six wide either way; what is being chosen here is whether the
disagreement is visible.

── one chassis, two cards ────────────────────────────────────────────────────

`loco` and this card hold the same `RpcProxy` and both call `Move`. Matching
`x-resource` does **not** keep them apart — ACP arbitrates its own scheduling,
not two different routes to one motor. So the two are explicitly wired together:
this card refuses to start while `loco` has a motion in flight, and `loco`'s own
`move`/`stop_move` pause this one first. An explicit instruction from a person
outranks a running policy, never the other way round. See README_dev.md
§ "x-resource is a label, not a lock".
"""

from __future__ import annotations

import json
import threading
import time

from common.control import ControlSink, parse_descriptor

# Taken from the quantities `loco`'s own tool schema declares for this chassis,
# rather than transcribed from a datasheet a second time — two copies of a limit
# is how one of them ends up wrong.
VX_LIMIT = 1.0          # m/s
VY_LIMIT = 1.0          # m/s
WZ_LIMIT = 2.0          # rad/s

# Acceleration caps, per step at `expected_hz`. Conservative and chosen here
# rather than derived from the hardware maximum: the SDK will accept a jump from
# rest to full speed, and "the chassis can survive it" is not the same statement
# as "a policy may ask for it". Same call as the arm cards make about URDF
# velocity limits.
VX_ACCEL = 0.15
VY_ACCEL = 0.15
WZ_ACCEL = 0.30

# Pinned axes still need a positive `max_delta_per_step` — parse_descriptor
# rejects zero, and their real bound is the `lower == upper == 0` above anyway.
PINNED_ACCEL = 1e-6

# Below these the robot does nothing at all. Measured on r1_sz by commanding one
# axis at a time: wz needs 1.0 rad/s, vx and vy need 0.4 m/s. Anything smaller
# is accepted by the SDK, returns 0, and produces no motion whatsoever.
#
# A legged robot has to assemble a whole gait cycle to move, so there is no
# "creep slowly" regime the way a wheeled base has. This is a property of the
# robot, which is why it is declared here and not assumed by whatever is driving
# it: a policy emitting a smooth ramp towards zero would spend its whole life in
# this band, commanding motion and producing none, with every layer in between
# reporting success.
MIN_VX = 0.4
MIN_VY = 0.4
MIN_WZ = 1.0


def _step_limit(accel: float, floor: float) -> float:
    """The acceleration cap, widened to at least the axis' deadband.

    **An acceleration cap finer than the deadband is not a cap — it is dead
    time.** The step clamp is applied to the command the policy sends, so a
    0.30 rad/s cap against a 1.0 rad/s floor ramps 0.30 → 0.60 → 0.90 → 1.0, and
    the robot executes precisely none of the first three: three ticks of
    silence, then the turn starts at full speed. That asymmetry — slow to start,
    instant to stop, because the ramp *down* crosses the floor on its first
    step — is what a lurch is made of, and neither the sink nor the SDK reports
    anything, because from their side every command was accepted.

    So the smallest meaningful step on a deadbanded axis is the deadband. Above
    the floor the configured cap is coarser than intended, which is a real cost
    and the honest one: it is the granularity the chassis has. Shaping
    acceleration below the floor is the gait controller's job, not ours.
    """
    return max(accel, floor)

DEFAULT_EXPECTED_HZ = 10.0
MAX_HZ = 20.0
# Generous next to an arm's 200 ms because a chassis at 0.4 m/s travels 12 cm in
# this window, and because the upstream is a perception pipeline whose frame
# cadence is lumpier than a policy's.
WATCHDOG_MS = 300
MAX_OBS_AGE_MS = 500

# FSM 811 — loco_mode. Anything else and the robot is on the ground, lying down,
# or mid-transition.
STANDING_FSM = 811

# How long an FSM reading stays good for. See `_posture_problem`.
FSM_CACHE_S = 1.0

AXIS_NAMES = ["vx", "vy", "vz", "wx", "wy", "wz"]
DOF = 6


def build_descriptor(expected_hz: float = DEFAULT_EXPECTED_HZ) -> dict:
    return {
        "control_interface": "motus.control/1",
        "mode": "twist",
        "dof": DOF,
        "joint_names": list(AXIS_NAMES),
        "units": {"linear": "m/s", "angular": "rad/s", "time": "s"},
        "limits": {
            "lower": [-VX_LIMIT, -VY_LIMIT, 0.0, 0.0, 0.0, -WZ_LIMIT],
            "upper": [VX_LIMIT, VY_LIMIT, 0.0, 0.0, 0.0, WZ_LIMIT],
            # Acceleration, not velocity — see the module docstring. Never finer
            # than the deadband on the same axis; see `_step_limit`.
            "max_delta_per_step": [_step_limit(VX_ACCEL, MIN_VX),
                                   _step_limit(VY_ACCEL, MIN_VY),
                                   PINNED_ACCEL, PINNED_ACCEL, PINNED_ACCEL,
                                   _step_limit(WZ_ACCEL, MIN_WZ)],
            # The deadband, per axis. 0 means "no threshold on this axis".
            # Consumers should either command 0 or at least this much — see
            # MIN_VX above.
            "min_magnitude": [MIN_VX, MIN_VY, 0.0, 0.0, 0.0, MIN_WZ],
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
        # so the absent protection is visible; parse_descriptor requires it.
        "force_torque": None,
    }


class LocoServoPlugin:
    """One card, one input topic, one sink, one `Move` call per command."""

    PREFIX = "locoservo"      # no underscore — dispatch routes on partition("_")

    def __init__(self, plugin_config: dict, namespace: str, executor,
                 loco_client, loco_plugin=None):
        self._ns = namespace
        self._executor = executor
        self._client = loco_client
        self._loco = loco_plugin
        config = plugin_config or {}

        self._expected_hz = float(config.get("expected_hz", DEFAULT_EXPECTED_HZ))
        if not 0 < self._expected_hz <= MAX_HZ:
            raise ValueError(f"loco_servo.expected_hz must be in (0, {MAX_HZ}]")
        # Defaults to **on**. The first thing anyone does with a new command card
        # is wire it up and watch, and a card that drives a chassis the first time
        # it is connected is the wrong default for that.
        self._dry_run = bool(config.get("dry_run", True))
        # Whether a command requires the robot to be standing. Checked per
        # command rather than at start — see `_posture_problem`. Configurable
        # only because a bench with no chassis attached cannot reach FSM 811.
        self._require_standing = bool(config.get("require_standing", True))
        # Turn in place only: vx and vy are zeroed before they reach the SDK.
        #
        # A deliberate restriction of the robot, not a rejection of the policy —
        # which is why it clamps rather than refusing. Pinning vx/vy in the
        # descriptor instead would make `ControlSink` reject the *whole*
        # command whenever the policy asked to move forward, taking the yaw
        # with it, so the robot would not even turn. The point of this switch is
        # that it still turns.
        #
        # It is loud on purpose: a card that quietly drops two thirds of every
        # command is exactly the failure this file keeps warning about.
        self._rotate_only = bool(config.get("rotate_only", False))
        self._suppressed = 0

        self._descriptor_raw = build_descriptor(self._expected_hz)
        self._descriptor = parse_descriptor(self._descriptor_raw)

        self._lock = threading.RLock()
        self._sink = None
        self._sub_node = None
        self._ticker = None
        self._input_topic = ""
        self._running = False
        self._paused = False
        self._last_command = None
        self._applied = 0
        self._holds = 0
        self._refused = 0
        self._sdk_errors = 0
        self._last_ret = 0
        self._fsm_problem = ""
        # -inf rather than 0: the first command must actually read the FSM.
        self._fsm_checked_at = float("-inf")

        if loco_plugin is not None and hasattr(loco_plugin, "attach_servo"):
            # Lets `loco.move` pause this card before it touches the chassis.
            loco_plugin.attach_servo(self)

    # ── tool ─────────────────────────────────────────────────────────────────

    def get_tool(self) -> dict:
        return {
            "name": "loco_servo",
            "type": "actuator",
            "multiInstance": False,
            "description": (
                "R1 底盘的连续速度控制：订阅一路 motus.control/1 twist 指令流"
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
        }

    def dispatch(self, action: str, args: dict):
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
        # The two gates about the *robot* come before the one about this
        # process. An operator whose robot is lying down, told "没有 ROS 上下文",
        # goes looking in the wrong place entirely — and both of these are
        # reasons to refuse regardless of whether the plumbing would have worked.
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

        try:
            self._open(topic)
        except Exception as exc:                              # noqa: BLE001
            with self._lock:
                self._running = False
                self._sink = None
            return {"state": "error", "message": f"启动失败: {exc}"}

        print(f"[loco_servo] streaming from {topic} "
              f"(dry_run={self._dry_run})", flush=True)
        return {"state": "running", "input": topic, "dry_run": self._dry_run,
                "rotate_only": self._rotate_only,
                "control_interface": self._descriptor_raw}

    def _halt(self, halted: bool):
        """`pause` / `resume`. Stops the chassis but keeps the subscription.

        Unlike an arm — which holds its last joint targets, so "stop sending" is
        itself the hold — a chassis given a velocity keeps travelling. So a pause
        here has to command zero, and it does that through the same `StopMove`
        the watchdog uses.
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
            sink, node, ticker = self._sink, self._sub_node, self._ticker
            self._sink = None
            self._sub_node = None
            self._ticker = None

        # Stop the chassis *before* tearing down, not after: between removing the
        # subscription and the SDK call the robot is still travelling at whatever
        # it was last told.
        self._hold()

        if ticker is not None and node is not None:
            try:
                node.destroy_timer(ticker)
            except Exception:                                 # noqa: BLE001
                pass
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
                "suppressed_translations": self._suppressed,
                "applied": self._applied,
                "holds": self._holds,
                "refused": self._refused,
                # Rejections from the SDK. A different fact from `refused`,
                # which is us declining to send: this one is "sent, and the
                # robot would not take it".
                "sdk_errors": self._sdk_errors,
                "last_ret": self._last_ret,
                # Empty when the posture is fine. A card that is subscribed and
                # running but refusing every command looks identical from the
                # canvas to one that is working, and this is the difference.
                "posture_problem": self._fsm_problem,
                "last": self._last_command,
                "control_interface": self._descriptor_raw,
            }

    # ── arbitration with the call-shaped card ────────────────────────────────

    def _chassis_conflict(self) -> str:
        """Refuse to start while `loco` is driving the same chassis.

        Starting anyway produces two sources writing `Move` at different rates,
        which reads on the robot as a stutter and in the logs as nothing at all.
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
        often lying down at that moment. Refusing to start then blocks the whole
        canvas — every other card with it — over a posture that says nothing
        about whether the wiring is right. It was also, in practice, simply
        wrong about the future: by the time a command arrives the robot may well
        have stood up, and by the time it *has* started the robot may have lain
        down again. Only the moment of the command can answer this.

        Cached, because at `expected_hz` an RPC per command would put a
        round-trip to the robot's own controller in the path of every velocity.
        The window is short enough that a posture change is noticed within a few
        commands, which is far inside the driver's own reaction time.
        """
        if not self._require_standing:
            return ""

        now = time.monotonic()
        if now - self._fsm_checked_at < FSM_CACHE_S:
            return self._fsm_problem

        self._fsm_checked_at = now
        try:
            code, fsm = self._client.GetFsmId()
        except Exception as exc:                              # noqa: BLE001
            self._fsm_problem = f"读不到 FSM 状态（{exc}）"
        else:
            if code != 0:
                # Acting on a failed read is how a "safe" call becomes a fall —
                # the same rule `switch_mode` already follows.
                self._fsm_problem = f"读不到 FSM 状态（code={code}）"
            elif fsm != STANDING_FSM:
                self._fsm_problem = (
                    f"机器人当前 FSM={fsm}，不是 loco_mode({STANDING_FSM})，"
                    "不执行速度指令。请先用 switch_mode 的 lie2standup 让它站起来")
            else:
                self._fsm_problem = ""
        return self._fsm_problem

    def pause_for_explicit_command(self, reason: str = "") -> bool:
        """Called by `loco` before it drives the chassis itself.

        A person or the LLM saying "move" outranks a policy that is streaming.
        Returns whether this card was actually running, so the caller can say so.
        """
        with self._lock:
            if not self._running or self._paused:
                return False
            self._paused = True
        print(f"[loco_servo] paused by an explicit loco command{': ' + reason if reason else ''}",
              flush=True)
        return True

    # ── wiring ───────────────────────────────────────────────────────────────

    def _open(self, topic: str):
        from rclpy.node import Node
        from std_msgs.msg import String

        node = Node(f"r1_loco_servo_{abs(hash(topic)) % 100000}")
        node.create_subscription(String, topic, self._on_message, 1)
        # The watchdog only fires from `tick`, so it needs a clock of its own —
        # the whole point is to notice that messages have *stopped* arriving.
        self._ticker = node.create_timer(WATCHDOG_MS / 2000.0, self._tick)
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

    def _tick(self):
        sink = self._sink
        if sink is not None and not self._paused:
            sink.tick()

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
            # refused command would bury every other log the robot produces,
            # and the first one already says everything the rest would.
            if self._refused == 1 or self._refused % 100 == 0:
                print(f"[loco_servo] 拒绝执行（第 {self._refused} 条）：{posture}",
                      flush=True)
            self._last_command = {"verdict": "REFUSED", "reason": posture,
                                  "at": time.time()}
            return

        self._refused = 0
        self._applied += 1
        if self._dry_run:
            print(f"[loco_servo] DRY RUN Move(vx={vx:+.3f}, vy={vy:+.3f}, "
                  f"vyaw={wz:+.3f})", flush=True)
            return

        # **Keep the return code.** Discard it and an SDK rejecting every
        # command looks identical from outside to one working perfectly:
        # `applied` climbs, nothing is logged, the robot stands still. That is
        # what it took a while to think of looking at on the robot — `loco`'s
        # own `move` has always reported `ret`, and this card missed it when it
        # copied the pattern.
        ret = self._client.Move(vx, vy, wz, True)
        if ret != 0:
            self._sdk_errors += 1
            self._last_ret = ret
            # Logged on the transition and every hundredth after: at 10 Hz a
            # line per command buries every other log the robot produces, and
            # the first one already says everything the rest would.
            if self._sdk_errors == 1 or self._sdk_errors % 100 == 0:
                print(f"[loco_servo] SDK 拒绝了指令（第 {self._sdk_errors} 条）："
                      f"Move(vx={vx:+.3f}, vy={vy:+.3f}, vyaw={wz:+.3f}) "
                      f"-> ret={ret}", flush=True)
        else:
            if self._sdk_errors:
                print(f"[loco_servo] SDK 恢复接受指令（此前拒绝 "
                      f"{self._sdk_errors} 条）", flush=True)
            self._sdk_errors = 0
            self._last_ret = 0

    def _hold(self, *_args):
        """Watchdog, abort, pause and teardown all land here.

        A chassis holds by being told to stop, not by being left alone — which is
        the one way this differs from every arm card in the repo.
        """
        self._holds += 1
        if self._dry_run:
            print("[loco_servo] DRY RUN StopMove()", flush=True)
            return
        try:
            self._client.StopMove()
        except Exception as exc:                              # noqa: BLE001
            print(f"[loco_servo] StopMove failed: {exc}", flush=True)
