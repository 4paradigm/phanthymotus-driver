"""Full absolute TCP and gripper targets for the TRON2 tabletop task executor."""
import asyncio
import json
import math
import os
import time
from pathlib import Path
from sys import float_info

from common.control import ControlSink, SCHEMA, parse_descriptor

from arm_servo import Limiter, MAX_ACCELERATION, MAX_OUTPUT_GAP, MAX_VELOCITY, OUTPUT_HZ, TRACKING_LIMIT, MAX_HZ, WATCHDOG_MS
from eef_kinematics import NoFeasibleStep
from eef_state import CONTROL_TOPIC, FRAME
from gripper import MAX_OPENING_MM, command
from vendor import ARM_LOWER, ARM_UPPER, MoveError, motion_readiness, reject_constant

SIDES = ("left", "right")
MAX_OBS_AGE_MS = 1000
MAX_FUTURE_MS = 100


def build_descriptor():
    names, groups = [], []
    for offset, side in ((0, "l"), (8, "r")):
        names.extend(f"eef_{side}_{axis}" for axis in ("x", "y", "z", "qx", "qy", "qz", "qw"))
        names.append(f"gripper_{side}")
        groups.extend([{"name": f"eef_{side}", "offset": offset, "count": 7, "unit": "m+quat",
                        "mode": "eef_pose", "resource": f"arm_{side}"},
                       {"name": f"gripper_{side}", "offset": offset + 7, "count": 1, "unit": "m",
                        "mode": "joint_position", "resource": f"gripper_{side}"}])
    # XYZ bounds describe accepted requests; joint-limited IK finds the reachable endpoint.
    lower = [-float_info.max] * 3 + [-1.] * 4 + [0.]
    upper = [float_info.max] * 3 + [1.] * 4 + [MAX_OPENING_MM / 1000.]
    return {"control_interface": SCHEMA, "mode": "eef_pose", "dof": 16, "frame": FRAME,
            "joint_names": names, "groups": groups, "units": {"length": "m", "rotation": "quat_xyzw", "time": "s"},
            "limits": {"lower": lower * 2, "upper": upper * 2}, "force_torque": None, "urdf_ref": "model",
            "end_effector": {"left": "L_ee_link", "right": "R_ee_link"},
            "rate": {"expected_hz": 10, "max_hz": MAX_HZ, "watchdog_ms": WATCHDOG_MS,
                     "max_obs_age_ms": MAX_OBS_AGE_MS}}


def canonical_pose(pose):
    quaternion = pose[3:]
    length = math.hypot(*quaternion)
    if abs(length - 1.) > .001:
        raise ValueError("Expected a unit quaternion (xyzw)")
    # q and -q command exactly the same orientation, including at half turns.
    sign = next(v for v in reversed(quaternion) if v != 0.)
    return pose[:3] + [v / math.copysign(length, sign) for v in quaternion]


def decode(raw):
    if not isinstance(raw, str) or len(raw) > 8192:
        raise ValueError("Expected a control JSON message of at most 8 KiB")
    msg = json.loads(raw, parse_constant=reject_constant)
    if not isinstance(msg, dict) or msg.get("frame", FRAME) != FRAME or msg.get("gripper") is not None:
        raise ValueError(f"Expected both grippers in values, with TCPs in {FRAME}")
    now = time.time_ns() // 1_000_000
    for field in ("stamp_ms", "obs_stamp_ms"):
        stamp = msg.get(field)
        if stamp is not None and (type(stamp) is not int or not 0 < stamp <= now + MAX_FUTURE_MS):
            raise ValueError(f"Expected a Unix millisecond {field} without future clock skew")
    # Saturate the two physical openings before the shared whole-frame hard-limit check.
    # Malformed fields are left for ControlSink's contract validation.
    overflow = {}
    values = msg.get("values")
    if isinstance(values, list) and len(values) == 16:
        for side, i in zip(SIDES, (7, 15)):
            gap = values[i]
            if type(gap) in (float, int) and math.isfinite(gap):
                limit = min(MAX_OPENING_MM / 1000., max(0., gap))
                if limit != gap:
                    overflow[side] = gap
                    values[i] = limit
    return msg, overflow


def _command_step(limiter, target, dt):
    """End the slew on the exact IK setpoint when a final bounded step can hold it."""
    previous, velocities = limiter.q.copy(), limiter.v.copy()
    limiter.step(target, dt)
    dv = MAX_ACCELERATION * dt
    for i, goal in enumerate(target):
        final_speed = (goal - previous[i]) / dt
        if abs(final_speed) <= min(MAX_VELOCITY, dv) and abs(final_speed - velocities[i]) <= dv:
            limiter.q[i], limiter.v[i] = goal, 0.
    return limiter.q.copy()


class EefServo:
    def __init__(self, adapter, state):
        self.adapter, self.reader, self.state = adapter, adapter.reader, state
        self.publisher = self.subscription = self.worker = self.output = self.watchdog = None
        self.running = self.paused = False
        self.queue = self.token = None
        self.latest = self.pending_ik = None
        self.descriptor = build_descriptor()
        self.sink = None
        self.deadline = 0.
        self.echo_arms, self.echo_jaws = {}, set()
        self.gripper_send = None
        self.last_stamp = -1
        self.arms = {}
        self.jaws = {}
        self.poses = {}
        self.held_at_boundary = set()
        self.target_session = None
        self.last = None
        self.rejected = self.accepted = 0
        self.overruns = 0
        self.topic = CONTROL_TOPIC

    def tool(self):
        return {"name": "servo_eef", "type": "actuator", "multiInstance": False,
                "description": "Absolute vendor TCP targets (L_ee_link / R_ee_link) in base_Link, metres and xyzw. "
                               "Left pose (7), opening (1), right pose (7), opening (1). "
                               "Send all 16 values in the motus.control/1 values array for each absolute target. The task evaluates arrival from state/eef.",
                "inputSchema": {"type": "object", "required": ["action"], "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "info"]},
                    "input_topic": {"type": "string", "default": CONTROL_TOPIC}},
                    "x-hooks": {"on_interrupt_motion": {"action": "pause"},
                                "on_interrupt_all": {"action": "pause"}},
                    "x-is-dangerous": True,
                    "x-resource": ["arm_l", "arm_r", "head", "gripper_l", "gripper_r"]},
                "topic_in": [{"format": "control/eef", "topic": CONTROL_TOPIC,
                              "desc": f"{SCHEMA}, both TCP poses and both gripper openings"}]}

    def info(self):
        return {"state": "idle" if not self.running else "paused" if self.paused else "running",
                "paused": self.paused,
                "topic_in": [{"format": "control/eef", "topic": self.topic}],
                "control_interface": self.descriptor, "sink": self.sink.stats() if self.sink else {},
                "accepted": self.accepted, "rejected": self.rejected, "queue_full": self.overruns,
                "active_arms": list(self.arms), "last": self.last}

    def _ready(self):
        self.adapter.arm_servo._conflict(self.token)
        status = self.reader.sample("robot_status")
        data = status.get("data") or {}
        if (Path(os.environ.get("TRON2_MOTION_MAINTENANCE_FILE",
                                "/config/tron2/motion-maintenance.json")).exists()
                or not status["available"] or data.get("motor") != "OK"
                or not motion_readiness(data)["ready"]
                or self.reader.configuration.get("mode") != "upper_limb"):
            raise ValueError("DACH upper-limb motion is unavailable")

    def _solve_arms(self, targets, joints):
        solutions, blocked = {}, set()
        for side, pose in targets.items():
            seed = joints[:7] if side == "left" else joints[7:14]
            try:
                solutions[side] = self.state.kinematics.solve(side, pose[:3], pose[3:], seed)
            except NoFeasibleStep:
                blocked.add(side)
        return solutions, blocked

    async def _apply(self, stamp, poses, requested_jaws, clamped, observed):
        echo_arms, echo_jaws = self.echo_arms, self.echo_jaws

        def current():
            # New measurement/command stamps may renew the same target during IK or SDK I/O.
            return (not self.paused and self.latest is not None and self.latest[1:3] == (poses, requested_jaws)
                    and self.echo_arms.keys() == echo_arms.keys() and self.echo_jaws == echo_jaws
                    and time.monotonic() < self.deadline)

        if not current():
            return
        self._ready()
        measured = self.state.values()
        session = self.reader.ws
        if self.target_session is not session:
            self.jaws.clear()
            self.poses.clear()
            self.held_at_boundary.clear()
            self.target_session = session
        # Only a previously commanded arm needs one joint-space hold on transition.
        to_hold = {side for side in echo_arms if side in self.poses
                   and (side not in self.held_at_boundary or side in self.arms)}
        to_solve = {}
        for i, side in enumerate(SIDES):
            pose = poses[side]
            if side in echo_arms or pose == self.poses.get(side):
                continue
            if side in self.poses or side in self.arms or pose != canonical_pose(measured[8*i:8*i + 7]):
                to_solve[side] = pose
        held = {s for s in self.held_at_boundary.difference(echo_arms) if poses[s] == self.poses[s]}
        solved = {}
        if to_solve:
            joints = self.state.joints()
            # Cancelling the wait discards an obsolete result. The solver thread
            # does only numerical work and has no access to motor output.
            loop = asyncio.get_running_loop()
            self.pending_ik = loop.run_in_executor(None, self._solve_arms, to_solve, joints)
            try:
                solved, blocked = await self.pending_ik
            except asyncio.CancelledError:
                if self.paused:
                    raise
                return
            finally:
                self.pending_ik = None
            if not current():
                return
            stamp = self.latest[0]
            if self.reader.ws is not session:
                raise ValueError("Robot connection changed while solving IK")
            self._ready()
            measured = self.state.values()
            held.update(blocked)
            to_hold.update(blocked.intersection(self.arms))
        if to_solve or to_hold:
            _, measured_q, speeds = self.state.joint_snapshot()
            if to_solve and max(abs(a - b) for a, b in zip(measured_q, joints)) > TRACKING_LIMIT:
                raise ValueError("Robot configuration changed while solving IK")
        if solved or to_hold:
            if self.token is None and max(map(abs, speeds)) > .05:
                raise ValueError("Cannot take over a moving arm or head")
            Limiter(measured_q)
            for side in to_hold:
                start = 7 * SIDES.index(side)
                # An echoed TCP refers to the encoders of that same observation.
                # A blocked IK step holds the live configuration instead.
                goal = self.echo_arms.get(side, measured_q[start:start + 7])
                solved[side] = [min(ARM_UPPER[i], max(ARM_LOWER[i], q))
                                for i, q in enumerate(goal, start)]
        # A previous command can still be travelling even when the encoder
        # currently reads the new goal. Replace that command if its goal changed.
        cmds = {}
        for i, side in enumerate(SIDES):
            requested = requested_jaws[side]
            gap = clamped.get(side, requested)
            if side in echo_jaws and side not in self.jaws:
                continue
            previous = self.jaws.get(side, measured[8 * i + 7])
            if gap != previous:
                cmds[side] = command("open", {"opening_mm": gap * 1000., "speed": 15, "force": 30}, side)
        planned_tcp = {side: self.state.kinematics.forward(side, joints) for side, joints in solved.items()}
        if not current():
            return
        for side, joint_goal in solved.items():
            self.arms[side] = (joint_goal, poses[side], stamp, self.deadline)
        if solved and (self.output is None or self.output.done()):
            self.output = asyncio.create_task(self._drive())
        self.held_at_boundary = held
        for side in echo_arms:
            self.poses.pop(side, None)
        self.poses.update(to_solve)
        for side in echo_jaws - cmds.keys():
            self.jaws.pop(side, None)
        for index, (side, cmd) in enumerate(cmds.items()):
            if index:
                # The first gripper reply yielded control to the SDK and other jobs.
                if self.paused or self.reader.ws is not session:
                    raise MoveError("Interrupted EEF command or changed robot connection", None)
                try:
                    self._ready()
                    self.state.values()
                except ValueError as exc:
                    raise MoveError(str(exc), None) from exc
            if not current():
                return
            send = self.gripper_send = asyncio.create_task(self.reader.send_gripper(cmd))
            try:
                await asyncio.shield(send)
            finally:
                if send.done() and self.gripper_send is send:
                    self.gripper_send = None
            if side in echo_jaws:
                self.jaws.pop(side, None)
            else:
                self.jaws[side] = clamped.get(side, requested_jaws[side])
        if not current():
            return
        self.accepted += 1
        self.last = {"status": "accepted", "stamp_ms": stamp, "planned_tcp": planned_tcp,
                     "clamped_opening_m": clamped, "held_at_boundary": sorted(held)}

    async def _drive(self):
        try:
            self._ready()
            _, initial, speeds = self.state.joint_snapshot()
            if max(map(abs, speeds)) > .05:
                raise ValueError("Cannot take over a moving arm or head")
            limiter = Limiter(initial)
            target = list(initial)
            with self.reader.pose_feedback(), self.reader.pose_servo({"motion_tool": "servo_eef"}) as token:
                self.token = token
                last = time.monotonic()
                next_check = last
                while self.arms and not self.paused:
                    now = time.monotonic()
                    for side, goal in self.arms.copy().items():
                        if now >= goal[3]:
                            del self.arms[side]
                            self.poses[side] = None
                            start = 7 * SIDES.index(side)
                            target[start:start + 7] = limiter.q[start:start + 7]
                            limiter.v[start:start + 7] = [0.] * 7
                    if not self.arms:
                        break
                    if token.get("error"):
                        raise ValueError(token["error"])
                    if now >= next_check:
                        self._ready()
                        next_check = now + .05
                    current = self.state.joints()
                    if max(abs(a - b) for a, b in zip(current, limiter.q)) > TRACKING_LIMIT:
                        raise ValueError("Tracking error exceeded the TRON2 servo limit")
                    goals = self.arms.copy()
                    for side, goal in goals.items():
                        start = 7 * SIDES.index(side)
                        target[start:start + 7] = goal[0]
                    dt = now - last
                    if dt > MAX_OUTPUT_GAP:
                        raise ValueError("ServoJ output missed its deadline")
                    command_q = _command_step(limiter, target, max(dt, .000001))
                    await self.reader.send_arm_servoj(command_q, token)
                    for side, goal in goals.items():
                        start = 7 * SIDES.index(side)
                        if (self.arms.get(side) is goal and command_q[start:start + 7] == goal[0]
                                and not any(limiter.v[start:start + 7])):
                            del self.arms[side]
                    if not self.arms:
                        break
                    last = now
                    await asyncio.sleep(max(0., 1 / OUTPUT_HZ - (time.monotonic() - now)))
        except Exception as exc:
            self._fault(exc)
            self.last["arm_command_stamps"] = {side: goal[2] for side, goal in self.arms.items()}
        finally:
            self.token = None
            self.arms.clear()

    def _fault(self, exc, stamp=None):
        self.paused = True
        self.last = {"status": "fault", "reason": str(exc)[:300]}
        if stamp is not None:
            self.last["stamp_ms"] = stamp

    async def _run(self, queue):
        while True:
            frame = await queue.get()
            stamp = frame[0]
            try:
                if not self.paused:
                    await self._apply(*frame)
            except MoveError as exc:
                self._fault(exc, stamp)
            except (ValueError, TypeError, OverflowError, RecursionError) as exc:
                self.rejected += 1
                self.last = {"status": "rejected", "reason": str(exc)[:300], "stamp_ms": stamp}
            except Exception as exc:
                self._fault(exc, stamp)

    def _echoed_groups(self, frame):
        _, poses, jaws, _, observed = frame
        arms, grippers = {}, set()
        source = self.state.observation(observed) if observed is not None else None
        if source is not None:
            eef, joints = source
            for i, side in enumerate(SIDES):
                start = 8 * i
                if poses[side] == canonical_pose(eef[start:start + 7]):
                    arms[side] = joints[7*i:7*i + 7]
                if jaws[side] == eef[start + 7]:
                    grippers.add(side)
        return arms, grippers

    def _clear_stream(self):
        self.latest = None
        if self.pending_ik is not None:
            self.pending_ik.cancel()
        for side in self.arms:
            # The last generated point may be in flight. A new frame may hold or retarget it.
            self.poses[side] = None
        self.arms.clear()
        if self.queue is not None and not self.queue.empty():
            self.queue.get_nowait()

    async def _watch(self):
        while True:
            outcome = self.sink.tick()
            if outcome is not None and not self.paused:
                self.last = {"status": outcome.verdict.value, "reason": outcome.reason}
            await asyncio.sleep(.02)

    def _subscribe(self):
        loop = asyncio.get_running_loop()
        queue = asyncio.Queue(maxsize=1)

        # ControlSink calls apply synchronously within submit, on this event loop.
        def apply(values, _gripper):
            stamp, observed = message["stamp_ms"], message.get("obs_stamp_ms")
            expires_ms = stamp + message["ttl_ms"]
            if observed is not None:
                expires_ms = min(expires_ms, observed + MAX_OBS_AGE_MS)
            # Only another frame for this arm target can extend its SDK output window.
            remaining_ms = min(WATCHDOG_MS, expires_ms - time.time_ns() / 1_000_000)
            self.deadline = time.monotonic() + remaining_ms / 1000
            poses = {s: canonical_pose(list(values[i:i + 7])) for s, i in zip(SIDES, (0, 8))}
            jaws = dict(zip(SIDES, (values[7], values[15])))
            clamped = {side: jaws[side] for side in overflow}
            jaws.update(overflow)  # An echoed measurement may itself exceed nominal stroke.
            frame = (stamp, poses, jaws, clamped, observed)
            self.last_stamp = stamp
            if self.latest is None or self.latest[1:] != frame[1:] or self.target_session is not self.reader.ws:
                self.echo_arms, self.echo_jaws = self._echoed_groups(frame)
            self.latest = frame
            for side, goal in self.arms.items():
                if poses[side] == goal[1]:
                    self.arms[side] = (*goal[:3], self.deadline)
            if self.pending_ik is not None and len(self.echo_arms) == len(self.echo_jaws) == len(SIDES):
                self.pending_ik.cancel()
            if queue.full():
                queue.get_nowait()
                self.overruns += 1
            queue.put_nowait(frame)

        message = overflow = None

        def offer(raw):
            nonlocal message, overflow
            if self.queue is not queue or self.paused:
                return
            try:
                message, overflow = decode(raw)
                outcome = self.sink.submit(message)
                if not outcome.applied:
                    raise ValueError(outcome.reason)
            except (ValueError, TypeError, OverflowError, RecursionError) as exc:
                self.rejected += 1
                self.last = {"status": "rejected", "reason": str(exc)[:300]}

        self.sink = ControlSink(parse_descriptor(self.descriptor), apply, on_watchdog=self._clear_stream)
        self.queue = queue
        self.subscription = self.publisher.subscribe_json(self.topic, lambda raw: loop.call_soon_threadsafe(offer, raw))
        self.worker = asyncio.create_task(self._run(queue))
        self.watchdog = asyncio.create_task(self._watch())

    async def preempt(self, *, interrupt_gripper=False):
        if not self.running:
            return
        self.paused = True
        self.queue = None
        if self.subscription is not None:
            self.publisher.unsubscribe(self.subscription)
            self.subscription = None
        tasks = [task for task in (self.worker, self.output, self.watchdog) if task is not None]
        pending = self.gripper_send
        if interrupt_gripper and pending is not None:
            # A single gripper ACK can race a stop of a multi-group command.
            # It cannot confirm delivery of the complete frame.
            if not pending.done():
                pending.cancel()
            self.last = {"status": "outcome_unknown", "stamp_ms": self.last_stamp,
                         "reason": "Command output stopped during gripper dispatch"}
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        if pending is not None:
            await asyncio.gather(pending, return_exceptions=True)
            self.gripper_send = None
        self.worker = self.output = self.watchdog = None
        self.arms.clear()
        self.jaws.clear()
        self.poses.clear()
        self.held_at_boundary.clear()
        self.latest = self.target_session = None

    async def call(self, args):
        action = args.get("action", "info")
        if action == "info":
            return self.info()
        if action in ("stop", "pause"):
            await self.preempt(interrupt_gripper=True)
            if action == "stop":
                self.running = False
            return self.info()
        if action not in ("start", "resume"):
            raise ValueError("Unsupported servo_eef action")
        if action == "start" and self.running or action == "resume" and not self.running:
            return self.info()
        if action == "resume" and not self.paused:
            return self.info()
        if self.publisher is None:
            raise ValueError("EEF subscription has no ROS publisher")
        if self.running:
            await self.preempt()
        else:
            topic = args.get("input_topic") or (args.get("input_topics") or [CONTROL_TOPIC])[0]
            if not isinstance(topic, str) or not topic.startswith("/") or len(topic) > 256:
                raise ValueError("Expected an absolute control/eef topic name")
            self.topic = topic
        self.adapter.arm_servo._conflict()
        self.paused = False
        self._subscribe()
        self.running = True
        return self.info()

    async def close(self):
        await self.call({"action": "stop"})
