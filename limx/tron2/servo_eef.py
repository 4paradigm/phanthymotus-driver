"""Receive motus.control/1 targets and dispatch paired native TRON2 MoveP."""
import asyncio
import json
import logging
import math
import os
import time
from pathlib import Path

from common.control import ControlSink, SCHEMA, parse_descriptor, rotation
from arm_servo import MAX_HZ, WATCHDOG_MS
from eef_state import CONTROL_TOPIC, FRAME, GRIPPER_MAX_AGE, SCHEMA as STATE_SCHEMA, SIDES
from gripper import MAX_OPENING_MM, command
from vendor import MOVE_P_BOUNDS, MoveError, motion_readiness, reject_constant

MAX_FUTURE_MS = 100
# A submitted native point move can finish after the publisher goes silent.
# Bound that commitment in both time and Cartesian distance.
MOVE_TIME_S = WATCHDOG_MS / 1000
LINEAR_M_S = .10
ANGULAR_RAD_S = .50
MAX_GAP_M = MAX_OPENING_MM / 1000
STREAM_MESSAGES = {
    "stream_interrupted": "servo_eef 控制流中断，已停止下发新的运动段",
    "stream_resumed": "servo_eef 已收到新的有效目标，控制流恢复",
}


def build_descriptor():
    names, groups, lower, upper = [], [], [], []
    for i, side in enumerate(SIDES):
        suffix = side[0]
        names.extend(f"eef_{suffix}_{axis}" for axis in ("x", "y", "z", "qx", "qy", "qz", "qw"))
        names.append(f"gripper_{suffix}")
        groups.extend([{"name": f"eef_{suffix}", "offset": 8*i, "count": 7, "unit": "m+quat_xyzw",
                        "mode": "eef_pose", "resource": f"arm_{suffix}"},
                       {"name": f"gripper_{suffix}", "offset": 8*i+7, "count": 1, "unit": "m",
                        "mode": "joint_position", "resource": f"gripper_{suffix}"}])
        xyz_min, xyz_max = zip(*MOVE_P_BOUNDS[side])
        lower.extend([*xyz_min, -1., -1., -1., -1., 0.])
        upper.extend([*xyz_max, 1., 1., 1., 1., MAX_GAP_M])
    return {"control_interface": SCHEMA, "mode": "eef_pose", "dof": len(names), "frame": FRAME,
            "joint_names": names, "groups": groups, "units": {"length": "m", "rotation": "quat_xyzw", "time": "s"},
            "limits": {"lower": lower, "upper": upper}, "force_torque": None,
            "tip_frames": {side: f"MoveP_{side}_TCP" for side in SIDES}, "opening_m": [0., MAX_GAP_M],
            "state": {"schema": STATE_SCHEMA, "field": "eef"},
            "end_effector": {"workspace_m": MOVE_P_BOUNDS, "pose_source": "request_get_move_pose"},
            "rate": {"expected_hz": 10, "max_hz": MAX_HZ, "watchdog_ms": WATCHDOG_MS,
                     "max_obs_age_ms": int(GRIPPER_MAX_AGE * 1000)}}


def canonical_pose(pose):
    q = rotation.normalize(pose[3:])
    sign = next(v for v in reversed(q) if v)
    return (*pose[:3], *(v * math.copysign(1., sign) for v in q))


def waypoint(side, start, goal):
    """Commit at most one watchdog period of Cartesian motion toward an admitted goal."""
    distance = math.dist(start[:3], goal[:3])
    angle = rotation.angle_between(start[3:], goal[3:])
    portion = min(1., LINEAR_M_S * MOVE_TIME_S / distance if distance else 1.,
                  ANGULAR_RAD_S * MOVE_TIME_S / angle if angle else 1.)
    if portion == 1.:
        return goal, False
    xyz = tuple(a + portion * (b-a) for a, b in zip(start[:3], goal[:3]))
    if any(not lo <= x <= hi for x, (lo, hi) in zip(xyz, MOVE_P_BOUNDS[side])):
        raise ValueError(f"{side} MoveP cannot take a bounded step from the measured pose")
    q, _ = rotation.slerp_limit(start[3:], goal[3:], angle * portion)
    return (*xyz, *q), True


class EefServo:
    def __init__(self, adapter, state):
        self.adapter, self.reader, self.state = adapter, adapter.reader, state
        self.publisher = self.subscription = self.sink = None
        self.descriptor = build_descriptor()
        self.worker = self.watchdog = self.wake = self.latest = None
        self.running = self.paused = False
        self.topic = CONTROL_TOPIC
        self.arm_intent = (None, None)
        self.commanded = (None, None)
        self.more = False
        self.arm_error = None
        self.activity = None
        self.next_segment = 0.
        self.jaw_targets = {}
        self.active_arms = []
        self.inflight = self.session = self.last = None
        self.accepted = self.rejected = self.overruns = 0

    def tool(self):
        return {"name": "servo_eef", "type": "actuator", "multiInstance": False,
                "description": "motus.control/1 absolute TCP and opening targets: left pose (7), opening (1), "
                               "right pose (7), opening (1), metres and xyzw in base_Link. Uses paired native MoveP.",
                "inputSchema": {"type": "object", "required": ["action"], "properties": {
                    "action": {"type": "string", "enum": ["start", "stop", "pause", "resume", "info"]},
                    "input_topic": {"type": "string", "default": CONTROL_TOPIC}},
                    "x-action-params": {
                        "start": {"params": ["input_topic"], "description": "Subscribe to the EEF control topic."},
                        "stop": {"params": [], "description": "Unsubscribe and stop accepting EEF targets."},
                        "pause": {"params": [], "description": "Pause the stream and clear queued targets."},
                        "resume": {"params": [], "description": "Resume on the same topic with fresh EEF targets."},
                        "info": {"params": [], "description": "Read the stream status and control descriptor."}},
                    "x-hooks": {"on_interrupt_motion": {"action": "pause"}, "on_interrupt_all": {"action": "pause"}},
                    "x-is-dangerous": True, "x-resource": ["arm_l", "arm_r", "gripper_l", "gripper_r"]},
                "topic_in": [{"format": "control/eef", "topic": CONTROL_TOPIC}]}

    def info(self):
        return {"state": "idle" if not self.running else "paused" if self.paused else "ready",
                "paused": self.paused, "topic_in": [{"format": "control/eef", "topic": self.topic}],
                "schema": SCHEMA, "frame": FRAME, "watchdog_ms": WATCHDOG_MS,
                "control_interface": self.descriptor, "sink": self.sink.stats() if self.sink else {},
                "accepted": self.accepted, "rejected": self.rejected, "queue_full": self.overruns,
                "active_arms": list(self.active_arms), "last": self.last, "activity": self.activity}

    def _ready(self, *, check_owner=True):
        if check_owner:
            self.adapter.arm_servo._conflict()
        status = self.reader.sample("robot_status")
        data = status.get("data") or {}
        if (Path(os.environ.get("TRON2_MOTION_MAINTENANCE_FILE", "/config/tron2/motion-maintenance.json")).exists()
                or self.reader.configuration.get("mode") != "upper_limb" or not status["available"]
                or data.get("motor") != "OK" or not motion_readiness(data)["ready"]):
            raise ValueError("DACH upper-limb motion is unavailable")

    def _announce(self, event):
        self.activity = {"event": event, "text": STREAM_MESSAGES[event], "stamp_ms": time.time_ns() // 1_000_000}
        logging.getLogger(__name__).info("%s %s", event, self.activity["text"])

    def _subscribe(self):
        loop = asyncio.get_running_loop()
        self.wake = wake = asyncio.Event()
        message = None

        def clear():
            if self.latest is not None:
                self._announce("stream_interrupted")
            self.latest = None
            wake.clear()

        def accept(values, _gripper):
            if sink.holding:
                self._announce("stream_resumed")
            stamp = message["stamp_ms"]
            now_ms = time.time_ns() // 1_000_000
            remaining_ms = min(WATCHDOG_MS, stamp + message["ttl_ms"] - now_ms)
            if message.get("obs_stamp_ms") is not None:
                remaining_ms = min(remaining_ms, message["obs_stamp_ms"] + layout.max_obs_age_ms - now_ms)
            self.latest = (values, stamp, time.monotonic() + remaining_ms/1000)
            self.overruns += int(wake.is_set())
            wake.set()

        layout = parse_descriptor(self.descriptor)
        self.sink = sink = ControlSink(layout, accept, on_watchdog=clear, on_abort=clear)

        async def watch():
            while True:
                if not self.paused:
                    sink.tick()
                await asyncio.sleep(.02)

        def offer(raw):
            nonlocal message
            if self.wake is not wake or self.paused:
                return
            try:
                if not isinstance(raw, str) or len(raw) > 8192:
                    raise ValueError("Expected a JSON control frame of at most 8 KiB")
                message = json.loads(raw, parse_constant=reject_constant)
                if not isinstance(message, dict) or message.get("frame", FRAME) != FRAME or message.get("gripper") is not None:
                    raise ValueError("Expected both grippers in values and TCPs in base_Link")
                now = time.time_ns() // 1_000_000
                for key in ("stamp_ms", "obs_stamp_ms"):
                    stamp = message.get(key)
                    if stamp is not None and (type(stamp) is not int or not 0 < stamp <= now + MAX_FUTURE_MS):
                        raise ValueError(f"Invalid {key}")
                sink.tick()
                outcome = sink.submit(message)
                if not outcome.applied:
                    self.rejected += 1
                    self.last = {"status": outcome.verdict.value, "reason": outcome.reason}
            except (ValueError, TypeError, OverflowError, RecursionError) as error:
                self.rejected += 1
                self.last = {"status": "rejected", "reason": str(error)[:300]}

        self.subscription = self.publisher.subscribe_json(self.topic, lambda raw: loop.call_soon_threadsafe(offer, raw))
        self.worker = asyncio.create_task(self._run(wake))
        self.watchdog = asyncio.create_task(watch())

    def _valid(self, frame):
        return (not self.paused and self.latest is not None and self.latest[0] == frame[0]
                and self.reader.ws is self.session and time.monotonic() < self.latest[-1])

    async def _arms(self, values, measured):
        poses = tuple(canonical_pose(values[i:i+7]) for i in (0, 8))
        starts = tuple(canonical_pose(measured[i:i+7]) for i in (0, 8))
        intent = tuple(None if goal == start else goal for goal, start in zip(poses, starts))
        now = time.monotonic()
        repeated = intent == self.arm_intent
        if repeated and (not self.more or now < self.next_segment):
            return
        self.arm_intent, self.more, self.arm_error = intent, False, None
        if intent == (None, None) and all(last is None or goal == last
                                           for goal, last in zip(poses, self.commanded)):
            self.commanded = (None, None)
            return
        pair, continuing = [], False
        for side, goal, requested, start in zip(SIDES, poses, intent, starts):
            if requested is None:
                point = start
                if any(not lo <= v <= hi for v, (lo, hi) in zip(point, MOVE_P_BOUNDS[side])):
                    raise ValueError(f"Cannot send paired MoveP: held {side} arm is outside its vendor workspace")
            else:
                point, more = waypoint(side, start, goal)
                continuing |= more
            pair.append(point)
        if tuple(pair) == starts and self.commanded == (None, None):
            return
        commanded = tuple(point if target is not None else None for point, target in zip(pair, intent))
        if repeated and commanded == self.commanded:
            self.more = continuing
            return
        self.inflight = "MoveP"
        self.active_arms = [side for side, old, target in zip(SIDES, self.commanded, intent)
                            if old is not None or target is not None]
        try:
            with self.reader.pose_servo({"motion_tool": "servo_eef"}) as token:
                self.next_segment = now + MOVE_TIME_S
                await self.reader.send_move_p(*pair, MOVE_TIME_S, token)
        finally:
            self.active_arms = []
            self.inflight = None
        self.more = continuing
        self.commanded = commanded

    async def _apply(self, frame):
        values, stamp, _ = frame
        self._ready()
        if self.session is not self.reader.ws:
            self.session = self.reader.ws
            self.arm_intent = (None, None)
            self.commanded = (None, None)
            self.arm_error = None
            self.more = False
            self.jaw_targets.clear()
        measured = self.state.values()
        if not self._valid(frame):
            return
        try:
            await self._arms(values, measured)
        except ValueError as error:
            if isinstance(error, MoveError) and (error.motion_sent is None or error.code == "outcome_unknown"):
                raise
            self.arm_error = str(error)
        jaw_sent = []
        for side, i in zip(SIDES, (7, 15)):
            if not self._valid(frame):
                return
            gap = values[i]
            if gap == measured[i] and side not in self.jaw_targets:
                continue
            if gap == self.jaw_targets.get(side, measured[i]):
                continue
            self._ready()
            self.state.values()
            self.inflight = side + " gripper"
            try:
                await self.reader.send_gripper(command("open", {"opening_mm": gap*1000, "speed": 15, "force": 30}, side))
            finally:
                self.inflight = None
            if gap == measured[i]:
                self.jaw_targets.pop(side, None)
            else:
                self.jaw_targets[side] = gap
            jaw_sent.append(side)
        if not self._valid(frame):
            return
        status = "accepted"
        if self.arm_error:
            status = "partial" if jaw_sent else "rejected"
        self.accepted += int(status != "rejected")
        self.rejected += int(status != "accepted")
        self.last = {"status": status, "stamp_ms": stamp, "grippers_sent": jaw_sent}
        if self.arm_error:
            self.last["arm_error"] = self.arm_error

    async def _run(self, wake):
        while True:
            await wake.wait()
            wake.clear()
            frame = self.latest
            if frame is None or self.paused:
                continue
            try:
                await self._apply(frame)
            except MoveError as error:
                if error.motion_sent is not False:
                    self.paused = True
                self.rejected += 1
                uncertain = self.paused and error.code != "vendor_rejected"
                self.last = {"status": "outcome_unknown" if uncertain else "rejected", "reason": str(error)}
            except (ValueError, TypeError, OverflowError) as error:
                self.rejected += 1
                self.last = {"status": "rejected", "reason": str(error)}
            except Exception as error:
                self.paused = True
                self.last = {"status": "fault", "reason": str(error)}

    async def preempt(self, *, interrupt_gripper=False):
        self.paused = True
        self.latest = self.wake = None
        if self.subscription is not None:
            self.publisher.unsubscribe(self.subscription)
            self.subscription = None
        tasks = [t for t in (self.worker, self.watchdog) if t is not None]
        if self.inflight:
            self.last = {"status": "outcome_unknown", "reason": "Stopped while awaiting " + self.inflight}
        if self.inflight == "MoveP":
            self.next_segment = time.monotonic() + MOVE_TIME_S
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        if self.next_segment > time.monotonic():
            await asyncio.sleep(self.next_segment - time.monotonic())
        self.worker = self.watchdog = None
        self.more = False
        self.arm_error = None
        self.arm_intent = (None, None)
        self.commanded = (None, None)
        self.jaw_targets.clear()

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
        if action == "start" and self.running or action == "resume" and (not self.running or not self.paused):
            if self.running and not self.paused:
                # A repeated start/resume already owns its stream and may be sending MoveP.
                self._ready(check_owner=False)
            return self.info()
        if self.running:
            await self.preempt()
        self._ready()
        if not self.running:
            topics = args.get("input_topics") or [CONTROL_TOPIC]
            topic = args.get("input_topic") or topics[0]
            if not isinstance(topic, str) or not topic.startswith("/") or len(topic) > 256:
                raise ValueError("Expected an absolute control/eef topic name")
            self.topic = topic
        self._subscribe()
        self.running, self.paused = True, False
        return self.info()

    async def close(self):
        await self.call({"action": "stop"})
