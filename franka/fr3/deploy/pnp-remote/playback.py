"""Run a recorded Teach session through the workstation's Polymetis backend."""
from datetime import datetime
import math
from pathlib import Path
import pickle
import subprocess
import time

from execution_status import report
from run_vendor import SCRIPTS, VENDOR_ROOT, stop_vendor, vendor_environment
from vendor_recordings import GRIPPER_OPEN_WIDTH_M, NAME, PLAYBACK_TIMESTAMP

ROOT = Path(__file__).parent
PAUSE = ROOT / "playback.pause"
START = ROOT / "playback.start"
APPROACH = ROOT / "playback.approach"
PERIOD = .02
TRACKING_LIMIT_RAD = math.radians(90)
ENTRY_LIMIT_RAD = .2


def load_trajectory(name):
    import numpy as np

    match = NAME.fullmatch(name)
    if not match:
        raise ValueError("Invalid Teach trajectory name")
    folder = Path.home() / "bc_data" / "teleop" / name
    files = sorted(folder.glob("*.pkl"))
    if folder.is_symlink() or not 2 <= len(files) <= 60000:
        raise ValueError("Playback requires a complete recording with 2–60000 frames")
    frames = []
    for path in files:
        if path.is_symlink() or path.stat().st_size > 1024 * 1024:
            raise ValueError("Invalid vendor frame file")
        with path.open("rb") as frame:
            record = pickle.load(frame)
        measured = np.asarray(record["joint_positions"], dtype=float)
        command = np.asarray(record["control"], dtype=float)
        if measured.shape != (8,) or command.shape != (8,):
            raise ValueError(f"Invalid FR3 joint or gripper data in {path.name}")
        # Replay the follower's actual arm motion and the original gripper intent.
        target = measured.copy()
        target[7] = command[7]
        clock = record.get(PLAYBACK_TIMESTAMP)
        if clock is not None and (type(clock) is not int or clock <= 0):
            raise ValueError(f"Invalid FR3 observation timestamp in {path.name}")
        frames.append((clock, path, target))

    if all(clock is not None for clock, _, _ in frames):
        frames.sort(key=lambda frame: frame[0])
        first = frames[0][0]
        stamps = np.array([(clock - first) / 1e9 for clock, _, _ in frames])
    elif all(clock is None for clock, _, _ in frames):
        # Older vendor recordings already contain measured joints and wall-clock filenames.
        dates = [datetime.fromisoformat(path.stem) for _, path, _ in frames]
        stamps = np.array([(date - dates[0]).total_seconds() for date in dates])
    else:
        raise ValueError("FR3 recording mixes observation timestamp formats")
    points = np.stack([target for _, _, target in frames])
    gaps = np.diff(stamps)
    lower = np.array([-2.9007, -1.8361, -2.9007, -3.077, -2.8763, .4398, -3.0508])
    upper = np.array([2.9007, 1.8361, 2.9007, -.1169, 2.8763, 4.6216, 3.0508])
    if (points.shape != (len(files), 8) or not np.isfinite(points).all()
            or (points[:, :7] < lower + .01).any() or (points[:, :7] > upper - .01).any()
            or (points[:, 7] < 0).any() or (points[:, 7] > 1).any()
            or (gaps <= 0).any() or (gaps > .25).any()):
        raise ValueError("Recording contains invalid joint targets or gaps")
    if stamps[-1] > 600:
        raise ValueError("Recorded trajectory exceeds ten minutes")
    return match[1], stamps, points


def _fresh(timestamp):
    age = time.time() - (timestamp.seconds + timestamp.nanos * 1e-9)
    return 0 <= age <= 1


def _ready_joints(state):
    return (_fresh(state.timestamp) and len(state.joint_positions) == 7
            and len(state.joint_velocities) == 7
            and all(map(math.isfinite, state.joint_positions))
            and all(map(math.isfinite, state.joint_velocities)))


def _check_backends(children):
    for name, process in children:
        code = process.poll()
        if code is not None:
            raise RuntimeError(f"PNP playback backend {name} exited with status {code}")


def _wait_for_feedback(children, backend, probe):
    import grpc

    deadline = time.monotonic() + 30
    last_error = "no fresh state received"
    while time.monotonic() < deadline:
        _check_backends(children)
        try:
            if probe():
                return
        except grpc.RpcError as exc:
            last_error = exc.details() or str(exc)
        time.sleep(.1)
    raise TimeoutError(f"PNP playback {backend} feedback not ready: {last_error}")


def _start_backends(arm, children):
    import grpc
    import polymetis_pb2 as pb
    import polymetis_pb2_grpc as rpc

    env = vendor_environment(arm)
    vendor = VENDOR_ROOT / f"{arm.side}_franka"

    def start(script):
        process = subprocess.Popen(["/bin/bash", script], cwd=vendor, env=env,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
        children.append((script, process))

    report("playback_starting", side=arm.side, backend="arm")
    start(SCRIPTS[0])
    with grpc.insecure_channel(f"127.0.0.1:{arm.arm_port}") as channel:
        arm_rpc = rpc.PolymetisControllerServerStub(channel)
        _wait_for_feedback(children, "arm", lambda: _ready_joints(
            arm_rpc.GetRobotState(pb.Empty(), timeout=.3)))

    report("playback_starting", side=arm.side, backend="gripper")
    start(SCRIPTS[1])
    with grpc.insecure_channel(f"127.0.0.1:{arm.hand_port}") as channel:
        gripper_rpc = rpc.GripperServerStub(channel)
        _wait_for_feedback(children, "gripper", lambda: _fresh(
            gripper_rpc.GetState(pb.Empty(), timeout=.3).timestamp))


def run(arm, stamps, points):
    import numpy as np
    import polymetis_pb2 as pb
    import torch
    from polymetis import GripperInterface, RobotInterface
    from torchcontrol.planning import generate_joint_space_min_jerk

    children = []
    robot = None
    policy = False
    try:
        _start_backends(arm, children)
        robot = RobotInterface(ip_address="127.0.0.1", port=arm.arm_port)
        hand = GripperInterface(ip_address="127.0.0.1", port=arm.hand_port)

        def read_robot_state():
            state = robot.grpc_connection.GetRobotState(pb.Empty(), timeout=.3)
            if not _ready_joints(state):
                raise RuntimeError("PNP playback requires fresh seven-axis position and velocity feedback")
            return state

        def feedback(target, tolerance):
            state = read_robot_state()
            actual = np.asarray(state.joint_positions)
            velocity = np.asarray(state.joint_velocities)
            errors = np.abs(actual - target)
            joint = int(np.argmax(errors))
            error = errors[joint]
            if error > tolerance:
                raise RuntimeError(
                    f"FR3 {arm.side} playback J{joint + 1} tracking error "
                    f"{math.degrees(error):.4f}° exceeds {math.degrees(tolerance):.4f}° "
                    f"(actual {actual[joint]:.5f} rad, target {target[joint]:.5f} rad)")
            return error, np.max(abs(velocity))

        def settle(target, phase):
            deadline = time.monotonic() + 4
            stable = 0
            while True:
                _check_backends(children)
                error, speed = feedback(target, TRACKING_LIMIT_RAD)
                stable = stable + 1 if error <= .04 and speed <= .025 else 0
                if stable >= 5:
                    return
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"FR3 {arm.side} did not settle at {phase}: "
                                       f"error={error:.4f} rad, speed={speed:.4f} rad/s")
                time.sleep(PERIOD)

        def wait_for_gate(gate, target, tolerance):
            start_time = None
            while start_time is None or time.monotonic() < start_time:
                _check_backends(children)
                feedback(target, tolerance)
                try:
                    start_time = float(gate.read_text())
                except FileNotFoundError:
                    pass
                time.sleep(.005)
            return start_time

        actual = np.asarray(read_robot_state().joint_positions)
        first = points[0, :7]
        delta = np.abs(actual - first)
        joint = int(np.argmax(delta))
        if delta[joint] > ENTRY_LIMIT_RAD:
            raise RuntimeError(f"FR3 {arm.side} J{joint + 1} is {delta[joint]:.3f} rad from the "
                               f"first recorded pose; limit is {ENTRY_LIMIT_RAD:.2f} rad")
        report("playback_entry_ready", side=arm.side, recorded_entry_s=0.)
        wait_for_gate(APPROACH, first, ENTRY_LIMIT_RAD)
        actual = np.asarray(read_robot_state().joint_positions)
        if np.max(np.abs(actual - first)) > ENTRY_LIMIT_RAD:
            raise RuntimeError(f"FR3 {arm.side} moved outside the first-frame entry limit")
        approach = generate_joint_space_min_jerk(
            start=torch.tensor(actual, dtype=torch.float32),
            goal=torch.tensor(first, dtype=torch.float32),
            time_to_go=4., hz=1 / PERIOD,
        )
        report("playback_approaching", side=arm.side, recorded_entry_s=0.)
        robot.start_joint_impedance()
        policy = True
        for waypoint in approach:
            tick = time.monotonic()
            _check_backends(children)
            target = waypoint["position"]
            feedback(target.numpy(), TRACKING_LIMIT_RAD)
            robot.update_desired_joint_positions(target)
            time.sleep(max(0., tick + PERIOD - time.monotonic()))
        settle(first, "recorded start")
        report("playback_aligned", side=arm.side, duration_s=float(stamps[-1]))
        start_time = wait_for_gate(START, first, .04)
        progress, rate, last_hand = 0., 1., None
        duration = float(stamps[-1])
        last_tick = start_time
        while True:
            _check_backends(children)
            now = time.monotonic()
            elapsed = now - last_tick
            last_tick = now
            paused = PAUSE.exists()
            change = 2 * elapsed
            next_rate = max(0., rate - change) if paused else min(1., rate + change)
            progress = min(duration, progress + (rate + next_rate) * .5 * elapsed)
            rate = next_rate
            target = np.array([np.interp(progress, stamps, column) for column in points.T])
            feedback(target[:7], TRACKING_LIMIT_RAD)
            robot.update_desired_joint_positions(torch.tensor(target[:7], dtype=torch.float32))
            width = GRIPPER_OPEN_WIDTH_M * (1 - target[7])
            if last_hand is None or abs(width - last_hand) >= .002:
                hand.goto(width=float(width), speed=.04, force=10, blocking=False)
                last_hand = width
            if paused and rate == 0:
                settle(target[:7], "pause point")
                report("playback_paused", position_s=progress, duration_s=duration, side=arm.side)
                while PAUSE.exists():
                    _check_backends(children)
                    feedback(target[:7], .04)
                    report("playback_paused", position_s=progress, duration_s=duration, side=arm.side)
                    time.sleep(.1)
                settle(target[:7], "resume point")
                last_tick = time.monotonic()
            else:
                report("playback_running", position_s=progress, duration_s=duration, side=arm.side)
            if progress >= duration:
                settle(target[:7], "trajectory end")
                break
            time.sleep(max(0., last_tick + PERIOD - time.monotonic()))
    finally:
        try:
            if policy:
                robot.terminate_current_policy()
        finally:
            stop_vendor(children)
    report("playback_completed", side=arm.side, duration_s=duration)
