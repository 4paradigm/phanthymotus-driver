"""Start the PNP arm scripts under the existing systemd service."""
import json
import os
from pathlib import Path
import re
import select
import signal
import socket
import subprocess
import time

from execution_status import report

VENDOR_ROOT = Path(os.environ.get("PNP_VENDOR_ROOT", str(Path.home() / "frankateleop")))
SCRIPTS = (
    "1_launch_robot.sh", "2_launch_gripper.sh", "3_launch_node.sh", "4_run_env.sh",
)


def stop_vendor(children):
    """Interrupt the vendor scripts in reverse startup order."""
    for name, process in reversed(children):
        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            continue
        print(f"Stopping {name} with SIGINT", flush=True)
        try:
            process.wait(timeout=.75)
        except subprocess.TimeoutExpired:
            # systemd reaps anything still alive in the service cgroup.
            pass


def vendor_environment(arm):
    env = os.environ.copy()
    runtime = Path(__file__).parent
    build = VENDOR_ROOT / "polymetis/polymetis/build"
    env["FR3_VENDOR_NODE"] = str((VENDOR_ROOT / "teleop/experiments/launch_nodes.py").resolve())
    env["FR3_VENDOR_ARM_PORT"] = str(arm.arm_port)
    env["FR3_VENDOR_SERVER"] = str(build / "run_server")
    env["FR3_VENDOR_CLIENT"] = str(build / "franka_panda_client")
    env["PATH"] = os.pathsep.join((str(runtime / "vendor-bin"),
                                  str(Path(os.environ.get("PNP_CONDA_BASE", str(Path.home() / "miniconda3"))) / "bin"), env["PATH"]))
    env["PYTHONUNBUFFERED"] = "1"
    env["PYTHONPATH"] = str(runtime) + os.pathsep + env.get("PYTHONPATH", "")
    env["FR3_ROBOT_IP"] = arm.robot_ip
    env["BASH_ENV"] = str(runtime / "vendor-follow-env.sh")
    return env


def run_vendor(arm):
    import grpc
    import polymetis_pb2 as pb
    import polymetis_pb2_grpc as rpc

    env = vendor_environment(arm)
    runtime = Path(__file__).parent
    vendor = VENDOR_ROOT / f"{arm.side}_franka"
    gate = runtime / f"follow-gate-{arm.side}"
    gate.unlink(missing_ok=True)
    record_marker = runtime / f"record-{arm.side}"
    record_marker.unlink(missing_ok=True)
    env["FR3_RECORD_MARKER"] = str(record_marker)
    env["FR3_ARM_SIDE"] = arm.side
    env["FR3_FOLLOW_GATE"] = str(gate)
    for name in SCRIPTS:
        if not (vendor / name).is_file():
            raise RuntimeError(f"Missing PNP script: {name}")
    original = VENDOR_ROOT / "teleop/experiments/run_env.py"
    corrected = runtime / "right_run_env.py"
    if not corrected.is_file():
        raise RuntimeError("Missing prepared PNP follow program")
    children = []

    def start(name, capture=False):
        report("vendor_starting_script", script=name)
        script_env = env
        if name == SCRIPTS[3]:
            script_env = dict(env, BASH_ENV=str(runtime / "vendor-follow-env.sh"),
                              FR3_VENDOR_RUN_ENV=str(original.resolve()),
                              FR3_VENDOR_RUN_ENV_COPY=str(corrected))
        process = subprocess.Popen(["/bin/bash", name], cwd=vendor, env=script_env,
                                   stdin=subprocess.DEVNULL, start_new_session=True,
                                   stdout=subprocess.PIPE if capture else None,
                                   stderr=subprocess.STDOUT if capture else None)
        children.append((name, process))
        return process

    def check_children():
        for name, process in children:
            code = process.poll()
            if code is not None:
                raise RuntimeError(f"{name} exited with status {code}; check PNP journal")

    def wait_ready(probe):
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            check_children()
            try:
                if probe():
                    return
            except (OSError, grpc.RpcError):
                pass
            time.sleep(.1)
        raise TimeoutError(f"Readiness timeout for {children[-1][0]}")

    def node_ready():
        with socket.create_connection(("127.0.0.1", arm.node_port), timeout=.3):
            return True

    try:
        start(SCRIPTS[0])
        with grpc.insecure_channel(f"127.0.0.1:{arm.arm_port}") as channel:
            robot = rpc.PolymetisControllerServerStub(channel)
            wait_ready(lambda: len(robot.GetRobotState(pb.Empty(), timeout=.3).joint_positions) == 7)
        start(SCRIPTS[1])
        with grpc.insecure_channel(f"127.0.0.1:{arm.hand_port}") as channel:
            hand = rpc.GripperServerStub(channel)
            wait_ready(lambda: hand.GetState(pb.Empty(), timeout=.3).timestamp.seconds > 0)
        start(SCRIPTS[2])
        wait_ready(node_ready)
        process = start(SCRIPTS[3], capture=True)
        startup_deadline = time.monotonic() + 30
        last_progress = None
        first_progress = False
        buffer = b""
        while True:
            check_children()
            now = time.monotonic()
            if last_progress is None and now > startup_deadline:
                raise RuntimeError("PNP follow loop did not start within 30 seconds")
            if last_progress is not None and now - last_progress > 2:
                raise RuntimeError("PNP follow loop stopped reporting progress")
            if not select.select([process.stdout], [], [], .2)[0]:
                continue
            chunk = os.read(process.stdout.fileno(), 4096)
            if not chunk:
                raise RuntimeError("PNP follow script closed its output")
            lines = re.split(rb"[\r\n]", buffer + chunk)
            buffer = lines.pop()
            for line in lines:
                if line.startswith(b"PNP_PHASE="):
                    phase = json.loads(line.split(b"=", 1)[1])
                    stage = phase.pop("stage")
                    if stage not in ("vendor_aligning", "vendor_aligned"):
                        raise RuntimeError("Unexpected vendor alignment phase")
                    report(stage, **phase)
                    last_progress = now
                elif b"Time passed:" in line:
                    # A second iteration implies the previous synchronous env.step returned.
                    if first_progress and (last_progress is None or now - last_progress >= .25):
                        report("vendor_following", script=SCRIPTS[3])
                        last_progress = now
                    first_progress = True
                elif line:
                    print(f"[{arm.side}] {line.decode(errors='replace')}", flush=True)
    finally:
        gate.unlink(missing_ok=True)
        record_marker.unlink(missing_ok=True)
        stop_vendor(children)
