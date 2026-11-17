"""Detect and supervise the connected PNP masters."""
import json
import os
from pathlib import Path
import resource
import signal
import socket
import subprocess
import sys
import time

import execution_status
from arm_config import ARMS, detect
from execution_status import report, run_checked
from run_vendor import run_vendor

POLY = Path(os.environ.get("PNP_VENDOR_ROOT", str(Path.home() / "frankateleop"))) / "polymetis/polymetis"
ENV = Path(sys.executable).parent.parent
RUNTIME = Path(__file__).parent

def check(selected):
    if Path("/sys/kernel/realtime").read_text().strip() != "1":
        raise RuntimeError("PNP workstation must be running its realtime kernel")
    if resource.getrlimit(resource.RLIMIT_RTPRIO)[0] < 80:
        raise RuntimeError("PNP service has insufficient realtime scheduling permissions")
    if resource.getrlimit(resource.RLIMIT_MEMLOCK)[0] < 2 * 1024**3:
        raise RuntimeError("Service requires at least 2 GiB of locked memory for Polymetis")
    for arm in selected:
        for port in (arm.arm_port, arm.hand_port, arm.node_port):
            with socket.socket() as sock:
                sock.bind(("127.0.0.1", port))


def launch():
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    os.environ["PATH"] = os.pathsep.join(
        (str(ENV / "bin"), str(POLY / "build"), os.environ["PATH"]))
    os.environ["CONDA_PREFIX"] = str(ENV)
    os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(
        filter(None, (str(ENV / "lib"), os.environ.get("LD_LIBRARY_PATH"))))
    detected = detect()
    request = RUNTIME / "requested-arm"
    try:
        side = request.read_text().strip()
    except FileNotFoundError:
        raise RuntimeError("No arm selected by the Orin card")
    finally:
        request.unlink(missing_ok=True)
    if side != "both" and side not in ARMS:
        raise RuntimeError("Invalid PNP arm selection")
    sides = tuple(ARMS) if side == "both" else (side,)
    selected = [ARMS[name] for name in sides]
    unavailable = [name for name in sides if not detected[name]["ready"]]
    for side in ARMS:
        (RUNTIME / f"execution-{side}.json").unlink(missing_ok=True)
    report("preflight", selected_arms=[arm.side for arm in selected])
    if unavailable:
        raise RuntimeError("PNP master unavailable: " + ", ".join(unavailable))
    check(selected)
    children = []
    try:
        for arm in selected:
            process = subprocess.Popen([sys.executable, __file__, "--arm", arm.side])
            children.append((arm, process))
        report("vendor_starting_arms", selected_arms=[arm.side for arm in selected])
        while True:
            for arm, child in children:
                code = child.poll()
                if code is not None:
                    raise RuntimeError(f"{arm.side} teleop stopped (exit {code}); see per-arm status")
            time.sleep(.1)
    finally:
        for _, child in children:
            if child.poll() is None:
                child.terminate()
        deadline = time.monotonic() + 3.5
        for _, child in children:
            try:
                child.wait(timeout=max(0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass


def run_arm(side):
    execution_status.STATUS = RUNTIME / f"execution-{side}.json"
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    report("worker_starting")
    run_checked(lambda: run_vendor(ARMS[side]))


if __name__ == "__main__":
    if sys.argv[1:] == ["--detect"]:
        print("Arms=" + json.dumps(detect()))
    elif len(sys.argv) == 3 and sys.argv[1] == "--arm":
        run_arm(sys.argv[2])
    else:
        run_checked(launch)
