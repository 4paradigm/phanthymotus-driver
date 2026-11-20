"""Detect and supervise the connected PNP masters."""
import errno
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

def check(selected, *, include_node=True):
    if Path("/sys/kernel/realtime").read_text().strip() != "1":
        raise RuntimeError("PNP workstation must be running its realtime kernel")
    if resource.getrlimit(resource.RLIMIT_RTPRIO)[0] < 80:
        raise RuntimeError("PNP service has insufficient realtime scheduling permissions")
    if resource.getrlimit(resource.RLIMIT_MEMLOCK)[0] < 2 * 1024**3:
        raise RuntimeError("Service requires at least 2 GiB of locked memory for Polymetis")
    for arm in selected:
        endpoints = [("arm", arm.arm_port), ("gripper", arm.hand_port)]
        if include_node:
            endpoints.append(("vendor node", arm.node_port))
        for role, port in endpoints:
            check_bind(arm.side, role, port)


def check_bind(side, role, port):
    with socket.socket() as sock:
        # Match the backend's restart semantics without sharing an active listener.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", port))
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
                raise
            raise RuntimeError(f"PNP {side} {role} port 127.0.0.1:{port} is still in use") from exc


def launch():
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    os.environ["PATH"] = os.pathsep.join(
        (str(ENV / "bin"), str(POLY / "build"), os.environ["PATH"]))
    os.environ["CONDA_PREFIX"] = str(ENV)
    os.environ["LD_LIBRARY_PATH"] = os.pathsep.join(
        filter(None, (str(ENV / "lib"), os.environ.get("LD_LIBRARY_PATH"))))
    playback_request = RUNTIME / "requested-playback"
    if playback_request.is_file():
        try:
            names = playback_request.read_text().splitlines()
        finally:
            playback_request.unlink(missing_ok=True)
        launch_playback(names)
        return
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
    sides = ("right", "left") if side == "both" else (side,)
    selected = [ARMS[name] for name in sides if name in ARMS]
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
        stop_workers(children)


def stop_workers(children):
    for _, child in children:
        if child.poll() is None:
            child.terminate()
    deadline = time.monotonic() + 3.5
    for _, child in children:
        try:
            child.wait(timeout=max(0, deadline - time.monotonic()))
        except subprocess.TimeoutExpired:
            pass


def playback_group_stage(stages, released):
    if not released:
        return ("playback_approaching" if any(stage in ("playback_approaching", "playback_aligned")
                                              for stage in stages) else "playback_starting")
    unfinished = [stage for stage in stages if stage != "playback_completed"]
    if unfinished and all(stage == "playback_paused" for stage in unfinished):
        return "playback_paused"
    return "playback_running"


def launch_playback(names):
    from playback import APPROACH, START, load_trajectory
    from vendor_recordings import NAME

    tracks = {}
    for name in names:
        match = NAME.fullmatch(name)
        if not match or match[1] not in ARMS or match[1] in tracks:
            raise ValueError("Select one valid recording per configured arm")
        tracks[match[1]] = name
    if not tracks:
        raise ValueError("Select at least one playback recording")
    for name in tracks.values():
        load_trajectory(name)
    check([ARMS[side] for side in tracks], include_node=False)
    status_files = {side: RUNTIME / f"execution-{side}.json" for side in tracks}
    for path in status_files.values():
        path.unlink(missing_ok=True)

    selected = list(tracks)
    children = []
    released = False
    pending_start = START.with_suffix(".start-tmp")
    approaching = False

    def release(gate):
        pending_start.write_text(str(time.monotonic() + .2))
        pending_start.replace(gate)
    report("playback_starting", selected_arms=selected)
    try:
        for side, name in tracks.items():
            children.append((side, subprocess.Popen([sys.executable, __file__, "--playback-arm", name])))
        while True:
            states = {side: execution_status.snapshot(path) for side, path in status_files.items()}
            codes = {side: child.poll() for side, child in children}
            stages = [state.get("stage") for state in states.values()]
            for side, state in states.items():
                code = codes[side]
                if state.get("stage") == "failed" or (code is not None and
                        (code != 0 or state.get("stage") != "playback_completed")):
                    raise RuntimeError(f"{side} playback failed: {state.get('error', f'worker exit {code}')}")

            durations = [state.get("duration_s", 0.) for state in states.values()]
            if all(code == 0 for code in codes.values()):
                report("playback_completed", selected_arms=selected, duration_s=max(durations))
                return
            if not approaching and all(stage == "playback_entry_ready" for stage in stages):
                release(APPROACH)
                approaching = True
            if not released and all(stage == "playback_aligned" for stage in stages):
                release(START)
                released = True
            positions = [state.get("position_s", state.get("duration_s", 0.)
                         if state.get("stage") == "playback_completed" else 0.)
                         for state in states.values()]
            report(playback_group_stage(stages, released), selected_arms=selected,
                   position_s=max(positions), duration_s=max(durations))
            time.sleep(.1)
    finally:
        stop_workers(children)
        pending_start.unlink(missing_ok=True)
        START.unlink(missing_ok=True)
        APPROACH.unlink(missing_ok=True)


def run_playback_arm(name):
    from vendor_recordings import NAME
    side = NAME.fullmatch(name)[1]
    execution_status.STATUS = RUNTIME / f"execution-{side}.json"
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    def execute():
        from playback import load_trajectory, run
        _, stamps, points = load_trajectory(name)
        run(ARMS[side], stamps, points)

    run_checked(execute)


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
    elif len(sys.argv) == 3 and sys.argv[1] == "--playback-arm":
        run_playback_arm(sys.argv[2])
    else:
        run_checked(launch)
