#!/usr/bin/env python3
"""Build and manage a Q1 driver beside an existing Motus Core."""
import argparse
import json
import os
import platform
import plistlib
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

DEPLOY = Path(__file__).resolve().parent
DRIVER = DEPLOY.parent
DDS_LINK = DEPLOY / "dds_link.py"
MOTUS_DDS = "/opt/phanthy-motus/dds-local.xml"
TARGET = "linux/arm64"
ROS_BASE = "docker.io/library/ros:humble-ros-base-jammy"
RELAY = "local.motus.primebot-q1-dds"


def command(*args, env=None, capture=False, check=True, quiet=False, input_text=None):
    return subprocess.run(
        args, env=env, text=True, check=check, capture_output=capture,
        input=input_text,
        stdout=subprocess.DEVNULL if quiet else None,
        stderr=subprocess.DEVNULL if quiet else None,
    )


def build(args):
    common = DRIVER / "common"
    if not common.is_dir():
        common = DRIVER.parents[1] / "common"
    with tempfile.TemporaryDirectory(prefix="primebot-q1-build-") as temp:
        context = Path(temp)
        (context / "common").mkdir()
        for name in ("__init__.py", "vendor_runtime.py", "logsafe.py"):
            shutil.copyfile(common / name, context / "common" / name)
        for name in ("Dockerfile", "main.py", "robot.py", "battery.py",
                     "preset_postures.py", "preset_robot.py", "config.yaml"):
            shutil.copyfile(DRIVER / name, context / name)
        (context / "deploy").mkdir()
        shutil.copytree(DRIVER / "motion_msgs", context / "motion_msgs")
        shutil.copyfile(DEPLOY / "service.yml", context / "deploy/service.yml")
        command("docker", "build", "--platform", TARGET, "--build-arg",
                f"ROS_BASE_IMAGE={args.base_image}", "-t", args.image, temp)


def relay_target():
    return f"gui/{os.getuid()}/{RELAY}"


def stop_relay():
    if command("launchctl", "print", relay_target(), check=False, quiet=True).returncode == 0:
        command("launchctl", "bootout", relay_target())


def start_relay(state, args):
    manifest = state / f"{RELAY}.plist"
    definition = plistlib.dumps({
        "Label": RELAY,
        "ProgramArguments": [sys.executable, str(DDS_LINK), "discovery",
                             "--mac", args.mac_ip, "--robot", args.robot_ip],
        "KeepAlive": True,
        "ThrottleInterval": 5,
        "StandardOutPath": str(state / "dds.log"),
        "StandardErrorPath": str(state / "dds.log"),
    })
    if manifest.exists() and manifest.read_bytes() == definition:
        if command("launchctl", "print", relay_target(), check=False, quiet=True).returncode == 0:
            return
    stop_relay()
    manifest.write_bytes(definition)
    command("launchctl", "bootstrap", f"gui/{os.getuid()}", str(manifest))


def prepare_mac(args, state, env):
    if f"inet {args.mac_ip} " not in command("ifconfig", capture=True).stdout:
        raise ValueError(f"Assign {args.mac_ip} to the wired Mac interface before starting.")
    core = json.loads(command("docker", "inspect", args.core_container, capture=True).stdout)[0]
    if not core["State"]["Running"]:
        raise ValueError(f"Start Motus Core container {args.core_container} first.")
    env["Q1_CORE_CONTAINER_ID"] = core["Id"]
    if args.core_dds is None:
        # The profile mounted into Core also configures the driver's Core-side participant.
        env["Q1_CORE_DDS"] = next(
            (mount["Source"] for mount in core["Mounts"] if mount["Destination"] == MOTUS_DDS),
            env["Q1_CORE_DDS"],
        )
    if args.docker_network is None:
        networks = core["NetworkSettings"]["Networks"]
        if len(networks) != 1:
            raise ValueError("Specify --docker-network when Core does not have exactly one network.")
        env["Q1_DOCKER_NETWORK"] = next(iter(networks))
    profile = command("docker", "run", "--rm", "--network", "none", "--platform", TARGET,
                      "-v", f"{DDS_LINK}:/link.py:ro", args.image, "/usr/bin/python3",
                      "/link.py", "profile", "--mac", args.mac_ip, "--robot", args.robot_ip,
                      capture=True).stdout
    (state / "fastdds.xml").write_text(profile)


def status(compose, env, mac):
    result = command(*compose, "exec", "-T", "driver", "/usr/bin/python3",
                     "/dev/stdin", env=env, capture=True, check=False,
                     input_text=(DEPLOY / "probe.py").read_text())
    if result.returncode:
        print(result.stderr.strip() or "Q1 driver is stopped.", file=sys.stderr)
        return False
    payload = json.loads(result.stdout)
    battery = payload["battery"]
    print(f"Core registration: {'registered' if payload['registered'] else 'waiting'}")
    print(f"Q1 battery: {json.dumps(battery, ensure_ascii=False)}")
    for error in payload.get("errors", []):
        print(error, file=sys.stderr)
    if mac and battery["state"] != "live":
        print(f"Mac DDS relay log: {env['Q1_STATE_DIR']}/dds.log")
    return payload["registered"] and battery["state"] == "live"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("build", "start", "stop", "status"))
    parser.add_argument("--image", default="primebot-q1:local")
    parser.add_argument("--base-image", default=ROS_BASE)
    parser.add_argument("--core-url", help="Core URL as seen by the driver")
    parser.add_argument("--core-dds", type=Path, help="Host path to Core's dds-local.xml")
    parser.add_argument("--core-container", default="fourparadigm-intern-core-1",
                        help="Existing Core container on Mac")
    parser.add_argument("--docker-network", help="Core bridge network on Mac, inferred when starting")
    parser.add_argument("--mac-ip", default="10.1.1.99", help="Mac's IP on the Q1 wired link")
    parser.add_argument("--robot-ip", default="10.1.1.100", help="Q1's IP on the Mac wired link")
    parser.add_argument("--state-dir", type=Path, default=DRIVER / ".runtime",
                        help="Directory for the Mac DDS profile and relay logs")
    args = parser.parse_args()
    mac = sys.platform == "darwin"
    if platform.machine().lower() not in ("arm64", "aarch64") or not (mac or sys.platform.startswith("linux")):
        raise ValueError("Expected Apple Silicon macOS or ARM64 Linux.")
    if args.action == "build":
        build(args)
        return 0

    state = args.state_dir.resolve()
    core_dds = args.core_dds or Path(MOTUS_DDS)
    env = dict(os.environ, Q1_IMAGE=args.image,
               Q1_CORE_DDS=str(core_dds.resolve()), Q1_STATE_DIR=str(state),
               Q1_CORE_URL=args.core_url or ("http://localhost:15678" if mac else "https://localhost:15678"),
               Q1_CORE_CONTAINER=args.core_container, Q1_CORE_CONTAINER_ID=args.core_container,
               Q1_DDS_LINK=str(DDS_LINK),
               Q1_DOCKER_NETWORK=args.docker_network or "fourparadigm-intern_default")
    compose = ["docker", "compose", "-p", "primebot-q1", "-f", str(DEPLOY / "compose.yaml")]
    if mac:
        compose += ["-f", str(DEPLOY / "compose.mac.yaml")]

    if args.action == "stop":
        if mac:
            stop_relay()
        command(*compose, "down", env=env)
        return 0
    if args.action == "status":
        return 0 if status(compose, env, mac) else 1
    if mac:
        state.mkdir(parents=True, exist_ok=True)
        prepare_mac(args, state, env)
    if not Path(env["Q1_CORE_DDS"]).is_file():
        raise ValueError(f"Core DDS profile missing: {env['Q1_CORE_DDS']}. Supply --core-dds.")
    command(*compose, "up", "-d", env=env)
    if mac:
        start_relay(state, args)
    print("Q1 services started. Run status to check Core registration and a live battery sample.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(getattr(exc, "stderr", None) or exc, file=sys.stderr)
        sys.exit(1)
