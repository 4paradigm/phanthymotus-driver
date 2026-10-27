#!/usr/bin/env python3
"""Install/stop only Phanthy's isolated native DOA sensor worker on the robot.

Uses the already-installed vendor audio image and its config volume read-only.
Does not restart or exec commands in the vendor audio container. No device,
authorization, SSH or Docker socket mounts are passed to the worker.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

NAME = "embodied-u1pro-native-doa"
LABEL = "com.phanthy.u1pro-native-doa"
ROOT = Path("/opt/phanthy-motus/data/u1_pro/native_doa")
VENDOR_AUDIO_REPOSITORIES = {
    "acr.rd.ubtrobot.com/glcr/uworld/robo/runtime/csh2/module/audio",
    "glcr.rd.ubtrobot.com/uworld/robo/runtime/csh2/module/audio",
}


def docker(*args, check=True):
    return subprocess.run(["docker", *args], check=check, capture_output=True, text=True)


def inspect(name):
    result = docker("inspect", name, check=False)
    return json.loads(result.stdout)[0] if result.returncode == 0 else None


def stop():
    existing = inspect(NAME)
    if existing:
        if existing.get("Config", {}).get("Labels", {}).get(LABEL) != "1":
            raise RuntimeError(f"Refusing to replace unrelated container {NAME}")
        docker("rm", "-f", NAME)


def start(image, native_container):
    native = inspect(native_container)
    if not native or not native.get("State", {}).get("Running"):
        raise RuntimeError("Vendor audio runtime is not running")
    reference = native.get("Config", {}).get("Image", "").split("@", 1)[0]
    repository = reference.rsplit(":", 1)[0] if ":" in reference.rsplit("/", 1)[-1] else reference
    if repository not in VENDOR_AUDIO_REPOSITORIES:
        raise RuntimeError("Refusing a container that is not a supported U1 vendor audio image")
    config = next((m for m in native["Mounts"] if m["Destination"] == "/opt/walker/config_csh2"), None)
    if not config or config["Type"] not in ("volume", "bind"):
        raise RuntimeError("Cannot locate the real vendor audio configuration mount")
    source = config["Name"] if config["Type"] == "volume" else config["Source"]
    mount = f"{source}:/opt/walker/config_csh2:ro"
    version = docker("image", "inspect", image, "--format", "{{.Id}}").stdout.strip().split(":")[-1]
    if len(version) != 64 or any(c not in "0123456789abcdef" for c in version):
        raise RuntimeError("Invalid driver image ID")
    destination = ROOT / version
    # Copy versioned configuration atomically, never change a running worker's
    # mounted tree in place. Extraction container is never started.
    with tempfile.TemporaryDirectory(prefix="u1-doa-config-") as temporary:
        cid = docker("create", "--entrypoint", "/bin/true", image).stdout.strip()
        try:
            docker("cp", f"{cid}:/deploy/native_doa/.", temporary)
        finally:
            docker("rm", cid)
        if not (Path(temporary) / "controller.yaml").is_file():
            raise RuntimeError("Driver image lacks native DOA configuration")
        if not destination.exists():
            ROOT.mkdir(parents=True, exist_ok=True)
            staging = ROOT / (version + ".tmp")
            if staging.exists():
                shutil.rmtree(staging)
            shutil.copytree(temporary, staging)
            staging.rename(destination)
    stop()
    docker("run", "-d", "--name", NAME, "--label", LABEL + "=1",
           "--label", "com.phanthy.driver-image-id=" + version,
           "--restart", "unless-stopped", "--network", "host", "--read-only",
           "--cap-drop", "ALL", "--security-opt", "no-new-privileges",
           "--tmpfs", "/tmp:rw,size=64m", "--tmpfs", "/data:rw,size=16m",
           "--volume", str(destination) + ":/run/u1-doa:ro", "--volume", mount,
           "--log-driver", "local", "--log-opt", "max-size=10m", "--log-opt", "max-file=3",
           "--env", "ROSA_DOMAIN_ID=43", "--env", "ROSA_NETWORK_MODE=LOCALHOST",
           "--env", "ROSA_ALLOW_INTERFACE=127.0.0.1", "--env", "ROSA_LOG_LEVEL=ERROR",
           "--env", "ROSA_LOG_TRACE=", "--env", "ROSA_LOG_DIR=/tmp/rosa-log",
           "--env", "ROSA_LOG_MAX_FILES=3",
           "--entrypoint", "/bin/bash", native["Image"], "-c",
           "source /opt/walker/setup.bash; "
           "export AMENT_PREFIX_PATH=/run/u1-doa/prefix:$AMENT_PREFIX_PATH; "
           "exec rosa run node_manager node_manager -- -p phanthy_u1_doa --log_name phanthy_doa")
    return {"container": NAME, "domain": 43, "native_image": native["Config"]["Image"],
            "native_image_id": native["Image"], "driver_image": image, "config_directory": str(destination)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "status"))
    parser.add_argument("--driver-image")
    parser.add_argument("--native-container", default="walker-audio.audio-1")
    args = parser.parse_args()
    if args.action == "start":
        if not args.driver_image:
            parser.error("start requires --driver-image")
        print(json.dumps(start(args.driver_image, args.native_container)))
    elif args.action == "stop":
        stop()
        print(json.dumps({"state": "stopped", "container": NAME}))
    else:
        item = inspect(NAME)
        print(json.dumps({"container": NAME, "state": item.get("State", {}).get("Status") if item else "absent",
                          "domain": 43, "restarts": item.get("RestartCount") if item else 0}))


if __name__ == "__main__":
    main()
