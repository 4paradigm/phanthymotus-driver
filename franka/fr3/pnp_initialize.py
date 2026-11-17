"""Install the image's support sources over an authenticated SSH connection."""
import base64
import hashlib
import json
import os
import shlex
from pathlib import Path
import subprocess
import tempfile

FILES = ("detect_paths.py", "arm_config.py", "run_right.py", "run_vendor.py", "execution_status.py",
         "prepare_vendor_follow.py", "stop_right.sh", "vendor-follow-env.sh",
         "vendor-bin/sudo", "install-pnp.sh", "pnp-card-ctl",
         "pnp-fr3-right-teleop.service", "README.md")
REMOTE = Path(__file__).with_name("pnp_install_remote.py").read_text()


def package():
    source = Path("/deploy/pnp-remote")
    files = {name: base64.b64encode((source / name).read_bytes()).decode() for name in FILES}
    digest = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    return files, digest


def paths():
    state = Path(os.environ.get("FRANKA_PNP_STATE_DIR", "/var/lib/fr3-pnp"))
    return state / "id_ed25519", state / "known_hosts"


def authenticated_command(host, password, command, timeout=15, payload=None):
    key, known = paths()
    key.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if not key.exists():
        subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "", "-f", str(key)],
                       check=True, capture_output=True)
    base = ["ssh", "-i", str(key), "-o", "IdentitiesOnly=yes", "-o",
            f"UserKnownHostsFile={known}", "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ConnectTimeout=3", "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=3"]
    if not password:
        return subprocess.run(base + ["-o", "BatchMode=yes", host, command],
                              input=payload, capture_output=True, text=True, timeout=timeout)
    with tempfile.TemporaryDirectory(prefix="fr3-askpass-") as directory:
        helper = Path(directory) / "askpass"
        helper.write_text('#!/bin/sh\nprintf "%s\n" "$FR3_SSH_PASSWORD"\n')
        helper.chmod(0o700)
        env = dict(os.environ, SSH_ASKPASS=str(helper), SSH_ASKPASS_REQUIRE="force",
                   DISPLAY=":0", FR3_SSH_PASSWORD=password)
        return subprocess.run(base + ["-o", "PubkeyAuthentication=no", "-o",
                              "PreferredAuthentications=password,keyboard-interactive", "-o",
                              "NumberOfPasswordPrompts=1", host, command],
                              input=payload, capture_output=True, text=True, env=env, timeout=timeout)


def initialize(host, password, progress, slave_ips=None):
    slave_ips = slave_ips or {"right": "172.16.0.2", "left": "172.16.0.3"}
    files, version = package()
    version = hashlib.sha256((version + json.dumps(slave_ips, sort_keys=True)).encode()).hexdigest()
    progress("checking_remote_version")
    check = authenticated_command(host, "", "pnp-package-version")
    if check.returncode == 0 and check.stdout.strip() == version:
        return {"state": "ready", "updated": False, "package_version": version}
    if not password:
        raise RuntimeError("An SSH initialization password is required for installation or updates")
    key, _ = paths()
    public = key.with_suffix(".pub").read_text().strip()
    payload = json.dumps(dict(files=files, version=version, password=password,
                              public_key=public, slave_ips=slave_ips))
    progress("installing_remote_package")
    result = authenticated_command(host, password, "python3 -c " + shlex.quote(REMOTE),
                                   timeout=125, payload=payload)
    if result.returncode or "PNP_INITIALIZED" not in result.stdout:
        detail = (result.stderr or result.stdout)[-2000:].replace(password, "[redacted]")
        raise RuntimeError("PNP initialization failed: " + detail)
    progress("verifying_key_access")
    checked = authenticated_command(host, "", "pnp-package-version")
    if checked.returncode or checked.stdout.strip() != version:
        raise RuntimeError("PNP key access verification failed after installation")
    return {"state": "ready", "updated": True, "package_version": version}
