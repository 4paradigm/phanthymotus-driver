"""Install the image's support sources over an authenticated SSH connection."""
import base64
import hashlib
import json
import os
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


def initialize(host, password, progress):
    key, known = paths()
    key.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if not key.exists():
        subprocess.run(["ssh-keygen", "-q", "-t", "ed25519", "-N", "",
                        "-f", str(key)], check=True, capture_output=True)
    files, version = package()
    base = ["ssh", "-i", str(key), "-o", "IdentitiesOnly=yes", "-o",
            f"UserKnownHostsFile={known}", "-o", "StrictHostKeyChecking=accept-new",
            "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=3",
            "-o", "ServerAliveCountMax=3"]
    progress("checking_remote_version")
    check = subprocess.run(base + ["-o", "BatchMode=yes", host, "pnp-package-version"],
                           capture_output=True, text=True, timeout=15)
    if check.returncode == 0 and check.stdout.strip() == version:
        progress("ready")
        return {"state": "ready", "updated": False, "package_version": version}
    if not password:
        raise RuntimeError("首次初始化或更新 PNP 需要填写 SSH 初始化密码")
    public = key.with_suffix(".pub").read_text().strip()
    payload = json.dumps(dict(files=files, version=version, password=password,
                              public_key=public))
    progress("installing_remote_package")
    with tempfile.TemporaryDirectory(prefix="fr3-askpass-") as directory:
        helper = Path(directory) / "askpass"
        helper.write_text('#!/bin/sh\nprintf "%s\n" "$FR3_SSH_PASSWORD"\n')
        helper.chmod(0o700)
        env = dict(os.environ, SSH_ASKPASS=str(helper), SSH_ASKPASS_REQUIRE="force",
                   DISPLAY=":0", FR3_SSH_PASSWORD=password)
        # The remote program and JSON travel through encrypted stdin.
        command = "python3 -c " + __import__("shlex").quote(REMOTE)
        result = subprocess.run(base + ["-o", "PubkeyAuthentication=no", "-o",
                                "PreferredAuthentications=password,keyboard-interactive",
                                "-o", "NumberOfPasswordPrompts=1", host, command],
                                input=payload, capture_output=True, text=True, env=env, timeout=125)
    if result.returncode or "PNP_INITIALIZED" not in result.stdout:
        detail = (result.stderr or result.stdout)[-2000:].replace(password, "[redacted]")
        raise RuntimeError("PNP 初始化失败：" + detail)
    progress("verifying_key_access")
    checked = subprocess.run(base + ["-o", "BatchMode=yes", host, "pnp-package-version"],
                             capture_output=True, text=True, timeout=15)
    if checked.returncode or checked.stdout.strip() != version:
        raise RuntimeError("PNP 安装后免密访问验证失败")
    return {"state": "ready", "updated": True, "package_version": version}
