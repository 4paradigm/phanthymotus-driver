"""Receive and install the authenticated image support package."""
import base64, ipaddress, json, os
from pathlib import Path
import subprocess, sys, tempfile
data = json.load(sys.stdin)
runtime = Path.home() / ".local/lib/fr3-pnp"
unit = "pnp-fr3-right-teleop.service"
state = subprocess.check_output(["systemctl", "show", unit, "-p", "ActiveState", "--value"], text=True).strip()
if state not in ("inactive", "failed"):
    raise RuntimeError("Stop the PNP teleoperation service before initialization")
password = data.pop("password")
with tempfile.TemporaryDirectory(prefix="fr3-card-install-") as directory:
    stage = Path(directory)
    for name, encoded in data["files"].items():
        path = stage / name
        if path.is_absolute() and not str(path).startswith(str(stage) + "/"):
            raise ValueError("Invalid package path")
        if ".." in Path(name).parts or Path(name).is_absolute():
            raise ValueError("Invalid package path")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(base64.b64decode(encoded))
    sys.path.insert(0, str(stage))
    from detect_paths import discover_paths
    vendor, conda = discover_paths()
    env = dict(os.environ, PNP_VENDOR_ROOT=str(vendor), PNP_CONDA_BASE=str(conda),
               PNP_INSTALL_STDIN_PASSWORD="1")
    auth = subprocess.run(["sudo", "-S", "-p", "", "-v"], input=password + "\n",
                          text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
    if auth.returncode:
        raise RuntimeError("PNP sudo authentication failed")
    install = subprocess.run(["sh", str(stage / "install-pnp.sh")], env=env,
                             input=password + "\n", stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, timeout=90)
    password = ""
    if install.returncode:
        raise RuntimeError("PNP installation failed: " + install.stdout[-2000:])
    ssh = Path.home() / ".ssh"
    ssh.mkdir(mode=0o700, exist_ok=True)
    authorized = ssh / "authorized_keys"
    previous = authorized.read_text() if authorized.exists() else ""
    key = data["public_key"].strip()
    key_type, key_value = key.split()[:2]
    entry = f'restrict,command="{runtime}/pnp-card-ctl" {key}\n'
    lines = [line for line in previous.splitlines(keepends=True) if key_value not in line]
    pending = ssh / ".authorized_keys.fr3"
    pending.write_text("".join(lines).rstrip("\n") + "\n" + entry)
    pending.chmod(0o600)
    pending.replace(authorized)
    robot_ips = data["slave_ips"]
    if set(robot_ips) != {"left", "right"} or len(set(robot_ips.values())) != 2:
        raise ValueError("Two distinct slave IP addresses are required")
    for address in robot_ips.values():
        ipaddress.IPv4Address(address)
    pending_config = runtime / ".robot_ips.json"
    pending_config.write_text(json.dumps(robot_ips))
    pending_config.replace(runtime / "robot_ips.json")
    (runtime / "vendor-root.txt").write_text(str(vendor) + "\n")
    (runtime / "package.sha256").write_text(data["version"] + "\n")
print("PNP_INITIALIZED")
