"""Enumerate master USB devices and resolve roles from vendor launchers."""
import ast
import json
import os
from pathlib import Path
import re
import shlex


def read_vendor_masters(home=None, serial_dir=None):
    home = Path.home() if home is None else Path(home)
    serial_dir = Path("/dev/serial/by-id") if serial_dir is None else Path(serial_dir)
    available = sorted(path.name for path in serial_dir.glob("*") if path.is_char_device())
    result = {"vendor_ready": False, "arms": {}, "errors": {}, "usb_devices": available}
    saved = home / ".local/lib/fr3-pnp/vendor-root.txt"
    roots = [Path(saved.read_text().strip())] if saved.is_file() else []
    roots += [home / "frankateleop", Path("/opt/frankateleop")]
    roots += [path.parents[2] for path in home.glob("*/teleop/experiments/run_env.py")]
    roots = list(dict.fromkeys(path.resolve() for path in roots
                               if (path / "teleop/experiments/run_env.py").is_file()))
    if len(roots) != 1:
        result["errors"]["configuration"] = "Vendor directory is missing or ambiguous"
        return result
    root = roots[0]
    try:
        tree = ast.parse((root / "teleop/teleop/agents/teleop_agent.py").read_text())
        dictionaries = []
        for node in tree.body:
            if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                if node.target.id == "PORT_CONFIG_MAP":
                    dictionaries.append(node.value)
            elif isinstance(node, ast.Assign):
                if any(isinstance(target, ast.Name) and target.id == "PORT_CONFIG_MAP"
                       for target in node.targets):
                    dictionaries.append(node.value)
        if len(dictionaries) != 1 or not isinstance(dictionaries[0], ast.Dict):
            raise ValueError("Could not locate the vendor port calibration map")
        calibrated = {key.value for key in dictionaries[0].keys
                      if isinstance(key, ast.Constant) and isinstance(key.value, str)}
        result["vendor_ready"] = True
    except (OSError, SyntaxError, ValueError) as exc:
        result["errors"]["configuration"] = str(exc)
        return result
    for side in ("right", "left"):
        try:
            script = (root / f"{side}_franka/4_run_env.sh").read_text()
            ports = []
            for line in script.replace(chr(92) + "\n", " ").splitlines():
                words = shlex.split(line, comments=True)
                for index, word in enumerate(words):
                    if word.startswith(("--teleop_port=", "--teleop-port=")):
                        ports.append(word.split("=", 1)[1])
                    elif word in ("--teleop_port", "--teleop-port") and index + 1 < len(words):
                        ports.append(words[index + 1])
            ports = list(dict.fromkeys(ports))
            if len(ports) != 1:
                raise ValueError("Expected exactly one master USB port in the side launcher")
            port = Path(ports[0])
            if port.parent != Path("/dev/serial/by-id") or not re.fullmatch(r"[A-Za-z0-9_.:-]+", port.name):
                raise ValueError("The vendor master port must be a stable /dev/serial/by-id path")
            if str(port) not in calibrated:
                raise ValueError("Master port has no vendor calibration profile")
            device = serial_dir / port.name
            identity = re.search(r"_Converter_(.+?)-if[0-9]+", port.name)
            result["arms"][side] = {
                "device": str(port), "serial": identity.group(1) if identity else port.name,
                "present": device.is_char_device(),
                "ready": device.is_char_device() and os.access(device, os.R_OK | os.W_OK),
                "source": "vendor_side_launcher",
            }
        except (OSError, ValueError) as exc:
            result["errors"][side] = str(exc)
    if len(result["arms"]) == 2 and result["arms"]["left"]["device"] == result["arms"]["right"]["device"]:
        result["arms"] = {}
        result["errors"]["configuration"] = "Left and right launchers reference the same master"
    matched = {Path(arm["device"]).name for arm in result["arms"].values()}
    result["unmatched_usb_devices"] = [name for name in available if name not in matched]
    return result


if __name__ == "__main__":
    print("PNP_PROBE=" + json.dumps(read_vendor_masters()))
