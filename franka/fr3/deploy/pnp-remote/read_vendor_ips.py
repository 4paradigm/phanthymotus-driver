"""Read left/right IP assignments from the manufacturer's PNP configuration."""
import ipaddress
import json
from pathlib import Path
import re


def read_vendor_ips(home=None):
    home = Path.home() if home is None else Path(home)
    configured = home / ".local/lib/fr3-pnp/vendor-root.txt"
    roots = [Path(configured.read_text().strip())] if configured.is_file() else []
    roots += [home / "frankateleop", Path("/opt/frankateleop")]
    roots += [path.parents[2] for path in home.glob("*/teleop/experiments/run_env.py")]
    roots = list(dict.fromkeys(root.resolve() for root in roots
                               if (root / "right_franka/1_launch_robot.sh").is_file()))
    result = {"ips": {}, "errors": {}}
    if len(roots) != 1:
        result["errors"]["configuration"] = "Vendor directory is missing or ambiguous"
        return result
    conf = roots[0] / "polymetis/polymetis/python/polymetis/conf"
    for side in ("right", "left"):
        try:
            addresses = []
            for relative in (f"robot_client/{side}_franka_hardware.yaml",
                             f"gripper/{side}_franka_hand.yaml"):
                values = re.findall(r'^\s*robot_ip:\s*["\']?([0-9.]+)["\']?\s*(?:#.*)?$',
                                    (conf / relative).read_text(), re.MULTILINE)
                if len(values) != 1:
                    raise ValueError("Expected one robot_ip in " + relative)
                addresses.append(str(ipaddress.IPv4Address(values[0])))
            if addresses[0] != addresses[1]:
                raise ValueError("Arm and gripper configurations reference different IPs")
            result["ips"][side] = addresses[0]
        except (OSError, ValueError) as exc:
            result["errors"][side] = str(exc)
    if len(result["ips"]) == 2 and len(set(result["ips"].values())) != 2:
        result["ips"] = {}
        result["errors"]["configuration"] = "Left and right configurations use the same IP"
    return result


if __name__ == "__main__":
    print("PNP_SLAVE_IPS=" + json.dumps(read_vendor_ips()))
