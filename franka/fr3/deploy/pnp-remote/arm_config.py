"""USB identities and ports from the installed PNP vendor configuration."""
from dataclasses import dataclass
from read_vendor_masters import read_vendor_masters
import ipaddress
import json
import os
from pathlib import Path


@dataclass(frozen=True)
class Arm:
    side: str
    serial: str
    robot_ip: str
    arm_port: int
    hand_port: int
    node_port: int
    device: str

    def master(self, serial_dir):
        return serial_dir / Path(self.device).name


CONFIG = Path(__file__).with_name("robot_ips.json")
ROBOT_IPS = json.loads(CONFIG.read_text()) if CONFIG.is_file() else {}
for address in ROBOT_IPS.values():
    ipaddress.IPv4Address(address)
ROLE_MAP = read_vendor_masters()
ARMS = {}
for side, ports in (("right", (50051, 50053, 6001)), ("left", (50052, 50054, 6002))):
    master = ROLE_MAP["arms"].get(side)
    if master:
        fallback_ip = "172.16.0.2" if side == "right" else "172.16.0.3"
        ARMS[side] = Arm(side, master["serial"], ROBOT_IPS.get(side, fallback_ip), *ports,
                         device=master["device"])
SERIAL_DIR = Path("/dev/serial/by-id")


def detect(serial_dir=SERIAL_DIR):
    arms = {}
    for side in ("right", "left"):
        arm = ARMS.get(side)
        if arm is None:
            arms[side] = {"present": False, "ready": False,
                          "error": ROLE_MAP["errors"].get(side, ROLE_MAP["errors"].get("configuration"))}
            continue
        master = arm.master(serial_dir)
        present = master.is_char_device()
        arms[side] = {"serial": arm.serial, "robot_ip": arm.robot_ip,
                      "present": present,
                      "ready": present and os.access(master, os.R_OK | os.W_OK)}
    return arms
