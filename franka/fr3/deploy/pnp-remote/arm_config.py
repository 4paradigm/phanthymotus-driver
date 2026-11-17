"""USB identities and ports from the installed PNP vendor configuration."""
from dataclasses import dataclass
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

    def master(self, serial_dir):
        return serial_dir / f"usb-FTDI_USB__-__Serial_Converter_{self.serial}-if00-port0"


CONFIG = Path(__file__).with_name("robot_ips.json")
ROBOT_IPS = json.loads(CONFIG.read_text()) if CONFIG.is_file() else {}
for address in ROBOT_IPS.values():
    ipaddress.IPv4Address(address)
ARMS = {
    "right": Arm("right", "FTBMDRZS", ROBOT_IPS.get("right", "172.16.0.2"), 50051, 50053, 6001),
    "left": Arm("left", "FTB57FAR", ROBOT_IPS.get("left", "172.16.0.3"), 50052, 50054, 6002),
}
SERIAL_DIR = Path("/dev/serial/by-id")


def detect(serial_dir=SERIAL_DIR):
    arms = {}
    for side, arm in ARMS.items():
        master = arm.master(serial_dir)
        present = master.is_char_device()
        arms[side] = {"serial": arm.serial, "robot_ip": arm.robot_ip,
                      "present": present,
                      "ready": present and os.access(master, os.R_OK | os.W_OK)}
    return arms
