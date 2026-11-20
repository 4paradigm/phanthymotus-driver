"""Read PNP arm addresses using a verified controller SSH connection."""

import ipaddress
import json
import os
import re
import socket
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import tempfile
import threading

import yaml

from hardware import canonical_ip
from franka.fr3.arm_ips import mapping


STATE = Path("/opt/phanthy-motus/franka-ip-sync")
ACCESS = Path("/opt/phanthy-motus/franka-controller-access")


def read_state(name):
    try:
        value = json.loads((STATE / name).read_text())
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


ROBOT_CONFIG = "/home/pnp/.frankadeskapi/robot.yaml"
HARDWARE_DIR = "/home/pnp/frankateleop/polymetis/polymetis/conf/robot_client"


def hardware_ip(value):
    if isinstance(value, dict):
        addresses = [canonical_ip(item) if key == "robot_ip" else hardware_ip(item)
                     for key, item in value.items()]
    elif isinstance(value, list):
        addresses = [hardware_ip(item) for item in value]
    else:
        return None
    addresses = set(filter(None, addresses))
    if len(addresses) > 1:
        raise ValueError("Ambiguous robot_ip in hardware YAML")
    return next(iter(addresses), None)


def network_json(*args):
    return json.loads(subprocess.check_output(["ip", "-j", "-4", *args], timeout=3))


class ControllerConfiguration:
    PREFIX = "controller_configuration"

    def __init__(self, arm_ips):
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.wake = threading.Event()
        self.thread = None
        self.generation = 0
        self.binding = None
        self.failure = "Discovering the PNP controller"
        self.arm_ips = mapping(arm_ips) if arm_ips else {}
        self.previous = read_state("binding.json").get("controller_ip", "")
        saved = read_state("credentials.json")
        self.username = saved.get("username", "pnp")
        self.password = saved.get("password", "ros")

    def get_tools(self):
        return [{
            "name": self.PREFIX, "title": "Controller Configuration",
            "type": "sensor", "multiInstance": False,
            "description": "Configure the SSH login used by the FR3 driver to discover the PNP controller.",
            "inputSchema": {"type": "object", "properties": {}},
            "configSchema": {"type": "object", "properties": {
                "username": {"type": "string", "title": "SSH username", "default": self.username, "scope": "tool"},
                "password": {"type": "string", "title": "SSH password", "format": "password",
                             "default": "ros", "scope": "tool", "x-sensitive": True},
            }},
            "topic_in": [], "topic_out": [],
        }]

    def start(self):
        with self.lock:
            if self.thread is None:
                self.thread = threading.Thread(target=self.discover, daemon=True, name="fr3-controller-discovery")
                self.thread.start()

    def stop(self):
        self.cancel.set()
        self.wake.set()

    def snapshot(self):
        with self.lock:
            if self.binding is None:
                raise RuntimeError(self.failure)
            address, arms = self.binding
            return address, dict(arms), self.username, self.password

    @staticmethod
    def persist(name, data):
        STATE.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=STATE, mode="w", delete=False) as output:
            temporary = Path(output.name)
            try:
                json.dump(data, output)
                output.close()
                temporary.replace(STATE / name)
            finally:
                temporary.unlink(missing_ok=True)

    def dispatch(self, action, args):
        if action in ("start", "stop", self.PREFIX):
            return {"ok": True, "state": "ready" if action != "stop" else "idle"}
        if action != "config":
            return None
        with self.lock:
            username = args.get("username", self.username)
            password = args.get("password", self.password)
            if not isinstance(username, str) or not re.fullmatch(r"[a-zA-Z_][a-zA-Z0-9_.-]{0,63}", username or "pnp"):
                return {"ok": False, "message": "Invalid SSH username"}
            if not isinstance(password, str) or any(char in password for char in "\r\n\x00"):
                return {"ok": False, "message": "Invalid SSH password"}
            username, password = username or "pnp", password or "ros"
            if (username, password) != (self.username, self.password):
                self.persist("credentials.json", dict(username=username, password=password))
                self.username, self.password = username, password
                self.binding = None
                self.failure = "Discovering the PNP controller with the updated SSH credentials"
                self.generation += 1
                self.wake.set()
            return {"ok": True, "username": username}

    def discover(self):
        while not self.cancel.is_set():
            with self.lock:
                self.wake.clear()
                generation = self.generation
                credentials = self.username, self.password
                previous = self.previous
            try:
                address, arms = self.find_binding(previous, credentials)
                with self.lock:
                    if generation != self.generation or self.cancel.is_set():
                        continue
                    self.persist("binding.json", dict(controller_ip=address, motion_arm_ips=arms))
                    self.binding = address, arms
                    self.previous, self.arm_ips = address, arms
                retry = None
            except (OSError, RuntimeError, ValueError) as error:
                with self.lock:
                    if generation != self.generation:
                        continue
                    self.failure = str(error)
                retry = 3
            self.wake.wait(retry)

    def find_binding(self, previous, credentials):
        try:
            addresses = self.candidates()
        except (OSError, subprocess.SubprocessError, ValueError, KeyError):
            addresses = []
        # Prefer the previous verified host when discovery finds it again.
        addresses = sorted(addresses, key=lambda address: address != previous)[:16]
        if previous and previous not in addresses:
            addresses.append(previous)
        for address in addresses:
            if self.cancel.is_set():
                break
            try:
                return address, self.read_controller(address, credentials)
            except (OSError, subprocess.SubprocessError, ValueError, TypeError, KeyError, RuntimeError, yaml.YAMLError):
                continue
        raise RuntimeError("No trusted PNP controller returned both arm addresses; check network, SSH credentials and controller files")

    def candidates(self):
        interface = os.environ.get("FRANKA_ROBOT_INTERFACE")
        devices = network_json("addr", "show")
        if not interface:
            devices = [d for d in devices if d["ifname"].startswith(("en", "eth")) and "UP" in d.get("flags", [])]
        else:
            devices = [d for d in devices if d["ifname"] == interface]
        hosts = {}
        for device in devices:
            for addr in device.get("addr_info", []):
                if addr.get("scope") != "global":
                    continue
                network = ipaddress.IPv4Network(f"{addr['local']}/{addr['prefixlen']}", strict=False)
                if network.num_addresses > 1024:
                    continue
                for host in network.hosts():
                    if str(host) != addr['local'] and str(host) not in self.arm_ips.values():
                        hosts[str(host)] = addr['local']
        def ssh_open(address):
            if self.cancel.is_set():
                return False
            try:
                with socket.create_connection((address, 22), timeout=.3,
                                              source_address=(hosts[address], 0)):
                    return True
            except OSError:
                return False
        with ThreadPoolExecutor(max_workers=64) as pool:
            return [address for address, found in zip(hosts, pool.map(ssh_open, hosts)) if found]

    def read_file(self, address, path, credentials):
        if self.cancel.is_set():
            raise RuntimeError("IP synchronization cancelled or timed out")
        username, password = credentials
        with tempfile.TemporaryFile() as secret:
            password = password.encode() + b"\n"
            secret.write(password)
            secret.seek(0)
            command = [
                "sshpass", "-d", str(secret.fileno()), "ssh", "-F", "/dev/null",
                "-o", "StrictHostKeyChecking=yes", "-o", "GlobalKnownHostsFile=/dev/null",
                "-o", f"UserKnownHostsFile={ACCESS / 'known_hosts'}",
                "-o", "HostKeyAlias=franka-pnp-controller",
                "-o", "ConnectTimeout=2", "-o", "NumberOfPasswordPrompts=1",
                "-o", "PreferredAuthentications=password",
                f"{username}@{address}", f"head -c 65537 -- {path}",
            ]
            raw = subprocess.check_output(
                command, pass_fds=(secret.fileno(),), stderr=subprocess.DEVNULL, timeout=5)
        if not raw or len(raw) > 65536:
            raise ValueError("Invalid PNP configuration size")
        return yaml.safe_load(raw)

    def read_controller(self, address, credentials):
        desk = {}
        try:
            robots = self.read_file(address, ROBOT_CONFIG, credentials)["robots"]
            for side, key in (("left", "left_fci"), ("right", "right_fci" if "right_fci" in robots else "fci")):
                if key in robots:
                    desk[side] = canonical_ip(robots[key]["host"])
        except subprocess.CalledProcessError as error:
            if error.returncode != 1:
                raise
        except (ValueError, TypeError, KeyError, yaml.YAMLError):
            pass
        hardware = {}
        for side, files in (
            ("left", ("left_franka_hardware.yaml",)),
            ("right", ("right_franka_hardware.yaml", "red_franka_hardware.yaml", "franka_hardware.yaml")),
        ):
            for filename in files:
                try:
                    value = hardware_ip(self.read_file(address, f"{HARDWARE_DIR}/{filename}", credentials))
                except (subprocess.SubprocessError, ValueError, TypeError, KeyError, yaml.YAMLError):
                    continue
                if value:
                    hardware[side] = value
                    break
        if any(desk[side] != hardware[side] for side in desk.keys() & hardware.keys()):
            raise ValueError("Desk and hardware files disagree on left/right IPs")
        return mapping({**hardware, **desk})
