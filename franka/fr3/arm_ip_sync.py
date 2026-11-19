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
import time
from uuid import uuid4

import yaml

from common.vendor_runtime import action_schema
from hardware import canonical_ip
from franka.fr3.arm_ips import mapping


STATE = Path("/opt/phanthy-motus/franka-ip-sync")
ACCESS = Path("/opt/phanthy-motus/franka-controller-access")


def load_binding(state=STATE):
    try:
        return json.loads((state / "binding.json").read_text())
    except FileNotFoundError:
        return {}


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


class ArmIPSyncPlugin:
    PREFIX = "arm_ip_sync"

    def __init__(self, arm_ips, namespace, ros2, callback=None):
        self.lock = threading.Lock()
        self.topic = f"/{namespace.strip('/')}/arm_ip_sync"
        self.ros2 = ros2
        self.publisher = None
        self.frames_published = 0
        self.last_published_at = None
        saved = load_binding()
        self.arm_ips = mapping(saved.get("motion_arm_ips", arm_ips))
        self.username = "pnp"
        self.password = "ros"
        self.controller_ip = saved.get("fallback_controller_ip", "")
        self.detected_controller_ip = saved.get("controller_ip", "")
        self.last_result = None
        self.cancel = threading.Event()
        self.deadline = float("inf")
        if callback is None:
            from device import JointControlPlugin
            callback = lambda aid, status, result: JointControlPlugin._post_completion(self, aid, status, result)
        self.callback = callback

    def get_tools(self):
        return [{
            "name": self.PREFIX, "type": "sensor", "multiInstance": False,
            "description": "Publish left/right FR3 controller IPs for connected action cards. Optional sync reads the trusted PNP host in the background; no robot motion.",
            "inputSchema": {**action_schema(
                {"sync": ([], "Discover the PNP controller on the robot network, then try the optional fallback IP. Update the saved arm IPs."),
                 "info": ([], "Show active and saved bindings.")},
                {}), "x-completion": {"actions": ["sync"], "timeout": 90}},
            "configSchema": {"type": "object", "properties": {
                "left_ip": {"type": "string", "title": "Left arm IP", "description": "Controller IP for the physical left arm. Applied immediately when idle; changing it invalidates the saved observation.", "default": self.arm_ips["left"], "scope": "instance"},
                "right_ip": {"type": "string", "title": "Right arm IP", "description": "Controller IP for the physical right arm. Applied immediately when idle; changing it invalidates the saved observation.", "default": self.arm_ips["right"], "scope": "instance"},
                "controller_ip": {"type": "string", "title": "PNP controller IP", "default": self.controller_ip, "scope": "instance",
                                  "description": "Optional fallback PNP host IPv4 address. Network discovery always runs first; this address is tried only if discovery fails.",},
                "username": {"type": "string", "title": "SSH username", "default": "pnp", "scope": "instance"},
                "password": {"type": "string", "format": "password", "title": "SSH password", "default": "ros",
                             "scope": "instance", "x-sensitive": True,
                             "description": "Default: ros. Change this value if the PNP controller uses a different SSH password. Blank uses the default."},
            }},
            "topic_in": [], "topic_out": [{"topic": self.topic, "format": "data/json"}],
        }]

    def start(self):
        pass

    def stop(self):
        self.cancel.set()
        if self.publisher:
            self.publisher.close()
            self.publisher = None

    def publish(self):
        if self.publisher:
            self.publisher.publish(dict(self.arm_ips, controller_ip=self.detected_controller_ip))
            self.frames_published += 1
            self.last_published_at = time.monotonic()

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
            if self.cancel.is_set() or time.monotonic() > self.deadline:
                return False
            try:
                with socket.create_connection((address, 22), timeout=.3,
                                              source_address=(hosts[address], 0)):
                    return True
            except OSError:
                return False
        with ThreadPoolExecutor(max_workers=32) as pool:
            return [address for address, found in zip(hosts, pool.map(ssh_open, hosts)) if found]

    def read_file(self, address, path):
        if self.cancel.is_set() or time.monotonic() > self.deadline:
            raise RuntimeError("IP synchronization cancelled or timed out")
        with tempfile.TemporaryFile() as secret:
            password = self.password.encode() + b"\n"
            secret.write(password)
            secret.seek(0)
            command = [
                "sshpass", "-d", str(secret.fileno()), "ssh", "-F", "/dev/null",
                "-o", "StrictHostKeyChecking=yes", "-o", "GlobalKnownHostsFile=/dev/null",
                "-o", f"UserKnownHostsFile={ACCESS / 'known_hosts'}",
                "-o", "HostKeyAlias=franka-pnp-controller",
                "-o", "ConnectTimeout=2", "-o", "NumberOfPasswordPrompts=1",
                "-o", "PreferredAuthentications=password",
                f"{self.username}@{address}", f"head -c 65537 -- {path}",
            ]
            raw = subprocess.check_output(
                command, pass_fds=(secret.fileno(),), stderr=subprocess.DEVNULL, timeout=5)
        if not raw or len(raw) > 65536:
            raise ValueError("Invalid PNP configuration size")
        return yaml.safe_load(raw)

    def read_controller(self, address):
        desk = {}
        try:
            robots = self.read_file(address, ROBOT_CONFIG)["robots"]
            for side, key in (("left", "left_fci"), ("right", "right_fci" if "right_fci" in robots else "fci")):
                if key in robots:
                    desk[side] = canonical_ip(robots[key]["host"])
        except (subprocess.SubprocessError, ValueError, TypeError, KeyError, yaml.YAMLError):
            pass
        hardware = {}
        for side, files in (
            ("left", ("left_franka_hardware.yaml",)),
            ("right", ("right_franka_hardware.yaml", "red_franka_hardware.yaml", "franka_hardware.yaml")),
        ):
            for filename in files:
                try:
                    value = hardware_ip(self.read_file(address, f"{HARDWARE_DIR}/{filename}"))
                except (subprocess.SubprocessError, ValueError, TypeError, KeyError, yaml.YAMLError):
                    continue
                if value:
                    hardware[side] = value
                    break
        if any(desk[side] != hardware[side] for side in desk.keys() & hardware.keys()):
            raise ValueError("Desk and hardware files disagree on left/right IPs")
        return mapping({**hardware, **desk})

    def _persist(self, binding):
        STATE.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=STATE, mode="w", delete=False) as output:
            temporary = Path(output.name)
            json.dump(binding, output)
        try:
            temporary.replace(STATE / "binding.json")
        finally:
            temporary.unlink(missing_ok=True)

    def save(self, binding):
        arms = mapping(binding["motion_arm_ips"])
        binding = {**binding, "motion_arm_ips": arms}
        self._persist(binding)
        self.arm_ips = arms
        self.detected_controller_ip = binding.get("controller_ip", "")
        self.publish()
        return dict(state="completed", restart_required=False,
                    config=self.configuration(), **binding)

    def configuration(self):
        return dict(left_ip=self.arm_ips["left"], right_ip=self.arm_ips["right"],
                    controller_ip=self.controller_ip, username=self.username)

    def dispatch(self, action, args):
        if action == "start" and self.ros2 and self.publisher is None:
            from device import RosPublisher
            self.publisher = RosPublisher(self.ros2, self.topic, 1, self.publish)
        if action == "stop":
            self.stop()
        if action in ("start", "stop", "info", self.PREFIX):
            age = None if self.last_published_at is None else time.monotonic() - self.last_published_at
            return dict(state="running" if self.publisher is not None else "idle",
                        frames_published=self.frames_published,
                        fresh=self.publisher is not None and age is not None and age < 3,
                        topic_out=self.get_tools()[0]["topic_out"],
                        controller_ip=self.controller_ip, config=self.configuration(),
                        detected_controller_ip=self.detected_controller_ip,
                        restart_required=False, last_result=self.last_result,
                        motion_arm_ips=dict(self.arm_ips))
        if action not in ("sync", "config"):
            return None
        if not self.lock.acquire(blocking=False):
            return dict(state="error", message="An IP sync is already running")
        if action == "sync":
            self.cancel.clear()
            self.deadline = time.monotonic() + 60
            action_id = "arm_ip_sync_" + uuid4().hex
            accepted = dict(state="running", action_id=action_id)
            self.last_result = accepted
            threading.Thread(target=self.sync, args=(action_id,), daemon=True).start()
            return dict(accepted)
        try:
            if action == "config":
                controller = args.get("controller_ip", self.controller_ip)
                controller = str(ipaddress.IPv4Address(controller)) if controller else ""
                username = args.get("username", self.username) or "pnp"
                password = args.get("password", self.password)
                if not isinstance(username, str) or not re.fullmatch(r"[a-zA-Z_][a-zA-Z0-9_.-]{0,63}", username):
                    raise ValueError("Invalid SSH username")
                if not isinstance(password, str) or any(char in password for char in "\r\n\x00"):
                    raise ValueError("Invalid SSH password")
                password = password or "ros"
                arms = mapping({side: args.get(side + "_ip", self.arm_ips[side])
                                for side in ("left", "right")})
                binding = dict(fallback_controller_ip=controller, motion_arm_ips=arms)
                if self.detected_controller_ip and arms == self.arm_ips:
                    binding["controller_ip"] = self.detected_controller_ip
                result = self.save(binding)
                self.controller_ip, self.username, self.password = controller, username, password
                return dict(result, ok=True, config=self.configuration())
        except (OSError, subprocess.SubprocessError, ValueError, RuntimeError, KeyError, IndexError) as exc:
            return dict(state="error", message=str(exc))
        finally:
            self.lock.release()

    def sync(self, action_id):
        status, result = "error", {"message": "No trusted PNP controller returned a valid arm mapping"}
        try:
            overall_deadline = self.deadline
            try:
                addresses = self.candidates()[:16]
            except (OSError, subprocess.SubprocessError, ValueError, KeyError, IndexError):
                addresses = []
            # Reserve time for the manually configured fallback after discovery.
            attempts = [(address, overall_deadline - 15) for address in addresses]
            if self.controller_ip:
                attempts.append((self.controller_ip, overall_deadline))
            for address, phase_deadline in attempts:
                if address in self.arm_ips.values():
                    continue
                if self.cancel.is_set():
                    raise RuntimeError("IP synchronization cancelled")
                if time.monotonic() >= phase_deadline:
                    continue
                self.deadline = min(phase_deadline, time.monotonic() + 10)
                try:
                    arms = self.read_controller(address)
                except (subprocess.SubprocessError, ValueError, TypeError, KeyError, RuntimeError, yaml.YAMLError):
                    continue
                if self.cancel.is_set() or time.monotonic() > self.deadline:
                    raise RuntimeError("IP synchronization cancelled or timed out")
                result = self.save(dict(controller_ip=address, fallback_controller_ip=self.controller_ip, motion_arm_ips=arms))
                status = "completed"
                break
        except Exception as exc:
            status = "cancelled" if self.cancel.is_set() else "error"
            result = {"message": str(exc)}
        finally:
            self.last_result = dict(state=status, action_id=action_id, result=result)
            try:
                self.callback(action_id, status, result)
                self.last_result["callback"] = "accepted"
            except Exception as exc:
                self.last_result.update(callback="failed", callback_error=str(exc))
            self.lock.release()
