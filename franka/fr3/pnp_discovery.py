"""Discover a wired PNP host and verify its two USB master identities."""
from concurrent.futures import ThreadPoolExecutor
import ipaddress
import json
from pathlib import Path
import select
import socket
import struct
import subprocess
import time



def wired_interfaces(networks=None):
    records = json.loads(subprocess.check_output(["ip", "-j", "-4", "addr", "show"], text=True))
    result = []
    for record in records:
        name = record["ifname"]
        sys = Path("/sys/class/net") / name
        if (sys / "wireless").exists() or not (sys / "device").exists():
            continue
        if not {"UP", "LOWER_UP"}.issubset(record.get("flags", [])):
            continue
        mac = (sys / "address").read_text().strip()
        addresses = []
        for address in record.get("addr_info", []):
            if address.get("scope") == "global" and (
                    networks is None or any(ipaddress.ip_address(address["local"]) in network
                                            for network in networks)):
                addresses.append(address["local"])
        if not addresses and networks:
            result.append((name, None, mac))
        result.extend((name, address, mac) for address in addresses)
    if not result:
        raise RuntimeError("No active wired IPv4 interface found on Orin")
    return result


def arp_hosts(interface, local, mac, networks):
    hardware = bytes.fromhex(mac.replace(":", ""))
    ethernet = b"\xff" * 6 + hardware + b"\x08\x06"
    peers = set()
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0806)) as link:
        link.bind((interface, 0))
        link.setblocking(False)
        targets = {str(ip) for network in networks for ip in network.hosts() if str(ip) != local}
        if not targets and all(network.prefixlen == 32 for network in networks):
            targets = {str(network.network_address) for network in networks}
        for target in targets:
            request = struct.pack("!HHBBH6s4s6s4s", 1, 0x0800, 6, 4, 1, hardware,
                                  socket.inet_aton(local), bytes(6), socket.inet_aton(target))
            link.send(ethernet + request)
            time.sleep(.001)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            if not select.select([link], [], [], .1)[0]:
                continue
            frame = link.recv(2048)
            if len(frame) >= 42 and frame[12:14] == b"\x08\x06" and frame[20:22] == b"\x00\x02":
                sender = socket.inet_ntoa(frame[28:32])
                if sender in targets:
                    peers.add(sender)
    return peers


def discover(slave_ips, user, password, progress):
    from franka.fr3.pnp_initialize import authenticated_command, paths
    networks = {ipaddress.ip_network(ip + "/24", strict=False) for ip in slave_ips.values()}
    links = wired_interfaces(networks)
    configured = [link for link in links if link[1] is not None]
    if not configured:
        progress("configuring_wired_ipv4")
        if len(links) != 1 or len(networks) != 1:
            raise RuntimeError("Automatic IPv4 setup needs one active wired interface and one robot subnet")
        interface, _, mac = links[0]
        network = next(iter(networks))
        for suffix in range(185, 200):
            address = str(network.network_address + suffix)
            if address in slave_ips.values():
                continue
            if arp_hosts(interface, "0.0.0.0", mac, [ipaddress.ip_network(address + "/32")]):
                continue
            subprocess.run(["ip", "address", "add", address + "/24", "dev", interface],
                           check=True, capture_output=True)
            configured = [(interface, address, mac)]
            break
        if not configured:
            raise RuntimeError("No free IPv4 address found in the robot subnet")
    links = configured
    excluded = set(slave_ips.values()) | {local for _, local, _ in links}
    candidates = {}
    progress("discovering_wired_hosts")
    with ThreadPoolExecutor(max_workers=min(8, len(links))) as pool:
        scans = [pool.submit(arp_hosts, interface, local, mac, networks)
                 for interface, local, mac in links]
        for (interface, local, _), scan in zip(links, scans):
            for peer in scan.result() - excluded:
                candidates.setdefault(peer, (interface, local))
    if len(candidates) > 16:
        raise RuntimeError("Too many wired candidates; specify the PNP IP manually")
    def ssh_open(item):
        peer, (interface, local) = item
        # ARP proved this host is on this physical link. Add only its host route.
        subprocess.run(["ip", "route", "replace", peer + "/32", "dev", interface, "src", local],
                       check=True, capture_output=True)
        try:
            with socket.create_connection((peer, 22), timeout=.7):
                return peer
        except OSError:
            return None
    with ThreadPoolExecutor(max_workers=8) as pool:
        reachable = list(filter(None, pool.map(ssh_open, candidates.items())))
    progress("checking_pnp_master_devices")
    matches = []
    for peer in sorted(reachable):
        host = user + "@" + peer
        # A previously initialized host can be identified with its restricted key.
        try:
            result = authenticated_command(host, "", "pnp-discover", timeout=10)
            if result.returncode and password:
                import shlex
                probe = Path("/deploy/pnp-remote/read_vendor_masters.py").read_text()
                result = authenticated_command(host, password, "python3 -c " + shlex.quote(probe), timeout=12)
        except subprocess.TimeoutExpired:
            continue
        if result.returncode:
            continue
        for line in result.stdout.splitlines():
            if line.startswith("PNP_PROBE="):
                data = json.loads(line.split("=", 1)[1])
                arms = data.get("arms", {})
                if (data.get("vendor_ready") is True and
                        all(arms.get(side, {}).get("ready") for side in ("right", "left"))):
                    matches.append(peer)
    if len(matches) != 1:
        raise RuntimeError(f"Expected one wired PNP with both accessible master USB devices; found {matches}. Check credentials or set the PNP IP manually.")
    return matches[0]
