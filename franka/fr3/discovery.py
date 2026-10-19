"""Bounded, read-only FR3 discovery on directly connected wired networks."""

from __future__ import annotations

import asyncio
import copy
import ipaddress
import json
from pathlib import Path
import re
import subprocess
import threading
import time

import aiohttp


def wired_networks():
    records = json.loads(subprocess.check_output(
        ["ip", "-j", "-4", "addr", "show"], text=True, timeout=3))
    networks = []
    local_addresses = {a["local"] for row in records for a in row.get("addr_info", [])}
    for row in records:
        device = Path("/sys/class/net") / row["ifname"]
        if (row.get("operstate") != "UP" or not (device / "device").exists()
                or (device / "wireless").exists()):
            continue
        for address in row.get("addr_info", []):
            if address.get("family") != "inet" or address.get("scope") != "global":
                continue
            network = ipaddress.ip_network(
                f'{address["local"]}/{address["prefixlen"]}', strict=False)
            if network.is_private and not network.is_loopback and 24 <= network.prefixlen <= 30:
                networks.append((row["ifname"], address["local"], network))
    # Never widen the scan to a corporate LAN, bridge, Wi-Fi or arbitrary CIDR.
    return networks[:4], local_addresses


def identify(ip, payload):
    if not isinstance(payload, dict):
        return None
    if (payload.get("status") != "Connected" or payload.get("robotCode") != "fr3_arm"
            or payload.get("robotLabel") != "Franka Research 3 Arm"):
        return None
    serial = payload.get("robotSerial")
    if not isinstance(serial, str) or not re.fullmatch(r"[A-Za-z0-9-]{3,64}", serial):
        return None
    return {"ip": ip, "serial": serial, "label": f"Franka FR3 · {serial} · {ip}"}


async def reject_redirect(session, context, params):
    raise ValueError("Discovery redirects are not permitted")


async def inspect_robot(session, ip):
    async with session.get(f"https://{ip}/desk/", allow_redirects=False) as response:
        if response.status != 200 or response.content_type != "text/html":
            return None
        body = bytearray()
        async for chunk in response.content.iter_chunked(8192):
            body.extend(chunk)
            if len(body) > 131072:
                return None
        text = body.decode("utf-8", errors="replace")
        if not (re.search(r"<title>\s*Franka (?:Robotics|Emika)\s*</title>", text)
                and re.search(r'src=["\']/app-shell/[^"\']+\.js["\']', text)):
            return None
    async with session.ws_connect(
        f"wss://{ip}/admin/api/robot/nameplate", max_msg_size=8192,
        receive_timeout=2, timeout=1, autoclose=True,
    ) as ws:
        message = await ws.receive()
        if message.type != aiohttp.WSMsgType.TEXT:
            return None
        try:
            return identify(ip, json.loads(message.data))
        except (ValueError, RecursionError):
            return None


async def scan_networks(networks, local_addresses, *, timeout=60):
    robots = []
    deadline = asyncio.get_running_loop().time() + timeout
    state, error = "ready", None
    semaphore = asyncio.Semaphore(12)
    trace = aiohttp.TraceConfig()
    trace.on_request_redirect.append(reject_redirect)
    for interface, source, network in networks:
        remaining = deadline - asyncio.get_running_loop().time()
        if remaining <= 0:
            state, error = "partial", "TimeoutError"
            break
        # IP-address discovery cannot validate Desk's robot.franka.de hostname.
        # This is a local device-type filter, not authentication. Send no secrets.
        connector = aiohttp.TCPConnector(ssl=False, local_addr=(source, 0), limit=12)
        async with aiohttp.ClientSession(
            connector=connector, timeout=aiohttp.ClientTimeout(total=3, sock_connect=0.6),
            cookie_jar=aiohttp.DummyCookieJar(), trust_env=False,
            headers={"Accept-Encoding": "identity"},
            auto_decompress=False, trace_configs=[trace],
        ) as session:
            async def probe(ip, interface=interface):
                async with semaphore:
                    try:
                        result = await asyncio.wait_for(inspect_robot(session, ip), 5)
                        if result:
                            result["interface"] = interface
                            robots.append(result)
                    except (OSError, ValueError, asyncio.TimeoutError, aiohttp.ClientError):
                        pass  # Unreachable hosts and non-Franka services are expected.

            probes = asyncio.gather(*(probe(str(ip)) for ip in network.hosts()
                                      if str(ip) not in local_addresses))
            try:
                await asyncio.wait_for(probes, remaining)
            except asyncio.TimeoutError:
                # wait_for cancels and drains unfinished probes. Keep devices
                # already identified even when another host exhausts the budget.
                state, error = "partial", "TimeoutError"
                break
    return {"robots": sorted(robots, key=lambda robot: ipaddress.ip_address(robot["ip"])),
            "state": state, "error": error}


class Discovery:
    """One cancellable scan, at most every two minutes; schemas read the cache."""

    def __init__(self):
        self._lock = threading.Lock()
        self._thread = None
        self._loop = None
        self._task = None
        self._closed = False
        self._next_scan = 0.0
        self._result = {"state": "idle", "robots": [], "networks": [], "error": None}

    def refresh(self):
        with self._lock:
            if (self._closed or (self._thread and self._thread.is_alive())
                    or time.monotonic() < self._next_scan):
                return
            self._result["state"] = "scanning"
            self._thread = threading.Thread(target=self._run, name="franka-discovery", daemon=True)
            self._thread.start()

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self._result)

    def _run(self):
        async def run():
            networks, addresses = wired_networks()
            result = await scan_networks(networks, addresses)
            return {**result,
                    "networks": [f"{name}: {net}" for name, _, net in networks],
                    "updated_at": time.time()}

        loop = asyncio.new_event_loop()
        try:
            with self._lock:
                if self._closed:
                    return
                self._loop = loop
                self._task = loop.create_task(run())
            result = loop.run_until_complete(self._task)
            with self._lock:
                self._result = result
        except asyncio.CancelledError:
            pass
        except Exception as exc:
            with self._lock:
                self._result.update(state="error", error=type(exc).__name__)
        finally:
            with self._lock:
                self._next_scan = time.monotonic() + 120
                self._loop = self._task = None
            loop.close()

    def close(self):
        with self._lock:
            self._closed = True
            thread = self._thread
            if self._loop and self._task:
                self._loop.call_soon_threadsafe(self._task.cancel)
        if thread and thread is not threading.current_thread():
            thread.join(timeout=5)
