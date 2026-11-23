"""Receive iNex joint/state notifications; never request robot control."""

from __future__ import annotations

import asyncio
import copy
import json
import math
import threading
import time
from urllib.parse import urlsplit


# Same ordering as iNex robot_model.yaml and its 3D viewer. Angles are radians.
GROUPS = {
    "left_arm": tuple(f"left_joint_{i}" for i in range(1, 8)),
    "right_arm": tuple(f"right_joint_{i}" for i in range(1, 8)),
    "waist": tuple(f"body_joint_{i}" for i in range(1, 5)),
    "head": tuple(f"head_joint_{i}" for i in range(1, 3)),
}
JOINT_NAMES = tuple(name for group in GROUPS.values() for name in group)


def positive_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a finite positive number")
    return float(value)


def validate_url(url):
    parts = urlsplit(url)
    if parts.scheme not in ("ws", "wss") or not parts.hostname or parts.username or parts.password or parts.fragment:
        raise ValueError("state_url must be a ws:// or wss:// endpoint without credentials or fragment")
    _ = parts.port  # Validate port syntax/range too.
    return url


def parse_state(message):
    """Reject partial/invalid frames instead of inventing missing joint values."""
    if not isinstance(message, dict):
        raise ValueError("expected a JSON object")
    if message.get("method") != "joint/state":
        return None
    data = message.get("params")
    if not isinstance(data, dict) or not isinstance(data.get("pos"), dict):
        raise ValueError("joint/state has no position object")
    velocity = data.get("vel")
    if velocity is not None and not isinstance(velocity, dict):
        raise ValueError("velocity must be an object")
    joints = []
    for group, names in GROUPS.items():
        pos = data["pos"].get(group)
        vel = None if velocity is None else velocity.get(group)
        for field, values, required in (("pos", pos, True), ("vel", vel, False)):
            if values is None and not required:
                continue
            if not isinstance(values, list) or len(values) != len(names):
                raise ValueError(f"{field}.{group} must have {len(names)} values")
            if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in values):
                raise ValueError(f"{field}.{group} contains a non-finite or nonnumeric value")
        for i, name in enumerate(names):
            joint = {"idx": len(joints), "name": name, "q": float(pos[i])}
            if vel is not None:
                joint["dq"] = float(vel[i])
            joints.append(joint)
    return joints


class StreamFeedbackClient:
    """One cancellable subscriber with a monotonic freshness deadline."""

    def __init__(self, url, *, parser, data_key, binary=False, subscription=None,
                 max_msg_size=65536, stale_timeout_s=1.0, reconnect_delay_s=1.0,
                 connect_timeout_s=3.0):
        self.url = validate_url(url)
        self._parser = parser
        self._data_key = data_key
        self._binary = binary
        self._subscription = subscription
        self._max_msg_size = max_msg_size
        self.stale_timeout = positive_number(stale_timeout_s, "stale_timeout_s")
        self.reconnect_delay = positive_number(reconnect_delay_s, "reconnect_delay_s")
        self.connect_timeout = positive_number(connect_timeout_s, "connect_timeout_s")
        self._lifecycle = threading.RLock()
        self._lock = threading.Lock()
        self._thread = None
        self._loop = None
        self._task = None
        self._stop = threading.Event()
        self._running = False
        self._connected = False
        self._data = None
        self._received = None
        self._received_wall = None
        self._error = ""
        self._frames = 0

    def start(self):
        with self._lifecycle:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop.clear()
            with self._lock:
                self._running = True
                self._error = ""
            self._thread = threading.Thread(target=self._run, name="helios-feedback", daemon=True)
            self._thread.start()

    def stop(self):
        with self._lifecycle:
            self._stop.set()
            with self._lock:
                loop, task = self._loop, self._task
            if loop is not None and task is not None:
                try:
                    loop.call_soon_threadsafe(task.cancel)
                except RuntimeError:
                    pass  # The worker has already closed its event loop.
            if self._thread is not None:
                self._thread.join(timeout=self.connect_timeout + 2)
                if self._thread.is_alive():
                    raise RuntimeError("feedback worker did not stop; refusing a competing subscriber")
                self._thread = None
            with self._lock:
                self._running = self._connected = False
                self._data = None
                self._error = ""

    def snapshot(self):
        with self._lock:
            age = None if self._received is None else max(0, time.monotonic() - self._received)
            available = bool(self._running and self._connected and self._data is not None
                             and age is not None and age <= self.stale_timeout)
            state = ("idle" if not self._running else "online" if available else
                     "stale" if self._connected and age is not None and age > self.stale_timeout else
                     "waiting" if self._connected and not self._error else
                     "invalid" if self._connected else "reconnecting")
            return {
                "state": state, "available": available, "connected": self._connected,
                "age_ms": None if age is None else round(age * 1000),
                "received_at_ms": self._received_wall, "timestamp_source": "driver_receive",
                "source_timestamp": None, "frames_received": self._frames,
                "error": self._error, self._data_key: copy.deepcopy(self._data) if available else [],
            }

    def _run(self):
        async def runner():
            with self._lock:
                self._loop = asyncio.get_running_loop()
                self._task = asyncio.current_task()
            try:
                if not self._stop.is_set():
                    await self._subscribe_forever()
            except asyncio.CancelledError:
                pass
            finally:
                with self._lock:
                    self._connected = False
                    self._data = None
                    self._loop = self._task = None
        try:
            asyncio.run(runner())
        except Exception as exc:
            with self._lock:
                self._error = f"subscriber failed: {type(exc).__name__}"

    async def _subscribe_forever(self):
        import aiohttp

        timeout = aiohttp.ClientTimeout(total=None, sock_connect=self.connect_timeout)
        async with aiohttp.ClientSession(timeout=timeout, trust_env=False) as session:
            while not self._stop.is_set():
                try:
                    # Entire handshake is bounded, including slow/nonresponsive HTTP servers.
                    ws = await asyncio.wait_for(session.ws_connect(self.url, max_msg_size=self._max_msg_size),
                                                self.connect_timeout)
                    try:
                        with self._lock:
                            self._connected = True
                            self._data = None
                            self._error = ""
                        if self._subscription is not None:
                            await ws.send_json(self._subscription)
                        last_good = time.monotonic()
                        while not self._stop.is_set():
                            if time.monotonic() - last_good > self.stale_timeout:
                                raise TimeoutError(f"{self._data_key} state expired")
                            try:
                                frame = await ws.receive(timeout=min(0.25, self.stale_timeout))
                            except asyncio.TimeoutError:
                                continue
                            expected = aiohttp.WSMsgType.BINARY if self._binary else aiohttp.WSMsgType.TEXT
                            if frame.type == expected:
                                try:
                                    joints = self._parser(frame.data if self._binary else json.loads(frame.data))
                                except (ValueError, TypeError) as exc:
                                    with self._lock:
                                        self._data = None
                                        self._error = str(exc)
                                    continue
                                if joints is not None:
                                    last_good = time.monotonic()
                                    with self._lock:
                                        self._data = joints
                                        self._received = last_good
                                        self._received_wall = int(time.time() * 1000)
                                        self._frames += 1
                                        self._error = ""
                            elif frame.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED,
                                                aiohttp.WSMsgType.ERROR):
                                raise ConnectionError("iNex closed the state stream")
                    finally:
                        # Closing the socket also removes the subscription on iNex.
                        await asyncio.wait_for(ws.close(), timeout=1)
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    with self._lock:
                        # Avoid logging endpoint credentials/response bodies from transport errors.
                        self._error = (str(exc) if type(exc) in (TimeoutError, ConnectionError)
                                       else f"state connection failed: {type(exc).__name__}")
                finally:
                    with self._lock:
                        self._connected = False
                        self._data = None
                await asyncio.sleep(self.reconnect_delay)


class FeedbackClient(StreamFeedbackClient):
    """Original iNex joints subscription contract."""

    def __init__(self, url, **kwargs):
        super().__init__(url, parser=parse_state, data_key="joints",
                         subscription={"jsonrpc": "2.0", "method": "subscribe", "params": ["joint/state"]},
                         **kwargs)


def parse_scope(message):
    """Current installed iNex JSON maps plain groups to motor_torque (Nm)."""
    if not isinstance(message, dict):
        raise ValueError("expected a JSON object")
    if message.get("method") != "joint/scope":
        return None
    data = message.get("params")
    if not isinstance(data, dict):
        raise ValueError("joint/scope has no data object")
    rows = []
    for group, names in GROUPS.items():
        motor = data.get(group)
        gravity = data.get(group + "_gravity") if group != "head" else None
        for field, values, required in ((group, motor, True), (group + "_gravity", gravity, False)):
            if values is None and not required:
                continue
            if not isinstance(values, list) or len(values) != len(names):
                raise ValueError(f"{field} must have {len(names)} values")
            if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in values):
                raise ValueError(f"{field} contains a non-finite or nonnumeric value")
        for i, name in enumerate(names):
            rows.append({"name": name, "motor_torque_nm": float(motor[i]),
                         "gravity_torque_nm": None if gravity is None else float(gravity[i])})
    return rows


class ScopeFeedbackClient(StreamFeedbackClient):
    def __init__(self, url, **kwargs):
        super().__init__(url, parser=parse_scope, data_key="torques",
                         subscription={"jsonrpc": "2.0", "method": "subscribe", "params": ["joint/scope"]},
                         **kwargs)
