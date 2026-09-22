"""TRON 2 EDU upper-level WebSocket protocol (SDK guide V1.2, §3).

There is one reader and no automatic reconnection or command replay. A response
must match the configured robot, request GUID and response title.
"""
from concurrent.futures import Future
import copy
import json
import threading
import time
from urllib.parse import urlsplit
from uuid import uuid4


class TronClient:
    def __init__(self, endpoint, accid, timeout=2.0, socket_factory=None):
        self.endpoint, self.accid, self.timeout = endpoint, accid, timeout
        self._factory = socket_factory
        self._lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._pending = {}
        self._notifications = {}
        self._socket = None
        self._reader = None
        self._error = None

    @property
    def connected(self):
        with self._lock:
            return self._socket is not None

    def connect(self):
        with self._lock:
            if self._socket is not None:
                return
            uri = urlsplit(self.endpoint)
            if (uri.scheme not in ("ws", "wss") or not uri.hostname
                    or uri.username or uri.password or uri.query or uri.fragment):
                raise ValueError("configure a ws:// or wss:// endpoint without credentials/query")
            if not isinstance(self.accid, str) or not self.accid.strip():
                raise ValueError("configure the robot ACCID before connecting")
            if self._factory is None:
                import websocket
                factory = websocket.create_connection
            else:
                factory = self._factory
            self._socket = factory(self.endpoint, timeout=self.timeout,
                                   http_no_proxy=[uri.hostname], enable_multithread=True)
            self._notifications.clear()
            self._error = None
            sock = self._socket
            self._reader = threading.Thread(target=self._read, args=(sock,), daemon=True)
            self._reader.start()

    def _read(self, sock):
        try:
            while True:
                raw = sock.recv()
                if not raw:
                    raise ConnectionError("robot connection closed")
                if len(raw) > 1024 * 1024:
                    raise ValueError("oversized robot message")
                self._receive(json.loads(raw))
        except Exception:
            # Do not log vendor payloads/endpoints/identifiers.
            self._disconnect(sock, "robot transport lost; reconnect explicitly")

    def _receive(self, message):
        if not isinstance(message, dict):
            raise ValueError("invalid robot message")
        if message.get("accid") != self.accid:
            raise ValueError("robot identity mismatch")
        title, data = message.get("title", ""), message.get("data")
        if not isinstance(title, str) or not isinstance(data, dict):
            raise ValueError("invalid robot envelope")
        with self._lock:
            if title.startswith("response_"):
                pending = self._pending.get(message.get("guid"))
                if pending and title == pending[0]:
                    future = pending[1]
                    if not future.done():
                        future.set_result(copy.deepcopy(data))
            elif title in ("notify_robot_info", "notify_imu", "notify_twist"):
                self._notifications[title] = (copy.deepcopy(data), time.monotonic())

    def notification(self, title, max_age=2.5):
        with self._lock:
            value = self._notifications.get(title)
            if not self._socket or value is None or time.monotonic() - value[1] > max_age:
                raise RuntimeError("fresh robot feedback unavailable")
            return copy.deepcopy(value[0])

    def _send(self, title, data, guid):
        message = {"accid": self.accid, "title": title, "guid": guid,
                   "timestamp": time.time_ns() // 1_000_000, "data": data}
        with self._send_lock:
            with self._lock:
                sock = self._socket
            if sock is None:
                raise ConnectionError("robot is disconnected")
            try:
                sock.send(json.dumps(message, allow_nan=False))
            except Exception:
                self._disconnect(sock, "robot send failed; outcome unknown")
                raise ConnectionError("robot send failed; outcome unknown") from None

    def send(self, title, data):
        guid = uuid4().hex
        self._send(title, data, guid)
        return guid

    def request(self, title, data=None):
        guid, future = uuid4().hex, Future()
        with self._lock:
            self._pending[guid] = (title.replace("request_", "response_", 1), future)
        try:
            self._send(title, data or {}, guid)
            result = future.result(timeout=self.timeout)
            if result.get("result") != "success":
                raise RuntimeError("robot rejected request or returned an invalid result")
            return result
        except TimeoutError:
            raise TimeoutError("robot response timed out; outcome unknown, no retry") from None
        finally:
            with self._lock:
                self._pending.pop(guid, None)

    def _disconnect(self, sock, reason):
        with self._lock:
            if self._socket is not sock:
                return
            self._socket, self._error = None, reason
            self._notifications.clear()
            for _, future in self._pending.values():
                if not future.done():
                    future.set_exception(ConnectionError(reason))
        try:
            sock.close()
        except Exception:
            pass

    def close(self):
        with self._lock:
            sock = self._socket
        if sock is not None:
            self._disconnect(sock, "robot disconnected")
        if self._reader and self._reader is not threading.current_thread():
            self._reader.join(timeout=self.timeout + 0.5)
