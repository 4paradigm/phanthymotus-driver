"""ROS 1 rosbridge v2 JSON client. Reconnect subscriptions, never replay motion."""
from __future__ import annotations

import json
import threading
import time
from uuid import uuid4

import websocket


def _log_text(value, limit=200):
    # Slice before escaping: never format a complete untrusted packet or string.
    if not isinstance(value, str):
        return f"<{type(value).__name__}>"
    return value[:limit].encode("unicode_escape").decode("ascii")[:limit]


class RosbridgeClient:
    def __init__(self, url: str):
        self.url = url
        self._lock = threading.RLock()
        self._ws = None
        self._connected = threading.Event()
        self._closed = threading.Event()
        self._generation = 0
        self._ready_at = float("inf")
        self._subscriptions = {}
        self._subscription_rates = {}
        self._publishers = {}
        self._services = {}
        self._retained_services = set()
        self._callback_failures = {}
        self._thread = threading.Thread(target=self._run, daemon=True, name="rosbridge")

    @property
    def connection(self) -> tuple[bool, int]:
        with self._lock:
            return self._connected.is_set(), self._generation

    @property
    def motion_ready(self) -> bool:
        with self._lock:
            return self._connected.is_set() and time.monotonic() >= self._ready_at

    def advertise(self, topic: str, message_type: str) -> None:
        with self._lock:
            if self._publishers.get(topic) == message_type:
                return
            self._publishers[topic] = message_type
            if self._connected.is_set():
                self._send_advertisement(topic, message_type)
                self._ready_at = max(self._ready_at, time.monotonic() + 1.0)

    def _send_advertisement(self, topic: str, message_type: str) -> None:
        self._send({"op": "advertise", "id": f"adv:{topic}",
                    "topic": topic, "type": message_type})

    def start(self) -> None:
        self._thread.start()

    def _disconnect(self, message: str) -> None:
        # Called with the connection lock held; waiting service calls must fail now.
        self._connected.clear()
        for event, result in self._services.values():
            result.update({"result": False, "values": message})
            event.set()
        self._services.clear()
        self._retained_services.clear()

    def close(self) -> None:
        self._closed.set()
        with self._lock:
            self._disconnect("rosbridge closed")
            ws = self._ws
        if ws is not None:
            ws.close()
        if self._thread.is_alive():
            self._thread.join(timeout=2)

    def subscribe(self, topic: str, message_type: str, callback, throttle_ms=0) -> None:
        with self._lock:
            self._subscriptions[topic] = (message_type, callback)
            self._subscription_rates[topic] = throttle_ms
            if self._connected.is_set():
                self._send_subscription(topic, message_type)

    def _send_subscription(self, topic: str, message_type: str) -> None:
        self._send({"op": "subscribe", "id": f"sub:{topic}", "topic": topic,
                    "type": message_type, "queue_length": 1,
                    "throttle_rate": self._subscription_rates.get(topic, 0)})

    def unsubscribe(self, topic: str) -> None:
        with self._lock:
            self._subscriptions.pop(topic, None)
            self._subscription_rates.pop(topic, None)
            self._callback_failures.pop(topic, None)
            if self._connected.is_set():
                self._send({"op": "unsubscribe", "id": f"sub:{topic}", "topic": topic})

    def get_param(self, name: str, default=None):
        # rosapi transports parameter values as JSON-encoded strings.
        response = self.call_service('/rosapi/get_param', 'rosapi/GetParam',
                                     {'name': name, 'default': json.dumps(default)}, timeout=2.0)
        raw = response.get('value')
        if not isinstance(raw, str):
            raise ValueError(f'invalid ROS parameter response: {name}')
        return json.loads(raw)

    def get_params(self, defaults: dict, timeout=2.0):
        """Read a bounded parameter set concurrently within one shared deadline.

        Only read-only rosapi calls are pipelined; motion services are unchanged.
        No partial result is returned if any value fails or misses the deadline.
        """
        if not 1 <= len(defaults) <= 16:
            raise ValueError('expected 1 to 16 ROS parameters')
        pending, values = {}, {}
        deadline = time.monotonic() + timeout
        try:
            for name, default in defaults.items():
                pending[name] = self.start_service('/rosapi/get_param', 'rosapi/GetParam',
                    {'name': name, 'default': json.dumps(default)})
            for name, request in pending.items():
                response = self.wait_service(request, max(0., deadline - time.monotonic()))
                raw = response.get('value')
                if not isinstance(raw, str):
                    raise ValueError(f'invalid ROS parameter response: {name}')
                values[name] = json.loads(raw)
            return values
        finally:
            # A failed early response must not leak the other pending reads.
            with self._lock:
                for request in pending.values():
                    self._services.pop(request[0], None)

    def publish(self, topic: str, message_type: str, message: dict) -> None:
        with self._lock:
            if not self._connected.is_set():
                raise ConnectionError("rosbridge is disconnected")
            if self._publishers.get(topic) != message_type:
                self._send_advertisement(topic, message_type)
                self._publishers[topic] = message_type
            self._send({"op": "publish", "id": f"pub:{topic}",
                        "topic": topic, "msg": message})

    def call_service(self, service: str, service_type: str, args: dict,
                     timeout: float = 5.0) -> dict:
        return self.wait_service(self.start_service(service, service_type, args), timeout)

    def start_service(self, service: str, service_type: str, args: dict):
        """Send atomically, then let the caller wait WITHOUT a motion lock."""
        # ROS 1 resolves service types server-side; protocol call_service has no type field.
        request_id = f"kuavo_{uuid4().hex}"
        event, result = threading.Event(), {}
        try:
            with self._lock:
                if not self._connected.is_set():
                    raise ConnectionError("rosbridge is disconnected")
                self._services[request_id] = (event, result)
                self._send({"op": "call_service", "id": request_id,
                            "service": service, "args": args})
            return request_id, event, result, service
        except Exception:
            with self._lock:
                self._services.pop(request_id, None)
            raise

    def wait_service(self, pending, timeout: float, cancelled=None) -> dict:
        request_id, event, result, service = pending
        deadline = time.monotonic() + timeout
        try:
            while True:
                if cancelled is not None and cancelled.is_set():
                    raise InterruptedError('control request superseded by stop')
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"ROS service timed out (not retried): {service}")
                if event.wait(min(0.02, remaining)):
                    break
            if cancelled is not None and cancelled.is_set():
                raise InterruptedError('control request superseded by stop')
            if result.get("result") is not True:
                raise RuntimeError(f"ROS service failed: {service}: {_log_text(result.get('values'))}")
            values = result.get("values")
            if not isinstance(values, dict):
                raise RuntimeError(f"Invalid service values: {service}")
            return values
        finally:
            with self._lock:
                if request_id not in self._retained_services:
                    self._services.pop(request_id, None)

    def retain_service(self, pending):
        """Keep a hazardous control response observable after local cancellation.

        The bundle admits at most one unresolved resume and one mode write. No command is retried;
        the entry is removed on response/disconnect, while its handle survives.
        """
        with self._lock:
            if pending[0] in self._services:
                self._retained_services.add(pending[0])

    def service_result(self, pending):
        """Return a controller response only; transport loss is not rejection."""
        with self._lock:
            _, event, result, _ = pending
            values = result.get('values')
            if event.is_set() and result.get('result') is True and isinstance(values, dict):
                return dict(values)
            return None

    def _send(self, value: dict) -> None:
        if self._ws is None:
            raise ConnectionError("rosbridge is disconnected")
        try:
            self._ws.send(json.dumps(value, separators=(",", ":"), allow_nan=False))
        except Exception:
            self._disconnect("rosbridge send failed")
            raise

    def _on_open(self, ws) -> None:
        with self._lock:
            if self._closed.is_set() or ws is not self._ws:
                ws.close()
                return
            self._generation += 1
            self._callback_failures.clear()
            # Establish ROS 1 publishers before accepting one-shot commands. ROS 1
            # TCPROS negotiation is asynchronous; this 1s grace matches vendor examples.
            # It is not a delivery acknowledgement and must be validated on the robot.
            self._ready_at = time.monotonic() + 1.0
            self._connected.set()
            for topic, message_type in self._publishers.items():
                self._send_advertisement(topic, message_type)
            for topic, (message_type, _) in self._subscriptions.items():
                self._send_subscription(topic, message_type)
        print(f"[rosbridge] connected: {_log_text(self.url)}", flush=True)

    def _on_close(self, ws, _status, _message) -> None:
        with self._lock:
            if ws is self._ws:
                self._disconnect("rosbridge disconnected")
        print("[rosbridge] disconnected", flush=True)

    def _on_error(self, ws, error) -> None:
        with self._lock:
            if ws is self._ws:
                self._disconnect(str(error))
        print(f"[rosbridge] connection error: {_log_text(str(error))}", flush=True)

    def _on_message(self, ws, raw: str) -> None:
        try:
            # Use CPython's strict decoder instead of websocket-client's slow
            # pure-Python per-byte validator. Never accept replacement characters
            # for malformed UTF-8 (json.loads(bytes) also accepts UTF-16/32).
            if isinstance(raw, bytes):
                raw = raw.decode('utf-8', errors='strict')
            packet = json.loads(raw)
        except (TypeError, ValueError):
            return
        if not isinstance(packet, dict):
            return
        with self._lock:
            if ws is not self._ws or not self._connected.is_set():
                return
            op = packet.get("op")
            if op == "service_response":
                pending = self._services.pop(packet.get("id"), None)
                self._retained_services.discard(packet.get('id'))
                if pending:
                    pending[1].update(packet)
                    pending[0].set()
                return
            if op == "status":
                if packet.get("level") == "error":
                    message = _log_text(packet.get("msg", "ROS protocol error"))
                    print(f"[rosbridge] protocol error id={_log_text(packet.get('id'))}: {message}", flush=True)
                    # A rejected command cannot be treated as successful motion.
                    self._disconnect(message)
                    ws.close()
                return
            entry = self._subscriptions.get(packet.get("topic")) if op == "publish" else None
        if entry and isinstance(packet.get("msg"), dict):
            topic = packet['topic']
            try:
                entry[1](packet["msg"])
            except Exception as exc:
                with self._lock:
                    failure = self._callback_failures.setdefault(topic, {'count': 0, 'logged_at': None})
                    failure['count'] += 1
                    now = time.monotonic()
                    if failure['logged_at'] is None or now - failure['logged_at'] >= 5.0:
                        failure['logged_at'] = now
                        print(f"[rosbridge] callback failed topic={_log_text(topic)} count={failure['count']}: "
                              f"{type(exc).__name__}: {_log_text(str(exc))}", flush=True)
            else:
                with self._lock:
                    failure = self._callback_failures.pop(topic, None)
                    if failure:
                        print(f"[rosbridge] callback recovered topic={_log_text(topic)} "
                              f"after {failure['count']} failures", flush=True)

    def _run(self) -> None:
        while not self._closed.is_set():
            app = websocket.WebSocketApp(
                self.url, on_open=self._on_open, on_message=self._on_message,
                on_error=self._on_error, on_close=self._on_close)
            with self._lock:
                if self._closed.is_set():
                    return
                self._ws = app
            try:
                app.run_forever(ping_interval=10, ping_timeout=5, skip_utf8_validation=True)
            except Exception as exc:
                self._on_error(app, exc)
            finally:
                with self._lock:
                    self._disconnect("rosbridge disconnected")
                    self._ws = None
            self._closed.wait(2)
