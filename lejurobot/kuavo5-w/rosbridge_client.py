"""ROS 1 rosbridge v2 JSON client. Reconnect subscriptions, never replay motion."""
from __future__ import annotations

import json
import threading
import time
from uuid import uuid4

import websocket


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
        self._publishers = {}
        self._services = {}
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

    def close(self) -> None:
        self._closed.set()
        with self._lock:
            self._disconnect("rosbridge closed")
            ws = self._ws
        if ws is not None:
            ws.close()
        if self._thread.is_alive():
            self._thread.join(timeout=2)

    def subscribe(self, topic: str, message_type: str, callback) -> None:
        with self._lock:
            self._subscriptions[topic] = (message_type, callback)
            if self._connected.is_set():
                self._send_subscription(topic, message_type)

    def _send_subscription(self, topic: str, message_type: str) -> None:
        self._send({"op": "subscribe", "id": f"sub:{topic}", "topic": topic,
                    "type": message_type, "queue_length": 1, "throttle_rate": 0})

    def unsubscribe(self, topic: str) -> None:
        with self._lock:
            self._subscriptions.pop(topic, None)
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
            if not event.wait(timeout):
                raise TimeoutError(f"ROS service timed out (not retried): {service}")
            if result.get("result") is not True:
                raise RuntimeError(f"ROS service failed: {service}: {result.get('values')}")
            values = result.get("values")
            if not isinstance(values, dict):
                raise RuntimeError(f"Invalid service values: {service}")
            return values
        finally:
            with self._lock:
                self._services.pop(request_id, None)

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
            # Establish ROS 1 publishers before accepting one-shot commands. ROS 1
            # TCPROS negotiation is asynchronous; this 1s grace matches vendor examples.
            # It is not a delivery acknowledgement and must be validated on the robot.
            self._ready_at = time.monotonic() + 1.0
            self._connected.set()
            for topic, message_type in self._publishers.items():
                self._send_advertisement(topic, message_type)
            for topic, (message_type, _) in self._subscriptions.items():
                self._send_subscription(topic, message_type)
        print(f"[rosbridge] connected: {self.url}", flush=True)

    def _on_close(self, ws, _status, _message) -> None:
        with self._lock:
            if ws is self._ws:
                self._disconnect("rosbridge disconnected")
        print("[rosbridge] disconnected", flush=True)

    def _on_error(self, ws, error) -> None:
        with self._lock:
            if ws is self._ws:
                self._disconnect(str(error))
        print(f"[rosbridge] connection error: {error}", flush=True)

    def _on_message(self, ws, raw: str) -> None:
        try:
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
                if pending:
                    pending[1].update(packet)
                    pending[0].set()
                return
            if op == "status":
                if packet.get("level") == "error":
                    print(f"[rosbridge] protocol error: {packet}", flush=True)
                    # A rejected command cannot be treated as successful motion.
                    self._disconnect(str(packet.get("msg", "ROS protocol error")))
                    ws.close()
                return
            entry = self._subscriptions.get(packet.get("topic")) if op == "publish" else None
        if entry and isinstance(packet.get("msg"), dict):
            try:
                entry[1](packet["msg"])
            except Exception as exc:
                print(f"[rosbridge] callback failed: {type(exc).__name__}: {exc}", flush=True)

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
                app.run_forever(ping_interval=10, ping_timeout=5)
            except Exception as exc:
                self._on_error(app, exc)
            finally:
                with self._lock:
                    self._disconnect("rosbridge disconnected")
                    self._ws = None
            self._closed.wait(2)
