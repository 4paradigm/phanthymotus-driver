"""山猫 M20 basic_server TCP/UDP 原生协议客户端。"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from datetime import datetime


SYNC = b"\xeb\x91\xeb\x90"
HEADER = struct.Struct("<4sHHB7x")


def encode_frame(message_id: int, message_type: int, command: int, items: dict | None = None) -> bytes:
    payload = json.dumps(
        {
            "PatrolDevice": {
                "Type": int(message_type),
                "Command": int(command),
                "Time": datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                "Items": items or {},
            }
        },
        ensure_ascii=False,
        separators=(",", ":"),
    ).encode("utf-8")
    return HEADER.pack(SYNC, len(payload), message_id & 0xFFFF, 1) + payload


def decode_frame(frame: bytes) -> tuple[int, dict]:
    if len(frame) < HEADER.size:
        raise ValueError("basic_server 报文头不完整")
    sync, length, message_id, data_format = HEADER.unpack_from(frame)
    if sync != SYNC:
        raise ValueError("basic_server 同步字符错误")
    if data_format != 1:
        raise ValueError("当前 Driver 仅支持官方推荐的 JSON ASDU")
    if len(frame) != HEADER.size + length:
        raise ValueError("basic_server ASDU 长度不匹配")
    return message_id, json.loads(frame[HEADER.size:].decode("utf-8"))


class StreamDecoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, dict]]:
        self.buffer.extend(data)
        frames = []
        while True:
            offset = self.buffer.find(SYNC)
            if offset < 0:
                self.buffer[:] = self.buffer[-3:]
                break
            if offset:
                del self.buffer[:offset]
            if len(self.buffer) < HEADER.size:
                break
            _, length, _, _ = HEADER.unpack_from(self.buffer)
            total = HEADER.size + length
            if len(self.buffer) < total:
                break
            frames.append(decode_frame(bytes(self.buffer[:total])))
            del self.buffer[:total]
        return frames


class BasicServerClient:
    """可靠指令走 TCP，高频速度指令走 UDP，并用 1Hz 心跳接收状态上报。"""

    def __init__(self, config: dict):
        native = config.get("basic_server", {})
        self.host = str(native.get("host", "10.21.31.103"))
        self.udp_port = int(native.get("udp_port", 30000))
        self.tcp_port = int(native.get("tcp_port", 30001))
        self.timeout = float(native.get("timeout", 1.5))
        self.enabled = bool(native.get("enabled", True))
        self._tcp = None
        self._udp = None
        self._decoder = StreamDecoder()
        # Reliable TCP requests may block for up to ``timeout`` while waiting
        # for a reply.  Keep them independent from the 20 Hz UDP velocity
        # stream so a slow heartbeat can never delay motion or its zero frame.
        self._tcp_lock = threading.Lock()
        self._udp_lock = threading.Lock()
        self._message_id_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._message_id = 0
        self._stop = threading.Event()
        self._thread = None
        self.latest = {}
        self.last_error = None

    def _next_id(self) -> int:
        with self._message_id_lock:
            self._message_id = (self._message_id + 1) & 0xFFFF
            return self._message_id

    def _connect_tcp(self):
        if self._tcp is None:
            self._tcp = socket.create_connection((self.host, self.tcp_port), timeout=self.timeout)
            self._tcp.settimeout(self.timeout)

    def _remember(self, payload: dict) -> None:
        body = payload.get("PatrolDevice", {})
        key = f"{body.get('Type')}:{body.get('Command')}"
        with self._state_lock:
            self.latest[key] = payload

    def request(self, message_type: int, command: int, items: dict | None = None) -> dict:
        if not self.enabled:
            return {"state": "disabled"}
        with self._tcp_lock:
            self._connect_tcp()
            message_id = self._next_id()
            self._tcp.sendall(encode_frame(message_id, message_type, command, items))
            deadline = time.monotonic() + self.timeout
            while time.monotonic() < deadline:
                for received_id, payload in self._decoder.feed(self._tcp.recv(65536)):
                    self._remember(payload)
                    body = payload.get("PatrolDevice", {})
                    if received_id == message_id and body.get("Type") == int(message_type) and body.get("Command") == int(command):
                        return payload
            raise TimeoutError("basic_server TCP 响应超时")

    def send_velocity(self, command: int, items: dict) -> dict:
        if not self.enabled:
            return {"state": "disabled"}
        with self._udp_lock:
            if self._udp is None:
                self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            message_id = self._next_id()
            self._udp.sendto(encode_frame(message_id, 2, command, items), (self.host, self.udp_port))
        return {"state": "sent", "transport": "udp", "type": 2, "command": command, "message_id": message_id}

    def start(self) -> None:
        if not self.enabled or self._thread:
            return
        def heartbeat():
            while not self._stop.wait(1.0):
                try:
                    self.request(100, 100)
                    self.last_error = None
                except Exception as exc:
                    self.last_error = str(exc)
                    self._close_tcp()
        self._thread = threading.Thread(target=heartbeat, daemon=True, name="m20-basic-server")
        self._thread.start()

    def snapshot(self) -> dict:
        with self._state_lock:
            latest = dict(self.latest)
        return {"enabled": self.enabled, "host": self.host, "connected": self._tcp is not None, "last_error": self.last_error, "reports": latest}

    def _close_tcp(self):
        with self._tcp_lock:
            if self._tcp:
                try: self._tcp.close()
                except OSError: pass
                self._tcp = None
            self._decoder = StreamDecoder()

    def _close_udp(self):
        with self._udp_lock:
            if self._udp:
                try: self._udp.close()
                except OSError: pass
                self._udp = None

    def _close_sockets(self):
        self._close_tcp()
        self._close_udp()

    def close(self) -> None:
        self._stop.set()
        self._close_sockets()
