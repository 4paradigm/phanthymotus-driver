#!/usr/bin/env python3
"""Lynx S10 basic_server TCP framing and client."""

from __future__ import annotations

import json
import hashlib
import hmac
import os
from pathlib import Path
import shlex
import shutil
import socket
import ssl
import struct
import subprocess
import threading
import time
from datetime import datetime


SYNC = b"\xeb\x91\xeb\x90"
HEADER = struct.Struct("<4sHHBBB5x")
MAX_ASDU = 1000
DEFAULT_TLS_SHA256_FINGERPRINT = (
    "6cd411bfaffed2c5d93f80d30edb514e6b194ef2e33a199a851bbff9ce50b415"
)
PLAINTEXT_GATE_TTL = 15.0

_DISABLE_TLS_SCRIPT = r'''set -eu
fail(){ echo "$*" >&2; exit 1; }
d=/var/opt/robot/conf/robot_server
f="$d/Network.toml"
u=robot_server.service
systemd-detect-virt --container >/dev/null || fail "请在目标容器内执行"
[ -f "$f" ] && [ ! -L "$f" ] || fail "配置文件不可用"
exec 9<"$d"
flock -n 9 || fail "已有脚本在执行"
key="^[[:space:]]*enableTls[[:space:]]*="
[ "$(grep -Ec "$key" "$f")" = 1 ] || fail "enableTls 缺失或重复"
if [ -f "$d/user_conf.toml" ] && grep -Eq "$key" "$d/user_conf.toml"; then
  fail "user_conf.toml 含 enableTls，请先核对覆盖配置"
fi
valid(){ awk -v v="$1" '/^[[:space:]]*\[/ {n=($0 ~ /^[[:space:]]*\[Network\][[:space:]]*(#.*)?$/)} n && $0 ~ ("^[[:space:]]*enableTls[[:space:]]*=[[:space:]]*" v "[[:space:]]*(#.*)?$") {ok=1} END {exit !ok}' "$f"; }
valid "(true|false)" || fail "[Network] 的 enableTls 格式异常"
if ! valid false; then
  b="$f.bak.$(date +%Y%m%d-%H%M%S).$$"
  cp -p "$f" "$b"
  echo "备份：$b"
  if ! sed -i -E 's/^([[:space:]]*enableTls[[:space:]]*=[[:space:]]*)true([[:space:]]*(#.*)?)$/\1false\2/' "$f" || ! valid false; then
    if cp -p "$b" "$f"; then fail "写入失败，已恢复原配置"; fi
    fail "写入失败，请从备份恢复：$b"
  fi
  sync
fi
echo "配置已确认：enableTls=false"
sce(){ python3 -c 'import socket,struct,json,datetime; m=bytes.fromhex("eb90eb90"); h=struct.Struct("<4sHHBBB5s"); b=json.dumps({"PatrolDevice":{"Type":1048676,"Command":1,"Time":datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),"Items":{}}}).encode(); s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.settimeout(1); s.connect(("127.0.0.1",40000)); s.send(h.pack(m,len(b),1,1,0,1,bytes(5))+b); r=s.recv(4096); magic,n,*_=h.unpack_from(r); p=json.loads(r[h.size:])["PatrolDevice"]; v=p["Items"]["SCE"]; assert magic==m and n==len(r)-h.size and p["Type"]==1048676 and p["Command"]==1 and type(v) is int and v in (0,1); print(v)' 2>/dev/null; }
state=$(sce || true)
if [ "$state" = 0 ]; then echo "加密已关闭，无需重启"; exit 0; fi
if ! systemctl is-active --quiet "$u"; then echo "配置已保存，请重启机器狗使配置生效。"; exit 0; fi
old=$(systemctl show -p InvocationID --value "$u")
[ -n "$old" ] || fail "配置已保存，无法读取服务实例，请重启机器狗使配置生效"
systemctl restart --no-block "$u" || fail "配置已保存，重启服务失败，请重启机器狗使配置生效"
echo "检测加密关闭中…"
deadline=$((SECONDS+60))
while [ "$SECONDS" -lt "$deadline" ]; do
  now=$(systemctl show -p InvocationID --value "$u" 2>/dev/null || true)
  if [ -n "$now" ] && [ "$now" != "$old" ] && systemctl is-active --quiet "$u" && valid false && [ "$(sce || true)" = 0 ]; then
    echo "加密已关闭"
    exit 0
  fi
  sleep 1
done
fail "配置已保存，检测超时，请重启机器狗后重新执行本命令确认状态"'''


def _protocol_int(value: object) -> int:
    """Parse protocol integers, including spaced hex strings used by examples."""
    if isinstance(value, int):
        return value
    text = "".join(str(value).strip().split())
    try:
        return int(text, 0)
    except ValueError:
        if text and all(char in "0123456789abcdefABCDEF" for char in text):
            return int(text, 16)
        raise


def encode_frame(message_id: int, message_type: int, command: int, items: dict | None = None) -> bytes:
    payload = json.dumps({"PatrolDevice": {
        "Type": int(message_type), "Command": int(command),
        "Time": datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
        "Items": items or {},
    }}, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    if len(payload) > MAX_ASDU:
        raise ValueError(f"ASDU exceeds documented {MAX_ASDU}-byte limit")
    return HEADER.pack(SYNC, len(payload), message_id & 0xFFFF, 1, 0, 1) + payload


def decode_frame(frame: bytes) -> tuple[int, dict]:
    if len(frame) < HEADER.size:
        raise ValueError("incomplete S10 frame header")
    sync, length, message_id, data_format, packet_number, version = HEADER.unpack_from(frame)
    if sync != SYNC or data_format != 1 or packet_number != 0 or version != 1:
        raise ValueError("invalid S10 sync or unsupported non-JSON ASDU")
    if length > MAX_ASDU or len(frame) != HEADER.size + length:
        raise ValueError("invalid S10 ASDU length")
    return message_id, json.loads(frame[HEADER.size:].decode("utf-8"))


class StreamDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.fragments = {}

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
            sync, length, message_id, data_format, packet_number, version = HEADER.unpack_from(self.buffer)
            if sync != SYNC or data_format != 1 or version != 1:
                del self.buffer[0]
                continue
            if length > MAX_ASDU:
                del self.buffer[0]
                continue
            total = HEADER.size + length
            if len(self.buffer) < total:
                break
            chunk = bytes(self.buffer[HEADER.size:total])
            del self.buffer[:total]
            if packet_number == 0:
                try:
                    payload = json.loads(chunk.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    payload = None
                if payload is not None:
                    frames.append((message_id, payload))
                    self.fragments.pop(message_id, None)
                    continue
                self.fragments[message_id] = {"next": 1, "payload": bytearray(chunk)}
                continue
            fragment = self.fragments.get(message_id)
            if fragment is None or packet_number != fragment["next"]:
                self.fragments.pop(message_id, None)
                continue
            if len(fragment["payload"]) + length > 65535:
                self.fragments.pop(message_id, None)
                continue
            fragment["payload"].extend(chunk)
            fragment["next"] += 1
            if length < MAX_ASDU:
                try:
                    payload = json.loads(fragment["payload"].decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    self.fragments.pop(message_id, None)
                    continue
                frames.append((message_id, payload))
                self.fragments.pop(message_id, None)
        return frames


class BasicServerClient:
    """TCP/TLS client for the S10 basic_server protocol."""

    def __init__(self, config: dict):
        native = config.get("basic_server", {})
        self.host = str(os.environ.get("S10_BASIC_SERVER_HOST", native.get("host", "10.21.33.103")))
        configured_transport = os.environ.get("S10_BASIC_SERVER_TRANSPORT") or native.get("transport")
        if not configured_transport:
            configured_transport = "tls" if (native.get("tls_client_cert") or
                                               native.get("tls_ca_file")) else "tcp"
        self.transport = str(configured_transport).lower()
        if self.transport not in {"tcp", "tls"}:
            raise ValueError("S10_BASIC_SERVER_TRANSPORT must be tcp or tls")
        self.port = int(native.get("tls_tcp_port" if self.transport == "tls" else "tcp_port",
                                   native.get("tcp_port", 30003)))
        self.timeout = float(native.get("timeout", 2.0))
        self.tls_server_name = str(os.environ.get(
            "S10_BASIC_SERVER_TLS_NAME", native.get("tls_server_name", "server.local")))
        self.tls_ca_file = os.environ.get("S10_BASIC_SERVER_TLS_CA") or native.get("tls_ca_file")
        self.tls_client_cert = (os.environ.get("S10_BASIC_SERVER_TLS_CLIENT_CERT") or
                                native.get("tls_client_cert"))
        self.tls_client_key = (os.environ.get("S10_BASIC_SERVER_TLS_CLIENT_KEY") or
                               native.get("tls_client_key"))
        if bool(self.tls_client_cert) != bool(self.tls_client_key):
            raise ValueError("S10 basic_server mTLS requires both client certificate and private key")
        configured_pin = os.environ.get("S10_BASIC_SERVER_TLS_SHA256") or native.get(
            "tls_sha256_fingerprint", DEFAULT_TLS_SHA256_FINGERPRINT)
        normalized_pin = str(configured_pin).lower().replace(":", "").replace(" ", "")
        if len(normalized_pin) != 64 or any(
                char not in "0123456789abcdef" for char in normalized_pin):
            raise ValueError("S10 basic_server TLS SHA-256 fingerprint must contain 64 hex digits")
        self.tls_sha256_fingerprint = normalized_pin
        self.config = config
        self.ssh_user = os.environ.get("S10_BASIC_SERVER_SSH_USER") or os.environ.get(
            "S10_AUDIO_SSH_USER", native.get("ssh_user", "user"))
        self.ssh_password = os.environ.get(
            "S10_BASIC_SERVER_SSH_PASSWORD",
            os.environ.get("S10_AUDIO_SSH_PASSWORD", os.environ.get("S10_DRMAP_PASSWORD", "")))
        self.ssh_known_hosts = os.environ.get(
            "S10_SSH_KNOWN_HOSTS",
            str(Path(__file__).with_name("resource") / "ssh_known_hosts"))
        self._gate_lock = threading.Lock()
        self._gate_verified_at = None
        self.enabled = bool(native.get("enabled", True))
        self._socket = None
        self._decoder = StreamDecoder()
        self._lock = threading.RLock()
        self._id_lock = threading.Lock()
        self._message_id = -1
        self._stop = threading.Event()
        self._thread = None
        self._reader_thread = None
        self._state_lock = threading.Lock()
        self._response_condition = threading.Condition()
        self._responses = {}
        self._pending = {}
        self.reports = {}
        self.last_error = None

    def _next_id(self):
        with self._id_lock:
            self._message_id = (self._message_id + 1) & 0xFFFF
            return self._message_id

    def _connect(self):
        if self._socket is None:
            if self.transport == "tcp":
                self._socket = socket.create_connection((self.host, self.port), timeout=self.timeout)
                self._socket.settimeout(None)
                return
            if not self.tls_client_cert:
                raise RuntimeError(
                    "S10 basic_server requires a vendor-issued TLS client certificate and private key")
            raw_socket = socket.create_connection((self.host, self.port), timeout=self.timeout)
            connection = None
            try:
                if self.tls_ca_file:
                    context = ssl.create_default_context(cafile=self.tls_ca_file)
                else:
                    context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
                    context.check_hostname = False
                    context.verify_mode = ssl.CERT_NONE
                context.load_cert_chain(self.tls_client_cert, self.tls_client_key)
                connection = context.wrap_socket(raw_socket, server_hostname=self.tls_server_name)
                certificate_digest = hashlib.sha256(connection.getpeercert(binary_form=True)).hexdigest()
                if not hmac.compare_digest(
                        certificate_digest, self.tls_sha256_fingerprint):
                    raise ssl.SSLCertVerificationError("S10 basic_server TLS certificate pin mismatch")
                connection.settimeout(None)
                self._socket = connection
            except Exception:
                (connection or raw_socket).close()
                raise

    def _remember(self, payload):
        body = payload.get("PatrolDevice", {})
        try:
            key = f"{_protocol_int(body.get('Type'))}:{_protocol_int(body.get('Command'))}"
        except (TypeError, ValueError):
            key = f"{body.get('Type')}:{body.get('Command')}"
        with self._state_lock:
            self.reports[key] = payload

    def request(self, message_type: int, command: int, items: dict | None = None, *, response_timeout: float | None = None):
        if not self.enabled:
            return {"state": "disabled"}
        with self._lock:
            self._connect()
            message_id = self._next_id()
            self._ensure_reader()
            with self._response_condition:
                self._pending[message_id] = (int(message_type), int(command))
            self._socket.sendall(encode_frame(message_id, message_type, command, items))
        deadline = time.monotonic() + (self.timeout if response_timeout is None else float(response_timeout))
        with self._response_condition:
            while message_id not in self._responses and time.monotonic() < deadline:
                self._response_condition.wait(deadline - time.monotonic())
            payload = self._responses.pop(message_id, None)
            self._pending.pop(message_id, None)
        if payload is None:
            raise TimeoutError("S10 TCP response timed out")
        return payload

    def send(self, message_type: int, command: int, items: dict | None = None):
        """Send a documented fire-and-forget command (axis/velocity have no reply)."""
        if not self.enabled:
            return {"state": "disabled"}
        with self._lock:
            self._connect()
            self._ensure_reader()
            message_id = self._next_id()
            self._socket.sendall(encode_frame(message_id, message_type, command, items))
            return {"state": "sent", "transport": "TCP", "message_id": message_id,
                    "type": int(message_type), "command": int(command)}

    def ensure_plaintext_transport(self):
        """Verify/disable AOS basic_server TLS before the frontend starts execution cards."""
        if not self.enabled:
            raise RuntimeError("S10 basic_server is disabled")
        with self._gate_lock:
            now = time.monotonic()
            if (self._gate_verified_at is not None
                    and now - self._gate_verified_at < PLAINTEXT_GATE_TTL):
                return {"state": "ready", "transport": "TCP", "cached": True}
            if not shutil.which("ssh"):
                raise RuntimeError("AOS TLS 检查失败：驱动容器中没有 ssh 客户端")
            target = f"{self.ssh_user}@{self.host}" if self.ssh_user else self.host
            sudo = ("sudo -S -p '' bash -c " if self.ssh_password
                    else "sudo -n bash -c ")
            remote = sudo + shlex.quote(_DISABLE_TLS_SCRIPT)
            command = ["ssh", "-T", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=yes",
                       "-o", f"UserKnownHostsFile={self.ssh_known_hosts}",
                       "-o", "GlobalKnownHostsFile=/dev/null", "-o", "ServerAliveInterval=5",
                       "-o", "ServerAliveCountMax=2"]
            env = None
            if self.ssh_password:
                if not shutil.which("sshpass"):
                    raise RuntimeError("AOS TLS 检查失败：配置了 AOS 密码但 sshpass 不可用")
                command = ["sshpass", "-e", *command]
                env = os.environ.copy()
                env["SSHPASS"] = self.ssh_password
            else:
                command.extend(["-o", "BatchMode=yes"])
            command.extend(["--", target, remote])
            sudo_stdin = f"{self.ssh_password}\n" if self.ssh_password else None
            try:
                result = subprocess.run(command, input=sudo_stdin, capture_output=True,
                                        text=True, timeout=75, check=False, env=env)
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise RuntimeError(f"AOS TLS 检查失败，执行卡未启动：{exc}") from exc
            output = "\n".join(part.strip() for part in (result.stdout, result.stderr) if part.strip())
            if "请重启机器狗" in output:
                raise RuntimeError(f"{output}\n请重启机器狗后，再次开启智能控制。")
            if result.returncode != 0:
                raise RuntimeError(f"AOS TLS 检查失败，执行卡未启动：{output or '远程脚本未成功'}")
            if "加密已关闭" not in output:
                raise RuntimeError(f"AOS TLS 检查未确认加密状态，执行卡未启动：{output or '无状态结果'}")
            self.close_socket()
            self.transport = "tcp"
            self.port = int(self.config.get("basic_server", {}).get("tcp_port", 30003))
            self._gate_verified_at = time.monotonic()
            return {"state": "ready", "transport": "TCP", "cached": False}

    def _ensure_reader(self):
        if self._reader_thread and self._reader_thread.is_alive():
            return
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True, name="s10-basic-server-reader")
        self._reader_thread.start()

    def _read_loop(self):
        while not self._stop.is_set():
            with self._lock:
                connection = self._socket
            if connection is None:
                self._stop.wait(0.1)
                continue
            try:
                data = connection.recv(65536)
                if not data:
                    raise ConnectionError("S10 TCP connection closed")
                for message_id, payload in self._decoder.feed(data):
                    self._remember(payload)
                    body = payload.get("PatrolDevice", {})
                    try:
                        response_key = (_protocol_int(body.get("Type")), _protocol_int(body.get("Command")))
                    except (TypeError, ValueError):
                        response_key = None
                    with self._response_condition:
                        # Some S10 firmware versions encode these fields as
                        # decimal strings, while request() always sends ints.
                        if self._pending.get(message_id) == response_key:
                            self._responses[message_id] = payload
                            self._response_condition.notify_all()
            except (OSError, ConnectionError) as exc:
                self.last_error = str(exc)
                self.close_socket()
                with self._response_condition:
                    self._response_condition.notify_all()

    def start(self):
        if not self.enabled or self._thread:
            return
        def heartbeat():
            while not self._stop.wait(1.0):
                try:
                    self.request(0x00100064, 0x00000005)
                    self.last_error = None
                except Exception as exc:
                    self.last_error = str(exc)
                    self.close_socket()
        self._thread = threading.Thread(target=heartbeat, daemon=True, name="s10-basic-server")
        self._thread.start()

    def snapshot(self):
        with self._state_lock:
            reports = dict(self.reports)
        return {"enabled": self.enabled, "transport": self.transport.upper(), "host": self.host,
                "port": self.port, "connected": self._socket is not None,
                "last_error": self.last_error, "reports": reports}

    def close_socket(self):
        with self._lock:
            if self._socket:
                try:
                    self._socket.close()
                except OSError:
                    pass
                self._socket = None
            self._decoder = StreamDecoder()

    def close(self):
        self._stop.set()
        self.close_socket()
        for thread in (self._thread, self._reader_thread):
            if thread and thread is not threading.current_thread():
                thread.join(timeout=1.0)
