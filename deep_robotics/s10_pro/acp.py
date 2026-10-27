"""Durable, acknowledged ACP terminal delivery for S10 Pro actions.

This module never creates actions or publishes robot commands. One owner process
uses one outbox file. A terminal payload is immutable once submitted; network
retries use the same action ID and bytes. Core's matched/cached/duplicate receipt
provides deduplication, including an early callback or a lost acknowledgement.

Construction performs no network I/O. Call ``start()`` explicitly in the driver,
or ``process_pending()`` in an isolated test. Pending records survive restart.
After the retry deadline/attempt limit they remain in the outbox as
``unacknowledged`` (or ``not_delivered`` if never attempted), never as delivered.
No manual trigger or alternative completion-notification channel is used.
"""
from __future__ import annotations

import hashlib
import hmac
import http.client
import json
import math
import os
from pathlib import Path
import sqlite3
import ssl
import threading
import time
from dataclasses import dataclass
from urllib.parse import urlsplit


TERMINAL_STATUSES = frozenset({"completed", "error", "cancelled"})
ACK_STATES = frozenset({"matched", "cached", "duplicate"})
MAX_PAYLOAD_BYTES = 32768
MAX_RESPONSE_BYTES = 65536


class ACPDeliveryError(RuntimeError):
    """A stable, non-secret transport diagnostic."""

    def __init__(self, code, message):
        self.code = code
        super().__init__(message)


@dataclass(frozen=True)
class ACPResponse:
    http_status: int
    body: object


def _strict_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError("Duplicate JSON field.")
        result[key] = value
    return result


def _decode_json(raw):
    return json.loads(raw, object_pairs_hook=_strict_object,
                      parse_constant=lambda value: (_ for _ in ()).throw(ValueError("Non-finite JSON number.")))


def _read_access_token():
    """Read current credentials without retaining or logging them."""
    value = os.environ.get("ACCESS_TOKEN", "").strip()
    if not value:
        configured = os.environ.get("S10_ACP_ENV_FILE")
        candidates = [configured] if configured else ["/opt/phanthy-motus/.env", "/work/.env"]
        for candidate in candidates:
            try:
                for line in Path(candidate).read_text(encoding="utf-8").splitlines():
                    line = line.strip()
                    if line.startswith("ACCESS_TOKEN="):
                        value = line.partition("=")[2].strip()
                        break
            except OSError:
                continue
            if value:
                break
    if "\r" in value or "\n" in value:
        raise ACPDeliveryError("invalid_credentials", "ACP credentials contain invalid characters.")
    return value


class HTTPSACPTransport:
    """Authenticated HTTPS delivery, with CA/hostname or explicit leaf trust.

    ``pinned_leaf_file`` is an operator-provided PEM certificate, not a certificate
    downloaded automatically. In that mode CERT_REQUIRED validates against the
    supplied trust anchor and an exact SHA-256 leaf comparison happens before
    sending credentials or the body. The exact pin replaces hostname matching.
    Otherwise normal CA validation and hostname matching are both required.
    Redirects and plaintext HTTP are never followed.
    """

    def __init__(self, base_url=None, *, ca_file=None, pinned_leaf_file=None,
                 timeout_s=3.0, token_provider=None):
        base_url = base_url or os.environ.get("AGENT_CORE_URL", "https://localhost:15678")
        parsed = urlsplit(base_url)
        if (parsed.scheme != "https" or not parsed.hostname or parsed.username is not None
                or parsed.password is not None or parsed.query or parsed.fragment):
            raise ValueError("ACP requires an HTTPS base URL without credentials, query, or fragment.")
        try:
            port = parsed.port or 443
        except ValueError as exc:
            raise ValueError("ACP HTTPS port is invalid.") from exc
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 5:
            raise ValueError("ACP request timeout must be greater than 0 and at most 5 seconds.")
        ca_file = ca_file or os.environ.get("S10_ACP_CA_FILE")
        pinned_leaf_file = pinned_leaf_file or os.environ.get("S10_ACP_PINNED_LEAF_FILE")
        if ca_file and pinned_leaf_file:
            raise ValueError("Configure either an ACP CA file or an exact pinned leaf file.")
        self._leaf_digest = None
        if pinned_leaf_file:
            pem = Path(pinned_leaf_file).read_text(encoding="ascii")
            if pem.count("-----BEGIN CERTIFICATE-----") != 1:
                raise ValueError("The ACP pinned leaf file must contain exactly one PEM certificate.")
            der = ssl.PEM_cert_to_DER_cert(pem)
            self._leaf_digest = hashlib.sha256(der).digest()
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
            context.check_hostname = False
            context.verify_mode = ssl.CERT_REQUIRED
            context.load_verify_locations(cadata=pem)
        else:
            context = ssl.create_default_context(cafile=ca_file)
        self._context = context
        self._host, self._port = parsed.hostname, port
        self._path = parsed.path.rstrip("/") + "/api/acp/complete"
        self._timeout = float(timeout_s)
        self._token_provider = token_provider or _read_access_token

    def __call__(self, payload):
        connection = http.client.HTTPSConnection(self._host, self._port,
                                                  timeout=self._timeout, context=self._context)
        try:
            connection.connect()
            if self._leaf_digest is not None:
                peer = connection.sock.getpeercert(binary_form=True)
                if not peer or not hmac.compare_digest(hashlib.sha256(peer).digest(), self._leaf_digest):
                    raise ACPDeliveryError("certificate_pin_mismatch", "ACP server certificate does not match the configured leaf.")
            headers = {"Content-Type": "application/json", "Accept": "application/json"}
            token = self._token_provider()
            if token:
                if not isinstance(token, str) or "\r" in token or "\n" in token:
                    raise ACPDeliveryError("invalid_credentials", "ACP credentials contain invalid characters.")
                headers["Authorization"] = "Bearer " + token
            data = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
                              allow_nan=False).encode("utf-8")
            connection.request("POST", self._path, body=data, headers=headers)
            response = connection.getresponse()
            raw = response.read(MAX_RESPONSE_BYTES + 1)
            if len(raw) > MAX_RESPONSE_BYTES:
                raise ACPDeliveryError("invalid_ack", "ACP acknowledgement exceeds the response size limit.")
            try:
                body = _decode_json(raw)
            except (ValueError, UnicodeError):
                body = None
            return ACPResponse(response.status, body)
        except ACPDeliveryError:
            raise
        except ssl.SSLError as exc:
            raise ACPDeliveryError("tls_failed", "ACP TLS verification or negotiation failed.") from exc
        except Exception as exc:
            # Exception strings can contain URLs, headers, or other credentials.
            raise ACPDeliveryError("transport_failed", "ACP HTTPS delivery did not return a valid receipt.") from exc
        finally:
            connection.close()


def _number(value, name, minimum, maximum):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be a finite number from {minimum} to {maximum}.")
    return float(value)


def _payload_json(payload):
    if not isinstance(payload, dict):
        raise ValueError("ACP terminal payload must be an object.")
    if set(payload) - {"action_id", "status", "result", "tool", "request_id", "ts"}:
        raise ValueError("ACP terminal payload contains unknown fields.")
    action_id = payload.get("action_id")
    if not isinstance(action_id, str) or not action_id.strip() or len(action_id) > 256 or any(ord(c) < 32 for c in action_id):
        raise ValueError("ACP action_id must be a non-empty string of at most 256 characters.")
    if not isinstance(payload.get("status"), str) or payload["status"] not in TERMINAL_STATUSES:
        raise ValueError("ACP status must be completed, error, or cancelled.")
    if not isinstance(payload.get("result"), dict):
        raise ValueError("ACP result must be an object.")
    for name in ("tool", "request_id"):
        if name in payload and (not isinstance(payload[name], str) or not payload[name].strip() or len(payload[name]) > 256):
            raise ValueError(f"ACP {name} must be a non-empty string of at most 256 characters.")
    if "ts" in payload:
        _number(payload["ts"], "ACP ts", 0, 1e14)
    def validate_json(value, depth=0):
        if depth > 32:
            raise ValueError("ACP terminal payload exceeds the JSON nesting limit.")
        if isinstance(value, dict):
            if any(not isinstance(key, str) for key in value):
                raise ValueError("ACP JSON object keys must be strings.")
            for child in value.values():
                validate_json(child, depth + 1)
        elif isinstance(value, list):
            for child in value:
                validate_json(child, depth + 1)
        elif value is None or isinstance(value, (str, bool, int)):
            return
        elif isinstance(value, float) and math.isfinite(value):
            return
        else:
            raise ValueError("ACP terminal payload must contain finite JSON values.")

    validate_json(payload)
    try:
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)
        size = len(encoded.encode("utf-8"))
    except (TypeError, ValueError, UnicodeError) as exc:
        raise ValueError("ACP terminal payload must contain finite JSON values.") from exc
    if size > MAX_PAYLOAD_BYTES:
        raise ValueError("ACP terminal payload exceeds the 32768-byte limit.")
    return encoded


class ACPNotifier:
    """A single-process durable outbox; action execution remains the caller's job.

    At most one delivery attempt per ID is in flight in this instance. A retry
    can repeat an HTTP POST after a lost ACK; it cannot create another terminal
    payload. Only Core's explicit, matching receipt marks delivery acknowledged.
    ``cached`` means Core accepted an early result for registration replay, not
    that the robot was successful or a collector notification was already shown.
    """

    def __init__(self, outbox_path, *, transport=None, clock=time.time,
                 delivery_window_s=60.0, retry_interval_s=1.0, max_attempts=12):
        self._window = _number(delivery_window_s, "ACP delivery window", 0.1, 86400)
        self._interval = _number(retry_interval_s, "ACP retry interval", 0.01, 3600)
        if isinstance(max_attempts, bool) or not isinstance(max_attempts, int) or not 1 <= max_attempts <= 1000:
            raise ValueError("ACP maximum attempts must be an integer from 1 to 1000.")
        self._max_attempts = max_attempts
        self._clock = clock
        self._transport = transport if transport is not None else HTTPSACPTransport()
        self._lock = threading.RLock()
        self._wake, self._stop = threading.Event(), threading.Event()
        self._thread = None
        self._closing, self._db_closed = False, False
        self._inflight = set()
        path = Path(outbox_path)
        if not path.parent.exists():
            path.parent.mkdir(mode=0o700, parents=True)
        if path.is_symlink() or (path.exists() and not path.is_file()):
            raise ValueError("ACP outbox must be a regular private file.")
        try:
            descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        except FileExistsError:
            pass
        else:
            os.close(descriptor)
        os.chmod(path, 0o600)
        self._db = sqlite3.connect(path, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        self._db.execute("PRAGMA synchronous=FULL")
        self._db.execute("""CREATE TABLE IF NOT EXISTS deliveries (
            action_id TEXT PRIMARY KEY, payload TEXT NOT NULL,
            delivery_state TEXT NOT NULL, created_at REAL NOT NULL,
            deadline_at REAL NOT NULL, next_attempt_at REAL NOT NULL,
            attempts INTEGER NOT NULL DEFAULT 0, max_attempts INTEGER NOT NULL,
            last_error_code TEXT, last_error TEXT, receipt_state TEXT,
            acknowledged_at REAL)""")
        self._db.commit()

    def _ensure_open(self):
        if self._closing or self._db_closed:
            raise RuntimeError("ACP notifier is closed.")

    def _expire(self, now):
        rows = self._db.execute("SELECT action_id,attempts,deadline_at FROM deliveries WHERE delivery_state='pending' AND (deadline_at<=? OR attempts>=max_attempts)", (now,)).fetchall()
        for row in rows:
            if row["action_id"] in self._inflight:
                continue
            state = "unacknowledged" if row["attempts"] else "not_delivered"
            code = "retry_deadline" if row["deadline_at"] <= now else "retry_attempt_limit"
            message = "ACP delivery deadline expired without an acknowledgement." if code == "retry_deadline" else "ACP delivery attempt limit was reached without an acknowledgement."
            self._db.execute("UPDATE deliveries SET delivery_state=?,last_error_code=?,last_error=? WHERE action_id=?", (state, code, message, row["action_id"]))
        self._db.commit()

    @staticmethod
    def _view(row):
        if row is None:
            return None
        payload = _decode_json(row["payload"])
        return {"action_id": row["action_id"], "terminal_status": payload["status"],
                "request_id": payload.get("request_id", payload["result"].get("request_id")),
                "delivery_state": row["delivery_state"], "pending": row["delivery_state"] == "pending",
                "acknowledged": row["delivery_state"] == "acknowledged", "outbox_retained": True,
                "attempts": row["attempts"], "max_attempts": row["max_attempts"],
                "created_at": row["created_at"], "deadline_at": row["deadline_at"],
                "next_attempt_at": row["next_attempt_at"] if row["delivery_state"] == "pending" else None,
                "receipt_state": row["receipt_state"], "acknowledged_at": row["acknowledged_at"],
                "last_error_code": row["last_error_code"], "last_error": row["last_error"]}

    def submit(self, payload):
        """Persist one immutable terminal result; never send on the caller thread."""
        encoded = _payload_json(payload)
        action_id = payload["action_id"]
        with self._lock:
            self._ensure_open()
            row = self._db.execute("SELECT * FROM deliveries WHERE action_id=?", (action_id,)).fetchone()
            if row is not None:
                if row["payload"] != encoded:
                    raise ValueError("ACP action_id already has a different terminal payload.")
                return self._view(row)
            now = self._clock()
            self._db.execute("INSERT INTO deliveries (action_id,payload,delivery_state,created_at,deadline_at,next_attempt_at,max_attempts) VALUES (?,?,'pending',?,?,?,?)",
                             (action_id, encoded, now, now + self._window, now, self._max_attempts))
            self._db.commit()
            row = self._db.execute("SELECT * FROM deliveries WHERE action_id=?", (action_id,)).fetchone()
            self._wake.set()
            return self._view(row)

    def status(self, action_id):
        """Inspect delivery without sending, refreshing, or renewing its deadline."""
        with self._lock:
            self._ensure_open()
            self._expire(self._clock())
            return self._view(self._db.execute("SELECT * FROM deliveries WHERE action_id=?", (action_id,)).fetchone())

    @staticmethod
    def _receipt(response, action_id):
        if not isinstance(response, ACPResponse) or isinstance(response.http_status, bool) or not isinstance(response.http_status, int) or not 200 <= response.http_status < 300:
            raise ACPDeliveryError("http_rejected", "ACP endpoint did not return a successful HTTP response.")
        body = response.body
        if (not isinstance(body, dict) or body.get("ok") is not True
                or body.get("action_id") != action_id or not isinstance(body.get("state"), str)
                or body["state"] not in ACK_STATES):
            raise ACPDeliveryError("invalid_ack", "ACP endpoint did not acknowledge this action ID.")
        return body["state"]

    def process_pending(self, limit=16):
        """Deliver due records once each, using an injected transport in tests."""
        if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= 1000:
            raise ValueError("ACP processing limit must be an integer from 1 to 1000.")
        processed = 0
        seen = set()
        for _ in range(limit):
            with self._lock:
                self._ensure_open()
                now = self._clock()
                self._expire(now)
                rows = self._db.execute("SELECT * FROM deliveries WHERE delivery_state='pending' AND next_attempt_at<=? ORDER BY created_at, action_id", (now,)).fetchall()
                row = next((item for item in rows if item["action_id"] not in self._inflight and item["action_id"] not in seen), None)
                if row is None:
                    break
                action_id = row["action_id"]
                seen.add(action_id)
                self._inflight.add(action_id)
                attempt = row["attempts"] + 1
                self._db.execute("UPDATE deliveries SET attempts=?,next_attempt_at=? WHERE action_id=?", (attempt, now + self._interval, action_id))
                self._db.commit()
                payload = _decode_json(row["payload"])
            receipt = None
            error = None
            try:
                receipt = self._receipt(self._transport(payload), action_id)
            except ACPDeliveryError as exc:
                error = (exc.code, str(exc))
            except Exception:
                error = ("transport_failed", "ACP delivery did not return a valid receipt.")
            with self._lock:
                now = self._clock()
                if receipt:
                    self._db.execute("UPDATE deliveries SET delivery_state='acknowledged',receipt_state=?,acknowledged_at=?,last_error_code=NULL,last_error=NULL WHERE action_id=?", (receipt, now, action_id))
                else:
                    exhausted = attempt >= row["max_attempts"] or now >= row["deadline_at"]
                    state = "unacknowledged" if exhausted else "pending"
                    self._db.execute("UPDATE deliveries SET delivery_state=?,last_error_code=?,last_error=? WHERE action_id=?", (state, error[0], error[1], action_id))
                self._db.commit()
                self._inflight.discard(action_id)
                processed += 1
                if self._closing:
                    break
        with self._lock:
            if self._closing and not self._inflight and not self._db_closed:
                self._db.close()
                self._db_closed = True
        return processed

    def start(self):
        """Explicitly start background delivery; repeated starts are harmless."""
        with self._lock:
            self._ensure_open()
            if self._thread is not None and self._thread.is_alive():
                return
            self._thread = threading.Thread(target=self._run, name="s10-acp-outbox", daemon=True)
            self._thread.start()

    def _run(self):
        try:
            while not self._stop.is_set():
                self._wake.clear()
                try:
                    self.process_pending()
                except RuntimeError:
                    if not self._closing:
                        raise
                self._wake.wait(0.25)
        finally:
            with self._lock:
                if self._closing and not self._inflight and not self._db_closed:
                    self._db.close()
                    self._db_closed = True

    def close(self, timeout_s=5.0):
        """Stop retries and close the outbox; never discard unacknowledged rows."""
        with self._lock:
            self._closing = True
            self._stop.set()
            self._wake.set()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=timeout_s)
        with self._lock:
            if not self._inflight and not self._db_closed:
                self._db.close()
                self._db_closed = True
            return self._db_closed
