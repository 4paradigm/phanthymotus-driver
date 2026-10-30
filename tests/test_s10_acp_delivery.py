"""Isolated ACP outbox and TLS tests; no robot connection or control output."""
import copy
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import shutil
import ssl
import subprocess
import threading
import time

import pytest

from deep_robotics.s10_pro.acp import (
    ACPDeliveryError, ACPNotifier, ACPResponse, HTTPSACPTransport,
)


class Clock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now

    def advance(self, seconds=1.0):
        self.now += seconds


def terminal(action_id="s10-action-1", status="completed"):
    return {"action_id": action_id, "status": status, "tool": "motion", "ts": 1000.0,
            "result": {"action_id": action_id, "request_id": "req-1", "stop_confirmed": True}}


def acknowledged(payload, state="matched"):
    return ACPResponse(200, {"ok": True, "action_id": payload["action_id"], "state": state})


@pytest.fixture
def rig(tmp_path):
    clock, posts = Clock(), []

    def transport(payload):
        posts.append(copy.deepcopy(payload))
        return acknowledged(payload)

    notifier = ACPNotifier(tmp_path / "private" / "acp.sqlite3", transport=transport, clock=clock)
    yield notifier, clock, posts
    notifier.close()


def test_construction_submit_and_queries_never_send(rig):
    notifier, clock, posts = rig
    original = terminal()
    record = notifier.submit(original)
    original["result"]["stop_confirmed"] = False
    assert record["delivery_state"] == "pending"
    assert record["attempts"] == 0
    assert record["request_id"] == "req-1"
    assert notifier.status("missing") is None
    for _ in range(20):
        assert notifier.status("s10-action-1")["pending"]
    assert posts == []
    notifier.process_pending()
    assert posts[0]["result"]["stop_confirmed"] is True


@pytest.mark.parametrize("receipt_state", ["matched", "cached", "duplicate"])
def test_each_explicit_core_receipt_is_acknowledged(tmp_path, receipt_state):
    notifier = ACPNotifier(tmp_path / "outbox", transport=lambda payload: acknowledged(payload, receipt_state))
    try:
        notifier.submit(terminal())
        assert notifier.process_pending() == 1
        record = notifier.status("s10-action-1")
        assert record["acknowledged"] is True
        assert record["pending"] is False
        assert record["receipt_state"] == receipt_state
        assert record["outbox_retained"] is True
        assert record["attempts"] == 1
        assert record["next_attempt_at"] is None
    finally:
        notifier.close()


@pytest.mark.parametrize("response", [
    ACPResponse(200, None), ACPResponse(200, {"ok": True}),
    ACPResponse(200, {"ok": True, "action_id": "wrong", "state": "matched"}),
    ACPResponse(200, {"ok": 1, "action_id": "s10-action-1", "state": "matched"}),
    ACPResponse(200, {"ok": False, "action_id": "s10-action-1", "state": "matched"}),
    ACPResponse(200, {"ok": True, "action_id": "s10-action-1", "state": "rejected"}),
    ACPResponse(200, {"ok": True, "action_id": "s10-action-1", "state": "pending"}),
    ACPResponse(500, {"ok": True, "action_id": "s10-action-1", "state": "matched"}),
    ACPResponse(401, {}), ACPResponse(True, {}),
    {"ok": True, "action_id": "s10-action-1", "state": "matched"},
])
def test_http_success_alone_or_wrong_ack_never_means_delivered(tmp_path, response):
    notifier = ACPNotifier(tmp_path / "outbox", transport=lambda payload: response)
    try:
        notifier.submit(terminal())
        notifier.process_pending()
        record = notifier.status("s10-action-1")
        assert record["acknowledged"] is False
        assert record["pending"] is True
        assert record["last_error_code"] in {"invalid_ack", "http_rejected"}
    finally:
        notifier.close()


def test_lost_ack_retries_identical_payload_and_core_deduplicates(tmp_path):
    clock, posts, core_received, collector = Clock(), [], set(), []

    def transport(payload):
        posts.append(copy.deepcopy(payload))
        action_id = payload["action_id"]
        if action_id not in core_received:
            core_received.add(action_id)
            collector.append(action_id)
            raise TimeoutError("ACK lost after Core accepted it")
        return acknowledged(payload, "duplicate")

    notifier = ACPNotifier(tmp_path / "outbox", transport=transport, clock=clock)
    try:
        notifier.submit(terminal())
        notifier.process_pending()
        assert notifier.status("s10-action-1")["pending"]
        assert notifier.process_pending() == 0
        clock.advance()
        notifier.process_pending()
        assert notifier.status("s10-action-1")["acknowledged"]
        assert posts == [terminal(), terminal()]
        assert collector == ["s10-action-1"]
        for _ in range(5):
            notifier.submit(terminal())
            notifier.status("s10-action-1")
            notifier.process_pending()
        assert len(posts) == 2
    finally:
        notifier.close()


def test_early_callback_retained_for_core_registration_replay(tmp_path):
    buffered, notifications = {}, []

    def transport(payload):
        buffered.setdefault(payload["action_id"], copy.deepcopy(payload))
        return acknowledged(payload, "cached")

    notifier = ACPNotifier(tmp_path / "outbox", transport=transport)
    try:
        notifier.submit(terminal())
        notifier.process_pending()
        assert notifier.status("s10-action-1")["receipt_state"] == "cached"
        assert notifications == []
        # Only the actual Core registration replays a cached result. The
        # notifier must not invent an extra manual trigger for this case.
        notifications.append(buffered.pop("s10-action-1"))
        notifier.submit(terminal())
        assert notifier.process_pending() == 0
        assert len(notifications) == 1
    finally:
        notifier.close()


def test_duplicate_submit_is_immutable_and_does_not_reset_deadline(rig):
    notifier, clock, posts = rig
    first = notifier.submit(terminal())
    clock.advance(5)
    second = notifier.submit(terminal())
    assert first == second
    changed = terminal(status="cancelled")
    with pytest.raises(ValueError, match="different terminal payload"):
        notifier.submit(changed)
    assert notifier.status("s10-action-1")["terminal_status"] == "completed"
    assert not posts


def test_pending_and_acknowledged_outbox_survive_restart(tmp_path):
    path, clock, posts = tmp_path / "outbox", Clock(), []
    first = ACPNotifier(path, transport=lambda payload: posts.append(payload), clock=clock)
    first.submit(terminal("pending"))
    first.submit(terminal("done"))
    first._transport = lambda payload: acknowledged(payload) if payload["action_id"] == "done" else (_ for _ in ()).throw(TimeoutError())
    first.process_pending()
    pending = first.status("pending")
    first.close()

    def transport(payload):
        posts.append(payload)
        return acknowledged(payload)

    second = ACPNotifier(path, transport=transport, clock=clock)
    try:
        assert second.status("pending")["deadline_at"] == pending["deadline_at"]
        assert second.status("done")["acknowledged"]
        clock.advance()
        second.process_pending()
        assert [p["action_id"] for p in posts] == ["pending"]
        assert second.status("pending")["acknowledged"]
        assert os.stat(path).st_mode & 0o777 == 0o600
    finally:
        second.close()


def test_expired_unattempted_record_is_retained_as_not_delivered(rig):
    notifier, clock, posts = rig
    notifier.submit(terminal())
    clock.advance(60)
    record = notifier.status("s10-action-1")
    assert record["delivery_state"] == "not_delivered"
    assert record["last_error_code"] == "retry_deadline"
    assert record["outbox_retained"]
    assert notifier.process_pending() == 0
    notifier.submit(terminal())
    assert notifier.status("s10-action-1")["delivery_state"] == "not_delivered"
    assert posts == []


def test_attempt_exhaustion_is_unacknowledged_without_changing_action_result(tmp_path):
    clock, posts = Clock(), []

    def transport(payload):
        posts.append(payload)
        raise RuntimeError("SECRET: must not be logged or retained")

    notifier = ACPNotifier(tmp_path / "outbox", transport=transport, clock=clock, max_attempts=2)
    try:
        notifier.submit(terminal(status="error"))
        notifier.process_pending()
        clock.advance()
        notifier.process_pending()
        record = notifier.status("s10-action-1")
        assert record["delivery_state"] == "unacknowledged"
        assert record["terminal_status"] == "error"
        assert record["attempts"] == 2
        assert "SECRET" not in json.dumps(record)
        clock.advance(100)
        assert notifier.process_pending() == 0
        assert len(posts) == 2
    finally:
        notifier.close()


def test_deadline_exhaustion_after_attempt_is_unacknowledged(tmp_path):
    clock = Clock()
    notifier = ACPNotifier(tmp_path / "outbox", clock=clock, delivery_window_s=.1,
                           transport=lambda payload: ACPResponse(200, {"ok": False}))
    try:
        notifier.submit(terminal(status="cancelled"))
        notifier.process_pending()
        clock.advance(.1)
        assert notifier.status("s10-action-1")["delivery_state"] == "unacknowledged"
        assert notifier.status("s10-action-1")["terminal_status"] == "cancelled"
        assert notifier.process_pending() == 0
    finally:
        notifier.close()


def test_restart_does_not_exceed_durably_reserved_attempt_limit(tmp_path):
    path, clock, posts = tmp_path / "outbox", Clock(), []
    first = ACPNotifier(path, clock=clock, max_attempts=1, transport=lambda payload: None)
    first.submit(terminal())
    # A process may die after reserving its final attempt and before persisting
    # the response. Recovery must keep that uncertainty rather than re-moving
    # a result's delivery window or silently permitting an unlimited retry.
    first._db.execute("UPDATE deliveries SET attempts=1")
    first._db.commit()
    first.close()
    second = ACPNotifier(path, clock=clock, transport=lambda payload: posts.append(payload))
    try:
        assert second.process_pending() == 0
        record = second.status("s10-action-1")
        assert record["delivery_state"] == "unacknowledged"
        assert record["last_error_code"] == "retry_attempt_limit"
        assert record["max_attempts"] == 1
        assert posts == []
    finally:
        second.close()


def test_inflight_duplicate_and_parallel_poll_do_not_double_post(tmp_path):
    entered, release, posts = threading.Event(), threading.Event(), []

    def transport(payload):
        posts.append(copy.deepcopy(payload))
        entered.set()
        assert release.wait(2)
        return acknowledged(payload)

    notifier = ACPNotifier(tmp_path / "outbox", transport=transport)
    notifier.submit(terminal())
    worker = threading.Thread(target=notifier.process_pending)
    worker.start()
    try:
        assert entered.wait(2)
        notifier.submit(terminal())
        assert notifier.process_pending() == 0
        assert notifier.status("s10-action-1")["attempts"] == 1
        assert len(posts) == 1
    finally:
        release.set()
        worker.join(2)
        notifier.close()


def test_background_start_is_explicit_and_idempotent(tmp_path):
    delivered = threading.Event()

    def transport(payload):
        delivered.set()
        return acknowledged(payload)

    notifier = ACPNotifier(tmp_path / "outbox", transport=transport)
    try:
        notifier.submit(terminal())
        assert not delivered.wait(.03)
        notifier.start()
        notifier.start()
        assert delivered.wait(2)
        for _ in range(100):
            if notifier.status("s10-action-1")["acknowledged"]:
                break
            time.sleep(.005)
        assert notifier.status("s10-action-1")["acknowledged"]
    finally:
        assert notifier.close()
    with pytest.raises(RuntimeError, match="closed"):
        notifier.submit(terminal("new"))


@pytest.mark.parametrize("change", [
    {"action_id": ""}, {"action_id": "a" * 257}, {"action_id": 1}, {"action_id": "a\n"},
    {"status": "running"}, {"status": "accepted"}, {"status": "failed"}, {"status": []},
    {"result": "done"}, {"result": {"value": float("nan")}}, {"result": {"value": float("inf")}},
    {"unknown": 1}, {"ts": True}, {"ts": float("inf")}, {"request_id": ""},
    {"tool": None}, {"result": {"large": "x" * 33000}},
    {"result": {1: "ambiguous", "1": "key"}}, {"result": {"value": (1, 2)}},
    {"result": {"value": "\ud800"}},
])
def test_invalid_terminal_never_persists_or_sends(rig, change):
    notifier, clock, posts = rig
    payload = terminal()
    payload.update(change)
    with pytest.raises(ValueError):
        notifier.submit(payload)
    assert notifier.process_pending() == 0
    assert not posts


@pytest.mark.parametrize("url", ["http://localhost:15678", "http://192.0.2.1", "file:///tmp/no", "https://user:secret@localhost", "https://localhost/?token=secret", "https://localhost/#bad"])
def test_https_transport_rejects_plaintext_and_url_credentials(url):
    with pytest.raises(ValueError, match="HTTPS"):
        HTTPSACPTransport(url)


@pytest.mark.parametrize("argument,value", [("max_attempts", True), ("max_attempts", 1.5),
    ("max_attempts", 0), ("delivery_window_s", float("inf")), ("retry_interval_s", 0)])
def test_invalid_delivery_policy_rejected_before_use(tmp_path, argument, value):
    with pytest.raises(ValueError):
        ACPNotifier(tmp_path / "outbox", transport=lambda payload: None, **{argument: value})


@pytest.fixture
def tls_server(tmp_path):
    if shutil.which("openssl") is None:
        pytest.skip("OpenSSL is required for the isolated TLS fixture")
    cert, key = tmp_path / "cert.pem", tmp_path / "key.pem"
    subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "1",
                    "-keyout", str(key), "-out", str(cert), "-subj", "/CN=localhost"],
                   check=True, capture_output=True)
    posts = []

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_POST(self):
            payload = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            posts.append({"payload": payload, "path": self.path, "authorization": self.headers.get("Authorization")})
            data = json.dumps({"ok": True, "action_id": payload["action_id"], "state": "matched"}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(cert, key)
    server.socket = context.wrap_socket(server.socket, server_side=True)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield server.server_port, cert, posts
    server.shutdown()
    server.server_close()
    thread.join(2)


def test_normal_ca_and_hostname_validation_with_bearer(tls_server):
    port, cert, posts = tls_server
    transport = HTTPSACPTransport(f"https://localhost:{port}", ca_file=cert, token_provider=lambda: "test-secret")
    assert transport._context.verify_mode == ssl.CERT_REQUIRED
    assert transport._context.check_hostname is True
    response = transport(terminal())
    assert response.body["ok"] is True
    assert posts == [{"path": "/api/acp/complete", "payload": terminal(), "authorization": "Bearer test-secret"}]


def test_unknown_certificate_and_wrong_hostname_never_send_body(tls_server):
    port, cert, posts = tls_server
    for transport in [HTTPSACPTransport(f"https://localhost:{port}"),
                      HTTPSACPTransport(f"https://127.0.0.1:{port}", ca_file=cert)]:
        with pytest.raises(ACPDeliveryError, match="TLS"):
            transport(terminal())
    assert posts == []


def test_explicit_leaf_pin_is_checked_before_credentials_and_body(tls_server):
    port, cert, posts = tls_server
    credentials = []
    transport = HTTPSACPTransport(f"https://127.0.0.1:{port}", pinned_leaf_file=cert,
                                  token_provider=lambda: credentials.append(True) or "test-secret")
    assert transport._context.verify_mode == ssl.CERT_REQUIRED
    assert transport(terminal()).body["ok"] is True
    assert len(posts) == len(credentials) == 1
    posts.clear()
    credentials.clear()
    transport._leaf_digest = hashlib.sha256(b"different trusted leaf").digest()
    with pytest.raises(ACPDeliveryError) as error:
        transport(terminal())
    assert error.value.code == "certificate_pin_mismatch"
    assert posts == credentials == []


def test_token_can_be_read_from_environment_or_mounted_env(tls_server, tmp_path, monkeypatch):
    port, cert, posts = tls_server
    env = tmp_path / "private.env"
    env.write_text("# private fixture\nACCESS_TOKEN=file-token\n")
    monkeypatch.setenv("S10_ACP_ENV_FILE", str(env))
    monkeypatch.delenv("ACCESS_TOKEN", raising=False)
    transport = HTTPSACPTransport(f"https://localhost:{port}", ca_file=cert)
    transport(terminal())
    monkeypatch.setenv("ACCESS_TOKEN", "env-token")
    transport(terminal("other"))
    assert [post["authorization"] for post in posts] == ["Bearer file-token", "Bearer env-token"]


def test_tampered_or_unconfigured_pin_not_automatically_trusted(tls_server, tmp_path):
    port, cert, posts = tls_server
    invalid = tmp_path / "invalid.pem"
    invalid.write_text("not a certificate")
    with pytest.raises(ValueError, match="exactly one"):
        HTTPSACPTransport(f"https://localhost:{port}", pinned_leaf_file=invalid)
    with pytest.raises(ValueError, match="either"):
        HTTPSACPTransport(f"https://localhost:{port}", pinned_leaf_file=cert, ca_file=cert)
    assert posts == []


def test_no_console_output_or_secrets_in_delivery_record(tmp_path, capsys):
    notifier = ACPNotifier(tmp_path / "outbox", transport=lambda payload: (_ for _ in ()).throw(RuntimeError("private-token")))
    try:
        notifier.submit(terminal())
        notifier.process_pending()
        assert "private-token" not in json.dumps(notifier.status("s10-action-1"))
        assert capsys.readouterr().out == ""
        assert capsys.readouterr().err == ""
    finally:
        notifier.close()


def test_official_legacy_core_receipt_stops_repeated_agent_wakeups(tmp_path):
    clock, posts = Clock(), []
    def legacy_core(payload):
        posts.append(copy.deepcopy(payload))
        return ACPResponse(200, {'ok': True, 'action_id': payload['action_id']})
    notifier = ACPNotifier(tmp_path / 'legacy-core-outbox', transport=legacy_core, clock=clock)
    try:
        notifier.submit(terminal())
        assert notifier.process_pending() == 1
        record = notifier.status('s10-action-1')
        assert record['acknowledged']
        assert record['receipt_state'] == 'endpoint_accepted'
        # Do not claim the old Core retained an early callback or deduplicated it.
        assert record['receipt_state'] not in ('matched', 'cached', 'duplicate')
        for _ in range(15):
            clock.advance()
            notifier.submit(terminal())
            assert notifier.process_pending() == 0
        assert len(posts) == 1
    finally:
        notifier.close()


@pytest.mark.parametrize('body', [
    {'ok': True, 'action_id': 's10-action-1', 'state': None},
    {'ok': True, 'action_id': 's10-action-1', 'error': 'rejected'},
    {'ok': True, 'action_id': 'wrong'},
    {'ok': 1, 'action_id': 's10-action-1'},
])
def test_legacy_receipt_compatibility_never_accepts_ambiguous_response(tmp_path, body):
    notifier = ACPNotifier(tmp_path / 'invalid-core-outbox',
                           transport=lambda payload: ACPResponse(200, body))
    try:
        notifier.submit(terminal())
        notifier.process_pending()
        assert not notifier.status('s10-action-1')['acknowledged']
        assert notifier.status('s10-action-1')['last_error_code'] == 'invalid_ack'
    finally:
        notifier.close()
