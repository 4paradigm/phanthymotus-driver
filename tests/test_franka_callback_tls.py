"""Exercise real legacy HTTPS callbacks without Core or a robot."""
import json
import shutil
import ssl
import subprocess
import tempfile
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from unittest import mock

import pytest
from test_franka_joint_control import device


def test_legacy_core_tls_for_acp_and_remote_verification():
    if not shutil.which("openssl"):
        pytest.skip("openssl is required to generate the legacy Core certificate")
    received = []

    class Handler(BaseHTTPRequestHandler):
        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            received.append((self.path, body))
            reply = {"ok": True, "action_id": body["action_id"]} if self.path.endswith("complete") else {"code": 200}
            self.send_response(200)
            self.end_headers()
            self.wfile.write(json.dumps(reply).encode())

        def log_message(self, *args):
            pass

    with tempfile.TemporaryDirectory() as temporary:
        cert, key = (Path(temporary) / name for name in ("cert.pem", "key.pem"))
        subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
                        "-keyout", str(key), "-out", str(cert), "-days", "1",
                        "-subj", "/CN=phanthy-motus"], check=True, capture_output=True)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(cert, key)
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        server.socket = context.wrap_socket(server.socket, server_side=True)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            url = f"https://127.0.0.1:{server.server_port}"
            with mock.patch.dict("os.environ", {"AGENT_CORE_URL": url,
                    "AGENT_CORE_CA_CERT": str(cert)}):
                # Demonstrate the old hostname check fails against this cert.
                strict = ssl.create_default_context(cafile=str(cert))
                import urllib.request
                with pytest.raises(urllib.error.URLError, match="CERTIFICATE_VERIFY_FAILED"):
                    urllib.request.build_opener(urllib.request.ProxyHandler({}),
                        urllib.request.HTTPSHandler(context=strict)).open(url, timeout=2)
                plugin = object.__new__(device.JointControlPlugin)
                plugin._post_completion("tls-test", "completed", {"measured": True})
                for host in ("localhost", "127.0.0.1", "[::1]"):
                    local = device._core_tls_context(f"https://{host}:15678")
                    assert not local.check_hostname and local.verify_mode == ssl.CERT_NONE
                for host in ("core.example", "localhost.example", "172.16.0.2"):
                    remote = device._core_tls_context(f"https://{host}:15678")
                    assert remote.check_hostname and remote.verify_mode == ssl.CERT_REQUIRED
            assert [path for path, _ in received] == ["/api/acp/complete"]
            assert received[0][1]["status"] == "completed"
        finally:
            server.shutdown()
            server.server_close()
            worker.join(2)
