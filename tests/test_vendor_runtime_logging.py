"""Shared MCP request logging contracts, without ROS or hardware."""

from http.client import HTTPResponse
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import socket
import sys
import threading
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common.vendor_runtime import DriverBundle, make_handler


class VendorRuntimeLoggingTests(unittest.TestCase):
    def setUp(self):
        self.handler_type = make_handler(lambda: DriverBundle([]), "test", "test")
        self.handler = object.__new__(self.handler_type)
        self.handler.client_address = ("127.0.0.1", 12345)
        self.printed = self.enterContext(mock.patch("common.vendor_runtime.print", create=True))

    def test_control_characters_and_unicode_are_escaped_after_formatting(self):
        self.handler.log_message('"%s" %s %s', 'GET /\x1b[31m\r\nforged\t中文 HTTP/1.1', 404, "-")
        self.printed.assert_called_once_with(
            '[mcp] 127.0.0.1 "GET /\\x1b[31m\\r\\nforged\\t\\u4e2d\\u6587 HTTP/1.1" 404 -')

    def test_long_ascii_message_is_capped(self):
        self.handler.log_message("%s", "x" * 1000)
        self.printed.assert_called_once_with("[mcp] 127.0.0.1 " + "x" * 200)

    def test_length_cap_applies_after_escape_expansion(self):
        self.handler.log_message("%s", "\x1b" * 100)
        self.printed.assert_called_once_with("[mcp] 127.0.0.1 " + "\\x1b" * 50)

    def test_normal_requests_and_mcp_errors_remain_visible(self):
        self.handler.log_message('"%s" %s %s', "GET /health HTTP/1.1", 200, "-")
        self.handler.log_message('"%s" %s %s', "POST /mcp HTTP/1.1", 500, "-")
        self.assertEqual(self.printed.call_args_list, [
            mock.call('[mcp] 127.0.0.1 "GET /health HTTP/1.1" 200 -'),
            mock.call('[mcp] 127.0.0.1 "POST /mcp HTTP/1.1" 500 -'),
        ])

    def test_successful_mcp_requests_remain_quiet(self):
        self.handler.log_message('"%s" %s %s', "POST /mcp HTTP/1.1", 200, "-")
        self.printed.assert_not_called()

    def test_http_request_is_served_with_escaped_and_capped_log(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), self.handler_type)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        try:
            with socket.create_connection(server.server_address, timeout=2) as connection:
                connection.sendall(b"GET /\x1b[31m" + b"x" * 1000 + b" HTTP/1.1\r\nHost: localhost\r\n\r\n")
                with HTTPResponse(connection) as response:
                    response.begin()
                    self.assertEqual(response.status, 404)
                    self.assertEqual(json.loads(response.read()), {"error": "not found"})
        finally:
            server.shutdown()
            worker.join(timeout=2)
            server.server_close()
        prefix = '[mcp] 127.0.0.1 "GET /\\x1b[31m'
        self.printed.assert_called_once_with(prefix + "x" * (200 - len('"GET /\\x1b[31m')))


if __name__ == "__main__":
    unittest.main()
