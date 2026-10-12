"""Shared MCP access logs stay bounded and printable without ROS or hardware."""

from contextlib import redirect_stdout
import io
import json
import unittest
from unittest import mock

from common.vendor_runtime import DriverBundle, make_handler


class HTTPLoggingTests(unittest.TestCase):
    def setUp(self):
        self.handler_type = make_handler(lambda: DriverBundle([]), "test", "test")

    def request(self, raw):
        response, output = io.BytesIO(), io.StringIO()
        connection = mock.Mock()
        connection.makefile.return_value = io.BytesIO(raw)
        connection.sendall.side_effect = response.write
        with redirect_stdout(output):
            self.handler_type(connection, ("127.0.0.1", 12345), mock.Mock())
        return response.getvalue(), output.getvalue()

    def test_health_response_and_access_log_remain_readable(self):
        response, output = self.request(b"GET /health HTTP/1.0\r\n\r\n")
        self.assertIn(b"200 OK", response)
        self.assertEqual(json.loads(response.split(b"\r\n\r\n", 1)[1]), {
            "state": "running", "driver": "test",
        })
        self.assertEqual(output, '[mcp] 127.0.0.1 "GET /health HTTP/1.0" 200 -\n')

    def test_request_path_controls_are_escaped(self):
        response, output = self.request(b"GET /bad\x00\x1b[31m\x7f\x80 HTTP/1.0\r\n\r\n")
        self.assertIn(b"404 Not Found", response)
        for escaped in (r"\x00", r"\x1b", r"\x7f", r"\x80"):
            self.assertIn(escaped, output)
        self.assertEqual(len(output.splitlines()), 1)
        self.assertTrue(output.rstrip("\n").isascii())
        self.assertTrue(all(32 <= ord(char) < 127 for char in output.rstrip("\n")))

    def test_large_request_logs_are_capped_after_escaping(self):
        for byte in (b"a", b"\x1b", b"\x80"):
            with self.subTest(byte=byte):
                response, output = self.request(b"GET /" + byte * 8192 + b" HTTP/1.0\r\n\r\n")
                self.assertIn(b"404 Not Found", response)
                self.assertTrue(output.startswith("[mcp] 127.0.0.1 "))
                self.assertLessEqual(len(output.rstrip("\n")), len("[mcp] 127.0.0.1 ") + 200)
                self.assertEqual(len(output.splitlines()), 1)

    def test_embedded_line_breaks_cannot_create_log_records(self):
        handler = object.__new__(self.handler_type)
        handler.client_address = ("127.0.0.1", 12345)
        output = io.StringIO()
        with redirect_stdout(output):
            handler.log_message("%s", "request\r\nforged\tline")
        self.assertEqual(output.getvalue(), "[mcp] 127.0.0.1 request\\r\\nforged\\tline\n")

    def test_successful_mcp_calls_stay_quiet(self):
        body = b'{"jsonrpc":"2.0","id":1,"method":"initialize"}'
        request = b"POST /mcp HTTP/1.0\r\nContent-Length: %d\r\n\r\n" % len(body)
        response, output = self.request(request + body)
        self.assertIn(b"200 OK", response)
        self.assertEqual(output, "")

    def test_failed_mcp_calls_are_still_logged(self):
        response, output = self.request(b"POST /mcp HTTP/1.0\r\nContent-Length: 1\r\n\r\n{")
        self.assertIn(b"400 Bad Request", response)
        self.assertEqual(output, '[mcp] 127.0.0.1 "POST /mcp HTTP/1.0" 400 -\n')


if __name__ == "__main__":
    unittest.main()
