"""Request-line escaping/capping in the shared MCP HTTP log path.

`make_handler().log_message` is the single choke point where every driver's
MCP server writes client-visible request lines into host logs. The request line
is attacker-controlled: a hostile client can embed control characters (fake
log records, terminal escape sequences) or multi-megabyte garbage. The shared
handler must keep the logged copy ASCII-safe and capped so no consumer of the
raw log can be forged or bloated (cross-driver security/logging contract).

The rest of the handler (routing, JSON-RPC dispatch) is exercised per-driver;
this file pins only the shared logging behaviour, which no driver test covers.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.vendor_runtime import make_handler  # noqa: E402


def build_log_handler():
    handler_class = make_handler(lambda: None, "test_server", "test_driver")
    captured = []
    handler = handler_class.__new__(handler_class)

    def address_string(self=None):
        return "10.0.0.9"

    handler.address_string = address_string.__get__(handler, handler_class)
    handler.captured = captured

    def fake_print(message):
        captured.append(message)

    # The handler prints via the module-level print(); capture it instead of
    # letting the test run write into the runner's real stdout.
    import builtins
    import contextlib
    import io
    buffer = io.StringIO()
    with contextlib.redirect_stdout(buffer):
        handler.log_message('%s', 'raw message')
    captured.append(buffer.getvalue())
    return captured, builtins


class McpRequestLineLoggingTests(unittest.TestCase):
    def _log(self, message):
        """Return the exact line the shared handler prints for one request."""
        import contextlib
        import io
        from common import vendor_runtime

        handler_class = make_handler(lambda: None, "test_server", "test_driver")
        handler = handler_class.__new__(handler_class)
        handler.address_string = lambda: "10.0.0.9"
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            handler.log_message('%s', message)
        return buffer.getvalue()

    def test_success_post_mcp_line_is_suppressed(self):
        # 200 POST /mcp lines are the hot path (every tool call); logging them
        # all would double the log volume of a busy driver.
        line = self._log('"POST /mcp HTTP/1.1" 200 123')
        self.assertEqual(line, "")

    def test_control_characters_are_escaped(self):
        # A request line carrying ANSI/newline payloads must not be able to
        # forge additional log records or execute terminal escapes.
        line = self._log('"GET /x\x1b[31m HTTP/1.1" 404 -\nFAKE LOG LINE')
        self.assertNotIn("\x1b", line)
        self.assertNotIn("\nFAKE", line)
        self.assertIn("FAKE", line)  # content preserved, but inert
        self.assertIn("10.0.0.9", line)

    def test_non_ascii_request_line_becomes_ascii_safe(self):
        # Homoglyph/CJK noise in a hostile path must not reach the log raw —
        # every byte of the logged record stays printable ASCII.
        line = self._log('"POST /攻击 HTTP/1.1" 404 -').rstrip("\n")
        self.assertTrue(all(0x20 <= ord(ch) < 0x7f for ch in line),
                        f"non-ASCII leaked into log line: {line!r}")
        self.assertIn("\\u", line)  # escaped form is visible, not raw

    def test_oversized_request_line_is_capped(self):
        # A megabyte request line must not bloat host logs — capped at 200.
        line = self._log('"' + "A" * 5000 + ' HTTP/1.1" 404 -')
        body = line.strip()
        self.assertLessEqual(len(body), 200 + len("[mcp] 10.0.0.9 "))


if __name__ == "__main__":
    unittest.main()
