"""HTTP request logging must not pass remote control bytes to Docker logs."""

from unittest.mock import patch

from common.vendor_runtime import make_handler


def _handler():
    handler_type = make_handler(lambda: None, "test", "test-driver")
    handler = handler_type.__new__(handler_type)
    handler.client_address = ("198.51.100.9", 12345)
    return handler


def test_request_log_escapes_control_bytes_and_caps_remote_text():
    handler = _handler()
    with patch("builtins.print") as output:
        handler.log_message('%s', 'GET /bad\n\x1b[31m' + 'x' * 500)

    line = output.call_args.args[0]
    assert line.startswith('[mcp] 198.51.100.9 GET /bad\\n\\x1b[31m')
    assert '\n' not in line and '\x1b' not in line
    assert len(line.removeprefix('[mcp] 198.51.100.9 ')) == 200


def test_successful_mcp_post_remains_suppressed():
    handler = _handler()
    with patch("builtins.print") as output:
        handler.log_message('%s', '"POST /mcp HTTP/1.1" 200 -')
    output.assert_not_called()
