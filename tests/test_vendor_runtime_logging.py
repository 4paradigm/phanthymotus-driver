"""Request log regressions shared by every vendor_runtime HTTP handler."""
from unittest.mock import patch

from common.vendor_runtime import make_handler


def request_logger():
    # No socket or vendor plugin: exercise the actual shared log formatter.
    handler = object.__new__(make_handler(lambda: None, "test", "test"))
    handler.client_address = ("127.0.0.1", 12345)
    return handler


def test_request_controls_cannot_inject_records_or_terminal_commands():
    with patch("builtins.print") as output:
        request_logger().log_message('%s', 'GET /evil\nforged\r\x00\x1b[31m\x7f 404')
    message = output.call_args.args[0]
    assert message.startswith('[mcp] 127.0.0.1 ')
    assert not any(ord(char) < 32 or ord(char) == 127 for char in message)
    assert r'\nforged\r\x00\x1b[31m\x7f' in message


def test_escaped_request_payload_is_capped_at_200_characters():
    with patch("builtins.print") as output:
        request_logger().log_message('GET /%s 404', '\x1b' * 1000)
    message = output.call_args.args[0]
    assert len(message.removeprefix('[mcp] 127.0.0.1 ')) == 200
    assert '\x1b' not in message


def test_successful_mcp_requests_stay_quiet_and_errors_are_logged():
    handler = request_logger()
    with patch("builtins.print") as output:
        handler.log_message('"POST /mcp HTTP/1.1" %s', '200')
        output.assert_not_called()
        handler.log_message('"POST /mcp HTTP/1.1" %s', '500')
        output.assert_called_once()
        assert '500' in output.call_args.args[0]
