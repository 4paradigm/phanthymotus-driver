"""Show actionable operator rejection reasons without retrying a request."""
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch
import urllib.error

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import control


class OperatorClientTests(unittest.TestCase):
    def test_clear_rejection_displays_reason_and_is_not_retried(self):
        response = urllib.error.HTTPError('http://127.0.0.1:15791/operator/clear',
            409, 'Conflict', {}, io.BytesIO(
                b'{"error":"Hand Guiding/teach is not confirmed inactive"}'))
        opener = Mock()
        opener.open.side_effect = response
        with patch.object(control.urllib.request, 'build_opener', return_value=opener):
            with self.assertRaisesRegex(ValueError, 'HTTP 409: Hand Guiding/teach'):
                control.post('operator/clear', {'confirm_inspected': True})
        self.assertEqual(opener.open.call_count, 1)

    def test_non_json_rejection_does_not_echo_response_body(self):
        response = urllib.error.HTTPError('http://127.0.0.1:15791/mcp',
            503, 'Unavailable', {}, io.BytesIO(b'<html>private upstream error</html>'))
        opener = Mock()
        opener.open.side_effect = response
        with patch.object(control.urllib.request, 'build_opener', return_value=opener):
            with self.assertRaisesRegex(ValueError, 'HTTP 503: Service rejected') as caught:
                control.post('mcp', {})
        self.assertNotIn('private upstream', str(caught.exception))
        self.assertEqual(opener.open.call_count, 1)
