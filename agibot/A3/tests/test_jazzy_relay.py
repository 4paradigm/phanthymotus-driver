"""Host-side relay preflight tests; no ROS installation or robot required."""

from __future__ import annotations

import contextlib
import io
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from agibot.A3 import jazzy_relay


class JazzyRelayPreflightTests(unittest.TestCase):
    def test_image_dependencies_are_imported_and_reported(self):
        calls = []
        cv2 = SimpleNamespace(
            __version__="4.test", setNumThreads=lambda count: calls.append(count))
        numpy = SimpleNamespace(__version__="2.test")
        output = io.StringIO()
        with patch.dict(sys.modules, {"cv2": cv2, "numpy": numpy}), \
                contextlib.redirect_stdout(output):
            jazzy_relay._check_image_dependencies()
        self.assertEqual(calls, [1])
        self.assertIn("numpy=2.test opencv=4.test", output.getvalue())

    def test_missing_host_image_dependency_fails_with_actionable_message(self):
        real_import = __import__

        def without_image_packages(name, *args, **kwargs):
            if name in {"cv2", "numpy"}:
                raise ImportError(f"No module named {name}")
            return real_import(name, *args, **kwargs)

        with patch("builtins.__import__", side_effect=without_image_packages):
            with self.assertRaisesRegex(RuntimeError, "container's pip packages"):
                jazzy_relay._check_image_dependencies()


if __name__ == "__main__":
    unittest.main()
