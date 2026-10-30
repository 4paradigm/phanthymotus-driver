"""Check logging protection before importing the driver runtime."""

import builtins
from pathlib import Path
import runpy
import unittest
from unittest import mock

from common import logsafe


class EntrypointTests(unittest.TestCase):
    def test_logsafe_is_installed_before_runtime_and_device_imports(self):
        original_import = builtins.__import__
        checked = []
        with mock.patch.object(logsafe, "install") as install:
            def guarded_import(name, *args, **kwargs):
                if name in ("common.vendor_runtime", "device"):
                    install.assert_called_once_with()
                    checked.append(name)
                return original_import(name, *args, **kwargs)

            with mock.patch.object(builtins, "__import__", side_effect=guarded_import):
                runpy.run_path(str(Path(__file__).with_name("main.py")), run_name="entrypoint_test")
        self.assertIn("common.vendor_runtime", checked)
        self.assertIn("device", checked)
