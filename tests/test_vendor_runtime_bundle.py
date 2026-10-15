"""Shared DriverBundle accepts ordered plugin collections and mappings."""

from __future__ import annotations

import sys
import unittest
from collections import OrderedDict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from common.vendor_runtime import DriverBundle  # noqa: E402


class DriverBundleCollectionTests(unittest.TestCase):
    def test_mapping_registers_plugin_values_not_string_keys(self):
        first, second = object(), object()
        bundle = DriverBundle(OrderedDict((('first', first), ('second', second))))
        self.assertEqual(bundle.plugins, [first, second])

    def test_list_registration_remains_unchanged(self):
        plugin = object()
        bundle = DriverBundle([plugin])
        self.assertEqual(bundle.plugins, [plugin])


if __name__ == "__main__":
    unittest.main()
