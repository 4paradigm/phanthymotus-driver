"""Offline failure-injection tests; the Docker build exercises the real .so."""
import importlib.util
from pathlib import Path
import types
import unittest

PATH = Path(__file__).resolve().parents[1] / 'unitree/go1/check_sdk_binding.py'
spec = importlib.util.spec_from_file_location('go1_binding_check', PATH)
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


class BindingCheckTest(unittest.TestCase):
    def make_sdk(self):
        class UDPState:
            def __init__(self):
                for field in check.FIELDS:
                    setattr(self, field, 0)
        state = UDPState()
        calls = []
        class UDP:
            def __init__(self, *args):
                calls.append(args)
            @property
            def udpState(self):
                return state
            def Send(self):
                raise AssertionError('Must not send')
            def Recv(self):
                raise AssertionError('Must not receive')
        return types.SimpleNamespace(UDP=UDP, UDPState=UDPState), state, calls

    def test_reads_all_real_fields_without_io(self):
        sdk, _, calls = self.make_sdk()
        self.assertEqual(check.check_binding(sdk), dict.fromkeys(check.FIELDS, 0))
        self.assertEqual(calls, [(0xEE, 0, '127.0.0.1', 9)])

    def test_unregistered_property_fails_even_if_hasattr_passes(self):
        sdk, _, _ = self.make_sdk()
        def unregistered(_):
            raise TypeError('Unregistered type: UDPState')
        sdk.UDP.udpState = property(unregistered)
        self.assertTrue(hasattr(sdk.UDP, 'udpState'))
        with self.assertRaisesRegex(TypeError, 'Unregistered type'):
            check.check_binding(sdk)

    def test_missing_counter_fails(self):
        sdk, state, _ = self.make_sdk()
        del state.RecvCount
        with self.assertRaises(AttributeError):
            check.check_binding(sdk)

    def test_bad_counter_fails(self):
        for value in (1, '0', False):
            with self.subTest(value=value):
                sdk, state, _ = self.make_sdk()
                state.RecvCount = value
                with self.assertRaises(ValueError):
                    check.check_binding(sdk)

    def test_build_executes_check_without_network(self):
        dockerfile = (PATH.parent / 'Dockerfile').read_text()
        self.assertIn('COPY check_sdk_binding.py /work/check_sdk_binding.py', dockerfile)
        self.assertIn('RUN --network=none python3 /work/check_sdk_binding.py', dockerfile)
        self.assertNotIn("assert hasattr(r.UDP, 'udpState')", dockerfile)


if __name__ == '__main__':
    unittest.main()
