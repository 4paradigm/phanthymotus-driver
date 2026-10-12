"""Offline motion-switcher contract tests. No DDS or robot connection."""
import ast
import contextlib
import io
import json
from pathlib import Path
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1] / "unitree/g1"


def extract_class(path, class_name, namespace):
    """Load one class without importing hardware-facing driver modules."""
    module = ast.parse(path.read_text())
    node = next(node for node in module.body
                if isinstance(node, ast.ClassDef) and node.name == class_name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), namespace)
    return namespace[class_name]


class FakeClient:
    def __init__(self, name="ai", form="0"):
        self.name = name
        self.form = form
        self.check_code = 0
        self.select_code = 0
        self.release_code = 0
        self.select_error = None
        self.calls = []

    def CheckMode(self):
        self.calls.append("check")
        return self.check_code, {"form": self.form, "name": self.name}

    def SelectMode(self, mode):
        self.calls.append(("select", mode))
        if self.select_error:
            raise self.select_error
        if self.select_code == 0:
            self.name = mode
        return self.select_code, None

    def ReleaseMode(self):
        self.calls.append("release")
        if self.release_code == 0:
            self.name = ""
        return self.release_code, None


def card(client=None, verifier=None):
    namespace = {"threading": threading}
    cls = extract_class(ROOT / "device.py", "MotionSwitcherPlugin", namespace)
    client = client or FakeClient()
    return cls({}, "g1", None, client, verifier), client


class TestMotionSwitcher(unittest.TestCase):
    def test_check_returns_sdk_state_and_form(self):
        plugin, _ = card()
        result = plugin.dispatch("check", {})
        self.assertEqual((result["ret"], result["mode"], result["form"]), (0, "ai", "0"))
        self.assertIn("SDK", result["mode_meaning"])

    def test_bundled_sdk_check_decodes_actual_rpc_shape(self):
        class Base:
            pass
        namespace = {"Client": Base, "json": json,
                     "MOTION_SWITCHER_API_ID_CHECK_MODE": 1001}
        sdk_class = extract_class(
            ROOT / "unitree_sdk2py/comm/motion_switcher/motion_switcher_client.py",
            "MotionSwitcherClient", namespace)
        sdk = sdk_class.__new__(sdk_class)
        sdk._Call = lambda api_id, payload: (0, '{"form":"0","name":"ai"}')
        self.assertEqual(sdk.CheckMode(), (0, {"form": "0", "name": "ai"}))
        sdk._Call = lambda api_id, payload: (7006, None)
        self.assertEqual(sdk.CheckMode(), (7006, None))

    def test_check_error_and_malformed_data(self):
        plugin, sdk = card()
        sdk.check_code = 7006
        self.assertEqual(plugin.dispatch("check", {})["ret"], 7006)
        sdk.check_code = 0
        sdk.CheckMode = lambda: (0, {"form": "0"})
        self.assertIn("error", plugin.dispatch("check", {}))

    def test_check_exception_is_returned(self):
        plugin, sdk = card()
        sdk.CheckMode = lambda: (_ for _ in ()).throw(TimeoutError("RPC timed out"))
        self.assertIn("TimeoutError", plugin.dispatch("check", {})["error"])

    def test_select_valid_and_already_selected(self):
        plugin, sdk = card()
        self.assertEqual(plugin.dispatch("select", {"mode": "normal"})["status"], "confirmed")
        self.assertEqual(plugin.dispatch("select", {"mode": "normal"})["status"], "already_selected")
        self.assertEqual(sdk.calls.count(("select", "normal")), 1)

    def test_select_rejects_missing_and_invalid_mode(self):
        plugin, sdk = card()
        for args in ({}, {"mode": "wrong"}, {"mode": None}):
            self.assertIn("error", plugin.dispatch("select", args))
        self.assertEqual(sdk.calls, [])

    def test_select_rejection_and_timeout(self):
        plugin, sdk = card()
        sdk.select_code = 7002
        self.assertEqual(plugin.dispatch("select", {"mode": "normal"})["ret"], 7002)
        sdk.select_error = TimeoutError("RPC timed out")
        self.assertIn("TimeoutError", plugin.dispatch("select", {"mode": "normal"})["error"])

    def test_select_7004_retained_with_device_specific_explanation(self):
        plugin, sdk = card()
        sdk.select_code = 7004
        result = plugin.dispatch("select", {"mode": "advanced"})
        self.assertEqual(result["ret"], 7004)
        self.assertEqual(result["status"], "error")
        self.assertEqual(result["previous_mode"], "ai")
        self.assertIn("可能不受支持", result["error"])

    def test_select_does_not_claim_completion_without_feedback(self):
        plugin, sdk = card()
        sdk.SelectMode = lambda mode: (0, None)
        result = plugin.dispatch("select", {"mode": "normal"})
        self.assertEqual(result["status"], "unconfirmed")
        self.assertEqual(result["mode"], "ai")

    def test_release_blocked_by_default_even_with_user_flag(self):
        plugin, sdk = card()
        result = plugin.dispatch("release", {"confirm": True, "controller_ready": True})
        self.assertEqual(result["status"], "blocked")
        self.assertFalse(result["released"])
        self.assertEqual(sdk.calls, [])

    def test_release_verifier_failure_never_calls_sdk(self):
        plugin, sdk = card(verifier=lambda: {"ready": False, "controller": "test"})
        self.assertEqual(plugin.dispatch("release", {"confirm": True})["status"], "blocked")
        self.assertEqual(sdk.calls, [])

    def test_release_is_marked_dangerous_and_requires_explicit_confirmation(self):
        plugin, sdk = card(verifier=lambda: {"ready": True, "controller": "offline-test"})
        schema = plugin.get_tool()["inputSchema"]
        self.assertIs(schema["x-is-dangerous"], True)
        self.assertEqual(schema["x-action-params"]["release"]["params"], ["confirm"])
        self.assertIs(schema["properties"]["confirm"]["const"], True)
        for args in ({}, {"confirm": False}, {"confirm": "true"}, {"confirm": 1}):
            result = plugin.dispatch("release", args)
            self.assertEqual(result["status"], "blocked")
            self.assertFalse(result["released"])
        self.assertEqual(sdk.calls, [])

    def test_release_sdk_failure_and_confirmed_success_with_internal_verifier(self):
        verifier = lambda: {"ready": True, "controller": "offline-test"}
        plugin, sdk = card(verifier=verifier)
        sdk.release_code = 7008
        with contextlib.redirect_stdout(io.StringIO()):
            failed = plugin.dispatch("release", {"confirm": True})
        self.assertEqual((failed["ret"], failed["released"]), (7008, None))
        sdk.release_code = 0
        with contextlib.redirect_stdout(io.StringIO()):
            done = plugin.dispatch("release", {"confirm": True})
        self.assertEqual((done["status"], done["released"]), ("mode_cleared", True))

    def test_release_after_rpc_timeout_does_not_claim_state(self):
        plugin, sdk = card(verifier=lambda: {"ready": True, "controller": "offline-test"})
        sdk.ReleaseMode = lambda: (_ for _ in ()).throw(TimeoutError("RPC timed out"))
        with contextlib.redirect_stdout(io.StringIO()):
            result = plugin.dispatch("release", {"confirm": True})
        self.assertEqual(result["status"], "error")
        self.assertIsNone(result["released"])

    def test_release_accepted_but_not_confirmed(self):
        plugin, sdk = card(verifier=lambda: {"ready": True, "controller": "offline-test"})
        sdk.ReleaseMode = lambda: (0, None)
        with contextlib.redirect_stdout(io.StringIO()):
            result = plugin.dispatch("release", {"confirm": True})
        self.assertEqual(result["status"], "unconfirmed")
        self.assertIsNone(result["released"])

    def test_silent_legacy_calls_return_unsupported_and_are_not_advertised(self):
        plugin, sdk = card()
        actions = plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
        self.assertEqual(actions, ["check", "select", "release"])
        for action in ("set_silent", "get_silent"):
            self.assertEqual(plugin.dispatch(action, {"silent": True})["status"], "unsupported")
        self.assertEqual(sdk.calls, [])


if __name__ == "__main__":
    unittest.main()
