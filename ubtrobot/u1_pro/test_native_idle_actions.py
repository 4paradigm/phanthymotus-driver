"""Test the native idle switch without Docker, ROSA, or robot side effects."""

import io
import json
import subprocess
import unittest
from contextlib import redirect_stdout
from unittest import mock

from native_idle_actions import IdleActionControl, _READ_CONFIG


def completed(output="", code=0, error=""):
    return subprocess.CompletedProcess([], code, stdout=output, stderr=error)


ACCEPTED = (
    "waiting for service to become available...\n"
    "response: config_msgs.srv.SetConfig.Response(data=True, message=Config updated successfully)\n"
)


class IdleActionControlTests(unittest.TestCase):
    def test_enable_and_disable_only_write_idle_preference_and_verify_readback(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled), mock.patch(
                "native_idle_actions.subprocess.run",
                side_effect=[completed(ACCEPTED), completed(json.dumps({"enabled": enabled}))],
            ) as run:
                result = IdleActionControl("test-behavior").set_enabled(enabled)
                self.assertIs(result["enabled"], enabled)
                self.assertEqual(result["state"]["source"], "vendor_user_config")
                self.assertEqual(run.call_count, 2)
                command = run.call_args_list[0].args[0]
                self.assertEqual(command[:11], ["nsenter", "-t", "1", "-m", "-u", "-i", "-n", "-p", "--wd=/", "--", "/usr/bin/docker"])
                self.assertEqual(command[11:15], ["exec", "test-behavior", "bash", "-c"])
                self.assertIn("--no-daemon /sys/config/user_manager/set_config", command[15])
                self.assertIn("timeout --signal=TERM --kill-after=2s 18s", command[15])
                self.assertIn('"$1"', command[15])
                self.assertEqual(json.loads(command[-1]), {
                    "config": [{"name": "idle_action_enabled", "value": {"type": 1, "bool_value": enabled}}],
                })
                self.assertNotIn("shell", run.call_args_list[0].kwargs)
                self.assertEqual(run.call_args_list[0].kwargs["timeout"], 25)

    def test_business_rejection_and_request_echo_are_not_success(self):
        for output in (
            "response: config_msgs.srv.SetConfig.Response(data=False, message=Rejected)\n",
            "requester: config_msgs.srv.SetConfig.Request(config=[{'bool_value': True}])\n",
            "",
        ):
            with self.subTest(output=output), mock.patch(
                "native_idle_actions.subprocess.run", return_value=completed(output),
            ) as run:
                with self.assertRaises(RuntimeError):
                    IdleActionControl().set_enabled(True)
                run.assert_called_once()

    def test_acknowledged_request_with_mismatched_readback_is_failure(self):
        with mock.patch("native_idle_actions.subprocess.run", side_effect=[
            completed(ACCEPTED), completed('{"enabled": false}'),
        ]):
            with self.assertRaisesRegex(RuntimeError, "state mismatch"):
                IdleActionControl().set_enabled(True)

    def test_timeout_and_missing_runtime_are_not_retried(self):
        for error in (subprocess.TimeoutExpired("native", 25), FileNotFoundError("nsenter")):
            with self.subTest(error=error), mock.patch(
                "native_idle_actions.subprocess.run", side_effect=error,
            ) as run:
                with self.assertRaises(RuntimeError):
                    IdleActionControl().set_enabled(False)
                run.assert_called_once()

    def test_nonzero_exit_cannot_report_success_even_with_response(self):
        with mock.patch("native_idle_actions.subprocess.run", return_value=completed(
            ACCEPTED, code=1, error="native runtime unavailable",
        )) as run:
            with self.assertRaisesRegex(RuntimeError, "native runtime unavailable"):
                IdleActionControl().set_enabled(False)
            run.assert_called_once()

    def test_status_does_not_call_config_service(self):
        with mock.patch("native_idle_actions.subprocess.run", return_value=completed('{"enabled": true}')) as run:
            self.assertTrue(IdleActionControl().status()["data"]["enabled"])
            run.assert_called_once()
            command = run.call_args.args[0]
            self.assertEqual(command[13:15], ["python3", "-c"])
            self.assertNotIn("rosa service call", command[-1])

    def test_invalid_readback_never_defaults_to_disabled(self):
        for output in ("not json", "[]", "{}", '{"enabled": 0}', '{"enabled": "false"}'):
            with self.subTest(output=output), mock.patch(
                "native_idle_actions.subprocess.run", return_value=completed(output),
            ):
                with self.assertRaises(RuntimeError):
                    IdleActionControl().status()

    def test_inputs_are_validated_before_any_command(self):
        with mock.patch("native_idle_actions.subprocess.run") as run:
            for container in ("", "--help", "behavior; command", None):
                with self.subTest(container=container), self.assertRaises(ValueError):
                    IdleActionControl(container)
            for enabled in ("false", 0, None):
                with self.subTest(enabled=enabled), self.assertRaises(ValueError):
                    IdleActionControl().set_enabled(enabled)
            run.assert_not_called()

    def test_native_reader_extracts_only_boolean_from_vendor_config(self):
        for enabled in (True, False):
            config = {"data": [
                {"name": "unrelated", "value": {"type": 1, "bool_value": not enabled}},
                {"name": "idle_action_enabled", "value": {"type": 1, "bool_value": enabled}},
            ]}
            output = io.StringIO()
            with self.subTest(enabled=enabled), mock.patch(
                "builtins.open", mock.mock_open(read_data=json.dumps(config)),
            ) as opened, redirect_stdout(output):
                exec(_READ_CONFIG, {})
            opened.assert_called_once_with('/data/user_data/user_config_center.json', encoding='utf-8')
            self.assertEqual(json.loads(output.getvalue()), {"enabled": enabled})

    def test_native_reader_rejects_missing_duplicate_and_invalid_vendor_values(self):
        entry = {"name": "idle_action_enabled", "value": {"type": 1, "bool_value": True}}
        cases = [[], [entry, entry]] + [
            [{"name": "idle_action_enabled", "value": value}]
            for value in ({"type": 2, "bool_value": True}, {"type": True, "bool_value": True},
                          {"type": 1, "bool_value": 1}, {"type": 1})
        ]
        for entries in cases:
            with self.subTest(entries=entries), mock.patch(
                "builtins.open", mock.mock_open(read_data=json.dumps({"data": entries})),
            ), self.assertRaises(RuntimeError):
                exec(_READ_CONFIG, {})


if __name__ == "__main__":
    unittest.main()
