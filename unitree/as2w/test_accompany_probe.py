"""Tests for the read-only accompany reconnaissance tool.

The tool itself only runs on the robot, but every decision it makes is a pure
function, so those are pinned here. In particular the safety guard is tested:
if it ever stops refusing, an unarmed run would push unknown api ids at the
firmware.
"""

import contextlib
import io
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import accompany_probe as probe


class ServiceParsingTest(unittest.TestCase):
    def test_request_and_response_are_merged_per_service(self):
        services = probe.api_services([
            "rt/api/sport/request",
            "rt/api/sport/response",
            "rt/api/slam_operate/request",
        ])
        self.assertEqual({"request": True, "response": True}, services["sport"])
        self.assertEqual({"request": True, "response": False},
                         services["slam_operate"])

    def test_non_api_topics_are_ignored(self):
        services = probe.api_services([
            "rt/lf/sportmodestate", "rt/lowstate", "rt/utlidar/cloud", "",
        ])
        self.assertEqual({}, services)

    def test_malformed_api_topics_are_ignored(self):
        services = probe.api_services([
            "rt/api/", "rt/api/sport", "rt/api/sport/other", "/api/sport/request",
        ])
        self.assertEqual({}, services)

    def test_suspicious_picks_the_names_worth_a_second_look(self):
        services = probe.api_services([
            "rt/api/sport/request", "rt/api/accompany/request",
            "rt/api/iss_operate/request", "rt/api/audio_led/request",
            "rt/api/follow/request",
        ])
        self.assertEqual(["accompany", "follow", "iss_operate"],
                         probe.suspicious(services))

    def test_suspicious_is_empty_when_nothing_matches(self):
        services = probe.api_services(["rt/api/sport/request"])
        self.assertEqual([], probe.suspicious(services))


class ModeNameTest(unittest.TestCase):
    def test_the_vendor_table_is_fourteen_entries(self):
        self.assertEqual(14, len(probe.MODE_NAMES))
        self.assertEqual("PASSIVE", probe.MODE_NAMES[0])
        self.assertEqual("BASE_HEIGHT_CTRL", probe.MODE_NAMES[13])

    def test_known_indices_resolve(self):
        self.assertEqual("RUNNING_MODE", probe.mode_name(4))
        self.assertEqual("HANDSTAND", probe.mode_name(8))

    def test_an_index_beyond_the_table_is_marked_undocumented(self):
        self.assertEqual("UNDOCUMENTED_14", probe.mode_name(14))
        self.assertTrue(probe.undocumented_mode(14))
        self.assertFalse(probe.undocumented_mode(13))

    def test_a_non_integer_mode_does_not_raise(self):
        self.assertIn("non-integer", probe.mode_name(None))
        self.assertTrue(probe.undocumented_mode("garbage"))


class ApiGapTest(unittest.TestCase):
    def test_gaps_are_exactly_the_holes_in_the_vendor_table(self):
        self.assertEqual(
            (1010, 1012, 1014,
             1022, 1023, 1024, 1025, 1026, 1027, 1028, 1029, 1030, 1031,
             1032, 1033,
             1035, 1036, 1037, 1038, 1039),
            probe.sport_api_gaps())

    def test_known_ids_are_not_reported_as_gaps(self):
        self.assertNotIn(1008, probe.sport_api_gaps())
        self.assertNotIn(1041, probe.sport_api_gaps())


class FsmNameTest(unittest.TestCase):
    def test_a_seen_name_is_not_new(self):
        self.assertFalse(probe.is_new_fsm_name("AI_FREE_WALK"))

    def test_case_and_whitespace_are_normalised(self):
        self.assertFalse(probe.is_new_fsm_name("  ai_free_walk "))

    def test_an_unseen_name_is_new(self):
        self.assertTrue(probe.is_new_fsm_name("AI_ACCOMPANY"))

    def test_empty_name_is_not_new(self):
        self.assertFalse(probe.is_new_fsm_name(""))


class WatchReportTest(unittest.TestCase):
    def test_no_frame_bucket_renders_without_crashing(self):
        """A degraded link yields mode=None; the report must still be written."""
        lines = probe.watch_mode_lines({None: 5})
        self.assertEqual(1, len(lines))
        self.assertIn("?", lines[0])
        self.assertIn("5", lines[0])

    def test_none_and_real_modes_sort_without_comparing_types(self):
        lines = probe.watch_mode_lines({3: 2, None: 5, 17: 1})
        self.assertEqual(3, len(lines))
        self.assertIn("?", lines[0], "the no-frame bucket sorts first")
        self.assertIn("帧数 2", " ".join(lines))


class WatchDegradedLinkTest(unittest.TestCase):
    def test_watch_finishes_without_any_mode_callback(self):
        """No DDS frame must still produce evidence JSON, not a TypeError."""
        channel = types.ModuleType("unitree_sdk2py.core.channel")

        class FakeSubscriber:
            def __init__(self, topic, msg_type):
                self.topic = topic

            def Init(self, callback, queue_depth):
                # Deliberately never fire callback: simulate a dead DDS link.
                pass

        channel.ChannelSubscriber = FakeSubscriber
        dds = types.ModuleType("unitree_sdk2py.idl.unitree_go.msg.dds_")
        dds.SportModeState_ = type("SportModeState_", (), {})
        client_module = types.ModuleType(
            "unitree_sdk2py.as2.sport.sport_client")

        class FakeSportClient:
            def SetTimeout(self, seconds):
                pass

            def Init(self):
                pass

            def GetState(self, state_map):  # exact SDK signature
                state_map["fsm_name"] = "AI_ACCOMPANY"
                return 0

        client_module.SportClient = FakeSportClient
        modules = {
            "unitree_sdk2py": types.ModuleType("unitree_sdk2py"),
            "unitree_sdk2py.idl": types.ModuleType("unitree_sdk2py.idl"),
            "unitree_sdk2py.idl.unitree_go": types.ModuleType(
                "unitree_sdk2py.idl.unitree_go"),
            "unitree_sdk2py.idl.unitree_go.msg": types.ModuleType(
                "unitree_sdk2py.idl.unitree_go.msg"),
            "unitree_sdk2py.idl.unitree_go.msg.dds_": dds,
            "unitree_sdk2py.core": types.ModuleType("unitree_sdk2py.core"),
            "unitree_sdk2py.core.channel": channel,
            "unitree_sdk2py.as2": types.ModuleType("unitree_sdk2py.as2"),
            "unitree_sdk2py.as2.sport": types.ModuleType(
                "unitree_sdk2py.as2.sport"),
            "unitree_sdk2py.as2.sport.sport_client": client_module,
        }
        originals = {name: sys.modules.get(name) for name in modules}
        sys.modules.update(modules)
        try:
            args = types.SimpleNamespace(watch=0.3)
            findings = {}
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                code = probe.cmd_watch(args, findings)
        finally:
            for name, original in originals.items():
                if original is None:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = original

        self.assertEqual(0, code)
        self.assertIn("?", output.getvalue())
        self.assertIn("None", findings["watch"]["modes"])
        # With no mode frame, pairs/names stay empty by design; the FSM name
        # still lives in the time-correlated samples, which is the evidence.
        samples = findings["watch"]["samples"]
        self.assertTrue(samples)
        self.assertEqual({"AI_ACCOMPANY"},
                         {sample["fsm_name"] for sample in samples})
        self.assertTrue(all(sample["mode"] is None for sample in samples))


class ArmedGuardTest(unittest.TestCase):
    def test_it_refuses_without_any_flag(self):
        with self.assertRaises(RuntimeError):
            probe._guard_armed(False, False, False)

    def test_it_names_every_missing_flag(self):
        with self.assertRaises(RuntimeError) as caught:
            probe._guard_armed(True, False, False)
        message = str(caught.exception)
        self.assertIn("--robot-on-stand", message)
        self.assertIn("--remote-in-hand", message)
        self.assertNotIn("--armed", message)

    def test_it_passes_when_all_three_are_given(self):
        self.assertIsNone(probe._guard_armed(True, True, True))

    def test_unarmed_cli_run_is_refused_before_any_sdk_import(self):
        args = probe.build_parser().parse_args(["--probe-ids", "1022"])
        findings = {}
        self.assertEqual(4, probe.cmd_probe(args, findings))
        self.assertEqual({}, findings)


class ParserTest(unittest.TestCase):
    def test_defaults_are_read_only(self):
        args = probe.build_parser().parse_args([])
        self.assertFalse(args.armed)
        self.assertFalse(args.robot_on_stand)
        self.assertFalse(args.remote_in_hand)
        self.assertEqual("", args.probe_ids)
        self.assertEqual(0.0, args.watch)

    def test_watch_takes_seconds(self):
        args = probe.build_parser().parse_args(["--watch", "60"])
        self.assertEqual(60.0, args.watch)


if __name__ == "__main__":
    unittest.main()
