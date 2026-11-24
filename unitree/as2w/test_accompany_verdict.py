"""Tests for the offline verdict tool.

Every branch of the ACCOMPANY_FINDINGS.md §5 table is pinned here with
synthetic evidence, so a future edit cannot silently change what an
on-robot run would conclude. The end-to-end test runs the real CLI over
temporary JSON files exactly like the operator will.
"""

import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import accompany_verdict as vd


class Step1Test(unittest.TestCase):
    def test_suspicious_service_is_h3(self):
        findings = vd.verdict_step1({
            "services": {"sport": {"request": True, "response": True},
                         "accompany": {"request": True, "response": False}},
        })
        self.assertEqual(1, len(findings))
        self.assertTrue(findings[0]["verdict"].startswith("H3"))
        self.assertIn("accompany", findings[0]["signal"])

    def test_response_only_service_is_flagged_as_uncertain(self):
        findings = vd.verdict_step1({
            "services": {"follow": {"request": False, "response": True}},
        })
        self.assertIn("未必 host", findings[0]["signal"])

    def test_clean_services_produce_no_finding(self):
        findings = vd.verdict_step1({
            "services": {"sport": {"request": True, "response": True},
                         "audio": {"request": True, "response": False}},
        })
        self.assertEqual([], findings)

    def test_endpoints_fallback_is_parsed(self):
        findings = vd.verdict_step1({
            "endpoints": {"rt/api/iss_operate/request": "dummy"},
        })
        self.assertEqual(1, len(findings))
        self.assertIn("iss_operate", findings[0]["signal"])

    def test_checkmode_payload_with_accompany_name_is_cheaper_h3(self):
        findings = vd.verdict_step1({
            "motion_switcher": {"code": 0,
                                "data": {"name": "AI_ACCOMPANY", "form": "as2"}},
        })
        self.assertTrue(findings[0]["verdict"].startswith("H3"))
        self.assertIn("更省事", findings[0]["verdict"])
        self.assertIn("AI_ACCOMPANY", findings[0]["signal"])

    def test_checkmode_payload_without_hits_produces_nothing(self):
        findings = vd.verdict_step1({
            "motion_switcher": {"code": 0, "data": {"name": "normal"}},
        })
        self.assertEqual([], findings)


class Step2Test(unittest.TestCase):
    def test_undocumented_mode_is_independent_mode(self):
        findings = vd.verdict_step2({
            "watch": {"modes": {"3": 120, "17": 8},
                      "fsm_names": {"WALK": 128}, "pairs": {}},
        })
        self.assertEqual(1, len(findings))
        self.assertIn("17", findings[0]["signal"])
        self.assertIn("独立 mode", findings[0]["verdict"])

    def test_new_fsm_name_is_h1_with_config_hint(self):
        findings = vd.verdict_step2({
            "watch": {"modes": {"3": 120},
                      "fsm_names": {"WALK": 100, "AI_ACCOMPANY": 8},
                      "pairs": {"3:WALK": 100, "17:AI_ACCOMPANY": 8}},
        })
        self.assertEqual(1, len(findings))
        self.assertTrue(findings[0]["verdict"].startswith("H1"))
        self.assertIn("accompany.fsm_names", findings[0]["next"])
        self.assertIn("17", findings[0]["signal"])

    def test_all_known_values_means_invisible(self):
        findings = vd.verdict_step2({
            "watch": {"modes": {"3": 120}, "fsm_names": {"WALK": 128},
                      "pairs": {}},
        })
        self.assertEqual(1, len(findings))
        self.assertIn("不可见", findings[0]["verdict"])
        self.assertIn("H2", findings[0]["next"])
        self.assertIn("厂商渠道", findings[0]["next"])

    def test_degraded_feed_still_reports_h1_from_fsm_name(self):
        """GetState-only evidence must not fall through to "both invisible"."""
        findings = vd.verdict_step2({
            "watch": {"modes": {"None": 40}, "pairs": {},
                      "fsm_names": {"AI_ACCOMPANY": 40}},
        })
        self.assertEqual(1, len(findings))
        self.assertTrue(findings[0]["verdict"].startswith("H1"))

    def test_degraded_feed_is_not_reported_as_an_undocumented_mode(self):
        """A missing frame is a dead link, not a vendor-secret mode value."""
        findings = vd.verdict_step2({
            "watch": {"modes": {"None": 40}, "pairs": {}, "fsm_names": {}},
        })
        self.assertEqual(1, len(findings))
        self.assertNotIn("独立 mode", findings[0]["verdict"])

    def test_empty_watch_also_verdicts_invisible(self):
        findings = vd.verdict_step2({"watch": {}})
        self.assertEqual(1, len(findings))
        self.assertIn("不可见", findings[0]["verdict"])


class Step3Test(unittest.TestCase):
    def test_code_zero_is_h2_candidate_with_reproduction_redline(self):
        findings = vd.verdict_step3({
            "probe": {"1022": {"code": 0, "data": None}},
        })
        self.assertTrue(findings[0]["verdict"].startswith("H2"))
        self.assertIn("3 次", findings[0]["next"])
        self.assertIn("别写进卡", findings[0]["next"])

    def test_four_thousand_codes_are_excluded(self):
        findings = vd.verdict_step3({
            "probe": {"1023": {"code": 4001, "data": None}},
        })
        self.assertIn("排除", findings[0]["verdict"])


class VerdictTest(unittest.TestCase):
    def test_merges_all_steps_in_order(self):
        findings = vd.verdict({
            "services": {"accompany": {"request": True, "response": False}},
            "watch": {"modes": {"17": 8}, "fsm_names": {}, "pairs": {}},
            "probe": {"1022": {"code": 0, "data": None}},
        })
        self.assertEqual([1, 2, 3], [item["step"] for item in findings])

    def test_overall_prefers_h3_over_everything(self):
        findings = vd.verdict({
            "services": {"accompany": {"request": True, "response": False}},
            "watch": {"modes": {"17": 8}, "fsm_names": {}, "pairs": {}},
        })
        self.assertTrue(vd.overall(findings).startswith("H3"))

    def test_overall_names_the_escalation_when_invisible(self):
        findings = vd.verdict({
            "watch": {"modes": {"3": 10}, "fsm_names": {"WALK": 10},
                      "pairs": {}},
        })
        self.assertIn("H2", vd.overall(findings))
        self.assertIn("厂商渠道", vd.overall(findings))

    def test_overall_on_empty_data_is_not_a_crash(self):
        self.assertIn("为空", vd.overall([]))
        self.assertIn("可判读", vd.overall([]))

    def test_verdict_on_empty_data_is_empty(self):
        self.assertEqual([], vd.verdict({"at": "2026-11-19T00:00:00"}))


class EndToEndTest(unittest.TestCase):
    def test_cli_reads_real_probe_json_shapes(self):
        step1 = {
            "at": "2026-11-19T10:00:00", "iface": "eth0",
            "endpoints": {"rt/api/sport/request": "Dummy",
                          "rt/api/sport/response": "Dummy",
                          "rt/api/iss_operate/request": "Dummy"},
            "services": {"sport": {"request": True, "response": True},
                         "iss_operate": {"request": True, "response": False}},
            "motion_switcher": {"code": 0, "data": {"name": "normal"}},
        }
        step2 = {
            "at": "2026-11-19T10:05:00", "iface": "eth0",
            "watch": {"modes": {"3": 40, "17": 9},
                      "mode_names": {"3": "DEFAULT_MODE",
                                     "17": "UNDOCUMENTED_17"},
                      "fsm_names": {"WALK": 40, "AI_ACCOMPANY": 9},
                      "pairs": {"3:WALK": 40, "17:AI_ACCOMPANY": 9}},
        }
        with tempfile.TemporaryDirectory() as tmp:
            paths = []
            for name, payload in (("step1.json", step1), ("step2.json", step2)):
                path = Path(tmp) / name
                path.write_text(json.dumps(payload), encoding="utf-8")
                paths.append(str(path))
            report = Path(tmp) / "verdict.json"
            code = vd.main(paths + ["--json", str(report)])
            self.assertEqual(0, code)
            payload = json.loads(report.read_text(encoding="utf-8"))
            self.assertTrue(payload["overall"].startswith("H3"))
            self.assertIn("H1", [item["verdict"][:2] for item in payload["findings"]])
            steps = {item["step"] for item in payload["findings"]}
            self.assertEqual({1, 2}, steps)


if __name__ == "__main__":
    unittest.main()
