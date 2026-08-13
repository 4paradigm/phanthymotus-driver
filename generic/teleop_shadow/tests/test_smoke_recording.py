from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from smoke_recording import run_smoke


class RecordingSmokeTests(unittest.TestCase):
    def test_smoke_produces_visible_apply_stop_and_release_proof(self):
        proof = run_smoke()
        self.assertFalse(proof["actuation_enabled"])
        self.assertEqual("active_shadow", proof["motion_state"])
        self.assertEqual(1, proof["last_would_apply_sequence"])
        self.assertEqual(
            {"state": "hold", "reason": "soft_stop", "acknowledged": True},
            proof["soft_stop"],
        )
        self.assertEqual("released", proof["release"]["state"])
        self.assertIn(
            {"kind": "would_apply", "sequence": 1},
            proof["recorded_decisions"],
        )
        self.assertIn(
            {"kind": "would_stop", "reason": "soft_stop"},
            proof["recorded_decisions"],
        )


if __name__ == "__main__":
    unittest.main()
