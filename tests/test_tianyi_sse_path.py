"""
test_tianyi_sse_path.py — the SSE endpoint has to sit where the client looks.

agent-core builds the stream URL as `<mcp url>/sse`, and the configured url for a
driver already ends in `/mcp` — so what it actually requests is `/mcp/sse`. This
driver served `/sse` at the root, so its SSE endpoint had never once been
reached: every probe since the subscription shipped answered 404. Four of the
fifteen drivers in this repo serve `/mcp/sse` (dji/M300, engineai/t800,
pndbotics/adam, unitree/as2w) and that is the convention.

The 404s were also the visible half of a separate agent-core bug (a leaked
subscription task per reconnect, phanthymotus#234) — 310262 of this container's
315088 log lines. That leak is fixed upstream; this is the other half, which
stops the endpoint from being dead code.

`/sse` stays accepted. Dropping it would be a bet that nothing else ever learned
to call it, and the bet buys nothing.

Run: cd phanthymotus-driver && python3 -m pytest tests/test_tianyi_sse_path.py
"""

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TIANYI_MAIN = ROOT / "x-humanoid" / "tianyi2.0" / "main.py"


class TianyiSsePathTests(unittest.TestCase):
    """Source assertions, matching how the other driver tests here check routing
    (see unitree/as2w/test_driver.py) — importing a driver main pulls in its SDK."""

    @classmethod
    def setUpClass(cls):
        cls.source = TIANYI_MAIN.read_text(encoding="utf-8")

    def test_it_serves_the_path_the_client_requests(self):
        self.assertIn('"/mcp/sse"', self.source,
                      "agent-core asks for /mcp/sse; without it the endpoint is unreachable")

    def test_the_old_path_still_works(self):
        self.assertIn('"/sse"', self.source)

    def test_both_paths_are_handled_by_the_same_branch(self):
        """Two separate branches would drift; one membership test cannot."""
        self.assertIn('in ("/mcp/sse", "/sse")', self.source)

    def test_the_query_string_is_still_stripped(self):
        """`?session_id=...` must not turn a match into a 404."""
        self.assertIn('self.path.split("?")[0]', self.source)


class ConventionTests(unittest.TestCase):
    def test_every_driver_that_serves_sse_covers_the_mcp_prefixed_path(self):
        """A driver may serve no SSE at all — agent-core now gives up after two
        404s. What it must not do is serve SSE on a path nobody requests."""
        offenders = []
        for main in sorted(ROOT.glob("*/*/main.py")):
            src = main.read_text(encoding="utf-8", errors="replace")
            if '"/sse"' in src and '"/mcp/sse"' not in src:
                offenders.append(str(main.relative_to(ROOT)))
        self.assertEqual(offenders, [],
                         f"these serve SSE where the client never looks: {offenders}")


if __name__ == "__main__":
    unittest.main()
