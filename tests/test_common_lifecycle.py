"""Lifecycle answers for plugins that implement no lifecycle of their own.

Most plugins across these drivers are stateless actuators — a gesture, a dock
command, a speaker — whose dispatch knows only the actions they perform. The
framework sends `start`, `stop` and `info` to every card on the canvas, so those
plugins answer `{"error": "unknown action: start"}`, or return nothing.

That was harmless while agent-core read a bare `error` as success. It no longer
does — correctly, since the old reading hid real breakage — and because
start-project is strict, one such card rolls the **entire** project back. On
Tianyi that surfaced as the head camera having no data: `home`, a dock command
with nothing to start, failed its card and `camera_head` never got started.

The rules below are the whole of the fix; the six bundles just call them.

Run: python3 -m pytest tests/test_common_lifecycle.py -q
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common import lifecycle  # noqa: E402


# ── declined vs. failed ──────────────────────────────────────────────────────

@pytest.mark.parametrize("result", [
    {"error": "unknown action: start"},
    {"error": "Unknown action: start (tool=home)"},
    {"error": "Unknown action 'start' for tool 'arm'"},
    {"error": "  UNKNOWN ACTION: info"},
])
def test_a_decline_is_recognised(result):
    assert lifecycle.is_declined(result)


@pytest.mark.parametrize("result", [
    # Carries a state, so it is reporting an outcome — this is precisely what
    # agent-core now exists to surface and must reach it untouched.
    {"state": "error", "error": "unknown action: start"},
    {"state": "error", "error": "camera start failed, see driver log"},
    # A real argument failure. The Unitree speaker answering this really is
    # bound to nothing, and must keep failing.
    {"error": "Missing input_topic"},
    {"error": "text is required"},
    {"error": "no camera frame received yet"},
    {"state": "running"},
    {},
    "not a dict",
])
def test_a_failure_is_not_mistaken_for_a_decline(result):
    assert not lifecycle.is_declined(result)


# ── what the bundle answers instead ──────────────────────────────────────────

def test_start_reports_running():
    assert lifecycle.reply("start") == {"state": "running"}


def test_stop_is_idle_because_nothing_was_armed():
    assert lifecycle.reply("stop") == {"state": "idle"}


def test_info_reflects_whether_the_bundle_started_it():
    assert lifecycle.reply("info", started=False) == {"state": "idle"}
    assert lifecycle.reply("info", started=True) == {"state": "running"}


def test_every_reply_carries_a_state():
    # The entire failure mode was a reply without one.
    for action in lifecycle.LIFECYCLE_ACTIONS:
        for started in (True, False):
            assert "state" in lifecycle.reply(action, started)


def test_only_the_framework_actions_are_substituted():
    # A typo'd verb must still come back as an error, not a cheerful "running",
    # so the substitution is gated on this list.
    assert set(lifecycle.LIFECYCLE_ACTIONS) == {"start", "stop", "info"}


# ── every bundle actually applies it ─────────────────────────────────────────

BUNDLES = [
    "x-humanoid/tianyi2.0/main.py",
    "engineai/t800/main.py",
    "unitree/g1/main.py",
    "unitree/go2/main.py",
    "unitree/r1/main.py",
    "pndbotics/adam/device.py",
]


@pytest.mark.parametrize("rel", BUNDLES)
def test_bundle_imports_and_applies_the_helper(rel):
    src = (ROOT / rel).read_text()
    assert "from common import lifecycle as _lifecycle" in src, \
        f"{rel} uses the helper but never imports it"
    assert "_lifecycle.is_declined(result)" in src, \
        f"{rel} does not consult the helper on its dispatch result"
    assert "_lifecycle.reply(" in src


@pytest.mark.parametrize("rel", BUNDLES)
def test_bundle_ships_the_common_package(rel):
    # common/ is imported at runtime, so the image has to contain it. Several of
    # these Dockerfiles COPY files by name, where an omission means a driver
    # that cannot import its own dependency and never starts at all.
    dockerfile = ROOT / Path(rel).parent / "Dockerfile"
    assert dockerfile.is_file(), f"no Dockerfile beside {rel}"
    assert re.search(r"^COPY\s+common/", dockerfile.read_text(), re.M), \
        f"{dockerfile} does not COPY common/"
