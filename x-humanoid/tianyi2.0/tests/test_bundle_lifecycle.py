"""The bundle's answer for plugins that implement no lifecycle of their own.

Most plugins here are stateless actuators — a gesture, a dock command — whose
dispatch knows only the actions they perform. The framework nonetheless sends
`start` to every card on the canvas, and those plugins answer
`{"error": "unknown action: start"}`.

That was harmless while agent-core read a bare `error` as success. Once it began
treating one as a failed start — correctly, since the old reading hid real
breakage — `home` failed its card, and because start-project is strict, that one
card rolled the whole project back. The robot came up with nothing running; it
was reported as the camera having lost its data, because `camera_head` never got
started either.

Run: cd x-humanoid/tianyi2.0 && python3 -m pytest tests/test_bundle_lifecycle.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import lifecycle  # noqa: E402


# ── telling a refusal apart from a failure ───────────────────────────────────

@pytest.mark.parametrize("phrasing", [
    "unknown action: start",
    "Unknown action: start (tool=home)",
    "  UNKNOWN ACTION: start",
])
def test_a_refusal_is_recognised_whatever_the_casing(phrasing):
    assert lifecycle.is_unknown_action({"error": phrasing})


@pytest.mark.parametrize("result", [
    # Carries a state, so it is describing an outcome — this is exactly what
    # agent-core now looks at, and it must reach it untouched.
    {"state": "error", "error": "unknown action: start"},
    {"error": "text is required"},       # a real argument error, not a refusal
    {"error": "no camera frame received yet"},
    {"state": "running"},
    {},
    None,
    "not a dict",
])
def test_everything_else_is_left_alone(result):
    assert not lifecycle.is_unknown_action(result)


# ── what the bundle answers in its place ─────────────────────────────────────

def test_start_reports_running_once_the_bundle_has_constructed_it():
    assert lifecycle.lifecycle_reply("start", started=True) == {"state": "running"}


def test_info_reflects_whether_it_was_ever_started():
    assert lifecycle.lifecycle_reply("info", started=False) == {"state": "idle"}
    assert lifecycle.lifecycle_reply("info", started=True) == {"state": "running"}


def test_stop_is_idle_because_nothing_was_armed():
    assert lifecycle.lifecycle_reply("stop", started=True) == {"state": "idle"}


def test_every_reply_carries_a_state():
    # The entire failure mode was a reply without one. None of these may ever
    # go back out as a bare error again.
    for action in lifecycle.LIFECYCLE_ACTIONS:
        for started in (True, False):
            assert "state" in lifecycle.lifecycle_reply(action, started)


def test_only_the_framework_actions_get_a_default():
    # A typo'd verb must still come back as an error rather than a cheerful
    # "running", so the substitution is gated on this list.
    assert "wave_goodbye" not in lifecycle.LIFECYCLE_ACTIONS
    assert set(lifecycle.LIFECYCLE_ACTIONS) == {"start", "stop", "info"}
