"""Bumi media-unit configurable settings, as pure functions.

Separate from ``device.py`` for the same reason ``media_status_spec.py`` and
``odom_spec.py`` are: that module imports ``rclpy``, so anything inside it can
only be exercised on a robot. What lives here is the part where a mistake would
be silent — which settings exist, what each one is called in Chinese, which of
them can be written, and what a valid value is.

── the rule this file is here to hold ──────────────────────────────────────

**Read-only is not a missing setter.**

The pybind module exposes ``get_wakeup_words`` but no ``set_wakeup_words``: the
vendor deliberately locks the wake word, so a card must present it as read-only
rather than advertising a set action that cannot exist. Distinguishing
readable-but-fixed from readable-and-writable is exactly the kind of judgment
that fails silently — a "set wake word" action that no-op errors on hardware —
so it belongs here where it can be tested without a robot.

Every writable setting also carries a value guard here, because ``set_timeout``
takes milliseconds (not seconds) and the audio-cue flag is a real boolean; a
wrong unit or a truthy string would be accepted by the C++ layer and only show
up later as "the robot does not sleep" or "the beep never fires".
"""
from __future__ import annotations


# ── The settings this card knows about ──────────────────────────────────────
#
# `key` is the MCP action/argument name a caller uses; `writable` separates the
# settings the SDK actually lets us change from the one it only lets us read.
# `label` is the human-facing Chinese name shown on a key/value panel.
SETTINGS = {
    "timeout": {
        "label": "自动休眠超时",
        "writable": True,
        "getter": "get_timeout",
        "setter": "set_timeout",
    },
    "audio_cue": {
        "label": "提示音开关",
        "writable": True,
        "getter": "get_audio_cue_enable",
        "setter": "set_audio_cue_enable",
    },
    "wakeup_response": {
        "label": "唤醒应答词",
        "writable": True,
        "getter": "get_wakeup_response_words",
        "setter": "set_wakeup_response_words",
    },
    "sleep_response": {
        "label": "休眠应答词",
        "writable": True,
        "getter": "get_sleep_response_words",
        "setter": "set_sleep_response_words",
    },
    "wakeup_words": {
        "label": "唤醒词",
        "writable": False,
        "getter": "get_wakeup_words",
        "setter": None,
    },
}

# Ordered for a stable `info` payload.
SETTING_KEYS = tuple(SETTINGS)

# The settings a caller may write. ``wakeup_words`` is deliberately absent.
WRITABLE_KEYS = tuple(k for k in SETTING_KEYS if SETTINGS[k]["writable"])

# The vendor's auto-sleep timeout unit is milliseconds. A value is only ever a
# number, but the range is what keeps a typo like ``set_timeout(5)`` — meaning
# "sleep after 5 ms" — from being accepted as a plausible "5 s".
TIMEOUT_MIN_MS = 1
TIMEOUT_MAX_MS = 3_600_000  # one hour, an upper bound far beyond any real use

# Response words are short text; the SDK stores them as a single string. No
# known hard limit, so the guard is a sanity cap against garbage, not a protocol
# limit. A non-empty string is required so a caller cannot silently blank a
# response the robot is expected to speak.
RESPONSE_WORDS_MAX_CHARS = 256


def is_writable(key: str) -> bool:
    """Whether the SDK lets us write ``key`` (``wakeup_words`` is read-only)."""
    return SETTINGS.get(key, {}).get("writable", False)


def label(key: str) -> str:
    """The Chinese label for a setting key, falling back to the raw key."""
    entry = SETTINGS.get(key)
    return entry["label"] if entry else key


def getter_name(key: str) -> str:
    return SETTINGS[key]["getter"]


def setter_name(key: str) -> str | None:
    return SETTINGS[key]["setter"]


def validate_timeout_ms(value) -> int:
    """Validate and normalise the auto-sleep timeout, in milliseconds.

    Rejects booleans (``True`` is not a duration) and non-finite numbers, then
    enforces the range. Returns an ``int``; the SDK takes milliseconds.
    """
    if isinstance(value, bool):
        raise ValueError("timeout_ms 必须是数字，单位毫秒")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("timeout_ms 必须是数字，单位毫秒") from exc
    if number != number or number in (float("inf"), float("-inf")):
        raise ValueError("timeout_ms 必须是有限数字")
    result = int(number)
    if not TIMEOUT_MIN_MS <= result <= TIMEOUT_MAX_MS:
        raise ValueError(
            f"timeout_ms 必须在 [{TIMEOUT_MIN_MS}, {TIMEOUT_MAX_MS}] 毫秒之间")
    return result


def validate_response_words(value) -> str:
    """Validate a response-words string.

    Must be a non-empty string within the sanity cap. Strips surrounding
    whitespace; a caller may legitimately want to clear a response, but that is
    a real decision and is rejected here rather than silently blanked.
    """
    if not isinstance(value, str):
        raise ValueError("应答词必须是字符串")
    text = value.strip()
    if not text:
        raise ValueError("应答词不能为空")
    if len(text) > RESPONSE_WORDS_MAX_CHARS:
        raise ValueError(f"应答词不能超过 {RESPONSE_WORDS_MAX_CHARS} 个字符")
    return text


def validate_audio_cue(value) -> bool:
    """Validate the audio-cue flag as a real boolean, not a truthy string."""
    if isinstance(value, bool):
        return value
    raise ValueError("audio_cue 必须是布尔值 true/false")


def validate(key: str, value):
    """Dispatch to the per-setting validator; unknown keys are rejected."""
    if key == "timeout":
        return validate_timeout_ms(value)
    if key in ("wakeup_response", "sleep_response"):
        return validate_response_words(value)
    if key == "audio_cue":
        return validate_audio_cue(value)
    raise ValueError(f"未知配置项: {key}")


def interface() -> dict:
    """The declaration a consumer reads once, at start.

    Lists the settings, which are writable, and how a value is validated, so a
    caller can decide before issuing a ``set`` whether it will be accepted.
    """
    return {
        "card": "media_config",
        "settings": [
            {
                "key": key,
                "label": SETTINGS[key]["label"],
                "writable": SETTINGS[key]["writable"],
            }
            for key in SETTING_KEYS
        ],
        "timeout_unit": "milliseconds",
        "timeout_range_ms": [TIMEOUT_MIN_MS, TIMEOUT_MAX_MS],
        "response_words_max_chars": RESPONSE_WORDS_MAX_CHARS,
    }
