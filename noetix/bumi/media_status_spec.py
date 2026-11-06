"""What Bumi's on-board media unit is doing, as pure functions.

Separate from `device.py` for the same reason `odom_spec.py` and
`camera_specs.py` are: that module imports `rclpy`, so anything inside it can
only be exercised on a robot. What lives here is the part where a mistake would
be silent — which states exist, what each one is called, and which of them count
as a fault.

── the one rule this file is here to hold ───────────────────────────────────

**A sleeping robot is not a broken robot.**

Bumi naps. The media unit puts itself to sleep after `sleep_after_s` of
inactivity, and it can also be sent to sleep by voice or by command. From the
outside every one of those looks identical: the microphone stops, the camera
goes dark, the robot answers nothing. An operator is then left guessing between
"it is asleep", "it was told to sleep" and "it crashed" — and the usual recovery
is a power cycle that fixes the first two and hides the third.

So a sample reports `work_status` *and* `reason`, and only `ERROR_SLEEPED` is a
fault — the same rule `device.py` already applies to playback
(`_AUDIO_RESET_RECOVERED_STATUSES`: "Only ERROR_SLEEPED means the audio agent
itself is down"). Folding them into a single `healthy: false` would turn the
most common benign state on this robot into an alarm, and an alarm that fires
during normal use is an alarm that gets ignored.

`healthy` is therefore deliberately narrow, and `None` when the status could not
be read at all. "I do not know" and "it is fine" are different answers, and a
failing SDK call is not evidence that the robot is well.
"""
from __future__ import annotations

# Bumi's media::WorkStatus, as the pybind enum spells it.
WORK_STATUS_TEXT = {
    "READY": "就绪",
    "SLEEPED": "已休眠",
    "WAKEUPED": "已唤醒",
    "EXIT": "已退出（复位中）",
}

# Bumi's media::StatusChangeReason — the whole point of this card. The reason is
# what separates "nobody talked to it for a while" from "it fell over".
REASON_TEXT = {
    "SYSTEM_LAUNCH": "开机启动",
    "CMD_RESET": "被命令复位",
    "AUDIO_WAKEUPED": "被语音唤醒",
    "CMD_WAKEUPED": "被命令唤醒",
    "AUDIO_SLEEPED": "被语音休眠",
    "CMD_SLEEPED": "被命令休眠",
    "TIMEOUT_SLEEPED": "超时自动休眠",
    "ERROR_SLEEPED": "出错休眠",
}

# States in which the unit is listening and speaking.
AWAKE_STATUSES = ("READY", "WAKEUPED")

# The only reason that means the unit itself is down. See the module docstring.
HARD_FAULT_REASONS = ("ERROR_SLEEPED",)

# Reset transitions. Normal, brief, and not a fault — but worth reporting as
# "do not trust the next second" rather than as either healthy or broken.
TRANSITION_REASONS = ("CMD_RESET",)
TRANSITION_STATUSES = ("EXIT",)

UNKNOWN_TEXT = "未知"

# How long the current `(work_status, reason)` pair has been observed. This is
# *our* observation, not the robot's clock: the SDK hands out no timestamps of
# any kind (the same gap `odom_spec.py` documents for the IMU), so a duration we
# measured ourselves is the only one this driver can honestly report.
OBSERVATION_NOTE = (
    "observed_for_s 是本驱动观察到该状态持续的时长，不是机器人上报的时间戳 —— "
    "这个 SDK 不提供任何时间戳。last_wakeup_age_s 只有在驱动运行期间真的观察到"
    "一次切换到 WAKEUPED 时才有值，否则为 null，而不是猜一个数。")


def status_name(value) -> str:
    """The bare name of a pybind11 enum value, or of a plain string.

    The SDK hands back `media.WorkStatus.SLEEPED`; a dashboard wants `SLEEPED`
    and this card wants to look it up in the tables below. Doing the
    normalisation here rather than in `device.py` is deliberate: it is the one
    part of reading the SDK that can be wrong in a way nobody notices — a
    label that silently falls back to `未知` because the enum arrived as an
    object — so it belongs where it can be tested without a robot.
    """
    if value is None:
        return ""
    name = getattr(value, "name", None)
    if name:
        return str(name)
    text = str(value)
    return text.rsplit(".", 1)[-1] if "." in text else text


def _label(table: dict, name) -> str:
    """Human-readable label for an enum name, falling back to the raw name.

    An unknown enum value is shown as itself rather than as 未知: a firmware
    update that adds a state should leave a readable trace instead of looking
    like a failed read.
    """
    if not name:
        return UNKNOWN_TEXT
    bare = status_name(name)
    return table.get(bare, bare)


def labels(work_status, reason) -> tuple[str, str]:
    return _label(WORK_STATUS_TEXT, work_status), _label(REASON_TEXT, reason)


def sample(*, work_status=None, reason=None, error_code: int = 0,
           error_message: str = "", sleep_after_ms=None, received_ms: int,
           observed_for_s=None, last_wakeup_age_s=None,
           read_errors: int = 0) -> dict:
    """One `data/json` sample of the media unit's state.

    Every field is a top-level scalar because the dashboard's `data/json`
    renderer draws a key/value panel and renders a nested object as raw JSON
    text — the same reason `chasing/qianjiao_p200_pro` flattens its battery
    payload.

    `work_status=None` means the read failed, not that the unit is idle: the
    state fields then report `unknown` and `healthy` is `None`.
    """
    bare_status = status_name(work_status)
    bare_reason = status_name(reason)
    known = bool(bare_status)
    work_text, reason_text = labels(bare_status, bare_reason)
    name = bare_status or "unknown"
    why = bare_reason or "unknown"

    if sleep_after_ms is None:
        sleep_after_s = None
    else:
        # The SDK reports milliseconds; a negative value is not a duration.
        sleep_after_s = round(max(0, int(sleep_after_ms)) / 1000.0, 1)

    return {
        "stamp_ms": int(received_ms),
        # The SDK stamps nothing, so this is when the sample reached us.
        "stamp_source": "received",
        "work_status": name,
        "work_status_text": work_text,
        "reason": why,
        "reason_text": reason_text,
        "awake": (name in AWAKE_STATUSES) if known else None,
        "healthy": (why not in HARD_FAULT_REASONS) if known else None,
        "transitioning": bool(
            known and (name in TRANSITION_STATUSES or why in TRANSITION_REASONS)
        ),
        "sleep_after_s": sleep_after_s,
        "error_code": int(error_code or 0),
        "error_message": str(error_message or ""),
        "observed_for_s": _seconds(observed_for_s),
        "last_wakeup_age_s": _seconds(last_wakeup_age_s),
        "read_errors": int(read_errors or 0),
    }


def _seconds(value):
    return None if value is None else round(max(0.0, float(value)), 1)


def interface() -> dict:
    """The declaration a consumer reads once, at start.

    Small on purpose: unlike `motus.odom/1` there is no shared format here, and
    a consumer that needs the reason vocabulary should get it from the card
    description rather than from a second schema that can drift.
    """
    return {
        "card": "media_status",
        "read_only": True,
        "awake_statuses": list(AWAKE_STATUSES),
        "fault_reasons": list(HARD_FAULT_REASONS),
        "timing_note": OBSERVATION_NOTE,
    }
