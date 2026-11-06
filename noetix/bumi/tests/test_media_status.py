"""The media-unit state card's payload rules, exercised off a robot.

`media_status_spec` imports no `rclpy` precisely so that these can run on a
laptop. What is worth pinning down here is not that the dictionary has the right
keys but the three decisions that would be silent if they broke: sleep is not a
fault, an unreadable status is not a healthy one, and nothing is nested (the
dashboard renders a nested object as raw JSON text).
"""
import sys
import types
from pathlib import Path

import pytest

from test_mic_card import _load_device

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import media_status_spec as spec  # noqa: E402


def _sample(**overrides):
    args = {"work_status": "READY", "reason": "SYSTEM_LAUNCH", "received_ms": 1}
    args.update(overrides)
    return spec.sample(**args)


def test_sleeping_after_a_timeout_is_not_a_fault():
    payload = _sample(work_status="SLEEPED", reason="TIMEOUT_SLEEPED")

    assert payload["healthy"] is True
    assert payload["awake"] is False
    assert payload["transitioning"] is False
    assert payload["work_status_text"] == "已休眠"
    assert payload["reason_text"] == "超时自动休眠"


def test_voice_triggered_sleep_is_not_confused_with_the_timeout():
    timeout = _sample(work_status="SLEEPED", reason="TIMEOUT_SLEEPED")
    voice = _sample(work_status="SLEEPED", reason="AUDIO_SLEEPED")

    assert timeout["work_status"] == voice["work_status"]
    assert timeout["reason"] != voice["reason"]
    assert timeout["reason_text"] != voice["reason_text"]


def test_only_an_error_sleep_is_unhealthy():
    assert _sample(work_status="SLEEPED", reason="ERROR_SLEEPED")["healthy"] is False
    assert _sample(work_status="SLEEPED", reason="CMD_SLEEPED")["healthy"] is True
    assert _sample(work_status="WAKEUPED", reason="AUDIO_WAKEUPED")["healthy"] is True


def test_an_unreadable_status_is_unknown_rather_than_healthy():
    payload = _sample(work_status=None, reason=None, read_errors=3)

    assert payload["healthy"] is None
    assert payload["awake"] is None
    assert payload["work_status"] == "unknown"
    assert payload["work_status_text"] == "未知"
    assert payload["read_errors"] == 3


def test_a_reset_transition_is_flagged_but_stays_healthy():
    payload = _sample(work_status="EXIT", reason="CMD_RESET")

    assert payload["transitioning"] is True
    assert payload["healthy"] is True


def test_sleep_threshold_is_reported_in_seconds_and_floored():
    assert _sample(sleep_after_ms=120000)["sleep_after_s"] == 120.0
    assert _sample(sleep_after_ms=-5000)["sleep_after_s"] == 0.0
    assert _sample(sleep_after_ms=None)["sleep_after_s"] is None


def test_observation_durations_are_never_negative():
    payload = _sample(observed_for_s=-1.0, last_wakeup_age_s=None)

    assert payload["observed_for_s"] == 0.0
    assert payload["last_wakeup_age_s"] is None


def test_an_unseen_wakeup_is_null_rather_than_a_guess():
    # The card starts while the robot is already awake: nothing was observed,
    # so there is no age to report and inventing one would be a claim.
    payload = _sample(work_status="WAKEUPED", reason="AUDIO_WAKEUPED",
                      last_wakeup_age_s=None)

    assert payload["last_wakeup_age_s"] is None


def test_a_state_added_by_a_firmware_update_stays_readable():
    payload = _sample(work_status="HIBERNATING", reason="SOMETHING_NEW")

    assert payload["work_status"] == "HIBERNATING"
    assert payload["work_status_text"] == "HIBERNATING"
    assert payload["awake"] is False
    assert payload["healthy"] is True


def test_a_pybind_enum_value_is_normalized_and_labelled():
    # What the SDK actually returns: an object, not a string. Passing it
    # through unnormalised is the failure this card would never show — every
    # label would read 未知 and look like a robot that will not answer.
    class _Enum:
        name = "SLEEPED"

        def __str__(self):
            return "media.WorkStatus.SLEEPED"

    payload = _sample(work_status=_Enum(), reason="TIMEOUT_SLEEPED")

    assert payload["work_status"] == "SLEEPED"
    assert payload["work_status_text"] == "已休眠"
    assert payload["awake"] is False


def test_an_enum_without_a_name_falls_back_to_its_last_component():
    class _Bare:
        def __str__(self):
            return "media.StatusChangeReason.CMD_RESET"

    assert spec.status_name(_Bare()) == "CMD_RESET"
    assert spec.status_name(None) == ""


def test_every_field_is_a_scalar_the_kv_panel_can_draw():
    payload = _sample(sleep_after_ms=120000, observed_for_s=133.24,
                      last_wakeup_age_s=133.24)

    for key, value in payload.items():
        assert not isinstance(value, (dict, list, tuple)), key
        assert value is None or isinstance(value, (str, int, float, bool)), key


def test_the_interface_names_the_fault_and_awake_sets():
    interface = spec.interface()

    assert interface["read_only"] is True
    assert interface["fault_reasons"] == ["ERROR_SLEEPED"]
    assert set(interface["awake_statuses"]) == {"READY", "WAKEUPED"}
    # The "these durations are our observation, not the robot's clock" caveat
    # ships with the card rather than living only in a README.
    assert interface["timing_note"]


@pytest.mark.parametrize("status", sorted(spec.WORK_STATUS_TEXT))
def test_every_work_status_the_sdk_can_report_has_a_label(status):
    assert spec.WORK_STATUS_TEXT[status] != spec.UNKNOWN_TEXT


@pytest.mark.parametrize("reason", sorted(spec.REASON_TEXT))
def test_every_reason_the_sdk_can_report_has_a_label(reason):
    assert spec.REASON_TEXT[reason] != spec.UNKNOWN_TEXT


# ── the node and the card: what only breaks against a live SDK ──────────────

class _Media:
    """The three read-only MediaController calls this card is allowed to make."""

    def __init__(self, work_status="READY", reason="SYSTEM_LAUNCH",
                 code=0, message="", timeout_ms=120000, fail=False):
        self.work_status = work_status
        self.reason = reason
        self.code = code
        self.message = message
        self.timeout_ms = timeout_ms
        self.fail = fail

    def get_system_status(self):
        if self.fail:
            raise RuntimeError("media unit unreachable")
        return types.SimpleNamespace(value=self.work_status, reason=self.reason)

    def get_system_error(self):
        return types.SimpleNamespace(code=self.code, message=self.message)

    def get_timeout(self):
        return self.timeout_ms


class _Clock:
    """A controllable stand-in for `time`, so durations are exact."""

    def __init__(self, now=1000.0):
        self.now = now

    def monotonic(self):
        return self.now

    def time(self):
        return 1_700_000_000.0 + self.now

    def sleep(self, seconds):
        self.now += seconds


def _node(monkeypatch, media, interval=1.0):
    module, _ = _load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    clock = _Clock()
    monkeypatch.setattr(module, "time", clock)
    node = module._MediaStatusNode("robot", media, interval)
    # The shared rclpy stub has no logger; record warnings instead.
    warned = []
    node.get_logger = lambda: types.SimpleNamespace(warn=warned.append,
                                                   info=warned.append)
    return module, node, clock, warned


def test_the_card_declares_one_read_only_json_topic(monkeypatch):
    module, _ = _load_device(monkeypatch)
    plugin = module.MediaStatusPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda _: None), _Media())
    tool = plugin.get_tool()

    assert tool["name"] == "media_status"
    assert tool["type"] == "sensor"
    assert tool["multiInstance"] is False
    assert tool["topic_out"] == [{"topic": "/robot/media/status",
                                  "format": "data/json"}]
    assert tool["inputSchema"]["properties"]["action"]["enum"] == [
        "start", "stop", "info"]


def test_a_watched_wakeup_reports_its_age(monkeypatch):
    media = _Media("SLEEPED", "TIMEOUT_SLEEPED")
    _, node, clock, _ = _node(monkeypatch, media)

    first = node.read_now()
    assert first["reason_text"] == "超时自动休眠"
    # Nothing was observed yet, so there is no wakeup to give an age for.
    assert first["last_wakeup_age_s"] is None

    clock.now += 5
    assert node.read_now()["observed_for_s"] == 5.0

    media.work_status, media.reason = "WAKEUPED", "AUDIO_WAKEUPED"
    clock.now += 1
    woke = node.read_now()
    assert woke["awake"] is True
    assert woke["reason_text"] == "被语音唤醒"
    assert woke["last_wakeup_age_s"] == 0.0

    clock.now += 3
    assert node.read_now()["last_wakeup_age_s"] == 3.0


def test_an_outage_breaks_the_observation_and_restarts_it(monkeypatch):
    media = _Media("READY", "SYSTEM_LAUNCH")
    _, node, clock, warned = _node(monkeypatch, media)

    node.read_now()                       # first sample starts the observation
    clock.now += 10
    assert node.read_now()["observed_for_s"] == 10.0

    media.fail = True
    broken = node.read_now()
    assert broken["healthy"] is None
    assert broken["read_errors"] == 1
    assert broken["observed_for_s"] is None
    assert warned, "a failed read must leave a log line"

    clock.now += 30
    media.fail = False
    recovered = node.read_now()
    assert recovered["healthy"] is True
    assert recovered["read_errors"] == 0
    # The 30 s gap was not observed, so it is not claimed as continuity.
    assert recovered["observed_for_s"] == 0.0


def test_info_answers_live_values_without_waiting_for_the_topic(monkeypatch):
    module, _ = _load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    plugin = module.MediaStatusPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda _: None),
        _Media("SLEEPED", "TIMEOUT_SLEEPED", timeout_ms=90000))

    result = plugin.dispatch("info", {})

    assert result["state"] == "idle"          # not started in this test
    assert result["work_status_text"] == "已休眠"
    assert result["sleep_after_s"] == 90.0
    assert result["topic_out"] == [{"topic": "/robot/media/status",
                                    "format": "data/json"}]
    # The vocabulary travels with the card so nothing else has to hard-code it.
    assert result["interface"]["fault_reasons"] == ["ERROR_SLEEPED"]
    assert result["interface"]["timing_note"]
    assert plugin.dispatch("stop", {}) == {"state": "idle"}
    assert plugin.dispatch("nonsense", {}) is None


def test_an_out_of_range_poll_interval_is_rejected(monkeypatch):
    module, _ = _load_device(monkeypatch)
    executor = types.SimpleNamespace(add_node=lambda _: None)

    with pytest.raises(ValueError):
        module.MediaStatusPlugin({"poll_interval_s": 30.0}, "robot",
                                  executor, _Media())



def test_a_superseded_polling_thread_publishes_nothing(monkeypatch):
    """`stop` joins for one second, which is shorter than the longest allowed
    interval, so the thread from an earlier start can still be asleep when the
    next one begins. It must bow out instead of publishing on the same topic
    twice per interval — a doubled rate that the payload cannot reveal."""
    _, node, _, _ = _node(monkeypatch, _Media(), interval=5.0)
    node._running = True
    node._generation = 7

    node._loop(6)                     # the thread an earlier start handed out

    assert node.published == []


def test_a_stopped_card_leaves_no_thread_behind(monkeypatch):
    _, node, _, _ = _node(monkeypatch, _Media(), interval=0.2)

    node.start_polling()
    first = node._thread
    node.start_polling()              # already running: still one thread

    assert node._thread is first

    node.stop_polling()

    assert node.running is False
    assert node._thread is None
    assert not first.is_alive()


def test_the_spec_module_reaches_the_image():
    """`device.py` imports the spec at runtime, and this bundle copies modules
    into the image one by one instead of copying the directory. A spec that is
    on disk but not in the image passes every test here and then publishes
    nothing at all on the robot — the import fails inside the polling thread and
    the card never appears. The COPY line is part of the contract."""
    dockerfile = (Path(__file__).resolve().parents[1] / "Dockerfile").read_text(
        encoding="utf-8")

    assert "COPY media_status_spec.py /work/media_status_spec.py" in dockerfile

def test_a_missing_controller_is_not_the_same_as_a_failed_read():
    """Both report `healthy: null`, so a renderer showing only that cannot tell
    "ask again in a second" from "this will not change until the container is
    restarted". `controller_present` is the difference."""
    failed_read = spec.sample(received_ms=0, read_errors=3)
    missing = spec.sample(received_ms=0, read_errors=3, controller_present=False)

    assert failed_read["healthy"] is None and missing["healthy"] is None
    assert failed_read["controller_present"] is True
    assert missing["controller_present"] is False


def test_a_media_unit_that_never_initialised_still_reports(monkeypatch):
    """`main.py` registers this card without a controller, because the media
    unit failing to come up is the condition the card exists to show. It must
    then publish a sample rather than die on the way in: `healthy: null`,
    `controller_present: false`, and a `read_errors` that keeps climbing."""
    _, node, _, warned = _node(monkeypatch, None)

    first = node.read_now()
    second = node.read_now()

    assert first["healthy"] is None
    assert first["awake"] is None
    assert first["controller_present"] is False
    assert first["read_errors"] == 1
    assert second["read_errors"] == 2
    # The log line is a diagnosis, not "NoneType has no attribute ...".
    assert "MediaController did not initialise" in str(warned[0])


def test_start_answers_the_documented_sensor_shape(monkeypatch):
    """One `state` and nothing else, as `motion_state` does: the topic
    description belongs to `get_tool()` and to `info`, and a bundle-level
    consumer that validates the lifecycle reply rejects extra fields."""
    module, _ = _load_device(monkeypatch)
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]))
    plugin = module.MediaStatusPlugin(
        {}, "robot", types.SimpleNamespace(add_node=lambda _: None), _Media())

    assert plugin.dispatch("start", {}) == {"state": "running"}

    plugin.dispatch("stop", {})
