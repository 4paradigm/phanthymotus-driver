"""End-to-end quick setup: no engineering JSON or invented verification flags."""
import copy
import time

import pytest

from test_payload_mass import Sampler, Loco, raw_frame, run, wait_for
from payload_mass import PayloadMassPlugin
from payload_mass_sampling import MassError, check_frame
from payload_mass_quick import profile_from_home


class QuickLoco(Loco):
    def _do_teaching_action(self, action, args):
        result = super()._do_teaching_action(action, args)
        self.sampler.frame["q"][3] = -1.2
        return result


@pytest.fixture
def quick(tmp_path):
    sampler = Sampler()
    sampler.frame["q"] = [.1] * 21
    sampler.frame["workmode"] = 2
    loco = QuickLoco(sampler)
    card = PayloadMassPlugin({"setup_mode": "quick", "recording_id": 12,
                              "data_dir": str(tmp_path / "trials")}, "quick_test",
                             sampler=sampler, loco_plugin=loco, notify=lambda *a: None)
    yield card, sampler, loco
    card.stop()


def set_mass(sampler, kg):
    sampler.frame["tau"] = raw_frame(kg)["tau"]


def test_quick_setup_single_reference_dynamic_unknown_and_home(quick):
    card, sampler, loco = quick
    assert run(card, "prepare")["state"] == "ready"
    assert card._session["p"]["pose_q"][3] == -1.2  # learned, not hand-entered
    baseline = copy.deepcopy(card._session["baseline"])
    assert run(card, "weigh")["state"] == "measuring"
    set_mass(sampler, .2)
    before = wait_for(card, lambda r: r.get("sampling_stable") and r.get("trial_file"))
    assert before["mass_kg"] is None
    calibrated = card.dispatch("calibrate", {"known_mass_kg": .2})
    assert calibrated["state"] == "calibrated"
    assert calibrated["validation_status"] == "not_validated"
    reference = wait_for(card, lambda r: r.get("valid") and r.get("mass_kg") == .2)
    assert reference["experimental"] and reference["remaining_s"] > 290
    set_mass(sampler, .35)
    unknown = wait_for(card, lambda r: r.get("stable") and r.get("mass_kg") == .35)
    assert unknown["quality"] == "experimental_single_reference"
    assert card._session["baseline"] == baseline
    set_mass(sampler, 0)
    wait_for(card, lambda r: r.get("stable") and r.get("mass_kg") == 0)
    assert run(card, "stop")["home_confirmed"]
    assert loco._payload_owner is None
    assert [a[1]["recording_id"] for a in loco.commands if a[0] == "play_recording"] == [12]


def test_only_recording_id_is_needed_and_configuration_is_atomic(quick):
    card, _, loco = quick
    card._recording_id = None
    assert run(card, "prepare")["error_code"] == "NOT_CONFIGURED"
    assert not loco.commands
    assert card.dispatch("config", {"recording_id": 123})["state"] == "configured"
    result = card.dispatch("config", {"recording_id": 456, "hold_duration_s": -1})
    assert result["error_code"] == "BAD_PARAMETER" and card._recording_id == 123
    assert run(card, "prepare")["state"] == "ready"


def test_calibration_requires_stable_reference_not_empty_or_truth_in_weigh(quick):
    card, sampler, _ = quick
    assert card.dispatch("calibrate", {"known_mass_kg": .2})["error_code"] == "NOT_MEASURING"
    run(card, "prepare")
    # Hidden form truth is ignored outside calibration; never changes tare.
    assert run(card, "weigh", known_mass_kg=99)["state"] == "measuring"
    wait_for(card, lambda r: r.get("sampling_stable"))
    assert card.dispatch("calibrate", {"known_mass_kg": .2})["error_code"] == "UNIDENTIFIABLE"
    set_mass(sampler, .2)
    wait_for(card, lambda r: r.get("sampling_stable") and r.get("trial_file"))
    assert card.dispatch("calibrate", {"known_mass_kg": .6})["error_code"] == "BAD_PARAMETER"


def test_live_five_minute_deadline_clears_reading_without_restarting_recording(quick):
    card, sampler, loco = quick
    run(card, "prepare"); run(card, "weigh")
    card._playback_started = time.monotonic() - 301
    result = wait_for(card, lambda r: r.get("error_code") == "RECORDING_ENDED")
    assert result["mass_kg"] is None
    assert loco._payload_owner is card
    assert sum(c[0] == "play_recording" for c in loco.commands) == 1
    assert run(card, "stop")["home_confirmed"]


def test_quick_mode_retains_undocumented_status_and_blocks_documented_fault(quick):
    card, sampler, _ = quick
    sampler.frame["errors"][4] = 1
    run(card, "prepare"); run(card, "weigh")
    result = wait_for(card, lambda r: r.get("unknown_status_codes") == [1])
    assert result["validation_status"] == "not_validated"
    sampler.frame["errors"][4] = 2
    assert wait_for(card, lambda r: r.get("error_code") == "MOTOR_FAULT")["mass_kg"] is None


def test_right_arm_profile_selects_actual_motor_ids():
    p = profile_from_home(raw_frame(), "right", 300)
    assert p["motor_ids"] == [10, 11, 12, 13]
    assert "mapping_verified" not in p and "feedback_verified" not in p
    assert p["max_mass_kg"] == .5


def test_pretending_to_hold_without_bending_does_not_prepare(quick):
    card, sampler, loco = quick
    def no_bend(action, args):
        sampler.frame["workmode"] = 23
        return {"command_sent": True, "confirmed_started": True, "workmode": 23}
    loco._do_teaching_action = no_bend
    original = card._load_quick
    def short_timeout():
        loaded = original(); loaded[2]["timeout_s"] = .1
        return loaded
    card._load_quick = short_timeout
    assert run(card, "prepare")["error_code"] == "POSE_TIMEOUT"
