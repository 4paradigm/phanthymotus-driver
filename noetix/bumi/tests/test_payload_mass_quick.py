"""End-to-end quick setup: no engineering JSON or invented verification flags."""
import copy
import threading
import time
from types import SimpleNamespace

import pytest

from test_payload_mass import Sampler, Loco, raw_frame, run, wait_for
from payload_mass import PayloadMassPlugin
from payload_mass_sampling import MassError, SharedSampler, check_frame
from payload_mass_quick import profile_from_home, fit_reference
from payload_mass_estimator import summarize
from payload_mass_live import evaluate_window


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


@pytest.mark.parametrize("arm, elbow", [("left", 3), ("right", 13)])
def test_gradual_bend_with_zero_reported_velocity_learns_final_stable_pose(tmp_path, arm, elbow):
    sampler = SharedSampler()
    current = raw_frame()
    current["q"] = [0.] * 21
    current["workmode"] = 2
    stopped = threading.Event()
    bend_at = [None]
    class GradualLoco(Loco):
        def _do_teaching_action(self, action, args):
            self.commands.append((action, args))
            current["workmode"] = 23
            bend_at[0] = time.monotonic()
            return {"command_sent": True, "confirmed_started": True, "workmode": 23}
    loco = GradualLoco(SimpleNamespace(frame=current))
    def feed():
        seq = 0
        while not stopped.is_set():
            seq += 1
            frame = copy.deepcopy(current)
            if bend_at[0] is not None and current["workmode"] == 23:
                elapsed = time.monotonic() - bend_at[0]
                frame["q"][elbow] = -.3 if elapsed < .2 else -.7 if elapsed < .4 else -1.2
            # Deliberately zero dq: stationarity must also use actual q movement.
            frame.update(source_seq=seq, dq=[0.] * 21)
            sampler.push(frame)
            stopped.wait(.015)
    thread = threading.Thread(target=feed)
    thread.start()
    card = PayloadMassPlugin({"setup_mode": "quick", "recording_id": 12, "arm": arm,
                              "data_dir": str(tmp_path)}, "gradual_bend", sampler=sampler,
                             loco_plugin=loco, notify=lambda *a: None)
    try:
        deadline = time.monotonic() + 1
        while sampler.diagnostics()["received_count"] < 2 and time.monotonic() < deadline:
            time.sleep(.01)
        assert card.dispatch("prepare", {})["state"] == "running"
        card._job["thread"].join(6)
        assert not card._job["thread"].is_alive()
        assert card.info()["state"] == "ready"
        assert card._session["p"]["pose_q"][elbow] == -1.2
        assert card._session["baseline"]["summary"]["pose_q"][elbow] == -1.2
        assert len([c for c in loco.commands if c[0] == "play_recording"]) == 1
        assert card.dispatch("weigh", {})["state"] == "measuring"
        # Once prepared, the learned target remains fixed during measurement.
        current["q"][elbow] = -.3
        bend_at[0] = None
        result = wait_for(card, lambda r: r.get("error_code") == "POSE_MISMATCH")
        assert result["mass_kg"] is None and f"motor_id={elbow}" in result["message"]
    finally:
        card.stop()
        stopped.set()
        thread.join(1)


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
    assert p["accept_host_only_freshness"] is False


def getter_values(frame):
    """Actual getter shape: motor/IMU values, with no source heartbeat."""
    motors = [SimpleNamespace(motor_id=i, pos=frame["q"][i], vel=frame["dq"][i],
                              tau=frame["tau"][i], temperature=frame["temperature"][i],
                              error=frame["errors"][i]) for i in range(21)]
    imu = SimpleNamespace(ori=frame["quaternion_xyzw"], angular_vel=frame["gyro"])
    return motors, imu, frame["workmode"]


@pytest.mark.parametrize("moving", [False, True])
@pytest.mark.parametrize("legacy_host_opt_in", [False, True])
def test_frozen_real_getter_cannot_produce_quick_kg(monkeypatch, moving, legacy_host_opt_in):
    import payload_mass_sampling as sampling
    clock = [10.0]
    monkeypatch.setattr(sampling, "time", SimpleNamespace(monotonic=lambda: clock[0], time=time.time))
    sampler = SharedSampler()
    frozen = raw_frame(.2)
    if moving:
        frozen["dq"][0] = .2
    values = getter_values(frozen)
    for _ in range(41):
        sampler.accept_getter(*values)
        clock[0] += .1
    d = sampler.diagnostics()
    assert d["freshness_status"] == "host_receipt_only" and d["host_age_s"] < .11
    frames = sampler.read_since(0, 0)
    assert frames[-1]["host_monotonic_s"] - frames[0]["host_monotonic_s"] > 3
    assert all(f["source_seq"] is None and f["source_timestamp"] is None for f in frames)
    p = profile_from_home(d["latest"], "left", 300)
    p["accept_host_only_freshness"] = legacy_host_opt_in
    empty = [raw_frame(0, n) for n in range(1, 33)]
    baseline = {"id": "freshness", "profile_fingerprint": p["fingerprint"],
                "created_s": time.time(), "summary": summarize(empty, p)}
    model = fit_reference([raw_frame(.2, n) for n in range(1, 33)], baseline, p, .2)
    # Even a calibrated scale must not publish provisional or stable kg from
    # repeated getter receipts; moving frames must not bypass this check either.
    with pytest.raises(MassError) as err:
        evaluate_window(frames, baseline, model, p, 3, time.time())
    assert err.value.code == "FRESHNESS_UNVERIFIED"


def test_quick_prepare_with_real_getter_rejects_before_motor_command(tmp_path):
    sampler = SharedSampler()
    home = raw_frame(); home["workmode"] = 2
    for _ in range(20):
        sampler.accept_getter(*getter_values(home))
    loco = Loco(SimpleNamespace(frame=home))
    card = PayloadMassPlugin({"setup_mode": "quick", "recording_id": 12,
                              "data_dir": str(tmp_path)}, "no_heartbeat",
                             sampler=sampler, loco_plugin=loco, notify=lambda *a: None)
    try:
        result = card.dispatch("prepare", {})
        assert result["error_code"] == "FRESHNESS_UNVERIFIED"
        assert result["mass_kg"] is None and not result["valid"] and not result["stable"]
        assert not loco.commands and loco._payload_owner is None
        assert card._session is None
    finally:
        card.stop()


@pytest.mark.parametrize("learning", [False, True])
def test_quick_actual_collect_rejects_repeated_getter_receipts(learning):
    sampler = SharedSampler()
    values = getter_values(raw_frame())
    sampler.accept_getter(*values)
    p = profile_from_home(sampler.diagnostics()["latest"], "left", 300)
    p.update(min_samples=2, settle_timeout_s=.3)
    done = threading.Event()
    def feed():
        while not done.wait(.005):
            sampler.accept_getter(*values)
    thread = threading.Thread(target=feed)
    thread.start()
    try:
        with pytest.raises(MassError) as err:
            sampler.collect(.03, p, threading.Event(),
                            learn_pose_from=[0.] * 21 if learning else None)
        assert err.value.code == "FRESHNESS_UNVERIFIED"
    finally:
        done.set()
        thread.join(1)


@pytest.mark.parametrize("heartbeat", ["source_seq", "source_timestamp"])
def test_quick_verified_source_can_be_constant_but_duplicates_cannot_refresh(monkeypatch, heartbeat):
    import payload_mass_sampling as sampling
    clock = [10.0]
    monkeypatch.setattr(sampling, "time", SimpleNamespace(monotonic=lambda: clock[0], time=time.time))
    sampler = SharedSampler()
    for n in range(41):
        frame = raw_frame(.2)
        frame.update(source_seq=None, source_timestamp=None)
        frame[heartbeat] = n
        assert sampler.push(frame)
        clock[0] += .1
    p = profile_from_home(frame, "left", 300)
    baseline = {"id": "verified", "profile_fingerprint": p["fingerprint"], "created_s": time.time(),
                "summary": summarize([raw_frame(0, n) for n in range(1, 33)], p)}
    model = fit_reference([raw_frame(.2, n) for n in range(1, 33)], baseline, p, .2)
    result = evaluate_window(sampler.read_since(0, 0), baseline, model, p, 3, time.time())
    assert result["mass_kg"] == .2 and result["valid"] and result["stable"]
    cursor = sampler.diagnostics()["received_count"]
    for _ in range(40):
        clock[0] += .1
        assert not sampler.push(frame)
    assert sampler.read_since(cursor, 0) == []
    assert sampler.diagnostics()["host_age_s"] > 3


def test_exact_500g_bound_accepts_roundoff_but_rejects_actual_excess():
    p = profile_from_home(raw_frame(), "left", 300)
    empty = [raw_frame(0, n) for n in range(1, 33)]
    baseline = {"id": "edge", "profile_fingerprint": p["fingerprint"], "created_s": time.time(),
                "summary": summarize(empty, p)}
    model = fit_reference([raw_frame(.2, n) for n in range(1, 33)], baseline, p, .2)
    accepted = evaluate_window([raw_frame(.5, n) for n in range(1, 33)], baseline, model, p, 3, time.time())
    assert accepted["mass_kg"] == .5 and accepted["stable"]
    rejected = evaluate_window([raw_frame(.501, n) for n in range(1, 33)], baseline, model, p, 3, time.time())
    assert rejected["error_code"] == "OUT_OF_RANGE" and rejected["mass_kg"] is None


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
