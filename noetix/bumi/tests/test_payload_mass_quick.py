"""End-to-end quick setup: no engineering JSON or invented verification flags."""
import copy
import math
import threading
import time
from types import SimpleNamespace

import pytest

from test_payload_mass import Sampler, Loco, raw_frame, run, wait_for, configuration
from payload_mass import PayloadMassPlugin
from payload_mass_sampling import MassError, SharedSampler, check_frame, pose_span_violation
from payload_mass_quick import profile_from_home, fit_reference
from payload_mass_estimator import summarize, validate_profile
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


@pytest.mark.parametrize("moving_arm", [False, True])
def test_three_second_capture_with_continuous_ankle_motion(monkeypatch, moving_arm):
    import payload_mass_sampling as sampling
    clock = [0.]
    monkeypatch.setattr(sampling, "time", SimpleNamespace(monotonic=lambda: clock[0], time=time.time))
    sampler = SharedSampler()
    home = [0.] * 21
    frame = raw_frame()
    frame["q"][3] = -.6
    p = profile_from_home(frame, "left", 300)
    p.update(settle_timeout_s=1, min_samples=15)
    ended = threading.Event()
    def feed():
        seq = 0
        while not ended.is_set():
            seq += 1
            now = seq * .1
            current = copy.deepcopy(frame)
            current["source_seq"] = seq
            # 0.1894-rad peak-to-peak ankle excursion, period 4 seconds.
            # Its true velocity stays below the retained 0.2-rad/s limit.
            current["q"][9] = .0947 * math.sin(now*math.pi/2)
            current["dq"][9] = .0947 * math.pi/2 * math.cos(now*math.pi/2)
            current["dq"][1] = .1343
            if moving_arm:
                current["q"][1] = .025*now
            clock[0] = now
            sampler.push(current)
            ended.wait(.005)
    thread = threading.Thread(target=feed)
    thread.start()
    try:
        if moving_arm:
            with pytest.raises(MassError) as err:
                sampler.collect(3, p, threading.Event(), learn_pose_from=home)
            assert err.value.code == "UNSTABLE"
            assert "joint angle span too high: motor_id=1" in str(err.value)
        else:
            frames = sampler.collect(3, p, threading.Event(), learn_pose_from=home)
            assert frames[-1]["host_monotonic_s"] - frames[0]["host_monotonic_s"] >= 3
            assert max(f["q"][9] for f in frames) - min(f["q"][9] for f in frames) > .15
            assert pose_span_violation(frames, p) is None
    finally:
        ended.set()
        thread.join(1)


@pytest.mark.parametrize("failure, expected", [
    ("velocity", "joint velocity: motor_id=6, dq=0.2100, limit=0.2000"),
    ("gyro", "body gyro: axis=2, gyro=0.1600, limit=0.1500"),
    ("bend", "elbow displacement below bend threshold: motor_id=3"),
    ("angle", "joint angle span too high: motor_id=3"),
    ("torque", "torque spread exceeds stable-window limit: motor_id=0"),
])
def test_stability_timeout_retains_actual_rejection_and_threshold(failure, expected):
    sampler = SharedSampler()
    home = [0.] * 21
    frame = raw_frame()
    frame["q"][3] = -.6
    p = profile_from_home(frame, "left", 300)
    p.update(min_samples=2, settle_timeout_s=.08, max_receive_gap_s=.05, max_torque_span=.05)
    done = threading.Event()
    def feed():
        seq = 0
        while not done.is_set():
            seq += 1
            current = copy.deepcopy(frame)
            current["source_seq"] = seq
            if failure == "velocity":
                current["dq"][6] = .21
            elif failure == "gyro":
                current["gyro"][2] = .16
            elif failure == "bend":
                current["q"][3] = .1
            elif failure == "angle":
                current["q"][3] -= .04 * (seq % 2)
            elif failure == "torque":
                current["tau"][0] += .1 * (seq % 2)
            sampler.push(current)
            done.wait(.005)
    thread = threading.Thread(target=feed)
    thread.start()
    try:
        with pytest.raises(MassError) as err:
            sampler.collect(.08, p, threading.Event(), learn_pose_from=home)
        assert err.value.code == "UNSTABLE"
        assert expected in str(err.value)
        assert "best_window_s=" in str(err.value) and "required_s=0.080" in str(err.value)
    finally:
        done.set()
        thread.join(1)


@pytest.mark.parametrize("arm, elbow", [("left", 3), ("right", 13)])
@pytest.mark.parametrize("reported_velocity", [0, .1343])
def test_gradual_bend_learns_final_stable_pose(tmp_path, arm, elbow, reported_velocity):
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
            frame.update(source_seq=seq, dq=[0.] * 21)
            # Test both unreported movement and the hardware-reported value:
            # actual angle movement must still determine the settled pose.
            frame["dq"][1 if arm == "left" else 11] = reported_velocity
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


@pytest.mark.parametrize("reported_velocity", [0, .1343])
@pytest.mark.parametrize("leg_shift", [0, .1885])
def test_quick_setup_single_reference_dynamic_unknown_and_home(quick, reported_velocity, leg_shift):
    card, sampler, loco = quick
    sampler.frame["dq"][1] = reported_velocity
    assert run(card, "prepare")["state"] == "ready"
    assert card._session["p"]["pose_q"][3] == -1.2  # learned, not hand-entered
    baseline = copy.deepcopy(card._session["baseline"])
    assert run(card, "weigh")["state"] == "measuring"
    sampler.frame["q"][9] -= leg_shift
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


@pytest.mark.parametrize("ankle", [9, 19])
def test_leg_stance_does_not_invalidate_arm_scale_or_change_tare(ankle):
    home = raw_frame()
    home["q"][ankle] = .0349
    p = profile_from_home(home, "left", 300)
    assert ankle not in p["fixed_pose_motor_ids"]
    empty = [raw_frame(0, n) for n in range(1, 33)]
    for f in empty:
        f["q"][ankle] = .0349
    baseline = {"id": "stance", "profile_fingerprint": p["fingerprint"], "created_s": time.time(),
                "summary": summarize(empty, p)}
    original = copy.deepcopy(baseline)
    model = fit_reference([raw_frame(.2, n) for n in range(1, 33)], baseline, p, .2)
    frames = [raw_frame(.35, n) for n in range(1, 33)]
    for f in frames:
        f["q"][ankle] = -.1535
    # Leg angles do not define the quick model's geometry, even when they
    # change inside the window. IMU, speed and arm/waist checks still apply.
    transient = copy.deepcopy(frames)
    transient[0]["q"][ankle] = .0349
    changing = evaluate_window(transient, baseline, model, p, 3, time.time())
    assert changing["stable"] and changing["valid"] and changing["mass_kg"] == .35
    settled = evaluate_window(frames, baseline, model, p, 3, time.time())
    assert settled["mass_kg"] == .35 and settled["stable"] and settled["valid"]
    assert baseline == original


@pytest.mark.parametrize("change, code", [
    (lambda f: f["q"].__setitem__(3, .2), "POSE_MISMATCH"),
    (lambda f: f["q"].__setitem__(20, .2), "POSE_MISMATCH"),
    (lambda f: f.__setitem__("quaternion_xyzw", [0, 0, .09983341664682815, .9950041652780258]), "BODY_MISMATCH"),
    (lambda f: f["errors"].__setitem__(9, 2), "MOTOR_FAULT"),
    (lambda f: f["dq"].__setitem__(9, .21), "UNSTABLE"),
    (lambda f: f.__setitem__("source_seq", None), "FRESHNESS_UNVERIFIED"),
])
def test_leg_pose_scope_retains_upper_body_fault_speed_and_source_checks(change, code):
    p = profile_from_home(raw_frame(), "left", 300)
    frame = raw_frame()
    frame["q"][9] = -.1885
    change(frame)
    with pytest.raises(MassError) as err:
        check_frame(frame, p)
    assert err.value.code == code


def test_engineering_profile_still_checks_leg_pose():
    p = validate_profile("weighing", configuration()["profile"], "left", loading=True)
    frame = raw_frame()
    frame["q"][9] = -.1885
    with pytest.raises(MassError) as err:
        check_frame(frame, p)
    assert err.value.code == "POSE_MISMATCH" and "motor_id=9" in str(err.value)
    assert pose_span_violation([raw_frame(), frame], p)[0] == 9


@pytest.mark.parametrize("changing_pose", [False, True])
@pytest.mark.parametrize("motor", [1, 11, 20])
def test_reported_velocity_tolerance_keeps_actual_pose_window_check(changing_pose, motor):
    p = profile_from_home(raw_frame(), "left", 300)
    empty = [raw_frame(0, n) for n in range(1, 33)]
    baseline = {"id": "velocity", "profile_fingerprint": p["fingerprint"], "created_s": time.time(),
                "summary": summarize(empty, p)}
    model = fit_reference([raw_frame(.2, n) for n in range(1, 33)], baseline, p, .2)
    frames = [raw_frame(.35, n) for n in range(1, 33)]
    for n, frame in enumerate(frames):
        frame["dq"][1] = .1343
        if changing_pose:
            # All frames stay inside the fixed +/-0.15 target tolerance; only
            # the tighter complete-window span detects this actual movement.
            half_span = .04 if motor in p["motor_ids"] else .1
            frame["q"][motor] = -half_span + 2*half_span*n/(len(frames)-1)
    result = evaluate_window(frames, baseline, model, p, 3, time.time())
    assert result["mass_kg"] == .35
    assert result["stable"] is (not changing_pose)
    assert result["valid"] is (not changing_pose)
    assert result["sampling_stable"] is (not changing_pose)


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
